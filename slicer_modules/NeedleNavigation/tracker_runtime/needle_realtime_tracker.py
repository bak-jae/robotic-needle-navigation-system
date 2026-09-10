# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

"""SmallUNet heatmap inference and encoder-constrained tip selection.

This module contains no ROS, Slicer, or motor-control code.  Keeping the tracking
math independent makes it testable with recorded frames and allows the model
weights to be replaced without changing the ROS/Slicer integration.

Model compatibility
-------------------
``NeedleRealtimeTracker`` constructs ``SmallUNet(base_channels=16)`` and loads a
PyTorch *state_dict*.  A replacement ``.pt`` file must therefore have exactly the
same layer names and tensor shapes.  The model file is deliberately excluded from
the public repository; place an authorized checkpoint at the path passed by the
launcher.  This is not a generic loader for unrelated architectures.

Coordinate contract
-------------------
Inference and all ``TrackingResult`` points use a canonical 512x512 image.  The
ROS adapter converts native encoder pixels into this space.  ``draw_overlay`` maps
the points back to native image dimensions so visualization never stretches the
ultrasound image.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Optional

import cv2
import numpy as np
import torch
import torch.nn as nn


CANONICAL_SIZE = 512


@dataclass
class EncoderState:
    """Encoder-predicted tip and shaft endpoints in canonical pixels."""
    tip_x: Optional[float] = None
    tip_y: Optional[float] = None
    line_x1: Optional[float] = None
    line_y1: Optional[float] = None
    line_x2: Optional[float] = None
    line_y2: Optional[float] = None


@dataclass
class TrackingResult:
    """One frame's candidate-selection and temporal-filtering result."""
    detected: bool
    final_x: Optional[float] = None
    final_y: Optional[float] = None
    pre_x: Optional[float] = None
    pre_y: Optional[float] = None
    kalman_x: Optional[float] = None
    kalman_y: Optional[float] = None
    encoder_x: Optional[float] = None
    encoder_y: Optional[float] = None
    used_kalman: bool = False
    reason: str = ""
    visible_run_count: int = 0
    cnn_value: Optional[float] = None
    final_score: Optional[float] = None
    displacement_error_px: Optional[float] = None
    trajectory_score: Optional[float] = None


class ConvBlock(nn.Module):
    """Two convolution/batch-normalization/ReLU stages used at each U-Net level."""
    def __init__(self, in_channels: int, out_channels: int):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, 3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.layers(x)


class SmallUNet(nn.Module):
    """Compact one-channel U-Net that produces a one-channel tip heatmap."""
    def __init__(self, base_channels: int = 16):
        super().__init__()
        self.enc1 = ConvBlock(1, base_channels)
        self.enc2 = ConvBlock(base_channels, base_channels * 2)
        self.enc3 = ConvBlock(base_channels * 2, base_channels * 4)
        self.pool = nn.MaxPool2d(2)
        self.up2 = nn.ConvTranspose2d(base_channels * 4, base_channels * 2, 2, stride=2)
        self.dec2 = ConvBlock(base_channels * 4, base_channels * 2)
        self.up1 = nn.ConvTranspose2d(base_channels * 2, base_channels, 2, stride=2)
        self.dec1 = ConvBlock(base_channels * 2, base_channels)
        self.out = nn.Conv2d(base_channels, 1, 1)

    def forward(self, x):
        enc1 = self.enc1(x)
        enc2 = self.enc2(self.pool(enc1))
        bottleneck = self.enc3(self.pool(enc2))
        dec2 = self.up2(bottleneck)
        dec2 = self.dec2(torch.cat([dec2, enc2], dim=1))
        dec1 = self.up1(dec2)
        dec1 = self.dec1(torch.cat([dec1, enc1], dim=1))
        return torch.sigmoid(self.out(dec1))


class PointKalmanFilter:
    """Constant-velocity 2D filter used only when a candidate jump is large."""
    def __init__(self, process_noise: float = 1.0, measurement_noise: float = 16.0):
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.state = None
        self.covariance = None

    def reset(self):
        self.state = None
        self.covariance = None

    def update(self, x_coord: float, y_coord: float):
        measurement = np.array([[x_coord], [y_coord]], dtype=np.float32)
        if self.state is None:
            self.state = np.array([[x_coord], [y_coord], [0.0], [0.0]], dtype=np.float32)
            self.covariance = np.eye(4, dtype=np.float32) * 10.0
            return float(x_coord), float(y_coord)

        transition = np.array(
            [[1.0, 0.0, 1.0, 0.0], [0.0, 1.0, 0.0, 1.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        observation = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float32)
        process_covariance = np.eye(4, dtype=np.float32) * self.process_noise
        measurement_covariance = np.eye(2, dtype=np.float32) * self.measurement_noise
        predicted_state = transition @ self.state
        predicted_covariance = transition @ self.covariance @ transition.T + process_covariance
        innovation = measurement - observation @ predicted_state
        innovation_covariance = observation @ predicted_covariance @ observation.T + measurement_covariance
        kalman_gain = predicted_covariance @ observation.T @ np.linalg.inv(innovation_covariance)
        self.state = predicted_state + kalman_gain @ innovation
        self.covariance = (np.eye(4, dtype=np.float32) - kalman_gain @ observation) @ predicted_covariance
        return float(self.state[0, 0]), float(self.state[1, 0])


def normalize_map(values: np.ndarray) -> np.ndarray:
    """Normalize a score image to [0, 1], guarding constant-valued arrays."""
    values = values.astype(np.float32)
    min_value = float(values.min())
    max_value = float(values.max())
    if max_value - min_value < 1e-8:
        return np.zeros_like(values, dtype=np.float32)
    return (values - min_value) / (max_value - min_value)


def valid_encoder_tip(encoder_tip):
    """Return whether an encoder prior lies inside the canonical image."""
    if encoder_tip is None:
        return False
    x_coord, y_coord = encoder_tip
    return 0 <= float(x_coord) < CANONICAL_SIZE and 0 <= float(y_coord) < CANONICAL_SIZE


def point_in_image(x_coord, y_coord, margin_px=0.0):
    return -margin_px <= x_coord < CANONICAL_SIZE + margin_px and -margin_px <= y_coord < CANONICAL_SIZE + margin_px


def segment_intersects_image(x1, y1, x2, y2, margin_px=0.0):
    """Test whether the predicted shaft segment intersects the image rectangle."""
    if point_in_image(x1, y1, margin_px) or point_in_image(x2, y2, margin_px):
        return True
    left, right = -margin_px, CANONICAL_SIZE + margin_px
    top, bottom = -margin_px, CANONICAL_SIZE + margin_px
    if max(x1, x2) < left or min(x1, x2) >= right or max(y1, y2) < top or min(y1, y2) >= bottom:
        return False

    def ccw(ax, ay, bx, by, cx, cy):
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    def intersects(ax, ay, bx, by, cx, cy, dx, dy):
        return ccw(ax, ay, cx, cy, dx, dy) != ccw(bx, by, cx, cy, dx, dy) and ccw(ax, ay, bx, by, cx, cy) != ccw(ax, ay, bx, by, dx, dy)

    edges = [(left, top, right, top), (right, top, right, bottom), (right, bottom, left, bottom), (left, bottom, left, top)]
    return any(intersects(x1, y1, x2, y2, *edge) for edge in edges)


def encoder_needle_in_image(encoder_state: EncoderState, margin_px=0.0):
    """Use the shaft when available, otherwise fall back to the tip point."""
    values = [encoder_state.line_x1, encoder_state.line_y1, encoder_state.line_x2, encoder_state.line_y2]
    if all(value is not None for value in values):
        return segment_intersects_image(*(float(value) for value in values), margin_px)
    if encoder_state.tip_x is None or encoder_state.tip_y is None:
        return False
    return point_in_image(float(encoder_state.tip_x), float(encoder_state.tip_y), margin_px)


def circular_mask(center_x_512, center_y_512, image_size, radius_px_512):
    """Build the encoder-centered candidate ROI at the model heatmap resolution."""
    scale = image_size / CANONICAL_SIZE
    cx = center_x_512 * scale
    cy = center_y_512 * scale
    radius = radius_px_512 * scale
    yy, xx = np.ogrid[:image_size, :image_size]
    return ((xx - cx) ** 2 + (yy - cy) ** 2) <= radius * radius


def gaussian_tip_prior(center_x_512, center_y_512, image_size, sigma_px_512):
    """Build a soft encoder prior at the model heatmap resolution."""
    scale = image_size / CANONICAL_SIZE
    cx = center_x_512 * scale
    cy = center_y_512 * scale
    sigma = sigma_px_512 * scale
    yy, xx = np.mgrid[:image_size, :image_size]
    return np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2.0 * sigma * sigma)).astype(np.float32)


def shift_array(values, dx, dy):
    """Translate an array without wraparound, filling newly exposed pixels with zero."""
    height, width = values.shape
    shifted = np.zeros_like(values, dtype=np.float32)
    src_x0 = max(0, -dx)
    src_x1 = min(width, width - dx)
    src_y0 = max(0, -dy)
    src_y1 = min(height, height - dy)
    dst_x0 = max(0, dx)
    dst_x1 = min(width, width + dx)
    dst_y0 = max(0, dy)
    dst_y1 = min(height, height + dy)
    if src_x0 < src_x1 and src_y0 < src_y1:
        shifted[dst_y0:dst_y1, dst_x0:dst_x1] = values[src_y0:src_y1, src_x0:src_x1]
    return shifted


def encoder_directional_motion_map(current_array, history_items, current_encoder_tip, min_displacement_px_512):
    """Estimate image evidence that moves consistently with encoder displacement."""
    if not history_items or current_encoder_tip is None:
        return np.zeros_like(current_array, dtype=np.float32)
    current_encoder_x, current_encoder_y = current_encoder_tip
    scale = current_array.shape[1] / CANONICAL_SIZE
    directional_maps = []
    for item in history_items:
        previous_encoder_tip = item.get("encoder_tip")
        previous_array = item.get("image")
        if previous_encoder_tip is None or previous_array is None:
            continue
        previous_encoder_x, previous_encoder_y = previous_encoder_tip
        displacement_x_512 = current_encoder_x - previous_encoder_x
        displacement_y_512 = current_encoder_y - previous_encoder_y
        if math.hypot(displacement_x_512, displacement_y_512) < min_displacement_px_512:
            continue
        displacement_x = int(round(displacement_x_512 * scale))
        displacement_y = int(round(displacement_y_512 * scale))
        if displacement_x == 0 and displacement_y == 0:
            continue
        shifted_previous = shift_array(previous_array, displacement_x, displacement_y)
        same_position_change = normalize_map(np.abs(current_array - previous_array))
        moved_brightness_agreement = normalize_map(current_array * shifted_previous)
        directional_maps.append(same_position_change * moved_brightness_agreement)
    if not directional_maps:
        return np.zeros_like(current_array, dtype=np.float32)
    return normalize_map(np.mean(directional_maps, axis=0))


def topk_points(score_map, mask, topk, nms_radius):
    """Extract spatially separated score maxima using simple non-max suppression."""
    work_map = np.where(mask, score_map, -np.inf).copy()
    points = []
    height, width = work_map.shape
    for _ in range(topk):
        flat_index = int(np.argmax(work_map))
        y_coord, x_coord = np.unravel_index(flat_index, work_map.shape)
        value = float(work_map[y_coord, x_coord])
        if not np.isfinite(value):
            break
        points.append((x_coord, y_coord, value))
        x_min = max(0, x_coord - nms_radius)
        x_max = min(width, x_coord + nms_radius + 1)
        y_min = max(0, y_coord - nms_radius)
        y_max = min(height, y_coord + nms_radius + 1)
        work_map[y_min:y_max, x_min:x_max] = -np.inf
    return points


def expected_position_from_encoder_history(history, current_encoder_tip):
    """Translate past detected tips by the matching encoder displacement."""
    expected_positions = []
    current_x, current_y = current_encoder_tip
    for item in history:
        if item["encoder_tip"] is None:
            continue
        previous_encoder_x, previous_encoder_y = item["encoder_tip"]
        expected_positions.append((item["final_x"] + current_x - previous_encoder_x, item["final_y"] + current_y - previous_encoder_y))
    if not expected_positions:
        return None
    return float(np.median([p[0] for p in expected_positions])), float(np.median([p[1] for p in expected_positions]))


def distance_to_trajectory_line(x_coord, y_coord, trajectory):
    dx = x_coord - trajectory["point_x"]
    dy = y_coord - trajectory["point_y"]
    return abs(dx * trajectory["dir_y"] - dy * trajectory["dir_x"])


def trajectory_prior_from_history(history, current_encoder_tip, args):
    """Fit a recent tip trajectory only when tip and encoder motion agree."""
    valid_items = [item for item in history if item.get("encoder_tip") is not None]
    if len(valid_items) < args.trajectory_min_history or current_encoder_tip is None:
        return None
    recent_items = valid_items[-args.trajectory_min_history :]
    first_item = recent_items[0]
    last_item = recent_items[-1]
    tip_dx = float(last_item["final_x"] - first_item["final_x"])
    tip_dy = float(last_item["final_y"] - first_item["final_y"])
    previous_encoder_x, previous_encoder_y = last_item["encoder_tip"]
    encoder_dx = float(current_encoder_tip[0] - previous_encoder_x)
    encoder_dy = float(current_encoder_tip[1] - previous_encoder_y)
    tip_norm = math.hypot(tip_dx, tip_dy)
    encoder_norm = math.hypot(encoder_dx, encoder_dy)
    if tip_norm < args.trajectory_min_tip_motion_px or encoder_norm < args.trajectory_min_encoder_motion_px:
        return None
    direction_cosine = (tip_dx * encoder_dx + tip_dy * encoder_dy) / (tip_norm * encoder_norm)
    if direction_cosine < args.trajectory_cos_threshold:
        return None
    points = np.array([[float(item["final_x"]), float(item["final_y"])] for item in recent_items], dtype=np.float32)
    center = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    direction = vh[0]
    return {"point_x": float(center[0]), "point_y": float(center[1]), "dir_x": float(direction[0]), "dir_y": float(direction[1])}


class NeedleRealtimeTracker:
    """Stateful per-frame detector combining CNN, encoder priors, and filtering."""
    def __init__(
        self,
        model_path: str | Path,
        device: str = "auto",
        image_size: int = 128,
        base_channels: int = 16,
        radius_px: float = 100.0,
        topk: int = 10,
        nms_radius: int = 4,
        motion_window: int = 5,
        cnn_weight: float = 1.0,
        motion_weight: float = 0.0,
        tip_weight: float = 0.5,
        tip_sigma_px: float = 35.0,
        displacement_weight: float = 0.5,
        displacement_sigma_px: float = 16.0,
        displacement_penalty_cap: float = 3.0,
        trajectory_weight: float = 0.25,
        trajectory_sigma_px: float = 12.0,
        trajectory_min_history: int = 10,
        conditional_kalman_jump_threshold_px: float = 12.0,
        encoder_static_hold_threshold_px: float = 0.0,
        encoder_static_hold_min_jump_px: float = 12.0,
        start_after_valid_encoder_frames: int = 10,
        require_encoder_needle_visible: bool = True,
        encoder_visible_margin_px: float = 0.0,
        min_motion_displacement_px: float = 1.0,
        kalman_process_noise: float = 1.0,
        kalman_measurement_noise: float = 16.0,
    ):
        use_cuda = torch.cuda.is_available() if device == "auto" else device == "cuda"
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested, but torch.cuda.is_available() is False.")
        self.device = torch.device("cuda" if use_cuda else "cpu")
        self.image_size = image_size
        self.args = SimpleNamespace(
            radius_px=radius_px,
            topk=topk,
            nms_radius=nms_radius,
            cnn_weight=cnn_weight,
            motion_weight=motion_weight,
            tip_weight=tip_weight,
            tip_sigma_px=tip_sigma_px,
            displacement_weight=displacement_weight,
            displacement_sigma_px=displacement_sigma_px,
            displacement_penalty_cap=displacement_penalty_cap,
            trajectory_weight=trajectory_weight,
            trajectory_sigma_px=trajectory_sigma_px,
            trajectory_min_history=trajectory_min_history,
            trajectory_cos_threshold=0.7,
            trajectory_min_tip_motion_px=1.0,
            trajectory_min_encoder_motion_px=1.0,
        )
        self.conditional_kalman_jump_threshold_px = conditional_kalman_jump_threshold_px
        self.encoder_static_hold_threshold_px = encoder_static_hold_threshold_px
        self.encoder_static_hold_min_jump_px = encoder_static_hold_min_jump_px
        self.start_after_valid_encoder_frames = start_after_valid_encoder_frames
        self.require_encoder_needle_visible = require_encoder_needle_visible
        self.encoder_visible_margin_px = encoder_visible_margin_px
        self.min_motion_displacement_px = min_motion_displacement_px
        self.image_history = deque(maxlen=motion_window)
        self.tracking_history = deque(maxlen=max(motion_window, trajectory_min_history))
        self.kalman = PointKalmanFilter(kalman_process_noise, kalman_measurement_noise)
        self.visible_run_count = 0
        self.was_visible = False

        self.model = SmallUNet(base_channels=base_channels).to(self.device)
        self.model.load_state_dict(
            torch.load(Path(model_path), map_location=self.device, weights_only=True)
        )
        self.model.eval()

    def reset(self):
        """Discard temporal state when the encoder needle leaves the image."""
        self.image_history.clear()
        self.tracking_history.clear()
        self.kalman.reset()
        self.visible_run_count = 0
        self.was_visible = False

    @torch.no_grad()
    def update(self, frame, encoder_state: Optional[EncoderState] = None) -> TrackingResult:
        """Process one native-size grayscale/BGR frame and return a 512-space tip."""
        canonical_gray = self._prepare_canonical_gray(frame)
        encoder_state = encoder_state or EncoderState()
        encoder_visible = encoder_needle_in_image(encoder_state, self.encoder_visible_margin_px)

        if self.require_encoder_needle_visible and not encoder_visible:
            self.reset()
            return TrackingResult(detected=False, reason="encoder_needle_not_visible")

        if encoder_visible:
            self.visible_run_count = self.visible_run_count + 1 if self.was_visible else 1
            self.was_visible = True
        else:
            self.visible_run_count = 0
            self.was_visible = False

        if self.visible_run_count <= self.start_after_valid_encoder_frames:
            self.image_history.append({"image": self._model_input_array(canonical_gray), "encoder_tip": self._encoder_tip_tuple(encoder_state)})
            return TrackingResult(detected=False, reason="warmup", visible_run_count=self.visible_run_count)

        image_array = self._model_input_array(canonical_gray)
        tensor = torch.from_numpy(image_array).unsqueeze(0).unsqueeze(0).to(self.device)
        cnn_map = self.model(tensor).detach().cpu().numpy()[0, 0].astype(np.float32)

        encoder_tip = self._encoder_tip_tuple(encoder_state)
        if valid_encoder_tip(encoder_tip):
            motion_map = encoder_directional_motion_map(image_array, list(self.image_history), encoder_tip, self.min_motion_displacement_px)
        else:
            motion_map = np.zeros_like(cnn_map, dtype=np.float32)

        selected, expected_position, encoder_valid, encoder_x, encoder_y = self._select_candidate(cnn_map, motion_map, encoder_tip)
        previous_position = None
        if self.tracking_history:
            previous_position = (self.tracking_history[-1]["final_x"], self.tracking_history[-1]["final_y"])
        pre_x, pre_y = selected["x"], selected["y"]
        pre_jump_px = math.hypot(pre_x - previous_position[0], pre_y - previous_position[1]) if previous_position else None
        kalman_x, kalman_y = self.kalman.update(pre_x, pre_y)
        used_kalman = pre_jump_px is not None and pre_jump_px >= self.conditional_kalman_jump_threshold_px
        final_x = kalman_x if used_kalman else pre_x
        final_y = kalman_y if used_kalman else pre_y

        self.image_history.append({"image": image_array, "encoder_tip": encoder_tip if valid_encoder_tip(encoder_tip) else None})
        self.tracking_history.append({"final_x": final_x, "final_y": final_y, "encoder_tip": encoder_tip if valid_encoder_tip(encoder_tip) else None})
        return TrackingResult(
            detected=True,
            final_x=final_x,
            final_y=final_y,
            pre_x=pre_x,
            pre_y=pre_y,
            kalman_x=kalman_x,
            kalman_y=kalman_y,
            encoder_x=encoder_x,
            encoder_y=encoder_y,
            used_kalman=used_kalman,
            reason=selected.get("selection_reason", "final_score"),
            visible_run_count=self.visible_run_count,
            cnn_value=selected.get("cnn_value"),
            final_score=selected.get("final_score"),
            displacement_error_px=selected.get("displacement_error_px"),
            trajectory_score=selected.get("trajectory_score"),
        )

    def draw_overlay(self, frame, result: TrackingResult, final_only: bool = True):
        """Draw diagnostics at native resolution without changing aspect ratio."""
        # Inference coordinates remain in the model's canonical 512x512 space, but
        # the visualization must retain the camera frame's native dimensions.  A
        # square output would change the ultrasound aspect ratio and would no
        # longer align with Slicer's physical image geometry.
        output = np.ascontiguousarray(frame.copy())
        if output.ndim == 2:
            output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
        height, width = output.shape[:2]
        scale_x = width / CANONICAL_SIZE
        scale_y = height / CANONICAL_SIZE
        marker_scale = max(0.5, min(scale_x, scale_y))

        def native_point(x_coord, y_coord):
            return (
                int(round(float(x_coord) * scale_x)),
                int(round(float(y_coord) * scale_y)),
            )

        if result.detected and result.final_x is not None and result.final_y is not None:
            x, y = native_point(result.final_x, result.final_y)
            dx = max(2, int(round(10 * scale_x)))
            dy = max(2, int(round(10 * scale_y)))
            cv2.line(output, (x - dx, y - dy), (x + dx, y + dy), (0, 255, 0), 2)
            cv2.line(output, (x - dx, y + dy), (x + dx, y - dy), (0, 255, 0), 2)
        if not final_only and result.pre_x is not None and result.pre_y is not None:
            x, y = native_point(result.pre_x, result.pre_y)
            cv2.drawMarker(output, (x, y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=max(6, int(round(18 * marker_scale))), thickness=2)
        if not final_only and result.encoder_x is not None and result.encoder_y is not None:
            x, y = native_point(result.encoder_x, result.encoder_y)
            cv2.drawMarker(output, (x, y), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=max(6, int(round(14 * marker_scale))), thickness=2)
            cv2.ellipse(
                output,
                (x, y),
                (
                    max(1, int(round(self.args.radius_px * scale_x))),
                    max(1, int(round(self.args.radius_px * scale_y))),
                ),
                0.0,
                0.0,
                360.0,
                (255, 255, 0),
                1,
            )
        status = "DETECTED" if result.detected else result.reason
        text_scale = 0.7 * marker_scale
        cv2.putText(output, status, native_point(10, 28), cv2.FONT_HERSHEY_SIMPLEX, text_scale, (255, 255, 255), 2)
        if result.detected:
            cv2.putText(output, f"cnn={result.cnn_value:.3f} kalman={result.used_kalman}", native_point(10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55 * marker_scale, (255, 255, 255), 1)
        return output

    def _prepare_canonical_gray(self, frame):
        """Convert to grayscale and create the canonical 512x512 working image."""
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        return cv2.resize(gray, (CANONICAL_SIZE, CANONICAL_SIZE), interpolation=cv2.INTER_LINEAR)

    def _model_input_array(self, canonical_gray):
        """Resize canonical data to the network input and normalize to [0, 1]."""
        small = cv2.resize(canonical_gray, (self.image_size, self.image_size), interpolation=cv2.INTER_LINEAR)
        return small.astype(np.float32) / 255.0

    def _encoder_tip_tuple(self, encoder_state: EncoderState):
        if encoder_state.tip_x is None or encoder_state.tip_y is None:
            return None
        return float(encoder_state.tip_x), float(encoder_state.tip_y)

    def _select_candidate(self, cnn_map, motion_map, encoder_tip):
        """Score heatmap maxima using encoder, displacement, and trajectory priors."""
        args = self.args
        encoder_valid = valid_encoder_tip(encoder_tip)
        encoder_x = encoder_y = None
        score_map = args.cnn_weight * cnn_map + args.motion_weight * motion_map
        if encoder_valid:
            encoder_x, encoder_y = encoder_tip
            score_map = score_map + args.tip_weight * gaussian_tip_prior(encoder_x, encoder_y, self.image_size, args.tip_sigma_px)
            candidate_mask = circular_mask(encoder_x, encoder_y, self.image_size, args.radius_px)
        else:
            candidate_mask = np.ones_like(score_map, dtype=bool)

        scale_back = CANONICAL_SIZE / self.image_size
        previous_position = None
        if self.tracking_history:
            previous_position = (self.tracking_history[-1]["final_x"], self.tracking_history[-1]["final_y"])

        encoder_delta_from_previous = None
        if encoder_valid and self.tracking_history and self.tracking_history[-1].get("encoder_tip") is not None:
            previous_encoder_x, previous_encoder_y = self.tracking_history[-1]["encoder_tip"]
            encoder_delta_from_previous = math.hypot(encoder_x - previous_encoder_x, encoder_y - previous_encoder_y)

        expected_position = expected_position_from_encoder_history(self.tracking_history, encoder_tip) if encoder_valid else None
        trajectory_prior = trajectory_prior_from_history(self.tracking_history, encoder_tip, args) if encoder_valid and args.trajectory_weight > 0 else None

        candidates = []
        for rank, (small_x, small_y, combined_value) in enumerate(topk_points(score_map, candidate_mask, args.topk, args.nms_radius), 1):
            candidate_x = small_x * scale_back
            candidate_y = small_y * scale_back
            cnn_value = float(cnn_map[small_y, small_x])
            tip_value = 0.0
            if encoder_valid:
                tip_distance = math.hypot(candidate_x - encoder_x, candidate_y - encoder_y)
                tip_value = math.exp(-(tip_distance * tip_distance) / (2.0 * args.tip_sigma_px * args.tip_sigma_px))
            displacement_error = None
            displacement_penalty = 0.0
            if expected_position is not None:
                displacement_error = math.hypot(candidate_x - expected_position[0], candidate_y - expected_position[1])
                displacement_penalty = min(displacement_error / max(args.displacement_sigma_px, 1e-6), args.displacement_penalty_cap)
            previous_jump = math.hypot(candidate_x - previous_position[0], candidate_y - previous_position[1]) if previous_position else None
            trajectory_score = 0.0
            if trajectory_prior is not None:
                trajectory_distance = distance_to_trajectory_line(candidate_x, candidate_y, trajectory_prior)
                trajectory_score = math.exp(-(trajectory_distance * trajectory_distance) / (2.0 * args.trajectory_sigma_px * args.trajectory_sigma_px))
            final_score = combined_value - args.displacement_weight * displacement_penalty + args.trajectory_weight * trajectory_score
            candidates.append(
                {
                    "rank": rank,
                    "x": candidate_x,
                    "y": candidate_y,
                    "final_score": final_score,
                    "cnn_value": cnn_value,
                    "tip_value": tip_value,
                    "displacement_error_px": displacement_error,
                    "previous_jump_px": previous_jump,
                    "trajectory_score": trajectory_score,
                }
            )

        if not candidates:
            y_coord, x_coord = np.unravel_index(int(np.argmax(score_map)), score_map.shape)
            candidates.append({"x": x_coord * scale_back, "y": y_coord * scale_back, "final_score": float(score_map[y_coord, x_coord]), "cnn_value": float(cnn_map[y_coord, x_coord]), "displacement_error_px": None, "previous_jump_px": None, "trajectory_score": 0.0})

        selected = max(candidates, key=lambda candidate: candidate["final_score"])
        selected["selection_reason"] = "final_score"
        if (
            previous_position is not None
            and encoder_delta_from_previous is not None
            and encoder_delta_from_previous <= self.encoder_static_hold_threshold_px
            and selected["previous_jump_px"] is not None
            and selected["previous_jump_px"] >= self.encoder_static_hold_min_jump_px
        ):
            selected = dict(selected)
            selected["x"], selected["y"] = previous_position
            selected["previous_jump_px"] = 0.0
            selected["selection_reason"] = "encoder_static_hold"
        return selected, expected_position, encoder_valid, encoder_x, encoder_y
