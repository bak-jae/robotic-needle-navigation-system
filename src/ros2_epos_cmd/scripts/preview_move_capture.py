# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

import math
import json
import os
import pathlib
import re
import time

import numpy as np
import qt
import slicer
import vtk

try:
    import cv2
except Exception:
    cv2 = None

# Recommended: add ``slicer_modules`` to Slicer's additional module paths and
# select the Needle Navigation module. This file remains available for legacy use.

TRANSFORM_NAME = "NeedleMove"
ENCODER_TRANSFORM_NAME = "NeedleMoveEncoder"
ENCODER_NEEDLE_MODEL_NAME = "NeedleEncoderModel"
PREVIEW_TRANSFORM_NAME = "NeedlePreview"
PREVIEW_NEEDLE_MODEL_NAME = "NeedlePreviewModel"
BOUNDARY_CURVE_NAME = "NeedleReachBoundary"
CLICK_POINT_NODE_NAME = "NeedleClickPoint"
TOPIC_CMD_THETA = "/needle/cmd/theta_deg"
TOPIC_CMD_LINEAR = "/needle/cmd/d_mm"
TOPIC_CMD_THETA_VELOCITY = "/needle/cmd/theta_velocity"
TOPIC_CMD_LINEAR_VELOCITY = "/needle/cmd/d_velocity"
TOPIC_STATE_THETA = "/needle/state/theta_deg"
TOPIC_STATE_LINEAR = "/needle/state/d_mm"
THETA_MIN_DEG = 0
THETA_MAX_DEG = 35
D_MIN_MM = 0
D_MAX_MM = 55
THETA_SLIDER_SCALE = 100.0
D_SLIDER_SCALE = 100.0
CLICK_SLICE_VIEWS = ("Red",)
EXECUTE_DEFAULT_STROKE_MM = 5.0
EXECUTE_DEFAULT_CYCLE_SEC = 1.0
LINEAR_CMD_SIGN = -1.0

x0 = 0.0
y0 = 0.0
L = 13.4
offset = math.radians(0.0)
D_SIGN = 1.0

POS_AXIS_X = "X"
POS_AXIS_Y = "Z"
POS_SIGN_X = -1.0
POS_SIGN_Y = -1.0
CENTER_RAS_X = 0.0
CENTER_RAS_Y = 0.0
CENTER_RAS_Z = 0.0
BASE_ROT_AXIS = "Y"
BASE_ROT_SIGN = -1.0

EXTRA_ROT_DEG = 180.0
EXTRA_ROT_AXIS = "Y"
VOLUME_NODE_NAME = "Image_Reference"
VOLUME_PREVIEW_INTERVAL_MS = 50
VOLUME_PREVIEW_WIDTH = 384
VOLUME_PROJECTION_MODE = "Max"
PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[3]
VOLUME_RECORD_OUTPUT_DIR = os.environ.get(
    "NEEDLE_RECORD_DIR",
    str(PROJECT_ROOT / "runtime" / "volume_recordings"),
)
BOUNDARY_LINE_THICKNESS = 0.2
BOUNDARY_GLYPH_SCALE = 0.2
LINEAR_MM_PER_REV = 2.0
LINEAR_GEAR_RATIO = 57.0 / 13.0
DEFAULT_ROTATIONAL_SPEED_RPM = 3
DEFAULT_LINEAR_SPEED_MM_PER_S = 60.0
PNG_COMPRESSION = 1
MOVE_D_TOL_MM = 0.1
MOVE_THETA_TOL_DEG = 0.1
MOVE_TIMER_INTERVAL_MS = 20
MOVE_PHASE_RETRACT = "retract"
MOVE_PHASE_ROTATE = "rotate"
MOVE_PHASE_INSERT = "insert"


def next_sequence_dir(output_dir):
    os.makedirs(output_dir, exist_ok=True)
    seq_re = re.compile(r"^seq_(\d+)$")
    max_index = 0
    for name in os.listdir(output_dir):
        path = os.path.join(output_dir, name)
        if not os.path.isdir(path):
            continue
        match = seq_re.match(name)
        if match:
            max_index = max(max_index, int(match.group(1)))
    return os.path.join(output_dir, f"seq_{max_index + 1:03d}")


def normalize_to_uint8(frame):
    frame = np.asarray(frame)
    if frame.dtype == np.uint8:
        return frame

    frame = frame.astype(np.float32, copy=False)
    fmin = float(frame.min())
    fmax = float(frame.max())
    if fmax <= fmin:
        return np.zeros(frame.shape, dtype=np.uint8)
    scaled = (frame - fmin) * (255.0 / (fmax - fmin))
    return np.clip(scaled, 0, 255).astype(np.uint8)


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def get_or_create_transform(name=TRANSFORM_NAME):
    try:
        return slicer.util.getNode(name)
    except slicer.util.MRMLNodeNotFoundException:
        return slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", name)


def get_or_create_click_point(name=CLICK_POINT_NODE_NAME):
    try:
        node = slicer.util.getNode(name)
    except slicer.util.MRMLNodeNotFoundException:
        node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", name)
        node.CreateDefaultDisplayNodes()
    display = node.GetDisplayNode()
    if display:
        display.SetColor(1.0, 1.0, 0.0)
        display.SetSelectedColor(1.0, 1.0, 0.0)
        if hasattr(display, "SetGlyphScale"):
            display.SetGlyphScale(2.2)
    return node


def get_or_create_boundary_curve(name=BOUNDARY_CURVE_NAME):
    try:
        node = slicer.util.getNode(name)
    except slicer.util.MRMLNodeNotFoundException:
        node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsClosedCurveNode", name)
        node.CreateDefaultDisplayNodes()
    display = node.GetDisplayNode()
    if display:
        display.SetColor(0.0, 1.0, 1.0)
        display.SetSelectedColor(0.0, 1.0, 1.0)
        display.SetLineThickness(BOUNDARY_LINE_THICKNESS)
        if hasattr(display, "SetGlyphScale"):
            display.SetGlyphScale(BOUNDARY_GLYPH_SCALE)
        if hasattr(display, "SetPropertiesLabelVisibility"):
            display.SetPropertiesLabelVisibility(False)
    return node


def get_or_create_needle_model(name, transform_node, color=(1.0, 0.55, 0.0), opacity=1.0):
    line = vtk.vtkLineSource()
    line.SetPoint1(0.0, 0.0, 0.0)
    line.SetPoint2(0.0, 0.0, -float(L))
    line.Update()

    tube = vtk.vtkTubeFilter()
    tube.SetInputConnection(line.GetOutputPort())
    tube.SetRadius(0.35)
    tube.SetNumberOfSides(16)
    tube.CappingOn()
    tube.Update()
    polydata = tube.GetOutput()

    try:
        node = slicer.util.getNode(name)
        node.SetAndObservePolyData(polydata)
    except slicer.util.MRMLNodeNotFoundException:
        node = slicer.modules.models.logic().AddModel(polydata)
        node.SetName(name)

    if node.GetDisplayNode() is None:
        node.CreateDefaultDisplayNodes()
    display = node.GetDisplayNode()
    if display:
        display.SetColor(float(color[0]), float(color[1]), float(color[2]))
        display.SetOpacity(float(opacity))
        display.SetVisibility2D(True)
        display.SetVisibility3D(True)

    node.SetAndObserveTransformNodeID(transform_node.GetID())
    return node


def compute_pose(theta_deg, d):
    theta = math.radians(theta_deg)
    effective_d = D_SIGN * d
    x = x0 + (L - effective_d) * math.sin(offset + theta)
    y = y0 - (L - effective_d) * math.cos(offset + theta)
    angle = offset + theta
    return x, y, angle


def ui_d_to_cmd_d(d_ui):
    return LINEAR_CMD_SIGN * float(d_ui)


def cmd_d_to_ui_d(d_cmd):
    return LINEAR_CMD_SIGN * float(d_cmd)


def rotation_matrix(angle_rad, axis):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    axis = axis.upper()
    if axis == "X":
        return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
    if axis == "Y":
        return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
    if axis == "Z":
        return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    raise ValueError(f"Unsupported axis: {axis}")


def matmul3(a, b):
    return [
        [
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ],
        [
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ],
        [
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ],
    ]


def motion_xy_to_ras(x, y):
    pos = {"X": CENTER_RAS_X, "Y": CENTER_RAS_Y, "Z": CENTER_RAS_Z}
    pos[POS_AXIS_X] += POS_SIGN_X * x
    pos[POS_AXIS_Y] += POS_SIGN_Y * y
    return pos["X"], pos["Y"], pos["Z"]


def ras_to_motion_xy(ras):
    axis = {"X": ras[0], "Y": ras[1], "Z": ras[2]}
    x = (axis[POS_AXIS_X] - {"X": CENTER_RAS_X, "Y": CENTER_RAS_Y, "Z": CENTER_RAS_Z}[POS_AXIS_X]) / POS_SIGN_X if POS_SIGN_X != 0.0 else 0.0
    y = (axis[POS_AXIS_Y] - {"X": CENTER_RAS_X, "Y": CENTER_RAS_Y, "Z": CENTER_RAS_Z}[POS_AXIS_Y]) / POS_SIGN_Y if POS_SIGN_Y != 0.0 else 0.0
    return x, y


def world_ras_to_node_local_ras(ras, node):
    parent_transform_node = node.GetParentTransformNode()
    if parent_transform_node is None:
        return ras

    transform = vtk.vtkGeneralTransform()
    slicer.vtkMRMLTransformNode.GetTransformBetweenNodes(None, parent_transform_node, transform)
    local_ras = transform.TransformPoint(ras[0], ras[1], ras[2])
    return local_ras


def solve_theta_d_from_xy(x, y, theta_min_deg, theta_max_deg, d_min_mm, d_max_mm):
    if x > 0.0:
        return None

    dx = x - x0
    dy = y - y0
    r = math.hypot(dx, dy)
    if r < 1e-9:
        return None

    alpha = math.atan2(dx, -dy)
    candidates = []
    for alpha_candidate, radial in ((alpha, r), (alpha + math.pi, -r)):
        theta_deg = math.degrees(alpha_candidate - offset)
        d_mm = (L - radial) / D_SIGN if D_SIGN != 0.0 else float("inf")
        if theta_min_deg <= theta_deg <= theta_max_deg and d_min_mm <= d_mm <= d_max_mm:
            candidates.append((theta_deg, d_mm))

    if not candidates:
        return None
    return min(candidates, key=lambda td: abs(td[0]) + abs(td[1]))


def apply_transform(transform_node, x, y, angle):
    r_base = rotation_matrix(angle * BASE_ROT_SIGN, BASE_ROT_AXIS)
    r_extra = rotation_matrix(math.radians(EXTRA_ROT_DEG), EXTRA_ROT_AXIS)
    r = matmul3(r_base, r_extra)

    m = vtk.vtkMatrix4x4()
    m.Identity()
    for rr in range(3):
        for cc in range(3):
            m.SetElement(rr, cc, r[rr][cc])

    rx, ry, rz = motion_xy_to_ras(x, y)
    m.SetElement(0, 3, rx)
    m.SetElement(1, 3, ry)
    m.SetElement(2, 3, rz)
    transform_node.SetMatrixTransformToParent(m)


def update_boundary_curve(curve_node, theta_min_deg, theta_max_deg, d_min_mm, d_max_mm):
    theta_samples = 100
    d_samples = 100
    boundary_xy = []

    for ti in range(theta_samples + 1):
        theta_deg = theta_min_deg + (theta_max_deg - theta_min_deg) * (ti / float(theta_samples))
        x, y, _ = compute_pose(theta_deg, d_min_mm)
        boundary_xy.append((x, y))

    for di in range(1, d_samples + 1):
        d_mm = d_min_mm + (d_max_mm - d_min_mm) * (di / float(d_samples))
        x, y, _ = compute_pose(theta_max_deg, d_mm)
        boundary_xy.append((x, y))

    for ti in range(theta_samples - 1, -1, -1):
        theta_deg = theta_min_deg + (theta_max_deg - theta_min_deg) * (ti / float(theta_samples))
        x, y, _ = compute_pose(theta_deg, d_max_mm)
        boundary_xy.append((x, y))

    for di in range(d_samples - 1, 0, -1):
        d_mm = d_min_mm + (d_max_mm - d_min_mm) * (di / float(d_samples))
        x, y, _ = compute_pose(theta_min_deg, d_mm)
        boundary_xy.append((x, y))

    curve_node.RemoveAllControlPoints()
    for x, y in boundary_xy:
        rx, ry, rz = motion_xy_to_ras(x, y)
        curve_node.AddControlPoint(vtk.vtkVector3d(rx, ry, rz))


def get_or_create_publisher(node, topic_name):
    existing = slicer.mrmlScene.GetNodesByClass("vtkMRMLROS2PublisherDoubleNode")
    for i in range(existing.GetNumberOfItems()):
        n = existing.GetItemAsObject(i)
        if hasattr(n, "GetTopicName") and n.GetTopicName() == topic_name:
            return n
    return node.CreateAndAddPublisherNode("Double", topic_name)


def get_or_create_bool_publisher(node, topic_name):
    existing = slicer.mrmlScene.GetNodesByClass("vtkMRMLROS2PublisherBoolNode")
    for i in range(existing.GetNumberOfItems()):
        n = existing.GetItemAsObject(i)
        if hasattr(n, "GetTopicName") and n.GetTopicName() == topic_name:
            return n
    return node.CreateAndAddPublisherNode("Bool", topic_name)


def get_or_create_subscriber(node, topic_name):
    existing = slicer.mrmlScene.GetNodesByClass("vtkMRMLROS2SubscriberDoubleNode")
    for i in range(existing.GetNumberOfItems()):
        n = existing.GetItemAsObject(i)
        if hasattr(n, "GetTopicName") and n.GetTopicName() == topic_name:
            return n
    return node.CreateAndAddSubscriberNode("Double", topic_name)


def remove_node_if_exists(name):
    try:
        node = slicer.util.getNode(name)
    except slicer.util.MRMLNodeNotFoundException:
        return
    slicer.mrmlScene.RemoveNode(node)


rosLogic = slicer.util.getModuleLogic("ROS2")
rosNode = rosLogic.GetDefaultROS2Node()
if rosNode is None:
    raise RuntimeError("Default ROS2 node is not available in Slicer")

# If the script is re-run, previously created nodes can remain in the ROS2 node.
# Remove by topic first to avoid duplicate subscriber/publisher errors.
for _topic in (TOPIC_STATE_THETA, TOPIC_STATE_LINEAR):
    try:
        rosNode.RemoveAndDeleteSubscriberNode(_topic)
    except Exception:
        pass
for _topic in (TOPIC_CMD_THETA, TOPIC_CMD_LINEAR, TOPIC_CMD_THETA_VELOCITY, TOPIC_CMD_LINEAR_VELOCITY):
    try:
        rosNode.RemoveAndDeletePublisherNode(_topic)
    except Exception:
        pass
pubTheta = get_or_create_publisher(rosNode, TOPIC_CMD_THETA)
pubLinear = get_or_create_publisher(rosNode, TOPIC_CMD_LINEAR)
pubThetaVelocity = get_or_create_publisher(rosNode, TOPIC_CMD_THETA_VELOCITY)
pubLinearVelocity = get_or_create_publisher(rosNode, TOPIC_CMD_LINEAR_VELOCITY)
subTheta = get_or_create_subscriber(rosNode, TOPIC_STATE_THETA)
subLinear = get_or_create_subscriber(rosNode, TOPIC_STATE_LINEAR)
transformNode = get_or_create_transform()
encoderTransformNode = get_or_create_transform(ENCODER_TRANSFORM_NAME)
encoderNeedleModelNode = get_or_create_needle_model(ENCODER_NEEDLE_MODEL_NAME, encoderTransformNode)
previewTransformNode = get_or_create_transform(PREVIEW_TRANSFORM_NAME)
previewNeedleModelNode = get_or_create_needle_model(
    PREVIEW_NEEDLE_MODEL_NAME,
    previewTransformNode,
    color=(0.0, 1.0, 0.25),
    opacity=0.55,
)
_preview_display = previewNeedleModelNode.GetDisplayNode()
if _preview_display:
    _preview_display.SetVisibility(False)
    if hasattr(_preview_display, "SetVisibility2D"):
        _preview_display.SetVisibility2D(False)
    if hasattr(_preview_display, "SetVisibility3D"):
        _preview_display.SetVisibility3D(False)
boundaryCurveNode = get_or_create_boundary_curve()
clickPointNode = get_or_create_click_point()
remove_node_if_exists("NeedleRotationCenter")
remove_node_if_exists("NeedleWorkPlane")


class NeedleSlicerClosedLoop(qt.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Preview Move Capture")

        self._cmd_theta = 0.0
        self._cmd_d = 0.0
        self._enc_theta = 0.0
        self._enc_d = 0.0
        self._is_execute_running = False
        self._execute_base_d = 0.0
        self._execute_forward_d = 0.0
        self._execute_phase = 0
        self._execute_cycles_total = 0
        self._execute_cycles_done = 0
        self._slice_click_observers = []
        self._execute_timer = qt.QTimer()
        self._execute_timer.timeout.connect(self.on_execute_tick)
        self._click_move_timer = qt.QTimer()
        self._click_move_timer.setInterval(20)
        self._click_move_timer.timeout.connect(self.on_click_move_tick)
        self._click_move_pending = False
        self._click_move_target_theta = 0.0
        self._click_move_target_d = 0.0
        self._click_move_target_theta_int = 0
        self._preview_has_target = False
        self._preview_target_theta = 0.0
        self._preview_target_d = 0.0
        self._preview_move_timer = qt.QTimer()
        self._preview_move_timer.setInterval(MOVE_TIMER_INTERVAL_MS)
        self._preview_move_timer.timeout.connect(self.on_preview_move_tick)
        self._preview_move_running = False
        self._preview_move_phase = None
        self._preview_move_target_theta_int = 0
        self._preview_move_target_d_int = 0
        self._home_move_timer = qt.QTimer()
        self._home_move_timer.setInterval(20)
        self._home_move_timer.timeout.connect(self.on_home_move_tick)
        self._home_move_pending = False
        self._home_target_theta = 0.0
        self._home_target_d = 0.0
        self._home_target_d_int = 0
        self._volume_timer = qt.QTimer()
        self._volume_timer.timeout.connect(self.update_volume_preview)
        self._volume_frame_index = 0
        self._volume_fps_window_start = None
        self._volume_fps_window_frames = 0
        self._volume_fps = 0.0
        self._recording = False
        self._video_writer = None
        self._video_path = None
        self._recording_session_dir = None
        self._recording_frames_dir = None
        self._recording_overlay_dir = None
        self._recording_log_path = None
        self._recording_log_file = None
        self._recording_log_footer_pos = None
        self._recording_log_frames_written = 0
        self._recording_start_time = None
        self._recording_frame_id = 0
        self._png_write_params = None

        layout = qt.QVBoxLayout(self)
        form = qt.QFormLayout()

        self.thetaSlider = qt.QSlider(qt.Qt.Horizontal)
        self.thetaSlider.setMinimum(self.theta_to_slider(float(THETA_MIN_DEG)))
        self.thetaSlider.setMaximum(self.theta_to_slider(float(THETA_MAX_DEG)))
        self.thetaSlider.setValue(0)

        self.dSlider = qt.QSlider(qt.Qt.Horizontal)
        self.dSlider.setMinimum(self.d_to_slider(float(D_MIN_MM)))
        self.dSlider.setMaximum(self.d_to_slider(float(D_MAX_MM)))
        self.dSlider.setValue(0)

        self.cmdThetaLabel = qt.QLabel("theta [deg]: 0.00")
        self.cmdDLabel = qt.QLabel("d [mm]: 0.00")
        self.currentPosLabel = qt.QLabel("Cmd tip (RAS) [mm]: -")
        self.encoderPosLabel = qt.QLabel("Encoder tip (RAS) [mm]: -")
        self.clickLabel = qt.QLabel("Last click (RAS) [mm]: -")
        self.ikLabel = qt.QLabel("Target from click: theta=- [deg], d=- [mm]")
        self.executeStrokeSpin = qt.QDoubleSpinBox()
        self.executeStrokeSpin.setMinimum(0.1)
        self.executeStrokeSpin.setMaximum(55.0)
        self.executeStrokeSpin.setDecimals(2)
        self.executeStrokeSpin.setSingleStep(0.5)
        self.executeStrokeSpin.setValue(EXECUTE_DEFAULT_STROKE_MM)
        self.executeStrokeSpin.setSuffix(" [mm]")
        self.executeCountSpin = qt.QSpinBox()
        self.executeCountSpin.setMinimum(1)
        self.executeCountSpin.setMaximum(1000)
        self.executeCountSpin.setValue(1)
        self.executeCountSpin.setSuffix(" cycles")
        self.moveButton = qt.QPushButton("Move")
        self.moveButton.setEnabled(False)
        self.stopButton = qt.QPushButton("Stop")
        self.executeButton = qt.QPushButton("Sample Start")
        self.executeButton.setCheckable(True)
        self.homeButton = qt.QPushButton("Home")
        self.executeStatusLabel = qt.QLabel("Sample: idle")
        self.previewStatusLabel = qt.QLabel("Preview: click Red slice")
        self.volumeNameLabel = qt.QLabel(f"Volume node: {VOLUME_NODE_NAME}")
        self.thetaVelocitySpin = qt.QSpinBox()
        self.thetaVelocitySpin.setMinimum(1)
        self.thetaVelocitySpin.setMaximum(100000)
        self.thetaVelocitySpin.setValue(DEFAULT_ROTATIONAL_SPEED_RPM)
        self.thetaVelocitySpin.setSuffix(" rpm")
        self.linearVelocitySpin = qt.QDoubleSpinBox()
        self.linearVelocitySpin.setMinimum(0.01)
        self.linearVelocitySpin.setMaximum(100000.0)
        self.linearVelocitySpin.setDecimals(2)
        self.linearVelocitySpin.setSingleStep(1.0)
        self.linearVelocitySpin.setValue(DEFAULT_LINEAR_SPEED_MM_PER_S)
        self.linearVelocitySpin.setSuffix(" mm/s")
        self.volumeIntervalSpin = qt.QSpinBox()
        self.volumeIntervalSpin.setMinimum(10)
        self.volumeIntervalSpin.setMaximum(5000)
        self.volumeIntervalSpin.setValue(VOLUME_PREVIEW_INTERVAL_MS)
        self.volumeIntervalSpin.setSuffix(" ms")
        self.volumeProjectionModeCombo = qt.QComboBox()
        self.volumeProjectionModeCombo.addItems(["Max", "Mean"])
        projection_mode_index = self.volumeProjectionModeCombo.findText(VOLUME_PROJECTION_MODE)
        if projection_mode_index >= 0:
            self.volumeProjectionModeCombo.setCurrentIndex(projection_mode_index)
        self.volumeSaveDirEdit = qt.QLineEdit(VOLUME_RECORD_OUTPUT_DIR)
        self.recordButton = qt.QPushButton("Record Start")
        self.recordButton.setCheckable(True)
        self.volumePreviewLabel = qt.QLabel("No volume frame")
        self.volumePreviewLabel.setMinimumSize(VOLUME_PREVIEW_WIDTH, VOLUME_PREVIEW_WIDTH)
        self.volumePreviewLabel.setAlignment(qt.Qt.AlignCenter)
        self.volumePreviewLabel.setStyleSheet("background-color: black; color: white;")
        self.volumeInfoLabel = qt.QLabel("volume numpy: -")

        form.addRow(self.cmdThetaLabel, self.thetaSlider)
        form.addRow(self.cmdDLabel, self.dSlider)
        form.addRow("Sample stroke", self.executeStrokeSpin)
        form.addRow("Sample count", self.executeCountSpin)
        form.addRow("", self.moveButton)
        form.addRow("", self.executeButton)
        form.addRow("", self.stopButton)
        form.addRow("", self.homeButton)
        form.addRow("", self.executeStatusLabel)
        form.addRow("", self.previewStatusLabel)
        form.addRow("", self.currentPosLabel)
        form.addRow("", self.encoderPosLabel)
        form.addRow("", self.clickLabel)
        form.addRow("", self.ikLabel)
        form.addRow("Rotational speed [tip rpm]", self.thetaVelocitySpin)
        form.addRow("Linear speed [mm/s]", self.linearVelocitySpin)
        form.addRow("", self.volumeNameLabel)
        form.addRow("Capture interval", self.volumeIntervalSpin)
        form.addRow("Projection mode", self.volumeProjectionModeCombo)
        form.addRow("Record dir", self.volumeSaveDirEdit)
        form.addRow("", self.recordButton)
        layout.addLayout(form)
        layout.addWidget(self.volumePreviewLabel)
        layout.addWidget(self.volumeInfoLabel)

        self.thetaSlider.valueChanged.connect(self.on_slider_changed)
        self.dSlider.valueChanged.connect(self.on_slider_changed)
        self.moveButton.clicked.connect(self.start_preview_move)
        self.stopButton.clicked.connect(self.stop_all_motion)
        self.executeButton.toggled.connect(self.on_execute_toggled)
        self.thetaVelocitySpin.valueChanged.connect(self.on_theta_velocity_changed)
        self.linearVelocitySpin.valueChanged.connect(self.on_linear_velocity_changed)
        self.homeButton.clicked.connect(self.start_home_move)
        self.volumeIntervalSpin.valueChanged.connect(self.on_volume_interval_changed)
        self.recordButton.toggled.connect(self.on_record_toggled)

        self.thetaObserver = subTheta.AddObserver("ModifiedEvent", self.on_theta_state)
        self.dObserver = subLinear.AddObserver("ModifiedEvent", self.on_d_state)
        self.install_slice_click_observers()

        update_boundary_curve(
            boundaryCurveNode,
            float(THETA_MIN_DEG),
            float(THETA_MAX_DEG),
            float(D_MIN_MM),
            float(D_MAX_MM),
        )
        self.on_slider_changed()
        self.update_encoder_transform()
        self.on_theta_velocity_changed(self.thetaVelocitySpin.value)
        self.on_linear_velocity_changed(self.linearVelocitySpin.value)
        self.start_volume_preview()

    def cleanup(self):
        self._execute_timer.stop()
        self._click_move_timer.stop()
        self._preview_move_timer.stop()
        self._home_move_timer.stop()
        self._volume_timer.stop()
        self.stop_recording()
        if getattr(self, "thetaObserver", None) is not None:
            subTheta.RemoveObserver(self.thetaObserver)
            self.thetaObserver = None
        if getattr(self, "dObserver", None) is not None:
            subLinear.RemoveObserver(self.dObserver)
            self.dObserver = None
        for interactor, observer_id in self._slice_click_observers:
            try:
                interactor.RemoveObserver(observer_id)
            except Exception:
                pass
        self._slice_click_observers = []

    def closeEvent(self, event):
        self.cleanup()
        try:
            qt.QWidget.closeEvent(self, event)
        except Exception:
            pass

    def theta_to_slider(self, theta_deg):
        return int(round(theta_deg * THETA_SLIDER_SCALE))

    def d_to_slider(self, d_mm):
        return int(round(d_mm * D_SLIDER_SCALE))

    def slider_to_theta(self, slider_value):
        return float(slider_value) / THETA_SLIDER_SCALE

    def slider_to_d(self, slider_value):
        return float(slider_value) / D_SLIDER_SCALE

    def on_slider_changed(self, *args):
        if self._is_execute_running or self._click_move_pending or self._preview_move_running or self._home_move_pending:
            return
        theta = self.slider_to_theta(self.thetaSlider.value)
        d = self.slider_to_d(self.dSlider.value)
        self.apply_command(theta, d)

    def apply_command(self, theta, d):
        self.cmdThetaLabel.setText(f"theta [deg]: {theta:.2f}")
        self.cmdDLabel.setText(f"d [mm]: {d:.2f}")
        self._cmd_theta = theta
        self._cmd_d = d
        self.update_command_transform()
        pubTheta.Publish(theta)
        pubLinear.Publish(ui_d_to_cmd_d(d))

    def on_theta_velocity_changed(self, value):
        pubThetaVelocity.Publish(float(value))

    def on_linear_velocity_changed(self, value):

        pubLinearVelocity.Publish(float(value))
    def on_execute_toggled(self, checked):
        if self._preview_move_running:
            self.executeButton.blockSignals(True)
            self.executeButton.setChecked(False)
            self.executeButton.blockSignals(False)
            self.executeStatusLabel.setText("Sample: ignored during Move")
            return
        if checked:
            if not self.start_execute():
                self.executeButton.blockSignals(True)
                self.executeButton.setChecked(False)
                self.executeButton.blockSignals(False)
            return
        self.stop_execute()

    def start_execute(self):
        if self._preview_move_running:
            self.executeStatusLabel.setText("Sample: ignored during Move")
            return False

        stroke_mm = float(self.executeStrokeSpin.value)
        if stroke_mm <= 0.0:
            self.executeStatusLabel.setText("Sample: invalid stroke [mm]")
            return False

        d_now = self.slider_to_d(self.dSlider.value)
        d_forward = d_now + stroke_mm
        if d_forward > float(D_MAX_MM):
            d_forward = d_now - stroke_mm
        if d_forward < float(D_MIN_MM) or d_forward > float(D_MAX_MM):
            self.executeStatusLabel.setText("Sample: out of d range [mm]")
            return False

        self._execute_base_d = d_now
        self._execute_forward_d = d_forward
        self._execute_phase = 0
        self._execute_cycles_total = int(self.executeCountSpin.value)
        self._execute_cycles_done = 0
        self._is_execute_running = True
        self.executeButton.setText("Sample Stop")
        self.executeStatusLabel.setText(
            f"Sample: running, base={self._execute_base_d:.2f} [mm], target={self._execute_forward_d:.2f} [mm], "
            f"cycles=0/{self._execute_cycles_total}"
        )

        half_period_ms = max(20, int(round(EXECUTE_DEFAULT_CYCLE_SEC * 500.0)))
        self._execute_timer.start(half_period_ms)
        self.send_execute_target(self._execute_forward_d)
        self._execute_phase = 1
        return True

    def stop_execute(self):
        if self._is_execute_running:
            self._execute_timer.stop()
            self._is_execute_running = False
            self.executeStatusLabel.setText("Sample: stopped")
        self.executeButton.setText("Sample Start")

    def on_execute_tick(self):
        if not self._is_execute_running:
            return
        if self._execute_phase == 0:
            self.send_execute_target(self._execute_forward_d)
            self._execute_phase = 1
        else:
            self.send_execute_target(self._execute_base_d)
            self._execute_phase = 0
            self._execute_cycles_done += 1
            if self._execute_cycles_done >= self._execute_cycles_total:
                self.executeButton.blockSignals(True)
                self.executeButton.setChecked(False)
                self.executeButton.blockSignals(False)
                self.stop_execute()
                self.executeStatusLabel.setText(
                    f"Sample: completed {self._execute_cycles_done}/{self._execute_cycles_total} cycles"
                )
                return
        self.executeStatusLabel.setText(
            f"Sample: running, base={self._execute_base_d:.2f} [mm], target={self._execute_forward_d:.2f} [mm], "
            f"cycles={self._execute_cycles_done}/{self._execute_cycles_total}"
        )

    def send_execute_target(self, d_target):
        theta = self.slider_to_theta(self.thetaSlider.value)
        d_int = self.d_to_slider(d_target)
        d_int = max(self.dSlider.minimum, min(self.dSlider.maximum, d_int))
        d_target = self.slider_to_d(d_int)
        self.dSlider.blockSignals(True)
        self.dSlider.setValue(d_int)
        self.dSlider.blockSignals(False)
        self.apply_command(theta, d_target)

    def start_click_move(self, theta_deg, d_mm):
        if self._is_execute_running:
            self.executeButton.blockSignals(True)
            self.executeButton.setChecked(False)
            self.executeButton.blockSignals(False)
            self.stop_execute()

        current_d = self._cmd_d
        theta_int = self.theta_to_slider(theta_deg)
        d_int = self.d_to_slider(d_mm)
        theta_int = max(self.thetaSlider.minimum, min(self.thetaSlider.maximum, theta_int))
        d_int = max(self.dSlider.minimum, min(self.dSlider.maximum, d_int))

        self._click_move_target_theta_int = int(theta_int)
        self._click_move_target_theta = self.slider_to_theta(theta_int)
        self._click_move_target_d = self.slider_to_d(d_int)
        self._click_move_pending = True

        self.thetaSlider.blockSignals(True)
        self.thetaSlider.setValue(theta_int)
        self.thetaSlider.blockSignals(False)
        self.dSlider.blockSignals(True)
        self.dSlider.setValue(d_int)
        self.dSlider.blockSignals(False)

        self.executeStatusLabel.setText(
            f"Click move: rotate to {self._click_move_target_theta:.2f} deg, then insert to {self._click_move_target_d:.2f} mm"
        )
        self.apply_command(self._click_move_target_theta, current_d)
        self._click_move_timer.start()

    def on_click_move_tick(self):
        if not self._click_move_pending:
            self._click_move_timer.stop()
            return
        if self.theta_to_slider(self._enc_theta) != self._click_move_target_theta_int:
            return

        self._click_move_pending = False
        self._click_move_timer.stop()
        self.apply_command(self._click_move_target_theta, self._click_move_target_d)
        self.executeStatusLabel.setText(
            f"Click move: inserted to {self._click_move_target_d:.2f} mm at {self._click_move_target_theta:.2f} deg"
        )

    def set_sliders_to_command(self, theta, d):
        theta_int = self.theta_to_slider(theta)
        d_int = self.d_to_slider(d)
        theta_int = max(self.thetaSlider.minimum, min(self.thetaSlider.maximum, theta_int))
        d_int = max(self.dSlider.minimum, min(self.dSlider.maximum, d_int))
        theta = self.slider_to_theta(theta_int)
        d = self.slider_to_d(d_int)

        self.thetaSlider.blockSignals(True)
        self.thetaSlider.setValue(theta_int)
        self.thetaSlider.blockSignals(False)
        self.dSlider.blockSignals(True)
        self.dSlider.setValue(d_int)
        self.dSlider.blockSignals(False)
        self.apply_command(theta, d)

    def stop_all_motion(self):
        if self._is_execute_running:
            self._execute_timer.stop()
            self._is_execute_running = False
        if self._click_move_pending:
            self._click_move_pending = False
            self._click_move_timer.stop()
        if self._preview_move_running:
            self._preview_move_running = False
            self._preview_move_phase = None
            self._preview_move_timer.stop()
        if self._home_move_pending:
            self._home_move_pending = False
            self._home_move_timer.stop()

        self.executeButton.blockSignals(True)
        self.executeButton.setChecked(False)
        self.executeButton.blockSignals(False)
        self.executeButton.setText("Sample Start")
        self.moveButton.setEnabled(self._preview_has_target)
        self.executeStatusLabel.setText("Stopped")

    def start_preview_move(self):
        if not self._preview_has_target:
            self.previewStatusLabel.setText("Preview: no target")
            return
        if self._preview_move_running:
            return

        if self._is_execute_running or self._click_move_pending or self._home_move_pending:
            self.stop_all_motion()

        self._preview_move_running = True
        self._preview_move_phase = MOVE_PHASE_RETRACT
        self._preview_move_target_theta_int = self.theta_to_slider(self._preview_target_theta)
        self._preview_move_target_d_int = self.d_to_slider(self._preview_target_d)
        self.moveButton.setEnabled(False)
        self.executeStatusLabel.setText(
            f"Move: retracting to d=0.00 mm before theta={self._preview_target_theta:.2f} deg"
        )
        self.set_sliders_to_command(self._cmd_theta, 0.0)
        self._preview_move_timer.start()

    def on_preview_move_tick(self):
        if not self._preview_move_running:
            self._preview_move_timer.stop()
            return

        if self._preview_move_phase == MOVE_PHASE_RETRACT:
            if abs(self._enc_d - 0.0) > MOVE_D_TOL_MM:
                return
            self._preview_move_phase = MOVE_PHASE_ROTATE
            self.executeStatusLabel.setText(
                f"Move: rotating to theta={self._preview_target_theta:.2f} deg at d=0.00 mm"
            )
            self.set_sliders_to_command(self._preview_target_theta, 0.0)
            return

        if self._preview_move_phase == MOVE_PHASE_ROTATE:
            if abs(self._enc_theta - self._preview_target_theta) > MOVE_THETA_TOL_DEG:
                return
            self._preview_move_phase = MOVE_PHASE_INSERT
            self.executeStatusLabel.setText(
                f"Move: inserting to d={self._preview_target_d:.2f} mm"
            )
            self.set_sliders_to_command(self._preview_target_theta, self._preview_target_d)
            return

        if self._preview_move_phase == MOVE_PHASE_INSERT:
            if abs(self._enc_d - self._preview_target_d) > MOVE_D_TOL_MM:
                return
            self._preview_move_running = False
            self._preview_move_phase = None
            self._preview_move_timer.stop()
            self.moveButton.setEnabled(self._preview_has_target)
            self.executeStatusLabel.setText(
                f"Move: reached preview target theta={self._preview_target_theta:.2f} deg, "
                f"d={self._preview_target_d:.2f} mm"
            )

    def start_home_move(self):
        if self._preview_move_running:
            self.executeStatusLabel.setText("Home: ignored during Move")
            return

        if self._is_execute_running:
            self.executeButton.blockSignals(True)
            self.executeButton.setChecked(False)
            self.executeButton.blockSignals(False)
            self.stop_execute()

        if self._click_move_pending:
            self._click_move_pending = False
            self._click_move_timer.stop()

        self._home_target_theta = 0.0
        self._home_target_d = 0.0
        self._home_target_d_int = self.d_to_slider(self._home_target_d)
        self._home_move_pending = True

        self.thetaSlider.blockSignals(True)
        self.thetaSlider.setValue(self.theta_to_slider(self._home_target_theta))
        self.thetaSlider.blockSignals(False)
        self.dSlider.blockSignals(True)
        self.dSlider.setValue(self._home_target_d_int)
        self.dSlider.blockSignals(False)

        current_theta = self._cmd_theta
        self.executeStatusLabel.setText("Home move: retract d first, then rotate to theta=0")
        self.apply_command(current_theta, self._home_target_d)
        self._home_move_timer.start()

    def on_home_move_tick(self):
        if not self._home_move_pending:
            self._home_move_timer.stop()
            return
        if self.d_to_slider(self._enc_d) != self._home_target_d_int:
            return

        self._home_move_pending = False
        self._home_move_timer.stop()
        self.apply_command(self._home_target_theta, self._home_target_d)
        self.executeStatusLabel.setText("Home move: reached theta=0, d=0")

    def start_volume_preview(self):
        self._volume_frame_index = 0
        self._volume_fps_window_start = time.perf_counter()
        self._volume_fps_window_frames = 0
        self._volume_fps = 0.0
        self._volume_timer.start(int(self.volumeIntervalSpin.value))

    def on_volume_interval_changed(self, value):
        if self._volume_timer.isActive():
            self._volume_timer.start(int(value))

    def on_record_toggled(self, checked):
        if checked:
            if not self.start_recording():
                self.recordButton.blockSignals(True)
                self.recordButton.setChecked(False)
                self.recordButton.blockSignals(False)
            return
        self.stop_recording()

    def start_recording(self):
        if cv2 is None:
            self.volumeInfoLabel.setText("OpenCV (cv2) is not available in this Slicer Python")
            return False

        output_dir = self.volumeSaveDirEdit.text.strip()
        if not output_dir:
            self.volumeInfoLabel.setText("Record dir is empty")
            return False

        os.makedirs(output_dir, exist_ok=True)
        session_dir = next_sequence_dir(output_dir)
        frames_dir = os.path.join(session_dir, "frames")
        overlay_dir = os.path.join(session_dir, "frames_overlay")
        os.makedirs(frames_dir, exist_ok=False)
        os.makedirs(overlay_dir, exist_ok=False)

        self._recording = True
        self._video_writer = None
        self._video_path = None
        self._recording_session_dir = session_dir
        self._recording_frames_dir = frames_dir
        self._recording_overlay_dir = overlay_dir
        self._recording_log_path = os.path.join(session_dir, "recording_log.json")
        self._recording_log_file = None
        self._recording_log_footer_pos = None
        self._recording_log_frames_written = 0
        self._recording_start_time = time.perf_counter()
        self._recording_frame_id = 0
        self._png_write_params = [cv2.IMWRITE_PNG_COMPRESSION, int(PNG_COMPRESSION)]
        self.start_recording_log()
        self.recordButton.setText("Record Stop")
        self.volumeInfoLabel.setText(f"Recording dataset: {session_dir}")
        return True

    def stop_recording(self):
        self._recording = False
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
        self._video_path = None
        self.close_recording_log()
        if hasattr(self, "recordButton"):
            self.recordButton.setText("Record Start")

    def write_video_frame(self, frame):
        if not self._recording:
            return None

        if cv2 is None:
            return None

        output_dir = self.volumeSaveDirEdit.text.strip()
        if not output_dir:
            return None

        if not self._recording_frames_dir or not self._recording_overlay_dir or not self._recording_log_path:
            return None

        self._recording_frame_id += 1
        frame_id = self._recording_frame_id
        image_file = f"{frame_id:06d}.png"
        overlay_file = f"overlay_{frame_id:06d}.png"
        image_path = os.path.join(self._recording_frames_dir, image_file)
        overlay_path = os.path.join(self._recording_overlay_dir, overlay_file)
        gray = normalize_to_uint8(frame)
        if not cv2.imwrite(image_path, gray, self._png_write_params):
            raise RuntimeError(f"Failed to write frame: {image_path}")

        encoder_log = self.current_encoder_log(frame.shape)
        overlay = self.encoder_overlay_image(gray, encoder_log)
        if not cv2.imwrite(overlay_path, overlay, self._png_write_params):
            raise RuntimeError(f"Failed to write overlay frame: {overlay_path}")

        timestamp_sec = 0.0
        if self._recording_start_time is not None:
            timestamp_sec = time.perf_counter() - self._recording_start_time

        self.append_recording_log_entry(
            {
                "frame_id": frame_id,
                "timestamp": round(timestamp_sec, 6),
                "image_file": image_file,
                "image_size": {
                    "width": int(gray.shape[1]),
                    "height": int(gray.shape[0]),
                },
                "overlay_file": overlay_file,
                "encoder": encoder_log,
            }
        )
        return image_path

    def encoder_overlay_image(self, gray, encoder_log):
        overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        predicted_line = encoder_log.get("predicted_line")
        predicted_tip = encoder_log.get("predicted_tip")
        if predicted_line is not None:
            x1, y1, x2, y2 = predicted_line
            cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2, cv2.LINE_AA)
        if predicted_tip is not None:
            x, y = predicted_tip
            cv2.circle(overlay, (x, y), 5, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(
            overlay,
            f"theta={encoder_log['theta_deg']:.2f} deg  d={encoder_log['d_mm']:.2f} mm",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return overlay

    def start_recording_log(self):
        if not self._recording_log_path:
            return
        self._recording_log_file = open(self._recording_log_path, "w+", encoding="utf-8")
        self._recording_log_file.write("{\n")
        self._recording_log_file.write(
            f'  "session_name": {json.dumps(os.path.basename(self._recording_session_dir))},\n'
        )
        self._recording_log_file.write(f'  "volume_node_name": {json.dumps(VOLUME_NODE_NAME)},\n')
        self._recording_log_file.write(f'  "capture_interval_ms": {int(self.volumeIntervalSpin.value)},\n')
        self._recording_log_file.write('  "slice_projection_range": "all",\n')
        self._recording_log_file.write(
            f'  "slice_projection_mode": {json.dumps(self.volumeProjectionModeCombo.currentText)},\n'
        )
        self._recording_log_file.write('  "frames": [')
        self._recording_log_footer_pos = self._recording_log_file.tell()
        self._recording_log_file.write("\n  ]\n}\n")
        self._recording_log_file.flush()

    def append_recording_log_entry(self, entry):
        if self._recording_log_file is None or self._recording_log_footer_pos is None:
            return
        self._recording_log_file.seek(self._recording_log_footer_pos)
        if self._recording_log_frames_written > 0:
            self._recording_log_file.write(",")
        entry_text = json.dumps(entry, indent=4)
        self._recording_log_file.write("\n")
        self._recording_log_file.write("\n".join("    " + line for line in entry_text.splitlines()))
        self._recording_log_footer_pos = self._recording_log_file.tell()
        self._recording_log_file.write("\n  ]\n}\n")
        self._recording_log_file.truncate()
        self._recording_log_file.flush()
        self._recording_log_frames_written += 1

    def close_recording_log(self):
        if self._recording_log_file is None:
            return
        self._recording_log_file.flush()
        self._recording_log_file.close()
        self._recording_log_file = None
        self._recording_log_footer_pos = None

    def current_encoder_log(self, frame_shape):
        predicted_tip = None
        predicted_line = None
        try:
            volume_node = slicer.util.getNode(VOLUME_NODE_NAME)
            tip_ras, base_ras = self.needle_tip_base_world_ras()
            tip_px = self.ras_to_frame_px(volume_node, tip_ras, frame_shape)
            base_px = self.ras_to_frame_px(volume_node, base_ras, frame_shape)
            predicted_tip = [int(round(tip_px[0])), int(round(tip_px[1]))]
            predicted_line = [
                int(round(tip_px[0])),
                int(round(tip_px[1])),
                int(round(base_px[0])),
                int(round(base_px[1])),
            ]
        except Exception:
            pass

        return {
            "theta_deg": round(float(self._enc_theta), 6),
            "d_mm": round(float(self._enc_d), 6),
            "predicted_tip": predicted_tip,
            "predicted_line": predicted_line,
        }

    def needle_tip_base_world_ras(self):
        transform = vtk.vtkGeneralTransform()
        slicer.vtkMRMLTransformNode.GetTransformBetweenNodes(encoderTransformNode, None, transform)
        tip_ras = transform.TransformPoint(0.0, 0.0, 0.0)
        base_ras = transform.TransformPoint(0.0, 0.0, -float(L))
        return tip_ras, base_ras

    def ras_to_frame_px(self, volume_node, ras, frame_shape):
        height, width = frame_shape[:2]
        local_ras = world_ras_to_node_local_ras(ras, volume_node)
        ras_to_ijk = vtk.vtkMatrix4x4()
        volume_node.GetRASToIJKMatrix(ras_to_ijk)
        ijk_h = [0.0, 0.0, 0.0, 1.0]
        ras_to_ijk.MultiplyPoint([float(local_ras[0]), float(local_ras[1]), float(local_ras[2]), 1.0], ijk_h)
        i = ijk_h[0] / ijk_h[3] if ijk_h[3] != 0.0 else ijk_h[0]
        j = ijk_h[1] / ijk_h[3] if ijk_h[3] != 0.0 else ijk_h[1]
        return finite_or_none((width - 1) - i), finite_or_none((height - 1) - j)

    def projection_settings(self):
        mode = self.volumeProjectionModeCombo.currentText
        if mode not in ("Max", "Mean"):
            mode = VOLUME_PROJECTION_MODE
        return mode

    def project_volume_slices(self, arr):
        if arr.ndim == 2:
            return arr, "single 2D frame"
        if arr.ndim != 3:
            raise RuntimeError(f"Unsupported volume numpy shape: {arr.shape}")

        depth = arr.shape[0]
        mode = self.projection_settings()
        slab = arr

        if mode == "Mean":
            frame = np.mean(slab.astype(np.float32, copy=False), axis=0)
        else:
            frame = np.max(slab, axis=0)

        return frame, f"{mode} projection all z=0:{depth}"

    def capture_volume_numpy(self):
        try:
            volume_node = slicer.util.getNode(VOLUME_NODE_NAME)
        except slicer.util.MRMLNodeNotFoundException:
            raise RuntimeError(f"Volume node not found: {VOLUME_NODE_NAME}")

        arr = slicer.util.arrayFromVolume(volume_node)
        frame, projection_info = self.project_volume_slices(arr)

        frame = np.flipud(np.fliplr(frame))
        return np.ascontiguousarray(frame), arr.shape, str(arr.dtype), projection_info

    def numpy_frame_to_qimage(self, frame):
        gray = normalize_to_uint8(frame)
        height, width = gray.shape
        rgba = np.empty((height, width, 4), dtype=np.uint8)
        rgba[:, :, 0] = gray
        rgba[:, :, 1] = gray
        rgba[:, :, 2] = gray
        rgba[:, :, 3] = 255
        return qt.QImage(
            rgba.tobytes(),
            width,
            height,
            qt.QImage.Format_RGBA8888,
        ).copy()

    def update_volume_preview(self):
        try:
            frame, volume_shape, volume_dtype, projection_info = self.capture_volume_numpy()
        except Exception as exc:
            self.volumeInfoLabel.setText(str(exc))
            self._volume_timer.stop()
            return

        now = time.perf_counter()
        if self._volume_fps_window_start is None:
            self._volume_fps_window_start = now
        self._volume_fps_window_frames += 1
        elapsed = now - self._volume_fps_window_start
        if elapsed >= 1.0:
            self._volume_fps = self._volume_fps_window_frames / elapsed
            self._volume_fps_window_start = now
            self._volume_fps_window_frames = 0

        globals()["latest_image_reference_frame_numpy"] = frame
        recorded_path = self.write_video_frame(frame)

        image = self.numpy_frame_to_qimage(frame)
        pixmap = qt.QPixmap.fromImage(image)
        self.volumePreviewLabel.setPixmap(
            pixmap.scaled(
                self.volumePreviewLabel.width,
                self.volumePreviewLabel.height,
                qt.Qt.KeepAspectRatio,
                qt.Qt.SmoothTransformation,
            )
        )
        self.volumeInfoLabel.setText(
            f"volume numpy: frame_shape={frame.shape}, frame_dtype={frame.dtype}, "
            f"volume_shape={volume_shape}, volume_dtype={volume_dtype}, "
            f"{projection_info}, min={frame.min()}, max={frame.max()} | FPS {self._volume_fps:.1f}"
        )
        if recorded_path:
            self.volumeInfoLabel.setText(self.volumeInfoLabel.text + f" | REC {os.path.basename(recorded_path)}")
        self._volume_frame_index += 1

    def on_theta_state(self, caller=None, event=None):
        try:
            self._enc_theta = float(subTheta.GetLastMessage())
        except Exception:
            return
        self.update_encoder_transform()

    def on_d_state(self, caller=None, event=None):
        try:
            d_cmd = float(subLinear.GetLastMessage())
            self._enc_d = cmd_d_to_ui_d(d_cmd)
        except Exception:
            return
        self.update_encoder_transform()

    def update_command_transform(self):
        x, y, angle = compute_pose(self._cmd_theta, self._cmd_d)
        apply_transform(transformNode, x, y, angle)
        rx, ry, rz = motion_xy_to_ras(x, y)
        self.currentPosLabel.setText(f"Cmd tip (RAS) [mm]: ({rx:.3f}, {ry:.3f}, {rz:.3f})")

    def update_encoder_transform(self):
        x, y, angle = compute_pose(self._enc_theta, self._enc_d)
        apply_transform(encoderTransformNode, x, y, angle)
        rx, ry, rz = motion_xy_to_ras(x, y)
        self.encoderPosLabel.setText(f"Encoder tip (RAS) [mm]: ({rx:.3f}, {ry:.3f}, {rz:.3f})")

    def set_preview_visibility(self, visible):
        display = previewNeedleModelNode.GetDisplayNode()
        if not display:
            return
        display.SetVisibility(bool(visible))
        if hasattr(display, "SetVisibility2D"):
            display.SetVisibility2D(bool(visible))
        if hasattr(display, "SetVisibility3D"):
            display.SetVisibility3D(bool(visible))

    def update_preview_transform(self, theta, d):
        x, y, angle = compute_pose(theta, d)
        apply_transform(previewTransformNode, x, y, angle)
        self.set_preview_visibility(True)

    def install_slice_click_observers(self):
        layout_manager = slicer.app.layoutManager()
        if layout_manager is None:
            return

        for view_name in CLICK_SLICE_VIEWS:
            slice_widget = layout_manager.sliceWidget(view_name)
            if slice_widget is None:
                continue
            interactor = slice_widget.sliceView().interactor()
            observer_id = interactor.AddObserver(
                "LeftButtonPressEvent",
                lambda caller, event, vn=view_name: self.on_slice_click(caller, event, vn),
            )
            self._slice_click_observers.append((interactor, observer_id))

    def on_slice_click(self, caller, event, view_name):
        if self._preview_move_running:
            self.previewStatusLabel.setText("Preview: click ignored during Move")
            return

        layout_manager = slicer.app.layoutManager()
        if layout_manager is None:
            return
        slice_widget = layout_manager.sliceWidget(view_name)
        if slice_widget is None:
            return

        x, y = caller.GetEventPosition()
        xy_to_ras = slice_widget.sliceLogic().GetSliceNode().GetXYToRAS()
        ras_h = [0.0, 0.0, 0.0, 1.0]
        xy_to_ras.MultiplyPoint([float(x), float(y), 0.0, 1.0], ras_h)
        if ras_h[3] != 0.0:
            ras = (ras_h[0] / ras_h[3], ras_h[1] / ras_h[3], ras_h[2] / ras_h[3])
        else:
            ras = (ras_h[0], ras_h[1], ras_h[2])

        print(
            f"[NeedleSlicerClosedLoop] {view_name} click (RAS) [mm]: "
            f"({ras[0]:.3f}, {ras[1]:.3f}, {ras[2]:.3f})"
        )
        self.clickLabel.setText(
            f"Last click (RAS) [mm]: {view_name} "
            f"({ras[0]:.3f}, {ras[1]:.3f}, {ras[2]:.3f})"
        )
        clickPointNode.RemoveAllControlPoints()
        clickPointNode.AddControlPoint(vtk.vtkVector3d(ras[0], ras[1], ras[2]))

        ik_ras = world_ras_to_node_local_ras(ras, boundaryCurveNode)
        mx, my = ras_to_motion_xy(ik_ras)
        target = solve_theta_d_from_xy(
            mx, my,
            float(THETA_MIN_DEG), float(THETA_MAX_DEG),
            float(D_MIN_MM), float(D_MAX_MM),
        )
        if target is None:
            self.ikLabel.setText("Target from click: out of range")
            print("[NeedleSlicerClosedLoop] target from click: out of range")
            return

        theta_deg, d_mm = target
        self._preview_has_target = True
        self._preview_target_theta = theta_deg
        self._preview_target_d = d_mm
        self.update_preview_transform(theta_deg, d_mm)
        self.moveButton.setEnabled(True)
        self.ikLabel.setText(f"Target from click: theta={theta_deg:.3f} [deg], d={d_mm:.3f} [mm]")
        self.previewStatusLabel.setText(f"Preview: theta={theta_deg:.3f} [deg], d={d_mm:.3f} [mm]")
        print(f"[NeedleSlicerClosedLoop] preview target: theta={theta_deg:.3f} [deg], d={d_mm:.3f} [mm]")


# Prevent duplicate windows when script is executed multiple times.
for _old_widget_name in (
    "preview_move_capture_widget",
    "volume_capture_and_needle_move_widget",
    "rl_dataset_widget",
):
    _old_widget = globals().get(_old_widget_name)
    if _old_widget:
        try:
            _old_widget.cleanup()
            _old_widget.close()
        except Exception:
            pass

preview_move_capture_widget = NeedleSlicerClosedLoop()
preview_move_capture_widget.show()
