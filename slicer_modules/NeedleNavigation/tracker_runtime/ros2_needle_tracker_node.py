# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

"""ROS 2 adapter for the replaceable SmallUNet needle-tip detector.

This process is started and stopped by the Needle Navigation Slicer module, but
it is intentionally outside the Slicer process.  It subscribes only to an
ultrasound image and encoder-derived pixel geometry.  It publishes a numeric
detection result and a mono8 preview; it never publishes a motor command.

The detector internally uses a canonical 512x512 coordinate system because the
trained SmallUNet and its priors were developed in that space.  Incoming native
pixel coordinates are scaled to 512 independently along X and Y.  The preview is
mapped back to the input width and height so its aspect ratio is unchanged.
"""

import argparse
import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# ``run_tracker.sh`` executes this file directly, while tests may import it as a
# package module.  Support both contexts without modifying sys.path globally.
try:
    from .needle_realtime_tracker import (
        CANONICAL_SIZE,
        EncoderState,
        NeedleRealtimeTracker,
    )
except ImportError:
    from needle_realtime_tracker import (
        CANONICAL_SIZE,
        EncoderState,
        NeedleRealtimeTracker,
    )


DEFAULT_IMAGE_TOPIC = "/ultrasound/projection/max"
DEFAULT_ENCODER_GEOMETRY_TOPIC = "/needle/encoder/geometry_px"
DEFAULT_RESULT_TOPIC = "/needle/tracking/result_px"
DEFAULT_OVERLAY_TOPIC = "/needle/tracking/overlay"


def finite_or_none(value):
    """Return a finite float, or None for NaN/Inf/missing geometry."""
    value = float(value)
    return value if math.isfinite(value) else None


def scale_pixel(value, source_size):
    """Map one native pixel coordinate into the 512-pixel canonical axis."""
    value = finite_or_none(value)
    source_size = finite_or_none(source_size)
    if value is None or source_size is None or source_size <= 0.0:
        return None
    return value * CANONICAL_SIZE / source_size


def encoder_state_from_geometry(values):
    """Convert Slicer [frame, tip x/y, base x/y, width, height] to 512px coordinates."""
    if len(values) < 7:
        raise ValueError(f"encoder geometry needs 7 values, received {len(values)}")

    _, tip_x, tip_y, base_x, base_y, width, height = values[:7]
    return EncoderState(
        tip_x=scale_pixel(tip_x, width),
        tip_y=scale_pixel(tip_y, height),
        line_x1=scale_pixel(tip_x, width),
        line_y1=scale_pixel(tip_y, height),
        line_x2=scale_pixel(base_x, width),
        line_y2=scale_pixel(base_y, height),
    )


class NeedleTrackerNode(Node):
    """Join Slicer image/geometry topics to the pure Python tracking algorithm."""

    def __init__(self, args):
        super().__init__("needle_tracker_node")
        self.bridge = CvBridge()
        self.tracker = NeedleRealtimeTracker(args.model, device=args.device)
        self.latest_encoder_state = EncoderState()
        self.latest_encoder_frame_index = -1
        self.last_encoder_time = 0.0
        self.max_encoder_age_sec = args.max_encoder_age_sec
        self.display = args.display
        self.final_only = args.final_only
        self.image_sequence = 0

        # The Slicer publisher may be RELIABLE, but BEST_EFFORT is intentional for
        # live images: keeping only the newest frame prevents inference backlog.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = self.create_subscription(
            Image, args.image_topic, self.on_image, image_qos
        )
        self.encoder_geometry_sub = self.create_subscription(
            Float64MultiArray,
            args.encoder_geometry_topic,
            self.on_encoder_geometry,
            10,
        )
        self.result_pub = self.create_publisher(
            Float64MultiArray, args.output_result_topic, 10
        )
        self.overlay_pub = self.create_publisher(Image, args.output_overlay_topic, 10)
        self.get_logger().info(
            "Needle tracker ready: "
            f"image={args.image_topic}, encoder={args.encoder_geometry_topic}, "
            f"result={args.output_result_topic}, overlay={args.output_overlay_topic}"
        )

    def on_encoder_geometry(self, msg: Float64MultiArray):
        """Cache the newest encoder prior and its arrival time for staleness checks."""
        try:
            values = [float(value) for value in msg.data]
            self.latest_encoder_state = encoder_state_from_geometry(values)
            self.latest_encoder_frame_index = int(round(values[0]))
            self.last_encoder_time = time.monotonic()
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid encoder geometry: {exc}")

    def on_image(self, msg: Image):
        """Run one inference update and publish result plus display-only preview."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        self.image_sequence += 1
        # An old encoder prior is more dangerous than no prior.  Reset to an empty
        # state when geometry has not arrived within the configured age limit.
        encoder_state = self.latest_encoder_state
        if time.monotonic() - self.last_encoder_time > self.max_encoder_age_sec:
            encoder_state = EncoderState()

        result = self.tracker.update(frame, encoder_state)
        self.publish_result(result)

        overlay_bgr = self.tracker.draw_overlay(
            frame, result, final_only=self.final_only
        )
        # The installed SlicerROS2 UInt8Image converter supports mono8 only.  Slicer
        # recolors the final tip red from the separate numeric result topic.
        overlay_gray = cv2.cvtColor(overlay_bgr, cv2.COLOR_BGR2GRAY)
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay_gray, encoding="mono8")
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)

        if self.display:
            cv2.imshow("Needle Tracking ROS2", overlay_bgr)
            cv2.waitKey(1)

    def publish_result(self, result):
        """Publish the stable eight-value Float64MultiArray contract.

        A one-dimensional layout entry is required by the SlicerROS2 DoubleArray
        converter.  Tip coordinates remain in canonical 512 space; consumers can
        map them to native pixels using x*width/512 and y*height/512.
        """
        values = Float64MultiArray()
        values.layout.dim = [
            MultiArrayDimension(label="tracking_result", size=8, stride=1)
        ]
        values.data = [
            float(self.image_sequence),
            float(self.latest_encoder_frame_index),
            1.0 if result.detected else 0.0,
            float(result.final_x) if result.final_x is not None else math.nan,
            float(result.final_y) if result.final_y is not None else math.nan,
            float(result.cnn_value) if result.cnn_value is not None else math.nan,
            1.0 if result.used_kalman else 0.0,
            float(result.visible_run_count),
        ]
        self.result_pub.publish(values)

    def destroy_node(self):
        if self.display:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main():
    """Parse ROS-facing configuration and spin until SIGINT/SIGTERM shutdown."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--device", default="auto", choices=["auto", "cuda", "cpu"])
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument(
        "--encoder-geometry-topic", default=DEFAULT_ENCODER_GEOMETRY_TOPIC
    )
    parser.add_argument("--output-result-topic", default=DEFAULT_RESULT_TOPIC)
    parser.add_argument("--output-overlay-topic", default=DEFAULT_OVERLAY_TOPIC)
    parser.add_argument("--max-encoder-age-sec", type=float, default=0.25)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--final-only", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = NeedleTrackerNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
