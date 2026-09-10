# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

"""Needle Navigation scripted module for 3D Slicer.

The module deliberately separates three responsibilities:

1. The existing ``preview_move_capture.py`` widget remains responsible for manual
   motor controls, encoder-driven MRML transforms, click targets, and recording.
2. This file publishes the live ultrasound maximum projection and encoder-derived
   needle geometry to ROS 2, manages the detector process, and renders its result.
3. ``tracker_runtime/ros2_needle_tracker_node.py`` performs inference outside the
   Slicer process so a CUDA/PyTorch failure cannot take down the Slicer UI.

The detector never publishes a motor command.  Its numeric result and preview are
visualization-only inputs to Slicer.  Red shows the original projection with the
navigation overlays; Yellow shows only the ultrasound image and the final red tip
cross.  Both views share the exact same SliceToRAS matrix so they cannot drift by a
90-degree rotation.

Coordinate conventions are important here.  The legacy preview flips the NumPy
projection in both axes before sending it to OpenCV.  ROS detector coordinates are
therefore expressed in that flipped image.  Before creating an MRML volume, this
module reverses the flip and copies the source volume's IJK-to-RAS matrix and parent
transform.  This keeps detector pixels aligned with the calibrated Slicer scene.
"""

import __main__
import math
import pathlib
import time
import traceback

import ctk
import numpy as np
import qt
import slicer
import vtk
from slicer.ScriptedLoadableModule import ScriptedLoadableModule
from slicer.ScriptedLoadableModule import ScriptedLoadableModuleWidget
from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy


MODULE_TITLE = "Needle Navigation"

# ROS interface shared with the out-of-process detector.  The Float64 arrays are
# used because the installed SlicerROS2 version exposes these converters directly.
PROJECTION_TOPIC = "/ultrasound/projection/max"
ENCODER_GEOMETRY_TOPIC = "/needle/encoder/geometry_px"
TRACKING_RESULT_TOPIC = "/needle/tracking/result_px"
TRACKING_OVERLAY_TOPIC = "/needle/tracking/overlay"

# The GitHub repository uses the correctly spelled lower-case filename.  The
# deployed laboratory copy may retain an older misspelling, but repository code
# should always resolve relative to the repository root and never to a user path.
LEGACY_SCRIPT_RELATIVE_PATH = pathlib.Path(
    "src/ros2_epos_cmd/scripts/preview_move_capture.py"
)
LEGACY_WINDOW_MARKER = "# Prevent duplicate windows when script is executed multiple times."


class NeedleNavigation(ScriptedLoadableModule):
    """Slicer module metadata."""

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = MODULE_TITLE
        self.parent.categories = ["IGT"]
        self.parent.dependencies = ["ROS2", "OpenIGTLinkIF"]
        self.parent.contributors = ["Needle Navigation project"]
        self.parent.helpText = (
            "Embedded needle motion, volume preview, recording, and ROS 2 maximum-intensity "
            "projection publishing. CAN setup, homing, and the EPOS ROS 2 bridge are started "
            "outside Slicer."
        )
        self.parent.acknowledgementText = (
            "This compatibility module reuses the existing Preview Move Capture implementation "
            "without modifying its source file."
        )


class NeedleNavigationWidget(ScriptedLoadableModuleWidget):
    """Own the embedded control widget and the perception visualization pipeline.

    Slicer creates one widget instance and calls ``setup`` once.  Expensive or
    hardware-affecting legacy initialization is deferred until ``enter`` so merely
    discovering the module does not publish commands.  Timers, VTK observers, ROS
    MRML nodes, and the child detector process are all explicitly released in
    ``cleanup`` because PythonQt parent ownership is not reliable for QProcess.
    """

    def __init__(self, parent=None):
        ScriptedLoadableModuleWidget.__init__(self, parent)

        # Compatibility/legacy UI state.
        self._initialized = False
        self._initialization_error = None
        self._legacy_namespace = None
        self._legacy_widget = None
        self._ros_node = None
        self._projection_publisher = None

        # Detector ROS interfaces.  Geometry and images travel to the external
        # process; result arrays and a mono8 preview travel back to Slicer.
        self._encoder_geometry_publisher = None
        self._tracking_result_subscriber = None
        self._tracking_overlay_subscriber = None
        self._tracking_result_observer = None
        self._tracking_overlay_observer = None
        self._tracker_process = None

        # MRML nodes and most recent frames used by the two-Slice presentation.
        self._source_projection_volume = None
        self._tracking_overlay_volume = None
        self._latest_projection = None
        self._latest_tracking_gray = None
        self._latest_tracking_point_512 = None
        self._slice_display_enabled = False
        self._slice_layout_configured = False

        # Timers, observers, and counters used for communication-rate diagnostics.
        self._projection_timer = None
        self._metrics_timer = None
        self._theta_subscriber = None
        self._linear_subscriber = None
        self._input_volume_node = None
        self._theta_rate_observer = None
        self._linear_rate_observer = None
        self._input_volume_rate_observer = None
        self._theta_state_count = 0
        self._linear_state_count = 0
        self._input_volume_update_count = 0
        self._projection_sent_bytes = 0
        self._last_subscriber_count = 0
        self._metrics_window_start_sec = time.monotonic()
        self._last_metrics_theta_count = 0
        self._last_metrics_linear_count = 0
        self._last_metrics_volume_count = 0
        self._last_metrics_image_count = 0
        self._last_metrics_image_bytes = 0
        self._last_volume_frame_index = -1
        self._projection_attempt_count = 0
        self._projection_sent_count = 0
        self._tracking_result_count = 0
        self._last_metrics_tracking_count = 0
        self._last_status_update_sec = 0.0

    def setup(self):
        """Build controls only; ROS and motor-related legacy code starts in enter()."""
        ScriptedLoadableModuleWidget.setup(self)

        projection_panel = ctk.ctkCollapsibleButton()
        projection_panel.text = "ROS 2 Ultrasound Projection"
        projection_panel.collapsed = False
        projection_form = qt.QFormLayout(projection_panel)

        self.inputVolumeSelector = slicer.qMRMLNodeComboBox()
        self.inputVolumeSelector.nodeTypes = ["vtkMRMLScalarVolumeNode"]
        self.inputVolumeSelector.selectNodeUponCreation = False
        self.inputVolumeSelector.addEnabled = False
        self.inputVolumeSelector.removeEnabled = False
        self.inputVolumeSelector.renameEnabled = False
        self.inputVolumeSelector.noneEnabled = True
        self.inputVolumeSelector.showHidden = False
        self.inputVolumeSelector.showChildNodeTypes = True
        self.inputVolumeSelector.setMRMLScene(slicer.mrmlScene)
        self.inputVolumeSelector.currentNodeChanged.connect(self._on_input_volume_changed)

        self.projectionTopicLabel = qt.QLabel(PROJECTION_TOPIC)
        self.projectionTopicLabel.setTextInteractionFlags(qt.Qt.TextSelectableByMouse)

        self.projectionPublishCheckBox = qt.QCheckBox("Publish full-resolution Max projection")
        self.projectionPublishCheckBox.checked = True
        self.projectionPublishCheckBox.toggled.connect(self._on_projection_publish_toggled)

        self.projectionStatusLabel = qt.QLabel(
            "Select this module to initialize the existing control interface."
        )
        self.projectionStatusLabel.wordWrap = True
        self.projectionStatusLabel.setSizePolicy(
            qt.QSizePolicy.Ignored,
            qt.QSizePolicy.Preferred,
        )

        self.communicationStatusLabel = qt.QLabel(
            "US OpenIGTLink RX: - Hz | Image ROS2 TX: - Hz | Motor ROS2 state RX: - Hz"
        )
        self.communicationStatusLabel.wordWrap = True
        self.communicationStatusLabel.setSizePolicy(
            qt.QSizePolicy.Ignored,
            qt.QSizePolicy.Preferred,
        )

        self.compatibilityWarningLabel = qt.QLabel(
            "Compatibility mode: selecting this module initializes the existing "
            "preview_move_capture behavior. If the EPOS bridge is already running, "
            "the legacy initialization may publish theta=0 and d=0 commands."
        )
        self.compatibilityWarningLabel.wordWrap = True
        self.compatibilityWarningLabel.setStyleSheet("color: #b45309;")

        projection_form.addRow("Input ultrasound volume", self.inputVolumeSelector)
        projection_form.addRow("Topic", self.projectionTopicLabel)
        projection_form.addRow("", self.projectionPublishCheckBox)
        projection_form.addRow("Status", self.projectionStatusLabel)
        projection_form.addRow("Rates", self.communicationStatusLabel)
        projection_form.addRow("", self.compatibilityWarningLabel)
        self.layout.addWidget(projection_panel)

        tracking_panel = ctk.ctkCollapsibleButton()
        tracking_panel.text = "Needle Tracking"
        tracking_panel.collapsed = False
        tracking_form = qt.QFormLayout(tracking_panel)

        self.encoderGeometryTopicLabel = qt.QLabel(ENCODER_GEOMETRY_TOPIC)
        self.trackingResultTopicLabel = qt.QLabel(TRACKING_RESULT_TOPIC)
        self.trackingStatusLabel = qt.QLabel("Tracker not connected")
        self.trackingStatusLabel.wordWrap = True
        self.trackerProcessStatusLabel = qt.QLabel("Tracker process: stopped")
        self.trackerProcessStatusLabel.wordWrap = True
        self.startTrackerButton = qt.QPushButton("Start Tracker")
        self.stopTrackerButton = qt.QPushButton("Stop Tracker")
        self.stopTrackerButton.enabled = False
        self.showTrackingSliceButton = qt.QPushButton(
            "Show Original + Detection in Slices"
        )
        tracker_buttons = qt.QHBoxLayout()
        tracker_buttons.addWidget(self.startTrackerButton)
        tracker_buttons.addWidget(self.stopTrackerButton)

        self.startTrackerButton.clicked.connect(self._start_tracker_process)
        self.stopTrackerButton.clicked.connect(self._stop_tracker_process)
        self.showTrackingSliceButton.clicked.connect(self._show_tracking_in_slice_views)

        tracking_form.addRow("Encoder geometry TX", self.encoderGeometryTopicLabel)
        tracking_form.addRow("Tracking result RX", self.trackingResultTopicLabel)
        tracking_form.addRow("Process", self.trackerProcessStatusLabel)
        tracking_form.addRow("Status", self.trackingStatusLabel)
        tracking_form.addRow("", tracker_buttons)
        tracking_form.addRow("", self.showTrackingSliceButton)
        self.layout.addWidget(tracking_panel)

        self.legacyContainer = qt.QWidget()
        self.legacyContainerLayout = qt.QVBoxLayout(self.legacyContainer)
        self.legacyContainerLayout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.legacyContainer)
        self.layout.addStretch(1)

    def enter(self):
        """Initialize the legacy control and ROS interfaces on first module entry.

        The guard intentionally makes this idempotent.  Re-entering the module must
        not create duplicate publishers, timers, observers, or control widgets.
        """
        if self._initialized or self._initialization_error is not None:
            return
        try:
            self._initialize_compatibility_module()
        except Exception as exc:
            self._initialization_error = exc
            details = traceback.format_exc()
            self.projectionStatusLabel.setText(f"Initialization failed: {exc}")
            slicer.util.errorDisplay(
                f"{MODULE_TITLE} initialization failed.\n\n{exc}\n\n{details}"
            )

    def exit(self):
        # Keep the legacy timers and ROS subscriptions alive when another Slicer module is
        # temporarily selected. This matches the previous standalone-window behavior.
        pass

    def cleanup(self):
        """Release every resource created by this scripted module instance."""
        if self._projection_timer is not None:
            self._projection_timer.stop()
            self._projection_timer = None

        if self._metrics_timer is not None:
            self._metrics_timer.stop()
            self._metrics_timer = None

        self._remove_communication_rate_observers()
        self._set_input_volume_rate_observer(None)
        self._remove_perception_interfaces()
        self._stop_tracker_process()

        if self._legacy_widget is not None:
            try:
                self._legacy_widget.cleanup()
            except Exception:
                pass
            self._legacy_widget = None

        self._remove_projection_publisher()
        self._legacy_namespace = None
        self._initialized = False

    def _hello_epos_root(self):
        # .../<repository>/slicer_modules/NeedleNavigation/NeedleNavigation.py
        return pathlib.Path(__file__).resolve().parents[2]

    def _legacy_script_path(self):
        return self._hello_epos_root() / LEGACY_SCRIPT_RELATIVE_PATH

    def _close_interactor_widgets(self):
        # If one of the old scripts is still open from the Python Interactor, stop it before
        # creating the embedded copy. This prevents duplicate timers and ROS publishers.
        for widget_name in (
            "preview_move_capture_widget",
            "volume_capture_and_needle_move_widget",
            "rl_dataset_widget",
        ):
            old_widget = getattr(__main__, widget_name, None)
            if old_widget is None:
                continue
            try:
                old_widget.cleanup()
                old_widget.close()
            except Exception:
                pass
            try:
                delattr(__main__, widget_name)
            except Exception:
                pass

    def _load_legacy_namespace(self):
        """Load legacy class/function definitions without opening its top-level window.

        The historical script contains both reusable definitions and executable UI
        startup code in one file.  The marker is a deliberate boundary: executing
        only the prefix gives this module the existing control class while preventing
        a second independent window and a second set of motor publishers.
        """
        script_path = self._legacy_script_path()
        if not script_path.is_file():
            raise RuntimeError(f"Legacy script not found: {script_path}")

        source = script_path.read_text(encoding="utf-8")
        marker_index = source.find(LEGACY_WINDOW_MARKER)
        if marker_index < 0:
            raise RuntimeError(
                "The legacy script window marker was not found; refusing to execute the "
                "script because it may create a separate top-level window."
            )

        source_without_window = source[:marker_index]
        namespace = {
            "__file__": str(script_path),
            "__name__": "needle_navigation_legacy_runtime",
        }
        exec(
            compile(source_without_window, str(script_path), "exec"),
            namespace,
            namespace,
        )
        return namespace

    def _initialize_compatibility_module(self):
        """Embed the legacy widget and attach all ROS perception interfaces."""
        self.projectionStatusLabel.setText("Initializing existing Preview Move Capture logic...")
        slicer.app.processEvents()

        self._close_interactor_widgets()
        self._legacy_namespace = self._load_legacy_namespace()
        self._select_initial_input_volume()
        selected_volume = self.inputVolumeSelector.currentNode()
        if selected_volume is not None:
            self._legacy_namespace["VOLUME_NODE_NAME"] = selected_volume.GetName()

        widget_class = self._legacy_namespace.get("NeedleSlicerClosedLoop")
        if widget_class is None:
            raise RuntimeError("NeedleSlicerClosedLoop class was not defined by the legacy script")

        self._legacy_widget = widget_class(self.legacyContainer)
        self._legacy_widget.setWindowFlags(qt.Qt.Widget)
        self._configure_legacy_widget_size_policy()
        self.legacyContainerLayout.addWidget(self._legacy_widget)
        self._legacy_widget.show()

        self._ros_node = self._legacy_namespace.get("rosNode")
        if self._ros_node is None:
            raise RuntimeError("The SlicerROS2 default node is unavailable")

        self._create_projection_publisher()
        self._create_perception_interfaces()
        self._start_communication_metrics()
        self._start_projection_timer()
        self._initialized = True
        volume_name = (
            selected_volume.GetName()
            if selected_volume is not None
            else "a selected volume"
        )
        self.projectionStatusLabel.setText(
            f"Ready; waiting for {volume_name} frames on {PROJECTION_TOPIC}"
        )

    def _configure_legacy_widget_size_policy(self):
        """Allow the embedded preview to shrink with Slicer's module panel."""
        self._legacy_widget.setMinimumWidth(0)
        self.legacyContainer.setMinimumWidth(0)

        preview_label = getattr(self._legacy_widget, "volumePreviewLabel", None)
        if preview_label is not None:
            preview_label.setMinimumWidth(0)
            preview_label.setSizePolicy(
                qt.QSizePolicy.Ignored,
                qt.QSizePolicy.Preferred,
            )

        info_label = getattr(self._legacy_widget, "volumeInfoLabel", None)
        if info_label is not None:
            info_label.setWordWrap(True)
            info_label.setSizePolicy(
                qt.QSizePolicy.Ignored,
                qt.QSizePolicy.Preferred,
            )

    def _create_projection_publisher(self):
        """Create the SlicerROS2 mono8 image publisher used by the detector."""
        self._remove_projection_publisher()
        self._projection_publisher = self._ros_node.CreateAndAddPublisherNode(
            "UInt8Image",
            PROJECTION_TOPIC,
        )
        if self._projection_publisher is None:
            raise RuntimeError(f"Failed to create UInt8Image publisher for {PROJECTION_TOPIC}")

    def _create_perception_interfaces(self):
        """Create geometry TX plus detector result/preview RX MRML nodes.

        SlicerROS2 represents ROS publishers and subscribers as MRML nodes.  VTK
        ModifiedEvent observers are therefore the bridge from incoming ROS messages
        back into Python callbacks on the Slicer application thread.
        """
        self._remove_perception_interfaces()
        self._encoder_geometry_publisher = self._ros_node.CreateAndAddPublisherNode(
            "DoubleArray",
            ENCODER_GEOMETRY_TOPIC,
        )
        self._tracking_result_subscriber = self._ros_node.CreateAndAddSubscriberNode(
            "DoubleArray",
            TRACKING_RESULT_TOPIC,
        )
        self._tracking_overlay_subscriber = self._ros_node.CreateAndAddSubscriberNode(
            "UInt8Image",
            TRACKING_OVERLAY_TOPIC,
        )
        if self._encoder_geometry_publisher is None:
            raise RuntimeError(
                f"Failed to create DoubleArray publisher for {ENCODER_GEOMETRY_TOPIC}"
            )
        if self._tracking_result_subscriber is None:
            raise RuntimeError(
                f"Failed to create DoubleArray subscriber for {TRACKING_RESULT_TOPIC}"
            )
        if self._tracking_overlay_subscriber is None:
            raise RuntimeError(
                f"Failed to create UInt8Image subscriber for {TRACKING_OVERLAY_TOPIC}"
            )
        self._tracking_result_observer = self._tracking_result_subscriber.AddObserver(
            "ModifiedEvent",
            self._on_tracking_result_received,
        )
        self._tracking_overlay_observer = self._tracking_overlay_subscriber.AddObserver(
            "ModifiedEvent",
            self._on_tracking_overlay_received,
        )

    def _remove_perception_interfaces(self):
        """Detach VTK observers before deleting their SlicerROS2 MRML nodes."""
        if (
            self._tracking_result_subscriber is not None
            and self._tracking_result_observer is not None
        ):
            self._tracking_result_subscriber.RemoveObserver(self._tracking_result_observer)
        if (
            self._tracking_overlay_subscriber is not None
            and self._tracking_overlay_observer is not None
        ):
            self._tracking_overlay_subscriber.RemoveObserver(self._tracking_overlay_observer)

        if self._ros_node is not None:
            for topic in (TRACKING_RESULT_TOPIC, TRACKING_OVERLAY_TOPIC):
                try:
                    self._ros_node.RemoveAndDeleteSubscriberNode(topic)
                except Exception:
                    pass
            try:
                self._ros_node.RemoveAndDeletePublisherNode(ENCODER_GEOMETRY_TOPIC)
            except Exception:
                pass

        self._encoder_geometry_publisher = None
        self._tracking_result_subscriber = None
        self._tracking_overlay_subscriber = None
        self._tracking_result_observer = None
        self._tracking_overlay_observer = None

    def _on_tracking_result_received(self, caller=None, event=None):
        """Decode the eight-value tracker result and cache the final 512-space tip."""
        try:
            message = self._tracking_result_subscriber.GetLastMessage()
            if message is None or message.GetNumberOfValues() < 8:
                return
            values = [float(message.GetValue(i)) for i in range(8)]
            image_sequence, encoder_frame, detected, tip_x, tip_y, cnn_value, used_kalman, warmup = values
            self._tracking_result_count += 1
            if detected >= 0.5 and math.isfinite(tip_x) and math.isfinite(tip_y):
                self._latest_tracking_point_512 = (tip_x, tip_y)
                self.trackingStatusLabel.setText(
                    f"DETECTED | tip512=({tip_x:.1f}, {tip_y:.1f}) | "
                    f"cnn={cnn_value:.3f} | kalman={bool(used_kalman)} | "
                    f"image={int(image_sequence)}, encoder={int(encoder_frame)}"
                )
            else:
                self._latest_tracking_point_512 = None
                self.trackingStatusLabel.setText(
                    f"Not detected / warmup | valid encoder frames={int(warmup)} | "
                    f"image={int(image_sequence)}, encoder={int(encoder_frame)}"
                )
        except Exception as exc:
            self.trackingStatusLabel.setText(f"Tracking result error: {exc}")

    def _on_tracking_overlay_received(self, caller=None, event=None):
        """Convert the mono8 ROS preview into the RGB Yellow Slice volume.

        The installed SlicerROS2 image converter does not retain ROS encoding or RGB
        channel metadata, so the transport intentionally stays mono8.  The final tip
        is colored red locally from the numeric result topic.
        """
        try:
            message = self._tracking_overlay_subscriber.GetLastMessage()
            if message is None:
                return
            height = int(message.GetNumberOfTuples())
            width = int(message.GetNumberOfComponents())
            if height <= 0 or width <= 0:
                return
            gray = np.asarray(vtk_to_numpy(message), dtype=np.uint8).reshape(height, width)
            self._latest_tracking_gray = np.ascontiguousarray(gray)
            if self._slice_display_enabled:
                self._tracking_overlay_volume = self._update_display_volume(
                    self._tracking_overlay_volume,
                    "NeedleTrackingOverlay",
                    self._tracker_frame_to_slicer_array(
                        self._tracking_overlay_rgb(gray)
                    ),
                    self.inputVolumeSelector.currentNode(),
                )
                if not self._slice_layout_configured:
                    self._configure_tracking_slice_layout()
        except Exception as exc:
            self.trackingStatusLabel.setText(f"Tracking overlay error: {exc}")

    def _tracker_runtime_dir(self):
        return pathlib.Path(__file__).resolve().parent / "tracker_runtime"

    def _start_tracker_process(self):
        """Launch the bundled ROS detector with final-result-only visualization.

        QProcess is intentionally parentless: PythonQt cannot use a scripted module
        wrapper as its QObject parent.  ``cleanup`` and ``_stop_tracker_process`` own
        termination explicitly.  The external node publishes no motor topics.
        """
        if self._tracker_process is not None and self._tracker_process.state() != qt.QProcess.NotRunning:
            self.trackerProcessStatusLabel.setText("Tracker process: already running")
            self._show_tracking_in_slice_views()
            return

        launcher = self._tracker_runtime_dir() / "run_tracker.sh"
        if not launcher.is_file():
            self.trackerProcessStatusLabel.setText(f"Tracker launcher missing: {launcher}")
            return

        # PythonQt cannot use a ScriptedLoadableModuleWidget wrapper as the QObject
        # parent of QProcess.  The process is owned and stopped explicitly by cleanup().
        self._tracker_process = qt.QProcess()
        self._tracker_process.setProcessChannelMode(qt.QProcess.MergedChannels)
        self._tracker_process.readyReadStandardOutput.connect(self._on_tracker_output)
        self._tracker_process.started.connect(self._on_tracker_started)
        self._tracker_process.finished.connect(self._on_tracker_finished)
        if hasattr(self._tracker_process, "errorOccurred"):
            self._tracker_process.errorOccurred.connect(self._on_tracker_error)

        # Yellow is a clean clinical view: source ultrasound plus final tip only.
        arguments = [str(launcher), "--final-only"]
        self._slice_display_enabled = True
        self._slice_layout_configured = False
        self.trackerProcessStatusLabel.setText("Tracker process: starting...")
        self.startTrackerButton.enabled = False
        self.stopTrackerButton.enabled = True
        self._tracker_process.start("/bin/bash", arguments)

    def _stop_tracker_process(self):
        """Gracefully terminate the detector, escalating to kill only after timeout."""
        process = self._tracker_process
        if process is None:
            return
        if process.state() != qt.QProcess.NotRunning:
            process.terminate()
            if not process.waitForFinished(3000):
                process.kill()
                process.waitForFinished(1000)
        if self._tracker_process is process:
            process.deleteLater()
            self._tracker_process = None
        if hasattr(self, "startTrackerButton"):
            self.startTrackerButton.enabled = True
            self.stopTrackerButton.enabled = False
            self.trackerProcessStatusLabel.setText("Tracker process: stopped")

    def _on_tracker_started(self):
        self.startTrackerButton.enabled = False
        self.stopTrackerButton.enabled = True
        self.trackerProcessStatusLabel.setText("Tracker process: running")

    def _on_tracker_output(self):
        try:
            data = bytes(self._tracker_process.readAllStandardOutput()).decode(
                "utf-8", errors="replace"
            )
        except Exception:
            return
        lines = [line.strip() for line in data.splitlines() if line.strip()]
        if lines:
            self.trackerProcessStatusLabel.setText(lines[-1])

    def _on_tracker_finished(self, *args):
        process = self._tracker_process
        exit_code = args[0] if args else (process.exitCode() if process else -1)
        self.startTrackerButton.enabled = True
        self.stopTrackerButton.enabled = False
        self.trackerProcessStatusLabel.setText(
            f"Tracker process: stopped (exit code {int(exit_code)})"
        )
        if process is not None:
            process.deleteLater()
        self._tracker_process = None

    def _on_tracker_error(self, *args):
        error_text = self._tracker_process.errorString() if self._tracker_process else "unknown error"
        self.trackerProcessStatusLabel.setText(f"Tracker process error: {error_text}")

    @staticmethod
    def _tracker_frame_to_slicer_array(frame):
        """Undo the legacy OpenCV flip before placing pixels in Slicer IJK space."""
        return np.ascontiguousarray(np.flipud(np.fliplr(frame)), dtype=np.uint8)

    def _tracking_overlay_rgb(self, gray):
        """Return RGB ultrasound with only the final detected tip drawn in red.

        ``point`` is in the detector's canonical 512x512 coordinates.  Scaling X and
        Y independently maps it back to the received native image without changing
        aspect ratio.  Drawing is implemented with NumPy because Slicer's bundled
        Python does not need an OpenCV dependency for visualization.
        """
        rgb = np.repeat(np.asarray(gray, dtype=np.uint8)[:, :, np.newaxis], 3, axis=2)
        point = self._latest_tracking_point_512
        if point is None:
            return np.ascontiguousarray(rgb)

        height, width = gray.shape
        center_x = int(round(float(point[0]) * width / 512.0))
        center_y = int(round(float(point[1]) * height / 512.0))
        arm_x = max(4, int(round(10.0 * width / 512.0)))
        arm_y = max(4, int(round(10.0 * height / 512.0)))
        thickness = max(2, int(round(min(width, height) / 256.0)))
        half_thickness = max(1, thickness // 2)

        # Draw the same diagonal cross used by the tracker, in true RGB red.
        for step in range(-max(arm_x, arm_y), max(arm_x, arm_y) + 1):
            x_offset = int(round(step * arm_x / max(arm_x, arm_y)))
            y_offset = int(round(step * arm_y / max(arm_x, arm_y)))
            for y_coord in (center_y + y_offset, center_y - y_offset):
                x_coord = center_x + x_offset
                x0 = max(0, x_coord - half_thickness)
                x1 = min(width, x_coord + half_thickness + 1)
                y0 = max(0, y_coord - half_thickness)
                y1 = min(height, y_coord + half_thickness + 1)
                if x0 < x1 and y0 < y1:
                    rgb[y0:y1, x0:x1] = (255, 0, 0)
        return np.ascontiguousarray(rgb)

    def _update_display_volume(
        self, volume_node, name, frame, reference_volume_node=None
    ):
        """Create/update a scalar or RGB volume while preserving source geometry.

        A default MRML volume uses identity orientation and unit spacing.  That would
        visually separate the detector result from calibrated boundaries even when
        the pixel arrays match.  Copying IJK-to-RAS and the parent transform keeps the
        derived projection in the same physical coordinate frame as the live image.
        """
        is_color = frame.ndim == 3 and frame.shape[2] in (3, 4)
        expected_class = (
            "vtkMRMLVectorVolumeNode" if is_color else "vtkMRMLScalarVolumeNode"
        )
        if volume_node is not None and not volume_node.IsA(expected_class):
            slicer.mrmlScene.RemoveNode(volume_node)
            volume_node = None
        if volume_node is None or volume_node.GetScene() is None:
            try:
                volume_node = slicer.util.getNode(name)
            except slicer.util.MRMLNodeNotFoundException:
                volume_node = None
            if volume_node is not None and not volume_node.IsA(expected_class):
                slicer.mrmlScene.RemoveNode(volume_node)
                volume_node = None
            if volume_node is None:
                volume_node = slicer.mrmlScene.AddNewNodeByClass(expected_class, name)
                volume_node.CreateDefaultDisplayNodes()
        volume_array = (
            frame[np.newaxis, :, :, :]
            if is_color
            else frame[np.newaxis, :, :]
        )
        slicer.util.updateVolumeFromArray(
            volume_node,
            np.ascontiguousarray(volume_array, dtype=np.uint8),
        )
        if reference_volume_node is not None:
            # Keep the derived 2D projection in exactly the same physical image
            # coordinate system as the live ultrasound volume.  This preserves
            # orientation, spacing, and alignment with motor boundaries/markups.
            ijk_to_ras = vtk.vtkMatrix4x4()
            reference_volume_node.GetIJKToRASMatrix(ijk_to_ras)
            volume_node.SetIJKToRASMatrix(ijk_to_ras)
            volume_node.SetAndObserveTransformNodeID(
                reference_volume_node.GetTransformNodeID()
            )
        display_node = volume_node.GetDisplayNode()
        if display_node is not None and not is_color:
            display_node.SetAutoWindowLevel(True)
        return volume_node

    def _show_tracking_in_slice_views(self):
        """Populate both derived volumes and restore the dedicated two-view layout."""
        if self._latest_projection is None or self._latest_tracking_gray is None:
            self.trackingStatusLabel.setText(
                "Original and tracking frames are not available yet; start the tracker first"
            )
            self._slice_display_enabled = True
            return

        self._slice_display_enabled = True
        self._source_projection_volume = self._update_display_volume(
            self._source_projection_volume,
            "NeedleUltrasoundProjection",
            self._tracker_frame_to_slicer_array(self._latest_projection),
            self.inputVolumeSelector.currentNode(),
        )
        self._tracking_overlay_volume = self._update_display_volume(
            self._tracking_overlay_volume,
            "NeedleTrackingOverlay",
            self._tracker_frame_to_slicer_array(
                self._tracking_overlay_rgb(self._latest_tracking_gray)
            ),
            self.inputVolumeSelector.currentNode(),
        )

        self._configure_tracking_slice_layout()

    def _configure_tracking_slice_layout(self):
        """Place original in Red and clean detection output in Yellow.

        Yellow must copy Red's exact SliceToRAS matrix.  Asking each Slice logic to
        independently select the closest volume axis can produce equally valid axes
        that differ by 90 degrees, which is confusing and breaks visual comparison.
        """
        if self._source_projection_volume is None or self._tracking_overlay_volume is None:
            return

        layout_manager = slicer.app.layoutManager()
        if layout_manager is None:
            return
        layout_manager.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutSideBySideView)
        slice_widgets = {}
        for view_name, volume_node, label in (
            ("Red", self._source_projection_volume, "Original"),
            ("Yellow", self._tracking_overlay_volume, "Detection"),
        ):
            slice_widget = layout_manager.sliceWidget(view_name)
            if slice_widget is None:
                continue
            slice_node = slice_widget.mrmlSliceNode()
            slice_node.SetLayoutLabel(label)
            composite_node = slice_widget.mrmlSliceCompositeNode()
            composite_node.SetBackgroundVolumeID(volume_node.GetID())
            composite_node.SetForegroundVolumeID(None)
            slice_widgets[view_name] = slice_widget

        red_widget = slice_widgets.get("Red")
        yellow_widget = slice_widgets.get("Yellow")
        if red_widget is not None:
            # Establish one canonical view orientation from the original image.
            red_widget.sliceLogic().RotateSliceToLowestVolumeAxes(True)
        if red_widget is not None and yellow_widget is not None:
            # Do not let Yellow independently choose an equivalent volume axis: that
            # can differ by 90 degrees.  Use the exact same camera plane as Red.
            red_slice_to_ras = red_widget.mrmlSliceNode().GetSliceToRAS()
            yellow_slice_node = yellow_widget.mrmlSliceNode()
            yellow_slice_node.GetSliceToRAS().DeepCopy(red_slice_to_ras)
            yellow_slice_node.UpdateMatrices()
            self._hide_navigation_overlays_from_yellow(yellow_slice_node)

        for slice_widget in slice_widgets.values():
            slice_widget.sliceLogic().FitSliceToAll()
        self._slice_layout_configured = True

    def _hide_navigation_overlays_from_yellow(self, yellow_slice_node):
        """Keep motor geometry and click annotations out of the detection view.

        MRML display nodes use an allow-list of view-node IDs; an empty list means
        visible everywhere.  For navigation objects with no existing restriction we
        therefore add every current view except Yellow.  Existing restrictions are
        narrowed instead of broadened, preserving any scene-specific visibility.
        """
        yellow_view_id = yellow_slice_node.GetID()
        if not yellow_view_id:
            return

        all_other_view_ids = []
        nodes = slicer.mrmlScene.GetNodes()
        nodes.InitTraversal()
        while True:
            node = nodes.GetNextItemAsObject()
            if node is None:
                break
            if node.IsA("vtkMRMLAbstractViewNode") and node.GetID() != yellow_view_id:
                all_other_view_ids.append(node.GetID())

        hidden_name_prefixes = (
            "NeedleReachBoundary",
            "NeedleClickPoint",
            "NeedleEncoderModel",
            "NeedlePreviewModel",
        )
        nodes.InitTraversal()
        while True:
            node = nodes.GetNextItemAsObject()
            if node is None:
                break
            node_name = node.GetName() or ""
            if not node_name.startswith(hidden_name_prefixes):
                continue
            display_node = node.GetDisplayNode() if hasattr(node, "GetDisplayNode") else None
            if display_node is None:
                continue
            existing_view_ids = list(display_node.GetViewNodeIDs())
            if existing_view_ids:
                allowed_view_ids = [
                    view_id
                    for view_id in existing_view_ids
                    if view_id != yellow_view_id
                ]
            else:
                allowed_view_ids = list(all_other_view_ids)
            if not allowed_view_ids:
                allowed_view_ids = ["__NeedleNavigationHiddenView__"]
            display_node.SetViewNodeIDs(allowed_view_ids)

    def _start_communication_metrics(self):
        self._theta_subscriber = self._legacy_namespace.get("subTheta")
        self._linear_subscriber = self._legacy_namespace.get("subLinear")
        if self._theta_subscriber is not None:
            self._theta_rate_observer = self._theta_subscriber.AddObserver(
                "ModifiedEvent",
                self._on_theta_state_received,
            )
        if self._linear_subscriber is not None:
            self._linear_rate_observer = self._linear_subscriber.AddObserver(
                "ModifiedEvent",
                self._on_linear_state_received,
            )

        self._metrics_window_start_sec = time.monotonic()
        self._last_metrics_theta_count = self._theta_state_count
        self._last_metrics_linear_count = self._linear_state_count
        self._last_metrics_volume_count = self._input_volume_update_count
        self._last_metrics_image_count = self._projection_sent_count
        self._last_metrics_image_bytes = self._projection_sent_bytes
        self._last_metrics_tracking_count = self._tracking_result_count

        self._metrics_timer = qt.QTimer()
        self._metrics_timer.setInterval(1000)
        self._metrics_timer.timeout.connect(self._update_communication_metrics)
        self._metrics_timer.start()

    def _remove_communication_rate_observers(self):
        if self._theta_subscriber is not None and self._theta_rate_observer is not None:
            self._theta_subscriber.RemoveObserver(self._theta_rate_observer)
        if self._linear_subscriber is not None and self._linear_rate_observer is not None:
            self._linear_subscriber.RemoveObserver(self._linear_rate_observer)
        self._theta_subscriber = None
        self._linear_subscriber = None
        self._theta_rate_observer = None
        self._linear_rate_observer = None

    def _on_theta_state_received(self, caller=None, event=None):
        self._theta_state_count += 1

    def _on_linear_state_received(self, caller=None, event=None):
        self._linear_state_count += 1

    def _on_input_volume_received(self, caller=None, event=None):
        self._input_volume_update_count += 1

    def _set_input_volume_rate_observer(self, volume_node):
        if (
            self._input_volume_node is not None
            and self._input_volume_rate_observer is not None
        ):
            self._input_volume_node.RemoveObserver(self._input_volume_rate_observer)

        self._input_volume_node = volume_node
        self._input_volume_rate_observer = None
        if volume_node is not None:
            self._input_volume_rate_observer = volume_node.AddObserver(
                slicer.vtkMRMLVolumeNode.ImageDataModifiedEvent,
                self._on_input_volume_received,
            )
        self._last_metrics_volume_count = self._input_volume_update_count

    def _update_communication_metrics(self):
        now_sec = time.monotonic()
        elapsed_sec = max(now_sec - self._metrics_window_start_sec, 1e-6)

        theta_hz = (
            self._theta_state_count - self._last_metrics_theta_count
        ) / elapsed_sec
        linear_hz = (
            self._linear_state_count - self._last_metrics_linear_count
        ) / elapsed_sec
        volume_rx_hz = (
            self._input_volume_update_count - self._last_metrics_volume_count
        ) / elapsed_sec
        image_hz = (
            self._projection_sent_count - self._last_metrics_image_count
        ) / elapsed_sec
        image_mib_per_sec = (
            self._projection_sent_bytes - self._last_metrics_image_bytes
        ) / elapsed_sec / (1024.0 * 1024.0)
        tracking_hz = (
            self._tracking_result_count - self._last_metrics_tracking_count
        ) / elapsed_sec
        self.communicationStatusLabel.setText(
            f"US OpenIGTLink RX: {volume_rx_hz:.1f} Hz | "
            f"Image ROS2 TX: {image_hz:.1f} Hz, {image_mib_per_sec:.2f} MiB/s, "
            f"subs={self._last_subscriber_count} | "
            f"Motor ROS2 state RX: theta={theta_hz:.1f} Hz, d={linear_hz:.1f} Hz | "
            f"Tracking RX: {tracking_hz:.1f} Hz"
        )

        self._metrics_window_start_sec = now_sec
        self._last_metrics_theta_count = self._theta_state_count
        self._last_metrics_linear_count = self._linear_state_count
        self._last_metrics_volume_count = self._input_volume_update_count
        self._last_metrics_image_count = self._projection_sent_count
        self._last_metrics_image_bytes = self._projection_sent_bytes
        self._last_metrics_tracking_count = self._tracking_result_count

    def _remove_projection_publisher(self):
        if self._ros_node is not None:
            try:
                self._ros_node.RemoveAndDeletePublisherNode(PROJECTION_TOPIC)
            except Exception:
                pass
        self._projection_publisher = None

    def _start_projection_timer(self):
        interval_ms = int(
            self._legacy_namespace.get("VOLUME_PREVIEW_INTERVAL_MS", 50)
        )
        interval_ms = max(10, interval_ms)
        self._projection_timer = qt.QTimer()
        self._projection_timer.setInterval(interval_ms)
        self._projection_timer.timeout.connect(self._publish_new_max_projection)
        self._projection_timer.start()

    def _on_projection_publish_toggled(self, checked):
        if not checked:
            self.projectionStatusLabel.setText("Projection publishing paused")
        elif self._initialized:
            self.projectionStatusLabel.setText(
                f"Projection publishing enabled on {PROJECTION_TOPIC}"
            )

    def _select_initial_input_volume(self):
        if self.inputVolumeSelector.currentNode() is not None:
            return
        try:
            default_volume = slicer.util.getNode("Image_Reference")
        except slicer.util.MRMLNodeNotFoundException:
            default_volume = None
        if default_volume is not None and default_volume.IsA("vtkMRMLScalarVolumeNode"):
            self.inputVolumeSelector.setCurrentNode(default_volume)

    def _on_input_volume_changed(self, volume_node):
        self._set_input_volume_rate_observer(volume_node)
        self._last_volume_frame_index = -1
        if volume_node is None:
            self.projectionStatusLabel.setText("Select an input ultrasound volume")
            return

        volume_name = volume_node.GetName()
        if self._legacy_namespace is not None:
            self._legacy_namespace["VOLUME_NODE_NAME"] = volume_name

        if self._legacy_widget is not None:
            if getattr(self._legacy_widget, "_recording", False):
                self._legacy_widget.stop_recording()
            if hasattr(self._legacy_widget, "volumeNameLabel"):
                self._legacy_widget.volumeNameLabel.setText(f"Volume node: {volume_name}")
            if hasattr(self._legacy_widget, "start_volume_preview"):
                self._legacy_widget.start_volume_preview()

        self.projectionStatusLabel.setText(
            f"Input volume: {volume_name}; publishing on {PROJECTION_TOPIC}"
        )

    def _current_max_projection(self):
        """Return the legacy-oriented, normalized uint8 maximum projection."""
        volume_node = self.inputVolumeSelector.currentNode()
        if volume_node is None:
            raise RuntimeError("Select an input ultrasound volume")
        volume_array = slicer.util.arrayFromVolume(volume_node)

        if volume_array.ndim == 2:
            projection = volume_array
        elif volume_array.ndim == 3:
            projection = np.max(volume_array, axis=0)
        else:
            raise RuntimeError(f"Unsupported volume shape: {volume_array.shape}")

        # Match the orientation used by the existing Preview Move Capture display and recording.
        projection = np.flipud(np.fliplr(projection))
        normalize_to_uint8 = self._legacy_namespace.get("normalize_to_uint8")
        if normalize_to_uint8 is None:
            raise RuntimeError("normalize_to_uint8 is unavailable in the legacy script")
        projection = normalize_to_uint8(projection)
        return np.ascontiguousarray(projection, dtype=np.uint8)

    def _publish_new_max_projection(self):
        """Publish each new volume frame once and refresh the Red Slice copy.

        ``_volume_frame_index`` is advanced by the embedded legacy preview.  Using it
        as the change detector avoids repeatedly processing the same MRML image from
        a faster Qt timer.
        """
        if (
            not self._initialized
            or not self.projectionPublishCheckBox.checked
            or self._projection_publisher is None
            or self._legacy_widget is None
        ):
            return

        frame_index = int(getattr(self._legacy_widget, "_volume_frame_index", -1))
        if frame_index < 0 or frame_index == self._last_volume_frame_index:
            return
        self._last_volume_frame_index = frame_index

        try:
            projection = self._current_max_projection()
            self._latest_projection = np.ascontiguousarray(projection)
            if self._slice_display_enabled:
                self._source_projection_volume = self._update_display_volume(
                    self._source_projection_volume,
                    "NeedleUltrasoundProjection",
                    self._tracker_frame_to_slicer_array(projection),
                    self.inputVolumeSelector.currentNode(),
                )
                if not self._slice_layout_configured:
                    self._configure_tracking_slice_layout()
            self._publish_encoder_geometry(frame_index, projection.shape)
            vtk_projection = numpy_to_vtk(
                num_array=projection,
                deep=True,
                array_type=vtk.VTK_UNSIGNED_CHAR,
            )
            subscriber_count = int(self._projection_publisher.Publish(vtk_projection))
            self._last_subscriber_count = subscriber_count
            self._projection_attempt_count += 1
            if subscriber_count > 0:
                self._projection_sent_count += 1
                self._projection_sent_bytes += int(projection.nbytes)

            now_sec = time.monotonic()
            if now_sec - self._last_status_update_sec >= 1.0:
                height, width = projection.shape
                self.projectionStatusLabel.setText(
                    f"Max projection {width}x{height} mono8 | frame={frame_index} | "
                    f"subscribers={subscriber_count} | sent={self._projection_sent_count}"
                )
                self._last_status_update_sec = now_sec
        except Exception as exc:
            now_sec = time.monotonic()
            if now_sec - self._last_status_update_sec >= 1.0:
                self.projectionStatusLabel.setText(f"Projection publish error: {exc}")
                self._last_status_update_sec = now_sec

    def _publish_encoder_geometry(self, frame_index, frame_shape):
        """Publish encoder tip/base in native projection pixels.

        Array contract: ``[frame, tip_x, tip_y, base_x, base_y, width, height]``.
        NaN coordinates explicitly mean that registration is unavailable or the
        transformed needle is outside the image; the detector then resets rather than
        reusing a stale prior.
        """
        height, width = frame_shape[:2]
        values = [
            float(frame_index),
            math.nan,
            math.nan,
            math.nan,
            math.nan,
            float(width),
            float(height),
        ]
        encoder_log = self._legacy_widget.current_encoder_log(frame_shape)
        predicted_tip = encoder_log.get("predicted_tip")
        predicted_line = encoder_log.get("predicted_line")
        if predicted_tip is not None and predicted_line is not None:
            values[1] = float(predicted_tip[0])
            values[2] = float(predicted_tip[1])
            values[3] = float(predicted_line[2])
            values[4] = float(predicted_line[3])

        vtk_values = vtk.vtkDoubleArray()
        vtk_values.SetNumberOfValues(len(values))
        for index, value in enumerate(values):
            vtk_values.SetValue(index, value)
        self._encoder_geometry_publisher.Publish(vtk_values)
