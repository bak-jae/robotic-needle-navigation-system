# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

import __main__
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
from vtk.util.numpy_support import numpy_to_vtk


MODULE_TITLE = "Needle Navigation"
PROJECTION_TOPIC = "/ultrasound/projection/max"
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
    """Embed the existing control widget in the Slicer module panel."""

    def __init__(self, parent=None):
        ScriptedLoadableModuleWidget.__init__(self, parent)
        self._initialized = False
        self._initialization_error = None
        self._legacy_namespace = None
        self._legacy_widget = None
        self._ros_node = None
        self._projection_publisher = None
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
        self._last_status_update_sec = 0.0

    def setup(self):
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

        self.legacyContainer = qt.QWidget()
        self.legacyContainerLayout = qt.QVBoxLayout(self.legacyContainer)
        self.legacyContainerLayout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.legacyContainer)
        self.layout.addStretch(1)

    def enter(self):
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
        if self._projection_timer is not None:
            self._projection_timer.stop()
            self._projection_timer = None

        if self._metrics_timer is not None:
            self._metrics_timer.stop()
            self._metrics_timer = None

        self._remove_communication_rate_observers()
        self._set_input_volume_rate_observer(None)

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
        self._start_communication_metrics()
        self._start_projection_timer()
        self._initialized = True
        volume_name = selected_volume.GetName() if selected_volume is not None else "a selected volume"
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
        self._remove_projection_publisher()
        self._projection_publisher = self._ros_node.CreateAndAddPublisherNode(
            "UInt8Image",
            PROJECTION_TOPIC,
        )
        if self._projection_publisher is None:
            raise RuntimeError(f"Failed to create UInt8Image publisher for {PROJECTION_TOPIC}")

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
        self.communicationStatusLabel.setText(
            f"US OpenIGTLink RX: {volume_rx_hz:.1f} Hz | "
            f"Image ROS2 TX: {image_hz:.1f} Hz, {image_mib_per_sec:.2f} MiB/s, "
            f"subs={self._last_subscriber_count} | "
            f"Motor ROS2 state RX: theta={theta_hz:.1f} Hz, d={linear_hz:.1f} Hz"
        )

        self._metrics_window_start_sec = now_sec
        self._last_metrics_theta_count = self._theta_state_count
        self._last_metrics_linear_count = self._linear_state_count
        self._last_metrics_volume_count = self._input_volume_update_count
        self._last_metrics_image_count = self._projection_sent_count
        self._last_metrics_image_bytes = self._projection_sent_bytes

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
