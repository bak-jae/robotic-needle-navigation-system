# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

import math

import qt
import slicer
import vtk

# Legacy registration helper. Prefer loading it from the repository path selected
# by the operator instead of embedding a machine-specific absolute path here.

TRANSFORM_NAME = "NeedleMove"
ENCODER_TRANSFORM_NAME = "NeedleMoveEncoder"
ENCODER_NEEDLE_MODEL_NAME = "NeedleEncoderModel"
CLICK_POINT_NODE_NAME = "NeedleClickPoint"
TOPIC_CMD_THETA = "/needle/cmd/theta_deg"
TOPIC_CMD_LINEAR = "/needle/cmd/d_mm"
TOPIC_EXECUTE_FAST_MODE = "/needle/cmd/execute_fast_mode"
TOPIC_STATE_THETA = "/needle/state/theta_deg"
TOPIC_STATE_LINEAR = "/needle/state/d_mm"
THETA_MIN_DEG = 0
THETA_MAX_DEG = 35
D_MIN_MM = 0
D_MAX_MM = 55
THETA_SLIDER_SCALE = 100.0
D_SLIDER_SCALE = 100.0
CLICK_SLICE_VIEWS = ("Red", "Yellow", "Green")
EXECUTE_DEFAULT_STROKE_MM = 10.0
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
BASE_ROT_AXIS = "Y"
BASE_ROT_SIGN = -1.0

EXTRA_ROT_DEG = 180.0
EXTRA_ROT_AXIS = "Y"


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


def get_or_create_needle_model(name, transform_node):
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
        display.SetColor(1.0, 0.55, 0.0)
        display.SetOpacity(1.0)
        display.SetVisibility2D(True)
        display.SetVisibility3D(True)
        display.SetSliceIntersectionVisibility(True)

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
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    pos[POS_AXIS_X] = POS_SIGN_X * x
    pos[POS_AXIS_Y] = POS_SIGN_Y * y
    return pos["X"], pos["Y"], pos["Z"]


def ras_to_motion_xy(ras):
    axis = {"X": ras[0], "Y": ras[1], "Z": ras[2]}
    x = axis[POS_AXIS_X] / POS_SIGN_X if POS_SIGN_X != 0.0 else 0.0
    y = axis[POS_AXIS_Y] / POS_SIGN_Y if POS_SIGN_Y != 0.0 else 0.0
    return x, y


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

    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    pos[POS_AXIS_X] = POS_SIGN_X * x
    pos[POS_AXIS_Y] = POS_SIGN_Y * y
    m.SetElement(0, 3, pos["X"])
    m.SetElement(1, 3, pos["Y"])
    m.SetElement(2, 3, pos["Z"])
    transform_node.SetMatrixTransformToParent(m)


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
for _topic in (TOPIC_CMD_THETA, TOPIC_CMD_LINEAR):
    try:
        rosNode.RemoveAndDeletePublisherNode(_topic)
    except Exception:
        pass
try:
    rosNode.RemoveAndDeletePublisherNode(TOPIC_EXECUTE_FAST_MODE)
except Exception:
    pass

pubTheta = get_or_create_publisher(rosNode, TOPIC_CMD_THETA)
pubLinear = get_or_create_publisher(rosNode, TOPIC_CMD_LINEAR)
pubExecuteFastMode = get_or_create_bool_publisher(rosNode, TOPIC_EXECUTE_FAST_MODE)
subTheta = get_or_create_subscriber(rosNode, TOPIC_STATE_THETA)
subLinear = get_or_create_subscriber(rosNode, TOPIC_STATE_LINEAR)
transformNode = get_or_create_transform()
encoderTransformNode = get_or_create_transform(ENCODER_TRANSFORM_NAME)
encoderNeedleModelNode = get_or_create_needle_model(ENCODER_NEEDLE_MODEL_NAME, encoderTransformNode)
clickPointNode = get_or_create_click_point()
remove_node_if_exists("NeedleReachBoundary")
remove_node_if_exists("NeedleRotationCenter")
remove_node_if_exists("NeedleWorkPlane")


class NeedleSlicerClosedLoop(qt.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Needle Control (No Offset)")

        self._cmd_theta = 0.0
        self._cmd_d = 0.0
        self._enc_theta = 0.0
        self._enc_d = 0.0
        self._is_execute_running = False
        self._execute_base_d = 0.0
        self._execute_forward_d = 0.0
        self._execute_phase = 0
        self._execute_fast_mode = False
        self._slice_click_observers = []
        self._execute_timer = qt.QTimer()
        self._execute_timer.timeout.connect(self.on_execute_tick)

        layout = qt.QFormLayout(self)

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
        self.executeButton = qt.QPushButton("Sample Start")
        self.executeButton.setCheckable(True)
        self.fastModeButton = qt.QPushButton("Speed: Slow")
        self.fastModeButton.setCheckable(True)
        self.executeStatusLabel = qt.QLabel("Sample: idle")

        layout.addRow(self.cmdThetaLabel, self.thetaSlider)
        layout.addRow(self.cmdDLabel, self.dSlider)
        layout.addRow("Sample stroke", self.executeStrokeSpin)
        layout.addRow("", self.executeButton)
        layout.addRow("", self.fastModeButton)
        layout.addRow("", self.executeStatusLabel)
        layout.addRow("", self.currentPosLabel)
        layout.addRow("", self.encoderPosLabel)
        layout.addRow("", self.clickLabel)
        layout.addRow("", self.ikLabel)

        self.thetaSlider.valueChanged.connect(self.on_slider_changed)
        self.dSlider.valueChanged.connect(self.on_slider_changed)
        self.executeButton.toggled.connect(self.on_execute_toggled)
        self.fastModeButton.toggled.connect(self.on_fast_mode_toggled)

        self.thetaObserver = subTheta.AddObserver("ModifiedEvent", self.on_theta_state)
        self.dObserver = subLinear.AddObserver("ModifiedEvent", self.on_d_state)
        self.install_slice_click_observers()

        self.on_slider_changed()
        self.update_encoder_transform()

    def cleanup(self):
        self._execute_timer.stop()
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
        if self._is_execute_running:
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

    def on_fast_mode_toggled(self, checked):
        self._execute_fast_mode = bool(checked)
        if self._execute_fast_mode:
            self.fastModeButton.setText("Speed: Fast")
            self.executeStatusLabel.setText("Sample: speed mode FAST")
        else:
            self.fastModeButton.setText("Speed: Slow")
            self.executeStatusLabel.setText("Sample: speed mode SLOW")
        pubExecuteFastMode.Publish(self._execute_fast_mode)

    def on_execute_toggled(self, checked):
        if checked:
            if not self.start_execute():
                self.executeButton.blockSignals(True)
                self.executeButton.setChecked(False)
                self.executeButton.blockSignals(False)
            return
        self.stop_execute()

    def start_execute(self):
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
        self._is_execute_running = True
        self.executeButton.setText("Sample Stop")
        self.executeStatusLabel.setText(
            f"Sample: running, base={self._execute_base_d:.2f} [mm], target={self._execute_forward_d:.2f} [mm]"
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

    def send_execute_target(self, d_target):
        theta = self.slider_to_theta(self.thetaSlider.value)
        d_int = self.d_to_slider(d_target)
        d_int = max(self.dSlider.minimum, min(self.dSlider.maximum, d_int))
        d_target = self.slider_to_d(d_int)
        self.dSlider.blockSignals(True)
        self.dSlider.setValue(d_int)
        self.dSlider.blockSignals(False)
        self.apply_command(theta, d_target)

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

        mx, my = ras_to_motion_xy(ras)
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
        self.ikLabel.setText(f"Target from click: theta={theta_deg:.3f} [deg], d={d_mm:.3f} [mm]")
        print(f"[NeedleSlicerClosedLoop] target from click: theta={theta_deg:.3f} [deg], d={d_mm:.3f} [mm]")
        theta_int = self.theta_to_slider(theta_deg)
        d_int = self.d_to_slider(d_mm)
        theta_int = max(self.thetaSlider.minimum, min(self.thetaSlider.maximum, theta_int))
        d_int = max(self.dSlider.minimum, min(self.dSlider.maximum, d_int))

        self.thetaSlider.blockSignals(True)
        self.dSlider.blockSignals(True)
        self.thetaSlider.setValue(theta_int)
        self.dSlider.setValue(d_int)
        self.thetaSlider.blockSignals(False)
        self.dSlider.blockSignals(False)
        self.on_slider_changed()


# Prevent duplicate windows when script is executed multiple times.
if "needle_slicer_closed_loop_widget" in globals() and needle_slicer_closed_loop_widget:
    try:
        needle_slicer_closed_loop_widget.cleanup()
        needle_slicer_closed_loop_widget.close()
    except Exception:
        pass

needle_slicer_closed_loop_widget = NeedleSlicerClosedLoop()
needle_slicer_closed_loop_widget.show()
