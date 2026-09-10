# ROS 2 nodes and topics

## Nodes

| Node/process | Implemented in | Runs in | Role |
|---|---|---|---|
| `/epos_motion_bridge_node` | C++ `epos_motion_bridge_node.cpp` | independent ROS 2 process | Converts degree/mm commands to EPOS counts, commands EPOS4, publishes encoder-derived state |
| Slicer ROS2 node | SlicerROS2 C++ extension + module Python | inside the Slicer process | Sends motor topics, receives state, publishes ultrasound projection |
| `/needle_tracker_node` | Python `ros2_needle_tracker_node.py` | independent ROS 2 process started by Slicer | Subscribes to image/encoder geometry and publishes visualization-only detection outputs |

`ros2 launch ros2_epos_cmd needle_closed_loop.launch.py` starts
`/epos_motion_bridge_node`. The node is compiled from C++ by `colcon`; launch does not
generate the executable at runtime.

## Current topic contract

| Topic | Type | Direction | Meaning / unit |
|---|---|---|---|
| `/needle/cmd/theta_deg` | `std_msgs/msg/Float64` | Slicer → bridge | absolute rotational target, degree |
| `/needle/cmd/d_mm` | `std_msgs/msg/Float64` | Slicer → bridge | absolute linear target, mm |
| `/needle/cmd/theta_velocity` | `std_msgs/msg/Float64` | Slicer → bridge | rotation speed target, tip rpm |
| `/needle/cmd/d_velocity` | `std_msgs/msg/Float64` | Slicer → bridge | linear speed target, mm/s |
| `/needle/cmd/execute_fast_mode` | `std_msgs/msg/Bool` | Slicer → bridge | select fast motion profile |
| `/needle/state/theta_deg` | `std_msgs/msg/Float64` | bridge → Slicer | encoder-derived rotation state, degree |
| `/needle/state/d_mm` | `std_msgs/msg/Float64` | bridge → Slicer | encoder-derived linear state, mm |
| `/ultrasound/projection/max` | `sensor_msgs/msg/Image` | Slicer → ROS 2 | full-resolution max projection, `mono8` |
| `/needle/encoder/geometry_px` | `std_msgs/msg/Float64MultiArray` | Slicer → tracker | `[frame, tip_x, tip_y, base_x, base_y, width, height]` in native projection pixels |
| `/needle/tracking/result_px` | `std_msgs/msg/Float64MultiArray` | tracker → Slicer | `[image_seq, encoder_frame, detected, x_512, y_512, cnn, used_kalman, visible_run]` |
| `/needle/tracking/overlay` | `sensor_msgs/msg/Image` | tracker → Slicer | native-size `mono8` tracking preview |

The bridge performs the hardware conversion in both directions:

```text
Slicer degree/mm command -> ROS topic -> bridge conversion -> EPOS counts
EPOS encoder counts -> bridge conversion -> ROS degree/mm state -> Slicer transform
```

Slicer converts the returned theta/d values to the visual `NeedleMoveEncoder`
transform. Therefore hardware unit conversion belongs to C++, while the needle model's
scene position/orientation belongs to Slicer Python.

## Inspect

```bash
ros2 node list
ros2 node info /epos_motion_bridge_node
ros2 topic list -t
ros2 topic hz /needle/state/theta_deg
ros2 topic hz /needle/state/d_mm
ros2 topic hz /ultrasound/projection/max
ros2 topic bw /ultrasound/projection/max
ros2 topic echo /needle/encoder/geometry_px
ros2 topic echo /needle/tracking/result_px
ros2 topic hz /needle/tracking/overlay
ros2 run rqt_image_view rqt_image_view
```

The verified bridge state period is 30 ms (target about 33.3 Hz), and the verified
image publisher is about 20 Hz for 512×512 mono8. Actual rates depend on Slicer UI,
OpenIGTLink input, CPU, and ROS middleware.
