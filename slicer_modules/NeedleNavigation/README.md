# Needle Navigation Slicer module

`NeedleNavigation.py` embeds the existing `preview_move_capture.py` control interface in
3D Slicer and adds an out-of-process needle-tip detection pipeline. It does not open a
separate tracking window.

## Responsibilities

The module:

- selects the incoming OpenIGTLink scalar volume;
- computes a full-resolution maximum projection across its Z axis;
- publishes the projection and encoder-derived needle geometry to ROS 2;
- starts and stops the local detector process;
- receives the numeric detection and mono8 preview;
- displays original and detected images in synchronized Red and Yellow Slice views.

CAN configuration, homing, and the EPOS bridge remain external operations. The detector
has no publisher for motor command topics.

## Slice presentation

The side-by-side layout is intentional:

- **Red / Original:** ultrasound plus existing motor boundary, encoder model, preview model,
  and click point;
- **Yellow / Detection:** ultrasound plus only the final detected tip, drawn as a red cross.

Yellow hides `NeedleReachBoundary`, `NeedleClickPoint`, `NeedleEncoderModel`, and
`NeedlePreviewModel` display nodes. Both views use the exact same SliceToRAS matrix. The
derived volumes also copy the input volume's IJK-to-RAS matrix and parent transform, so image
direction, physical pixel spacing, aspect ratio, and calibrated boundary alignment are
preserved.

The legacy OpenCV preview uses a 180-degree NumPy flip. Detector data follows that convention;
the module reverses the flip before placing pixels back into Slicer IJK space.

## Load in Slicer

1. Open **Edit > Application Settings > Modules**.
2. Add `/path/to/robotic-needle-navigation-system/slicer_modules` to
   **Additional module paths**.
3. Restart Slicer.
4. Select **IGT > Needle Navigation**.
5. Select the received ultrasound scalar volume.

Do not run `preview_move_capture.py` from the Python Interactor while the module is active.
That would create duplicate timers, ROS publishers, and possibly motor commands.

## Start detection

Before starting Slicer, prepare the external Python environment and supply an authorized
checkpoint as described in [`tracker_runtime/README.md`](tracker_runtime/README.md). Then:

1. Press **Start Tracker**.
2. Wait for valid encoder geometry. The first ten visible frames are warmup frames.
3. Use **Show Original + Detection in Slices** whenever another Slicer layout has replaced
   the side-by-side view.
4. Press **Stop Tracker** to terminate only the detector process.

The default checkpoint location is `tracker_runtime/best_model.pt`, but model files are
ignored by Git. A replacement checkpoint must match the exact checked-in SmallUNet structure.

## ROS topic contract

| Topic | ROS type | Direction | Payload |
|---|---|---|---|
| `/ultrasound/projection/max` | `sensor_msgs/msg/Image` | module → tracker | native-size `mono8` maximum projection |
| `/needle/encoder/geometry_px` | `std_msgs/msg/Float64MultiArray` | module → tracker | `[frame, tip_x, tip_y, base_x, base_y, width, height]` in native pixels |
| `/needle/tracking/result_px` | `std_msgs/msg/Float64MultiArray` | tracker → module | `[image_seq, encoder_frame, detected, x_512, y_512, cnn, used_kalman, visible_run]` |
| `/needle/tracking/overlay` | `sensor_msgs/msg/Image` | tracker → module | native-size `mono8` preview |

`NaN` tip/base coordinates mean that registration is unavailable or the encoder needle is
outside the image. The tracker clears temporal state instead of returning a stale result.

## Important compatibility behavior

The embedded legacy widget preserves its original initialization behavior. Selecting this
module may publish theta=0 and d=0 if the EPOS bridge is already running. Establish a safe
hardware state before selecting or reloading the module.

## Diagnostics

```bash
ros2 node list
ros2 node info /needle_tracker_node
ros2 topic hz /ultrasound/projection/max
ros2 topic echo /needle/encoder/geometry_px
ros2 topic echo /needle/tracking/result_px
ros2 topic hz /needle/tracking/overlay
```
