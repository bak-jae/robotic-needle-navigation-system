# Needle tracker runtime

This directory is the replaceable, out-of-process detection layer used by the
`NeedleNavigation` Slicer module. It contains the model definition and ROS 2 adapter, but
**does not contain trained weights**.

## Files

| File | Responsibility |
|---|---|
| `needle_realtime_tracker.py` | SmallUNet definition, heatmap inference, encoder prior, candidate selection, and Kalman filtering |
| `ros2_needle_tracker_node.py` | ROS 2 image/geometry subscribers and result/preview publishers |
| `run_tracker.sh` | Environment isolation, validation, and process launcher used by Slicer |
| `requirements.txt` | Python packages installed into the external virtual environment |

The detector has no publisher for `/needle/cmd/*`; it cannot command either motor. The
Slicer module uses its output for visualization only.

## Model checkpoint contract

The default checkpoint path is:

```text
slicer_modules/NeedleNavigation/tracker_runtime/best_model.pt
```

That file is ignored by Git. Supply an authorized PyTorch `state_dict` produced for the
exact `SmallUNet(base_channels=16)` class in `needle_realtime_tracker.py`. Matching only the
`.pt` extension is not sufficient: layer names and tensor shapes must match.

To keep the checkpoint somewhere else, set:

```bash
export NEEDLE_MODEL=/absolute/path/to/best_model.pt
```

After changing a checkpoint, stop and restart the tracker so the new process loads it.

## External Python environment

Slicer and ROS 2 Humble use different Python runtimes in the verified development setup.
Create a repository-local Python 3.10 environment that can see ROS packages:

```bash
cd /path/to/robotic-needle-navigation-system
/usr/bin/python3 -m venv --system-site-packages .venv-needle-tracker
.venv-needle-tracker/bin/pip install -r \
  slicer_modules/NeedleNavigation/tracker_runtime/requirements.txt
```

Install the PyTorch build appropriate for the workstation separately. For the verified
CUDA 12.1 setup:

```bash
.venv-needle-tracker/bin/pip install torch==2.4.1 \
  --index-url https://download.pytorch.org/whl/cu121
```

`rclpy` and `cv_bridge` normally come from the ROS 2 installation through
`--system-site-packages`. Verify the complete runtime before opening Slicer:

```bash
source /opt/ros/humble/setup.bash
.venv-needle-tracker/bin/python -c \
  "import cv2, cv_bridge, numpy, rclpy, torch; print(torch.cuda.is_available())"
```

## Topic and coordinate contract

| Topic | Type | Direction relative to detector | Payload |
|---|---|---|---|
| `/ultrasound/projection/max` | `sensor_msgs/msg/Image` | input | native-size `mono8` ultrasound projection |
| `/needle/encoder/geometry_px` | `std_msgs/msg/Float64MultiArray` | input | `[frame, tip_x, tip_y, base_x, base_y, width, height]` in native pixels |
| `/needle/tracking/result_px` | `std_msgs/msg/Float64MultiArray` | output | `[image_seq, encoder_frame, detected, x_512, y_512, cnn, used_kalman, visible_run]` |
| `/needle/tracking/overlay` | `sensor_msgs/msg/Image` | output | native-size `mono8` preview |

The numeric detector coordinates remain in a canonical 512×512 space. The Slicer module
maps the final point back to the native image size and colors it red locally. Keeping ROS
image transport mono8 is required by the pinned SlicerROS2 `UInt8Image` converter.
