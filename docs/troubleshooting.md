# Troubleshooting

## `rqt_image_view` topic exists but the image is blank

Check `ros2 topic echo --once /ultrasound/projection/max`. For the verified 512×512 mono8
image, `step` must be 512 and `data` must be non-empty. Apply
`patches/slicer_ros2_image_step.patch`, rebuild SlicerROS2, and restart Slicer. Also select
the actual input volume in Needle Navigation; the volume name itself need not be
`Image_Reference`.

## `RemovePublisherNode` / `RemoveSubscriberNode ... does not exist`

These warnings can occur while the compatibility module removes stale SlicerROS2 MRML
references during reload. If publishing/subscribing works afterward they are generally
cleanup warnings. Persistent failure requires checking that exactly one Needle Navigation
instance is active and restarting Slicer with the ROS workspace sourced.

## Slicer module panel becomes too wide

Long status text can impose a QLabel minimum width. This repository sets status labels to
`QSizePolicy.Ignored` horizontally with word wrapping, allowing the left panel to shrink.

## EPOS library not found during CMake

Install EPOS Command Library 6.8.1.0 in its standard `/opt` location or set:

```bash
export EPOS_CMD_ROOT=/path/to/EposCmdLib_6.8.1.0
```

Then delete only the package's CMake cache/build directory or rebuild with
`colcon build --packages-select ros2_epos_cmd --cmake-clean-cache`.

## Slicer cannot see ROS resources

Start Slicer from a terminal after sourcing both ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source /path/to/ros2_ws/install/setup.bash
/path/to/Slicer-build/Slicer
```

## Humble build fails on missing rosbag2 service types

Apply `patches/slicer_ros2_humble_rosbag_services.patch` to the pinned SlicerROS2 commit.
It excludes service definitions absent from the verified Humble interface set.

## No CAN traffic or bridge fails to open EPOS4

Check `ip -details link show can0`, bitrate 1,000,000, adapter name, power, termination, and
node IDs. `interface_name`/`port_name` in `epos.yaml` are Maxon EPOS library identifiers and
may not exactly match Linux SocketCAN naming on a different adapter.
