# Robotic Needle Navigation System

A research platform that receives ultrasound volumes in 3D Slicer, aligns a virtual
needle model with the image coordinate system, and controls separate rotational and
linear EPOS4 axes through ROS 2. The Slicer module publishes a full-resolution maximum
projection as a `mono8` image on `/ultrasound/projection/max`. An independent ROS 2
SmallUNet process consumes that image and returns a visualization-only needle-tip result
to Slicer. Trained model weights are intentionally not included.

> **License:** Academic research and education use only. This public repository is
> source-available but is not OSI-approved open-source software. Commercial, clinical,
> and production use require separate written permission. See [LICENSE](LICENSE).

## Scope

- `ros2_epos_cmd`: EPOS4 ROS 2 bridge, launch file, and parameter YAML
- `NeedleNavigation`: Slicer module containing the existing control, preview, recording,
  ROS 2 ultrasound projection, detector process controls, and dual-Slice visualization
- `NeedleNavigation/tracker_runtime`: replaceable SmallUNet inference structure and ROS 2
  adapter; checkpoint excluded
- Compatibility patches used for the pinned SlicerROS2 and SlicerIGT revisions
- Installation, registration, Plus/Telemed, topic, and operating documentation

The repository does not contain ultrasound recordings, training datasets, trained model
weights, build output, Plus or Maxon installers, vendor headers/libraries, or the previous
Maxon example-derived homing helper.

## Tested software setup

The system was built and tested with the combination below. The short revision value
identifies the exact source snapshot that was used; it is not a separate version that
must be entered in Slicer.

| Component | Setup used for testing | Source revision |
|---|---|---|
| Operating system | Ubuntu 22.04.5 LTS | - |
| ROS 2 | Humble | - |
| 3D Slicer | Version 5.6.2, built from source | `f10cd8c` |
| SlicerROS2 | ROS 2 integration with two included compatibility fixes | `2abf6a9` |
| SlicerOpenIGTLink | OpenIGTLink communication for receiving ultrasound images | `f806007` |
| SlicerIGT | Image-guided therapy support with one included build fix | `c73b307` |
| Motor controller | EPOS4 with EPOS Command Library 6.8.1.0 | - |
| Ultrasound | Telemed ArtUS with Plus 2.8.0 Telemed Win32 | - |

These are the versions known to work together. Other versions may also work, but they
have not been verified for this repository. Full revision IDs and the names of the
included fixes are listed in
[`docs/verified-environment.md`](docs/verified-environment.md).

## Documentation

1. [Installation](docs/installation.md)
2. [Telemed ArtUS and Plus connection](docs/plus-artus.md)
3. [Fiducial registration and transform hierarchy](docs/registration.md)
4. [Operating sequence](docs/operation.md)
5. [System architecture](docs/architecture.md)
6. [ROS 2 nodes and topics](docs/topics.md)
7. [Values to customize](docs/customization.md)
8. [Troubleshooting](docs/troubleshooting.md)

## Quick start

The following assumes this repository is located under a ROS 2 workspace `src`
directory and that the proprietary EPOS Command Library has already been installed.

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_epos_cmd
source install/setup.bash

sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Perform the separately validated homing procedure for both axes.
ros2 launch ros2_epos_cmd needle_closed_loop.launch.py
```

In another terminal, source ROS 2 and the SlicerROS2 workspace before starting the
source-built Slicer executable. Add this repository's `slicer_modules` directory to
Slicer's **Additional module paths**, restart Slicer, and select
**IGT > Needle Navigation**. Selecting the module does not start CAN, homing, or the
EPOS bridge. The module's **Start Tracker** button starts only the detector process; the
detector has no motor-command publisher.

## Configuration

- Motor, CAN, and topic parameters: `src/ros2_epos_cmd/config/epos.yaml`
- Site-specific reference values: copy `config/system.example.yaml` to the ignored
  `config/system.local.yaml`
- Recording directory: `NEEDLE_RECORD_DIR`; defaults to the ignored
  `runtime/volume_recordings`
- EPOS SDK path: standard `/opt/EposCmdLib_6.8.1.0` location or `EPOS_CMD_ROOT`

## License and third-party software

Original project content is licensed under the
[PNU Advanced Robotics Lab Academic Research License 1.0](LICENSE).

Copyright 2026 Pusan National University, Advanced Robotics Lab

The Maxon EPOS Command Library is a separate proprietary dependency. Users must obtain
it from an official Maxon source and comply with the applicable Maxon EULA. 3D Slicer,
ROS 2, Plus, and the Slicer extensions retain their respective upstream licenses. See
[`THIRD_PARTY.md`](THIRD_PARTY.md).

## Safety

This software is intended for laboratory research and education. It is not a certified
medical device and does not implement a complete functional-safety system. Before
connecting hardware, validate node IDs, gear ratios, direction, travel limits, homing,
watchdog behavior, command units, and a physical emergency-stop or power-removal method.
