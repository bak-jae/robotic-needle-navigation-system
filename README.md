# Robotic Needle Navigation System

A research platform that receives ultrasound volumes in 3D Slicer, aligns a virtual
needle model with the image coordinate system, and controls separate rotational and
linear EPOS4 axes through ROS 2. The Slicer module publishes a full-resolution maximum
projection as a `mono8` image on `/ultrasound/projection/max`, providing an input for
future real-time needle detection and closed-loop research.

> **License:** Academic research and education use only. This public repository is
> source-available but is not OSI-approved open-source software. Commercial, clinical,
> and production use require separate written permission. See [LICENSE](LICENSE).

## Scope

- `ros2_epos_cmd`: EPOS4 ROS 2 bridge, launch file, and parameter YAML
- `NeedleNavigation`: Slicer module containing the existing control, preview, recording,
  and ROS 2 ultrasound projection interface
- Compatibility patches used for the pinned SlicerROS2 and SlicerIGT revisions
- Installation, registration, Plus/Telemed, topic, and operating documentation

The repository does not contain ultrasound recordings, training datasets, build output,
Plus or Maxon installers, vendor headers/libraries, or the previous Maxon example-derived
homing helper.

## Verified environment

| Component | Verified version |
|---|---|
| Operating system | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble |
| 3D Slicer | 5.6.2 source build, commit `f10cd8c229b50e4d96e55743385f17482214433a` |
| SlicerROS2 | commit `2abf6a9c0563fe5d2041a038fccdffedfa7cb7c3` plus repository patches |
| SlicerOpenIGTLink | commit `f806007ccc4b5c7cb6ea14979670a468b39c5a45` |
| SlicerIGT | commit `c73b30761f01822081cfa86a3b95f429ef7a5389` plus repository patch |
| Motor controller | EPOS4 with EPOS Command Library 6.8.1.0 |
| Ultrasound | Telemed ArtUS with Plus 2.8.0 Telemed Win32 |

A pinned commit is a fixed source revision used to reproduce the verified build instead
of relying on a branch that changes over time. See
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
EPOS bridge.

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
