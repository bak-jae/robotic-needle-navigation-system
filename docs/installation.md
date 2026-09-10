# Installation

This guide targets the verified Ubuntu 22.04, ROS 2 Humble, and Slicer 5.6.2
configuration. The SlicerROS2 prerequisites also specify Slicer 5.6.2 source, system Qt,
a source-built Slicer, and system OpenSSL for Ubuntu 22.04/Humble.

Reference: [SlicerROS2 getting started](https://slicer-ros2.readthedocs.io/en/v1.0/pages/getting-started.html#pre-requisites)

## 1. Install ROS 2 and build tools

Install ROS 2 Humble using its official instructions, then install the required build
tools.

```bash
sudo apt install python3-colcon-common-extensions cmake-curses-gui
source /opt/ros/humble/setup.bash
```

## 2. Build Slicer 5.6.2 from source

A source build is required to compile and load the C++ extensions. Choose any workspace
location and replace each `/path/to/...` placeholder with the corresponding local path.

```bash
git clone https://github.com/Slicer/Slicer.git /path/to/Slicer
git -C /path/to/Slicer checkout f10cd8c229b50e4d96e55743385f17482214433a
mkdir -p /path/to/Slicer-SuperBuild
cd /path/to/Slicer-SuperBuild
cmake -DSlicer_USE_SYSTEM_OpenSSL=ON -DCMAKE_BUILD_TYPE=Release /path/to/Slicer
cmake --build . -j"$(nproc)"
```

`Slicer_DIR` must point to the inner `Slicer-build` directory containing
`SlicerConfig.cmake`, not the outer superbuild directory. Use the system/native Qt
installation; do not mix it with a separate Qt online installation.

## 3. Build SlicerROS2

```bash
mkdir -p /path/to/ros2_ws/src
git clone https://github.com/rosmed/slicer_ros2_module.git \
  /path/to/ros2_ws/src/slicer_ros2_module
git -C /path/to/ros2_ws/src/slicer_ros2_module checkout \
  2abf6a9c0563fe5d2041a038fccdffedfa7cb7c3

git -C /path/to/ros2_ws/src/slicer_ros2_module apply \
  /path/to/robotic-needle-navigation-system/patches/slicer_ros2_humble_rosbag_services.patch
git -C /path/to/ros2_ws/src/slicer_ros2_module apply \
  /path/to/robotic-needle-navigation-system/patches/slicer_ros2_image_step.patch

cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select slicer_ros2_module --cmake-args \
  -DSlicer_DIR:PATH=/path/to/Slicer-SuperBuild/Slicer-build \
  -DCMAKE_BUILD_TYPE=Release
```

See [`extensions.md`](extensions.md) for the additional extensions.

## 4. Install the EPOS Command Library

Obtain and install Linux EPOS Command Library **6.8.1.0** through an official Maxon
source. The vendor archive, `Definitions.h`, and `libEposCmd.so` are not distributed in
this repository. The standard location on the verified system is
`/opt/EposCmdLib_6.8.1.0`.

The repository also excludes the previous Maxon example-derived homing helper. Each
institution must establish an authorized and hardware-validated homing procedure using
the official SDK, tools, and documentation available to it.

If the SDK is installed elsewhere, set its root before building.

```bash
export EPOS_CMD_ROOT=/path/to/EposCmdLib_6.8.1.0
```

## 5. Build this ROS 2 package

Clone or symlink the repository under a ROS 2 workspace `src` directory.

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_epos_cmd
source install/setup.bash
```

## 6. Load the Slicer module

Start Slicer from a terminal that has sourced both ROS 2 and the SlicerROS2 workspace.

```bash
source /opt/ros/humble/setup.bash
source /path/to/ros2_ws/install/setup.bash
cd /path/to/Slicer-SuperBuild/Slicer-build
./Slicer
```

In Slicer, open **Edit > Application Settings > Modules > Additional module paths**,
add `/path/to/robotic-needle-navigation-system/slicer_modules`, and restart Slicer.

## 7. Configure local data paths

```bash
export NEEDLE_RECORD_DIR=/path/to/writable/volume_recordings
```

If this variable is not set, recordings are written under the ignored
`runtime/volume_recordings` directory in the repository.

## 8. Prepare the optional detector runtime

The detection source is included, but trained weights are not. Create a Python 3.10 virtual
environment that can import the system ROS 2 packages:

```bash
cd /path/to/robotic-needle-navigation-system
/usr/bin/python3 -m venv --system-site-packages .venv-needle-tracker
.venv-needle-tracker/bin/pip install -r \
  slicer_modules/NeedleNavigation/tracker_runtime/requirements.txt
```

Install a PyTorch wheel suitable for the workstation. The verified CUDA 12.1 environment used:

```bash
.venv-needle-tracker/bin/pip install torch==2.4.1 \
  --index-url https://download.pytorch.org/whl/cu121
```

Copy an authorized, compatible SmallUNet `state_dict` to
`slicer_modules/NeedleNavigation/tracker_runtime/best_model.pt`, or set `NEEDLE_MODEL` to an
external absolute path. The model file is intentionally ignored by Git. See the runtime
[README](../slicer_modules/NeedleNavigation/tracker_runtime/README.md) for the exact contract.
