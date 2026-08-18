# Third-party software and dependencies

The academic research license in this repository applies only to original project content for
which Pusan National University, Advanced Robotics Lab has the right to grant that license. It
does not relicense third-party software.

## Maxon EPOS Command Library

- Status: proprietary external dependency
- Version used during verification: 6.8.1.0
- Distributed here: **no**
- Required separately: `Definitions.h`, `libEposCmd.so`, installer/SDK and documentation

Users must obtain the library from Maxon, accept the applicable Maxon EULA, and use it only
with eligible products and within the granted terms. This repository only contains original
bridge code that calls the installed API. The previous Maxon example-derived homing helper is
intentionally excluded from the public repository.

## 3D Slicer and extensions

These projects are installed separately and retain their own licenses:

- 3D Slicer: Slicer license, BSD-style with project-specific terms
- SlicerROS2: MIT License, copyright ROS-MED
- SlicerIGT: BSD 3-Clause License, copyright PerkLab
- SlicerOpenIGTLink: 3D Slicer license

Patch files under `patches/` contain small modifications intended for the pinned upstream
versions. When applying or redistributing patched upstream software, retain the upstream
license and copyright notices.

## ROS 2, Plus Toolkit, Qt, OpenCV, NumPy and VTK

These are external runtime/build dependencies and are not vendored in this repository. Each
component remains subject to its upstream license. The Plus Telemed installer and device
drivers must be obtained from their respective official sources.

Product names and trademarks are used only to identify interoperability requirements. No
endorsement by the respective owners is implied.
