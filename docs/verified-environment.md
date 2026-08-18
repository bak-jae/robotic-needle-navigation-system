# Verified environment and reproducibility pins

The following values were inspected on the original working system. On a new computer,
check out the same commits and apply the repository patches to reproduce the closest
known source state.

| Component | Version / commit | Local change |
|---|---|---|
| Ubuntu | 22.04.5 LTS | none |
| ROS 2 | Humble | none |
| 3D Slicer | 5.6.2, `f10cd8c229b50e4d96e55743385f17482214433a` | system OpenSSL, Release build |
| SlicerROS2 | `2abf6a9c0563fe5d2041a038fccdffedfa7cb7c3` | `slicer_ros2_humble_rosbag_services.patch`, `slicer_ros2_image_step.patch` |
| SlicerOpenIGTLink | `f806007ccc4b5c7cb6ea14979670a468b39c5a45` | none |
| SlicerIGT | `c73b30761f01822081cfa86a3b95f429ef7a5389` | `slicerigt_qtablewidget_include.patch` |
| Maxon EPOS Command Library | 6.8.1.0 | official binary installed outside the repository |
| Plus | 2.8.0 Telemed Win32, release commit `96b89a6` | site-specific Plus XML not yet recorded |

The SHA-256 observed for the original EPOS Linux archive was
`467df7a69d67ae02d976c23a807e6bc4c00e4635e06d1330d29637868aa335b5`.
This value identifies the locally used archive; it does not authorize or imply
redistribution of any vendor file.

The previous Maxon example-derived homing helper is excluded from the public source tree.

## Local-only Slicer source change

The original Slicer source tree contained a local development-version string change in
`Utilities/Scripts/SlicerWizard/__version__.py`. It did not affect the runtime workflow
and is therefore not included as a patch.

## Why pin commits?

A branch such as `main` changes over time. A commit hash records an exact revision,
reducing extension API and build-condition drift and making it possible to return to the
same source state during troubleshooting. Update the pins and patches together after
validating a newer environment.
