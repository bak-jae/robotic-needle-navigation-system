# Tested software versions

This page records the exact software combination used on the working research system.
It is a reproducibility reference, not a list of values that must be entered into the
Needle Navigation module.

How to read the table:

- **Version** is the user-facing release, such as Slicer 5.6.2.
- **Source revision** is the exact Git source snapshot used to build that component.
- **Additional repository fix** means the named file from this repository's `patches/`
  directory was applied after downloading the original source.

| Component | Tested release or purpose | Exact source revision | Additional repository fix |
|---|---|---|---|
| Ubuntu | 22.04.5 LTS | - | None |
| ROS 2 | Humble | - | None |
| 3D Slicer | 5.6.2, source-built in Release mode using system OpenSSL | `f10cd8c229b50e4d96e55743385f17482214433a` | None |
| SlicerROS2 | Connects Slicer to ROS 2 | `2abf6a9c0563fe5d2041a038fccdffedfa7cb7c3` | `slicer_ros2_humble_rosbag_services.patch` and `slicer_ros2_image_step.patch` |
| SlicerOpenIGTLink | Receives OpenIGTLink image data in Slicer | `f806007ccc4b5c7cb6ea14979670a468b39c5a45` | None |
| SlicerIGT | Provides image-guided therapy features used by the workflow | `c73b30761f01822081cfa86a3b95f429ef7a5389` | `slicerigt_qtablewidget_include.patch` |
| Maxon EPOS Command Library | 6.8.1.0 | - | Official binary installed outside the repository |
| Plus | 2.8.0 Telemed Win32 | Release commit `96b89a6` | Site-specific Plus XML is not yet recorded |

To reproduce the closest known working environment on another computer, check out the
listed source revisions and then apply only the fixes named in the last column. Newer
versions may work, but they require separate compatibility testing.

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
