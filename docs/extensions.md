# Slicer extensions

The closest reproduction of the verified Slicer 5.6.2 environment is obtained by
building the pinned extension commits from source rather than mixing them with current
Extension Manager binaries.

## SlicerOpenIGTLink

```bash
git clone https://github.com/openigtlink/SlicerOpenIGTLink.git \
  /path/to/SlicerOpenIGTLink
git -C /path/to/SlicerOpenIGTLink checkout \
  f806007ccc4b5c7cb6ea14979670a468b39c5a45

cmake -S /path/to/SlicerOpenIGTLink -B /path/to/SlicerOpenIGTLink-build \
  -DSlicer_DIR:PATH=/path/to/Slicer-SuperBuild/Slicer-build \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build /path/to/SlicerOpenIGTLink-build -j"$(nproc)"
```

## SlicerIGT

```bash
git clone https://github.com/SlicerIGT/SlicerIGT.git /path/to/SlicerIGT
git -C /path/to/SlicerIGT checkout c73b30761f01822081cfa86a3b95f429ef7a5389
git -C /path/to/SlicerIGT apply \
  /path/to/robotic-needle-navigation-system/patches/slicerigt_qtablewidget_include.patch

cmake -S /path/to/SlicerIGT -B /path/to/SlicerIGT-build \
  -DSlicer_DIR:PATH=/path/to/Slicer-SuperBuild/Slicer-build \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build /path/to/SlicerIGT-build -j"$(nproc)"
```

Exact configure options may vary with the selected commit and local environment. If
Slicer does not discover an extension after building it, add the corresponding loadable
or scripted module output directory to Slicer's Additional module paths.

Required capabilities are:

- **SlicerROS2**: ROS node, publisher, and subscriber MRML nodes
- **SlicerOpenIGTLink / OpenIGTLinkIF**: PlusServer IMAGE reception
- **SlicerIGT**: fiducial registration and IGT transform workflow
