# Fiducial registration and transform hierarchy

This describes the post-registration state used by the current workflow. Confirm
all coordinate conventions and transform directions on the actual system before motion.

## Registration

1. Place an image fiducial on the visible needle-tip position in the ultrasound image.
2. Record the corresponding tracked/physical needle-tip point in the probe frame.
3. Use SlicerIGT Fiducial Registration to compute `ImageToProbe`.
4. Invert it to obtain `ProbeToImage`.
5. Place the dynamic needle transform and needle model under the registered transform.

The intended composition is:

```text
Image/world
└── ProbeToImage                 fixed registration transform
    └── NeedleMoveEncoder        dynamic theta/d transform from ROS state
        └── NeedleEncoderModel   displayed needle model
```

Conceptually:

```text
T(Needle→Image) = T(Probe→Image) × T(Needle→Probe)
```

Depending on how the Fiducial Registration module names moving/fixed coordinate systems,
the generated matrix may be the opposite direction. Validate by applying a known physical
motion and checking that the model moves in the correct image direction. Do not rely only
on transform node names.

## Current script responsibilities

`preview_move_capture.py` creates/updates `NeedleMoveEncoder` and
`NeedleEncoderModel`, and maps `/needle/state/theta_deg` and `/needle/state/d_mm`
to the dynamic model transform. It does not automatically discover and parent the model
under `ProbeToImage`; that scene hierarchy remains a manual post-registration step in the
current compatibility version.

Save the Slicer scene or record the exact hierarchy after successful registration. A future
version can add explicit transform selectors and validation without changing the motor topic
contract.
