# Needle Navigation Slicer module

This module embeds the existing needle control, volume preview, and recording interface
from `preview_move_capture.py` in the Slicer module panel instead of opening a separate
window. It computes `numpy.max(volume, axis=0)` across all Z slices of the selected scalar
volume and publishes the result at its original resolution on
`/ultrasound/projection/max` before Qt preview scaling.

## Load the module

1. In Slicer, open **Edit > Application Settings > Modules**.
2. Add this repository's `slicer_modules` directory to **Additional module paths**.
3. Restart Slicer and select **IGT > Needle Navigation**.
4. Select the OpenIGTLink volume under **Input ultrasound volume**.

The volume does not need to be named `Image_Reference`; selecting any scalar volume binds
it to the publisher and preview at runtime. CAN setup, homing, and
`needle_closed_loop.launch.py` remain external terminal operations.

Do not run `preview_move_capture.py` in the Python Interactor while this module is active.
Doing so may create duplicate timers, publishers, and motor commands. To preserve the
existing behavior, module initialization may publish theta=0 and d=0 commands.

```bash
ros2 topic info -v /ultrasound/projection/max
ros2 topic hz /ultrasound/projection/max
ros2 topic bw /ultrasound/projection/max
ros2 run rqt_image_view rqt_image_view
```

The verified image was `mono8`, 512 x 512, at approximately 20 Hz. The module's Rates
display reports OpenIGTLink volume RX, ROS image TX, and motor-state RX rates.
