# Operation

The order below preserves the current working behavior. Keep the mechanism clear and have
a hardware emergency-stop/power removal procedure available.

## 1. Start CAN

```bash
ip link show
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
ip -details link show can0
```

The equivalent helper is `scripts/setup_can.sh can0 1000000`.

## 2. Home both motors using an authorized external procedure

The public repository intentionally does not distribute the previous Maxon example-derived
homing helper. Home node 1 and node 2 using an organization-approved tool or a procedure
implemented from documentation you are authorized to use. Confirm homing direction, current
threshold, travel limits, and emergency-stop behavior on the actual mechanism before starting
the bridge. Do not assume the two axes use the same homing direction.

## 3. Start the ROS 2 EPOS bridge

```bash
source /opt/ros/humble/setup.bash
source /path/to/ros2_ws/install/setup.bash
ros2 launch ros2_epos_cmd needle_closed_loop.launch.py
```

To use a machine-specific copy of the parameter file:

```bash
ros2 launch ros2_epos_cmd needle_closed_loop.launch.py \
  config_file:=/path/to/epos.local.yaml
```

This terminal must remain running. The launch starts `/epos_motion_bridge_node`; Slicer does
not start it in the current version.

## 4. Start Plus and connect Slicer

Start PlusServer on the Windows ArtUS PC, then create/activate an OpenIGTLinkIF client in
Slicer using the configured sender IP and port. See [`plus-artus.md`](plus-artus.md).

## 5. Register and operate

After Fiducial Registration, arrange the transform hierarchy described in
[`registration.md`](registration.md). Select **IGT > Needle Navigation**, select the received
volume, and operate the existing angle/linear controls. Do not run the legacy Python script
at the same time as the Slicer module.

## 6. Start visualization-only needle detection

In the **Needle Tracking** section, press **Start Tracker**. This launches the repository-local
ROS 2 detector and switches Slicer to the side-by-side layout. Red retains navigation overlays;
Yellow shows only ultrasound and the final red detection cross. Use **Show Original + Detection
in Slices** to restore this layout after changing views.

Press **Stop Tracker** before replacing model weights. The detector does not publish motor
commands and is not a closed-loop controller.

## 7. Verify before closed-loop experiments

```bash
ros2 node info /epos_motion_bridge_node
ros2 topic hz /needle/state/theta_deg
ros2 topic hz /needle/state/d_mm
ros2 topic hz /ultrasound/projection/max
ros2 topic echo --once /ultrasound/projection/max
ros2 topic echo /needle/tracking/result_px
ros2 topic hz /needle/tracking/overlay
```

Confirm motor direction and scaling with small commands, confirm the model follows encoder
state, and confirm the image header reports width 512, height 512, encoding `mono8`, step
512 in the currently verified configuration.
