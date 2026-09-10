# Values to customize before sharing or deploying

The code is uploadable now, but the following values are intentionally easy to change later.

| Value | Where to change | Current value/status |
|---|---|---|
| Repository owner / maintainer | `NOTICE`, `package.xml` | Pusan National University, Advanced Robotics Lab |
| Visibility | GitHub repository settings | Public |
| Plus sender IP | `config/system.local.yaml`, Slicer connector | example `192.168.0.102` |
| PlusServer port | local config and Slicer connector | not yet recorded |
| OpenIGTLink device/volume name | local config or Slicer selector | selector supports any scalar volume |
| Plus XML | future sanitized file under `config/plus/` | not yet included |
| EPOS library version/path | docs, `EPOS_CMD_ROOT`, CMake | 6.8.1.0 / standard `/opt` path |
| CAN adapter/bitrate/node IDs | `epos.yaml` | can0, 1 Mbps, nodes 1 and 2 |
| Gear ratio/direction/speed | `epos.yaml` | verified current working values |
| Motion limits and geometry | `preview_move_capture.py` | theta 0–35°, d 0–55 mm, current geometry constants |
| ROS topic names | `epos.yaml` and Slicer scripts together | current `/needle/...` contract |
| Input volume / projection mode | Slicer module UI / legacy constants | selected volume, Max |
| Recording directory | `NEEDLE_RECORD_DIR` | repository-local ignored runtime directory |
| Tracker Python | `NEEDLE_TRACKER_PYTHON` | repository-local `.venv-needle-tracker/bin/python` |
| Tracker checkpoint | `NEEDLE_MODEL` or `tracker_runtime/best_model.pt` | not included; exact SmallUNet state_dict required |
| Registration transform names/hierarchy | Slicer scene and registration doc | manual post-registration step |
| Dependency commits | `verified-environment.md` | pinned to the current machine |

The public repository does not include the Maxon example-derived homing helper. Each deployment
must document and validate its authorized homing procedure separately.

The public repository also excludes `.pt`, `.pth`, and `.ckpt` files from the tracker runtime.
Changing a checkpoint does not change the architecture: it must match the checked-in
`SmallUNet(base_channels=16)` definition exactly. Stop and restart the tracker after replacing
weights.

When changing a topic name, update both publisher and subscriber sides or remap it at launch.
When changing a motor calibration value, verify small motions without relying on the visual
model alone. When Plus settings are confirmed, add a sanitized XML example that excludes
patient data, credentials, serial numbers if sensitive, and machine-specific absolute paths.
