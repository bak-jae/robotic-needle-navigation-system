#!/usr/bin/env bash
# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

# Launch the detector in a Python environment that is independent from Slicer.
#
# Slicer exports PYTHONHOME and PYTHONPATH for its bundled Python 3.9 runtime.
# Keeping those variables would corrupt a Python 3.10 ROS 2 virtual environment,
# so they must be removed before sourcing ROS 2 and starting the child process.
set -eo pipefail

# Slicer ships its own Python runtime and exports PYTHONHOME/PYTHONPATH.  Those
# values make the bundled Python 3.10 tracker fail when it is launched through
# QProcess, so start the external ROS process with a clean Python environment.
unset PYTHONHOME
unset PYTHONPATH

source /opt/ros/humble/setup.bash
set -u

runtime_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
navigation_root="$(cd "${runtime_dir}/../../.." && pwd)"

# Both paths can be overridden without editing tracked files.  By default the
# virtual environment is repository-local and the authorized SmallUNet state_dict
# is expected next to this script.  The checkpoint itself is intentionally ignored.
tracker_python="${NEEDLE_TRACKER_PYTHON:-${navigation_root}/.venv-needle-tracker/bin/python}"
model_path="${NEEDLE_MODEL:-${runtime_dir}/best_model.pt}"

if ros2 node list 2>/dev/null | grep -Fxq "/needle_tracker_node"; then
  echo "Needle tracker is already running as /needle_tracker_node"
  exit 3
fi

if [[ ! -x "${tracker_python}" ]]; then
  echo "Tracker Python is missing or not executable: ${tracker_python}"
  exit 4
fi

if [[ ! -f "${model_path}" ]]; then
  echo "Tracker model is missing: ${model_path}"
  echo "Copy an authorized compatible SmallUNet state_dict to that path or set NEEDLE_MODEL."
  exit 5
fi

exec "${tracker_python}" "${runtime_dir}/ros2_needle_tracker_node.py" \
  --model "${model_path}" \
  --device "${NEEDLE_DEVICE:-cuda}" \
  "$@"
