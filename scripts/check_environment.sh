#!/usr/bin/env bash
# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

set -u

status=0

check_command() {
  if command -v "$1" >/dev/null 2>&1; then
    printf 'OK   %-18s %s\n' "$1" "$(command -v "$1")"
  else
    printf 'MISS %-18s\n' "$1"
    status=1
  fi
}

check_command ros2
check_command colcon
check_command cmake
check_command ip

if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  printf 'INFO OS                 %s\n' "${PRETTY_NAME:-unknown}"
fi

epos_root="${EPOS_CMD_ROOT:-/opt/EposCmdLib_6.8.1.0}"
if [[ -f "${epos_root}/include/Definitions.h" ]]; then
  printf 'OK   EPOS header        %s\n' "${epos_root}/include/Definitions.h"
else
  printf 'MISS EPOS header        expected under %s\n' "${epos_root}"
  status=1
fi

if [[ -e /sys/class/net/can0 ]]; then
  printf 'OK   CAN interface      can0\n'
else
  printf 'WARN CAN interface      can0 not present\n'
fi

exit "${status}"
