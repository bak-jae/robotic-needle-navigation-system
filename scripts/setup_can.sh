#!/usr/bin/env bash
# Copyright 2026 Pusan National University, Advanced Robotics Lab
# SPDX-License-Identifier: LicenseRef-PNU-ARL-Academic-Research-Only-1.0

set -euo pipefail

can_device="${1:-can0}"
can_bitrate="${2:-1000000}"

if [[ ! "${can_device}" =~ ^can[0-9]+$ ]]; then
  echo "Refusing unexpected CAN interface name: ${can_device}" >&2
  exit 2
fi
if [[ ! "${can_bitrate}" =~ ^[0-9]+$ ]]; then
  echo "Bitrate must be numeric: ${can_bitrate}" >&2
  exit 2
fi

sudo ip link set "${can_device}" down 2>/dev/null || true
sudo ip link set "${can_device}" type can bitrate "${can_bitrate}"
sudo ip link set "${can_device}" up
ip -details link show "${can_device}"
