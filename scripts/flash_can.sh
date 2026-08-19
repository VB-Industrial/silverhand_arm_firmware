#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${1:-pi@192.168.30.146}"
CAN_INTERFACE="${2:-vcan1.0}"
APP_HEX="build/RelWithDebInfo/silver_hand_firmware.hex"
UPLOADER="VBBoot/tools/flash_bootloader_socketcan.py"

JOINT_INDEX="$(sed -n 's/.*SR_JOINT_INDEX = \([0-9][0-9]*\)U.*/\1/p' Core/Inc/robot_config.h)"
if [ -z "$JOINT_INDEX" ]; then
    echo "Cannot determine SR_JOINT_INDEX from robot_config.h" >&2
    exit 1
fi
EXPECTED_NODE_ID="$((20 + JOINT_INDEX))"
NODE_ID="$EXPECTED_NODE_ID"
echo "Flashing joint $JOINT_INDEX / Cyphal node $NODE_ID from robot_config.h"
if [ ! -f "$APP_HEX" ] || [ ! -f "$UPLOADER" ]; then
    echo "Build firmware and initialize the VBBoot submodule first." >&2
    exit 1
fi

REMOTE_HEX="/tmp/silver_hand_firmware_node_${NODE_ID}.hex"
REMOTE_UPLOADER="/tmp/flash_bootloader_socketcan.py"
SSH_AUTH=()
if [ -n "${SSHPASS:-}" ]; then
    SSH_AUTH=(sshpass -e)
fi
"${SSH_AUTH[@]}" scp "$APP_HEX" "$PI_HOST:$REMOTE_HEX"
"${SSH_AUTH[@]}" scp "$UPLOADER" "$PI_HOST:$REMOTE_UPLOADER"
"${SSH_AUTH[@]}" ssh "$PI_HOST" "y r '$NODE_ID' bootloader 1 && sleep 1 && python3 '$REMOTE_UPLOADER' --hex '$REMOTE_HEX' --channel '$CAN_INTERFACE' --node-id '$NODE_ID' --id-format extended --app-end 0x08080000 --data-chunk-size 47 --inter-frame-delay-ms 3 --brs"
