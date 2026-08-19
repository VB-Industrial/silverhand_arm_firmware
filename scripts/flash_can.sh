#!/usr/bin/env bash
set -euo pipefail

NODE_ID="${1:?Usage: flash_can.sh NODE_ID [PI_HOST] [CAN_INTERFACE]}"
PI_HOST="${2:-pi@192.168.30.146}"
CAN_INTERFACE="${3:-vcan1.0}"
APP_HEX="build/RelWithDebInfo/silver_hand_firmware.hex"
UPLOADER="VBBoot/tools/flash_bootloader_socketcan.py"

JOINT_INDEX="$(sed -n 's/.*SR_JOINT_INDEX = \([0-9][0-9]*\)U.*/\1/p' Core/Inc/robot_config.h)"
if [ -z "$JOINT_INDEX" ]; then
    echo "Cannot determine SR_JOINT_INDEX from robot_config.h" >&2
    exit 1
fi
EXPECTED_NODE_ID="$((20 + JOINT_INDEX))"
if [ "$NODE_ID" -ne "$EXPECTED_NODE_ID" ]; then
    echo "Refusing to flash node $NODE_ID: current image is for joint $JOINT_INDEX / node $EXPECTED_NODE_ID" >&2
    exit 1
fi
if [ ! -f "$APP_HEX" ] || [ ! -f "$UPLOADER" ]; then
    echo "Build firmware and initialize the VBBoot submodule first." >&2
    exit 1
fi

REMOTE_HEX="/tmp/silver_hand_firmware_node_${NODE_ID}.hex"
REMOTE_UPLOADER="/tmp/flash_bootloader_socketcan.py"
scp "$APP_HEX" "$PI_HOST:$REMOTE_HEX"
scp "$UPLOADER" "$PI_HOST:$REMOTE_UPLOADER"
ssh "$PI_HOST" "y r '$NODE_ID' bootloader 1 && sleep 1 && python3 '$REMOTE_UPLOADER' --hex '$REMOTE_HEX' --channel '$CAN_INTERFACE' --node-id '$NODE_ID' --id-format extended --app-end 0x08080000 --data-chunk-size 47 --inter-frame-delay-ms 3 --brs"
