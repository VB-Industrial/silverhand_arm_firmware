#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${1:-${RUKA_PI_HOST:-pi@10.77.0.4}}"
CAN_INTERFACE="${2:-${RUKA_CAN_INTERFACE:-vcan1.0}}"
APP_HEX="${3:-build/RelWithDebInfo/silver_hand_firmware.hex}"

JOINT_INDEX="$(sed -n 's/.*SR_JOINT_INDEX = \([0-9][0-9]*\)U.*/\1/p' Core/Inc/robot_config.h)"
if [ -z "$JOINT_INDEX" ]; then
    echo "Cannot determine SR_JOINT_INDEX from robot_config.h" >&2
    exit 1
fi
EXPECTED_NODE_ID="$((20 + JOINT_INDEX))"
NODE_ID="$EXPECTED_NODE_ID"
echo "Flashing joint $JOINT_INDEX / Cyphal node $NODE_ID from robot_config.h"
if [ ! -f "$APP_HEX" ]; then
    echo "Firmware HEX not found: $APP_HEX" >&2
    exit 1
fi

REMOTE_HEX="/tmp/silver_hand_firmware_node_${NODE_ID}_$$.hex"
SSH_AUTH=()
if [ -n "${SSHPASS:-}" ]; then
    SSH_AUTH=(sshpass -e)
fi
"${SSH_AUTH[@]}" scp "$APP_HEX" "$PI_HOST:$REMOTE_HEX"
"${SSH_AUTH[@]}" ssh "$PI_HOST" \
    "sudo /usr/local/sbin/ruka-flash-can --node '$NODE_ID' --channel '$CAN_INTERFACE' --hex '$REMOTE_HEX'"
