#!/usr/bin/env bash
set -euo pipefail

FIRMWARE_PATH="${1:-build/RelWithDebInfo/silver_hand_firmware_full.bin}"
ADAPTER_SPEED_KHZ="${2:-4000}"

if ! command -v openocd >/dev/null 2>&1; then
    echo "openocd is not installed." >&2
    echo "Run: ./scripts/setup_wsl_open_source.sh" >&2
    exit 1
fi

if [ ! -f "$FIRMWARE_PATH" ]; then
    echo "Firmware file not found: $FIRMWARE_PATH" >&2
    echo "Build first: cmake --preset RelWithDebInfo && cmake --build --preset RelWithDebInfo" >&2
    exit 1
fi

FIRMWARE_ABS_PATH="$(realpath "$FIRMWARE_PATH")"

PROGRAM_ARGUMENT="\"${FIRMWARE_ABS_PATH}\""
case "$FIRMWARE_PATH" in
    *.bin) PROGRAM_ARGUMENT="${PROGRAM_ARGUMENT} 0x08000000" ;;
esac

echo "Flashing $FIRMWARE_ABS_PATH via OpenOCD..."
openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "transport select hla_swd" \
    -c "adapter speed ${ADAPTER_SPEED_KHZ}" \
    -c "program ${PROGRAM_ARGUMENT} verify reset exit"
