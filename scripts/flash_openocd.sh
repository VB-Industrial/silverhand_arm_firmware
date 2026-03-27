#!/usr/bin/env bash
set -euo pipefail

ELF_PATH="${1:-build/RelWithDebInfo/silver_hand_firmware.elf}"
ADAPTER_SPEED_KHZ="${2:-4000}"

if ! command -v openocd >/dev/null 2>&1; then
    echo "openocd is not installed." >&2
    echo "Run: ./scripts/setup_wsl_open_source.sh" >&2
    exit 1
fi

if [ "${EUID}" -ne 0 ]; then
    echo "OpenOCD in WSL often needs root to access ST-Link over usbipd." >&2
    echo "Run with sudo: sudo ./scripts/flash_openocd.sh" >&2
    exit 1
fi

if [ ! -f "$ELF_PATH" ]; then
    echo "Firmware file not found: $ELF_PATH" >&2
    echo "Build first: cmake --preset RelWithDebInfo && cmake --build --preset RelWithDebInfo" >&2
    exit 1
fi

ELF_ABS_PATH="$(realpath "$ELF_PATH")"

echo "Flashing $ELF_ABS_PATH via OpenOCD..."
openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "transport select hla_swd" \
    -c "adapter speed ${ADAPTER_SPEED_KHZ}" \
    -c "program \"${ELF_ABS_PATH}\" verify reset exit"
