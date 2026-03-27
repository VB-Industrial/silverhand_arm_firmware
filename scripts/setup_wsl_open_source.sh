#!/usr/bin/env bash
set -euo pipefail

if ! command -v apt-get >/dev/null 2>&1; then
    echo "This script supports Debian/Ubuntu (apt-get)." >&2
    exit 1
fi

PACKAGES=(
    build-essential
    cmake
    ninja-build
    gcc-arm-none-eabi
    binutils-arm-none-eabi
    gdb-multiarch
    openocd
    usbutils
    git
)

echo "Installing open-source STM32 toolchain packages:"
printf '  - %s\n' "${PACKAGES[@]}"

sudo apt-get update
sudo apt-get install -y "${PACKAGES[@]}"

echo
echo "Installed tool versions:"
cmake --version | head -n 1
ninja --version
arm-none-eabi-gcc --version | head -n 1
gdb-multiarch --version | head -n 1
openocd --version 2>&1 | head -n 1

echo
echo "Done. Next steps:"
echo "  1) Build firmware: cmake --preset RelWithDebInfo && cmake --build --preset RelWithDebInfo"
echo "  2) Attach ST-Link to WSL (usbipd on Windows side)."
echo "  3) Flash firmware: ./scripts/flash_openocd.sh"
