# Silver Rover Arm Firmware

Independent firmware project for a single manipulator joint controller.

Current assumptions:
- MCU family: STM32G4
- Transport: Cyphal over CAN FD
- Control target: one actuator node per joint
- Baseline reference: nearby `bldc_codex` project

Goals of this package:
- keep manipulator firmware buildable on its own
- isolate board support, transport, and motion-control logic
- make it possible to reuse the same firmware architecture across all six joints

Planned module split:
- `App/` application entry point and composition
- `Core/` STM32 platform glue and interrupt handlers
- `config/` board and joint-specific configuration
- `docs/` protocol notes and implementation decisions
- `cmake/` toolchain support

Near-term milestones:
1. Import the minimal STM32G4 build baseline.
2. Extract reusable Cyphal transport adapter.
3. Define joint command and telemetry contract.
4. Port and clean up the motor-control loop.
5. Add board profile support for all manipulator joints.

## Development environment

The supported development setup is VS Code connected to Ubuntu in WSL 2. The
firmware uses only open-source host tools: CMake, Ninja, GNU Arm Embedded
Toolchain, GDB and OpenOCD.

### Clone the repository

Clone the repository together with its submodules:

```bash
git clone --recurse-submodules https://github.com/VB-Industrial/silverhand_arm_firmware.git
cd silverhand_arm_firmware
```

If the repository was already cloned without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

### Install the WSL toolchain

The setup script installs all build, debug and flashing dependencies on
Debian/Ubuntu:

```bash
./scripts/setup_wsl_open_source.sh
```

Equivalent manual installation:

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake ninja-build \
    gcc-arm-none-eabi binutils-arm-none-eabi \
    gdb-multiarch openocd usbutils git
```

### Build from the terminal

Configure and build the default development preset:

```bash
cmake --preset RelWithDebInfo
cmake --build --preset RelWithDebInfo
```

The resulting firmware image is:

```text
build/RelWithDebInfo/silver_hand_firmware.elf
```

To discard a cache created on another machine or operating system and configure
again:

```bash
cmake -E remove_directory build/RelWithDebInfo
cmake --preset RelWithDebInfo
cmake --build --preset RelWithDebInfo
```

Other available configure/build presets are `Debug`, `Release` and
`MinSizeRel`.

## Attach ST-Link to WSL

Install `usbipd-win` on Windows if it is not installed yet:

```powershell
winget install --interactive --exact dorssel.usbipd-win
wsl --update
```

Connect ST-Link and find its `BUSID`:

```powershell
usbipd list
```

Share and attach the current ST-Link from an Administrator PowerShell:

```powershell
usbipd bind --busid 5-4; usbipd attach --wsl --busid 5-4
```

The semicolon runs the commands sequentially. Do not replace it with the
PowerShell pipeline operator `|`: `usbipd attach` does not consume the output of
`usbipd bind`.

After the device has been shared once, subsequent attachments can be performed
from a normal PowerShell while a WSL terminal is open:

```powershell
usbipd attach --wsl --busid 5-4
```

Verify the device inside WSL:

```bash
lsusb
```

The list should contain an STMicroelectronics ST-LINK device. USB devices must
be attached again after unplugging them or restarting WSL. To return the device
to Windows:

```powershell
usbipd detach --busid 5-4
```

### Allow OpenOCD access without sudo

VS Code tasks intentionally run OpenOCD without `sudo`. Install the OpenOCD
udev rules before attaching ST-Link:

```bash
rules_file="$(dpkg -L openocd | grep '/60-openocd.rules$' | head -n 1)"
sudo install -m 0644 "$rules_file" /etc/udev/rules.d/60-openocd.rules
sudo service udev restart
```

Detach and attach ST-Link again after installing the rules. If OpenOCD still
reports `LIBUSB_ERROR_ACCESS`, check that the rules file was found and restart
WSL with `wsl --shutdown` from PowerShell.

## Flash and debug

Build and flash the default image from WSL:

```bash
cmake --preset RelWithDebInfo
cmake --build --preset RelWithDebInfo
./scripts/flash_openocd.sh
```

Flash a specific ELF or select a different SWD speed in kHz:

```bash
./scripts/flash_openocd.sh path/to/firmware.elf
./scripts/flash_openocd.sh build/RelWithDebInfo/silver_hand_firmware.elf 1000
```

Start an OpenOCD server without flashing:

```bash
openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "transport select hla_swd" \
    -c "adapter speed 4000"
```

Do not add `sudo` to these commands. Fix USB/udev permissions instead.

## VS Code workflow

Open the repository root from WSL:

```bash
code .
```

Install the recommended extensions in the WSL remote environment. To install
the status-bar task buttons manually:

```bash
code --install-extension Groobz.command-buttons
```

Run `Developer: Reload Window` from the Command Palette after installing the
extensions. Extensions installed on Windows and extensions installed in WSL are
separate.

The workspace exposes these status-bar buttons through
`.vscode/command-buttons.json`:

- `Build` runs `Build firmware`.
- `Flash` runs `OpenOCD: Flash project (SWD)`.
- `Build + Flash` configures, builds and flashes sequentially.

Other useful entry points:

- `Tasks: Run Task` shows every task from `.vscode/tasks.json`.
- `Ctrl+Shift+B` runs the default `Build + Flash` task.
- `Run and Debug` provides build/debug, build/flash/debug and attach profiles.
- `Command Buttons: Reload` refreshes the status-bar buttons without restarting
  VS Code.

All VS Code build and OpenOCD tasks run inside WSL and do not invoke `sudo`.
