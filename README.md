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

## Runtime diagnostics

Cyphal command and feedback subjects are defined per joint in `robot_config.h`:

- `1001`: shared `Planar.0.1` joint feedback; identify a joint by source node-ID.
- `1121`–`1126`: per-joint `Planar.0.1` SERVO commands from the controller.
- `1131`–`1136`: per-joint `angular_velocity.Scalar.1.0` DIRECT commands
  in joint rad/s from the controller.

The node exposes the following read-only Cyphal registers:

- `fault_active` and `fault_latched`: current and session-latched fault masks.
- `fault_level`: `0` nominal, `1` warning, `2` degraded, `3` fault.
- `stop_reason`: last commanded stop cause: `0` none, `1` network offline,
  `2` position mismatch, `3` TMC5160 fault, `4` controller offline,
  `5` velocity-command timeout.
- `network_state`: `0` starting, `1` online, `2` offline. A heartbeat from any
  node marks the network online.
- `controller_state`: `0` starting, `1` online, `2` offline. Only heartbeat
  from `controller_node_id` in `robot_config.h` updates this state. Operational
  joint commands are accepted only from this node.
- `control_mode`: `0` hold, `1` servo, `2` direct, `3` calibration.
- `cal_cmd`: write `1` to enter calibration (stop and disarm), or `2` to
  abort (stop and disarm).
- `auto_cal`: reading this trigger register starts a fully automatic low-current
  limit search and calibration; use `cal_state` for status instead of reading
  `auto_cal` again.
- `luft_cal`: reading this trigger register starts only the low-current
  backlash-rocking stage around the current position. It preserves the stored
  limits, correction table, TMC span, and geometric zero; use `cal_state` for
  status instead of reading `luft_cal` again.
- `zero_cal`: reading this trigger register at the known geometric-zero pose
  stops the motor, stores the current output-encoder raw value, and sets the
  TMC5160 position to zero. Use `cal_result` for status; every read triggers a
  new zero calibration.
- `cal_next`: write `1` to capture limit A, capture limit B, and finally start
  the automatic pass, according to the current calibration state.
- `cal_state`: `[state, error, progress_percent]`.
- `cal_result`: `[state, error, progress, limit_a_raw, limit_b_raw,
  signed_manual_span, safe_margin, point_count, tmc_span_steps,
  zero_valid, zero_raw, backlash_steps, manual_total_travel]`. The reported
  TMC span is the signed average of the forward and reverse measurement spans;
  backlash is the positive median effective lost motion from six midpoint
  reversals, expressed in TMC microsteps.
- `enc_status`: `[last_read_ok, HAL_status, transfer_count, error_count,
  raw_16bit_frame, has_valid_angle]`. The working single-frame AS50xx exchange
  is intentionally unchanged; a failed transfer preserves the last valid angle.
- `fusion_diag`: `[calibrated_encoder_angle, calibrated_tmc_angle,
  floating_offset, fused_angle, encoder_residual, measured_backlash,
  calibrated_encoder_active]`, in manipulator radians. The final element is
  `1.0` when the EEPROM correction table and geometric zero are active.
- `fault_log_count`: sequence number of the latest persistent fault record.
- `fault_log_last`: `[sequence, uptime_ms, fault_mask, GSTAT, DRV_STATUS]`.

Fault mask bits are:

| Bit | Meaning |
| --- | --- |
| 0 | Output encoder and TMC5160 position mismatch |
| 1 | TMC5160 communication failure |
| 2 | TMC5160 enable-pin readback failure |
| 3 | TMC5160 configuration readback failure |
| 4 | TMC5160 charge-pump undervoltage |
| 5 | TMC5160 overtemperature warning |
| 6 | TMC5160 overtemperature shutdown |
| 7 | TMC5160 short circuit |
| 8 | Unclassified critical TMC5160 driver status |
| 9 | EEPROM unavailable |
| 10 | EEPROM fault-log write failure |
| 11 | Velocity-command timeout (session-latched only) |

Position-mismatch evaluation remains temporarily disabled while the calibrated
joint-angle estimator is commissioned on hardware.

## Output-encoder calibration

Calibration builds and stores the data used by the runtime joint-angle
estimator: the encoder correction table, measured TMC/output scale, effective
backlash, physical limits, and geometric zero.

1. Write `cal_cmd=1`. Firmware stops and disarms the driver.
2. Move the joint by hand to either mechanical limit and write `cal_next=1`.
3. Move it by hand through the usable path to the opposite limit and write
   `cal_next=1`. Firmware tracks the signed, unwrapped encoder path, including
   crossings through `0/16383` and a mechanical range of up to two encoder
   revolutions.
4. Remove hands and fixtures, then write `cal_next=1` again. Firmware arms at
   limited current, moves back inside limit A, settles, and performs
   constant-speed measurement passes from A to B and back to A.
5. A table of up to 256 uniformly spaced `int16` correction points is checked
   for monotonicity. The mean of the normalized forward/reverse tables captures
   encoder nonlinearity.
6. Firmware moves to the middle of the range and performs four low-current
   rocking cycles between `middle - 15 degrees` and `middle + 15 degrees`.
   For a narrow joint, each side is limited to one quarter of the calibrated
   span. Each half-cycle subtracts the encoder-implied TMC travel from actual
   TMC travel; the first cycle is discarded and the median of the remaining
   six reversals is stored as effective backlash.
7. The joint returns to the middle, then the table and backlash are saved to
   redundant EEPROM slots A/B with version, sequence, CRC, valid marker, and
   readback verification. Completion and any abort leave the driver disarmed;
   reboot before normal operation.

The automatic path stays inside the manually captured physical limits. Its
margin is 2 percent of the observed span, clamped to 16...200 encoder ticks.
Writing `cal_cmd=2` aborts from any state.

### Runtime joint-angle estimate

With a valid EEPROM calibration, firmware unwraps the AS5047 reading inside the
stored physical span, interpolates the 256-point correction table, and subtracts
the corrected geometric zero. TMC position increments are converted to output
radians using the measured `tmc_span_steps`, so the motor and output encoder use
the same calibrated coordinate scale.

The estimator keeps one floating offset between the relative TMC position and
the absolute output encoder. TMC increments provide fine motion inside a
two-encoder-tick corridor. If the prediction leaves that corridor, only the
offset required to return to its edge is applied. The offset can therefore move
partially or completely as mechanical backlash is taken up; firmware does not
infer a full backlash transition from a velocity reversal. Stored
`backlash_steps` is converted to output radians and reported as the expected
offset range in `fusion_diag`, but is not added to the position unconditionally.

The fused velocity is the filtered derivative of this final angle. If the
output encoder becomes unavailable, relative tracking continues from TMC
increments; recovery reanchors the estimator to the absolute encoder without
retaining a stale offset.

For a fully automatic calibration, invoke the trigger register without a
value:

```bash
y r 26 auto_cal
```

Firmware seeks the first mechanical stop at limited current, detects it from
the absence of output-encoder motion, backs off, seeks the opposite stop, and
then runs the same measurement and EEPROM-save stages. Manual calibration
remains available as a fallback.

Calibration states are `0` idle, `1` wait limit A, `2` wait limit B, `3`
ready, `4` move to A, `5` settle, `6` sweep to B, `7` processing, `8` saving,
`9` complete, `10` failed, and `11` aborted.
Automatic discovery additionally uses `12` seek limit A, `13` back off A, and
`14` seek limit B. Bidirectional measurement uses `15` settle at B and `16`
sweep back to A. Midpoint backlash measurement uses `17` move to rocking
start, `18` settle, `19` sweep between rocking endpoints, `20` return to the
middle, and `21` settle in the middle.

To refresh only the backlash estimate after mechanical wear, place the joint
at any suitable position inside its stored safe range and invoke:

```bash
y r 26 luft_cal
```

The current position becomes the rocking center. Firmware stops any previous
motion, enables the driver if necessary, applies the same limited calibration
current, and performs four cycles with up to `+/-15 degrees` of output motion.
The amplitude is reduced symmetrically near a stored safe limit; the trigger is
rejected if fewer than eight encoder ticks are available on either side. The
first cycle is discarded and the median of the remaining six reversals replaces
only `backlash_steps`. The joint returns to the starting position, saves the
updated EEPROM record, and disarms. Reboot before normal operation.

After calibrating the limits/table, place the joint at its geometric zero and
invoke the trigger without a value:

```bash
y r 26 zero_cal
```

This command is accepted only in calibration state `0` with a valid stored
encoder calibration and a working output encoder. It stops and waits for the
motor, persists the encoder zero in the same redundant EEPROM record, then
sets the TMC5160 position and the reported joint angle to zero. Existing
version-1 and version-2 calibration records are read without losing their
limit, table, or geometric-zero data; the next successful calibration write
uses version 3.

Only overtemperature shutdowns and short circuits are written to EEPROM.
Network, communication and position-mismatch events remain session-local. The
AT24C64 fault log uses the final 2 KiB as a 64-record ring; the first 6 KiB are
reserved for future calibration data.

While the driver is enabled, firmware checks `IOIN`, `GSTAT` and `DRV_STATUS`
every 100 ms. An invalid `IOIN` response is retried immediately; if both reads
fail, firmware raises `DRV_EN` and enters the TMC communication-fault state.
Configuration registers are verified only after driver initialization or rearm.

Raw velocity commands must be refreshed at least once every 1 second (1000 ms). On timeout the
firmware commands zero velocity and keeps the driver armed. Loss of controller
heartbeat has the same stop/hold behavior; heartbeats from other nodes continue
to indicate a live network but do not keep controller state online.
