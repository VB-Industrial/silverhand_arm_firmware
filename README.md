# Silver Rover Arm Firmware

Firmware for the six distributed RUKA2 joint controllers. Every joint runs the
same STM32G474 application; the active hardware and kinematic profile is
selected by `SR_JOINT_INDEX` in `Core/Inc/robot_config.h` before building.
Joint indices 1 through 6 correspond to Cyphal node IDs 21 through 26.

The application controls a TMC5160 stepper driver, reads an absolute output
encoder, persists calibration in AT24C64 EEPROM, and communicates with the arm
controller over Cyphal/CAN FD. VBBoot occupies the first 12 KiB of internal
flash and allows later application updates over CAN without an ST-Link.

## Main features

- TMC5160-owned position ramps for smooth and repeatable motion.
- MoveIt trajectory input with position, velocity feed-forward, acceleration,
  and low-speed final positioning.
- Independent DIRECT velocity control for service and controller use.
- Hybrid joint-state estimate: absolute encoder localization and external
  motion detection combined with repeatable TMC incremental motion.
- Manual physical-span calibration, backlash measurement, and separate
  geometric-zero calibration.
- Separate physical calibration limits, logical hard limits, and braking-aware
  soft limits.
- Startup recovery when a calibrated joint powers up outside its logical range.
- Latched motor-slip and TMC fault handling with explicit acknowledgement.
- Cyphal diagnostics for control, encoder, TMC, EEPROM, boot, limits, faults,
  and firmware version.
- Initial/recovery flashing over SWD and routine application updates through
  VBBoot over CAN FD.

## Joint configuration

`kRobotJointProfiles` in `Core/Inc/robot_config.h` is the authoritative table
for node/subject IDs, motor direction, encoder direction, gearing, currents,
speed limits, and logical travel limits. Always set `SR_JOINT_INDEX` to the
physical joint being built. The CAN flashing script derives the destination
node from this value, preventing a build configured for one joint from being
silently sent to another node.

The manually maintained version is exposed both by `uavcan.node.GetInfo` and
the `version` register:

```text
2.<repository commit number>.<uncommitted trial number>
```

Update `SR_FIRMWARE_VERSION_MAJOR`, `SR_FIRMWARE_VERSION_MINOR`, and
`SR_FIRMWARE_VERSION_TRIAL` before distributing a new image. The fourth value
in the `version` register is the compiled joint index.

Current motion/current profiles are:

| Joint | Node | IHOLD | IRUN | SERVO/DIRECT limit, rad/s |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 21 | 2 | 6 | 0.7 |
| 2 | 22 | 3 | 6 | 0.5 |
| 3 | 23 | 2 | 6 | 0.5 |
| 4 | 24 | 1 | 3 | 1.0 |
| 5 | 25 | 1 | 3 | 1.0 |
| 6 | 26 | 1 | 3 | 1.0 |

## Runtime modes

The active mode is reported in `pos_get` and `control_diag`:

| Value | Mode | Purpose |
| ---: | --- | --- |
| 0 | HOLD | Driver holds its current TMC position; no active motion target. |
| 1 | SERVO | Absolute joint target from MoveIt/Cyphal or `pos_set`. |
| 2 | DIRECT | Joint velocity from the Cyphal velocity subject or `move`. |
| 3 | CALIBRATION | Physical-span, backlash, or zero calibration activity. |
| 4 | TMC_POSITION | Raw TMC target from `tmc_pos_set` or internal recovery. |

Feedback is independent of the command source. The same hybrid estimator and
the same fused joint angle remain active in SERVO, DIRECT, and HOLD. When the
driver is disarmed, deliberate hand motion is followed from the absolute
encoder and slip detection is disabled.

### Common service operations

Replace node 26 with the required joint node:

```bash
# Inspect firmware, position, limits, and faults.
y r 26 version
y r 26 pos_get
y r 26 limits
y r 26 errors

# Move to an absolute joint angle in radians at the service speed.
y r 26 pos_set 0.5

# Disarm/rearm and acknowledge a recoverable latched fault.
y r 26 arm 0
y r 26 arm 1
y r 26 fail_ack 1
```

`pos_set`, calibration, `move`, and `tmc_pos_set` are service/debug interfaces.
Normal coordinated operation uses the per-joint Cyphal SERVO subjects from
controller node 100. Keep the arm mechanically supported before disarming a
loaded joint, calibrating, or entering the bootloader.

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

The build produces three useful images:

```text
build/RelWithDebInfo/silver_hand_firmware.elf
build/RelWithDebInfo/silver_hand_firmware.hex
build/RelWithDebInfo/silver_hand_firmware_full.bin
```

The ELF is intended for debugging, the HEX contains only the application for
CAN updates, and `full.bin` combines VBBoot with the application for initial
installation or SWD recovery.

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

## Initial installation and SWD recovery

The first installation of VBBoot, recovery from a damaged bootloader, and
low-level debugging require ST-Link. Build and flash the combined image at
`0x08000000` from WSL:

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
- `Flash ST-Link` writes the combined VBBoot/application image over SWD.
- `Flash CAN` builds the selected joint and updates its application through
  the Raspberry Pi and VBBoot.
- `Build + Flash` configures, builds, and flashes through ST-Link.

Other useful entry points:

- `Tasks: Run Task` shows every task from `.vscode/tasks.json`.
- `Ctrl+Shift+B` runs the default `Build + Flash` task.
- `Run and Debug` provides build/debug, build/flash/debug and attach profiles.
- `Command Buttons: Reload` refreshes the status-bar buttons without restarting
  VS Code.

All VS Code build and OpenOCD tasks run inside WSL and do not invoke `sudo`.

## Runtime diagnostics

The following additional registers are read-only and intended for fleet-wide
inspection after flashing:

- `version` (`integer32[4]`): `[major, minor, trial, joint_index]`.
- `limits` (`real32[9]`): `[localized, calibration_physical_lower,
  logical_hard_lower, soft_lower, soft_upper, logical_hard_upper,
  calibration_physical_upper, startup_recovery_state,
  startup_recovery_target]`. The physical limits are derived from the corrected
  encoder calibration table and its stored zero; the logical limits come from
  `robot_config.h`.
- `control_diag` (`real32[8]`): `[control_mode, target_position,
  position_error, command_velocity, tmc_target_steps, position_reached,
  command_age_ms, servo_state]`.
- `tmc_diag` (`integer32[11]`): `[GSTAT, DRV_STATUS, IOIN, GCONF, CHOPCONF,
  IHOLD_IRUN, RAMPMODE, XACTUAL, XTARGET, VACTUAL, RAMPSTAT]`.
- `boot_diag` (`integer32[5]`): `[uptime_ms, reset_reason,
  tmc_initialize_count, tmc_enable_count, tmc_disable_count]`.
- `cyphal_diag` (`integer32[6]`): `[accepted_rx_count, periodic_tx_count,
  servo_command_age_ms, direct_command_age_ms,
  controller_heartbeat_age_ms, last_command_source_node]`. An age is zero until
  the corresponding message has first been received.
- `encoder_diag` (`real32[12]`): `[raw, corrected_ticks, calibrated_angle,
  read_count, error_count, maximum_frame_delta, rejected_spikes,
  persistent_residuals, hybrid_state, backlash_rad, innovation_rad,
  encoder_weight]`.
- `eeprom_diag` (`integer32[6]`): `[connected, valid_slot_mask, active_slot,
  sequence, last_save_ok, saves_since_boot]`; active slots are `1=A`, `2=B`.
- `fault_history` (`integer32[6]`): the most recent persistent ring-log entry as
  `[valid, sequence, uptime_ms, fault_mask, GSTAT, DRV_STATUS]`.
- `bootloader` (`integer32`): write `1` to stop the motor, preserve this joint's
  CAN transport settings in STM32 backup registers, and reboot into VBBoot.

## CAN bootloader and firmware updates

### Boot layout and behavior

The normal application is linked at `0x08003000`; the first 12 KiB are reserved
for VBBoot. A regular build produces:

- `silver_hand_firmware.hex` — application-only image used for later CAN
  updates;
- `silver_hand_firmware_full.bin` — VBBoot plus application, used for the first
  installation with ST-Link.

Initialize submodules and build both images:

```bash
git submodule update --init --recursive
cmake --preset RelWithDebInfo
cmake --build --preset RelWithDebInfo
```

On every reset, VBBoot starts at `0x08000000`. With no update request it
validates and jumps to the application at `0x08003000`. Writing `1` to the
application's `bootloader` register stops the motor, preserves the node's CAN
transport settings in backup registers, and performs a software reset. VBBoot
then stays active and accepts an application image addressed to that node.

Install `build/RelWithDebInfo/silver_hand_firmware_full.bin` once at
`0x08000000`. Subsequent updates replace only the application and are performed
for one joint at a time. For example, for node 26:

```bash
y r 26 bootloader 1
python3 VBBoot/tools/flash_bootloader_socketcan.py \
  --hex build/RelWithDebInfo/silver_hand_firmware.hex \
  --channel vcan1.0 --node-id 26 --id-format extended \
  --app-end 0x08080000 --data-chunk-size 47 \
  --inter-frame-delay-ms 3 --brs
```

The bootloader validates the complete image size and CRC32 before jumping to
the new application. If transfer or validation fails, it remains in boot mode so the
same node can be flashed again. Do not use the combined `full.bin` as the CAN
update payload. The uploader sends 47 application bytes per 48-byte CAN FD
frame. The Ethernet-CAN bridge used by RUKA2
does not currently preserve the maximum 64-byte payload reliably.

The default streaming update omits per-frame ACKs, waits 3 ms between DATA
frames, and validates the complete image size and CRC32 at `DONE`. The legacy
per-frame-confirmed mode remains available with `--ack-each-frame`.

If an update is interrupted, rerun the same upload for that node; no ST-Link is
needed while VBBoot itself remains intact. Use ST-Link and `full.bin` only when
the node cannot enter/respond in bootloader mode or the bootloader needs to be
replaced.

### Remote update through the RUKA Raspberry Pi

The Raspberry Pi keeps the CAN uploader at
`/opt/ruka2/firmware-tools/flash_bootloader_socketcan.py` and exposes the
validated, serialized update command `/usr/local/sbin/ruka-flash-can`.
`scripts/flash_can.sh` uploads only the locally-built application HEX and then
invokes that command.  By default the Pi is addressed over WireGuard as
`pi@10.77.0.4`:

```bash
./scripts/flash_can.sh
```

Override the destination and CAN interface when needed:

```bash
RUKA_PI_HOST=pi@192.168.30.146 RUKA_CAN_INTERFACE=vcan1.1 \
  ./scripts/flash_can.sh
```

The script configures and rebuilds the firmware, derives the Cyphal node-ID
from `SR_JOINT_INDEX`, uploads
`build/RelWithDebInfo/silver_hand_firmware.hex`, and never accepts the combined
first-installation image. The Pi keeps timestamped copies and a per-node
`latest` symlink in `/var/lib/ruka2-firmware/`; concurrent updates are rejected.

### Recommended CAN update sequence

1. Set `SR_JOINT_INDEX` and the firmware version in `robot_config.h`.
2. Ensure the Raspberry Pi can see the target node and that the arm is in a
   mechanically safe position.
3. Run `./scripts/flash_can.sh` or press `Flash CAN` in VS Code.
4. Confirm `version`, `boot_diag`, and `errors` after the node returns.

The script requests bootloader mode automatically through the remote wrapper;
the explicit `y r <node> bootloader 1` command is useful for diagnosis and
manual uploader operation.

## Cyphal interface

### Subjects

Cyphal command and feedback subjects are defined per joint in `robot_config.h`:

- `1001`: shared `Planar.0.1` joint feedback; identify a joint by source node-ID.
- `1121`–`1126`: per-joint `Planar.0.1` SERVO commands from the controller.
- `1131`–`1136`: per-joint `angular_velocity.Scalar.1.0` DIRECT commands
  in joint rad/s from the controller.

Only commands from `controller_node_id` (currently node 100) are accepted on
the controller subjects. The shared feedback subject is distinguished by the
source node ID.

### Registers

The node exposes the following service, control, and diagnostic registers.

The standard `uavcan.node.GetInfo.1.0` response exposes the manually maintained
firmware build as `2.<commit-number>`. `software_vcs_revision_id` contains the
uncommitted development-trial number. The three values are hardcoded as
`SR_FIRMWARE_VERSION_MAJOR`, `SR_FIRMWARE_VERSION_MINOR`, and
`SR_FIRMWARE_VERSION_TRIAL` in `robot_config.h`; update them before flashing a
new firmware series. This makes mixed joint firmware visible in a Cyphal node
monitor without a custom diagnostic register.

- `pos_get` (read-only `real32[36]`): `[encoder_raw, tmc_raw_steps,
  encoder_angle, tmc_angle, fusion_offset, fused_angle, fused_velocity,
  encoder_residual, measured_backlash, control_mode, calibrated,
  hard_lower, soft_lower, soft_upper, hard_upper, allowed_velocity_min,
  allowed_velocity_max, calibration_state, calibration_progress,
  innovation_rad, applied_correction_rad, slip_window_residual_rad,
  rejected_spike_count, persistent_residual_count, slip_candidate,
  encoder_raw_min, encoder_raw_max, encoder_raw_span,
  encoder_maximum_frame_delta, hybrid_state, takeup_tmc_travel_rad,
  takeup_encoder_travel_rad, encoder_weight, slip_latched,
  startup_recovery_state, startup_recovery_target]`. Angles are
  in manipulator radians and velocities in rad/s. Fields 19 through 24 are
  non-blocking fusion/slip diagnostics. Fields 25 through 28 accumulate raw
  encoder statistics on every 10-ms motor cycle since boot; min/max are in an
  unwrapped coordinate. Fields 29 through 33 report the hybrid state,
  current take-up travel, encoder weight, and latched slip. Fields 34 and 35
  report startup recovery state and its target angle. Calibration state and
  progress remain at indices 17 and 18. `control_mode` is `0` hold, `1` servo,
  `2` direct, `3` calibration, or `4` raw TMC position. Raw integer positions
  and counters are encoded as `real32` because Cyphal register arrays cannot
  mix element types.

On startup, a calibrated joint that is already inside its operational soft
limits remains exactly where it is. If its absolute encoder position is still
inside the calibrated table but outside an operational soft limit, the joint
moves inward to the nearest soft limit at `0.02 rad/s`. Normal motion commands
are blocked until this recovery finishes. An encoder position that cannot be
localized, or a recovery whose TMC target is reached without the encoder
returning inside the operational limits, remains blocked for manual recovery.
`startup_recovery_state` is `0` checking, `1` already in range, `2` returning
from the lower side, `3` returning from the upper side, `4` complete,
`5` unlocalized, or `6` failed.

The physical encoder span and the operational limits are intentionally separate.
The EEPROM calibration table contains the two measured mechanical endpoints and
the encoder map between them. `robot_config.h` contains per-joint
`logical_hard_lower_rad`, `logical_hard_upper_rad`, and
`soft_limit_margin_rad`, all expressed in joint radians relative to the stored
zero. A logical range that does not fit inside the physical calibrated span is
rejected as an invalid limit envelope.
- `pos_set`: writing an absolute manipulator position in radians (`real32`)
  starts a TMC-owned position segment at `0.2 rad/s`. The target is converted
  from the current fused angle into an exact relative TMC step displacement.
  Once the ramp reports `position_reached`, residual fused-angle error is
  removed with short position segments at `0.04 rad/s`. Four/eight encoder-tick
  hysteresis prevents stationary noise from causing continuous corrections.
  This service command does not require a command stream or controller
  heartbeat. DIRECT mode, a fault, calibration, disarm, or reset cancels it.
- `luft_cal`: reading this trigger register starts only the low-current
  backlash-rocking stage around the current position. It preserves the stored
  limits, correction table, TMC span, and geometric zero.
- `zero_set`: reading this trigger register at the known geometric-zero pose
  stops the motor, stores the current output-encoder raw value, and sets the
  TMC5160 position to zero. Every read triggers a new zero calibration.
- `move`: debug DIRECT velocity command in motor microsteps/s. Use `--` before
  a negative value, for example `y r 26 move -- -20000`; write zero to stop.
- `tmc_pos_set`: debug absolute TMC5160 position target as an `int32` number of
  microsteps, for example `y r 26 tmc_pos_set 120000`. The exact integer is
  written to `XTARGET` without conversion through radians or `float`; reading
  returns the current `XACTUAL`. Driver, fault, calibration, and calibrated hard
  limit checks remain active. The move uses the configured SERVO velocity but
  deliberately does not perform fused-angle settling afterward.
- `arm`: write scalar `0` to disarm or `1` to arm the TMC5160 driver. Reading
  returns `int32[5]`: `[enabled, hybrid_state, encoder_weight_milli,
  slip_candidate, slip_latched]`. This is a service/debug control; normal
  startup arms automatically.
- `fail_ack`: write `1` to acknowledge a recoverable session-latched fault.
- `errors` (read-only `int32[25]`): `[active_mask, latched_mask, fault_level,
  stop_reason, network_state, controller_state, tmc_state, tmc_error,
  reset_reason, encoder_read_ok, encoder_hal_status, encoder_transfer_count,
  encoder_error_count, encoder_raw_frame, encoder_valid, calibration_state,
  calibration_error, eeprom_log_sequence, eeprom_log_uptime,
  eeprom_log_fault_mask, eeprom_log_gstat, eeprom_log_drv_status,
  tmc_health_read_failure_count, tmc_enable_readback_mismatch_count,
  tmc_critical_status_count]`. The final three counters accumulate transient
  TMC health observations since boot. A failed health read is diagnostic only:
  it does not remove phase current or reinitialize the driver.

The full TMC5160 initialization sequence is executed only once after MCU boot.
Runtime recovery never rewrites chopper, current, or ramp configuration. It
verifies that the powered driver retained its static configuration and, after a
cleared critical condition, only reasserts `DRV_ENN`. A configuration mismatch
remains faulted until the joint is reset.

`fault_level` is `0` nominal, `1` warning, `2` degraded, or `3` fault.
`stop_reason` is `0` none, `1` network offline, `2` motor slip, `3` TMC5160
fault, `4` controller offline, or `5` velocity-command timeout. Network and
controller states are `0` starting, `1` online, or `2` offline. Reset-reason
bits are: 0 external/reset pin, 1 brownout, 2 software, 3 IWDG, 4 WWDG,
5 low-power, and 6 option-byte reload.

Fault mask bits are:

| Bit | Meaning |
| --- | --- |
| 0 | Confirmed motor slip: encoder/TMC motion mismatch (latched) |
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

## SERVO control

The per-joint `Planar.0.1` command is interpreted as a trajectory point. The
position is always the authoritative absolute target, while the absolute
angular velocity supplied in the message is feed-forward. While the command
stream is alive, firmware passes that speed to the TMC5160 without adding an
output-position-error correction. The TMC therefore owns the position ramp and
braking toward the final target while MoveIt shapes its velocity. The
acceleration field selects the TMC5160 ramp acceleration and deceleration. Zero
requests the maximum default `A1/AMAX/DMAX/D1` profile; a finite positive value
is converted from joint rad/s^2 to motor microsteps/s^2. Negative and non-finite
acceleration commands are rejected. DIRECT velocity commands use their separate
`1131`-`1136` subjects.

Repeated commands containing the same final position remain in feed-forward
tracking and update its velocity. Firmware remembers the peak feed-forward
speed. When MoveIt publishes zero speed, or the stream is silent for 0.5 second,
the TMC completes its existing position segment at
`max(0.02 rad/s, peak_velocity/5)`. After `position_reached`, firmware checks the
fused angle and, if necessary, issues relative TMC position corrections at the
same finishing speed. Per-joint SERVO and DIRECT
velocity caps are configured by `maximum_servo_velocity_rad_s` and
`maximum_direct_velocity_rad_s` in `robot_config.h`; the current per-joint caps
are `[0.7, 0.5, 0.5, 1.0, 1.0, 1.0] rad/s` for joints 1 through 6.
It stops inside four output-encoder ticks and resumes correction only after the
output moves farther than eight ticks. This hysteresis prevents stationary
encoder noise from making the motor hunt around a TMC position that has already
been reached. The position error is not angle-wrapped because the public joint
coordinate intentionally covers `-2*pi` through `+2*pi`.

The `pos_set` register uses the same TMC-owned position-segment logic at a fixed
`0.2 rad/s`; any fused-angle correction segments run at `0.04 rad/s`. It is a
service target and does not require controller heartbeat. A fault, DIRECT
command, another position command, disarm, or calibration cancels it. A missing
SERVO update does not raise the DIRECT velocity-command-timeout fault.

## Output-encoder calibration

Calibration builds and stores the physical data used by the runtime joint-angle
estimator: the encoder correction table, measured TMC/output scale, effective
backlash, both mechanical endpoints, and a provisional zero. Calibration is performed with
the five-step `man_cal` command; replace node ID `21` below as appropriate.

1. Run `y r 21 man_cal 1`. Firmware stops, disarms the driver, and enters
   calibration mode. The expected return value is `2`.
2. Move the joint by hand to the first mechanical limit and run
   `y r 21 man_cal 2`. The expected return value is `3`.
3. Move the joint by hand to an approximate zero, normally near the middle of
   its travel, and run `y r 21 man_cal 3`. The expected return value
   is `4`.
4. Move the joint by hand to the second mechanical limit and run
   `y r 21 man_cal 4`. The expected return value is `5`.
5. Remove hands and fixtures, then run `y r 21 man_cal 5`. The expected return
   value is `6`; firmware arms at limited current and starts calibration.

Commands are accepted only in this order. Any error or an out-of-order command
returns `0`. After stage 5 the motor first moves to a safe point near the second
limit, absorbing spring-back from the hand-loaded endpoint, and then measures
from the second limit toward the first. The safe margin keeps motor motion away
from both physical stops, so it is normal that the motor does not touch them.

Firmware then builds the correction table, measures backlash with four short
rocking cycles near the captured middle, returns to that middle, and saves the
result to redundant EEPROM slots with CRC and readback verification. The first
rocking cycle is discarded; the median of the remaining six reversals is stored
as effective backlash. Completion and any abort leave the driver disarmed.

Monitor progress with `y r 21 pos_get`: elements 17 and 18 are calibration
state and progress in percent. State `9` with progress `100` means success;
state `10` means failure. Reboot the joint after a successful save before
normal operation. Stage 3 supplies the initial coordinate reference so the
table can be used immediately. Put the assembled robot at its exact geometric
zero and call `zero_set` separately to store the final raw encoder zero in
EEPROM. This shifts the physical endpoints in logical coordinates without
changing the physical table or the logical limits in `robot_config.h`.

Calibration states are `0` idle, `1` wait limit A, `2` wait limit B, `3`
ready, `4` move to A, `5` settle, `6` sweep to B, `7` processing, `8` saving,
`9` complete, `10` failed, and `11` aborted. Manual capture additionally uses
`22` wait for middle and `23` move to the safe start near limit B. Backlash
measurement uses `17` move to rocking start, `18` settle, `19` sweep between
rocking endpoints, `20` return to middle, and `21` settle in the middle.

Before calibrated limits are available, `move_cal` can jog raw TMC velocity:

```bash
y r 23 arm 1
y r 23 move_cal 10000
y r 23 move_cal 0
```

`move_cal` intentionally ignores travel limits. Use only low velocities while
watching the joint. The driver must already be armed, fault checks remain
active, and a nonzero jog stops unless refreshed within one second.

`cal_data` reports the stored scale for checking `joint_full_steps`:

```text
[valid, manual_span_ticks, tmc_span_steps, measured_full_steps,
 configured_full_steps, backlash_steps, point_count, safe_margin_ticks]
```

`measured_full_steps` is calculated from the measured TMC travel and the safe
encoder span, excluding the margins at both physical limits.

### Runtime joint-angle estimate

With a valid EEPROM calibration, firmware unwraps the AS5047 reading inside the
stored physical span, interpolates the 256-point correction table, and subtracts
the corrected geometric zero. TMC position increments are converted to output
radians using the measured `tmc_span_steps`, so the motor and output encoder use
the same calibrated coordinate scale.

The hybrid estimator anchors the initial absolute angle to the calibrated
output encoder and uses one prediction/correction model. Its prediction is the
previous fused angle plus the calibrated TMC increment. The accepted encoder
innovation is applied with a state-dependent weight: zero for consistent motion
with backlash locked, and one for confirmed encoder-only motion or backlash
take-up. Fused velocity is always the filtered derivative of that same final
angle; there is no separate velocity estimator branch.

With the driver disarmed, TMC motion is not trusted. Encoder motion beyond the
spike/deadband gate updates the physical angle, while small stationary noise is
ignored and slip detection is disabled. Arming reanchors the estimate to the
current encoder pose and resets backlash direction to unknown. The first motor
motion or a direction reversal is therefore tracked from the encoder until
16 consistent frames (160 ms) confirm that backlash has been taken up. The
encoder/TMC offset is averaged across those frames instead of being captured
from one noisy sample; subsequent consistent movement is integrated exactly
from TMC increments for repeatability.

During locked motor motion, a rolling encoder/TMC residual above eight encoder
ticks for six accepted frames (60 ms) switches the estimate to encoder-only
`MOTION_MISMATCH`. This source change is deliberately separate from the much
larger, slower threshold that declares a motor slip.

Hybrid states are `0` unknown, `1` positive take-up, `2` positive locked, `3`
negative take-up, `4` negative locked, `5` motion mismatch, and `6` manual or
encoder-only motion. An encoder increment larger than 24 ticks must remain
direction- and magnitude-consistent for six frames (60 ms) before it is
accepted; an encoder jump exceeding the physical-rate gate is rejected. With
stationary TMC, external motion is recognized after the encoder span within a
rolling 500-ms stationary window exceeds 24 ticks for six frames (60 ms), so
the observed idle span of about 20 ticks does not masquerade as hand movement
while a backlash rocking span does. Once in `MANUAL`, the angle follows only
the encoder through a 100-ms first-order filter; the entry threshold no longer
quantizes subsequent forward or reverse motion. A 500-ms encoder window no
wider than 24 ticks confirms rest, freezes the fused angle, and returns the
state to direction-neutral `UNKNOWN`. The next motor motion must therefore pass
through backlash take-up before either directional lock is asserted.

The absolute floating offset is reported continuously by `pos_get` for logging.
The legacy blocking check against an absolute fusion offset remains disabled;
fault bit 0 is now driven only by the windowed motion-mismatch detector.

The slip detector compares accepted output-encoder increments with accumulated
TMC increments in a rolling diagnostic window. While armed and moving, a
residual above ten times the calibrated joint backlash for six consecutive
evaluations (60 ms) latches motor-slip
fault bit 0, stops motion, and blocks subsequent motion commands. The fault can
be cleared only by `fail_ack 1` or a node reset. A successful acknowledgement
reanchors the estimator to the current encoder angle and resets backlash state;
a persistent mismatch will latch again. Disarmed/manual movement never raises
this fault. `rejected_spike_count`, signed window residual, encoder weight, and
the latest applied correction are reported for tuning.

The fused velocity is the filtered derivative of this final angle. If the
output encoder becomes unavailable, relative tracking continues from TMC
increments; recovery reanchors the estimator to the absolute encoder without
retaining a stale offset.

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
y r 26 zero_set
```

This command is accepted only in calibration state `0` with a valid stored
encoder calibration and a working output encoder. It stops and waits for the
motor, persists the encoder zero in the same redundant EEPROM record, then
sets the TMC5160 position and the reported joint angle to zero. Existing
version-1 and version-2 calibration records are read without losing their
limit, table, or geometric-zero data; the next successful calibration write
uses version 3.

Only overtemperature shutdowns and short circuits are written to EEPROM.
Network, communication and motor-slip events remain session-local. The
AT24C64 fault log uses the final 2 KiB as a 64-record ring; the first 6 KiB are
reserved for future calibration data.

While the driver is enabled, firmware checks `IOIN`, `GSTAT` and `DRV_STATUS`
every 100 ms. Transient failed health reads are counted and require three
consecutive failures before reporting a communication diagnostic; they do not
disarm or reinitialize the driver. Configuration registers are verified only
after driver initialization or rearm.

Raw DIRECT velocity commands must be refreshed at least once every 1 second
(1000 ms). On timeout the firmware commands zero velocity and keeps the driver
armed. Loss of controller heartbeat has the same stop/hold behavior; heartbeats
from other nodes continue to indicate a live network but do not keep controller
state online.

## Joint travel limits

With valid stored encoder calibration and geometric zero, normal SERVO and
DIRECT control use `logical_hard_lower_rad` and `logical_hard_upper_rad` from
the selected `robot_config.h` profile as hard limits. Without a complete EEPROM
calibration, or if those logical limits do not fit inside the physical table,
all normal motion commands are rejected and the reported velocity bounds are
`0...0`; only calibration may move the motor. `soft_limit_margin_rad` defines
an inner soft boundary at each logical end. Outward
DIRECT velocity is linearly reduced from the selected joint profile's
`maximum_direct_velocity_rad_s` at the soft boundary to zero at the hard limit.
SERVO uses the same envelope with `maximum_servo_velocity_rad_s`. Inward
velocity remains available at full speed. If the measured
position is already outside a hard limit, all farther
outward motion is blocked while recovery motion toward the valid range remains
allowed.

The configured soft margin is expanded when necessary so the soft zone is
at least the conservative braking distance `v^2/(2*a) + v*t_reaction`, using
`a = 1 rad/s^2` and `t_reaction = 20 ms`. A quarter of the logical travel is
the maximum expansion on very narrow joints.

Position targets outside the configured logical hard range are rejected. Velocity limiting is
recomputed from the fused output position on every motor update, including
between received DIRECT commands, so a silent sender cannot continue driving
through a limit before the one-second command watchdog expires. Calibration
motion retains its separate endpoint and stall protections.

The hardware IWDG starts after boot-time peripheral, Cyphal, motor, and EEPROM
initialization. Its nominal timeout is 2 seconds. Firmware refreshes it only
after a main-loop pass has completed both the scheduled `motor_update` and
`cyphal_loop`; a stall in either path therefore resets the MCU. The watchdog is
frozen while the CPU is halted by a debugger. After an IWDG reset the normal
initialization leaves the joint in HOLD, and controller-originated movement
still requires a valid controller heartbeat and a new command. Reset causes are
session-only and available through `reset_reason`; they are not written to
EEPROM.
