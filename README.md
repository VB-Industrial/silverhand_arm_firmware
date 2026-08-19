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

The node exposes a compact public interface of twelve Cyphal registers:

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
  report startup recovery state and its target angle. Calibration state and progress remain at indices 17
  and 18. `control_mode` is `0` hold, `1` servo, `2` direct, `3` calibration,
  or `4` raw TMC position. Raw integer positions and counters are encoded as `real32`
  because Cyphal register arrays cannot mix element types.

On startup, a calibrated joint that is already inside its operational hard
limits remains exactly where it is. If its absolute encoder position is still
inside the calibrated table but outside an operational hard limit, the joint
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
  immediately enters fused-angle SERVO settling. This one-shot service target does not require a
  controller or network heartbeat and remains held until another command,
  DIRECT mode, a fault, calibration, or reset cancels it. Its correction speed
  is saturated by the per-joint SERVO velocity limit. The TMC5160 position
  ramp uses `1 rad/s^2`; after the driver reports `position_reached`, the
  fused-angle P loop removes the remaining output-position error.
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
stream is alive and feed-forward is above `0.02 rad/s`, firmware passes that
speed to the TMC5160 without adding output-position-error correction. Once
feed-forward reaches `0.02 rad/s`, firmware holds a fixed `0.02 rad/s` finishing
speed. Direction follows the position error, and the TMC5160 remains in position
mode toward the absolute target. Thus a lagging joint continues approaching the
final target after feed-forward reaches zero without modulating velocity from
the quantized output-angle error. The
acceleration field selects the TMC5160 ramp acceleration and deceleration. Zero
requests the maximum default `A1/AMAX/DMAX/D1` profile; a finite positive value
is converted from joint rad/s^2 to motor microsteps/s^2. Negative and non-finite
acceleration commands are rejected. DIRECT velocity commands use their separate
`1131`-`1136` subjects.

Repeated commands containing the same final position remain in feed-forward
tracking and update its velocity. After the command stream becomes silent, a
proportional settling controller (`Kp = 4 s^-1`) commands joint velocity against
the fused output angle. Per-joint SERVO and DIRECT
velocity caps are configured by `maximum_servo_velocity_rad_s` and
`maximum_direct_velocity_rad_s` in `robot_config.h`; their current defaults are
`0.1 rad/s` and the separately tested `0.12 rad/s`.
It stops inside four output-encoder ticks and resumes correction only after the
output moves farther than eight ticks. This hysteresis prevents stationary
encoder noise from making the motor hunt around a TMC position that has already
been reached. The position error is not angle-wrapped because the public joint
coordinate intentionally covers `-2*pi` through `+2*pi`.

If the position-command stream becomes silent for 0.5 second, the latest target
is retained and the same fused-angle settling loop takes over. It remains a
closed-loop position HOLD while heartbeat from `controller_node_id` is alive.
Loss of that controller heartbeat, a fault, a DIRECT command, or entry into
calibration cancels the target and commands stop/HOLD. A missing SERVO update
therefore does not raise the DIRECT velocity-command-timeout fault.

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
direction- and magnitude-consistent for six frames (60 ms) before it is accepted; an
encoder jump exceeding the physical-rate gate is rejected. With stationary TMC,
external motion is recognized after the encoder span within a rolling 500-ms
stationary window exceeds 24 ticks for six frames (60 ms), so the observed idle span of about 20 ticks does
not masquerade as hand movement while a backlash rocking span does. Once in
`MANUAL`, the angle follows only the
encoder through a 100-ms first-order filter; the entry threshold no longer
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
DIRECT velocity is linearly reduced from its `0.12 rad/s` cap at the soft
boundary to zero at the hard limit. SERVO uses the same envelope with its own
`0.1 rad/s` cap. Inward velocity remains available at full speed. If the measured
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
