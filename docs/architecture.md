# Arm Firmware Architecture

## Purpose

This firmware package is the starting point for the manipulator actuator controller.
It is intentionally independent from rover firmware and from the ROS 2 stack.

## Initial architecture

The firmware should be split into four layers:

1. `platform`
   STM32 HAL startup, clocks, timers, SPI, ADC, FDCAN, watchdog, interrupts.

2. `transport`
   Cyphal node lifecycle, heartbeat, registers, command subscriptions, telemetry publication.

3. `control`
   Encoder acquisition, calibration, state estimation, motor-control loop, fault handling.

4. `application`
   Joint state machine, mode arbitration, parameter binding, safety policy.

## Initial implementation strategy

Use `../bldc_codex` as the technical reference for:
- STM32G4 startup and build setup
- FDCAN provider
- Cyphal node handling
- baseline motor-control composition

Do not clone its structure blindly into this project.
Instead, import only the pieces that survive a clearer module boundary.

## Current assumptions

- One firmware instance controls one manipulator joint.
- All six joints should share one source tree.
- Per-joint differences should be expressed through configuration, not forks.
- Cyphal topic and register layout will be cleaned up before ROS 2 integration starts.

## Known open questions

- Exact STM32 part number and board revision.
- Motor driver stage and current-sense topology.
- Encoder set: motor-side only or motor-side plus output-side.
- Required control modes: torque, velocity, position, position+velocity.
- Homing and absolute zero strategy.
- Safety behavior on CAN timeout and heartbeat loss.
