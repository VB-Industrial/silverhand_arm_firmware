# Ruka Baseline Map

This document captures the current `Ruka` project as the behavioral baseline for
`silver_hand_firmware`.

Source location:
- `/mnt/c/Users/VR/STM32CubeIDE/workspace_1.8.0/Ruka`

## Baseline Goals

- preserve existing manipulator-joint behavior before major refactors
- migrate away from STM32CubeIDE-centered build flow
- replace the embedded `libcyphal` copy with current `libcxxcanard`
- prepare the firmware for long-term reuse outside of Silver Rover

## Keep As Baseline

These files/modules are the first candidates to preserve semantically:

- `Core/Inc/joint_config.h`
- `Core/Src/joint_config.c`
- `Core/Inc/tmc5160.h`
- `Core/Src/tmc5160.c`
- `Core/Inc/as50xx.h`
- `Core/Src/as50xx.c`
- `Core/Inc/utility.h`
- `Core/Src/utility.c`
- `Core/Src/main_impl.cpp`
- `Core/Inc/mainimpl.h`
- `Core/Inc/ruka_joints.h`

## Cube-Generated Platform Layer

These files should be treated as generated STM32 platform code and refreshed via
CubeIDE/CubeMX when updating the STM32 package:

- `Core/Src/main.c`
- `Core/Src/dma.c`
- `Core/Src/fdcan.c`
- `Core/Src/gpio.c`
- `Core/Src/i2c.c`
- `Core/Src/spi.c`
- `Core/Src/stm32g4xx_hal_msp.c`
- `Core/Src/stm32g4xx_it.c`
- `Core/Src/system_stm32g4xx.c`
- `Core/Src/tim.c`
- `Core/Src/usart.c`
- `Core/Inc/*.h` matching the generated peripherals
- `Core/Startup/startup_stm32g474retx.s`
- `Drivers/CMSIS/*`
- `Drivers/STM32G4xx_HAL_Driver/*`
- `Ruka.ioc`
- `STM32G474RETX_FLASH.ld`
- `STM32G474RETX_RAM.ld`

## Remove Or Replace

These pieces should not survive as-is in the target project:

- `Core/Inc/jsm.h`
- `Core/Src/jsm.cpp`
- `Core/Src/main_impl.cpp.bak`
- `Drivers/libcyphal/*`

Rationale:
- `jsm` was an abandoned FSM experiment
- `.bak` should not remain in a maintainable product repo
- `libcyphal` is being replaced with current `libcxxcanard`

## Refactor Hotspots

### `main_impl.cpp`

This file currently mixes:
- Cyphal setup
- subscriptions and register services
- node info handling
- command interpretation
- actuator control behavior

It should be split into at least:
- `App/app.*`
- `App/communication/*`
- `App/control/*`

### Position Estimation

Current baseline behavior appears to derive joint position mainly from TMC5160
step accounting.

Target behavior:
- no encoder: preserve current step-based mode
- encoder present: enable fusion of encoder measurements and step-based estimate

### Joint Limits

Software joint limits should be removed from low-level control logic except where
they are needed for hardware-safety reasons.

## Reusable Ideas To Pull From `bldc_joint`

Use as reference from:
- `/home/robot/projects/bldc_joint`

Main areas of reuse:
- CMake + Ninja + VSCode build flow
- `cmake/stm32cubemx` layout
- Cyphal register patterns already proven useful
- general project organization for a standalone firmware product

## Current External Dependencies

- STM32 HAL/CMSIS from Cube package
- `libcxxcanard` target source:
  `https://github.com/VBCores/libcxxcanard`
- TMC5160 driver and related support code from `Ruka`

