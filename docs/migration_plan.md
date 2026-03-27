# Silver Hand Firmware Migration Plan

## Phase 1. Freeze The Baseline

Goal:
- define what `Ruka` provides today and what we intentionally discard

Exit criteria:
- baseline map is documented
- removal list is agreed
- target project name and root layout are fixed

Status:
- in progress

## Phase 2. Rebuild The Project Skeleton

Goal:
- make `silver_hand_firmware` buildable in VSCode with CMake and Ninja

Inputs:
- build approach from `/home/robot/projects/bldc_joint`
- current minimal skeleton in this repository

Work items:
- import top-level `CMakeLists.txt` approach
- add `CMakePresets.json`
- add `.vscode` launch/tasks/settings
- add `cmake/stm32cubemx/CMakeLists.txt`
- align executable name, linker script, startup file, include paths, and defines

Exit criteria:
- project configures and builds outside STM32CubeIDE

## Phase 3. Refresh STM32 Generated Code

Goal:
- update HAL/CMSIS and generated platform files using the current Cube package

Work items:
- open `Ruka.ioc`
- upgrade MCU package if prompted
- regenerate source and headers
- compare regenerated platform files against the baseline
- move regenerated files into `silver_hand_firmware`

Expected impact:
- the build may break temporarily
- generated file names, initialization details, and include dependencies may shift

Exit criteria:
- refreshed generated STM32 layer is imported into the target project

## Phase 4. Recover The Build

Goal:
- restore a stable embedded build after Cube regeneration

Work items:
- fix startup and linker wiring
- fix include paths and compile definitions
- resolve missing source files and changed generated dependencies
- ensure `main.c` and user entrypoint composition still work

Exit criteria:
- clean configure/build in VSCode/CMake

## Phase 5. Port Functional Logic From `Ruka`

Goal:
- preserve current manipulator joint behavior in the new project structure

Work items:
- move `joint_config`
- move `tmc5160`
- move `as50xx`
- move utility helpers
- replace monolithic `main_impl.cpp` with modular application/control files
- remove `jsm` entirely

Exit criteria:
- the joint controller logic builds and behaves like the old firmware baseline

## Phase 6. Migrate Communication To Current `libcxxcanard`

Goal:
- replace legacy embedded `libcyphal` copy with current `libcxxcanard`

Source of truth:
- `https://github.com/VBCores/libcxxcanard`

Work items:
- vendor or attach the current library
- build it under the new CMake layout
- port Cyphal interface initialization
- port heartbeat
- port node info
- port register list/access
- port command subscription
- port telemetry publication

Exit criteria:
- communication uses current `libcxxcanard`
- no dependency on `Drivers/libcyphal` remains

## Phase 7. Unify And Extend Cyphal Registers

Goal:
- preserve existing arm registers and selectively import proven useful ones from
  `bldc_joint`

Work items:
- inventory current register names and types
- remove obsolete/test-only registers
- merge useful control/configuration registers from `bldc_joint`
- document the resulting register contract

Exit criteria:
- a clean, maintained register set exists for the hand firmware

## Phase 8. Add Optional Encoder Support

Goal:
- support both hardware variants with and without an encoder

Configuration requirement:
- add a compile-time define in `robot_config` or equivalent project config

Modes:
- encoder absent: current step-based behavior
- encoder present: enable estimator/fusion path

Exit criteria:
- both configurations build from one codebase

## Phase 9. Implement Encoder + Steps Fusion

Goal:
- improve joint state estimation when encoder hardware is present

Initial scope:
- keep the first estimator simple and debuggable
- use steps as motion prediction
- use encoder as correction/validation
- detect excessive mismatch

Explicit non-goal:
- do not tie this logic to software joint limits

Exit criteria:
- estimator works in encoder-enabled builds
- fallback to steps-only mode remains intact

## Phase 10. Cleanup And Productization

Goal:
- make the firmware maintainable beyond the Silver Rover project

Work items:
- remove stale files and dead code
- normalize naming and module boundaries
- improve comments where code is non-obvious
- document the build, flash, debug, and configuration workflow
- leave the project in a reusable standalone state

Exit criteria:
- firmware is understandable, buildable, and ready for continued evolution

## Working Order

Recommended execution order:

1. baseline freeze
2. build skeleton migration
3. Cube regeneration
4. build recovery
5. functional logic port
6. `libcxxcanard` migration
7. register cleanup and extension
8. optional encoder support
9. fusion
10. final cleanup

