cmake_minimum_required(VERSION 3.22)

include(ExternalProject)

set(CMAKE_PROJECT_NAME silver_hand_firmware)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

include("cmake/gcc-arm-none-eabi.cmake")

enable_language(C CXX ASM)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

option(SR_ARM_ENABLE_CYPHAL "Enable Cyphal/CANFD transport" ON)
option(SR_ARM_ENABLE_MOTOR_CONTROL "Enable motor-control application" ON)
option(SR_BUILD_BOOTLOADER "Build the VBBoot CAN bootloader and combined factory image" ON)

add_executable(${CMAKE_PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32cubemx/CMakeLists.txt")
    add_subdirectory(cmake/stm32cubemx)
endif()

set(USER_SOURCES
    Core/Src/bootloader_request.c
    Core/Src/fault_manager.cpp
    Core/Src/fault_log.c
    Core/Src/encoder_calibration.cpp
    Core/Src/encoder_calibration_storage.c
    Core/Src/communications.cpp
    Core/Src/motor.cpp
    Core/Src/system_watchdog.c
    Core/Src/tmc5160_state.cpp
    Core/Src/as50xx.c
    Core/Src/at24_eeprom.c
    Core/Src/tmc5160.c
    Core/Src/utility.c
)

file(GLOB_RECURSE LIBCXXCANARD_SOURCES
    "Drivers/libcxxcanard/cyphal/*.cpp"
    "Drivers/libcxxcanard/libs/*.c"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    ${LIBCXXCANARD_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME}
    PRIVATE
        App
        Core/Inc
        config
        Drivers/libcxxcanard
        Drivers/libcxxcanard/libs
)

target_compile_definitions(${CMAKE_PROJECT_NAME}
    PRIVATE
        $<$<BOOL:${SR_ARM_ENABLE_CYPHAL}>:SR_ARM_ENABLE_CYPHAL>
        $<$<BOOL:${SR_ARM_ENABLE_MOTOR_CONTROL}>:SR_ARM_ENABLE_MOTOR_CONTROL>
        STM32G4
        STM32G
)

target_compile_options(${CMAKE_PROJECT_NAME}
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

if(TARGET stm32cubemx)
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
        stm32cubemx
    )
endif()

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Building ${CMAKE_PROJECT_NAME}.bin and ${CMAKE_PROJECT_NAME}.hex"
)

if(SR_BUILD_BOOTLOADER)
    set(VBBOOT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/VBBoot)
    set(VBBOOT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/VBBoot)
    set(VBBOOT_BIN ${VBBOOT_BINARY_DIR}/VBBoot.bin)
    set(VBBOOT_PADDED_BIN ${CMAKE_CURRENT_BINARY_DIR}/VBBoot_padded.bin)
    set(APP_BIN ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)
    set(FULL_BIN ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_full.bin)

    if(NOT EXISTS ${VBBOOT_SOURCE_DIR}/CMakeLists.txt)
        message(FATAL_ERROR "VBBoot submodule is missing; run git submodule update --init")
    endif()

    ExternalProject_Add(VBBoot_firmware
        SOURCE_DIR ${VBBOOT_SOURCE_DIR}
        BINARY_DIR ${VBBOOT_BINARY_DIR}
        CMAKE_GENERATOR "${CMAKE_GENERATOR}"
        CMAKE_ARGS
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_TOOLCHAIN_FILE=${VBBOOT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake
            -DBOOTLOADER_MCU=STM32G474xx
            -DBOOTLOADER_STARTUP_FILE=${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32g474xx.s
            -DBOOTLOADER_APP_END=0x08080000UL
            -DBOOTLOADER_RAM_END=0x20020000UL
            -DBOOTLOADER_FDCAN_CLOCK_SOURCE=PLL
            -DBOOTLOADER_FDCAN_ID_FORMAT=extended
            -DBOOTLOADER_USE_EEPROM_CONFIG=OFF
        INSTALL_COMMAND ""
        BUILD_ALWAYS TRUE
        BUILD_BYPRODUCTS ${VBBOOT_BIN}
    )

    add_custom_command(
        OUTPUT ${FULL_BIN}
        COMMAND ${CMAKE_OBJCOPY} -I binary -O binary --gap-fill 0xFF --pad-to 0x3000 ${VBBOOT_BIN} ${VBBOOT_PADDED_BIN}
        COMMAND ${CMAKE_COMMAND} -E cat ${VBBOOT_PADDED_BIN} ${APP_BIN} > ${FULL_BIN}
        DEPENDS ${CMAKE_PROJECT_NAME} VBBoot_firmware ${APP_BIN} ${VBBOOT_BIN}
        COMMENT "Building bootloader + application factory image"
        VERBATIM
    )
    add_custom_target(${CMAKE_PROJECT_NAME}_full_bin ALL DEPENDS ${FULL_BIN})
endif()
