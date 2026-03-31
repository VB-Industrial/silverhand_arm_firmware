cmake_minimum_required(VERSION 3.22)

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

add_executable(${CMAKE_PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32cubemx/CMakeLists.txt")
    add_subdirectory(cmake/stm32cubemx)
endif()

set(USER_SOURCES
    Core/Src/alert_monitor.cpp
    Core/Src/communications.cpp
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
