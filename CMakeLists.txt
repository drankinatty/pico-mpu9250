# Set minimum required version of CMake
cmake_minimum_required (VERSION 3.12)

#include build functions from Pico SDK
include ($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ Standards
project (i2c-mpu9250 C CXX ASM)
set (CMAKE_C_STANDARD 11)
set (CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# set preprocessor defines to adjust MPU values displayed
# (any, all or none may be specified)
#
#   MPU9250 - define to activate magnetometer (none on MPU6050)
#   DENOISE - define to apply denoise limits to prevent bounce around zero
#   SETBIAS - apply chip-specific axis bias to zero axis
#   MPUCHIP - =No. allow multiple sets of axis biases for different boards
#   PICOTOOL - make i2c pins available to picotool
#
add_compile_definitions (PRIVATE
  MPU9250
  DENOISE
  SETBIAS
  # MPUCHIP=1    (see pico_mpu.c examples 1,2 are for my 2 boards)
)

# create executable with project name
add_executable (${PROJECT_NAME})

# sources to include
target_sources (${PROJECT_NAME} PRIVATE
  ${PROJECT_NAME}.c
  pico_mpu.c
)

# pull in common dependencies and additional spi hardware support
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_i2c)

# enable usb output, disable uart output (enable if using 1 pico over USB)
# pico_enable_stdio_usb (${PROJECT_NAME} 1)
# pico_enable_stdio_uart (${PROJECT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})
