cmake_minimum_required(VERSION 3.22)
include(ExternalProject)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

set(CMAKE_TOOLCHAIN_FILE gcc-arm-none-eabi.cmake)

# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Set the project name
set(CMAKE_PROJECT_NAME 05_usb_hub_init)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Include external project file
include("mx-generated.cmake")