cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define optimized build types
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME datalogging_stm32cube)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Optimization flags for different build types
set(CMAKE_C_FLAGS_DEBUG "-Og -g -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-Os -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Os -g -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG")

# Additional optimization flags for all builds
add_compile_options(
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fstack-usage
        -Wall
        -Wextra
        -Wpedantic
)

# Link-time optimization flags
add_link_options(
        -Wl,--gc-sections,--print-memory-usage
        -Wl,--sort-section=alignment
        -Wl,--cref
        -Wl,-Map=${PROJECT_NAME}.map,--cref
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Inc/zedf9p.h
        Core/Inc/zedf9p_platform.h
        Core/Src/zedf9p.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Size optimization macros
        $<$<CONFIG:Release>:USE_HAL_DRIVER>
        $<$<CONFIG:Release>:STM32F405xx>
        # Disable unused HAL modules (add these based on what you don't use)
        # HAL_ADC_MODULE_DISABLED
        # HAL_CAN_MODULE_DISABLED
        # HAL_CRC_MODULE_DISABLED
        # HAL_DAC_MODULE_DISABLED
        # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        # Add user defined libraries
)

# Post-build commands to generate bin file and show size
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        COMMENT "Building ${CMAKE_PROJECT_NAME}.bin and showing memory usage"
)

# Fixed DFU upload target (use .bin file, not .elf)
add_custom_target(upload_dfu
        COMMAND dfu-util -a 0 -s 0x08000000:leave -D ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Uploading ${CMAKE_PROJECT_NAME}.bin via DFU"
)

# Additional utility targets
add_custom_target(size
        COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Showing detailed memory usage"
)

add_custom_target(hexdump
        COMMAND ${CMAKE_OBJDUMP} -h -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> > ${CMAKE_PROJECT_NAME}.lst
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Creating disassembly listing"
)