cmake_minimum_required(VERSION 3.13)

# Set the Pico SDK path
set(PICO_SDK_PATH "/home/deepspace/pico/pico-sdk")

set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)
set(PICO_BOARD adafruit_metro_rp2350 CACHE STRING "Board type")

# Use RP2350 Cortex-M33 toolchain
set(CMAKE_TOOLCHAIN_FILE ${PICO_SDK_PATH}/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake)

# Include the Pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(gnss_datalogging C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable
add_executable(gnss_datalogging
    src/main.c
    src/zedf9p_platform_pico.c
    src/hw_config.c
    include/main.h
    src/diskio.c
    include/hw_config.h
)

# Add FatFs source files
target_sources(gnss_datalogging PRIVATE
    src/main.c
    src/diskio.c
    src/hw_config.c
    src/zedf9p_platform_pico.c
    src/fatfs_utils.c

    libraries/external/FatFs/src/ff15/source/ff.c
    libraries/external/FatFs/src/ff15/source/ffsystem.c
    libraries/external/FatFs/src/ff15/source/ffunicode.c

    libraries/external/FatFs/src/sd_driver/sd_card.c
    libraries/external/FatFs/src/sd_driver/dma_interrupts.c
    libraries/external/FatFs/src/sd_driver/SPI/my_spi.c
    libraries/external/FatFs/src/sd_driver/SPI/sd_card_spi.c
    libraries/external/FatFs/src/sd_driver/SPI/sd_spi.c
    libraries/external/FatFs/src/sd_driver/sd_timeouts.c

    libraries/external/FatFs/src/src/util.c
    libraries/external/FatFs/src/src/f_util.c
)

# Add included directories
target_include_directories(gnss_datalogging PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    libraries/external/FatFs/src/src
    libraries/external/FatFs/src/include
    libraries/external/FatFs/src/ff15/source
    libraries/external/FatFs/src/sd_driver
)

# Add subdirectories
add_subdirectory(libraries/local/zedf9p zedf9p_build)

# Link libraries
target_link_libraries(gnss_datalogging
    pico_stdlib
    hardware_i2c
    hardware_uart
    hardware_dma
    hardware_spi
    hardware_gpio
    hardware_clocks
    hardware_pio
    zedf9p
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(gnss_datalogging 1)
pico_enable_stdio_uart(gnss_datalogging 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(gnss_datalogging)

# At the end of your CMakeLists.txt
add_custom_target(flash
    COMMAND picotool load -x -f ${CMAKE_CURRENT_BINARY_DIR}/gnss_datalogging.uf2 && sleep 2 && picotool reboot -f
    DEPENDS gnss_datalogging
    COMMENT "Flashing to Pico"
)