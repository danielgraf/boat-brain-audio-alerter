cmake_minimum_required(VERSION 3.13)

# Board and platform must be set before include(pico_sdk_import.cmake)
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform")

# Pull in Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

project(boat_brain_audio_alerter
    VERSION 1.0.0
    DESCRIPTION "Boat Brain Audio Alerter - I2S audio warning system for Pico 2 + MAX98357A"
    LANGUAGES C CXX ASM
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(boat_brain_audio_alerter
    src/main.cpp
    src/audio/i2s_output.cpp
    src/audio/tone_generator.cpp
    src/audio/wav_player.cpp
    src/alerts/alert_manager.cpp
)

# Generate the C header from the PIO assembly
pico_generate_pio_header(boat_brain_audio_alerter
    ${CMAKE_CURRENT_LIST_DIR}/src/audio/i2s_output.pio
)

target_include_directories(boat_brain_audio_alerter PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(boat_brain_audio_alerter
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_gpio
    hardware_clocks
    pico_multicore
)

# Enable USB serial output, disable UART
pico_enable_stdio_usb(boat_brain_audio_alerter 1)
pico_enable_stdio_uart(boat_brain_audio_alerter 0)

# Generate .uf2, .bin, .hex, .map output files
pico_add_extra_outputs(boat_brain_audio_alerter)
