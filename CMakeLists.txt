# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Uncomment if you must specify the board here
set(BOARD nucleo_f446re)
# set(BOARD stm32f373rbt6)

set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/usq_zrtos.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(usq_zrtos)

# Explicitly list source files
set(app_sources
    src/button.c
    src/dac.c
    src/uart.c
    src/led.c
    src/main.c
    src/sequencer.c
    src/pot.c
    src/usb.c

    # Add other source files here
)

target_sources(app PRIVATE ${app_sources})

# Main Include Directory
target_include_directories(app PRIVATE include)



