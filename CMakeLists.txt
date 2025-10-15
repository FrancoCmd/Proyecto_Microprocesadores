# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Indoor C CXX ASM)


# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Indoor Indoor.c )

pico_set_program_name(Indoor "Indoor")
pico_set_program_version(Indoor "0.1")

# Generate PIO header
pico_generate_pio_header(Indoor ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Indoor 0)
pico_enable_stdio_usb(Indoor 1)

# Add the standard library to the build
target_link_libraries(Indoor
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Indoor PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

#include onewire
add_subdirectory(onewire_library)

# Add any user requested libraries
target_link_libraries(Indoor 
        hardware_spi
        hardware_i2c
        hardware_pio
        hardware_adc
        onewire_library
        )

pico_add_extra_outputs(Indoor)

#example_auto_set_url(Indoor)

