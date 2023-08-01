cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

project(climate_control C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Avoid issues with old SDK version and check it explicitly
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

# Recommended options for Pico
add_compile_options(-Wall
    -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
)

if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()


# Standalone example that reads from the on board temperature sensor and sends notifications via BLE
# Flashes slowly each second to show it's running
add_executable(climate_control_sensor
    sensor.c
    bt_services/ess.c
    bme680/bme68x.c
)

target_compile_definitions(climate_control_sensor PRIVATE
    BME68X_DO_NOT_USE_FPU
)

target_link_libraries(climate_control_sensor
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_i2c
)

target_include_directories(climate_control_sensor PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
    ${CMAKE_CURRENT_LIST_DIR}/bt_services
    ${CMAKE_CURRENT_LIST_DIR}/bme680
)

pico_btstack_make_gatt_header(climate_control_sensor PRIVATE "${CMAKE_CURRENT_LIST_DIR}/sensor.gatt")

# create map/bin/hex file etc.
pico_add_extra_outputs(climate_control_sensor)

# enable usb output, disable uart output
pico_enable_stdio_usb(climate_control_sensor 1)
pico_enable_stdio_uart(climate_control_sensor 0)
