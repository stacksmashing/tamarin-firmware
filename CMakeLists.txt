cmake_minimum_required(VERSION 3.13)
# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
project(tamarin_firmware)
 
pico_sdk_init()

add_executable(tamarin_firmware)

# target_compile_definitions(tamarin_firmware
#   PUBLIC PICO_UART_ENABLE_CRLF_SUPPORT=0
#   PUBLIC PICO_STDIO_ENABLE_CRLF_SUPPORT=0
#   PUBLIC PICO_STDIO_DEFAULT_CRLF=0
# )

target_compile_options(tamarin_firmware INTERFACE -DPICO_ENTER_USB_BOOT_ON_EXIT=1)


pico_generate_pio_header(tamarin_firmware ${CMAKE_CURRENT_LIST_DIR}/lightning_tx.pio)
pico_generate_pio_header(tamarin_firmware ${CMAKE_CURRENT_LIST_DIR}/lightning_rx.pio)
pico_generate_pio_header(tamarin_firmware ${CMAKE_CURRENT_LIST_DIR}/probe.pio)

# Required so the build finds tusb_config.h
target_include_directories(tamarin_firmware PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_sources(tamarin_firmware PRIVATE
        tamarin_probe.c
        main.c
        usb_descriptors.c
        util.c
        )

target_link_libraries(tamarin_firmware PRIVATE pico_stdlib tinyusb_device tinyusb_board pico_stdio hardware_i2c pico_multicore pico_time pico_sync hardware_pio)
pico_add_extra_outputs(tamarin_firmware)

# enable usb output, disable uart output
#pico_enable_stdio_usb(tamarin_firmware 1)
#pico_enable_stdio_uart(tamarin_firmware 0)
