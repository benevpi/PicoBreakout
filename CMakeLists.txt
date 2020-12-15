cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(picogames)

pico_sdk_init()


add_executable(pio_st7789_lcd)

pico_generate_pio_header(pio_st7789_lcd ${CMAKE_CURRENT_LIST_DIR}/st7789_lcd.pio)

target_sources(pio_st7789_lcd PRIVATE st7789_lcd.c)

target_include_directories(pio_st7789_lcd PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(pio_st7789_lcd PRIVATE pico_stdlib hardware_pio hardware_interp pico_multicore tinyusb_host tinyusb_board)
pico_add_extra_outputs(pio_st7789_lcd)
