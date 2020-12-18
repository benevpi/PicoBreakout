cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(picogames)

pico_sdk_init()

#Add lib?
add_subdirectory(sprite)


add_executable(pio_st7789_lcd)

target_compile_definitions(pio_st7789_lcd PRIVATE
            PICO_SVIDEO_MAX_SCANLINE_BUFFER_WORDS=500
            )

target_sources(pio_st7789_lcd PRIVATE st7789_lcd.c)

target_link_libraries(pio_st7789_lcd PRIVATE pico_stdlib pico_multicore pico_scanvideo_dpi
            sprite)
pico_add_extra_outputs(pio_st7789_lcd)
