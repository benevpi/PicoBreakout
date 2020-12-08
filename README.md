# PicoBreakout
A very simple implementation of breakout on Raspberry Pi Pico. This uses an 240x320 ST7789 SPI screen. It's possible the screen I tested this with may have colours and directions inverted. Not too sure, and the other test screen is still in transit from China. Connections are in the 

There is a bit of an odd dual core thing in there. I initially intended to offload some graphics processing to one core and manage the display on the other, but it's not worked out that way, and it's effectively running single-threaded just in two threads at the moment.

## TODO
 - draw better graphics
 - get some system of scoring
 - maybe do some levels?
 - colours perhaps?