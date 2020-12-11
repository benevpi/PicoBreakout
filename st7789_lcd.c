/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"
#include "pico/multicore.h"

#include "st7789_lcd.pio.h"

#include "ball.h"
#include "bat.h"
#include "brick.h"
#include "brick2.h"

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define IMAGE_SIZE 256
#define LOG_IMAGE_SIZE 8

#define PIN_DIN 0
#define PIN_CLK 1
#define PIN_CS 2
#define PIN_DC 3
#define PIN_RESET 4
#define PIN_BL 5

#define BALL_SIZE 20

#define SERIAL_CLK_DIV 1.f

#define BUTTON_RIGHT_GPIO 15
#define BUTTON_LEFT_GPIO 14

//each pixel contains a byte which is a lookup to the colours table
uint8_t pixels[SCREEN_WIDTH][SCREEN_HEIGHT]; 

//each block is 24 pixels by 10, and there are 10x5 of them that can be there

bool blocks[10][5];

//put usefule colours in here
uint16_t colours[255];

bool pause = false;

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
static const uint16_t st7789_init_seq[] = {
    1, 20,  0x01,                         // Software reset
    1, 10,  0x11,                         // Exit sleep mode
    2, 2,   0x3a, 0x55,                   // Set colour mode to 16 bit
    2, 0,   0x36, 0x00,                   // Set MADCTL: row then column, refresh is bottom to top ????
    5, 0,   0x2a, 0x00, 0x00, 0x00, 0xf0, // CASET: column addresses from 0 to 240 (f0)
    5, 0,   0x2b, 0x00, 0x00, 0x01, 0x40, // RASET: row addresses from 0 to 240 (f0)
    1, 2,   0x20,                         // Inversion on, then 10 ms delay (supposedly a hack?)
    1, 2,   0x13,                         // Normal display on, then 10 ms delay
    1, 2,   0x29,                         // Main screen turn on, then wait 500 ms
    0                                     // Terminate list
};

static inline void lcd_set_dc_cs(bool dc, bool cs) {
    sleep_us(1);
    gpio_put_mask((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
    sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint16_t *cmd, size_t count) {
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(0, 0);
    st7789_lcd_put(pio, sm, *cmd++);
    if (count >= 2) {
        st7789_lcd_wait_idle(pio, sm);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            st7789_lcd_put(pio, sm, *cmd++);
    }
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint16_t *init_seq)
{
    const uint16_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void st7789_start_pixels(PIO pio, uint sm)
{
    uint16_t cmd = 0x2c; // RAMWR
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
}

void draw_square(int x, int y, int width, int height, int colour) {
	for(int i = 0; i<width; i++) {
		for(int j = 0; j<height; j++) {
			pixels[x+i][y+j] = colour;
		}
	}
	
}

//abosolutely no idea of the colour model this is using.
//not going to worry about it too much. Wait and see how other displays work
void load_sprite_colours(uint16_t sprite_colours[][3], int size, int offset) {
	for(int i=0; i<size;i++) {
		colours[i+offset] = (sprite_colours[i][2]&0xf8)<<8 | (sprite_colours[i][1]&0xF8)<<3 | (sprite_colours[i][0]&0xf8)>>3;
	}
	
}

//bugger, need to convert the sprite to the correct format.
void draw_sprite(int x, int y, int width, int height, uint8_t sprite[], int transparent_index, int colour_offset) {
	
	for(int i=0; i<width; i++) {
		for(int j=0; j<height; j++) {
			if(sprite[(j*width)+i] != transparent_index) {
				pixels[x+i][y+j] = sprite[(j*width)+i]+colour_offset;
			}
			
		}
	}
	
	
}

void pixels_core() {
	//do our processing and drawing here
	colours[0] = 0; // black -- acutally, this is white. Something's going a bit fishy
	colours[1] = 0xffff; //white -- actually black. Not to sure what's going on.
	colours[2] = 0x7c; //green?
	colours[3] = 0x1000; // some shade of red perhaps?
	colours[4] = 0x0f00;
	
	int x = 0;
	int y = 200;
	int direction_x = 1;
	int direction_y = -1;
	
	int bat_x = 100;
	int bat_y = 300;
	
	//init all blocks
	for(int i = 0; i<10;i++) {
		for(int j=0;j<5;j++) {
			blocks[i][j] = true;
		}
	}
	
	while(1) {
		
		pause = true;
		sleep_us(10); // wait for buffers to flush?

		
		//blank the display
		for(int i = 0; i<240; i++) {
			for(int j = 0; j<320;j++) {
				
				pixels[i][j] = 1;
			}
		}
		
		//bounce a square 'ball'
		//let's draw a sprite
		//draw_square(x,y,BALL_SIZE,BALL_SIZE,2);
		draw_sprite(x,y,BALL_SIZE,BALL_SIZE, ball, 15, 10);
		
		//draw the blocks
		for(int i = 0; i<10;i++) {
			for(int j=0;j<5;j++) {
				if(blocks[i][j]) {
					if((i+j)%2) {
						draw_sprite((i*24),(j*10), 23, 9, brick, 15, 50);
					}
					else {
						draw_sprite((i*24),(j*10), 23, 9, brick2, 15, 50);
					}
					//draw_square((i*24), (j*10),23,9,3);
				}
			}
		}
		
		//move the bat
		if (!gpio_get(BUTTON_RIGHT_GPIO) && bat_x > 0) { bat_x--;}
		if (!gpio_get(BUTTON_LEFT_GPIO) && bat_x < 189) { bat_x++;}
		
		//draw the bat
		//draw_square(bat_x, bat_y, 70,10,4);
		draw_sprite(bat_x,bat_y,70,10, bat, 15, 30);
		
		//collision detection
		if(y<51) {
		//check against all the blocks
			for(int i = 0; i<10;i++) {
				for(int j=0;j<5;j++) {
					//note currently assume always bounce down not across
					//and from the mid-point x, but any y.
					
					if(x>(i*24) && x<((i+1)*24)){
						if(y-(BALL_SIZE/2)<((j+1)*10) && (y+(BALL_SIZE/2))>(j*10)) {
							if(blocks[i][j]) {
								blocks[i][j]  = false;
								direction_y = direction_y*-1;
							}
						}
					}
				}
			}
			
		}
		
		x=x+direction_x;
		y=y+direction_y;
		
		if((x>215 && direction_x > 0) || (x<5 && direction_x < 0)) { direction_x = -1*direction_x; }
		
		if(y<=0) { direction_y = 1; } // bounce off the roof
		
		//collision detection with the bat
		
		if( y==280) {
			int center_x = (x+10);
			if(center_x>bat_x && x<(center_x + 70)) { 
				int posn = 3 - (int)((center_x-bat_x) /10); // bounce depending where on bat hit
				direction_x = -1 * (int)(posn/2); // 
			
				direction_y = direction_y*-1;} // need to make this more based on where it hits on the bat.
		}
		
		
		if(y>310) { while(1) {}} //just crash if you loose
		
		pause = false;
		
		sleep_us(5000);
	}
}



int main() {
    //setup_default_uart();
	
	gpio_init(BUTTON_LEFT_GPIO);
    gpio_dir(BUTTON_LEFT_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_LEFT_GPIO);
	
	gpio_init(BUTTON_RIGHT_GPIO);
    gpio_dir(BUTTON_RIGHT_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_RIGHT_GPIO);
	
	multicore_launch_core1(pixels_core);

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &st7789_lcd_program);
    st7789_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

    gpio_init(PIN_CS);
    gpio_init(PIN_DC);
    gpio_init(PIN_RESET);
    gpio_init(PIN_BL);
    gpio_dir(PIN_CS, GPIO_OUT);
    gpio_dir(PIN_DC, GPIO_OUT);
    gpio_dir(PIN_RESET, GPIO_OUT);
    gpio_dir(PIN_BL, GPIO_OUT);

    gpio_put(PIN_CS, 1);
    gpio_put(PIN_RESET, 1);
    lcd_init(pio, sm, st7789_init_seq);
    gpio_put(PIN_BL, 1);

	
	pause=false;

    load_sprite_colours(ball_colours, 16, 10);
	load_sprite_colours(bat_colours, 16, 30);
	load_sprite_colours(brick_colours, 16, 50);
	load_sprite_colours(brick2_colours, 16, 70);
	
	//just yeet out the pixels as fast as possible
	//no doubt this could be handled better with DMA, but this'll do for now.
    while (1) {

        st7789_start_pixels(pio, sm);
        for (int y = 0; y < SCREEN_HEIGHT; ++y) {
            for (int x = 0; x < SCREEN_WIDTH; ++x) {
				while(pause) {sleep_us(1);}
			
                st7789_lcd_put(pio, sm, colours[pixels[x][y]] >> 8);
                st7789_lcd_put(pio, sm, colours[pixels[x][y]] & 0xff);
            }
        }
    }
}
