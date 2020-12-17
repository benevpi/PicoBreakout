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
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/sync.h"
#include "hardware/clocks.h"
#include "hardware/structs/vreg_and_chip_reset.h"

#include "ball.h"
#include "bat.h"
#include "brick.h"
#include "brick2.h"

//note rotate the screen 90 degrees because then it'll fit in VGA res nicely
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define IMAGE_SIZE 256
#define LOG_IMAGE_SIZE 8


#define BALL_SIZE 20


//moved to GPIOs not used by VGA
//Note, there's cross-talk if this is too close to a high-speed signal.
#define BUTTON_RIGHT_GPIO 19
#define BUTTON_LEFT_GPIO 20

#define TURBO_BOOST
#define VREG_VSEL VREG_VOLTAGE_1_30
#include "hardware/vreg.h"


#define VGA_MODE vga_mode_320x240_60

//each pixel contains a byte which is a lookup to the colours table
uint8_t pixels[SCREEN_WIDTH][SCREEN_HEIGHT]; 
//put usefule colours in here
uint16_t colours[255];

uint16_t this_line[320];


int x = 0;
int y = 200;
int direction_x = 1;
int direction_y = -1;

int bat_x = 100;
int bat_y = 300;

extern const struct scanvideo_pio_program video_24mhz_composable;

// to make sure only one core updates the state when the frame number changes
// todo note we should actually make sure here that the other core isn't still rendering (i.e. all must arrive before either can proceed - a la barrier)
//don't think I need this, but fish it out later
static struct mutex frame_logic_mutex;

static void frame_update_logic();
static void render_scanline(struct scanvideo_scanline_buffer *dest, int core);
void fill_scanline_buffer(struct scanvideo_scanline_buffer *buffer);
void update_scene();
void init_scene();


//Bits from sprite_demo.c
// "Worker thread" for each core

void __time_critical_func(render_loop)() {
    static uint32_t last_frame_num = 0;
    int core_num = get_core_num();
    printf("Rendering on core %d\n", core_num);
    while (true) {
        struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);
        //mutex_enter_blocking(&frame_logic_mutex);
        uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);
        // Note that with multiple cores we may have got here not for the first
        // scanline, however one of the cores will do this logic first before either
        // does the actual generation
		
		
        if (frame_num != last_frame_num) {
            last_frame_num = frame_num;
            update_scene();
        } 
		
        //mutex_exit(&frame_logic_mutex);

        render_scanline(scanline_buffer, core_num);

        // Release the rendered buffer into the wild
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}


struct semaphore video_setup_complete;

void core1_func() {
    sem_acquire_blocking(&video_setup_complete);
    render_loop();
}


void vga_main() {
    mutex_init(&frame_logic_mutex);
    sem_init(&video_setup_complete, 0, 1);

    // Core 1 will wait for us to finish video setup, and then start rendering

    multicore_launch_core1(core1_func);


    hard_assert(VGA_MODE.width + 4 <= PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS * 2);
    scanvideo_setup(&VGA_MODE);
	
    scanvideo_timing_enable(true);

    sem_release(&video_setup_complete);
    render_loop(); // don't actually want core 0 to render. Let's leave all that to core 1

}

// Helper functions to:
// - Get a scanbuf into a state where a region of it can be directly rendered to,
//   and return a pointer to this region
// - After rendering, manipulate this scanbuffer into a form where PIO can
//   yeet it out on VGA

static inline uint16_t *raw_scanline_prepare(struct scanvideo_scanline_buffer *dest, uint width) {
    assert(width >= 3);
    assert(width % 2 == 0);
    // +1 for the black pixel at the end, -3 because the program outputs n+3 pixels.
    dest->data[0] = COMPOSABLE_RAW_RUN | (width + 1 - 3 << 16);
    // After user pixels, 1 black pixel then discard remaining FIFO data
    dest->data[width / 2 + 2] = 0x0000u | (COMPOSABLE_EOL_ALIGN << 16);
    dest->data_used = width / 2 + 2;
    assert(dest->data_used <= dest->data_max);
    return (uint16_t *) &dest->data[1];
}

static inline void raw_scanline_finish(struct scanvideo_scanline_buffer *dest) {
    // Need to pivot the first pixel with the count so that PIO can keep up
    // with its 1 pixel per 2 clocks
    uint32_t first = dest->data[0];
    uint32_t second = dest->data[1];
    dest->data[0] = (first & 0x0000ffffu) | ((second & 0x0000ffffu) << 16);
    dest->data[1] = (second & 0xffff0000u) | ((first & 0xffff0000u) >> 16);
    dest->status = SCANLINE_OK;
}


void __time_critical_func(render_scanline)(struct scanvideo_scanline_buffer *dest, int core) {
    int l = scanvideo_scanline_number(dest->scanline_id);
    uint16_t *colour_buf = raw_scanline_prepare(dest, (VGA_MODE.width));

	for(int i=0; i<320; i++) {
		//note the pixels array was originally for a portrait array, so need to rotate it 90 degrees for a VGA display		
		colour_buf[i] =  ((colours[pixels[l][i]]));

	}
		
    raw_scanline_finish(dest);
}




//end bits from sprite_demo


bool blocks[10][5];



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

void init_scene() {
		//do our processing and drawing here
	colours[0] = 0; // black -- acutally, this is white. Something's going a bit fishy
	colours[1] = 0xffff; //white -- actually black. Not to sure what's going on.
	colours[2] = 0x7c; //green?
	colours[3] = 0x1000; // some shade of red perhaps?
	colours[4] = 0x0f00;
	

	
	//init all blocks
	for(int i = 0; i<10;i++) {
		for(int j=0;j<5;j++) {
			blocks[i][j] = true;
		}
	}
}
	

void update_scene() {
		
		
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
		//horrific and doesn't really work. NEED TO FIX
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
		//need to speed things up on th
		y=y+(2*direction_y);
		
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
		
		
		//sleep seems to be doing weird things. Does this help?
		//not the sleep, it's the calculations.
		
		for(int i=0;i<5;i++) {
			sleep_us(10); //need to do something here to make this pause work properly.
		}
		

}



int main() {
	//vreg_set_voltage(VREG_VSEL);
    //setup_default_uart();
	//crank the speed. The VGA output may need exactly this speed?
	set_sys_clock(1536 * MHZ, 4, 2);

//no idea what this does -- taken from sprite_demo	
#ifdef PICO_SMPS_MODE_PIN
    gpio_init(PICO_SMPS_MODE_PIN);
    gpio_dir(PICO_SMPS_MODE_PIN, GPIO_OUT);
    gpio_put(PICO_SMPS_MODE_PIN, 1);
#endif

	
	gpio_init(BUTTON_LEFT_GPIO);
    gpio_dir(BUTTON_LEFT_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_LEFT_GPIO);
	
	gpio_init(BUTTON_RIGHT_GPIO);
    gpio_dir(BUTTON_RIGHT_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_RIGHT_GPIO);


    load_sprite_colours(ball_colours, 16, 10);
	load_sprite_colours(bat_colours, 16, 30);
	load_sprite_colours(brick_colours, 16, 50);
	load_sprite_colours(brick2_colours, 16, 70);
	
	init_scene();
	

	vga_main(); // note -- never returns from this.
	
	return 0;
}
