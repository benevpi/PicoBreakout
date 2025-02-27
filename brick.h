/*  GIMP header image file format (INDEXED): C:\Users\ben\Desktop\PicoBreakout\brick.h  */

static unsigned int width = 23;
static unsigned int height = 9;

/*  Call this macro repeatedly.  After each use, the pixel data can be extracted  */

#define HEADER_PIXEL(data,pixel) {\
pixel[0] = header_data_cmap[(unsigned char)data[0]][0]; \
pixel[1] = header_data_cmap[(unsigned char)data[0]][1]; \
pixel[2] = header_data_cmap[(unsigned char)data[0]][2]; \
data ++; }

static uint16_t  brick_colours[16][3] = {
	{  4,  7,  2},
	{  5,  8,  4},
	{ 15,  9,  7},
	{ 16, 10,  8},
	{ 54, 30,  9},
	{ 90, 50,  5},
	{ 92, 52, 14},
	{133, 75, 15},
	{134, 76, 16},
	{154, 87, 23},
	{196,110, 30},
	{208,120, 32},
	{225,128, 35},
	{229,131, 28},
	{255,147, 41},
	{255,255,255},
	
	};
static uint8_t brick[] = {
	14,14,14,14,14,0,12,14,14,14,14,9,9,14,14,14,
	12,0,14,14,14,14,14,
	14,14,14,14,14,2,12,14,14,14,14,5,5,14,14,14,
	12,2,14,14,14,14,14,
	11,12,14,14,14,0,12,14,14,14,14,9,9,14,14,14,
	14,0,14,14,14,14,14,
	2,0,0,0,9,5,7,12,9,0,0,2,0,9,9,5,
	9,2,7,9,9,0,0,
	14,14,14,12,9,6,0,0,9,12,12,7,7,9,9,5,
	9,0,7,9,9,12,14,
	14,14,14,14,14,13,14,14,14,14,14,9,9,14,14,14,
	14,9,9,14,14,14,14,
	3,3,0,3,3,3,3,3,3,0,3,3,3,0,3,3,
	6,4,4,9,3,3,3,
	14,14,14,12,12,10,0,12,14,14,14,9,9,14,14,12,
	6,8,3,9,14,14,14,
	14,14,14,14,14,14,0,14,14,14,14,9,9,14,14,14,
	14,14,0,12,14,14,14
	};
