/*  GIMP header image file format (INDEXED): C:\Users\ben\Desktop\PicoBreakout\ball-output.h  */

static unsigned int width = 20;
static unsigned int height = 20;

/*  Call this macro repeatedly.  After each use, the pixel data can be extracted  */

#define HEADER_PIXEL(data,pixel) {\
pixel[0] = header_data_cmap[(unsigned char)data[0]][0]; \
pixel[1] = header_data_cmap[(unsigned char)data[0]][1]; \
pixel[2] = header_data_cmap[(unsigned char)data[0]][2]; \
data ++; }

static unsigned char header_data_cmap[256][3] = {
	{ 32, 63, 38},
	{ 30, 95, 45},
	{ 24,134, 54},
	{ 11,193, 71},
	{  6,235, 79},
	{ 28,229, 85},
	{ 59,237,106},
	{106,238,128},
	{133,240,149},
	{160,243,168},
	{178,245,188},
	{192,246,198},
	{208,248,212},
	{226,251,228},
	{236,252,239},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255},
	{255,255,255}
	};
static unsigned char header_data[] = {
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,
	15,15,15,15,15,15,15,14,13,13,13,13,14,15,15,15,
	15,15,15,15,
	15,15,15,15,15,13,12,11,9,9,9,9,10,12,14,15,
	15,15,15,15,
	15,15,15,14,13,10,8,5,5,4,5,5,5,8,10,12,
	15,15,15,15,
	15,15,15,13,9,6,4,4,5,4,4,4,4,4,6,9,
	13,15,15,15,
	15,15,13,9,6,4,4,4,4,4,5,4,4,4,5,5,
	10,13,15,15,
	15,14,12,7,4,5,5,4,4,4,5,5,5,4,4,4,
	7,12,14,15,
	15,14,9,4,5,4,4,5,5,6,5,4,4,4,5,4,
	4,9,14,15,
	15,13,8,4,5,5,4,5,2,0,1,2,6,5,5,4,
	5,8,12,15,
	14,12,7,5,4,4,5,3,0,0,0,0,1,5,4,5,
	4,7,12,14,
	15,12,7,4,4,4,5,1,0,0,0,0,0,3,4,4,
	4,7,12,15,
	14,12,7,6,5,5,5,2,0,0,0,0,0,5,5,5,
	5,7,12,14,
	15,13,8,4,4,5,5,3,0,0,0,0,3,5,4,4,
	5,9,13,15,
	15,14,11,4,5,4,4,6,3,2,2,3,5,4,5,4,
	4,10,14,15,
	15,14,12,9,4,5,4,6,5,6,4,4,4,4,5,5,
	8,12,15,15,
	15,15,14,11,7,4,4,4,4,4,4,5,4,6,4,7,
	11,14,15,15,
	15,15,15,14,11,8,4,4,4,5,4,5,4,4,8,11,
	14,15,15,15,
	15,15,15,15,14,12,9,8,6,4,4,5,8,9,12,14,
	15,15,15,15,
	15,15,15,15,15,15,13,12,12,11,11,11,13,13,14,15,
	15,15,15,15,
	15,15,15,15,15,15,15,15,14,14,14,15,15,15,15,15,
	15,15,15,15
	};
