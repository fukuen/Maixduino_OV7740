
#include <Maixduino_OV7740.h>

//////////// HAL ///////////////
#include "sysctl.h"
#include "fpioa.h"
#include "dvp.h"
#include "sleep.h"
#include "stdlib.h"
#include "utils.h"
#include "plic.h"
#include "math.h"
#include "Arduino.h" // millis
#include "stdio.h"
#include "cambus.h"

volatile static uint8_t g_dvp_finish_flag = 0;

#define IM_MAX(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define IM_MIN(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define IM_DIV(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _b ? (_a / _b) : 0; })
#define IM_MOD(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _b ? (_a % _b) : 0; })


typedef enum {
    ACTIVE_LOW,
    ACTIVE_HIGH,
    ACTIVE_BINOCULAR,
} polarity_t;

#define DCMI_RESET_LOW()      dvp->cmos_cfg &= ~DVP_CMOS_RESET
#define DCMI_RESET_HIGH()     dvp->cmos_cfg |= DVP_CMOS_RESET
#define DCMI_PWDN_LOW()       dvp->cmos_cfg |= DVP_CMOS_POWER_DOWN
#define DCMI_PWDN_HIGH()      dvp->cmos_cfg &= ~DVP_CMOS_POWER_DOWN

#define SVGA_HSIZE     (800)
#define SVGA_VSIZE     (600)

#define UXGA_HSIZE     (1600)
#define UXGA_VSIZE     (1200)
static const uint8_t ov7740_default[][2] = { //k210 
	{0x47, 0x02}  ,
	{0x17, 0x27}  ,
	{0x04, 0x40}  ,
	{0x1B, 0x81}  ,
	{0x29, 0x17}  ,
	{0x5F, 0x03}  ,
	{0x3A, 0x09}  ,
	{0x33, 0x44}  ,
	{0x68, 0x1A}  ,
	{0x14, 0x38}  ,
	{0x5F, 0x04}  ,
	{0x64, 0x00}  ,
	{0x67, 0x90}  ,
	{0x27, 0x80}  ,
	{0x45, 0x41}  ,
	{0x4B, 0x40}  ,
	{0x36, 0x2f}  ,
	{0x11, 0x00}  ,  // 60fps
	{0x36, 0x3f}  ,
	// {0x0c, 0x12}  , // default YUYV
	{0x12, 0x00}  ,
	{0x17, 0x25}  ,
	{0x18, 0xa0}  ,
	{0x1a, 0xf0}  ,
	{0x31, 0x50}  ,
	{0x32, 0x78}  ,
	{0x82, 0x3f}  ,
	{0x85, 0x08}  ,
	{0x86, 0x02}  ,
	{0x87, 0x01}  ,
	{0xd5, 0x10}  ,
	{0x0d, 0x34}  ,
	{0x19, 0x03}  ,
	{0x2b, 0xf8}  ,
	{0x2c, 0x01}  ,
	{0x53, 0x00}  ,
	{0x89, 0x30}  ,
	{0x8d, 0x30}  ,
	{0x8f, 0x85}  ,
	{0x93, 0x30}  ,
	{0x95, 0x85}  ,
	{0x99, 0x30}  ,
	{0x9b, 0x85}  ,
	{0xac, 0x6E}  ,
	{0xbe, 0xff}  ,
	{0xbf, 0x00}  ,
	{0x38, 0x14}  ,
	{0xe9, 0x00}  ,
	{0x3D, 0x08}  ,
	{0x3E, 0x80}  ,
	{0x3F, 0x40}  ,
	{0x40, 0x7F}  ,
	{0x41, 0x6A}  ,
	{0x42, 0x29}  ,
	{0x49, 0x64}  ,
	{0x4A, 0xA1}  ,
	{0x4E, 0x13}  ,
	{0x4D, 0x50}  ,
	{0x44, 0x58}  ,
	{0x4C, 0x1A}  ,
	{0x4E, 0x14}  ,
	{0x38, 0x11}  ,
	{0x84, 0x70}  ,
	{0,0}

};

#define NUM_BRIGHTNESS_LEVELS (9)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS][2] = {
    {0x06, 0x40}, /* -4 */
    {0x06, 0x30}, /* -3 */
    {0x06, 0x20}, /* -2 */
    {0x06, 0x10}, /* -1 */
    {0x0E, 0x00}, /*  0 */
    {0x0E, 0x10}, /* +1 */
    {0x0E, 0x20}, /* +2 */
    {0x0E, 0x30}, /* +3 */
    {0x0E, 0x40}, /* +4 */
};

#define NUM_CONTRAST_LEVELS (9)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS][3] = {
    {0x20, 0x10, 0xD0}, /* -4 */
    {0x20, 0x14, 0x80}, /* -3 */
    {0x20, 0x18, 0x48}, /* -2 */
    {0x20, 0x1C, 0x20}, /* -1 */
    {0x20, 0x20, 0x00}, /*  0 */
    {0x20, 0x24, 0x00}, /* +1 */
    {0x20, 0x28, 0x00}, /* +2 */
    {0x20, 0x2C, 0x00}, /* +3 */
    {0x20, 0x30, 0x00}, /* +4 */
};

#define NUM_SATURATION_LEVELS (9)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS][2] = {
    {0x00, 0x00}, /* -4 */
    {0x10, 0x10}, /* -3 */
    {0x20, 0x20}, /* -2 */
    {0x30, 0x30}, /* -1 */
    {0x40, 0x40}, /*  0 */
    {0x50, 0x50}, /* +1 */
    {0x60, 0x60}, /* +2 */
    {0x70, 0x70}, /* +3 */
    {0x80, 0x80}, /* +4 */
};

#define OV7740_SET_MIRROR(r, x)   ((r&0xBF)|((x&1)<<6))
#define OV7740_SET_FLIP(r, x)     ((r&0x7F)|((x&1)<<7))
#define OV7740_SET_SP(r, x)     ((r&0xEE)|((x&1)<<4)|(x&1))


uint8_t gc0328_default_regs[][2] = {
	{0xfe , 0x80},
	{0xfe , 0x80},
	{0xfc , 0x16},
	{0xfc , 0x16},
	{0xfc , 0x16},
	{0xfc , 0x16},
	{0xf1 , 0x00},
	{0xf2 , 0x00},
	{0xfe , 0x00},
	{0x4f , 0x00},
	{0x42 , 0x00},  
	{0x03 , 0x00},  
	{0x04 , 0xc0},  
	{0x77 , 0x62},  
	{0x78 , 0x40},  
	{0x79 , 0x4d},  

	{0xfe , 0x00},
	{0x16 , 0x00},
	{0x17 , 0x14},
	{0x18 , 0x0e},
	{0x19 , 0x06},

	{0x1b , 0x48},
	{0x1f , 0xC8},
	{0x20 , 0x01},
	{0x21 , 0x78},
	{0x22 , 0xb0},
	{0x23 , 0x04},//0x06  20140519 GC0328C
	{0x24 , 0x11}, 
	{0x26 , 0x00},

	//global gain for range 
	{0x70 , 0x85},   

	/////////////banding/////////////
	{0x05 , 0x00},//hb
	{0x06 , 0x6a},//
	{0x07 , 0x00},//vb
	{0x08 , 0x0c},//
	{0xfe , 0x01},//
	{0x29 , 0x00},//anti-flicker step [11:8]
	{0x2a , 0x96},//anti-flicker step [7:0]
	{0xfe , 0x00},//

	///////////////AWB//////////////
	{0xfe , 0x01},
	{0x50 , 0x00},
	{0x4f , 0x00},
	{0x4c , 0x01},
	{0x4f , 0x00},
	{0x4f , 0x00},
	{0x4f , 0x00},
	{0x4f , 0x00},
	{0x4f , 0x00}, 
	{0x4d , 0x30},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x40},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x50},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x60},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x70},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4f , 0x01},
	{0x50 , 0x88},
	{0xfe , 0x00},

	//////////// BLK//////////////////////
	{0xfe , 0x00}, 
	{0x27 , 0xb7},
	{0x28 , 0x7F},
	{0x29 , 0x20},
	{0x33 , 0x20},
	{0x34 , 0x20},
	{0x35 , 0x20},
	{0x36 , 0x20},
	{0x32 , 0x08},
	{0x3b , 0x00}, 
	{0x3c , 0x00},
	{0x3d , 0x00},
	{0x3e , 0x00},
	{0x47 , 0x00},
	{0x48 , 0x00}, 

	//////////// block enable/////////////
	{0x40 , 0x7f}, 
	{0x41 , 0x26}, 
	{0x42 , 0xfb},
	{0x44 , 0x00}, //yuv
	{0x45 , 0x00},
	{0x46 , 0x03},
	{0x4f , 0x01},
	{0x4b , 0x01},
	{0x50 , 0x01}, 

	/////DN & EEINTP/////
	{0x7e , 0x0a}, 
	{0x7f , 0x03}, 
	{0x81 , 0x15}, 
	{0x82 , 0x85},    
	{0x83 , 0x03},
	{0x84 , 0xe5},
	{0x90 , 0xac},    
	{0x92 , 0x02},
	{0x94 , 0x02},
	{0x95 , 0x32},    

	////////////YCP///////////
	{0xd1 , 0x28},
	{0xd2 , 0x28},
	{0xd3 , 0x40},
	{0xdd , 0x58},
	{0xde , 0x36},
	{0xe4 , 0x88},
	{0xe5 , 0x40}, 
	{0xd7 , 0x0e}, 

	///////////rgb gamma ////////////
	{0xfe , 0x00},
	{0xbf , 0x0e},
	{0xc0 , 0x1c},
	{0xc1 , 0x34},
	{0xc2 , 0x48},
	{0xc3 , 0x5a},
	{0xc4 , 0x6e},
	{0xc5 , 0x80},
	{0xc6 , 0x9c},
	{0xc7 , 0xb4},
	{0xc8 , 0xc7},
	{0xc9 , 0xd7},
	{0xca , 0xe3},
	{0xcb , 0xed},
	{0xcc , 0xf2},
	{0xcd , 0xf8},
	{0xce , 0xfd},
	{0xcf , 0xff},

	/////////////Y gamma//////////
	{0xfe , 0x00},
	{0x63 , 0x00},
	{0x64 , 0x05},
	{0x65 , 0x0b},
	{0x66 , 0x19},
	{0x67 , 0x2e},
	{0x68 , 0x40},
	{0x69 , 0x54},
	{0x6a , 0x66},
	{0x6b , 0x86},
	{0x6c , 0xa7},
	{0x6d , 0xc6},
	{0x6e , 0xe4},
	{0x6f , 0xff},

	//////////////ASDE/////////////
	{0xfe , 0x01},
	{0x18 , 0x02},
	{0xfe , 0x00},
	{0x98 , 0x00},    
	{0x9b , 0x20},    
	{0x9c , 0x80},    
	{0xa4 , 0x10},    
	{0xa8 , 0xB0},    
	{0xaa , 0x40},    
	{0xa2 , 0x23},    
	{0xad , 0x01},    

	//////////////abs///////////
	{0xfe , 0x01},
	{0x9c , 0x02},   
	{0x9e , 0xc0}, 
	{0x9f , 0x40},	

	////////////// AEC////////////
	{0x08 , 0xa0},
	{0x09 , 0xe8},
	{0x10 , 0x00},  
	{0x11 , 0x11},   
	{0x12 , 0x10},   
	{0x13 , 0x98},   
	{0x15 , 0xfc},   
	{0x18 , 0x03},
	{0x21 , 0xc0},   
	{0x22 , 0x60},   
	{0x23 , 0x30},
	{0x25 , 0x00},
	{0x24 , 0x14},
	{0x3d , 0x80},
	{0x3e , 0x40},

	////////////////AWB///////////
	{0xfe , 0x01},
	{0x51 , 0x88},
	{0x52 , 0x12},
	{0x53 , 0x80},
	{0x54 , 0x60},
	{0x55 , 0x01},
	{0x56 , 0x02},
	{0x58 , 0x00},
	{0x5b , 0x02},
	{0x5e , 0xa4},
	{0x5f , 0x8a},
	{0x61 , 0xdc},
	{0x62 , 0xdc},
	{0x70 , 0xfc},
	{0x71 , 0x10},
	{0x72 , 0x30},
	{0x73 , 0x0b},
	{0x74 , 0x0b},
	{0x75 , 0x01},
	{0x76 , 0x00},
	{0x77 , 0x40},
	{0x78 , 0x70},
	{0x79 , 0x00},
	{0x7b , 0x00},
	{0x7c , 0x71},
	{0x7d , 0x00},
	{0x80 , 0x70},
	{0x81 , 0x58},
	{0x82 , 0x98},
	{0x83 , 0x60},
	{0x84 , 0x58},
	{0x85 , 0x50},
	{0xfe , 0x00},	

	////////////////LSC////////////////
	{0xfe , 0x01},
	{0xc0 , 0x10},
	{0xc1 , 0x0c},
	{0xc2 , 0x0a},
	{0xc6 , 0x0e},
	{0xc7 , 0x0b},
	{0xc8 , 0x0a},
	{0xba , 0x26},
	{0xbb , 0x1c},
	{0xbc , 0x1d},
	{0xb4 , 0x23},
	{0xb5 , 0x1c},
	{0xb6 , 0x1a},
	{0xc3 , 0x00},
	{0xc4 , 0x00},
	{0xc5 , 0x00},
	{0xc9 , 0x00},
	{0xca , 0x00},
	{0xcb , 0x00},
	{0xbd , 0x00},
	{0xbe , 0x00},
	{0xbf , 0x00},
	{0xb7 , 0x07},
	{0xb8 , 0x05},
	{0xb9 , 0x05},
	{0xa8 , 0x07},
	{0xa9 , 0x06},
	{0xaa , 0x00},
	{0xab , 0x04},
	{0xac , 0x00},
	{0xad , 0x02},
	{0xae , 0x0d},
	{0xaf , 0x05},
	{0xb0 , 0x00},
	{0xb1 , 0x07},
	{0xb2 , 0x03},
	{0xb3 , 0x00},
	{0xa4 , 0x00},
	{0xa5 , 0x00},
	{0xa6 , 0x00},
	{0xa7 , 0x00},
	{0xa1 , 0x3c},
	{0xa2 , 0x50},
	{0xfe , 0x00},

	///////////////CCT ///////////
	{0xb1 , 0x12},
	{0xb2 , 0xf5},
	{0xb3 , 0xfe},
	{0xb4 , 0xe0},
	{0xb5 , 0x15},
	{0xb6 , 0xc8},

	/////skin CC for front //////
	{0xb1 , 0x00},
	{0xb2 , 0x00},
	{0xb3 , 0x05},
	{0xb4 , 0xf0},
	{0xb5 , 0x00},
	{0xb6 , 0x00},
	
	///////////////AWB////////////////
	{0xfe , 0x01},
	{0x50 , 0x00},
	{0xfe , 0x01}, 
	{0x4f , 0x00},
	{0x4c , 0x01},
	{0x4f , 0x00},
	{0x4f , 0x00},
	{0x4f , 0x00}, 
	{0x4d , 0x34},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4e , 0x02},
	{0x4e , 0x02},
	{0x4d , 0x44},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x53},
	{0x4e , 0x00},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x65},
	{0x4e , 0x04},
	{0x4d , 0x73},
	{0x4e , 0x20},
	{0x4d , 0x83},
	{0x4e , 0x20},
	{0x4f , 0x01}, 
	{0x50 , 0x88}, 

    {0xfe , 0x00},
    // window
        //windowing mode
	// {0x09 , 0x00},
    // {0x0a , 0x78},
	// {0x0b , 0x00},
	// {0x0c , 0xa0},
    // {0x0d , 0x00},
	// {0x0e , 0xf8},
	// {0x0f , 0x01},
	// {0x10 , 0x48},
        //crop mode 
    {0x50 , 0x01},
    // {0x51, 0x00},
    // {0x52, 0x78},
    // {0x53, 0x00},
    // {0x54, 0xa0},
    // {0x55, 0x00},
    // {0x56, 0xf0},
    // {0x57, 0x01},
    // {0x58, 0x40},
    //subsample 1/2
    {0x59, 0x22},
    {0x5a, 0x00},
    {0x5b, 0x00},
    {0x5c, 0x00},
    {0x5d, 0x00},
    {0x5e, 0x00},
    {0x5f, 0x00},
    {0x60, 0x00},
    {0x61, 0x00},
    {0x62, 0x00},

    //Exp level
    {0xfe, 0x01},
    {0x2b , 0x02},//exp level 0  30fps => 16fps
	{0x2c , 0x00},//			 
	{0x2d , 0x02},//exp level 1  12.50fps
	{0x2e , 0x00},//			 
	{0x2f , 0x02},//exp level 2  10.00fps
	{0x30 , 0x00},//			 
	{0x31 , 0x02},//exp level 3  7.14fps
	{0x32 , 0x00},//
    {0x33, 0x00},

	/////////output//////// 
	{0xfe , 0x00},	
	{0xf1 , 0x07}, 
	{0xf2 , 0x01}, 

    {0x00, 0x00}
};

static const uint8_t qvga_config[][2] = { //k210 
    {0xfe , 0x00},
    // window
        //windowing mode
	// {0x09 , 0x00},
    // {0x0a , 0x78},
	// {0x0b , 0x00},
	// {0x0c , 0xa0},
    // {0x0d , 0x00},
	// {0x0e , 0xf8},
	// {0x0f , 0x01},
	// {0x10 , 0x48},
        //crop mode 
    {0x50 , 0x01},
    // {0x51, 0x00},
    // {0x52, 0x78},
    // {0x53, 0x00},
    // {0x54, 0xa0},
    // {0x55, 0x00},
    // {0x56, 0xf0},
    // {0x57, 0x01},
    // {0x58, 0x40},
    //subsample 1/2
    {0x59, 0x22},
    {0x5a, 0x00},
    {0x5b, 0x00},
    {0x5c, 0x00},
    {0x5d, 0x00},
    {0x5e, 0x00},
    {0x5f, 0x00},
    {0x60, 0x00},
    {0x61, 0x00},
    {0x62, 0x00},

    {0x00, 0x00}
};

static const uint8_t vga_config[][2] = { //k210 
    {0xfe , 0x00},
    // window
        //windowing mode
	// {0x09 , 0x00},
    // {0x0a , 0x78},
	// {0x0b , 0x00},
	// {0x0c , 0xa0},
    // {0x0d , 0x00},
	// {0x0e , 0xf8},
	// {0x0f , 0x01},
	// {0x10 , 0x48},
        //crop mode 
    {0x50 , 0x00},
    // {0x51, 0x00},
    // {0x52, 0x78},
    // {0x53, 0x00},
    // {0x54, 0xa0},
    // {0x55, 0x00},
    // {0x56, 0xf0},
    // {0x57, 0x01},
    // {0x58, 0x40},
    //subsample 1/2
    // {0x59, 0x00},
    // {0x5a, 0x00},
    // {0x5b, 0x00},
    // {0x5c, 0x00},
    // {0x5d, 0x00},
    // {0x5e, 0x00},
    // {0x5f, 0x00},
    // {0x60, 0x00},
    // {0x61, 0x00},
    // {0x62, 0x00},
    {0x00, 0x00}
};

static const uint8_t gc0328_yuv422_regs[][2] = {
    {0xfe , 0x00},
	{0x44 , 0x00}, //yuv
    {0x00, 0x00}
};

static const uint8_t gc0328_rgb565_regs[][2] = {
    {0xfe , 0x00},
	{0x44 , 0x06},
    {0x00, 0x00}
};



Maixduino_OV7740::Maixduino_OV7740( framesize_t frameSize, pixformat_t pixFormat)
:Camera(frameSize, pixFormat),
_dataBuffer(NULL), _aiBuffer(NULL),
_resetPoliraty(ACTIVE_HIGH), _pwdnPoliraty(ACTIVE_HIGH),
_slaveAddr(0x00),
_id(0)
{
    configASSERT(pixFormat == PIXFORMAT_RGB565 || pixFormat==PIXFORMAT_YUV422);
}



Maixduino_OV7740::Maixduino_OV7740(uint16_t width, uint16_t height, pixformat_t pixFormat)
:Camera(width, height, pixFormat),
_dataBuffer(NULL), _aiBuffer(NULL),
_resetPoliraty(ACTIVE_HIGH), _pwdnPoliraty(ACTIVE_HIGH),
_slaveAddr(0x00),
_id(0)
{
    configASSERT(pixFormat == PIXFORMAT_RGB565 || pixFormat==PIXFORMAT_YUV422);
}


Maixduino_OV7740::~Maixduino_OV7740()
{
    end();
}

bool Maixduino_OV7740::begin()
{
    if(_dataBuffer)
        free(_dataBuffer);
    if(_aiBuffer)
        free(_aiBuffer);
    _dataBuffer = (uint8_t*)malloc(_width*_height*2); //RGB565
    if(!_dataBuffer)
    {
        _width = 0;
        _height = 0;
        return false;
    }
    _aiBuffer = (uint8_t*)malloc(_width*_height*3);   //RGB888
    if(!_aiBuffer)
    {
        _width = 0;
        _height = 0;
        free(_dataBuffer);
        return false;
    }
    if(!reset())
        return false;
    if( !setPixFormat(_pixFormat))
        return false;
    if(!setFrameSize(_frameSize))
        return false;
    return true;
}

void Maixduino_OV7740::end()
{
    if(_dataBuffer)
        free(_dataBuffer);
    if(_aiBuffer)
        free(_aiBuffer);
    _dataBuffer = nullptr;
    _aiBuffer   = nullptr;
}

bool Maixduino_OV7740::reset()
{
    if(dvpInit() != 0)
        return false;
//    if(ov7740_reset() != 0)
//        return false;
    if(_id == 0x9d)
    {
        if(gc0328_reset() != 0)
            return false;
    }
    else
    {
        if(ov7740_reset() != 0)
            return false;
    }
    if(dvpInitIrq() != 0)
        return false;
    return true;
}

bool Maixduino_OV7740::setPixFormat(pixformat_t pixFormat)
{
//    if(ov7740_set_pixformat(pixFormat) != 0)
//        return false;
    if(_id == 0x9d)
    {
        if(gc0328_set_pixformat(pixFormat) != 0)
            return false;
    }
    else
    {
        if(ov7740_set_pixformat(pixFormat) != 0)
            return false;
    }
    return true;
}

bool Maixduino_OV7740::setFrameSize(framesize_t frameSize)
{
//    if(ov7740_set_framesize(frameSize) != 0)
//        return false;
    if(_id == 0x9d)
    {
        if(gc0328_set_framesize(frameSize) != 0)
            return false;
    }
    else
    {
        if(ov7740_set_framesize(frameSize) != 0)
            return false;
    }
    return true;
}  

bool Maixduino_OV7740::run(bool run)
{
	if(run)
	{
		dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
		plic_irq_enable(IRQN_DVP_INTERRUPT);
		dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);
	}
	else{
		plic_irq_disable(IRQN_DVP_INTERRUPT);
		dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
		dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
	}
    return true;
}

int Maixduino_OV7740::id()
{
    return _id;
}

/**
 * @return pixels 
 *         If pixels format is RGB565: return RGB565 pixels with every uint16_t one pixel, e.g. RED: 0xF800
 */
uint8_t* Maixduino_OV7740::snapshot()
{
    if ( sensor_snapshot() != 0)
        return nullptr;
    return _dataBuffer;
}

void Maixduino_OV7740::setRotaion(uint8_t rotation)
{
    //FIXME
    if (rotation == 0)
    {
        //
    }
    else if (rotation == 2)
    {
        ov7740_set_hmirror(1);
        ov7740_set_vflip(1);
    }
}

void Maixduino_OV7740::setInvert(bool invert)
{
    if (invert)
    {
        ov7740_set_hmirror(1); 
        ov7740_set_vflip(1);
    }
    else{
        ov7740_set_hmirror(0); 
        ov7740_set_vflip(0);
    }
    
    return;
}



int Maixduino_OV7740::dvpInit(uint32_t freq)
{
    // just support RGB565 and YUV442 on k210
    configASSERT(_pixFormat==PIXFORMAT_RGB565 || _pixFormat==PIXFORMAT_YUV422);
    _freq  = freq;

	fpioa_set_function(47, FUNC_CMOS_PCLK);
	fpioa_set_function(46, FUNC_CMOS_XCLK);
	fpioa_set_function(45, FUNC_CMOS_HREF);
	fpioa_set_function(44, FUNC_CMOS_PWDN);
	fpioa_set_function(43, FUNC_CMOS_VSYNC);
	fpioa_set_function(42, FUNC_CMOS_RST);
//	fpioa_set_function(41, FUNC_SCCB_SCLK);
//	fpioa_set_function(40, FUNC_SCCB_SDA);

    /* Do a power cycle */
//    DCMI_PWDN_HIGH();
//    msleep(10);

//    DCMI_PWDN_LOW();
//    msleep(10);

    // Initialize the camera bus, 8bit reg
//    dvp_init(8);
    cambus_init(8, 2, 41, 40, 0, 0);
	 // Initialize dvp interface
	dvp_set_xclk_rate(freq);

    /* Do a power cycle */
    DCMI_PWDN_HIGH();
    msleep(10);

    DCMI_PWDN_LOW();
    msleep(10);

    if(0 == sensor_ov_detect()){//find ov sensor
        // printf("find ov sensor\n");
        dvp_set_xclk_rate(22000000);
    }
    else if(0 == sensro_gc_detect()){//find gc0328 sensor
        // printf("find gc3028\n");
    }
    else{
        return -1;
    }

	dvp->cmos_cfg |= DVP_CMOS_CLK_DIV(3) | DVP_CMOS_CLK_ENABLE;
    dvp_enable_burst();
	dvp_disable_auto();
	dvp_set_output_enable(DVP_OUTPUT_AI, 1);	//enable to AI
	dvp_set_output_enable(DVP_OUTPUT_DISPLAY, 1);	//enable to lcd
    if( _pixFormat == PIXFORMAT_YUV422)
        dvp_set_image_format(DVP_CFG_YUV_FORMAT);
    else
	    dvp_set_image_format(DVP_CFG_RGB_FORMAT);
	dvp_set_image_size(_width, _height);	//set QVGA default
	dvp_set_ai_addr( (uint32_t)((long)_aiBuffer), (uint32_t)((long)(_aiBuffer+_width*_height)), (uint32_t)((long)(_aiBuffer+_width*_height*2)));
	dvp_set_display_addr( (uint32_t)((long)_dataBuffer) );

/*
    if(0 == sensor_ov_detect()){//find ov sensor
        // printf("find ov sensor\n");
        return 0;
    }
    else if(0 == sensro_gc_detect()){//find gc0328 sensor
        // printf("find gc3028\n");
        return 0;
    }
*/

    return 0;
}

/*
int Maixduino_OV7740::cambus_scan()
{

	uint16_t manuf_id = 0;
	uint16_t device_id = 0;
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
		cambus_read_id(addr ,&manuf_id,&device_id);
		if(0xffff != device_id)
		{
			return addr ;
		}
    }
    return 0;
}

int Maixduino_OV7740::cambus_read_id(uint8_t addr,uint16_t *manuf_id, uint16_t *device_id)
{
	dvp_sccb_send_data(addr, 0xFF, 0x01);
	*manuf_id = (dvp_sccb_receive_data(addr, 0x1C) << 8) | dvp_sccb_receive_data(addr, 0x1D);
	*device_id = (dvp_sccb_receive_data(addr, 0x0A) << 8) | dvp_sccb_receive_data(addr, 0x0B);
	return 0;
}

int Maixduino_OV7740::cambus_scan_gc0328(void)
{
    dvp_sccb_send_data(GC0328_ADDR, 0xFE, 0x00);
    msleep(4);
    uint8_t id = dvp_sccb_receive_data(GC0328_ADDR, 0xf0);
    if (id != 0x9d)
    {
        // printf("error gc0328 detect, ret id is 0x%x\r\n", id);
        return 0;
    }
    return id;
}

int Maixduino_OV7740::cambus_readb(uint8_t slv_addr, uint8_t reg_addr, uint8_t *reg_data)
{

    int ret = 0;
	*reg_data = dvp_sccb_receive_data(slv_addr, reg_addr);

	if(0xff == *reg_data)
		ret = -1;

    return ret;

}


int Maixduino_OV7740::cambus_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data)
{

	dvp_sccb_send_data(slv_addr,reg_addr,reg_data);
	return 0;
}

int Maixduino_OV7740::cambus_readw(uint8_t slv_addr, uint8_t reg_addr, uint16_t *reg_data)
{
    return 0;
}

int Maixduino_OV7740::cambus_writew(uint8_t slv_addr, uint8_t reg_addr, uint16_t reg_data)
{
    return 0;
}

int Maixduino_OV7740::cambus_readw2(uint8_t slv_addr, uint16_t reg_addr, uint16_t *reg_data)
{
    return 0;
}

int Maixduino_OV7740::cambus_writew2(uint8_t slv_addr, uint16_t reg_addr, uint16_t reg_data)
{
    return 0;
}
*/

int Maixduino_OV7740::sensor_ov_detect()
{
    /* Reset the sensor */
    DCMI_RESET_HIGH();
    msleep(10);

    DCMI_RESET_LOW();
    msleep(10);

    /* Probe the ov sensor */
    _slaveAddr = cambus_scan();
    if (_slaveAddr == 0) {
        /* Sensor has been held in reset,
           so the reset line is active low */
        _resetPoliraty = ACTIVE_LOW;

        /* Pull the sensor out of the reset state,systick_sleep() */
        DCMI_PWDN_HIGH();
        msleep(10);
        DCMI_PWDN_LOW();
        msleep(10);
        DCMI_RESET_HIGH();
        msleep(10);

        /* Probe again to set the slave addr */
        _slaveAddr = cambus_scan();
        if (_slaveAddr == 0) {
            _pwdnPoliraty = ACTIVE_LOW;

            DCMI_PWDN_HIGH();
            msleep(10);
            DCMI_RESET_LOW();
            msleep(10);
            DCMI_RESET_HIGH();
            msleep(10);

            _slaveAddr = cambus_scan();
            if (_slaveAddr == 0) {
                _resetPoliraty = ACTIVE_HIGH;

                DCMI_PWDN_LOW();
                msleep(10);
                DCMI_PWDN_HIGH();
                msleep(10);
                DCMI_RESET_LOW();
                msleep(10);

                _slaveAddr = cambus_scan();
                if(_slaveAddr == 0) {
                    //should do something?
                    return -2;
                }
            }
        }
    }

    // Clear sensor chip ID.
    _id = 0;

    if (_slaveAddr == LEPTON_ID) {
        _id = LEPTON_ID;
		/*set LEPTON xclk rate*/
		/*lepton_init*/
    } else {
        // Read ON semi sensor ID.
        cambus_readb(_slaveAddr, ON_CHIP_ID, &_id);
        if (_id == MT9V034_ID) {
			/*set MT9V034 xclk rate*/
			/*mt9v034_init*/
        } else { // Read OV sensor ID.
            cambus_readb(_slaveAddr, OV_CHIP_ID, &_id);
            // Initialize sensor struct.
            switch (_id) {
                case OV9650_ID:
					/*ov9650_init*/
                    break;
                case OV2640_ID:
                    printf("detect ov2640, id:%x\n", _slaveAddr);
                    break;
                case OV7740_ID:
                    printf("detect ov7740, id:%x\n", _slaveAddr);
                    break;
                default:
                    // Sensor is not supported.
                    return -3;
            }
        }
    }

    // if (init_ret != 0 ) {
    //     // Sensor init failed.
    //     return -4;
    // }
    return 0;
}

int Maixduino_OV7740::sensro_gc_detect()
{
    DCMI_PWDN_HIGH();//enable gc0328 要恢复 normal 工作模式，需将 PWDN pin 接入低电平即可，同时写入初始化寄存器即可
//    DCMI_PWDN_LOW();
    DCMI_RESET_LOW();//reset gc3028
    msleep(10);
    DCMI_RESET_HIGH();
    msleep(10);
    uint8_t id = cambus_scan_gc0328();
    if(0 == id)
    {
        return -3;
    }
    else
    {
        // printf("[MAIXPY]: gc0328 id = %x\n",id); 
        _slaveAddr = GC0328_ADDR;
        _id = id;
        cambus_set_writeb_delay(2);
    }
    return 0;
}

#ifdef __cplusplus
extern "C" {
#endif

static int sensor_irq(void *ctx)
{
	if (dvp_get_interrupt(DVP_STS_FRAME_FINISH)) {	//frame end
		dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
		g_dvp_finish_flag = 1;
	} else {	//frame start
        if(g_dvp_finish_flag == 0)  //only we finish the convert, do transmit again
            dvp_start_convert();	//so we need deal img ontime, or skip one framebefore next
		dvp_clear_interrupt(DVP_STS_FRAME_START);
	}

	return 0;
}

#ifdef __cplusplus
}
#endif



int Maixduino_OV7740::dvpInitIrq()
{
	dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
	plic_set_priority(IRQN_DVP_INTERRUPT, 2);
    /* set irq handle */
	plic_irq_register(IRQN_DVP_INTERRUPT, sensor_irq, (void*)NULL);

	plic_irq_disable(IRQN_DVP_INTERRUPT);
	dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
	dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);

	return 0;
}




int Maixduino_OV7740::ov7740_reset()
{
    int i=0;
    const uint8_t (*regs)[2];

    /* Reset all registers */
    cambus_writeb(_slaveAddr, 0x12, 0x80);

    /* delay n ms */
    msleep(2);

    i = 0;
    regs = ov7740_default;//default_regs,ov7740_default
    /* Write initial regsiters */
    while (regs[i][0]) {
        cambus_writeb(_slaveAddr, regs[i][0], regs[i][1]);
        i++;
    }

    return 0;
}

int Maixduino_OV7740::gc0328_reset()
{
    uint16_t index = 0;
    
    cambus_writeb(GC0328_ADDR, 0xfe, 0x01);
//    msleep(1);
    for (index = 0; gc0328_default_regs[index][0]; index++)
    {
        if(gc0328_default_regs[index][0] == 0xff){
//            mp_hal_delay_ms(sensor_default_regs[index][1]);
            msleep(gc0328_default_regs[index][1]);
            continue;
        }
        if(gc0328_default_regs[index][0] == 0x00){
            continue;
        }
        // mp_printf(&mp_plat_print, "0x12,0x%02x,0x%02x,\r\n", sensor_default_regs[index][0], sensor_default_regs[index][1]);//debug
        cambus_writeb(GC0328_ADDR, gc0328_default_regs[index][0], gc0328_default_regs[index][1]);
        // mp_hal_delay_ms(1);
//        msleep(1);
    }
    return 0;
}

int Maixduino_OV7740::ov7740_read_reg(uint8_t reg_addr)
{
    uint8_t reg_data;
    if (cambus_readb(_slaveAddr, reg_addr, &reg_data) != 0) {
        return -1;
    }
    return reg_data;
}

int Maixduino_OV7740::ov7740_write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    return cambus_writeb(_slaveAddr, reg_addr, reg_data);
}


int Maixduino_OV7740::ov7740_set_pixformat(pixformat_t pixformat)
{
    return 0;
}

int Maixduino_OV7740::ov7740_set_framesize(framesize_t framesize)
{
    int ret=0;
    uint8_t clkrc;
    uint16_t w = _width;
    uint16_t h = _height;

    // VGA
    if ((w > 320) || (h > 240))
    {
        ret |= cambus_writeb(_slaveAddr, 0x31, 0xA0);
        ret |= cambus_writeb(_slaveAddr, 0x32, 0xF0);
        ret |= cambus_writeb(_slaveAddr, 0x82, 0x32);
    }
    // QVGA
    else if( ((w <= 320) && (h <= 240)) && ((w > 160) || (h > 120)) )
    {
        ret |= cambus_writeb(_slaveAddr, 0x31, 0x50);
        ret |= cambus_writeb(_slaveAddr, 0x32, 0x78);
        ret |= cambus_writeb(_slaveAddr, 0x82, 0x3F);
    }
    // QQVGA
    else
    {
        ret |= cambus_writeb(_slaveAddr, 0x31, 0x28);
        ret |= cambus_writeb(_slaveAddr, 0x32, 0x3c);
        ret |= cambus_writeb(_slaveAddr, 0x82, 0x3F);
    }    

    /* delay n ms */
    msleep(30);
	dvp_set_image_size(w, h);
    return ret;
}

int Maixduino_OV7740::gc0328_set_pixformat(pixformat_t pixformat)
{
    int i=0;
    const uint8_t (*regs)[2]=NULL;

    /* read pixel format reg */
    switch (pixformat) {
        case PIXFORMAT_RGB565:
            regs = gc0328_rgb565_regs;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
            regs = gc0328_yuv422_regs;
            break;
        default:
            return -1;
    }

    /* Write initial regsiters */
    while (regs[i][0]) {
        cambus_writeb(_slaveAddr, regs[i][0], regs[i][1]);
        i++;
    }
    switch (pixformat) {
        case PIXFORMAT_RGB565:
			dvp_set_image_format(DVP_CFG_RGB_FORMAT);
            break;
        case PIXFORMAT_YUV422:
            dvp_set_image_format(DVP_CFG_YUV_FORMAT);
            break;
        case PIXFORMAT_GRAYSCALE:
			dvp_set_image_format(DVP_CFG_Y_FORMAT);
            break;
        case PIXFORMAT_JPEG:
			dvp_set_image_format(DVP_CFG_RGB_FORMAT);
            break;
        default:
            return -1;
    }
    /* delay n ms */
    msleep(30);
    return 0;
}

int Maixduino_OV7740::gc0328_set_framesize(framesize_t framesize)
{
    int ret=0;
    uint16_t w = _width;
    uint16_t h = _height;

    int i=0;
    const uint8_t (*regs)[2];

    if ((w <= 320) && (h <= 240)) {
        regs = qvga_config;
    } else {
        regs = vga_config;
    }

    while (regs[i][0]) {
        cambus_writeb(_slaveAddr, regs[i][0], regs[i][1]);
//        msleep(1);
        i++;
    }
    /* delay n ms */
//    mp_hal_delay_ms(30);
    msleep(30);
	dvp_set_image_size(w, h);
    return ret;
}

int Maixduino_OV7740::ov7740_set_framerate(framerate_t framerate)
{
    int ret = 0;
    switch(framerate)
    {
        case FRAMERATE_60FPS:
            ret |= cambus_writeb(_slaveAddr, 0x11, 0x00);
            ret |= cambus_writeb(_slaveAddr, 0x55, 0x40);
            ret |= cambus_writeb(_slaveAddr, 0x2b, 0xF0);
            ret |= cambus_writeb(_slaveAddr, 0x2c, 0x01);
            break;
        case FRAMERATE_30FPS:
            ret |= cambus_writeb(_slaveAddr, 0x11, 0x01);
            ret |= cambus_writeb(_slaveAddr, 0x55, 0x40);
            ret |= cambus_writeb(_slaveAddr, 0x2b, 0xF0);
            ret |= cambus_writeb(_slaveAddr, 0x2c, 0x01);
            break;
//        case FRAMERATE_25FPS:
//            ret |= cambus_writeb(_slaveAddr, 0x11, 0x01);
//            ret |= cambus_writeb(_slaveAddr, 0x55, 0x40);
//            ret |= cambus_writeb(_slaveAddr, 0x2b, 0x5E);
//            ret |= cambus_writeb(_slaveAddr, 0x2c, 0x02);
//            break;
        case FRAMERATE_15FPS:
            ret |= cambus_writeb(_slaveAddr, 0x11, 0x03);
            ret |= cambus_writeb(_slaveAddr, 0x55, 0x40);
            ret |= cambus_writeb(_slaveAddr, 0x2b, 0xF0);
            ret |= cambus_writeb(_slaveAddr, 0x2c, 0x01);
            break;
        default:
            return -1;
    }
    return ret;
}

int Maixduino_OV7740::ov7740_set_contrast(int level)
{
    int ret=0;
    uint8_t tmp = 0;

    level += (NUM_CONTRAST_LEVELS / 2);
    if (level < 0 || level >= NUM_CONTRAST_LEVELS) {
        return -1;
    }
    ret |= cambus_readb(_slaveAddr, 0x81,&tmp);
    tmp |= 0x20;
    ret |= cambus_writeb(_slaveAddr, 0x81, tmp);
    ret |= cambus_readb(_slaveAddr, 0xDA,&tmp);
    tmp |= 0x04;
    ret |= cambus_writeb(_slaveAddr, 0xDA, tmp);
    ret |= cambus_writeb(_slaveAddr, 0xE1, contrast_regs[level][0]);
    ret |= cambus_writeb(_slaveAddr, 0xE2, contrast_regs[level][1]);
    ret |= cambus_writeb(_slaveAddr, 0xE3, contrast_regs[level][2]);
    ret |= cambus_readb(_slaveAddr, 0xE4,&tmp);
    tmp &= 0xFB;
    ret |= cambus_writeb(_slaveAddr, 0xE4, tmp);

    return ret;
}

int Maixduino_OV7740::ov7740_set_brightness(int level)
{
    int ret=0;
    uint8_t tmp = 0;

    level += (NUM_BRIGHTNESS_LEVELS / 2);
    if (level < 0 || level >= NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }
    ret |= cambus_readb(_slaveAddr, 0x81,&tmp);
    tmp |= 0x20;
    ret |= cambus_writeb(_slaveAddr, 0x81, tmp);
    ret |= cambus_readb(_slaveAddr, 0xDA,&tmp);
    tmp |= 0x04;
    ret |= cambus_writeb(_slaveAddr, 0xDA, tmp);
    ret |= cambus_writeb(_slaveAddr, 0xE4, brightness_regs[level][0]);
    ret |= cambus_writeb(_slaveAddr, 0xE3, brightness_regs[level][1]);

    return ret;
}

int Maixduino_OV7740::ov7740_set_saturation(int level)
{
    int ret=0;
    uint8_t tmp = 0;

    level += (NUM_SATURATION_LEVELS / 2 );
    if (level < 0 || level >= NUM_SATURATION_LEVELS) {
        return -1;
    }
    ret |= cambus_readb(_slaveAddr, 0x81,&tmp);
    tmp |= 0x20;
    ret |= cambus_writeb(_slaveAddr, 0x81, tmp);
    ret |= cambus_readb(_slaveAddr, 0xDA,&tmp);
    tmp |= 0x02;
    ret |= cambus_writeb(_slaveAddr, 0xDA, tmp);
    ret |= cambus_writeb(_slaveAddr, 0xDD, saturation_regs[level][0]);
    ret |= cambus_writeb(_slaveAddr, 0xDE, saturation_regs[level][1]);

    return ret;
}

int Maixduino_OV7740::ov7740_set_gainceiling( gainceiling_t gainceiling)
{
    int ret=0;

    uint8_t tmp = 0;
    uint8_t ceiling = (uint8_t)gainceiling;
    if(ceiling > GAINCEILING_32X)
        ceiling = GAINCEILING_32X;
    tmp = (ceiling & 0x07) << 4;
    ret |= cambus_writeb(_slaveAddr, 0x14, tmp);

    return ret;
}

int Maixduino_OV7740::ov7740_set_quality(int qs)
{
    return 0;
}

int Maixduino_OV7740::ov7740_set_colorbar(int enable)
{
    int ret=0;

    if(enable)
    {
        ret |= cambus_writeb(_slaveAddr, 0x38, 0x07);
        ret |= cambus_writeb(_slaveAddr, 0x84, 0x02);
    }
    else
    {
        ret |= cambus_writeb(_slaveAddr, 0x38, 0x07);
        ret |= cambus_writeb(_slaveAddr, 0x84, 0x00);
    }

    return ret;
}

int Maixduino_OV7740::ov7740_set_auto_gain(int enable, float gain_db, float gain_db_ceiling)
{
    int ret=0;

    uint8_t tmp = 0;
    uint16_t gain = (uint16_t)gain_db;
    uint8_t ceiling = (uint8_t)gain_db_ceiling;

    ret |= cambus_readb(_slaveAddr, 0x13, &tmp);
    if(enable != 0)
    {
        ret |= cambus_writeb(_slaveAddr, 0x13, tmp | 0x04);
    }
    else
    {
        ret |= cambus_writeb(_slaveAddr, 0x13, tmp & 0xFB);
        if(gain!=0xFFFF && (uint16_t)gain_db_ceiling!=0xFFFF)
        {
            ret |= cambus_readb(_slaveAddr, 0x15, &tmp);
            tmp = (tmp & 0xFC) | (gain>>8 & 0x03);
            ret |= cambus_writeb(_slaveAddr, 0x15, tmp);
            tmp = gain & 0xFF;
            ret |= cambus_writeb(_slaveAddr, 0x00, tmp);
            tmp = (ceiling & 0x07) << 4;
            ret |= cambus_writeb(_slaveAddr, 0x14, tmp);
        }
    }

    return ret;
}

int Maixduino_OV7740::ov7740_get_gain_db(float *gain_db)
{
    int ret=0;
    uint8_t tmp = 0;
    uint16_t gain;

    ret |= cambus_readb(_slaveAddr, 0x00, &tmp);
    gain = tmp;
    ret |= cambus_readb(_slaveAddr, 0x15, &tmp);
    gain |= ((uint16_t)(tmp & 0x03))<<8;
    *gain_db = (float)gain;
    return ret;
}

int Maixduino_OV7740::ov7740_set_auto_exposure(int enable, int exposure_us)
{
    int ret=0;
    uint8_t tmp = 0;

    ret |= cambus_readb(_slaveAddr, 0x13, &tmp);
    if(enable != 0)
    {
        ret |= cambus_writeb(_slaveAddr, 0x13, tmp | 0x01);
    }
    else
    {
        ret |= cambus_writeb(_slaveAddr, 0x13, tmp & 0xFE);
        ret |= cambus_writeb(_slaveAddr, 0x0F, (uint8_t)(exposure_us>>8));
        ret |= cambus_writeb(_slaveAddr, 0x10, (uint8_t)exposure_us);
    }

    return ret;
}

int Maixduino_OV7740::ov7740_get_exposure_us(int *exposure_us)
{
    int ret=0;
    uint8_t tmp = 0;

    ret |= cambus_readb(_slaveAddr, 0x0F, &tmp);
    *exposure_us = tmp<<8 & 0xFF00;
    ret |= cambus_readb(_slaveAddr, 0x10, &tmp);
    *exposure_us = tmp | *exposure_us;

    return ret;
}

int Maixduino_OV7740::ov7740_set_auto_whitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    int ret=0;
    uint8_t tmp = 0;

    ret |= cambus_readb(_slaveAddr, 0x80, &tmp);
    if(enable != 0)
    {
        ret |= cambus_writeb(_slaveAddr, 0x80, tmp | 0x14);
    }
    else
    {
        if((uint16_t)r_gain_db!= 0xFFFF && (uint16_t)g_gain_db!=0xFFFF && (uint16_t)b_gain_db!=0xFFFF)
        {
            ret |= cambus_writeb(_slaveAddr, 0x80, tmp & 0xEF);
            ret |= cambus_writeb(_slaveAddr, 0x01, (uint8_t)b_gain_db);
            ret |= cambus_writeb(_slaveAddr, 0x02, (uint8_t)r_gain_db);
            ret |= cambus_writeb(_slaveAddr, 0x03, (uint8_t)g_gain_db);
        }
        else
        {
            ret |= cambus_writeb(_slaveAddr, 0x80, tmp & 0xEB);
        }
    }

    return ret;
}

int Maixduino_OV7740::ov7740_get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    int ret=0;
    uint8_t tmp = 0;

    ret |= cambus_readb(_slaveAddr, 0x02, &tmp);
    *r_gain_db = (float)tmp;
    ret |= cambus_readb(_slaveAddr, 0x03, &tmp);
    *g_gain_db = (float)tmp;
    ret |= cambus_readb(_slaveAddr, 0x01, &tmp);
    *b_gain_db = (float)tmp;

    return ret;
}

int Maixduino_OV7740::ov7740_set_hmirror(int enable)
{
    uint8_t reg;
    int ret = cambus_readb(_slaveAddr, 0x0C, &reg);
    ret |= cambus_writeb(_slaveAddr, 0x0C, OV7740_SET_MIRROR(reg, enable));

    ret = cambus_readb(_slaveAddr, 0x16, &reg);
    ret |= cambus_writeb(_slaveAddr, 0x16, OV7740_SET_SP(reg, enable));

    return ret;
}

int Maixduino_OV7740::ov7740_set_vflip(int enable)
{
    uint8_t reg;
    int ret = cambus_readb(_slaveAddr, 0x0C, &reg);
    ret |= cambus_writeb(_slaveAddr, 0x0C, OV7740_SET_FLIP(reg, enable));

    return ret;
}

int Maixduino_OV7740::ov7740_set_special_effect(int sde)
{
    int ret;
    uint8_t reg;
    switch (sde)
    {
        case 0: // SDE_NORMAL:
            ret = cambus_readb(_slaveAddr, 0x81, &reg);
            ret |= cambus_writeb(_slaveAddr, 0x81, reg & 0xFE);
            ret = cambus_readb(_slaveAddr, 0xDA, &reg);
            ret |= cambus_writeb(_slaveAddr, 0xDA, reg & 0xBF);
            break;
        case 1: // SDE_NEGATIVE:
            ret = cambus_readb(_slaveAddr, 0x81, &reg);
            ret |= cambus_writeb(_slaveAddr, 0x81, reg | 0x01);
            ret = cambus_readb(_slaveAddr, 0xDA, &reg);
            ret |= cambus_writeb(_slaveAddr, 0xDA, reg | 0x40);
            break;
    
        default:
            return -1;
    }
    return ret;
}

int Maixduino_OV7740::reverse_u32pixel(uint32_t* addr,uint32_t length)
{
  if(NULL == addr)
    return -1;

  uint32_t data;
  uint32_t* pend = addr+length;
  for(;addr<pend;addr++)
  {
	  data = *(addr);
	  *(addr) = ((data & 0x000000FF) << 24) | ((data & 0x0000FF00) << 8) | 
                ((data & 0x00FF0000) >> 8) | ((data & 0xFF000000) >> 24) ;
  }  //1.7ms
  
  
  return 0;
}


int Maixduino_OV7740::sensor_snapshot( )
{	
    //wait for new frame
    g_dvp_finish_flag = 0;
    uint32_t start =  millis();
    while (g_dvp_finish_flag == 0)
    {
        usleep(50);
        if(millis() - start > 300)//wait for 300ms
            return -1;
    }
    reverse_u32pixel((uint32_t*)_dataBuffer, _width*_height/2);
    return 0;
}
