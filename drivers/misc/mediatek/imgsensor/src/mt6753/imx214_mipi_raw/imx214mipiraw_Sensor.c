/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX214mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "ois_otp_def.h"

#include "imx214mipiraw_Sensor.h"

#define PFX "IMX214_camera_sensor"
#define LOG_INF(format, args...)	pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define PREVIEW_16_9
#define VIDEO_16_9

#define OTP_SIZE 452
u8 OTPData[OTP_SIZE];
static int normal_fps = 1;
extern  int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata);


static imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX214_SENSOR_ID,
	.checksum_value =0xc15d2913,
	.pre = {//binning 4:3
#if 1
#ifdef PREVIEW_16_9
        .pclk = 184800000,              //record different mode's pclk
		.linelength = 5008,			//record different mode's linelength
        .framelength = 1224,           //record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1184,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
#else
        .pclk = 242400000,              //record different mode's pclk
		.linelength = 5008,			//record different mode's linelength
        .framelength = 1600,           //record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
#endif
#else
	//workaround for recording delay
		.pclk =480000000,
		.linelength = 5008,
		.framelength = 3163,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	//workaround for recording delay end
#endif
	},
	.cap = { //full size 4:3
		.pclk =480000000,
		.linelength = 5008,
		.framelength = 3163,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.normal_video = {//2014x1560 @30fps
#ifdef VIDEO_16_9
        .pclk = 184800000,              //record different mode's pclk
		.linelength = 5008,			//record different mode's linelength
        .framelength = 1224,           //record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1184,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
#else
		.pclk = 242400000,
		.linelength = 5008,
		.framelength = 1600,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1560,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
#endif
	},
	.fastHD_video = {//720p 60fps
		.pclk = 230400000,
		.linelength = 5008,
		.framelength = 766,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
	},
	.slow_motion_video = {//768x432 96fps
		.pclk =233600000,
		.linelength = 5008,
		.framelength = 484,
		.startx = 0,
		.starty =0,
		.grabwindow_width = 768,
		.grabwindow_height = 432,
		.mipi_data_lp2hs_settle_dc =85,
		.max_framerate = 963,
	},
	.margin = 10,
	.min_shutter = 1,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame =1,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.fastHD_video_delay_frame = 3,
	.slow_motion_video_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_2MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34,0x20,0xff},
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_mode = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x34,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
#if 1
#ifdef PREVIEW_16_9
    { 4208, 3120,	  0,	0, 4208, 2368, 2104,  1184, 0000, 0000, 2104,  1184,	  0,	0, 2104, 1184}, // Preview
#else
    { 4208, 3120,	  0,	0, 4208, 3120, 2104,  1560, 0000, 0000, 2104,  1560,	  0,	0, 2104, 1560}, // Preview
#endif
#else
	//workaround for recording delay
 { 4208, 3120,	  0,	0, 4208, 3120, 4208,  3120, 0000, 0000, 4208,  3120,	  0,	0, 4208, 3120}, // capture
	//workaround for recording delay end
#endif
 { 4208, 3120,	  0,	0, 4208, 3120, 4208,  3120, 0000, 0000, 4208,  3120,	  0,	0, 4208, 3120}, // capture
#ifdef VIDEO_16_9
    { 4208, 3120,	  0,	0, 4208, 2368, 2104,  1184, 0000, 0000, 2104,  1184,	  0,	0, 2104, 1184}, // record
#else
    { 4208, 3120,	  0,	0, 4208, 3120, 2104,  1560, 0000, 0000, 2104,  1560,	  0,	0, 2104, 1560}, // record
#endif
 //{ 4208, 3120,	  0,	0, 4208, 3120, 4208,  3120, 0000, 0000, 4208,  3120,	  0,	0, 4208, 3120}, // video
 { 4208, 3120,	  0,    0, 4208, 3120, 2104,  1560, 0000, 0000, 2104,  1560,	  412,	420, 1280, 720}, //hight speed video
 { 4208, 3120,	  0,    0, 4208, 3120, 1052,  780,  0000, 0000, 1052,  780,  142,	174, 768, 432}};// slim video

SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
{/* Preview mode setting */
 {0x02, 0x0a,   0x00,   0x08, 0x40, 0x00,
  0x00, 0x2b, 0x0838, 0x0618, 0x01, 0x35, 0x0200, 0x0001,
  0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000},
  /* Video mode setting */
 {0x02, 0x0a,   0x00,   0x08, 0x40, 0x00,
  0x00, 0x2b, 0x1070, 0x0c30, 0x01, 0x35, 0x0200, 0x0001,
  0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000},
  /* Capture mode setting */
 {0x02, 0x0a,   0x00,   0x08, 0x40, 0x00,
  0x00, 0x2b, 0x1070, 0x0c30, 0x01, 0x35, 0x0200, 0x0001,
  0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000}};

typedef struct
{
    MUINT16 DarkLimit_H;
    MUINT16 DarkLimit_L;
    MUINT16 OverExp_Min_H;
    MUINT16 OverExp_Min_L;
    MUINT16 OverExp_Max_H;
    MUINT16 OverExp_Max_L;
}SENSOR_ATR_INFO, *pSENSOR_ATR_INFO;


SENSOR_ATR_INFO sensorATR_Info[4]=
{/* Strength Range Min */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /* Strength Range Std */
    {0x00, 0x32, 0x00, 0x3c, 0x03, 0xff},
    /* Strength Range Max */
    {0x3f, 0xff, 0x3f, 0xff, 0x3f, 0xff},
    /* Strength Range Custom */
    {0x3F, 0xFF, 0x00, 0x0, 0x3F, 0xFF}};

#define MIPI_MaxGainIndex 159

static kal_uint16 sensorGainMapping[MIPI_MaxGainIndex][2] = {

	{64 , 1  },
	{65 , 8  },
	{66 , 13 },
	{67 , 23 },
	{68 , 27 },
	{69 , 36 },
	{70 , 41 },
	{71 , 49 },
	{72 , 53 },
	{73 , 61 },
	{74 , 69 },
	{75 , 73 },
	{76 , 80 },
	{77 , 88 },
	{78 , 91 },
	{79 , 98 },
	{80 , 101},
	{81 , 108},
	{82 , 111},
	{83 , 117},
	{84 , 120},
	{85 , 126},
	{86 , 132},
	{87 , 135},
	{88 , 140},
	{89 , 143},
	{90 , 148},
	{91 , 151},
	{92 , 156},
	{93 , 161},
	{94 , 163},
	{95 , 168},
	{96 , 170},
	{97 , 175},
	{98 , 177},
	{99 , 181},
	{100, 185},
	{101, 187},
	{102, 191},
	{103, 193},
	{104, 197},
	{105, 199},
	{106, 203},
	{107, 205},
	{108, 207},
	{109, 212},
	{110, 214},
	{111, 217},
	{112, 219},
	{113, 222},
	{114, 224},
	{115, 227},
	{116, 230},
	{117, 232},
	{118, 234},
	{119, 236},
	{120, 239},
	{122, 244},
	{123, 245},
	{124, 248},
	{125, 249},
	{126, 252},
	{127, 253},
	{128, 256},
	{129, 258},
	{130, 260},
	{131, 262},
	{132, 263},
	{133, 266},
	{134, 268},
	{136, 272},
	{138, 274},
	{139, 276},
	{140, 278},
	{141, 280},
	{143, 282},
	{144, 284},
	{145, 286},
	{147, 288},
	{148, 290},
	{149, 292},
	{150, 294},
	{152, 296},
	{153, 298},
	{155, 300},
	{156, 302},
	{157, 304},
	{159, 306},
	{161, 308},
	{162, 310},
	{164, 312},
	{166, 314},
	{167, 316},
	{169, 318},
	{171, 320},
	{172, 322},
	{174, 324},
	{176, 326},
	{178, 328},
	{180, 330},
	{182, 332},
	{184, 334},
	{186, 336},
	{188, 338},
	{191, 340},
	{193, 342},
	{195, 344},
	{197, 346},
	{200, 348},
	{202, 350},
	{205, 352},
	{207, 354},
	{210, 356},
	{212, 358},
	{216, 360},
	{218, 362},
	{221, 364},
	{225, 366},
	{228, 368},
	{231, 370},
	{234, 372},
	{237, 374},
	{241, 376},
	{244, 378},
	{248, 380},
	{252, 382},
	{256, 384},
	{260, 386},
	{264, 388},
	{269, 390},
	{273, 392},
	{278, 394},
	{282, 396},
	{287, 398},
	{292, 400},
	{298, 402},
	{303, 404},
	{309, 406},
	{315, 408},
	{321, 410},
	{328, 412},
	{334, 414},
	{341, 416},
	{349, 418},
	{356, 420},
	{364, 422},
	{372, 424},
	{381, 426},
	{390, 428},
	{399, 430},
	{410, 432},
	{420, 434},
	{431, 436},
	{443, 438},
	{455, 440},
	{468, 442},
	{482, 444},
	{497, 446},
	{512, 448}
};

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
      iReadReg((u16) addr ,(u8*)&get_byte, imgsensor.i2c_write_id);
      return get_byte;
}

#define write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1,  imgsensor.i2c_write_id)

static kal_uint32 imx214_ATR(UINT16 DarkLimit, UINT16 OverExp)
{
    write_cmos_sensor(0x6e50,sensorATR_Info[DarkLimit].DarkLimit_H);
    write_cmos_sensor(0x6e51,sensorATR_Info[DarkLimit].DarkLimit_L);
    write_cmos_sensor(0x9340,sensorATR_Info[OverExp].OverExp_Min_H);
    write_cmos_sensor(0x9341,sensorATR_Info[OverExp].OverExp_Min_L);
    write_cmos_sensor(0x9342,sensorATR_Info[OverExp].OverExp_Max_H);
    write_cmos_sensor(0x9343,sensorATR_Info[OverExp].OverExp_Max_L);
    write_cmos_sensor(0x9706,0x10);
    write_cmos_sensor(0x9707,0x03);
    write_cmos_sensor(0x9708,0x03);
    write_cmos_sensor(0x9e24,0x00);
    write_cmos_sensor(0x9e25,0x8c);
    write_cmos_sensor(0x9e26,0x00);
    write_cmos_sensor(0x9e27,0x94);
    write_cmos_sensor(0x9e28,0x00);
    write_cmos_sensor(0x9e29,0x96);
    LOG_INF("DarkLimit 0x6e50(0x%x), 0x6e51(0x%x)\n",sensorATR_Info[DarkLimit].DarkLimit_H,
                                                     sensorATR_Info[DarkLimit].DarkLimit_L);
    LOG_INF("OverExpMin 0x9340(0x%x), 0x9341(0x%x)\n",sensorATR_Info[OverExp].OverExp_Min_H,
                                                     sensorATR_Info[OverExp].OverExp_Min_L);
    LOG_INF("OverExpMin 0x9342(0x%x), 0x9343(0x%x)\n",sensorATR_Info[OverExp].OverExp_Max_H,
                                                     sensorATR_Info[OverExp].OverExp_Max_L);
    return ERROR_NONE;
}
static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
       write_cmos_sensor(0x0104, 1);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor(0x0104, 0);

}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);

    LOG_INF("realtime_fps = %d,pclk = %d\n",(imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length),imgsensor.pclk); 

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	uint16_t linelength_multiple = 0;
	static uint16_t last_linelength_multiple = 0;

	LOG_INF("+ requested shutter =%d\n", shutter);
	LOG_INF("+ realtime_fps = %d,pclk = %d\n",(imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length),imgsensor.pclk); 
	spin_lock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter){
		LOG_INF("requested shutter less than min shutter %d, use min value\n", imgsensor_info.min_shutter);
		shutter = imgsensor_info.min_shutter;
	}
	if (shutter < imgsensor.min_frame_length - imgsensor_info.margin){
		//shutter = imgsensor.min_frame_length - imgsensor_info.margin;
		LOG_INF("frame length less than min frame length %d, use min value\n", imgsensor.min_frame_length);
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	else if (shutter > imgsensor_info.max_frame_length - imgsensor_info.margin){
		//shutter /= 2;//after test found shutter is 2 times of real needed shutter value
		LOG_INF("long exposure ------\n");
		while(shutter > imgsensor_info.max_frame_length - imgsensor_info.margin){
			shutter /= 2;
			linelength_multiple ++;
		}
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	}
	else{
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	}
	spin_unlock(&imgsensor_drv_lock);

	//long exposure recover to normal exposure
	if(last_linelength_multiple && !linelength_multiple){
		write_cmos_sensor(0x0350, 0x00);
		write_cmos_sensor(0x3028, 0x00);
	}

	if (imgsensor.autoflicker_en &&  !linelength_multiple) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		LOG_INF("realtime_fps = %d,pclk = %d\n",realtime_fps,imgsensor.pclk); 
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		// Extend frame length
		write_cmos_sensor(0x0104, 1);
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0);
	    }
	} else {
		if(linelength_multiple){
			// Extend frame length
			LOG_INF("linelength_multiple is %d\n", linelength_multiple);
			write_cmos_sensor(0x0350,0x01);
			write_cmos_sensor(0x3028, linelength_multiple);
			write_cmos_sensor(0x0104, 1);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0);
		}else{
			write_cmos_sensor(0x0104, 1);
			write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0);
		}
	}

	// Update Shutter
	write_cmos_sensor(0x0104, 1);
	LOG_INF("normal_fps is %d\n", normal_fps);
	if(normal_fps){
		write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
		write_cmos_sensor(0x0203, shutter  & 0xFF);
	}else{
		write_cmos_sensor(0x0202,0xBB);
		write_cmos_sensor(0x0203,0x33);
	}
	write_cmos_sensor(0x0104, 0);

	last_linelength_multiple = linelength_multiple;
	LOG_INF("- shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
	LOG_INF("- realtime_fps = %d,pclk = %d\n",(imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length),imgsensor.pclk); 
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	LOG_INF("####shutter =%d\n", shutter);

	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;

	for (iI = 0; iI < (MIPI_MaxGainIndex-1); iI++) {
			if(gain <= sensorGainMapping[iI][0]){
				break;
			}
		}

	return sensorGainMapping[iI][1];
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 32 * BASEGAIN)
            gain = 32 * BASEGAIN;
    }

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

    write_cmos_sensor(0x0104, 1);
    if(normal_fps){
        /* Global analog Gain for Long expo*/
        write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
        write_cmos_sensor(0x0205, reg_gain & 0xFF);
        /* Global analog Gain for Short expo*/
        write_cmos_sensor(0x0216, (reg_gain>>8)& 0xFF);
        write_cmos_sensor(0x0217, reg_gain & 0xFF);
    }
    write_cmos_sensor(0x0104, 0);

    return gain;
}	/*	set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    kal_uint16 reg_gain;

    LOG_INF("realtime_fps = %d,pclk = %d\n",(imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length),imgsensor.pclk); 
    spin_lock(&imgsensor_drv_lock);
    if (le > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = le + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;

    LOG_INF("realtime_fps = %d,pclk = %d\n",(imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length),imgsensor.pclk); 
    spin_unlock(&imgsensor_drv_lock);
    if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        write_cmos_sensor(0x0104, 1);
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0);
        }
    } else {
        write_cmos_sensor(0x0104, 1);
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0);
    }
    write_cmos_sensor(0x0104, 1);
    /* Long exposure */
    if(normal_fps){
        write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
        write_cmos_sensor(0x0203, le  & 0xFF);
    }else{
        write_cmos_sensor(0x0202,0xBB);
        write_cmos_sensor(0x0203,0x33);
    }
    /* Short exposure */
    write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor(0x0225, se  & 0xFF);
    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    /* Global analog Gain for Long expo*/
    if(normal_fps){
        write_cmos_sensor(0x0204, (reg_gain>>8)& 0xFF);
        write_cmos_sensor(0x0205, reg_gain & 0xFF);
        /* Global analog Gain for Short expo*/
        write_cmos_sensor(0x0216, (reg_gain>>8)& 0xFF);
        write_cmos_sensor(0x0217, reg_gain & 0xFF);
    }
    write_cmos_sensor(0x0104, 0);

}

static void ihdr_write_shutter(kal_uint16 le, kal_uint16 se)
{
	LOG_INF("le:0x%x, se:0x%x\n",le,se);
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    kal_uint16 reg_gain;
    spin_lock(&imgsensor_drv_lock);

    LOG_INF("realtime_fps = %d,pclk = %d\n",(imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length),imgsensor.pclk); 
    if (le > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = le + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("realtime_fps = %d,pclk = %d\n",(imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length),imgsensor.pclk); 
    if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        write_cmos_sensor(0x0104, 1);
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0);
        }
    } else {
        write_cmos_sensor(0x0104, 1);
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        write_cmos_sensor(0x0104, 0);
    }
    write_cmos_sensor(0x0104, 1);
    /* Long exposure */

    if(normal_fps){
        write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
        write_cmos_sensor(0x0203, le  & 0xFF);
    }else{
        write_cmos_sensor(0x0202,0xBB);
        write_cmos_sensor(0x0203,0x33);
    }

    /* Short exposure */
    write_cmos_sensor(0x0224, (se >> 8) & 0xFF);
    write_cmos_sensor(0x0225, se  & 0xFF);

    write_cmos_sensor(0x0104, 0);

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	kal_uint8 itemp;

	itemp=read_cmos_sensor(0x0101);
	itemp &= ~0x03;

	switch(image_mirror)
		{

		   case IMAGE_NORMAL:
		   	     write_cmos_sensor(0x0101, itemp);
			      break;

		   case IMAGE_V_MIRROR:
			     write_cmos_sensor(0x0101, itemp | 0x02);
			     break;

		   case IMAGE_H_MIRROR:
			     write_cmos_sensor(0x0101, itemp | 0x01);
			     break;

		   case IMAGE_HV_MIRROR:
			     write_cmos_sensor(0x0101, itemp | 0x03);
			     break;
		}
}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("E\n");
    write_cmos_sensor(0x0136,0x18);
    write_cmos_sensor(0x0137,0x00);
    write_cmos_sensor(0x0101,0x00);
    write_cmos_sensor(0x0105,0x01);
    write_cmos_sensor(0x0106,0x01);
    write_cmos_sensor(0x4601,0x00);
    write_cmos_sensor(0x4642,0x05);
    write_cmos_sensor(0x6276,0x00);
    write_cmos_sensor(0x900E,0x06);
    write_cmos_sensor(0xA802,0x90);
    write_cmos_sensor(0xA803,0x11);
    write_cmos_sensor(0xA804,0x62);
    write_cmos_sensor(0xA805,0x77);
    write_cmos_sensor(0xA806,0xAE);
    write_cmos_sensor(0xA807,0x34);
    write_cmos_sensor(0xA808,0xAE);
    write_cmos_sensor(0xA809,0x35);
    write_cmos_sensor(0xA80A,0x62);
    write_cmos_sensor(0xA80B,0x83);
    write_cmos_sensor(0xAE33,0x00);
    write_cmos_sensor(0x4174,0x00);
    write_cmos_sensor(0x4175,0x11);
    write_cmos_sensor(0x4612,0x29);
    write_cmos_sensor(0x461B,0x12);
    write_cmos_sensor(0x461F,0x06);
    write_cmos_sensor(0x4635,0x07);
    write_cmos_sensor(0x4637,0x30);
    write_cmos_sensor(0x463F,0x18);
    write_cmos_sensor(0x4641,0x0D);
    write_cmos_sensor(0x465B,0x12);
    write_cmos_sensor(0x465F,0x11);
    write_cmos_sensor(0x4663,0x11);
    write_cmos_sensor(0x4667,0x0F);
    write_cmos_sensor(0x466F,0x0F);
    write_cmos_sensor(0x470E,0x09);
    write_cmos_sensor(0x4909,0xAB);
    write_cmos_sensor(0x490B,0x95);
    write_cmos_sensor(0x4915,0x5D);
    write_cmos_sensor(0x4A5F,0xFF);
    write_cmos_sensor(0x4A61,0xFF);
    write_cmos_sensor(0x4A73,0x62);
    write_cmos_sensor(0x4A85,0x00);
    write_cmos_sensor(0x4A87,0xFF);
    write_cmos_sensor(0x583C,0x04);
    write_cmos_sensor(0x620E,0x04);
    write_cmos_sensor(0x6EB2,0x01);
    write_cmos_sensor(0x6EB3,0x00);
    write_cmos_sensor(0x9300,0x02);

}	/*	sensor_init  */

static void imx214_setting_4_16x9(){
    LOG_INF("xxxxxxxxxxE\n");
	// 30.01fps
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x0220,0x00);
    write_cmos_sensor(0x0221,0x11);
    write_cmos_sensor(0x0222,0x01);
    write_cmos_sensor(0x0340,0x04);
    write_cmos_sensor(0x0341,0xC8);
    write_cmos_sensor(0x0342,0x13);
    write_cmos_sensor(0x0343,0x90);
    write_cmos_sensor(0x0344,0x00);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x01);
    write_cmos_sensor(0x0347,0x78);
    write_cmos_sensor(0x0348,0x10);
    write_cmos_sensor(0x0349,0x6F);
    write_cmos_sensor(0x034A,0x0A);
    write_cmos_sensor(0x034B,0xB7);
    write_cmos_sensor(0x0381,0x01);
    write_cmos_sensor(0x0383,0x01);
    write_cmos_sensor(0x0385,0x01);
    write_cmos_sensor(0x0387,0x01);
    write_cmos_sensor(0x0900,0x01);
    write_cmos_sensor(0x0901,0x22);
    write_cmos_sensor(0x0902,0x02);
    write_cmos_sensor(0x3000,0x35);
    write_cmos_sensor(0x3054,0x01);
    write_cmos_sensor(0x305C,0x11);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x034C,0x08);
    write_cmos_sensor(0x034D,0x38);
    write_cmos_sensor(0x034E,0x04);
    write_cmos_sensor(0x034F,0xA0);
    write_cmos_sensor(0x0401,0x00);
    write_cmos_sensor(0x0404,0x00);
    write_cmos_sensor(0x0405,0x10);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x00);
    write_cmos_sensor(0x040C,0x08);
    write_cmos_sensor(0x040D,0x38);
    write_cmos_sensor(0x040E,0x04);
    write_cmos_sensor(0x040F,0xA0);
    write_cmos_sensor(0x0301,0x05);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x04);
    write_cmos_sensor(0x0306,0x00);
    write_cmos_sensor(0x0307,0x4D);
    write_cmos_sensor(0x0309,0x0A);
    write_cmos_sensor(0x030B,0x01);
    write_cmos_sensor(0x0310,0x00);
    write_cmos_sensor(0x0820,0x07);
    write_cmos_sensor(0x0821,0x38);
    write_cmos_sensor(0x0822,0x00);
    write_cmos_sensor(0x0823,0x00);
    write_cmos_sensor(0x3A03,0x06);
    write_cmos_sensor(0x3A04,0x68);
    write_cmos_sensor(0x3A05,0x01);
    write_cmos_sensor(0x0B06,0x01);
    write_cmos_sensor(0x30A2,0x00);
    write_cmos_sensor(0x30B4,0x00);
    write_cmos_sensor(0x3A02,0xFF);
    write_cmos_sensor(0x3013,0x00);
    write_cmos_sensor(0x0202,0x04);
    write_cmos_sensor(0x0203,0xBE);
    write_cmos_sensor(0x0224,0x01);
    write_cmos_sensor(0x0225,0xF4);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x00);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x0216,0x00);
    write_cmos_sensor(0x0217,0x00);
    write_cmos_sensor(0x4170,0x00);
    write_cmos_sensor(0x4171,0x10);
    write_cmos_sensor(0x4176,0x00);
    write_cmos_sensor(0x4177,0x3C);
    write_cmos_sensor(0xAE20,0x04);
    write_cmos_sensor(0xAE21,0x5C);
    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);
}


static void imx213_setting_4_4x3(void)
{
    LOG_INF("xxxxxxxxxxE\n");
	//Preview 2104*1560 30fps 24M MCLK 4lane 608Mbps/lane
	// preview 30.01fps
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x0220,0x00);
    write_cmos_sensor(0x0221,0x11);
    write_cmos_sensor(0x0222,0x01);
    write_cmos_sensor(0x0340,0x06);
    write_cmos_sensor(0x0341,0x40);
    write_cmos_sensor(0x0342,0x13);
    write_cmos_sensor(0x0343,0x90);
    write_cmos_sensor(0x0344,0x00);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x00);
    write_cmos_sensor(0x0347,0x00);
    write_cmos_sensor(0x0348,0x10);
    write_cmos_sensor(0x0349,0x6F);
    write_cmos_sensor(0x034A,0x0C);
    write_cmos_sensor(0x034B,0x2F);
    write_cmos_sensor(0x0381,0x01);
    write_cmos_sensor(0x0383,0x01);
    write_cmos_sensor(0x0385,0x01);
    write_cmos_sensor(0x0387,0x01);
    write_cmos_sensor(0x0900,0x01);
    write_cmos_sensor(0x0901,0x22);
    write_cmos_sensor(0x0902,0x02);
    write_cmos_sensor(0x3000,0x35);
    write_cmos_sensor(0x3054,0x01);
    write_cmos_sensor(0x305C,0x11);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x034C,0x08);
    write_cmos_sensor(0x034D,0x38);
    write_cmos_sensor(0x034E,0x06);
    write_cmos_sensor(0x034F,0x18);
    write_cmos_sensor(0x0401,0x00);
    write_cmos_sensor(0x0404,0x00);
    write_cmos_sensor(0x0405,0x10);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x00);
    write_cmos_sensor(0x040C,0x08);
    write_cmos_sensor(0x040D,0x38);
    write_cmos_sensor(0x040E,0x06);
    write_cmos_sensor(0x040F,0x18);
    write_cmos_sensor(0x0301,0x05);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x04);
    write_cmos_sensor(0x0306,0x00);
    write_cmos_sensor(0x0307,0x65);
    write_cmos_sensor(0x0309,0x0A);
    write_cmos_sensor(0x030B,0x01);
    write_cmos_sensor(0x0310,0x00);
    write_cmos_sensor(0x0820,0x09);
    write_cmos_sensor(0x0821,0x78);
    write_cmos_sensor(0x0822,0x00);
    write_cmos_sensor(0x0823,0x00);
    write_cmos_sensor(0x3A03,0x06);
    write_cmos_sensor(0x3A04,0x68);
    write_cmos_sensor(0x3A05,0x01);
    write_cmos_sensor(0x0B06,0x01);
    write_cmos_sensor(0x30A2,0x00);
    write_cmos_sensor(0x30B4,0x00);
    write_cmos_sensor(0x3A02,0xFF);
    write_cmos_sensor(0x3013,0x00);

    if(normal_fps){
        write_cmos_sensor(0x0202,0x06);
        write_cmos_sensor(0x0203,0x36);
    }else{
        write_cmos_sensor(0x0350,0x01);
        write_cmos_sensor(0x3028,0x01);
        write_cmos_sensor(0x0202,0xBB);
        write_cmos_sensor(0x0203,0x33);
    }

    write_cmos_sensor(0x0224,0x01);
    write_cmos_sensor(0x0225,0xF4);

    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x00);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x0216,0x00);
    write_cmos_sensor(0x0217,0x00);

    write_cmos_sensor(0x4170,0x00);
    write_cmos_sensor(0x4171,0x10);
    write_cmos_sensor(0x4176,0x00);
    write_cmos_sensor(0x4177,0x3C);
    write_cmos_sensor(0xAE20,0x04);
    write_cmos_sensor(0xAE21,0x5C);
    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);
}

static void imx214_setting_full_size_4x3(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n",currefps);
    // full size 29.76fps
    // capture setting 4208*3120  480MCLK 1.2Gp/lane
    // full size 30.33ps
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x0220,0x00);
    write_cmos_sensor(0x0221,0x11);
    write_cmos_sensor(0x0222,0x01);
    write_cmos_sensor(0x0340,0x0C);
    write_cmos_sensor(0x0341,0x58);
    write_cmos_sensor(0x0342,0x13);
    write_cmos_sensor(0x0343,0x90);
    write_cmos_sensor(0x0344,0x00);
    write_cmos_sensor(0x0345,0x00);
    write_cmos_sensor(0x0346,0x00);
    write_cmos_sensor(0x0347,0x00);
    write_cmos_sensor(0x0348,0x10);
    write_cmos_sensor(0x0349,0x6F);
    write_cmos_sensor(0x034A,0x0C);
    write_cmos_sensor(0x034B,0x2F);
    write_cmos_sensor(0x0381,0x01);
    write_cmos_sensor(0x0383,0x01);
    write_cmos_sensor(0x0385,0x01);
    write_cmos_sensor(0x0387,0x01);
    write_cmos_sensor(0x0900,0x00);
    write_cmos_sensor(0x0901,0x00);
    write_cmos_sensor(0x0902,0x00);
    write_cmos_sensor(0x3000,0x35);
    write_cmos_sensor(0x3054,0x01);
    write_cmos_sensor(0x305C,0x11);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x034C,0x10);
    write_cmos_sensor(0x034D,0x70);
    write_cmos_sensor(0x034E,0x0C);
    write_cmos_sensor(0x034F,0x30);
    write_cmos_sensor(0x0401,0x00);
    write_cmos_sensor(0x0404,0x00);
    write_cmos_sensor(0x0405,0x10);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x00);
    write_cmos_sensor(0x040C,0x10);
    write_cmos_sensor(0x040D,0x70);
    write_cmos_sensor(0x040E,0x0C);
    write_cmos_sensor(0x040F,0x30);
    write_cmos_sensor(0x0301,0x05);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x03);
    write_cmos_sensor(0x0306,0x00);
    write_cmos_sensor(0x0307,0x96);
    write_cmos_sensor(0x0309,0x0A);
    write_cmos_sensor(0x030B,0x01);
    write_cmos_sensor(0x0310,0x00);
    write_cmos_sensor(0x0820,0x12);
    write_cmos_sensor(0x0821,0xC0);
    write_cmos_sensor(0x0822,0x00);
    write_cmos_sensor(0x0823,0x00);
    write_cmos_sensor(0x3A03,0x08);
    write_cmos_sensor(0x3A04,0xC0);
    write_cmos_sensor(0x3A05,0x02);
    write_cmos_sensor(0x0B06,0x01);
    write_cmos_sensor(0x30A2,0x00);
    write_cmos_sensor(0x30B4,0x00);
    write_cmos_sensor(0x3A02,0xFF);
    write_cmos_sensor(0x3013,0x00);
    if(normal_fps){
        write_cmos_sensor(0x0202,0x0C);
        write_cmos_sensor(0x0203,0x4E);
    }else{
        write_cmos_sensor(0x0350,0x01);
        write_cmos_sensor(0x3028,0x01);
        write_cmos_sensor(0x0202,0xBB);
        write_cmos_sensor(0x0203,0x33);
    }

    write_cmos_sensor(0x0224,0x01);
    write_cmos_sensor(0x0225,0xF4);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x00);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x0216,0x00);
    write_cmos_sensor(0x0217,0x00);
    write_cmos_sensor(0x4170,0x00);
    write_cmos_sensor(0x4171,0x10);
    write_cmos_sensor(0x4176,0x00);
    write_cmos_sensor(0x4177,0x3C);
    write_cmos_sensor(0xAE20,0x04);
    write_cmos_sensor(0xAE21,0x5C);
    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);

}

static void fastHD_video_setting()
{
    LOG_INF("E\n");
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x0220,0x00);
    write_cmos_sensor(0x0221,0x11);
    write_cmos_sensor(0x0222,0x01);
    write_cmos_sensor(0x0340,0x02);
    write_cmos_sensor(0x0341,0xFE);
    write_cmos_sensor(0x0342,0x13);
    write_cmos_sensor(0x0343,0x90);
    write_cmos_sensor(0x0344,0x03);
    write_cmos_sensor(0x0345,0x38);
    write_cmos_sensor(0x0346,0x03);
    write_cmos_sensor(0x0347,0x48);
    write_cmos_sensor(0x0348,0x0D);
    write_cmos_sensor(0x0349,0x37);
    write_cmos_sensor(0x034A,0x08);
    write_cmos_sensor(0x034B,0xE7);
    write_cmos_sensor(0x0381,0x01);
    write_cmos_sensor(0x0383,0x01);
    write_cmos_sensor(0x0385,0x01);
    write_cmos_sensor(0x0387,0x01);
    write_cmos_sensor(0x0900,0x01);
    write_cmos_sensor(0x0901,0x22);
    write_cmos_sensor(0x0902,0x02);
    write_cmos_sensor(0x3000,0x35);
    write_cmos_sensor(0x3054,0x01);
    write_cmos_sensor(0x305C,0x11);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x034C,0x05);
    write_cmos_sensor(0x034D,0x00);
    write_cmos_sensor(0x034E,0x02);
    write_cmos_sensor(0x034F,0xD0);
    write_cmos_sensor(0x0401,0x00);
    write_cmos_sensor(0x0404,0x00);
    write_cmos_sensor(0x0405,0x10);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x00);
    write_cmos_sensor(0x040C,0x05);
    write_cmos_sensor(0x040D,0x00);
    write_cmos_sensor(0x040E,0x02);
    write_cmos_sensor(0x040F,0xD0);
    write_cmos_sensor(0x0301,0x05);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x03);
    write_cmos_sensor(0x0306,0x00);
    write_cmos_sensor(0x0307,0x48);
    write_cmos_sensor(0x0309,0x0A);
    write_cmos_sensor(0x030B,0x01);
    write_cmos_sensor(0x0310,0x00);
    write_cmos_sensor(0x0820,0x09);
    write_cmos_sensor(0x0821,0x00);
    write_cmos_sensor(0x0822,0x00);
    write_cmos_sensor(0x0823,0x00);
    write_cmos_sensor(0x3A03,0x05);
    write_cmos_sensor(0x3A04,0x20);
    write_cmos_sensor(0x3A05,0x02);
    write_cmos_sensor(0x0B06,0x01);
    write_cmos_sensor(0x30A2,0x00);
    write_cmos_sensor(0x30B4,0x00);
    write_cmos_sensor(0x3A02,0xFF);
    write_cmos_sensor(0x3013,0x00);


    if(normal_fps){
        write_cmos_sensor(0x0202,0x02);
        write_cmos_sensor(0x0203,0xF4);
    }else{
        write_cmos_sensor(0x0350,0x01);
        write_cmos_sensor(0x3028,0x01);
        write_cmos_sensor(0x0202,0xBB);
        write_cmos_sensor(0x0203,0x33);
    }

    write_cmos_sensor(0x0224,0x01);
    write_cmos_sensor(0x0225,0xF4);
        write_cmos_sensor(0x0204,0x00);
        write_cmos_sensor(0x0205,0x00);
        write_cmos_sensor(0x020E,0x01);
        write_cmos_sensor(0x020F,0x00);
        write_cmos_sensor(0x0210,0x01);
        write_cmos_sensor(0x0211,0x00);
        write_cmos_sensor(0x0212,0x01);
        write_cmos_sensor(0x0213,0x00);
        write_cmos_sensor(0x0214,0x01);
        write_cmos_sensor(0x0215,0x00);
        write_cmos_sensor(0x0216,0x00);
        write_cmos_sensor(0x0217,0x00);
    write_cmos_sensor(0x4170,0x00);
    write_cmos_sensor(0x4171,0x10);
    write_cmos_sensor(0x4176,0x00);
    write_cmos_sensor(0x4177,0x3C);
    write_cmos_sensor(0xAE20,0x04);
    write_cmos_sensor(0xAE21,0x5C);
    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);
}

static void slow_motion_120fps_video_setting(){
    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x0220,0x00);
    write_cmos_sensor(0x0221,0x11);
    write_cmos_sensor(0x0222,0x01);
    write_cmos_sensor(0x0340,0x01);
    write_cmos_sensor(0x0341,0xDE);
    write_cmos_sensor(0x0342,0x13);
    write_cmos_sensor(0x0343,0x90);
    write_cmos_sensor(0x0344,0x02);
    write_cmos_sensor(0x0345,0x38);
    write_cmos_sensor(0x0346,0x02);
    write_cmos_sensor(0x0347,0xB8);
    write_cmos_sensor(0x0348,0x0E);
    write_cmos_sensor(0x0349,0x37);
    write_cmos_sensor(0x034A,0x09);
    write_cmos_sensor(0x034B,0x77);
    write_cmos_sensor(0x0381,0x01);
    write_cmos_sensor(0x0383,0x01);
    write_cmos_sensor(0x0385,0x01);
    write_cmos_sensor(0x0387,0x01);
    write_cmos_sensor(0x0900,0x01);
    write_cmos_sensor(0x0901,0x44);
    write_cmos_sensor(0x0902,0x02);
    write_cmos_sensor(0x3000,0x35);
    write_cmos_sensor(0x3054,0x01);
    write_cmos_sensor(0x305C,0x11);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x034C,0x03);
    write_cmos_sensor(0x034D,0x00);
    write_cmos_sensor(0x034E,0x01);
    write_cmos_sensor(0x034F,0xB0);
    write_cmos_sensor(0x0401,0x00);
    write_cmos_sensor(0x0404,0x00);
    write_cmos_sensor(0x0405,0x10);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x00);
    write_cmos_sensor(0x040C,0x03);
    write_cmos_sensor(0x040D,0x00);
    write_cmos_sensor(0x040E,0x01);
    write_cmos_sensor(0x040F,0xB0);
    write_cmos_sensor(0x0301,0x05);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x03);
    write_cmos_sensor(0x0306,0x00);
    write_cmos_sensor(0x0307,0x5A);
    write_cmos_sensor(0x0309,0x0A);
    write_cmos_sensor(0x030B,0x01);
    write_cmos_sensor(0x0310,0x00);
    write_cmos_sensor(0x0820,0x0B);
    write_cmos_sensor(0x0821,0x40);
    write_cmos_sensor(0x0822,0x00);
    write_cmos_sensor(0x0823,0x00);
    write_cmos_sensor(0x3A03,0x02);
    write_cmos_sensor(0x3A04,0xF8);
    write_cmos_sensor(0x3A05,0x01);
    write_cmos_sensor(0x0B06,0x01);
    write_cmos_sensor(0x30A2,0x00);
    write_cmos_sensor(0x30B4,0x00);
    write_cmos_sensor(0x3A02,0xFF);
    write_cmos_sensor(0x3013,0x00);

    if(normal_fps){
        write_cmos_sensor(0x0202,0x01);
        write_cmos_sensor(0x0203,0xD4);
    }else{
        write_cmos_sensor(0x0350,0x01);
        write_cmos_sensor(0x3028,0x01);
        write_cmos_sensor(0x0202,0xBB);
        write_cmos_sensor(0x0203,0x33);
    }
    write_cmos_sensor(0x0224,0x01);
    write_cmos_sensor(0x0225,0xF4);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x00);
    write_cmos_sensor(0x020E,0x01);
    write_cmos_sensor(0x020F,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x0216,0x00);
    write_cmos_sensor(0x0217,0x00);
    write_cmos_sensor(0x4170,0x00);
    write_cmos_sensor(0x4171,0x10);
    write_cmos_sensor(0x4176,0x00);
    write_cmos_sensor(0x4177,0x3C);
    write_cmos_sensor(0xAE20,0x04);
    write_cmos_sensor(0xAE21,0x5C);

    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);
}

static void slow_motion_video_setting()
{
    LOG_INF("E\n"); //96 fps

    write_cmos_sensor(0x0100,0x00);
    write_cmos_sensor(0x0114,0x03);
    write_cmos_sensor(0x0220,0x00);
    write_cmos_sensor(0x0221,0x11);
    write_cmos_sensor(0x0222,0x01);
    write_cmos_sensor(0x0340,0x01);
    write_cmos_sensor(0x0341,0xE4);
    write_cmos_sensor(0x0342,0x13);
    write_cmos_sensor(0x0343,0x90);
    write_cmos_sensor(0x0344,0x02);
    write_cmos_sensor(0x0345,0x38);
    write_cmos_sensor(0x0346,0x02);
    write_cmos_sensor(0x0347,0xB8);
    write_cmos_sensor(0x0348,0x0E);
    write_cmos_sensor(0x0349,0x37);
    write_cmos_sensor(0x034A,0x09);
    write_cmos_sensor(0x034B,0x77);
    write_cmos_sensor(0x0381,0x01);
    write_cmos_sensor(0x0383,0x01);
    write_cmos_sensor(0x0385,0x01);
    write_cmos_sensor(0x0387,0x01);
    write_cmos_sensor(0x0900,0x01);
    write_cmos_sensor(0x0901,0x44);
    write_cmos_sensor(0x0902,0x02);
    write_cmos_sensor(0x3000,0x35);
    write_cmos_sensor(0x3054,0x01);
    write_cmos_sensor(0x305C,0x11);
    write_cmos_sensor(0x0112,0x0A);
    write_cmos_sensor(0x0113,0x0A);
    write_cmos_sensor(0x034C,0x03);
    write_cmos_sensor(0x034D,0x00);
    write_cmos_sensor(0x034E,0x01);
    write_cmos_sensor(0x034F,0xB0);
    write_cmos_sensor(0x0401,0x00);
    write_cmos_sensor(0x0404,0x00);
    write_cmos_sensor(0x0405,0x10);
    write_cmos_sensor(0x0408,0x00);
    write_cmos_sensor(0x0409,0x00);
    write_cmos_sensor(0x040A,0x00);
    write_cmos_sensor(0x040B,0x00);
    write_cmos_sensor(0x040C,0x03);
    write_cmos_sensor(0x040D,0x00);
    write_cmos_sensor(0x040E,0x01);
    write_cmos_sensor(0x040F,0xB0);
    write_cmos_sensor(0x0301,0x05);
    write_cmos_sensor(0x0303,0x02);
    write_cmos_sensor(0x0305,0x03);
    write_cmos_sensor(0x0306,0x00);
    write_cmos_sensor(0x0307,0x49);
    write_cmos_sensor(0x0309,0x0A);
    write_cmos_sensor(0x030B,0x01);
    write_cmos_sensor(0x0310,0x00);
    write_cmos_sensor(0x0820,0x09);
    write_cmos_sensor(0x0821,0x20);
    write_cmos_sensor(0x0822,0x00);
    write_cmos_sensor(0x0823,0x00);
    write_cmos_sensor(0x3A03,0x02);
    write_cmos_sensor(0x3A04,0xF8);
    write_cmos_sensor(0x3A05,0x00);
    write_cmos_sensor(0x0B06,0x01);
    write_cmos_sensor(0x30A2,0x00);
    write_cmos_sensor(0x30B4,0x00);
    write_cmos_sensor(0x3A02,0xFF);
    write_cmos_sensor(0x3013,0x00);

    if(normal_fps){
        write_cmos_sensor(0x0202,0x01);
        write_cmos_sensor(0x0203,0xDA);
    }else{
        write_cmos_sensor(0x0350,0x01);
        write_cmos_sensor(0x3028,0x01);
        write_cmos_sensor(0x0202,0xbb);
        write_cmos_sensor(0x0203,0x33);
    }
    write_cmos_sensor(0x0224,0x01);
    write_cmos_sensor(0x0225,0xf4);
    write_cmos_sensor(0x0204,0x00);
    write_cmos_sensor(0x0205,0x00);
    write_cmos_sensor(0x020e,0x01);
    write_cmos_sensor(0x020f,0x00);
    write_cmos_sensor(0x0210,0x01);
    write_cmos_sensor(0x0211,0x00);
    write_cmos_sensor(0x0212,0x01);
    write_cmos_sensor(0x0213,0x00);
    write_cmos_sensor(0x0214,0x01);
    write_cmos_sensor(0x0215,0x00);
    write_cmos_sensor(0x0216,0x00);
    write_cmos_sensor(0x0217,0x00);
    write_cmos_sensor(0x4170,0x00);
    write_cmos_sensor(0x4171,0x10);
    write_cmos_sensor(0x4176,0x00);
    write_cmos_sensor(0x4177,0x3C);
    write_cmos_sensor(0xAE20,0x04);
    write_cmos_sensor(0xAE21,0x5C);
    write_cmos_sensor(0x0138,0x01);
    write_cmos_sensor(0x0100,0x01);
}



static int read_cmos_sensor_otp(kal_uint32 addr, u8* get_byte )
{
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    return iReadRegI2C(pu_send_cmd, 2, get_byte, 1, imgsensor.i2c_write_id);
}

static int write_cmos_sensor_otp(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    return iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static int read_imx214_otp_flag = 0;
char imx214_otp_data[64];

static int read_imx214_otp(void){
    LOG_INF("==========read_imx214_otp_flag =%d=======\n",read_imx214_otp_flag);
    if(read_imx214_otp_flag !=0){
        LOG_INF("==========imx214 otp readed=======\n");
        return 0;
    }
    int page = 0;
    int valid_layer = -1;
    int valid_layer_htc = -1;
    int i = 0;
    u8 read_data = 0;
    int ret = 0;

    for(page = 2; page >= 0; page--)   //page 0~2 for module info
    {
        write_cmos_sensor_otp(0x0A02, page);
        write_cmos_sensor_otp(0x0A00, 0x01);

        mdelay(10);
        if(valid_layer == -1) {
            for(i = 0; i < 16; i++) {
                ret = read_cmos_sensor_otp(0x0A04 + i, &read_data);
                if (ret < 0) {
                    LOG_INF(" i2c_read 0x%x failed\n", (0x0A04 + i));
                } else {
                    LOG_INF("page = %d, addr = 0x%x data = 0x%x\n",page,(0x0A04 + i), read_data);
                    if(read_data){
                        valid_layer = page;
                    }
                    if(valid_layer !=-1){
                        imx214_otp_data[i]=read_data;
                    }
                    read_data = 0;
                }
            }
        }
        if(valid_layer_htc == -1){
            for(i = 16; i < 32; i++) {
                ret = read_cmos_sensor_otp(0x0A04 + i, &read_data);
                if (ret < 0) {
                    LOG_INF(" i2c_read 0x%x failed\n", (0x0A04 + i));
                } else {
                    LOG_INF("page = %d, addr = 0x%x data = 0x%x\n",page,(0x0A04 + i), read_data);
                    if(read_data){
                        valid_layer_htc = page;
                    }
                    if(valid_layer_htc !=-1) {
                        imx214_otp_data[i]=read_data;
                    }
                    read_data = 0;
                }
            }
        }

        if(valid_layer != -1 && valid_layer_htc != -1) {
            LOG_INF("valid_layer of module info:%d, htc_info in page:%d\n", valid_layer, valid_layer_htc);
            break;
        }
    }
#if 1
    LOG_INF("OTP Module vendor = 0x%x\n",               imx214_otp_data[0x10]);
    LOG_INF("OTP LENS = 0x%x\n",                        imx214_otp_data[0x11]);
    LOG_INF("OTP Sensor Version = 0x%x\n",              imx214_otp_data[0x12]);
    LOG_INF("OTP Driver IC Vendor & Version = 0x%x\n",  imx214_otp_data[0x13]);
    LOG_INF("OTP Actuator vender ID & Version = 0x%x\n",imx214_otp_data[0x14]);

    LOG_INF("OTP AF Infinity position code (MSByte) = 0x%x\n", imx214_otp_data[0x19]);
    LOG_INF("OTP AF Infinity position code (LSByte) = 0x%x\n", imx214_otp_data[0x1A]);
    LOG_INF("OTP AF Macro position code (MSByte) = 0x%x\n",    imx214_otp_data[0x1B]);
    LOG_INF("OTP AF Macro position code (LSByte) = 0x%x\n",    imx214_otp_data[0x1C]);
#endif

    if(valid_layer!=-1) {
        spin_lock(&imgsensor_drv_lock);
        read_imx214_otp_flag = 1;
        spin_unlock(&imgsensor_drv_lock);
    }
    LOG_INF("==========exit imx214 read_otp=======\n");
    return ret;
}

/**********************OIS OTP data related***************************/
static int read_imx214_ois_otp_flag = 0;
static char imx214_ois_otp_data[OIS_OTP_SIZE];

static void mem_print(char*data_p, int size)
{
	int i = 0;
	printk("\n--ois_otp dump--");
	while(size--){
		if(!(i & 0x0f))
			printk("\nois_otp 0x%03x: ", i);
		printk("%02x ", data_p[i]);
		i++;
	}
	printk("\n----------------\n");
}

/**** return read size if success, 0 if fail ****/
static int ois_otp_page_read(int page_start, int page_end, char*data_p, int size){
	int page = 0;
	int valid_layer = -1;
	int i = 0;
	char read_data = 0;
	int ret = 0;

	for(page = page_start; page >= page_end; page--)
	{
		LOG_INF("ois_otp: trying page %d \n", page);
		write_cmos_sensor_otp(0x0A02, page);
		write_cmos_sensor_otp(0x0A00, 0x01);

		mdelay(10);
		for(i = 0; i < size; i++)
		{
			ret = read_cmos_sensor_otp(0x0A04 + i, &read_data);

			if (ret < 0){
				LOG_INF("ois_otp: i2c_read 0x%x failed\n", (0x0A04 + i));
			} else {
				//LOG_INF("ois_otp: page = %d, addr = 0x%04x data = 0x%02x\n",page,(0x0A04 + i), read_data);
				if(read_data){
					valid_layer = page;
				}

				if(valid_layer !=-1){
					data_p[i]=read_data;
				}
				read_data = 0;
			}
		}

		if(valid_layer != -1)
		{
			LOG_INF("ois_otp: valid_layer:%d \n", valid_layer);
#ifdef OIS_OTP_DUMP
			mem_print(data_p, size);
#endif
			return size;
		}
	}
	LOG_INF("ois_otp: no valid_layer\n");
	return 0;
}

static int read_imx214_ois_otp(void){
	int ret = 0;

	LOG_INF("ois_otp:flag %d\n",read_imx214_ois_otp_flag);
	if(read_imx214_ois_otp_flag !=0){
		LOG_INF("ois_otp: ready!\n");
		return 0;
	}

	LOG_INF("ois_otp: read NVR0\n");
	ret += ois_otp_page_read(5, 3, imx214_ois_otp_data + NVR0_OFFSET, NVR0_SIZE);
	LOG_INF("ois_otp: read NVR1_HALL_CAL_GYRO_GAIN_1\n");
	ret += ois_otp_page_read(8, 6, imx214_ois_otp_data + NVR1_HALL_CAL_GYRO_GAIN_1_OFFSET, NVR1_HALL_CAL_GYRO_GAIN_1_SIZE);
	LOG_INF("ois_otp: read NVR1_HALL_CAL_GYRO_GAIN_2\n");
	ret += ois_otp_page_read(11, 9, imx214_ois_otp_data + NVR1_HALL_CAL_GYRO_GAIN_2_OFFSET, NVR1_HALL_CAL_GYRO_GAIN_2_SIZE);
	LOG_INF("ois_otp: read NVR1_PRI_AFMIDPOS_COO_ETC_DATA_OFFSET\n");
	ret += ois_otp_page_read(14, 12, imx214_ois_otp_data + NVR1_PRI_AFMIDPOS_COO_ETC_DATA_OFFSET, NVR1_PRI_AFMIDPOS_COO_ETC_DATA_SIZE);


	if(ret == OIS_OTP_SIZE) {
		read_imx214_ois_otp_flag = 1;
	}
	LOG_INF("ois_otp:read end\n");
	return ret;
}
/**********************OIS OTP data related END***************************/

static ssize_t sensor_otp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

    LOG_INF("OTP AF Infinity position code (MSByte) = 0x%x\n", imx214_otp_data[0x19]);
    LOG_INF("OTP AF Infinity position code (LSByte) = 0x%x\n", imx214_otp_data[0x1A]);
    LOG_INF("OTP AF Macro position code (MSByte) = 0x%x\n",    imx214_otp_data[0x1B]);
    LOG_INF("OTP AF Macro position code (LSByte) = 0x%x\n",    imx214_otp_data[0x1C]);
    unsigned short inf_val = (imx214_otp_data[0x19]<<8 | imx214_otp_data[0x1A]);
    unsigned short mac_val = (imx214_otp_data[0x1B]<<8 | imx214_otp_data[0x1C]);
	sprintf(buf, "lenc:%u:%u\n",inf_val,mac_val);
	ret = strlen(buf) + 1;
	return ret;
}

static int imx214_sysfs_init(void);

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#define IMX214_DRIVER_IC_VENDER_VERSION 0x13
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x0016) << 8) | read_cmos_sensor(0x0017));
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				iReadData(0x00,452,OTPData);
				imx214_sysfs_init();
				read_imx214_otp();
				LOG_INF("driver ic vendor version: 0x%x\n",imx214_otp_data[IMX214_DRIVER_IC_VENDER_VERSION]);
				if((imx214_otp_data[IMX214_DRIVER_IC_VENDER_VERSION] == 0x53) || (imx214_otp_data[IMX214_DRIVER_IC_VENDER_VERSION] == 0x55)){
					read_imx214_ois_otp();
					len_sysboot_check(imx214_ois_otp_data);
				}
				else{
					LOG_INF("No ois_otp data\n");
					len_sysboot_check(NULL);
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	LOG_INF("PLATFORM:MT6595,MIPI 2LANE\n");
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");

	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor(0x0016) << 8) | read_cmos_sensor(0x0017));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id:0x%x id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
#if 1
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
#ifdef PREVIEW_16_9
    imx214_setting_4_16x9();
#else
    imx213_setting_4_4x3();
#endif
#else
	//workaround for recording delay
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	imx214_setting_full_size_4x3(imgsensor.current_fps);
	//workaround for recording delay end
#endif
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	imx214_setting_full_size_4x3(imgsensor.current_fps);
	return ERROR_NONE;
}	/* capture() */



static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
#ifdef VIDEO_16_9
    imx214_setting_4_16x9();
#else
    imx213_setting_4_4x3();
#endif
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 fastHD_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.fastHD_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.fastHD_video.linelength;
	imgsensor.frame_length = imgsensor_info.fastHD_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.fastHD_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	fastHD_video_setting();
	return ERROR_NONE;
}	/*	fastHD_video   */


static kal_uint32 slow_motion_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slow_motion_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slow_motion_video.linelength;
	imgsensor.frame_length = imgsensor_info.slow_motion_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slow_motion_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slow_motion_video_setting();
    //slow_motion_120fps_video_setting();

	return ERROR_NONE;
}	/*	slow_motion_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.fastHD_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.fastHD_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slow_motion_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slow_motion_video.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	//sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	//sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.fastHD_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slow_motion_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.fastHD_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.fastHD_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.fastHD_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slow_motion_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slow_motion_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slow_motion_video.mipi_data_lp2hs_settle_dc;

			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("XXXXXXXXXXXXXXX scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			fastHD_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slow_motion_video(image_window, sensor_config_data);
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("set_video_mode framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            /* fix preview max fps will limit max fps
			frame_length = imgsensor_info.fastHD_video.pclk / framerate * 10 / imgsensor_info.fastHD_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.fastHD_video.framelength) ? (frame_length - imgsensor_info.fastHD_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.fastHD_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            */
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
            /*
			frame_length = imgsensor_info.slow_motion_video.pclk / framerate * 10 / imgsensor_info.slow_motion_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slow_motion_video.framelength) ? (frame_length - imgsensor_info.slow_motion_video.framelength): 0;
			imgsensor.frame_length = imgsensor_info.slow_motion_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            */
			//set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.fastHD_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slow_motion_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0601,0x0002);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0601,0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 imx214_awb_gain(SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
    LOG_INF("imx214_awb_gain\n");
    UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

    grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
    rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
    bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
    gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

    LOG_INF("[imx214_awb_gain] ABS_GAIN_GR:%d, grgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GR, grgain_32);
    LOG_INF("[imx214_awb_gain] ABS_GAIN_R:%d, rgain_32:%d\n", pSetSensorAWB->ABS_GAIN_R, rgain_32);
    LOG_INF("[imx214_awb_gain] ABS_GAIN_B:%d, bgain_32:%d\n", pSetSensorAWB->ABS_GAIN_B, bgain_32);
    LOG_INF("[imx214_awb_gain] ABS_GAIN_GB:%d, gbgain_32:%d\n", pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

    write_cmos_sensor(0x0b8e, (grgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b8f, grgain_32 & 0xFF);
    write_cmos_sensor(0x0b90, (rgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b91, rgain_32 & 0xFF);
    write_cmos_sensor(0x0b92, (bgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b93, bgain_32 & 0xFF);
    write_cmos_sensor(0x0b94, (gbgain_32 >> 8) & 0xFF);
    write_cmos_sensor(0x0b95, gbgain_32 & 0xFF);
    return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    SENSOR_VC_INFO_STRUCT *pvcinfo;
    SET_SENSOR_AWB_GAIN *pSetSensorAWB=(SET_SENSOR_AWB_GAIN *)feature_para;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("frame rate current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.ihdr_mode = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
        case SENSOR_FEATURE_GET_VC_INFO:
            LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
            pvcinfo = (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            default:
                memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],sizeof(SENSOR_VC_INFO_STRUCT));
                break;
            }
            break;
        case SENSOR_FEATURE_SET_AWB_GAIN:
            imx214_awb_gain(pSetSensorAWB);
            break;
        case SENSOR_FEATURE_SET_HDR_SHUTTER:
            LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1));
            ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1));
            break;
        default:
            break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

//kin0603
UINT32 IMX214_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
//UINT32 IMX214_MIPI_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/



static const char *imx214Vendor = "Sony";
static const char *imx214NAME = "imx214_main";
static const char *imx214Size = "13.0M";

static ssize_t sensor_vendor_restore(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
    if(buf[0] == '1'){
        normal_fps = 0;
    }else{
        normal_fps = 1;
    }
	return size;
}

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", imx214Vendor, imx214NAME, imx214Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0666, sensor_vendor_show, sensor_vendor_restore);
static DEVICE_ATTR(otp, 0444, sensor_otp_show, NULL);

static struct kobject *android_imx214;


static int imx214_sysfs_init(void)
{
	int ret ;
	LOG_INF("kobject creat and add\n");
    if(android_imx214 == NULL){
        android_imx214 = kobject_create_and_add("android_camera", NULL);
        if (android_imx214 == NULL) {
            LOG_INF("subsystem_register failed\n");
            ret = -ENOMEM;
            return ret ;
        }
        LOG_INF("sysfs_create_file\n");
        ret = sysfs_create_file(android_imx214, &dev_attr_sensor.attr);
        if (ret) {
            LOG_INF("sysfs_create_file " \
                    "failed\n");
            kobject_del(android_imx214);
            android_imx214 = NULL;
        }

        ret = sysfs_create_file(android_imx214, &dev_attr_otp.attr);
        if (ret) {
            LOG_INF("sysfs_create_file " \
                    "failed\n");
            kobject_del(android_imx214);
            android_imx214 = NULL;
        }
    }
	return 0 ;
}
