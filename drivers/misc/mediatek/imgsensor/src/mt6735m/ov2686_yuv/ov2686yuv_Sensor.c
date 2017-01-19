/*****************************************************************************
 *
 * Filename:
 * ---------
 *   OV2686yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.0.0
 *
 * Author:
 * -------
 *   Mormo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2011/10/25 Firsty Released By Mormo(using "OV2686.set Revision1721" )
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "ov2686yuv_Sensor.h"
//#include "ov2686yuv_Camera_Sensor_para.h"
//#include "ov2686yuv_CameraCustomized.h"

#define OV2686YUV_DEBUG
#ifdef OV2686YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif
struct OV2686_Sensor_Struct
{
#if 0
	//kal_uint8   Banding;
    //kal_bool	  NightMode;
	//kal_bool	  VideoMode;
	//kal_uint16  Fps;
    //kal_uint16  ShutterStep;
	//kal_uint8   IsPVmode;
//	kal_uint32  PreviewDummyPixels;
//	kal_uint32  PreviewDummyLines;
//	kal_uint32  CaptureDummyPixels;
//	kal_uint32  CaptureDummyLines;
//	kal_uint32  PreviewPclk;
//	kal_uint32  CapturePclk;
//	kal_uint32  ZsdturePclk;
//	kal_uint32  PreviewShutter;
//	kal_uint32  PreviewExtraShutter;
//	kal_uint32  SensorGain;
//	kal_bool    	manualAEStart;
	kal_bool    	userAskAeLock;
    kal_bool    	userAskAwbLock;
//	kal_uint32      currentExposureTime;
//    kal_uint32      currentShutter;
//	kal_uint32      currentextshutter;
//    kal_uint32      currentAxDGain;
	kal_uint32  	sceneMode;
    unsigned char isoSpeed;
	kal_bool    	AE_ENABLE;
	
//	unsigned char zsd_flag;
   OV2686_SENSOR_MODE SensorMode;
	kal_uint16 wb;
#else
 kal_uint8   IsPVmode;
	kal_uint32  PreviewDummyPixels;
	kal_uint32  PreviewDummyLines;
	kal_uint32  CaptureDummyPixels;
	kal_uint32  CaptureDummyLines;
	kal_uint32  PreviewPclk;
	kal_uint32  CapturePclk;
	kal_uint32  PreviewShutter;
	kal_uint32  SensorGain;

	kal_bool    	userAskAeLock;
    kal_bool    	userAskAwbLock;
//#ifdef MT6572
	kal_uint32  sceneMode;
	kal_uint32  SensorShutter;
	unsigned char 	isoSpeed;
	kal_bool    	AE_ENABLE;
//#endif
	OV2686_SENSOR_MODE SensorMode;
	kal_uint16 wb;
#endif

} ;


   

static kal_uint8 OV2686_Banding_setting = AE_FLICKER_MODE_50HZ;

static struct OV2686_Sensor_Struct OV2686_SensorDriver;
static DEFINE_SPINLOCK(OV2686_drv_lock);
static MSDK_SCENARIO_ID_ENUM OV2686_CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
#define OV2686_TEST_PATTERN_CHECKSUM (0xc3bfa122)//(0xa2054b84)//0x12345678

static kal_uint32 OV2686_zoom_factor = 0; 


///////////////////////////////////
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

/*************************************************************************
* FUNCTION
*    OV2686_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define OV2686_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,OV2686_WRITE_ID)

kal_uint16 OV2686_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV2686_WRITE_ID);
    
    return get_byte;
}

/*
static void OV2686_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), OV2686_WRITE_ID); 

#if (defined(__OV2686_DEBUG_TRACE__))
  if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}
*/

/*************************************************************************
* FUNCTION
*    OV2686_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
/*
static kal_uint8 OV2686_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;
   
    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), OV2686_WRITE_ID)) {
        SENSORDB("ERROR: OV2686_read_cmos_sensor \n");
    }

#if (defined(__OV2686_DEBUG_TRACE__))
  if (size != rt) printk("I2C read %x error\n", addr);
#endif

  return in_buff[0];
}

*/
/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   OV2686_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 OV2686_dummy_pixels = 0, OV2686_dummy_lines = 0;
kal_bool   OV2686_MODE_CAPTURE = KAL_FALSE;
kal_bool   OV2686_CAM_BANDING_50HZ = KAL_FALSE;

kal_uint32 OV2686_isp_master_clock;
static kal_uint32 OV2686_g_fPV_PCLK = 26;

kal_uint8 OV2686_sensor_write_I2C_address = OV2686_WRITE_ID;
kal_uint8 OV2686_sensor_read_I2C_address = OV2686_READ_ID;

UINT8 OV2686PixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT OV2686SensorConfigData;

//#define //OV2686_SET_PAGE0 	OV2686_write_cmos_sensor(0xfe, 0x00) //ov2686 no page selecting
//#define //OV2686_SET_PAGE1 	OV2686_write_cmos_sensor(0xfe, 0x01) //ov2686 no page selection

kal_bool OV2686_night_mode_enable = KAL_FALSE;

static kal_uint32 MINFramerate = 0;
static kal_uint32 MAXFramerate = 0;



/*************************************************************************
 * FUNCTION
 *	OV2686_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of OV2686 to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void OV2686_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_OV2686_Shutter */


/*************************************************************************
 * FUNCTION
 *	OV2686_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of OV2686 .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 OV2686_Read_Shutter(void)
{

    kal_uint16 temp_reg1, temp_reg2 ,temp_reg3 ,shutter;
	
	temp_reg1 = OV2686_read_cmos_sensor(0x3500);   
	temp_reg2 = OV2686_read_cmos_sensor(0x3501);  
	temp_reg3 = OV2686_read_cmos_sensor(0x3502); 

	shutter  = (temp_reg1 <<12)|(temp_reg2<<4)|(temp_reg3>>4);


	return shutter;
} /* OV2686_read_shutter */


 void OV2686WriteShutter(kal_uint32 shutter)
{
	if (shutter<1)
	{
		  shutter=1;
	}	
	shutter*=16;
	
	OV2686_write_cmos_sensor(0x3502, shutter & 0x00FF);          
	OV2686_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  
	OV2686_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));
 }

unsigned char OV2686_get_iso(kal_uint16 readgain)
{
    kal_uint32 sensor_gain = 0;
	kal_uint8 iso_value;

	sensor_gain = readgain;
	sensor_gain = sensor_gain>>4;

    if(sensor_gain < 4)
    {
        iso_value = AE_ISO_100;
	}
	else if(sensor_gain > 15)
	{
        iso_value = AE_ISO_400;
	}
	else
	{
        iso_value = AE_ISO_200;
	}

	return iso_value;
}
/*************************************************************************
 * FUNCTION
 *	OV2686_write_reg
 *
 * DESCRIPTION
 *	This function set the register of OV2686.
 *
 * PARAMETERS
 *	addr : the register index of OV2686
 *  para : setting parameter of the specified register of OV2686
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void OV2686_write_reg(kal_uint32 addr, kal_uint32 para)
{
	OV2686_write_cmos_sensor(addr, para);
} /* OV2686_write_reg() */


/*************************************************************************
 * FUNCTION
 *	OV2686_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from OV2686.
 *
 * PARAMETERS
 *	addr : the register index of OV2686
 *
 * RETURNS
 *	the data that read from OV2686
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 OV2686_read_reg(kal_uint32 addr)
{
	return OV2686_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	OV2686_awb_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void OV2686_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = OV2686_read_cmos_sensor(0x5180);
	spin_lock(&OV2686_drv_lock);
	temp_AWB_reg = OV2686_SensorDriver.wb;
	spin_unlock(&OV2686_drv_lock);
	
	SENSORDB("[OV2686]OV2686_awb_enable reg 0x42=%x:\n ", temp_AWB_reg);
	if (enalbe)
	{
		temp_AWB_reg = (temp_AWB_reg &0xfd);
		OV2686_write_cmos_sensor(0x5180, temp_AWB_reg);
	}
	else
	{
		temp_AWB_reg = (temp_AWB_reg |0x02);
		OV2686_write_cmos_sensor(0x5180, temp_AWB_reg);
	}

	spin_lock(&OV2686_drv_lock);
	OV2686_SensorDriver.wb = temp_AWB_reg;
	spin_unlock(&OV2686_drv_lock);

}
static void OV2686_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 AeTemp;
	SENSORDB("[OV2686]enter OV2686_set_AE_mode function:\n ");
   AeTemp = OV2686_read_cmos_sensor(0x3503);
    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
        OV2686_write_cmos_sensor(0x3503, AeTemp&(~0x03));
    }
    else
    {
        // turn off AEC/AGC
		OV2686_write_cmos_sensor(0x3503, AeTemp |0x03);
    }
	SENSORDB("[OV2686]exit OV2686_set_AE_mode function:\n ");
}

/*************************************************************************
 * FUNCTION
 *	OV2686_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of OV2686 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from OV2686
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void OV2686_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* OV2686_config_window */


/*************************************************************************
 * FUNCTION
 *	OV2686_SetGain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 OV2686_SetGain(kal_uint16 iGain)
{
	return iGain;
}
kal_uint16 OV2686_ReadGain(void)
{
    	kal_uint8 temp_reg0;
	kal_uint16 gain;

	temp_reg0 = OV2686_read_cmos_sensor(0x350b);

	gain = (temp_reg0 & 0xFF);

	return gain;
}

/*************************************************************************
 * FUNCTION
 *	OV2686_GAMMA_Select
 *
 * DESCRIPTION
 *	This function select gamma of OV2686.
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
void OV2686_GAMMA_Select(kal_uint32 GammaLvl)
{
#if 0
	switch(GammaLvl)
	{
		case OV2686_RGB_Gamma_m1:											  //smallest gamma curve
			
			break;
		case OV2686_RGB_Gamma_m2:
			
			break;
			
		case OV2686_RGB_Gamma_m3:			
			
			break;
			
		case OV2686_RGB_Gamma_m4:
			
			break;
			
		case OV2686_RGB_Gamma_m5:
			
			break;
			
		case OV2686_RGB_Gamma_m6:
			
			break;
		case OV2686_RGB_Gamma_night:								
			
			break;
		default:
			break;
	}
#endif
}

/*************************************************************************
 * FUNCTION
 *	OV2686_NightMode
 *
 * DESCRIPTION
 *	This function night mode of OV2686.
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
void OV2686_night_mode(kal_bool bEnable)
{
	kal_uint16 night = OV2686_read_cmos_sensor(0x3A00);
	if (bEnable)
	{
		OV2686_write_cmos_sensor(0x3A00,night|0x02); //30fps-5fps
       	OV2686_write_cmos_sensor(0x382a,0x08);//disable 0x00
       	OV2686_write_cmos_sensor(0x3a0a,0x0f); 
      	OV2686_write_cmos_sensor(0x3a0b,0x18);                         
      	OV2686_write_cmos_sensor(0x3a0c,0x0f); 
      	OV2686_write_cmos_sensor(0x3a0d,0x18);              
	}
	else 
	{
		OV2686_write_cmos_sensor(0x3A00,night|0x02); //30fps-10fps   
        OV2686_write_cmos_sensor(0x382a,0x08);//disable 0x00
       	OV2686_write_cmos_sensor(0x3a0a,0x0a); 
      	OV2686_write_cmos_sensor(0x3a0b,0x92);                         
      	OV2686_write_cmos_sensor(0x3a0c,0x0a); 
      	OV2686_write_cmos_sensor(0x3a0d,0x92);  
	}
	
	spin_lock(&OV2686_drv_lock);
	//OV2686_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
	OV2686_night_mode_enable = bEnable;
	spin_unlock(&OV2686_drv_lock);
	SENSORDB("[OV2686]CONTROLFLOW OV2686_night_mode mode = %d", bEnable);
} /* OV2686_NightMode */


/*************************************************************************
* FUNCTION
*	OV2686_Sensor_Init
*
* DESCRIPTION
*	This function apply all of the initial setting to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
*************************************************************************/
void OV2686_Sensor_Init(void)
{
	OV2686_write_cmos_sensor(0x3000, 0x03);
OV2686_write_cmos_sensor(0x3001, 0xff);
OV2686_write_cmos_sensor(0x3002, 0x1a);
OV2686_write_cmos_sensor(0x3011, 0x03);
OV2686_write_cmos_sensor(0x301d, 0xf0);
OV2686_write_cmos_sensor(0x3020, 0x00);
OV2686_write_cmos_sensor(0x3021, 0x23);

OV2686_write_cmos_sensor(0x3082, 0x2c); 
OV2686_write_cmos_sensor(0x3083, 0x00);
OV2686_write_cmos_sensor(0x3084, 0x07);
OV2686_write_cmos_sensor(0x3085, 0x03);
OV2686_write_cmos_sensor(0x3086, 0x01); //15fps 01=30fps
OV2686_write_cmos_sensor(0x3087, 0x00);
OV2686_write_cmos_sensor(0x3106, 0x01);

OV2686_write_cmos_sensor(0x3501, 0x26);
OV2686_write_cmos_sensor(0x3502, 0x40);
OV2686_write_cmos_sensor(0x3503, 0x03);
OV2686_write_cmos_sensor(0x350b, 0x36);
OV2686_write_cmos_sensor(0x3600, 0xb4);
OV2686_write_cmos_sensor(0x3603, 0x35);
OV2686_write_cmos_sensor(0x3604, 0x24);
OV2686_write_cmos_sensor(0x3605, 0x00);
OV2686_write_cmos_sensor(0x3620, 0x25);
OV2686_write_cmos_sensor(0x3621, 0x37);
OV2686_write_cmos_sensor(0x3622, 0x23);
OV2686_write_cmos_sensor(0x3628, 0x10);
OV2686_write_cmos_sensor(0x3701, 0x64);
OV2686_write_cmos_sensor(0x3705, 0x3c);
OV2686_write_cmos_sensor(0x370a, 0x23);
OV2686_write_cmos_sensor(0x370c, 0x50);
OV2686_write_cmos_sensor(0x370d, 0xc0);
OV2686_write_cmos_sensor(0x3717, 0x58);
OV2686_write_cmos_sensor(0x3718, 0x80);
OV2686_write_cmos_sensor(0x3720, 0x00);
OV2686_write_cmos_sensor(0x3721, 0x00);
OV2686_write_cmos_sensor(0x3722, 0x00);
OV2686_write_cmos_sensor(0x3723, 0x00);
OV2686_write_cmos_sensor(0x3738, 0x00);
OV2686_write_cmos_sensor(0x3781, 0x80);
OV2686_write_cmos_sensor(0x3789, 0x60);

OV2686_write_cmos_sensor(0x5606, 0x23);//contrast
OV2686_write_cmos_sensor(0x3800, 0x00);
OV2686_write_cmos_sensor(0x3801, 0x00);
OV2686_write_cmos_sensor(0x3802, 0x00);
OV2686_write_cmos_sensor(0x3803, 0x00);
OV2686_write_cmos_sensor(0x3804, 0x06);
OV2686_write_cmos_sensor(0x3805, 0x4f);
OV2686_write_cmos_sensor(0x3806, 0x04);
OV2686_write_cmos_sensor(0x3807, 0xbf);
OV2686_write_cmos_sensor(0x3808, 0x03);
OV2686_write_cmos_sensor(0x3809, 0x20);
OV2686_write_cmos_sensor(0x380a, 0x02);
OV2686_write_cmos_sensor(0x380b, 0x58);
OV2686_write_cmos_sensor(0x3810, 0x00);
OV2686_write_cmos_sensor(0x3811, 0x04);
OV2686_write_cmos_sensor(0x3812, 0x00);
OV2686_write_cmos_sensor(0x3813, 0x04);
OV2686_write_cmos_sensor(0x3814, 0x31);
OV2686_write_cmos_sensor(0x3815, 0x31);

OV2686_write_cmos_sensor(0x3820, 0xc6); //modified by zhangxinghong
OV2686_write_cmos_sensor(0x3821, 0x05);

OV2686_write_cmos_sensor(0x380c, 0x06);
OV2686_write_cmos_sensor(0x380d, 0xac);
OV2686_write_cmos_sensor(0x380e, 0x02);
OV2686_write_cmos_sensor(0x380f, 0x84);

OV2686_write_cmos_sensor(0x3a02, 0x90);//50hz 10=60hz
OV2686_write_cmos_sensor(0x3a06, 0x00);
OV2686_write_cmos_sensor(0x3a07, 0xc2);
OV2686_write_cmos_sensor(0x3a08, 0x00);
OV2686_write_cmos_sensor(0x3a09, 0xa1);

OV2686_write_cmos_sensor(0x3a0e, 0x02);
OV2686_write_cmos_sensor(0x3a0f, 0x46);
OV2686_write_cmos_sensor(0x3a10, 0x02);
OV2686_write_cmos_sensor(0x3a11, 0x84);

//night mode enable  0x3a00=0x43,0x382a=0x08
//night mode disable 0x3a00=0x41,0x382a=0x00
OV2686_write_cmos_sensor(0x3a00, 0x43);
OV2686_write_cmos_sensor(0x382a, 0x08);
OV2686_write_cmos_sensor(0x3a0a, 0x07);
OV2686_write_cmos_sensor(0x3a0b, 0x8c);
OV2686_write_cmos_sensor(0x3a0c, 0x07);
OV2686_write_cmos_sensor(0x3a0d, 0x8c);

OV2686_write_cmos_sensor(0x3a13,0x58);
OV2686_write_cmos_sensor(0x4000, 0x81);
OV2686_write_cmos_sensor(0x4001, 0x40);
OV2686_write_cmos_sensor(0x4008, 0x00);
OV2686_write_cmos_sensor(0x4009, 0x03);
OV2686_write_cmos_sensor(0x4300, 0x32);		//0x31
OV2686_write_cmos_sensor(0x430e, 0x00);
OV2686_write_cmos_sensor(0x4602, 0x02);
OV2686_write_cmos_sensor(0x5000, 0xff);
OV2686_write_cmos_sensor(0x5001, 0x05);
OV2686_write_cmos_sensor(0x5002, 0x32);
OV2686_write_cmos_sensor(0x5003, 0x04);
OV2686_write_cmos_sensor(0x5004, 0xff);
OV2686_write_cmos_sensor(0x5005, 0x12);
OV2686_write_cmos_sensor(0x3784, 0x08);

OV2686_write_cmos_sensor(0x5180, 0xf4);
OV2686_write_cmos_sensor(0x5181, 0x11);
OV2686_write_cmos_sensor(0x5182, 0x41);
OV2686_write_cmos_sensor(0x5183, 0x42);
OV2686_write_cmos_sensor(0x5184, 0x6e);
OV2686_write_cmos_sensor(0x5185, 0x56);
OV2686_write_cmos_sensor(0x5186, 0xb4);
OV2686_write_cmos_sensor(0x5187, 0xb2);
OV2686_write_cmos_sensor(0x5188, 0x08);
OV2686_write_cmos_sensor(0x5189, 0x0e);
OV2686_write_cmos_sensor(0x518a, 0x0e);
OV2686_write_cmos_sensor(0x518b, 0x46);
OV2686_write_cmos_sensor(0x518c, 0x38);
OV2686_write_cmos_sensor(0x518d, 0xf8);
OV2686_write_cmos_sensor(0x518e, 0x04);
OV2686_write_cmos_sensor(0x518f, 0x7f);
OV2686_write_cmos_sensor(0x5190, 0x40);
OV2686_write_cmos_sensor(0x5191, 0x5f);
OV2686_write_cmos_sensor(0x5192, 0x40);
OV2686_write_cmos_sensor(0x5193, 0xff);
OV2686_write_cmos_sensor(0x5194, 0x40);
OV2686_write_cmos_sensor(0x5195, 0x07);
OV2686_write_cmos_sensor(0x5196, 0x04);
OV2686_write_cmos_sensor(0x5197, 0x04);
OV2686_write_cmos_sensor(0x5198, 0x00);
OV2686_write_cmos_sensor(0x5199, 0x05);
OV2686_write_cmos_sensor(0x519a, 0xd2);
OV2686_write_cmos_sensor(0x519b, 0x04);

OV2686_write_cmos_sensor(0x5200, 0x09);
OV2686_write_cmos_sensor(0x5201, 0x00);
OV2686_write_cmos_sensor(0x5202, 0x06);
OV2686_write_cmos_sensor(0x5203, 0x20);
OV2686_write_cmos_sensor(0x5204, 0x41);
OV2686_write_cmos_sensor(0x5205, 0x16);
OV2686_write_cmos_sensor(0x5206, 0x00);
OV2686_write_cmos_sensor(0x5207, 0x05);
OV2686_write_cmos_sensor(0x520b, 0x30);
OV2686_write_cmos_sensor(0x520c, 0x75);
OV2686_write_cmos_sensor(0x520d, 0x00);
OV2686_write_cmos_sensor(0x520e, 0x30);
OV2686_write_cmos_sensor(0x520f, 0x75);
OV2686_write_cmos_sensor(0x5210, 0x00);

OV2686_write_cmos_sensor(0x5280, 0x14);
OV2686_write_cmos_sensor(0x5281, 0x02);
OV2686_write_cmos_sensor(0x5282, 0x02);
OV2686_write_cmos_sensor(0x5283, 0x04);
OV2686_write_cmos_sensor(0x5284, 0x06);
OV2686_write_cmos_sensor(0x5285, 0x08);
OV2686_write_cmos_sensor(0x5286, 0x0c);
OV2686_write_cmos_sensor(0x5287, 0x10);

OV2686_write_cmos_sensor(0x5300, 0xc5);
OV2686_write_cmos_sensor(0x5301, 0xa0);
OV2686_write_cmos_sensor(0x5302, 0x06);
OV2686_write_cmos_sensor(0x5303, 0x0a);
OV2686_write_cmos_sensor(0x5304, 0x30);
OV2686_write_cmos_sensor(0x5305, 0x60);
OV2686_write_cmos_sensor(0x5306, 0x90);
OV2686_write_cmos_sensor(0x5307, 0xc0);
OV2686_write_cmos_sensor(0x5308, 0x82);
OV2686_write_cmos_sensor(0x5309, 0x00);
OV2686_write_cmos_sensor(0x530a, 0x26);
OV2686_write_cmos_sensor(0x530b, 0x02);
OV2686_write_cmos_sensor(0x530c, 0x02);
OV2686_write_cmos_sensor(0x530d, 0x00);
OV2686_write_cmos_sensor(0x530e, 0x0c);
OV2686_write_cmos_sensor(0x530f, 0x14);
OV2686_write_cmos_sensor(0x5310, 0x1a);
OV2686_write_cmos_sensor(0x5311, 0x20);
OV2686_write_cmos_sensor(0x5312, 0x80);
OV2686_write_cmos_sensor(0x5313, 0x4b);

OV2686_write_cmos_sensor(0x5380, 0x01);
OV2686_write_cmos_sensor(0x5381, 0x0c);
OV2686_write_cmos_sensor(0x5382, 0x00);
OV2686_write_cmos_sensor(0x5383, 0x16);
OV2686_write_cmos_sensor(0x5384, 0x00);
OV2686_write_cmos_sensor(0x5385, 0xb3);
OV2686_write_cmos_sensor(0x5386, 0x00);
OV2686_write_cmos_sensor(0x5387, 0x7e);
OV2686_write_cmos_sensor(0x5388, 0x00);
OV2686_write_cmos_sensor(0x5389, 0x07);
OV2686_write_cmos_sensor(0x538a, 0x01);
OV2686_write_cmos_sensor(0x538b, 0x35);
OV2686_write_cmos_sensor(0x538c, 0x00);

/*
OV2686_write_cmos_sensor(0x5380,0x1 );
OV2686_write_cmos_sensor(0x5381,0x52);
OV2686_write_cmos_sensor(0x5382,0x0 );
OV2686_write_cmos_sensor(0x5383,0x4A);
OV2686_write_cmos_sensor(0x5384,0x0 );
OV2686_write_cmos_sensor(0x5385,0xB6);
OV2686_write_cmos_sensor(0x5386,0x0 );
OV2686_write_cmos_sensor(0x5387,0x8D);
OV2686_write_cmos_sensor(0x5388,0x0 );
OV2686_write_cmos_sensor(0x5389,0x3A);
OV2686_write_cmos_sensor(0x538a,0x0 );
OV2686_write_cmos_sensor(0x538b,0xA6);
OV2686_write_cmos_sensor(0x538c,0x0 );
*/
OV2686_write_cmos_sensor(0x5400, 0x0d);
OV2686_write_cmos_sensor(0x5401, 0x18);
OV2686_write_cmos_sensor(0x5402, 0x31);
OV2686_write_cmos_sensor(0x5403, 0x5a);
OV2686_write_cmos_sensor(0x5404, 0x65);
OV2686_write_cmos_sensor(0x5405, 0x6f);
OV2686_write_cmos_sensor(0x5406, 0x77);
OV2686_write_cmos_sensor(0x5407, 0x80);
OV2686_write_cmos_sensor(0x5408, 0x87);
OV2686_write_cmos_sensor(0x5409, 0x8f);
OV2686_write_cmos_sensor(0x540a, 0xa2);
OV2686_write_cmos_sensor(0x540b, 0xb2);
OV2686_write_cmos_sensor(0x540c, 0xcc);
OV2686_write_cmos_sensor(0x540d, 0xe4);
OV2686_write_cmos_sensor(0x540e, 0xf0);
OV2686_write_cmos_sensor(0x540f, 0xa0);
OV2686_write_cmos_sensor(0x5410, 0x6e);
OV2686_write_cmos_sensor(0x5411, 0x06);

OV2686_write_cmos_sensor(0x5480, 0x19);
OV2686_write_cmos_sensor(0x5481, 0x00);
OV2686_write_cmos_sensor(0x5482, 0x09);
OV2686_write_cmos_sensor(0x5483, 0x12);
OV2686_write_cmos_sensor(0x5484, 0x04);
OV2686_write_cmos_sensor(0x5485, 0x06);
OV2686_write_cmos_sensor(0x5486, 0x08);
OV2686_write_cmos_sensor(0x5487, 0x0c);
OV2686_write_cmos_sensor(0x5488, 0x10);
OV2686_write_cmos_sensor(0x5489, 0x18);

OV2686_write_cmos_sensor(0x5500, 0x02);
OV2686_write_cmos_sensor(0x5501, 0x03);
OV2686_write_cmos_sensor(0x5502, 0x04);
OV2686_write_cmos_sensor(0x5503, 0x05);
OV2686_write_cmos_sensor(0x5504, 0x06);
OV2686_write_cmos_sensor(0x5505, 0x08);
OV2686_write_cmos_sensor(0x5506, 0x00);
OV2686_write_cmos_sensor(0x5600, 0x06);
OV2686_write_cmos_sensor(0x5603, 0x40);
OV2686_write_cmos_sensor(0x5604, 0x28);
OV2686_write_cmos_sensor(0x5609, 0x20);
OV2686_write_cmos_sensor(0x560a, 0x60);

OV2686_write_cmos_sensor(0x5780, 0x3e);
OV2686_write_cmos_sensor(0x5781, 0x0f);
OV2686_write_cmos_sensor(0x5782, 0x04);
OV2686_write_cmos_sensor(0x5783, 0x02);
OV2686_write_cmos_sensor(0x5784, 0x01);
OV2686_write_cmos_sensor(0x5785, 0x01);
OV2686_write_cmos_sensor(0x5786, 0x00);
OV2686_write_cmos_sensor(0x5787, 0x04);
OV2686_write_cmos_sensor(0x5788, 0x02);
OV2686_write_cmos_sensor(0x5789, 0x00);
OV2686_write_cmos_sensor(0x578a, 0x01);
OV2686_write_cmos_sensor(0x578b, 0x02);
OV2686_write_cmos_sensor(0x578c, 0x03);
OV2686_write_cmos_sensor(0x578d, 0x03);
OV2686_write_cmos_sensor(0x578e, 0x08);
OV2686_write_cmos_sensor(0x578f, 0x0c);
OV2686_write_cmos_sensor(0x5790, 0x08);
OV2686_write_cmos_sensor(0x5791, 0x04);
OV2686_write_cmos_sensor(0x5792, 0x00);
OV2686_write_cmos_sensor(0x5793, 0x00);
OV2686_write_cmos_sensor(0x5794, 0x03);

OV2686_write_cmos_sensor(0x5800, 0x03);
OV2686_write_cmos_sensor(0x5801, 0x14);
OV2686_write_cmos_sensor(0x5802, 0x02);
OV2686_write_cmos_sensor(0x5803, 0x64);
OV2686_write_cmos_sensor(0x5804, 0x22);//0x1e
OV2686_write_cmos_sensor(0x5805, 0x05);
OV2686_write_cmos_sensor(0x5806, 0x2a);
OV2686_write_cmos_sensor(0x5807, 0x05);
OV2686_write_cmos_sensor(0x5808, 0x03);
OV2686_write_cmos_sensor(0x5809, 0x17);
OV2686_write_cmos_sensor(0x580a, 0x02);
OV2686_write_cmos_sensor(0x580b, 0x63);
OV2686_write_cmos_sensor(0x580c, 0x1a);
OV2686_write_cmos_sensor(0x580d, 0x05);
OV2686_write_cmos_sensor(0x580e, 0x1f);
OV2686_write_cmos_sensor(0x580f, 0x05);
OV2686_write_cmos_sensor(0x5810, 0x03);
OV2686_write_cmos_sensor(0x5811, 0x0c);
OV2686_write_cmos_sensor(0x5812, 0x02);
OV2686_write_cmos_sensor(0x5813, 0x5e);
OV2686_write_cmos_sensor(0x5814, 0x18);
OV2686_write_cmos_sensor(0x5815, 0x05);
OV2686_write_cmos_sensor(0x5816, 0x19);
OV2686_write_cmos_sensor(0x5817, 0x05);
OV2686_write_cmos_sensor(0x5818, 0x0d);
OV2686_write_cmos_sensor(0x5819, 0x40);
OV2686_write_cmos_sensor(0x581a, 0x04);
OV2686_write_cmos_sensor(0x581b, 0x0c);

OV2686_write_cmos_sensor(0x3106, 0x21);

OV2686_write_cmos_sensor(0x3a03, 0x4c);
OV2686_write_cmos_sensor(0x3a04, 0x40);

OV2686_write_cmos_sensor(0x3503, 0x00);

OV2686_write_cmos_sensor(0x0100, 0x01);

}

static void OV2686PreviewSetting(void)
{
OV2686_write_cmos_sensor(0x3500,((OV2686_SensorDriver.PreviewShutter*16)>>16)&0xff); 
OV2686_write_cmos_sensor(0x3501,((OV2686_SensorDriver.PreviewShutter*16)>>8)&0xff); 
OV2686_write_cmos_sensor(0x3502,(OV2686_SensorDriver.PreviewShutter*16)&0xff); 
OV2686_write_cmos_sensor(0x350B, OV2686_SensorDriver.SensorGain);           

OV2686_write_cmos_sensor(0x370a, 0x23);

OV2686_write_cmos_sensor(0x3808, 0x03);
OV2686_write_cmos_sensor(0x3809, 0x20);
OV2686_write_cmos_sensor(0x380a, 0x02);
OV2686_write_cmos_sensor(0x380b, 0x58);
OV2686_write_cmos_sensor(0x3810, 0x00);
OV2686_write_cmos_sensor(0x3811, 0x04);
OV2686_write_cmos_sensor(0x3812, 0x00);
OV2686_write_cmos_sensor(0x3813, 0x04);
OV2686_write_cmos_sensor(0x3814, 0x31);
OV2686_write_cmos_sensor(0x3815, 0x31);

OV2686_write_cmos_sensor(0x3086, 0x01);

OV2686_write_cmos_sensor(0x380c, 0x06);
OV2686_write_cmos_sensor(0x380d, 0xac);
OV2686_write_cmos_sensor(0x380e, 0x02);
OV2686_write_cmos_sensor(0x380f, 0x84);

OV2686_write_cmos_sensor(0x3820, 0xc6); //modified by zhangxinghong
OV2686_write_cmos_sensor(0x3821, 0x05);

OV2686_write_cmos_sensor(0x3a06, 0x00);
OV2686_write_cmos_sensor(0x3a07, 0xc2);
OV2686_write_cmos_sensor(0x3a08, 0x00);
OV2686_write_cmos_sensor(0x3a09, 0xa1);

OV2686_write_cmos_sensor(0x3a0e, 0x02);
OV2686_write_cmos_sensor(0x3a0f, 0x46);
OV2686_write_cmos_sensor(0x3a10, 0x02);
OV2686_write_cmos_sensor(0x3a11, 0x84);

OV2686_write_cmos_sensor(0x3a0a, 0x0a);
OV2686_write_cmos_sensor(0x3a0b, 0x92);
OV2686_write_cmos_sensor(0x3a0c, 0x0a);
OV2686_write_cmos_sensor(0x3a0d, 0x10);

OV2686_write_cmos_sensor(0x4008, 0x00);
OV2686_write_cmos_sensor(0x4009, 0x03);

OV2686_write_cmos_sensor(0x3503, 0x00);
OV2686_write_cmos_sensor(0x3a00, 0x43);
	
spin_lock(&OV2686_drv_lock);
OV2686_SensorDriver.IsPVmode = KAL_TRUE;
OV2686_SensorDriver.PreviewPclk= 480;
OV2686_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
spin_unlock(&OV2686_drv_lock);

}

void OV2686FullSizeCaptureSetting(void)
{
OV2686_write_cmos_sensor(0x3503, 0x03);
OV2686_write_cmos_sensor(0x3a00, 0x41);

OV2686_write_cmos_sensor(0x370a, 0x21);

OV2686_write_cmos_sensor(0x3808, 0x06);
OV2686_write_cmos_sensor(0x3809, 0x40);
OV2686_write_cmos_sensor(0x380a, 0x04);
OV2686_write_cmos_sensor(0x380b, 0xb0);
OV2686_write_cmos_sensor(0x3810, 0x00);
OV2686_write_cmos_sensor(0x3811, 0x08);
OV2686_write_cmos_sensor(0x3812, 0x00);
OV2686_write_cmos_sensor(0x3813, 0x08);
OV2686_write_cmos_sensor(0x3814, 0x11);
OV2686_write_cmos_sensor(0x3815, 0x11);

OV2686_write_cmos_sensor(0x3086, 0x01);

OV2686_write_cmos_sensor(0x380c, 0x06);
OV2686_write_cmos_sensor(0x380d, 0xa4);
OV2686_write_cmos_sensor(0x380e, 0x05);
OV2686_write_cmos_sensor(0x380f, 0x0e);

OV2686_write_cmos_sensor(0x3820, 0xc4); //modified by zhangxinghong
OV2686_write_cmos_sensor(0x3821, 0x04);

OV2686_write_cmos_sensor(0x3a06, 0x00);
OV2686_write_cmos_sensor(0x3a07, 0xc2);
OV2686_write_cmos_sensor(0x3a08, 0x00);
OV2686_write_cmos_sensor(0x3a09, 0xa1);

OV2686_write_cmos_sensor(0x3a0e, 0x04);
OV2686_write_cmos_sensor(0x3a0f, 0x8c);
OV2686_write_cmos_sensor(0x3a10, 0x05);
OV2686_write_cmos_sensor(0x3a11, 0x08);

OV2686_write_cmos_sensor(0x4008, 0x02);
OV2686_write_cmos_sensor(0x4009, 0x09);
	
spin_lock(&OV2686_drv_lock);
OV2686_SensorDriver.IsPVmode = KAL_FALSE;
OV2686_SensorDriver.CapturePclk= 585;
//OV2686_SensorDriver.SensorMode= SENSOR_MODE_CAPTURE;
spin_unlock(&OV2686_drv_lock);

}

void OV2686FullSizeCaptureSetting_ZSD(void)
{
OV2686_write_cmos_sensor(0x3503, 0x00);
OV2686_write_cmos_sensor(0x3a00, 0x41);
OV2686_write_cmos_sensor(0x3a0a, 0x05);
OV2686_write_cmos_sensor(0x3a0b, 0x0e);
OV2686_write_cmos_sensor(0x3a0c, 0x05);
OV2686_write_cmos_sensor(0x3a0d, 0x0e);

OV2686_write_cmos_sensor(0x370a, 0x21);

OV2686_write_cmos_sensor(0x3808, 0x06);
OV2686_write_cmos_sensor(0x3809, 0x40);
OV2686_write_cmos_sensor(0x380a, 0x04);
OV2686_write_cmos_sensor(0x380b, 0xb0);
OV2686_write_cmos_sensor(0x3810, 0x00);
OV2686_write_cmos_sensor(0x3811, 0x08);
OV2686_write_cmos_sensor(0x3812, 0x00);
OV2686_write_cmos_sensor(0x3813, 0x08);
OV2686_write_cmos_sensor(0x3814, 0x11);
OV2686_write_cmos_sensor(0x3815, 0x11);

OV2686_write_cmos_sensor(0x3086, 0x01);

OV2686_write_cmos_sensor(0x380c, 0x06);
OV2686_write_cmos_sensor(0x380d, 0xa4);
OV2686_write_cmos_sensor(0x380e, 0x05);
OV2686_write_cmos_sensor(0x380f, 0x0e);

OV2686_write_cmos_sensor(0x3820, 0xc4);  //modified by zhangxinghong
OV2686_write_cmos_sensor(0x3821, 0x04);

OV2686_write_cmos_sensor(0x3a06, 0x00);
OV2686_write_cmos_sensor(0x3a07, 0xc2);
OV2686_write_cmos_sensor(0x3a08, 0x00);
OV2686_write_cmos_sensor(0x3a09, 0xa1);

OV2686_write_cmos_sensor(0x3a0e, 0x04);
OV2686_write_cmos_sensor(0x3a0f, 0x8c);
OV2686_write_cmos_sensor(0x3a10, 0x05);
OV2686_write_cmos_sensor(0x3a11, 0x08);

OV2686_write_cmos_sensor(0x4008, 0x02);
OV2686_write_cmos_sensor(0x4009, 0x09);
	
spin_lock(&OV2686_drv_lock);
OV2686_SensorDriver.IsPVmode = KAL_FALSE;
OV2686_SensorDriver.CapturePclk= 585;
//OV2686_SensorDriver.SensorMode= SENSOR_MODE_CAPTURE;
spin_unlock(&OV2686_drv_lock);

}
/*************************************************************************
* FUNCTION
*	OV2686_Lens_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate lens parameter.
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
void OV2686_Lens_Select(kal_uint8 Lens_Tag)
{
	switch(Lens_Tag)
	{
		case CHT_806C_2:
			
			break;

		case CHT_808C_2:
			
			break;
			
		case LY_982A_H114:
			
			break;

		case XY_046A:
			
			break;

		case XY_0620:
			
			break;

		case XY_078V: 
			
			break;

		case YG1001A_F:
			
			break;

		default:
			break;
	}
}


/*************************************************************************
* FUNCTION
*	OV2686_GAMMA_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate GAMMA curve.
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
// add for deviceinfo class by hao.ren.hz@tcl.com at 20150331 
extern char Back_Camera[100];
extern char Front_Camera_Name[256]; 
UINT32 OV2686GetSensorID(UINT32 *sensorID)
{
    kal_uint16 sensor_id=0;
    int i;
	
	SENSORDB("[OV2686]CONTROLFLOW OV2686GetSensorID\n");

    OV2686_write_cmos_sensor(0x0103, 0x01);
    Sleep(20);

    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	sensor_id = (OV2686_read_cmos_sensor(0x300A) << 8) | OV2686_read_cmos_sensor(0x300B);
	            	printk("OV2686 Sensor id = %x\n", sensor_id);
	            	if (sensor_id == OV2686_SENSOR_ID)
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);

    printk("[==CAMERA YCD==] sensor_id = 0x%x", sensor_id);
    if(sensor_id != OV2686_SENSOR_ID)
    {
        SENSORDB("OV2686 Sensor id read failed, ID = %x\n", sensor_id);
		//sensor_id = OV2686_SENSOR_ID;
		*sensorID = 0xffffffff;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    sprintf(Back_Camera,"ov2686");
    *sensorID = sensor_id;
	sprintf(Front_Camera_Name,"OV2686:SHINETECH:SUP2:2M");

    RETAILMSG(1, (TEXT("Sensor Read ID OK \n")));
	
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	OV2686_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_OV2686() directly.
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
void OV2686_Write_More_Registers(void)
{
    //OV2686_GAMMA_Select(0);//0:use default
    //OV2686_Lens_Select(0);//0:use default
}


/*************************************************************************
 * FUNCTION
 *	OV2686Open
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
UINT32 OV2686Open(void)
{
    kal_uint16 sensor_id=0;
    int i;

	SENSORDB("[OV2686]CONTROLFLOW OV2686Open\n");

    OV2686_write_cmos_sensor(0x0103, 0x01);
    Sleep(20);

    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	           sensor_id = (OV2686_read_cmos_sensor(0x300A) << 8) | OV2686_read_cmos_sensor(0x300B);
	            	if (sensor_id == OV2686_SENSOR_ID)
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);

    if(sensor_id != OV2686_SENSOR_ID)
    {
        SENSORDB("OV2686 Sensor id read failed, ID = %x\n", sensor_id);
		//sensor_id = OV2686_SENSOR_ID;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    RETAILMSG(1, (TEXT("Sensor Read ID OK \n")));
    // initail sequence write in
    OV2686_Sensor_Init();
    OV2686_Write_More_Registers();
	
	spin_lock(&OV2686_drv_lock);
	//OV2686_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
	OV2686_night_mode_enable = KAL_FALSE;
	OV2686_MPEG4_encode_mode = KAL_FALSE;
	OV2686_SensorDriver.wb = 0xf4;
	OV2686_SensorDriver.SensorMode= SENSOR_MODE_INIT;
	spin_unlock(&OV2686_drv_lock);
    return ERROR_NONE;
} /* OV2686Open */

/*************************************************************************
 * FUNCTION
 *	OV2686Close
 *
 * DESCRIPTION
 *	This function is to turn off sensor module power.
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
UINT32 OV2686Close(void)
{

	SENSORDB("[OV2686]CONTROLFLOW OV2686Close\n");

    return ERROR_NONE;
} /* OV2686Close */


/*************************************************************************
 * FUNCTION
 * OV2686Preview
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
UINT32 OV2686Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;
    OV2686PreviewSetting();
    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
		SENSORDB("[OV2686]CONTROLFLOW OV2686Preview Video\n");
		    spin_lock(&OV2686_drv_lock);
        OV2686_MPEG4_encode_mode = KAL_TRUE;
    spin_unlock(&OV2686_drv_lock);
       
    }
    else
    {
		SENSORDB("[OV2686]CONTROLFLOW OV2686Preview camera\n");

		    spin_lock(&OV2686_drv_lock);
        OV2686_MPEG4_encode_mode = KAL_FALSE;
    spin_unlock(&OV2686_drv_lock);
    }
    /*
    image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_SVGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;
	*/
   image_window->GrabStartX= IMAGE_SENSOR_SVGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_SVGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&OV2686SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
//	OV2686_night_mode(OV2686_night_mode_enable);
    return ERROR_NONE;
} /* OV2686Preview */


/*************************************************************************
 * FUNCTION
 *	OV2686Capture
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
UINT32 OV2686Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
#if 0
		    spin_lock(&OV2686_drv_lock);
    OV2686_MODE_CAPTURE=KAL_TRUE;
        OV2686_MPEG4_encode_mode = KAL_FALSE;
    spin_unlock(&OV2686_drv_lock);

	
	SENSORDB("[OV2686]CONTROLFLOW OV2686Capture\n");

    image_window->GrabStartX = IMAGE_SENSOR_FULL_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_FULL_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;


    // copy sensor_config_data
    memcpy(&OV2686SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	//OV2686_night_mode(OV2686_night_mode_enable);
#endif

	kal_uint32 shutter = 0, prev_line_len = 0, cap_line_len = 0, temp = 0;

	SENSORDB("[OV2686Capture]enter\n");
	image_window->GrabStartX = IMAGE_SENSOR_FULL_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_FULL_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;
	
	OV2686_write_cmos_sensor(0x3503,OV2686_read_cmos_sensor(0x3503)|0x03);	
	OV2686_write_cmos_sensor(0x3a00,OV2686_read_cmos_sensor(0x3a00)&0xfd);  
	
	if(SENSOR_MODE_PREVIEW == OV2686_SensorDriver.SensorMode )
	{
		SENSORDB("[OV2686Capture]Normal Capture\n ");

		//OV2686_set_AE_mode(KAL_FALSE);
		//OV2686_set_AWB_mode(KAL_FALSE);	
		shutter=OV2686_Read_Shutter();
		temp =OV2686_ReadGain();	
		
		mDELAY(30);
		OV2686FullSizeCaptureSetting();
		
		spin_lock(&OV2686_drv_lock);
		OV2686_SensorDriver.SensorMode= SENSOR_MODE_CAPTURE;
		//OV2686_SensorDriver.CaptureDummyPixels = 0;
  	//OV2686_SensorDriver.CaptureDummyLines = 0;
		spin_unlock(&OV2686_drv_lock);
		
  		//OV2686SetDummy(OV2686_SensorDriver.CaptureDummyPixels, OV2686_SensorDriver.CaptureDummyLines);

		//prev_line_len = OV2686_PV_PERIOD_PIXEL_NUMS + OV2686_SensorDriver.PreviewDummyPixels;
  		//cap_line_len = OV2686_FULL_PERIOD_PIXEL_NUMS + OV2686_SensorDriver.CaptureDummyPixels;
  		//shutter = (shutter * OV2686_SensorDriver.CapturePclk) / OV2686_SensorDriver.PreviewPclk;
  		//shutter = (shutter * prev_line_len) / cap_line_len;	

  	OV2686WriteShutter(shutter);
  	mDELAY(150);
  		//OV2686WriteSensorGain(OV2686_SensorDriver.SensorGain);

		spin_lock(&OV2686_drv_lock);
  		OV2686_SensorDriver.SensorGain= temp;
		OV2686_SensorDriver.SensorShutter = shutter;
		spin_unlock(&OV2686_drv_lock);
	}
	else if(SENSOR_MODE_ZSD == OV2686_SensorDriver.SensorMode)
	{
		//for zsd hdr use
		shutter=OV2686_Read_Shutter();
		temp =OV2686_ReadGain();
		
		spin_lock(&OV2686_drv_lock);
  		OV2686_SensorDriver.SensorGain= temp;
		OV2686_SensorDriver.SensorShutter = shutter;
		spin_unlock(&OV2686_drv_lock);
	}
	
	SENSORDB("[OV2686Capture]exit\n");
	OV2686_SensorDriver.isoSpeed = OV2686_get_iso(temp);
	
	return ERROR_NONE; 
} /* OV2686_Capture() */



UINT32 OV2686GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_PV_HEIGHT;

    pSensorResolution->SensorHighSpeedVideoWidth = IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorHighSpeedVideoHeight = IMAGE_SENSOR_PV_HEIGHT;

    pSensorResolution->SensorSlimVideoWidth = IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorSlimVideoHeight = IMAGE_SENSOR_PV_HEIGHT;
    
    
    return ERROR_NONE;
} /* OV2686GetResolution() */


UINT32 OV2686GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_WIDTH;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

#if 0
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=FALSE;
 #endif
    pSensorInfo->CaptureDelayFrame = 3;
    pSensorInfo->PreviewDelayFrame = 14; // 12
    pSensorInfo->VideoDelayFrame = 4;


    pSensorInfo->HighSpeedVideoDelayFrame = 4;
    pSensorInfo->SlimVideoDelayFrame = 4;

    
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_4MA;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
   // case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;  
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
		pSensorInfo->SensorGrabStartX = 3;//IMAGE_SENSOR_SVGA_GRAB_PIXELS+3;
		pSensorInfo->SensorGrabStartY = 1;//IMAGE_SENSOR_SVGA_GRAB_LINES;

        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = 3;//IMAGE_SENSOR_VGA_GRAB_PIXELS+3;
		pSensorInfo->SensorGrabStartY = 1;//IMAGE_SENSOR_VGA_GRAB_LINES;
        break;
    default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = 3;//IMAGE_SENSOR_VGA_GRAB_PIXELS+3;
        pSensorInfo->SensorGrabStartY = 1;//IMAGE_SENSOR_VGA_GRAB_LINES;
        break;
    }
    OV2686PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &OV2686SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* OV2686GetInfo() */


UINT32 OV2686Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	SENSORDB("[OV2686]CONTROLFLOW OV2686Control ScenarioId = %d\n", ScenarioId); 

	spin_lock(&OV2686_drv_lock);
	OV2686_CurrentScenarioId = ScenarioId;
	spin_unlock(&OV2686_drv_lock);

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        OV2686Preview(pImageWindow, pSensorConfigData);
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
   // case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        OV2686Capture(pImageWindow, pSensorConfigData);
        break;
    }


    return ERROR_NONE;
}	/* OV2686Control() */

BOOL OV2686_set_param_wb(UINT16 para)
{

	SENSORDB("[OV2686]CONTROLFLOW OV2686_set_param_wb para = %d\n", para); 

	switch (para)
	{
		case AWB_MODE_OFF:
			OV2686_awb_enable(KAL_TRUE);

			
		break;
		
		case AWB_MODE_AUTO:
		default:

			OV2686_awb_enable(KAL_TRUE);
		break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy,D75
			OV2686_awb_enable(KAL_FALSE);
			OV2686_write_cmos_sensor(0x3208, 0x00); 
        	 
			       OV2686_write_cmos_sensor(0x5195, 0x07);
			       OV2686_write_cmos_sensor(0x5196, 0xdc);
			       OV2686_write_cmos_sensor(0x5197, 0x04);
			       OV2686_write_cmos_sensor(0x5198, 0x00);
			       OV2686_write_cmos_sensor(0x5199, 0x05);
			       OV2686_write_cmos_sensor(0x519a, 0xd3);
				   OV2686_write_cmos_sensor(0x3208, 0x10); 
             OV2686_write_cmos_sensor(0x3208, 0xa0); 
		break;
		
		case AWB_MODE_DAYLIGHT: //sunny,D65
			OV2686_awb_enable(KAL_FALSE);
			OV2686_write_cmos_sensor(0x3208, 0x00);   
                                     
			       OV2686_write_cmos_sensor(0x5195, 0x07);
			       OV2686_write_cmos_sensor(0x5196, 0x9c);
			       OV2686_write_cmos_sensor(0x5197, 0x04);
			       OV2686_write_cmos_sensor(0x5198, 0x00);
			       OV2686_write_cmos_sensor(0x5199, 0x05);
			       OV2686_write_cmos_sensor(0x519a, 0xf3);
			 
             OV2686_write_cmos_sensor(0x3208, 0x10); 
             OV2686_write_cmos_sensor(0x3208, 0xa0); 
		break;
		
		case AWB_MODE_INCANDESCENT: //office,TL84
			OV2686_awb_enable(KAL_FALSE);
			OV2686_write_cmos_sensor(0x3208, 0x00); 
             
			       OV2686_write_cmos_sensor(0x5195, 0x06);
			       OV2686_write_cmos_sensor(0x5196, 0xb8);
			       OV2686_write_cmos_sensor(0x5197, 0x04);
			       OV2686_write_cmos_sensor(0x5198, 0x00);
			       OV2686_write_cmos_sensor(0x5199, 0x06);
			       OV2686_write_cmos_sensor(0x519a, 0x5f);
				   OV2686_write_cmos_sensor(0x3208, 0x10); 
             OV2686_write_cmos_sensor(0x3208, 0xa0); 
		break;
		
		case AWB_MODE_TUNGSTEN: //home,A
			OV2686_awb_enable(KAL_FALSE);
			OV2686_write_cmos_sensor(0x3208, 0x00);  
                                      
             OV2686_write_cmos_sensor(0x5195, 0x04);
			       OV2686_write_cmos_sensor(0x5196, 0x90);
			       OV2686_write_cmos_sensor(0x5197, 0x04);
			       OV2686_write_cmos_sensor(0x5198, 0x00);
			       OV2686_write_cmos_sensor(0x5199, 0x09);
			       OV2686_write_cmos_sensor(0x519a, 0x20); 
                          
             OV2686_write_cmos_sensor(0x3208, 0x10); 
             OV2686_write_cmos_sensor(0x3208, 0xa0); 
		break;
		
		case AWB_MODE_FLUORESCENT://CWF
			OV2686_awb_enable(KAL_FALSE);
			OV2686_write_cmos_sensor(0x3208, 0x00);  
                                                    
             OV2686_write_cmos_sensor(0x5195, 0x06);
			       OV2686_write_cmos_sensor(0x5196, 0x30);
			       OV2686_write_cmos_sensor(0x5197, 0x04);
			       OV2686_write_cmos_sensor(0x5198, 0x00);
			       OV2686_write_cmos_sensor(0x5199, 0x04);
			       OV2686_write_cmos_sensor(0x519a, 0x30);
                                              
             OV2686_write_cmos_sensor(0x3208, 0x10); 
             OV2686_write_cmos_sensor(0x3208, 0xa0); 
		break;
		
		//default:
		//return FALSE;
	}

	return TRUE;
} /* OV2686_set_param_wb */


BOOL OV2686_set_param_effect(UINT16 para)
{
	switch (para)
    {
        case MEFFECT_OFF:
          	OV2686_write_cmos_sensor(0x3208,0x00);           	          
          
            OV2686_write_cmos_sensor(0x5600,0x06); 
           	OV2686_write_cmos_sensor(0x5603,0x40); 
            OV2686_write_cmos_sensor(0x5604,0x28);  
                     
           	OV2686_write_cmos_sensor(0x3208,0x10); 
            OV2686_write_cmos_sensor(0x3208,0xa0); 
			      break;
        case MEFFECT_SEPIA:
            OV2686_write_cmos_sensor(0x3208,0x00); 
                      
           	OV2686_write_cmos_sensor(0x5600,0x1e); 
           	OV2686_write_cmos_sensor(0x5603,0x40); 
            OV2686_write_cmos_sensor(0x5604,0xa0);
           	         
          	OV2686_write_cmos_sensor(0x3208,0x10); 
           	OV2686_write_cmos_sensor(0x3208,0xa0); 
		      	break;
        case MEFFECT_NEGATIVE:
          	OV2686_write_cmos_sensor(0x3208,0x00); 
          	          
            OV2686_write_cmos_sensor(0x5600,0x46);            	
           	          
            OV2686_write_cmos_sensor(0x3208,0x10); 
          	OV2686_write_cmos_sensor(0x3208,0xa0); 
			      break;
        case MEFFECT_SEPIAGREEN:
           	OV2686_write_cmos_sensor(0x3208,0x00); 
           	         
           	OV2686_write_cmos_sensor(0x5600,0x1e); 
           	OV2686_write_cmos_sensor(0x5603,0x60); 
            OV2686_write_cmos_sensor(0x5604,0x60);
           	          
           	OV2686_write_cmos_sensor(0x3208,0x10); 
           	OV2686_write_cmos_sensor(0x3208,0xa0); 
            break;
        case MEFFECT_SEPIABLUE:
          	OV2686_write_cmos_sensor(0x3208,0x00); 
          	          
            OV2686_write_cmos_sensor(0x5600,0x1e); 
           	OV2686_write_cmos_sensor(0x5603,0xa0); 
            OV2686_write_cmos_sensor(0x5604,0x40);
           	         
         	  OV2686_write_cmos_sensor(0x3208,0x10); 
           	OV2686_write_cmos_sensor(0x3208,0xa0); 
            break;
		case MEFFECT_MONO:
          	OV2686_write_cmos_sensor(0x3208,0x00); 
          	
            OV2686_write_cmos_sensor(0x5600,0x1e); 
           	OV2686_write_cmos_sensor(0x5603,0x80); 
            OV2686_write_cmos_sensor(0x5604,0x80); 
           	
          	OV2686_write_cmos_sensor(0x3208,0x10); 
           	OV2686_write_cmos_sensor(0x3208,0xa0); 
			      break;
     default:
            return KAL_FALSE;
    }

    return KAL_TRUE;

} /* OV2686_set_param_effect */


BOOL OV2686_set_param_banding(UINT16 para)
{
   kal_uint8 banding;
	SENSORDB("[OV2686]CONTROLFLOW OV2686_set_param_banding para = %d\n", para); 

	banding = OV2686_read_cmos_sensor(0x3A02);
	
    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
			spin_lock(&OV2686_drv_lock);
			OV2686_Banding_setting = AE_FLICKER_MODE_50HZ;
			spin_unlock(&OV2686_drv_lock);
			OV2686_write_cmos_sensor(0x3a02, banding|0x80);	
			OV2686_write_cmos_sensor(0x3a0a, 0x0a);
			OV2686_write_cmos_sensor(0x3a0b, 0x92);
			
					
			break;
        case AE_FLICKER_MODE_60HZ:			
			spin_lock(&OV2686_drv_lock);
            OV2686_Banding_setting = AE_FLICKER_MODE_60HZ;
			spin_unlock(&OV2686_drv_lock);
			OV2686_write_cmos_sensor(0x3a02, banding&0x7f);
			OV2686_write_cmos_sensor(0x3a0c, 0x0a);
			OV2686_write_cmos_sensor(0x3a0d, 0x10);
			break;
		case AE_FLICKER_MODE_AUTO:
		case AE_FLICKER_MODE_OFF:
		default:
			OV2686_write_cmos_sensor(0x3a02, banding|0x80);	
			OV2686_write_cmos_sensor(0x3a0a, 0x0a);
			OV2686_write_cmos_sensor(0x3a0b, 0x92);
			OV2686_CAM_BANDING_50HZ = KAL_TRUE;
			break;
		//return FALSE;
	}
	
    //	Sleep(500);
	return TRUE;
} /* OV2686_set_param_banding */

BOOL OV2686_set_param_exposure_for_HDR(UINT16 para)
{
	
	SENSORDB("[OV2686]CONTROLFLOW OV2686_set_param_exposure_for_HDR\n ");

	return TRUE;
}

BOOL OV2686_set_param_exposure(UINT16 para)
{

	 SENSORDB("[OV2686]CONTROLFLOW enter OV2686_set_param_exposure function:\n ");
	 SENSORDB("[OV2686]para=%d:\n",para);
	 //spin_lock(&OV2686_drv_lock);
/*	if (SCENE_MODE_HDR == OV2686_SensorDriver.sceneMode && 
	 SENSOR_MODE_CAPTURE == OV2686_SensorDriver.SensorMode)
	{
		//spin_unlock(&OV2686_drv_lock);
		OV2686_set_param_exposure_for_HDR(para);
		return TRUE;
	}*/


	kal_uint8 EvTemp0 = 0x00, EvTemp1 = 0x00, temp_reg= 0x00;

	
//modified by zhangxinghong
#if 0
	if (SCENE_MODE_HDR == OV2686_SensorDriver.sceneMode)
   {
       OV2686_set_param_exposure_for_HDR(para);
       return TRUE;
   }
#endif
	
	  temp_reg=OV2686_read_cmos_sensor(0x5608);
	  OV2686_write_cmos_sensor(0x5600,OV2686_read_cmos_sensor(0x5600)|0x04);

    switch (para)
    {	
		case AE_EV_COMP_30:
			                   EvTemp0= 0x30;        //modified by zhangxinghong
			EvTemp1= temp_reg&0xf7;
			break;
	    	case AE_EV_COMP_20:
			                   EvTemp0= 0x20;
			EvTemp1= temp_reg&0xf7;
			break;
		case AE_EV_COMP_10:
			                   EvTemp0= 0x10;
			EvTemp1= temp_reg&0xf7;
			break;
		case AE_EV_COMP_00:
			                   EvTemp0= 0x00;
			EvTemp1= temp_reg&0xf7;
			break;
		case AE_EV_COMP_n10:
			                   EvTemp0= 0x10;
			EvTemp1= temp_reg|0x08;	
			break;
               case AE_EV_COMP_n20:
			                   EvTemp0= 0x20;
			EvTemp1= temp_reg|0x08;	
			break;	
		case AE_EV_COMP_n30:
			                   EvTemp0= 0x30;
			EvTemp1= temp_reg|0x08;	              ////modified by zhangxinghong
			break;		

	
    default:
            return FALSE;
    }
    OV2686_write_cmos_sensor(0x3208, 0x00); 
    
	  OV2686_write_cmos_sensor(0x5607, EvTemp0);
	  OV2686_write_cmos_sensor(0x5608, EvTemp1);
	  	
    OV2686_write_cmos_sensor(0x3208, 0x10); 
    OV2686_write_cmos_sensor(0x3208, 0xa0); 
	return TRUE;
} /* OV2686_set_param_exposure */
///add
void OV2686_set_contrast(UINT16 para)
{   
    SENSORDB("[OV2686]CONTROLFLOW enter OV2686_set_contrast function:\n ");
    switch (para)
    {
        case ISP_CONTRAST_LOW:			 
			
			break;
        case ISP_CONTRAST_HIGH:			 
			
			break;
        case ISP_CONTRAST_MIDDLE: 
        default:
			 
			break;
        //default:
		//	break;
    }
    SENSORDB("[OV2686]exit OV2686_set_contrast function:\n ");
    return;
}

void OV2686_set_brightness(UINT16 para)
{
    SENSORDB("[OV2686]CONTROLFLOW enter OV2686_set_brightness function:\n ");
	//return;
    switch (para)
    {
        case ISP_BRIGHT_LOW:
		
		break;
        case ISP_BRIGHT_HIGH:
		
			break;
        case ISP_BRIGHT_MIDDLE:
        default:
		
		break;
		
    }
    SENSORDB("[OV2686]exit OV2686_set_brightness function:\n ");
    return;
}
void OV2686_set_saturation(UINT16 para)
{
	SENSORDB("[OV2686]CONTROLFLOW enter OV2686_set_saturation function:\n ");
    switch (para)
    {
        case ISP_SAT_HIGH:
			
			break;
        case ISP_SAT_LOW:
			
			break;
        case ISP_SAT_MIDDLE:
        default:
			
			break;
		//	return KAL_FALSE;
		//	break;
    }
	SENSORDB("[OV2686]exit OV2686_set_saturation function:\n ");
     return;
}
void OV2686_set_scene_mode(UINT16 para)
{
	SENSORDB("[OV2686]CONTROLFLOW enter OV2686_set_scene_mode function:\n ");
	SENSORDB("[OV2686] OV2686_set_scene_mode=%d",para);	
	spin_lock(&OV2686_drv_lock);
	OV2686_SensorDriver.sceneMode=para;
	spin_unlock(&OV2686_drv_lock);
    switch (para)
    { 

		case SCENE_MODE_NIGHTSCENE:
          	OV2686_night_mode(KAL_TRUE); 
			break;
        case SCENE_MODE_PORTRAIT:
			/*Night mode disable*/
          	OV2686_night_mode(KAL_FALSE); 
			
            break;
        case SCENE_MODE_LANDSCAPE:
			/*Night mode disable*/
          	OV2686_night_mode(KAL_FALSE); 
			
             break;
        case SCENE_MODE_SUNSET:
			/*Night mode disable*/
          	OV2686_night_mode(KAL_FALSE); 
			
	 
            break;
        case SCENE_MODE_SPORTS:
             /*Night mode disable*/
          	OV2686_night_mode(KAL_FALSE); 
			
	 
            break;
        case SCENE_MODE_BEACH:
        case SCENE_MODE_SNOW:
          	OV2686_night_mode(KAL_FALSE); 			
			
            break;
        case SCENE_MODE_HDR:
        //    if (1 == OV2686_SensorDriver.manualAEStart)
            {
                OV2686_set_AE_mode(KAL_TRUE);//Manual AE disable
                spin_lock(&OV2686_drv_lock);
            	//OV2686_SensorDriver.manualAEStart = 0;
                //OV2686_SensorDriver.currentExposureTime = 0;
                //OV2686_SensorDriver.currentAxDGain = 0;
				spin_unlock(&OV2686_drv_lock);
            }
            break;
        case SCENE_MODE_OFF:
        default:
	
          	OV2686_night_mode(KAL_FALSE); 
			break;
		//	return KAL_FALSE;
        //    break;
    }
	SENSORDB("[OV2686]exit OV2686_set_scene_mode function:\n ");
	//if( OV2686_SensorDriver.sceneMode != SCENE_MODE_NIGHTSCENE) && ( OV2686_SensorDriver.sceneMode != SCENE_MODE_NIGHTSCENE)
	return;
}
void OV2686_set_iso(UINT16 para)
{
    spin_lock(&OV2686_drv_lock);
    OV2686_SensorDriver.isoSpeed = para;
    spin_unlock(&OV2686_drv_lock);   

	SENSORDB("[OV2686]CONTROLFLOW OV2686_set_iso:\n ");

    switch (para)
	{
        case AE_ISO_100:
             //ISO 100
			OV2686_write_cmos_sensor(0x3a13,0x38);
			break;
        case AE_ISO_200:
             //ISO 200
			 OV2686_write_cmos_sensor(0x3a13,0x58);
			break;
        case AE_ISO_400:
             //ISO 400
			OV2686_write_cmos_sensor(0x3a13,0xf8);
			break;
		case AE_ISO_AUTO:
			default:

			break;
	}
    return;
}

UINT32 OV2686YUVSetVideoMode(UINT16 u2FrameRate)    // lanking add
{
  
		    spin_lock(&OV2686_drv_lock);
        OV2686_MPEG4_encode_mode = KAL_TRUE;
    spin_unlock(&OV2686_drv_lock);
	SENSORDB("[OV2686]CONTROLFLOW OV2686 Frame Rate= %d\n", u2FrameRate); 
    if (u2FrameRate == 30)
	{
      OV2686_write_cmos_sensor(0x3086, 0x01);

      OV2686_write_cmos_sensor(0x380c, 0x06);
      OV2686_write_cmos_sensor(0x380d, 0xac);
      OV2686_write_cmos_sensor(0x380e, 0x02);
      OV2686_write_cmos_sensor(0x380f, 0x84);
      
      OV2686_write_cmos_sensor(0x3820, 0xc6); //modified by zhangxinghong
      OV2686_write_cmos_sensor(0x3821, 0x05);
      
      OV2686_write_cmos_sensor(0x3a06, 0x00);
      OV2686_write_cmos_sensor(0x3a07, 0xc2);
      OV2686_write_cmos_sensor(0x3a08, 0x00);
      OV2686_write_cmos_sensor(0x3a09, 0xa1);
      
      OV2686_write_cmos_sensor(0x3a0e, 0x02);
      OV2686_write_cmos_sensor(0x3a0f, 0x46);
      OV2686_write_cmos_sensor(0x3a10, 0x02);
      OV2686_write_cmos_sensor(0x3a11, 0x84);
      
      OV2686_write_cmos_sensor(0x3a00, 0x41);
      OV2686_write_cmos_sensor(0x3a0a, 0x02);
      OV2686_write_cmos_sensor(0x3a0b, 0x84);
      OV2686_write_cmos_sensor(0x3a0c, 0x02);
      OV2686_write_cmos_sensor(0x3a0d, 0x84);
	}
  else if (u2FrameRate == 15)   
	{
      OV2686_write_cmos_sensor(0x3086, 0x03);

      OV2686_write_cmos_sensor(0x380c, 0x06);
      OV2686_write_cmos_sensor(0x380d, 0xac);
      OV2686_write_cmos_sensor(0x380e, 0x02);
      OV2686_write_cmos_sensor(0x380f, 0x84);
      
      OV2686_write_cmos_sensor(0x3820, 0xc6); //modified by zhangxinghong
      OV2686_write_cmos_sensor(0x3821, 0x05);
      
      OV2686_write_cmos_sensor(0x3a06, 0x00);
      OV2686_write_cmos_sensor(0x3a07, 0x61);
      OV2686_write_cmos_sensor(0x3a08, 0x00);
      OV2686_write_cmos_sensor(0x3a09, 0x51);
      
      OV2686_write_cmos_sensor(0x3a0e, 0x02);
      OV2686_write_cmos_sensor(0x3a0f, 0x46);
      OV2686_write_cmos_sensor(0x3a10, 0x02);
      OV2686_write_cmos_sensor(0x3a11, 0x84);
      
      OV2686_write_cmos_sensor(0x3a00, 0x41);
      OV2686_write_cmos_sensor(0x3a0a, 0x02);
      OV2686_write_cmos_sensor(0x3a0b, 0x84);
      OV2686_write_cmos_sensor(0x3a0c, 0x02);
      OV2686_write_cmos_sensor(0x3a0d, 0x84); 
	}
  else 
  {
    // printk("Wrong frame rate setting \n");
  }   
	mDELAY(30);

  return TRUE;

}


void OV2686_SetMaxMinFps(UINT32 u2MinFrameRate, UINT32 u2MaxFrameRate)
{
	SENSORDB("[OV2686]CONTROLFLOW enter OV2686_SetMaxMinFps:\n ");
	SENSORDB("OV2686_SetMaxMinFps+ :FrameRate= %d %d\n",u2MinFrameRate,u2MaxFrameRate);
	spin_lock(&OV2686_drv_lock);
	MINFramerate = u2MinFrameRate;
	MAXFramerate = u2MaxFrameRate;
	
	spin_unlock(&OV2686_drv_lock);
	return;
}


UINT32 OV2686YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
    switch (iCmd) {
    case FID_AWB_MODE:
        OV2686_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        OV2686_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        OV2686_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
        OV2686_set_param_banding(iPara);
        break;
	case FID_SCENE_MODE:
		OV2686_night_mode(iPara);
		break;
		
    default:
        break;
    }
	
		switch (iCmd) {
			case FID_SCENE_MODE:
				OV2686_set_scene_mode(iPara);
				break;		
			case FID_AWB_MODE:
					OV2686_set_param_wb(iPara);
				  break;
			case FID_COLOR_EFFECT:				
					OV2686_set_param_effect(iPara);
				  break;
			case FID_AE_EV:   
					OV2686_set_param_exposure(iPara);
				break;
			case FID_AE_FLICKER:					
					OV2686_set_param_banding(iPara);
				  break;
			case FID_AE_SCENE_MODE: 
				if (iPara == AE_MODE_OFF) 
				{
					spin_lock(&OV2686_drv_lock);
					OV2686_SensorDriver.AE_ENABLE = KAL_FALSE; 
					spin_unlock(&OV2686_drv_lock);
				}
				else 
				{
					spin_lock(&OV2686_drv_lock);
					OV2686_SensorDriver.AE_ENABLE = KAL_TRUE; 
					spin_unlock(&OV2686_drv_lock);
				}
				OV2686_set_AE_mode(OV2686_SensorDriver.AE_ENABLE);
			break; 
			case FID_ISP_CONTRAST:
				OV2686_set_contrast(iPara);
				break;
			case FID_ISP_BRIGHT:
				OV2686_set_brightness(iPara);
				break;
			case FID_ISP_SAT:
				OV2686_set_saturation(iPara);
			break; 
		case FID_ZOOM_FACTOR:
				SENSORDB("FID_ZOOM_FACTOR:%d\n", iPara);		
				spin_lock(&OV2686_drv_lock);
				OV2686_zoom_factor = iPara; 
				spin_unlock(&OV2686_drv_lock);
				break; 
			case FID_AE_ISO:
				OV2686_set_iso(iPara);
				break;
#if 0 //afc
			case FID_AF_MODE:
				 OV2686_set_param_afmode(iPara);
						break;	   
#endif            
		  default:
					  break;
		}

    return TRUE;
} /* OV2686YUVSensorSetting */
void OV2686GetExifInfo(uintptr_t exifAddr)
{
	SENSORDB("[OV2686]enter OV2686GetExifInfo function\n");
	SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
	pExifInfo->FNumber = 20;
	pExifInfo->AEISOSpeed = OV2686_SensorDriver.isoSpeed;
	pExifInfo->FlashLightTimeus = 0;
	pExifInfo->RealISOValue = OV2686_SensorDriver.isoSpeed;
	SENSORDB("[OV2686]exit OV2686GetExifInfo function\n");
}

static void OV2686SetDummy(kal_uint32 dummy_pixels, kal_uint32 dummy_lines)
{
/*	SENSORDB("[OV2686]enter OV2686SetDummy function:\n ");
	if (OV2686_SensorDriver.IsPVmode)	
	{
		dummy_pixels = dummy_pixels+OV2686_PV_PERIOD_PIXEL_NUMS; 
		OV2686_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));		   
		OV2686_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
  
		dummy_lines= dummy_lines+OV2686_PV_PERIOD_LINE_NUMS; 
		OV2686_write_cmos_sensor(0x380F,(dummy_lines&0xFF));	   
		OV2686_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
	} 
	else
	{
		dummy_pixels = dummy_pixels+OV2686_FULL_PERIOD_PIXEL_NUMS; 
		OV2686_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));		   
		OV2686_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
  
		dummy_lines= dummy_lines+OV2686_FULL_PERIOD_LINE_NUMS; 
		OV2686_write_cmos_sensor(0x380F,(dummy_lines&0xFF));	   
		OV2686_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
	} 
	SENSORDB("[OV2686]exit OV2686SetDummy function:\n ");*/
}	 /* OV2686_set_dummy */


/**************************/
static void OV2686GetEvAwbRef(uintptr_t pSensorAEAWBRefStruct)
{
	PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
	Ref->SensorAERef.AeRefLV05Shutter=0x170c;
	Ref->SensorAERef.AeRefLV05Gain=0x30;
	Ref->SensorAERef.AeRefLV13Shutter=0x24e;
	Ref->SensorAERef.AeRefLV13Gain=0x10;
	Ref->SensorAwbGainRef.AwbRefD65Rgain=0x610;
	Ref->SensorAwbGainRef.AwbRefD65Bgain=0x448;
	Ref->SensorAwbGainRef.AwbRefCWFRgain=0x4e0;
	Ref->SensorAwbGainRef.AwbRefCWFBgain=0x5a0;	
}

static void OV2686GetCurAeAwbInfo(uintptr_t pSensorAEAWBCurStruct)
{

	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	Info->SensorAECur.AeCurShutter=100;//OV2686ReadShutter();
	Info->SensorAECur.AeCurGain=10;//OV2686ReadSensorGain() ;
	Info->SensorAwbGainCur.AwbCurRgain=10;//((OV2686YUV_read_cmos_sensor(0x3401)&&0xff)+((OV2686YUV_read_cmos_sensor(0x3400)&&0xff)*256));
	Info->SensorAwbGainCur.AwbCurBgain=10;//((OV2686YUV_read_cmos_sensor(0x3405)&&0xff)+((OV2686YUV_read_cmos_sensor(0x3404)&&0xff)*256));

}
UINT32 OV2686MaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
	{
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		SENSORDB("OV2686MaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		SENSORDB("[OV2686]enter OV2686MaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 480/10;
				lineLength = SVGA_PERIOD_PIXEL_NUMS;
				frameHeight = ( pclk)/frameRate/lineLength;
				dummyLine = frameHeight - SVGA_PERIOD_LINE_NUMS;
				if(dummyLine<0)
					dummyLine = 0;
				
				spin_lock(&OV2686_drv_lock);
				//OV2686_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
				OV2686_dummy_lines = dummyLine;
				spin_unlock(&OV2686_drv_lock);
				OV2686SetDummy(OV2686_dummy_lines, OV2686_dummy_pixels);			
				break;			
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 480/10;
				lineLength = SVGA_PERIOD_PIXEL_NUMS;
				frameHeight = ( pclk)/frameRate/lineLength;
				dummyLine = frameHeight - SVGA_PERIOD_LINE_NUMS;
				if(dummyLine<0)
					dummyLine = 0;
				
				spin_lock(&OV2686_drv_lock);
				//OV2686_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
				OV2686_dummy_lines = dummyLine;
				spin_unlock(&OV2686_drv_lock);
				OV2686SetDummy(OV2686_dummy_lines, OV2686_dummy_pixels);		
				break;			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:	
				pclk = 480/10;
				lineLength = FULL_PERIOD_PIXEL_NUMS;
				frameHeight = ( pclk)/frameRate/lineLength;
				dummyLine = frameHeight - FULL_PERIOD_LINE_NUMS;
				if(dummyLine<0)
					dummyLine = 0;
				
				spin_lock(&OV2686_drv_lock);
				//OV2686_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
				OV2686_dummy_lines = dummyLine;
				spin_unlock(&OV2686_drv_lock);
				OV2686SetDummy(OV2686_dummy_lines, OV2686_dummy_pixels);				
				break;		
			case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
				break;		
			default:
				break;
		}	
		SENSORDB("[OV2686]exit OV2686MaxFramerateByScenario function:\n ");
		return ERROR_NONE;
	}
UINT32 OV2686GetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	SENSORDB("[OV2686]enter OV2686GetDefaultFramerateByScenario function:\n ");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 300;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			 *pframeRate = 300;
			break;
	}
	SENSORDB("[OV2686]exit OV2686GetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}
void OV2686_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	SENSORDB("[OV2686]enter OV2686_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	SENSORDB("[OV2686]OV2686_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	SENSORDB("[OV2686]exit OV2686_get_AEAWB_lock function:\n ");
}
void OV2686_GetDelayInfo(uintptr_t delayAddr)
{
	SENSORDB("[OV2686]enter OV2686_GetDelayInfo function:\n ");
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay=5;
	pDelayInfo->EffectDelay=4;
	pDelayInfo->AwbDelay=3;
	
	//pDelayInfo->FlickerDelayFrame=3;
	pDelayInfo->AFSwitchDelayFrame=50;
	SENSORDB("[OV2686]exit OV2686_GetDelayInfo function:\n ");
}
void OV2686_AutoTestCmd(UINT32 *cmd,UINT32 *para)
{
	SENSORDB("[OV2686]enter OV2686_AutoTestCmd function:\n ");
	switch(*cmd)
	{
		case YUV_AUTOTEST_SET_SHADDING:
			SENSORDB("YUV_AUTOTEST_SET_SHADDING:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAMMA:
			SENSORDB("YUV_AUTOTEST_SET_GAMMA:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_AE:
			SENSORDB("YUV_AUTOTEST_SET_AE:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_SHUTTER:
			SENSORDB("YUV_AUTOTEST_SET_SHUTTER:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAIN:
			SENSORDB("YUV_AUTOTEST_SET_GAIN:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_GET_SHUTTER_RANGE:
			//*para=8228;
			break;
		default:
			SENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
			break;	
	}
	SENSORDB("[OV2686]exit OV2686_AutoTestCmd function:\n ");
}
UINT32 OV2686SetTestPatternMode(kal_bool bEnable)
{
	SENSORDB("[OV2686_OV2686SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);

if(bEnable)	
		OV2686_write_cmos_sensor(0x5080,0x80);		
	else	
		OV2686_write_cmos_sensor(0x5080,0x00);		

	
	return ERROR_NONE;
}

void OV2686_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
	SENSORDB("[OV2686]enter ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          spin_lock(&OV2686_drv_lock);
          OV2686_SensorDriver.userAskAeLock = TRUE;
          spin_unlock(&OV2686_drv_lock);
          OV2686_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
          spin_lock(&OV2686_drv_lock);
          OV2686_SensorDriver.userAskAeLock = FALSE;
          spin_unlock(&OV2686_drv_lock);
          OV2686_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          spin_lock(&OV2686_drv_lock);
          OV2686_SensorDriver.userAskAwbLock = TRUE;
          spin_unlock(&OV2686_drv_lock);
      //    OV2686_awb_enable(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          spin_lock(&OV2686_drv_lock);
          OV2686_SensorDriver.userAskAwbLock = FALSE;
          spin_unlock(&OV2686_drv_lock);
      //    OV2686_awb_enable(KAL_TRUE);
      break;
      default:
      	break;
   }
   SENSORDB("[OV2686]exit ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   return;
}

UINT32 OV2686FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	
	unsigned long long *feature_data=(unsigned long long *) pFeaturePara;
	unsigned long long *feature_return_para=(unsigned long long *) pFeaturePara;

	
	UINT32 Tony_Temp1 = 0;
	UINT32 Tony_Temp2 = 0;
	Tony_Temp1 = pFeaturePara[0];
	Tony_Temp2 = pFeaturePara[1];
	//SENSORDB("[OV2686]CONTROLFLOW [OV2686FeatureControl]feature id=%d \n",FeatureId);
	SENSORDB("[OV2686] [OV2686FeatureControl]feature id=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=FULL_PERIOD_PIXEL_NUMS;//VGA_PERIOD_PIXEL_NUMS;
			*pFeatureReturnPara16=FULL_PERIOD_LINE_NUMS;//VGA_PERIOD_LINE_NUMS;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(OV2686_CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=FULL_PERIOD_PIXEL_NUMS + OV2686_dummy_pixels;
					*pFeatureReturnPara16=FULL_PERIOD_LINE_NUMS + OV2686_dummy_lines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=SVGA_PERIOD_PIXEL_NUMS + OV2686_dummy_pixels;
					*pFeatureReturnPara16=SVGA_PERIOD_LINE_NUMS + OV2686_dummy_lines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV2686_CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV2686_SensorDriver.PreviewPclk * 1000 *1000;  //unit: Hz 			
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV2686_SensorDriver.PreviewPclk * 1000 *1000;  //unit: Hz
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			OV2686GetExifInfo((uintptr_t)*feature_data);//(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			OV2686_night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			OV2686_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = OV2686_read_cmos_sensor(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &OV2686SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
			*pFeatureReturnPara32++=0;
			*pFeatureParaLen=4;    
			break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:			 
			OV2686SetTestPatternMode((BOOL)*feature_data);			
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			OV2686GetSensorID(pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=OV2686_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			OV2686YUVSensorSetting((FEATURE_ID)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
			OV2686_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*feature_data);
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			OV2686YUVSetVideoMode(*feature_data);
			break; 
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			OV2686GetEvAwbRef(*feature_data);
			break;
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			OV2686GetCurAeAwbInfo(*feature_data); 		
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV2686MaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));//((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV2686GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));//((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
		//	OV2686_get_AEAWB_lock((MUINT32 *)(uintptr_t)(*(feature_data)), (MUINT32 *)(uintptr_t)(*(feature_data+1)));//(*pFeatureData32, *(pFeatureData32+1));
			OV2686_get_AEAWB_lock((uintptr_t)(*feature_data),(uintptr_t)(*(feature_data+1)));//(*pFeatureData32, *(pFeatureData32+1));

			break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
			OV2686_GetDelayInfo((uintptr_t)*feature_data);//(*pFeatureData32);
			break;
		case SENSOR_FEATURE_AUTOTEST_CMD:
			SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
			OV2686_AutoTestCmd((uintptr_t)(*feature_data),(uintptr_t)(*(feature_data+1)));//(*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_SET_MIN_MAX_FPS:
			
			SENSORDB("SENSOR_FEATURE_SET_MIN_MAX_FPS:[%d,%d]\n",*pFeatureData32,*(pFeatureData32+1));
			OV2686_SetMaxMinFps((UINT32)*feature_data, (UINT32)*(feature_data+1));
			break;


		//AF
		case SENSOR_FEATURE_INITIALIZE_AF:			 
			 break;
		case SENSOR_FEATURE_MOVE_FOCUS_LENS:
		//	  OV2686_FOCUS_Move_to(*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_AF_STATUS:
		//	  OV2686_FOCUS_OVT_AFC_Get_AF_Status(pFeatureReturnPara32); 		   
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_AF_INF:
		//	  OV2686_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
			*pFeatureParaLen=4; 		   
			break;
		case SENSOR_FEATURE_GET_AF_MACRO:
		//	  OV2686_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
			*pFeatureParaLen=4; 		   
			break;
		case SENSOR_FEATURE_CONSTANT_AF:
		//	OV2686_FOCUS_OVT_AFC_Constant_Focus();
			 break;
		case SENSOR_FEATURE_SET_AF_WINDOW:		 
			//OV2686_FOCUS_Set_AF_Window(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			//OV2686_FOCUS_OVT_AFC_Single_Focus();
			break;	
		case SENSOR_FEATURE_CANCEL_AF:
			//OV2686_FOCUS_OVT_AFC_Cancel_Focus();
			break;					
		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			//OV2686_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);			
			*pFeatureParaLen=4;
			break;		  
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			//OV2686_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32); 		   
			*pFeatureParaLen=4;
			break;		  
		case SENSOR_FEATURE_SET_AE_WINDOW:
			SENSORDB("AE zone addr = 0x%x\n",*pFeatureData32);			
			//OV2686_FOCUS_Set_AE_Window(*pFeatureData32);
			break; 
		default:
			SENSORDB("OV2686FeatureControl:default \n");
			break;			
	}
	SENSORDB("[OV2686]exit OV2686FeatureControl function:\n ");
	return ERROR_NONE;
}	/* OV2686MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncOV2686YUV=
{
	OV2686Open,
	OV2686GetInfo,
	OV2686GetResolution,
	OV2686FeatureControl,
	OV2686Control,
	OV2686Close
};


UINT32 OV2686_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV2686YUV;
	return ERROR_NONE;
} /* SensorInit() */



