/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S5K3H7mipi_Sensor.c
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
//#include <linux/xlog.h>
#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k3h7yxmipiraw_Sensor.h"


#define PFX "s5k3h7yx_camera_sensor"
#define LOG_1 LOG_INF("s5k3h7yx,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2664*1500@30fps,888Mbps/lane; video 5328*3000@30fps,1390Mbps/lane; capture 16M@30fps,1390Mbps/lane\n")
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	printk(  PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOGE(format, args...)   printk(PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
#define SLOW_MOTION_120FPS
extern int kdSetI2CBusNum(u32 i2cBusNum);

#define S5K3H7Y_OTP_USE
#ifdef S5K3H7Y_OTP_USE
extern bool otp_sunny_update_wb();
//extern bool wb_gain_set();
//bool otp_flag = 0;
#endif

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = S5K3H7YX_SENSOR_ID,
	//.checksum_value = 0x1fb39e9e,
	.checksum_value = 0x9c198b8c,
	.pre = {
		.pclk = 280000000,				//record different mode's pclk
		.linelength = 3688,				//record different mode's linelength
		.framelength =2530, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,//85unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 280000000,
		.linelength =3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width =3264,//5334,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 300,
		},
	.cap1 = {
		.pclk = 280000000,
		.linelength =3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width =3264,//5334,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 300,	
	},
	.normal_video = {
		.pclk = 280000000,
		.linelength = 3688,
		.framelength = 2530,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//5334,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 300,
	},
	
	.hs_video = {
		.pclk = 280000000,
		.linelength = 3560,
		.framelength = 654,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 800,		//record different mode's width of grabwindow
		.grabwindow_height = 600,		//record different mode's height of grabwindow
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 1200,
	},

	.slim_video = {
		.pclk = 280000000,
		.linelength = 3088,
		.framelength = 766,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1200,//1280,
		.grabwindow_height =700,// 720,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns
		.max_framerate = 300,
	},
	.margin = 16,
	.min_shutter = 3,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3, 
	.pre_delay_frame = 3,  
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0x5A, 0xff},
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 30,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x5A,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{
	{ 3280, 2464,	  0,   0, 3280, 2464, 1634,  1224, 0000, 0000, 1634, 1224,0,   0, 1600,  1200}, // Preview
	{ 3280, 2464,	  0,   0, 3280, 2464, 3264, 2448, 0000, 0000, 3264, 2448,	 0,   0, 3200, 2400}, // capture
	{ 3280, 2464,	  0,   0, 3280, 2464, 3264, 2448, 0000, 0000, 3264, 2448,	 0,   0,3200,2400}, // video
	{ 3280, 2464,	  0,   0, 3280, 2464, 816, 612, 0000, 0000, 816, 612,	  0,   0, 800, 600}, //hight speed video
	{ 3280, 2464,	  0,   0, 3280, 2464, 1280,  720, 0000, 0000, 1280,  720,	 0,   0, 1200,  700}// slim video
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	//kdSetI2CBusNum(0);
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	//kdSetI2CBusNum(0);
	iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	//kdSetI2CBusNum(0);
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	//kdSetI2CBusNum(0);
	iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

inline void S5K3H7Y_wordwrite_cmos_sensor(u16 addr, kal_uint16 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,  (char)(para >> 8),	(char)(para & 0xFF) };
	//kdSetI2CBusNum(0);
	iWriteRegI2C(puSendCmd , 4,imgsensor.i2c_write_id);
}

inline void S5K3H7Y_bytewrite_cmos_sensor(u16 addr, kal_uint16 para)
{
	char puSendCmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	//kdSetI2CBusNum(0);
	iWriteRegI2C(puSendCmd , 3,imgsensor.i2c_write_id);
}

unsigned char S5K3H7Y_byteread_cmos_sensor(u16 addr)
{
	unsigned char get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	//kdSetI2CBusNum(0);
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}

static void set_dummy(void)
{
    LOG_INF("currently the mode is %d,dummyline = %d, dummypixels = %d ", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0342, imgsensor.line_length);
    write_cmos_sensor_8(0x0104, 0x00);
} /*set_dummy*/

static kal_uint32 return_sensor_id(void)
{
	return (read_cmos_sensor(0x0000));
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
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


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
#if 1 //auto flicker: 1 enable, 0 disable
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
	} else {
		// Extend frame length
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length);
		write_cmos_sensor_8(0x0104, 0x00);
	}
#endif
	// Update Shutter
	//write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor(0x0340, imgsensor.frame_length);
	write_cmos_sensor(0x0202, shutter);
	//write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("Currently camera mode is %d,shutter is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.shutter,imgsensor.frame_length,imgsensor.line_length);
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
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
	LOG_INF("Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0;

	reg_gain = gain/2;
	return (kal_uint16)reg_gain;
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
	
	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X  */
	/* [4:9] = M meams M X 	  */
	/* Total gain = M + N /16 X   */

	//
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
	LOG_INF("gain = %d , reg_gain = 0x%x,shutter=%d,the result of gain*shutter is %d ", gain, reg_gain,imgsensor.shutter,gain*(imgsensor.shutter));

	//write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0204,(reg_gain>>8));
	write_cmos_sensor_8(0x0205,(reg_gain&0xff));
	//write_cmos_sensor_8(0x0104, 0x00);

	return gain;
}	

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
#if 1
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length = imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


		// Extend frame length first
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor(0x0340, imgsensor.frame_length);

		//write_cmos_sensor(0x0202, se);
		//write_cmos_sensor(0x021e,le);
		write_cmos_sensor(0x602A, 0x021e);
		write_cmos_sensor(0x6f12, le);
		write_cmos_sensor(0x602A, 0x0202);
		write_cmos_sensor(0x6f12, se);
		write_cmos_sensor_8(0x0104, 0x00);
		LOG_INF("iHDR:imgsensor.frame_length=%d\n",imgsensor.frame_length);
		set_gain(gain);
	}

#endif
}


static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d", image_mirror);

	/********************************************************
	*
	*   0x3820[2] ISP Vertical flip
	*   0x3820[1] Sensor Vertical flip
	*
	*   0x3821[2] ISP Horizontal mirror
	*   0x3821[1] Sensor Horizontal mirror
	*
	*   ISP and Sensor flip or mirror register bit should be the same!!
	*
	********************************************************/
	spin_lock(&imgsensor_drv_lock);
	imgsensor.mirror= image_mirror;
	spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {

	case IMAGE_NORMAL:
		write_cmos_sensor_8(0x0101, 0x00); // Gr
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor_8(0x0101, 0x01);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor_8(0x0101, 0x02);
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor_8(0x0101, 0x03); //Gb
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
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

static void sensor_init(void)
{
	LOG_INF("enter\n");

	// SENSORDB("enter\n");
	//1600x1200
	write_cmos_sensor(0x6010, 0x0001);   // Reset
	mdelay(10);//; delay(10ms)
	//write_cmos_sensor_8(0x0101, 0x03);
	write_cmos_sensor(0x6028, 0x7000);
	write_cmos_sensor(0x602A, 0x1750);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x6DFB);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x6FFB);
	write_cmos_sensor(0x6F12, 0x10BC);
	write_cmos_sensor(0x6F12, 0x08BC);
	write_cmos_sensor(0x6F12, 0x1847);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0x3840);
	write_cmos_sensor(0x6F12, 0x10E3);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0x9F15);
	write_cmos_sensor(0x6F12, 0x6006);
	write_cmos_sensor(0x6F12, 0x9015);
	write_cmos_sensor(0x6F12, 0x2400);
	write_cmos_sensor(0x6F12, 0x5013);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x000A);
	write_cmos_sensor(0x6F12, 0x0A00);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x5446);
	write_cmos_sensor(0x6F12, 0xD4E5);
	write_cmos_sensor(0x6F12, 0xD700);
	write_cmos_sensor(0x6F12, 0x50E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x001A);
	write_cmos_sensor(0x6F12, 0x0600);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x2F00);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0xA901);
	write_cmos_sensor(0x6F12, 0xDDE5);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0xA001);
	write_cmos_sensor(0x6F12, 0xC4E5);
	write_cmos_sensor(0x6F12, 0xD700);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0500);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x3840);
	write_cmos_sensor(0x6F12, 0x00EA);
	write_cmos_sensor(0x6F12, 0xA501);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x2016);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x012C);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0x8030);
	write_cmos_sensor(0x6F12, 0x83E2);
	write_cmos_sensor(0x6F12, 0x013C);
	write_cmos_sensor(0x6F12, 0x80E2);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x50E3);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0xC3E1);
	write_cmos_sensor(0x6F12, 0xBE28);
	write_cmos_sensor(0x6F12, 0xFFBA);
	write_cmos_sensor(0x6F12, 0xF9FF);
	write_cmos_sensor(0x6F12, 0x2FE1);
	write_cmos_sensor(0x6F12, 0x1EFF);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0x7040);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xF4C5);
	write_cmos_sensor(0x6F12, 0xDCE5);
	write_cmos_sensor(0x6F12, 0x1021);
	write_cmos_sensor(0x6F12, 0x52E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x001A);
	write_cmos_sensor(0x6F12, 0x0A00);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xE8E5);
	write_cmos_sensor(0x6F12, 0x8CE0);
	write_cmos_sensor(0x6F12, 0x0231);
	write_cmos_sensor(0x6F12, 0x8EE0);
	write_cmos_sensor(0x6F12, 0x8250);
	write_cmos_sensor(0x6F12, 0xD5E1);
	write_cmos_sensor(0x6F12, 0xB050);
	write_cmos_sensor(0x6F12, 0x93E5);
	write_cmos_sensor(0x6F12, 0xD840);
	write_cmos_sensor(0x6F12, 0x82E2);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x04E0);
	write_cmos_sensor(0x6F12, 0x9504);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x2444);
	write_cmos_sensor(0x6F12, 0x52E3);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x83E5);
	write_cmos_sensor(0x6F12, 0xD840);
	write_cmos_sensor(0x6F12, 0xFFBA);
	write_cmos_sensor(0x6F12, 0xF5FF);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x7040);
	write_cmos_sensor(0x6F12, 0x00EA);
	write_cmos_sensor(0x6F12, 0x8B01);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0x1040);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x8B01);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xB005);
	write_cmos_sensor(0x6F12, 0xD0E5);
	write_cmos_sensor(0x6F12, 0x7310);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x1040);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xA805);
	write_cmos_sensor(0x6F12, 0xFFEA);
	write_cmos_sensor(0x6F12, 0xE6FF);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xFF4F);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x8C45);
	write_cmos_sensor(0x6F12, 0x4DE2);
	write_cmos_sensor(0x6F12, 0xA4D0);
	write_cmos_sensor(0x6F12, 0xD4E1);
	write_cmos_sensor(0x6F12, 0xB20D);
	write_cmos_sensor(0x6F12, 0xD4E5);
	write_cmos_sensor(0x6F12, 0x9CA0);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0150);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x5800);
	write_cmos_sensor(0x6F12, 0xD4E1);
	write_cmos_sensor(0x6F12, 0xB40D);
	write_cmos_sensor(0x6F12, 0x5AE3);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x6F12, 0xA023);
	write_cmos_sensor(0x6F12, 0x10A0);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x5400);
	write_cmos_sensor(0x6F12, 0xD4E5);
	write_cmos_sensor(0x6F12, 0xDB00);
	write_cmos_sensor(0x6F12, 0xD4E5);
	write_cmos_sensor(0x6F12, 0xD710);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x2020);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x1500);
	write_cmos_sensor(0x6F12, 0x81E2);
	write_cmos_sensor(0x6F12, 0x0310);
	write_cmos_sensor(0x6F12, 0x01E2);
	write_cmos_sensor(0x6F12, 0xFF70);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0xA000);
	write_cmos_sensor(0x6F12, 0xCDE1);
	write_cmos_sensor(0x6F12, 0xBC07);
	write_cmos_sensor(0x6F12, 0xCDE1);
	write_cmos_sensor(0x6F12, 0xBC05);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x4C10);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x5010);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xF600);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x4800);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0xD4E5);
	write_cmos_sensor(0x6F12, 0xD910);
	write_cmos_sensor(0x6F12, 0xD0E5);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x4400);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x1500);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0xA00F);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0xC000);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x1815);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x6201);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x1015);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x1820);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x5E01);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x0425);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x92E5);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xDC04);
	write_cmos_sensor(0x6F12, 0xD2E5);
	write_cmos_sensor(0x6F12, 0x5921);
	write_cmos_sensor(0x6F12, 0x90E5);
	write_cmos_sensor(0x6F12, 0x4010);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xAC30);
	write_cmos_sensor(0x6F12, 0x82E0);
	write_cmos_sensor(0x6F12, 0x8221);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0x8210);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0x8310);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xFA30);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xBE04);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xF210);
	write_cmos_sensor(0x6F12, 0x60E2);
	write_cmos_sensor(0x6F12, 0x012C);
	write_cmos_sensor(0x6F12, 0x02E0);
	write_cmos_sensor(0x6F12, 0x9302);
	write_cmos_sensor(0x6F12, 0x20E0);
	write_cmos_sensor(0x6F12, 0x9120);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x4008);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x9884);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x5000);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x5410);
	write_cmos_sensor(0x6F12, 0x88E0);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xFC0B);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0150);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x9100);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0064);
	write_cmos_sensor(0x6F12, 0xB0E1);
	write_cmos_sensor(0x6F12, 0x4668);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA000);
	write_cmos_sensor(0x6F12, 0xA053);
	write_cmos_sensor(0x6F12, 0x0210);
	write_cmos_sensor(0x6F12, 0xE043);
	write_cmos_sensor(0x6F12, 0x0110);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x3F01);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0098);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x4998);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0140);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x05B0);
	write_cmos_sensor(0x6F12, 0x00EA);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x5C00);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x8450);
	write_cmos_sensor(0x6F12, 0x55E1);
	write_cmos_sensor(0x6F12, 0xF200);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA010);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x9600);
	write_cmos_sensor(0x6F12, 0x89E0);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x3301);
	write_cmos_sensor(0x6F12, 0x84E2);
	write_cmos_sensor(0x6F12, 0x0140);
	write_cmos_sensor(0x6F12, 0xC5E1);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0x54E1);
	write_cmos_sensor(0x6F12, 0x0A00);
	write_cmos_sensor(0x6F12, 0xFFDA);
	write_cmos_sensor(0x6F12, 0xF4FF);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0090);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x2004);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x5810);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x8900);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xFE09);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x9100);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0064);
	write_cmos_sensor(0x6F12, 0xB0E1);
	write_cmos_sensor(0x6F12, 0x4668);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA000);
	write_cmos_sensor(0x6F12, 0xA053);
	write_cmos_sensor(0x6F12, 0x0210);
	write_cmos_sensor(0x6F12, 0xE043);
	write_cmos_sensor(0x6F12, 0x0110);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x2301);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0088);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x4888);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0140);
	write_cmos_sensor(0x6F12, 0x00EA);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x7C00);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x8450);
	write_cmos_sensor(0x6F12, 0x55E1);
	write_cmos_sensor(0x6F12, 0xF200);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA010);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x9600);
	write_cmos_sensor(0x6F12, 0x88E0);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x1801);
	write_cmos_sensor(0x6F12, 0x84E2);
	write_cmos_sensor(0x6F12, 0x0140);
	write_cmos_sensor(0x6F12, 0xC5E1);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0x54E1);
	write_cmos_sensor(0x6F12, 0x0A00);
	write_cmos_sensor(0x6F12, 0xFFDA);
	write_cmos_sensor(0x6F12, 0xF4FF);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0860);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0850);
	write_cmos_sensor(0x6F12, 0x00EA);
	write_cmos_sensor(0x6F12, 0x2300);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x00EA);
	write_cmos_sensor(0x6F12, 0x1E00);
	write_cmos_sensor(0x6F12, 0x45E0);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x7C10);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x5C20);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0x8410);
	write_cmos_sensor(0x6F12, 0x82E0);
	write_cmos_sensor(0x6F12, 0x8000);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xF010);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xF000);
	write_cmos_sensor(0x6F12, 0x0BE0);
	write_cmos_sensor(0x6F12, 0x9100);
	write_cmos_sensor(0x6F12, 0x5BE3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA000);
	write_cmos_sensor(0x6F12, 0xA0A3);
	write_cmos_sensor(0x6F12, 0x0210);
	write_cmos_sensor(0x6F12, 0xE0B3);
	write_cmos_sensor(0x6F12, 0x0110);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA010);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x0B00);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0xFE00);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA410);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0120);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0x8610);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xF010);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x9100);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0210);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x1117);
	write_cmos_sensor(0x6F12, 0x50E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x1227);
	write_cmos_sensor(0x6F12, 0x62B2);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0xF200);
	write_cmos_sensor(0x6F12, 0x88E0);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x6F12, 0x86E2);
	write_cmos_sensor(0x6F12, 0x0160);
	write_cmos_sensor(0x6F12, 0x84E2);
	write_cmos_sensor(0x6F12, 0x0140);
	write_cmos_sensor(0x6F12, 0x54E1);
	write_cmos_sensor(0x6F12, 0x0500);
	write_cmos_sensor(0x6F12, 0xFFDA);
	write_cmos_sensor(0x6F12, 0xDEFF);
	write_cmos_sensor(0x6F12, 0x85E2);
	write_cmos_sensor(0x6F12, 0x0150);
	write_cmos_sensor(0x6F12, 0x55E1);
	write_cmos_sensor(0x6F12, 0x0A00);
	write_cmos_sensor(0x6F12, 0xFFDA);
	write_cmos_sensor(0x6F12, 0xD9FF);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x021B);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x9800);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0x4014);
	write_cmos_sensor(0x6F12, 0x51E3);
	write_cmos_sensor(0x6F12, 0x020B);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x4C10);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xAC20);
	write_cmos_sensor(0x6F12, 0xA0A1);
	write_cmos_sensor(0x6F12, 0x4004);
	write_cmos_sensor(0x6F12, 0x80A2);
	write_cmos_sensor(0x6F12, 0x020B);
	write_cmos_sensor(0x6F12, 0x82E0);
	write_cmos_sensor(0x6F12, 0x0111);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA820);
	write_cmos_sensor(0x6F12, 0xA0B3);
	write_cmos_sensor(0x6F12, 0x020B);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x4008);
	write_cmos_sensor(0x6F12, 0x82E0);
	write_cmos_sensor(0x6F12, 0x8110);
	write_cmos_sensor(0x6F12, 0xC1E1);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x0410);
	write_cmos_sensor(0x6F12, 0x89E2);
	write_cmos_sensor(0x6F12, 0x0190);
	write_cmos_sensor(0x6F12, 0x50E1);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0xA0D1);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x4C00);
	write_cmos_sensor(0x6F12, 0x59E3);
	write_cmos_sensor(0x6F12, 0x0F00);
	write_cmos_sensor(0x6F12, 0x80E2);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x4C00);
	write_cmos_sensor(0x6F12, 0xFFBA);
	write_cmos_sensor(0x6F12, 0xA1FF);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x5000);
	write_cmos_sensor(0x6F12, 0x80E2);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x50E3);
	write_cmos_sensor(0x6F12, 0x0B00);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x5000);
	write_cmos_sensor(0x6F12, 0xFFBA);
	write_cmos_sensor(0x6F12, 0x7EFF);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xAC20);
	write_cmos_sensor(0x6F12, 0x50E3);
	write_cmos_sensor(0x6F12, 0x020A);
	write_cmos_sensor(0x6F12, 0xA0C1);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0xA0C1);
	write_cmos_sensor(0x6F12, 0xC01F);
	write_cmos_sensor(0x6F12, 0x80C0);
	write_cmos_sensor(0x6F12, 0xA109);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x8412);
	write_cmos_sensor(0x6F12, 0xA0D3);
	write_cmos_sensor(0x6F12, 0x010C);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0x8210);
	write_cmos_sensor(0x6F12, 0xA0C1);
	write_cmos_sensor(0x6F12, 0xC006);
	write_cmos_sensor(0x6F12, 0x8DE5);
	write_cmos_sensor(0x6F12, 0x9C10);
	write_cmos_sensor(0x6F12, 0xC1E1);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x9C10);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xF400);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xB010);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0050);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0xB800);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0088);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x4888);
	write_cmos_sensor(0x6F12, 0xC0E1);
	write_cmos_sensor(0x6F12, 0xB480);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x4800);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0060);
	write_cmos_sensor(0x6F12, 0x40E2);
	write_cmos_sensor(0x6F12, 0x029B);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xB010);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x8600);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xF000);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xF210);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x40E0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x07E0);
	write_cmos_sensor(0x6F12, 0x9000);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xB010);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x8400);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xF000);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xF010);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xF8C1);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0130);
	write_cmos_sensor(0x6F12, 0x40E0);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x02E0);
	write_cmos_sensor(0x6F12, 0x9000);
	write_cmos_sensor(0x6F12, 0xDCE5);
	write_cmos_sensor(0x6F12, 0xD800);
	write_cmos_sensor(0x6F12, 0x82E0);
	write_cmos_sensor(0x6F12, 0x0720);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x1310);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0xA11F);
	write_cmos_sensor(0x6F12, 0x82E0);
	write_cmos_sensor(0x6F12, 0xC110);
	write_cmos_sensor(0x6F12, 0xDCE5);
	write_cmos_sensor(0x6F12, 0xDA20);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x3110);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x1302);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x4030);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x44C0);
	write_cmos_sensor(0x6F12, 0x23E0);
	write_cmos_sensor(0x6F12, 0x9931);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x533C);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x40C0);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0xA00F);
	write_cmos_sensor(0x6F12, 0x21E0);
	write_cmos_sensor(0x6F12, 0x98C1);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x44C0);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x511C);
	write_cmos_sensor(0x6F12, 0x01E0);
	write_cmos_sensor(0x6F12, 0x9301);
	write_cmos_sensor(0x6F12, 0x81E0);
	write_cmos_sensor(0x6F12, 0xC000);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x50B2);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0x9C00);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xA820);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xB010);
	write_cmos_sensor(0x6F12, 0x9DE5);
	write_cmos_sensor(0x6F12, 0xAC00);
	write_cmos_sensor(0x6F12, 0x80E0);
	write_cmos_sensor(0x6F12, 0x0501);
	write_cmos_sensor(0x6F12, 0x82E0);
	write_cmos_sensor(0x6F12, 0x80A0);
	write_cmos_sensor(0x6F12, 0xDAE1);
	write_cmos_sensor(0x6F12, 0xF000);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x8300);
	write_cmos_sensor(0x6F12, 0x40E0);
	write_cmos_sensor(0x6F12, 0x0B00);
	write_cmos_sensor(0x6F12, 0x84E2);
	write_cmos_sensor(0x6F12, 0x0140);
	write_cmos_sensor(0x6F12, 0x54E3);
	write_cmos_sensor(0x6F12, 0x0F00);
	write_cmos_sensor(0x6F12, 0x85E2);
	write_cmos_sensor(0x6F12, 0x0150);
	write_cmos_sensor(0x6F12, 0xCAE1);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0xFFBA);
	write_cmos_sensor(0x6F12, 0xD3FF);
	write_cmos_sensor(0x6F12, 0x86E2);
	write_cmos_sensor(0x6F12, 0x0160);
	write_cmos_sensor(0x6F12, 0x56E3);
	write_cmos_sensor(0x6F12, 0x0B00);
	write_cmos_sensor(0x6F12, 0xFFBA);
	write_cmos_sensor(0x6F12, 0xC8FF);
	write_cmos_sensor(0x6F12, 0x8DE2);
	write_cmos_sensor(0x6F12, 0xB4D0);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF04F);
	write_cmos_sensor(0x6F12, 0x2FE1);
	write_cmos_sensor(0x6F12, 0x1EFF);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x7700);
	write_cmos_sensor(0x6F12, 0x50E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xBD08);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0xA003);
	write_cmos_sensor(0x6F12, 0x0010);
	write_cmos_sensor(0x6F12, 0xA003);
	write_cmos_sensor(0x6F12, 0x3800);
	write_cmos_sensor(0x6F12, 0x000A);
	write_cmos_sensor(0x6F12, 0x7400);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x4811);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xBA01);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xBC21);
	write_cmos_sensor(0x6F12, 0xD1E1);
	write_cmos_sensor(0x6F12, 0xBE11);
	write_cmos_sensor(0x6F12, 0x80E1);
	write_cmos_sensor(0x6F12, 0x0208);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x7000);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x3051);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x3001);
	write_cmos_sensor(0x6F12, 0xD5E1);
	write_cmos_sensor(0x6F12, 0xF030);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xBAEA);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xBCCA);
	write_cmos_sensor(0x6F12, 0xD5E1);
	write_cmos_sensor(0x6F12, 0xF220);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x930C);
	write_cmos_sensor(0x6F12, 0x42E0);
	write_cmos_sensor(0x6F12, 0x0360);
	write_cmos_sensor(0x6F12, 0x02E0);
	write_cmos_sensor(0x6F12, 0x9E02);
	write_cmos_sensor(0x6F12, 0x4CE0);
	write_cmos_sensor(0x6F12, 0x0E40);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0410);
	write_cmos_sensor(0x6F12, 0x40E0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x5C00);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0x10E3);
	write_cmos_sensor(0x6F12, 0x020C);
	write_cmos_sensor(0x6F12, 0xA011);
	write_cmos_sensor(0x6F12, 0x0700);
	write_cmos_sensor(0x6F12, 0x001B);
	write_cmos_sensor(0x6F12, 0x5E00);
	write_cmos_sensor(0x6F12, 0x56E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xE003);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x000A);
	write_cmos_sensor(0x6F12, 0x0300);
	write_cmos_sensor(0x6F12, 0x47E0);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x00E0);
	write_cmos_sensor(0x6F12, 0x9400);
	write_cmos_sensor(0x6F12, 0xA0E1);
	write_cmos_sensor(0x6F12, 0x0610);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x5500);
	write_cmos_sensor(0x6F12, 0xC5E1);
	write_cmos_sensor(0x6F12, 0xB400);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0xF041);
	write_cmos_sensor(0x6F12, 0x2FE1);
	write_cmos_sensor(0x6F12, 0x1EFF);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xC810);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xC800);
	write_cmos_sensor(0x6F12, 0x2DE9);
	write_cmos_sensor(0x6F12, 0x1040);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xC420);
	write_cmos_sensor(0x6F12, 0x80E5);
	write_cmos_sensor(0x6F12, 0x5010);
	write_cmos_sensor(0x6F12, 0x42E0);
	write_cmos_sensor(0x6F12, 0x0110);
	write_cmos_sensor(0x6F12, 0xC0E1);
	write_cmos_sensor(0x6F12, 0xB415);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xB800);
	write_cmos_sensor(0x6F12, 0x4FE2);
	write_cmos_sensor(0x6F12, 0xD410);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x4C00);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xB000);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0xB040);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x7020);
	write_cmos_sensor(0x6F12, 0x84E5);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x011C);
	write_cmos_sensor(0x6F12, 0x82E0);
	write_cmos_sensor(0x6F12, 0x8030);
	write_cmos_sensor(0x6F12, 0x80E2);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x50E3);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0xC3E1);
	write_cmos_sensor(0x6F12, 0xB010);
	write_cmos_sensor(0x6F12, 0xFF3A);
	write_cmos_sensor(0x6F12, 0xFAFF);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x8C00);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x9010);
	write_cmos_sensor(0x6F12, 0x84E5);
	write_cmos_sensor(0x6F12, 0x5C00);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x8400);
	write_cmos_sensor(0x6F12, 0x84E5);
	write_cmos_sensor(0x6F12, 0x2C00);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x8400);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x3A00);
	write_cmos_sensor(0x6F12, 0x4FE2);
	write_cmos_sensor(0x6F12, 0x650E);
	write_cmos_sensor(0x6F12, 0x84E5);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x9FE5);
	write_cmos_sensor(0x6F12, 0x4C00);
	write_cmos_sensor(0x6F12, 0xD0E1);
	write_cmos_sensor(0x6F12, 0xB012);
	write_cmos_sensor(0x6F12, 0x51E3);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x6F12, 0x009A);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x6F12, 0xA0E3);
	write_cmos_sensor(0x6F12, 0x090C);
	write_cmos_sensor(0x6F12, 0x00EB);
	write_cmos_sensor(0x6F12, 0x2C00);
	write_cmos_sensor(0x6F12, 0xFFEA);
	write_cmos_sensor(0x6F12, 0xFEFF);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x1040);
	write_cmos_sensor(0x6F12, 0x2FE1);
	write_cmos_sensor(0x6F12, 0x1EFF);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x7004);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xD005);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x5014);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xD41E);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x1013);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xB412);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x9C1E);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xBC1E);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x00D0);
	write_cmos_sensor(0x6F12, 0x0093);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x8012);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xC00B);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xE012);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xDC1E);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x7005);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x902D);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x90A6);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x4018);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xF804);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xDC17);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0x2418);
	write_cmos_sensor(0x6F12, 0x0070);
	write_cmos_sensor(0x6F12, 0xB417);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xC06A);
	write_cmos_sensor(0x6F12, 0x7847);
	write_cmos_sensor(0x6F12, 0xC046);
	write_cmos_sensor(0x6F12, 0xFFEA);
	write_cmos_sensor(0x6F12, 0xBFFF);
	write_cmos_sensor(0x6F12, 0x7847);
	write_cmos_sensor(0x6F12, 0xC046);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x6CCE);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x54C0);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x8448);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x146C);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x4C7E);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x8CDC);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x48DD);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x7C55);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x744C);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xE8DE);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x4045);
	write_cmos_sensor(0x6F12, 0x1FE5);
	write_cmos_sensor(0x6F12, 0x04F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0xE8CD);
	write_cmos_sensor(0x6F12, 0x80F9);
	write_cmos_sensor(0x6F12, 0x00FA);
	write_cmos_sensor(0x6F12, 0x00FB);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0x00FD);
	write_cmos_sensor(0x6F12, 0x00FE);
	write_cmos_sensor(0x6F12, 0x00FF);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0x0005);
	write_cmos_sensor(0x6F12, 0x0006);
	write_cmos_sensor(0x6F12, 0x8006);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x00FB);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0x00FD);
	write_cmos_sensor(0x6F12, 0x00FE);
	write_cmos_sensor(0x6F12, 0x00FF);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0x0005);
	write_cmos_sensor(0x6F12, 0x0000);

	//write_cmos_sensor(0x6028, 0xD000);
	write_cmos_sensor(0x38FA, 0x0030);
	write_cmos_sensor(0x38FC, 0x0030);
	write_cmos_sensor(0x0086, 0x01FF);
	write_cmos_sensor(0x6218, 0xF1D0);
	write_cmos_sensor(0x6214, 0xF9F0);
	write_cmos_sensor(0x6226, 0x0001);
	write_cmos_sensor(0xB0C0, 0x000C);
	write_cmos_sensor(0xF400, 0x0BBC);
	write_cmos_sensor(0xF616, 0x0004);
	write_cmos_sensor(0x6226, 0x0000);
	write_cmos_sensor(0x6218, 0xF9F0);
	write_cmos_sensor(0x6218, 0xF1D0);
	write_cmos_sensor(0x6214, 0xF9F0);
	write_cmos_sensor(0x6226, 0x0001);
	write_cmos_sensor(0xB0C0, 0x000C);
	write_cmos_sensor(0x6226, 0x0000);
	write_cmos_sensor(0x6218, 0xF9F0);
	write_cmos_sensor(0x38FA, 0x0030);
	write_cmos_sensor(0x38FC, 0x0030);
	write_cmos_sensor(0x32CE, 0x0060);
	write_cmos_sensor(0x32D0, 0x0024);
	write_cmos_sensor_8(0x0114, 0x03);
	write_cmos_sensor(0x030E, 0x00A5);
	write_cmos_sensor(0x0342, 0x0E68);
	write_cmos_sensor(0x0340, 0x09E2);
	write_cmos_sensor(0x0200, 0x0618);
	write_cmos_sensor(0x0202, 0x09C2);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor_8(0x3011, 0x02);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x12);
	write_cmos_sensor(0x034C, 0x0662);
	write_cmos_sensor(0x034E, 0x04C8);
	write_cmos_sensor(0x6028, 0xD000);
	write_cmos_sensor(0x602A, 0x012A);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x7077);
	write_cmos_sensor(0x6F12, 0x7777);
	//s5k3h7y.sensorBaseGain=64;

	write_cmos_sensor(0x0204, 0x0020); // X1
	write_cmos_sensor_8(0x0B05, 0x0001); // #smiaRegs_rw_isp_mapped_couplet_correct_enable
	write_cmos_sensor_8(0x0B00, 0x0000); // #smiaRegs_rw_isp_shading_correction_enable

	//CONFIGURATION REGISTERS

	write_cmos_sensor(0x31FE, 0xC004); // ash_uDecompressXgrid[0]
	write_cmos_sensor(0x3200, 0xC4F0); // ash_uDecompressXgrid[1]
	write_cmos_sensor(0x3202, 0xCEC8); // ash_uDecompressXgrid[2]
	write_cmos_sensor(0x3204, 0xD8A0); // ash_uDecompressXgrid[3]
	write_cmos_sensor(0x3206, 0xE278); // ash_uDecompressXgrid[4]
	write_cmos_sensor(0x3208, 0xEC50); // ash_uDecompressXgrid[5]
	write_cmos_sensor(0x320A, 0xF628); // ash_uDecompressXgrid[6]
	write_cmos_sensor(0x320C, 0x0000); // ash_uDecompressXgrid[7]
	write_cmos_sensor(0x320E, 0x09D8); // ash_uDecompressXgrid[8]
	write_cmos_sensor(0x3210, 0x13B0); // ash_uDecompressXgrid[9]
	write_cmos_sensor(0x3212, 0x1D88); // ash_uDecompressXgrid[10]
	write_cmos_sensor(0x3214, 0x2760); // ash_uDecompressXgrid[11]
	write_cmos_sensor(0x3216, 0x3138); // ash_uDecompressXgrid[12]
	write_cmos_sensor(0x3218, 0x3B10); // ash_uDecompressXgrid[13]
	write_cmos_sensor(0x321A, 0x3FFC); // ash_uDecompressXgrid[14]

	write_cmos_sensor(0x321C, 0xC004); // ash_uDecompressYgrid[0]
	write_cmos_sensor(0x321E, 0xCCD0); // ash_uDecompressYgrid[1]
	write_cmos_sensor(0x3220, 0xD99C); // ash_uDecompressYgrid[2]
	write_cmos_sensor(0x3222, 0xE668); // ash_uDecompressYgrid[3]
	write_cmos_sensor(0x3224, 0xF334); // ash_uDecompressYgrid[4]
	write_cmos_sensor(0x3226, 0x0000); // ash_uDecompressYgrid[5]
	write_cmos_sensor(0x3228, 0x0CCC); // ash_uDecompressYgrid[6]
	write_cmos_sensor(0x322A, 0x1998); // ash_uDecompressYgrid[7]
	write_cmos_sensor(0x322C, 0x2664); // ash_uDecompressYgrid[8]
	write_cmos_sensor(0x322E, 0x3330); // ash_uDecompressYgrid[9]
	write_cmos_sensor(0x3230, 0x3FFC); // ash_uDecompressYgrid[10]

	write_cmos_sensor(0x3232, 0x0100); // ash_uDecompressWidth
	write_cmos_sensor(0x3234, 0x0100); // ash_uDecompressHeight

	write_cmos_sensor_8(0x3237, 0x00); // ash_uDecompressScale          // 00 - the value for this register is read from NVM page #0 byte #47 bits [3]-[7] i.e. 5 MSB bits  // other value - e.g. 0E, will be read from this register settings in the set file and ignore the value set in NVM as described above
	write_cmos_sensor_8(0x3238, 0x09); // ash_uDecompressRadiusShifter
	write_cmos_sensor_8(0x3239, 0x09); // ash_uDecompressParabolaScale
	write_cmos_sensor_8(0x323A, 0x0B); // ash_uDecompressFinalScale
	write_cmos_sensor_8(0x3160, 0x06); // ash_GrasCfg  06  // 36  // [5:5] fegras_gain_clamp   0 _ clamp gain to 0..1023 // _V_// 1 _ clamp_gain to 256..1023// [4:4] fegras_plus_zero   Adjust final gain by the one or the zero // 0 _ [Output = Input x Gain x Alfa]  // _V_// 1 _ [Output = Input x (1 + Gain x Alfa)]
	//BASE Profile parabola start
	//BASE Profile parabola end
	write_cmos_sensor_8(0x0B01, 0x32); // smiaRegs_rw_isp_luminance_correction_level 4F :85%  32:70%

	write_cmos_sensor_8(0x3161, 0x00); // ash_GrasShifter 00
	write_cmos_sensor(0x3164, 0x09C4); // ash_luma_params[0]_tmpr
	write_cmos_sensor(0x3166, 0x0100); // ash_luma_params[0]_alpha[0]
	write_cmos_sensor(0x3168, 0x0100); // ash_luma_params[0]_alpha[1]
	write_cmos_sensor(0x316A, 0x0100); // ash_luma_params[0]_alpha[2]
	write_cmos_sensor(0x316C, 0x0100); // ash_luma_params[0]_alpha[3]
	write_cmos_sensor(0x316E, 0x0011); // ash_luma_params[0]_beta[0]
	write_cmos_sensor(0x3170, 0x002F); // ash_luma_params[0]_beta[1]
	write_cmos_sensor(0x3172, 0x0000); // ash_luma_params[0]_beta[2]
	write_cmos_sensor(0x3174, 0x0011); // ash_luma_params[0]_beta[3]
	write_cmos_sensor(0x3176, 0x0A8C); // ash_luma_params[1]_tmpr
	write_cmos_sensor(0x3178, 0x0100); // ash_luma_params[1]_alpha[0]
	write_cmos_sensor(0x317A, 0x0100); // ash_luma_params[1]_alpha[1]
	write_cmos_sensor(0x317C, 0x0100); // ash_luma_params[1]_alpha[2]
	write_cmos_sensor(0x317E, 0x0100); // ash_luma_params[1]_alpha[3]
	write_cmos_sensor(0x3180, 0x0011); // ash_luma_params[1]_beta[0]
	write_cmos_sensor(0x3182, 0x002F); // ash_luma_params[1]_beta[1]
	write_cmos_sensor(0x3184, 0x0000); // ash_luma_params[1]_beta[2]
	write_cmos_sensor(0x3186, 0x0011); // ash_luma_params[1]_beta[3]
	write_cmos_sensor(0x3188, 0x0CE4); // ash_luma_params[2]_tmpr
	write_cmos_sensor(0x318A, 0x0100); // ash_luma_params[2]_alpha[0]
	write_cmos_sensor(0x318C, 0x0100); // ash_luma_params[2]_alpha[1]
	write_cmos_sensor(0x318E, 0x0100); // ash_luma_params[2]_alpha[2]
	write_cmos_sensor(0x3190, 0x0100); // ash_luma_params[2]_alpha[3]
	write_cmos_sensor(0x3192, 0x0011); // ash_luma_params[2]_beta[0]
	write_cmos_sensor(0x3194, 0x002F); // ash_luma_params[2]_beta[1]
	write_cmos_sensor(0x3196, 0x0000); // ash_luma_params[2]_beta[2]
	write_cmos_sensor(0x3198, 0x0011); // ash_luma_params[2]_beta[3]
	write_cmos_sensor(0x319A, 0x1004); // ash_luma_params[3]_tmpr
	write_cmos_sensor(0x319C, 0x0100); // ash_luma_params[3]_alpha[0]
	write_cmos_sensor(0x319E, 0x0100); // ash_luma_params[3]_alpha[1]
	write_cmos_sensor(0x31A0, 0x0100); // ash_luma_params[3]_alpha[2]
	write_cmos_sensor(0x31A2, 0x0100); // ash_luma_params[3]_alpha[3]
	write_cmos_sensor(0x31A4, 0x0011); // ash_luma_params[3]_beta[0]
	write_cmos_sensor(0x31A6, 0x002F); // ash_luma_params[3]_beta[1]
	write_cmos_sensor(0x31A8, 0x0000); // ash_luma_params[3]_beta[2]
	write_cmos_sensor(0x31AA, 0x0011); // ash_luma_params[3]_beta[3]
	write_cmos_sensor(0x31AC, 0x1388); // ash_luma_params[4]_tmpr
	write_cmos_sensor(0x31AE, 0x0100); // ash_luma_params[4]_alpha[0]
	write_cmos_sensor(0x31B0, 0x0100); // ash_luma_params[4]_alpha[1]
	write_cmos_sensor(0x31B2, 0x0100); // ash_luma_params[4]_alpha[2]
	write_cmos_sensor(0x31B4, 0x0100); // ash_luma_params[4]_alpha[3]
	write_cmos_sensor(0x31B6, 0x0011); // ash_luma_params[4]_beta[0]
	write_cmos_sensor(0x31B8, 0x002F); // ash_luma_params[4]_beta[1]
	write_cmos_sensor(0x31BA, 0x0000); // ash_luma_params[4]_beta[2]
	write_cmos_sensor(0x31BC, 0x0011); // ash_luma_params[4]_beta[3]
	write_cmos_sensor(0x31BE, 0x1964); // ash_luma_params[5]_tmpr
	write_cmos_sensor(0x31C0, 0x0100); // ash_luma_params[5]_alpha[0]
	write_cmos_sensor(0x31C2, 0x0100); // ash_luma_params[5]_alpha[1]
	write_cmos_sensor(0x31C4, 0x0100); // ash_luma_params[5]_alpha[2]
	write_cmos_sensor(0x31C6, 0x0100); // ash_luma_params[5]_alpha[3]
	write_cmos_sensor(0x31C8, 0x0011); // ash_luma_params[5]_beta[0]
	write_cmos_sensor(0x31CA, 0x002F); // ash_luma_params[5]_beta[1]
	write_cmos_sensor(0x31CC, 0x0000); // ash_luma_params[5]_beta[2]
	write_cmos_sensor(0x31CE, 0x0011); // ash_luma_params[5]_beta[3]
	write_cmos_sensor(0x31D0, 0x1D4C); // ash_luma_params[6]_tmpr
	write_cmos_sensor(0x31D2, 0x0100); // ash_luma_params[6]_alpha[0]
	write_cmos_sensor(0x31D4, 0x0100); // ash_luma_params[6]_alpha[1]
	write_cmos_sensor(0x31D6, 0x0100); // ash_luma_params[6]_alpha[2]
	write_cmos_sensor(0x31D8, 0x0100); // ash_luma_params[6]_alpha[3]
	write_cmos_sensor(0x31DA, 0x0011); // ash_luma_params[6]_beta[0]
	write_cmos_sensor(0x31DC, 0x002F); // ash_luma_params[6]_beta[1]
	write_cmos_sensor(0x31DE, 0x0000); // ash_luma_params[6]_beta[2]
	write_cmos_sensor(0x31E0, 0x0011); // ash_luma_params[6]_beta[3]
	write_cmos_sensor_8(0x3162, 0x00); // ash_bLumaMode 01
	write_cmos_sensor(0x301C, 0x0100); // smiaRegs_vendor_gras_nvm_address
	write_cmos_sensor_8(0x301E, 0x03); // smiaRegs_vendor_gras_load_from 03
	write_cmos_sensor_8(0x323C, 0x00); // ash_bSkipNvmGrasOfs 01 // skipping the value set in nvm page 0 address 47
	write_cmos_sensor_8(0x323D, 0x01); // ash_uNvmGrasTblOfs 01 // load shading table 1 from nvm
	write_cmos_sensor_8(0x1989, 0x04); //smiaRegs_ro_edof_cap_uAlphaTempInd 04

	//write_cmos_sensor_8(0x0B05, 0x01); // #smiaRegs_rw_isp_mapped_couplet_correct_enable
	//write_cmos_sensor_8(0x0100, 0x01); // smiaRegs_rw_general_setup_mode_select

	//write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x0B00, 0x01); // smiaRegs_rw_isp_shading_correction_enable 01
	//write_cmos_sensor_8(0x0100, 0x01); // smiaRegs_rw_general_setup_mode_select
	mdelay(20);

	LOG_INF("exit init\n");

}	/*	sensor_init  */


static void preview_setting(void)
{
	//1600x1200
	LOG_INF("preview seting enter,I'm the new one\n");
	write_cmos_sensor(0x0100, 0x0000);
	mdelay(10);

	write_cmos_sensor_8(0x0105, 0x04); // mask interrupt
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0114, 0x03); //3
	write_cmos_sensor(0x030E, 0x00A5);
	write_cmos_sensor(0x0342, 0x0E68);
	write_cmos_sensor(0x0340, 0x09E2);
	write_cmos_sensor(0x0200, 0x0618);
	write_cmos_sensor(0x0202, 0x09C2);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor_8(0x3011, 0x01);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x22);
	write_cmos_sensor(0x0344, 0x0004);
	write_cmos_sensor(0x0346, 0x0004);
	write_cmos_sensor(0x0348, 0x0CC3); //3267
	write_cmos_sensor(0x034A, 0x0993); //2451
	write_cmos_sensor(0x034C, 0x0660);
	write_cmos_sensor(0x034E, 0x04C8);
	write_cmos_sensor(0x6004, 0x0000);
	write_cmos_sensor(0x6028, 0xD000);
	write_cmos_sensor(0x602A, 0x012A);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x7077);
	write_cmos_sensor(0x6F12, 0x7777);
	write_cmos_sensor_8(0x0104, 0x00);
	//mdelay(10);
	write_cmos_sensor_8(0x0B00, 0x01);
	write_cmos_sensor(0x0100, 0x0100);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.line_length = imgsensor_info.pre.linelength;
	spin_unlock(&imgsensor_drv_lock);
	mdelay(50);

#ifdef S5K3H7Y_OTP_USE
	otp_sunny_update_wb();
#endif
}	/*	preview_setting  */


static void normal_capture_setting(void)
{
	LOG_INF("capture setting enter\n");
	
	write_cmos_sensor(0x0100, 0x0000);
	mdelay(10);
	write_cmos_sensor_8(0x0105, 0x04);
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0114, 0x03);
	write_cmos_sensor(0x030E, 0x00A5);
	write_cmos_sensor(0x0342, 0x0E68);
	write_cmos_sensor(0x0340, 0x09E2);
	write_cmos_sensor(0x0200, 0x0618);
	write_cmos_sensor(0x0202, 0x09C2);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0346, 0x0004);
	write_cmos_sensor(0x034A, 0x0993);
	write_cmos_sensor(0x034C, 0x0CC0);
	write_cmos_sensor(0x034E, 0x0990);
	write_cmos_sensor(0x0900, 0x0011);
	write_cmos_sensor(0x0901, 0x0011);
	write_cmos_sensor(0x3011, 0x0001);
	write_cmos_sensor(0x6004, 0x0000);
	write_cmos_sensor(0x6028, 0xD000);
	write_cmos_sensor(0x602A, 0x012A);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x7077) ;
	write_cmos_sensor(0x6F12, 0x7777);
	write_cmos_sensor_8(0x0104, 0x00);
	//mdelay(10);
	write_cmos_sensor_8(0x0B00, 0x01);
	write_cmos_sensor(0x0100, 0x0100);
	//write_cmos_sensor_8(0x0104, 0x00);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.line_length = imgsensor_info.cap.linelength;
	spin_unlock(&imgsensor_drv_lock);
	mdelay(2);
}

static void pip_capture_setting(voi)
{
	normal_capture_setting();
}


static void capture_setting(kal_uint16 currefps)
{
	//capture_20fps();
	normal_capture_setting();
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF(" normal video setting,use capture setting,now grab window is %d * %d",imgsensor_info.normal_video.grabwindow_width,imgsensor_info.normal_video.grabwindow_height);
#if 0
	write_cmos_sensor(0x0100, 0x0000);
	mdelay(10);
	write_cmos_sensor_8(0x0105, 0x04);
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0114, 0x03);
	write_cmos_sensor(0x030E, 0x00A5);
	write_cmos_sensor(0x0342, 0x0E68);
	write_cmos_sensor(0x0340, 0x09E2);
	write_cmos_sensor(0x0200, 0x0618);
	write_cmos_sensor(0x0202, 0x09C2);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0346, 0x0136);
	write_cmos_sensor(0x034A, 0x0861);
	write_cmos_sensor(0x034C, 0x0CC0);
	write_cmos_sensor(0x034E, 0x072C);
	write_cmos_sensor(0x0900, 0x0011);
	write_cmos_sensor(0x0901, 0x0011);
	write_cmos_sensor(0x3011, 0x0001);
	write_cmos_sensor(0x6004, 0x0000);
	write_cmos_sensor(0x6028, 0xD000);
	write_cmos_sensor(0x602A, 0x012A);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x7077);
	write_cmos_sensor(0x6F12, 0x7777);
	write_cmos_sensor_8(0x0104, 0x00);
	write_cmos_sensor(0x0100, 0x0100);
#endif
	// use capture setting
	normal_capture_setting();

}
static void hs_video_setting(void)
{
	LOG_INF("enter hs_video setting\n");
	write_cmos_sensor(0x0100, 0x00);
	mdelay(10);

	write_cmos_sensor(0x6218, 0xF1D0);
	write_cmos_sensor(0x6214, 0xF9F0);
	write_cmos_sensor(0x6226, 0x0001);
	write_cmos_sensor(0xB0C0, 0x000C);
	write_cmos_sensor(0x6226, 0x0000);
	write_cmos_sensor(0x6218, 0xF9F0);
	write_cmos_sensor_8(0x0114, 0x03);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0002);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x008C);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0006);
	write_cmos_sensor(0x030E, 0x00AF);
	write_cmos_sensor(0x0342, 0x0DE8);
	write_cmos_sensor(0x0340, 0x028E);
	write_cmos_sensor(0x0200, 0x0618);
	write_cmos_sensor(0x0202, 0x0002);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor_8(0x3011, 0x04);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x14);
	write_cmos_sensor(0x0344, 0x0000);
	write_cmos_sensor(0x0348, 0x0CC7);
	write_cmos_sensor(0x0346, 0x0004);
	write_cmos_sensor(0x034A, 0x0993);
	write_cmos_sensor(0x034C, 0x0330);
	write_cmos_sensor(0x034E, 0x0264);
	write_cmos_sensor(0x0400, 0x0002);
	write_cmos_sensor(0x0404, 0x0010);
	mdelay(10);
	write_cmos_sensor_8(0x0100, 0x01);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	spin_unlock(&imgsensor_drv_lock);
	mdelay(2);
}

static void slim_video_setting(void)
{
	LOG_INF("enter slim_video setting\n");
	write_cmos_sensor(0x0100, 0x00);
	mdelay(10);

	write_cmos_sensor(0x6218, 0xF1D0);
	write_cmos_sensor(0x6214, 0xF9F0);
	write_cmos_sensor(0x6226, 0x0001);
	write_cmos_sensor(0xB0C0, 0x000C);
	write_cmos_sensor(0x6226, 0x0000);
	write_cmos_sensor(0x6218, 0xF9F0);
	write_cmos_sensor_8(0x0114, 0x03);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0300, 0x0002);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x008C);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x030C, 0x0006);
	write_cmos_sensor(0x030E, 0x00A5);
	write_cmos_sensor(0x0342, 0x0C10);
	write_cmos_sensor(0x0340, 0x02FE);
	write_cmos_sensor(0x0200, 0x0618);
	write_cmos_sensor(0x0202, 0x0002);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor_8(0x3011, 0x01);
	write_cmos_sensor(0x0382, 0x0003);
	write_cmos_sensor(0x0386, 0x0003);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x22);
	write_cmos_sensor(0x0344, 0x0164);
	write_cmos_sensor(0x0348, 0x0B63);
	write_cmos_sensor(0x0346, 0x01FC);
	write_cmos_sensor(0x034A, 0x079B);
	write_cmos_sensor(0x034C, 0x0500);
	write_cmos_sensor(0x034E, 0x02D0);
	write_cmos_sensor(0x0400, 0x0002);
	write_cmos_sensor(0x0404, 0x0010);
	mdelay(10);
	write_cmos_sensor_8(0x0100, 0x01);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	spin_unlock(&imgsensor_drv_lock);
	mdelay(2);
}


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
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
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
	LOG_1;
	LOG_2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
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
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
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
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("Currently camera mode is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.frame_length,imgsensor.line_length);

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
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
	{
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
	{
		if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);   
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 

	spin_unlock(&imgsensor_drv_lock);

	LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
	set_dummy();
	capture_setting(imgsensor.current_fps); 
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("Currently camera mode is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.frame_length,imgsensor.line_length);

	return ERROR_NONE;
}	/* capture() */


static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);
	LOG_INF("Currently camera mode is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.frame_length,imgsensor.line_length);
	return ERROR_NONE;
}	/*	normal_video   */


static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("hs_video enter i ");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
	hs_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	LOG_INF("Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);
}	/*	hs_video   */


static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("slim video enter in");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
	slim_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	LOG_INF("Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF(" get resolution, now the mode is%d",imgsensor.sensor_mode);
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d", scenario_id);
	
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
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame; 
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

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
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 
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
	LOG_INF("scenario_id = %d", scenario_id);
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
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
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
	LOG_INF("framerate = %d\n ", framerate);
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
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) 	  
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
			if(framerate==300)
			{
				frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
				frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
				spin_lock(&imgsensor_drv_lock);
				imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
				imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length = imgsensor.frame_length;
				spin_unlock(&imgsensor_drv_lock);
			}
			//set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			LOG_INF("scenario hs_video:Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);
			//set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
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
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable){
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0600, 0x0002);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0600, 0x0000);
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
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
     //unsigned long long *feature_return_para=(unsigned long long *) feature_para;
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
			write_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
		     //night_mode((BOOL) *feature_data);
			break;
		case SENSOR_FEATURE_SET_GAIN:		
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			if((sensor_reg_data->RegData>>8)>0)
			   write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			else
				write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
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
			get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
			LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);		
			break;
		case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = (BOOL)*feature_data;
			imgsensor.ihdr_en = KAL_FALSE;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data_32);
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
			break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
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

UINT32 S5K3H7YX_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	printk("Jeff,enter S5K3H7YX_MIPI_RAW_SensorInit\n");
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;

	return ERROR_NONE;
}	/*	s5k2p8_MIPI_RAW_SensorInit	*/
