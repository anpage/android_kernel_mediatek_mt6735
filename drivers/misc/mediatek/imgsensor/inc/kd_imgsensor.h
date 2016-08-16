#ifndef _KD_IMGSENSOR_H
#define _KD_IMGSENSOR_H

#include <linux/ioctl.h>
/* #define CONFIG_COMPAT */
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#ifndef ASSERT
#define ASSERT(expr)        BUG_ON(!(expr))
#endif

#define IMGSENSORMAGIC 'i'
/* IOCTRL(inode * ,file * ,cmd ,arg ) */
/* S means "set through a ptr" */
/* T means "tell by a arg value" */
/* G means "get by a ptr" */
/* Q means "get by return a value" */
/* X means "switch G and S atomically" */
/* H means "switch T and Q atomically" */

/*******************************************************************************
*
********************************************************************************/
#define YUV_INFO(_id, name, getCalData)\
	{ \
		_id, name, \
NSFeature :  : YUVSensorInfo < _id >  :  : createInstance(name, #name), \
		(NSFeature :  : SensorInfoBase*(*)()) \
NSFeature :  : YUVSensorInfo < _id >  :  : getInstance, \
NSFeature :  : YUVSensorInfo < _id >  :  : getDefaultData, \
		getCalData, \
NSFeature :  : YUVSensorInfo < _id >  :  : getNullFlickerPara \
	}
#define RAW_INFO(_id, name, getCalData)\
	{ \
		_id, name, \
NSFeature :  : RAWSensorInfo < _id >  :  : createInstance(name, #name), \
		(NSFeature :  : SensorInfoBase*(*)()) \
NSFeature :  : RAWSensorInfo < _id >  :  : getInstance, \
NSFeature :  : RAWSensorInfo < _id >  :  : getDefaultData, \
		getCalData, \
NSFeature :  : RAWSensorInfo < _id >  :  : getFlickerPara \
	}
/*******************************************************************************
*
********************************************************************************/

/* sensorOpen */
#define KDIMGSENSORIOC_T_OPEN                       _IO(IMGSENSORMAGIC, 0)
/* sensorGetInfo */
#define KDIMGSENSORIOC_X_GETINFO                    _IOWR(IMGSENSORMAGIC, 5, ACDK_SENSOR_GETINFO_STRUCT)
/* sensorGetResolution */
#define KDIMGSENSORIOC_X_GETRESOLUTION              _IOWR(IMGSENSORMAGIC, 10, ACDK_SENSOR_RESOLUTION_INFO_STRUCT)
/* For kernel 64-bit */
#define KDIMGSENSORIOC_X_GETRESOLUTION2             _IOWR(IMGSENSORMAGIC, 10, ACDK_SENSOR_PRESOLUTION_STRUCT)
/* sensorFeatureControl */
#define KDIMGSENSORIOC_X_FEATURECONCTROL            _IOWR(IMGSENSORMAGIC, 15, ACDK_SENSOR_FEATURECONTROL_STRUCT)
/* sensorControl */
#define KDIMGSENSORIOC_X_CONTROL                    _IOWR(IMGSENSORMAGIC, 20, ACDK_SENSOR_CONTROL_STRUCT)
/* sensorClose */
#define KDIMGSENSORIOC_T_CLOSE                      _IO(IMGSENSORMAGIC, 25)
/* sensorSearch */
#define KDIMGSENSORIOC_T_CHECK_IS_ALIVE             _IO(IMGSENSORMAGIC, 30)
/* set sensor driver */
#define KDIMGSENSORIOC_X_SET_DRIVER                 _IOWR(IMGSENSORMAGIC, 35, SENSOR_DRIVER_INDEX_STRUCT)
/* get socket postion */
#define KDIMGSENSORIOC_X_GET_SOCKET_POS             _IOWR(IMGSENSORMAGIC, 40, u32)
/* set I2C bus */
#define KDIMGSENSORIOC_X_SET_I2CBUS                 _IOWR(IMGSENSORMAGIC, 45, u32)
/* set I2C bus */
#define KDIMGSENSORIOC_X_RELEASE_I2C_TRIGGER_LOCK   _IO(IMGSENSORMAGIC, 50)
/* Set Shutter Gain Wait Done */
#define KDIMGSENSORIOC_X_SET_SHUTTER_GAIN_WAIT_DONE _IOWR(IMGSENSORMAGIC, 55, u32)
/* set mclk */
#define KDIMGSENSORIOC_X_SET_MCLK_PLL               _IOWR(IMGSENSORMAGIC, 60, ACDK_SENSOR_MCLK_STRUCT)
#define KDIMGSENSORIOC_X_GETINFO2                   _IOWR(IMGSENSORMAGIC, 65, IMAGESENSOR_GETINFO_STRUCT)
/* set open/close sensor index */
#define KDIMGSENSORIOC_X_SET_CURRENT_SENSOR         _IOWR(IMGSENSORMAGIC, 70, u32)
/* set GPIO */
#define KDIMGSENSORIOC_X_SET_GPIO                   _IOWR(IMGSENSORMAGIC, 75, IMGSENSOR_GPIO_STRUCT)
/* Get ISP CLK */
#define KDIMGSENSORIOC_X_GET_ISP_CLK                _IOWR(IMGSENSORMAGIC, 80, u32)

#ifdef CONFIG_COMPAT
#define COMPAT_KDIMGSENSORIOC_X_GETINFO            _IOWR(IMGSENSORMAGIC, 5, COMPAT_ACDK_SENSOR_GETINFO_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL    _IOWR(IMGSENSORMAGIC, 15, COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_CONTROL            _IOWR(IMGSENSORMAGIC, 20, COMPAT_ACDK_SENSOR_CONTROL_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_GETINFO2           _IOWR(IMGSENSORMAGIC, 65, COMPAT_IMAGESENSOR_GETINFO_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_GETRESOLUTION2     _IOWR(IMGSENSORMAGIC, 10, COMPAT_ACDK_SENSOR_PRESOLUTION_STRUCT)
#endif

/*******************************************************************************
*
********************************************************************************/
/* SENSOR CHIP VERSION */
#define OV5670MIPI_SENSOR_ID                    0x5670
#define OV5670MIPI_CMK_SENSOR_ID                0x5671
#define S5K3M2_SENSOR_ID                        0x30D2 //yangchen add 
#define HI842_SENSOR_ID							0x0842
#define IMX258_SENSOR_ID						0x0258
#define IMX258_SUNNY_SENSOR_ID					0x0259
#define S5K4H8_SENSOR_ID                        0x4088
#define OV13850_SENSOR_ID                       0xD850
#define IMX179_SENSOR_ID                        0x0179
#define IMX179MAIN_SENSOR_ID					0x017A 
#define S5K3H7YX_SENSOR_ID                      0x3087
#define IMX219_SENSOR_ID                        0x0219
#define GC2355_SENSOR_ID                        0x2355
#define OV5648MIPI_SENSOR_ID                    0x5648
#define S5K2P8_SENSOR_ID                        0x2108
#define IMX135_SENSOR_ID                        0x0135
#define OV5648MIPI_SENSOR_ID                    0x5648
#define OV5648MIPI_SENSOR_ID_DARLING            0x5649 //yangchen add 
#define OV5648MIPI_SENSOR_ID_SUNWIN             0x5650 //yangchen add 

/* CAMERA DRIVER NAME */
#define CAMERA_HW_DEVNAME                       "kd_camera_hw"
/* SENSOR DEVICE DRIVER NAME */
#define SENSOR_DRVNAME_S5K3M2_MIPI_RAW           "s5k3m2mipiraw" //yangchen add 
#define SENSOR_DRVNAME_OV5648_MIPI_RAW_DARLING   "ov5648mipi_darling" //yangchen add 
#define SENSOR_DRVNAME_OV5648_MIPI_RAW_SUNWIN    "ov5648mipi_sunwin"  //yangchen add 
#define SENSOR_DRVNAME_HI842_MIPI_RAW			"hi842mipiraw_sunwin"
#define SENSOR_DRVNAME_IMX258_MIPI_RAW			"imx258mipiraw_liteon"
#define SENSOR_DRVNAME_IMX258_SUNNY_MIPI_RAW	"imx258mipiraw_sunny"
#define SENSOR_DRVNAME_S5K4H8_MIPI_RAW          "s5k4h8mipiraw_darling"
#define SENSOR_DRVNAME_OV5670_MIPI_RAW          "ov5670mipiraw_sunwin"
#define SENSOR_DRVNAME_OV5670_CMK_MIPI_RAW      "ov5670mipiraw_cmk"
#define SENSOR_DRVNAME_OV13850_MIPI_RAW         "ov13850mipiraw"
#define SENSOR_DRVNAME_IMX179_MIPI_RAW          "imx179mipiraw_sunny"
#define SENSOR_DRVNAME_IMX179MAIN_MIPI_RAW      "imx179mainmipiraw_sunny"
#define SENSOR_DRVNAME_S5K3H7YX_MIPI_RAW		"s5k3h7yxmipiraw_sunny" 
#define SENSOR_DRVNAME_IMX219_MIPI_RAW          "imx219mipiraw"
#define SENSOR_DRVNAME_GC2355_MIPI_RAW          "gc2355mipiraw"
#define SENSOR_DRVNAME_OV5648_MIPI_RAW          "ov5648mipi"
#define SENSOR_DRVNAME_S5K2P8_MIPI_RAW          "s5k2p8mipiraw"
#define SENSOR_DRVNAME_IMX135_MIPI_RAW          "imx135mipiraw"



/*******************************************************************************
*
********************************************************************************/
void KD_IMGSENSOR_PROFILE_INIT(void);
void KD_IMGSENSOR_PROFILE(char *tag);

#define mDELAY(ms)     mdelay(ms)
#define uDELAY(us)       udelay(us)
#endif              /* _KD_IMGSENSOR_H */
