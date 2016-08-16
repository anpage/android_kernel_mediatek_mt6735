
/*************************************************************************************************
3h7_otp.c
---------------------------------------------------------
OTP Application file From Truly for S5K3H7
2013.01.14
---------------------------------------------------------
NOTE:
The modification is appended to initialization of image sensor.
After sensor initialization, use the function , and get the id value.
bool otp_wb_update(BYTE zone)
and
bool otp_lenc_update(BYTE zone),
then the calibration of AWB and LSC will be applied.
After finishing the OTP written, we will provide you the golden_rg and golden_bg settings.
**************************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3h7yxmipiraw_Sensor.h"
//#include "s5k3h7ymipiraw_Camera_Sensor_para.h"
//#include "s5k3h7ymipiraw_CameraCustomized.h"

//#include <linux/xlog.h>


extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

#define SENSORDB_S5K3H7_OTP(fmt,arg...) xlog_printk(ANDROID_LOG_DEBUG , "[S5K3H7_OTP]", fmt, ##arg)                             //printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)

extern  void S5K3H7Y_wordwrite_cmos_sensor(u16 addr, kal_uint16 para);
extern  void S5K3H7Y_bytewrite_cmos_sensor(u16 addr, kal_uint16 para);
extern unsigned char S5K3H7Y_byteread_cmos_sensor(u16 addr);

#define USHORT             unsigned short
#define BYTE unsigned char
#define Sleep(ms) mdelay(ms)

#define SUNNY_ID 0x01
#define LARGAN_LENS 0x05
#define DONGWOON 0x03
#define ALPS_VCM 0x05
#define VALID_OTP_SUNNY 0x40

#define GAIN_DEFAULT 0x0100
#define GAIN_GREEN1_ADDR 0x020E
#define GAIN_BLUE_ADDR 0x0212
#define GAIN_RED_ADDR 0x0210
#define GAIN_GREEN2_ADDR 0x0214

USHORT sunny_golden_r;
USHORT sunny_golden_gr;
USHORT sunny_golden_gb;
USHORT sunny_golden_b;
USHORT sunny_current_r;
USHORT sunny_current_gr;
USHORT sunny_current_gb;
USHORT sunny_current_b;
kal_uint32 sunny_r_ratio;
kal_uint32 sunny_b_ratio;

kal_uint32 GOLDEN_RG_RATIO = 0x015A; //Typical R/G
kal_uint32 GOLDEN_BG_RATIO = 0x014A; //Typical B/G

/*************************************************************************************************
* Function : start_read_otp_sunny
* Description : before read otp_sunny , set the reading block setting
* Parameters : [BYTE] zone : otp_sunny PAGE index , 0x00~0x0f
* Return : 0, reading block setting err
1, reading block setting ok
**************************************************************************************************/
static bool start_read_otp_sunny(BYTE zone)
{
    BYTE val = 0;
    int i;
    S5K3H7Y_wordwrite_cmos_sensor(0xFCFC, 0xD000);
    S5K3H7Y_bytewrite_cmos_sensor(0x0A02, zone); //Select the page to write by writing to 0xD0000A02 0x01~0x0C
    S5K3H7Y_bytewrite_cmos_sensor(0x0A00, 0x0101); //Enter read mode by writing 01h    to 0xD0000A00
    for(i=0; i<100; i++)
    {
        val = S5K3H7Y_byteread_cmos_sensor(0x0A01);
        SENSORDB_S5K3H7_OTP("Yixuhong read 0x0A01 = 0x %x\n,",val);
        if(val == 0x01)
            break;
        Sleep(2);
    }
    if(i == 100)
    {
        SENSORDB_S5K3H7_OTP("Read Page %d Err!", zone); // print log
        S5K3H7Y_bytewrite_cmos_sensor(0x0A00, 0x00); //Reset the NVM interface by  writing 00h to 0xD0000A00
        return 0;
    }
    return 1;
}

/*************************************************************************************************
* Function : stop_read_otp_sunny
* Description : after read otp_sunny , stop and reset otp_sunny block setting
**************************************************************************************************/
static void stop_read_otp_sunny()
{
    S5K3H7Y_bytewrite_cmos_sensor(0x0A00, 0x00); //Reset the NVM interface bywriting 00h to 0xD0000A00
}

/*************************************************************************************************
* Function : get_otp_sunny_flag
* Description : get otp_sunny WRITTEN_FLAG
* Parameters : [BYTE] zone : otp_sunny PAGE index , 0x00~0x0f
* Return : [BYTE], if 0x40 , this type has valid otp_sunny data, otherwise,
invalid otp_sunny data
**************************************************************************************************/
static BYTE get_otp_sunny_flag_wb(BYTE zone)
{
    BYTE flag = 0;
    if(!start_read_otp_sunny(zone))
    {
        SENSORDB_S5K3H7_OTP("Start read Page %d Fail!", zone);
        return 0;
    }
    flag = S5K3H7Y_byteread_cmos_sensor(0x0A04);
    SENSORDB_S5K3H7_OTP("Yixuhong read 0x0A04 = 0x %x\n,",flag);
    stop_read_otp_sunny();
    flag = flag & 0xc0;
    SENSORDB_S5K3H7_OTP("Flag:0x%02x",flag );
    return flag;
}

/*************************************************************************************************
* Function : get_otp_sunny_date
* Description : get otp_sunny date value
* Parameters : [BYTE] zone : otp_sunny PAGE index , 0x00~0x0f
**************************************************************************************************/
static bool get_otp_sunny_date(BYTE zone)
{
    BYTE year = 0;
    BYTE month = 0;
    BYTE day = 0;
    if(!start_read_otp_sunny(zone))
    {
        SENSORDB_S5K3H7_OTP("Start read Page %d Fail!", zone);
        return 0;
    }
    year = S5K3H7Y_byteread_cmos_sensor(0x0A05);
    month = S5K3H7Y_byteread_cmos_sensor(0x0A06);
    day = S5K3H7Y_byteread_cmos_sensor(0x0A07);
    stop_read_otp_sunny();
    SENSORDB_S5K3H7_OTP("otp_sunny date=%02d.%02d.%02d", year,month,day);
    return 1;
}

/*************************************************************************************************
* Function : get_otp_sunny_module_id
* Description : get otp_sunny MID value
* Parameters : [BYTE] zone : otp_sunny PAGE index , 0x00~0x0f
* Return : [BYTE] 0 : otp_sunny data fail
other value : module ID data , TRULY ID is 0x0001
**************************************************************************************************/
static BYTE get_otp_sunny_module_id(BYTE zone)
{
    BYTE module_id = 0;
    if(!start_read_otp_sunny(zone))
    {
        SENSORDB_S5K3H7_OTP("Start read Page %d Fail!", zone);
        return 0;
    }
    module_id = S5K3H7Y_byteread_cmos_sensor(0x0A08);
    stop_read_otp_sunny();
    SENSORDB_S5K3H7_OTP("otp_sunny_Module ID: 0x%02x.\n",module_id);
    return module_id;
}

/*************************************************************************************************
* Function : get_otp_sunny_lens_id
* Description : get otp_sunny LENS_ID value
* Parameters : [BYTE] zone : otp_sunny PAGE index , 0x00~0x0f
* Return : [BYTE] 0 : otp_sunny data fail
other value : LENS ID data
**************************************************************************************************/
static BYTE get_otp_sunny_lens_id(BYTE zone)
{
    BYTE lens_id = 0;
    if(!start_read_otp_sunny(zone))
    {
        SENSORDB_S5K3H7_OTP("Start read Page %d Fail!", zone);
        return 0;
    }
    lens_id = S5K3H7Y_byteread_cmos_sensor(0x0A0A);
    stop_read_otp_sunny();
    SENSORDB_S5K3H7_OTP("otp_sunny_Lens ID: 0x%02x.\n",lens_id);
    return lens_id;
}

/*************************************************************************************************
* Function : get_otp_sunny_vcm_id
* Description : get otp_sunny VCM_ID value
* Parameters : [BYTE] zone : otp_sunny PAGE index , 0x00~0x0f
* Return : [BYTE] 0 : otp_sunny data fail
other value : VCM ID data
**************************************************************************************************/
static BYTE get_otp_sunny_vcm_id(BYTE zone)
{
    BYTE vcm_id = 0;
    if(!start_read_otp_sunny(zone))
    {
        SENSORDB_S5K3H7_OTP("Start read Page %d Fail!", zone);
        return 0;
    }
    vcm_id = S5K3H7Y_byteread_cmos_sensor(0x0A0B);
    stop_read_otp_sunny();
    SENSORDB_S5K3H7_OTP("otp_sunny_VCM ID: 0x%02x.\n",vcm_id);
    return vcm_id;
}

/*************************************************************************************************
* Function : get_otp_sunny_driver_id
* Description : get otp_sunny driver id value
* Parameters : [BYTE] zone : otp_sunny PAGE index , 0x00~0x0f
* Return : [BYTE] 0 : otp_sunny data fail
other value : driver ID data
**************************************************************************************************/
static BYTE get_otp_sunny_driver_id(BYTE zone)
{
    BYTE driver_id = 0;
    if(!start_read_otp_sunny(zone))
    {
        SENSORDB_S5K3H7_OTP("Start read Page %d Fail!", zone);
        return 0;
    }
    driver_id = S5K3H7Y_byteread_cmos_sensor(0x0A0C);
    stop_read_otp_sunny();
    SENSORDB_S5K3H7_OTP("otp_sunny_Driver ID: 0x%02x.\n",driver_id);
    return driver_id;
}

/*************************************************************************************************
* Function : sunny_wb_gain_set
* Description : Set WB ratio to register gain setting 512x
* Parameters : [int] sunny_r_ratio : R ratio data compared with golden module R
sunny_b_ratio : B ratio data compared with golden module B
* Return : [bool] 0 : set wb fail
1 : WB set success
**************************************************************************************************/
static bool sunny_wb_gain_set()
{
    USHORT R_gain,B_gain,G_gain,G_gain_R,G_gain_B;	
	USHORT R_GAIN_H,R_GAIN_L,B_GAIN_H,B_GAIN_L,G_GAIN_H,G_GAIN_L;
	
    if(!sunny_r_ratio || !sunny_b_ratio)
    {
        SENSORDB_S5K3H7_OTP("otp_sunny WB ratio Data Err!");
        return 0;
    }
    if (sunny_b_ratio < GOLDEN_BG_RATIO)
    {
        if (sunny_r_ratio< GOLDEN_RG_RATIO)
        {
            G_gain = GAIN_DEFAULT;
            B_gain = (GAIN_DEFAULT * GOLDEN_BG_RATIO) / sunny_b_ratio;
            R_gain = (GAIN_DEFAULT * GOLDEN_RG_RATIO) / sunny_r_ratio;
        }
        else
        {
            R_gain = GAIN_DEFAULT;
            G_gain = (GAIN_DEFAULT * sunny_r_ratio) / GOLDEN_RG_RATIO;
            B_gain = (G_gain * GOLDEN_BG_RATIO) /sunny_b_ratio;
        }
    }
    else
    {
        if (sunny_r_ratio < GOLDEN_RG_RATIO)
        {
            B_gain = GAIN_DEFAULT;
            G_gain = (GAIN_DEFAULT * sunny_b_ratio)/ GOLDEN_BG_RATIO;
            R_gain = (G_gain * GOLDEN_RG_RATIO)/ sunny_r_ratio;
        }
        else
        {
            G_gain_B = (GAIN_DEFAULT * sunny_b_ratio) / GOLDEN_BG_RATIO;
            G_gain_R = (GAIN_DEFAULT * sunny_r_ratio) / GOLDEN_RG_RATIO;
            if(G_gain_B > G_gain_R )
            {
                B_gain = GAIN_DEFAULT;
                G_gain = G_gain_B;
                R_gain = (G_gain * GOLDEN_RG_RATIO) /sunny_r_ratio;
            }
            else
            {
                R_gain = GAIN_DEFAULT;
                G_gain = G_gain_R;
                B_gain = (G_gain * GOLDEN_BG_RATIO) / sunny_b_ratio;
            }
        }
    }
    SENSORDB_S5K3H7_OTP("otp_sunny_sunny_r_ratio=%d,sunny_b_ratio=%d \n",sunny_r_ratio,sunny_b_ratio);
    SENSORDB_S5K3H7_OTP("otp_sunny_sunny_r_gain=%d,sunny_b_gain=%d,sunny_g_gain=%d \n",R_gain,B_gain,G_gain);
    if(R_gain < GAIN_DEFAULT)
        R_gain = GAIN_DEFAULT;
    if(G_gain < GAIN_DEFAULT)
        G_gain = GAIN_DEFAULT;
    if(B_gain < GAIN_DEFAULT)
        B_gain = GAIN_DEFAULT;
	
    R_GAIN_H = (R_gain >> 8) & 0xFF;
    R_GAIN_L = R_gain & 0xFF;
    B_GAIN_H = (B_gain >> 8) & 0xFF;
    B_GAIN_L = B_gain & 0xFF;
    G_GAIN_H = (G_gain >> 8) & 0xFF;
    G_GAIN_L = G_gain & 0xFF;
    S5K3H7Y_bytewrite_cmos_sensor(GAIN_RED_ADDR, R_GAIN_H);
    S5K3H7Y_bytewrite_cmos_sensor(GAIN_RED_ADDR+1, R_GAIN_L);
    S5K3H7Y_bytewrite_cmos_sensor(GAIN_BLUE_ADDR, B_GAIN_H);
    S5K3H7Y_bytewrite_cmos_sensor(GAIN_BLUE_ADDR+1, B_GAIN_L);
    S5K3H7Y_bytewrite_cmos_sensor(GAIN_GREEN1_ADDR, G_GAIN_H);
    S5K3H7Y_bytewrite_cmos_sensor(GAIN_GREEN1_ADDR+1, G_GAIN_L); //Green 1default gain 1x
    S5K3H7Y_bytewrite_cmos_sensor(GAIN_GREEN2_ADDR, G_GAIN_H); //Green 2 default   gain 1x
    S5K3H7Y_bytewrite_cmos_sensor(GAIN_GREEN2_ADDR+1, G_GAIN_L);
    SENSORDB_S5K3H7_OTP("otp_sunny WB Update Finished! \n");
    return 1;
}

/*************************************************************************************************
* Function : get_otp_sunny_wb
* Description : Get WB data
* Parameters : [BYTE] zone : otp_sunny PAGE index , 0x00~0x0f
**************************************************************************************************/
static bool get_otp_sunny_wb(BYTE zone)
{
    BYTE temph = 0;
    BYTE templ = 0;
    sunny_golden_r = 0, sunny_golden_gr = 0, sunny_golden_gb = 0, sunny_golden_b = 0;
    sunny_current_r = 0, sunny_current_gr = 0, sunny_current_gb = 0, sunny_current_b
                                            = 0;
    if(!start_read_otp_sunny(zone))
    {
        SENSORDB_S5K3H7_OTP("Start read Page %d Fail!", zone);
        return 0;
    }
    temph = S5K3H7Y_byteread_cmos_sensor(0x0A14);
    templ = S5K3H7Y_byteread_cmos_sensor(0x0A15);
    sunny_golden_r = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
    templ = S5K3H7Y_byteread_cmos_sensor(0x0A16);
    sunny_golden_b = (USHORT)templ + (((USHORT)temph & 0x0C) << 6);
    templ = S5K3H7Y_byteread_cmos_sensor(0x0A17);
    sunny_golden_gr = (USHORT)templ + (((USHORT)temph & 0x30) << 4);
    templ = S5K3H7Y_byteread_cmos_sensor(0x0A18);
    sunny_golden_gb = (USHORT)templ + (((USHORT)temph & 0xC0) << 2);
    temph = S5K3H7Y_byteread_cmos_sensor(0x0A19);
    templ = S5K3H7Y_byteread_cmos_sensor(0x0A1A);
    sunny_current_r = (USHORT)templ + (((USHORT)temph & 0x03) << 8);
    templ = S5K3H7Y_byteread_cmos_sensor(0x0A1B);
    sunny_current_b = (USHORT)templ + (((USHORT)temph & 0x0C) << 6);
    templ = S5K3H7Y_byteread_cmos_sensor(0x0A1C);
    sunny_current_gr = (USHORT)templ + (((USHORT)temph & 0x30) << 4);
    templ = S5K3H7Y_byteread_cmos_sensor(0x0A1D);
    sunny_current_gb = (USHORT)templ + (((USHORT)temph & 0xC0) << 2);
    stop_read_otp_sunny();
    return 1;
}

/*************************************************************************************************
* Function : otp_sunny_wb_update
* Description : Update WB correction
* Return : [bool] 0 : otp_sunny data fail
1 : otp_sunny_WB update success
**************************************************************************************************/
static bool otp_sunny_wb_update(BYTE zone)
{
    USHORT golden_g, current_g;
    if(!get_otp_sunny_wb(zone)) // get wb data from otp_sunny
        return 0;
    current_g = (sunny_current_gr + sunny_current_gb) / 2;
    if(!current_g || !sunny_current_r || !sunny_current_b)
    {
        SENSORDB_S5K3H7_OTP("WB update Err !");
        return 0;
    }
    sunny_r_ratio = 512 * sunny_current_r / current_g;
    sunny_b_ratio = 512 * sunny_current_b / current_g;
    sunny_wb_gain_set();
    SENSORDB_S5K3H7_OTP("WB update finished! \n");
    return 1;
}

/*************************************************************************************************
* Function : otp_sunny_update_wb()
* Description : update otp_sunny data from otp_sunny , it otp_sunny data is valid,
it include get ID and WB update function
* Return : [bool] 0 : update fail
1 : update success
**************************************************************************************************/
bool otp_sunny_update_wb()
{
    BYTE zone = 0x01;
    BYTE FLG = 0x00;
    BYTE MID = 0x00;
	//BYTE LENS_ID= 0x00,VCM_ID= 0x00;
    int i;
    for(i=0; i<3; i++)
    {
        FLG = get_otp_sunny_flag_wb(zone);
        if(FLG == VALID_OTP_SUNNY)
            break;
        else
            zone++;
    }
    if(i==3)
    {
        SENSORDB_S5K3H7_OTP("Yixuhong: No otp_sunny Data or otp_sunny data is invalid!!");
        return 0;
    }
    MID = get_otp_sunny_module_id(zone);
#if	0
    LENS_ID= get_otp_sunny_lens_id(zone);
    VCM_ID= get_otp_sunny_vcm_id(zone);
    get_otp_sunny_driver_id(zone);
#endif	
    if(MID !=SUNNY_ID)
    {
        SENSORDB_S5K3H7_OTP("Yixuhong: No SUNNY Module !!");
        return 0;
    }
    otp_sunny_wb_update(zone);
    return 1;
}
