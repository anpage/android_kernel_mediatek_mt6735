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


#define PFX "IMX258_liteon_otp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imx258mipiraw_Sensor.h"

#ifdef IMX258_LITEON_OTP
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);

#define IMX258MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1,  0x20)

struct otp_struct
{
    int product_year;
    int product_month;
    int product_day;
    int module_integrator_id;
    u32 rg_ratio;
    u32 bg_ratio;
    u32 gg_ratio;
};

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define IMX258_EEPROM_READ_ID  0xA0
#define IMX258_EEPROM_WRITE_ID   0xA1
#define IMX258_I2C_SPEED        100  
#define IMX258_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
static BYTE imx258_liteon_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

static kal_uint16 imx258_liteon_read_eeprom(kal_uint16 addr, kal_uint8 write_id)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,write_id);
    return get_byte;
}

#ifdef IMX258_LITEON_OTP_DEBUG
void IMX258_Test_OTP(void)
{
    kal_uint8 value = 0;
    kal_uint16 addr = 0;
    kal_uint8 write_id = 0;
    addr = 0x00;
    write_id = IMX258_EEPROM_READ_ID;
    for(addr = 0; addr <= 0x1b; addr++)
    {
        value = imx258_liteon_read_eeprom(addr,write_id);
        LOG_INF("%s:write_id = %x, addr:%x, value:%x \n",__func__,write_id,addr,value);
    }
}
#endif /*IMX258_LITEON_OTP_DEBUG*/

#ifdef IMX258_LITEON_CHECK_VENDOR_ID
int is_liteon_camera(void)
{
	kal_uint8 vendor_id = imx258_liteon_read_eeprom(0x01,IMX258_EEPROM_READ_ID);
	if(0x3 == vendor_id)//liteon = 0x3
	{
		return 1;
	}
	return 0;
}
#endif /*IMX258_LITEON_CHECK_VENDOR_ID*/

#if defined(IMX258_LITEON_USE_WB_OTP)
// R/G and B/G of typical camera module is defined here
static int RG_Ratio_Typical_Liteon = 0x01DD;//0x0230;
static int BG_Ratio_Typical_Liteon = 0x0274;//0x0248;
static int GG_Ratio_Typical_Liteon = 0x0400;//0x0405 ;
#define GAIN_DEFAULT       0x0100

// return:  0: OTP value not valid, 1: OTP value valid
static int check_otp_wb(void)
{
    int flag_Pro,flag_AWB;
    int index;
    int bank, address;
    kal_uint8 write_id;

    write_id = 0xA0;

    flag_Pro= imx258_liteon_read_eeprom(0x01,write_id);
    flag_AWB= imx258_liteon_read_eeprom(0x1C,write_id);

    LOG_INF("check_otp_wb, flag_Pro = %x, flag_AWB = %x\n", flag_Pro, flag_AWB);
    if((0x03 == flag_Pro)&&(flag_AWB&0x01))
        return 1;
    else
        return 0;
}
// For HW

// otp_ptr: pointer of otp_struct
// return:  0,
static int read_otp_wb( struct otp_struct * otp_ptr)
{
    /*AWB write_ID:A0, 0x06~0x0B    */
    kal_uint16 otp_data[6]= {0};
    kal_uint16 i=0;
	
    for(i=0; i<6; i++)
    {
        otp_data[i] = imx258_liteon_read_eeprom(0x1D+i,0xA0);
		#ifdef IMX258_LITEON_OTP_DEBUG
			LOG_INF("read_otp_wb addr=0x%x value=0x%x \n",0x1D+i,otp_data[i]);
		#endif
    }

    otp_ptr->rg_ratio = (otp_data[0]<<8)|otp_data[1];
    otp_ptr->bg_ratio = (otp_data[2]<<8)|otp_data[3];
    otp_ptr->gg_ratio = (otp_data[4]<<8)|otp_data[5];
	LOG_INF("%s,r_gain=0x%x b_gain=0x%x g_gain=0x%x \n",__func__,otp_ptr->rg_ratio,otp_ptr->bg_ratio,otp_ptr->gg_ratio);
    return 0;
}


// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int update_awb_gain(u32 R_gain, u32 G_gain_R,u32 G_gain_B, u32 B_gain)
{
    LOG_INF("%s,R_gain=0x%x,G_gain_R=0x%x,G_gain_B=0x%x,B_gain=0x%x \n",__func__,R_gain,G_gain_R,G_gain_B,B_gain);
    IMX258MIPI_write_cmos_sensor(0x020E, G_gain_R>>8);
    IMX258MIPI_write_cmos_sensor(0x020F, G_gain_R& 0xFF);
    IMX258MIPI_write_cmos_sensor(0x0210, R_gain >>8);
    IMX258MIPI_write_cmos_sensor(0x0211, R_gain & 0xFF);
    IMX258MIPI_write_cmos_sensor(0x0212, B_gain >>8);
    IMX258MIPI_write_cmos_sensor(0x0213, B_gain & 0xFF);
    IMX258MIPI_write_cmos_sensor(0x0214, G_gain_B>>8);
    IMX258MIPI_write_cmos_sensor(0x0215, G_gain_B& 0xFF);
    return 0;
}

// call this function after IMX258 initialization
// return value: 0 update success
//      1, no OTP
int imx258_liteon_update_otp_wb(void)
{
    struct otp_struct current_otp;
    int otp_valid;
    u32 R_gain, B_gain, G_gain,G_gain_R, G_gain_B;

    // R/G and B/G of current camera module is read out from sensor OTP
    // check first OTP with valid data
    LOG_INF("imx258_liteon_update_otp_wb,\n");
    otp_valid = check_otp_wb();
    if(otp_valid==0)
    {
        // no valid wb OTP data
        LOG_INF("imx258_liteon_update_otp_wb,no valid wb OTP data\n");
        return 1;
    }

    read_otp_wb(&current_otp);

	if((0 == current_otp.rg_ratio) || (0 ==  current_otp.bg_ratio))
	{
		LOG_INF("imx258_liteon_update_otp_wb get bad otp data !\n current_otp.rg_ratio=0x%x,current_otp.bg_ratio=0x%x",current_otp.rg_ratio,current_otp.bg_ratio);
		return 1;
	}
	if (current_otp.bg_ratio < BG_Ratio_Typical_Liteon) 
	{
        if (current_otp.rg_ratio< RG_Ratio_Typical_Liteon) 
		{
            G_gain = GAIN_DEFAULT;
            B_gain = (GAIN_DEFAULT * BG_Ratio_Typical_Liteon) / current_otp.bg_ratio;
            R_gain = (GAIN_DEFAULT * RG_Ratio_Typical_Liteon) / current_otp.rg_ratio;
        }
		else 
        {
            R_gain = GAIN_DEFAULT;
            G_gain = (GAIN_DEFAULT * current_otp.rg_ratio) / RG_Ratio_Typical_Liteon;
            B_gain = (G_gain * BG_Ratio_Typical_Liteon) /current_otp.bg_ratio;
    	}
    }
	else 
    {
        if (current_otp.rg_ratio < RG_Ratio_Typical_Liteon) 
		{   B_gain = GAIN_DEFAULT;
            G_gain = (GAIN_DEFAULT * current_otp.bg_ratio)/ BG_Ratio_Typical_Liteon;
            R_gain = (G_gain * RG_Ratio_Typical_Liteon)/ current_otp.rg_ratio;
        }
		else 
        {
            G_gain_B = (GAIN_DEFAULT * current_otp.bg_ratio) / BG_Ratio_Typical_Liteon;
            G_gain_R = (GAIN_DEFAULT * current_otp.rg_ratio) / RG_Ratio_Typical_Liteon;

            if(G_gain_B > G_gain_R ) 
			{
                B_gain = GAIN_DEFAULT;
                G_gain = G_gain_B;
                R_gain = (G_gain * RG_Ratio_Typical_Liteon) /current_otp.rg_ratio;
            }else 
            {
                R_gain = GAIN_DEFAULT;
                G_gain = G_gain_R;
                B_gain = (G_gain * BG_Ratio_Typical_Liteon) / current_otp.bg_ratio;
            }
        }
    }

	if (current_otp.gg_ratio <= GG_Ratio_Typical_Liteon)
	{
		G_gain_R = G_gain;
        G_gain_B = G_gain * GG_Ratio_Typical_Liteon / current_otp.gg_ratio;		 
    }
	else 
	{
		G_gain_B = G_gain;
		G_gain_R = G_gain * current_otp.gg_ratio /GG_Ratio_Typical_Liteon;
		R_gain = R_gain * current_otp.gg_ratio /GG_Ratio_Typical_Liteon;
		B_gain = B_gain * current_otp.gg_ratio /GG_Ratio_Typical_Liteon;
	}

    LOG_INF("[yixuhong] R_gain=0x%x,G_gain_R=0x%x, G_gain_B=0x%x,B_gain=0x%x",R_gain,G_gain_R,G_gain_B,B_gain );

    update_awb_gain(R_gain,G_gain_R,G_gain_B, B_gain);

    return 0;
}
#endif /*IMX258_LITEON_USE_WB_OTP*/

#if defined(IMX258_LITEON_USE_LSC_OTP)
// For HW
// index: index of otp group. (0, 1, 2)
// otp_ptr: pointer of otp_struct
// return:  0,
static int read_otp_lenc(int index, u32 * otp_ptr)
{
    kal_uint16 i = 0;

    for(i = 0; i <= 504; i++)
    {
        otp_ptr[i] = imx258_liteon_read_eeprom(0x41+i,IMX258_EEPROM_READ_ID);
		#ifdef IMX258_LITEON_OTP_DEBUG
        LOG_INF("%s:write_id = %x, addr:%x, value:%x \n",__func__,IMX258_EEPROM_READ_ID,0x41+i,otp_ptr[i]);
        #endif
    }
    return 0;
}



// call this function after IMX258 initialization
// otp_ptr: pointer of otp_struct
static int update_lenc(u32 * otp_ptr)
{
    int i;
    //Turn off lsc
   // IMX258MIPI_write_cmos_sensor(0x4500, 0x00);
   IMX258MIPI_write_cmos_sensor(0x0B00, 0x00);
   //IMX258MIPI_write_cmos_sensor(0x3021, 0x00);

    //Access LSC table //table1
    for(i=0; i<504; i++)
    {
        IMX258MIPI_write_cmos_sensor(0xA100+i, otp_ptr[i]);
#ifdef IMX258_LITEON_OTP_DEBUG
        LOG_INF("[yixuhong] otp_lsc_data[%d] == 0x%x\n",i,otp_ptr[i]);
#endif
    }
    //Turn on lsc
    //IMX258MIPI_write_cmos_sensor(0x4500, 0x1f);
    IMX258MIPI_write_cmos_sensor(0x0B00, 0x01);
    IMX258MIPI_write_cmos_sensor(0x3021, 0x00);

    return 0;
}

int imx258_liteon_update_otp_lenc(void)
{
    int otp_valid;
	u32* otp_data_ptr;

	otp_data_ptr=(u32*)kmalloc(504*4, GFP_KERNEL);
	if (NULL == otp_data_ptr)
    {
        LOG_INF("imx258_liteon_update_otp_lenc malloc memory fail\n");
        return 1;
    }
	memset(otp_data_ptr, 0, 504*4);
    // R/G and B/G of current camera module is read out from sensor OTP
    // check first OTP with valid data

    otp_valid = check_otp_wb();
    if(0 == otp_valid)
    {
        // no valid wb OTP data
        return 1;
    }
    read_otp_lenc(otp_valid, otp_data_ptr);

    update_lenc(otp_data_ptr);

	kfree(otp_data_ptr);
    return 0;
}
#endif /*IMX258_LITEON_USE_LSC_OTP*/

static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > IMX258_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(IMX258_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, IMX258_EEPROM_READ_ID)<0)
		return false;
    return true;
}

static bool _read_imx258_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_imx258_liteon_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size){
	addr = 0x0763;
	size = 1404;
	
	LOG_INF("read imx258 eeprom, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx258_eeprom(addr, imx258_liteon_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	
	memcpy(data, imx258_liteon_eeprom_data, size);
    return true;
}

bool read_imx258_liteon_eeprom_SPC( kal_uint16 addr, BYTE* data, kal_uint32 size){
	int i;
	addr = 0x023B;//0x0F73;
	size = 126;
	
	LOG_INF("read imx258 eeprom, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx258_eeprom(addr, imx258_liteon_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	
	memcpy(data, imx258_liteon_eeprom_data, size);
    return true;
}
#endif /*IMX258_LITEON_OTP*/

