#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>

#include "kd_camera_typedef.h"

//#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "hi842_otp.h"
#include "hi842_define.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define USE_OTP_AF_PERCENT
#define AF_INFINITY_PERCENT 80
#define AF_MACRO_PERCENT    130

#define PFX "HI842_OTP_FMT"

//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
  #define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __FUNCTION__, ##arg)
  #define CAM_CALDB(fmt, arg...)     pr_debug("[%s] " fmt, __FUNCTION__, ##arg)
  #define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __FUNCTION__, ##arg)
#else
  #define CAM_CALINF(x,...)
  #define CAM_CALDB(x,...)
  #define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __FUNCTION__, ##arg)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0

/*******************************************************************************
*
********************************************************************************/
#define OTP_READ_SKIP 0 
#define OTP_READ 1 

/*******************************************************************************
*
********************************************************************************/
//#define MTK_WB_USE // Open : MTK WB Cal use //Close : Sensor WB Cal use 

/*******************************************************************************
* Sensor WB Cal . read ratio
********************************************************************************/
int RG_ratio_unit = 0; 
int RG_ratio_golden = 0; 
int BG_ratio_unit = 0; 
int BG_ratio_golden = 0; 

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;

#define MAX_LSC_SIZE 2048
#define MAX_OTP_SIZE 2200
static int Hi842_otp_read = 0;

struct Hynix_OTP_Sensor_reg { 
	u16 addr;
	u16 val;
};

u16 otp_af_data[2];


typedef struct { 
    u8     flag;               // 0
	u32    CaliVer;            // 1~4
	u16    SerialNum;          // 5,6
	u8     Version;            // 7
	u8     AwbAfInfo;          // 8
	u8     UnitAwbR;           // 9
	u8     UnitAwbGr;          // 10
	u8     UnitAwbGb;          // 11
	u8     UnitAwbB;           // 012
	u8     GoldenAwbR;         // 13
	u8     GoldenAwbGr;        // 14
	u8     GoldenAwbGb;        // 15
	u8     GoldenAwbB;         // 16
	u16    AfInfinite;         // 17 18
	u16    AfMacro;            // 19 20
	u16    LscSize;            // 21 22 
	u8     Lsc[MAX_LSC_SIZE];  // 23
	u8     PixelID_forTinno;   // 2070 
}OTP_MTK_TYPE;

typedef union {
        u8 Data[MAX_OTP_SIZE];  
        OTP_MTK_TYPE       MtkOtpData; 
} MTK_OTP_DATA;

typedef struct {
	u8     UnitAwbR;          // 0
	u8     UnitAwbGr;         // 1   
	u8     UnitAwbGb;         // 2 
	u8     UnitAwbB;          // 3
	u8     GoldenAwbR;        // 4
	u8     GoldenAwbGr;       // 5
	u8     GoldenAwbGb;       // 6
	u8     GoldenAwbB;        // 7
	u16    AfInfinite;        // 8,9
	u16    AfMacro;           // 10,11
    u8     R_Ratio;           // 12
    u8     B_Ratio;           // 13
    u8     Gr_Ratio;          // 14
    u8     Gb_Ratio;          // 15
	u8   Lsc[MAX_LSC_SIZE];   // 16
}OTP_QCT_TYPE;

typedef union {
        u8 Data[MAX_OTP_SIZE]; 
        OTP_QCT_TYPE       QctOtpData; 
} QCT_OTP_DATA;

MTK_OTP_DATA hi842_otp_data = {{0}}; 
QCT_OTP_DATA hi842_tmp_data = {{0}}; 

/*******************************************************************************
* I2C Read / Write Func 
********************************************************************************/
extern kal_uint16 read_cmos_sensor_8(kal_uint16 addr);
extern void write_cmos_sensor_16(kal_uint16 addr, kal_uint16 para);
extern void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para);

static void Enable_Hi842_OTP_Read( void)
{
    write_cmos_sensor_16(0x0a00, 0x0000); 
    write_cmos_sensor_16(0x0e00, 0x0002); 
    write_cmos_sensor_16(0x0e02, 0x0002); 
    write_cmos_sensor_16(0x2000, 0x4031);
    write_cmos_sensor_16(0x2002, 0x83f0);
    write_cmos_sensor_16(0x2004, 0x40f2);
    write_cmos_sensor_16(0x2006, 0x000a);
    write_cmos_sensor_16(0x2008, 0x0f90);
    write_cmos_sensor_16(0x200a, 0x43c2);
    write_cmos_sensor_16(0x200c, 0x0f82);
    write_cmos_sensor_16(0x200e, 0x40b1);
    write_cmos_sensor_16(0x2010, 0x1805);
    write_cmos_sensor_16(0x2012, 0x0000);
    write_cmos_sensor_16(0x2014, 0x40b1);
    write_cmos_sensor_16(0x2016, 0x3540);
    write_cmos_sensor_16(0x2018, 0x0002);
    write_cmos_sensor_16(0x201a, 0x40b1);
    write_cmos_sensor_16(0x201c, 0x3540);
    write_cmos_sensor_16(0x201e, 0x0004);
    write_cmos_sensor_16(0x2020, 0x4381);
    write_cmos_sensor_16(0x2022, 0x0006);
    write_cmos_sensor_16(0x2024, 0x4381);
    write_cmos_sensor_16(0x2026, 0x0008);
    write_cmos_sensor_16(0x2028, 0x4381);
    write_cmos_sensor_16(0x202a, 0x000a);
    write_cmos_sensor_16(0x202c, 0x40b1);
    write_cmos_sensor_16(0x202e, 0x0600);
    write_cmos_sensor_16(0x2030, 0x000c);
    write_cmos_sensor_16(0x2032, 0x430d);
    write_cmos_sensor_16(0x2034, 0x4d0e);
    write_cmos_sensor_16(0x2036, 0x5e0e);
    write_cmos_sensor_16(0x2038, 0x4e0f);
    write_cmos_sensor_16(0x203a, 0x510f);
    write_cmos_sensor_16(0x203c, 0x4fae);
    write_cmos_sensor_16(0x203e, 0x0b82);
    write_cmos_sensor_16(0x2040, 0x0c0a);
    write_cmos_sensor_16(0x2042, 0x531d);
    write_cmos_sensor_16(0x2044, 0x903d);
    write_cmos_sensor_16(0x2046, 0x0007);
    write_cmos_sensor_16(0x2048, 0x3bf5);
    write_cmos_sensor_16(0x204a, 0x40b2);
    write_cmos_sensor_16(0x204c, 0x290b);
    write_cmos_sensor_16(0x204e, 0x0b96);
    write_cmos_sensor_16(0x2050, 0x0c0a);
    write_cmos_sensor_16(0x2052, 0x40b2);
    write_cmos_sensor_16(0x2054, 0xc009);
    write_cmos_sensor_16(0x2056, 0x0b98);
    write_cmos_sensor_16(0x2058, 0x0c0a);
    write_cmos_sensor_16(0x205a, 0x0900);
    write_cmos_sensor_16(0x205c, 0x7312);
    write_cmos_sensor_16(0x205e, 0x43d2);
    write_cmos_sensor_16(0x2060, 0x0f82);
    write_cmos_sensor_16(0x2062, 0x0cff);
    write_cmos_sensor_16(0x2064, 0x0cff);
    write_cmos_sensor_16(0x2066, 0x0cff);
    write_cmos_sensor_16(0x2068, 0x0cff);
    write_cmos_sensor_16(0x206a, 0x0cff);
    write_cmos_sensor_16(0x206c, 0x0cff);
    write_cmos_sensor_16(0x206e, 0x0cff);
    write_cmos_sensor_16(0x2070, 0x0cff);
    write_cmos_sensor_16(0x2072, 0x0cff);
    write_cmos_sensor_16(0x2074, 0x0cff);
    write_cmos_sensor_16(0x2076, 0x0cff);
    write_cmos_sensor_16(0x2078, 0x0cff);
    write_cmos_sensor_16(0x207a, 0x0cff);
    write_cmos_sensor_16(0x207c, 0x0cff);
    write_cmos_sensor_16(0x207e, 0x0cff);
    write_cmos_sensor_16(0x2080, 0x0cff);
    write_cmos_sensor_16(0x2082, 0x0cff);
    write_cmos_sensor_16(0x2084, 0x0cff);
    write_cmos_sensor_16(0x2086, 0x40f2);
    write_cmos_sensor_16(0x2088, 0x000e);
    write_cmos_sensor_16(0x208a, 0x0f90);
    write_cmos_sensor_16(0x208c, 0x43d2);
    write_cmos_sensor_16(0x208e, 0x0180);
    write_cmos_sensor_16(0x2090, 0x4392);
    write_cmos_sensor_16(0x2092, 0x760e);
    write_cmos_sensor_16(0x2094, 0x9382);
    write_cmos_sensor_16(0x2096, 0x760c);
    write_cmos_sensor_16(0x2098, 0x2002);
    write_cmos_sensor_16(0x209a, 0x0c64);
    write_cmos_sensor_16(0x209c, 0x3ffb);
    write_cmos_sensor_16(0x209e, 0x421f);
    write_cmos_sensor_16(0x20a0, 0x760a);
    write_cmos_sensor_16(0x20a2, 0x931f);
    write_cmos_sensor_16(0x20a4, 0x2013);
    write_cmos_sensor_16(0x20a6, 0x421b);
    write_cmos_sensor_16(0x20a8, 0x018a);
    write_cmos_sensor_16(0x20aa, 0x4b82);
    write_cmos_sensor_16(0x20ac, 0x7600);
    write_cmos_sensor_16(0x20ae, 0x0260);
    write_cmos_sensor_16(0x20b0, 0x0000);
    write_cmos_sensor_16(0x20b2, 0x0c4c);
    write_cmos_sensor_16(0x20b4, 0x0240);
    write_cmos_sensor_16(0x20b6, 0x0000);
    write_cmos_sensor_16(0x20b8, 0x0260);
    write_cmos_sensor_16(0x20ba, 0x0000);
    write_cmos_sensor_16(0x20bc, 0x0c18);
    write_cmos_sensor_16(0x20be, 0x4b0f);
    write_cmos_sensor_16(0x20c0, 0x12b0);
    write_cmos_sensor_16(0x20c2, 0xf11a);
    write_cmos_sensor_16(0x20c4, 0x4fc2);
    write_cmos_sensor_16(0x20c6, 0x0188);
    write_cmos_sensor_16(0x20c8, 0x4b0c);
    write_cmos_sensor_16(0x20ca, 0x3fe2);
    write_cmos_sensor_16(0x20cc, 0x903f);
    write_cmos_sensor_16(0x20ce, 0x0201);
    write_cmos_sensor_16(0x20d0, 0x23df);
    write_cmos_sensor_16(0x20d2, 0x531b);
    write_cmos_sensor_16(0x20d4, 0x4b0e);
    write_cmos_sensor_16(0x20d6, 0x108e);
    write_cmos_sensor_16(0x20d8, 0xf37e);
    write_cmos_sensor_16(0x20da, 0xc312);
    write_cmos_sensor_16(0x20dc, 0x100e);
    write_cmos_sensor_16(0x20de, 0x110e);
    write_cmos_sensor_16(0x20e0, 0x110e);
    write_cmos_sensor_16(0x20e2, 0x110e);
    write_cmos_sensor_16(0x20e4, 0x4c0f);
    write_cmos_sensor_16(0x20e6, 0x108f);
    write_cmos_sensor_16(0x20e8, 0xf37f);
    write_cmos_sensor_16(0x20ea, 0xc312);
    write_cmos_sensor_16(0x20ec, 0x100f);
    write_cmos_sensor_16(0x20ee, 0x110f);
    write_cmos_sensor_16(0x20f0, 0x110f);
    write_cmos_sensor_16(0x20f2, 0x110f);
    write_cmos_sensor_16(0x20f4, 0x9f0e);
    write_cmos_sensor_16(0x20f6, 0x27e3);
    write_cmos_sensor_16(0x20f8, 0x0261);
    write_cmos_sensor_16(0x20fa, 0x0000);
    write_cmos_sensor_16(0x20fc, 0x4b82);
    write_cmos_sensor_16(0x20fe, 0x7600);
    write_cmos_sensor_16(0x2100, 0x0260);
    write_cmos_sensor_16(0x2102, 0x0000);
    write_cmos_sensor_16(0x2104, 0x0c4c);
    write_cmos_sensor_16(0x2106, 0x0240);
    write_cmos_sensor_16(0x2108, 0x0000);
    write_cmos_sensor_16(0x210a, 0x0260);
    write_cmos_sensor_16(0x210c, 0x0000);
    write_cmos_sensor_16(0x210e, 0x0c18);
    write_cmos_sensor_16(0x2110, 0x3fd6);
    write_cmos_sensor_16(0x2112, 0x5031);
    write_cmos_sensor_16(0x2114, 0x0010);
    write_cmos_sensor_16(0x2116, 0x4030);
    write_cmos_sensor_16(0x2118, 0xf13a);
    write_cmos_sensor_16(0x211a, 0x4382);
    write_cmos_sensor_16(0x211c, 0x7602);
    write_cmos_sensor_16(0x211e, 0x4f82);
    write_cmos_sensor_16(0x2120, 0x7600);
    write_cmos_sensor_16(0x2122, 0x0270);
    write_cmos_sensor_16(0x2124, 0x0000);
    write_cmos_sensor_16(0x2126, 0x0c17);
    write_cmos_sensor_16(0x2128, 0x0270);
    write_cmos_sensor_16(0x212a, 0x0001);
    write_cmos_sensor_16(0x212c, 0x403e);
    write_cmos_sensor_16(0x212e, 0x7606);
    write_cmos_sensor_16(0x2130, 0x4e2f);
    write_cmos_sensor_16(0x2132, 0x4fc2);
    write_cmos_sensor_16(0x2134, 0x0188);
    write_cmos_sensor_16(0x2136, 0x4e2f);
    write_cmos_sensor_16(0x2138, 0x4130);
    write_cmos_sensor_16(0x213a, 0xdf02);
    write_cmos_sensor_16(0x213c, 0x3ffe);
    write_cmos_sensor_16(0x213e, 0x0000);
    write_cmos_sensor_16(0x2ffe, 0xf000);
    write_cmos_sensor_16(0x0d00, 0x0707);
    write_cmos_sensor_16(0x0d04, 0x0100);
    write_cmos_sensor_16(0x004c, 0x0100); 
    write_cmos_sensor_16(0x0a00, 0x0100); 

}

static void Disble_Hi842_OTP_Read(void) 
{
    write_cmos_sensor_16(0x0a00, 0x0000);
}

int read_Hi842_Ratio(u8 enable)
{
// Tinno Project dows not use raio value. 
#if 0 
    int Bank_No = 0 ;      
    int Start_Add = 0 ;   
    int Read_Size = 0;    
    int i = 0;
    int Read_Data = 0; 
    
    if( OTP_READ_SKIP == enable )
      return 0; 

// OTP Flag Read
    write_cmos_sensor_16(0x010a, 0x1D89); 
    write_cmos_sensor_8(0x0102,0x01);    
    Bank_No = read_cmos_sensor_8(0x0108);
    CAM_CALDB("read_Hi842_Ratio -------> 0x0890(8) is 0x%x \n",Bank_No);
    
// Bank Select 
    switch(Bank_No)
    {
        case 0x01: Start_Add = 0x1D8E; 
            break ;
        case 0x13: Start_Add = 0x1D9B; 
            break;
        case 0x37: Start_Add = 0x1DA8; 
            break;
        default : // Error .. 
            CAM_CALDB("read_Hi842_Ratio -> Bank_No Read ERROR !! \n");
            return 0; // does not check ratio flag. -> return 0 
    }
       
    write_cmos_sensor_16(0x010a, Start_Add); 
    write_cmos_sensor_8(0x0102,0x01);

    for(i = 0; i < 4; i++)
    {
        Read_Data = read_cmos_sensor_8(0x0108); 
        hi842_tmp_data.Data[i+12] = Read_Data;

        //CAM_CALDB("read_Hi842_Ratio(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
        Start_Add++;
	}  

    return 0;
#endif     
}

 
int read_Hi842_Chip_Info(u8 enable)
{
    int Bank_No = 0 ; 
    int Start_Add = 0 ;   
    int Read_Size = 0;   
    int i = 0;
    int Read_Data = 0; 
    int DataSum = 0 ; 
    int CheckSum_Add = 0;
    
    if( OTP_READ_SKIP == enable )
      return 0; // Data Read Skip 

// OTP Flag Read
    write_cmos_sensor_16(0x010a, CI_FLAG_A); 
    write_cmos_sensor_8(0x0102,0x01); 
    Bank_No = read_cmos_sensor_8(0x0108);
#ifdef HI842_CHIPINFO_DBG    
    CAM_CALDB("read_Hi842_Chip_Info -------> .0x%x is 0x%x \n",CI_FLAG_A,Bank_No);
#endif     
// Bank Select 
    switch(Bank_No)
    {
        case 0x01: Start_Add = CI_B1_SA; CheckSum_Add = CI_CS1_SA;
            break ;
        case 0x13: Start_Add = CI_B2_SA; CheckSum_Add = CI_CS2_SA;
            break;
        case 0x37: Start_Add = CI_B3_SA; CheckSum_Add = CI_CS3_SA;
            break;
        default : // Error .. 
#ifdef HI842_CHIPINFO_DBG            
            CAM_CALDB("read_Hi842_Chip_Info -> Bank_No Read ERROR !! \n");
#endif 
            return 1; 
    }
       
    write_cmos_sensor_16(0x010a, Start_Add); 
    write_cmos_sensor_8(0x0102,0x01); 
	for(i = 0; i < 11; i++)
    {
        Read_Data = read_cmos_sensor_8(0x0108);
        DataSum += Read_Data;

        if( i == 10) 
        {
            hi842_otp_data.Data[2070] = Read_Data;
#ifdef HI842_CHIPINFO_DBG                
            CAM_CALDB("hi842_otp_data.Data[2070] is 0x%x\n",hi842_otp_data.Data[2070]);
#endif             
        }
#ifdef HI842_CHIPINFO_DBG    
        CAM_CALDB("read_Hi842_Chip_Info(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
#endif 
        Start_Add++;
	}  

	DataSum = (BYTE)((DataSum%0xFF)+1); 
	
	write_cmos_sensor_16(0x010a, CheckSum_Add);  
   	write_cmos_sensor_8(0x0102,0x01); 
	Read_Data = read_cmos_sensor_8(0x0108); 

#if(HI842_CHECKSUM_ENABLE == HI842_CHECKSUM_ON)
	if(Read_Data != DataSum) 
		return 1;  
#endif 

#ifdef HI842_CHIPINFO_DBG    	
        CAM_CALDB("read_Hi842_Chip_InfoData Sum(%d) Read_Data(%d) -------> \n",DataSum,Read_Data); 
#endif         
    return 0;
}



int read_Hi842_AF_Info(u8 enable)
{
    int Bank_No = 0 ; 
    int Start_Add = 0 ;   
    int Read_Size = 0;    
    int i = 0;
    int Read_Data = 0; 
    int DataSum = 0 ; 
    int CheckSum_Add = 0;
    u32 iTmp = 0;
    
    if( OTP_READ_SKIP == enable )
      return 0; 

// OTP Flag Read
    write_cmos_sensor_16(0x010a, AF_FLAG_A);
    write_cmos_sensor_8(0x0102,0x01); 
    Bank_No = read_cmos_sensor_8(0x0108);
#ifdef HI842_AF_DBG    
    CAM_CALDB(" read_Hi842_AF_Info -------> .0x%x is 0x%x \n",AF_FLAG_A,Bank_No);
#endif     
// Bank Select 
    switch(Bank_No)
    {
        case 0x01: Start_Add = AF_B1_SA; CheckSum_Add = AF_CS1_SA;
            break ;
        case 0x13: Start_Add = AF_B2_SA; CheckSum_Add = AF_CS2_SA;
            break;
        case 0x37: Start_Add = AF_B3_SA; CheckSum_Add = AF_CS3_SA;
            break;
        default : // Error .. 
#ifdef HI842_AF_DBG    
            CAM_CALDB("read_Hi842_AF_Info -> Bank_No Read ERROR !! \n");
#endif 
            return 1; 
    }
       
    write_cmos_sensor_16(0x010a, Start_Add); 
    write_cmos_sensor_8(0x0102,0x01);  
	for(i = 0; i < 10; i++) 
    {
        Read_Data = read_cmos_sensor_8(0x0108); 
        DataSum += Read_Data; 
        if(i<4) 
        {
            hi842_otp_data.Data[i+17] = Read_Data;
#ifdef HI842_AF_DBG    
            CAM_CALDB("read_Hi842_AF_Info(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
#endif 
        }
        Start_Add++;
	}  

	otp_af_data[0]=(hi842_otp_data.Data[17]<<8)|hi842_otp_data.Data[18];
	otp_af_data[1]=(hi842_otp_data.Data[19]<<8)|hi842_otp_data.Data[20];
	CAM_CALDB("HJDDbgAF, HYNIX_CAM_CAL otp_af_data[0]= 0x%x(%d), otp_af_data[1] = 0x%x(%d)\n",otp_af_data[0],otp_af_data[0],otp_af_data[1],otp_af_data[1]);
	DataSum = (BYTE)((DataSum%0xFF)+1); 

#ifdef USE_OTP_AF_PERCENT
    iTmp = otp_af_data[0] * AF_INFINITY_PERCENT / 100;
    otp_af_data[0] = (u16)iTmp;

    iTmp = otp_af_data[1] * AF_MACRO_PERCENT / 100;
    if (iTmp > 1023) {
        iTmp = 1023;
    }
    otp_af_data[1] = (u16)iTmp;
    
	CAM_CALDB("HJDDbgAF Caution!! after percent, HYNIX_CAM_CAL otp_af_data[0]= 0x%x(%d), otp_af_data[1] = 0x%x(%d)\n",otp_af_data[0],otp_af_data[0],otp_af_data[1],otp_af_data[1]);
#endif
    
	write_cmos_sensor_16(0x010a, CheckSum_Add); 
   	write_cmos_sensor_8(0x0102,0x01); 
	Read_Data = read_cmos_sensor_8(0x0108); 

#if(HI842_CHECKSUM_ENABLE == HI842_CHECKSUM_ON)
	if(Read_Data != DataSum) 
		return 1;  
#endif 

#ifdef HI842_AF_DBG    
        CAM_CALDB("DataSum(%d) Read_Data(%d) -------> \n",DataSum,Read_Data); 
#endif 

    return 0;
}

int read_QCT_Hi842_LSC_Info(u8 enable)
{
    int Bank_No = 0 ; 
    int Start_Add = 0 ;  
    int Read_Size = 1768;  
    int i = 0;
    int Read_Data = 0; 
    int DataSum = 0 ;
    int CheckSum_Add = 0;
    if( OTP_READ_SKIP == enable )
       return 0;

// OTP Flag Read
    write_cmos_sensor_16(0x010a, LSC_FLAG_A);
    write_cmos_sensor_8(0x0102,0x01);

    Bank_No = read_cmos_sensor_8(0x0108);
#ifdef HI842_LSC_DBG    
    CAM_CALDB("read_Hi842_LSC_Info -------> .0x%x(8) is 0x%x \n",LSC_FLAG_A,Bank_No);
#endif 

// Bank Select 
    switch(Bank_No)
    {
        case 0x01: Start_Add = LSC_B1_SA; CheckSum_Add = LSC_CS1_SA;
            break ;
        case 0x13: Start_Add = LSC_B2_SA; CheckSum_Add = LSC_CS2_SA;
            break;
        case 0x37: Start_Add = LSC_B3_SA; CheckSum_Add = LSC_CS3_SA;
            break;
        default : // Error .. 
#ifdef HI842_LSC_DBG    
            CAM_CALDB("read_Hi842_LSC_Info -> Bank_No Read ERROR !! \n");
#endif 
            return 1; 
    }
	   
    write_cmos_sensor_16(0x010a, Start_Add);
    write_cmos_sensor_8(0x0102,0x01);

    for(i = 0; i < 1768; i++)
    {
        Read_Data = read_cmos_sensor_8(0x0108); 
        DataSum += Read_Data;

        // LSB / MSB Data Change 
		if (i % 2) // odd
		{
			hi842_tmp_data.Data[i+16-1] = Read_Data;
		}
		else // Even
		{
			hi842_tmp_data.Data[i+16+1] = Read_Data;
		}
#ifdef HI842_LSC_DBG    
        CAM_CALDB("read_Hi842_LSC_Info(%d) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
#endif 
        Start_Add++;
    }
	
	DataSum = (BYTE)((DataSum%0xFF)+1);  
	
	write_cmos_sensor_16(0x010a, CheckSum_Add); 
   	write_cmos_sensor_8(0x0102,0x01);           
	Read_Data = read_cmos_sensor_8(0x0108);    


#if(HI842_CHECKSUM_ENABLE == HI842_CHECKSUM_ON) 
	if(Read_Data != DataSum)
    	return 1;  
#endif 


    
#ifdef HI842_LSC_DBG    
        CAM_CALDB("DataSum(%d) Read_Data(%d) -------> \n",DataSum,Read_Data);
#endif 
    return 0;
}

int read_Hi842_WB_Info(u8 enable)
{
    int Bank_No = 0 ; 
    int Start_Add = 0;
    int Read_Size = 12;
    int i = 0;
    int Read_Data = 0; 
    int DataSum = 0 ; 
    int CheckSum_Add = 0; 
    
    if( OTP_READ_SKIP == enable )
      return 0; 
      
    write_cmos_sensor_16(0x010a, WB_FLAG_A);
    write_cmos_sensor_8(0x0102,0x01); 
    Bank_No = read_cmos_sensor_8(0x0108);

#ifdef HI842_WB_DBG    
    CAM_CALDB("read_Hi842_WB_Info -------> .0x%x data is 0x%x \n",WB_FLAG_A,Bank_No);
#endif 

// Read WB Data 
    switch(Bank_No)
    {
        case 0x01: Start_Add = WB_B1_SA; CheckSum_Add = WB_CS1_SA;
            break ;
        case 0x13: Start_Add = WB_B2_SA; CheckSum_Add = WB_CS2_SA; 
            break;
        case 0x37: Start_Add = WB_B3_SA; CheckSum_Add = WB_CS3_SA; 
            break;
        default : // Error .. 
#ifdef HI842_WB_DBG    
            CAM_CALDB("read_Hi842_WB_Info -> Bank_No Read ERROR !! \n");
#endif 
            return 1; 
    }

    write_cmos_sensor_16(0x010a, Start_Add); 
    write_cmos_sensor_8(0x0102,0x01);

    for(i = 0; i < Read_Size; i++)
    {
        Read_Data = read_cmos_sensor_8(0x0108);
        DataSum += Read_Data;

	if(i<4)
       	 hi842_otp_data.Data[i+9] = Read_Data;

	if((i > 5) && (i <10))
	        hi842_otp_data.Data[i+7] = Read_Data;

#ifdef HI842_WB_DBG    
	 CAM_CALDB("read_Hi842_WB_Info(0x%x) -------> add 0x%x, Read_Data is 0x%x\n",i,Start_Add,Read_Data);
#endif 
        Start_Add++;
    }

#if (HI842_WB_METHOD == HI842_GOLDEN_WB_SW) 
		hi842_otp_data.Data[13] = 0x03; 
		hi842_otp_data.Data[14] = 0x49; 	
		hi842_otp_data.Data[15] = 0x02; 
		hi842_otp_data.Data[16] = 0xa4; 
#endif 

#ifdef HI842_WB_DBG    
	CAM_CALDB("read_Hi842_WB_Info [9]  = (0x%x)\n",hi842_otp_data.Data[9]);
	CAM_CALDB("read_Hi842_WB_Info [10] = (0x%x)\n",hi842_otp_data.Data[10]);
	CAM_CALDB("read_Hi842_WB_Info [11] = (0x%x)\n",hi842_otp_data.Data[11]);
	CAM_CALDB("read_Hi842_WB_Info [12] = (0x%x)\n",hi842_otp_data.Data[12]);
	CAM_CALDB("read_Hi842_WB_Info [13] = (0x%x)\n",hi842_otp_data.Data[13]);
	CAM_CALDB("read_Hi842_WB_Info [14] = (0x%x)\n",hi842_otp_data.Data[14]);
	CAM_CALDB("read_Hi842_WB_Info [15] = (0x%x)\n",hi842_otp_data.Data[15]);
	CAM_CALDB("read_Hi842_WB_Info [16] = (0x%x)\n",hi842_otp_data.Data[16]);
#endif 

	DataSum = (BYTE)((DataSum%0xFF)+1);
	
	write_cmos_sensor_16(0x010a, CheckSum_Add); 
   	write_cmos_sensor_8(0x0102,0x01);
	Read_Data = read_cmos_sensor_8(0x0108);

#if(HI842_CHECKSUM_ENABLE == HI842_CHECKSUM_ON) 
	if(Read_Data != DataSum)
		return 1;  
#endif 	

#ifdef HI842_WB_DBG    
        CAM_CALDB("DataSum(%d) Read_Data(%d) -------> \n",DataSum,Read_Data); 
#endif 
     return 0;
}

int Get_Hi842_WB_Info(u8 enable)
{

    int Bank_No = 0 ; 
    int Start_Add = 0;     
    int Read_Size = 12;    
    int i = 0;
    int Read_Data_a = 0; 
    int Read_Data_b = 0; 	
    int DataSum = 0 ;
    int CheckSum_Add = 0;

    int Read_Checksum_Data = 0 ;
    int Gold_R_ratio = 0;   
    int Gold_B_ratio = 0;   
    int Gold_G_ratio = 0;   

    int Unit_R_ratio = 0;   
    int Unit_B_ratio = 0;   
    int Unit_G_ratio = 0;   
    int Temp_ratio = 0;
    
    int Ru = 0;
    int Bu = 0;
    int Gru = 0;
    int Gbu = 0;
    int Gavu = 0; 
	
    int Rg = 0;
    int Bg = 0;
    int Grg = 0;
    int Gbg = 0;
    int Gavg = 0; 
    
    if( OTP_READ_SKIP == enable )
      return 0; // Data Read Skip 
      
// Read OTP Flag Data 
    write_cmos_sensor_16(0x010a, WB_FLAG_A);
    write_cmos_sensor_8(0x0102,0x01);    
    Bank_No = read_cmos_sensor_8(0x0108);
#ifdef HI842_WB_DBG    
    CAM_CALDB("read_Hi842_WB_Info -------> .0x%x data is 0x%x \n",WB_FLAG_A,Bank_No);
#endif     
// Read WB Data 
    switch(Bank_No)
    {
        case 0x01: Start_Add = WB_B1_SA;
			    CheckSum_Add = WB_CS1_SA;
            break ;
        case 0x13: Start_Add = WB_B2_SA;
		           CheckSum_Add = WB_CS2_SA;
            break;
        case 0x37: Start_Add = WB_B3_SA;
		            CheckSum_Add = WB_CS3_SA;
            break;
        default : // Error .. 
#ifdef HI842_WB_DBG        
            CAM_CALDB("read_Hi842_WB_Info -> Bank_No Read ERROR !! \n");
#endif 
            return 1; 
    }

    write_cmos_sensor_16(0x010a, Start_Add);
    write_cmos_sensor_8(0x0102,0x01);

    Read_Data_a = read_cmos_sensor_8(0x0108); DataSum += Read_Data_a;	
    Read_Data_b = read_cmos_sensor_8(0x0108); DataSum += Read_Data_b;  
    Unit_R_ratio = Read_Data_a<<8 | Read_Data_b; 

    Read_Data_a = read_cmos_sensor_8(0x0108);  DataSum += Read_Data_a;
    Read_Data_b = read_cmos_sensor_8(0x0108);  DataSum += Read_Data_b;
    Unit_B_ratio = Read_Data_a<<8 | Read_Data_b; 

    Read_Data_a = read_cmos_sensor_8(0x0108);  DataSum += Read_Data_a;
    Read_Data_b = read_cmos_sensor_8(0x0108);  DataSum += Read_Data_b;
    Unit_G_ratio = Read_Data_a<<8 | Read_Data_b; 

    Read_Data_a = read_cmos_sensor_8(0x0108); DataSum += Read_Data_a;
    Read_Data_b = read_cmos_sensor_8(0x0108); DataSum += Read_Data_b;
    Gold_R_ratio = Read_Data_a<<8 | Read_Data_b; 

    Read_Data_a = read_cmos_sensor_8(0x0108);  DataSum += Read_Data_a;
    Read_Data_b = read_cmos_sensor_8(0x0108);  DataSum += Read_Data_b;
    Gold_B_ratio = Read_Data_a<<8 | Read_Data_b; 

    Read_Data_a = read_cmos_sensor_8(0x0108);  DataSum += Read_Data_a;
    Read_Data_b = read_cmos_sensor_8(0x0108);  DataSum += Read_Data_b;
    Gold_G_ratio = Read_Data_a<<8 | Read_Data_b; 

#if(HI842_WB_METHOD == HI842_GOLDEN_WB_SW)
    Gold_R_ratio = 0x34B;// 0x34B JINDY, old:0x349;
    Gold_B_ratio = 0x29D;//0x29D, old:0x2a4
    Gold_G_ratio = 0x3FB; // 0x3FB, old:0x3fa
#endif 

#ifdef HI842_WB_DBG    
	CAM_CALDB("Unit_R_ratio(0x%x) \n",Unit_R_ratio); 
	CAM_CALDB("Unit_B_ratio(0x%x) \n",Unit_B_ratio); 
	CAM_CALDB("Unit_G_ratio(0x%x) \n",Unit_G_ratio);
	
	CAM_CALDB("Gold_R_ratio(0x%x) \n",Gold_R_ratio);
	CAM_CALDB("Gold_B_ratio(0x%x) \n",Gold_B_ratio);
	CAM_CALDB("Gold_G_ratio(0x%x) \n",Gold_G_ratio);
#endif 

#if 0 
	Ru = Unit_R_ratio * 800 / 1024 ; 
	Bu = Unit_B_ratio * (Unit_G_ratio * 800 / 1024)/1024;
       Gru = 800; 
	Gbu =  Unit_G_ratio * 800 /1024 ;   
       Gavu = (Gru + Gbu ) / 2; 
	   
	Rg = Gold_R_ratio * 800 / 1024 ; 
	Bg = Gold_B_ratio * (Gold_G_ratio * 800 / 1024)/1024;
       Grg = 800; 
	Gbg =  Gold_G_ratio * 800 /1024 ;  
	Gavg = (Grg + Gbg ) / 2; 
	
	RG_ratio_unit =1024 * (Ru-64)/(Gavu-64); 
	RG_ratio_golden = 1024 * (Rg-64)/(Gavg-64); 

	BG_ratio_unit = 1024 * (Bu-64)/(Gavu-64);
	BG_ratio_golden = 1024 * (Bg-64)/(Gavg-64);
#else
	RG_ratio_unit =Unit_R_ratio; 
	RG_ratio_golden = Gold_R_ratio; 

	BG_ratio_unit = Unit_B_ratio;
	BG_ratio_golden = Gold_B_ratio;

    printk("HJDDbgAWB OTP, Unit_R_ratio=0x%x(Gold=0x%x), Unit_B_ratio=0x%x(Gold=0x%x) \n"
        , Unit_R_ratio, Gold_R_ratio, Unit_B_ratio, Gold_B_ratio);
#endif 

	DataSum = (BYTE)((DataSum%0xFF)+1); 
	
	write_cmos_sensor_16(0x010a, CheckSum_Add);
   	write_cmos_sensor_8(0x0102,0x01);
	Read_Checksum_Data = read_cmos_sensor_8(0x0108);

#if(HI842_CHECKSUM_ENABLE == HI842_CHECKSUM_ON) 
	if(Read_Checksum_Data != DataSum) 
        return 1;  
#endif

#ifdef HI842_WB_DBG    
        CAM_CALDB("DataSum(%d) Read_Data(%d) -------> \n",DataSum,Read_Checksum_Data);
#endif 
     return 0;
}

void set_Hi842_LSC_Table(void)
{
    hi842_otp_data.Data[23] = 0xff; // SlimLscType, for more detals, see CAM_CAL_LSC_MTK_TYPE in camera_custom_cam_cal.h
    hi842_otp_data.Data[24] = 0x00; 
    hi842_otp_data.Data[25] = 0x02; 
    hi842_otp_data.Data[26] = 0x01; 
    
    hi842_otp_data.Data[27] = 0x08; // PreviewWH 1288 772
    hi842_otp_data.Data[28] = 0x05; 
    hi842_otp_data.Data[29] = 0xC4; 
    hi842_otp_data.Data[30] = 0x03; 

    hi842_otp_data.Data[31] = 0x0C; // PreviewOffSet 12 12
    hi842_otp_data.Data[32] = 0x00; 
    hi842_otp_data.Data[33] = 0x0C; 
    hi842_otp_data.Data[34] = 0x00; 
    
    hi842_otp_data.Data[35] = 0x60; // CaptureWH 1632 1224
    hi842_otp_data.Data[36] = 0x06; 
    hi842_otp_data.Data[37] = 0xC8; 
    hi842_otp_data.Data[38] = 0x04; 
    
    hi842_otp_data.Data[39] = 0x0A; // CaptureOffSet 10 10
    hi842_otp_data.Data[40] = 0x00; 
    hi842_otp_data.Data[41] = 0x0A; 
    hi842_otp_data.Data[42] = 0x00; 
    
    hi842_otp_data.Data[43] = 0x00; // PreviewTblSize
    hi842_otp_data.Data[44] = 0x00; 
    hi842_otp_data.Data[45] = 0x00; 
    hi842_otp_data.Data[46] = 0x00; 
    
    hi842_otp_data.Data[47] = 0xC6; // CaptureTblSize
    hi842_otp_data.Data[48] = 0x00; 
    hi842_otp_data.Data[49] = 0x00; 
    hi842_otp_data.Data[50] = 0x00; 

    hi842_otp_data.Data[51] = 0x00; // PvIspReg[0]
    hi842_otp_data.Data[52] = 0x00; 
    hi842_otp_data.Data[53] = 0x00; 
    hi842_otp_data.Data[54] = 0x30; 
    
    hi842_otp_data.Data[55] = 0x78; // PvIspReg[1]
    hi842_otp_data.Data[56] = 0x30; 
    hi842_otp_data.Data[57] = 0x80; 
    hi842_otp_data.Data[58] = 0x40; 
    
    hi842_otp_data.Data[59] = 0x00; // PvIspReg[2]
    hi842_otp_data.Data[60] = 0x00; 
    hi842_otp_data.Data[61] = 0x00; 
    hi842_otp_data.Data[62] = 0x40; 
    
    hi842_otp_data.Data[63] = 0x7A; // PvIspReg[3]
    hi842_otp_data.Data[64] = 0x00; 
    hi842_otp_data.Data[65] = 0x84; 
    hi842_otp_data.Data[66] = 0x00; 
    
    hi842_otp_data.Data[67] = 0x20; // PvIspReg[4]
    hi842_otp_data.Data[68] = 0x20; 
    hi842_otp_data.Data[69] = 0x20; 
    hi842_otp_data.Data[70] = 0x20;

    hi842_otp_data.Data[71] = 0x00; // CapIspReg[0]
    hi842_otp_data.Data[72] = 0x00; 
    hi842_otp_data.Data[73] = 0x00; 
    hi842_otp_data.Data[74] = 0x30; 
    
    hi842_otp_data.Data[75] = 0x3D; // CapIspReg[1]
    hi842_otp_data.Data[76] = 0x90; 
    hi842_otp_data.Data[77] = 0x66; 
    hi842_otp_data.Data[78] = 0x70; 
    
    hi842_otp_data.Data[79] = 0x00; // CapIspReg[2]
    hi842_otp_data.Data[80] = 0x00; 
    hi842_otp_data.Data[81] = 0x00; 
    hi842_otp_data.Data[82] = 0x40; 
    
    hi842_otp_data.Data[83] = 0x3F; // CapIspReg[3]
    hi842_otp_data.Data[84] = 0x00; 
    hi842_otp_data.Data[85] = 0x66; 
    hi842_otp_data.Data[86] = 0x00; 
    
    hi842_otp_data.Data[87] = 0x20; // CapIspReg[4]
    hi842_otp_data.Data[88] = 0x20; 
    hi842_otp_data.Data[89] = 0x20; 
    hi842_otp_data.Data[90] = 0x20;
    
}

int read_Hi842_otp(void)
{ 
	int i = 0;
	int offset = 0;
    int rec = 0 ; 
    
	CAM_CALDB("start HI842 OTP read \n");

	if(Hi842_otp_read ==1)
	{
		return 0;  // temp code must open  
	} 
	
    Enable_Hi842_OTP_Read(); // Enable OTP Read Init Register Write 

    // Check Data Correction 
    hi842_otp_data.Data[0] = 0x01; 

    // Calibration Version 
    hi842_otp_data.Data[1] = 0xff; 
    hi842_otp_data.Data[2] = 0x00; 
    hi842_otp_data.Data[3] = 0x08; 
    hi842_otp_data.Data[4] = 0x42; 

    // Serial Number
    hi842_otp_data.Data[5] = 0x00;  
    hi842_otp_data.Data[6] = 0x00; 

    // AWB / AF Calibration Information 
    hi842_otp_data.Data[7] = 0x01;

    /////////////////////////////////////////////////////////////////////////////////
    // If you want to use MTK AF Cal Only, open below code
    hi842_otp_data.Data[8] = 0x0e; 
    // If you want to use MTK WB Cal Only, open below code   
    // hi842_otp_data.Data[8] = 0x01; 
    /////////////////////////////////////////////////////////////////////////////////

    // WB Value Read, hi842_otp_data.Data[9-16]
#ifdef MTK_WB_USE	
    rec |= read_Hi842_WB_Info(OTP_READ); // Tinno does not use mtk wb cal. -> close 
#else 
    rec |= Get_Hi842_WB_Info(OTP_READ);  // Tinno use sensor wb cal -> open
#endif 

    // AF Information Read, hi842_otp_data.Data[17-20] Infinity:17-18 Macro:19-20, MSB LSB
    rec |= read_Hi842_AF_Info(OTP_READ);
    // For tinno, READ 
    rec |= read_Hi842_Chip_Info(OTP_READ); // hi842_otp_data.Data[2070] 

    // Set LSC Table Size 
    hi842_otp_data.Data[21] = 0x5c;
    hi842_otp_data.Data[22] = 0x03;

    // Set LSC Table.
    set_Hi842_LSC_Table(); // hi842_otp_data.Data[23-90]
    rec |= read_QCT_Hi842_LSC_Info(OTP_READ);
	
    if(rec > 0) // Read Fail 
    {
        CAM_CALINF(" Final Read Fail. ---------- \n");
    
        hi842_otp_data.Data[0] = 0x00; 
        hi842_otp_data.Data[1] = 0x00; 
        hi842_otp_data.Data[2] = 0x00;
        hi842_otp_data.Data[3] = 0x00; 
        hi842_otp_data.Data[4] = 0x00; 
    }
    else // Read Pass       
    {
        CAM_CALINF(" Final Read Pass. ---------- \n");   
       Hi842_otp_read = 1 ;
    }
    
    Disble_Hi842_OTP_Read();
    return 0 ; 

}


#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long Hi842otp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
    
    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
    {
        CAM_CALERR("Hi842otp_Ioctl_Compat -> return \n");
        return -ENOTTY;
    }

    switch (cmd) {
        
//----------------------------------------------------------------------------------------------------------------//            

    case COMPAT_CAM_CALIOC_H_READ:
    {
        CAM_CALERR("otp_Ioctl_Compat--COMPAT_CAM_CALIOC_H_READ \n");
     
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_H_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }

//----------------------------------------------------------------------------------------------------------------//            

    case COMPAT_CAM_CALIOC_G_READ:
    {
        CAM_CALERR("Hi842otp_Ioctl_Compat--COMPAT_CAM_CALIOC_G_READ \n");
        
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }

//----------------------------------------------------------------------------------------------------------------//            
    
    default:
        return -ENOIOCTLCMD;
    }
}
#endif

static int selective_Q_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{    
    CAM_CALDB("QCT DATA LOAD offset =%x size %d data read = %d\n", offset,size, *data);
    memcpy((void *)data,(void *)&hi842_tmp_data.Data[offset] ,size);
    return size;
}

static int selective_read_region(u32 offset, BYTE* data, u16 i2c_id, u32 size)
{    
    CAM_CALDB("[HYNIX_CAM_CAL] selective_read_region offset=%d, size=%d, data=%p \n", offset,size, data);

    if (1 == size && 8 == offset)
	{
        u8 AWBAFConfig = 0x2; // AF only
        
        CAM_CALDB("[HYNIX_CAM_CAL] HJDDbg, AWBAFConfig=%d(1=AWB, 2=AF) \n", AWBAFConfig);
        memcpy((void *)data, (void *)&AWBAFConfig, size);
    }
	else if(size == 2)
	{
		if(offset == 17) 
		{ 
			memcpy((void *)data,(void *)&otp_af_data[0],size);
		}
		else if(offset == 19)
		{  
			memcpy((void *)data,(void *)&otp_af_data[1],size);
		}
		else 
		{
			memcpy((void *)data,(void *)&hi842_otp_data.Data[offset],size);
		}
	}
	else
	{
	    memcpy((void *)data,(void *)&hi842_otp_data.Data[offset],size);
	}

    return size;
}

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    CAM_CALDB("CAM_CAL_Ioctl\n" );

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
        CAM_CALDB("CAM_CAL_Ioctl ~ work \n" );
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALERR("ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALERR("ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALERR("ioctl allocate mem failed\n");
        return -ENOMEM;
    }


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALERR("ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
//----------------------------------------------------------------------------------------------------------------//            
    
        case CAM_CALIOC_S_WRITE:
    CAM_CALDB("CAM_CALIOC_S_WRITE \n");        
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = 0;//iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
#endif
            break;


//----------------------------------------------------------------------------------------------------------------//            

        case CAM_CALIOC_G_READ:
    	CAM_CALDB("CAM_CALIOC_G_READ \n");        
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, Hi842_OTP_DEVICE_ID, ptempbuf->u4Length);

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;

//----------------------------------------------------------------------------------------------------------------//

        case CAM_CALIOC_H_READ:
    CAM_CALDB("CAM_CALIOC_H_READ \n");        

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = selective_Q_read_region(ptempbuf->u4Offset, pu1Params, Hi842_OTP_DEVICE_ID, ptempbuf->u4Length);

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;

//----------------------------------------------------------------------------------------------------------------//

        default :
      	     CAM_CALINF("No CMD \n");
            i4RetValue = -EPERM;
        break;

//----------------------------------------------------------------------------------------------------------------//        

    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALERR("ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = Hi842otp_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1

inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALERR("Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALERR("Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR("Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR("Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);
    CAM_CALDB("RegisterCAM_CALCharDrv PASSS\n");

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{

    return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    //i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_init(void)
{
    int i4RetValue = 0;
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB("register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB("Attached!! \n");

  //  i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("failed to register 842otp driver\n");
        return -ENODEV;
    }
#ifdef CONFIG_MTK_LEGACY

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("failed to register 842otp driver, 2nd time\n");
        return -ENODEV;
    }
#endif
    CAM_CALDB("HYNIX_CAM_CAL_i2C_init PASSS\n");

    return 0;
}

static void __exit CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_init);
module_exit(CAM_CAL_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
