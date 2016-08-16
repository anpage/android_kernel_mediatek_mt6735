#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
//#include <mach/mt_pm_ldo.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#endif
//#include <cust_gpio_usage.h>
//#include <cust_i2c.h>
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif
#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#define LCM_DBG(fmt, arg...) \
    LCM_PRINT("[boe] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)


extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#define AUXADC_LCD_ID_CHANNEL   12
#define BOE_LCM_MIN_VOL  500
#define BOE_LCM_MAX_VOL      2000
#define GPIO_LCD_ID (GPIO68 | 0x80000000)

static const unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util;


#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST         (GPIO146 | 0x80000000)
#endif

#ifndef GPIO_LCD_ENN_PIN
#define GPIO_LCD_ENN_PIN         (GPIO100 | 0x80000000)
#define GPIO_LCD_ENN_PIN_M_GPIO  GPIO_MODE_00
#endif

#ifndef GPIO_LCD_ENP_PIN
#define GPIO_LCD_ENP_PIN         (GPIO101 | 0x80000000)
#define GPIO_LCD_ENP_PIN_M_GPIO  GPIO_MODE_00
#endif

//#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define SET_RESET_PIN(v)    \
    mt_set_gpio_mode(GPIO_LCM_RST,GPIO_MODE_00);    \
    mt_set_gpio_dir(GPIO_LCM_RST,GPIO_DIR_OUT);     \
    if(v)                                           \
        mt_set_gpio_out(GPIO_LCM_RST,GPIO_OUT_ONE); \
    else                                            \
        mt_set_gpio_out(GPIO_LCM_RST,GPIO_OUT_ZERO);

#define MDELAY(n)                                           (lcm_util.mdelay(n))
#define UDELAY(n)                                           (lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE                                    0
#define FRAME_WIDTH                                         (1080)
#define FRAME_HEIGHT                                        (1920)
#define GPIO_65132_ENP GPIO_LCD_ENP_PIN
#define GPIO_65132_ENN GPIO_LCD_ENN_PIN


#define REGFLAG_DELAY                                           0xFC
#define REGFLAG_UDELAY                                          0xFB

#define REGFLAG_END_OF_TABLE                                	  0xFD   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW                                       0xFE
#define REGFLAG_RESET_HIGH                                      0xFF

#ifndef BUILD_LK
static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------


struct LCM_setting_table
{
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] =
{
	
	{0x00, 1,{0x00}},
	{0x99, 2,{0x95,0x27}},	
	
	{0x28, 1, {0x00}}, 
	{REGFLAG_DELAY, 50, {}}, 
	
	{0x10, 1, {0x00}}, 
	{REGFLAG_DELAY, 120, {}}, 
	
	{0x00, 1, {0x00}},
	{0xF7, 4, {0x5A,0xA5,0x19,0x01}},//deep sleep in
	
	{0x00, 1,{0x00}},
	{0x99, 2,{0x11,0x11}},	
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

//AUO_ini_1901_5P2(H520DAN01.1)_typeA_nmos_20150714.txt
static struct LCM_setting_table lcm_initialization_setting[] =
{
	{0x00, 1,{0x00}},
	{0xFF, 3,{0x19,0x01,0x01}},
	{0x00, 1,{0x80}},
	{0xFF, 2,{0x19,0x01}},
	
	{0x00, 1,{0x00}},
	{0x1c, 1,{0x33}},
	{0x00, 1,{0xa0}},
	{0xc1, 1,{0xe8}},                               
	
	{0x00, 1,{0xa7}},
	{0xc1, 1,{0x00}},                                
	{0x00, 1,{0x90}},
	{0xc0, 6,{0x00,0x2F,0x00,0x00,0x00,0x01}},                                 
	{0x00, 1,{0xC0}},
	{0xc0, 6,{0x00,0x2F,0x00,0x00,0x00,0x01}},                                 
	
	{0x00, 1,{0x9a}},
	{0xc0, 1,{0x1e}},                                 
	{0x00, 1,{0xac}},
	{0xc0, 1,{0x06}},                                 
	
	{0x00, 1,{0xdc}},
	{0xc0, 1,{0x06}},
	{0x00, 1,{0x81}},    
	{0xa5, 1,{0x04}}, 
    {0x00, 1,{0x84}},    
	{0xc4, 1,{0x20}}, 
	
	{0x00, 1,{0xa5}},    
	{0xb3, 1,{0x1d}},
	{0x00, 1,{0x92}},
	{0xe9, 1,{0x00}},
	
	{0x00, 1,{0x90}},
	{0xF3, 1,{0x01}},
	{0x00, 1,{0xf7}},    
	{0xC3, 4,{0x04,0x18,0x04,0x04}}, 
	
	{0x00, 1,{0xB4}},
	{0xC0, 1,{0x80}},
    {0x00, 1,{0x93}},
	{0xC5, 1,{0x19}},
	
	{0x00, 1,{0x95}},
	{0xC5, 1,{0x2D}},
    {0x00, 1,{0x97}},
	{0xC5, 1,{0x14}},
	
	{0x00, 1,{0x99}},
	{0xC5, 1,{0x29}},
    {0x00, 1,{0x00}},
	{0xD8, 2,{0x23,0x23}}, 
	
	{0x00, 1,{0x00}},
	{0xE1,24,{0x00,0x08,0x0E,0x1A,0x23,0x2B,0x38,0x49,0x54,0x66,0x70,0x76,0x86,0x81,0x7C,0x6D,0x58,0x45,0x37,0x2F,0x25,0x19,0x17,0x03}},
	{0x00, 1,{0x00}},
	{0xE2, 24,{0x00,0x08,0x0E,0x1A,0x23,0x2B,0x38,0x49,0x54,0x66,0x70,0x76,0x86,0x81,0x7C,0x6D,0x58,0x45,0x37,0x2F,0x25,0x19,0x17,0x03}},
	{0x00, 1,{0x00}},
	{0xE3, 24,{0x00,0x08,0x0E,0x1A,0x23,0x2B,0x38,0x49,0x54,0x66,0x70,0x76,0x86,0x81,0x7C,0x6D,0x58,0x45,0x37,0x2F,0x25,0x19,0x17,0x03}},                               
	{0x00, 1,{0x0}},
	{0xE4, 24,{0x00,0x08,0x0E,0x1A,0x23,0x2B,0x38,0x49,0x54,0x66,0x70,0x76,0x86,0x81,0x7C,0x6D,0x58,0x45,0x37,0x2F,0x25,0x19,0x17,0x03}},                                
	{0x00, 1,{0x00}},
	{0xE5, 24,{0x00,0x08,0x0E,0x1A,0x23,0x2B,0x38,0x49,0x54,0x66,0x70,0x76,0x86,0x81,0x7C,0x6D,0x58,0x45,0x37,0x2F,0x25,0x19,0x17,0x03}},                                
	{0x00, 1,{0x00}},
	{0xE6, 24,{0x00,0x08,0x0E,0x1A,0x23,0x2B,0x38,0x49,0x54,0x66,0x70,0x76,0x86,0x81,0x7C,0x6D,0x58,0x45,0x37,0x2F,0x25,0x19,0x17,0x03}},                                
	
	{0x00, 1,{0x81}},
	{0xA5, 1,{0x07}},
	{0x00, 1,{0x9D}},
	{0xC5, 1,{0x77}},
	
	{0x00, 1,{0xB3}},
	{0xC0, 1,{0xCC}},
	{0x00, 1,{0xBC}},
	{0xC0, 1,{0x00}},
	
	{0x00, 1,{0x80}},
	{0xC0,14,{0x00,0x87,0x00,0x0A,0x0A,0x00,0x87,0x0A,0x0A,0x00,0x87,0x00,0x0A,0x0A}},
	{0x00, 1,{0xF0}},
	{0xC3,6,{0x22,0x02,0x00,0x00,0x00,0x0C}},
	
	{0x00, 1,{0xA0}},
	{0xC0,7,{0x00,0x00,0x00,0x00,0x03,0x22,0x03}},
	{0x00, 1,{0xD0}},
	{0xC0, 7,{0x00,0x00,0x00,0x00,0x03,0x22,0x03}},
	
	{0x00, 1,{0x90}},
	{0xC2, 8,{0x83,0x01,0x00,0x00,0x82,0x01,0x00,0x00}},
	{0x00, 1,{0x80}},
	{0xC3, 12,{0x82,0x02,0x03,0x00,0x03,0x84,0x81,0x03,0x03,0x00,0x03,0x84}},
	
	{0x00, 1,{0x90}},
	{0xC3, 12,{0x00,0x01,0x03,0x00,0x03,0x84,0x01,0x02,0x03,0x00,0x03,0x84}},
	{0x00, 1,{0x80}},
	{0xCC,15,{0x09,0x0A,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x28,0x28,0x28,0x28,0x28}},
	
	{0x00, 1,{0x90}},
	{0xCC,15,{0x0A,0x09,0x14,0x13,0x12,0x11,0x15,0x16,0x17,0x18,0x28,0x28,0x28,0x28,0x28}},
	{0x00, 1,{0xA0}},
	{0xCC,15,{0x1D,0x1E,0x1F,0x19,0x1A,0x1B,0x1C,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27}},
	
	{0x00, 1,{0xB0}},
	{0xCC,8,{0x01,0x02,0x03,0x05,0x06,0x07,0x04,0x08}},
	{0x00, 1,{0xC0}},
	{0xCC,12,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x77}},
	
	{0x00, 1,{0xD0}},
	{0xCC,12,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x77}},
	{0x00, 1,{0x80}},
	{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
	{0x00, 1,{0x90}},
	{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0x00, 1,{0xA0}},
	{0xCB,15,{0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
	{0x00, 1,{0xB0}},
	{0xCB,15,{0x00,0x01,0xFD,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x00, 1,{0xC0}},
	{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x77,0x77}},
	
	{0x00, 1,{0xD0}},
	{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x77,0x77}},
	{0x00, 1,{0xE0}},
	{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x77,0x77}},
	
	{0x00, 1,{0xF0}},
	{0xCB,8,{0x01,0x01,0x01,0x00,0x00,0x00,0x77,0x77}},
	{0x00, 1,{0x80}},
	{0xCD,15,{0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x02,0x12,0x11,0x3F,0x04,0x3F}},
	
	{0x00, 1,{0x90}},
	{0xCD,11,{0x06,0x3F,0x3F,0x26,0x26,0x26,0x21,0x20,0x1F,0x26,0x26}},
	{0x00, 1,{0xA0}},
	{0xCD,15,{0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x01,0x12,0x11,0x3F,0x03,0x3F}},
	
	{0x00, 1,{0xB0}},
	{0xCD,11,{0x05,0x3F,0x3F,0x26,0x26,0x26,0x21,0x20,0x1F,0x26,0x26}},
	{0x00, 1,{0x9B}},
	{0xC5,2,{0x55,0x55}},
	
	{0x00, 1,{0x80}},
	{0xC4,1,{0x15}},
	{0x00, 1,{0x0}},
	{0xFF,3,{0xFF,0xFF,0xFF}},

	{0x00, 1,{0x00}},//te on
	{0x35, 1,{0x00}},//te on
	
	{0x11, 1,{0x00}},  
  	{REGFLAG_DELAY,120,{}},

	{0x29, 1,{0x00}},
	{REGFLAG_DELAY,20,{}},
	
	{0x00, 1,{0x00}},
	{0x99, 2,{0x11,0x11}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};               

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;

            case REGFLAG_UDELAY :
                UDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width  = 64.8;
	params->physical_height = 115.2;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = BURST_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.ssc_disable = 1;
	//params->dsi.ssc_range = 8;

	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 8;
	params->dsi.vertical_frontporch 				= 14;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 20;
	params->dsi.horizontal_backporch				= 32;
	params->dsi.horizontal_frontporch				= 31;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

	//yixuhong 20150511 add esd check function
#ifndef BUILD_LK	
	params->dsi.esd_check_enable = 1; 
	params->dsi.customization_esd_check_enable = 0;//0:te esd check 1:read register
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	
	params->dsi.lcm_esd_check_table[1].cmd = 0xAC;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x0;
#endif /*BUILD_LK*/	

	params->dsi.PLL_CLOCK = 414;

}

static void lcm_init(void)
{
    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
	MDELAY(10);

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);

    SET_RESET_PIN(1);
    MDELAY(120);

    // when phone initial , config output high, enable backlight drv chip
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	
   push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
   
	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
    MDELAY(10);	   
    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);
    MDELAY(10);
    SET_RESET_PIN(0);
}

static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	s32 lcd_hw_id = -1;
	
	lcd_hw_id = mt_get_gpio_in(GPIO_LCD_ID);
	LCM_DBG("lcm_compare_id lcd_hw_id=%d \n",lcd_hw_id);
	if (1==lcd_hw_id)
	{
		return 1;
	}
	else
	{
		return 0;
	}
#if 0
	int data[4] = {0, 0, 0, 0};
	int tmp = 0 ,rc = 0, iVoltage = 0;
	rc = IMM_GetOneChannelValue(AUXADC_LCD_ID_CHANNEL, data, &tmp);
	if(rc < 0)
	{
		LCM_DBG("read LCD_ID vol error\n");
		return 0;
	}
	else
	{
		iVoltage = (data[0]*1000) + (data[1]*10) + (data[2]);
		LCM_DBG("data[0]=%d,data[1]=%d,data[2]=%d,data[3]=%d,iVoltage=%d\n",
				data[0], data[1], data[2], data[3], iVoltage);
		if((BOE_LCM_MIN_VOL <= iVoltage) && (iVoltage < BOE_LCM_MAX_VOL))
			return 1;
		else
			return 0;
	}
	return 0;
#endif	
}

LCM_DRIVER otm1901a_fhd_dsi_vdo_boe_tps65132_lcm_drv=
{
    .name               = "otm1901a_fhd_dsi_vdo_boe_tps65132",
    .set_util_funcs     = lcm_set_util_funcs,
    .get_params         = lcm_get_params,
    .init               = lcm_init,
    .suspend            = lcm_suspend,
    .resume             = lcm_resume,
    .compare_id        	= lcm_compare_id,
};
/* END PN:DTS2013053103858 , Added by d00238048, 2013.05.31*/
