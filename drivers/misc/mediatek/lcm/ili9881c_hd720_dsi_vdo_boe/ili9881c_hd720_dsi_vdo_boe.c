#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
    #include <string.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#if defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#endif

#define LCM_DBG(fmt, arg...) \
    LCM_PRINT ("[ili9881c_boe] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;
#if 0
#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#else
extern int DispTEpin_Enable(void);
extern int DispTEpin_Disable(void);
#define SET_RESET_PIN(v)    \
    if(v)                                           \
        DispTEpin_Enable(); \
    else                                           \
        DispTEpin_Disable();
#endif
/*#define SET_RESET_PIN(v)    \
    mt_set_gpio_mode(GPIO_LCM_RST,GPIO_MODE_00);    \
    mt_set_gpio_dir(GPIO_LCM_RST,GPIO_DIR_OUT);     \
    if(v)                                           \
        mt_set_gpio_out(GPIO_LCM_RST,GPIO_OUT_ONE); \
    else                                            \
        mt_set_gpio_out(GPIO_LCM_RST,GPIO_OUT_ZERO);
#ifdef GPIO_LCM_PWR
#define SET_PWR_PIN(v)    \
    mt_set_gpio_mode(GPIO_LCM_PWR,GPIO_MODE_00);    \
    mt_set_gpio_dir(GPIO_LCM_PWR,GPIO_DIR_OUT);     \
    if(v)                                           \
        mt_set_gpio_out(GPIO_LCM_PWR,GPIO_OUT_ONE); \
    else                                            \
        mt_set_gpio_out(GPIO_LCM_PWR,GPIO_OUT_ZERO);	
#endif*/
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#define AUXADC_LCD_ID_CHANNEL   12
#define BOE_LCM_MIN_VOL 0
#define BOE_LCM_MAX_VOL 120
#define LCM_ID_ILI9881	0x9881 
#define GPIO_LCD_ID (GPIO60 | 0x80000000)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0

#define REGFLAG_END_OF_TABLE                                	  0xFD   // END OF REGISTERS MARKER
#define REGFLAG_DELAY                                           0xFC

struct LCM_setting_table
{
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table  lcm_deep_sleep_mode_in_setting_v2[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
};

static struct LCM_setting_table lcm_initialization_setting_v2[] = 
{
	{0xFF,3,{0x98,0x81,0x03}},
	{0x01,1,{0x00}},
	{0x02,1,{0x00}},
	{0x03,1,{0x73}},
	{0x04,1,{0x00}},
	{0x05,1,{0x00}},
	{0x06,1,{0x08}},
	{0x07,1,{0x00}},
	{0x08,1,{0x00}},
	{0x09,1,{0x1B}},
	{0x0A,1,{0x01}},
	{0x0B,1,{0x01}},
	{0x0C,1,{0x0D}},
	{0x0D,1,{0x01}},
	{0x0E,1,{0x01}},
	{0x0F,1,{0x26}},
	{0x10,1,{0x26}},
	{0x11,1,{0x00}},
	{0x12,1,{0x00}},
	{0x13,1,{0x02}},
	{0x14,1,{0x00}},
	{0x15,1 ,{0x10}},
	{0x16,1 ,{0x10}},
	{0x17,1,{0x00}},
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1A,1,{0x00}},
	{0x1B,1,{0x00}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x00}},
	{0x1E,1,{0x40}},
	{0x1F,1,{0xC0}},
	{0x20,1,{0x06}},
	{0x21,1,{0x01}},
	{0x22,1,{0x06}},
	{0x23,1,{0x01}},
	{0x24,1,{0x88}},
	{0x25,1,{0x88}},
	{0x26,1,{0x00}},
	{0x27,1,{0x00}},
	{0x28,1,{0x3B}},
	{0x29,1,{0x03}},
	{0x2A,1,{0x00}},
	{0x2B,1,{0x00}},
	{0x2C,1,{0x00}},
	{0x2D,1,{0x00}},
	{0x2E,1,{0x00}},
	{0x2F,1,{0x00}},
	{0x30,1,{0x00}},
	{0x31,1,{0x00}},
	{0x32,1,{0x00}},
	{0x33,1,{0x00}},
	{0x34,1,{0x00}},
	{0x35,1,{0x00}},
	{0x36,1,{0x00}},
	{0x37,1,{0x00}},
	{0x38,1,{0x00}},
	{0x39,1,{0x00}},
	{0x3A,1,{0x00}},
	{0x3B,1,{0x00}},
	{0x3C,1,{0x00}},
	{0x3D,1,{0x00}},
	{0x3E,1,{0x00}},
	{0x3F,1,{0x00}},
	{0x40,1,{0x00}},
	{0x41,1,{0x00}},
	{0x42,1,{0x00}},
	{0x43,1,{0x00}},
	{0x44,1,{0x00}},
	{0x50,1,{0x01}},
	{0x51,1,{0x23}},
	{0x52,1,{0x45}},
	{0x53,1,{0x67}},
	{0x54,1,{0x89}},
	{0x55,1,{0xAB}},
	{0x56,1,{0x01}},
	{0x57,1,{0x23}},
	{0x58,1,{0x45}},
	{0x59,1,{0x67}},
	{0x5A,1,{0x89}},
	{0x5B,1,{0xAB}},
	{0x5C,1,{0xCD}},
	{0x5D,1,{0xEF}},
	{0x5E,1,{0x11}},
	{0x5F,1,{0x02}},
	{0x60,1,{0x00}},
	{0x61,1,{0x0E}},
	{0x62,1,{0x0F}},
	{0x63,1,{0x0C}},
	{0x64,1,{0x0D}},
	{0x65,1,{0x02}},
	{0x66,1,{0x02}},
	{0x67,1,{0x02}},
	{0x68,1,{0x02}},
	{0x69,1,{0x02}},
	{0x6A,1,{0x02}},
	{0x6B,1,{0x02}},
	{0x6C,1,{0x02}},
	{0x6D,1,{0x02}},
	{0x6E,1,{0x05}},
	{0x6F,1,{0x05}},
	{0x70,1,{0x05}},
	{0x71,1,{0x05}},
	{0x72,1,{0x01}},
	{0x73,1,{0x06}},
	{0x74,1,{0x07}},
	{0x75,1,{0x02}},
	{0x76,1,{0x00}},
	{0x77,1,{0x0E}},
	{0x78,1,{0x0F}},
	{0x79,1,{0x0C}},
	{0x7A,1,{0x0D}},
	{0x7B,1,{0x02}},
	{0x7C,1,{0x02}},
	{0x7D,1,{0x02}},
	{0x7E,1,{0x02}},
	{0x7F,1,{0x02}},
	{0x80,1,{0x02}},
	{0x81,1,{0x02}},
	{0x82,1,{0x02}},
	{0x83,1,{0x02}},
	{0x84,1,{0x05}},
	{0x85,1,{0x05}},
	{0x86,1,{0x05}},
	{0x87,1,{0x05}},
	{0x88,1,{0x01}},
	{0x89,1,{0x06}},
	{0x8A,1,{0x07}},
	{0xFF,3,{0x98,0x81,0x04}},
	{0x00,1,{0x80}},               // 80 4lane   00 3lane
	{0x3B,1,{0x80}},
	{0x6C,1,{0x15}},
	{0x6E,1,{0x1A}},
	{0x6F,1,{0x25}},
	{0x8D,1,{0x1F}},
	{0x87,1,{0xBA}},
	{0x7A,1,{0x10}},
	{0x71,1,{0xB0}},
	{0x3A,1,{0x24}},
	{0x35,1,{0x1F}},
	{0x0B,1 ,{0x80}},
	{0x0E,1 ,{0x17}},
	{0xFF,3,{0x98,0x81,0x01}},
	{0x22,1,{0x09}},               //  scan direction
	{0x31,1,{0x00}},
	{0x40,1,{0x33}},
	{0x43,1,{0x66}},
	{0x50,1,{0x85}},
	{0x51,1,{0x82}},
	{0x60,1,{0x19}},
	{0x61,1,{0x01}},
	{0x62,1,{0x0C}},
	{0x63,1,{0x00}},
	{0xA0,1,{0x00}},
	{0xA1,1,{0x16}},
	{0xA2,1,{0x24}},
	{0xA3,1,{0x13}},
	{0xA4,1,{0x17}},
	{0xA5,1,{0x29}},
	{0xA6,1,{0x1C}},
	{0xA7,1,{0x1E}},
	{0xA8,1,{0x81}},
	{0xA9,1,{0x1E}},
	{0xAA,1,{0x29}},
	{0xAB,1,{0x6F}},
	{0xAC,1,{0x18}},
	{0xAD,1,{0x16}},
	{0xAE,1,{0x48}},
	{0xAF,1,{0x1D}},
	{0xB0,1,{0x26}},
	{0xB1,1,{0x4A}},
	{0xB2,1,{0x5D}},
	{0xB3,1,{0x39}},
	{0xC0,1,{0x00}},
	{0xC1,1,{0x16}},
	{0xC2,1,{0x24}},
	{0xC3,1,{0x13}},
	{0xC4,1,{0x17}},
	{0xC5,1,{0x29}},
	{0xC6,1,{0x1C}},
	{0xC7,1,{0x1E}},
	{0xC8,1,{0x81}},
	{0xC9,1,{0x1E}},
	{0xCA,1,{0x29}},
	{0xCB,1,{0x6F}},
	{0xCC,1,{0x18}},
	{0xCD,1,{0x16}},
	{0xCE,1,{0x48}},
	{0xCF,1,{0x1D}},
	{0xD0,1,{0x26}},
	{0xD1,1,{0x4A}},
	{0xD2,1,{0x5D}},
	{0xD3,1,{0x39}},
	{0xFF,3,{0x98,0x81,0x00}},
	{0x36,1,{0x00}},
	{0x35,1,{0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY,150,{}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY,20,{}},				
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
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
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }

}


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

		//bingqian.tang date20160413 add for GGAFMA-394
		#ifdef  CONFIG_PROJECT_P6601_WIK_FR
        	params->physical_width  = 62.10; //62.10;
        	params->physical_height = 110.40; //110.40;
		#endif
		//bingqian.tang end

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active = 6;
		params->dsi.vertical_backporch = 14;
		params->dsi.vertical_frontporch	= 10;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active = 30;
		params->dsi.horizontal_backporch = 30;
		params->dsi.horizontal_frontporch = 60;
		params->dsi.horizontal_active_pixel = FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.PLL_CLOCK=205;
		params->dsi.PLL_CLOCK=207;  //LINE </EGAFM-297> <change the mipi clock to reduce disturbing the wifi> <20160413> panzaoyan
		params->dsi.ssc_disable=1;
		
		//yixuhong 20150511 add esd check function
#ifndef BUILD_LK	
	params->dsi.esd_check_enable = 1; 
	params->dsi.customization_esd_check_enable = 1;//0:te esd check 1:read register
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	
	//params->dsi.lcm_esd_check_table[1].cmd = 0x0B;
//	params->dsi.lcm_esd_check_table[1].count = 1;
//	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
	params->dsi.lcm_esd_check_table[1].cmd = 0x0D;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
#endif /*BUILD_LK*/

}

static unsigned int lcm_compare_id(void);

static void lcm_init(void)
{
	LCM_DBG(); 
	SET_RESET_PIN(1);
	MDELAY(20);
#ifdef GPIO_LCM_PWR
	SET_PWR_PIN(0);
	MDELAY(20);
	SET_PWR_PIN(1);
	MDELAY(150);
#endif	
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(120);
	LCM_DBG("jacky debug,lcm reset end \n");
	push_table(lcm_initialization_setting_v2, sizeof(lcm_initialization_setting_v2) / sizeof(struct LCM_setting_table), 1);
	LCM_DBG("jacy debug,lcm init end \n");

	//lcm_compare_id();
}

static void lcm_suspend(void)
{	
	LCM_DBG();
    push_table(lcm_deep_sleep_mode_in_setting_v2, sizeof(lcm_deep_sleep_mode_in_setting_v2) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20); // 1ms

    SET_RESET_PIN(1);
    MDELAY(120);
}

static void lcm_resume(void)
{
	LCM_DBG();
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(120); 
	push_table(lcm_initialization_setting_v2, sizeof(lcm_initialization_setting_v2) / sizeof(struct LCM_setting_table), 1);
}      

static unsigned int lcm_compare_id(void)
{
#if 0
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
#else
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


static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 1);

	LCM_DBG("%s:esd buffer = 0x%x\n",__func__, buffer[0]);
	
	if(buffer[0]==0x9c)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
#else
	return FALSE;
#endif

}

static unsigned int lcm_esd_recover(void)
{
	LCM_DBG();
	lcm_init();
	return TRUE;
}


LCM_DRIVER ili9881c_hd720_dsi_vdo_boe_lcm_drv = 
{
	.name		= "ili9881c_hd720_dsi_vdo_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
