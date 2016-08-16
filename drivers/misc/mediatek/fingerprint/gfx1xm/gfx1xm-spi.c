#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/input.h>   
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/usb.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/completion.h>
#include <mach/mt_clkmgr.h>
#include <gfx1xm-spi.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#include "../fp_drv/fp_drv.h"

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static struct task_struct *gfx1xm_irq_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int irq_flag = 0;
static int suspend_flag = 0;
int chip_version_config=0;
static struct gfx1xm_dev gfx1xm;
//guomingyi.
static int ftm_gfx_irq_state = 0;
static int ftm_gfx_irq_flag = 0;
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static ssize_t gfx1xm_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gfx1xm_config_write_proc(struct file *, const char __user *, size_t, loff_t *);
static struct proc_dir_entry *gfx1xm_config_proc = NULL;
static const struct file_operations config_proc_ops = {
    .owner = THIS_MODULE,
    .read = gfx1xm_config_read_proc,
    .write = gfx1xm_config_write_proc,	
};

static int suspend_test_flag = 0;	


/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static struct class *gfx1xm_spi_class;
/*************************data stream***********************
 *	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
 *     1B      |   1B    |  1B     |    2048B     |  2B      |
 ************************************************************/
static unsigned bufsiz = 8 * (2048+5);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

#if FW_UPDATE	
static unsigned char GFX1XM318_FW[]=
{
#include "gf318_fw.i"
};
static unsigned char GFX1XM316_FW[]=
{
#include "gf316_fw.i"
};

#define FW_LENGTH (42*1024)
#endif

struct gfx1xm_platform_data {
	int irq_gpio;
	int reset_gpio;
	int external_supply_mv;
};

void print_16hex(u8 *config, u8 len)
{
    u8 i,j = 0;  
    gfx1xm_debug(DEFAULT_DEBUG,"dump hex ");
    for(i = 0 ; i< len ; i++) {
		gfx1xm_debug(DEFAULT_DEBUG,"0x%x " , config[i]);
		if(j++ == 15) {
		    j = 0;
		}
    } 
}
typedef enum {
	SPEED_500KHZ=0,
	SPEED_1MHZ,
	SPEED_2MHZ,
	SPEED_3MHZ,
	SPEED_4MHZ,
	SPEED_6MHZ,
	SPEED_8MHZ,
	SPEED_KEEP,
	SPEED_UNSUPPORTED
}SPI_SPEED;

static struct mt_chip_conf spi_conf_mt65xx = {
	.setuptime = 15,
	.holdtime = 15,
	.high_time = 21, 
	.low_time = 21,	
	.cs_idletime = 20,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = FIFO_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

/****************************************************
**Setup SPI speed and transfer mode
**SPEEND = SPI_MODULE_CLOCK/(high_time+low_time)  KHZ
**eg:SPI_MODULE_CLOCK = 104000khz
**   high_time = 10   low_time = 10
**SPEEND=104000/(10+10)=5200KHZ=5.2M
*****************************************************/
static void gfx1xm_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{
	struct mt_chip_conf *mcc = &spi_conf_mt65xx;
	if(flag == 0) {
		mcc->com_mod = FIFO_TRANSFER;
	} else {
		mcc->com_mod = DMA_TRANSFER;
	}
	switch(speed)
	{
		case SPEED_500KHZ:
			mcc->high_time = 120;
			mcc->low_time = 120;
			break;
		case SPEED_1MHZ:
			mcc->high_time = 60;
			mcc->low_time = 60;
			break;
		case SPEED_2MHZ:
			mcc->high_time = 30;
			mcc->low_time = 30;
			break;
		case SPEED_3MHZ:
			mcc->high_time = 20;
			mcc->low_time = 20;
			break;
		case SPEED_4MHZ:
			mcc->high_time = 15;
			mcc->low_time = 15;
			break;

		case SPEED_6MHZ:
			mcc->high_time = 10;
			mcc->low_time = 10;
			break;
		case SPEED_8MHZ:
		    mcc->high_time = 8;
			mcc->low_time = 8;
			break;  
		case SPEED_KEEP:
		case SPEED_UNSUPPORTED:
			break;
	}
	if(spi_setup(spi) < 0){
		gfx1xm_error("gfx1xm:Failed to set spi.");
	}
}

/* -------------------------------------------------------------------- */
static DEFINE_MUTEX(gfx1xm_set_gpio_mutex);
static void gfx1xm_gpio_as_int(void)
{
	mutex_lock(&gfx1xm_set_gpio_mutex);
	printk("[gfx1xm]gfx1xm_gpio_as_int\n");
	pinctrl_select_state(gfx1xm.pinctrl1, gfx1xm.eint_as_int);
	mutex_unlock(&gfx1xm_set_gpio_mutex);
}

static void gfx1xm_reset_output(int level)
{
	mutex_lock(&gfx1xm_set_gpio_mutex);
	printk("[gfx1xm]gfx1xm_reset_output level = %d\n", level);

	if (level)
		pinctrl_select_state(gfx1xm.pinctrl1, gfx1xm.fp_rst_high);
	else
		pinctrl_select_state(gfx1xm.pinctrl1, gfx1xm.fp_rst_low);
	mutex_unlock(&gfx1xm_set_gpio_mutex);
}

static void gfx1xm_irq_pulldown(int enable)
{
	mutex_lock(&gfx1xm_set_gpio_mutex);
	printk("[gfx1xm]gfx1xm_irq_pulldown enable = %d\n", enable);

	if (enable)
		pinctrl_select_state(gfx1xm.pinctrl1, gfx1xm.eint_in_low);
	else
		pinctrl_select_state(gfx1xm.pinctrl1, gfx1xm.eint_in_float);
	mutex_unlock(&gfx1xm_set_gpio_mutex);
}

static void gfx1xm_miso_pullup(int enable)
{
	mutex_lock(&gfx1xm_set_gpio_mutex);
	printk("[gfx1xm]gfx1xm_irq_pulldown enable = %d\n", enable);

	if (enable)
		pinctrl_select_state(gfx1xm.pinctrl1, gfx1xm.miso_pull_up);
	else
		pinctrl_select_state(gfx1xm.pinctrl1, gfx1xm.miso_pull_disable);
	mutex_unlock(&gfx1xm_set_gpio_mutex);
}

static void gfx1xm_hw_reset(struct gfx1xm_dev *gfx1xm_dev, int ms)
{	
	gfx1xm_debug(DEFAULT_DEBUG, "gfx1xm_hw_reset\n");
	gfx1xm_miso_pullup(1);
       gfx1xm_reset_output(0);
	msleep(5);  //delay for power to reset  typical:10ms max:50ms
	gfx1xm_reset_output(1);
	
	if (ms)
	    msleep(ms);
       gfx1xm_miso_pullup(0);
}
/* -------------------------------------------------------------------- */
/*********************************************************
**Power control
**function: hwPowerOn   hwPowerDown
*********************************************************/
int gfx1xm_power_on(struct gfx1xm_dev *gfx1xm_dev, bool onoff)
{
	int ret;

	printk("Device Tree get regulator!");
	gfx1xm_debug(DEFAULT_DEBUG,"%s onoff = %d", __func__, onoff);
	if(onoff) {
		if (0 == gfx1xm_dev->poweron){
			/*INT GPIO Pull-down before power on*/
			gfx1xm_irq_pulldown(1);
			/*Reset GPIO Output-low before poweron*/
			gfx1xm_reset_output(0);
			msleep(5);

			/*power on*/
               	
            		ret = regulator_enable(gfx1xm_dev->reg);	/*enable regulator*/
			if (ret)
				printk("regulator_enable() failed!\n");
			gfx1xm_dev->poweron = 1;
			msleep(20);

			/*INT GPIO set floating after poweron and controlled by GFX1XM*/
			gfx1xm_irq_pulldown(0);
			msleep(1);
			
			/*Reset GPIO Output-high, GFX1XM works*/
                     gfx1xm_reset_output(1);
			msleep(60);
		}
	} else {
		if (1 == gfx1xm_dev->poweron){
			gfx1xm_irq_pulldown(1);
			gfx1xm_reset_output(0);
			msleep(10);
			ret = regulator_disable(gfx1xm_dev->reg);	/*disable regulator*/
			if (ret)
				printk("regulator_disable() failed!\n");
			gfx1xm_dev->poweron = 0;
			msleep(50);
		}
	}
	return 0;
}

static ssize_t gfx1xm_config_read_proc(struct file *filp, char __user *page, size_t size, loff_t *ppos)
{
    struct gfx1xm_dev *gfx1xm_dev = &gfx1xm;
	char *ptr =page;
	int i = 0;
    unsigned char version[16];

	gfx1xm_debug(DEFAULT_DEBUG,"%s", __func__);

	if (*ppos)
		return 0;
#if CFG_UPDATE
	ptr += sprintf(ptr, "\n====  GFX1XM config init value ====\n");

	for (i=0; i<GFX1XM_CFG_LEN; i++) {
		if (i%10 == 0)
			ptr += sprintf(ptr, "\n");
		
		ptr += sprintf(ptr, "0x%02X,", gfx1xm_dev->config[i+GFX1XM_WDATA_OFFSET]);

	}
	ptr += sprintf(ptr, "\n");
#endif
	ptr += sprintf(ptr, "\n====  GFX1XM config real value ====\n");
	mutex_lock(&gfx1xm_dev->buf_lock);
	gfx1xm_spi_read_bytes(gfx1xm_dev, GFX1XM_CFG_ADDR, GFX1XM_CFG_LEN, gfx1xm_dev->buffer);	
	for (i=0; i<GFX1XM_CFG_LEN; i++) {
		if (i%10 == 0)
			ptr += sprintf(ptr, "\n");
		
		ptr += sprintf(ptr, "0x%02X,", gfx1xm_dev->buffer[i+GFX1XM_RDATA_OFFSET]);
	}
	ptr += sprintf(ptr, "\n");
	
    gfx1xm_spi_read_bytes(gfx1xm_dev,0x8000,16,gfx1xm_dev->buffer);
    memcpy(version, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, 16);
	ptr += sprintf(ptr, "Chip version: %c%c%c%c%c%c%c%x.%02x.%02x,", version[0],version[1],version[2],version[3],version[4],version[5],version[6],version[7],version[8],version[9]);

	ptr += sprintf(ptr, "\n");
	mutex_unlock(&gfx1xm_dev->buf_lock);
	*ppos += ptr - page;
	return (ptr -page);
}

static ssize_t gfx1xm_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    struct gfx1xm_dev *gfx1xm_dev = &gfx1xm;

	gfx1xm_debug(DEFAULT_DEBUG,"%s", __func__);
	
	if (count > GFX1XM_CFG_LEN){
//		gfx1xm_error("size not match [%d:%d]", GFX1XM_CFG_LEN, count);
		return -EFAULT;
	}

	mutex_lock(&gfx1xm_dev->buf_lock);
	if (copy_from_user(&gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET], buffer, count)){
		gfx1xm_error("copy from user fail");
		mutex_unlock(&gfx1xm_dev->buf_lock);
		return -EFAULT;		
	}
	gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_CFG_ADDR, count, gfx1xm_dev->buffer);
	mutex_unlock(&gfx1xm_dev->buf_lock);
	return count;
}


/* -------------------------------------------------------------------- */
/* devfs                                                                */
/* -------------------------------------------------------------------- */
static ssize_t gfx1xm_debug_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	return (sprintf(buf, "%d\n", g_debug_level));
}
static ssize_t gfx1xm_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int g_debug = 0;
	sscanf(buf, "%d", &g_debug);
	gfx1xm_debug_level(g_debug);
	//return strnlen(buf, count); 
	return count;
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gfx1xm_debug_show, gfx1xm_debug_store);

static struct attribute *gfx1xm_debug_attrs[] = {
    &dev_attr_debug.attr,
    NULL
};

static const struct attribute_group gfx1xm_debug_attr_group = {
    .attrs = gfx1xm_debug_attrs,
    .name = "gfx1xm_debug"
};

/********************************************
**SPI setup
*********************************************/
void gfx1xm_spi_setup(struct gfx1xm_dev *gfx1xm_dev, int max_speed_hz)
{
    gfx1xm_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
    gfx1xm_dev->spi->max_speed_hz = max_speed_hz; 
    gfx1xm_dev->spi->bits_per_word = 8;
    gfx1xm_dev->spi->controller_data  = (void*)&spi_conf_mt65xx;
    spi_setup(gfx1xm_dev->spi);
}

/**********************************************************
 *Message format:
 *	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/
int gfx1xm_spi_write_bytes(struct gfx1xm_dev *gfx1xm_dev,
				u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32  package_num = (data_len + 2*GFX1XM_WDATA_OFFSET)>>MTK_SPI_ALIGN_MASK_NUM;
	u32  reminder = (data_len + 2*GFX1XM_WDATA_OFFSET) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8   twice = 0;
	int ret = 0;

	/*set spi mode.*/
	if((data_len + GFX1XM_WDATA_OFFSET) > 32) {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_KEEP, 1); //FIFO
	} else {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		/*copy the reminder data to temporarity buffer.*/
		reminder_buf = kzalloc(reminder + GFX1XM_WDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			gfx1xm_error("gfx1xm:No memory for exter data.");
			return -ENOMEM;
		}
		memcpy(reminder_buf + GFX1XM_WDATA_OFFSET, tx_buf + 2*GFX1XM_WDATA_OFFSET+data_len - reminder, reminder);
        gfx1xm_debug(SPI_DEBUG,"gfx1xm:w-reminder:0x%x-0x%x,0x%x", reminder_buf[GFX1XM_WDATA_OFFSET],reminder_buf[GFX1XM_WDATA_OFFSET+1],
                reminder_buf[GFX1XM_WDATA_OFFSET + 2]);
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	}
	if( xfer == NULL){
		gfx1xm_error("gfx1xm:No memory for command.");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}

	//gfx1xm_debug(SPI_DEBUG,"gfx1xm:write twice = %d. data_len = %d, package_num = %d, reminder = %d\n", (int)twice, (int)data_len, (int)package_num, (int)reminder);
	/*if the length is not align with 1024. Need 2 transfer at least.*/
	spi_message_init(&msg);
	tx_buf[0] = GFX1XM_W;
	tx_buf[1] = (u8)((addr >> 8)&0xFF);
	tx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tx_buf;
	//xfer[0].delay_usecs = 5;
	if(twice == 1) {
		xfer[0].len = package_num << MTK_SPI_ALIGN_MASK_NUM;
		spi_message_add_tail(&xfer[0], &msg);
		addr += (data_len - reminder + GFX1XM_WDATA_OFFSET);
		reminder_buf[0] = GFX1XM_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[1].tx_buf = reminder_buf;
		xfer[1].len = reminder + 2*GFX1XM_WDATA_OFFSET;
		//xfer[1].delay_usecs = 5;
		spi_message_add_tail(&xfer[1], &msg);
	} else {
		xfer[0].len = data_len + GFX1XM_WDATA_OFFSET;
		spi_message_add_tail(&xfer[0], &msg);
	}
	ret = spi_sync(gfx1xm_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1)
			ret = msg.actual_length - 2*GFX1XM_WDATA_OFFSET;
		else
			ret = msg.actual_length - GFX1XM_WDATA_OFFSET;
	} else 	{
		gfx1xm_debug(SPI_DEBUG,"gfx1xm:write async failed. ret = %d", ret);
	}

	if(xfer != NULL) {
		kfree(xfer);
		xfer = NULL;
	}
	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}
	
	return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
int gfx1xm_spi_read_bytes(struct gfx1xm_dev *gfx1xm_dev,
				u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32  package_num = (data_len + 1 + 1)>>MTK_SPI_ALIGN_MASK_NUM;
	u32  reminder = (data_len + 1 + 1) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8   twice = 0;
	int ret = 0;
	
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		reminder_buf = kzalloc(reminder + GFX1XM_RDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			gfx1xm_error("No memory for exter data.");
			return -ENOMEM;
		}
		xfer = kzalloc(sizeof(*xfer)*4, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	}
	if( xfer == NULL){
		gfx1xm_error("No memory for command.");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}
	/*set spi mode.*/
	if((data_len + GFX1XM_RDATA_OFFSET) > 32) {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	spi_message_init(&msg);
    /*send GFX1XM command to device.*/
	rx_buf[0] = GFX1XM_W;
	rx_buf[1] = (u8)((addr >> 8)&0xFF);
	rx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = rx_buf;
	xfer[0].len = 3;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gfx1xm_dev->spi, &msg);
	spi_message_init(&msg);

	/*if wanted to read data from GFX1XM. 
	 *Should write Read command to device
	 *before read any data from device.
	 */
	//memset(rx_buf, 0xff, data_len);
	rx_buf[4] = GFX1XM_R;
	xfer[1].tx_buf = &rx_buf[4];
	xfer[1].rx_buf = &rx_buf[4];
	if(twice == 1)
		xfer[1].len = (package_num << MTK_SPI_ALIGN_MASK_NUM);
	else
		xfer[1].len = data_len + 1;
	spi_message_add_tail(&xfer[1], &msg);
	if(twice == 1) {
		addr += data_len - reminder + 1;
		reminder_buf[0] = GFX1XM_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[2].tx_buf = reminder_buf;
		xfer[2].len = 3;
		spi_message_add_tail(&xfer[2], &msg);
		spi_sync(gfx1xm_dev->spi, &msg);
		spi_message_init(&msg);
		reminder_buf[4] = GFX1XM_R;
		xfer[3].tx_buf = &reminder_buf[4];
		xfer[3].rx_buf = &reminder_buf[4];
		xfer[3].len = reminder + 1 + 1;
		spi_message_add_tail(&xfer[3], &msg);
	}
	ret = spi_sync(gfx1xm_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1) {
            gfx1xm_debug(SPI_DEBUG,"gfx1xm:reminder:0x%x:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x", reminder_buf[0], reminder_buf[1], 
                    reminder_buf[2], reminder_buf[3],reminder_buf[4],reminder_buf[5],reminder_buf[6],reminder_buf[7]);
			memcpy(rx_buf + GFX1XM_RDATA_OFFSET + data_len - reminder + 1, reminder_buf + GFX1XM_RDATA_OFFSET, reminder);
			ret = data_len;//msg.actual_length - 1; //8 
		} else {
			ret = data_len;//msg.actual_length - 1; //4
		}
	}else {
        gfx1xm_error("gfx1xm: read failed. ret = %d", ret);
  }

	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;
	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}	
	//gfx1xm_debug(SPI_DEBUG,"gfx1xm:read twice = %d, data_len = %d, package_num = %d, reminder = %d\n",(int)twice, (int)data_len, (int)package_num, (int)reminder);
	//gfx1xm_debug(SPI_DEBUG,"gfx1xm:data_len = %d, msg.actual_length = %d, ret = %d\n", (int)data_len, (int)msg.actual_length, ret);
	return ret;
}

static int gfx1xm_spi_read_byte(struct gfx1xm_dev *gfx1xm_dev, u16 addr, u8 *value)
{
    int status = 0;
    mutex_lock(&gfx1xm_dev->buf_lock);

    status = gfx1xm_spi_read_bytes(gfx1xm_dev, addr, 1, gfx1xm_dev->buffer);
    *value = gfx1xm_dev->buffer[GFX1XM_RDATA_OFFSET];
    mutex_unlock(&gfx1xm_dev->buf_lock);
    return status;
}
static int gfx1xm_spi_write_byte(struct gfx1xm_dev *gfx1xm_dev, u16 addr, u8 value)
{
    int status = 0;
    mutex_lock(&gfx1xm_dev->buf_lock);
    gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET] = value;
    status = gfx1xm_spi_write_bytes(gfx1xm_dev, addr, 1, gfx1xm_dev->buffer);
    mutex_unlock(&gfx1xm_dev->buf_lock);
    return status;
}

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t gfx1xm_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    ssize_t			status = 0;
    //long int t1, t2;
    FUNC_ENTRY();
    if ((count > bufsiz)||(count == 0)) {
		gfx1xm_error("Max size for write buffer is %d. wanted length is %d", (int)bufsiz, (int)count);
		return -EMSGSIZE;
    }
    mutex_lock(&gfx1xm_dev->fb_lock);
    mutex_lock(&gfx1xm_dev->buf_lock);
	  gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_4MHZ, 0);
    status = gfx1xm_spi_read_bytes(gfx1xm_dev, GFX1XM_BUFFER_DATA, count, gfx1xm_dev->buffer);
    gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_1MHZ, 0);
    if(status > 0) {
		unsigned long missing = 0;
		missing = copy_to_user(buf, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, status);
		if(missing == status)
	 	   status = -EFAULT;
    } else {
		gfx1xm_error("Failed to read data from SPI device.");
		status = -EFAULT;
    }
    mutex_unlock(&gfx1xm_dev->buf_lock);
    mutex_unlock(&gfx1xm_dev->fb_lock);
    FUNC_EXIT();
    return status;
}

/* Write-only message with current device setup */
static ssize_t gfx1xm_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    ssize_t			status = 0;
	
    FUNC_ENTRY();
    if(count > bufsiz) {
		gfx1xm_error("Max size for write buffer is %d", bufsiz);
		return -EMSGSIZE;
    } 
    mutex_lock(&gfx1xm_dev->buf_lock);
    status = copy_from_user(gfx1xm_dev->buffer + GFX1XM_WDATA_OFFSET, buf, count);
    if(status == 0) {
		status = gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_BUFFER_DATA, count, gfx1xm_dev->buffer);
    } else {
		gfx1xm_error("Failed to xfer data through SPI bus.");
		status = -EFAULT;
    }
    mutex_unlock(&gfx1xm_dev->buf_lock);
    FUNC_EXIT();
    return status;
}

static long gfx1xm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gfx1xm_dev *gfx1xm_dev = (struct gfx1xm_dev *)filp->private_data;
    struct gfx1xm_ioc_transfer *ioc = NULL;
    int			err = 0;
    u32			tmp = 0;
    int 		retval = 0;
    u8 *temp_buf;
    if (_IOC_TYPE(cmd) != GFX1XM_IOC_MAGIC)
            return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
			(void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));
    if (err)
		return -EFAULT;
	
    FUNC_ENTRY();
    switch(cmd) {
	case GFX1XM_IOC_CMD:
	    ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
	    /*copy command data from user to kernel.*/
	    if(copy_from_user(ioc, (struct gfx1xm_ioc_transfer*)arg, sizeof(*ioc))){
			gfx1xm_error("Failed to copy command from user to kernel.");
			retval = -EFAULT;
			break;
	    }

	    if((ioc->len > bufsiz)||(ioc->len == 0)) {
			gfx1xm_error("The request length[%d] is longer than supported maximum buffer length[%d].", 
				ioc->len, bufsiz);
			retval = -EMSGSIZE;
			break;
	    }
		mutex_lock(&gfx1xm_dev->fb_lock);
	    mutex_lock(&gfx1xm_dev->buf_lock);
#if PROCESSOR_64_BIT
	    if(ioc->cmd == GFX1XM_R) {
			/*if want to read data from hardware.*/
			//gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm_ioctl Read data from 0x%x, len = 0x%x buf = 0x%p\n", (int)ioc->addr, (int)ioc->len, (void __user*)((unsigned long)ioc->buf));
			gfx1xm_spi_read_bytes(gfx1xm_dev, ioc->addr, ioc->len, gfx1xm_dev->buffer);
			if(copy_to_user((void __user*)((unsigned long)ioc->buf), gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, ioc->len)) {
			    gfx1xm_error("Failed to copy data from kernel to user.");
			    retval = -EFAULT;
			    mutex_unlock(&gfx1xm_dev->buf_lock);
				mutex_unlock(&gfx1xm_dev->fb_lock);
			    break;
			}
	    } else if (ioc->cmd == GFX1XM_W) {
			/*if want to read data from hardware.*/
			gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm_ioctl Write data to 0x%x, len = 0x%x", ioc->addr, ioc->len);
			if(ioc->addr == GFX1XM_MODE_STATUS) {
				temp_buf=(void __user*)(unsigned long)ioc->buf;
				gfx1xm_debug(DEFAULT_DEBUG," temp_buf=%x,%x,%x,%x",temp_buf[0],temp_buf[1],temp_buf[2],temp_buf[3]);
			    gfx1xm_dev->mode = temp_buf[0];
			    gfx1xm_debug(DEFAULT_DEBUG,"set mode 0x%x \n", gfx1xm_dev->mode);
			}
			if (ioc->addr == 0x8045){
				u8 buferr[5]={0x00,0x00,0x00,0x00,0x00};
				gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_BUFFER_STATUS, 1, buferr);
				mdelay(200);
			}
			if(copy_from_user(gfx1xm_dev->buffer + GFX1XM_WDATA_OFFSET, (void __user*)((unsigned long) ioc->buf), ioc->len)){
			    gfx1xm_error("Failed to copy data from user to kernel.");
			    retval = -EFAULT;
			    mutex_unlock(&gfx1xm_dev->buf_lock);
				mutex_unlock(&gfx1xm_dev->fb_lock);
			    break;
			}
			gfx1xm_spi_write_bytes(gfx1xm_dev, ioc->addr, ioc->len, gfx1xm_dev->buffer);
	    }
#else
		if(ioc->cmd == GFX1XM_R) {
			/*if want to read data from hardware.*/
			//gfx1xm_debug(DEFAULT_DEBUG,"Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, ioc->buf);
			gfx1xm_spi_read_bytes(gfx1xm_dev, ioc->addr, ioc->len, gfx1xm_dev->buffer);
			if(copy_to_user(ioc->buf, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, ioc->len)) {
			    gfx1xm_error("Failed to copy data from kernel to user.");
			    retval = -EFAULT;
			    mutex_unlock(&gfx1xm_dev->buf_lock);
				mutex_unlock(&gfx1xm_dev->fb_lock);
			    break;
			}
	    } else if (ioc->cmd == GFX1XM_W) {
			/*if want to read data from hardware.*/
			gfx1xm_debug(DEFAULT_DEBUG,"Write data to 0x%x, len = 0x%x", ioc->addr, ioc->len);
			if(ioc->addr == GFX1XM_MODE_STATUS) {
			    gfx1xm_dev->mode = ioc->buf[0];
			    gfx1xm_debug(DEFAULT_DEBUG,"set mode 0x%x ", gfx1xm_dev->mode);
			}
			if (ioc->addr == 0x8045){
				u8 buferr[5]={0x00,0x00,0x00,0x00,0x00};
				gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_BUFFER_STATUS, 1, buferr);
				mdelay(200);
			}
			if(copy_from_user(gfx1xm_dev->buffer + GFX1XM_WDATA_OFFSET, ioc->buf, ioc->len)){
			    gfx1xm_error("Failed to copy data from user to kernel.");
			    retval = -EFAULT;
			    mutex_unlock(&gfx1xm_dev->buf_lock);
				mutex_unlock(&gfx1xm_dev->fb_lock);
			    break;
			}
			gfx1xm_spi_write_bytes(gfx1xm_dev, ioc->addr, ioc->len, gfx1xm_dev->buffer);
	    } 
#endif
		else {
			gfx1xm_error("Error command for gfx1xm_ioctl.");
	    }
	    if(ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
	    }
	    mutex_unlock(&gfx1xm_dev->buf_lock);
		mutex_unlock(&gfx1xm_dev->fb_lock);
	    break;
	case GFX1XM_IOC_REINIT:
		mutex_lock(&gfx1xm_dev->fb_lock);
	    disable_irq(gfx1xm_dev->spi->irq);
	    gfx1xm_hw_reset(gfx1xm_dev, 60);
	    enable_irq(gfx1xm_dev->spi->irq);
	    gfx1xm_debug(FLOW_DEBUG,"wake-up gfx1xm");
		mutex_unlock(&gfx1xm_dev->fb_lock);
	    break;
	case GFX1XM_IOC_SETSPEED:
		mutex_lock(&gfx1xm_dev->fb_lock);
	    retval = __get_user(tmp, (u32 __user*)arg);
	    if(tmp > 8*1000*1000) {
			gfx1xm_error("The maximum SPI speed is 8MHz.");
			retval = -EMSGSIZE;
			break;
	    }
	    if(retval == 0) {
			//gfx1xm_spi_set_mode(gfx1xm_dev->spi, tmp, 0);
			gfx1xm_debug(DEFAULT_DEBUG, "spi speed changed to %d", tmp);
	    }	
		mutex_unlock(&gfx1xm_dev->fb_lock);
	    break;
		
//guomingyi add start.
	case GFX1XM_IOC_SET_MODE:
                mutex_lock(&gfx1xm_dev->fb_lock);
		retval = __get_user(tmp, (u32 __user*)arg);
		if(tmp >= GFX1XM_IMAGE_MODE && tmp <= GFX1XM_FF_MODE){
			gfx1xm_spi_write_byte(gfx1xm_dev, 0x8043, tmp);
			gfx1xm_debug(DEFAULT_DEBUG, "[FTM]:set mode:[0x8043][%d]\n", tmp);
		}
                mutex_unlock(&gfx1xm_dev->fb_lock);
	break;
	case GFX1XM_IOC_FTM:
	{
		int tmp1 = 0;
		retval = __get_user(tmp1, (u32 __user*)arg);
		tmp = ftm_gfx_irq_state;
		if(tmp1 == 0) {
			ftm_gfx_irq_state = 0;
		}
        	retval = __put_user(tmp, (__u32 __user *)arg);
	}
	break;
//guomingyi add end.

	default:
	    gfx1xm_error("gfx1xm doesn't support this command(%d)", cmd);
	    break;
    }
    FUNC_EXIT();
    return retval;
}

static unsigned int gfx1xm_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS, &gfx1xm_dev->buf_status);
    if((gfx1xm_dev->buf_status & GFX1XM_BUF_STA_MASK) == GFX1XM_BUF_STA_READY) {
		return (POLLIN|POLLRDNORM);
    } else {
		gfx1xm_debug(DEFAULT_DEBUG, "Poll no data.");
    }
    return 0;
}


#if FW_UPDATE
/*************************************************
**Judge update
**1.0x41E4 is equal to 0xBE or not
**2.Pid is same or not
**3.File's Vid is equal to the IC's Vid or not
**************************************************/
static int isUpdate(struct gfx1xm_dev *gfx1xm_dev)
{
    unsigned char version[16];
    unsigned int ver_fw = 0;
    unsigned int ver_file = 0;
	unsigned char* fw318 = GFX1XM318_FW; 
	unsigned char* fw316 = GFX1XM316_FW; 
    unsigned char fw_running = 0;

    gfx1xm_spi_read_byte(gfx1xm_dev, 0x41e4, &fw_running);
    gfx1xm_debug(DEFAULT_DEBUG,"%s: 0x41e4 = 0x%x", __func__, fw_running);
    if(fw_running == 0xbe) {
		/*firmware running*/		
		if(chip_version_config == 0x38){
			ver_file = (int)(fw318[12] & 0xF0) <<12;
			ver_file |= (int)(fw318[12] & 0x0F)<<8;
			ver_file |= fw318[13];	//get the fw version in the i file;
		}else if(chip_version_config == 0x36){
			ver_file = (int)(fw316[12] & 0xF0) <<12;
			ver_file |= (int)(fw316[12] & 0x0F)<<8;
			ver_file |= fw316[13]; //get the fw version in the i file;
		}

		/*In case we want to upgrade to a special firmware. Such as debug firmware.*/
		if(ver_file != 0x5a5a) {
		    mutex_lock(&gfx1xm_dev->buf_lock);
		    gfx1xm_spi_read_bytes(gfx1xm_dev,0x8000,16,gfx1xm_dev->buffer);
		    memcpy(version, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, 16);
		    mutex_unlock(&gfx1xm_dev->buf_lock);

		    if((version[7]>9) || ((version[8])>9)) {
				gfx1xm_debug(DEFAULT_DEBUG,"version: 8-0x%x; 9-0x%x", version[7], version[8]);
				return 1;
		    }

		    //get the current fw version
		    ver_fw  = (unsigned int)version[7] << 16;
		    ver_fw |= (unsigned int)version[8] << 8;
		    ver_fw |= (unsigned int)version[9];
		    gfx1xm_debug(DEFAULT_DEBUG,"ver_fw: 0x%06x; ver_file:0x%06x", ver_fw, ver_file);

		    if(ver_fw >= ver_file){
				/*If the running firmware is or ahead of the file's firmware. No need to do upgrade.*/
				return 0;
		    }
		}
		gfx1xm_debug(DEFAULT_DEBUG,"Current Ver: 0x%x, Upgrade to Ver: 0x%x", ver_fw, ver_file);
    }else {
		/*no firmware.*/
		gfx1xm_error("No running firmware. Value = 0x%x", fw_running);
    }
    return 1;
}

static int gfx1xm_fw_update_init(struct gfx1xm_dev *gfx1xm_dev)
{
    u8 retry_cnt = 5;     
    u8 value;

    while(retry_cnt--) {
		gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_1MHZ, 0);

		mdelay(10);
		gfx1xm_spi_write_byte(gfx1xm_dev, 0x5081, 0x00);
		gfx1xm_spi_write_byte(gfx1xm_dev, 0x4180, 0x0C);
		gfx1xm_spi_read_byte(gfx1xm_dev, 0x4180, &value);
		if (value == 0x0C)
		{             
		    gfx1xm_debug(DEFAULT_DEBUG,"hold SS51 and DSP successfully!");
		    break;
		}
	}

    if(retry_cnt == 0xFF) {   
		gfx1xm_error("Faile to hold SS51 and DSP.");
		return 0;
    } else {
		gfx1xm_debug(DEFAULT_DEBUG,"Hold retry_cnt=%d",retry_cnt);
		gfx1xm_spi_write_byte(gfx1xm_dev, 0x4010, 0);
		return 1;         
    }  
}
#endif

#if ESD_PROTECT
/***************************************************
**ESD switch function
**
***************************************************/
void gfx1xm_esd_switch(struct gfx1xm_dev *gfx1xm_dev, s32 on)
{
	spin_lock_irq(&gfx1xm_dev->spi_lock);
	if (1 == on) {//switch on esd
		if (0 == gfx1xm_dev->esd_running){
			gfx1xm_dev->esd_running = 1;
			spin_unlock_irq(&gfx1xm_dev->spi_lock);
			queue_delayed_work(gfx1xm_dev->esd_wq, &gfx1xm_dev->esd_check_work, gfx1xm_dev->clk_tick_cnt);
			
		}else {
			spin_unlock_irq(&gfx1xm_dev->spi_lock);
		}
	} else {			//switch off esd
		if (1 == gfx1xm_dev->esd_running) {
			gfx1xm_dev->esd_running = 0;
			spin_unlock_irq(&gfx1xm_dev->spi_lock);
			cancel_delayed_work_sync(&gfx1xm_dev->esd_check_work);
		}else {
			spin_unlock_irq(&gfx1xm_dev->spi_lock);	
		}
	}
}
/***************************************************
**ESD check function
**1.check SPI communication is normal or not
**2.check 0x8040 is equal to 0xC6 or not
**Period:every gfx1xm_dev->clk_tick_cnt second(s)
**
***************************************************/
static void gfx1xm_esd_work(struct work_struct *work)
{	
    unsigned char value[4];
    struct gfx1xm_dev *gfx1xm_dev;
	u8 retry;

    FUNC_ENTRY();
    if(work == NULL)
    {
		gfx1xm_error(" %s wrong work",__func__);
		goto exit;
    }
	
    gfx1xm_dev = container_of(work, struct gfx1xm_dev, esd_check_work);   
	mutex_lock(&gfx1xm_dev->fb_lock);
    if(gfx1xm_dev->mode == GFX1XM_FF_MODE)
		goto exit; 

    for(retry=0;retry<3;retry++){
		gfx1xm_spi_read_bytes(gfx1xm_dev, GFX1XM_CONFIG_DATA, 1, gfx1xm_dev->buffer);
		value[0] = gfx1xm_dev->buffer[GFX1XM_RDATA_OFFSET];
		if(value[0] == 0xC6){
			break;
		}
		mdelay(5);
	}

	if (retry >= 3){
		gfx1xm_debug(DEFAULT_DEBUG, "%s ESD check has a problem", __func__);
		gfx1xm_hw_reset(gfx1xm_dev, 60);
		gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET] = 0x00;
		gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_BUFFER_STATUS+1, 1, gfx1xm_dev->buffer);
		gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET] = gfx1xm_dev->mode;
		gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_MODE_STATUS, 1, gfx1xm_dev->buffer);
		gfx1xm_debug(DEFAULT_DEBUG, "write mode =0x%02X", gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET]);
	}

	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET] = 0xAA;
	gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_CONFIG_DATA, 1, gfx1xm_dev->buffer);

exit:	
	queue_delayed_work(gfx1xm_dev->esd_wq, &gfx1xm_dev->esd_check_work, gfx1xm_dev->clk_tick_cnt);
	mutex_unlock(&gfx1xm_dev->fb_lock);
    FUNC_EXIT(); 
}
#endif

#if CONFIG_HAS_EARLYSUSPEND
static void gfx1xm_early_suspend(struct early_suspend *h)
{    
	struct gfx1xm_dev *gfx1xm_dev = container_of(h, struct gfx1xm_dev, early_fp);
	suspend_flag = 1;
	gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm  suspend.");
#if ESD_PROTECT		
	gfx1xm_esd_switch(gfx1xm_dev, 0);
#endif
}


static void gfx1xm_late_resume(struct early_suspend *h)
{
	struct gfx1xm_dev *gfx1xm_dev = container_of(h, struct gfx1xm_dev, early_fp);	
	gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm  resume");
#if ESD_PROTECT		
	gfx1xm_esd_switch(gfx1xm_dev, 1);
#endif
	suspend_flag = 0;
}
#endif

/*******************************************
**Interrupter
**
*******************************************/
static irqreturn_t gfx1xm_irq(unsigned irq, struct irq_desc *desc)
{
    irq_flag = 1;
    gfx1xm_debug(DEFAULT_DEBUG, "==========gfx1xm: %s ", __func__);
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

/********************************************************
**Interrupt event handler 
**0x8140:
**bit 7    |     bit 6    |    bit 5    |   bit 4   
BUFFER_STA |   IMAGE_EN   | HOME KEY_EN | HOME KEY_STA
** bit 3   |     bit 2    |    bit 1    |   bit 0
MENU KEY_EN| MENU KEY_STA | BACK KEY_EN | BACK KEY_STA
********************************************************/
static int gfx1xm_event_handler(void *para)
{
    struct gfx1xm_dev *gfx1xm_dev = (struct gfx1xm_dev *)para;
    u8 mode = 0x80;
    u8 status, status_1;
	
	do{
		mode = 0x80;
		status = 0x00;
		status_1 = 0x00;
		gfx1xm_debug(DEFAULT_DEBUG, "waiter event");
		wait_event_interruptible(waiter, irq_flag != 0);
		mutex_lock(&gfx1xm_dev->fb_lock);
		irq_flag = 0;
		
		if(suspend_test_flag == 1)
		{
			gfx1xm_debug(DEFAULT_DEBUG,"suspend flag is true: return");
			mutex_unlock(&gfx1xm_dev->fb_lock);
			continue;
		}
		
		
		gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS+1, &status_1);
		if (status_1 & GFX1XM_BUF_STA_MASK){
			gfx1xm_debug(DEFAULT_DEBUG, "%s, ESD reset irq, status_1=0x%02X", __func__, status_1);
			gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS+1, status_1&0x7F);
			if (1 == suspend_flag){
				gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_MODE_STATUS, 0x03); //write to FF mode
			}
			mutex_unlock(&gfx1xm_dev->fb_lock);
			continue;		
		}
	    gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS, &status);
	    gfx1xm_debug(DEFAULT_DEBUG, "IRQ status = 0x%x", status);
	    if(!(status & GFX1XM_BUF_STA_MASK)) {
			gfx1xm_debug(DEFAULT_DEBUG, "Invalid IRQ = 0x%x", status);
			mutex_unlock(&gfx1xm_dev->fb_lock);
			continue;
	    } 
	    gfx1xm_spi_read_byte(gfx1xm_dev, GFX1XM_MODE_STATUS, &mode);
	    gfx1xm_debug(DEFAULT_DEBUG, "status = 0x%x, mode = %d", status, mode);

	    ftm_gfx_irq_state = 0; //guomingyi add .

	    switch(mode){
		case GFX1XM_FF_MODE://FF mode could get data
		    if((status & GFX1XM_HOME_KEY_MASK) && (status & GFX1XM_HOME_KEY_STA)){
				gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm: wake device.");
				input_report_key(gfx1xm_dev->input, GFX1XM_FF_KEY, 1);
				input_sync(gfx1xm_dev->input);	    
				input_report_key(gfx1xm_dev->input, GFX1XM_FF_KEY, 0);
				input_sync(gfx1xm_dev->input);
		    } 
		    
		case GFX1XM_IMAGE_MODE:
#if GFX1XM_FASYNC
		    if(gfx1xm_dev->async) {
				gfx1xm_debug(DEFAULT_DEBUG,"async ");
				kill_fasync(&gfx1xm_dev->async, SIGIO, POLL_IN);
		    }
#endif
		    break;
		case GFX1XM_KEY_MODE:
		    if  ((status & GFX1XM_KEY_MASK) && (status & GFX1XM_BUF_STA_MASK)) {
			gfx1xm_debug(DEFAULT_DEBUG,"gfx1xm:Key mode: status = 0x%x", status);
		//guomingyi add start.
			ftm_gfx_irq_state = (status == 0xb0) ? (1) : (status == 0xa0 ? 2 : 0);
			gfx1xm_spi_write_byte(gfx1xm_dev, 0x8140, 0x00);  
		//guomingyi add end.
		    }
			
		 #if GFX1XM_FASYNC
			if(gfx1xm_dev->async) {
				gfx1xm_debug(DEFAULT_DEBUG, "key async ");
				kill_fasync(&gfx1xm_dev->async, SIGIO, POLL_IN);
			}
		 #endif
		    break;
		case GFX1XM_SLEEP_MODE:
		    gfx1xm_error("gfx1xm:Should not happen in sleep mode.");
		    break;
		case GFX1XM_DEBUG_MODE:
#if GFX1XM_FASYNC
		    if(gfx1xm_dev->async) {
				kill_fasync(&gfx1xm_dev->async, SIGIO, POLL_IN);
		    }
#endif
		    break;
		default:
		    gfx1xm_error("gfx1xm:Unknown mode. mode = 0x%x", mode);
		    break;
	    }
		mutex_unlock(&gfx1xm_dev->fb_lock);
	}while(!kthread_should_stop());
	return 0;
}

#if GFX1XM_FASYNC
static int gfx1xm_fasync(int fd, struct file *filp, int mode)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    int ret;

    FUNC_ENTRY();
    ret = fasync_helper(fd, filp, mode, &gfx1xm_dev->async);
    FUNC_EXIT();
    return ret;
}
#endif

static int gfx1xm_open(struct inode *inode, struct file *filp)
{
    struct gfx1xm_dev *gfx1xm_dev;
    int			status = -ENXIO;

    FUNC_ENTRY();
    mutex_lock(&device_list_lock);

    list_for_each_entry(gfx1xm_dev, &device_list, device_entry) {
		if(gfx1xm_dev->devt == inode->i_rdev) {
		    gfx1xm_debug(DEFAULT_DEBUG, "Found");
		    status = 0;
		    break;
		}
    }

    if(status == 0){
		mutex_lock(&gfx1xm_dev->buf_lock);
		if( gfx1xm_dev->buffer == NULL) {
		    gfx1xm_dev->buffer = kzalloc(bufsiz + GFX1XM_RDATA_OFFSET, GFP_KERNEL);
		    if(gfx1xm_dev->buffer == NULL) {
				dev_dbg(&gfx1xm_dev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
		    }
		}
		mutex_unlock(&gfx1xm_dev->buf_lock);

		if(status == 0) {
		    gfx1xm_dev->users++;
		    filp->private_data = gfx1xm_dev;
		    nonseekable_open(inode, filp);
		    gfx1xm_debug(DEFAULT_DEBUG, "Succeed to open device. irq = %d", gfx1xm_dev->spi->irq);
		    enable_irq(gfx1xm_dev->spi->irq);
		}
	} else {
		gfx1xm_error("No device for minor %d", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

static int gfx1xm_release(struct inode *inode, struct file *filp)
{
    struct gfx1xm_dev *gfx1xm_dev = filp->private_data;
    int    status = 0;
    FUNC_ENTRY();
    mutex_lock(&device_list_lock);

    filp->private_data = NULL;

    /*last close??*/
    gfx1xm_dev->users --;
    if(!gfx1xm_dev->users) {
		gfx1xm_debug(DEFAULT_DEBUG, "disble_irq. irq = %d", gfx1xm_dev->spi->irq);
		disable_irq(gfx1xm_dev->spi->irq);
    }
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}
static long gfx1xm_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return gfx1xm_ioctl(filp, cmd, (unsigned long)(arg));
}

static const struct file_operations gfx1xm_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =	gfx1xm_write,
    .read =		gfx1xm_read,
    .unlocked_ioctl = gfx1xm_ioctl,
	.compat_ioctl	= gfx1xm_compat_ioctl,
    .open =		gfx1xm_open,
    .release =	gfx1xm_release,
    .poll   = gfx1xm_poll,
#if GFX1XM_FASYNC
    .fasync = gfx1xm_fasync,
#endif
};

#if CFG_UPDATE
int gfx1xm_config_update(struct gfx1xm_dev *gfx1xm_dev)
{
	int rery;
	int ret;
	unsigned char tmp;
	unsigned char buffer[16]={0};

	u8 cfg8_info_group0[] = GFX18M_CFG_GROUP0;
	u8 cfg8_info_group1[] = GFX18M_CFG_GROUP1;
	//u8 cfg8_info_group2[] = GFX18M_CFG_GROUP2;

	u8 cfg6_info_group0[] = GFX16M_CFG_GROUP0;
	u8 cfg6_info_group1[] = GFX16M_CFG_GROUP1;
	//u8 cfg6_info_group2[] = GFX16M_CFG_GROUP2;
	
	//init Vendor ID mode		
	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET]=0x08;		
	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET+1]=0xF8;
	gfx1xm_spi_write_bytes(gfx1xm_dev, 0x8041, 2, gfx1xm_dev->buffer);
	rery = 10;
	while(rery--){
		gfx1xm_spi_read_byte(gfx1xm_dev, 0x8042, &tmp);
		if (0xAA == tmp){
			printk("gfx1xm: %s, read Vendor ID\n", __func__);
			break;
		}
		printk("gfx1xm: read 0x8042=0x%02X retry=%d\n", tmp, rery);
		msleep(5);
	}
	//enter Vendor ID mode
	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET]=0x0C;	
	gfx1xm_spi_write_bytes(gfx1xm_dev, 0x5094, 1, gfx1xm_dev->buffer);
	rery = 10;
	while(rery--){
		gfx1xm_spi_read_byte(gfx1xm_dev, 0x5094, &tmp);
		if (0 == tmp){
			printk("gfx1xm: %s, read Vendor ID is ready!!!\n", __func__);
			break;
		}
		printk("gfx1xm: read 0x5094=0x%02X retry=%d\n", tmp, rery);
		msleep(5);
	}
	//read Vendor ID
	gfx1xm_spi_read_bytes(gfx1xm_dev, 0x8148, 10, buffer);
	if(chip_version_config==0x38){		
		if(!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_1, 1)) {
			printk("gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, cfg8_info_group0, GFX1XM_CFG_LEN);
		} else /*if (!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_2, 1)) */{
			printk("gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, cfg8_info_group1, GFX1XM_CFG_LEN);
		} /*else  (!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_3, 1)) {
			printk("gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, cfg8_info_group2, GFX1XM_CFG_LEN);
		}	/*else {	//default config is config318_list[0].config[0].buffer
			printk("gfx1xm: Vendor ID not matched!!! no send\n");
			return 0;
		}*/
	}else if(chip_version_config==0x36){
		if(!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_1, 1)) {
			printk("gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, cfg6_info_group0, GFX1XM_CFG_LEN);
		} else /*if (!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_2, 1))*/ {
			printk("gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, cfg6_info_group1, GFX1XM_CFG_LEN);
		} /*else if (!memcmp(buffer+GFX1XM_RDATA_OFFSET, vendor_id_3, 1)) {
			printk("gfx1xm: %s, read Vendor ID is !!!\n", __func__);
			memcpy(gfx1xm_dev->config + GFX1XM_WDATA_OFFSET, cfg6_info_group2, GFX1XM_CFG_LEN);
		} /*else {	//default config is config316_list[0].config[0].buffer
			printk("gfx1xm: Vendor ID not matched!!! no send\n");
			return 0;
		}*/
	}
	//leave Vendor ID mode
	gfx1xm_dev->buffer[GFX1XM_WDATA_OFFSET]=0x00;
	gfx1xm_spi_write_bytes(gfx1xm_dev, 0x8041, 1, gfx1xm_dev->buffer);	
	
	/*write config*/
	mdelay(100);
	ret = gfx1xm_spi_write_bytes(gfx1xm_dev, GFX1XM_CFG_ADDR, GFX1XM_CFG_LEN, gfx1xm_dev->config);
	if(ret <= 0)
		gfx1xm_error("[info] %s write config fail", __func__);
	mdelay(200);
	return 1;
}
#endif

static int gfx1xm_check_9p_chip(struct gfx1xm_dev *gfx1xm_dev)
{
	u32 time_out =0;
	u8 reg_value[10+GFX1XM_RDATA_OFFSET] = {0};
	
	if (gfx1xm_dev == NULL)
		return 0;
	do{
		gfx1xm_hw_reset(gfx1xm_dev, 40);
		memset(reg_value, 0xFF, 10);
		gfx1xm_spi_read_bytes(gfx1xm_dev, 0x4220, 10, reg_value);
		printk("%s,9p chip version is 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", __func__, reg_value[GFX1XM_RDATA_OFFSET], reg_value[GFX1XM_RDATA_OFFSET+1], reg_value[GFX1XM_RDATA_OFFSET+2], reg_value[GFX1XM_RDATA_OFFSET+3]);
		time_out++;
		if ( (0x00 == reg_value[GFX1XM_RDATA_OFFSET+3])&& (0x90 == reg_value[GFX1XM_RDATA_OFFSET+2]) && (0x08 == reg_value[GFX1XM_RDATA_OFFSET+1])){
			printk("%s,9p chip version check pass!, time_out=%d\n", __func__, time_out);
			return 1;
		}
	}while(time_out<10);
	
	printk("%s,9p chip version check failed!, time_out=%d\n", __func__, time_out);
	return 0;
}


int gfx1xm_dts_init(struct gfx1xm_dev *pdev)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
       if (node) {
        	int ret = -1;
        	printk("[fp] mt_fp_pinctrl+++++++++++++++++\n");

        	gfx1xm.fp_rst_high = pinctrl_lookup_state(gfx1xm.pinctrl1, "fp_rst_high");
        	if (IS_ERR(gfx1xm.fp_rst_high)) {
        		ret = PTR_ERR(gfx1xm.fp_rst_high);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl fp_rst_high!\n");
        		return ret;
        	}
        	gfx1xm.fp_rst_low = pinctrl_lookup_state(gfx1xm.pinctrl1, "fp_rst_low");
        	if (IS_ERR(gfx1xm.fp_rst_low)) {
        		ret = PTR_ERR(gfx1xm.fp_rst_low);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl fp_rst_low!\n");
        		return ret;
        	}
        	gfx1xm.eint_as_int = pinctrl_lookup_state(gfx1xm.pinctrl1, "eint_as_int");
        	if (IS_ERR(gfx1xm.eint_as_int)) {
        		ret = PTR_ERR(gfx1xm.eint_as_int);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl eint_as_int!\n");
        		return ret;
        	}
        	gfx1xm.eint_in_low = pinctrl_lookup_state(gfx1xm.pinctrl1, "eint_in_low");
        	if (IS_ERR(gfx1xm.eint_in_low)) {
        		ret = PTR_ERR(gfx1xm.eint_in_low);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl eint_output_low!\n");
        		return ret;
        	}
        	gfx1xm.eint_in_float = pinctrl_lookup_state(gfx1xm.pinctrl1, "eint_in_float");
        	if (IS_ERR(gfx1xm.eint_in_float)) {
        		ret = PTR_ERR(gfx1xm.eint_in_float);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl eint_output_high!\n");
        		return ret;
        	}
            	gfx1xm.miso_pull_up = pinctrl_lookup_state(gfx1xm.pinctrl1, "miso_pull_up");
        	if (IS_ERR(gfx1xm.miso_pull_up)) {
        		ret = PTR_ERR(gfx1xm.miso_pull_up);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl miso_pull_up!\n");
        		return ret;
        	}
        	gfx1xm.miso_pull_disable = pinctrl_lookup_state(gfx1xm.pinctrl1, "miso_pull_disable");
        	if (IS_ERR(gfx1xm.miso_pull_disable)) {
        		ret = PTR_ERR(gfx1xm.miso_pull_disable);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl miso_pull_disable!\n");
        		return ret;
        	}
               return ret;
        	printk("[FP] mt_fp_pinctrl----------\n");
       }else{

       }
	return 0;
}
static int gfx1xm_irq_init(struct gfx1xm_dev *pdev)
{
    int ret;
    struct device_node *node;
    u32 ints[2] = {0, 0};
    node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");

	if ( node)
	{
		of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "fingerprint-irq");
		gpio_set_debounce(ints[0], ints[1]);
		printk("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		 pdev->spi->irq = irq_of_parse_and_map( node, 0);
	
		printk(" gfx1xm_dev->spi->irq = %d\n",  pdev->spi->irq);
		if (! pdev->spi->irq)
		{
			printk("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
	}
	else
	{
		printk("null irq node!!\n");
		return -EINVAL;
	}
    return 0;
}

/*-------------------------------------------------------------------------*/
static int  gfx1xm_probe(struct spi_device *spi)
{
	struct gfx1xm_dev *gfx1xm_dev = &gfx1xm;
	int status = -1;
	int ret;
	unsigned long minor;
	unsigned char version[16] = {0};
	
	FUNC_ENTRY();
	printk("driver version is %s.\n",GFX1XM_DRIVER_VERSION);

	/* Initialize the driver data */
	gfx1xm_dev->spi = spi;
	spin_lock_init(&gfx1xm_dev->spi_lock);
	mutex_init(&gfx1xm_dev->buf_lock);
	mutex_init(&gfx1xm_dev->fb_lock);
	INIT_LIST_HEAD(&gfx1xm_dev->device_entry);
	
	/* Allocate driver data */
	gfx1xm_dev->buffer = kzalloc(bufsiz + GFX1XM_RDATA_OFFSET, GFP_KERNEL);
	if(gfx1xm_dev->buffer == NULL) {
		status = -ENOMEM;
		goto err_alloc_buffer;
	}
       spi->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
       gfx1xm_dev->reg = regulator_get(&spi->dev, "vfp");
	ret = regulator_set_voltage(gfx1xm_dev->reg, 2800000, 2800000);	/*set 2.8v*/
	if (ret) {
		dev_err("regulator_set_voltage(%d) failed!\n", ret);
		return -1;
	}
    	gfx1xm_dev->pinctrl1 = devm_pinctrl_get(&spi->dev);
    	if (IS_ERR(gfx1xm_dev->pinctrl1)) {
    		ret = PTR_ERR(gfx1xm_dev->pinctrl1);
    		dev_err(&spi->dev, "fwq Cannot find fp pinctrl1!\n");
    		return ret;
    	}
	gfx1xm_dts_init(gfx1xm_dev);
	/*power on*/
       gfx1xm_dev->poweron = 0;
	gfx1xm_power_on(gfx1xm_dev, 1);
	/*setup gfx1xm configurations.*/
	gfx1xm_debug(DEFAULT_DEBUG, "Setting gfx1xm device configuration.");

	/*SPI parameters.*/	
	gfx1xm_spi_setup(gfx1xm_dev, 8*1000*1000);
	gfx1xm_spi_set_mode(gfx1xm_dev->spi, SPEED_1MHZ, 0);

	gfx1xm_spi_pins_config();
	gfx1xm_hw_reset(gfx1xm_dev, 60);
	/*spi test*/
	if (!gfx1xm_check_9p_chip(gfx1xm_dev)){
		gfx1xm_error("%s,9p chip version check failed!", __func__);
		goto err_check_9p;
	}

	full_fp_chip_name(DEV_NAME);
	
	ret = gfx1xm_spi_read_bytes(gfx1xm_dev,0x8000,16,gfx1xm_dev->buffer);
	memcpy(version, gfx1xm_dev->buffer + GFX1XM_RDATA_OFFSET, 16);		
	for(ret = 0; ret <16; ret++)
		gfx1xm_debug(DEFAULT_DEBUG,"chip===>version[%d] = %x ", ret,version[ret]);
#if defined GFX18M_PID
	chip_version_config = 0x38;
#elif defined GFX16M_PID
	chip_version_config = 0x36;
#else
#error "have't define any product for fw"
#endif
	gfx1xm_debug(DEFAULT_DEBUG,"goodix chip_version_config=%x ",chip_version_config);
	
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */

	gfx1xm_config_proc = proc_create(GFX1XM_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
	if (gfx1xm_config_proc == NULL) {
		gfx1xm_error("create proc entry %s success.", GFX1XM_CONFIG_PROC_FILE);
	}else{
		gfx1xm_debug(DEFAULT_DEBUG, "create proc entry %s success.", GFX1XM_CONFIG_PROC_FILE);		  
	}
    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gfx1xm_fops);
    if (status < 0){
		gfx1xm_error("Failed to register char device!");
		goto err_register_char;
    }
    gfx1xm_spi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gfx1xm_spi_class)) {
		gfx1xm_error("Failed to create class.");
		FUNC_EXIT();
		status = PTR_ERR(gfx1xm_spi_class);
		goto err_creat_class;
    }
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		status = sysfs_create_group(&spi->dev.kobj,&gfx1xm_debug_attr_group);
		if(status){
		    gfx1xm_error("Failed to create sysfs file.");
		    goto err_creat_group;
		}
		gfx1xm_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gfx1xm_spi_class, &spi->dev, gfx1xm_dev->devt,
			  gfx1xm_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		gfx1xm_error( "no minor number available!");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gfx1xm_dev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	
	spi_set_drvdata(spi, gfx1xm_dev);
	/*register device within input system.*/
	gfx1xm_dev->input = input_allocate_device();
	if(gfx1xm_dev->input == NULL) {
		gfx1xm_error("Failed to allocate input device.");
		status = -ENOMEM;
		goto err_alloc_input;
	}
	__set_bit(EV_KEY, gfx1xm_dev->input->evbit);
	__set_bit(GFX1XM_INPUT_HOME_KEY, gfx1xm_dev->input->keybit);
	__set_bit(GFX1XM_INPUT_MENU_KEY, gfx1xm_dev->input->keybit);
	__set_bit(GFX1XM_INPUT_BACK_KEY, gfx1xm_dev->input->keybit);
	__set_bit(GFX1XM_FF_KEY, gfx1xm_dev->input->keybit);	
	gfx1xm_dev->input->name = "gf-key";
	if(input_register_device(gfx1xm_dev->input)) {
		gfx1xm_error("Failed to register input device.");
		goto err_free_input;
	}
#if FW_UPDATE
	if(isUpdate(gfx1xm_dev)) {
		unsigned char* fw318 = GFX1XM318_FW; 
		unsigned char* fw316 = GFX1XM316_FW; 
		/*Do upgrade action.*/         
		if(gfx1xm_fw_update_init(gfx1xm_dev)) {
			if(chip_version_config==0x38){
				gfx1xm_fw_update(gfx1xm_dev, fw318, FW_LENGTH);
			}else if(chip_version_config==0x36){
				gfx1xm_fw_update(gfx1xm_dev, fw316, FW_LENGTH);
			}
			gfx1xm_hw_reset(gfx1xm_dev, 60);
		}
	}
#endif

#if CFG_UPDATE
	gfx1xm_config_update(gfx1xm_dev);
#endif

	/*irq config and interrupt init*/
	gfx1xm_gpio_as_int();
	gfx1xm_irq_init(gfx1xm_dev);
	gfx1xm_irq_thread = kthread_run(gfx1xm_event_handler, (void *)gfx1xm_dev, "gfx1xm");
	if (IS_ERR(gfx1xm_irq_thread))
	{
		gfx1xm_error("=====Failed to create kernel thread: %ld", PTR_ERR(gfx1xm_irq_thread));
	}
       ret = request_irq(gfx1xm_dev->spi->irq, (irq_handler_t)gfx1xm_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "FP-eint", NULL);
   	if (ret) {
		pr_err("%s : =====EINT IRQ LINE NOT AVAILABLE  %d\n", __func__,ret);
	} else {
		pr_debug("%s : =====set EINT finished, fp_irq=%d", __func__,
			 gfx1xm_dev->spi->irq);
		disable_irq(gfx1xm_dev->spi->irq);
	}
	//mt_eint_registration(gfx1xm_dev->spi->irq, EINTF_TRIGGER_RISING, gfx1xm_irq, 1);
       //mt_eint_mask(gfx1xm_dev->spi->irq); //mask interrupt
	
#if ESD_PROTECT
	gfx1xm_dev->clk_tick_cnt = 2*HZ;   //2*HZ is 2 seconds
	INIT_DELAYED_WORK(&gfx1xm_dev->esd_check_work, gfx1xm_esd_work);
	gfx1xm_dev->esd_wq = create_workqueue("gfx1xm_esd_check");
	gfx1xm_esd_switch(gfx1xm_dev, 1);
#endif 

#if CONFIG_HAS_EARLYSUSPEND		
	gfx1xm_dev->early_fp.level		= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	gfx1xm_dev->early_fp.suspend	= gfx1xm_early_suspend,		
	gfx1xm_dev->early_fp.resume		= gfx1xm_late_resume,    	
	register_early_suspend(&gfx1xm_dev->early_fp);
#endif

	gfx1xm_spi_write_byte(gfx1xm_dev, GFX1XM_BUFFER_STATUS+1, 0x00);
	gfx1xm_dev->mode = GFX1XM_IMAGE_MODE;		// IC works at Image mode default
	gfx1xm_debug(DEFAULT_DEBUG,"GFX1XM installed.");

	return status;

err_free_input:
	if (gfx1xm_dev->input != NULL)
		input_free_device(gfx1xm_dev->input);
err_alloc_input:
err_creat_group:
	class_destroy(gfx1xm_spi_class);
err_creat_class:	
	unregister_chrdev(SPIDEV_MAJOR, SPI_DEV_NAME);
err_register_char:
err_check_9p:
	if (gfx1xm_dev->buffer !=NULL)
		kfree(gfx1xm_dev->buffer);
       regulator_disable(gfx1xm_dev->reg);
	    regulator_put(gfx1xm_dev->reg);
err_alloc_buffer:	
	FUNC_EXIT();
	return status;

}

static int  gfx1xm_remove(struct spi_device *spi)
{
    struct gfx1xm_dev	*gfx1xm_dev = spi_get_drvdata(spi);
    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if(gfx1xm_dev->spi->irq) {
		free_irq(gfx1xm_dev->spi->irq, gfx1xm_dev);
    }

#if ESD_PROTECT
    destroy_workqueue(gfx1xm_dev->esd_wq);
#endif

    spin_lock_irq(&gfx1xm_dev->spi_lock);
    gfx1xm_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&gfx1xm_dev->spi_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    sysfs_remove_group(&spi->dev.kobj, &gfx1xm_debug_attr_group);
    list_del(&gfx1xm_dev->device_entry);
    device_destroy(gfx1xm_spi_class, gfx1xm_dev->devt);
    clear_bit(MINOR(gfx1xm_dev->devt), minors);
    if (gfx1xm_dev->users == 0) {
		if(gfx1xm_dev->input != NULL)
		    input_unregister_device(gfx1xm_dev->input);

		if(gfx1xm_dev->buffer != NULL)
		    kfree(gfx1xm_dev->buffer);
    }
    mutex_unlock(&device_list_lock);
    class_destroy(gfx1xm_spi_class);
    unregister_chrdev(SPIDEV_MAJOR, SPI_DEV_NAME);
    FUNC_EXIT();
    return 0;
}
static int gfx1xm_suspend_test(struct device *dev)
{
   // g_debug_level |= SUSPEND_DEBUG;
   suspend_test_flag = 1;
   	printk("gfx1xm: %s\n", __func__);
    return 0;
}

static int gfx1xm_resume_test(struct device *dev)
{
    //g_debug &= ~SUSPEND_DEBUG;
     suspend_test_flag = 0;
    printk("gfx1xm: %s\n", __func__);
    return 0;
}
static const struct dev_pm_ops gfx1xm_pm = {
    .suspend = gfx1xm_suspend_test,
    .resume = gfx1xm_resume_test
};

#ifdef CONFIG_OF
static struct of_device_id gfx1xm_of_match[] = {
	{ .compatible = "mediatek,fingerprint", },
	{}
};

MODULE_DEVICE_TABLE(of, gfx1xm_of_match);
#endif

static struct spi_driver gfx1xm_spi_driver = {
    .driver = {
	.name =		SPI_DEV_NAME,
	.owner =	THIS_MODULE,
	.bus	= &spi_bus_type,
	.pm = &gfx1xm_pm,
#ifdef CONFIG_OF
	.of_match_table = gfx1xm_of_match,
#endif
    },
    .probe =	gfx1xm_probe,
    .remove =	gfx1xm_remove,
    //.suspend = gfx1xm_suspend_test,
    //.resume = gfx1xm_resume_test,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};
/*-------------------------------------------------------------------------*/
static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
	    .modalias=SPI_DEV_NAME,
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
		.controller_data = &spi_conf_mt65xx,
	},
};

static int __init gfx1xm_init(void)
{
    int status;

	spi_register_board_info(spi_board_devs,ARRAY_SIZE(spi_board_devs));

    status = spi_register_driver(&gfx1xm_spi_driver);
    if (status < 0) {
		gfx1xm_error("Failed to register SPI driver.");
    }
    return status;
}
module_init(gfx1xm_init);

static void __exit gfx1xm_exit(void)
{
    spi_unregister_driver(&gfx1xm_spi_driver);
}
module_exit(gfx1xm_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gfx1xm-spi");
