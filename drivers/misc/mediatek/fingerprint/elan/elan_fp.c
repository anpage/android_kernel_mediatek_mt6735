#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/wakelock.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif
#include "../../../../spi/mediatek/mt6735/mt_spi.h"
#include <linux/miscdevice.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>

#include "../fp_drv/fp_drv.h"

#define ELAN_FP_NAME "elan_fp"

#define ELAN_CHIP_ID 0x5a3c

#define SPI_ALINE_BYTE 4

#define SPI_MAX_SPEED 3*1000*1000

/**********MTK GPIO INIT*************/
//static int elan_fp_reset = (GPIO63 | 0x80000000);
//static int elan_fp_intr = (GPIO1 | 0x80000000);
//static int elan_fp_irq_num = 1;

/**********MTK GPIO INIT*************/
struct delayed_work check_work;
struct workqueue_struct *check_spi_wq;
#define PRINT_ELAN_INFO
#ifdef PRINT_ELAN_INFO 
	#define elan_info(fmt, args...) do{\
		if(debug_flage)\
                printk("[elan debug]:"fmt"\n", ##args);\
	}while(0);
#else
	#define elan_info(fmt, args...)
#endif

struct elan_fp_platform_data {
	unsigned short version;
	int intr_gpio;
	int rst_gpio;
};

struct elan_fp_data {
	struct spi_device *spi;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_fp_wq;
	struct work_struct work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
       struct miscdevice elan_fp_mdev;
	char *image_buf;
	char *tx_buf;
	char *rx_buf;
       int irq;
       struct regulator *reg;
       struct pinctrl *pinctrl1;
       struct pinctrl_state *pins_default;
       struct pinctrl_state *eint_as_int, *eint_in_low, *eint_in_float, *fp_rst_low, *fp_rst_high,*miso_pull_up,*miso_pull_disable;

};

/*********************************ioctl data********************************************/
#define ELAN_IOCTLID	0xD0
#define IOCTL_RESET	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_IRQ_MASK _IOW(ELAN_IOCTLID, 2, int)
#define IOCTL_READ_MODE _IOW(ELAN_IOCTLID, 3, int)
#define IOCTL_WORK_MODE _IOW(ELAN_IOCTLID, 4, int)
#define IOCTL_SET_XY _IOW(ELAN_IOCTLID, 5, int)
#define IOCTL_SET_SCAN_FLAG _IOW(ELAN_IOCTLID, 6, int)
#define IOCTL_POWER_KEY _IOW(ELAN_IOCTLID, 7, int)
#define IOCTL_SPI_CONFIG _IOW(ELAN_IOCTLID, 8, int)
#define IOCTL_DEBUG_LOG_SWITCH _IOW(ELAN_IOCTLID, 9, int)
#define IOCTL_READ_KEY_STATUS _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_WRITE_KEY_STATUS _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_WAKE_UP_SUSPEND _IOW(ELAN_IOCTLID, 12, int)

/*********************************global data********************************************/
static struct elan_fp_data *elan_fp = NULL;
static int elan_work_mode = 0;
static int elan_read_mode = 0;
static int elan_interrupt_cnt = 0;
static int image_ok_flag = 0;
static int elan_work_flag = 0;
static bool debug_flage = true;
static int key_status = 0;

static int max_width_pix = 120;
static int max_heigh_pix = 120;
static int total_byte = 120 * 120 * 2;

static DECLARE_WAIT_QUEUE_HEAD(elan_poll_wq);
static DECLARE_WAIT_QUEUE_HEAD(image_waiter);
static struct fasync_struct *fasync_queue = NULL;
static struct wake_lock elan_wake_lock;


/************************************** function list**************************************/
static DEFINE_MUTEX(efsa120s_set_gpio_mutex);
static void efsa120s_gpio_as_int(void)
{
	mutex_lock(&efsa120s_set_gpio_mutex);
	printk("[efsa120s]efsa120s_gpio_as_int\n");
	pinctrl_select_state(elan_fp->pinctrl1, elan_fp->eint_as_int);
	mutex_unlock(&efsa120s_set_gpio_mutex);
}

static void efsa120s_reset_output(int level)
{
	mutex_lock(&efsa120s_set_gpio_mutex);
	printk("[efsa120s]efsa120s_reset_output level = %d   ,%d,   %d\n", level,elan_fp->pinctrl1,elan_fp->fp_rst_low);

	if (level)
		pinctrl_select_state(elan_fp->pinctrl1, elan_fp->fp_rst_high);
	else
		pinctrl_select_state(elan_fp->pinctrl1, elan_fp->fp_rst_low);
	mutex_unlock(&efsa120s_set_gpio_mutex);
}
static void elan_fp_reset(void)
{
	printk("[elan]:%s enter\n", __func__);
	//mt_set_gpio_out(elan_fp_reset, 0);
       efsa120s_reset_output(0);
	mdelay(5);
	//mt_set_gpio_out(elan_fp_reset, 1);
       efsa120s_reset_output(1);
	mdelay(50);
}

void efsa120s_gpio_power(int onoff)
{
	int ret;
       printk("%s onoff = %d", __func__, onoff);
       if(onoff){
           ret = regulator_enable(elan_fp->reg);	/*enable regulator*/
           if (ret)
               printk("regulator_enable() failed!\n");
       }else{
           ret = regulator_disable(elan_fp->reg);	/*disable regulator*/
           if (ret)
               printk("regulator_disable() failed!\n");
       }
}

static void elan_fp_switch_irq(int on)
{
    printk("[elan] %s enter, irq = %d, on = %d\n", __func__, elan_fp->irq, on);
    if (on){
        //mt_eint_unmask(elan_fp_irq_num);
        enable_irq(elan_fp->irq);
    }
    else {
        //mt_eint_mask(elan_fp_irq_num);
        disable_irq(elan_fp->irq);
    }
}

static irqreturn_t elan_fp_irq_handler(int irq, void *dev_id)
{
	elan_info("%s============= enter", __func__);
	//elan_fp_switch_irq(0);
	queue_work(elan_fp->elan_fp_wq, &elan_fp->work);
	return IRQ_HANDLED;
}

static void elan_fp_gpio_int_config(struct elan_fp_data *fp)
{
	int ret = -1;

	printk("[elan]:%s enter\n", __func__);
    struct device_node *node;
    u32 ints[2] = {0, 0};
    node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");

	if ( node)
	{
		of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "fingerprint-irq");
		gpio_set_debounce(ints[0], ints[1]);
              efsa120s_gpio_as_int();
		printk("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		 fp->irq = irq_of_parse_and_map( node, 0);
	
		printk(" efsa120s->irq = %d\n",  fp->irq);
		if (! fp->irq)
		{
			printk("irq_of_parse_and_map fail!!\n");
			return ;
		}
	}
	else
	{
		printk("null irq node!!\n");
		return ;
	}
       //efsa120s_gpio_as_int();
       ret = request_irq(fp->irq, elan_fp_irq_handler,
				IRQF_TRIGGER_NONE, "FP-eint", NULL);
   	if (ret) {
		pr_err("%s : =====EINT IRQ LINE NOT AVAILABLE  %d\n", __func__,ret);
	} else {
		pr_debug("%s : =====set EINT finished, fp_irq=%d", __func__,
			 fp->irq);
              elan_fp_switch_irq(0);
              msleep(20);
              elan_fp_switch_irq(1);
	}
    return ;
/*	printk("[elan_reset] %d\n", elan_fp_reset);
	printk("[elan_intr] %d\n", elan_fp_intr);
	printk("[elan_irq] %d\n", elan_fp_irq_num);

	//set reset output high
	mt_set_gpio_dir(elan_fp_reset, GPIO_DIR_OUT);

	//set int pin input
	mt_set_gpio_dir(elan_fp_intr, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(elan_fp_intr, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(elan_fp_intr, GPIO_PULL_UP);

	mt_eint_registration(elan_fp_irq_num, EINTF_TRIGGER_RISING, elan_fp_irq_handler, 1);
	mt_eint_unmask(elan_fp_irq_num);*/
}

static int elan_fp_spi_transfer(struct spi_device *spi, const char *txbuf, char *rxbuf, int len)
{
	struct spi_transfer t;
	struct spi_message m;
	
	memset(&t, 0, sizeof(t));
	spi_message_init(&m);
	t.tx_buf = txbuf;
	t.rx_buf = rxbuf;
	t.bits_per_word = 8;
	t.len = len;
	spi_message_add_tail(&t, &m);
	
	return spi_sync(spi, &m);
}

static int elan_fp_alloc_image_buffer(struct elan_fp_data *fp)
{
	int alloc_len = 0;

	if (fp == NULL) {
		printk("[elan error] %s: allocate elan_fp_data failed\n", __func__);
		return -ENOMEM;
	}

	if(fp->image_buf){
		kfree(fp->image_buf);		
	}

	//(line len + dummy(1))*line counts + cmd(1) + 4Byte alignment(MTK DMA mode)
	alloc_len = ((max_width_pix*2+1)*max_heigh_pix + 1 + (1024-1))/ 1024 * 1024;
	fp->image_buf = kzalloc(sizeof(char)*alloc_len, GFP_KERNEL);
		if (fp->image_buf == NULL) {
		printk("[elan error] %s: allocate image_buf failed\n", __func__);
		return -ENOMEM;
	}

	//max len + cmd(1) + dummy(1) + 4Byte alignment(MTK DMA mode)
	alloc_len = (max_width_pix*2 + 2 + (SPI_ALINE_BYTE-1))/ SPI_ALINE_BYTE * SPI_ALINE_BYTE;

	if(fp->tx_buf){
		kfree(fp->tx_buf);		
	}
	fp->tx_buf = kzalloc(sizeof(char)*alloc_len, GFP_KERNEL);
	if (fp->tx_buf == NULL) {
		printk("[elan error] %s: allocate tx_buf failed\n", __func__);
		return -ENOMEM;
	}

	if(fp->rx_buf){
		kfree(fp->rx_buf);		
	}
	fp->rx_buf = kzalloc(sizeof(char)*alloc_len, GFP_KERNEL);
	if (fp->rx_buf == NULL) {
		printk("[elan error] %s: allocate rx_buf failed\n", __func__);
		return -ENOMEM;
	}
	
	return 0;
}

static int elan_fp_open(struct inode *inode, struct file *filp)
{ 
	elan_info("%s enter", __func__);
	if (elan_fp == NULL){
		printk("elan_fp is NULL~~~");
	}
	return 0;
}

static int elan_fp_release(struct inode *inode, struct file *filp)
{    
	elan_info("%s enter", __func__);
	return 0;
}

ssize_t elan_fp_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	ssize_t ret = 0;;
	
	elan_info("%s enter", __func__);
	
	if ( elan_read_mode == 1 ){
		printk("[elan] register read \n");
		ret = copy_to_user(buff, elan_fp->rx_buf, count);
	}
	else{
		ret = wait_event_timeout(image_waiter, image_ok_flag!= 0, 1*HZ);
		if ( ret != 0 ){
			ret = copy_to_user(buff, elan_fp->image_buf, total_byte);
			elan_info("image is readly ");
			ret = 1;
		}
	}
	
	return image_ok_flag;
}

static ssize_t elan_fp_write(struct file *filp, const char *buff, size_t count, loff_t *offp)
{
	int ret = 0;
	
	memset(elan_fp->tx_buf, 0, max_width_pix);
	
   	ret = copy_from_user(elan_fp->tx_buf, buff, count);
	elan_fp_spi_transfer(elan_fp->spi, elan_fp->tx_buf, elan_fp->rx_buf, count);
	
	return 0;
}

static long elan_fp_ioctl( struct file *filp, unsigned int cmd, unsigned long arg)
{
	int buf[8] = {0};
	int ret = 0;
	int i = 0;
	struct mt_chip_conf *spi_conf_mt65xx;
	
	elan_info("%s enter, cmd value %x",__func__, cmd);

	switch (cmd) {
		case IOCTL_RESET:
			elan_fp_reset();
			break;
		case IOCTL_IRQ_MASK:
			elan_fp_switch_irq(arg);
			break;
		case IOCTL_READ_MODE:
			elan_read_mode = arg;
			//printk("[elan] IOCTL_READ_MODE %x \n", elan_read_mode);
			break;
		case IOCTL_WORK_MODE:
			elan_work_mode = arg;
			//printk("[elan] IOCTL_WORK_MODE %x \n", elan_work_mode);
			break;
		case IOCTL_SET_XY:
			ret = copy_from_user(buf, (int *)arg, 2*sizeof(int));
			if(ret)
				break;
			max_width_pix = buf[0]; 
			max_heigh_pix = buf[1];
			total_byte = max_width_pix * max_heigh_pix * 2;
			//printk("[elan] max_width_pix: %d max_heigh_pix:%d \n", max_width_pix, max_heigh_pix);
			break;
		case IOCTL_SET_SCAN_FLAG:
			image_ok_flag = 0;
			elan_interrupt_cnt = 0;
			break;
		case IOCTL_POWER_KEY:
			input_report_key(elan_fp->input_dev, KEY_POWER, 1);
			input_report_key(elan_fp->input_dev, KEY_POWER, 0);
			input_sync(elan_fp->input_dev);
			break;
		case IOCTL_SPI_CONFIG:
			ret = copy_from_user(buf, (int *)arg, 6*sizeof(int));
			if(ret)
				break;
			printk("[elan]: ");
			for(i=0; i<6; i++)
				printk("%x ", buf[i]);
			printk("\n");
			spi_conf_mt65xx = (struct mt_chip_conf *) elan_fp->spi->controller_data;
			spi_conf_mt65xx->setuptime = buf[0];
			spi_conf_mt65xx->holdtime = buf[1];
			spi_conf_mt65xx->high_time = buf[2];
			spi_conf_mt65xx->low_time = buf[3];
			spi_conf_mt65xx->cs_idletime = buf[4];
			spi_conf_mt65xx->ulthgh_thrsh = buf[5];
			break;
		case IOCTL_DEBUG_LOG_SWITCH:
			debug_flage = (arg==0?false:true);
			break;
		case IOCTL_READ_KEY_STATUS:
			return key_status;
		case IOCTL_WRITE_KEY_STATUS:
			key_status = arg;
			if (fasync_queue){
				kill_fasync(&fasync_queue, SIGIO, POLL_IN);
			}
			break;
		case IOCTL_WAKE_UP_SUSPEND:
			if(arg){
				wake_lock(&elan_wake_lock);
			}
			else{
				wake_unlock(&elan_wake_lock);
			}
			break;
		default:
			break;
	}
	return ret;
}

static unsigned int elan_fp_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	
	poll_wait(file, &elan_poll_wq, wait);
	
	if(elan_work_flag > 0){
		mask = elan_work_flag;
	}
	
	elan_work_flag = 0;
	return mask;
}

static int elan_fp_fasync(int fd, struct file * filp, int on)
{
	printk("%s enter \n",__func__);
	return fasync_helper(fd, filp, on, &fasync_queue);
}

static struct file_operations elan_fp_fops = {    
	.open = elan_fp_open,  
	.release =  elan_fp_release,
	.read = elan_fp_read,
	.write = elan_fp_write,
	.unlocked_ioctl = elan_fp_ioctl, 
	.compat_ioctl = elan_fp_ioctl,
	.poll = elan_fp_poll,
	.fasync = elan_fp_fasync,
};

static int elan_fp_suspend(struct spi_device *spi, pm_message_t mesg)
{
	printk("[elan]:%s enter\n", __func__);
	
	return 0;
}

static int elan_fp_resume(struct spi_device *spi)
{
	printk("[elan]:%s enter\n", __func__);
	
	return 0;
}

static void elan_fp_early_suspend(struct early_suspend *h)
{
	elan_fp_suspend(elan_fp->spi, PMSG_SUSPEND);
}

static void elan_fp_late_resume(struct early_suspend *h)
{
	elan_fp_resume(elan_fp->spi);
}

static void elan_fp_work(struct work_struct *work)
{
	int len = max_width_pix*2+2;
	
	elan_fp->tx_buf[0] = 0x10;

recv_one_frame_image:
	if (elan_work_mode & 0x01 || elan_work_mode & 0x08){
		elan_info("elan_fp_work ->woe mode WAKEUP or frame recve !\n");	
	}
	else{
		if (elan_work_mode & 0x02){
			//adc 8bit mode
			len = max_width_pix + 2;
			elan_fp_spi_transfer(elan_fp->spi, elan_fp->tx_buf, elan_fp->rx_buf, len);
		}
		else{
			//adc 14bit mode
			elan_fp_spi_transfer(elan_fp->spi, elan_fp->tx_buf, elan_fp->rx_buf, len);
		}
		//copy data to image, sheet 2byte(cmd+dummy)
		if (elan_interrupt_cnt < max_heigh_pix){
			memcpy(&elan_fp->image_buf[(len-2)*elan_interrupt_cnt], &elan_fp->rx_buf[2], len-2);
		}
	}
	
	if( elan_work_mode & 0x01 ){
		elan_interrupt_cnt = 0;
		elan_work_flag = 1;
		wake_up(&elan_poll_wq);
	}
	else{
		elan_interrupt_cnt++;
		elan_info("elan_interrupt_cnt = %d", elan_interrupt_cnt);
		
		if( (elan_interrupt_cnt == max_heigh_pix) ){
			elan_interrupt_cnt = 0;
			image_ok_flag = 1;
			wake_up(&image_waiter);
		}
		else if(elan_work_mode & 0x04){
			//one frame image mode
			goto recv_one_frame_image;
		}
		else if(elan_work_mode & 0x08){
			elan_info("recv alll pix in one transfer!\n");
			if(elan_work_mode & 0x02){
				len = ((max_width_pix+1)*max_heigh_pix + 1 + (1024-1))/ 1024 * 1024;
			}
			else{
				len = ((max_width_pix*2+1)*max_heigh_pix + 1 + (1024-1))/ 1024 * 1024;
			}
			elan_fp->tx_buf[0] = 0x10;
			elan_fp_spi_transfer(elan_fp->spi, elan_fp->tx_buf, elan_fp->image_buf, len);
			for(elan_interrupt_cnt = 0; elan_interrupt_cnt < max_heigh_pix; elan_interrupt_cnt++){
				if(elan_interrupt_cnt == 0){         		
					memcpy(elan_fp->image_buf, &elan_fp->image_buf[2], max_width_pix*2);
				}
				else{						
					memcpy(&elan_fp->image_buf[max_width_pix*2*elan_interrupt_cnt], \
						&elan_fp->image_buf[(max_width_pix*2+1)*elan_interrupt_cnt + 2], max_width_pix*2);
				}
    		}
			elan_interrupt_cnt = 0;
			image_ok_flag = 1;
			wake_up(&image_waiter);
		}
	}
	
	//elan_fp_switch_irq(1);
}

static int elan_fp_detect_id(void)
{
	char *cmd = elan_fp->tx_buf;
	int i = 0;
	uint16_t chip_id = 0;

	//write to register
	cmd[0] = 0x80;
	cmd[1] = 0x5A;
	cmd[2] = 0x81;
	cmd[3] = (ELAN_CHIP_ID) >> 8;
	cmd[4] = 0x82;
	cmd[5] = (ELAN_CHIP_ID) & 0x00FF;
	for(i=0; i<6; i+=2)
		elan_fp_spi_transfer(elan_fp->spi, cmd+i, NULL, 2);

	//read register
	memset(elan_fp->tx_buf, 0, max_width_pix);
	cmd[0] = 0xC1;
	elan_fp_spi_transfer(elan_fp->spi, cmd, elan_fp->rx_buf, 4);
	//elan_fp_reset();
	
	for(i = 0; i < 4; i++)
		printk("%x ", elan_fp->rx_buf[i]);
	printk("\n");
	chip_id = (elan_fp->rx_buf[2] << 8) + elan_fp->rx_buf[3];

	if(chip_id == ELAN_CHIP_ID){
		printk("[elan] ELAN_CHIP_ID = %x detect ok \n", chip_id);
		cmd[0] = 0x80+0x2A;
		cmd[1] = 0x00;
		elan_fp_spi_transfer(elan_fp->spi, cmd, NULL, 2);
		printk("[elan] set power down ok \n");
		return 0;
	}
	else{
		printk("[elan] ELAN_CHIP_ID = %x detect error \n", chip_id);
		return -1;
	}
}
int efsa120s_dts_init(struct elan_fp_data *pdev)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
       if (node) {
        	int ret;
        	printk("[fp] mt_fp_pinctrl+++++++++++++++++\n");

        	pdev->fp_rst_high = pinctrl_lookup_state(pdev->pinctrl1, "fp_rst_high");
        	if (IS_ERR(pdev->fp_rst_high)) {
        		ret = PTR_ERR(pdev->fp_rst_high);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl fp_rst_high!\n");
        		return ret;
        	}
        	pdev->fp_rst_low = pinctrl_lookup_state(pdev->pinctrl1, "fp_rst_low");
        	if (IS_ERR(pdev->fp_rst_low)) {
        		ret = PTR_ERR(pdev->fp_rst_low);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl fp_rst_low!\n");
        		return ret;
        	}
        	pdev->eint_as_int = pinctrl_lookup_state(pdev->pinctrl1, "eint_as_int");
        	if (IS_ERR(pdev->eint_as_int)) {
        		ret = PTR_ERR(pdev->eint_as_int);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl eint_as_int!\n");
        		return ret;
        	}
        	pdev->eint_in_low = pinctrl_lookup_state(pdev->pinctrl1, "eint_in_low");
        	if (IS_ERR(pdev->eint_in_low)) {
        		ret = PTR_ERR(pdev->eint_in_low);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl eint_output_low!\n");
        		return ret;
        	}
        	pdev->eint_in_float = pinctrl_lookup_state(pdev->pinctrl1, "eint_in_float");
        	if (IS_ERR(pdev->eint_in_float)) {
        		ret = PTR_ERR(pdev->eint_in_float);
        		dev_err(&pdev->spi->dev, "fwq Cannot find fp pinctrl eint_output_high!\n");
        		return ret;
        	}
               return ret;
        	printk("[FP] mt_fp_pinctrl----------\n");
       }else{

       }
	return 0;
}

static void check_spi_work(struct work_struct *work)
{	
    efsa120s_gpio_power(1);
    elan_fp_detect_id();
    queue_delayed_work(check_spi_wq, &check_work, 2*HZ);
}

static int elan_fp_probe(struct spi_device *spi)
{
	struct elan_fp_data *fp = NULL;
	struct elan_fp_platform_data *pdata = spi->dev.platform_data;
	int err = 0;

	printk("[elan]:%s enter\n", __func__);

	spi->bits_per_word = 8;
	
	fp = kzalloc(sizeof(struct elan_fp_data), GFP_KERNEL);
	if (fp == NULL) {
		printk("[elan error] %s: allocate elan_fp_data failed\n", __func__);
		return -ENOMEM;
	}
	err = elan_fp_alloc_image_buffer(fp);
	if(err != 0){
		return -ENOMEM;		
	}
	
	fp->spi = spi;
	elan_fp = fp;
       spi_setup(fp->spi);
       spi->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
       fp->reg = regulator_get(&spi->dev, "vfp");
	err = regulator_set_voltage(fp->reg, 2800000, 2800000);	/*set 2.8v*/
    	if (err) {
		dev_err("regulator_set_voltage(%d) failed!\n", err);
		return -1;
	}
    	fp->pinctrl1 = devm_pinctrl_get(&spi->dev);
    	if (IS_ERR(fp->pinctrl1)) {
    		err = PTR_ERR(fp->pinctrl1);
    		dev_err(&spi->dev, "fwq Cannot find fp pinctrl1!\n");
    		return err;
    	}
       efsa120s_dts_init(fp);
       efsa120s_gpio_power(1);
       msleep(10);
       elan_fp_reset();
	err=elan_fp_detect_id();
       if(-1==err){
           dev_err(&spi->dev,"======detect elan id failed!\n");
           regulator_disable(fp->reg);
	    regulator_put(fp->reg);
           return err;
        }

	full_fp_chip_name(ELAN_FP_NAME);
	   
	fp->elan_fp_mdev.minor = MISC_DYNAMIC_MINOR;	
	fp->elan_fp_mdev.name = "elan_fp";
	fp->elan_fp_mdev.fops = &elan_fp_fops;
	fp->elan_fp_mdev.mode = S_IFREG|S_IRWXUGO; 
	if (misc_register(&fp->elan_fp_mdev) < 0)
		printk("[elan] misc_register failed!!\n");
	else
		printk("[elan] misc_register ok!!\n");
		
	/*if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}
	else{
	//	fp->rst_gpio = pdata->rst_gpio;
       // fp->intr_gpio = pdata->intr_gpio;
        //fp->elan_irq = gpio_to_irq(fp->intr_gpio);
	}*/
	elan_fp_gpio_int_config(fp);
	printk("[elan] SPI max speed %d \n", spi->max_speed_hz);
	
	INIT_WORK(&elan_fp->work, elan_fp_work);
	fp->elan_fp_wq = create_singlethread_workqueue(ELAN_FP_NAME); 
	if (!fp->elan_fp_wq) {
		printk("[elan error] %s: create_singlethread_workqueue failed\n", __func__);
		return -EINVAL;
	}
	printk("[elan]:%s elan_fp_gpio_int_config ok \n", __func__);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	fp->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	fp->early_suspend.suspend = elan_fp_early_suspend;
	fp->early_suspend.resume = elan_fp_late_resume;
	register_early_suspend(&fp->early_suspend);
#endif

	fp->input_dev = input_allocate_device();
    if (fp->input_dev == NULL) {
        printk("[elan error] Failed to allocate input device\n");
        return -EINVAL;;
    }

	fp->input_dev->evbit[0] = BIT(EV_KEY)|BIT_MASK(EV_REP);
	__set_bit(KEY_POWER, fp->input_dev->keybit);
	
	fp->input_dev->name = ELAN_FP_NAME;
	fp->input_dev->phys = "input/fp"; 
	fp->input_dev->id.bustype = BUS_SPI; 
	fp->input_dev->id.vendor = 0xDEAD; 
	fp->input_dev->id.product = 0xBEEF; 
	fp->input_dev->id.version = 2015;

	err = input_register_device(fp->input_dev);
	if (err) {
		input_free_device(fp->input_dev);
		printk("[elan error]%s: unable to register %s input device\n", __func__, fp->input_dev->name);
		return err;
	}
//       INIT_DELAYED_WORK(&check_work, check_spi_work);
//       check_spi_wq = create_workqueue("spi_check");
//       queue_delayed_work(check_spi_wq, &check_work, 2*HZ);

    wake_lock_init(&elan_wake_lock, WAKE_LOCK_SUSPEND, "elan_wake_lock");
	printk("[elan]:++++++++++%s end ++++++++++++ \n", __func__);

	return 0;
}

static int elan_fp_remove(struct spi_device *spi)
{
	printk("[elan]:%s enter\n", __func__);
       struct elan_fp_data	*elan_data = spi_get_drvdata(spi);
       disable_irq(elan_data->irq);
       if(spi->irq) {
		free_irq(elan_data->irq, elan_data);
       }
	return 0;
}

static const struct spi_device_id efp_id[] = {
	{ELAN_FP_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(spi, efp_id);
#ifdef CONFIG_OF
static struct of_device_id efsa120s_of_match[] = {
	{ .compatible = "mediatek,fingerprint", },
	{}
};

MODULE_DEVICE_TABLE(of, efsa120s_of_match);
#endif
static struct spi_driver efp_driver = {
	.driver = {
		.name = ELAN_FP_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
	       .of_match_table = efsa120s_of_match,
#endif
	},
	.probe 	= elan_fp_probe,
	.remove = elan_fp_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = elan_fp_suspend,
	.resume = elan_fp_resume,
#endif
	.id_table = efp_id,
	
};

static struct mt_chip_conf spi_conf_mt65xx = {
	//SPI speed
	.setuptime = 3,
	.holdtime = 3,
	.high_time = 10,
	.low_time = 10,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	
	//SPI mode
	.cpol = 0,
	.cpha = 0,
	
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	
	.tx_endian = 0,
	.rx_endian = 0,
	
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

static struct spi_board_info elan_fp_spi_devs[] __initdata = {
	{
		.modalias		= ELAN_FP_NAME,
		.bus_num 		= 0,
		.chip_select 	= 0,
		.max_speed_hz	= SPI_MAX_SPEED,
		.mode 			= SPI_MODE_0,
		.controller_data = (void*)&spi_conf_mt65xx,
	},
};

static int __init elan_fp_init(void)
{
	printk("[elan]:%s enter\n", __func__);
	spi_register_board_info(elan_fp_spi_devs, ARRAY_SIZE(elan_fp_spi_devs));
	return spi_register_driver(&efp_driver);
}

static void __exit elan_fp_exit(void)
{
	printk("[elan]:%s enter\n", __func__);
	spi_unregister_driver(&efp_driver);
}

module_init(elan_fp_init);
module_exit(elan_fp_exit);

MODULE_DESCRIPTION("elan finger print driver");
MODULE_LICENSE("GPL");
