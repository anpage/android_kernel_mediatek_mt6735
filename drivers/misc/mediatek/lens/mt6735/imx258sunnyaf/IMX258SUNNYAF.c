/*
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "IMX258SUNNYAF.h"
// #include "../camera/kd_camera_hw.h"
#include <mt-plat/mt_boot_common.h> // Jiangde++
#include "lens_info.h"

#define LENS_I2C_BUSNUM 0

#define IMX258SUNNYAF_DRVNAME "IMX258SUNNYAF"
#define IMX258SUNNYAF_VCM_WRITE_ID        0x18
#define I2C_REGISTER_ID            0x1c
#define PLATFORM_DRIVER_NAME "lens_actuator_imx258sunnyaf"
#define AF_DRIVER_CLASS_NAME "actuatordrv_imx258sunnyaf"

static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO(IMX258SUNNYAF_DRVNAME, I2C_REGISTER_ID)};

#define IMX258SUNNYAF_DEBUG
#ifdef IMX258SUNNYAF_DEBUG
#define LOG_INF pr_debug
#else
#define LOG_INF(x,...)
#endif


static struct i2c_client * g_pstAF_I2Cclient = NULL;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static dev_t g_IMX258SUNNYAF_devno;
static struct cdev * g_pIMX258SUNNYAF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4IMX258SUNNYAF_INF = 0;
static unsigned long g_u4IMX258SUNNYAF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition   = 0;

static int g_sr = 3;

extern unsigned int get_boot_mode(void); // Jiangde++

static int i2c_read(u8 a_u2Addr , u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)(a_u2Addr)};
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 1);
	if (i4RetValue != 2) {
	    LOG_INF(" I2C write failed!! \n");
	    return -1;
	}
	//
	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
	    LOG_INF(" I2C read failed!! \n");
	    return -1;
	}
   
    return 0;
}

static u8 read_data(u8 addr)
{
	u8 get_byte=0;
    i2c_read( addr ,&get_byte);
    LOG_INF("[IMX258SUNNYAF]  get_byte %d \n",  get_byte);
    return get_byte;
}

static int s4IMX258SUNNYAF_ReadReg(unsigned short * a_pu2Result)
{
    //int  i4RetValue = 0;
    //char pBuff[2];

    *a_pu2Result = (read_data(0x02) << 8) + (read_data(0x03)&0xff);

    LOG_INF("[IMX258SUNNYAF]  s4IMX258SUNNYAF_ReadReg %d \n",  *a_pu2Result);
    return 0;
}

static int s4IMX258SUNNYAF_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;

    char puSendCmd[3] = {0x02,(char)(a_u2Data >> 8) , (char)(a_u2Data & 0xFF)};

    LOG_INF("[IMX258SUNNYAF]  write %d \n",  a_u2Data);

#ifdef CONFIG_MTK_LENS_AF_DEBUG
    printk("HJDDbgAF, %s, write addr=0x%x, data=%d \n", __FILE__, IMX258SUNNYAF_VCM_WRITE_ID, a_u2Data);
#endif

	g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);
	
    if (i4RetValue < 0) 
    {
        LOG_INF("[IMX258SUNNYAF] I2C send failed!! \n");
        return -1;
    }

    return 0;
}

inline static int getIMX258SUNNYAFInfo(__user stIMX258SUNNYAF_MotorInfo * pstMotorInfo)
{
    stIMX258SUNNYAF_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4IMX258SUNNYAF_MACRO;
    stMotorInfo.u4InfPosition     = g_u4IMX258SUNNYAF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = 1;

	if (g_i4MotorStatus == 1)	{stMotorInfo.bIsMotorMoving = 1;}
	else						{stMotorInfo.bIsMotorMoving = 0;}

	if (*g_pAF_Opened >= 1)	{stMotorInfo.bIsMotorOpen = 1;}
	else						{stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stIMX258SUNNYAF_MotorInfo)))
    {
        LOG_INF("[IMX258SUNNYAF] copy to user failed when getting motor information \n");
    }

    return 0;
}
static void initdrv(void)
{
	char puSendCmd2[2] = { 0x01, 0x38 };
	char puSendCmd3[2] = { 0x05, 0x01 };
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
}

inline static int moveIMX258SUNNYAF(unsigned long a_u4Position)
{
    int ret = 0;
 
    if((a_u4Position > g_u4IMX258SUNNYAF_MACRO) || (a_u4Position < g_u4IMX258SUNNYAF_INF))
    {
        LOG_INF("[IMX258SUNNYAF] out of range \n");
        return -EINVAL;
    }

    if (*g_pAF_Opened == 1)
    {
        unsigned short InitPos;
		initdrv();
        ret = s4IMX258SUNNYAF_ReadReg(&InitPos);
	    
        if(ret == 0)
        {
            LOG_INF("[IMX258SUNNYAF] Init Pos %6d \n", InitPos);
			
			spin_lock(g_pAF_SpinLock);
            g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);
        }
        else
        {		
			spin_lock(g_pAF_SpinLock);
            g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
        }
		
		spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 2;
        spin_unlock(g_pAF_SpinLock);
		
    }

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(g_pAF_SpinLock);	
        g_i4Dir = 1;
        spin_unlock(g_pAF_SpinLock);	
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(g_pAF_SpinLock);	
        g_i4Dir = -1;
        spin_unlock(g_pAF_SpinLock);			
    }
    else	
	{return 0;}

	    spin_lock(g_pAF_SpinLock);    
	    g_u4TargetPosition = a_u4Position;
	    spin_unlock(g_pAF_SpinLock);	

	    //LOG_INF("[IMX258SUNNYAF] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);

        spin_lock(g_pAF_SpinLock);
        g_sr = 3;
        g_i4MotorStatus = 0;
        spin_unlock(g_pAF_SpinLock);	
	
        if(s4IMX258SUNNYAF_WriteReg((unsigned short)g_u4TargetPosition) == 0)
        {
            spin_lock(g_pAF_SpinLock);		
            g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
            spin_unlock(g_pAF_SpinLock);				
        }
        else
        {
            LOG_INF("[IMX258SUNNYAF] set I2C failed when moving the motor \n");			
            spin_lock(g_pAF_SpinLock);
            g_i4MotorStatus = -1;
            spin_unlock(g_pAF_SpinLock);				
        }

    return 0;
}

inline static int setIMX258SUNNYAFInf(unsigned long a_u4Position)
{
    spin_lock(g_pAF_SpinLock);
    g_u4IMX258SUNNYAF_INF = a_u4Position;
    spin_unlock(g_pAF_SpinLock);	
    return 0;
}

inline static int setIMX258SUNNYAFMacro(unsigned long a_u4Position)
{
    spin_lock(g_pAF_SpinLock);
    g_u4IMX258SUNNYAF_MACRO = a_u4Position;
    spin_unlock(g_pAF_SpinLock);	
    return 0;	
}

////////////////////////////////////////////////////////////////
static long IMX258SUNNYAF_Ioctl2(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case IMX258SUNNYAFIOC_G_MOTORINFO :
            i4RetValue = getIMX258SUNNYAFInfo((__user stIMX258SUNNYAF_MotorInfo *)(a_u4Param));
        break;

        case IMX258SUNNYAFIOC_T_MOVETO :
            i4RetValue = moveIMX258SUNNYAF(a_u4Param);
        break;
 
        case IMX258SUNNYAFIOC_T_SETINFPOS :
            i4RetValue = setIMX258SUNNYAFInf(a_u4Param);
        break;

        case IMX258SUNNYAFIOC_T_SETMACROPOS :
            i4RetValue = setIMX258SUNNYAFMacro(a_u4Param);
        break;
		
        default :
      	    LOG_INF("[IMX258SUNNYAF] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    return i4RetValue;
}

//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
// 3.Update f_op pointer.
// 4.Fill data structures into private_data
//CAM_RESET
static int IMX258SUNNYAF_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    LOG_INF("[IMX258SUNNYAF] IMX258SUNNYAF_Open - Start\n");

    if(*g_pAF_Opened)
    {
        LOG_INF("[IMX258SUNNYAF] the device is opened \n");
        return -EBUSY;
    }

    spin_lock(g_pAF_SpinLock);
    *g_pAF_Opened = 1;
    spin_unlock(g_pAF_SpinLock);

    LOG_INF("[IMX258SUNNYAF] IMX258SUNNYAF_Open - End\n");

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int IMX258SUNNYAF_Release2(struct inode * a_pstInode, struct file * a_pstFile)
{
    LOG_INF("[IMX258SUNNYAF] IMX258SUNNYAF_Release - Start\n");

    if (*g_pAF_Opened == 2)
    {
        g_sr = 5;
    }

    if (*g_pAF_Opened)
    {
        LOG_INF("[IMX258SUNNYAF] feee \n");

        spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 0;
        spin_unlock(g_pAF_SpinLock);
    }    

    LOG_INF("[IMX258SUNNYAF] IMX258SUNNYAF_Release - End\n");

    return 0;
}

static const struct file_operations g_stIMX258SUNNYAF_fops = 
{
    .owner = THIS_MODULE,
    .open = IMX258SUNNYAF_Open,
    .release = IMX258SUNNYAF_Release2,
    .unlocked_ioctl = IMX258SUNNYAF_Ioctl2
};

inline static int Register_IMX258SUNNYAF_CharDrv(void)
{
    struct device* vcm_device = NULL;

    LOG_INF("[IMX258SUNNYAF] Register_IMX258SUNNYAF_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_IMX258SUNNYAF_devno, 0, 1,IMX258SUNNYAF_DRVNAME) )
    {
        LOG_INF("[IMX258SUNNYAF] Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pIMX258SUNNYAF_CharDrv = cdev_alloc();

    if(NULL == g_pIMX258SUNNYAF_CharDrv)
    {
        unregister_chrdev_region(g_IMX258SUNNYAF_devno, 1);

        LOG_INF("[IMX258SUNNYAF] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pIMX258SUNNYAF_CharDrv, &g_stIMX258SUNNYAF_fops);

    g_pIMX258SUNNYAF_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pIMX258SUNNYAF_CharDrv, g_IMX258SUNNYAF_devno, 1))
    {
        LOG_INF("[IMX258SUNNYAF] Attatch file operation failed\n");

        unregister_chrdev_region(g_IMX258SUNNYAF_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, AF_DRIVER_CLASS_NAME);
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        LOG_INF("Unable to create class, err = %d\n", ret);
        return ret;            
    }

    vcm_device = device_create(actuator_class, NULL, g_IMX258SUNNYAF_devno, NULL, IMX258SUNNYAF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    LOG_INF("[IMX258SUNNYAF] Register_IMX258SUNNYAF_CharDrv - End\n");    
    return 0;
}

inline static void Unregister_IMX258SUNNYAF_CharDrv(void)
{
    LOG_INF("[IMX258SUNNYAF] Unregister_IMX258SUNNYAF_CharDrv - Start\n");

    //Release char driver
    cdev_del(g_pIMX258SUNNYAF_CharDrv);

    unregister_chrdev_region(g_IMX258SUNNYAF_devno, 1);
    
    device_destroy(actuator_class, g_IMX258SUNNYAF_devno);

    class_destroy(actuator_class);

    LOG_INF("[IMX258SUNNYAF] Unregister_IMX258SUNNYAF_CharDrv - End\n");    
}

//////////////////////////////////////////////////////////////////////

static int IMX258SUNNYAF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int IMX258SUNNYAF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id IMX258SUNNYAF_i2c_id[] = {{IMX258SUNNYAF_DRVNAME,0},{}};   
struct i2c_driver IMX258SUNNYAF_i2c_driver = {                       
    .probe = IMX258SUNNYAF_i2c_probe,                                   
    .remove = IMX258SUNNYAF_i2c_remove,                           
    .driver.name = IMX258SUNNYAF_DRVNAME,                 
    .id_table = IMX258SUNNYAF_i2c_id,                             
};  

#if 0 
static int IMX258SUNNYAF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, IMX258SUNNYAF_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int IMX258SUNNYAF_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int IMX258SUNNYAF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    LOG_INF("[IMX258SUNNYAF] IMX258SUNNYAF_i2c_probe\n");

    /* Kirby: add new-style driver { */
    g_pstAF_I2Cclient = client;
    
    g_pstAF_I2Cclient->addr = IMX258SUNNYAF_VCM_WRITE_ID >> 1;
    
    //Register char driver
    i4RetValue = Register_IMX258SUNNYAF_CharDrv();

    if(i4RetValue){

        LOG_INF("[IMX258SUNNYAF] register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(g_pAF_SpinLock);

    LOG_INF("[IMX258SUNNYAF] Attached!! \n");

    return 0;
}

static int IMX258SUNNYAF_probe(struct platform_device *pdev)
{
	LOG_INF("IMX258SUNNYAF_probe \n");
    return i2c_add_driver(&IMX258SUNNYAF_i2c_driver);
}

static int IMX258SUNNYAF_remove(struct platform_device *pdev)
{
    i2c_del_driver(&IMX258SUNNYAF_i2c_driver);
    return 0;
}

static int IMX258SUNNYAF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int IMX258SUNNYAF_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stIMX258SUNNYAF_Driver = {
    .probe		= IMX258SUNNYAF_probe,
    .remove		= IMX258SUNNYAF_remove,
    .suspend	= IMX258SUNNYAF_suspend,
    .resume		= IMX258SUNNYAF_resume,
    .driver		= {
        .name	= PLATFORM_DRIVER_NAME,
        .owner	= THIS_MODULE,
    }
};

static struct platform_device g_stIMX258SUNNYAF_device = {
    .name = PLATFORM_DRIVER_NAME,
    .id = 0,
    .dev = {}
};

static int __init IMX258SUNNYAF_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);

	if(platform_device_register(&g_stIMX258SUNNYAF_device)){
        LOG_INF("failed to register IMX258SUNNYAF device\n");
        return -ENODEV;
    }
	
    if(platform_driver_register(&g_stIMX258SUNNYAF_Driver)){
        LOG_INF("failed to register IMX258SUNNYAF driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit IMX258SUNNYAF_i2C_exit(void)
{
	platform_driver_unregister(&g_stIMX258SUNNYAF_Driver);
}

/* ////////////////////////////////////////////////////////////// */
long IMX258SUNNYAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getIMX258SUNNYAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveIMX258SUNNYAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setIMX258SUNNYAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setIMX258SUNNYAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int IMX258SUNNYAF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

    if (*g_pAF_Opened == 2 && FACTORY_BOOT != get_boot_mode())
    {
        LOG_INF("HJDDbgAF, Releasing, g_u4CurrPosition=%d \n", g_u4CurrPosition);
        if (g_u4CurrPosition > 500)
        {
            moveIMX258SUNNYAF(500);
            LOG_INF("HJDDbgAF, Wait--moveIMX258SUNNYAF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 400)
        {
            moveIMX258SUNNYAF(400);
            LOG_INF("HJDDbgAF, Wait--moveIMX258SUNNYAF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 300)
        {
            moveIMX258SUNNYAF(300);
            LOG_INF("HJDDbgAF, Wait--moveIMX258SUNNYAF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 200)
        {
            moveIMX258SUNNYAF(200);
            LOG_INF("HJDDbgAF, Wait--moveIMX258SUNNYAF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 100)
        {
            moveIMX258SUNNYAF(100);
            LOG_INF("HJDDbgAF, Wait--moveIMX258SUNNYAF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 50)
        {
            moveIMX258SUNNYAF(50);
            LOG_INF("HJDDbgAF, Wait--moveIMX258SUNNYAF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        LOG_INF("HJDDbgAF, Released, g_u4CurrPosition=%d \n", g_u4CurrPosition);
    }
    
	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

void IMX258SUNNYAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}

module_init(IMX258SUNNYAF_i2C_init);
module_exit(IMX258SUNNYAF_i2C_exit);

// MODULE_DESCRIPTION("IMX258SUNNYAF lens module driver");
// MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
// MODULE_LICENSE("GPL");


