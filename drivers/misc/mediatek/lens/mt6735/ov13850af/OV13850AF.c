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
#include "OV13850AF.h"
// #include "../camera/kd_camera_hw.h"
#include <mt-plat/mt_boot_common.h> // Jiangde++
#include "lens_info.h"

#define LENS_I2C_BUSNUM 0

#define OV13850AF_DRVNAME "OV13850AF"
#define OV13850AF_VCM_WRITE_ID        0x18
#define I2C_REGISTER_ID            0x18
#define PLATFORM_DRIVER_NAME "lens_actuator_OV13850af"
#define AF_DRIVER_CLASS_NAME "actuatordrv_OV13850af"

static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO(OV13850AF_DRVNAME, I2C_REGISTER_ID)};

#define OV13850AF_DEBUG
#ifdef OV13850AF_DEBUG
#define LOG_INF pr_err
#else
#define LOG_INF(x,...)
#endif

static struct i2c_client * g_pstAF_I2Cclient = NULL;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static dev_t g_OV13850AF_devno;
static struct cdev * g_pOV13850AF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4OV13850AF_INF = 0;
static unsigned long g_u4OV13850AF_MACRO = 1023;
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
    LOG_INF("[OV13850AF]  get_byte %d \n",  get_byte);
    return get_byte;
}

static int s4OV13850AF_ReadReg(unsigned short * a_pu2Result)
{
    //int  i4RetValue = 0;
    //char pBuff[2];

    *a_pu2Result = (read_data(0x02) << 8) + (read_data(0x03)&0xff);

    LOG_INF("[OV13850AF]  s4OV13850AF_ReadReg %d \n",  *a_pu2Result);
    return 0;
}

static int s4OV13850AF_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;

    char puSendCmd[3] = {0x02,(char)(a_u2Data >> 8) , (char)(a_u2Data & 0xFF)};

    LOG_INF("[OV13850AF]  write %d \n",  a_u2Data);

#ifdef CONFIG_MTK_LENS_AF_DEBUG
    printk("HJDDbgAF, %s, write addr=0x%x, data=%d \n", __FILE__, OV13850AF_VCM_WRITE_ID, a_u2Data);
#endif

	g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);
	
    if (i4RetValue < 0) 
    {
        LOG_INF("[OV13850AF] I2C send failed!! \n");
        return -1;
    }

    return 0;
}

inline static int getOV13850AFInfo(__user stOV13850AF_MotorInfo * pstMotorInfo)
{
    stOV13850AF_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4OV13850AF_MACRO;
    stMotorInfo.u4InfPosition     = g_u4OV13850AF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = 1;

	if (g_i4MotorStatus == 1)	{stMotorInfo.bIsMotorMoving = 1;}
	else						{stMotorInfo.bIsMotorMoving = 0;}

	if (*g_pAF_Opened >= 1)	{stMotorInfo.bIsMotorOpen = 1;}
	else						{stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stOV13850AF_MotorInfo)))
    {
        LOG_INF("[OV13850AF] copy to user failed when getting motor information \n");
    }

    return 0;
}
static void initdrv(void)
{
	char puSendCmd2[2] = { 0x01, 0x39 };
	char puSendCmd3[2] = { 0x05, 0x65 };
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
}

inline static int moveOV13850AF(unsigned long a_u4Position)
{
    int ret = 0;
 
    if((a_u4Position > g_u4OV13850AF_MACRO) || (a_u4Position < g_u4OV13850AF_INF))
    {
        LOG_INF("[OV13850AF] out of range \n");
        return -EINVAL;
    }

    if (*g_pAF_Opened == 1)
    {
        unsigned short InitPos;
		initdrv();
        ret = s4OV13850AF_ReadReg(&InitPos);
	    
        if(ret == 0)
        {
            LOG_INF("[OV13850AF] Init Pos %6d \n", InitPos);
			
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

    //LOG_INF("[OV13850AF] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);

            spin_lock(g_pAF_SpinLock);
            g_sr = 3;
            g_i4MotorStatus = 0;
            spin_unlock(g_pAF_SpinLock);	
		
            if(s4OV13850AF_WriteReg((unsigned short)g_u4TargetPosition) == 0)
            {
                spin_lock(g_pAF_SpinLock);		
                g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
                spin_unlock(g_pAF_SpinLock);				
            }
            else
            {
                LOG_INF("[OV13850AF] set I2C failed when moving the motor \n");			
                spin_lock(g_pAF_SpinLock);
                g_i4MotorStatus = -1;
                spin_unlock(g_pAF_SpinLock);				
            }

    return 0;
}

inline static int setOV13850AFInf(unsigned long a_u4Position)
{
    spin_lock(g_pAF_SpinLock);
    g_u4OV13850AF_INF = a_u4Position;
    spin_unlock(g_pAF_SpinLock);	
    return 0;
}

inline static int setOV13850AFMacro(unsigned long a_u4Position)
{
    spin_lock(g_pAF_SpinLock);
    g_u4OV13850AF_MACRO = a_u4Position;
    spin_unlock(g_pAF_SpinLock);	
    return 0;	
}

////////////////////////////////////////////////////////////////
static long OV13850AF_Ioctl2(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case OV13850AFIOC_G_MOTORINFO :
            i4RetValue = getOV13850AFInfo((__user stOV13850AF_MotorInfo *)(a_u4Param));
        break;

        case OV13850AFIOC_T_MOVETO :
            i4RetValue = moveOV13850AF(a_u4Param);
        break;
 
        case OV13850AFIOC_T_SETINFPOS :
            i4RetValue = setOV13850AFInf(a_u4Param);
        break;

        case OV13850AFIOC_T_SETMACROPOS :
            i4RetValue = setOV13850AFMacro(a_u4Param);
        break;
		
        default :
      	    LOG_INF("[OV13850AF] No CMD \n");
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
static int OV13850AF_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    LOG_INF("[OV13850AF] OV13850AF_Open - Start\n");

    if(*g_pAF_Opened)
    {
        LOG_INF("[OV13850AF] the device is opened \n");
        return -EBUSY;
    }

    spin_lock(g_pAF_SpinLock);
    *g_pAF_Opened = 1;
    spin_unlock(g_pAF_SpinLock);

    LOG_INF("[OV13850AF] OV13850AF_Open - End\n");

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int OV13850AF_Release2(struct inode * a_pstInode, struct file * a_pstFile)
{
    LOG_INF("[OV13850AF] OV13850AF_Release2 - Start\n");

    if (*g_pAF_Opened == 2)
    {
        g_sr = 5;
    }

    if (*g_pAF_Opened)
    {
        LOG_INF("[OV13850AF] feee \n");

        spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 0;
        spin_unlock(g_pAF_SpinLock);
    }    

    LOG_INF("[OV13850AF] OV13850AF_Release2 - End\n");

    return 0;
}

static const struct file_operations g_stOV13850AF_fops = 
{
    .owner = THIS_MODULE,
    .open = OV13850AF_Open,
    .release = OV13850AF_Release2,
    .unlocked_ioctl = OV13850AF_Ioctl2
};

inline static int Register_OV13850AF_CharDrv(void)
{
    struct device* vcm_device = NULL;

    LOG_INF("[OV13850AF] Register_OV13850AF_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_OV13850AF_devno, 0, 1,OV13850AF_DRVNAME) )
    {
        LOG_INF("[OV13850AF] Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pOV13850AF_CharDrv = cdev_alloc();

    if(NULL == g_pOV13850AF_CharDrv)
    {
        unregister_chrdev_region(g_OV13850AF_devno, 1);

        LOG_INF("[OV13850AF] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pOV13850AF_CharDrv, &g_stOV13850AF_fops);

    g_pOV13850AF_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pOV13850AF_CharDrv, g_OV13850AF_devno, 1))
    {
        LOG_INF("[OV13850AF] Attatch file operation failed\n");

        unregister_chrdev_region(g_OV13850AF_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, AF_DRIVER_CLASS_NAME);
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        LOG_INF("Unable to create class, err = %d\n", ret);
        return ret;            
    }

    vcm_device = device_create(actuator_class, NULL, g_OV13850AF_devno, NULL, OV13850AF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    LOG_INF("[OV13850AF] Register_OV13850AF_CharDrv - End\n");    
    return 0;
}

inline static void Unregister_OV13850AF_CharDrv(void)
{
    LOG_INF("[OV13850AF] Unregister_OV13850AF_CharDrv - Start\n");

    //Release char driver
    cdev_del(g_pOV13850AF_CharDrv);

    unregister_chrdev_region(g_OV13850AF_devno, 1);
    
    device_destroy(actuator_class, g_OV13850AF_devno);

    class_destroy(actuator_class);

    LOG_INF("[OV13850AF] Unregister_OV13850AF_CharDrv - End\n");    
}

//////////////////////////////////////////////////////////////////////

static int OV13850AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int OV13850AF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id OV13850AF_i2c_id[] = {{OV13850AF_DRVNAME,0},{}};   
struct i2c_driver OV13850AF_i2c_driver = {                       
    .probe = OV13850AF_i2c_probe,                                   
    .remove = OV13850AF_i2c_remove,                           
    .driver.name = OV13850AF_DRVNAME,                 
    .id_table = OV13850AF_i2c_id,                             
};  

#if 0 
static int OV13850AF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, OV13850AF_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int OV13850AF_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int OV13850AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    LOG_INF("[OV13850AF] OV13850AF_i2c_probe\n");

    /* Kirby: add new-style driver { */
    g_pstAF_I2Cclient = client;
    
    g_pstAF_I2Cclient->addr = OV13850AF_VCM_WRITE_ID >> 1;
    
    //Register char driver
    i4RetValue = Register_OV13850AF_CharDrv();

    if(i4RetValue){

        LOG_INF("[OV13850AF] register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(g_pAF_SpinLock);

    LOG_INF("[OV13850AF] Attached!! \n");

    return 0;
}

static int OV13850AF_probe(struct platform_device *pdev)
{
	LOG_INF("OV13850AF_probe \n");
    return i2c_add_driver(&OV13850AF_i2c_driver);
}

static int OV13850AF_remove(struct platform_device *pdev)
{
    i2c_del_driver(&OV13850AF_i2c_driver);
    return 0;
}

static int OV13850AF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int OV13850AF_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stOV13850AF_Driver = {
    .probe		= OV13850AF_probe,
    .remove		= OV13850AF_remove,
    .suspend	= OV13850AF_suspend,
    .resume		= OV13850AF_resume,
    .driver		= {
        .name	= PLATFORM_DRIVER_NAME,
        .owner	= THIS_MODULE,
    }
};

static struct platform_device g_stOV13850AF_device = {
    .name = PLATFORM_DRIVER_NAME,
    .id = 0,
    .dev = {}
};

static int __init OV13850AF_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);

	if(platform_device_register(&g_stOV13850AF_device)){
        LOG_INF("failed to register OV13850AF device\n");
        return -ENODEV;
    }
	
    if(platform_driver_register(&g_stOV13850AF_Driver)){
        LOG_INF("failed to register OV13850AF driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit OV13850AF_i2C_exit(void)
{
	platform_driver_unregister(&g_stOV13850AF_Driver);
}



/* ////////////////////////////////////////////////////////////// */
long OV13850AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getOV13850AFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveOV13850AF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setOV13850AFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setOV13850AFMacro(a_u4Param);
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
int OV13850AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

    if (*g_pAF_Opened == 2 && FACTORY_BOOT != get_boot_mode())
    {
        LOG_INF("HJDDbgAF, Releasing, g_u4CurrPosition=%d \n", g_u4CurrPosition);
        if (g_u4CurrPosition > 500)
        {
            moveOV13850AF(500);
            LOG_INF("HJDDbgAF, Wait--moveOV13850AF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 400)
        {
            moveOV13850AF(400);
            LOG_INF("HJDDbgAF, Wait--moveOV13850AF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 300)
        {
            moveOV13850AF(300);
            LOG_INF("HJDDbgAF, Wait--moveOV13850AF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 200)
        {
            moveOV13850AF(200);
            LOG_INF("HJDDbgAF, Wait--moveOV13850AF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 100)
        {
            moveOV13850AF(100);
            LOG_INF("HJDDbgAF, Wait--moveOV13850AF(%d) \n", g_u4CurrPosition);
            msleep(10);
        }

        if (g_u4CurrPosition > 50)
        {
            moveOV13850AF(50);
            LOG_INF("HJDDbgAF, Wait--moveOV13850AF(%d) \n", g_u4CurrPosition);
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

void OV13850AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	*g_pAF_Opened = pAF_Opened;
}

// module_init(OV13850AF_i2C_init);
// module_exit(OV13850AF_i2C_exit);

// MODULE_DESCRIPTION("OV13850AF lens module driver");
// MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
// MODULE_LICENSE("GPL");
