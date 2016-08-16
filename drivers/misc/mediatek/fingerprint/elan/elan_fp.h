#ifndef _LINUX_ELAN_FP_H
#define _LINUX_ELAN_FP_H

#define FINGERPRINT_IOCTL					0x80
#define ID_IOCTL_INIT					_IOW(FINGERPRINT_IOCTL, 0,  int) /* To Get Raw Image (14->8)*/

#define ID_IOCTL_READ_REGISTER				_IOW(FINGERPRINT_IOCTL, 2,  int)
#define ID_IOCTL_WRITE_REGISTER				_IOW(FINGERPRINT_IOCTL, 3,  int)

#define ID_IOCTL_RESET					_IOW(FINGERPRINT_IOCTL, 6,  int)

#define ID_IOCTL_GET_RAW_IMAGE				_IOW(FINGERPRINT_IOCTL, 10, int) /* To Get Raw Image (Original)*/


#define ID_IOCTL_STATUS					_IOW(FINGERPRINT_IOCTL, 12, int)
#define ID_IOCTL_SET_AUTO_RAW_IMAGE			_IOW(FINGERPRINT_IOCTL, 13, int)
#define ID_IOCTL_GET_AUTO_RAW_IMAGE			_IOW(FINGERPRINT_IOCTL, 14, int)
#define ID_IOCTL_READ_CMD				_IOW(FINGERPRINT_IOCTL, 15, int) // General read cmd
#define ID_IOCTL_WRITE_CMD				_IOW(FINGERPRINT_IOCTL, 16, int) // General write cmd
#define ID_IOCTL_IOIRQ_STATUS				_IOW(FINGERPRINT_IOCTL, 17, int) // Use INT to read buffer
#define ID_IOCTL_SPI_STATUS				_IOW(FINGERPRINT_IOCTL, 18, int) // UPdate SPI Speed & CS delay
#define ID_IOCTL_SIG_PID				_IOW(FINGERPRINT_IOCTL, 19, int) // WOE signal event to pid
#define ID_IOCTL_GPIO_PULSE				_IOW(FINGERPRINT_IOCTL, 20, int) // added v1.44 GPIO Pulse, low -> high -> low
#define ID_IOCTL_READ_ALL				_IOW(FINGERPRINT_IOCTL, 21, int) // added v1.441 In IRQ, read all image data, not only one raw.
#define ID_IOCTL_POLL_INIT				_IOW(FINGERPRINT_IOCTL, 22, int) // poll init
#define IOCTL_SPI_CONFIG 				_IOW(FINGERPRINT_IOCTL, 23, int)

#define CUSTOMER_IOCTLID	0xD0	//For customer define

#endif /* _LINUX_ELAN_FP_H */

