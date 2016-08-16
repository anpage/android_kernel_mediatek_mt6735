include $(srctree)/drivers/misc/mediatek/Makefile.custom
# Makefile for the fingerprint device.

#ifeq ($(CONFIG_FINGER_PRINT_CONPAT),y)
obj-y	+=  fp_drv/
#endif

ifeq ($(CONFIG_FINGERPRINT_ELAN),y)
obj-y	+=  elan/
endif

ifeq ($(CONFIG_FINGERPRINT_GOODIX_GFX1XM),y)
obj-y	+=  gfx1xm/
endif

ifeq ($(CONFIG_FINGER_PRINT_GSL6163),y)
obj-y	+=  gsl6163/
endif


