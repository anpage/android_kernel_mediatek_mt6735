#
# Makefile for misc devices that really don't fit anywhere else.
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek
ccflags-y += -I$(MTK_CUSTOM_PATH)/cam_cal/inc \
             -I$(MTK_CUSTOM_PATH)/camera/$(MTK_PLATFORM)
include $(srctree)/drivers/misc/mediatek/Makefile.custom
custom_cam_cal := $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_CAM_CAL_DRV)))        	
obj-y += $(custom_cam_cal)
