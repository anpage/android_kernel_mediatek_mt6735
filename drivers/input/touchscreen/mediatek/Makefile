include $(srctree)/drivers/misc/mediatek/Makefile.custom
#subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT910)	+=  GT910/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XXTB_HOTKNOT)	+=  GT9XXTB_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_I2C_RMI4)	+=  synaptics_i2c_rmi4/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT928)	+=  GT928/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_SCP)	+=  GT9XX_hotknot_scp/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5X0X)	+=  ft5x0x/

obj-$(CONFIG_MTK_TGESTURE)	+=  tgesture/

ifeq ($(CONFIG_TOUCHSCREEN_SYNAPTICS),y)
obj-y	+=  synaptics/
endif

ifeq ($(CONFIG_TOUCHSCREEN_MTK_GT615),y)
obj-y	+=  gt615/
endif

ifeq ($(CONFIG_TINNO_FT6X06),y)
obj-y	+=  focaltech/
endif

ifeq ($(CONFIG_TINNO_FT5346),y)
obj-y	+=  ft5346/
endif

ifeq ($(CONFIG_MTK_FT6X06),y)
obj-y	+=  ft6x06/
endif
