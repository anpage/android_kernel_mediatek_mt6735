include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(srctree)/drivers/misc/mediatek/accelerometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/

ifeq ($(strip $(CONFIG_ARCH_MTK_PROJECT)),"p4605")
	ccflags-y +=	-DCONFIG_PROJECT_P4605
endif

ifeq ($(strip $(CONFIG_ARCH_MTK_PROJECT)),"p4601")
	ccflags-y +=	-DCONFIG_PROJECT_P4601
endif

ifeq ($(strip $(CONFIG_ARCH_MTK_PROJECT)),"p6601")
	ccflags-y +=	-DCONFIG_PROJECT_P6601
endif

obj-y	:=  bma222E.o

