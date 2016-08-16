
# Makefile for the linux kernel.
#
# Only aarch32 need to build core/ (need machine descriptor)

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARM64), y)
# For arm32
obj-y += mt_cpuxgpt.o
obj-y += mt_gpt.o
obj-y += ca53_timer.o

## Hibernation
obj-$(CONFIG_MTK_HIBERNATION)	+= cpu_hibernate.o
# Module project related part
#obj-y += $(ARCH_MTK_PROJECT)/

else
############################ For arm64 #############################

## Hibernation
obj-$(CONFIG_MTK_HIBERNATION)  += cpu_hibernate64.o
obj-y += mt_cpuxgpt.o
obj-y += mt_gpt.o
obj-y += ca53_timer.o
# Module project related part
#obj-y += $(ARCH_MTK_PROJECT)/

endif
WIKO_UNIFY_DEF := $(strip $(subst ro.wiko.type =,,$(shell cat $(PWD)/build.ini |grep ^ro.wiko.type)))
ifeq ($(strip $(WIKO_UNIFY_DEF)),unify)
ccflags-y += -DCONFIG_WIKO_UNIFY
$(warning ---add marco CONFIG_WIKO_UNIFY---)
endif
#common part
