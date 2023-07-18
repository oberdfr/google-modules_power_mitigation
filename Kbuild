# SPDX-License-Identifier: GPL-2.0

subdir-ccflags-y += \
		-I$(KERNEL_SRC)/../private/google-modules/bms \

obj-$(CONFIG_GOOGLE_BCL) += google_bcl.o
google_bcl-y			+= google_bcl_core.o
google_bcl-y			+= google_bcl_sysfs.o
google_bcl-y			+= google_bcl_util.o
google_bcl-y			+= google_bcl_qos.o
google_bcl-y			+= google_bcl_debugfs.o
