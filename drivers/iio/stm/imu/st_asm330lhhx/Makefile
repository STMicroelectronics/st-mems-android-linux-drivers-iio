# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for industrial I/O ASM330LHHX driver
#

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_ASM330LHHX=m
KBUILD_OPTIONS += CONFIG_IIO_ST_ASM330LHHX_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_ASM330LHHX_SPI=m

KBUILD_OPTIONS += CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP=y
KBUILD_OPTIONS += CONFIG_IIO_ST_ASM330LHHX_MLC_PRELOAD=y
KBUILD_OPTIONS += CONFIG_IIO_ST_ASM330LHHX_STORE_SAMPLE_FIFO_SUSPEND=y

EXTRA_CFLAGS += "-DCONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP -DCONFIG_IIO_ST_ASM330LHHX_MLC_PRELOAD -DCONFIG_IIO_ST_ASM330LHHX_STORE_SAMPLE_FIFO_SUSPEND"

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS=$(EXTRA_CFLAGS) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
