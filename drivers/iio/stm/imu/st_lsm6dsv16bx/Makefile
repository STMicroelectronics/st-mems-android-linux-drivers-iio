# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for industrial I/O LSM6DSV16BX driver
#

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSV16BX=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSV16BX_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSV16BX_SPI=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSV16BX_ASYNC_HW_TIMESTAMP=y

EXTRA_CFLAGS += "-DCONFIG_IIO_ST_LSM6DSV16BX_ASYNC_HW_TIMESTAMP -DCONFIG_IIO_ST_LSM6DSV16BX_MLC_PRELOAD"

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS=$(EXTRA_CFLAGS) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
