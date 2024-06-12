# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for industrial I/O LSM6DSO16IS driver
#

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSO16IS=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSO16IS_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSO16IS_SPI=m

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
