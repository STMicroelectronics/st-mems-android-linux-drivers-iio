# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for industrial I/O ISM330IS driver
#

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_ISM330IS=m
KBUILD_OPTIONS += CONFIG_IIO_ST_ISM330IS_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_ISM330IS_SPI=m

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
