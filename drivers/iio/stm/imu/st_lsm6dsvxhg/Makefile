# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for industrial I/O LSM6DSVXHG driver
#

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSVXHG=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSVXHG_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LSM6DSVXHG_SPI=m

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS=$(EXTRA_CFLAGS) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
