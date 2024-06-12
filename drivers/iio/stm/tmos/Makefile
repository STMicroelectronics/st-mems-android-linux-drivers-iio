# SPDX-License-Identifier: GPL-2.0
#
# Makefile for industrial I/O STHS34PF80 driver
#

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_STHS34PF80=m
KBUILD_OPTIONS += CONFIG_IIO_ST_STHS34PF80_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_STHS34PF80_SPI=m

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
