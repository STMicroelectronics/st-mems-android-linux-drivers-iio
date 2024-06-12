# SPDX-License-Identifier: GPL-2.0
#
# Makefile for industrial I/O Magnetometer sensor drivers
#

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_ST_MAG3D_IIO=m
KBUILD_OPTIONS += CONFIG_ST_MAG3D_I2C_IIO=m
KBUILD_OPTIONS += CONFIG_ST_MAG3D_SPI_IIO=m
KBUILD_OPTIONS += CONFIG_ST_MAG40_IIO=m
KBUILD_OPTIONS += CONFIG_ST_MAG40_I2C_IIO=m
KBUILD_OPTIONS += CONFIG_ST_MAG40_SPI_IIO=m

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
