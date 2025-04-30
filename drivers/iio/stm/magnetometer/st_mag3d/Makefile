# SPDX-License-Identifier: GPL-2.0

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_ST_MAG3D_IIO=m
KBUILD_OPTIONS += CONFIG_ST_MAG3D_I2C_IIO=m
KBUILD_OPTIONS += CONFIG_ST_MAG3D_SPI_IIO=m

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
