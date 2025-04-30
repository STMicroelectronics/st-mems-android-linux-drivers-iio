# SPDX-License-Identifier: GPL-2.0

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_ACC33=m
KBUILD_OPTIONS += CONFIG_IIO_ST_ACC33_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_ACC33_SPI=m

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
