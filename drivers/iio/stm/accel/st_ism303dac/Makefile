# SPDX-License-Identifier: GPL-2.0

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_ISM303DAC=m
KBUILD_OPTIONS += CONFIG_IIO_ST_ISM303DAC_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_ISM303DAC_SPI=m

KBUILD_OPTIONS += CONFIG_IIO_ST_ISM303DAC_EN_BASIC_FEATURES=n
KBUILD_OPTIONS += CONFIG_ST_ISM303DAC_IIO_LIMIT_FIFO=0

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules $(KBUILD_OPTIONS)
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules);)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules_install);)

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) clean);)
endif
