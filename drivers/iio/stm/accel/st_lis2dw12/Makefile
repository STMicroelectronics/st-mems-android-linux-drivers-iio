# SPDX-License-Identifier: GPL-2.0

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_LIS2DW12=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LIS2DW12_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LIS2DW12_SPI=m

KBUILD_OPTIONS += CONFIG_IIO_ST_LIS2DW12_STORE_SAMPLE_FIFO_SUSPEND=n

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
