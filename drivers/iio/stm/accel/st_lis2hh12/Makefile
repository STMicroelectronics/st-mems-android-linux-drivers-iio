# SPDX-License-Identifier: GPL-2.0

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_IIO_ST_LIS2HH12=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LIS2HH12_I2C=m
KBUILD_OPTIONS += CONFIG_IIO_ST_LIS2HH12_SPI=m

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS=$(EXTRA_CFLAGS) modules $(KBUILD_OPTIONS)
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules);)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules_install);)

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) clean);)
endif
