# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for industrial I/O LSM6DSM driver
#

include Kbuild

ifeq ($(KERNELRELEASE),)

KBUILD_OPTIONS += CONFIG_ST_LSM6DSM_IIO=m
KBUILD_OPTIONS += CONFIG_ST_LSM6DSM_I2C_IIO=m
KBUILD_OPTIONS += CONFIG_ST_LSM6DSM_SPI_IIO=m
KBUILD_OPTIONS += CONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT=y

EXTRA_CFLAGS += "-DCONFIG_ST_LSM6DSM_IIO_LIMIT_FIFO=0 \
		 -DCONFIG_ST_LSM6DSM_IIO_MASTER_SUPPORT \
		 -DCONFIG_ST_LSM6DSM_IIO_EXT0_LIS2MDL"

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) EXTRA_CFLAGS=$(EXTRA_CFLAGS) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
