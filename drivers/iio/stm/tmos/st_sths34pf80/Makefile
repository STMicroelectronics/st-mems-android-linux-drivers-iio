# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for industrial I/O STHS34PF80 driver
#

include Kbuild

ifeq ($(KERNELRELEASE),)
modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
