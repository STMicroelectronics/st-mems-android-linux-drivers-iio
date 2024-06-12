# SPDX-License-Identifier: GPL-2.0-only
#
# STM Makefile for Industrial I/O drivers
#

include Kbuild

ifeq ($(KERNELRELEASE),)
# List of subdirectories to build
SUBDIRS := accel imu magnetometer pressure temperature tmos

all:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules);)

modules:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules);)

modules_install:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules_install);)

uninstall:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) uninstall);)

clean:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) clean);)
endif
