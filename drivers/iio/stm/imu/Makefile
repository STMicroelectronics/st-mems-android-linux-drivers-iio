# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Inertial Measurement Units
#

include Kbuild

ifeq ($(KERNELRELEASE),)

# List of subdirectories to build
SUBDIRS := $(shell ls -d */ | tr -d '/')

# Default target
all:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules);)

# Target for building all modules
modules:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules);)

# Target for installing all modules
modules_install:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) modules_install);)

# Target for cleaning all modules
clean:
	$(foreach subdir,$(SUBDIRS),(cd $(subdir) && $(MAKE) clean);)

endif
