# SPDX-License-Identifier: GPL-2.0-only
st_iis2iclx-y := st_iis2iclx_core.o st_iis2iclx_buffer.o \
		  st_iis2iclx_events.o \
		  st_iis2iclx_shub.o \
		  st_iis2iclx_mlc.o

st_iis2iclx-$(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP) += st_iis2iclx_hwtimestamp.o

obj-$(CONFIG_IIO_ST_IIS2ICLX) += st_iis2iclx.o
obj-$(CONFIG_IIO_ST_IIS2ICLX_I2C) += st_iis2iclx_i2c.o
obj-$(CONFIG_IIO_ST_IIS2ICLX_SPI) += st_iis2iclx_spi.o
