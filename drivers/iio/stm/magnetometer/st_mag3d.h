/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics st_mag3d driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#ifndef __ST_MAG3D_H
#define __ST_MAG3D_H

#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/version.h>

#define LIS3MDL_DEV_NAME		"lis3mdl_magn"
#define LSM9DS1_DEV_NAME		"lsm9ds1_magn"

#define ST_MAG3D_TX_MAX_LENGTH		16
#define ST_MAG3D_RX_MAX_LENGTH		16

#define ST_MAG3D_SAMPLE_SIZE		6

#define ST_MAG3D_EWMA_DIV		128
#define ST_MAG3D_EWMA_WEIGHT		96

#define ST_MAG3D_SHIFT_VAL(val, mask)	(((val) << __ffs(mask)) & (mask))

struct iio_dev;

struct st_mag3d_hw {
	struct device *dev;
	struct mutex lock;
	u8 buffer[ALIGN(ST_MAG3D_SAMPLE_SIZE, sizeof(s64)) + sizeof(s64)];
	u16 odr;
	u16 gain;
	u8 stodis;

	s64 timestamp;
	s64 delta_ts;
	s64 mag_ts;

	struct iio_trigger *trig;
	int irq;

	struct regmap *regmap;
	struct iio_dev *iio_dev;
};

static inline s64 st_mag3d_get_time_ns(struct iio_dev *iio_dev)
{
	return iio_get_time_ns(iio_dev);
}

int st_mag3d_probe(struct device *dev, int irq, const char *name,
		   struct regmap *regmap);
void st_mag3d_remove(struct iio_dev *iio_dev);
int st_mag3d_allocate_buffer(struct iio_dev *iio_dev);
void st_mag3d_deallocate_buffer(struct iio_dev *iio_dev);
int st_mag3d_allocate_trigger(struct iio_dev *iio_dev);
void st_mag3d_deallocate_trigger(struct iio_dev *iio_dev);
int st_mag3d_enable_sensor(struct st_mag3d_hw *hw, bool enable);

#endif /* __ST_MAG3D_H */
