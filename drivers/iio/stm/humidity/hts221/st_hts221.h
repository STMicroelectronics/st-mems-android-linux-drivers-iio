/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics st_hts221 sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#ifndef ST_HTS221_H
#define ST_HTS221_H

#include <linux/iio/iio.h>

#define ST_HTS221_DEV_NAME	"sst_hts221"
#define ST_HTS221_DATA_SIZE	2

enum st_hts221_sensor_type {
	ST_HTS221_SENSOR_H,
	ST_HTS221_SENSOR_T,
	ST_HTS221_SENSOR_MAX,
};

struct st_hts221_sensor {
	u8 cur_avg_idx;
	int slope, b_gen;
};

struct st_hts221_hw {
	const char *name;
	struct device *dev;
	struct regmap *regmap;

	struct iio_trigger *trig;
	int irq;

	struct st_hts221_sensor sensors[ST_HTS221_SENSOR_MAX];

	bool enabled;
	u8 odr;
};

extern const struct dev_pm_ops st_hts221_pm_ops;

int st_hts221_probe(struct device *dev, int irq, const char *name,
		    struct regmap *regmap);
int st_hts221_set_enable(struct st_hts221_hw *hw, bool enable);
int st_hts221_allocate_buffers(struct iio_dev *iio_dev);
int st_hts221_allocate_trigger(struct iio_dev *iio_dev);

#endif /* ST_HTS221_H */
