// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lsm6dsvx imu sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2022 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>

#include <linux/platform_data/st_sensors_pdata.h>

#include "st_lsm6dsvx.h"

/**
 * List of supported device settings
 *
 * The following table list all device supported by st_lsm6dsvx driver.
 */
static const struct st_lsm6dsvx_settings st_lsm6dsvx_sensor_settings[] = {
	{
		.id = {
			.hw_id = ST_LSM6DSVX_ID,
			.name = ST_LSM6DSV16X_DEV_NAME,
		},
		.st_qvar_probe = true,
	},
};

static const struct st_lsm6dsvx_odr_table_entry
st_lsm6dsvx_odr_table[] = {
	[ST_LSM6DSVX_ID_ACC] = {
		.size = 8,
		.reg = {
			.addr = ST_LSM6DSVX_REG_CTRL1_ADDR,
			.mask = GENMASK(6, 0),
		},
		/*                 odr   val batch */
		.odr_avl[1] = {   7, 0, 0x03, 0x02 },
		.odr_avl[2] = {  15, 0, 0x03, 0x03 },
		.odr_avl[3] = {  30, 0, 0x04, 0x04 },
		.odr_avl[4] = {  60, 0, 0x05, 0x05 },
		.odr_avl[5] = { 120, 0, 0x06, 0x06 },
		.odr_avl[6] = { 240, 0, 0x07, 0x07 },
		.odr_avl[7] = { 480, 0, 0x08, 0x08 },
		.odr_avl[8] = { 960, 0, 0x09, 0x09 },
	},
	[ST_LSM6DSVX_ID_GYRO] = {
		.size = 8,
		.reg = {
			.addr = ST_LSM6DSVX_REG_CTRL2_ADDR,
			.mask = GENMASK(6, 0),
		},
		/* G LP MODE 7 Hz batch 7 Hz */
		.odr_avl[1] = {   7, 0, 0x52, 0x02 },
		.odr_avl[2] = {  15, 0, 0x03, 0x03 },
		.odr_avl[3] = {  30, 0, 0x04, 0x04 },
		.odr_avl[4] = {  60, 0, 0x05, 0x05 },
		.odr_avl[5] = { 120, 0, 0x06, 0x06 },
		.odr_avl[6] = { 240, 0, 0x07, 0x07 },
		.odr_avl[7] = { 480, 0, 0x08, 0x08 },
		.odr_avl[8] = { 960, 0, 0x09, 0x09 },
	}
};

static const struct st_lsm6dsvx_fs_table_entry st_lsm6dsvx_fs_table[] = {
	[ST_LSM6DSVX_ID_ACC] = {
		.size = 4,
		.fs_avl[0] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL8_ADDR,
				.mask = GENMASK(1, 0),
			},
			.gain = ST_LSM6DSVX_ACC_FS_2G_GAIN,
			.val = 0x0,
		},
		.fs_avl[1] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL8_ADDR,
				.mask = GENMASK(1, 0),
			},
			.gain = ST_LSM6DSVX_ACC_FS_4G_GAIN,
			.val = 0x1,
		},
		.fs_avl[2] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL8_ADDR,
				.mask = GENMASK(1, 0),
			},
			.gain = ST_LSM6DSVX_ACC_FS_8G_GAIN,
			.val = 0x2,
		},
		.fs_avl[3] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL8_ADDR,
				.mask = GENMASK(1, 0),
			},
			.gain = ST_LSM6DSVX_ACC_FS_16G_GAIN,
			.val = 0x3,
		},
	},
	[ST_LSM6DSVX_ID_GYRO] = {
		.size = 6,
		.fs_avl[0] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL6_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSVX_GYRO_FS_125_GAIN,
			.val = 0x0,
		},
		.fs_avl[1] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL6_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSVX_GYRO_FS_250_GAIN,
			.val = 0x1,
		},
		.fs_avl[2] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL6_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSVX_GYRO_FS_500_GAIN,
			.val = 0x2,
		},
		.fs_avl[3] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL6_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSVX_GYRO_FS_1000_GAIN,
			.val = 0x3,
		},
		.fs_avl[4] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL6_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSVX_GYRO_FS_2000_GAIN,
			.val = 0x4,
		},
		.fs_avl[5] = {
			.reg = {
				.addr = ST_LSM6DSVX_REG_CTRL6_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSVX_GYRO_FS_4000_GAIN,
			.val = 0x6,
		},
	}
};

static const struct iio_mount_matrix *
st_lsm6dsvx_get_mount_matrix(const struct iio_dev *iio_dev,
			     const struct iio_chan_spec *ch)
{
	struct st_lsm6dsvx_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dsvx_hw *hw = sensor->hw;

	return &hw->orientation;
}

static const struct iio_chan_spec_ext_info st_lsm6dsvx_chan_spec_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, st_lsm6dsvx_get_mount_matrix),
	{ }
};

static const struct iio_chan_spec st_lsm6dsvx_acc_channels[] = {
	ST_LSM6DSVX_DATA_CHANNEL(IIO_ACCEL,
				 ST_LSM6DSVX_REG_OUTX_L_A_ADDR,
				 1, IIO_MOD_X, 0, 16, 16, 's',
				 st_lsm6dsvx_chan_spec_ext_info),
	ST_LSM6DSVX_DATA_CHANNEL(IIO_ACCEL,
				 ST_LSM6DSVX_REG_OUTY_L_A_ADDR,
				 1, IIO_MOD_Y, 1, 16, 16, 's',
				 st_lsm6dsvx_chan_spec_ext_info),
	ST_LSM6DSVX_DATA_CHANNEL(IIO_ACCEL,
				 ST_LSM6DSVX_REG_OUTZ_L_A_ADDR,
				 1, IIO_MOD_Z, 2, 16, 16, 's',
				 st_lsm6dsvx_chan_spec_ext_info),
	ST_LSM6DSVX_EVENT_CHANNEL(IIO_ACCEL, flush),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_lsm6dsvx_gyro_channels[] = {
	ST_LSM6DSVX_DATA_CHANNEL(IIO_ANGL_VEL,
				 ST_LSM6DSVX_REG_OUTX_L_G_ADDR,
				 1, IIO_MOD_X, 0, 16, 16, 's',
				 st_lsm6dsvx_chan_spec_ext_info),
	ST_LSM6DSVX_DATA_CHANNEL(IIO_ANGL_VEL,
				 ST_LSM6DSVX_REG_OUTY_L_G_ADDR,
				 1, IIO_MOD_Y, 1, 16, 16, 's',
				 st_lsm6dsvx_chan_spec_ext_info),
	ST_LSM6DSVX_DATA_CHANNEL(IIO_ANGL_VEL,
				 ST_LSM6DSVX_REG_OUTZ_L_G_ADDR,
				 1, IIO_MOD_Z, 2, 16, 16, 's',
				 st_lsm6dsvx_chan_spec_ext_info),
	ST_LSM6DSVX_EVENT_CHANNEL(IIO_ANGL_VEL, flush),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static int st_lsm6dsvx_check_whoami(struct st_lsm6dsvx_hw *hw, int id)
{
	int data, err, i;

	for (i = 0; i < ARRAY_SIZE(st_lsm6dsvx_sensor_settings); i++) {
		if (st_lsm6dsvx_sensor_settings[i].id.name &&
		    st_lsm6dsvx_sensor_settings[i].id.hw_id == id)
			break;
	}

	if (i == ARRAY_SIZE(st_lsm6dsvx_sensor_settings)) {
		dev_err(hw->dev, "unsupported hw id [%02x]\n", id);

		return -ENODEV;
	}

	err = regmap_read(hw->regmap, ST_LSM6DSVX_REG_WHOAMI_ADDR, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != ST_LSM6DSVX_WHOAMI_VAL) {
		dev_err(hw->dev, "unsupported whoami [%02x]\n", data);
		return -ENODEV;
	}

	hw->settings = &st_lsm6dsvx_sensor_settings[i];

	return 0;
}

static int st_lsm6dsvx_get_odr_calibration(struct st_lsm6dsvx_hw *hw)
{
	s64 odr_calib;
	int data;
	int err;

	err = regmap_read(hw->regmap,
			  ST_LSM6DSVX_REG_INTERNAL_FREQ_FINE, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read %d register\n",
			ST_LSM6DSVX_REG_INTERNAL_FREQ_FINE);
		return err;
	}

	odr_calib = (data * 37500) / 1000;
	hw->ts_delta_ns = ST_LSM6DSVX_TS_DELTA_NS - odr_calib;

	dev_info(hw->dev, "Freq Fine %lld (ts %lld)\n",
		 odr_calib, hw->ts_delta_ns);

	return 0;
}

static int st_lsm6dsvx_set_full_scale(struct st_lsm6dsvx_sensor *sensor,
				      u32 gain)
{
	enum st_lsm6dsvx_sensor_id id = sensor->id;
	int i, err;
	u8 val;

	for (i = 0; i < st_lsm6dsvx_fs_table[id].size; i++)
		if (st_lsm6dsvx_fs_table[id].fs_avl[i].gain == gain)
			break;

	if (i == st_lsm6dsvx_fs_table[id].size)
		return -EINVAL;

	val = st_lsm6dsvx_fs_table[id].fs_avl[i].val;
	err = st_lsm6dsvx_write_with_mask(sensor->hw,
			    st_lsm6dsvx_fs_table[id].fs_avl[i].reg.addr,
			    st_lsm6dsvx_fs_table[id].fs_avl[i].reg.mask,
			    val);
	if (err < 0)
		return err;

	sensor->gain = gain;

	return 0;
}

static int st_lsm6dsvx_get_odr_val(enum st_lsm6dsvx_sensor_id id,
				   int odr, int uodr, int *podr,
				   int *puodr, u8 *val)
{
	int required_odr = ST_LSM6DSVX_ODR_EXPAND(odr, uodr);
	int sensor_odr;
	int i;

	for (i = 0; i < st_lsm6dsvx_odr_table[id].size; i++) {
		sensor_odr = ST_LSM6DSVX_ODR_EXPAND(
					st_lsm6dsvx_odr_table[id].odr_avl[i].hz,
					st_lsm6dsvx_odr_table[id].odr_avl[i].uhz);
		if (sensor_odr >= required_odr)
			break;
	}

	if (i == st_lsm6dsvx_odr_table[id].size)
		return -EINVAL;

	*val = st_lsm6dsvx_odr_table[id].odr_avl[i].val;

	if (podr && puodr) {
		*podr = st_lsm6dsvx_odr_table[id].odr_avl[i].hz;
		*puodr = st_lsm6dsvx_odr_table[id].odr_avl[i].uhz;
	}

	return 0;
}

int st_lsm6dsvx_get_batch_val(struct st_lsm6dsvx_sensor *sensor,
			      int odr, int uodr, u8 *val)
{
	int required_odr = ST_LSM6DSVX_ODR_EXPAND(odr, uodr);
	enum st_lsm6dsvx_sensor_id id = sensor->id;
	int sensor_odr;
	int i;

	for (i = 0; i < st_lsm6dsvx_odr_table[id].size; i++) {
		sensor_odr = ST_LSM6DSVX_ODR_EXPAND(
				st_lsm6dsvx_odr_table[id].odr_avl[i].hz,
				st_lsm6dsvx_odr_table[id].odr_avl[i].uhz);
		if (sensor_odr >= required_odr)
			break;
	}

	if (i == st_lsm6dsvx_odr_table[id].size)
		return -EINVAL;

	*val = st_lsm6dsvx_odr_table[id].odr_avl[i].batch_val;

	return 0;
}

static u16
st_lsm6dsvx_check_odr_dependency(struct st_lsm6dsvx_hw *hw, int odr,
				 int uodr,
				 enum st_lsm6dsvx_sensor_id ref_id)
{
	struct st_lsm6dsvx_sensor *ref = iio_priv(hw->iio_devs[ref_id]);
	bool enable = ST_LSM6DSVX_ODR_EXPAND(odr, uodr) > 0;
	u16 ret;

	if (enable) {
		/* uodr not used */
		if (hw->enable_mask & BIT(ref_id))
			ret = max_t(int, ref->odr, odr);
		else
			ret = odr;
	} else {
		ret = (hw->enable_mask & BIT(ref_id)) ? ref->odr : 0;
	}

	return ret;
}

static int st_lsm6dsvx_set_odr(struct st_lsm6dsvx_sensor *sensor,
			       int req_odr, int req_uodr)
{
	enum st_lsm6dsvx_sensor_id id = sensor->id;
	struct st_lsm6dsvx_hw *hw = sensor->hw;
	u8 val = 0;
	int err;

	switch (id) {
	case ST_LSM6DSVX_ID_QVAR:
	case ST_LSM6DSVX_ID_EXT0:
	case ST_LSM6DSVX_ID_EXT1:
	case ST_LSM6DSVX_ID_ACC: {
		int odr;
		int i;

		id = ST_LSM6DSVX_ID_ACC;
		for (i = ST_LSM6DSVX_ID_ACC;
		     i < ST_LSM6DSVX_ID_MAX; i++) {
			if (!hw->iio_devs[i])
				continue;

			if (i == sensor->id)
				continue;

			/* req_uodr not used */
			odr = st_lsm6dsvx_check_odr_dependency(hw,
							      req_odr,
							      req_uodr,
							      i);
			if (odr != req_odr) {
				/* device already configured */
				return 0;
			}
		}
		break;
	}
	default:
		break;
	}

	if (ST_LSM6DSVX_ODR_EXPAND(req_odr, req_uodr) > 0) {
		err = st_lsm6dsvx_get_odr_val(id, req_odr, req_uodr,
					      &req_odr, &req_uodr,
					      &val);
		if (err < 0)
			return err;
	}

	err = st_lsm6dsvx_write_with_mask(hw,
				     st_lsm6dsvx_odr_table[id].reg.addr,
				     st_lsm6dsvx_odr_table[id].reg.mask,
				     val);

	return err < 0 ? err : 0;
}

int st_lsm6dsvx_sensor_set_enable(struct st_lsm6dsvx_sensor *sensor,
				  bool enable)
{
	int uodr = enable ? sensor->uodr : 0;
	int odr = enable ? sensor->odr : 0;
	int err;

	err = st_lsm6dsvx_set_odr(sensor, odr, uodr);
	if (err < 0)
		return err;

	if (enable)
		sensor->hw->enable_mask |= BIT(sensor->id);
	else
		sensor->hw->enable_mask &= ~BIT(sensor->id);

	return 0;
}

static int st_lsm6dsvx_read_oneshot(struct st_lsm6dsvx_sensor *sensor,
				    u8 addr, int *val)
{
	int err, delay;
	__le16 data;

	err = st_lsm6dsvx_sensor_set_enable(sensor, true);
	if (err < 0)
		return err;

	delay = 1000000 / sensor->odr;
	usleep_range(delay, 2 * delay);

	err = st_lsm6dsvx_read_locked(sensor->hw, addr, (u8 *)&data,
				      sizeof(data));
	if (err < 0)
		return err;

	st_lsm6dsvx_sensor_set_enable(sensor, false);

	*val = (s16)le16_to_cpu(data);

	return IIO_VAL_INT;
}

static int st_lsm6dsvx_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *ch,
				int *val, int *val2, long mask)
{
	struct st_lsm6dsvx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			return ret;

		ret = st_lsm6dsvx_read_oneshot(sensor, ch->address,
					       val);
		iio_device_release_direct_mode(iio_dev);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = (int)sensor->odr;
		*val2 = (int)sensor->uodr;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sensor->gain;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int st_lsm6dsvx_write_raw(struct iio_dev *iio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct st_lsm6dsvx_sensor *sensor = iio_priv(iio_dev);
	int err;

	mutex_lock(&iio_dev->mlock);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = st_lsm6dsvx_set_full_scale(sensor, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		int todr, tuodr;
		u8 data;

		err = st_lsm6dsvx_get_odr_val(sensor->id, val, val2,
					      &todr, &tuodr, &data);
		if (!err) {
			sensor->odr = val;
			sensor->uodr = tuodr;

			/*
			 * VTS test testSamplingRateHotSwitchOperation
			 * not toggle the enable status of sensor after
			 * changing the ODR -> force it
			 */
			if (sensor->hw->enable_mask & BIT(sensor->id)) {
				switch (sensor->id) {
				case ST_LSM6DSVX_ID_GYRO:
				case ST_LSM6DSVX_ID_ACC:
					err = st_lsm6dsvx_set_odr(sensor,
							  sensor->odr,
							  sensor->uodr);
					if (err < 0)
						break;

					err = st_lsm6dsvx_update_batching(iio_dev, 1);
				default:
					break;
				}
			}
		}
		break;
	}
	default:
		err = -EINVAL;
		break;
	}

	mutex_unlock(&iio_dev->mlock);

	return err < 0 ? err : 0;
}

static ssize_t
st_lsm6dsvx_sysfs_sampling_frequency_avail(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct st_lsm6dsvx_sensor *sensor = iio_priv(dev_get_drvdata(dev));
	enum st_lsm6dsvx_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_lsm6dsvx_odr_table[id].size; i++) {
		if (!st_lsm6dsvx_odr_table[id].odr_avl[i].hz)
			continue;

		len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%06d ",
			      st_lsm6dsvx_odr_table[id].odr_avl[i].hz,
			      st_lsm6dsvx_odr_table[id].odr_avl[i].uhz);
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t
st_lsm6dsvx_sysfs_scale_avail(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct st_lsm6dsvx_sensor *sensor = iio_priv(dev_get_drvdata(dev));
	enum st_lsm6dsvx_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_lsm6dsvx_fs_table[id].size; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
				 st_lsm6dsvx_fs_table[id].fs_avl[i].gain);
	buf[len - 1] = '\n';

	return len;
}

static __maybe_unused int st_lsm6dsvx_reg_access(struct iio_dev *iio_dev,
				 unsigned int reg, unsigned int writeval,
				 unsigned int *readval)
{
	struct st_lsm6dsvx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(iio_dev);
	if (ret)
		return ret;

	if (readval == NULL)
		ret = regmap_write(sensor->hw->regmap, reg, writeval);
	else
		ret = regmap_read(sensor->hw->regmap, reg, readval);

	iio_device_release_direct_mode(iio_dev);

	return (ret < 0) ? ret : 0;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_lsm6dsvx_sysfs_sampling_frequency_avail);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_lsm6dsvx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_anglvel_scale_available, 0444,
		       st_lsm6dsvx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_lsm6dsvx_get_max_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, 0200, NULL,
		       st_lsm6dsvx_flush_fifo, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644,
		       st_lsm6dsvx_get_watermark,
		       st_lsm6dsvx_set_watermark, 0);

static struct attribute *st_lsm6dsvx_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsvx_acc_attribute_group = {
	.attrs = st_lsm6dsvx_acc_attributes,
};

static const struct iio_info st_lsm6dsvx_acc_info = {
	.attrs = &st_lsm6dsvx_acc_attribute_group,
	.read_raw = st_lsm6dsvx_read_raw,
	.write_raw = st_lsm6dsvx_write_raw,
	.debugfs_reg_access = st_lsm6dsvx_reg_access,
};

static struct attribute *st_lsm6dsvx_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsvx_gyro_attribute_group = {
	.attrs = st_lsm6dsvx_gyro_attributes,
};

static const struct iio_info st_lsm6dsvx_gyro_info = {
	.attrs = &st_lsm6dsvx_gyro_attribute_group,
	.read_raw = st_lsm6dsvx_read_raw,
	.write_raw = st_lsm6dsvx_write_raw,
};

static const unsigned long st_lsm6dsvx_available_scan_masks[] = {
	GENMASK(2, 0), 0x0
};

static int st_lsm6dsvx_of_get_pin(struct st_lsm6dsvx_hw *hw, int *pin)
{
	struct device_node *np = hw->dev->of_node;

	if (!np)
		return -EINVAL;

	return of_property_read_u32(np, "st,int-pin", pin);
}

static int st_lsm6dsvx_get_int_reg(struct st_lsm6dsvx_hw *hw,
				   u8 *drdy_reg, u8 *ef_irq_reg)
{
	int err = 0, int_pin;

	if (st_lsm6dsvx_of_get_pin(hw, &int_pin) < 0) {
		struct st_sensors_platform_data *pdata;
		struct device *dev = hw->dev;

		pdata = (struct st_sensors_platform_data *)dev->platform_data;
		int_pin = pdata ? pdata->drdy_int_pin : 1;
	}

	switch (int_pin) {
	case 1:
		*drdy_reg = ST_LSM6DSVX_REG_INT1_CTRL_ADDR;
		break;
	case 2:
		*drdy_reg = ST_LSM6DSVX_REG_INT2_CTRL_ADDR;
		break;
	default:
		dev_err(hw->dev, "unsupported interrupt pin\n");
		err = -EINVAL;
		break;
	}

	return err;
}

static int st_lsm6dsvx_reset_device(struct st_lsm6dsvx_hw *hw)
{
	int err;

	/* sw reset */
	err = st_lsm6dsvx_write_with_mask(hw,
					  ST_LSM6DSVX_REG_CTRL3_ADDR,
					  ST_LSM6DSVX_SW_RESET_MASK,
					  1);
	if (err < 0)
		return err;

	msleep(10);

	/* boot */
	err = st_lsm6dsvx_write_with_mask(hw,
					  ST_LSM6DSVX_REG_CTRL3_ADDR,
					  ST_LSM6DSVX_BOOT_MASK, 1);

	msleep(50);

	return err;
}

static int st_lsm6dsvx_init_device(struct st_lsm6dsvx_hw *hw)
{
	u8 drdy_reg, ef_irq_reg;
	int err;

	/* latch interrupts */
	err = st_lsm6dsvx_write_with_mask(hw,
					  ST_LSM6DSVX_REG_TAP_CFG0_ADDR,
					  ST_LSM6DSVX_LIR_MASK, 1);
	if (err < 0)
		return err;

	/* enable Block Data Update */
	err = st_lsm6dsvx_write_with_mask(hw,
					  ST_LSM6DSVX_REG_CTRL3_ADDR,
					  ST_LSM6DSVX_BDU_MASK, 1);
	if (err < 0)
		return err;

	/* init timestamp engine */
	err = st_lsm6dsvx_write_with_mask(hw,
				  ST_LSM6DSVX_REG_FUNCTIONS_ENABLE_ADDR,
				  ST_LSM6DSVX_TIMESTAMP_EN_MASK, 1);
	if (err < 0)
		return err;

	err = st_lsm6dsvx_get_int_reg(hw, &drdy_reg, &ef_irq_reg);
	if (err < 0)
		return err;

	/* enable DRDY MASK for filters settling time */
	err = st_lsm6dsvx_write_with_mask(hw,
					  ST_LSM6DSVX_REG_CTRL4_ADDR,
					  ST_LSM6DSVX_DRDY_MASK, 1);
	if (err < 0)
		return err;

	/* enable FIFO watermak interrupt */
	return st_lsm6dsvx_write_with_mask(hw, drdy_reg,
					   ST_LSM6DSVX_INT_FIFO_TH_MASK,
					   1);
}

static struct iio_dev *
st_lsm6dsvx_alloc_iiodev(struct st_lsm6dsvx_hw *hw,
			 enum st_lsm6dsvx_sensor_id id)
{
	struct st_lsm6dsvx_sensor *sensor;
	struct iio_dev *iio_dev;

	iio_dev = devm_iio_device_alloc(hw->dev, sizeof(*sensor));
	if (!iio_dev)
		return NULL;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = hw->dev;

	sensor = iio_priv(iio_dev);
	sensor->id = id;
	sensor->hw = hw;
	sensor->watermark = 1;

	sensor->decimator = 0;
	sensor->dec_counter = 0;

	switch (id) {
	case ST_LSM6DSVX_ID_ACC:
		iio_dev->channels = st_lsm6dsvx_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dsvx_acc_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			 "%s_accel", hw->settings->id.name);
		iio_dev->info = &st_lsm6dsvx_acc_info;
		iio_dev->available_scan_masks = st_lsm6dsvx_available_scan_masks;

		sensor->batch_reg.addr = ST_LSM6DSVX_REG_FIFO_CTRL3_ADDR;
		sensor->batch_reg.mask = ST_LSM6DSVX_BDR_XL_MASK;
		sensor->max_watermark = ST_LSM6DSVX_MAX_FIFO_DEPTH;
		sensor->odr = st_lsm6dsvx_odr_table[id].odr_avl[1].hz;
		sensor->uodr = st_lsm6dsvx_odr_table[id].odr_avl[1].uhz;
		sensor->gain = st_lsm6dsvx_fs_table[id].fs_avl[0].gain;
		break;
	case ST_LSM6DSVX_ID_GYRO:
		iio_dev->channels = st_lsm6dsvx_gyro_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dsvx_gyro_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			 "%s_gyro", hw->settings->id.name);
		iio_dev->info = &st_lsm6dsvx_gyro_info;
		iio_dev->available_scan_masks = st_lsm6dsvx_available_scan_masks;

		sensor->batch_reg.addr = ST_LSM6DSVX_REG_FIFO_CTRL3_ADDR;
		sensor->batch_reg.mask = ST_LSM6DSVX_BDR_GY_MASK;
		sensor->max_watermark = ST_LSM6DSVX_MAX_FIFO_DEPTH;
		sensor->odr = st_lsm6dsvx_odr_table[id].odr_avl[1].hz;
		sensor->uodr = st_lsm6dsvx_odr_table[id].odr_avl[1].uhz;
		sensor->gain = st_lsm6dsvx_fs_table[id].fs_avl[1].gain;
		break;
	default:
		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

static void st_lsm6dsvx_disable_regulator_action(void *_data)
{
	struct st_lsm6dsvx_hw *hw = _data;

	regulator_disable(hw->vddio_supply);
	regulator_disable(hw->vdd_supply);
}

static int st_lsm6dsvx_power_enable(struct st_lsm6dsvx_hw *hw)
{
	int err;

	hw->vdd_supply = devm_regulator_get(hw->dev, "vdd");
	if (IS_ERR(hw->vdd_supply)) {
		if (PTR_ERR(hw->vdd_supply) != -EPROBE_DEFER)
			dev_err(hw->dev,
				"Failed to get vdd regulator %d\n",
				(int)PTR_ERR(hw->vdd_supply));

		return PTR_ERR(hw->vdd_supply);
	}

	hw->vddio_supply = devm_regulator_get(hw->dev, "vddio");
	if (IS_ERR(hw->vddio_supply)) {
		if (PTR_ERR(hw->vddio_supply) != -EPROBE_DEFER)
			dev_err(hw->dev,
				"Failed to get vddio regulator %d\n",
				(int)PTR_ERR(hw->vddio_supply));

		return PTR_ERR(hw->vddio_supply);
	}

	err = regulator_enable(hw->vdd_supply);
	if (err) {
		dev_err(hw->dev,
			"Failed to enable vdd regulator: %d\n", err);
		return err;
	}

	err = regulator_enable(hw->vddio_supply);
	if (err) {
		regulator_disable(hw->vdd_supply);
		return err;
	}

	err = devm_add_action_or_reset(hw->dev,
				       st_lsm6dsvx_disable_regulator_action,
				       hw);
	if (err) {
		dev_err(hw->dev,
			"Failed to setup regulator cleanup action %d\n",
			err);
		return err;
	}

	return 0;
}

int st_lsm6dsvx_probe(struct device *dev, int irq, int hw_id,
		      struct regmap *regmap)
{
	struct st_lsm6dsvx_hw *hw;
	int i, err;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	dev_set_drvdata(dev, (void *)hw);

	mutex_init(&hw->lock);
	mutex_init(&hw->fifo_lock);
	mutex_init(&hw->page_lock);

	hw->dev = dev;
	hw->irq = irq;
	hw->regmap = regmap;

	err = st_lsm6dsvx_power_enable(hw);
	if (err != 0)
		return err;

	/* select register bank zero */
	err = st_lsm6dsvx_set_page_access(hw,
				  ST_LSM6DSVX_EMB_FUNC_REG_ACCESS_MASK |
				  ST_LSM6DSVX_SHUB_REG_ACCESS_MASK,
				  0);
	if (err < 0)
		return err;

	err = st_lsm6dsvx_check_whoami(hw, hw_id);
	if (err < 0)
		return err;

	err = st_lsm6dsvx_get_odr_calibration(hw);
	if (err < 0)
		return err;

	err = st_lsm6dsvx_reset_device(hw);
	if (err < 0)
		return err;

	err = st_lsm6dsvx_init_device(hw);
	if (err < 0)
		return err;

#if KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE
	err = iio_read_mount_matrix(dev, &hw->orientation);
#elif KERNEL_VERSION(5, 2, 0) <= LINUX_VERSION_CODE
	err = iio_read_mount_matrix(dev, "mount-matrix", &hw->orientation);
#else /* LINUX_VERSION_CODE */
	err = of_iio_read_mount_matrix(dev, "mount-matrix", &hw->orientation);
#endif /* LINUX_VERSION_CODE */

	if (err) {
		dev_err(dev, "Failed to retrieve mounting matrix %d\n", err);
		return err;
	}

	for (i = 0; i < ARRAY_SIZE(st_lsm6dsvx_main_sensor_list); i++) {
		enum st_lsm6dsvx_sensor_id id = st_lsm6dsvx_main_sensor_list[i];

		hw->iio_devs[id] = st_lsm6dsvx_alloc_iiodev(hw, id);
		if (!hw->iio_devs[id])
			return -ENOMEM;
	}

	err = st_lsm6dsvx_shub_probe(hw);
	if (err < 0)
		return err;

	if (hw->settings->st_qvar_probe) {
		err = st_lsm6dsvx_qvar_probe(hw);
		if (err)
			return err;
	}

	if (hw->irq > 0) {
		err = st_lsm6dsvx_buffers_setup(hw);
		if (err < 0)
			return err;
	}

	for (i = 0; i < ST_LSM6DSVX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

#if defined(CONFIG_IIO_ST_LSM6DSVX_MAY_WAKEUP)
	if (hw->enable_mask & ST_LSM6DSVX_WAKE_UP_SENSORS) {
		if (device_may_wakeup(dev))
			enable_irq_wake(hw->irq);
	}
#endif /* CONFIG_IIO_ST_LSM6DSVX_MAY_WAKEUP */

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsvx_probe);

int st_lsm6dsvx_remove(struct device *dev)
{
	struct st_lsm6dsvx_hw *hw = dev_get_drvdata(dev);
	int ret = 0;

	if (hw->settings->st_qvar_probe)
		ret = st_lsm6dsvx_qvar_remove(dev);

	return ret;
}
EXPORT_SYMBOL(st_lsm6dsvx_remove);

static int __maybe_unused st_lsm6dsvx_suspend(struct device *dev)
{
	struct st_lsm6dsvx_hw *hw = dev_get_drvdata(dev);
	struct st_lsm6dsvx_sensor *sensor;
	int i, err = 0;

	for (i = 0; i < ST_LSM6DSVX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		err = st_lsm6dsvx_set_odr(sensor, 0, 0);
		if (err < 0)
			return err;
	}

	if (st_lsm6dsvx_is_fifo_enabled(hw))
		err = st_lsm6dsvx_suspend_fifo(hw);

#ifdef CONFIG_IIO_ST_LSM6DSVX_MAY_WAKEUP
	if (device_may_wakeup(dev))
		enable_irq_wake(hw->irq);
#endif /* CONFIG_IIO_ST_LSM6DSVX_MAY_WAKEUP */

	dev_info(dev, "Suspending device\n");

	return err < 0 ? err : 0;
}

static int __maybe_unused st_lsm6dsvx_resume(struct device *dev)
{
	struct st_lsm6dsvx_hw *hw = dev_get_drvdata(dev);
	struct st_lsm6dsvx_sensor *sensor;
	int i, err = 0;

	dev_info(dev, "Resuming device\n");

#ifdef CONFIG_IIO_ST_LSM6DSVX_MAY_WAKEUP
	if (device_may_wakeup(dev))
		disable_irq_wake(hw->irq);
#endif /* CONFIG_IIO_ST_LSM6DSVX_MAY_WAKEUP */

	for (i = 0; i < ST_LSM6DSVX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		err = st_lsm6dsvx_set_odr(sensor, sensor->odr,
					  sensor->uodr);
		if (err < 0)
			return err;
	}

	if (st_lsm6dsvx_is_fifo_enabled(hw))
		err = st_lsm6dsvx_set_fifo_mode(hw, ST_LSM6DSVX_FIFO_CONT);

	return err < 0 ? err : 0;
}

const struct dev_pm_ops st_lsm6dsvx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lsm6dsvx_suspend, st_lsm6dsvx_resume)
};
EXPORT_SYMBOL(st_lsm6dsvx_pm_ops);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dsvx driver");
MODULE_LICENSE("GPL v2");