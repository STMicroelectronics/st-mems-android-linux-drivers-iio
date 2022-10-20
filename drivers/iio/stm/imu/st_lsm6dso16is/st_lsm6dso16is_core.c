// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lsm6dso16is sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2022 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>

#include "st_lsm6dso16is.h"

static const struct st_lsm6dso16is_odr_table_entry st_lsm6dso16is_odr_table[] = {
	[ST_LSM6DSO16IS_ID_ACC] = {
		.size = 7,
		.reg = {
			.addr = ST_LSM6DSO16IS_REG_CTRL1_XL_ADDR,
			.mask = ST_LSM6DSO16IS_ODR_XL_MASK,
		},
		.odr_avl[0] = {  12500,  0x01 },
		.odr_avl[1] = {  26000,  0x02 },
		.odr_avl[2] = {  52000,  0x03 },
		.odr_avl[3] = { 104000,  0x04 },
		.odr_avl[4] = { 208000,  0x05 },
		.odr_avl[5] = { 416000,  0x06 },
		.odr_avl[6] = { 833000,  0x07 },
	},
	[ST_LSM6DSO16IS_ID_GYRO] = {
		.size = 7,
		.reg = {
			.addr = ST_LSM6DSO16IS_REG_CTRL2_G_ADDR,
			.mask = ST_LSM6DSO16IS_ODR_G_MASK,
		},
		.odr_avl[0] = {  12500,  0x01 },
		.odr_avl[1] = {  26000,  0x02 },
		.odr_avl[2] = {  52000,  0x03 },
		.odr_avl[3] = { 104000,  0x04 },
		.odr_avl[4] = { 208000,  0x05 },
		.odr_avl[5] = { 416000,  0x06 },
		.odr_avl[6] = { 833000,  0x07 },
	},
	[ST_LSM6DSO16IS_ID_TEMP] = {
		.size = 2,
		.odr_avl[0] = { 12500,  0x01 },
		.odr_avl[1] = { 52000,  0x03 },
	},
};

static struct st_lsm6dso16is_fs_table_entry st_lsm6dso16is_fs_table[] = {
	[ST_LSM6DSO16IS_ID_ACC] = {
		.fs_len = 4,
		.reg = {
			.addr = ST_LSM6DSO16IS_REG_CTRL1_XL_ADDR,
			.mask = ST_LSM6DSO16IS_FS_XL_MASK,
		},
		.fs_avl[0] = {  IIO_G_TO_M_S_2(61000), 0x0 },
		.fs_avl[1] = { IIO_G_TO_M_S_2(122000), 0x2 },
		.fs_avl[2] = { IIO_G_TO_M_S_2(244000), 0x3 },
		.fs_avl[3] = { IIO_G_TO_M_S_2(488000), 0x1 },
	},
	[ST_LSM6DSO16IS_ID_GYRO] = {
		.fs_len = 4,
		.reg = {
			.addr = ST_LSM6DSO16IS_REG_CTRL2_G_ADDR,
			.mask = ST_LSM6DSO16IS_FS_G_MASK,
		},
		.fs_avl[0] = {  IIO_DEGREE_TO_RAD(8750000), 0x0 },
		.fs_avl[1] = { IIO_DEGREE_TO_RAD(17500000), 0x1 },
		.fs_avl[2] = { IIO_DEGREE_TO_RAD(35000000), 0x2 },
		.fs_avl[3] = { IIO_DEGREE_TO_RAD(70000000), 0x3 },
	},
	[ST_LSM6DSO16IS_ID_TEMP] = {
		.fs_len = 1,
		.fs_avl[0] = {  ST_LSM6DSO16IS_TEMP_GAIN, 0x0 },
	},
};

static const struct iio_mount_matrix *
st_lsm6dso16is_get_mount_matrix(const struct iio_dev *iio_dev,
			     const struct iio_chan_spec *ch)
{
	struct st_lsm6dso16is_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dso16is_hw *hw = sensor->hw;

	return &hw->orientation;
}

static const struct iio_chan_spec_ext_info st_lsm6dso16is_chan_spec_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, st_lsm6dso16is_get_mount_matrix),
	{ }
};

static const struct iio_chan_spec st_lsm6dso16is_acc_channels[] = {
	ST_LSM6DSO16IS_DATA_CHANNEL(IIO_ACCEL, ST_LSM6DSO16IS_REG_OUTX_L_A_ADDR,
				1, IIO_MOD_X, 0, 16, 16, 's',
				st_lsm6dso16is_chan_spec_ext_info),
	ST_LSM6DSO16IS_DATA_CHANNEL(IIO_ACCEL, ST_LSM6DSO16IS_REG_OUTY_L_A_ADDR,
				1, IIO_MOD_Y, 1, 16, 16, 's',
				st_lsm6dso16is_chan_spec_ext_info),
	ST_LSM6DSO16IS_DATA_CHANNEL(IIO_ACCEL, ST_LSM6DSO16IS_REG_OUTZ_L_A_ADDR,
				1, IIO_MOD_Z, 2, 16, 16, 's',
				st_lsm6dso16is_chan_spec_ext_info),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_lsm6dso16is_gyro_channels[] = {
	ST_LSM6DSO16IS_DATA_CHANNEL(IIO_ANGL_VEL, ST_LSM6DSO16IS_REG_OUTX_L_G_ADDR,
				1, IIO_MOD_X, 0, 16, 16, 's',
				st_lsm6dso16is_chan_spec_ext_info),
	ST_LSM6DSO16IS_DATA_CHANNEL(IIO_ANGL_VEL, ST_LSM6DSO16IS_REG_OUTY_L_G_ADDR,
				1, IIO_MOD_Y, 1, 16, 16, 's',
				st_lsm6dso16is_chan_spec_ext_info),
	ST_LSM6DSO16IS_DATA_CHANNEL(IIO_ANGL_VEL, ST_LSM6DSO16IS_REG_OUTZ_L_G_ADDR,
				1, IIO_MOD_Z, 2, 16, 16, 's',
				st_lsm6dso16is_chan_spec_ext_info),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_lsm6dso16is_temp_channels[] = {
	{
		.type = IIO_TEMP,
		.address = ST_LSM6DSO16IS_REG_OUT_TEMP_L_ADDR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_OFFSET)
				| BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		}
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static __maybe_unused int st_lsm6dso16is_reg_access(struct iio_dev *iio_dev,
						    unsigned int reg,
						    unsigned int writeval,
						    unsigned int *readval)
{
	struct st_lsm6dso16is_sensor *sensor = iio_priv(iio_dev);
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

/**
 * st_lsm6dso16is_check_whoami - Detect device HW ID
 *
 * Check the value of the device HW ID if valid
 *
 * @param  hw: ST IMU MEMS hw instance.
 * @return  0 if OK, negative value for ERROR
 */
static int st_lsm6dso16is_check_whoami(struct st_lsm6dso16is_hw *hw)
{
	int err, data;

	err = regmap_read(hw->regmap, ST_LSM6DSO16IS_REG_WHOAMI_ADDR, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");

		return err;
	}

	if (data != ST_LSM6DSO16IS_WHOAMI_VAL) {
		dev_err(hw->dev, "unsupported whoami [%02x]\n", data);

		return -ENODEV;
	}

	return 0;
}

/**
 * st_lsm6dso16is_set_full_scale - Set sensor full scale
 *
 * Check the value of requested gain, apply it if supported.
 * NOTE: support also sensor with only one FS available (Temp gain is fixed).
 *
 * @param  sensor: ST IMU MEMS sensor instance.
 * @param  gain: Sensor gain.
 * @return  0 if OK, negative value for ERROR
 */
static int st_lsm6dso16is_set_full_scale(struct st_lsm6dso16is_sensor *sensor,
					 u32 gain)
{
	enum st_lsm6dso16is_sensor_id id = sensor->id;
	struct st_lsm6dso16is_hw *hw = sensor->hw;
	int i, err;
	u8 val;

	for (i = 0; i < st_lsm6dso16is_fs_table[id].fs_len; i++)
		if (st_lsm6dso16is_fs_table[id].fs_avl[i].gain == gain)
			break;

	if (i == st_lsm6dso16is_fs_table[id].fs_len)
		return -EINVAL;

	val = st_lsm6dso16is_fs_table[id].fs_avl[i].val;
	err = regmap_update_bits(hw->regmap,
				 st_lsm6dso16is_fs_table[id].reg.addr,
				 st_lsm6dso16is_fs_table[id].reg.mask,
				 ST_LSM6DSO16IS_SHIFT_VAL(val,
					st_lsm6dso16is_fs_table[id].reg.mask));
	if (err < 0)
		return err;

	sensor->gain = gain;

	return 0;
}

static int st_lsm6dso16is_get_odr_val(enum st_lsm6dso16is_sensor_id id, u32 mhz,
				      u32 *val, u32 *modr)
{
	u32 sensor_odr;
	int i;

	/* this avoid entry  0mHz to ODR table */
	if (mhz == 0) {
		*val = 0;
		*modr = 0;

		return 0;
	}

	for (i = 0; i < st_lsm6dso16is_odr_table[id].size; i++) {
		sensor_odr = st_lsm6dso16is_odr_table[id].odr_avl[i].mhz;
		if (sensor_odr >= mhz) {
			*val = st_lsm6dso16is_odr_table[id].odr_avl[i].val;
			*modr = sensor_odr;

			return 0;
		}
	}

	return -EINVAL;
}


static u32
st_lsm6dso16is_check_odr_dependency(struct st_lsm6dso16is_hw *hw, u32 mhz,
				    enum st_lsm6dso16is_sensor_id ref_id)
{
	struct st_lsm6dso16is_sensor *ref = iio_priv(hw->iio_devs[ref_id]);
	u32 ret = mhz;

	if (mhz > 0) {
		if (hw->enable_mask & BIT(ref_id))
			ret = max_t(u32, ref->mhz, mhz);
	} else {
		ret = (hw->enable_mask & BIT(ref_id)) ? ref->mhz : 0;
	}

	return ret;
}

static int st_lsm6dso16is_set_odr(struct st_lsm6dso16is_sensor *sensor, u32 mhz)
{
	enum st_lsm6dso16is_sensor_id id = sensor->id;
	struct st_lsm6dso16is_hw *hw = sensor->hw;
	int val, err, odr, modr;

	switch (id) {
	case ST_LSM6DSO16IS_ID_TEMP:
	case ST_LSM6DSO16IS_ID_ACC: {
		int i;

		id = ST_LSM6DSO16IS_ID_ACC;
		for (i = ST_LSM6DSO16IS_ID_ACC; i < ST_LSM6DSO16IS_ID_MAX; i++) {
			if (!hw->iio_devs[i] || i == sensor->id)
				continue;

			odr = st_lsm6dso16is_check_odr_dependency(hw, mhz, i);
			if (odr != mhz)
				return 0;
		}
		break;
	}
	default:
		break;
	}

	err = st_lsm6dso16is_get_odr_val(id, odr, &val, &modr);
	if (err < 0)
		return err;

	return st_lsm6dso16is_update_bits_locked(hw,
					st_lsm6dso16is_odr_table[id].reg.addr,
					st_lsm6dso16is_odr_table[id].reg.mask,
					val);
}

static int
st_lsm6dso16is_sensor_set_enable(struct st_lsm6dso16is_sensor *sensor,
				 bool enable)
{
	int mhz = enable ? sensor->mhz : 0;
	int err;

	err = st_lsm6dso16is_set_odr(sensor, mhz);
	if (err < 0)
		return err;

	if (enable)
		sensor->hw->enable_mask |= BIT(sensor->id);
	else
		sensor->hw->enable_mask &= ~BIT(sensor->id);

	return 0;
}

static int st_lsm6dso16is_read_oneshot(struct st_lsm6dso16is_sensor *sensor,
				       u8 addr, int *val)
{
	struct st_lsm6dso16is_hw *hw = sensor->hw;
	int err, delay;
	__le16 data;

	/*
	 * adjust delay for data valid because of turn-on time:
	 *  - Acc, Power-down -> High-performance discard 1 sample
	 *  - Gyro, Power-down -> High-performance wait 70 ms + f(ODR)
	 *  - Temp, 1 ODR
	 * NOTE: we use conversion 1100000000 to also take into account the
	 *       internal oscillator tolerance of 10%
	 */
	switch (sensor->id) {
	case ST_LSM6DSO16IS_ID_GYRO: {
		int n = 3;

		if (sensor->mhz >
		    st_lsm6dso16is_odr_table[sensor->id].odr_avl[0].mhz)
			n++;

		delay = 70000 + n * (1100000000 / sensor->mhz);
		}
		break;
	case ST_LSM6DSO16IS_ID_ACC:
		delay = 2 * (1100000000 / sensor->mhz);
		break;
	case ST_LSM6DSO16IS_ID_TEMP:
		delay = 1100000000 / sensor->mhz;
		break;
	default:
		return -EINVAL;
	}

	err = st_lsm6dso16is_sensor_set_enable(sensor, true);
	if (err < 0)
		return err;

	usleep_range(delay, delay + (delay >> 1));
	err = st_lsm6dso16is_read_locked(hw, addr, &data, sizeof(data));
	st_lsm6dso16is_sensor_set_enable(sensor, false);
	if (err < 0)
		return err;

	*val = (s16)le16_to_cpu(data);

	return IIO_VAL_INT;
}

static int st_lsm6dso16is_write_raw_get_fmt(struct iio_dev *indio_dev,
					    struct iio_chan_spec const *chan,
					    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
		case IIO_ACCEL:
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int st_lsm6dso16is_read_raw(struct iio_dev *iio_dev,
				   struct iio_chan_spec const *ch,
				   int *val, int *val2, long mask)
{
	struct st_lsm6dso16is_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			return ret;

		ret = st_lsm6dso16is_read_oneshot(sensor, ch->address, val);
		iio_device_release_direct_mode(iio_dev);
		break;
	case IIO_CHAN_INFO_OFFSET:
		switch (ch->type) {
		case IIO_TEMP:
			*val = sensor->offset;
			ret = IIO_VAL_INT;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = sensor->mhz / 1000;
		*val2 = (sensor->mhz % 1000) * 1000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (ch->type) {
		case IIO_TEMP:
			*val = 1000;
			*val2 = ST_LSM6DSO16IS_TEMP_GAIN;
			ret = IIO_VAL_FRACTIONAL;
			break;
		case IIO_ACCEL:
		case IIO_ANGL_VEL:
			*val = 0;
			*val2 = sensor->gain;
			ret = IIO_VAL_INT_PLUS_NANO;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int st_lsm6dso16is_write_raw(struct iio_dev *iio_dev,
				    struct iio_chan_spec const *chan,
				    int val, int val2, long mask)
{
	struct st_lsm6dso16is_sensor *sensor = iio_priv(iio_dev);
	int err;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = st_lsm6dso16is_set_full_scale(sensor, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		u32 reg, modr;

		val = val * 1000 + val2 / 1000;

		err = st_lsm6dso16is_get_odr_val(sensor->id, val, &reg, &modr);
		if (err < 0)
			goto release;

		sensor->mhz = modr;
		break;
	}
	default:
		err = -EINVAL;
		break;
	}

release:
	iio_device_release_direct_mode(iio_dev);

	return err;
}

static ssize_t
st_lsm6dso16is_sysfs_sampling_frequency_avail(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct st_lsm6dso16is_sensor *sensor = iio_priv(dev_get_drvdata(dev));
	const struct st_lsm6dso16is_odr_table_entry *odr_table;
	enum st_lsm6dso16is_sensor_id id = sensor->id;
	int i, len = 0;

	odr_table = &st_lsm6dso16is_odr_table[id];
	for (i = 0; i < st_lsm6dso16is_odr_table[id].size; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%03d ",
				 odr_table->odr_avl[i].mhz / 1000,
				 odr_table->odr_avl[i].mhz % 1000);
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6dso16is_sysfs_scale_avail(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct st_lsm6dso16is_sensor *sensor = iio_priv(dev_get_drvdata(dev));
	enum st_lsm6dso16is_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_lsm6dso16is_fs_table[id].fs_len; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%09u ",
				 st_lsm6dso16is_fs_table[id].fs_avl[i].gain);
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_lsm6dso16is_sysfs_sampling_frequency_avail);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_lsm6dso16is_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_anglvel_scale_available, 0444,
		       st_lsm6dso16is_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_temp_scale_available, 0444,
		       st_lsm6dso16is_sysfs_scale_avail, NULL, 0);

static struct attribute *st_lsm6dso16is_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dso16is_acc_attribute_group = {
	.attrs = st_lsm6dso16is_acc_attributes,
};

static const struct iio_info st_lsm6dso16is_acc_info = {
	.attrs = &st_lsm6dso16is_acc_attribute_group,
	.read_raw = st_lsm6dso16is_read_raw,
	.write_raw = st_lsm6dso16is_write_raw,
	.write_raw_get_fmt = st_lsm6dso16is_write_raw_get_fmt,
	.debugfs_reg_access = st_lsm6dso16is_reg_access,
};

static struct attribute *st_lsm6dso16is_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dso16is_gyro_attribute_group = {
	.attrs = st_lsm6dso16is_gyro_attributes,
};

static const struct iio_info st_lsm6dso16is_gyro_info = {
	.attrs = &st_lsm6dso16is_gyro_attribute_group,
	.read_raw = st_lsm6dso16is_read_raw,
	.write_raw = st_lsm6dso16is_write_raw,
	.write_raw_get_fmt = st_lsm6dso16is_write_raw_get_fmt,
};

static struct attribute *st_lsm6dso16is_temp_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_temp_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dso16is_temp_attribute_group = {
	.attrs = st_lsm6dso16is_temp_attributes,
};

static const struct iio_info st_lsm6dso16is_temp_info = {
	.attrs = &st_lsm6dso16is_temp_attribute_group,
	.read_raw = st_lsm6dso16is_read_raw,
	.write_raw = st_lsm6dso16is_write_raw,
	.write_raw_get_fmt = st_lsm6dso16is_write_raw_get_fmt,
};

static int st_lsm6dso16is_reset_device(struct st_lsm6dso16is_hw *hw)
{
	int err;

	/* sw reset */
	err = regmap_update_bits(hw->regmap, ST_LSM6DSO16IS_REG_CTRL3_C_ADDR,
				 ST_LSM6DSO16IS_SW_RESET_MASK,
				 FIELD_PREP(ST_LSM6DSO16IS_SW_RESET_MASK, 1));
	if (err < 0)
		return err;

	/* software reset procedure takes a maximum of 50 µs */
	usleep_range(50, 60);

	return err;
}

static int st_lsm6dso16is_init_device(struct st_lsm6dso16is_hw *hw)
{
	/* enable Block Data Update */
	return regmap_update_bits(hw->regmap, ST_LSM6DSO16IS_REG_CTRL3_C_ADDR,
				  ST_LSM6DSO16IS_BDU_MASK,
				  FIELD_PREP(ST_LSM6DSO16IS_BDU_MASK, 1));
}

static struct iio_dev *st_lsm6dso16is_alloc_iiodev(struct st_lsm6dso16is_hw *hw,
						enum st_lsm6dso16is_sensor_id id)
{
	struct st_lsm6dso16is_sensor *sensor;
	struct iio_dev *iio_dev;

	iio_dev = devm_iio_device_alloc(hw->dev, sizeof(*sensor));
	if (!iio_dev)
		return NULL;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = hw->dev;

	sensor = iio_priv(iio_dev);
	sensor->id = id;
	sensor->hw = hw;

	switch (id) {
	case ST_LSM6DSO16IS_ID_ACC:
		iio_dev->channels = st_lsm6dso16is_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso16is_acc_channels);
		scnprintf(sensor->name, sizeof(sensor->name), "%s_accel",
			  ST_LSM6DSO16IS_DEV_NAME);
		iio_dev->info = &st_lsm6dso16is_acc_info;
		st_lsm6dso16is_set_full_scale(sensor,
				st_lsm6dso16is_fs_table[id].fs_avl[0].gain);
		sensor->offset = 0;
		sensor->mhz = st_lsm6dso16is_odr_table[id].odr_avl[1].mhz;
		break;
	case ST_LSM6DSO16IS_ID_GYRO:
		iio_dev->channels = st_lsm6dso16is_gyro_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso16is_gyro_channels);
		scnprintf(sensor->name, sizeof(sensor->name), "%s_gyro",
			  ST_LSM6DSO16IS_DEV_NAME);
		iio_dev->info = &st_lsm6dso16is_gyro_info;
		st_lsm6dso16is_set_full_scale(sensor,
				st_lsm6dso16is_fs_table[id].fs_avl[0].gain);
		sensor->offset = 0;
		sensor->mhz = st_lsm6dso16is_odr_table[id].odr_avl[1].mhz;
		break;
	case ST_LSM6DSO16IS_ID_TEMP:
		iio_dev->channels = st_lsm6dso16is_temp_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso16is_temp_channels);
		scnprintf(sensor->name, sizeof(sensor->name), "%s_temp",
			  ST_LSM6DSO16IS_DEV_NAME);
		iio_dev->info = &st_lsm6dso16is_temp_info;
		sensor->offset = ST_LSM6DSO16IS_TEMP_OFFSET;
		sensor->mhz = st_lsm6dso16is_odr_table[id].odr_avl[1].mhz;
		break;
	default:
		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

static void st_lsm6dso16is_disable_regulator_action(void *_data)
{
	struct st_lsm6dso16is_hw *hw = _data;

	regulator_disable(hw->vddio_supply);
	regulator_disable(hw->vdd_supply);
}

static int st_lsm6dso16is_power_enable(struct st_lsm6dso16is_hw *hw)
{
	int err;

	hw->vdd_supply = devm_regulator_get(hw->dev, "vdd");
	if (IS_ERR(hw->vdd_supply)) {
		if (PTR_ERR(hw->vdd_supply) != -EPROBE_DEFER)
			dev_err(hw->dev, "Failed to get vdd regulator %d\n",
				(int)PTR_ERR(hw->vdd_supply));

		return PTR_ERR(hw->vdd_supply);
	}

	hw->vddio_supply = devm_regulator_get(hw->dev, "vddio");
	if (IS_ERR(hw->vddio_supply)) {
		if (PTR_ERR(hw->vddio_supply) != -EPROBE_DEFER)
			dev_err(hw->dev, "Failed to get vddio regulator %d\n",
				(int)PTR_ERR(hw->vddio_supply));

		return PTR_ERR(hw->vddio_supply);
	}

	err = regulator_enable(hw->vdd_supply);
	if (err) {
		dev_err(hw->dev, "Failed to enable vdd regulator: %d\n", err);
		return err;
	}

	err = regulator_enable(hw->vddio_supply);
	if (err) {
		regulator_disable(hw->vdd_supply);
		return err;
	}

	err = devm_add_action_or_reset(hw->dev,
				       st_lsm6dso16is_disable_regulator_action,
				       hw);
	if (err) {
		dev_err(hw->dev,
			"Failed to setup regulator cleanup action %d\n",
			err);
		return err;
	}

	return 0;
}

/**
 * Probe device function
 * Implements [MODULE] feature for Power Management
 *
 * @param  dev: Device pointer.
 * @param  irq: I2C/SPI/I3C client irq.
 * @param  hw_id: Sensor HW id.
 * @param  regmap: Bus Transfer Function pointer.
 * @retval 0 if OK, < 0 for error
 */
int st_lsm6dso16is_probe(struct device *dev, int irq, struct regmap *regmap)
{
	struct st_lsm6dso16is_hw *hw;
	int i, err;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	dev_set_drvdata(dev, (void *)hw);

	mutex_init(&hw->page_lock);

	hw->regmap = regmap;
	hw->dev = dev;
	hw->irq = irq;

	err = st_lsm6dso16is_power_enable(hw);
	if (err != 0)
		return err;

	/*
	 * After the device is powered up, it performs a 10 ms (maximum) boot
	 * procedure to load the trimming parameters.
	 * After the boot is completed, both the accelerometer and the gyroscope
	 * are automatically configured in power-down mode.
	 */
	usleep_range(10000, 11000);

	err = regmap_write(hw->regmap,
			   ST_LSM6DSO16IS_REG_FUNC_CFG_ACCESS_ADDR, 0);
	if (err < 0)
		return err;

	err = st_lsm6dso16is_check_whoami(hw);
	if (err < 0)
		return err;

	err = st_lsm6dso16is_reset_device(hw);
	if (err < 0)
		return err;

	err = st_lsm6dso16is_init_device(hw);
	if (err < 0)
		return err;

#if KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE
	err = iio_read_mount_matrix(hw->dev, &hw->orientation);
#elif KERNEL_VERSION(5, 2, 0) <= LINUX_VERSION_CODE
	err = iio_read_mount_matrix(hw->dev, "mount-matrix", &hw->orientation);
#else /* LINUX_VERSION_CODE */
	err = of_iio_read_mount_matrix(hw->dev, "mount-matrix",
				       &hw->orientation);
#endif /* LINUX_VERSION_CODE */

	if (err) {
		dev_err(dev, "Failed to retrieve mounting matrix %d\n", err);

		return err;
	}

	/* register only data sensors */
	for (i = 0; i < ARRAY_SIZE(st_lsm6dso16is_main_sensor_list); i++) {
		enum st_lsm6dso16is_sensor_id id = st_lsm6dso16is_main_sensor_list[i];

		hw->iio_devs[id] = st_lsm6dso16is_alloc_iiodev(hw, id);
		if (!hw->iio_devs[id])
			return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(st_lsm6dso16is_main_sensor_list); i++) {
		enum st_lsm6dso16is_sensor_id id = st_lsm6dso16is_main_sensor_list[i];

		if (!hw->iio_devs[id])
			continue;

		err = devm_iio_device_register(hw->dev, hw->iio_devs[id]);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(st_lsm6dso16is_probe);

static int __maybe_unused st_lsm6dso16is_suspend(struct device *dev)
{
	struct st_lsm6dso16is_hw *hw = dev_get_drvdata(dev);
	struct st_lsm6dso16is_sensor *sensor;
	int i, err = 0;

	for (i = 0; i < ST_LSM6DSO16IS_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);
		if (!hw->iio_devs[i])
			continue;

		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		err = st_lsm6dso16is_set_odr(sensor, 0);
		if (err < 0)
			return err;
	}

	return err < 0 ? err : 0;
}

static int __maybe_unused st_lsm6dso16is_resume(struct device *dev)
{
	struct st_lsm6dso16is_hw *hw = dev_get_drvdata(dev);
	struct st_lsm6dso16is_sensor *sensor;
	int i, err = 0;

	for (i = 0; i < ST_LSM6DSO16IS_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);
		if (!hw->iio_devs[i])
			continue;

		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		err = st_lsm6dso16is_set_odr(sensor, sensor->mhz);
		if (err < 0)
			return err;
	}

	return err < 0 ? err : 0;
}

const struct dev_pm_ops st_lsm6dso16is_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lsm6dso16is_suspend, st_lsm6dso16is_resume)
};
EXPORT_SYMBOL(st_lsm6dso16is_pm_ops);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dso16is driver");
MODULE_LICENSE("GPL v2");
