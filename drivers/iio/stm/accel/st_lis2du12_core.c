// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2du12 driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2022 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <asm/unaligned.h>

#include <linux/platform_data/st_sensors_pdata.h>

#include "st_lis2du12.h"

static const struct st_lis2du12_std_entry {
	u16 odr;
	u8 val;
} st_lis2du12_std_table[] = {
	{    6,  3 },
	{   12,  3 },
	{   25,  3 },
	{   50,  3 },
	{  100,  3 },
	{  200,  3 },
	{  400,  3 },
	{  800,  3 },
};

static const struct st_lis2du12_odr st_lis2du12_odr_table[ST_LIS2DU12_ODR_TABLE_SIZE] = {
	{    0,      0, 0x00 },
	{    1, 500000, 0x01 },
	{    3,      0, 0x02 },
	{    6,      0, 0x04 },
	{   12, 500000, 0x05 },
	{   25,      0, 0x06 },
	{   50,      0, 0x07 },
	{  100,      0, 0x08 },
	{  200,      0, 0x09 },
	{  400,      0, 0x0a },
	{  800,      0, 0x0b },
};

static const struct st_lis2du12_fs st_lis2du12_fs_table[] = {
	{ IIO_G_TO_M_S_2(976),  0x00 },
	{ IIO_G_TO_M_S_2(1952), 0x01 },
	{ IIO_G_TO_M_S_2(3904), 0x02 },
	{ IIO_G_TO_M_S_2(7808), 0x03 },
};

static const struct st_lis2du12_selftest_req {
	char *mode;
	u8 val;
} st_lis2du12_selftest_table[] = {
	{ "disabled",      0x0 },
	{ "positive-sign", 0x6 },
	{ "negative-sign", 0x1 },
};

static const struct iio_chan_spec st_lis2du12_acc_channels[] = {
	ST_LIS2DU12_ACC_CHAN(ST_LIS2DU12_OUT_X_L_ADDR, IIO_MOD_X, 0),
	ST_LIS2DU12_ACC_CHAN(ST_LIS2DU12_OUT_Y_L_ADDR, IIO_MOD_Y, 1),
	ST_LIS2DU12_ACC_CHAN(ST_LIS2DU12_OUT_Z_L_ADDR, IIO_MOD_Z, 2),
	ST_LIS2DU12_EVENT_CHANNEL(IIO_ACCEL, fifo_flush),
	ST_LIS2DU12_EVENT_CHANNEL(IIO_ACCEL, freefall),
	ST_LIS2DU12_EVENT_CHANNEL(IIO_ACCEL, wakeup),
	ST_LIS2DU12_EVENT_CHANNEL(IIO_ACCEL, 6D),

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ST_LIS2DU12_EVENT_CHANNEL(IIO_ACCEL, tap),
	ST_LIS2DU12_EVENT_CHANNEL(IIO_ACCEL, dtap),
#endif /* LINUX_VERSION_CODE */

	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_lis2du12_temp_channels[] = {
	ST_LIS2DU12_TEMP_CHAN(ST_LIS2DU12_TEMP_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static int st_lis2du12_set_fs(struct st_lis2du12_sensor *sensor,
			      u16 gain)
{
	struct st_lis2du12_hw *hw = sensor->hw;
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_lis2du12_fs_table); i++)
		if (st_lis2du12_fs_table[i].gain == gain)
			break;

	if (i == ARRAY_SIZE(st_lis2du12_fs_table))
		return -EINVAL;

	err = st_lis2du12_write_locked_delayed(hw, ST_LIS2DU12_CTRL5_ADDR,
					       ST_LIS2DU12_FS_MASK,
					       st_lis2du12_fs_table[i].val);
	if (err < 0)
		return err;

	sensor->gain = gain;

	return 0;
}

static inline int st_lis2du12_get_odr_idx(u16 odr, u32 uodr)
{
	int uhz = ST_LIS2DU12_ODR_EXPAND(odr, uodr);
	int ref_uhz;
	int i;

	for (i = 0; i < ARRAY_SIZE(st_lis2du12_odr_table); i++) {
		ref_uhz = ST_LIS2DU12_ODR_EXPAND(st_lis2du12_odr_table[i].hz,
						 st_lis2du12_odr_table[i].uhz);
		if (ref_uhz >= uhz)
			break;
	}

	if (i == ARRAY_SIZE(st_lis2du12_odr_table))
		return -EINVAL;

	return i;
}

static int st_lis2du12_check_odr_dependency(struct st_lis2du12_hw *hw,
					    u16 odr, u32 uodr,
					    enum st_lis2du12_sensor_id ref_id)
{
	struct st_lis2du12_sensor *ref = iio_priv(hw->iio_devs[ref_id]);
	int ref_uhz = ST_LIS2DU12_ODR_EXPAND(ref->odr, ref->uodr);
	int uhz = ST_LIS2DU12_ODR_EXPAND(odr, uodr);
	int ret;

	if (ref_uhz > 0) {
		if (hw->enable_mask & BIT(ref_id))
			ret = max_t(int, ref_uhz, uhz);
		else
			ret = uhz;
	} else {
		ret = (hw->enable_mask & BIT(ref_id)) ? ref_uhz : 0;
	}

	return ret;
}

int st_lis2du12_set_odr(struct st_lis2du12_sensor *sensor,
			u16 req_odr, u32 req_uodr)
{
	int uhz = ST_LIS2DU12_ODR_EXPAND(req_odr, req_uodr);
	struct st_lis2du12_hw *hw = sensor->hw;
	int err, odr, ret;
	u8 i;

	for (i = 0; i < ST_LIS2DU12_ID_MAX; i++) {
		if (i == sensor->id)
			continue;

		odr = st_lis2du12_check_odr_dependency(hw, req_odr,
						       req_uodr, i);
		/* check if device is already configured */
		if (odr != uhz)
			return 0;
	}

	ret = st_lis2du12_get_odr_idx(req_odr, req_uodr);
	if (ret < 0)
		return ret;

	mutex_lock(&hw->lock);
	err = st_lis2du12_write_delayed(sensor->hw,
					ST_LIS2DU12_CTRL5_ADDR,
					ST_LIS2DU12_ODR_MASK,
					st_lis2du12_odr_table[ret].val);
	if (ret < 0)
		goto unlock;

unlock:
	mutex_unlock(&hw->lock);

	return 0;
}

static int st_lis2du12_check_whoami(struct st_lis2du12_hw *hw)
{
	int err, data;

	err = regmap_read(hw->regmap, ST_LIS2DU12_WHOAMI_ADDR, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");

		return err;
	}

	if (data != ST_LIS2DU12_WHOAMI_VAL) {
		dev_err(hw->dev, "wrong whoami %02x vs %02x\n",
			data, ST_LIS2DU12_WHOAMI_VAL);
		return -ENODEV;
	}

	hw->odr_table = st_lis2du12_odr_table;
	hw->fs_tablefs_table = st_lis2du12_fs_table;

	return 0;
}

static int st_lis2du12_of_get_int_pin(struct st_lis2du12_hw *hw, int *int_pin)
{
	struct device_node *np = hw->dev->of_node;

	if (!np)
		return -EINVAL;

	return of_property_read_u32(np, "st,int-pin", int_pin);
}

static int st_lis2du12_get_int_pin(struct st_lis2du12_hw *hw)
{
	int err = 0, drdy_pin;

	if (st_lis2du12_of_get_int_pin(hw, &drdy_pin) < 0) {
		struct st_sensors_platform_data *pdata;
		struct device *dev = hw->dev;

		pdata = (struct st_sensors_platform_data *)dev->platform_data;
		drdy_pin = pdata ? pdata->drdy_int_pin : 1;
	}

	switch (drdy_pin) {
	case 1:
		hw->drdy_reg = ST_LIS2DU12_CTRL2_ADDR;
		hw->md_reg = ST_LIS2DU12_MD1_CFG_ADDR;
		break;
	case 2:
		hw->drdy_reg = ST_LIS2DU12_CTRL3_ADDR;
		hw->md_reg = ST_LIS2DU12_MD2_CFG_ADDR;
		break;
	default:
		dev_err(hw->dev, "unsupported interrupt pin\n");
		err = -EINVAL;
		break;
	}

	return err;
}

static int st_lis2du12_init_hw(struct st_lis2du12_hw *hw)
{
	int err;

	/* read interrupt pin configuration */
	err = st_lis2du12_get_int_pin(hw);
	if (err < 0)
		return err;

	/* soft reset procedure */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_CTRL1_ADDR,
				 ST_LIS2DU12_SW_RESET_MASK,
				 FIELD_PREP(ST_LIS2DU12_SW_RESET_MASK, 1));
	if (err < 0)
		return err;

	/* wait 50 μs */
	usleep_range(55, 60);

	/* boot procedure */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_CTRL4_ADDR,
				 ST_LIS2DU12_BOOT_MASK,
				 FIELD_PREP(ST_LIS2DU12_BOOT_MASK, 1));
	if (err < 0)
		return err;

	usleep_range(20000, 20100);

	/* enable BDU */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_CTRL4_ADDR,
				 ST_LIS2DU12_BDU_MASK,
				 FIELD_PREP(ST_LIS2DU12_BDU_MASK, 1));
	if (err < 0)
		return err;

	/*
	 * register address is automatically incremented during a
	 * multiple-byte access with a serial interface
	 */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_CTRL1_ADDR,
				 ST_LIS2DU12_IF_ADD_INC_MASK,
				 FIELD_PREP(ST_LIS2DU12_IF_ADD_INC_MASK, 1));
	if (err < 0)
		return err;


	/* set interrupt latched */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_INTERRUPT_CFG_ADDR,
				 ST_LIS2DU12_INT_SHORT_EN_MASK,
				 FIELD_PREP(ST_LIS2DU12_INT_SHORT_EN_MASK, 1));
	if (err < 0)
		return err;

	return regmap_update_bits(hw->regmap,
				  ST_LIS2DU12_INTERRUPT_CFG_ADDR,
				  ST_LIS2DU12_LIR_MASK,
				  FIELD_PREP(ST_LIS2DU12_LIR_MASK, 1));
}

static ssize_t
st_lis2du12_sysfs_sampling_frequency_avl(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i, len = 0;

	for (i = 1; i < ARRAY_SIZE(st_lis2du12_odr_table); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%06d ",
				 st_lis2du12_odr_table[i].hz,
				 st_lis2du12_odr_table[i].uhz);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lis2du12_sysfs_scale_avail(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(st_lis2du12_fs_table); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
				 st_lis2du12_fs_table[i].gain);
	buf[len - 1] = '\n';

	return len;
}

int st_lis2du12_sensor_set_enable(struct st_lis2du12_sensor *sensor,
				  bool enable)
{
	struct st_lis2du12_hw *hw = sensor->hw;
	int err;

	err = st_lis2du12_set_odr(sensor, enable ? sensor->odr : 0,
				  enable ? sensor->uodr : 0);
	if (err < 0)
		return err;

	if (enable)
		hw->enable_mask |= BIT(sensor->id);
	else
		hw->enable_mask &= ~BIT(sensor->id);

	return 0;
}

static int st_lis2du12_read_oneshot(struct st_lis2du12_sensor *sensor,
				    u8 addr, int *val)
{
	struct st_lis2du12_hw *hw = sensor->hw;
	int err, delay;
	u8 data[2];

	err = st_lis2du12_sensor_set_enable(sensor, true);
	if (err < 0)
		return err;

	/* sample to discard, 3 * odr us */
	delay = 3000000 / sensor->odr;
	usleep_range(delay, delay + 1);

	err = st_lis2du12_read_locked(hw, addr, &data, sizeof(data));
	if (err < 0)
		return err;

	st_lis2du12_sensor_set_enable(sensor, false);

	*val = (s16)get_unaligned_le16(data);

	return IIO_VAL_INT;
}

static int st_lis2du12_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *ch,
				int *val, int *val2, long mask)
{
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			return ret;

		ret = st_lis2du12_read_oneshot(sensor, ch->address,
					       val);
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
	case IIO_CHAN_INFO_SCALE:
		switch (ch->type) {
		case IIO_TEMP:
			*val = 1000;
			*val2 = sensor->gain;
			ret = IIO_VAL_FRACTIONAL;
			break;
		case IIO_ACCEL:
			*val = 0;
			*val2 = sensor->gain;
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = (int)sensor->odr;
		*val2 = (int)sensor->uodr;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int st_lis2du12_write_raw(struct iio_dev *iio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);
	int err = 0;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = st_lis2du12_set_fs(sensor, val2);
		if (chan->type == IIO_ACCEL)
			err = st_lis2du12_update_threshold_events(sensor->hw);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		int ret;

		ret = st_lis2du12_get_odr_idx(val, val2);
		if (ret < 0) {
			err = ret;
			break;
		}

		sensor->hw->std_level = st_lis2du12_std_table[ret].val;
		sensor->odr = val;
		sensor->uodr = val2;

		/* some events depends on xl odr */
		if (chan->type == IIO_ACCEL)
			err = st_lis2du12_update_duration_events(sensor->hw);
		break;
	}
	default:
		err = -EINVAL;
		break;
	}

	iio_device_release_direct_mode(iio_dev);

	return err;
}

static ssize_t st_lis2du12_get_hwfifo_watermark(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);

	return sprintf(buf, "%d\n", sensor->watermark);
}

static ssize_t
st_lis2du12_get_max_hwfifo_watermark(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return sprintf(buf, "%d\n", ST_LIS2DU12_MAX_WATERMARK);
}

static ssize_t st_lis2du12_get_selftest_avail(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return sprintf(buf, "%s, %s\n", st_lis2du12_selftest_table[1].mode,
		       st_lis2du12_selftest_table[2].mode);
}

static ssize_t st_lis2du12_get_selftest_status(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2du12_hw *hw = sensor->hw;
	char *ret;

	switch (hw->st_status) {
	case ST_LIS2DU12_ST_PASS:
		ret = "pass";
		break;
	case ST_LIS2DU12_ST_FAIL:
		ret = "fail";
		break;
	default:
	case ST_LIS2DU12_ST_RESET:
		ret = "na";
		break;
	}

	return sprintf(buf, "%s\n", ret);
}

static ssize_t st_lis2du12_enable_selftest(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct iio_chan_spec const *ch = iio_dev->channels;
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2du12_hw *hw = sensor->hw;
	u8 data[ST_LIS2DU12_ACC_DATA_SIZE], val;
	s16 out1[3], out2[3];
	int i, err, gain;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	for (i = 0; i < ARRAY_SIZE(st_lis2du12_selftest_table); i++)
		if (!strncmp(buf, st_lis2du12_selftest_table[i].mode,
			     size - 2))
			break;

	if (i == ARRAY_SIZE(st_lis2du12_selftest_table)) {
		err = -EINVAL;
		goto unlock;
	}

	hw->st_status = ST_LIS2DU12_ST_RESET;
	val = st_lis2du12_selftest_table[i].val;
	gain = sensor->gain;

	/* set self test mode */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_ST_SIGN_ADDR,
				 ST_LIS2DU12_STSIGN_MASK,
				 FIELD_PREP(ST_LIS2DU12_STSIGN_MASK, val));
	if (err < 0)
		goto unlock;

	err = st_lis2du12_set_odr(sensor, 0, 0);
	if (err < 0)
		goto unlock;

	/* start test mode */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_CTRL3_ADDR,
				 ST_LIS2DU12_ST_MASK,
				 FIELD_PREP(ST_LIS2DU12_ST_MASK, 0x02));
	if (err < 0)
		goto unlock;

	/* fs = 8g, odr = 200Hz */
	err = st_lis2du12_set_fs(sensor, IIO_G_TO_M_S_2(3904));
	if (err < 0)
		goto unlock;

	err = st_lis2du12_set_odr(sensor, 200, 0);
	if (err < 0)
		goto unlock;

	err = st_lis2du12_sensor_set_enable(sensor, true);
	if (err < 0)
		goto unlock;

	msleep(30);

	err = regmap_bulk_read(hw->regmap, ch[0].address,
			       data, sizeof(data));
	if (err < 0)
		goto unlock;

	err = st_lis2du12_set_odr(sensor, 0, 0);
	if (err < 0)
		goto unlock;

	out1[0] = ((s16)get_unaligned_le16(&data[0]) >> 4);
	out1[1] = ((s16)get_unaligned_le16(&data[2]) >> 4);
	out1[2] = ((s16)get_unaligned_le16(&data[4]) >> 4);

	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_CTRL3_ADDR,
				 ST_LIS2DU12_ST_MASK,
				 FIELD_PREP(ST_LIS2DU12_ST_MASK, 0x01));
	if (err < 0)
		goto unlock;

	/* fs = 8g, odr = 200Hz */
	err = st_lis2du12_set_odr(sensor, 200, 0);
	if (err < 0)
		goto unlock;

	msleep(30);

	err = regmap_bulk_read(hw->regmap, ch[0].address,
			       data, sizeof(data));
	if (err < 0)
		goto unlock;

	err = st_lis2du12_set_odr(sensor, 0, 0);
	if (err < 0)
		goto unlock;

	out2[0] = ((s16)get_unaligned_le16(&data[0]) >> 4);
	out2[1] = ((s16)get_unaligned_le16(&data[2]) >> 4);
	out2[2] = ((s16)get_unaligned_le16(&data[4]) >> 4);

	/* disable self test */
	err = regmap_update_bits(hw->regmap, ST_LIS2DU12_CTRL3_ADDR,
				 ST_LIS2DU12_ST_MASK,
				 FIELD_PREP(ST_LIS2DU12_ST_MASK, 0));
	if (err < 0)
		goto unlock;

	if ((abs(out2[0] - out1[0]) >= 180) ||
	    (abs(out2[0] - out1[0]) < 12)) {
		hw->st_status = ST_LIS2DU12_ST_FAIL;
		goto selftest_restore;
	}

	if ((abs(out2[1] - out1[1]) >= 180) ||
	    (abs(out2[1] - out1[1]) < 12)) {
		hw->st_status = ST_LIS2DU12_ST_FAIL;
		goto selftest_restore;
	}

	if ((abs(out2[2] - out1[2]) >= 308) ||
	    (abs(out2[2] - out1[2]) < 51)) {
		hw->st_status = ST_LIS2DU12_ST_FAIL;
		goto selftest_restore;
	}

	hw->st_status = ST_LIS2DU12_ST_PASS;

selftest_restore:
	err = st_lis2du12_set_fs(sensor, gain);
	if (err < 0)
		goto unlock;

	err = st_lis2du12_sensor_set_enable(sensor, false);

unlock:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_lis2du12_sysfs_sampling_frequency_avl);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_lis2du12_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644,
		       st_lis2du12_get_hwfifo_watermark,
		       st_lis2du12_set_hwfifo_watermark, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_lis2du12_get_max_hwfifo_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, 0200, NULL,
		       st_lis2du12_flush_fifo, 0);
static IIO_DEVICE_ATTR(selftest_available, 0444,
		       st_lis2du12_get_selftest_avail, NULL, 0);
static IIO_DEVICE_ATTR(selftest, 0644, st_lis2du12_get_selftest_status,
		       st_lis2du12_enable_selftest, 0);

static struct attribute *st_lis2du12_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2du12_acc_attribute_group = {
	.attrs = st_lis2du12_acc_attributes,
};

static const struct iio_info st_lis2du12_acc_info = {
	.attrs = &st_lis2du12_acc_attribute_group,
	.read_raw = st_lis2du12_read_raw,
	.write_raw = st_lis2du12_write_raw,

	.read_event_config = st_lis2du12_read_event_config,
	.write_event_config = st_lis2du12_write_event_config,
	.write_event_value = st_lis2du12_write_event_value,
	.read_event_value = st_lis2du12_read_event_value,
};

static struct attribute *st_lis2du12_temp_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2du12_temp_attribute_group = {
	.attrs = st_lis2du12_temp_attributes,
};

static const struct iio_info st_lis2du12_temp_info = {
	.attrs = &st_lis2du12_temp_attribute_group,
	.read_raw = st_lis2du12_read_raw,
	.write_raw = st_lis2du12_write_raw,
};

static const unsigned long st_lis2du12_avail_acc_scan_masks[] = {
	BIT(2) | BIT(1) | BIT(0), 0x0
};

static const unsigned long st_lis2du12_avail_temp_scan_masks[] = {
	BIT(0), 0x0
};

static struct iio_dev *st_lis2du12_alloc_iiodev(struct st_lis2du12_hw *hw,
						enum st_lis2du12_sensor_id id)
{
	struct st_lis2du12_sensor *sensor;
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
	case ST_LIS2DU12_ID_ACC:
		iio_dev->channels = st_lis2du12_acc_channels;
		iio_dev->num_channels =
				ARRAY_SIZE(st_lis2du12_acc_channels);
		iio_dev->name = ST_LIS2DU12_DEV_NAME "_accel";
		iio_dev->info = &st_lis2du12_acc_info;
		iio_dev->available_scan_masks =
				st_lis2du12_avail_acc_scan_masks;

		sensor->odr = st_lis2du12_odr_table[4].hz;
		sensor->uodr = st_lis2du12_odr_table[4].uhz;
		st_lis2du12_set_fs(sensor, st_lis2du12_fs_table[0].gain);
		sensor->offset = 0;
		sensor->watermark = 1;
		break;
	case ST_LIS2DU12_ID_TEMP:
		iio_dev->channels = st_lis2du12_temp_channels;
		iio_dev->num_channels =
				ARRAY_SIZE(st_lis2du12_temp_channels);
		iio_dev->name = ST_LIS2DU12_DEV_NAME "_temp";
		iio_dev->info = &st_lis2du12_temp_info;
		iio_dev->available_scan_masks =
				st_lis2du12_avail_temp_scan_masks;

		sensor->odr = st_lis2du12_odr_table[4].hz;
		sensor->uodr = st_lis2du12_odr_table[4].uhz;

		/* temperature has fixed gain and offset */
		sensor->gain = ST_LIS2DU12_TEMP_GAIN;
		sensor->offset = ST_LIS2DU12_TEMP_OFFSET;
		sensor->watermark = 1;
		break;
	default:
		return NULL;
	}

	return iio_dev;
}

int st_lis2du12_probe(struct device *dev, int irq, struct regmap *regmap)
{
	struct st_lis2du12_hw *hw;
	int i, id, err;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	dev_set_drvdata(dev, (void *)hw);

	mutex_init(&hw->fifo_lock);
	mutex_init(&hw->lock);

	hw->dev = dev;
	hw->irq = irq;
	hw->regmap = regmap;

	err = st_lis2du12_check_whoami(hw);
	if (err < 0)
		return err;

	err = st_lis2du12_init_hw(hw);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(st_lis2du12_hw_sensor_list); i++) {
		id = st_lis2du12_hw_sensor_list[i];
		hw->iio_devs[id] = st_lis2du12_alloc_iiodev(hw, id);
		if (!hw->iio_devs[id])
			return -ENOMEM;
	}

	if (hw->irq > 0) {
		err = st_lis2du12_event_init(hw);
		if (err < 0)
			return err;

		err = st_lis2du12_buffer_setup(hw);
		if (err < 0)
			return err;
	}

	for (i = 0; i < ST_LIS2DU12_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(st_lis2du12_probe);

static int __maybe_unused st_lis2du12_resume(struct device *dev)
{
	struct st_lis2du12_hw *hw = dev_get_drvdata(dev);

	dev_info(dev, "Resuming device\n");

	if (device_may_wakeup(dev))
		disable_irq_wake(hw->irq);

	return 0;
}

static int __maybe_unused st_lis2du12_suspend(struct device *dev)
{
	struct st_lis2du12_hw *hw = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(hw->irq);

	dev_info(dev, "Suspending device\n");

	return 0;
}

const struct dev_pm_ops st_lis2du12_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lis2du12_suspend,
				st_lis2du12_resume)
};
EXPORT_SYMBOL(st_lis2du12_pm_ops);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_lis2du12 driver");
MODULE_LICENSE("GPL v2");
