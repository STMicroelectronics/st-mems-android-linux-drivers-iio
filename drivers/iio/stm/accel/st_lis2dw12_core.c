// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2dw12 driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <asm/unaligned.h>

#include <linux/platform_data/st_sensors_pdata.h>

#include "st_lis2dw12.h"

static const enum
st_lis2dw12_sensor_id st_lis2dw12_hw_sensor_list[] = {
	[0] = ST_LIS2DW12_ID_ACC,
	[1] = ST_LIS2DW12_ID_TEMP,
};

struct st_lis2dw12_std_entry {
	u16 odr;
	u8 val;
};

struct st_lis2dw12_std_entry st_lis2dw12_std_table[] = {
	{   12, 12 },
	{   25, 18 },
	{   50, 24 },
	{  100, 24 },
	{  200, 32 },
	{  400, 48 },
	{  800, 64 },
	{ 1600, 64 },
};

static const struct st_lis2dw12_odr_entry_t st_lis2dw12_odr_table[] = {
	[ST_LIS2DW12_ID_ACC] = {
		.size = 9,
		.odr[0] = {    0, 0x0 }, /* power-down */
		.odr[1] = {   12, 0x2 }, /* LP 12.5Hz */
		.odr[2] = {   25, 0x3 }, /* LP 25Hz*/
		.odr[3] = {   50, 0x4 }, /* LP 50Hz*/
		.odr[4] = {  100, 0x5 }, /* LP 100Hz*/
		.odr[5] = {  200, 0x6 }, /* LP 200Hz*/
		.odr[6] = {  400, 0x7 }, /* HP 400Hz*/
		.odr[7] = {  800, 0x8 }, /* HP 800Hz*/
		.odr[8] = { 1600, 0x9 }, /* HP 1600Hz*/
	},
	[ST_LIS2DW12_ID_TEMP] = {
		.size = 4,
		.odr[0] = {  0, 0x0 },
		.odr[1] = { 12, 0x2 },
		.odr[2] = { 25, 0x3 },
		.odr[3] = { 50, 0x4 },
	},
};

struct st_lis2dw12_fs {
	u32 gain;
	u8 val;
};

struct st_lis2dw12_fs_entry_t {
	u8 size;
	struct st_lis2dw12_fs fs[4];
};

static const struct st_lis2dw12_fs_entry_t st_lis2dw12_fs_table[] = {
	[ST_LIS2DW12_ID_ACC] = {
		.size = 4,
		.fs[0] = {  ST_LIS2DW12_FS_2G_GAIN, 0x0 },
		.fs[1] = {  ST_LIS2DW12_FS_4G_GAIN, 0x1 },
		.fs[2] = {  ST_LIS2DW12_FS_8G_GAIN, 0x2 },
		.fs[3] = { ST_LIS2DW12_FS_16G_GAIN, 0x3 },
	},
	[ST_LIS2DW12_ID_TEMP] = {
		.size = 1,
		.fs[0] = { ST_LIS2DW12_FS_TEMP_GAIN, 0x0 },
	},
};

struct st_lis2dw12_selftest_req {
	char *mode;
	u8 val;
};

struct st_lis2dw12_selftest_req st_lis2dw12_selftest_table[] = {
	{ "disabled", 0x0 },
	{ "positive-sign", 0x1 },
	{ "negative-sign", 0x2 },
};

#define ST_LIS2DW12_ACC_CHAN(addr, ch2, idx)				\
{									\
	.type = IIO_ACCEL,						\
	.address = addr,						\
	.modified = 1,							\
	.channel2 = ch2,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = idx,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 14,						\
		.storagebits = 16,					\
		.shift = 2,						\
		.endianness = IIO_LE,					\
	},								\
}

const struct iio_event_spec st_lis2dw12_fifo_flush_event = {
	.type = STM_IIO_EV_TYPE_FIFO_FLUSH,
	.dir = IIO_EV_DIR_EITHER,
};

static const struct iio_chan_spec st_lis2dw12_acc_channels[] = {
	ST_LIS2DW12_ACC_CHAN(ST_LIS2DW12_OUT_X_L_ADDR, IIO_MOD_X, 0),
	ST_LIS2DW12_ACC_CHAN(ST_LIS2DW12_OUT_Y_L_ADDR, IIO_MOD_Y, 1),
	ST_LIS2DW12_ACC_CHAN(ST_LIS2DW12_OUT_Z_L_ADDR, IIO_MOD_Z, 2),
	ST_LIS2DW12_EVENT_CHANNEL(IIO_ACCEL, &st_lis2dw12_fifo_flush_event),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_lis2dw12_temp_channels[] = {
	{
		.type = IIO_TEMP,
		.address = ST_LIS2DW12_TEMP_OUT_T_L_ADDR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.channel2 = IIO_NO_MOD,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 12,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1)
};

static int st_lis2dw12_set_fs(struct st_lis2dw12_sensor *sensor, u16 gain)
{
	int i, err;

	for (i = 0; i < st_lis2dw12_fs_table[sensor->id].size; i++)
		if (st_lis2dw12_fs_table[sensor->id].fs[i].gain == gain)
			break;

	if (i == st_lis2dw12_fs_table[sensor->id].size)
		return -EINVAL;

	err = st_lis2dw12_write_with_mask_locked(sensor->hw,
				    ST_LIS2DW12_CTRL6_ADDR,
				    ST_LIS2DW12_FS_MASK,
				    st_lis2dw12_fs_table[sensor->id].fs[i].val);
	if (err < 0)
		return err;

	sensor->gain = gain;

	return 0;
}

static inline int st_lis2dw12_get_odr_idx(struct st_lis2dw12_sensor *sensor,
					  u16 odr, u8 *idx)
{
	enum st_lis2dw12_sensor_id id = sensor->id;
	int i;

	for (i = 0; i < st_lis2dw12_odr_table[id].size; i++)
		if (st_lis2dw12_odr_table[id].odr[i].hz == odr)
			break;

	if (i == st_lis2dw12_odr_table[id].size)
		return -EINVAL;

	*idx = i;

	return 0;
}

static int st_lis2dw12_set_std_level(struct st_lis2dw12_hw *hw, u16 odr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_lis2dw12_std_table); i++)
		if (st_lis2dw12_std_table[i].odr == odr)
			break;

	if (i == ARRAY_SIZE(st_lis2dw12_std_table))
		return -EINVAL;

	hw->std_level = st_lis2dw12_std_table[i].val;

	return 0;
}

static u16 st_lis2dw12_check_odr_dependency(struct st_lis2dw12_hw *hw, u16 odr,
					    enum st_lis2dw12_sensor_id ref_id)
{
	struct st_lis2dw12_sensor *ref = iio_priv(hw->iio_devs[ref_id]);
	bool enable = odr > 0;
	u16 ret;

	if (enable) {
		if (hw->enable_mask & BIT(ref_id))
			ret = max_t(u32, ref->odr, odr);
		else
			ret = odr;
	} else {
		ret = (hw->enable_mask & BIT(ref_id)) ? ref->odr : 0;
	}

	return ret;
}

static int st_lis2dw12_set_odr(struct st_lis2dw12_sensor *sensor, u16 req_odr)
{
	struct st_lis2dw12_hw *hw = sensor->hw;
	struct st_lis2dw12_sensor *ref =
				     iio_priv(hw->iio_devs[ST_LIS2DW12_ID_ACC]);
	u16 upd_odr = req_odr;
	u8 mode, val, i;
	int err, odr;

	for (i = 0; i < ST_LIS2DW12_ID_MAX; i++) {
		if (i == sensor->id)
			continue;

		odr = st_lis2dw12_check_odr_dependency(hw, upd_odr, i);
		if (odr > upd_odr)
			upd_odr = odr;
	}

	err = st_lis2dw12_get_odr_idx(ref, upd_odr, &i);
	if (err < 0)
		return err;

	mode = req_odr > 200 ? 0x1 : 0x0;
	val = (st_lis2dw12_odr_table[ST_LIS2DW12_ID_ACC].odr[i].val << __ffs(ST_LIS2DW12_ODR_MASK)) |
	      (mode << __ffs(ST_LIS2DW12_MODE_MASK)) | 0x01;

	/*
	 * disable cache support when setting odr register, use the
	 * driver api to restore it
	 */
	regcache_cache_bypass(hw->regmap, true);
	err = st_lis2dw12_write_locked(hw, ST_LIS2DW12_CTRL1_ADDR,
				       &val, sizeof(val));
	regcache_cache_bypass(hw->regmap, false);

	return err < 0 ? err : 0;
}

static int st_lis2dw12_check_whoami(struct st_lis2dw12_hw *hw)
{
	int data, err;

	err = regmap_read(hw->regmap, ST_LIS2DW12_WHOAMI_ADDR, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != ST_LIS2DW12_WHOAMI_VAL) {
		dev_err(hw->dev, "wrong whoami %02x vs %02x\n",
			data, ST_LIS2DW12_WHOAMI_VAL);
		return -ENODEV;
	}

	hw->odr_entry = st_lis2dw12_odr_table;

	return 0;
}

static int st_lis2dw12_of_get_drdy_pin(struct st_lis2dw12_hw *hw)
{
	struct device_node *np = hw->dev->of_node;
	int err;

	if (!np)
		return -EINVAL;

	err = of_property_read_u32(np, "st,drdy-int-pin", &hw->irq_pin);
	if (err != 0)
		return -EINVAL;

	hw->irq = of_irq_get(np, 0);
	if (hw->irq < 0)
		return hw->irq;

	if (hw->irq_pin == 1) {
		/* in case same pin only one irq is requested */
		hw->irq_emb = -1;

		return 0;
	}

	/*
	 * embedded feature when irq is on int 2 require a new dedicated
	 * irq line
	 */
	hw->irq_emb = of_irq_get(np, 1);
	if (hw->irq_emb < 0) {
		dev_err(hw->dev,
			"embedded feature require a irq line\n");

		return -EINVAL;
	}

	return 0;
}

static int st_lis2dw12_get_drdy_pin(struct st_lis2dw12_hw *hw)
{
	int err = 0;

	if (st_lis2dw12_of_get_drdy_pin(hw) < 0) {
		struct st_sensors_platform_data *pdata;
		struct device *dev = hw->dev;

		pdata = (struct st_sensors_platform_data *)dev->platform_data;
		hw->irq_pin = pdata ? pdata->drdy_int_pin : 1;
	}

	switch (hw->irq_pin) {
	case 1:
		hw->irq_reg = ST_LIS2DW12_CTRL4_INT1_CTRL_ADDR;
		break;
	case 2:
		hw->irq_reg = ST_LIS2DW12_CTRL5_INT2_CTRL_ADDR;
		break;
	default:
		dev_err(hw->dev, "unsupported interrupt pin\n");
		err = -EINVAL;
		break;
	}

	return err;
}

static int st_lis2dw12_init_hw(struct st_lis2dw12_hw *hw)
{
	int err;

	/* soft reset the device */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_CTRL2_ADDR,
						 ST_LIS2DW12_RESET_MASK, 1);
	if (err < 0)
		return err;

	/* enable BDU */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_CTRL2_ADDR,
						 ST_LIS2DW12_BDU_MASK, 1);
	if (err < 0)
		return err;

	/* enable all interrupts */
	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_ABS_INT_CFG_ADDR,
						 ST_LIS2DW12_ALL_INT_MASK, 1);
	if (err < 0)
		return err;

	/* configure fifo watermark */
	err = st_lis2dw12_update_fifo_watermark(hw, hw->watermark);
	if (err < 0)
		return err;

	/* low noise enabled by default */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_CTRL6_ADDR,
						 ST_LIS2DW12_LN_MASK, 1);
	if (err < 0)
		return err;

	/* BW = ODR/4 */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_CTRL6_ADDR,
						 ST_LIS2DW12_BW_MASK, 1);
	if (err < 0)
		return err;

	/* enable latched mode */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_CTRL3_ADDR,
						 ST_LIS2DW12_LIR_MASK, 1);
	if (err < 0)
		return err;

	/* configure interrupt pin */
	err = st_lis2dw12_get_drdy_pin(hw);
	if (err < 0)
		return err;

	return st_lis2dw12_write_with_mask_locked(hw, hw->irq_reg,
						  ST_LIS2DW12_FTH_INT_MASK, 1);
}

static ssize_t
st_lis2dw12_sysfs_sampling_frequency_avl(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	int i, len = 0;

	for (i = 1; i < st_lis2dw12_odr_table[sensor->id].size; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 st_lis2dw12_odr_table[sensor->id].odr[i].hz);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lis2dw12_sysfs_scale_avail(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	int i, len = 0;

	for (i = 0; i < st_lis2dw12_fs_table[sensor->id].size; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
				 st_lis2dw12_fs_table[sensor->id].fs[i].gain);
	buf[len - 1] = '\n';

	return len;
}

int st_lis2dw12_sensor_set_enable(struct st_lis2dw12_sensor *sensor,
				  bool enable)
{
	struct st_lis2dw12_hw *hw = sensor->hw;
	u16 val = enable ? sensor->odr : 0;
	int err;

	err = st_lis2dw12_set_odr(sensor, val);
	if (err < 0)
		return err;

	if (enable) {
		if (sensor->id == ST_LIS2DW12_ID_TEMP) {
			int64_t newTime;

			newTime = 1000000000 / sensor->odr;
			hw->oldktime = ktime_set(0, newTime);
			hrtimer_start(&hw->hr_timer, hw->oldktime,
				      HRTIMER_MODE_REL);
		}

		hw->enable_mask |= BIT(sensor->id);
	} else {
		if (sensor->id == ST_LIS2DW12_ID_TEMP) {
			cancel_work_sync(&hw->iio_work);
			hrtimer_cancel(&hw->hr_timer);
		}

		hw->enable_mask &= ~BIT(sensor->id);
	}

	return 0;
}

static int st_lis2dw12_read_oneshot(struct st_lis2dw12_sensor *sensor,
				    u8 addr, int *val)
{
	struct st_lis2dw12_hw *hw = sensor->hw;
	int err, delay;
	u8 data[2];

	err = st_lis2dw12_sensor_set_enable(sensor, true);
	if (err < 0)
		return err;

	/* sample to discard, 3 * odr us */
	delay = 3000000 / sensor->odr;
	usleep_range(delay, delay + 1);

	err = st_lis2dw12_read(hw, addr, &data, sizeof(data));
	if (err < 0)
		return err;

	st_lis2dw12_sensor_set_enable(sensor, false);

	if (sensor->id == ST_LIS2DW12_ID_ACC)
		*val = (s16)get_unaligned_le16(data);
	else
		*val = (s16)get_unaligned_le16(&data[0]) >> 4;

	return IIO_VAL_INT;
}

static int st_lis2dw12_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *ch,
				int *val, int *val2, long mask)
{
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			return ret;

		ret = st_lis2dw12_read_oneshot(sensor, ch->address, val);
		iio_device_release_direct_mode(iio_dev);
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (ch->type) {
		case IIO_ACCEL:
			*val = 0;
			*val2 = sensor->gain;
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		case IIO_TEMP:
			*val = 1000;
			*val2 = sensor->gain;
			ret = IIO_VAL_FRACTIONAL;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_OFFSET:
		switch (ch->type) {
		case IIO_TEMP:
			*val = 400;
			ret = IIO_VAL_INT;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = sensor->odr;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int st_lis2dw12_write_raw(struct iio_dev *iio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	int err;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (sensor->id == ST_LIS2DW12_ID_TEMP) {
			/* temperature sensor not allow FS change */
			err = -EINVAL;

			goto unlock;
		}

		err = st_lis2dw12_set_fs(sensor, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		u8 data;

		/* std level decimator is just for accel */
		if (sensor->id == ST_LIS2DW12_ID_ACC) {
			err = st_lis2dw12_set_std_level(sensor->hw,
							val);
			if (err < 0)
				break;
		}

		err = st_lis2dw12_get_odr_idx(sensor, val, &data);
		if (!err)
			sensor->odr = val;
		break;
	}
	default:
		err = -EINVAL;
		break;
	}

unlock:
	iio_device_release_direct_mode(iio_dev);

	return err;
}

static ssize_t st_lis2dw12_get_hwfifo_watermark(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;

	return sprintf(buf, "%d\n", hw->watermark);
}

static ssize_t
st_lis2dw12_get_max_hwfifo_watermark(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return sprintf(buf, "%d\n", ST_LIS2DW12_MAX_WATERMARK);
}

static ssize_t st_lis2dw12_get_selftest_avail(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return sprintf(buf, "%s, %s\n", st_lis2dw12_selftest_table[1].mode,
		       st_lis2dw12_selftest_table[2].mode);
}

static ssize_t st_lis2dw12_get_selftest_status(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;
	char *ret;

	switch (hw->st_status) {
	case ST_LIS2DW12_ST_PASS:
		ret = "pass";
		break;
	case ST_LIS2DW12_ST_FAIL:
		ret = "fail";
		break;
	default:
	case ST_LIS2DW12_ST_RESET:
		ret = "na";
		break;
	}

	return sprintf(buf, "%s\n", ret);
}

static ssize_t st_lis2dw12_enable_selftest(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;
	s16 acc_st_x = 0, acc_st_y = 0, acc_st_z = 0;
	s16 acc_x = 0, acc_y = 0, acc_z = 0;
	u8 data[ST_LIS2DW12_DATA_SIZE], val;
	int i, err, gain;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	if (iio_buffer_enabled(iio_dev)) {
		err = -EBUSY;
		goto unlock;
	}

	for (i = 0; i < ARRAY_SIZE(st_lis2dw12_selftest_table); i++)
		if (!strncmp(buf, st_lis2dw12_selftest_table[i].mode,
			     size - 2))
			break;

	if (i == ARRAY_SIZE(st_lis2dw12_selftest_table)) {
		err = -EINVAL;
		goto unlock;
	}

	hw->st_status = ST_LIS2DW12_ST_RESET;
	val = st_lis2dw12_selftest_table[i].val;
	gain = sensor->gain;

	/* fs = 2g, odr = 50Hz */
	err = st_lis2dw12_set_fs(sensor, ST_LIS2DW12_FS_2G_GAIN);
	if (err < 0)
		goto unlock;

	err = st_lis2dw12_set_odr(sensor, 50);
	if (err < 0)
		goto unlock;

	msleep(200);

	for (i = 0; i < 5; i++) {
		err = st_lis2dw12_read(hw, ST_LIS2DW12_OUT_X_L_ADDR,
				       &data, sizeof(data));
		if (err < 0)
			goto unlock;

		acc_x += ((s16)get_unaligned_le16(&data[0]) >> 2) / 5;
		acc_y += ((s16)get_unaligned_le16(&data[2]) >> 2) / 5;
		acc_z += ((s16)get_unaligned_le16(&data[4]) >> 2) / 5;

		msleep(10);
	}

	/* enable self test */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_CTRL3_ADDR,
						 ST_LIS2DW12_ST_MASK, val);
	if (err < 0)
		goto unlock;

	msleep(200);

	for (i = 0; i < 5; i++) {
		err = st_lis2dw12_read(hw, ST_LIS2DW12_OUT_X_L_ADDR,
				       &data, sizeof(data));
		if (err < 0)
			goto unlock;

		acc_st_x += ((s16)get_unaligned_le16(&data[0]) >> 2) / 5;
		acc_st_y += ((s16)get_unaligned_le16(&data[2]) >> 2) / 5;
		acc_st_z += ((s16)get_unaligned_le16(&data[4]) >> 2) / 5;

		msleep(10);
	}

	if (abs(acc_st_x - acc_x) >= ST_LIS2DW12_SELFTEST_MIN &&
	    abs(acc_st_x - acc_x) <= ST_LIS2DW12_SELFTEST_MAX &&
	    abs(acc_st_y - acc_y) >= ST_LIS2DW12_SELFTEST_MIN &&
	    abs(acc_st_y - acc_y) >= ST_LIS2DW12_SELFTEST_MIN &&
	    abs(acc_st_z - acc_z) >= ST_LIS2DW12_SELFTEST_MIN &&
	    abs(acc_st_z - acc_z) >= ST_LIS2DW12_SELFTEST_MIN)
		hw->st_status = ST_LIS2DW12_ST_PASS;
	else
		hw->st_status = ST_LIS2DW12_ST_FAIL;

	/* disable self test */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_CTRL3_ADDR,
						 ST_LIS2DW12_ST_MASK, 0);
	if (err < 0)
		goto unlock;

	err = st_lis2dw12_set_fs(sensor, gain);
	if (err < 0)
		goto unlock;

	err = st_lis2dw12_sensor_set_enable(sensor, false);

unlock:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_lis2dw12_sysfs_sampling_frequency_avl);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_lis2dw12_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_temp_scale_available, 0444,
		       st_lis2dw12_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644,
		       st_lis2dw12_get_hwfifo_watermark,
		       st_lis2dw12_set_hwfifo_watermark, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_lis2dw12_get_max_hwfifo_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, S_IWUSR, NULL,
		       st_lis2dw12_flush_fifo, 0);
static IIO_DEVICE_ATTR(selftest_available, 0444,
		       st_lis2dw12_get_selftest_avail, NULL, 0);
static IIO_DEVICE_ATTR(selftest, 0644, st_lis2dw12_get_selftest_status,
		       st_lis2dw12_enable_selftest, 0);

static struct attribute *st_lis2dw12_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2dw12_acc_attribute_group = {
	.attrs = st_lis2dw12_acc_attributes,
};

static const struct iio_info st_lis2dw12_acc_info = {
	.attrs = &st_lis2dw12_acc_attribute_group,
	.read_raw = st_lis2dw12_read_raw,
	.write_raw = st_lis2dw12_write_raw,
};

static struct attribute *st_lis2dw12_temp_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_temp_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2dw12_temp_attribute_group = {
	.attrs = st_lis2dw12_temp_attributes,
};

static const struct iio_info st_lis2dw12_temp_info = {
	.attrs = &st_lis2dw12_temp_attribute_group,
	.read_raw = st_lis2dw12_read_raw,
	.write_raw = st_lis2dw12_write_raw,
};

static const unsigned long st_lis2dw12_avail_scan_masks[] = { 0x7, 0x0 };
static const unsigned long st_lis2dw12_temp_avail_scan_masks[] = { 0x1, 0x0 };

static void st_lis2dw12_flush_works(struct st_lis2dw12_hw *hw)
{
	flush_workqueue(hw->temp_workqueue);
}

static void st_lis2dw12_destroy_workqueue(struct st_lis2dw12_hw *hw)
{
	if (hw->temp_workqueue)
		destroy_workqueue(hw->temp_workqueue);

	hw->temp_workqueue = NULL;
}

static void st_lis2dw12_cancel_workqueue(struct st_lis2dw12_hw *hw)
{
	/* cancel any pending work */
	cancel_work_sync(&hw->iio_work);
	hrtimer_cancel(&hw->hr_timer);
}

static int st_lis2dw12_allocate_workqueue(struct st_lis2dw12_hw *hw)
{
	struct iio_dev *iio_dev = hw->iio_devs[ST_LIS2DW12_ID_TEMP];
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);

	if (!hw->temp_workqueue)
		hw->temp_workqueue =
		      create_workqueue(sensor->name);

	if (!hw->temp_workqueue)
		return -ENOMEM;

	return 0;
}

static enum hrtimer_restart
st_lis2dw12_temp_poll_function_read(struct hrtimer *timer)
{
	struct st_lis2dw12_hw *hw;

	hw = container_of((struct hrtimer *)timer,
			  struct st_lis2dw12_hw, hr_timer);

	hw->timestamp = iio_get_time_ns(hw->iio_devs[ST_LIS2DW12_ID_TEMP]);
	queue_work(hw->temp_workqueue, &hw->iio_work);

	return HRTIMER_NORESTART;
}

static void
st_lis2dw12_temp_poll_function_work(struct work_struct *iio_work)
{
	u8 iio_buf[ALIGN(2, sizeof(s64)) + sizeof(s64)];
	struct st_lis2dw12_hw *hw;
	struct iio_dev *iio_dev;
	s16 temp;
	u8 data[2];
	int err;

	hw = container_of((struct work_struct *)iio_work,
			      struct st_lis2dw12_hw, iio_work);

	iio_dev = hw->iio_devs[ST_LIS2DW12_ID_TEMP];
	hrtimer_start(&hw->hr_timer, hw->oldktime, HRTIMER_MODE_REL);
	err = st_lis2dw12_read(hw, iio_dev->channels[0].address, data, 2);
	if (err < 0)
		return;

	temp = ((s16)get_unaligned_le16(&data[0]) >> 4);

	memcpy(iio_buf, &temp, 2);
	iio_push_to_buffers_with_timestamp(iio_dev, iio_buf, hw->timestamp);
}

static struct iio_dev *st_lis2dw12_alloc_iiodev(struct st_lis2dw12_hw *hw,
						enum st_lis2dw12_sensor_id id)
{
	struct st_lis2dw12_sensor *sensor;
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
	case ST_LIS2DW12_ID_ACC:
		iio_dev->channels = st_lis2dw12_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lis2dw12_acc_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_accel", hw->name);
		iio_dev->info = &st_lis2dw12_acc_info;
		iio_dev->available_scan_masks = st_lis2dw12_avail_scan_masks;

		sensor->odr =
			    st_lis2dw12_odr_table[ST_LIS2DW12_ID_ACC].odr[1].hz;
		sensor->gain =
			    st_lis2dw12_fs_table[ST_LIS2DW12_ID_ACC].fs[0].gain;
		break;
	case ST_LIS2DW12_ID_TEMP:
		iio_dev->channels = st_lis2dw12_temp_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lis2dw12_temp_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_temp", hw->name);
		iio_dev->info = &st_lis2dw12_temp_info;
		iio_dev->available_scan_masks =
					      st_lis2dw12_temp_avail_scan_masks;

		sensor->odr =
			   st_lis2dw12_odr_table[ST_LIS2DW12_ID_TEMP].odr[1].hz;
		sensor->gain =
			   st_lis2dw12_fs_table[ST_LIS2DW12_ID_TEMP].fs[0].gain;

		/* configure hrtimer and workqueue */
		hrtimer_init(&hw->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		hw->hr_timer.function = &st_lis2dw12_temp_poll_function_read;

		hw->oldktime = ktime_set(0, 1000000000 / sensor->odr);
		INIT_WORK(&hw->iio_work, st_lis2dw12_temp_poll_function_work);
		break;
	default:
		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

int st_lis2dw12_probe(struct device *dev, int irq, const char *name,
		      struct regmap *regmap)
{
	struct st_lis2dw12_hw *hw;
	int i, err;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	dev_set_drvdata(dev, (void *)hw);

	mutex_init(&hw->fifo_lock);
	mutex_init(&hw->lock);

	hw->dev = dev;
	hw->irq = irq;
	hw->regmap = regmap;
	hw->watermark = 1;

	err = st_lis2dw12_check_whoami(hw);
	if (err < 0)
		return err;

	scnprintf(hw->name, sizeof(hw->name), "%s", name);

	err = st_lis2dw12_init_hw(hw);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(st_lis2dw12_hw_sensor_list); i++) {
		hw->iio_devs[i] = st_lis2dw12_alloc_iiodev(hw,
						 st_lis2dw12_hw_sensor_list[i]);
		if (!hw->iio_devs[i])
			continue;
	}

	if (hw->irq > 0) {
		err = st_lis2dw12_fifo_setup(hw);
		if (err)
			return err;
	}

#ifdef CONFIG_IIO_ST_LIS2DW12_EN_BASIC_FEATURES
	err = st_lis2dw12_embedded_function_probe(hw);
	if (err < 0)
		return err;
#endif /* CONFIG_IIO_ST_LIS2DW12_EN_BASIC_FEATURES */

	/* allocate temperature sensor workqueue */
	err = st_lis2dw12_allocate_workqueue(hw);
	if (err < 0)
		return err;

	for (i = 0; i < ST_LIS2DW12_ID_MAX; i++) {
		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

	device_init_wakeup(dev,
			   device_property_read_bool(dev,
						     "wakeup-source"));
	return 0;
}
EXPORT_SYMBOL(st_lis2dw12_probe);

int st_lis2dw12_remove(struct device *dev)
{
	struct st_lis2dw12_hw *hw = dev_get_drvdata(dev);

	st_lis2dw12_flush_works(hw);
	st_lis2dw12_destroy_workqueue(hw);

	return 0;
}
EXPORT_SYMBOL(st_lis2dw12_remove);

static int __maybe_unused st_lis2dw12_suspend(struct device *dev)
{
	struct st_lis2dw12_hw *hw = dev_get_drvdata(dev);
	struct st_lis2dw12_sensor *sensor;
	int i, err = 0;

	if (hw->irq > 0)
		disable_hardirq(hw->irq);

	/* cancel any pending work */
	st_lis2dw12_cancel_workqueue(hw);

	for (i = 0; i < ST_LIS2DW12_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		/* do not disable sensors if requested by wake-up */
		if (!((hw->enable_mask & BIT(sensor->id)) &
		      ST_LIS2DW12_WAKE_UP_SENSORS)) {
			err = st_lis2dw12_set_odr(sensor, 0);
			if (err < 0)
				return err;
		} else {
			err = st_lis2dw12_set_odr(sensor,
					 ST_LIS2DW12_MIN_ODR_IN_WAKEUP);
			if (err < 0)
				return err;
		}
	}

	if (st_lis2dw12_is_fifo_enabled(hw)) {
		err = st_lis2dw12_suspend_fifo(hw);
		if (err < 0)
			return err;
	}

	regcache_mark_dirty(hw->regmap);

	if (hw->enable_mask & ST_LIS2DW12_WAKE_UP_SENSORS) {
		if (device_may_wakeup(dev))
			enable_irq_wake(hw->irq);
	}

	dev_info(dev, "Suspending device\n");

	return err < 0 ? err : 0;
}

static int __maybe_unused st_lis2dw12_resume(struct device *dev)
{
	struct st_lis2dw12_hw *hw = dev_get_drvdata(dev);
	struct st_lis2dw12_sensor *sensor;
	int i, err = 0;

	dev_info(dev, "Resuming device\n");

	regcache_sync(hw->regmap);

	if (hw->enable_mask & ST_LIS2DW12_WAKE_UP_SENSORS) {
		if (device_may_wakeup(dev))
			disable_irq_wake(hw->irq);
	}

	for (i = 0; i < ST_LIS2DW12_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		err = st_lis2dw12_set_odr(sensor, sensor->odr);
		if (err < 0)
			return err;
	}

	if (st_lis2dw12_is_fifo_enabled(hw))
		err = st_lis2dw12_set_fifo_mode(hw,
					   ST_LIS2DW12_FIFO_CONTINUOUS);

	if (hw->irq > 0)
		enable_irq(hw->irq);

	return err < 0 ? err : 0;
}

const struct dev_pm_ops st_lis2dw12_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lis2dw12_suspend, st_lis2dw12_resume)
};
EXPORT_SYMBOL(st_lis2dw12_pm_ops);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_lis2dw12 driver");
MODULE_LICENSE("GPL v2");
