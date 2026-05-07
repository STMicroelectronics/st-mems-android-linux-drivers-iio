// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_acc33 sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/version.h>

#include "st_acc33.h"

#define ST_ACC33_DATA_CHANNEL(addr, modx, scan_idx)			\
{									\
	.type = IIO_ACCEL,						\
	.address = addr,						\
	.modified = 1,							\
	.channel2 = modx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = scan_idx,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 12,						\
		.storagebits = 16,					\
		.shift = 4,						\
		.endianness = IIO_LE,					\
	},								\
}

#define ST_ACC33_EVENT_CHANNEL(chan_type, evt_spec)	\
{							\
	.type = chan_type,				\
	.modified = 0,					\
	.scan_index = -1,				\
	.indexed = -1,					\
	.event_spec = &st_acc33_##evt_spec##_event,	\
	.num_event_specs = 1,				\
}

struct st_acc33_std_entry {
	u16 odr;
	u8 val;
};

struct st_acc33_std_entry st_acc33_std_table[] = {
	{   1,  3 },
	{  10, 10 },
	{  25, 18 },
	{  50, 24 },
	{ 100, 24 },
	{ 200, 32 },
	{ 400, 48 },
};

struct st_acc33_odr {
	u32 hz;
	u8 val;
};

static const struct st_acc33_odr st_acc33_odr_table[] = {
	{   0, 0x00 },	/* power down */
	{   1, 0x01 },	/* 1Hz */
	{  10, 0x02 },	/* 10Hz */
	{  25, 0x03 },	/* 25Hz */
	{  50, 0x04 },	/* 50Hz */
	{ 100, 0x05 },	/* 100Hz */
	{ 200, 0x06 },	/* 200Hz */
	{ 400, 0x07 },	/* 400Hz */
};

struct st_acc33_fs {
	u32 gain;
	u8 fs;
	u8 val;
};

static const struct st_acc33_fs st_acc33_fs_table[] = {
	{  ST_ACC33_FS_2G,  2, 0x0 },
	{  ST_ACC33_FS_4G,  4, 0x1 },
	{  ST_ACC33_FS_8G,  8, 0x2 },
	{ ST_ACC33_FS_16G, 16, 0x3 },
};

const struct iio_event_spec st_acc33_fifo_flush_event = {
	.type = (enum iio_event_type)STM_IIO_EV_TYPE_FIFO_FLUSH,
	.dir = IIO_EV_DIR_EITHER,
};

static const struct iio_event_spec st_acc33_wakeup_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			 BIT(IIO_EV_INFO_ENABLE),
};

static const struct iio_event_spec st_acc33_freefall_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_FALLING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			 BIT(IIO_EV_INFO_ENABLE) |
			 BIT(IIO_EV_INFO_PERIOD),
};

static const struct iio_chan_spec st_acc33_channels[] = {
	ST_ACC33_DATA_CHANNEL(REG_OUTX_L_ADDR, IIO_MOD_X, 0),
	ST_ACC33_DATA_CHANNEL(REG_OUTY_L_ADDR, IIO_MOD_Y, 1),
	ST_ACC33_DATA_CHANNEL(REG_OUTZ_L_ADDR, IIO_MOD_Z, 2),
	ST_ACC33_EVENT_CHANNEL(IIO_ACCEL, fifo_flush),
	ST_ACC33_EVENT_CHANNEL(IIO_ACCEL, wakeup),
	ST_ACC33_EVENT_CHANNEL(IIO_ACCEL, freefall),

	IIO_CHAN_SOFT_TIMESTAMP(3),
};

int st_acc33_write_with_mask(struct st_acc33_hw *hw, unsigned int addr,
			     unsigned int mask, unsigned int val)
{
	unsigned int data = ST_ACC33_SHIFT_VAL(val, mask);
	int ret;

	mutex_lock(&hw->lock);
	ret = regmap_update_bits(hw->regmap, addr, mask, data);
	mutex_unlock(&hw->lock);

	return ret;
}

static int st_acc33_get_odr_val(u16 odr, u8 *val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_acc33_odr_table); i++)
		if (st_acc33_odr_table[i].hz == odr)
			break;

	if (i == ARRAY_SIZE(st_acc33_odr_table))
		return -EINVAL;

	*val = st_acc33_odr_table[i].val;

	return 0;
}

static int st_acc33_set_std_level(struct st_acc33_hw *hw, u16 odr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_acc33_std_table); i++)
		if (st_acc33_std_table[i].odr == odr)
			break;

	if (i == ARRAY_SIZE(st_acc33_std_table))
		return -EINVAL;

	hw->std_level = st_acc33_std_table[i].val;

	return 0;
}

int st_acc33_update_odr(struct st_acc33_hw *hw, u16 odr,
			bool event, bool enable)
{
	u16 req_odr;
	int err;
	u8 val;

	bool is_enabled = event ? hw->enable : hw->enable_ev_mask;

	if (enable)
		req_odr = is_enabled ? max_t(u16, odr, hw->odr) : odr;
	else
		req_odr = is_enabled ? hw->odr : 0;

	err = st_acc33_get_odr_val(req_odr, &val);
	if (err < 0)
		return err;

	return st_acc33_write_with_mask(hw, REG_CTRL1_ADDR,
					REG_CTRL1_ODR_MASK, val);
}

int st_acc33_set_enable(struct st_acc33_hw *hw, bool enable)
{
	int err;

	err = st_acc33_update_odr(hw, hw->odr, false, enable);
	if (err >= 0)
		hw->enable = enable;

	return err < 0 ? err : 0;
}

static int st_acc33_set_fs(struct st_acc33_hw *hw, u32 gain)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_acc33_fs_table); i++) {
		if (st_acc33_fs_table[i].gain >= gain)
			break;
	}

	if (i == ARRAY_SIZE(st_acc33_fs_table))
		return -EINVAL;

	err = st_acc33_write_with_mask(hw, REG_CTRL4_ADDR,
				       REG_CTRL4_FS_MASK,
				       st_acc33_fs_table[i].val);
	if (err < 0)
		return err;

	hw->gain = gain;
	hw->fs = st_acc33_fs_table[i].fs;

	return 0;
}

static int st_acc33_read_raw(struct iio_dev *iio_dev,
			     struct iio_chan_spec const *ch,
			     int *val, int *val2, long mask)
{
	struct st_acc33_hw *hw = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW: {
		int err, delay;
		u8 data[2];

		err = st_iio_device_claim_direct(iio_dev);
		if (err)
			return err;

		err = st_acc33_set_enable(hw, true);
		if (err < 0) {
			st_iio_device_release_direct(iio_dev);
			return err;
		}

		/* sample to discard, 3 * odr us */
		delay = 3000000 / hw->odr;
		usleep_range(delay, delay + 1);

		ret = regmap_bulk_read(hw->regmap, ch->address,
				       (void *)&data, 2);
		if (err < 0) {
			st_iio_device_release_direct(iio_dev);
			return err;
		}

		err = st_acc33_set_enable(hw, false);
		if (err < 0) {
			st_iio_device_release_direct(iio_dev);
			return err;
		}

		*val = (s16)get_unaligned_le16(data);
		*val = *val >> ch->scan_type.shift;

		st_iio_device_release_direct(iio_dev);

		ret = IIO_VAL_INT;
		break;
	}
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = hw->gain;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = hw->odr;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int st_acc33_write_raw(struct iio_dev *iio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct st_acc33_hw *hw = iio_priv(iio_dev);
	int err;

	err = st_iio_device_claim_direct(iio_dev);
	if (err)
		return err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = st_acc33_set_fs(hw, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		u8 data;

		err = st_acc33_set_std_level(hw, val);
		if (err < 0)
			break;

		err = st_acc33_get_odr_val(val, &data);
		if (!err)
			hw->odr = val;
		break;
	}
	default:
		err = -EINVAL;
		break;
	}
	st_iio_device_release_direct(iio_dev);

	return err;
}

static ssize_t
st_acc33_get_sampling_frequency_avail(struct device *device,
				      struct device_attribute *attr,
				      char *buf)
{
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(st_acc33_odr_table); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 st_acc33_odr_table[i].hz);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_acc33_get_scale_avail(struct device *device,
					struct device_attribute *attr,
					char *buf)
{
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(st_acc33_fs_table); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
				 st_acc33_fs_table[i].gain);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_acc33_get_module_id(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_acc33_hw *hw = iio_priv(iio_dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", hw->module_id);
}

static __maybe_unused int st_acc33_reg_access(struct iio_dev *iio_dev,
					      unsigned int reg,
					      unsigned int writeval,
					      unsigned int *readval)
{
	struct st_acc33_hw *hw = iio_priv(iio_dev);
	int ret;

	ret = st_iio_device_claim_direct(iio_dev);
	if (ret)
		return ret;

	if (readval == NULL)
		ret = regmap_write(hw->regmap, reg, writeval);
	else
		ret = regmap_read(hw->regmap, reg, readval);

	st_iio_device_release_direct(iio_dev);

	return (ret < 0) ? ret : 0;
}

static int st_acc33_enable_selftest(struct st_acc33_hw *hw, bool enable)
{
	return st_acc33_write_with_mask(hw, REG_CTRL4_ADDR,
					REG_CTRL4_ST_MASK,
					enable ? 0x02 : 0x00);
}

static int st_acc33_run_selftest(struct st_acc33_hw *hw)
{
	int x = 0, y = 0, z = 0, try_count = 0, n;
	int x_st = 0, y_st = 0, z_st = 0;
	u8 acc[6], bkregs[4];
	int delay;
	int err;
	int i;

	err = st_iio_device_claim_direct(hw->iio_dev);
	if (err)
		return err;

	err = regmap_bulk_read(hw->regmap, REG_CTRL1_ADDR,
			       (void *)&bkregs[0], sizeof(bkregs));
	if (err < 0)
		goto selftest_release;

	/*
	 * initialize sensor, turn on sensor, enable x/y/z axes
	 * set bdu, fs 2G, normal mode, odr 100 hz
	 */
	err = regmap_write(hw->regmap, REG_CTRL2_ADDR, 0x00);
	if (err < 0)
		return err;

	err = regmap_write(hw->regmap, REG_CTRL3_ADDR, 0x00);
	if (err < 0)
		goto selftest_restore;

	err = regmap_write(hw->regmap, REG_CTRL4_ADDR, 0x81);
	if (err < 0)
		goto selftest_restore;

	err = regmap_write(hw->regmap, REG_CTRL1_ADDR, 0x57);
	if (err < 0)
		goto selftest_restore;

	/* wait for 90 ms to be sure to have stable output */
	usleep_range(90000, 91000);

	/* set delay to 10ms (ODR 100Hz) */
	delay = 10000;

	/* reset number of samples read */
	n = 0;

	/* for 5 times, after checking status bit, read the output registers */
	for (i = 0; i < 6; i++) {
		unsigned int status;

		try_count = 0;

		while (try_count < 3) {
			usleep_range(delay, delay + delay / 10);

			err = regmap_read(hw->regmap, REG_STATUS_REG_A_ADDR,
					  &status);
			if (err < 0) {
				dev_err(hw->dev,
					"failed to read status register\n");

				goto selftest_exit;
			}

			if ((status & REG_STATUS_ZYXDA_MASK) && (i > 0)) {
				/*
				 * for 5 times, after checking status bit,
				 * read the output registers, skip the first
				 * read because it could be a stale sample
				 * from before enabling the sensor
				 */
				err = regmap_bulk_read(hw->regmap,
						       REG_OUTX_L_ADDR,
						       (void *)&acc[0],
						       sizeof(acc));
				if (err < 0) {
					dev_err(hw->dev,
						"failed to read output registers\n");

					goto selftest_exit;
				}

				/*
				 * shift right by 6 to remove the 6 LSBs (left
				 * justified 10-bit data)
				 */
				x += ((s16)get_unaligned_le16(&acc[0])) >> 6;
				y += ((s16)get_unaligned_le16(&acc[2])) >> 6;
				z += ((s16)get_unaligned_le16(&acc[4])) >> 6;
				n++;

				break;
			}

			try_count++;
		}
	}

	if ((i - 1) != n) {
		dev_err(hw->dev,
			"II step: samples missing (expected %d, read %d)\n",
			(i - 1), n);
		err = -1;

		goto selftest_exit;
	}

	/* average the stored data on each axis */
	x /= n;
	y /= n;
	z /= n;
	n = 0;

	/* write self test registers */
	st_acc33_enable_selftest(hw, true);
	if (err < 0)
		return err;

	/* wait for 90 ms to be sure to have stable output */
	usleep_range(90000, 91000);

	/* for 5 times, after checking status bit, read the output registers */
	for (i = 0; i < 6; i++) {
		unsigned int status;

		try_count = 0;

		while (try_count < 3) {
			usleep_range(delay, delay + delay / 10);

			err = regmap_read(hw->regmap, REG_STATUS_REG_A_ADDR,
					  &status);
			if (err < 0) {
				dev_err(hw->dev,
					"failed to read status register\n");

				goto selftest_exit;
			}

			if ((status & REG_STATUS_ZYXDA_MASK) && (i > 0)) {
				/*
				 * for 5 times, after checking status bit,
				 * read the output registers, skip the first
				 * read because it could be a stale sample
				 * from before enabling the sensor
				 */
				err = regmap_bulk_read(hw->regmap,
						       REG_OUTX_L_ADDR,
						       (void *)&acc[0],
						       sizeof(acc));
				if (err < 0) {
					dev_err(hw->dev,
						"failed to read output registers\n");

					goto selftest_exit;
				}

				/*
				 * shift right by 6 to remove the 6 LSBs (left
				 * justified 10-bit data)
				 */
				x_st += ((s16)get_unaligned_le16(&acc[0])) >> 6;
				y_st += ((s16)get_unaligned_le16(&acc[2])) >> 6;
				z_st += ((s16)get_unaligned_le16(&acc[4])) >> 6;
				n++;

				break;
			}

			try_count++;
		}
	}

	if ((i - 1) != n) {
		dev_err(hw->dev,
			"II step: samples missing (expected %d, read %d)\n",
			(i - 1), n);
		err = -1;

		goto selftest_exit;
	}

	/* average the stored data on each axis */
	x_st /= n;
	y_st /= n;
	z_st /= n;

	/* check if the difference between self test and normal values is in
	 * the expected range
	 */
	if ((abs(x_st - x) >= ST_ACC33_ST_MIN) && (abs(x_st - x) <= ST_ACC33_ST_MAX) &&
	    (abs(y_st - y) >= ST_ACC33_ST_MIN) && (abs(y_st - y) <= ST_ACC33_ST_MAX) &&
	    (abs(z_st - z) >= ST_ACC33_ST_MIN) && (abs(z_st - z) <= ST_ACC33_ST_MAX)) {
		hw->self_test = ST_ACC33_SELFTEST_SUCCESS;
		dev_info(hw->dev,
			 "self test passed: x_st %d, y_st %d, z_st %d\n",
			 abs(x_st - x), abs(y_st - y), abs(z_st - z));
	} else {
		hw->self_test = ST_ACC33_SELFTEST_FAIL;
		dev_err(hw->dev,
			"self test failed: x_st %d, y_st %d, z_st %d\n",
			abs(x_st - x), abs(y_st - y), abs(z_st - z));
	}

selftest_exit:
	st_acc33_enable_selftest(hw, false);

selftest_restore:
	/* restore registers sensor */
	regmap_write(hw->regmap, REG_CTRL2_ADDR, bkregs[1]);
	regmap_write(hw->regmap, REG_CTRL4_ADDR, bkregs[3]);
	regmap_write(hw->regmap, REG_CTRL1_ADDR, bkregs[0]);
	regmap_write(hw->regmap, REG_CTRL3_ADDR, bkregs[2]);

selftest_release:
	st_iio_device_release_direct(hw->iio_dev);

	return err;
}

static ssize_t st_acc33_get_selftest_status(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_acc33_hw *hw = iio_priv(iio_dev);
	int self_test = hw->self_test;

	hw->self_test = ST_ACC33_SELFTEST_NA;

	return scnprintf(buf, PAGE_SIZE, "%d\n", self_test);
}

static ssize_t st_acc33_set_selftest_status(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_acc33_hw *hw = iio_priv(iio_dev);
	int i;

	if (kstrtoint(buf, 10, &i))
		return -EINVAL;

	if (i == 1) {
		int err;

		hw->self_test = ST_ACC33_SELFTEST_FAIL;

		err = st_acc33_run_selftest(hw);
		if (err < 0)
			return err;
	}

	return size;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_acc33_get_sampling_frequency_avail);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_acc33_get_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644,
		       st_acc33_get_hwfifo_watermark,
		       st_acc33_set_hwfifo_watermark, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_acc33_get_max_hwfifo_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, 0200, NULL, st_acc33_flush_hwfifo, 0);
static IIO_DEVICE_ATTR(module_id, 0444, st_acc33_get_module_id, NULL, 0);
static IIO_DEVICE_ATTR(selftest, 0644, st_acc33_get_selftest_status,
		       st_acc33_set_selftest_status, 0);

static struct attribute *st_acc33_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_acc33_attribute_group = {
	.attrs = st_acc33_attributes,
};

static const struct iio_info st_acc33_info = {
	.attrs = &st_acc33_attribute_group,
	.read_raw = st_acc33_read_raw,
	.write_raw = st_acc33_write_raw,
	.read_event_config = st_acc33_read_event_config,
	.write_event_config = st_acc33_write_event_config,
	.write_event_value = st_acc33_write_event_value,
	.read_event_value = st_acc33_read_event_value,

#if IS_ENABLED(CONFIG_DEBUG_FS)
	.debugfs_reg_access = &st_acc33_reg_access,
#endif /* CONFIG_DEBUG_FS */

};

static int st_acc33_check_whoami(struct st_acc33_hw *hw)
{
	unsigned int data;
	int err;

	err = regmap_read(hw->regmap, REG_WHOAMI_ADDR, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != REG_WHOAMI_VAL) {
		dev_err(hw->dev, "wrong whoami {%02x-%02x}\n",
			data, REG_WHOAMI_VAL);
		return -ENODEV;
	}

	return 0;
}

static int st_acc33_init_device(struct st_acc33_hw *hw)
{
	int err;

	err = st_acc33_set_fs(hw, ST_ACC33_FS_4G);
	if (err < 0)
		return err;

	err = st_acc33_write_with_mask(hw, REG_CTRL4_ADDR,
				       REG_CTRL4_BDU_MASK, 1);
	if (err < 0)
		return err;

	err = st_acc33_update_watermark(hw, hw->watermark);
	if (err < 0)
		return err;

	hw->self_test = ST_ACC33_SELFTEST_NA;

	return st_acc33_write_with_mask(hw, REG_CTRL3_ADDR,
					REG_CTRL3_I1_WTM_MASK, 1);
}

static void st_acc33_get_properties(struct st_acc33_hw *hw)
{
	if (device_property_read_u32(hw->dev, "st,module_id", &hw->module_id)) {
		hw->module_id = 1;
	}
}

static int st_acc33_init_interface(struct st_acc33_hw *hw)
{
	struct device_node *np = hw->dev->of_node;

	if (np && of_find_property(np, "spi-3wire", NULL))
		return regmap_write(hw->regmap, REG_CTRL4_ADDR,
				    REG_CTRL4_SIM_MASK);

	return 0;
}

int st_acc33_probe(struct device *device, int irq, const char *name,
		   struct regmap *regmap)
{
	struct st_acc33_hw *hw;
	struct iio_dev *iio_dev;
	int err;

	iio_dev = devm_iio_device_alloc(device, sizeof(*hw));
	if (!iio_dev)
		return -ENOMEM;

	dev_set_drvdata(device, (void *)iio_dev);

	iio_dev->channels = st_acc33_channels;
	iio_dev->num_channels = ARRAY_SIZE(st_acc33_channels);
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->info = &st_acc33_info;
	iio_dev->dev.parent = device;
	iio_dev->name = name;

	hw = iio_priv(iio_dev);

	mutex_init(&hw->fifo_lock);
	mutex_init(&hw->lock);

	hw->odr = st_acc33_odr_table[1].hz;
	hw->watermark = 1;
	hw->dev = device;
	hw->regmap = regmap;
	hw->name = name;
	hw->irq = irq;
	hw->iio_dev = iio_dev;

	err = st_acc33_init_interface(hw);
	if (err < 0)
		return err;

	err = st_acc33_check_whoami(hw);
	if (err < 0)
		return err;

	st_acc33_get_properties(hw);

	err = st_acc33_init_device(hw);
	if (err < 0)
		return err;

	if (hw->irq > 0) {
		err = st_acc33_fifo_setup(hw);
		if (err < 0)
			return err;

		err = st_acc33_event_init(hw);
		if (err < 0)
			return err;
	}

	return devm_iio_device_register(hw->dev, iio_dev);
}
EXPORT_SYMBOL(st_acc33_probe);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_acc33 sensor driver");
MODULE_LICENSE("GPL v2");
