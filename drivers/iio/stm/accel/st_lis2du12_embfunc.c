// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lis2du12 embedded function sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2024 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/irq.h>
#include <linux/module.h>

#include "st_lis2du12.h"

static const enum
st_lis2du12_sensor_id st_lis2du12_embedded_function_sensor_list[] = {
	[0] = ST_LIS2DU12_ID_TAP_TAP,
	[1] = ST_LIS2DU12_ID_TAP,
	[2] = ST_LIS2DU12_ID_WU,
	[3] = ST_LIS2DU12_ID_FF,
	[4] = ST_LIS2DU12_ID_6D,
	[5] = ST_LIS2DU12_ID_ACT,
};

static bool
st_lis2du12_interrupts_enabled(struct st_lis2du12_hw *hw)
{
	return hw->enable_mask & (BIT(ST_LIS2DU12_ID_FF) |
				  BIT(ST_LIS2DU12_ID_TAP_TAP) |
				  BIT(ST_LIS2DU12_ID_TAP)  |
				  BIT(ST_LIS2DU12_ID_WU) |
				  BIT(ST_LIS2DU12_ID_6D) |
				  BIT(ST_LIS2DU12_ID_ACT));
}

static const struct iio_event_spec st_lis2du12_rthr_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

static const struct iio_chan_spec st_lis2du12_tap_tap_channels[] = {
	ST_LIS2DU12_EVENT_CHANNEL(STM_IIO_TAP_TAP,
				  &st_lis2du12_rthr_event),
};

static const struct iio_chan_spec st_lis2du12_tap_channels[] = {
	ST_LIS2DU12_EVENT_CHANNEL(STM_IIO_TAP,
				  &st_lis2du12_rthr_event),
};

static const struct iio_chan_spec st_lis2du12_wu_channels[] = {
	ST_LIS2DU12_EVENT_CHANNEL(STM_IIO_GESTURE,
				  &st_lis2du12_rthr_event),
};

static const struct iio_chan_spec st_lis2du12_ff_channels[] = {
	ST_LIS2DU12_EVENT_CHANNEL(STM_IIO_GESTURE,
				  &st_lis2du12_rthr_event),
};

static const struct iio_chan_spec st_lis2du12_6d_channels[] = {
	ST_LIS2DU12_EVENT_CHANNEL(STM_IIO_GESTURE,
				  &st_lis2du12_rthr_event),
};

static const struct iio_chan_spec st_lis2du12_act_channels[] = {
	ST_LIS2DU12_EVENT_CHANNEL(STM_IIO_GESTURE,
				  &st_lis2du12_rthr_event),
};

static int st_lis2du12_embedded_config(struct st_lis2du12_hw *hw)
{
	struct st_lis2du12_sensor *sensor;
	int err;

	/* configure default free fall event threshold */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_FREE_FALL_ADDR,
				 ST_LIS2DU12_FF_THS_MASK,
				 FIELD_PREP(ST_LIS2DU12_FF_THS_MASK, 1));
	if (err < 0)
		return err;

	sensor = st_lis2du12_get_sensor_from_id(hw, ST_LIS2DU12_ID_FF);
	sensor->ff_ths = 1;

	/* configure default free fall event duration */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_FREE_FALL_ADDR,
				 ST_LIS2DU12_FF_DUR_MASK,
				 FIELD_PREP(ST_LIS2DU12_FF_DUR_MASK, 1));
	if (err < 0)
		return err;

	sensor = st_lis2du12_get_sensor_from_id(hw, ST_LIS2DU12_ID_FF);
	sensor->ff_dur = 1;

	/* enable tap event on all axes */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_TAP_THS_Z_ADDR,
				 ST_LIS2DU12_TAP_EN_MASK,
				 FIELD_PREP(ST_LIS2DU12_TAP_EN_MASK, 0x7));
	if (err < 0)
		return err;

	sensor = st_lis2du12_get_sensor_from_id(hw, ST_LIS2DU12_ID_TAP);
	sensor->tap_en = 0x7;
	sensor = st_lis2du12_get_sensor_from_id(hw, ST_LIS2DU12_ID_TAP_TAP);
	sensor->tap_en = 0x7;

	/* enable wake-up event on all axes */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_CTRL1_ADDR,
				 ST_LIS2DU12_WU_EN_MASK,
				 FIELD_PREP(ST_LIS2DU12_WU_EN_MASK, 0x7));
	if (err < 0)
		return err;

	sensor = st_lis2du12_get_sensor_from_id(hw, ST_LIS2DU12_ID_WU);
	sensor->wk_en = 0x7;

	/* double tap event detection configuration */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_INT_DUR_ADDR,
				 ST_LIS2DU12_SHOCK_MASK,
				 FIELD_PREP(ST_LIS2DU12_SHOCK_MASK, 0x3));
	if (err < 0)
		return err;

	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_INT_DUR_ADDR,
				 ST_LIS2DU12_QUIET_MASK,
				 FIELD_PREP(ST_LIS2DU12_QUIET_MASK, 0x3));
	if (err < 0)
		return err;

	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_INT_DUR_ADDR,
				 ST_LIS2DU12_LATENCY_MASK,
				 FIELD_PREP(ST_LIS2DU12_LATENCY_MASK, 0x7));
	if (err < 0)
		return err;

	/* configure default threshold for tap event recognition */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_TAP_THS_X_ADDR,
				 ST_LIS2DU12_TAP_THS_X_MASK,
				 FIELD_PREP(ST_LIS2DU12_TAP_THS_X_MASK, 0x9));
	if (err < 0)
		return err;

	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_TAP_THS_Y_ADDR,
				 ST_LIS2DU12_TAP_THS_Y_MASK,
				 FIELD_PREP(ST_LIS2DU12_TAP_THS_Y_MASK, 0x9));
	if (err < 0)
		return err;

	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_TAP_THS_Z_ADDR,
				 ST_LIS2DU12_TAP_THS_Z_MASK,
				 FIELD_PREP(ST_LIS2DU12_TAP_THS_Z_MASK, 0x9));

	if (err < 0)
		return err;

	sensor = st_lis2du12_get_sensor_from_id(hw, ST_LIS2DU12_ID_TAP_TAP);
	sensor->shock = 0x3;
	sensor->quiet = 0x3;
	sensor->latency = 0x7;
	sensor->tap_ths_x = 0x9;
	sensor->tap_ths_y = 0x9;
	sensor->tap_ths_z = 0x9;

	/* setup wake-up threshold */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_WAKE_UP_THS_ADDR,
				 ST_LIS2DU12_WK_THS_MASK,
				 FIELD_PREP(ST_LIS2DU12_WK_THS_MASK, 0x2));
	if (err < 0)
		return err;

	sensor = st_lis2du12_get_sensor_from_id(hw, ST_LIS2DU12_ID_WU);
	sensor->wh_ths = 0x2;

	/* set 6D threshold to 60 degree */
	err = regmap_update_bits(hw->regmap,
				 ST_LIS2DU12_TAP_THS_X_ADDR,
				 ST_LIS2DU12_D6D_THS_MASK,
				 FIELD_PREP(ST_LIS2DU12_D6D_THS_MASK, 0x2));
	if (err < 0)
		return err;

	sensor = st_lis2du12_get_sensor_from_id(hw, ST_LIS2DU12_ID_6D);
	sensor->d6d_ths = 0x2;

	return err;
}

static int st_lis2du12_read_event_config(struct iio_dev *iio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir)
{
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2du12_hw *hw = sensor->hw;

	return !!(hw->enable_mask & BIT(sensor->id));
}

static int st_lis2du12_write_event_config(struct iio_dev *iio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir,
					  int enable)
{
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2du12_hw *hw = sensor->hw;
	bool enable_int;
	u8 md_mask;
	int err;

	switch (sensor->id) {
	case ST_LIS2DU12_ID_WU:
		md_mask = ST_LIS2DU12_INT_WU_MASK;
		break;
	case ST_LIS2DU12_ID_TAP:
		md_mask = ST_LIS2DU12_INT_SINGLE_TAP_MASK;
		err = regmap_update_bits(hw->regmap,
					 ST_LIS2DU12_WAKE_UP_THS_ADDR,
					 ST_LIS2DU12_SINGLE_DOUBLE_TAP_MASK,
					 FIELD_PREP(ST_LIS2DU12_SINGLE_DOUBLE_TAP_MASK, 0));
		if (err < 0)
			return err;
		break;
	case ST_LIS2DU12_ID_TAP_TAP:
		md_mask = ST_LIS2DU12_INT_DOUBLE_TAP_MASK;
		err = regmap_update_bits(hw->regmap,
					 ST_LIS2DU12_WAKE_UP_THS_ADDR,
					 ST_LIS2DU12_SINGLE_DOUBLE_TAP_MASK,
					 FIELD_PREP(ST_LIS2DU12_SINGLE_DOUBLE_TAP_MASK, enable));
		if (err < 0)
			return err;
		break;
	case ST_LIS2DU12_ID_FF:
		md_mask = ST_LIS2DU12_INT_FF_MASK;
		break;
	case ST_LIS2DU12_ID_6D:
		md_mask = ST_LIS2DU12_INT_6D_MASK;
		break;
	case ST_LIS2DU12_ID_ACT:
		md_mask = ST_LIS2DU12_INT_SLEEP_CHANGE_MASK;
		err = regmap_update_bits(hw->regmap,
					 ST_LIS2DU12_WAKE_UP_THS_ADDR,
					 ST_LIS2DU12_SLEEP_ON_MASK,
					 FIELD_PREP(ST_LIS2DU12_SLEEP_ON_MASK, enable));
		if (err < 0)
			return err;
		break;
	default:
		return -EINVAL;
	}

	err = regmap_update_bits(hw->regmap, hw->md_reg, md_mask,
				 ST_LIS2DU12_SHIFT_VAL(enable, md_mask));
	if (err < 0)
		return err;

	err = st_lis2du12_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	/* enable global interrupt pin */
	enable_int = st_lis2du12_interrupts_enabled(hw);
	if ((enable_int && enable == 1) || (!enable_int && enable == 0))
		err = regmap_update_bits(hw->regmap,
					 ST_LIS2DU12_INTERRUPT_CFG_ADDR,
					 ST_LIS2DU12_INTERRUPTS_ENABLE_MASK,
					 FIELD_PREP(ST_LIS2DU12_INTERRUPTS_ENABLE_MASK, enable));

	return err;
}

static ssize_t st_lis2du12_get_4d(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);

	return sprintf(buf, "%d\n", !!sensor->hw->fourd_enabled);
}

static ssize_t st_lis2du12_set_4d(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);
	int err, val;

	mutex_lock(&iio_dev->mlock);
	if (iio_buffer_enabled(iio_dev)) {
		err = -EBUSY;

		goto unlock;
	}

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		goto unlock;

	val = val >= 1 ? 1 : 0;

	/* set Enable */
	err = regmap_update_bits(sensor->hw->regmap, ST_LIS2DU12_TAP_THS_X_ADDR,
				 ST_LIS2DU12_D4D_EN_MASK,
				 FIELD_PREP(ST_LIS2DU12_D4D_EN_MASK, val));
	if (err < 0)
		goto unlock;

	sensor->hw->fourd_enabled = val;

unlock:
	mutex_unlock(&iio_dev->mlock);

	return err < 0 ? err : size;
}

static int st_lis2du12_embebbed_config(struct st_lis2du12_sensor *sensor,
				       enum st_lis2du12_attr_id id,
				       u8 *reg, u8 *mask, u8 **attr)
{
	int err = 0;

	switch (id) {
	case ST_LIS2DU12_WK_THS_ATTR_ID:
		*attr = &sensor->wh_ths;
		*reg = ST_LIS2DU12_WAKE_UP_THS_ADDR;
		*mask = ST_LIS2DU12_WK_THS_MASK;
		break;
	case ST_LIS2DU12_WK_DUR_ATTR_ID:
		*attr = &sensor->wh_dur;
		*reg = ST_LIS2DU12_WAKE_UP_DUR_ADDR;
		*mask = ST_LIS2DU12_WAKE_DUR_MASK;
		break;
	case ST_LIS2DU12_FF_THS_ATTR_ID:
		*attr = &sensor->ff_ths;
		*reg = ST_LIS2DU12_FREE_FALL_ADDR;
		*mask = ST_LIS2DU12_FF_THS_MASK;
		break;
	case ST_LIS2DU12_FF_DUR_ATTR_ID:
		*attr = &sensor->ff_dur;
		*reg = ST_LIS2DU12_FREE_FALL_ADDR;
		*mask = ST_LIS2DU12_FF_DUR_MASK;
		break;
	case ST_LIS2DU12_6D_THS_ATTR_ID:
		*attr = &sensor->d6d_ths;
		*reg = ST_LIS2DU12_TAP_THS_X_ADDR;
		*mask = ST_LIS2DU12_D6D_THS_MASK;
		break;
	case ST_LIS2DU12_LATENCY_ATTR_ID:
		*attr = &sensor->latency;
		*reg = ST_LIS2DU12_INT_DUR_ADDR;
		*mask = ST_LIS2DU12_LATENCY_MASK;
		break;
	case ST_LIS2DU12_QUIET_ATTR_ID:
		*attr = &sensor->quiet;
		*reg = ST_LIS2DU12_INT_DUR_ADDR;
		*mask = ST_LIS2DU12_QUIET_MASK;
		break;
	case ST_LIS2DU12_SHOCK_ATTR_ID:
		*attr = &sensor->shock;
		*reg = ST_LIS2DU12_INT_DUR_ADDR;
		*mask = ST_LIS2DU12_SHOCK_MASK;
		break;
	case ST_LIS2DU12_TAP_PRIORITY_ATTR_ID:
		*attr = &sensor->tap_priority;
		*reg = ST_LIS2DU12_TAP_THS_Y_ADDR;
		*mask = ST_LIS2DU12_TAP_PRIORITY_MASK;
		break;
	case ST_LIS2DU12_SLEEP_DUR_ATTR_ID:
		*attr = &sensor->sleep_dur;
		*reg = ST_LIS2DU12_WAKE_UP_DUR_ADDR;
		*mask = ST_LIS2DU12_SLEEP_DUR_MASK;
		break;
	case ST_LIS2DU12_TAP_THRESHOLD_X_ATTR_ID:
		*attr = &sensor->tap_ths_x;
		*reg = ST_LIS2DU12_TAP_THS_X_ADDR;
		*mask = ST_LIS2DU12_TAP_THS_X_MASK;
		break;
	case ST_LIS2DU12_TAP_THRESHOLD_Y_ATTR_ID:
		*attr = &sensor->tap_ths_y;
		*reg = ST_LIS2DU12_TAP_THS_Y_ADDR;
		*mask = ST_LIS2DU12_TAP_THS_Y_MASK;
		break;
	case ST_LIS2DU12_TAP_THRESHOLD_Z_ATTR_ID:
		*attr = &sensor->tap_ths_z;
		*reg = ST_LIS2DU12_TAP_THS_Z_ADDR;
		*mask = ST_LIS2DU12_TAP_THS_Z_MASK;
		break;
	case ST_LIS2DU12_TAP_ENABLE_ATTR_ID:
		*attr = &sensor->tap_en;
		*reg = ST_LIS2DU12_TAP_THS_Z_ADDR;
		*mask = ST_LIS2DU12_TAP_EN_MASK;
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static ssize_t
st_lis2du12_embebbed_threshold_get(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct st_lis2du12_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	enum st_lis2du12_attr_id id;
	u8 reg, mask, *val;
	int err;

	id = (enum st_lis2du12_attr_id)this_attr->address;
	err = st_lis2du12_embebbed_config(sensor, id, &reg, &mask, &val);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", *val);
}

ssize_t st_lis2du12_embebbed_threshold_set(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct st_lis2du12_sensor *sensor = iio_priv(iio_dev);
	enum st_lis2du12_attr_id id;
	u8 reg, mask, *val;
	int err, data;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	err = kstrtoint(buf, 10, &data);
	if (err < 0)
		goto out;

	id = (enum st_lis2du12_attr_id)this_attr->address;
	err = st_lis2du12_embebbed_config(sensor, id, &reg, &mask, &val);
	if (err < 0)
		goto out;

	err = regmap_update_bits(sensor->hw->regmap, reg, mask,
				 ST_LIS2DU12_SHIFT_VAL(data, mask));
	if (err < 0)
		goto out;

	if (id == ST_LIS2DU12_FF_DUR_ATTR_ID) {
		/* free fall duration split on two registers */
		err = regmap_update_bits(sensor->hw->regmap,
					 ST_LIS2DU12_WAKE_UP_DUR_ADDR,
					 ST_LIS2DU12_FF_DUR5_MASK,
					 ST_LIS2DU12_SHIFT_VAL((data >> 5 & 0x01),
						     ST_LIS2DU12_FF_DUR5_MASK));
		if (err < 0)
			goto out;
	}

	*val = (u8)data;

out:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

static IIO_DEVICE_ATTR(enable_4d, 0644, st_lis2du12_get_4d,
		       st_lis2du12_set_4d, 0);
static IIO_DEVICE_ATTR(wakeup_threshold, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_WK_THS_ATTR_ID);
static IIO_DEVICE_ATTR(wakeup_duration, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_WK_DUR_ATTR_ID);
static IIO_DEVICE_ATTR(freefall_threshold, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_FF_THS_ATTR_ID);
static IIO_DEVICE_ATTR(freefall_duration, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_FF_DUR_ATTR_ID);
static IIO_DEVICE_ATTR(sixd_threshold, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_6D_THS_ATTR_ID);
static IIO_DEVICE_ATTR(latency_threshold, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_LATENCY_ATTR_ID);
static IIO_DEVICE_ATTR(quiet_threshold, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_QUIET_ATTR_ID);
static IIO_DEVICE_ATTR(shock_threshold, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_SHOCK_ATTR_ID);
static IIO_DEVICE_ATTR(tap_priority, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_TAP_PRIORITY_ATTR_ID);
static IIO_DEVICE_ATTR(tap_thr_x, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_TAP_THRESHOLD_X_ATTR_ID);
static IIO_DEVICE_ATTR(tap_thr_y, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_TAP_THRESHOLD_Y_ATTR_ID);
static IIO_DEVICE_ATTR(tap_thr_z, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_TAP_THRESHOLD_Z_ATTR_ID);
static IIO_DEVICE_ATTR(tap_enable, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_TAP_ENABLE_ATTR_ID);
static IIO_DEVICE_ATTR(sleep_dur, 0644,
		       st_lis2du12_embebbed_threshold_get,
		       st_lis2du12_embebbed_threshold_set,
		       ST_LIS2DU12_SLEEP_DUR_ATTR_ID);

static struct attribute *st_lis2du12_wu_attributes[] = {
	&iio_dev_attr_wakeup_threshold.dev_attr.attr,
	&iio_dev_attr_wakeup_duration.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2du12_wu_attribute_group = {
	.attrs = st_lis2du12_wu_attributes,
};

static const struct iio_info st_lis2du12_wu_info = {
	.attrs = &st_lis2du12_wu_attribute_group,
	.read_event_config = st_lis2du12_read_event_config,
	.write_event_config = st_lis2du12_write_event_config,
};

static struct attribute *st_lis2du12_tap_tap_attributes[] = {
	&iio_dev_attr_latency_threshold.dev_attr.attr,
	&iio_dev_attr_quiet_threshold.dev_attr.attr,
	&iio_dev_attr_shock_threshold.dev_attr.attr,
	&iio_dev_attr_tap_priority.dev_attr.attr,
	&iio_dev_attr_tap_thr_x.dev_attr.attr,
	&iio_dev_attr_tap_thr_y.dev_attr.attr,
	&iio_dev_attr_tap_thr_z.dev_attr.attr,
	&iio_dev_attr_tap_enable.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2du12_tap_tap_attribute_group = {
	.attrs = st_lis2du12_tap_tap_attributes,
};

static const struct iio_info st_lis2du12_tap_tap_info = {
	.attrs = &st_lis2du12_tap_tap_attribute_group,
	.read_event_config = st_lis2du12_read_event_config,
	.write_event_config = st_lis2du12_write_event_config,
};

static struct attribute *st_lis2du12_tap_attributes[] = {
	&iio_dev_attr_quiet_threshold.dev_attr.attr,
	&iio_dev_attr_shock_threshold.dev_attr.attr,
	&iio_dev_attr_tap_priority.dev_attr.attr,
	&iio_dev_attr_tap_thr_x.dev_attr.attr,
	&iio_dev_attr_tap_thr_y.dev_attr.attr,
	&iio_dev_attr_tap_thr_z.dev_attr.attr,
	&iio_dev_attr_tap_enable.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2du12_tap_attribute_group = {
	.attrs = st_lis2du12_tap_attributes,
};

static const struct iio_info st_lis2du12_tap_info = {
	.attrs = &st_lis2du12_tap_attribute_group,
	.read_event_config = st_lis2du12_read_event_config,
	.write_event_config = st_lis2du12_write_event_config,
};

static struct attribute *st_lis2du12_ff_attributes[] = {
	&iio_dev_attr_freefall_threshold.dev_attr.attr,
	&iio_dev_attr_freefall_duration.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2du12_ff_attribute_group = {
	.attrs = st_lis2du12_ff_attributes,
};

static const struct iio_info st_lis2du12_ff_info = {
	.attrs = &st_lis2du12_ff_attribute_group,
	.read_event_config = st_lis2du12_read_event_config,
	.write_event_config = st_lis2du12_write_event_config,
};

static struct attribute *st_lis2du12_6d_attributes[] = {
	&iio_dev_attr_sixd_threshold.dev_attr.attr,
	&iio_dev_attr_enable_4d.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2du12_6d_attribute_group = {
	.attrs = st_lis2du12_6d_attributes,
};

static const struct iio_info st_lis2du12_6d_info = {
	.attrs = &st_lis2du12_6d_attribute_group,
	.read_event_config = st_lis2du12_read_event_config,
	.write_event_config = st_lis2du12_write_event_config,
};

static struct attribute *st_lis2du12_act_attributes[] = {
	&iio_dev_attr_sleep_dur.dev_attr.attr,
	&iio_dev_attr_wakeup_threshold.dev_attr.attr,
	&iio_dev_attr_wakeup_duration.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2du12_act_attribute_group = {
	.attrs = st_lis2du12_act_attributes,
};

static const struct iio_info st_lis2du12_act_info = {
	.attrs = &st_lis2du12_act_attribute_group,
	.read_event_config = st_lis2du12_read_event_config,
	.write_event_config = st_lis2du12_write_event_config,
};

static const unsigned long st_lis2du12_event_avail_scan_masks[] = {
	BIT(0), 0x0
};

static struct
iio_dev *st_lis2du12_alloc_embfunc_iiodev(struct st_lis2du12_hw *hw,
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
	case ST_LIS2DU12_ID_WU:
		iio_dev->channels = st_lis2du12_wu_channels;
		iio_dev->num_channels =
				ARRAY_SIZE(st_lis2du12_wu_channels);
		iio_dev->name = ST_LIS2DU12_DEV_NAME "_wk";
		iio_dev->info = &st_lis2du12_wu_info;
		iio_dev->available_scan_masks =
				st_lis2du12_event_avail_scan_masks;

		sensor->odr = hw->odr_table[9].hz;
		break;
	case ST_LIS2DU12_ID_TAP_TAP:
		iio_dev->channels = st_lis2du12_tap_tap_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lis2du12_tap_tap_channels);
		iio_dev->name = ST_LIS2DU12_DEV_NAME "_tap_tap";
		iio_dev->info = &st_lis2du12_tap_tap_info;
		iio_dev->available_scan_masks =
				st_lis2du12_event_avail_scan_masks;

		sensor->odr = hw->odr_table[9].hz;
		break;
	case ST_LIS2DU12_ID_TAP:
		iio_dev->channels = st_lis2du12_tap_channels;
		iio_dev->num_channels =
				ARRAY_SIZE(st_lis2du12_tap_channels);
		iio_dev->name = ST_LIS2DU12_DEV_NAME "_tap";
		iio_dev->info = &st_lis2du12_tap_info;
		iio_dev->available_scan_masks =
				st_lis2du12_event_avail_scan_masks;

		sensor->odr = hw->odr_table[9].hz;
		break;
	case ST_LIS2DU12_ID_FF:
		iio_dev->channels = st_lis2du12_ff_channels;
		iio_dev->num_channels =
				ARRAY_SIZE(st_lis2du12_ff_channels);
		iio_dev->name = ST_LIS2DU12_DEV_NAME "_ff";
		iio_dev->info = &st_lis2du12_ff_info;
		iio_dev->available_scan_masks =
				st_lis2du12_event_avail_scan_masks;

		sensor->odr = hw->odr_table[8].hz;
		break;
	case ST_LIS2DU12_ID_6D:
		iio_dev->channels = st_lis2du12_6d_channels;
		iio_dev->num_channels =
				ARRAY_SIZE(st_lis2du12_6d_channels);
		iio_dev->name = ST_LIS2DU12_DEV_NAME "_6d";
		iio_dev->info = &st_lis2du12_6d_info;
		iio_dev->available_scan_masks =
				st_lis2du12_event_avail_scan_masks;

		sensor->odr = hw->odr_table[8].hz;
		break;
	case ST_LIS2DU12_ID_ACT:
		iio_dev->channels = st_lis2du12_act_channels;
		iio_dev->num_channels =
				ARRAY_SIZE(st_lis2du12_act_channels);
		iio_dev->name = ST_LIS2DU12_DEV_NAME "_act";
		iio_dev->info = &st_lis2du12_act_info;
		iio_dev->available_scan_masks =
				st_lis2du12_event_avail_scan_masks;

		sensor->odr = hw->odr_table[8].hz;
		break;
	default:
		return NULL;
	}

	return iio_dev;
}

int st_lis2du12_handler_embfunc_thread(struct st_lis2du12_hw *hw)
{
	bool reset_int = false;
	int all_int_source;
	s64 code;
	int err;

	if (!st_lis2du12_interrupts_enabled(hw))
		return IRQ_HANDLED;

	if (hw->enable_mask & (BIT(ST_LIS2DU12_ID_TAP) |
			       BIT(ST_LIS2DU12_ID_TAP_TAP))) {
		struct iio_dev *iio_dev;
		enum iio_chan_type type;
		int tap_src;

		err = regmap_read(hw->regmap, ST_LIS2DU12_TAP_SRC_ADDR,
				  &tap_src);
		if (err < 0)
			return IRQ_HANDLED;

		if ((hw->enable_mask & BIT(ST_LIS2DU12_ID_TAP_TAP)) &&
		    (tap_src & ST_LIS2DU12_DOUBLE_TAP_IA_MASK)) {
			iio_dev = hw->iio_devs[ST_LIS2DU12_ID_TAP_TAP];
			type = STM_IIO_TAP_TAP;
			code = IIO_UNMOD_EVENT_CODE(type, -1,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING);
			reset_int = true;
			iio_push_event(iio_dev, code,
				       st_lis2du12_get_timestamp(hw));
		}

		if ((hw->enable_mask & BIT(ST_LIS2DU12_ID_TAP)) &&
		    (tap_src & ST_LIS2DU12_SINGLE_TAP_IA_MASK)) {
			iio_dev = hw->iio_devs[ST_LIS2DU12_ID_TAP];
			type = STM_IIO_TAP;
			code = IIO_UNMOD_EVENT_CODE(type, -1,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING);
			reset_int = true;
			iio_push_event(iio_dev, code,
				       st_lis2du12_get_timestamp(hw));
		}
	}

	if (hw->enable_mask & BIT(ST_LIS2DU12_ID_WU)) {
		struct iio_dev *iio_dev;
		enum iio_chan_type type;
		int wu_src;

		err = regmap_read(hw->regmap,
				  ST_LIS2DU12_WAKE_UP_SRC_ADDR,
				  &wu_src);
		if (err < 0)
			return IRQ_HANDLED;

		if (wu_src & ST_LIS2DU12_WU_IA_MASK) {
			wu_src &= ST_LIS2DU12_WU_MASK;
			iio_dev = hw->iio_devs[ST_LIS2DU12_ID_WU];
			/* use STM_IIO_GESTURE event type for custom events */
			type = STM_IIO_GESTURE;
			code = IIO_UNMOD_EVENT_CODE(type,
					 (wu_src & ST_LIS2DU12_WU_MASK),
					 IIO_EV_TYPE_THRESH,
					 IIO_EV_DIR_RISING);
			reset_int = true;
			iio_push_event(iio_dev, code,
				       st_lis2du12_get_timestamp(hw));
		}
	}

	if (hw->enable_mask & BIT(ST_LIS2DU12_ID_FF)) {
		struct iio_dev *iio_dev;
		enum iio_chan_type type;
		int wu_src;

		err = regmap_read(hw->regmap,
				  ST_LIS2DU12_WAKE_UP_SRC_ADDR,
				  &wu_src);
		if (err < 0)
			return IRQ_HANDLED;

		if (wu_src & ST_LIS2DU12_FF_IA_MASK) {
			iio_dev = hw->iio_devs[ST_LIS2DU12_ID_FF];
			/* use STM_IIO_GESTURE event type for custom events */
			type = STM_IIO_GESTURE;
			code = IIO_UNMOD_EVENT_CODE(type, 1,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING);
			reset_int = true;
			iio_push_event(iio_dev, code,
				       st_lis2du12_get_timestamp(hw));
		}
	}

	if (hw->enable_mask & BIT(ST_LIS2DU12_ID_6D)) {
		struct iio_dev *iio_dev;
		enum iio_chan_type type;
		int sixd_src;

		err = regmap_read(hw->regmap, ST_LIS2DU12_SIXD_SRC_ADDR,
				  &sixd_src);
		if (err < 0)
			return IRQ_HANDLED;

		if (sixd_src & ST_LIS2DU12_D6D_IA_MASK) {
			iio_dev = hw->iio_devs[ST_LIS2DU12_ID_6D];
			/* use IIO_GESTURE event type for custom events */
			type = STM_IIO_GESTURE;
			code = IIO_UNMOD_EVENT_CODE(type,
					sixd_src & ST_LIS2DU12_OVERTHRESHOLD_MASK,
					IIO_EV_TYPE_THRESH,
					IIO_EV_DIR_RISING);
			reset_int = true;
			iio_push_event(iio_dev, code,
				       st_lis2du12_get_timestamp(hw));
		}
	}

	if (hw->enable_mask & BIT(ST_LIS2DU12_ID_ACT)) {
		struct iio_dev *iio_dev;
		enum iio_chan_type type;
		int wu_src;

		err = regmap_read(hw->regmap,
				  ST_LIS2DU12_WAKE_UP_SRC_ADDR,
				  &wu_src);
		if (err < 0)
			return IRQ_HANDLED;

		if (wu_src & ST_LIS2DU12_SLEEP_STATE_MASK) {
			iio_dev = hw->iio_devs[ST_LIS2DU12_ID_ACT];
			/* use IIO_GESTURE event type for custom events */
			type = STM_IIO_GESTURE;
			code = IIO_UNMOD_EVENT_CODE(type,
				  wu_src & ST_LIS2DU12_SLEEP_STATE_MASK,
				  IIO_EV_TYPE_THRESH,
				  IIO_EV_DIR_RISING);
			reset_int = true;
			iio_push_event(iio_dev, code,
				st_lis2du12_get_timestamp(hw));
		}
	}

	if (reset_int) {
		err = regmap_read(hw->regmap,
				  ST_LIS2DU12_ALL_INT_SRC_ADDR,
				  &all_int_source);
	}

	return IRQ_HANDLED;
}

int st_lis2du12_embedded_function_probe(struct st_lis2du12_hw *hw)
{
	int err, i, id;

	for (i = 0;
	     i < ARRAY_SIZE(st_lis2du12_embedded_function_sensor_list);
	     i++) {
		id = st_lis2du12_embedded_function_sensor_list[i];

		hw->iio_devs[id] = st_lis2du12_alloc_embfunc_iiodev(hw, id);
		if (!hw->iio_devs[id])
			return -ENOMEM;
	}

	err = st_lis2du12_embedded_config(hw);

	return err < 0 ? err : 0;
}
