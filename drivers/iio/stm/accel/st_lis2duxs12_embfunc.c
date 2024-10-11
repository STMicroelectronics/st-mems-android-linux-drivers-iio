// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lis2duxs12 embedded function sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2022 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/irq.h>
#include <linux/module.h>

#include "st_lis2duxs12.h"

static const struct
iio_chan_spec st_lis2duxs12_step_counter_channels[] = {
	{
		.type = STM_IIO_STEP_COUNTER,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	ST_LIS2DUXS12_EVENT_CHANNEL(STM_IIO_STEP_COUNTER, flush),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct
iio_chan_spec st_lis2duxs12_step_detector_channels[] = {
	ST_LIS2DUXS12_EVENT_CHANNEL(IIO_STEPS, thr),
};

static const struct
iio_chan_spec st_lis2duxs12_sign_motion_channels[] = {
	ST_LIS2DUXS12_EVENT_CHANNEL(STM_IIO_SIGN_MOTION, thr),
};

static const struct iio_chan_spec st_lis2duxs12_tilt_channels[] = {
	ST_LIS2DUXS12_EVENT_CHANNEL(STM_IIO_TILT, thr),
};

static const unsigned long st_lis2duxs12_emb_available_scan_masks[] = {
	BIT(0), 0x0
};

static int
st_lis2duxs12_read_embfunc_event_config(struct iio_dev *iio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir)
{
	struct st_lis2duxs12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2duxs12_hw *hw = sensor->hw;

	return !!(hw->enable_mask & BIT(sensor->id));
}

static int
st_lis2duxs12_write_embfunc_event_config(struct iio_dev *iio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir,
					 int state)
{
	struct st_lis2duxs12_sensor *sensor = iio_priv(iio_dev);
	int err;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	err = st_lis2duxs12_embfunc_sensor_set_enable(sensor, state);
	iio_device_release_direct_mode(iio_dev);

	return err;
}

static ssize_t
st_lis2duxs12_sysfs_reset_step_counter(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	int err;

	err = st_lis2duxs12_reset_step_counter(iio_dev);

	return err < 0 ? err : size;
}

static IIO_DEVICE_ATTR(reset_counter, 0200, NULL,
		       st_lis2duxs12_sysfs_reset_step_counter, 0);

static IIO_DEVICE_ATTR(hwfifo_stepc_watermark_max, 0444,
		       st_lis2duxs12_get_max_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_stepc_flush, 0200, NULL,
		       st_lis2duxs12_flush_fifo, 0);
static IIO_DEVICE_ATTR(hwfifo_stepc_watermark, 0644,
		       st_lis2duxs12_get_watermark,
		       st_lis2duxs12_set_watermark, 0);

static struct attribute *st_lis2duxs12_sc_attributes[] = {
	&iio_dev_attr_hwfifo_stepc_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_stepc_watermark.dev_attr.attr,
	&iio_dev_attr_reset_counter.dev_attr.attr,
	&iio_dev_attr_hwfifo_stepc_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2duxs12_sc_attribute_group = {
	.attrs = st_lis2duxs12_sc_attributes,
};

static const struct iio_info st_lis2duxs12_sc_info = {
	.attrs = &st_lis2duxs12_sc_attribute_group,
};

static struct attribute *st_lis2duxs12_sd_attributes[] = {
	NULL,
};

static const struct attribute_group st_lis2duxs12_sd_attribute_group = {
	.attrs = st_lis2duxs12_sd_attributes,
};

static const struct iio_info st_lis2duxs12_sd_info = {
	.attrs = &st_lis2duxs12_sd_attribute_group,
	.read_event_config = st_lis2duxs12_read_embfunc_event_config,
	.write_event_config = st_lis2duxs12_write_embfunc_event_config,
};

static struct attribute *st_lis2duxs12_sm_attributes[] = {
	NULL,
};

static const struct attribute_group st_lis2duxs12_sm_attribute_group = {
	.attrs = st_lis2duxs12_sm_attributes,
};

static const struct iio_info st_lis2duxs12_sm_info = {
	.attrs = &st_lis2duxs12_sm_attribute_group,
	.read_event_config = st_lis2duxs12_read_event_config,
	.write_event_config = st_lis2duxs12_write_event_config,
};

static struct attribute *st_lis2duxs12_tilt_attributes[] = {
	NULL,
};

static const struct attribute_group st_lis2duxs12_tilt_attribute_group = {
	.attrs = st_lis2duxs12_tilt_attributes,
};

static const struct iio_info st_lis2duxs12_tilt_info = {
	.attrs = &st_lis2duxs12_tilt_attribute_group,
	.read_event_config = st_lis2duxs12_read_event_config,
	.write_event_config = st_lis2duxs12_write_event_config,
};

static int
st_lis2duxs12_ef_pg1_sensor_set_enable(struct st_lis2duxs12_sensor *sensor,
				       u8 mask, u8 irq_mask, bool enable)
{
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int err;

	err = st_lis2duxs12_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, 1);
	if (err < 0)
		goto unlock;

	err = __st_lis2duxs12_write_with_mask(hw,
					ST_LIS2DUXS12_EMB_FUNC_EN_A_ADDR,
					mask, enable);
	if (err < 0)
		goto reset_page;

	err = __st_lis2duxs12_write_with_mask(hw, hw->emb_int_reg,
					      irq_mask, enable);

reset_page:
	st_lis2duxs12_set_emb_access(hw, 0);

	if (err < 0)
		goto unlock;

	if (((hw->enable_mask & ST_LIS2DUXS12_EMB_FUNC_ENABLED) && enable) ||
	    (!(hw->enable_mask & ST_LIS2DUXS12_EMB_FUNC_ENABLED) && !enable)) {
		err = __st_lis2duxs12_write_with_mask(hw, hw->md_int_reg,
					ST_LIS2DUXS12_INT_EMB_FUNC_MASK,
					enable);
	}

unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Enable Embedded Function sensor [EMB_FUN]
 *
 * @param  sensor: ST ACC sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
int st_lis2duxs12_embfunc_sensor_set_enable(struct st_lis2duxs12_sensor *sensor,
					    bool enable)
{
	int err;

	switch (sensor->id) {
	case ST_LIS2DUXS12_ID_STEP_DETECTOR:
		err = st_lis2duxs12_ef_pg1_sensor_set_enable(sensor,
				ST_LIS2DUXS12_PEDO_EN_MASK,
				ST_LIS2DUXS12_INT_STEP_DETECTOR_MASK,
				enable);
		break;
	case ST_LIS2DUXS12_ID_SIGN_MOTION:
		err = st_lis2duxs12_ef_pg1_sensor_set_enable(sensor,
				ST_LIS2DUXS12_SIGN_MOTION_EN_MASK,
				ST_LIS2DUXS12_INT_SIG_MOT_MASK,
				enable);
		break;
	case ST_LIS2DUXS12_ID_TILT:
		err = st_lis2duxs12_ef_pg1_sensor_set_enable(sensor,
					ST_LIS2DUXS12_TILT_EN_MASK,
					ST_LIS2DUXS12_INT_TILT_MASK,
					enable);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

int st_lis2duxs12_step_counter_set_enable(struct st_lis2duxs12_sensor *sensor,
					  bool enable)
{
	struct st_lis2duxs12_hw *hw = sensor->hw;
	bool run_enable = false;
	int err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, 1);
	if (err < 0)
		goto unlock;

	err = __st_lis2duxs12_write_with_mask(hw,
					ST_LIS2DUXS12_EMB_FUNC_EN_A_ADDR,
					ST_LIS2DUXS12_PEDO_EN_MASK,
					enable);
	if (err < 0)
		goto reset_page;

	err = __st_lis2duxs12_write_with_mask(hw,
				ST_LIS2DUXS12_EMB_FUNC_FIFO_EN_ADDR,
				ST_LIS2DUXS12_STEP_COUNTER_FIFO_EN_MASK,
				enable);
	if (err < 0)
		goto reset_page;

	run_enable = true;

reset_page:
	st_lis2duxs12_set_emb_access(hw, 0);

unlock:
	mutex_unlock(&hw->page_lock);

	if (run_enable)
		err = st_lis2duxs12_sensor_set_enable(sensor, enable);

	return err;
}

int st_lis2duxs12_reset_step_counter(struct iio_dev *iio_dev)
{
	struct st_lis2duxs12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2duxs12_hw *hw = sensor->hw;
	__le16 data;
	int err;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	err = st_lis2duxs12_step_counter_set_enable(sensor, true);
	if (err < 0)
		goto unlock_iio_dev;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, 1);
	if (err < 0)
		goto unlock_page;

	err = __st_lis2duxs12_write_with_mask(hw,
				ST_LIS2DUXS12_EMB_FUNC_SRC_ADDR,
				ST_LIS2DUXS12_PEDO_RST_STEP_MASK, 1);
	if (err < 0)
		goto reset_page;

	msleep(100);

	regmap_bulk_read(hw->regmap, ST_LIS2DUXS12_STEP_COUNTER_L_ADDR,
			 (u8 *)&data, sizeof(data));

reset_page:
	st_lis2duxs12_set_emb_access(hw, 0);

unlock_page:
	mutex_unlock(&hw->page_lock);

	err = st_lis2duxs12_step_counter_set_enable(sensor, false);

unlock_iio_dev:
	iio_device_release_direct_mode(iio_dev);

	return err;
}

static struct
iio_dev *st_lis2duxs12_alloc_emb_func_iiodev(struct st_lis2duxs12_hw *hw,
					     enum st_lis2duxs12_sensor_id id)
{
	struct st_lis2duxs12_sensor *sensor;
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

	switch (id) {
	case ST_LIS2DUXS12_ID_STEP_COUNTER:
		iio_dev->channels = st_lis2duxs12_step_counter_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lis2duxs12_step_counter_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			 "%s_step_c", hw->settings->id.name);
		iio_dev->info = &st_lis2duxs12_sc_info;
		iio_dev->available_scan_masks =
				st_lis2duxs12_emb_available_scan_masks;

		/* request ODR @50 Hz to works properly */
		sensor->max_watermark = 1;
		sensor->odr = 50;
		sensor->uodr = 0;
		break;
	case ST_LIS2DUXS12_ID_STEP_DETECTOR:
		iio_dev->channels = st_lis2duxs12_step_detector_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lis2duxs12_step_detector_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			 "%s_step_d", hw->settings->id.name);
		iio_dev->info = &st_lis2duxs12_sd_info;
		iio_dev->available_scan_masks =
				st_lis2duxs12_emb_available_scan_masks;

		/* request ODR @50 Hz to works properly */
		sensor->odr = 50;
		sensor->uodr = 0;
		break;
	case ST_LIS2DUXS12_ID_SIGN_MOTION:
		iio_dev->channels = st_lis2duxs12_sign_motion_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lis2duxs12_sign_motion_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			 "%s_sign_motion", hw->settings->id.name);
		iio_dev->info = &st_lis2duxs12_sm_info;
		iio_dev->available_scan_masks =
				st_lis2duxs12_emb_available_scan_masks;

		/* request ODR @50 Hz to works properly */
		sensor->odr = 50;
		sensor->uodr = 0;
		break;
	case ST_LIS2DUXS12_ID_TILT:
		iio_dev->channels = st_lis2duxs12_tilt_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lis2duxs12_tilt_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			 "%s_tilt", hw->settings->id.name);
		iio_dev->info = &st_lis2duxs12_tilt_info;
		iio_dev->available_scan_masks =
				st_lis2duxs12_emb_available_scan_masks;

		/* request ODR @50 Hz to works properly */
		sensor->odr = 50;
		sensor->uodr = 0;
		break;
	default:
		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

int st_lis2duxs12_embedded_function_handler(struct st_lis2duxs12_hw *hw)
{
	if (hw->enable_mask & (BIT(ST_LIS2DUXS12_ID_STEP_DETECTOR) |
			       BIT(ST_LIS2DUXS12_ID_TILT) |
			       BIT(ST_LIS2DUXS12_ID_SIGN_MOTION))) {
		struct iio_dev *iio_dev;
		u8 status;
		s64 event;
		int err;

		err = st_lis2duxs12_read_locked(hw,
			    ST_LIS2DUXS12_EMB_FUNC_STATUS_MAINPAGE_ADDR,
			    &status, sizeof(status));
		if (err < 0)
			return IRQ_HANDLED;

		/* embedded function sensors */
		if (status & ST_LIS2DUXS12_IS_STEP_DET_MASK) {
			iio_dev = hw->iio_devs[ST_LIS2DUXS12_ID_STEP_DETECTOR];
			event = IIO_UNMOD_EVENT_CODE(IIO_STEPS, -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       iio_get_time_ns(iio_dev));
		}

		if (status & ST_LIS2DUXS12_IS_SIGMOT_MASK) {
			iio_dev = hw->iio_devs[ST_LIS2DUXS12_ID_SIGN_MOTION];
			event = IIO_UNMOD_EVENT_CODE(STM_IIO_SIGN_MOTION, -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       iio_get_time_ns(iio_dev));
		}

		if (status & ST_LIS2DUXS12_IS_TILT_MASK) {
			iio_dev = hw->iio_devs[ST_LIS2DUXS12_ID_TILT];
			event = IIO_UNMOD_EVENT_CODE(STM_IIO_TILT, -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       iio_get_time_ns(iio_dev));
		}
	}

	return IRQ_HANDLED;
}

static int st_lis2duxs12_embedded_function_init(struct st_lis2duxs12_hw *hw)
{
	int err;

	err = st_lis2duxs12_update_bits_locked(hw,
					ST_LIS2DUXS12_CTRL4_ADDR,
					ST_LIS2DUXS12_EMB_FUNC_EN_MASK,
					1);
	if (err < 0)
		return err;

	usleep_range(5000, 6000);

	/* enable latched interrupts */
	return st_lis2duxs12_update_page_bits_locked(hw,
						ST_LIS2DUXS12_PAGE_RW_ADDR,
						ST_LIS2DUXS12_EMB_FUNC_LIR_MASK,
						1);
}

int st_lis2duxs12_embedded_function_probe(struct st_lis2duxs12_hw *hw)
{
	int err, i, id;

	for (i = 0;
	     i < ARRAY_SIZE(st_lis2duxs12_embedded_function_sensor_list);
	     i++) {
		id = st_lis2duxs12_embedded_function_sensor_list[i];

		hw->iio_devs[id] = st_lis2duxs12_alloc_emb_func_iiodev(hw, id);
		if (!hw->iio_devs[id])
			return -ENOMEM;
	}

	err = st_lis2duxs12_embedded_function_init(hw);

	return err < 0 ? err : 0;
}
