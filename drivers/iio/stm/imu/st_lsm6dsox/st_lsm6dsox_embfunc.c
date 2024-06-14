// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lsm6dsox embedded function sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2021 STMicroelectronics Inc.
 */

#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "st_lsm6dsox.h"

/**
 * Step Counter IIO channels description
 *
 * Step Counter exports to IIO framework the following data channels:
 * Step Counters (16 bit unsigned in little endian)
 * Timestamp (64 bit signed in little endian)
 * Step Counter exports to IIO framework the following event channels:
 * Flush event done
 */
static const struct iio_chan_spec st_lsm6dsox_step_counter_channels[] = {
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
	ST_LSM6DSOX_EVENT_CHANNEL(STM_IIO_STEP_COUNTER, flush),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

/**
 * @brief  Step Detector IIO channels description
 *
 * Step Detector exports to IIO framework the following event channels:
 * Step detection event detection
 */
static const struct iio_chan_spec st_lsm6dsox_step_detector_channels[] = {
	ST_LSM6DSOX_EVENT_CHANNEL(IIO_STEPS, thr),
};

/**
 * Significant Motion IIO channels description
 *
 * Significant Motion exports to IIO framework the following event channels:
 * Significant Motion event detection
 */
static const struct iio_chan_spec st_lsm6dsox_sign_motion_channels[] = {
	ST_LSM6DSOX_EVENT_CHANNEL(STM_IIO_SIGN_MOTION, thr),
};

/**
 * Tilt IIO channels description
 *
 * Tilt exports to IIO framework the following event channels:
 * Tilt event detection
 */
static const struct iio_chan_spec st_lsm6dsox_tilt_channels[] = {
	ST_LSM6DSOX_EVENT_CHANNEL(STM_IIO_TILT, thr),
};

static int
st_lsm6dsox_ef_pg1_sensor_set_enable(struct st_lsm6dsox_sensor *sensor,
				     u8 mask, u8 irq_mask, bool enable)
{
	struct st_lsm6dsox_hw *hw = sensor->hw;
	int err;

	err = st_lsm6dsox_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lsm6dsox_set_page_access(hw, true,
					  ST_LSM6DSOX_REG_FUNC_CFG_MASK);
	if (err < 0)
		goto unlock;

	err = __st_lsm6dsox_write_with_mask(hw,
					    ST_LSM6DSOX_EMB_FUNC_EN_A_ADDR,
					    mask, enable);
	if (err < 0)
		goto reset_page;

	err = __st_lsm6dsox_write_with_mask(hw, hw->embfunc_irq_reg, irq_mask,
					    enable);

reset_page:
	st_lsm6dsox_set_page_access(hw, false, ST_LSM6DSOX_REG_FUNC_CFG_MASK);
unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Enable Embedded Function sensor [EMB_FUN]
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
static int
st_lsm6dsox_embfunc_sensor_set_enable(struct st_lsm6dsox_sensor *sensor,
				      bool enable)
{
	int err;

	switch (sensor->id) {
	case ST_LSM6DSOX_ID_STEP_DETECTOR:
		err = st_lsm6dsox_ef_pg1_sensor_set_enable(sensor,
					ST_LSM6DSOX_PEDO_EN_MASK,
					ST_LSM6DSOX_INT_STEP_DET_MASK,
					enable);
		break;
	case ST_LSM6DSOX_ID_SIGN_MOTION:
		err = st_lsm6dsox_ef_pg1_sensor_set_enable(sensor,
					ST_LSM6DSOX_SIGN_MOTION_EN_MASK,
					ST_LSM6DSOX_INT_SIGMOT_MASK,
					enable);
		break;
	case ST_LSM6DSOX_ID_TILT:
		err = st_lsm6dsox_ef_pg1_sensor_set_enable(sensor,
						ST_LSM6DSOX_TILT_EN_MASK,
						ST_LSM6DSOX_INT_TILT_MASK,
						enable);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

/**
 * Enable Step Counter Sensor [EMB_FUN]
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
int
st_lsm6dsox_step_counter_set_enable(struct st_lsm6dsox_sensor *sensor,
				    bool enable)
{
	struct st_lsm6dsox_hw *hw = sensor->hw;
	int err;

	err = st_lsm6dsox_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lsm6dsox_set_page_access(hw, true,
					  ST_LSM6DSOX_REG_FUNC_CFG_MASK);
	if (err < 0)
		goto unlock;

	err = __st_lsm6dsox_write_with_mask(hw,
					    ST_LSM6DSOX_EMB_FUNC_EN_A_ADDR,
					    ST_LSM6DSOX_PEDO_EN_MASK,
					    enable);
	if (err < 0)
		goto reset_page;

	err = __st_lsm6dsox_write_with_mask(hw,
					    ST_LSM6DSOX_EMB_FUNC_FIFO_CFG_ADDR,
					    ST_LSM6DSOX_PEDO_FIFO_EN_MASK,
					    enable);

reset_page:
	st_lsm6dsox_set_page_access(hw, false, ST_LSM6DSOX_REG_FUNC_CFG_MASK);
unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Reset Step Counter value [EMB_FUN]
 *
 * @param  iio_dev: IIO device
 * @return  < 0 if error, 0 otherwise
 */
static int st_lsm6dsox_reset_step_counter(struct iio_dev *iio_dev)
{
	struct st_lsm6dsox_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dsox_hw *hw = sensor->hw;
	u16 prev_val, val = 0;
	__le16 data;
	int err;

	mutex_lock(&iio_dev->mlock);
	if (iio_buffer_enabled(iio_dev)) {
		err = -EBUSY;
		goto unlock_iio_dev;
	}

	err = st_lsm6dsox_step_counter_set_enable(sensor, true);
	if (err < 0)
		goto unlock_iio_dev;

	mutex_lock(&hw->page_lock);
	err = st_lsm6dsox_set_page_access(hw, true,
					  ST_LSM6DSOX_REG_FUNC_CFG_MASK);
	if (err < 0)
		goto unlock_page;

	do {
		prev_val = val;
		err = __st_lsm6dsox_write_with_mask(hw,
					ST_LSM6DSOX_REG_EMB_FUNC_SRC_ADDR,
					ST_LSM6DSOX_REG_PEDO_RST_STEP_MASK, 1);
		if (err < 0)
			goto reset_page;

		msleep(100);

		err = regmap_bulk_read(hw->regmap,
				       ST_LSM6DSOX_REG_STEP_COUNTER_L_ADDR,
				       (u8 *)&data, sizeof(data));
		if (err < 0)
			goto reset_page;

		val = le16_to_cpu(data);
	} while (val && val >= prev_val);

reset_page:
	st_lsm6dsox_set_page_access(hw, false, ST_LSM6DSOX_REG_FUNC_CFG_MASK);
unlock_page:
	mutex_unlock(&hw->page_lock);

	err = st_lsm6dsox_step_counter_set_enable(sensor, false);
unlock_iio_dev:
	mutex_unlock(&iio_dev->mlock);

	return err;
}

/**
 * Reset step counter value
 *
 * @param  dev: IIO Device.
 * @param  attr: IIO Channel attribute.
 * @param  buf: User buffer.
 * @param  size: User buffer size.
 * @return  buffer len, negative for ERROR
 */
static ssize_t st_lsm6dsox_sysfs_reset_step_counter(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	int err;

	err = st_lsm6dsox_reset_step_counter(iio_dev);

	return err < 0 ? err : size;
}

/**
 * Read sensor event configuration
 *
 * @param  iio_dev: IIO Device.
 * @param  chan: IIO Channel.
 * @param  type: Event Type.
 * @param  dir: Event Direction.
 * @return  1 if Enabled, 0 Disabled
 */
static int st_lsm6dsox_read_embfunc_config(struct iio_dev *iio_dev,
					   const struct iio_chan_spec *chan,
					   enum iio_event_type type,
					   enum iio_event_direction dir)
{
	struct st_lsm6dsox_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dsox_hw *hw = sensor->hw;

	return !!(hw->enable_mask & BIT(sensor->id));
}

/**
 * Write sensor event configuration
 *
 * @param  iio_dev: IIO Device.
 * @param  chan: IIO Channel.
 * @param  type: Event Type.
 * @param  dir: Event Direction.
 * @param  state: New event state.
 * @return  0 if OK, negative for ERROR
 */
static int st_lsm6dsox_write_embfunc_config(struct iio_dev *iio_dev,
					    const struct iio_chan_spec *chan,
					    enum iio_event_type type,
					    enum iio_event_direction dir,
					    int state)
{
	struct st_lsm6dsox_sensor *sensor = iio_priv(iio_dev);
	int err;

	mutex_lock(&iio_dev->mlock);
	err = st_lsm6dsox_embfunc_sensor_set_enable(sensor, state);
	mutex_unlock(&iio_dev->mlock);

	return err;
}

static IIO_DEVICE_ATTR(reset_counter, 0200, NULL,
		       st_lsm6dsox_sysfs_reset_step_counter, 0);
static IIO_DEVICE_ATTR(hwfifo_stepc_watermark_max, 0444,
		       st_lsm6dsox_get_max_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_stepc_flush, 0200, NULL,
		       st_lsm6dsox_flush_fifo, 0);
static IIO_DEVICE_ATTR(hwfifo_stepc_watermark, 0644,
		       st_lsm6dsox_get_watermark,
		       st_lsm6dsox_set_watermark, 0);
static IIO_DEVICE_ATTR(module_id, 0444,
		       st_lsm6dsox_get_module_id, NULL, 0);

static struct attribute *st_lsm6dsox_step_counter_attributes[] = {
	&iio_dev_attr_hwfifo_stepc_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_stepc_watermark.dev_attr.attr,
	&iio_dev_attr_reset_counter.dev_attr.attr,
	&iio_dev_attr_hwfifo_stepc_flush.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsox_step_counter_attribute_group = {
	.attrs = st_lsm6dsox_step_counter_attributes,
};

static const struct iio_info st_lsm6dsox_step_counter_info = {
	.attrs = &st_lsm6dsox_step_counter_attribute_group,
};

static struct attribute *st_lsm6dsox_step_detector_attributes[] = {
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsox_step_detector_attribute_group = {
	.attrs = st_lsm6dsox_step_detector_attributes,
};

static const struct iio_info st_lsm6dsox_step_detector_info = {
	.attrs = &st_lsm6dsox_step_detector_attribute_group,
	.read_event_config = st_lsm6dsox_read_embfunc_config,
	.write_event_config = st_lsm6dsox_write_embfunc_config,
};

static struct attribute *st_lsm6dsox_sign_motion_attributes[] = {
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsox_sign_motion_attribute_group = {
	.attrs = st_lsm6dsox_sign_motion_attributes,
};

static const struct iio_info st_lsm6dsox_sign_motion_info = {
	.attrs = &st_lsm6dsox_sign_motion_attribute_group,
	.read_event_config = st_lsm6dsox_read_embfunc_config,
	.write_event_config = st_lsm6dsox_write_embfunc_config,
};

static struct attribute *st_lsm6dsox_tilt_attributes[] = {
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dsox_tilt_attribute_group = {
	.attrs = st_lsm6dsox_tilt_attributes,
};

static const struct iio_info st_lsm6dsox_tilt_info = {
	.attrs = &st_lsm6dsox_tilt_attribute_group,
	.read_event_config = st_lsm6dsox_read_embfunc_config,
	.write_event_config = st_lsm6dsox_write_embfunc_config,
};

static const unsigned long st_lsm6dsox_emb_available_scan_masks[] = {
	0x1, 0x0
};

static struct
iio_dev *st_lsm6dsox_alloc_embfunc_iiodev(struct st_lsm6dsox_hw *hw,
					  enum st_lsm6dsox_sensor_id id)
{
	struct st_lsm6dsox_sensor *sensor;
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
	sensor->last_fifo_timestamp = 0;

	switch (id) {
	case ST_LSM6DSOX_ID_STEP_COUNTER:
		iio_dev->channels = st_lsm6dsox_step_counter_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lsm6dsox_step_counter_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_step_c", hw->dev_name);
		iio_dev->info = &st_lsm6dsox_step_counter_info;
		iio_dev->available_scan_masks =
					st_lsm6dsox_emb_available_scan_masks;

		/* request an acc ODR at least of 26 Hz to works properly */
		sensor->max_watermark = 1;
		sensor->odr =
			hw->st_lsm6dsox_odr_table[ST_LSM6DSOX_ID_ACC].odr_avl[2].hz;
		sensor->uodr =
			hw->st_lsm6dsox_odr_table[ST_LSM6DSOX_ID_ACC].odr_avl[2].uhz;
		break;
	case ST_LSM6DSOX_ID_STEP_DETECTOR:
		iio_dev->channels = st_lsm6dsox_step_detector_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lsm6dsox_step_detector_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_step_d", hw->dev_name);
		iio_dev->info = &st_lsm6dsox_step_detector_info;
		iio_dev->available_scan_masks =
					st_lsm6dsox_emb_available_scan_masks;

		sensor->odr =
			hw->st_lsm6dsox_odr_table[ST_LSM6DSOX_ID_ACC].odr_avl[2].hz;
		sensor->uodr =
			hw->st_lsm6dsox_odr_table[ST_LSM6DSOX_ID_ACC].odr_avl[2].uhz;
		break;
	case ST_LSM6DSOX_ID_SIGN_MOTION:
		iio_dev->channels = st_lsm6dsox_sign_motion_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lsm6dsox_sign_motion_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_sign_motion", hw->dev_name);
		iio_dev->info = &st_lsm6dsox_sign_motion_info;
		iio_dev->available_scan_masks =
					st_lsm6dsox_emb_available_scan_masks;

		sensor->odr =
			hw->st_lsm6dsox_odr_table[ST_LSM6DSOX_ID_ACC].odr_avl[2].hz;
		sensor->uodr =
			hw->st_lsm6dsox_odr_table[ST_LSM6DSOX_ID_ACC].odr_avl[2].uhz;
		break;
	case ST_LSM6DSOX_ID_TILT:
		iio_dev->channels = st_lsm6dsox_tilt_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dsox_tilt_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_tilt", hw->dev_name);
		iio_dev->info = &st_lsm6dsox_tilt_info;
		iio_dev->available_scan_masks =
					st_lsm6dsox_emb_available_scan_masks;

		sensor->odr =
			hw->st_lsm6dsox_odr_table[ST_LSM6DSOX_ID_ACC].odr_avl[2].hz;
		sensor->uodr =
			hw->st_lsm6dsox_odr_table[ST_LSM6DSOX_ID_ACC].odr_avl[2].uhz;
		break;
	default:
		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

/**
 * Initialize Embedded funcrtion HW block [EMB_FUN]
 *
 * @param  hw: ST IMU MEMS hw instance
 * @return  < 0 if error, 0 otherwise
 */
static int st_lsm6dsox_embedded_function_init(struct st_lsm6dsox_hw *hw)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = st_lsm6dsox_set_page_access(hw, true,
					  ST_LSM6DSOX_REG_FUNC_CFG_MASK);
	if (err < 0)
		goto unlock;

	/* enable latched interrupts */
	err  = __st_lsm6dsox_write_with_mask(hw, ST_LSM6DSOX_PAGE_RW_ADDR,
					     ST_LSM6DSOX_REG_EMB_FUNC_LIR_MASK,
					     1);

	st_lsm6dsox_set_page_access(hw, false, ST_LSM6DSOX_REG_FUNC_CFG_MASK);
unlock:
	mutex_unlock(&hw->page_lock);

	/* enable embedded function interrupts enable */
	err = regmap_update_bits(hw->regmap, hw->embfunc_pg0_irq_reg,
				 ST_LSM6DSOX_REG_INT_EMB_FUNC_MASK,
				 FIELD_PREP(ST_LSM6DSOX_REG_INT_EMB_FUNC_MASK,
					    1));
	if (err < 0)
		return err;


	return err;
}

/**
 * st_lsm6dsox_embfunc_handler_thread() - Bottom handler for embedded
 *					  function event detection
 *
 * @hw: ST IMU MEMS hw instance.
 *
 * return IRQ_HANDLED or < 0 for error
 */
int st_lsm6dsox_embfunc_handler_thread(struct st_lsm6dsox_hw *hw)
{
	if (hw->enable_mask & (BIT(ST_LSM6DSOX_ID_STEP_DETECTOR) |
			       BIT(ST_LSM6DSOX_ID_TILT) |
			       BIT(ST_LSM6DSOX_ID_SIGN_MOTION))) {
		struct iio_dev *iio_dev;
		u8 status;
		s64 event;
		int err;

		err = st_lsm6dsox_read_locked(hw,
			       ST_LSM6DSOX_REG_EMB_FUNC_STATUS_MAINPAGE,
			       &status, sizeof(status));
		if (err < 0)
			return IRQ_HANDLED;

		/* embedded function sensors */
		if (status & ST_LSM6DSOX_REG_INT_STEP_DET_MASK) {
			iio_dev = hw->iio_devs[ST_LSM6DSOX_ID_STEP_DETECTOR];
			event = IIO_UNMOD_EVENT_CODE(IIO_STEPS,
						     -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       iio_get_time_ns(iio_dev));
		}

		if (status & ST_LSM6DSOX_REG_INT_SIGMOT_MASK) {
			iio_dev = hw->iio_devs[ST_LSM6DSOX_ID_SIGN_MOTION];
			event = IIO_UNMOD_EVENT_CODE(STM_IIO_SIGN_MOTION,
						     -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       iio_get_time_ns(iio_dev));
		}

		if (status & ST_LSM6DSOX_REG_INT_TILT_MASK) {
			iio_dev = hw->iio_devs[ST_LSM6DSOX_ID_TILT];
			event = IIO_UNMOD_EVENT_CODE(STM_IIO_TILT, -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       iio_get_time_ns(iio_dev));
		}
	}

	return IRQ_HANDLED;
}

/**
 * st_lsm6dsox_probe_embfunc() - Allocate IIO embedded function device
 *
 * @hw: ST IMU MEMS hw instance.
 *
 * return 0 or < 0 for error
 */
int st_lsm6dsox_probe_embfunc(struct st_lsm6dsox_hw *hw)
{
	enum st_lsm6dsox_sensor_id id;
	int i;

	for (i = 0; i < ARRAY_SIZE(st_lsm6dsox_embfunc_sensor_list);
	     i++) {

		id = st_lsm6dsox_embfunc_sensor_list[i];
		hw->iio_devs[id] = st_lsm6dsox_alloc_embfunc_iiodev(hw,
								    id);
		if (!hw->iio_devs[id])
			return -ENOMEM;
	}

	return st_lsm6dsox_embedded_function_init(hw);
}
