// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lis2duxs12 embedded function sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2024 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/version.h>

#include "st_lis2duxs12.h"

#define ST_LIS2DUXS12_STEP_CHANNEL(addr)				\
{									\
	.type = IIO_STEPS,						\
	.address = addr,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |		\
			      BIT(IIO_CHAN_INFO_ENABLE),		\
	.scan_index = 0,						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_LE,					\
	},								\
	.event_spec = st_lis2duxs12_step_events,			\
	.num_event_specs = ARRAY_SIZE(st_lis2duxs12_step_events),	\
}

static const struct iio_event_spec st_lis2duxs12_step_events[] = {
	{
		/* step detector */
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_NONE,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
	{
		/* significan motion */
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec st_lis2duxs12_step_channels[] = {
	ST_LIS2DUXS12_STEP_CHANNEL(ST_LIS2DUXS12_STEP_COUNTER_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

/**
 * Reset Step Counter register value
 *
 * @param  iio_dev: IIO device
 * @return  < 0 if error, 0 otherwise
 */
int st_lis2duxs12_reset_step_counter(struct iio_dev *iio_dev)
{
	struct st_lis2duxs12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2duxs12_hw *hw = sensor->hw;
	struct st_lis2duxs12_sensor *sensor_acc;
	struct iio_dev *iio_dev_acc;
	bool release_acc = false;
	u16 odr_acc = hw->xl_odr;
	int err;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	mutex_lock(&hw->page_lock);

	/*
	 * the step count is not reset to zero when the accelerometer is
	 * configured in power-down
	 */
	if (!(hw->enable_mask & BIT(ST_LIS2DUXS12_ID_ACC))) {
		iio_dev_acc = hw->iio_devs[ST_LIS2DUXS12_ID_ACC];

		/* claim and lock accel iio device during reset step counter */
		err = iio_device_claim_direct_mode(iio_dev_acc);
		if (err)
			goto unlock;

		release_acc = true;
		sensor_acc = iio_priv(iio_dev_acc);

		/* set accel odr to 400 Hz to speed up reset operation */
		err = __st_lis2duxs12_write_with_mask(hw,
			     hw->odr_table_entry[ST_LIS2DUXS12_ID_ACC].reg.addr,
			     hw->odr_table_entry[ST_LIS2DUXS12_ID_ACC].reg.mask,
			     0x0a);
		if (err)
			goto unlock;

		odr_acc = 400;
	}

	err = st_lis2duxs12_set_emb_access(hw, true);
	if (err < 0)
		goto unlock;

	err = __st_lis2duxs12_write_with_mask(hw,
					      ST_LIS2DUXS12_EMB_FUNC_EN_A_ADDR,
					      ST_LIS2DUXS12_PEDO_EN_MASK,
					      1);
	if (err < 0)
		goto unlock;

	err = __st_lis2duxs12_write_with_mask(hw,
					      ST_LIS2DUXS12_EMB_FUNC_SRC_ADDR,
					      ST_LIS2DUXS12_PEDO_RST_STEP_MASK,
					      1);

	st_lis2duxs12_set_emb_access(hw, false);

	/* wait at least one odr */
	if (odr_acc == 0)
		goto unlock;

	usleep_range(1000000 / odr_acc, 1100000 / odr_acc);

unlock:
	__st_lis2duxs12_write_with_mask(hw, ST_LIS2DUXS12_EMB_FUNC_EN_A_ADDR,
					ST_LIS2DUXS12_PEDO_EN_MASK, 0);
	if (release_acc) {
		iio_device_release_direct_mode(iio_dev_acc);
		__st_lis2duxs12_write_with_mask(hw,
			hw->odr_table_entry[ST_LIS2DUXS12_ID_ACC].reg.addr,
			hw->odr_table_entry[ST_LIS2DUXS12_ID_ACC].reg.mask,
			0);
	}

	mutex_unlock(&hw->page_lock);

	iio_device_release_direct_mode(iio_dev);

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

/**
 * Pedometer enable
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
static int st_lis2duxs12_pedometer_enable(struct st_lis2duxs12_sensor *sensor,
					  bool enable)
{
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int err;

	err = st_lis2duxs12_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, true);
	if (err < 0)
		goto unlock;

	err = __st_lis2duxs12_write_with_mask(hw,
					  ST_LIS2DUXS12_EMB_FUNC_EN_A_ADDR,
					  ST_LIS2DUXS12_PEDO_EN_MASK,
					  enable);

	st_lis2duxs12_set_emb_access(hw, false);

unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Enable Step detection event detection
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
static int
st_lis2duxs12_step_event_enable(struct st_lis2duxs12_sensor *sensor,
				bool enable)
{
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int err;

	err = st_lis2duxs12_pedometer_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, true);
	if (err < 0)
		goto unlock;

	err = __st_lis2duxs12_write_with_mask(hw, hw->emb_int_reg,
					   ST_LIS2DUXS12_INT_STEP_DETECTOR_MASK,
					   enable);
	st_lis2duxs12_set_emb_access(hw, false);

unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Significant motion enable
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
static int st_lis2duxs12_signmot_enable(struct st_lis2duxs12_sensor *sensor,
					bool enable)
{
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int err;

	err = st_lis2duxs12_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, true);
	if (err < 0)
		goto unlock;

	err = __st_lis2duxs12_write_with_mask(hw,
					      ST_LIS2DUXS12_EMB_FUNC_EN_A_ADDR,
					      ST_LIS2DUXS12_SIGN_MOTION_EN_MASK,
					      enable);

	st_lis2duxs12_set_emb_access(hw, false);

unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Enable significant motion event detection
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
static int
st_lis2duxs12_signmot_event_enable(struct st_lis2duxs12_sensor *sensor,
				   bool enable)
{
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int err;

	err = st_lis2duxs12_signmot_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, true);
	if (err < 0)
		goto unlock;

	err = __st_lis2duxs12_write_with_mask(hw, hw->emb_int_reg,
					      ST_LIS2DUXS12_INT_SIG_MOT_MASK,
					      enable);
	st_lis2duxs12_set_emb_access(hw, false);

unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Enable Step sensor
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
int st_lis2duxs12_step_enable(struct st_lis2duxs12_sensor *sensor,
			      bool enable)
{
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int err;

	err = st_lis2duxs12_pedometer_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, true);
	if (err < 0)
		goto unlock;

	/* store step counter on FIFO */
	err = __st_lis2duxs12_write_with_mask(hw,
					ST_LIS2DUXS12_EMB_FUNC_FIFO_EN_ADDR,
					ST_LIS2DUXS12_STEP_COUNTER_FIFO_EN_MASK,
					enable);
	if (err < 0)
		goto reset_page;

	if (enable)
		hw->enable_mask |= BIT(sensor->id);
	else
		hw->enable_mask &= ~BIT(sensor->id);

reset_page:
	st_lis2duxs12_set_emb_access(hw, false);

unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

static int
st_lis2duxs12_read_step_counter(struct st_lis2duxs12_sensor *sensor,
				u8 addr, int *val)
{
	struct st_lis2duxs12_hw *hw = sensor->hw;
	__le16 data;
	int err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, true);
	if (err < 0)
		goto unlock;

	err = regmap_bulk_read(hw->regmap, addr, &data, sizeof(data));
	if (err < 0)
		goto reset_page;

	*val = (s16)le16_to_cpu(data);

reset_page:
	st_lis2duxs12_set_emb_access(hw, false);

unlock:
	mutex_unlock(&hw->page_lock);

	return err < 0 ? err : IIO_VAL_INT;
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
static int
st_lis2duxs12_read_event_step_config(struct iio_dev *iio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct st_lis2duxs12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int err = -EINVAL;

	if (chan->type == IIO_STEPS) {
		switch (type) {
		case IIO_EV_TYPE_CHANGE:
			switch (dir) {
			/*
			 * this is the step detect event, use the dir
			 * IIO_EV_TYPE_CHANGE because don't exist a specific
			 * iio_event_type related to this events
			 */
			case IIO_EV_DIR_NONE:
				return !!(hw->enable_ev_mask &
					  BIT(ST_LIS2DUXS12_EVENT_STEPC));

			/*
			 * this is the significant motion event, use the dir
			 * IIO_EV_DIR_RISING because don't exist a specific
			 * iio_event_type related to this events
			 */
			case IIO_EV_DIR_RISING:
				return !!(hw->enable_ev_mask &
					  BIT(ST_LIS2DUXS12_EVENT_SIGNMOT));

			default:
				return -EINVAL;
			}
			break;

		default:
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	return err;
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
static int
st_lis2duxs12_write_event_step_config(struct iio_dev *iio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      int state)
{
	struct st_lis2duxs12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int err = -EINVAL;

	if (chan->type == IIO_STEPS) {
		switch (type) {
		case IIO_EV_TYPE_CHANGE:
			switch (dir) {
			/*
			 * this is the step detect event, use the dir
			 * IIO_EV_TYPE_CHANGE because don't exist a specific
			 * iio_event_type related to this events
			 */
			case IIO_EV_DIR_NONE:
				err = st_lis2duxs12_step_event_enable(sensor,
								      state);
				if (err < 0)
					return err;

				if (state)
					hw->enable_ev_mask |=
						BIT(ST_LIS2DUXS12_EVENT_STEPC);
				else
					hw->enable_ev_mask &=
					       ~BIT(ST_LIS2DUXS12_EVENT_STEPC);

				break;

			/*
			 * this is the significant motion event, use the dir
			 * IIO_EV_DIR_RISING because don't exist a specific
			 * iio_event_type related to this events
			 */
			case IIO_EV_DIR_RISING:
				err = st_lis2duxs12_signmot_event_enable(sensor,
									 state);
				if (err < 0)
					return err;

				if (state)
					hw->enable_ev_mask |=
					      BIT(ST_LIS2DUXS12_EVENT_SIGNMOT);
				else
					hw->enable_ev_mask &=
					     ~BIT(ST_LIS2DUXS12_EVENT_SIGNMOT);
				break;

			default:
				return -EINVAL;
			}
			break;

		default:
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	return err;
}

static int st_lis2duxs12_read_step_raw(struct iio_dev *iio_dev,
				       struct iio_chan_spec const *ch,
				       int *val, int *val2, long mask)
{
	struct st_lis2duxs12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2duxs12_hw *hw = sensor->hw;
	int ret = -EINVAL;

	switch (ch->type) {
	case IIO_STEPS:
		switch (mask) {
		case IIO_CHAN_INFO_PROCESSED:
			ret = iio_device_claim_direct_mode(iio_dev);
			if (ret)
				return ret;

			ret = st_lis2duxs12_read_step_counter(sensor,
							      ch->address, val);
			iio_device_release_direct_mode(iio_dev);
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_ENABLE:
			*val = !!(hw->enable_mask & BIT(sensor->id));
			return IIO_VAL_INT;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int st_lis2duxs12_write_step_raw(struct iio_dev *iio_dev,
					struct iio_chan_spec const *ch,
					int val, int val2, long mask)
{
	struct st_lis2duxs12_sensor *sensor = iio_priv(iio_dev);

	if ((ch->type == IIO_STEPS) &&
	    (mask == IIO_CHAN_INFO_ENABLE))
		return st_lis2duxs12_step_enable(sensor, val);

	return -EINVAL;
}

static IIO_DEVICE_ATTR(reset_counter, 0200, NULL,
		       st_lis2duxs12_sysfs_reset_step_counter, 0);

static struct attribute *st_lis2duxs12_step_attributes[] = {
	&iio_dev_attr_reset_counter.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lis2duxs12_step_attribute_group = {
	.attrs = st_lis2duxs12_step_attributes,
};

static const struct iio_info st_lis2duxs12_step_info = {
	.attrs = &st_lis2duxs12_step_attribute_group,
	.read_raw = st_lis2duxs12_read_step_raw,
	.write_raw = st_lis2duxs12_write_step_raw,
	.read_event_config = st_lis2duxs12_read_event_step_config,
	.write_event_config = st_lis2duxs12_write_event_step_config,
};

/**
 * st_lis2duxs12_embfunc_handler_thread() - Bottom handler embedded function
 *
 * @hw: ST IMU MEMS hw instance.
 *
 * return IRQ_HANDLED or < 0 for error
 */
int st_lis2duxs12_embfunc_handler_thread(struct st_lis2duxs12_hw *hw)
{
	struct iio_dev *iio_dev;
	u8 status;
	int err;

	if (!(hw->enable_mask & BIT(ST_LIS2DUXS12_ID_STEP_COUNTER)))
		return IRQ_HANDLED;

	err = st_lis2duxs12_read_locked(hw,
				    ST_LIS2DUXS12_EMB_FUNC_STATUS_MAINPAGE_ADDR,
				    &status, sizeof(status));
	if (err < 0)
		return IRQ_HANDLED;

	iio_dev = hw->iio_devs[ST_LIS2DUXS12_ID_STEP_COUNTER];
	if (status & ST_LIS2DUXS12_IS_STEP_DET_MASK)
		iio_push_event(iio_dev,
			       IIO_MOD_EVENT_CODE(IIO_STEPS, 0, IIO_NO_MOD,
						  IIO_EV_TYPE_CHANGE,
						  IIO_EV_DIR_NONE),
			       iio_get_time_ns(iio_dev));

	if (status & ST_LIS2DUXS12_IS_SIGMOT_MASK)
		iio_push_event(iio_dev,
			       IIO_MOD_EVENT_CODE(IIO_STEPS, 0, IIO_NO_MOD,
						  IIO_EV_TYPE_CHANGE,
						  IIO_EV_DIR_RISING),
			       iio_get_time_ns(iio_dev));

	return IRQ_HANDLED;
}

static struct
iio_dev *st_lis2duxs12_alloc_step_iiodev(struct st_lis2duxs12_hw *hw,
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
	sensor->decimator = 0;
	sensor->dec_counter = 0;

	iio_dev->channels = st_lis2duxs12_step_channels;
	iio_dev->num_channels = ARRAY_SIZE(st_lis2duxs12_step_channels);
	scnprintf(sensor->name, sizeof(sensor->name),
		  "%s_step", hw->settings->id.name);
	iio_dev->info = &st_lis2duxs12_step_info;

	/* request acc ODR to works properly */
	sensor->odr = ST_LIS2DUXS12_MIN_ODR_IN_EMB_FUNC;
	sensor->uodr = 0;

	iio_dev->name = sensor->name;

	return iio_dev;
}

/**
 * Initialize Embedded funcrtion HW block
 *
 * @param  hw: ST IMU MEMS hw instance
 * @return  < 0 if error, 0 otherwise
 */
static int st_lis2duxs12_embedded_function_init(struct st_lis2duxs12_hw *hw)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = st_lis2duxs12_set_emb_access(hw, true);
	if (err < 0)
		goto unlock;

	/* enable latched interrupts */
	__st_lis2duxs12_write_with_mask(hw, ST_LIS2DUXS12_PAGE_RW_ADDR,
					ST_LIS2DUXS12_EMB_FUNC_LIR_MASK, 1);
	st_lis2duxs12_set_emb_access(hw, false);

	/* enable embedded function */
	err = __st_lis2duxs12_write_with_mask(hw, ST_LIS2DUXS12_CTRL4_ADDR,
					      ST_LIS2DUXS12_EMB_FUNC_EN_MASK,
					      1);
	if (err < 0)
		goto unlock;

	/* enable embedded function interrupts enable */
	err  = __st_lis2duxs12_write_with_mask(hw, hw->md_int_reg,
					       ST_LIS2DUXS12_INT_EMB_FUNC_MASK,
					       1);
unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * st_lis2duxs12_embfunc_probe() - Enable embedded functionalities
 *
 * @hw: ST IMU MEMS hw instance.
 *
 * return 0 or < 0 for error
 */
int st_lis2duxs12_embfunc_probe(struct st_lis2duxs12_hw *hw)
{
	enum st_lis2duxs12_sensor_id id;

	id = ST_LIS2DUXS12_ID_STEP_COUNTER;
	hw->iio_devs[id] = st_lis2duxs12_alloc_step_iiodev(hw, id);
	if (!hw->iio_devs[id])
		return -ENOMEM;

	return st_lis2duxs12_embedded_function_init(hw);
}
