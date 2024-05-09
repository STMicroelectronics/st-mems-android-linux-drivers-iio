// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2dw12 embedded function
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2024 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>

#include "st_lis2dw12.h"

static const enum
st_lis2dw12_sensor_id st_lis2dw12_embedded_function_sensor_list[] = {
	[0] = ST_LIS2DW12_ID_TAP_TAP,
	[1] = ST_LIS2DW12_ID_TAP,
	[2] = ST_LIS2DW12_ID_WU,
};

const struct iio_event_spec st_lis2dw12_rthr_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

static const struct iio_chan_spec st_lis2dw12_tap_tap_channels[] = {
	ST_LIS2DW12_EVENT_CHANNEL(STM_IIO_TAP_TAP, &st_lis2dw12_rthr_event),
};

static const struct iio_chan_spec st_lis2dw12_tap_channels[] = {
	ST_LIS2DW12_EVENT_CHANNEL(STM_IIO_TAP, &st_lis2dw12_rthr_event),
};

static const struct iio_chan_spec st_lis2dw12_wu_channels[] = {
	ST_LIS2DW12_EVENT_CHANNEL(STM_IIO_GESTURE, &st_lis2dw12_rthr_event),
};

static int st_lis2dw12_init_embfunc_hw(struct st_lis2dw12_hw *hw)
{
	int err;

	/* configure default free fall event threshold */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_FREE_FALL_ADDR,
						 ST_LIS2DW12_FREE_FALL_THS_MASK,
						 1);
	if (err < 0)
		return err;

	/* configure default free fall event duration */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_FREE_FALL_ADDR,
						 ST_LIS2DW12_FREE_FALL_DUR_MASK,
						 1);
	if (err < 0)
		return err;

	/* enable tap event on all axes */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_TAP_THS_Z_ADDR,
						 ST_LIS2DW12_TAP_AXIS_MASK,
						 0x7);
	if (err < 0)
		return err;

	/* configure default threshold for Tap event recognition */
	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_TAP_THS_X_ADDR,
						 ST_LIS2DW12_TAP_THS_MAK, 9);
	if (err < 0)
		return err;

	err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_TAP_THS_Y_ADDR,
						 ST_LIS2DW12_TAP_THS_MAK, 9);
	if (err < 0)
		return err;

	return st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_TAP_THS_Z_ADDR,
						  ST_LIS2DW12_TAP_THS_MAK, 9);
}

static int st_lis2dw12_read_event_config(struct iio_dev *iio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir)
{
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;

	return !!(hw->enable_mask & BIT(sensor->id));
}

static int st_lis2dw12_write_event_config(struct iio_dev *iio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir,
					  int state)
{
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;
	u8 data[2] = {}, drdy_val, drdy_mask;
	int err;

	/* Read initial configuration data */
	err = st_lis2dw12_read(hw, ST_LIS2DW12_INT_DUR_ADDR,
			       &data, sizeof(data));
	if (err < 0)
		return -EINVAL;

	switch (sensor->id) {
	case ST_LIS2DW12_ID_WU:
		drdy_mask = ST_LIS2DW12_WU_INT1_MASK;
		drdy_val = state ? 1 : 0;
		data[1] = state ? 0x02 : 0;
		break;
	case ST_LIS2DW12_ID_TAP_TAP:
		drdy_mask = ST_LIS2DW12_TAP_TAP_INT1_MASK;
		drdy_val = state ? 1 : 0;
		if (state) {
			data[0] |= 0x7f;
			data[1] |= 0x80;
		} else {
			data[0] &= ~0x7f;
			data[1] &= ~0x80;
		}
		break;
	case ST_LIS2DW12_ID_TAP:
		drdy_mask = ST_LIS2DW12_TAP_INT1_MASK;
		drdy_val = state ? 1 : 0;
		if (state)
			data[0] |= 6;
		else
			data[0] &= ~6;
		break;
	default:

		return -EINVAL;
	}

	err = st_lis2dw12_write_locked(hw, ST_LIS2DW12_INT_DUR_ADDR,
				       data, sizeof(data));
	if (err < 0)
		return err;

	err = st_lis2dw12_write_with_mask_locked(hw,
					       ST_LIS2DW12_CTRL4_INT1_CTRL_ADDR,
					       drdy_mask, drdy_val);
	if (err < 0)
		return err;

	return st_lis2dw12_sensor_set_enable(sensor, state);
}

static struct attribute *st_lis2dw12_wu_attributes[] = {
	NULL,
};

static const struct attribute_group st_lis2dw12_wu_attribute_group = {
	.attrs = st_lis2dw12_wu_attributes,
};

static const struct iio_info st_lis2dw12_wu_info = {
	.attrs = &st_lis2dw12_wu_attribute_group,
	.read_event_config = st_lis2dw12_read_event_config,
	.write_event_config = st_lis2dw12_write_event_config,
};

static struct attribute *st_lis2dw12_tap_tap_attributes[] = {
	NULL,
};

static const struct attribute_group st_lis2dw12_tap_tap_attribute_group = {
	.attrs = st_lis2dw12_tap_tap_attributes,
};

static const struct iio_info st_lis2dw12_tap_tap_info = {
	.attrs = &st_lis2dw12_tap_tap_attribute_group,
	.read_event_config = st_lis2dw12_read_event_config,
	.write_event_config = st_lis2dw12_write_event_config,
};

static struct attribute *st_lis2dw12_tap_attributes[] = {
	NULL,
};

static const struct attribute_group st_lis2dw12_tap_attribute_group = {
	.attrs = st_lis2dw12_tap_attributes,
};

static const struct iio_info st_lis2dw12_tap_info = {
	.attrs = &st_lis2dw12_tap_attribute_group,
	.read_event_config = st_lis2dw12_read_event_config,
	.write_event_config = st_lis2dw12_write_event_config,
};

static const unsigned long st_lis2dw12_event_avail_scan_masks[] = { 0x1, 0x0 };

static struct
iio_dev *st_lis2dw12_alloc_embfunc_iiodev(struct st_lis2dw12_hw *hw,
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
	case ST_LIS2DW12_ID_WU:
		iio_dev->channels = st_lis2dw12_wu_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lis2dw12_wu_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_wk", hw->name);
		iio_dev->info = &st_lis2dw12_wu_info;
		iio_dev->available_scan_masks =
				st_lis2dw12_event_avail_scan_masks;

		sensor->odr = hw->odr_entry[ST_LIS2DW12_ID_ACC].odr[5].hz;
		break;
	case ST_LIS2DW12_ID_TAP_TAP:
		iio_dev->channels = st_lis2dw12_tap_tap_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lis2dw12_tap_tap_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_tap_tap", hw->name);
		iio_dev->info = &st_lis2dw12_tap_tap_info;
		iio_dev->available_scan_masks =
				st_lis2dw12_event_avail_scan_masks;

		sensor->odr = hw->odr_entry[ST_LIS2DW12_ID_ACC].odr[6].hz;
		break;
	case ST_LIS2DW12_ID_TAP:
		iio_dev->channels = st_lis2dw12_tap_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lis2dw12_tap_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_tap", hw->name);
		iio_dev->info = &st_lis2dw12_tap_info;
		iio_dev->available_scan_masks =
				st_lis2dw12_event_avail_scan_masks;

		sensor->odr = hw->odr_entry[ST_LIS2DW12_ID_ACC].odr[6].hz;
		break;
	default:
		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

int st_lis2dw12_emb_event(struct st_lis2dw12_hw *hw)
{
	u8 status;
	s64 code;
	int err;

	err = st_lis2dw12_read(hw, ST_LIS2DW12_ALL_INT_SRC_ADDR,
			       &status, sizeof(status));
	if (err < 0)
		return IRQ_HANDLED;

	if (((status & ST_LIS2DW12_ALL_INT_SRC_TAP_MASK) &&
	     (hw->enable_mask & BIT(ST_LIS2DW12_ID_TAP))) ||
	    ((status & ST_LIS2DW12_ALL_INT_SRC_TAP_TAP_MASK) &&
	     (hw->enable_mask & BIT(ST_LIS2DW12_ID_TAP_TAP)))) {
		struct iio_dev *iio_dev;
		enum iio_chan_type type;
		u8 source;

		err = st_lis2dw12_read(hw, ST_LIS2DW12_TAP_SRC_ADDR,
			       &source, sizeof(source));
		if (err < 0)
			return IRQ_HANDLED;

		/*
		 * consider can have Tap and Double Tap events
		 * contemporarely
		 */
		if (source & ST_LIS2DW12_DTAP_SRC_MASK) {
			iio_dev = hw->iio_devs[ST_LIS2DW12_ID_TAP_TAP];
			type = STM_IIO_TAP_TAP;
			code = IIO_UNMOD_EVENT_CODE(type, -1,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, code, hw->ts_irq);
		}

		if (source & ST_LIS2DW12_STAP_SRC_MASK) {
			iio_dev = hw->iio_devs[ST_LIS2DW12_ID_TAP];
			type = STM_IIO_TAP;
			code = IIO_UNMOD_EVENT_CODE(type, -1,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, code, hw->ts_irq);
		}
	}

	if (status & ST_LIS2DW12_ALL_INT_SRC_WU_MASK) {
		u8 wu_src;

		err = st_lis2dw12_read(hw, ST_LIS2DW12_WU_SRC_ADDR,
			       &wu_src, sizeof(wu_src));
		if (err < 0)
			return IRQ_HANDLED;

		code = IIO_UNMOD_EVENT_CODE(STM_IIO_TAP_TAP, -1,
					    IIO_EV_TYPE_THRESH,
					    IIO_EV_DIR_RISING);
		iio_push_event(hw->iio_devs[ST_LIS2DW12_ID_WU],
			       STM_IIO_GESTURE, hw->ts_irq);
	}

	return IRQ_HANDLED;
}

static irqreturn_t st_lis2dw12_handler_irq_emb(int irq, void *private)
{
	struct st_lis2dw12_hw *hw = private;
	s64 ts;

	ts = st_lis2dw12_get_timestamp(hw);
	hw->ts_irq = ts;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_lis2dw12_handler_thread_emb(int irq, void *private)
{
	return st_lis2dw12_emb_event((struct st_lis2dw12_hw *)private);
}

int st_lis2dw12_embedded_function_probe(struct st_lis2dw12_hw *hw)
{
	int err, i, id;

	for (i = 0;
	     i < ARRAY_SIZE(st_lis2dw12_embedded_function_sensor_list);
	     i++) {
		id = st_lis2dw12_embedded_function_sensor_list[i];

		hw->iio_devs[id] = st_lis2dw12_alloc_embfunc_iiodev(hw, id);
		if (!hw->iio_devs[id])
			continue;
	}

	if (hw->irq_emb > 0) {
		err = devm_request_threaded_irq(hw->dev, hw->irq_emb,
				       st_lis2dw12_handler_irq_emb,
				       st_lis2dw12_handler_thread_emb,
				       IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				       "st_lis2dw12", hw);
		if (err) {
			dev_err(hw->dev, "failed to request trigger irq %d\n",
				hw->irq_emb);
			return err;
		}
	}

	err = st_lis2dw12_init_embfunc_hw(hw);

	return err < 0 ? err : 0;
}
