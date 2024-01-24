// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_iis2iclx events function sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
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

#include "st_iis2iclx.h"

#ifdef CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES

#define ST_IIS2ICLX_REG_ALL_INT_SRC_ADDR	0x1a
#define ST_IIS2ICLX_WU_IA_MASK			BIT(1)
#define ST_IIS2ICLX_D6D_IA_MASK			BIT(4)
#define ST_IIS2ICLX_SLEEP_CHANGE_MASK		BIT(5)

#define ST_IIS2ICLX_REG_WAKE_UP_SRC_ADDR	0x1b
#define ST_IIS2ICLX_WAKE_UP_EVENT_MASK		GENMASK(3, 0)

#define ST_IIS2ICLX_REG_D6D_SRC_ADDR		0x1d
#define ST_IIS2ICLX_D6D_EVENT_MASK		GENMASK(5, 0)

#define ST_IIS2ICLX_REG_INT_CFG1_ADDR		0x58
#define ST_IIS2ICLX_INTERRUPTS_ENABLE_MASK	BIT(7)

#define ST_IIS2ICLX_REG_MD1_CFG_ADDR		0x5e
#define ST_IIS2ICLX_REG_MD2_CFG_ADDR		0x5f
#define ST_IIS2ICLX_INT_6D_MASK			BIT(2)
#define ST_IIS2ICLX_INT_WU_MASK			BIT(5)
#define ST_IIS2ICLX_INT_SLEEP_CHANGE_MASK	BIT(7)

static const unsigned long st_iis2iclx_event_available_scan_masks[] = {
	0x1, 0x0
};

static const struct iio_chan_spec st_iis2iclx_wk_channels[] = {
	{
		.type = STM_IIO_GESTURE,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 8,
			.storagebits = 8,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec st_iis2iclx_sc_channels[] = {
	ST_IIS2ICLX_EVENT_CHANNEL(STM_IIO_GESTURE, thr),
};

static const struct iio_chan_spec st_iis2iclx_6D_channels[] = {
	{
		.type = STM_IIO_GESTURE,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 8,
			.storagebits = 8,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static
int st_iis2iclx_event_sensor_set_enable(struct st_iis2iclx_sensor *sensor,
					bool enable)
{
	int err, eint = !!enable;
	struct st_iis2iclx_hw *hw = sensor->hw;
	u8 int_reg = hw->int_pin == 1 ? ST_IIS2ICLX_REG_MD1_CFG_ADDR :
					ST_IIS2ICLX_REG_MD2_CFG_ADDR;

	err = st_iis2iclx_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	switch (sensor->id) {
	case ST_IIS2ICLX_ID_WK:
		err = st_iis2iclx_write_with_mask_locked(hw, int_reg,
							ST_IIS2ICLX_INT_WU_MASK,
							eint);
		if (err < 0)
			return err;
		break;
	case ST_IIS2ICLX_ID_SC:
		err = st_iis2iclx_write_with_mask_locked(hw, int_reg,
					      ST_IIS2ICLX_INT_SLEEP_CHANGE_MASK,
					      eint);
		if (err < 0)
			return err;
		break;
	case ST_IIS2ICLX_ID_6D:
		err = st_iis2iclx_write_with_mask_locked(hw, int_reg,
							ST_IIS2ICLX_INT_6D_MASK,
							eint);
		if (err < 0)
			return err;
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err >= 0) {
		err = st_iis2iclx_write_with_mask_locked(hw,
					     ST_IIS2ICLX_REG_INT_CFG1_ADDR,
					     ST_IIS2ICLX_INTERRUPTS_ENABLE_MASK,
					     eint);
		if (eint == 0)
			hw->enable_mask &= ~BIT_ULL(sensor->id);
		else
			hw->enable_mask |= BIT_ULL(sensor->id);
	}

	return err;
}

static int st_iis2iclx_read_event_config(struct iio_dev *iio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	struct st_iis2iclx_hw *hw = sensor->hw;

	return !!(hw->enable_mask & BIT_ULL(sensor->id));
}

static int st_iis2iclx_write_event_config(struct iio_dev *iio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir,
					  int state)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	int err;

	mutex_lock(&iio_dev->mlock);
	err = st_iis2iclx_event_sensor_set_enable(sensor, state);
	mutex_unlock(&iio_dev->mlock);

	return err;
}

ssize_t st_iis2iclx_wakeup_threshold_get(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", sensor->conf[0]);
}

ssize_t st_iis2iclx_wakeup_threshold_set(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	int err, val;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		goto out;

	err = st_iis2iclx_set_wake_up_thershold(sensor->hw, val);
	if (err < 0)
		goto out;

	sensor->conf[0] = val;

out:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

ssize_t st_iis2iclx_wakeup_duration_get(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", sensor->conf[1]);
}

ssize_t st_iis2iclx_wakeup_duration_set(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	int err, val;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		goto out;

	err = st_iis2iclx_set_wake_up_duration(sensor->hw, val);
	if (err < 0)
		goto out;

	sensor->conf[1] = val;

out:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

ssize_t st_iis2iclx_6D_threshold_get(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", sensor->conf[3]);
}

ssize_t st_iis2iclx_6D_threshold_set(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	int err, val;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		goto out;

	err = st_iis2iclx_set_6D_threshold(sensor->hw, val);
	if (err < 0)
		goto out;

	sensor->conf[3] = val;

out:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

static IIO_DEVICE_ATTR(wakeup_threshold, 0644,
		       st_iis2iclx_wakeup_threshold_get,
		       st_iis2iclx_wakeup_threshold_set, 0);

static IIO_DEVICE_ATTR(wakeup_duration, 0644,
		       st_iis2iclx_wakeup_duration_get,
		       st_iis2iclx_wakeup_duration_set, 0);

static IIO_DEVICE_ATTR(sixd_threshold, 0644,
		       st_iis2iclx_6D_threshold_get,
		       st_iis2iclx_6D_threshold_set, 0);
static IIO_DEVICE_ATTR(module_id, 0444, st_iis2iclx_get_module_id, NULL, 0);

static struct attribute *st_iis2iclx_wk_attributes[] = {
	&iio_dev_attr_wakeup_threshold.dev_attr.attr,
	&iio_dev_attr_wakeup_duration.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_iis2iclx_wk_attribute_group = {
	.attrs = st_iis2iclx_wk_attributes,
};

static const struct iio_info st_iis2iclx_wk_info = {
	.attrs = &st_iis2iclx_wk_attribute_group,
};

static struct attribute *st_iis2iclx_sc_attributes[] = {
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_iis2iclx_sc_attribute_group = {
	.attrs = st_iis2iclx_sc_attributes,
};

static const struct iio_info st_iis2iclx_sc_info = {
	.attrs = &st_iis2iclx_sc_attribute_group,
	.read_event_config = st_iis2iclx_read_event_config,
	.write_event_config = st_iis2iclx_write_event_config,
};

static struct attribute *st_iis2iclx_6D_attributes[] = {
	&iio_dev_attr_sixd_threshold.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_iis2iclx_6D_attribute_group = {
	.attrs = st_iis2iclx_6D_attributes,
};

static const struct iio_info st_iis2iclx_6D_info = {
	.attrs = &st_iis2iclx_6D_attribute_group,
};

static
struct iio_dev *st_iis2iclx_alloc_event_iiodev(struct st_iis2iclx_hw *hw,
					       enum st_iis2iclx_sensor_id id)
{
	struct st_iis2iclx_sensor *sensor;
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
	sensor->odr = 26;
	iio_dev->available_scan_masks = st_iis2iclx_event_available_scan_masks;

	switch (id) {
	case ST_IIS2ICLX_ID_WK:
		iio_dev->channels = st_iis2iclx_wk_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_iis2iclx_wk_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "iis2iclx_wk");
		iio_dev->info = &st_iis2iclx_wk_info;
		break;
	case ST_IIS2ICLX_ID_SC:
		iio_dev->channels = st_iis2iclx_sc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_iis2iclx_sc_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "iis2iclx_sc");
		iio_dev->info = &st_iis2iclx_sc_info;
		break;
	case ST_IIS2ICLX_ID_6D:
		iio_dev->channels = st_iis2iclx_6D_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_iis2iclx_6D_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "iis2iclx_6d");
		iio_dev->info = &st_iis2iclx_6D_info;
		break;
	default:
		dev_err(hw->dev, "invalid event id %d\n", id);
		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

int st_iis2iclx_event_handler(struct st_iis2iclx_hw *hw)
{
	struct iio_dev *iio_dev;
	u8 status;
	s64 event;
	int err;

	if (hw->enable_mask &
	    (BIT_ULL(ST_IIS2ICLX_ID_WK) | BIT_ULL(ST_IIS2ICLX_ID_SC) |
	     BIT_ULL(ST_IIS2ICLX_ID_6D))) {
		err = regmap_bulk_read(hw->regmap,
				       ST_IIS2ICLX_REG_ALL_INT_SRC_ADDR,
				       &status, sizeof(status));
		if (err < 0)
			return IRQ_HANDLED;

		if (status & ST_IIS2ICLX_WU_IA_MASK) {
			struct st_iis2iclx_sensor *sensor;

			iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_WK];
			sensor = iio_priv(iio_dev);
			iio_trigger_poll_chained(sensor->trig);
		}
		if (status & ST_IIS2ICLX_SLEEP_CHANGE_MASK) {
			iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_SC];
			event = IIO_UNMOD_EVENT_CODE(STM_IIO_GESTURE, -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       st_iis2iclx_get_time_ns(iio_dev));
		}
		if (status & ST_IIS2ICLX_D6D_IA_MASK) {
			struct st_iis2iclx_sensor *sensor;

			iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_6D];
			sensor = iio_priv(iio_dev);
			iio_trigger_poll_chained(sensor->trig);
		}
	}

	return IRQ_HANDLED;
}

static inline int st_iis2iclx_get_6D(struct st_iis2iclx_hw *hw, u8 *out)
{
	return st_iis2iclx_read_with_mask(hw, ST_IIS2ICLX_REG_D6D_SRC_ADDR,
					  ST_IIS2ICLX_D6D_EVENT_MASK, out);
}

static inline int st_iis2iclx_get_wk(struct st_iis2iclx_hw *hw, u8 *out)
{
	return st_iis2iclx_read_with_mask(hw, ST_IIS2ICLX_REG_WAKE_UP_SRC_ADDR,
					  ST_IIS2ICLX_WAKE_UP_EVENT_MASK, out);
}

static irqreturn_t st_iis2iclx_6D_handler_thread(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);

	st_iis2iclx_get_6D(sensor->hw, &sensor->scan.event);
	iio_push_to_buffers_with_timestamp(iio_dev, &sensor->scan.event,
					   st_iis2iclx_get_time_ns(iio_dev));
	iio_trigger_notify_done(sensor->trig);

	return IRQ_HANDLED;
}

static irqreturn_t st_iis2iclx_wk_handler_thread(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);

	st_iis2iclx_get_wk(sensor->hw, &sensor->scan.event);
	iio_push_to_buffers_with_timestamp(iio_dev, &sensor->scan.event,
					   st_iis2iclx_get_time_ns(iio_dev));
	iio_trigger_notify_done(sensor->trig);

	return IRQ_HANDLED;
}

int st_iis2iclx_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *iio_dev = iio_trigger_get_drvdata(trig);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);

	dev_info(sensor->hw->dev, "trigger set %d\n", state);

	return 0;
}

static const struct iio_trigger_ops st_iis2iclx_trigger_ops = {
	.set_trigger_state = &st_iis2iclx_trig_set_state,
};

static int st_iis2iclx_buffer_preenable(struct iio_dev *iio_dev)
{
	return st_iis2iclx_event_sensor_set_enable(iio_priv(iio_dev), true);
}

static int st_iis2iclx_buffer_postdisable(struct iio_dev *iio_dev)
{
	return st_iis2iclx_event_sensor_set_enable(iio_priv(iio_dev), false);
}

static const struct iio_buffer_setup_ops st_iis2iclx_buffer_ops = {
	.preenable = st_iis2iclx_buffer_preenable,

#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
#endif /* LINUX_VERSION_CODE */

	.postdisable = st_iis2iclx_buffer_postdisable,
};

int st_iis2iclx_probe_event(struct st_iis2iclx_hw *hw)
{
	struct st_iis2iclx_sensor *sensor;
	struct iio_dev *iio_dev;
	irqreturn_t (*pthread[ST_IIS2ICLX_ID_MAX - ST_IIS2ICLX_ID_TRIGGER])(int irq, void *p) = {
		[0] = st_iis2iclx_wk_handler_thread,
		[1] = st_iis2iclx_6D_handler_thread,
		/* add here all other trigger handler funcions */
	};
	int i, err;

	for (i = ST_IIS2ICLX_ID_EVENT; i < ST_IIS2ICLX_ID_MAX; i++) {
		hw->iio_devs[i] = st_iis2iclx_alloc_event_iiodev(hw, i);
		if (!hw->iio_devs[i])
			return -ENOMEM;
	}

	/* configure trigger sensors */
	for (i = ST_IIS2ICLX_ID_TRIGGER; i < ST_IIS2ICLX_ID_MAX; i++) {
		iio_dev = hw->iio_devs[i];
		sensor = iio_priv(iio_dev);

		err = devm_iio_triggered_buffer_setup(hw->dev, iio_dev,
				NULL, pthread[i - ST_IIS2ICLX_ID_TRIGGER],
				&st_iis2iclx_buffer_ops);
		if (err < 0)
			return err;

		sensor->trig = devm_iio_trigger_alloc(hw->dev, "%s-trigger",
						      iio_dev->name);
		if (!sensor->trig)
			return -ENOMEM;

		sensor->trig->ops = &st_iis2iclx_trigger_ops;
		sensor->trig->dev.parent = hw->dev;
		iio_trigger_set_drvdata(sensor->trig, iio_dev);

		err = devm_iio_trigger_register(hw->dev, sensor->trig);
		if (err)
			return err;

		iio_dev->trig = iio_trigger_get(sensor->trig);
	}

	for (i = ST_IIS2ICLX_ID_EVENT; i < ST_IIS2ICLX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

	return 0;
}
#endif /* CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES */
