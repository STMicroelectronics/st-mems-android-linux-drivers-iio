// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_mag40 driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/version.h>

#include "st_mag40_core.h"

#define ST_MAG40_EWMA_DIV			128
static inline s64 st_mag40_ewma(s64 old, s64 new, int weight)
{
	s64 diff, incr;

	diff = new - old;
	incr = div_s64((ST_MAG40_EWMA_DIV - weight) * diff,
			ST_MAG40_EWMA_DIV);

	return old + incr;
}

static void st_mag40_update_timestamp(struct st_mag40_data *cdata)
{
	struct iio_dev *iio_dev = dev_get_drvdata(cdata->dev);
	u8 weight = (cdata->odr >= 50) ? 96 : 0;
	s64 ts;

	ts = st_mag40_get_timestamp(iio_dev);
	cdata->delta_ts = st_mag40_ewma(cdata->delta_ts,
					ts - cdata->ts,
					weight);
	cdata->ts_irq = ts;
}

static irqreturn_t st_mag40_trigger_irq_handler(int irq, void *private)
{
	struct st_mag40_data *cdata = private;

	st_mag40_update_timestamp(cdata);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_mag40_trigger_thread_handler(int irq, void *private)
{
	struct st_mag40_data *cdata = private;
	u8 status;
	int err;

	err = cdata->tf->read(cdata, ST_MAG40_STATUS_ADDR,
			      sizeof(status), &status);
	if (err < 0)
		return IRQ_HANDLED;

	if (!(status & ST_MAG40_AVL_DATA_MASK))
		return IRQ_NONE;

#if KERNEL_VERSION(6, 4, 0) <= LINUX_VERSION_CODE
	iio_trigger_poll_nested(cdata->iio_trig);
#else /* LINUX_VERSION_CODE */
	iio_trigger_poll_chained(cdata->iio_trig);
#endif /* LINUX_VERSION_CODE */

	return IRQ_HANDLED;
}

static int st_mag40_push_data(struct iio_dev *iio_dev)
{
	u8 buffer[ALIGN(ST_MAG40_OUT_LEN, sizeof(s64)) + sizeof(s64)];
	struct st_mag40_data *cdata = iio_priv(iio_dev);
	int err;

	err = cdata->tf->read(cdata, ST_MAG40_OUTX_L_ADDR,
			      ST_MAG40_OUT_LEN, buffer);
	if (err < 0)
		return err;

	/* discard samples generated during the turn-on time */
	if (cdata->samples_to_discard > 0) {
		cdata->samples_to_discard--;
		return 0;
	}

	/* flush events timestamp must be equal to the last data timestamp */
	cdata->last_timestamp = cdata->ts;
	iio_push_to_buffers_with_timestamp(iio_dev, buffer,
					   cdata->last_timestamp);
	cdata->ts += cdata->delta_ts;

	return 0;
}

static irqreturn_t st_mag40_buffer_thread_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct st_mag40_data *cdata = iio_priv(iio_dev);
	int err;

	mutex_lock(&cdata->flush_lock);
	st_mag40_update_timestamp(cdata);
	err = st_mag40_push_data(iio_dev);
	mutex_unlock(&cdata->flush_lock);

	/* check if using irq trigger or external trigger */
	if (cdata->irq > 0)
		iio_trigger_notify_done(cdata->iio_trig);
	else
		iio_trigger_notify_done(iio_dev->trig);

	return IRQ_HANDLED;
}

ssize_t st_mag40_flush_hwfifo(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_mag40_data *cdata = iio_priv(iio_dev);
	s64 timestamp;
	s64 code;
	int err;
	u8 drdy;

	mutex_lock(&cdata->flush_lock);
	err = cdata->tf->read(cdata, ST_MAG40_STATUS_ADDR,
			      sizeof(drdy), &drdy);
	if (err < 0)
		goto out;

	/* check for any pending data in output registers */
	if (drdy & ST_MAG40_STATUS_ZYXDA_MASK)
		err = st_mag40_push_data(iio_dev);

	timestamp = cdata->last_timestamp;

	code = IIO_UNMOD_EVENT_CODE(IIO_MAGN, -1,
				    STM_IIO_EV_TYPE_FIFO_FLUSH,
				    IIO_EV_DIR_EITHER);
	iio_push_event(iio_dev, code, timestamp);

out:
	mutex_unlock(&cdata->flush_lock);

	return err < 0 ? err : count;
}

static int st_mag40_buffer_preenable(struct iio_dev *indio_dev)
{
	struct st_mag40_data *cdata = iio_priv(indio_dev);

	return st_mag40_set_enable(cdata, true);
}

static int st_mag40_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct st_mag40_data *cdata = iio_priv(indio_dev);
	int err;

	err = st_mag40_set_enable(cdata, false);

	return err < 0 ? err : 0;
}

static const struct iio_buffer_setup_ops st_mag40_buffer_setup_ops = {
	.preenable = st_mag40_buffer_preenable,
#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
#endif /* LINUX_VERSION_CODE */
	.postdisable = st_mag40_buffer_postdisable,
};

int st_mag40_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct st_mag40_data *cdata = iio_priv(iio_trigger_get_drvdata(trig));
	struct iio_dev *iio_dev = dev_get_drvdata(cdata->dev);
	int err;

	err = st_mag40_write_register(cdata, ST_MAG40_INT_DRDY_ADDR,
				      ST_MAG40_INT_DRDY_MASK, state);

	/* set timestamp after enabling the sensor */
	if (state)
		cdata->ts = st_mag40_get_timestamp(iio_dev);

	return err < 0 ? err : 0;
}

int st_mag40_allocate_ring(struct iio_dev *iio_dev)
{
	struct st_mag40_data *cdata = iio_priv(iio_dev);

	return  devm_iio_triggered_buffer_setup(cdata->dev, iio_dev, NULL,
						st_mag40_buffer_thread_handler,
						&st_mag40_buffer_setup_ops);
}

static const struct iio_trigger_ops st_mag40_trigger_ops = {
	.set_trigger_state = st_mag40_trig_set_state,
};

int st_mag40_allocate_trigger(struct iio_dev *iio_dev)
{
	struct st_mag40_data *cdata = iio_priv(iio_dev);
	int err;

	cdata->iio_trig = devm_iio_trigger_alloc(cdata->dev, "%s-trigger",
						 iio_dev->name);
	if (!cdata->iio_trig) {
		dev_err(cdata->dev, "failed to allocate iio trigger.\n");
		return -ENOMEM;
	}
	iio_trigger_set_drvdata(cdata->iio_trig, iio_dev);
	cdata->iio_trig->ops = &st_mag40_trigger_ops;
	cdata->iio_trig->dev.parent = cdata->dev;

	err = devm_iio_trigger_register(cdata->dev, cdata->iio_trig);
	if (err < 0) {
		dev_err(cdata->dev, "failed to register iio trigger.\n");
		return err;
	}
	iio_dev->trig = iio_trigger_get(cdata->iio_trig);

	return devm_request_threaded_irq(cdata->dev, cdata->irq,
					 st_mag40_trigger_irq_handler,
					 st_mag40_trigger_thread_handler,
					 IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					 cdata->name, cdata);
}
