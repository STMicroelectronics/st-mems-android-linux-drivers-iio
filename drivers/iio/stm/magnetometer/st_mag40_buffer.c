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
					ts - cdata->ts_irq,
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

static irqreturn_t st_mag40_buffer_thread_handler(int irq, void *p)
{
	u8 buffer[ALIGN(ST_MAG40_OUT_LEN, sizeof(s64)) + sizeof(s64)];
	struct iio_poll_func *pf = p;
	struct iio_dev *iio_dev = pf->indio_dev;
	struct st_mag40_data *cdata = iio_priv(iio_dev);
	int err;

	st_mag40_update_timestamp(cdata);

	err = cdata->tf->read(cdata, ST_MAG40_OUTX_L_ADDR,
			      ST_MAG40_OUT_LEN, buffer);
	if (err < 0)
		goto out;

	/* discard samples generated during the turn-on time */
	if (cdata->samples_to_discard > 0) {
		cdata->samples_to_discard--;
		goto out;
	}

	iio_push_to_buffers_with_timestamp(iio_dev, buffer, cdata->ts);
	cdata->ts += cdata->delta_ts;

out:
	/* check if using irq trigger or external trigger */
	if (cdata->irq > 0)
		iio_trigger_notify_done(cdata->iio_trig);
	else
		iio_trigger_notify_done(iio_dev->trig);

	return IRQ_HANDLED;
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
	.postdisable = st_mag40_buffer_postdisable,
};

int st_mag40_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct st_mag40_data *cdata = iio_priv(iio_trigger_get_drvdata(trig));
	int err;

	err = st_mag40_write_register(cdata, ST_MAG40_INT_DRDY_ADDR,
				      ST_MAG40_INT_DRDY_MASK, state);

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

	err = devm_request_threaded_irq(cdata->dev, cdata->irq,
					st_mag40_trigger_irq_handler, st_mag40_trigger_thread_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					cdata->name, cdata);
	if (err)
		return err;

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

	return 0;
}
