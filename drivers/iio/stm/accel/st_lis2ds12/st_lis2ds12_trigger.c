// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2ds12 driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2015 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>
#include <linux/version.h>

#include "st_lis2ds12.h"

static void lis2ds12_event_management(struct lis2ds12_data *cdata, u8 int_reg_val,
				      u8 ck_gate_val)
{
	if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_TAP) &&
	    (int_reg_val & LIS2DS12_TAP_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DS12_TAP],
			       IIO_UNMOD_EVENT_CODE(STM_IIO_TAP, 0,
			       IIO_EV_TYPE_THRESH,
			       IIO_EV_DIR_EITHER),
			       cdata->timestamp);

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_DOUBLE_TAP) &&
	    (int_reg_val & LIS2DS12_DOUBLE_TAP_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DS12_DOUBLE_TAP],
			       IIO_UNMOD_EVENT_CODE(STM_IIO_TAP_TAP, 0,
			       IIO_EV_TYPE_THRESH,
			       IIO_EV_DIR_EITHER),
			       cdata->timestamp);

	if (ck_gate_val & LIS2DS12_FUNC_CK_GATE_STEP_D_MASK) {
		if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_STEP_D))
			iio_push_event(cdata->iio_sensors_dev[LIS2DS12_STEP_D],
				       IIO_UNMOD_EVENT_CODE(IIO_STEPS, 0,
				       IIO_EV_TYPE_THRESH,
				       IIO_EV_DIR_EITHER),
				       cdata->timestamp);

		if(CHECK_BIT(cdata->enabled_sensor, LIS2DS12_STEP_C))
			lis2ds12_read_step_c(cdata);
	}

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_TILT) &&
			(ck_gate_val & LIS2DS12_FUNC_CK_GATE_TILT_INT_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DS12_TILT],
			       IIO_UNMOD_EVENT_CODE(STM_IIO_TILT, 0,
			       IIO_EV_TYPE_THRESH,
			       IIO_EV_DIR_EITHER),
			       cdata->timestamp);

	if (CHECK_BIT(cdata->enabled_sensor, LIS2DS12_SIGN_M) &&
			(ck_gate_val & LIS2DS12_FUNC_CK_GATE_SIGN_M_DET_MASK))
		iio_push_event(cdata->iio_sensors_dev[LIS2DS12_SIGN_M],
			       IIO_UNMOD_EVENT_CODE(STM_IIO_SIGN_MOTION, 0,
			       IIO_EV_TYPE_THRESH,
			       IIO_EV_DIR_EITHER),
			       cdata->timestamp);
}

static inline s64 st_lis2ds12_ewma(s64 old, s64 new, int weight)
{
	s64 diff, incr;

	diff = new - old;
	incr = div_s64((LIS2DS12_EWMA_DIV - weight) * diff, LIS2DS12_EWMA_DIV);

	return old + incr;
}

static irqreturn_t lis2ds12_irq_handler(int irq, void *private)
{
	struct lis2ds12_data *cdata = private;
	u8 ewma_level;
	s64 ts;

	ewma_level = (cdata->common_odr >= 100) ? 120 : 96;

	ts = lis2ds12_get_time_ns(cdata);
	cdata->accel_deltatime = st_lis2ds12_ewma(cdata->accel_deltatime,
						  ts - cdata->timestamp,
						  ewma_level);
	cdata->timestamp = ts;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t lis2ds12_irq_thread(int irq, void *private)
{
	u8 status, func;
	struct lis2ds12_data *cdata = private;

	if (cdata->hwfifo_enabled) {
		mutex_lock(&cdata->fifo_lock);
		lis2ds12_read_fifo(cdata, true);
		mutex_unlock(&cdata->fifo_lock);
	} else {
		cdata->tf->read(cdata, LIS2DS12_STATUS_DUP_ADDR, 1,
				&status, true);
		if (status & (LIS2DS12_DRDY_MASK))
			lis2ds12_read_xyz(cdata);
	}

	if (cdata->enabled_sensor & ~(1 << LIS2DS12_ACCEL)) {
		cdata->tf->read(cdata, LIS2DS12_FUNC_CK_GATE_ADDR, 1,
				&func, true);
		cdata->tf->read(cdata, LIS2DS12_STATUS_DUP_ADDR, 1,
				&status, true);
		if (status & (LIS2DS12_EVENT_MASK | LIS2DS12_FUNC_CK_GATE_MASK))
			lis2ds12_event_management(cdata, status, func);
	}

	return IRQ_HANDLED;
}

int lis2ds12_allocate_triggers(struct lis2ds12_data *cdata,
			       const struct iio_trigger_ops *trigger_ops)
{
	int err, i;

	err = devm_request_threaded_irq(cdata->dev,
					cdata->irq,
					lis2ds12_irq_handler,
					lis2ds12_irq_thread,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					cdata->name, cdata);
	if (err) {
		dev_err(cdata->dev,
			"failed to request threaded irq %d.\n",
			cdata->irq);

		return err;
	}

	for (i = 0; i < LIS2DS12_SENSORS_NUMB; i++) {
		if (!cdata->iio_sensors_dev[i])
			continue;

		cdata->iio_trig[i] = devm_iio_trigger_alloc(cdata->dev,
					       "%s-trigger",
					       cdata->iio_sensors_dev[i]->name);
		if (!cdata->iio_trig[i]) {
			dev_err(cdata->dev,
				"failed to allocate iio trigger.\n");
			err = -ENOMEM;

			return err;
		}
		iio_trigger_set_drvdata(cdata->iio_trig[i],
					cdata->iio_sensors_dev[i]);
		cdata->iio_trig[i]->ops = trigger_ops;
		cdata->iio_trig[i]->dev.parent = cdata->dev;

		err = devm_iio_trigger_register(cdata->dev, cdata->iio_trig[i]);
		if (err < 0) {
			dev_err(cdata->dev,
				"failed to register iio trigger.\n");

			return err;
		}

		cdata->iio_sensors_dev[i]->trig =
					    iio_trigger_get(cdata->iio_trig[i]);
	}

	return 0;
}
EXPORT_SYMBOL(lis2ds12_allocate_triggers);

