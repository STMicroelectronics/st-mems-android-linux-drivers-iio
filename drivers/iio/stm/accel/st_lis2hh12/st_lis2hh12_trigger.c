// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2hh12 driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>
#include <linux/version.h>

#include "st_lis2hh12.h"

static irqreturn_t lis2hh12_irq_management(int irq, void *private)
{
	struct lis2hh12_data *cdata = private;
	u8 status;

	cdata->timestamp =
			iio_get_time_ns(cdata->iio_sensors_dev[LIS2HH12_ACCEL]);

	if (cdata->hwfifo_enabled) {
		lis2hh12_read_register(cdata, LIS2HH12_FIFO_STATUS_ADDR,
				       1, &status);

		if (status & LIS2HH12_FIFO_SRC_FTH_MASK)
			lis2hh12_read_fifo(cdata, true);
	} else {
		lis2hh12_read_register(cdata, LIS2HH12_STATUS_ADDR,
				       1, &status);

		if (status & LIS2HH12_DATA_XYZ_RDY)
			lis2hh12_read_xyz(cdata);
	}

	return IRQ_HANDLED;
}

int lis2hh12_allocate_triggers(struct lis2hh12_data *cdata,
			       const struct iio_trigger_ops *trigger_ops)
{
	int err, i;

	for (i = 0;
	     i < ARRAY_SIZE(lis2hh12_main_sensor_list);
	     i++) {
		enum lis2hh12_sensor_id id = lis2hh12_main_sensor_list[i];

		if (!cdata->iio_sensors_dev[id])
			continue;

		cdata->iio_trig[id] = devm_iio_trigger_alloc(cdata->dev,
					       "%s-trigger%d",
					       cdata->iio_sensors_dev[id]->name,
					       id);
		if (!cdata->iio_trig[id]) {
			dev_err(cdata->dev,
				"failed to allocate iio trigger.\n");

			return -ENOMEM;
		}

		iio_trigger_set_drvdata(cdata->iio_trig[id],
					cdata->iio_sensors_dev[id]);
		cdata->iio_trig[id]->ops = trigger_ops;
		cdata->iio_trig[id]->dev.parent = cdata->dev;

		err = devm_iio_trigger_register(cdata->dev,
						cdata->iio_trig[id]);
		if (err < 0) {
			dev_err(cdata->dev,
				"failed to register iio trigger %d\n", id);

			return err;
		}

		cdata->iio_sensors_dev[id]->trig =
					   iio_trigger_get(cdata->iio_trig[id]);
	}

	err = devm_request_threaded_irq(cdata->dev, cdata->irq, NULL,
					lis2hh12_irq_management,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					cdata->name, cdata);
	if (err) {
		dev_err(cdata->dev, "failed to request trigger irq %d\n",
			cdata->irq);

		return err;
	}

	return 0;
}
EXPORT_SYMBOL(lis2hh12_allocate_triggers);
