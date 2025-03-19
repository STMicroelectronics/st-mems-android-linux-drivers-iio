// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lsm6dsv16bx hwtimestamp library driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
 */
#include <asm/unaligned.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/iio.h>
#include <linux/module.h>

#include "st_lsm6dsv16bx.h"

#define ST_LSM6DSV16BX_TSYNC_OFFSET_NS		(300 * 1000LL)
#define ST_LSM6DSV16BX_TSYNC_DECREMENT(_id)	do { \
							if ((hw->enable_mask & BIT_ULL(_id)) && \
							    (hw->timesync_c[_id] > 0)) { \
								hw->timesync_c[_id]--; \
							} \
						} while (0)

static void st_lsm6dsv16bx_read_hw_timestamp(struct st_lsm6dsv16bx_hw *hw)
{
	s64 timestamp_hw_global;
	s64 eventLSB, eventMSB;
	__le32 timestamp_hw;
	s64 timestamp_cpu;
	__le32 tmp;
	int err;

	err = st_lsm6dsv16bx_read_locked(hw, ST_LSM6DSV16BX_REG_TIMESTAMP0_ADDR,
				      (u8 *)&timestamp_hw,
				      sizeof(timestamp_hw));
	if (err < 0)
		return;

	timestamp_cpu = iio_get_time_ns(hw->iio_devs[0]) -
					ST_LSM6DSV16BX_TSYNC_OFFSET_NS;

	eventLSB = IIO_EVENT_CODE(IIO_COUNT, 0, 0, 0,
				  STM_IIO_EV_TYPE_TIME_SYNC, 0, 0, 0);
	eventMSB = IIO_EVENT_CODE(IIO_COUNT, 0, 0, 1,
				  STM_IIO_EV_TYPE_TIME_SYNC, 0, 0, 0);

	spin_lock_irq(&hw->hwtimestamp_lock);
	timestamp_hw_global = (hw->hw_timestamp_global & GENMASK_ULL(63, 32)) |
			      (u32)le32_to_cpu(timestamp_hw);

	ST_LSM6DSV16BX_TSYNC_DECREMENT(ST_LSM6DSV16BX_ID_GYRO);
	ST_LSM6DSV16BX_TSYNC_DECREMENT(ST_LSM6DSV16BX_ID_ACC);
	ST_LSM6DSV16BX_TSYNC_DECREMENT(ST_LSM6DSV16BX_ID_TEMP);

	if ((hw->timesync_c[ST_LSM6DSV16BX_ID_GYRO] == 0) &&
	    (hw->timesync_c[ST_LSM6DSV16BX_ID_ACC] == 0) &&
	    (hw->timesync_c[ST_LSM6DSV16BX_ID_TEMP] == 0)) {
		hw->timesync_ktime = ktime_set(0, ST_LSM6DSV16BX_DEFAULT_KTIME);
	}
	spin_unlock_irq(&hw->hwtimestamp_lock);

	tmp = cpu_to_le32((u32)timestamp_hw_global);
	memcpy(&((int8_t *)&eventLSB)[0], &tmp, sizeof(tmp));

	tmp = cpu_to_le32((u32)(timestamp_hw_global >> 32));
	memcpy(&((int8_t *)&eventMSB)[0], &tmp, sizeof(tmp));

	if (hw->enable_mask & BIT_ULL(ST_LSM6DSV16BX_ID_GYRO)) {
		iio_push_event(hw->iio_devs[ST_LSM6DSV16BX_ID_GYRO], eventLSB,
			       timestamp_cpu);
		iio_push_event(hw->iio_devs[ST_LSM6DSV16BX_ID_GYRO], eventMSB,
			       timestamp_cpu);
	}
	if (hw->enable_mask & BIT_ULL(ST_LSM6DSV16BX_ID_ACC)) {
		iio_push_event(hw->iio_devs[ST_LSM6DSV16BX_ID_ACC], eventLSB,
			       timestamp_cpu);
		iio_push_event(hw->iio_devs[ST_LSM6DSV16BX_ID_ACC], eventMSB,
			       timestamp_cpu);
	}
	if (hw->enable_mask & BIT_ULL(ST_LSM6DSV16BX_ID_TEMP)) {
		iio_push_event(hw->iio_devs[ST_LSM6DSV16BX_ID_TEMP], eventLSB,
			       timestamp_cpu);
		iio_push_event(hw->iio_devs[ST_LSM6DSV16BX_ID_TEMP], eventMSB,
			       timestamp_cpu);
	}
}

static void st_lsm6dsv16bx_timesync_fn(struct work_struct *work)
{
	struct st_lsm6dsv16bx_hw *hw = container_of(work, struct st_lsm6dsv16bx_hw,
						 timesync_work);

	st_lsm6dsv16bx_read_hw_timestamp(hw);
}

static enum hrtimer_restart st_lsm6dsv16bx_timer_fn(struct hrtimer *timer)
{
	struct st_lsm6dsv16bx_hw *hw;

	hw = container_of(timer, struct st_lsm6dsv16bx_hw, timesync_timer);
	hrtimer_forward(timer, hrtimer_cb_get_time(timer), hw->timesync_ktime);
	queue_work(hw->timesync_workqueue, &hw->timesync_work);

	return HRTIMER_RESTART;
}

int st_lsm6dsv16bx_hwtimesync_init(struct st_lsm6dsv16bx_hw *hw)
{
	memset(hw->timesync_c, 0, sizeof(hw->timesync_c));
	hw->timesync_ktime = ktime_set(0, ST_LSM6DSV16BX_DEFAULT_KTIME);
	hrtimer_init(&hw->timesync_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hw->timesync_timer.function = st_lsm6dsv16bx_timer_fn;

	spin_lock_init(&hw->hwtimestamp_lock);
	hw->hw_timestamp_global = 0;

	hw->timesync_workqueue = create_singlethread_workqueue("st_lsm6dsv16bx_workqueue");
	if (!hw->timesync_workqueue)
		return -ENOMEM;

	INIT_WORK(&hw->timesync_work, st_lsm6dsv16bx_timesync_fn);

	return 0;
}
EXPORT_SYMBOL(st_lsm6dsv16bx_hwtimesync_init);
