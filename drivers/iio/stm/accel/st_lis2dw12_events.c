// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lis2dw12 xl based events function sensor driver
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

#include "st_lis2dw12.h"

#define ST_LIS2DW12_IS_EVENT_ENABLED(_event_id) (!!(hw->enable_ev_mask & \
						    BIT_ULL(_event_id)))

static const struct iio_event_spec st_lis2dw12_flush_event = {
	.type = (enum iio_event_type)STM_IIO_EV_TYPE_FIFO_FLUSH,
	.dir = IIO_EV_DIR_EITHER,
};

static const struct iio_event_spec st_lis2dw12_wakeup_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			 BIT(IIO_EV_INFO_ENABLE) |
			 BIT(IIO_EV_INFO_PERIOD),
};

static const struct iio_event_spec st_lis2dw12_freefall_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_FALLING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			 BIT(IIO_EV_INFO_ENABLE),
};

static const struct iio_event_spec st_lis2dw12_6D_event = {
	.type = IIO_EV_TYPE_CHANGE,
	.dir = IIO_EV_DIR_EITHER,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			 BIT(IIO_EV_INFO_ENABLE),
};

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static const struct iio_event_spec st_lis2dw12_tap_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_SINGLETAP,
	.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
			       BIT(IIO_EV_INFO_ENABLE) |
			       BIT(IIO_EV_INFO_RESET_TIMEOUT),
};

static const struct iio_event_spec st_lis2dw12_dtap_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_DOUBLETAP,
	.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
			       BIT(IIO_EV_INFO_ENABLE) |
			       BIT(IIO_EV_INFO_RESET_TIMEOUT) |
			       BIT(IIO_EV_INFO_TAP2_MIN_DELAY),
};
#endif /* LINUX_VERSION_CODE */

struct st_lis2dw12_event_t {
	enum st_lis2dw12_event_id id;
	char *name;
	u8 irq_mask;
	int req_odr;
};

static const struct st_lis2dw12_events_t {
	enum st_lis2dw12_hw_id hw_ids[ST_LIS2DW12_MAX_ID];
	struct st_lis2dw12_event_t event[ST_LIS2DW12_EVENT_MAX];
} st_lis2dw12_events[] = {
	[0] = {
		.hw_ids = { ST_LIS2DW12_ID, ST_IIS2DLPC_ID, ST_AIS2IH_ID },
		.event = {
			[ST_LIS2DW12_EVENT_FF] = {
				.id = ST_LIS2DW12_EVENT_FF,
				.name = "free_fall",
				.irq_mask = ST_LIS2DW12_FF_INT1_MASK,
				.req_odr = ST_LIS2DW12_MIN_ODR_IN_FREEFALL,
			},
			[ST_LIS2DW12_EVENT_WAKEUP] = {
				.id = ST_LIS2DW12_EVENT_WAKEUP,
				.name = "wake_up",
				.irq_mask = ST_LIS2DW12_WU_INT1_MASK,
				.req_odr = ST_LIS2DW12_MIN_ODR_IN_WAKEUP,
			},
			[ST_LIS2DW12_EVENT_6D] = {
				.id = ST_LIS2DW12_EVENT_6D,
				.name = "sixD",
				.irq_mask = ST_LIS2DW12_6D_INT1_MASK,
				.req_odr = ST_LIS2DW12_MIN_ODR_IN_6D,
			},

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
			[ST_LIS2DW12_EVENT_TAP] = {
				.id = ST_LIS2DW12_EVENT_TAP,
				.name = "tap",
				.irq_mask = ST_LIS2DW12_TAP_INT1_MASK,
				.req_odr = ST_LIS2DW12_MIN_ODR_IN_TAP,
			},
			[ST_LIS2DW12_EVENT_DTAP] = {
				.id = ST_LIS2DW12_EVENT_DTAP,
				.name = "dtap",
				.irq_mask = ST_LIS2DW12_TAP_TAP_INT1_MASK,
				.req_odr = ST_LIS2DW12_MIN_ODR_IN_DTAP,
			},
#endif /* LINUX_VERSION_CODE */

		},
	},
	[1] = {
		.hw_ids = { ST_AIS2DW12_ID },
		.event = {
			[ST_LIS2DW12_EVENT_FF] = {
				.id = ST_LIS2DW12_EVENT_FF,
				.name = "free_fall",
				.irq_mask = ST_LIS2DW12_FF_INT1_MASK,
				.req_odr = ST_LIS2DW12_MIN_ODR_IN_FREEFALL,
			},
			[ST_LIS2DW12_EVENT_WAKEUP] = {
				.id = ST_LIS2DW12_EVENT_WAKEUP,
				.name = "wake_up",
				.irq_mask = ST_LIS2DW12_WU_INT1_MASK,
				.req_odr = ST_LIS2DW12_MIN_ODR_IN_WAKEUP,
			},
			[ST_LIS2DW12_EVENT_6D] = {
				.id = ST_LIS2DW12_EVENT_6D,
				.name = "sixD",
				.irq_mask = ST_LIS2DW12_6D_INT1_MASK,
				.req_odr = ST_LIS2DW12_MIN_ODR_IN_6D,
			},
		},
	},
};

static const struct st_lis2dw12_ff_th st_lis2dw12_free_fall_threshold[] = {
	[0] = { .val = 0x00, .mg = 156 },
	[1] = { .val = 0x01, .mg = 219 },
	[2] = { .val = 0x02, .mg = 250 },
	[3] = { .val = 0x03, .mg = 312 },
	[4] = { .val = 0x04, .mg = 344 },
	[5] = { .val = 0x05, .mg = 406 },
	[6] = { .val = 0x06, .mg = 469 },
	[7] = { .val = 0x07, .mg = 500 },
};

static const struct st_lis2dw12_6D_th st_lis2dw12_6D_threshold[] = {
	[0] = { .val = 0x00, .deg = 80 },
	[1] = { .val = 0x01, .deg = 70 },
	[2] = { .val = 0x02, .deg = 60 },
	[3] = { .val = 0x03, .deg = 50 },
};

static const struct st_lis2dw12_event_t *st_lis2dw12_get_events(struct st_lis2dw12_hw *hw)
{
	int hw_id = hw->settings->id.hw_id, i, j;

	for (i = 0; i < ARRAY_SIZE(st_lis2dw12_events); i++) {
		for (j = 0; j < ST_LIS2DW12_MAX_ID; j++) {
			if (st_lis2dw12_events[i].hw_ids[j] == hw_id)
				goto out;
		}
	}

	if (i == ARRAY_SIZE(st_lis2dw12_events))
		return ERR_PTR(-ENODEV);

out:
	return st_lis2dw12_events[i].event;
}

static inline bool
st_lis2dw12_events_enabled(struct st_lis2dw12_hw *hw)
{
	return (ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_FF) ||
		ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_WAKEUP) ||

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_TAP) ||
		ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_DTAP) ||
#endif /* LINUX_VERSION_CODE */

		ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_6D));
}

static int st_lis2dw12_get_xl_fs(struct st_lis2dw12_hw *hw, u8 *xl_fs)
{
	u8 fs_xl_g[] = { 2, 4, 8, 16 }, fs_xl;
	int err;

	err = st_lis2dw12_read_with_mask(hw, ST_LIS2DW12_CTRL6_ADDR,
					 ST_LIS2DW12_FS_MASK, &fs_xl);
	if (err < 0)
		return err;

	*xl_fs = fs_xl_g[fs_xl];

	return err;
}

static int st_lis2dw12_get_xl_odr(struct st_lis2dw12_hw *hw, int *xl_odr)
{
	int i, err;
	u8 odr_xl;

	err = st_lis2dw12_read_with_mask(hw, ST_LIS2DW12_CTRL1_ADDR,
					 ST_LIS2DW12_ODR_MASK, &odr_xl);
	if (err < 0)
		return err;

	if (odr_xl == 0)
		return 0;

	for (i = 0; i < hw->odr_entry[ST_LIS2DW12_ID_ACC].size; i++) {
		if (odr_xl == hw->odr_entry[ST_LIS2DW12_ID_ACC].odr[i].val)
			break;
	}

	if (i == hw->odr_entry[ST_LIS2DW12_ID_ACC].size)
		return -EINVAL;

	/* for frequency values with decimal part just return the integer */
	*xl_odr = hw->odr_entry[ST_LIS2DW12_ID_ACC].odr[i].hz;

	return err;
}

static int st_lis2dw12_get_default_xl_odr(struct st_lis2dw12_hw *hw,
					  enum st_lis2dw12_event_id id,
					  int *xl_odr)
{
	int err;
	int odr;
	int req_odr;
	static const struct st_lis2dw12_event_t *event;

	err = st_lis2dw12_get_xl_odr(hw, &odr);
	if (err < 0)
		return err;

	event = st_lis2dw12_get_events(hw);
	if (IS_ERR(event))
		return PTR_ERR(event);

	req_odr = event[id].req_odr;

	if (odr > req_odr)
		*xl_odr = odr;
	else
		*xl_odr = req_odr;

	return err;
}

/*
 * st_lis2dw12_set_wake_up_thershold - set wake-up threshold in mg
 * @hw - ST IMU MEMS hw instance
 * @wake_up_thershold_mg - wake-up threshold in mg
 *
 * wake-up thershold register val = (th_mg * 2 ^ 6) / (1000 * FS_XL)
 */
static int st_lis2dw12_set_wake_up_thershold(struct st_lis2dw12_hw *hw,
					     int wake_up_thershold_mg)
{
	u8 wake_up_thershold, max_th;
	int tmp, err;
	u8 fs_xl_g;

	err = st_lis2dw12_get_xl_fs(hw, &fs_xl_g);
	if (err < 0)
		return err;

	tmp = (wake_up_thershold_mg * 64) / (fs_xl_g * 1000);
	wake_up_thershold = (u8)tmp;
	max_th = ST_LIS2DW12_WAKE_UP_THS_MASK >>
		  __ffs(ST_LIS2DW12_WAKE_UP_THS_MASK);
	if (wake_up_thershold > max_th)
		wake_up_thershold = max_th;

	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_WAKE_UP_THS_ADDR,
						 ST_LIS2DW12_WAKE_UP_THS_MASK,
						 wake_up_thershold);
	if (err < 0)
		return err;

	hw->wk_th_mg = wake_up_thershold_mg;

	return 0;
}

/*
 * st_lis2dw12_set_wake_up_duration - set wake-up duration in ms
 * @hw - ST IMU MEMS hw instance
 * @wake_up_duration_ms - wake-up duration in ms
 *
 * wake-up duration register val = (dur_ms * ODR_XL) / (32 * 1000)
 */
static int st_lis2dw12_set_wake_up_duration(struct st_lis2dw12_hw *hw,
					    int wake_up_duration_ms)
{
	int tmp, sensor_odr, err;
	u8 wake_up_duration, max_dur;

	err = st_lis2dw12_get_default_xl_odr(hw, ST_LIS2DW12_EVENT_WAKEUP,
					     &sensor_odr);
	if (err < 0)
		return err;

	tmp = (wake_up_duration_ms * sensor_odr) / 32000;
	wake_up_duration = (u8)tmp;
	max_dur = ST_LIS2DW12_WAKE_UP_DUR_MASK >>
		  __ffs(ST_LIS2DW12_WAKE_UP_DUR_MASK);
	if (wake_up_duration > max_dur)
		wake_up_duration = max_dur;

	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_WAKE_UP_DUR_ADDR,
						 ST_LIS2DW12_WAKE_UP_DUR_MASK,
						 wake_up_duration);
	if (err < 0)
		return err;

	hw->wk_dur_ms = wake_up_duration_ms;

	return 0;
}

/*
 * st_lis2dw12_set_freefall_threshold - set free fall threshold detection mg
 * @hw - ST IMU MEMS hw instance
 * @freefall_threshold_mg - free fall threshold in mg
 */
static int st_lis2dw12_set_freefall_threshold(struct st_lis2dw12_hw *hw,
					      int freefall_threshold_mg)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_lis2dw12_free_fall_threshold); i++) {
		if (freefall_threshold_mg >=
					st_lis2dw12_free_fall_threshold[i].mg)
			break;
	}

	if (i == ARRAY_SIZE(st_lis2dw12_free_fall_threshold))
		return -EINVAL;

	err = st_lis2dw12_write_with_mask_locked(hw,
					ST_LIS2DW12_FREE_FALL_ADDR,
					ST_LIS2DW12_FREE_FALL_THS_MASK,
					st_lis2dw12_free_fall_threshold[i].val);
	if (err < 0)
		return err;

	hw->freefall_threshold = freefall_threshold_mg;

	return 0;
}

/*
 * st_lis2dw12_set_6D_threshold - set 6D threshold detection in degrees
 * @hw - ST IMU MEMS hw instance
 * @sixD_threshold_deg - 6D threshold in degrees
 */
static int st_lis2dw12_set_6D_threshold(struct st_lis2dw12_hw *hw,
					int sixD_threshold_deg)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_lis2dw12_6D_threshold); i++) {
		if (sixD_threshold_deg >= st_lis2dw12_6D_threshold[i].deg)
			break;
	}

	if (i == ARRAY_SIZE(st_lis2dw12_6D_threshold))
		return -EINVAL;

	err = st_lis2dw12_write_with_mask_locked(hw,
					       ST_LIS2DW12_TAP_THS_X_ADDR,
					       ST_LIS2DW12_SIXD_THS_MASK,
					       st_lis2dw12_6D_threshold[i].val);
	if (err < 0)
		return err;

	hw->sixD_threshold = sixD_threshold_deg;

	return 0;
}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
/*
 * st_lis2dw12_set_tap_threshold - set TAP/DTAP threshold detection in mg
 * @hw - ST IMU MEMS hw instance
 * @tap_threshold_mg - TAP/DTAP threshold in mg
 *
 * th_ug thershold register val = (th_ug * 2 ^ 5) / (1000 * FS_XL)
 */
static int st_lis2dw12_set_tap_threshold(struct st_lis2dw12_hw *hw,
					 int tap_threshold_mg)
{
	u8 tap_threshold, max_th;
	int tmp, err;
	u8 fs_xl_g;

	err = st_lis2dw12_get_xl_fs(hw, &fs_xl_g);
	if (err < 0)
		return err;

	tmp = (tap_threshold_mg * 32) / (fs_xl_g * 1000);
	tap_threshold = (u8)tmp;
	max_th = ST_LIS2DW12_TAP_THS_X_MASK >>
		  __ffs(ST_LIS2DW12_TAP_THS_X_MASK);
	if (tap_threshold > max_th)
		tap_threshold = max_th;

	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_TAP_THS_X_ADDR,
						 ST_LIS2DW12_TAP_THS_X_MASK,
						 tap_threshold);
	if (err < 0)
		return err;

	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_TAP_THS_Y_ADDR,
						 ST_LIS2DW12_TAP_THS_Y_MASK,
						 tap_threshold);
	if (err < 0)
		return err;

	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_TAP_THS_Z_ADDR,
						 ST_LIS2DW12_TAP_THS_Z_MASK,
						 tap_threshold);
	if (err < 0)
		return err;

	hw->tap_threshold = tap_threshold_mg;

	return 0;
}

/*
 * st_lis2dw12_set_tap_quiet_time - set TAP/DTAP quiet time in ms
 * @hw - ST IMU MEMS hw instance
 * @tap_quiet_time_ms - TAP/DTAP quiet time in LSB
 */
static int st_lis2dw12_set_tap_quiet_time(struct st_lis2dw12_hw *hw,
					  u8 tap_quiet_time_ms)
{
	u8 tap_quiet_time, max_quite;
	int sensor_odr, err, tmp;

	err = st_lis2dw12_get_default_xl_odr(hw, ST_LIS2DW12_EVENT_TAP,
					     &sensor_odr);
	if (err < 0)
		return err;

	/*
	 * Quiet time is the time after the first detected tap in which there
	 * must not be any overthreshold event. The default value of these bits
	 * is 00b which corresponds to 2/ODR_XL time. If the QUIET[1:0] bits are
	 * set to a different value, 1LSB corresponds to 4/ODR_XL time.
	 */
	if (tap_quiet_time_ms <= (2000 / sensor_odr))
		tmp = 0;
	else
		tmp = (tap_quiet_time_ms * sensor_odr) / 4000;

	tap_quiet_time = (u8)tmp;
	max_quite = ST_LIS2DW12_QUIET_MASK >> __ffs(ST_LIS2DW12_QUIET_MASK);
	if (tap_quiet_time > max_quite)
		tap_quiet_time = max_quite;

	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_INT_DUR_ADDR,
						 ST_LIS2DW12_QUIET_MASK,
						 tap_quiet_time);
	if (err < 0)
		return err;

	hw->tap_quiet_time = tap_quiet_time_ms;

	return 0;
}

/*
 * st_lis2dw12_set_tap_shock_time - set TAP/DTAP shock time in ms
 * @hw - ST IMU MEMS hw instance
 * @tap_shock_time_ms - TAP/DTAP shock time in ms
 */
static int st_lis2dw12_set_tap_shock_time(struct st_lis2dw12_hw *hw,
					  u8 tap_shock_time_ms)
{
	u8 tap_shock_time, max_tap_shock_time;
	int sensor_odr, err, tmp;

	err = st_lis2dw12_get_default_xl_odr(hw, ST_LIS2DW12_EVENT_TAP,
					     &sensor_odr);
	if (err < 0)
		return err;

	/*
	 * Maximum duration is the maximum time of an overthreshold signal
	 * detection to be recognized as a tap event.
	 * The default value of these bits is 00b which corresponds to 4/ODR_XL
	 * time. If the SHOCK[1:0] bits are set to a different value, 1LSB
	 * corresponds to 8/ODR_XL time.
	 */
	if (tap_shock_time_ms <= (4000 / sensor_odr))
		tmp = 0;
	else
		tmp = (tap_shock_time_ms * sensor_odr) / 8000;

	tap_shock_time = (u8)tmp;
	max_tap_shock_time = ST_LIS2DW12_QUIET_MASK >>
			    __ffs(ST_LIS2DW12_QUIET_MASK);
	if (tap_shock_time > max_tap_shock_time)
		tap_shock_time = max_tap_shock_time;

	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_INT_DUR_ADDR,
						 ST_LIS2DW12_SHOCK_MASK,
						 tap_shock_time);
	if (err < 0)
		return err;

	hw->tap_shock_time = tap_shock_time_ms;

	return 0;
}

/*
 * st_lis2dw12_set_dtap_duration - set DTAP durationin ms (min time)
 * @hw - ST IMU MEMS hw instance
 * @dtap_duration_ms - DTAP min delay time in ms
 */
static int st_lis2dw12_set_dtap_duration(struct st_lis2dw12_hw *hw,
					 int dtap_duration_ms)
{
	int sensor_odr, tmp, err;
	u8 dtap_duration, max_dtap_duration;

	err = st_lis2dw12_get_default_xl_odr(hw, ST_LIS2DW12_EVENT_TAP,
					     &sensor_odr);
	if (err < 0)
		return err;

	/*
	 * When double-tap recognition is enabled, this register expresses the
	 * maximum time between two consecutive detected taps to determine a
	 * double-tap event. The default value of these bits is 0000b which
	 * corresponds to 16/ODR_XL time. If the DUR[3:0] bits are set to a
	 * different value, 1LSB corresponds to 32/ODR_XL time.
	 */
	if (dtap_duration_ms <= (16000 / sensor_odr))
		tmp = 0;
	else
		tmp = (dtap_duration_ms * sensor_odr) / 32000;

	dtap_duration = (u8)tmp;
	max_dtap_duration = ST_LIS2DW12_LATENCY_MASK >>
			    __ffs(ST_LIS2DW12_LATENCY_MASK);

	if (dtap_duration > max_dtap_duration)
		dtap_duration = max_dtap_duration;

	err = st_lis2dw12_write_with_mask_locked(hw,
						 ST_LIS2DW12_INT_DUR_ADDR,
						 ST_LIS2DW12_LATENCY_MASK,
						 dtap_duration);
	if (err < 0)
		return err;

	hw->dtap_duration = dtap_duration_ms;

	return 0;
}

#endif /* LINUX_VERSION_CODE */

int st_lis2dw12_read_event_config(struct iio_dev *iio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir)
{
	if (chan->type == IIO_ACCEL) {
		struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
		struct st_lis2dw12_hw *hw = sensor->hw;

		switch (type) {
		case IIO_EV_TYPE_THRESH:
			switch (dir) {
			case IIO_EV_DIR_FALLING:
				return FIELD_GET(BIT(ST_LIS2DW12_EVENT_FF),
						 hw->enable_ev_mask);

			case IIO_EV_DIR_RISING:
				return FIELD_GET(BIT(ST_LIS2DW12_EVENT_WAKEUP),
						 hw->enable_ev_mask);

			default:
				return -EINVAL;
			}
			break;

		case IIO_EV_TYPE_CHANGE:
			switch (dir) {
			case IIO_EV_DIR_EITHER:
				return FIELD_GET(BIT(ST_LIS2DW12_EVENT_6D),
						 hw->enable_ev_mask);

			default:
				return -EINVAL;
			}
			break;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		case IIO_EV_TYPE_GESTURE:
			switch (dir) {
			case IIO_EV_DIR_SINGLETAP:
				return FIELD_GET(BIT(ST_LIS2DW12_EVENT_TAP),
						 hw->enable_ev_mask);

			case IIO_EV_DIR_DOUBLETAP:
				return FIELD_GET(BIT(ST_LIS2DW12_EVENT_DTAP),
						 hw->enable_ev_mask);

			default:
				return -EINVAL;
			}
			break;
#endif /* LINUX_VERSION_CODE */

		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static int
st_lis2dw12_check_events_odr_dependency(struct st_lis2dw12_hw *hw,
					int odr,
					enum st_lis2dw12_event_id id,
					int enable)
{
	int i, ret = 0;
	int current_odr;
	static const struct st_lis2dw12_event_t *event;

	event = st_lis2dw12_get_events(hw);
	if (IS_ERR(event))
		return PTR_ERR(event);

	for (i = 0; i < ST_LIS2DW12_EVENT_MAX; i++) {
		if (event[i].req_odr == 0)
			continue;

		current_odr = event[i].req_odr;

		if (hw->enable_ev_mask & BIT(i)) {
			if ((i == id) && (!enable))
				continue;

			ret = max_t(int, current_odr, enable ? odr : ret);
		} else if (i == id) {
			ret = enable ? max_t(int, current_odr, odr) : ret;
		}
	}

	return ret;
}

int st_lis2dw12_write_event_config(struct iio_dev *iio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   int enable)
{
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	static const struct st_lis2dw12_event_t *event;
	struct st_lis2dw12_hw *hw = sensor->hw;
	int req_odr = 0;
	int id = -1;
	u8 int_val;
	int err;

	if (chan->type == IIO_ACCEL) {
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			switch (dir) {
			/*
			 * this is the ff event, use the dir IIO_EV_DIR_FALLING
			 * because don't exist a specific iio_event_type related
			 * to free fall events
			 */
			case IIO_EV_DIR_FALLING:
				id = ST_LIS2DW12_EVENT_FF;
				break;

			/*
			 * this is the wk event, use the dir IIO_EV_DIR_RISING
			 * because don't exist a specific iio_event_type related
			 * to wakeup events
			 */
			case IIO_EV_DIR_RISING:
				id = ST_LIS2DW12_EVENT_WAKEUP;
				break;

			default:
				return -EINVAL;
			}
			break;

		case IIO_EV_TYPE_CHANGE:
			switch (dir) {
			/*
			 * this is the 6D event, use the type IIO_EV_TYPE_CHANGE
			 * because don't exist a specific iio_event_type related
			 * to 6D events
			 */
			case IIO_EV_DIR_EITHER:
				id = ST_LIS2DW12_EVENT_6D;
				break;

			default:
				return -EINVAL;
			}
			break;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		case IIO_EV_TYPE_GESTURE:
			switch (dir) {
			case IIO_EV_DIR_SINGLETAP:
				id = ST_LIS2DW12_EVENT_TAP;
				break;

			case IIO_EV_DIR_DOUBLETAP:
				err = st_lis2dw12_write_with_mask_locked(hw,
					     ST_LIS2DW12_WAKE_UP_THS_ADDR,
					     ST_LIS2DW12_SINGLE_DOUBLE_TAP_MASK,
					     enable);
				if (err < 0)
					return err;

				id = ST_LIS2DW12_EVENT_DTAP;
				break;

			default:
				return -EINVAL;
			}
			break;
#endif /* LINUX_VERSION_CODE */

		default:
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	err = st_lis2dw12_read_locked(hw, ST_LIS2DW12_CTRL4_INT1_CTRL_ADDR,
				      &int_val, 1);
	if (err < 0)
		return err;

	event = st_lis2dw12_get_events(hw);
	if (IS_ERR(event))
		return PTR_ERR(event);

	int_val = st_lis2dw12_manipulate_bit(int_val,
					     event[id].irq_mask,
					     enable);

	err = st_lis2dw12_write_locked(hw, ST_LIS2DW12_CTRL4_INT1_CTRL_ADDR,
				       (u8 *)&int_val, 1);
	if (err < 0)
		return err;

	/* check enabled events ODR dependency */
	req_odr = st_lis2dw12_check_events_odr_dependency(hw,
							  event[id].req_odr,
							  id, enable);

	hw->req_odr_events = req_odr;
	err = st_lis2dw12_set_odr(iio_priv(hw->iio_devs[ST_LIS2DW12_ID_ACC]),
				  req_odr);
	if (err < 0)
		return err;

	if (enable == 0)
		hw->enable_ev_mask &= ~BIT_ULL(id);
	else
		hw->enable_ev_mask |= BIT_ULL(id);

	return err;
}

int st_lis2dw12_read_event_value(struct iio_dev *iio_dev,
				 const struct iio_chan_spec *chan,
				 enum iio_event_type type,
				 enum iio_event_direction dir,
				 enum iio_event_info info,
				 int *val, int *val2)
{
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;
	int err = -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			/*
			 * FF is classified as threshold event with dir
			 * falling
			 */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				*val = (int)hw->freefall_threshold;

				return IIO_VAL_INT;

			default:
				break;
			}
			break;

		case IIO_EV_DIR_RISING:
			/*
			 * Wake-up is classified as threshold event with dir
			 * rising
			 */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				*val = (int)hw->wk_th_mg;

				return IIO_VAL_INT;

			case IIO_EV_INFO_PERIOD:
				*val = (int)hw->wk_dur_ms;

				return IIO_VAL_INT;

			default:
				break;
			}
			break;

		default:
			break;
		}
		break;

	case IIO_EV_TYPE_CHANGE:
		switch (dir) {
		case IIO_EV_DIR_EITHER:
			/* 6D is classified as change event with dir either */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				*val = (int)hw->sixD_threshold;

				return IIO_VAL_INT;

			default:
				break;
			}
			break;

		default:
			break;
		}
		break;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	case IIO_EV_TYPE_GESTURE:
		switch (dir) {
		case IIO_EV_DIR_SINGLETAP:
			switch (info) {
			case IIO_EV_INFO_VALUE:
				*val = (int)hw->tap_threshold;

				return IIO_VAL_INT;

			case IIO_EV_INFO_RESET_TIMEOUT:
				*val = (int)hw->tap_quiet_time;

				return IIO_VAL_INT;

			default:
				break;
			}
			break;

		case IIO_EV_DIR_DOUBLETAP:
			switch (info) {
			case IIO_EV_INFO_VALUE:
				*val = (int)hw->tap_threshold;

				return IIO_VAL_INT;

			case IIO_EV_INFO_RESET_TIMEOUT:
				*val = (int)hw->tap_quiet_time;

				return IIO_VAL_INT;

			case IIO_EV_INFO_TAP2_MIN_DELAY:
				*val = (int)hw->dtap_duration;

				return IIO_VAL_INT;

			default:
				break;
			}
			break;

		default:
			break;
		}
		break;
#endif /* LINUX_VERSION_CODE */

	default:
		break;
	}

	return err;
}

int st_lis2dw12_write_event_value(struct iio_dev *iio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir,
				  enum iio_event_info info,
				  int val, int val2)
{
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;
	int err = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			switch (info) {
			/* Free Fall event configuration */
			case IIO_EV_INFO_VALUE:
				err = st_lis2dw12_set_freefall_threshold(hw,
									 val);
				break;

			default:
				break;
			}
			break;

		case IIO_EV_DIR_RISING:
			/* wake-up event configuration */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				err = st_lis2dw12_set_wake_up_thershold(hw,
									val);
				break;

			case IIO_EV_INFO_PERIOD:
				err = st_lis2dw12_set_wake_up_duration(hw,
								       val);
				break;

			default:
				break;
			}
			break;
		default:
			break;
		}
		break;

	case IIO_EV_TYPE_CHANGE:
		switch (dir) {
		case IIO_EV_DIR_EITHER:
			/* 6D event configuration */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				err = st_lis2dw12_set_6D_threshold(hw, val);
				break;

			default:
				break;
			}
			break;

		default:
			break;
		}
		break;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	case IIO_EV_TYPE_GESTURE:
		switch (dir) {
		case IIO_EV_DIR_SINGLETAP:
			switch (info) {
			case IIO_EV_INFO_VALUE:
				err = st_lis2dw12_set_tap_threshold(hw, val);
				break;

			case IIO_EV_INFO_RESET_TIMEOUT:
				err = st_lis2dw12_set_tap_quiet_time(hw, val);
				break;

			default:
				break;
			}
			break;

		case IIO_EV_DIR_DOUBLETAP:
			switch (info) {
			case IIO_EV_INFO_VALUE:
				err = st_lis2dw12_set_tap_threshold(hw, val);
				break;

			case IIO_EV_INFO_RESET_TIMEOUT:
				err = st_lis2dw12_set_tap_quiet_time(hw, val);
				break;

			case IIO_EV_INFO_TAP2_MIN_DELAY:
				err = st_lis2dw12_set_dtap_duration(hw, val);
				break;

			default:
				break;
			}
			break;

		default:
			break;
		}
		break;
#endif /* LINUX_VERSION_CODE */

	default:
		break;
	}

	return err;
}

int st_lis2dw12_event_handler(struct st_lis2dw12_hw *hw)
{
	struct iio_dev *iio_dev;
	u8 reg_src[3];
	s64 event;
	int err;

	if (!st_lis2dw12_events_enabled(hw))
		return IRQ_HANDLED;

	err = regmap_bulk_read(hw->regmap,
			       ST_LIS2DW12_WU_SRC_ADDR,
			       reg_src, sizeof(reg_src));
	if (err < 0)
		return IRQ_HANDLED;

	if ((reg_src[0] & ST_LIS2DW12_WAKE_UP_SRC_FF_IA_MASK) &&
	    ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_FF)) {
		iio_dev = hw->iio_devs[ST_LIS2DW12_ID_ACC];

		event = IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
					     IIO_EV_TYPE_THRESH,
					     IIO_EV_DIR_FALLING);
		iio_push_event(iio_dev, event,
			       iio_get_time_ns(iio_dev));
	}

	if ((reg_src[0] & ST_LIS2DW12_WAKE_UP_SRC_WU_IA_MASK) &&
	    ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_WAKEUP)) {
		iio_dev = hw->iio_devs[ST_LIS2DW12_ID_ACC];
		if (reg_src[0] & ST_LIS2DW12_Z_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Z,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}

		if (reg_src[0] & ST_LIS2DW12_Y_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Y,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}

		if (reg_src[0] & ST_LIS2DW12_X_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_X,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}
	}

	if ((reg_src[2] & ST_LIS2DW12_D6D_SRC_D6D_IA_MASK) &&
	    ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_6D)) {
		u8 dir;

		iio_dev = hw->iio_devs[ST_LIS2DW12_ID_ACC];
		if (reg_src[2] & (ST_LIS2DW12_ZH_MASK | ST_LIS2DW12_ZL_MASK)) {
			dir = (reg_src[2] & ST_LIS2DW12_ZH_MASK) ?
			      IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Z,
							  IIO_EV_TYPE_CHANGE,
							  dir),
				       iio_get_time_ns(iio_dev));
		}

		if (reg_src[2] & (ST_LIS2DW12_YH_MASK | ST_LIS2DW12_YL_MASK)) {
			dir = (reg_src[2] & ST_LIS2DW12_YH_MASK) ?
			      IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Y,
							  IIO_EV_TYPE_CHANGE,
							  dir),
				       iio_get_time_ns(iio_dev));
		}

		if (reg_src[2] & (ST_LIS2DW12_XH_MASK | ST_LIS2DW12_XL_MASK)) {
			dir = (reg_src[2] & ST_LIS2DW12_XH_MASK) ?
			      IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_X,
							  IIO_EV_TYPE_CHANGE,
							  dir),
				       iio_get_time_ns(iio_dev));
		}
	}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	if ((reg_src[1] & ST_LIS2DW12_SINGLE_TAP_IA_MASK) &&
	    ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_TAP)) {
		u8 mod;

		iio_dev = hw->iio_devs[ST_LIS2DW12_ID_ACC];
		if (reg_src[1] & ST_LIS2DW12_X_TAP_MASK)
			mod = IIO_MOD_X;
		else if (reg_src[1] & ST_LIS2DW12_Y_TAP_MASK)
			mod = IIO_MOD_Y;
		else if (reg_src[1] & ST_LIS2DW12_Z_TAP_MASK)
			mod = IIO_MOD_Z;
		else
			return IRQ_HANDLED;

		iio_push_event(iio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  mod,
						  IIO_EV_TYPE_GESTURE,
						  IIO_EV_DIR_SINGLETAP),
			       iio_get_time_ns(iio_dev));
	}

	if ((reg_src[1] & ST_LIS2DW12_DOUBLE_TAP_IA_MASK) &&
	    ST_LIS2DW12_IS_EVENT_ENABLED(ST_LIS2DW12_EVENT_DTAP)) {
		u8 mod;

		iio_dev = hw->iio_devs[ST_LIS2DW12_ID_ACC];
		if (reg_src[1] & ST_LIS2DW12_X_TAP_MASK)
			mod = IIO_MOD_X;
		else if (reg_src[1] & ST_LIS2DW12_Y_TAP_MASK)
			mod = IIO_MOD_Y;
		else if (reg_src[1] & ST_LIS2DW12_Z_TAP_MASK)
			mod = IIO_MOD_Z;
		else
			return IRQ_HANDLED;

		iio_push_event(iio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
						  mod,
						  IIO_EV_TYPE_GESTURE,
						  IIO_EV_DIR_DOUBLETAP),
			       iio_get_time_ns(iio_dev));
	}
#endif /* LINUX_VERSION_CODE */

	return IRQ_HANDLED;
}

int st_lis2dw12_update_threshold_events(struct st_lis2dw12_hw *hw)
{
	int ret;

	ret = st_lis2dw12_set_wake_up_thershold(hw, hw->wk_th_mg);
	if (ret < 0)
		return ret;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ret = st_lis2dw12_set_tap_threshold(hw, hw->tap_threshold);
	if (ret < 0)
		return ret;
#endif /* LINUX_VERSION_CODE */

	return ret;
}

int st_lis2dw12_update_duration_events(struct st_lis2dw12_hw *hw)
{
	int ret;

	ret = st_lis2dw12_set_wake_up_duration(hw, hw->wk_dur_ms);
	if (ret < 0)
		return ret;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ret = st_lis2dw12_set_tap_quiet_time(hw, hw->tap_quiet_time);
	if (ret < 0)
		return ret;

	ret = st_lis2dw12_set_dtap_duration(hw, hw->dtap_duration);
	if (ret < 0)
		return ret;

	ret = st_lis2dw12_set_tap_shock_time(hw, hw->tap_shock_time);
	if (ret < 0)
		return ret;
#endif /* LINUX_VERSION_CODE */

	return ret;
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
	return st_lis2dw12_event_handler((struct st_lis2dw12_hw *)private);
}

/*
 * Configure the xl based events function default threshold and duration/delay
 *
 * wake_up_thershold = 100 mg
 * wake_up_duration = 0 ms
 * freefall_threshold = 312 mg
 * 6D_threshold = 60 deg
 * tap_threshold = 570 mg
 * tap_quiet_time = 10 ms
 * dtap_duration = 540 ms
 * tap_shock_time = 40 ms (this parameter is not possible to configure through
 * 			   IIO attribute)
 */
int st_lis2dw12_event_init(struct st_lis2dw12_hw *hw)
{
	int err;
	/* start from last data channel (three axes and timestamp) */
	int i = 4;

	hw->st_lis2dw12_acc_channels[i++] = (struct iio_chan_spec)ST_LIS2DW12_EVENT_CHANNEL(IIO_ACCEL, flush);
	hw->st_lis2dw12_acc_channels[i++] = (struct iio_chan_spec)ST_LIS2DW12_EVENT_CHANNEL(IIO_ACCEL, wakeup);
	hw->st_lis2dw12_acc_channels[i++] = (struct iio_chan_spec)ST_LIS2DW12_EVENT_CHANNEL(IIO_ACCEL, freefall);
	hw->st_lis2dw12_acc_channels[i++] = (struct iio_chan_spec)ST_LIS2DW12_EVENT_CHANNEL(IIO_ACCEL, 6D);

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	if (hw->settings->id.hw_id != ST_AIS2DW12_ID) {
		hw->st_lis2dw12_acc_channels[i++] = (struct iio_chan_spec)ST_LIS2DW12_EVENT_CHANNEL(IIO_ACCEL, tap);
		hw->st_lis2dw12_acc_channels[i++] = (struct iio_chan_spec)ST_LIS2DW12_EVENT_CHANNEL(IIO_ACCEL, dtap);
	}
#endif /* LINUX_VERSION_CODE */

	/* Set default wake-up thershold to 100 mg */
	err = st_lis2dw12_set_wake_up_thershold(hw, 100);
	if (err < 0)
		return err;

	/* Set default wake-up duration to 0 */
	err = st_lis2dw12_set_wake_up_duration(hw, 0);
	if (err < 0)
		return err;

	/* setting default FF threshold to 312 mg */
	err = st_lis2dw12_set_freefall_threshold(hw, 312);
	if (err < 0)
		return err;

	/* setting default 6D threshold to 60 degrees */
	err = st_lis2dw12_set_6D_threshold(hw, 60);
	if (err < 0)
		return err;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	if (hw->settings->id.hw_id != ST_AIS2DW12_ID) {
		/* setting default tap/dtap threshold to 570 mg */
		err = st_lis2dw12_set_tap_threshold(hw, 570);
		if (err < 0)
			return err;

		/* setting default tap/dtap quiet time to 10 ms */
		err = st_lis2dw12_set_tap_quiet_time(hw, 10);
		if (err < 0)
			return err;

		/* setting default dtap min delay time to 540 ms */
		err = st_lis2dw12_set_dtap_duration(hw, 540);
		if (err < 0)
			return err;

		/* setting default tap/dtap shock time to 40 ms */
		err = st_lis2dw12_set_tap_shock_time(hw, 40);
		if (err < 0)
			return err;

		/* enable tap detection on all axis */
		err = st_lis2dw12_write_with_mask_locked(hw, ST_LIS2DW12_TAP_THS_Z_ADDR,
							ST_LIS2DW12_TAP_EN_MASK, 0x07);
		if (err < 0)
			return err;
	}
#endif /* LINUX_VERSION_CODE */

	/*
	 * TODO:
	 * add embedded function irq when a separate line required by
	 * configuration
	 */
	if (hw->irq_emb > 0) {
		err = devm_request_threaded_irq(hw->dev, hw->irq_emb,
						st_lis2dw12_handler_irq_emb,
						st_lis2dw12_handler_thread_emb,
						IRQF_TRIGGER_HIGH |
						IRQF_ONESHOT,
						"st_lis2dw12", hw);
		if (err) {
			dev_err(hw->dev, "failed to request trigger irq %d\n",
				hw->irq_emb);

			return err;
		}
	}

	return err;
}
