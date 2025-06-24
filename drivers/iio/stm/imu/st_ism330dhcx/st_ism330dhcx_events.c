// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_ism330dhcx xl based events function sensor driver
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

#include "st_ism330dhcx.h"

#define ST_ISM330DHCX_IS_EVENT_ENABLED(_event_id) (!!(hw->enable_ev_mask & \
						      BIT_ULL(_event_id)))

/* this is the minimal ODR for event sensors and dependencies */
#define ST_ISM330DHCX_MIN_ODR_IN_WAKEUP			26
#define ST_ISM330DHCX_MIN_ODR_IN_FREEFALL		26
#define ST_ISM330DHCX_MIN_ODR_IN_6D			26
#define ST_ISM330DHCX_MIN_ODR_IN_TAP			416
#define ST_ISM330DHCX_MIN_ODR_IN_DTAP			416

static const struct st_ism330dhcx_event_t {
	enum st_ism330dhcx_event_id id;
	char *name;
	u8 irq_mask;
	int req_odr;
	} st_ism330dhcx_events[] = {
	[ST_ISM330DHCX_EVENT_FF] = {
		.id = ST_ISM330DHCX_EVENT_FF,
		.name = "free_fall",
		.irq_mask = ST_ISM330DHCX_INT_FF_MASK,
		.req_odr = ST_ISM330DHCX_MIN_ODR_IN_FREEFALL,
	},
	[ST_ISM330DHCX_EVENT_WAKEUP] = {
		.id = ST_ISM330DHCX_EVENT_WAKEUP,
		.name = "wake_up",
		.irq_mask = ST_ISM330DHCX_INT_WU_MASK,
		.req_odr = ST_ISM330DHCX_MIN_ODR_IN_WAKEUP,
	},
	[ST_ISM330DHCX_EVENT_6D] = {
		.id = ST_ISM330DHCX_EVENT_6D,
		.name = "sixD",
		.irq_mask = ST_ISM330DHCX_INT_6D_MASK,
		.req_odr = ST_ISM330DHCX_MIN_ODR_IN_6D,
	},

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	[ST_ISM330DHCX_EVENT_TAP] = {
		.id = ST_ISM330DHCX_EVENT_TAP,
		.name = "tap",
		.irq_mask = ST_ISM330DHCX_INT_SINGLE_TAP_MASK,
		.req_odr = ST_ISM330DHCX_MIN_ODR_IN_TAP,
	},
	[ST_ISM330DHCX_EVENT_DTAP] = {
		.id = ST_ISM330DHCX_EVENT_DTAP,
		.name = "dtap",
		.irq_mask = ST_ISM330DHCX_INT_DOUBLE_TAP_MASK,
		.req_odr = ST_ISM330DHCX_MIN_ODR_IN_DTAP,
	},
#endif /* LINUX_VERSION_CODE */

};

static const struct st_ism330dhcx_ff_th st_ism330dhcx_free_fall_threshold[] = {
	[0] = { .val = 0x00, .mg = 156 },
	[1] = { .val = 0x01, .mg = 219 },
	[2] = { .val = 0x02, .mg = 250 },
	[3] = { .val = 0x03, .mg = 312 },
	[4] = { .val = 0x04, .mg = 344 },
	[5] = { .val = 0x05, .mg = 406 },
	[6] = { .val = 0x06, .mg = 469 },
	[7] = { .val = 0x07, .mg = 500 },
};

static const struct st_ism330dhcx_6D_th st_ism330dhcx_6D_threshold[] = {
	[0] = { .val = 0x00, .deg = 80 },
	[1] = { .val = 0x01, .deg = 70 },
	[2] = { .val = 0x02, .deg = 60 },
	[3] = { .val = 0x03, .deg = 50 },
};

static inline bool
st_ism330dhcx_events_enabled(struct st_ism330dhcx_hw *hw)
{
	return (ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_FF) ||
		ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_WAKEUP) ||

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_TAP) ||
		ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_DTAP) ||
#endif /* LINUX_VERSION_CODE */

		ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_6D));
}

static int st_ism330dhcx_get_xl_fs(struct st_ism330dhcx_hw *hw, u8 *xl_fs)
{
	u8 fs_xl_g[] = { 2, 16, 4, 8 }, fs_xl;
	int err;

	err = st_ism330dhcx_read_with_mask(hw,
				    hw->fs_table[ST_ISM330DHCX_ID_ACC].reg.addr,
				    hw->fs_table[ST_ISM330DHCX_ID_ACC].reg.mask,
				    &fs_xl);
	if (err < 0)
		return err;

	*xl_fs = fs_xl_g[fs_xl];

	return err;
}

static int st_ism330dhcx_get_xl_odr(struct st_ism330dhcx_hw *hw, int *xl_odr)
{
	int i, err;
	u8 odr_xl;

	err = st_ism330dhcx_read_with_mask(hw,
				   hw->odr_table[ST_ISM330DHCX_ID_ACC].reg.addr,
				   hw->odr_table[ST_ISM330DHCX_ID_ACC].reg.mask,
				   &odr_xl);
	if (err < 0)
		return err;

	if (odr_xl == 0)
		return 0;

	for (i = 0; i < hw->odr_table[ST_ISM330DHCX_ID_ACC].size; i++) {
		if (odr_xl ==
		     hw->odr_table[ST_ISM330DHCX_ID_ACC].odr_avl[i].val)
			break;
	}

	if (i == hw->odr_table[ST_ISM330DHCX_ID_ACC].size)
		return -EINVAL;

	/* for frequency values with decimal part just return the integer */
	*xl_odr = hw->odr_table[ST_ISM330DHCX_ID_ACC].odr_avl[i].hz;

	return err;
}

static int st_ism330dhcx_get_default_xl_odr(struct st_ism330dhcx_hw *hw,
					    enum st_ism330dhcx_event_id id,
					    int *xl_odr)
{
	int err;
	int odr;
	int req_odr;

	err = st_ism330dhcx_get_xl_odr(hw, &odr);
	if (err < 0)
		return err;

	req_odr = st_ism330dhcx_events[id].req_odr;

	if (odr > req_odr)
		*xl_odr = odr;
	else
		*xl_odr = req_odr;

	return err;
}

/*
 * st_ism330dhcx_set_wake_up_threshold - set wake-up threshold in mg
 * @hw - ST IMU MEMS hw instance
 * @wake_up_threshold_mg - wake-up threshold in mg
 *
 * wake-up threshold register val = (th_mg * 2 ^ 6) / (1000 * FS_XL)
 */
static int st_ism330dhcx_set_wake_up_threshold(struct st_ism330dhcx_hw *hw,
					       int wake_up_threshold_mg)
{
	u8 wake_up_threshold, max_th;
	int tmp, err;
	u8 fs_xl_g;

	err = st_ism330dhcx_get_xl_fs(hw, &fs_xl_g);
	if (err < 0)
		return err;

	tmp = (wake_up_threshold_mg * 64) / (fs_xl_g * 1000);
	wake_up_threshold = (u8)tmp;
	max_th = ST_ISM330DHCX_WAKE_UP_THS_MASK >>
		  __ffs(ST_ISM330DHCX_WAKE_UP_THS_MASK);
	if (wake_up_threshold > max_th)
		wake_up_threshold = max_th;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_WAKE_UP_THS_ADDR,
					    ST_ISM330DHCX_WAKE_UP_THS_MASK,
					    wake_up_threshold);
	if (err < 0)
		return err;

	hw->wk_th_mg = wake_up_threshold_mg;

	return 0;
}

/*
 * st_ism330dhcx_set_wake_up_duration - set wake-up duration in ms
 * @hw - ST IMU MEMS hw instance
 * @wake_up_duration_ms - wake-up duration in ms
 *
 * wake-up duration register val = (dur_ms * ODR_XL) / (32 * 1000)
 */
static int st_ism330dhcx_set_wake_up_duration(struct st_ism330dhcx_hw *hw,
					      int wake_up_duration_ms)
{
	int tmp, sensor_odr, err;
	u8 wake_up_duration, max_dur;

	err = st_ism330dhcx_get_default_xl_odr(hw, ST_ISM330DHCX_EVENT_WAKEUP,
					       &sensor_odr);
	if (err < 0)
		return err;

	tmp = (wake_up_duration_ms * sensor_odr) / 32000;
	wake_up_duration = (u8)tmp;
	max_dur = ST_ISM330DHCX_WAKE_UP_DUR_MASK >>
		  __ffs(ST_ISM330DHCX_WAKE_UP_DUR_MASK);
	if (wake_up_duration > max_dur)
		wake_up_duration = max_dur;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_WAKE_UP_DUR_ADDR,
					    ST_ISM330DHCX_WAKE_UP_DUR_MASK,
					    wake_up_duration);
	if (err < 0)
		return err;

	hw->wk_dur_ms = wake_up_duration_ms;

	return 0;
}

/*
 * st_ism330dhcx_set_freefall_threshold - set free fall threshold detection mg
 * @hw - ST IMU MEMS hw instance
 * @freefall_threshold_mg - free fall threshold in mg
 */
static int st_ism330dhcx_set_freefall_threshold(struct st_ism330dhcx_hw *hw,
						int freefall_threshold_mg)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_ism330dhcx_free_fall_threshold); i++) {
		if (freefall_threshold_mg >=
					st_ism330dhcx_free_fall_threshold[i].mg)
			break;
	}

	if (i == ARRAY_SIZE(st_ism330dhcx_free_fall_threshold))
		return -EINVAL;

	err = st_ism330dhcx_write_with_mask(hw,
				      ST_ISM330DHCX_REG_FREE_FALL_ADDR,
				      ST_ISM330DHCX_FF_THS_MASK,
				      st_ism330dhcx_free_fall_threshold[i].val);
	if (err < 0)
		return err;

	hw->freefall_threshold = freefall_threshold_mg;

	return 0;
}

/*
 * st_ism330dhcx_set_6D_threshold - set 6D threshold detection in degrees
 * @hw - ST IMU MEMS hw instance
 * @sixD_threshold_deg - 6D threshold in degrees
 */
static int st_ism330dhcx_set_6D_threshold(struct st_ism330dhcx_hw *hw,
					  int sixD_threshold_deg)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_ism330dhcx_6D_threshold); i++) {
		if (sixD_threshold_deg >= st_ism330dhcx_6D_threshold[i].deg)
			break;
	}

	if (i == ARRAY_SIZE(st_ism330dhcx_6D_threshold))
		return -EINVAL;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_TAP_THS_6D_ADDR,
					    ST_ISM330DHCX_SIXD_THS_MASK,
					    st_ism330dhcx_6D_threshold[i].val);
	if (err < 0)
		return err;

	hw->sixD_threshold = sixD_threshold_deg;

	return 0;
}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
/*
 * st_ism330dhcx_set_tap_threshold - set TAP/DTAP threshold detection in mg
 * @hw - ST IMU MEMS hw instance
 * @tap_threshold_mg - TAP/DTAP threshold in mg
 *
 * th_ug threshold register val = (th_ug * 2 ^ 5) / (1000 * FS_XL)
 */
static int st_ism330dhcx_set_tap_threshold(struct st_ism330dhcx_hw *hw,
					   int tap_threshold_mg)
{
	u8 tap_threshold, max_th;
	int tmp, err;
	u8 fs_xl_g;

	err = st_ism330dhcx_get_xl_fs(hw, &fs_xl_g);
	if (err < 0)
		return err;

	tmp = (tap_threshold_mg * 32) / (fs_xl_g * 1000);
	tap_threshold = (u8)tmp;
	max_th = ST_ISM330DHCX_TAP_THS_X_MASK >>
		  __ffs(ST_ISM330DHCX_TAP_THS_X_MASK);
	if (tap_threshold > max_th)
		tap_threshold = max_th;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_TAP_CFG1_ADDR,
					    ST_ISM330DHCX_TAP_THS_X_MASK,
					    tap_threshold);
	if (err < 0)
		return err;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_TAP_CFG2_ADDR,
					    ST_ISM330DHCX_TAP_THS_Y_MASK,
					    tap_threshold);
	if (err < 0)
		return err;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_TAP_THS_6D_ADDR,
					    ST_ISM330DHCX_TAP_THS_Z_MASK,
					    tap_threshold);
	if (err < 0)
		return err;

	hw->tap_threshold = tap_threshold_mg;

	return 0;
}

/*
 * st_ism330dhcx_set_tap_quiet_time - set TAP/DTAP quiet time in ms
 * @hw - ST IMU MEMS hw instance
 * @tap_quiet_time_ms - TAP/DTAP quiet time in LSB
 */
static int st_ism330dhcx_set_tap_quiet_time(struct st_ism330dhcx_hw *hw,
					    u8 tap_quiet_time_ms)
{
	u8 tap_quiet_time, max_quite;
	int sensor_odr, err, tmp;

	err = st_ism330dhcx_get_default_xl_odr(hw, ST_ISM330DHCX_EVENT_TAP,
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
	max_quite = ST_ISM330DHCX_QUIET_MASK >> __ffs(ST_ISM330DHCX_QUIET_MASK);
	if (tap_quiet_time > max_quite)
		tap_quiet_time = max_quite;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_INT_DUR2_ADDR,
					    ST_ISM330DHCX_QUIET_MASK,
					    tap_quiet_time);
	if (err < 0)
		return err;

	hw->tap_quiet_time = tap_quiet_time_ms;

	return 0;
}

/*
 * st_ism330dhcx_set_tap_shock_time - set TAP/DTAP shock time in ms
 * @hw - ST IMU MEMS hw instance
 * @tap_shock_time_ms - TAP/DTAP shock time in ms
 */
static int st_ism330dhcx_set_tap_shock_time(struct st_ism330dhcx_hw *hw,
					    u8 tap_shock_time_ms)
{
	u8 tap_shock_time, max_tap_shock_time;
	int sensor_odr, err, tmp;

	err = st_ism330dhcx_get_default_xl_odr(hw, ST_ISM330DHCX_EVENT_TAP,
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
	max_tap_shock_time = ST_ISM330DHCX_QUIET_MASK >>
			    __ffs(ST_ISM330DHCX_QUIET_MASK);
	if (tap_shock_time > max_tap_shock_time)
		tap_shock_time = max_tap_shock_time;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_INT_DUR2_ADDR,
					    ST_ISM330DHCX_SHOCK_MASK,
					    tap_shock_time);
	if (err < 0)
		return err;

	hw->tap_shock_time = tap_shock_time_ms;

	return 0;
}

/*
 * st_ism330dhcx_set_dtap_duration - set DTAP durationin ms (min time)
 * @hw - ST IMU MEMS hw instance
 * @dtap_duration_ms - DTAP min delay time in ms
 */
static int st_ism330dhcx_set_dtap_duration(struct st_ism330dhcx_hw *hw,
					  int dtap_duration_ms)
{
	int sensor_odr, tmp, err;
	u8 dtap_duration, max_dtap_duration;

	err = st_ism330dhcx_get_default_xl_odr(hw, ST_ISM330DHCX_EVENT_TAP,
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
	max_dtap_duration = ST_ISM330DHCX_DUR_MASK >>
			    __ffs(ST_ISM330DHCX_DUR_MASK);

	if (dtap_duration > max_dtap_duration)
		dtap_duration = max_dtap_duration;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_INT_DUR2_ADDR,
					    ST_ISM330DHCX_DUR_MASK,
					    dtap_duration);
	if (err < 0)
		return err;

	hw->dtap_duration = dtap_duration_ms;

	return 0;
}

#endif /* LINUX_VERSION_CODE */

int st_ism330dhcx_read_event_config(struct iio_dev *iio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	if (chan->type == IIO_ACCEL) {
		struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
		struct st_ism330dhcx_hw *hw = sensor->hw;

		switch (type) {
		case IIO_EV_TYPE_THRESH:
			switch (dir) {
			case IIO_EV_DIR_FALLING:
				return FIELD_GET(BIT(ST_ISM330DHCX_EVENT_FF),
						 hw->enable_ev_mask);

			case IIO_EV_DIR_RISING:
				return FIELD_GET(BIT(ST_ISM330DHCX_EVENT_WAKEUP),
						 hw->enable_ev_mask);

			default:
				return -EINVAL;
			}
			break;

		case IIO_EV_TYPE_CHANGE:
			switch (dir) {
			case IIO_EV_DIR_EITHER:
				return FIELD_GET(BIT(ST_ISM330DHCX_EVENT_6D),
						 hw->enable_ev_mask);

			default:
				return -EINVAL;
			}
			break;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		case IIO_EV_TYPE_GESTURE:
			switch (dir) {
			case IIO_EV_DIR_SINGLETAP:
				return FIELD_GET(BIT(ST_ISM330DHCX_EVENT_TAP),
						 hw->enable_ev_mask);

			case IIO_EV_DIR_DOUBLETAP:
				return FIELD_GET(BIT(ST_ISM330DHCX_EVENT_DTAP),
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
st_ism330dhcx_check_events_odr_dependency(struct st_ism330dhcx_hw *hw,
					  int odr,
					  enum st_ism330dhcx_event_id id,
					  int enable)
{
	int i, ret = 0;
	int current_odr;

	for (i = 0; i < ARRAY_SIZE(st_ism330dhcx_events); i++) {
		current_odr = st_ism330dhcx_events[i].req_odr;

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

int st_ism330dhcx_write_event_config(struct iio_dev *iio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int enable)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	struct st_ism330dhcx_hw *hw = sensor->hw;
	u8 int_reg = hw->embfunc_pg0_irq_reg;
	u8 int_val;
	int id = -1;
	int req_odr = 0;
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
				id = ST_ISM330DHCX_EVENT_FF;
				break;

			/*
			 * this is the wk event, use the dir IIO_EV_DIR_RISING
			 * because don't exist a specific iio_event_type related
			 * to wakeup events
			 */
			case IIO_EV_DIR_RISING:
				id = ST_ISM330DHCX_EVENT_WAKEUP;
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
				id = ST_ISM330DHCX_EVENT_6D;
				break;

			default:
				return -EINVAL;
			}
			break;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
		case IIO_EV_TYPE_GESTURE:
			switch (dir) {
			case IIO_EV_DIR_SINGLETAP:
				id = ST_ISM330DHCX_EVENT_TAP;
				break;

			case IIO_EV_DIR_DOUBLETAP:
				err = st_ism330dhcx_write_with_mask(hw,
					   ST_ISM330DHCX_REG_WAKE_UP_THS_ADDR,
					   ST_ISM330DHCX_SINGLE_DOUBLE_TAP_MASK,
					   enable);
				if (err < 0)
					return err;

				id = ST_ISM330DHCX_EVENT_DTAP;
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

	err = st_ism330dhcx_read_atomic(hw, int_reg, 1, &int_val);
	if (err < 0)
		return err;

	int_val = st_ism330dhcx_manipulate_bit(int_val,
					      st_ism330dhcx_events[id].irq_mask,
					      enable);

	err = st_ism330dhcx_write_atomic(hw, int_reg, 1,
					 (unsigned int *)&int_val);
	if (err < 0)
		return err;

	err = st_ism330dhcx_write_with_mask(hw,
					    ST_ISM330DHCX_REG_TAP_CFG2_ADDR,
					    ST_ISM330DHCX_INTERRUPTS_ENABLE_MASK,
					    !!(int_val & ~ST_ISM330DHCX_REG_INT_EMB_FUNC_MASK));
	if (err < 0)
		return err;

	/* check enabled events ODR dependency */
	req_odr = st_ism330dhcx_check_events_odr_dependency(hw,
					       st_ism330dhcx_events[id].req_odr,
					       id, enable);

	err = st_ism330dhcx_set_odr(iio_priv(hw->iio_devs[ST_ISM330DHCX_ID_ACC]),
				    req_odr, 0);
	if (err < 0)
		return err;

	if (enable == 0)
		hw->enable_ev_mask &= ~BIT_ULL(id);
	else
		hw->enable_ev_mask |= BIT_ULL(id);

	return err;
}

int st_ism330dhcx_read_event_value(struct iio_dev *iio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int *val, int *val2)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	struct st_ism330dhcx_hw *hw = sensor->hw;
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

int st_ism330dhcx_write_event_value(struct iio_dev *iio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	struct st_ism330dhcx_hw *hw = sensor->hw;
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
				err = st_ism330dhcx_set_freefall_threshold(hw,
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
				err = st_ism330dhcx_set_wake_up_threshold(hw,
									  val);
				break;

			case IIO_EV_INFO_PERIOD:
				err = st_ism330dhcx_set_wake_up_duration(hw,
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
				err = st_ism330dhcx_set_6D_threshold(hw, val);
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
				err = st_ism330dhcx_set_tap_threshold(hw, val);
				break;

			case IIO_EV_INFO_RESET_TIMEOUT:
				err = st_ism330dhcx_set_tap_quiet_time(hw, val);
				break;

			default:
				break;
			}
			break;

		case IIO_EV_DIR_DOUBLETAP:
			switch (info) {
			case IIO_EV_INFO_VALUE:
				err = st_ism330dhcx_set_tap_threshold(hw, val);
				break;

			case IIO_EV_INFO_RESET_TIMEOUT:
				err = st_ism330dhcx_set_tap_quiet_time(hw, val);
				break;

			case IIO_EV_INFO_TAP2_MIN_DELAY:
				err = st_ism330dhcx_set_dtap_duration(hw, val);
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

int st_ism330dhcx_event_handler(struct st_ism330dhcx_hw *hw)
{
	struct iio_dev *iio_dev;
	u8 reg_src[3];
	s64 event;
	int err;

	if (!st_ism330dhcx_events_enabled(hw))
		return IRQ_HANDLED;

	err = regmap_bulk_read(hw->regmap,
			       ST_ISM330DHCX_REG_WAKE_UP_SRC_ADDR,
			       reg_src, sizeof(reg_src));
	if (err < 0)
		return IRQ_HANDLED;

	if ((reg_src[0] & ST_ISM330DHCX_WAKE_UP_SRC_FF_IA_MASK) &&
	    ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_FF)) {
		iio_dev = hw->iio_devs[ST_ISM330DHCX_ID_ACC];

		event = IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
					     IIO_EV_TYPE_THRESH,
					     IIO_EV_DIR_FALLING);
		iio_push_event(iio_dev, event,
			       iio_get_time_ns(iio_dev));
	}

	if ((reg_src[0] & ST_ISM330DHCX_WAKE_UP_SRC_WU_IA_MASK) &&
	    ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_WAKEUP)) {
		iio_dev = hw->iio_devs[ST_ISM330DHCX_ID_ACC];
		if (reg_src[0] & ST_ISM330DHCX_Z_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Z,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}

		if (reg_src[0] & ST_ISM330DHCX_Y_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Y,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}

		if (reg_src[0] & ST_ISM330DHCX_X_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_X,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}
	}

	if ((reg_src[2] & ST_ISM330DHCX_D6D_SRC_D6D_IA_MASK) &&
	    ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_6D)) {
		u8 dir;

		iio_dev = hw->iio_devs[ST_ISM330DHCX_ID_ACC];
		if (reg_src[2] & (ST_ISM330DHCX_ZH_MASK | ST_ISM330DHCX_ZL_MASK)) {
			dir = (reg_src[2] & ST_ISM330DHCX_ZH_MASK) ?
			      IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Z,
							  IIO_EV_TYPE_CHANGE,
							  dir),
				       iio_get_time_ns(iio_dev));
		}

		if (reg_src[2] & (ST_ISM330DHCX_YH_MASK | ST_ISM330DHCX_YL_MASK)) {
			dir = (reg_src[2] & ST_ISM330DHCX_YH_MASK) ?
			      IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Y,
							  IIO_EV_TYPE_CHANGE,
							  dir),
				       iio_get_time_ns(iio_dev));
		}

		if (reg_src[2] & (ST_ISM330DHCX_XH_MASK | ST_ISM330DHCX_XL_MASK)) {
			dir = (reg_src[2] & ST_ISM330DHCX_XH_MASK) ?
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
	if ((reg_src[1] & ST_ISM330DHCX_SINGLE_TAP_IA_MASK) &&
	    ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_TAP)) {
		u8 mod;

		iio_dev = hw->iio_devs[ST_ISM330DHCX_ID_ACC];
		if (reg_src[1] & ST_ISM330DHCX_X_TAP_MASK)
			mod = IIO_MOD_X;
		else if (reg_src[1] & ST_ISM330DHCX_Y_TAP_MASK)
			mod = IIO_MOD_Y;
		else if (reg_src[1] & ST_ISM330DHCX_Z_TAP_MASK)
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

	if ((reg_src[1] & ST_ISM330DHCX_DOUBLE_TAP_IA_MASK) &&
	    ST_ISM330DHCX_IS_EVENT_ENABLED(ST_ISM330DHCX_EVENT_DTAP)) {
		u8 mod;

		iio_dev = hw->iio_devs[ST_ISM330DHCX_ID_ACC];
		if (reg_src[1] & ST_ISM330DHCX_X_TAP_MASK)
			mod = IIO_MOD_X;
		else if (reg_src[1] & ST_ISM330DHCX_Y_TAP_MASK)
			mod = IIO_MOD_Y;
		else if (reg_src[1] & ST_ISM330DHCX_Z_TAP_MASK)
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

int st_ism330dhcx_update_threshold_events(struct st_ism330dhcx_hw *hw)
{
	int ret;

	ret = st_ism330dhcx_set_wake_up_threshold(hw, hw->wk_th_mg);
	if (ret < 0)
		return ret;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ret = st_ism330dhcx_set_tap_threshold(hw, hw->tap_threshold);
	if (ret < 0)
		return ret;
#endif /* LINUX_VERSION_CODE */

	return ret;
}

int st_ism330dhcx_update_duration_events(struct st_ism330dhcx_hw *hw)
{
	int ret;

	ret = st_ism330dhcx_set_wake_up_duration(hw, hw->wk_dur_ms);
	if (ret < 0)
		return ret;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ret = st_ism330dhcx_set_tap_quiet_time(hw, hw->tap_quiet_time);
	if (ret < 0)
		return ret;

	ret = st_ism330dhcx_set_dtap_duration(hw, hw->dtap_duration);
	if (ret < 0)
		return ret;

	ret = st_ism330dhcx_set_tap_shock_time(hw, hw->tap_shock_time);
	if (ret < 0)
		return ret;
#endif /* LINUX_VERSION_CODE */

	return ret;
}

/*
 * Configure the xl based events function default threshold and duration/delay
 *
 * wake_up_threshold = 100 mg
 * wake_up_duration = 0 ms
 * freefall_threshold = 312 mg
 * 6D_threshold = 60 deg
 * tap_threshold = 570 mg
 * tap_quiet_time = 10 ms
 * dtap_duration = 540 ms
 * tap_shock_time = 40 ms (this parameter is not possible to configure through
 * 			   IIO attribute)
 */
int st_ism330dhcx_event_init(struct st_ism330dhcx_hw *hw)
{
	int err;

	/* latch interrupts and clean immediately the source */
	err = regmap_update_bits(hw->regmap, ST_ISM330DHCX_REG_TAP_CFG0_ADDR,
				 ST_ISM330DHCX_REG_INT_CLR_ON_READ_MASK,
				 FIELD_PREP(ST_ISM330DHCX_REG_INT_CLR_ON_READ_MASK,
					    1));
	if (err < 0)
		return err;

	err = regmap_update_bits(hw->regmap, ST_ISM330DHCX_REG_TAP_CFG0_ADDR,
				 ST_ISM330DHCX_TAP_SLOPE_FDS_MASK,
				 FIELD_PREP(ST_ISM330DHCX_TAP_SLOPE_FDS_MASK,
					    1));
	if (err < 0)
		return err;

	/* Set default wake-up threshold to 100 mg */
	err = st_ism330dhcx_set_wake_up_threshold(hw, 100);
	if (err < 0)
		return err;

	/* Set default wake-up duration to 0 */
	err = st_ism330dhcx_set_wake_up_duration(hw, 0);
	if (err < 0)
		return err;

	/* setting default FF threshold to 312 mg */
	err = st_ism330dhcx_set_freefall_threshold(hw, 312);
	if (err < 0)
		return err;

	/* setting default 6D threshold to 60 degrees */
	err = st_ism330dhcx_set_6D_threshold(hw, 60);
	if (err < 0)
		return err;

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	/* setting default tap/dtap threshold to 570 mg */
	err = st_ism330dhcx_set_tap_threshold(hw, 570);
	if (err < 0)
		return err;

	/* setting default tap/dtap quiet time to 10 ms */
	err = st_ism330dhcx_set_tap_quiet_time(hw, 10);
	if (err < 0)
		return err;

	/* setting default dtap min delay time to 540 ms */
	err = st_ism330dhcx_set_dtap_duration(hw, 540);
	if (err < 0)
		return err;

	/* setting default tap/dtap shock time to 40 ms */
	err = st_ism330dhcx_set_tap_shock_time(hw, 40);
	if (err < 0)
		return err;

	/* enable tap detection on all axis */
	err = st_ism330dhcx_write_with_mask(hw, ST_ISM330DHCX_REG_TAP_CFG0_ADDR,
					    ST_ISM330DHCX_TAP_EN_MASK, 0x07);
	if (err < 0)
		return err;
#endif /* LINUX_VERSION_CODE */

	return err;
}
