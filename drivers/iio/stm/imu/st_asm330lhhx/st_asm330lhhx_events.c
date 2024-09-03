// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_asm330lhhx xl based events function sensor driver
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

#include "st_asm330lhhx.h"

#define ST_ASM330LHHX_IS_EVENT_ENABLED(_event_id) (!!(hw->enable_ev_mask & \
						      BIT_ULL(_event_id)))

static struct st_asm330lhhx_event_t {
	enum st_asm330lhhx_event_id id;
	char *name;
	u8 irq_mask;
	int req_odr;
	} st_asm330lhhx_events[] = {
	[ST_ASM330LHHX_EVENT_FF] = {
		.id = ST_ASM330LHHX_EVENT_FF,
		.name = "free_fall",
		.irq_mask = ST_ASM330LHHX_INT_FF_MASK,
		.req_odr = ST_ASM330LHHX_MIN_ODR_IN_FREEFALL,
	},
	[ST_ASM330LHHX_EVENT_WAKEUP] = {
		.id = ST_ASM330LHHX_EVENT_WAKEUP,
		.name = "wake_up",
		.irq_mask = ST_ASM330LHHX_INT_WU_MASK,
		.req_odr = ST_ASM330LHHX_MIN_ODR_IN_WAKEUP,
	},
	[ST_ASM330LHHX_EVENT_6D] = {
		.id = ST_ASM330LHHX_EVENT_6D,
		.name = "sixD",
		.irq_mask = ST_ASM330LHHX_INT_6D_MASK,
		.req_odr = ST_ASM330LHHX_MIN_ODR_IN_6D,
	},
};

static const struct st_asm330lhhx_ff_th st_asm330lhhx_free_fall_threshold[] = {
	[0] = { .val = 0x00, .mg = 156 },
	[1] = { .val = 0x01, .mg = 219 },
	[2] = { .val = 0x02, .mg = 250 },
	[3] = { .val = 0x03, .mg = 312 },
	[4] = { .val = 0x04, .mg = 344 },
	[5] = { .val = 0x05, .mg = 406 },
	[6] = { .val = 0x06, .mg = 469 },
	[7] = { .val = 0x07, .mg = 500 },
};

static const struct st_asm330lhhx_6D_th st_asm330lhhx_6D_threshold[] = {
	[0] = { .val = 0x00, .deg = 80 },
	[1] = { .val = 0x01, .deg = 70 },
	[2] = { .val = 0x02, .deg = 60 },
	[3] = { .val = 0x03, .deg = 50 },
};

static inline bool
st_asm330lhhx_events_enabled(struct st_asm330lhhx_hw *hw)
{
	return (ST_ASM330LHHX_IS_EVENT_ENABLED(ST_ASM330LHHX_EVENT_FF) ||
		ST_ASM330LHHX_IS_EVENT_ENABLED(ST_ASM330LHHX_EVENT_WAKEUP) ||
		ST_ASM330LHHX_IS_EVENT_ENABLED(ST_ASM330LHHX_EVENT_6D));
}

static int st_asm330lhhx_get_xl_fs(struct st_asm330lhhx_hw *hw, u8 *xl_fs)
{
	u8 fs_xl_g[] = { 2, 16, 4, 8 }, fs_xl;
	int err;

	err = st_asm330lhhx_read_with_mask(hw,
				    hw->fs_table[ST_ASM330LHHX_ID_ACC].reg.addr,
				    hw->fs_table[ST_ASM330LHHX_ID_ACC].reg.mask,
				    &fs_xl);
	if (err < 0)
		return err;

	*xl_fs = fs_xl_g[fs_xl];

	return err;
}

static int st_asm330lhhx_get_xl_odr(struct st_asm330lhhx_hw *hw, int *xl_odr)
{
	int i, err;
	u8 odr_xl;

	err = st_asm330lhhx_read_with_mask(hw,
		hw->odr_table_entry[ST_ASM330LHHX_ID_ACC].reg.addr,
		hw->odr_table_entry[ST_ASM330LHHX_ID_ACC].reg.mask,
		&odr_xl);
	if (err < 0)
		return err;

	if (odr_xl == 0)
		return 0;

	for (i = 0; i < hw->odr_table_entry[ST_ASM330LHHX_ID_ACC].size; i++) {
		if (odr_xl ==
		     hw->odr_table_entry[ST_ASM330LHHX_ID_ACC].odr_avl[i].val)
			break;
	}

	if (i == hw->odr_table_entry[ST_ASM330LHHX_ID_ACC].size)
		return -EINVAL;

	/* for frequency values with decimal part just return the integer */
	*xl_odr = hw->odr_table_entry[ST_ASM330LHHX_ID_ACC].odr_avl[i].hz;

	return err;
}

static int st_asm330lhhx_get_default_xl_odr(struct st_asm330lhhx_hw *hw,
					    enum st_asm330lhhx_event_id id,
					    int *xl_odr)
{
	int err;
	int odr;
	int req_odr;

	err = st_asm330lhhx_get_xl_odr(hw, &odr);
	if (err < 0)
		return err;

	req_odr = st_asm330lhhx_events[id].req_odr;

	if (odr > req_odr)
		*xl_odr = odr;
	else
		*xl_odr = req_odr;

	return err;
}

/*
 * st_asm330lhhx_set_wake_up_thershold - set wake-up threshold in mg
 * @hw - ST IMU MEMS hw instance
 * @wake_up_thershold_mg - wake-up threshold in mg
 *
 * wake-up thershold register val = (th_mg * 2 ^ 6) / (1000 * FS_XL)
 */
static int st_asm330lhhx_set_wake_up_thershold(struct st_asm330lhhx_hw *hw,
					       int wake_up_thershold_mg)
{
	u8 wake_up_thershold, max_th;
	int tmp, err;
	u8 fs_xl_g;

	err = st_asm330lhhx_get_xl_fs(hw, &fs_xl_g);
	if (err < 0)
		return err;

	tmp = (wake_up_thershold_mg * 64) / (fs_xl_g * 1000);
	wake_up_thershold = (u8)tmp;
	max_th = ST_ASM330LHHX_WAKE_UP_THS_MASK >>
		  __ffs(ST_ASM330LHHX_WAKE_UP_THS_MASK);
	if (wake_up_thershold > max_th)
		wake_up_thershold = max_th;

	err = st_asm330lhhx_write_with_mask_locked(hw,
					   ST_ASM330LHHX_REG_WAKE_UP_THS_ADDR,
					   ST_ASM330LHHX_WAKE_UP_THS_MASK,
					   wake_up_thershold);
	if (err < 0)
		return err;

	hw->wk_th_mg = wake_up_thershold_mg;

	return 0;
}

/*
 * st_asm330lhhx_set_wake_up_duration - set wake-up duration in ms
 * @hw - ST IMU MEMS hw instance
 * @wake_up_duration_ms - wake-up duration in ms
 *
 * wake-up duration register val = (dur_ms * ODR_XL) / (32 * 1000)
 */
static int st_asm330lhhx_set_wake_up_duration(struct st_asm330lhhx_hw *hw,
					      int wake_up_duration_ms)
{
	int tmp, sensor_odr, err;
	u8 wake_up_duration, max_dur;

	err = st_asm330lhhx_get_default_xl_odr(hw, ST_ASM330LHHX_EVENT_WAKEUP,
					       &sensor_odr);
	if (err < 0)
		return err;

	tmp = (wake_up_duration_ms * sensor_odr) / 32000;
	wake_up_duration = (u8)tmp;
	max_dur = ST_ASM330LHHX_WAKE_UP_DUR_MASK >>
		  __ffs(ST_ASM330LHHX_WAKE_UP_DUR_MASK);
	if (wake_up_duration > max_dur)
		wake_up_duration = max_dur;

	err = st_asm330lhhx_write_with_mask_locked(hw,
					    ST_ASM330LHHX_REG_WAKE_UP_DUR_ADDR,
					    ST_ASM330LHHX_WAKE_UP_DUR_MASK,
					    wake_up_duration);
	if (err < 0)
		return err;

	hw->wk_dur_ms = wake_up_duration_ms;

	return 0;
}

/*
 * st_asm330lhhx_set_freefall_threshold - set free fall threshold detection mg
 * @hw - ST IMU MEMS hw instance
 * @freefall_threshold_mg - free fall threshold in mg
 */
static int st_asm330lhhx_set_freefall_threshold(struct st_asm330lhhx_hw *hw,
						int freefall_threshold_mg)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_asm330lhhx_free_fall_threshold); i++) {
		if (freefall_threshold_mg >=
					st_asm330lhhx_free_fall_threshold[i].mg)
			break;
	}

	if (i == ARRAY_SIZE(st_asm330lhhx_free_fall_threshold))
		return -EINVAL;

	err = st_asm330lhhx_write_with_mask_locked(hw,
				      ST_ASM330LHHX_REG_FREE_FALL_ADDR,
				      ST_ASM330LHHX_FF_THS_MASK,
				      st_asm330lhhx_free_fall_threshold[i].val);
	if (err < 0)
		return err;

	hw->freefall_threshold = freefall_threshold_mg;

	return 0;
}

/*
 * st_asm330lhhx_set_6D_threshold - set 6D threshold detection in degrees
 * @hw - ST IMU MEMS hw instance
 * @sixD_threshold_deg - 6D threshold in degrees
 */
static int st_asm330lhhx_set_6D_threshold(struct st_asm330lhhx_hw *hw,
					  int sixD_threshold_deg)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_asm330lhhx_6D_threshold); i++) {
		if (sixD_threshold_deg >= st_asm330lhhx_6D_threshold[i].deg)
			break;
	}

	if (i == ARRAY_SIZE(st_asm330lhhx_6D_threshold))
		return -EINVAL;

	err = st_asm330lhhx_write_with_mask_locked(hw,
					    ST_ASM330LHHX_REG_THS_6D_ADDR,
					    ST_ASM330LHHX_SIXD_THS_MASK,
					    st_asm330lhhx_6D_threshold[i].val);
	if (err < 0)
		return err;

	hw->sixD_threshold = sixD_threshold_deg;

	return 0;
}

int st_asm330lhhx_read_event_config(struct iio_dev *iio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	if (chan->type == IIO_ACCEL) {
		struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
		struct st_asm330lhhx_hw *hw = sensor->hw;

		switch (type) {
		case IIO_EV_TYPE_THRESH:
			switch (dir) {
			case IIO_EV_DIR_FALLING:
				return FIELD_GET(BIT(ST_ASM330LHHX_EVENT_FF),
						 hw->enable_ev_mask);

			case IIO_EV_DIR_RISING:
				return FIELD_GET(BIT(ST_ASM330LHHX_EVENT_WAKEUP),
						 hw->enable_ev_mask);

			default:
				return -EINVAL;
			}
			break;

		case IIO_EV_TYPE_CHANGE:
			switch (dir) {
			case IIO_EV_DIR_EITHER:
				return FIELD_GET(BIT(ST_ASM330LHHX_EVENT_6D),
						 hw->enable_ev_mask);

			default:
				return -EINVAL;
			}
			break;

		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static int
st_asm330lhhx_check_events_odr_dependency(struct st_asm330lhhx_hw *hw,
					  int odr,
					  enum st_asm330lhhx_event_id id,
					  int enable)
{
	int i, ret = 0;
	int current_odr;

	for (i = 0; i < ARRAY_SIZE(st_asm330lhhx_events); i++) {
		current_odr = st_asm330lhhx_events[i].req_odr;

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

int st_asm330lhhx_write_event_config(struct iio_dev *iio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int enable)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
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
				id = ST_ASM330LHHX_EVENT_FF;
				break;

			/*
			 * this is the wk event, use the dir IIO_EV_DIR_RISING
			 * because don't exist a specific iio_event_type related
			 * to wakeup events
			 */
			case IIO_EV_DIR_RISING:
				id = ST_ASM330LHHX_EVENT_WAKEUP;
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
				id = ST_ASM330LHHX_EVENT_6D;
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

	err = st_asm330lhhx_read_locked(hw, int_reg, &int_val, 1);
	if (err < 0)
		return err;

	int_val = st_asm330lhhx_manipulate_bit(int_val,
					      st_asm330lhhx_events[id].irq_mask,
					      enable);
	err = st_asm330lhhx_write_locked(hw, int_reg, int_val);
	if (err < 0)
		return err;

	err = st_asm330lhhx_write_with_mask_locked(hw,
			    ST_ASM330LHHX_REG_INT_CFG1_ADDR,
			    ST_ASM330LHHX_INTERRUPTS_ENABLE_MASK,
			    !!(int_val & ~ST_ASM330LHHX_INT_EMB_FUNC_MASK));
	if (err < 0)
		return err;

	/* check enabled events ODR dependency */
	req_odr = st_asm330lhhx_check_events_odr_dependency(hw,
					       st_asm330lhhx_events[id].req_odr,
					       id, enable);

	err = st_asm330lhhx_set_odr(iio_priv(hw->iio_devs[ST_ASM330LHHX_ID_ACC]),
				    req_odr, 0);
	if (err < 0)
		return err;

	if (enable == 0)
		hw->enable_ev_mask &= ~BIT_ULL(id);
	else
		hw->enable_ev_mask |= BIT_ULL(id);

	return err;
}

int st_asm330lhhx_read_event_value(struct iio_dev *iio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int *val, int *val2)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
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

	default:
		break;
	}

	return err;
}

int st_asm330lhhx_write_event_value(struct iio_dev *iio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
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
				err = st_asm330lhhx_set_freefall_threshold(hw,
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
				err = st_asm330lhhx_set_wake_up_thershold(hw,
									  val);
				break;

			case IIO_EV_INFO_PERIOD:
				err = st_asm330lhhx_set_wake_up_duration(hw,
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
				err = st_asm330lhhx_set_6D_threshold(hw, val);
				break;

			default:
				break;
			}
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}

	return err;
}

int st_asm330lhhx_event_handler(struct st_asm330lhhx_hw *hw)
{
	struct iio_dev *iio_dev;
	u8 reg_src[3];
	s64 event;
	int err;

	if (!st_asm330lhhx_events_enabled(hw))
		return IRQ_HANDLED;

	err = regmap_bulk_read(hw->regmap,
			       ST_ASM330LHHX_REG_WAKE_UP_SRC_ADDR,
			       reg_src, sizeof(reg_src));
	if (err < 0)
		return IRQ_HANDLED;

	if ((reg_src[0] & ST_ASM330LHHX_WAKE_UP_SRC_FF_IA_MASK) &&
	    ST_ASM330LHHX_IS_EVENT_ENABLED(ST_ASM330LHHX_EVENT_FF)) {
		iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_ACC];

		event = IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
					     IIO_EV_TYPE_THRESH,
					     IIO_EV_DIR_FALLING);
		iio_push_event(iio_dev, event,
			       iio_get_time_ns(iio_dev));
	}

	if ((reg_src[0] & ST_ASM330LHHX_WAKE_UP_SRC_WU_IA_MASK) &&
	    ST_ASM330LHHX_IS_EVENT_ENABLED(ST_ASM330LHHX_EVENT_WAKEUP)) {
		iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_ACC];
		if (reg_src[0] & ST_ASM330LHHX_Z_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Z,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}

		if (reg_src[0] & ST_ASM330LHHX_Y_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Y,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}

		if (reg_src[0] & ST_ASM330LHHX_X_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_X,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}
	}

	if ((reg_src[2] & ST_ASM330LHHX_D6D_SRC_D6D_IA_MASK) &&
	    ST_ASM330LHHX_IS_EVENT_ENABLED(ST_ASM330LHHX_EVENT_6D)) {
		u8 dir;

		iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_ACC];
		if (reg_src[2] & (ST_ASM330LHHX_ZH_MASK | ST_ASM330LHHX_ZL_MASK)) {
			dir = (reg_src[2] & ST_ASM330LHHX_ZH_MASK) ?
			      IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Z,
							  IIO_EV_TYPE_CHANGE,
							  dir),
				       iio_get_time_ns(iio_dev));
		}

		if (reg_src[2] & (ST_ASM330LHHX_YH_MASK | ST_ASM330LHHX_YL_MASK)) {
			dir = (reg_src[2] & ST_ASM330LHHX_YH_MASK) ?
			      IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Y,
							  IIO_EV_TYPE_CHANGE,
							  dir),
				       iio_get_time_ns(iio_dev));
		}

		if (reg_src[2] & (ST_ASM330LHHX_XH_MASK | ST_ASM330LHHX_XL_MASK)) {
			dir = (reg_src[2] & ST_ASM330LHHX_XH_MASK) ?
			      IIO_EV_DIR_RISING : IIO_EV_DIR_FALLING;
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_X,
							  IIO_EV_TYPE_CHANGE,
							  dir),
				       iio_get_time_ns(iio_dev));
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t st_asm330lhhx_handler_thread_emb(int irq, void *private)
{
	return st_asm330lhhx_event_handler((struct st_asm330lhhx_hw *)private);
}

int st_asm330lhhx_update_threshold_events(struct st_asm330lhhx_hw *hw)
{
	int ret;

	ret = st_asm330lhhx_set_wake_up_thershold(hw, hw->wk_th_mg);
	if (ret < 0)
		return ret;

	return ret;
}

int st_asm330lhhx_update_duration_events(struct st_asm330lhhx_hw *hw)
{
	int ret;

	ret = st_asm330lhhx_set_wake_up_duration(hw, hw->wk_dur_ms);
	if (ret < 0)
		return ret;

	return ret;
}

int st_asm330lhhx_event_init(struct st_asm330lhhx_hw *hw)
{
	int err;

	/* latch interrupts and clean immediately the source */
	err = regmap_update_bits(hw->regmap, ST_ASM330LHHX_REG_INT_CFG0_ADDR,
			     ST_ASM330LHHX_INT_CLR_ON_READ_MASK,
			     FIELD_PREP(ST_ASM330LHHX_INT_CLR_ON_READ_MASK, 1));
	if (err < 0)
		return err;

	/* apply HPF on wake-up */
	err = regmap_update_bits(hw->regmap, ST_ASM330LHHX_REG_INT_CFG0_ADDR,
				 ST_ASM330LHHX_SLOPE_FDS_MASK,
				 FIELD_PREP(ST_ASM330LHHX_SLOPE_FDS_MASK, 1));
	if (err < 0)
		return err;

	/* Set default wake-up thershold to 100 mg */
	err = st_asm330lhhx_set_wake_up_thershold(hw, 100);
	if (err < 0)
		return err;

	/* Set default wake-up duration to 0 */
	err = st_asm330lhhx_set_wake_up_duration(hw, 0);
	if (err < 0)
		return err;

	/* setting default FF threshold to 312 mg */
	err = st_asm330lhhx_set_freefall_threshold(hw, 312);
	if (err < 0)
		return err;

	/* setting default 6D threshold to 60 degrees */
	err = st_asm330lhhx_set_6D_threshold(hw, 60);
	if (err < 0)
		return err;

	/* check if embedded function routed to a dedicated irq line */
	if (hw->irq_emb != hw->irq) {
		unsigned long irq_type;

		irq_type = irqd_get_trigger_type(irq_get_irq_data(hw->irq_emb));
		if (irq_type == IRQF_TRIGGER_NONE)
			irq_type = IRQF_TRIGGER_HIGH;

		err = devm_request_threaded_irq(hw->dev, hw->irq_emb, NULL,
					       st_asm330lhhx_handler_thread_emb,
					       irq_type | IRQF_ONESHOT,
					       "asm330lhhx_emb", hw);
		if (err) {
			dev_err(hw->dev, "failed to request trigger irq %d\n",
				hw->irq_emb);

			return err;
		}
	}

	return err;
}

