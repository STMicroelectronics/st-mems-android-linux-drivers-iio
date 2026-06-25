// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lsm6dsvxhg high-g xl based events function
 * sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2026 STMicroelectronics Inc.
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

#include "st_lsm6dsvxhg.h"

#define ST_LSM6DSVXHG_HG_WAKE_UP_SRC_ADDR	0x4c
#define ST_LSM6DSVXHG_HG_SHOCK_CHANGE_IA_MASK	BIT(6)
#define ST_LSM6DSVXHG_HG_SHOCK_STATE_MASK	BIT(5)
#define ST_LSM6DSVXHG_HG_WU_CHANGE_IA_MASK	BIT(4)
#define ST_LSM6DSVXHG_HG_WU_IA_MASK		BIT(3)
#define ST_LSM6DSVXHG_HG_X_WU_MASK		BIT(2)
#define ST_LSM6DSVXHG_HG_Y_WU_MASK		BIT(1)
#define ST_LSM6DSVXHG_HG_Z_WU_MASK		BIT(0)

#define ST_LSM6DSVXHG_HG_FUNCTIONS_ENABLE_ADDR	0x52
#define ST_LSM6DSVXHG_HG_INTERRUPTS_ENABLE_MASK	BIT(7)
#define ST_LSM6DSVXHG_HG_WU_CHANGE_INT_SEL_MASK	BIT(6)
#define ST_LSM6DSVXHG_INT2_HG_WU_MASK		BIT(5)
#define ST_LSM6DSVXHG_INT1_HG_WU_MASK		BIT(4)
#define ST_LSM6DSVXHG_HG_SHOCK_DUR_MASK		GENMASK(3, 0)

#define ST_LSM6DSVXHG_HG_WAKE_UP_THS_ADDR	0x53

#define ST_LSM6DSVXHG_INACTIVITY_THS_ADDR	0x55
#define ST_LSM6DSVXHG_INT2_HG_SHOCK_CHANGE_MASK	BIT(7)
#define ST_LSM6DSVXHG_INT1_HG_SHOCK_CHANGE_MASK	BIT(6)

/* this is the minimal ODR for event sensors and dependencies */
#define ST_LSM6DSVXHG_MIN_ODR_IN_HG_WAKEUP	480
#define ST_LSM6DSVXHG_MIN_ODR_IN_HG_SHOCK	480

#define ST_LSM6DSVXHG_IS_HG_EVENT_ENABLED(_event_id) (!!(hw->enable_ev_mask & \
						      BIT_ULL(_event_id)))

static struct st_lsm6dsvxhg_hgevent_t {
	enum st_lsm6dsvxhg_event_id id;
	char *name;
	u8 irq_mask;
	u8 irq_reg;
	int req_odr;
	} st_lsm6dsvxhg_hgevents[] = {
	[0] = {
		.id = ST_SM6DSVXHG_EVENT_HG_WAKEUP,
		.name = "hg_wake_up",
		.irq_mask = ST_LSM6DSVXHG_INT1_HG_WU_MASK |
			    ST_LSM6DSVXHG_INT2_HG_WU_MASK,
		.irq_reg = ST_LSM6DSVXHG_HG_FUNCTIONS_ENABLE_ADDR,
		.req_odr = ST_LSM6DSVXHG_MIN_ODR_IN_HG_WAKEUP,
	},
	[1] = {
		.id = ST_SM6DSVXHG_EVENT_HG_SHOCK,
		.name = "hg_shock",
		.irq_mask = ST_LSM6DSVXHG_INT1_HG_SHOCK_CHANGE_MASK |
			    ST_LSM6DSVXHG_INT2_HG_SHOCK_CHANGE_MASK,
		.irq_reg = ST_LSM6DSVXHG_INACTIVITY_THS_ADDR,
		.req_odr = ST_LSM6DSVXHG_MIN_ODR_IN_HG_SHOCK,
	},
};

static int st_lsm6dsvxhg_hgevents_get_index(enum st_lsm6dsvxhg_event_id id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_lsm6dsvxhg_hgevents); i++) {
		if (st_lsm6dsvxhg_hgevents[i].id == id)
			break;
	}

	if (i == ARRAY_SIZE(st_lsm6dsvxhg_hgevents))
		return -EINVAL;

	return i;
}

static inline bool
st_lsm6dsvxhg_hgevents_enabled(struct st_lsm6dsvxhg_hw *hw)
{
	return (ST_LSM6DSVXHG_IS_HG_EVENT_ENABLED(ST_SM6DSVXHG_EVENT_HG_WAKEUP) ||
		ST_LSM6DSVXHG_IS_HG_EVENT_ENABLED(ST_SM6DSVXHG_EVENT_HG_SHOCK));
}

static int st_lsm6dsvxhg_get_hg_xl_fs(struct st_lsm6dsvxhg_hw *hw, u32 *xl_fs)
{
	u8 fs_xl;
	int err;

	err = st_lsm6dsvxhg_read_with_mask(hw,
			  hw->settings->fs_table[ST_LSM6DSVXHG_ID_HG_ACC].reg.addr,
			  hw->settings->fs_table[ST_LSM6DSVXHG_ID_HG_ACC].reg.mask,
			  &fs_xl);
	if (err < 0)
		return err;


	if (fs_xl >= hw->settings->fs_table[ST_LSM6DSVXHG_ID_HG_ACC].size)
		return -EINVAL;

	*xl_fs = hw->settings->fs_table[ST_LSM6DSVXHG_ID_HG_ACC].fs_avl[fs_xl].fs;

	return 0;
}

static int st_lsm6dsvxhg_get_hg_xl_odr(struct st_lsm6dsvxhg_hw *hw,
				       int *xl_odr)
{
	int i, err;
	u8 odr_xl;

	err = st_lsm6dsvxhg_read_with_mask(hw,
				  hw->odr_table[ST_LSM6DSVXHG_ID_HG_ACC].reg.addr,
				  hw->odr_table[ST_LSM6DSVXHG_ID_HG_ACC].reg.mask,
				  &odr_xl);
	if (err < 0)
		return err;

	if (odr_xl == 0)
		return 0;

	for (i = 0; i < hw->odr_table[ST_LSM6DSVXHG_ID_HG_ACC].size; i++) {
		if (odr_xl ==
			hw->odr_table[ST_LSM6DSVXHG_ID_HG_ACC].odr_avl[i].val)
			break;
	}

	if (i == hw->odr_table[ST_LSM6DSVXHG_ID_HG_ACC].size)
		return -EINVAL;

	/* for frequency values with decimal part just return the integer */
	*xl_odr = hw->odr_table[ST_LSM6DSVXHG_ID_HG_ACC].odr_avl[i].hz;

	return err;
}

static int st_lsm6dsvxhg_get_default_hg_xl_odr(struct st_lsm6dsvxhg_hw *hw,
					       enum st_lsm6dsvxhg_event_id id,
					       int *xl_odr)
{
	int req_odr;
	int err;
	int odr;
	int index;

	err = st_lsm6dsvxhg_get_hg_xl_odr(hw, &odr);
	if (err < 0)
		return err;

	index = st_lsm6dsvxhg_hgevents_get_index(id);
	if (index < 0)
		return -EINVAL;

	req_odr = st_lsm6dsvxhg_hgevents[index].req_odr;
	if (odr > req_odr)
		*xl_odr = odr;
	else
		*xl_odr = req_odr;

	return err;
}

/*
 * st_lsm6dsvxhg_set_hg_wake_up_threshold - set wake-up threshold in mg
 * @hw - ST IMU MEMS hw instance
 * @wake_up_threshold_mg - wake-up threshold in mg
 */
static int st_lsm6dsvxhg_set_hg_wake_up_threshold(struct st_lsm6dsvxhg_hw *hw,
						  int wake_up_threshold_mg)
{
	u8 wake_up_threshold;
	int lsb, err;
	u32 fs_xl_g;

	if (wake_up_threshold_mg < 0)
		return -EINVAL;

	err = st_lsm6dsvxhg_get_hg_xl_fs(hw, &fs_xl_g);
	if (err < 0)
		return err;

	if (fs_xl_g <= 256)
		lsb = wake_up_threshold_mg / 1000;
	else
		lsb = wake_up_threshold_mg / 1250;

	wake_up_threshold = (u8)lsb;

	err = st_lsm6dsvxhg_write_locked(hw, ST_LSM6DSVXHG_HG_WAKE_UP_THS_ADDR,
					 wake_up_threshold);
	if (err < 0)
		return err;

	hw->wk_hg_th_mg = wake_up_threshold_mg;

	return 0;
}

/*
 * st_lsm6dsvxhg_set_hg_shock_duration - set wake-up duration in ms
 * @hw - ST IMU MEMS hw instance
 * @wake_up_duration_ms - wake-up duration in ms
 *
 * HG_SHOCK_DUR[s] = (HG_SHOCK_DUR_[3:0] + 1) * 512 / ODR_XL_HG
 * HG_SHOCK_DUR_[3:0] = ((HG_SHOCK_DUR[ms] * ODR_XL_HG) / 512000) - 1
 */
static int st_lsm6dsvxhg_set_hg_shock_duration(struct st_lsm6dsvxhg_hw *hw,
					       int shock_duration_ms)
{
	int tmp, sensor_odr, err;
	u8 shock_duration, max_dur;

	if (shock_duration_ms < 0)
		return -EINVAL;

	err = st_lsm6dsvxhg_get_default_hg_xl_odr(hw,
						  ST_SM6DSVXHG_EVENT_HG_SHOCK,
						  &sensor_odr);
	if (err < 0)
		return err;

	tmp = ((shock_duration_ms * sensor_odr) / 512000) - 1;
	if (tmp < 0)
		tmp = 0;

	shock_duration = (u8)tmp;
	max_dur = ST_LSM6DSVXHG_HG_SHOCK_DUR_MASK >>
		  __ffs(ST_LSM6DSVXHG_HG_SHOCK_DUR_MASK);

	if (shock_duration > max_dur)
		shock_duration = max_dur;

	err = st_lsm6dsvxhg_write_with_mask(hw,
					ST_LSM6DSVXHG_HG_FUNCTIONS_ENABLE_ADDR,
					ST_LSM6DSVXHG_HG_SHOCK_DUR_MASK,
					shock_duration);
	if (err < 0)
		return err;

	hw->shock_hg_dur_ms = shock_duration_ms;

	return 0;
}

int st_lsm6dsvxhg_read_hg_event_config(struct iio_dev *iio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir)
{
	if (chan->type == IIO_ACCEL) {
		struct st_lsm6dsvxhg_sensor *sensor = iio_priv(iio_dev);
		struct st_lsm6dsvxhg_hw *hw = sensor->hw;

		switch (type) {
		case IIO_EV_TYPE_THRESH:
			switch (dir) {
			case IIO_EV_DIR_RISING:
				return FIELD_GET(BIT(ST_SM6DSVXHG_EVENT_HG_WAKEUP),
						 hw->enable_ev_mask);

			default:
				return -EINVAL;
			}
			break;

		case IIO_EV_TYPE_CHANGE:
			switch (dir) {
			case IIO_EV_DIR_EITHER:
				return FIELD_GET(BIT(ST_SM6DSVXHG_EVENT_HG_SHOCK),
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

int st_lsm6dsvxhg_write_hg_event_config(struct iio_dev *iio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir,
					ST_IIO_EVENT_EN_TYPE enable)
{
	struct st_lsm6dsvxhg_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dsvxhg_hw *hw = sensor->hw;
	int req_odr = 0;
	int id = -1;
	u8 irq_mask;
	u8 irq_val;
	u8 irq_reg;
	int err;

	/* disable all event interrupts */
	err = st_lsm6dsvxhg_write_with_mask(hw,
				ST_LSM6DSVXHG_HG_FUNCTIONS_ENABLE_ADDR,
				ST_LSM6DSVXHG_HG_INTERRUPTS_ENABLE_MASK |
				ST_LSM6DSVXHG_INT1_HG_WU_MASK |
				ST_LSM6DSVXHG_INT2_HG_WU_MASK,
				0);
	if (err < 0)
		return err;

	err = st_lsm6dsvxhg_write_with_mask(hw,
				ST_LSM6DSVXHG_INACTIVITY_THS_ADDR,
				ST_LSM6DSVXHG_INT1_HG_SHOCK_CHANGE_MASK |
				ST_LSM6DSVXHG_INT2_HG_SHOCK_CHANGE_MASK, 0);
	if (err < 0)
		return err;

	if (chan->type == IIO_ACCEL) {
		int index;

		switch (type) {
		case IIO_EV_TYPE_THRESH:
			switch (dir) {
			/*
			 * this is the wk event, use the dir IIO_EV_DIR_RISING
			 * because don't exist a specific iio_event_type related
			 * to wakeup events
			 */
			case IIO_EV_DIR_RISING:
				/*
				 * if shock_hg_dur_ms is 0 we can detect only
				 * wake-up events
				 */
				if (hw->shock_hg_dur_ms == 0)
					id = ST_SM6DSVXHG_EVENT_HG_WAKEUP;
				else
					id = ST_SM6DSVXHG_EVENT_HG_SHOCK;

				index = st_lsm6dsvxhg_hgevents_get_index(id);
				if (index < 0)
					return -EINVAL;

				irq_reg = st_lsm6dsvxhg_hgevents[index].irq_reg;
				irq_mask = st_lsm6dsvxhg_hgevents[index].irq_mask;

				if (enable) {
					irq_val = hw->int_pin == 1 ? BIT(0) : BIT(1);
					req_odr = st_lsm6dsvxhg_hgevents[index].req_odr;
				} else {
					irq_val = 0;
					req_odr = 0;
				}
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

	err = st_lsm6dsvxhg_write_with_mask(hw, irq_reg, irq_mask, irq_val);
	if (err < 0)
		return err;

	err = st_lsm6dsvxhg_write_with_mask(hw,
				ST_LSM6DSVXHG_HG_FUNCTIONS_ENABLE_ADDR,
				ST_LSM6DSVXHG_HG_INTERRUPTS_ENABLE_MASK,
				!!enable);
	if (err < 0)
		return err;

	err = st_lsm6dsvxhg_set_odr(iio_priv(hw->iio_devs[ST_LSM6DSVXHG_ID_HG_ACC]),
					     true, req_odr, 0);
	if (err < 0)
		return err;

	if (enable == 0)
		hw->enable_ev_mask &= ~BIT_ULL(id);
	else
		hw->enable_ev_mask |= BIT_ULL(id);

	return err;
}

int st_lsm6dsvxhg_read_hg_event_value(struct iio_dev *iio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      enum iio_event_info info,
				      int *val, int *val2)
{
	struct st_lsm6dsvxhg_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dsvxhg_hw *hw = sensor->hw;
	int err = -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_RISING:
			/*
			 * wake-up is classified as threshold event with dir
			 * rising
			 */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				*val = (int)hw->wk_hg_th_mg;

				return IIO_VAL_INT;

			case IIO_EV_INFO_PERIOD:
				*val = (int)hw->shock_hg_dur_ms;

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

int st_lsm6dsvxhg_write_hg_event_value(struct iio_dev *iio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       enum iio_event_info info,
				       int val, int val2)
{
	struct st_lsm6dsvxhg_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dsvxhg_hw *hw = sensor->hw;
	int err = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_RISING:
			/* wake-up event configuration */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				err = st_lsm6dsvxhg_set_hg_wake_up_threshold(hw, val);
				break;

			case IIO_EV_INFO_PERIOD:
				err = st_lsm6dsvxhg_set_hg_shock_duration(hw, val);
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

int st_lsm6dsvxhg_hg_event_handler(struct st_lsm6dsvxhg_hw *hw)
{
	struct iio_dev *iio_dev;
	u8 reg_src;
	int err;

	if (!st_lsm6dsvxhg_hgevents_enabled(hw))
		return IRQ_HANDLED;

	err = regmap_bulk_read(hw->regmap,
			       ST_LSM6DSVXHG_HG_WAKE_UP_SRC_ADDR,
			       &reg_src, sizeof(reg_src));
	if (err < 0)
		return IRQ_HANDLED;

	if ((reg_src & ST_LSM6DSVXHG_HG_WU_IA_MASK) &&
	    ST_LSM6DSVXHG_IS_HG_EVENT_ENABLED(ST_SM6DSVXHG_EVENT_HG_WAKEUP)) {
		iio_dev = hw->iio_devs[ST_LSM6DSVXHG_ID_HG_ACC];
		if (reg_src & ST_LSM6DSVXHG_HG_X_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Z,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}

		if (reg_src & ST_LSM6DSVXHG_HG_Y_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_Y,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}

		if (reg_src & ST_LSM6DSVXHG_HG_X_WU_MASK) {
			iio_push_event(iio_dev,
				       IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							  IIO_MOD_X,
							  IIO_EV_TYPE_THRESH,
							  IIO_EV_DIR_RISING),
							  iio_get_time_ns(iio_dev));
		}
	}

	return IRQ_HANDLED;
}

int st_lsm6dsvxhg_update_hg_threshold_events(struct st_lsm6dsvxhg_hw *hw)
{
	int ret;

	ret = st_lsm6dsvxhg_set_hg_wake_up_threshold(hw, hw->wk_hg_th_mg);

	return ret < 0 ? ret : 0;
}

int st_lsm6dsvxhg_update_hg_duration_events(struct st_lsm6dsvxhg_hw *hw)
{
	int ret;

	ret = st_lsm6dsvxhg_set_hg_shock_duration(hw, hw->wk_dur_ms);

	return ret < 0 ? ret : 0;
}

/*
 * Configure the high-g xl based events function default threshold
 * and duration/delay
 *
 * wake_up_threshold = 2500 mg
 * shock_duration = 0 ms
 */
int st_lsm6dsvxhg_hg_event_init(struct st_lsm6dsvxhg_hw *hw)
{
	int err;

	/* set default wake-up threshold to 2500 mg */
	err = st_lsm6dsvxhg_set_hg_wake_up_threshold(hw, 2500);
	if (err < 0)
		return err;

	/* set default shock duration to 0 */
	err = st_lsm6dsvxhg_set_hg_shock_duration(hw, 0);

	return err < 0 ? err : 0;
}

