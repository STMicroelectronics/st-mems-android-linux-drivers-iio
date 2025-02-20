// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_acc33 xl based events function sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2024 STMicroelectronics Inc.
 */

#include <linux/bitfield.h>
#include <linux/kernel.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/module.h>
#include <linux/version.h>

#include "st_acc33.h"

#define REG_INT1_SRC_ADDR			0x31
#define REG_INT1_SRC_IA_MASK			BIT(6)
#define REG_INT1_SRC_ZYX_MASK			GENMASK(5, 0)

#define REG_INT1_THS_ADDR			0x32
#define REG_INT1_THS_MASK			0x7f

#define REG_INT1_DURATION_ADDR			0x33
#define REG_INT1_DURATION_MASK			0x7f

#define ST_ACC33_MIN_ODR_IN_WAKEUP		100
#define ST_ACC33_MIN_ODR_IN_FREEFALL		100

#define ST_ACC33_IS_EVENT_ENABLED(_event_id)	(!!(hw->enable_ev_mask & \
						    BIT(_event_id)))

static const struct st_acc33_event_t {
	enum st_acc33_event_id id;
	char *name;
	int req_odr;
} st_acc33_events[] = {
	[ST_ACC33_EVENT_FF] = {
		.id = ST_ACC33_EVENT_FF,
		.name = "free_fall",
		.req_odr = ST_ACC33_MIN_ODR_IN_FREEFALL,
	},
	[ST_ACC33_EVENT_WAKEUP] = {
		.id = ST_ACC33_EVENT_WAKEUP,
		.name = "wake_up",
		.req_odr = ST_ACC33_MIN_ODR_IN_WAKEUP,
	},
};

static inline bool
st_acc33_events_enabled(struct st_acc33_hw *hw)
{
	return (ST_ACC33_IS_EVENT_ENABLED(ST_ACC33_EVENT_FF) ||
		ST_ACC33_IS_EVENT_ENABLED(ST_ACC33_EVENT_WAKEUP));
}

/*
 * st_acc33_get_th_lsb - get accel lsb threshold in mg
 * @hw - ST IMU MEMS hw instance
 *
 * 1 LSb = 16 mg @ FS = 2 g
 * 1 LSb = 32 mg @ FS = 4 g
 * 1 LSb = 62 mg @ FS = 8 g
 * 1 LSb = 186 mg @ FS = 16 g
 */
static u8 st_acc33_get_th_lsb(struct st_acc33_hw *hw)
{
	u8 lsb[] = { 16, 32, 62, 186 };

	return lsb[__ffs(hw->fs) - 1];
}

static int st_acc33_get_xl_odr(struct st_acc33_hw *hw,
			       enum st_acc33_event_id id)
{
	return max_t(int, st_acc33_events[id].req_odr, hw->odr);
}

/*
 * st_acc33_set_thershold - set accel threshold in mg
 * @hw - ST IMU MEMS hw instance
 * @thershold_mg - accel threshold in mg
 */
static int st_acc33_set_thershold(struct st_acc33_hw *hw, int thershold_mg)
{
	u8 threg = REG_INT1_THS_ADDR;
	u8 thershold;
	int err;
	u8 lsb;

	lsb = st_acc33_get_th_lsb(hw);

	thershold = thershold_mg / lsb;
	thershold = min_t(u8, thershold, REG_INT1_THS_MASK);

	err = st_acc33_write_with_mask(hw, threg, REG_INT1_THS_MASK, thershold);
	if (err < 0)
		return err;

	hw->xl_th_mg = thershold_mg;

	return 0;
}

/*
 * st_acc33_set_duration - set duration in ms
 * @hw - ST IMU MEMS hw instance
 * @duration_ms - accel duration in ms
 */
static int st_acc33_set_duration(struct st_acc33_hw *hw, int duration_ms)
{
	u8 dureg = REG_INT1_DURATION_ADDR;
	int sensor_odr, err;
	u8 duration;

	sensor_odr = st_acc33_get_xl_odr(hw, ST_ACC33_EVENT_WAKEUP);

	duration = (duration_ms * sensor_odr) / 1000;
	duration = min_t(u8, duration, REG_INT1_DURATION_MASK);

	err = st_acc33_write_with_mask(hw, dureg,
				       REG_INT1_DURATION_MASK, duration);
	if (err < 0)
		return err;

	hw->duration_ms = duration_ms;

	return 0;
}

int st_acc33_read_event_config(struct iio_dev *iio_dev,
			       const struct iio_chan_spec *chan,
			       enum iio_event_type type,
			       enum iio_event_direction dir)
{
	if (chan->type == IIO_ACCEL) {
		struct st_acc33_hw *hw = iio_priv(iio_dev);

		switch (type) {
		case IIO_EV_TYPE_THRESH:
			switch (dir) {
			case IIO_EV_DIR_FALLING:
				return FIELD_GET(BIT(ST_ACC33_EVENT_FF),
						 hw->enable_ev_mask);
			case IIO_EV_DIR_RISING:
				return FIELD_GET(BIT(ST_ACC33_EVENT_WAKEUP),
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
st_acc33_check_events_odr_dependency(struct st_acc33_hw *hw, int odr,
				     enum st_acc33_event_id id, int enable)
{
	int i, ret = 0;
	int current_odr;

	for (i = 0; i < ARRAY_SIZE(st_acc33_events); i++) {
		current_odr = st_acc33_events[i].req_odr;

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

int st_acc33_write_event_config(struct iio_dev *iio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				int enable)
{
	struct st_acc33_hw *hw = iio_priv(iio_dev);
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
				id = ST_ACC33_EVENT_FF;
				int_val = REG_INT1_CFG_AOI_MASK |
					  REG_INT1_CFG_ZLIE_MASK |
					  REG_INT1_CFG_YLIE_MASK |
					  REG_INT1_CFG_XLIE_MASK;
				break;
			/*
			 * this is the wk event, use the dir IIO_EV_DIR_RISING
			 * because don't exist a specific iio_event_type related
			 * to wakeup events
			 */
			case IIO_EV_DIR_RISING:
				id = ST_ACC33_EVENT_WAKEUP;
				int_val = REG_INT1_CFG_ZHIE_MASK |
					  REG_INT1_CFG_YHIE_MASK |
					  REG_INT1_CFG_XHIE_MASK;

#ifdef ST_ACC33_ENABLE_HPF
				if (!hw->enable) {
					u8 data;

					/*
					 * high-pass filter enabled on
					 * interrupt activity
					 */
					err = hw->tf->read(hw->dev,
							   REG_CTRL2_ADDR, 1,
							   &data);
					if (err < 0)
						return err;

					if (enable)
						data |= (REG_CTRL2_FDS_MASK |
							 REG_CTRL2_HPIS1_MASK);
					else
						data &= ~(REG_CTRL2_FDS_MASK |
							  REG_CTRL2_HPIS1_MASK);

					err = hw->tf->write(hw->dev,
							    REG_CTRL2_ADDR, 1,
							    &data);
					if (err < 0)
						return err;
					}
#endif /* ST_ACC33_ENABLE_HPF */

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

	/* interrupt activity 1 driven to INT1 pin */
	err = st_acc33_write_with_mask(hw, REG_CTRL3_ADDR,
				       REG_CTRL3_I1_AOI1_MASK, !!enable);
	if (err < 0)
		return err;

	/* interrupt 1 pin latched */
	err = st_acc33_write_with_mask(hw, REG_CTRL5_ACC_ADDR,
				       REG_CTRL5_ACC_LIR_INT1_MASK, !!enable);
	if (err < 0)
		return err;

	/* configure interrupt generation */
	err = hw->tf->write(hw->dev, REG_INT1_CFG_ADDR, 1, &int_val);
	if (err < 0)
		return err;

	/* check enabled events ODR dependency */
	req_odr = st_acc33_check_events_odr_dependency(hw,
						    st_acc33_events[id].req_odr,
						    id, enable);

	/* update xl odr */
	req_odr = max_t(int, hw->odr, enable ? req_odr : 0);

	err = st_acc33_update_odr(hw, req_odr, true, enable);
	if (err < 0)
		return err;

	if (enable == 0)
		hw->enable_ev_mask &= ~BIT(id);
	else
		hw->enable_ev_mask |= BIT(id);

	return err;
}

int st_acc33_read_event_value(struct iio_dev *iio_dev,
			      const struct iio_chan_spec *chan,
			      enum iio_event_type type,
			      enum iio_event_direction dir,
			      enum iio_event_info info,
			      int *val, int *val2)
{
	struct st_acc33_hw *hw = iio_priv(iio_dev);
	int err = -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			/*
			 * free fall is classified as threshold event with dir
			 * falling
			 */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				*val = (int)hw->xl_th_mg;

				return IIO_VAL_INT;
			case IIO_EV_INFO_PERIOD:
				*val = (int)hw->duration_ms;

				return IIO_VAL_INT;
			default:
				break;
			}
			break;
		case IIO_EV_DIR_RISING:
			/*
			 * wake-up is classified as threshold event with dir
			 * rising
			 */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				*val = (int)hw->xl_th_mg;

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

int st_acc33_write_event_value(struct iio_dev *iio_dev,
			       const struct iio_chan_spec *chan,
			       enum iio_event_type type,
			       enum iio_event_direction dir,
			       enum iio_event_info info,
			       int val, int val2)
{
	struct st_acc33_hw *hw = iio_priv(iio_dev);
	int err = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			switch (info) {
			/* free fall event configuration */
			case IIO_EV_INFO_VALUE:
				err = st_acc33_set_thershold(hw, val);
				break;
			case IIO_EV_INFO_PERIOD:
				err = st_acc33_set_duration(hw, val);
				break;
			default:
				break;
			}
			break;
		case IIO_EV_DIR_RISING:
			/* wake-up event configuration */
			switch (info) {
			case IIO_EV_INFO_VALUE:
				err = st_acc33_set_thershold(hw, val);
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

int st_acc33_event_handler(struct st_acc33_hw *hw)
{
	u8 int_src;
	s64 event;
	int err;

	if (!st_acc33_events_enabled(hw))
		return IRQ_HANDLED;

	err = hw->tf->read(hw->dev, REG_INT1_SRC_ADDR, 1, &int_src);
	if (err < 0) {
		dev_err(hw->dev, "failed to read INT1_SRC register\n");

		return IRQ_HANDLED;
	}

	if (int_src & REG_INT1_SRC_IA_MASK) {
		if (ST_ACC33_IS_EVENT_ENABLED(ST_ACC33_EVENT_WAKEUP)) {
			event = IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(hw->iio_dev, event,
				       iio_get_time_ns(hw->iio_dev));
		}

		if (ST_ACC33_IS_EVENT_ENABLED(ST_ACC33_EVENT_FF)) {
			event = IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_FALLING);
			iio_push_event(hw->iio_dev, event,
				       iio_get_time_ns(hw->iio_dev));
		}
	}

	return IRQ_HANDLED;
}

int st_acc33_update_threshold_events(struct st_acc33_hw *hw)
{
	int ret;

	ret = st_acc33_set_thershold(hw, hw->xl_th_mg);

	return ret < 0 ? ret : 0;
}

int st_acc33_update_duration_events(struct st_acc33_hw *hw)
{
	int ret;

	ret = st_acc33_set_duration(hw, hw->duration_ms);

	return ret < 0 ? ret : 0;
}

/*
 * Configure the xl based events function default threshold and duration/delay
 *
 * thershold = 100 mg
 * duration = 0 ms
 */
int st_acc33_event_init(struct st_acc33_hw *hw)
{
	int err;

	/* Set default thershold to 100 mg */
	err = st_acc33_set_thershold(hw, 100);
	if (err < 0)
		return err;

	/* Set default duration to 0 ms */
	err = st_acc33_set_duration(hw, 0);

	return err < 0 ? err : 0;
}
