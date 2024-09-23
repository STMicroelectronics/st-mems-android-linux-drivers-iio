// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_ism330dhcx machine learning core driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2022 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/firmware.h>
#include <linux/version.h>

#include "st_ism330dhcx.h"

#define ST_ISM330DHCX_MLC_LOADER_VERSION	"0.8"

/* number of machine learning core available on device hardware */
#define ST_ISM330DHCX_MLC_NUMBER		8
#define ST_ISM330DHCX_FSM_NUMBER		16
#define ST_ISM330DHCX_MLC_FIRMWARE_NAME		"st_ism330dhcx_mlc.bin"

#ifdef CONFIG_IIO_ST_ISM330DHCX_MLC_PRELOAD
#include "st_ism330dhcx_preload_mlc.h"
#endif /* CONFIG_IIO_ST_ISM330DHCX_MLC_PRELOAD */

#define FSM_PAGE(__addr)			((u8)(((__addr >> 8) << 4) | 0x01))
#define FSM_PAGE_MASK(__addr)			((u8)(__addr >> 8))
#define FSM_OFFSET(__addr)			((u8)(__addr & 0x00FF))

/* specific events for MLC */
#define ST_ISM330DHCX_MLC_FSM_TRSD_UPDATE	1

static struct iio_dev *st_ism330dhcx_mlc_alloc_iio_dev(struct st_ism330dhcx_hw *hw,
						       enum st_ism330dhcx_sensor_id id);

/* FSM / MLC IIO event channel definition */
static const struct iio_chan_spec st_ism330dhcx_mlc_fsm_x_ch[] = {
	ST_ISM330DHCX_EVENT_CHANNEL(IIO_ACTIVITY, thr),
};

static const unsigned long st_ism330dhcx_fsm_mlc_available_scan_masks[] = {
	BIT(0), 0x0
};

/* remove old mlc/fsm configuration */
static int st_ism330dhcx_mlc_purge_config(struct st_ism330dhcx_hw *hw)
{
	int err;

	err = st_ism330dhcx_update_page_bits_locked(hw,
		 ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
		 ST_ISM330DHCX_MLC_EN_MASK,
		 ST_ISM330DHCX_SHIFT_VAL(0, ST_ISM330DHCX_MLC_EN_MASK));
	if (err < 0)
		return err;

	err = st_ism330dhcx_update_page_bits_locked(hw,
		 ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
		 ST_ISM330DHCX_FSM_EN_MASK,
		 ST_ISM330DHCX_SHIFT_VAL(0, ST_ISM330DHCX_FSM_EN_MASK));
	if (err < 0)
		return err;

	/* wait ~10 ms */
	usleep_range(10000, 10100);

	return 0;
}

static int
st_ism330dhcx_mlc_enable_sensor(struct st_ism330dhcx_sensor *sensor,
				bool enable)
{
	struct st_ism330dhcx_hw *hw = sensor->hw;
	int err = -ENODEV;

	if (sensor->id >= ST_ISM330DHCX_ID_MLC_0 &&
	    sensor->id <= ST_ISM330DHCX_ID_MLC_7) {
		int int_mlc_value;
		int mlc_running;

		mlc_running = st_ism330dhcx_mlc_running(hw);

		int_mlc_value = enable ? hw->mlc_config->mlc_int_mask : 0;
		if (hw->mlc_config->mlc_int_pin & BIT(0)) {
			err = st_ism330dhcx_write_page_locked(hw,
					ST_ISM330DHCX_MLC_INT1_ADDR,
					&int_mlc_value, 1);
			if (err < 0)
				return err;
		}

		if (hw->mlc_config->mlc_int_pin & BIT(1)) {
			err = st_ism330dhcx_write_page_locked(hw,
					ST_ISM330DHCX_MLC_INT2_ADDR,
					&int_mlc_value, 1);
			if (err < 0)
				return err;
		}

		err = st_ism330dhcx_sensor_set_enable(sensor, enable);
		if (err < 0)
			return err;

		/* check for any other mlc already enabled */
		if ((!mlc_running && st_ism330dhcx_mlc_running(hw)) ||
		    (mlc_running && !st_ism330dhcx_mlc_running(hw))) {
			dev_info(sensor->hw->dev, "Reset MLC Algos\n");
			err = st_ism330dhcx_update_page_bits_locked(hw,
				   ST_ISM330DHCX_REG_EMB_FUNC_INIT_B_ADDR,
				   ST_ISM330DHCX_MLC_INIT_MASK,
				   ST_ISM330DHCX_SHIFT_VAL(1,
						ST_ISM330DHCX_MLC_INIT_MASK));
			if (err < 0)
				return err;
		}

		dev_info(sensor->hw->dev,
			 "%s MLC sensor %d (INT %x)\n",
			 enable ? "Enabling" : "Disabling",
			 sensor->id, int_mlc_value);
	}

	return err < 0 ? err : 0;
}

static int
st_ism330dhcx_fsm_enable_sensor(struct st_ism330dhcx_sensor *sensor,
				bool enable)
{
	struct st_ism330dhcx_hw *hw = sensor->hw;
	int fsm_running = st_ism330dhcx_fsm_running(hw);
	int id = sensor->id;
	u8 mask, bitmask;
	int err = 0;

	if (id >= ST_ISM330DHCX_ID_FSM_0 &&
	    id < ST_ISM330DHCX_ID_FSM_8) {
		mask = BIT(id - ST_ISM330DHCX_ID_FSM_0);
		err = st_ism330dhcx_update_page_bits_locked(hw,
				ST_ISM330DHCX_FSM_ENABLE_A_ADDR,
				mask,
				ST_ISM330DHCX_SHIFT_VAL(enable ? 1 : 0,
						mask));
		if (err < 0)
			return err;

		/* enable interrupts only if requested by ucf */
		bitmask = mask & hw->mlc_config->fsm_int_mask[0];
		if (bitmask) {
			if (hw->mlc_config->mlc_int_pin  & BIT(0)) {
				err = st_ism330dhcx_update_page_bits_locked(hw,
						ST_ISM330DHCX_FSM_INT1_A_ADDR,
						bitmask,
						ST_ISM330DHCX_SHIFT_VAL(enable ? 1 : 0, bitmask));
				if (err < 0)
					return err;
			}

			if (hw->mlc_config->mlc_int_pin  & BIT(1)) {
				err = st_ism330dhcx_update_page_bits_locked(hw,
						ST_ISM330DHCX_FSM_INT2_A_ADDR,
						bitmask,
						ST_ISM330DHCX_SHIFT_VAL(enable ? 1 : 0, bitmask));
				if (err < 0)
					return err;
			}
		}

		dev_info(sensor->hw->dev,
			 "%s FSM A sensor %d (INT %x)\n",
			 enable ? "Enabling" : "Disabling",
			 id, bitmask);
	} else if (id >= ST_ISM330DHCX_ID_FSM_8 &&
		   id < ST_ISM330DHCX_ID_FSM_15) {
		mask = BIT(id - ST_ISM330DHCX_ID_FSM_8);
		err = st_ism330dhcx_update_page_bits_locked(hw,
				ST_ISM330DHCX_FSM_ENABLE_B_ADDR,
				mask,
				ST_ISM330DHCX_SHIFT_VAL(enable ? 1 : 0,
						mask));
		if (err < 0)
			return err;

		bitmask = mask & hw->mlc_config->fsm_int_mask[1];
		if (bitmask) {
			if (hw->mlc_config->mlc_int_pin  & BIT(0)) {
				err = st_ism330dhcx_update_page_bits_locked(hw,
						ST_ISM330DHCX_FSM_INT1_B_ADDR,
						bitmask,
						ST_ISM330DHCX_SHIFT_VAL(enable ? 1 : 0, bitmask));
				if (err < 0)
					return err;
			}

			if (hw->mlc_config->mlc_int_pin  & BIT(1)) {
				err = st_ism330dhcx_update_page_bits_locked(hw,
						ST_ISM330DHCX_FSM_INT2_B_ADDR,
						bitmask,
						ST_ISM330DHCX_SHIFT_VAL(enable ? 1 : 0, bitmask));
				if (err < 0)
					return err;
			}
		}

		dev_info(sensor->hw->dev,
			 "%s FSM B sensor %d (INT %x)\n",
			 enable ? "Enabling" : "Disabling",
			 id, bitmask);
	} else {
		dev_info(sensor->hw->dev, "Invalid fsm id %d\n", id);

		return -ENODEV;
	}

	err = st_ism330dhcx_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	/* check for any other fsm already enabled */
	if ((!fsm_running && st_ism330dhcx_fsm_running(hw)) ||
	    (fsm_running && !st_ism330dhcx_fsm_running(hw))) {
		dev_info(sensor->hw->dev, "Reset FSM Algos\n");
		err = st_ism330dhcx_update_page_bits_locked(hw,
			   ST_ISM330DHCX_REG_EMB_FUNC_INIT_B_ADDR,
			   ST_ISM330DHCX_FSM_INIT_MASK,
			   ST_ISM330DHCX_SHIFT_VAL(1,
					ST_ISM330DHCX_FSM_INIT_MASK));
		if (err < 0)
			return err;

		err = st_ism330dhcx_update_page_bits_locked(hw,
				ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
				ST_ISM330DHCX_FSM_EN_MASK,
				ST_ISM330DHCX_SHIFT_VAL(enable,
					ST_ISM330DHCX_FSM_EN_MASK));
		if (err < 0)
			return err;
	}

	return 0;
}

static int
st_ism330dhcx_mlc_fsm_enable_sensor(struct st_ism330dhcx_sensor *sensor,
				    bool enable)
{
	int err = 0;

	if (sensor->status == ST_ISM330DHCX_MLC_ENABLED) {
		if (sensor->id == ST_ISM330DHCX_ID_MLC) {
			sensor->hw->enable_mask &= ~BIT_ULL(sensor->id);
			if (enable)
				sensor->hw->enable_mask |= BIT_ULL(sensor->id);
		} else {
			err = st_ism330dhcx_mlc_enable_sensor(sensor, enable);
		}
	} else if (sensor->status == ST_ISM330DHCX_FSM_ENABLED) {
		err = st_ism330dhcx_fsm_enable_sensor(sensor, enable);
	} else {
		return -ENODEV;
	}

	return err < 0 ? err : 0;
}

static int st_ism330dhcx_mlc_write_event_config(struct iio_dev *iio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				int state)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);

	return st_ism330dhcx_mlc_fsm_enable_sensor(sensor, state);
}

static int st_ism330dhcx_mlc_read_event_config(struct iio_dev *iio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	struct st_ism330dhcx_hw *hw = sensor->hw;

	return !!(hw->enable_mask & BIT_ULL(sensor->id));
}

/* parse and program mlc / fsm fragments */
static int st_ism330dhcx_program_mlc(const struct firmware *fw,
				     struct st_ism330dhcx_hw *hw)
{
	u8 fsm_int[2] = { 0, 0 }, fsm_enable[2] = { 0, 0 };
	u8 mlc_fsm_en = 0, fsm_mlc_requested_odr = 0;
	u8 mlc_int = 0, mlc_num = 0, fsm_num = 0;
	bool stmc_page = false, skip = false;
	int reg, val, ret, i = 0;
	u32 uodr = 0;
	u16 odr = 0;

	if (!hw->preload_mlc) {
		ret = st_ism330dhcx_mlc_purge_config(hw);
		if (ret < 0)
			return ret;
	}

	mutex_lock(&hw->page_lock);

	while (i < fw->size) {
		reg = fw->data[i++];
		val = fw->data[i++];

		if (reg == 0x01 && val == 0x80) {
			stmc_page = true;
		} else if (reg == 0x01 && val == 0x00) {
			stmc_page = false;
		} else if (stmc_page) {
			/* catch configuration in stmc page */
			switch (reg) {
			case ST_ISM330DHCX_MLC_INT1_ADDR:
			case ST_ISM330DHCX_MLC_INT2_ADDR:
				mlc_int |= val;
				mlc_num = hweight8(mlc_int);
				skip = true;
				break;
			case ST_ISM330DHCX_FSM_INT1_A_ADDR:
			case ST_ISM330DHCX_FSM_INT2_A_ADDR:
				fsm_int[0] |= val;
				fsm_num = hweight16(*(u16 *)fsm_int);
				skip = true;
				break;
			case ST_ISM330DHCX_FSM_INT1_B_ADDR:
			case ST_ISM330DHCX_FSM_INT2_B_ADDR:
				fsm_int[1] |= val;
				fsm_num = hweight16(*(u16 *)fsm_int);
				skip = true;
				break;
			case ST_ISM330DHCX_FSM_ENABLE_A_ADDR:
				fsm_enable[0] |= val;
				skip = true;
				break;
			case ST_ISM330DHCX_FSM_ENABLE_B_ADDR:
				fsm_enable[1] |= val;
				skip = true;
				break;
			case ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR:
				/*
				 * check if mlc or fsm need to be
				 * enabled even if the interrupts are
				 * not used
				 */
				mlc_fsm_en |= val;
				skip = true;
				break;
			default:
				break;
			}
		} else if (!stmc_page) {
			/* catch configuration in page 0 */
			switch (reg) {
			/* skip FIFO and interrupt registers */
			case ST_ISM330DHCX_REG_FIFO_CTRL1_ADDR:
			case ST_ISM330DHCX_REG_FIFO_CTRL2_ADDR:
			case ST_ISM330DHCX_REG_FIFO_CTRL3_ADDR:
			case ST_ISM330DHCX_REG_FIFO_CTRL4_ADDR:
			case ST_ISM330DHCX_REG_INT1_CTRL_ADDR:
			case ST_ISM330DHCX_REG_INT2_CTRL_ADDR:
			case ST_ISM330DHCX_REG_CTRL3_C_ADDR:
			case ST_ISM330DHCX_REG_CTRL4_C_ADDR:
			case ST_ISM330DHCX_REG_CTRL5_C_ADDR:
			case ST_ISM330DHCX_REG_CTRL6_C_ADDR:
			case ST_ISM330DHCX_REG_CTRL7_G_ADDR:
			case ST_ISM330DHCX_REG_CTRL10_C_ADDR:
				skip = true;
				break;
			/* save requested odr for later */
			case ST_ISM330DHCX_CTRL1_XL_ADDR:
				fsm_mlc_requested_odr = val >> 4;
				skip = true;
				break;
			default:
				break;
			}
		}

		if (!skip) {
			ret = regmap_write(hw->regmap, reg, val);
			if (ret) {
				dev_err(hw->dev, "regmap_write fails\n");

				goto unlock_page;
			}
		}

		skip = false;

		if (mlc_num >= ST_ISM330DHCX_MLC_NUMBER ||
		    fsm_num >= ST_ISM330DHCX_FSM_NUMBER)
			break;
	}

	/* if MLC/FSM ODR is not configured uses first available */
	if (!fsm_mlc_requested_odr)
		fsm_mlc_requested_odr = 0x01;

	ret = st_ism330dhcx_get_odr_from_reg(ST_ISM330DHCX_ID_ACC,
					     fsm_mlc_requested_odr,
					     &odr, &uodr);
	if (ret < 0) {
		fsm_num = 0;
		mlc_num = 0;

		dev_err(hw->dev,
			"unsupported ODR %d for MLC/FSM\n",
			fsm_mlc_requested_odr);

		goto unlock_page;
	}

	if (mlc_num) {
		hw->mlc_config->mlc_int_mask = mlc_int;
		hw->mlc_config->status |= ST_ISM330DHCX_MLC_ENABLED;
		hw->mlc_config->mlc_configured = mlc_num;
	}

	if (fsm_num) {
		hw->mlc_config->fsm_int_mask[0] = fsm_int[0];
		hw->mlc_config->fsm_int_mask[1] = fsm_int[1];

		hw->mlc_config->status |= ST_ISM330DHCX_FSM_ENABLED;
		hw->mlc_config->fsm_configured = fsm_num;

		hw->mlc_config->fsm_enabled_mask[0] = fsm_enable[0];
		hw->mlc_config->fsm_enabled_mask[1] = fsm_enable[1];
	}

	hw->mlc_config->mlc_fsm_en = mlc_fsm_en;
	hw->mlc_config->bin_len = fw->size;
	hw->mlc_config->fsm_mlc_requested_odr = odr;
	hw->mlc_config->fsm_mlc_requested_uodr = uodr;

unlock_page:
	mutex_unlock(&hw->page_lock);

	return (fsm_num + mlc_num) > 0 ? fsm_num + mlc_num : 0;
}

static void st_ism330dhcx_mlc_update(const struct firmware *fw,
				     void *context)
{
	bool force_mlc_enabled, force_fsm_enabled = false;
	struct st_ism330dhcx_hw *hw = context;
	enum st_ism330dhcx_sensor_id id;
	u16 fsm_mask = 0;
	u8 mlc_mask = 0;
	int ret, i;

	force_mlc_enabled = true;

	if (!fw) {
		dev_err(hw->dev, "could not get binary firmware\n");
		return;
	}

	mutex_lock(&hw->fifo_lock);

	ret = st_ism330dhcx_program_mlc(fw, hw);
	if (!ret)
		goto release;

	mlc_mask = hw->mlc_config->mlc_int_mask;
	for (i = 0; i < ST_ISM330DHCX_MLC_NUMBER; i++) {
		if (mlc_mask & BIT(i)) {
			id = st_ism330dhcx_mlc_sensor_list[i];
			hw->iio_devs[id] =
				st_ism330dhcx_mlc_alloc_iio_dev(hw, id);
			if (!hw->iio_devs[id])
				goto release;

			ret = iio_device_register(hw->iio_devs[id]);
			if (ret)
				goto release;
		}
	}

	fsm_mask = (u16)(((u16)hw->mlc_config->fsm_enabled_mask[1] << 8) |
			  hw->mlc_config->fsm_enabled_mask[0]);
	for (i = 0; i < ST_ISM330DHCX_FSM_NUMBER; i++) {
		if (fsm_mask & BIT(i)) {
			id = st_ism330dhcx_fsm_sensor_list[i];
			hw->iio_devs[id] =
				st_ism330dhcx_mlc_alloc_iio_dev(hw, id);
			if (!hw->iio_devs[id])
				goto release;

			ret = iio_device_register(hw->iio_devs[id]);
			if (ret)
				goto release;
		}
	}

	/*
	 * check if int are not configured but mlc/fsm need to
	 * be enabled
	 */
	if ((hw->mlc_config->mlc_fsm_en & ST_ISM330DHCX_MLC_EN_MASK) &&
	    (!hw->mlc_config->mlc_int_mask))
		force_mlc_enabled = true;

	if ((hw->mlc_config->mlc_fsm_en & ST_ISM330DHCX_FSM_EN_MASK) &&
	    (!hw->mlc_config->fsm_int_mask[0]) &&
	    (!hw->mlc_config->fsm_int_mask[1]))
		force_fsm_enabled = true;

	if (force_mlc_enabled) {
		ret = st_ism330dhcx_update_page_bits_locked(hw,
				ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
				ST_ISM330DHCX_MLC_EN_MASK,
				ST_ISM330DHCX_SHIFT_VAL(true,
					    ST_ISM330DHCX_MLC_EN_MASK));
		if (ret < 0)
			goto release;
	}

	if (force_fsm_enabled) {
		ret = st_ism330dhcx_update_page_bits_locked(hw,
				ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
				ST_ISM330DHCX_FSM_EN_MASK,
				ST_ISM330DHCX_SHIFT_VAL(true,
					ST_ISM330DHCX_FSM_EN_MASK));
		if (ret < 0)
			goto release;
	}

	dev_info(hw->dev, "MLC loaded (%d) MLC %x FSM %x-%x (MLC %s FSM %s)\n",
		 ret, mlc_mask,
		 (fsm_mask >> 8) & 0xFF, fsm_mask & 0xFF,
		 force_mlc_enabled ? "Forced" : "On req",
		 force_fsm_enabled ? "Forced" : "On req");

release:
	mutex_unlock(&hw->fifo_lock);

	if (hw->preload_mlc) {
		hw->preload_mlc = 0;

		return;
	}

	release_firmware(fw);
}

static int st_ism330dhcx_mlc_flush_all(struct st_ism330dhcx_hw *hw)
{
	struct st_ism330dhcx_sensor *sensor_mlc;
	struct iio_dev *iio_dev;
	int ret = 0, id, i;

	for (i = 0; i < ARRAY_SIZE(st_ism330dhcx_mlc_sensor_list); i++) {
		id = st_ism330dhcx_mlc_sensor_list[i];
		iio_dev = hw->iio_devs[id];
		if (!iio_dev)
			continue;

		sensor_mlc = iio_priv(iio_dev);
		ret = st_ism330dhcx_mlc_fsm_enable_sensor(sensor_mlc,
							  false);
		if (ret < 0)
			break;

		iio_device_unregister(iio_dev);
		iio_device_free(iio_dev);
		hw->iio_devs[id] = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(st_ism330dhcx_fsm_sensor_list); i++) {
		id = st_ism330dhcx_fsm_sensor_list[i];
		iio_dev = hw->iio_devs[id];
		if (!iio_dev)
			continue;

		sensor_mlc = iio_priv(iio_dev);
		ret = st_ism330dhcx_mlc_fsm_enable_sensor(sensor_mlc,
							  false);
		if (ret < 0)
			break;

		iio_device_unregister(iio_dev);
		iio_device_free(iio_dev);
		hw->iio_devs[id] = NULL;
	}

	return st_ism330dhcx_mlc_purge_config(hw);
}

static ssize_t st_ism330dhcx_mlc_info(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	struct st_ism330dhcx_hw *hw = sensor->hw;

	return sysfs_emit(buf, "mlc %02x fsm %02x\n",
			  hw->mlc_config->mlc_configured,
			  hw->mlc_config->fsm_configured);
}

static ssize_t
st_ism330dhcx_mlc_get_version(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return sysfs_emit(buf, "mlc loader Version %s\n",
			  ST_ISM330DHCX_MLC_LOADER_VERSION);
}

static ssize_t
st_ism330dhcx_mlc_upload_firmware(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	int err;

	err = request_firmware_nowait(THIS_MODULE, true,
				      ST_ISM330DHCX_MLC_FIRMWARE_NAME,
				      dev, GFP_KERNEL,
				      sensor->hw,
				      st_ism330dhcx_mlc_update);

	return err < 0 ? err : size;
}

static ssize_t st_ism330dhcx_mlc_flush(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	struct st_ism330dhcx_hw *hw = sensor->hw;
	int ret;

	ret = st_ism330dhcx_mlc_flush_all(hw);

	hw->mlc_config->status = 0;
	hw->mlc_config->fsm_configured = 0;
	hw->mlc_config->mlc_configured = 0;
	hw->mlc_config->mlc_fsm_en = 0;
	hw->mlc_config->bin_len = 0;
	hw->mlc_config->fsm_mlc_requested_odr = 0;
	hw->mlc_config->fsm_mlc_requested_uodr = 0;

	return ret < 0 ? ret : size;
}

static IIO_DEVICE_ATTR(mlc_info, 0444,
		       st_ism330dhcx_mlc_info, NULL, 0);
static IIO_DEVICE_ATTR(mlc_flush, 0200,
		       NULL, st_ism330dhcx_mlc_flush, 0);
static IIO_DEVICE_ATTR(mlc_version, 0444,
		       st_ism330dhcx_mlc_get_version, NULL, 0);
static IIO_DEVICE_ATTR(load_mlc, 0200,
		       NULL, st_ism330dhcx_mlc_upload_firmware, 0);
static IIO_DEVICE_ATTR(module_id, 0444, st_ism330dhcx_get_module_id, NULL, 0);

static struct attribute *st_ism330dhcx_mlc_event_attributes[] = {
	&iio_dev_attr_mlc_info.dev_attr.attr,
	&iio_dev_attr_mlc_version.dev_attr.attr,
	&iio_dev_attr_load_mlc.dev_attr.attr,
	&iio_dev_attr_mlc_flush.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_ism330dhcx_mlc_event_attribute_group = {
	.attrs = st_ism330dhcx_mlc_event_attributes,
};

static const struct iio_info st_ism330dhcx_mlc_event_info = {
	.attrs = &st_ism330dhcx_mlc_event_attribute_group,
	.read_event_config = st_ism330dhcx_mlc_read_event_config,
	.write_event_config = st_ism330dhcx_mlc_write_event_config,
};

static struct attribute *st_ism330dhcx_mlc_x_event_attributes[] = {
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_ism330dhcx_mlc_x_attribute_group = {
	.attrs = st_ism330dhcx_mlc_x_event_attributes,
};

static const struct iio_info st_ism330dhcx_mlc_x_event_info = {
	.attrs = &st_ism330dhcx_mlc_x_attribute_group,
	.read_event_config = st_ism330dhcx_mlc_read_event_config,
	.write_event_config = st_ism330dhcx_mlc_write_event_config,
};

static struct iio_dev *
st_ism330dhcx_mlc_alloc_iio_dev(struct st_ism330dhcx_hw *hw,
				enum st_ism330dhcx_sensor_id id)
{
	struct st_ism330dhcx_sensor *sensor;
	struct iio_dev *iio_dev = NULL;

	/* devm management only for ST_ISM330DHCX_ID_MLC */
	if (id == ST_ISM330DHCX_ID_MLC) {
		iio_dev = devm_iio_device_alloc(hw->dev, sizeof(*sensor));
	} else {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0)
		iio_dev = iio_device_alloc(NULL, sizeof(*sensor));
#else /* LINUX_VERSION_CODE */
		iio_dev = iio_device_alloc(sizeof(*sensor));
#endif /* LINUX_VERSION_CODE */

	}

	if (!iio_dev)
		return NULL;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = hw->dev;

	sensor = iio_priv(iio_dev);
	sensor->id = id;
	sensor->hw = hw;

	switch (id) {
	case ST_ISM330DHCX_ID_MLC:
		iio_dev->available_scan_masks =
			st_ism330dhcx_fsm_mlc_available_scan_masks;
		iio_dev->channels = st_ism330dhcx_mlc_fsm_x_ch;
		iio_dev->num_channels = ARRAY_SIZE(st_ism330dhcx_mlc_fsm_x_ch);
		iio_dev->info = &st_ism330dhcx_mlc_event_info;
		scnprintf(sensor->name, sizeof(sensor->name), "ism330dhcx_mlc");
		break;
	case ST_ISM330DHCX_ID_MLC_0:
	case ST_ISM330DHCX_ID_MLC_1:
	case ST_ISM330DHCX_ID_MLC_2:
	case ST_ISM330DHCX_ID_MLC_3:
	case ST_ISM330DHCX_ID_MLC_4:
	case ST_ISM330DHCX_ID_MLC_5:
	case ST_ISM330DHCX_ID_MLC_6:
	case ST_ISM330DHCX_ID_MLC_7:
		iio_dev->available_scan_masks =
			st_ism330dhcx_fsm_mlc_available_scan_masks;
		iio_dev->channels = st_ism330dhcx_mlc_fsm_x_ch;
		iio_dev->num_channels = ARRAY_SIZE(st_ism330dhcx_mlc_fsm_x_ch);
		iio_dev->info = &st_ism330dhcx_mlc_x_event_info;
		scnprintf(sensor->name, sizeof(sensor->name),
			  "ism330dhcx_mlc_%d", id - ST_ISM330DHCX_ID_MLC_0);
		sensor->outreg_addr = ST_ISM330DHCX_REG_MLC0_SRC_ADDR +
				id - ST_ISM330DHCX_ID_MLC_0;
		sensor->status = ST_ISM330DHCX_MLC_ENABLED;
		sensor->odr = hw->mlc_config->fsm_mlc_requested_odr;
		sensor->uodr = hw->mlc_config->fsm_mlc_requested_uodr;
		break;
	case ST_ISM330DHCX_ID_FSM_0:
	case ST_ISM330DHCX_ID_FSM_1:
	case ST_ISM330DHCX_ID_FSM_2:
	case ST_ISM330DHCX_ID_FSM_3:
	case ST_ISM330DHCX_ID_FSM_4:
	case ST_ISM330DHCX_ID_FSM_5:
	case ST_ISM330DHCX_ID_FSM_6:
	case ST_ISM330DHCX_ID_FSM_7:
	case ST_ISM330DHCX_ID_FSM_8:
	case ST_ISM330DHCX_ID_FSM_9:
	case ST_ISM330DHCX_ID_FSM_10:
	case ST_ISM330DHCX_ID_FSM_11:
	case ST_ISM330DHCX_ID_FSM_12:
	case ST_ISM330DHCX_ID_FSM_13:
	case ST_ISM330DHCX_ID_FSM_14:
	case ST_ISM330DHCX_ID_FSM_15:
		iio_dev->available_scan_masks =
			st_ism330dhcx_fsm_mlc_available_scan_masks;
		iio_dev->channels = st_ism330dhcx_mlc_fsm_x_ch;
		iio_dev->num_channels = ARRAY_SIZE(st_ism330dhcx_mlc_fsm_x_ch);
		iio_dev->info = &st_ism330dhcx_mlc_x_event_info;
		scnprintf(sensor->name, sizeof(sensor->name),
			  "ism330dhcx_fsm_%d", id - ST_ISM330DHCX_ID_FSM_0);
		sensor->outreg_addr = ST_ISM330DHCX_FSM_OUTS1_ADDR +
				id - ST_ISM330DHCX_ID_FSM_0;
		sensor->status = ST_ISM330DHCX_FSM_ENABLED;
		sensor->odr = hw->mlc_config->fsm_mlc_requested_odr;
		sensor->uodr = hw->mlc_config->fsm_mlc_requested_uodr;
		break;
	default:
		dev_err(hw->dev, "invalid sensor id %d\n", id);
		iio_device_free(iio_dev);

		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

/*
 * st_ism330dhcx_mlc_check_status - check for mlc/fsm events
 *
 * return: MLC/FSM index, < 0 for error
 */
int st_ism330dhcx_mlc_check_status(struct st_ism330dhcx_hw *hw)
{
	struct st_ism330dhcx_sensor *sensor;
	struct iio_dev *iio_dev;
	__le16 __fsm_status = 0;
	int ret = 0, notify = 0;
	u8 i, mlc_status, id;
	u16 fsm_status;

	if (hw->mlc_config->status & ST_ISM330DHCX_MLC_ENABLED) {
		ret = st_ism330dhcx_read_atomic(hw,
					ST_ISM330DHCX_MLC_STATUS_MAINPAGE,
					sizeof(mlc_status), (void *)&mlc_status);
		if (ret)
			return ret;

		if (mlc_status) {
			u8 mlc_event[ST_ISM330DHCX_MLC_NUMBER];

			for (i = 0; i < ST_ISM330DHCX_MLC_NUMBER; i++) {
				id = st_ism330dhcx_mlc_sensor_list[i];
				if (!(hw->enable_mask & BIT_ULL(id)))
					continue;

				if (mlc_status & BIT(i)) {
					iio_dev = hw->iio_devs[id];
					if (!iio_dev) {
						ret = -ENOENT;

						return ret;
					}

					sensor = iio_priv(iio_dev);
					ret = st_ism330dhcx_read_atomic(hw,
						sensor->outreg_addr,
						1, (void *)&mlc_event[i]);
					if (ret)
						return ret;

					iio_push_event(iio_dev, (u64)mlc_event[i],
						       iio_get_time_ns(iio_dev));

					dev_info(hw->dev,
						 "MLC %d Status %x MLC EVENT %llx\n",
						 id, mlc_status,
						 (u64)mlc_event[i]);

					notify |= BIT(i);
				}
			}
		}
	}

	if (hw->mlc_config->status & ST_ISM330DHCX_FSM_ENABLED) {
		ret = st_ism330dhcx_read_atomic(hw,
					ST_ISM330DHCX_FSM_STATUS_A_MAINPAGE,
					2, (void *)&__fsm_status);
		if (ret)
			return ret;

		fsm_status = le16_to_cpu(__fsm_status);
		if (fsm_status) {
			u8 fsm_event[ST_ISM330DHCX_FSM_NUMBER];

			for (i = 0; i < ST_ISM330DHCX_FSM_NUMBER; i++) {
				id = st_ism330dhcx_fsm_sensor_list[i];
				if (!(hw->enable_mask & BIT_ULL(id)))
					continue;

				if (fsm_status & BIT(i)) {
					iio_dev = hw->iio_devs[id];
					if (!iio_dev) {
						ret = -ENOENT;

						return ret;
					}

					sensor = iio_priv(iio_dev);
					ret = st_ism330dhcx_read_atomic(hw,
						sensor->outreg_addr,
						1, (void *)&fsm_event[i]);
					if (ret)
						return ret;

					iio_push_event(iio_dev, (u64)fsm_event[i],
						       iio_get_time_ns(iio_dev));

					dev_info(hw->dev,
						 "FSM %d Status %x FSM EVENT %llx\n",
						 id, fsm_status,
						 (u64)fsm_event[i]);

					notify |= BIT(i + ST_ISM330DHCX_MLC_NUMBER);
				}
			}
		}
	}

	return ret < 0 ? ret : notify;
}

/**
 * st_ism330dhcx_of_get_mlc_int_pin - Read of configuration of mlc int
 *
 * @hw: Sensor hw structure.
 * @pin: Interrupt pin used by MLC.
 *
 * Possible configurations are:
 * st,mlc-int-pin = <1>; int1 pin will be used by MLC
 * st,mlc-int-pin = <2>; int2 pin will be used by MLC
 * st,mlc-int-pin = <3>; both interrupt pins will be used by MLC
 */
static int st_ism330dhcx_of_get_mlc_int_pin(struct st_ism330dhcx_hw *hw,
					    int *pin)
{
	struct device_node *np = hw->dev->of_node;
	int int_pin;
	int ret;

	if (!np)
		return -EINVAL;

	ret = of_property_read_u32(np, "st,mlc-int-pin", &int_pin);
	if (ret < 0) {
		dev_info(hw->dev,
			 "missing mlc-int-pin, using default (%d)\n",
			 hw->int_pin);
		int_pin = hw->int_pin;
	}

	if (!(int_pin & 0x03)) {
		dev_err(hw->dev,
			"invalid mlc interrupt configuration (%d)\n",
			int_pin);

		return -EINVAL;
	}

	*pin = int_pin;

	return 0;
}

int st_ism330dhcx_mlc_probe(struct st_ism330dhcx_hw *hw)
{
	int int_pin;
	int ret;

	ret = st_ism330dhcx_of_get_mlc_int_pin(hw, &int_pin);
	if (ret)
		return -EINVAL;

	hw->iio_devs[ST_ISM330DHCX_ID_MLC] =
		st_ism330dhcx_mlc_alloc_iio_dev(hw,
						ST_ISM330DHCX_ID_MLC);
	if (!hw->iio_devs[ST_ISM330DHCX_ID_MLC])
		return -ENOMEM;

	hw->mlc_config = devm_kzalloc(hw->dev,
				      sizeof(struct st_ism330dhcx_mlc_config_t),
				      GFP_KERNEL);
	if (!hw->mlc_config)
		return -ENOMEM;

	hw->mlc_config->mlc_int_pin = int_pin;

	return 0;
}

int st_ism330dhcx_mlc_remove(struct device *dev)
{
	struct st_ism330dhcx_hw *hw = dev_get_drvdata(dev);

	return st_ism330dhcx_mlc_flush_all(hw);
}
EXPORT_SYMBOL(st_ism330dhcx_mlc_remove);

int st_ism330dhcx_mlc_init_preload(struct st_ism330dhcx_hw *hw)
{

#ifdef CONFIG_IIO_ST_ISM330DHCX_MLC_PRELOAD
	hw->preload_mlc = 1;
	st_ism330dhcx_mlc_update(&st_ism330dhcx_mlc_preload, hw);
#endif /* CONFIG_IIO_ST_ISM330DHCX_MLC_PRELOAD */

	return 0;
}
