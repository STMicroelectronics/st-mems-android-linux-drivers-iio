// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_iis2iclx sensor hub library driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <asm/unaligned.h>

#include "st_iis2iclx.h"

#define ST_IIS2ICLX_MAX_SLV_NUM			2

/**
 * @struct  st_iis2iclx_ext_pwr
 * @brief  External device Power Management description
 *
 * reg: Generic sensor register description.
 * off_val: Value to write into register to power off external sensor.
 * on_val: Value to write into register for power on external sensor.
 */
struct st_iis2iclx_ext_pwr {
	struct st_iis2iclx_reg reg;
	u8 off_val;
	u8 on_val;
};

/**
 * @struct  st_iis2iclx_ext_dev_settings
 * @brief  External sensor descritor entry
 *
 * i2c_addr: External I2C device address (max two).
 * wai_addr: Device ID address.
 * wai_val: Device ID value.
 * odr_table: ODR sensor table.
 * fs_table: Full scale table.
 * temp_comp_reg: Temperature compensation registers.
 * pwr_table: External device Power Management description.
 * off_canc_reg: Offset cancellation registers.
 * bdu_reg: Block Data Update registers.
 * ext_available_scan_masks: IIO device scan mask.
 * ext_channels:IIO device channel specifications.
 * ext_chan_depth: Max number of IIO device channel specifications.
 * data_len: Sensor output data len.
 */
struct st_iis2iclx_ext_dev_settings {
	u8 i2c_addr[2];
	u8 wai_addr;
	u8 wai_val;
	struct st_iis2iclx_odr_table_entry odr_table;
	struct st_iis2iclx_fs_table_entry fs_table;
	struct st_iis2iclx_reg temp_comp_reg;
	struct st_iis2iclx_ext_pwr pwr_table;
	struct st_iis2iclx_reg off_canc_reg;
	struct st_iis2iclx_reg bdu_reg;
	unsigned long ext_available_scan_masks[2];
	const struct iio_chan_spec ext_channels[5];
	u8 ext_chan_depth;
	u8 data_len;
};

static const struct
st_iis2iclx_ext_dev_settings st_iis2iclx_ext_dev_table[] = {
	{
		/* LIS2MDL */
		.i2c_addr = { 0x1e },
		.wai_addr = 0x4f,
		.wai_val = 0x40,
		.odr_table = {
			.size = 5,
			.reg = {
				.addr = 0x60,
				.mask = GENMASK(3, 2),
			},
			/*
			 * added 5Hz for CTS coverage, reg value
			 * is the same for 5 and 10 Hz
			 */
			.odr_avl[0] = {   5,  1,  0x0,  0 },
			.odr_avl[1] = {  10,  0,  0x0,  0 },
			.odr_avl[2] = {  20,  0,  0x1,  0 },
			.odr_avl[3] = {  50,  0,  0x2,  0 },
			.odr_avl[4] = { 100,  0,  0x3,  0 },
		},
		.fs_table = {
			.size = 1,
			.fs_avl[0] = {
				.gain = 1500,
				.val = 0x0,
			}, /* 1500 uG/LSB */
		},
		.temp_comp_reg = {
			.addr = 0x60,
			.mask = BIT(7),
		},
		.pwr_table = {
			.reg = {
				.addr = 0x60,
				.mask = GENMASK(1, 0),
			},
			.off_val = 0x2,
			.on_val = 0x0,
		},
		.off_canc_reg = {
			.addr = 0x61,
			.mask = BIT(1),
		},
		.bdu_reg = {
			.addr = 0x62,
			.mask = BIT(4),
		},
		.ext_available_scan_masks = { 0x7, 0x0 },
		.ext_channels[0] = ST_IIS2ICLX_DATA_CHANNEL(IIO_MAGN, 0x68,
					1, IIO_MOD_X, 0, 16, 16, 's', NULL),
		.ext_channels[1] = ST_IIS2ICLX_DATA_CHANNEL(IIO_MAGN, 0x6a,
					1, IIO_MOD_Y, 1, 16, 16, 's', NULL),
		.ext_channels[2] = ST_IIS2ICLX_DATA_CHANNEL(IIO_MAGN, 0x6c,
					1, IIO_MOD_Z, 2, 16, 16, 's', NULL),
		.ext_channels[3] = ST_IIS2ICLX_EVENT_CHANNEL(IIO_MAGN, flush),
		.ext_channels[4] = IIO_CHAN_SOFT_TIMESTAMP(3),
		.ext_chan_depth = 5,
		.data_len = 6,
	},
	{
		/* LIS3MDL */
		.i2c_addr = { 0x1c, 0x1e },
		.wai_addr = 0x0f,
		.wai_val = 0x3d,
		.odr_table = {
			.size = 5,
			.reg = {
				.addr = 0x20,
				.mask = GENMASK(4, 2),
			},
			.odr_avl[0] = {   5,  0,  0x3,  0 },
			.odr_avl[1] = {  10,  0,  0x3,  0 },
			.odr_avl[2] = {  20,  0,  0x4,  0 },
			.odr_avl[3] = {  40,  0,  0x5,  0 },
			.odr_avl[4] = {  80,  0,  0x6,  0 },
			.odr_avl[5] = { 100,  0,  0x7,  0 },
		},
		.fs_table = {
			.size = 4,
			.fs_avl[0] = {
				.reg = {
					.addr = 0x21,
					.mask = GENMASK(6, 5),
				},
				.gain = 6842,
				.val = 0x0,
			},
			.fs_avl[1] = {
				.reg = {
					.addr = 0x21,
					.mask = GENMASK(6, 5),
				},
				.gain = 3421,
				.val = 0x1,
			},
			.fs_avl[2] = {
				.reg = {
					.addr = 0x21,
					.mask = GENMASK(6, 5),
				},
				.gain = 2281,
				.val = 0x2,
			},
			.fs_avl[3] = {
				.reg = {
					.addr = 0x21,
					.mask = GENMASK(6, 5),
				},
				.gain = 1711,
				.val = 0x3,
			},
		},
		.temp_comp_reg = {
			.addr = 0x20,
			.mask = BIT(7),
		},
		.pwr_table = {
			.reg = {
				.addr = 0x22,
				.mask = GENMASK(1, 0),
			},
			.off_val = 0x2,
			.on_val = 0x0,
		},
		.bdu_reg = {
			.addr = 0x24,
			.mask = BIT(6),
		},
		.ext_available_scan_masks = { 0x7, 0x0 },
		.ext_channels[0] = ST_IIS2ICLX_DATA_CHANNEL(IIO_MAGN, 0x28,
					1, IIO_MOD_X, 0, 16, 16, 's', NULL),
		.ext_channels[1] = ST_IIS2ICLX_DATA_CHANNEL(IIO_MAGN, 0x2a,
					1, IIO_MOD_Y, 1, 16, 16, 's', NULL),
		.ext_channels[2] = ST_IIS2ICLX_DATA_CHANNEL(IIO_MAGN, 0x2c,
					1, IIO_MOD_Z, 2, 16, 16, 's', NULL),
		.ext_channels[3] = ST_IIS2ICLX_EVENT_CHANNEL(IIO_MAGN, flush),
		.ext_channels[4] = IIO_CHAN_SOFT_TIMESTAMP(3),
		.ext_chan_depth = 5,
		.data_len = 6,
	},
	{
		/* LPS22HB */
		.i2c_addr = { 0x5c, 0x5d },
		.wai_addr = 0x0f,
		.wai_val = 0xb1,
		.odr_table = {
			.size = 4,
			.reg = {
				.addr = 0x10,
				.mask = GENMASK(6, 4),
			},
			.odr_avl[0] = {  1,  0,  0x1,  0 },
			.odr_avl[1] = { 10,  0,  0x2,  0 },
			.odr_avl[2] = { 25,  0,  0x3,  0 },
			.odr_avl[3] = { 50,  0,  0x4,  0 },
		},
		.fs_table = {
			.size = 1,
			/* hPa miscro scale */
			.fs_avl[0] = {
				.gain = 1000000UL/4096UL,
				.val = 0x0,
			},
		},
		.bdu_reg = {
			.addr = 0x10,
			.mask = BIT(1),
		},
		.ext_available_scan_masks = { 0x1, 0x0 },
		.ext_channels[0] = ST_IIS2ICLX_DATA_CHANNEL(IIO_PRESSURE, 0x28,
					0, IIO_NO_MOD, 0, 24, 32, 'u', NULL),
		.ext_channels[1] = ST_IIS2ICLX_EVENT_CHANNEL(IIO_PRESSURE,
							     flush),
		.ext_channels[2] = IIO_CHAN_SOFT_TIMESTAMP(1),
		.ext_chan_depth = 3,
		.data_len = 3,
	},
	{
		/* LPS22HH */
		.i2c_addr = { 0x5c, 0x5d },
		.wai_addr = 0x0f,
		.wai_val = 0xb3,
		.odr_table = {
			.size = 5,
			.reg = {
				.addr = 0x10,
				.mask = GENMASK(6, 4),
			},
			.odr_avl[0] = {   1,  0,  0x1,  0 },
			.odr_avl[1] = {  10,  0,  0x2,  0 },
			.odr_avl[2] = {  25,  0,  0x3,  0 },
			.odr_avl[3] = {  50,  0,  0x4,  0 },
			.odr_avl[4] = { 100,  0,  0x6,  0 },
		},
		.fs_table = {
			.size = 1,
			/* hPa miscro scale */
			.fs_avl[0] = {
				.gain = 1000000UL/4096UL,
				.val = 0x0,
			},
		},
		.bdu_reg = {
			.addr = 0x10,
			.mask = BIT(1),
		},
		.ext_available_scan_masks = { 0x1, 0x0 },
		.ext_channels[0] = ST_IIS2ICLX_DATA_CHANNEL(IIO_PRESSURE, 0x28,
					0, IIO_NO_MOD, 0, 24, 32, 'u', NULL),
		.ext_channels[1] = ST_IIS2ICLX_EVENT_CHANNEL(IIO_PRESSURE,
							     flush),
		.ext_channels[2] = IIO_CHAN_SOFT_TIMESTAMP(1),
		.ext_chan_depth = 3,
		.data_len = 3,
	},
};

/**
 * Wait write trigger [SHUB]
 *
 * In write on external device register, each operation is triggered
 * by accel/gyro data ready, this means that wait time depends on ODR
 * plus i2c time
 * NOTE: Be sure to enable Acc or Gyro before this operation
 *
 * @param  hw: ST MEMS hw instance.
 */
static inline void
st_iis2iclx_shub_wait_complete(struct st_iis2iclx_hw *hw)
{
	struct st_iis2iclx_sensor *sensor;
	int odr, uodr;

	sensor = iio_priv(hw->iio_devs[ST_IIS2ICLX_ID_ACC]);

	/* check if acc is enabled (it should be) */
	if (hw->enable_mask & BIT_ULL(ST_IIS2ICLX_ID_ACC)) {
		odr = sensor->odr;
		uodr = sensor->uodr;
	} else {
		odr = 12;
		uodr = 500000;
	}

	msleep((2000000000U / (odr * 1000000 + uodr)) + 1);
}

/**
 * Read from sensor hub bank register [SHUB]
 *
 * NOTE: uses page_lock
 *
 * @param  hw: ST MEMS hw instance.
 * @param  addr: Remote address register.
 * @param  data: Data buffer.
 * @param  len: Data read len.
 * @return  0 if OK, < 0 if ERROR
 */
static int st_iis2iclx_shub_read_reg(struct st_iis2iclx_hw *hw,
				     u8 addr, u8 *data, int len)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = st_iis2iclx_set_page_access(hw, true, ST_IIS2ICLX_SHUB_REG_MASK);
	if (err < 0)
		goto out;

	err = regmap_bulk_read(hw->regmap, (unsigned int)addr,
			       (unsigned int *)data, len);
	st_iis2iclx_set_page_access(hw, false, ST_IIS2ICLX_SHUB_REG_MASK);
out:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Write to sensor hub bank register [SHUB]
 *
 * NOTE: uses page_lock
 *
 * @param  hw: ST MEMS hw instance.
 * @param  addr: Remote address register.
 * @param  data: Data buffer.
 * @param  len: Data read len.
 * @return  0 if OK, < 0 if ERROR
 */
static int st_iis2iclx_shub_write_reg(struct st_iis2iclx_hw *hw,
				      u8 addr, u8 *data, int len)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = st_iis2iclx_set_page_access(hw, true, ST_IIS2ICLX_SHUB_REG_MASK);
	if (err < 0)
		goto out;

	err = regmap_bulk_write(hw->regmap, (unsigned int)addr,
				(unsigned int *)data, len);
	st_iis2iclx_set_page_access(hw, false, ST_IIS2ICLX_SHUB_REG_MASK);
out:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Enable sensor hub interface [SHUB]
 *
 * NOTE: uses page_lock
 *
 * @param  sensor: ST sensor instance
 * @param  enable: Master Enable/Disable.
 * @return  0 if OK, < 0 if ERROR
 */
static int st_iis2iclx_shub_master_enable(struct st_iis2iclx_sensor *sensor,
					  bool enable)
{
	struct st_iis2iclx_hw *hw = sensor->hw;
	int err;

	/* enable acc sensor as trigger */
	err = st_iis2iclx_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_iis2iclx_set_page_access(hw, true, ST_IIS2ICLX_SHUB_REG_MASK);
	if (err < 0)
		goto out;

	err = __st_iis2iclx_write_with_mask(hw,
					    ST_IIS2ICLX_REG_MASTER_CONFIG_ADDR,
					    ST_IIS2ICLX_MASTER_ON_MASK,
					    enable);

	st_iis2iclx_set_page_access(hw, false, ST_IIS2ICLX_SHUB_REG_MASK);

out:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Read sensor data register from shub interface
 *
 * NOTE: use SLV3 i2c slave for one-shot read operation
 *
 * @param  sensor: ST sensor instance
 * @param  addr: Remote address register.
 * @param  data: Data buffer.
 * @param  len: Data read len.
 * @return  0 if OK, < 0 if ERROR
 */
static int st_iis2iclx_shub_read(struct st_iis2iclx_sensor *sensor,
				 u8 addr, u8 *data, int len)
{
	struct st_iis2iclx_ext_dev_info *ext_info = &sensor->ext_dev_info;
	struct st_iis2iclx_hw *hw = sensor->hw;
	u8 out_addr = ST_IIS2ICLX_REG_SLV0_OUT_ADDR + hw->ext_data_len;
	u8 config[3];
	int err;

	config[0] = (ext_info->ext_dev_i2c_addr << 1) | 1;
	config[1] = addr;
	config[2] = len & 0x7;

	err = st_iis2iclx_shub_write_reg(hw, ST_IIS2ICLX_REG_SLV3_ADDR,
					 config, sizeof(config));
	if (err < 0)
		return err;

	err = st_iis2iclx_shub_master_enable(sensor, true);
	if (err < 0)
		return err;

	st_iis2iclx_shub_wait_complete(hw);

	err = st_iis2iclx_shub_read_reg(hw, out_addr, data, len & 0x7);

	st_iis2iclx_shub_master_enable(sensor, false);

	memset(config, 0, sizeof(config));

	return st_iis2iclx_shub_write_reg(hw, ST_IIS2ICLX_REG_SLV3_ADDR,
					  config, sizeof(config));
}

/**
 * Write sensor data register from shub interface
 *
 * NOTE: use SLV0 i2c slave for write operation
 *
 * @param  sensor: ST sensor instance
 * @param  addr: Remote address register.
 * @param  data: Data buffer.
 * @param  len: Data read len.
 * @return  0 if OK, < 0 if ERROR
 */
static int st_iis2iclx_shub_write(struct st_iis2iclx_sensor *sensor,
				  u8 addr, u8 *data, int len)
{
	struct st_iis2iclx_ext_dev_info *ext_info = &sensor->ext_dev_info;
	struct st_iis2iclx_hw *hw = sensor->hw;
	u8 mconfig = ST_IIS2ICLX_WRITE_ONCE_MASK | 3 | hw->i2c_master_pu;
	u8 config[3] = {};
	int err, i;

	/* AuxSens = 3 + wr once + pull up configuration */
	err = st_iis2iclx_shub_write_reg(hw, ST_IIS2ICLX_REG_MASTER_CONFIG_ADDR,
					 &mconfig, sizeof(mconfig));
	if (err < 0)
		return err;

	config[0] = ext_info->ext_dev_i2c_addr << 1;
	for (i = 0; i < len; i++) {
		config[1] = addr + i;

		err = st_iis2iclx_shub_write_reg(hw, ST_IIS2ICLX_REG_SLV0_ADDR,
						 config, sizeof(config));
		if (err < 0)
			return err;

		err = st_iis2iclx_shub_write_reg(hw,
					ST_IIS2ICLX_REG_DATAWRITE_SLV0_ADDR,
					&data[i], 1);
		if (err < 0)
			return err;

		err = st_iis2iclx_shub_master_enable(sensor, true);
		if (err < 0)
			return err;

		st_iis2iclx_shub_wait_complete(hw);

		st_iis2iclx_shub_master_enable(sensor, false);
	}

	return st_iis2iclx_shub_write_reg(hw, ST_IIS2ICLX_REG_SLV0_ADDR,
					  config, sizeof(config));
}

/**
 * Write sensor data register from shub interface using register bitmask
 *
 * @param  sensor: ST sensor instance
 * @param  addr: Remote address register.
 * @param  mask: Register bitmask.
 * @param  val: Data buffer.
 * @return  0 if OK, < 0 if ERROR
 */
static int
st_iis2iclx_shub_write_with_mask(struct st_iis2iclx_sensor *sensor,
				 u8 addr, u8 mask, u8 val)
{
	int err;
	u8 data;

	err = st_iis2iclx_shub_read(sensor, addr, &data, sizeof(data));
	if (err < 0)
		return err;

	data = (data & ~mask) | ST_IIS2ICLX_SHIFT_VAL(val, mask);

	return st_iis2iclx_shub_write(sensor, addr, &data, sizeof(data));
}

/**
 * Configure external sensor connected on master I2C interface
 *
 * NOTE: use SLV1/SLV2 i2c slave for FIFO read operation
 *
 * @param  sensor: ST sensor instance
 * @param  enable: Enable/Disable sensor.
 * @return  0 if OK, < 0 if ERROR
 */
static int
st_iis2iclx_shub_config_channels(struct st_iis2iclx_sensor *sensor, bool enable)
{
	struct st_iis2iclx_ext_dev_info *ext_info;
	struct st_iis2iclx_hw *hw = sensor->hw;
	struct st_iis2iclx_sensor *cur_sensor;
	u8 config[6] = {}, enable_mask;
	int i, j = 0;

	enable_mask = enable ? hw->enable_mask | BIT_ULL(sensor->id)
			     : hw->enable_mask & ~BIT_ULL(sensor->id);

	for (i = ST_IIS2ICLX_ID_EXT0; i <= ST_IIS2ICLX_ID_EXT1; i++) {
		if (!hw->iio_devs[i])
			continue;

		cur_sensor = iio_priv(hw->iio_devs[i]);
		if (!(enable_mask & BIT_ULL(cur_sensor->id)))
			continue;

		ext_info = &cur_sensor->ext_dev_info;
		config[j] = (ext_info->ext_dev_i2c_addr << 1) | 1;
		config[j + 1] =
		    ext_info->ext_dev_settings->ext_channels[0].address;
		config[j + 2] = ST_IIS2ICLX_BATCH_EXT_SENS_EN_MASK |
				(ext_info->ext_dev_settings->data_len &
				 ST_IIS2ICLX_SLAVE_NUMOP_MASK);
		j += 3;
	}

	return st_iis2iclx_shub_write_reg(hw, ST_IIS2ICLX_REG_SLV1_ADDR,
					  config, sizeof(config));
}

/**
 * Get a valid ODR [SHUB]
 *
 * Check a valid ODR closest to the passed value
 *
 * @param  sensor: ST sensor instance.
 * @param  odr: ODR value (in Hz).
 * @param  val: ODR register value data pointer.
 * @return  0 if OK, negative value for ERROR
 */
static int
st_iis2iclx_shub_get_odr_val(struct st_iis2iclx_sensor *sensor,
			     u16 odr, u8 *val)
{
	struct st_iis2iclx_ext_dev_info *ext_info = &sensor->ext_dev_info;
	int i;

	for (i = 0; i < ext_info->ext_dev_settings->odr_table.size; i++)
		if (ext_info->ext_dev_settings->odr_table.odr_avl[i].hz >= odr)
			break;

	if (i == ext_info->ext_dev_settings->odr_table.size)
		return -EINVAL;

	*val = ext_info->ext_dev_settings->odr_table.odr_avl[i].val;

	/* set decimator for low ODR */
	sensor->decimator =
		ext_info->ext_dev_settings->odr_table.odr_avl[i].uhz;
	sensor->dec_counter = 0;

	return 0;
}

/**
 * Set new ODR to sensor [SHUB]
 *
 * Set a valid ODR closest to the passed value
 *
 * @param  sensor: ST sensor instance
 * @param  odr: ODR value (in Hz).
 * @return  0 if OK, negative value for ERROR
 */
static int
st_iis2iclx_shub_set_odr(struct st_iis2iclx_sensor *sensor, u16 odr)
{
	struct st_iis2iclx_ext_dev_info *ext_info = &sensor->ext_dev_info;
	struct st_iis2iclx_hw *hw = sensor->hw;
	u8 odr_val;
	int err;

	err = st_iis2iclx_shub_get_odr_val(sensor, odr, &odr_val);
	if (err < 0)
		return err;

	if (sensor->odr == odr && (hw->enable_mask & BIT_ULL(sensor->id)))
		return 0;

	return st_iis2iclx_shub_write_with_mask(sensor,
				 ext_info->ext_dev_settings->odr_table.reg.addr,
				 ext_info->ext_dev_settings->odr_table.reg.mask,
				 odr_val);
}

/**
 * Enable or Disable sensor [SHUB]
 *
 * @param  sensor: ST sensor instance
 * @param  enable: Enable or disable the sensor [true,false].
 * @return  0 if OK, negative value for ERROR
 */
int st_iis2iclx_shub_set_enable(struct st_iis2iclx_sensor *sensor, bool enable)
{
	struct st_iis2iclx_ext_dev_info *ext_info = &sensor->ext_dev_info;
	int err;

	err = st_iis2iclx_shub_config_channels(sensor, enable);
	if (err < 0)
		return err;

	if (enable) {
		err = st_iis2iclx_shub_set_odr(sensor, sensor->odr);
		if (err < 0)
			return err;
	} else {
		err = st_iis2iclx_shub_write_with_mask(sensor,
				 ext_info->ext_dev_settings->odr_table.reg.addr,
				 ext_info->ext_dev_settings->odr_table.reg.mask,
				 0);
		if (err < 0)
			return err;
	}

	if (ext_info->ext_dev_settings->pwr_table.reg.addr) {
		u8 val;

		val = enable ? ext_info->ext_dev_settings->pwr_table.on_val :
			       ext_info->ext_dev_settings->pwr_table.off_val;
		err = st_iis2iclx_shub_write_with_mask(sensor,
			 ext_info->ext_dev_settings->pwr_table.reg.addr,
			 ext_info->ext_dev_settings->pwr_table.reg.mask,
			 val);
		if (err < 0)
			return err;
	}

	return st_iis2iclx_shub_master_enable(sensor, enable);
}

static inline u32 st_iis2iclx_get_unaligned_le24(const u8 *p)
{
	return (s32)((p[0] | p[1] << 8 | p[2] << 16) << 8) >> 8;
}

/**
 * Single sensor read operation [SHUB]
 *
 * @param  sensor: ST sensor instance
 * @param  ch: IIO Channel.
 * @param  val: Output data register value.
 * @return  IIO_VAL_INT if OK, negative value for ERROR
 */
static int
st_iis2iclx_shub_read_oneshot(struct st_iis2iclx_sensor *sensor,
			      struct iio_chan_spec const *ch, int *val)
{
	int err, delay, len = ch->scan_type.realbits >> 3;
	u8 data[4];

	if (len > ARRAY_SIZE(data))
		return -ENOMEM;

	err = st_iis2iclx_shub_set_enable(sensor, true);
	if (err < 0)
		return err;

	delay = 1000000 / sensor->odr;
	usleep_range(delay, 2 * delay);

	err = st_iis2iclx_shub_read(sensor, ch->address, data, len);
	if (err < 0)
		return err;

	st_iis2iclx_shub_set_enable(sensor, false);

	switch (len) {
	case 3:
		*val = (s32)st_iis2iclx_get_unaligned_le24(data);
		break;
	case 2:
		*val = (s16)get_unaligned_le16(data);
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

/**
 * Read Sensor data configuration [SHUB]
 *
 * @param  iio_dev: IIO Device.
 * @param  ch: IIO Channel.
 * @param  val: Data Buffer (MSB).
 * @param  val2: Data Buffer (LSB).
 * @param  mask: Data Mask.
 * @return  0 if OK, -EINVAL value for ERROR
 */
static int st_iis2iclx_shub_read_raw(struct iio_dev *iio_dev,
				     struct iio_chan_spec const *ch,
				     int *val, int *val2, long mask)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			return ret;

		ret = st_iis2iclx_shub_read_oneshot(sensor, ch, val);
		iio_device_release_direct_mode(iio_dev);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = sensor->odr;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sensor->gain;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * Write Sensor data configuration [SHUB]
 *
 * @param  iio_dev: IIO Device.
 * @param  chan: IIO Channel.
 * @param  val: Data Buffer (MSB).
 * @param  val2: Data Buffer (LSB).
 * @param  mask: Data Mask.
 * @return  0 if OK, -EINVAL value for ERROR
 */
static int
st_iis2iclx_shub_write_raw(struct iio_dev *iio_dev,
			   struct iio_chan_spec const *chan,
			   int val, int val2, long mask)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	int err;

	mutex_lock(&iio_dev->mlock);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ: {
		u8 data;

		err = st_iis2iclx_shub_get_odr_val(sensor, val, &data);
		if (!err)
			sensor->odr = val;
		break;
	}
	case IIO_CHAN_INFO_SCALE:
		err = 0;
		break;
	default:
		err = -EINVAL;
		break;
	}

	mutex_unlock(&iio_dev->mlock);

	return err;
}

/**
 * Get a list of available sensor ODR [SHUB]
 *
 * List of available ODR returned separated by commas
 *
 * @param  dev: IIO Device.
 * @param  attr: IIO Channel attribute.
 * @param  buf: User buffer.
 * @return  buffer len
 */
static ssize_t
st_iis2iclx_sysfs_shub_sampling_freq_avail(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	struct st_iis2iclx_ext_dev_info *ext_info = &sensor->ext_dev_info;
	int i, len = 0;

	for (i = 0; i < ext_info->ext_dev_settings->odr_table.size; i++) {
		u16 val = ext_info->ext_dev_settings->odr_table.odr_avl[i].hz;

		if (val > 0)
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "%d ", val);
	}
	buf[len - 1] = '\n';

	return len;
}

/**
 * Get a list of available sensor Full Scale [SHUB]
 *
 * List of available Full Scale returned separated by commas
 *
 * @param  dev: IIO Device.
 * @param  attr: IIO Channel attribute.
 * @param  buf: User buffer.
 * @return  buffer len
 */
static ssize_t
st_iis2iclx_sysfs_shub_scale_avail(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	struct st_iis2iclx_ext_dev_info *ext_info = &sensor->ext_dev_info;
	int i, len = 0;

	for (i = 0; i < ext_info->ext_dev_settings->fs_table.size; i++) {
		u16 val = ext_info->ext_dev_settings->fs_table.fs_avl[i].gain;

		if (val > 0)
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "0.%06u ", val);
	}
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_iis2iclx_sysfs_shub_sampling_freq_avail);
static IIO_DEVICE_ATTR(in_ext_scale_available, 0444,
		       st_iis2iclx_sysfs_shub_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_iis2iclx_get_max_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, 0200, NULL,
		       st_iis2iclx_flush_fifo, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644,
		       st_iis2iclx_get_watermark,
		       st_iis2iclx_set_watermark, 0);
static IIO_DEVICE_ATTR(module_id, 0444, st_iis2iclx_get_module_id, NULL, 0);

static struct attribute *st_iis2iclx_ext_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_ext_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_iis2iclx_ext_attribute_group = {
	.attrs = st_iis2iclx_ext_attributes,
};

static const struct iio_info st_iis2iclx_ext_info = {
	.attrs = &st_iis2iclx_ext_attribute_group,
	.read_raw = st_iis2iclx_shub_read_raw,
	.write_raw = st_iis2iclx_shub_write_raw,
};

/**
 * Allocate IIO device [SHUB]
 *
 * @param  hw: ST MEMS hw instance.
 * @param  ext_settings: xternal sensor descritor entry.
 * @param  id: Sensor Identifier.
 * @param  i2c_addr: external I2C address on master bus.
 * @return  struct iio_dev *, NULL if ERROR
 */
static struct iio_dev *
st_iis2iclx_shub_alloc_iio_dev(struct st_iis2iclx_hw *hw,
			const struct st_iis2iclx_ext_dev_settings *ext_settings,
			enum st_iis2iclx_sensor_id id, u8 i2c_addr)
{
	struct st_iis2iclx_sensor *sensor;
	struct iio_dev *iio_dev;

	iio_dev = devm_iio_device_alloc(hw->dev, sizeof(*sensor));
	if (!iio_dev)
		return NULL;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = hw->dev;
	iio_dev->available_scan_masks = ext_settings->ext_available_scan_masks;
	iio_dev->info = &st_iis2iclx_ext_info;
	iio_dev->channels = ext_settings->ext_channels;
	iio_dev->num_channels = ext_settings->ext_chan_depth;

	sensor = iio_priv(iio_dev);
	sensor->id = id;
	sensor->hw = hw;
	sensor->odr = ext_settings->odr_table.odr_avl[0].hz;
	sensor->gain = ext_settings->fs_table.fs_avl[0].gain;
	sensor->max_watermark = ST_IIS2ICLX_MAX_FIFO_DEPTH;
	sensor->watermark = 1;
	sensor->ext_dev_info.ext_dev_i2c_addr = i2c_addr;
	sensor->ext_dev_info.ext_dev_settings = ext_settings;
	sensor->decimator = 0;
	sensor->dec_counter = 0;

	switch (iio_dev->channels[0].type) {
	case IIO_MAGN:
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_magn", ST_IIS2ICLX_DEV_NAME);
		break;
	case IIO_PRESSURE:
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_press", ST_IIS2ICLX_DEV_NAME);
		break;
	default:
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_ext", ST_IIS2ICLX_DEV_NAME);
		break;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

static int
st_iis2iclx_shub_init_remote_sensor(struct st_iis2iclx_sensor *sensor)
{
	struct st_iis2iclx_ext_dev_info *ext_info = &sensor->ext_dev_info;
	int err = 0;

	if (ext_info->ext_dev_settings->bdu_reg.addr)
		err = st_iis2iclx_shub_write_with_mask(sensor,
				ext_info->ext_dev_settings->bdu_reg.addr,
				ext_info->ext_dev_settings->bdu_reg.mask, 1);

	if (ext_info->ext_dev_settings->temp_comp_reg.addr)
		err = st_iis2iclx_shub_write_with_mask(sensor,
				ext_info->ext_dev_settings->temp_comp_reg.addr,
				ext_info->ext_dev_settings->temp_comp_reg.mask,
				1);

	if (ext_info->ext_dev_settings->off_canc_reg.addr)
		err = st_iis2iclx_shub_write_with_mask(sensor,
				ext_info->ext_dev_settings->off_canc_reg.addr,
				ext_info->ext_dev_settings->off_canc_reg.mask,
				1);

	return err;
}

/**
 * Probe device function [SHUB]
 *
 * @param  hw: ST MEMS hw instance.
 * @return  0 if OK, negative for ERROR
 */
int st_iis2iclx_shub_probe(struct st_iis2iclx_hw *hw)
{
	const struct st_iis2iclx_ext_dev_settings *settings;
	struct st_iis2iclx_sensor *acc_sensor, *sensor;
	struct device_node *np = hw->dev->of_node;
	u8 config[3], data, num_ext_dev = 0;
	enum st_iis2iclx_sensor_id id;
	int err, i = 0, j;

	if (np && of_property_read_bool(np, "drive-pullup-shub")) {
		dev_info(hw->dev, "enabling pull up on i2c master\n");
		err = st_iis2iclx_shub_read_reg(hw,
					ST_IIS2ICLX_REG_MASTER_CONFIG_ADDR,
					&data, sizeof(data));
		if (err < 0)
			return err;

		data |= ST_IIS2ICLX_SHUB_PU_EN_MASK;
		err = st_iis2iclx_shub_write_reg(hw,
					ST_IIS2ICLX_REG_MASTER_CONFIG_ADDR,
					&data, sizeof(data));

		if (err < 0)
			return err;

		hw->i2c_master_pu = ST_IIS2ICLX_SHUB_PU_EN_MASK;
	}

	acc_sensor = iio_priv(hw->iio_devs[ST_IIS2ICLX_ID_ACC]);
	while (i < ARRAY_SIZE(st_iis2iclx_ext_dev_table) &&
	       num_ext_dev < ST_IIS2ICLX_MAX_SLV_NUM) {
		settings = &st_iis2iclx_ext_dev_table[i];

		for (j = 0; j < ARRAY_SIZE(settings->i2c_addr); j++) {
			if (!settings->i2c_addr[j])
				continue;

			/* read wai slave register */
			config[0] = (settings->i2c_addr[j] << 1) | 1;
			config[1] = settings->wai_addr;
			config[2] = 1;

			err = st_iis2iclx_shub_write_reg(hw,
						ST_IIS2ICLX_REG_SLV0_ADDR,
						config, sizeof(config));
			if (err < 0)
				return err;

			err = st_iis2iclx_shub_master_enable(acc_sensor, true);
			if (err < 0)
				return err;

			st_iis2iclx_shub_wait_complete(hw);

			err = st_iis2iclx_shub_read_reg(hw,
						ST_IIS2ICLX_REG_SLV0_OUT_ADDR,
						&data, sizeof(data));

			st_iis2iclx_shub_master_enable(acc_sensor, false);

			if (err < 0)
				return err;

			if (data != settings->wai_val)
				continue;

			id = ST_IIS2ICLX_ID_EXT0 + num_ext_dev;
			hw->iio_devs[id] = st_iis2iclx_shub_alloc_iio_dev(hw,
							settings, id,
							settings->i2c_addr[j]);
			if (!hw->iio_devs[id])
				return -ENOMEM;

			sensor = iio_priv(hw->iio_devs[id]);
			err = st_iis2iclx_shub_init_remote_sensor(sensor);
			if (err < 0)
				return err;

			num_ext_dev++;
			hw->ext_data_len += settings->data_len;
			break;
		}

		i++;
	}

	if (!num_ext_dev)
		return 0;

	memset(config, 0, sizeof(config));
	err = st_iis2iclx_shub_write_reg(hw, ST_IIS2ICLX_REG_SLV0_ADDR,
					 config, sizeof(config));
	if (err < 0)
		return err;

	/* AuxSens = 3 + wr once */
	data = ST_IIS2ICLX_WRITE_ONCE_MASK | 3 | hw->i2c_master_pu;
	return st_iis2iclx_shub_write_reg(hw,
					  ST_IIS2ICLX_REG_MASTER_CONFIG_ADDR,
					  &data, sizeof(data));
}
