// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_ism330dhcx sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2020 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/property.h>

#include <linux/platform_data/st_sensors_pdata.h>

#include "st_ism330dhcx.h"
static int __maybe_unused st_ism330dhcx_restore_regs(struct st_ism330dhcx_hw *hw);
static int __maybe_unused st_ism330dhcx_bk_regs(struct st_ism330dhcx_hw *hw);

static struct st_ism330dhcx_selftest_table {
	char *string_mode;
	u8 accel_value;
	u8 gyro_value;
	u8 gyro_mask;
} st_ism330dhcx_selftest_table[] = {
	[0] = {
		.string_mode = "disabled",
		.accel_value = ST_ISM330DHCX_SELF_TEST_DISABLED_VAL,
		.gyro_value = ST_ISM330DHCX_SELF_TEST_DISABLED_VAL,
	},
	[1] = {
		.string_mode = "positive-sign",
		.accel_value = ST_ISM330DHCX_SELF_TEST_POS_SIGN_VAL,
		.gyro_value = ST_ISM330DHCX_SELF_TEST_POS_SIGN_VAL
	},
	[2] = {
		.string_mode = "negative-sign",
		.accel_value = ST_ISM330DHCX_SELF_TEST_NEG_ACCEL_SIGN_VAL,
		.gyro_value = ST_ISM330DHCX_SELF_TEST_NEG_GYRO_SIGN_VAL
	},
};

static struct st_ism330dhcx_suspend_resume_entry
	st_ism330dhcx_suspend_resume[ST_ISM330DHCX_SUSPEND_RESUME_REGS] = {
		[ST_ISM330DHCX_CTRL1_XL_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_CTRL1_XL_ADDR,
			.mask = GENMASK(3, 2),
		},
		[ST_ISM330DHCX_CTRL2_G_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_CTRL2_G_ADDR,
			.mask = GENMASK(3, 2),
		},
		[ST_ISM330DHCX_REG_CTRL3_C_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_CTRL3_C_ADDR,
			.mask = ST_ISM330DHCX_REG_BDU_MASK       |
				ST_ISM330DHCX_REG_PP_OD_MASK     |
				ST_ISM330DHCX_REG_H_LACTIVE_MASK,
		},
		[ST_ISM330DHCX_REG_CTRL4_C_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_CTRL4_C_ADDR,
			.mask = ST_ISM330DHCX_REG_DRDY_MASK,
		},
		[ST_ISM330DHCX_REG_CTRL5_C_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_CTRL5_C_ADDR,
			.mask = ST_ISM330DHCX_REG_ROUNDING_MASK,
		},
		[ST_ISM330DHCX_REG_CTRL10_C_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_CTRL10_C_ADDR,
			.mask = ST_ISM330DHCX_REG_TIMESTAMP_EN_MASK,
		},
		[ST_ISM330DHCX_REG_TAP_CFG0_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_TAP_CFG0_ADDR,
			.mask = ST_ISM330DHCX_REG_LIR_MASK |
				ST_ISM330DHCX_TAP_EN_MASK |
				ST_ISM330DHCX_REG_INT_CLR_ON_READ_MASK,
		},
		[ST_ISM330DHCX_REG_TAP_CFG1_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_TAP_CFG1_ADDR,
			.mask = ST_ISM330DHCX_TAP_THS_X_MASK |
				ST_ISM330DHCX_TAP_PRIORITY_MASK,
		},
		[ST_ISM330DHCX_REG_TAP_CFG2_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_TAP_CFG2_ADDR,
			.mask = ST_ISM330DHCX_TAP_THS_Y_MASK |
				ST_ISM330DHCX_INTERRUPTS_ENABLE_MASK,
		},
		[ST_ISM330DHCX_REG_TAP_THS_6D_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_TAP_THS_6D_ADDR,
			.mask = ST_ISM330DHCX_TAP_THS_Z_MASK |
				ST_ISM330DHCX_SIXD_THS_MASK,
		},
		[ST_ISM330DHCX_REG_INT_DUR2_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_INT_DUR2_ADDR,
			.mask = ST_ISM330DHCX_SHOCK_MASK |
				ST_ISM330DHCX_QUIET_MASK |
				ST_ISM330DHCX_DUR_MASK,
		},
		[ST_ISM330DHCX_REG_WAKE_UP_THS_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_WAKE_UP_THS_ADDR,
			.mask = ST_ISM330DHCX_WAKE_UP_THS_MASK |
				ST_ISM330DHCX_SINGLE_DOUBLE_TAP_MASK,
		},
		[ST_ISM330DHCX_REG_WAKE_UP_DUR_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_WAKE_UP_DUR_ADDR,
			.mask = ST_ISM330DHCX_WAKE_UP_DUR_MASK,
		},
		[ST_ISM330DHCX_REG_FREE_FALL_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_FREE_FALL_ADDR,
			.mask = GENMASK(7, 0),
		},
		[ST_ISM330DHCX_REG_INT1_CTRL_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_INT1_CTRL_ADDR,
			.mask = ST_ISM330DHCX_REG_INT_FIFO_TH_MASK,
		},
		[ST_ISM330DHCX_REG_INT2_CTRL_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_INT2_CTRL_ADDR,
			.mask = ST_ISM330DHCX_REG_INT_FIFO_TH_MASK,
		},
		[ST_ISM330DHCX_REG_FIFO_CTRL1_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_FIFO_CTRL1_ADDR,
			.mask = GENMASK(7, 0),
		},
		[ST_ISM330DHCX_REG_FIFO_CTRL2_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_FIFO_CTRL2_ADDR,
			.mask = ST_ISM330DHCX_REG_FIFO_WTM8_MASK,
		},
		[ST_ISM330DHCX_REG_FIFO_CTRL3_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_FIFO_CTRL3_ADDR,
			.mask = ST_ISM330DHCX_REG_BDR_XL_MASK |
				ST_ISM330DHCX_REG_BDR_GY_MASK,
		},
		[ST_ISM330DHCX_REG_FIFO_CTRL4_REG] = {
			.page = FUNC_CFG_ACCESS_0,
			.addr = ST_ISM330DHCX_REG_FIFO_CTRL4_ADDR,
			.mask = ST_ISM330DHCX_REG_DEC_TS_MASK |
				ST_ISM330DHCX_REG_ODR_T_BATCH_MASK,
		},
		[ST_ISM330DHCX_REG_EMB_FUNC_EN_A_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_EMB_FUNC_EN_A_ADDR,
			.mask = ST_ISM330DHCX_PEDO_EN_MASK |
				ST_ISM330DHCX_TILT_EN_MASK |
				ST_ISM330DHCX_SIGN_MOTION_EN_MASK,
		},
		[ST_ISM330DHCX_REG_EMB_FUNC_EN_B_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
			.mask = ST_ISM330DHCX_FSM_EN_MASK |
				ST_ISM330DHCX_MLC_EN_MASK,
		},
		[ST_ISM330DHCX_REG_EMB_FUNC_FIFO_CFG_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_EMB_FUNC_FIFO_CFG_ADDR,
			.mask = ST_ISM330DHCX_PEDO_FIFO_EN_MASK,
		},
		[ST_ISM330DHCX_REG_PAGE_RW_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_PAGE_RW_ADDR,
			.mask = ST_ISM330DHCX_REG_EMB_FUNC_LIR_MASK,
		},


		[ST_ISM330DHCX_REG_FSM_INT1_A_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_FSM_INT1_A_ADDR,
			.mask = GENMASK(7, 0),
		},
		[ST_ISM330DHCX_REG_FSM_INT1_B_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_FSM_INT1_B_ADDR,
			.mask = GENMASK(7, 0),
		},
		[ST_ISM330DHCX_REG_MLC_INT1_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_MLC_INT1_ADDR,
			.mask = GENMASK(7, 0),
		},


		[ST_ISM330DHCX_REG_FSM_INT2_A_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_FSM_INT2_A_ADDR,
			.mask = GENMASK(7, 0),
		},
		[ST_ISM330DHCX_REG_FSM_INT2_B_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_FSM_INT2_B_ADDR,
			.mask = GENMASK(7, 0),
		},
		[ST_ISM330DHCX_REG_MLC_INT2_REG] = {
			.page = FUNC_CFG_ACCESS_FUNC_CFG,
			.addr = ST_ISM330DHCX_MLC_INT2_ADDR,
			.mask = GENMASK(7, 0),
		},
	};

/**
 * List of supported ODR
 *
 * The following table is complete list of supported ODR by Acc, Gyro and Temp
 * sensors. ODR value can be also decimal (i.e 12.5 Hz)
 */
static const struct st_ism330dhcx_odr_table_entry st_ism330dhcx_odr_table[] = {
	[ST_ISM330DHCX_ID_ACC] = {
		.size = 8,
		.reg = {
			.addr = ST_ISM330DHCX_CTRL1_XL_ADDR,
			.mask = GENMASK(7, 4),
		},
		.batching_reg = {
			.addr = ST_ISM330DHCX_REG_FIFO_CTRL3_ADDR,
			.mask = ST_ISM330DHCX_REG_BDR_XL_MASK,
		},
		.odr_avl[0] = {   0, 0,       0x00, 0x00 },
		.odr_avl[1] = {  12, 500000,  0x01, 0x01 },
		.odr_avl[2] = {  26, 0,       0x02, 0x02 },
		.odr_avl[3] = {  52, 0,       0x03, 0x03 },
		.odr_avl[4] = { 104, 0,       0x04, 0x04 },
		.odr_avl[5] = { 208, 0,       0x05, 0x05 },
		.odr_avl[6] = { 416, 0,       0x06, 0x06 },
		.odr_avl[7] = { 833, 0,       0x07, 0x07 },
	},
	[ST_ISM330DHCX_ID_GYRO] = {
		.size = 8,
		.reg = {
			.addr = ST_ISM330DHCX_CTRL2_G_ADDR,
			.mask = GENMASK(7, 4),
		},
		.batching_reg = {
			.addr = ST_ISM330DHCX_REG_FIFO_CTRL3_ADDR,
			.mask = ST_ISM330DHCX_REG_BDR_GY_MASK,
		},
		.odr_avl[0] = {   0, 0,       0x00, 0x00 },
		.odr_avl[1] = {  12, 500000,  0x01, 0x01 },
		.odr_avl[2] = {  26, 0,       0x02, 0x02 },
		.odr_avl[3] = {  52, 0,       0x03, 0x03 },
		.odr_avl[4] = { 104, 0,       0x04, 0x04 },
		.odr_avl[5] = { 208, 0,       0x05, 0x05 },
		.odr_avl[6] = { 416, 0,       0x06, 0x06 },
		.odr_avl[7] = { 833, 0,       0x07, 0x07 },
	},
	[ST_ISM330DHCX_ID_TEMP] = {
		.size = 3,
		.batching_reg = {
			.addr = ST_ISM330DHCX_REG_FIFO_CTRL4_ADDR,
			.mask = ST_ISM330DHCX_REG_ODR_T_BATCH_MASK,
		},
		.odr_avl[0] = {  0,      0,   0x00,  0x00 },
		.odr_avl[1] = { 12, 500000,   0x02,  0x02 },
		.odr_avl[2] = { 52,      0,   0x03,  0x03 },
	},
};

/**
 * List of supported Full Scale Value
 *
 * The following table is complete list of supported Full Scale by Acc, Gyro
 * and Temp sensors.
 */
static const struct st_ism330dhcx_fs_table_entry st_ism330dhcx_fs_table[] = {
	[ST_ISM330DHCX_ID_ACC] = {
		.reg = {
			.addr = ST_ISM330DHCX_CTRL1_XL_ADDR,
			.mask = GENMASK(3, 2),
		},
		.fs_avl[0] = { ST_ISM330DHCX_ACC_FS_2G_GAIN,  0x00 },
		.fs_avl[1] = { ST_ISM330DHCX_ACC_FS_4G_GAIN,  0x02 },
		.fs_avl[2] = { ST_ISM330DHCX_ACC_FS_8G_GAIN,  0x03 },
		.fs_avl[3] = { ST_ISM330DHCX_ACC_FS_16G_GAIN, 0x01 },
		.size = ST_ISM330DHCX_FS_ACC_LIST_SIZE,
	},
	[ST_ISM330DHCX_ID_GYRO] = {
		.reg = {
			.addr = ST_ISM330DHCX_CTRL2_G_ADDR,
			.mask = GENMASK(3, 0),
		},
		.fs_avl[0] = { ST_ISM330DHCX_GYRO_FS_250_GAIN,  0x00 },
		.fs_avl[1] = { ST_ISM330DHCX_GYRO_FS_500_GAIN,  0x04 },
		.fs_avl[2] = { ST_ISM330DHCX_GYRO_FS_1000_GAIN, 0x08 },
		.fs_avl[3] = { ST_ISM330DHCX_GYRO_FS_2000_GAIN, 0x0C },
		.fs_avl[4] = { ST_ISM330DHCX_GYRO_FS_4000_GAIN, 0x01 },
		.size = ST_ISM330DHCX_FS_GYRO_LIST_SIZE,
	},
	[ST_ISM330DHCX_ID_TEMP] = {
		.reg = { 0 },
		.fs_avl[0] = { ST_ISM330DHCX_TEMP_FS_GAIN, 0x00 },
		.size = ST_ISM330DHCX_FS_TEMP_LIST_SIZE,
	},
};

#define IIO_CHAN_HW_TIMESTAMP(si) {					\
	.type = IIO_COUNT,						\
	.address = ST_ISM330DHCX_REG_TIMESTAMP0_ADDR,			\
	.scan_index = si,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 64,						\
		.storagebits = 64,					\
		.endianness = IIO_LE,					\
	},								\
}

/**
 * Accelerometer IIO channels description
 *
 * Accelerometer exports to IIO framework the following data channels:
 * X Axis (16 bit signed in little endian)
 * Y Axis (16 bit signed in little endian)
 * Z Axis (16 bit signed in little endian)
 * Timestamp (64 bit signed in little endian)
 * Accelerometer exports to IIO framework the following event channels:
 * Flush event done
 */
static const struct iio_chan_spec st_ism330dhcx_acc_channels[] = {
	ST_ISM330DHCX_DATA_CHANNEL(IIO_ACCEL, ST_ISM330DHCX_REG_OUTX_L_A_ADDR,
				1, IIO_MOD_X, 0, 16, 16, 's'),
	ST_ISM330DHCX_DATA_CHANNEL(IIO_ACCEL, ST_ISM330DHCX_REG_OUTY_L_A_ADDR,
				1, IIO_MOD_Y, 1, 16, 16, 's'),
	ST_ISM330DHCX_DATA_CHANNEL(IIO_ACCEL, ST_ISM330DHCX_REG_OUTZ_L_A_ADDR,
				1, IIO_MOD_Z, 2, 16, 16, 's'),
	ST_ISM330DHCX_EVENT_CHANNEL(IIO_ACCEL, flush),

	ST_ISM330DHCX_EVENT_CHANNEL(IIO_ACCEL, freefall),
	ST_ISM330DHCX_EVENT_CHANNEL(IIO_ACCEL, wakeup),
	ST_ISM330DHCX_EVENT_CHANNEL(IIO_ACCEL, 6D),

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ST_ISM330DHCX_EVENT_CHANNEL(IIO_ACCEL, tap),
	ST_ISM330DHCX_EVENT_CHANNEL(IIO_ACCEL, dtap),
#endif /* LINUX_VERSION_CODE */

#if defined(CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP)
	IIO_CHAN_HW_TIMESTAMP(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
#else /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */
	IIO_CHAN_SOFT_TIMESTAMP(3),
#endif /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */

};

/**
 * Gyro IIO channels description
 *
 * Gyro exports to IIO framework the following data channels:
 * X Axis (16 bit signed in little endian)
 * Y Axis (16 bit signed in little endian)
 * Z Axis (16 bit signed in little endian)
 * Timestamp (64 bit signed in little endian)
 * Gyro exports to IIO framework the following event channels:
 * Flush event done
 */
static const struct iio_chan_spec st_ism330dhcx_gyro_channels[] = {
	ST_ISM330DHCX_DATA_CHANNEL(IIO_ANGL_VEL, ST_ISM330DHCX_REG_OUTX_L_G_ADDR,
				1, IIO_MOD_X, 0, 16, 16, 's'),
	ST_ISM330DHCX_DATA_CHANNEL(IIO_ANGL_VEL, ST_ISM330DHCX_REG_OUTY_L_G_ADDR,
				1, IIO_MOD_Y, 1, 16, 16, 's'),
	ST_ISM330DHCX_DATA_CHANNEL(IIO_ANGL_VEL, ST_ISM330DHCX_REG_OUTZ_L_G_ADDR,
				1, IIO_MOD_Z, 2, 16, 16, 's'),
	ST_ISM330DHCX_EVENT_CHANNEL(IIO_ANGL_VEL, flush),

#if defined(CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP)
	IIO_CHAN_HW_TIMESTAMP(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
#else /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */
	IIO_CHAN_SOFT_TIMESTAMP(3),
#endif /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */

};

/**
 * Temperature IIO channels description
 *
 * Temperature exports to IIO framework the following data channels:
 * Temperature (16 bit signed in little endian)
 * Temperature exports to IIO framework the following event channels:
 * Temperature event threshold
 */
static const struct iio_chan_spec st_ism330dhcx_temp_channels[] = {
	{
		.type = IIO_TEMP,
		.address = ST_ISM330DHCX_REG_OUT_TEMP_L_ADDR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_OFFSET) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		}
	},
	ST_ISM330DHCX_EVENT_CHANNEL(IIO_TEMP, flush),

#if defined(CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP)
	IIO_CHAN_HW_TIMESTAMP(1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
#else /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */
	IIO_CHAN_SOFT_TIMESTAMP(1),
#endif /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */

};

/**
 * Detect device ID
 *
 * Check the value of the Device ID if valid
 *
 * @param  hw: ST IMU MEMS hw instance
 * @return  0 if OK, negative value for ERROR
 */
static int st_ism330dhcx_check_whoami(struct st_ism330dhcx_hw *hw)
{
	int err;
	int data;

	err = regmap_read(hw->regmap, ST_ISM330DHCX_REG_WHOAMI_ADDR, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != ST_ISM330DHCX_WHOAMI_VAL) {
		dev_err(hw->dev, "unsupported whoami [%02x]\n", data);
		return -ENODEV;
	}

	return 0;
}

/**
 * Get timestamp calibration
 *
 * Read timestamp calibration data and trim delta time
 *
 * @param  hw: ST IMU MEMS hw instance
 * @return  0 if OK, negative value for ERROR
 */
static int st_ism330dhcx_get_odr_calibration(struct st_ism330dhcx_hw *hw)
{
	s64 odr_calib;
	int data;
	int err;

	err = regmap_read(hw->regmap, ST_ISM330DHCX_INTERNAL_FREQ_FINE, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read %d register\n",
				ST_ISM330DHCX_INTERNAL_FREQ_FINE);
		return err;
	}

	odr_calib = ((s8)data * 37500) / 1000;
	hw->ts_delta_ns = ST_ISM330DHCX_TS_DELTA_NS - odr_calib;

	dev_info(hw->dev, "Freq Fine %lld (ts %lld)\n",
		 odr_calib, hw->ts_delta_ns);

	return 0;
}

/**
 * Set sensor Full Scale
 *
 * Set new Full Scale value for a specific sensor
 *
 * @param  sensor: ST IMU sensor instance
 * @param  gain: New gain value
 * @return  0 if OK, negative value for ERROR
 */
static int st_ism330dhcx_set_full_scale(struct st_ism330dhcx_sensor *sensor, u32 gain)
{
	enum st_ism330dhcx_sensor_id id = sensor->id;
	int i, err;
	u8 val;

	for (i = 0; i < st_ism330dhcx_fs_table[id].size; i++)
		if (st_ism330dhcx_fs_table[id].fs_avl[i].gain >= gain)
			break;

	if (i == st_ism330dhcx_fs_table[id].size)
		return -EINVAL;

	val = st_ism330dhcx_fs_table[id].fs_avl[i].val;
	err = st_ism330dhcx_write_with_mask(sensor->hw,
				st_ism330dhcx_fs_table[id].reg.addr,
				st_ism330dhcx_fs_table[id].reg.mask,
				val);
	if (err < 0)
		return err;

	sensor->gain = gain;

	return 0;
}

/**
 * Get a valid ODR
 *
 * Check a valid ODR closest to the passed value
 *
 * @param  id: Sensor Identifier
 * @param  odr: Most significant part of ODR value (in Hz).
 * @param  uodr: Least significant part of ODR value (in micro Hz).
 * @param  podr: User data pointer.
 * @param  puodr: User data pointer.
 * @param  val: ODR register value data pointer.
 * @return  0 if OK, negative value for ERROR
 */
int st_ism330dhcx_get_odr_val(enum st_ism330dhcx_sensor_id id, int odr,
			      int uodr, int *podr, int *puodr, u8 *val)
{
	int i;
	int sensor_odr;
	int all_odr = ST_ISM330DHCX_ODR_EXPAND(odr, uodr);

	if (all_odr == 0) {
		*val = 0;
		if (podr && puodr) {
			*podr = 0;
			*puodr = 0;
		}

		return 0;
	}

	for (i = 0; i < st_ism330dhcx_odr_table[id].size; i++) {
		sensor_odr =
		   ST_ISM330DHCX_ODR_EXPAND(st_ism330dhcx_odr_table[id].odr_avl[i].hz,
		   st_ism330dhcx_odr_table[id].odr_avl[i].uhz);
		if (sensor_odr >= all_odr)
			break;
	}

	if (i == st_ism330dhcx_odr_table[id].size)
		return -EINVAL;

	*val = st_ism330dhcx_odr_table[id].odr_avl[i].val;
	*podr = st_ism330dhcx_odr_table[id].odr_avl[i].hz;
	*puodr = st_ism330dhcx_odr_table[id].odr_avl[i].uhz;

	return 0;
}

int __maybe_unused
st_ism330dhcx_get_odr_from_reg(enum st_ism330dhcx_sensor_id id,
			       u8 reg_val, u16 *podr, u32 *puodr)
{
	int i;

	for (i = 0; i < st_ism330dhcx_odr_table[id].size; i++) {
		if (reg_val == st_ism330dhcx_odr_table[id].odr_avl[i].val)
			break;
	}

	if (i == st_ism330dhcx_odr_table[id].size)
		return -EINVAL;

	*podr = st_ism330dhcx_odr_table[id].odr_avl[i].hz;
	*puodr = st_ism330dhcx_odr_table[id].odr_avl[i].uhz;

	return 0;
}

int st_ism330dhcx_get_batch_val(struct st_ism330dhcx_sensor *sensor,
				int odr, int uodr, u8 *val)
{
	int required_odr = ST_ISM330DHCX_ODR_EXPAND(odr, uodr);
	enum st_ism330dhcx_sensor_id id = sensor->id;
	int sensor_odr;
	int i;

	for (i = 0; i < st_ism330dhcx_odr_table[id].size; i++) {
		sensor_odr = ST_ISM330DHCX_ODR_EXPAND(
				st_ism330dhcx_odr_table[id].odr_avl[i].hz,
				st_ism330dhcx_odr_table[id].odr_avl[i].uhz);
		if (sensor_odr >= required_odr)
			break;
	}

	if (i == st_ism330dhcx_odr_table[id].size)
		return -EINVAL;

	*val = st_ism330dhcx_odr_table[id].odr_avl[i].batch_val;

	return 0;
}

static int st_ism330dhcx_update_odr_fsm(struct st_ism330dhcx_hw *hw,
					enum st_ism330dhcx_sensor_id id,
					enum st_ism330dhcx_sensor_id id_req,
					int val, int delay)
{
	bool fsm_running = st_ism330dhcx_fsm_running(hw);
	bool mlc_running = st_ism330dhcx_mlc_running(hw);
	int ret = 0;
	int status;

	if (fsm_running || mlc_running || (id_req > ST_ISM330DHCX_ID_MLC)) {
		/*
		 * In STMC_PAGE:
		 * Addr 0x02 bit 1 set to 1 -- CLK Disable
		 * Addr 0x05 bit 0 set to 0 -- FSM_EN=0
		 * Addr 0x05 bit 4 set to 0 -- MLC_EN=0
		 * Addr 0x67 bit 0 set to 0 -- FSM_INIT=0
		 * Addr 0x67 bit 4 set to 0 -- MLC_INIT=0
		 * Addr 0x02 bit 1 set to 0 -- CLK Disable
		 * - ODR change
		 * - Wait (~3 ODRs)
		 * In STMC_PAGE:
		 * Addr 0x05 bit 0 set to 1 -- FSM_EN = 1
		 * Addr 0x05 bit 4 set to 1 -- MLC_EN = 1
		 */
		mutex_lock(&hw->page_lock);
		ret = st_ism330dhcx_set_page_access(hw,
					ST_ISM330DHCX_REG_FUNC_CFG_MASK, true);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_read(hw->regmap, ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
				  &status);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 ST_ISM330DHCX_PAGE_SEL_ADDR,
					 BIT(1), FIELD_PREP(BIT(1), 1));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
					 ST_ISM330DHCX_FSM_EN_MASK,
					 FIELD_PREP(ST_ISM330DHCX_FSM_EN_MASK,
						    0));
		if (ret < 0)
			goto unlock_page;

		if (st_ism330dhcx_mlc_running(hw)) {
			ret = regmap_update_bits(hw->regmap,
					ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
					ST_ISM330DHCX_MLC_EN_MASK,
					FIELD_PREP(ST_ISM330DHCX_MLC_EN_MASK,
						   0));
			if (ret < 0)
				goto unlock_page;
		}

		ret = regmap_update_bits(hw->regmap,
					 ST_ISM330DHCX_REG_EMB_FUNC_INIT_B_ADDR,
					 ST_ISM330DHCX_MLC_INIT_MASK,
					 FIELD_PREP(ST_ISM330DHCX_MLC_INIT_MASK,
						    0));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 ST_ISM330DHCX_REG_EMB_FUNC_INIT_B_ADDR,
					 ST_ISM330DHCX_FSM_INIT_MASK,
					 FIELD_PREP(ST_ISM330DHCX_FSM_INIT_MASK,
						    0));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 ST_ISM330DHCX_PAGE_SEL_ADDR,
					 BIT(1), FIELD_PREP(BIT(1), 0));
		if (ret < 0)
			goto unlock_page;

		ret = st_ism330dhcx_set_page_access(hw,
					ST_ISM330DHCX_REG_FUNC_CFG_MASK, false);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 st_ism330dhcx_odr_table[id].reg.addr,
					 st_ism330dhcx_odr_table[id].reg.mask,
					 ST_ISM330DHCX_SHIFT_VAL(val,
					 st_ism330dhcx_odr_table[id].reg.mask));
		if (ret < 0)
			goto unlock_page;

		usleep_range(delay, delay + (delay / 10));

		st_ism330dhcx_set_page_access(hw,
					ST_ISM330DHCX_REG_FUNC_CFG_MASK, true);

		ret = regmap_write(hw->regmap, ST_ISM330DHCX_EMB_FUNC_EN_B_ADDR,
				   status);
unlock_page:
		st_ism330dhcx_set_page_access(hw,
					      ST_ISM330DHCX_REG_FUNC_CFG_MASK, false);
		mutex_unlock(&hw->page_lock);
	} else {
		ret = st_ism330dhcx_write_with_mask(hw,
					st_ism330dhcx_odr_table[id].reg.addr,
					st_ism330dhcx_odr_table[id].reg.mask,
					val);
	}

	return ret;
}

/**
 * Set new ODR to sensor
 * Set a valid ODR closest to the passed value
 *
 * @param  sensor: ST IMU sensor instance
 * @param  req_odr: Most significant part of ODR value (in Hz).
 * @param  req_uodr: Least significant part of ODR value (in micro Hz).
 * @return  0 if OK, negative value for ERROR
 */
int st_ism330dhcx_set_odr(struct st_ism330dhcx_sensor *sensor, int req_odr,
			  int req_uodr)
{
	enum st_ism330dhcx_sensor_id id_req = sensor->id;
	enum st_ism330dhcx_sensor_id id = sensor->id;
	struct st_ism330dhcx_hw *hw = sensor->hw;
	int err, delay;
	u8 val = 0;

	switch (id) {
	case ST_ISM330DHCX_ID_TEMP:
	case ST_ISM330DHCX_ID_EXT0:
	case ST_ISM330DHCX_ID_EXT1:
	case ST_ISM330DHCX_ID_MLC_0:
	case ST_ISM330DHCX_ID_MLC_1:
	case ST_ISM330DHCX_ID_MLC_2:
	case ST_ISM330DHCX_ID_MLC_3:
	case ST_ISM330DHCX_ID_MLC_4:
	case ST_ISM330DHCX_ID_MLC_5:
	case ST_ISM330DHCX_ID_MLC_6:
	case ST_ISM330DHCX_ID_MLC_7:
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
	case ST_ISM330DHCX_ID_STEP_COUNTER:
	case ST_ISM330DHCX_ID_ACC: {
		int odr;
		int i;

		id = ST_ISM330DHCX_ID_ACC;
		for (i = ST_ISM330DHCX_ID_ACC; i < ST_ISM330DHCX_ID_MAX; i++) {
			if (!hw->iio_devs[i])
				continue;

			if (i == sensor->id)
				continue;

			/* req_uodr not used */
			odr = st_ism330dhcx_check_odr_dependency(hw, req_odr,
								 req_uodr, i);
			if (odr != req_odr)
				/* device already configured */
				return 0;
		}
		break;
	}
	case ST_ISM330DHCX_ID_GYRO:
		break;
	default:
		return 0;
	}

	err = st_ism330dhcx_get_odr_val(id, req_odr, req_uodr, &req_odr,
					&req_uodr, &val);
	if (err < 0)
		return err;

	delay = 4000000 / req_odr;

	return st_ism330dhcx_update_odr_fsm(hw, id, id_req, val, delay);
}

/**
 * Enable or Disable sensor
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable or disable the sensor [true,false].
 * @return  0 if OK, negative value for ERROR
 */
int st_ism330dhcx_sensor_set_enable(struct st_ism330dhcx_sensor *sensor,
				    bool enable)
{
	int uodr = 0;
	int odr = 0;
	int err;

	if (enable) {
		odr = sensor->odr;
		uodr = sensor->uodr;
	}

	err = st_ism330dhcx_set_odr(sensor, odr, uodr);
	if (err < 0)
		return err;

	if (enable)
		sensor->hw->enable_mask |= BIT_ULL(sensor->id);
	else
		sensor->hw->enable_mask &= ~BIT_ULL(sensor->id);

	return 0;
}

/**
 * Single sensor read operation
 *
 * @param  sensor: ST IMU sensor instance
 * @param  addr: Output data register value.
 * @param  val: Output data buffer.
 * @return  IIO_VAL_INT if OK, negative value for ERROR
 */
static int st_ism330dhcx_read_oneshot(struct st_ism330dhcx_sensor *sensor,
				      u8 addr, int *val)
{
	int err, delay;
	__le16 data;

	if (sensor->id > ST_ISM330DHCX_ID_HW)
		return -ENODEV;

	err = st_ism330dhcx_sensor_set_enable(sensor, true);
	if (err < 0)
		return err;

	if (sensor->id == ST_ISM330DHCX_ID_GYRO) {
		/*
		 * for gyro the time to wait before read for having stable
		 * output is about 70 ms + see Table 17 or Table 18 of AN5398
		 */
		delay = 70000 + 3 * (1000000 / sensor->odr);
		usleep_range(delay, delay + 1000);

		err = st_ism330dhcx_read_atomic(sensor->hw, addr, sizeof(data),
						(u8 *)&data);
		if (err < 0)
			return err;
	} else if (sensor->id == ST_ISM330DHCX_ID_ACC) {
		/*
		 * for accel the time to wait before read for having stable
		 * output is about 2 odr
		 */
		delay = 2 * (1000000 / sensor->odr);
		usleep_range(delay, delay + 1000);

		err = st_ism330dhcx_read_atomic(sensor->hw, addr, sizeof(data),
						(u8 *)&data);
		if (err < 0)
			return err;
	} else {
		/*
		 * for temperature sensor the time to wait before read for
		 * having stable output is about 1 odr
		 */
		delay = 1000000 / sensor->odr;
		usleep_range(delay, delay + 1000);

		err = st_ism330dhcx_read_atomic(sensor->hw, addr, sizeof(data),
						(u8 *)&data);
		if (err < 0)
			return err;
	}

	st_ism330dhcx_sensor_set_enable(sensor, false);

	*val = (s16)le16_to_cpu(data);

	return IIO_VAL_INT;
}

/**
 * Read Sensor data configuration
 *
 * @param  iio_dev: IIO Device.
 * @param  ch: IIO Channel.
 * @param  val: Data Buffer (MSB).
 * @param  val2: Data Buffer (LSB).
 * @param  mask: Data Mask.
 * @return  0 if OK, -EINVAL value for ERROR
 */
static int st_ism330dhcx_read_raw(struct iio_dev *iio_dev,
			       struct iio_chan_spec const *ch,
			       int *val, int *val2, long mask)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			return ret;

		ret = st_ism330dhcx_read_oneshot(sensor, ch->address, val);
		iio_device_release_direct_mode(iio_dev);
		break;
	case IIO_CHAN_INFO_OFFSET:
		switch (ch->type) {
		case IIO_TEMP:
			*val = sensor->offset;
			ret = IIO_VAL_INT;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = (int)sensor->odr;
		*val2 = (int)sensor->uodr;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (ch->type) {
		case IIO_TEMP:
			*val = 1000;
			*val2 = ST_ISM330DHCX_TEMP_GAIN;
			ret = IIO_VAL_FRACTIONAL;
			break;
		case IIO_ACCEL:
		case IIO_ANGL_VEL: {
			*val = 0;
			*val2 = sensor->gain;
			ret = IIO_VAL_INT_PLUS_NANO;
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * Write Sensor data configuration
 *
 * @param  iio_dev: IIO Device.
 * @param  chan: IIO Channel.
 * @param  val: Data Buffer (MSB).
 * @param  val2: Data Buffer (LSB).
 * @param  mask: Data Mask.
 * @return  0 if OK, -EINVAL value for ERROR
 */
static int st_ism330dhcx_write_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	int err;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = st_ism330dhcx_set_full_scale(sensor, val2);
		if (err)
			goto release_iio;

		/* some events depends on xl full scale */
		if (chan->type == IIO_ACCEL)
			err = st_ism330dhcx_update_threshold_events(sensor->hw);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		u8 data;
		int todr, tuodr;

		err = st_ism330dhcx_get_odr_val(sensor->id, val, val2, &todr,
						&tuodr, &data);
		if (!err) {
			sensor->odr = todr;
			sensor->uodr = tuodr;
		}

		/*
		 * VTS test testSamplingRateHotSwitchOperation not toggle the
		 * enable status of sensor after changing the ODR -> force it
		 */
		if (sensor->hw->enable_mask & BIT_ULL(sensor->id)) {
			switch(sensor->id) {
			case ST_ISM330DHCX_ID_GYRO:
			case ST_ISM330DHCX_ID_ACC:
				err = st_ism330dhcx_set_odr(sensor, sensor->odr,
							 sensor->uodr);
				/* I2C interface err can be positive */
				if (err < 0)
					break;

				err = st_ism330dhcx_update_batching(iio_dev, 1);

				/* some events depends on xl odr */
				if (chan->type == IIO_ACCEL)
					st_ism330dhcx_update_duration_events(sensor->hw);
				break;
			default:
				break;
			}
		}
		break;
	}
	default:
		err = -EINVAL;
		break;
	}

release_iio:
	iio_device_release_direct_mode(iio_dev);

	return err;
}

#ifdef CONFIG_DEBUG_FS
static int st_ism330dhcx_reg_access(struct iio_dev *iio_dev, unsigned int reg,
				 unsigned int writeval, unsigned int *readval)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(iio_dev);
	if (ret)
		return ret;

	if (readval == NULL)
		ret = regmap_write(sensor->hw->regmap, reg, writeval);
	else
		ret = regmap_read(sensor->hw->regmap, reg, readval);

	iio_device_release_direct_mode(iio_dev);

	return (ret < 0) ? ret : 0;
}
#endif /* CONFIG_DEBUG_FS */

/**
 * Get a list of available sensor ODR
 *
 * List of available ODR returned separated by commas
 *
 * @param  dev: IIO Device.
 * @param  attr: IIO Channel attribute.
 * @param  buf: User buffer.
 * @return  buffer len
 */
static ssize_t
st_ism330dhcx_sysfs_sampling_frequency_avail(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	enum st_ism330dhcx_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < ST_ISM330DHCX_ODR_LIST_SIZE; i++) {
		if (!st_ism330dhcx_odr_table[id].odr_avl[i].hz)
			continue;

		if (st_ism330dhcx_odr_table[id].odr_avl[i].uhz == 0)
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				st_ism330dhcx_odr_table[id].odr_avl[i].hz);
		else
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%d ",
				st_ism330dhcx_odr_table[id].odr_avl[i].hz,
				st_ism330dhcx_odr_table[id].odr_avl[i].uhz);
	}

	buf[len - 1] = '\n';

	return len;
}

/**
 * Get a list of available sensor Full Scale
 *
 * List of available Full Scale returned separated by commas
 *
 * @param  dev: IIO Device.
 * @param  attr: IIO Channel attribute.
 * @param  buf: User buffer.
 * @return  buffer len
 */
static ssize_t st_ism330dhcx_sysfs_scale_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct st_ism330dhcx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	enum st_ism330dhcx_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_ism330dhcx_fs_table[id].size; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%09u ",
				 st_ism330dhcx_fs_table[id].fs_avl[i].gain);
	buf[len - 1] = '\n';

	return len;
}

static int st_ism330dhcx_write_raw_get_fmt(struct iio_dev *indio_dev,
					   struct iio_chan_spec const *chan,
					   long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
		case IIO_ACCEL:
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			return IIO_VAL_FRACTIONAL;
		default:
			return IIO_VAL_INT_PLUS_MICRO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

ssize_t st_ism330dhcx_get_module_id(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	struct st_ism330dhcx_hw *hw = sensor->hw;

	return scnprintf(buf, PAGE_SIZE, "%u\n", hw->module_id);
}

static int
st_ism330dhcx_set_selftest(struct st_ism330dhcx_sensor *sensor,
			    int index)
{
	u8 mode, mask;

	switch (sensor->id) {
	case ST_ISM330DHCX_ID_ACC:
		mask = ST_ISM330DHCX_REG_ST_XL_MASK;
		mode = st_ism330dhcx_selftest_table[index].accel_value;
		break;
	case ST_ISM330DHCX_ID_GYRO:
		mask = ST_ISM330DHCX_REG_ST_G_MASK;
		mode = st_ism330dhcx_selftest_table[index].gyro_value;
		break;
	default:
		return -EINVAL;
	}

	return st_ism330dhcx_write_with_mask(sensor->hw,
					     ST_ISM330DHCX_REG_CTRL5_C_ADDR,
					     mask, mode);
}

static ssize_t
st_ism330dhcx_sysfs_get_selftest_available(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	return sprintf(buf, "%s, %s\n",
		       st_ism330dhcx_selftest_table[1].string_mode,
		       st_ism330dhcx_selftest_table[2].string_mode);
}

static ssize_t
st_ism330dhcx_sysfs_get_selftest_status(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int8_t result;
	char *message = NULL;
	struct st_ism330dhcx_sensor *sensor =
					  iio_priv(dev_to_iio_dev(dev));
	enum st_ism330dhcx_sensor_id id = sensor->id;

	if (id != ST_ISM330DHCX_ID_ACC &&
	    id != ST_ISM330DHCX_ID_GYRO)
		return -EINVAL;

	result = sensor->selftest_status;
	if (result == 0)
		message = "na";
	else if (result < 0)
		message = "fail";
	else if (result > 0)
		message = "pass";

	return sprintf(buf, "%s\n", message);
}

static int
st_ism330dhcx_selftest_sensor(struct st_ism330dhcx_sensor *sensor,
			      int test)
{
	int x_selftest = 0, y_selftest = 0, z_selftest = 0;
	int x = 0, y = 0, z = 0, try_count = 0;
	u8 i, status, n = 0;
	u8 reg, bitmask;
	int ret, delay, data_delay = 100000;
	u8 raw_data[6];

	switch (sensor->id) {
	case ST_ISM330DHCX_ID_ACC:
		reg = ST_ISM330DHCX_REG_OUTX_L_A_ADDR;
		bitmask = ST_ISM330DHCX_REG_STATUS_XLDA;
		data_delay = 50000;
		break;
	case ST_ISM330DHCX_ID_GYRO:
		reg = ST_ISM330DHCX_REG_OUTX_L_G_ADDR;
		bitmask = ST_ISM330DHCX_REG_STATUS_GDA;
		break;
	default:
		return -EINVAL;
	}

	/* reset selftest_status */
	sensor->selftest_status = -1;

	/* set selftest normal mode */
	ret = st_ism330dhcx_set_selftest(sensor, 0);
	if (ret < 0)
		return ret;

	ret = st_ism330dhcx_sensor_set_enable(sensor, true);
	if (ret < 0)
		return ret;

	/*
	 * wait at least one ODRs plus 10 % to be sure to fetch new
	 * sample data
	 */
	delay = 1000000 / sensor->odr;

	/* power up, wait for stable output */
	usleep_range(data_delay, data_delay + data_delay / 100);

	/* after enabled the sensor trash first sample */
	while (try_count < 3) {
		usleep_range(delay, delay + delay/10);
		ret = st_ism330dhcx_read_atomic(sensor->hw,
					  ST_ISM330DHCX_REG_STATUS_ADDR,
					  sizeof(status), &status);
		if (ret < 0)
			goto selftest_failure;

		if (status & bitmask) {
			st_ism330dhcx_read_atomic(sensor->hw, reg,
						  sizeof(raw_data),
						  raw_data);
			break;
		}

		try_count++;
	}

	if (try_count == 3)
		goto selftest_failure;

	/*
	 * for 5 times, after checking status bit, read the output
	 * registers
	 */
	for (i = 0; i < 5; i++) {
		try_count = 0;
		while (try_count < 3) {
			usleep_range(delay, delay + delay/10);
			ret = st_ism330dhcx_read_atomic(sensor->hw,
					  ST_ISM330DHCX_REG_STATUS_ADDR,
					  sizeof(status), &status);
			if (ret < 0)
				goto selftest_failure;

			if (status & bitmask) {
				ret = st_ism330dhcx_read_atomic(sensor->hw,
						       reg,
						       sizeof(raw_data),
						       raw_data);
				if (ret < 0)
					goto selftest_failure;

				/*
				 * for 5 times, after checking status
				 * bit, read the output registers
				 */
				x += ((s16)*(u16 *)&raw_data[0]) / 5;
				y += ((s16)*(u16 *)&raw_data[2]) / 5;
				z += ((s16)*(u16 *)&raw_data[4]) / 5;
				n++;

				break;
			}

			try_count++;
		}
	}

	if (i != n) {
		dev_err(sensor->hw->dev,
			"some acc samples missing (expected %d, read %d)\n",
			i, n);
		ret = -1;

		goto selftest_failure;
	}

	n = 0;

	/* set selftest mode */
	st_ism330dhcx_set_selftest(sensor, test);

	/* wait for stable output */
	usleep_range(data_delay, data_delay + data_delay / 100);

	try_count = 0;

	/* after enabled the sensor trash first sample */
	while (try_count < 3) {
		usleep_range(delay, delay + delay/10);
		ret = st_ism330dhcx_read_atomic(sensor->hw,
					  ST_ISM330DHCX_REG_STATUS_ADDR,
					  sizeof(status), &status);
		if (ret < 0)
			goto selftest_failure;

		if (status & bitmask) {
			st_ism330dhcx_read_atomic(sensor->hw, reg,
						  sizeof(raw_data),
						  raw_data);
			break;
		}

		try_count++;
	}

	if (try_count == 3)
		goto selftest_failure;

	/*
	 * for 5 times, after checking status bit, read the output
	 * registers
	 */
	for (i = 0; i < 5; i++) {
		try_count = 0;
		while (try_count < 3) {
			usleep_range(delay, delay + delay/10);
			ret = st_ism330dhcx_read_atomic(sensor->hw,
					  ST_ISM330DHCX_REG_STATUS_ADDR,
					  sizeof(status), &status);
			if (ret < 0)
				goto selftest_failure;

			if (status & bitmask) {
				ret = st_ism330dhcx_read_atomic(sensor->hw,
						  reg, sizeof(raw_data),
						  raw_data);
				if (ret < 0)
					goto selftest_failure;

				x_selftest += ((s16)*(u16 *)&raw_data[0]) / 5;
				y_selftest += ((s16)*(u16 *)&raw_data[2]) / 5;
				z_selftest += ((s16)*(u16 *)&raw_data[4]) / 5;
				n++;

				break;
			}

			try_count++;
		}
	}

	if (i != n) {
		dev_err(sensor->hw->dev,
			"some samples missing (expected %d, read %d)\n",
			i, n);
		ret = -1;

		goto selftest_failure;
	}

	if ((abs(x_selftest - x) < sensor->min_st) ||
	    (abs(x_selftest - x) > sensor->max_st)) {
		sensor->selftest_status = -1;
		dev_info(sensor->hw->dev, "st: failure on x: non-st(%d), st(%d)\n",
			 x, x_selftest);
		goto selftest_failure;
	}

	if ((abs(y_selftest - y) < sensor->min_st) ||
	    (abs(y_selftest - y) > sensor->max_st)) {
		sensor->selftest_status = -1;
		dev_info(sensor->hw->dev, "st: failure on y: non-st(%d), st(%d)\n",
			 y, y_selftest);
		goto selftest_failure;
	}

	if ((abs(z_selftest - z) < sensor->min_st) ||
	    (abs(z_selftest - z) > sensor->max_st)) {
		sensor->selftest_status = -1;
		dev_info(sensor->hw->dev, "st: failure on z: non-st(%d), st(%d)\n",
			 z, z_selftest);
		goto selftest_failure;
	}

	sensor->selftest_status = 1;

selftest_failure:
	/* restore selftest to normal mode */
	st_ism330dhcx_set_selftest(sensor, 0);

	return st_ism330dhcx_sensor_set_enable(sensor, false);
}

static ssize_t
st_ism330dhcx_sysfs_start_selftest(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_ism330dhcx_sensor *sensor = iio_priv(iio_dev);
	enum st_ism330dhcx_sensor_id id = sensor->id;
	struct st_ism330dhcx_hw *hw = sensor->hw;
	int ret, test;
	u32 gain;

	if (id != ST_ISM330DHCX_ID_ACC &&
	    id != ST_ISM330DHCX_ID_GYRO)
		return -EINVAL;

	for (test = 0; test < ARRAY_SIZE(st_ism330dhcx_selftest_table);
	     test++) {
		if (strncmp(buf, st_ism330dhcx_selftest_table[test].string_mode,
			strlen(st_ism330dhcx_selftest_table[test].string_mode)) == 0)
			break;
	}

	if (test == ARRAY_SIZE(st_ism330dhcx_selftest_table))
		return -EINVAL;

	ret = iio_device_claim_direct_mode(iio_dev);
	if (ret)
		return ret;

	/* self test mode unavailable if sensor enabled */
	if (hw->enable_mask & BIT_ULL(id)) {
		ret = -EBUSY;

		goto out_claim;
	}

	st_ism330dhcx_bk_regs(hw);

	/* disable interrupt on FIFO watermak */
	if (hw->has_hw_fifo) {
		ret = st_ism330dhcx_get_int_reg(hw);
		if (ret < 0)
			goto restore_regs;

		ret = st_ism330dhcx_write_with_mask(hw, hw->irq_reg,
					 ST_ISM330DHCX_REG_INT_FIFO_TH_MASK, 0);
		if (ret < 0)
			goto restore_regs;
	}

	gain = sensor->gain;
	if (id == ST_ISM330DHCX_ID_ACC) {
		/* set BDU = 1, FS = 4 g, ODR = 52 Hz */
		st_ism330dhcx_set_full_scale(sensor,
					  ST_ISM330DHCX_ACC_FS_4G_GAIN);
		st_ism330dhcx_set_odr(sensor, 52, 0);
		st_ism330dhcx_selftest_sensor(sensor, test);

		/* restore full scale after test */
		st_ism330dhcx_set_full_scale(sensor, gain);
	} else {
		/* set BDU = 1, ODR = 208 Hz, FS = 2000 dps */
		st_ism330dhcx_set_full_scale(sensor,
				       ST_ISM330DHCX_GYRO_FS_2000_GAIN);
		/*
		 * before enable gyro add 150 ms delay when gyro
		 * self-test
		 */
		usleep_range(150000, 151000);

		st_ism330dhcx_set_odr(sensor, 208, 0);
		st_ism330dhcx_selftest_sensor(sensor, test);

		/* restore full scale after test */
		st_ism330dhcx_set_full_scale(sensor, gain);
	}

restore_regs:
	st_ism330dhcx_restore_regs(hw);

out_claim:
	iio_device_release_direct_mode(iio_dev);

	return size;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_ism330dhcx_sysfs_sampling_frequency_avail);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_ism330dhcx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_anglvel_scale_available, 0444,
		       st_ism330dhcx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_temp_scale_available, 0444,
		       st_ism330dhcx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_ism330dhcx_get_max_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, 0200, NULL, st_ism330dhcx_flush_fifo, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644, st_ism330dhcx_get_watermark,
		       st_ism330dhcx_set_watermark, 0);
static IIO_DEVICE_ATTR(module_id, 0444, st_ism330dhcx_get_module_id, NULL, 0);
static IIO_DEVICE_ATTR(selftest_available, 0444,
		       st_ism330dhcx_sysfs_get_selftest_available,
		       NULL, 0);
static IIO_DEVICE_ATTR(selftest, 0644,
		       st_ism330dhcx_sysfs_get_selftest_status,
		       st_ism330dhcx_sysfs_start_selftest, 0);

static struct attribute *st_ism330dhcx_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_ism330dhcx_acc_attribute_group = {
	.attrs = st_ism330dhcx_acc_attributes,
};

static const struct iio_info st_ism330dhcx_acc_info = {
	.attrs = &st_ism330dhcx_acc_attribute_group,
	.read_raw = st_ism330dhcx_read_raw,
	.write_raw = st_ism330dhcx_write_raw,
	.write_raw_get_fmt = st_ism330dhcx_write_raw_get_fmt,
	.read_event_config = st_ism330dhcx_read_event_config,
	.write_event_config = st_ism330dhcx_write_event_config,
	.write_event_value = st_ism330dhcx_write_event_value,
	.read_event_value = st_ism330dhcx_read_event_value,

#ifdef CONFIG_DEBUG_FS
	/* connect debug info to first device */
	.debugfs_reg_access = st_ism330dhcx_reg_access,
#endif /* CONFIG_DEBUG_FS */

};

static struct attribute *st_ism330dhcx_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_ism330dhcx_gyro_attribute_group = {
	.attrs = st_ism330dhcx_gyro_attributes,
};

static const struct iio_info st_ism330dhcx_gyro_info = {
	.attrs = &st_ism330dhcx_gyro_attribute_group,
	.read_raw = st_ism330dhcx_read_raw,
	.write_raw = st_ism330dhcx_write_raw,
	.write_raw_get_fmt = st_ism330dhcx_write_raw_get_fmt,
};

static struct attribute *st_ism330dhcx_temp_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_temp_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_ism330dhcx_temp_attribute_group = {
	.attrs = st_ism330dhcx_temp_attributes,
};

static const struct iio_info st_ism330dhcx_temp_info = {
	.attrs = &st_ism330dhcx_temp_attribute_group,
	.read_raw = st_ism330dhcx_read_raw,
	.write_raw = st_ism330dhcx_write_raw,
	.write_raw_get_fmt = st_ism330dhcx_write_raw_get_fmt,
};

static const unsigned long st_ism330dhcx_available_scan_masks[] = {

#if defined(CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP)
	GENMASK(3, 0), 0x0
#else /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */
	GENMASK(2, 0), 0x0
#endif /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */

};
static const unsigned long st_ism330dhcx_sc_available_scan_masks[] = {

#if defined(CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP)
	GENMASK(1, 0), 0x0
#else /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */
	BIT(0), 0x0
#endif /* CONFIG_IIO_ST_ISM330DHCX_ASYNC_HW_TIMESTAMP */

};

static int st_ism330dhcx_check_irq_config_pin(struct st_ism330dhcx_hw *hw)
{
	struct fwnode_handle *fwnode;
	struct device *dev = hw->dev;
	int int_pin, ret;

	fwnode = dev_fwnode(dev);
	if (!fwnode)
		return -EINVAL;

	ret = fwnode_property_read_u32(fwnode, "st,int-pin", &int_pin);
	if (ret < 0) {
		struct st_sensors_platform_data *pdata;
		struct device *dev = hw->dev;

		pdata = (struct st_sensors_platform_data *)dev->platform_data;
		int_pin = pdata ? pdata->drdy_int_pin : 1;
	}

	hw->int_pin = int_pin;

	return 0;
}

int st_ism330dhcx_get_int_reg(struct st_ism330dhcx_hw *hw)
{
	int err;

	err = st_ism330dhcx_check_irq_config_pin(hw);
	if (err < 0)
		return err;

	switch (hw->int_pin) {
	case 1:
		hw->irq_reg = ST_ISM330DHCX_REG_INT1_CTRL_ADDR;
		hw->embfunc_pg0_irq_reg = ST_ISM330DHCX_REG_MD1_CFG_ADDR;
		hw->embfunc_irq_reg = ST_ISM330DHCX_REG_EMB_FUNC_INT1_ADDR;
		break;
	case 2:
		hw->irq_reg = ST_ISM330DHCX_REG_INT2_CTRL_ADDR;
		hw->embfunc_pg0_irq_reg = ST_ISM330DHCX_REG_MD2_CFG_ADDR;
		hw->embfunc_irq_reg = ST_ISM330DHCX_REG_EMB_FUNC_INT2_ADDR;
		break;
	default:
		dev_err(hw->dev, "unsupported interrupt pin\n");
		err = -EINVAL;
		break;
	}

	return err;
}

static int st_ism330dhcx_reset_device(struct st_ism330dhcx_hw *hw)
{
	int err;

	/* disable I3C */
	err = st_ism330dhcx_write_with_mask(hw, ST_ISM330DHCX_REG_CTRL9_XL_ADDR,
					 ST_ISM330DHCX_REG_I3C_DISABLE_MASK, 1);
	if (err < 0)
		return err;

	/* sw reset */
	err = st_ism330dhcx_write_with_mask(hw,
					 ST_ISM330DHCX_REG_CTRL3_C_ADDR,
					 ST_ISM330DHCX_REG_SW_RESET_MASK, 1);
	if (err < 0)
		return err;

	usleep_range(15, 20);

	/* boot */
	err = st_ism330dhcx_write_with_mask(hw,
					 ST_ISM330DHCX_REG_CTRL3_C_ADDR,
					 ST_ISM330DHCX_REG_BOOT_MASK, 1);

	msleep(20);

	return err;
}

static int st_ism330dhcx_init_device(struct st_ism330dhcx_hw *hw)
{
	int err;

	/* enable Block Data Update */
	err = st_ism330dhcx_write_with_mask(hw,
					 ST_ISM330DHCX_REG_CTRL3_C_ADDR,
					 ST_ISM330DHCX_REG_BDU_MASK, 1);
	if (err < 0)
		return err;

	/* enable rouding for fast FIFO reading */
	err = st_ism330dhcx_write_with_mask(hw,
					 ST_ISM330DHCX_REG_CTRL5_C_ADDR,
					 ST_ISM330DHCX_REG_ROUNDING_MASK, 3);
	if (err < 0)
		return err;

	/* Enable DRDY MASK for filters settling time */
	return st_ism330dhcx_write_with_mask(hw,
					     ST_ISM330DHCX_REG_CTRL4_C_ADDR,
					     ST_ISM330DHCX_REG_DRDY_MASK, 1);
}

/**
 * Allocate IIO device
 *
 * @param  hw: ST IMU MEMS hw instance.
 * @param  id: Sensor Identifier.
 * @retval  struct iio_dev *, NULL if ERROR
 */
static struct iio_dev *st_ism330dhcx_alloc_iiodev(struct st_ism330dhcx_hw *hw,
					       enum st_ism330dhcx_sensor_id id)
{
	struct st_ism330dhcx_sensor *sensor;
	struct iio_dev *iio_dev;

	iio_dev = devm_iio_device_alloc(hw->dev, sizeof(*sensor));
	if (!iio_dev)
		return NULL;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = hw->dev;

	sensor = iio_priv(iio_dev);
	sensor->id = id;
	sensor->hw = hw;
	sensor->watermark = 1;

	sensor->decimator = 0;
	sensor->dec_counter = 0;

	switch (id) {
	case ST_ISM330DHCX_ID_ACC:
		iio_dev->channels = st_ism330dhcx_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_ism330dhcx_acc_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "ism330dhcx_accel");
		iio_dev->info = &st_ism330dhcx_acc_info;
		iio_dev->available_scan_masks = st_ism330dhcx_available_scan_masks;

		sensor->batch_reg.addr = ST_ISM330DHCX_REG_FIFO_CTRL3_ADDR;
		sensor->batch_reg.mask = ST_ISM330DHCX_REG_BDR_XL_MASK;
		sensor->max_watermark = ST_ISM330DHCX_MAX_FIFO_DEPTH;
		sensor->odr = st_ism330dhcx_odr_table[id].odr_avl[1].hz;
		sensor->uodr = st_ism330dhcx_odr_table[id].odr_avl[1].uhz;
		sensor->min_st = ST_ISM330DHCX_SELFTEST_ACCEL_MIN;
		sensor->max_st = ST_ISM330DHCX_SELFTEST_ACCEL_MAX;
		st_ism330dhcx_set_full_scale(sensor,
				st_ism330dhcx_fs_table[id].fs_avl[1].gain);
		break;
	case ST_ISM330DHCX_ID_GYRO:
		iio_dev->channels = st_ism330dhcx_gyro_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_ism330dhcx_gyro_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "ism330dhcx_gyro");
		iio_dev->info = &st_ism330dhcx_gyro_info;
		iio_dev->available_scan_masks = st_ism330dhcx_available_scan_masks;

		sensor->batch_reg.addr = ST_ISM330DHCX_REG_FIFO_CTRL3_ADDR;
		sensor->batch_reg.mask = ST_ISM330DHCX_REG_BDR_GY_MASK;
		sensor->max_watermark = ST_ISM330DHCX_MAX_FIFO_DEPTH;
		sensor->odr = st_ism330dhcx_odr_table[id].odr_avl[1].hz;
		sensor->uodr = st_ism330dhcx_odr_table[id].odr_avl[1].uhz;
		sensor->min_st = ST_ISM330DHCX_SELFTEST_GYRO_MIN;
		sensor->max_st = ST_ISM330DHCX_SELFTEST_GYRO_MAX;
		st_ism330dhcx_set_full_scale(sensor,
				st_ism330dhcx_fs_table[id].fs_avl[2].gain);
		break;
	case ST_ISM330DHCX_ID_TEMP:
		iio_dev->channels = st_ism330dhcx_temp_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_ism330dhcx_temp_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "ism330dhcx_temp");
		iio_dev->info = &st_ism330dhcx_temp_info;

		sensor->batch_reg.addr = ST_ISM330DHCX_REG_FIFO_CTRL4_ADDR;
		sensor->batch_reg.mask = ST_ISM330DHCX_REG_ODR_T_BATCH_MASK;
		sensor->max_watermark = ST_ISM330DHCX_MAX_FIFO_DEPTH;
		sensor->odr = st_ism330dhcx_odr_table[id].odr_avl[1].hz;
		sensor->uodr = st_ism330dhcx_odr_table[id].odr_avl[1].uhz;
		sensor->gain = st_ism330dhcx_fs_table[id].fs_avl[0].gain;
		sensor->offset = ST_ISM330DHCX_TEMP_OFFSET;
		break;
	default:
		return NULL;
	}

	iio_dev->name = sensor->name;

	return iio_dev;
}

static void st_ism330dhcx_get_properties(struct st_ism330dhcx_hw *hw)
{
	if (device_property_read_u32(hw->dev, "st,module_id",
				     &hw->module_id)) {
		hw->module_id = 1;
	}
}

/**
 * Probe device function
 * Implements [MODULE] feature for Power Management
 *
 * @param  dev: Device pointer.
 * @param  irq: I2C/SPI client irq.
 * @param  tf_ops: Bus Transfer Function pointer.
 * @retval  struct iio_dev *, NULL if ERROR
 */
int st_ism330dhcx_probe(struct device *dev, int irq, struct regmap *regmap)
{
	struct st_ism330dhcx_hw *hw;
	int i, err;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	dev_set_drvdata(dev, (void *)hw);

	mutex_init(&hw->lock);
	mutex_init(&hw->fifo_lock);
	mutex_init(&hw->page_lock);

	hw->regmap = regmap;
	hw->dev = dev;
	hw->irq = irq;
	hw->odr_table = st_ism330dhcx_odr_table;
	hw->fs_table = st_ism330dhcx_fs_table;
	hw->has_hw_fifo = hw->irq > 0 ? true : false;

	err = st_ism330dhcx_check_whoami(hw);
	if (err < 0)
		return err;

	st_ism330dhcx_get_properties(hw);

	err = st_ism330dhcx_get_odr_calibration(hw);
	if (err < 0)
		return err;

	err = st_ism330dhcx_reset_device(hw);
	if (err < 0)
		return err;

	err = st_ism330dhcx_init_device(hw);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(st_ism330dhcx_main_sensor_list); i++) {
		enum st_ism330dhcx_sensor_id id = st_ism330dhcx_main_sensor_list[i];

		hw->iio_devs[id] = st_ism330dhcx_alloc_iiodev(hw, id);
		if (!hw->iio_devs[id])
			continue;
	}

	err = st_ism330dhcx_shub_probe(hw);
	if (err < 0)
		return err;

	err = st_ism330dhcx_allocate_buffers(hw);
	if (err < 0)
		return err;

	/* if fifo not supported just few sensors can be enabled */
	if (hw->has_hw_fifo) {
		err = st_ism330dhcx_trigger_setup(hw);
		if (err < 0)
			return err;
		err = st_ism330dhcx_event_init(hw);
		if (err < 0)
			return err;

		err = st_ism330dhcx_embfunc_probe(hw);
		if (err < 0)
			return err;

		err = st_ism330dhcx_mlc_probe(hw);
		if (err < 0)
			return err;
	}

	for (i = 0; i < ST_ISM330DHCX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

	if (hw->has_hw_fifo) {
		err = st_ism330dhcx_mlc_init_preload(hw);
		if (err)
			return err;
	}

	device_init_wakeup(dev,
			   device_property_read_bool(dev, "wakeup-source"));

	dev_info(dev, "Device probed\n");

	return 0;
}
EXPORT_SYMBOL(st_ism330dhcx_probe);

void st_ism330dhcx_remove(struct device *dev)
{
	st_ism330dhcx_mlc_remove(dev);
}
EXPORT_SYMBOL(st_ism330dhcx_remove);

static int __maybe_unused st_ism330dhcx_bk_regs(struct st_ism330dhcx_hw *hw)
{
	unsigned int data;
	bool restore = 0;
	int i, err = 0;

	mutex_lock(&hw->page_lock);

	for (i = 0; i < ST_ISM330DHCX_SUSPEND_RESUME_REGS; i++) {
		if (st_ism330dhcx_suspend_resume[i].page != FUNC_CFG_ACCESS_0) {
			err = regmap_update_bits(hw->regmap,
				     ST_ISM330DHCX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_ISM330DHCX_REG_ACCESS_MASK,
				     FIELD_PREP(ST_ISM330DHCX_REG_ACCESS_MASK,
					 st_ism330dhcx_suspend_resume[i].page));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_ism330dhcx_suspend_resume[i].addr);
				break;
			}

			restore = 1;
		}

		err = regmap_read(hw->regmap,
				  st_ism330dhcx_suspend_resume[i].addr,
				  &data);
		if (err < 0) {
			dev_err(hw->dev,
				"failed to save register %02x\n",
				st_ism330dhcx_suspend_resume[i].addr);
			goto out_lock;
		}

		if (restore) {
			err = regmap_update_bits(hw->regmap,
				     ST_ISM330DHCX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_ISM330DHCX_REG_ACCESS_MASK,
				     FIELD_PREP(ST_ISM330DHCX_REG_ACCESS_MASK,
						FUNC_CFG_ACCESS_0));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_ism330dhcx_suspend_resume[i].addr);
				break;
			}

			restore = 0;
		}

		st_ism330dhcx_suspend_resume[i].val = data;
	}

out_lock:
	mutex_unlock(&hw->page_lock);

	return err;
}

static int __maybe_unused st_ism330dhcx_restore_regs(struct st_ism330dhcx_hw *hw)
{
	bool restore = 0;
	int i, err = 0;

	mutex_lock(&hw->page_lock);

	for (i = 0; i < ST_ISM330DHCX_SUSPEND_RESUME_REGS; i++) {
		if (st_ism330dhcx_suspend_resume[i].page != FUNC_CFG_ACCESS_0) {
			err = regmap_update_bits(hw->regmap,
				     ST_ISM330DHCX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_ISM330DHCX_REG_ACCESS_MASK,
				     FIELD_PREP(ST_ISM330DHCX_REG_ACCESS_MASK,
					st_ism330dhcx_suspend_resume[i].page));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_ism330dhcx_suspend_resume[i].addr);
				break;
			}

			restore = 1;
		}

		err = regmap_update_bits(hw->regmap,
					 st_ism330dhcx_suspend_resume[i].addr,
					 st_ism330dhcx_suspend_resume[i].mask,
					 st_ism330dhcx_suspend_resume[i].val);
		if (err < 0) {
			dev_err(hw->dev,
				"failed to update %02x reg\n",
				st_ism330dhcx_suspend_resume[i].addr);
			break;
		}

		if (restore) {
			err = regmap_update_bits(hw->regmap,
				     ST_ISM330DHCX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_ISM330DHCX_REG_ACCESS_MASK,
				     FIELD_PREP(ST_ISM330DHCX_REG_ACCESS_MASK,
						FUNC_CFG_ACCESS_0));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_ism330dhcx_suspend_resume[i].addr);
				break;
			}

			restore = 0;
		}
	}

	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Power Management suspend callback [MODULE]
 * Implements [MODULE] feature for Power Management
 *
 * @param  dev: Device pointer.
 * @retval  0 is OK, negative value if ERROR
 */
static int __maybe_unused st_ism330dhcx_suspend(struct device *dev)
{
	struct st_ism330dhcx_hw *hw = dev_get_drvdata(dev);
	struct st_ism330dhcx_sensor *sensor;
	int i, err = 0;

	dev_info(dev, "Suspending device\n");

	for (i = 0; i < ST_ISM330DHCX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT_ULL(sensor->id)))
			continue;

		/* power off enabled sensors */
		err = st_ism330dhcx_set_odr(sensor, 0, 0);
		if (err < 0)
			return err;
	}

	if (st_ism330dhcx_is_fifo_enabled(hw)) {
		err = st_ism330dhcx_suspend_fifo(hw);
		if (err < 0)
			return err;
	}

	err = st_ism330dhcx_bk_regs(hw);

	if (device_may_wakeup(dev))
		enable_irq_wake(hw->irq);

	return err < 0 ? err : 0;
}

/**
 * Power Management resume callback [MODULE]
 * Implements [MODULE] feature for Power Management
 *
 * @param  dev: Device pointer.
 * @retval  0 is OK, negative value if ERROR
 */
static int __maybe_unused st_ism330dhcx_resume(struct device *dev)
{
	struct st_ism330dhcx_hw *hw = dev_get_drvdata(dev);
	struct st_ism330dhcx_sensor *sensor;
	int i, err = 0;

	dev_info(dev, "Resuming device\n");

	if (device_may_wakeup(dev))
		disable_irq_wake(hw->irq);

	err = st_ism330dhcx_restore_regs(hw);
	if (err < 0)
		return err;

	for (i = 0; i < ST_ISM330DHCX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT_ULL(sensor->id)))
			continue;

		err = st_ism330dhcx_set_odr(sensor, sensor->odr, sensor->uodr);
		if (err < 0)
			return err;
	}

	err = st_ism330dhcx_reset_hwts(hw);
	if (err < 0)
		return err;

	if (st_ism330dhcx_is_fifo_enabled(hw))
		err = st_ism330dhcx_set_fifo_mode(hw, ST_ISM330DHCX_FIFO_CONT);

	return err < 0 ? err : 0;
}

const struct dev_pm_ops st_ism330dhcx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_ism330dhcx_suspend, st_ism330dhcx_resume)
};
EXPORT_SYMBOL(st_ism330dhcx_pm_ops);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_ism330dhcx driver");
MODULE_LICENSE("GPL v2");
