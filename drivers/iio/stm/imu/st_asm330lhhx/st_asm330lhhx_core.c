// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_asm330lhhx sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2019 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>

#include <linux/platform_data/st_sensors_pdata.h>

#include "st_asm330lhhx.h"

static struct st_asm330lhhx_selftest_table {
	char *string_mode;
	u8 accel_value;
	u8 gyro_value;
	u8 gyro_mask;
} st_asm330lhhx_selftest_table[] = {
	[0] = {
		.string_mode = "disabled",
		.accel_value = ST_ASM330LHHX_SELF_TEST_DISABLED_VAL,
		.gyro_value = ST_ASM330LHHX_SELF_TEST_DISABLED_VAL,
	},
	[1] = {
		.string_mode = "positive-sign",
		.accel_value = ST_ASM330LHHX_SELF_TEST_POS_SIGN_VAL,
		.gyro_value = ST_ASM330LHHX_SELF_TEST_POS_SIGN_VAL
	},
	[2] = {
		.string_mode = "negative-sign",
		.accel_value = ST_ASM330LHHX_SELF_TEST_NEG_ACCEL_SIGN_VAL,
		.gyro_value = ST_ASM330LHHX_SELF_TEST_NEG_GYRO_SIGN_VAL
	},
};

static const int st_asm330lhhx_odr_index[] = {
	1, 12, 26, 52, 104, 208, 416, 833
};

static const int st_asm330lhhx_odr_divider_index[] = {
	2, 4, 10, 20, 45, 100, 200, 400, 800
};

/*
 * LPF filter configuration
 *
 * the total amount of sample to discard is set to the value of
 * samples_to_discard plus the settling_samples related to the LPF
 * configuration and the sensor odr set.
 */
static const struct st_asm330lhhx_lpf_discard_table_t {
	u32 samples_to_discard[ST_ASM330LHHX_ODR_LIST_SIZE];
	u32 settling_samples[9][ST_ASM330LHHX_ODR_LIST_SIZE];
} st_asm330lhhx_lpf_discard_table[ST_ASM330LHHX_ID_HW] = {
	[ST_ASM330LHHX_ID_GYRO] = {
		/* samples_to_discard when no filter enabled */
		.samples_to_discard =    { 2, 2, 3, 3, 3, 3, 3, 3 },

		/* settling_samples vs ODRs and FTYPE table for gyro */
		.settling_samples[0] = {  0,  0,  0,  0,  0,  0,  2,  3 }, /* FTYPE 0 */
		.settling_samples[1] = {  0,  0,  0,  0,  0,  1,  2,  4 }, /* FTYPE 1 */
		.settling_samples[2] = {  0,  0,  0,  0,  1,  1,  3,  5 }, /* FTYPE 2 */
		.settling_samples[3] = {  0,  0,  0,  0,  0,  0,  1,  2 }, /* FTYPE 3 */
		.settling_samples[4] = {  0,  0,  0,  1,  1,  2,  4,  9 }, /* FTYPE 4 */
		.settling_samples[5] = {  0,  0,  1,  1,  2,  4,  9, 18 }, /* FTYPE 5 */
		.settling_samples[6] = {  0,  0,  1,  2,  3,  6, 12, 24 }, /* FTYPE 6 */
		.settling_samples[7] = {  0,  1,  2,  3,  6, 12, 24, 48 }, /* FTYPE 7 */
	},
	[ST_ASM330LHHX_ID_ACC] = {
		/* samples_to_discard when no filter enabled */
		.samples_to_discard =     { 3, 3, 3, 3, 3, 3, 3, 3, 3 },

		/* settling_samples vs ODRs and accel Bandwidth table */
		.settling_samples[0] = {   0,   0,   0,   0,   0,   0,   0,   0 }, /* ODR/2 */
		.settling_samples[1] = {   0,   0,   0,   0,   0,   0,   0,   0 }, /* ODR/4 */
		.settling_samples[2] = {  10,  10,  10,  10,  10,  10,  10,  10 }, /* ODR/10 */
		.settling_samples[3] = {  19,  19,  19,  19,  19,  19,  19,  19 }, /* ODR/20 */
		.settling_samples[4] = {  38,  38,  38,  38,  38,  38,  38,  38 }, /* ODR/45 */
		.settling_samples[5] = {  75,  75,  75,  75,  75,  75,  75,  75 }, /* ODR/100 */
		.settling_samples[6] = { 150, 150, 150, 150, 150, 150, 150, 150 }, /* ODR/200 */
		.settling_samples[7] = { 296, 296, 296, 296, 296, 296, 296, 296 }, /* ODR/400 */
		.settling_samples[8] = { 595, 595, 595, 595, 595, 595, 595, 595 }, /* ODR/800 */
	},
};

static const struct st_asm330lhhx_power_mode_table {
	char *string_mode;
	enum st_asm330lhhx_pm_t val;
} st_asm330lhhx_power_mode[] = {
	[0] = {
		.string_mode = "HP_MODE",
		.val = ST_ASM330LHHX_HP_MODE,
	},
	[1] = {
		.string_mode = "LP_MODE",
		.val = ST_ASM330LHHX_LP_MODE,
	},
};

static struct st_asm330lhhx_suspend_resume_entry
	st_asm330lhhx_suspend_resume[ST_ASM330LHHX_SUSPEND_RESUME_REGS] = {
	[ST_ASM330LHHX_CTRL1_XL_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_CTRL1_XL_ADDR,
		.mask = ST_ASM330LHHX_REG_FS_XL_MASK |
			ST_ASM330LHHX_REG_LPF2_XL_EN_MASK,
	},
	[ST_ASM330LHHX_CTRL2_G_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_CTRL2_G_ADDR,
		.mask = ST_ASM330LHHX_REG_FS_G_MASK,
	},
	[ST_ASM330LHHX_REG_CTRL3_C_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_CTRL3_C_ADDR,
		.mask = ST_ASM330LHHX_REG_BDU_MASK	|
			ST_ASM330LHHX_REG_PP_OD_MASK	|
			ST_ASM330LHHX_REG_H_LACTIVE_MASK,
	},
	[ST_ASM330LHHX_REG_CTRL4_C_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_CTRL4_C_ADDR,
		.mask = ST_ASM330LHHX_REG_DRDY_MASK |
			ST_ASM330LHHX_REG_LPF1_SEL_G_MASK,
	},
	[ST_ASM330LHHX_REG_CTRL5_C_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_CTRL5_C_ADDR,
		.mask = ST_ASM330LHHX_REG_ROUNDING_MASK,
	},
	[ST_ASM330LHHX_REG_CTRL6_C_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_CTRL6_C_ADDR,
		.mask = ST_ASM330LHHX_REG_FTYPE_MASK,
	},
	[ST_ASM330LHHX_REG_CTRL8_XL_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_CTRL8_XL_ADDR,
		.mask = ST_ASM330LHHX_REG_HPCF_XL_MASK,
	},
	[ST_ASM330LHHX_REG_CTRL10_C_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_CTRL10_C_ADDR,
		.mask = ST_ASM330LHHX_REG_TIMESTAMP_EN_MASK,
	},
	[ST_ASM330LHHX_REG_INT_CFG0_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_INT_CFG0_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_INT_CFG1_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_INT_CFG1_ADDR,
		.mask = ST_ASM330LHHX_INTERRUPTS_ENABLE_MASK,
	},
	[ST_ASM330LHHX_REG_MD1_CFG_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_MD1_CFG_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_MD2_CFG_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_MD2_CFG_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_INT1_CTRL_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_INT1_CTRL_ADDR,
		.mask = ST_ASM330LHHX_REG_INT_FIFO_TH_MASK,
	},
	[ST_ASM330LHHX_REG_INT2_CTRL_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_INT2_CTRL_ADDR,
		.mask = ST_ASM330LHHX_REG_INT_FIFO_TH_MASK,
	},
	[ST_ASM330LHHX_REG_FIFO_CTRL1_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_FIFO_CTRL1_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_FIFO_CTRL2_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_FIFO_CTRL2_ADDR,
		.mask = ST_ASM330LHHX_REG_FIFO_WTM8_MASK,
	},
	[ST_ASM330LHHX_REG_FIFO_CTRL3_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_FIFO_CTRL3_ADDR,
		.mask = ST_ASM330LHHX_REG_BDR_XL_MASK |
			ST_ASM330LHHX_REG_BDR_GY_MASK,
	},
	[ST_ASM330LHHX_REG_FIFO_CTRL4_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_ASM330LHHX_REG_FIFO_CTRL4_ADDR,
		.mask = ST_ASM330LHHX_REG_DEC_TS_MASK |
			ST_ASM330LHHX_REG_ODR_T_BATCH_MASK,
	},
	[ST_ASM330LHHX_REG_EMB_FUNC_EN_B_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_ASM330LHHX_EMB_FUNC_EN_B_ADDR,
		.mask = ST_ASM330LHHX_FSM_EN_MASK |
			ST_ASM330LHHX_MLC_EN_MASK,
	},
	[ST_ASM330LHHX_REG_FSM_INT1_A_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_ASM330LHHX_FSM_INT1_A_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_FSM_INT1_B_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_ASM330LHHX_FSM_INT1_B_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_MLC_INT1_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_ASM330LHHX_MLC_INT1_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_FSM_INT2_A_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_ASM330LHHX_FSM_INT2_A_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_FSM_INT2_B_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_ASM330LHHX_FSM_INT2_B_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_ASM330LHHX_REG_MLC_INT2_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_ASM330LHHX_MLC_INT2_ADDR,
		.mask = GENMASK(7, 0),
	},
};

static const struct st_asm330lhhx_odr_table_entry st_asm330lhhx_odr_table[] = {
	[ST_ASM330LHHX_ID_ACC] = {
		.size = 9,
		.reg = {
			.addr = ST_ASM330LHHX_CTRL1_XL_ADDR,
			.mask = GENMASK(7, 4),
		},
		.pm = {
			.addr = ST_ASM330LHHX_REG_CTRL6_C_ADDR,
			.mask = ST_ASM330LHHX_REG_XL_HM_MODE_MASK,
		},
		.batching_reg = {
			.addr = ST_ASM330LHHX_REG_FIFO_CTRL3_ADDR,
			.mask = GENMASK(3, 0),
		},
		.odr_avl[0] = {   0,      0,  0x00,  0x00 },
		.odr_avl[1] = {   1, 625000,  0x01,  0x0b },
		.odr_avl[2] = {  12, 500000,  0x01,  0x01 },
		.odr_avl[3] = {  26,      0,  0x02,  0x02 },
		.odr_avl[4] = {  52,      0,  0x03,  0x03 },
		.odr_avl[5] = { 104,      0,  0x04,  0x04 },
		.odr_avl[6] = { 208,      0,  0x05,  0x05 },
		.odr_avl[7] = { 416,      0,  0x06,  0x06 },
		.odr_avl[8] = { 833,      0,  0x07,  0x07 },
	},
	[ST_ASM330LHHX_ID_GYRO] = {
		.size = 9,
		.reg = {
			.addr = ST_ASM330LHHX_CTRL2_G_ADDR,
			.mask = GENMASK(7, 4),
		},
		.pm = {
			.addr = ST_ASM330LHHX_REG_CTRL7_G_ADDR,
			.mask = ST_ASM330LHHX_REG_G_HM_MODE_MASK,
		},
		.batching_reg = {
			.addr = ST_ASM330LHHX_REG_FIFO_CTRL3_ADDR,
			.mask = GENMASK(7, 4),
		},
		.odr_avl[0] = {   0,      0,  0x00,  0x00 },
		.odr_avl[1] = {   6, 500000,  0x01,  0x0b },
		.odr_avl[2] = {  12, 500000,  0x01,  0x01 },
		.odr_avl[3] = {  26,      0,  0x02,  0x02 },
		.odr_avl[4] = {  52,      0,  0x03,  0x03 },
		.odr_avl[5] = { 104,      0,  0x04,  0x04 },
		.odr_avl[6] = { 208,      0,  0x05,  0x05 },
		.odr_avl[7] = { 416,      0,  0x06,  0x06 },
		.odr_avl[8] = { 833,      0,  0x07,  0x07 },
	},
	[ST_ASM330LHHX_ID_TEMP] = {
		.size = 3,
		.batching_reg = {
			.addr = ST_ASM330LHHX_REG_FIFO_CTRL4_ADDR,
			.mask = GENMASK(5, 4),
		},
		.odr_avl[0] = {  0,      0,   0x00,  0x00 },
		.odr_avl[1] = { 12, 500000,   0x02,  0x02 },
		.odr_avl[2] = { 52,      0,   0x03,  0x03 },
	},
};

/**
 * List of supported supported device settings
 *
 * The following table list all device features in terms of supported
 * MLC and SHUB.
 */
static const struct st_asm330lhhx_settings st_asm330lhhx_sensor_settings[] = {
	{
		.id = {
			.hw_id = ST_ASM330LHHX_ID,
			.name = ST_ASM330LHHX_DEV_NAME,
		},
		.st_mlc_probe = true,
		.st_shub_probe = true,
		.st_power_mode = true,
	},
	{
		.id = {
			.hw_id = ST_ASM330LHH_ID,
			.name = ST_ASM330LHH_DEV_NAME,
		},
	},
	{
		.id = {
			.hw_id = ST_ASM330LHHXG1_ID,
			.name = ST_ASM330LHHXG1_DEV_NAME,
		},
		.st_mlc_probe = true,
		.st_shub_probe = true,
		.st_power_mode = true,
	},
	{
		.id = {
			.hw_id = ST_ASM330LHB_ID,
			.name = ST_ASM330LHB_DEV_NAME,
		},
		.st_mlc_probe = true,
		.st_shub_probe = true,
		.st_power_mode = true,
	},
};

static const struct st_asm330lhhx_fs_table_entry st_asm330lhhx_fs_table[] = {
	[ST_ASM330LHHX_ID_ACC] = {
		.reg = {
			.addr = ST_ASM330LHHX_CTRL1_XL_ADDR,
			.mask = ST_ASM330LHHX_REG_FS_XL_MASK,
		},
		.fs_avl[0] = { ST_ASM330LHHX_ACC_FS_2G_GAIN,  0x00 },
		.fs_avl[1] = { ST_ASM330LHHX_ACC_FS_4G_GAIN,  0x02 },
		.fs_avl[2] = { ST_ASM330LHHX_ACC_FS_8G_GAIN,  0x03 },
		.fs_avl[3] = { ST_ASM330LHHX_ACC_FS_16G_GAIN, 0x01 },
		.size = ST_ASM330LHHX_FS_ACC_LIST_SIZE,
	},
	[ST_ASM330LHHX_ID_GYRO] = {
		.reg = {
			.addr = ST_ASM330LHHX_CTRL2_G_ADDR,
			.mask = ST_ASM330LHHX_REG_FS_G_MASK,
		},
		.fs_avl[0] = { ST_ASM330LHHX_GYRO_FS_125_GAIN,  0x02 },
		.fs_avl[1] = { ST_ASM330LHHX_GYRO_FS_250_GAIN,  0x00 },
		.fs_avl[2] = { ST_ASM330LHHX_GYRO_FS_500_GAIN,  0x04 },
		.fs_avl[3] = { ST_ASM330LHHX_GYRO_FS_1000_GAIN, 0x08 },
		.fs_avl[4] = { ST_ASM330LHHX_GYRO_FS_2000_GAIN, 0x0C },
		.fs_avl[5] = { ST_ASM330LHHX_GYRO_FS_4000_GAIN, 0x01 },
		.size = ST_ASM330LHHX_FS_GYRO_LIST_SIZE,
	},
	[ST_ASM330LHHX_ID_TEMP] = {
		.size = ST_ASM330LHHX_FS_TEMP_LIST_SIZE,
		.fs_avl[0] = { ST_ASM330LHHX_TEMP_FS_GAIN, 0x0 },
	},
};

static const struct st_asm330lhhx_xl_lpf_bw_config_t st_asm330lhhx_xl_bw = {
	.reg = ST_ASM330LHHX_REG_CTRL8_XL_ADDR,
	.mask = ST_ASM330LHHX_REG_HPCF_XL_MASK,
	.size = 9,
	.st_asm330lhhx_xl_lpf_bw[0] = {
		.lpf2_xl_en = 0,
		.div = 2,
		.val = 0,
	},
	.st_asm330lhhx_xl_lpf_bw[1] = {
		.lpf2_xl_en = 1,
		.div = 4,
		.val = 0,
	},
	.st_asm330lhhx_xl_lpf_bw[2] = {
		.lpf2_xl_en = 1,
		.div = 10,
		.val = 1,
	},
	.st_asm330lhhx_xl_lpf_bw[3] = {
		.lpf2_xl_en = 1,
		.div = 20,
		.val = 2,
	},
	.st_asm330lhhx_xl_lpf_bw[4] = {
		.lpf2_xl_en = 1,
		.div = 45,
		.val = 3,
	},
	.st_asm330lhhx_xl_lpf_bw[5] = {
		.lpf2_xl_en = 1,
		.div = 100,
		.val = 4,
	},
	.st_asm330lhhx_xl_lpf_bw[6] = {
		.lpf2_xl_en = 1,
		.div = 200,
		.val = 5,
	},
	.st_asm330lhhx_xl_lpf_bw[7] = {
		.lpf2_xl_en = 1,
		.div = 400,
		.val = 6,
	},
	.st_asm330lhhx_xl_lpf_bw[8] = {
		.lpf2_xl_en = 1,
		.div = 800,
		.val = 7,
	},
};

static const struct st_asm330lhhx_g_lpf_bw_config_t st_asm330lhhx_g_bw = {
	.reg = ST_ASM330LHHX_REG_CTRL6_C_ADDR,
	.mask = ST_ASM330LHHX_REG_FTYPE_MASK,
	.size = 8,
	.ftype = { 0, 1, 2, 3, 4, 5, 6, 7 },
};

static const inline struct iio_mount_matrix *
st_asm330lhhx_get_mount_matrix(const struct iio_dev *iio_dev,
			      const struct iio_chan_spec *chan)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;

	return &hw->orientation;
}

static const struct iio_chan_spec_ext_info st_asm330lhhx_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_ALL, st_asm330lhhx_get_mount_matrix),
	{},
};

#define IIO_CHAN_HW_TIMESTAMP(si) {					\
	.type = IIO_COUNT,						\
	.address = ST_ASM330LHHX_REG_TIMESTAMP0_ADDR,			\
	.scan_index = si,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 64,						\
		.storagebits = 64,					\
		.endianness = IIO_LE,					\
	},								\
}

static const struct iio_chan_spec st_asm330lhhx_acc_channels[] = {
	ST_ASM330LHHX_DATA_CHANNEL(IIO_ACCEL, ST_ASM330LHHX_REG_OUTX_L_A_ADDR,
				1, IIO_MOD_X, 0, 16, 16, 's', st_asm330lhhx_ext_info),
	ST_ASM330LHHX_DATA_CHANNEL(IIO_ACCEL, ST_ASM330LHHX_REG_OUTY_L_A_ADDR,
				1, IIO_MOD_Y, 1, 16, 16, 's', st_asm330lhhx_ext_info),
	ST_ASM330LHHX_DATA_CHANNEL(IIO_ACCEL, ST_ASM330LHHX_REG_OUTZ_L_A_ADDR,
				1, IIO_MOD_Z, 2, 16, 16, 's', st_asm330lhhx_ext_info),
	ST_ASM330LHHX_EVENT_CHANNEL(IIO_ACCEL, flush),

	ST_ASM330LHHX_EVENT_CHANNEL(IIO_ACCEL, freefall),
	ST_ASM330LHHX_EVENT_CHANNEL(IIO_ACCEL, wakeup),
	ST_ASM330LHHX_EVENT_CHANNEL(IIO_ACCEL, 6D),

#if defined(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)
	IIO_CHAN_HW_TIMESTAMP(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
#else /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */
	IIO_CHAN_SOFT_TIMESTAMP(3),
#endif /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */

};

static const struct iio_chan_spec st_asm330lhhx_gyro_channels[] = {
	ST_ASM330LHHX_DATA_CHANNEL(IIO_ANGL_VEL, ST_ASM330LHHX_REG_OUTX_L_G_ADDR,
				1, IIO_MOD_X, 0, 16, 16, 's', st_asm330lhhx_ext_info),
	ST_ASM330LHHX_DATA_CHANNEL(IIO_ANGL_VEL, ST_ASM330LHHX_REG_OUTY_L_G_ADDR,
				1, IIO_MOD_Y, 1, 16, 16, 's', st_asm330lhhx_ext_info),
	ST_ASM330LHHX_DATA_CHANNEL(IIO_ANGL_VEL, ST_ASM330LHHX_REG_OUTZ_L_G_ADDR,
				1, IIO_MOD_Z, 2, 16, 16, 's', st_asm330lhhx_ext_info),
	ST_ASM330LHHX_EVENT_CHANNEL(IIO_ANGL_VEL, flush),

#if defined(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)
	IIO_CHAN_HW_TIMESTAMP(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
#else /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */
	IIO_CHAN_SOFT_TIMESTAMP(3),
#endif /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */

};

static
__maybe_unused const struct iio_chan_spec st_asm330lhhx_temp_channels[] = {
	{
		.type = IIO_TEMP,
		.address = ST_ASM330LHHX_REG_OUT_TEMP_L_ADDR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_OFFSET)
				| BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		}
	},
	ST_ASM330LHHX_EVENT_CHANNEL(IIO_TEMP, flush),

#if defined(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)
	IIO_CHAN_HW_TIMESTAMP(1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
#else /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */
	IIO_CHAN_SOFT_TIMESTAMP(1),
#endif /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */

};

static inline int st_asm330lhhx_get_odr_index(int odr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_asm330lhhx_odr_index); i++)
		if (st_asm330lhhx_odr_index[i] >= odr)
			break;

	if (i == ARRAY_SIZE(st_asm330lhhx_odr_index))
		return -EINVAL;

	return i;
}

static inline int st_asm330lhhx_get_odr_divider_index(int odr_div)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_asm330lhhx_odr_divider_index); i++)
		if (st_asm330lhhx_odr_divider_index[i] >= odr_div)
			break;

	if (i == ARRAY_SIZE(st_asm330lhhx_odr_divider_index))
		return -EINVAL;

	return i;
}

int st_asm330lhhx_of_get_pin(struct st_asm330lhhx_hw *hw, int *pin)
{
	if (!dev_fwnode(hw->dev))
		return -EINVAL;

	return device_property_read_u32(hw->dev, "st,int-pin", pin);
}

int st_asm330lhhx_get_int_reg(struct st_asm330lhhx_hw *hw)
{
	int err = 0, int_pin;

	if (st_asm330lhhx_of_get_pin(hw, &int_pin) < 0) {
		struct st_sensors_platform_data *pdata;
		struct device *dev = hw->dev;

		pdata = (struct st_sensors_platform_data *)dev->platform_data;
		int_pin = pdata ? pdata->drdy_int_pin : 1;
	}

	switch (int_pin) {
	case 1:
		hw->drdy_reg = ST_ASM330LHHX_REG_INT1_CTRL_ADDR;
		hw->embfunc_pg0_irq_reg = ST_ASM330LHHX_REG_MD1_CFG_ADDR;
		break;
	case 2:
		hw->drdy_reg = ST_ASM330LHHX_REG_INT2_CTRL_ADDR;
		hw->embfunc_pg0_irq_reg = ST_ASM330LHHX_REG_MD2_CFG_ADDR;
		break;
	default:
		dev_err(hw->dev, "unsupported interrupt pin\n");
		err = -EINVAL;
		break;
	}

	hw->int_pin = int_pin;

	return err;
}

static int __maybe_unused st_asm330lhhx_bk_regs(struct st_asm330lhhx_hw *hw)
{
	unsigned int data;
	bool restore = 0;
	int i, err = 0;

	mutex_lock(&hw->page_lock);

	for (i = 0; i < ST_ASM330LHHX_SUSPEND_RESUME_REGS; i++) {
		if (st_asm330lhhx_suspend_resume[i].page != FUNC_CFG_ACCESS_0) {
			/* skip if not support mlc */
			if (!hw->settings->st_mlc_probe)
				continue;

			err = regmap_update_bits(hw->regmap,
				     ST_ASM330LHHX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_ASM330LHHX_REG_ACCESS_MASK,
				     FIELD_PREP(ST_ASM330LHHX_REG_ACCESS_MASK,
					st_asm330lhhx_suspend_resume[i].page));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_asm330lhhx_suspend_resume[i].addr);
				break;
			}

			restore = 1;
		}

		err = regmap_read(hw->regmap,
				  st_asm330lhhx_suspend_resume[i].addr,
				  &data);
		if (err < 0) {
			dev_err(hw->dev,
				"failed to save register %02x\n",
				st_asm330lhhx_suspend_resume[i].addr);
			goto out_lock;
		}

		if (restore) {
			err = regmap_update_bits(hw->regmap,
				     ST_ASM330LHHX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_ASM330LHHX_REG_ACCESS_MASK,
				     FIELD_PREP(ST_ASM330LHHX_REG_ACCESS_MASK,
						    FUNC_CFG_ACCESS_0));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_asm330lhhx_suspend_resume[i].addr);
				break;
			}

			restore = 0;
		}

		st_asm330lhhx_suspend_resume[i].val = data;
	}

out_lock:
	mutex_unlock(&hw->page_lock);

	return err;
}

static int __maybe_unused st_asm330lhhx_restore_regs(struct st_asm330lhhx_hw *hw)
{
	bool restore = 0;
	int i, err = 0;

	mutex_lock(&hw->page_lock);

	for (i = 0; i < ST_ASM330LHHX_SUSPEND_RESUME_REGS; i++) {
		if (st_asm330lhhx_suspend_resume[i].page != FUNC_CFG_ACCESS_0) {
			/* skip if not support mlc */
			if (!hw->settings->st_mlc_probe)
				continue;

			err = regmap_update_bits(hw->regmap,
				     ST_ASM330LHHX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_ASM330LHHX_REG_ACCESS_MASK,
				     FIELD_PREP(ST_ASM330LHHX_REG_ACCESS_MASK,
					st_asm330lhhx_suspend_resume[i].page));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_asm330lhhx_suspend_resume[i].addr);
				break;
			}

			restore = 1;
		}

		err = regmap_update_bits(hw->regmap,
					 st_asm330lhhx_suspend_resume[i].addr,
					 st_asm330lhhx_suspend_resume[i].mask,
					 st_asm330lhhx_suspend_resume[i].val);
		if (err < 0) {
			dev_err(hw->dev,
				"failed to update %02x reg\n",
				st_asm330lhhx_suspend_resume[i].addr);
			break;
		}

		if (restore) {
			err = regmap_update_bits(hw->regmap,
				     ST_ASM330LHHX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_ASM330LHHX_REG_ACCESS_MASK,
				     FIELD_PREP(ST_ASM330LHHX_REG_ACCESS_MASK,
						    FUNC_CFG_ACCESS_0));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_asm330lhhx_suspend_resume[i].addr);
				break;
			}

			restore = 0;
		}
	}

	mutex_unlock(&hw->page_lock);

	return err;
}

static int st_asm330lhhx_set_selftest(
				struct st_asm330lhhx_sensor *sensor, int index)
{
	u8 mode, mask;

	switch (sensor->id) {
	case ST_ASM330LHHX_ID_ACC:
		mask = ST_ASM330LHHX_REG_ST_XL_MASK;
		mode = st_asm330lhhx_selftest_table[index].accel_value;
		break;
	case ST_ASM330LHHX_ID_GYRO:
		mask = ST_ASM330LHHX_REG_ST_G_MASK;
		mode = st_asm330lhhx_selftest_table[index].gyro_value;
		break;
	default:
		return -EINVAL;
	}

	return st_asm330lhhx_update_bits_locked(sensor->hw,
					    ST_ASM330LHHX_REG_CTRL5_C_ADDR,
					    mask, mode);
}

static ssize_t st_asm330lhhx_sysfs_get_selftest_available(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%s, %s\n",
			  st_asm330lhhx_selftest_table[1].string_mode,
			  st_asm330lhhx_selftest_table[2].string_mode);
}

static ssize_t st_asm330lhhx_sysfs_get_selftest_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t result;
	char *message = NULL;
	struct st_asm330lhhx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	enum st_asm330lhhx_sensor_id id = sensor->id;

	if (id != ST_ASM330LHHX_ID_ACC &&
	    id != ST_ASM330LHHX_ID_GYRO)
		return -EINVAL;

	result = sensor->selftest_status;
	if (result == 0)
		message = "na";
	else if (result < 0)
		message = "fail";
	else if (result > 0)
		message = "pass";

	return sysfs_emit(buf, "%s\n", message);
}

static __maybe_unused int st_asm330lhhx_reg_access(struct iio_dev *iio_dev,
				 unsigned int reg, unsigned int writeval,
				 unsigned int *readval)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
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

static int st_asm330lhhx_set_page_0(struct st_asm330lhhx_hw *hw)
{
	return regmap_write(hw->regmap,
			    ST_ASM330LHHX_REG_FUNC_CFG_ACCESS_ADDR, 0);
}

static int st_asm330lhhx_check_whoami(struct st_asm330lhhx_hw *hw,
				      int id)
{
	int err, i;
	int data;

	for (i = 0; i < ARRAY_SIZE(st_asm330lhhx_sensor_settings); i++) {
			if (st_asm330lhhx_sensor_settings[i].id.name &&
			    st_asm330lhhx_sensor_settings[i].id.hw_id == id)
				break;
	}

	if (i == ARRAY_SIZE(st_asm330lhhx_sensor_settings)) {
		dev_err(hw->dev, "unsupported hw id [%02x]\n", id);

		return -ENODEV;
	}

	err = regmap_read(hw->regmap, ST_ASM330LHHX_REG_WHOAMI_ADDR, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != ST_ASM330LHHX_WHOAMI_VAL) {
		dev_err(hw->dev, "unsupported whoami [%02x]\n", data);
		return -ENODEV;
	}

	hw->settings = &st_asm330lhhx_sensor_settings[i];
	hw->fs_table = &st_asm330lhhx_fs_table[0];

	return 0;
}

static int st_asm330lhhx_get_odr_calibration(struct st_asm330lhhx_hw *hw)
{
	s64 odr_calib;
	int data;
	int err;

	err = regmap_read(hw->regmap, ST_ASM330LHHX_INTERNAL_FREQ_FINE,
			  &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read %d register\n",
				ST_ASM330LHHX_INTERNAL_FREQ_FINE);
		return err;
	}

	odr_calib = ((s8)data * 37500) / 1000;
	hw->ts_delta_ns = ST_ASM330LHHX_TS_DELTA_NS - odr_calib;

	dev_info(hw->dev, "Freq Fine %lld (ts %lld)\n", odr_calib, hw->ts_delta_ns);

	return 0;
}

static int st_asm330lhhx_set_full_scale(struct st_asm330lhhx_sensor *sensor,
				     u32 gain)
{
	enum st_asm330lhhx_sensor_id id = sensor->id;
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int i, err;
	u8 val;

	/* for other sensors gain is fixed */
	if (id > ST_ASM330LHHX_ID_ACC)
		return 0;

	for (i = 0; i < st_asm330lhhx_fs_table[id].size; i++)
		if (st_asm330lhhx_fs_table[id].fs_avl[i].gain >= gain)
			break;

	if (i == st_asm330lhhx_fs_table[id].size)
		return -EINVAL;

	val = st_asm330lhhx_fs_table[id].fs_avl[i].val;
	err = regmap_update_bits(hw->regmap,
				 st_asm330lhhx_fs_table[id].reg.addr,
				 st_asm330lhhx_fs_table[id].reg.mask,
				 ST_ASM330LHHX_SHIFT_VAL(val,
					  st_asm330lhhx_fs_table[id].reg.mask));
	if (err < 0)
		return err;

	sensor->gain = st_asm330lhhx_fs_table[id].fs_avl[i].gain;

	return 0;
}

int st_asm330lhhx_get_odr_val(enum st_asm330lhhx_sensor_id id, int odr,
			      int uodr, int *podr, int *puodr, u8 *val)
{
	int required_odr = ST_ASM330LHHX_ODR_EXPAND(odr, uodr);
	int sensor_odr;
	int i;

	for (i = 0; i < st_asm330lhhx_odr_table[id].size; i++) {
		sensor_odr = ST_ASM330LHHX_ODR_EXPAND(
				st_asm330lhhx_odr_table[id].odr_avl[i].hz,
				st_asm330lhhx_odr_table[id].odr_avl[i].uhz);
		if (sensor_odr >= required_odr)
			break;
	}

	if (i == st_asm330lhhx_odr_table[id].size)
		return -EINVAL;

	*val = st_asm330lhhx_odr_table[id].odr_avl[i].val;

	if (podr && puodr) {
		*podr = st_asm330lhhx_odr_table[id].odr_avl[i].hz;
		*puodr = st_asm330lhhx_odr_table[id].odr_avl[i].uhz;
	}

	return 0;
}

int __maybe_unused
st_asm330lhhx_get_odr_from_reg(enum st_asm330lhhx_sensor_id id,
			       u8 reg_val, u16 *podr, u32 *puodr)
{
	int i;

	for (i = 0; i < st_asm330lhhx_odr_table[id].size; i++) {
		if (reg_val == st_asm330lhhx_odr_table[id].odr_avl[i].val)
			break;
	}

	if (i == st_asm330lhhx_odr_table[id].size)
		return -EINVAL;

	*podr = st_asm330lhhx_odr_table[id].odr_avl[i].hz;
	*puodr = st_asm330lhhx_odr_table[id].odr_avl[i].uhz;

	return 0;
}

int st_asm330lhhx_get_batch_val(struct st_asm330lhhx_sensor *sensor,
			       int odr, int uodr, u8 *val)
{
	int required_odr = ST_ASM330LHHX_ODR_EXPAND(odr, uodr);
	enum st_asm330lhhx_sensor_id id = sensor->id;
	int sensor_odr;
	int i;

	for (i = 0; i < st_asm330lhhx_odr_table[id].size; i++) {
		sensor_odr = ST_ASM330LHHX_ODR_EXPAND(
				st_asm330lhhx_odr_table[id].odr_avl[i].hz,
				st_asm330lhhx_odr_table[id].odr_avl[i].uhz);
		if (sensor_odr >= required_odr)
			break;
	}

	if (i == st_asm330lhhx_odr_table[id].size)
		return -EINVAL;

	*val = st_asm330lhhx_odr_table[id].odr_avl[i].batch_val;

	return 0;
}

static int st_asm330lhhx_update_odr_fsm(struct st_asm330lhhx_hw *hw,
					enum st_asm330lhhx_sensor_id id,
					enum st_asm330lhhx_sensor_id id_req,
					int val, int delay)
{
	bool fsm_running = st_asm330lhhx_fsm_running(hw);
	bool mlc_running = st_asm330lhhx_mlc_running(hw);
	int ret = 0;
	int status;

	if (fsm_running || mlc_running ||
	    (id_req > ST_ASM330LHHX_ID_MLC)) {
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
		ret = st_asm330lhhx_set_page_access(hw, true,
				       ST_ASM330LHHX_REG_FUNC_CFG_MASK);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_read(hw->regmap,
				  ST_ASM330LHHX_EMB_FUNC_EN_B_ADDR,
				  &status);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 ST_ASM330LHHX_PAGE_SEL_ADDR,
					 BIT(1), FIELD_PREP(BIT(1), 1));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
			ST_ASM330LHHX_EMB_FUNC_EN_B_ADDR,
			ST_ASM330LHHX_FSM_EN_MASK,
			FIELD_PREP(ST_ASM330LHHX_FSM_EN_MASK, 0));
		if (ret < 0)
			goto unlock_page;

		if (st_asm330lhhx_mlc_running(hw)) {
			ret = regmap_update_bits(hw->regmap,
				ST_ASM330LHHX_EMB_FUNC_EN_B_ADDR,
				ST_ASM330LHHX_MLC_EN_MASK,
				FIELD_PREP(ST_ASM330LHHX_MLC_EN_MASK, 0));
			if (ret < 0)
				goto unlock_page;
		}

		ret = regmap_update_bits(hw->regmap,
			ST_ASM330LHHX_REG_EMB_FUNC_INIT_B_ADDR,
			ST_ASM330LHHX_MLC_INIT_MASK,
			FIELD_PREP(ST_ASM330LHHX_MLC_INIT_MASK, 0));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
			ST_ASM330LHHX_REG_EMB_FUNC_INIT_B_ADDR,
			ST_ASM330LHHX_FSM_INIT_MASK,
			FIELD_PREP(ST_ASM330LHHX_FSM_INIT_MASK, 0));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 ST_ASM330LHHX_PAGE_SEL_ADDR,
					 BIT(1), FIELD_PREP(BIT(1), 0));
		if (ret < 0)
			goto unlock_page;

		ret = st_asm330lhhx_set_page_access(hw, false,
				      ST_ASM330LHHX_REG_FUNC_CFG_MASK);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
			st_asm330lhhx_odr_table[id].reg.addr,
			st_asm330lhhx_odr_table[id].reg.mask,
			ST_ASM330LHHX_SHIFT_VAL(val,
				st_asm330lhhx_odr_table[id].reg.mask));
		if (ret < 0)
			goto unlock_page;

		usleep_range(delay, delay + (delay / 10));

		st_asm330lhhx_set_page_access(hw, true,
				       ST_ASM330LHHX_REG_FUNC_CFG_MASK);

		ret = regmap_write(hw->regmap,
				   ST_ASM330LHHX_EMB_FUNC_EN_B_ADDR,
				   status);
unlock_page:
		st_asm330lhhx_set_page_access(hw, false,
				       ST_ASM330LHHX_REG_FUNC_CFG_MASK);
		mutex_unlock(&hw->page_lock);
	} else {
		ret = st_asm330lhhx_update_bits_locked(hw,
				st_asm330lhhx_odr_table[id].reg.addr,
				st_asm330lhhx_odr_table[id].reg.mask,
				val);
	}

	return ret;
}

static int st_asm330lhhx_update_decimator(struct st_asm330lhhx_sensor *sensor,
					  int odr)
{
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int odr_index, odr_div_index, ret = 0;

	if (hw->enable_drdy_mask) {
		sensor->decimator = 0;

		return 0;
	}

	odr_index = st_asm330lhhx_get_odr_index(odr);
	if (odr_index < 0)
		return odr_index;

	mutex_lock(&hw->fifo_lock);

	switch (sensor->id) {
	case ST_ASM330LHHX_ID_ACC:
		odr_div_index = st_asm330lhhx_get_odr_divider_index(hw->xl_odr_div);
		if (odr_div_index < 0) {
			ret = odr_div_index;

			goto unlock;
		}

		sensor->discard_samples = st_asm330lhhx_lpf_discard_table[sensor->id].samples_to_discard[odr_index] +
				st_asm330lhhx_lpf_discard_table[sensor->id].settling_samples[odr_div_index][odr_index];
		break;
	case ST_ASM330LHHX_ID_GYRO:
		sensor->discard_samples = st_asm330lhhx_lpf_discard_table[sensor->id].samples_to_discard[odr_index] +
				st_asm330lhhx_lpf_discard_table[sensor->id].settling_samples[hw->g_ftype][odr_index];
		break;
	default:
		break;
	}

unlock:
	mutex_unlock(&hw->fifo_lock);

	return ret;
}

int st_asm330lhhx_set_odr(struct st_asm330lhhx_sensor *sensor,
			  int req_odr, int req_uodr)
{
	enum st_asm330lhhx_sensor_id id_req = sensor->id;
	enum st_asm330lhhx_sensor_id id = sensor->id;
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int err, delay;
	u8 val = 0;

	switch (id) {
	case ST_ASM330LHHX_ID_EXT0:
	case ST_ASM330LHHX_ID_EXT1:
	case ST_ASM330LHHX_ID_MLC_0:
	case ST_ASM330LHHX_ID_MLC_1:
	case ST_ASM330LHHX_ID_MLC_2:
	case ST_ASM330LHHX_ID_MLC_3:
	case ST_ASM330LHHX_ID_MLC_4:
	case ST_ASM330LHHX_ID_MLC_5:
	case ST_ASM330LHHX_ID_MLC_6:
	case ST_ASM330LHHX_ID_MLC_7:
	case ST_ASM330LHHX_ID_FSM_0:
	case ST_ASM330LHHX_ID_FSM_1:
	case ST_ASM330LHHX_ID_FSM_2:
	case ST_ASM330LHHX_ID_FSM_3:
	case ST_ASM330LHHX_ID_FSM_4:
	case ST_ASM330LHHX_ID_FSM_5:
	case ST_ASM330LHHX_ID_FSM_6:
	case ST_ASM330LHHX_ID_FSM_7:
	case ST_ASM330LHHX_ID_FSM_8:
	case ST_ASM330LHHX_ID_FSM_9:
	case ST_ASM330LHHX_ID_FSM_10:
	case ST_ASM330LHHX_ID_FSM_11:
	case ST_ASM330LHHX_ID_FSM_12:
	case ST_ASM330LHHX_ID_FSM_13:
	case ST_ASM330LHHX_ID_FSM_14:
	case ST_ASM330LHHX_ID_FSM_15:
	case ST_ASM330LHHX_ID_TEMP:
	case ST_ASM330LHHX_ID_ACC: {
		int odr;
		int i;

		id = ST_ASM330LHHX_ID_ACC;
		for (i = ST_ASM330LHHX_ID_ACC; i < ST_ASM330LHHX_ID_MAX; i++) {
			if (!hw->iio_devs[i])
				continue;

			if (i == sensor->id)
				continue;

			odr = st_asm330lhhx_check_odr_dependency(hw, req_odr,
								req_uodr, i);
			if (odr != req_odr) {
				/* device already configured */
				return 0;
			}
		}
		break;
	}
	case ST_ASM330LHHX_ID_GYRO:
		break;
	default:
		return 0;
	}

	err = st_asm330lhhx_get_odr_val(id, req_odr, req_uodr, NULL,
				       NULL, &val);
	if (err < 0)
		return err;

	err = st_asm330lhhx_update_decimator(iio_priv(hw->iio_devs[id]),
					     req_odr);
	if (err < 0)
		return err;

	/* check if sensor supports power mode setting */
	if (sensor->pm != ST_ASM330LHHX_NO_MODE &&
	    hw->settings->st_power_mode) {
		err = regmap_update_bits(hw->regmap,
				st_asm330lhhx_odr_table[id].pm.addr,
				st_asm330lhhx_odr_table[id].pm.mask,
				ST_ASM330LHHX_SHIFT_VAL(sensor->pm,
				 st_asm330lhhx_odr_table[id].pm.mask));
		if (err < 0)
			return err;
	}

	delay = req_odr > 0 ? 4000000 / req_odr : 0;

	return st_asm330lhhx_update_odr_fsm(hw, id, id_req, val, delay);
}

int st_asm330lhhx_sensor_set_enable(struct st_asm330lhhx_sensor *sensor,
				 bool enable)
{
	int uodr = enable ? sensor->uodr : 0;
	int odr = enable ? sensor->odr : 0;
	int err;

	err = st_asm330lhhx_set_odr(sensor, odr, uodr);
	if (err < 0)
		return err;

	if (enable)
		sensor->hw->enable_mask |= BIT_ULL(sensor->id);
	else
		sensor->hw->enable_mask &= ~BIT_ULL(sensor->id);

	return 0;
}

static int st_asm330lhhx_read_oneshot(struct st_asm330lhhx_sensor *sensor,
				   u8 addr, int *val)
{
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int err, delay;
	__le16 data;

	err = st_asm330lhhx_sensor_set_enable(sensor, true);
	if (err < 0)
		return err;

	/* Use big delay for data valid because of drdy mask enabled */
	delay = 10000000 / sensor->odr;
	usleep_range(delay, 2 * delay);

	err = st_asm330lhhx_read_locked(hw, addr,
				    &data,
				    sizeof(data));
	if (err < 0)
		return err;

	err = st_asm330lhhx_sensor_set_enable(sensor, false);

	*val = (s16)le16_to_cpu(data);

	return IIO_VAL_INT;
}

static int st_asm330lhhx_read_raw(struct iio_dev *iio_dev,
			       struct iio_chan_spec const *ch,
			       int *val, int *val2, long mask)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			break;

		ret = st_asm330lhhx_read_oneshot(sensor, ch->address, val);
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
			*val2 = ST_ASM330LHHX_TEMP_GAIN;
			ret = IIO_VAL_FRACTIONAL;
			break;
		case IIO_ACCEL:
		case IIO_ANGL_VEL:
			*val = 0;
			*val2 = sensor->gain;
			ret = IIO_VAL_INT_PLUS_NANO;
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

static int st_asm330lhhx_write_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct st_asm330lhhx_sensor *s = iio_priv(iio_dev);
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = iio_device_claim_direct_mode(iio_dev);
		if (err)
			return err;

		err = st_asm330lhhx_set_full_scale(s, val2);
		if (err) {
			iio_device_release_direct_mode(iio_dev);
			return err;
		}

		/* some events depends on xl full scale */
		if (chan->type == IIO_ACCEL)
			err = st_asm330lhhx_update_threshold_events(s->hw);
		iio_device_release_direct_mode(iio_dev);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		int todr, tuodr;
		u8 data;

		err = st_asm330lhhx_get_odr_val(s->id, val, val2,
						 &todr, &tuodr, &data);
		if (!err) {
			s->odr = todr;
			s->uodr = tuodr;

			/*
			 * VTS test testSamplingRateHotSwitchOperation not
			 * toggle the enable status of sensor after changing
			 * the ODR -> force it
			 */
			if (s->hw->enable_mask & BIT_ULL(s->id)) {
				switch (s->id) {
				case ST_ASM330LHHX_ID_GYRO:
				case ST_ASM330LHHX_ID_ACC:
					err = st_asm330lhhx_set_odr(s, s->odr, s->uodr);
					if (err < 0)
						break;

					st_asm330lhhx_update_batching(iio_dev, 1);

					/* some events depends on xl odr */
					if (chan->type == IIO_ACCEL)
						st_asm330lhhx_update_duration_events(s->hw);
					break;
				default:
					break;
				}
			}
		}
		break;
	}
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static ssize_t
st_asm330lhhx_sysfs_sampling_freq_avail(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	enum st_asm330lhhx_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_asm330lhhx_odr_table[id].size; i++) {
		/* skip zero */
		if (st_asm330lhhx_odr_table[id].odr_avl[i].hz == 0 &&
		    st_asm330lhhx_odr_table[id].odr_avl[i].uhz == 0)
			continue;

		len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%06d ",
				 st_asm330lhhx_odr_table[id].odr_avl[i].hz,
				 st_asm330lhhx_odr_table[id].odr_avl[i].uhz);
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_asm330lhhx_sysfs_scale_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	enum st_asm330lhhx_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_asm330lhhx_fs_table[id].size; i++) {
		if (sensor->id != ST_ASM330LHHX_ID_TEMP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "0.%09u ",
					 st_asm330lhhx_fs_table[id].fs_avl[i].gain);
		} else {
			int hi, low;

			hi = (int)(st_asm330lhhx_fs_table[id].fs_avl[i].gain / 1000);
			low = (int)(st_asm330lhhx_fs_table[id].fs_avl[i].gain % 1000);
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%d ",
						     hi, low);
		}
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t
st_asm330lhhx_sysfs_get_power_mode_avail(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int i, len = 0;

	/* check for supported feature */
	if (hw->settings->st_power_mode) {
		for (i = 0; i < ARRAY_SIZE(st_asm330lhhx_power_mode); i++) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%s ",
					 st_asm330lhhx_power_mode[i].string_mode);
		}
	} else {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s ",
				 st_asm330lhhx_power_mode[0].string_mode);
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t
st_asm330lhhx_get_power_mode(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);

	return sysfs_emit(buf, "%s\n",
		      st_asm330lhhx_power_mode[sensor->pm].string_mode);
}

static ssize_t
st_asm330lhhx_set_power_mode(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int err, i;

	/* check for supported feature */
	if (!hw->settings->st_power_mode)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(st_asm330lhhx_power_mode); i++) {
		if (strncmp(buf, st_asm330lhhx_power_mode[i].string_mode,
		    strlen(st_asm330lhhx_power_mode[i].string_mode)) == 0)
			break;
	}

	if (i == ARRAY_SIZE(st_asm330lhhx_power_mode))
		return -EINVAL;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	/* update power mode */
	sensor->pm = st_asm330lhhx_power_mode[i].val;

	iio_device_release_direct_mode(iio_dev);

	return size;
}

static int st_asm330lhhx_selftest_sensor(struct st_asm330lhhx_sensor *sensor,
					int test)
{
	int x_selftest = 0, y_selftest = 0, z_selftest = 0;
	int x = 0, y = 0, z = 0, try_count = 0;
	u8 i, status, n = 0;
	u8 reg, bitmask;
	int ret, delay, data_delay = 100000;
	u8 raw_data[6];

	switch(sensor->id) {
	case ST_ASM330LHHX_ID_ACC:
		reg = ST_ASM330LHHX_REG_OUTX_L_A_ADDR;
		bitmask = ST_ASM330LHHX_REG_STATUS_XLDA;
		data_delay = 50000;
		break;
	case ST_ASM330LHHX_ID_GYRO:
		reg = ST_ASM330LHHX_REG_OUTX_L_G_ADDR;
		bitmask = ST_ASM330LHHX_REG_STATUS_GDA;
		break;
	default:
		return -EINVAL;
	}

	/* reset selftest_status */
	sensor->selftest_status = -1;

	/* set selftest normal mode */
	ret = st_asm330lhhx_set_selftest(sensor, 0);
	if (ret < 0)
		return ret;

	ret = st_asm330lhhx_sensor_set_enable(sensor, true);
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
		ret = st_asm330lhhx_read_locked(sensor->hw,
						ST_ASM330LHHX_REG_STATUS_ADDR,
						&status, sizeof(status));
		if (ret < 0)
			goto selftest_failure;

		if (status & bitmask) {
			st_asm330lhhx_read_locked(sensor->hw, reg,
						  raw_data, sizeof(raw_data));
			break;
		}

		try_count++;
	}

	if (try_count == 3)
		goto selftest_failure;

	/* for 5 times, after checking status bit, read the output registers */
	for (i = 0; i < 5; i++) {
		try_count = 0;
		while (try_count < 3) {
			usleep_range(delay, delay + delay/10);
			ret = st_asm330lhhx_read_locked(sensor->hw,
						ST_ASM330LHHX_REG_STATUS_ADDR,
						&status, sizeof(status));
			if (ret < 0)
				goto selftest_failure;

			if (status & bitmask) {
				ret = st_asm330lhhx_read_locked(sensor->hw,
							    reg, raw_data,
							    sizeof(raw_data));
				if (ret < 0)
					goto selftest_failure;

				/*
				 * for 5 times, after checking status bit,
				 * read the output registers
				 */
				x += ((s16)*(u16 *)&raw_data[0]) / 5;
				y += ((s16)*(u16 *)&raw_data[2]) / 5;
				z += ((s16)*(u16 *)&raw_data[4]) / 5;
				n++;

				break;
			} else {
				try_count++;
			}
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
	st_asm330lhhx_set_selftest(sensor, test);

	/* wait for stable output */
	usleep_range(data_delay, data_delay + data_delay / 100);

	try_count = 0;

	/* after enabled the sensor trash first sample */
	while (try_count < 3) {
		usleep_range(delay, delay + delay/10);
		ret = st_asm330lhhx_read_locked(sensor->hw,
						ST_ASM330LHHX_REG_STATUS_ADDR,
						&status, sizeof(status));
		if (ret < 0)
			goto selftest_failure;

		if (status & bitmask) {
			st_asm330lhhx_read_locked(sensor->hw, reg,
						  raw_data, sizeof(raw_data));
			break;
		}

		try_count++;
	}

	if (try_count == 3)
		goto selftest_failure;

	/* for 5 times, after checking status bit, read the output registers */
	for (i = 0; i < 5; i++) {
		try_count = 0;
		while (try_count < 3) {
			usleep_range(delay, delay + delay/10);
			ret = st_asm330lhhx_read_locked(sensor->hw,
						ST_ASM330LHHX_REG_STATUS_ADDR,
						&status, sizeof(status));
			if (ret < 0)
				goto selftest_failure;

			if (status & bitmask) {
				ret = st_asm330lhhx_read_locked(sensor->hw,
							      reg, raw_data,
							      sizeof(raw_data));
				if (ret < 0)
					goto selftest_failure;

				x_selftest += ((s16)*(u16 *)&raw_data[0]) / 5;
				y_selftest += ((s16)*(u16 *)&raw_data[2]) / 5;
				z_selftest += ((s16)*(u16 *)&raw_data[4]) / 5;
				n++;

				break;
			} else {
				try_count++;
			}
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
	st_asm330lhhx_set_selftest(sensor, 0);

	return st_asm330lhhx_sensor_set_enable(sensor, false);
}

static ssize_t st_asm330lhhx_sysfs_start_selftest(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	enum st_asm330lhhx_sensor_id id = sensor->id;
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int ret, test;
	u32 gain;

	if (id != ST_ASM330LHHX_ID_ACC &&
	    id != ST_ASM330LHHX_ID_GYRO)
		return -EINVAL;

	for (test = 0; test < ARRAY_SIZE(st_asm330lhhx_selftest_table); test++) {
		if (strncmp(buf, st_asm330lhhx_selftest_table[test].string_mode,
			strlen(st_asm330lhhx_selftest_table[test].string_mode)) == 0)
			break;
	}

	if (test == ARRAY_SIZE(st_asm330lhhx_selftest_table))
		return -EINVAL;

	ret = iio_device_claim_direct_mode(iio_dev);
	if (ret)
		return ret;

	/* self test mode unavailable if sensor enabled */
	if (hw->enable_mask & BIT_ULL(id)) {
		ret = -EBUSY;

		goto out_claim;
	}

	st_asm330lhhx_bk_regs(hw);

	/* disable FIFO watermak interrupt */
	if (hw->irq > 0) {
		/* disable FIFO watermak interrupt */
		ret = st_asm330lhhx_get_int_reg(hw);
		if (ret < 0)
			goto restore_regs;

		ret = st_asm330lhhx_update_bits_locked(hw, hw->drdy_reg,
					     ST_ASM330LHHX_REG_INT_FIFO_TH_MASK,
					     0);
		if (ret < 0)
			goto restore_regs;
	}

	gain = sensor->gain;
	if (id == ST_ASM330LHHX_ID_ACC) {
		/* set BDU = 1, FS = 4 g, ODR = 52 Hz */
		st_asm330lhhx_set_full_scale(sensor,
					    ST_ASM330LHHX_ACC_FS_4G_GAIN);
		st_asm330lhhx_set_odr(sensor, 52, 0);
		st_asm330lhhx_selftest_sensor(sensor, test);

		/* restore full scale after test */
		st_asm330lhhx_set_full_scale(sensor, gain);
	} else {
		/* set BDU = 1, ODR = 208 Hz, FS = 2000 dps */
		st_asm330lhhx_set_full_scale(sensor,
					    ST_ASM330LHHX_GYRO_FS_2000_GAIN);
		/* before enable gyro add 150 ms delay when gyro self-test */
		usleep_range(150000, 151000);

		st_asm330lhhx_set_odr(sensor, 208, 0);
		st_asm330lhhx_selftest_sensor(sensor, test);

		/* restore full scale after test */
		st_asm330lhhx_set_full_scale(sensor, gain);
	}

restore_regs:
	st_asm330lhhx_restore_regs(hw);

out_claim:
	iio_device_release_direct_mode(iio_dev);

	return size;
}

ssize_t st_asm330lhhx_get_module_id(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;

	return scnprintf(buf, PAGE_SIZE, "%u\n", hw->module_id);
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_asm330lhhx_sysfs_sampling_freq_avail);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_asm330lhhx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_anglvel_scale_available, 0444,
		       st_asm330lhhx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_asm330lhhx_get_max_watermark, NULL, 0);
static IIO_DEVICE_ATTR(in_temp_scale_available, 0444,
		       st_asm330lhhx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, 0200, NULL, st_asm330lhhx_flush_fifo, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644, st_asm330lhhx_get_watermark,
		       st_asm330lhhx_set_watermark, 0);

static IIO_DEVICE_ATTR(power_mode_available, 0444,
		       st_asm330lhhx_sysfs_get_power_mode_avail, NULL, 0);
static IIO_DEVICE_ATTR(power_mode, 0644,
		       st_asm330lhhx_get_power_mode,
		       st_asm330lhhx_set_power_mode, 0);

static IIO_DEVICE_ATTR(selftest_available, 0444,
		       st_asm330lhhx_sysfs_get_selftest_available,
		       NULL, 0);
static IIO_DEVICE_ATTR(selftest, 0644,
		       st_asm330lhhx_sysfs_get_selftest_status,
		       st_asm330lhhx_sysfs_start_selftest, 0);
static IIO_DEVICE_ATTR(module_id, 0444, st_asm330lhhx_get_module_id, NULL, 0);

static
ssize_t __maybe_unused st_asm330lhhx_get_discharded_samples(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	ret = sysfs_emit(buf, "%d\n", sensor->discharged_samples);

	/* reset counter */
	sensor->discharged_samples = 0;

	return ret;
}

static int st_asm330lhhx_write_raw_get_fmt(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan, long mask)
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

static
ssize_t __maybe_unused st_asm330lhhx_get_wakeup_status(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int len;

	len = scnprintf(buf, PAGE_SIZE, "%x\n", hw->wakeup_status);

	/* reset status */
	hw->wakeup_status = 0;

	return len;
}

static ssize_t
st_asm330lhhx_sysfs_read_enable_wakeup(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;

	return scnprintf(buf, PAGE_SIZE, "%d\n", (int)hw->wakeup_source);

}

static ssize_t
st_asm330lhhx_sysfs_write_enable_wakeup(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int err;
	u32 val;

	err = kstrtouint(buf, 10, &val);
	if (err < 0)
		return err;

	if (val > 1)
		return -EINVAL;

	hw->wakeup_source = !!val;

	return size;
}

static IIO_DEVICE_ATTR(discharded_samples, 0444,
		       st_asm330lhhx_get_discharded_samples, NULL, 0);
static IIO_DEVICE_ATTR(wakeup_status, 0444,
		       st_asm330lhhx_get_wakeup_status, NULL, 0);
static IIO_DEVICE_ATTR(enable_wakeup, 0644,
		       st_asm330lhhx_sysfs_read_enable_wakeup,
		       st_asm330lhhx_sysfs_write_enable_wakeup, 0);

static struct attribute *st_asm330lhhx_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_power_mode_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_power_mode.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,

#ifdef ST_ASM330LHHX_DEBUG_DISCHARGE
	&iio_dev_attr_discharded_samples.dev_attr.attr,
#endif /* ST_ASM330LHHX_DEBUG_DISCHARGE */

	&iio_dev_attr_wakeup_status.dev_attr.attr,
	&iio_dev_attr_enable_wakeup.dev_attr.attr,

	NULL,
};

static const struct attribute_group st_asm330lhhx_acc_attribute_group = {
	.attrs = st_asm330lhhx_acc_attributes,
};

static const struct iio_info st_asm330lhhx_acc_info = {
	.attrs = &st_asm330lhhx_acc_attribute_group,
	.read_raw = st_asm330lhhx_read_raw,
	.write_raw = st_asm330lhhx_write_raw,
	.write_raw_get_fmt = st_asm330lhhx_write_raw_get_fmt,
	.read_event_config = st_asm330lhhx_read_event_config,
	.write_event_config = st_asm330lhhx_write_event_config,
	.write_event_value = st_asm330lhhx_write_event_value,
	.read_event_value = st_asm330lhhx_read_event_value,

#ifdef CONFIG_DEBUG_FS
	.debugfs_reg_access = &st_asm330lhhx_reg_access,
#endif /* CONFIG_DEBUG_FS */

};

static struct attribute *st_asm330lhhx_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_power_mode_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_power_mode.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,

#ifdef ST_ASM330LHHX_DEBUG_DISCHARGE
	&iio_dev_attr_discharded_samples.dev_attr.attr,
#endif /* ST_ASM330LHHX_DEBUG_DISCHARGE */

	NULL,
};

static const struct attribute_group st_asm330lhhx_gyro_attribute_group = {
	.attrs = st_asm330lhhx_gyro_attributes,
};

static const struct iio_info st_asm330lhhx_gyro_info = {
	.attrs = &st_asm330lhhx_gyro_attribute_group,
	.read_raw = st_asm330lhhx_read_raw,
	.write_raw = st_asm330lhhx_write_raw,
	.write_raw_get_fmt = st_asm330lhhx_write_raw_get_fmt,
};

static struct attribute *st_asm330lhhx_temp_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_temp_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_asm330lhhx_temp_attribute_group = {
	.attrs = st_asm330lhhx_temp_attributes,
};

static const struct iio_info st_asm330lhhx_temp_info = {
	.attrs = &st_asm330lhhx_temp_attribute_group,
	.read_raw = st_asm330lhhx_read_raw,
	.write_raw = st_asm330lhhx_write_raw,
	.write_raw_get_fmt = st_asm330lhhx_write_raw_get_fmt,
};

static const unsigned long st_asm330lhhx_available_scan_masks[] = {

#if defined(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)
	GENMASK(3, 0), 0x0
#else /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */
	GENMASK(2, 0), 0x0
#endif /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */

};

static const unsigned long st_asm330lhhx_temp_available_scan_masks[] = {

#if defined(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)
	GENMASK(1, 0), 0x0
#else /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */
	BIT(0), 0x0
#endif /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */

};

static int st_asm330lhhx_init_xl_filters(struct st_asm330lhhx_hw *hw)
{
	int div;
	int err;
	int i;

	err = device_property_read_u32(hw->dev, "st,xl_lpf_div", &div);
	if (err < 0) {
		/* if st,xl_lpf_div not available disable XL LPF2 */
		err = regmap_update_bits(hw->regmap,
					 ST_ASM330LHHX_CTRL1_XL_ADDR,
					 ST_ASM330LHHX_REG_LPF2_XL_EN_MASK,
					 FIELD_PREP(ST_ASM330LHHX_REG_LPF2_XL_EN_MASK, 0));
		return err < 0 ? err : 0;
	}

	for (i = 0; i < st_asm330lhhx_xl_bw.size; i++) {
		if (st_asm330lhhx_xl_bw.st_asm330lhhx_xl_lpf_bw[i].div >= div)
			break;
	}

	if (i == st_asm330lhhx_xl_bw.size)
		return -EINVAL;

	/* set XL LPF2 BW */
	err = regmap_update_bits(hw->regmap,
				 st_asm330lhhx_xl_bw.reg,
				 st_asm330lhhx_xl_bw.mask,
				 ST_ASM330LHHX_SHIFT_VAL(st_asm330lhhx_xl_bw.st_asm330lhhx_xl_lpf_bw[i].val,
							 st_asm330lhhx_xl_bw.mask));
	if (err < 0)
		return err;

	/* enable/disable LPF2 filter selection */
	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_CTRL1_XL_ADDR,
				 ST_ASM330LHHX_REG_LPF2_XL_EN_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_LPF2_XL_EN_MASK,
					    st_asm330lhhx_xl_bw.st_asm330lhhx_xl_lpf_bw[i].lpf2_xl_en));

	if (err < 0)
		return err;

	hw->enable_drdy_mask = false;
	hw->xl_odr_div = div;

	return 0;
}

static int st_asm330lhhx_init_g_filters(struct st_asm330lhhx_hw *hw)
{
	int ftype;
	int err;
	int i;

	err = device_property_read_u32(hw->dev, "st,g_lpf_ftype", &ftype);
	if (err < 0) {
		/* disable LPF1 filter if st,g_lpf_ftype not available */
		return regmap_update_bits(hw->regmap,
					  ST_ASM330LHHX_REG_CTRL4_C_ADDR,
					  ST_ASM330LHHX_REG_LPF1_SEL_G_MASK,
					  FIELD_PREP(ST_ASM330LHHX_REG_LPF1_SEL_G_MASK, 0));
	}

	for (i = 0; i < st_asm330lhhx_g_bw.size; i++) {
		if (st_asm330lhhx_g_bw.ftype[i] >= ftype)
			break;
	}

	if (i == st_asm330lhhx_g_bw.size)
		return -EINVAL;

	/* set G LPF1 FTYPE */
	err = regmap_update_bits(hw->regmap,
				 st_asm330lhhx_g_bw.reg,
				 st_asm330lhhx_g_bw.mask,
				 ST_ASM330LHHX_SHIFT_VAL(st_asm330lhhx_g_bw.ftype[i],
							 st_asm330lhhx_g_bw.mask));
	if (err < 0)
		return err;

	/* enable LPF1 filter selection if st,g_lpf_ftype available */
	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_CTRL4_C_ADDR,
				 ST_ASM330LHHX_REG_LPF1_SEL_G_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_LPF1_SEL_G_MASK, 1));
	if (err < 0)
		return err;

	hw->enable_drdy_mask = false;
	hw->g_ftype = ftype;

	return 0;
}

static int st_asm330lhhx_reset_device(struct st_asm330lhhx_hw *hw)
{
	int err;

	/* set configuration bit */
	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_CTRL9_XL_ADDR,
				 ST_ASM330LHHX_REG_DEVICE_CONF_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_DEVICE_CONF_MASK, 1));
	if (err < 0)
		return err;

	/* sw reset */
	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_CTRL3_C_ADDR,
				 ST_ASM330LHHX_REG_SW_RESET_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_SW_RESET_MASK, 1));
	if (err < 0)
		return err;

	msleep(50);

	/* boot */
	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_CTRL3_C_ADDR,
				 ST_ASM330LHHX_REG_BOOT_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_BOOT_MASK, 1));

	msleep(50);

	return err;
}

static int st_asm330lhhx_init_device(struct st_asm330lhhx_hw *hw)
{
	int err;

	/* enable Block Data Update */
	err = regmap_update_bits(hw->regmap, ST_ASM330LHHX_REG_CTRL3_C_ADDR,
				 ST_ASM330LHHX_REG_BDU_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_BDU_MASK, 1));
	if (err < 0)
		return err;

	err = regmap_update_bits(hw->regmap, ST_ASM330LHHX_REG_CTRL5_C_ADDR,
				 ST_ASM330LHHX_REG_ROUNDING_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_ROUNDING_MASK, 3));
	if (err < 0)
		return err;

	/* initialize sensors filter bandwidth configuration */
	hw->enable_drdy_mask = true;
	err = st_asm330lhhx_init_xl_filters(hw);
	if (err < 0)
		return err;

	err = st_asm330lhhx_init_g_filters(hw);
	if (err < 0)
		return err;

	/* Enable DRDY MASK for filters settling time */
	return regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_CTRL4_C_ADDR,
				 ST_ASM330LHHX_REG_DRDY_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_DRDY_MASK,
					    hw->enable_drdy_mask ? 1 : 0));
}

static struct iio_dev *st_asm330lhhx_alloc_iiodev(struct st_asm330lhhx_hw *hw,
					       enum st_asm330lhhx_sensor_id id)
{
	struct st_asm330lhhx_sensor *sensor;
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
	sensor->discard_samples = 0;
	sensor->last_fifo_timestamp = 0;

#ifdef ST_ASM330LHHX_DEBUG_DISCHARGE
	sensor->discharged_samples = 0;
#endif /* ST_ASM330LHHX_DEBUG_DISCHARGE */

	/*
	 * for acc/gyro the default Android full scale settings are:
	 * Acc FS 8g (78.40 m/s^2)
	 * Gyro FS 1000dps (16.45 radians/sec)
	 */
	switch (id) {
	case ST_ASM330LHHX_ID_ACC:
		iio_dev->channels = st_asm330lhhx_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_asm330lhhx_acc_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			 "%s_accel", hw->settings->id.name);
		iio_dev->info = &st_asm330lhhx_acc_info;
		iio_dev->available_scan_masks =
					st_asm330lhhx_available_scan_masks;
		sensor->max_watermark = ST_ASM330LHHX_MAX_FIFO_DEPTH;
		sensor->gain = st_asm330lhhx_fs_table[id].fs_avl[ST_ASM330LHHX_DEFAULT_XL_FS_INDEX].gain;
		sensor->odr = st_asm330lhhx_odr_table[id].odr_avl[ST_ASM330LHHX_DEFAULT_XL_ODR_INDEX].hz;
		sensor->uodr = st_asm330lhhx_odr_table[id].odr_avl[ST_ASM330LHHX_DEFAULT_XL_ODR_INDEX].uhz;
		sensor->offset = 0;
		sensor->pm = ST_ASM330LHHX_HP_MODE;
		sensor->min_st = ST_ASM330LHHX_SELFTEST_ACCEL_MIN;
		sensor->max_st = ST_ASM330LHHX_SELFTEST_ACCEL_MAX;
		break;
	case ST_ASM330LHHX_ID_GYRO:
		iio_dev->channels = st_asm330lhhx_gyro_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_asm330lhhx_gyro_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_gyro", hw->settings->id.name);
		iio_dev->info = &st_asm330lhhx_gyro_info;
		iio_dev->available_scan_masks =
					st_asm330lhhx_available_scan_masks;
		sensor->max_watermark = ST_ASM330LHHX_MAX_FIFO_DEPTH;
		sensor->gain = st_asm330lhhx_fs_table[id].fs_avl[ST_ASM330LHHX_DEFAULT_G_FS_INDEX].gain;
		sensor->odr = st_asm330lhhx_odr_table[id].odr_avl[ST_ASM330LHHX_DEFAULT_G_ODR_INDEX].hz;
		sensor->uodr = st_asm330lhhx_odr_table[id].odr_avl[ST_ASM330LHHX_DEFAULT_G_ODR_INDEX].uhz;
		sensor->offset = 0;
		sensor->pm = ST_ASM330LHHX_HP_MODE;
		sensor->min_st = ST_ASM330LHHX_SELFTEST_GYRO_MIN;
		sensor->max_st = ST_ASM330LHHX_SELFTEST_GYRO_MAX;
		break;
	case ST_ASM330LHHX_ID_TEMP:
		iio_dev->channels = st_asm330lhhx_temp_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_asm330lhhx_temp_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_temp", hw->settings->id.name);
		iio_dev->info = &st_asm330lhhx_temp_info;
		iio_dev->available_scan_masks =
				    st_asm330lhhx_temp_available_scan_masks;
		sensor->max_watermark = ST_ASM330LHHX_MAX_FIFO_DEPTH;
		sensor->gain = st_asm330lhhx_fs_table[id].fs_avl[ST_ASM330LHHX_DEFAULT_T_FS_INDEX].gain;
		sensor->odr = st_asm330lhhx_odr_table[id].odr_avl[ST_ASM330LHHX_DEFAULT_T_ODR_INDEX].hz;
		sensor->uodr = st_asm330lhhx_odr_table[id].odr_avl[ST_ASM330LHHX_DEFAULT_T_ODR_INDEX].uhz;
		sensor->offset = ST_ASM330LHHX_TEMP_OFFSET;
		sensor->pm = ST_ASM330LHHX_NO_MODE;
		break;
	default:
		return NULL;
	}

	st_asm330lhhx_set_full_scale(sensor, sensor->gain);
	iio_dev->name = sensor->name;

	return iio_dev;
}

static void st_asm330lhhx_get_properties(struct st_asm330lhhx_hw *hw)
{
	if (device_property_read_u32(hw->dev, "st,module_id", &hw->module_id))
		hw->module_id = 1;

	if (device_property_read_bool(hw->dev, "wakeup-source"))
		hw->wakeup_source = true;
}

static void st_asm330lhhx_disable_regulator_action(void *_data)
{
	struct st_asm330lhhx_hw *hw = _data;

	regulator_disable(hw->vddio_supply);
	regulator_disable(hw->vdd_supply);
}

static int st_asm330lhhx_power_enable(struct st_asm330lhhx_hw *hw)
{
	int err;

	hw->vdd_supply = devm_regulator_get(hw->dev, "vdd");
	if (IS_ERR(hw->vdd_supply)) {
		if (PTR_ERR(hw->vdd_supply) != -EPROBE_DEFER)
			dev_err(hw->dev, "Failed to get vdd regulator %d\n",
				(int)PTR_ERR(hw->vdd_supply));

		return PTR_ERR(hw->vdd_supply);
	}

	hw->vddio_supply = devm_regulator_get(hw->dev, "vddio");
	if (IS_ERR(hw->vddio_supply)) {
		if (PTR_ERR(hw->vddio_supply) != -EPROBE_DEFER)
			dev_err(hw->dev, "Failed to get vddio regulator %d\n",
				(int)PTR_ERR(hw->vddio_supply));

		return PTR_ERR(hw->vddio_supply);
	}

	err = regulator_enable(hw->vdd_supply);
	if (err) {
		dev_err(hw->dev, "Failed to enable vdd regulator: %d\n", err);
		return err;
	}

	err = regulator_enable(hw->vddio_supply);
	if (err) {
		regulator_disable(hw->vdd_supply);
		return err;
	}

	err = devm_add_action_or_reset(hw->dev,
				       st_asm330lhhx_disable_regulator_action,
				       hw);
	if (err) {
		dev_err(hw->dev, "Failed to setup regulator cleanup action %d\n", err);
		return err;
	}

	return 0;
}

int st_asm330lhhx_probe(struct device *dev, int irq, int hw_id,
			struct regmap *regmap)
{
	struct st_asm330lhhx_hw *hw;
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
	hw->odr_table_entry = st_asm330lhhx_odr_table;
	hw->hw_timestamp_global = 0;
	hw->has_hw_fifo = hw->irq > 0 ? true : false;

	err = st_asm330lhhx_power_enable(hw);
	if (err != 0)
		return err;

	/* set page zero before access to registers */
	if (hw_id == ST_ASM330LHHX_ID) {
		err = st_asm330lhhx_set_page_0(hw);
		if (err < 0)
			return err;
	}

	err = st_asm330lhhx_check_whoami(hw, hw_id);
	if (err < 0)
		return err;

	st_asm330lhhx_get_properties(hw);

	err = st_asm330lhhx_get_odr_calibration(hw);
	if (err < 0)
		return err;

	err = st_asm330lhhx_reset_device(hw);
	if (err < 0)
		return err;

	err = st_asm330lhhx_init_device(hw);
	if (err < 0)
		return err;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)
	err = iio_read_mount_matrix(hw->dev, &hw->orientation);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5,2,0)
	err = iio_read_mount_matrix(hw->dev, "mount-matrix", &hw->orientation);
#else /* LINUX_VERSION_CODE */
	err = of_iio_read_mount_matrix(hw->dev, "mount-matrix", &hw->orientation);
#endif /* LINUX_VERSION_CODE */

	if (err) {
		dev_err(dev, "Failed to retrieve mounting matrix %d\n", err);
		return err;
	}

	for (i = ST_ASM330LHHX_ID_GYRO; i <= ST_ASM330LHHX_ID_TEMP; i++) {
		hw->iio_devs[i] = st_asm330lhhx_alloc_iiodev(hw, i);
		if (!hw->iio_devs[i])
			return -ENOMEM;
	}

	if (hw->settings->st_shub_probe) {
		err = st_asm330lhhx_shub_probe(hw);
		if (err < 0)
			return err;
	}

	err = st_asm330lhhx_allocate_buffers(hw);
	if (err < 0)
		return err;

	if (hw->has_hw_fifo) {
		err = st_asm330lhhx_trigger_setup(hw);
		if (err < 0)
			return err;

		err = st_asm330lhhx_event_init(hw);
		if (err < 0)
			return err;

		if (hw->settings->st_mlc_probe) {
			err = st_asm330lhhx_mlc_probe(hw);
			if (err < 0)
				return err;
		}
	}

	for (i = ST_ASM330LHHX_ID_GYRO; i < ST_ASM330LHHX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

	if (hw->settings->st_mlc_probe && hw->has_hw_fifo) {
		err = st_asm330lhhx_mlc_init_preload(hw);
		if (err)
			return err;
	}

	device_init_wakeup(dev,
			   device_property_read_bool(dev, "wakeup-source"));

	dev_info(dev, "Device probed\n");

	return 0;
}
EXPORT_SYMBOL(st_asm330lhhx_probe);

void st_asm330lhhx_remove(struct device *dev)
{
	st_asm330lhhx_mlc_remove(dev);
}
EXPORT_SYMBOL(st_asm330lhhx_remove);

static int __maybe_unused
st_asm330lhhx_configure_wake_up(struct st_asm330lhhx_hw *hw)
{

#if defined(CONFIG_IIO_ST_ASM330LHHX_STORE_SAMPLE_FIFO_SUSPEND)
	u16 fifo_watermark = ST_ASM330LHHX_MAX_FIFO_DEPTH;
	__le16 wdata;
#endif /* CONFIG_IIO_ST_ASM330LHHX_STORE_SAMPLE_FIFO_SUSPEND */

	int err;

	/* disable fifo wtm interrupt */
	err = regmap_update_bits(hw->regmap, ST_ASM330LHHX_REG_INT1_CTRL_ADDR,
				 ST_ASM330LHHX_REG_INT_FIFO_TH_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_INT_FIFO_TH_MASK,
					    0));
	if (err < 0)
		return err;

	/* set fifo in bypass mode */
	err = regmap_update_bits(hw->regmap, ST_ASM330LHHX_REG_FIFO_CTRL4_ADDR,
				 ST_ASM330LHHX_REG_FIFO_MODE_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_FIFO_MODE_MASK,
					    ST_ASM330LHHX_FIFO_BYPASS));
	if (err < 0)
		return err;

	/* enable wake-up interrupt */
	err = regmap_update_bits(hw->regmap, hw->embfunc_pg0_irq_reg,
				 ST_ASM330LHHX_INT_WU_MASK,
				 FIELD_PREP(ST_ASM330LHHX_INT_WU_MASK,
					    0x01));
	if (err < 0)
		return err;

#if defined(CONFIG_IIO_ST_ASM330LHHX_STORE_SAMPLE_FIFO_SUSPEND)
	/* set max fifo watermark level */
	wdata = cpu_to_le16(fifo_watermark);
	err = regmap_bulk_write(hw->regmap, ST_ASM330LHHX_REG_FIFO_CTRL1_ADDR,
				&wdata, sizeof(wdata));
	if (err < 0)
		return err;

	/* enable xl batching at 26 Hz */
	err = regmap_update_bits(hw->regmap, ST_ASM330LHHX_REG_FIFO_CTRL3_ADDR,
				 ST_ASM330LHHX_REG_BDR_XL_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_BDR_XL_MASK,
					    0x02));
	if (err < 0)
		return err;

	err = st_asm330lhhx_reset_hwts(hw);
	if (err < 0)
		return err;

	/* set fifo mode continuous for batching */
	err = regmap_update_bits(hw->regmap, ST_ASM330LHHX_REG_FIFO_CTRL4_ADDR,
				 ST_ASM330LHHX_REG_FIFO_MODE_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_FIFO_MODE_MASK,
					    ST_ASM330LHHX_FIFO_CONT));
	if (err < 0)
		return err;
#endif /* CONFIG_IIO_ST_ASM330LHHX_STORE_SAMPLE_FIFO_SUSPEND */

	/* set sensors odr to 26 Hz */
	err = regmap_update_bits(hw->regmap,
			st_asm330lhhx_odr_table[ST_ASM330LHHX_ID_ACC].reg.addr,
			st_asm330lhhx_odr_table[ST_ASM330LHHX_ID_ACC].reg.mask,
			ST_ASM330LHHX_SHIFT_VAL(0x02,
				st_asm330lhhx_odr_table[ST_ASM330LHHX_ID_ACC].reg.mask));

	return err < 0 ? err : 0;
}

static int __maybe_unused st_asm330lhhx_suspend(struct device *dev)
{
	struct st_asm330lhhx_hw *hw = dev_get_drvdata(dev);
	int i, err = 0;

	dev_info(dev, "Suspending device\n");

	disable_hardirq(hw->irq);

	for (i = 0; i < ST_ASM330LHHX_ID_MAX; i++) {
		struct st_asm330lhhx_sensor *sensor;

		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT_ULL(sensor->id)))
			continue;

		/* power off enabled sensors */
		err = st_asm330lhhx_set_odr(sensor, 0, 0);
		if (err < 0)
			return err;
	}

	if (st_asm330lhhx_is_fifo_enabled(hw)) {
		err = st_asm330lhhx_suspend_fifo(hw);
		if (err < 0)
			return err;

#if defined(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)
		hrtimer_cancel(&hw->timesync_timer);
		cancel_work_sync(&hw->timesync_work);
#endif /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */

	}

	err = st_asm330lhhx_bk_regs(hw);
	if (err < 0)
		return err;

	if (device_may_wakeup(dev) && (hw->wakeup_source == true)) {
		u32 dummy;

		/* configure wake-up events trigger */
		err = st_asm330lhhx_configure_wake_up(hw);
		if (err < 0)
			return err;

		/* avoid false wake-up events */
		usleep_range(40000, 41000);

		/* clean wake-up source */
		err = regmap_read(hw->regmap,
				  ST_ASM330LHHX_REG_ALL_INT_SRC_ADDR,
				  &dummy);
		if (err < 0)
			return err;

		/*
		 * enable event interrupt
		 *
		 * NOTE: be careful, check whether wake-up threshold value is
		 * greater than 0 before enable the interrupt, the risk is that
		 * with zero thresholds system will be immediately reawakened
		 */
		err = regmap_update_bits(hw->regmap,
					 ST_ASM330LHHX_REG_INT_CFG1_ADDR,
					 ST_ASM330LHHX_INTERRUPTS_ENABLE_MASK,
					 FIELD_PREP(ST_ASM330LHHX_INTERRUPTS_ENABLE_MASK,
						    0x01));
		if (err < 0)
			return err;

		enable_irq_wake(hw->irq);
		dev_info(dev, "Enabling wake-up\n");
	}

	return err < 0 ? err : 0;
}

static int __maybe_unused st_asm330lhhx_resume(struct device *dev)
{
	struct st_asm330lhhx_hw *hw = dev_get_drvdata(dev);
	int i, err = 0;

	dev_info(dev, "Resuming device\n");

	if (device_may_wakeup(dev) && (hw->wakeup_source == true)) {
		/* set accel sensor in power down */
		err = regmap_update_bits(hw->regmap,
				st_asm330lhhx_odr_table[ST_ASM330LHHX_ID_ACC].reg.addr,
				st_asm330lhhx_odr_table[ST_ASM330LHHX_ID_ACC].reg.mask,
				ST_ASM330LHHX_SHIFT_VAL(0x00,
					st_asm330lhhx_odr_table[ST_ASM330LHHX_ID_ACC].reg.mask));

		disable_irq_wake(hw->irq);

		/* clean wake-up source */
		err = regmap_read(hw->regmap,
				  ST_ASM330LHHX_REG_WAKE_UP_SRC_ADDR,
				  &hw->wakeup_status);
		if (err < 0)
			return err;

		/* unmask only bits: WU_IA, X_WU, Y_WU and Z_WU */
		hw->wakeup_status &= ST_ASM330LHHX_WAKE_UP_EVENT_MASK;

#if defined(CONFIG_IIO_ST_ASM330LHHX_STORE_SAMPLE_FIFO_SUSPEND)
	/* flush fifo data to user space */
		st_asm330lhhx_flush_fifo_during_resume(hw);
#endif /* CONFIG_IIO_ST_ASM330LHHX_STORE_SAMPLE_FIFO_SUSPEND */

	}

	err = st_asm330lhhx_restore_regs(hw);
	if (err < 0)
		return err;

	for (i = 0; i < ST_ASM330LHHX_ID_MAX; i++) {
		struct st_asm330lhhx_sensor *sensor;

		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT_ULL(sensor->id)))
			continue;

		err = st_asm330lhhx_set_odr(sensor, sensor->odr, sensor->uodr);
		if (err < 0)
			return err;
	}

	err = st_asm330lhhx_reset_hwts(hw);
	if (err < 0)
		return err;

	if (st_asm330lhhx_is_fifo_enabled(hw))
		err = st_asm330lhhx_set_fifo_mode(hw, ST_ASM330LHHX_FIFO_CONT);

#if defined(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)
	if (hw->fifo_mode != ST_ASM330LHHX_FIFO_BYPASS) {
		hrtimer_start(&hw->timesync_timer,
			      ktime_set(0, 0),
			      HRTIMER_MODE_REL);
	}
#endif /* CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP */

	enable_irq(hw->irq);

	return err < 0 ? err : 0;
}

const struct dev_pm_ops st_asm330lhhx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_asm330lhhx_suspend, st_asm330lhhx_resume)
};
EXPORT_SYMBOL(st_asm330lhhx_pm_ops);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_asm330lhhx driver");
MODULE_LICENSE("GPL v2");
