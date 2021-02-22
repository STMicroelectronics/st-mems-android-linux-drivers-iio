/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics st_lsm6dsrx sensor driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Tesi Mario <mario.tesi@st.com>
 */

#ifndef ST_LSM6DSRX_H
#define ST_LSM6DSRX_H

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/bitfield.h>

#define ST_LSM6DSRX_ODR_EXPAND(odr, uodr)	(((odr) * 1000000) + (uodr))

#define ST_LSM6DSRX_DEV_NAME			"lsm6dsrx"
#define ST_LSM6DSRX_DRV_VERSION			"1.1"

#define ST_LSM6DSRX_REG_FUNC_CFG_ACCESS_ADDR	0x01
#define ST_LSM6DSRX_REG_SHUB_REG_MASK		BIT(6)
#define ST_LSM6DSRX_REG_FUNC_CFG_MASK		BIT(7)
#define ST_LSM6DSRX_REG_ACCESS_MASK		GENMASK(7, 6)

#define ST_LSM6DSRX_REG_FIFO_CTRL1_ADDR		0x07
#define ST_LSM6DSRX_REG_FIFO_CTRL2_ADDR		0x08
#define ST_LSM6DSRX_REG_FIFO_WTM_MASK		GENMASK(8, 0)
#define ST_LSM6DSRX_REG_FIFO_WTM8_MASK		BIT(0)
#define ST_LSM6DSRX_REG_FIFO_STATUS_DIFF	GENMASK(9, 0)

#define ST_LSM6DSRX_REG_FIFO_CTRL3_ADDR		0x09
#define ST_LSM6DSRX_REG_BDR_XL_MASK		GENMASK(3, 0)
#define ST_LSM6DSRX_REG_BDR_GY_MASK		GENMASK(7, 4)

#define ST_LSM6DSRX_REG_FIFO_CTRL4_ADDR		0x0a
#define ST_LSM6DSRX_REG_FIFO_MODE_MASK		GENMASK(2, 0)
#define ST_LSM6DSRX_REG_ODR_T_BATCH_MASK	GENMASK(5, 4)
#define ST_LSM6DSRX_REG_DEC_TS_MASK		GENMASK(7, 6)

#define ST_LSM6DSRX_REG_INT1_CTRL_ADDR		0x0d
#define ST_LSM6DSRX_REG_INT2_CTRL_ADDR		0x0e
#define ST_LSM6DSRX_REG_FIFO_TH_MASK		BIT(3)

#define ST_LSM6DSRX_REG_WHOAMI_ADDR		0x0f
#define ST_LSM6DSRX_WHOAMI_VAL			0x6b

#define ST_LSM6DSRX_CTRL1_XL_ADDR		0x10
#define ST_LSM6DSRX_CTRL2_G_ADDR		0x11

#define ST_LSM6DSRX_REG_CTRL3_C_ADDR		0x12
#define ST_LSM6DSRX_REG_SW_RESET_MASK		BIT(0)
#define ST_LSM6DSRX_REG_PP_OD_MASK		BIT(4)
#define ST_LSM6DSRX_REG_H_LACTIVE_MASK		BIT(5)
#define ST_LSM6DSRX_REG_BDU_MASK		BIT(6)
#define ST_LSM6DSRX_REG_BOOT_MASK		BIT(7)

#define ST_LSM6DSRX_REG_CTRL4_C_ADDR		0x13
#define ST_LSM6DSRX_REG_DRDY_MASK		BIT(3)

#define ST_LSM6DSRX_REG_CTRL5_C_ADDR		0x14
#define ST_LSM6DSRX_REG_ROUNDING_MASK		GENMASK(6, 5)
#define ST_LSM6DSRX_REG_ST_G_MASK		GENMASK(3, 2)
#define ST_LSM6DSRX_REG_ST_XL_MASK		GENMASK(1, 0)

#define ST_LSM6DSRX_SELFTEST_ACCEL_MIN		737
#define ST_LSM6DSRX_SELFTEST_ACCEL_MAX		13934
#define ST_LSM6DSRX_SELFTEST_GYRO_MIN		2142
#define ST_LSM6DSRX_SELFTEST_GYRO_MAX		10000

#define ST_LSM6DSRX_SELF_TEST_DISABLED_VAL	0
#define ST_LSM6DSRX_SELF_TEST_POS_SIGN_VAL	1
#define ST_LSM6DSRX_SELF_TEST_NEG_ACCEL_SIGN_VAL	2
#define ST_LSM6DSRX_SELF_TEST_NEG_GYRO_SIGN_VAL	3

#define ST_LSM6DSRX_REG_STATUS_MASTER_MAINPAGE_ADDR	0x39
#define ST_LSM6DSRX_REG_STATUS_SENS_HUB_ENDOP_MASK	BIT(0)

#define ST_LSM6DSRX_REG_CTRL6_C_ADDR		0x15
#define ST_LSM6DSRX_REG_XL_HM_MODE_MASK		BIT(4)

#define ST_LSM6DSRX_REG_CTRL7_G_ADDR		0x16
#define ST_LSM6DSRX_REG_G_HM_MODE_MASK		BIT(7)

#define ST_LSM6DSRX_REG_CTRL10_C_ADDR		0x19
#define ST_LSM6DSRX_REG_TIMESTAMP_EN_MASK	BIT(5)

#define ST_LSM6DSRX_REG_STATUS_ADDR		0x1e
#define ST_LSM6DSRX_REG_STATUS_XLDA		BIT(0)
#define ST_LSM6DSRX_REG_STATUS_GDA		BIT(1)
#define ST_LSM6DSRX_REG_STATUS_TDA		BIT(2)

#define ST_LSM6DSRX_REG_OUT_TEMP_L_ADDR		0x20

#define ST_LSM6DSRX_REG_OUTX_L_A_ADDR		0x28
#define ST_LSM6DSRX_REG_OUTY_L_A_ADDR		0x2a
#define ST_LSM6DSRX_REG_OUTZ_L_A_ADDR		0x2c

#define ST_LSM6DSRX_REG_OUTX_L_G_ADDR		0x22
#define ST_LSM6DSRX_REG_OUTY_L_G_ADDR		0x24
#define ST_LSM6DSRX_REG_OUTZ_L_G_ADDR		0x26

#define ST_LSM6DSRX_FSM_STATUS_A_MAINPAGE	0x36
#define ST_LSM6DSRX_FSM_STATUS_B_MAINPAGE	0x37
#define ST_LSM6DSRX_MLC_STATUS_MAINPAGE 	0x38

#define ST_LSM6DSRX_REG_FIFO_STATUS1_ADDR	0x3a
#define ST_LSM6DSRX_REG_TIMESTAMP0_ADDR		0x40
#define ST_LSM6DSRX_REG_TIMESTAMP2_ADDR		0x42

#define ST_LSM6DSRX_REG_TAP_CFG0_ADDR		0x56
#define ST_LSM6DSRX_REG_LIR_MASK		BIT(0)

#define ST_LSM6DSRX_REG_MD1_CFG_ADDR		0x5e
#define ST_LSM6DSRX_REG_MD2_CFG_ADDR		0x5f
#define ST_LSM6DSRX_REG_INT2_TIMESTAMP_MASK	BIT(0)
#define ST_LSM6DSRX_REG_INT_EMB_FUNC_MASK	BIT(1)

#define ST_LSM6DSRX_INTERNAL_FREQ_FINE		0x63

#define ST_LSM6DSRX_REG_FIFO_DATA_OUT_TAG_ADDR	0x78

/* shub registers */
#define ST_LSM6DSRX_REG_MASTER_CONFIG_ADDR	0x14
#define ST_LSM6DSRX_REG_WRITE_ONCE_MASK		BIT(6)
#define ST_LSM6DSRX_REG_SHUB_PU_EN_MASK      	BIT(3)
#define ST_LSM6DSRX_REG_MASTER_ON_MASK		BIT(2)

#define ST_LSM6DSRX_REG_SLV0_ADDR		0x15
#define ST_LSM6DSRX_REG_SLV0_CFG		0x17
#define ST_LSM6DSRX_REG_SLV1_ADDR		0x18
#define ST_LSM6DSRX_REG_SLV2_ADDR		0x1b
#define ST_LSM6DSRX_REG_SLV3_ADDR		0x1e
#define ST_LSM6DSRX_REG_DATAWRITE_SLV0_ADDR	0x21
#define ST_LSM6DSRX_REG_BATCH_EXT_SENS_EN_MASK	BIT(3)
#define ST_LSM6DSRX_REG_SLAVE_NUMOP_MASK	GENMASK(2, 0)

#define ST_LSM6DSRX_REG_STATUS_MASTER_ADDR	0x22
#define ST_LSM6DSRX_REG_SENS_HUB_ENDOP_MASK	BIT(0)

#define ST_LSM6DSRX_REG_SLV0_OUT_ADDR		0x02

/* embedded function registers */
#define ST_LSM6DSRX_EMB_FUNC_EN_B_ADDR		0x05
#define ST_LSM6DSRX_FSM_EN_MASK			BIT(0)
#define ST_LSM6DSRX_MLC_EN_MASK			BIT(4)

#define ST_LSM6DSRX_FSM_INT1_A_ADDR		0x0b
#define ST_LSM6DSRX_FSM_INT1_B_ADDR		0x0c
#define ST_LSM6DSRX_MLC_INT1_ADDR		0x0d

#define ST_LSM6DSRX_FSM_INT2_A_ADDR		0x0f
#define ST_LSM6DSRX_FSM_INT2_B_ADDR		0x10
#define ST_LSM6DSRX_MLC_INT2_ADDR		0x11

#define ST_LSM6DSRX_REG_MLC_STATUS_ADDR		0x15

#define ST_LSM6DSRX_FSM_ENABLE_A_ADDR		0x46
#define ST_LSM6DSRX_FSM_ENABLE_B_ADDR		0x47

#define ST_LSM6DSRX_FSM_OUTS1_ADDR		0x4c

#define ST_LSM6DSRX_REG_MLC0_SRC_ADDR		0x70

/* Timestamp Tick 25us/LSB */
#define ST_LSM6DSRX_TS_DELTA_NS			25000ULL

/* Temperature in uC */
#define ST_LSM6DSRX_TEMP_GAIN			256
#define ST_LSM6DSRX_TEMP_OFFSET			6400

/* FIFO simple size and depth */
#define ST_LSM6DSRX_SAMPLE_SIZE			6
#define ST_LSM6DSRX_TS_SAMPLE_SIZE		4
#define ST_LSM6DSRX_TAG_SIZE			1
#define ST_LSM6DSRX_FIFO_SAMPLE_SIZE		(ST_LSM6DSRX_SAMPLE_SIZE + \
						 ST_LSM6DSRX_TAG_SIZE)
#define ST_LSM6DSRX_MAX_FIFO_DEPTH		416

#define ST_LSM6DSRX_DATA_CHANNEL(chan_type, addr, mod, ch2, scan_idx,	\
				rb, sb, sg, ext_inf)			\
{									\
	.type = chan_type,						\
	.address = addr,						\
	.modified = mod,						\
	.channel2 = ch2,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = scan_idx,						\
	.scan_type = {							\
		.sign = sg,						\
		.realbits = rb,						\
		.storagebits = sb,					\
		.endianness = IIO_LE,					\
	},								\
	.ext_info = ext_inf,						\
}

static const struct iio_event_spec st_lsm6dsrx_flush_event = {
	.type = IIO_EV_TYPE_FIFO_FLUSH,
	.dir = IIO_EV_DIR_EITHER,
};

static const struct iio_event_spec st_lsm6dsrx_thr_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

#define ST_LSM6DSRX_EVENT_CHANNEL(ctype, etype)	\
{							\
	.type = ctype,					\
	.modified = 0,					\
	.scan_index = -1,				\
	.indexed = -1,					\
	.event_spec = &st_lsm6dsrx_##etype##_event,	\
	.num_event_specs = 1,				\
}

#define ST_LSM6DSRX_SHIFT_VAL(val, mask)	(((val) << __ffs(mask)) & (mask))

enum st_lsm6dsrx_pm_t {
	ST_LSM6DSRX_HP_MODE = 0,
	ST_LSM6DSRX_LP_MODE,
	ST_LSM6DSRX_NO_MODE,
};

#ifdef CONFIG_IIO_ST_LSM6DSRX_MLC
enum st_lsm6dsrx_fsm_mlc_enable_id {
	ST_LSM6DSRX_MLC_FSM_DISABLED = 0,
	ST_LSM6DSRX_MLC_ENABLED = BIT(0),
	ST_LSM6DSRX_FSM_ENABLED = BIT(1),
};

/**
 * struct mlc_config_t -
 * @mlc_int_addr: interrupt register address.
 * @mlc_int_mask: interrupt register mask.
 * @fsm_int_addr: interrupt register address.
 * @fsm_int_mask: interrupt register mask.
 * @mlc_configured: number of mlc configured.
 * @fsm_configured: number of fsm configured.
 * @bin_len: fw binary size.
 * @requested_odr: Min ODR requested to works properly.
 * @status: MLC / FSM enabled status.
 */
struct st_lsm6dsrx_mlc_config_t {
	uint8_t mlc_int_addr;
	uint8_t mlc_int_mask;
	uint8_t fsm_int_addr[2];
	uint8_t fsm_int_mask[2];
	uint8_t mlc_configured;
	uint8_t fsm_configured;
	uint16_t bin_len;
	uint16_t requested_odr;
	enum st_lsm6dsrx_fsm_mlc_enable_id status;
};
#endif /* CONFIG_IIO_ST_LSM6DSRX_MLC */

/**
 * struct st_lsm6dsrx_reg - Generic sensor register description (addr + mask)
 * @addr: Address of register.
 * @mask: Bitmask register for proper usage.
 */
struct st_lsm6dsrx_reg {
	u8 addr;
	u8 mask;
};

enum st_asm330lhh_suspend_resume_register {
	ST_LSM6DSRX_CTRL1_XL_REG = 0,
	ST_LSM6DSRX_CTRL2_G_REG,
	ST_LSM6DSRX_REG_CTRL3_C_REG,
	ST_LSM6DSRX_REG_CTRL4_C_REG,
	ST_LSM6DSRX_REG_CTRL5_C_REG,
	ST_LSM6DSRX_REG_CTRL10_C_REG,
	ST_LSM6DSRX_REG_TAP_CFG0_REG,
	ST_LSM6DSRX_REG_INT1_CTRL_REG,
	ST_LSM6DSRX_REG_INT2_CTRL_REG,
	ST_LSM6DSRX_REG_FIFO_CTRL1_REG,
	ST_LSM6DSRX_REG_FIFO_CTRL2_REG,
	ST_LSM6DSRX_REG_FIFO_CTRL3_REG,
	ST_LSM6DSRX_REG_FIFO_CTRL4_REG,
	ST_LSM6DSRX_REG_EMB_FUNC_EN_B_REG,
	ST_LSM6DSRX_REG_FSM_INT1_A_REG,
	ST_LSM6DSRX_REG_FSM_INT1_B_REG,
	ST_LSM6DSRX_REG_MLC_INT1_REG,
	ST_LSM6DSRX_REG_FSM_INT2_A_REG,
	ST_LSM6DSRX_REG_FSM_INT2_B_REG,
	ST_LSM6DSRX_REG_MLC_INT2_REG,
	ST_LSM6DSRX_SUSPEND_RESUME_REGS,
};

/**
 * Define embedded functions register access
 *
 * FUNC_CFG_ACCESS_0 is default bank
 * FUNC_CFG_ACCESS_SHUB_REG Enable access to the sensor hub (I2C master)
 *                          registers.
 * FUNC_CFG_ACCESS_FUNC_CFG Enable access to the embedded functions
 *                          configuration registers.
 */
enum st_asm330lhh_page_sel_register {
	FUNC_CFG_ACCESS_0 = 0,
	FUNC_CFG_ACCESS_SHUB_REG,
	FUNC_CFG_ACCESS_FUNC_CFG,
};

/**
 * struct st_lsm6dsrx_suspend_resume_entry - Register value for backup/restore
 * @page: Page bank reg map.
 * @addr: Address of register.
 * @val: Register value.
 * @mask: Bitmask register for proper usage.
 */
struct st_lsm6dsrx_suspend_resume_entry {
	u8 page;
	u8 addr;
	u8 val;
	u8 mask;
};

/**
 * struct st_lsm6dsrx_odr - Single ODR entry
 * @hz: Most significant part of the sensor ODR (Hz).
 * @uhz: Less significant part of the sensor ODR (micro Hz).
 * @val: ODR register value.
 * @batch_val: Batching ODR register value.
 */
struct st_lsm6dsrx_odr {
	u16 hz;
	u32 uhz;
	u8 val;
	u8 batch_val;
};

/**
 * struct st_lsm6dsrx_odr_table_entry - Sensor ODR table
 * @size: Size of ODR table.
 * @reg: ODR register.
 * @pm: Power mode register.
 * @batching_reg: ODR register for batching on fifo.
 * @odr_avl: Array of supported ODR value.
 */
struct st_lsm6dsrx_odr_table_entry {
	u8 size;
	struct st_lsm6dsrx_reg reg;
	struct st_lsm6dsrx_reg pm;
	struct st_lsm6dsrx_reg batching_reg;
	struct st_lsm6dsrx_odr odr_avl[8];
};

/**
 * struct st_lsm6dsrx_fs - Full Scale sensor table entry
 * @reg: Register description for FS settings.
 * @gain: Sensor sensitivity (mdps/LSB, mg/LSB and uC/LSB).
 * @val: FS register value.
 */
struct st_lsm6dsrx_fs {
	struct st_lsm6dsrx_reg reg;
	u32 gain;
	u8 val;
};

/**
 * struct st_lsm6dsrx_fs_table_entry - Full Scale sensor table
 * @size: Full Scale sensor table size.
 * @fs_avl: Full Scale list entries.
 */
struct st_lsm6dsrx_fs_table_entry {
	u8 size;
	struct st_lsm6dsrx_fs fs_avl[5];
};

enum st_lsm6dsrx_sensor_id {
	ST_LSM6DSRX_ID_GYRO = 0,
	ST_LSM6DSRX_ID_ACC,
	ST_LSM6DSRX_ID_TEMP,
	ST_LSM6DSRX_ID_EXT0,
	ST_LSM6DSRX_ID_EXT1,
#ifdef CONFIG_IIO_ST_LSM6DSRX_MLC
	ST_LSM6DSRX_ID_MLC,
	ST_LSM6DSRX_ID_MLC_0,
	ST_LSM6DSRX_ID_MLC_1,
	ST_LSM6DSRX_ID_MLC_2,
	ST_LSM6DSRX_ID_MLC_3,
	ST_LSM6DSRX_ID_MLC_4,
	ST_LSM6DSRX_ID_MLC_5,
	ST_LSM6DSRX_ID_MLC_6,
	ST_LSM6DSRX_ID_MLC_7,
	ST_LSM6DSRX_ID_FSM_0,
	ST_LSM6DSRX_ID_FSM_1,
	ST_LSM6DSRX_ID_FSM_2,
	ST_LSM6DSRX_ID_FSM_3,
	ST_LSM6DSRX_ID_FSM_4,
	ST_LSM6DSRX_ID_FSM_5,
	ST_LSM6DSRX_ID_FSM_6,
	ST_LSM6DSRX_ID_FSM_7,
	ST_LSM6DSRX_ID_FSM_8,
	ST_LSM6DSRX_ID_FSM_9,
	ST_LSM6DSRX_ID_FSM_10,
	ST_LSM6DSRX_ID_FSM_11,
	ST_LSM6DSRX_ID_FSM_12,
	ST_LSM6DSRX_ID_FSM_13,
	ST_LSM6DSRX_ID_FSM_14,
	ST_LSM6DSRX_ID_FSM_15,
#endif /* CONFIG_IIO_ST_LSM6DSRX_MLC */
	ST_LSM6DSRX_ID_MAX,
};

static const enum st_lsm6dsrx_sensor_id st_lsm6dsrx_mlc_sensor_list[] = {
	 [0] = ST_LSM6DSRX_ID_MLC_0,
	 [1] = ST_LSM6DSRX_ID_MLC_1,
	 [2] = ST_LSM6DSRX_ID_MLC_2,
	 [3] = ST_LSM6DSRX_ID_MLC_3,
	 [4] = ST_LSM6DSRX_ID_MLC_4,
	 [5] = ST_LSM6DSRX_ID_MLC_5,
	 [6] = ST_LSM6DSRX_ID_MLC_6,
	 [7] = ST_LSM6DSRX_ID_MLC_7,
};

static const enum st_lsm6dsrx_sensor_id st_lsm6dsrx_fsm_sensor_list[] = {
	 [0] = ST_LSM6DSRX_ID_FSM_0,
	 [1] = ST_LSM6DSRX_ID_FSM_1,
	 [2] = ST_LSM6DSRX_ID_FSM_2,
	 [3] = ST_LSM6DSRX_ID_FSM_3,
	 [4] = ST_LSM6DSRX_ID_FSM_4,
	 [5] = ST_LSM6DSRX_ID_FSM_5,
	 [6] = ST_LSM6DSRX_ID_FSM_6,
	 [7] = ST_LSM6DSRX_ID_FSM_7,
	 [8] = ST_LSM6DSRX_ID_FSM_8,
	 [9] = ST_LSM6DSRX_ID_FSM_9,
	 [10] = ST_LSM6DSRX_ID_FSM_10,
	 [11] = ST_LSM6DSRX_ID_FSM_11,
	 [12] = ST_LSM6DSRX_ID_FSM_12,
	 [13] = ST_LSM6DSRX_ID_FSM_13,
	 [14] = ST_LSM6DSRX_ID_FSM_14,
	 [15] = ST_LSM6DSRX_ID_FSM_15,
};

#define ST_LSM6DSRX_ID_ALL_FSM_MLC BIT(ST_LSM6DSRX_ID_MLC_0)  | \
				   BIT(ST_LSM6DSRX_ID_MLC_1)  | \
				   BIT(ST_LSM6DSRX_ID_MLC_2)  | \
				   BIT(ST_LSM6DSRX_ID_MLC_3)  | \
				   BIT(ST_LSM6DSRX_ID_MLC_4)  | \
				   BIT(ST_LSM6DSRX_ID_MLC_5)  | \
				   BIT(ST_LSM6DSRX_ID_MLC_6)  | \
				   BIT(ST_LSM6DSRX_ID_MLC_7)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_0)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_1)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_2)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_3)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_4)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_5)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_6)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_7)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_8)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_9)  | \
				   BIT(ST_LSM6DSRX_ID_FSM_10) | \
				   BIT(ST_LSM6DSRX_ID_FSM_11) | \
				   BIT(ST_LSM6DSRX_ID_FSM_12) | \
				   BIT(ST_LSM6DSRX_ID_FSM_13) | \
				   BIT(ST_LSM6DSRX_ID_FSM_14) | \
				   BIT(ST_LSM6DSRX_ID_FSM_15)

/*
 * HW devices that can wakeup the target
 */
#define ST_LSM6DSRX_WAKE_UP_SENSORS (BIT(ST_LSM6DSRX_ID_GYRO) | \
				     BIT(ST_LSM6DSRX_ID_ACC)  | \
				     ST_LSM6DSRX_ID_ALL_FSM_MLC)

/* this is the minimal ODR for wake-up sensors and dependencies */
#define ST_LSM6DSRX_MIN_ODR_IN_WAKEUP	26

enum st_lsm6dsrx_fifo_mode {
	ST_LSM6DSRX_FIFO_BYPASS = 0x0,
	ST_LSM6DSRX_FIFO_CONT = 0x6,
};

enum {
	ST_LSM6DSRX_HW_FLUSH,
	ST_LSM6DSRX_HW_OPERATIONAL,
};

struct st_lsm6dsrx_ext_dev_info {
	const struct st_lsm6dsrx_ext_dev_settings *ext_dev_settings;
	u8 ext_dev_i2c_addr;
};

/**
 * struct st_lsm6dsrx_sensor - ST IMU sensor instance
 * @name: Sensor name.
 * @id: Sensor identifier.
 * @hw: Pointer to instance of struct st_lsm6dsrx_hw.
 * @ext_dev_info: For sensor hub indicate device info struct.
 * @gain: Configured sensor sensitivity.
 * @odr: Output data rate of the sensor [Hz].
 * @uodr: Output data rate of the sensor [uHz].
 * @offset: Sensor data offset.
 * decimator: Sensor decimator
 * dec_counter: Sensor decimator counter
 * @old_data: Used by Temperature sensor for data comtinuity.
 * @max_watermark: Max supported watermark level.
 * @watermark: Sensor watermark level.
 * @pm: sensor power mode (HP, LP).
 * @selftest_status: Report last self test status.
 * @min_st: Min self test raw data value.
 * @max_st: Max self test raw data value.
 */
struct st_lsm6dsrx_sensor {
	char name[32];
	enum st_lsm6dsrx_sensor_id id;
	struct st_lsm6dsrx_hw *hw;
	struct st_lsm6dsrx_ext_dev_info ext_dev_info;

	int odr;
	int uodr;

	union {
		struct {
			u32 gain;
			u32 offset;
			u8 decimator;
			u8 dec_counter;
			__le16 old_data;
			u16 max_watermark;
			u16 watermark;
			enum st_lsm6dsrx_pm_t pm;
			s64 last_fifo_timestamp;

			/* self test */
			int8_t selftest_status;
			int min_st;
			int max_st;
		};
		struct {
			uint8_t status_reg;
			uint8_t outreg_addr;
			enum st_lsm6dsrx_fsm_mlc_enable_id status;
		};
	};
};

/**
 * struct st_lsm6dsrx_hw - ST IMU MEMS hw instance
 * @dev: Pointer to instance of struct device (I2C or SPI).
 * @irq: Device interrupt line (I2C or SPI).
 * @regmap: Register map of the device.
 * @fifo_lock: Mutex to prevent concurrent access to the hw FIFO.
 * @fifo_mode: FIFO operating mode supported by the device.
 * @state: hw operational state.
 * @enable_mask: Enabled sensor bitmask.
 * @requested_mask: Sensor requesting bitmask.
 * @ext_data_len: Number of i2c slave devices connected to I2C master.
 * @ts_delta_ns: Calibrated delta timestamp.
 * @ts_offset: Hw timestamp offset.
 * @hw_ts: Latest hw timestamp from the sensor.
 * @tsample: Sample timestamp.
 * @delta_ts: Delta time between two consecutive interrupts.
 * @ts: Latest timestamp from irq handler.
 * @i2c_master_pu: I2C master line Pull Up configuration.
 * @orientation: Sensor orientation matrix.
 * @mlc_config:
 * @odr_table_entry: Sensors ODR table.
 * @iio_devs: Pointers to acc/gyro iio_dev instances.
 */
struct st_lsm6dsrx_hw {
	struct device *dev;
	int irq;
	struct regmap *regmap;
	struct mutex page_lock;
	struct mutex fifo_lock;
	enum st_lsm6dsrx_fifo_mode fifo_mode;
	unsigned long state;
	u32 enable_mask;
	u32 requested_mask;
	u8 ext_data_len;
	u64 ts_delta_ns;
	s64 ts_offset;
	s64 hw_ts;
	s64 tsample;
	s64 delta_ts;
	s64 ts;
	u8 i2c_master_pu;
	struct iio_mount_matrix orientation;
	struct regulator *vdd_supply;
	struct regulator *vddio_supply;

	struct st_lsm6dsrx_mlc_config_t *mlc_config;
	const struct st_lsm6dsrx_odr_table_entry *odr_table_entry;

	bool preload_mlc;

	struct iio_dev *iio_devs[ST_LSM6DSRX_ID_MAX];
};

extern const struct dev_pm_ops st_lsm6dsrx_pm_ops;

static inline bool st_lsm6dsrx_is_fifo_enabled(struct st_lsm6dsrx_hw *hw)
{
	return hw->enable_mask & (BIT(ST_LSM6DSRX_ID_GYRO) |
				  BIT(ST_LSM6DSRX_ID_ACC));
}

static inline int __st_lsm6dsrx_write_with_mask(struct st_lsm6dsrx_hw *hw,
						unsigned int addr,
						unsigned int mask,
						unsigned int data)
{
	int err;
	unsigned int val = ST_LSM6DSRX_SHIFT_VAL(data, mask);

	err = regmap_update_bits(hw->regmap, addr, mask, val);

	return err;
}

static inline int
st_lsm6dsrx_update_bits_locked(struct st_lsm6dsrx_hw *hw, unsigned int addr,
			       unsigned int mask, unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = __st_lsm6dsrx_write_with_mask(hw, addr, mask, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

/* use when mask is constant */
static inline int st_lsm6dsrx_write_with_mask_locked(struct st_lsm6dsrx_hw *hw,
						     unsigned int addr,
						     unsigned int mask,
						     unsigned int data)
{
	int err;
	unsigned int val = FIELD_PREP(mask, data);

	mutex_lock(&hw->page_lock);
	err = regmap_update_bits(hw->regmap, addr, mask, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
st_lsm6dsrx_read_locked(struct st_lsm6dsrx_hw *hw, unsigned int addr,
			void *val, unsigned int len)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_bulk_read(hw->regmap, addr, val, len);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
st_lsm6dsrx_write_locked(struct st_lsm6dsrx_hw *hw, unsigned int addr,
			 unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_write(hw->regmap, addr, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int st_lsm6dsrx_set_page_access(struct st_lsm6dsrx_hw *hw,
					      unsigned int val,
					      unsigned int mask)
{
	return regmap_update_bits(hw->regmap,
				 ST_LSM6DSRX_REG_FUNC_CFG_ACCESS_ADDR,
				 mask,
				 ST_LSM6DSRX_SHIFT_VAL(val, mask));
}

int st_lsm6dsrx_probe(struct device *dev, int irq,
		      struct regmap *regmap);
int st_lsm6dsrx_sensor_set_enable(struct st_lsm6dsrx_sensor *sensor,
				  bool enable);
int st_lsm6dsrx_buffers_setup(struct st_lsm6dsrx_hw *hw);
int st_lsm6dsrx_get_batch_val(struct st_lsm6dsrx_sensor *sensor, int odr,
			      int uodr, u8 *val);
int st_lsm6dsrx_update_watermark(struct st_lsm6dsrx_sensor *sensor,
				 u16 watermark);
ssize_t st_lsm6dsrx_flush_fifo(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size);
ssize_t st_lsm6dsrx_get_max_watermark(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);
ssize_t st_lsm6dsrx_get_watermark(struct device *dev,
				  struct device_attribute *attr, char *buf);
ssize_t st_lsm6dsrx_set_watermark(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size);
int st_lsm6dsrx_suspend_fifo(struct st_lsm6dsrx_hw *hw);
int st_lsm6dsrx_set_fifo_mode(struct st_lsm6dsrx_hw *hw,
			      enum st_lsm6dsrx_fifo_mode fifo_mode);
int st_lsm6dsrx_update_batching(struct iio_dev *iio_dev, bool enable);
int st_lsm6dsrx_of_get_pin(struct st_lsm6dsrx_hw *hw, int *pin);
int st_lsm6dsrx_shub_probe(struct st_lsm6dsrx_hw *hw);
int st_lsm6dsrx_shub_set_enable(struct st_lsm6dsrx_sensor *sensor,
				bool enable);

#ifdef CONFIG_IIO_ST_LSM6DSRX_MLC
int st_lsm6dsrx_mlc_probe(struct st_lsm6dsrx_hw *hw);
int st_lsm6dsrx_mlc_remove(struct device *dev);
int st_lsm6dsrx_mlc_check_status(struct st_lsm6dsrx_hw *hw);
int st_lsm6dsrx_mlc_init_preload(struct st_lsm6dsrx_hw *hw);
#endif /* CONFIG_IIO_ST_LSM6DSRX_MLC */

#endif /* ST_LSM6DSRX_H */
