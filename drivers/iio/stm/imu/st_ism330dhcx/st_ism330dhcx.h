/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics st_ism330dhcx sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2020 STMicroelectronics Inc.
 */

#ifndef ST_ISM330DHCX_H
#define ST_ISM330DHCX_H

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "../../common/stm_iio_types.h"

#define ST_ISM330DHCX_MAX_ODR			833
#define ST_ISM330DHCX_ODR_LIST_SIZE		8
#define ST_ISM330DHCX_ODR_EXPAND(odr, uodr)	((odr * 1000000) + uodr)

#define ST_ISM330DHCX_DEV_NAME			"ism330dhcx"

#define ST_ISM330DHCX_REG_FUNC_CFG_ACCESS_ADDR	0x01
#define ST_ISM330DHCX_REG_SHUB_REG_MASK		BIT(6)
#define ST_ISM330DHCX_REG_FUNC_CFG_MASK		BIT(7)
#define ST_ISM330DHCX_REG_ACCESS_MASK		GENMASK(7, 6)

#define ST_ISM330DHCX_REG_FIFO_CTRL1_ADDR	0x07
#define ST_ISM330DHCX_REG_FIFO_CTRL2_ADDR	0x08
#define ST_ISM330DHCX_REG_FIFO_WTM_MASK		GENMASK(8, 0)
#define ST_ISM330DHCX_REG_FIFO_WTM8_MASK	BIT(0)

#define ST_ISM330DHCX_REG_FIFO_CTRL3_ADDR	0x09
#define ST_ISM330DHCX_REG_BDR_XL_MASK		GENMASK(3, 0)
#define ST_ISM330DHCX_REG_BDR_GY_MASK		GENMASK(7, 4)

#define ST_ISM330DHCX_REG_FIFO_CTRL4_ADDR	0x0a
#define ST_ISM330DHCX_REG_FIFO_MODE_MASK	GENMASK(2, 0)
#define ST_ISM330DHCX_REG_ODR_T_BATCH_MASK	GENMASK(5, 4)
#define ST_ISM330DHCX_REG_DEC_TS_MASK		GENMASK(7, 6)

#define ST_ISM330DHCX_REG_INT1_CTRL_ADDR	0x0d
#define ST_ISM330DHCX_REG_INT2_CTRL_ADDR	0x0e
#define ST_ISM330DHCX_REG_INT_FIFO_TH_MASK	BIT(3)

#define ST_ISM330DHCX_REG_WHOAMI_ADDR		0x0f
#define ST_ISM330DHCX_WHOAMI_VAL		0x6b

#define ST_ISM330DHCX_CTRL1_XL_ADDR		0x10
#define ST_ISM330DHCX_CTRL2_G_ADDR		0x11
#define ST_ISM330DHCX_REG_CTRL3_C_ADDR		0x12
#define ST_ISM330DHCX_REG_SW_RESET_MASK		BIT(0)
#define ST_ISM330DHCX_REG_PP_OD_MASK		BIT(4)
#define ST_ISM330DHCX_REG_H_LACTIVE_MASK	BIT(5)
#define ST_ISM330DHCX_REG_BDU_MASK		BIT(6)
#define ST_ISM330DHCX_REG_BOOT_MASK		BIT(7)

#define ST_ISM330DHCX_REG_CTRL4_C_ADDR		0x13
#define ST_ISM330DHCX_REG_DRDY_MASK		BIT(3)

#define ST_ISM330DHCX_REG_CTRL5_C_ADDR		0x14
#define ST_ISM330DHCX_REG_ROUNDING_MASK		GENMASK(6, 5)
#define ST_ISM330DHCX_REG_ST_G_MASK		GENMASK(3, 2)
#define ST_ISM330DHCX_REG_ST_XL_MASK		GENMASK(1, 0)

#define ST_ISM330DHCX_SELFTEST_ACCEL_MIN	737
#define ST_ISM330DHCX_SELFTEST_ACCEL_MAX	13934
#define ST_ISM330DHCX_SELFTEST_GYRO_MIN		2142
#define ST_ISM330DHCX_SELFTEST_GYRO_MAX		10000

#define ST_ISM330DHCX_SELF_TEST_DISABLED_VAL	0
#define ST_ISM330DHCX_SELF_TEST_POS_SIGN_VAL	1
#define ST_ISM330DHCX_SELF_TEST_NEG_ACCEL_SIGN_VAL	2
#define ST_ISM330DHCX_SELF_TEST_NEG_GYRO_SIGN_VAL	3

#define ST_ISM330DHCX_REG_CTRL9_XL_ADDR		0x18
#define ST_ISM330DHCX_REG_I3C_DISABLE_MASK	BIT(1)

#define ST_ISM330DHCX_REG_CTRL10_C_ADDR		0x19
#define ST_ISM330DHCX_REG_TIMESTAMP_EN_MASK	BIT(5)

#define ST_ISM330DHCX_REG_ALL_INT_SRC_ADDR	0x1a
#define ST_ISM330DHCX_FF_IA_MASK		BIT(0)
#define ST_ISM330DHCX_WU_IA_MASK		BIT(1)
#define ST_ISM330DHCX_SINGLE_TAP_MASK		BIT(2)
#define ST_ISM330DHCX_DOUBLE_TAP_MASK		BIT(3)
#define ST_ISM330DHCX_D6D_IA_MASK		BIT(4)
#define ST_ISM330DHCX_SLEEP_CHANGE_MASK		BIT(5)

#define ST_ISM330DHCX_REG_WAKE_UP_SRC_ADDR	0x1b
#define ST_ISM330DHCX_WAKE_UP_EVENT_MASK	GENMASK(3, 0)
#define ST_ISM330DHCX_WAKE_UP_SRC_FF_IA_MASK	BIT(5)
#define ST_ISM330DHCX_WAKE_UP_SRC_WU_IA_MASK	BIT(3)
#define ST_ISM330DHCX_X_WU_MASK			BIT(2)
#define ST_ISM330DHCX_Y_WU_MASK			BIT(1)
#define ST_ISM330DHCX_Z_WU_MASK			BIT(0)

#define ST_ISM330DHCX_REG_TAP_SRC_ADDR		0x1c
#define ST_ISM330DHCX_Z_TAP_MASK		BIT(0)
#define ST_ISM330DHCX_Y_TAP_MASK		BIT(1)
#define ST_ISM330DHCX_X_TAP_MASK		BIT(2)
#define ST_ISM330DHCX_TAP_SIGN_MASK		BIT(3)
#define ST_ISM330DHCX_DOUBLE_TAP_IA_MASK	BIT(4)
#define ST_ISM330DHCX_SINGLE_TAP_IA_MASK	BIT(5)
#define ST_ISM330DHCX_TAP_IA_MASK		GENMASK(2, 0)

#define ST_ISM330DHCX_REG_D6D_SRC_ADDR		0x1d
#define ST_ISM330DHCX_D6D_EVENT_MASK		GENMASK(5, 0)
#define ST_ISM330DHCX_D6D_SRC_D6D_IA_MASK	BIT(6)
#define ST_ISM330DHCX_ZH_MASK			BIT(5)
#define ST_ISM330DHCX_ZL_MASK			BIT(4)
#define ST_ISM330DHCX_YH_MASK			BIT(3)
#define ST_ISM330DHCX_YL_MASK			BIT(2)
#define ST_ISM330DHCX_XH_MASK			BIT(1)
#define ST_ISM330DHCX_XL_MASK			BIT(0)

#define ST_ISM330DHCX_REG_STATUS_ADDR		0x1e
#define ST_ISM330DHCX_REG_STATUS_XLDA		BIT(0)
#define ST_ISM330DHCX_REG_STATUS_GDA		BIT(1)
#define ST_ISM330DHCX_REG_STATUS_TDA		BIT(2)

#define ST_ISM330DHCX_REG_OUT_TEMP_L_ADDR	0x20

#define ST_ISM330DHCX_REG_OUTX_L_G_ADDR		0x22
#define ST_ISM330DHCX_REG_OUTY_L_G_ADDR		0x24
#define ST_ISM330DHCX_REG_OUTZ_L_G_ADDR		0x26

#define ST_ISM330DHCX_REG_OUTX_L_A_ADDR		0x28
#define ST_ISM330DHCX_REG_OUTY_L_A_ADDR		0x2a
#define ST_ISM330DHCX_REG_OUTZ_L_A_ADDR		0x2c

#define ST_ISM330DHCX_REG_EMB_FUNC_STATUS_MAINPAGE	0x35
#define ST_ISM330DHCX_REG_INT_STEP_DET_MASK	BIT(3)
#define ST_ISM330DHCX_REG_INT_TILT_MASK		BIT(4)
#define ST_ISM330DHCX_REG_INT_SIGMOT_MASK	BIT(5)

#define ST_ISM330DHCX_REG_FIFO_STATUS1_ADDR	0x3a
#define ST_ISM330DHCX_REG_FIFO_STATUS_DIFF	GENMASK(9, 0)

#define ST_ISM330DHCX_REG_TIMESTAMP0_ADDR	0x40
#define ST_ISM330DHCX_REG_TIMESTAMP2_ADDR	0x42

#define ST_ISM330DHCX_REG_TAP_CFG0_ADDR		0x56
#define ST_ISM330DHCX_REG_LIR_MASK		BIT(0)
#define ST_ISM330DHCX_TAP_X_EN_MASK		BIT(1)
#define ST_ISM330DHCX_TAP_Y_EN_MASK		BIT(2)
#define ST_ISM330DHCX_TAP_Z_EN_MASK		BIT(3)
#define ST_ISM330DHCX_TAP_EN_MASK		GENMASK(3, 1)
#define ST_ISM330DHCX_TAP_SLOPE_FDS_MASK	BIT(4)
#define ST_ISM330DHCX_REG_INT_CLR_ON_READ_MASK	BIT(6)

#define ST_ISM330DHCX_REG_TAP_CFG1_ADDR		0x57
#define ST_ISM330DHCX_TAP_THS_X_MASK		GENMASK(4, 0)
#define ST_ISM330DHCX_TAP_PRIORITY_MASK		GENMASK(7, 5)

#define ST_ISM330DHCX_REG_TAP_CFG2_ADDR		0x58
#define ST_ISM330DHCX_TAP_THS_Y_MASK		GENMASK(4, 0)
#define ST_ISM330DHCX_INTERRUPTS_ENABLE_MASK	BIT(7)

#define ST_ISM330DHCX_REG_TAP_THS_6D_ADDR	0x59
#define ST_ISM330DHCX_TAP_THS_Z_MASK		GENMASK(4, 0)
#define ST_ISM330DHCX_SIXD_THS_MASK		GENMASK(6, 5)

#define ST_ISM330DHCX_REG_INT_DUR2_ADDR		0x5a
#define ST_ISM330DHCX_SHOCK_MASK		GENMASK(1, 0)
#define ST_ISM330DHCX_QUIET_MASK		GENMASK(3, 2)
#define ST_ISM330DHCX_DUR_MASK			GENMASK(7, 4)

#define ST_ISM330DHCX_REG_WAKE_UP_THS_ADDR	0x5b
#define ST_ISM330DHCX_WAKE_UP_THS_MASK		GENMASK(5, 0)
#define ST_ISM330DHCX_SINGLE_DOUBLE_TAP_MASK	BIT(7)

#define ST_ISM330DHCX_REG_WAKE_UP_DUR_ADDR	0x5c
#define ST_ISM330DHCX_WAKE_UP_DUR_MASK		GENMASK(6, 5)

#define ST_ISM330DHCX_REG_FREE_FALL_ADDR	0x5d
#define ST_ISM330DHCX_FF_THS_MASK		GENMASK(2, 0)

#define ST_ISM330DHCX_REG_MD1_CFG_ADDR		0x5e
#define ST_ISM330DHCX_REG_MD2_CFG_ADDR		0x5f
#define ST_ISM330DHCX_REG_INT2_TIMESTAMP_MASK	BIT(0)
#define ST_ISM330DHCX_REG_INT_EMB_FUNC_MASK	BIT(1)
#define ST_ISM330DHCX_INT_6D_MASK		BIT(2)
#define ST_ISM330DHCX_INT_DOUBLE_TAP_MASK	BIT(3)
#define ST_ISM330DHCX_INT_FF_MASK		BIT(4)
#define ST_ISM330DHCX_INT_WU_MASK		BIT(5)
#define ST_ISM330DHCX_INT_SINGLE_TAP_MASK	BIT(6)
#define ST_ISM330DHCX_INT_SLEEP_CHANGE_MASK	BIT(7)

#define ST_ISM330DHCX_INTERNAL_FREQ_FINE	0x63

#define ST_ISM330DHCX_REG_FIFO_DATA_OUT_TAG_ADDR	0x78

/* embedded registers */
#define ST_ISM330DHCX_EMB_FUNC_EN_A_ADDR	0x04
#define ST_ISM330DHCX_PEDO_EN_MASK		BIT(3)
#define ST_ISM330DHCX_TILT_EN_MASK		BIT(4)
#define ST_ISM330DHCX_SIGN_MOTION_EN_MASK	BIT(5)

#define ST_ISM330DHCX_REG_EMB_FUNC_INT1_ADDR	0x0a
#define ST_ISM330DHCX_REG_EMB_FUNC_INT2_ADDR	0x0e
#define ST_ISM330DHCX_INT_STEP_DET_MASK		BIT(3)
#define ST_ISM330DHCX_INT_TILT_MASK		BIT(4)
#define ST_ISM330DHCX_INT_SIGMOT_MASK		BIT(5)

#define ST_ISM330DHCX_PAGE_RW_ADDR		0x17
#define ST_ISM330DHCX_REG_EMB_FUNC_LIR_MASK	BIT(7)

#define ST_ISM330DHCX_EMB_FUNC_FIFO_CFG_ADDR	0x44
#define ST_ISM330DHCX_PEDO_FIFO_EN_MASK		BIT(6)

#define ST_ISM330DHCX_REG_STEP_COUNTER_L_ADDR	0x62

#define ST_ISM330DHCX_REG_EMB_FUNC_SRC_ADDR	0x64
#define ST_ISM330DHCX_REG_PEDO_RST_STEP_MASK	BIT(7)

/* Timestamp Tick 25us/LSB */
#define ST_ISM330DHCX_TS_DELTA_NS		25000ULL

#define ST_ISM330DHCX_TEMP_GAIN			256
#define ST_ISM330DHCX_TEMP_FS_GAIN		(1000000 / ST_ISM330DHCX_TEMP_GAIN)
#define ST_ISM330DHCX_TEMP_OFFSET		6400

#define ST_ISM330DHCX_SAMPLE_SIZE		6
#define ST_ISM330DHCX_PT_SAMPLE_SIZE		2
#define ST_ISM330DHCX_SC_SAMPLE_SIZE		2
#define ST_ISM330DHCX_TS_SAMPLE_SIZE		4
#define ST_ISM330DHCX_TAG_SIZE			1
#define ST_ISM330DHCX_FIFO_SAMPLE_SIZE		(ST_ISM330DHCX_SAMPLE_SIZE + \
						 ST_ISM330DHCX_TAG_SIZE)
#define ST_ISM330DHCX_MAX_FIFO_DEPTH		416

#define ST_ISM330DHCX_MIN_ODR_IN_EMB_FUNC	26

#define ST_ISM330DHCX_SHIFT_VAL(val, mask)	(((val) << __ffs(mask)) & (mask))

#define ST_ISM330DHCX_DATA_CHANNEL(chan_type, addr, mod, ch2, scan_idx,	\
				rb, sb, sg)				\
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
}

static const struct iio_event_spec st_ism330dhcx_flush_event = {
	.type = STM_IIO_EV_TYPE_FIFO_FLUSH,
	.dir = IIO_EV_DIR_EITHER,
};

static const struct iio_event_spec st_ism330dhcx_thr_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

static const struct iio_event_spec st_ism330dhcx_wakeup_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			 BIT(IIO_EV_INFO_ENABLE) |
			 BIT(IIO_EV_INFO_PERIOD),
};

static const struct iio_event_spec st_ism330dhcx_freefall_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_FALLING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			 BIT(IIO_EV_INFO_ENABLE),
};

static const struct iio_event_spec st_ism330dhcx_6D_event = {
	.type = IIO_EV_TYPE_CHANGE,
	.dir = IIO_EV_DIR_EITHER,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			 BIT(IIO_EV_INFO_ENABLE),
};

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static const struct iio_event_spec st_ism330dhcx_tap_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_SINGLETAP,
	.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
			       BIT(IIO_EV_INFO_ENABLE) |
			       BIT(IIO_EV_INFO_RESET_TIMEOUT),
};

static const struct iio_event_spec st_ism330dhcx_dtap_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_DOUBLETAP,
	.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
			       BIT(IIO_EV_INFO_ENABLE) |
			       BIT(IIO_EV_INFO_RESET_TIMEOUT) |
			       BIT(IIO_EV_INFO_TAP2_MIN_DELAY),
};
#endif /* LINUX_VERSION_CODE */

enum st_ism330dhcx_event_id {
	ST_ISM330DHCX_EVENT_FF,
	ST_ISM330DHCX_EVENT_WAKEUP,
	ST_ISM330DHCX_EVENT_6D,

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ST_ISM330DHCX_EVENT_TAP,
	ST_ISM330DHCX_EVENT_DTAP,
#endif /* LINUX_VERSION_CODE */

	ST_ISM330DHCX_EVENT_STEPC,
	ST_ISM330DHCX_EVENT_SIGNMOT,

	ST_ISM330DHCX_EVENT_MAX
};


#define ST_ISM330DHCX_EVENT_CHANNEL(ctype, etype)	\
{							\
	.type = ctype,					\
	.modified = 0,					\
	.scan_index = -1,				\
	.indexed = -1,					\
	.event_spec = &st_ism330dhcx_##etype##_event,	\
	.num_event_specs = 1,				\
}

/**
 * @struct st_ism330dhcx_reg
 * @brief Generic sensor register description
 *
 * addr: Register arress value.
 * mask: Register bitmask.
 */
struct st_ism330dhcx_reg {
	u8 addr;
	u8 mask;
};

enum st_ism330dhcx_suspend_resume_register {
	ST_ISM330DHCX_CTRL1_XL_REG = 0,
	ST_ISM330DHCX_CTRL2_G_REG,
	ST_ISM330DHCX_REG_CTRL3_C_REG,
	ST_ISM330DHCX_REG_CTRL4_C_REG,
	ST_ISM330DHCX_REG_CTRL5_C_REG,
	ST_ISM330DHCX_REG_CTRL10_C_REG,
	ST_ISM330DHCX_REG_TAP_CFG0_REG,
	ST_ISM330DHCX_REG_TAP_CFG1_REG,
	ST_ISM330DHCX_REG_TAP_CFG2_REG,
	ST_ISM330DHCX_REG_TAP_THS_6D_REG,
	ST_ISM330DHCX_REG_INT_DUR2_REG,
	ST_ISM330DHCX_REG_WAKE_UP_THS_REG,
	ST_ISM330DHCX_REG_WAKE_UP_DUR_REG,
	ST_ISM330DHCX_REG_FREE_FALL_REG,
	ST_ISM330DHCX_REG_INT1_CTRL_REG,
	ST_ISM330DHCX_REG_INT2_CTRL_REG,
	ST_ISM330DHCX_REG_FIFO_CTRL1_REG,
	ST_ISM330DHCX_REG_FIFO_CTRL2_REG,
	ST_ISM330DHCX_REG_FIFO_CTRL3_REG,
	ST_ISM330DHCX_REG_FIFO_CTRL4_REG,
	ST_ISM330DHCX_REG_EMB_FUNC_EN_A_REG,
	ST_ISM330DHCX_REG_EMB_FUNC_FIFO_CFG_REG,
	ST_ISM330DHCX_REG_PAGE_RW_REG,
	ST_ISM330DHCX_SUSPEND_RESUME_REGS,
};

enum st_ism330dhcx_page_sel_register {
	FUNC_CFG_ACCESS_0 = 0,
	FUNC_CFG_ACCESS_SHUB_REG,
	FUNC_CFG_ACCESS_FUNC_CFG,
};

struct st_ism330dhcx_suspend_resume_entry {
	u8 page;
	u8 addr;
	u8 val;
	u8 mask;
};

/**
 * @struct st_ism330dhcx_odr
 * @brief ODR sensor table entry
 *
 * In the ODR table the possible ODR supported by sensor can be defined in the
 * following format:
 *    .odr_avl[0] = {   0, 0,       0x00 },
 *    .odr_avl[1] = {  12, 500000,  0x01 }, ..... it means 12.5 Hz
 *    .odr_avl[2] = {  26, 0,       0x02 }, ..... it means 26.0 Hz
 *
 * hz: Most significant part of ODR value (in Hz).
 * uhz: Least significant part of ODR value (in micro Hz).
 * val: Register value tu set ODR.
 */
struct st_ism330dhcx_odr {
	int hz;
	int uhz;
	u8 val;
};

/**
 * @struct st_ism330dhcx_odr_table_entry
 * @brief ODR sensor table
 *
 * odr_size: ODR table size.
 * reg: Sensor register description for ODR (address and mask).
 * odr_avl: All supported ODR values.
 */
struct st_ism330dhcx_odr_table_entry {
	u8 odr_size;
	struct st_ism330dhcx_reg reg;
	struct st_ism330dhcx_odr odr_avl[ST_ISM330DHCX_ODR_LIST_SIZE];
};

/**
 * @struct st_ism330dhcx_fs
 * @brief Full scale entry
 *
 * gain: The gain to obtain data value from raw data (LSB).
 * val: Register value.
 */
struct st_ism330dhcx_fs {
	u32 gain;
	u8 val;
};

/**
 * @struct st_ism330dhcx_fs_table_entry
 * @brief Full scale table
 *
 * reg: Sensor register description for FS (address and mask).
 * size: Full scale number of entry.
 * fs_avl: Full scale entry.
 */
#define ST_ISM330DHCX_FS_LIST_SIZE			5
#define ST_ISM330DHCX_FS_ACC_LIST_SIZE		4
#define ST_ISM330DHCX_FS_GYRO_LIST_SIZE		5
#define ST_ISM330DHCX_FS_TEMP_LIST_SIZE		1
struct st_ism330dhcx_fs_table_entry {
	struct st_ism330dhcx_reg reg;
	u8 size;
	struct st_ism330dhcx_fs fs_avl[ST_ISM330DHCX_FS_LIST_SIZE];
};

#define ST_ISM330DHCX_ACC_FS_2G_GAIN	IIO_G_TO_M_S_2(61000)
#define ST_ISM330DHCX_ACC_FS_4G_GAIN	IIO_G_TO_M_S_2(122000)
#define ST_ISM330DHCX_ACC_FS_8G_GAIN	IIO_G_TO_M_S_2(244000)
#define ST_ISM330DHCX_ACC_FS_16G_GAIN	IIO_G_TO_M_S_2(488000)

#define ST_ISM330DHCX_GYRO_FS_250_GAIN	IIO_DEGREE_TO_RAD(8750000)
#define ST_ISM330DHCX_GYRO_FS_500_GAIN	IIO_DEGREE_TO_RAD(17500000)
#define ST_ISM330DHCX_GYRO_FS_1000_GAIN	IIO_DEGREE_TO_RAD(35000000)
#define ST_ISM330DHCX_GYRO_FS_2000_GAIN	IIO_DEGREE_TO_RAD(70000000)
#define ST_ISM330DHCX_GYRO_FS_4000_GAIN	IIO_DEGREE_TO_RAD(140000000)

struct st_ism330dhcx_ext_dev_info {
	const struct st_ism330dhcx_ext_dev_settings *ext_dev_settings;
	u8 ext_dev_i2c_addr;
};

/**
 * @enum st_ism330dhcx_sensor_id
 * @brief Sensor Identifier
 */
enum st_ism330dhcx_sensor_id {
	ST_ISM330DHCX_ID_GYRO,
	ST_ISM330DHCX_ID_ACC,
	ST_ISM330DHCX_ID_TEMP,
	ST_ISM330DHCX_ID_EXT0,
	ST_ISM330DHCX_ID_EXT1,
	ST_ISM330DHCX_ID_STEP_COUNTER,
	ST_ISM330DHCX_ID_MAX,
};

static const enum st_ism330dhcx_sensor_id st_ism330dhcx_main_sensor_list[] = {
	 [0] = ST_ISM330DHCX_ID_GYRO,
	 [1] = ST_ISM330DHCX_ID_ACC,
	 [2] = ST_ISM330DHCX_ID_TEMP,
};

static const enum st_ism330dhcx_sensor_id
st_ism330dhcx_buffered_sensor_list[] = {
	[0] = ST_ISM330DHCX_ID_GYRO,
	[1] = ST_ISM330DHCX_ID_ACC,
	[2] = ST_ISM330DHCX_ID_TEMP,
	[3] = ST_ISM330DHCX_ID_EXT0,
	[4] = ST_ISM330DHCX_ID_EXT1,
	[5] = ST_ISM330DHCX_ID_STEP_COUNTER,
};

/**
 * @enum st_ism330dhcx_fifo_mode
 * @brief FIFO Modes
 */
enum st_ism330dhcx_fifo_mode {
	ST_ISM330DHCX_FIFO_BYPASS = 0x0,
	ST_ISM330DHCX_FIFO_CONT = 0x6,
};

/**
 * @enum st_ism330dhcx_fifo_mode - FIFO Buffer Status
 */
enum st_ism330dhcx_fifo_status {
	ST_ISM330DHCX_HW_FLUSH,
	ST_ISM330DHCX_HW_OPERATIONAL,
};

/**
 * @struct st_ism330dhcx_sensor
 * @brief ST IMU sensor instance
 *
 * id: Sensor identifier
 * hw: Pointer to instance of struct st_ism330dhcx_hw
 * ext_dev_info: Sensor hub i2c slave settings.
 * trig: Sensor iio trigger.
 * gain: Configured sensor sensitivity
 * odr: Output data rate of the sensor [Hz]
 * uodr: Output data rate of the sensor [uHz]
 * offset: Sensor data offset
 * decimator: Sensor decimator
 * dec_counter: Sensor decimator counter
 * max_watermark: Max supported watermark level
 * watermark: Sensor watermark level
 * batch_reg: Sensor reg/mask for FIFO batching register
 * last_fifo_timestamp: Store last sample timestamp in FIFO, used by flush
 * selftest_status: Last status of self test output
 * min_st, max_st: Min/Max acc/gyro data values during self test procedure
 */
struct st_ism330dhcx_sensor {
	enum st_ism330dhcx_sensor_id id;
	struct st_ism330dhcx_hw *hw;

	struct st_ism330dhcx_ext_dev_info ext_dev_info;

	struct iio_trigger *trig;

	u32 gain;
	int odr;
	int uodr;

	u32 offset;
	u8 decimator;
	u8 dec_counter;

	u16 max_watermark;
	u16 watermark;

	struct st_ism330dhcx_reg batch_reg;
	s64 last_fifo_timestamp;

	/* self test */
	int8_t selftest_status;
	int min_st;
	int max_st;
};

/**
 * @struct st_ism330dhcx_hw
 * @brief ST IMU MEMS hw instance
 *
 * dev: Pointer to instance of struct device (I2C or SPI).
 * irq: Device interrupt line (I2C or SPI).
 * regmap: Register map of the device.
 * int_pin: Save interrupt pin used by sensor.
 * lock: Mutex to protect read and write operations.
 * fifo_lock: Mutex to prevent concurrent access to the hw FIFO.
 * page_lock: Mutex to prevent concurrent memory page configuration.
 * fifo_mode: FIFO operating mode supported by the device.
 * state: hw operational state.
 * enable_mask: Enabled sensor bitmask.
 * enable_ev_mask: Enabled event bitmask.
 * fsm_enable_mask: FSM Enabled sensor bitmask.
 * embfunc_irq_reg: Embedded function irq configutation register (other).
 * embfunc_pg0_irq_reg: Embedded function irq configutation register (page 0).
 * ext_data_len: Number of i2c slave devices connected to I2C master.
 * odr: Timestamp sample ODR [Hz]
 * uodr: Timestamp sample ODR [uHz]
 * ts_offset: Hw timestamp offset.
 * ts_delta_ns: Delta time since irq.
 * hw_ts: Latest hw timestamp from the sensor.
 * u32 val_ts_old: Manage timestamp rollover.
 * hw_ts_high: Manage timestamp rollover.
 * tsample: Estimated sample timestamp.
 * hw_ts_old: Manage timestamp rollover.
 * delta_ts: Delta time between two consecutive interrupts.
 * ts: Latest timestamp from irq handler.
 * i2c_master_pu: I2C master line Pull Up configuration.
 * module_id: identify iio devices of the same sensor module.
 * has_hw_fifo: FIFO hw support flag.
 * iio_devs: Pointers to iio_dev sensor instances.
 * odr_table: The sensor ODR table.
 * freefall_threshold: Accelerometer threshold for free fall algorithm.
 * wk_th_ug: Wake-up threshold in mg.
 * wk_dur_ms: Wake-up duration in ms.
 * sixD_threshold: 6D threshold in mg.
 * tap_threshold: tap/dtap treshold in mg.
 * tap_quiet_time: tap quiet time in ms.
 * tap_shock_time: tap shock time in ms.
 * dtap_duration: double tap duration time (min time) in ms.
 */
struct st_ism330dhcx_hw {
	struct device *dev;
	int irq;
	struct regmap *regmap;
	int int_pin;

	struct mutex lock;
	struct mutex fifo_lock;
	struct mutex page_lock;

	enum st_ism330dhcx_fifo_mode fifo_mode;
	unsigned long state;
	u32 enable_mask;
	u64 enable_ev_mask;

	u16 fsm_enable_mask;
	u8 irq_reg;
	u8 embfunc_irq_reg;
	u8 embfunc_pg0_irq_reg;

	u8 ext_data_len;

	int odr;
	int uodr;

	s64 ts_offset;
	u64 ts_delta_ns;
	s64 hw_ts;
	u32 val_ts_old;
	u32 hw_ts_high;
	s64 tsample;
	s64 delta_ts;
	s64 ts;
	u8 i2c_master_pu;
	u32 module_id;
	bool has_hw_fifo;

	struct iio_dev *iio_devs[ST_ISM330DHCX_ID_MAX];
	const struct st_ism330dhcx_odr_table_entry *odr_table;
	const struct st_ism330dhcx_fs_table_entry *fs_table;

	u32 freefall_threshold;
	u32 wk_th_mg;
	u32 wk_dur_ms;
	u32 sixD_threshold;
	u32 tap_threshold;
	u32 tap_quiet_time;
	u32 tap_shock_time;
	u32 dtap_duration;
};

/**
 * struct st_ism330dhcx_ff_th - Free Fall threshold table
 * @mg: Threshold in mg.
 * @val: Register value.
 */
struct st_ism330dhcx_ff_th {
	u32 mg;
	u8 val;
};

/**
 * struct st_ism330dhcx_6D_th - 6D threshold table
 * @deg: Threshold in degrees.
 * @val: Register value.
 */
struct st_ism330dhcx_6D_th {
	u8 deg;
	u8 val;
};

/**
 * @struct dev_pm_ops
 * @brief Power mamagement callback function structure
 */
extern const struct dev_pm_ops st_ism330dhcx_pm_ops;

static inline int st_ism330dhcx_manipulate_bit(int int_reg, int irq_mask, int en)
{
	int bit_position = __ffs(irq_mask);
	int bit_mask = 1 << bit_position;

	int_reg &= ~bit_mask;
	int_reg |= (en << bit_position);

	return int_reg;
}

static inline int st_ism330dhcx_read_atomic(struct st_ism330dhcx_hw *hw,
					    u8 addr, unsigned int len,
					    void *data)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_bulk_read(hw->regmap, addr, data, len);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
__maybe_unused st_ism330dhcx_read_with_mask(struct st_ism330dhcx_hw *hw,
					  u8 addr, u8 mask, u8 *val)
{
	u8 data;
	int err;

	err = regmap_bulk_read(hw->regmap, addr, &data, sizeof(data));
	if (err < 0) {
		dev_err(hw->dev, "failed to read %02x register\n", addr);

		goto out;
	}

	*val = (data & mask) >> __ffs(mask);

out:
	return (err < 0) ? err : 0;
}

static inline int st_ism330dhcx_write_atomic(struct st_ism330dhcx_hw *hw,
					     u8 addr, unsigned int len,
					     unsigned int *data)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_bulk_write(hw->regmap, addr, data, len);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int __st_ism330dhcx_write_with_mask(struct st_ism330dhcx_hw *hw,
						  u8 addr, unsigned int mask,
						  unsigned int data)
{
	unsigned int val = ST_ISM330DHCX_SHIFT_VAL(data, mask);
	int err;

	err = regmap_update_bits(hw->regmap, addr, mask, val);

	return err;
}

static inline int st_ism330dhcx_write_with_mask(struct st_ism330dhcx_hw *hw,
						u8 addr, unsigned int mask,
						unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = __st_ism330dhcx_write_with_mask(hw, addr, mask, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int st_ism330dhcx_set_page_access(struct st_ism330dhcx_hw *hw,
						u8 mask, u8 data)
{
	return __st_ism330dhcx_write_with_mask(hw,
					 ST_ISM330DHCX_REG_FUNC_CFG_ACCESS_ADDR,
					 mask, data);
}

static inline bool st_ism330dhcx_is_fifo_enabled(struct st_ism330dhcx_hw *hw)
{
	return hw->enable_mask & (BIT(ST_ISM330DHCX_ID_GYRO)	  |
				  BIT(ST_ISM330DHCX_ID_ACC)	  |
				  BIT(ST_ISM330DHCX_ID_EXT0)	  |
				  BIT(ST_ISM330DHCX_ID_EXT1));
}

int st_ism330dhcx_probe(struct device *dev, int irq,
			struct regmap *regmap);
int st_ism330dhcx_shub_set_enable(struct st_ism330dhcx_sensor *sensor,
				  bool enable);
int st_ism330dhcx_shub_probe(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_shub_read(struct st_ism330dhcx_sensor *sensor, u8 addr,
			    u8 *data, int len);
int st_ism330dhcx_sensor_set_enable(struct st_ism330dhcx_sensor *sensor,
				    bool enable);
int st_ism330dhcx_get_int_reg(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_allocate_sw_trigger(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_hw_trigger_setup(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_get_odr_val(enum st_ism330dhcx_sensor_id id,
			      int odr, int uodr,
			      int *podr, int *puodr, u8 *val);
int st_ism330dhcx_set_odr(struct st_ism330dhcx_sensor *sensor, int req_odr,
			  int req_uodr);
int st_ism330dhcx_update_watermark(struct st_ism330dhcx_sensor *sensor,
				   u16 watermark);
ssize_t st_ism330dhcx_flush_fifo(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size);
ssize_t st_ism330dhcx_get_max_watermark(struct device *dev,
					struct device_attribute *attr,
					char *buf);
ssize_t st_ism330dhcx_get_watermark(struct device *dev,
				    struct device_attribute *attr,
				    char *buf);
ssize_t st_ism330dhcx_set_watermark(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size);
ssize_t st_ism330dhcx_get_module_id(struct device *dev,
				    struct device_attribute *attr,
				    char *buf);

int st_ism330dhcx_set_page_access(struct st_ism330dhcx_hw *hw,
				  u8 mask, u8 data);
int st_ism330dhcx_suspend_fifo(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_set_fifo_mode(struct st_ism330dhcx_hw *hw,
				enum st_ism330dhcx_fifo_mode fifo_mode);
int __st_ism330dhcx_set_sensor_batching_odr(struct st_ism330dhcx_sensor *sensor,
					    bool enable);
int st_ism330dhcx_update_batching(struct iio_dev *iio_dev, bool enable);
int st_ism330dhcx_reset_hwts(struct st_ism330dhcx_hw *hw);

/* xl events */
int st_ism330dhcx_read_event_config(struct iio_dev *iio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir);
int st_ism330dhcx_write_event_config(struct iio_dev *iio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   int enable);
int st_ism330dhcx_read_event_value(struct iio_dev *iio_dev,
				 const struct iio_chan_spec *chan,
				 enum iio_event_type type,
				 enum iio_event_direction dir,
				 enum iio_event_info info,
				 int *val, int *val2);
int st_ism330dhcx_write_event_value(struct iio_dev *iio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir,
				  enum iio_event_info info,
				  int val, int val2);
int st_ism330dhcx_update_threshold_events(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_update_duration_events(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_event_init(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_event_handler(struct st_ism330dhcx_hw *hw);

/* embedded functions: step counter / event detector / significant motion */
int st_ism330dhcx_embfunc_probe(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_embfunc_handler_thread(struct st_ism330dhcx_hw *hw);
int st_ism330dhcx_step_enable(struct st_ism330dhcx_sensor *sensor, bool enable);

#endif /* ST_ISM330DHCX_H */
