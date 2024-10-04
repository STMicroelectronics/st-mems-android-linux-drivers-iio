/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics st_iis2iclx sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
 */

#ifndef ST_IIS2ICLX_H
#define ST_IIS2ICLX_H

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/version.h>

#include "../../common/stm_iio_types.h"

#define ST_IIS2ICLX_DEBUG_DISCHARGE

#define ST_IIS2ICLX_MAX_ODR			833
#define ST_IIS2ICLX_ODR_LIST_SIZE		8
#define ST_IIS2ICLX_ODR_EXPAND(odr, uodr)	((odr * 1000000) + uodr)

#define ST_IIS2ICLX_DEV_NAME			"iis2iclx"

#define ST_IIS2ICLX_DEFAULT_XL_FS_INDEX		2
#define ST_IIS2ICLX_DEFAULT_XL_ODR_INDEX	2
#define ST_IIS2ICLX_DEFAULT_T_FS_INDEX		0
#define ST_IIS2ICLX_DEFAULT_T_ODR_INDEX		1

#define ST_IIS2ICLX_REG_FUNC_CFG_ACCESS_ADDR	0x01
#define ST_IIS2ICLX_SHUB_REG_MASK		BIT(6)
#define ST_IIS2ICLX_FUNC_CFG_MASK		BIT(7)
#define ST_IIS2ICLX_ACCESS_MASK			GENMASK(7, 6)

#define ST_IIS2ICLX_REG_FIFO_CTRL1_ADDR		0x07
#define ST_IIS2ICLX_REG_FIFO_CTRL2_ADDR		0x08
#define ST_IIS2ICLX_FIFO_WTM8_MASK		BIT(0)
#define ST_IIS2ICLX_FIFO_WTM_MASK		GENMASK(8, 0)
#define ST_IIS2ICLX_FIFO_STATUS_DIFF		GENMASK(9, 0)

#define ST_IIS2ICLX_REG_FIFO_CTRL3_ADDR		0x09
#define ST_IIS2ICLX_BDR_XL_MASK			GENMASK(3, 0)

#define ST_IIS2ICLX_REG_FIFO_CTRL4_ADDR		0x0a
#define ST_IIS2ICLX_FIFO_MODE_MASK		GENMASK(2, 0)
#define ST_IIS2ICLX_ODR_T_BATCH_MASK		GENMASK(5, 4)
#define ST_IIS2ICLX_DEC_TS_MASK			GENMASK(7, 6)

#define ST_IIS2ICLX_REG_INT1_CTRL_ADDR		0x0d
#define ST_IIS2ICLX_REG_INT2_CTRL_ADDR		0x0e
#define ST_IIS2ICLX_INT_FIFO_TH_MASK		BIT(3)

#define ST_IIS2ICLX_REG_WHOAMI_ADDR		0x0f
#define ST_IIS2ICLX_WHOAMI_VAL			0x6b

#define ST_IIS2ICLX_REG_CTRL1_XL_ADDR		0x10
#define ST_IIS2ICLX_LPF2_XL_EN_MASK		BIT(1)
#define ST_IIS2ICLX_FS_XL_MASK			GENMASK(3, 2)
#define ST_IIS2ICLX_ODR_XL_MASK			GENMASK(7, 4)

#define ST_IIS2ICLX_REG_CTRL3_C_ADDR		0x12
#define ST_IIS2ICLX_SW_RESET_MASK		BIT(0)
#define ST_IIS2ICLX_PP_OD_MASK			BIT(4)
#define ST_IIS2ICLX_H_LACTIVE_MASK		BIT(5)
#define ST_IIS2ICLX_BDU_MASK			BIT(6)
#define ST_IIS2ICLX_BOOT_MASK			BIT(7)

#define ST_IIS2ICLX_REG_CTRL4_C_ADDR		0x13
#define ST_IIS2ICLX_DRDY_MASK			BIT(3)

#define ST_IIS2ICLX_REG_CTRL5_C_ADDR		0x14
#define ST_IIS2ICLX_ST_XL_MASK			GENMASK(1, 0)
#define ST_IIS2ICLX_SELFTEST_MIN		737
#define ST_IIS2ICLX_SELFTEST_MAX		13934

#define ST_IIS2ICLX_SELF_TEST_DISABLED_VAL	0
#define ST_IIS2ICLX_SELF_TEST_POS_SIGN_VAL	1
#define ST_IIS2ICLX_SELF_TEST_NEG_SIGN_VAL	2

#define ST_IIS2ICLX_REG_CTRL8_XL_ADDR		0x17
#define ST_IIS2ICLX_HPCF_XL_MASK		GENMASK(7, 5)

#define ST_IIS2ICLX_REG_CTRL9_XL_ADDR		0x18
#define ST_IIS2ICLX_DEVICE_CONF_MASK		BIT(1)

#define ST_IIS2ICLX_REG_CTRL10_C_ADDR		0x19
#define ST_IIS2ICLX_TIMESTAMP_EN_MASK		BIT(5)

#define ST_IIS2ICLX_REG_ALL_INT_SRC_ADDR	0x1a
#define ST_IIS2ICLX_WU_IA_MASK			BIT(1)
#define ST_IIS2ICLX_SLEEP_CHANGE_MASK		BIT(5)

#define ST_IIS2ICLX_REG_WAKE_UP_SRC_ADDR	0x1b
#define ST_IIS2ICLX_WAKE_UP_EVENT_MASK		GENMASK(3, 1)
#define ST_IIS2ICLX_WAKE_UP_SRC_WU_IA_MASK	BIT(3)
#define ST_IIS2ICLX_X_WU_MASK			BIT(2)
#define ST_IIS2ICLX_Y_WU_MASK			BIT(1)

#define ST_IIS2ICLX_REG_TAP_SRC_ADDR		0x1c
#define ST_IIS2ICLX_Y_TAP_MASK			BIT(1)
#define ST_IIS2ICLX_X_TAP_MASK			BIT(2)
#define ST_IIS2ICLX_TAP_SIGN_MASK		BIT(3)
#define ST_IIS2ICLX_DOUBLE_TAP_IA_MASK		BIT(4)
#define ST_IIS2ICLX_SINGLE_TAP_IA_MASK		BIT(5)
#define ST_IIS2ICLX_XY_TAP_IA_MASK		GENMASK(2, 1)

#define ST_IIS2ICLX_REG_STATUS_ADDR		0x1e
#define ST_IIS2ICLX_STATUS_XLDA			BIT(0)

#define ST_IIS2ICLX_REG_OUT_TEMP_L_ADDR		0x20

#define ST_IIS2ICLX_REG_OUTX_L_A_ADDR		0x28
#define ST_IIS2ICLX_REG_OUTY_L_A_ADDR		0x2a
#define ST_IIS2ICLX_REG_OUTZ_L_A_ADDR		0x2c

#define ST_IIS2ICLX_REG_FSM_STATUS_A_MAINPAGE	0x36
#define ST_IIS2ICLX_REG_FSM_STATUS_B_MAINPAGE	0x37
#define ST_IIS2ICLX_REG_MLC_STATUS_MAINPAGE	0x38

#define ST_IIS2ICLX_REG_TIMESTAMP0_ADDR		0x40

#define ST_IIS2ICLX_REG_TAP_CFG0_ADDR		0x56
#define ST_IIS2ICLX_LIR_MASK			BIT(0)
#define ST_IIS2ICLX_TAP_Y_EN_MASK		BIT(2)
#define ST_IIS2ICLX_TAP_X_EN_MASK		BIT(3)
#define ST_IIS2ICLX_SLOPE_FDS_MASK		BIT(4)
#define ST_IIS2ICLX_INT_CLR_ON_READ_MASK	BIT(6)
#define ST_IIS2ICLX_TAP_EN_MASK			GENMASK(3, 2)

#define ST_IIS2ICLX_REG_TAP_CFG1_ADDR		0x57
#define ST_IIS2ICLX_TAP_THS_X_MASK		GENMASK(4, 0)
#define ST_IIS2ICLX_TAP_PRIORITY_MASK		BIT(5)

#define ST_IIS2ICLX_REG_TAP_CFG2_ADDR		0x58
#define ST_IIS2ICLX_TAP_THS_Y_MASK		GENMASK(4, 0)
#define ST_IIS2ICLX_INTERRUPTS_ENABLE_MASK	BIT(7)

#define ST_IIS2ICLX_REG_INT_DUR2_ADDR		0x5a
#define ST_IIS2ICLX_SHOCK_MASK			GENMASK(1, 0)
#define ST_IIS2ICLX_QUIET_MASK			GENMASK(3, 2)
#define ST_IIS2ICLX_DUR_MASK			GENMASK(7, 4)

#define ST_IIS2ICLX_REG_WAKE_UP_THS_ADDR	0x5b
#define ST_IIS2ICLX_WAKE_UP_THS_MASK		GENMASK(5, 0)
#define ST_IIS2ICLX_SINGLE_DOUBLE_TAP_MASK	BIT(7)

#define ST_IIS2ICLX_REG_WAKE_UP_DUR_ADDR	0x5c
#define ST_IIS2ICLX_WAKE_UP_DUR_MASK		GENMASK(6, 5)

#define ST_IIS2ICLX_REG_MD1_CFG_ADDR		0x5e
#define ST_IIS2ICLX_REG_MD2_CFG_ADDR		0x5f
#define ST_IIS2ICLX_INT_EMB_FUNC_MASK		BIT(1)
#define ST_IIS2ICLX_INT_DOUBLE_TAP_MASK		BIT(3)
#define ST_IIS2ICLX_INT_WU_MASK			BIT(5)
#define ST_IIS2ICLX_INT_SINGLE_TAP_MASK		BIT(6)
#define ST_IIS2ICLX_INT_SLEEP_CHANGE_MASK	BIT(7)

#define ST_IIS2ICLX_REG_INTERNAL_FREQ_FINE	0x63

/* shub registers */
#define ST_IIS2ICLX_REG_MASTER_CONFIG_ADDR	0x14
#define ST_IIS2ICLX_MASTER_ON_MASK		BIT(2)
#define ST_IIS2ICLX_SHUB_PU_EN_MASK		BIT(3)
#define ST_IIS2ICLX_WRITE_ONCE_MASK		BIT(6)

#define ST_IIS2ICLX_REG_SLV0_ADDR		0x15
#define ST_IIS2ICLX_REG_SLV0_CFG		0x17
#define ST_IIS2ICLX_REG_SLV1_ADDR		0x18
#define ST_IIS2ICLX_REG_SLV2_ADDR		0x1b
#define ST_IIS2ICLX_REG_SLV3_ADDR		0x1e

#define ST_IIS2ICLX_REG_DATAWRITE_SLV0_ADDR	0x21
#define ST_IIS2ICLX_SLAVE_NUMOP_MASK		GENMASK(2, 0)
#define ST_IIS2ICLX_BATCH_EXT_SENS_EN_MASK	BIT(3)

#define ST_IIS2ICLX_REG_SLV0_OUT_ADDR		0x02

/* embedded function registers */
#define ST_IIS2ICLX_REG_PAGE_SEL_ADDR		0x02
#define ST_IIS2ICLX_PAGE_SEL_MASK		GENMASK(7, 4)

#define ST_IIS2ICLX_REG_EMB_FUNC_EN_B_ADDR	0x05
#define ST_IIS2ICLX_FSM_EN_MASK			BIT(0)
#define ST_IIS2ICLX_MLC_EN_MASK			BIT(4)

#define ST_IIS2ICLX_REG_FSM_INT1_A_ADDR		0x0b
#define ST_IIS2ICLX_REG_FSM_INT1_B_ADDR		0x0c
#define ST_IIS2ICLX_REG_MLC_INT1_ADDR		0x0d

#define ST_IIS2ICLX_REG_FSM_INT2_A_ADDR		0x0f
#define ST_IIS2ICLX_REG_FSM_INT2_B_ADDR		0x10
#define ST_IIS2ICLX_REG_MLC_INT2_ADDR		0x11

#define ST_IIS2ICLX_REG_MLC_STATUS_ADDR		0x15

#define ST_IIS2ICLX_REG_PAGE_RW			0x17
#define ST_IIS2ICLX_PAGE_READ_MASK		BIT(5)
#define ST_IIS2ICLX_PAGE_WRITE_MASK		BIT(6)
#define ST_IIS2ICLX_EMB_FUNC_LIR_MASK		BIT(7)

#define ST_IIS2ICLX_REG_FSM_ENABLE_A_ADDR	0x46
#define ST_IIS2ICLX_REG_FSM_ENABLE_B_ADDR	0x47

#define ST_IIS2ICLX_REG_FSM_OUTS1_ADDR		0x4c

#define ST_IIS2ICLX_REG_EMB_FUNC_INIT_B_ADDR	0x67
#define ST_IIS2ICLX_FSM_INIT_MASK		BIT(0)
#define ST_IIS2ICLX_MLC_INIT_MASK		BIT(4)

#define ST_IIS2ICLX_REG_MLC0_SRC_ADDR		0x70

/* timestamp Tick 25us/LSB */
#define ST_IIS2ICLX_TS_DELTA_NS			25000ULL

/* temperature in uC */
#define ST_IIS2ICLX_TEMP_GAIN			256
#define ST_IIS2ICLX_TEMP_FS_GAIN		(1000000 / ST_IIS2ICLX_TEMP_GAIN)
#define ST_IIS2ICLX_TEMP_OFFSET			6400

/* fifo simple size and depth */
#define ST_IIS2ICLX_SAMPLE_SIZE			6
#define ST_IIS2ICLX_TS_SAMPLE_SIZE		4
#define ST_IIS2ICLX_TAG_SIZE			1
#define ST_IIS2ICLX_FIFO_SAMPLE_SIZE		(ST_IIS2ICLX_SAMPLE_SIZE + \
						 ST_IIS2ICLX_TAG_SIZE)
#define ST_IIS2ICLX_MAX_FIFO_DEPTH		416

#define ST_IIS2ICLX_DEFAULT_KTIME		200000000
#define ST_IIS2ICLX_FAST_KTIME			5000000
#define ST_IIS2ICLX_FAST_TO_DEFAULT		10

/* full scales */
#define ST_IIS2ICLX_FS_LIST_SIZE		6
#define ST_IIS2ICLX_FS_ACC_LIST_SIZE		4
#define ST_IIS2ICLX_FS_TEMP_LIST_SIZE		1

#define ST_IIS2ICLX_ACC_FS_05G_GAIN		IIO_G_TO_M_S_2(15000)
#define ST_IIS2ICLX_ACC_FS_1G_GAIN		IIO_G_TO_M_S_2(31000)
#define ST_IIS2ICLX_ACC_FS_2G_GAIN		IIO_G_TO_M_S_2(61000)
#define ST_IIS2ICLX_ACC_FS_3G_GAIN		IIO_G_TO_M_S_2(122000)

#define ST_IIS2ICLX_DATA_CHANNEL(chan_type, addr, mod, ch2, scan_idx,	\
				 rb, sb, sg, ex_info)			\
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
	.ext_info = ex_info,						\
}

static const struct iio_event_spec st_iis2iclx_flush_event = {
	.type = STM_IIO_EV_TYPE_FIFO_FLUSH,
	.dir = IIO_EV_DIR_EITHER,
};

static const struct iio_event_spec st_iis2iclx_thr_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};

static const struct iio_event_spec st_iis2iclx_wakeup_event = {
	.type = IIO_EV_TYPE_THRESH,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE) |
			 BIT(IIO_EV_INFO_PERIOD),
};

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static const struct iio_event_spec st_iis2iclx_tap_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_SINGLETAP,
	.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
			       BIT(IIO_EV_INFO_ENABLE) |
			       BIT(IIO_EV_INFO_RESET_TIMEOUT),
};

static const struct iio_event_spec st_iis2iclx_dtap_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_DOUBLETAP,
	.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
			       BIT(IIO_EV_INFO_ENABLE) |
			       BIT(IIO_EV_INFO_RESET_TIMEOUT) |
			       BIT(IIO_EV_INFO_TAP2_MIN_DELAY),
};
#endif /* LINUX_VERSION_CODE */

enum st_iis2iclx_event_id {
	ST_IIS2ICLX_EVENT_WAKEUP,

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ST_IIS2ICLX_EVENT_TAP,
	ST_IIS2ICLX_EVENT_DTAP,
#endif /* LINUX_VERSION_CODE */

	ST_IIS2ICLX_EVENT_MAX
};

#define ST_IIS2ICLX_EVENT_CHANNEL(ctype, etype)		\
{							\
	.type = ctype,					\
	.modified = 0,					\
	.scan_index = -1,				\
	.indexed = -1,					\
	.event_spec = &st_iis2iclx_##etype##_event,	\
	.num_event_specs = 1,				\
}

#define ST_IIS2ICLX_SHIFT_VAL(val, mask)	(((val) << __ffs(mask)) & \
						 (mask))

enum st_iis2iclx_fsm_mlc_enable_id {
	ST_IIS2ICLX_MLC_FSM_DISABLED = 0,
	ST_IIS2ICLX_MLC_ENABLED = BIT(0),
	ST_IIS2ICLX_FSM_ENABLED = BIT(1),
};

/**
 * struct mlc_config_t - MLC/FSM configuration management
 *
 * @mlc_int_mask: interrupt register mask
 * @fsm_enabled_mask: enable fsm register mask
 * @fsm_int_mask: interrupt register mask
 * @mlc_configured: number of mlc configured
 * @fsm_configured: number of fsm configured
 * @bin_len: fw binary size
 * @mlc_int_pin: where route mlc int pin
 * @mlc_fsm_en: mlc and fsm enable bit reported by ucf file
 * @fsm_mlc_requested_odr, fsm_mlc_requested_uodr: Min required ODR by
 * MLC/FSM.
 * @status: mlc/fsm status
 */
struct st_iis2iclx_mlc_config_t {
	u8 mlc_int_mask;
	u8 fsm_enabled_mask[2];
	u8 fsm_int_mask[2];
	u8 mlc_configured;
	u8 fsm_configured;
	u16 bin_len;
	int mlc_int_pin;
	u8 mlc_fsm_en;
	u16 fsm_mlc_requested_odr;
	u32 fsm_mlc_requested_uodr;
	enum st_iis2iclx_fsm_mlc_enable_id status;
};

/**
 * struct st_iis2iclx_reg - Generic sensor register description (addr + mask)
 *
 * @addr: Address of register.
 * @mask: Bitmask register for proper usage.
 */
struct st_iis2iclx_reg {
	u8 addr;
	u8 mask;
};

enum st_iis2iclx_suspend_resume_register {
	ST_IIS2ICLX_CTRL1_XL_REG = 0,
	ST_IIS2ICLX_REG_CTRL3_C_REG,
	ST_IIS2ICLX_REG_CTRL4_C_REG,
	ST_IIS2ICLX_REG_CTRL5_C_REG,
	ST_IIS2ICLX_REG_CTRL6_C_REG,
	ST_IIS2ICLX_REG_CTRL8_XL_REG,
	ST_IIS2ICLX_REG_CTRL10_C_REG,
	ST_IIS2ICLX_REG_TAP_CFG0_REG,
	ST_IIS2ICLX_REG_TAP_CFG1_REG,
	ST_IIS2ICLX_REG_TAP_CFG2_REG,
	ST_IIS2ICLX_REG_INT_DUR2_REG,
	ST_IIS2ICLX_REG_WAKE_UP_THS_REG,
	ST_IIS2ICLX_REG_WAKE_UP_DUR_REG,
	ST_IIS2ICLX_REG_MD1_CFG_REG,
	ST_IIS2ICLX_REG_MD2_CFG_REG,
	ST_IIS2ICLX_REG_INT1_CTRL_REG,
	ST_IIS2ICLX_REG_INT2_CTRL_REG,
	ST_IIS2ICLX_REG_FIFO_CTRL1_REG,
	ST_IIS2ICLX_REG_FIFO_CTRL2_REG,
	ST_IIS2ICLX_REG_FIFO_CTRL3_REG,
	ST_IIS2ICLX_REG_FIFO_CTRL4_REG,
	ST_IIS2ICLX_REG_EMB_FUNC_EN_B_REG,
	ST_IIS2ICLX_REG_FSM_INT1_A_REG,
	ST_IIS2ICLX_REG_FSM_INT1_B_REG,
	ST_IIS2ICLX_REG_MLC_INT1_REG,
	ST_IIS2ICLX_REG_FSM_INT2_A_REG,
	ST_IIS2ICLX_REG_FSM_INT2_B_REG,
	ST_IIS2ICLX_REG_MLC_INT2_REG,
	ST_IIS2ICLX_SUSPEND_RESUME_REGS,
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
enum st_iis2iclx_page_sel_register {
	FUNC_CFG_ACCESS_0 = 0,
	FUNC_CFG_ACCESS_SHUB_REG,
	FUNC_CFG_ACCESS_FUNC_CFG,
};

/**
 * struct st_iis2iclx_suspend_resume_entry - Register value for
 * backup/restore
 *
 * @page: Page bank reg map.
 * @addr: Address of register.
 * @val: Register value.
 * @mask: Bitmask register for proper usage.
 */
struct st_iis2iclx_suspend_resume_entry {
	u8 page;
	u8 addr;
	u8 val;
	u8 mask;
};

/**
 * struct st_iis2iclx_odr - Single ODR entry
 *
 * @hz: Most significant part of the sensor ODR (Hz).
 * @uhz: Less significant part of the sensor ODR (micro Hz).
 * @val: ODR register value.
 * @batch_val: Batching ODR register value.
 */
struct st_iis2iclx_odr {
	u16 hz;
	u32 uhz;
	u8 val;
	u8 batch_val;
};

/**
 * struct st_iis2iclx_odr_table_entry - Sensor ODR table
 *
 * @size: Size of ODR table.
 * @reg: ODR register.
 * @batching_reg: ODR register for batching on fifo.
 * @odr_avl: Array of supported ODR value.
 */
struct st_iis2iclx_odr_table_entry {
	u8 size;
	struct st_iis2iclx_reg reg;
	struct st_iis2iclx_reg batching_reg;
	struct st_iis2iclx_odr odr_avl[ST_IIS2ICLX_ODR_LIST_SIZE];
};

/**
 * struct st_iis2iclx_fs - Full Scale sensor table entry
 *
 * @reg: Register description for FS settings.
 * @gain: Sensor sensitivity (mdps/LSB, mg/LSB and uC/LSB).
 * @val: FS register value.
 */
struct st_iis2iclx_fs {
	u32 gain;
	u8 val;
};

/**
 * struct st_iis2iclx_fs_table_entry - Full Scale sensor table
 *
 * @reg: Full Scale device register.
 * @fs_avl: Full Scale list entries.
 * @size: Full Scale sensor table size.
 */
struct st_iis2iclx_fs_table_entry {
	struct st_iis2iclx_reg reg;
	struct st_iis2iclx_fs fs_avl[ST_IIS2ICLX_FS_LIST_SIZE];
	u8 size;
};

enum st_iis2iclx_sensor_id {
	ST_IIS2ICLX_ID_ACC,
	ST_IIS2ICLX_ID_TEMP,
	ST_IIS2ICLX_ID_HW = ST_IIS2ICLX_ID_TEMP,
	ST_IIS2ICLX_ID_EXT0,
	ST_IIS2ICLX_ID_EXT1,
	ST_IIS2ICLX_ID_MLC,
	ST_IIS2ICLX_ID_MLC_0,
	ST_IIS2ICLX_ID_MLC_1,
	ST_IIS2ICLX_ID_MLC_2,
	ST_IIS2ICLX_ID_MLC_3,
	ST_IIS2ICLX_ID_MLC_4,
	ST_IIS2ICLX_ID_MLC_5,
	ST_IIS2ICLX_ID_MLC_6,
	ST_IIS2ICLX_ID_MLC_7,
	ST_IIS2ICLX_ID_FSM_0,
	ST_IIS2ICLX_ID_FSM_1,
	ST_IIS2ICLX_ID_FSM_2,
	ST_IIS2ICLX_ID_FSM_3,
	ST_IIS2ICLX_ID_FSM_4,
	ST_IIS2ICLX_ID_FSM_5,
	ST_IIS2ICLX_ID_FSM_6,
	ST_IIS2ICLX_ID_FSM_7,
	ST_IIS2ICLX_ID_FSM_8,
	ST_IIS2ICLX_ID_FSM_9,
	ST_IIS2ICLX_ID_FSM_10,
	ST_IIS2ICLX_ID_FSM_11,
	ST_IIS2ICLX_ID_FSM_12,
	ST_IIS2ICLX_ID_FSM_13,
	ST_IIS2ICLX_ID_FSM_14,
	ST_IIS2ICLX_ID_FSM_15,
	ST_IIS2ICLX_ID_MAX,
};

static const enum st_iis2iclx_sensor_id st_iis2iclx_mlc_sensor_list[] = {
	 [0] = ST_IIS2ICLX_ID_MLC_0,
	 [1] = ST_IIS2ICLX_ID_MLC_1,
	 [2] = ST_IIS2ICLX_ID_MLC_2,
	 [3] = ST_IIS2ICLX_ID_MLC_3,
	 [4] = ST_IIS2ICLX_ID_MLC_4,
	 [5] = ST_IIS2ICLX_ID_MLC_5,
	 [6] = ST_IIS2ICLX_ID_MLC_6,
	 [7] = ST_IIS2ICLX_ID_MLC_7,
};

static const enum st_iis2iclx_sensor_id st_iis2iclx_fsm_sensor_list[] = {
	 [0] = ST_IIS2ICLX_ID_FSM_0,
	 [1] = ST_IIS2ICLX_ID_FSM_1,
	 [2] = ST_IIS2ICLX_ID_FSM_2,
	 [3] = ST_IIS2ICLX_ID_FSM_3,
	 [4] = ST_IIS2ICLX_ID_FSM_4,
	 [5] = ST_IIS2ICLX_ID_FSM_5,
	 [6] = ST_IIS2ICLX_ID_FSM_6,
	 [7] = ST_IIS2ICLX_ID_FSM_7,
	 [8] = ST_IIS2ICLX_ID_FSM_8,
	 [9] = ST_IIS2ICLX_ID_FSM_9,
	 [10] = ST_IIS2ICLX_ID_FSM_10,
	 [11] = ST_IIS2ICLX_ID_FSM_11,
	 [12] = ST_IIS2ICLX_ID_FSM_12,
	 [13] = ST_IIS2ICLX_ID_FSM_13,
	 [14] = ST_IIS2ICLX_ID_FSM_14,
	 [15] = ST_IIS2ICLX_ID_FSM_15,
};

#define ST_IIS2ICLX_ID_ALL_FSM_MLC (BIT_ULL(ST_IIS2ICLX_ID_MLC_0)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_MLC_1)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_MLC_2)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_MLC_3)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_MLC_4)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_MLC_5)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_MLC_6)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_MLC_7)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_0)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_1)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_2)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_3)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_4)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_5)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_6)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_7)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_8)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_9)  | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_10) | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_11) | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_12) | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_13) | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_14) | \
				    BIT_ULL(ST_IIS2ICLX_ID_FSM_15))

/* hw devices that can wakeup the target */
#define ST_IIS2ICLX_WAKE_UP_SENSORS (BIT_ULL(ST_IIS2ICLX_ID_ACC) | \
				     ST_IIS2ICLX_ID_ALL_FSM_MLC)

/* this is the minimal ODR for wake-up sensors and dependencies */
#define ST_IIS2ICLX_MIN_ODR_IN_WAKEUP		26
#define ST_IIS2ICLX_MIN_ODR_IN_FREEFALL		26
#define ST_IIS2ICLX_MIN_ODR_IN_TAP		416
#define ST_IIS2ICLX_MIN_ODR_IN_DTAP		416

enum st_iis2iclx_fifo_mode {
	ST_IIS2ICLX_FIFO_BYPASS = 0x0,
	ST_IIS2ICLX_FIFO_CONT = 0x6,
};

enum {
	ST_IIS2ICLX_HW_FLUSH,
	ST_IIS2ICLX_HW_OPERATIONAL,
};

struct st_iis2iclx_ext_dev_info {
	const struct st_iis2iclx_ext_dev_settings *ext_dev_settings;
	u8 ext_dev_i2c_addr;
};

/**
 * struct st_iis2iclx_sensor - ST sensor instance
 *
 * @name: Sensor name.
 * @id: Sensor identifier.
 * @hw: Pointer to instance of struct st_iis2iclx_hw.
 * @ext_dev_info: For sensor hub indicate device info struct.
 * @trig: Trigger used by IIO event sensors.
 * @gain: Configured sensor sensitivity.
 * @offset: Sensor data offset.
 * @decimator: Sensor decimator
 * @dec_counter: Sensor decimator counter
 * @discard_samples: Sensor discard sample counter, used when LPF enabled.
 * @odr: Output data rate of the sensor [Hz].
 * @uodr: Output data rate of the sensor [uHz].
 * @discharged_samples: Report number of samples discharded by drdy mask filters.
 * @max_watermark: Max supported watermark level.
 * @watermark: Sensor watermark level.
 * @last_fifo_timestamp: Store last sample timestamp in FIFO, used by flush
 * @selftest_status: Last status of self test output
 * @min_st, @max_st: Min/Max accel data values during self test procedure
 * @status_reg: Generic status register used by MLC/FSM.
 * @outreg_addr: MLC/FSM output data registers.
 * @status: MLC/FSM enable status.
 * @conf: Used in case of sensor event to manage configuration.
 * @scan: Scan buffer for triggered sensors event.
 */
struct st_iis2iclx_sensor {
	char name[32];
	enum st_iis2iclx_sensor_id id;
	struct st_iis2iclx_hw *hw;
	struct st_iis2iclx_ext_dev_info ext_dev_info;
	struct iio_trigger *trig;

	/* sensor with odrs, gain and offset */
	u32 gain;
	u32 offset;
	u32 decimator;
	u32 dec_counter;
	u32 discard_samples;
	int odr;
	int uodr;

#ifdef ST_IIS2ICLX_DEBUG_DISCHARGE
	u32 discharged_samples;
#endif /* ST_IIS2ICLX_DEBUG_DISCHARGE */

	u16 max_watermark;
	u16 watermark;
	s64 last_fifo_timestamp;

	/* self test */
	int8_t selftest_status;

	/* mlc / fsm registers */
	u8 status_reg;
	u8 outreg_addr;
	enum st_iis2iclx_fsm_mlc_enable_id status;
};

/**
 * struct st_iis2iclx_hw - ST MEMS hw instance
 *
 * @dev: Pointer to instance of struct device (I2C or SPI).
 * @irq: Device interrupt line (I2C or SPI).
 * @regmap: Register map of the device.
 * @int_pin: Save interrupt pin used by sensor.
 * @lock: Mutex to protect read and write operations.
 * @fifo_lock: Mutex to prevent concurrent access to the hw FIFO.
 * @page_lock: Mutex to prevent concurrent memory page configuration.
 * @fifo_mode: FIFO operating mode supported by the device.
 * @state: hw operational state.
 * @enable_mask: Enabled sensor bitmask.
 * @enable_ev_mask: Enabled event bitmask.
 * @ext_data_len: SHUB external sensor data len.
 * @hw_timestamp_global: hw timestamp value always monotonic where the most
 *                       significant 8byte are incremented at every disable/enable.
 * @timesync_workqueue: runs the async task in private workqueue.
 * @timesync_work: actual work to be done in the async task workqueue.
 * @timesync_timer: hrtimer used to schedule period read for the async task.
 * @hwtimestamp_lock: spinlock for the 64bit timestamp value.
 * @timesync_ktime: interval value used by the hrtimer.
 * @timestamp_c: counter used for counting number of timesync updates.
 * @ts_offset: Hw timestamp offset.
 * @ts_delta_ns: Calibrate delta time tick.
 * @hw_ts: Latest hw timestamp from the sensor.
 * @val_ts_old: Hold hw timestamp for timer rollover.
 * @hw_ts_high: Save MSB hw timestamp.
 * @tsample: Timestamp for each sensor sample.
 * @delta_ts: Delta time between two consecutive interrupts.
 * @irq_ts: Latest timestamp from irq handler.
 * @i2c_master_pu: I2C master line Pull Up configuration.
 * @module_id: identify iio devices of the same sensor module.
 * @odr_table_entry: Sensors ODR table.
 * @iio_devs: Pointers to iio_dev instances.
 * @vdd_supply: Voltage regulator for VDD.
 * @vddio_supply: Voltage regulator for VDDIIO.
 * @orientation: sensor chip orientation relative to main hardware.
 * @mlc_config: Pointer to MLC/FSM configuration structure.
 * @preload_mlc: Indicate to preload firmware for MLC/FSM.
 * @enable_drdy_mask: Indicate if drdy mask is enabled.
 * @xl_odr_div: Configured accel odr bandwidth.
 * @drdy_reg: Interrupt configuration register.
 * @embfunc_pg0_irq_reg: Embedded function irq configuration register (page 0).
 * @wk_th_mg: Wake-up threshold in mg.
 * @wk_dur_ms: Wake-up duration in ms.
 * @tap_threshold: tap/dtap treshold in mg.
 * @tap_quiet_time: tap quiet time in ms.
 * @tap_shock_time: tap shock time in ms.
 * @dtap_duration: tap duration time in ms.
 */
struct st_iis2iclx_hw {
	struct device *dev;
	int irq;
	struct regmap *regmap;
	int int_pin;

	struct mutex lock;
	struct mutex fifo_lock;
	struct mutex page_lock;

	enum st_iis2iclx_fifo_mode fifo_mode;
	unsigned long state;
	u64 enable_mask;
	u64 enable_ev_mask;
	u64 requested_mask;
	u8 ext_data_len;
	s64 hw_timestamp_global;

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
	struct workqueue_struct *timesync_workqueue;
	struct work_struct timesync_work;
	struct hrtimer timesync_timer;
	spinlock_t hwtimestamp_lock;
	ktime_t timesync_ktime;
	int timesync_c[ST_IIS2ICLX_ID_HW + 1];
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

	s64 ts_offset;
	u64 ts_delta_ns;
	s64 hw_ts;
	u32 val_ts_old;
	u32 hw_ts_high;
	s64 tsample;
	s64 delta_ts;
	s64 irq_ts;
	u8 i2c_master_pu;
	u32 module_id;

	const struct st_iis2iclx_odr_table_entry *odr_table_entry;
	const struct st_iis2iclx_fs_table_entry *fs_table;
	struct iio_dev *iio_devs[ST_IIS2ICLX_ID_MAX];

	struct regulator *vdd_supply;
	struct regulator *vddio_supply;

	struct iio_mount_matrix orientation;

	struct st_iis2iclx_mlc_config_t *mlc_config;
	bool preload_mlc;
	bool enable_drdy_mask;
	int xl_odr_div;

	u8 drdy_reg;
	u8 embfunc_pg0_irq_reg;
	u32 wk_th_mg;
	u32 wk_dur_ms;
	u32 tap_threshold;
	u32 tap_quiet_time;
	u32 tap_shock_time;
	u32 dtap_duration;
};

/**
 * struct st_iis2iclx_xl_lpf_bw_t - Accel Low Pass Filter bandwidth
 *
 * @lpf2_xl_en: Enable LPF2 filter (0 = disable, 1 = enable).
 * @val: Selected BW register value.
 * @div: BW divider.
 */
struct st_iis2iclx_xl_lpf_bw_t {
	u8 lpf2_xl_en;
	u8 val;
	u16 div;
};

/**
 * struct st_iis2iclx_xl_lpf_bw_config_t - Accel Low Pass Filter bandwidth
 *                                         configuration
 *
 * @reg: Register address involved in Accel LPF configuration.
 * @mask: Register mask involved in Accel LPF configuration (HPCF_XL).
 * @size: st_iis2iclx_xl_lpf_bw_t array size.
 * @st_iis2iclx_xl_lpf_bw_t: Accel Low Pass Filter bandwidth data struct.
 */
struct st_iis2iclx_xl_lpf_bw_config_t {
	u8 reg;
	u8 mask;
	u8 size;
	struct st_iis2iclx_xl_lpf_bw_t st_iis2iclx_xl_lpf_bw[9];
};

extern const struct dev_pm_ops st_iis2iclx_pm_ops;

static inline int st_iis2iclx_manipulate_bit(int int_reg, int irq_mask, int en)
{
	int bit_position = __ffs(irq_mask);
	int bit_mask = 1 << bit_position;

	int_reg &= ~bit_mask;
	int_reg |= (en << bit_position);

	return int_reg;
}

static inline int __st_iis2iclx_write_with_mask(struct st_iis2iclx_hw *hw,
						unsigned int addr,
						unsigned int mask,
						unsigned int data)
{
	int err;
	unsigned int val = ST_IIS2ICLX_SHIFT_VAL(data, mask);

	err = regmap_update_bits(hw->regmap, addr, mask, val);

	return err;
}

static inline int
st_iis2iclx_update_bits_locked(struct st_iis2iclx_hw *hw, unsigned int addr,
			       unsigned int mask, unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = __st_iis2iclx_write_with_mask(hw, addr, mask, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

/* use when mask is constant */
static inline int
st_iis2iclx_write_with_mask_locked(struct st_iis2iclx_hw *hw, unsigned int addr,
				   unsigned int mask, unsigned int data)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = __st_iis2iclx_write_with_mask(hw, addr, mask, data);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
st_iis2iclx_read_locked(struct st_iis2iclx_hw *hw, unsigned int addr,
			void *val, unsigned int len)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_bulk_read(hw->regmap, addr, val, len);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
__maybe_unused st_iis2iclx_read_with_mask(struct st_iis2iclx_hw *hw,
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

static inline int
st_iis2iclx_write_locked(struct st_iis2iclx_hw *hw, unsigned int addr,
			 unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = regmap_write(hw->regmap, addr, val);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int st_iis2iclx_set_page_access(struct st_iis2iclx_hw *hw,
					      unsigned int val,
					      unsigned int mask)
{
	return regmap_update_bits(hw->regmap,
				  ST_IIS2ICLX_REG_FUNC_CFG_ACCESS_ADDR,
				  mask,
				  ST_IIS2ICLX_SHIFT_VAL(val, mask));
}

static inline bool st_iis2iclx_is_fifo_enabled(struct st_iis2iclx_hw *hw)
{
	return hw->enable_mask & (BIT_ULL(ST_IIS2ICLX_ID_ACC));
}

static inline s64 st_iis2iclx_get_time_ns(struct iio_dev *iio_dev)
{
	return iio_get_time_ns(iio_dev);
}

static inline int
st_iis2iclx_read_page_locked(struct st_iis2iclx_hw *hw, unsigned int addr,
			     void *val, unsigned int len)
{
	int err;

	mutex_lock(&hw->page_lock);
	st_iis2iclx_set_page_access(hw, true, ST_IIS2ICLX_FUNC_CFG_MASK);
	err = regmap_bulk_read(hw->regmap, addr, val, len);
	st_iis2iclx_set_page_access(hw, false, ST_IIS2ICLX_FUNC_CFG_MASK);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
st_iis2iclx_write_page_locked(struct st_iis2iclx_hw *hw, unsigned int addr,
			      unsigned int *val, unsigned int len)
{
	int err;

	mutex_lock(&hw->page_lock);
	st_iis2iclx_set_page_access(hw, true, ST_IIS2ICLX_FUNC_CFG_MASK);
	err = regmap_bulk_write(hw->regmap, addr, val, len);
	st_iis2iclx_set_page_access(hw, false, ST_IIS2ICLX_FUNC_CFG_MASK);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline int
st_iis2iclx_update_page_bits_locked(struct st_iis2iclx_hw *hw,
				    unsigned int addr, unsigned int mask,
				    unsigned int val)
{
	int err;

	mutex_lock(&hw->page_lock);
	st_iis2iclx_set_page_access(hw, true, ST_IIS2ICLX_FUNC_CFG_MASK);
	err = regmap_update_bits(hw->regmap, addr, mask, val);
	st_iis2iclx_set_page_access(hw, false, ST_IIS2ICLX_FUNC_CFG_MASK);
	mutex_unlock(&hw->page_lock);

	return err;
}

static inline bool st_iis2iclx_fsm_running(struct st_iis2iclx_hw *hw)
{
	return hw->enable_mask & GENMASK_ULL(ST_IIS2ICLX_ID_FSM_15,
					     ST_IIS2ICLX_ID_FSM_0);
}

static inline bool st_iis2iclx_mlc_running(struct st_iis2iclx_hw *hw)
{
	return hw->enable_mask &
	       GENMASK_ULL(ST_IIS2ICLX_ID_MLC_7,
			   ST_IIS2ICLX_ID_MLC_0);
}

int st_iis2iclx_probe(struct device *dev, int irq, struct regmap *regmap);
void st_iis2iclx_remove(struct device *dev);
int st_iis2iclx_set_odr(struct st_iis2iclx_sensor *sensor,
			int req_odr, int req_uodr);
int st_iis2iclx_get_int_reg(struct st_iis2iclx_hw *hw);
int st_iis2iclx_sensor_set_enable(struct st_iis2iclx_sensor *sensor,
				  bool enable);
int st_iis2iclx_buffers_setup(struct st_iis2iclx_hw *hw);
int st_iis2iclx_get_odr_from_reg(enum st_iis2iclx_sensor_id id,
				 u8 reg_val, u16 *podr, u32 *puodr);
int st_iis2iclx_get_batch_val(struct st_iis2iclx_sensor *sensor,
			      int odr, int uodr, u8 *val);
int st_iis2iclx_update_watermark(struct st_iis2iclx_sensor *sensor,
				 u16 watermark);
ssize_t st_iis2iclx_flush_fifo(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size);
ssize_t st_iis2iclx_get_max_watermark(struct device *dev,
				      struct device_attribute *attr, char *buf);
ssize_t st_iis2iclx_get_watermark(struct device *dev,
				  struct device_attribute *attr, char *buf);
ssize_t st_iis2iclx_set_watermark(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size);
ssize_t st_iis2iclx_get_module_id(struct device *dev,
				  struct device_attribute *attr, char *buf);

int st_iis2iclx_suspend_fifo(struct st_iis2iclx_hw *hw);
int st_iis2iclx_set_fifo_mode(struct st_iis2iclx_hw *hw,
			      enum st_iis2iclx_fifo_mode fifo_mode);
int __st_iis2iclx_set_sensor_batching_odr(struct st_iis2iclx_sensor *sensor,
					  bool enable);
int st_iis2iclx_update_batching(struct iio_dev *iio_dev, bool enable);
int st_iis2iclx_reset_hwts(struct st_iis2iclx_hw *hw);
int st_iis2iclx_shub_probe(struct st_iis2iclx_hw *hw);
int st_iis2iclx_shub_set_enable(struct st_iis2iclx_sensor *sensor, bool enable);
int st_iis2iclx_of_get_pin(struct st_iis2iclx_hw *hw, int *pin);

/* xl events */
int st_iis2iclx_read_event_config(struct iio_dev *iio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir);
int st_iis2iclx_write_event_config(struct iio_dev *iio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   int enable);
int st_iis2iclx_read_event_value(struct iio_dev *iio_dev,
				 const struct iio_chan_spec *chan,
				 enum iio_event_type type,
				 enum iio_event_direction dir,
				 enum iio_event_info info,
				 int *val, int *val2);
int st_iis2iclx_write_event_value(struct iio_dev *iio_dev,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir,
				  enum iio_event_info info,
				  int val, int val2);
int st_iis2iclx_event_init(struct st_iis2iclx_hw *hw);
int st_iis2iclx_event_handler(struct st_iis2iclx_hw *hw);
int st_iis2iclx_update_threshold_events(struct st_iis2iclx_hw *hw);
int st_iis2iclx_update_duration_events(struct st_iis2iclx_hw *hw);

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
int st_iis2iclx_hwtimesync_init(struct st_iis2iclx_hw *hw);
#else /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */
static inline int st_iis2iclx_hwtimesync_init(struct st_iis2iclx_hw *hw)
{
	return 0;
}
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

int st_iis2iclx_mlc_probe(struct st_iis2iclx_hw *hw);
int st_iis2iclx_mlc_remove(struct device *dev);
int st_iis2iclx_mlc_check_status(struct st_iis2iclx_hw *hw);
int st_iis2iclx_mlc_init_preload(struct st_iis2iclx_hw *hw);

#endif /* ST_IIS2ICLX_H */
