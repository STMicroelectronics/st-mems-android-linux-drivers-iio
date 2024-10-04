// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_iis2iclx sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
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

#include "st_iis2iclx.h"

static struct st_iis2iclx_selftest_table {
	char *string_mode;
	u8 accel_value;
} st_iis2iclx_selftest_table[] = {
	[0] = {
		.string_mode = "disabled",
		.accel_value = ST_IIS2ICLX_SELF_TEST_DISABLED_VAL,
	},
	[1] = {
		.string_mode = "positive-sign",
		.accel_value = ST_IIS2ICLX_SELF_TEST_POS_SIGN_VAL,
	},
	[2] = {
		.string_mode = "negative-sign",
		.accel_value = ST_IIS2ICLX_SELF_TEST_NEG_SIGN_VAL,
	},
};

static const int st_iis2iclx_odr_index[] = {
	12, 26, 52, 104, 208, 416, 833
};

static const int st_iis2iclx_odr_divider_index[] = {
	2, 4, 10, 20, 45, 100, 200, 400, 800
};

/*
 * LPF filter configuration
 *
 * the total amount of sample to discard is set to the value of
 * samples_to_discard plus the settling_samples related to the LPF
 * configuration and the sensor odr set.
 */
static const struct st_iis2iclx_lpf_discard_table_t {
	u32 samples_to_discard[ST_IIS2ICLX_ODR_LIST_SIZE];
	u32 settling_samples[9][ST_IIS2ICLX_ODR_LIST_SIZE];
} st_iis2iclx_lpf_discard_table[ST_IIS2ICLX_ODR_LIST_SIZE] = {
	[ST_IIS2ICLX_ID_ACC] = {
		/* samples_to_discard when no filter enabled */
		.samples_to_discard = { 3, 3, 3, 3, 3, 3, 3, 3 },

		/* settling_samples vs ODRs and accel Bandwidth table */
		.settling_samples[0] = {   0,   0,   0,   0,   0,   0,   0 },
		.settling_samples[1] = {   0,   0,   0,   0,   0,   0,   0 },
		.settling_samples[2] = {  10,  10,  10,  10,  10,  10,  10 },
		.settling_samples[3] = {  19,  19,  19,  19,  19,  19,  19 },
		.settling_samples[4] = {  38,  38,  38,  38,  38,  38,  38 },
		.settling_samples[5] = {  75,  75,  75,  75,  75,  75,  75 },
		.settling_samples[6] = { 150, 150, 150, 150, 150, 150, 150 },
		.settling_samples[7] = { 296, 296, 296, 296, 296, 296, 296 },
		.settling_samples[8] = { 595, 595, 595, 595, 595, 595, 595 },
	},
};

static struct st_iis2iclx_suspend_resume_entry
	st_iis2iclx_suspend_resume[ST_IIS2ICLX_SUSPEND_RESUME_REGS] = {
	[ST_IIS2ICLX_CTRL1_XL_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_CTRL1_XL_ADDR,
		.mask = ST_IIS2ICLX_FS_XL_MASK |
			ST_IIS2ICLX_LPF2_XL_EN_MASK,
	},
	[ST_IIS2ICLX_REG_CTRL3_C_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_CTRL3_C_ADDR,
		.mask = ST_IIS2ICLX_BDU_MASK |
			ST_IIS2ICLX_PP_OD_MASK |
			ST_IIS2ICLX_H_LACTIVE_MASK,
	},
	[ST_IIS2ICLX_REG_CTRL4_C_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_CTRL4_C_ADDR,
		.mask = ST_IIS2ICLX_DRDY_MASK,
	},
	[ST_IIS2ICLX_REG_CTRL8_XL_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_CTRL8_XL_ADDR,
		.mask = ST_IIS2ICLX_HPCF_XL_MASK,
	},
	[ST_IIS2ICLX_REG_CTRL10_C_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_CTRL10_C_ADDR,
		.mask = ST_IIS2ICLX_TIMESTAMP_EN_MASK,
	},
	[ST_IIS2ICLX_REG_TAP_CFG0_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_TAP_CFG0_ADDR,
		.mask = ST_IIS2ICLX_LIR_MASK,
	},
	[ST_IIS2ICLX_REG_INT1_CTRL_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_INT1_CTRL_ADDR,
		.mask = ST_IIS2ICLX_INT_FIFO_TH_MASK,
	},
	[ST_IIS2ICLX_REG_INT2_CTRL_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_INT2_CTRL_ADDR,
		.mask = ST_IIS2ICLX_INT_FIFO_TH_MASK,
	},
	[ST_IIS2ICLX_REG_FIFO_CTRL1_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_FIFO_CTRL1_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_IIS2ICLX_REG_FIFO_CTRL2_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_FIFO_CTRL2_ADDR,
		.mask = ST_IIS2ICLX_FIFO_WTM8_MASK,
	},
	[ST_IIS2ICLX_REG_FIFO_CTRL3_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_FIFO_CTRL3_ADDR,
		.mask = ST_IIS2ICLX_BDR_XL_MASK,
	},
	[ST_IIS2ICLX_REG_FIFO_CTRL4_REG] = {
		.page = FUNC_CFG_ACCESS_0,
		.addr = ST_IIS2ICLX_REG_FIFO_CTRL4_ADDR,
		.mask = ST_IIS2ICLX_DEC_TS_MASK |
			ST_IIS2ICLX_ODR_T_BATCH_MASK,
	},
	[ST_IIS2ICLX_REG_EMB_FUNC_EN_B_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_IIS2ICLX_REG_EMB_FUNC_EN_B_ADDR,
		.mask = ST_IIS2ICLX_FSM_EN_MASK |
			ST_IIS2ICLX_MLC_EN_MASK,
	},
	[ST_IIS2ICLX_REG_FSM_INT1_A_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_IIS2ICLX_REG_FSM_INT1_A_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_IIS2ICLX_REG_FSM_INT1_B_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_IIS2ICLX_REG_FSM_INT1_B_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_IIS2ICLX_REG_MLC_INT1_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_IIS2ICLX_REG_MLC_INT1_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_IIS2ICLX_REG_FSM_INT2_A_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_IIS2ICLX_REG_FSM_INT2_A_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_IIS2ICLX_REG_FSM_INT2_B_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_IIS2ICLX_REG_FSM_INT2_B_ADDR,
		.mask = GENMASK(7, 0),
	},
	[ST_IIS2ICLX_REG_MLC_INT2_REG] = {
		.page = FUNC_CFG_ACCESS_FUNC_CFG,
		.addr = ST_IIS2ICLX_REG_MLC_INT2_ADDR,
		.mask = GENMASK(7, 0),
	},
};

static const struct st_iis2iclx_odr_table_entry st_iis2iclx_odr_table[] = {
	[ST_IIS2ICLX_ID_ACC] = {
		.size = 7,
		.reg = {
			.addr = ST_IIS2ICLX_REG_CTRL1_XL_ADDR,
			.mask = ST_IIS2ICLX_ODR_XL_MASK,
		},
		.batching_reg = {
			.addr = ST_IIS2ICLX_REG_FIFO_CTRL3_ADDR,
			.mask = ST_IIS2ICLX_BDR_XL_MASK,
		},
		.odr_avl[0] = {  12, 500000,  0x01,  0x01 },
		.odr_avl[1] = {  26,      0,  0x02,  0x02 },
		.odr_avl[2] = {  52,      0,  0x03,  0x03 },
		.odr_avl[3] = { 104,      0,  0x04,  0x04 },
		.odr_avl[4] = { 208,      0,  0x05,  0x05 },
		.odr_avl[5] = { 416,      0,  0x06,  0x06 },
		.odr_avl[6] = { 833,      0,  0x07,  0x07 },
	},
	[ST_IIS2ICLX_ID_TEMP] = {
		.size = 2,
		.batching_reg = {
			.addr = ST_IIS2ICLX_REG_FIFO_CTRL4_ADDR,
			.mask = ST_IIS2ICLX_ODR_T_BATCH_MASK,
		},
		.odr_avl[0] = { 12, 500000,   0x02,  0x02 },
		.odr_avl[1] = { 52,      0,   0x03,  0x03 },
	},
};

static const struct st_iis2iclx_fs_table_entry st_iis2iclx_fs_table[] = {
	[ST_IIS2ICLX_ID_ACC] = {
		.size = ST_IIS2ICLX_FS_ACC_LIST_SIZE,
		.reg = {
			.addr = ST_IIS2ICLX_REG_CTRL1_XL_ADDR,
			.mask = ST_IIS2ICLX_FS_XL_MASK,
		},
		.fs_avl[0] = { ST_IIS2ICLX_ACC_FS_05G_GAIN, 0x0 },
		.fs_avl[1] = { ST_IIS2ICLX_ACC_FS_1G_GAIN,  0x2 },
		.fs_avl[2] = { ST_IIS2ICLX_ACC_FS_2G_GAIN,  0x3 },
		.fs_avl[3] = { ST_IIS2ICLX_ACC_FS_3G_GAIN,  0x1 },
	},
	[ST_IIS2ICLX_ID_TEMP] = {
		.size = ST_IIS2ICLX_FS_TEMP_LIST_SIZE,
		.fs_avl[0] = { ST_IIS2ICLX_TEMP_FS_GAIN, 0x0 },
	},
};

#ifdef CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES
static const struct st_iis2iclx_6D_th st_iis2iclx_6D_threshold[] = {
	[0] = {
		.val = 0x00,
		.deg = 80,
	},
	[1] = {
		.val = 0x01,
		.deg = 70,
	},
	[2] = {
		.val = 0x02,
		.deg = 60,
	},
	[3] = {
		.val = 0x03,
		.deg = 50,
	},
};
#endif /* CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES */

static const struct st_iis2iclx_xl_lpf_bw_config_t st_iis2iclx_xl_bw = {
	.reg = ST_IIS2ICLX_REG_CTRL8_XL_ADDR,
	.mask = ST_IIS2ICLX_HPCF_XL_MASK,
	.size = 9,
	.st_iis2iclx_xl_lpf_bw[0] = {
		.lpf2_xl_en = 0,
		.div = 2,
		.val = 0,
	},
	.st_iis2iclx_xl_lpf_bw[1] = {
		.lpf2_xl_en = 1,
		.div = 4,
		.val = 0,
	},
	.st_iis2iclx_xl_lpf_bw[2] = {
		.lpf2_xl_en = 1,
		.div = 10,
		.val = 1,
	},
	.st_iis2iclx_xl_lpf_bw[3] = {
		.lpf2_xl_en = 1,
		.div = 20,
		.val = 2,
	},
	.st_iis2iclx_xl_lpf_bw[4] = {
		.lpf2_xl_en = 1,
		.div = 45,
		.val = 3,
	},
	.st_iis2iclx_xl_lpf_bw[5] = {
		.lpf2_xl_en = 1,
		.div = 100,
		.val = 4,
	},
	.st_iis2iclx_xl_lpf_bw[6] = {
		.lpf2_xl_en = 1,
		.div = 200,
		.val = 5,
	},
	.st_iis2iclx_xl_lpf_bw[7] = {
		.lpf2_xl_en = 1,
		.div = 400,
		.val = 6,
	},
	.st_iis2iclx_xl_lpf_bw[8] = {
		.lpf2_xl_en = 1,
		.div = 800,
		.val = 7,
	},
};

static const inline struct iio_mount_matrix *
st_iis2iclx_get_mount_matrix(const struct iio_dev *iio_dev,
			     const struct iio_chan_spec *chan)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	struct st_iis2iclx_hw *hw = sensor->hw;

	return &hw->orientation;
}

static const struct iio_chan_spec_ext_info st_iis2iclx_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_ALL, st_iis2iclx_get_mount_matrix),
	{},
};

#define IIO_CHAN_HW_TIMESTAMP(si) {				\
	.type = IIO_COUNT,					\
	.address = ST_IIS2ICLX_REG_TIMESTAMP0_ADDR,		\
	.scan_index = si,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 64,					\
		.storagebits = 64,				\
		.endianness = IIO_LE,				\
	},							\
}

static const struct iio_chan_spec st_iis2iclx_acc_channels[] = {
	ST_IIS2ICLX_DATA_CHANNEL(IIO_ACCEL, ST_IIS2ICLX_REG_OUTX_L_A_ADDR,
				 1, IIO_MOD_X, 0, 16, 16, 's',
				 st_iis2iclx_ext_info),
	ST_IIS2ICLX_DATA_CHANNEL(IIO_ACCEL, ST_IIS2ICLX_REG_OUTY_L_A_ADDR,
				 1, IIO_MOD_Y, 1, 16, 16, 's',
				 st_iis2iclx_ext_info),
	ST_IIS2ICLX_EVENT_CHANNEL(IIO_ACCEL, flush),
	IIO_CHAN_HW_TIMESTAMP(2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static __maybe_unused const struct iio_chan_spec st_iis2iclx_temp_channels[] = {
	{
		.type = IIO_TEMP,
		.address = ST_IIS2ICLX_REG_OUT_TEMP_L_ADDR,
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
	ST_IIS2ICLX_EVENT_CHANNEL(IIO_TEMP, flush),
	IIO_CHAN_HW_TIMESTAMP(1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static inline int st_iis2iclx_get_odr_index(int odr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_iis2iclx_odr_index); i++)
		if (st_iis2iclx_odr_index[i] >= odr)
			break;

	if (i == ARRAY_SIZE(st_iis2iclx_odr_index))
		return -EINVAL;

	return i;
}

static inline int st_iis2iclx_get_odr_divider_index(int odr_div)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_iis2iclx_odr_divider_index); i++)
		if (st_iis2iclx_odr_divider_index[i] >= odr_div)
			break;

	if (i == ARRAY_SIZE(st_iis2iclx_odr_divider_index))
		return -EINVAL;

	return i;
}

int __maybe_unused st_iis2iclx_read_with_mask(struct st_iis2iclx_hw *hw,
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

int st_iis2iclx_of_get_pin(struct st_iis2iclx_hw *hw, int *pin)
{
	if (!dev_fwnode(hw->dev))
		return -EINVAL;

	return device_property_read_u32(hw->dev, "st,int-pin", pin);
}

static int st_iis2iclx_get_int_reg(struct st_iis2iclx_hw *hw, u8 *drdy_reg)
{
	int err = 0, int_pin;

	if (st_iis2iclx_of_get_pin(hw, &int_pin) < 0) {
		struct st_sensors_platform_data *pdata;
		struct device *dev = hw->dev;

		pdata = (struct st_sensors_platform_data *)dev->platform_data;
		int_pin = pdata ? pdata->drdy_int_pin : 1;
	}

	switch (int_pin) {
	case 1:
		*drdy_reg = ST_IIS2ICLX_REG_INT1_CTRL_ADDR;
		break;
	case 2:
		*drdy_reg = ST_IIS2ICLX_REG_INT2_CTRL_ADDR;
		break;
	default:
		dev_err(hw->dev, "unsupported interrupt pin\n");
		err = -EINVAL;
		break;
	}

	hw->int_pin = int_pin;

	return err;
}

static int __maybe_unused st_iis2iclx_bk_regs(struct st_iis2iclx_hw *hw)
{
	unsigned int data;
	bool restore = 0;
	int i, err = 0;

	mutex_lock(&hw->page_lock);

	for (i = 0; i < ST_IIS2ICLX_SUSPEND_RESUME_REGS; i++) {
		if (st_iis2iclx_suspend_resume[i].page != FUNC_CFG_ACCESS_0) {
			err = regmap_update_bits(hw->regmap,
				     ST_IIS2ICLX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_IIS2ICLX_ACCESS_MASK,
				     FIELD_PREP(ST_IIS2ICLX_ACCESS_MASK,
					   st_iis2iclx_suspend_resume[i].page));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_iis2iclx_suspend_resume[i].addr);
				break;
			}

			restore = 1;
		}

		err = regmap_read(hw->regmap,
				  st_iis2iclx_suspend_resume[i].addr,
				  &data);
		if (err < 0) {
			dev_err(hw->dev,
				"failed to save register %02x\n",
				st_iis2iclx_suspend_resume[i].addr);
			goto out_lock;
		}

		if (restore) {
			err = regmap_update_bits(hw->regmap,
				     ST_IIS2ICLX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_IIS2ICLX_ACCESS_MASK,
				     FIELD_PREP(ST_IIS2ICLX_ACCESS_MASK,
						FUNC_CFG_ACCESS_0));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_iis2iclx_suspend_resume[i].addr);
				break;
			}

			restore = 0;
		}

		st_iis2iclx_suspend_resume[i].val = data;
	}

out_lock:
	mutex_unlock(&hw->page_lock);

	return err;
}

static int __maybe_unused st_iis2iclx_restore_regs(struct st_iis2iclx_hw *hw)
{
	bool restore = 0;
	int i, err = 0;

	mutex_lock(&hw->page_lock);

	for (i = 0; i < ST_IIS2ICLX_SUSPEND_RESUME_REGS; i++) {
		if (st_iis2iclx_suspend_resume[i].page != FUNC_CFG_ACCESS_0) {
			err = regmap_update_bits(hw->regmap,
				     ST_IIS2ICLX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_IIS2ICLX_ACCESS_MASK,
				     FIELD_PREP(ST_IIS2ICLX_ACCESS_MASK,
					   st_iis2iclx_suspend_resume[i].page));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_iis2iclx_suspend_resume[i].addr);
				break;
			}

			restore = 1;
		}

		err = regmap_update_bits(hw->regmap,
					 st_iis2iclx_suspend_resume[i].addr,
					 st_iis2iclx_suspend_resume[i].mask,
					 st_iis2iclx_suspend_resume[i].val);
		if (err < 0) {
			dev_err(hw->dev,
				"failed to update %02x reg\n",
				st_iis2iclx_suspend_resume[i].addr);
			break;
		}

		if (restore) {
			err = regmap_update_bits(hw->regmap,
				     ST_IIS2ICLX_REG_FUNC_CFG_ACCESS_ADDR,
				     ST_IIS2ICLX_ACCESS_MASK,
				     FIELD_PREP(ST_IIS2ICLX_ACCESS_MASK,
						FUNC_CFG_ACCESS_0));
			if (err < 0) {
				dev_err(hw->dev,
					"failed to update %02x reg\n",
					st_iis2iclx_suspend_resume[i].addr);
				break;
			}

			restore = 0;
		}
	}

	mutex_unlock(&hw->page_lock);

	return err;
}

static int
st_iis2iclx_set_selftest(struct st_iis2iclx_sensor *sensor, int index)
{
	if (sensor->id != ST_IIS2ICLX_ID_ACC)
		return -EINVAL;

	return st_iis2iclx_update_bits_locked(sensor->hw,
				ST_IIS2ICLX_REG_CTRL5_C_ADDR,
				ST_IIS2ICLX_ST_XL_MASK,
				st_iis2iclx_selftest_table[index].accel_value);
}

static ssize_t
st_iis2iclx_sysfs_get_selftest_available(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sprintf(buf, "%s, %s\n",
		       st_iis2iclx_selftest_table[1].string_mode,
		       st_iis2iclx_selftest_table[2].string_mode);
}

static ssize_t
st_iis2iclx_sysfs_get_selftest_status(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	enum st_iis2iclx_sensor_id id = sensor->id;
	char *message = NULL;
	int8_t result;

	if (id != ST_IIS2ICLX_ID_ACC)
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

#ifdef CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES
/*
 * st_iis2iclx_set_wake_up_thershold - set wake-up threshold in ug
 * @hw - ST MEMS hw instance
 * @th_ug - wake-up threshold in ug (micro g)
 *
 * wake-up thershold register val = (th_ug * 2 ^ 6) / (1000000 * FS_XL)
 */
int st_iis2iclx_set_wake_up_thershold(struct st_iis2iclx_hw *hw, int th_ug)
{
	struct st_iis2iclx_sensor *sensor;
	struct iio_dev *iio_dev;
	u8 val, fs_xl, max_th;
	int tmp, err;

	err = st_iis2iclx_read_with_mask(hw,
		st_iis2iclx_fs_table[ST_IIS2ICLX_ID_ACC].reg.addr,
		st_iis2iclx_fs_table[ST_IIS2ICLX_ID_ACC].reg.mask,
		&fs_xl);
	if (err < 0)
		return err;

	tmp = (th_ug * 64) / (fs_xl * 1000000);
	val = (u8)tmp;
	max_th = ST_IIS2ICLX_WAKE_UP_THS_MASK >>
		  __ffs(ST_IIS2ICLX_WAKE_UP_THS_MASK);
	if (val > max_th)
		val = max_th;

	err = st_iis2iclx_write_with_mask_locked(hw,
				ST_IIS2ICLX_REG_WAKE_UP_THS_ADDR,
				ST_IIS2ICLX_WAKE_UP_THS_MASK, val);
	if (err < 0)
		return err;

	iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_WK];
	sensor = iio_priv(iio_dev);
	sensor->conf[0] = th_ug;

	return 0;
}

/*
 * st_iis2iclx_set_wake_up_duration - set wake-up duration in ms
 * @hw - ST MEMS hw instance
 * @dur_ms - wake-up duration in ms
 *
 * wake-up duration register val is related to XL ODR
 */
int st_iis2iclx_set_wake_up_duration(struct st_iis2iclx_hw *hw, int dur_ms)
{
	struct st_iis2iclx_sensor *sensor;
	struct iio_dev *iio_dev;
	int i, tmp, sensor_odr, err;
	u8 val, odr_xl, max_dur;

	err = st_iis2iclx_read_with_mask(hw,
			st_iis2iclx_odr_table[ST_IIS2ICLX_ID_ACC].reg.addr,
			st_iis2iclx_odr_table[ST_IIS2ICLX_ID_ACC].reg.mask,
			&odr_xl);
	if (err < 0)
		return err;

	if (odr_xl == 0)
		odr_xl = st_iis2iclx_odr_table[ST_IIS2ICLX_ID_ACC].odr_avl[ST_IIS2ICLX_DEFAULT_XL_ODR_INDEX].val;

	for (i = 0; i < st_iis2iclx_odr_table[ST_IIS2ICLX_ID_ACC].size; i++) {
		if (odr_xl ==
		     st_iis2iclx_odr_table[ST_IIS2ICLX_ID_ACC].odr_avl[i].val)
			break;
	}

	if (i == st_iis2iclx_odr_table[ST_IIS2ICLX_ID_ACC].size)
		return -EINVAL;


	sensor_odr = ST_IIS2ICLX_ODR_EXPAND(
		st_iis2iclx_odr_table[ST_IIS2ICLX_ID_ACC].odr_avl[i].hz,
		st_iis2iclx_odr_table[ST_IIS2ICLX_ID_ACC].odr_avl[i].uhz);

	tmp = dur_ms / (1000000 / (sensor_odr / 1000));
	val = (u8)tmp;
	max_dur = ST_IIS2ICLX_WAKE_UP_DUR_MASK >>
		  __ffs(ST_IIS2ICLX_WAKE_UP_DUR_MASK);
	if (val > max_dur)
		val = max_dur;

	err = st_iis2iclx_write_with_mask_locked(hw,
				ST_IIS2ICLX_REG_WAKE_UP_DUR_ADDR,
				ST_IIS2ICLX_WAKE_UP_DUR_MASK, val);
	if (err < 0)
		return err;

	iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_WK];
	sensor = iio_priv(iio_dev);
	sensor->conf[1] = dur_ms;

	return 0;
}

/*
 * st_iis2iclx_set_6D_threshold - set 6D threshold detection in degrees
 * @hw - ST MEMS hw instance
 * @deg - 6D threshold in degrees
 */
int st_iis2iclx_set_6D_threshold(struct st_iis2iclx_hw *hw, int deg)
{
	struct st_iis2iclx_sensor *sensor;
	struct iio_dev *iio_dev;
	int i, err;

	for (i = 0; i < ARRAY_SIZE(st_iis2iclx_6D_threshold); i++) {
		if (deg >= st_iis2iclx_6D_threshold[i].deg)
			break;
	}

	if (i == ARRAY_SIZE(st_iis2iclx_6D_threshold))
		return -EINVAL;

	err = st_iis2iclx_write_with_mask_locked(hw,
				ST_IIS2ICLX_REG_THS_6D_ADDR,
				ST_IIS2ICLX_SIXD_THS_MASK,
				st_iis2iclx_6D_threshold[i].val);
	if (err < 0)
		return err;

	iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_6D];
	sensor = iio_priv(iio_dev);
	sensor->conf[3] = deg;

	return 0;
}
#endif /* CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES */

static __maybe_unused int st_iis2iclx_reg_access(struct iio_dev *iio_dev,
						 unsigned int reg,
						 unsigned int writeval,
						 unsigned int *readval)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
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

static int st_iis2iclx_set_page_0(struct st_iis2iclx_hw *hw)
{
	return regmap_write(hw->regmap,
			    ST_IIS2ICLX_REG_FUNC_CFG_ACCESS_ADDR, 0);
}

static int st_iis2iclx_check_whoami(struct st_iis2iclx_hw *hw)
{
	int err, data;

	err = regmap_read(hw->regmap, ST_IIS2ICLX_REG_WHOAMI_ADDR, &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");

		return err;
	}

	if (data != ST_IIS2ICLX_WHOAMI_VAL) {
		dev_err(hw->dev, "unsupported whoami [%02x]\n", data);

		return -ENODEV;
	}

	return 0;
}

static int st_iis2iclx_get_odr_calibration(struct st_iis2iclx_hw *hw)
{
	s64 odr_calib;
	int data;
	int err;

	err = regmap_read(hw->regmap, ST_IIS2ICLX_REG_INTERNAL_FREQ_FINE,
			  &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read %d register\n",
			ST_IIS2ICLX_REG_INTERNAL_FREQ_FINE);
		return err;
	}

	odr_calib = ((s8)data * 37500) / 1000;
	hw->ts_delta_ns = ST_IIS2ICLX_TS_DELTA_NS - odr_calib;

	dev_info(hw->dev, "Freq Fine %lld (ts %lld)\n",
		 odr_calib, hw->ts_delta_ns);

	return 0;
}

static int st_iis2iclx_set_full_scale(struct st_iis2iclx_sensor *sensor,
				      u32 gain)
{
	enum st_iis2iclx_sensor_id id = sensor->id;
	struct st_iis2iclx_hw *hw = sensor->hw;
	int i, err;
	u8 val;

	/* for other sensors gain is fixed */
	if (id > ST_IIS2ICLX_ID_ACC)
		return 0;

	for (i = 0; i < st_iis2iclx_fs_table[id].size; i++)
		if (st_iis2iclx_fs_table[id].fs_avl[i].gain >= gain)
			break;

	if (i == st_iis2iclx_fs_table[id].size)
		return -EINVAL;

	val = st_iis2iclx_fs_table[id].fs_avl[i].val;
	err = regmap_update_bits(hw->regmap,
				 st_iis2iclx_fs_table[id].reg.addr,
				 st_iis2iclx_fs_table[id].reg.mask,
				 ST_IIS2ICLX_SHIFT_VAL(val,
					    st_iis2iclx_fs_table[id].reg.mask));
	if (err < 0)
		return err;

	sensor->gain = st_iis2iclx_fs_table[id].fs_avl[i].gain;

	return 0;
}

int st_iis2iclx_get_odr_val(enum st_iis2iclx_sensor_id id, int odr,
			    int uodr, int *podr, int *puodr, u8 *val)
{
	int required_odr = ST_IIS2ICLX_ODR_EXPAND(odr, uodr);
	int sensor_odr;
	int i;

	for (i = 0; i < st_iis2iclx_odr_table[id].size; i++) {
		sensor_odr = ST_IIS2ICLX_ODR_EXPAND(
				st_iis2iclx_odr_table[id].odr_avl[i].hz,
				st_iis2iclx_odr_table[id].odr_avl[i].uhz);
		if (sensor_odr >= required_odr)
			break;
	}

	if (i == st_iis2iclx_odr_table[id].size)
		return -EINVAL;

	*val = st_iis2iclx_odr_table[id].odr_avl[i].val;

	if (podr && puodr) {
		*podr = st_iis2iclx_odr_table[id].odr_avl[i].hz;
		*puodr = st_iis2iclx_odr_table[id].odr_avl[i].uhz;
	}

	return 0;
}

int __maybe_unused
st_iis2iclx_get_odr_from_reg(enum st_iis2iclx_sensor_id id,
			     u8 reg_val, u16 *podr, u32 *puodr)
{
	int i;

	for (i = 0; i < st_iis2iclx_odr_table[id].size; i++) {
		if (reg_val == st_iis2iclx_odr_table[id].odr_avl[i].val)
			break;
	}

	if (i == st_iis2iclx_odr_table[id].size)
		return -EINVAL;

	*podr = st_iis2iclx_odr_table[id].odr_avl[i].hz;
	*puodr = st_iis2iclx_odr_table[id].odr_avl[i].uhz;

	return 0;
}

int st_iis2iclx_get_batch_val(struct st_iis2iclx_sensor *sensor,
			      int odr, int uodr, u8 *val)
{
	int required_odr = ST_IIS2ICLX_ODR_EXPAND(odr, uodr);
	enum st_iis2iclx_sensor_id id = sensor->id;
	int sensor_odr;
	int i;

	for (i = 0; i < st_iis2iclx_odr_table[id].size; i++) {
		sensor_odr = ST_IIS2ICLX_ODR_EXPAND(
				st_iis2iclx_odr_table[id].odr_avl[i].hz,
				st_iis2iclx_odr_table[id].odr_avl[i].uhz);
		if (sensor_odr >= required_odr)
			break;
	}

	if (i == st_iis2iclx_odr_table[id].size)
		return -EINVAL;

	*val = st_iis2iclx_odr_table[id].odr_avl[i].batch_val;

	return 0;
}

static u16 st_iis2iclx_check_odr_dependency(struct st_iis2iclx_hw *hw,
					    int odr, int uodr,
					    enum st_iis2iclx_sensor_id ref_id)
{
	struct st_iis2iclx_sensor *ref = iio_priv(hw->iio_devs[ref_id]);
	bool enable = ST_IIS2ICLX_ODR_EXPAND(odr, uodr) > 0;
	u16 ret;

	if (enable) {
		/* uodr not used */
		if (hw->enable_mask & BIT_ULL(ref_id))
			ret = max_t(u16, ref->odr, odr);
		else
			ret = odr;
	} else {
		ret = (hw->enable_mask & BIT_ULL(ref_id)) ? ref->odr : 0;
	}

	return ret;
}

static int st_iis2iclx_update_odr_fsm(struct st_iis2iclx_hw *hw,
				      enum st_iis2iclx_sensor_id id,
				      enum st_iis2iclx_sensor_id id_req,
				      int val, int delay)
{
	bool fsm_running = st_iis2iclx_fsm_running(hw);
	bool mlc_running = st_iis2iclx_mlc_running(hw);
	int ret = 0;
	int status;

	if (fsm_running || mlc_running || (id_req > ST_IIS2ICLX_ID_MLC)) {
		/*
		 * in STMC_PAGE:
		 * Addr 0x02 bit 1 set to 1 -- CLK Disable
		 * Addr 0x05 bit 0 set to 0 -- FSM_EN=0
		 * Addr 0x05 bit 4 set to 0 -- MLC_EN=0
		 * Addr 0x67 bit 0 set to 0 -- FSM_INIT=0
		 * Addr 0x67 bit 4 set to 0 -- MLC_INIT=0
		 * Addr 0x02 bit 1 set to 0 -- CLK Disable
		 * - ODR change
		 * - Wait (~3 ODRs)
		 * in STMC_PAGE:
		 * Addr 0x05 bit 0 set to 1 -- FSM_EN = 1
		 * Addr 0x05 bit 4 set to 1 -- MLC_EN = 1
		 */
		mutex_lock(&hw->page_lock);
		ret = st_iis2iclx_set_page_access(hw, true,
						  ST_IIS2ICLX_FUNC_CFG_MASK);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_read(hw->regmap,
				  ST_IIS2ICLX_REG_EMB_FUNC_EN_B_ADDR,
				  &status);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 ST_IIS2ICLX_REG_PAGE_SEL_ADDR,
					 BIT(1), FIELD_PREP(BIT(1), 1));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
				ST_IIS2ICLX_REG_EMB_FUNC_EN_B_ADDR,
				ST_IIS2ICLX_FSM_EN_MASK,
				FIELD_PREP(ST_IIS2ICLX_FSM_EN_MASK, 0));
		if (ret < 0)
			goto unlock_page;

		if (st_iis2iclx_mlc_running(hw)) {
			ret = regmap_update_bits(hw->regmap,
				ST_IIS2ICLX_REG_EMB_FUNC_EN_B_ADDR,
				ST_IIS2ICLX_MLC_EN_MASK,
				FIELD_PREP(ST_IIS2ICLX_MLC_EN_MASK, 0));
			if (ret < 0)
				goto unlock_page;
		}

		ret = regmap_update_bits(hw->regmap,
				ST_IIS2ICLX_REG_EMB_FUNC_INIT_B_ADDR,
				ST_IIS2ICLX_MLC_INIT_MASK,
				FIELD_PREP(ST_IIS2ICLX_MLC_INIT_MASK, 0));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
				ST_IIS2ICLX_REG_EMB_FUNC_INIT_B_ADDR,
				ST_IIS2ICLX_FSM_INIT_MASK,
				FIELD_PREP(ST_IIS2ICLX_FSM_INIT_MASK, 0));
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
					 ST_IIS2ICLX_REG_PAGE_SEL_ADDR,
					 BIT(1), FIELD_PREP(BIT(1), 0));
		if (ret < 0)
			goto unlock_page;

		ret = st_iis2iclx_set_page_access(hw, false,
						  ST_IIS2ICLX_FUNC_CFG_MASK);
		if (ret < 0)
			goto unlock_page;

		ret = regmap_update_bits(hw->regmap,
				st_iis2iclx_odr_table[id].reg.addr,
				st_iis2iclx_odr_table[id].reg.mask,
				ST_IIS2ICLX_SHIFT_VAL(val,
					st_iis2iclx_odr_table[id].reg.mask));
		if (ret < 0)
			goto unlock_page;

		usleep_range(delay, delay + (delay / 10));

		st_iis2iclx_set_page_access(hw, true,
					    ST_IIS2ICLX_FUNC_CFG_MASK);

		ret = regmap_write(hw->regmap,
				   ST_IIS2ICLX_REG_EMB_FUNC_EN_B_ADDR,
				   status);

unlock_page:
		st_iis2iclx_set_page_access(hw, false,
					    ST_IIS2ICLX_FUNC_CFG_MASK);
		mutex_unlock(&hw->page_lock);
	} else {
		ret = st_iis2iclx_update_bits_locked(hw,
				st_iis2iclx_odr_table[id].reg.addr,
				st_iis2iclx_odr_table[id].reg.mask,
				val);
	}

	return ret;
}

static int st_iis2iclx_update_decimator(struct st_iis2iclx_sensor *sensor,
					int odr)
{
	struct st_iis2iclx_hw *hw = sensor->hw;
	enum st_iis2iclx_sensor_id id = sensor->id;
	int oix, odix, ret = 0;

	if (hw->enable_drdy_mask) {
		sensor->decimator = 0;

		return 0;
	}

	oix = st_iis2iclx_get_odr_index(odr);
	if (oix < 0)
		return oix;

	mutex_lock(&hw->fifo_lock);

	if (id != ST_IIS2ICLX_ID_ACC) {
		ret = -EINVAL;

		goto unlock;
	}

	odix = st_iis2iclx_get_odr_divider_index(hw->xl_odr_div);
	if (odix < 0) {
		ret = odix;

		goto unlock;
	}

	sensor->discard_samples =
		st_iis2iclx_lpf_discard_table[id].samples_to_discard[oix] +
		st_iis2iclx_lpf_discard_table[id].settling_samples[odix][oix];

unlock:
	mutex_unlock(&hw->fifo_lock);

	return ret;
}

static int st_iis2iclx_set_odr(struct st_iis2iclx_sensor *sensor,
			       int req_odr, int req_uodr)
{
	enum st_iis2iclx_sensor_id id_req = sensor->id;
	enum st_iis2iclx_sensor_id id = sensor->id;
	struct st_iis2iclx_hw *hw = sensor->hw;
	int err, delay;
	u8 val = 0;

	switch (id) {
	case ST_IIS2ICLX_ID_EXT0:
	case ST_IIS2ICLX_ID_EXT1:
	case ST_IIS2ICLX_ID_MLC_0:
	case ST_IIS2ICLX_ID_MLC_1:
	case ST_IIS2ICLX_ID_MLC_2:
	case ST_IIS2ICLX_ID_MLC_3:
	case ST_IIS2ICLX_ID_MLC_4:
	case ST_IIS2ICLX_ID_MLC_5:
	case ST_IIS2ICLX_ID_MLC_6:
	case ST_IIS2ICLX_ID_MLC_7:
	case ST_IIS2ICLX_ID_FSM_0:
	case ST_IIS2ICLX_ID_FSM_1:
	case ST_IIS2ICLX_ID_FSM_2:
	case ST_IIS2ICLX_ID_FSM_3:
	case ST_IIS2ICLX_ID_FSM_4:
	case ST_IIS2ICLX_ID_FSM_5:
	case ST_IIS2ICLX_ID_FSM_6:
	case ST_IIS2ICLX_ID_FSM_7:
	case ST_IIS2ICLX_ID_FSM_8:
	case ST_IIS2ICLX_ID_FSM_9:
	case ST_IIS2ICLX_ID_FSM_10:
	case ST_IIS2ICLX_ID_FSM_11:
	case ST_IIS2ICLX_ID_FSM_12:
	case ST_IIS2ICLX_ID_FSM_13:
	case ST_IIS2ICLX_ID_FSM_14:
	case ST_IIS2ICLX_ID_FSM_15:
	case ST_IIS2ICLX_ID_WK:
	case ST_IIS2ICLX_ID_SC:
	case ST_IIS2ICLX_ID_6D:
	case ST_IIS2ICLX_ID_TEMP:
	case ST_IIS2ICLX_ID_ACC: {
		int odr;
		int i;

		id = ST_IIS2ICLX_ID_ACC;
		for (i = ST_IIS2ICLX_ID_ACC; i < ST_IIS2ICLX_ID_MAX; i++) {
			if (!hw->iio_devs[i])
				continue;

			if (i == sensor->id)
				continue;

			odr = st_iis2iclx_check_odr_dependency(hw, req_odr,
							       req_uodr, i);
			if (odr != req_odr) {
				/* device already configured */
				return 0;
			}
		}
		break;
	}
	default:
		return 0;
	}

	err = st_iis2iclx_get_odr_val(id, req_odr, req_uodr, NULL, NULL, &val);
	if (err < 0)
		return err;

	err = st_iis2iclx_update_decimator(iio_priv(hw->iio_devs[id]), req_odr);
	if (err < 0)
		return err;

	delay = req_odr > 0 ? 4000000 / req_odr : 0;

	return st_iis2iclx_update_odr_fsm(hw, id, id_req, val, delay);
}

int st_iis2iclx_sensor_set_enable(struct st_iis2iclx_sensor *sensor,
				  bool enable)
{
	int uodr = enable ? sensor->uodr : 0;
	int odr = enable ? sensor->odr : 0;
	int err;

	err = st_iis2iclx_set_odr(sensor, odr, uodr);
	if (err < 0)
		return err;

	if (enable)
		sensor->hw->enable_mask |= BIT_ULL(sensor->id);
	else
		sensor->hw->enable_mask &= ~BIT_ULL(sensor->id);

	return 0;
}

static int st_iis2iclx_read_oneshot(struct st_iis2iclx_sensor *sensor,
				    u8 addr, int *val)
{
	struct st_iis2iclx_hw *hw = sensor->hw;
	int err, delay;
	__le16 data;

	err = st_iis2iclx_sensor_set_enable(sensor, true);
	if (err < 0)
		return err;

	/* Use big delay for data valid because of drdy mask enabled */
	delay = 10000000 / sensor->odr;
	usleep_range(delay, 2 * delay);

	err = st_iis2iclx_read_locked(hw, addr, &data, sizeof(data));
	if (err < 0)
		return err;

	err = st_iis2iclx_sensor_set_enable(sensor, false);

	*val = (s16)le16_to_cpu(data);

	return IIO_VAL_INT;
}

static int st_iis2iclx_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *ch,
				int *val, int *val2, long mask)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(iio_dev);
		if (ret)
			break;

		ret = st_iis2iclx_read_oneshot(sensor, ch->address, val);
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
			*val2 = ST_IIS2ICLX_TEMP_GAIN;
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

static int st_iis2iclx_write_raw(struct iio_dev *iio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct st_iis2iclx_sensor *s = iio_priv(iio_dev);
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = iio_device_claim_direct_mode(iio_dev);
		if (err)
			return err;

		err = st_iis2iclx_set_full_scale(s, val2);
		iio_device_release_direct_mode(iio_dev);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		int todr, tuodr;
		u8 data;

		err = st_iis2iclx_get_odr_val(s->id, val, val2,
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
				case ST_IIS2ICLX_ID_ACC:
					err = st_iis2iclx_set_odr(s, s->odr,
								  s->uodr);
					if (err < 0)
						break;

					st_iis2iclx_update_batching(iio_dev, 1);
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
st_iis2iclx_sysfs_sampling_freq_avail(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	enum st_iis2iclx_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_iis2iclx_odr_table[id].size; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%06d ",
				 st_iis2iclx_odr_table[id].odr_avl[i].hz,
				 st_iis2iclx_odr_table[id].odr_avl[i].uhz);
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_iis2iclx_sysfs_scale_avail(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	enum st_iis2iclx_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_iis2iclx_fs_table[id].size; i++) {
		if (sensor->id != ST_IIS2ICLX_ID_TEMP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "0.%09u ",
					 st_iis2iclx_fs_table[id].fs_avl[i].gain);
		} else {
			int hi, low;

			hi = (int)(st_iis2iclx_fs_table[id].fs_avl[i].gain / 1000);
			low = (int)(st_iis2iclx_fs_table[id].fs_avl[i].gain % 1000);
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d.%d ",
					 hi, low);
		}
	}

	buf[len - 1] = '\n';

	return len;
}

static int st_iis2iclx_selftest_sensor(struct st_iis2iclx_sensor *sensor,
				       int test)
{
	int x_selftest = 0, y_selftest = 0, z_selftest = 0;
	int x = 0, y = 0, z = 0, try_count = 0;
	int ret, delay, data_delay = 100000;
	u8 i, status, n = 0;
	u8 reg, bitmask;
	u8 raw_data[6];

	if (sensor->id != ST_IIS2ICLX_ID_ACC)
		return -EINVAL;

	reg = ST_IIS2ICLX_REG_OUTX_L_A_ADDR;
	bitmask = ST_IIS2ICLX_STATUS_XLDA;
	data_delay = 50000;

	/* reset selftest_status */
	sensor->selftest_status = -1;

	/* set selftest normal mode */
	ret = st_iis2iclx_set_selftest(sensor, 0);
	if (ret < 0)
		return ret;

	ret = st_iis2iclx_sensor_set_enable(sensor, true);
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
		usleep_range(delay, delay + delay / 10);
		ret = st_iis2iclx_read_locked(sensor->hw,
					      ST_IIS2ICLX_REG_STATUS_ADDR,
					      &status, sizeof(status));
		if (ret < 0)
			goto selftest_failure;

		if (status & bitmask) {
			st_iis2iclx_read_locked(sensor->hw, reg,
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
			usleep_range(delay, delay + delay / 10);
			ret = st_iis2iclx_read_locked(sensor->hw,
						    ST_IIS2ICLX_REG_STATUS_ADDR,
						    &status, sizeof(status));
			if (ret < 0)
				goto selftest_failure;

			if (status & bitmask) {
				ret = st_iis2iclx_read_locked(sensor->hw,
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
	st_iis2iclx_set_selftest(sensor, test);

	/* wait for stable output */
	usleep_range(data_delay, data_delay + data_delay / 100);

	try_count = 0;

	/* after enabled the sensor trash first sample */
	while (try_count < 3) {
		usleep_range(delay, delay + delay / 10);
		ret = st_iis2iclx_read_locked(sensor->hw,
					      ST_IIS2ICLX_REG_STATUS_ADDR,
					      &status, sizeof(status));
		if (ret < 0)
			goto selftest_failure;

		if (status & bitmask) {
			st_iis2iclx_read_locked(sensor->hw, reg,
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
			usleep_range(delay, delay + delay / 10);
			ret = st_iis2iclx_read_locked(sensor->hw,
						    ST_IIS2ICLX_REG_STATUS_ADDR,
						    &status, sizeof(status));
			if (ret < 0)
				goto selftest_failure;

			if (status & bitmask) {
				ret = st_iis2iclx_read_locked(sensor->hw,
							      reg, raw_data,
							      sizeof(raw_data));
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
			"some samples missing (expected %d, read %d)\n", i, n);
		ret = -1;

		goto selftest_failure;
	}

	if ((abs(x_selftest - x) < ST_IIS2ICLX_SELFTEST_MIN) ||
	    (abs(x_selftest - x) > ST_IIS2ICLX_SELFTEST_MAX)) {
		sensor->selftest_status = -1;
		dev_info(sensor->hw->dev,
			 "st: failure on x: non-st(%d), st(%d)\n",
			 x, x_selftest);

		goto selftest_failure;
	}

	if ((abs(y_selftest - y) < ST_IIS2ICLX_SELFTEST_MIN) ||
	    (abs(y_selftest - y) > ST_IIS2ICLX_SELFTEST_MAX)) {
		sensor->selftest_status = -1;
		dev_info(sensor->hw->dev,
			 "st: failure on y: non-st(%d), st(%d)\n",
			 y, y_selftest);

		goto selftest_failure;
	}

	if ((abs(z_selftest - z) < ST_IIS2ICLX_SELFTEST_MIN) ||
	    (abs(z_selftest - z) > ST_IIS2ICLX_SELFTEST_MAX)) {
		sensor->selftest_status = -1;
		dev_info(sensor->hw->dev,
			 "st: failure on z: non-st(%d), st(%d)\n",
			 z, z_selftest);

		goto selftest_failure;
	}

	sensor->selftest_status = 1;

selftest_failure:
	/* restore selftest to normal mode */
	st_iis2iclx_set_selftest(sensor, 0);

	return st_iis2iclx_sensor_set_enable(sensor, false);
}

static ssize_t st_iis2iclx_sysfs_start_selftest(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	enum st_iis2iclx_sensor_id id = sensor->id;
	struct st_iis2iclx_hw *hw = sensor->hw;
	int ret, test;
	u8 drdy_reg;
	u32 gain;

	if (id != ST_IIS2ICLX_ID_ACC)
		return -EINVAL;

	for (test = 0; test < ARRAY_SIZE(st_iis2iclx_selftest_table); test++) {
		if (strncmp(buf, st_iis2iclx_selftest_table[test].string_mode,
			strlen(st_iis2iclx_selftest_table[test].string_mode)) == 0)
			break;
	}

	if (test == ARRAY_SIZE(st_iis2iclx_selftest_table))
		return -EINVAL;

	ret = iio_device_claim_direct_mode(iio_dev);
	if (ret)
		return ret;

	/* self test mode unavailable if sensor enabled */
	if (hw->enable_mask & BIT_ULL(id)) {
		ret = -EBUSY;

		goto out_claim;
	}

	st_iis2iclx_bk_regs(hw);

	/* disable FIFO watermak interrupt */
	ret = st_iis2iclx_get_int_reg(hw, &drdy_reg);
	if (ret < 0)
		goto restore_regs;

	ret = st_iis2iclx_update_bits_locked(hw, drdy_reg,
					     ST_IIS2ICLX_INT_FIFO_TH_MASK, 0);
	if (ret < 0)
		goto restore_regs;

	gain = sensor->gain;

	/* set BDU = 1, FS = 2 g, ODR = 52 Hz */
	st_iis2iclx_set_full_scale(sensor, ST_IIS2ICLX_ACC_FS_2G_GAIN);
	st_iis2iclx_set_odr(sensor, 52, 0);
	st_iis2iclx_selftest_sensor(sensor, test);

	/* restore full scale after test */
	st_iis2iclx_set_full_scale(sensor, gain);

restore_regs:
	st_iis2iclx_restore_regs(hw);

out_claim:
	iio_device_release_direct_mode(iio_dev);

	return size;
}

ssize_t st_iis2iclx_get_module_id(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	struct st_iis2iclx_hw *hw = sensor->hw;

	return scnprintf(buf, PAGE_SIZE, "%u\n", hw->module_id);
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_iis2iclx_sysfs_sampling_freq_avail);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_iis2iclx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_anglvel_scale_available, 0444,
		       st_iis2iclx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_iis2iclx_get_max_watermark, NULL, 0);
static IIO_DEVICE_ATTR(in_temp_scale_available, 0444,
		       st_iis2iclx_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, 0200, NULL, st_iis2iclx_flush_fifo, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644, st_iis2iclx_get_watermark,
		       st_iis2iclx_set_watermark, 0);

static IIO_DEVICE_ATTR(selftest_available, 0444,
		       st_iis2iclx_sysfs_get_selftest_available,
		       NULL, 0);
static IIO_DEVICE_ATTR(selftest, 0644,
		       st_iis2iclx_sysfs_get_selftest_status,
		       st_iis2iclx_sysfs_start_selftest, 0);
static IIO_DEVICE_ATTR(module_id, 0444, st_iis2iclx_get_module_id, NULL, 0);

static ssize_t
__maybe_unused st_iis2iclx_get_discharded_samples(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	int ret;

	ret = sprintf(buf, "%d\n", sensor->discharged_samples);

	/* reset counter */
	sensor->discharged_samples = 0;

	return ret;
}

static int st_iis2iclx_write_raw_get_fmt(struct iio_dev *indio_dev,
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

static IIO_DEVICE_ATTR(discharded_samples, 0444,
		       st_iis2iclx_get_discharded_samples, NULL, 0);

static struct attribute *st_iis2iclx_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,

#ifdef ST_IIS2ICLX_DEBUG_DISCHARGE
	&iio_dev_attr_discharded_samples.dev_attr.attr,
#endif /* ST_IIS2ICLX_DEBUG_DISCHARGE */

	NULL,
};

static const struct attribute_group st_iis2iclx_acc_attribute_group = {
	.attrs = st_iis2iclx_acc_attributes,
};

static const struct iio_info st_iis2iclx_acc_info = {
	.attrs = &st_iis2iclx_acc_attribute_group,
	.read_raw = st_iis2iclx_read_raw,
	.write_raw = st_iis2iclx_write_raw,
	.write_raw_get_fmt = st_iis2iclx_write_raw_get_fmt,

#ifdef CONFIG_DEBUG_FS
	.debugfs_reg_access = &st_iis2iclx_reg_access,
#endif /* CONFIG_DEBUG_FS */
};

static struct attribute *st_iis2iclx_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_selftest_available.dev_attr.attr,
	&iio_dev_attr_selftest.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,

#ifdef ST_IIS2ICLX_DEBUG_DISCHARGE
	&iio_dev_attr_discharded_samples.dev_attr.attr,
#endif /* ST_IIS2ICLX_DEBUG_DISCHARGE */

	NULL,
};

static const struct attribute_group st_iis2iclx_gyro_attribute_group = {
	.attrs = st_iis2iclx_gyro_attributes,
};

static const struct iio_info st_iis2iclx_gyro_info = {
	.attrs = &st_iis2iclx_gyro_attribute_group,
	.read_raw = st_iis2iclx_read_raw,
	.write_raw = st_iis2iclx_write_raw,
	.write_raw_get_fmt = st_iis2iclx_write_raw_get_fmt,
};

static struct attribute *st_iis2iclx_temp_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_temp_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	&iio_dev_attr_module_id.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_iis2iclx_temp_attribute_group = {
	.attrs = st_iis2iclx_temp_attributes,
};

static const struct iio_info st_iis2iclx_temp_info = {
	.attrs = &st_iis2iclx_temp_attribute_group,
	.read_raw = st_iis2iclx_read_raw,
	.write_raw = st_iis2iclx_write_raw,
	.write_raw_get_fmt = st_iis2iclx_write_raw_get_fmt,
};

static const unsigned long st_iis2iclx_available_scan_masks[] = {
	GENMASK(2, 0), 0x0
};

static const unsigned long st_iis2iclx_temp_available_scan_masks[] = {
	GENMASK(1, 0), 0x0
};

static int st_iis2iclx_init_xl_filters(struct st_iis2iclx_hw *hw)
{
	int div;
	int err;
	int i;

	err = device_property_read_u32(hw->dev, "st,xl_lpf_div", &div);
	if (err < 0) {
		/* if st,xl_lpf_div not available disable XL LPF2 */
		err = regmap_update_bits(hw->regmap,
					 ST_IIS2ICLX_REG_CTRL1_XL_ADDR,
					 ST_IIS2ICLX_LPF2_XL_EN_MASK,
					 FIELD_PREP(ST_IIS2ICLX_LPF2_XL_EN_MASK, 0));
		return err < 0 ? err : 0;
	}

	for (i = 0; i < st_iis2iclx_xl_bw.size; i++) {
		if (st_iis2iclx_xl_bw.st_iis2iclx_xl_lpf_bw[i].div >= div)
			break;
	}

	if (i == st_iis2iclx_xl_bw.size)
		return -EINVAL;

	/* set XL LPF2 BW */
	err = regmap_update_bits(hw->regmap,
				 st_iis2iclx_xl_bw.reg,
				 st_iis2iclx_xl_bw.mask,
				 ST_IIS2ICLX_SHIFT_VAL(st_iis2iclx_xl_bw.st_iis2iclx_xl_lpf_bw[i].val,
						       st_iis2iclx_xl_bw.mask));
	if (err < 0)
		return err;

	/* enable/disable LPF2 filter selection */
	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_CTRL1_XL_ADDR,
				 ST_IIS2ICLX_LPF2_XL_EN_MASK,
				 FIELD_PREP(ST_IIS2ICLX_LPF2_XL_EN_MASK,
					st_iis2iclx_xl_bw.st_iis2iclx_xl_lpf_bw[i].lpf2_xl_en));

	if (err < 0)
		return err;

	hw->enable_drdy_mask = false;
	hw->xl_odr_div = div;

	return 0;
}

static int st_iis2iclx_reset_device(struct st_iis2iclx_hw *hw)
{
	int err;

	/* set configuration bit */
	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_CTRL9_XL_ADDR,
				 ST_IIS2ICLX_DEVICE_CONF_MASK,
				 FIELD_PREP(ST_IIS2ICLX_DEVICE_CONF_MASK, 1));
	if (err < 0)
		return err;

	/* sw reset */
	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_CTRL3_C_ADDR,
				 ST_IIS2ICLX_SW_RESET_MASK,
				 FIELD_PREP(ST_IIS2ICLX_SW_RESET_MASK, 1));
	if (err < 0)
		return err;

	msleep(50);

	/* boot */
	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_CTRL3_C_ADDR,
				 ST_IIS2ICLX_BOOT_MASK,
				 FIELD_PREP(ST_IIS2ICLX_BOOT_MASK, 1));

	msleep(50);

	return err;
}

static int st_iis2iclx_init_device(struct st_iis2iclx_hw *hw)
{
	u8 drdy_reg;
	int err;

	/* latch interrupts */
	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_TAP_CFG0_ADDR,
				 ST_IIS2ICLX_LIR_MASK,
				 FIELD_PREP(ST_IIS2ICLX_LIR_MASK, 1));
	if (err < 0)
		return err;

	/* enable Block Data Update */
	err = regmap_update_bits(hw->regmap, ST_IIS2ICLX_REG_CTRL3_C_ADDR,
				 ST_IIS2ICLX_BDU_MASK,
				 FIELD_PREP(ST_IIS2ICLX_BDU_MASK, 1));
	if (err < 0)
		return err;

	/* init timestamp engine */
	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_CTRL10_C_ADDR,
				 ST_IIS2ICLX_TIMESTAMP_EN_MASK,
				 ST_IIS2ICLX_SHIFT_VAL(true,
					ST_IIS2ICLX_TIMESTAMP_EN_MASK));
	if (err < 0)
		return err;

	err = st_iis2iclx_get_int_reg(hw, &drdy_reg);
	if (err < 0)
		return err;

	/* initialize sensors filter bandwidth configuration */
	hw->enable_drdy_mask = true;
	err = st_iis2iclx_init_xl_filters(hw);
	if (err < 0)
		return err;

	/* Enable DRDY MASK for filters settling time */
	err = regmap_update_bits(hw->regmap, ST_IIS2ICLX_REG_CTRL4_C_ADDR,
				 ST_IIS2ICLX_DRDY_MASK,
				 FIELD_PREP(ST_IIS2ICLX_DRDY_MASK,
					    hw->enable_drdy_mask ? 1 : 0));

	if (err < 0)
		return err;

	/* enable FIFO watermak interrupt */
	return regmap_update_bits(hw->regmap, drdy_reg,
				  ST_IIS2ICLX_INT_FIFO_TH_MASK,
				  FIELD_PREP(ST_IIS2ICLX_INT_FIFO_TH_MASK, 1));
}

#ifdef CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES
static int st_iis2iclx_post_init_device(struct st_iis2iclx_hw *hw)
{
	int err;

	/* Set default wake-up thershold to 93750 ug */
	err = st_iis2iclx_set_wake_up_thershold(hw, 93750);
	if (err < 0)
		return err;

	/* Set default wake-up duration to 0 */
	err = st_iis2iclx_set_wake_up_duration(hw, 0);
	if (err < 0)
		return err;

	/* setting default 6D threshold to 60 degrees */
	return st_iis2iclx_set_6D_threshold(hw, 60);
}
#endif /* CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES */

static struct iio_dev *st_iis2iclx_alloc_iiodev(struct st_iis2iclx_hw *hw,
						enum st_iis2iclx_sensor_id id)
{
	struct st_iis2iclx_sensor *sensor;
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

#ifdef ST_IIS2ICLX_DEBUG_DISCHARGE
	sensor->discharged_samples = 0;
#endif /* ST_IIS2ICLX_DEBUG_DISCHARGE */

	switch (id) {
	case ST_IIS2ICLX_ID_ACC:
		iio_dev->channels = st_iis2iclx_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_iis2iclx_acc_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_accel", ST_IIS2ICLX_DEV_NAME);
		iio_dev->info = &st_iis2iclx_acc_info;
		iio_dev->available_scan_masks =
					st_iis2iclx_available_scan_masks;
		sensor->max_watermark = ST_IIS2ICLX_MAX_FIFO_DEPTH;
		sensor->gain = st_iis2iclx_fs_table[id].fs_avl[ST_IIS2ICLX_DEFAULT_XL_FS_INDEX].gain;
		sensor->odr = st_iis2iclx_odr_table[id].odr_avl[ST_IIS2ICLX_DEFAULT_XL_ODR_INDEX].hz;
		sensor->uodr = st_iis2iclx_odr_table[id].odr_avl[ST_IIS2ICLX_DEFAULT_XL_ODR_INDEX].uhz;
		sensor->offset = 0;
		break;
	case ST_IIS2ICLX_ID_TEMP:
		iio_dev->channels = st_iis2iclx_temp_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_iis2iclx_temp_channels);
		scnprintf(sensor->name, sizeof(sensor->name),
			  "%s_temp", ST_IIS2ICLX_DEV_NAME);
		iio_dev->info = &st_iis2iclx_temp_info;
		iio_dev->available_scan_masks =
					  st_iis2iclx_temp_available_scan_masks;
		sensor->max_watermark = ST_IIS2ICLX_MAX_FIFO_DEPTH;
		sensor->gain = st_iis2iclx_fs_table[id].fs_avl[ST_IIS2ICLX_DEFAULT_T_FS_INDEX].gain;
		sensor->odr = st_iis2iclx_odr_table[id].odr_avl[ST_IIS2ICLX_DEFAULT_T_ODR_INDEX].hz;
		sensor->uodr = st_iis2iclx_odr_table[id].odr_avl[ST_IIS2ICLX_DEFAULT_T_ODR_INDEX].uhz;
		sensor->offset = ST_IIS2ICLX_TEMP_OFFSET;
		break;
	default:
		dev_err(hw->dev, "invalid sensor id %d\n", id);
		return NULL;
	}

	st_iis2iclx_set_full_scale(sensor, sensor->gain);
	iio_dev->name = sensor->name;

	return iio_dev;
}

static void st_iis2iclx_get_properties(struct st_iis2iclx_hw *hw)
{
	if (device_property_read_u32(hw->dev, "st,module_id", &hw->module_id))
		hw->module_id = 1;
}

static void st_iis2iclx_disable_regulator_action(void *_data)
{
	struct st_iis2iclx_hw *hw = _data;

	regulator_disable(hw->vddio_supply);
	regulator_disable(hw->vdd_supply);
}

static int st_iis2iclx_power_enable(struct st_iis2iclx_hw *hw)
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
				       st_iis2iclx_disable_regulator_action,
				       hw);
	if (err) {
		dev_err(hw->dev,
			"Failed to setup regulator cleanup action %d\n", err);

		return err;
	}

	return 0;
}

int st_iis2iclx_probe(struct device *dev, int irq, struct regmap *regmap)
{
	struct st_iis2iclx_hw *hw;
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
	hw->odr_table_entry = st_iis2iclx_odr_table;
	hw->hw_timestamp_global = 0;

	err = st_iis2iclx_power_enable(hw);
	if (err != 0)
		return err;

	/* set page zero before access to registers */
	err = st_iis2iclx_set_page_0(hw);
	if (err < 0)
		return err;

	err = st_iis2iclx_check_whoami(hw);
	if (err < 0)
		return err;

	st_iis2iclx_get_properties(hw);

	err = st_iis2iclx_get_odr_calibration(hw);
	if (err < 0)
		return err;

	err = st_iis2iclx_reset_device(hw);
	if (err < 0)
		return err;

	err = st_iis2iclx_init_device(hw);
	if (err < 0)
		return err;

#if KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE
	err = iio_read_mount_matrix(hw->dev, &hw->orientation);
#elif KERNEL_VERSION(5, 2, 0) <= LINUX_VERSION_CODE
	err = iio_read_mount_matrix(hw->dev, "mount-matrix", &hw->orientation);
#else /* LINUX_VERSION_CODE */
	err = of_iio_read_mount_matrix(hw->dev, "mount-matrix",
				       &hw->orientation);
#endif /* LINUX_VERSION_CODE */

	if (err) {
		dev_err(dev, "Failed to retrieve mounting matrix %d\n", err);

		return err;
	}

	for (i = ST_IIS2ICLX_ID_ACC; i <= ST_IIS2ICLX_ID_TEMP; i++) {
		hw->iio_devs[i] = st_iis2iclx_alloc_iiodev(hw, i);
		if (!hw->iio_devs[i])
			return -ENOMEM;
	}

	err = st_iis2iclx_shub_probe(hw);
	if (err < 0)
		return err;

	if (hw->irq > 0) {
		err = st_iis2iclx_buffers_setup(hw);
		if (err < 0)
			return err;
	}

	err = st_iis2iclx_mlc_probe(hw);
	if (err < 0)
		return err;

	for (i = ST_IIS2ICLX_ID_ACC; i < ST_IIS2ICLX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

	err = st_iis2iclx_mlc_init_preload(hw);
	if (err)
		goto remove_mlc;

#ifdef CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES
	err = st_iis2iclx_probe_event(hw);
	if (err < 0)
		goto remove_mlc;

	err = st_iis2iclx_post_init_device(hw);
	if (err < 0)
		goto remove_mlc;
#endif /* CONFIG_IIO_ST_IIS2ICLX_EN_BASIC_FEATURES */

	device_init_wakeup(dev,
			   device_property_read_bool(dev, "wakeup-source"));

	dev_info(dev, "Device probed\n");

	return 0;

remove_mlc:
	/*
	 * mlc/fsm iio devices are dinamically created during
	 * st_iis2iclx_mlc_init_preload so this devices must be flushed after
	 * this procedure in case of failure
	 */
	st_iis2iclx_mlc_remove(hw->dev);

	return err;
}
EXPORT_SYMBOL(st_iis2iclx_probe);

void st_iis2iclx_remove(struct device *dev)
{
	st_iis2iclx_mlc_remove(dev);
}
EXPORT_SYMBOL(st_iis2iclx_remove);

static int __maybe_unused st_iis2iclx_suspend(struct device *dev)
{
	struct st_iis2iclx_hw *hw = dev_get_drvdata(dev);
	struct st_iis2iclx_sensor *sensor;
	int i, err = 0;

	dev_info(dev, "Suspending device\n");

	disable_hardirq(hw->irq);

	for (i = 0; i < ST_IIS2ICLX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT_ULL(sensor->id)))
			continue;

		/* power off enabled sensors */
		err = st_iis2iclx_set_odr(sensor, 0, 0);
		if (err < 0)
			return err;
	}

	if (st_iis2iclx_is_fifo_enabled(hw)) {
		err = st_iis2iclx_suspend_fifo(hw);
		if (err < 0)
			return err;
	}

	err = st_iis2iclx_bk_regs(hw);

	if (device_may_wakeup(dev))
		enable_irq_wake(hw->irq);

	return err < 0 ? err : 0;
}

static int __maybe_unused st_iis2iclx_resume(struct device *dev)
{
	struct st_iis2iclx_hw *hw = dev_get_drvdata(dev);
	struct st_iis2iclx_sensor *sensor;
	int i, err = 0;

	dev_info(dev, "Resuming device\n");

	if (device_may_wakeup(dev))
		disable_irq_wake(hw->irq);

	err = st_iis2iclx_restore_regs(hw);
	if (err < 0)
		return err;

	for (i = 0; i < ST_IIS2ICLX_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		sensor = iio_priv(hw->iio_devs[i]);
		if (!(hw->enable_mask & BIT_ULL(sensor->id)))
			continue;

		err = st_iis2iclx_set_odr(sensor, sensor->odr, sensor->uodr);
		if (err < 0)
			return err;
	}

	err = st_iis2iclx_reset_hwts(hw);
	if (err < 0)
		return err;

	if (st_iis2iclx_is_fifo_enabled(hw))
		err = st_iis2iclx_set_fifo_mode(hw, ST_IIS2ICLX_FIFO_CONT);

	enable_irq(hw->irq);

	return err < 0 ? err : 0;
}

const struct dev_pm_ops st_iis2iclx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_iis2iclx_suspend, st_iis2iclx_resume)
};
EXPORT_SYMBOL(st_iis2iclx_pm_ops);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_iis2iclx driver");
MODULE_LICENSE("GPL v2");
