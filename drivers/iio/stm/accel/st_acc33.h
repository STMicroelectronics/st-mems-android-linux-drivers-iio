/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics st_acc33 sensor driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#ifndef ST_ACC33_H
#define ST_ACC33_H

#include <linux/iio/iio.h>
#include <linux/version.h>

#include "../common/stm_iio_types.h"

#define LIS2DH_DEV_NAME			"lis2dh_accel"
#define LIS2DH12_DEV_NAME		"lis2dh12_accel"
#define LIS3DH_DEV_NAME			"lis3dh_accel"
#define LSM303AGR_DEV_NAME		"lsm303agr_accel"
#define IIS2DH_DEV_NAME			"iis2dh_accel"

#define REG_WHOAMI_ADDR			0x0f
#define REG_WHOAMI_VAL			0x33

#define REG_CTRL1_ADDR			0x20
#define REG_CTRL1_ODR_MASK		GENMASK(7, 4)

#define REG_CTRL2_ADDR			0x21
#define REG_CTRL2_FDS_MASK		BIT(3)
#define REG_CTRL2_HPIS1_MASK		BIT(0)

#define REG_CTRL3_ADDR			0x22
#define REG_CTRL3_I1_OVR_MASK		BIT(1)
#define REG_CTRL3_I1_WTM_MASK		BIT(2)
#define REG_CTRL3_I1_DRDY1_MASK		BIT(4)
#define REG_CTRL3_I1_AOI1_MASK		BIT(6)

#define REG_CTRL4_ADDR			0x23
#define REG_CTRL4_BDU_MASK		BIT(7)
#define REG_CTRL4_FS_MASK		GENMASK(5, 4)

#define REG_CTRL5_ACC_ADDR		0x24
#define REG_CTRL5_ACC_LIR_INT1_MASK	BIT(3)
#define REG_CTRL5_ACC_FIFO_EN_MASK	BIT(6)
#define REG_CTRL5_ACC_BOOT_MASK		BIT(7)

#define REG_CTRL6_ACC_ADDR		0x25

#define REG_OUTX_L_ADDR			0x28
#define REG_OUTY_L_ADDR			0x2a
#define REG_OUTZ_L_ADDR			0x2c

#define REG_FIFO_CTRL_REG		0x2e
#define REG_FIFO_CTRL_REG_WTM_MASK	GENMASK(4, 0)
#define REG_FIFO_CTRL_MODE_MASK		GENMASK(7, 6)

#define REG_FIFO_SRC_ADDR		0x2f
#define REG_FIFO_SRC_WTM_MASK		BIT(7)
#define REG_FIFO_SRC_OVR_MASK		BIT(6)
#define REG_FIFO_SRC_FSS_MASK		GENMASK(4, 0)

#define REG_INT1_CFG_ADDR		0x30
#define REG_INT1_CFG_AOI_MASK		BIT(7)
#define REG_INT1_CFG_ZHIE_MASK		BIT(5)
#define REG_INT1_CFG_ZLIE_MASK		BIT(4)
#define REG_INT1_CFG_YHIE_MASK		BIT(3)
#define REG_INT1_CFG_YLIE_MASK		BIT(2)
#define REG_INT1_CFG_XHIE_MASK		BIT(1)
#define REG_INT1_CFG_XLIE_MASK		BIT(0)

#define ST_ACC33_FS_2G			IIO_G_TO_M_S_2(980)
#define ST_ACC33_FS_4G			IIO_G_TO_M_S_2(1950)
#define ST_ACC33_FS_8G			IIO_G_TO_M_S_2(3900)
#define ST_ACC33_FS_16G			IIO_G_TO_M_S_2(11720)

#define ST_ACC33_DATA_SIZE		6

#define ST_ACC33_RX_MAX_LENGTH		96
#define ST_ACC33_TX_MAX_LENGTH		8

enum st_acc33_event_id {
	ST_ACC33_EVENT_FF,
	ST_ACC33_EVENT_WAKEUP,
	ST_ACC33_EVENT_6D,

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
	ST_ACC33_EVENT_TAP,
	ST_ACC33_EVENT_DTAP,
#endif /* LINUX_VERSION_CODE */

	ST_ACC33_EVENT_MAX
};

struct st_acc33_transfer_buffer {
	u8 rx_buf[ST_ACC33_RX_MAX_LENGTH];
	u8 tx_buf[ST_ACC33_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct st_acc33_transfer_function {
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
};

enum st_acc33_fifo_mode {
	ST_ACC33_FIFO_BYPASS = 0x0,
	ST_ACC33_FIFO_STREAM = 0x2,
};

struct st_acc33_hw {
	struct device *dev;
	const char *name;
	int irq;

	u32 module_id;

	struct mutex fifo_lock;
	struct mutex lock;

	u8 watermark;
	u32 gain;
	u8 fs;
	u16 odr;
	u8 enable_ev_mask;
	bool enable;

	u64 samples;
	u8 std_level;

	s64 delta_ts;
	s64 ts_irq;
	s64 ts;

	int xl_th_mg;
	int duration_ms;

	struct iio_dev *iio_dev;

	const struct st_acc33_transfer_function *tf;
	struct st_acc33_transfer_buffer tb;
};

int st_acc33_write_with_mask(struct st_acc33_hw *hw, u8 addr, u8 mask,
			     u8 val);
int st_acc33_update_odr(struct st_acc33_hw *hw, u16 odr,
			bool is_event, bool enable);
int st_acc33_set_enable(struct st_acc33_hw *hw, bool enable);
int st_acc33_probe(struct device *device, int irq, const char *name,
		   const struct st_acc33_transfer_function *tf_ops);
int st_acc33_fifo_setup(struct st_acc33_hw *hw);
ssize_t st_acc33_flush_hwfifo(struct device *device,
			      struct device_attribute *attr,
			      const char *buf, size_t size);
ssize_t st_acc33_get_max_hwfifo_watermark(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);
ssize_t st_acc33_get_hwfifo_watermark(struct device *device,
				      struct device_attribute *attr,
				      char *buf);
ssize_t st_acc33_set_hwfifo_watermark(struct device *device,
				      struct device_attribute *attr,
				      const char *buf, size_t size);
int st_acc33_update_watermark(struct st_acc33_hw *hw, u8 watermark);

int st_acc33_update_threshold_events(struct st_acc33_hw *hw);
int st_acc33_update_duration_events(struct st_acc33_hw *hw);
int st_acc33_event_init(struct st_acc33_hw *hw);

int st_acc33_read_event_config(struct iio_dev *iio_dev,
			       const struct iio_chan_spec *chan,
			       enum iio_event_type type,
			       enum iio_event_direction dir);
int st_acc33_write_event_config(struct iio_dev *iio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				int enable);
int st_acc33_read_event_value(struct iio_dev *iio_dev,
			      const struct iio_chan_spec *chan,
			      enum iio_event_type type,
			      enum iio_event_direction dir,
			      enum iio_event_info info,
			      int *val, int *val2);
int st_acc33_write_event_value(struct iio_dev *iio_dev,
			       const struct iio_chan_spec *chan,
			       enum iio_event_type type,
			       enum iio_event_direction dir,
			       enum iio_event_info info,
			       int val, int val2);
int st_acc33_event_handler(struct st_acc33_hw *hw);

#endif /* ST_ACC33_H */
