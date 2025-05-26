// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_asm330lhhx FIFO buffer library driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2019 STMicroelectronics Inc.
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/version.h>

#if KERNEL_VERSION(6, 11, 0) < LINUX_VERSION_CODE
#include <linux/unaligned.h>
#else /* LINUX_VERSION_CODE */
#include <asm/unaligned.h>
#endif /* LINUX_VERSION_CODE */

#include "st_asm330lhhx.h"

#define ST_ASM330LHHX_REG_FIFO_STATUS1_ADDR		0x3a
#define ST_ASM330LHHX_REG_FIFO_WTM_IA_MASK		BIT(7)

#define ST_ASM330LHHX_REG_TIMESTAMP2_ADDR		0x42
#define ST_ASM330LHHX_REG_FIFO_DATA_OUT_TAG_ADDR	0x78

#define ST_ASM330LHHX_SAMPLE_DISCHARD			0x7ffd

/* Timestamp convergence filter parameter */
#define ST_ASM330LHHX_EWMA_LEVEL			120
#define ST_ASM330LHHX_EWMA_DIV				128

#define ST_ASM330LHHX_TIMESTAMP_RESET_VALUE		0xaa

enum {
	ST_ASM330LHHX_GYRO_TAG = 0x01,
	ST_ASM330LHHX_ACC_TAG = 0x02,
	ST_ASM330LHHX_TEMP_TAG = 0x03,
	ST_ASM330LHHX_TS_TAG = 0x04,
	ST_ASM330LHHX_EXT0_TAG = 0x0f,
	ST_ASM330LHHX_EXT1_TAG = 0x10,
};

/* Default timeout before to re-enable gyro */
int delay_gyro = 10;
module_param(delay_gyro, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(delay_gyro, "Delay for Gyro arming");
static bool delayed_enable_gyro;

static inline s64 st_asm330lhhx_ewma(s64 old, s64 new, int weight)
{
	s64 diff, incr;

	diff = new - old;
	incr = div_s64((ST_ASM330LHHX_EWMA_DIV - weight) * diff,
		       ST_ASM330LHHX_EWMA_DIV);

	return old + incr;
}

inline int st_asm330lhhx_reset_hwts(struct st_asm330lhhx_hw *hw)
{
	u8 data = ST_ASM330LHHX_TIMESTAMP_RESET_VALUE;
	int ret;

	ret = st_asm330lhhx_write_locked(hw,
					 ST_ASM330LHHX_REG_TIMESTAMP2_ADDR,
					 data);
	if (ret < 0)
		return ret;

	hw->ts = st_asm330lhhx_get_time_ns(hw->iio_devs[0]);
	hw->ts_offset = hw->ts;
	hw->val_ts_old = 0ULL;
	hw->hw_ts_high_fifo = 0ULL;
	hw->tsample = 0ULL;

	return 0;
}

static void __maybe_unused
st_asm330lhhx_init_timesync_counter(struct st_asm330lhhx_sensor *sensor,
				    struct st_asm330lhhx_hw *hw,
				    bool enable)
{
	spin_lock_irq(&hw->hwtimestamp_lock);
	hw->timesync_ktime = ktime_set(0, ST_ASM330LHHX_FAST_KTIME);
	if (sensor->id <= ST_ASM330LHHX_ID_EXT1)
		hw->timesync_c[sensor->id] = enable ? ST_ASM330LHHX_FAST_TO_DEFAULT : 0;

	spin_unlock_irq(&hw->hwtimestamp_lock);
}

int st_asm330lhhx_set_fifo_mode(struct st_asm330lhhx_hw *hw,
				enum st_asm330lhhx_fifo_mode fifo_mode)
{
	int err;

	err = st_asm330lhhx_write_with_mask_locked(hw,
					  ST_ASM330LHHX_REG_FIFO_CTRL4_ADDR,
					  ST_ASM330LHHX_REG_FIFO_MODE_MASK,
					  fifo_mode);
	if (err < 0)
		return err;

	hw->fifo_mode = fifo_mode;

	if (fifo_mode == ST_ASM330LHHX_FIFO_BYPASS)
		clear_bit(ST_ASM330LHHX_HW_OPERATIONAL, &hw->state);
	else
		set_bit(ST_ASM330LHHX_HW_OPERATIONAL, &hw->state);

	return 0;
}

static inline int
st_asm330lhhx_set_sensor_batching_odr(struct st_asm330lhhx_sensor *s,
					    bool enable)
{
	enum st_asm330lhhx_sensor_id id = s->id;
	struct st_asm330lhhx_hw *hw = s->hw;
	int req_odr = enable ? s->odr : 0;
	u8 data = 0;
	int err;

	switch (id) {
	case ST_ASM330LHHX_ID_EXT0:
	case ST_ASM330LHHX_ID_EXT1:
	case ST_ASM330LHHX_ID_TEMP:
	case ST_ASM330LHHX_ID_ACC: {
		int odr;
		int i;

		if (id == ST_ASM330LHHX_ID_TEMP) {
			if (enable) {
				err = st_asm330lhhx_get_batch_val(s, req_odr,
								  0, &data);
				if (err < 0)
					return err;
			}

			err = st_asm330lhhx_update_bits_locked(hw,
				hw->odr_table_entry[id].batching_reg.addr,
				hw->odr_table_entry[id].batching_reg.mask,
				data);
			if (err < 0)
				return err;
		}

		id = ST_ASM330LHHX_ID_ACC;
		for (i = ST_ASM330LHHX_ID_ACC; i < ST_ASM330LHHX_ID_MAX; i++) {
			if (!hw->iio_devs[i] || i == s->id)
				continue;

			odr = st_asm330lhhx_check_odr_dependency(hw, req_odr,
								 0, i);
			if (odr != req_odr)
				return 0;

			req_odr = max_t(int, req_odr, odr);
		}
		break;
	}
	case ST_ASM330LHHX_ID_GYRO:
		break;
	default:
		return 0;
	}

	err = st_asm330lhhx_get_batch_val(s, req_odr, 0, &data);
	if (err < 0)
		return err;

	return st_asm330lhhx_update_bits_locked(hw,
				hw->odr_table_entry[id].batching_reg.addr,
				hw->odr_table_entry[id].batching_reg.mask,
				data);
}

int st_asm330lhhx_update_watermark(struct st_asm330lhhx_sensor *sensor,
				   u16 watermark)
{
	u16 fifo_watermark = ST_ASM330LHHX_MAX_FIFO_DEPTH, cur_watermark = 0;
	struct st_asm330lhhx_hw *hw = sensor->hw;
	struct st_asm330lhhx_sensor *cur_sensor;
	__le16 wdata;
	int data = 0;
	int i, err;

	for (i = ST_ASM330LHHX_ID_GYRO; i <= ST_ASM330LHHX_ID_EXT1; i++) {
		if (!hw->iio_devs[i])
			continue;

		cur_sensor = iio_priv(hw->iio_devs[i]);

		if (!(hw->enable_mask & BIT_ULL(cur_sensor->id)))
			continue;

		cur_watermark = (cur_sensor == sensor) ? watermark
						       : cur_sensor->watermark;

		fifo_watermark = min_t(u16, fifo_watermark, cur_watermark);
	}

	fifo_watermark = max_t(u16, fifo_watermark, 2);

	mutex_lock(&hw->page_lock);
	err = regmap_read(hw->regmap, ST_ASM330LHHX_REG_FIFO_CTRL1_ADDR + 1,
			  &data);
	if (err < 0)
		goto out;

	fifo_watermark = ((data << 8) & ~ST_ASM330LHHX_REG_FIFO_WTM_MASK) |
			 (fifo_watermark & ST_ASM330LHHX_REG_FIFO_WTM_MASK);
	wdata = cpu_to_le16(fifo_watermark);

	err = regmap_bulk_write(hw->regmap,
				ST_ASM330LHHX_REG_FIFO_CTRL1_ADDR,
				&wdata, sizeof(wdata));
out:
	mutex_unlock(&hw->page_lock);

	return err < 0 ? err : 0;
}

static struct iio_dev *st_asm330lhhx_get_iiodev_from_tag(struct st_asm330lhhx_hw *hw,
							 u8 tag)
{
	struct iio_dev *iio_dev;

	switch (tag) {
	case ST_ASM330LHHX_GYRO_TAG:
		iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_GYRO];
		break;
	case ST_ASM330LHHX_ACC_TAG:
		iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_ACC];
		break;
	case ST_ASM330LHHX_TEMP_TAG:
		iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_TEMP];
		break;
	case ST_ASM330LHHX_EXT0_TAG:
		if (hw->enable_mask & BIT_ULL(ST_ASM330LHHX_ID_EXT0))
			iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_EXT0];
		else
			iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_EXT1];
		break;
	case ST_ASM330LHHX_EXT1_TAG:
		iio_dev = hw->iio_devs[ST_ASM330LHHX_ID_EXT1];
		break;
	default:
		iio_dev = NULL;
		break;
	}

	return iio_dev;
}

static inline void st_asm330lhhx_sync_hw_ts(struct st_asm330lhhx_hw *hw, s64 ts)
{
	s64 delta = ts - hw->hw_ts;

	hw->ts_offset = st_asm330lhhx_ewma(hw->ts_offset, delta,
					  ST_ASM330LHHX_EWMA_LEVEL);
}

static int st_asm330lhhx_enable_irqline(struct st_asm330lhhx_hw *hw,
					bool enable)
{
	int err;

	/* disable FIFO watermak interrupt */
	err = st_asm330lhhx_write_locked(hw, hw->drdy_reg,
					enable ?
					ST_ASM330LHHX_REG_INT_FIFO_TH_MASK : 0);
	if (err < 0)
		return err;

	/* disable embedded function interrupt */
	if (st_asm330lhhx_events_enabled(hw)) {
		err = st_asm330lhhx_write_locked(hw,
				      hw->embfunc_pg0_irq_reg,
				      enable ? hw->interrupt_enable : 0);
		if (err < 0)
			return err;
	}

	return 0;
}

static int st_asm330lhhx_read_fifo(struct st_asm330lhhx_hw *hw)
{
	u8 iio_buf[ALIGN(ST_ASM330LHHX_SAMPLE_SIZE, sizeof(s64)) +
		   sizeof(s64) + sizeof(s64)];
	u8 buf[32 * ST_ASM330LHHX_FIFO_SAMPLE_SIZE], tag, *ptr;
	int i, err, word_len, fifo_len, read_len;
	bool already_updated = false;
	__le64 hw_timestamp_push;
	struct iio_dev *iio_dev;
	s64 ts_irq, hw_ts_old;
	__le16 fifo_status;
	u16 fifo_depth;
	s16 drdymask;
	u32 val;

	/* return if FIFO is already disabled */
	if (!test_bit(ST_ASM330LHHX_HW_OPERATIONAL, &hw->state)) {
		dev_warn(hw->dev, "%s: FIFO in bypass mode\n", __func__);

		return 0;
	}

	if (hw->irq_edge) {
		/* disable interrupt line */
		err = st_asm330lhhx_enable_irqline(hw, false);
		if (err < 0)
			return err;
	}

	ts_irq = hw->ts - hw->delta_ts;

	err = st_asm330lhhx_read_locked(hw, ST_ASM330LHHX_REG_FIFO_STATUS1_ADDR,
				    &fifo_status, sizeof(fifo_status));
	if (err < 0)
		goto enable_fifo;

	fifo_depth = le16_to_cpu(fifo_status) &
		     ST_ASM330LHHX_REG_FIFO_STATUS_DIFF;
	if (!fifo_depth)
		goto enable_fifo;

	fifo_len = fifo_depth * ST_ASM330LHHX_FIFO_SAMPLE_SIZE;
	read_len = 0;
	while (read_len < fifo_len) {
		word_len = min_t(int, fifo_len - read_len, sizeof(buf));
		err = st_asm330lhhx_read_locked(hw,
				   ST_ASM330LHHX_REG_FIFO_DATA_OUT_TAG_ADDR,
				   buf, word_len);
		if (err < 0)
			goto enable_fifo;

		for (i = 0; i < word_len; i += ST_ASM330LHHX_FIFO_SAMPLE_SIZE) {
			ptr = &buf[i + ST_ASM330LHHX_TAG_SIZE];
			tag = buf[i] >> 3;

			if (tag == ST_ASM330LHHX_TS_TAG) {
				val = get_unaligned_le32(ptr);

				/* check hw rollover, just once for batching cycle */
				if ((hw->val_ts_old > val) &&
				    !already_updated) {
					hw->hw_ts_high_fifo++;
					already_updated = true;
				}

				hw->val_ts_old = val;

				hw_ts_old = hw->hw_ts;

				if (IS_ENABLED(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)) {
					spin_lock_irq(&hw->hwtimestamp_lock);

					hw->hw_timestamp_global =
						(hw->hw_timestamp_global &
						GENMASK_ULL(63, ST_ASM330LHHX_RESET_COUNT)) |
						((s64)hw->hw_ts_high_fifo << 32) | val;

					spin_unlock_irq(&hw->hwtimestamp_lock);
				}

				hw->hw_ts = (val + ((s64)hw->hw_ts_high_fifo << 32)) *
					    hw->ts_delta_ns;
				hw->ts_offset = st_asm330lhhx_ewma(hw->ts_offset,
						ts_irq - hw->hw_ts,
						ST_ASM330LHHX_EWMA_LEVEL);

				if (!test_bit(ST_ASM330LHHX_HW_FLUSH, &hw->state))
					/* sync ap timestamp and sensor one */
					st_asm330lhhx_sync_hw_ts(hw, ts_irq);

				ts_irq += hw->hw_ts;

				if (!hw->tsample)
					hw->tsample = hw->ts_offset + hw->hw_ts;
				else
					hw->tsample = hw->tsample + hw->hw_ts - hw_ts_old;
			} else {
				struct st_asm330lhhx_sensor *sensor;

				iio_dev = st_asm330lhhx_get_iiodev_from_tag(hw, tag);
				if (!iio_dev)
					continue;

				sensor = iio_priv(iio_dev);

				/* skip samples if not ready */
				drdymask = (s16)le16_to_cpu(get_unaligned_le16(ptr));
				if (unlikely(drdymask >= ST_ASM330LHHX_SAMPLE_DISCHARD)) {
#ifdef ST_ASM330LHHX_DEBUG_DISCHARGE
					sensor->discharged_samples++;
#endif /* ST_ASM330LHHX_DEBUG_DISCHARGE */
					continue;
				}

				memcpy(iio_buf, ptr, ST_ASM330LHHX_SAMPLE_SIZE);
				hw->tsample = min_t(s64,
						    st_asm330lhhx_get_time_ns(hw->iio_devs[0]),
						    hw->tsample);

				if (IS_ENABLED(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)) {
					spin_lock_irq(&hw->hwtimestamp_lock);

					hw_timestamp_push = cpu_to_le64(hw->hw_timestamp_global);

					memcpy(&iio_buf[ALIGN(ST_ASM330LHHX_SAMPLE_SIZE, sizeof(s64))],
					       &hw_timestamp_push, sizeof(hw_timestamp_push));

					spin_unlock_irq(&hw->hwtimestamp_lock);
				} else {
					hw_timestamp_push = hw->tsample;
				}

				/* LPF sample discard */
				if (sensor->discard_samples) {
					sensor->discard_samples--;
					continue;
				}

				/* support decimation for ODR < 12.5 Hz */
				if (sensor->dec_counter > 0) {
					sensor->dec_counter--;
				} else {
					sensor->dec_counter = sensor->decimator;
					iio_push_to_buffers_with_timestamp(iio_dev,
								iio_buf,
								hw->tsample);
					sensor->last_fifo_timestamp = hw_timestamp_push;
				}
			}
		}
		read_len += word_len;
	}

enable_fifo:
	if (hw->irq_edge) {
		/* enable interrupt line */
		err = st_asm330lhhx_enable_irqline(hw, true);
		if (err < 0)
			return err;
	}

	return read_len;
}

ssize_t st_asm330lhhx_get_max_watermark(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%d\n", sensor->max_watermark);
}

ssize_t st_asm330lhhx_get_watermark(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%d\n", sensor->watermark);
}

ssize_t st_asm330lhhx_set_watermark(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	int err, val;

	if (!sensor->hw->has_hw_fifo) {
		err = -EINVAL;

		return err;
	}

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		goto out;

	err = st_asm330lhhx_update_watermark(sensor, val);
	if (err < 0)
		goto out;

	sensor->watermark = val;

out:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

int __maybe_unused
st_asm330lhhx_flush_fifo_during_resume(struct st_asm330lhhx_hw *hw)
{
	int count;

	mutex_lock(&hw->fifo_lock);
	count = st_asm330lhhx_read_fifo(hw);
	mutex_unlock(&hw->fifo_lock);

	return count;
}

ssize_t st_asm330lhhx_flush_fifo(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
	s64 event;
	int count;
	s64 type;
	s64 fts;
	s64 ts;

	if (!hw->has_hw_fifo)
		return -EINVAL;

	mutex_lock(&hw->fifo_lock);
	ts = st_asm330lhhx_get_time_ns(iio_dev);
	hw->delta_ts = ts - hw->ts;
	hw->ts = ts;
	set_bit(ST_ASM330LHHX_HW_FLUSH, &hw->state);
	count = st_asm330lhhx_read_fifo(hw);
	sensor->dec_counter = 0;
	fts = sensor->last_fifo_timestamp;
	mutex_unlock(&hw->fifo_lock);

	type = count > 0 ? STM_IIO_EV_DIR_FIFO_DATA : STM_IIO_EV_DIR_FIFO_EMPTY;
	event = IIO_UNMOD_EVENT_CODE(iio_dev->channels[0].type, -1,
				     STM_IIO_EV_TYPE_FIFO_FLUSH, type);
	iio_push_event(iio_dev, event, fts);

	return size;
}

int st_asm330lhhx_suspend_fifo(struct st_asm330lhhx_hw *hw)
{
	int err;

	if (!hw->has_hw_fifo)
		return 0;

	mutex_lock(&hw->fifo_lock);
	st_asm330lhhx_read_fifo(hw);
	err = st_asm330lhhx_set_fifo_mode(hw, ST_ASM330LHHX_FIFO_BYPASS);
	mutex_unlock(&hw->fifo_lock);

	return err;
}

int st_asm330lhhx_update_batching(struct iio_dev *iio_dev, bool enable)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int err;

	if (!hw->has_hw_fifo)
		return 0;

	disable_irq(hw->irq);

	err = st_asm330lhhx_set_sensor_batching_odr(sensor, enable);
	enable_irq(hw->irq);

	return err;
}

static int
st_asm330lhhx_update_fifo(struct st_asm330lhhx_sensor *sensor,
			  bool enable)
{
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int err;

	if (sensor->id == ST_ASM330LHHX_ID_GYRO && !enable)
		delayed_enable_gyro = true;

	if (sensor->id == ST_ASM330LHHX_ID_GYRO &&
	    enable && delayed_enable_gyro) {
		delayed_enable_gyro = false;
		msleep(delay_gyro);
	}

	disable_irq(hw->irq);

	if (IS_ENABLED(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)) {
		hrtimer_cancel(&hw->timesync_timer);
		cancel_work_sync(&hw->timesync_work);
	}

	if (sensor->id == ST_ASM330LHHX_ID_EXT0 ||
	    sensor->id == ST_ASM330LHHX_ID_EXT1) {
		err = st_asm330lhhx_shub_set_enable(sensor, enable);
		if (err < 0)
			goto out;
	} else {
		err = st_asm330lhhx_sensor_set_enable(sensor, enable);
		if (err < 0)
			goto out;

		err = st_asm330lhhx_set_sensor_batching_odr(sensor, enable);
		if (err < 0)
			goto out;
	}

	err = st_asm330lhhx_update_watermark(sensor, sensor->watermark);
	if (err < 0)
		goto out;

	if (enable && hw->fifo_mode == ST_ASM330LHHX_FIFO_BYPASS) {
		if (!IS_ENABLED(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP))
			st_asm330lhhx_reset_hwts(hw);

		err = st_asm330lhhx_set_fifo_mode(hw, ST_ASM330LHHX_FIFO_CONT);
	} else if (!hw->enable_mask) {
		err = st_asm330lhhx_set_fifo_mode(hw, ST_ASM330LHHX_FIFO_BYPASS);
	}

	if (IS_ENABLED(CONFIG_IIO_ST_ASM330LHHX_ASYNC_HW_TIMESTAMP)) {
		st_asm330lhhx_init_timesync_counter(sensor, hw, enable);
		if (hw->fifo_mode != ST_ASM330LHHX_FIFO_BYPASS) {
			hrtimer_start(&hw->timesync_timer,
				ktime_set(0, 0),
				HRTIMER_MODE_REL);
		}
	}

out:
	enable_irq(hw->irq);

	return err;
}

static int st_asm330lhhx_update_enable(struct st_asm330lhhx_sensor *sensor,
				       bool enable)
{
	if (sensor->id == ST_ASM330LHHX_ID_EXT0 ||
	    sensor->id == ST_ASM330LHHX_ID_EXT1)
		return st_asm330lhhx_shub_set_enable(sensor, enable);

	return st_asm330lhhx_sensor_set_enable(sensor, enable);
}

static int st_asm330lhhx_buffer_enable(struct iio_dev *iio_dev, bool enable)
{
	struct st_asm330lhhx_sensor *sensor = iio_priv(iio_dev);

	if (sensor->hw->has_hw_fifo)
		return st_asm330lhhx_update_fifo(sensor, enable);

	return st_asm330lhhx_update_enable(sensor, enable);
}

static irqreturn_t st_asm330lhhx_handler_irq(int irq, void *private)
{
	struct st_asm330lhhx_hw *hw = (struct st_asm330lhhx_hw *)private;
	s64 ts = st_asm330lhhx_get_time_ns(hw->iio_devs[0]);

	hw->delta_ts = ts - hw->ts;
	hw->ts = ts;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_asm330lhhx_handler_thread(int irq, void *private)
{
	struct st_asm330lhhx_hw *hw = (struct st_asm330lhhx_hw *)private;

	if (hw->settings->st_mlc_probe)
		st_asm330lhhx_mlc_check_status(hw);

	mutex_lock(&hw->fifo_lock);
	st_asm330lhhx_read_fifo(hw);
	clear_bit(ST_ASM330LHHX_HW_FLUSH, &hw->state);
	mutex_unlock(&hw->fifo_lock);

	/* if irq line is the same of FIFO manage it */
	if (hw->irq_emb == hw->irq)
		return st_asm330lhhx_event_handler(hw);

	return IRQ_HANDLED;
}

static int st_asm330lhhx_fifo_preenable(struct iio_dev *iio_dev)
{
	return st_asm330lhhx_buffer_enable(iio_dev, true);
}

static int st_asm330lhhx_fifo_postdisable(struct iio_dev *iio_dev)
{
	return st_asm330lhhx_buffer_enable(iio_dev, false);
}

static const struct iio_buffer_setup_ops st_asm330lhhx_buffer_setup_ops = {
	.preenable = st_asm330lhhx_fifo_preenable,
#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
#endif /* LINUX_VERSION_CODE */
	.postdisable = st_asm330lhhx_fifo_postdisable,
 };

static irqreturn_t st_asm330lhhx_buffer_pollfunc(int irq, void *private)
{
	u8 iio_buf[ALIGN(ST_ASM330LHHX_SAMPLE_SIZE, sizeof(s64)) +
		   sizeof(s64) + sizeof(s64)];
	struct iio_poll_func *pf = private;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct st_asm330lhhx_sensor *sensor = iio_priv(indio_dev);
	struct st_asm330lhhx_hw *hw = sensor->hw;
	int addr = indio_dev->channels[0].address;

	switch (indio_dev->channels[0].type) {
	case IIO_ACCEL:
	case IIO_ANGL_VEL:
		st_asm330lhhx_read_locked(hw, addr, &iio_buf,
					  ST_ASM330LHHX_SAMPLE_SIZE);
		break;
	case IIO_TEMP:
		st_asm330lhhx_read_locked(hw, addr, &iio_buf,
					  ST_ASM330LHHX_PT_SAMPLE_SIZE);
		break;
	case IIO_PRESSURE:
		st_asm330lhhx_shub_read(sensor, addr, (u8 *)&iio_buf,
					ST_ASM330LHHX_PT_SAMPLE_SIZE);
		break;
	case IIO_MAGN:
		st_asm330lhhx_shub_read(sensor, addr, (u8 *)&iio_buf,
					ST_ASM330LHHX_SAMPLE_SIZE);
		break;
	default:
		return -EINVAL;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, iio_buf,
				  st_asm330lhhx_get_time_ns(indio_dev));
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int st_asm330lhhx_trig_set_state(struct iio_trigger *trig,
					bool state)
{
	struct st_asm330lhhx_hw *hw = iio_trigger_get_drvdata(trig);

	dev_dbg(hw->dev, "trigger set %d\n", state);

	return 0;
}

static const struct iio_trigger_ops st_asm330lhhx_trigger_ops = {
	.set_trigger_state = st_asm330lhhx_trig_set_state,
};

static int st_asm330lhhx_config_interrupt(struct st_asm330lhhx_hw *hw,
					  bool enable)
{
	int err;

	err = st_asm330lhhx_get_int_reg(hw);
	if (err < 0)
		return err;

	/* latch interrupts */
	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_INT_CFG0_ADDR,
				 ST_ASM330LHHX_LIR_MASK,
				 FIELD_PREP(ST_ASM330LHHX_LIR_MASK,
					    enable ? 1 : 0));
	if (err < 0)
		return err;

	/* enable FIFO watermak interrupt */
	return regmap_update_bits(hw->regmap, hw->drdy_reg,
				  ST_ASM330LHHX_REG_INT_FIFO_TH_MASK,
				  FIELD_PREP(ST_ASM330LHHX_REG_INT_FIFO_TH_MASK,
					     enable ? 1 : 0));
}

static int st_asm330lhhx_config_timestamp(struct st_asm330lhhx_hw *hw)
{
	int err;

	err = st_asm330lhhx_hwtimesync_init(hw);
	if (err)
		return err;

	/* init timestamp engine */
	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_CTRL10_C_ADDR,
				 ST_ASM330LHHX_REG_TIMESTAMP_EN_MASK,
				 ST_ASM330LHHX_SHIFT_VAL(1,
				  ST_ASM330LHHX_REG_TIMESTAMP_EN_MASK));
	if (err < 0)
		return err;

	return regmap_update_bits(hw->regmap,
				  ST_ASM330LHHX_REG_FIFO_CTRL4_ADDR,
				  ST_ASM330LHHX_REG_DEC_TS_MASK,
				  FIELD_PREP(ST_ASM330LHHX_REG_DEC_TS_MASK, 1));
}

int st_asm330lhhx_allocate_buffers(struct st_asm330lhhx_hw *hw)
{
	int i;

	for (i = 0;
	     i < ARRAY_SIZE(st_asm330lhhx_triggered_main_sensor_list);
	     i++) {
		enum st_asm330lhhx_sensor_id id;
		int err;

		id = st_asm330lhhx_triggered_main_sensor_list[i];
		if (!hw->iio_devs[id])
			continue;

		err = devm_iio_triggered_buffer_setup(hw->dev,
				hw->iio_devs[id], NULL,
				st_asm330lhhx_buffer_pollfunc,
				&st_asm330lhhx_buffer_setup_ops);
		if (err)
		return err;
	}

	return 0;
}

int st_asm330lhhx_trigger_setup(struct st_asm330lhhx_hw *hw)
{
	unsigned long irq_type;
	bool irq_active_low;
	bool irq_edge;
	int i, err;

	irq_type = irqd_get_trigger_type(irq_get_irq_data(hw->irq));
	if (irq_type == IRQF_TRIGGER_NONE)
		irq_type = IRQF_TRIGGER_HIGH;

	switch (irq_type) {
	case IRQF_TRIGGER_HIGH:
		irq_active_low = false;
		irq_edge = false;
		break;
	case IRQF_TRIGGER_LOW:
		irq_active_low = true;
		irq_edge = false;
		break;
	case IRQF_TRIGGER_RISING:
		irq_active_low = false;
		irq_edge = true;
		break;
	case IRQF_TRIGGER_FALLING:
		irq_active_low = true;
		irq_edge = true;
		break;
	default:
		dev_info(hw->dev, "mode %lx unsupported\n", irq_type);
		return -EINVAL;
	}

	err = regmap_update_bits(hw->regmap,
				 ST_ASM330LHHX_REG_CTRL3_C_ADDR,
				 ST_ASM330LHHX_REG_H_LACTIVE_MASK,
				 FIELD_PREP(ST_ASM330LHHX_REG_H_LACTIVE_MASK,
				 irq_active_low));
	if (err < 0)
		return err;

	if (device_property_read_bool(hw->dev, "drive-open-drain")) {
		err = regmap_update_bits(hw->regmap,
					 ST_ASM330LHHX_REG_CTRL3_C_ADDR,
					 ST_ASM330LHHX_REG_PP_OD_MASK,
					 FIELD_PREP(ST_ASM330LHHX_REG_PP_OD_MASK,
						    1));
		if (err < 0)
			return err;

		irq_type |= IRQF_SHARED;
	}

	err = devm_request_threaded_irq(hw->dev, hw->irq,
					st_asm330lhhx_handler_irq,
					st_asm330lhhx_handler_thread,
					irq_type | IRQF_ONESHOT,
					hw->settings->id.name, hw);
	if (err) {
		dev_err(hw->dev, "failed to request trigger irq %d\n",
			hw->irq);
		return err;
	}

	/* attach trigger to iio devs */
	for (i = 0;
	     i < ARRAY_SIZE(st_asm330lhhx_triggered_main_sensor_list);
	     i++) {
		struct st_asm330lhhx_sensor *sensor;
		enum st_asm330lhhx_sensor_id id;

		id = st_asm330lhhx_triggered_main_sensor_list[i];
		if (!hw->iio_devs[id])
			continue;

		sensor = iio_priv(hw->iio_devs[id]);
		sensor->trig = devm_iio_trigger_alloc(hw->dev,
						"st_%s-trigger%d",
						hw->iio_devs[id]->name,
						hw->module_id);
		if (!sensor->trig) {
			dev_err(hw->dev, "failed to allocate iio trigger.\n");

			return -ENOMEM;
		}

		iio_trigger_set_drvdata(sensor->trig, hw);
		sensor->trig->ops = &st_asm330lhhx_trigger_ops;
		sensor->trig->dev.parent = hw->dev;

		err = devm_iio_trigger_register(hw->dev, sensor->trig);
		if (err < 0) {
			dev_err(hw->dev, "failed to register iio trigger.\n");

			return err;
		}

		hw->iio_devs[id]->trig = iio_trigger_get(sensor->trig);
	}

	err = st_asm330lhhx_config_interrupt(hw, true);
	if (err < 0)
		return err;

	hw->irq_edge = irq_edge;

	return st_asm330lhhx_config_timestamp(hw);
}
