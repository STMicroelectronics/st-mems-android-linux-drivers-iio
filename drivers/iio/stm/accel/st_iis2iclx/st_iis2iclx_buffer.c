// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_iis2iclx FIFO buffer library driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
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

#include "st_iis2iclx.h"

#define ST_IIS2ICLX_REG_FIFO_STATUS1_ADDR		0x3a
#define ST_IIS2ICLX_REG_TIMESTAMP2_ADDR			0x42
#define ST_IIS2ICLX_REG_FIFO_DATA_OUT_TAG_ADDR		0x78

#define ST_IIS2ICLX_SAMPLE_DISCHARD			0x7ffd

/* Timestamp convergence filter parameter */
#define ST_IIS2ICLX_EWMA_LEVEL				120
#define ST_IIS2ICLX_EWMA_DIV				128

#define ST_IIS2ICLX_TIMESTAMP_RESET_VALUE		0xaa

enum {
	ST_IIS2ICLX_ACC_TAG = 0x02,
	ST_IIS2ICLX_TEMP_TAG = 0x03,
	ST_IIS2ICLX_TS_TAG = 0x04,
	ST_IIS2ICLX_EXT0_TAG = 0x0f,
	ST_IIS2ICLX_EXT1_TAG = 0x10,
};

static inline s64 st_iis2iclx_ewma(s64 old, s64 new, int weight)
{
	s64 diff, incr;

	diff = new - old;
	incr = div_s64((ST_IIS2ICLX_EWMA_DIV - weight) * diff,
		       ST_IIS2ICLX_EWMA_DIV);

	return old + incr;
}

inline int st_iis2iclx_reset_hwts(struct st_iis2iclx_hw *hw)
{
	u8 data = ST_IIS2ICLX_TIMESTAMP_RESET_VALUE;
	int ret;

	ret = st_iis2iclx_write_locked(hw, ST_IIS2ICLX_REG_TIMESTAMP2_ADDR,
				       data);
	if (ret < 0)
		return ret;

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
	spin_lock_irq(&hw->hwtimestamp_lock);
	hw->hw_timestamp_global = (hw->hw_timestamp_global + (1LL << 32)) &
				  GENMASK_ULL(63, 32);
	hw->timesync_ktime = ktime_set(0, ST_IIS2ICLX_FAST_KTIME);
	spin_unlock_irq(&hw->hwtimestamp_lock);
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

	hw->irq_ts = st_iis2iclx_get_time_ns(hw->iio_devs[0]);
	hw->ts_offset = hw->irq_ts;
	hw->val_ts_old = 0ULL;
	hw->hw_ts_high = 0ULL;
	hw->tsample = 0ULL;

	return 0;
}

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
static void st_iis2iclx_init_timesync_counter(struct st_iis2iclx_sensor *sensor,
					      struct st_iis2iclx_hw *hw,
					      bool enable)
{
	spin_lock_irq(&hw->hwtimestamp_lock);
	if (sensor->id <= ST_IIS2ICLX_ID_HW)
		hw->timesync_c[sensor->id] = enable ?
					     ST_IIS2ICLX_FAST_TO_DEFAULT : 0;

	spin_unlock_irq(&hw->hwtimestamp_lock);
}
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

int st_iis2iclx_set_fifo_mode(struct st_iis2iclx_hw *hw,
			      enum st_iis2iclx_fifo_mode fifo_mode)
{
	int err;

	err = st_iis2iclx_write_with_mask_locked(hw,
					ST_IIS2ICLX_REG_FIFO_CTRL4_ADDR,
					ST_IIS2ICLX_FIFO_MODE_MASK,
					fifo_mode);
	if (err < 0)
		return err;

	hw->fifo_mode = fifo_mode;

	if (fifo_mode == ST_IIS2ICLX_FIFO_BYPASS)
		clear_bit(ST_IIS2ICLX_HW_OPERATIONAL, &hw->state);
	else
		set_bit(ST_IIS2ICLX_HW_OPERATIONAL, &hw->state);

	return 0;
}

static inline int
st_iis2iclx_set_sensor_batching_odr(struct st_iis2iclx_sensor *s, bool enable)
{
	enum st_iis2iclx_sensor_id id = s->id;
	struct st_iis2iclx_hw *hw = s->hw;
	u8 data = 0;
	int err;

	if (enable) {
		err = st_iis2iclx_get_batch_val(s, s->odr, s->uodr, &data);
		if (err < 0)
			return err;
	}

	return st_iis2iclx_update_bits_locked(hw,
				hw->odr_table_entry[id].batching_reg.addr,
				hw->odr_table_entry[id].batching_reg.mask,
				data);
}

int st_iis2iclx_update_watermark(struct st_iis2iclx_sensor *sensor,
				 u16 watermark)
{
	u16 fifo_watermark = ST_IIS2ICLX_MAX_FIFO_DEPTH, cur_watermark = 0;
	struct st_iis2iclx_hw *hw = sensor->hw;
	struct st_iis2iclx_sensor *cur_sensor;
	__le16 wdata;
	int data = 0;
	int i, err;

	for (i = ST_IIS2ICLX_ID_ACC; i <= ST_IIS2ICLX_ID_EXT1; i++) {
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
	err = regmap_read(hw->regmap, ST_IIS2ICLX_REG_FIFO_CTRL1_ADDR + 1,
			  &data);
	if (err < 0)
		goto out;

	fifo_watermark = ((data << 8) & ~ST_IIS2ICLX_FIFO_WTM_MASK) |
			  (fifo_watermark & ST_IIS2ICLX_FIFO_WTM_MASK);
	wdata = cpu_to_le16(fifo_watermark);

	err = regmap_bulk_write(hw->regmap, ST_IIS2ICLX_REG_FIFO_CTRL1_ADDR,
				&wdata, sizeof(wdata));
out:
	mutex_unlock(&hw->page_lock);

	return err < 0 ? err : 0;
}

static struct
iio_dev *st_iis2iclx_get_iiodev_from_tag(struct st_iis2iclx_hw *hw, u8 tag)
{
	struct iio_dev *iio_dev;

	switch (tag) {
	case ST_IIS2ICLX_ACC_TAG:
		iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_ACC];
		break;
	case ST_IIS2ICLX_TEMP_TAG:
		iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_TEMP];
		break;
	case ST_IIS2ICLX_EXT0_TAG:
		if (hw->enable_mask & BIT_ULL(ST_IIS2ICLX_ID_EXT0))
			iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_EXT0];
		else
			iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_EXT1];
		break;
	case ST_IIS2ICLX_EXT1_TAG:
		iio_dev = hw->iio_devs[ST_IIS2ICLX_ID_EXT1];
		break;
	default:
		iio_dev = NULL;
		break;
	}

	return iio_dev;
}

static inline void st_iis2iclx_sync_hw_ts(struct st_iis2iclx_hw *hw, s64 time)
{
	s64 delta = time - hw->hw_ts;

	hw->ts_offset = st_iis2iclx_ewma(hw->ts_offset, delta,
					 ST_IIS2ICLX_EWMA_LEVEL);
}

static int st_iis2iclx_read_fifo(struct st_iis2iclx_hw *hw)
{
	u8 iio_buf[ALIGN(ST_IIS2ICLX_SAMPLE_SIZE, sizeof(s64)) +
		   sizeof(s64) + sizeof(s64)];
	u8 buf[6 * ST_IIS2ICLX_FIFO_SAMPLE_SIZE], tag, *ptr;
	int i, err, word_len, fifo_len, read_len;
	__le64 hw_timestamp_push;
	struct iio_dev *iio_dev;
	s64 ts_irq, hw_ts_old;
	__le16 fifo_status;
	u16 fifo_depth;
	s16 drdymask;
	u32 val;

	/* return if FIFO is already disabled */
	if (!test_bit(ST_IIS2ICLX_HW_OPERATIONAL, &hw->state)) {
		dev_warn(hw->dev, "%s: FIFO in bypass mode\n", __func__);

		return 0;
	}

	ts_irq = hw->irq_ts - hw->delta_ts;

	err = st_iis2iclx_read_locked(hw, ST_IIS2ICLX_REG_FIFO_STATUS1_ADDR,
				      &fifo_status, sizeof(fifo_status));
	if (err < 0)
		return err;

	fifo_depth = le16_to_cpu(fifo_status) &
		     ST_IIS2ICLX_FIFO_STATUS_DIFF;
	if (!fifo_depth)
		return 0;

	fifo_len = fifo_depth * ST_IIS2ICLX_FIFO_SAMPLE_SIZE;
	read_len = 0;
	while (read_len < fifo_len) {
		word_len = min_t(int, fifo_len - read_len, sizeof(buf));
		err = st_iis2iclx_read_locked(hw,
				ST_IIS2ICLX_REG_FIFO_DATA_OUT_TAG_ADDR,
				buf, word_len);
		if (err < 0)
			return err;

		for (i = 0; i < word_len; i += ST_IIS2ICLX_FIFO_SAMPLE_SIZE) {
			ptr = &buf[i + ST_IIS2ICLX_TAG_SIZE];
			tag = buf[i] >> 3;

			if (tag == ST_IIS2ICLX_TS_TAG) {
				val = get_unaligned_le32(ptr);

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
				spin_lock_irq(&hw->hwtimestamp_lock);
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

				hw->hw_timestamp_global =
					(hw->hw_timestamp_global &
					 GENMASK_ULL(63, 32)) |
					(u32)le32_to_cpu(get_unaligned_le32(ptr));

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
				spin_unlock_irq(&hw->hwtimestamp_lock);
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

				/* check hw rollover */
				if (hw->val_ts_old > val)
					hw->hw_ts_high++;

				hw_ts_old = hw->hw_ts;

				hw->val_ts_old = val;
				hw->hw_ts = (val + ((s64)hw->hw_ts_high << 32)) *
					    hw->ts_delta_ns;
				hw->ts_offset = st_iis2iclx_ewma(hw->ts_offset,
						ts_irq - hw->hw_ts,
						ST_IIS2ICLX_EWMA_LEVEL);

				if (!test_bit(ST_IIS2ICLX_HW_FLUSH, &hw->state))
					/* sync ap timestamp and sensor one */
					st_iis2iclx_sync_hw_ts(hw, ts_irq);

				ts_irq += hw->hw_ts;

				if (!hw->tsample)
					hw->tsample = hw->ts_offset + hw->hw_ts;
				else
					hw->tsample = hw->tsample + hw->hw_ts -
						      hw_ts_old;
			} else {
				struct st_iis2iclx_sensor *sensor;

				iio_dev = st_iis2iclx_get_iiodev_from_tag(hw,
									  tag);
				if (!iio_dev)
					continue;

				sensor = iio_priv(iio_dev);

				/* skip samples if not ready */
				drdymask = (s16)le16_to_cpu(get_unaligned_le16(ptr));
				if (unlikely(drdymask >= ST_IIS2ICLX_SAMPLE_DISCHARD)) {

#ifdef ST_IIS2ICLX_DEBUG_DISCHARGE
					sensor->discharged_samples++;
#endif /* ST_IIS2ICLX_DEBUG_DISCHARGE */

					continue;
				}

				memcpy(iio_buf, ptr, ST_IIS2ICLX_SAMPLE_SIZE);
				hw->tsample = min_t(s64,
				       st_iis2iclx_get_time_ns(hw->iio_devs[0]),
				       hw->tsample);

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
				spin_lock_irq(&hw->hwtimestamp_lock);

				hw_timestamp_push = cpu_to_le64(hw->hw_timestamp_global);
				memcpy(&iio_buf[ALIGN(ST_IIS2ICLX_SAMPLE_SIZE,
						      sizeof(s64))],
				       &hw_timestamp_push, sizeof(hw_timestamp_push));

				spin_unlock_irq(&hw->hwtimestamp_lock);
#else /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */
				hw_timestamp_push = hw->tsample;
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

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
							  iio_buf, hw->tsample);
					sensor->last_fifo_timestamp = hw_timestamp_push;
				}
			}
		}

		read_len += word_len;
	}

	return read_len;
}

ssize_t st_iis2iclx_get_max_watermark(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", sensor->max_watermark);
}

ssize_t st_iis2iclx_get_watermark(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", sensor->watermark);
}

ssize_t st_iis2iclx_set_watermark(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
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

	err = st_iis2iclx_update_watermark(sensor, val);
	if (err < 0)
		goto out;

	sensor->watermark = val;

out:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

ssize_t st_iis2iclx_flush_fifo(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	struct st_iis2iclx_hw *hw = sensor->hw;
	s64 timestamp;
	s64 event;
	int count;
	s64 type;
	s64 fts;

	if (!hw->has_hw_fifo)
		return -EINVAL;

	mutex_lock(&hw->fifo_lock);
	timestamp = st_iis2iclx_get_time_ns(iio_dev);
	hw->delta_ts = timestamp - hw->irq_ts;
	hw->irq_ts = timestamp;
	set_bit(ST_IIS2ICLX_HW_FLUSH, &hw->state);
	count = st_iis2iclx_read_fifo(hw);
	sensor->dec_counter = 0;
	fts = sensor->last_fifo_timestamp;
	mutex_unlock(&hw->fifo_lock);

	type = count > 0 ? STM_IIO_EV_DIR_FIFO_DATA : STM_IIO_EV_DIR_FIFO_EMPTY;
	event = IIO_UNMOD_EVENT_CODE(iio_dev->channels[0].type, -1,
				     STM_IIO_EV_TYPE_FIFO_FLUSH, type);
	iio_push_event(iio_dev, event, fts);

	return size;
}

int st_iis2iclx_suspend_fifo(struct st_iis2iclx_hw *hw)
{
	int err;

	if (!hw->has_hw_fifo)
		return 0;

	mutex_lock(&hw->fifo_lock);
	st_iis2iclx_read_fifo(hw);
	err = st_iis2iclx_set_fifo_mode(hw, ST_IIS2ICLX_FIFO_BYPASS);
	mutex_unlock(&hw->fifo_lock);

	return err;
}

int st_iis2iclx_update_batching(struct iio_dev *iio_dev, bool enable)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);
	struct st_iis2iclx_hw *hw = sensor->hw;
	int err;

	if (!hw->has_hw_fifo)
		return 0;

	disable_irq(hw->irq);

	err = st_iis2iclx_set_sensor_batching_odr(sensor, enable);
	enable_irq(hw->irq);

	return err;
}

static int st_iis2iclx_update_fifo(struct st_iis2iclx_sensor *sensor,
				   bool enable)
{
	struct st_iis2iclx_hw *hw = sensor->hw;
	int err;

	disable_irq(hw->irq);

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
	hrtimer_cancel(&hw->timesync_timer);
	cancel_work_sync(&hw->timesync_work);
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

	if (sensor->id == ST_IIS2ICLX_ID_EXT0 ||
	    sensor->id == ST_IIS2ICLX_ID_EXT1) {
		err = st_iis2iclx_shub_set_enable(sensor, enable);
		if (err < 0)
			goto out;
	} else {
		err = st_iis2iclx_sensor_set_enable(sensor, enable);
		if (err < 0)
			goto out;

		err = st_iis2iclx_set_sensor_batching_odr(sensor, enable);
		if (err < 0)
			goto out;
	}

	/*
	 * This is an auxiliary sensor, it need to get batched
	 * toghether at least with a primary sensor (Acc).
	 */
	if (sensor->id == ST_IIS2ICLX_ID_TEMP) {
		if (!(hw->enable_mask & BIT_ULL(ST_IIS2ICLX_ID_ACC))) {
			struct st_iis2iclx_sensor *acc_sensor;
			u8 data = 0;

			acc_sensor = iio_priv(hw->iio_devs[ST_IIS2ICLX_ID_ACC]);
			if (enable) {
				err = st_iis2iclx_get_batch_val(acc_sensor,
						sensor->odr, sensor->uodr,
						&data);
				if (err < 0)
					goto out;
			}

			err = st_iis2iclx_update_bits_locked(hw,
				hw->odr_table_entry[ST_IIS2ICLX_ID_ACC].batching_reg.addr,
				hw->odr_table_entry[ST_IIS2ICLX_ID_ACC].batching_reg.mask,
				data);
			if (err < 0)
				goto out;
		}
	}

	err = st_iis2iclx_update_watermark(sensor, sensor->watermark);
	if (err < 0)
		goto out;

	if (enable && hw->fifo_mode == ST_IIS2ICLX_FIFO_BYPASS) {
		st_iis2iclx_reset_hwts(hw);
		err = st_iis2iclx_set_fifo_mode(hw, ST_IIS2ICLX_FIFO_CONT);
	} else if (!hw->enable_mask) {
		err = st_iis2iclx_set_fifo_mode(hw, ST_IIS2ICLX_FIFO_BYPASS);
	}

#if defined(CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP)
	st_iis2iclx_init_timesync_counter(sensor, hw, enable);
	if (hw->fifo_mode != ST_IIS2ICLX_FIFO_BYPASS) {
		hrtimer_start(&hw->timesync_timer, ktime_set(0, 0),
			      HRTIMER_MODE_REL);
	}
#endif /* CONFIG_IIO_ST_IIS2ICLX_ASYNC_HW_TIMESTAMP */

out:
	enable_irq(hw->irq);

	return err;
}

static int st_iis2iclx_update_enable(struct st_iis2iclx_sensor *sensor,
				     bool enable)
{
	if (sensor->id == ST_IIS2ICLX_ID_EXT0 ||
	    sensor->id == ST_IIS2ICLX_ID_EXT1)
		return st_iis2iclx_shub_set_enable(sensor, enable);

	return st_iis2iclx_sensor_set_enable(sensor, enable);
}

static int st_iis2iclx_buffer_enable(struct iio_dev *iio_dev, bool enable)
{
	struct st_iis2iclx_sensor *sensor = iio_priv(iio_dev);

	if (sensor->hw->has_hw_fifo)
		return st_iis2iclx_update_fifo(sensor, enable);

	return st_iis2iclx_update_enable(sensor, enable);
}

static irqreturn_t st_iis2iclx_handler_irq(int irq, void *private)
{
	struct st_iis2iclx_hw *hw = (struct st_iis2iclx_hw *)private;
	s64 timestamp = st_iis2iclx_get_time_ns(hw->iio_devs[0]);

	hw->delta_ts = timestamp - hw->irq_ts;
	hw->irq_ts = timestamp;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_iis2iclx_handler_thread(int irq, void *private)
{
	struct st_iis2iclx_hw *hw = (struct st_iis2iclx_hw *)private;

	st_iis2iclx_mlc_check_status(hw);

	mutex_lock(&hw->fifo_lock);
	st_iis2iclx_read_fifo(hw);
	clear_bit(ST_IIS2ICLX_HW_FLUSH, &hw->state);
	mutex_unlock(&hw->fifo_lock);

	return st_iis2iclx_event_handler(hw);
}

static int st_iis2iclx_fifo_preenable(struct iio_dev *iio_dev)
{
	return st_iis2iclx_buffer_enable(iio_dev, true);
}

static int st_iis2iclx_fifo_postdisable(struct iio_dev *iio_dev)
{
	return st_iis2iclx_buffer_enable(iio_dev, false);
}

static const struct iio_buffer_setup_ops st_iis2iclx_buffer_setup_ops = {
	.preenable = st_iis2iclx_fifo_preenable,

#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
	.postenable = iio_triggered_buffer_postenable,
	.predisable = iio_triggered_buffer_predisable,
#endif /* LINUX_VERSION_CODE */

	.postdisable = st_iis2iclx_fifo_postdisable,
};

static irqreturn_t st_iis2iclx_buffer_pollfunc(int irq, void *private)
{
	u8 iio_buf[ALIGN(ST_IIS2ICLX_SAMPLE_SIZE, sizeof(s64)) +
		   sizeof(s64) + sizeof(s64)];
	struct iio_poll_func *pf = private;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct st_iis2iclx_sensor *sensor = iio_priv(indio_dev);
	struct st_iis2iclx_hw *hw = sensor->hw;
	int addr = indio_dev->channels[0].address;

	switch (indio_dev->channels[0].type) {
	case IIO_ACCEL:
	case IIO_ANGL_VEL:
		st_iis2iclx_read_locked(hw, addr, &iio_buf,
					ST_IIS2ICLX_SAMPLE_SIZE);
		break;
	case IIO_TEMP:
		st_iis2iclx_read_locked(hw, addr, &iio_buf,
					ST_IIS2ICLX_PT_SAMPLE_SIZE);
		break;
	case IIO_PRESSURE:
		st_iis2iclx_shub_read(sensor, addr, (u8 *)&iio_buf,
				      ST_IIS2ICLX_PT_SAMPLE_SIZE);
		break;
	case IIO_MAGN:
		st_iis2iclx_shub_read(sensor, addr, (u8 *)&iio_buf,
				      ST_IIS2ICLX_SAMPLE_SIZE);
		break;
	default:
		return -EINVAL;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, iio_buf,
					   st_iis2iclx_get_time_ns(indio_dev));
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int st_iis2iclx_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct st_iis2iclx_hw *hw = iio_trigger_get_drvdata(trig);

	dev_dbg(hw->dev, "trigger set %d\n", state);

	return 0;
}

static const struct iio_trigger_ops st_iis2iclx_trigger_ops = {
	.set_trigger_state = st_iis2iclx_trig_set_state,
 };

static int st_iis2iclx_config_interrupt(struct st_iis2iclx_hw *hw,
					bool enable)
{
	int err;

	err = st_iis2iclx_get_int_reg(hw);
	if (err < 0)
		return err;

	/* latch interrupts */
	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_TAP_CFG0_ADDR,
				 ST_IIS2ICLX_LIR_MASK,
				 FIELD_PREP(ST_IIS2ICLX_LIR_MASK,
					    enable ? 1 : 0));
	if (err < 0)
		return err;

	/* enable FIFO watermak interrupt */
	return regmap_update_bits(hw->regmap, hw->drdy_reg,
				  ST_IIS2ICLX_INT_FIFO_TH_MASK,
				  FIELD_PREP(ST_IIS2ICLX_INT_FIFO_TH_MASK,
					     enable ? 1 : 0));
}

static int st_iis2iclx_config_timestamp(struct st_iis2iclx_hw *hw)
{
	int err;

	err = st_iis2iclx_hwtimesync_init(hw);
	if (err)
		return err;

	/* init timestamp engine */
	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_CTRL10_C_ADDR,
				 ST_IIS2ICLX_TIMESTAMP_EN_MASK,
				 ST_IIS2ICLX_SHIFT_VAL(1,
					    ST_IIS2ICLX_TIMESTAMP_EN_MASK));
	if (err < 0)
		return err;

	return regmap_update_bits(hw->regmap,
				  ST_IIS2ICLX_REG_FIFO_CTRL4_ADDR,
				  ST_IIS2ICLX_DEC_TS_MASK,
				  FIELD_PREP(ST_IIS2ICLX_DEC_TS_MASK, 1));
}

int st_iis2iclx_allocate_buffers(struct st_iis2iclx_hw *hw)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_iis2iclx_triggered_main_sensor_list);
	     i++) {
		enum st_iis2iclx_sensor_id id;
		int err;

		id = st_iis2iclx_triggered_main_sensor_list[i];
		if (!hw->iio_devs[id])
			continue;

		err = devm_iio_triggered_buffer_setup(hw->dev, hw->iio_devs[id],
						 NULL,
						 st_iis2iclx_buffer_pollfunc,
						 &st_iis2iclx_buffer_setup_ops);
		if (err)
			return err;
	}

	return 0;
}

int st_iis2iclx_trigger_setup(struct st_iis2iclx_hw *hw)
{
	unsigned long irq_type;
	bool irq_active_low;
	int i, err;

	irq_type = irqd_get_trigger_type(irq_get_irq_data(hw->irq));
	if (irq_type == IRQF_TRIGGER_NONE)
		irq_type = IRQF_TRIGGER_HIGH;

	switch (irq_type) {
	case IRQF_TRIGGER_HIGH:
		irq_active_low = false;
		break;
	case IRQF_TRIGGER_LOW:
		irq_active_low = true;
		break;
	default:
		dev_info(hw->dev, "mode %lx unsupported\n", irq_type);
		return -EINVAL;
	}

	err = regmap_update_bits(hw->regmap,
				 ST_IIS2ICLX_REG_CTRL3_C_ADDR,
				 ST_IIS2ICLX_H_LACTIVE_MASK,
				 FIELD_PREP(ST_IIS2ICLX_H_LACTIVE_MASK,
					    irq_active_low));
	if (err < 0)
		return err;

	if (device_property_read_bool(hw->dev, "drive-open-drain")) {
		err = regmap_update_bits(hw->regmap,
					 ST_IIS2ICLX_REG_CTRL3_C_ADDR,
					 ST_IIS2ICLX_PP_OD_MASK,
					 FIELD_PREP(ST_IIS2ICLX_PP_OD_MASK, 1));
		if (err < 0)
			return err;

		irq_type |= IRQF_SHARED;
	}

	err = devm_request_threaded_irq(hw->dev, hw->irq,
					st_iis2iclx_handler_irq,
					st_iis2iclx_handler_thread,
					irq_type | IRQF_ONESHOT,
					ST_IIS2ICLX_DEV_NAME, hw);
	if (err) {
		dev_err(hw->dev, "failed to request trigger irq %d\n",
			hw->irq);
		return err;
	}

	/* attach trigger to iio devs */
	for (i = 0; i < ARRAY_SIZE(st_iis2iclx_triggered_main_sensor_list);
	     i++) {
		struct st_iis2iclx_sensor *sensor;
		enum st_iis2iclx_sensor_id id;

		id = st_iis2iclx_triggered_main_sensor_list[i];
		if (!hw->iio_devs[id])
			continue;

		sensor = iio_priv(hw->iio_devs[id]);
		sensor->trig = devm_iio_trigger_alloc(hw->dev, "st_%s-trigger",
						      hw->iio_devs[id]->name);
		if (!sensor->trig) {
			dev_err(hw->dev, "failed to allocate iio trigger.\n");

			return -ENOMEM;
		}

		iio_trigger_set_drvdata(sensor->trig, hw);
		sensor->trig->ops = &st_iis2iclx_trigger_ops;
		sensor->trig->dev.parent = hw->dev;

		err = devm_iio_trigger_register(hw->dev, sensor->trig);
		if (err < 0) {
			dev_err(hw->dev, "failed to register iio trigger.\n");

			return err;
		}

		hw->iio_devs[id]->trig = iio_trigger_get(sensor->trig);
	}

	err = st_iis2iclx_config_interrupt(hw, true);
	if (err < 0)
		return err;

	return st_iis2iclx_config_timestamp(hw);
}
