// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2dw12 fifo driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/version.h>

#include "st_lis2dw12.h"

#define ST_LIS2DW12_EWMA_LEVEL			120
#define ST_LIS2DW12_EWMA_DIV			128
static inline s64 st_lis2dw12_ewma(s64 old, s64 new, int weight)
{
	s64 diff, incr;

	diff = new - old;
	incr = div_s64((ST_LIS2DW12_EWMA_DIV - weight) * diff,
		       ST_LIS2DW12_EWMA_DIV);

	return old + incr;
}

static int st_lis2dw12_update_fifo(struct iio_dev *iio_dev, bool enable)
{
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;
	enum st_lis2dw12_fifo_mode mode;
	int err;

	if (enable) {
		hw->ts_irq = hw->ts = st_lis2dw12_get_timestamp(hw);
		hw->delta_ts = div_s64(1000000000LL, sensor->odr) *
			       hw->watermark;
		hw->samples = 0;
	}

	/* enable FIFO only for acc */
	if (sensor->id == ST_LIS2DW12_ID_ACC) {
		mode = enable ? ST_LIS2DW12_FIFO_CONTINUOUS :
				ST_LIS2DW12_FIFO_BYPASS;
		err = st_lis2dw12_write_with_mask_locked(hw,
						     ST_LIS2DW12_FIFO_CTRL_ADDR,
						     ST_LIS2DW12_FIFOMODE_MASK,
						     mode);
		if (err < 0)
			return err;
	}

	return st_lis2dw12_sensor_set_enable(sensor, enable);
}

int st_lis2dw12_update_fifo_watermark(struct st_lis2dw12_hw *hw, u8 watermark)
{
	return st_lis2dw12_write_with_mask_locked(hw,
						  ST_LIS2DW12_FIFO_CTRL_ADDR,
						  ST_LIS2DW12_FTH_MASK,
						  watermark);
}

ssize_t st_lis2dw12_set_hwfifo_watermark(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;
	int err, val;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	if (iio_buffer_enabled(iio_dev)) {
		err = -EBUSY;
		goto unlock;
	}

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		goto unlock;

	if (val < 1 || val > ST_LIS2DW12_MAX_WATERMARK) {
		err = -EINVAL;
		goto unlock;
	}

	err = st_lis2dw12_update_fifo_watermark(hw, val);
	if (err < 0)
		goto unlock;

	hw->watermark = val;

unlock:
	iio_device_release_direct_mode(iio_dev);

	return err < 0 ? err : size;
}

static int st_lis2dw12_buffer_preenable(struct iio_dev *iio_dev)
{
	return st_lis2dw12_update_fifo(iio_dev, true);
}

static int st_lis2dw12_buffer_postdisable(struct iio_dev *iio_dev)
{
	return st_lis2dw12_update_fifo(iio_dev, false);
}

static const struct iio_buffer_setup_ops st_lis2dw12_acc_buffer_setup_ops = {
	.preenable = st_lis2dw12_buffer_preenable,
	.postdisable = st_lis2dw12_buffer_postdisable,
};

static int st_lis2dw12_read_fifo(struct st_lis2dw12_hw *hw)
{
	u8 iio_buff[ALIGN(ST_LIS2DW12_DATA_SIZE, sizeof(s64)) + sizeof(s64)];
	u8 buff[6 * ST_LIS2DW12_DATA_SIZE], status, samples;
	struct iio_dev *iio_dev = hw->iio_devs[ST_LIS2DW12_ID_ACC];
	struct iio_chan_spec const *ch = iio_dev->channels;
	int i, err, word_len, fifo_len, read_len = 0;
	s64 delta_ts;
	u8 ovr;

	err = st_lis2dw12_read(hw, ST_LIS2DW12_FIFO_SAMPLES_ADDR,
			       &status, sizeof(status));
	if (err < 0)
		return err;

	samples = status & ST_LIS2DW12_FIFO_SAMPLES_DIFF_MASK;
	ovr = status & ST_LIS2DW12_FIFO_SAMPLES_OVR_MASK;
	delta_ts = div_s64(hw->delta_ts, hw->watermark);

	/* if overrun flush all */
	if (!ovr)
		fifo_len = samples * ST_LIS2DW12_DATA_SIZE;
	else
		fifo_len = ST_LIS2DW12_FIFO_SAMPLES_DIFF_MASK *
			   ST_LIS2DW12_DATA_SIZE;

	while (read_len < fifo_len) {
		word_len = min_t(int, fifo_len - read_len, sizeof(buff));
		err = st_lis2dw12_read(hw, ch[0].address, buff, word_len);
		if (err < 0)
			return err;

		for (i = 0; i < word_len; i += ST_LIS2DW12_DATA_SIZE) {
			if (unlikely(++hw->samples < hw->std_level)) {
				hw->ts += delta_ts;
				continue;
			}

			hw->ts = min_t(s64, st_lis2dw12_get_timestamp(hw),
				       hw->ts);
			memcpy(iio_buff, &buff[i], ST_LIS2DW12_DATA_SIZE);
			iio_push_to_buffers_with_timestamp(iio_dev, iio_buff,
							   hw->ts);
			hw->ts += delta_ts;
		}
		read_len += word_len;
	}

	return read_len;
}

ssize_t st_lis2dw12_flush_fifo(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_to_iio_dev(dev);
	struct st_lis2dw12_sensor *sensor = iio_priv(iio_dev);
	struct st_lis2dw12_hw *hw = sensor->hw;
	s64 code;
	int err;

	mutex_lock(&hw->fifo_lock);

	err = st_lis2dw12_read_fifo(hw);
	hw->ts_irq = st_lis2dw12_get_timestamp(hw);

	mutex_unlock(&hw->fifo_lock);

	code = IIO_UNMOD_EVENT_CODE(IIO_ACCEL, -1,
				    STM_IIO_EV_TYPE_FIFO_FLUSH,
				    IIO_EV_DIR_EITHER);
	iio_push_event(iio_dev, code, hw->ts_irq);

	return err < 0 ? err : size;
}

int __maybe_unused
st_lis2dw12_flush_fifo_during_resume(struct st_lis2dw12_hw *hw)
{
	int count;

	mutex_lock(&hw->fifo_lock);
	count = st_lis2dw12_read_fifo(hw);
	mutex_unlock(&hw->fifo_lock);

	return count;
}

int st_lis2dw12_suspend_fifo(struct st_lis2dw12_hw *hw)
{
	int err;

	mutex_lock(&hw->fifo_lock);
	st_lis2dw12_read_fifo(hw);
	err = st_lis2dw12_set_fifo_mode(hw, ST_LIS2DW12_FIFO_BYPASS);
	mutex_unlock(&hw->fifo_lock);

	return err;
}

static irqreturn_t st_lis2dw12_handler_irq(int irq, void *private)
{
	struct st_lis2dw12_hw *hw = private;
	s64 ts;

	ts = st_lis2dw12_get_timestamp(hw);
	hw->delta_ts = st_lis2dw12_ewma(hw->delta_ts, ts - hw->ts_irq,
					ST_LIS2DW12_EWMA_LEVEL);
	hw->ts_irq = ts;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_lis2dw12_handler_thread(int irq, void *private)
{
	struct st_lis2dw12_hw *hw = private;
	u8 status;
	int err;

	err = st_lis2dw12_read(hw, ST_LIS2DW12_STATUS_ADDR,
			       &status, sizeof(status));
	if (err < 0)
		return IRQ_HANDLED;

	if (status & ST_LIS2DW12_STATUS_FTH_MASK) {
		mutex_lock(&hw->fifo_lock);
		st_lis2dw12_read_fifo(hw);
		mutex_unlock(&hw->fifo_lock);
	}

#ifdef CONFIG_IIO_ST_LIS2DW12_EN_BASIC_FEATURES
	if (hw->irq_emb > 0)
		return IRQ_HANDLED;

	err = st_lis2dw12_emb_event(hw);
#endif /* CONFIG_IIO_ST_LIS2DW12_EN_BASIC_FEATURES */

	return IRQ_HANDLED;
}

int st_lis2dw12_fifo_setup(struct st_lis2dw12_hw *hw)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,13,0)
	struct iio_buffer *buffer;
#endif /* LINUX_VERSION_CODE */

	int ret, i;

	ret = devm_request_threaded_irq(hw->dev, hw->irq,
					st_lis2dw12_handler_irq,
					st_lis2dw12_handler_thread,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					hw->name, hw);
	if (ret) {
		dev_err(hw->dev, "failed to request trigger irq %d\n",
			hw->irq);
		return ret;
	}

	for (i = ST_LIS2DW12_ID_ACC; i <= ST_LIS2DW12_ID_TEMP; i++) {
		if (!hw->iio_devs[i])
			continue;

#if KERNEL_VERSION(5, 19, 0) <= LINUX_VERSION_CODE
		ret = devm_iio_kfifo_buffer_setup(hw->dev, hw->iio_devs[i],
					     &st_lis2dw12_acc_buffer_setup_ops);
		if (ret)
			return ret;
#elif KERNEL_VERSION(5, 13, 0) <= LINUX_VERSION_CODE
		ret = devm_iio_kfifo_buffer_setup(hw->dev, hw->iio_devs[i],
					     INDIO_BUFFER_SOFTWARE,
					     &st_lis2dw12_acc_buffer_setup_ops);
		if (ret)
			return ret;
#else /* LINUX_VERSION_CODE */
		buffer = devm_iio_kfifo_allocate(hw->dev);
		if (!buffer)
			return -ENOMEM;

		iio_device_attach_buffer(hw->iio_devs[i], buffer);
		hw->iio_devs[i]->setup_ops = &st_lis2dw12_acc_buffer_setup_ops;
		hw->iio_devs[i]->modes |= INDIO_BUFFER_SOFTWARE;
#endif /* LINUX_VERSION_CODE */

	}

	return 0;
}
