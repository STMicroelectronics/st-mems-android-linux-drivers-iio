// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22df buffer driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2021 STMicroelectronics Inc.
 */

#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/events.h>
#include <linux/version.h>

#include "st_lps22df.h"

#define ST_LPS22DF_EWMA_LEVEL			96
#define ST_LPS22DF_EWMA_DIV			128

static inline s64 st_lps22df_ewma(s64 old, s64 new, int weight)
{
	s64 diff, incr;

	diff = new - old;
	incr = div_s64((ST_LPS22DF_EWMA_DIV - weight) * diff,
		       ST_LPS22DF_EWMA_DIV);

	return old + incr;
}

static inline s64 st_lps22df_get_time_ns(struct st_lps22df_hw *hw)
{
	return iio_get_time_ns(hw->iio_devs[ST_LPS22DF_PRESS]);
}

static int st_lps22df_set_fifo_mode(struct st_lps22df_hw *hw,
				    enum st_lps22df_fifo_mode mode)
{
	switch (mode) {
	case ST_LPS22DF_BYPASS:
	case ST_LPS22DF_STREAM:
		break;
	default:
		return -EINVAL;
	}

	return st_lps22df_write_with_mask(hw, ST_LPS22DF_FIFO_CTRL_ADDR,
					  ST_LPS22DF_FIFO_MODE_MASK, mode);
}

static int st_lps22df_update_fifo_watermark(struct st_lps22df_hw *hw, u8 val)
{
	int err;

	err = st_lps22df_write_with_mask(hw, ST_LPS22DF_FIFO_WTM_ADDR,
					 ST_LPS22DF_FIFO_THS_MASK, val);
	if (err < 0)
		return err;

	hw->watermark = val;

	return 0;
}

ssize_t st_lps22df_sysfs_set_hwfifo_watermark(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct st_lps22df_sensor *sensor = iio_priv(dev_to_iio_dev(dev));
	int err, watermark;

	err = kstrtoint(buf, 10, &watermark);
	if (err < 0)
		return err;

	if (watermark < 1 || watermark > ST_LPS22DF_MAX_FIFO_LENGTH)
		return -EINVAL;

	err = st_lps22df_update_fifo_watermark(sensor->hw, watermark);

	return err < 0 ? err : count;
}

static int st_lps22df_read_fifo(struct st_lps22df_hw *hw, s64 delta_ts)
{
	u8 iio_buff[ALIGN(sizeof(u32) + sizeof(s64), sizeof(s64))];
	u8 buff[ST_LPS22DF_RX_MAX_LENGTH];
	int err, i, read_len;
	__le16 fifo_status;

	err = regmap_bulk_read(hw->regmap, ST_LPS22DF_FIFO_STATUS1_ADDR,
			       (unsigned int *)&fifo_status,
			       sizeof(fifo_status));
	if (err < 0)
		return err;

	read_len = (le16_to_cpu(fifo_status) & ST_LPS22DF_FIFO_SRC_DIFF_MASK) *
		   ST_LPS22DF_PRESS_SAMPLE_LEN;
	if (!read_len)
		return 0;

	err = regmap_bulk_read(hw->regmap,
			       ST_LPS22DF_FIFO_DATA_OUT_PRESS_XL_ADDR,
			       (unsigned int *)&buff, read_len);
	if (err < 0)
		return err;

	for (i = 0; i < read_len; i += ST_LPS22DF_PRESS_SAMPLE_LEN) {
		memcpy(iio_buff, buff + i, ST_LPS22DF_PRESS_SAMPLE_LEN);
		iio_push_to_buffers_with_timestamp(
				hw->iio_devs[ST_LPS22DF_PRESS],
				iio_buff, hw->ts);
		hw->ts += delta_ts;
	}

	hw->last_fifo_ts = hw->ts;

	return read_len;
}

ssize_t st_lps22df_sysfs_flush_fifo(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct st_lps22df_sensor *sensor = iio_priv(indio_dev);
	struct st_lps22df_hw *hw = sensor->hw;
	enum iio_chan_type type;
	int len, dir;
	s64 fts;

	mutex_lock(&hw->fifo_lock);

	hw->ts = st_lps22df_get_time_ns(hw);
	hw->ts_irq = hw->ts;

	switch (sensor->type) {
	case ST_LPS22DF_PRESS:
		type = IIO_PRESSURE;
		len = st_lps22df_read_fifo(hw, hw->delta_ts);

		/*
		 * flush event timestamp must match with last sample pushed
		 * in fifo
		 */
		if (len)
			fts = hw->ts;
		else
			fts = hw->last_fifo_ts;

		break;
	case ST_LPS22DF_TEMP:
		type = IIO_TEMP;

		/* temperature not support FIFO */
		len = 0;
		fts = hw->ts;
		break;
	default:
		mutex_unlock(&hw->fifo_lock);
		return -EINVAL;
	}

	mutex_unlock(&hw->fifo_lock);

	dir = len > 0 ? STM_IIO_EV_DIR_FIFO_DATA : STM_IIO_EV_DIR_FIFO_EMPTY;
	iio_push_event(indio_dev,
		       IIO_UNMOD_EVENT_CODE(type, -1,
					    STM_IIO_EV_TYPE_FIFO_FLUSH, dir),
		       fts);

	return size;
}

static irqreturn_t st_lps22df_irq_handler(int irq, void *private)
{
	struct st_lps22df_hw *hw = private;
	s64 delta_ts, ts = st_lps22df_get_time_ns(hw);

	delta_ts = div_s64((ts - hw->ts_irq), hw->watermark);
	if (hw->odr >= 50)
		hw->delta_ts = st_lps22df_ewma(hw->delta_ts, delta_ts,
					       ST_LPS22DF_EWMA_LEVEL);
	else
		hw->delta_ts = delta_ts;

	hw->ts_irq = ts;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_lps22df_irq_thread(int irq, void *private)
{
	struct st_lps22df_hw *hw = private;

	mutex_lock(&hw->fifo_lock);
	st_lps22df_read_fifo(hw, hw->delta_ts);
	mutex_unlock(&hw->fifo_lock);

	return IRQ_HANDLED;
}

static int st_lps22df_buffer_preenable(struct iio_dev *indio_dev)
{
	struct st_lps22df_sensor *sensor = iio_priv(indio_dev);
	struct st_lps22df_hw *hw = sensor->hw;
	int err;

	switch (sensor->type) {
	case ST_LPS22DF_PRESS:
		err = st_lps22df_set_fifo_mode(sensor->hw, ST_LPS22DF_STREAM);
		if (err < 0)
			return err;

		err = st_lps22df_update_fifo_watermark(hw, hw->watermark);
		if (err < 0)
			return err;

		err = st_lps22df_write_with_mask(sensor->hw,
						 ST_LPS22DF_CTRL4_ADDR,
						 ST_LPS22DF_INT_F_WTM_MASK,
						 true);
		if (err < 0)
			return err;

		hw->delta_ts = div_s64(1000000000UL, hw->odr);
		hw->ts = st_lps22df_get_time_ns(hw);
		hw->ts_irq = hw->ts;
		break;
	case ST_LPS22DF_TEMP: {
			ktime_t ktime = ktime_set(0, 1000000000 / sensor->odr);

			hrtimer_start(&hw->hr_timer, ktime, HRTIMER_MODE_REL);
			hw->ktime = ktime;
		}
		break;
	default:
		return -EINVAL;
	}

	return st_lps22df_set_enable(sensor, true);
}

static int st_lps22df_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct st_lps22df_sensor *sensor = iio_priv(indio_dev);
	int err;

	switch (sensor->type) {
	case ST_LPS22DF_PRESS:
		err = st_lps22df_set_fifo_mode(sensor->hw, ST_LPS22DF_BYPASS);
		if (err < 0)
			return err;

		err = st_lps22df_write_with_mask(sensor->hw,
						 ST_LPS22DF_CTRL4_ADDR,
						 ST_LPS22DF_INT_F_WTM_MASK,
						 false);
		if (err < 0)
			return err;
		break;
	case ST_LPS22DF_TEMP:
		cancel_work_sync(&sensor->hw->iio_work);
		hrtimer_cancel(&sensor->hw->hr_timer);
		break;
	default:
		return -EINVAL;
	}

	return st_lps22df_set_enable(sensor, false);
}

static const struct iio_buffer_setup_ops st_lps22df_buffer_ops = {
	.preenable = st_lps22df_buffer_preenable,
	.postdisable = st_lps22df_buffer_postdisable,
};

static enum hrtimer_restart st_lps22df_poll_function_read(struct hrtimer *timer)
{
	struct st_lps22df_sensor *sensor;
	struct st_lps22df_hw *hw;

	hw = container_of((struct hrtimer *)timer,
			   struct st_lps22df_hw, hr_timer);

	sensor = iio_priv(hw->iio_devs[ST_LPS22DF_TEMP]);
	sensor->timestamp = st_lps22df_get_time_ns(hw);
	queue_work(hw->workqueue, &hw->iio_work);

	return HRTIMER_NORESTART;
}

static void st_lps22df_report_temp(struct st_lps22df_sensor *sensor,
				    u8 *tmp, int64_t timestamp)
{
	struct iio_dev *iio_dev = sensor->hw->iio_devs[ST_LPS22DF_TEMP];
	u8 iio_buf[ALIGN(2, sizeof(s64)) + sizeof(s64)];

	memcpy(iio_buf, tmp, 2);
	iio_push_to_buffers_with_timestamp(iio_dev, iio_buf, timestamp);
}

static void st_lps22df_poll_function_work(struct work_struct *iio_work)
{
	struct st_lps22df_sensor *sensor;
	struct st_lps22df_hw *hw;
	ktime_t tmpkt, ktdelta;
	u8 data[2];
	int err;

	hw = container_of((struct work_struct *)iio_work,
			   struct st_lps22df_hw, iio_work);
	sensor = iio_priv(hw->iio_devs[ST_LPS22DF_TEMP]);

	/* adjust delta time */
	ktdelta = ktime_set(0, st_lps22df_get_time_ns(hw) - sensor->timestamp);

	/* avoid negative value in case of high odr */
	mutex_lock(&hw->lock);
	if (ktime_after(hw->ktime, ktdelta))
		tmpkt = ktime_sub(hw->ktime, ktdelta);
	else
		tmpkt = hw->ktime;

	hrtimer_start(&hw->hr_timer, tmpkt, HRTIMER_MODE_REL);
	mutex_unlock(&hw->lock);

	err = regmap_bulk_read(hw->regmap,
			       hw->iio_devs[ST_LPS22DF_TEMP]->channels->address,
			       (unsigned int *)&data, sizeof(data));
	mutex_unlock(&hw->lock);

	if (err < 0)
		return;

	st_lps22df_report_temp(sensor, data, sensor->timestamp);
}

static inline void st_lps22df_flush_works(struct st_lps22df_hw *hw)
{
	flush_workqueue(hw->workqueue);
}

static inline int st_lps22df_destroy_workqueue(struct st_lps22df_hw *hw)
{
	if (hw->workqueue)
		destroy_workqueue(hw->workqueue);

	return 0;
}

static inline int st_lps22df_allocate_workqueue(struct st_lps22df_hw *hw)
{
	if (!hw->workqueue)
		hw->workqueue = create_workqueue(ST_LPS22DF_DEV_NAME);

	return !hw->workqueue ? -ENOMEM : 0;
}

int st_lps22df_allocate_buffers(struct st_lps22df_hw *hw)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,13,0)
	struct iio_buffer *buffer;
#endif /* LINUX_VERSION_CODE */
	unsigned long irq_type;
	u8 int_active = 0;
	int err, i;

	irq_type = irqd_get_trigger_type(irq_get_irq_data(hw->irq));

	switch (irq_type) {
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_RISING:
		int_active = 0;
		break;
	case IRQF_TRIGGER_LOW:
	case IRQF_TRIGGER_FALLING:
		int_active = 1;
		break;
	default:
		dev_info(hw->dev, "mode %lx unsupported\n", irq_type);
		return -EINVAL;
	}

	/* int pin active low */
	if (device_property_read_bool(hw->dev, "int-active-low")) {
		err = st_lps22df_write_with_mask(hw, ST_LPS22DF_CTRL3_ADDR,
						 ST_LPS22DF_INT_H_L_MASK, 1);
		if (err < 0)
			return err;
	}

	/* int pin open drain configuration */
	if (device_property_read_bool(hw->dev, "int-open-drain")) {
		err = st_lps22df_write_with_mask(hw, ST_LPS22DF_CTRL3_ADDR,
						 ST_LPS22DF_PP_OD_MASK, 1);
		if (err < 0)
			return err;
	}

	err = st_lps22df_write_with_mask(hw, ST_LPS22DF_CTRL3_ADDR,
					 ST_LPS22DF_INT_H_L_MASK,
					 int_active);
	if (err < 0)
		return err;

	err = devm_request_threaded_irq(hw->dev, hw->irq,
					st_lps22df_irq_handler,
					st_lps22df_irq_thread,
					irq_type | IRQF_ONESHOT,
					"lps22df", hw);
	if (err)
		return err;

	for (i = 0; i < ST_LPS22DF_SENSORS_NUMB; i++) {

#if KERNEL_VERSION(5, 19, 0) <= LINUX_VERSION_CODE
		err = devm_iio_kfifo_buffer_setup(hw->dev,
						  hw->iio_devs[i],
						  &st_lps22df_buffer_ops);
		if (err)
			return err;
#elif KERNEL_VERSION(5, 13, 0) <= LINUX_VERSION_CODE
		err = devm_iio_kfifo_buffer_setup(hw->dev,
						  hw->iio_devs[i],
						  INDIO_BUFFER_SOFTWARE,
						  &st_lps22df_buffer_ops);
		if (err)
			return err;
#else /* LINUX_VERSION_CODE */
		buffer = devm_iio_kfifo_allocate(hw->dev);
		if (!buffer)
			return -ENOMEM;

		iio_device_attach_buffer(hw->iio_devs[i], buffer);
		hw->iio_devs[i]->modes |= INDIO_BUFFER_SOFTWARE;
		hw->iio_devs[i]->setup_ops = &st_lps22df_buffer_ops;
#endif /* LINUX_VERSION_CODE */

	}

	/* configure sensor hrtimer for temperature */
	err = st_lps22df_allocate_workqueue(hw);
	if (err)
		return err;

	hrtimer_init(&hw->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hw->hr_timer.function = &st_lps22df_poll_function_read;
	INIT_WORK(&hw->iio_work, st_lps22df_poll_function_work);

	return 0;
}

int st_lps22df_deallocate_buffers(struct st_lps22df_hw *hw)
{
	st_lps22df_flush_works(hw);
	st_lps22df_destroy_workqueue(hw);

	return 0;
}
