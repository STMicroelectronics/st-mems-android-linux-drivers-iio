/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * STMicroelectronics Kernel affected version header file
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2026 STMicroelectronics Inc.
 */

#ifndef __STM_IIO_LINUX_COMPAT_H__
#define __STM_IIO_LINUX_COMPAT_H__

#include <linux/hrtimer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/version.h>

#if KERNEL_VERSION(5, 13, 0) > LINUX_VERSION_CODE
#include <linux/iio/buffer.h>
#endif /* LINUX_VERSION_CODE */

#if KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE
#include <linux/iio/buffer_impl.h>
#endif /* LINUX_VERSION_CODE */

#if KERNEL_VERSION(6, 11, 0) < LINUX_VERSION_CODE
#include <linux/unaligned.h>
#else /* LINUX_VERSION_CODE */
#include <asm/unaligned.h>
#endif /* LINUX_VERSION_CODE */

#if KERNEL_VERSION(5, 19, 0) <= LINUX_VERSION_CODE
#include <linux/iio/iio-opaque.h>
#endif /* LINUX_VERSION_CODE */

#if KERNEL_VERSION(6, 13, 0) <= LINUX_VERSION_CODE
#define ST_IIO_EVENT_EN_TYPE bool
#else /* LINUX_VERSION_CODE */
#define ST_IIO_EVENT_EN_TYPE int
#endif /* LINUX_VERSION_CODE */

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
#define ST_I2C_PROBE(_name) static int _name(struct i2c_client *client)
#define ST_I2C_GET_PROBE_ID(_client, _id) \
	const struct i2c_device_id *_id = i2c_client_get_device_id(_client)
#else /* LINUX_VERSION_CODE */
#define ST_I2C_PROBE(_name) \
	static int _name(struct i2c_client *client, \
			 const struct i2c_device_id *id)
#define ST_I2C_GET_PROBE_ID(_client, _id) \
	const struct i2c_device_id *_id = id
#endif /* LINUX_VERSION_CODE */

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
#define ST_I2C_REMOVE(_name, _remove_cb) \
	static void _name(struct i2c_client *client) \
	{ \
		_remove_cb(&client->dev); \
	}
#else /* LINUX_VERSION_CODE */
#define ST_I2C_REMOVE(_name, _remove_cb) \
	static int _name(struct i2c_client *client) \
	{ \
		_remove_cb(&client->dev); \
		return 0; \
	}
#endif /* LINUX_VERSION_CODE */

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
#define ST_SPI_REMOVE(_name, _remove_cb) \
	static void _name(struct spi_device *spi) \
	{ \
		_remove_cb(&spi->dev); \
	}
#else /* LINUX_VERSION_CODE */
#define ST_SPI_REMOVE(_name, _remove_cb) \
	static int _name(struct spi_device *spi) \
	{ \
		_remove_cb(&spi->dev); \
		return 0; \
	}
#endif /* LINUX_VERSION_CODE */

#if KERNEL_VERSION(5, 12, 0) <= LINUX_VERSION_CODE
#define ST_I3C_REMOVE(_name, _remove_cb) \
	static void _name(struct i3c_device *i3cdev) \
	{ \
		_remove_cb(&i3cdev->dev); \
		i3c_device_disable_ibi(i3cdev); \
		i3c_device_free_ibi(i3cdev); \
	}
#else /* LINUX_VERSION_CODE */
#define ST_I3C_REMOVE(_name, _remove_cb) \
	static int _name(struct i3c_device *i3cdev) \
	{ \
		_remove_cb(&i3cdev->dev); \
		i3c_device_disable_ibi(i3cdev); \
		i3c_device_free_ibi(i3cdev); \
		return 0; \
	}
#endif /* LINUX_VERSION_CODE */

static inline void st_hrtimer_setup(struct hrtimer *timer,
		enum hrtimer_restart (*function)(struct hrtimer *))
{
#if KERNEL_VERSION(6, 13, 0) <= LINUX_VERSION_CODE
	hrtimer_setup(timer, function, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
#else /* LINUX_VERSION_CODE */
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->function = function;
#endif /* LINUX_VERSION_CODE */
}

static inline void st_iio_trigger_poll(struct iio_trigger *trig)
{
#if KERNEL_VERSION(6, 4, 0) <= LINUX_VERSION_CODE
	iio_trigger_poll_nested(trig);
#else /* LINUX_VERSION_CODE */
	iio_trigger_poll_chained(trig);
#endif /* LINUX_VERSION_CODE */
}

static inline void st_iio_get_timesync_event_codes(long long *event_lsb,
						    long long *event_msb,
						    int chan_type,
						    int ev_type)
{
#if KERNEL_VERSION(6, 13, 0) <= LINUX_VERSION_CODE
	*event_lsb = IIO_UNMOD_EVENT_CODE(chan_type, 0, ev_type, 0);
	*event_msb = IIO_UNMOD_EVENT_CODE(chan_type, 0, ev_type, 1);
#else /* LINUX_VERSION_CODE */
	*event_lsb = IIO_EVENT_CODE(chan_type, 0, 0, 0,
				   ev_type, 0, 0, 0);
	*event_msb = IIO_EVENT_CODE(chan_type, 0, 0, 1,
				   ev_type, 0, 0, 0);
#endif /* LINUX_VERSION_CODE */
}

#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
#define ST_IIO_TRIGGERED_OLD_SETUP_OPS \
	.postenable = iio_triggered_buffer_postenable, \
	.predisable = iio_triggered_buffer_predisable,
#else /* LINUX_VERSION_CODE */
#define ST_IIO_TRIGGERED_OLD_SETUP_OPS
#endif /* LINUX_VERSION_CODE */

static inline int st_iio_read_mount_matrix(struct device *dev,
					   struct iio_mount_matrix *orientation)
{
#if KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE
	return iio_read_mount_matrix(dev, orientation);
#elif KERNEL_VERSION(5, 2, 0) <= LINUX_VERSION_CODE
	return iio_read_mount_matrix(dev, "mount-matrix", orientation);
#else /* LINUX_VERSION_CODE */
	return of_iio_read_mount_matrix(dev, "mount-matrix", orientation);
#endif /* LINUX_VERSION_CODE */
}

static inline int st_iio_device_claim_direct(struct iio_dev *indio_dev)
{
#if KERNEL_VERSION(6, 13, 0) <= LINUX_VERSION_CODE
	bool ret;

	ret = iio_device_claim_direct(indio_dev);

	return ret == true ? 0 : -EBUSY;
#else /* LINUX_VERSION_CODE */
	return iio_device_claim_direct_mode(indio_dev);
#endif /* LINUX_VERSION_CODE */
}

static inline struct iio_dev *st_iio_device_alloc(int sizeof_priv)
{
#if KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE
	return iio_device_alloc(0, sizeof_priv);
#else /* LINUX_VERSION_CODE */
	return iio_device_alloc(sizeof_priv);
#endif /* LINUX_VERSION_CODE */
}

static inline void st_iio_device_release_direct(struct iio_dev *indio_dev)
{
#if KERNEL_VERSION(6, 13, 0) <= LINUX_VERSION_CODE
	iio_device_release_direct(indio_dev);
#else /* LINUX_VERSION_CODE */
	iio_device_release_direct_mode(indio_dev);
#endif /* LINUX_VERSION_CODE */
}

static inline int st_devm_iio_kfifo_buffer_setup(struct device *dev,
				struct iio_dev *indio_dev,
				const struct iio_buffer_setup_ops *setup_ops)
{
	int err = 0;

#if KERNEL_VERSION(5, 19, 0) <= LINUX_VERSION_CODE
	err = devm_iio_kfifo_buffer_setup(dev, indio_dev, setup_ops);
	if (err)
		return err;
#elif KERNEL_VERSION(5, 13, 0) <= LINUX_VERSION_CODE
	err = devm_iio_kfifo_buffer_setup(dev,
					  indio_dev,
					  INDIO_BUFFER_SOFTWARE,
					  setup_ops);
	if (err)
		return err;
#else /* LINUX_VERSION_CODE */
	struct iio_buffer *buffer;

	buffer = devm_iio_kfifo_allocate(dev);
	if (!buffer)
		return -ENOMEM;

	indio_dev->modes |= INDIO_BUFFER_SOFTWARE;
	indio_dev->setup_ops = setup_ops;

	iio_device_attach_buffer(indio_dev, buffer);
#endif /* LINUX_VERSION_CODE */

	return err;
}
#endif /* __STM_IIO_LINUX_COMPAT_H__ */
