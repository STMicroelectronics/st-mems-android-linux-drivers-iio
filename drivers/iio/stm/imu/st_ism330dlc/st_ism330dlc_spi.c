// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics ism330dlc spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/iio/iio.h>

#include "st_ism330dlc.h"

#define ST_SENSORS_SPI_READ			0x80

static int st_ism330dlc_spi_read(struct ism330dlc_data *cdata,
				 u8 reg_addr, int len, u8 *data, bool b_lock)
{
	int err;

	struct spi_transfer xfers[] = {
		{
			.tx_buf = cdata->tb.tx_buf,
			.bits_per_word = 8,
			.len = 1,
		},
		{
			.rx_buf = cdata->tb.rx_buf,
			.bits_per_word = 8,
			.len = len,
		}
	};

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr | ST_SENSORS_SPI_READ;

	err = spi_sync_transfer(to_spi_device(cdata->dev),
				xfers, ARRAY_SIZE(xfers));
	if (err)
		goto acc_spi_read_error;

	memcpy(data, cdata->tb.rx_buf, len*sizeof(u8));
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static int st_ism330dlc_spi_write(struct ism330dlc_data *cdata,
				u8 reg_addr, int len, u8 *data, bool b_lock)
{
	int err;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= ST_ISM330DLC_TX_MAX_LENGTH)
		return -ENOMEM;

	if (b_lock)
		mutex_lock(&cdata->bank_registers_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	err = spi_sync_transfer(to_spi_device(cdata->dev), &xfers, 1);
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->bank_registers_lock);

	return err;
}

static const struct st_ism330dlc_transfer_function st_ism330dlc_tf_spi = {
	.write = st_ism330dlc_spi_write,
	.read = st_ism330dlc_spi_read,
};

static int st_ism330dlc_spi_probe(struct spi_device *spi)
{
	int err;
	struct ism330dlc_data *cdata;

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	spi_set_drvdata(spi, cdata);

	cdata->tf = &st_ism330dlc_tf_spi;

	err = st_ism330dlc_common_probe(cdata, spi->irq);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void st_ism330dlc_spi_remove(struct spi_device *spi)
{
	struct ism330dlc_data *cdata = spi_get_drvdata(spi);

	st_ism330dlc_common_remove(cdata, spi->irq);
	kfree(cdata);
}
#else /* LINUX_VERSION_CODE */
static int st_ism330dlc_spi_remove(struct spi_device *spi)
{
	struct ism330dlc_data *cdata = spi_get_drvdata(spi);

	st_ism330dlc_common_remove(cdata, spi->irq);
	kfree(cdata);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

#ifdef CONFIG_PM
static int __maybe_unused st_ism330dlc_suspend(struct device *dev)
{
	struct ism330dlc_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return st_ism330dlc_common_suspend(cdata);
}

static int __maybe_unused st_ism330dlc_resume(struct device *dev)
{
	struct ism330dlc_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return st_ism330dlc_common_resume(cdata);
}

static const struct dev_pm_ops st_ism330dlc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_ism330dlc_suspend, st_ism330dlc_resume)
};

#define ST_ISM330DLC_PM_OPS		(&st_ism330dlc_pm_ops)
#else /* CONFIG_PM */
#define ST_ISM330DLC_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id st_ism330dlc_id_table[] = {
	{ ISM330DLC_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_ism330dlc_id_table);

#ifdef CONFIG_OF
static const struct of_device_id ism330dlc_of_match[] = {
	{
		.compatible = "st,ism330dlc",
		.data = ISM330DLC_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ism330dlc_of_match);
#else /* CONFIG_OF */
#define ism330dlc_of_match		NULL
#endif /* CONFIG_OF */

static struct spi_driver st_ism330dlc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-ism330dlc-spi",
		.pm = ST_ISM330DLC_PM_OPS,
		.of_match_table = of_match_ptr(ism330dlc_of_match),
	},
	.probe = st_ism330dlc_spi_probe,
	.remove = st_ism330dlc_spi_remove,
	.id_table = st_ism330dlc_id_table,
};
module_spi_driver(st_ism330dlc_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics ism330dlc spi driver");
MODULE_LICENSE("GPL v2");
