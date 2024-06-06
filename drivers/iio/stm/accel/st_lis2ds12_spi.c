// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2ds12 spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "st_lis2ds12.h"

#define ST_SENSORS_SPI_READ			0x80

static int lis2ds12_spi_read(struct lis2ds12_data *cdata,
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
		mutex_lock(&cdata->regs_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr | ST_SENSORS_SPI_READ;

	err = spi_sync_transfer(to_spi_device(cdata->dev),
				xfers, ARRAY_SIZE(xfers));
	if (err)
		goto acc_spi_read_error;

	memcpy(data, cdata->tb.rx_buf, len*sizeof(u8));
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->regs_lock);

	return len;

acc_spi_read_error:
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->regs_lock);

	return err;
}

static int lis2ds12_spi_write(struct lis2ds12_data *cdata,
			      u8 reg_addr, int len, u8 *data, bool b_lock)
{
	int err;

	struct spi_transfer xfers = {
		.tx_buf = cdata->tb.tx_buf,
		.bits_per_word = 8,
		.len = len + 1,
	};

	if (len >= LIS2DS12_TX_MAX_LENGTH)
		return -ENOMEM;

	if (b_lock)
		mutex_lock(&cdata->regs_lock);

	mutex_lock(&cdata->tb.buf_lock);
	cdata->tb.tx_buf[0] = reg_addr;

	memcpy(&cdata->tb.tx_buf[1], data, len);

	err = spi_sync_transfer(to_spi_device(cdata->dev), &xfers, 1);
	mutex_unlock(&cdata->tb.buf_lock);
	if (b_lock)
		mutex_unlock(&cdata->regs_lock);

	return err;
}

static const struct lis2ds12_transfer_function lis2ds12_tf_spi = {
	.write = lis2ds12_spi_write,
	.read = lis2ds12_spi_read,
};

static int lis2ds12_spi_probe(struct spi_device *spi)
{
	struct lis2ds12_data *cdata;

	cdata = devm_kzalloc(&spi->dev, sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->tf = &lis2ds12_tf_spi;
	spi_set_drvdata(spi, cdata);

	return lis2ds12_common_probe(cdata, spi->irq);
}

#ifdef CONFIG_PM
static int __maybe_unused lis2ds12_suspend(struct device *dev)
{
	struct lis2ds12_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lis2ds12_common_suspend(cdata);
}

static int __maybe_unused lis2ds12_resume(struct device *dev)
{
	struct lis2ds12_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lis2ds12_common_resume(cdata);
}

static const struct dev_pm_ops lis2ds12_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis2ds12_suspend, lis2ds12_resume)
};

#define LIS2DS12_PM_OPS		(&lis2ds12_pm_ops)
#else /* CONFIG_PM */
#define LIS2DS12_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id lis2ds12_ids[] = {
	{"lis2ds12", 0},
	{"lsm303ah", 0},
	{"lis2dg", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, lis2ds12_ids);

#ifdef CONFIG_OF
static const struct of_device_id lis2ds12_id_table[] = {
	{.compatible = "st,lis2ds12",},
	{.compatible = "st,lsm303ah",},
	{.compatible = "st,lis2dg",},
	{},
};

MODULE_DEVICE_TABLE(of, lis2ds12_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lis2ds12_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LIS2DS12_DEV_NAME,
		   .pm = LIS2DS12_PM_OPS,
#ifdef CONFIG_OF
		   .of_match_table = lis2ds12_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = lis2ds12_spi_probe,
	.id_table = lis2ds12_ids,
};

module_spi_driver(lis2ds12_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2ds12 spi driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
