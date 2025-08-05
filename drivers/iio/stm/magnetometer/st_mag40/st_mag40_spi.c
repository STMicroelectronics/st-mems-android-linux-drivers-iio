// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_mag40 driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/version.h>

#include "st_mag40_core.h"

static const struct regmap_config st_mag40_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_mag40_spi_probe(struct spi_device *spi)
{
	struct st_mag40_data *cdata;
	struct iio_dev *iio_dev;
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_mag40_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	iio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*cdata));
	if (!iio_dev)
		return -ENOMEM;

	spi_set_drvdata(spi, iio_dev);
	iio_dev->dev.parent = &spi->dev;
	iio_dev->name = spi->modalias;

	cdata = iio_priv(iio_dev);
	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->regmap = regmap;
	cdata->irq = spi->irq;

	return st_mag40_common_probe(iio_dev);
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void st_mag40_spi_remove(struct spi_device *spi)
{
	st_mag40_remove(&spi->dev);
}
#else /* LINUX_VERSION_CODE */
static int st_mag40_spi_remove(struct spi_device *spi)
{
	st_mag40_remove(&spi->dev);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused st_mag40_spi_suspend(struct device *dev)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct st_mag40_data *cdata = iio_priv(iio_dev);

	return st_mag40_common_suspend(cdata);
}

static int __maybe_unused st_mag40_spi_resume(struct device *dev)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct st_mag40_data *cdata = iio_priv(iio_dev);

	return st_mag40_common_resume(cdata);
}

static const struct dev_pm_ops st_mag40_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_mag40_spi_suspend, st_mag40_spi_resume)
};
#define ST_MAG40_PM_OPS		(&st_mag40_spi_pm_ops)
#else /* CONFIG_PM */
#define ST_MAG40_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id st_mag40_ids[] = {
	{ LSM303AH_DEV_NAME, 0 },
	{ LSM303AGR_DEV_NAME, 0 },
	{ LIS2MDL_DEV_NAME, 0 },
	{ ISM303DAC_DEV_NAME, 0 },
	{ IIS2MDC_DEV_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(spi, st_mag40_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id st_mag40_id_table[] = {
	{
		.compatible = "st,lsm303ah_magn",
		.data = LSM303AH_DEV_NAME,
	},
	{
		.compatible = "st,lsm303agr_magn",
		.data = LSM303AGR_DEV_NAME,
	},
	{
		.compatible = "st,lis2mdl_magn",
		.data = LSM303AGR_DEV_NAME,
	},
	{
		.compatible = "st,ism303dac_magn",
		.data = ISM303DAC_DEV_NAME,
	},
	{
		.compatible = "st,iis2mdc_magn",
		.data = IIS2MDC_DEV_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, st_mag40_id_table);
#endif /* CONFIG_OF */

static struct spi_driver st_mag40_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = ST_MAG40_DEV_NAME,
		   .pm = ST_MAG40_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = st_mag40_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = st_mag40_spi_probe,
	.remove = st_mag40_spi_remove,
	.id_table = st_mag40_ids,
};
module_spi_driver(st_mag40_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics st_mag40 spi driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
