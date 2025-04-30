// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics mag3d spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/iio/iio.h>
#include <linux/spi/spi.h>

#include "st_mag3d.h"

static const struct regmap_config st_mag3d_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_mag3d_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_mag3d_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_mag3d_probe(&spi->dev, spi->irq, spi->modalias, regmap);
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void st_mag3d_spi_remove(struct spi_device *spi)
{
	struct iio_dev *iio_dev = spi_get_drvdata(spi);

	st_mag3d_remove(iio_dev);
}
#else /* LINUX_VERSION_CODE */
static int st_mag3d_spi_remove(struct spi_device *spi)
{
	struct iio_dev *iio_dev = spi_get_drvdata(spi);

	st_mag3d_remove(iio_dev);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

static const struct spi_device_id st_mag3d_ids[] = {
	{ LIS3MDL_DEV_NAME },
	{ LSM9DS1_DEV_NAME },
	{}
};
MODULE_DEVICE_TABLE(spi, st_mag3d_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id st_mag3d_id_table[] = {
	{
		.compatible = "st,lis3mdl_magn",
	},
	{
		.compatible = "st,lsm9ds1_magn",
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_mag3d_id_table);
#endif /* CONFIG_OF */

static struct spi_driver st_mag3d_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "st_mag3d_spi",
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = st_mag3d_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = st_mag3d_spi_probe,
	.remove = st_mag3d_spi_remove,
	.id_table = st_mag3d_ids,
};
module_spi_driver(st_mag3d_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics mag3d spi driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
