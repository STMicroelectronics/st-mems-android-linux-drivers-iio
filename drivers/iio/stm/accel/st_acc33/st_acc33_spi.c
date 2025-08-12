// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_acc33 spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "st_acc33.h"

static const struct regmap_config st_acc33_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_acc33_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_acc33_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_acc33_probe(&spi->dev, spi->irq, spi->modalias, regmap);
}

static const struct of_device_id st_acc33_spi_of_match[] = {
	{
		.compatible = "st,lis2dh_accel",
		.data = LIS2DH_DEV_NAME,
	},
	{
		.compatible = "st,lis2dh12_accel",
		.data = LIS2DH12_DEV_NAME,
	},
	{
		.compatible = "st,lis3dh_accel",
		.data = LIS3DH_DEV_NAME,
	},
	{
		.compatible = "st,lsm303agr_accel",
		.data = LSM303AGR_DEV_NAME,
	},
	{
		.compatible = "st,iis2dh_accel",
		.data = IIS2DH_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_acc33_spi_of_match);

static const struct spi_device_id st_acc33_spi_id_table[] = {
	{ LIS2DH_DEV_NAME },
	{ LIS2DH12_DEV_NAME },
	{ LIS3DH_DEV_NAME },
	{ LSM303AGR_DEV_NAME },
	{ IIS2DH_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_acc33_spi_id_table);

static struct spi_driver st_acc33_driver = {
	.driver = {
		.name = "st_acc33_spi",
		.of_match_table = of_match_ptr(st_acc33_spi_of_match),
	},
	.probe = st_acc33_spi_probe,
	.id_table = st_acc33_spi_id_table,
};
module_spi_driver(st_acc33_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_acc33 spi driver");
MODULE_LICENSE("GPL v2");
