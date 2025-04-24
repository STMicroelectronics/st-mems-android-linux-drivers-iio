// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_imu68 spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/of.h>

#include "st_imu68.h"

static const struct regmap_config st_imu68_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_imu68_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_imu68_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_imu68_probe(&spi->dev, spi->irq, spi->modalias, regmap);
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void st_imu68_spi_remove(struct spi_device *spi)
{
	st_imu68_remove(&spi->dev);
}
#else /* LINUX_VERSION_CODE */
static int st_imu68_spi_remove(struct spi_device *spi)
{
	st_imu68_remove(&spi->dev);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

static const struct of_device_id st_imu68_spi_of_match[] = {
	{
		.compatible = "st,lsm9ds1",
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_imu68_spi_of_match);

static const struct spi_device_id st_imu68_spi_id_table[] = {
	{ ST_LSM9DS1_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_imu68_spi_id_table);

static struct spi_driver st_imu68_driver = {
	.driver = {
		.name = "st_imu68_spi",
		.of_match_table = of_match_ptr(st_imu68_spi_of_match),
	},
	.probe = st_imu68_spi_probe,
	.remove = st_imu68_spi_remove,
	.id_table = st_imu68_spi_id_table,
};
module_spi_driver(st_imu68_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_imu68 spi driver");
MODULE_LICENSE("GPL v2");
