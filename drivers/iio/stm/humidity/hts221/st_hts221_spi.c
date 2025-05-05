// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_hts221 spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#include "st_hts221.h"

#define ST_HTS221_SPI_READ		BIT(7)
#define ST_HTS221_SPI_AUTO_INCREMENT	BIT(6)

static const struct regmap_config st_hts221_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.write_flag_mask = ST_HTS221_SPI_AUTO_INCREMENT,
	.read_flag_mask = ST_HTS221_SPI_READ | ST_HTS221_SPI_AUTO_INCREMENT,
};

static int st_hts221_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_hts221_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_hts221_probe(&spi->dev, spi->irq,
			    spi->modalias, regmap);
}

static const struct of_device_id st_hts221_spi_of_match[] = {
	{ .compatible = "st,st_hts221", },
	{},
};
MODULE_DEVICE_TABLE(of, st_hts221_spi_of_match);

static const struct spi_device_id st_hts221_spi_id_table[] = {
	{ ST_HTS221_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_hts221_spi_id_table);

static struct spi_driver st_hts221_driver = {
	.driver = {
		.name = "st_hts221_spi",
		.pm = &st_hts221_pm_ops,
		.of_match_table = of_match_ptr(st_hts221_spi_of_match),
	},
	.probe = st_hts221_spi_probe,
	.id_table = st_hts221_spi_id_table,
};
module_spi_driver(st_hts221_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_hts221 spi driver");
MODULE_LICENSE("GPL v2");
