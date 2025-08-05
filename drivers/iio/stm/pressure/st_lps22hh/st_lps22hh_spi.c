// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22hh spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2017 STMicroelectronics Inc.
 */

#include <linux/spi/spi.h>
#include <linux/of.h>

#include "st_lps22hh.h"

static const struct regmap_config st_lps22hh_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_lps22hh_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_lps22hh_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_lps22hh_common_probe(&spi->dev, spi->irq, spi->modalias,
				       regmap);
}

static const struct spi_device_id st_lps22hh_ids[] = {
	{ "lps22ch" },
	{ "lps22hh" },
	{ "lps27hhw" },
	{}
};
MODULE_DEVICE_TABLE(spi, st_lps22hh_ids);

static const struct of_device_id st_lps22hh_id_table[] = {
	{ .compatible = "st,lps22ch" },
	{ .compatible = "st,lps22hh" },
	{ .compatible = "st,lps27hhw" },
	{},
};
MODULE_DEVICE_TABLE(of, st_lps22hh_id_table);

static struct spi_driver st_lps22hh_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "st_lps22hh_spi",
		   .of_match_table = of_match_ptr(st_lps22hh_id_table),
	},
	.probe = st_lps22hh_spi_probe,
	.id_table = st_lps22hh_ids,
};
module_spi_driver(st_lps22hh_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lps22hh spi driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
