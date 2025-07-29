// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22df spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2021 STMicroelectronics Inc.
 */

#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/version.h>

#include "st_lps22df.h"

static const struct regmap_config st_lps22df_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_lps22df_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_lps22df_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));

		return PTR_ERR(regmap);
	}

	return st_lps22df_probe(&spi->dev, spi->irq,
				id->driver_data, regmap);
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void st_lps22df_spi_remove(struct spi_device *spi)
{
	st_lps22df_remove(&spi->dev);
}
#else /* LINUX_VERSION_CODE */
static int st_lps22df_spi_remove(struct spi_device *spi)
{
	return st_lps22df_remove(&spi->dev);
}
#endif /* LINUX_VERSION_CODE */

static const struct spi_device_id st_lps22df_ids[] = {
	{ "lps22df", ST_LPS22DF_ID },
	{ "lps28dfw", ST_LPS28DFW_ID },
	{}
};
MODULE_DEVICE_TABLE(spi, st_lps22df_ids);

static const struct of_device_id st_lps22df_id_table[] = {
	{
		.compatible = "st,lps22df",
		.data = (void *)ST_LPS22DF_ID,
	},
	{
		.compatible = "st,lps28dfw",
		.data = (void *)ST_LPS28DFW_ID,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_lps22df_id_table);

static struct spi_driver st_lps22df_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "st_lps22df_spi",
		   .of_match_table = of_match_ptr(st_lps22df_id_table),
	},
	.probe = st_lps22df_spi_probe,
	.remove = st_lps22df_spi_remove,
	.id_table = st_lps22df_ids,
};
module_spi_driver(st_lps22df_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lps22df spi driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
