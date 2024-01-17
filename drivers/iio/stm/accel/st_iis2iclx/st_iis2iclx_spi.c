// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_iis2iclx spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/version.h>

#include "st_iis2iclx.h"

static const struct regmap_config st_iis2iclx_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_iis2iclx_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_iis2iclx_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));

		return PTR_ERR(regmap);
	}

	return st_iis2iclx_probe(&spi->dev, spi->irq, regmap);
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void st_iis2iclx_spi_remove(struct spi_device *spi)
{
	st_iis2iclx_remove(&spi->dev);
}
#else /* LINUX_VERSION_CODE */
static int st_iis2iclx_spi_remove(struct spi_device *spi)
{
	st_iis2iclx_remove(&spi->dev);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

static const struct of_device_id st_iis2iclx_spi_of_match[] = {
	{ .compatible = "st," ST_IIS2ICLX_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, st_iis2iclx_spi_of_match);

static const struct spi_device_id st_iis2iclx_spi_id_table[] = {
	{ ST_IIS2ICLX_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_iis2iclx_spi_id_table);

static struct spi_driver st_iis2iclx_driver = {
	.driver = {
		.name = "st_iis2iclx_spi",
		.pm = &st_iis2iclx_pm_ops,
		.of_match_table = st_iis2iclx_spi_of_match,
	},
	.probe = st_iis2iclx_spi_probe,
	.remove = st_iis2iclx_spi_remove,
	.id_table = st_iis2iclx_spi_id_table,
};
module_spi_driver(st_iis2iclx_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_iis2iclx spi driver");
MODULE_LICENSE("GPL v2");
