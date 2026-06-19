// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lsm6ds3h spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016, 2026 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/iio/iio.h>

#include "st_lsm6ds3h.h"

static const struct regmap_config st_lsm6dsm_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_lsm6ds3h_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_lsm6dsm_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_lsm6ds3h_probe(&spi->dev, spi->irq,
				 spi->modalias, regmap);
}

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused st_lsm6ds3h_suspend(struct device *dev)
{
	struct lsm6ds3h_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return st_lsm6ds3h_common_suspend(cdata);
}

static int __maybe_unused st_lsm6ds3h_resume(struct device *dev)
{
	struct lsm6ds3h_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return st_lsm6ds3h_common_resume(cdata);
}

static const struct dev_pm_ops st_lsm6ds3h_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lsm6ds3h_suspend, st_lsm6ds3h_resume)
};

#define ST_LSM6DS3H_PM_OPS		(&st_lsm6ds3h_pm_ops)
#else /* CONFIG_PM */
#define ST_LSM6DS3H_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id st_lsm6ds3h_id_table[] = {
	{ LSM6DS3H_DEV_NAME },
	{ },
};
MODULE_DEVICE_TABLE(spi, st_lsm6ds3h_id_table);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lsm6ds3h_of_match[] = {
	{
		.compatible = "st,lsm6ds3h",
		.data = LSM6DS3H_DEV_NAME,
	},
	{}
};
MODULE_DEVICE_TABLE(of, lsm6ds3h_of_match);
#else /* CONFIG_OF */
#define lsm6ds3h_of_match		NULL
#endif /* CONFIG_OF */

static struct spi_driver st_lsm6ds3h_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-lsm6ds3h-spi",
		.pm = ST_LSM6DS3H_PM_OPS,
		.of_match_table = of_match_ptr(lsm6ds3h_of_match),
	},
	.probe = st_lsm6ds3h_spi_probe,
	.id_table = st_lsm6ds3h_id_table,
};
module_spi_driver(st_lsm6ds3h_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics lsm6ds3h spi driver");
MODULE_LICENSE("GPL v2");
