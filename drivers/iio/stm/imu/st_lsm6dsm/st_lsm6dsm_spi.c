// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lsm6dsm spi driver
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

#include "st_lsm6dsm.h"

static const struct regmap_config st_lsm6dsm_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_lsm6dsm_spi_probe(struct spi_device *spi)
{
	struct lsm6dsm_data *cdata;
	struct regmap *regmap;
	int err;

	regmap = devm_regmap_init_spi(spi, &st_lsm6dsm_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->regmap = regmap;
	spi_set_drvdata(spi, cdata);

	err = st_lsm6dsm_common_probe(cdata, spi->irq);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void st_lsm6dsm_spi_remove(struct spi_device *spi)
{
	struct lsm6dsm_data *cdata = spi_get_drvdata(spi);

	st_lsm6dsm_common_remove(cdata, spi->irq);
	kfree(cdata);
}
#else /* LINUX_VERSION_CODE */
static int st_lsm6dsm_spi_remove(struct spi_device *spi)
{
	struct lsm6dsm_data *cdata = spi_get_drvdata(spi);

	st_lsm6dsm_common_remove(cdata, spi->irq);
	kfree(cdata);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused st_lsm6dsm_suspend(struct device *dev)
{
	struct lsm6dsm_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return st_lsm6dsm_common_suspend(cdata);
}

static int __maybe_unused st_lsm6dsm_resume(struct device *dev)
{
	struct lsm6dsm_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return st_lsm6dsm_common_resume(cdata);
}

static const struct dev_pm_ops st_lsm6dsm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lsm6dsm_suspend, st_lsm6dsm_resume)
};

#define ST_LSM6DSM_PM_OPS		(&st_lsm6dsm_pm_ops)
#else /* CONFIG_PM */
#define ST_LSM6DSM_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id st_lsm6dsm_id_table[] = {
	{ LSM6DSM_DEV_NAME },
	{ LSM6DSL_DEV_NAME },
	{ },
};
MODULE_DEVICE_TABLE(spi, st_lsm6dsm_id_table);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lsm6dsm_of_match[] = {
	{
		.compatible = "st,lsm6dsm",
		.data = LSM6DSM_DEV_NAME,
	},
	{
		.compatible = "st,lsm6dsl",
		.data = LSM6DSL_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, lsm6dsm_of_match);
#else /* CONFIG_OF */
#define lsm6dsm_of_match		NULL
#endif /* CONFIG_OF */

static struct spi_driver st_lsm6dsm_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-lsm6dsm-spi",
		.pm = ST_LSM6DSM_PM_OPS,
		.of_match_table = of_match_ptr(lsm6dsm_of_match),
	},
	.probe = st_lsm6dsm_spi_probe,
	.remove = st_lsm6dsm_spi_remove,
	.id_table = st_lsm6dsm_id_table,
};
module_spi_driver(st_lsm6dsm_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics lsm6dsm spi driver");
MODULE_LICENSE("GPL v2");
