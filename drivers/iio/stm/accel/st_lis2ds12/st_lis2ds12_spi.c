// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2ds12 spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "st_lis2ds12.h"

static const struct regmap_config st_lis2ds12_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int lis2ds12_spi_probe(struct spi_device *spi)
{
	struct lis2ds12_data *cdata;
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_lis2ds12_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));

		return PTR_ERR(regmap);
	}

	cdata = devm_kzalloc(&spi->dev, sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &spi->dev;
	cdata->name = spi->modalias;
	cdata->regmap = regmap;
	spi_set_drvdata(spi, cdata);

	return lis2ds12_common_probe(cdata, spi->irq);
}

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused lis2ds12_suspend(struct device *dev)
{
	struct lis2ds12_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lis2ds12_common_suspend(cdata);
}

static int __maybe_unused lis2ds12_resume(struct device *dev)
{
	struct lis2ds12_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return lis2ds12_common_resume(cdata);
}

static const struct dev_pm_ops lis2ds12_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis2ds12_suspend, lis2ds12_resume)
};

#define LIS2DS12_PM_OPS		(&lis2ds12_pm_ops)
#else /* CONFIG_PM */
#define LIS2DS12_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id lis2ds12_ids[] = {
	{"lis2ds12", 0},
	{"lsm303ah", 0},
	{"lis2dg", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, lis2ds12_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lis2ds12_id_table[] = {
	{.compatible = "st,lis2ds12",},
	{.compatible = "st,lsm303ah",},
	{.compatible = "st,lis2dg",},
	{},
};

MODULE_DEVICE_TABLE(of, lis2ds12_id_table);
#endif /* CONFIG_OF */

static struct spi_driver lis2ds12_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LIS2DS12_DEV_NAME,
		   .pm = LIS2DS12_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = lis2ds12_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = lis2ds12_spi_probe,
	.id_table = lis2ds12_ids,
};

module_spi_driver(lis2ds12_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2ds12 spi driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
