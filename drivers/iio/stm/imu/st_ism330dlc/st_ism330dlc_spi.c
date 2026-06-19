// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics ism330dlc spi driver
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

#include "st_ism330dlc.h"

static const struct regmap_config st_ism330dlc_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_ism330dlc_spi_probe(struct spi_device *spi)
{
	struct ism330dlc_data *cdata;
	struct regmap *regmap;
	int err;

	regmap = devm_regmap_init_spi(spi, &st_ism330dlc_spi_regmap_config);
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

	err = st_ism330dlc_common_probe(cdata, spi->irq);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
static void st_ism330dlc_spi_remove(struct spi_device *spi)
{
	struct ism330dlc_data *cdata = spi_get_drvdata(spi);

	st_ism330dlc_common_remove(cdata, spi->irq);
	kfree(cdata);
}
#else /* LINUX_VERSION_CODE */
static int st_ism330dlc_spi_remove(struct spi_device *spi)
{
	struct ism330dlc_data *cdata = spi_get_drvdata(spi);

	st_ism330dlc_common_remove(cdata, spi->irq);
	kfree(cdata);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused st_ism330dlc_suspend(struct device *dev)
{
	struct ism330dlc_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return st_ism330dlc_common_suspend(cdata);
}

static int __maybe_unused st_ism330dlc_resume(struct device *dev)
{
	struct ism330dlc_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return st_ism330dlc_common_resume(cdata);
}

static const struct dev_pm_ops st_ism330dlc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_ism330dlc_suspend, st_ism330dlc_resume)
};

#define ST_ISM330DLC_PM_OPS		(&st_ism330dlc_pm_ops)
#else /* CONFIG_PM */
#define ST_ISM330DLC_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id st_ism330dlc_id_table[] = {
	{ ISM330DLC_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_ism330dlc_id_table);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ism330dlc_of_match[] = {
	{
		.compatible = "st,ism330dlc",
		.data = ISM330DLC_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ism330dlc_of_match);
#else /* CONFIG_OF */
#define ism330dlc_of_match		NULL
#endif /* CONFIG_OF */

static struct spi_driver st_ism330dlc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-ism330dlc-spi",
		.pm = ST_ISM330DLC_PM_OPS,
		.of_match_table = of_match_ptr(ism330dlc_of_match),
	},
	.probe = st_ism330dlc_spi_probe,
	.remove = st_ism330dlc_spi_remove,
	.id_table = st_ism330dlc_id_table,
};
module_spi_driver(st_ism330dlc_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics ism330dlc spi driver");
MODULE_LICENSE("GPL v2");
