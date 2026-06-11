// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics ism303dac spi driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2018, 2026 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "st_ism303dac.h"

static const struct regmap_config ism303dac_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ism303dac_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &ism303dac_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return ism303dac_probe(&spi->dev, spi->irq,
			       spi->modalias, regmap);
}

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused ism303dac_suspend(struct device *dev)
{
	struct ism303dac_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return ism303dac_common_suspend(cdata);
}

static int __maybe_unused ism303dac_resume(struct device *dev)
{
	struct ism303dac_data *cdata = spi_get_drvdata(to_spi_device(dev));

	return ism303dac_common_resume(cdata);
}

static const struct dev_pm_ops ism303dac_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ism303dac_suspend, ism303dac_resume)
};

#define ISM303DAC_PM_OPS		(&ism303dac_pm_ops)
#else /* CONFIG_PM */
#define ISM303DAC_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct spi_device_id ism303dac_ids[] = {
	{ ISM303DAC_DEV_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(spi, ism303dac_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ism303dac_id_table[] = {
	{ .compatible = "st,ism303dac", },
	{},
};

MODULE_DEVICE_TABLE(of, ism303dac_id_table);
#endif /* CONFIG_OF */

static struct spi_driver ism303dac_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = ISM303DAC_DEV_NAME,
		   .pm = ISM303DAC_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = ism303dac_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = ism303dac_spi_probe,
	.id_table = ism303dac_ids,
};

module_spi_driver(ism303dac_spi_driver);

MODULE_DESCRIPTION("STMicroelectronics ism303dac spi driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
