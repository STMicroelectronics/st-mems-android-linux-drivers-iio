// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics ism303dac i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2018, 2026 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/hrtimer.h>
#include <linux/types.h>
#include <linux/version.h>

#include "st_ism303dac.h"

static const struct regmap_config ism303dac_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

ST_I2C_PROBE(ism303dac_i2c_probe)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &ism303dac_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return ism303dac_probe(&client->dev, client->irq,
			       client->name, regmap);
}

static const struct i2c_device_id ism303dac_ids[] = {
	{ ISM303DAC_DEV_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ism303dac_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ism303dac_id_table[] = {
	{ .compatible = "st,ism303dac", },
	{},
};

MODULE_DEVICE_TABLE(of, ism303dac_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver ism303dac_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = ISM303DAC_DEV_NAME,
		   .pm = &ism303dac_pm_ops,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = ism303dac_id_table,
#endif /* CONFIG_OF */
	},
	.probe = ism303dac_i2c_probe,
	.id_table = ism303dac_ids,
};

module_i2c_driver(ism303dac_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics ism303dac i2c driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
