// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22hh i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2017 STMicroelectronics Inc.
 */

#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/version.h>

#include "st_lps22hh.h"

static const struct regmap_config st_lps22hh_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
static int st_lps22hh_i2c_probe(struct i2c_client *client)
{
#else /* LINUX_VERSION_CODE */
static int st_lps22hh_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
#endif /* LINUX_VERSION_CODE */

	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_lps22hh_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));

		return PTR_ERR(regmap);
	}

	return st_lps22hh_common_probe(&client->dev, client->irq,
				       client->name, regmap);
}

static const struct i2c_device_id st_lps22hh_ids[] = {
	{ "lps22ch" },
	{ "lps22hh" },
	{ "lps27hhw" },
	{}
};
MODULE_DEVICE_TABLE(i2c, st_lps22hh_ids);

static const struct of_device_id st_lps22hh_id_table[] = {
	{ .compatible = "st,lps22ch" },
	{ .compatible = "st,lps22hh" },
	{ .compatible = "st,lps27hhw" },
	{},
};
MODULE_DEVICE_TABLE(of, st_lps22hh_id_table);

static struct i2c_driver st_lps22hh_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "st_lps22hh_i2c",
		   .of_match_table = of_match_ptr(st_lps22hh_id_table),
	},
	.probe = st_lps22hh_i2c_probe,
	.id_table = st_lps22hh_ids,
};
module_i2c_driver(st_lps22hh_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lps22hh i2c driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
