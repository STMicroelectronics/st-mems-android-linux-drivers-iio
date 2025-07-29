// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lps22df i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2021 STMicroelectronics Inc.
 */

#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/version.h>

#include "st_lps22df.h"

static const struct regmap_config st_lps22df_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
static int st_lps22df_i2c_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
#else /* LINUX_VERSION_CODE */
static int st_lps22df_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
#endif /* LINUX_VERSION_CODE */

	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_lps22df_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));

		return PTR_ERR(regmap);
	}

	return st_lps22df_probe(&client->dev, client->irq,
				id->driver_data, regmap);
}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static void st_lps22df_i2c_remove(struct i2c_client *client)
{
	st_lps22df_remove(&client->dev);
}
#else /* LINUX_VERSION_CODE */
static int st_lps22df_i2c_remove(struct i2c_client *client)
{
	return st_lps22df_remove(&client->dev);
}
#endif /* LINUX_VERSION_CODE */

static const struct i2c_device_id st_lps22df_ids[] = {
	{ "lps22df", ST_LPS22DF_ID },
	{ "lps28dfw", ST_LPS28DFW_ID },
	{}
};
MODULE_DEVICE_TABLE(i2c, st_lps22df_ids);

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

static struct i2c_driver st_lps22df_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "st_lps22df_i2c",
		   .of_match_table = of_match_ptr(st_lps22df_id_table),
	},
	.probe = st_lps22df_i2c_probe,
	.remove = st_lps22df_i2c_remove,
	.id_table = st_lps22df_ids,
};
module_i2c_driver(st_lps22df_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lps22df i2c driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
