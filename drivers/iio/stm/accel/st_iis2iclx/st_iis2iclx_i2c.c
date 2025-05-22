// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_iis2iclx i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/version.h>

#include "st_iis2iclx.h"

static const struct regmap_config st_iis2iclx_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
static int st_iis2iclx_i2c_probe(struct i2c_client *client)
#else /* LINUX_VERSION_CODE */
static int st_iis2iclx_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
#endif /* LINUX_VERSION_CODE */
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_iis2iclx_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));

		return PTR_ERR(regmap);
	}

	return st_iis2iclx_probe(&client->dev, client->irq, regmap);
}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static void st_iis2iclx_i2c_remove(struct i2c_client *client)
{
	st_iis2iclx_remove(&client->dev);
}
#else /* LINUX_VERSION_CODE */
static int st_iis2iclx_i2c_remove(struct i2c_client *client)
{
	st_iis2iclx_remove(&client->dev);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

static const struct of_device_id st_iis2iclx_i2c_of_match[] = {
	{ .compatible = "st," ST_IIS2ICLX_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, st_iis2iclx_i2c_of_match);

static const struct i2c_device_id st_iis2iclx_i2c_id_table[] = {
	{ ST_IIS2ICLX_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_iis2iclx_i2c_id_table);

static struct i2c_driver st_iis2iclx_driver = {
	.driver = {
		.name = "st_iis2iclx_i2c",
		.pm = &st_iis2iclx_pm_ops,
		.of_match_table = st_iis2iclx_i2c_of_match,
	},
	.probe = st_iis2iclx_i2c_probe,
	.remove = st_iis2iclx_i2c_remove,
	.id_table = st_iis2iclx_i2c_id_table,
};
module_i2c_driver(st_iis2iclx_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_iis2iclx i2c driver");
MODULE_LICENSE("GPL v2");
