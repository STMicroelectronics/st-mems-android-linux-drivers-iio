// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_h3lis331dl i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2023 STMicroelectronics Inc.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/version.h>

#include "st_h3lis331dl.h"

static const struct regmap_config st_h3lis331dl_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#if KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE
static int st_h3lis331dl_i2c_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	enum st_h3lis331dl_hw_id hw_id;
	struct regmap *regmap;
	const void *data;

	data = device_get_match_data(&client->dev);
	if (data)
		hw_id = (uintptr_t)data;
	else if (id)
		hw_id = (enum st_h3lis331dl_hw_id)id->driver_data;
	else
		return -ENOSYS;

	regmap = devm_regmap_init_i2c(client,
				      &st_h3lis331dl_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_h3lis331dl_probe(&client->dev, client->irq,
			       hw_id, regmap);
}
#else /* LINUX_VERSION_CODE */
static int st_h3lis331dl_i2c_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	int hw_id = id->driver_data;
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client,
				      &st_h3lis331dl_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_h3lis331dl_probe(&client->dev, client->irq,
			       hw_id, regmap);
}
#endif /* LINUX_VERSION_CODE */

static const struct of_device_id st_h3lis331dl_i2c_of_match[] = {
	{
		.compatible = "st," ST_H3LIS331DL_DEV_NAME,
	},
	{
		.compatible = "st," ST_LIS331DLH_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_h3lis331dl_i2c_of_match);

static const struct i2c_device_id st_h3lis331dl_i2c_id_table[] = {
	{ ST_H3LIS331DL_DEV_NAME, ST_H3LIS331DL_ID },
	{ ST_LIS331DLH_DEV_NAME, ST_LIS331DLH_ID },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_h3lis331dl_i2c_id_table);

static struct i2c_driver st_h3lis331dl_driver = {
	.driver = {
		.name = "st_h3lis331dl_i2c",
		.pm = &st_h3lis331dl_pm_ops,
		.of_match_table = st_h3lis331dl_i2c_of_match,
	},
	.probe = st_h3lis331dl_i2c_probe,
	.id_table = st_h3lis331dl_i2c_id_table,
};
module_i2c_driver(st_h3lis331dl_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_h3lis331dl i2c driver");
MODULE_LICENSE("GPL v2");
