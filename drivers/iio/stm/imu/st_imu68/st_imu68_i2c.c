// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_imu68 i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>

#include "st_imu68.h"

static const struct regmap_config st_imu68_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

ST_I2C_PROBE(st_imu68_i2c_probe)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_imu68_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_imu68_probe(&client->dev, client->irq, client->name, regmap);
}

ST_I2C_REMOVE(st_imu68_i2c_remove, st_imu68_remove)

static const struct of_device_id st_imu68_i2c_of_match[] = {
	{
		.compatible = "st,lsm9ds1",
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_imu68_i2c_of_match);

static const struct i2c_device_id st_imu68_i2c_id_table[] = {
	{ ST_LSM9DS1_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_imu68_i2c_id_table);

static struct i2c_driver st_imu68_driver = {
	.driver = {
		.name = "st_imu68_i2c",
		.of_match_table = of_match_ptr(st_imu68_i2c_of_match),
	},
	.probe = st_imu68_i2c_probe,
	.remove = st_imu68_i2c_remove,
	.id_table = st_imu68_i2c_id_table,
};
module_i2c_driver(st_imu68_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_imu68 i2c driver");
MODULE_LICENSE("GPL v2");
