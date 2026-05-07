// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics mag3d i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/version.h>

#include "st_mag3d.h"

static const struct regmap_config st_mag3d_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

ST_I2C_PROBE(st_mag3d_i2c_probe)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_mag3d_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_mag3d_probe(&client->dev, client->irq, client->name, regmap);
}

static void st_mag3d_i2c_remove_dev(struct device *dev)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);

	st_mag3d_remove(iio_dev);
}

ST_I2C_REMOVE(st_mag3d_i2c_remove, st_mag3d_i2c_remove_dev)

static const struct i2c_device_id st_mag3d_ids[] = {
	{ LIS3MDL_DEV_NAME },
	{ LSM9DS1_DEV_NAME },
	{}
};
MODULE_DEVICE_TABLE(i2c, st_mag3d_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id st_mag3d_id_table[] = {
	{
		.compatible = "st,lis3mdl_magn",
	},
	{
		.compatible = "st,lsm9ds1_magn",
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_mag3d_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver st_mag3d_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "st_mag3d_i2c",
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = st_mag3d_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = st_mag3d_i2c_probe,
	.remove = st_mag3d_i2c_remove,
	.id_table = st_mag3d_ids,
};
module_i2c_driver(st_mag3d_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics mag3d i2c driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
