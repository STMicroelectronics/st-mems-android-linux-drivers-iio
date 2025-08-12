// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_acc33 i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/version.h>

#include "st_acc33.h"

#define ST_ACC33_AUTO_INCREMENT		BIT(7)

static const struct regmap_config st_acc33_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.write_flag_mask = ST_ACC33_AUTO_INCREMENT,
	.read_flag_mask = ST_ACC33_AUTO_INCREMENT,
};

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
static int st_acc33_i2c_probe(struct i2c_client *client)
{
#else /* LINUX_VERSION_CODE */
static int st_acc33_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
#endif /* LINUX_VERSION_CODE */

	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_acc33_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_acc33_probe(&client->dev, client->irq, client->name, regmap);
}

static const struct of_device_id st_acc33_i2c_of_match[] = {
	{
		.compatible = "st,lis2dh_accel",
		.data = LIS2DH_DEV_NAME,
	},
	{
		.compatible = "st,lis2dh12_accel",
		.data = LIS2DH12_DEV_NAME,
	},
	{
		.compatible = "st,lis3dh_accel",
		.data = LIS3DH_DEV_NAME,
	},
	{
		.compatible = "st,lsm303agr_accel",
		.data = LSM303AGR_DEV_NAME,
	},
	{
		.compatible = "st,iis2dh_accel",
		.data = IIS2DH_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_acc33_i2c_of_match);

static const struct i2c_device_id st_acc33_i2c_id_table[] = {
	{ LIS2DH_DEV_NAME },
	{ LIS2DH12_DEV_NAME },
	{ LIS3DH_DEV_NAME },
	{ LSM303AGR_DEV_NAME },
	{ IIS2DH_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_acc33_i2c_id_table);

static struct i2c_driver st_acc33_driver = {
	.driver = {
		.name = "st_acc33_i2c",
		.of_match_table = of_match_ptr(st_acc33_i2c_of_match),
	},
	.probe = st_acc33_i2c_probe,
	.id_table = st_acc33_i2c_id_table,
};
module_i2c_driver(st_acc33_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_acc33 i2c driver");
MODULE_LICENSE("GPL v2");
