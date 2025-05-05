// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_hts221 i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "st_hts221.h"

#define ST_HTS221_I2C_AUTO_INCREMENT	BIT(7)

static const struct regmap_config st_hts221_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.write_flag_mask = ST_HTS221_I2C_AUTO_INCREMENT,
	.read_flag_mask = ST_HTS221_I2C_AUTO_INCREMENT,
};

#if KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE
static int st_hts221_i2c_probe(struct i2c_client *client)
{
#else /* LINUX_VERSION_CODE */
static int st_hts221_i2c_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
#endif /* LINUX_VERSION_CODE */

	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_hts221_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_hts221_probe(&client->dev, client->irq,
			       client->name, regmap);
}

static const struct acpi_device_id st_hts221_acpi_match[] = {
	{"SMO9100", 0},
	{ },
};
MODULE_DEVICE_TABLE(acpi, st_hts221_acpi_match);

static const struct of_device_id st_hts221_i2c_of_match[] = {
	{ .compatible = "st,st_hts221", },
	{},
};
MODULE_DEVICE_TABLE(of, st_hts221_i2c_of_match);

static const struct i2c_device_id st_hts221_i2c_id_table[] = {
	{ ST_HTS221_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_hts221_i2c_id_table);

static struct i2c_driver st_hts221_driver = {
	.driver = {
		.name = "st_hts221_i2c",
		.pm = &st_hts221_pm_ops,
		.of_match_table = of_match_ptr(st_hts221_i2c_of_match),
		.acpi_match_table = ACPI_PTR(st_hts221_acpi_match),
	},
	.probe = st_hts221_i2c_probe,
	.id_table = st_hts221_i2c_id_table,
};
module_i2c_driver(st_hts221_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_hts221 i2c driver");
MODULE_LICENSE("GPL v2");
