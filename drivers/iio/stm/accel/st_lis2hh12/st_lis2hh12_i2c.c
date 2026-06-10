// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2hh12 driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/version.h>

#include "st_lis2hh12.h"

static const struct regmap_config lis2hh12_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

ST_I2C_PROBE(lis2hh12_i2c_probe)
{
	struct lis2hh12_data *cdata;
	struct regmap *regmap;
	int err;

	regmap = devm_regmap_init_i2c(client, &lis2hh12_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	cdata = devm_kzalloc(&client->dev, sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->regmap = regmap;
	i2c_set_clientdata(client, cdata);

	err = lis2hh12_common_probe(cdata, client->irq);

	return err < 0 ? err : 0;
}

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused lis2hh12_suspend(struct device *dev)
{
	struct lis2hh12_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lis2hh12_common_suspend(cdata);
}

static int __maybe_unused lis2hh12_resume(struct device *dev)
{
	struct lis2hh12_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lis2hh12_common_resume(cdata);
}

static const struct dev_pm_ops lis2hh12_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis2hh12_suspend, lis2hh12_resume)
};

#define LIS2HH12_PM_OPS		(&lis2hh12_pm_ops)
#else /* CONFIG_PM */
#define LIS2HH12_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lis2hh12_ids[] = {
	{"lis2hh12", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lis2hh12_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lis2hh12_id_table[] = {
	{.compatible = "st,lis2hh12",},
	{},
};

MODULE_DEVICE_TABLE(of, lis2hh12_id_table);
#endif /* CONFIG_OF */

static struct i2c_driver lis2hh12_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LIS2HH12_DEV_NAME,
		   .pm = LIS2HH12_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = lis2hh12_id_table,
#endif /* CONFIG_OF */
		   },
	.probe = lis2hh12_i2c_probe,
	.id_table = lis2hh12_ids,
};

module_i2c_driver(lis2hh12_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2hh12 i2c driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
