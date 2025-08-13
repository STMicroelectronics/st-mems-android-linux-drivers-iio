// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lis2ds12 i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2015 STMicroelectronics Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/types.h>

#include "st_lis2ds12.h"

static const struct regmap_config st_lis2ds12_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
static int lis2ds12_i2c_probe(struct i2c_client *client)
{
#else /* LINUX_VERSION_CODE */
static int lis2ds12_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
#endif /* LINUX_VERSION_CODE */

	struct lis2ds12_data *cdata;
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_lis2ds12_i2c_regmap_config);
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

	return lis2ds12_common_probe(cdata, client->irq);
}

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused lis2ds12_suspend(struct device *dev)
{
	struct lis2ds12_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lis2ds12_common_suspend(cdata);
}

static int __maybe_unused lis2ds12_resume(struct device *dev)
{
	struct lis2ds12_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return lis2ds12_common_resume(cdata);
}

static const struct dev_pm_ops lis2ds12_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis2ds12_suspend, lis2ds12_resume)
};

#define LIS2DS12_PM_OPS		(&lis2ds12_pm_ops)
#else /* CONFIG_PM */
#define LIS2DS12_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lis2ds12_ids[] = {
	{"lis2ds12", 0},
	{"lsm303ah", 0},
	{"lis2dg", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lis2ds12_ids);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lis2ds12_id_table[] = {
	{.compatible = "st,lis2ds12",},
	{.compatible = "st,lsm303ah",},
	{.compatible = "st,lis2dg",},
	{},
};

MODULE_DEVICE_TABLE(of, lis2ds12_id_table);
#endif

static struct i2c_driver lis2ds12_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LIS2DS12_DEV_NAME,
		   .pm = LIS2DS12_PM_OPS,
#if IS_ENABLED(CONFIG_OF)
		   .of_match_table = lis2ds12_id_table,
#endif
		   },
	.probe = lis2ds12_i2c_probe,
	.id_table = lis2ds12_ids,
};

module_i2c_driver(lis2ds12_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lis2ds12 i2c driver");
MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_LICENSE("GPL v2");
