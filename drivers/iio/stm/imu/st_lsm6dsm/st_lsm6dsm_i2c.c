// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics lsm6dsm i2c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2016, 2026 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/iio/iio.h>

#include "st_lsm6dsm.h"

static const struct regmap_config st_lsm6dsm_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

ST_I2C_PROBE(st_lsm6dsm_i2c_probe)
{
	struct lsm6dsm_data *cdata;
	struct regmap *regmap;
	int err;

	regmap = devm_regmap_init_i2c(client, &st_lsm6dsm_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev,
			"Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	cdata = kmalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->dev = &client->dev;
	cdata->name = client->name;
	cdata->regmap = regmap;
	i2c_set_clientdata(client, cdata);

	err = st_lsm6dsm_common_probe(cdata, client->irq);
	if (err < 0)
		goto free_data;

	return 0;

free_data:
	kfree(cdata);
	return err;
}

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
static void st_lsm6dsm_i2c_remove(struct i2c_client *client)
{
	struct lsm6dsm_data *cdata = i2c_get_clientdata(client);

	st_lsm6dsm_common_remove(cdata, client->irq);
	kfree(cdata);
}
#else /* LINUX_VERSION_CODE */
static int st_lsm6dsm_i2c_remove(struct i2c_client *client)
{
	struct lsm6dsm_data *cdata = i2c_get_clientdata(client);

	st_lsm6dsm_common_remove(cdata, client->irq);
	kfree(cdata);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

#if IS_ENABLED(CONFIG_PM)
static int __maybe_unused st_lsm6dsm_suspend(struct device *dev)
{
	struct lsm6dsm_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return st_lsm6dsm_common_suspend(cdata);
}

static int __maybe_unused st_lsm6dsm_resume(struct device *dev)
{
	struct lsm6dsm_data *cdata = i2c_get_clientdata(to_i2c_client(dev));

	return st_lsm6dsm_common_resume(cdata);
}

static const struct dev_pm_ops st_lsm6dsm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lsm6dsm_suspend, st_lsm6dsm_resume)
};

#define ST_LSM6DSM_PM_OPS		(&st_lsm6dsm_pm_ops)
#else /* CONFIG_PM */
#define ST_LSM6DSM_PM_OPS		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id st_lsm6dsm_id_table[] = {
	{ LSM6DSM_DEV_NAME },
	{ LSM6DSL_DEV_NAME },
	{ },
};
MODULE_DEVICE_TABLE(i2c, st_lsm6dsm_id_table);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lsm6dsm_of_match[] = {
	{
		.compatible = "st,lsm6dsm",
		.data = LSM6DSM_DEV_NAME,
	},
	{
		.compatible = "st,lsm6dsl",
		.data = LSM6DSL_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, lsm6dsm_of_match);
#else /* CONFIG_OF */
#define lsm6dsm_of_match		NULL
#endif /* CONFIG_OF */

static struct i2c_driver st_lsm6dsm_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "st-lsm6dsm-i2c",
		.pm = ST_LSM6DSM_PM_OPS,
		.of_match_table = of_match_ptr(lsm6dsm_of_match),
	},
	.probe = st_lsm6dsm_i2c_probe,
	.remove = st_lsm6dsm_i2c_remove,
	.id_table = st_lsm6dsm_id_table,
};
module_i2c_driver(st_lsm6dsm_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics lsm6dsm i2c driver");
MODULE_LICENSE("GPL v2");
