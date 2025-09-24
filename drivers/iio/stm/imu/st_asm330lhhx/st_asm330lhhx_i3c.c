// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_asm330lhhx i3c driver
 *
 * MEMS Software Solutions Team
 *
 * Copyright 2025 STMicroelectronics Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/of.h>
#include <linux/version.h>

#include "st_asm330lhhx.h"

static const struct i3c_device_id st_asm330lhhx_i3c_ids[] = {
	I3C_DEVICE(0x0104, ST_ASM330LHHX_WHOAMI_VAL, (void *)ST_ASM330LHHX_ID),
	{},
};
MODULE_DEVICE_TABLE(i3c, st_asm330lhhx_i3c_ids);

static int st_asm330lhhx_i3c_probe(struct i3c_device *i3cdev)
{
	struct regmap_config st_asm330lhhx_i3c_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	const struct i3c_device_id *id =
		      i3c_device_match_id(i3cdev, st_asm330lhhx_i3c_ids);
	struct regmap *regmap;

	regmap = devm_regmap_init_i3c(i3cdev, &st_asm330lhhx_i3c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&i3cdev->dev,
			"Failed to register i3c regmap %d\n",
			(int)PTR_ERR(regmap));

		return PTR_ERR(regmap);
	}

	return st_asm330lhhx_probe(&i3cdev->dev, 0, (uintptr_t)id->data,
				 regmap, true);
}

#if KERNEL_VERSION(5, 12, 0) <= LINUX_VERSION_CODE
static void st_asm330lhhx_i3c_remove(struct i3c_device *i3cdev)
{
	st_asm330lhhx_remove(&i3cdev->dev);
	i3c_device_disable_ibi(i3cdev);
	i3c_device_free_ibi(i3cdev);
}
#else /* LINUX_VERSION_CODE */
static int st_asm330lhhx_i3c_remove(struct i3c_device *i3cdev)
{
	st_asm330lhhx_remove(&i3cdev->dev);
	i3c_device_disable_ibi(i3cdev);
	i3c_device_free_ibi(i3cdev);

	return 0;
}
#endif /* LINUX_VERSION_CODE */

static struct i3c_driver st_asm330lhhx_driver = {
	.driver = {
		.name = "st_asm330lhhx_i3c",
		.pm = &st_asm330lhhx_pm_ops,
	},
	.probe = st_asm330lhhx_i3c_probe,
	.remove = st_asm330lhhx_i3c_remove,
	.id_table = st_asm330lhhx_i3c_ids,
};
module_i3c_driver(st_asm330lhhx_driver);

MODULE_AUTHOR("MEMS Software Solutions Team");
MODULE_DESCRIPTION("STMicroelectronics st_asm330lhhx i3c driver");
MODULE_LICENSE("GPL v2");
