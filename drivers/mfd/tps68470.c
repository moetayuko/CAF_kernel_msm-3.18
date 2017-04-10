/*
 * TPS68470 chip family multi-function driver
 *
 * Copyright (C) 2017 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps68470.h>
#include <linux/module.h>
#include <linux/regmap.h>

static const struct mfd_cell tps68470s[] = {
	{
		.name = "tps68470-gpio",
	},
	{
		.name = "tps68470_pmic_opregion",
	},
};

static const struct regmap_config tps68470_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = TPS68470_REG_MAX,
};

static int tps68470_chip_init(struct device *dev, struct regmap *regmap)
{
	unsigned int version;
	int ret;

	ret = regmap_read(regmap, TPS68470_REG_REVID, &version);
	if (ret < 0) {
		dev_err(dev, "Failed to read revision register: %d\n", ret);
		return ret;
	}

	ret = regmap_write(regmap, TPS68470_REG_RESET, 0xff);
	if (ret < 0)
		return ret;

	/* FIXME: configure these dynamically */
	/* Enable Daisy Chain LDO and configure relevant GPIOs as output */
	ret = regmap_write(regmap, TPS68470_REG_S_I2C_CTL, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, TPS68470_REG_GPCTL4A, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, TPS68470_REG_GPCTL5A, 2);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, TPS68470_REG_GPCTL6A, 2);
	if (ret < 0)
		return ret;

	/*
	 * When SDA and SCL are routed to GPIO1 and GPIO2, the mode
	 * for these GPIOs must be configured using their respective
	 * GPCTLxA registers as inputs with no pull-ups.
	 */
	ret = regmap_write(regmap, TPS68470_REG_GPCTL1A, 0);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, TPS68470_REG_GPCTL2A, 0);
	if (ret < 0)
		return ret;

	/* Enable daisy chain */
	ret = regmap_update_bits(regmap, TPS68470_REG_S_I2C_CTL, 1, 1);
	if (ret < 0)
		return ret;

	/* Typical PLL startup time is 1 ms */
	usleep_range(TPS68470_DAISY_CHAIN_DELAY_US,
			TPS68470_DAISY_CHAIN_DELAY_US + 10);

	dev_info(dev, "TPS68470 REVID: 0x%x\n", version);

	return 0;
}

static int tps68470_probe(struct i2c_client *client,
			  const struct i2c_device_id *ids)
{
	struct device *dev = &client->dev;
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(client, &tps68470_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "devm_regmap_init_i2c Error %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	i2c_set_clientdata(client, regmap);
	ret = mfd_add_devices(dev, PLATFORM_DEVID_NONE, tps68470s,
			      ARRAY_SIZE(tps68470s), NULL, 0, NULL);
	if (ret < 0) {
		dev_err(dev, "mfd_add_devices failed: %d\n", ret);
		return ret;
	}

	ret = tps68470_chip_init(dev, regmap);
	if (ret < 0) {
		dev_err(dev, "TPS68470 Init Error %d\n", ret);
		return ret;
	}

	return 0;
}

static int tps68470_remove(struct i2c_client *client)
{

	mfd_remove_devices(&client->dev);

	return 0;
}

static const struct i2c_device_id tps68470_id_table[] = {
	{},
};

MODULE_DEVICE_TABLE(i2c, tps68470_id_table);

static const struct acpi_device_id tps68470_acpi_ids[] = {
	{"INT3472"},
	{},
};

MODULE_DEVICE_TABLE(acpi, tps68470_acpi_ids);

static struct i2c_driver tps68470_driver = {
	.driver = {
		   .name = "tps68470",
		   .acpi_match_table = tps68470_acpi_ids,
	},
	.id_table = tps68470_id_table,
	.probe = tps68470_probe,
	.remove = tps68470_remove,
};
module_i2c_driver(tps68470_driver);

MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Jian Xu Zheng <jian.xu.zheng@intel.com>");
MODULE_AUTHOR("Yuning Pu <yuning.pu@intel.com>");
MODULE_AUTHOR("Rajmohan Mani <rajmohan.mani@intel.com>");
MODULE_DESCRIPTION("TPS68470 chip family multi-function driver");
MODULE_LICENSE("GPL v2");
