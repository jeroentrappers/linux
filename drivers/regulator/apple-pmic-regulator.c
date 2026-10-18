// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple PMIC regulator driver
 *
 * Enable-only LDO switches in the Dialog based PMICs of Apple A8 devices.
 * The PMIC is described by an apple,i2c-pmic MFD, which provides the 16 bit
 * register map shared by its children. The register that holds the enable
 * bit comes from the "reg" property, the bit itself from the compatible.
 *
 * Only the enable bit is touched and no voltage is programmed: the rails keep
 * the voltage the bootloader set up.
 */

#include <linux/bits.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>

struct apple_pmic_ldo_data {
	const char *name;
	unsigned int enable_mask;
};

static const struct regulator_ops apple_pmic_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static int apple_pmic_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct apple_pmic_ldo_data *data;
	struct regulator_config config = { };
	struct regulator_desc *desc;
	struct regulator_dev *rdev;
	u32 reg;
	int ret;

	data = device_get_match_data(dev);

	ret = of_property_read_u32_index(dev->of_node, "reg", 0, &reg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read reg\n");

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->name = data->name;
	desc->of_match = data->name;
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;
	desc->ops = &apple_pmic_ldo_ops;
	desc->enable_reg = reg;
	desc->enable_mask = data->enable_mask;

	config.dev = dev;
	config.regmap = dev_get_regmap(dev->parent, NULL);
	if (!config.regmap)
		return dev_err_probe(dev, -ENODEV, "parent has no regmap\n");

	rdev = devm_regulator_register(dev, desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "failed to register %s\n", data->name);

	return 0;
}

/* D2186: register 0x0085 holds the enables of ldo14..ldo18 in bits 0..4 */
static const struct apple_pmic_ldo_data adi_pmic_ldo15 = {
	.name = "ldo15",
	.enable_mask = BIT(1),
};

static const struct of_device_id apple_pmic_regulator_of_match[] = {
	{ .compatible = "apple,adi-pmic-ldo15", .data = &adi_pmic_ldo15 },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_pmic_regulator_of_match);

static struct platform_driver apple_pmic_regulator_driver = {
	.probe = apple_pmic_regulator_probe,
	.driver = {
		.name = "apple-pmic-regulator",
		.of_match_table = apple_pmic_regulator_of_match,
	},
};
module_platform_driver(apple_pmic_regulator_driver);

MODULE_DESCRIPTION("Apple PMIC regulator driver");
MODULE_LICENSE("GPL");
