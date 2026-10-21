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
 *
 * The rails of the M8 motion coprocessor ("Oscar") are read-only unless
 * oscar_rails is set: they are only switched during its bring-up, by
 * userspace through regulator-output consumers.
 */

#include <linux/bits.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>

static bool oscar_rails;
module_param(oscar_rails, bool, 0644);
MODULE_PARM_DESC(oscar_rails, "Allow switching the motion coprocessor rails");

struct apple_pmic_ldo_data {
	const char *name;
	unsigned int enable_mask;
	bool gated;
};

static const struct regulator_ops apple_pmic_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static int apple_pmic_ldo_gated_enable(struct regulator_dev *rdev)
{
	if (!oscar_rails)
		return -EPERM;

	return regulator_enable_regmap(rdev);
}

static int apple_pmic_ldo_gated_disable(struct regulator_dev *rdev)
{
	if (!oscar_rails)
		return -EPERM;

	return regulator_disable_regmap(rdev);
}

static const struct regulator_ops apple_pmic_ldo_gated_ops = {
	.enable = apple_pmic_ldo_gated_enable,
	.disable = apple_pmic_ldo_gated_disable,
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
	desc->ops = data->gated ? &apple_pmic_ldo_gated_ops : &apple_pmic_ldo_ops;
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

/*
 * D2186 enable bits, from the iOS PMU rail table: register 0x0082 holds
 * ldo4 in bit 4, register 0x0085 ldo14..ldo16 in bits 0..2 and ldo17/ldo18
 * in bits 4/5.
 */
static const struct apple_pmic_ldo_data adi_pmic_ldo4 = {
	.name = "ldo4",
	.enable_mask = BIT(4),
	.gated = true,
};

static const struct apple_pmic_ldo_data adi_pmic_ldo15 = {
	.name = "ldo15",
	.enable_mask = BIT(1),
};

static const struct apple_pmic_ldo_data adi_pmic_ldo16 = {
	.name = "ldo16",
	.enable_mask = BIT(2),
	.gated = true,
};

static const struct apple_pmic_ldo_data adi_pmic_ldo18 = {
	.name = "ldo18",
	.enable_mask = BIT(5),
	.gated = true,
};

static const struct of_device_id apple_pmic_regulator_of_match[] = {
	{ .compatible = "apple,adi-pmic-ldo4", .data = &adi_pmic_ldo4 },
	{ .compatible = "apple,adi-pmic-ldo15", .data = &adi_pmic_ldo15 },
	{ .compatible = "apple,adi-pmic-ldo16", .data = &adi_pmic_ldo16 },
	{ .compatible = "apple,adi-pmic-ldo18", .data = &adi_pmic_ldo18 },
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
