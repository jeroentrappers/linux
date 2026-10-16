// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI SN2400 USB charger driver
 *
 * The SN2400 ("Tigris") is a TI charger made for Apple and used in the
 * iPhone 6. There is no public datasheet; the register layout below was
 * recovered from the iOS 12.5.5 driver. The chip charges on its own once
 * it has an input current limit, so the driver only programs the input
 * and charge current and reads back the ADC.
 *
 * The interrupt line goes through the PMU, which has no driver yet, so
 * the input limit is checked by polling instead.
 */

#include <linux/bits.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define SN2400_EVENT_MASK1	0x01	/* 0x01-0x03, 1 = masked */
#define SN2400_EVENT1		0x04	/* 0x04-0x06 */
#define SN2400_INPUT_LIMIT	0x09
#define SN2400_CHG_CURRENT	0x0d
#define SN2400_CHG_CURRENT_RB	0x0e
#define SN2400_CHG_VOLTAGE	0x0f
#define SN2400_CONFIG		0x10
#define SN2400_ADC_CTRL		0x11
#define SN2400_ADC_DATA		0x12
#define SN2400_MAX_REGISTER	0x1d

#define SN2400_CONFIG_NORMAL	BIT(2)
#define SN2400_CONFIG_SUSPEND	BIT(3)
#define SN2400_ADC_START	BIT(4)

#define SN2400_ADC_VBUS		0
#define SN2400_ADC_IBUS		2

/* All limits in uA / uV */
#define SN2400_INPUT_MIN	90000
#define SN2400_INPUT_MAX	2000000
#define SN2400_INPUT_STEP	10000
#define SN2400_CHG_MAX		2550000
#define SN2400_CHG_STEP		10000
#define SN2400_VBUS_STEP	25000
#define SN2400_IBUS_STEP	7800
#define SN2400_VBUS_ONLINE	4000000
#define SN2400_VBUS_LIMIT	4500000

#define SN2400_DEF_INPUT	500000
#define SN2400_DEF_CHG		1000000
#define SN2400_POLL_MS		5000

struct sn2400 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work work;
	struct mutex lock;	/* serialises ADC use and the limits */
	u32 input_limit;	/* requested, 0 = input suspended */
	u32 chg_current;
};

static const struct regmap_config sn2400_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SN2400_MAX_REGISTER,
	.cache_type = REGCACHE_NONE,
};

static int sn2400_read_adc(struct sn2400 *sn, unsigned int chan,
			   unsigned int *val)
{
	unsigned int ctrl;
	int ret;

	ret = regmap_write(sn->regmap, SN2400_ADC_CTRL,
			   (chan & 0xe) | SN2400_ADC_START);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(sn->regmap, SN2400_ADC_CTRL, ctrl,
				       !(ctrl & SN2400_ADC_START),
				       1000, 50000);
	if (ret)
		return ret;

	return regmap_read(sn->regmap, SN2400_ADC_DATA, val);
}

static int sn2400_vbus(struct sn2400 *sn, int *uv)
{
	unsigned int val;
	int ret;

	ret = sn2400_read_adc(sn, SN2400_ADC_VBUS, &val);
	if (!ret)
		*uv = val * SN2400_VBUS_STEP;

	return ret;
}

static int sn2400_set_input_limit(struct sn2400 *sn, u32 ua)
{
	unsigned int code;
	int ret;

	if (!ua) {
		ret = regmap_write(sn->regmap, SN2400_INPUT_LIMIT, 0);
		if (ret)
			return ret;
		return regmap_write(sn->regmap, SN2400_CONFIG,
				    SN2400_CONFIG_NORMAL | SN2400_CONFIG_SUSPEND);
	}

	ua = clamp(ua, SN2400_INPUT_MIN, SN2400_INPUT_MAX);
	code = (ua - SN2400_INPUT_MIN) / SN2400_INPUT_STEP;

	ret = regmap_write(sn->regmap, SN2400_CONFIG, SN2400_CONFIG_NORMAL);
	if (ret)
		return ret;

	return regmap_write(sn->regmap, SN2400_INPUT_LIMIT, code);
}

static int sn2400_set_chg_current(struct sn2400 *sn, u32 ua)
{
	ua = min(ua, SN2400_CHG_MAX);

	return regmap_write(sn->regmap, SN2400_CHG_CURRENT,
			    ua / SN2400_CHG_STEP);
}

/*
 * The chip lowers the input limit by itself when VBUS collapses. Put the
 * requested limit back once VBUS has recovered.
 */
static void sn2400_work(struct work_struct *work)
{
	struct sn2400 *sn = container_of(work, struct sn2400, work.work);
	unsigned int code, want;
	int vbus;

	mutex_lock(&sn->lock);

	if (!sn->input_limit)
		goto out;

	want = (clamp(sn->input_limit, SN2400_INPUT_MIN, SN2400_INPUT_MAX) -
		SN2400_INPUT_MIN) / SN2400_INPUT_STEP;

	if (regmap_read(sn->regmap, SN2400_INPUT_LIMIT, &code) ||
	    code >= want)
		goto out;

	if (sn2400_vbus(sn, &vbus) || vbus < SN2400_VBUS_LIMIT) {
		dev_dbg(sn->dev, "input limit reduced to %u mA\n",
			90 + 10 * code);
		goto out;
	}

	dev_dbg(sn->dev, "restoring input limit\n");
	sn2400_set_input_limit(sn, sn->input_limit);
	power_supply_changed(sn->psy);

out:
	mutex_unlock(&sn->lock);
	schedule_delayed_work(&sn->work, msecs_to_jiffies(SN2400_POLL_MS));
}

static const enum power_supply_property sn2400_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static int sn2400_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct sn2400 *sn = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret, uv = 0;

	mutex_lock(&sn->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = sn2400_vbus(sn, &uv);
		val->intval = uv >= SN2400_VBUS_ONLINE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sn2400_vbus(sn, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sn2400_read_adc(sn, SN2400_ADC_IBUS, &reg);
		val->intval = reg * SN2400_IBUS_STEP;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(sn->regmap, SN2400_CONFIG, &reg);
		if (ret)
			break;
		if (reg & SN2400_CONFIG_SUSPEND) {
			val->intval = 0;
			break;
		}
		ret = regmap_read(sn->regmap, SN2400_INPUT_LIMIT, &reg);
		val->intval = SN2400_INPUT_MIN + reg * SN2400_INPUT_STEP;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(sn->regmap, SN2400_CHG_CURRENT_RB, &reg);
		val->intval = reg * SN2400_CHG_STEP;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = regmap_read(sn->regmap, SN2400_CHG_VOLTAGE, &reg);
		val->intval = 3000000 + reg * 10000;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&sn->lock);

	return ret;
}

static int sn2400_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	struct sn2400 *sn = power_supply_get_drvdata(psy);
	int ret;

	if (val->intval < 0)
		return -EINVAL;

	mutex_lock(&sn->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sn2400_set_input_limit(sn, val->intval);
		if (!ret)
			sn->input_limit = val->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sn2400_set_chg_current(sn, val->intval);
		if (!ret)
			sn->chg_current = val->intval;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&sn->lock);

	return ret;
}

static int sn2400_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
	       psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
}

static const struct power_supply_desc sn2400_desc = {
	.name = "sn2400-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sn2400_props,
	.num_properties = ARRAY_SIZE(sn2400_props),
	.get_property = sn2400_get_property,
	.set_property = sn2400_set_property,
	.property_is_writeable = sn2400_property_is_writeable,
};

static int sn2400_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	static const u8 mask[3] = { 0xff, 0xff, 0xff };
	u8 events[3];
	struct sn2400 *sn;
	int ret;

	sn = devm_kzalloc(dev, sizeof(*sn), GFP_KERNEL);
	if (!sn)
		return -ENOMEM;

	sn->dev = dev;
	sn->input_limit = SN2400_DEF_INPUT;
	sn->chg_current = SN2400_DEF_CHG;
	device_property_read_u32(dev, "input-current-limit-microamp",
				 &sn->input_limit);
	device_property_read_u32(dev, "constant-charge-current-max-microamp",
				 &sn->chg_current);

	ret = devm_mutex_init(dev, &sn->lock);
	if (ret)
		return ret;

	sn->regmap = devm_regmap_init_i2c(client, &sn2400_regmap);
	if (IS_ERR(sn->regmap))
		return dev_err_probe(dev, PTR_ERR(sn->regmap),
				     "failed to init regmap\n");

	/* No interrupt yet: mask all events and clear pending ones */
	ret = regmap_bulk_write(sn->regmap, SN2400_EVENT_MASK1, mask,
				sizeof(mask));
	if (ret)
		return dev_err_probe(dev, ret, "failed to mask events\n");

	ret = regmap_bulk_read(sn->regmap, SN2400_EVENT1, events,
			       sizeof(events));
	if (ret)
		return ret;

	ret = sn2400_set_chg_current(sn, sn->chg_current);
	if (ret)
		return ret;

	ret = sn2400_set_input_limit(sn, sn->input_limit);
	if (ret)
		return ret;

	psy_cfg.drv_data = sn;
	psy_cfg.fwnode = dev_fwnode(dev);
	sn->psy = devm_power_supply_register(dev, &sn2400_desc, &psy_cfg);
	if (IS_ERR(sn->psy))
		return dev_err_probe(dev, PTR_ERR(sn->psy),
				     "failed to register power supply\n");

	ret = devm_delayed_work_autocancel(dev, &sn->work, sn2400_work);
	if (ret)
		return ret;

	schedule_delayed_work(&sn->work, msecs_to_jiffies(SN2400_POLL_MS));

	dev_info(dev, "input limit %u mA, charge current %u mA\n",
		 sn->input_limit / 1000, sn->chg_current / 1000);

	return 0;
}

static const struct of_device_id sn2400_of_match[] = {
	{ .compatible = "ti,sn2400" },
	{ }
};
MODULE_DEVICE_TABLE(of, sn2400_of_match);

static const struct i2c_device_id sn2400_id[] = {
	{ "sn2400" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn2400_id);

static struct i2c_driver sn2400_driver = {
	.driver = {
		.name = "sn2400-charger",
		.of_match_table = sn2400_of_match,
	},
	.probe = sn2400_probe,
	.id_table = sn2400_id,
};
module_i2c_driver(sn2400_driver);

MODULE_DESCRIPTION("TI SN2400 USB charger driver");
MODULE_LICENSE("GPL");
