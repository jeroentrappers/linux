// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI SN2400 USB charger driver
 *
 * The SN2400 is a USB battery charger made by TI for Apple and used in the
 * iPhone 6. There is no public datasheet. The chip charges on its own once
 * it has an input current limit, so the driver only programs the input and
 * charge currents and reads the input voltage and current from the ADC.
 *
 * When VBUS sags the chip lowers the input current limit by itself, and
 * after a plug it starts again from its 90 mA minimum. The driver puts the
 * requested limit back once VBUS is above 4.5 V again: right after a cable
 * event when the interrupt line is wired up, and from a 5 s poll in any
 * case.
 *
 * Copyright (C) 2026 Jeroen Trappers <jeroen.trappers@gmail.com>
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/container_of.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#define SN2400_EVENT_MASK(n)	(0x01 + (n))	/* 1 = masked */
#define SN2400_EVENT(n)		(0x04 + (n))	/* clear on read */
#define SN2400_NUM_EVENT_REGS	3
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

#define SN2400_ADC_CHAN		GENMASK(3, 1)
#define SN2400_ADC_START	BIT(4)
#define SN2400_ADC_VBUS		0
#define SN2400_ADC_IBUS		1

/*
 * Events signalled on the interrupt line: event 1 bits 2 and 4 (adapter
 * change), event 1 bits 6 and 7 and event 3 bit 0 (VBUS and input limit
 * change). Event 3 bit 2 (input power limited) stays masked, it fires
 * continuously while VBUS sags.
 */
#define SN2400_EV1_IRQS		(BIT(2) | BIT(4) | BIT(6) | BIT(7))
#define SN2400_EV2_IRQS		0
#define SN2400_EV3_IRQS		BIT(0)
#define SN2400_IRQ_MAX_PER_SEC	20

/* Currents in uA, voltages in uV */
#define SN2400_INPUT_MIN	90000
#define SN2400_INPUT_MAX	2000000
#define SN2400_INPUT_STEP	10000
#define SN2400_CHG_MAX		2550000
#define SN2400_CHG_STEP		10000
#define SN2400_CHG_VOLTAGE_MIN	3000000
#define SN2400_CHG_VOLTAGE_STEP	10000
#define SN2400_VBUS_STEP	25000
#define SN2400_IBUS_STEP	7800
#define SN2400_VBUS_ONLINE	4000000
#define SN2400_VBUS_RESTORE	4500000

#define SN2400_DEF_INPUT	500000
#define SN2400_DEF_CHG		1000000
#define SN2400_POLL_MS		5000
#define SN2400_VBUS_SETTLE_MS	500

struct sn2400 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work work;
	/* Serialises ADC conversions and the fields below */
	struct mutex lock;
	u32 input_limit;	/* requested, 0 = input suspended */
	u32 chg_current;
	bool suspended;
	bool cable_event;	/* report a change from the next work run */
	/* Interrupt storm detection, only used by the IRQ thread */
	u8 irq_mask[SN2400_NUM_EVENT_REGS];
	unsigned long irq_window;
	unsigned int irq_count;
};

static bool sn2400_precious_reg(struct device *dev, unsigned int reg)
{
	return reg >= SN2400_EVENT(0) &&
	       reg < SN2400_EVENT(SN2400_NUM_EVENT_REGS);
}

static const struct regmap_config sn2400_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SN2400_MAX_REGISTER,
	.precious_reg = sn2400_precious_reg,
	.cache_type = REGCACHE_NONE,
};

static int sn2400_read_adc(struct sn2400 *sn, unsigned int chan,
			   unsigned int *val)
{
	unsigned int ctrl;
	int ret;

	ret = regmap_write(sn->regmap, SN2400_ADC_CTRL,
			   FIELD_PREP(SN2400_ADC_CHAN, chan) | SN2400_ADC_START);
	if (ret)
		return ret;

	/* START clears once the conversion is done */
	ret = regmap_read_poll_timeout(sn->regmap, SN2400_ADC_CTRL, ctrl,
				       !(ctrl & SN2400_ADC_START),
				       1000, 50000);
	if (ret)
		return ret;

	return regmap_read(sn->regmap, SN2400_ADC_DATA, val);
}

static int sn2400_get_vbus(struct sn2400 *sn, int *uv)
{
	unsigned int val;
	int ret;

	ret = sn2400_read_adc(sn, SN2400_ADC_VBUS, &val);
	if (ret)
		return ret;

	*uv = val * SN2400_VBUS_STEP;

	return 0;
}

static unsigned int sn2400_input_limit_code(u32 ua)
{
	ua = clamp(ua, SN2400_INPUT_MIN, SN2400_INPUT_MAX);

	return (ua - SN2400_INPUT_MIN) / SN2400_INPUT_STEP;
}

/* A limit of 0 suspends the input */
static int sn2400_set_input_limit(struct sn2400 *sn, u32 ua)
{
	int ret;

	if (!ua) {
		ret = regmap_write(sn->regmap, SN2400_INPUT_LIMIT, 0);
		if (ret)
			return ret;

		return regmap_write(sn->regmap, SN2400_CONFIG,
				    SN2400_CONFIG_NORMAL | SN2400_CONFIG_SUSPEND);
	}

	ret = regmap_write(sn->regmap, SN2400_CONFIG, SN2400_CONFIG_NORMAL);
	if (ret)
		return ret;

	return regmap_write(sn->regmap, SN2400_INPUT_LIMIT,
			    sn2400_input_limit_code(ua));
}

static int sn2400_set_chg_current(struct sn2400 *sn, u32 ua)
{
	ua = min(ua, SN2400_CHG_MAX);

	return regmap_write(sn->regmap, SN2400_CHG_CURRENT,
			    ua / SN2400_CHG_STEP);
}

static void sn2400_work(struct work_struct *work)
{
	struct sn2400 *sn = container_of(work, struct sn2400, work.work);
	unsigned int code;
	bool changed;
	int vbus;

	mutex_lock(&sn->lock);

	changed = sn->cable_event;
	sn->cable_event = false;

	if (!sn->input_limit)
		goto out;

	if (regmap_read(sn->regmap, SN2400_INPUT_LIMIT, &code) ||
	    code >= sn2400_input_limit_code(sn->input_limit))
		goto out;

	if (sn2400_get_vbus(sn, &vbus) || vbus < SN2400_VBUS_RESTORE) {
		dev_dbg(sn->dev, "input limit reduced to %u uA\n",
			SN2400_INPUT_MIN + code * SN2400_INPUT_STEP);
		goto out;
	}

	dev_dbg(sn->dev, "restoring input limit\n");
	if (!sn2400_set_input_limit(sn, sn->input_limit))
		changed = true;

out:
	mutex_unlock(&sn->lock);

	if (changed)
		power_supply_changed(sn->psy);

	schedule_delayed_work(&sn->work, msecs_to_jiffies(SN2400_POLL_MS));
}

/*
 * The interrupt line is level triggered and stays asserted until the events
 * have been read, which also clears them, so the events are always read,
 * also while the system is suspending. A source that keeps firing gets all
 * events masked.
 */
static irqreturn_t sn2400_irq(int irq, void *data)
{
	struct sn2400 *sn = data;
	u8 ev[SN2400_NUM_EVENT_REGS];
	bool active = false;
	int i, ret;

	ret = regmap_bulk_read(sn->regmap, SN2400_EVENT(0), ev, sizeof(ev));
	if (ret) {
		dev_err_ratelimited(sn->dev, "failed to read events: %d\n", ret);
		return IRQ_NONE;
	}

	dev_dbg(sn->dev, "events %*ph\n", (int)sizeof(ev), ev);

	for (i = 0; i < SN2400_NUM_EVENT_REGS; i++)
		if (ev[i] & ~sn->irq_mask[i])
			active = true;

	if (!active)
		return IRQ_HANDLED;

	if (time_after(jiffies, sn->irq_window + HZ)) {
		sn->irq_window = jiffies;
		sn->irq_count = 0;
	}

	if (++sn->irq_count > SN2400_IRQ_MAX_PER_SEC) {
		dev_err(sn->dev, "interrupt storm (events %*ph), masking all events\n",
			(int)sizeof(ev), ev);
		memset(sn->irq_mask, 0xff, sizeof(sn->irq_mask));
		/* Clear what latched before the mask took effect */
		if (!regmap_bulk_write(sn->regmap, SN2400_EVENT_MASK(0),
				       sn->irq_mask, sizeof(sn->irq_mask)))
			regmap_bulk_read(sn->regmap, SN2400_EVENT(0), ev,
					 sizeof(ev));
	}

	/* Let VBUS settle, then check the input limit and report the change */
	mutex_lock(&sn->lock);
	sn->cable_event = true;
	if (!sn->suspended)
		mod_delayed_work(system_percpu_wq, &sn->work,
				 msecs_to_jiffies(SN2400_VBUS_SETTLE_MS));
	mutex_unlock(&sn->lock);

	return IRQ_HANDLED;
}

static int sn2400_init_irq(struct sn2400 *sn, int irq)
{
	struct device *dev = sn->dev;
	int ret;

	sn->irq_mask[0] = 0xff & ~SN2400_EV1_IRQS;
	sn->irq_mask[1] = 0xff & ~SN2400_EV2_IRQS;
	sn->irq_mask[2] = 0xff & ~SN2400_EV3_IRQS;
	sn->irq_window = jiffies;

	ret = devm_request_threaded_irq(dev, irq, NULL, sn2400_irq,
					IRQF_ONESHOT, dev_name(dev), sn);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	ret = regmap_bulk_write(sn->regmap, SN2400_EVENT_MASK(0), sn->irq_mask,
				sizeof(sn->irq_mask));
	if (ret)
		return dev_err_probe(dev, ret, "failed to unmask events\n");

	return 0;
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
	int ret, uv;

	mutex_lock(&sn->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = sn2400_get_vbus(sn, &uv);
		if (!ret)
			val->intval = uv >= SN2400_VBUS_ONLINE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sn2400_get_vbus(sn, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sn2400_read_adc(sn, SN2400_ADC_IBUS, &reg);
		if (!ret)
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
		if (!ret)
			val->intval = SN2400_INPUT_MIN +
				      reg * SN2400_INPUT_STEP;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(sn->regmap, SN2400_CHG_CURRENT_RB, &reg);
		if (!ret)
			val->intval = reg * SN2400_CHG_STEP;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = regmap_read(sn->regmap, SN2400_CHG_VOLTAGE, &reg);
		if (!ret)
			val->intval = SN2400_CHG_VOLTAGE_MIN +
				      reg * SN2400_CHG_VOLTAGE_STEP;
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
	static const u8 mask_all[SN2400_NUM_EVENT_REGS] = { 0xff, 0xff, 0xff };
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	u8 events[SN2400_NUM_EVENT_REGS];
	struct sn2400 *sn;
	int ret;

	sn = devm_kzalloc(dev, sizeof(*sn), GFP_KERNEL);
	if (!sn)
		return -ENOMEM;

	sn->dev = dev;
	i2c_set_clientdata(client, sn);

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

	/* Mask all events and clear the pending ones */
	ret = regmap_bulk_write(sn->regmap, SN2400_EVENT_MASK(0), mask_all,
				sizeof(mask_all));
	if (ret)
		return dev_err_probe(dev, ret, "failed to mask events\n");

	ret = regmap_bulk_read(sn->regmap, SN2400_EVENT(0), events,
			       sizeof(events));
	if (ret)
		return dev_err_probe(dev, ret, "failed to clear events\n");

	ret = sn2400_set_chg_current(sn, sn->chg_current);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set charge current\n");

	ret = sn2400_set_input_limit(sn, sn->input_limit);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set input limit\n");

	psy_cfg.drv_data = sn;
	psy_cfg.fwnode = dev_fwnode(dev);
	sn->psy = devm_power_supply_register(dev, &sn2400_desc, &psy_cfg);
	if (IS_ERR(sn->psy))
		return dev_err_probe(dev, PTR_ERR(sn->psy),
				     "failed to register power supply\n");

	ret = devm_delayed_work_autocancel(dev, &sn->work, sn2400_work);
	if (ret)
		return ret;

	memset(sn->irq_mask, 0xff, sizeof(sn->irq_mask));
	if (client->irq > 0) {
		ret = sn2400_init_irq(sn, client->irq);
		if (ret)
			return ret;
	}

	schedule_delayed_work(&sn->work, msecs_to_jiffies(SN2400_POLL_MS));

	return 0;
}

/*
 * The work item is not freezable and would otherwise run with the I2C
 * controller already suspended. Stop it across system suspend and check the
 * input limit right after resume, as a plug while asleep leaves the chip at
 * its minimum.
 */
static int sn2400_suspend(struct device *dev)
{
	struct sn2400 *sn = dev_get_drvdata(dev);

	mutex_lock(&sn->lock);
	sn->suspended = true;
	mutex_unlock(&sn->lock);

	cancel_delayed_work_sync(&sn->work);

	return 0;
}

static int sn2400_resume(struct device *dev)
{
	struct sn2400 *sn = dev_get_drvdata(dev);

	mutex_lock(&sn->lock);
	sn->suspended = false;
	mutex_unlock(&sn->lock);

	mod_delayed_work(system_percpu_wq, &sn->work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sn2400_pm_ops, sn2400_suspend, sn2400_resume);

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
		.pm = pm_sleep_ptr(&sn2400_pm_ops),
	},
	.probe = sn2400_probe,
	.id_table = sn2400_id,
};
module_i2c_driver(sn2400_driver);

MODULE_AUTHOR("Jeroen Trappers <jeroen.trappers@gmail.com>");
MODULE_DESCRIPTION("TI SN2400 USB charger driver");
MODULE_LICENSE("GPL");
