// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: sensor power-up (stage 8 prologue, sens_pwr=1).
 *
 * iOS power_on_hardware (kext 0x845d2d4) powers the sensors from the AP before the
 * fw probes them at START (re/CAMERA-STREAM.md §1):
 *   cam_avdd_ldo GPIO 198 = 1, 1 ms; ISPC ref clocks (PMGR 0x20e010218 + 4*n:
 *   (old & 0xfcffffc0) | srcSel << 24 | cfg, wait for bit 30 to clear); per sensor:
 *   extclk pin (191 rear, 197 front) to its alternate function, shutdown pin
 *   (190 rear, 196 front) = 1 (active high), 5 ms.
 * GPIO outputs go through gpiolib (the pinctrl driver owns the pins); the extclk
 * pin function is a direct read-modify-write of the pin register (PERIPH, bits 6:5),
 * restored on power-off. Everything is undone in reverse.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "isp.h"

#define PINCTRL_PHYS		0x20e300000ULL
#define PIN_PERIPH		GENMASK(6, 5)
#define REFCLK_PHYS		0x20e010218ULL	/* kRegGroupISPRef0ClkCfg: AppleT7000PMGR 0x8537a38 */
#define REFCLK_BUSY		BIT(30)
#define REFCLK_KEEP		0xfcffffc0	/* configISPRefClock 0x852b49c */

#define PIN_AVDD		198
#define PIN_SHUT_REAR		190
#define PIN_SHUT_FRONT		196
#define PIN_CLK_REAR		191
#define PIN_CLK_FRONT		197

static const u8 isp_log_pins[] = { 190, 191, 196, 197, 198 };

static void isp_pins_log(struct t7000_isp *isp, const char *when)
{
	u32 v[ARRAY_SIZE(isp_log_pins)];
	int i;

	if (!isp->pinregs)
		return;
	for (i = 0; i < ARRAY_SIZE(isp_log_pins); i++)
		v[i] = readl(isp->pinregs + 4 * isp_log_pins[i]);
	dev_info(isp->dev, "sensor pins %s: 190 %08x 191 %08x 196 %08x 197 %08x 198 %08x; refclk %08x %08x\n",
		 when, v[0], v[1], v[2], v[3], v[4],
		 isp->refclk ? readl(isp->refclk) : 0, isp->refclk ? readl(isp->refclk + 4) : 0);
}

/* gpiolib number of an AP GPIO (ADT phandle 24 = pinctrl@20e300000) */
static int isp_gpio_base(struct t7000_isp *isp)
{
	struct device_node *np;
	struct gpio_device *gdev = NULL;
	struct resource res;
	int base = -ENODEV;

	for_each_compatible_node(np, NULL, "apple,pinctrl") {
		if (of_address_to_resource(np, 0, &res) || res.start != PINCTRL_PHYS)
			continue;
		gdev = gpio_device_find_by_fwnode(of_fwnode_handle(np));
		of_node_put(np);
		break;
	}
	if (gdev) {
		base = gpio_device_get_base(gdev);
		gpio_device_put(gdev);
	}
	return base;
}

static int isp_pin_out(struct t7000_isp *isp, int idx, int pin, const char *name)
{
	int ret = gpio_request_one(isp->gpio_base + pin, GPIOF_OUT_INIT_HIGH, name);

	if (ret) {
		dev_err(isp->dev, "sens_pwr: gpio %d (%s) request failed: %d\n", pin, name, ret);
		return ret;
	}
	isp->gpio_on[idx] = pin;
	dev_info(isp->dev, "sens_pwr: gpio %d (%s) = 1, reads %d\n", pin, name,
		 gpio_get_value(isp->gpio_base + pin));
	return 0;
}

static int isp_refclk_set(struct t7000_isp *isp, int n)
{
	void __iomem *r = isp->refclk + 4 * n;
	u32 old = readl(r), v;
	int ret;

	isp->refclk_old[n] = old;
	isp->refclk_saved |= BIT(n);
	writel((old & REFCLK_KEEP) | (isp_refclk_cfg & 0x3f) | ((isp_refclk_cfg >> 6) & 3) << 24, r);
	ret = readl_poll_timeout(r, v, !(v & REFCLK_BUSY), 10, 10000);
	dev_info(isp->dev, "sens_pwr: ref clock %d (PMGR +0x%x): 0x%08x -> 0x%08x%s\n", n,
		 0x10218 + 4 * n, old, v, ret ? " (TIMEOUT: bit 30 still set after 10 ms)" : "");
	return ret;
}

static void isp_extclk_set(struct t7000_isp *isp, int n, int pin)
{
	void __iomem *r = isp->pinregs + 4 * pin;
	u32 old = readl(r);

	isp->extclk_old[n] = old;
	isp->extclk_saved |= BIT(n);
	writel((old & ~PIN_PERIPH) | FIELD_PREP(PIN_PERIPH, isp_extclk_periph), r);
	dev_info(isp->dev, "sens_pwr: extclk pin %d: 0x%08x -> 0x%08x (periph %d)\n", pin, old,
		 readl(r), isp_extclk_periph);
}

int isp_sensor_power_on(struct t7000_isp *isp)
{
	int ret, n;

	if (!isp_sens_pwr) {
		dev_info(isp->dev, "sens_pwr=0: not touching the sensor rails/pins\n");
		return 0;
	}
	isp->gpio_base = isp_gpio_base(isp);
	if (isp->gpio_base < 0) {
		dev_err(isp->dev, "sens_pwr: pinctrl@20e300000 gpio chip not found (%d)\n",
			isp->gpio_base);
		return -ENODEV;
	}
	isp->sens_on = true;	/* from here on isp_sensor_power_off() undoes it */
	isp->pinregs = ioremap(PINCTRL_PHYS, 0x1000);
	isp->refclk = ioremap(REFCLK_PHYS, 8);
	if (!isp->pinregs || !isp->refclk)
		return -ENOMEM;
	isp_pins_log(isp, "before");

	ret = isp_pin_out(isp, 0, PIN_AVDD, "isp-cam-avdd");
	if (ret)
		return ret;
	msleep(2);				/* iOS: IOSleep(1) after the rails */
	if (isp_refclk)
		for (n = 0; n < 2; n++)
			if (isp_refclk_set(isp, n))
				return -ETIMEDOUT;
	if (isp_extclk_periph >= 0 && isp_extclk_periph <= 3) {
		isp_extclk_set(isp, 0, PIN_CLK_REAR);
		isp_extclk_set(isp, 1, PIN_CLK_FRONT);
	}
	ret = isp_pin_out(isp, 1, PIN_SHUT_REAR, "isp-rear-shutdown");
	if (!ret)
		ret = isp_pin_out(isp, 2, PIN_SHUT_FRONT, "isp-front-shutdown");
	if (ret)
		return ret;
	msleep(10);				/* iOS: IOSleep(5) after EnableSensorN */
	isp_pins_log(isp, "after power-up");
	return 0;
}

static void isp_pin_release(struct t7000_isp *isp, int idx)
{
	if (!isp->gpio_on[idx])
		return;
	gpio_set_value(isp->gpio_base + isp->gpio_on[idx], 0);
	gpio_free(isp->gpio_base + isp->gpio_on[idx]);
	isp->gpio_on[idx] = 0;
}

/* Reverse order: shutdown pins low, extclk + ref clocks restored, AVDD off. */
void isp_sensor_power_off(struct t7000_isp *isp)
{
	int n;

	if (!isp->sens_on)
		return;
	isp_pin_release(isp, 2);
	isp_pin_release(isp, 1);
	for (n = 1; n >= 0; n--) {
		if (isp->extclk_saved & BIT(n))
			writel(isp->extclk_old[n],
			       isp->pinregs + 4 * (n ? PIN_CLK_FRONT : PIN_CLK_REAR));
		if (isp->refclk_saved & BIT(n))
			writel(isp->refclk_old[n], isp->refclk + 4 * n);
	}
	isp->extclk_saved = isp->refclk_saved = 0;
	isp_pin_release(isp, 0);
	isp_pins_log(isp, "after power-off");
	if (isp->refclk)
		iounmap(isp->refclk);
	if (isp->pinregs)
		iounmap(isp->pinregs);
	isp->refclk = isp->pinregs = NULL;
	isp->sens_on = false;
}
