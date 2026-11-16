// SPDX-License-Identifier: GPL-2.0-only
/*
 * EHCI glue for the Apple S5L8960X/T7000 HSIC host (iPhone 6: usb-ehci2, the
 * HSIC link to the baseband modem).
 *
 * The controller itself is a Synopsys EHCI; around it iOS does:
 *  - per-controller control register (usb-complex block, ehci2 at +0x18):
 *    clear bit 1 (clock/idle request), set bits 0-3 of the next word (cfg-incr)
 *  - EHCI+0xb0 (Synopsys INSNREG area) = mask of the HSIC ports
 *  - HSIC PHY block (usb-complex block +0x60): set bits 0,1, write the ADT
 *    "cfg" word at +0x8, clear bits 1,2,5, 20 us, clear bit 0, 1 ms
 *  - host_ready GPIO high once the port is up, then device_ready follows
 *    within 500 ms
 * The PHY power domain (hsic1_phy, child of usb2host2) is the device's domain.
 *
 * Bring-up is experimental, so the steps can be tuned with module
 * parameters instead of rebuilding.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ehci.h"

#define HSIC_INSNREG_PORTS	0xb0

#define CTL_GATE		0x0	/* bit 1: clock gate / idle request */
#define CTL_GATE_IDLE		BIT(1)
#define CTL_INCR		0x4	/* bits 0-3: AHB INCRx burst enables */

#define PHY_CTL			0x0
#define PHY_CFG			0x8

static int segment = -1;
module_param(segment, int, 0444);
MODULE_PARM_DESC(segment, "CTRLDSSEGMENT to force after start (-1: leave Linux's value; iOS uses 8)");

static bool phy_init = true;
module_param(phy_init, bool, 0444);
MODULE_PARM_DESC(phy_init, "run the HSIC PHY bring-up");

static unsigned int phy_cfg = 0x1f7;
module_param(phy_cfg, uint, 0444);
MODULE_PARM_DESC(phy_cfg, "HSIC PHY cfg word (ADT hsic-baseband cfg)");

static unsigned int port_mask = 0x1;
module_param(port_mask, uint, 0444);
MODULE_PARM_DESC(port_mask, "value for EHCI+0xb0 (HSIC port mask)");

struct apple_hsic {
	void __iomem *ctl;
	void __iomem *phy;
	struct gpio_desc *host_ready;
	struct gpio_desc *device_ready;
};

static struct hc_driver __read_mostly apple_hsic_hc_driver;

static void apple_hsic_dump(struct device *dev, struct usb_hcd *hcd, struct apple_hsic *h, const char *when)
{
	dev_info(dev, "%s: ctl %08x %08x phy %08x %08x %08x %08x hcs %08x hcc %08x b0 %08x\n", when,
		 readl(h->ctl), readl(h->ctl + 4),
		 readl(h->phy), readl(h->phy + 4), readl(h->phy + 8), readl(h->phy + 12),
		 readl(hcd->regs + 4), readl(hcd->regs + 8), readl(hcd->regs + HSIC_INSNREG_PORTS));
}

static int apple_hsic_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int ret;

	ehci->caps = hcd->regs;
	ret = ehci_setup(hcd);		/* halts + HCRESETs the controller */
	if (ret)
		return ret;
	/* write after HCRESET in case the reset clears it */
	writel(port_mask, hcd->regs + HSIC_INSNREG_PORTS);
	return 0;
}

static const struct ehci_driver_overrides apple_hsic_overrides __initconst = {
	.extra_priv_size = sizeof(struct apple_hsic),
	.reset = apple_hsic_reset,
};

static void apple_hsic_phy_on(struct apple_hsic *h)
{
	u32 v;

	writel(readl(h->ctl + CTL_GATE) & ~CTL_GATE_IDLE, h->ctl + CTL_GATE);
	writel(readl(h->ctl + CTL_INCR) | 0xf, h->ctl + CTL_INCR);
	if (!phy_init)
		return;
	v = readl(h->phy + PHY_CTL) | 0x3;
	writel(v, h->phy + PHY_CTL);
	writel(phy_cfg, h->phy + PHY_CFG);
	writel(readl(h->phy + PHY_CTL) & ~0x26, h->phy + PHY_CTL);
	udelay(20);
	writel(readl(h->phy + PHY_CTL) & ~0x1, h->phy + PHY_CTL);
	usleep_range(1000, 1500);
}

static void apple_hsic_phy_off(struct apple_hsic *h)
{
	writel(readl(h->phy + PHY_CTL) | 0x7, h->phy + PHY_CTL);
	writel(readl(h->ctl + CTL_GATE) | CTL_GATE_IDLE, h->ctl + CTL_GATE);
}

static int apple_hsic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usb_hcd *hcd;
	struct apple_hsic *h;
	struct resource *res;
	int irq, ret, v;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);	/* powers hsic1_phy -> usb2host2 -> usbctrl -> usb */
	if (ret)
		goto err_pm;

	hcd = usb_create_hcd(&apple_hsic_hc_driver, dev, dev_name(dev));
	if (!hcd) {
		ret = -ENOMEM;
		goto err_put;
	}
	h = (struct apple_hsic *)hcd_to_ehci(hcd)->priv;

	hcd->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto err_hcd;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	h->ctl = devm_platform_ioremap_resource(pdev, 1);
	h->phy = IS_ERR(h->ctl) ? h->ctl : devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(h->phy)) {
		ret = PTR_ERR(h->phy);
		goto err_hcd;
	}

	h->host_ready = devm_gpiod_get_optional(dev, "host-ready", GPIOD_OUT_LOW);
	h->device_ready = devm_gpiod_get_optional(dev, "device-ready", GPIOD_IN);
	if (IS_ERR(h->host_ready) || IS_ERR(h->device_ready)) {
		ret = IS_ERR(h->host_ready) ? PTR_ERR(h->host_ready) : PTR_ERR(h->device_ready);
		goto err_hcd;
	}

	apple_hsic_dump(dev, hcd, h, "before");
	apple_hsic_phy_on(h);

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto err_phy;
	device_wakeup_enable(hcd->self.controller);

	if (segment >= 0)
		writel(segment, &hcd_to_ehci(hcd)->regs->segment);
	apple_hsic_dump(dev, hcd, h, "started");

	if (h->host_ready) {
		gpiod_set_value_cansleep(h->host_ready, 1);
		if (h->device_ready &&
		    read_poll_timeout(gpiod_get_value_cansleep, v, v == 1, 5000, 500000, false,
				      h->device_ready))
			dev_info(dev, "device_ready low (expected while the modem runs its boot ROM)\n");
		else if (h->device_ready)
			dev_info(dev, "device_ready asserted\n");
	}
	platform_set_drvdata(pdev, hcd);
	return 0;

err_phy:
	apple_hsic_phy_off(h);
err_hcd:
	usb_put_hcd(hcd);
err_put:
	pm_runtime_put_sync(dev);
err_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void apple_hsic_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct apple_hsic *h = (struct apple_hsic *)hcd_to_ehci(hcd)->priv;

	if (h->host_ready)
		gpiod_set_value_cansleep(h->host_ready, 0);
	usb_remove_hcd(hcd);
	apple_hsic_phy_off(h);
	usb_put_hcd(hcd);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id apple_hsic_of_match[] = {
	{ .compatible = "apple,t7000-ehci-hsic" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_hsic_of_match);

static struct platform_driver apple_hsic_driver = {
	.probe = apple_hsic_probe,
	.remove = apple_hsic_remove,
	.driver = {
		.name = "ehci-apple-hsic",
		.of_match_table = apple_hsic_of_match,
	},
};

static int __init apple_hsic_init(void)
{
	ehci_init_driver(&apple_hsic_hc_driver, &apple_hsic_overrides);
	return platform_driver_register(&apple_hsic_driver);
}
module_init(apple_hsic_init);

static void __exit apple_hsic_exit(void)
{
	platform_driver_unregister(&apple_hsic_driver);
}
module_exit(apple_hsic_exit);

MODULE_DESCRIPTION("Apple T7000 HSIC EHCI glue");
MODULE_LICENSE("GPL");
