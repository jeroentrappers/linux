// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 (T7000) GPU "G4P" (PowerVR Rogue, GX6450 class): power layer and core ID.
 *
 * Milestones M0-M2 of the bring-up (linux-on-iphone6 re/GPU-DRIVER.md): bind without touching the
 * hardware, power the GPU up on request (PMGR ps_gfx through genpd, fabric bridges 10/11, the Fender
 * power controller), and read the Rogue core ID.
 *
 * Any Rogue register read hangs the SoC while PMGR MiscACG (auto clock gating, 0x20e032000) is 0. iBoot
 * clears it before hand-off and XNU's ApplePMGR::enableACG sets it again; nothing on the Linux boot path
 * does (re/GPU-BRINGUP.md 8.19, 8.20). Until PMGR/SoC init owns it, this driver sets it before the first
 * RGX access, and refuses RGX access when the "misc-acg" range is missing. So:
 *  - nothing is written at probe; power-up only through debugfs, and only with experimental=1;
 *  - at power-up: MiscACG on, ps_gfx, bridges, Fender wake, then CORE_REVISION / CORE_ID as AGX probe;
 *  - a power-up that fails is always undone.
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/vmalloc.h>

#include "g4p.h"

#define G4P_FW_NAME		"apple/agx-g4p-b0.ldr"

static bool experimental;
module_param(experimental, bool, 0444);
MODULE_PARM_DESC(experimental, "allow GPU power-up through debugfs (the sequence is not complete yet)");

static unsigned int fw_irq;
module_param(fw_irq, uint, 0444);
MODULE_PARM_DESC(fw_irq, "AIC IRQ number of the firmware interrupt when the DT has none (bring-up; 0x86 on T7000)");

/* bring-up fallback for images whose GPU node has no interrupts property: map AIC_IRQ fw_irq ourselves */
static int g4p_irq_from_param(struct device *dev)
{
	struct of_phandle_args spec = { .args_count = 3, .args = { 0 /* AIC_IRQ */, fw_irq, IRQ_TYPE_LEVEL_HIGH } };
	int irq;

	spec.np = of_find_compatible_node(NULL, NULL, "apple,aic");
	if (!spec.np)
		return -ENODEV;
	irq = irq_create_of_mapping(&spec);
	of_node_put(spec.np);
	if (!irq)
		return -EINVAL;
	dev_info(dev, "no interrupts in the DT: using AIC IRQ 0x%x (fw_irq) as virq %d\n", fw_irq, irq);
	return irq;
}

static bool read_id = true;
module_param(read_id, bool, 0644);
MODULE_PARM_DESC(read_id, "read the Rogue core ID at power-up (needs the misc-acg range)");

/* ------------------------------------------------------------------------------------------------ */
/* power layer */

static void g4p_bridges(struct g4p *g)
{
	/* ApplePMGR::_configureBridge on every GFX power-on: bridge 10, then bridge 11 (ADT bridge-settings) */
	writel(g->bridge10_cfg[1], g->bridge10 + g->bridge10_cfg[0]);
	for (int i = 0; i < G4P_BRIDGE11_WORDS; i++)
		writel(g->bridge11_cfg[i], g->bridge11 + 4 * i);
}

static int g4p_fender_perf(struct g4p *g, u32 val)
{
	u32 v;

	writel(val, g->fender + FNDR_PERF_CTL);
	return readl_poll_timeout(g->fender + FNDR_PERF_CTL, v, !(v & FNDR_PERF_CTL_BUSY), 10, FNDR_TIMEOUT_US);
}

/* ProbeConfig::powerOn: empty script, sleep timer control, perf state 1, wait for the request to clear */
static int g4p_fender_wake(struct g4p *g)
{
	int ret;

	writel(0xffff0000, g->fender + FNDR_SLEEP_TIMER);
	writel(0x40000000, g->fender + FNDR_CTRL);
	ret = g4p_fender_perf(g, FNDR_PERF_CTL_BUSY | 1);
	udelay(10);
	return ret;
}

/* XNU ApplePMGR::enableACG(true), PMGR part: the Rogue register bus needs it (re/GPU-BRINGUP.md 8.20) */
static void g4p_misc_acg(struct g4p *g)
{
	u32 v = readl(g->misc_acg);

	if (v & PMGR_MISC_ACG_EN)
		return;
	writel(v | PMGR_MISC_ACG_EN, g->misc_acg);
	dev_info(g->dev, "PMGR MiscACG was off (0x%08x), enabled (0x%08x)\n", v, readl(g->misc_acg));
}

static void g4p_read_id(struct g4p *g)
{
	unsigned long major, minor;

	/* as AGX readChipInfo: 32-bit reads, CORE_REVISION then CORE_ID, straight after the wake */
	g->core_rev = readl(g->rgx + RGX_CR_CORE_REVISION);
	g->core_id = readl(g->rgx + RGX_CR_CORE_ID);
	g->have_id = true;
	major = FIELD_GET(GENMASK(23, 16), g->core_rev);
	minor = FIELD_GET(GENMASK(15, 8), g->core_rev);
	dev_info(g->dev, "core revision 0x%08x (major %lu minor %lu maint %lu) id 0x%08x (%lu)\n",
		 g->core_rev, major, minor, FIELD_GET(GENMASK(7, 0), g->core_rev), g->core_id,
		 FIELD_GET(GENMASK(15, 8), g->core_id));
	if (major != 3 || (minor != 8 && minor != 10))	/* AGX G4P_B0 readChipInfo accepts 3.8 and 3.10 */
		dev_warn(g->dev, "unexpected core revision\n");
}

static void g4p_power_off_locked(struct g4p *g)
{
	if (!g->powered)
		return;
	if (g4p_fender_perf(g, FNDR_PERF_CTL_BUSY | 0))
		dev_warn(g->dev, "Fender: power-down request not acknowledged\n");
	/* TODO (8.12): turn GFX DVFM off as gpuprobe "off" does; a stuck transition left on crashes the SoC */
	g4p_boot_irq_off(g);	/* before the power goes: the handler reads RGX */
	pm_runtime_put_sync(g->dev);
	g->powered = false;
	g4p_boot_free(g);	/* the GPU is off: its memory can go */
	dev_info(g->dev, "powered down\n");
}

static int g4p_power_on_locked(struct g4p *g)
{
	int ret;

	if (g->powered)
		return 0;
	ret = pm_runtime_resume_and_get(g->dev);	/* ps_gfx: target 0xf, wait for actual (genpd) */
	if (ret)
		return ret;
	g->powered = true;
	if (g->misc_acg)
		g4p_misc_acg(g);
	g4p_bridges(g);
	ret = g4p_fender_wake(g);
	if (!ret && read_id && g->misc_acg)
		g4p_read_id(g);
	else if (!ret && read_id)
		dev_warn(g->dev, "no misc-acg range: not reading RGX (it hangs the SoC with MiscACG off)\n");
	if (ret) {
		dev_err(g->dev, "Fender: wake request not acknowledged (perf ctl 0x%08x)\n",
			readl(g->fender + FNDR_PERF_CTL));
		g4p_power_off_locked(g);
		return ret;
	}
	dev_info(g->dev, "powered up: Fender ctrl 0x%08x perf 0x%08x next 0x%08x\n",
		 readl(g->fender + FNDR_CTRL), readl(g->fender + FNDR_PERF_CTL), readl(g->fender + FNDR_PERF_NEXT));
	return 0;
}

/* firmware -> host: META slave-port trigger 2 on AIC 0x86 (GPU-BRINGUP 8.28); only enabled while booted */
static irqreturn_t g4p_irq(int irq, void *data)
{
	struct g4p *g = data;
	u32 st = readl(g->rgx + 0x0ac8);

	if (!(st & BIT(2)))
		return IRQ_NONE;
	writel(0, g->rgx + 0x0ac8);	/* AGXMetaFirmware::clearOutstandingFirmwareInterrupts */
	atomic_inc(&g->irq_count);
	if (g->boot)	/* memory only: the firmware blocks once the trace/statistics rings are full */
		g4p_boot_irq_rings(g);
	wake_up(&g->irq_wq);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------------------------------ */
/* debugfs */

static int g4p_power_get(void *data, u64 *val)
{
	struct g4p *g = data;

	*val = g->powered;
	return 0;
}

static int g4p_power_set(void *data, u64 val)
{
	struct g4p *g = data;
	int ret = 0;

	if (val && !experimental)
		return -EPERM;
	mutex_lock(&g->lock);
	if (val)
		ret = g4p_power_on_locked(g);
	else
		g4p_power_off_locked(g);
	mutex_unlock(&g->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(g4p_power_fops, g4p_power_get, g4p_power_set, "%llu\n");

static int g4p_status_show(struct seq_file *s, void *unused)
{
	struct g4p *g = s->private;

	mutex_lock(&g->lock);
	seq_printf(s, "powered %d experimental %d read_id %d\n", g->powered, experimental, read_id);
	if (g->misc_acg)
		seq_printf(s, "pmgr misc-acg 0x%08x\n", readl(g->misc_acg));
	if (g->powered) {	/* the Fender reads fine while ps_gfx is on (re/GPU-IOS-READS.md) */
		seq_printf(s, "fender ctrl 0x%08x perf 0x%08x next 0x%08x\n", readl(g->fender + FNDR_CTRL),
			   readl(g->fender + FNDR_PERF_CTL), readl(g->fender + FNDR_PERF_NEXT));
	}
	/* never read RGX here, only the values taken at power-up (AGX accepts 3.8 and 3.10) */
	if (g->have_id)
		seq_printf(s, "core revision 0x%08x id 0x%08x (read at power-up)\n", g->core_rev, g->core_id);
	else
		seq_puts(s, "core id: not read yet\n");
	g4p_boot_show(s, g);
	mutex_unlock(&g->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(g4p_status);

/* load, parse and lay out the firmware on the host (no hardware access) */
static int g4p_fw_load_locked(struct g4p *g)
{
	int ret;

	if (g->fw.blob)
		return 0;
	ret = g4p_fw_request(g->dev, G4P_FW_NAME, &g->fw);
	if (ret)
		return ret;
	g->fw_image = vmalloc(g->fw.image_size);
	ret = g->fw_image ? g4p_fw_build_image(&g->fw, g->fw_image, g->fw.image_size) : -ENOMEM;
	if (ret) {
		vfree(g->fw_image);
		g->fw_image = NULL;
		g4p_fw_release(&g->fw);
	}
	return ret;
}

static int g4p_firmware_show(struct seq_file *s, void *unused)
{
	struct g4p *g = s->private;
	int ret;

	mutex_lock(&g->lock);
	ret = g4p_fw_load_locked(g);
	if (!ret)
		g4p_fw_show(s, &g->fw, g->fw_image);
	mutex_unlock(&g->lock);
	return ret;
}
DEFINE_SHOW_ATTRIBUTE(g4p_firmware);

/* M3: power up (if needed) and boot META with the firmware image only (g4p_boot.c) */
static int g4p_boot_set(void *data, u64 val)
{
	struct g4p *g = data;
	int ret;

	if (!experimental)
		return -EPERM;
	if (val != 1)
		return -EINVAL;
	mutex_lock(&g->lock);
	ret = g4p_fw_load_locked(g);
	if (!ret)
		ret = g4p_power_on_locked(g);
	if (!ret && !g->have_id)
		ret = -ENODEV;	/* never boot without the core ID read (MiscACG and the wake checked) */
	if (!ret)
		ret = g4p_boot_start(g);
	mutex_unlock(&g->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(g4p_boot_fops, NULL, g4p_boot_set, "%llu\n");

/* M5: echo 9 > devctl queues a device-control no-op (types 9..11 only) and waits for the firmware */
static int g4p_devctl_set(void *data, u64 val)
{
	struct g4p *g = data;
	int ret;

	if (!experimental)
		return -EPERM;
	mutex_lock(&g->lock);
	ret = g4p_boot_devctl_nop(g, val);
	mutex_unlock(&g->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(g4p_devctl_fops, NULL, g4p_devctl_set, "%llu\n");

/* M5 checkpoints (re/GPU-FIRST-JOB.md 7): echo 1..5 > cl: C1 (empty channel), C2, C3 (empty job), C4 (+ resume stream, PDS), C5 (PDS-only kernel) */
static int g4p_cl_set(void *data, u64 val)
{
	struct g4p *g = data;
	int ret;

	if (!experimental)
		return -EPERM;
	if (val < 1 || val > 7)
		return -EINVAL;
	mutex_lock(&g->lock);
	ret = val == 1 ? g4p_boot_cl_c1(g) : g4p_boot_cl_job(g, val);
	mutex_unlock(&g->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(g4p_cl_fops, NULL, g4p_cl_set, "%llu\n");

/* ------------------------------------------------------------------------------------------------ */
/* probe: map and parse only, no register access */

static void __iomem *g4p_map(struct platform_device *pdev, const char *name)
{
	return devm_platform_ioremap_resource_byname(pdev, name);
}

static int g4p_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct g4p *g;
	int ret;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->dev = dev;
	mutex_init(&g->lock);

	g->rgx = g4p_map(pdev, "rgx");
	g->fender = g4p_map(pdev, "fender");
	g->bridge10 = g4p_map(pdev, "bridge10");
	g->bridge11 = g4p_map(pdev, "bridge11");
	if (IS_ERR(g->rgx) || IS_ERR(g->fender) || IS_ERR(g->bridge10) || IS_ERR(g->bridge11))
		return dev_err_probe(dev, -ENODEV, "missing register range\n");
	/* inside the PMGR block, which the PMGR drivers own: map without claiming it */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "misc-acg");
	if (res) {
		g->misc_acg = devm_ioremap(dev, res->start, resource_size(res));
		if (!g->misc_acg)
			return -ENOMEM;
	}

	ret = of_property_read_u32_array(dev->of_node, "apple,bridge10-settings", g->bridge10_cfg, 2);
	if (!ret)
		ret = of_property_read_u32_array(dev->of_node, "apple,bridge11-settings", g->bridge11_cfg,
						 G4P_BRIDGE11_WORDS);
	if (ret)
		return dev_err_probe(dev, ret, "missing bridge settings\n");
	if (g->bridge10_cfg[0] & 3 || g->bridge10_cfg[0] >= resource_size(
		platform_get_resource_byname(pdev, IORESOURCE_MEM, "bridge10")))
		return dev_err_probe(dev, -EINVAL, "bad bridge10 offset\n");

	/* no IOMMU in front of the GPU (re/GPU-META-BOOT.md 5): its MMU walks 40-bit physical addresses */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	init_waitqueue_head(&g->irq_wq);
	g->irq = platform_get_irq_optional(pdev, 0);
	if (g->irq == -ENXIO && fw_irq)
		g->irq = g4p_irq_from_param(dev);
	if (g->irq >= 0) {
		ret = devm_request_irq(dev, g->irq, g4p_irq, IRQF_NO_AUTOEN, dev_name(dev), g);
		if (ret)
			return dev_err_probe(dev, ret, "irq\n");
	} else if (g->irq != -ENXIO) {
		return dev_err_probe(dev, g->irq, "irq\n");
	}

	platform_set_drvdata(pdev, g);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	g->debugfs = debugfs_create_dir("apple-g4p", NULL);
	debugfs_create_file("power", 0600, g->debugfs, g, &g4p_power_fops);
	debugfs_create_file("status", 0400, g->debugfs, g, &g4p_status_fops);
	debugfs_create_file("firmware", 0400, g->debugfs, g, &g4p_firmware_fops);
	debugfs_create_file("boot", 0200, g->debugfs, g, &g4p_boot_fops);
	debugfs_create_file("devctl", 0200, g->debugfs, g, &g4p_devctl_fops);
	debugfs_create_file("cl", 0200, g->debugfs, g, &g4p_cl_fops);
	dev_info(dev, "bound; GPU left off (power-up: experimental=1, debugfs apple-g4p/power)\n");
	return 0;
}

static void g4p_remove(struct platform_device *pdev)
{
	struct g4p *g = platform_get_drvdata(pdev);

	debugfs_remove_recursive(g->debugfs);
	mutex_lock(&g->lock);
	g4p_power_off_locked(g);
	vfree(g->fw_image);
	g4p_fw_release(&g->fw);
	mutex_unlock(&g->lock);
}

static const struct of_device_id g4p_of_match[] = {
	{ .compatible = "apple,t7000-g4p" },
	{ }
};
MODULE_DEVICE_TABLE(of, g4p_of_match);

static struct platform_driver g4p_driver = {
	.probe = g4p_probe,
	.remove = g4p_remove,
	.driver = {
		.name = "apple-g4p",
		.of_match_table = g4p_of_match,
	},
};
module_platform_driver(g4p_driver);

MODULE_DESCRIPTION("Apple A8 GPU (G4P) power layer and core ID (bring-up)");
MODULE_LICENSE("Dual MIT/GPL");
