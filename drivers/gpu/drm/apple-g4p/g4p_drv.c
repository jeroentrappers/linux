// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 (T7000) GPU "G4P" (PowerVR Rogue, GX6450 class): power layer and core ID.
 *
 * Milestones M0-M2 of the bring-up (linux-on-iphone6 re/GPU-DRIVER.md): bind without touching the
 * hardware, power the GPU up on request (PMGR ps_gfx through genpd, fabric bridges 10/11, the Fender
 * power controller), and read the Rogue core ID once that is known to be safe.
 *
 * The GPU does not complete its first GFX DVFM transition under Linux yet (re/GPU-BRINGUP.md 8.10-8.12),
 * and a read of an unclocked Rogue core hangs the SoC. So:
 *  - nothing is written at probe; power-up only through debugfs, and only with experimental=1;
 *  - the Rogue core ID is read once, right after the Fender wake as iOS does (AGX probe reads it 10 us
 *    later in the same function), and only with unsafe_rgx=1: a read some 100 ms later hung the SoC
 *    (re/GPU-BRINGUP.md test D), most likely because the idle core is gated again by then;
 *  - a power-up that fails is always undone.
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

/* Fender, Apple's GPU power controller (AGX_CR_FNDR_*; re/GPU-BRINGUP.md 1, 8.11) */
#define FNDR_CTRL		0x0000	/* 0x40000000 at wake; bit 29 is firmware state (8.11), never written */
#define FNDR_PERF_CTL		0x1000	/* [3:0] perf state, bit 4 = request pending */
#define FNDR_PERF_CTL_BUSY	BIT(4)
#define FNDR_PERF_NEXT		0x1004	/* read only (iOS dumps it) */
#define FNDR_SLEEP_TIMER	0x10000
#define FNDR_TIMEOUT_US		100000

/* Rogue core (upstream ROGUE_CR_*) */
#define RGX_CR_CORE_ID		0x0018
#define RGX_CR_CORE_REVISION	0x0020

#define G4P_BRIDGE11_WORDS	32

struct g4p {
	struct device *dev;
	void __iomem *rgx;
	void __iomem *fender;
	void __iomem *bridge10;
	void __iomem *bridge11;
	u32 bridge10_cfg[2];			/* offset, value */
	u32 bridge11_cfg[G4P_BRIDGE11_WORDS];	/* words from offset 0 */
	struct mutex lock;
	bool powered;
	bool have_id;
	u32 core_rev, core_id;
	struct dentry *debugfs;
};

static bool experimental;
module_param(experimental, bool, 0444);
MODULE_PARM_DESC(experimental, "allow GPU power-up through debugfs (the sequence is not complete yet)");

static bool rgx_posted;
module_param(rgx_posted, bool, 0444);
MODULE_PARM_DESC(rgx_posted, "map the Rogue registers posted (Device-nGnRE) instead of the bus default non-posted");

static bool unsafe_rgx;
module_param(unsafe_rgx, bool, 0644);
MODULE_PARM_DESC(unsafe_rgx, "read the Rogue core ID right after the Fender wake (can hang the SoC)");

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


static void g4p_power_off_locked(struct g4p *g)
{
	if (!g->powered)
		return;
	if (g4p_fender_perf(g, FNDR_PERF_CTL_BUSY | 0))
		dev_warn(g->dev, "Fender: power-down request not acknowledged\n");
	/* TODO (8.12): turn GFX DVFM off as gpuprobe "off" does; a stuck transition left on crashes the SoC */
	pm_runtime_put_sync(g->dev);
	g->powered = false;
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
	g4p_bridges(g);
	ret = g4p_fender_wake(g);
	if (!ret && unsafe_rgx) {
		/* logged first: if the read below hangs, this is the last line in the log */
		dev_info(g->dev, "woken (Fender ctrl 0x%08x perf 0x%08x); reading the core ID (%s mapping)\n",
			 readl(g->fender + FNDR_CTRL), readl(g->fender + FNDR_PERF_CTL),
			 rgx_posted ? "posted" : "non-posted");
		/* as AGX probe: CORE_REVISION, then CORE_ID, straight after the wake (re/GPU-IOS-TRACE.md 4) */
		g->core_rev = readl(g->rgx + RGX_CR_CORE_REVISION);
		g->core_id = readl(g->rgx + RGX_CR_CORE_ID);
		g->have_id = true;
		dev_info(g->dev, "core revision 0x%08x (major %lu minor %lu maint %lu) id 0x%08x (clusters %lu)\n",
			 g->core_rev, FIELD_GET(GENMASK(23, 16), g->core_rev), FIELD_GET(GENMASK(15, 8), g->core_rev),
			 FIELD_GET(GENMASK(7, 0), g->core_rev), g->core_id, FIELD_GET(GENMASK(15, 8), g->core_id));
	}
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
	seq_printf(s, "powered %d experimental %d unsafe_rgx %d\n", g->powered, experimental, unsafe_rgx);
	if (g->powered) {	/* the Fender reads fine while ps_gfx is on (re/GPU-IOS-READS.md) */
		seq_printf(s, "fender ctrl 0x%08x perf 0x%08x next 0x%08x\n", readl(g->fender + FNDR_CTRL),
			   readl(g->fender + FNDR_PERF_CTL), readl(g->fender + FNDR_PERF_NEXT));
	}
	/* never read RGX here: only the values taken right after the wake (iOS accepts major 3 with minor 8
	 * (G4P B0) or 10 (B1); clusters = CORE_ID[15:8], re/GPU-BRINGUP.md 2) */
	if (g->have_id)
		seq_printf(s, "core revision 0x%08x id 0x%08x (read at power-up)\n", g->core_rev, g->core_id);
	else
		seq_puts(s, "core id: not read (needs unsafe_rgx=1 at power-up)\n");
	mutex_unlock(&g->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(g4p_status);

/* ------------------------------------------------------------------------------------------------ */
/* probe: map and parse only, no register access */

static void __iomem *g4p_map(struct platform_device *pdev, const char *name)
{
	return devm_platform_ioremap_resource_byname(pdev, name);
}

static int g4p_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct g4p *g;
	int ret;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->dev = dev;
	mutex_init(&g->lock);

	if (rgx_posted) {	/* plain ioremap = Device-nGnRE; the bus's nonposted-mmio would give nGnRnE */
		struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rgx");

		g->rgx = res ? devm_ioremap(dev, res->start, resource_size(res)) : NULL;
		if (!g->rgx)
			g->rgx = IOMEM_ERR_PTR(-ENOMEM);
	} else {
		g->rgx = g4p_map(pdev, "rgx");
	}
	g->fender = g4p_map(pdev, "fender");
	g->bridge10 = g4p_map(pdev, "bridge10");
	g->bridge11 = g4p_map(pdev, "bridge11");
	if (IS_ERR(g->rgx) || IS_ERR(g->fender) || IS_ERR(g->bridge10) || IS_ERR(g->bridge11))
		return dev_err_probe(dev, -ENODEV, "missing register range\n");

	ret = of_property_read_u32_array(dev->of_node, "apple,bridge10-settings", g->bridge10_cfg, 2);
	if (!ret)
		ret = of_property_read_u32_array(dev->of_node, "apple,bridge11-settings", g->bridge11_cfg,
						 G4P_BRIDGE11_WORDS);
	if (ret)
		return dev_err_probe(dev, ret, "missing bridge settings\n");
	if (g->bridge10_cfg[0] & 3 || g->bridge10_cfg[0] >= resource_size(
		platform_get_resource_byname(pdev, IORESOURCE_MEM, "bridge10")))
		return dev_err_probe(dev, -EINVAL, "bad bridge10 offset\n");

	platform_set_drvdata(pdev, g);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	g->debugfs = debugfs_create_dir("apple-g4p", NULL);
	debugfs_create_file("power", 0600, g->debugfs, g, &g4p_power_fops);
	debugfs_create_file("status", 0400, g->debugfs, g, &g4p_status_fops);
	dev_info(dev, "bound; GPU left off (power-up: experimental=1, debugfs apple-g4p/power)\n");
	return 0;
}

static void g4p_remove(struct platform_device *pdev)
{
	struct g4p *g = platform_get_drvdata(pdev);

	debugfs_remove_recursive(g->debugfs);
	mutex_lock(&g->lock);
	g4p_power_off_locked(g);
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
