// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: PMGR power (stage 1) and bridge/clock setup (stage 2).
 *
 * Rule: the ISP window is touched only after ps_media and ps_isp read ACTUAL == 0xf.
 * Reading it gated can wedge the fabric.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>

#include "isp.h"

static u32 ps_rd(struct t7000_isp *isp, u32 off)
{
	return readl(isp->pmgr + off);
}

static bool ps_is_on(u32 v)
{
	return FIELD_GET(PS_ACTUAL, v) == PS_ACTIVE;
}

void isp_pmgr_log(struct t7000_isp *isp, const char *when)
{
	dev_info(isp->dev, "pmgr %s: media 0x%08x isp 0x%08x ispsens0 0x%08x ispsens1 0x%08x\n",
		 when, ps_rd(isp, PMGR_PS_MEDIA), ps_rd(isp, PMGR_PS_ISP),
		 ps_rd(isp, PMGR_PS_ISPSENS0), ps_rd(isp, PMGR_PS_ISPSENS1));
}

static int ps_set(struct t7000_isp *isp, u32 off, const char *name, u32 state)
{
	u32 v = ps_rd(isp, off);
	int ret;

	v &= ~(PS_AUTO_ENABLE | PS_FLAGS | PS_TARGET);
	v |= FIELD_PREP(PS_TARGET, state);
	writel(v, isp->pmgr + off);
	ret = readl_poll_timeout(isp->pmgr + off, v,
				 FIELD_GET(PS_ACTUAL, v) == state, 10, 100000);
	if (ret)
		dev_err(isp->dev, "pmgr %s: timeout (100 ms) waiting for ACTUAL == 0x%x, reg 0x%08x\n",
			name, state, v);
	else
		dev_info(isp->dev, "pmgr %s: -> 0x%x ok, reg 0x%08x\n", name, state, v);
	return ret;
}

/* Power one domain on unless it already is; *ours tells whether we did it. */
static int ps_on(struct t7000_isp *isp, u32 off, const char *name, bool *ours)
{
	u32 v = ps_rd(isp, off);
	int ret;

	*ours = false;
	if (!ps_is_on(v)) {
		*ours = true;	/* even on timeout: the target was changed, undo it */
		ret = ps_set(isp, off, name, PS_ACTIVE);
		if (ret)
			return ret;
		v = ps_rd(isp, off);
	} else {
		dev_info(isp->dev, "pmgr %s: already on (0x%08x), will leave it on\n", name, v);
	}
	if (v & (PS_RESET | PS_DEV_DISABLE)) {
		dev_err(isp->dev, "pmgr %s: RESET/DEV_DISABLE set (0x%08x), refusing\n", name, v);
		return -EIO;
	}
	return 0;
}

int isp_power_on(struct t7000_isp *isp)
{
	int ret;

	isp->isp_was_on = ps_is_on(ps_rd(isp, PMGR_PS_ISP));
	ret = ps_on(isp, PMGR_PS_MEDIA, "media", &isp->on_media);
	if (ret)
		return ret;
	ret = ps_on(isp, PMGR_PS_ISP, "isp", &isp->on_isp);
	if (ret)
		return ret;
	if (isp_sens) {
		ret = ps_on(isp, PMGR_PS_ISPSENS0, "ispsens0", &isp->on_sens0);
		if (!ret)
			ret = ps_on(isp, PMGR_PS_ISPSENS1, "ispsens1", &isp->on_sens1);
		if (ret)
			return ret;
	}
	isp->version = isp_rd(isp, ISP_VERSION);
	dev_info(isp->dev, "version reg (+0xa0000) = 0x%08x -> rev %u%s\n",
		 isp->version, isp->version >> 16,
		 (isp->version >> 16) == 4 ? "" : " (only rev 4 is implemented; stage 2+ will refuse)");
	isp->version >>= 16;
	/*
	 * Nothing else in the window until stage 2 has written the clock enable
	 * +0x20080 (iOS power_on_hardware, Asahi stage 1): only +0xa0000 is read
	 * before it, like the kext's start().
	 */
	return 0;
}

void isp_power_off(struct t7000_isp *isp)
{
	if (isp->on_sens1) {
		ps_set(isp, PMGR_PS_ISPSENS1, "ispsens1", PS_PWRGATE);
		isp->on_sens1 = false;
	}
	if (isp->on_sens0) {
		ps_set(isp, PMGR_PS_ISPSENS0, "ispsens0", PS_PWRGATE);
		isp->on_sens0 = false;
	}
	if (isp->on_isp) {
		ps_set(isp, PMGR_PS_ISP, "isp", PS_PWRGATE);
		isp->on_isp = false;
	}
	if (isp->on_media) {
		ps_set(isp, PMGR_PS_MEDIA, "media", PS_PWRGATE);
		isp->on_media = false;
	}
}

void isp_log_window(struct t7000_isp *isp, const char *when)
{
	dev_info(isp->dev, "%s: gpio0-7 %08x %08x %08x %08x %08x %08x %08x %08x\n", when,
		 isp_rd(isp, ISP_GPIO(0)), isp_rd(isp, ISP_GPIO(1)), isp_rd(isp, ISP_GPIO(2)),
		 isp_rd(isp, ISP_GPIO(3)), isp_rd(isp, ISP_GPIO(4)), isp_rd(isp, ISP_GPIO(5)),
		 isp_rd(isp, ISP_GPIO(6)), isp_rd(isp, ISP_GPIO(7)));
	dev_info(isp->dev, "%s: ctrl %08x fwbase %08x win %08x %08x %08x %08x %08x mbox p%08x e%08x\n",
		 when, isp_rd(isp, ISP_CTRL_V34), isp_rd(isp, ISP_FW_BASE),
		 isp_rd(isp, ISP_WIN_0C), isp_rd(isp, ISP_WIN_10), isp_rd(isp, ISP_WIN_14),
		 isp_rd(isp, ISP_WIN_18), isp_rd(isp, ISP_WIN_1C),
		 isp_rd(isp, ISP_MBOX_PENDING), isp_rd(isp, ISP_MBOX_ENABLE));
	dev_info(isp->dev, "%s: clken %08x bridge 1c=%08x 20=%08x 48=%08x 1000=%08x tl-out 44=%08x 68=%08x\n",
		 when, isp_rd(isp, ISP_CLOCK_EN), isp_rd(isp, 0x1c), isp_rd(isp, 0x20),
		 isp_rd(isp, 0x48), isp_rd(isp, 0x1000), isp_rd(isp, ISP_TLIMIT0_OUT),
		 isp_rd(isp, ISP_TLIMIT1_OUT));
}

/* PMGR bridge-settings-6 (= ISP, PMGR reg index 8 = 0x5b00000), applied by iOS PMGR on power-up */
static const u32 isp_bridge6[][2] = {
	{ 0x1c, 0x80000004 }, { 0x20, 0x80240010 },
	{ 0x48, 0x2020 }, { 0x4c, 0x2020 }, { 0x50, 0x2020 }, { 0x54, 0x2020 },
	{ 0x58, 0x2020 }, { 0x5c, 0x2020 }, { 0x60, 0x2020 }, { 0x64, 0x2020 },
	{ 0x1000, 0xc00f000f }, { 0x1008, 0xc00f000f },
	{ 0x21000, 0x808 }, { 0x21004, 0x808 },
};

int isp_stage_clocks(struct t7000_isp *isp)
{
	int i;

	if (isp->version != 4) {
		dev_err(isp->dev, "rev %u: only rev 4 is implemented\n", isp->version);
		return -ENODEV;
	}
	for (i = 0; i < ARRAY_SIZE(isp_bridge6); i++) {
		u32 off = isp_bridge6[i][0], old = isp_rd(isp, off);

		isp_wr(isp, off, isp_bridge6[i][1]);
		dev_info(isp->dev, "bridge +0x%05x: 0x%08x -> 0x%08x (readback 0x%08x)\n",
			 off, old, isp_bridge6[i][1], isp_rd(isp, off));
	}
	isp_wr(isp, ISP_CLOCK_EN, 1);
	dev_info(isp->dev, "clock enable +0x20080 = 1 (readback 0x%08x)\n",
		 isp_rd(isp, ISP_CLOCK_EN));
	isp_log_window(isp, "after stage 2");
	return 0;
}

/*
 * dry_run: reads only. If media + isp are already on, only the version reg is
 * read (the rest of the window may be clock-gated: we do not know whether
 * +0x20080 was written by whoever powered it).
 */
void isp_dry_read(struct t7000_isp *isp)
{
	if (ps_is_on(ps_rd(isp, PMGR_PS_MEDIA)) && ps_is_on(ps_rd(isp, PMGR_PS_ISP))) {
		isp->version = isp_rd(isp, ISP_VERSION);
		dev_info(isp->dev, "dry_run: version reg 0x%08x (rev %u); not reading the rest of the window\n",
			 isp->version, isp->version >> 16);
	} else {
		dev_info(isp->dev, "dry_run: ps_media/ps_isp not both on: not reading the ISP window\n");
	}
	isp_fw_peek(isp);
}
