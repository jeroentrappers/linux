// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: rear-camera autofocus (ISP_IOC_FOCUS, the stream-start hook and
 * the actuator's power rail).
 *
 * Command set (fw 13.42 names from its dispatcher table, layouts from iOS 12
 * H6ISP.mediacapture; packet = header, u32 channel at +8, arguments from +0xc):
 *   0x0407 CH_AF_ONE_SHOT                       -                       (iOS AutoFocusNow)
 *   0x840f APPLE_CH_AF_CONTINUOUS_FOCUS_ENABLE  +0xc u8 on, +0xd u8 mode (iOS SetContinuousAutoFocusNow
 *                                                                        = (1, mode), SetLockAFNow = (0, 0))
 *   0x8401 APPLE_CH_AF_WINDOW_PARAM_SET         +0xc u32 window, +0x10 u16 x, y, w, h
 *   0x8200 CH_AE_WINDOW_PARAM_SET               +0xc u32 window, +0x10 u16 x, y, w, h
 *   0x0702 CH_FOCUS_POSITION_SET                +0xc u8 position, +0x10 u32 (iOS ManualFocusNow)
 *   0x0703 CH_FOCUS_POSITION_GET                reply +0xc u8
 *   0x070a CH_FOCUS_ENABLE                      +0xc u16 on, +0xe u16
 *   0x8410 APPLE_CH_AF_FOCUS_PIXELS_ENABLE      +0xc u8
 *   0x0400 CH_AF_START                          -                       (iOS 12 never sends it)
 * Windows are in per-mille (0..1000) of the channel's sensor crop: the fw rejects x or y
 * > 999, w or h of 0 or > 1000 and x + w or y + h > 1000 (fw 0x17e80), then scales them
 * (fw 0x7218). iOS's stream start sends FOCUS_PIXELS_ENABLE and CONTINUOUS_FOCUS_ENABLE
 * before CH_START when its client asked for them.
 *
 * The rear module's VCM driver is powered by focus_ldo = D2186 ldo20 (enable = PMU
 * register 0x80 bit 6); iOS switches it on right after the sensor AVDD, before the fw
 * boots (power_on_hardware step 2). The DT has no regulator for it, so this file sets
 * the bit through the PMIC's regmap and restores it at power-off.
 */
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <uapi/linux/apple-isp-t7000.h>

#include "isp.h"

#define AF_SENSOR_REAR		0x237	/* IMX237 + VCM; the front IMX190 is fixed-focus */
#define PMIC_LDO20_REG		0x80
#define PMIC_LDO20_EN		BIT(6)

static int isp_af = 1;		/* 1 = continuous AF on the rear camera at stream start, 0 = no AF commands */
module_param_named(af, isp_af, int, 0644);
MODULE_PARM_DESC(af, "rear camera: 1 = continuous AF from stream start (default), 0 = send no AF command");
static uint isp_af_mode;	/* CONTINUOUS_FOCUS_ENABLE +0xd (fw: 0, 1 or 2) */
module_param_named(af_mode, isp_af_mode, uint, 0644);
static int isp_af_fpix = -1;	/* -1 = not sent; 0/1 = FOCUS_PIXELS_ENABLE before CH_START */
module_param_named(af_fpix, isp_af_fpix, int, 0644);
static int isp_af_enable = -1;	/* -1 = not sent; else FOCUS_ENABLE(on = this, 0) before CH_START */
module_param_named(af_enable, isp_af_enable, int, 0644);
static bool isp_af_start;	/* CH_AF_START before CH_START */
module_param_named(af_start, isp_af_start, bool, 0644);
static bool isp_focus_ldo = true;	/* switch focus_ldo on with the sensor rails */
module_param_named(focus_ldo, isp_focus_ldo, bool, 0644);

static struct device *af_pmic;	/* holds a reference while we own the ldo20 bit */
static bool af_ldo_set;		/* we set the bit (it was off): clear it at power-off */

/* Send one AF command (@size bytes in and out). A fw status lands in *fwst (first one wins). */
static int af_cmd(struct t7000_isp *isp, const char *name, u8 *pkt, u32 size, u16 *fwst)
{
	u16 st = 0xffff;
	int ret = isp_cmd_send(isp, pkt, size, size, true, &st);

	if (ret) {
		dev_warn(isp->dev, "af: %s: send failed (%d)\n", name, ret);
		return ret;
	}
	if (st) {
		dev_warn(isp->dev, "af: %s: fw status 0x%04x\n", name, st);
		if (fwst && !*fwst)
			*fwst = st;
	} else {
		dev_dbg(isp->dev, "af: %s: ok\n", name);
	}
	return 0;
}

static void af_pkt(u8 *pkt, u16 op, u32 ch)
{
	memset(pkt, 0, 0x20);
	put_unaligned_le16(op, pkt + 4);
	put_unaligned_le32(ch, pkt + 8);
}

static int af_simple(struct t7000_isp *isp, const char *name, u16 op, u32 size, u8 b0, u8 b1,
		     u16 *fwst)
{
	u8 pkt[0x20];

	af_pkt(pkt, op, isp->st.ch);
	pkt[0xc] = b0;
	pkt[0xd] = b1;
	return af_cmd(isp, name, pkt, size, fwst);
}

static int af_window(struct t7000_isp *isp, const char *name, u16 op, const struct isp_focus *f,
		     u16 *fwst)
{
	u8 pkt[0x20];

	af_pkt(pkt, op, isp->st.ch);
	put_unaligned_le32(0, pkt + 0xc);	/* window 0, as iOS */
	put_unaligned_le16(f->x, pkt + 0x10);
	put_unaligned_le16(f->y, pkt + 0x12);
	put_unaligned_le16(f->w, pkt + 0x14);
	put_unaligned_le16(f->h, pkt + 0x16);
	return af_cmd(isp, name, pkt, 0x18, fwst);
}

/* CH_FOCUS_POSITION_GET: 0..255, -1 when the fw does not answer */
static int af_position(struct t7000_isp *isp)
{
	u16 st = 0;
	u8 pkt[0x20];

	af_pkt(pkt, 0x703, isp->st.ch);
	if (af_cmd(isp, "FOCUS_POSITION_GET", pkt, 0x10, &st) || st)
		return -1;
	return pkt[0xc];
}

static bool af_rear(struct t7000_isp *isp)
{
	return isp->st.on && isp->st.sensor_id == AF_SENSOR_REAR;
}

/* the replayed request holds the lens (one scan done, lock, manual) rather than running CAF */
static bool af_held(struct isp_stream *st)
{
	return st->af_have && st->af_mode != ISP_FOCUS_CONTINUOUS;
}

/* open / last release: nothing of an earlier client is replayed */
void isp_af_forget(struct t7000_isp *isp)
{
	isp->st.af_have = false;
	isp->st.af_lens = -1;
	memset(isp->st.af_win, 0, sizeof(isp->st.af_win));
}

/*
 * isp_stream_start(), right before CH_START (iOS H6ISPCaptureStreamStart order). Never fatal.
 * A client's last ISP_IOC_FOCUS on this channel is replayed instead of plain CAF, so a STOP +
 * START (Orchard's photo: viewfinder -> still preset -> viewfinder) keeps tap-to-focus / AF lock:
 * its window, and CAF off when the lens was held (isp_af_stream_on puts the lens back).
 */
void isp_af_prestart(struct t7000_isp *isp)
{
	struct isp_stream *st = &isp->st;
	int fpix = READ_ONCE(isp_af_fpix), en = READ_ONCE(isp_af_enable);
	u8 pkt[0x20];

	if (st->af_have && st->af_ch != st->ch)
		isp_af_forget(isp);	/* (the other camera's request) */
	if (!READ_ONCE(isp_af) || !af_rear(isp))
		return;
	if (en >= 0) {
		af_pkt(pkt, 0x70a, st->ch);
		put_unaligned_le16(en, pkt + 0xc);
		af_cmd(isp, "FOCUS_ENABLE", pkt, 0x14, NULL);
	}
	if (fpix >= 0)
		af_simple(isp, "AF_FOCUS_PIXELS_ENABLE", 0x8410, 0x10, !!fpix, 0, NULL);
	if (READ_ONCE(isp_af_start))
		af_simple(isp, "AF_START", 0x400, 0xc, 0, 0, NULL);
	if (st->af_have && st->af_win[2] && st->af_win[3]) {
		struct isp_focus f = { .x = st->af_win[0], .y = st->af_win[1], .w = st->af_win[2],
				       .h = st->af_win[3] };

		af_window(isp, "AF_WINDOW_PARAM_SET", 0x8401, &f, NULL);
	}
	if (af_held(st))
		af_simple(isp, "AF_CONTINUOUS_FOCUS_ENABLE", 0x840f, 0x10, 0, 0, NULL);
	else
		af_simple(isp, "AF_CONTINUOUS_FOCUS_ENABLE", 0x840f, 0x10, 1, READ_ONCE(isp_af_mode),
			  NULL);
}

/* isp_stream_start(), after CH_START succeeded */
void isp_af_stream_on(struct t7000_isp *isp)
{
	struct isp_stream *st = &isp->st;
	int pos;

	if (!READ_ONCE(isp_af) || !af_rear(isp))
		return;
	pos = af_position(isp);
	if (af_held(st)) {
		int was = pos;

		if (st->af_lens >= 0 && pos != st->af_lens) {	/* the fw moved it across STOP/START */
			u8 pkt[0x20];

			af_pkt(pkt, 0x702, st->ch);
			pkt[0xc] = st->af_lens;
			if (!af_cmd(isp, "FOCUS_POSITION_SET", pkt, 0x14, NULL))
				pos = af_position(isp);
		}
		dev_info(isp->dev, "af: replayed mode %u window %u,%u %ux%u: lens held, %d -> %d (was %d at stop)\n",
			 st->af_mode, st->af_win[0], st->af_win[1], st->af_win[2], st->af_win[3], was, pos,
			 st->af_lens);
		return;
	}
	dev_info(isp->dev, "af: continuous AF on (mode %u%s), lens position %d, focus_ldo %s\n",
		 READ_ONCE(isp_af_mode), st->af_have ? ", replayed window" : "", pos,
		 af_pmic ? (af_ldo_set ? "switched on" : "was on") : "not controlled");
}

/* isp_stream_stop(), before CH_STOP (isp->lock held): where a held lens is, for the next START */
void isp_af_prestop(struct t7000_isp *isp)
{
	struct isp_stream *st = &isp->st;

	if (READ_ONCE(isp_af) && af_rear(isp) && af_held(st))
		st->af_lens = af_position(isp);
}

static void af_clamp(u16 *pos, u16 *len)
{
	if (*len > 1000)
		*len = 1000;
	if (*pos > 1000 - *len)
		*pos = 1000 - *len;
}

/* ISP_IOC_FOCUS (isp-cdev.c). Caller holds no lock. */
long isp_af_ioctl(struct t7000_isp *isp, void __user *up)
{
	struct isp_focus f;
	bool win, rear;
	u16 fwst = 0;
	int ret = 0;

	if (copy_from_user(&f, up, sizeof(f)))
		return -EFAULT;
	if (f.mode > ISP_FOCUS_QUERY || f.flags & ~ISP_FOCUS_F_AE)
		return -EINVAL;
	win = f.w && f.h;
	if (win) {
		af_clamp(&f.x, &f.w);
		af_clamp(&f.y, &f.h);
	}

	mutex_lock(&isp->lock);
	if (!isp->st.on || isp->st.stopping) {
		ret = -EAGAIN;
		goto out;
	}
	rear = af_rear(isp);
	if (win && f.flags & ISP_FOCUS_F_AE)
		ret = af_window(isp, "AE_WINDOW_PARAM_SET", 0x8200, &f, &fwst);
	if (!rear || f.mode == ISP_FOCUS_QUERY || ret || !READ_ONCE(isp_af))	/* af=0: no AF command at all */
		goto pos;
	if (win && f.mode != ISP_FOCUS_MANUAL)
		ret = af_window(isp, "AF_WINDOW_PARAM_SET", 0x8401, &f, &fwst);
	if (ret)
		goto pos;
	switch (f.mode) {
	case ISP_FOCUS_CONTINUOUS:
		ret = af_simple(isp, "AF_CONTINUOUS_FOCUS_ENABLE", 0x840f, 0x10, 1,
				READ_ONCE(isp_af_mode), &fwst);
		break;
	case ISP_FOCUS_ONCE:
		ret = af_simple(isp, "AF_CONTINUOUS_FOCUS_ENABLE", 0x840f, 0x10, 0, 0, &fwst);
		if (!ret)
			ret = af_simple(isp, "AF_ONE_SHOT", 0x407, 0xc, 0, 0, &fwst);
		break;
	case ISP_FOCUS_LOCK:
		ret = af_simple(isp, "AF_CONTINUOUS_FOCUS_ENABLE", 0x840f, 0x10, 0, 0, &fwst);
		break;
	case ISP_FOCUS_MANUAL: {
		u8 pkt[0x20];

		ret = af_simple(isp, "AF_CONTINUOUS_FOCUS_ENABLE", 0x840f, 0x10, 0, 0, &fwst);
		if (ret)
			break;
		af_pkt(pkt, 0x702, isp->st.ch);
		pkt[0xc] = clamp(f.position, 0, 255);
		ret = af_cmd(isp, "FOCUS_POSITION_SET", pkt, 0x14, &fwst);
		break;
	}
	}
	if (!ret) {	/* replayed at the next START of this channel (isp_af_prestart) */
		isp->st.af_have = true;
		isp->st.af_ch = isp->st.ch;
		isp->st.af_mode = f.mode;
		isp->st.af_lens = -1;
		if (win) {
			isp->st.af_win[0] = f.x, isp->st.af_win[1] = f.y;
			isp->st.af_win[2] = f.w, isp->st.af_win[3] = f.h;
		}
	}
pos:
	f.position = rear && READ_ONCE(isp_af) ? af_position(isp) : -1;
	f.fw_status = fwst;
	dev_info(isp->dev, "af: mode %u window %u,%u %ux%u%s -> lens %d, fw status 0x%04x%s\n", f.mode,
		 f.x, f.y, f.w, f.h, f.flags & ISP_FOCUS_F_AE ? " +AE" : "", f.position, fwst,
		 rear ? "" : " (fixed-focus camera)");
out:
	mutex_unlock(&isp->lock);
	isp_term_flush(isp);
	if (ret)
		return ret;
	return copy_to_user(up, &f, sizeof(f)) ? -EFAULT : 0;
}

/*
 * focus_ldo: called by isp_sensor_power_on() after the AVDD rail (on) and by
 * isp_sensor_power_off() before it (off). Never fatal: without it AF does nothing.
 */
void isp_af_power(struct t7000_isp *isp, bool on)
{
	struct device_node *np;
	struct i2c_client *cl;
	struct regmap *map;
	unsigned int v = 0;

	if (!on) {
		if (!af_pmic)
			return;
		map = dev_get_regmap(af_pmic, NULL);
		if (map && af_ldo_set)
			regmap_update_bits(map, PMIC_LDO20_REG, PMIC_LDO20_EN, 0);
		put_device(af_pmic);
		af_pmic = NULL;
		af_ldo_set = false;
		return;
	}
	if (!READ_ONCE(isp_focus_ldo) || af_pmic)
		return;
	np = of_find_compatible_node(NULL, NULL, "apple,adi-pmic");
	if (!np) {
		dev_warn(isp->dev, "af: no apple,adi-pmic node: focus_ldo stays as it is\n");
		return;
	}
	cl = of_find_i2c_device_by_node(np);
	of_node_put(np);
	map = cl ? dev_get_regmap(&cl->dev, NULL) : NULL;
	if (!map) {
		dev_warn(isp->dev, "af: PMIC regmap not available: focus_ldo stays as it is\n");
		if (cl)
			put_device(&cl->dev);
		return;
	}
	af_pmic = &cl->dev;
	if (regmap_read(map, PMIC_LDO20_REG, &v)) {
		dev_warn(isp->dev, "af: PMIC 0x%x read failed\n", PMIC_LDO20_REG);
		return;
	}
	if (!(v & PMIC_LDO20_EN)) {
		if (regmap_update_bits(map, PMIC_LDO20_REG, PMIC_LDO20_EN, PMIC_LDO20_EN)) {
			dev_warn(isp->dev, "af: focus_ldo enable failed\n");
			return;
		}
		af_ldo_set = true;
		usleep_range(1000, 2000);	/* iOS: IOSleep(1) */
	}
	dev_info(isp->dev, "af: focus_ldo (PMIC 0x%x = 0x%02x) %s\n", PMIC_LDO20_REG, v,
		 af_ldo_set ? "switched on" : "already on");
}

/*
 * ISP_IOC_TORCH (isp-cdev.c): the fw's LED torch commands on channel 0 (rear), as iOS sends them:
 *   0x0606 CH_LED_TORCH_COLOR_MODE_SET  +0xc u16 mode (2 = manual split), +0xe u16 % on LED2 (0 = LED1 only)
 *   0x0603 CH_LED_TORCH_ON              +0xc u32 level 1..5
 *   0x0602 CH_LED_TORCH_OFF
 *   0x0607 CH_LED_TORCH_CURRENTS_GET    reply +0xc u16 LED1 mA, +0xe u16 LED2 mA
 * Levels above 5 are clamped to 5: the fw's "full" is the chip's 187 mA total, without a timeout.
 */
long isp_torch_ioctl(struct t7000_isp *isp, void __user *up)
{
	struct isp_torch t;
	u16 fwst = 0;
	u8 pkt[0x20];
	int ret = 0;

	if (copy_from_user(&t, up, sizeof(t)))
		return -EFAULT;
	if (t.split > 100 || t.split < -1)
		return -EINVAL;
	if (t.level > 5)
		t.level = 5;

	mutex_lock(&isp->lock);
	if (!isp->st.on || isp->st.stopping) {
		ret = -EAGAIN;
		goto out;
	}
	if (t.split >= 0) {
		af_pkt(pkt, 0x606, 0);
		put_unaligned_le16(2, pkt + 0xc);
		put_unaligned_le16(t.split, pkt + 0xe);
		ret = af_cmd(isp, "LED_TORCH_COLOR_MODE_SET", pkt, 0x10, &fwst);
	}
	if (!ret) {
		af_pkt(pkt, t.level ? 0x603 : 0x602, 0);
		put_unaligned_le32(t.level, pkt + 0xc);
		ret = af_cmd(isp, t.level ? "LED_TORCH_ON" : "LED_TORCH_OFF", pkt, 0x10, &fwst);
	}
	t.led1_ma = t.led2_ma = 0;
	if (!ret) {
		u16 st = 0;

		af_pkt(pkt, 0x607, 0);
		if (!af_cmd(isp, "LED_TORCH_CURRENTS_GET", pkt, 0x10, &st) && !st) {
			t.led1_ma = get_unaligned_le16(pkt + 0xc);
			t.led2_ma = get_unaligned_le16(pkt + 0xe);
		}
	}
	dev_info(isp->dev, "torch: level %u split %d -> LED1 %u mA, LED2 %u mA, fw status 0x%04x\n",
		 t.level, t.split, t.led1_ma, t.led2_ma, fwst);
	t.fw_status = fwst;
out:
	mutex_unlock(&isp->lock);
	if (ret)
		return ret;
	return copy_to_user(up, &t, sizeof(t)) ? -EFAULT : 0;
}
