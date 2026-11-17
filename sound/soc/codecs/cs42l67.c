// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cirrus Logic CS42L67 audio codec (Apple iPhone 6, "audio-control,cs42l67")
 *
 * There is no public datasheet. Everything here comes from reverse
 * engineering Apple's AppleCS42L67Audio / AppleEmbeddedAudio kexts (iOS
 * 12.5.5); see re/CODEC-DRIVER.md in the iphone6-linux project. Register
 * names are ours. Values that iOS writes are copied verbatim; the meaning of
 * most bits is inferred from how iOS uses them.
 *
 * Supported: the XSP serial port as a TDM slave (the MCA is the clock
 * master), capture from the analog microphones through the two ADC pairs,
 * an experimental mono headphone path, and an optional polled jack status.
 *
 * Analog inputs: each ADC pair owns four pins, and an ADC's 2-bit input
 * select is (AINn - 1) % 4. iOS gives the A channel the odd pins and the B
 * channel the even ones (AppleCS42L67Audio's input table is ain1, ain3, ain4,
 * ain5, ain6, ain7, ain8, dmic1a..dmic2b; ain2 is the headset mic):
 *   ADC1A: AIN1 = 0, AIN3 = 2    ADC1B: AIN2 (headset) = 1, AIN4 = 3
 *   ADC2A: AIN5 = 0, AIN7 = 2    ADC2B: AIN6 = 1, AIN8 = 3
 * n61: AIN1 = bottom mic (MICBIAS1), AIN5 = rear (MICBIAS4), AIN6 = front
 * (MICBIAS3), so the rear mic is ADC2A and the front mic ADC2B.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

/* Registers are page:reg, 128 per page; the SPI frame carries both bytes */
#define CS42L67_DEVID_AB	0x001	/* 0x42 */
#define CS42L67_DEVID_CD	0x002	/* 0xa6 */
#define CS42L67_DEVID_E		0x003	/* 0x7x */
#define CS42L67_REVID		0x005
#define CS42L67_PWR		0x006
#define  PWR_PDN		BIT(7)	/* iOS clears it to power up */
#define  PWR_PDN2		BIT(6)	/* iOS sets it after PDN_DONE */
#define CS42L67_ID_CTL		0x007	/* bit7 cleared around the ID read */
#define CS42L67_MICBIAS1	0x00e	/* [1:0] MICBIAS1 */
#define CS42L67_MICBIAS34	0x00f	/* [1:0] MICBIAS3, [3:2] MICBIAS4 */
#define  MICBIAS_2V7		2	/* > 2.25 V; 1 = the low setting */
#define CS42L67_PATH_PDN	0x010
#define  PATH_PDN_OUT		BIT(7)
#define  PATH_PDN_IN		BIT(6)
#define CS42L67_XSP_CFG1	0x011
#define CS42L67_SP_FS		0x012	/* [3:0] serial port rate, CS42L73 VXSPFS codes */
#define  SP_FS_MASK		GENMASK(3, 0)
#define  SP_FS_48K		9	/* 8k=1 11.025k=2 12k=3 16k=4 22.05k=5 24k=6 32k=7 44.1k=8 */
#define CS42L67_XSP_CTL		0x013
#define  XSP_PDN		BIT(7)
#define  XSP_TX_PDN		BIT(6)	/* codec -> AP */
#define  XSP_RX_PDN		BIT(4)	/* AP -> codec */
#define CS42L67_XSP_CFG2	0x014
#define CS42L67_XSP_SRC		0x015	/* bit7: rate converter in path */
#define CS42L67_XSP_TX_POS(n)	(0x016 + (n))	/* bit7 off, [6:0] byte offset */
#define  TX_POS_OFF		BIT(7)
#define CS42L67_XSP_TX_MASK(n)	(0x020 + (n))	/* 32 bit byte mask, MSB first */
#define CS42L67_XSP_RX_POS(n)	(0x024 + (n))	/* [7:5] bytes-1, [4:0] offset */
#define CS42L67_VSP_CTL		0x02e
#define CS42L67_VSP_CFG		0x02f
#define CS42L67_STATUS		0x04b
#define  STATUS_PDN_DONE	BIT(7)
#define  STATUS_DET_DONE	BIT(6)
#define CS42L67_HSDET_STATUS	0x075	/* [6:5] read by iOS' jack code */
#define CS42L67_ASP_MISC	0x101	/* iOS: 0 unless a 'mb1x' DSP source is used */
/*
 * ASP TX channel n (n = 0, 1), 5 registers from 0x103 + 5n: [6:5] ADC
 * source; then attenuation-like bytes, 0x3f = off (iOS route fn, the
 * 0x171 "mic" loop, re/cs42l67/l67-annot.c ~1000-1130)
 */
#define CS42L67_ASP_TX(n, r)	(0x103 + 5 * (n) + (r))
#define CS42L67_VOICE_SRC	0x102	/* [3:0] L, [7:4] R: XSP RX ch (1-based) */
#define CS42L67_MISC_130	0x130
#define CS42L67_XSP_TX_SRC(n)	(0x141 + (n))
#define CS42L67_SRC_RATE	0x15e	/* 0x0c = 48 kHz / no conversion */
#define CS42L67_ANA_201		0x201
#define CS42L67_ANA_202		0x202
#define CS42L67_ADC1_PDN	0x203	/* bit7 ADC1B, bit6 ADC1A, bit2 DMIC1 */
#define CS42L67_ADC1_DMIC	0x204
#define CS42L67_ADC1_MUX	0x205	/* [5:4] ADC1A input, [7:6] ADC1B input */
#define  ADC_MUX_A_SHIFT	4
#define  ADC_MUX_B_SHIFT	6
#define CS42L67_ADC1A_GAIN	0x206
#define CS42L67_ADC1B_GAIN	0x207
#define CS42L67_ADC1A_BOOST	0x208
#define CS42L67_ADC1B_BOOST	0x209
#define  ADC_BOOST		BIT(7)	/* iOS ctl 0x16f; set, the ADC outputs digital zero on n61 */
#define CS42L67_ADC2_PDN	0x217	/* bit7 ADC2B, bit6 ADC2A, bit2 DMIC2 */
#define CS42L67_ADC2_DMIC	0x218
#define CS42L67_ADC2_MUX	0x219	/* [5:4] ADC2A input, [7:6] ADC2B input */
#define CS42L67_ADC2A_GAIN	0x21a
#define CS42L67_ADC2B_GAIN	0x21b
#define CS42L67_ADC2A_BOOST	0x21c
#define CS42L67_ADC2B_BOOST	0x21d
#define  ADC_PDN_IDLE		0xc6	/* iOS' starting value */
/*
 * ADC PGA (0x206/0x207/0x21a/0x21b) as iOS drives it from its ctl 0x170,
 * which it declares as 0..76 = -6 dB .. +32 dB in 0.5 dB steps: [5:0] =
 * signed fine gain in 0.5 dB (0x34 = -6 dB), bit6 / bit7 = two fixed stages
 * that the encoding only makes continuous at +10 / +20 dB.
 */
#define  ADC_VOL_MAX		76
/*
 * Default +20 dB (0x80, stage 2 + 0 fine). iOS' own ctl 0x170 default is
 * index 0 (-6 dB) with CoreAudio raising it at run time; at -6 dB the n61
 * mics read about -57 dBFS (90th percentile) for speech at arm's length,
 * so +26 dB puts speech near -30 dBFS with room left for taps.
 */
#define  ADC_VOL_DEFAULT	52
#define CS42L67_ADC1_EN		0x234	/* iOS: 0x80 while an ADC1 channel is on */
#define CS42L67_ADC2_EN		0x235	/* iOS: 0x80 while an ADC2 channel is on */
#define  ADC_EN_SHIFT		4	/* [7:4]: 0x8 on (iOS), 0x7 = reset value */
#define CS42L67_ANA_239		0x239
#define CS42L67_OUT_PDN1	0x242
#define  OUT_PDN_A		BIT(5)
#define  OUT_PDN_B		BIT(3)
#define CS42L67_OUT_CTL		0x243
#define  OUT_CTL_IDLE		BIT(3)	/* set by iOS while no output plays */
#define CS42L67_OUTA_AGAIN	0x244
#define CS42L67_OUTA_AGAIN_R	0x245
#define CS42L67_OUTA_VOL	0x246
#define CS42L67_OUTA_VOL_R	0x247
#define CS42L67_OUTB_VOL	0x248
#define CS42L67_OUTB_VOL_R	0x249
#define  OUT_VOL_MUTE		0x40
#define  OUT_AGAIN_MUTE		0x19
#define CS42L67_OUT_PDN2	0x24f
#define CS42L67_OUTC_GAIN	0x252
#define CS42L67_OUTD_GAIN	0x253
#define CS42L67_DSP_PDN		0x301
#define CS42L67_RESET		0x9901	/* 0xa5 = held in reset */
#define  RESET_HOLD		0xa5
#define CS42L67_TRIM		0xc952

#define CS42L67_MAX_REG		0xc97f
#define CS42L67_TX_CHANNELS	10
#define CS42L67_TX_ROUTED	4	/* XSP TX channels with a source mux */
#define CS42L67_FRAME_SLOTS	8	/* MCA2: 8 x 16-bit slots */
#define CS42L67_DAI_XSP		0
#define CS42L67_DAI_ASP		1
#define CS42L67_SPI_WRITE	0x6c
#define CS42L67_SPI_READ	0x6d

/* XSP TX channel source codes (what iOS writes to 0x141 + n) */
#define SRC_ADC1A		2
#define SRC_ADC1B		3
#define SRC_ADC2A		5
#define SRC_ADC2B		6

/* ASP TX channel source codes (0x103 / 0x108 [6:5]) */
#define ASP_SRC_ADC1A		(0 << 5)
#define ASP_SRC_ADC2A		(2 << 5)

static bool allow_missing = true;
module_param(allow_missing, bool, 0444);
MODULE_PARM_DESC(allow_missing,
		 "register an inert DAI when the chip ID does not match, so the rest of the sound card still binds");

static bool jack_poll;
module_param(jack_poll, bool, 0444);
MODULE_PARM_DESC(jack_poll, "poll the (unverified) jack status register every 500 ms");

struct cs42l67 {
	struct device *dev;
	struct spi_device *spi;
	struct regmap *regmap;
	struct clk *mclk;
	struct snd_soc_component *component;
	unsigned int tx_slot;		/* first TDM slot of XSP TX channel 0 */
	unsigned int rx_slot;		/* TDM slot XSP RX channel 0 listens to */
	bool inert;
	bool powered;
	int rcv_gain;			/* earpiece gain, 0.5 dB steps (0x252 = this, signed) */
	bool rcv_on;			/* output C (earpiece) powered */
	unsigned int dbg_reg;		/* "Debug Reg" / "Debug Val" controls */

	struct snd_soc_jack jack;
	struct delayed_work jack_work;
	int jack_status;

	u8 buf[5] ____cacheline_aligned;
	u8 rbuf[5];
};

/* ---- register access: 6C/6D page reg 00 data ---- */

static int cs42l67_spi_write(void *ctx, unsigned int reg, unsigned int val)
{
	struct cs42l67 *cs = ctx;

	cs->buf[0] = CS42L67_SPI_WRITE;
	cs->buf[1] = reg >> 8;
	cs->buf[2] = reg & 0x7f;
	cs->buf[3] = 0;
	cs->buf[4] = val;
	return spi_write(cs->spi, cs->buf, 5);
}

static int cs42l67_spi_read(void *ctx, unsigned int reg, unsigned int *val)
{
	struct cs42l67 *cs = ctx;
	struct spi_transfer t = {
		.tx_buf = cs->buf,
		.rx_buf = cs->rbuf,
		.len = 5,
	};
	int ret;

	/* full duplex; the data byte comes back in the 5th rx byte */
	cs->buf[0] = CS42L67_SPI_READ;
	cs->buf[1] = reg >> 8;
	cs->buf[2] = reg & 0x7f;
	cs->buf[3] = 0;
	cs->buf[4] = 0;
	ret = spi_sync_transfer(cs->spi, &t, 1);
	if (ret)
		return ret;
	*val = cs->rbuf[4];
	return 0;
}

static const struct regmap_bus cs42l67_bus = {
	.reg_write = cs42l67_spi_write,
	.reg_read = cs42l67_spi_read,
};

/* pages 0-3 hold everything iOS touches, plus two test/trim registers */
static bool cs42l67_readable_reg(struct device *dev, unsigned int reg)
{
	return reg <= 0x37f && (reg & 0xff) < 0x80;
}

static bool cs42l67_writeable_reg(struct device *dev, unsigned int reg)
{
	return cs42l67_readable_reg(dev, reg) || reg == CS42L67_RESET ||
	       reg == CS42L67_TRIM;
}

static bool cs42l67_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config cs42l67_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = CS42L67_MAX_REG,
	.readable_reg = cs42l67_readable_reg,
	.writeable_reg = cs42l67_writeable_reg,
	.volatile_reg = cs42l67_volatile_reg,
	/* no cache: the chip is reset on every power-up */
	.cache_type = REGCACHE_NONE,
};

static const unsigned int cs42l67_adc_gain_regs[] = {
	CS42L67_ADC1A_GAIN, CS42L67_ADC1B_GAIN, CS42L67_ADC2A_GAIN, CS42L67_ADC2B_GAIN,
};

/* iOS' PGA encoding (route fn 0x…86ec1dc, decomp ~l.1427-1438) */
static unsigned int cs42l67_adc_vol_to_reg(unsigned int vol)
{
	unsigned int stage = 0;

	if (vol >= 0x34) {
		stage = 0x80;
		vol -= 0x28;
	} else if (vol >= 0x20) {
		stage = 0x40;
		vol -= 0x14;
	}
	return ((vol + 0x34) & 0x3f) | stage;
}

static unsigned int cs42l67_reg_to_adc_vol(unsigned int val)
{
	int vol = sign_extend32(val & 0x3f, 5) + 12;

	if (val & 0x80)
		vol += 40;
	else if (val & 0x40)
		vol += 20;
	return clamp(vol, 0, ADC_VOL_MAX);
}

/* ---- power sequencing (AppleCS42L67Audio enable 0x…86ebf04 / disable 0x…86ee3b8) ---- */

static int cs42l67_power_up(struct cs42l67 *cs)
{
	struct regmap *map = cs->regmap;
	unsigned int rev, i;
	int ret;

	if (cs->powered)
		return 0;

	ret = clk_prepare_enable(cs->mclk);
	if (ret)
		return ret;

	/* reset pulse: iOS does this on a cold enable only */
	regmap_write(map, CS42L67_RESET, RESET_HOLD);
	usleep_range(10000, 11000);
	regmap_write(map, CS42L67_RESET, 0);
	regmap_read(map, CS42L67_REVID, &rev);
	regmap_write(map, CS42L67_TRIM, 0x88);
	regmap_update_bits(map, CS42L67_ID_CTL, 0xc4, 0x04);
	regmap_write(map, CS42L67_XSP_CFG1, 0x11);
	regmap_write(map, CS42L67_XSP_CFG2, 0x11);
	regmap_write(map, CS42L67_VSP_CFG, 0x00);
	regmap_update_bits(map, CS42L67_XSP_CTL, XSP_PDN, 0);
	regmap_update_bits(map, CS42L67_VSP_CTL, XSP_PDN, 0);
	regmap_update_bits(map, CS42L67_PWR, PWR_PDN, 0);
	regmap_write(map, CS42L67_ANA_201, 0xe0);
	regmap_write(map, CS42L67_ANA_202, 0x00);
	regmap_write(map, CS42L67_ANA_239, 0x00);
	regmap_update_bits(map, CS42L67_OUT_CTL, 0x20, 0x20);
	/* iOS sets the rate here (0x…86eeb58); 0 = "unknown" on the CS42L73 */
	regmap_update_bits(map, CS42L67_SP_FS, SP_FS_MASK, SP_FS_48K);
	regmap_write(map, CS42L67_MISC_130, 0xa0);

	/*
	 * The idle state iOS' route function leaves behind: all ADC
	 * channels, both port directions, both paths and all outputs off,
	 * outputs muted, no rate converter. DAPM turns pieces on from here.
	 */
	regmap_update_bits(map, CS42L67_PATH_PDN, PATH_PDN_OUT | PATH_PDN_IN,
			   PATH_PDN_OUT | PATH_PDN_IN);
	regmap_update_bits(map, CS42L67_XSP_CTL, XSP_TX_PDN | XSP_RX_PDN,
			   XSP_TX_PDN | XSP_RX_PDN);
	regmap_update_bits(map, CS42L67_VSP_CTL, XSP_TX_PDN | XSP_RX_PDN,
			   XSP_TX_PDN | XSP_RX_PDN);
	regmap_write(map, CS42L67_SRC_RATE, 0x0c);
	regmap_update_bits(map, CS42L67_XSP_SRC, 0x80, 0);
	regmap_write(map, CS42L67_ADC1_PDN, ADC_PDN_IDLE);
	regmap_write(map, CS42L67_ADC1_DMIC, 0);
	regmap_write(map, CS42L67_ADC2_PDN, ADC_PDN_IDLE);
	regmap_write(map, CS42L67_ADC2_DMIC, 0);
	/*
	 * PGA = ADC_VOL_DEFAULT. iOS' ctl 0x16f ("boost") defaults to 1, but
	 * with that bit set the n61 ADCs capture nothing but digital zero
	 * (hw 2026-10-22: all four, any PGA value), so leave it clear.
	 */
	for (i = 0; i < ARRAY_SIZE(cs42l67_adc_gain_regs); i++) {
		regmap_write(map, cs42l67_adc_gain_regs[i],
			     cs42l67_adc_vol_to_reg(ADC_VOL_DEFAULT));
		regmap_write(map, cs42l67_adc_gain_regs[i] + 2, 0);
	}
	/*
	 * The n61 mic map as default input selects (item 0 of every mux):
	 * ADC1A = AIN1 (bottom), ADC1B = AIN2 (headset), ADC2A = AIN5 (rear),
	 * ADC2B = AIN6 (front). Written before DAPM reads the muxes (see
	 * cs42l67_component_probe), so its view matches the chip.
	 */
	regmap_write(map, CS42L67_ADC1_MUX, 0 << ADC_MUX_A_SHIFT | 1 << ADC_MUX_B_SHIFT);
	regmap_write(map, CS42L67_ADC2_MUX, 0 << ADC_MUX_A_SHIFT | 1 << ADC_MUX_B_SHIFT);
	regmap_write(map, CS42L67_DSP_PDN, 0x4c);
	/*
	 * Both ASP TX channels unused, as iOS' route function leaves them;
	 * the sources default to TX1 = ADC1A (bottom), TX2 = ADC2A (rear)
	 */
	regmap_write(map, CS42L67_ASP_MISC, 0);
	for (i = 0; i < 2; i++) {
		regmap_write(map, CS42L67_ASP_TX(i, 0), i ? ASP_SRC_ADC2A : ASP_SRC_ADC1A);
		regmap_write(map, CS42L67_ASP_TX(i, 1), 0x3f);
		regmap_write(map, CS42L67_ASP_TX(i, 2), 0x3f | i << 6);
		regmap_write(map, CS42L67_ASP_TX(i, 3), 0x3f | i << 6);
		regmap_write(map, CS42L67_ASP_TX(i, 4), 0x3f | i << 6);
	}
	regmap_write(map, CS42L67_OUTA_VOL, OUT_VOL_MUTE);
	regmap_write(map, CS42L67_OUTA_VOL_R, OUT_VOL_MUTE);
	regmap_write(map, CS42L67_OUTB_VOL, OUT_VOL_MUTE);
	regmap_write(map, CS42L67_OUTB_VOL_R, OUT_VOL_MUTE);
	regmap_write(map, CS42L67_OUTA_AGAIN, OUT_AGAIN_MUTE);
	regmap_write(map, CS42L67_OUTA_AGAIN_R, OUT_AGAIN_MUTE);
	regmap_write(map, CS42L67_OUTC_GAIN, OUT_AGAIN_MUTE);
	regmap_write(map, CS42L67_OUTD_GAIN, OUT_AGAIN_MUTE);
	regmap_write(map, CS42L67_OUT_PDN1, 0x28);
	regmap_write(map, CS42L67_OUT_PDN2, 0x60);
	regmap_update_bits(map, CS42L67_OUT_CTL, OUT_CTL_IDLE, OUT_CTL_IDLE);

	cs->powered = true;
	dev_dbg(cs->dev, "powered up, rev %#x\n", rev);
	return 0;
}

static void cs42l67_power_down(struct cs42l67 *cs)
{
	struct regmap *map = cs->regmap;
	unsigned int val;
	int ret;

	if (!cs->powered)
		return;

	regmap_write(map, CS42L67_OUTA_VOL, OUT_VOL_MUTE);
	regmap_write(map, CS42L67_OUTA_VOL_R, OUT_VOL_MUTE);
	regmap_write(map, CS42L67_OUTB_VOL, OUT_VOL_MUTE);
	regmap_write(map, CS42L67_OUTB_VOL_R, OUT_VOL_MUTE);
	regmap_write(map, CS42L67_OUTA_AGAIN, OUT_AGAIN_MUTE);
	regmap_write(map, CS42L67_OUTA_AGAIN_R, OUT_AGAIN_MUTE);
	regmap_write(map, CS42L67_OUTC_GAIN, OUT_AGAIN_MUTE);
	regmap_write(map, CS42L67_OUTD_GAIN, OUT_AGAIN_MUTE);
	regmap_write(map, CS42L67_OUT_PDN1, 0x28);
	regmap_write(map, CS42L67_OUT_PDN2, 0x60);
	regmap_update_bits(map, CS42L67_PWR, PWR_PDN, PWR_PDN);

	/* iOS polls every 10 ms, 100 tries; PDN_DONE needs MCLK */
	ret = regmap_read_poll_timeout(map, CS42L67_STATUS, val,
				       val & STATUS_PDN_DONE, 10000, 1000000);
	if (ret)
		dev_warn(cs->dev, "timed out waiting for codec shutdown (no MCLK?)\n");

	regmap_update_bits(map, CS42L67_PWR, PWR_PDN2, PWR_PDN2);
	usleep_range(10000, 11000);
	regmap_write(map, CS42L67_RESET, RESET_HOLD);
	clk_disable_unprepare(cs->mclk);
	cs->powered = false;
}

static int cs42l67_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	struct cs42l67 *cs = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(component);
	enum snd_soc_bias_level old = snd_soc_dapm_get_bias_level(dapm);
	int ret;

	if (cs->inert)
		return 0;

	if (level == SND_SOC_BIAS_STANDBY && old == SND_SOC_BIAS_OFF) {
		ret = cs42l67_power_up(cs);
		if (ret)
			return ret;
	} else if (level == SND_SOC_BIAS_OFF && old != SND_SOC_BIAS_OFF) {
		cs42l67_power_down(cs);
	}

	return 0;
}

/*
 * Output C = the earpiece (ADT aout1-rcvr; re/CODEC-EARPIECE.md): powered by 0x24f bit 5, gain 0x252 (2 x dB,
 * signed, -102..+12 dB), fed by its own mixer pair 0x12d/0x12e (L) and 0x132/0x133 (R). On power-up the writes of
 * iOS' route function, in its order: mixer headers with sidetone off, voice source = XSP RX channel 1, the headphone
 * mixers off, the earpiece mixers' voice input at 0 dB (their main input muted), sidetone/ANC off, headphone and
 * hearing-aid outputs muted; 0x243 bit 3 stays set (iOS clears it only for headphones / line out).
 */
static int cs42l67_rcv_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct cs42l67 *cs = snd_soc_component_get_drvdata(c);

	if (event == SND_SOC_DAPM_PRE_PMU) {
		snd_soc_component_write(c, 0x121, 0x00);
		snd_soc_component_write(c, 0x126, 0x20);
		snd_soc_component_write(c, 0x12b, 0x40);
		snd_soc_component_write(c, 0x130, 0x60);
		snd_soc_component_write(c, CS42L67_VOICE_SRC, 0x11);
		snd_soc_component_write(c, 0x123, 0x3f);
		snd_soc_component_write(c, 0x124, 0x3f);
		snd_soc_component_write(c, 0x128, 0x7f);
		snd_soc_component_write(c, 0x129, 0x7f);
		snd_soc_component_write(c, 0x12e, 0x80);
		snd_soc_component_write(c, 0x133, 0x80);
		snd_soc_component_write(c, 0x12d, 0xbf);
		snd_soc_component_write(c, 0x132, 0xbf);
		snd_soc_component_write(c, CS42L67_DSP_PDN, 0x4c);
		snd_soc_component_write(c, 0x122, 0x3f);
		snd_soc_component_write(c, 0x127, 0x3f);
		snd_soc_component_write(c, 0x131, 0x3f);
		snd_soc_component_update_bits(c, 0x054, BIT(1), BIT(1));
		snd_soc_component_write(c, CS42L67_OUTA_AGAIN, OUT_AGAIN_MUTE);
		snd_soc_component_write(c, CS42L67_OUTC_GAIN, cs->rcv_gain & 0xff);
		snd_soc_component_write(c, CS42L67_OUTD_GAIN, OUT_AGAIN_MUTE);
		snd_soc_component_update_bits(c, CS42L67_OUT_PDN1, 0xa8, 0x28);
	} else if (event == SND_SOC_DAPM_POST_PMU) {
		snd_soc_component_update_bits(c, 0x251, 0x03, 0x01);
		cs->rcv_on = true;
	} else if (event == SND_SOC_DAPM_PRE_PMD) {
		cs->rcv_on = false;
		snd_soc_component_write(c, 0x12d, 0xbf);
		snd_soc_component_write(c, 0x12e, 0xbf);
		snd_soc_component_write(c, 0x132, 0xbf);
		snd_soc_component_write(c, 0x133, 0xbf);
		snd_soc_component_write(c, CS42L67_OUTC_GAIN, OUT_AGAIN_MUTE);
	}
	return 0;
}

static const struct snd_kcontrol_new hp_switch = SOC_DAPM_SINGLE_VIRT("Switch", 1);
static const struct snd_kcontrol_new rcv_switch = SOC_DAPM_SINGLE_VIRT("Switch", 0);

/* ---- DAPM ---- */

/*
 * MICBIAS outputs are 2-bit fields (0 off, 1 low, 2 = 2.7 V); n61's ADT asks
 * for 2.7 V on all three bias outputs it uses. iOS waits 50 ms after
 * changing them.
 */
static int cs42l67_micbias_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	unsigned int mask = 3 << w->shift;
	unsigned int val = SND_SOC_DAPM_EVENT_ON(event) ? MICBIAS_2V7 << w->shift : 0;

	snd_soc_component_update_bits(component, w->reg, mask, val);
	if (SND_SOC_DAPM_EVENT_ON(event))
		msleep(50);
	return 0;
}

#define CS42L67_MICBIAS(wname, wreg, wshift) \
{	.id = snd_soc_dapm_supply, .name = wname, .reg = wreg, \
	.shift = wshift, .mask = 0, .event = cs42l67_micbias_event, \
	.event_flags = SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD, }

/* 0x234 / 0x235: iOS writes the whole byte, 0x80, while the pair is in use */
#define CS42L67_ADC_EN(wname, wreg) \
{	.id = snd_soc_dapm_supply, .name = wname, .reg = wreg, \
	.shift = ADC_EN_SHIFT, .mask = 0xf, .on_val = 0x8, .off_val = 0x7, }

static int cs42l67_out_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	/* iOS: OUT_CTL bit3 and the analog gain only while an output plays */
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_component_update_bits(component, CS42L67_OUT_CTL,
					      OUT_CTL_IDLE, 0);
		snd_soc_component_write(component, CS42L67_OUTA_AGAIN, 0);
		snd_soc_component_write(component, CS42L67_OUTA_AGAIN_R, 0);
	} else {
		snd_soc_component_write(component, CS42L67_OUTA_AGAIN, OUT_AGAIN_MUTE);
		snd_soc_component_write(component, CS42L67_OUTA_AGAIN_R, OUT_AGAIN_MUTE);
		snd_soc_component_update_bits(component, CS42L67_OUT_CTL,
					      OUT_CTL_IDLE, OUT_CTL_IDLE);
	}
	return 0;
}

/* ASP TX channel on: the two attenuation-like bytes to 0, as iOS does */
static int cs42l67_asp_tx_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	unsigned int ch = w->shift, on = SND_SOC_DAPM_EVENT_ON(event);

	snd_soc_component_write(component, CS42L67_ASP_TX(ch, 1), on ? 0 : 0x3f);
	snd_soc_component_write(component, CS42L67_ASP_TX(ch, 4),
				(on ? 0 : 0x3f) | ch << 6);
	return 0;
}

/*
 * ADC input muxes: the values iOS writes for each ADT "ainN" input (see the
 * table at the top). Item 0 of each is the n61 default.
 */
static const char * const adc1a_texts[] = { "AIN1", "AIN3" };
static const unsigned int adc1a_values[] = { 0, 2 };
static const char * const adc1b_texts[] = { "HSMIC", "AIN4" };
static const unsigned int adc1b_values[] = { 1, 3 };
static const char * const adc2a_texts[] = { "AIN5", "AIN7" };
static const unsigned int adc2a_values[] = { 0, 2 };
static const char * const adc2b_texts[] = { "AIN6", "AIN8" };
static const unsigned int adc2b_values[] = { 1, 3 };

static SOC_VALUE_ENUM_SINGLE_DECL(adc1a_enum, CS42L67_ADC1_MUX, ADC_MUX_A_SHIFT, 3,
				  adc1a_texts, adc1a_values);
static SOC_VALUE_ENUM_SINGLE_DECL(adc1b_enum, CS42L67_ADC1_MUX, ADC_MUX_B_SHIFT, 3,
				  adc1b_texts, adc1b_values);
static SOC_VALUE_ENUM_SINGLE_DECL(adc2a_enum, CS42L67_ADC2_MUX, ADC_MUX_A_SHIFT, 3,
				  adc2a_texts, adc2a_values);
static SOC_VALUE_ENUM_SINGLE_DECL(adc2b_enum, CS42L67_ADC2_MUX, ADC_MUX_B_SHIFT, 3,
				  adc2b_texts, adc2b_values);

static const struct snd_kcontrol_new adc1a_mux = SOC_DAPM_ENUM("ADC1A Input", adc1a_enum);
static const struct snd_kcontrol_new adc1b_mux = SOC_DAPM_ENUM("ADC1B Input", adc1b_enum);
static const struct snd_kcontrol_new adc2a_mux = SOC_DAPM_ENUM("ADC2A Input", adc2a_enum);
static const struct snd_kcontrol_new adc2b_mux = SOC_DAPM_ENUM("ADC2B Input", adc2b_enum);

/* XSP TX channel sources */
static const char * const xsp_tx_texts[] = { "Off", "ADC1A", "ADC1B", "ADC2A", "ADC2B" };
static const unsigned int xsp_tx_values[] = { 0, SRC_ADC1A, SRC_ADC1B, SRC_ADC2A, SRC_ADC2B };

#define XSP_TX_ENUM(n) \
	static SOC_VALUE_ENUM_SINGLE_DECL(xsp_tx##n##_enum, CS42L67_XSP_TX_SRC((n) - 1), \
					  0, 0x3f, xsp_tx_texts, xsp_tx_values); \
	static const struct snd_kcontrol_new xsp_tx##n##_mux = \
		SOC_DAPM_ENUM("XSP TX" #n " Source", xsp_tx##n##_enum)

static const char * const asp_tx_texts[] = { "ADC1A", "ADC1B", "ADC2A", "ADC2B" };
static SOC_ENUM_SINGLE_DECL(asp_tx1_enum, CS42L67_ASP_TX(0, 0), 5, asp_tx_texts);
static SOC_ENUM_SINGLE_DECL(asp_tx2_enum, CS42L67_ASP_TX(1, 0), 5, asp_tx_texts);
static const struct snd_kcontrol_new asp_tx1_mux = SOC_DAPM_ENUM("ASP TX1 Source", asp_tx1_enum);
static const struct snd_kcontrol_new asp_tx2_mux = SOC_DAPM_ENUM("ASP TX2 Source", asp_tx2_enum);

XSP_TX_ENUM(1);
XSP_TX_ENUM(2);
XSP_TX_ENUM(3);
XSP_TX_ENUM(4);

static const struct snd_soc_dapm_widget cs42l67_widgets[] = {
	SND_SOC_DAPM_INPUT("AIN1"),
	SND_SOC_DAPM_INPUT("AIN3"),
	SND_SOC_DAPM_INPUT("AIN4"),
	SND_SOC_DAPM_INPUT("AIN5"),
	SND_SOC_DAPM_INPUT("AIN6"),
	SND_SOC_DAPM_INPUT("AIN7"),
	SND_SOC_DAPM_INPUT("AIN8"),
	SND_SOC_DAPM_INPUT("HSMIC"),	/* AIN2 */

	CS42L67_MICBIAS("MICBIAS1", CS42L67_MICBIAS1, 0),
	CS42L67_MICBIAS("MICBIAS3", CS42L67_MICBIAS34, 0),
	CS42L67_MICBIAS("MICBIAS4", CS42L67_MICBIAS34, 2),

	SND_SOC_DAPM_MUX("ADC1A Mux", SND_SOC_NOPM, 0, 0, &adc1a_mux),
	SND_SOC_DAPM_MUX("ADC1B Mux", SND_SOC_NOPM, 0, 0, &adc1b_mux),
	SND_SOC_DAPM_MUX("ADC2A Mux", SND_SOC_NOPM, 0, 0, &adc2a_mux),
	SND_SOC_DAPM_MUX("ADC2B Mux", SND_SOC_NOPM, 0, 0, &adc2b_mux),

	SND_SOC_DAPM_SUPPLY("Input Path", CS42L67_PATH_PDN, 6, 1, NULL, 0),
	SND_SOC_DAPM_ADC("ADC1A", NULL, CS42L67_ADC1_PDN, 6, 1),
	SND_SOC_DAPM_ADC("ADC1B", NULL, CS42L67_ADC1_PDN, 7, 1),
	SND_SOC_DAPM_ADC("ADC2A", NULL, CS42L67_ADC2_PDN, 6, 1),
	SND_SOC_DAPM_ADC("ADC2B", NULL, CS42L67_ADC2_PDN, 7, 1),

	SND_SOC_DAPM_MUX("XSP TX1 Mux", SND_SOC_NOPM, 0, 0, &xsp_tx1_mux),
	SND_SOC_DAPM_MUX("XSP TX2 Mux", SND_SOC_NOPM, 0, 0, &xsp_tx2_mux),
	SND_SOC_DAPM_MUX("XSP TX3 Mux", SND_SOC_NOPM, 0, 0, &xsp_tx3_mux),
	SND_SOC_DAPM_MUX("XSP TX4 Mux", SND_SOC_NOPM, 0, 0, &xsp_tx4_mux),
	SND_SOC_DAPM_AIF_OUT("XSP TX", "XSP Capture", 0, CS42L67_XSP_CTL, 6, 1),

	/* the mic path iOS uses: ASP (I2S to MCA0), 2 channels */
	CS42L67_ADC_EN("ADC1 Enable", CS42L67_ADC1_EN),
	CS42L67_ADC_EN("ADC2 Enable", CS42L67_ADC2_EN),
	SND_SOC_DAPM_MUX("ASP TX1 Mux", SND_SOC_NOPM, 0, 0, &asp_tx1_mux),
	SND_SOC_DAPM_MUX("ASP TX2 Mux", SND_SOC_NOPM, 0, 0, &asp_tx2_mux),
	SND_SOC_DAPM_PGA_E("ASP TX1 Path", SND_SOC_NOPM, 0, 0, NULL, 0,
			   cs42l67_asp_tx_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("ASP TX2 Path", SND_SOC_NOPM, 1, 0, NULL, 0,
			   cs42l67_asp_tx_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_AIF_OUT("ASP TX", "ASP Capture", 0, SND_SOC_NOPM, 0, 0),

	/* experimental: XSP RX channel 1 -> "voice" L/R -> output A */
	SND_SOC_DAPM_AIF_IN("XSP RX", "XSP Playback", 0, CS42L67_XSP_CTL, 4, 1),
	SND_SOC_DAPM_SUPPLY("Output Path", CS42L67_PATH_PDN, 7, 1, NULL, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV_E("OUTA", CS42L67_OUT_PDN1, 5, 1, NULL, 0,
			       cs42l67_out_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("HPOUT"),

	/* which output the playback goes to: "Headphone" (output A) / "Earpiece" (output C) Playback Switch */
	SND_SOC_DAPM_SWITCH("Headphone", SND_SOC_NOPM, 0, 0, &hp_switch),
	SND_SOC_DAPM_SWITCH("Earpiece", SND_SOC_NOPM, 0, 0, &rcv_switch),
	SND_SOC_DAPM_OUT_DRV_E("OUTC", CS42L67_OUT_PDN2, 5, 1, NULL, 0, cs42l67_rcv_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("RCV"),
};

static const struct snd_soc_dapm_route cs42l67_routes[] = {
	{ "ADC1A Mux", "AIN1", "AIN1" },
	{ "ADC1A Mux", "AIN3", "AIN3" },
	{ "ADC1B Mux", "HSMIC", "HSMIC" },
	{ "ADC1B Mux", "AIN4", "AIN4" },
	{ "ADC2A Mux", "AIN5", "AIN5" },
	{ "ADC2A Mux", "AIN7", "AIN7" },
	{ "ADC2B Mux", "AIN6", "AIN6" },
	{ "ADC2B Mux", "AIN8", "AIN8" },

	{ "ADC1A", NULL, "ADC1A Mux" },
	{ "ADC1B", NULL, "ADC1B Mux" },
	{ "ADC2A", NULL, "ADC2A Mux" },
	{ "ADC2B", NULL, "ADC2B Mux" },
	{ "ADC1A", NULL, "Input Path" },
	{ "ADC1B", NULL, "Input Path" },
	{ "ADC2A", NULL, "Input Path" },
	{ "ADC2B", NULL, "Input Path" },

	{ "XSP TX1 Mux", "ADC1A", "ADC1A" },
	{ "XSP TX1 Mux", "ADC1B", "ADC1B" },
	{ "XSP TX1 Mux", "ADC2A", "ADC2A" },
	{ "XSP TX1 Mux", "ADC2B", "ADC2B" },
	{ "XSP TX", NULL, "XSP TX1 Mux" },
	{ "XSP TX2 Mux", "ADC1A", "ADC1A" },
	{ "XSP TX2 Mux", "ADC1B", "ADC1B" },
	{ "XSP TX2 Mux", "ADC2A", "ADC2A" },
	{ "XSP TX2 Mux", "ADC2B", "ADC2B" },
	{ "XSP TX", NULL, "XSP TX2 Mux" },
	{ "XSP TX3 Mux", "ADC1A", "ADC1A" },
	{ "XSP TX3 Mux", "ADC1B", "ADC1B" },
	{ "XSP TX3 Mux", "ADC2A", "ADC2A" },
	{ "XSP TX3 Mux", "ADC2B", "ADC2B" },
	{ "XSP TX", NULL, "XSP TX3 Mux" },
	{ "XSP TX4 Mux", "ADC1A", "ADC1A" },
	{ "XSP TX4 Mux", "ADC1B", "ADC1B" },
	{ "XSP TX4 Mux", "ADC2A", "ADC2A" },
	{ "XSP TX4 Mux", "ADC2B", "ADC2B" },
	{ "XSP TX", NULL, "XSP TX4 Mux" },

	{ "ADC1A", NULL, "ADC1 Enable" },
	{ "ADC1B", NULL, "ADC1 Enable" },
	{ "ADC2A", NULL, "ADC2 Enable" },
	{ "ADC2B", NULL, "ADC2 Enable" },
	{ "ASP TX1 Mux", "ADC1A", "ADC1A" },
	{ "ASP TX1 Mux", "ADC1B", "ADC1B" },
	{ "ASP TX1 Mux", "ADC2A", "ADC2A" },
	{ "ASP TX1 Mux", "ADC2B", "ADC2B" },
	{ "ASP TX2 Mux", "ADC1A", "ADC1A" },
	{ "ASP TX2 Mux", "ADC1B", "ADC1B" },
	{ "ASP TX2 Mux", "ADC2A", "ADC2A" },
	{ "ASP TX2 Mux", "ADC2B", "ADC2B" },
	{ "ASP TX1 Path", NULL, "ASP TX1 Mux" },
	{ "ASP TX2 Path", NULL, "ASP TX2 Mux" },
	{ "ASP TX", NULL, "ASP TX1 Path" },
	{ "ASP TX", NULL, "ASP TX2 Path" },

	{ "DAC", NULL, "XSP RX" },
	{ "DAC", NULL, "Output Path" },
	{ "Headphone", "Switch", "DAC" },
	{ "OUTA", NULL, "Headphone" },
	{ "HPOUT", NULL, "OUTA" },
	{ "Earpiece", "Switch", "DAC" },
	{ "OUTC", NULL, "Earpiece" },
	{ "RCV", NULL, "OUTC" },
};

static int cs42l67_adc_vol_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;

	ucontrol->value.integer.value[0] =
		cs42l67_reg_to_adc_vol(snd_soc_component_read(component, mc->reg));
	return 0;
}

static int cs42l67_adc_vol_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	long vol = ucontrol->value.integer.value[0];

	if (vol < 0 || vol > ADC_VOL_MAX)
		return -EINVAL;
	return snd_soc_component_update_bits(component, mc->reg, 0xff,
					     cs42l67_adc_vol_to_reg(vol));
}

/* Earpiece volume: control 0..228 = -102..+12 dB in 0.5 dB steps */
#define RCV_GAIN_MIN	(-204)
#define RCV_GAIN_MAX	24
static const DECLARE_TLV_DB_SCALE(rcv_vol_tlv, -10200, 50, 0);

static int cs42l67_rcv_vol_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *u)
{
	struct cs42l67 *cs = snd_soc_component_get_drvdata(snd_kcontrol_chip(kcontrol));

	u->value.integer.value[0] = cs->rcv_gain - RCV_GAIN_MIN;
	return 0;
}

static int cs42l67_rcv_vol_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *u)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l67 *cs = snd_soc_component_get_drvdata(c);
	long v = u->value.integer.value[0];

	if (v < 0 || v > RCV_GAIN_MAX - RCV_GAIN_MIN)
		return -EINVAL;
	if (cs->rcv_gain == v + RCV_GAIN_MIN)
		return 0;
	cs->rcv_gain = v + RCV_GAIN_MIN;
	if (cs->rcv_on)
		snd_soc_component_write(c, CS42L67_OUTC_GAIN, cs->rcv_gain & 0xff);
	return 1;
}

/* bring-up aid: "Debug Reg" selects a page:reg, "Debug Val" reads / writes it (amixer cset) */
static int cs42l67_dbg_reg_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *u)
{
	u->value.integer.value[0] = ((struct cs42l67 *)snd_soc_component_get_drvdata(snd_kcontrol_chip(kcontrol)))->dbg_reg;
	return 0;
}

static int cs42l67_dbg_reg_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *u)
{
	struct cs42l67 *cs = snd_soc_component_get_drvdata(snd_kcontrol_chip(kcontrol));

	cs->dbg_reg = u->value.integer.value[0];
	return 1;
}

static int cs42l67_dbg_val_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *u)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l67 *cs = snd_soc_component_get_drvdata(c);

	u->value.integer.value[0] = cs->powered ? snd_soc_component_read(c, cs->dbg_reg) : 0;
	return 0;
}

static int cs42l67_dbg_val_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *u)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kcontrol);
	struct cs42l67 *cs = snd_soc_component_get_drvdata(c);

	if (!cs->powered)
		return -EBUSY;
	snd_soc_component_write(c, cs->dbg_reg, u->value.integer.value[0]);
	return 1;
}

/* iOS' declared scale for ctl 0x170; the two stage steps are inferred */
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -600, 50, 0);

#define CS42L67_ADC_VOL(xname, xreg) \
	SOC_SINGLE_EXT_TLV(xname, xreg, 0, ADC_VOL_MAX, 0, \
			   cs42l67_adc_vol_get, cs42l67_adc_vol_put, adc_vol_tlv)

static const struct snd_kcontrol_new cs42l67_controls[] = {
	CS42L67_ADC_VOL("ADC1A Capture Volume", CS42L67_ADC1A_GAIN),
	CS42L67_ADC_VOL("ADC1B Capture Volume", CS42L67_ADC1B_GAIN),
	CS42L67_ADC_VOL("ADC2A Capture Volume", CS42L67_ADC2A_GAIN),
	CS42L67_ADC_VOL("ADC2B Capture Volume", CS42L67_ADC2B_GAIN),
	/* the whole PGA byte: [5:0] fine gain, bit6/bit7 the stages */
	SOC_SINGLE("ADC1A Gain Raw", CS42L67_ADC1A_GAIN, 0, 0xff, 0),
	SOC_SINGLE("ADC1B Gain Raw", CS42L67_ADC1B_GAIN, 0, 0xff, 0),
	SOC_SINGLE("ADC2A Gain Raw", CS42L67_ADC2A_GAIN, 0, 0xff, 0),
	SOC_SINGLE("ADC2B Gain Raw", CS42L67_ADC2B_GAIN, 0, 0xff, 0),
	SOC_SINGLE("ADC1A Boost Switch", CS42L67_ADC1A_BOOST, 7, 1, 0),
	SOC_SINGLE("ADC1B Boost Switch", CS42L67_ADC1B_BOOST, 7, 1, 0),
	SOC_SINGLE("ADC2A Boost Switch", CS42L67_ADC2A_BOOST, 7, 1, 0),
	SOC_SINGLE("ADC2B Boost Switch", CS42L67_ADC2B_BOOST, 7, 1, 0),
	/* 0x40 = mute (what iOS writes); iOS maps volume v to (v - 0x37) & 0x7f */
	SOC_DOUBLE_R("OUTA Volume Raw", CS42L67_OUTA_VOL, CS42L67_OUTA_VOL_R,
		     0, 0x7f, 0),
	SOC_SINGLE_EXT_TLV("Earpiece Playback Volume", SND_SOC_NOPM, 0, RCV_GAIN_MAX - RCV_GAIN_MIN, 0,
			   cs42l67_rcv_vol_get, cs42l67_rcv_vol_put, rcv_vol_tlv),
	SOC_SINGLE_EXT("Debug Reg", SND_SOC_NOPM, 0, 0x37f, 0, cs42l67_dbg_reg_get, cs42l67_dbg_reg_put),
	SOC_SINGLE_EXT("Debug Val", SND_SOC_NOPM, 0, 0xff, 0, cs42l67_dbg_val_get, cs42l67_dbg_val_put),
};

/* ---- DAI ---- */

static int cs42l67_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/*
	 * As iOS runs them, the AP's MCA provides BCLK/FSYNC on both ports:
	 * XSP = DSP mode A on MCA2's TDM bus, ASP = I2S from MCA0
	 */
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC ||
	    (fmt & SND_SOC_DAIFMT_FORMAT_MASK) !=
	    (dai->id == CS42L67_DAI_ASP ? SND_SOC_DAIFMT_I2S : SND_SOC_DAIFMT_DSP_A) ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;
	return 0;
}

static int cs42l67_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct cs42l67 *cs = snd_soc_component_get_drvdata(component);
	unsigned int bytes = 2, ch = params_channels(params);
	unsigned int i, off;
	u32 mask = 0;

	if (cs->inert)
		return -ENODEV;

	if (params_rate(params) != 48000 ||
	    params_format(params) != SNDRV_PCM_FORMAT_S16_LE)
		return -EINVAL;

	/* ASP: fixed 2 x 16-bit I2S, the sources are DAPM muxes */
	if (dai->id == CS42L67_DAI_ASP)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* RX channel 1 listens to one slot; "voice" L and R both take it */
		snd_soc_component_write(component, CS42L67_XSP_RX_POS(0),
					((bytes - 1) << 5) | (cs->rx_slot * bytes));
		snd_soc_component_write(component, CS42L67_VOICE_SRC, 0x11);
		return 0;
	}

	/*
	 * Channel n -> slot tx_slot + n. Only the channels that have a source
	 * mux and fit in the frame drive the bus; with more capture channels
	 * than that (e.g. 8, to see every slot) the rest of the frame is
	 * whatever else drives it, which is the point of such a capture.
	 */
	for (i = 0; i < CS42L67_TX_CHANNELS; i++) {
		if (i < ch && i < CS42L67_TX_ROUTED &&
		    cs->tx_slot + i < CS42L67_FRAME_SLOTS) {
			off = (cs->tx_slot + i) * bytes;
			mask |= GENMASK(bytes - 1, 0) << off;
			snd_soc_component_write(component, CS42L67_XSP_TX_POS(i), off);
		} else {
			snd_soc_component_write(component, CS42L67_XSP_TX_POS(i),
						TX_POS_OFF);
		}
	}
	for (i = 0; i < 4; i++)
		snd_soc_component_write(component, CS42L67_XSP_TX_MASK(i),
					(mask >> (24 - 8 * i)) & 0xff);
	return 0;
}

static const struct snd_soc_dai_ops cs42l67_dai_ops = {
	.set_fmt = cs42l67_set_fmt,
	.hw_params = cs42l67_hw_params,
};

static struct snd_soc_dai_driver cs42l67_dais[] = {
	{
	.name = "cs42l67-xsp",
	.id = CS42L67_DAI_XSP,
	.playback = {
		.stream_name = "XSP Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "XSP Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cs42l67_dai_ops,
	},
	{
	.name = "cs42l67-asp",
	.id = CS42L67_DAI_ASP,
	.capture = {
		.stream_name = "ASP Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cs42l67_dai_ops,
	},
};

/* ---- jack (optional, unverified register) ---- */

static void cs42l67_jack_work(struct work_struct *work)
{
	struct cs42l67 *cs = container_of(work, struct cs42l67, jack_work.work);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(cs->component);
	unsigned int val;
	int status = 0;

	if (snd_soc_dapm_get_bias_level(dapm) != SND_SOC_BIAS_OFF &&
	    !regmap_read(cs->regmap, CS42L67_HSDET_STATUS, &val)) {
		if (val & 0x60)
			status = SND_JACK_HEADPHONE;
		if (status != cs->jack_status) {
			dev_info(cs->dev, "jack status %#x -> %s\n", val,
				 status ? "plugged" : "unplugged");
			cs->jack_status = status;
			snd_soc_jack_report(&cs->jack, status, SND_JACK_HEADPHONE);
		}
	}

	schedule_delayed_work(&cs->jack_work, msecs_to_jiffies(500));
}

static int cs42l67_component_probe(struct snd_soc_component *component)
{
	struct cs42l67 *cs = snd_soc_component_get_drvdata(component);
	int ret;

	cs->component = component;
	if (cs->inert)
		return 0;

	/*
	 * Cold-start the chip now, before the core adds the routes: DAPM
	 * reads the input/source muxes from the chip once, when it adds them,
	 * and the power-up reset would otherwise change them behind its back.
	 * set_bias_level() then finds the chip already up.
	 */
	ret = cs42l67_power_up(cs);
	if (ret)
		return ret;

	if (!jack_poll)
		return 0;

	ret = snd_soc_card_jack_new(component->card, "Headphone Jack",
				    SND_JACK_HEADPHONE, &cs->jack);
	if (ret) {
		/* a failed probe gets no remove() */
		cs42l67_power_down(cs);
		return ret;
	}
	INIT_DELAYED_WORK(&cs->jack_work, cs42l67_jack_work);
	schedule_delayed_work(&cs->jack_work, msecs_to_jiffies(500));
	return 0;
}

static void cs42l67_component_remove(struct snd_soc_component *component)
{
	struct cs42l67 *cs = snd_soc_component_get_drvdata(component);

	if (!cs->inert && jack_poll)
		cancel_delayed_work_sync(&cs->jack_work);
	/* normally already done by the card's BIAS_OFF */
	if (!cs->inert)
		cs42l67_power_down(cs);
}

static const struct snd_soc_component_driver cs42l67_component = {
	.probe = cs42l67_component_probe,
	.remove = cs42l67_component_remove,
	.set_bias_level = cs42l67_set_bias_level,
	.controls = cs42l67_controls,
	.num_controls = ARRAY_SIZE(cs42l67_controls),
	.dapm_widgets = cs42l67_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs42l67_widgets),
	.dapm_routes = cs42l67_routes,
	.num_dapm_routes = ARRAY_SIZE(cs42l67_routes),
	/*
	 * Stay powered while the card is up: the chip is reset on power-up
	 * and there is no register cache, so mixer settings would not survive
	 * an idle power-down. It also keeps the jack status readable.
	 */
	.idle_bias_on = 1,
	.suspend_bias_off = 1,
	.endianness = 1,
};

/* ---- probe ---- */

/* AppleCS42L67Audio 0x…86ebe38: bit7 of 0x007 is cleared around the read */
static int cs42l67_read_id(struct cs42l67 *cs, u8 id[3])
{
	unsigned int ctl, v;
	int ret, i;

	ret = regmap_read(cs->regmap, CS42L67_ID_CTL, &ctl);
	if (ret)
		return ret;
	regmap_write(cs->regmap, CS42L67_ID_CTL, ctl & 0x7f);
	for (i = 0; i < 3; i++) {
		ret = regmap_read(cs->regmap, CS42L67_DEVID_AB + i, &v);
		if (ret)
			break;
		id[i] = v;
	}
	regmap_write(cs->regmap, CS42L67_ID_CTL, ctl);
	return ret;
}

static int cs42l67_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct cs42l67 *cs;
	u8 id[3] = {};
	int ret;

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	cs->rcv_gain = -24;	/* earpiece -12 dB until someone sets it: it sits at the ear */
	cs->dev = dev;
	cs->spi = spi;
	spi_set_drvdata(spi, cs);

	cs->regmap = devm_regmap_init(dev, &cs42l67_bus, cs, &cs42l67_regmap);
	if (IS_ERR(cs->regmap))
		return dev_err_probe(dev, PTR_ERR(cs->regmap), "regmap init failed\n");

	cs->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(cs->mclk))
		return dev_err_probe(dev, PTR_ERR(cs->mclk), "no MCLK\n");

	cs->tx_slot = 4;
	device_property_read_u32(dev, "cirrus,xsp-tx-slot", &cs->tx_slot);
	cs->rx_slot = 0;
	device_property_read_u32(dev, "cirrus,xsp-rx-slot", &cs->rx_slot);
	if (cs->tx_slot > 15 || cs->rx_slot > 15)
		return dev_err_probe(dev, -EINVAL, "bad XSP slot\n");

	ret = cs42l67_read_id(cs, id);
	if (ret || id[0] != 0x42 || id[1] != 0xa6 || (id[2] & 0xf0) != 0x70) {
		dev_err(dev, "no CS42L67: ret %d, ID %02x %02x %02x\n",
			ret, id[0], id[1], id[2]);
		if (!allow_missing)
			return -ENODEV;
		/*
		 * Keep the widgets (the card's routes name them) but never
		 * power the chip; hw_params fails, the speaker link still works.
		 */
		cs->inert = true;
		dev_warn(dev, "registering an inert DAI\n");
	} else {
		dev_info(dev, "CS42L67 ID %02x%02x%02x, XSP TX from slot %u, RX slot %u\n",
			 id[0], id[1], id[2], cs->tx_slot, cs->rx_slot);
	}

	return devm_snd_soc_register_component(dev, &cs42l67_component,
					       cs42l67_dais, ARRAY_SIZE(cs42l67_dais));
}

static const struct of_device_id cs42l67_of_match[] = {
	{ .compatible = "cirrus,cs42l67" },
	{}
};
MODULE_DEVICE_TABLE(of, cs42l67_of_match);

static const struct spi_device_id cs42l67_spi_id[] = {
	{ "cs42l67" },
	{}
};
MODULE_DEVICE_TABLE(spi, cs42l67_spi_id);

static struct spi_driver cs42l67_driver = {
	.driver = {
		.name = "cs42l67",
		.of_match_table = cs42l67_of_match,
	},
	.probe = cs42l67_probe,
	.id_table = cs42l67_spi_id,
};
module_spi_driver(cs42l67_driver);

MODULE_DESCRIPTION("ASoC Cirrus Logic CS42L67 codec driver (reverse engineered)");
MODULE_LICENSE("GPL");
