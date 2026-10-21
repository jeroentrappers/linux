// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (t7000) MCA "version 1" I2S/TDM controller, PIO playback and capture.
 *
 * On these SoCs iOS hands audio to the SmartIO (SIO) coprocessor, whose
 * firmware memcpys PCM into a small SRAM FIFO (the "shim") that hardware
 * then drains into the MCA. There is no DMA engine in the path, so this
 * driver does what the SIO firmware does from the AP: an hrtimer tops up
 * the TX shim FIFO from the ALSA ring buffer, and drains the RX shim FIFO
 * into the capture ring buffer. The SIO CPU is never started.
 *
 * Only what has been seen working on the iPhone 6 speaker path is
 * supported: MCA as clock master, 48 kHz, 16-bit, one playback channel in
 * slot 0 of an 8 x 16-bit frame. Capture takes 1..8 consecutive slots
 * starting at rx_slot_base.
 *
 * DAI 0 is the speaker link, DAI 1 the headset codec link. Both share the
 * one TX slot, so only one playback stream can run at a time.
 *
 * The node can also provide the MCLK pads of all MCAs (wrapper +0x2000 +
 * 4n, nclk / 2 = 12 MHz) as clocks, for codecs that need MCLK from a pad
 * whose MCA is otherwise unused.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* MCA core registers */
#define MCA_FIFO_PAD		0x08
#define MCA_FRAME_CFG		0x0c
#define  FRAME_CFG_MASTER	BIT(0)
#define  FRAME_CFG_RUN		BIT(1)
/*
 * 16-bit FSYNC, 16-bit slots, 8 slots per frame, BCLK = NCO / 2, bit 2 as
 * iOS sets it when the two ends disagree on who is master (always for us)
 */
#define  FRAME_CFG_8X16		(0x0f107000 | BIT(2))
#define MCA_IRQ_EN		0x10
#define MCA_PAD_BITS		0x14
#define MCA_TX_CTL		0x18
#define  TX_CTL_EN		BIT(0)
#define MCA_TX_UNK1C		0x1c
#define MCA_TX_FIFO		0x20
#define MCA_TX_MASK		0x24
#define MCA_RX_CTL		0x2c
#define MCA_RX_UNK30		0x30
#define MCA_RX_FIFO		0x34
#define MCA_RX_MASK		0x38
#define MCA_IRQ_STATUS		0x40
#define  IRQ_STATUS_ALL		0xb0003000
#define MCA_CMD			0x44
#define  CMD_TX_FLUSH		BIT(8)
#define  CMD_RX_FLUSH		BIT(9)
#define MCA_RESET		0x48
#define  RESET_GO		BIT(0)
#define  RX_CTL_EN		BIT(0)
#define  RX_CTL_IDLE		0x10a	/* what iOS and the speaker path use */

/* TX shim, 0x1000 above the MCA core; its FIFO write port at 0x2000 */
#define SHIM_TX			0x1000
#define SHIM_XFER		0x00
#define  XFER_16BIT		0x21
#define SHIM_TIMEOUT		0x04
#define SHIM_WINDOW		0x08
#define SHIM_TRIGGER		0x0c
#define SHIM_ENABLE		0x10
#define SHIM_LINK		0x14
#define SHIM_COUNT		0x18
#define  COUNT_FREE		GENMASK(8, 0)
#define SHIM_TX_PORT		0x2000
#define SHIM_PORT_WORDS		8
/* RX shim, 0x1800 above the MCA core; its FIFO read port at 0x3000 */
#define SHIM_RX			0x1800
#define  COUNT_FILLED		GENMASK(8, 0)
#define SHIM_RX_PORT		0x3000

/* audio wrapper page (0x20a000000) */
#define WRAP_FLUSH		0x0000
#define WRAP_DATA_MASK_TX	0x1000
#define WRAP_DATA_MASK_RX	0x1004
#define WRAP_SHIM_OFF_TX	0x1008
#define WRAP_SHIM_OFF_RX	0x100c
#define WRAP_SHIM_ON_TX		0x1010
#define WRAP_SHIM_ON_RX		0x1014
#define WRAP_UNAVAIL_MASK_TX	0x1040
#define WRAP_UNAVAIL_MASK_RX	0x1044
#define WRAP_TIMEOUT_MASK_TX	0x1080
#define WRAP_TIMEOUT_MASK_RX	0x1084
#define WRAP_MCLK_CTL(n)	(0x2000 + 4 * (n))
#define  MCLK_CTL_PIN_EN	BIT(6)
#define  MCLK_CTL_NCLK_DIV2	0x18
#define MCA_NUM_MCLK		5
#define MCA_MCLK_PAD_RATE	12000000
#define WRAP_SWITCH_PIN(n)	(0x3000 + 4 * (n))
#define WRAP_SWITCH_AUD(n)	(0x3000 + 4 * ((n) + 5))
#define WRAP_SWITCH_PINCTL(n)	(0x303c + 4 * (n))
/*
 * Switch words name the node feeding each field (AUDn = n, PINn = n + 8,
 * 0x18 = none). AUDn [12:8] is the data AUDn samples: without PINn there,
 * the MCA's RX serdes sees nothing (all-zero capture).
 */
#define  SWITCH_IDLE		0x00181818
#define  SWITCH_NODE_PIN(n)	((n) + 8)
#define  SWITCH_AUD_RX(pin)	(0x00180018 | SWITCH_NODE_PIN(pin) << 8)

/* second shim wrapper page (0x20a100000) */
#define SHIM1_CFG		0x004
#define SHIM1_UNK800		0x800

/* MCA shims follow SPI/UART in the SIO numbering: MCAn TX = shim 13 + n */
#define MCA_SHIM_INDEX(n)	(13 + (n))

#define MCA_RATE		48000
#define MCA_MCLK_RATE		(MCA_RATE * 256)	/* NCO; BCLK = NCO / 2 */

/*
 * The shim SRAM pool for all MCA TX shims is 0x400 bytes; this instance
 * takes the first 0xcc like the SIO firmware does for MCA2. COUNT is 9
 * bits wide, so up to 0x1fc should work too.
 */
static unsigned int fifo_bytes = 0xcc;
module_param(fifo_bytes, uint, 0444);
MODULE_PARM_DESC(fifo_bytes, "shim FIFO size in bytes (multiple of 4, <= 0x1fc)");

static unsigned int rx_fifo_bytes = 0xcc;
module_param(rx_fifo_bytes, uint, 0444);
MODULE_PARM_DESC(rx_fifo_bytes, "RX shim FIFO size in bytes (multiple of 4, <= 0x1fc)");

static unsigned int rx_slot_base = 4;
module_param(rx_slot_base, uint, 0644);
MODULE_PARM_DESC(rx_slot_base, "first TDM slot captured (capture channel 1)");

/*
 * RX control: iOS uses 0x10a as a slave; as a master the TX control needs
 * [16:8] one higher (0x208), so the same is assumed for RX.
 */
static unsigned int rx_ctl = 0x20a;
module_param(rx_ctl, uint, 0644);
MODULE_PARM_DESC(rx_ctl, "MCA RX control word while capturing, enable bit clear (iOS slave value 0x10a)");

static unsigned int refill_us = 500;
module_param(refill_us, uint, 0644);
MODULE_PARM_DESC(refill_us, "longest FIFO refill period in microseconds (capture may poll faster)");

/* below this the hrtimer would eat the CPU */
#define MCA_MIN_PERIOD_US	50

struct mca_dir {
	struct snd_pcm_substream *substream;
	bool running;
	unsigned int word;		/* next shim port slot */
	unsigned int pos;		/* byte offset into the ring buffer */
	unsigned int period_left;	/* bytes until the next period */
	unsigned int xruns;
};

struct mca_mclk {
	struct clk_hw hw;
	struct mca_t7000 *mca;
	unsigned int pad;
};

struct mca_t7000 {
	struct device *dev;
	void __iomem *base;
	void __iomem *wrap;
	void __iomem *shim1;
	struct clk *clk;
	unsigned int index;
	unsigned int pin;

	struct hrtimer timer;
	struct mca_dir tx, rx;
	unsigned int users;		/* prepared streams, BIT(stream) */
	unsigned int rx_period_us;	/* drain period that keeps the RX shim from overflowing */
	struct mca_mclk mclk[MCA_NUM_MCLK];
};

static inline u32 shim_free(struct mca_t7000 *mca)
{
	return FIELD_GET(COUNT_FREE, readl_relaxed(mca->base + SHIM_TX + SHIM_COUNT));
}

static inline u32 shim_filled(struct mca_t7000 *mca)
{
	return FIELD_GET(COUNT_FILLED, readl_relaxed(mca->base + SHIM_RX + SHIM_COUNT));
}

/*
 * The timer serves both directions. TX is mono (96 bytes/ms), so the default
 * period is plenty; an 8-slot capture fills the 204-byte RX shim in ~265 us,
 * and an overflow drops words, which rotates the channels for good.
 */
static ktime_t mca_t7000_period(struct mca_t7000 *mca)
{
	unsigned int us = READ_ONCE(refill_us);

	if (READ_ONCE(mca->rx.running))
		us = min(us, mca->rx_period_us);
	return us_to_ktime(max(us, MCA_MIN_PERIOD_US));
}

static unsigned int mca_t7000_period_step(struct mca_dir *d, unsigned int moved)
{
	struct snd_pcm_runtime *runtime = d->substream->runtime;
	unsigned int period_bytes = frames_to_bytes(runtime, runtime->period_size);
	unsigned int elapsed = 0;

	while (moved >= d->period_left) {
		moved -= d->period_left;
		d->period_left = period_bytes;
		elapsed++;
	}
	d->period_left -= moved;
	return elapsed;
}

/* Copy up to the shim's free space from the ring buffer. Returns bytes moved. */
static unsigned int mca_t7000_fill(struct mca_t7000 *mca)
{
	struct mca_dir *d = &mca->tx;
	struct snd_pcm_runtime *runtime = d->substream->runtime;
	unsigned int buf_bytes = frames_to_bytes(runtime, runtime->buffer_size);
	unsigned int free = shim_free(mca) & ~3;
	unsigned int done = 0;
	u8 *area = runtime->dma_area;

	while (done < free) {
		writel_relaxed(*(u32 *)(area + d->pos),
			       mca->base + SHIM_TX_PORT + 4 * d->word);
		d->word = (d->word + 1) % SHIM_PORT_WORDS;
		d->pos += 4;
		if (d->pos >= buf_bytes)
			d->pos = 0;
		done += 4;
	}

	return done;
}

/* Copy what the RX shim holds into the capture ring buffer. Returns bytes moved. */
static unsigned int mca_t7000_drain(struct mca_t7000 *mca)
{
	struct mca_dir *d = &mca->rx;
	struct snd_pcm_runtime *runtime = d->substream->runtime;
	unsigned int buf_bytes = frames_to_bytes(runtime, runtime->buffer_size);
	unsigned int filled = min(shim_filled(mca), rx_fifo_bytes) & ~3;
	unsigned int done = 0;
	u8 *area = runtime->dma_area;

	while (done < filled) {
		*(u32 *)(area + d->pos) =
			readl_relaxed(mca->base + SHIM_RX_PORT + 4 * d->word);
		d->word = (d->word + 1) % SHIM_PORT_WORDS;
		d->pos += 4;
		if (d->pos >= buf_bytes)
			d->pos = 0;
		done += 4;
	}

	return done;
}

static enum hrtimer_restart mca_t7000_timer(struct hrtimer *timer)
{
	struct mca_t7000 *mca = container_of(timer, struct mca_t7000, timer);
	bool tx = READ_ONCE(mca->tx.running), rx = READ_ONCE(mca->rx.running);
	unsigned int tx_elapsed = 0, rx_elapsed = 0;

	if (!tx && !rx)
		return HRTIMER_NORESTART;

	if (tx) {
		if (shim_free(mca) >= fifo_bytes)
			mca->tx.xruns++;
		tx_elapsed = mca_t7000_period_step(&mca->tx, mca_t7000_fill(mca));
	}
	if (rx) {
		if (shim_filled(mca) >= rx_fifo_bytes)
			mca->rx.xruns++;
		rx_elapsed = mca_t7000_period_step(&mca->rx, mca_t7000_drain(mca));
	}

	if (tx_elapsed)
		snd_pcm_period_elapsed(mca->tx.substream);
	if (rx_elapsed)
		snd_pcm_period_elapsed(mca->rx.substream);

	if (!READ_ONCE(mca->tx.running) && !READ_ONCE(mca->rx.running))
		return HRTIMER_NORESTART;

	hrtimer_forward_now(timer, mca_t7000_period(mca));
	return HRTIMER_RESTART;
}

static void mca_t7000_hw_init(struct mca_t7000 *mca)
{
	unsigned int shim = BIT(MCA_SHIM_INDEX(mca->index));
	void __iomem *tx = mca->base + SHIM_TX;
	void __iomem *rx = mca->base + SHIM_RX;
	u32 val;

	/* MCLK pad = nclk / 2 = 12 MHz for the amp, pad driven */
	writel(MCLK_CTL_PIN_EN | MCLK_CTL_NCLK_DIV2,
	       mca->wrap + WRAP_MCLK_CTL(mca->index));

	/* what the SIO firmware does at boot */
	writel(~0, mca->wrap + WRAP_DATA_MASK_TX);
	writel(~0, mca->wrap + WRAP_DATA_MASK_RX);
	writel(~0, mca->wrap + WRAP_TIMEOUT_MASK_TX);
	writel(~0, mca->wrap + WRAP_TIMEOUT_MASK_RX);
	writel(0, mca->wrap + WRAP_UNAVAIL_MASK_TX);
	writel(0, mca->wrap + WRAP_UNAVAIL_MASK_RX);
	writel(0x03000102, mca->shim1 + SHIM1_CFG);
	writel(0x118, mca->shim1 + SHIM1_UNK800);

	/* MCA soft reset, then the early registers, as iOS orders it */
	writel(3, mca->base + MCA_CMD);
	writel(RESET_GO, mca->base + MCA_RESET);
	if (readl_poll_timeout(mca->base + MCA_RESET, val, !(val & RESET_GO),
			       10, 10000))
		dev_warn(mca->dev, "MCA reset timed out\n");
	writel(0, mca->base + MCA_IRQ_EN);
	writel(0xd, mca->base + MCA_TX_UNK1C);
	writel(0xd, mca->base + MCA_RX_UNK30);

	/* program the TX shim (SIO fw op 5) */
	writel(shim, mca->wrap + WRAP_SHIM_OFF_TX);
	writel(0, tx + SHIM_LINK);
	writel(0, tx + SHIM_ENABLE);
	readl(mca->wrap + WRAP_FLUSH);
	readl(mca->wrap + WRAP_FLUSH);
	writel(XFER_16BIT, tx + SHIM_XFER);
	writel((fifo_bytes / 4) << 16, tx + SHIM_WINDOW);
	writel(fifo_bytes - 0x20, tx + SHIM_TRIGGER);
	writel(0, tx + SHIM_TIMEOUT);
	writel(1, tx + SHIM_ENABLE);

	/* the RX shim, from the separate RX SRAM pool (offset 0) */
	writel(shim, mca->wrap + WRAP_SHIM_OFF_RX);
	writel(0, rx + SHIM_LINK);
	writel(0, rx + SHIM_ENABLE);
	readl(mca->wrap + WRAP_FLUSH);
	readl(mca->wrap + WRAP_FLUSH);
	writel(XFER_16BIT, rx + SHIM_XFER);
	writel((rx_fifo_bytes / 4) << 16, rx + SHIM_WINDOW);
	writel(1, rx + SHIM_TRIGGER);
	writel(0, rx + SHIM_TIMEOUT);
	writel(1, rx + SHIM_ENABLE);

	/* 16-bit slots, 8 per frame, TX slot 0, clock master */
	writel(0x1010, mca->base + MCA_FIFO_PAD);
	writel(0x00170018, mca->base + MCA_TX_FIFO);
	writel(0x00030004, mca->base + MCA_RX_FIFO);
	writel(0, mca->base + MCA_PAD_BITS);
	writel(1, mca->base + MCA_TX_MASK);
	writel(0xf, mca->base + MCA_RX_MASK);
	writel(0x208, mca->base + MCA_TX_CTL);
	writel(RX_CTL_IDLE, mca->base + MCA_RX_CTL);
	writel(1, mca->base + MCA_IRQ_EN);
	writel(IRQ_STATUS_ALL, mca->base + MCA_IRQ_STATUS);
	writel(FRAME_CFG_8X16 | FRAME_CFG_MASTER, mca->base + MCA_FRAME_CFG);

	/* route MCAn's data out to its I2S pins; the pins clock from the MCA */
	writel(SWITCH_IDLE, mca->wrap + WRAP_SWITCH_AUD(mca->index));
	writel(0x00180000 | (mca->index << 8) | mca->index,
	       mca->wrap + WRAP_SWITCH_PIN(mca->pin));
	writel(0x11, mca->wrap + WRAP_SWITCH_PINCTL(mca->pin));
}

static void mca_t7000_hw_stop_tx(struct mca_t7000 *mca)
{
	unsigned int shim = BIT(MCA_SHIM_INDEX(mca->index));
	u32 val;

	/* the frame clock keeps running until hw_free, for the amp's sake */
	writel_relaxed(0x208, mca->base + MCA_TX_CTL);
	writel_relaxed(0, mca->base + SHIM_TX + SHIM_LINK);
	writel_relaxed(shim, mca->wrap + WRAP_SHIM_OFF_TX);
	writel(CMD_TX_FLUSH, mca->base + MCA_CMD);
	readl_poll_timeout_atomic(mca->base + MCA_CMD, val,
				  !(val & CMD_TX_FLUSH), 1, 1000);
}

static void mca_t7000_hw_stop_rx(struct mca_t7000 *mca)
{
	unsigned int shim = BIT(MCA_SHIM_INDEX(mca->index));
	u32 val;

	writel_relaxed(RX_CTL_IDLE, mca->base + MCA_RX_CTL);
	writel_relaxed(0, mca->base + SHIM_RX + SHIM_LINK);
	writel_relaxed(shim, mca->wrap + WRAP_SHIM_OFF_RX);
	writel(CMD_RX_FLUSH, mca->base + MCA_CMD);
	readl_poll_timeout_atomic(mca->base + MCA_CMD, val,
				  !(val & CMD_RX_FLUSH), 1, 1000);
}

static int mca_t7000_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* the only tested frame: DSP A, MCA provides BCLK and FSYNC */
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_DSP_A ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BP_FP)
		return -EINVAL;

	return 0;
}

static int mca_t7000_dai_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params,
				   struct snd_soc_dai *dai)
{
	struct mca_t7000 *mca = snd_soc_dai_get_drvdata(dai);
	int ret;

	if (params_rate(params) != MCA_RATE ||
	    params_format(params) != SNDRV_PCM_FORMAT_S16_LE)
		return -EINVAL;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    params_channels(params) != 1)
		return -EINVAL;
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
	    rx_slot_base + params_channels(params) > 8)
		return -EINVAL;

	ret = clk_set_rate(mca->clk, MCA_MCLK_RATE);
	if (ret)
		return ret;

	return 0;
}

static int mca_t7000_dai_prepare(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct mca_t7000 *mca = snd_soc_dai_get_drvdata(dai);
	unsigned int ch;

	/* the reset in hw_init would kill a stream already running */
	if (!(mca->users & ~BIT(substream->stream))) {
		mca_t7000_hw_init(mca);

		/*
		 * Start BCLK/FSYNC now, before DAPM powers the amp up: the
		 * CS35L19 latches ADSPCLK_ERR if it powers up without a serial
		 * clock. TX stays disabled until trigger; the shim link starts
		 * fine late.
		 */
		writel(FRAME_CFG_8X16 | FRAME_CFG_RUN | FRAME_CFG_MASTER,
		       mca->base + MCA_FRAME_CFG);
	}
	mca->users |= BIT(substream->stream);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		ch = substream->runtime->channels;
		writel(GENMASK(rx_slot_base + ch - 1, rx_slot_base),
		       mca->base + MCA_RX_MASK);
		/* the pin's data into this MCA, or RX samples nothing */
		writel(SWITCH_AUD_RX(mca->pin),
		       mca->wrap + WRAP_SWITCH_AUD(mca->index));
		/* drain at half full: ch * 2 bytes arrive per frame */
		mca->rx_period_us = rx_fifo_bytes / 2 * USEC_PER_SEC /
				    (MCA_RATE * frames_to_bytes(substream->runtime, 1));
	}
	return 0;
}

static int mca_t7000_dai_hw_free(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct mca_t7000 *mca = snd_soc_dai_get_drvdata(dai);

	mca->users &= ~BIT(substream->stream);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		writel(SWITCH_IDLE, mca->wrap + WRAP_SWITCH_AUD(mca->index));
	if (!mca->users)
		writel(FRAME_CFG_8X16 | FRAME_CFG_MASTER,
		       mca->base + MCA_FRAME_CFG);
	return 0;
}

static const struct snd_soc_dai_ops mca_t7000_dai_ops = {
	.set_fmt = mca_t7000_dai_set_fmt,
	.hw_params = mca_t7000_dai_hw_params,
	.prepare = mca_t7000_dai_prepare,
	.hw_free = mca_t7000_dai_hw_free,
};

static struct snd_soc_dai_driver mca_t7000_dais[] = {
	{
		.name = "mca-t7000",
		.id = 0,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &mca_t7000_dai_ops,
	},
	{
		.name = "mca-t7000-codec",
		.id = 1,
		.playback = {
			.stream_name = "Codec Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "Codec Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &mca_t7000_dai_ops,
	},
};

static const struct snd_pcm_hardware mca_t7000_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BATCH,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = MCA_RATE,
	.rate_max = MCA_RATE,
	.channels_min = 1,
	.channels_max = 8,
	.buffer_bytes_max = SZ_128K,
	.period_bytes_min = 1024,	/* > the largest shim FIFO, see trigger */
	.period_bytes_max = SZ_32K,
	.periods_min = 2,
	.periods_max = 64,
};

/*
 * Wait for a running timer callback to finish, so that it no longer looks at
 * a stopped stream, and keep it going for the other direction.
 */
static void mca_t7000_timer_sync(struct mca_t7000 *mca)
{
	hrtimer_cancel(&mca->timer);
	if (READ_ONCE(mca->tx.running) || READ_ONCE(mca->rx.running))
		hrtimer_start(&mca->timer, mca_t7000_period(mca),
			      HRTIMER_MODE_REL_HARD);
}

static int mca_t7000_pcm_open(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	struct mca_t7000 *mca = snd_soc_component_get_drvdata(component);
	struct mca_dir *d = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			    &mca->tx : &mca->rx;
	int ret;

	/* one TX slot, one RX shim: one stream per direction */
	if (d->substream)
		return -EBUSY;

	ret = snd_soc_set_runtime_hwparams(substream, &mca_t7000_pcm_hw);
	if (ret)
		return ret;

	/* the shim takes whole 32-bit words, i.e. pairs of mono frames */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 4);
	if (ret)
		return ret;
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 4);
	if (ret)
		return ret;

	d->substream = substream;
	return 0;
}

static int mca_t7000_pcm_close(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct mca_t7000 *mca = snd_soc_component_get_drvdata(component);

	struct mca_dir *d = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			    &mca->tx : &mca->rx;

	mca_t7000_timer_sync(mca);
	d->substream = NULL;
	return 0;
}

static int mca_t7000_pcm_prepare(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream)
{
	struct mca_t7000 *mca = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mca_dir *d = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			    &mca->tx : &mca->rx;

	d->pos = 0;
	d->period_left = frames_to_bytes(runtime, runtime->period_size);
	d->xruns = 0;
	return 0;
}

static int mca_t7000_pcm_trigger(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream, int cmd)
{
	struct mca_t7000 *mca = snd_soc_component_get_drvdata(component);
	unsigned int shim = BIT(MCA_SHIM_INDEX(mca->index));
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	struct mca_dir *d = playback ? &mca->tx : &mca->rx;
	struct mca_dir *other = playback ? &mca->rx : &mca->tx;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		d->word = 0;
		if (playback) {
			/* prefill, the SIO fw start writes, then enable TX */
			mca->tx.period_left -= mca_t7000_fill(mca);
			writel_relaxed(fifo_bytes - 0x20,
				       mca->base + SHIM_TX + SHIM_TRIGGER);
			writel_relaxed(shim, mca->wrap + WRAP_SHIM_ON_TX);
			writel_relaxed(shim, mca->wrap + WRAP_SHIM_ON_RX);
			writel_relaxed(1, mca->base + SHIM_TX + SHIM_LINK);
			writel(0x208 | TX_CTL_EN, mca->base + MCA_TX_CTL);
		} else {
			/* the SIO fw RX start: TRIGGER = 1, shim on, then RX */
			writel_relaxed(1, mca->base + SHIM_RX + SHIM_TRIGGER);
			writel_relaxed(shim, mca->wrap + WRAP_SHIM_ON_RX);
			writel_relaxed(1, mca->base + SHIM_RX + SHIM_LINK);
			writel(rx_ctl | RX_CTL_EN, mca->base + MCA_RX_CTL);
		}
		WRITE_ONCE(d->running, true);
		/*
		 * (Re)arm even when the other direction runs: a capture
		 * starting next to playback needs the shorter period from
		 * the first tick on.
		 */
		hrtimer_start(&mca->timer, mca_t7000_period(mca),
			      HRTIMER_MODE_REL_HARD);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* may run inside our own timer callback: never wait for it */
		WRITE_ONCE(d->running, false);
		if (!READ_ONCE(other->running))
			hrtimer_try_to_cancel(&mca->timer);
		if (playback)
			mca_t7000_hw_stop_tx(mca);
		else
			mca_t7000_hw_stop_rx(mca);
		if (d->xruns)
			dev_dbg(mca->dev, "%u shim %s\n", d->xruns,
				playback ? "underruns" : "overruns");
		return 0;
	default:
		return -EINVAL;
	}
}

static int mca_t7000_pcm_sync_stop(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	struct mca_t7000 *mca = snd_soc_component_get_drvdata(component);

	mca_t7000_timer_sync(mca);
	return 0;
}

static snd_pcm_uframes_t mca_t7000_pcm_pointer(struct snd_soc_component *component,
					       struct snd_pcm_substream *substream)
{
	struct mca_t7000 *mca = snd_soc_component_get_drvdata(component);
	struct mca_dir *d = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
			    &mca->tx : &mca->rx;

	return bytes_to_frames(substream->runtime, READ_ONCE(d->pos));
}

static int mca_t7000_pcm_new(struct snd_soc_component *component,
				   struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
				       0, 0);
	return 0;
}

static const struct snd_soc_component_driver mca_t7000_component = {
	.name = "apple-mca-t7000",
	.open = mca_t7000_pcm_open,
	.close = mca_t7000_pcm_close,
	.prepare = mca_t7000_pcm_prepare,
	.trigger = mca_t7000_pcm_trigger,
	.sync_stop = mca_t7000_pcm_sync_stop,
	.pointer = mca_t7000_pcm_pointer,
	.pcm_new = mca_t7000_pcm_new,
	.legacy_dai_naming = 1,
};

/* MCLK pads: wrapper +0x2000 + 4n, nclk / 2 = 12 MHz, pad output enabled */
static struct mca_mclk *to_mca_mclk(struct clk_hw *hw)
{
	return container_of(hw, struct mca_mclk, hw);
}

static int mca_t7000_mclk_enable(struct clk_hw *hw)
{
	struct mca_mclk *m = to_mca_mclk(hw);

	writel(MCLK_CTL_PIN_EN | MCLK_CTL_NCLK_DIV2,
	       m->mca->wrap + WRAP_MCLK_CTL(m->pad));
	return 0;
}

static void mca_t7000_mclk_disable(struct clk_hw *hw)
{
	struct mca_mclk *m = to_mca_mclk(hw);

	/* our own pad also feeds the speaker amp; hw_init owns it */
	if (m->pad != m->mca->index)
		writel(0, m->mca->wrap + WRAP_MCLK_CTL(m->pad));
}

static int mca_t7000_mclk_is_enabled(struct clk_hw *hw)
{
	struct mca_mclk *m = to_mca_mclk(hw);

	return !!(readl(m->mca->wrap + WRAP_MCLK_CTL(m->pad)) & MCLK_CTL_PIN_EN);
}

static unsigned long mca_t7000_mclk_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	return MCA_MCLK_PAD_RATE;
}

static const struct clk_ops mca_t7000_mclk_ops = {
	.enable = mca_t7000_mclk_enable,
	.disable = mca_t7000_mclk_disable,
	.is_enabled = mca_t7000_mclk_is_enabled,
	.recalc_rate = mca_t7000_mclk_recalc_rate,
};

static struct clk_hw *mca_t7000_mclk_get(struct of_phandle_args *spec, void *data)
{
	struct mca_t7000 *mca = data;

	if (spec->args_count != 1 || spec->args[0] >= MCA_NUM_MCLK)
		return ERR_PTR(-EINVAL);
	return &mca->mclk[spec->args[0]].hw;
}

static int mca_t7000_register_mclks(struct mca_t7000 *mca)
{
	struct device *dev = mca->dev;
	struct clk_init_data init = {
		.ops = &mca_t7000_mclk_ops,
		/* iBoot may leave a pad running; never gate it behind our back */
		.flags = CLK_IGNORE_UNUSED,
	};
	unsigned int i;
	int ret;

	if (!of_property_present(dev->of_node, "#clock-cells"))
		return 0;

	for (i = 0; i < MCA_NUM_MCLK; i++) {
		init.name = devm_kasprintf(dev, GFP_KERNEL, "mca%u-mclk", i);
		if (!init.name)
			return -ENOMEM;
		mca->mclk[i].mca = mca;
		mca->mclk[i].pad = i;
		mca->mclk[i].hw.init = &init;
		ret = devm_clk_hw_register(dev, &mca->mclk[i].hw);
		if (ret)
			return ret;
	}

	return devm_of_clk_add_hw_provider(dev, mca_t7000_mclk_get, mca);
}

static int mca_t7000_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mca_t7000 *mca;
	struct resource *res;
	int ret;

	if (fifo_bytes < 0x40 || fifo_bytes > 0x1fc || fifo_bytes % 4)
		return dev_err_probe(dev, -EINVAL, "bad fifo_bytes %#x\n", fifo_bytes);
	if (rx_fifo_bytes < 0x20 || rx_fifo_bytes > 0x1fc || rx_fifo_bytes % 4)
		return dev_err_probe(dev, -EINVAL, "bad rx_fifo_bytes %#x\n", rx_fifo_bytes);

	mca = devm_kzalloc(dev, sizeof(*mca), GFP_KERNEL);
	if (!mca)
		return -ENOMEM;
	mca->dev = dev;

	mca->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(mca->base))
		return PTR_ERR(mca->base);
	/* MCAn sits at 0x20a0a0000 + n * 0x4000 */
	mca->index = (res->start >> 14) & 0x7;

	/* the wrapper pages are shared by all MCAs: map without claiming */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -EINVAL;
	mca->wrap = devm_ioremap(dev, res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res)
		return -EINVAL;
	mca->shim1 = devm_ioremap(dev, res->start, resource_size(res));
	if (!mca->wrap || !mca->shim1)
		return -ENOMEM;

	mca->pin = mca->index;
	of_property_read_u32(dev->of_node, "apple,i2s-pin", &mca->pin);

	mca->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(mca->clk))
		return dev_err_probe(dev, PTR_ERR(mca->clk), "no clock\n");

	hrtimer_setup(&mca->timer, mca_t7000_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_HARD);
	platform_set_drvdata(pdev, mca);

	ret = mca_t7000_register_mclks(mca);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register the MCLK pads\n");

	ret = devm_snd_soc_register_component(dev, &mca_t7000_component,
					      mca_t7000_dais,
					      ARRAY_SIZE(mca_t7000_dais));
	if (ret)
		return dev_err_probe(dev, ret, "failed to register component\n");

	dev_info(dev, "MCA%u, shim FIFO %u/%u bytes, PIO every %u us, capture from slot %u\n",
		 mca->index, fifo_bytes, rx_fifo_bytes, refill_us, rx_slot_base);
	return 0;
}

static const struct of_device_id mca_t7000_of_match[] = {
	{ .compatible = "apple,t7000-mca", },
	{}
};
MODULE_DEVICE_TABLE(of, mca_t7000_of_match);

static struct platform_driver mca_t7000_driver = {
	.driver = {
		.name = "apple-mca-t7000",
		.of_match_table = mca_t7000_of_match,
	},
	.probe = mca_t7000_probe,
};
module_platform_driver(mca_t7000_driver);

MODULE_DESCRIPTION("Apple A8 MCA (v1) ASoC driver, PIO playback and capture");
MODULE_LICENSE("Dual MIT/GPL");
