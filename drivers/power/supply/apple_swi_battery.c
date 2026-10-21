// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SWI battery gauge driver
 *
 * The iPhone 6 has a TI bq27540 gas gauge in the battery pack. It talks
 * HDQ over a single wire ("SWI") that is driven by one of the SoC UARTs,
 * the same way iOS's AppleHDQGasGauge does it:
 *
 *  - the UART runs at 57600 baud, 8 data bits, 2 stop bits, so one frame
 *    lasts about as long as one HDQ bit (190 us);
 *  - the host sends a '1' as the frame 0xfe and a '0' as 0xc0, LSB first;
 *  - TX and RX share the wire, so every frame sent comes back as an echo;
 *  - a frame received from the gauge that is above 0xf8 is a '1';
 *  - a break is one 0x00 frame sent at 9600 baud (about 940 us low).
 *
 * The wire also goes through the SN2400 charger, which must hand the bus
 * to the SoC before the gauge can be reached: write 0x04 to its register
 * 0x1d and wait for bit 5, write 0 to give the bus back. iOS asks for it
 * around every access, and so does this driver (it gives the bus back
 * when the UART goes idle).
 *
 * The gauge uses the standard bq27541 command set, so the reading and the
 * power_supply side are left to the bq27xxx driver. Nothing is ever written
 * to the gauge.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/power/bq27xxx_battery.h>

/* S5L UART registers */
#define UART_ULCON		0x00
#define UART_UCON		0x04
#define UART_UFCON		0x08
#define UART_UTRSTAT		0x10
#define UART_UERSTAT		0x14
#define UART_UFSTAT		0x18
#define UART_UTXH		0x20
#define UART_URXH		0x24
#define UART_UBRDIV		0x28

#define ULCON_8N2		0x07
#define UCON_POLLED		0x05	/* RX and TX in polling / IRQ mode */
#define UFCON_FIFO		BIT(0)
#define UFCON_RESET		(BIT(1) | BIT(2))
#define UTRSTAT_TXE		BIT(2)	/* FIFO and shifter empty */
#define UFSTAT_RXCOUNT		GENMASK(3, 0)
#define UFSTAT_RXFULL		BIT(8)

#define SWI_CLK			24000000
#define SWI_BAUD		57600
#define SWI_BREAK_BAUD		9600
#define SWI_BIT_ONE		0xfe
#define SWI_BIT_ZERO		0xc0
#define SWI_RX_ONE_MIN		0xf9

#define HDQ_WRITE		BIT(7)	/* never set: the gauge is only read */
#define HDQ_TRIES		3

/* SN2400 bus hand-over */
#define SN2400_HDQ_CTRL		0x1d
#define SN2400_HDQ_REQUEST	0x04
#define SN2400_HDQ_GRANTED	BIT(5)

#define SWI_IDLE_MS		500

/*
 * Frame times: 11 bits (start, 8 data, 2 stop) at 57600 and at 9600 baud.
 * The waits below sleep through most of a transfer in one go and only
 * then poll: polling every 100 us from the start cost ~55 timer wakeups
 * per byte, ~170 per 16-bit register.
 */
#define SWI_FRAME_US		191
#define SWI_BREAK_US		1146
#define SWI_RESPONSE_US		320	/* gauge answers within this */

struct apple_swi {
	struct device *dev;
	void __iomem *base;
	struct i2c_client *master;
	struct mutex lock;	/* one HDQ transaction at a time */
	bool granted;
	struct bq27xxx_device_info di;
};

static inline u32 swi_rd(struct apple_swi *swi, u32 reg)
{
	return readl(swi->base + reg);
}

static inline void swi_wr(struct apple_swi *swi, u32 reg, u32 val)
{
	writel(val, swi->base + reg);
}

static void swi_set_baud(struct apple_swi *swi, unsigned int baud)
{
	swi_wr(swi, UART_UBRDIV, SWI_CLK / (16 * baud) - 1);
}

static unsigned int swi_rx_count(struct apple_swi *swi)
{
	u32 st = swi_rd(swi, UART_UFSTAT);

	return st & UFSTAT_RXFULL ? 16 : FIELD_GET(UFSTAT_RXCOUNT, st);
}

static void swi_drain(struct apple_swi *swi)
{
	unsigned int i;

	for (i = 0; i < 32 && swi_rx_count(swi); i++)
		swi_rd(swi, UART_URXH);
	swi_rd(swi, UART_UERSTAT);
}

static int swi_wait_tx(struct apple_swi *swi)
{
	u32 st;

	return readl_poll_timeout(swi->base + UART_UTRSTAT, st,
				  st & UTRSTAT_TXE, 50, 10000);
}

/* Wait for n frames that start arriving within lead_us */
static int swi_wait_rx(struct apple_swi *swi, unsigned int n,
		       unsigned int lead_us)
{
	unsigned int cnt, us = lead_us + n * SWI_FRAME_US;

	/* 8 frames take 1.5 ms: sleep through them, then poll the rest */
	usleep_range(us, us + 100);

	return read_poll_timeout(swi_rx_count, cnt, cnt >= n, 100, 10000,
				 false, swi);
}

static void swi_uart_setup(struct apple_swi *swi)
{
	swi_wr(swi, UART_UCON, 0);
	swi_wr(swi, UART_UFCON, UFCON_FIFO | UFCON_RESET);
	udelay(10);
	swi_wr(swi, UART_UFCON, UFCON_FIFO);
	swi_wr(swi, UART_ULCON, ULCON_8N2);
	swi_set_baud(swi, SWI_BAUD);
	swi_wr(swi, UART_UCON, UCON_POLLED);
	swi_drain(swi);
}

static int swi_break(struct apple_swi *swi)
{
	int ret;

	ret = swi_wait_tx(swi);
	if (ret)
		return ret;

	swi_set_baud(swi, SWI_BREAK_BAUD);
	swi_wr(swi, UART_UTXH, 0);
	usleep_range(SWI_BREAK_US, SWI_BREAK_US + 100);
	ret = swi_wait_tx(swi);
	swi_set_baud(swi, SWI_BAUD);
	/* break recovery, at least 40 us */
	usleep_range(100, 200);
	swi_drain(swi);

	return ret;
}

static int swi_send(struct apple_swi *swi, u8 byte)
{
	u8 frame[8];
	int i, ret;

	for (i = 0; i < 8; i++) {
		frame[i] = byte & BIT(i) ? SWI_BIT_ONE : SWI_BIT_ZERO;
		swi_wr(swi, UART_UTXH, frame[i]);
	}

	ret = swi_wait_rx(swi, 8, 0);	/* the echo */
	if (ret)
		return ret;

	for (i = 0; i < 8; i++)
		if ((swi_rd(swi, UART_URXH) & 0xff) != frame[i])
			ret = -EIO;

	return ret;
}

static int swi_recv(struct apple_swi *swi)
{
	int i, ret, val = 0;

	ret = swi_wait_rx(swi, 8, SWI_RESPONSE_US);
	if (ret)
		return ret;

	for (i = 0; i < 8; i++)
		if ((swi_rd(swi, UART_URXH) & 0xff) >= SWI_RX_ONE_MIN)
			val |= BIT(i);

	if (swi_rx_count(swi))
		return -EIO;

	return val;
}

static int swi_grant(struct apple_swi *swi)
{
	int ret, val;

	if (swi->granted)
		return 0;

	ret = i2c_smbus_write_byte_data(swi->master, SN2400_HDQ_CTRL,
					SN2400_HDQ_REQUEST);
	if (ret)
		return ret;

	ret = read_poll_timeout(i2c_smbus_read_byte_data, val,
				val < 0 || (val & SN2400_HDQ_GRANTED),
				10000, 1000000, false,
				swi->master, SN2400_HDQ_CTRL);
	if (!ret && val < 0)
		ret = val;
	if (ret) {
		i2c_smbus_write_byte_data(swi->master, SN2400_HDQ_CTRL, 0);
		dev_err(swi->dev, "charger did not hand over the bus: %d\n", ret);
		return ret;
	}

	swi->granted = true;
	return 0;
}

static void swi_release(struct apple_swi *swi)
{
	if (!swi->granted)
		return;

	i2c_smbus_write_byte_data(swi->master, SN2400_HDQ_CTRL, 0);
	swi->granted = false;
}

static int swi_hdq_read8(struct apple_swi *swi, u8 reg)
{
	int ret, try;

	for (try = 0; try < HDQ_TRIES; try++) {
		ret = swi_break(swi);
		if (!ret)
			ret = swi_send(swi, reg & ~HDQ_WRITE);
		if (!ret)
			ret = swi_recv(swi);
		if (ret >= 0)
			return ret;
	}

	return ret;
}

/* 16-bit registers: read the high byte around the low one, as iOS does */
static int swi_hdq_read16(struct apple_swi *swi, u8 reg)
{
	int hi, lo, hi2, try;

	for (try = 0; try < HDQ_TRIES; try++) {
		hi = swi_hdq_read8(swi, reg + 1);
		if (hi < 0)
			return hi;
		lo = swi_hdq_read8(swi, reg);
		if (lo < 0)
			return lo;
		hi2 = swi_hdq_read8(swi, reg + 1);
		if (hi2 < 0)
			return hi2;
		if (hi == hi2)
			return hi << 8 | lo;
	}

	return -EIO;
}

static int swi_begin(struct apple_swi *swi)
{
	int ret;

	ret = pm_runtime_resume_and_get(swi->dev);
	if (ret)
		return ret;

	mutex_lock(&swi->lock);
	ret = swi_grant(swi);
	if (ret) {
		mutex_unlock(&swi->lock);
		pm_runtime_put_autosuspend(swi->dev);
	}

	return ret;
}

static void swi_end(struct apple_swi *swi)
{
	mutex_unlock(&swi->lock);
	pm_runtime_put_autosuspend(swi->dev);
}

static int swi_bq27xxx_read(struct bq27xxx_device_info *di, u8 reg,
			    bool single)
{
	struct apple_swi *swi = container_of(di, struct apple_swi, di);
	int ret;

	ret = swi_begin(swi);
	if (ret)
		return ret;

	ret = single ? swi_hdq_read8(swi, reg) : swi_hdq_read16(swi, reg);

	swi_end(swi);

	return ret;
}

static int swi_runtime_suspend(struct device *dev)
{
	struct apple_swi *swi = dev_get_drvdata(dev);

	mutex_lock(&swi->lock);
	swi_release(swi);
	swi_wr(swi, UART_UCON, 0);
	mutex_unlock(&swi->lock);

	return 0;
}

static int swi_runtime_resume(struct device *dev)
{
	struct apple_swi *swi = dev_get_drvdata(dev);

	mutex_lock(&swi->lock);
	swi_uart_setup(swi);
	mutex_unlock(&swi->lock);

	return 0;
}

static struct i2c_client *swi_get_master(struct device *dev)
{
	struct device_node *np;
	struct i2c_client *client;

	np = of_parse_phandle(dev->of_node, "apple,swi-master", 0);
	if (!np)
		return ERR_PTR(-ENODEV);

	client = of_find_i2c_device_by_node(np);
	of_node_put(np);

	return client ?: ERR_PTR(-EPROBE_DEFER);
}

static void swi_put_master(void *data)
{
	put_device(&((struct i2c_client *)data)->dev);
}

static int apple_swi_setup(struct platform_device *pdev,
			   struct i2c_client *master)
{
	struct device *dev = &pdev->dev;
	struct apple_swi *swi;
	int ret;

	ret = devm_add_action_or_reset(dev, swi_put_master, master);
	if (ret)
		return ret;

	swi = devm_kzalloc(dev, sizeof(*swi), GFP_KERNEL);
	if (!swi)
		return -ENOMEM;

	swi->dev = dev;
	swi->master = master;
	platform_set_drvdata(pdev, swi);

	swi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(swi->base))
		return PTR_ERR(swi->base);

	ret = devm_mutex_init(dev, &swi->lock);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, SWI_IDLE_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	swi->di.dev = dev;
	swi->di.chip = BQ27541;
	swi->di.name = "battery";
	swi->di.bus.read = swi_bq27xxx_read;

	/* Check that the gauge answers before registering anything */
	ret = swi_bq27xxx_read(&swi->di, 0x08, false);	/* Voltage() */
	if (ret < 0)
		return dev_err_probe(dev, ret, "no answer from the gauge\n");
	dev_info(dev, "gauge found, battery at %d mV\n", ret);

	ret = bq27xxx_battery_setup(&swi->di);
	if (ret)
		return ret;

	return 0;
}

static int apple_swi_probe(struct platform_device *pdev)
{
	struct i2c_client *master;

	master = swi_get_master(&pdev->dev);
	if (IS_ERR(master))
		return dev_err_probe(&pdev->dev, PTR_ERR(master),
				     "no bus master (apple,swi-master)\n");

	return apple_swi_setup(pdev, master);
}

static void apple_swi_remove(struct platform_device *pdev)
{
	struct apple_swi *swi = platform_get_drvdata(pdev);

	bq27xxx_battery_teardown(&swi->di);
}

static DEFINE_RUNTIME_DEV_PM_OPS(apple_swi_pm_ops, swi_runtime_suspend,
				 swi_runtime_resume, NULL);

static const struct of_device_id apple_swi_of_match[] = {
	{ .compatible = "apple,t7000-swi-gauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_swi_of_match);

static struct platform_driver apple_swi_driver = {
	.driver = {
		.name = "apple-swi-battery",
		.of_match_table = apple_swi_of_match,
		.pm = pm_ptr(&apple_swi_pm_ops),
	},
	.probe = apple_swi_probe,
	.remove = apple_swi_remove,
};
module_platform_driver(apple_swi_driver);

MODULE_DESCRIPTION("Apple SWI (HDQ over UART) battery gauge driver");
MODULE_LICENSE("GPL");
