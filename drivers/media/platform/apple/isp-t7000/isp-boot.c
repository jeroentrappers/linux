// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: firmware load (stage 3), CPU start (4) and the boot
 * handshakes (5, 6), following AppleH6CamIn::ISP_Init for HW rev 4.
 */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/string.h>

#include "isp.h"

#define ISP_FW_NAME	"apple/isp-t7000.bin"

/* fabric TLIMIT list (kext __const 0xfffffff00712f9f0), rev 4/5 */
static const u16 isp_tlimit[] = {
	0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c, 0x40,
	0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c, 0x60, 0x64,
};

/*
 * Poll (reg & mask) == want every 10 ms. With @fault_abort a (new) DART fault
 * aborts early; without it the fault is only logged + acked and the wait runs
 * its full time (used for the TLIMIT drains before power-off).
 */
static int __isp_wait(struct t7000_isp *isp, u32 off, u32 mask, u32 want, uint ms,
		      const char *what, bool fault_abort)
{
	unsigned long start = jiffies, end = start + msecs_to_jiffies(ms);
	u32 v;

	dev_info(isp->dev, "waiting for %s: reg +0x%x & 0x%x == 0x%x (timeout %u ms)\n",
		 what, off, mask, want, ms);
	for (;;) {
		v = isp_rd(isp, off);
		if ((v & mask) == want) {
			dev_info(isp->dev, "got %s after %u ms (reg 0x%08x)\n", what,
				 jiffies_to_msecs(jiffies - start), v);
			return 0;
		}
		if (isp_dart_fault(isp)) {
			dev_err(isp->dev, "DART fault while waiting for %s (reg 0x%08x)%s\n", what, v,
				fault_abort ? "" : ", still waiting");
			if (fault_abort) {
				isp_log_window(isp, "on fault");
				return -EIO;
			}
		}
		if (time_after(jiffies, end))
			break;
		msleep(10);
	}
	dev_err(isp->dev, "TIMEOUT after %u ms waiting for %s: reg +0x%x = 0x%08x, want (& 0x%x) 0x%x\n",
		ms, what, off, v, mask, want);
	isp_log_window(isp, "on timeout");
	isp_dart_log(isp, "on timeout");
	return -ETIMEDOUT;
}

int isp_wait(struct t7000_isp *isp, u32 off, u32 mask, u32 want, uint ms,
	     const char *what)
{
	return __isp_wait(isp, off, mask, want, ms, what, true);
}

static void isp_fw_strings(struct t7000_isp *isp, const u8 *d, size_t len)
{
	if (len < 0x10a0) {
		dev_err(isp->dev, "firmware too short for the header strings\n");
		return;
	}
	dev_info(isp->dev, "fw header: mode '%.8s' (adc = normal) '%.8s' build '%.12s %.12s' version '%.16s' tree '%.8s'\n",
		 d + 0x1040, d + 0x1050, d + 0x1060, d + 0x106c, d + 0x1080, d + 0x1090);
}

/* dry_run: load the file, log size + strings, map nothing. */
void isp_fw_peek(struct t7000_isp *isp)
{
	const struct firmware *fw;
	int ret = request_firmware_direct(&fw, ISP_FW_NAME, isp->dev);

	if (ret) {
		dev_err(isp->dev, "dry_run: request_firmware(%s) = %d\n", ISP_FW_NAME, ret);
		return;
	}
	dev_info(isp->dev, "dry_run: %s is 0x%zx bytes (fw surface 0x%x), word0 0x%08x\n",
		 ISP_FW_NAME, fw->size, ISP_FW_SIZE_V34,
		 fw->size >= 4 ? *(const u32 *)fw->data : 0);
	isp_fw_strings(isp, fw->data, fw->size);
	release_firmware(fw);
}

int isp_stage_fw(struct t7000_isp *isp)
{
	const struct firmware *fw;
	int ret;

	ret = isp_dart_init(isp);
	if (ret)
		return ret;
	ret = request_firmware_direct(&fw, ISP_FW_NAME, isp->dev);
	if (ret) {
		dev_err(isp->dev, "request_firmware(%s) = %d (copy it to /lib/firmware/apple/)\n",
			ISP_FW_NAME, ret);
		return ret;
	}
	if (fw->size > ISP_FW_SIZE_V34 || fw->size < 0x2000) {
		dev_err(isp->dev, "firmware size 0x%zx not in 0x2000..0x%x\n", fw->size,
			ISP_FW_SIZE_V34);
		release_firmware(fw);
		return -EINVAL;
	}
	isp->fw = isp_surf_alloc(isp, ISP_FW_SIZE_V34, 0x20205746 /* "FW  " */);
	if (!isp->fw) {
		release_firmware(fw);
		return -ENOMEM;
	}
	memcpy(isp->fw->virt, fw->data, fw->size);
	dma_wmb();
	dev_info(isp->dev, "firmware 0x%zx bytes copied to iova 0x%08x, word0 0x%08x\n",
		 fw->size, isp->fw->iova, *(u32 *)isp->fw->virt);
	isp_fw_strings(isp, fw->data, fw->size);
	release_firmware(fw);
	isp_dart_log(isp, "after fw load");
	return 0;
}

/* ResetISPCPU (kext 0x846a2f4), rev 4: only when gpio1 says it was not power-cycled */
static void isp_reset_cpu(struct t7000_isp *isp)
{
	int i;

	dev_info(isp->dev, "gpio1 == 0xfeedbabe: ISP not power-cycled, running ResetISPCPU\n");
	isp_wr(isp, ISP_CTRL_V34, 0);
	msleep(50);
	for (i = 0; i < ARRAY_SIZE(isp_tlimit); i++)
		isp_wr(isp, isp_tlimit[i], 0xffff);
	for (i = 0; i < 3; i++)
		isp_wr(isp, ISP_IRQMASK(i), 0xffffffff);
	for (i = 0; i < 8; i++)
		isp_rd(isp, ISP_CPU_STAT);
	msleep(50);
}

int isp_stage_cpu(struct t7000_isp *isp)
{
	int i;

	if (isp_rd(isp, ISP_GPIO(1)) == ISP_MAGIC_IDLE)
		isp_reset_cpu(isp);
	for (i = 0; i < 8; i++)
		isp_wr(isp, ISP_GPIO(i), 0);
	isp_wr(isp, ISP_FW_BASE, isp->fw->iova);
	isp_wr(isp, ISP_WIN_0C, 0);
	isp_wr(isp, ISP_WIN_10, 0);
	isp_wr(isp, ISP_WIN_14, 0);
	isp_wr(isp, ISP_WIN_18, 0xfffff000);
	isp_wr(isp, ISP_WIN_1C, 0);
	isp_log_window(isp, "before CPU run");
	wmb();
	isp->cpu_started = true;
	isp_wr(isp, ISP_CTRL_V34, ISP_CTRL_RUN_V34);
	dev_info(isp->dev, "CTRL +0x100028 = 0x10 (readback 0x%08x): ISP CPU released\n",
		 isp_rd(isp, ISP_CTRL_V34));
	return isp_wait(isp, ISP_GPIO(7), ~0U, ISP_MAGIC_WAKE, isp_boot_timeout_ms,
			"wake magic 0x8042006 in gpio7");
}

/* Hard stop: mbox off, CTRL = 0, block new transactions, wait for drain. */
void isp_cpu_stop(struct t7000_isp *isp)
{
	int i;

	isp_wr(isp, ISP_MBOX_ENABLE, 0);
	isp_wr(isp, ISP_CTRL_V34, 0);
	msleep(10);
	for (i = 0; i < ARRAY_SIZE(isp_tlimit); i++)
		isp_wr(isp, isp_tlimit[i], 0);
	/* kext ISP_deInit always waits up to 100 ms per drain, fault or not */
	__isp_wait(isp, ISP_TLIMIT0_OUT, ~0U, 0, 100, "TLIMIT0 drain (+0x44 == 0)", false);
	__isp_wait(isp, ISP_TLIMIT1_OUT, ~0U, 0, 100, "TLIMIT1 drain (+0x68 == 0)", false);
	isp->cpu_started = false;
	isp_log_window(isp, "after CPU stop");
}

/* 64-byte-aligned bump allocator in the IPC pool (kext FUN_846b4d4 granularity) */
static int isp_pool_alloc(struct t7000_isp *isp, u32 size)
{
	u32 off = isp->pool_used;

	size = ALIGN(size, 64);
	if (!size || off + size > isp->ipc->size)
		return -1;
	isp->pool_used += size;
	return off;
}

/* Boot stage 2: host-allocated IPC queue + heap + 0x74-byte u32 boot args */
int isp_stage_args(struct t7000_isp *isp)
{
	u32 pool, heap, *a;
	int q, args, cmd, i;

	isp->nchans = isp_rd(isp, ISP_GPIO(0));
	isp->qsize = isp_rd(isp, ISP_GPIO(1)) + 1;
	heap = isp_rd(isp, ISP_GPIO(3));
	dev_info(isp->dev, "fw asks: numIPCChannels %u, ipcQueueSize 0x%x (gpio1+1), heap 0x%x (gpio3)\n",
		 isp->nchans, isp->qsize, heap);
	if (!isp->nchans || isp->nchans > ISP_MAX_CHANS || isp->qsize > SZ_4M || heap > SZ_16M) {
		dev_err(isp->dev, "implausible channel count / queue size / heap size\n");
		return -EIO;
	}
	pool = max_t(u32, ISP_IPC_POOL_MIN,
		     PAGE_ALIGN(ALIGN(isp->qsize, 64) + 0x80 + ISP_CMD_BUF_SIZE + 0x1000));
	isp->ipc = isp_surf_alloc(isp, pool, 0x20435049 /* "IPC " */);
	if (!isp->ipc)
		return -ENOMEM;
	q = isp_pool_alloc(isp, isp->qsize);
	args = isp_pool_alloc(isp, ISP_ARGS_SIZE);
	cmd = isp_pool_alloc(isp, ISP_CMD_BUF_SIZE);
	if (q < 0 || args < 0 || cmd < 0)
		return -ENOMEM;
	isp->args_off = args;
	isp->cmd_off = cmd;
	if (heap) {
		isp->heapsize = max_t(u32, heap, 0x1000);
		isp->heap = isp_surf_alloc(isp, isp->heapsize, 0x50414548 /* "HEAP" */);
		if (!isp->heap)
			return -ENOMEM;
	}

	a = (u32 *)((u8 *)isp->ipc->virt + args);
	a[0] = isp->fw->iova;				/* absolute fw IOVA */
	a[1] = isp_rel(isp->ipc->iova + q);		/* queue, fw-relative */
	a[2] = ISP_FW_SIZE_V34;
	a[3] = ISP_IOVA_SPAN - ISP_FW_SIZE_V34;
	a[4] = isp->heap ? isp_rel(isp->heap->iova) : 0;
	a[5] = isp->heap ? isp->heapsize : 0;
	a[6] = isp_platform_id;
	a[0xf] = 0xc;
	for (i = 0; i < ISP_ARGS_SIZE / 4; i += 8)
		dev_info(isp->dev, "bootargs[%02x]: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			 i, a[i], a[i + 1], a[i + 2], a[i + 3], a[i + 4], a[i + 5],
			 i + 6 < ISP_ARGS_SIZE / 4 ? a[i + 6] : 0,
			 i + 7 < ISP_ARGS_SIZE / 4 ? a[i + 7] : 0);
	dma_wmb();
	isp_wr(isp, ISP_GPIO(0), isp_rel(isp->ipc->iova + args));
	dev_info(isp->dev, "gpio0 = 0x%08x (fw-rel boot args), gpio7 = 0xf7fbdff9\n",
		 isp_rel(isp->ipc->iova + args));
	isp_wr(isp, ISP_GPIO(7), ISP_MAGIC_ARGS);
	return isp_wait(isp, ISP_GPIO(7), ~0U, ISP_MAGIC_WAKE, isp_boot_timeout_ms,
			"second magic 0x8042006 in gpio7 (channel table ready)");
}

/* Boot stage 3: parse the channel table, prime the command rings, release the fw */
int isp_stage_chans(struct t7000_isp *isp)
{
	u32 tbl = isp_rd(isp, ISP_GPIO(0)), n, j;
	const u8 *d = isp_translate(isp, tbl, isp->nchans * ISP_CHAN_DESC_SIZE);

	if (!d) {
		dev_err(isp->dev, "channel table at fw-rel 0x%08x is outside every surface\n", tbl);
		return -EIO;
	}
	dev_info(isp->dev, "channel table at fw-rel 0x%08x, %u channels\n", tbl, isp->nchans);
	for (n = 0; n < isp->nchans; n++, d += ISP_CHAN_DESC_SIZE) {
		struct isp_chan *c = &isp->chans[n];
		const u32 *w = (const u32 *)(d + 0x40);

		memcpy(c->name, d, 0x40);
		c->name[0x40] = 0;
		c->type = w[0];
		c->src = w[1];
		c->num = w[2];
		c->rel = w[3];
		c->idx = 0;
		dev_info(isp->dev, "chan %u '%s': type %u src %u num %u ring fw-rel 0x%08x\n",
			 n, c->name, c->type, c->src, c->num, c->rel);
		if (!c->num || c->num > 1024 || c->src > 31 ||
		    !(c->ring = isp_translate(isp, c->rel, c->num * ISP_MSG_SIZE))) {
			dev_err(isp->dev, "chan %u: bad ring (num %u src %u rel 0x%x)\n",
				n, c->num, c->src, c->rel);
			return -EIO;
		}
		if (c->type != CHAN_CMD)
			continue;
		for (j = 0; j < c->num; j++) {
			u32 *m = (u32 *)(c->ring + j * ISP_MSG_SIZE);

			m[1] = 0;
			m[2] = 0;
			m[0] = 1;	/* slot owned by the host */
		}
	}
	dma_wmb();
	isp_wr(isp, ISP_GPIO(3), ISP_MAGIC_WAKE);
	return isp_wait(isp, ISP_GPIO(3), ~0U, 0, isp_boot_timeout_ms,
			"gpio3 0x8042006 -> 0 (magic 1)");
}
