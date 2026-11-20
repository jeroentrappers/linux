// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 GPU (G4P): GPU MMU for the firmware image and META boot (milestone M3).
 *
 * Follows the direct (no-GPC, gpu_gpc=0) path of AGXMetaFirmware::prepareFirmwareForBoot / bootFirmware
 * (linux-on-iphone6 re/GPU-META-BOOT.md 6). M3 gives the firmware its image only: no shared structures
 * (the pointer pair at image + 0xff8 is zero) and no register windows, so the firmware is expected to stop
 * or fault early. The checks are whether the META threads run (slave port), whether the firmware wrote
 * into its own stacks, and the BIF fault registers.
 *
 * Safety: the GPU MMU maps exactly the image pages, so the GPU can reach no other memory. All buffers are
 * freed only after the GPU is powered off.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "g4p.h"

/* Rogue registers (upstream ROGUE_CR_*), as AGX writes them (re/GPU-BRINGUP.md 4.1) */
#define RGX_CR_SOFT_RESET		0x0100
#define RGX_CR_EVENT_STATUS		0x0130
#define RGX_CR_META_SP_MSLVDATAX	0x0a00
#define RGX_CR_META_SP_MSLVCTRL0	0x0a10
#define RGX_CR_META_SP_MSLVCTRL0_RD	BIT(0)
#define RGX_CR_META_SP_MSLVCTRL1	0x0a18
#define RGX_CR_META_SP_MSLVCTRL1_IDLE	(BIT(24) | BIT(26))	/* READY | GBLPORT_IDLE */
#define RGX_CR_MTS_SCHEDULE		0x0b00
#define RGX_CR_MTS_SCHEDULE_DM0	0x10	/* TASK_COUNTED, DM 0: AGX's kick (0x…8501704) */
#define RGX_CR_MTS_SCHEDULE_DM2	0x12	/* DM 2 = CL (0x…850122c, re/GPU-FIRST-JOB.md 2.2) */

#define G4P_CHAN_RING_SIZE	144	/* 9 x 16, as AGXChannel::init */
#define RGX_CR_META_BOOT		0x0bf8
#define RGX_CR_META_SP_MSLVIRQSTATUS	0x0ac8	/* TRIGVECT2 (bit 2): firmware -> host; write 0 = ack (AGX) */
#define RGX_CR_BIF_CAT_BASE0		0x1200
#define RGX_CR_BIF_FAULT_BANK0_MMU	0x12b0
#define RGX_CR_BIF_FAULT_BANK0_REQ	0x12b8
#define RGX_CR_BIF_FAULT_BANK1_MMU	0x12c0
#define RGX_CR_BIF_FAULT_BANK1_REQ	0x12c8
#define RGX_CR_SLC_CTRL_MISC		0x3800
#define RGX_CR_SLC_3808			0x3808	/* Apple/variant SLC control, no upstream name */

#define G4P_SOFT_RESET_ALL		0x00e7fffffffffc1dULL
#define G4P_SOFT_RESET_NO_RASCALDUSTS	0x00e7fe071ffffc1dULL
#define G4P_SOFT_RESET_GARTEN		0x0000000100000000ULL
#define G4P_SLC_CTRL_MISC		0x00200001ULL	/* PVR_HASH1 | BYPASS_BURST_COMBINER */

/* META registers through the slave port (upstream pvr_rogue_meta.h) */
#define META_CR_TXENABLE(t)		(0x04800000 + 0x1000 * (t))
#define META_CR_TXSTATUS(t)		(0x04800010 + 0x1000 * (t))
#define META_CR_MMCU_SEGMENT_BASE(n)	(0x04850000 + 0x10 * (n))


/* boot arguments (re/GPU-META-BOOT.md 3) */
#define G4P_BOOTARG_OFFSET		0x200
#define G4P_ROOTS_OFFSET		0xff8	/* u32 root A, u32 root B (FW addresses); 0 for M3 */
#define G4P_SEG_BIFDM			7

struct g4p_boot {
	struct g4p_mmu mmu;
	struct g4p_bo image;	/* the only buffer in the FW heap: GPU VA 0xe1c0000000 */
	struct g4p_fwinit fi;	/* shared structures (M4) */
	u32 roots[2];		/* FW addresses of root A, root B */
	bool alive;
	u32 nr_events, ev_type, ev_payload;
	unsigned int alive_ms;
	u32 fender_perf;
	u32 devctl_sent, devctl_done, devctl_last_us;
	int irqs;	/* firmware interrupts seen by the end of the alive wait */
	/* CL channel (re/GPU-FIRST-JOB.md 2.1): state in heap 7, ring ctl + command pointers in heap 6 */
	struct g4p_bo cl_state, cl_ring;
	int cl_ret;
	u32 cl_us, cl_state_1c, cl_state_38, cl_ring_30, cl_rd;
	/* results */
	int ret;
	u64 soft_reset;
	u32 mslvctrl1, txenable[2], txstatus[2], meta_ok;
	u64 fault[4];
	u64 event_status;
	unsigned int stack_dirty[2];	/* nonzero words in the t0 / t1 stacks after boot */
};

static int g4p_meta_read(struct g4p *g, u32 addr, u32 *val)
{
	u32 v;
	int ret;

	ret = readl_poll_timeout(g->rgx + RGX_CR_META_SP_MSLVCTRL1, v,
				 (v & RGX_CR_META_SP_MSLVCTRL1_IDLE) == RGX_CR_META_SP_MSLVCTRL1_IDLE, 10, 10000);
	if (ret)
		return ret;
	writel(addr | RGX_CR_META_SP_MSLVCTRL0_RD, g->rgx + RGX_CR_META_SP_MSLVCTRL0);
	readl(g->rgx + RGX_CR_META_SP_MSLVCTRL0);
	ret = readl_poll_timeout(g->rgx + RGX_CR_META_SP_MSLVCTRL1, v,
				 (v & RGX_CR_META_SP_MSLVCTRL1_IDLE) == RGX_CR_META_SP_MSLVCTRL1_IDLE, 10, 10000);
	if (ret)
		return ret;
	*val = readl(g->rgx + RGX_CR_META_SP_MSLVDATAX);
	return 0;
}

static void g4p_wr64_fenced(struct g4p *g, u64 val, u32 reg)
{
	writeq(val, g->rgx + reg);
	readq(g->rgx + reg);
}

static int g4p_boot_args(struct g4p *g, struct g4p_boot *b)
{
	static const unsigned int seg_region[] = { 0, 1, 3 };	/* segment n = region n; region 2 is the default */
	u32 *arg = b->image.cpu + G4P_BOOTARG_OFFSET;
	u32 *end = b->image.cpu + G4P_ROOTS_OFFSET;
	const struct g4p_fw *fw = &g->fw;

	if (fw->region[0].offset < G4P_ROOTS_OFFSET + 8)	/* args + roots live in region 2's page */
		return -EINVAL;
	for (unsigned int i = 0; i < ARRAY_SIZE(seg_region); i++) {
		const struct g4p_fw_region *r = &fw->region[seg_region[i]];
		u64 va = b->image.gpuva + r->offset;
		u32 seg = META_CR_MMCU_SEGMENT_BASE(seg_region[i]);

		if (!r->used)
			continue;
		if (end - arg < 8)
			return -ENOSPC;
		*arg++ = seg;		*arg++ = r->lo | 0xf02;		/* BASE: all threads, writable */
		*arg++ = seg + 4;	*arg++ = max_t(u32, r->hi - r->lo, SZ_4K) - 1;	/* LIMIT */
		*arg++ = seg + 8;	*arg++ = lower_32_bits(va);	/* OUTA0 */
		*arg++ = seg + 12;	*arg++ = (G4P_SEG_BIFDM << 8) | upper_32_bits(va);	/* OUTA1 */
	}
	for (unsigned int i = 0; i < fw->nr_cfg; i++) {
		if (end - arg < 2)
			return -ENOSPC;
		*arg++ = fw->cfg[i].reg;
		*arg++ = fw->cfg[i].val;
	}
	if (end - arg < 3)
		return -ENOSPC;
	*arg++ = 0;
	*arg++ = 0;
	*arg++ = g4p_fw_addr(b->image.gpuva + G4P_ROOTS_OFFSET, false);
	end[0] = b->roots[0];	/* root A */
	end[1] = b->roots[1];	/* root B */
	return 0;
}

static void g4p_boot_free_mem(struct device *dev, struct g4p_boot *b)
{
	g4p_bo_free(dev, &b->cl_state);
	g4p_bo_free(dev, &b->cl_ring);
	g4p_fwinit_free(dev, &b->fi);
	g4p_bo_free(dev, &b->image);
	g4p_mmu_fini(dev, &b->mmu);
	kfree(b);
}

/* the IRQ handler reads RGX: on only between a boot and the power-off */
void g4p_boot_irq_off(struct g4p *g)
{
	if (g->irq_on) {
		disable_irq(g->irq);
		g->irq_on = false;
	}
}

/* only once the GPU is powered off: until then it may still access these pages */
void g4p_boot_free(struct g4p *g)
{
	if (!g->boot)
		return;
	WARN_ON(g->powered);
	g4p_boot_free_mem(g->dev, g->boot);
	g->boot = NULL;
}

static unsigned int g4p_count_nonzero(const void *p, size_t len)
{
	const u32 *w = p;
	unsigned int n = 0;

	for (size_t i = 0; i < len / 4; i++)
		n += !!READ_ONCE(w[i]);
	return n;
}

/* caller: lock held, GPU powered with the core ID read, firmware loaded, no earlier boot */
int g4p_boot_start(struct g4p *g)
{
	const struct g4p_fw *fw = &g->fw;
	struct g4p_boot *b;
	int ret;

	if (g->boot)
		return -EBUSY;
	b = kzalloc_obj(*b);
	if (!b)
		return -ENOMEM;
	ret = g4p_mmu_init(g->dev, &b->mmu);
	if (!ret)
		ret = g4p_bo_alloc(g->dev, &b->mmu, G4P_HEAP_FW, fw->image_size, &b->image);
	if (!ret)
		ret = g4p_fw_build_image(fw, b->image.cpu, b->image.size);
	if (!ret)
		ret = g4p_fwinit_build(g, &b->mmu, &b->fi, &b->image, b->roots);
	if (!ret)
		ret = g4p_boot_args(g, b);
	if (ret) {
		g4p_boot_free_mem(g->dev, b);
		return ret;
	}

	wmb();	/* tables and image in memory before the GPU can fetch them */
	g->boot = b;
	if (g->irq >= 0 && !g->irq_on) {
		atomic_set(&g->irq_count, 0);
		enable_irq(g->irq);
		g->irq_on = true;
	}

	dev_info(g->dev, "META boot: image %pad (0x%zx), PC %pad, %u boot args\n", &b->image.dma, b->image.size,
		 &b->mmu.pc_dma, fw->nr_cfg + 12);

	/* AGX prepareFirmwareForBoot, direct path, with upstream pvr_fw_start()'s read-back fences */
	g4p_wr64_fenced(g, G4P_SOFT_RESET_ALL, RGX_CR_SOFT_RESET);
	g4p_wr64_fenced(g, G4P_SOFT_RESET_NO_RASCALDUSTS, RGX_CR_SOFT_RESET);
	g4p_wr64_fenced(g, G4P_SOFT_RESET_GARTEN, RGX_CR_SOFT_RESET);
	writel(1, g->rgx + RGX_CR_META_BOOT);
	readl(g->rgx + RGX_CR_META_BOOT);
	g4p_wr64_fenced(g, b->mmu.pc_dma, RGX_CR_BIF_CAT_BASE0);
	g4p_wr64_fenced(g, G4P_SLC_CTRL_MISC, RGX_CR_SLC_CTRL_MISC);
	g4p_wr64_fenced(g, 1, RGX_CR_SLC_3808);
	udelay(3);
	/* bootFirmware: release META */
	g4p_wr64_fenced(g, 0, RGX_CR_SOFT_RESET);
	udelay(3);
	/* waitForFWAlive: iOS waits up to 10 s; event type 0 = alive (re/GPU-FW-INIT.md 7) */
	for (b->alive_ms = 0; b->alive_ms < 2000 && !b->alive; b->alive_ms += 10) {
		int seen = atomic_read(&g->irq_count);

		u32 type, payload;

		while (g4p_fwinit_event(&b->fi, &type, &payload)) {
			if (!b->nr_events++ || type == 0) {
				b->ev_type = type;
				b->ev_payload = payload;
			}
			if (type == 0)
				b->alive = true;
		}
		if (b->alive)
			break;
		if (g->irq_on)	/* the event producer raises the IRQ (GPU-BRINGUP 8.28); poll as fallback */
			wait_event_timeout(g->irq_wq, atomic_read(&g->irq_count) != seen, msecs_to_jiffies(10));
		else
			usleep_range(10000, 11000);
	}
	b->irqs = atomic_read(&g->irq_count);

	/* RGX reads only while the Fender still reports a non-zero perf state with no request pending */
	b->fender_perf = readl(g->fender + 0x1000);
	if (!(b->fender_perf & 0xf) || (b->fender_perf & BIT(4))) {
		dev_warn(g->dev, "META boot: Fender perf 0x%08x after boot, not reading RGX\n", b->fender_perf);
		b->ret = -EIO;
		return 0;
	}
	b->soft_reset = readq(g->rgx + RGX_CR_SOFT_RESET);
	b->event_status = readq(g->rgx + RGX_CR_EVENT_STATUS);
	b->mslvctrl1 = readl(g->rgx + RGX_CR_META_SP_MSLVCTRL1);
	b->fault[0] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK0_MMU);
	b->fault[1] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK0_REQ);
	b->fault[2] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK1_MMU);
	b->fault[3] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK1_REQ);
	b->ret = 0;
	for (unsigned int t = 0; t < 2 && !b->ret; t++) {
		b->ret = g4p_meta_read(g, META_CR_TXENABLE(t), &b->txenable[t]);
		if (!b->ret)
			b->ret = g4p_meta_read(g, META_CR_TXSTATUS(t), &b->txstatus[t]);
	}
	b->meta_ok = !b->ret;
	/* stacks grow upwards from SP (re/GPU-BRINGUP.md 3.2): look at the first 4 KiB above each SP */
	for (unsigned int i = 0; i < fw->nr_threads && i < 2; i++) {
		const struct g4p_fw_thread *t = &fw->thread[i];
		const struct g4p_fw_region *r = &fw->region[(t->sp >> 29) & 3];
		size_t off = r->offset + (t->sp & ~BIT(31)) - r->lo;

		if (t->thread < 2 && off + SZ_4K <= b->image.size)
			b->stack_dirty[t->thread] = g4p_count_nonzero(b->image.cpu + off, SZ_4K);
	}
	dev_info(g->dev, "META boot: soft_reset 0x%llx mslvctrl1 0x%08x T0 en 0x%x st 0x%x T1 en 0x%x st 0x%x (sp %d)\n",
		 b->soft_reset, b->mslvctrl1, b->txenable[0], b->txstatus[0], b->txenable[1], b->txstatus[1], b->ret);
	dev_info(g->dev, "META boot: %d firmware IRQs during the alive wait\n", b->irqs);
	dev_info(g->dev, "META boot: %s after %u ms (%u events, first type %u payload 0x%x), roots 0x%08x 0x%08x\n",
		 b->alive ? "firmware ALIVE" : "no alive event", b->alive_ms, b->nr_events, b->ev_type, b->ev_payload,
		 b->roots[0], b->roots[1]);
	dev_info(g->dev, "META boot: stack words written t0 %u t1 %u; BIF fault 0x%llx/0x%llx 0x%llx/0x%llx; event 0x%llx\n",
		 b->stack_dirty[0], b->stack_dirty[1], b->fault[0], b->fault[1], b->fault[2], b->fault[3],
		 b->event_status);
	return 0;
}

/*
 * M5 step 1: one device-control no-op. Types 9..11 (RDE*) are only consumed by the firmware's dispatcher
 * (fw 0x1000328c: they branch to 0x100033d4, which advances the read index), so the response to check is
 * the firmware read index catching up with ours.
 */
int g4p_boot_devctl_nop(struct g4p *g, u32 type)
{
	struct g4p_boot *b = g->boot;
	u32 entry[5] = { type };
	u32 perf;
	int wr;

	if (type < 9 || type > 11)
		return -EINVAL;
	if (!b || !b->alive || !g->powered)
		return -ENODEV;
	perf = readl(g->fender + 0x1000);
	if (!(perf & 0xf) || (perf & BIT(4)))
		return -EIO;	/* GPU not in a running perf state: no RGX access */
	wr = g4p_fwinit_ring_push(&b->fi, 3, entry);
	if (wr < 0)
		return wr;
	writel(RGX_CR_MTS_SCHEDULE_DM0, g->rgx + RGX_CR_MTS_SCHEDULE);
	readl(g->rgx + RGX_CR_MTS_SCHEDULE);
	b->devctl_sent++;
	for (b->devctl_last_us = 0; b->devctl_last_us < 100000; b->devctl_last_us += 10) {
		if (g4p_fwinit_ring_read_index(&b->fi, 3) == (u32)wr) {
			b->devctl_done++;
			dev_info(g->dev, "devctl type %u: consumed after %u us (index %d)\n", type, b->devctl_last_us, wr);
			return 0;
		}
		udelay(10);
	}
	dev_warn(g->dev, "devctl type %u: not consumed in 100 ms (read %u, write %d)\n", type,
		 g4p_fwinit_ring_read_index(&b->fi, 3), wr);
	return -ETIMEDOUT;
}

/*
 * M5 checkpoint C1 (re/GPU-FIRST-JOB.md 7): an empty CL channel. Channel state + a channel ring with no
 * commands, one ring 2 entry {2, FW(state), write index 0, first submit}, kick DM 2. Expected: ring 2's
 * read index advances; the firmware queues the channel (state +0x1c = 1, +0x38 = 1, ring ctl +0x30 = 0)
 * and its scheduler finds no command (ctl +0x00 == +0x40). Everything the firmware writes back is passed
 * in the uncached address form so the host sees it (META's data cache is on).
 */
int g4p_boot_cl_c1(struct g4p *g)
{
	struct g4p_boot *b = g->boot;
	u32 entry[5] = { 2 };
	void *st, *ctl;
	u32 perf;
	int wr, ret;

	if (!b || !b->alive || !g->powered)
		return -ENODEV;
	if (b->cl_state.cpu)
		return -EBUSY;	/* one C1 per boot */
	perf = readl(g->fender + 0x1000);
	if (!(perf & 0xf) || (perf & BIT(4)))
		return -EIO;
	ret = g4p_bo_alloc(g->dev, &b->mmu, G4P_HEAP_PRIVATE, 0x80, &b->cl_state);
	if (!ret)
		ret = g4p_bo_alloc(g->dev, &b->mmu, G4P_HEAP_SHARED, 0x60 + 4 * G4P_CHAN_RING_SIZE, &b->cl_ring);
	if (ret)
		return ret;
	st = b->cl_state.cpu;
	ctl = b->cl_ring.cpu;
	/* resetChannelState: ring ctl indices 0, +0x50 = size; state +0 ctl, +4 pointer array, +0x18 = ~0 */
	*(u32 *)(ctl + 0x50) = G4P_CHAN_RING_SIZE;
	*(u32 *)(st + 0x00) = g4p_fw_addr(b->cl_ring.gpuva, false);
	*(u32 *)(st + 0x04) = g4p_fw_addr(b->cl_ring.gpuva + 0x60, false);
	*(u32 *)(st + 0x18) = 0xffffffff;
	wmb();	/* channel objects before the ring entry that names them */

	entry[1] = g4p_fw_addr(b->cl_state.gpuva, false);
	entry[2] = 0;	/* channel ring write index: no commands */
	entry[3] = 1;	/* first submission */
	wr = g4p_fwinit_ring_push(&b->fi, 2, entry);
	if (wr < 0)
		return wr;
	writel(RGX_CR_MTS_SCHEDULE_DM2, g->rgx + RGX_CR_MTS_SCHEDULE);
	readl(g->rgx + RGX_CR_MTS_SCHEDULE);
	b->cl_ret = -ETIMEDOUT;
	for (b->cl_us = 0; b->cl_us < 100000; b->cl_us += 10) {
		if (g4p_fwinit_ring_read_index(&b->fi, 2) == (u32)wr) {
			b->cl_ret = 0;
			break;
		}
		udelay(10);
	}
	udelay(100);	/* let the scheduler pass run before reading the channel back */
	rmb();
	b->cl_rd = g4p_fwinit_ring_read_index(&b->fi, 2);
	b->cl_state_1c = READ_ONCE(*(u32 *)(st + 0x1c));
	b->cl_state_38 = READ_ONCE(*(u32 *)(st + 0x38));
	b->cl_ring_30 = READ_ONCE(*(u32 *)(ctl + 0x30));
	dev_info(g->dev, "C1: ring 2 entry %s after %u us (read %u, write %d); chan state +0x1c %u +0x38 %u, ring ctl +0x30 %u +0x00 %u +0x10 %u\n",
		 b->cl_ret ? "NOT consumed" : "consumed", b->cl_us, b->cl_rd, wr, b->cl_state_1c, b->cl_state_38,
		 b->cl_ring_30, READ_ONCE(*(u32 *)ctl), READ_ONCE(*(u32 *)(ctl + 0x10)));
	return b->cl_ret;
}

void g4p_boot_show(struct seq_file *s, struct g4p *g)
{
	struct g4p_boot *b = g->boot;

	if (!b) {
		seq_puts(s, "boot: none\n");
		return;
	}
	seq_printf(s, "boot: image %pad size 0x%zx pc %pad\n", &b->image.dma, b->image.size, &b->mmu.pc_dma);
	seq_printf(s, "  soft_reset 0x%016llx event_status 0x%016llx mslvctrl1 0x%08x\n", b->soft_reset,
		   b->event_status, b->mslvctrl1);
	seq_printf(s, "  slave port %s: T0 enable 0x%08x status 0x%08x, T1 enable 0x%08x status 0x%08x\n",
		   b->meta_ok ? "ok" : "timeout", b->txenable[0], b->txstatus[0], b->txenable[1], b->txstatus[1]);
	seq_printf(s, "  stack words written: t0 %u t1 %u (of 1024)\n", b->stack_dirty[0], b->stack_dirty[1]);
	seq_printf(s, "  BIF fault bank0 mmu 0x%016llx req 0x%016llx bank1 mmu 0x%016llx req 0x%016llx\n",
		   b->fault[0], b->fault[1], b->fault[2], b->fault[3]);
	seq_printf(s, "  fender perf after boot 0x%08x\n", b->fender_perf);
	seq_printf(s, "  alive %d after %u ms, %u events (type %u payload 0x%x), roots 0x%08x 0x%08x\n", b->alive,
		   b->alive_ms, b->nr_events, b->ev_type, b->ev_payload, b->roots[0], b->roots[1]);
	seq_printf(s, "  irq %d: %d during the alive wait, %d total\n", g->irq, b->irqs, atomic_read(&g->irq_count));
	seq_printf(s, "  devctl: %u sent, %u consumed (last %u us), read index %u\n", b->devctl_sent, b->devctl_done,
		   b->devctl_last_us, g4p_fwinit_ring_read_index(&b->fi, 3));
	if (b->cl_state.cpu)
		seq_printf(s, "  C1: %d after %u us, ring 2 read %u; chan state +0x1c %u +0x38 %u, ring ctl +0x30 %u\n",
			   b->cl_ret, b->cl_us, b->cl_rd, b->cl_state_1c, b->cl_state_38, b->cl_ring_30);
	g4p_fwinit_show_log(s, &b->fi);
	seq_printf(s, "  boot args: first [0x%08x]=0x%08x\n", ((u32 *)(b->image.cpu + G4P_BOOTARG_OFFSET))[0],
		   ((u32 *)(b->image.cpu + G4P_BOOTARG_OFFSET))[1]);
}
