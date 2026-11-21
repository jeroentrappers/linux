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

static unsigned int cl_kick = RGX_CR_MTS_SCHEDULE_DM2;
module_param(cl_kick, uint, 0644);
MODULE_PARM_DESC(cl_kick, "MTS_SCHEDULE value for the C1 kick (bring-up experiments; AGX uses 0x12 for CL)");

static bool gart_inval = true;
module_param(gart_inval, bool, 0644);
MODULE_PARM_DESC(gart_inval, "ask the firmware to invalidate the GPU MMU caches after post-boot mappings");

static bool ack_rings = true;
module_param(ack_rings, bool, 0644);
MODULE_PARM_DESC(ack_rings, "consume the trace/statistics rings so the firmware does not block on them");

static unsigned int c3_var;
module_param(c3_var, uint, 0644);
MODULE_PARM_DESC(c3_var, "C3/C4 variant (GPU-BRINGUP 8.35): 1 = stream base at an unmapped VA, 2 = CDM context state base set (C3)");
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
	/*
	 * Job memory mapped before the firmware boots (GPU-BRINGUP 8.34): pages mapped after boot were not
	 * seen by the GPU MMU on the second post-boot mapping even with gart_inval.
	 */
	struct g4p_bo arena[2];		/* [0] heap 7 (private), [1] heap 6 (shared) */
	size_t arena_used[2];
	struct g4p_bo cl_state, cl_ring;
	struct g4p_bo pds;		/* one page at G4P_PDS_VA: C4's CDM terminate program */
	struct g4p_cljob {
		struct g4p_bo cmd, stream, cdm, misc;
		unsigned int which, stream_words, us;
		u32 ring2_rd, chan_rd, chan_10, stamp0, stamp1;
		int irq0, irqs;
		u64 fault[4];
	} job;
	int cl_ret;
	u32 cl_us, cl_state_1c, cl_state_38, cl_ring_30, cl_rd, cl_state_24;
	u64 cl_fault[4], cl_event_status;
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

/*
 * C4 (GPU-BRINGUP 8.35): with a context-state buffer the CDM reads PDS memory at 0x300000000 = PDS offset 0,
 * so the PDS execution base is 0x300000000 here; iOS's CDM terminate program sits at offset 0xde0. One page,
 * mapped before boot like the job arena, holds Mesa's fence-terminate program (pvr_pds.c
 * pvr_pds_generate_fence_terminate_program, encoded with pvr_rogue_pds_encode.h): code at +0x100, its
 * 16-byte data segment (two zero 64-bit DOUTC sources) at +0x200.
 */
#define G4P_PDS_VA		0x300000000ULL
#define G4P_PDS_TERM_CODE	0x100
#define G4P_PDS_TERM_DATA	0x200

static const u32 g4p_pds_fence_terminate[] = {
	0xc1100001,	/* bra if0, setc if1, +1: is this the coefficient-sync task? */
	0xdb000000,	/* halt (if1) */
	0xc1080001,	/* bra if0, setc if0, +1: default predicate back to if0 */
	0xf0000005,	/* doutc */
	0xd3000000,	/* halt */
};

/* doutc; halt only (pvr_pds_generate_doutc, without the barrier-conditional prefix): iOS's code fits 16 bytes */
static const u32 g4p_pds_doutc_halt[] = { 0xf0000005, 0xd3000000 };

static unsigned int c4_prog;
static bool c4_ctx;
static unsigned int c4_pattern = 3;	/* 8.38: an empty resume stream completes the job */
module_param(c4_pattern, uint, 0644);
MODULE_PARM_DESC(c4_pattern, "C4: fill the CDM context-state buffer with word i = 0x10000 + 0x10 * i (it does: word 0, 8.37); 2 = words 0..2 = the fence-terminate program (data, task word, code), 3 = STREAM_TERMINATE (the buffer is the CDM resume control stream, Mesa lls.xml)");
module_param(c4_ctx, bool, 0644);
MODULE_PARM_DESC(c4_ctx, "C4: point the context PDS fields (cmd+0x224/+0x22c -> RGX 0x4a8/0x4b0) at a halt program in the page");
module_param(c4_prog, uint, 0644);
MODULE_PARM_DESC(c4_prog, "C4 terminate program: 0 = Mesa fence terminate (5 instructions), 1 = doutc; halt, 2 = code address on an unmapped PDS page (does the CDM fetch it?), 3 = also a halt at PDS offset 0, 4 = page moved to +0x1000 (offset 0 unmapped)");

static void g4p_boot_free_mem(struct device *dev, struct g4p_boot *b);

static int g4p_boot_pds_page(struct g4p *g, struct g4p_boot *b)
{
	struct g4p_bo *p = &b->pds;

	p->cpu = dma_alloc_coherent(g->dev, SZ_4K, &p->dma, GFP_KERNEL);
	if (!p->cpu)
		return -ENOMEM;
	p->size = SZ_4K;
	p->gpuva = G4P_PDS_VA + (c4_prog == 4 ? SZ_4K : 0);
	if (c4_prog == 3)
		*(u32 *)p->cpu = 0xd3000000;	/* halt: for a program that the CDM runs from a zero register */
	*(u32 *)(p->cpu + 0x300) = 0xd3000000;	/* halt: c4_ctx's context program; data at +0x400 (zero) */
	if (c4_prog == 1)
		memcpy(p->cpu + G4P_PDS_TERM_CODE, g4p_pds_doutc_halt, sizeof(g4p_pds_doutc_halt));
	else
		memcpy(p->cpu + G4P_PDS_TERM_CODE, g4p_pds_fence_terminate, sizeof(g4p_pds_fence_terminate));
	return g4p_mmu_map_page(g->dev, &b->mmu, p->gpuva, p->dma, G4P_PTE_SLC_BYPASS);
}

static void g4p_boot_free_mem(struct device *dev, struct g4p_boot *b)
{
	g4p_bo_free(dev, &b->arena[0]);
	g4p_bo_free(dev, &b->arena[1]);
	g4p_bo_free(dev, &b->pds);
	g4p_bo_free(dev, &b->job.cmd);
	g4p_bo_free(dev, &b->job.stream);
	g4p_bo_free(dev, &b->job.cdm);
	g4p_bo_free(dev, &b->job.misc);
	g4p_bo_free(dev, &b->cl_state);
	g4p_bo_free(dev, &b->cl_ring);
	g4p_fwinit_free(dev, &b->fi);
	g4p_bo_free(dev, &b->image);
	g4p_mmu_fini(dev, &b->mmu);
	kfree(b);
}

#define G4P_ARENA_SIZE	SZ_64K

/* a 4 KiB aligned piece of the pre-boot arena: [0] private (heap 7), [1] shared (heap 6) */
static int g4p_carve(struct g4p_boot *b, unsigned int which, size_t size, struct g4p_bo *bo)
{
	size_t off = b->arena_used[which];

	size = ALIGN(size, SZ_4K);
	if (!b->arena[which].cpu || off + size > b->arena[which].size)
		return -ENOSPC;
	*bo = (struct g4p_bo){ .cpu = b->arena[which].cpu + off, .dma = b->arena[which].dma + off, .size = size,
			       .gpuva = b->arena[which].gpuva + off, .view = true };
	memset(bo->cpu, 0, size);
	b->arena_used[which] = off + size;
	return 0;
}

/* the IRQ handler reads RGX: on only between a boot and the power-off */
void g4p_boot_irq_off(struct g4p *g)
{
	if (g->irq_on) {
		disable_irq(g->irq);
		g->irq_on = false;
	}
}

/* from the IRQ handler: keep the host-bound trace/statistics rings drained */
void g4p_boot_irq_rings(struct g4p *g)
{
	if (ack_rings && g->boot->fi.list_b.cpu)
		g4p_fwinit_ack_trace_stats(&g->boot->fi);
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
		ret = g4p_bo_alloc(g->dev, &b->mmu, G4P_HEAP_PRIVATE, G4P_ARENA_SIZE, &b->arena[0]);
	if (!ret)
		ret = g4p_bo_alloc(g->dev, &b->mmu, G4P_HEAP_SHARED, G4P_ARENA_SIZE, &b->arena[1]);
	if (!ret)
		ret = g4p_boot_pds_page(g, b);
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
	if (ack_rings)
		g4p_fwinit_ack_trace_stats(&b->fi);

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
static bool g4p_gpu_running(struct g4p *g)
{
	u32 perf = readl(g->fender + 0x1000);

	return (perf & 0xf) && !(perf & BIT(4));
}

/* channel state + channel ring as AGXChannel::init / resetChannelState; no submission */
static int g4p_cl_channel(struct g4p_boot *b)
{
	void *st, *ctl;
	int ret;

	if (b->cl_state.cpu)
		return 0;
	ret = g4p_carve(b, 0, 0x80, &b->cl_state);
	if (!ret)
		ret = g4p_carve(b, 1, 0x60 + 4 * G4P_CHAN_RING_SIZE, &b->cl_ring);
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
	return 0;
}

/*
 * C1 submits the channel with no command. NOTE (GPU-BRINGUP 8.34): the firmware's scheduler then fetches the
 * current command pointer anyway, reads 0 and T0 faults; iOS never submits an empty channel. Kept as a
 * diagnostic only: it kills the firmware until the next power cycle.
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
	if (!g4p_gpu_running(g))
		return -EIO;
	ret = g4p_cl_channel(b);
	if (ret)
		return ret;
	st = b->cl_state.cpu;
	ctl = b->cl_ring.cpu;
	if (gart_inval)	/* new pages after boot: the BIF has a stale PT line (re/GPU-C1-DEBUG.md) */
		g4p_fwinit_gart_inval(&b->fi);

	entry[1] = g4p_fw_addr(b->cl_state.gpuva, false);
	entry[2] = 0;	/* channel ring write index: no commands */
	entry[3] = 1;	/* first submission */
	wr = g4p_fwinit_ring_push(&b->fi, 2, entry);
	if (wr < 0)
		return wr;
	writel(cl_kick & 0xff, g->rgx + RGX_CR_MTS_SCHEDULE);
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
	b->cl_state_24 = READ_ONCE(*(u32 *)(st + 0x24));
	/* live MMU fault state after C1 (b->fault[] is the boot-time snapshot) */
	perf = readl(g->fender + 0x1000);
	if ((perf & 0xf) && !(perf & BIT(4))) {
		b->cl_fault[0] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK0_MMU);
		b->cl_fault[1] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK0_REQ);
		b->cl_fault[2] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK1_MMU);
		b->cl_fault[3] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK1_REQ);
		b->cl_event_status = readq(g->rgx + RGX_CR_EVENT_STATUS);
	}
	dev_info(g->dev, "C1: gart_inval %d; live BIF fault 0x%llx/0x%llx 0x%llx/0x%llx, event_status 0x%llx; state +0x24 %u\n",
		 gart_inval, b->cl_fault[0], b->cl_fault[1], b->cl_fault[2], b->cl_fault[3], b->cl_event_status,
		 b->cl_state_24);
	dev_info(g->dev, "C1 (kick 0x%02x): ring 2 entry %s after %u us (read %u, write %d); chan state +0x1c %u +0x38 %u, ring ctl +0x30 %u +0x00 %u +0x10 %u\n",
		 cl_kick & 0xff, b->cl_ret ? "NOT consumed" : "consumed", b->cl_us, b->cl_rd, wr, b->cl_state_1c, b->cl_state_38,
		 b->cl_ring_30, READ_ONCE(*(u32 *)ctl), READ_ONCE(*(u32 *)(ctl + 0x10)));
	return b->cl_ret;
}

/*
 * M5 checkpoints C2/C3 (re/GPU-FIRST-JOB.md 3, 4, 7): one CDM command on the C1 channel.
 *   C2: command stream = the end record only.
 *   C3: the six records iOS emits for a CDM command (2 kick, 3, 4 WFI, 6, 7 finish, 8 end); CDM control
 *       stream = one STREAM_TERMINATE; terminate/context program fields 0 (iOS uses real ones, GPU-BRINGUP
 *       8.31: this tests whether the CDM needs them for an empty stream).
 *   C4: C3 plus a CDM context-state buffer (cmd+0x78) and the fence-terminate PDS program (cmd+0x70/+0x18),
 *       GPU-BRINGUP 8.35.
 * Command, streams, stamps and the +0x0c buffer live in the uncached shared heap; the stamp record is
 * {FW(stamp0), FW(stamp1), value, index}. Results: channel ring read index, stamp, events, live BIF faults.
 */
#define G4P_CDM_CMD_SIZE	0x2c0	/* iOS CDM pool slot size (capture 1) */
#define G4P_STAMP_VALUE		0x5a5a0001

static void w32a(void *p, u32 off, u32 v)
{
	*(u32 *)(p + off) = v;
}

/*
 * C5 (GPU-BRINGUP 8.39): a 1x1x1 compute kernel with no USC work. Its PDS program stores a word itself
 * (Mesa pvr_rogue_pds_encode.h: limm temp0, #value; st const64[0]; wdf; halt); the data segment holds the ST
 * descriptor (destination dword address << 2, count 1, source temp 0, write-through). CDM block: KERNEL0..5, 8
 * (Mesa cdm.xml, pvr_compute_generate_control_stream), then STREAM_TERMINATE.
 */
#define G4P_C5_OUT		0x800		/* in the PDS page: PDS ST addresses are 38 bits (8.39), below the FW heaps */
#define G4P_C5_VALUE		0x1234
#define G4P_PDS_C5_CODE		0x400
#define G4P_PDS_C5_DATA		0x500

static void g4p_c5_stream(struct g4p_boot *b, struct g4p_cljob *j)
{
	u32 po = b->pds.gpuva - G4P_PDS_VA;
	u64 dst = b->pds.gpuva + G4P_C5_OUT;
	u64 st = ((dst >> 2) & GENMASK_ULL(35, 0)) << 2 | 1ULL << 40;	/* count 1, src temp 0, write-through */
	u32 *code = b->pds.cpu + G4P_PDS_C5_CODE, *data = b->pds.cpu + G4P_PDS_C5_DATA;
	u32 *w = j->misc.cpu + 0x800;	/* the buffer at cmd+0x78 -> RGX 0x480: the stream the CDM runs (8.39) */

	code[0] = 0xd1800000 | ((u32)G4P_C5_VALUE << 2);	/* limm temp0, #value */
	code[1] = 0xd0800000;	/* st const64[0] */
	code[2] = 0xd1000000;	/* wdf */
	code[3] = 0xd3000000;	/* halt */
	data[0] = lower_32_bits(st);
	data[1] = upper_32_bits(st);
	data[2] = 0;
	data[3] = 0;
	w[0] = 1 << 8 | 1 << 2 | 1 << 1;	/* KERNEL0: compute kernel, pds temps 1, pds data 1 (x16 B), usc target any */
	w[1] = po + G4P_PDS_C5_DATA;		/* KERNEL1: data address, sd_type none */
	w[2] = po + G4P_PDS_C5_CODE;		/* KERNEL2: code address */
	w[3] = 0;				/* KERNEL3..5: 1 x 1 x 1 workgroups (count - 1) */
	w[4] = 0;
	w[5] = 0;
	w[6] = 0;				/* KERNEL8: max instances 32, workgroup size 1 x 1 x 1 */
	w[7] = 0x80000000;			/* STREAM_TERMINATE */
	wmb();
}

int g4p_boot_cl_job(struct g4p *g, unsigned int which)
{
	struct g4p_boot *b = g->boot;
	struct g4p_cljob *j;
	u32 entry[5] = { 2 };
	u32 fcmd, fchan, fa3, *w;
	void *ctl;
	unsigned int n = 0, end;
	int wr, cwr, ret;
	bool first;

	if (which < 2 || which > 5)
		return -EINVAL;
	if (!b || !b->alive || !g->powered)
		return -ENODEV;
	first = !b->cl_state.cpu;
	ret = g4p_cl_channel(b);	/* set up only: the first ring 2 entry carries the command */
	if (ret)
		return ret;
	if (b->job.cmd.cpu)
		return -EBUSY;	/* one job per boot for now */
	if (!g4p_gpu_running(g))
		return -EIO;
	j = &b->job;
	/*
	 * iOS keeps commands and command streams in the private heap (7) and passes the cached firmware form:
	 * the firmware invalidates the D-cache lines before reading them (fw 0x10028220); with the uncached
	 * form the first command read faulted on META (GPU-BRINGUP 8.33).
	 */
	ret = g4p_carve(b, 0, G4P_CDM_CMD_SIZE, &j->cmd);
	if (!ret)
		ret = g4p_carve(b, 0, SZ_4K, &j->stream);
	if (!ret)
		ret = g4p_carve(b, 1, SZ_4K, &j->cdm);
	if (!ret)
		ret = g4p_carve(b, 1, SZ_4K, &j->misc);
	if (ret)
		return ret;
	fcmd = g4p_fw_addr(j->cmd.gpuva, true);
	fchan = g4p_fw_addr(b->cl_state.gpuva, false);
	fa3 = g4p_fw_addr(g4p_fwinit_list_a_va(&b->fi) + 0x2400, true);	/* list A #3: CDM DM state */

	/* CDM control stream: STREAM_TERMINATE (block_type 2, Mesa cdm.xml) */
	w32a(j->cdm.cpu, 0, 0x80000000);
	/* the command (§3.1); stamps at misc +0 / +4, the +0x0c buffer at misc +0x100 */
	w32a(j->cmd.cpu, 0x000, 3);			/* CDM */
	w32a(j->cmd.cpu, 0x008, 0);			/* context 0 = the boot page catalogue */
	w32a(j->cmd.cpu, 0x00c, g4p_fw_addr(j->misc.gpuva + 0x100, false));
	*(u64 *)(j->cmd.cpu + 0x068) = j->cdm.gpuva;	/* CDM control stream base -> RGX 0x498 */
	/*
	 * 8.35 variants: 1 tells whether the CDM fetches from 0x498 at all (the fault address then moves to
	 * the unmapped VA), 2 gives the CDM a zeroed context state buffer (iOS: cmd+0x78 non-zero, 8.31).
	 */
	if (which >= 3 && c3_var == 1)
		*(u64 *)(j->cmd.cpu + 0x068) = 0xe1c2f00000ULL;	/* heap 6, never mapped */
	if ((which == 3 && c3_var == 2) || which >= 4)
		*(u64 *)(j->cmd.cpu + 0x078) = j->misc.gpuva + 0x800;	/* -> RGX 0x480 */
	if (which == 4 && c4_pattern == 1)
		for (unsigned int i = 0; i < 0x800 / 4; i++)	/* PDS offsets on unmapped pages of the PDS window */
			w32a(j->misc.cpu, 0x800 + 4 * i, 0x10000 + 0x10 * i);
	if (which == 4 && c4_pattern == 3)
		w32a(j->misc.cpu, 0x800, 0x80000000);	/* CDM STREAM_TERMINATE: an empty resume stream */
	if (which == 5)
		g4p_c5_stream(b, j);
	if (which == 4 && c4_pattern == 2) {
		u32 po = b->pds.gpuva - G4P_PDS_VA;

		/* 8.37: word 0 = PDS data address (PDSRW read), word 2 = code address (PDS fetch); word 1 = task word [G] */
		w32a(j->misc.cpu, 0x800, po + G4P_PDS_TERM_DATA);
		w32a(j->misc.cpu, 0x804, 0x20100003);
		w32a(j->misc.cpu, 0x808, po + G4P_PDS_TERM_CODE);
	}
	if (which >= 4) {
		/* CDM_TERMINATE_PDS (code bits 4..31, data bits 36..63, both << 4) and PDS1 exactly as iOS (8.31) */
		u64 o = b->pds.gpuva - G4P_PDS_VA;	/* PDS offset of the page */

		*(u64 *)(j->cmd.cpu + 0x070) = ((u64)(o + G4P_PDS_TERM_DATA) << 32) |
					       (o + G4P_PDS_TERM_CODE + (c4_prog == 2 ? SZ_4K : 0));
		w32a(j->cmd.cpu, 0x018, 0x20100003);	/* pds_seq_dep, common_shared, data size 1, fence */
		if (c4_ctx) {
			/* iOS: +0x224 = {code 0xdc0, data 0x14000}, +0x22c = 0x28800005 (8.36) */
			*(u64 *)(j->cmd.cpu + 0x224) = ((o + 0x400) << 32) | (o + 0x300);
			w32a(j->cmd.cpu, 0x22c, 0x28000003);	/* pds_seq_dep, target, data size 1, fence */
		}
	}
	w32a(j->cmd.cpu, 0x1e8, g4p_fw_addr(j->stream.gpuva, true));
	w32a(j->cmd.cpu, 0x260, g4p_fw_addr(j->misc.gpuva, false));
	w32a(j->cmd.cpu, 0x264, g4p_fw_addr(j->misc.gpuva + 4, false));
	w32a(j->cmd.cpu, 0x268, G4P_STAMP_VALUE);
	w32a(j->cmd.cpu, 0x26c, 0);			/* stamp index: event 1 bit 0 */

	/* the command stream (§4) */
	w = j->stream.cpu;
	if (which >= 3) {
		static const u32 hdr2 = 0x32, hdr3 = 0x80000018, hdr6 = 0x18, hdr7 = 0x33;

		/* 2: CDM kick */
		w[n++] = hdr2; w[n++] = fcmd + 0x18; w[n++] = fcmd + 0x68; w[n++] = fa3 + 0x720; w[n++] = fchan;
		w[n++] = 0; w[n++] = 0; w[n++] = 0; w[n++] = 0; w[n++] = fcmd + 0x1f0; w[n++] = 0; w[n++] = 0;
		w[n++] = 0;
		/* 3: timestamps */
		w[n++] = hdr3; w[n++] = fcmd + 0x280; w[n++] = fcmd + 0x288; w[n++] = fcmd + 0x288; w[n++] = 0;
		/* 4: compute wait-for-idle (encodeComputeWFI: 0x101, literal 0x…7134908, 1) */
		w[n++] = 0x101; w[n++] = 0x004a000a; w[n++] = 1; w[n++] = 1;
		/* 6: timestamps */
		w[n++] = hdr6; w[n++] = fcmd + 0x280; w[n++] = fcmd + 0x288; w[n++] = fcmd + 0x28c; w[n++] = 0;
		/* 7: finish (stamp); last word = stream offset of the end record */
		end = (n + 11) * 4;
		w[n++] = hdr7; w[n++] = fa3 + 0x720; w[n++] = fchan; w[n++] = 0; w[n++] = fcmd + 0x1f0; w[n++] = 0;
		w[n++] = 0; w[n++] = 0; w[n++] = g4p_fw_addr(j->misc.gpuva + 4, false); w[n++] = G4P_STAMP_VALUE; w[n++] = end;
	}
	w[n++] = 0x40000017;			/* 8: end */
	w32a(j->cmd.cpu, 0x1ec, ALIGN(n * 4, 64));
	j->stream_words = n;
	wmb();	/* command and streams before the channel ring names them */

	/* channel ring: ptr[wr] = FW(cmd), host write index ctl +0x40 */
	ctl = b->cl_ring.cpu;
	cwr = *(u32 *)(ctl + 0x40);
	*(u32 *)(ctl + 0x60 + 4 * cwr) = fcmd;
	wmb();	/* the pointer before the write index */
	cwr = (cwr + 1) % G4P_CHAN_RING_SIZE;
	*(u32 *)(ctl + 0x40) = cwr;
	if (gart_inval)
		g4p_fwinit_gart_inval(&b->fi);

	entry[1] = fchan;
	entry[2] = cwr;
	entry[3] = first;	/* 1 on the channel's first submission */
	j->irq0 = atomic_read(&g->irq_count);
	wr = g4p_fwinit_ring_push(&b->fi, 2, entry);
	if (wr < 0)
		return wr;
	writel(cl_kick & 0xff, g->rgx + RGX_CR_MTS_SCHEDULE);
	readl(g->rgx + RGX_CR_MTS_SCHEDULE);

	for (j->us = 0; j->us < (which >= 4 ? 1000000 : 200000); j->us += 20) {
		if (READ_ONCE(*(u32 *)j->misc.cpu) == G4P_STAMP_VALUE)
			break;
		if (which == 2 && READ_ONCE(*(u32 *)ctl) == (u32)cwr)
			break;
		udelay(20);
	}
	udelay(200);
	rmb();	/* the firmware writes before reading them */
	j->which = which;
	j->ring2_rd = g4p_fwinit_ring_read_index(&b->fi, 2);
	j->chan_rd = READ_ONCE(*(u32 *)ctl);
	j->chan_10 = READ_ONCE(*(u32 *)(ctl + 0x10));
	j->stamp0 = READ_ONCE(*(u32 *)j->misc.cpu);
	j->stamp1 = READ_ONCE(*(u32 *)(j->misc.cpu + 4));
	if (which == 5)
		dev_info(g->dev, "C5: word at GPU VA 0x%llx = 0x%08x (kernel stores 0x%x)\n",
			 (u64)b->pds.gpuva + G4P_C5_OUT, READ_ONCE(*(u32 *)(b->pds.cpu + G4P_C5_OUT)), G4P_C5_VALUE);
	j->irqs = atomic_read(&g->irq_count) - j->irq0;
	if (g4p_gpu_running(g)) {
		j->fault[0] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK0_MMU);
		j->fault[1] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK0_REQ);
		j->fault[2] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK1_MMU);
		j->fault[3] = readq(g->rgx + RGX_CR_BIF_FAULT_BANK1_REQ);
		/*
		 * CDM registers as the hardware holds them after the job (read only; GPU-BRINGUP 8.35): the C3
		 * fault is a CDM control stream read at VA 0 (DDK rgxdebug.c tag 8 / sub-tag 0).
		 */
		dev_info(g->dev, "C%u CDM: 0x478 %08x 0x480 %016llx 0x488 %08x 0x490 %08x 0x498 %016llx 0x4a0 %08x 0x4a8 %016llx 0x4b0 %08x 0x4b8 %016llx 0x4c0 %08x 0x4c8 %08x; stream base sent %016llx (var %u)\n",
			 which, readl(g->rgx + 0x478), readq(g->rgx + 0x480), readl(g->rgx + 0x488),
			 readl(g->rgx + 0x490), readq(g->rgx + 0x498), readl(g->rgx + 0x4a0), readq(g->rgx + 0x4a8),
			 readl(g->rgx + 0x4b0), readq(g->rgx + 0x4b8), readl(g->rgx + 0x4c0), readl(g->rgx + 0x4c8),
			 *(u64 *)(j->cmd.cpu + 0x068), c3_var);
		/* the registers the DDK's debug dump shows for a stuck job (rgxdebug.c, non-S7 infrastructure) */
		dev_info(g->dev, "C%u state: EVENT_STATUS 0x130 %08x MTS_GPU_INT_STATUS 0xbb0 %08x SIDEKICK_IDLE 0x3c8 %08x SLC_IDLE 0x3898 %08x SLC_STATUS0 0x3820 %08x MCU_FENCE 0x1740 %016llx PERF_COMPUTE_PHASE 0x6018 %08x PERF_COMPUTE_CYCLE 0x6030 %08x\n",
			 which, readl(g->rgx + 0x130), readl(g->rgx + 0xbb0), readl(g->rgx + 0x3c8),
			 readl(g->rgx + 0x3898), readl(g->rgx + 0x3820), readq(g->rgx + 0x1740),
			 readl(g->rgx + 0x6018), readl(g->rgx + 0x6030));
	}
	/* host view of what the scheduler reads (GPU-BRINGUP 8.33), with the firmware addresses */
	dev_info(g->dev, "C%u host view: state FW 0x%08x [+0]=0x%08x [+4]=0x%08x; ring ctl FW 0x%08x +0x00=%u +0x10=%u +0x20=%u +0x30=%u +0x40=%u; ptr[0] (FW 0x%08x) = 0x%08x; cmd FW 0x%08x [+0]=0x%08x\n",
		 which, fchan, *(u32 *)b->cl_state.cpu, *(u32 *)(b->cl_state.cpu + 4),
		 g4p_fw_addr(b->cl_ring.gpuva, false), *(u32 *)ctl, *(u32 *)(ctl + 0x10), *(u32 *)(ctl + 0x20),
		 *(u32 *)(ctl + 0x30), *(u32 *)(ctl + 0x40), g4p_fw_addr(b->cl_ring.gpuva + 0x60, false),
		 *(u32 *)(ctl + 0x60), fcmd, *(u32 *)j->cmd.cpu);
	dev_info(g->dev, "C%u: %u stream words; after %u us: ring 2 read %u (write %d), channel read %u (+0x10 %u, write %d), stamp 0x%08x/0x%08x, %d IRQs, BIF fault 0x%llx/0x%llx 0x%llx/0x%llx\n",
		 which, n, j->us, j->ring2_rd, wr, j->chan_rd, j->chan_10, cwr, j->stamp0, j->stamp1, j->irqs,
		 j->fault[0], j->fault[1], j->fault[2], j->fault[3]);
	return j->stamp0 == G4P_STAMP_VALUE || (which == 2 && j->chan_rd == (u32)cwr) ? 0 : -ETIMEDOUT;
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
	seq_printf(s, "  trace ring rd %u wr %u, statistics ring rd %u wr %u\n", g4p_fwinit_ring_ctl(&b->fi, 0, 0),
		   g4p_fwinit_ring_ctl(&b->fi, 0, 0x10), g4p_fwinit_ring_ctl(&b->fi, 1, 0),
		   g4p_fwinit_ring_ctl(&b->fi, 1, 0x10));
	if (b->cl_state.cpu)
		seq_printf(s, "  C1 live: BIF fault 0x%llx/0x%llx 0x%llx/0x%llx event_status 0x%llx state +0x24 %u\n",
			   b->cl_fault[0], b->cl_fault[1], b->cl_fault[2], b->cl_fault[3], b->cl_event_status,
			   b->cl_state_24);
	if (b->job.cmd.cpu)
		seq_printf(s, "  C%u: %u us, ring 2 read %u, channel read %u (+0x10 %u), stamp 0x%08x/0x%08x, %d IRQs, BIF fault 0x%llx/0x%llx 0x%llx/0x%llx\n",
			   b->job.which, b->job.us, b->job.ring2_rd, b->job.chan_rd, b->job.chan_10, b->job.stamp0,
			   b->job.stamp1, b->job.irqs, b->job.fault[0], b->job.fault[1], b->job.fault[2],
			   b->job.fault[3]);
	g4p_fwinit_show_events(s, &b->fi);
	g4p_fwinit_show_log(s, &b->fi);
	seq_printf(s, "  boot args: first [0x%08x]=0x%08x\n", ((u32 *)(b->image.cpu + G4P_BOOTARG_OFFSET))[0],
		   ((u32 *)(b->image.cpu + G4P_BOOTARG_OFFSET))[1]);
}
