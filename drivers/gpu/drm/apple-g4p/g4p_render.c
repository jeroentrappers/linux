// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 GPU (G4P): render submission (linux-on-iphone6 re/GPU-BRINGUP.md 9.09).
 *
 * User space builds the TA (0x580 bytes) and 3D (0x8c0 bytes) command bodies of one render pass (geometry state,
 * sizes, render target and its programs, all in its own address space); the kernel owns every firmware object
 * around them, as the 9.07 replay showed them to be enough:
 *   - two channels (TA, 3D) as the CL one: state {ring control, ring array, ..., +0x18 ~0} and a 0x144-entry ring;
 *   - per submission slot: the TA command, the 3D command, their scheduler nodes (type 0x23 / 0x25, 9.06), the
 *     type-4 wait record the 3D channel runs before its command;
 *   - the parameter buffer (8.89): 11 physically contiguous 128 KiB blocks, descriptors {next, 32 x PA >> 12};
 *   - the PM header with 36 per-pass entries (8.91, 9.00) and, in the submitting address space, the PM buffers and
 *     the free-page stack (they must be in the PB's memory context); installed with a type-5 TA ring record
 *     {5, slot, PM index, FW(header), FW(descriptor 1), FW(descriptor 0)} whenever the address space changes;
 *   - the stamps: the TA writes its value at TA +0x538 (the 3D channel's wait record waits for it), the 3D at
 *     3D +0x874; +0x534 / +0x870 name an uncached word each.
 * The kernel overwrites every firmware word of the bodies (TA +0x00c..+0x018, +0x45c, +0x464, +0x468, +0x51c,
 * +0x534..+0x558; 3D +0x00c..+0x01c, +0x850, +0x870..+0x88c, +0x8a0, +0x8ac); the rest is GPU state that only
 * reads and writes the submitter's address space. Synchronous: the ioctl returns once the 3D stamp is written.
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <drm/apple_g4p_drm.h>
#include "g4p.h"

#define G4P_RING_N		0x144
#define G4P_RSLOTS		2
#define G4P_PM_ENTRIES		36
#define G4P_PB_BLOCKS		11
#define G4P_PB_BLOCK		SZ_128K
#define G4P_PM_VA		0x33c000000ULL		/* = APPLE_G4P_USER_VA_END: kernel-owned top of each space */
#define G4P_PM_FSTACK		0x18000			/* free-page stack (0x300 blocks x 32 pages x 4 bytes) */
#define G4P_PM_BUF		(G4P_PM_VA + 0x100000)
#define G4P_PM_BUF_SIZE		(G4P_PM_ENTRIES * (0x80 + 0x10000) + 0x600)
#define G4P_PM_SIZE		(0x100000 + ALIGN(G4P_PM_BUF_SIZE, SZ_4K))

/* offsets in a submission slot (8 KiB, private heap) */
#define RS_TA			0x0000
#define RS_TD			0x0600
#define RS_TAN			0x0f00
#define RS_TDN			0x1000
#define RS_WAIT			0x1140

struct g4p_render {
	struct g4p_bo st[2], ring[2];	/* [0] TA, [1] 3D: state (private heap), ring control + array (shared) */
	struct g4p_bo slot[G4P_RSLOTS];
	struct g4p_bo pmh;		/* PM header + 36 entries (private) */
	struct g4p_bo stamp;		/* +0 TA, +4 3D (private, cached form) */
	struct g4p_bo shared;		/* +0x000 uncached stamp words, +0x100 TA/3D +0x00c object, +0x200 3D +0x8ac,
					 * +0x800 the type-5 record (shared heap) */
	struct g4p_bo pbd;		/* PB descriptors: buffer header at +0, descriptors from +0x80 (shared) */
	void *pb_cpu[G4P_PB_BLOCKS];
	dma_addr_t pb_dma[G4P_PB_BLOCKS];
	u32 seq;
	u32 wr[2];			/* channel ring write indices */
	unsigned int pm_vm;		/* address space the PB/PM set is installed for */
	struct {
		void *cpu;
		dma_addr_t dma;
		bool mapped;
	} pm[G4P_NR_VM];
};

static void w32(void *p, unsigned int off, u32 v)
{
	*(u32 *)(p + off) = v;
}

static void w64(void *p, unsigned int off, u64 v)
{
	*(u64 *)(p + off) = v;
}

void g4p_render_free(struct g4p *g)
{
	struct g4p_render *r = g->render;

	if (!r)
		return;
	for (unsigned int i = 0; i < G4P_PB_BLOCKS; i++)
		if (r->pb_cpu[i])
			dma_free_coherent(g->dev, G4P_PB_BLOCK, r->pb_cpu[i], r->pb_dma[i]);
	for (unsigned int i = 0; i < G4P_NR_VM; i++)
		if (r->pm[i].cpu)
			dma_free_coherent(g->dev, G4P_PM_SIZE, r->pm[i].cpu, r->pm[i].dma);
	kfree(r);	/* the carved objects go with the boot's arenas */
	g->render = NULL;
}

static int g4p_render_init(struct g4p *g)
{
	struct g4p_render *r;
	int ret = 0;

	if (g->render)
		return 0;
	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;
	g->render = r;
	for (unsigned int c = 0; c < 2 && !ret; c++) {
		ret = g4p_boot_carve(g, 0, 0x80, &r->st[c]);
		if (!ret)
			ret = g4p_boot_carve(g, 1, 0x60 + 4 * G4P_RING_N, &r->ring[c]);
	}
	for (unsigned int i = 0; i < G4P_RSLOTS && !ret; i++)
		ret = g4p_boot_carve(g, 0, SZ_8K, &r->slot[i]);
	if (!ret)
		ret = g4p_boot_carve(g, 0, 0x80 + 0xc0 * G4P_PM_ENTRIES, &r->pmh);
	if (!ret)
		ret = g4p_boot_carve(g, 0, SZ_4K, &r->stamp);
	if (!ret)
		ret = g4p_boot_carve(g, 1, SZ_4K, &r->shared);
	if (!ret)
		ret = g4p_boot_carve(g, 1, SZ_4K, &r->pbd);
	for (unsigned int i = 0; i < G4P_PB_BLOCKS && !ret; i++) {
		r->pb_cpu[i] = dma_alloc_coherent(g->dev, G4P_PB_BLOCK, &r->pb_dma[i], GFP_KERNEL);
		if (!r->pb_cpu[i])
			ret = -ENOMEM;
	}
	if (ret) {
		g4p_render_free(g);
		return ret;
	}
	/* channels as the CL one (9.07): ring control indices 0, size; state +0 control, +4 array, +0x18 ~0 */
	for (unsigned int c = 0; c < 2; c++) {
		w32(r->ring[c].cpu, 0x50, G4P_RING_N);
		w32(r->st[c].cpu, 0x00, g4p_fw_addr(r->ring[c].gpuva, false));
		w32(r->st[c].cpu, 0x04, g4p_fw_addr(r->ring[c].gpuva + 0x60, false));
		w32(r->st[c].cpu, 0x18, 0xffffffff);
	}
	/* PB descriptors (8.89): buffer header +0x60 = 1; block 0 alone, blocks 1.. chained */
	w32(r->pbd.cpu, 0x60, 1);
	for (unsigned int i = 0; i < G4P_PB_BLOCKS; i++) {
		void *d = r->pbd.cpu + 0x80 + 0x84 * i;
		u64 dva = r->pbd.gpuva + 0x80 + 0x84 * i;

		w32(d, 0, (i == 0 || i == G4P_PB_BLOCKS - 1) ? 0 : g4p_fw_addr(dva + 0x84, false));
		for (unsigned int p = 0; p < G4P_PB_BLOCK / SZ_4K; p++)
			w32(d, 4 + 4 * p, (r->pb_dma[i] + (u64)p * SZ_4K) >> 12);
	}
	wmb();
	return 0;
}

/* the PM header and its 36 per-pass entries for the PM memory at G4P_PM_VA (kext reset, 8.91 / 9.00) */
static void g4p_render_pm_header(struct g4p_render *r)
{
	void *h = r->pmh.cpu;
	u32 fdesc = g4p_fw_addr(r->pbd.gpuva, false);

	memset(h, 0, r->pmh.size);
	w32(h, 0x0c, 0x7f);
	w64(h, 0x18, G4P_PM_VA);
	w32(h, 0x20, G4P_PM_FSTACK);
	w32(h, 0x28, 0xffffffff);
	w32(h, 0x38, 0x1000);
	w32(h, 0x44, fdesc);
	w32(h, 0x58, 1);
	w32(h, 0x5c, 0x300 << 5);
	w32(h, 0x60, 0x100 << 5);
	for (unsigned int i = 0; i < G4P_PM_ENTRIES; i++) {
		void *e = h + 0x80 + 0xc0 * i;

		w64(e, 0xa8, G4P_PM_BUF + 0x80 * i);
		w64(e, 0xa0, G4P_PM_BUF + G4P_PM_ENTRIES * 0x80 + 0x10000ULL * i);
		w64(e, 0xb0, G4P_PM_BUF + G4P_PM_ENTRIES * 0x80 + 0x10000ULL * G4P_PM_ENTRIES);
		w32(e, 0xb8, g4p_fw_addr(r->pbd.gpuva + 0x40, false));
	}
}

/* PM memory of address space @ctx, mapped at G4P_PM_VA in it (once per boot) */
static int g4p_render_pm_map(struct g4p *g, struct g4p_render *r, unsigned int ctx)
{
	int ret;

	if (!r->pm[ctx].cpu) {
		r->pm[ctx].cpu = dma_alloc_coherent(g->dev, G4P_PM_SIZE, &r->pm[ctx].dma, GFP_KERNEL);
		if (!r->pm[ctx].cpu)
			return -ENOMEM;
	}
	if (r->pm[ctx].mapped)
		return 0;
	memset(r->pm[ctx].cpu, 0, G4P_PM_SIZE);
	ret = g4p_boot_map_range(g, ctx, G4P_PM_VA, r->pm[ctx].dma, G4P_PM_SIZE);
	if (!ret)
		r->pm[ctx].mapped = true;
	return ret;
}

static void g4p_render_ring_put(struct g4p_render *r, unsigned int c, u32 fw)
{
	w32(r->ring[c].cpu, 0x60 + 4 * r->wr[c], fw);
	r->wr[c] = (r->wr[c] + 1) % G4P_RING_N;
}

/* scheduler nodes (9.06; g4prender.gen_tanode / gen_3dnode) */
static void g4p_render_nodes(struct g4p *g, struct g4p_render *r, void *s, u64 sva, u32 seq, u32 ta_id, u32 td_id,
			     u32 fentry, u32 fhdr, unsigned int hw)
{
	u32 ta = g4p_fw_addr(sva + RS_TA, true), td = g4p_fw_addr(sva + RS_TD, true);
	u32 la_ta = g4p_boot_list_a_fw(g, 0x1864), la_3d = g4p_boot_list_a_fw(g, 0x2224);
	u32 sta = g4p_fw_addr(r->st[0].gpuva, true), std = g4p_fw_addr(r->st[1].gpuva, true);
	u32 fs_ta = g4p_fw_addr(r->stamp.gpuva, true), fs_td = g4p_fw_addr(r->stamp.gpuva + 4, true);
	void *n = s + RS_TAN;
	static const u16 tc[][2] = {	/* constant words of the TA node */
		{ 0x58, 0x801 }, { 0x5c, 0x18 }, { 0x70, 0x24 },
	};
	static const u32 dc[][2] = {	/* constant words of the 3D node */
		{ 0x60, 0x80000018 }, { 0x74, 0x201 }, { 0x78, 0x138204 }, { 0x7c, 0x10 },
		{ 0x80, 0xf }, { 0x88, 0x14 }, { 0x8c, 0x109f }, { 0x90, 0x104000a }, { 0x94, 0x1f }, { 0x98, 0x1f },
		{ 0x9c, 0xf48008 }, { 0xa0, 0x10000015 }, { 0xa4, 4 }, { 0xac, 0x14 }, { 0xb0, 0x109f },
		{ 0xb4, 0x13000a }, { 0xb8, 0x4000 }, { 0xbc, 0x4000 }, { 0xc0, 0x138204 }, { 0xc4, 0x4000 },
		{ 0xc8, 0x104000a }, { 0xcc, 8 }, { 0xd0, 8 }, { 0xd4, 0x18 }, { 0xe8, 0x26 }, { 0x100, 1 },
		{ 0x12c, 0xffffff18 }, { 0x130, 0x40000017 },
	};

	memset(n, 0, 0x100);
	for (unsigned int i = 0; i < ARRAY_SIZE(tc); i++)
		w32(n, tc[i][0], tc[i][1]);
	/*
	 * 9.12: iOS's "2" and "3" in the nodes are the address space's memory context slot and the PM context
	 * index (TA +0x010, the type-5 record): both the submitter's hardware slot here
	 */
	w32(n, 0x18, hw);
	w32(n, 0x20, hw);
	w32(n, 0x84, hw);
	w32(n, 0x44, 0x80000018);
	w32(n, 0x00, 0x23);
	w32(n, 0x04, ta + 0x3ac);
	w32(n, 0x08, ta + 0x28);
	w32(n, 0x0c, fentry);
	w32(n, 0x10, la_ta);
	w32(n, 0x14, sta);
	w32(n, 0x1c, seq);
	w32(n, 0x28, seq - 1);
	w32(n, 0x30, ta + 0x46c);
	w32(n, 0x40, ta_id);
	w32(n, 0x48, ta + 0x558);
	w32(n, 0x4c, ta + 0x560);
	w32(n, 0x50, ta + 0x560);
	w32(n, 0x54, ta_id);
	w32(n, 0x60, ta + 0x558);
	w32(n, 0x64, ta + 0x560);
	w32(n, 0x68, ta + 0x564);
	w32(n, 0x6c, ta_id);
	w32(n, 0x74, fentry);
	w32(n, 0x78, fhdr);
	w32(n, 0x7c, la_ta);
	w32(n, 0x80, sta);
	w32(n, 0x8c, ta + 0x46c);
	w32(n, 0x98, ta_id);
	w32(n, 0x9c, fs_ta);
	w32(n, 0xa0, seq);
	w32(n, 0xa4, 0xffffff90);
	w32(n, 0xa8, 0x40000017);

	n = s + RS_TDN;
	memset(n, 0, 0x140);
	for (unsigned int i = 0; i < ARRAY_SIZE(dc); i++)
		w32(n, dc[i][0], dc[i][1]);
	w32(n, 0x28, hw);
	w32(n, 0x30, hw);
	w32(n, 0x118, hw);
	w32(n, 0x00, 0x25);
	w32(n, 0x04, td + 0x3a8);
	w32(n, 0x08, td + 0x60);
	w32(n, 0x0c, fentry);
	w32(n, 0x10, la_3d);
	w32(n, 0x14, td + 0x7d8);
	w32(n, 0x18, td + 0x7dc);
	w32(n, 0x1c, td + 0x860);
	w32(n, 0x20, std);
	w32(n, 0x24, td);
	w32(n, 0x2c, seq);
	w32(n, 0x38, seq - 1);
	w32(n, 0x40, td + 0x6c0);
	w32(n, 0x44, td + 0x894);
	w32(n, 0x5c, td_id);
	w32(n, 0x64, td + 0x8a0);
	w32(n, 0x68, td + 0x8a8);
	w32(n, 0x6c, td + 0x8a8);
	w32(n, 0x70, td_id);
	w32(n, 0x84, la_3d);
	w32(n, 0xd8, td + 0x8a0);
	w32(n, 0xdc, td + 0x8a8);
	w32(n, 0xe0, td + 0x8ac);
	w32(n, 0xe4, td_id);
	w32(n, 0xec, td_id);
	w32(n, 0xf0, fs_td);
	w32(n, 0xf4, seq);
	w32(n, 0xf8, fentry);
	w32(n, 0xfc, fhdr);
	w32(n, 0x104, la_3d);
	w32(n, 0x108, td + 0x860);
	w32(n, 0x10c, td + 0x7d8);
	w32(n, 0x110, std);
	w32(n, 0x114, td);
	w32(n, 0x11c, td + 0x6c0);
	/* 3D node +0x120: 1 with an occlusion query (3D +0x078 set) */
	w32(n, 0x120, *(u64 *)(s + RS_TD + 0x78) ? 1 : 0);
}

/*
 * One render pass of address space @ctx (bound to a hardware slot): @ta / @td are the user's command bodies.
 * Called with g->lock held, the firmware booted and nothing else in flight.
 */
int g4p_render_submit(struct g4p *g, unsigned int ctx, const void *ta, const void *td, u32 *status, u32 *elapsed_us)
{
	struct g4p_render *r;
	unsigned int hw = g4p_boot_vm_slot(g, ctx), us;
	u32 e[5], seq, ta_id, td_id, fentry, fhdr, obj, fs_ta, fs_td;
	volatile u32 *st;
	void *s;
	u64 sva;
	bool install;
	int ret;

	if (!ctx || !hw)
		return -EINVAL;
	ret = g4p_render_init(g);
	if (ret)
		return ret;
	r = g->render;
	ret = g4p_render_pm_map(g, r, ctx);
	if (ret)
		return ret;
	install = r->pm_vm != ctx;
	if (install) {
		g4p_render_pm_header(r);
		r->pm_vm = ctx;
	}
	seq = ++r->seq;
	ta_id = 0x03000000 + 2 * seq + 1;
	td_id = ta_id - 1;
	s = r->slot[seq % G4P_RSLOTS].cpu;
	sva = r->slot[seq % G4P_RSLOTS].gpuva;
	fhdr = g4p_fw_addr(r->pmh.gpuva, true);
	fentry = g4p_fw_addr(r->pmh.gpuva + 0x80 + 0xc0 * (seq % G4P_PM_ENTRIES), true);
	obj = g4p_fw_addr(r->shared.gpuva + 0x100, false);
	fs_ta = g4p_fw_addr(r->stamp.gpuva, true);
	fs_td = g4p_fw_addr(r->stamp.gpuva + 4, true);

	memcpy(s + RS_TA, ta, APPLE_G4P_TA_CMD_SIZE);
	memcpy(s + RS_TD, td, APPLE_G4P_3D_CMD_SIZE);
	/* TA command: firmware words (9.01) */
	w32(s + RS_TA, 0x00c, obj);
	w32(s + RS_TA, 0x010, hw);	/* PM context index */
	w32(s + RS_TA, 0x014, fhdr);
	w32(s + RS_TA, 0x018, fentry);
	w32(s + RS_TA, 0x45c, g4p_fw_addr(sva + RS_TAN, true));
	w32(s + RS_TA, 0x464, g4p_fw_addr(r->st[1].gpuva, true));
	w32(s + RS_TA, 0x468, 2 * seq + 1);
	w32(s + RS_TA, 0x51c, 0x02000000 + seq);
	w32(s + RS_TA, 0x534, g4p_fw_addr(r->shared.gpuva, false));
	w32(s + RS_TA, 0x538, fs_ta);
	w32(s + RS_TA, 0x53c, seq);
	w32(s + RS_TA, 0x54c, ta_id);
	w32(s + RS_TA, 0x550, seq - 1);
	/* 3D command */
	w32(s + RS_TD, 0x00c, g4p_fw_addr(sva + RS_TDN, true));
	w32(s + RS_TD, 0x014, obj);
	w32(s + RS_TD, 0x018, fhdr);
	w32(s + RS_TD, 0x01c, fentry);
	w32(s + RS_TD, 0x850, 0x02000000 + seq);
	w32(s + RS_TD, 0x870, g4p_fw_addr(r->shared.gpuva + 4, false));
	w32(s + RS_TD, 0x874, fs_td);
	w32(s + RS_TD, 0x878, seq);
	w32(s + RS_TD, 0x888, td_id);
	w32(s + RS_TD, 0x88c, seq - 1);
	w32(s + RS_TD, 0x8ac, g4p_fw_addr(r->shared.gpuva + 0x200, false));
	g4p_render_nodes(g, r, s, sva, seq, ta_id, td_id, fentry, fhdr, hw);
	/* the 3D channel's wait for the TA stamp, then its command */
	w32(s, RS_WAIT + 0, 4);
	w32(s, RS_WAIT + 4, fs_ta);
	w32(s, RS_WAIT + 8, seq);
	w32(s, RS_WAIT + 12, td_id);
	st = r->stamp.cpu;
	st[0] = 0;
	st[1] = 0;
	if (install) {	/* {5, slot, PM index, FW(header), FW(descriptor 1), FW(descriptor 0)} */
		u32 *rec = r->shared.cpu + 0x800;

		rec[0] = 5;
		rec[1] = hw;
		rec[2] = hw;
		rec[3] = fhdr;
		rec[4] = g4p_fw_addr(r->pbd.gpuva + 0x80 + 0x84, false);
		rec[5] = g4p_fw_addr(r->pbd.gpuva + 0x80, false);
		g4p_render_ring_put(r, 0, g4p_fw_addr(r->shared.gpuva + 0x800, false));
	}
	g4p_render_ring_put(r, 0, g4p_fw_addr(sva + RS_TA, true));
	g4p_render_ring_put(r, 1, g4p_fw_addr(sva + RS_WAIT, true));
	g4p_render_ring_put(r, 1, g4p_fw_addr(sva + RS_TD, true));
	w32(r->ring[0].cpu, 0x40, r->wr[0]);
	w32(r->ring[1].cpu, 0x40, r->wr[1]);
	wmb();	/* everything before the device ring entries */

	e[0] = 0;
	e[1] = g4p_fw_addr(r->st[0].gpuva, true);
	e[2] = r->wr[0];
	e[3] = 1;
	e[4] = 0;
	ret = g4p_boot_ring_kick(g, 0, e, 0, hw);
	if (!ret) {
		e[0] = 1;
		e[1] = g4p_fw_addr(r->st[1].gpuva, true);
		e[2] = r->wr[1];
		ret = g4p_boot_ring_kick(g, 1, e, 0x10, hw);
	}
	if (ret)
		return ret;
	for (us = 0; us < 2000000; us += 50) {
		if (!(us % 1000))
			g4p_boot_ack_rings(g);	/* host-bound trace / statistics rings (8.89) */
		if (READ_ONCE(st[1]) == seq)
			break;
		udelay(50);
	}
	rmb();
	*elapsed_us = us;
	*status = READ_ONCE(st[1]) == seq ? 0 : READ_ONCE(st[0]) == seq ? 2 : 1;
	dev_info(g->dev, "render %u (ctx %u slot %u%s): TA stamp %u, 3D stamp %u after %u us\n", seq, ctx, hw,
		 install ? ", PB installed" : "", READ_ONCE(st[0]), READ_ONCE(st[1]), us);
	if (*status) {	/* where the firmware is, and how far it read the channel rings */
		u32 v[4] = { 0 };

		for (unsigned int t = 0; t < 2; t++) {
			g4p_boot_meta_core_read(g, t, 5, 0, &v[2 * t]);
			g4p_boot_meta_core_read(g, t, 5, 1, &v[2 * t + 1]);
		}
		dev_info(g->dev, "render: META T0 PC %08x PCX %08x, T1 PC %08x PCX %08x; TA ring %u %u %u %u / %u, 3D ring %u %u %u %u / %u; MMU fault ctx %d\n",
			 v[0], v[1], v[2], v[3], READ_ONCE(*(u32 *)r->ring[0].cpu), READ_ONCE(*(u32 *)(r->ring[0].cpu + 0x10)),
			 READ_ONCE(*(u32 *)(r->ring[0].cpu + 0x20)), READ_ONCE(*(u32 *)(r->ring[0].cpu + 0x30)), r->wr[0],
			 READ_ONCE(*(u32 *)r->ring[1].cpu), READ_ONCE(*(u32 *)(r->ring[1].cpu + 0x10)),
			 READ_ONCE(*(u32 *)(r->ring[1].cpu + 0x20)), READ_ONCE(*(u32 *)(r->ring[1].cpu + 0x30)), r->wr[1],
			 g4p_boot_mmu_fault(g));
	}
	return 0;
}
