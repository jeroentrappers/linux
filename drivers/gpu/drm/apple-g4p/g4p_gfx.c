// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 GPU (G4P): C9, a data-driven graphics harness (linux-on-iphone6 re/GPU-BRINGUP.md 8.88).
 *
 * debugfs apple-g4p/gfx runs /lib/firmware/agx-g4p-gfx.bin against the running firmware boot; gfxout returns what
 * its READ records collected. The image (tools/gpuios/g4pgfx.py) is a header {magic "G4PG", version 1, 0, 0} and
 * records {u32 type, u32 len, payload[len] padded to 4}:
 *   1 FWPAGES  {u64 gpuva, bytes}: pages in the firmware heaps (kernel context) at gpuva, with that content
 *   2 USERPAGES {u64 gpuva, bytes}: pages in the C9 address space at gpuva (any VA below 64 GiB)
 *   3 CTX      {u32 slot}: load the C9 address space into memory context slot (1..7)
 *   4 PUSH     {u32 ring, u32 kick, u32 entry[5]}: a device ring entry (ring 0 TA, 1 3D, 2 CL), then
 *              MTS_SCHEDULE = kick if non-zero
 *   5 WAIT     {u32 fwaddr, u32 mask, u32 value, u32 timeout_ms}: until (word & mask) == value
 *   6 READ     {u32 where (0 C9's firmware heap pages, 1 C9 space, 2 the driver's own lists A/B), u32 len,
 *              u64 gpuva}: appended to gfxout
 *   7 SLEEP    {u32 ms}
 *   8 PBALLOC  {u32 nblocks, u32 0, u64 desc_va}: the parameter buffer, as iOS's AGXParameterManagementPhysical
 *              does it (8.89): nblocks physically contiguous 128 KiB blocks; descriptors {u32 next FW address,
 *              u32 page[32] = PA >> 12}, 0x84 apart at desc_va (pages FWPAGES put into the shared heap): block 0
 *              alone, blocks 1.. chained; then devctl 12 {12, FW(desc 1), FW(desc 0), 1, 0}
 * Only for bring-up (module parameter experimental), only pages that C9 mapped itself are written or read, and the
 * firmware heap pages must not collide with the driver's own (module parameter ios_layout moves those away).
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include "g4p.h"

#define G4P_GFX_MAGIC	0x47503447	/* "G4PG" */
#define G4P_GFX_GIBS	64		/* C9 address space: VA < 64 GiB */
#define G4P_GFX_OUT_MAX	SZ_8M

struct g4p_gfx_page {
	struct list_head node;
	u64 va;
	bool user;
	void *cpu;
	dma_addr_t dma;
};

struct g4p_gfx_gib {
	u64 *pd;
	dma_addr_t pd_dma;
	u64 *pt[512];
	dma_addr_t pt_dma[512];
};

#define G4P_GFX_PB_BLOCK	SZ_128K
#define G4P_GFX_PB_MAX		64

struct g4p_gfx {
	struct list_head pages;
	void *pb_cpu[G4P_GFX_PB_MAX];
	dma_addr_t pb_dma[G4P_GFX_PB_MAX];
	unsigned int pb_n;
	u32 *pc;
	dma_addr_t pc_dma;
	struct g4p_gfx_gib *gib[G4P_GFX_GIBS];
	unsigned int slot;
	void *out;
	size_t out_len;
};

static struct g4p_gfx_page *g4p_gfx_find(struct g4p_gfx *x, u64 va, bool user)
{
	struct g4p_gfx_page *p;

	list_for_each_entry(p, &x->pages, node)
		if (p->va == (va & ~0xfffULL) && p->user == user)
			return p;
	return NULL;
}

/* the page at va in the C9 address space, mapped on first use */
static int g4p_gfx_user_map(struct g4p *g, struct g4p_gfx *x, struct g4p_gfx_page *p)
{
	unsigned int gi = p->va >> 30, d = (p->va >> 21) & 0x1ff, t = (p->va >> 12) & 0x1ff;
	struct g4p_gfx_gib *b;

	if (gi >= G4P_GFX_GIBS)
		return -EINVAL;
	if (!x->pc) {
		x->pc = dma_alloc_coherent(g->dev, SZ_4K, &x->pc_dma, GFP_KERNEL);
		if (!x->pc)
			return -ENOMEM;
	}
	b = x->gib[gi];
	if (!b) {
		b = kzalloc(sizeof(*b), GFP_KERNEL);
		if (!b)
			return -ENOMEM;
		b->pd = dma_alloc_coherent(g->dev, SZ_4K, &b->pd_dma, GFP_KERNEL);
		if (!b->pd) {
			kfree(b);
			return -ENOMEM;
		}
		x->gib[gi] = b;
		x->pc[gi] = ((b->pd_dma >> 12) << 4) | 1;
	}
	if (!b->pt[d]) {
		b->pt[d] = dma_alloc_coherent(g->dev, SZ_4K, &b->pt_dma[d], GFP_KERNEL);
		if (!b->pt[d])
			return -ENOMEM;
		b->pd[d] = (b->pt_dma[d] & GENMASK_ULL(39, 12)) | 1;
	}
	b->pt[d][t] = (p->dma & GENMASK_ULL(39, 12)) | (g4p_boot_heap_pte() & (G4P_PTE_CC | G4P_PTE_SLC_BYPASS)) |
		      G4P_PTE_VALID;
	return 0;
}

static int g4p_gfx_page(struct g4p *g, struct g4p_gfx *x, u64 va, bool user, const void *src, size_t n)
{
	struct g4p_gfx_page *p = g4p_gfx_find(x, va, user);
	int ret;

	if (!p) {
		p = kzalloc(sizeof(*p), GFP_KERNEL);
		if (!p)
			return -ENOMEM;
		p->va = va & ~0xfffULL;
		p->user = user;
		p->cpu = dma_alloc_coherent(g->dev, SZ_4K, &p->dma, GFP_KERNEL);
		if (!p->cpu) {
			kfree(p);
			return -ENOMEM;
		}
		ret = user ? g4p_gfx_user_map(g, x, p) : g4p_boot_fw_map_page(g, p->va, p->dma);
		if (ret) {
			dma_free_coherent(g->dev, SZ_4K, p->cpu, p->dma);
			kfree(p);
			return ret;
		}
		list_add_tail(&p->node, &x->pages);
	}
	memcpy(p->cpu + (va & 0xfff), src, n);
	return 0;
}

static int g4p_gfx_pages(struct g4p *g, struct g4p_gfx *x, u64 va, bool user, const u8 *d, size_t n)
{
	int ret = 0;

	if (va & 0xfff)
		return -EINVAL;
	for (size_t o = 0; o < n && !ret; o += SZ_4K)
		ret = g4p_gfx_page(g, x, va + o, user, d + o, min_t(size_t, SZ_4K, n - o));
	return ret;
}

static void *g4p_gfx_ptr(struct g4p_gfx *x, u64 va, bool user)
{
	struct g4p_gfx_page *p = g4p_gfx_find(x, va, user);

	return p ? p->cpu + (va & 0xfff) : NULL;
}

void g4p_gfx_free(struct g4p *g)
{
	struct g4p_gfx *x = g->gfx;
	struct g4p_gfx_page *p, *t;

	if (!x)
		return;
	list_for_each_entry_safe(p, t, &x->pages, node) {
		dma_free_coherent(g->dev, SZ_4K, p->cpu, p->dma);
		kfree(p);
	}
	for (unsigned int i = 0; i < G4P_GFX_GIBS; i++) {
		struct g4p_gfx_gib *b = x->gib[i];

		if (!b)
			continue;
		for (unsigned int d = 0; d < 512; d++)
			if (b->pt[d])
				dma_free_coherent(g->dev, SZ_4K, b->pt[d], b->pt_dma[d]);
		dma_free_coherent(g->dev, SZ_4K, b->pd, b->pd_dma);
		kfree(b);
	}
	if (x->pc)
		dma_free_coherent(g->dev, SZ_4K, x->pc, x->pc_dma);
	for (unsigned int i = 0; i < x->pb_n; i++)
		dma_free_coherent(g->dev, G4P_GFX_PB_BLOCK, x->pb_cpu[i], x->pb_dma[i]);
	kvfree(x->out);
	kfree(x);
	g->gfx = NULL;
}

static int g4p_gfx_read(struct g4p *g, struct g4p_gfx *x, u32 where, u64 va, u32 len)
{
	bool user = where == 1;
	void *n;

	if (x->out_len + 16 + len > G4P_GFX_OUT_MAX)
		return -ENOSPC;
	n = kvrealloc(x->out, x->out_len + 16 + ALIGN(len, 4), GFP_KERNEL);
	if (!n)
		return -ENOMEM;
	x->out = n;
	*(u64 *)(x->out + x->out_len) = va;
	*(u32 *)(x->out + x->out_len + 8) = user;
	*(u32 *)(x->out + x->out_len + 12) = len;
	x->out_len += 16;
	for (u32 o = 0; o < len; o += 4) {
		u32 *s = where == 3 ? g4p_boot_image_ptr(g, va + o) :
			 where == 2 ? g4p_boot_drv_ptr(g, va + o) : g4p_gfx_ptr(x, va + o, user);

		*(u32 *)(x->out + x->out_len + o) = s ? READ_ONCE(*s) : 0xeeeeeeee;
	}
	x->out_len += ALIGN(len, 4);
	return 0;
}

static int g4p_gfx_pballoc(struct g4p *g, struct g4p_gfx *x, unsigned int n, u64 desc_va)
{
	u32 e[5];

	if (n < 2 || x->pb_n + n > G4P_GFX_PB_MAX)
		return -EINVAL;
	for (unsigned int i = 0; i < n; i++) {
		unsigned int k = x->pb_n + i;

		x->pb_cpu[k] = dma_alloc_coherent(g->dev, G4P_GFX_PB_BLOCK, &x->pb_dma[k], GFP_KERNEL);
		if (!x->pb_cpu[k])
			return -ENOMEM;
	}
	for (unsigned int i = 0; i < n; i++) {
		u64 d = desc_va + 0x84ULL * i;
		dma_addr_t pa = x->pb_dma[x->pb_n + i];
		u32 next = (i == 0 || i == n - 1) ? 0 : g4p_fw_addr(d + 0x84, false);
		u32 *w = g4p_gfx_ptr(x, d, false);

		if (!w || !g4p_gfx_ptr(x, d + 0x80, false))
			return -EINVAL;	/* the descriptor pages must be C9's (FWPAGES) */
		*w = next;
		for (unsigned int p = 0; p < G4P_GFX_PB_BLOCK / SZ_4K; p++)
			*(u32 *)g4p_gfx_ptr(x, d + 4 + 4 * p, false) = (pa + (u64)p * SZ_4K) >> 12;
	}
	x->pb_n += n;
	wmb();	/* descriptors before the firmware is told about them */
	e[0] = 12;
	e[1] = g4p_fw_addr(desc_va + 0x84, false);
	e[2] = g4p_fw_addr(desc_va, false);
	e[3] = 1;
	e[4] = 0;
	dev_info(g->dev, "C9 PB: %u blocks of 128 KiB (first PA %pad), devctl 12 {%08x %08x 1 0}\n", n,
		 &x->pb_dma[x->pb_n - n], e[1], e[2]);
	return g4p_boot_devctl5(g, e);
}

int g4p_gfx_run(struct g4p *g)
{
	const struct firmware *fw;
	const u8 *d, *end;
	struct g4p_gfx *x;
	unsigned int nrec = 0;
	u32 hdr[4];
	int ret;

	if (!g4p_boot_alive(g))
		return -ENODEV;
	ret = request_firmware(&fw, "agx-g4p-gfx.bin", g->dev);
	if (ret)
		return ret;
	if (!g->gfx) {
		g->gfx = kzalloc(sizeof(*g->gfx), GFP_KERNEL);
		if (!g->gfx) {
			ret = -ENOMEM;
			goto out;
		}
		INIT_LIST_HEAD(&g->gfx->pages);
	}
	x = g->gfx;
	x->out_len = 0;
	ret = -EINVAL;
	if (fw->size < sizeof(hdr))
		goto out;
	memcpy(hdr, fw->data, sizeof(hdr));
	if (hdr[0] != G4P_GFX_MAGIC || hdr[1] != 1)
		goto out;
	d = fw->data + sizeof(hdr);
	end = fw->data + fw->size;
	ret = 0;
	while (d < end && !ret) {
		u32 type, len;
		const u8 *p;

		if (end - d < 8) {
			ret = -EINVAL;
			break;
		}
		type = get_unaligned_le32(d);
		len = get_unaligned_le32(d + 4);
		p = d + 8;
		if (len > end - p) {
			ret = -EINVAL;
			break;
		}
		d = p + ALIGN(len, 4);
		nrec++;
		switch (type) {
		case 1:	/* FWPAGES */
		case 2:	/* USERPAGES */
			if (len < 8) {
				ret = -EINVAL;
				break;
			}
			ret = g4p_gfx_pages(g, x, get_unaligned_le64(p), type == 2, p + 8, len - 8);
			if (!ret && type == 1)
				g4p_boot_fw_pages_changed(g);
			break;
		case 3:	/* CTX */
			if (len < 4 || !x->pc || !get_unaligned_le32(p) || get_unaligned_le32(p) >= G4P_NR_CTX) {
				ret = -EINVAL;
				break;
			}
			x->slot = get_unaligned_le32(p);
			wmb();	/* page tables before the context table names them */
			ret = g4p_boot_load_ctx(g, x->slot, x->pc_dma);
			break;
		case 4: {	/* PUSH */
			u32 e[5];

			if (len < 28) {
				ret = -EINVAL;
				break;
			}
			for (unsigned int i = 0; i < 5; i++)
				e[i] = get_unaligned_le32(p + 8 + 4 * i);
			wmb();	/* everything written so far before the ring entry */
			ret = g4p_boot_ring_kick(g, get_unaligned_le32(p), e, get_unaligned_le32(p + 4), x->slot);
			break;
		}
		case 5: {	/* WAIT */
			u32 fa = get_unaligned_le32(p), mask = get_unaligned_le32(p + 4), val = get_unaligned_le32(p + 8);
			u32 ms = min(get_unaligned_le32(p + 12), 10000U);
			u32 *w = g4p_gfx_ptr(x, g4p_fw_to_gpuva(fa), false);
			unsigned int us;

			if (!w) {
				ret = -EINVAL;
				break;
			}
			for (us = 0; us < ms * 1000; us += 50) {
				if (!(us % 1000))
					g4p_boot_ack_rings(g);	/* host-bound trace / statistics rings (8.89) */
				if ((READ_ONCE(*w) & mask) == val)
					break;
				udelay(50);
			}
			dev_info(g->dev, "C9 wait %08x & %08x == %08x: %s after %u us (now %08x)\n", fa, mask, val,
				 us < ms * 1000 ? "done" : "TIMEOUT", us, READ_ONCE(*w));
			break;
		}
		case 6:	/* READ */
			if (len < 16) {
				ret = -EINVAL;
				break;
			}
			rmb();
			ret = g4p_gfx_read(g, x, get_unaligned_le32(p), get_unaligned_le64(p + 8), get_unaligned_le32(p + 4));
			break;
		case 8:	/* PBALLOC */
			if (len < 16) {
				ret = -EINVAL;
				break;
			}
			ret = g4p_gfx_pballoc(g, x, get_unaligned_le32(p), get_unaligned_le64(p + 8));
			if (ret == -ETIMEDOUT)
				ret = 0;	/* logged; keep going so that later records show the state */
			break;
		case 9: {	/* METAPC: T0/T1 PC and PCX into gfxout and the log */
			u32 v[4] = { 0 };

			for (unsigned int t = 0; t < 2; t++) {
				g4p_boot_meta_core_read(g, t, 5, 0, &v[2 * t]);
				g4p_boot_meta_core_read(g, t, 5, 1, &v[2 * t + 1]);
			}
			dev_info(g->dev, "C9 META: T0 PC %08x PCX %08x, T1 PC %08x PCX %08x\n", v[0], v[1], v[2], v[3]);
			break;
		}
		case 7:	/* SLEEP */
			msleep(min(len >= 4 ? get_unaligned_le32(p) : 0, 10000U));
			break;
		default:
			ret = -EINVAL;
		}
	}
	dev_info(g->dev, "C9: %u records, %d; %zu bytes of output\n", nrec, ret, x->out_len);
out:
	release_firmware(fw);
	return ret;
}

ssize_t g4p_gfx_out(struct g4p *g, char __user *ubuf, size_t len, loff_t *pos)
{
	if (!g->gfx || !g->gfx->out)
		return 0;
	return simple_read_from_buffer(ubuf, len, pos, g->gfx->out, g->gfx->out_len);
}
