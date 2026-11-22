// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 GPU (G4P): GPU MMU for the firmware heaps, and GPU buffers in them.
 *
 * Stock Rogue MMU, 4 KiB pages, 40-bit VA and PA (linux-on-iphone6 re/GPU-META-BOOT.md 1.2):
 * PC (1024 x u32, VA[39:30]) -> PD (512 x u64, VA[29:21]) -> PT (512 x u64, VA[20:12]). All firmware heaps
 * (AGXGart heaps 5..8, 0xe1c0000000..0xe1c4600000) sit under one PC entry, so this keeps one PD and
 * allocates page tables on demand. Only what is mapped here is reachable by the GPU.
 */

#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/string.h>

#include "g4p.h"

#define MMU_PC_VALID		BIT(0)
#define MMU_PD_VALID		BIT_ULL(0)
#define MMU_PA_MASK		GENMASK_ULL(39, 12)
#define MMU_PC_INDEX(va)	(((va) >> 30) & 0x3ff)
#define MMU_PD_INDEX(va)	(((va) >> 21) & 0x1ff)
#define MMU_PT_INDEX(va)	(((va) >> 12) & 0x1ff)
#define APPLE_G4P_POOL_VA_K	0x300000000ULL	/* = APPLE_G4P_POOL_VA (uapi) */

/* AGXGart heap table (iOS 10 k10:0x1320 = iOS 12 0x…7135ed0): base, size, PTE flags */
static const struct {
	u64 base, size, flags;
} g4p_heap_cfg[G4P_NR_HEAPS] = {
	[G4P_HEAP_FW]      = { 0xe1c0000000ULL, SZ_2M,  G4P_PTE_PM_META_PROTECT },
	[G4P_HEAP_SHARED]  = { 0xe1c0200000ULL, SZ_32M + SZ_16M, G4P_PTE_PM_META_PROTECT | G4P_PTE_CC },
	[G4P_HEAP_PRIVATE] = { 0xe1c3200000ULL, SZ_16M, G4P_PTE_PM_META_PROTECT | G4P_PTE_CC },
	[G4P_HEAP_PIO]     = { 0xe1c4200000ULL, SZ_4M,  G4P_PTE_PM_META_PROTECT | G4P_PTE_SLC_BYPASS },
};

/*
 * iOS maps the shared heaps cache-coherent (CC) and turns snooping on (Fender +0 bit 29). Linux does not
 * snoop yet, so with CC the GPU's SLC kept a stale copy of host-written memory (a channel pointer read
 * back as 0 after the host had written it, GPU-BRINGUP 8.33). Bring-up default: bypass the SLC instead.
 */
static bool slc_bypass = true;
module_param(slc_bypass, bool, 0444);
MODULE_PARM_DESC(slc_bypass, "map the shared/private firmware heaps SLC-bypass instead of cache-coherent");

u64 g4p_mmu_heap_flags(enum g4p_heap heap)
{
	u64 f = g4p_heap_cfg[heap].flags;

	if (slc_bypass && (heap == G4P_HEAP_SHARED || heap == G4P_HEAP_PRIVATE))
		f = (f & ~G4P_PTE_CC) | G4P_PTE_SLC_BYPASS;
	return f;
}


int g4p_mmu_init(struct device *dev, struct g4p_mmu *mmu)
{
	u64 base = g4p_heap_cfg[G4P_HEAP_FW].base;

	memset(mmu, 0, sizeof(*mmu));
	mmu->pc = dma_alloc_coherent(dev, SZ_4K, &mmu->pc_dma, GFP_KERNEL);
	mmu->pd = dma_alloc_coherent(dev, SZ_4K, &mmu->pd_dma, GFP_KERNEL);
	if (!mmu->pc || !mmu->pd) {
		g4p_mmu_fini(dev, mmu);
		return -ENOMEM;
	}
	mmu->pc_index = MMU_PC_INDEX(base);
	mmu->pc[mmu->pc_index] = ((mmu->pd_dma >> 12) << 4) | MMU_PC_VALID;
	for (unsigned int i = 0; i < G4P_NR_HEAPS; i++)
		mmu->heap_next[i] = g4p_heap_cfg[i].base;
	return 0;
}

void g4p_mmu_fini(struct device *dev, struct g4p_mmu *mmu)
{
	for (unsigned int c = 1; c < G4P_NR_CTX; c++)
		g4p_mmu_ctx_fini(dev, mmu, c);
	for (unsigned int i = 0; i < ARRAY_SIZE(mmu->xpt); i++)
		if (mmu->xpt[i])
			dma_free_coherent(dev, SZ_4K, mmu->xpt[i], mmu->xpt_dma[i]);
	if (mmu->xpd)
		dma_free_coherent(dev, SZ_4K, mmu->xpd, mmu->xpd_dma);
	for (unsigned int i = 0; i < ARRAY_SIZE(mmu->pt); i++)
		if (mmu->pt[i])
			dma_free_coherent(dev, SZ_4K, mmu->pt[i], mmu->pt_dma[i]);
	if (mmu->pd)
		dma_free_coherent(dev, SZ_4K, mmu->pd, mmu->pd_dma);
	if (mmu->pc)
		dma_free_coherent(dev, SZ_4K, mmu->pc, mmu->pc_dma);
	memset(mmu, 0, sizeof(*mmu));
}

/* map [pa, pa + size) at GPU VA @va (all 4 KiB aligned, inside this PC entry) */
int g4p_mmu_map(struct device *dev, struct g4p_mmu *mmu, u64 va, phys_addr_t pa, size_t size, u64 flags)
{
	if (!IS_ALIGNED(va | pa | size, SZ_4K) || !size || MMU_PC_INDEX(va) != mmu->pc_index ||
	    MMU_PC_INDEX(va + size - 1) != mmu->pc_index || (pa + size - 1) & ~GENMASK_ULL(39, 0))
		return -EINVAL;
	for (size_t off = 0; off < size; off += SZ_4K) {
		u64 a = va + off;
		unsigned int d = MMU_PD_INDEX(a);

		if (!mmu->pt[d]) {
			mmu->pt[d] = dma_alloc_coherent(dev, SZ_4K, &mmu->pt_dma[d], GFP_KERNEL);
			if (!mmu->pt[d])
				return -ENOMEM;
			mmu->pd[d] = (mmu->pt_dma[d] & MMU_PA_MASK) | MMU_PD_VALID;
		}
		if (mmu->pt[d][MMU_PT_INDEX(a)])
			return -EEXIST;
		mmu->pt[d][MMU_PT_INDEX(a)] = ((pa + off) & MMU_PA_MASK) | flags | G4P_PTE_VALID;
	}
	return 0;
}

/* 8.77: the extra entry's PD, created on first use for the 1 GiB at @va */
static int g4p_mmu_xpd_get(struct device *dev, struct g4p_mmu *mmu, u64 va)
{
	if (!mmu->xpd) {
		mmu->xpd = dma_alloc_coherent(dev, SZ_4K, &mmu->xpd_dma, GFP_KERNEL);
		if (!mmu->xpd)
			return -ENOMEM;
		mmu->xva = va & ~(u64)(SZ_1G - 1);
		mmu->pc[MMU_PC_INDEX(va)] = ((mmu->xpd_dma >> 12) << 4) | MMU_PC_VALID;
	}
	return (va & ~(u64)(SZ_1G - 1)) == mmu->xva ? 0 : -EINVAL;
}

/*
 * One 4 KiB page at @va outside the firmware heaps' PC entry: all such pages share one 1 GiB PC entry (own PD,
 * page tables per 2 MiB on demand), set by the first call. For the code heap / user pool at 0x300000000.
 */
int g4p_mmu_map_page(struct device *dev, struct g4p_mmu *mmu, u64 va, phys_addr_t pa, u64 flags)
{
	unsigned int pc = MMU_PC_INDEX(va), d = MMU_PD_INDEX(va);

	if (!IS_ALIGNED(va | pa, SZ_4K) || pc == mmu->pc_index || va & ~GENMASK_ULL(39, 0) ||
	    pa & ~GENMASK_ULL(39, 0))
		return -EINVAL;
	int ret = g4p_mmu_xpd_get(dev, mmu, va);

	if (ret)
		return ret;
	if (!mmu->xpt[d]) {
		mmu->xpt[d] = dma_alloc_coherent(dev, SZ_4K, &mmu->xpt_dma[d], GFP_KERNEL);
		if (!mmu->xpt[d])
			return -ENOMEM;
		mmu->xpd[d] = (mmu->xpt_dma[d] & MMU_PA_MASK) | MMU_PD_VALID;
	}
	if (mmu->xpt[d][MMU_PT_INDEX(va)])
		return -EEXIST;
	mmu->xpt[d][MMU_PT_INDEX(va)] = (pa & MMU_PA_MASK) | flags | G4P_PTE_VALID;
	return 0;
}

/*
 * R5 (8.77, 8.79): memory context @ctx (1..7). Its PC is the kernel's with the extra entry replaced by the
 * context's own PD; that PD starts empty and has page tables of its own only: nothing of the extra GiB is
 * shared with the kernel or other contexts (the caller maps the pool pages the context may use). Idempotent.
 */
int g4p_mmu_ctx_init(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx)
{
	struct g4p_mmu_ctx *c;
	int ret;

	if (!ctx || ctx >= G4P_NR_CTX)
		return -EINVAL;
	if (mmu->ctx[ctx])
		return 0;
	ret = g4p_mmu_xpd_get(dev, mmu, APPLE_G4P_POOL_VA_K);
	if (ret)
		return ret;
	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	mmu->ctx[ctx] = c;
	c->pc = dma_alloc_coherent(dev, SZ_4K, &c->pc_dma, GFP_KERNEL);
	c->pd = dma_alloc_coherent(dev, SZ_4K, &c->pd_dma, GFP_KERNEL);
	c->fpd = dma_alloc_coherent(dev, SZ_4K, &c->fpd_dma, GFP_KERNEL);
	for (unsigned int i = 0; i < G4P_CTX_FW_PAGES && c->fpd; i++)
		if (!(c->fpage[i] = dma_alloc_coherent(dev, SZ_4K, &c->fpage_dma[i], GFP_KERNEL)))
			break;
	if (!c->pc || !c->pd || !c->fpd || !c->fpage[G4P_CTX_FW_PAGES - 1]) {
		g4p_mmu_ctx_fini(dev, mmu, ctx);
		return -ENOMEM;
	}
	/*
	 * 8.81: only two PC entries: the extra GiB (own PD) and the firmware heaps' GiB (own PD, empty until the
	 * job slots' private pages are mapped). The kernel's firmware heaps, PIO register windows included, are
	 * not reachable from a user context.
	 */
	c->pc[MMU_PC_INDEX(mmu->xva)] = ((c->pd_dma >> 12) << 4) | MMU_PC_VALID;
	c->pc[mmu->pc_index] = ((c->fpd_dma >> 12) << 4) | MMU_PC_VALID;
	return 0;
}

void g4p_mmu_ctx_fini(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx)
{
	struct g4p_mmu_ctx *c = mmu->ctx[ctx];

	if (!c)
		return;
	for (unsigned int i = 0; i < ARRAY_SIZE(c->pt); i++)
		if (c->pt[i])
			dma_free_coherent(dev, SZ_4K, c->pt[i], c->pt_dma[i]);
	for (unsigned int i = 0; i < ARRAY_SIZE(c->fpt); i++)
		if (c->fpt[i])
			dma_free_coherent(dev, SZ_4K, c->fpt[i], c->fpt_dma[i]);
	for (unsigned int i = 0; i < G4P_CTX_FW_PAGES; i++)
		if (c->fpage[i])
			dma_free_coherent(dev, SZ_4K, c->fpage[i], c->fpage_dma[i]);
	if (c->fpd)
		dma_free_coherent(dev, SZ_4K, c->fpd, c->fpd_dma);
	if (c->pd)
		dma_free_coherent(dev, SZ_4K, c->pd, c->pd_dma);
	if (c->pc)
		dma_free_coherent(dev, SZ_4K, c->pc, c->pc_dma);
	kfree(c);
	mmu->ctx[ctx] = NULL;
}

/*
 * 8.81: private page @page of context @ctx at @va in the firmware heaps' GiB of that context only (idempotent:
 * the same page at the same VA again is fine)
 */
int g4p_mmu_ctx_map_fw_page(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx, u64 va, unsigned int page,
			    u64 flags)
{
	struct g4p_mmu_ctx *c = ctx && ctx < G4P_NR_CTX ? mmu->ctx[ctx] : NULL;
	unsigned int d = MMU_PD_INDEX(va);
	u64 pte;

	if (!c || page >= G4P_CTX_FW_PAGES || MMU_PC_INDEX(va) != mmu->pc_index || !IS_ALIGNED(va, SZ_4K))
		return -EINVAL;
	if (!c->fpt[d]) {
		c->fpt[d] = dma_alloc_coherent(dev, SZ_4K, &c->fpt_dma[d], GFP_KERNEL);
		if (!c->fpt[d])
			return -ENOMEM;
		c->fpd[d] = (c->fpt_dma[d] & MMU_PA_MASK) | MMU_PD_VALID;
	}
	pte = (c->fpage_dma[page] & MMU_PA_MASK) | flags | G4P_PTE_VALID;
	if (c->fpt[d][MMU_PT_INDEX(va)] && c->fpt[d][MMU_PT_INDEX(va)] != pte)
		return -EEXIST;
	c->fpt[d][MMU_PT_INDEX(va)] = pte;
	return 0;
}

/* one page of the extra GiB (pool or user range) in context @ctx */
int g4p_mmu_ctx_map_page(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx, u64 va, phys_addr_t pa,
			 u64 flags)
{
	struct g4p_mmu_ctx *c = ctx && ctx < G4P_NR_CTX ? mmu->ctx[ctx] : NULL;
	unsigned int d = MMU_PD_INDEX(va);

	if (!c || (va & ~(u64)(SZ_1G - 1)) != mmu->xva || !IS_ALIGNED(va | pa, SZ_4K) || pa & ~GENMASK_ULL(39, 0))
		return -EINVAL;
	if (!c->pt[d]) {
		c->pt[d] = dma_alloc_coherent(dev, SZ_4K, &c->pt_dma[d], GFP_KERNEL);
		if (!c->pt[d])
			return -ENOMEM;
		c->pd[d] = (c->pt_dma[d] & MMU_PA_MASK) | MMU_PD_VALID;
	}
	if (c->pt[d][MMU_PT_INDEX(va)])
		return -EEXIST;
	c->pt[d][MMU_PT_INDEX(va)] = (pa & MMU_PA_MASK) | flags | G4P_PTE_VALID;
	return 0;
}

void g4p_mmu_ctx_unmap_page(struct g4p_mmu *mmu, unsigned int ctx, u64 va)
{
	struct g4p_mmu_ctx *c = ctx && ctx < G4P_NR_CTX ? mmu->ctx[ctx] : NULL;
	unsigned int d = MMU_PD_INDEX(va);

	if (c && (va & ~(u64)(SZ_1G - 1)) == mmu->xva && c->pt[d])
		c->pt[d][MMU_PT_INDEX(va)] = 0;
}

/* clear one page of the extra entry (the next job's MMU invalidate makes it visible) */
void g4p_mmu_unmap_page(struct g4p_mmu *mmu, u64 va)
{
	unsigned int d = MMU_PD_INDEX(va);

	if (mmu->xpd && (va & ~(u64)(SZ_1G - 1)) == mmu->xva && mmu->xpt[d])
		mmu->xpt[d][MMU_PT_INDEX(va)] = 0;
}

/* a zeroed, non-cacheable buffer mapped into @heap with the heap's PTE flags (bump allocated) */
int g4p_bo_alloc(struct device *dev, struct g4p_mmu *mmu, enum g4p_heap heap, size_t size, struct g4p_bo *bo)
{
	u64 va = mmu->heap_next[heap];
	int ret;

	size = ALIGN(size, SZ_4K);
	if (va + size > g4p_heap_cfg[heap].base + g4p_heap_cfg[heap].size)
		return -ENOSPC;
	bo->cpu = dma_alloc_coherent(dev, size, &bo->dma, GFP_KERNEL);
	if (!bo->cpu)
		return -ENOMEM;
	bo->size = size;
	bo->gpuva = va;
	ret = g4p_mmu_map(dev, mmu, va, bo->dma, size, g4p_mmu_heap_flags(heap));
	if (ret) {
		dma_free_coherent(dev, size, bo->cpu, bo->dma);
		bo->cpu = NULL;
		return ret;
	}
	mmu->heap_next[heap] = va + size;
	return 0;
}

/* a physical register window in the PIO heap (the firmware's view of RGX, Fender, AIC) */
int g4p_mmu_map_pio(struct device *dev, struct g4p_mmu *mmu, phys_addr_t pa, size_t size, u64 *gpuva)
{
	u64 va = mmu->heap_next[G4P_HEAP_PIO];
	int ret;

	size = ALIGN(size, SZ_4K);
	if (va + size > g4p_heap_cfg[G4P_HEAP_PIO].base + g4p_heap_cfg[G4P_HEAP_PIO].size)
		return -ENOSPC;
	ret = g4p_mmu_map(dev, mmu, va, pa, size, g4p_heap_cfg[G4P_HEAP_PIO].flags);
	if (ret)
		return ret;
	mmu->heap_next[G4P_HEAP_PIO] = va + size;
	*gpuva = va;
	return 0;
}

/* the buffer's pages stay mapped (the GPU MMU tables go with g4p_mmu_fini, after power-off) */
void g4p_bo_free(struct device *dev, struct g4p_bo *bo)
{
	if (bo->view) {
		memset(bo, 0, sizeof(*bo));
		return;
	}
	if (bo->cpu)
		dma_free_coherent(dev, bo->size, bo->cpu, bo->dma);
	bo->cpu = NULL;
}

u64 g4p_fw_heap_base(void)
{
	return g4p_heap_cfg[G4P_HEAP_FW].base;
}

/* the firmware's address for a GPU VA in its heaps (convertGPUVA, re/GPU-META-BOOT.md 2.2) */
u32 g4p_fw_addr(u64 gpuva, bool cached)
{
	return (u32)(gpuva + 0x40000000) | (cached ? 0x40000000 : 0xc0000000);
}
