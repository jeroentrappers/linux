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

static u64 g4p_heap_flags(enum g4p_heap heap)
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
 * R5 (8.77): memory context @ctx (1..7). Its PC is the kernel's with the extra entry replaced by the context's
 * PD. That PD's low half (the pool, PD entries 0..255) points at the kernel's page tables and is refreshed
 * here; call it again after the kernel mapped new pool page tables. Idempotent.
 */
int g4p_mmu_ctx_init(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx)
{
	struct g4p_mmu_ctx *c;
	int ret;

	if (!ctx || ctx >= G4P_NR_CTX)
		return -EINVAL;
	ret = g4p_mmu_xpd_get(dev, mmu, APPLE_G4P_POOL_VA_K);
	if (ret)
		return ret;
	c = &mmu->ctx[ctx];
	if (!c->pc) {
		c->pc = dma_alloc_coherent(dev, SZ_4K, &c->pc_dma, GFP_KERNEL);
		c->pd = dma_alloc_coherent(dev, SZ_4K, &c->pd_dma, GFP_KERNEL);
		if (!c->pc || !c->pd) {
			g4p_mmu_ctx_fini(dev, mmu, ctx);
			return -ENOMEM;
		}
	}
	memcpy(c->pc, mmu->pc, SZ_4K);
	c->pc[MMU_PC_INDEX(mmu->xva)] = ((c->pd_dma >> 12) << 4) | MMU_PC_VALID;
	memcpy(c->pd, mmu->xpd, 256 * sizeof(u64));
	return 0;
}

void g4p_mmu_ctx_fini(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx)
{
	struct g4p_mmu_ctx *c = &mmu->ctx[ctx];

	for (unsigned int i = 0; i < ARRAY_SIZE(c->pt); i++)
		if (c->pt[i])
			dma_free_coherent(dev, SZ_4K, c->pt[i], c->pt_dma[i]);
	if (c->pd)
		dma_free_coherent(dev, SZ_4K, c->pd, c->pd_dma);
	if (c->pc)
		dma_free_coherent(dev, SZ_4K, c->pc, c->pc_dma);
	memset(c, 0, sizeof(*c));
}

/* one page in context @ctx's own half of the extra entry (VA 0x320000000..0x33fffffff) */
int g4p_mmu_ctx_map_page(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx, u64 va, phys_addr_t pa,
			 u64 flags)
{
	struct g4p_mmu_ctx *c = &mmu->ctx[ctx];
	unsigned int d = MMU_PD_INDEX(va);

	if (!ctx || ctx >= G4P_NR_CTX || !c->pc || (va & ~(u64)(SZ_1G - 1)) != mmu->xva || d < 256 ||
	    !IS_ALIGNED(va | pa, SZ_4K) || pa & ~GENMASK_ULL(39, 0))
		return -EINVAL;
	if (!c->pt[d - 256]) {
		c->pt[d - 256] = dma_alloc_coherent(dev, SZ_4K, &c->pt_dma[d - 256], GFP_KERNEL);
		if (!c->pt[d - 256])
			return -ENOMEM;
		c->pd[d] = (c->pt_dma[d - 256] & MMU_PA_MASK) | MMU_PD_VALID;
	}
	if (c->pt[d - 256][MMU_PT_INDEX(va)])
		return -EEXIST;
	c->pt[d - 256][MMU_PT_INDEX(va)] = (pa & MMU_PA_MASK) | flags | G4P_PTE_VALID;
	return 0;
}

void g4p_mmu_ctx_unmap_page(struct g4p_mmu *mmu, unsigned int ctx, u64 va)
{
	struct g4p_mmu_ctx *c = &mmu->ctx[ctx];
	unsigned int d = MMU_PD_INDEX(va);

	if (ctx && ctx < G4P_NR_CTX && c->pc && (va & ~(u64)(SZ_1G - 1)) == mmu->xva && d >= 256 && c->pt[d - 256])
		c->pt[d - 256][MMU_PT_INDEX(va)] = 0;
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
	ret = g4p_mmu_map(dev, mmu, va, bo->dma, size, g4p_heap_flags(heap));
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
