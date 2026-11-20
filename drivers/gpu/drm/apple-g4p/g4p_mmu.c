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

#include "g4p.h"

#define MMU_PC_VALID		BIT(0)
#define MMU_PD_VALID		BIT_ULL(0)
#define MMU_PA_MASK		GENMASK_ULL(39, 12)
#define MMU_PC_INDEX(va)	(((va) >> 30) & 0x3ff)
#define MMU_PD_INDEX(va)	(((va) >> 21) & 0x1ff)
#define MMU_PT_INDEX(va)	(((va) >> 12) & 0x1ff)

/* AGXGart heap table (iOS 10 k10:0x1320 = iOS 12 0x…7135ed0): base, size, PTE flags */
static const struct {
	u64 base, size, flags;
} g4p_heap_cfg[G4P_NR_HEAPS] = {
	[G4P_HEAP_FW]      = { 0xe1c0000000ULL, SZ_2M,  G4P_PTE_PM_META_PROTECT },
	[G4P_HEAP_SHARED]  = { 0xe1c0200000ULL, SZ_32M + SZ_16M, G4P_PTE_PM_META_PROTECT | G4P_PTE_CC },
	[G4P_HEAP_PRIVATE] = { 0xe1c3200000ULL, SZ_16M, G4P_PTE_PM_META_PROTECT | G4P_PTE_CC },
	[G4P_HEAP_PIO]     = { 0xe1c4200000ULL, SZ_4M,  G4P_PTE_PM_META_PROTECT | G4P_PTE_SLC_BYPASS },
};

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
	ret = g4p_mmu_map(dev, mmu, va, bo->dma, size, g4p_heap_cfg[heap].flags);
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
	if (bo->cpu)
		dma_free_coherent(dev, bo->size, bo->cpu, bo->dma);
	bo->cpu = NULL;
}

/* the firmware's address for a GPU VA in its heaps (convertGPUVA, re/GPU-META-BOOT.md 2.2) */
u32 g4p_fw_addr(u64 gpuva, bool cached)
{
	return (u32)(gpuva + 0x40000000) | (cached ? 0x40000000 : 0xc0000000);
}
