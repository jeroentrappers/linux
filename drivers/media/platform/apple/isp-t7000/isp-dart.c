// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: minimal in-module driver for dart-isp (s5l8960x format) and
 * the DART-mapped surfaces (firmware, IPC pool, heap, SHAREDMALLOC buffers).
 *
 * One TTBR (IOVA < 1 GB), 3 levels with 4K pages: L1 index = iova[29:21], L2 index =
 * iova[20:12]. Tables are dma_alloc_coherent (non-cacheable here); surface pages are
 * single pages, cache-cleaned with dma_map_page and accessed through a write-combine
 * vmap, so DART walk coherency does not matter. Only the DART block is programmed;
 * the SMMU block at +0x24000 is read-only unless smmu_en=1.
 */
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "isp.h"

#define dart_rd(isp, o)		isp_rd(isp, ISP_DART + (o))
#define dart_wr(isp, o, v)	isp_wr(isp, ISP_DART + (o), v)

void isp_dart_log(struct t7000_isp *isp, const char *when)
{
	u32 s[16];
	int i;

	dev_info(isp->dev, "dart %s: cmd %08x tcr %08x err %08x erraddr %08x ttbr0 %08x %08x %08x %08x\n",
		 when, dart_rd(isp, DART_STREAM_CMD), dart_rd(isp, DART_TCR),
		 dart_rd(isp, DART_ERROR), dart_rd(isp, DART_ERROR_ADDR),
		 dart_rd(isp, DART_TTBR(0, 0)), dart_rd(isp, DART_TTBR(1, 0)),
		 dart_rd(isp, DART_TTBR(2, 0)), dart_rd(isp, DART_TTBR(3, 0)));
	for (i = 0; i < 16; i++)
		s[i] = isp_rd(isp, ISP_SMMU + 4 * i);
	dev_info(isp->dev, "smmu %s: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		 when, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9],
		 s[10], s[11], s[12], s[13], s[14], s[15]);
}

bool isp_dart_fault(struct t7000_isp *isp)
{
	u32 err = dart_rd(isp, DART_ERROR);

	if (!(err & DART_ERROR_FLAG))
		return false;
	if (isp->dart_err_logged++ < 5)
		dev_err(isp->dev, "DART fault: error 0x%08x (stream %lu code 0x%02x) addr 0x%08x\n",
			err, (unsigned long)((err >> 24) & 3), err & 0xff,
			dart_rd(isp, DART_ERROR_ADDR));
	/* W1C ack (mainline apple_dart_s5l8960x_irq): only new faults count from here on */
	dart_wr(isp, DART_ERROR, err);
	return true;
}

static void isp_dart_flush(struct t7000_isp *isp)
{
	u32 v;

	if (!isp->dart_on)
		return;
	dma_wmb();
	dart_wr(isp, DART_STREAM_CMD, (isp_dart_sids << 8) | DART_STREAM_CMD_INVAL);
	if (readl_poll_timeout_atomic(isp->regs + ISP_DART + DART_STREAM_CMD, v,
				      !(v & DART_STREAM_CMD_BUSY), 1, 10000))
		dev_err(isp->dev, "DART TLB invalidate: timeout (10 ms) waiting for BUSY to clear, cmd 0x%08x\n", v);
}

static u32 tcr_mask(void)
{
	u32 m = 0;
	int sid;

	for (sid = 0; sid < 4; sid++)
		if (isp_dart_sids & BIT(sid))
			m |= 0xffU << (8 * sid);
	return m;
}

int isp_dart_init(struct t7000_isp *isp)
{
	u32 m = tcr_mask(), tcr, err;
	int sid, i;

	isp->l1 = dma_alloc_coherent(isp->dev, PAGE_SIZE, &isp->l1_dma, GFP_KERNEL);
	if (!isp->l1)
		return -ENOMEM;
	if (isp->l1_dma >= BIT_ULL(36)) {
		dev_err(isp->dev, "L1 table at %pad is above the 36-bit DART range\n", &isp->l1_dma);
		return -EIO;
	}
	isp_dart_log(isp, "before init");

	tcr = dart_rd(isp, DART_TCR);
	dart_wr(isp, DART_TCR, tcr & ~m);
	err = dart_rd(isp, DART_ERROR);
	dart_wr(isp, DART_ERROR, err);
	for (sid = 0; sid < 4; sid++) {
		if (!(isp_dart_sids & BIT(sid)))
			continue;
		dart_wr(isp, DART_TTBR(sid, 0), DART_TTBR_VALID | (u32)(isp->l1_dma >> 12));
		for (i = 1; i < 4; i++)
			dart_wr(isp, DART_TTBR(sid, i), 0);
	}
	dart_wr(isp, DART_TCR, (tcr & ~m) | (0x80808080U & m));
	isp->dart_on = true;
	isp_dart_flush(isp);
	if (isp_smmu_en) {
		u32 v = isp_rd(isp, ISP_SMMU + 0xc);

		isp_wr(isp, ISP_SMMU + 0xc, v | (0x80808080U & m));
		dev_info(isp->dev, "smmu_en: SMMU+0xc 0x%08x -> 0x%08x\n", v,
			 isp_rd(isp, ISP_SMMU + 0xc));
	}
	isp->iova_next = isp_fw_iova;
	dev_info(isp->dev, "dart: L1 at %pad, sids 0x%x, old tcr 0x%08x, old err 0x%08x\n",
		 &isp->l1_dma, isp_dart_sids, tcr, err);
	isp_dart_log(isp, "after init");
	return 0;
}

/* Hardware off (called while still powered). Tables are freed in isp_surf_free_all. */
void isp_dart_fini(struct t7000_isp *isp)
{
	int sid, i;

	isp_dart_fault(isp);
	dart_wr(isp, DART_TCR, dart_rd(isp, DART_TCR) & ~tcr_mask());
	for (sid = 0; sid < 4; sid++)
		if (isp_dart_sids & BIT(sid))
			for (i = 0; i < 4; i++)
				dart_wr(isp, DART_TTBR(sid, i), 0);
	isp_dart_flush(isp);	/* still dart_on: drop any cached translation */
	isp->dart_on = false;
	dev_info(isp->dev, "dart: translation disabled\n");
}

static int dart_map_page(struct t7000_isp *isp, u32 iova, dma_addr_t pa)
{
	unsigned int i1 = (iova >> 21) & 511, i2 = (iova >> 12) & 511;

	if (!isp->l2[i1]) {
		isp->l2[i1] = dma_alloc_coherent(isp->dev, PAGE_SIZE, &isp->l2_dma[i1], GFP_KERNEL);
		if (!isp->l2[i1])
			return -ENOMEM;
		if (isp->l2_dma[i1] >= BIT_ULL(36))
			return -EIO;
		dma_wmb();
		isp->l1[i1] = (isp->l2_dma[i1] & DART_PTE_PA) | DART_PTE_VALID;
	}
	isp->l2[i1][i2] = (pa & DART_PTE_PA) | DART_PTE_VALID;
	return 0;
}

static void dart_unmap_page(struct t7000_isp *isp, u32 iova)
{
	unsigned int i1 = (iova >> 21) & 511, i2 = (iova >> 12) & 511;

	if (isp->l2[i1])
		isp->l2[i1][i2] = 0;
}

/*
 * Two passes: clear every PTE and flush the DART TLB first, only then give the
 * pages back, so the ISP can never reach a page the kernel already reuses.
 * At runtime only the alloc-failure path gets here; fw frees are ack-only.
 */
void isp_surf_free(struct t7000_isp *isp, struct isp_surf *s)
{
	unsigned int i;

	if (s->virt)
		vunmap(s->virt);
	for (i = 0; s->pages && i < s->npages; i++)
		dart_unmap_page(isp, s->iova + i * PAGE_SIZE);
	isp_dart_flush(isp);
	for (i = 0; s->pages && s->dma && i < s->npages; i++) {
		if (!s->pages[i])
			continue;
		if (!dma_mapping_error(isp->dev, s->dma[i]))
			dma_unmap_page(isp->dev, s->dma[i], PAGE_SIZE, DMA_BIDIRECTIONAL);
		__free_page(s->pages[i]);
	}
	isp->shared_bytes -= s->size;
	list_del(&s->list);
	kfree(s->pages);
	kfree(s->dma);
	kfree(s);
}

struct isp_surf *isp_surf_alloc(struct t7000_isp *isp, u32 size, u32 tag)
{
	struct isp_surf *s;
	unsigned int i;

	size = PAGE_ALIGN(size);
	if (!size || (u64)isp->iova_next + size > (u64)isp_fw_iova + ISP_IOVA_SPAN ||
	    isp->shared_bytes + size > (u64)isp_max_shared_mb << 20) {
		dev_err(isp->dev, "surface '%4.4s' 0x%x: out of IOVA space or over max_shared_mb\n",
			(char *)&tag, size);
		return NULL;
	}
	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return NULL;
	s->npages = size >> PAGE_SHIFT;
	s->pages = kcalloc(s->npages, sizeof(*s->pages), GFP_KERNEL);
	s->dma = kcalloc(s->npages, sizeof(*s->dma), GFP_KERNEL);
	s->iova = isp->iova_next;
	s->size = size;
	s->tag = tag;
	INIT_LIST_HEAD(&s->list);
	list_add_tail(&s->list, &isp->surfs);
	isp->shared_bytes += size;
	if (!s->pages || !s->dma)
		goto fail;
	for (i = 0; i < s->npages; i++)
		s->dma[i] = DMA_MAPPING_ERROR;
	for (i = 0; i < s->npages; i++) {
		s->pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!s->pages[i])
			goto fail;
		s->dma[i] = dma_map_page(isp->dev, s->pages[i], 0, PAGE_SIZE, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(isp->dev, s->dma[i]) ||
		    s->dma[i] != page_to_phys(s->pages[i]) || s->dma[i] >= BIT_ULL(36)) {
			dev_err(isp->dev, "page dma %pad != phys (bounce?) or >= 2^36\n", &s->dma[i]);
			goto fail;
		}
		if (dart_map_page(isp, s->iova + i * PAGE_SIZE, s->dma[i]))
			goto fail;
	}
	s->virt = vmap(s->pages, s->npages, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	if (!s->virt)
		goto fail;
	isp_dart_flush(isp);
	/* one unmapped guard page between surfaces: overruns become DART faults */
	isp->iova_next += size + PAGE_SIZE;
	dev_info(isp->dev, "surface '%4.4s': iova 0x%08x (fw-rel 0x%08x) size 0x%x\n",
		 (char *)&tag, s->iova, isp_rel(s->iova), size);
	return s;
fail:
	dev_err(isp->dev, "surface '%4.4s' 0x%x: allocation/mapping failed\n", (char *)&tag, size);
	isp_surf_free(isp, s);
	return NULL;
}

void isp_surf_free_all(struct t7000_isp *isp)
{
	struct isp_surf *s, *n;
	int i;

	list_for_each_entry_safe(s, n, &isp->surfs, list)
		isp_surf_free(isp, s);
	isp->fw = isp->ipc = isp->heap = NULL;
	for (i = 0; i < 512; i++) {
		if (isp->l2[i])
			dma_free_coherent(isp->dev, PAGE_SIZE, isp->l2[i], isp->l2_dma[i]);
		isp->l2[i] = NULL;
	}
	if (isp->l1)
		dma_free_coherent(isp->dev, PAGE_SIZE, isp->l1, isp->l1_dma);
	isp->l1 = NULL;
}

struct isp_surf *isp_surf_find(struct t7000_isp *isp, u32 iova)
{
	struct isp_surf *s;

	list_for_each_entry(s, &isp->surfs, list)
		if (iova >= s->iova && iova - s->iova < s->size)
			return s;
	return NULL;
}

/* fw-relative address -> CPU pointer, or NULL if [rel, rel+len) is not inside one surface */
void *isp_translate(struct t7000_isp *isp, u32 rel, u32 len)
{
	u32 iova = isp_fw_iova + rel;
	struct isp_surf *s = isp_surf_find(isp, iova);

	if (!s || !s->virt || len > s->size - (iova - s->iova))
		return NULL;
	return (u8 *)s->virt + (iova - s->iova);
}
