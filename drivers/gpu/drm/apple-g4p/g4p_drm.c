// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 GPU (G4P): DRM render node, bring-up version (linux-on-iphone6 re/GPU-BRINGUP.md 8.64).
 *
 * Compute only. One contiguous pool (module parameter pool_mb) is allocated on first use and mapped at GPU VA
 * 0x300000000 before every firmware boot the DRM path starts, so buffers need no mapping after the boot
 * (post-boot mappings are not reliable yet, 8.34). Buffers are page ranges of the pool; page 0 holds the
 * driver's own PDS programs. SUBMIT runs a job image in the C8 record format (g4p_boot.c g4p_c8_job) on the
 * firmware's compute channel and waits for its completion stamp; jobs follow each other in one firmware boot
 * (8.63). Since then: separate BOs and VM_BIND (8.65, 8.71), async submits with sync objects (8.68, 8.69),
 * idle power-off (8.67) and (R5, 8.77) one GPU memory context per open file: separate BOs live in the file's
 * own page tables, the pool (code) stays shared by all.
 */

#include <linux/bitmap.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_print.h>
#include <drm/drm_syncobj.h>
#include <linux/dma-fence.h>
#include <uapi/drm/apple_g4p_drm.h>

#include "g4p.h"

static unsigned int idle_ms = 1000;
module_param(idle_ms, uint, 0644);
MODULE_PARM_DESC(idle_ms, "power the GPU off this long after the last DRM submit (0 = stay on); 8.67");

static bool raw_jobs;
module_param(raw_jobs, bool, 0644);
MODULE_PARM_DESC(raw_jobs, "accept HEAP, CMD and REG records in DRM job images (bring-up tools only: they reach memory and registers outside the file's context)");

static unsigned int pool_mb = 4;	/* at most 64: the VM_BIND code window starts at pool + 64 MiB (9.19) */
module_param(pool_mb, uint, 0444);
MODULE_PARM_DESC(pool_mb, "size of the DRM pool in MiB (contiguous, from CMA; mapped at GPU VA 0x300000000)");

struct g4p_gem {
	struct drm_gem_object base;
	unsigned int first, npages;	/* pool pages */
	/* APPLE_G4P_BO_SEPARATE (8.65) */
	bool separate;
	struct g4p_bo mem;		/* own allocation, mapped at mem.gpuva */
	struct list_head node;		/* g->sep_bos, while mapped */
	bool mapped;
	unsigned int ctx;		/* R5: the memory context of the file that created it */
	/* 9.15: imported dma-buf (one contiguous DMA segment, no CPU mapping); its context is set by VM_BIND */
	struct sg_table *sgt;
};

/* 9.22: a VM_BIND range mapping; holds a reference on its buffer */
struct g4p_map {
	struct list_head node;		/* g->maps */
	struct g4p_gem *bo;
	unsigned int ctx;
	u64 va, offset, size;
};

/* R5 (8.77): per DRM file: its own GPU memory context (1..7) and VA allocator for separate BOs */
struct g4p_file {
	unsigned int ctx;
	u64 sep_next;
};

#define G4P_SEP_VA	0x320000000ULL

struct g4p_zombie {
	struct g4p_bo mem;
	struct list_head node;
};

/* after power-off (g4p_boot_free): release memory of separate BOs freed while the GPU was on */
void g4p_drm_reap(struct g4p *g)
{
	struct g4p_zombie *z, *t;

	WARN_ON(g->powered);
	list_for_each_entry_safe(z, t, &g->zombies, node) {
		dma_free_coherent(g->dev, z->mem.size, z->mem.cpu, z->mem.dma);
		list_del(&z->node);
		kfree(z);
	}
}	/* separate BOs: from here, never reused (bring-up) */

#define to_g4p(d)	container_of(d, struct g4p, drm)
#define to_g4p_gem(o)	container_of(o, struct g4p_gem, base)

static int g4p_pool_get_locked(struct g4p *g)
{
	size_t size = (size_t)clamp(pool_mb, 1U, 64U) * SZ_1M;

	if (g->pool.cpu)
		return 0;
	g->pool_map = bitmap_zalloc(size / SZ_4K, GFP_KERNEL);
	g->pool_owner = kzalloc(size / SZ_4K, GFP_KERNEL);
	if (!g->pool_map || !g->pool_owner) {
		bitmap_free(g->pool_map);
		kfree(g->pool_owner);
		g->pool_map = NULL;
		g->pool_owner = NULL;
		return -ENOMEM;
	}
	g->pool.cpu = dma_alloc_coherent(g->dev, size, &g->pool.dma, GFP_KERNEL);
	if (!g->pool.cpu) {
		bitmap_free(g->pool_map);
		kfree(g->pool_owner);
		g->pool_map = NULL;
		g->pool_owner = NULL;
		return -ENOMEM;
	}
	g->pool.size = size;
	g->pool.gpuva = APPLE_G4P_POOL_VA;
	g->pool_pages = size / SZ_4K;
	set_bit(0, g->pool_map);	/* the driver's PDS programs (terminate at +0x100, data at +0x200) */
	return 0;
}

static struct dma_fence *g4p_wait_target(struct g4p *g, bool all);

/*
 * 8.70: the GPU MMU invalidate that the firmware runs at a kick (gart_inval) only takes effect for the job after
 * that one: a job right after an unmap can still use the old translation, any later job faults. So after an
 * unmap one empty job (STREAM_TERMINATE only) makes the unmap effective, and the memory can be freed.
 *
 * 8.78: the same empty job is what completes a job: some of a compute job's stores are still in a GPU cache
 * when its stamp is written, and reach memory only at the next kick, translated in *that* kick's memory
 * context (seen: another client's buffer at the same VA received them). So every DRM job is followed by an
 * empty job in its own context (g->job_ctx), and its fence waits for that one.
 */
static int g4p_flush_unmap_locked(struct g4p *g)
{
	static const u32 empty[4] = { 0x4a503447, 1, 0, 1000 };	/* "G4PJ", v1, no records */
	int ret;

	g->job_img = empty;
	g->job_img_size = sizeof(empty);
	ret = g4p_boot_cl_job(g, 8);
	g->job_img = NULL;
	return ret;
}

/*
 * 8.79: the same, in address space @ctx (an unmap there: its invalidate word changes). 8.84: one that is in no
 * hardware slot needs none: its jobs are done, and loading it into a slot again flushes the MMU caches.
 */
static int g4p_flush_ctx_locked(struct g4p *g, unsigned int ctx)
{
	unsigned int saved = g->job_ctx;
	int ret;

	if (!g4p_boot_ctx_resident(g, ctx))
		return 0;
	g->job_ctx = ctx;
	ret = g4p_flush_unmap_locked(g);
	g->job_ctx = saved;
	return ret;
}

static void g4p_gem_free(struct drm_gem_object *obj)
{
	struct g4p *g = to_g4p(obj->dev);
	struct g4p_gem *bo = to_g4p_gem(obj);
	struct dma_fence *f;

	/* jobs in flight may still use this buffer: let them finish first */
	while ((f = g4p_wait_target(g, true))) {
		dma_fence_wait(f, false);
		dma_fence_put(f);
	}
	mutex_lock(&g->lock);
	if (bo->sgt) {	/* 9.15: imported: out of the GPU MMU, then back to the exporter */
		if (bo->mapped) {
			list_del(&bo->node);
			g4p_boot_unmap_range(g, bo->ctx, bo->mem.gpuva, bo->mem.size);
			if (g->powered && g->boot && g4p_flush_ctx_locked(g, bo->ctx))
				g4p_recover_locked(g);
		}
		mutex_unlock(&g->lock);
		drm_prime_gem_destroy(obj, bo->sgt);
		drm_gem_object_release(obj);
		kfree(bo);
		return;
	}
	if (bo->separate && !bo->mapped) {
		dma_free_coherent(g->dev, bo->mem.size, bo->mem.cpu, bo->mem.dma);
	} else if (bo->separate) {
		list_del(&bo->node);
		g4p_boot_unmap_range(g, bo->ctx, bo->mem.gpuva, bo->mem.size);
		if (g->powered && g->boot && g4p_flush_ctx_locked(g, bo->ctx)) {
			/* the flush job failed: park the memory until the GPU is off (and recover) */
			struct g4p_zombie *z = kmalloc(sizeof(*z), GFP_KERNEL);

			if (z) {
				z->mem = bo->mem;
				list_add_tail(&z->node, &g->zombies);
			} /* else: leak rather than let the GPU write freed memory */
			g4p_recover_locked(g);
		} else {
			dma_free_coherent(g->dev, bo->mem.size, bo->mem.cpu, bo->mem.dma);
		}
	} else {
		/* 8.79: out of its context first, so the pages can go to another file */
		g4p_boot_pool_unmap(g, bo->ctx, bo->first, bo->npages);
		if (bo->ctx && g->powered && g->boot && g4p_flush_ctx_locked(g, bo->ctx))
			g4p_recover_locked(g);	/* the next boot maps without these pages */
		memset(g->pool_owner + bo->first, 0, bo->npages);
		bitmap_clear(g->pool_map, bo->first, bo->npages);
	}
	mutex_unlock(&g->lock);
	drm_gem_free_mmap_offset(obj);	/* else drm_mm_takedown warns at unload */
	drm_gem_object_release(obj);
	kfree(bo);
}

static int g4p_gem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct g4p *g = to_g4p(obj->dev);
	struct g4p_gem *bo = to_g4p_gem(obj);

	if (vma->vm_end - vma->vm_start > (unsigned long)bo->npages * SZ_4K)
		return -EINVAL;
	vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	if (bo->separate) {
		vma->vm_pgoff = 0;
		return dma_mmap_coherent(g->dev, vma, bo->mem.cpu, bo->mem.dma, bo->mem.size);
	}
	/* dma_mmap_coherent() maps the pool from page vm_pgoff: point it at this buffer's pages */
	vma->vm_pgoff = bo->first;
	return dma_mmap_coherent(g->dev, vma, g->pool.cpu, g->pool.dma, g->pool.size);
}

/* drm_gem_mmap_obj() takes a reference per mapping; these drop it when the mapping goes */
static const struct vm_operations_struct g4p_gem_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct drm_gem_object_funcs g4p_gem_funcs = {
	.free = g4p_gem_free,
	.mmap = g4p_gem_mmap,
	.vm_ops = &g4p_gem_vm_ops,
};

static int g4p_ioctl_get_param(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct drm_apple_g4p_get_param *a = data;

	switch (a->param) {
	case DRM_APPLE_G4P_PARAM_POOL_VA:
		a->value = APPLE_G4P_POOL_VA;
		return 0;
	case DRM_APPLE_G4P_PARAM_POOL_SIZE:
		a->value = (u64)clamp(pool_mb, 1U, 64U) * SZ_1M;
		return 0;
	case DRM_APPLE_G4P_PARAM_CORE_ID:
		a->value = g->have_id ? g->core_id : 0;
		return 0;
	case DRM_APPLE_G4P_PARAM_CORE_REVISION:
		a->value = g->have_id ? g->core_rev : 0;
		return 0;
	case DRM_APPLE_G4P_PARAM_MEMORY_CONTEXT:
		a->value = ((struct g4p_file *)file->driver_priv)->ctx;
		return 0;
	}
	return -EINVAL;
}

/* map every separate BO into a new boot (g4p_boot_start, before the firmware runs) */
int g4p_drm_map_separate(struct g4p *g)
{
	struct g4p_gem *bo;
	int ret = 0;

	struct g4p_map *m;

	list_for_each_entry(bo, &g->sep_bos, node)
		if (!ret)
			ret = g4p_boot_map_range(g, bo->ctx, bo->mem.gpuva, bo->mem.dma, bo->mem.size);
	list_for_each_entry(m, &g->maps, node)
		if (!ret)
			ret = g4p_boot_map_range(g, m->ctx, m->va, m->bo->mem.dma + m->offset, m->size);
	return ret;
}

static int g4p_bo_create_separate(struct g4p *g, struct drm_file *file, struct g4p_gem *bo, unsigned int n,
				  struct drm_apple_g4p_bo_create *a)
{
	struct g4p_file *gf = file->driver_priv;
	size_t size = (size_t)n * SZ_4K;
	int ret;

	bo->separate = true;
	bo->ctx = gf->ctx;
	bo->npages = n;	/* mmap size check */
	bo->mem.cpu = dma_alloc_coherent(g->dev, size, &bo->mem.dma, GFP_KERNEL);
	if (!bo->mem.cpu) {
		kfree(bo);
		return -ENOMEM;
	}
	bo->mem.size = size;
	mutex_lock(&g->lock);
	ret = 0;
	if (!(a->flags & APPLE_G4P_BO_NOMAP)) {
		/* driver-chosen VA from the top half of the user range (VM_BIND users get the bottom half) */
		if (!gf->sep_next)
			gf->sep_next = G4P_SEP_VA;
		if (gf->sep_next + size > APPLE_G4P_USER_VA_END) {
			ret = -ENOSPC;
			goto unlock;
		}
		bo->mem.gpuva = gf->sep_next;
		gf->sep_next += ALIGN(size, SZ_64K);	/* a gap between buffers: overruns fault */
		ret = g4p_boot_map_range(g, bo->ctx, bo->mem.gpuva, bo->mem.dma, size);	/* post-boot if running */
		if (!ret) {
			list_add_tail(&bo->node, &g->sep_bos);
			bo->mapped = true;
		}
	}
unlock:
	mutex_unlock(&g->lock);
	if (ret) {
		dma_free_coherent(g->dev, size, bo->mem.cpu, bo->mem.dma);
		kfree(bo);
		return ret;
	}
	bo->base.funcs = &g4p_gem_funcs;
	drm_gem_private_object_init(&g->drm, &bo->base, size);
	ret = drm_gem_create_mmap_offset(&bo->base);
	if (!ret)
		ret = drm_gem_handle_create(file, &bo->base, &a->handle);
	if (ret) {
		drm_gem_object_put(&bo->base);
		return ret;
	}
	a->gpu_va = bo->mapped ? bo->mem.gpuva : 0;
	a->mmap_offset = drm_vma_node_offset_addr(&bo->base.vma_node);
	drm_gem_object_put(&bo->base);
	return 0;
}

/* 8.71: does [va, va + size) overlap a mapped separate BO of memory context ctx? */
static bool g4p_va_busy(struct g4p *g, unsigned int ctx, u64 va, u64 size)
{
	struct g4p_gem *o;

	struct g4p_map *m;

	list_for_each_entry(o, &g->sep_bos, node)
		if (o->ctx == ctx && va < o->mem.gpuva + o->mem.size && o->mem.gpuva < va + size)
			return true;
	list_for_each_entry(m, &g->maps, node)
		if (m->ctx == ctx && va < m->va + m->size && m->va < va + size)
			return true;
	return false;
}

/* 9.22: map / unmap a range of a buffer; called with g->lock held */
static int g4p_vm_bind_range_locked(struct g4p *g, struct g4p_gem *bo, struct drm_apple_g4p_vm_bind *a)
{
	struct g4p_map *m;
	int ret;

	if (a->op == APPLE_G4P_VM_BIND_UNMAP) {
		list_for_each_entry(m, &g->maps, node) {
			if (m->bo != bo || m->va != a->gpu_va)
				continue;
			list_del(&m->node);
			g4p_boot_unmap_range(g, m->ctx, m->va, m->size);
			if (g->powered && g->boot && g4p_flush_ctx_locked(g, m->ctx))
				g4p_recover_locked(g);
			drm_gem_object_put(&bo->base);	/* the buffer itself is kept by its handle */
			kfree(m);
			return 0;
		}
		return -ENOENT;
	}
	if (a->op != APPLE_G4P_VM_BIND_MAP)
		return -EINVAL;
	{
		bool user = a->gpu_va >= APPLE_G4P_USER_VA_START && a->gpu_va + a->size <= APPLE_G4P_USER_VA_END;
		bool code = a->gpu_va >= APPLE_G4P_CODE_VA_START && a->gpu_va + a->size <= APPLE_G4P_CODE_VA_END;

		if (!IS_ALIGNED(a->gpu_va | a->offset | a->size, SZ_4K) || (!user && !code) ||
		    a->offset >= bo->mem.size || a->size > bo->mem.size - a->offset)
			return -EINVAL;
	}
	if (g4p_va_busy(g, bo->ctx, a->gpu_va, a->size))
		return -EBUSY;
	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	*m = (struct g4p_map){ .bo = bo, .ctx = bo->ctx, .va = a->gpu_va, .offset = a->offset, .size = a->size };
	ret = g4p_boot_map_range(g, m->ctx, m->va, bo->mem.dma + m->offset, m->size);
	if (ret) {
		g4p_boot_unmap_range(g, m->ctx, m->va, m->size);
		kfree(m);
		return ret;
	}
	drm_gem_object_get(&bo->base);
	list_add_tail(&m->node, &g->maps);
	return 0;
}

static int g4p_ioctl_vm_bind(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct drm_apple_g4p_vm_bind *a = data;
	struct drm_gem_object *obj;
	struct g4p_gem *bo;
	struct dma_fence *f;
	int ret = 0;

	obj = drm_gem_object_lookup(file, a->handle);
	if (!obj)
		return -ENOENT;
	bo = to_g4p_gem(obj);
	if (bo->sgt && !bo->ctx)	/* 9.15: an import belongs to the address space that binds it first */
		bo->ctx = ((struct g4p_file *)file->driver_priv)->ctx;
	if (!bo->separate || bo->ctx != ((struct g4p_file *)file->driver_priv)->ctx) {
		ret = -EINVAL;
		goto out;
	}
	if (a->op == APPLE_G4P_VM_BIND_UNMAP) {
		while ((f = g4p_wait_target(g, true))) {	/* jobs in flight may use it */
			dma_fence_wait(f, false);
			dma_fence_put(f);
		}
	}
	mutex_lock(&g->lock);
	if (a->size || (a->op == APPLE_G4P_VM_BIND_UNMAP && !(bo->mapped && bo->mem.gpuva == a->gpu_va))) {
		ret = g4p_vm_bind_range_locked(g, bo, a);
		mutex_unlock(&g->lock);
		goto out;
	}
	switch (a->op) {
	case APPLE_G4P_VM_BIND_MAP:
		bool user = a->gpu_va >= APPLE_G4P_USER_VA_START && a->gpu_va + bo->mem.size <= APPLE_G4P_USER_VA_END;
		bool code = a->gpu_va >= APPLE_G4P_CODE_VA_START && a->gpu_va + bo->mem.size <= APPLE_G4P_CODE_VA_END;

		if (bo->mapped || !IS_ALIGNED(a->gpu_va, SZ_4K) || (!user && !code) ||
		    a->gpu_va + bo->mem.size < a->gpu_va) {
			ret = -EINVAL;
		} else if (g4p_va_busy(g, bo->ctx, a->gpu_va, bo->mem.size)) {
			ret = -EBUSY;
		} else {
			bo->mem.gpuva = a->gpu_va;
			ret = g4p_boot_map_range(g, bo->ctx, bo->mem.gpuva, bo->mem.dma, bo->mem.size);
			if (!ret) {
				list_add_tail(&bo->node, &g->sep_bos);
				bo->mapped = true;
			}
		}
		break;
	case APPLE_G4P_VM_BIND_UNMAP:
		if (!bo->mapped) {
			ret = -EINVAL;
			break;
		}
		list_del(&bo->node);
		bo->mapped = false;
		g4p_boot_unmap_range(g, bo->ctx, bo->mem.gpuva, bo->mem.size);
		if (g->powered && g->boot && g4p_flush_ctx_locked(g, bo->ctx))
			g4p_recover_locked(g);	/* the flush job failed: power off, the next boot maps without it */
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&g->lock);
out:
	drm_gem_object_put(obj);
	return ret;
}

static int g4p_ioctl_bo_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct drm_apple_g4p_bo_create *a = data;
	struct g4p_gem *bo;
	unsigned long first;
	unsigned int n;
	int ret;

	if ((a->flags & ~(APPLE_G4P_BO_SEPARATE | APPLE_G4P_BO_NOMAP)) || !a->size || a->size > SZ_64M ||
	    ((a->flags & APPLE_G4P_BO_NOMAP) && !(a->flags & APPLE_G4P_BO_SEPARATE)))
		return -EINVAL;
	n = DIV_ROUND_UP(a->size, SZ_4K);
	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return -ENOMEM;
	if (a->flags & APPLE_G4P_BO_SEPARATE)
		return g4p_bo_create_separate(g, file, bo, n, a);
	mutex_lock(&g->lock);
	ret = g4p_pool_get_locked(g);
	if (!ret) {
		first = bitmap_find_next_zero_area(g->pool_map, g->pool_pages, 1, n, 0);
		if (first >= g->pool_pages) {
			ret = -ENOSPC;
		} else {
			bitmap_set(g->pool_map, first, n);
			/* 8.79: only the owner's context maps these pages */
			memset(g->pool_owner + first, ((struct g4p_file *)file->driver_priv)->ctx, n);
			memset(g->pool.cpu + (size_t)first * SZ_4K, 0, (size_t)n * SZ_4K);
			ret = g4p_boot_pool_map(g, ((struct g4p_file *)file->driver_priv)->ctx, first, n);
			if (ret) {
				g4p_boot_pool_unmap(g, ((struct g4p_file *)file->driver_priv)->ctx, first, n);
				memset(g->pool_owner + first, 0, n);
				bitmap_clear(g->pool_map, first, n);
			}
		}
	}
	mutex_unlock(&g->lock);
	if (ret) {
		kfree(bo);
		return ret;
	}
	bo->first = first;
	bo->npages = n;
	bo->ctx = ((struct g4p_file *)file->driver_priv)->ctx;
	bo->base.funcs = &g4p_gem_funcs;
	drm_gem_private_object_init(drm, &bo->base, (size_t)n * SZ_4K);
	ret = drm_gem_create_mmap_offset(&bo->base);
	if (!ret)
		ret = drm_gem_handle_create(file, &bo->base, &a->handle);
	if (ret) {
		drm_gem_object_put(&bo->base);
		return ret;
	}
	a->gpu_va = APPLE_G4P_POOL_VA + (u64)first * SZ_4K;
	a->mmap_offset = drm_vma_node_offset_addr(&bo->base.vma_node);
	drm_gem_object_put(&bo->base);	/* the handle holds the reference */
	return 0;
}

/* 8.68: one fence per async job; signalled from the firmware IRQ or, on timeout, with an error */
static const char *g4p_fence_driver(struct dma_fence *f) { return "apple-g4p"; }
static const char *g4p_fence_timeline(struct dma_fence *f) { return "compute"; }

static const struct dma_fence_ops g4p_fence_ops = {
	.get_driver_name = g4p_fence_driver,
	.get_timeline_name = g4p_fence_timeline,
};

#define G4P_FQ		8	/* = G4P_JOB_SLOTS (g4p_boot.c) */
#define G4P_INFLIGHT	(G4P_FQ / 2)	/* submits in flight: two job slots each (8.78) */

static void g4p_arm_timeout(struct g4p *g);

/* complete jobs in flight, oldest first, while their stamps are in memory (IRQ or process context) */
void g4p_drm_irq(struct g4p *g)
{
	struct dma_fence *done[G4P_FQ];
	unsigned int n = 0;
	unsigned long flags;

	spin_lock_irqsave(&g->fence_lock, flags);
	/*
	 * each entry: the job and its write-back job right after it (8.78). The firmware skips the first one's
	 * stamp (the second is already queued), so the second's stamp completes both.
	 */
	while (g->fq_tail != g->fq_head && g4p_boot_job_done(g, g->fq_job[g->fq_tail % G4P_FQ] + 1)) {
		g4p_boot_job_retire(g, g->fq_job[g->fq_tail % G4P_FQ]);
		done[n++] = g->fq[g->fq_tail % G4P_FQ];
		g->fq[g->fq_tail % G4P_FQ] = NULL;
		g->fq_tail++;
	}
	spin_unlock_irqrestore(&g->fence_lock, flags);
	for (unsigned int i = 0; i < n; i++) {
		dma_fence_signal(done[i]);
		dma_fence_put(done[i]);
	}
	if (n && g->fq_tail != g->fq_head)	/* the next oldest job gets a fresh timeout */
		g4p_arm_timeout(g);
}

/*
 * fail every job in flight (timeout, fault, or a job that was never kicked): with @fctx (1 + the faulting
 * context) the jobs of that context get -EFAULT, the others -ECANCELED (the recovery drops them); else the
 * oldest gets @err and the rest @rest
 */
static void g4p_fail_inflight(struct g4p *g, int err, int rest, int fctx)
{
	struct dma_fence *f;
	unsigned long flags;
	int e;

	for (;;) {
		spin_lock_irqsave(&g->fence_lock, flags);
		f = NULL;
		if (g->fq_tail != g->fq_head) {
			f = g->fq[g->fq_tail % G4P_FQ];
			e = fctx ? (g->fq_ctx[g->fq_tail % G4P_FQ] + 1 == fctx ? -EFAULT : -ECANCELED) : err;
			g->fq[g->fq_tail % G4P_FQ] = NULL;
			g->fq_tail++;
		}
		spin_unlock_irqrestore(&g->fence_lock, flags);
		if (!f)
			break;
		dma_fence_set_error(f, e);
		dma_fence_signal(f);
		dma_fence_put(f);
		err = rest;
	}
}

#define G4P_TIMEOUT_MS		1000
#define G4P_FAULT_POLL_MS	20	/* 8.83: a faulted job never completes; look for the fault this often */

/* (re)start the oldest job's timeout */
static void g4p_arm_timeout(struct g4p *g)
{
	g->fq_deadline = jiffies + msecs_to_jiffies(G4P_TIMEOUT_MS);
	mod_delayed_work(system_wq, &g->timeout_work, msecs_to_jiffies(G4P_FAULT_POLL_MS));
}

static void g4p_timeout_work(struct work_struct *w)
{
	struct g4p *g = container_of(to_delayed_work(w), struct g4p, timeout_work);
	int fctx;

	mutex_lock(&g->lock);
	g4p_drm_irq(g);	/* a missed interrupt: check the stamps once more */
	if (g->fq_tail == g->fq_head)
		goto out;
	/* 8.80/8.83: a job that faulted never completes; the BIF fault status names its context */
	fctx = g4p_boot_mmu_fault(g);
	if (!fctx && time_before(jiffies, g->fq_deadline)) {
		mod_delayed_work(system_wq, &g->timeout_work, msecs_to_jiffies(G4P_FAULT_POLL_MS));
		goto out;
	}
	g4p_boot_stall_dbg(g);
	for (u32 i = g->fq_tail; i != g->fq_head; i++) {
		g4p_boot_job_dbg(g, g->fq_job[i % G4P_FQ]);
		g4p_boot_job_dbg(g, g->fq_job[i % G4P_FQ] + 1);
	}
	g4p_fail_inflight(g, -ETIMEDOUT, -ECANCELED, fctx);
	g4p_recover_locked(g);
out:
	mutex_unlock(&g->lock);
}

/* the fence to wait for before submitting: the oldest if all slots are busy, or (all) the newest; referenced */
static struct dma_fence *g4p_wait_target(struct g4p *g, bool all)
{
	struct dma_fence *f = NULL;

	spin_lock_irq(&g->fence_lock);
	if (g->fq_head - g->fq_tail >= (all ? 1 : G4P_INFLIGHT))
		f = dma_fence_get(g->fq[(all ? g->fq_head - 1 : g->fq_tail) % G4P_FQ]);
	spin_unlock_irq(&g->fence_lock);
	return f;
}

static void g4p_idle_work(struct work_struct *w)
{
	struct g4p *g = container_of(to_delayed_work(w), struct g4p, idle_work);

	mutex_lock(&g->lock);
	g4p_idle_off_locked(g);	/* also releases parked separate-BO memory (g4p_drm_reap) */
	mutex_unlock(&g->lock);
}

/*
 * 8.79: only STREAM records (the CDM control stream, run by the CDM in the file's own context) unless raw_jobs:
 * HEAP writes anywhere in the pool, CMD fields are dereferenced by the firmware in its own context, REG writes
 * MCU_FENCE. Full parsing is g4p_c8_job's; this only checks the record types.
 */
static bool g4p_job_stream_only(const void *img, size_t size)
{
	const u8 *d = img + 16, *end = img + size;

	if (size < 16)
		return false;
	while (d < end) {
		u32 r[3];

		if (end - d < sizeof(r))
			return false;
		memcpy(r, d, sizeof(r));
		if (r[0] != 2 /* G4P_JOB_STREAM */ || r[2] > end - d - sizeof(r))
			return false;
		d += sizeof(r) + ALIGN((size_t)r[2], 4);
	}
	return true;
}

static int g4p_ioctl_submit(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct drm_apple_g4p_submit *a = data;
	void *img;
	int ret;

	bool async = a->flags & APPLE_G4P_SUBMIT_ASYNC;
	struct drm_syncobj *sync = NULL;
	struct dma_fence *prev;

	if ((a->flags & ~APPLE_G4P_SUBMIT_ASYNC) || !a->job_size || a->job_size > SZ_1M)
		return -EINVAL;
	if (a->out_sync) {
		sync = drm_syncobj_find(file, a->out_sync);
		if (!sync)
			return -ENOENT;
	}
	img = vmemdup_user(u64_to_user_ptr(a->job), a->job_size);
	if (IS_ERR(img)) {
		ret = PTR_ERR(img);
		goto out_sync;
	}
	if (!raw_jobs && !g4p_job_stream_only(img, a->job_size)) {
		ret = -EPERM;
		goto out_img;
	}
	/*
	 * 8.69: up to G4P_FQ jobs in flight. An async submit waits only while every slot is busy; a synchronous one
	 * waits for everything in flight (it reads its own result back directly).
	 */
	for (;;) {
		prev = g4p_wait_target(g, !async);
		if (!prev)
			break;
		ret = dma_fence_wait(prev, true);
		dma_fence_put(prev);
		if (ret)
			goto out_img;
	}
	mutex_lock(&g->lock);
	ret = g4p_pool_get_locked(g);
	if (!ret)
		ret = g4p_ensure_booted_locked(g);
	/* R5: the file's address space in a hardware slot of this boot (CAT_BASE[slot] loaded before the kick) */
	while (!ret && (ret = g4p_boot_ctx_bind(g, ((struct g4p_file *)file->driver_priv)->ctx)) == -EAGAIN) {
		/* 8.84: all seven slots have jobs in flight: wait for them, then bind again */
		mutex_unlock(&g->lock);
		ret = 0;
		while (!ret && (prev = g4p_wait_target(g, true))) {
			ret = dma_fence_wait(prev, true);
			dma_fence_put(prev);
		}
		mutex_lock(&g->lock);
		if (!ret)
			ret = g4p_pool_get_locked(g);
		if (!ret)
			ret = g4p_ensure_booted_locked(g);
	}
	g->job_ctx = ((struct g4p_file *)file->driver_priv)->ctx;
	if (!ret && async) {
		struct dma_fence *f = kzalloc(sizeof(*f), GFP_KERNEL);

		if (!f) {
			ret = -ENOMEM;
		} else {
			dma_fence_init(f, &g4p_fence_ops, &g->fence_lock, g->fence_ctx, ++g->fence_seq);
			g->job_img = img;
			g->job_img_size = a->job_size;
			g->job_async = true;
			spin_lock_irq(&g->fence_lock);
			if (g->fq_head - g->fq_tail >= G4P_INFLIGHT) {
				ret = -EBUSY;	/* raced with another submitter: user space retries */
			} else {
				g->fq_job[g->fq_head % G4P_FQ] = g4p_boot_next_job(g);
				g->fq_ctx[g->fq_head % G4P_FQ] = g->job_ctx;
				g->fq[g->fq_head % G4P_FQ] = dma_fence_get(f);	/* before the kick: the IRQ may come */
				g->fq_head++;
			}
			spin_unlock_irq(&g->fence_lock);
			if (!ret)
				ret = g4p_boot_cl_job(g, 8);
			if (!ret)	/* its write-back job, same context (8.78) */
				ret = g4p_flush_unmap_locked(g);
			g->job_async = false;
			g->job_img = NULL;
			if (ret == -EBUSY && g->fq[(g->fq_head - 1) % G4P_FQ] != f) {
				dma_fence_set_error(f, ret);	/* not queued: signal it here */
				dma_fence_signal(f);
			} else if (ret) {
				g4p_fail_inflight(g, ret, ret, 0);	/* queued but never kicked */
			} else {
				g4p_drm_irq(g);	/* very short jobs: already done */
				if (!delayed_work_pending(&g->timeout_work))
					g4p_arm_timeout(g);
			}
			if (sync)
				drm_syncobj_replace_fence(sync, f);
			dma_fence_put(f);
		}
	} else if (!ret) {
		g->job_img = img;
		g->job_img_size = a->job_size;
		ret = g4p_boot_cl_job(g, 8);
		g->job_img = NULL;
		if (ret == 0 || ret == -ETIMEDOUT) {
			g4p_boot_job_result(g, &a->stamp, &a->elapsed_us, a->fault);
			if (!ret)	/* its write-back job, same context (8.78) */
				ret = g4p_flush_unmap_locked(g);
			a->status = ret ? 1 : 0;
			if (ret)
				g4p_recover_locked(g);	/* the next submit boots the firmware again */
			ret = 0;
		}
	}
	g->job_ctx = 0;
	if (idle_ms && g->boot)
		mod_delayed_work(system_wq, &g->idle_work, msecs_to_jiffies(idle_ms));
	mutex_unlock(&g->lock);
out_img:
	kvfree(img);
out_sync:
	if (sync)
		drm_syncobj_put(sync);
	return ret;
}

/*
 * 9.63: a render of another address space than the boot's renders so far, in a boot whose renders began with one
 * process (g4pvk) and continue with another (Zink), faults in the 3D stage (a read of address 0) on the new process's
 * first render; after a fresh boot it does not. Until the firmware state behind it is known, such a switch powers the
 * idle GPU off first (the next submit boots again, ~10 ms).
 */
static bool render_switch_reboot = true;
module_param(render_switch_reboot, bool, 0644);
MODULE_PARM_DESC(render_switch_reboot, "power-cycle the idle GPU when renders switch to another address space (default 1)");

/* 9.09: one render pass of the file's address space, synchronous (g4p_render.c) */
static int g4p_ioctl_submit_render(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct drm_apple_g4p_submit_render *a = data;
	unsigned int ctx = ((struct g4p_file *)file->driver_priv)->ctx;
	void *ta, *td = NULL;
	struct dma_fence *prev;
	int ret;

	if (!raw_jobs)
		return -EPERM;
	if (a->flags)
		return -EINVAL;
	ta = memdup_user(u64_to_user_ptr(a->ta_cmd), APPLE_G4P_TA_CMD_SIZE);
	if (IS_ERR(ta))
		return PTR_ERR(ta);
	td = memdup_user(u64_to_user_ptr(a->td_cmd), APPLE_G4P_3D_CMD_SIZE);
	if (IS_ERR(td)) {
		ret = PTR_ERR(td);
		td = NULL;
		goto out;
	}
	ret = 0;
	while (!ret && (prev = g4p_wait_target(g, true))) {	/* nothing else in flight */
		ret = dma_fence_wait(prev, true);
		dma_fence_put(prev);
	}
	if (ret)
		goto out;
	mutex_lock(&g->lock);
	ret = g4p_pool_get_locked(g);
	if (!ret && render_switch_reboot && g->boot) {
		const unsigned int last = g4p_render_ctx(g);

		if (last && last != ctx) {
			dev_info(g->dev, "renders switch from address space %u to %u: fresh boot\n", last, ctx);
			g4p_idle_off_locked(g);
		}
	}
	if (!ret)
		ret = g4p_ensure_booted_locked(g);
	if (!ret)
		ret = g4p_boot_ctx_bind(g, ctx);
	if (!ret)
		ret = g4p_render_submit(g, ctx, ta, td, &a->status, &a->elapsed_us);
	if (!ret && a->status)
		g4p_recover_locked(g);	/* the next submit boots the firmware again */
	if (idle_ms && g->boot)
		mod_delayed_work(system_wq, &g->idle_work, msecs_to_jiffies(idle_ms));
	mutex_unlock(&g->lock);
out:
	kfree(td);
	kfree(ta);
	return ret;
}

static const struct drm_ioctl_desc g4p_ioctls[] = {
	DRM_IOCTL_DEF_DRV(APPLE_G4P_GET_PARAM, g4p_ioctl_get_param, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_G4P_BO_CREATE, g4p_ioctl_bo_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_G4P_SUBMIT, g4p_ioctl_submit, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_G4P_VM_BIND, g4p_ioctl_vm_bind, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_G4P_SUBMIT_RENDER, g4p_ioctl_submit_render, DRM_RENDER_ALLOW),
};

/*
 * R5 (8.77): each open file gets its own GPU address space 1..G4P_NR_VM - 1 (0 is the kernel's); 8.84: it is
 * loaded into one of the firmware's seven memory context slots when the file submits
 */
static int g4p_drm_open(struct drm_device *drm, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct g4p_file *gf = kzalloc(sizeof(*gf), GFP_KERNEL);
	unsigned int c;

	if (!gf)
		return -ENOMEM;
	mutex_lock(&g->lock);
	c = find_next_zero_bit(&g->ctx_used, G4P_NR_VM, 1);
	if (c < G4P_NR_VM)
		set_bit(c, &g->ctx_used);
	mutex_unlock(&g->lock);
	if (c >= G4P_NR_VM) {
		kfree(gf);
		return -EBUSY;	/* all address spaces in use */
	}
	gf->ctx = c;
	file->driver_priv = gf;
	return 0;
}

/*
 * after the file's handles are gone (drm_gem_release runs first). Buffers can outlive them (CPU mappings):
 * unmap whatever is still mapped in this context now, so that a later owner of the number cannot reach them.
 */
static void g4p_drm_postclose(struct drm_device *drm, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct g4p_file *gf = file->driver_priv;
	struct g4p_gem *bo, *t;
	struct dma_fence *f;
	bool any = false;
	LIST_HEAD(dead);

	while ((f = g4p_wait_target(g, true))) {
		dma_fence_wait(f, false);
		dma_fence_put(f);
	}
	mutex_lock(&g->lock);
	list_for_each_entry_safe(bo, t, &g->sep_bos, node) {
		if (bo->ctx != gf->ctx)
			continue;
		list_del(&bo->node);
		bo->mapped = false;
		g4p_boot_unmap_range(g, bo->ctx, bo->mem.gpuva, bo->mem.size);
		any = true;
	}
	{
		struct g4p_map *m, *mt;

		list_for_each_entry_safe(m, mt, &g->maps, node) {
			if (m->ctx != gf->ctx)
				continue;
			list_move_tail(&m->node, &dead);
			g4p_boot_unmap_range(g, m->ctx, m->va, m->size);
			any = true;
		}
	}
	if (any && g->powered && g->boot && g4p_flush_ctx_locked(g, gf->ctx))
		g4p_recover_locked(g);
	g4p_boot_ctx_detach(g, gf->ctx);
	clear_bit(gf->ctx, &g->ctx_used);
	mutex_unlock(&g->lock);
	{	/* their buffer references: dropped outside the lock (the last one frees the buffer, which locks) */
		struct g4p_map *m, *mt;

		list_for_each_entry_safe(m, mt, &dead, node) {
			drm_gem_object_put(&m->bo->base);
			kfree(m);
		}
	}
	kfree(gf);
}

DEFINE_DRM_GEM_FOPS(g4p_drm_fops);

/*
 * 9.15: PRIME import (a scanout buffer of the display driver as a render target): one contiguous DMA segment,
 * mapped into an address space with VM_BIND; no CPU mapping through this driver
 */
static struct drm_gem_object *g4p_gem_prime_import_sg_table(struct drm_device *drm, struct dma_buf_attachment *attach,
							    struct sg_table *sgt)
{
	struct g4p_gem *bo;
	struct scatterlist *sg;
	dma_addr_t next = 0;
	size_t size = attach->dmabuf->size;
	unsigned int i;

	if (!sgt->nents || !IS_ALIGNED(size, SZ_4K))
		return ERR_PTR(-EINVAL);
	for_each_sgtable_dma_sg(sgt, sg, i) {
		if (i && sg_dma_address(sg) != next)
			return ERR_PTR(-EINVAL);	/* not contiguous in the GPU's DMA space */
		next = sg_dma_address(sg) + sg_dma_len(sg);
	}
	if (next - sg_dma_address(sgt->sgl) < size || !IS_ALIGNED(sg_dma_address(sgt->sgl), SZ_4K))
		return ERR_PTR(-EINVAL);
	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return ERR_PTR(-ENOMEM);
	bo->separate = true;
	bo->sgt = sgt;
	bo->mem.dma = sg_dma_address(sgt->sgl);
	bo->mem.size = size;
	bo->base.funcs = &g4p_gem_funcs;
	drm_gem_private_object_init(drm, &bo->base, size);
	return &bo->base;
}

static const struct drm_driver g4p_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_RENDER | DRIVER_SYNCOBJ,
	.open = g4p_drm_open,
	.postclose = g4p_drm_postclose,
	.gem_prime_import_sg_table = g4p_gem_prime_import_sg_table,
	.ioctls = g4p_ioctls,
	.num_ioctls = ARRAY_SIZE(g4p_ioctls),
	.fops = &g4p_drm_fops,
	.name = "apple-g4p",
	.desc = "Apple A8 GPU (bring-up)",
	.major = 0,
	.minor = 1,
};

struct g4p *g4p_drm_alloc(struct device *dev)
{
	struct g4p *g = devm_drm_dev_alloc(dev, &g4p_drm_driver, struct g4p, drm);

	if (!IS_ERR(g)) {
		INIT_LIST_HEAD(&g->sep_bos);
		INIT_LIST_HEAD(&g->maps);
		INIT_LIST_HEAD(&g->zombies);
		INIT_DELAYED_WORK(&g->idle_work, g4p_idle_work);
		INIT_DELAYED_WORK(&g->timeout_work, g4p_timeout_work);
		spin_lock_init(&g->fence_lock);
		g->fence_ctx = dma_fence_context_alloc(1);
	}
	return g;
}

int g4p_drm_register(struct g4p *g)
{
	return drm_dev_register(&g->drm, 0);
}

void g4p_drm_unregister(struct g4p *g)
{
	drm_dev_unplug(&g->drm);
	cancel_delayed_work_sync(&g->idle_work);
	cancel_delayed_work_sync(&g->timeout_work);
}

/* only once the GPU is powered off: until then it may still access the pool */
void g4p_drm_pool_free(struct g4p *g)
{
	WARN_ON(g->powered);
	g4p_drm_reap(g);
	if (g->pool.cpu)
		dma_free_coherent(g->dev, g->pool.size, g->pool.cpu, g->pool.dma);
	g->pool.cpu = NULL;
	bitmap_free(g->pool_map);
	kfree(g->pool_owner);
	g->pool_map = NULL;
	g->pool_owner = NULL;
}
