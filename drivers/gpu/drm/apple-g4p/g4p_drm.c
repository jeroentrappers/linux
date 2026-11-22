// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 GPU (G4P): DRM render node, bring-up version (linux-on-iphone6 re/GPU-BRINGUP.md 8.64).
 *
 * Compute only. One contiguous pool (module parameter pool_mb) is allocated on first use and mapped at GPU VA
 * 0x300000000 before every firmware boot the DRM path starts, so buffers need no mapping after the boot
 * (post-boot mappings are not reliable yet, 8.34). Buffers are page ranges of the pool; page 0 holds the
 * driver's own PDS programs. SUBMIT runs a job image in the C8 record format (g4p_boot.c g4p_c8_job) on the
 * firmware's compute channel and waits for its completion stamp; jobs follow each other in one firmware boot
 * (8.63). Not yet: VM_BIND, sync objects, a scheduler, runtime PM.
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
#include <uapi/drm/apple_g4p_drm.h>

#include "g4p.h"

static unsigned int idle_ms = 1000;
module_param(idle_ms, uint, 0644);
MODULE_PARM_DESC(idle_ms, "power the GPU off this long after the last DRM submit (0 = stay on); 8.67");

static unsigned int pool_mb = 4;
module_param(pool_mb, uint, 0444);
MODULE_PARM_DESC(pool_mb, "size of the DRM pool in MiB (contiguous, from CMA; mapped at GPU VA 0x300000000)");

struct g4p_gem {
	struct drm_gem_object base;
	unsigned int first, npages;	/* pool pages */
	/* APPLE_G4P_BO_SEPARATE (8.65) */
	bool separate;
	struct g4p_bo mem;		/* own allocation, mapped at mem.gpuva */
	struct list_head node;		/* g->sep_bos */
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
	if (!g->pool_map)
		return -ENOMEM;
	g->pool.cpu = dma_alloc_coherent(g->dev, size, &g->pool.dma, GFP_KERNEL);
	if (!g->pool.cpu) {
		bitmap_free(g->pool_map);
		g->pool_map = NULL;
		return -ENOMEM;
	}
	g->pool.size = size;
	g->pool.gpuva = APPLE_G4P_POOL_VA;
	g->pool_pages = size / SZ_4K;
	set_bit(0, g->pool_map);	/* the driver's PDS programs (terminate at +0x100, data at +0x200) */
	return 0;
}

static void g4p_gem_free(struct drm_gem_object *obj)
{
	struct g4p *g = to_g4p(obj->dev);
	struct g4p_gem *bo = to_g4p_gem(obj);

	mutex_lock(&g->lock);
	if (bo->separate) {
		/*
		 * Unmap, then free. The GPU only runs jobs inside SUBMIT (synchronous, under g->lock), so no job can
		 * reach these pages; the next job's MMU invalidate drops stale translations.
		 */
		list_del(&bo->node);
		g4p_boot_unmap_range(g, bo->mem.gpuva, bo->mem.size);
		/*
		 * 8.65: the per-job MMU invalidate does not drop cached translations (a job wrote through a freed
		 * BO's old VA), so the memory is only released once the GPU is off: park it until then.
		 */
		if (g->powered) {
			struct g4p_zombie *z = kmalloc(sizeof(*z), GFP_KERNEL);

			if (z) {
				z->mem = bo->mem;
				list_add_tail(&z->node, &g->zombies);
			} /* else: leak rather than let the GPU write freed memory */
		} else {
			dma_free_coherent(g->dev, bo->mem.size, bo->mem.cpu, bo->mem.dma);
		}
	} else {
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
	}
	return -EINVAL;
}

/* map every separate BO into a new boot (g4p_boot_start, before the firmware runs) */
int g4p_drm_map_separate(struct g4p *g)
{
	struct g4p_gem *bo;
	int ret = 0;

	list_for_each_entry(bo, &g->sep_bos, node)
		if (!ret)
			ret = g4p_boot_map_range(g, bo->mem.gpuva, bo->mem.dma, bo->mem.size);
	return ret;
}

static int g4p_bo_create_separate(struct g4p *g, struct drm_file *file, struct g4p_gem *bo, unsigned int n,
				  struct drm_apple_g4p_bo_create *a)
{
	size_t size = (size_t)n * SZ_4K;
	int ret;

	bo->separate = true;
	bo->npages = n;	/* mmap size check */
	bo->mem.cpu = dma_alloc_coherent(g->dev, size, &bo->mem.dma, GFP_KERNEL);
	if (!bo->mem.cpu) {
		kfree(bo);
		return -ENOMEM;
	}
	bo->mem.size = size;
	mutex_lock(&g->lock);
	if (!g->sep_next)
		g->sep_next = G4P_SEP_VA;
	bo->mem.gpuva = g->sep_next;
	g->sep_next += ALIGN(size, SZ_64K);	/* a gap between buffers: overruns fault */
	ret = g4p_boot_map_range(g, bo->mem.gpuva, bo->mem.dma, size);	/* post-boot if running */
	if (!ret)
		list_add_tail(&bo->node, &g->sep_bos);
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
	a->gpu_va = bo->mem.gpuva;
	a->mmap_offset = drm_vma_node_offset_addr(&bo->base.vma_node);
	drm_gem_object_put(&bo->base);
	return 0;
}

static int g4p_ioctl_bo_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct drm_apple_g4p_bo_create *a = data;
	struct g4p_gem *bo;
	unsigned long first;
	unsigned int n;
	int ret;

	if ((a->flags & ~APPLE_G4P_BO_SEPARATE) || !a->size || a->size > SZ_64M)
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
		if (first >= g->pool_pages)
			ret = -ENOSPC;
		else
			bitmap_set(g->pool_map, first, n);
	}
	mutex_unlock(&g->lock);
	if (ret) {
		kfree(bo);
		return ret;
	}
	bo->first = first;
	bo->npages = n;
	bo->base.funcs = &g4p_gem_funcs;
	drm_gem_private_object_init(drm, &bo->base, (size_t)n * SZ_4K);
	memset(g->pool.cpu + (size_t)first * SZ_4K, 0, (size_t)n * SZ_4K);
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

static void g4p_idle_work(struct work_struct *w)
{
	struct g4p *g = container_of(to_delayed_work(w), struct g4p, idle_work);

	mutex_lock(&g->lock);
	g4p_idle_off_locked(g);	/* also releases parked separate-BO memory (g4p_drm_reap) */
	mutex_unlock(&g->lock);
}

static int g4p_ioctl_submit(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct drm_apple_g4p_submit *a = data;
	void *img;
	int ret;

	if (a->flags || !a->job_size || a->job_size > SZ_1M)
		return -EINVAL;
	img = vmemdup_user(u64_to_user_ptr(a->job), a->job_size);
	if (IS_ERR(img))
		return PTR_ERR(img);
	mutex_lock(&g->lock);
	ret = g4p_pool_get_locked(g);
	if (!ret)
		ret = g4p_ensure_booted_locked(g);
	if (!ret) {
		g->job_img = img;
		g->job_img_size = a->job_size;
		ret = g4p_boot_cl_job(g, 8);
		g->job_img = NULL;
		if (ret == 0 || ret == -ETIMEDOUT) {
			g4p_boot_job_result(g, &a->stamp, &a->elapsed_us, a->fault);
			a->status = ret ? 1 : 0;
			if (ret)
				g4p_recover_locked(g);	/* the next submit boots the firmware again */
			ret = 0;
		}
	}
	if (idle_ms && g->boot)
		mod_delayed_work(system_wq, &g->idle_work, msecs_to_jiffies(idle_ms));
	mutex_unlock(&g->lock);
	kvfree(img);
	return ret;
}

static const struct drm_ioctl_desc g4p_ioctls[] = {
	DRM_IOCTL_DEF_DRV(APPLE_G4P_GET_PARAM, g4p_ioctl_get_param, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_G4P_BO_CREATE, g4p_ioctl_bo_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_G4P_SUBMIT, g4p_ioctl_submit, DRM_RENDER_ALLOW),
};

DEFINE_DRM_GEM_FOPS(g4p_drm_fops);

static const struct drm_driver g4p_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_RENDER,
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
		INIT_LIST_HEAD(&g->zombies);
		INIT_DELAYED_WORK(&g->idle_work, g4p_idle_work);
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
	g->pool_map = NULL;
}
