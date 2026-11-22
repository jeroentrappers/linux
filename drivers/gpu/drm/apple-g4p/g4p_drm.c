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

static unsigned int pool_mb = 4;
module_param(pool_mb, uint, 0444);
MODULE_PARM_DESC(pool_mb, "size of the DRM pool in MiB (contiguous, from CMA; mapped at GPU VA 0x300000000)");

struct g4p_gem {
	struct drm_gem_object base;
	unsigned int first, npages;	/* pool pages */
};

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
	bitmap_clear(g->pool_map, bo->first, bo->npages);
	mutex_unlock(&g->lock);
	drm_gem_object_release(obj);
	kfree(bo);
}

static int g4p_gem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct g4p *g = to_g4p(obj->dev);
	struct g4p_gem *bo = to_g4p_gem(obj);

	if (vma->vm_end - vma->vm_start > (unsigned long)bo->npages * SZ_4K)
		return -EINVAL;
	/* dma_mmap_coherent() maps the pool from page vm_pgoff: point it at this buffer's pages */
	vma->vm_pgoff = bo->first;
	vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	return dma_mmap_coherent(g->dev, vma, g->pool.cpu, g->pool.dma, g->pool.size);
}

static const struct drm_gem_object_funcs g4p_gem_funcs = {
	.free = g4p_gem_free,
	.mmap = g4p_gem_mmap,
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

static int g4p_ioctl_bo_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct g4p *g = to_g4p(drm);
	struct drm_apple_g4p_bo_create *a = data;
	struct g4p_gem *bo;
	unsigned long first;
	unsigned int n;
	int ret;

	if (a->flags || !a->size || a->size > SZ_64M)
		return -EINVAL;
	n = DIV_ROUND_UP(a->size, SZ_4K);
	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return -ENOMEM;
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
			ret = 0;
		}
	}
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
	return devm_drm_dev_alloc(dev, &g4p_drm_driver, struct g4p, drm);
}

int g4p_drm_register(struct g4p *g)
{
	return drm_dev_register(&g->drm, 0);
}

void g4p_drm_unregister(struct g4p *g)
{
	drm_dev_unplug(&g->drm);
}

/* only once the GPU is powered off: until then it may still access the pool */
void g4p_drm_pool_free(struct g4p *g)
{
	WARN_ON(g->powered);
	if (g->pool.cpu)
		dma_free_coherent(g->dev, g->pool.size, g->pool.cpu, g->pool.dma);
	g->pool.cpu = NULL;
	bitmap_free(g->pool_map);
	g->pool_map = NULL;
}
