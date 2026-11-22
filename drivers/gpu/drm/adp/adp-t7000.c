// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) display pipe ("AppleH7DisplayPipe" in iOS), first draft.
 *
 * Same H7 ADP IP as the M1 Touch Bar pipe driven by adp_drv.c (identical layer, blend and param-FIFO
 * registers), but on A8:
 *  - vblank and "param packet applied" come from the ADP block itself (IRQ enable 0x1028, W1C status
 *    0x102c, AIC IRQ 103), not from the fe block's 0x34/0x100;
 *  - there is no DSI bridge to bind: iBoot leaves the Synopsys DSI host, the panel and the ADBE0 timing
 *    generator running, and this driver never touches them (fixed mode, no modeset);
 *  - the dart-disp0 DART keeps iBoot's page tables. Scanout buffers are carved out of the 12 MB "vram"
 *    region that iBoot already maps for the boot framebuffer, so no IOMMU domain switch happens and the
 *    pipe can never fetch from an unmapped IOVA.
 *
 * Reverse engineering notes: re/DISPLAY.md and re/DISPLAY-PHASE2.md in the iphone6-linux repo.
 *
 * Test mode: with standalone=1 the module creates its own device on the running kernel (no DT node
 * needed, hard-coded A8 addresses, the power domains taken from /chosen/framebuffer) and takes over
 * from simpledrm.
 *
 * Screen off (CRTC off = DPMS off / ACTIVE=0): panel_sleep=1 sends the panel to sleep; deep_off=1
 * first stops the pipe's vram fetch (no layer, ADP IRQs masked) the way iOS does, then sleeps the
 * panel (unless deep_panel=0). The timing generator and the DSI link keep running in both cases.
 * debugfs "panel" and "power" run the same sequences without DRM master. A CRTC on whose panel
 * wake fails is retried in the background; sysfs display_state says "bad" while the layer or the
 * panel is off.
 */

#include <linux/aperture.h>
#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/genalloc.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irqdomain.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_dumb_buffers.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_file.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_prime.h>
#include <linux/dma-buf.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include <video/mipi_display.h>

/* ADP ("be" in adp_drv.c) */
#define ADP_PARAM_STATE		0x101c
#define ADP_PARAM_STATE_PARMST	GENMASK(7, 4)
#define ADP_IRQ_EN		0x1028
#define ADP_IRQ_STATUS		0x102c
#define ADP_IRQ_VBI		BIT(0)
#define ADP_IRQ_UNDERRUN	BIT(1)
#define ADP_IRQ_PKT_MISSED	BIT(3)
#define ADP_IRQ_PF_ALIGN	BIT(6)
#define ADP_IRQ_PF_OVER		BIT(9)
#define ADP_IRQ_PKT_DONE	BIT(10)
#define ADP_IRQ_RDERR		BIT(11)
#define ADP_IRQ_ERRORS		(ADP_IRQ_UNDERRUN | ADP_IRQ_PKT_MISSED | ADP_IRQ_PF_ALIGN | \
				 ADP_IRQ_PF_OVER | ADP_IRQ_RDERR)
#define ADP_FRAME_COUNT		0x1034
/*
 * Layer control. iBoot leaves 0x11001 (bit 0 = pipe 0 on, bit 12 = a mode flag iOS sets from its
 * state, bit 16 = a flag iOS copies from the hardware at start). iOS's "no layer" frame keeps only
 * bit 16 (AppleH7DisplayPipe @0xfffffff0088ebf88), the same value mainline adp_drv.c writes on
 * plane disable (ADBE_LAYER_CTL_ENABLE).
 */
#define ADP_LAYER_CTL		0x1038
#define ADP_LAYER_CTL_KEEP	BIT(16)
#define ADP_FIFO_LEVEL		0x1040
#define ADP_FIFO_LEVEL_MASK	GENMASK(12, 0)
#define ADP_FIFO		0x10c0
/*
 * Param FIFO packet (ADPParamFIFODriver::programFIFO @0xfffffff0088e774c): a header, then per run of
 * ADP registers (count << 16) | offset followed by count values. Registers of other blocks use a
 * 3-word entry 0x40020000, address, value (not used here). The header's word count excludes itself.
 */
#define ADP_FIFO_HDR		BIT(31)
#define ADP_FIFO_HDR_SYNC	BIT(30)		/* apply at the next frame boundary (iOS's normal swap) */
#define ADP_FIFO_HDR_WORDS	GENMASK(28, 16)
#define ADP_FIFO_HDR_TAG	GENMASK(15, 0)
#define ADP_FIFO_SYNC		(ADP_FIFO_HDR | ADP_FIFO_HDR_SYNC)
#define ADP_FIFO_RUN_COUNT	GENMASK(28, 16)
#define ADP_FIFO_RUN_REG	GENMASK(15, 0)

/* generic pipe p (layer) at 0x4000 + 0x2000 * p */
#define PIPE(p)			(0x4000 + 0x2000 * (p))
#define PIPE_PIX_FMT		0x2c
#define PIPE_PIX_FMT_XRGB32	0x53e4001
#define PIPE_PIX_FMT_XRGB32_N61	0x52e4001	/* what iBoot leaves on n61 (bit 20 differs; same pixels on hw) */
#define PIPE_BASE		0x30
#define PIPE_STRIDE		0x38
#define PIPE_SRC_SIZE		0x48

/* ADBE0 ("fe"): read only here */
#define ADBE_VFTG_CTL		0x08	/* bit 31 = run, bit 30 = running (ADBE0Driver_Impl @0xfffffff008907978/a70) */
#define ADBE_SCREEN_SIZE	0x0c
#define ADBE_LINE		0x1c	/* [31:16] frame, [15:0] line (read by iOS at runtime) */

/* A8 constants for standalone=1 (ADT disp0 / dart-disp0 / vram / PMGR) */
#define T7000_ADP_PHYS		0x206200000ULL
#define T7000_ADBE_PHYS		0x206400000ULL
#define T7000_DART_PHYS		0x206304000ULL
#define T7000_PMGR_PS_PAGE	0x20e020000ULL
#define T7000_VRAM_PHYS		0x83e900000ULL
#define T7000_VRAM_SIZE		0xbfb000ULL
#define T7000_ADP_HWIRQ		103

/*
 * Synopsys DSI host (ADT mipi-dsim reg[0], same IP and offsets as adp-mipi.c): the generic packet
 * interface iOS uses for panel commands (AppleSynopsysMIPIDSIController::sendShortCommandGated writes
 * dt | d0 << 8 | d1 << 16 to 0x6c, then waits for GEN_CMD_EMPTY in 0x74). Not in the DT binding yet.
 */
#define T7000_DSI_PHYS		0x206600000ULL
#define DSI_GEN_HDR		0x6c
#define DSI_CMD_PKT_STATUS	0x74
#define DSI_GEN_CMD_EMPTY	BIT(0)
#define DSI_GEN_CMD_FULL	BIT(1)
#define DSI_CMD_TIMEOUT_US	20000

/* MIPI DCS: 120 ms between sleep in and sleep out, and after sleep out before display on */
#define PANEL_SLEEP_MS		120

/*
 * LAYER_CTL is only written through the param FIFO (adpt_layer_ctl_set()), like iOS: the packet is
 * applied at a frame boundary and signalled by IRQ_STATUS bit 10. On hw a direct write killed the pipe
 * (RdErr, frame counter frozen) both at random points (racing flips) and from the VBI interrupt, which
 * on n61 fires at line 541 of 1334, mid-scan (ADBE0 0x24 = 0x1021d). ADPT_LATCH_MS: one more frame
 * after the packet before anything relies on it.
 */
#define ADPT_LATCH_MS		20
#define ADPT_FIFO_IDLE_MS	100	/* iOS waits up to 1000 x 1 ms (@0xfffffff0089085bc) */
#define ADPT_PKT_TIMEOUT_MS	100	/* six frames */

/* stall watchdog: vblank enabled but no VBI for ADPT_WD_MISSES periods = the pipe stopped */
#define ADPT_WD_PERIOD_MS	100
#define ADPT_WD_MISSES		3

#define DART_TCR		0x0c
#define DART_TTBR(sid, i)	(0x40 + 0x10 * (sid) + 4 * (i))
#define DART_TTBR_VALID		BIT(31)
#define DART_PTE_VALID		BIT_ULL(0)
#define DART_PA_MASK		GENMASK_ULL(35, 12)

#define ADP_PITCH_ALIGN		64

/* DRAM window on A8 (1 GB at 0x800000000): DART table pointers outside it are never followed */
#define T7000_DRAM_BASE		0x800000000ULL
#define T7000_DRAM_SIZE		SZ_1G

/*
 * IRQ storm guard, per window: normal traffic is ~6 VBI + ~6 packet-done (+ per-frame error bits) per
 * 100 ms. Past STORM_MASK every source but VBI is masked in ADP_IRQ_EN; past STORM_KILL (status stuck
 * after the W1C ack, a source the enable mask doesn't gate, or the line asserted with status 0) the AIC
 * line is disabled for good and an hrtimer stands in for the vblank interrupt.
 */
#define ADPT_STORM_WINDOW	(HZ / 10 ?: 1)
#define ADPT_STORM_MASK		200
#define ADPT_STORM_KILL		1000

static bool standalone;
module_param(standalone, bool, 0444);
MODULE_PARM_DESC(standalone, "test mode: create the device without a DT node (hard-coded A8 addresses)");
static bool fifo_sync;
module_param(fifo_sync, bool, 0444);
MODULE_PARM_DESC(fifo_sync, "latch layer registers with a param-FIFO sync word (else: direct writes)");
static uint iova_base;
module_param(iova_base, uint, 0444);
MODULE_PARM_DESC(iova_base, "DART IOVA of vram+0 (0 = read pipe 0 base left by iBoot)");
static bool verify_dart = true;
module_param(verify_dart, bool, 0444);
MODULE_PARM_DESC(verify_dart, "walk iBoot's dart-disp0 tables and only use the linearly mapped part of vram");
static bool force;
module_param(force, bool, 0444);
MODULE_PARM_DESC(force, "probe even if the iBoot pipe state looks unexpected");
static bool test_fail;
module_param(test_fail, bool, 0444);
MODULE_PARM_DESC(test_fail, "test: fail the probe at its last step, after all setup (simpledrm must stay)");
static bool test_noack;
module_param(test_noack, bool, 0444);
MODULE_PARM_DESC(test_noack, "test: never ack ADP IRQs (forces a storm on the first enabled bit, exercises the guard)");
static bool panel_sleep;
module_param(panel_sleep, bool, 0644);
MODULE_PARM_DESC(panel_sleep, "CRTC off (DPMS off / ACTIVE=0) also puts the panel to sleep: DCS display off + sleep in (default: off)");
static bool deep_off;
module_param(deep_off, bool, 0644);
MODULE_PARM_DESC(deep_off, "CRTC off also stops the pipe's vram fetch (no layer, ADP IRQs masked) before the panel sleep; implies panel_sleep unless deep_panel=0 (default: off)");
static bool deep_panel = true;
module_param(deep_panel, bool, 0644);
MODULE_PARM_DESC(deep_panel, "deep_off also puts the panel to sleep (default: on; 0 = layer off only, the panel stays awake behind backlight 0)");

struct adpt {
	struct drm_device drm;
	struct drm_plane primary;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct drm_display_mode mode;

	void __iomem *adp;
	void __iomem *adbe;
	int irq;
	struct dev_pm_domain_list *pds;

	phys_addr_t vram_phys;
	size_t vram_size;	/* usable part */
	void *vram;		/* WC kernel mapping */
	u32 vram_iova;
	struct gen_pool *pool;

	spinlock_t irq_lock;	/* ADP_IRQ_EN read-modify-write */
	u32 irq_en_orig;
	struct {
		u32 base, stride;
	} boot;			/* iBoot state, restored on remove */

	u32 pitch;
	u16 tag;
	u64 flush_seq;		/* vblank count at the last flush */
	struct drm_pending_vblank_event *event;
	/*
	 * direct mode: the new scanout base waits for the next VBI (written from the IRQ, so the switch happens in
	 * blanking, not at the scan line: no tearing); base_seq = the vblank count it was written at. Under event_lock.
	 */
	u32 pending_base;
	bool base_pending;
	u64 base_seq;
	unsigned long storm_jiffies;
	unsigned int storm_count;
	bool irq_dead;		/* storm guard disabled the line; vbl_timer fakes vblanks */
	struct hrtimer vbl_timer;
	ktime_t vbl_period;

	void __iomem *dsi;	/* NULL: no panel commands */
	struct mutex panel_lock;	/* panel and scanout power state below */
	bool panel_asleep;	/* we sent sleep in; only then is sleep out sent */
	bool panel_keep;	/* remove/shutdown: CRTC off must not touch the panel or the pipe */
	ktime_t panel_sleep_at;
	bool scanout_off;	/* we cleared the layer: only then is layer_ctl written back */
	u32 layer_ctl;		/* ADP_LAYER_CTL before scanout off */
	u32 irq_en_off;		/* non-VBI ADP_IRQ_EN bits masked by scanout off */

	/* param FIFO packet in flight (pkt_wait/pkt_done under irq_lock) and what the last one did */
	bool pkt_wait;
	bool pkt_done;		/* IRQ_STATUS bit 10 came while pkt_wait */
	u32 pkt_send_line;	/* ADBE0 0x1c (frame << 16 | line) when the packet was written */
	u32 pkt_done_line;	/* ... when the packet-done IRQ came */
	u32 pkt_hdr;
	int pkt_ret;
	unsigned int pkt_ms;
	wait_queue_head_t layer_wq;

	/* VBI watchdog (work on system_freezable_wq: frozen across suspend with the IRQs) */
	struct delayed_work vbl_wd;
	unsigned int vbi_count;	/* real VBIs seen by the IRQ handler */
	unsigned int wd_last;	/* the work's own; enable_vblank only sets wd_reset */
	unsigned int wd_misses;
	bool wd_reset;
	bool wd_trip;		/* debugfs "stall trip": declare a stall on the next run */
	bool vbl_on;		/* DRM vblank enabled (also gates the fallback hrtimer) */
	bool stalled;		/* the watchdog gave up on the pipe (debugfs "stall recover" undoes it) */
	bool storm_killed;	/* the storm guard gave up on the line: permanent */
	bool stall_irq_off;	/* adpt_pipe_stalled() did disable_irq() */
	u32 irq_en_stall;	/* ADP_IRQ_EN when the stall was declared */

	/*
	 * CRTC on but the screen isn't (the layer packet or the panel wake failed): atomic_enable can't
	 * return an error, so the heal work keeps retrying while the CRTC stays on, and display_state
	 * (sysfs) tells userspace, which must not light the backlight over a black screen meanwhile.
	 */
	bool crtc_on;		/* between atomic_enable and atomic_disable (panel_lock) */
	bool dbg_off;		/* debugfs "power off" switched the screen off on purpose: no healing */
	struct delayed_work heal;
	unsigned int heal_tries;
	unsigned int n_off, n_on, n_fail;	/* CRTC off/on transitions, failed CRTC ons */
	int last_err;		/* first failing step of the last on/off: 0, or -errno */
	const char *last_step;	/* which step that was */
};

#define to_adpt(x) container_of(x, struct adpt, drm)

/* ---------------------------------------------------------------- GEM: vram carveout objects */

struct adpt_bo {
	struct drm_gem_object base;
	unsigned long vaddr;
	phys_addr_t phys;
	u32 iova;
};

#define to_bo(x) container_of(x, struct adpt_bo, base)

static void adpt_bo_free(struct drm_gem_object *obj)
{
	struct adpt *adp = to_adpt(obj->dev);
	struct adpt_bo *bo = to_bo(obj);

	drm_gem_free_mmap_offset(obj);
	drm_gem_object_release(obj);
	gen_pool_free(adp->pool, bo->vaddr, obj->size);
	kfree(bo);
}

static int adpt_bo_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct adpt_bo *bo = to_bo(obj);
	unsigned long pages = vma_pages(vma);

	/* drop the fake offset: from here vm_pgoff is the offset into the object */
	vma->vm_pgoff -= drm_vma_node_start(&obj->vma_node);
	if (vma->vm_pgoff + pages > obj->size >> PAGE_SHIFT)
		return -EINVAL;

	vm_flags_set(vma, VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
	return remap_pfn_range(vma, vma->vm_start, (bo->phys >> PAGE_SHIFT) + vma->vm_pgoff,
			       pages << PAGE_SHIFT, vma->vm_page_prot);
}

static const struct vm_operations_struct adpt_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

/*
 * PRIME export of a carveout buffer (for the GPU to render into it, linux-on-iphone6 re/GPU-BRINGUP.md 9.15): the
 * carveout has no struct pages for the generic helpers, so the attachment gets one DMA range = the physical
 * address (importers without an IOMMU between them and memory, the A8 GPU's own MMU maps it).
 */
static struct sg_table *adpt_map_dma_buf(struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct adpt_bo *bo = to_bo((struct drm_gem_object *)attach->dmabuf->priv);
	struct sg_table *sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);

	if (!sgt)
		return ERR_PTR(-ENOMEM);
	if (sg_alloc_table(sgt, 1, GFP_KERNEL)) {
		kfree(sgt);
		return ERR_PTR(-ENOMEM);
	}
	sg_dma_address(sgt->sgl) = bo->phys;
	sg_dma_len(sgt->sgl) = bo->base.size;
	return sgt;
}

static void adpt_unmap_dma_buf(struct dma_buf_attachment *attach, struct sg_table *sgt, enum dma_data_direction dir)
{
	sg_free_table(sgt);
	kfree(sgt);
}

static const struct dma_buf_ops adpt_dmabuf_ops = {
	.map_dma_buf = adpt_map_dma_buf,
	.unmap_dma_buf = adpt_unmap_dma_buf,
	.release = drm_gem_dmabuf_release,
	.mmap = drm_gem_dmabuf_mmap,
};

static struct dma_buf *adpt_bo_export(struct drm_gem_object *obj, int flags)
{
	struct dma_buf_export_info exp = {
		.exp_name = KBUILD_MODNAME,
		.owner = obj->dev->driver->fops->owner,
		.ops = &adpt_dmabuf_ops,
		.size = obj->size,
		.flags = flags,
		.priv = obj,
		.resv = obj->resv,
	};

	return drm_gem_dmabuf_export(obj->dev, &exp);
}

static const struct drm_gem_object_funcs adpt_bo_funcs = {
	.free = adpt_bo_free,
	.export = adpt_bo_export,
	.mmap = adpt_bo_mmap,
	.vm_ops = &adpt_vm_ops,
};

static struct adpt_bo *adpt_bo_create(struct adpt *adp, size_t size)
{
	struct adpt_bo *bo;
	int ret;

	size = PAGE_ALIGN(size);
	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return ERR_PTR(-ENOMEM);

	bo->vaddr = gen_pool_alloc(adp->pool, size);
	if (!bo->vaddr) {
		kfree(bo);
		return ERR_PTR(-ENOMEM);
	}
	bo->phys = gen_pool_virt_to_phys(adp->pool, bo->vaddr);
	bo->iova = adp->vram_iova + (u32)(bo->phys - adp->vram_phys);
	memset((void *)bo->vaddr, 0, size);

	drm_gem_private_object_init(&adp->drm, &bo->base, size);
	bo->base.funcs = &adpt_bo_funcs;
	ret = drm_gem_create_mmap_offset(&bo->base);
	if (ret) {
		drm_gem_object_put(&bo->base);
		return ERR_PTR(ret);
	}
	return bo;
}

static int adpt_dumb_create(struct drm_file *file, struct drm_device *drm,
			    struct drm_mode_create_dumb *args)
{
	struct adpt *adp = to_adpt(drm);
	struct adpt_bo *bo;
	int ret;

	ret = drm_mode_size_dumb(drm, args, ADP_PITCH_ALIGN, PAGE_SIZE);
	if (ret)
		return ret;

	bo = adpt_bo_create(adp, args->size);
	if (IS_ERR(bo))
		return PTR_ERR(bo);

	ret = drm_gem_handle_create(file, &bo->base, &args->handle);
	drm_gem_object_put(&bo->base);
	return ret;
}

/*
 * PRIME import of our own dma-bufs (another open file of this device, e.g. a GPU client's scanout buffer coming
 * back to KMS): the core's self-import only knows the generic GEM dma-buf ops, ours are custom
 */
static struct drm_gem_object *adpt_gem_prime_import(struct drm_device *drm, struct dma_buf *dma_buf)
{
	if (dma_buf->ops == &adpt_dmabuf_ops) {
		struct drm_gem_object *obj = dma_buf->priv;

		if (obj->dev == drm) {
			drm_gem_object_get(obj);
			return obj;
		}
	}
	return ERR_PTR(-EOPNOTSUPP);
}

DEFINE_DRM_GEM_FOPS(adpt_fops);

static const struct drm_driver adpt_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &adpt_fops,
	.dumb_create = adpt_dumb_create,
	.gem_prime_import = adpt_gem_prime_import,
	.name = "adp-t7000",
	.desc = "Apple A8 display pipe",
	.major = 0,
	.minor = 1,
};

/* ---------------------------------------------------------------- param FIFO */

/* latch shadowed registers with a sync word; never polls: a busy FIFO takes the values with the packet in flight */
static void adpt_fifo_kick(struct adpt *adp)
{
	u32 st = readl(adp->adp + ADP_PARAM_STATE), lvl = readl(adp->adp + ADP_FIFO_LEVEL);

	if (FIELD_GET(ADP_PARAM_STATE_PARMST, st) == 2 || (lvl & ADP_FIFO_LEVEL_MASK))
		drm_err_ratelimited(&adp->drm, "param FIFO busy (%08x/%08x), no sync\n", st, lvl);
	else
		writel(ADP_FIFO_SYNC | (++adp->tag ?: ++adp->tag), adp->adp + ADP_FIFO);
}

/* ---------------------------------------------------------------- IRQ */

static void adpt_irq_en(struct adpt *adp, u32 set, u32 clear)
{
	unsigned long flags;

	spin_lock_irqsave(&adp->irq_lock, flags);
	if (adp->irq_dead)
		set = 0;	/* the line is off for good: keep the block quiet too */
	writel((readl(adp->adp + ADP_IRQ_EN) & ~clear) | set, adp->adp + ADP_IRQ_EN);
	spin_unlock_irqrestore(&adp->irq_lock, flags);
}

/* vblank accounting + flip completion for status bits @st (from the IRQ or from vbl_timer) */
static void adpt_handle_events(struct adpt *adp, u32 st)
{
	struct drm_crtc *crtc = &adp->crtc;

	if (st & ADP_IRQ_VBI)
		drm_crtc_handle_vblank(crtc);

	spin_lock(&adp->drm.event_lock);
	if ((st & ADP_IRQ_VBI) && adp->base_pending) {
		writel(adp->pending_base, adp->adp + PIPE(0) + PIPE_BASE);
		adp->base_pending = false;
		adp->base_seq = drm_crtc_vblank_count(crtc);
	}
	if (adp->event) {
		bool done;

		u64 seq = drm_crtc_vblank_count(crtc);

		if (fifo_sync)	/* packet applied; two VBIs without one = give up waiting */
			done = (st & ADP_IRQ_PKT_DONE) || seq > adp->flush_seq + 1;
		else	/*
			 * the base is written in this VBI's blanking: the old buffer is no longer scanned out once
			 * the frame starts, so complete now (one VBI later halves the rate of a double-buffered
			 * swapchain: 30 fps)
			 */
			done = (st & ADP_IRQ_VBI) && seq > adp->flush_seq && !adp->base_pending;
		if (done) {
			drm_crtc_send_vblank_event(crtc, adp->event);
			adp->event = NULL;
			drm_crtc_vblank_put(crtc);
		}
	}
	spin_unlock(&adp->drm.event_lock);
}

/*
 * Stand-in for IRQ 103 once the storm guard or the stall watchdog took it away: flips still complete,
 * nothing waits for timeouts. Runs only while DRM vblank is enabled (enable_vblank restarts it), so a
 * dead line costs no 60 Hz wakeups with the screen off. Never cancelled from disable_vblank: that runs
 * under vbl_lock, which this callback can reach through drm_vblank_put().
 */
static enum hrtimer_restart adpt_vbl_timer(struct hrtimer *t)
{
	struct adpt *adp = container_of(t, struct adpt, vbl_timer);

	if (!READ_ONCE(adp->vbl_on))
		return HRTIMER_NORESTART;
	adpt_handle_events(adp, ADP_IRQ_VBI | ADP_IRQ_PKT_DONE);
	hrtimer_forward_now(t, adp->vbl_period);
	return HRTIMER_RESTART;
}

static void adpt_vbl_timer_cancel(void *arg)
{
	hrtimer_cancel(&((struct adpt *)arg)->vbl_timer);
}

/* true when the guard decided to stop taking this interrupt */
static bool adpt_storm_guard(struct adpt *adp, u32 st)
{
	if (adp->irq_dead)
		return true;
	if (time_after(jiffies, adp->storm_jiffies + ADPT_STORM_WINDOW)) {
		adp->storm_jiffies = jiffies;
		adp->storm_count = 0;
	}
	adp->storm_count++;
	if (adp->storm_count == ADPT_STORM_MASK + 1) {
		/* a stuck non-VBI status bit: keep VBI only (flips fall back to the VBI watchdog) */
		adpt_irq_en(adp, 0, (u32)~ADP_IRQ_VBI);
		drm_err_ratelimited(&adp->drm, "IRQ storm, status %08x: masked all but VBI\n", st);
	}
	if (adp->storm_count <= ADPT_STORM_KILL)
		return false;

	/*
	 * Still storming with only VBI enabled (or nothing we can mask): never let it take the phone
	 * down. Mask the block, turn the AIC line off and fake vblanks from a timer from now on.
	 */
	adp->irq_dead = true;
	adp->storm_killed = true;
	adpt_irq_en(adp, 0, ~0u);
	disable_irq_nosync(adp->irq);
	hrtimer_start(&adp->vbl_timer, adp->vbl_period, HRTIMER_MODE_REL_HARD);
	drm_err_ratelimited(&adp->drm,
			    "IRQ %d storm (%u in %u ms, status %08x): line disabled, timer vblank from now on\n",
			    adp->irq, adp->storm_count, jiffies_to_msecs(ADPT_STORM_WINDOW), st);
	return true;
}

static irqreturn_t adpt_irq(int irq, void *arg)
{
	struct adpt *adp = arg;
	u32 st;

	st = readl(adp->adp + ADP_IRQ_STATUS);
	/* counted before the !st check: a line asserted with a zero status is a storm too */
	if (adpt_storm_guard(adp, st))
		return IRQ_HANDLED;
	if (!st)
		return IRQ_NONE;
	if (!test_noack)
		writel(st, adp->adp + ADP_IRQ_STATUS);	/* W1C, as iOS does */

	if (st & ADP_IRQ_PKT_DONE) {
		bool done = false;

		spin_lock(&adp->irq_lock);
		if (adp->pkt_wait) {
			adp->pkt_done_line = readl(adp->adbe + ADBE_LINE);
			adp->pkt_wait = false;
			adp->pkt_done = true;
			done = true;
		}
		spin_unlock(&adp->irq_lock);
		if (done)
			wake_up(&adp->layer_wq);
	}
	if (st & ADP_IRQ_VBI)
		WRITE_ONCE(adp->vbi_count, adp->vbi_count + 1);

	if (st & ADP_IRQ_ERRORS)
		drm_err_ratelimited(&adp->drm, "ADP error status %08x%s%s%s%s%s\n", st,
				    st & ADP_IRQ_UNDERRUN ? " underrun" : "",
				    st & ADP_IRQ_PKT_MISSED ? " packet-missed" : "",
				    st & ADP_IRQ_PF_ALIGN ? " fifo-align" : "",
				    st & ADP_IRQ_PF_OVER ? " fifo-overflow" : "",
				    st & ADP_IRQ_RDERR ? " read-error" : "");

	adpt_handle_events(adp, st);
	return IRQ_HANDLED;
}

/*
 * The pipe stopped: vblank is enabled but no VBI came for ADPT_WD_MISSES * ADPT_WD_PERIOD_MS. Seen on
 * hw after racing LAYER_CTL writes: frame counter frozen, IRQ_STATUS RdErr (bit 11), VFTG still
 * "running". iOS has no recovery either: RdErr, PFAlign, PFOver and (unless disabled by boot-arg)
 * underrun are "fatal error" in AppleH7DisplayPipe's interrupt filter (@0xfffffff0088e8b04, mask 0xa42)
 * and it panics (@0xfffffff0088e8e8c), i.e. the pipe only comes back through a full re-init. Here: say
 * so once, and switch to the storm guard's fallback (line off, 60 Hz hrtimer vblank) so flips, vblank
 * waits and DPMS keep completing instead of blocking for seconds. The screen needs a reboot.
 * @test: debugfs "stall trip" on a healthy pipe (undone with "stall recover").
 */
static void adpt_pipe_stalled(struct adpt *adp, bool test)
{
	unsigned long flags;

	spin_lock_irqsave(&adp->irq_lock, flags);
	if (adp->irq_dead) {
		spin_unlock_irqrestore(&adp->irq_lock, flags);
		return;
	}
	adp->irq_dead = true;
	adp->stalled = true;
	adp->pkt_wait = false;
	adp->irq_en_stall = readl(adp->adp + ADP_IRQ_EN);
	writel(0, adp->adp + ADP_IRQ_EN);
	spin_unlock_irqrestore(&adp->irq_lock, flags);
	wake_up(&adp->layer_wq);

	disable_irq(adp->irq);
	adp->stall_irq_off = true;
	hrtimer_start(&adp->vbl_timer, adp->vbl_period, HRTIMER_MODE_REL_HARD);	/* stops itself if vblank is off */
	drm_err(&adp->drm,
		"display pipe stalled%s: no vblank for %u ms (IRQ_STATUS %08x, frame %04x, VFTG %08x, LAYER_CTL %08x); timer vblank from now on, reboot to get the screen back\n",
		test ? " (TEST, debugfs stall trip)" : "",
		ADPT_WD_MISSES * ADPT_WD_PERIOD_MS, readl(adp->adp + ADP_IRQ_STATUS),
		readl(adp->adp + ADP_FRAME_COUNT) & 0xffff, readl(adp->adbe + ADBE_VFTG_CTL),
		readl(adp->adp + ADP_LAYER_CTL));
}

static void adpt_vbl_wd(struct work_struct *work)
{
	struct adpt *adp = container_of(to_delayed_work(work), struct adpt, vbl_wd);
	unsigned int n = READ_ONCE(adp->vbi_count);

	if (READ_ONCE(adp->wd_trip)) {
		WRITE_ONCE(adp->wd_trip, false);
		adpt_pipe_stalled(adp, true);
		return;
	}
	if (!READ_ONCE(adp->vbl_on) || READ_ONCE(adp->irq_dead))
		return;
	if (READ_ONCE(adp->wd_reset)) {
		WRITE_ONCE(adp->wd_reset, false);
		adp->wd_last = n;
		adp->wd_misses = 0;
	} else if (n != adp->wd_last) {
		adp->wd_last = n;
		adp->wd_misses = 0;
	} else if (++adp->wd_misses >= ADPT_WD_MISSES) {
		adpt_pipe_stalled(adp, false);
		return;
	}
	queue_delayed_work(system_freezable_wq, &adp->vbl_wd, msecs_to_jiffies(ADPT_WD_PERIOD_MS));
}

static void adpt_vbl_wd_cancel(void *arg)
{
	cancel_delayed_work_sync(&((struct adpt *)arg)->vbl_wd);
}

/* ---------------------------------------------------------------- panel: DCS over the DSI host */

/* the first step of the current display off/on that failed, for display_state (caller holds panel_lock) */
static void adpt_fail(struct adpt *adp, const char *step, int err)
{
	if (!adp->last_err) {
		adp->last_err = err ?: -EIO;
		adp->last_step = step;
	}
}

/* one DCS short write without parameter, on virtual channel 0 */
static int adpt_dcs(struct adpt *adp, u8 cmd)
{
	u32 st;
	int ret;

	ret = readl_poll_timeout(adp->dsi + DSI_CMD_PKT_STATUS, st, !(st & DSI_GEN_CMD_FULL), 100,
				 DSI_CMD_TIMEOUT_US);
	if (!ret) {
		writel(MIPI_DSI_DCS_SHORT_WRITE | (cmd << 8), adp->dsi + DSI_GEN_HDR);
		ret = readl_poll_timeout(adp->dsi + DSI_CMD_PKT_STATUS, st, st & DSI_GEN_CMD_EMPTY, 100,
					 DSI_CMD_TIMEOUT_US);
	}
	if (ret)
		drm_err(&adp->drm, "DSI: DCS %02x not sent (status %08x)\n", cmd, st);
	return ret;
}

/*
 * What ApplePinotLCD does first on display off (display off, then sleep in). iOS then also stops the
 * DSI HS clock, resets the panel and cuts its LDO; none of that is done here: the DSI link, the timing
 * generator and the pipe keep running, so waking needs no panel init (sleep out + display on).
 * Called with panel_lock held and inside drm_dev_enter().
 */
static void adpt_panel_sleep_locked(struct adpt *adp)
{
	if (!adp->dsi || adp->panel_asleep)
		return;
	if (adpt_dcs(adp, MIPI_DCS_SET_DISPLAY_OFF)) {
		adpt_fail(adp, "panel sleep", -EIO);
		return;
	}
	msleep(20);	/* a frame */
	if (!adpt_dcs(adp, MIPI_DCS_ENTER_SLEEP_MODE)) {
		adp->panel_asleep = true;
		adp->panel_sleep_at = ktime_get();
	} else {
		adpt_fail(adp, "panel sleep", -EIO);
		adpt_dcs(adp, MIPI_DCS_SET_DISPLAY_ON);
	}
}

static void adpt_panel_wake_locked(struct adpt *adp)
{
	s64 ms;

	if (!adp->panel_asleep)
		return;
	ms = ktime_ms_delta(ktime_get(), adp->panel_sleep_at);
	if (ms < PANEL_SLEEP_MS)
		msleep(PANEL_SLEEP_MS - ms);
	if (!adpt_dcs(adp, MIPI_DCS_EXIT_SLEEP_MODE)) {
		msleep(PANEL_SLEEP_MS);
		if (!adpt_dcs(adp, MIPI_DCS_SET_DISPLAY_ON))
			adp->panel_asleep = false;
	}
	if (adp->panel_asleep) {
		adpt_fail(adp, "panel wake", -EIO);
		drm_err(&adp->drm, "panel did not take the wake-up commands\n");
	}
}

/* iOS's idle test (ADPParamFIFODriver @0xfffffff0088e7670): level 0x1040[12:0] == 0, ParmSt != 2 (and <= 5) */
static bool adpt_fifo_idle(struct adpt *adp)
{
	u32 ps = FIELD_GET(ADP_PARAM_STATE_PARMST, readl(adp->adp + ADP_PARAM_STATE));

	return !(readl(adp->adp + ADP_FIFO_LEVEL) & ADP_FIFO_LEVEL_MASK) && ps != 2 && ps <= 5;
}

/*
 * Send one param-FIFO packet of @n words (runs of ADP registers, see ADP_FIFO_RUN_*) with the sync
 * flag, as iOS's programFifoPacket_gated (@0xfffffff0088ead14 -> 0xfffffff0088e774c) does, and wait for
 * the packet-done interrupt. Waits for the FIFO to be idle first (1 ms sleeps, never a spin). Caller
 * holds panel_lock (one packet at a time). Needs the IRQ line: with the line off (storm guard / stall)
 * nothing could report the packet, and a stalled pipe takes none.
 */
static int adpt_fifo_packet(struct adpt *adp, const u32 *words, unsigned int n)
{
	ktime_t t0 = ktime_get();
	unsigned long flags;
	bool en_had, done;
	unsigned int i;

	if (READ_ONCE(adp->irq_dead))
		return -EIO;
	while (!adpt_fifo_idle(adp)) {
		if (ktime_ms_delta(ktime_get(), t0) > ADPT_FIFO_IDLE_MS) {
			drm_err(&adp->drm, "param FIFO not idle (state %08x level %08x): packet not sent\n",
				readl(adp->adp + ADP_PARAM_STATE), readl(adp->adp + ADP_FIFO_LEVEL));
			return -EBUSY;
		}
		usleep_range(1000, 2000);
	}

	spin_lock_irqsave(&adp->irq_lock, flags);
	en_had = readl(adp->adp + ADP_IRQ_EN) & ADP_IRQ_PKT_DONE;
	writel(ADP_IRQ_PKT_DONE, adp->adp + ADP_IRQ_STATUS);	/* a stale bit would complete us at once */
	writel(readl(adp->adp + ADP_IRQ_EN) | ADP_IRQ_PKT_DONE, adp->adp + ADP_IRQ_EN);
	adp->pkt_done = false;
	adp->pkt_wait = true;
	adp->pkt_hdr = ADP_FIFO_SYNC | FIELD_PREP(ADP_FIFO_HDR_WORDS, n) |
		       FIELD_PREP(ADP_FIFO_HDR_TAG, ++adp->tag ?: ++adp->tag);
	adp->pkt_send_line = readl(adp->adbe + ADBE_LINE);
	writel(adp->pkt_hdr, adp->adp + ADP_FIFO);
	for (i = 0; i < n; i++)
		writel(words[i], adp->adp + ADP_FIFO);
	spin_unlock_irqrestore(&adp->irq_lock, flags);
	t0 = ktime_get();

	wait_event_timeout(adp->layer_wq, !READ_ONCE(adp->pkt_wait), msecs_to_jiffies(ADPT_PKT_TIMEOUT_MS));

	spin_lock_irqsave(&adp->irq_lock, flags);
	done = adp->pkt_done;
	adp->pkt_wait = false;
	if (!en_had && !adp->irq_dead)
		writel(readl(adp->adp + ADP_IRQ_EN) & ~ADP_IRQ_PKT_DONE, adp->adp + ADP_IRQ_EN);
	spin_unlock_irqrestore(&adp->irq_lock, flags);
	adp->pkt_ms = ktime_ms_delta(ktime_get(), t0);
	adp->pkt_ret = done ? 0 : -ETIMEDOUT;
	return adp->pkt_ret;
}

/*
 * LAYER_CTL = @val through the param FIFO (applied at a frame boundary), then one more frame. Without
 * a packet-done IRQ the register is read back after another 100 ms: if it holds @val the packet did
 * land (the IRQ is what's missing), otherwise it did not (or not yet); either way the caller's state
 * follows the hardware. Never a direct write: that is what kills the pipe.
 */
static int adpt_layer_ctl_set(struct adpt *adp, u32 val)
{
	const u32 pkt[] = { FIELD_PREP(ADP_FIFO_RUN_COUNT, 1) | ADP_LAYER_CTL, val };
	int ret;

	ret = adpt_fifo_packet(adp, pkt, ARRAY_SIZE(pkt));
	if (ret == -ETIMEDOUT) {
		msleep(ADPT_PKT_TIMEOUT_MS);
		if (readl(adp->adp + ADP_LAYER_CTL) == val) {
			drm_warn(&adp->drm, "LAYER_CTL packet: no packet-done IRQ, but it reads back %08x\n", val);
			ret = 0;
		}
	}
	if (ret)
		return ret;
	drm_dbg_driver(&adp->drm, "LAYER_CTL %08x: packet sent at %08x, done at %08x (ADBE f<<16|line)\n",
		       val, adp->pkt_send_line, adp->pkt_done_line);
	msleep(ADPT_LATCH_MS);
	return 0;
}

/*
 * Stop the pipe's vram fetch, so DRAM can drop into self-refresh while the screen is off. This is
 * the first thing iOS does on display off (AppleMobileADBE0::do_power_state_change_gated, power-off
 * branch @0xfffffff0088f5198 -> vtable 0xc00 = 0xfffffff0088ee678): with the timing generator still
 * running it masks every ADP interrupt (0x1028 &= ~0xfff), queues a frame with no layer
 * (0x1038 = bit 16 only, @0xfffffff0088ebf88) and waits until the frame has latched. Only after
 * that come backlight off, panel off, timing generator stop and the power gate.
 *
 * Here VBI is left to the DRM vblank code (already off on the CRTC disable path; on the debugfs path
 * flips keep completing). Nothing else changes: base, stride, format, blend and the timing
 * generator stay as they are, so writing the saved LAYER_CTL back is the whole undo.
 *
 * Not done: iOS stops the timing generator (ADBE0 0x08 bit 31, then waits for bit 30) only after the
 * panel and the DSI link are down, and power-gates the pipe right after; it never restarts timings
 * under a DSI link left in video mode. Whether that restart works without a DSI/panel re-init is not
 * known, so VFTG, the DSI link (no ULPS on n61 either) and the power domains stay on.
 * Called with panel_lock held and inside drm_dev_enter().
 */
static void adpt_scanout_off_locked(struct adpt *adp)
{
	unsigned long flags;
	u32 ctl;
	int ret;

	if (adp->scanout_off)
		return;

	spin_lock_irqsave(&adp->irq_lock, flags);
	adp->irq_en_off = readl(adp->adp + ADP_IRQ_EN) & ~ADP_IRQ_VBI;
	writel(readl(adp->adp + ADP_IRQ_EN) & ADP_IRQ_VBI, adp->adp + ADP_IRQ_EN);
	spin_unlock_irqrestore(&adp->irq_lock, flags);

	ctl = readl(adp->adp + ADP_LAYER_CTL);
	adp->layer_ctl = ctl;	/* also for a late-landing packet, see adpt_scanout_on_locked() */
	ret = adpt_layer_ctl_set(adp, ctl & ADP_LAYER_CTL_KEEP);
	if (ret) {
		/* layer untouched: unmask again and leave the pipe scanning */
		adpt_irq_en(adp, adp->irq_en_off, 0);
		adpt_fail(adp, "layer off", ret);
		drm_warn(&adp->drm, "scanout off skipped: layer packet not applied (%d)\n", ret);
		return;
	}
	adp->scanout_off = true;
	drm_dbg_driver(&adp->drm, "scanout off: LAYER_CTL %08x -> %08x, IRQ_EN masked %08x\n", ctl,
		       readl(adp->adp + ADP_LAYER_CTL), adp->irq_en_off);
}

static void adpt_scanout_on_locked(struct adpt *adp)
{
	u32 off = adp->layer_ctl & ADP_LAYER_CTL_KEEP;
	int ret;

	if (!adp->scanout_off) {
		/* an off packet that timed out but landed later: the layer is off although we gave up */
		if (!adp->layer_ctl || adp->layer_ctl == off || readl(adp->adp + ADP_LAYER_CTL) != off)
			return;
		drm_warn(&adp->drm, "late layer-off packet found (LAYER_CTL %08x): turning the layer back on\n", off);
		adp->irq_en_off = 0;
		adp->scanout_off = true;
	}

	/* the first frame the panel shows after wake has the layer */
	ret = adpt_layer_ctl_set(adp, adp->layer_ctl);
	if (ret) {
		/*
		 * Still off (a direct write is what kills the pipe): stay in the off state, so the next
		 * CRTC on or debugfs "power on" tries again. The screen is black meanwhile.
		 */
		adpt_fail(adp, "layer on", ret);
		drm_err(&adp->drm, "scanout on failed: layer packet not applied (%d), screen stays black\n", ret);
		return;
	}
	/* drop what latched while masked (W1C, as iOS), then unmask what was enabled before */
	if (!test_noack)
		writel(readl(adp->adp + ADP_IRQ_STATUS) & adp->irq_en_off, adp->adp + ADP_IRQ_STATUS);
	adpt_irq_en(adp, adp->irq_en_off, 0);
	adp->scanout_off = false;
}

/* CRTC on and everything this driver switched off is back (no DSI: the panel can't be asleep) */
static bool adpt_on_ok_locked(struct adpt *adp)
{
	return !adp->scanout_off && !adp->panel_asleep;
}

/*
 * Screen off, in iOS's order: scanout off (deep), then the panel (unless deep_panel=0). @deep / @force
 * come from debugfs (@force = a deliberate off there: no healing until the next on); the CRTC disable
 * path passes false/false and the module parameters decide.
 */
static void adpt_display_off(struct adpt *adp, bool deep, bool force)
{
	ktime_t t0 = ktime_get();
	bool sleep, acted = false;
	int idx;

	mutex_lock(&adp->panel_lock);
	if (!force && adp->panel_keep)
		goto out;
	if (!force)
		deep = READ_ONCE(deep_off);
	sleep = force || READ_ONCE(panel_sleep) || (deep && READ_ONCE(deep_panel));
	if (!drm_dev_enter(&adp->drm, &idx))
		goto out;
	adp->dbg_off = force;
	adp->last_err = 0;
	adp->last_step = NULL;
	if (deep) {
		adpt_scanout_off_locked(adp);
		acted = true;
	}
	if (sleep) {
		adpt_panel_sleep_locked(adp);
		acted = true;
	}
	drm_dev_exit(idx);
	/* one line per real transition: the netconsole timeline next to the key presses */
	if (!force && acted)
		drm_info(&adp->drm, "display off #%u: scanout %s, panel %s, %lld ms%s%s\n", ++adp->n_off,
			 adp->scanout_off ? "off" : "ON",
			 adp->panel_asleep ? "asleep" : adp->dsi ? "AWAKE" : "(no DSI)",
			 ktime_ms_delta(ktime_get(), t0), adp->last_step ? "; failed: " : "", adp->last_step ?: "");
out:
	mutex_unlock(&adp->panel_lock);
}

/*
 * Undo whatever adpt_display_off() did (no-op otherwise): layer back first, then panel wake.
 * @crtc: the CRTC enable path. If a step fails there the CRTC is on but the screen isn't:
 * display_state reports "bad" until it is fixed, and the heal work retries a failed panel wake
 * in the background.
 */
static void adpt_display_on(struct adpt *adp, bool scanout, bool crtc)
{
	ktime_t t0 = ktime_get();
	bool was_off;
	int idx;

	mutex_lock(&adp->panel_lock);
	if (crtc) {
		adp->crtc_on = true;
		adp->heal_tries = 0;
	}
	if (scanout || crtc)	/* a whole on (debugfs "panel wake" alone keeps a deliberate layer off) */
		adp->dbg_off = false;
	if (!drm_dev_enter(&adp->drm, &idx))
		goto out;
	was_off = adp->scanout_off || adp->panel_asleep;
	adp->last_err = 0;
	adp->last_step = NULL;
	if (scanout)
		adpt_scanout_on_locked(adp);
	adpt_panel_wake_locked(adp);
	drm_dev_exit(idx);
	if (!crtc || !was_off)
		goto out;
	adp->n_on++;
	if (adpt_on_ok_locked(adp)) {
		drm_info(&adp->drm, "display on #%u: %lld ms\n", adp->n_on, ktime_ms_delta(ktime_get(), t0));
	} else {
		adp->n_fail++;
		drm_err(&adp->drm, "display on #%u FAILED at %s (%d) after %lld ms: scanout %s, panel %s; %s\n",
			adp->n_on, adp->last_step ?: "?", adp->last_err, ktime_ms_delta(ktime_get(), t0),
			adp->scanout_off ? "off" : "on", adp->panel_asleep ? "asleep" : "awake",
			adp->panel_asleep ? "retrying the panel" : "needs a CRTC off/on");
		if (!adp->panel_keep && adp->panel_asleep)
			mod_delayed_work(system_freezable_wq, &adp->heal, msecs_to_jiffies(100));
	}
out:
	mutex_unlock(&adp->panel_lock);
}

/*
 * CRTC on but the panel didn't wake: retry the wake, 100 ms doubling to 2 s, for as long as the
 * CRTC stays on. Only the panel (DCS): a layer packet from here could race a flip's BASE/STRIDE
 * writes, which only the modeset locks exclude (see adpt_power_set()), and taking those here would
 * deadlock against the cancel_delayed_work_sync() in atomic_disable. A failed layer on stays "bad"
 * in display_state until userspace does a CRTC off/on (Orchard does after ~2 s).
 */
static void adpt_heal_work(struct work_struct *work)
{
	struct adpt *adp = container_of(to_delayed_work(work), struct adpt, heal);
	unsigned int tries;
	bool ok, layer_off;
	int idx;

	mutex_lock(&adp->panel_lock);
	if (!adp->crtc_on || adp->dbg_off || adp->panel_keep || !adp->panel_asleep) {
		mutex_unlock(&adp->panel_lock);
		return;
	}
	if (drm_dev_enter(&adp->drm, &idx)) {
		adp->last_err = 0;
		adp->last_step = NULL;
		adpt_panel_wake_locked(adp);
		drm_dev_exit(idx);
	}
	ok = !adp->panel_asleep;
	layer_off = adp->scanout_off;
	tries = ++adp->heal_tries;
	mutex_unlock(&adp->panel_lock);
	if (ok) {
		drm_info(&adp->drm, "panel awake after %u background retr%s%s\n", tries,
			 tries == 1 ? "y" : "ies",
			 layer_off ? " (scanout still off: needs a CRTC off/on)" : "");
		return;
	}
	drm_err_ratelimited(&adp->drm, "panel still asleep after %u retries\n", tries);
	queue_delayed_work(system_freezable_wq, &adp->heal, msecs_to_jiffies(min(100U << min(tries, 5U), 2000U)));
}

static void adpt_heal_cancel(void *arg)
{
	cancel_delayed_work_sync(&((struct adpt *)arg)->heal);
}

/*
 * sysfs display_state (the platform device, i.e. /sys/class/drm/cardN/device/): first word "ok", or
 * "bad" while the CRTC is on but the layer or the panel isn't (userspace keeps the backlight off then).
 */
static ssize_t display_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adpt *adp = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&adp->panel_lock);
	len = sysfs_emit(buf, "%s crtc %s scanout %s panel %s%s; on %u off %u failed %u heal %u; last %s %d\n",
			 adp->crtc_on && !adp->dbg_off && !adpt_on_ok_locked(adp) ? "bad" : "ok",
			 adp->crtc_on ? "on" : "off", adp->scanout_off ? "off" : "on",
			 !adp->dsi ? "nodsi" : adp->panel_asleep ? "asleep" : "awake",
			 adp->stalled ? " STALLED" : adp->irq_dead ? " IRQ-OFF" : "",
			 adp->n_on, adp->n_off, adp->n_fail, adp->heal_tries, adp->last_step ?: "-", adp->last_err);
	mutex_unlock(&adp->panel_lock);
	return len;
}
static DEVICE_ATTR_RO(display_state);

static void adpt_remove_state_file(void *arg)
{
	device_remove_file(arg, &dev_attr_display_state);
}

/* debugfs "panel": read the state, write "sleep" / "wake" (a test path that needs no DRM master) */
static int adpt_panel_show(struct seq_file *m, void *unused)
{
	struct adpt *adp = m->private;

	seq_printf(m, "%s%s\n", adp->panel_asleep ? "asleep" : "awake", adp->dsi ? "" : " (no DSI)");
	return 0;
}

static int adpt_panel_open(struct inode *inode, struct file *file)
{
	return single_open(file, adpt_panel_show, inode->i_private);
}

static ssize_t adpt_panel_write(struct file *file, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct adpt *adp = ((struct seq_file *)file->private_data)->private;
	char buf[8] = {};

	if (copy_from_user(buf, ubuf, min(len, sizeof(buf) - 1)))
		return -EFAULT;
	if (sysfs_streq(buf, "sleep"))
		adpt_display_off(adp, false, true);
	else if (sysfs_streq(buf, "wake"))
		adpt_display_on(adp, false, false);
	else
		return -EINVAL;
	return len;
}

static const struct file_operations adpt_panel_fops = {
	.owner = THIS_MODULE,
	.open = adpt_panel_open,
	.read = seq_read,
	.write = adpt_panel_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * debugfs "power": the whole deep off/on sequence without DRM master or a DPMS change ("off" = scanout
 * off + panel sleep regardless of deep_off/panel_sleep, "on" = undo). Reading shows the state and the
 * registers involved.
 */
static int adpt_power_show(struct seq_file *m, void *unused)
{
	struct adpt *adp = m->private;
	int idx;

	mutex_lock(&adp->panel_lock);
	seq_printf(m, "scanout %s, panel %s%s%s; deep_off=%d panel_sleep=%d; vbi %u\n",
		   adp->scanout_off ? "off" : "on", adp->panel_asleep ? "asleep" : "awake",
		   adp->dsi ? "" : " (no DSI)",
		   adp->stalled ? "; PIPE STALLED (timer vblank)" : adp->irq_dead ? "; IRQ off (timer vblank)" : "",
		   READ_ONCE(deep_off), READ_ONCE(panel_sleep), READ_ONCE(adp->vbi_count));
	seq_printf(m, "last packet %08x: %d after %u ms, sent at ADBE %08x, done at %08x (f<<16|line)\n",
		   adp->pkt_hdr, adp->pkt_ret, adp->pkt_ms, adp->pkt_send_line, adp->pkt_done_line);
	if (drm_dev_enter(&adp->drm, &idx)) {
		seq_printf(m, "LAYER_CTL %08x (saved %08x) IRQ_EN %08x (masked %08x) IRQ_STATUS %08x frame %04x VFTG %08x\n",
			   readl(adp->adp + ADP_LAYER_CTL), adp->layer_ctl, readl(adp->adp + ADP_IRQ_EN),
			   adp->scanout_off ? adp->irq_en_off : 0, readl(adp->adp + ADP_IRQ_STATUS),
			   readl(adp->adp + ADP_FRAME_COUNT) & 0xffff, readl(adp->adbe + ADBE_VFTG_CTL));
		drm_dev_exit(idx);
	}
	mutex_unlock(&adp->panel_lock);
	return 0;
}

static int adpt_power_open(struct inode *inode, struct file *file)
{
	return single_open(file, adpt_power_show, inode->i_private);
}

/* the CRTC's newest commit has written its registers and sent its event (caller holds the modeset locks) */
static int adpt_wait_commit_idle(struct adpt *adp)
{
	struct drm_crtc *crtc = &adp->crtc;
	struct drm_crtc_commit *commit;
	int ret = 0;

	spin_lock(&crtc->commit_lock);
	commit = list_first_entry_or_null(&crtc->commit_list, struct drm_crtc_commit, commit_entry);
	if (commit)
		drm_crtc_commit_get(commit);
	spin_unlock(&crtc->commit_lock);
	if (!commit)
		return 0;
	if (!wait_for_completion_timeout(&commit->hw_done, HZ) ||
	    !wait_for_completion_timeout(&commit->flip_done, HZ))
		ret = -ETIMEDOUT;
	drm_crtc_commit_put(commit);
	return ret;
}

/*
 * Same exclusion as an atomic commit: all modeset locks (no new commit or flip can start), then wait
 * until the last one is done with the hardware, so no BASE/STRIDE write races the layer change. Only
 * while the CRTC is on: with it off the pipe belongs to the DPMS path.
 */
enum adpt_dbg_op { ADPT_OP_OFF, ADPT_OP_ON, ADPT_OP_FIFO_NOP, ADPT_OP_FIFO_SAME };

/* debugfs "fifo" tests: a header-only packet, or LAYER_CTL = its current value (no visible change) */
static int adpt_fifo_test(struct adpt *adp, bool same)
{
	int idx, ret = -ENODEV;

	mutex_lock(&adp->panel_lock);
	if (drm_dev_enter(&adp->drm, &idx)) {
		if (same) {
			u32 pkt[] = { FIELD_PREP(ADP_FIFO_RUN_COUNT, 1) | ADP_LAYER_CTL,
				      readl(adp->adp + ADP_LAYER_CTL) };

			ret = adpt_fifo_packet(adp, pkt, ARRAY_SIZE(pkt));
		} else {
			ret = adpt_fifo_packet(adp, NULL, 0);
		}
		drm_info(&adp->drm, "fifo test %s: %d after %u ms, header %08x, sent at %08x, done at %08x, LAYER_CTL %08x, IRQ_STATUS %08x\n",
			 same ? "same" : "nop", ret, adp->pkt_ms, adp->pkt_hdr, adp->pkt_send_line,
			 adp->pkt_done_line, readl(adp->adp + ADP_LAYER_CTL), readl(adp->adp + ADP_IRQ_STATUS));
		drm_dev_exit(idx);
	}
	mutex_unlock(&adp->panel_lock);
	return ret;
}

static int adpt_power_set(struct adpt *adp, enum adpt_dbg_op op)
{
	struct drm_device *drm = &adp->drm;
	struct drm_modeset_acquire_ctx ctx;
	int ret;

	DRM_MODESET_LOCK_ALL_BEGIN(drm, ctx, 0, ret);
	if (!adp->crtc.state->active) {
		ret = -EBUSY;
	} else {
		ret = adpt_wait_commit_idle(adp);
		if (!ret && op == ADPT_OP_OFF)
			adpt_display_off(adp, true, true);
		else if (!ret && op == ADPT_OP_ON)
			adpt_display_on(adp, true, false);
		else if (!ret)
			ret = adpt_fifo_test(adp, op == ADPT_OP_FIFO_SAME);
	}
	DRM_MODESET_LOCK_ALL_END(drm, ctx, ret);
	return ret;
}

static ssize_t adpt_power_write(struct file *file, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct adpt *adp = ((struct seq_file *)file->private_data)->private;
	char buf[8] = {};
	int ret;

	if (copy_from_user(buf, ubuf, min(len, sizeof(buf) - 1)))
		return -EFAULT;
	if (sysfs_streq(buf, "off"))
		ret = adpt_power_set(adp, ADPT_OP_OFF);
	else if (sysfs_streq(buf, "on"))
		ret = adpt_power_set(adp, ADPT_OP_ON);
	else if (sysfs_streq(buf, "nop"))		/* param FIFO test: header-only packet */
		ret = adpt_power_set(adp, ADPT_OP_FIFO_NOP);
	else if (sysfs_streq(buf, "same"))		/* param FIFO test: LAYER_CTL = its value */
		ret = adpt_power_set(adp, ADPT_OP_FIFO_SAME);
	else
		return -EINVAL;
	return ret ?: len;
}

static const struct file_operations adpt_power_fops = {
	.owner = THIS_MODULE,
	.open = adpt_power_open,
	.read = seq_read,
	.write = adpt_power_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * debugfs "stall": test the stall safety net on a healthy pipe. "trip" makes the watchdog declare a
 * stall on its next run (same path as a real one: IRQ masked + off, timer vblank); "recover" undoes a
 * stall (also a real one: if the pipe is really dead the watchdog trips again 300 ms later). A storm
 * guard kill is not undone.
 */
static int adpt_stall_show(struct seq_file *m, void *unused)
{
	struct adpt *adp = m->private;

	seq_printf(m, "%s; vblank %s; vbi %u\n",
		   adp->storm_killed ? "storm-killed (permanent)" : adp->stalled ? "stalled (timer vblank)" : "ok",
		   READ_ONCE(adp->vbl_on) ? "on" : "off", READ_ONCE(adp->vbi_count));
	return 0;
}

static int adpt_stall_open(struct inode *inode, struct file *file)
{
	return single_open(file, adpt_stall_show, inode->i_private);
}

static int adpt_stall_recover(struct adpt *adp)
{
	unsigned long flags;
	bool ok;
	u32 en;

	mutex_lock(&adp->panel_lock);
	spin_lock_irqsave(&adp->irq_lock, flags);
	ok = adp->stalled && !adp->storm_killed;
	if (ok) {
		adp->irq_dead = false;
		adp->stalled = false;
	}
	spin_unlock_irqrestore(&adp->irq_lock, flags);
	if (!ok) {
		mutex_unlock(&adp->panel_lock);
		return -EINVAL;
	}
	hrtimer_cancel(&adp->vbl_timer);	/* no lock held that its callback takes */
	if (!test_noack)
		writel(readl(adp->adp + ADP_IRQ_STATUS), adp->adp + ADP_IRQ_STATUS);
	en = (adp->irq_en_stall & ~ADP_IRQ_VBI) | (READ_ONCE(adp->vbl_on) ? ADP_IRQ_VBI : 0);
	adpt_irq_en(adp, en, ~0u);
	if (adp->stall_irq_off) {
		adp->stall_irq_off = false;
		enable_irq(adp->irq);
	}
	if (READ_ONCE(adp->vbl_on)) {
		WRITE_ONCE(adp->wd_reset, true);
		mod_delayed_work(system_freezable_wq, &adp->vbl_wd, msecs_to_jiffies(ADPT_WD_PERIOD_MS));
	}
	mutex_unlock(&adp->panel_lock);
	drm_info(&adp->drm, "stall recovered: IRQ back on (IRQ_EN %08x)\n", en);
	return 0;
}

static ssize_t adpt_stall_write(struct file *file, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct adpt *adp = ((struct seq_file *)file->private_data)->private;
	char buf[8] = {};
	int ret = 0;

	if (copy_from_user(buf, ubuf, min(len, sizeof(buf) - 1)))
		return -EFAULT;
	if (sysfs_streq(buf, "trip")) {
		WRITE_ONCE(adp->wd_trip, true);
		mod_delayed_work(system_freezable_wq, &adp->vbl_wd, 0);
	} else if (sysfs_streq(buf, "recover")) {
		ret = adpt_stall_recover(adp);
	} else {
		return -EINVAL;
	}
	return ret ?: len;
}

static const struct file_operations adpt_stall_fops = {
	.owner = THIS_MODULE,
	.open = adpt_stall_open,
	.read = seq_read,
	.write = adpt_stall_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ---------------------------------------------------------------- plane / crtc */

static int adpt_plane_atomic_check(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs;
	struct drm_framebuffer *fb = ps->fb;
	int ret;

	if (!ps->crtc)
		return 0;
	cs = drm_atomic_get_crtc_state(state, ps->crtc);
	if (IS_ERR(cs))
		return PTR_ERR(cs);

	/* full screen only for now: pipe 0 scaler stays in iBoot's (bypass) configuration */
	ret = drm_atomic_helper_check_plane_state(ps, cs, DRM_PLANE_NO_SCALING, DRM_PLANE_NO_SCALING,
						  false, false);
	if (ret || !ps->visible)
		return ret;

	if (fb->obj[0]->funcs != &adpt_bo_funcs)
		return -EINVAL;	/* only our vram objects are DART-mapped */
	if (fb->pitches[0] % ADP_PITCH_ALIGN || fb->offsets[0] % ADP_PITCH_ALIGN)
		return -EINVAL;
	if (ps->src.x1 || ps->src.y1)
		return -EINVAL;
	return 0;
}

static void adpt_plane_atomic_update(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	struct adpt *adp = to_adpt(plane->dev);
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = ps->fb;
	int idx;

	if (!fb || !drm_dev_enter(&adp->drm, &idx))
		return;

	if (fb->pitches[0] != adp->pitch) {
		writel(fb->pitches[0], adp->adp + PIPE(0) + PIPE_STRIDE);
		adp->pitch = fb->pitches[0];
	}
	if (!fifo_sync && READ_ONCE(adp->vbl_on) && !READ_ONCE(adp->irq_dead)) {
		/* written from the next VBI (adpt_handle_events): a mid-frame base write tears */
		unsigned long flags;

		spin_lock_irqsave(&adp->drm.event_lock, flags);
		adp->pending_base = to_bo(fb->obj[0])->iova + fb->offsets[0];
		adp->base_pending = true;
		spin_unlock_irqrestore(&adp->drm.event_lock, flags);
	} else {
		writel(to_bo(fb->obj[0])->iova + fb->offsets[0], adp->adp + PIPE(0) + PIPE_BASE);
	}
	drm_dev_exit(idx);
}

static void adpt_plane_atomic_disable(struct drm_plane *plane, struct drm_atomic_commit *state)
{
	/*
	 * No known-safe layer disable yet (LAYER_EN* semantics on A8 are unverified). The pipe keeps
	 * scanning the last buffer; that is harmless because every carveout page stays DART-mapped.
	 */
}

static const struct drm_plane_helper_funcs adpt_plane_helper_funcs = {
	.atomic_check = adpt_plane_atomic_check,
	.atomic_update = adpt_plane_atomic_update,
	.atomic_disable = adpt_plane_atomic_disable,
};

static const struct drm_plane_funcs adpt_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

/* one opaque layer: ARGB8888 is scanned out as XRGB8888 (GL clients with an alpha channel, glmark2) */
static const u32 adpt_formats[] = { DRM_FORMAT_XRGB8888, DRM_FORMAT_ARGB8888 };

static enum drm_mode_status adpt_crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *mode)
{
	struct adpt *adp = to_adpt(crtc->dev);

	/* the timing generator is iBoot's; only its mode exists */
	if (mode->hdisplay != adp->mode.hdisplay || mode->vdisplay != adp->mode.vdisplay)
		return MODE_BAD;
	return MODE_OK;
}

static void adpt_crtc_atomic_enable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	drm_crtc_vblank_on(crtc);			/* first: the layer packet needs the IRQ */
	adpt_display_on(to_adpt(crtc->dev), true, true);	/* no-op unless this driver turned it off */
}

static void adpt_crtc_atomic_disable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct adpt *adp = to_adpt(crtc->dev);
	unsigned long flags;

	/* no background retry of an on from here (the work takes panel_lock, never the modeset locks) */
	mutex_lock(&adp->panel_lock);
	adp->crtc_on = false;
	mutex_unlock(&adp->panel_lock);
	cancel_delayed_work_sync(&adp->heal);

	/* a flip still waiting for its vblank would never complete once VBI is off */
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (adp->event) {
		drm_crtc_send_vblank_event(crtc, adp->event);
		adp->event = NULL;
		drm_crtc_vblank_put(crtc);
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

	/*
	 * Only with panel_sleep=1 or deep_off=1. Before vblank off: the layer write is timed by the VBI.
	 * Timings keep running: see adpt_scanout_off_locked() for why VFTG is not stopped.
	 */
	adpt_display_off(adp, false, false);
	drm_crtc_vblank_off(crtc);
}

static void adpt_crtc_atomic_flush(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct adpt *adp = to_adpt(crtc->dev);
	struct drm_pending_vblank_event *event = crtc->state->event;
	unsigned long flags;
	int idx;

	if (fifo_sync && drm_dev_enter(&adp->drm, &idx)) {
		adpt_fifo_kick(adp);
		drm_dev_exit(idx);
	}

	if (!event)
		return;
	crtc->state->event = NULL;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (drm_crtc_vblank_get(crtc) != 0) {
		drm_crtc_send_vblank_event(crtc, event);
	} else {
		WARN_ON(adp->event);
		adp->event = event;
		adp->flush_seq = drm_crtc_vblank_count(crtc);
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static int adpt_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct adpt *adp = to_adpt(crtc->dev);

	writel(ADP_IRQ_VBI, adp->adp + ADP_IRQ_STATUS);
	adpt_irq_en(adp, ADP_IRQ_VBI, 0);
	WRITE_ONCE(adp->vbl_on, true);
	if (READ_ONCE(adp->irq_dead)) {
		/* always (re)start: one returning NORESTART right now keeps a timer started meanwhile */
		hrtimer_start(&adp->vbl_timer, adp->vbl_period, HRTIMER_MODE_REL_HARD);
	} else {
		WRITE_ONCE(adp->wd_reset, true);
		mod_delayed_work(system_freezable_wq, &adp->vbl_wd, msecs_to_jiffies(ADPT_WD_PERIOD_MS));
	}
	return 0;
}

static void adpt_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct adpt *adp = to_adpt(crtc->dev);

	WRITE_ONCE(adp->vbl_on, false);	/* the watchdog and the fallback timer stop by themselves */
	adpt_irq_en(adp, 0, ADP_IRQ_VBI);
}

static const struct drm_crtc_helper_funcs adpt_crtc_helper_funcs = {
	.mode_valid = adpt_crtc_mode_valid,
	.atomic_enable = adpt_crtc_atomic_enable,
	.atomic_disable = adpt_crtc_atomic_disable,
	.atomic_flush = adpt_crtc_atomic_flush,
};

static const struct drm_crtc_funcs adpt_crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = adpt_crtc_enable_vblank,
	.disable_vblank = adpt_crtc_disable_vblank,
};

/* ---------------------------------------------------------------- connector: iBoot's fixed panel mode */

static int adpt_connector_get_modes(struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &to_adpt(connector->dev)->mode);
}

static const struct drm_connector_helper_funcs adpt_connector_helper_funcs = {
	.get_modes = adpt_connector_get_modes,
};

static const struct drm_connector_funcs adpt_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_encoder_funcs adpt_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_mode_config_funcs adpt_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,	/* no .dirty: buffers are scanned out directly */
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void adpt_init_mode(struct adpt *adp, u32 w, u32 h, u32 wmm, u32 hmm)
{
	struct drm_display_mode *m = &adp->mode;

	/*
	 * The real porches live in ADBE0 0x10..0x18 (semantics not verified yet). Vblank timestamps come
	 * from the IRQ, so only the refresh rate matters: pick small porches and a clock that gives 60 Hz.
	 */
	m->hdisplay = w;
	m->hsync_start = w + 8;
	m->hsync_end = w + 16;
	m->htotal = w + 24;
	m->vdisplay = h;
	m->vsync_start = h + 8;
	m->vsync_end = h + 10;
	m->vtotal = h + 20;
	m->clock = DIV_ROUND_UP(m->htotal * m->vtotal * 60, 1000);
	m->width_mm = wmm;
	m->height_mm = hmm;
	m->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(m);
}

static int adpt_modeset_init(struct adpt *adp)
{
	struct drm_device *drm = &adp->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = adp->mode.hdisplay;
	drm->mode_config.max_width = ALIGN(adp->mode.hdisplay, 64);
	drm->mode_config.min_height = adp->mode.vdisplay;
	drm->mode_config.max_height = ALIGN(adp->mode.vdisplay, 64);
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.prefer_shadow = 0;
	drm->mode_config.funcs = &adpt_mode_config_funcs;

	ret = drm_universal_plane_init(drm, &adp->primary, 1, &adpt_plane_funcs, adpt_formats,
				       ARRAY_SIZE(adpt_formats), NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&adp->primary, &adpt_plane_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, &adp->crtc, &adp->primary, NULL, &adpt_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(&adp->crtc, &adpt_crtc_helper_funcs);

	ret = drm_encoder_init(drm, &adp->encoder, &adpt_encoder_funcs, DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		return ret;
	adp->encoder.possible_crtcs = drm_crtc_mask(&adp->crtc);

	ret = drmm_connector_init(drm, &adp->connector, &adpt_connector_funcs, DRM_MODE_CONNECTOR_DSI, NULL);
	if (ret)
		return ret;
	drm_connector_helper_add(&adp->connector, &adpt_connector_helper_funcs);
	adp->connector.display_info.width_mm = adp->mode.width_mm;
	adp->connector.display_info.height_mm = adp->mode.height_mm;
	ret = drm_connector_attach_encoder(&adp->connector, &adp->encoder);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);
	return 0;
}

/* ---------------------------------------------------------------- resources */

static bool adpt_pmgr_on(struct adpt *adp)
{
	static const struct { u32 off; const char *name; } ps[] = {
		{ 0x2a8, "disp_busmux" }, { 0x2b0, "disp0" }, { 0x2b8, "mipi_dsi" },
	};
	void __iomem *pm = ioremap_np(T7000_PMGR_PS_PAGE, PAGE_SIZE);
	bool ok = true;
	int i;

	if (!pm)
		return false;
	for (i = 0; i < ARRAY_SIZE(ps); i++) {
		u32 v = readl(pm + ps[i].off);

		if ((v & 0xff) != 0xff || (v & (BIT(31) | BIT(10)))) {
			drm_err(&adp->drm, "power domain %s not on (%08x)\n", ps[i].name, v);
			ok = false;
		}
	}
	iounmap(pm);
	return ok;
}

static void __iomem *adpt_map(struct platform_device *pdev, const char *name, resource_size_t phys,
			      resource_size_t size)
{
	struct resource res = DEFINE_RES_MEM_NAMED(phys, size, name);

	if (!standalone)
		return devm_platform_ioremap_resource_byname(pdev, name);
	res.flags |= IORESOURCE_MEM_NONPOSTED;
	return devm_ioremap_resource(&pdev->dev, &res);
}

/* panel commands are optional: without the DSI host, panel_sleep does nothing */
static void __iomem *adpt_map_dsi(struct adpt *adp, struct device *dev)
{
	struct resource res = DEFINE_RES_MEM_NAMED(T7000_DSI_PHYS, SZ_4K, "dsi");
	void __iomem *dsi;

	res.flags |= IORESOURCE_MEM_NONPOSTED;
	dsi = devm_ioremap_resource(dev, &res);
	if (IS_ERR(dsi)) {
		drm_warn(&adp->drm, "DSI host not mapped (%pe): no panel sleep\n", dsi);
		return NULL;
	}
	return dsi;
}

static int adpt_get_irq(struct platform_device *pdev)
{
	struct irq_fwspec fw = { .param_count = 3, .param = { 0, T7000_ADP_HWIRQ, IRQ_TYPE_LEVEL_HIGH } };
	struct device_node *aic;
	int irq;

	if (!standalone)
		return platform_get_irq_byname(pdev, "adp");

	aic = of_find_compatible_node(NULL, NULL, "apple,aic");
	if (!aic)
		return -ENODEV;
	fw.fwnode = of_fwnode_handle(aic);
	irq = irq_create_fwspec_mapping(&fw);
	of_node_put(aic);
	return irq ?: -ENODEV;
}

static int adpt_get_vram(struct adpt *adp, struct device *dev)
{
	struct device_node *np;
	struct reserved_mem *rmem;

	if (standalone) {
		adp->vram_phys = T7000_VRAM_PHYS;
		adp->vram_size = T7000_VRAM_SIZE;
		return 0;
	}
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return -ENODEV;
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem)
		return -ENODEV;
	adp->vram_phys = rmem->base;
	adp->vram_size = rmem->size;
	return 0;
}

/* map one iBoot DART table page; NULL ends the usable part (table inside vram, or unmappable) */
static u64 *adpt_map_table(struct adpt *adp, phys_addr_t pa)
{
	/* a cacheable read of a junk PA (unclaimed MMIO) can hang the bus: only ever follow DRAM */
	if (pa < T7000_DRAM_BASE || pa + SZ_4K > T7000_DRAM_BASE + T7000_DRAM_SIZE) {
		drm_err(&adp->drm, "DART table pointer %pa is outside DRAM: usable part ends there\n", &pa);
		return NULL;
	}
	if (pa >= adp->vram_phys && pa < adp->vram_phys + adp->vram_size) {
		drm_warn(&adp->drm, "DART table %pa lies inside vram: usable part ends there\n", &pa);
		return NULL;
	}
	if (region_intersects(pa, SZ_4K, IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) != REGION_DISJOINT)
		drm_warn(&adp->drm, "DART table %pa is in Linux RAM: it can get overwritten\n", &pa);
	/* WB: the linear map for RAM, a cacheable mapping otherwise (ioremap_wc refuses RAM) */
	return memremap(pa, SZ_4K, MEMREMAP_WB);
}

/*
 * Walk iBoot's dart-disp0 stream 0 tables (S5L8960X: 4 TTBRs x 1 GB, 4K pages, two levels) and return
 * how much of vram, from its start, is mapped 1:1 at vram_iova. Table pages inside vram end the usable
 * part, so no buffer can ever overwrite a page table.
 */
static size_t adpt_dart_usable(struct adpt *adp)
{
	void __iomem *dart = ioremap_np(T7000_DART_PHYS, PAGE_SIZE);
	size_t off, usable = 0;
	u64 *l1 = NULL, *l2 = NULL;
	phys_addr_t l1_pa = 0, l2_pa = 0;

	if (!dart)
		return 0;
	/*
	 * n61: iBoot leaves stream 0 untranslated (TCR byte 0 without bit 7, TTBRs hold junk that must
	 * never be followed): the pipe fetches PA = 0x800000000 + its 32-bit address, so all of vram is usable.
	 */
	if (!(readl(dart + DART_TCR) & 0x80)) {
		iounmap(dart);
		if ((phys_addr_t)adp->vram_iova + 0x800000000ULL != adp->vram_phys) {
			drm_err(&adp->drm, "DART bypass but vram iova %#x doesn't match pa %pa\n", adp->vram_iova,
				&adp->vram_phys);
			return 0;
		}
		drm_info(&adp->drm, "dart-disp0 in bypass: all %zu KB of vram usable\n", adp->vram_size >> 10);
		return adp->vram_size;
	}
	for (off = 0; off < adp->vram_size; off += SZ_4K) {
		u32 iova = adp->vram_iova + off;
		u32 ttbr = readl(dart + DART_TTBR(0, iova >> 30));
		phys_addr_t pa;
		u64 e;

		if (!(ttbr & DART_TTBR_VALID))
			break;
		pa = (phys_addr_t)(ttbr & 0xffffff) << 12;
		if (pa != l1_pa) {
			if (l1)
				memunmap(l1);
			l1 = adpt_map_table(adp, pa);
			l1_pa = pa;
			if (!l1)
				break;
		}
		e = READ_ONCE(l1[(iova >> 21) & 511]);
		if (!(e & DART_PTE_VALID))
			break;
		pa = e & DART_PA_MASK;
		if (pa != l2_pa) {
			if (l2)
				memunmap(l2);
			l2 = adpt_map_table(adp, pa);
			l2_pa = pa;
			if (!l2)
				break;
		}
		e = READ_ONCE(l2[(iova >> 12) & 511]);
		if (!(e & DART_PTE_VALID) || (e & DART_PA_MASK) != adp->vram_phys + off)
			break;
		usable = off + SZ_4K;
	}
	if (l1)
		memunmap(l1);
	if (l2)
		memunmap(l2);
	iounmap(dart);
	return usable;
}

static void adpt_pool_fini(struct drm_device *drm, void *arg)
{
	struct adpt *adp = to_adpt(drm);

	gen_pool_destroy(adp->pool);
	memunmap(adp->vram);
}

static int adpt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dev_pm_domain_attach_data pd_data = { .pd_flags = PD_FLAG_DEV_LINK_ON };
	struct device_node *fbnp;
	struct adpt *adp;
	u32 w = 750, h = 1334, wmm = 0, hmm = 0, fmt, size;
	size_t usable;
	int ret;

	adp = devm_drm_dev_alloc(dev, &adpt_driver, struct adpt, drm);
	if (IS_ERR(adp))
		return PTR_ERR(adp);
	spin_lock_init(&adp->irq_lock);
	ret = drmm_mutex_init(&adp->drm, &adp->panel_lock);
	if (ret)
		return ret;

	/* hold disp0 + mipi_dsi before simpledrm (their other user) goes away */
	ret = devm_pm_domain_attach_list(dev, &pd_data, &adp->pds);
	if (ret < 0)
		return dev_err_probe(dev, ret, "power domains\n");
	if (!adpt_pmgr_on(adp))
		return -ENODEV;

	adp->adp = adpt_map(pdev, "adp", T7000_ADP_PHYS, 0x8000);
	if (IS_ERR(adp->adp))
		return PTR_ERR(adp->adp);
	adp->adbe = adpt_map(pdev, "adbe", T7000_ADBE_PHYS, 0x8000);
	if (IS_ERR(adp->adbe))
		return PTR_ERR(adp->adbe);
	adp->irq = adpt_get_irq(pdev);
	if (adp->irq < 0)
		return adp->irq;
	adp->dsi = adpt_map_dsi(adp, dev);
	ret = adpt_get_vram(adp, dev);
	if (ret)
		return dev_err_probe(dev, ret, "no vram carveout\n");

	/* the boot framebuffer's geometry (m1n1 fills it from iBoot's boot_args) */
	fbnp = of_find_compatible_node(NULL, NULL, "simple-framebuffer");
	if (fbnp) {
		of_property_read_u32(fbnp, "width", &w);
		of_property_read_u32(fbnp, "height", &h);
		of_node_put(fbnp);
	}
	if (dev->of_node && !standalone) {
		of_property_read_u32(dev->of_node, "width-mm", &wmm);
		of_property_read_u32(dev->of_node, "height-mm", &hmm);
	}
	adpt_init_mode(adp, w, h, wmm, hmm);

	adp->boot.base = readl(adp->adp + PIPE(0) + PIPE_BASE);
	adp->boot.stride = readl(adp->adp + PIPE(0) + PIPE_STRIDE);
	adp->pitch = adp->boot.stride;
	adp->irq_en_orig = readl(adp->adp + ADP_IRQ_EN);
	fmt = readl(adp->adp + PIPE(0) + PIPE_PIX_FMT);
	size = readl(adp->adbe + ADBE_SCREEN_SIZE);
	drm_info(&adp->drm, "iBoot pipe0: base %08x stride %u fmt %08x src %08x, ADBE size %08x, mask %08x, layer ctl %08x\n",
		 adp->boot.base, adp->boot.stride, fmt, readl(adp->adp + PIPE(0) + PIPE_SRC_SIZE), size,
		 adp->irq_en_orig, readl(adp->adp + ADP_LAYER_CTL));
	if (fmt != PIPE_PIX_FMT_XRGB32 && fmt != PIPE_PIX_FMT_XRGB32_N61 && !force)
		return dev_err_probe(dev, -ENODEV, "unexpected pipe0 format %08x (force=1 to ignore)\n", fmt);

	adp->vram_iova = iova_base ?: adp->boot.base;
	if (!adp->vram_iova || adp->vram_iova & (SZ_4K - 1))
		return dev_err_probe(dev, -ENODEV, "vram IOVA %08x not usable\n", adp->vram_iova);

	usable = verify_dart ? adpt_dart_usable(adp) : adp->vram_size;
	drm_info(&adp->drm, "vram %pa+%zx at iova %08x, %zx usable\n", &adp->vram_phys, adp->vram_size,
		 adp->vram_iova, usable);
	if (usable < 2 * PAGE_ALIGN(ALIGN(w * 4, ADP_PITCH_ALIGN) * h))
		return dev_err_probe(dev, -ENODEV, "less than two frames of vram are DART-mapped\n");
	adp->vram_size = usable;

	/*
	 * Everything that can fail happens before simpledrm is removed, so a refused or failed probe
	 * always leaves the boot display (and Orchard's fallback path) intact. simpledrm keeps scanning
	 * vram+0 meanwhile; nothing below writes vram or the pipe's layer registers.
	 */
	adp->vram = memremap(adp->vram_phys, adp->vram_size, MEMREMAP_WC);
	if (!adp->vram)
		return -ENOMEM;
	adp->pool = gen_pool_create(PAGE_SHIFT, NUMA_NO_NODE);
	if (!adp->pool) {
		memunmap(adp->vram);
		return -ENOMEM;
	}
	ret = gen_pool_add_virt(adp->pool, (unsigned long)adp->vram, adp->vram_phys, adp->vram_size,
				NUMA_NO_NODE);
	if (ret) {
		gen_pool_destroy(adp->pool);
		memunmap(adp->vram);
		return ret;
	}
	/* freed with the drm_device, i.e. after the last GEM object */
	ret = drmm_add_action_or_reset(&adp->drm, adpt_pool_fini, NULL);
	if (ret)
		return ret;

	ret = adpt_modeset_init(adp);
	if (ret)
		return ret;

	adp->vbl_period = ns_to_ktime(NSEC_PER_SEC / (drm_mode_vrefresh(&adp->mode) ?: 60));
	hrtimer_setup(&adp->vbl_timer, adpt_vbl_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	/* devm runs in reverse: free_irq first, then this, so the timer can't be restarted after it */
	ret = devm_add_action_or_reset(dev, adpt_vbl_timer_cancel, adp);
	if (ret)
		return ret;
	init_waitqueue_head(&adp->layer_wq);
	INIT_DELAYED_WORK(&adp->vbl_wd, adpt_vbl_wd);
	INIT_DELAYED_WORK(&adp->heal, adpt_heal_work);
	ret = devm_add_action_or_reset(dev, adpt_heal_cancel, adp);
	if (ret)
		return ret;
	/* before the timer cancel above runs: the watchdog may start the timer */
	ret = devm_add_action_or_reset(dev, adpt_vbl_wd_cancel, adp);
	if (ret)
		return ret;

	if (!test_noack)
		writel(readl(adp->adp + ADP_IRQ_STATUS), adp->adp + ADP_IRQ_STATUS);
	ret = devm_request_irq(dev, adp->irq, adpt_irq, 0, "adp-t7000", adp);
	if (ret)
		return ret;
	/* VBI is enabled on demand; packet-done is needed for flip completion in FIFO mode */
	adpt_irq_en(adp, fifo_sync ? ADP_IRQ_PKT_DONE : 0, ADP_IRQ_VBI);

	platform_set_drvdata(pdev, adp);
	ret = test_fail ? -EAGAIN : drm_dev_register(&adp->drm, 0);
	if (ret) {
		writel(adp->irq_en_orig, adp->adp + ADP_IRQ_EN);
		return dev_err_probe(dev, ret, "%s; simpledrm left in place\n",
				     test_fail ? "test_fail=1" : "drm_dev_register failed");
	}

	/*
	 * Point of no return, and it cannot fail (aperture_remove_conflicting_devices() always returns
	 * 0): unbind simpledrm, which scans the same vram. Our power domain links keep the pipe on.
	 * Registering first is fine here: no fbdev, and nothing touches the hardware until a client's
	 * first commit (Orchard starts after the 48-display hook has returned).
	 */
	aperture_remove_conflicting_devices(adp->vram_phys, adp->vram_size, adpt_driver.name);
	debugfs_create_file("panel", 0600, adp->drm.debugfs_root, adp, &adpt_panel_fops);
	debugfs_create_file("power", 0600, adp->drm.debugfs_root, adp, &adpt_power_fops);
	debugfs_create_file("stall", 0600, adp->drm.debugfs_root, adp, &adpt_stall_fops);
	/* past the point of no return: without the file userspace just can't check the on state */
	if (device_create_file(dev, &dev_attr_display_state) ||
	    devm_add_action_or_reset(dev, adpt_remove_state_file, dev))
		dev_warn(dev, "no display_state file\n");
	return 0;
}

static void adpt_remove(struct platform_device *pdev)
{
	struct adpt *adp = platform_get_drvdata(pdev);

	/* leave the pipe and the panel as iBoot did: scanning out, awake */
	adp->panel_keep = true;
	adpt_display_on(adp, true, false);
	cancel_delayed_work_sync(&adp->heal);
	drm_dev_unplug(&adp->drm);
	drm_atomic_helper_shutdown(&adp->drm);
	WRITE_ONCE(adp->vbl_on, false);
	cancel_delayed_work_sync(&adp->vbl_wd);
	hrtimer_cancel(&adp->vbl_timer);	/* no-op unless the storm guard or the watchdog started it */
	/* back to iBoot's register state; the image there is whatever was last drawn into vram+0 */
	writel(adp->boot.stride, adp->adp + PIPE(0) + PIPE_STRIDE);
	writel(adp->boot.base, adp->adp + PIPE(0) + PIPE_BASE);
	if (fifo_sync)
		writel(ADP_FIFO_SYNC | ++adp->tag, adp->adp + ADP_FIFO);
	writel(adp->irq_en_orig, adp->adp + ADP_IRQ_EN);
}

static void adpt_shutdown(struct platform_device *pdev)
{
	struct adpt *adp = platform_get_drvdata(pdev);

	if (adp) {
		adp->panel_keep = true;
		adpt_display_on(adp, true, false);
		cancel_delayed_work_sync(&adp->heal);
		drm_atomic_helper_shutdown(&adp->drm);
	}
}

static const struct of_device_id adpt_of_match[] = {
	{ .compatible = "apple,t7000-display-pipe" },
	{ }
};
MODULE_DEVICE_TABLE(of, adpt_of_match);

static struct platform_driver adpt_platform_driver = {
	.driver = {
		.name = "adp-t7000",
		.of_match_table = adpt_of_match,
	},
	.probe = adpt_probe,
	.remove = adpt_remove,
	.shutdown = adpt_shutdown,
};

static struct platform_device *adpt_test_pdev;

static int __init adpt_init(void)
{
	struct platform_device *pdev;
	struct device_node *fbnp;
	int ret;

	ret = platform_driver_register(&adpt_platform_driver);
	if (ret || !standalone)
		return ret;

	/*
	 * Test mode: borrow /chosen/framebuffer's node for its power-domains (disp0 + mipi_dsi). The node
	 * says "simple-framebuffer", so pin the device to this driver or simpledrm would grab it.
	 */
	fbnp = of_find_compatible_node(NULL, NULL, "simple-framebuffer");
	if (!fbnp) {
		ret = -ENODEV;
		goto err_driver;
	}
	pdev = platform_device_alloc("adp-t7000", PLATFORM_DEVID_NONE);
	if (!pdev) {
		of_node_put(fbnp);
		ret = -ENOMEM;
		goto err_driver;
	}
	device_set_node(&pdev->dev, of_fwnode_handle(fbnp));	/* reference dropped on release */
	dev_set_of_node_reused(&pdev->dev);
	ret = device_set_driver_override(&pdev->dev, "adp-t7000");
	if (!ret)
		ret = platform_device_add(pdev);
	if (ret) {
		platform_device_put(pdev);
		goto err_driver;
	}
	adpt_test_pdev = pdev;
	return 0;

err_driver:
	platform_driver_unregister(&adpt_platform_driver);
	return ret;
}

static void __exit adpt_exit(void)
{
	if (adpt_test_pdev)
		platform_device_unregister(adpt_test_pdev);
	platform_driver_unregister(&adpt_platform_driver);
}

module_init(adpt_init);
module_exit(adpt_exit);

MODULE_DESCRIPTION("Apple A8 (T7000) display pipe DRM driver");
MODULE_LICENSE("GPL");
