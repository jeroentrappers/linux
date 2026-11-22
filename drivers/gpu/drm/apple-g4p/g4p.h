/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple A8 GPU (G4P): shared driver state.
 */
#ifndef G4P_H
#define G4P_H

#include <linux/bits.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#include <drm/drm_device.h>

#include "g4p_fw.h"

struct dentry;
struct dma_fence;
struct device;
struct seq_file;

/* Fender, Apple's GPU power controller (AGX_CR_FNDR_*; re/GPU-BRINGUP.md 1, 8.11) */
#define FNDR_CTRL		0x0000	/* 0x40000000 at wake; bit 29 is firmware state (8.11), never written */
#define FNDR_PERF_CTL		0x1000	/* [3:0] perf state, bit 4 = request pending */
#define FNDR_PERF_CTL_BUSY	BIT(4)
#define FNDR_PERF_NEXT		0x1004	/* read only (iOS dumps it) */
#define FNDR_SLEEP_TIMER	0x10000
#define FNDR_TIMEOUT_US		100000

/* Rogue core (upstream ROGUE_CR_*) */
#define RGX_CR_CORE_ID		0x0018
#define RGX_CR_CORE_REVISION	0x0020

#define G4P_BRIDGE11_WORDS	32

/* PMGR MiscACG (XNU _regGroups[kRegGroupMiscACG]); bit 0 = auto clock gating enable */
#define PMGR_MISC_ACG_EN	BIT(0)

struct g4p_bo {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u64 gpuva;
	bool view;	/* carved out of another buffer: not freed on its own */
};

struct g4p {
	struct drm_device drm;			/* render node (g4p_drm.c, GPU-BRINGUP 8.64) */
	struct device *dev;
	void __iomem *rgx;
	void __iomem *fender;
	void __iomem *bridge10;
	void __iomem *bridge11;
	void __iomem *misc_acg;			/* NULL: not described, RGX access refused */
	u32 bridge10_cfg[2];			/* offset, value */
	u32 bridge11_cfg[G4P_BRIDGE11_WORDS];	/* words from offset 0 */
	struct mutex lock;
	bool powered;
	bool have_id;
	u32 core_rev, core_id;
	struct dentry *debugfs;
	struct g4p_fw fw;			/* loaded on first read of debugfs firmware */
	void *fw_image;				/* host copy (vmalloc), for debugfs firmware */
	int irq;				/* firmware -> host, < 0 if not described */
	atomic_t irq_count;
	wait_queue_head_t irq_wq;
	bool irq_on;
	struct g4p_boot *boot;			/* GPU memory of a META boot; freed after power-off */
	struct g4p_gfx *gfx;			/* 8.88: C9 graphics harness state of this boot */
	/*
	 * DRM pool (8.64): one contiguous buffer, mapped at G4P_PDS_VA (0x300000000) before each firmware boot
	 * that the DRM path starts; page 0 holds the driver's PDS programs, BOs are page ranges after it.
	 */
	struct g4p_bo pool;
	unsigned long *pool_map;		/* bitmap of used pages */
	unsigned int pool_pages;
	struct list_head sep_bos;		/* separate BOs (8.65), mapped into every boot */
	struct list_head zombies;		/* their memory, freed while the GPU was on */
	u64 sep_next;				/* next free GPU VA for them (context 0) */
	unsigned long ctx_used;			/* R5: address spaces owned by DRM files (bit n = address space n) */
	u8 *pool_owner;				/* 8.79: per pool page, the context of the file that owns it */
	unsigned int job_ctx;			/* set during a DRM submit: the job's address space (8.84) */
	struct delayed_work idle_work;		/* 8.67: power off after idle_ms without a submit */
	/* 8.68: the one asynchronous job in flight */
	bool job_async;				/* set during a submit: g4p_boot_cl_job returns after the kick */
	spinlock_t fence_lock;
	struct dma_fence *fq[8];		/* 8.69: jobs in flight, oldest at fq_tail; IRQ / timeout signal */
	u32 fq_job[8];				/* their job numbers in the boot */
	u8 fq_ctx[8];				/* their address spaces (8.83: whose job faulted) */
	unsigned long fq_deadline;		/* 8.83: jiffies when the oldest job times out */
	unsigned int fq_head, fq_tail;
	u64 fence_ctx, fence_seq;
	struct delayed_work timeout_work;
	bool drm_booted;			/* the running boot was started by the DRM path */
	const void *job_img;			/* set during a DRM submit: C8 job image from memory */
	size_t job_img_size;
};

/* g4p_mmu.c: GPU MMU and buffers in the firmware heaps (AGXGart heaps 5..8) */
enum g4p_heap {
	G4P_HEAP_FW,		/* 0xe1c0000000, the firmware image */
	G4P_HEAP_SHARED,	/* 0xe1c0200000, "Firmware/Driver Shared" (list B) */
	G4P_HEAP_PRIVATE,	/* 0xe1c3200000, "Cached Firmware Private" (list A, pools) */
	G4P_HEAP_PIO,		/* 0xe1c4200000, register windows for the firmware */
	G4P_NR_HEAPS
};

#define G4P_PTE_VALID		BIT_ULL(0)
#define G4P_PTE_READ_ONLY	BIT_ULL(1)
#define G4P_PTE_CC		BIT_ULL(2)
#define G4P_PTE_SLC_BYPASS	BIT_ULL(3)
#define G4P_PTE_PM_META_PROTECT	BIT_ULL(62)

#define G4P_CTX_FW_PAGES	16	/* 8.81: two per job slot */
#define G4P_NR_CTX	8	/* firmware memory contexts (CAT_BASE0..7); 0 = kernel */
#define G4P_NR_VM	32	/* 8.84: GPU address spaces (one per open file, 0 = kernel), in slots 1..7 */

struct g4p_mmu {
	u32 *pc;
	u64 *pd;
	dma_addr_t pc_dma, pd_dma;
	unsigned int pc_index;
	u64 *pt[512];
	dma_addr_t pt_dma[512];
	u64 heap_next[G4P_NR_HEAPS];
	/*
	 * One extra 1 GiB PC entry outside the firmware heaps (the code heap / user pool at 0x300000000,
	 * GPU-BRINGUP 8.35, 8.64): its PD and page tables on demand, one per 2 MiB.
	 */
	u64 *xpd, *xpt[512];
	dma_addr_t xpd_dma, xpt_dma[512];
	u64 xva;		/* 1 GiB aligned base of the entry, 0 = none */
	/*
	 * R5 (8.77, 8.79): per-client memory contexts 1..7, allocated on use. Each has its own PC (a copy of the
	 * kernel's, context 0) with its own PD and page tables for the extra GiB: pool page 0 (read-only) and the
	 * client's own pool pages and separate buffers only. The firmware loads CAT_BASE[n] from its context
	 * table; the CDM kick names the context (stream record 2, payload word 4).
	 */
	struct g4p_mmu_ctx {
		u32 *pc;
		u64 *pd, *pt[512];
		dma_addr_t pc_dma, pd_dma, pt_dma[512];
		/*
		 * 8.81: the firmware heaps' GiB in this context: its own PD, nothing mapped but the context's
		 * private pages (the CDM's stream and terminate pages per job slot, at the slots' VAs)
		 */
		u64 *fpd, *fpt[512];
		dma_addr_t fpd_dma, fpt_dma[512];
		void *fpage[G4P_CTX_FW_PAGES];
		dma_addr_t fpage_dma[G4P_CTX_FW_PAGES];
	} *ctx[G4P_NR_VM];	/* per address space (8.84), not per hardware slot */
};

int g4p_mmu_init(struct device *dev, struct g4p_mmu *mmu);
int g4p_mmu_ctx_map_fw_page(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx, u64 va, unsigned int page,
			    u64 flags);
u64 g4p_mmu_heap_flags(enum g4p_heap heap);
void g4p_mmu_fini(struct device *dev, struct g4p_mmu *mmu);
int g4p_mmu_map(struct device *dev, struct g4p_mmu *mmu, u64 va, phys_addr_t pa, size_t size, u64 flags);
int g4p_mmu_map_page(struct device *dev, struct g4p_mmu *mmu, u64 va, phys_addr_t pa, u64 flags);
void g4p_mmu_unmap_page(struct g4p_mmu *mmu, u64 va);
int g4p_mmu_ctx_init(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx);
void g4p_mmu_ctx_fini(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx);
int g4p_mmu_ctx_map_page(struct device *dev, struct g4p_mmu *mmu, unsigned int ctx, u64 va, phys_addr_t pa,
			 u64 flags);
void g4p_mmu_ctx_unmap_page(struct g4p_mmu *mmu, unsigned int ctx, u64 va);
int g4p_mmu_map_pio(struct device *dev, struct g4p_mmu *mmu, phys_addr_t pa, size_t size, u64 *gpuva);
int g4p_bo_alloc(struct device *dev, struct g4p_mmu *mmu, enum g4p_heap heap, size_t size, struct g4p_bo *bo);
void g4p_bo_free(struct device *dev, struct g4p_bo *bo);
u32 g4p_fw_addr(u64 gpuva, bool cached);
u64 g4p_fw_heap_base(void);

/* g4p_fwinit.c: firmware shared structures (re/GPU-FW-INIT.md) */
struct g4p_fwinit {
	struct g4p_bo list_a;	/* heap 7 */
	struct g4p_bo list_b;	/* heap 6 */
	u64 pio_va[3];		/* heap 8: Fender, AIC timer, AIC SW-int */
};

int g4p_fwinit_build(struct g4p *g, struct g4p_mmu *mmu, struct g4p_fwinit *fi, const struct g4p_bo *image,
		     u32 *roots);
void g4p_fwinit_free(struct device *dev, struct g4p_fwinit *fi);
bool g4p_fwinit_event(struct g4p_fwinit *fi, u32 *type, u32 *payload);
void g4p_fwinit_show_log(struct seq_file *s, struct g4p_fwinit *fi);
void g4p_fwinit_show_events(struct seq_file *s, struct g4p_fwinit *fi);
void g4p_fwinit_gart_inval(struct g4p_fwinit *fi, unsigned int ctx);
void g4p_fwinit_set_ctx(struct g4p_fwinit *fi, unsigned int ctx, u64 pc_dma);
void g4p_fwinit_ack_trace_stats(struct g4p_fwinit *fi);
u32 g4p_fwinit_ring_ctl(struct g4p_fwinit *fi, unsigned int which, unsigned int off);
int g4p_fwinit_ring_push(struct g4p_fwinit *fi, unsigned int ring, const u32 *entry);
u32 g4p_fwinit_ring_read_index(struct g4p_fwinit *fi, unsigned int ring);
u64 g4p_fwinit_list_a_va(struct g4p_fwinit *fi);

/* g4p_boot.c */
int g4p_boot_start(struct g4p *g);
void g4p_boot_free(struct g4p *g);
void g4p_boot_show(struct seq_file *s, struct g4p *g);
int g4p_boot_devctl_nop(struct g4p *g, u32 type);
/* 8.88: C9 graphics harness (g4p_gfx.c) and its hooks in g4p_boot.c */
struct g4p_gfx;
int g4p_gfx_run(struct g4p *g);
ssize_t g4p_gfx_out(struct g4p *g, char __user *ubuf, size_t len, loff_t *pos);
void g4p_gfx_free(struct g4p *g);
bool g4p_boot_alive(struct g4p *g);
u64 g4p_boot_heap_pte(void);
int g4p_boot_fw_map_page(struct g4p *g, u64 va, dma_addr_t dma);
void g4p_boot_fw_pages_changed(struct g4p *g);
int g4p_boot_load_ctx(struct g4p *g, unsigned int slot, u64 pc_dma);
int g4p_boot_ring_kick(struct g4p *g, unsigned int ring, const u32 *entry, u32 kick, unsigned int slot);
u64 g4p_fw_to_gpuva(u32 fwaddr);
int g4p_mmu_heap_of(u64 va);
u32 *g4p_boot_drv_ptr(struct g4p *g, u64 va);
int g4p_boot_cl_c1(struct g4p *g);
int g4p_boot_cl_job(struct g4p *g, unsigned int which);
ssize_t g4p_boot_c8mem(struct g4p *g, void **bufp);
void g4p_boot_irq_off(struct g4p *g);
void g4p_boot_irq_rings(struct g4p *g);
bool g4p_boot_uses_pool(struct g4p *g);
int g4p_boot_map_range(struct g4p *g, unsigned int ctx, u64 va, dma_addr_t dma, size_t size);
void g4p_boot_unmap_range(struct g4p *g, unsigned int ctx, u64 va, size_t size);
int g4p_boot_ctx_attach(struct g4p *g, unsigned int ctx);
int g4p_boot_pool_map(struct g4p *g, unsigned int ctx, unsigned int first, unsigned int n);
void g4p_boot_pool_unmap(struct g4p *g, unsigned int ctx, unsigned int first, unsigned int n);
void g4p_boot_ctx_detach(struct g4p *g, unsigned int ctx);
int g4p_boot_ctx_bind(struct g4p *g, unsigned int ctx);
bool g4p_boot_ctx_resident(struct g4p *g, unsigned int ctx);
int g4p_drm_map_separate(struct g4p *g);	/* g4p_drm.c: map all separate BOs into a new boot */
void g4p_boot_job_result(struct g4p *g, u32 *stamp, u32 *us, u64 *fault);
bool g4p_boot_job_done(struct g4p *g, u32 n);
u32 g4p_boot_next_job(struct g4p *g);
void g4p_boot_job_dbg(struct g4p *g, u32 n);
void g4p_boot_job_retire(struct g4p *g, u32 n);
void g4p_boot_stall_dbg(struct g4p *g);
int g4p_boot_mmu_fault(struct g4p *g);

/* g4p_drv.c */
int g4p_ensure_booted_locked(struct g4p *g);
void g4p_recover_locked(struct g4p *g);
void g4p_idle_off_locked(struct g4p *g);

/* g4p_drm.c */
struct g4p *g4p_drm_alloc(struct device *dev);
int g4p_drm_register(struct g4p *g);
void g4p_drm_unregister(struct g4p *g);
void g4p_drm_pool_free(struct g4p *g);
void g4p_drm_reap(struct g4p *g);
void g4p_drm_irq(struct g4p *g);


#endif
