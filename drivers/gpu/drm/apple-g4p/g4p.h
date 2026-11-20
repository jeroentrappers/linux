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

#include "g4p_fw.h"

struct dentry;
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

struct g4p {
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

struct g4p_mmu {
	u32 *pc;
	u64 *pd;
	dma_addr_t pc_dma, pd_dma;
	unsigned int pc_index;
	u64 *pt[512];
	dma_addr_t pt_dma[512];
	u64 heap_next[G4P_NR_HEAPS];
};

struct g4p_bo {
	void *cpu;
	dma_addr_t dma;
	size_t size;
	u64 gpuva;
};

int g4p_mmu_init(struct device *dev, struct g4p_mmu *mmu);
void g4p_mmu_fini(struct device *dev, struct g4p_mmu *mmu);
int g4p_mmu_map(struct device *dev, struct g4p_mmu *mmu, u64 va, phys_addr_t pa, size_t size, u64 flags);
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
int g4p_fwinit_devctl_push(struct g4p_fwinit *fi, const u32 *entry);
u32 g4p_fwinit_devctl_read_index(struct g4p_fwinit *fi);

/* g4p_boot.c */
int g4p_boot_start(struct g4p *g);
void g4p_boot_free(struct g4p *g);
void g4p_boot_show(struct seq_file *s, struct g4p *g);
int g4p_boot_devctl_nop(struct g4p *g, u32 type);
void g4p_boot_irq_off(struct g4p *g);


#endif
