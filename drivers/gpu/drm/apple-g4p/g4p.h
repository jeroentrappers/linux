/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple A8 GPU (G4P): shared driver state.
 */
#ifndef G4P_H
#define G4P_H

#include <linux/mutex.h>
#include <linux/types.h>

#include "g4p_fw.h"

struct dentry;
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
	struct g4p_boot *boot;			/* GPU memory of a META boot; freed after power-off */
};

/* g4p_boot.c */
int g4p_boot_start(struct g4p *g);
void g4p_boot_free(struct g4p *g);
void g4p_boot_show(struct seq_file *s, struct g4p *g);


#endif
