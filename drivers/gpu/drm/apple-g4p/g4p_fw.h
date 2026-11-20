/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple A8 GPU (G4P): META firmware container (Imagination META LDR, as Apple ships it).
 */
#ifndef G4P_FW_H
#define G4P_FW_H

#include <linux/types.h>

struct device;
struct firmware;
struct seq_file;

/*
 * AGX buckets every LDR address by bits [30:29] into four regions and lays them out in one allocation as
 * r2 | r0 | r1 | r3 (re/GPU-BRINGUP.md 3.3). Bit 31 is META's global alias and is ignored for placement.
 */
#define G4P_FW_NR_REGIONS	4

struct g4p_fw_region {
	bool used;
	u32 lo, hi;		/* META addresses (bit 31 cleared), hi exclusive */
	size_t offset;		/* in the image */
};

struct g4p_fw_cfg {		/* LDR CONFIG type 2 (register write): becomes a boot argument */
	u32 reg, val;
};

struct g4p_fw_thread {		/* LDR START_THREADS entry */
	u32 thread, sp, pc;
};

struct g4p_fw {
	struct device *dev;
	const struct firmware *blob;
	u32 dev_id;
	unsigned int nr_blocks, nr_loads, nr_zeros, nr_comments;
	struct g4p_fw_region region[G4P_FW_NR_REGIONS];
	size_t image_size;
	struct g4p_fw_cfg *cfg;
	unsigned int nr_cfg;
	struct g4p_fw_thread thread[2];
	unsigned int nr_threads;
};

int g4p_fw_request(struct device *dev, const char *name, struct g4p_fw *fw);
void g4p_fw_release(struct g4p_fw *fw);
int g4p_fw_build_image(const struct g4p_fw *fw, void *image, size_t size);
void g4p_fw_show(struct seq_file *s, const struct g4p_fw *fw, const void *image);

#endif
