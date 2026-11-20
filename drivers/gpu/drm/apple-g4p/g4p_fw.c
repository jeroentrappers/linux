// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 GPU (G4P): META firmware container.
 *
 * Apple ships the GPU firmware (g4p_b0_t0/t1) as a bare Imagination META LDR image, the container that
 * upstream drm/imagination parses in pvr_fw_meta.c (re/GPU-BRINGUP.md 3.2). The user extracts it from
 * their own iOS (re/tools/agxfw.py) and installs it as /lib/firmware/apple/agx-g4p-b0.ldr; it is never
 * redistributed.
 *
 * This file only parses and lays out the image; it never touches the hardware. As AGXMetaFirmware does
 * (re/GPU-BRINGUP.md 3.3): LOADMEM copies, ZEROMEM clears, CONFIG type 2 writes become boot arguments,
 * START_THREADS records PC/SP, LOADCORE / LOADMMREG are rejected.
 */

#include <linux/align.h>
#include <linux/crc32.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/minmax.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "g4p_fw.h"

/* Imagination META LDR layout (same as upstream pvr_rogue_meta.h) */
struct ldr_hdr {
	__le32 dev_id;
	__le32 sl_code;
	__le32 sl_data;		/* offset of the first L1 block */
	__le16 pc_ctrl;
	__le16 crc;
} __packed;

struct ldr_l1 {
	__le16 cmd;
	__le16 length;
	__le32 next;		/* offset of the next L1 block, 0xffffffff = end */
	__le32 data[4];
} __packed;

struct ldr_l2 {
	__le16 tag;
	__le16 length;		/* including this header and a trailing 16-bit CRC */
	u8 payload[];
} __packed;

#define LDR_CMD_MASK		0x000f
#define LDR_CMD_COMMENT		0x0010
#define LDR_CMD_LOADMEM		0
#define LDR_CMD_LOADCORE	1
#define LDR_CMD_LOADMMREG	2
#define LDR_CMD_START_THREADS	3
#define LDR_CMD_ZEROMEM		4
#define LDR_CMD_CONFIG		5
#define LDR_CFG_WRITE		2
#define LDR_END			0xffffffff
#define LDR_MAX_BLOCKS		4096

/* AGX layout order: region 2 (the 0x4/0xc bootloader segment) first, then 0, 1, 3 */
static const unsigned int g4p_region_order[G4P_FW_NR_REGIONS] = { 2, 0, 1, 3 };

static unsigned int g4p_region_of(u32 addr)
{
	return (addr >> 29) & 3;
}

/* the L2 payload at @off; NULL if it does not fit */
static const u8 *ldr_l2(const struct firmware *blob, u32 off, size_t *len)
{
	const struct ldr_l2 *l2;
	size_t l;

	if (off > blob->size || blob->size - off < sizeof(*l2) + 2)
		return NULL;
	l2 = (const void *)(blob->data + off);
	l = le16_to_cpu(l2->length);
	if (l < sizeof(*l2) + 2 || l > blob->size - off)
		return NULL;
	*len = l - sizeof(*l2) - 2;
	return l2->payload;
}

static int g4p_fw_note_range(struct device *dev, struct g4p_fw *fw, u32 addr, u32 len)
{
	u32 a = addr & ~BIT(31);
	struct g4p_fw_region *r = &fw->region[g4p_region_of(a)];

	if (!len || a + len < a) {
		dev_err(dev, "firmware: bad range 0x%08x+0x%x\n", addr, len);
		return -EINVAL;
	}
	if (!r->used) {
		r->used = true;
		r->lo = a;
		r->hi = a + len;
	} else {
		r->lo = min(r->lo, a);
		r->hi = max(r->hi, a + len);
	}
	if (g4p_region_of(r->hi - 1) != g4p_region_of(r->lo)) {
		dev_err(dev, "firmware: range 0x%08x+0x%x crosses a region\n", addr, len);
		return -EINVAL;
	}
	return 0;
}

/*
 * Walk the L1 chain. Pass 0 sizes the regions and counts CONFIG writes; pass 1 (image != NULL) copies the
 * memory blocks and fills fw->cfg.
 */
static int g4p_fw_walk(struct g4p_fw *fw, void *image)
{
	struct device *dev = fw->dev;
	const struct firmware *blob = fw->blob;
	const struct ldr_hdr *hdr = (const void *)blob->data;
	u32 off = le32_to_cpu(hdr->sl_data);
	unsigned int n = 0, cfg = 0;
	const u8 *p;
	size_t len;
	int ret;

	while (off != LDR_END) {
		const struct ldr_l1 *b;
		u16 cmd;
		u32 dst;

		if (++n > LDR_MAX_BLOCKS || off > blob->size || blob->size - off < sizeof(*b) || off & 3) {
			dev_err(dev, "firmware: bad L1 chain at 0x%x\n", off);
			return -EINVAL;
		}
		b = (const void *)(blob->data + off);
		cmd = le16_to_cpu(b->cmd);
		dst = le32_to_cpu(b->data[0]);

		if (cmd & LDR_CMD_COMMENT) {
			fw->nr_comments += !image;
			off = le32_to_cpu(b->next);
			continue;
		}
		switch (cmd & LDR_CMD_MASK) {
		case LDR_CMD_LOADMEM:
			p = ldr_l2(blob, le32_to_cpu(b->data[1]), &len);
			if (!p || !len)
				goto bad;
			if (!image) {
				ret = g4p_fw_note_range(dev, fw, dst, len);
				if (ret)
					return ret;
				fw->nr_loads++;
			} else {
				const struct g4p_fw_region *r = &fw->region[g4p_region_of(dst & ~BIT(31))];

				memcpy(image + r->offset + ((dst & ~BIT(31)) - r->lo), p, len);
			}
			break;
		case LDR_CMD_ZEROMEM:
			len = le32_to_cpu(b->data[1]);
			if (!image) {
				ret = g4p_fw_note_range(dev, fw, dst, len);
				if (ret)
					return ret;
				fw->nr_zeros++;
			} else {
				const struct g4p_fw_region *r = &fw->region[g4p_region_of(dst & ~BIT(31))];

				memset(image + r->offset + ((dst & ~BIT(31)) - r->lo), 0, len);
			}
			break;
		case LDR_CMD_CONFIG:
			p = ldr_l2(blob, dst, &len);
			if (!p)
				goto bad;
			for (size_t i = 0; i + 12 <= len; i += 12) {
				u32 t = get_unaligned_le32(p + i);

				if (t != LDR_CFG_WRITE) {
					dev_err(dev, "firmware: CONFIG type %u not supported\n", t);
					return -EINVAL;
				}
				if (image && fw->cfg) {
					fw->cfg[cfg].reg = get_unaligned_le32(p + i + 4);
					fw->cfg[cfg].val = get_unaligned_le32(p + i + 8);
				}
				cfg++;
			}
			break;
		case LDR_CMD_START_THREADS:
			p = ldr_l2(blob, dst, &len);
			if (!p)
				goto bad;
			if (!image) {
				for (size_t i = 0; i + 16 <= len; i += 16) {
					struct g4p_fw_thread *t;

					if (fw->nr_threads >= ARRAY_SIZE(fw->thread))
						goto bad;
					t = &fw->thread[fw->nr_threads++];
					t->thread = get_unaligned_le32(p + i);
					t->sp = get_unaligned_le32(p + i + 4);
					t->pc = get_unaligned_le32(p + i + 8);
				}
			}
			break;
		default:	/* LOADCORE, LOADMMREG: AGX treats them as errors too */
			dev_err(dev, "firmware: L1 command 0x%x not supported\n", cmd);
			return -EINVAL;
		}
		off = le32_to_cpu(b->next);
	}
	if (!image) {
		fw->nr_blocks = n;
		fw->nr_cfg = cfg;
	}
	return 0;
bad:
	dev_err(dev, "firmware: bad L1 block at 0x%x\n", off);
	return -EINVAL;
}

int g4p_fw_request(struct device *dev, const char *name, struct g4p_fw *fw)
{
	const struct ldr_hdr *hdr;
	size_t pos = 0;
	int ret;

	memset(fw, 0, sizeof(*fw));
	fw->dev = dev;
	ret = request_firmware(&fw->blob, name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "firmware %s (extract it from your own iOS: re/tools/agxfw.py)\n",
				     name);
	ret = -EINVAL;
	if (fw->blob->size < sizeof(*hdr))
		goto err;
	hdr = (const void *)fw->blob->data;
	fw->dev_id = le32_to_cpu(hdr->dev_id);
	if (le32_to_cpu(hdr->sl_code) < sizeof(*hdr) || le32_to_cpu(hdr->sl_data) <= le32_to_cpu(hdr->sl_code) ||
	    le32_to_cpu(hdr->sl_data) >= fw->blob->size) {
		dev_err(dev, "firmware: bad LDR header\n");
		goto err;
	}
	ret = g4p_fw_walk(fw, NULL);
	if (ret)
		goto err;
	if (fw->nr_threads == 0 || !fw->region[2].used) {
		dev_err(dev, "firmware: no thread start or no bootloader region\n");
		ret = -EINVAL;
		goto err;
	}
	for (unsigned int i = 0; i < G4P_FW_NR_REGIONS; i++) {
		struct g4p_fw_region *r = &fw->region[g4p_region_order[i]];

		if (!r->used)
			continue;
		r->offset = pos;
		pos += ALIGN(r->hi - r->lo, SZ_4K);
	}
	fw->image_size = pos;
	if (fw->nr_cfg) {
		fw->cfg = kcalloc(fw->nr_cfg, sizeof(*fw->cfg), GFP_KERNEL);
		if (!fw->cfg) {
			ret = -ENOMEM;
			goto err;
		}
	}
	return 0;
err:
	release_firmware(fw->blob);
	fw->blob = NULL;
	return ret;
}

void g4p_fw_release(struct g4p_fw *fw)
{
	kfree(fw->cfg);
	fw->cfg = NULL;
	release_firmware(fw->blob);
	fw->blob = NULL;
}

/* lay the firmware out in @image (fw->image_size bytes); also fills fw->cfg */
int g4p_fw_build_image(const struct g4p_fw *fw, void *image, size_t size)
{
	if (!fw->blob || size < fw->image_size)
		return -EINVAL;
	memset(image, 0, fw->image_size);
	/* the walk only writes into fw->cfg, which the caller owns through @fw */
	return g4p_fw_walk((struct g4p_fw *)fw, image);
}

/* @image: built image or NULL; with it, print a CRC32 per region to check the layout against the host */
void g4p_fw_show(struct seq_file *s, const struct g4p_fw *fw, const void *image)
{
	if (!fw->blob) {
		seq_puts(s, "firmware: not loaded\n");
		return;
	}
	seq_printf(s, "firmware: %zu bytes, dev_id 0x%08x, %u blocks (%u load, %u zero, %u comment), %u config writes\n",
		   fw->blob->size, fw->dev_id, fw->nr_blocks, fw->nr_loads, fw->nr_zeros, fw->nr_comments,
		   fw->nr_cfg);
	for (unsigned int i = 0; i < G4P_FW_NR_REGIONS; i++) {
		const struct g4p_fw_region *r = &fw->region[i];

		if (!r->used)
			continue;
		seq_printf(s, "  region %u: 0x%08x..0x%08x (0x%x) at image +0x%zx", i, r->lo, r->hi,
			   r->hi - r->lo, r->offset);
		if (image)
			seq_printf(s, " crc32 0x%08x", crc32_le(~0, image + r->offset, r->hi - r->lo) ^ ~0);
		seq_putc(s, '\n');
	}
	seq_printf(s, "  image 0x%zx bytes\n", fw->image_size);
	for (unsigned int i = 0; i < fw->nr_threads; i++)
		seq_printf(s, "  thread %u: pc 0x%08x sp 0x%08x\n", fw->thread[i].thread, fw->thread[i].pc,
			   fw->thread[i].sp);
	if (image)
		for (unsigned int i = 0; i < fw->nr_cfg; i++)
			seq_printf(s, "  boot arg %2u: [0x%08x] = 0x%08x\n", i, fw->cfg[i].reg, fw->cfg[i].val);
}
