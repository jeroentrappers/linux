// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: stream configuration (ISP_IOC_START). The command sequence and
 * every packet layout are in re/CAMERA-STREAM.md §3/§5 (fw 13.42.00 dispatcher + kext).
 * Called with isp->lock held; every command is bounded by cmd_timeout_ms.
 */
#include <linux/firmware.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "isp.h"

#define ISP_INFO_SIZE		0xc4	/* kext ISP_InitialSensorDetection */
#define ISP_CAMCFG_SIZE		0x84	/* kext CAMERA_CONFIG_GET */
#define ISP_POOLGET_SIZE	0x110	/* fw 0x16b90 writes up to 15 x 0x10 records from +0x10 */
#define ISP_MAX_PRESETS		16
#define ISP_SETFILE_MAX		SZ_64K

/* Send @size bytes in and out; a non-zero fw status is an error when @must. */
static int isp_ch_cmd(struct t7000_isp *isp, const char *name, void *pkt, u32 size, bool must)
{
	u16 st = 0xffff;
	int ret = isp_cmd_simple(isp, name, pkt, size, size, &st);

	if (ret)
		return ret;
	if (st && must)
		return -EIO;
	return 0;
}

static int isp_ch_simple(struct t7000_isp *isp, const char *name, u16 op, u32 ch, u32 size,
			 u32 a0, u32 a1, bool must)
{
	struct isp_cmd_ch c = { .h.opcode = op, .chan = ch, .arg = { a0, a1 } };

	return isp_ch_cmd(isp, name, &c, size, must);
}

static u16 rd16(const u8 *p, u32 off)
{
	return *(const u16 *)(p + off);
}

static u32 rd32(const u8 *p, u32 off)
{
	return *(const u32 *)(p + off);
}

/* CH_INFO_GET + CAMERA_CONFIG_GET per preset; picks the preset to stream. */
static int isp_pick_preset(struct t7000_isp *isp, u8 *buf)
{
	struct isp_stream *st = &isp->st;
	u32 p, best = U32_MAX, best_area = U32_MAX, ow, oh, small = U32_MAX, small_area = U32_MAX;
	u16 small_dim[4] = {};
	int ret;

	memset(buf, 0, ISP_INFO_SIZE);
	((struct isp_cmd_hdr *)buf)->opcode = CISP_CMD_CH_INFO_GET;
	*(u32 *)(buf + 8) = st->ch;
	ret = isp_ch_cmd(isp, "CH_INFO_GET", buf, ISP_INFO_SIZE, true);
	if (ret)
		return ret;
	st->sensor_id = rd32(buf, 0x20);
	st->npresets = rd32(buf, 0x60);
	dev_info(isp->dev, "ch %u: sensor 0x%04x, %u preset(s), ts freq %u, module '%.18s'\n",
		 st->ch, st->sensor_id, st->npresets, rd32(buf, 0x98), buf + 0x9e);
	print_hex_dump(KERN_INFO, "apple-isp: CH_INFO: ", DUMP_PREFIX_OFFSET, 32, 4, buf,
		       ISP_INFO_SIZE, false);
	if (!st->npresets) {
		dev_err(isp->dev, "ch %u: no sensor (0 presets): sensor power/clock problem?\n", st->ch);
		return -ENODEV;
	}
	snprintf(st->sensor_name, sizeof(st->sensor_name), "%s",
		 st->sensor_id == 0x237 ? "IMX237 rear" :
		 st->sensor_id == 0x190 ? "IMX190 front" : "unknown");

	for (p = 0; p < min_t(u32, st->npresets, ISP_MAX_PRESETS); p++) {
		memset(buf, 0, ISP_CAMCFG_SIZE);
		((struct isp_cmd_hdr *)buf)->opcode = CISP_CMD_CH_CAMERA_CONFIG_GET;
		*(u32 *)(buf + 8) = st->ch;
		*(u32 *)(buf + 0xc) = p;
		ret = isp_ch_cmd(isp, "CAMERA_CONFIG_GET", buf, ISP_CAMCFG_SIZE, true);
		if (ret)
			return ret;
		ow = rd16(buf, 0x14);
		oh = rd16(buf, 0x16);
		dev_info(isp->dev, "ch %u preset %u: in %ux%u out %ux%u, +0x18 %04x %04x +0x1c %04x %04x\n",
			 st->ch, p, rd16(buf, 0x10), rd16(buf, 0x12), ow, oh, rd16(buf, 0x18),
			 rd16(buf, 0x1a), rd16(buf, 0x1c), rd16(buf, 0x1e));
		if (!ow || !oh || ow > 4096 || oh > 4096)
			continue;
		if (ow * oh < small_area) {	/* fallback when nothing is >= 640x480 */
			small = p;
			small_area = ow * oh;
			small_dim[0] = rd16(buf, 0x10);
			small_dim[1] = rd16(buf, 0x12);
			small_dim[2] = ow;
			small_dim[3] = oh;
		}
		if (isp_preset >= 0 ? p == isp_preset :
		    ow >= 640 && oh >= 480 && ow * oh < best_area) {
			best = p;
			best_area = ow * oh;
			st->in_w = rd16(buf, 0x10);
			st->in_h = rd16(buf, 0x12);
			st->width = ow;
			st->height = oh;
		}
	}
	if (best == U32_MAX && isp_preset < 0 && small != U32_MAX) {
		best = small;
		st->in_w = small_dim[0];
		st->in_h = small_dim[1];
		st->width = small_dim[2];
		st->height = small_dim[3];
	}
	if (best == U32_MAX) {
		dev_err(isp->dev, "ch %u: no usable preset (preset=%d)\n", st->ch, isp_preset);
		return -EINVAL;
	}
	st->preset = best;
	st->stride = ALIGN(st->width, 64);
	st->bufsize = PAGE_ALIGN(st->stride * st->height * 3 / 2);
	dev_info(isp->dev, "ch %u: streaming preset %u: %ux%u NV12, stride %u, buffer 0x%x\n",
		 st->ch, best, st->width, st->height, st->stride, st->bufsize);
	return 0;
}

/* Optional tuning data (Asahi: a missing setfile only costs calibration). */
static void isp_load_setfile(struct t7000_isp *isp)
{
	struct isp_stream *st = &isp->st;
	u32 mod = st->ch ? isp_setfile_front : isp_setfile_rear;
	const struct firmware *fw;
	char name[40];
	u32 magic;

	if (!mod)
		return;
	snprintf(name, sizeof(name), "apple/isp_%04x_01XX.dat", mod);
	if (request_firmware_direct(&fw, name, isp->dev)) {
		dev_warn(isp->dev, "setfile %s not found: streaming without calibration\n", name);
		return;
	}
	magic = fw->size >= 4 ? be32_to_cpup((const __be32 *)fw->data) : 0;
	if (fw->size > ISP_SETFILE_MAX || magic >> 16 != mod) {
		dev_warn(isp->dev, "setfile %s: size 0x%zx magic 0x%08x does not match, skipped\n",
			 name, fw->size, magic);
		goto out;
	}
	if (!st->setf)
		st->setf = isp_surf_alloc(isp, ISP_SETFILE_MAX, 0x46544553 /* "SETF" */);
	if (!st->setf)
		goto out;
	memcpy(st->setf->virt, fw->data, fw->size);
	dma_wmb();
	dev_info(isp->dev, "setfile %s: 0x%zx bytes, magic 0x%08x\n", name, fw->size, magic);
	isp_ch_simple(isp, "CH_SET_FILE_LOAD", CISP_CMD_CH_SET_FILE_LOAD, st->ch, 0x14,
		      isp_rel(st->setf->iova), fw->size, false);
out:
	release_firmware(fw);
}

/* BUFFER_POOL_CONFIG_GET (logged) gives the META pool size; then POOL_CONFIG_SET(META). */
static void isp_config_meta(struct t7000_isp *isp, u8 *buf)
{
	struct isp_stream *st = &isp->st;
	u32 i, off;

	memset(buf, 0, ISP_POOLGET_SIZE);
	((struct isp_cmd_hdr *)buf)->opcode = CISP_CMD_CH_BUFFER_POOL_CONFIG_GET;
	*(u32 *)(buf + 8) = st->ch;
	st->meta_size = 0;
	if (isp_ch_cmd(isp, "BUFFER_POOL_CONFIG_GET", buf, ISP_POOLGET_SIZE, true))
		return;
	for (i = 0; i < 15; i++) {
		off = 0x10 + i * 0x10;
		if (!rd32(buf, off) && !rd32(buf, off + 4) && !rd32(buf, off + 8))
			continue;
		dev_info(isp->dev, "pool record %u: type %u id %u %04x count %u size 0x%x %04x %04x\n",
			 i, rd16(buf, off), rd16(buf, off + 2), rd16(buf, off + 4),
			 rd16(buf, off + 6), rd32(buf, off + 8), rd16(buf, off + 0xc),
			 rd16(buf, off + 0xe));
		if (rd16(buf, off) == ISP_POOL_META && rd32(buf, off + 8) <= SZ_1M)
			st->meta_size = rd32(buf, off + 8);
	}
	if (!st->meta_size) {
		dev_warn(isp->dev, "no META pool reported: not submitting meta buffers\n");
		return;
	}
	/* fw 0x16b64 reads: u16 pool type @0xc, u32 @0x10, u32 @0x14 (Asahi meta_size0/1) */
	memset(buf, 0, 0x9c);
	((struct isp_cmd_hdr *)buf)->opcode = CISP_CMD_CH_BUFFER_POOL_CONFIG_SET;
	*(u32 *)(buf + 8) = st->ch;
	*(u16 *)(buf + 0xc) = ISP_POOL_META;
	*(u16 *)(buf + 0xe) = clamp_t(u32, isp_nbufs, 2, ISP_MAX_BUFS);
	*(u32 *)(buf + 0x10) = st->meta_size;
	*(u32 *)(buf + 0x14) = st->meta_size;
	isp_ch_cmd(isp, "BUFFER_POOL_CONFIG_SET(META)", buf, 0x9c, false);
}

/*
 * Frame (+ meta) surfaces and the H2T list area. Surfaces can only be freed after the
 * ISP is powered off, so they are allocated once per boot, sized @want (the larger of
 * the two cameras, see isp_stream_start), and reused by later STARTs. If a START still
 * needs more, a bigger set is allocated and the old one stays mapped (unused) until
 * power-off. st->bufsize = the real buffer size = the mmap stride between buffers.
 */
static int isp_alloc_bufs(struct t7000_isp *isp, u32 want)
{
	struct isp_stream *st = &isp->st;
	u32 n = clamp_t(u32, isp_nbufs, 2, ISP_MAX_BUFS), i, need = st->bufsize;
	bool frames, metas;

	frames = !st->nbufs || st->buf[0].surf->size < need;
	metas = st->meta_size && (frames || !st->meta[0].surf ||
				  st->meta[0].surf->size < st->meta_size);
	if (st->nbufs && frames)
		dev_warn(isp->dev, "buffers of this boot are 0x%x bytes, preset needs 0x%x: allocating a bigger set (the old one is freed at power-off)\n",
			 st->buf[0].surf->size, need);
	if (!st->lists)
		st->lists = isp_surf_alloc(isp, isp->bh->num * ISP_BUFLIST_SLOT,
					   0x4c465542 /* "BUFL" */);
	if (!st->lists)
		return -ENOMEM;
	if (frames) {
		st->nbufs = 0;	/* until the new set is complete */
		for (i = 0; i < n; i++) {
			st->buf[i].surf = isp_surf_alloc(isp, max(want, need), 0x304d5246 + (i << 24));
			if (!st->buf[i].surf)
				return -ENOMEM;
			st->buf[i].state = BUF_FREE;
		}
	} else {
		n = st->nbufs;
	}
	for (i = 0; metas && i < n; i++) {
		st->meta[i].surf = isp_surf_alloc(isp, st->meta_size, 0x3041544d + (i << 24));
		if (!st->meta[i].surf)
			return -ENOMEM;
		st->meta[i].state = BUF_FREE;
	}
	st->nbufs = n;
	st->bufsize = st->buf[0].surf->size;
	return 0;
}

struct isp_cmd_output_config {
	struct isp_cmd_hdr h;
	u32 chan, width, height, colorspace, format;
	u32 strides[3];
	u32 padding_rows, unk_h0, compress, unk_w2;
} __packed;
static_assert(sizeof(struct isp_cmd_output_config) == 0x38);

/* ISP_IOC_START: camera 0 = rear = fw channel 0, 1 = front = channel 1 (kext row 0x2a). */
int isp_stream_start(struct t7000_isp *isp, u32 camera)
{
	struct isp_stream *st = &isp->st;
	struct isp_cmd_output_config oc = { .h.opcode = CISP_CMD_CH_OUTPUT_CONFIG_SET };
	u32 want;
	uint quiet;
	u8 *buf;
	int ret;

	if (camera > 1)
		return -EINVAL;
	if (st->on)
		return -EBUSY;
	if (!isp->started || isp->io_dead || !isp->bh || !isp->bt) {
		dev_err(isp->dev, "stream: fw not started or BUF_H2T/BUF_T2H missing\n");
		return -ENODEV;
	}
	buf = kzalloc(ISP_POOLGET_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	/*
	 * First START of this boot: size the buffers for the other camera too, so a
	 * rear <-> front switch on the same fd reuses them (no new allocation).
	 */
	want = 0;
	if (!st->nbufs) {
		st->ch = !camera;
		if (!isp_pick_preset(isp, buf))
			want = st->bufsize;
		else
			dev_info(isp->dev, "camera %u not usable: sizing the buffers for camera %u only\n",
				 !camera, camera);
	}
	st->ch = camera;
	ret = isp_pick_preset(isp, buf);
	if (ret)
		goto out;
	isp_load_setfile(isp);
	ret = isp_ch_simple(isp, "CAMERA_CONFIG_SELECT", CISP_CMD_CH_CAMERA_CONFIG_SELECT,
			    st->ch, 0x10, st->preset, 0, true);
	if (!ret)
		ret = isp_ch_simple(isp, "BUFFER_RECYCLE_MODE_SET", CISP_CMD_CH_BUFFER_RECYCLE_MODE_SET,
				    st->ch, 0x10, 1, 0, false);
	if (!ret)
		ret = isp_ch_simple(isp, "BUFFER_RECYCLE_START", CISP_CMD_CH_BUFFER_RECYCLE_START,
				    st->ch, 0xc, 0, 0, false);
	if (ret)
		goto out;
	{
		struct isp_cmd_ch c = { .h.opcode = CISP_CMD_CH_CROP_SET, .chan = st->ch,
					.arg = { 0, 0, st->in_w, st->in_h } };

		ret = isp_ch_cmd(isp, "CROP_SET", &c, 0x1c, false);
		if (ret)
			goto out;
	}
	oc.chan = st->ch;
	oc.width = st->width;
	oc.height = st->height;
	/* fw 0x1b99c logs this field as "range" ("format = %d, range %d"): Asahi calls it colorspace */
	oc.colorspace = isp_out_range;
	oc.format = 0;			/* 2-plane YUV 4:2:0 = NV12 (fw 0x162b8) */
	oc.strides[0] = st->stride;
	oc.strides[1] = st->stride;
	oc.unk_h0 = st->height;
	oc.unk_w2 = st->width;
	ret = isp_ch_cmd(isp, "OUTPUT_CONFIG_SET", &oc, sizeof(oc), true);
	if (!ret)
		ret = isp_ch_simple(isp, "PREVIEW_STREAM_SET", CISP_CMD_CH_PREVIEW_STREAM_SET,
				    st->ch, 0x10, 1, 0, true);
	if (ret)
		goto out;
	isp_ch_simple(isp, "SIF_PIXEL_FORMAT_SET", CISP_CMD_CH_SIF_PIXEL_FORMAT_SET, st->ch, 0x14,
		      3 | 1 << 8, 0, false);	/* u8 format 3, u8 type 1 (Asahi) */
	isp_config_meta(isp, buf);
	ret = isp_alloc_bufs(isp, want);
	if (ret)
		goto out;
	st->seq = 0;
	st->nready = 0;
	st->stopping = false;
	st->on = true;
	if (READ_ONCE(isp_stream_nostart)) {
		dev_info(isp->dev, "stream_nostart=1: configured, no buffers posted, no CH_START\n");
		goto out;
	}
	isp_stream_kick(isp);		/* every buffer is FREE: one or two lists on BUF_H2T */
	if (READ_ONCE(isp_stream_nobufs))
		dev_info(isp->dev, "stream_nobufs=1: CH_START with no buffer posted (only fw-allocated DMA, e.g. 'RAW0')\n");
	isp_canary_arm(isp);		/* canary=1: before CH_START, so it is logged even if the SoC wedges */
	quiet = READ_ONCE(isp_quiet_ms);
	isp->skip_final_service = quiet != 0;
	ret = isp_ch_simple(isp, "CH_START", CISP_CMD_CH_START, st->ch, 0xc, 0, 0, true);
	isp->skip_final_service = false;
	if (!ret && quiet)
		isp_quiet_window(isp, quiet);
	if (ret) {
		isp_stream_stop(isp);
		goto out;
	}
	dev_info(isp->dev, "stream on: camera %u (%s 0x%04x), %ux%u, %u buffer(s)%s\n", camera,
		 st->sensor_name, st->sensor_id, st->width, st->height, st->nbufs,
		 st->meta_size ? " + meta" : "");
out:
	kfree(buf);
	return ret;
}
