// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: stream configuration (ISP_IOC_START). The command sequence and
 * every packet layout are in re/CAMERA-STREAM.md §3/§5 (fw 13.42.00 dispatcher + kext).
 * Called with isp->lock held; every command is bounded by cmd_timeout_ms.
 */
#include <linux/firmware.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include <uapi/linux/apple-isp-t7000.h>
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

/*
 * Optional pre-CH_START commands (one knob each, all off by default) to bisect why the fw
 * returns every buffer empty. Opcodes are in the fw 13.42 dispatcher (re/isp/fw-cmd-table.txt);
 * layouts from iOS 12 H6ISP.mediacapture (H6ISPDevice::*) and Asahi isp-cmd.c:
 *  - iOS H6ISPFrameReceiver::startReceive, right before StartCamera (CH_START):
 *    ALS_ENABLE 0xa1c (+0xc method, iOS 1), AE_FRAME_RATE_MAX/MIN_SET 0x208/0x20a (+0xc u16
 *    fps * 256), EnableTNR = TEMPORAL_FILTER_START 0xc100 (+ MOTION_HISTORY_START 0xc102),
 *    then CH_START, then the buffers (ISP_SendBuffers) -> bufs_after_start=1.
 *  - Asahi isp_ch_configure_capture: CNR_START 0xa2f, AE_METERING_MODE_SET 0x8206 (3),
 *    AE_STABILITY_SET 0x21a (32), AE_STABILITY_TO_STABLE_SET 0x229 (20), TEMPORAL_FILTER_ENABLE
 *    0xc113. AE_START 0x200 (iOS EnableAE) is in neither start path but exists here.
 * SBS_ENABLE 0x13b / MBNR 0xa3a do not exist in this fw.
 */
static uint isp_view_w, isp_view_h;	/* experiment: the viewfinder preset's output size (isp_pick_preset) */
module_param_named(view_w, isp_view_w, uint, 0644);
module_param_named(view_h, isp_view_h, uint, 0644);
static uint isp_fps_max;	/* 0 = off; 7680 = 30 fps (Asahi ISP_FRAME_RATE_DEN) */
module_param_named(fps_max, isp_fps_max, uint, 0644);
static uint isp_fps_min;	/* 0 = off; 3840 = 15 fps (Asahi ISP_FRAME_RATE_DEN2) */
module_param_named(fps_min, isp_fps_min, uint, 0644);
static int isp_als = -1;	/* -1 = off; iOS startReceive sends method 1 */
module_param_named(als, isp_als, int, 0644);
static uint isp_tnr;		/* bit0 TF_START 0xc100, bit1 MH_START 0xc102, bit2 TF_ENABLE 0xc113 */
module_param_named(tnr, isp_tnr, uint, 0644);
static bool isp_cnr;		/* CNR_START 0xa2f */
module_param_named(cnr, isp_cnr, bool, 0644);
static int isp_ae_meter = -1;	/* AE_METERING_MODE_SET 0x8206; Asahi 3 */
module_param_named(ae_meter, isp_ae_meter, int, 0644);
static int isp_ae_stab = -1;	/* AE_STABILITY_SET 0x21a; Asahi 32 */
module_param_named(ae_stab, isp_ae_stab, int, 0644);
static int isp_ae_stab2 = -1;	/* AE_STABILITY_TO_STABLE_SET 0x229; Asahi 20 */
module_param_named(ae_stab2, isp_ae_stab2, int, 0644);
static bool isp_ae_start;	/* AE_START 0x200 */
module_param_named(ae_start, isp_ae_start, bool, 0644);
/*
 * Full-resolution presets (no binning) make the fw run the back end from a RAW frame in
 * memory: CH_START asserts "CImageFlowH6.cpp, 2185: chEntryApple->icRawBufferOut != 0"
 * (hw, 2026-10-21) unless it may use its own RAW buffer ('0WAR', SHAREDMALLOC at boot).
 * iOS H6ISPCaptureStreamStart sends EnableLocalRawBufferMode(ch, 1) = 0x125 for that.
 * -1 = for ISP_START_STILL only, 0 = never, 1 = every START.
 */
static int isp_raw_local;
module_param_named(raw_local, isp_raw_local, int, 0644);
MODULE_PARM_DESC(raw_local, "LOCAL_RAW_BUFFER_MODE 0x125 before CH_START (-1 = STILL starts only; accepted but not what the assert wants)");

/* host RAW pool (fw type 3) buffers for STILL starts: not needed (the fw's RAW is a SHAREDMALLOC) */
static bool isp_raw_pool;
module_param_named(raw_pool, isp_raw_pool, bool, 0644);

static bool isp_bufs_after_start;	/* iOS order: CH_START first, then post the buffers */
module_param_named(bufs_after_start, isp_bufs_after_start, bool, 0644);

/* Asahi-style tuning, after PREVIEW_STREAM_SET / SIF; never fatal (status logged). */
static void isp_extra_cfg(struct t7000_isp *isp, u32 ch)
{
	if (READ_ONCE(isp_cnr))
		isp_ch_simple(isp, "CNR_START", 0xa2f, ch, 0xc, 0, 0, false);
	if (READ_ONCE(isp_ae_meter) >= 0)
		isp_ch_simple(isp, "AE_METERING_MODE_SET", 0x8206, ch, 0x10, isp_ae_meter, 0, false);
	if (READ_ONCE(isp_ae_stab) >= 0)
		isp_ch_simple(isp, "AE_STABILITY_SET", 0x21a, ch, 0x10, isp_ae_stab, 0, false);
	if (READ_ONCE(isp_ae_stab2) >= 0)
		isp_ch_simple(isp, "AE_STABILITY_TO_STABLE_SET", 0x229, ch, 0x10, isp_ae_stab2, 0, false);
}

/* iOS startReceive order, right before CH_START; never fatal. */
static void isp_extra_prestart(struct t7000_isp *isp, u32 ch)
{
	uint tnr = READ_ONCE(isp_tnr);

	if (READ_ONCE(isp_ae_start))
		isp_ch_simple(isp, "AE_START", 0x200, ch, 0xc, 0, 0, false);
	if (READ_ONCE(isp_als) >= 0)
		isp_ch_simple(isp, "ALS_ENABLE", 0xa1c, ch, 0x10, isp_als, 0, false);
	if (READ_ONCE(isp_fps_max))
		isp_ch_simple(isp, "AE_FRAME_RATE_MAX_SET", 0x208, ch, 0x10, isp_fps_max, 0, false);
	if (READ_ONCE(isp_fps_min))
		isp_ch_simple(isp, "AE_FRAME_RATE_MIN_SET", 0x20a, ch, 0x10, isp_fps_min, 0, false);
	if (tnr & 1)
		isp_ch_simple(isp, "TEMPORAL_FILTER_START", 0xc100, ch, 0xc, 0, 0, false);
	if (tnr & 2)
		isp_ch_simple(isp, "MOTION_HISTORY_START", 0xc102, ch, 0xc, 0, 0, false);
	if (tnr & 4)
		isp_ch_simple(isp, "TEMPORAL_FILTER_ENABLE", 0xc113, ch, 0xc, 0, 0, false);
}

/*
 * Preset choice for st->mode (ISP_START_*): the explicit index; the largest output (STILL);
 * the smallest output >= 640x480 (default, PHOTO_VIEW). PHOTO_VIEW also wants the aspect
 * ratio of the largest preset (st->full_w x st->full_h, set by the first pass) within 8 %.
 */
static bool isp_preset_ok(struct isp_stream *st, u32 p, u32 ow, u32 oh, u32 best_area)
{
	u64 a, b;

	if (st->mode & ISP_START_PRESET)
		return p == ((st->mode >> ISP_START_PRESET_SHIFT) & 0xff);
	if (st->mode & ISP_START_STILL)
		return ow == st->full_w && oh == st->full_h && best_area == U32_MAX;
	if (!(st->mode & ISP_START_PHOTO_VIEW) && isp_preset >= 0)
		return p == isp_preset;
	if (ow < 640 || oh < 480 || ow * oh >= best_area)
		return false;
	if (!(st->mode & ISP_START_PHOTO_VIEW) || !st->full_w)
		return true;
	a = (u64)ow * st->full_h * 100;		/* ow/oh vs full_w/full_h, cross-multiplied */
	b = (u64)st->full_w * oh;
	return a <= b * 108 && a * 108 >= b * 100 * 100;
}

/* CH_INFO_GET + CAMERA_CONFIG_GET per preset; picks the preset to stream (isp_preset_ok). */
static int isp_pick_preset(struct t7000_isp *isp, u8 *buf)
{
	struct isp_stream *st = &isp->st;
	u32 p, n, best = U32_MAX, best_area = U32_MAX, small = U32_MAX, small_area = U32_MAX;
	u32 full_area = 0;
	u16 dim[ISP_MAX_PRESETS][4] = {};	/* in w, in h, out w, out h */
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
	n = min_t(u32, st->npresets, ISP_MAX_PRESETS);
	st->full_w = st->full_h = 0;
	for (p = 0; p < n; p++) {
		memset(buf, 0, ISP_CAMCFG_SIZE);
		((struct isp_cmd_hdr *)buf)->opcode = CISP_CMD_CH_CAMERA_CONFIG_GET;
		*(u32 *)(buf + 8) = st->ch;
		*(u32 *)(buf + 0xc) = p;
		ret = isp_ch_cmd(isp, "CAMERA_CONFIG_GET", buf, ISP_CAMCFG_SIZE, true);
		if (ret)
			return ret;
		dev_dbg(isp->dev, "ch %u preset %u: in %ux%u out %ux%u, +0x18 %04x %04x +0x1c %04x %04x\n",
			st->ch, p, rd16(buf, 0x10), rd16(buf, 0x12), rd16(buf, 0x14), rd16(buf, 0x16),
			rd16(buf, 0x18), rd16(buf, 0x1a), rd16(buf, 0x1c), rd16(buf, 0x1e));
		if (!rd16(buf, 0x14) || !rd16(buf, 0x16) || rd16(buf, 0x14) > 4096 || rd16(buf, 0x16) > 4096)
			continue;
		dim[p][0] = rd16(buf, 0x10);
		dim[p][1] = rd16(buf, 0x12);
		dim[p][2] = rd16(buf, 0x14);
		dim[p][3] = rd16(buf, 0x16);
		/*
		 * IMX190 (front, 1.2 MP) preset 3 claims 2592x1936: CH_START takes it, but no
		 * frame ever comes and the fw stops answering (CH_STOP times out; hw
		 * 2026-10-21). Its real full resolution is 1376x968 (presets 0-2).
		 */
		if (st->sensor_id == 0x190 && (u32)dim[p][2] * dim[p][3] > SZ_2M) {
			dim[p][2] = dim[p][3] = 0;
			continue;
		}
		if ((u32)dim[p][2] * dim[p][3] > full_area) {	/* the full-resolution preset */
			full_area = (u32)dim[p][2] * dim[p][3];
			st->full_w = dim[p][2];
			st->full_h = dim[p][3];
		}
	}
	for (p = 0; p < n; p++) {
		u32 area = (u32)dim[p][2] * dim[p][3];

		if (!area)
			continue;
		if (area < small_area) {	/* fallback when nothing is >= 640x480 */
			small = p;
			small_area = area;
		}
		if (isp_preset_ok(st, p, dim[p][2], dim[p][3], best_area)) {
			best = p;
			best_area = area;
		}
	}
	if (best == U32_MAX && !(st->mode & ISP_START_PRESET) && (st->mode || isp_preset < 0))
		best = small;
	if (best == U32_MAX) {
		dev_err(isp->dev, "ch %u: no usable preset (mode 0x%x, preset=%d)\n", st->ch, st->mode,
			isp_preset);
		return -EINVAL;
	}
	st->preset = best;
	st->in_w = dim[best][0];
	st->in_h = dim[best][1];
	st->width = dim[best][2];
	st->height = dim[best][3];
	/*
	 * view_w x view_h (experiment, 0 = off): the viewfinder preset's output scaled by the ISP to this size
	 * (OUTPUT_CONFIG_SET with CROP_SET = the whole input) instead of the preset's own. Orchard's 3:4
	 * viewfinder is 750x1000 = a 1000x750 landscape stream: 1.1 MB per frame instead of 3.0 MB and no CPU
	 * decimation (the ISP's scaler filters; nearest-neighbour picking 61 % of the rows/columns aliases).
	 * Only when it shrinks the picture, keeps it even and keeps the aspect within 1 %.
	 */
	if ((st->mode & ISP_START_PHOTO_VIEW) && isp_view_w && isp_view_h && !(isp_view_w & 1) &&
	    !(isp_view_h & 1) && isp_view_w < st->width && isp_view_h < st->height &&
	    abs((int)(isp_view_w * st->height) - (int)(isp_view_h * st->width)) * 100 <=
	    (int)(isp_view_h * st->width)) {
		dev_info(isp->dev, "ch %u: viewfinder output scaled %ux%u -> %ux%u (view_w/view_h)\n", st->ch,
			 st->width, st->height, isp_view_w, isp_view_h);
		st->width = isp_view_w;
		st->height = isp_view_h;
	}
	st->stride = ALIGN(st->width, 64);
	st->bufsize = PAGE_ALIGN(st->stride * st->height * 3 / 2);
	dev_info(isp->dev, "ch %u: streaming preset %u (mode 0x%x): %ux%u NV12, stride %u, buffer 0x%x\n",
		 st->ch, best, st->mode, st->width, st->height, st->stride, st->bufsize);
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
	*(u32 *)(buf + 0xc) = 15;	/* fw 0x16b90: max records; 0 = "Not all the supported pools are reported!" after 1 */
	st->meta_size = 0;
	st->pool_id[ISP_POOL_META] = 1;		/* ch 0 ids seen on hw: META 1; fw: id = ch * n + index + 1 */
	st->pool_id[ISP_POOL_RENDERED] = 2;
	st->pool_id[ISP_POOL_RAW] = 4;
	st->raw_size = 0;
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
		if (rd16(buf, off) <= ISP_POOL_RENDERED)	/* type 0 META, 1 RENDERED */
			st->pool_id[rd16(buf, off)] = rd16(buf, off + 2);
		if (rd16(buf, off) == ISP_FW_POOL_RAW && rd32(buf, off + 8) <= SZ_32M) {
			st->pool_id[ISP_POOL_RAW] = rd16(buf, off + 2);
			st->raw_size = rd32(buf, off + 8);
		}
	}
	if (isp_pool_meta >= 0)
		st->pool_id[ISP_POOL_META] = isp_pool_meta;
	if (isp_pool_frame >= 0)
		st->pool_id[ISP_POOL_RENDERED] = isp_pool_frame;
	dev_info(isp->dev, "pool ids: META %u, RENDERED %u\n", st->pool_id[ISP_POOL_META],
		 st->pool_id[ISP_POOL_RENDERED]);
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
	if (!(st->mode & ISP_START_STILL) || !st->raw_size || !READ_ONCE(isp_raw_pool)) {
		st->raw_size = 0;	/* binned presets stream without host RAW buffers */
		return;
	}
	memset(buf, 0, 0x9c);
	((struct isp_cmd_hdr *)buf)->opcode = CISP_CMD_CH_BUFFER_POOL_CONFIG_SET;
	*(u32 *)(buf + 8) = st->ch;
	*(u16 *)(buf + 0xc) = ISP_FW_POOL_RAW;
	*(u16 *)(buf + 0xe) = ISP_MAX_RAW;
	*(u32 *)(buf + 0x10) = st->raw_size;
	*(u32 *)(buf + 0x14) = st->raw_size;
	isp_ch_cmd(isp, "BUFFER_POOL_CONFIG_SET(RAW)", buf, 0x9c, false);
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
	st->nraw = 0;
	for (i = 0; st->raw_size && i < ISP_MAX_RAW; i++) {
		if (!st->raw[i].surf || st->raw[i].surf->size < st->raw_size)
			st->raw[i].surf = isp_surf_alloc(isp, st->raw_size, 0x30574152 + (i << 24));
		if (!st->raw[i].surf)
			return -ENOMEM;
		if (st->raw[i].state != BUF_FW)	/* (still held after a timed-out STOP) */
			st->raw[i].state = BUF_FREE;
	}
	if (st->raw_size)
		st->nraw = ISP_MAX_RAW;
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
int isp_stream_start(struct t7000_isp *isp, u32 arg)
{
	u32 camera = arg & ISP_START_CAMERA_MASK, mode = arg & ~ISP_START_CAMERA_MASK;
	struct isp_stream *st = &isp->st;
	struct isp_cmd_output_config oc = { .h.opcode = CISP_CMD_CH_OUTPUT_CONFIG_SET };
	u32 want;
	uint quiet;
	u8 *buf;
	int ret;

	if (camera > 1 || hweight32(mode & (ISP_START_PHOTO_VIEW | ISP_START_STILL | ISP_START_PRESET)) > 1 ||
	    (mode & ~(ISP_START_PHOTO_VIEW | ISP_START_STILL | ISP_START_PRESET |
		      0xffu << ISP_START_PRESET_SHIFT)) ||
	    (!(mode & ISP_START_PRESET) && (mode >> ISP_START_PRESET_SHIFT)))
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
	st->mode = mode & ISP_START_PRESET ? 0 : mode;	/* (an index is per camera) */
	if (!st->nbufs) {
		st->ch = !camera;
		if (!isp_pick_preset(isp, buf))
			want = st->bufsize;
		else
			dev_info(isp->dev, "camera %u not usable: sizing the buffers for camera %u only\n",
				 !camera, camera);
	}
	st->ch = camera;
	st->mode = mode;
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
	{
		int rl = READ_ONCE(isp_raw_local);
		bool on = rl > 0 || (rl < 0 && (st->mode & ISP_START_STILL));

		if (on || st->raw_local[st->ch]) {	/* (switched off again for the next preview) */
			isp_ch_simple(isp, "LOCAL_RAW_BUFFER_MODE", 0x125, st->ch, 0x10, on, 0, false);
			st->raw_local[st->ch] = on;
		}
	}
	isp_extra_cfg(isp, st->ch);
	isp_config_meta(isp, buf);
	ret = isp_alloc_bufs(isp, want);
	if (ret)
		goto out;
	st->seq = 0;
	st->nready = 0;
	st->mapped_cached = false;	/* (the previous START's mappings are gone: STOP = unmap) */
	st->stopping = false;
	st->on = true;
	if (READ_ONCE(isp_stream_nostart)) {
		dev_info(isp->dev, "stream_nostart=1: configured, no buffers posted, no CH_START\n");
		goto out;
	}
	isp_extra_prestart(isp, st->ch);
	isp_af_prestart(isp);
	if (!READ_ONCE(isp_bufs_after_start))
		isp_stream_kick(isp);	/* every buffer is FREE: one or two lists on BUF_H2T */
	if (READ_ONCE(isp_stream_nobufs))
		dev_info(isp->dev, "stream_nobufs=1: CH_START with no buffer posted (only fw-allocated DMA, e.g. 'RAW0')\n");
	isp_canary_arm(isp);		/* canary=1: before CH_START, so it is logged even if the SoC wedges */
	quiet = READ_ONCE(isp_quiet_ms);
	isp->skip_final_service = quiet != 0;
	ret = isp_ch_simple(isp, "CH_START", CISP_CMD_CH_START, st->ch, 0xc, 0, 0, true);
	isp->skip_final_service = false;
	if (!ret && READ_ONCE(isp_bufs_after_start))
		isp_stream_kick(isp);
	if (!ret && quiet)
		isp_quiet_window(isp, quiet);
	if (ret) {
		isp_stream_stop(isp);
		goto out;
	}
	isp_af_stream_on(isp);
	dev_info(isp->dev, "stream on: camera %u (%s 0x%04x), preset %u, %ux%u, %u buffer(s)%s, %u raw\n",
		 camera, st->sensor_name, st->sensor_id, st->preset, st->width, st->height, st->nbufs,
		 st->meta_size ? " + meta" : "", st->nraw);
out:
	kfree(buf);
	return ret;
}
