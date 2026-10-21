/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Apple A8 (T7000) ISP bring-up test driver: shared definitions.
 *
 * Register meanings come from static RE of the iOS 12 AppleH6CamIn kext and the
 * ISP firmware (see re/CAMERA-PHASE0.md and re/CAMERA-PHASE2-STATUS.md in the
 * iphone6-linux repo). Everything here is unverified on hardware until the first
 * runbook pass; the dmesg of each stage confirms or refutes it.
 */
#ifndef __APPLE_ISP_T7000_H__
#define __APPLE_ISP_T7000_H__

#include <linux/build_bug.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/* --- physical addresses (standalone mode, no DT node) --- */
#define ISP_PHYS_BASE		0x205b00000ULL	/* ISP window, 5 MB */
#define ISP_PHYS_SIZE		0x500000
#define PMGR_PS_PHYS		0x20e020000ULL	/* PMGR power-state page */
#define PMGR_PS_SIZE		0x1000
#define PMGR_PS_ISPSENS0	0x210
#define PMGR_PS_ISPSENS1	0x218
#define PMGR_PS_ISP		0x2d0
#define PMGR_PS_MEDIA		0x2d8

/* PMGR power-state register (same layout as drivers/pmdomain/apple) */
#define PS_RESET		BIT(31)
#define PS_AUTO_ENABLE		BIT(28)
#define PS_DEV_DISABLE		BIT(10)
#define PS_FLAGS		(BIT(9) | BIT(8))
#define PS_ACTUAL		GENMASK(7, 4)
#define PS_TARGET		GENMASK(3, 0)
#define PS_ACTIVE		0xf
#define PS_PWRGATE		0x0

/* --- offsets inside the ISP window --- */
#define ISP_VERSION		0xa0000		/* >> 16 = 3/4/5 */
#define ISP_CLOCK_EN		0x20080		/* kext power_on_hardware writes 1 */
#define ISP_DART		0x22000		/* dart-isp (s5l8960x) */
#define ISP_SMMU		0x24000		/* dart-isp second block, read-only here */
#define ISP_MBOX_PENDING	0x40100
#define ISP_MBOX_ENABLE		0x40104
#define ISP_GPIO(n)		(0xc0100 + 4 * (n))
#define ISP_DOORBELL		0xc0500		/* write 1 << src */
#define ISP_MBOX_ACK		0xc050c		/* write back the pending value */
#define ISP_FW_BASE		0x100008
#define ISP_WIN_0C		0x10000c
#define ISP_WIN_10		0x100010
#define ISP_WIN_14		0x100014
#define ISP_WIN_18		0x100018	/* 0xfffff000 */
#define ISP_WIN_1C		0x10001c
#define ISP_CTRL_V34		0x100028	/* 0 = hold, 0x10 = run */
#define ISP_CTRL_RUN_V34	0x10
#define ISP_IRQMASK(n)		(0x100a00 + 4 * (n))
#define ISP_CPU_STAT		0x10081c	/* read 8x by ResetISPCPU */
#define ISP_TLIMIT0_OUT		0x44		/* outstanding transactions */
#define ISP_TLIMIT1_OUT		0x68

#define ISP_MAGIC_WAKE		0x8042006
#define ISP_MAGIC_ARGS		0xf7fbdff9
#define ISP_MAGIC_IDLE		0xfeedbabe

/* s5l8960x DART registers (drivers/iommu/apple-dart.c) */
#define DART_STREAM_CMD		0x0
#define DART_STREAM_CMD_INVAL	BIT(1)
#define DART_STREAM_CMD_BUSY	BIT(3)
#define DART_TCR		0xc		/* 8 bits per stream */
#define DART_TCR_ENABLE		0x80
#define DART_ERROR		0x10
#define DART_ERROR_FLAG		BIT(31)
#define DART_ERROR_ADDR		0x1c
#define DART_TTBR(sid, i)	(0x40 + 0x10 * (sid) + 4 * (i))
/*
 * DART kext restore 0x7fe6204: +0x20 = ADT diag-config (0x100), +0x30 = ADT fetch-config
 * (0xe0303); 0x7fe6448: +0x2c = ADT bypass-address (absent on dart-isp -> 0).
 */
#define DART_DIAG_CFG		0x20
#define DART_ERR_REFLECT	0x24		/* iOS: ADT error-reflector >> 12 */
#define DART_BYPASS		0x2c
#define DART_FETCH_CFG		0x30
#define ISP_DART_DIAG_IOS	0x100		/* ADT dart-isp diag-config */
#define ISP_DART_FETCH_IOS	0xe0303		/* ADT dart-isp fetch-config */
/*
 * SMMU block (+0x24000): names from the DART kext state dump 0x7fe5788. No TTBRs: iOS
 * never writes any, the block walks the DART's tables [A]. iOS setTranslationEnable
 * (0x7fe35a8 -> 0x7fe361c) sets CONFIG byte[sid] |= 0x80 right after the DART TCR.
 */
#define SMMU_STREAM_EN		0xc		/* SMMU_CONFIG: byte[sid] |= 0x80 */
#define SMMU_ERROR		0x10		/* SMMU_ERROR_STATUS, bit 31 = fault */
#define SMMU_ERR_AXI0		0x14
#define SMMU_ERR_AXI1		0x18
#define SMMU_ERR_ADDR		0x1c
#define SMMU_DIAG		0x20
#define SMMU_ERR_REFLECT	0x24		/* iOS: error-reflector[1] >> 12 */
#define SMMU_BYPASS		0x2c		/* reset 0x08080808; iOS: bypass-address[1] = 0 */
#define SMMU_ERR_ACK		0x0c00006c	/* iOS fault handler 0x7fe46c4: write back status & this */
#define ISP_ERR_REFLECTOR	0x20ffff000ULL	/* ADT dart-isp error-reflector: 2 x 0x2_0fff_f000 */
#define DART_TTBR_VALID		BIT(31)
#define DART_PTE_VALID		0x3		/* A8: tables + leaves need bits 1:0 = 11 */
#define DART_PTE_PA		GENMASK_ULL(35, 12)

#define ISP_FW_SIZE_V34		0x300000
#define ISP_IOVA_SPAN		0x10000000	/* fw sees fw_base .. fw_base + 256 MB */
#define ISP_IPC_POOL_MIN	0x18000
#define ISP_ARGS_SIZE		0x74
#define ISP_MSG_SIZE		0x40
#define ISP_CHAN_DESC_SIZE	0x100
#define ISP_MAX_CHANS		0x20
#define ISP_CMD_BUF_SIZE	0x400
#define ISP_TERM_MAX		0x80	/* kext caps TERMINAL strings at 0x80 */
#define ISP_TERM_LINES		64	/* fwlog=1: lines buffered until printed unlocked */
#define ISP_TERM_SHOW		200	/* fwlog=1: lines printed before rate-limiting */

enum { CHAN_CMD = 0, CHAN_REPLY = 1, CHAN_REPORT = 2 };

/* CISP opcodes used here (u16 at +4 of a command packet, status u16 at +6) */
#define CISP_CMD_START		0x0
#define CISP_CMD_STOP		0x1
#define CISP_CMD_CONFIG_GET	0x3
#define CISP_CMD_PRINT_ENABLE	0x4
#define CISP_CMD_BUILDINFO	0x6
#define CISP_CMD_POWER_DOWN	0x10
#define CISP_CMD_CH_START	0x100
#define CISP_CMD_CH_STOP	0x101
#define CISP_CMD_CH_BUFFER_RETURN	0x104
#define CISP_CMD_CH_CAMERA_CONFIG_GET	0x106
#define CISP_CMD_CH_CAMERA_CONFIG_SELECT 0x107
#define CISP_CMD_CH_INFO_GET	0x10d
#define CISP_CMD_CH_BUFFER_RECYCLE_MODE_SET 0x10e
#define CISP_CMD_CH_BUFFER_RECYCLE_START 0x10f
#define CISP_CMD_CH_SET_FILE_LOAD	0x111
#define CISP_CMD_CH_SIF_PIXEL_FORMAT_SET 0x115
#define CISP_CMD_CH_BUFFER_POOL_CONFIG_GET 0x116
#define CISP_CMD_CH_BUFFER_POOL_CONFIG_SET 0x117
#define CISP_CMD_CH_CROP_SET	0x801
#define CISP_CMD_CH_OUTPUT_CONFIG_SET	0xb01
#define CISP_CMD_CH_PREVIEW_STREAM_SET	0xb0d

#define ISP_SENS_GPIOS		3	/* avdd, rear shutdown, front shutdown */
#define ISP_MAX_BUFS		8
#define ISP_POOL_META		0
#define ISP_POOL_RENDERED	1
#define ISP_POOL_RAW		2	/* fw pool type 3: the RAW frame full-res presets need */
#define ISP_POOL_KINDS		3
#define ISP_FW_POOL_RAW		3
#define ISP_MAX_RAW		2
#define ISP_BUFLIST_SLOT	0x200	/* one H2T list per BUF_H2T ring slot */
#define ISP_BUFLIST_MAX		8	/* entries per list (kext clamps T2H lists to 8) */

enum isp_buf_state { BUF_FREE, BUF_FW, BUF_READY, BUF_USER };

struct isp_sbuf {
	struct isp_surf *surf;
	enum isp_buf_state state;
	u32 seq;
	u64 ts;
	u32 fwflags;
	bool filled;		/* came back in a RENDER list (T2H word2 0x1xxxxxxx) */
};

struct isp_stream {
	bool on, stopping;
	u32 ch, sensor_id, preset, npresets;
	u32 mode;		/* ISP_START_* flags of the running / next START */
	u32 full_w, full_h;	/* largest preset output of st->ch */
	bool raw_local[2];	/* LOCAL_RAW_BUFFER_MODE sent as on, per channel */
	u32 in_w, in_h, width, height, stride, bufsize, nbufs;
	u32 meta_size;
	u32 pool_id[ISP_POOL_KINDS];		/* fw pool id per ISP_POOL_* kind (BUF list +0x1c), from POOL_CONFIG_GET */
	u32 empty;		/* buffers returned without the 0x40000000 data flag */
	char sensor_name[32];
	struct isp_sbuf buf[ISP_MAX_BUFS];
	struct isp_sbuf meta[ISP_MAX_BUFS];
	struct isp_sbuf raw[ISP_MAX_RAW];
	u32 raw_size, nraw;	/* RAW pool: size from POOL_CONFIG_GET; buffers posted this START */
	struct isp_surf *lists;		/* ISP_BUFLIST_SLOT per BUF_H2T slot */
	struct isp_surf *setf;		/* setfile copy for CH_SET_FILE_LOAD */
	u32 seq;
	/* frame queue (frame_lock): indices in completion order */
	spinlock_t frame_lock;
	wait_queue_head_t wait;
	u32 ready[ISP_MAX_BUFS];
	u32 nready;
	/* debug counters (debugfs) */
	u64 frames, dropped, recycled, errors, h2t_posts, h2t_busy, t2h_msgs;
};

struct dentry;

/* CISP packet header: opcode u16 at +4, status u16 at +6 (A8 kext ISP_deInit) */
struct isp_cmd_hdr {
	u32 unk0;
	u16 opcode;
	u16 status;
} __packed;
static_assert(sizeof(struct isp_cmd_hdr) == 8);

/* most per-channel commands: header + channel (+ args) */
struct isp_cmd_ch {
	struct isp_cmd_hdr h;
	u32 chan;
	u32 arg[5];
} __packed;

struct isp_surf {
	struct list_head list;
	u32 iova;		/* absolute DART IOVA */
	u32 size;		/* multiple of PAGE_SIZE */
	u32 tag;		/* 4cc, for logs */
	bool sm;		/* allocated by a SHAREDMALLOC request */
	bool sm_freed;		/* fw freed it: kept mapped until teardown */
	unsigned int npages;
	struct page **pages;
	dma_addr_t *dma;
	void *virt;		/* write-combine (non-cacheable) vmap */
};

struct isp_chan {
	char name[0x41];
	u32 type, src, num, rel;
	u8 *ring;		/* CPU view of num * 0x40 bytes */
	u32 idx;		/* next slot to send (cmd) or to look at (reply/report) */
	unsigned long msgs;
};

struct t7000_isp {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *pmgr;
	void __iomem *regs;
	struct mutex lock;

	int done;		/* highest stage completed */
	u32 version;
	bool on_media, on_isp, on_sens0, on_sens1;	/* we powered these on */
	bool isp_was_on;

	/* mini DART */
	bool dart_on;
	u32 dart_old_reflect, smmu_old_reflect;
	bool smmu_set, dart_ios_set;	/* smmu_en / dart_ios wrote these: restore at fini */
	u32 smmu_old_cfg, dart_old_diag, dart_old_fetch, dart_old_bypass, smmu_old_bypass;
	unsigned long smmu_err_logged, smmu_faults;

	/* canary=1: pages at canary_base + IOVA, where an untranslated (bypassed) DMA lands [A] */
#define ISP_CANARIES		6
	u64 canary_pa[ISP_CANARIES];
	u64 canary_sum[ISP_CANARIES];
	const char *canary_what[ISP_CANARIES];
	unsigned int ncanary;
	unsigned long canary_hits;
	u64 *l1;
	dma_addr_t l1_dma;
	u64 *l2[512];
	dma_addr_t l2_dma[512];
	u32 iova_next;
	u64 shared_bytes;
	struct list_head surfs;
	struct isp_surf *fw, *ipc, *heap;

	/* boot */
	bool cpu_started, fw_up;
	u32 nchans, qsize, heapsize, pool_used;
	u32 args_off, cmd_off;
	struct isp_chan chans[ISP_MAX_CHANS];
	struct isp_chan *io, *sm, *tm, *bh, *bt, *it, *dg;

	/* runtime */
	struct delayed_work poll_work;
	bool poll_init, poll_on;
	bool started;		/* START acked */
	bool drain_failed;	/* TLIMIT drains did not reach 0 at the last CPU stop */
	bool io_dead;		/* an IO command timed out: fw not answering */
	bool skip_final_service;	/* quiet_ms: no service pass right after CH_START */
	u32 cmd_seq;
	int irq;
	atomic_t irq_count;
	unsigned long dart_err_logged;

	/* fw TERMINAL lines: dev_dbg, or (fwlog=1) buffered and printed without isp->lock */
	spinlock_t term_lock;
	char term_buf[ISP_TERM_LINES][ISP_TERM_MAX + 1];
	unsigned int term_head, term_count;
	unsigned long term_lines, term_shown, term_dropped;

	/* sensor power (isp-sens.c) */
	bool sens_on;
	int gpio_base;
	int gpio_on[ISP_SENS_GPIOS];	/* pin numbers we requested, 0 = none */
	void __iomem *pinregs, *refclk;
	u32 refclk_old[2], extclk_old[2];
	unsigned int refclk_saved, extclk_saved;

	/* streaming (isp-stream.c, isp-cdev.c) */
	struct isp_stream st;
	bool cdev_on;
	int open_count;
	struct dentry *dbg;
};

/* module parameters (isp-main.c) */
extern int isp_dry_run, isp_irq_en, isp_sens, isp_smmu_en, isp_graceful, isp_fwlog;
extern uint isp_fw_iova, isp_dart_sids, isp_platform_id, isp_poll_ms;
extern uint isp_boot_timeout_ms, isp_cmd_timeout_ms, isp_max_shared_mb;
extern int isp_sens_pwr, isp_refclk, isp_extclk_periph, isp_stream_en, isp_stream_dump;
extern int isp_keep_on, isp_preset, isp_stream_nostart, isp_dart_reflector, isp_dart_alias;
extern uint isp_refclk_cfg, isp_nbufs, isp_out_range, isp_setfile_rear, isp_setfile_front;
/* wedge hunt (re/CAMERA-WEDGE.md), all default off */
extern uint isp_quiet_ms;
extern int isp_stream_nobufs, isp_dart_ios, isp_canary, isp_pool_meta, isp_pool_frame, isp_keep_empty;
extern unsigned long long isp_canary_base;

/*
 * trace=1: every ISP-window write (and the PMGR / pin / ref clock writes) is printed
 * BEFORE it happens, followed by msleep(50) in task context, so with the console on
 * tty0 the last line on screen names the access that hung. trace=2 adds the reads.
 */
extern int isp_trace;
void isp_trace_io(struct t7000_isp *isp, char rw, const char *blk, u64 phys, u32 val);
void isp_trace_mark(struct t7000_isp *isp, const char *fmt, ...) __printf(2, 3);

static inline u32 isp_rd(struct t7000_isp *isp, u32 off)
{
	if (unlikely(READ_ONCE(isp_trace) >= 2))
		isp_trace_io(isp, 'R', "ISP", ISP_PHYS_BASE + off, 0);
	return readl(isp->regs + off);
}

static inline void isp_wr(struct t7000_isp *isp, u32 off, u32 val)
{
	if (unlikely(READ_ONCE(isp_trace)))
		isp_trace_io(isp, 'W', "ISP", ISP_PHYS_BASE + off, val);
	writel(val, isp->regs + off);
}

static inline u32 isp_rel(u32 iova)
{
	return iova - isp_fw_iova;
}

/* isp-pwr.c */
void isp_pmgr_log(struct t7000_isp *isp, const char *when);
int isp_power_on(struct t7000_isp *isp);
int isp_power_off(struct t7000_isp *isp);
void isp_log_window(struct t7000_isp *isp, const char *when);
int isp_stage_clocks(struct t7000_isp *isp);
void isp_dry_read(struct t7000_isp *isp);

/* isp-dart.c */
int isp_dart_init(struct t7000_isp *isp);
void isp_dart_fini(struct t7000_isp *isp);
void isp_dart_log(struct t7000_isp *isp, const char *when);
bool isp_dart_fault(struct t7000_isp *isp);
bool isp_smmu_fault(struct t7000_isp *isp);
struct isp_surf *isp_surf_alloc(struct t7000_isp *isp, u32 size, u32 tag);
void isp_surf_free(struct t7000_isp *isp, struct isp_surf *s);
void isp_surf_free_all(struct t7000_isp *isp);
void *isp_translate(struct t7000_isp *isp, u32 rel, u32 len);
struct isp_surf *isp_surf_find(struct t7000_isp *isp, u32 iova);

/* isp-boot.c */
int isp_wait(struct t7000_isp *isp, u32 off, u32 mask, u32 want, uint ms,
	     const char *what);
bool isp_cpu_drained(struct t7000_isp *isp);
void isp_fw_peek(struct t7000_isp *isp);
int isp_stage_fw(struct t7000_isp *isp);
int isp_stage_cpu(struct t7000_isp *isp);
int isp_stage_args(struct t7000_isp *isp);
int isp_stage_chans(struct t7000_isp *isp);
void isp_cpu_stop(struct t7000_isp *isp);

/* isp-ipc.c */
int isp_service(struct t7000_isp *isp);
void isp_term_flush(struct t7000_isp *isp);
int isp_cmd_send(struct t7000_isp *isp, void *pkt, u32 insize, u32 outsize,
		 bool wait, u16 *status);
int isp_stage_mbox(struct t7000_isp *isp);
int isp_stage_start(struct t7000_isp *isp);
int isp_stage_query(struct t7000_isp *isp);
void isp_ipc_stop(struct t7000_isp *isp);
void isp_fw_shutdown(struct t7000_isp *isp);
int isp_cmd_simple(struct t7000_isp *isp, const char *name, void *pkt,
		   u32 insize, u32 outsize, u16 *status);
void isp_canary_arm(struct t7000_isp *isp);
void isp_quiet_window(struct t7000_isp *isp, uint ms);

/* isp-sens.c */
int isp_sensor_power_on(struct t7000_isp *isp);
void isp_sensor_power_off(struct t7000_isp *isp);

/* isp-stream.c */
u32 isp_bt_handle(struct t7000_isp *isp, u32 w0, u32 w1, u32 w2);
int isp_stream_start(struct t7000_isp *isp, u32 arg);
void isp_stream_stop(struct t7000_isp *isp);
void isp_stream_release(struct t7000_isp *isp, u32 index);
void isp_stream_kick(struct t7000_isp *isp);

/* isp-af.c */
void isp_af_power(struct t7000_isp *isp, bool on);
void isp_af_prestart(struct t7000_isp *isp);
void isp_af_stream_on(struct t7000_isp *isp);
long isp_af_ioctl(struct t7000_isp *isp, void __user *up);

/* isp-cdev.c */
int isp_cdev_init(struct t7000_isp *isp);
void isp_cdev_exit(struct t7000_isp *isp);

/* isp-main.c */
int isp_boot(struct t7000_isp *isp, int upto);
void isp_teardown(struct t7000_isp *isp);

#endif
