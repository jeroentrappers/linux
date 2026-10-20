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

#include <linux/io.h>
#include <linux/list.h>
#include <linux/mutex.h>
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

enum { CHAN_CMD = 0, CHAN_REPLY = 1, CHAN_REPORT = 2 };

/* CISP opcodes used here (u16 at +4 of a command packet, status u16 at +6) */
#define CISP_CMD_START		0x0
#define CISP_CMD_STOP		0x1
#define CISP_CMD_CONFIG_GET	0x3
#define CISP_CMD_PRINT_ENABLE	0x4
#define CISP_CMD_BUILDINFO	0x6
#define CISP_CMD_POWER_DOWN	0x10

struct isp_surf {
	struct list_head list;
	u32 iova;		/* absolute DART IOVA */
	u32 size;		/* multiple of PAGE_SIZE */
	u32 tag;		/* 4cc, for logs */
	bool sm;		/* allocated by a SHAREDMALLOC request */
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
	bool io_dead;		/* an IO command timed out: fw not answering */
	u32 cmd_seq;
	int irq;
	atomic_t irq_count;
	unsigned long dart_err_logged;
};

/* module parameters (isp-main.c) */
extern int isp_dry_run, isp_irq_en, isp_sens, isp_smmu_en, isp_graceful;
extern uint isp_fw_iova, isp_dart_sids, isp_platform_id, isp_poll_ms;
extern uint isp_boot_timeout_ms, isp_cmd_timeout_ms, isp_max_shared_mb;

static inline u32 isp_rd(struct t7000_isp *isp, u32 off)
{
	return readl(isp->regs + off);
}

static inline void isp_wr(struct t7000_isp *isp, u32 off, u32 val)
{
	writel(val, isp->regs + off);
}

static inline u32 isp_rel(u32 iova)
{
	return iova - isp_fw_iova;
}

/* isp-pwr.c */
void isp_pmgr_log(struct t7000_isp *isp, const char *when);
int isp_power_on(struct t7000_isp *isp);
void isp_power_off(struct t7000_isp *isp);
void isp_log_window(struct t7000_isp *isp, const char *when);
int isp_stage_clocks(struct t7000_isp *isp);
void isp_dry_read(struct t7000_isp *isp);

/* isp-dart.c */
int isp_dart_init(struct t7000_isp *isp);
void isp_dart_fini(struct t7000_isp *isp);
void isp_dart_log(struct t7000_isp *isp, const char *when);
bool isp_dart_fault(struct t7000_isp *isp);
struct isp_surf *isp_surf_alloc(struct t7000_isp *isp, u32 size, u32 tag);
void isp_surf_free(struct t7000_isp *isp, struct isp_surf *s);
void isp_surf_free_all(struct t7000_isp *isp);
void *isp_translate(struct t7000_isp *isp, u32 rel, u32 len);
struct isp_surf *isp_surf_find(struct t7000_isp *isp, u32 iova);

/* isp-boot.c */
int isp_wait(struct t7000_isp *isp, u32 off, u32 mask, u32 want, uint ms,
	     const char *what);
void isp_fw_peek(struct t7000_isp *isp);
int isp_stage_fw(struct t7000_isp *isp);
int isp_stage_cpu(struct t7000_isp *isp);
int isp_stage_args(struct t7000_isp *isp);
int isp_stage_chans(struct t7000_isp *isp);
void isp_cpu_stop(struct t7000_isp *isp);

/* isp-ipc.c */
int isp_service(struct t7000_isp *isp);
int isp_cmd_send(struct t7000_isp *isp, void *pkt, u32 insize, u32 outsize,
		 bool wait, u16 *status);
int isp_stage_mbox(struct t7000_isp *isp);
int isp_stage_start(struct t7000_isp *isp);
int isp_stage_query(struct t7000_isp *isp);
void isp_ipc_stop(struct t7000_isp *isp);
void isp_fw_shutdown(struct t7000_isp *isp);

#endif
