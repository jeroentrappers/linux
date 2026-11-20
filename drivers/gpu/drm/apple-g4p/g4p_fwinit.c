// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A8 GPU (G4P): firmware shared structures (milestone M4, "firmware alive").
 *
 * The layout and every value follow the iOS 12.5.8 host recipe in linux-on-iphone6 re/GPU-FW-INIT.md
 * (section numbers below refer to it): list A (10 objects, heap 7, cached firmware addresses), list B
 * (20 objects, heap 6, uncached), three register windows in heap 8, and the root pointer pair at
 * image + 0xff8. Deliberate differences from iOS: config +0x250 (GPC mode) = 0 for the direct boot; the
 * heap 4 buffers (config +0x30/+0x188/+0x4a8, list A #6) and the colour matrices (config +0x38) are 0.
 * The n61 perf/power values are the ADT-derived numbers of the recipe (TODO: read them from the DT).
 */

#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "g4p.h"

/* list A (§1.2) and list B (§1.3) object offsets */
#define LA_ROOT_A	0x0000
#define LA_1		0x1140
#define LA_2		0x1b00
#define LA_3		0x2400
#define LA_POWER	0x2cc0
#define LA_5		0x3d40
#define LA_6		0x3d80
#define LA_CONFIG	0x3dc0
#define LA_8		0x42c0
#define LA_9		0x4380
#define LA_SIZE		0x43a0

#define LB_ROOT_B	0x00000
#define LB_EVENT_CTL	0x01540
#define LB_LOG_CTL	0x01580
#define LB_TRACE_CTL	0x01640
#define LB_STAT_CTL	0x01680
#define LB_EVENT	0x016c0
#define LB_LOG_INDEX	0x042c0
#define LB_LOG_MSG	0x0f2c0
#define LB_TRACE	0x452c0
#define LB_STAT		0x47ec0
#define LB_RING_CTL(n)	(0x4a2c0 + 0x40 * (n))	/* host->FW rings 0..3 (TA, 3D, CL, device control) */
#define LB_RING(n)	(0x4a3c0 + 0x1400 * (n))
#define LB_CTX_U32	0x4f3c0
#define LB_CTX_PC	0x4f400
#define LB_SIZE		0x4f440

#define EVENT_ENTRY	0x2c
#define LOG_MSG		0xd8

/* PIO windows (§5): slot, phys, size */
static const struct {
	unsigned int slot;
	phys_addr_t phys;
	u32 size;
} g4p_pio[] = {
	{ 0, 0x201e00000ULL, 0x5000 },	/* FenderRegs (sgx reg[1]) */
	{ 1, 0x20e101000ULL, 0x1000 },	/* AICTimerRegs */
	{ 2, 0x20e104000ULL, 0x1000 },	/* AICSWIntRegs (ADT meta-sw-interrupt SET 0x20e104010) */
};

/* config perf tables (§6.2): ADT sgx perf-states on n61, MHz / mV */
static const u32 g4p_n61_mhz[] = { 0, 365, 450, 500, 182, 225, 250 };
static const u32 g4p_n61_mv[] = { 600, 871, 896, 912, 871, 896, 912 };

static inline void w32(void *base, u32 off, u32 v)
{
	put_unaligned_le32(v, base + off);
}

static inline void w64(void *base, u32 off, u64 v)
{
	put_unaligned_le64(v, base + off);
}

static u64 g4p_timebase(void)
{
#ifdef CONFIG_ARM64
	return read_sysreg(cntvct_el0);	/* the 24 MHz counter iOS reads as mach_absolute_time() */
#else
	return 0;
#endif
}

/* list A #1..#3 header (§2.2): {0, FW(self), 0, FW(self) + 8} */
static void g4p_list_head(void *a, u64 a_va, u32 off)
{
	u32 self = g4p_fw_addr(a_va + off, true);

	w32(a, off + 4, self);
	w32(a, off + 0xc, self + 8);
}

/* list A #4, power/perf state (§6.1); floats as their IEEE-754 bit patterns */
static void g4p_power_data(void *p)
{
	w32(p, 0x004, 0x10);
	w32(p, 0x010, 0x3f800000);
	w32(p, 0x028, 1);
	w32(p, 0x02c, 1);
	w32(p, 0x03c, 100);
	w32(p, 0x040, 0x4294fc36);	/* gpu-pwr-calc-coeff0..2 */
	w32(p, 0x044, 0x40041a37);
	w32(p, 0x048, 0x4219ba44);
	w32(p, 0x0e0, 0x42849d8b);	/* gpu-pwr-calc-static */
	w32(p, 0x164, 0x3f800000);	/* gpu-pwr-perf-scale1..4 (scale0 = 0) */
	w32(p, 0x168, 0x3faab021);
	w32(p, 0x16c, 0x3fc820c5);
	w32(p, 0x170, 0x3fe9d495);
	w32(p, 0x634, 625);
	w32(p, 0x640, 0x3f7f2e9f);	/* gpu-pwr-filter-time-constant 313: (n-1)/n, 1/n */
	w32(p, 0x648, 0x3b516154);
	w32(p, 0x650, 0x3ca59586);	/* gpu-pwr-integral-gain */
	w32(p, 0x65c, 0x452ba000);	/* 2746.0 = gpu-device-max-power */
	w32(p, 0x660, 0x40a90fdb);	/* gpu-pwr-proportional-gain */
	w32(p, 0x668, 0xbddfbe5e);
	w32(p, 0x66c, 0xfffffefc);
	w32(p, 0x674, 300);
	w32(p, 0x67c, 2746);
	w32(p, 0x694, 300);
	w32(p, 0x72c, 95);		/* gpu-perf-tgt-utilization */
	w32(p, 0x730, 6);
	w32(p, 0x734, 1);
	w32(p, 0x73c, 0x3f4ccccd);	/* gpu-perf-filter-time-constant 5 */
	w32(p, 0x744, 0x3e4ccccd);
	w32(p, 0x740, 0x3f7ae148);	/* gpu-perf-filter-time-constant2 50 */
	w32(p, 0x748, 0x3ca3d70a);
	w32(p, 0x74c, 0x40fca970);
	w32(p, 0x750, 0x3e4a2121);	/* gpu-perf-integral-gain2 */
	w32(p, 0x758, 0x42be0000);	/* 95.0 */
	w32(p, 0x75c, 0x416b53d1);
	w32(p, 0x760, 0x40db53d0);	/* gpu-perf-proportional-gain2 */
	w32(p, 0x764, 0x4006bca2);
	w32(p, 0x76c, 200);
	w32(p, 0x770, 100);
	w32(p, 0x778, 95);
	w32(p, 0x790, 100);
	w32(p, 0x7a8, 0x42c80000);	/* 100.0 */
	w32(p, 0x7ac, 300);
	w32(p, 0x7b4, 0x3f4ccccd);
	w32(p, 0x7b8, 0x3e4ccccd);
	w32(p, 0x904, 300);
	w64(p, 0x1008, g4p_timebase());
}

/* list A #7, firmware config (§4) */
static void g4p_config(struct g4p *g, void *c, const u64 *pio_va)
{
	const struct g4p_fw *fw = &g->fw;
	const struct g4p_fw_thread *t1 = NULL;

	for (unsigned int i = 0; i < 6; i++)		/* {5,6,1,1,3,3} << 32 */
		w64(c, 8 * i, (u64)((const u32[]){ 5, 6, 1, 1, 3, 3 })[i] << 32);
	for (unsigned int i = 0; i < ARRAY_SIZE(g4p_pio); i++) {
		u32 o = 0x190 + 16 * g4p_pio[i].slot;

		w64(c, o, g4p_pio[i].phys);
		w32(c, o + 8, g4p_fw_addr(pio_va[i], false));
		w32(c, o + 0xc, g4p_pio[i].size);
	}
	w64(c, 0x190 + 16 * 3, 0x201000000ULL);		/* slot 3 RGXRegs: listed, not mapped */
	w32(c, 0x190 + 16 * 3 + 0xc, 0x20000);
	w32(c, 0x250, 0);		/* GPC mode off: direct boot */
	w32(c, 0x254, 1);		/* lockup detection */
	w32(c, 0x258, 1);
	w32(c, 0x274, 1);		/* agf_timeslice */
	w32(c, 0x278, 1);
	w32(c, 0x27c, 1);		/* agf_timer */
	w32(c, 0x280, 24000);		/* agf_timer_period 1000 us in 24 MHz ticks */
	w32(c, 0x284, 0x10);
	w32(c, 0x290, 1);		/* agf_dwb */
	w32(c, 0x294, 1);		/* agf_itr_cache */
	w32(c, 0x29c, 1);		/* agf_deferred_opaque */
	w32(c, 0x2b4, 0x1f);		/* agf_max_tiles_per_pipe */
	w32(c, 0x2dc, 1);		/* agf_frg_vce_inval */
	w64(c, 0x300, 0x0000000400000002ULL);	/* family 2 (G4P), revision index 4 */
	w32(c, 0x308, 4);		/* cores */
	w32(c, 0x30c, 3);		/* gpu-num-perf-states */
	w32(c, 0x310, 16);		/* perf-state-count */
	for (unsigned int i = 0; i < ARRAY_SIZE(g4p_n61_mhz); i++) {
		w32(c, 0x314 + 4 * i, g4p_n61_mhz[i]);
		w32(c, 0x354 + 4 * i, g4p_n61_mv[i]);
	}
	w32(c, 0x494, 0x10);		/* AIC SW_SET word in window 2 */
	w32(c, 0x498, 0x400);		/* bit for IRQ 0x8a */
	w32(c, 0x49c, 0x20);		/* AIC timer register in window 1 */
	for (unsigned int i = 0; i < fw->nr_threads; i++)
		if (fw->thread[i].thread == 1)
			t1 = &fw->thread[i];
	if (t1) {
		w32(c, 0x4a0, t1->pc);
		w32(c, 0x4a4, t1->sp);
	}
	w64(c, 0x4b0, g4p_fw_heap_base());
	w32(c, 0x4bc, 7);		/* BIF-DM */
	w32(c, 0x4c8, 0xffff);		/* agf_gtp_ks */
	w32(c, 0x4cc, 7);		/* agf_frg_ks */
	w32(c, 0x4d0, 7);		/* agf_compute_ks */
	w64(c, 0x4d4, g4p_timebase());	/* wake timebase, last before boot */
}

int g4p_fwinit_build(struct g4p *g, struct g4p_mmu *mmu, struct g4p_fwinit *fi, const struct g4p_bo *image,
		     u32 *roots)
{
	struct device *dev = g->dev;
	void *a, *b;
	u64 av, bv;
	int ret;

	ret = g4p_bo_alloc(dev, mmu, G4P_HEAP_PRIVATE, LA_SIZE, &fi->list_a);
	if (!ret)
		ret = g4p_bo_alloc(dev, mmu, G4P_HEAP_SHARED, LB_SIZE, &fi->list_b);
	for (unsigned int i = 0; !ret && i < ARRAY_SIZE(g4p_pio); i++)
		ret = g4p_mmu_map_pio(dev, mmu, g4p_pio[i].phys, g4p_pio[i].size, &fi->pio_va[i]);
	if (ret)
		return ret;
	a = fi->list_a.cpu;
	b = fi->list_b.cpu;
	av = fi->list_a.gpuva;
	bv = fi->list_b.gpuva;
#define FA(off)	g4p_fw_addr(av + (off), true)
#define FB(off)	g4p_fw_addr(bv + (off), false)

	/* root A (§2.1) */
	for (unsigned int n = 0; n < 4; n++) {
		w32(a, LA_ROOT_A + 8 * n, FB(LB_RING_CTL(n)));
		w32(a, LA_ROOT_A + 8 * n + 4, FB(LB_RING(n)));
	}
	w32(a, LA_ROOT_A + 0x20, FB(LB_EVENT_CTL));
	w32(a, LA_ROOT_A + 0x24, FB(LB_EVENT));
	w32(a, LA_ROOT_A + 0x28, FB(LB_LOG_CTL));
	w32(a, LA_ROOT_A + 0x2c, FB(LB_LOG_INDEX));
	w32(a, LA_ROOT_A + 0x30, FB(LB_TRACE_CTL));
	w32(a, LA_ROOT_A + 0x34, FB(LB_TRACE));
	w32(a, LA_ROOT_A + 0x38, FB(LB_STAT_CTL));
	w32(a, LA_ROOT_A + 0x3c, FB(LB_STAT));
	w32(a, LA_ROOT_A + 0x88, FA(LA_1));
	w32(a, LA_ROOT_A + 0x8c, FA(LA_2));
	w32(a, LA_ROOT_A + 0x90, FA(LA_3));
	w32(a, LA_ROOT_A + 0x94, FA(LA_POWER));
	w32(a, LA_ROOT_A + 0x98, FA(LA_5));
	w32(a, LA_ROOT_A + 0x470, FA(LA_8));
	w32(a, LA_ROOT_A + 0x474, FA(LA_6));
	w32(a, LA_ROOT_A + 0x478, FA(LA_CONFIG));
	w32(a, LA_ROOT_A + 0x47c, FA(LA_9));
	w32(a, LA_ROOT_A + 0x480, FB(LB_LOG_MSG));
	/* coremem: image end, size 0 (§2.1) */
	w32(a, LA_ROOT_A + 0x4a4, g4p_fw_addr(image->gpuva + g->fw.image_size, true));
	w64(a, LA_ROOT_A + 0x4a8, image->gpuva + g->fw.image_size);

	g4p_list_head(a, av, LA_1);
	g4p_list_head(a, av, LA_2);
	g4p_list_head(a, av, LA_3);
	g4p_power_data(a + LA_POWER);

	/* root B (§3, §6.3) */
	/*
	 * iOS: 1 = idle power-off enabled. Kept 0 for bring-up: the firmware has its own Fender window, and if
	 * it powers the GPU down by itself, a later host RGX read hangs the SoC.
	 */
	w32(b, LB_ROOT_B + 0x028, 0);
	w32(b, LB_ROOT_B + 0x034, 0x78);	/* gpu_temperature_limit */
	w32(b, LB_ROOT_B + 0x04a, 1);
	w32(b, LB_ROOT_B + 0xc66, 2746);	/* PowerTarget */
	w32(b, LB_ROOT_B + 0xc6a, 300);		/* CLTMPerfMax */
	w32(b, LB_ROOT_B + 0xc6e, 300);		/* PPMPerfLimit */
	w32(b, LB_ROOT_B + 0xd0e, FB(LB_CTX_U32));
	w32(b, LB_ROOT_B + 0xd12, FB(LB_CTX_PC));
	w64(b, LB_CTX_PC, mmu->pc_dma);		/* per-context PC table [0] = CAT_BASE0 (§8.2) */

	g4p_config(g, a + LA_CONFIG, fi->pio_va);

	roots[0] = FA(LA_ROOT_A);
	roots[1] = FB(LB_ROOT_B);
#undef FA
#undef FB
	return 0;
}

void g4p_fwinit_free(struct device *dev, struct g4p_fwinit *fi)
{
	g4p_bo_free(dev, &fi->list_a);
	g4p_bo_free(dev, &fi->list_b);
}

/* event ring (§7): FW write index at ctl + 0x10, host read index at ctl + 0 */
bool g4p_fwinit_event(struct g4p_fwinit *fi, u32 *type, u32 *payload)
{
	void *b = fi->list_b.cpu;
	u32 rd = get_unaligned_le32(b + LB_EVENT_CTL) & 0xff;
	u32 wr = READ_ONCE(*(u32 *)(b + LB_EVENT_CTL + 0x10)) & 0xff;
	void *e;

	if (rd == wr)
		return false;
	rmb();	/* the entry after its write index */
	e = b + LB_EVENT + EVENT_ENTRY * rd;
	*type = get_unaligned_le32(e);
	*payload = get_unaligned_le32(e + 4);
	w32(b, LB_EVENT_CTL, (rd + 1) & 0xff);
	return true;
}

/*
 * Queue one 0x14-byte entry on host->firmware ring @ring (0 TA, 1 3D, 2 CL, 3 device control; list B
 * #10..#13 ctl, #14..#17 entries; re/GPU-FW-PROTO.md): ctl +0x00 = firmware read index, +0x10 = host write
 * index, 256 entries. Returns the new write index, or -EBUSY when the ring is full. The caller kicks.
 */
int g4p_fwinit_ring_push(struct g4p_fwinit *fi, unsigned int ring, const u32 *entry)
{
	void *b = fi->list_b.cpu;
	void *ctl = b + LB_RING_CTL(ring);
	u32 rd = READ_ONCE(*(u32 *)ctl) & 0xff;
	u32 wr = get_unaligned_le32(ctl + 0x10) & 0xff;

	if (((wr + 1) & 0xff) == rd)
		return -EBUSY;
	for (unsigned int i = 0; i < 5; i++)
		w32(b, LB_RING(ring) + 0x14 * wr + 4 * i, entry[i]);
	wmb();	/* the entry before the index that publishes it */
	wr = (wr + 1) & 0xff;
	w32(ctl, 0x10, wr);
	wmb();	/* the index before the kick */
	return wr;
}

u32 g4p_fwinit_ring_read_index(struct g4p_fwinit *fi, unsigned int ring)
{
	return READ_ONCE(*(u32 *)(fi->list_b.cpu + LB_RING_CTL(ring))) & 0xff;
}

/* first messages of log sub-ring 0, as text (§7: message +0 level, +4 seq, +8 time, +0x10 text) */
void g4p_fwinit_show_log(struct seq_file *s, struct g4p_fwinit *fi)
{
	void *b = fi->list_b.cpu;

	for (unsigned int k = 0; k < 4; k++) {
		u32 wr = READ_ONCE(*(u32 *)(b + LB_LOG_CTL + 0x30 * k + 0x10));

		seq_printf(s, "  log sub-ring %u: write index %u\n", k, wr);
	}
	for (unsigned int i = 0; i < 8; i++) {
		const char *m = b + LB_LOG_MSG + LOG_MSG * i;
		char text[LOG_MSG - 0x10 + 1];

		if (!get_unaligned_le32(m + 4) && !m[0x10])
			continue;
		memcpy(text, m + 0x10, LOG_MSG - 0x10);
		text[LOG_MSG - 0x10] = 0;
		seq_printf(s, "  log msg %u: level %u seq %u: %s\n", i, get_unaligned_le32(m),
			   get_unaligned_le32(m + 4), text);
	}
}
