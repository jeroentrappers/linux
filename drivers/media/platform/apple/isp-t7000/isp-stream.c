// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: preview streaming (stage 10, stream=1). See re/CAMERA-STREAM.md.
 *
 * Buffers travel in u32 buffer lists (fw 0xa9590 / 0xa8ff0, kext 0x845bfb0):
 *   { u32 type, u32 count, count x { u32 addr[3], u32 flags[3], u32 nplanes, u32 pool, u64 tag } }
 * Host -> fw on BUF_H2T (type-0 ring: word0 = fw-rel list, word1 = length, word2 bit 28 =
 * list mode); fw -> host on BUF_T2H (type-1 ring: reply word0 = list | 1, word2 = 0x80000000).
 * Buffer states change under st.frame_lock; everything that talks to the fw holds isp->lock.
 */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "isp.h"

#define ISP_TAG_MAGIC		0x49535054ULL	/* 'ISPT' */
#define ISP_TAG(pool, i)	((ISP_TAG_MAGIC << 32) | ((u64)(pool) << 16) | (i))
#define ISP_H2T_LIST		0x10000000	/* word2 bit 28: word0 is a buffer list */
#define ISP_H2T_WORD2		0x30000000	/* Asahi's BUFEXC COMMAND value (bit 28 set) */
#define ISP_BUF_FILLED		0x40000000	/* per-plane flag (Asahi submits it; fw sets it on data) */
#define ISP_LIST_MIN		0x180		/* the fw's own T2H list size */

struct isp_bl_entry {
	u32 addr[3];
	u32 flags[3];
	u32 nplanes;
	u32 pool;
	u64 tag;
} __packed;
static_assert(sizeof(struct isp_bl_entry) == 0x28);

struct isp_bl {
	u32 type;
	u32 count;
	struct isp_bl_entry e[];
} __packed;

static_assert(sizeof(struct isp_bl) + ISP_BUFLIST_MAX * sizeof(struct isp_bl_entry) <=
	      ISP_BUFLIST_SLOT);

static void isp_fill_entry(struct t7000_isp *isp, struct isp_bl_entry *e, u32 pool, u32 i)
{
	struct isp_stream *st = &isp->st;

	memset(e, 0, sizeof(*e));
	e->pool = pool;
	e->tag = ISP_TAG(pool, i);
	if (pool == ISP_POOL_META) {
		e->addr[0] = isp_rel(st->meta[i].surf->iova);
		e->flags[0] = ISP_BUF_FILLED;
		e->nplanes = 1;
	} else {
		e->addr[0] = isp_rel(st->buf[i].surf->iova);
		e->addr[1] = isp_rel(st->buf[i].surf->iova + st->stride * st->height);
		e->flags[0] = e->flags[1] = ISP_BUF_FILLED;
		e->nplanes = 2;
	}
}

/* Post one buffer list on BUF_H2T without waiting. isp->lock held. */
static int isp_h2t_post(struct t7000_isp *isp, u32 n, const u32 *pool, const u32 *idx)
{
	struct isp_stream *st = &isp->st;
	struct isp_chan *c = isp->bh;
	u32 slot, *m, rel, len, i;
	struct isp_bl *bl;

	if (!c || !st->lists || !n)
		return -ENODEV;
	slot = c->idx;
	m = (u32 *)(c->ring + slot * ISP_MSG_SIZE);
	if (!(READ_ONCE(m[0]) & 1)) {
		st->h2t_busy++;
		return -EBUSY;		/* fw still owns the slot: retried on the next service pass */
	}
	bl = (struct isp_bl *)((u8 *)st->lists->virt + (slot % c->num) * ISP_BUFLIST_SLOT);
	bl->type = 1;
	bl->count = n;
	for (i = 0; i < n; i++)
		isp_fill_entry(isp, &bl->e[i], pool[i], idx[i]);
	len = sizeof(*bl) + n * sizeof(bl->e[0]);
	rel = isp_rel(st->lists->iova + (slot % c->num) * ISP_BUFLIST_SLOT);
	WRITE_ONCE(m[1], max_t(u32, len, ISP_LIST_MIN));
	WRITE_ONCE(m[2], ISP_H2T_WORD2);
	dma_wmb();
	WRITE_ONCE(m[0], rel);
	dma_wmb();
	isp_wr(isp, ISP_DOORBELL, BIT(c->src));
	c->idx = (slot + 1) % c->num;
	c->msgs++;
	st->h2t_posts++;
	if (READ_ONCE(isp_stream_dump))
		dev_info(isp->dev, "H2T slot %u: list fw-rel 0x%08x, %u buffer(s), first pool %u idx %u\n",
			 slot, rel, n, pool[0], idx[0]);
	return 0;
}

/* Hand every FREE frame/meta buffer back to the fw, one list per H2T slot. isp->lock held. */
void isp_stream_kick(struct t7000_isp *isp)
{
	struct isp_stream *st = &isp->st;
	u32 pool[ISP_BUFLIST_MAX], idx[ISP_BUFLIST_MAX], n, i, round;

	for (round = 0; round < 4 && st->on && !st->stopping; round++) {
		n = 0;
		spin_lock(&st->frame_lock);
		for (i = 0; i < st->nbufs && n < ISP_BUFLIST_MAX; i++) {
			if (st->meta[i].surf && st->meta[i].state == BUF_FREE) {
				pool[n] = ISP_POOL_META;
				idx[n++] = i;
			}
			if (n < ISP_BUFLIST_MAX && st->buf[i].state == BUF_FREE) {
				pool[n] = ISP_POOL_RENDERED;
				idx[n++] = i;
			}
		}
		/* claim them before posting: a T2H for them cannot come earlier than the post */
		for (i = 0; i < n; i++) {
			if (pool[i] == ISP_POOL_META)
				st->meta[idx[i]].state = BUF_FW;
			else
				st->buf[idx[i]].state = BUF_FW;
		}
		spin_unlock(&st->frame_lock);
		if (!n)
			return;
		if (isp_h2t_post(isp, n, pool, idx)) {
			spin_lock(&st->frame_lock);	/* slot busy: give them back, retry later */
			for (i = 0; i < n; i++) {
				if (pool[i] == ISP_POOL_META)
					st->meta[idx[i]].state = BUF_FREE;
				else
					st->buf[idx[i]].state = BUF_FREE;
			}
			spin_unlock(&st->frame_lock);
			return;
		}
	}
}

/*
 * Keep at least one buffer with the fw (READY + USER <= nbufs - 1): drop older ready
 * frames first (the newest stays), then recycle the oldest buffer userspace still holds.
 * frame_lock held.
 */
static void isp_frame_limit(struct isp_stream *st)
{
	u32 i, held = 0, oldest;

	for (i = 0; i < st->nbufs; i++)
		if (st->buf[i].state == BUF_USER)
			held++;
	while (st->nready > 1 && st->nready + held > st->nbufs - 1) {
		st->buf[st->ready[0]].state = BUF_FREE;
		memmove(st->ready, st->ready + 1, --st->nready * sizeof(st->ready[0]));
		st->dropped++;
	}
	while (held && st->nready + held > st->nbufs - 1) {
		oldest = U32_MAX;
		for (i = 0; i < st->nbufs; i++)
			if (st->buf[i].state == BUF_USER &&
			    (oldest == U32_MAX || st->buf[i].seq < st->buf[oldest].seq))
				oldest = i;
		st->buf[oldest].state = BUF_FREE;	/* userspace kept it too long */
		st->recycled++;
		held--;
	}
}

static void isp_t2h_entry(struct t7000_isp *isp, const struct isp_bl_entry *e)
{
	struct isp_stream *st = &isp->st;
	u32 pool = e->pool & 0x0fffffff, i = e->tag & 0xffff;
	struct isp_sbuf *b;

	if ((e->tag >> 32) != ISP_TAG_MAGIC || ((e->tag >> 16) & 0xffff) != pool ||
	    i >= st->nbufs || (pool != ISP_POOL_META && pool != ISP_POOL_RENDERED)) {
		st->errors++;
		dev_warn_ratelimited(isp->dev, "T2H: unknown buffer: pool 0x%x tag 0x%016llx addr 0x%08x\n",
				     e->pool, e->tag, e->addr[0]);
		return;
	}
	b = pool == ISP_POOL_META ? &st->meta[i] : &st->buf[i];
	if (READ_ONCE(isp_stream_dump))
		dev_info(isp->dev, "T2H: pool %u idx %u planes %u addr %08x %08x flags %08x %08x\n",
			 pool, i, e->nplanes, e->addr[0], e->addr[1], e->flags[0], e->flags[1]);
	spin_lock(&st->frame_lock);
	if (b->state != BUF_FW) {
		st->errors++;
		spin_unlock(&st->frame_lock);
		dev_warn_ratelimited(isp->dev, "T2H: pool %u buffer %u returned twice (state %d)\n",
				     pool, i, b->state);
		return;
	}
	if (pool == ISP_POOL_META || !st->on || st->stopping) {
		b->state = BUF_FREE;	/* meta, or a buffer handed back by CH_BUFFER_RETURN */
		spin_unlock(&st->frame_lock);
		return;
	}
	b->state = BUF_READY;
	b->seq = st->seq++;
	b->ts = ktime_get_ns();
	b->fwflags = e->flags[0];
	st->ready[st->nready++] = i;
	st->frames++;
	isp_frame_limit(st);
	spin_unlock(&st->frame_lock);
	if (READ_ONCE(isp_stream_dump))
		dev_info(isp->dev, "frame seq %u: buffer %u, %u bytes, fw flags 0x%08x\n",
			 b->seq, i, st->stride * st->height * 3 / 2, b->fwflags);
	wake_up_interruptible(&st->wait);
}

/*
 * BUF_T2H request (isp->lock held, from the ring service): walk the list, reply like
 * the kext (0x845c744): word0 = list | 1 in list mode, else 1.
 */
u32 isp_bt_handle(struct t7000_isp *isp, u32 w0, u32 w1, u32 w2)
{
	struct isp_stream *st = &isp->st;
	u32 rel = w0 & ~3U, n, i;
	const struct isp_bl *bl;

	st->t2h_msgs++;
	if (!(w2 & ISP_H2T_LIST)) {
		dev_warn_ratelimited(isp->dev, "T2H: single-buffer message %08x %08x %08x ignored\n",
				     w0, w1, w2);
		return 1;
	}
	bl = isp_translate(isp, rel, sizeof(*bl));
	if (!bl) {
		st->errors++;
		dev_warn_ratelimited(isp->dev, "T2H: list fw-rel 0x%08x outside every surface\n", rel);
		return rel | 1;
	}
	n = min_t(u32, bl->count, ISP_BUFLIST_MAX);	/* kext clamps to 8 */
	if (bl->type == 4 || bl->type == 0 || bl->type > 2 ||
	    !isp_translate(isp, rel, sizeof(*bl) + n * sizeof(bl->e[0]))) {
		dev_info_ratelimited(isp->dev, "T2H: list type %u count %u (not a buffer return)\n",
				     bl->type, bl->count);
		return rel | 1;
	}
	if (READ_ONCE(isp_stream_dump))
		dev_info(isp->dev, "T2H: list fw-rel 0x%08x type %u count %u w1 0x%x w2 0x%08x\n",
			 rel, bl->type, bl->count, w1, w2);
	for (i = 0; i < n; i++)
		isp_t2h_entry(isp, &bl->e[i]);
	return rel | 1;
}

/* ISP_IOC_RELEASE: userspace is done with buffer @index. isp->lock held. */
void isp_stream_release(struct t7000_isp *isp, u32 index)
{
	struct isp_stream *st = &isp->st;

	spin_lock(&st->frame_lock);
	if (index < st->nbufs && st->buf[index].state == BUF_USER)
		st->buf[index].state = BUF_FREE;
	spin_unlock(&st->frame_lock);
	isp_stream_kick(isp);
}

static bool isp_bufs_with_fw(struct isp_stream *st)
{
	bool any = false;
	u32 i;

	spin_lock(&st->frame_lock);
	for (i = 0; i < st->nbufs; i++)
		if (st->buf[i].state == BUF_FW || st->meta[i].state == BUF_FW)
			any = true;
	spin_unlock(&st->frame_lock);
	return any;
}

/* ISP_IOC_STOP / close / teardown. isp->lock held. */
void isp_stream_stop(struct t7000_isp *isp)
{
	struct isp_stream *st = &isp->st;
	struct isp_cmd_ch c = { .chan = st->ch };
	unsigned long end;
	u32 i;

	if (!st->on)
		return;
	st->stopping = true;
	if (isp->io && !isp->io_dead) {
		c.h.opcode = CISP_CMD_CH_STOP;
		isp_cmd_simple(isp, "CH_STOP", &c, 0xc, 0xc, NULL);
		c.h = (struct isp_cmd_hdr){ .opcode = CISP_CMD_CH_BUFFER_RETURN };
		isp_cmd_simple(isp, "CH_BUFFER_RETURN", &c, 0xc, 0xc, NULL);
		end = jiffies + msecs_to_jiffies(500);
		while (isp_bufs_with_fw(st) && !isp->io_dead && time_before(jiffies, end)) {
			isp_service(isp);
			isp_dart_fault(isp);
			usleep_range(1000, 2000);
		}
		if (isp_bufs_with_fw(st))
			dev_warn(isp->dev, "stream stop: fw kept some buffers after 500 ms (they stay mapped until power-off)\n");
	}
	spin_lock(&st->frame_lock);
	/* buffers the fw still holds stay BUF_FW: a late T2H frees them, START never reposts them */
	for (i = 0; i < st->nbufs; i++) {
		if (st->buf[i].state != BUF_FW)
			st->buf[i].state = BUF_FREE;
		if (st->meta[i].state != BUF_FW)
			st->meta[i].state = BUF_FREE;
	}
	st->nready = 0;
	st->on = false;
	st->stopping = false;
	spin_unlock(&st->frame_lock);
	wake_up_interruptible(&st->wait);
	dev_info(isp->dev, "stream stopped: %llu frame(s), %llu dropped, %llu recycled, %llu error(s)\n",
		 st->frames, st->dropped, st->recycled, st->errors);
}
