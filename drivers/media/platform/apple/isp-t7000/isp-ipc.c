// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: IPC rings (stage 7), commands (8, 9), graceful shutdown.
 *
 * Slots are 0x40 bytes of u32 words, word0 written last (kext 0x846b060/b138/b248/b364):
 *  - type 0 (host -> fw command, IO): every slot primed with word0 = 1 in stage 6
 *    (= free). Send: word1 = insize, word2 = outsize, then word0 = fw-rel packet
 *    address (bit0 = 0), doorbell 1 << src. Done when the fw sets bit0 again.
 *  - type 1 (fw request needing a reply, SHAREDMALLOC / IO_T2H): present when
 *    bit0 == 0; the host replies in the same slot, word0 = value | 1 (words 1/2
 *    echoed), then the doorbell.
 *  - type 2 (fw report, TERMINAL): present when bit0 == 0; consume with
 *    word0 = 1, no doorbell.
 * All addresses are fw-relative (IOVA - fw IOVA). Nothing here is verified on hw.
 */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/string.h>

#include "isp.h"

#define ISP_AIC_IRQ		91	/* ADT isp interrupts[0]; the kext uses only index 0 */
#define ISP_TERM_MAX		0x80	/* kext caps TERMINAL strings at 0x80 */
#define ISP_SM_MAX		SZ_32M	/* largest single SHAREDMALLOC we honour */
#define ISP_IRQ_STORM		100000	/* give up on the IRQ after this many */
#define ISP_IRQ_SAFETY_MS	1000	/* poll interval while the IRQ is in use */

static inline u32 *isp_slot(struct isp_chan *c, u32 i)
{
	return (u32 *)(c->ring + i * ISP_MSG_SIZE);
}

static void isp_doorbell(struct t7000_isp *isp, struct isp_chan *c)
{
	dma_wmb();
	isp_wr(isp, ISP_DOORBELL, BIT(c->src));
}

/* TERMINAL: word0 & ~3 = fw-rel string, word1 = length */
static void isp_terminal(struct t7000_isp *isp, u32 rel, u32 len)
{
	char buf[ISP_TERM_MAX + 1];
	const char *p;
	u32 i;

	if (!len)
		return;
	len = min_t(u32, len, ISP_TERM_MAX);
	p = isp_translate(isp, rel, len);
	if (!p) {
		dev_warn_ratelimited(isp->dev, "TERMINAL: string at fw-rel 0x%08x len %u is outside every surface\n",
				     rel, len);
		return;
	}
	memcpy(buf, p, len);
	buf[len] = 0;
	len = strnlen(buf, len);
	while (len && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
		buf[--len] = 0;
	for (i = 0; i < len; i++)
		if (buf[i] < 0x20 || buf[i] > 0x7e)
			buf[i] = '.';
	dev_info(isp->dev, "ISPCPU: %s\n", buf);
}

/*
 * SHAREDMALLOC request: word0 == 0 -> alloc (word1 size, word2 4cc tag),
 * reply word0 = fw-rel address | 1. word0 != 0 -> free that address, reply 1.
 * A failed alloc replies 1 (= NULL) so the fw sees a failure, not a hang.
 */
static u32 isp_sm_request(struct t7000_isp *isp, u32 w0, u32 size, u32 tag)
{
	u32 rel = w0 & ~3U;
	struct isp_surf *s;

	if (!rel) {
		if (!size || size > ISP_SM_MAX) {
			dev_err(isp->dev, "SHAREDMALLOC: implausible size 0x%x '%4.4s', replying NULL\n",
				size, (char *)&tag);
			return 1;
		}
		s = isp_surf_alloc(isp, max_t(u32, size, 0x1000), tag);
		if (!s) {
			dev_err(isp->dev, "SHAREDMALLOC: alloc 0x%x '%4.4s' failed, replying NULL\n",
				size, (char *)&tag);
			return 1;
		}
		s->sm = true;
		dev_info(isp->dev, "SHAREDMALLOC: alloc 0x%x '%4.4s' -> fw-rel 0x%08x\n",
			 size, (char *)&tag, isp_rel(s->iova));
		return isp_rel(s->iova) | 1;
	}
	s = isp_surf_find(isp, isp_fw_iova + rel);
	if (!s || !s->sm || s->iova != isp_fw_iova + rel) {
		dev_warn(isp->dev, "SHAREDMALLOC: free of unknown fw-rel 0x%08x ignored\n", rel);
		return 1;
	}
	dev_info(isp->dev, "SHAREDMALLOC: free fw-rel 0x%08x (0x%x '%4.4s')\n", rel, s->size,
		 (char *)&s->tag);
	isp_surf_free(isp, s);
	return 1;
}

/* Handle every pending message of one type 1/2 ring; returns how many. */
static int isp_drain(struct t7000_isp *isp, struct isp_chan *c)
{
	u32 n, *m, w0, w1, w2, reply;
	int done = 0;

	for (n = 0; n < c->num; n++) {
		m = isp_slot(c, c->idx);
		w0 = READ_ONCE(m[0]);
		if (w0 & 1)
			break;		/* slot is host-owned: nothing pending */
		dma_rmb();
		w1 = READ_ONCE(m[1]);
		w2 = READ_ONCE(m[2]);
		/*
		 * The kext never initialises fw->host rings (only type 0 ones), so
		 * the fw must write word0 = 1 into them. An all-zero slot means it
		 * has not done that yet: leave the ring alone instead of treating
		 * zeroed memory as a NULL alloc / NULL string.
		 */
		if (!w0 && !w1 && !w2)
			break;
		c->msgs++;
		done++;
		if (c->type == CHAN_REPORT) {
			if (c == isp->tm)
				isp_terminal(isp, w0 & ~3U, w1);
			else
				dev_info(isp->dev, "report '%s'[%u]: %08x %08x %08x\n",
					 c->name, c->idx, w0, w1, w2);
			WRITE_ONCE(m[0], 1);	/* consume, no doorbell */
		} else {
			if (c == isp->sm) {
				reply = isp_sm_request(isp, w0, w1, w2);
			} else {
				if (c == isp->tm)
					isp_terminal(isp, w0 & ~3U, w1);
				else
					dev_info(isp->dev, "request '%s'[%u]: %08x %08x %08x (acked)\n",
						 c->name, c->idx, w0, w1, w2);
				reply = w0 | 1;
			}
			dma_wmb();
			WRITE_ONCE(m[0], reply);
			isp_doorbell(isp, c);
		}
		c->idx = (c->idx + 1) % c->num;
	}
	return done;
}

/* One service pass: ack the mailbox, drain all fw->host rings, check the DART. */
int isp_service(struct t7000_isp *isp)
{
	u32 pending, i;
	int n = 0;

	pending = isp_rd(isp, ISP_MBOX_PENDING);
	if (pending)
		isp_wr(isp, ISP_MBOX_ACK, pending);
	for (i = 0; i < isp->nchans; i++) {
		struct isp_chan *c = &isp->chans[i];

		if (!c->ring || c->type == CHAN_CMD)
			continue;
		n += isp_drain(isp, c);
	}
	isp_dart_fault(isp);
	return n;
}

/*
 * Send one command on the IO ring. @pkt must hold max(insize, outsize) bytes
 * (opcode u16 at +4); with @wait the fw's output is copied back into it and the
 * u16 status at +6 is returned in @status. While waiting (isp_cmd_timeout_ms,
 * iOS: 2 s) the other rings keep being serviced, so SHAREDMALLOC requests made
 * while handling the command are answered. Called with isp->lock held.
 */
int isp_cmd_send(struct t7000_isp *isp, void *pkt, u32 insize, u32 outsize,
		 bool wait, u16 *status)
{
	struct isp_chan *c = isp->io;
	unsigned long start, end;
	u32 *m, rel, w0, slot, seq;
	u8 *buf;
	u16 op;

	if (!c || !isp->ipc)
		return -ENODEV;
	if (isp->io_dead) {
		dev_err(isp->dev, "IO: the fw stopped answering earlier, not sending\n");
		return -EIO;
	}
	if (insize < 8 || insize > ISP_CMD_BUF_SIZE || outsize > ISP_CMD_BUF_SIZE)
		return -EINVAL;
	op = ((u16 *)pkt)[2];
	seq = ++isp->cmd_seq;
	slot = c->idx;
	m = isp_slot(c, slot);
	w0 = READ_ONCE(m[0]);
	if (!(w0 & 1)) {
		dev_err(isp->dev, "cmd #%u opcode 0x%04x: IO slot %u still owned by the fw (word0 0x%08x)\n",
			seq, op, slot, w0);
		return -EBUSY;
	}
	buf = (u8 *)isp->ipc->virt + isp->cmd_off;
	rel = isp_rel(isp->ipc->iova + isp->cmd_off);	/* 64-byte aligned: bit0 = 0 */
	memset(buf, 0, max(insize, outsize));
	memcpy(buf, pkt, insize);
	WRITE_ONCE(m[1], insize);
	WRITE_ONCE(m[2], outsize);
	dma_wmb();
	WRITE_ONCE(m[0], rel);		/* word0 last: hands the slot to the fw */
	isp_doorbell(isp, c);
	c->idx = (slot + 1) % c->num;
	c->msgs++;
	dev_info(isp->dev, "cmd #%u opcode 0x%04x: IO slot %u, packet fw-rel 0x%08x, in 0x%x out 0x%x%s\n",
		 seq, op, slot, rel, insize, outsize, wait ? "" : " (not waiting)");
	if (!wait)
		return 0;

	start = jiffies;
	end = start + msecs_to_jiffies(isp_cmd_timeout_ms);
	for (;;) {
		w0 = READ_ONCE(m[0]);
		if (w0 & 1)
			break;
		isp_service(isp);
		if (isp_dart_fault(isp)) {
			dev_err(isp->dev, "cmd #%u opcode 0x%04x: DART fault while waiting\n", seq, op);
			isp->io_dead = true;
			isp_log_window(isp, "on cmd fault");
			return -EIO;
		}
		if (time_after(jiffies, end)) {
			dev_err(isp->dev, "cmd #%u opcode 0x%04x: TIMEOUT after %u ms, IO slot %u word0 0x%08x mbox pending 0x%08x\n",
				seq, op, isp_cmd_timeout_ms, slot, w0,
				isp_rd(isp, ISP_MBOX_PENDING));
			isp->io_dead = true;
			isp_log_window(isp, "on cmd timeout");
			isp_dart_log(isp, "on cmd timeout");
			return -ETIMEDOUT;
		}
		usleep_range(1000, 2000);
	}
	dma_rmb();
	if (outsize)
		memcpy(pkt, buf, outsize);
	op = ((u16 *)buf)[2];
	if (status)
		*status = ((u16 *)buf)[3];
	dev_info(isp->dev, "cmd #%u opcode 0x%04x: done after %u ms, word0 0x%08x w1 0x%x w2 0x%x, status 0x%04x\n",
		 seq, op, jiffies_to_msecs(jiffies - start), w0, READ_ONCE(m[1]),
		 READ_ONCE(m[2]), ((u16 *)buf)[3]);
	isp_service(isp);
	return 0;
}

/* Hard half: ack the mailbox (as the kext ISR does) and hand off to the thread. */
static irqreturn_t isp_irq_hard(int irq, void *data)
{
	struct t7000_isp *isp = data;
	u32 pending = isp_rd(isp, ISP_MBOX_PENDING);

	if (!pending)
		return IRQ_NONE;
	isp_wr(isp, ISP_MBOX_ACK, pending);
	if (atomic_inc_return(&isp->irq_count) == ISP_IRQ_STORM) {
		disable_irq_nosync(irq);
		dev_err(isp->dev, "IRQ %d: %d interrupts, disabling it (the poll worker keeps going)\n",
			irq, ISP_IRQ_STORM);
	}
	return IRQ_WAKE_THREAD;
}

static irqreturn_t isp_irq_thread(int irq, void *data)
{
	struct t7000_isp *isp = data;

	mutex_lock(&isp->lock);
	isp_service(isp);
	mutex_unlock(&isp->lock);
	return IRQ_HANDLED;
}

/* AIC IRQ 91 by hand: there is no DT node in standalone mode. */
static int isp_irq_setup(struct t7000_isp *isp)
{
	struct of_phandle_args args = {
		.args_count = 3,
		.args = { 0 /* AIC_IRQ */, ISP_AIC_IRQ, IRQ_TYPE_LEVEL_HIGH },
	};
	unsigned int virq;
	int ret;

	args.np = of_find_compatible_node(NULL, NULL, "apple,aic");
	if (!args.np) {
		dev_err(isp->dev, "irq=1: no apple,aic node\n");
		return -ENODEV;
	}
	virq = irq_create_of_mapping(&args);
	of_node_put(args.np);
	if (!virq) {
		dev_err(isp->dev, "irq=1: mapping AIC IRQ %d failed\n", ISP_AIC_IRQ);
		return -ENXIO;
	}
	ret = request_threaded_irq(virq, isp_irq_hard, isp_irq_thread, IRQF_ONESHOT,
				   "apple-isp-t7000", isp);
	if (ret) {
		dev_err(isp->dev, "irq=1: request_threaded_irq(%u) = %d\n", virq, ret);
		irq_dispose_mapping(virq);
		return ret;
	}
	isp->irq = virq;
	dev_info(isp->dev, "AIC IRQ %d = virq %u requested\n", ISP_AIC_IRQ, virq);
	return 0;
}

static uint isp_poll_interval(struct t7000_isp *isp)
{
	return isp->irq >= 0 ? ISP_IRQ_SAFETY_MS : max_t(uint, isp_poll_ms, 5);
}

static void isp_poll_fn(struct work_struct *work)
{
	struct t7000_isp *isp = container_of(to_delayed_work(work), struct t7000_isp,
					     poll_work);
	int n = 0;

	mutex_lock(&isp->lock);
	if (READ_ONCE(isp->poll_on))
		n = isp_service(isp);
	mutex_unlock(&isp->lock);
	if (n && isp->irq >= 0)
		dev_info_ratelimited(isp->dev, "safety poll handled %d message(s) the IRQ did not (irqs so far: %d)\n",
				     n, atomic_read(&isp->irq_count));
	if (READ_ONCE(isp->poll_on))
		schedule_delayed_work(&isp->poll_work,
				      msecs_to_jiffies(isp_poll_interval(isp)));
}

static struct isp_chan *isp_chan_find(struct t7000_isp *isp, const char *name)
{
	u32 i;

	for (i = 0; i < isp->nchans; i++)
		if (!strcmp(isp->chans[i].name, name))
			return &isp->chans[i];
	return NULL;
}

static void isp_ring_log(struct t7000_isp *isp, struct isp_chan *c)
{
	u32 i, pend = 0, zero = 0, *m;

	for (i = 0; i < c->num; i++) {
		m = isp_slot(c, i);
		if (!(READ_ONCE(m[0]) & 1))
			pend++;
		if (!READ_ONCE(m[0]) && !READ_ONCE(m[1]) && !READ_ONCE(m[2]))
			zero++;
	}
	m = isp_slot(c, 0);
	dev_info(isp->dev, "ring '%s' type %u src %u: slot0 %08x %08x %08x, %u/%u slots bit0=0, %u all-zero\n",
		 c->name, c->type, c->src, READ_ONCE(m[0]), READ_ONCE(m[1]),
		 READ_ONCE(m[2]), pend, c->num, zero);
}

/* Stage 7: name the channels, enable the mailbox, start IRQ and/or poller. */
int isp_stage_mbox(struct t7000_isp *isp)
{
	static const char *const names[] = {
		"SHAREDMALLOC", "TERMINAL", "BUF_H2T", "BUF_T2H", "IO", "IO_T2H", "DEBUG",
	};
	struct isp_chan **ptr[] = {
		&isp->sm, &isp->tm, &isp->bh, &isp->bt, &isp->io, &isp->it, &isp->dg,
	};
	u32 i;
	int ret, n;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		*ptr[i] = isp_chan_find(isp, names[i]);
		if (*ptr[i])
			dev_info(isp->dev, "channel %-12s = #%td (type %u src %u num %u)\n",
				 names[i], *ptr[i] - isp->chans, (*ptr[i])->type,
				 (*ptr[i])->src, (*ptr[i])->num);
		else
			dev_warn(isp->dev, "channel %-12s missing from the table\n", names[i]);
	}
	if (!isp->io || !isp->sm || !isp->tm) {
		dev_err(isp->dev, "IO, SHAREDMALLOC and TERMINAL are required\n");
		return -ENODEV;
	}
	if (isp->io->type != CHAN_CMD || isp->sm->type != CHAN_REPLY ||
	    isp->tm->type == CHAN_CMD) {
		dev_err(isp->dev, "unexpected types: IO %u (want 0) SHAREDMALLOC %u (want 1) TERMINAL %u (want 2)\n",
			isp->io->type, isp->sm->type, isp->tm->type);
		return -EIO;
	}
	if (isp->tm->type != CHAN_REPORT)
		dev_warn(isp->dev, "TERMINAL is type %u, not 2: replying to it like a request ring\n",
			 isp->tm->type);
	for (i = 0; i < isp->nchans; i++)
		isp_ring_log(isp, &isp->chans[i]);
	if (isp->cmd_off + ISP_CMD_BUF_SIZE > isp->ipc->size) {
		dev_err(isp->dev, "command buffer outside the IPC pool\n");
		return -EIO;
	}

	dev_info(isp->dev, "mbox before enable: pending 0x%08x enable 0x%08x\n",
		 isp_rd(isp, ISP_MBOX_PENDING), isp_rd(isp, ISP_MBOX_ENABLE));
	isp_wr(isp, ISP_MBOX_ENABLE, 0xf);
	dev_info(isp->dev, "mbox enable +0x40104 = 0xf (readback 0x%08x), pending 0x%08x\n",
		 isp_rd(isp, ISP_MBOX_ENABLE), isp_rd(isp, ISP_MBOX_PENDING));
	isp->fw_up = true;	/* from here on, unwind tries STOP + POWER_DOWN */

	if (isp_irq_en) {
		ret = isp_irq_setup(isp);
		if (ret)
			return ret;
	}
	INIT_DELAYED_WORK(&isp->poll_work, isp_poll_fn);
	isp->poll_init = true;
	WRITE_ONCE(isp->poll_on, true);
	n = isp_service(isp);
	dev_info(isp->dev, "initial service pass: %d message(s); %s, poll every %u ms\n", n,
		 isp->irq >= 0 ? "IRQ mode" : "poll mode", isp_poll_interval(isp));
	schedule_delayed_work(&isp->poll_work, msecs_to_jiffies(isp_poll_interval(isp)));
	return 0;
}

/* Called without isp->lock: the IRQ thread and the worker take it. IRQ first. */
void isp_ipc_stop(struct t7000_isp *isp)
{
	u32 i;

	if (isp->irq >= 0) {
		free_irq(isp->irq, isp);
		irq_dispose_mapping(isp->irq);
		dev_info(isp->dev, "IRQ %d freed after %d interrupt(s)\n", isp->irq,
			 atomic_read(&isp->irq_count));
		isp->irq = -1;
	}
	if (isp->poll_init) {
		WRITE_ONCE(isp->poll_on, false);
		cancel_delayed_work_sync(&isp->poll_work);
		isp->poll_init = false;
		dev_info(isp->dev, "poll worker stopped\n");
	}
	for (i = 0; i < isp->nchans; i++)
		if (isp->chans[i].msgs)
			dev_info(isp->dev, "ring '%s': %lu message(s)\n", isp->chans[i].name,
				 isp->chans[i].msgs);
}
