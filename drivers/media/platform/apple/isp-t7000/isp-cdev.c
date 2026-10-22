// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP: /dev/isp0 (stream=1). Contract: include/uapi/linux/apple-isp-t7000.h
 * and BRIEF-CAMERA-STREAM.md. First open boots the ISP (stages 1-9) if it is off; last
 * close stops the stream and, unless keep_on=1, powers sensor + ISP off again.
 * Frame buffers are DART surfaces (isp-dart.c), mapped read-only write-combine.
 */
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <uapi/linux/apple-isp-t7000.h>

#include "isp.h"

#define ISP_FULL_STAGE	9

static DEFINE_MUTEX(isp_open_lock);
static struct t7000_isp *cdev_isp;

static int isp_open(struct inode *inode, struct file *f)
{
	struct t7000_isp *isp = cdev_isp;
	int ret = 0;

	mutex_lock(&isp_open_lock);
	isp_trace_mark(isp, "open (count %d, stages done %d)", isp->open_count, isp->done);
	if (isp->open_count == 0)
		isp_af_forget(isp);	/* (a new client: no AF request of the last one is replayed) */
	if (isp->open_count == 0 && isp->done < ISP_FULL_STAGE) {
		dev_info(isp->dev, "/dev/isp0 opened: booting the ISP\n");
		ret = isp_boot(isp, ISP_FULL_STAGE);
		if (ret)
			isp_teardown(isp);
	}
	if (!ret) {
		isp->open_count++;
		f->private_data = isp;
	}
	mutex_unlock(&isp_open_lock);
	return ret;
}

static int isp_release(struct inode *inode, struct file *f)
{
	struct t7000_isp *isp = f->private_data;

	mutex_lock(&isp_open_lock);
	isp_trace_mark(isp, "release (count %d)", isp->open_count);
	if (--isp->open_count == 0) {
		mutex_lock(&isp->lock);
		isp_stream_stop(isp);
		isp_af_forget(isp);
		mutex_unlock(&isp->lock);
		if (!isp_keep_on) {
			dev_info(isp->dev, "/dev/isp0 closed: powering sensor + ISP off (keep_on=0)\n");
			isp_teardown(isp);
		}
	}
	mutex_unlock(&isp_open_lock);
	return 0;
}

static bool isp_mmap_cached = true;
module_param_named(mmap_cached, isp_mmap_cached, bool, 0644);
MODULE_PARM_DESC(mmap_cached, "frames mapped cacheable + invalidated on read() (default); 0 = write-combine (old). Read at mmap time (the next START)");

static long isp_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct t7000_isp *isp = f->private_data;
	struct isp_stream *st = &isp->st;
	void __user *up = (void __user *)arg;
	struct isp_info info;
	u32 v;
	long ret;

	isp_trace_mark(isp, "ioctl 0x%x", cmd);
	switch (cmd) {
	case ISP_IOC_INFO:
		memset(&info, 0, sizeof(info));
		info.version = ISP_INFO_VERSION;
		mutex_lock(&isp->lock);
		if (st->on) {
			info.width = st->width;
			info.height = st->height;
			info.stride = st->stride;
			info.nbufs = st->nbufs;
			info.bufsize = st->bufsize;
			info.sensor_id = st->sensor_id;
			strscpy(info.sensor_name, st->sensor_name, sizeof(info.sensor_name));
		}
		mutex_unlock(&isp->lock);
		return copy_to_user(up, &info, sizeof(info)) ? -EFAULT : 0;
	case ISP_IOC_START:
		if (get_user(v, (u32 __user *)up))
			return -EFAULT;
		mutex_lock(&isp->lock);
		ret = isp_stream_start(isp, v);
		mutex_unlock(&isp->lock);
		isp_term_flush(isp);
		return ret;
	case ISP_IOC_STOP:	/* (the AF request stays: the next START of this channel replays it) */
		mutex_lock(&isp->lock);
		isp_stream_stop(isp);
		mutex_unlock(&isp->lock);
		isp_term_flush(isp);
		return 0;
	case ISP_IOC_RELEASE:
		if (get_user(v, (u32 __user *)up))
			return -EFAULT;
		mutex_lock(&isp->lock);
		isp_stream_release(isp, v);
		mutex_unlock(&isp->lock);
		return 0;
	case ISP_IOC_FOCUS:
		return isp_af_ioctl(isp, up);
	}
	return -ENOTTY;
}

/*
 * mmap_cached=1: userspace maps the frames cacheable and the buffer's cache lines are invalidated
 * here, when it is handed out (BUF_USER: the fw no longer writes it). The page-by-page
 * dma_sync_single_for_cpu(FROM_DEVICE) is arm64 dcache_inval_poc over the frame: ~48 k lines for a
 * 1632x1224 NV12 frame. Lines the CPU pulled in earlier (speculation through the linear map or the
 * user mapping) are dropped, so the reader sees what the ISP wrote. The pages were cleaned at
 * dma_map_page time and the user mapping is read-only, so nothing dirty is ever written back over a frame.
 */
static void isp_frame_sync_for_cpu(struct t7000_isp *isp, struct isp_surf *s)
{
	unsigned int i;

	for (i = 0; i < s->npages; i++)
		dma_sync_single_for_cpu(isp->dev, s->dma[i], PAGE_SIZE, DMA_BIDIRECTIONAL);	/* (as mapped) */
}

static bool isp_frame_ready(struct isp_stream *st)
{
	return READ_ONCE(st->nready) || !READ_ONCE(st->on);
}

static ssize_t isp_read(struct file *f, char __user *ubuf, size_t len, loff_t *off)
{
	struct t7000_isp *isp = f->private_data;
	struct isp_stream *st = &isp->st;
	struct isp_frame fr = {};
	struct isp_sbuf *b;
	int ret;

	if (len < sizeof(fr))
		return -EINVAL;
	for (;;) {
		if (!READ_ONCE(st->on))
			return -EIO;
		spin_lock(&st->frame_lock);
		if (st->nready) {
			fr.index = st->ready[0];
			memmove(st->ready, st->ready + 1, --st->nready * sizeof(st->ready[0]));
			b = &st->buf[fr.index];
			b->state = BUF_USER;
			fr.seq = b->seq;
			fr.timestamp_ns = b->ts;
			fr.bytesused = st->stride * st->height * 3 / 2;
			fr.flags = b->filled ? 0 : ISP_FRAME_FLAG_NOT_FILLED;
			spin_unlock(&st->frame_lock);
			if (READ_ONCE(st->mapped_cached))	/* (latched at mmap: the knob may have changed since) */
				isp_frame_sync_for_cpu(isp, b->surf);
			break;
		}
		spin_unlock(&st->frame_lock);
		if (f->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(st->wait, isp_frame_ready(st));
		if (ret)
			return ret;
	}
	return copy_to_user(ubuf, &fr, sizeof(fr)) ? -EFAULT : sizeof(fr);
}

static __poll_t isp_poll(struct file *f, poll_table *wait)
{
	struct t7000_isp *isp = f->private_data;

	poll_wait(f, &isp->st.wait, wait);
	if (READ_ONCE(isp->st.nready))
		return EPOLLIN | EPOLLRDNORM;
	return READ_ONCE(isp->st.on) ? 0 : EPOLLERR;
}

/*
 * nbufs * bufsize, read-only. Write-combine (non-cacheable) by default before mmap_cached: every CPU load
 * from it is a bus read — Orchard's old converter read a 1632x1224 frame a byte at a time (3 M bus reads,
 * most of 12-15 ms). mmap_cached=1 (default): cacheable, invalidated on read() (isp_frame_sync_for_cpu).
 */
static int isp_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct t7000_isp *isp = f->private_data;
	struct isp_stream *st = &isp->st;
	unsigned long addr = vma->vm_start, len = vma->vm_end - vma->vm_start;
	unsigned long first = vma->vm_pgoff, pg, npg = len >> PAGE_SHIFT, per;
	int ret = 0;

	if (vma->vm_flags & VM_WRITE)
		return -EPERM;
	mutex_lock(&isp->lock);
	per = st->bufsize >> PAGE_SHIFT;
	if (!st->on || !per || first + npg > st->nbufs * per) {
		ret = -EINVAL;
		goto out;
	}
	vm_flags_mod(vma, VM_DONTEXPAND | VM_DONTDUMP, VM_MAYWRITE);
	if (!READ_ONCE(isp_mmap_cached))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	else
		WRITE_ONCE(st->mapped_cached, true);	/* sticky until the next START: any cached mapping is synced */
	for (pg = first; pg < first + npg && !ret; pg++, addr += PAGE_SIZE)
		ret = vm_insert_page(vma, addr, st->buf[pg / per].surf->pages[pg % per]);
out:
	mutex_unlock(&isp->lock);
	return ret;
}

static const struct file_operations isp_fops = {
	.owner = THIS_MODULE,
	.open = isp_open,
	.release = isp_release,
	.unlocked_ioctl = isp_ioctl,
	.read = isp_read,
	.poll = isp_poll,
	.mmap = isp_mmap,
	.llseek = noop_llseek,
};

static struct miscdevice isp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "isp0",
	.fops = &isp_fops,
	.mode = 0600,
};

int isp_cdev_init(struct t7000_isp *isp)
{
	struct isp_stream *st = &isp->st;
	int ret;

	cdev_isp = isp;
	ret = misc_register(&isp_misc);
	if (ret) {
		dev_err(isp->dev, "misc_register(isp0) = %d\n", ret);
		return ret;
	}
	isp->cdev_on = true;
	isp->dbg = debugfs_create_dir("apple-isp", NULL);
	debugfs_create_u64("frames", 0444, isp->dbg, &st->frames);
	debugfs_create_u64("dropped", 0444, isp->dbg, &st->dropped);
	debugfs_create_u64("recycled", 0444, isp->dbg, &st->recycled);
	debugfs_create_u64("errors", 0444, isp->dbg, &st->errors);
	debugfs_create_u64("h2t_posts", 0444, isp->dbg, &st->h2t_posts);
	debugfs_create_u64("h2t_busy", 0444, isp->dbg, &st->h2t_busy);
	debugfs_create_u32("empty", 0444, isp->dbg, &st->empty);
	debugfs_create_u64("t2h_msgs", 0444, isp->dbg, &st->t2h_msgs);
	dev_info(isp->dev, "stream: /dev/isp0 ready (debugfs apple-isp/)\n");
	return 0;
}

void isp_cdev_exit(struct t7000_isp *isp)
{
	if (!isp->cdev_on)
		return;
	debugfs_remove_recursive(isp->dbg);
	misc_deregister(&isp_misc);
	isp->cdev_on = false;
}
