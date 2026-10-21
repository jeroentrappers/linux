/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Apple A8 (T7000) ISP test driver: /dev/isp0 streaming interface (v1, no V4L2).
 *
 *   fd = open("/dev/isp0", O_RDWR);          boots the ISP if it is off
 *   ioctl(fd, ISP_IOC_START, &camera);       0 = rear, 1 = front; picks the preset
 *   ioctl(fd, ISP_IOC_INFO, &info);          width/height/stride/nbufs/bufsize (valid after START)
 *   p = mmap(NULL, nbufs * bufsize, PROT_READ, MAP_SHARED, fd, 0);
 *   read(fd, &frame, sizeof(frame));         blocks (poll POLLIN) until the next frame
 *   ... use p + frame.index * bufsize ...    NV12: Y at 0, UV at stride * height
 *   ioctl(fd, ISP_IOC_RELEASE, &frame.index);
 *   ioctl(fd, ISP_IOC_STOP);  close(fd);     last close powers the sensor/ISP down (keep_on=0)
 *
 * Userspace may hold at most nbufs - 1 buffers; beyond that the driver recycles the
 * oldest one it handed out. O_NONBLOCK: read() returns -EAGAIN when no frame is ready.
 * INFO reports the format of the camera started last (re-read + remap after each START).
 *
 * Upstreaming TODO: ioctl letter 'I' collides with ISDN in
 * Documentation/userspace-api/ioctl/ioctl-number.rst; pick a free range before posting.
 */
#ifndef _UAPI_LINUX_APPLE_ISP_T7000_H
#define _UAPI_LINUX_APPLE_ISP_T7000_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ISP_INFO_VERSION	1
#define ISP_UAPI_VERSION	ISP_INFO_VERSION	/* name used by Orchard's copy (ui/src/isp_uapi.h) */

struct isp_info {
	__u32 version;		/* ISP_INFO_VERSION */
	__u32 width;
	__u32 height;
	__u32 stride;		/* Y and UV stride, width rounded up to 64 */
	__u32 nbufs;
	__u32 bufsize;		/* page aligned; buffer i at mmap offset i * bufsize */
	__u32 sensor_id;	/* 0x237 rear IMX237, 0x190 front IMX190, 0 = not streaming */
	char sensor_name[32];
};

#define ISP_FRAME_FLAG_NOT_FILLED	0x1	/* fw did not mark plane 0 filled (data may be stale) */

struct isp_frame {
	__u32 index;		/* buffer index */
	__u32 seq;		/* frame counter since START */
	__u64 timestamp_ns;	/* CLOCK_MONOTONIC when the driver got it */
	__u32 bytesused;	/* stride * height * 3 / 2 */
	__u32 flags;		/* ISP_FRAME_FLAG_* */
};

#define ISP_IOC_INFO		_IOR('I', 1, struct isp_info)
#define ISP_IOC_START		_IOW('I', 2, __u32)
#define ISP_IOC_STOP		_IO('I', 3)
#define ISP_IOC_RELEASE		_IOW('I', 4, __u32)

#endif /* _UAPI_LINUX_APPLE_ISP_T7000_H */
