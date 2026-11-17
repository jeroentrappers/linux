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
/*
 * ISP_IOC_START argument: the camera in bits 7:0 (0 = rear, 1 = front) plus at most one
 * preset-choice flag. 0 / 1 alone keep the old choice (smallest preset >= 640x480; the
 * "preset" module parameter overrides it). Drivers before these flags reject them with
 * EINVAL, so a caller can retry with the bare camera number.
 */
#define ISP_START_CAMERA_MASK	0xff
#define ISP_START_PHOTO_VIEW	0x100	/* viewfinder for stills: smallest preset >= 640x480 with
					 * the aspect ratio of the full-resolution one (4:3) */
#define ISP_START_STILL		0x200	/* the sensor's full-resolution preset (largest output) */
#define ISP_START_PRESET	0x400	/* exactly the preset in bits 23:16 */
#define ISP_START_PRESET_SHIFT	16

#define ISP_IOC_START		_IOW('I', 2, __u32)	/* *arg: camera | ISP_START_* */
#define ISP_IOC_STOP		_IO('I', 3)
#define ISP_IOC_RELEASE		_IOW('I', 4, __u32)

/*
 * Minor 1: ISP_IOC_FOCUS (older drivers answer -ENOTTY). INFO's version stays 1.
 * Autofocus of the rear camera (IMX237 + VCM); the front camera is fixed-focus, there
 * only ISP_FOCUS_F_AE does something and position comes back -1.
 * Window: per-mille (0..1000) of the frame as delivered (sensor orientation, not
 * mirrored); w = 0 or h = 0 keeps the fw's current window; clamped into the frame.
 * -EAGAIN when no camera is streaming. Blocks for the fw round trips (a few ms).
 */
#define ISP_UAPI_MINOR		2

struct isp_focus {
	__u32 mode;		/* ISP_FOCUS_* */
	__u32 flags;		/* ISP_FOCUS_F_* */
	__u16 x, y, w, h;	/* window, per-mille */
	__s32 position;		/* in: lens position for ISP_FOCUS_MANUAL (0..255);
				 * out: fw lens position after the call, -1 = unknown */
	__u32 fw_status;	/* out: first non-zero fw status of the commands sent, 0 = all ok */
	__u32 reserved[2];
};

#define ISP_FOCUS_CONTINUOUS	0	/* window + continuous AF (the default at stream start) */
#define ISP_FOCUS_ONCE		1	/* window + one AF scan, then hold (tap to focus) */
#define ISP_FOCUS_LOCK		2	/* continuous AF off: the lens stays where it is (AF lock) */
#define ISP_FOCUS_MANUAL	3	/* continuous AF off, lens to .position */
#define ISP_FOCUS_QUERY		4	/* send nothing (except F_AE), read the lens position */

#define ISP_FOCUS_F_AE		0x1	/* also meter exposure on the window */

#define ISP_IOC_FOCUS		_IOWR('I', 0x20, struct isp_focus)

/*
 * ISP_IOC_TORCH (minor 2): the rear flash LEDs (white + amber) in torch mode, through the ISP
 * firmware, which owns the flash chip on the rear sensor's I2C bus. Works with or without a stream
 * (-EAGAIN only while the ISP is not booted); the last close of /dev/isp0 powers the ISP off, and
 * the light with it, so a torch-only client keeps the device open. level 0 = off, 1..5 =
 * brightness steps (fw: 6 and up = full);
 * split: -1 = leave the colour mix alone, 0..100 = % of the current on LED2 (fw manual mode;
 * measured on n61: 0 = LED1 only, 100 = LED2 only).
 * The currents the fw reports (mA) come back after the call; both 0 also when it found no chip.
 */
struct isp_torch {
	__u32 level;
	__s32 split;
	__u16 led1_ma, led2_ma;	/* out */
	__u32 fw_status;	/* out: first non-zero fw status, 0 = all ok */
	__u32 reserved[2];
};

#define ISP_IOC_TORCH		_IOWR('I', 0x21, struct isp_torch)

#endif /* _UAPI_LINUX_APPLE_ISP_T7000_H */
