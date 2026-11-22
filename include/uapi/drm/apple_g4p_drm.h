/* SPDX-License-Identifier: (GPL-2.0-only WITH Linux-syscall-note) OR MIT */
/*
 * Apple A8 GPU (G4P) DRM interface, bring-up version (linux-on-iphone6 re/GPU-BRINGUP.md 8.64).
 *
 * Experimental and not stable: compute only, one GPU-visible pool, synchronous submission. Buffers are carved
 * from a pool that is mapped at GPU VA APPLE_G4P_POOL_VA before the firmware boots; a buffer's GPU VA minus
 * that base is also its PDS/USC code offset. A submission is a job image in the record format of the C8 debugfs
 * harness (tools/gpulinux/g4pjob.py): a header and records that write the pool, the CDM control stream and
 * command fields. The call returns when the job's completion stamp is seen or the timeout expires.
 */
#ifndef _UAPI_APPLE_G4P_DRM_H_
#define _UAPI_APPLE_G4P_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define APPLE_G4P_POOL_VA		0x300000000ULL

#define DRM_APPLE_G4P_GET_PARAM		0x00
#define DRM_APPLE_G4P_BO_CREATE		0x01
#define DRM_APPLE_G4P_SUBMIT		0x02

enum drm_apple_g4p_param {
	DRM_APPLE_G4P_PARAM_POOL_VA = 1,	/* GPU VA of the pool (APPLE_G4P_POOL_VA) */
	DRM_APPLE_G4P_PARAM_POOL_SIZE = 2,	/* bytes */
	DRM_APPLE_G4P_PARAM_CORE_ID = 3,	/* RGX CORE_ID, valid once the GPU has been powered */
	DRM_APPLE_G4P_PARAM_CORE_REVISION = 4,
};

struct drm_apple_g4p_get_param {
	__u32 param;		/* in: enum drm_apple_g4p_param */
	__u32 pad;
	__u64 value;		/* out */
};

struct drm_apple_g4p_bo_create {
	__u64 size;		/* in: bytes, rounded up to 4 KiB */
	__u32 flags;		/* in: 0 */
	__u32 handle;		/* out: GEM handle */
	__u64 gpu_va;		/* out: GPU VA (pool offset = gpu_va - APPLE_G4P_POOL_VA) */
	__u64 mmap_offset;	/* out: offset for mmap() on the DRM fd */
};

struct drm_apple_g4p_submit {
	__u64 job;		/* in: user pointer to a job image */
	__u32 job_size;		/* in: bytes, at most 1 MiB */
	__u32 flags;		/* in: 0 */
	__u32 status;		/* out: 0 = done, 1 = timed out */
	__u32 stamp;		/* out: the completion stamp read back */
	__u32 elapsed_us;	/* out */
	__u32 pad;
	__u64 fault[2];		/* out: BIF fault bank 0 MMU / REQ status after the job */
};

#define DRM_IOCTL_APPLE_G4P_GET_PARAM	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_G4P_GET_PARAM, struct drm_apple_g4p_get_param)
#define DRM_IOCTL_APPLE_G4P_BO_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_G4P_BO_CREATE, struct drm_apple_g4p_bo_create)
#define DRM_IOCTL_APPLE_G4P_SUBMIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_G4P_SUBMIT, struct drm_apple_g4p_submit)

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_APPLE_G4P_DRM_H_ */
