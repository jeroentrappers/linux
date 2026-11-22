/* SPDX-License-Identifier: (GPL-2.0-only WITH Linux-syscall-note) OR MIT */
/*
 * Apple A8 GPU (G4P) DRM interface, bring-up version (linux-on-iphone6 re/GPU-BRINGUP.md 8.64).
 *
 * Experimental and not stable: compute only. Every open file has its own GPU address space (8.77, 8.79): its pool
 * buffers and separate buffers are mapped only in it (plus pool page 0, the driver's programs, read-only; none of
 * the firmware's memory, 8.81). Job
 * images may contain STREAM records only, unless the module runs with raw_jobs=1 (bring-up tools). Buffers are carved
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
#define DRM_APPLE_G4P_VM_BIND		0x03

enum drm_apple_g4p_param {
	DRM_APPLE_G4P_PARAM_POOL_VA = 1,	/* GPU VA of the pool (APPLE_G4P_POOL_VA) */
	DRM_APPLE_G4P_PARAM_POOL_SIZE = 2,	/* bytes */
	DRM_APPLE_G4P_PARAM_CORE_ID = 3,	/* RGX CORE_ID, valid once the GPU has been powered */
	DRM_APPLE_G4P_PARAM_CORE_REVISION = 4,
	DRM_APPLE_G4P_PARAM_MEMORY_CONTEXT = 5,	/* this file's GPU address space (1..31), 8.77, 8.84 */
};

struct drm_apple_g4p_get_param {
	__u32 param;		/* in: enum drm_apple_g4p_param */
	__u32 pad;
	__u64 value;		/* out */
};

/*
 * BO_CREATE flags. APPLE_G4P_BO_SEPARATE: own allocation outside the pool, at GPU VA 0x320000000 and up, mapped
 * into the running firmware boot immediately (8.65: post-boot mappings, made visible by the per-job MMU
 * invalidate) and before every later boot. Its GPU VA is not a code offset (keep code in pool BOs).
 */
#define APPLE_G4P_BO_SEPARATE		(1 << 0)
/* with SEPARATE: no GPU VA yet, user space binds it with VM_BIND (gpu_va returns 0) */
#define APPLE_G4P_BO_NOMAP		(1 << 1)

struct drm_apple_g4p_bo_create {
	__u64 size;		/* in: bytes, rounded up to 4 KiB */
	__u32 flags;		/* in: APPLE_G4P_BO_* */
	__u32 handle;		/* out: GEM handle */
	__u64 gpu_va;		/* out: GPU VA (pool offset = gpu_va - APPLE_G4P_POOL_VA) */
	__u64 mmap_offset;	/* out: offset for mmap() on the DRM fd */
};

/*
 * SUBMIT flags. APPLE_G4P_SUBMIT_ASYNC (8.68): return right after the kick; the job's fence is installed in
 * the syncobj out_sync (DRM_IOCTL_SYNCOBJ_* to wait), signalled from the firmware's completion interrupt, or with
 * an error after timeout_ms (the GPU is then powered off for recovery). Up to four submits are in flight; a
 * further async submit waits for the oldest, a synchronous one for all. A job's fence signals only once all of
 * its stores are in memory (8.78). status/stamp/elapsed_us/fault are only filled in for synchronous submits. Fence errors (8.80): -EFAULT the job faulted
 * (GPU MMU, found within ~20 ms), -ECANCELED dropped by the recovery after another job failed, -ETIMEDOUT (1 s);
 * read them through a sync file (SYNC_IOC_FILE_INFO).
 */
#define APPLE_G4P_SUBMIT_ASYNC		(1 << 0)

struct drm_apple_g4p_submit {
	__u64 job;		/* in: user pointer to a job image */
	__u32 job_size;		/* in: bytes, at most 1 MiB */
	__u32 flags;		/* in: APPLE_G4P_SUBMIT_* */
	__u32 status;		/* out: 0 = done, 1 = timed out */
	__u32 stamp;		/* out: the completion stamp read back */
	__u32 elapsed_us;	/* out */
	__u32 pad;
	__u64 fault[2];		/* out: BIF fault bank 0 MMU / REQ status after the job */
	__u32 out_sync;		/* in: syncobj handle for the job's fence (ASYNC), 0 = none */
	__u32 pad2;
};

/*
 * VM_BIND (8.71): map a NOMAP buffer at a GPU VA chosen by user space, or unmap it. Since 8.77 every open file
 * has its own GPU address space for the user range [APPLE_G4P_USER_VA_START, APPLE_G4P_USER_VA_END): separate
 * buffers are only visible to the jobs of the file that created them, like its pool buffers. At most 31 files can be
 * open at once (open fails with EBUSY); the firmware's seven memory context slots are shared among them (8.84).
 * Bindings are 4 KiB aligned, one per
 * buffer, whole buffer, no overlaps. Unmapping (and freeing a bound buffer) waits for the jobs in flight and
 * makes the unmap effective before it returns.
 */
#define APPLE_G4P_USER_VA_START		0x320000000ULL
#define APPLE_G4P_USER_VA_END		0x340000000ULL
#define APPLE_G4P_VM_BIND_MAP		0
#define APPLE_G4P_VM_BIND_UNMAP		1

struct drm_apple_g4p_vm_bind {
	__u32 handle;		/* in: GEM handle of a SEPARATE|NOMAP buffer */
	__u32 op;		/* in: APPLE_G4P_VM_BIND_* */
	__u64 gpu_va;		/* in (MAP): where */
};

#define DRM_IOCTL_APPLE_G4P_GET_PARAM	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_G4P_GET_PARAM, struct drm_apple_g4p_get_param)
#define DRM_IOCTL_APPLE_G4P_BO_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_G4P_BO_CREATE, struct drm_apple_g4p_bo_create)
#define DRM_IOCTL_APPLE_G4P_SUBMIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_G4P_SUBMIT, struct drm_apple_g4p_submit)
#define DRM_IOCTL_APPLE_G4P_VM_BIND	DRM_IOW(DRM_COMMAND_BASE + DRM_APPLE_G4P_VM_BIND, struct drm_apple_g4p_vm_bind)

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_APPLE_G4P_DRM_H_ */
