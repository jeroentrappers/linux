// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple A8 (T7000) ISP bring-up test driver: module entry + stage runner.
 *
 * insmod apple-isp.ko stage=N runs stages 1..N and stays loaded; rmmod (or any
 * failure) unwinds in reverse: CPU halt, DART off, power off what we turned on,
 * then free memory. stage=0 (default) only logs the PMGR state; dry_run=1 only
 * reads registers (and peeks at the firmware file), it never writes anything.
 *
 *  1 power-up (PMGR media, isp) + version read     6 boot stage 3 (channel table)
 *  2 bridge settings + clock enable                7 mailbox + rings, poll worker
 *  3 mini DART + firmware into a DART surface      8 PRINT_ENABLE + START
 *  4 ISP CPU start, wait for the wake magic        9 CONFIG_GET + BUILDINFO
 *  5 boot stage 2 (IPC pool + boot args)      (8 starts with the sensor power-up, sens_pwr=1)
 *
 * stream=1 (with stage=9) adds /dev/isp0 (isp-cdev.c): the ISP is powered off again
 * after the boot test unless keep_on=1, booted on open and powered off on last close.
 */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "isp.h"

static int stage;
module_param(stage, int, 0444);
MODULE_PARM_DESC(stage, "run stages 1..N and stay loaded (0 = only log PMGR state)");
int isp_dry_run;
module_param_named(dry_run, isp_dry_run, int, 0444);
MODULE_PARM_DESC(dry_run, "1 = only read + log registers (ISP window only if already powered)");
int isp_irq_en;
module_param_named(irq, isp_irq_en, int, 0444);
MODULE_PARM_DESC(irq, "1 = use AIC IRQ 91 for the mailbox (default: poll)");
int isp_sens = 1;
module_param_named(sens, isp_sens, int, 0444);
MODULE_PARM_DESC(sens, "1 = also power ps_ispsens0/1 like the kext EnableISPClocksAndPower (default); 0 = ISP only");
int isp_smmu_en;
module_param_named(smmu_en, isp_smmu_en, int, 0444);
MODULE_PARM_DESC(smmu_en, "1 = also set SMMU+0xc byte[sid] |= 0x80 like iOS (default: SMMU read-only)");
int isp_graceful = 1;
module_param_named(graceful, isp_graceful, int, 0444);
MODULE_PARM_DESC(graceful, "1 = STOP + POWER_DOWN handshake before halting a booted fw");
int isp_fwlog;
module_param_named(fwlog, isp_fwlog, int, 0644);
MODULE_PARM_DESC(fwlog, "1 = print fw TERMINAL lines (ISPCPU:) at info level, rate-limited; default dev_dbg");
uint isp_fw_iova;
module_param_named(fw_iova, isp_fw_iova, uint, 0444);
MODULE_PARM_DESC(fw_iova, "DART IOVA of the firmware (default 0: fw-relative == absolute)");
uint isp_dart_sids = 0xf;
module_param_named(dart_sids, isp_dart_sids, uint, 0444);
MODULE_PARM_DESC(dart_sids, "DART stream mask to translate (default 0xf = all 4)");
uint isp_platform_id = 0x2a;
module_param_named(platform_id, isp_platform_id, uint, 0444);
MODULE_PARM_DESC(platform_id, "boot-args platform id (ADT sensor-type, default 0x2a)");
uint isp_poll_ms = 50;
module_param_named(poll_ms, isp_poll_ms, uint, 0444);
MODULE_PARM_DESC(poll_ms, "ring poll interval once the fw is up");
uint isp_boot_timeout_ms = 10000;
module_param_named(boot_timeout_ms, isp_boot_timeout_ms, uint, 0444);
MODULE_PARM_DESC(boot_timeout_ms, "timeout of each boot handshake (iOS: 10 s)");
uint isp_cmd_timeout_ms = 2000;
module_param_named(cmd_timeout_ms, isp_cmd_timeout_ms, uint, 0444);
MODULE_PARM_DESC(cmd_timeout_ms, "timeout of one fw command (iOS: 2 s)");
uint isp_max_shared_mb = 64;
module_param_named(max_shared_mb, isp_max_shared_mb, uint, 0444);
MODULE_PARM_DESC(max_shared_mb, "cap on SHAREDMALLOC memory");

int isp_sens_pwr = 1;
module_param_named(sens_pwr, isp_sens_pwr, int, 0444);
MODULE_PARM_DESC(sens_pwr, "1 = power the sensors before START like iOS power_on_hardware (GPIO 198/190/196, ref clocks, extclk pins)");
int isp_refclk = 1;
module_param_named(refclk, isp_refclk, int, 0444);
MODULE_PARM_DESC(refclk, "1 = program the PMGR sensor ref clocks (0x20e010218/1c) with refclk_cfg");
uint isp_refclk_cfg = 0x28;
module_param_named(refclk_cfg, isp_refclk_cfg, uint, 0444);
MODULE_PARM_DESC(refclk_cfg, "ISPC byte: srcSel = bits 7:6, divider cfg = bits 5:0 (ADT ISPC arg byte 0 = 0x28)");
int isp_extclk_periph = 1;
module_param_named(extclk_periph, isp_extclk_periph, int, 0444);
MODULE_PARM_DESC(extclk_periph, "pin function (PERIPH) for the MCLK pins 191/197; -1 = leave as iBoot set them");
int isp_stream_en;
module_param_named(stream, isp_stream_en, int, 0444);
MODULE_PARM_DESC(stream, "1 (with stage=9) = streaming via /dev/isp0");
int isp_stream_dump;
module_param_named(stream_dump, isp_stream_dump, int, 0644);
MODULE_PARM_DESC(stream_dump, "1 = log every frame (seq, index, bytes) and every buffer list");
int isp_keep_on;
module_param_named(keep_on, isp_keep_on, int, 0644);
MODULE_PARM_DESC(keep_on, "stream=1: 1 = keep the ISP booted when /dev/isp0 is closed (default: power everything off, boot again on open)");
int isp_preset = -1;
module_param_named(preset, isp_preset, int, 0644);
MODULE_PARM_DESC(preset, "sensor preset to stream (-1 = smallest output >= 640x480)");
uint isp_out_range = 1;
module_param_named(out_range, isp_out_range, uint, 0644);
MODULE_PARM_DESC(out_range, "OUTPUT_CONFIG_SET +0x14, logged by the fw as 'range' (Asahi: colorspace 1 = REC709)");
uint isp_nbufs = 4;
module_param_named(nbufs, isp_nbufs, uint, 0444);
MODULE_PARM_DESC(nbufs, "frame buffers (2..8)");
uint isp_setfile_rear = 0x8421;
module_param_named(setfile_rear, isp_setfile_rear, uint, 0644);
MODULE_PARM_DESC(setfile_rear, "IMX237 module setfile apple/isp_<hex>_01XX.dat (0x8421/0x8422/0x8423, 0 = none)");
uint isp_setfile_front = 0x1421;
module_param_named(setfile_front, isp_setfile_front, uint, 0644);
MODULE_PARM_DESC(setfile_front, "IMX190 module setfile (0x1421/0x1422, 0 = none)");

static struct t7000_isp *the_isp;

static int isp_stage_power(struct t7000_isp *isp)
{
	return isp_power_on(isp);
}

static int (*const isp_stages[])(struct t7000_isp *) = {
	NULL,
	isp_stage_power,	/* 1 */
	isp_stage_clocks,	/* 2 */
	isp_stage_fw,		/* 3 */
	isp_stage_cpu,		/* 4 */
	isp_stage_args,		/* 5 */
	isp_stage_chans,	/* 6 */
	isp_stage_mbox,		/* 7 */
	isp_stage_start,	/* 8 */
	isp_stage_query,	/* 9 */
};

/* Reverse unwind. Memory is freed only after the ISP is powered off. Caller: no isp->lock. */
void isp_teardown(struct t7000_isp *isp)
{
	dev_info(isp->dev, "teardown from stage %d\n", isp->done);
	if (isp->st.on) {
		mutex_lock(&isp->lock);
		isp_stream_stop(isp);
		mutex_unlock(&isp->lock);
	}
	isp_ipc_stop(isp);
	mutex_lock(&isp->lock);
	if (isp->fw_up && isp_graceful)
		isp_fw_shutdown(isp);
	if (isp->cpu_started)
		isp_cpu_stop(isp);
	isp_sensor_power_off(isp);
	if (isp->dart_on)
		isp_dart_fini(isp);
	if (isp_power_off(isp) && (isp->l1 || !list_empty(&isp->surfs))) {
		/*
		 * ps_isp did not gate: the ISP (and its DART) may still be live, so
		 * every page it could reach stays allocated. Leaked on purpose.
		 */
		dev_err(isp->dev, "!!! ps_isp power-off TIMED OUT: LEAKING all ISP surfaces + DART tables (%llu bytes) so the ISP cannot DMA into reused memory; reboot before the next test !!!\n",
			isp->shared_bytes);
		INIT_LIST_HEAD(&isp->surfs);
		isp->fw = isp->ipc = isp->heap = NULL;
		isp->l1 = NULL;
		memset(isp->l2, 0, sizeof(isp->l2));
	} else {
		isp_surf_free_all(isp);
	}
	/* stream buffers were surfaces: forget them (counters stay for debugfs) */
	memset(isp->st.buf, 0, sizeof(isp->st.buf));
	memset(isp->st.meta, 0, sizeof(isp->st.meta));
	isp->st.lists = isp->st.setf = NULL;
	isp->st.nbufs = 0;
	/* per-boot state, so isp_boot() can run again (stream=1, open after idle-off) */
	isp->done = 0;
	isp->started = isp->io_dead = isp->fw_up = false;
	isp->nchans = isp->pool_used = 0;
	isp->io = isp->sm = isp->tm = isp->bh = isp->bt = isp->it = isp->dg = NULL;
	memset(isp->chans, 0, sizeof(isp->chans));
	mutex_unlock(&isp->lock);
	isp_term_flush(isp);
	isp_pmgr_log(isp, "after teardown");
}

/* Run stages done+1 .. upto. On failure the caller unwinds with isp_teardown(). */
int isp_boot(struct t7000_isp *isp, int upto)
{
	int n, ret;

	for (n = isp->done + 1; n <= upto; n++) {
		dev_info(isp->dev, "stage %d: start\n", n);
		if (n > 1 && isp->isp_was_on) {
			dev_err(isp->dev, "ps_isp was already on at load: someone else owns it, refusing writes\n");
			return -EBUSY;
		}
		mutex_lock(&isp->lock);
		ret = isp_stages[n](isp);
		mutex_unlock(&isp->lock);
		isp_term_flush(isp);
		if (ret) {
			dev_err(isp->dev, "stage %d: FAILED (%d), unwinding\n", n, ret);
			return ret;
		}
		isp->done = n;
		dev_info(isp->dev, "stage %d: done\n", n);
	}
	return 0;
}

static int isp_run(struct t7000_isp *isp)
{
	if (stage < 0 || stage >= ARRAY_SIZE(isp_stages)) {
		dev_err(isp->dev, "stage=%d out of range 0..%zu\n", stage,
			ARRAY_SIZE(isp_stages) - 1);
		return -EINVAL;
	}
	if (isp_stream_en && stage != ARRAY_SIZE(isp_stages) - 1) {
		dev_err(isp->dev, "stream=1 needs stage=%zu\n", ARRAY_SIZE(isp_stages) - 1);
		return -EINVAL;
	}
	return isp_boot(isp, stage);
}

static int __init isp_init(void)
{
	struct platform_device_info pinfo = {
		.name = "apple-isp-t7000",
		.id = PLATFORM_DEVID_NONE,
		.dma_mask = DMA_BIT_MASK(64),
	};
	struct t7000_isp *isp;
	int ret;

	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;
	mutex_init(&isp->lock);
	spin_lock_init(&isp->term_lock);
	spin_lock_init(&isp->st.frame_lock);
	init_waitqueue_head(&isp->st.wait);
	INIT_LIST_HEAD(&isp->surfs);
	atomic_set(&isp->irq_count, 0);
	isp->irq = -1;

	isp->pdev = platform_device_register_full(&pinfo);
	if (IS_ERR(isp->pdev)) {
		ret = PTR_ERR(isp->pdev);
		goto err_free;
	}
	isp->dev = &isp->pdev->dev;
	dev_info(isp->dev, "load: stage=%d dry_run=%d irq=%d sens=%d smmu_en=%d fw_iova=0x%x dart_sids=0x%x platform_id=0x%x\n",
		 stage, isp_dry_run, isp_irq_en, isp_sens, isp_smmu_en,
		 isp_fw_iova, isp_dart_sids, isp_platform_id);

	ret = -ENOMEM;
	isp->pmgr = ioremap_np(PMGR_PS_PHYS, PMGR_PS_SIZE);
	isp->regs = ioremap_np(ISP_PHYS_BASE, ISP_PHYS_SIZE);
	if (!isp->pmgr || !isp->regs)
		goto err_unmap;

	isp_pmgr_log(isp, "load");
	if (isp_dry_run) {
		isp_dry_read(isp);
		goto ok;
	}
	if (isp_fw_iova & 0xfff || isp_fw_iova >= 0x30000000 || !isp_dart_sids ||
	    isp_dart_sids > 0xf) {
		dev_err(isp->dev, "bad fw_iova/dart_sids\n");
		ret = -EINVAL;
		goto err_unmap;
	}
	ret = isp_run(isp);
	if (ret) {
		isp_teardown(isp);
		goto err_unmap;
	}
	if (isp_stream_en) {
		ret = isp_cdev_init(isp);
		if (ret) {
			isp_teardown(isp);
			goto err_unmap;
		}
		if (!isp_keep_on) {
			dev_info(isp->dev, "stream: boot ok; powering the ISP off until /dev/isp0 is opened (keep_on=0)\n");
			isp_teardown(isp);
		}
	}
ok:
	dev_info(isp->dev, "loaded, stages done: %d (rmmod unwinds)\n", isp->done);
	the_isp = isp;
	return 0;

err_unmap:
	if (isp->regs)
		iounmap(isp->regs);
	if (isp->pmgr)
		iounmap(isp->pmgr);
	platform_device_unregister(isp->pdev);
err_free:
	kfree(isp);
	return ret;
}

static void __exit isp_exit(void)
{
	struct t7000_isp *isp = the_isp;

	isp_cdev_exit(isp);	/* no open fd can exist: each one holds a module reference */
	if (!isp_dry_run)
		isp_teardown(isp);
	iounmap(isp->regs);
	iounmap(isp->pmgr);
	dev_info(isp->dev, "unloaded\n");
	platform_device_unregister(isp->pdev);
	kfree(isp);
}

module_init(isp_init);
module_exit(isp_exit);
MODULE_DESCRIPTION("Apple A8 (T7000) ISP bring-up test driver");
MODULE_LICENSE("GPL");
