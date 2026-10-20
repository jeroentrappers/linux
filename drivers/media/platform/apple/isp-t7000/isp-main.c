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
 *  5 boot stage 2 (IPC pool + boot args)
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
int isp_sens;
module_param_named(sens, isp_sens, int, 0444);
MODULE_PARM_DESC(sens, "1 = also power ps_ispsens0/1 (iOS does; default off)");
int isp_smmu_en;
module_param_named(smmu_en, isp_smmu_en, int, 0444);
MODULE_PARM_DESC(smmu_en, "1 = also set SMMU+0xc byte[sid] |= 0x80 like iOS (default: SMMU read-only)");
int isp_graceful = 1;
module_param_named(graceful, isp_graceful, int, 0444);
MODULE_PARM_DESC(graceful, "1 = STOP + POWER_DOWN handshake before halting a booted fw");
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

/* Reverse unwind. Memory is freed only after the ISP is powered off. */
static void isp_teardown(struct t7000_isp *isp)
{
	dev_info(isp->dev, "teardown from stage %d\n", isp->done);
	isp_ipc_stop(isp);
	mutex_lock(&isp->lock);
	if (isp->fw_up && isp_graceful)
		isp_fw_shutdown(isp);
	if (isp->cpu_started)
		isp_cpu_stop(isp);
	if (isp->dart_on)
		isp_dart_fini(isp);
	isp_power_off(isp);
	isp_surf_free_all(isp);
	isp->done = 0;
	mutex_unlock(&isp->lock);
	isp_pmgr_log(isp, "after teardown");
}

static int isp_run(struct t7000_isp *isp)
{
	int n, ret;

	if (stage < 0 || stage >= ARRAY_SIZE(isp_stages)) {
		dev_err(isp->dev, "stage=%d out of range 0..%zu\n", stage,
			ARRAY_SIZE(isp_stages) - 1);
		return -EINVAL;
	}
	for (n = 1; n <= stage; n++) {
		dev_info(isp->dev, "stage %d: start\n", n);
		if (n > 1 && isp->isp_was_on) {
			dev_err(isp->dev, "ps_isp was already on at load: someone else owns it, refusing writes\n");
			return -EBUSY;
		}
		mutex_lock(&isp->lock);
		ret = isp_stages[n](isp);
		mutex_unlock(&isp->lock);
		if (ret) {
			dev_err(isp->dev, "stage %d: FAILED (%d), unwinding\n", n, ret);
			return ret;
		}
		isp->done = n;
		dev_info(isp->dev, "stage %d: done\n", n);
	}
	return 0;
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
