// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU idle driver for Apple A7-A11 cores (Cyclone to Monsoon/Mistral)
 *
 * These cores cannot keep their state across a core power-down: "deep WFI"
 * (CYC_OVRD WFI mode 3) resets the core to RVBAR, in EL3 on parts that have
 * one. The boot loader (m1n1) also forces WFI to clock-gate only (CYC_OVRD
 * ok2pwrdn = force up). Without a resume path through the reset vector the
 * only usable state is WFI, which is what this driver provides, with two
 * additions over the default arm64 idle loop:
 *
 *  - the iOS WFI workaround for Typhoon (A8/A8X): around every idle WFI the
 *    iOS 12 kernel sets HID2 bit 13 (disMMUmtlbPrefetch) and clears it again
 *    afterwards (typhoon_prepare_for_wfi / typhoon_return_from_wfi, for a core
 *    that could get stuck entering WFI because of an MMU prefetch). The
 *    wfi_wa parameter turns it off.
 *  - cpuidle usage and residency statistics in sysfs, which the default idle
 *    loop does not have (the time a core really spends idle).
 *
 * The HID2 write is the only new instruction in the idle path. The driver
 * reads and writes back HID2 once on every CPU, in process context, before it
 * registers, so an unexpected trap shows up as an oops in a kworker at load
 * time and not in the idle loop.
 */

#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/smp.h>

#include <asm/barrier.h>
#include <asm/cpuidle.h>
#include <asm/cputype.h>
#include <asm/sysreg.h>

#define SYS_IMP_APL_HID2			sys_reg(3, 0, 15, 2, 0)
#define HID2_DIS_MMU_MTLB_PREFETCH		BIT_ULL(13)

static bool wfi_wa = true;
module_param(wfi_wa, bool, 0644);
MODULE_PARM_DESC(wfi_wa, "Typhoon (A8/A8X): set HID2.disMMUmtlbPrefetch around WFI, as iOS does (default: on)");

static bool typhoon;

static const struct midr_range apple_a7_a11[] = {
	MIDR_ALL_VERSIONS(MIDR_APPLE_A7_CYCLONE),
	MIDR_ALL_VERSIONS(MIDR_APPLE_A8_TYPHOON),
	MIDR_ALL_VERSIONS(MIDR_APPLE_A8X_TYPHOON),
	MIDR_ALL_VERSIONS(MIDR_APPLE_SAMSUNG_TWISTER),
	MIDR_ALL_VERSIONS(MIDR_APPLE_TSMC_TWISTER),
	MIDR_ALL_VERSIONS(MIDR_APPLE_A10_T2_HURRICANE_ZEPHYR),
	MIDR_ALL_VERSIONS(MIDR_APPLE_A10X_HURRICANE_ZEPHYR),
	MIDR_ALL_VERSIONS(MIDR_APPLE_A11_MONSOON),
	MIDR_ALL_VERSIONS(MIDR_APPLE_A11_MISTRAL),
	{}
};

static const struct midr_range apple_typhoon[] = {
	MIDR_ALL_VERSIONS(MIDR_APPLE_A8_TYPHOON),
	MIDR_ALL_VERSIONS(MIDR_APPLE_A8X_TYPHOON),
	{}
};

/* cpu_do_idle() is not exported: the same sequence, with the HID2 bit around it on Typhoon */
static __cpuidle int apple_idle_enter_wfi(struct cpuidle_device *dev,
					  struct cpuidle_driver *drv, int index)
{
	struct arm_cpuidle_irq_context context;
	bool wa = typhoon && READ_ONCE(wfi_wa);

	arm_cpuidle_save_irq_context(&context);

	if (wa) {
		sysreg_clear_set_s(SYS_IMP_APL_HID2, 0, HID2_DIS_MMU_MTLB_PREFETCH);
		isb();
	}

	dsb(sy);
	wfi();

	if (wa) {
		sysreg_clear_set_s(SYS_IMP_APL_HID2, HID2_DIS_MMU_MTLB_PREFETCH, 0);
		dsb(sy);
		isb();
	}

	arm_cpuidle_restore_irq_context(&context);

	return index;
}

static struct cpuidle_driver apple_idle_driver = {
	.name = "apple_idle",
	.owner = THIS_MODULE,
	.states[0] = {
		.enter			= apple_idle_enter_wfi,
		.exit_latency		= 1,
		.target_residency	= 1,
		.power_usage		= UINT_MAX,
		.name			= "WFI",
		.desc			= "WFI, clock gated",
	},
	.state_count = 1,
};

/* in process context on @cpu: a trap here is an oops in a kworker, not a hang in the idle loop */
static int apple_idle_probe_hid2(void *unused)
{
	u64 v = read_sysreg_s(SYS_IMP_APL_HID2);

	write_sysreg_s(v, SYS_IMP_APL_HID2);
	isb();
	return 0;
}

static int __init apple_idle_init(void)
{
	unsigned int cpu;
	int ret;

	if (!of_machine_is_compatible("apple,arm-platform") ||
	    !is_midr_in_range_list(apple_a7_a11))
		return -ENODEV;

	typhoon = is_midr_in_range_list(apple_typhoon);
	if (typhoon) {
		for_each_online_cpu(cpu) {
			ret = smp_call_on_cpu(cpu, apple_idle_probe_hid2, NULL, false);
			if (ret)
				return ret;
		}
	}

	ret = cpuidle_register(&apple_idle_driver, NULL);
	if (ret)
		return ret;

	pr_info("apple_idle: WFI%s\n",
		typhoon ? (wfi_wa ? " with the Typhoon HID2 workaround" : " (Typhoon workaround off)") : "");
	return 0;
}
module_init(apple_idle_init);

static void __exit apple_idle_exit(void)
{
	cpuidle_unregister(&apple_idle_driver);
}
module_exit(apple_idle_exit);

MODULE_DESCRIPTION("CPU idle driver for Apple A7-A11 cores");
MODULE_LICENSE("GPL");
