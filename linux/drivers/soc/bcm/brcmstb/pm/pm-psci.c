// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom STB PSCI based system wide PM support
 *
 * Copyright © 2018 Broadcom
 */

#define pr_fmt(fmt) "brcmstb-pm-psci: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/arm-smccc.h>
#include <linux/psci.h>
#include <linux/suspend.h>
#include <linux/brcmstb/brcmstb.h>
#include <linux/brcmstb/memory_api.h>
#include <linux/soc/brcmstb/brcmstb.h>
#include <linux/brcmstb/brcmstb-smccc.h>
#include <linux/reboot.h>

#include <uapi/linux/psci.h>

#include <asm/suspend.h>
#include <asm/system_misc.h>

#include "pm-common.h"

static psci_fn *invoke_psci_fn;
static bool brcmstb_psci_system_reset2_supported;

static int brcmstb_psci_integ_region(unsigned long function_id,
				     unsigned long base,
				     unsigned long size)
{
	unsigned long end;

	if (!size)
		return -EINVAL;

	end = DIV_ROUND_UP(base + size, SIP_MIN_REGION_SIZE);
	base /= SIP_MIN_REGION_SIZE;
	size = end - base;

	return invoke_psci_fn(function_id, base, size, 0);
}

static int brcmstb_psci_integ_region_set(unsigned long base,
					 unsigned long size)
{
	return brcmstb_psci_integ_region(SIP_FUNC_INTEG_REGION_SET, base, size);
}

static int brcmstb_psci_integ_region_del(unsigned long base,
					 unsigned long size)
{
	return brcmstb_psci_integ_region(SIP_FUNC_INTEG_REGION_DEL, base, size);
}

static int brcmstb_psci_integ_region_reset_all(void)
{
	return invoke_psci_fn(SIP_FUNC_INTEG_REGION_RESET_ALL, 0, 0, 0);
}

static int psci_system_suspend(unsigned long unused)
{
	return invoke_psci_fn(PSCI_FN_NATIVE(1_0, SYSTEM_SUSPEND),
			      virt_to_phys(cpu_resume), 0, 0);
}

int brcmstb_psci_system_mem_finish(void)
{
	struct dma_region combined_regions[MAX_EXCLUDE + MAX_REGION + MAX_EXTRA];
	const int max = ARRAY_SIZE(combined_regions);
	unsigned int i;
	int nregs, ret;

	memset(&combined_regions, 0, sizeof(combined_regions));
	nregs = configure_main_hash(combined_regions, max,
				    exclusions, num_exclusions);
	if (nregs < 0)
		return nregs;

	for (i = 0; i < num_regions && nregs + i < max; i++)
		combined_regions[nregs + i] = regions[i];
	nregs += i;

	for (i = 0; i < nregs; i++) {
		ret = brcmstb_psci_integ_region_set(combined_regions[i].addr,
						    combined_regions[i].len);
		if (ret != PSCI_RET_SUCCESS) {
			pr_err("Error setting combined region %d\n", i);
			continue;
		}
	}

	for (i = 0; i < num_exclusions; i++) {
		ret = brcmstb_psci_integ_region_del(exclusions[i].addr,
						    exclusions[i].len);
		if (ret != PSCI_RET_SUCCESS) {
			pr_err("Error removing exclusion region %d\n", i);
			continue;
		}
	}

	return cpu_suspend(0, psci_system_suspend);
}

static void brcmstb_psci_sys_reset(enum reboot_mode reboot_mode,
				   const char *cmd)
{
	/*
	 * reset_type[31] = 0 (architectural)
	 * reset_type[30:0] = 0 (SYSTEM_WARM_RESET)
	 * cookie = 0 (ignored by the implementation)
	 */
	uint32_t reboot_type = 0;

	if ((reboot_mode == REBOOT_COLD || reboot_mode == REBOOT_WARM ||
	    reboot_mode == REBOOT_SOFT) &&
	    brcmstb_psci_system_reset2_supported) {
		if (cmd && !strcmp(cmd, "powercycle"))
			reboot_type = BIT(31) | 1;
		invoke_psci_fn(PSCI_FN_NATIVE(1_1, SYSTEM_RESET2), reboot_type, 0, 0);
	} else {
		invoke_psci_fn(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0);
	}
}

void brcmstb_psci_sys_poweroff(void)
{
	invoke_psci_fn(PSCI_0_2_FN_SYSTEM_OFF, 0, 0, 0);
}

static int psci_features(u32 psci_func_id)
{
	u32 features_func_id;

	switch (ARM_SMCCC_OWNER_NUM(psci_func_id)) {
	case ARM_SMCCC_OWNER_SIP:
		features_func_id = SIP_FUNC_PSCI_FEATURES;
		break;
	case ARM_SMCCC_OWNER_STANDARD:
		features_func_id = PSCI_1_0_FN_PSCI_FEATURES;
		break;
	default:
		return PSCI_RET_NOT_SUPPORTED;
	}

	return invoke_psci_fn(features_func_id, psci_func_id, 0, 0);
}

static int brcmstb_psci_enter(suspend_state_t state)
{
	/* Request a SYSTEM level power state with retention */
	u32 pstate = 2 << PSCI_0_2_POWER_STATE_AFFL_SHIFT |
		     0 << PSCI_0_2_POWER_STATE_TYPE_SHIFT;
	int ret = -EINVAL;

	switch (state) {
	case PM_SUSPEND_STANDBY:
		ret = psci_ops.cpu_suspend(pstate, 0);
		break;
	case PM_SUSPEND_MEM:
		ret = brcmstb_psci_system_mem_finish();
		break;
	}

	return ret;
}

static int brcmstb_psci_valid(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return true;
	default:
		return false;
	}
}

static const struct platform_suspend_ops brcmstb_psci_ops = {
	.enter	= brcmstb_psci_enter,
	.valid	= brcmstb_psci_valid,
};

static int brcmstb_psci_panic_notify(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	int ret;

	ret = invoke_psci_fn(SIP_FUNC_PANIC_NOTIFY, BRCMSTB_PANIC_MAGIC, 0, 0);
	if (ret != PSCI_RET_SUCCESS)
		return NOTIFY_BAD;

	return NOTIFY_DONE;
}

static struct notifier_block brcmstb_psci_nb = {
	.notifier_call = brcmstb_psci_panic_notify,
};

int brcmstb_pm_psci_init(void)
{
	unsigned long funcs_id[] = {
		PSCI_0_2_FN_SYSTEM_OFF,
		SIP_FUNC_INTEG_REGION_SET,
		SIP_FUNC_INTEG_REGION_DEL,
		SIP_FUNC_INTEG_REGION_RESET_ALL,
	};
	struct arm_smccc_res res = { };
	unsigned int i;
	int ret;

	switch (psci_ops.conduit) {
	case PSCI_CONDUIT_HVC:
		invoke_psci_fn = __invoke_psci_fn_hvc;
		break;
	case PSCI_CONDUIT_SMC:
		invoke_psci_fn = __invoke_psci_fn_smc;
		break;
	default:
		return -EINVAL;
	}

	/* Check the revision of Mon64 */
	if (invoke_psci_fn == __invoke_psci_fn_hvc)
		arm_smccc_hvc(SIP_SVC_REVISION,
			      0, 0, 0, 0, 0, 0, 0, &res);
	else
		arm_smccc_smc(SIP_SVC_REVISION,
			      0, 0, 0, 0, 0, 0, 0, &res);

	/* Test for our supported features */
	for (i = 0; i < ARRAY_SIZE(funcs_id); i++) {
		ret = psci_features(funcs_id[i]);
		if (ret == PSCI_RET_NOT_SUPPORTED) {
			pr_err("Firmware does not support function 0x%lx\n",
			       funcs_id[i]);
			return -EOPNOTSUPP;
		}
	}

	ret = psci_features(PSCI_FN_NATIVE(1_1, SYSTEM_RESET2));
	if (ret != PSCI_RET_NOT_SUPPORTED)
		brcmstb_psci_system_reset2_supported = true;

	ret = brcmstb_psci_integ_region_reset_all();
	if (ret != PSCI_RET_SUCCESS) {
		pr_err("Error resetting all integrity checking regions\n");
		return -EIO;
	}

	/* Firmware is new enough to participate in S3/S5, but does not
	 * take over all suspend operations and still needs assistance
	 * from pm-arm.c.
	 */
	if (res.a0 == SIP_REVISION_MAJOR && res.a1 < SIP_REVISION_MINOR) {
		brcmstb_pm_method = BRCMSTB_PM_PSCI_ASSISTED;
		pr_info("Using PSCI based system PM (assisted)\n");
		return 0;
	}

	/* Firmware is fully taking over the S2/S3/S5 states and requires
	 * only PSCI calls to enter those states
	 */
	ret = brcmstb_memory_get(&bm);
	if (ret)
		return ret;

	ret = brcmstb_regsave_init();
	if (ret)
		return ret;

	brcmstb_pm_method = BRCMSTB_PM_PSCI_FULL;
	pm_power_off = brcmstb_psci_sys_poweroff;
	arm_pm_restart = brcmstb_psci_sys_reset;
	suspend_set_ops(&brcmstb_psci_ops);
	atomic_notifier_chain_register(&panic_notifier_list,
				       &brcmstb_psci_nb);

	pr_info("Using PSCI based system PM (full featured)\n");

	return 0;
}
module_init(brcmstb_pm_psci_init);
