/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __BRCMSTB_PM_PSCI_H
#define __BRCMSTB_PM_PSCI_H

#include <linux/err.h>

#ifdef CONFIG_ARM_PSCI_FW
int brcmstb_psci_system_mem_finish(void);
void brcmstb_psci_sys_poweroff(void);
int brcmstb_pm_psci_init(void);
#else
static inline int brcmstb_psci_system_mem_finish(void)
{
	return -EOPNOTSUPP;
}

static inline void brcmstb_psci_sys_poweroff(void) { }

static inline int brcmstb_pm_psci_init(void)
{
	return -EOPNOTSUPP;
}
#endif /* CONFIG_ARM_PSCI_FW */

#endif /* __BRCMSTB_PM_PSCI_H */
