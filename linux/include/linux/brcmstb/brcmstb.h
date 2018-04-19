/*
 * Copyright © 2009-2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * A copy of the GPL is available at
 * http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 * Foundation at https://www.gnu.org/licenses/ .
 */

/*
 * **********************
 * READ ME BEFORE EDITING
 * **********************
 *
 * If you update this file, make sure to bump BRCMSTB_H_VERSION if there is an
 * API change!
 */

#ifndef _ASM_BRCMSTB_BRCMSTB_H
#define _ASM_BRCMSTB_BRCMSTB_H

#define BRCMSTB_H_VERSION  10

#if !defined(__ASSEMBLY__)

#include <linux/types.h>
#include <linux/smp.h>
#include <linux/device.h>
#include <linux/brcmstb/memory_api.h>
#include <linux/brcmstb/irq_api.h>
#include <linux/brcmstb/gpio_api.h>
#include <linux/brcmstb/reg_api.h>

#if defined(CONFIG_MIPS)
#include <asm/addrspace.h>
#include <asm/mipsregs.h>
#include <asm/setup.h>
#include <irq.h>
#include <spaces.h>
#endif

#if defined(CONFIG_ARM_BRCMSTB_AVS_CPUFREQ) || defined(CONFIG_ARM_SCMI_CPUFREQ)
int brcmstb_stb_dvfs_get_pstate(unsigned int idx, unsigned int *pstate,
				u32 *info);
int brcmstb_stb_dvfs_set_pstate(unsigned int idx, unsigned int pstate,
				unsigned int num_clk_writes,
				const u32 *clk_params);
int brcmstb_stb_avs_read_debug(unsigned int debug_idx, u32 *value);
#endif

#if defined(CONFIG_BRCMSTB_PM) && !defined(CONFIG_MIPS)
/*
 * Exclude a given memory range from the MAC authentication process during S3
 * suspend/resume. Ranges are reset after each MAC (i.e., after each S3
 * suspend/resume cycle). Returns non-zero on error.
 */
int brcmstb_pm_mem_exclude(phys_addr_t addr, size_t len);
/* So users can determine whether the kernel provides this API */
#define BRCMSTB_HAS_PM_MEM_EXCLUDE

/* Add region to be hashed during S3 suspend/resume. */
int brcmstb_pm_mem_region(phys_addr_t addr, size_t len);
#define BRCMSTB_HAS_PM_MEM_REGION
#endif

#endif /* !defined(__ASSEMBLY__) */

#endif /* _ASM_BRCMSTB_BRCMSTB_H */
