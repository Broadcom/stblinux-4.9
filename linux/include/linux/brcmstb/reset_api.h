/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2019 Broadcom */
#ifndef _ASM_BRCMSTB_RESET_API_H
#define _ASM_BRCMSTB_RESET_API_H

#define BRST_SW_OFFSET		0x5000

#define BRST_SW_CPU_CORE	(BRST_SW_OFFSET + 0x0)
#define BRST_SW_V3D		(BRST_SW_OFFSET + 0x1)
#define BRST_SW_SYSIF		(BRST_SW_OFFSET + 0x2)
#define BRST_SW_SCB		(BRST_SW_OFFSET + 0x3)
#define BRST_SW_HVD0		(BRST_SW_OFFSET + 0x4)
#define BRST_SW_RAAGA0		(BRST_SW_OFFSET + 0x5)
#define BRST_SW_VICE0		(BRST_SW_OFFSET + 0x6)
#define BRST_SW_VICE0_PSS	(BRST_SW_OFFSET + 0x7)
#define BRST_SW_VICE1		(BRST_SW_OFFSET + 0x8)
#define BRST_SW_VICE1_PSS	(BRST_SW_OFFSET + 0x9)
#define BRST_SW_XPT		(BRST_SW_OFFSET + 0xa)
#define BRST_SW_M2MC0		(BRST_SW_OFFSET + 0xb)
#define BRST_SW_M2MC1		(BRST_SW_OFFSET + 0xc)
#define BRST_SW_MIPMAP0		(BRST_SW_OFFSET + 0xd)
#define BRST_SW_TSX0		(BRST_SW_OFFSET + 0xe)
#define BRST_SW_SMARTCARD0	(BRST_SW_OFFSET + 0xf)
#define BRST_SW_SMARTCARD1	(BRST_SW_OFFSET + 0x10)
#define BRST_SW_VPU0		(BRST_SW_OFFSET + 0x11)
#define BRST_SW_BNE0		(BCLK_SW_OFFSET + 0x12)
#define BRST_SW_ASP0		(BCLK_SW_OFFSET + 0x13)
/* If you add a reset core, please update below */
#define BRST_SW_NUM_CORES	(BRST_SW_ASP0 + 1)



#ifdef CONFIG_BRCMSTB_NEXUS_RESET_API
int brcm_reset_assert(unsigned int rst_id);
int brcm_reset_deassert(unsigned int rst_id);
int brcm_overtemp_reset(unsigned int temp);

#else
static inline int brcm_reset_assert(unsigned int rst_id)
{
	return -ENOTSUPP;
}
static inline int brcm_reset_deassert(unsigned int rst_id)
{
	return -ENOTSUPP;
}
static inline int brcm_overtemp_reset(unsigned int temp)
{
	return -ENOTSUPP;
}
#endif
#endif
