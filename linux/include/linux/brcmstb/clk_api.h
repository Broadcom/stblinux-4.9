/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2018 Broadcom */
#ifndef _ASM_BRCMSTB_CLK_API_H
#define _ASM_BRCMSTB_CLK_API_H

#define BCLK_NULL		0xffffffff
#define BCLK_HW_OFFSET		0x0
#define BCLK_SW_OFFSET		0x5000

/*
 * o The following may be considered clocks, Pmap cores, or both.
 * o The following constants are permanent and cannot be changed,
 *   or moved to the 2nd and 3rd lists.
 * o New constants may be added to the end of the following list.
 * o Constants from the second list may be "promoted" to the following list.
 * o The following list is stored internally by AMS/mon64.
 */
#define BCLK_SW_CPU_CORE	(BCLK_SW_OFFSET + 0x0)
#define BCLK_SW_V3D		(BCLK_SW_OFFSET + 0x1)
#define BCLK_SW_SYSIF		(BCLK_SW_OFFSET + 0x2)
#define BCLK_SW_SCB		(BCLK_SW_OFFSET + 0x3)
#define BCLK_SW_HVD0		(BCLK_SW_OFFSET + 0x4)
#define BCLK_SW_RAAGA0		(BCLK_SW_OFFSET + 0x5)
#define BCLK_SW_VICE0		(BCLK_SW_OFFSET + 0x6)
#define BCLK_SW_VICE0_PSS	(BCLK_SW_OFFSET + 0x7)
#define BCLK_SW_VICE1		(BCLK_SW_OFFSET + 0x8)
#define BCLK_SW_VICE1_PSS	(BCLK_SW_OFFSET + 0x9)
#define BCLK_SW_XPT		(BCLK_SW_OFFSET + 0xa)
#define BCLK_SW_M2MC0		(BCLK_SW_OFFSET + 0xb)
#define BCLK_SW_M2MC1		(BCLK_SW_OFFSET + 0xc)
#define BCLK_SW_MIPMAP0		(BCLK_SW_OFFSET + 0xd)
#define BCLK_SW_TSX0		(BCLK_SW_OFFSET + 0xe)
#define BCLK_SW_SMARTCARD0	(BCLK_SW_OFFSET + 0xf)
#define BCLK_SW_SMARTCARD1	(BCLK_SW_OFFSET + 0x10)
#define BCLK_SW_VPU0		(BCLK_SW_OFFSET + 0x11)
#define BCLK_SW_BNE0		(BCLK_SW_OFFSET + 0x12)
#define BCLK_SW_ASP0		(BCLK_SW_OFFSET + 0x13)
/* If you add a clk/core above, please update below */
#define BCLK_SW_NUM_CORES	(BCLK_SW_ASP0 + 1)

/* Keep some space reserved for future cores.  */

/*
 * o The following lists can be rearranged if desired.
 * o Entries from the following lists may be promoted
 *   to the top/core list if needed.
 */
#define BCLK_SW_AIO		(BCLK_SW_OFFSET + 0x30)
#define BCLK_SW_BVN		(BCLK_SW_OFFSET + 0x31)
#define BCLK_SW_DVPHR		(BCLK_SW_OFFSET + 0x32)
#define BCLK_SW_DVPHT		(BCLK_SW_OFFSET + 0x33)
#define BCLK_SW_GENET0		(BCLK_SW_OFFSET + 0x34)
#define BCLK_SW_GENETWOL0	(BCLK_SW_OFFSET + 0x35)
#define BCLK_SW_HVD0_CPU	(BCLK_SW_OFFSET + 0x36)
#define BCLK_SW_ITU656		(BCLK_SW_OFFSET + 0x37)
#define BCLK_SW_MMM2MC0		(BCLK_SW_OFFSET + 0x38)
#define BCLK_SW_PCIE0		(BCLK_SW_OFFSET + 0x39)
#define BCLK_SW_PCIE1		(BCLK_SW_OFFSET + 0x3a)
#define BCLK_SW_POTP		(BCLK_SW_OFFSET + 0x3b)
#define BCLK_SW_RAAGA0_CPU	(BCLK_SW_OFFSET + 0x3c)
#define BCLK_SW_SATA3		(BCLK_SW_OFFSET + 0x3d)
#define BCLK_SW_SDIO0		(BCLK_SW_OFFSET + 0x3e)
#define BCLK_SW_SDIO1		(BCLK_SW_OFFSET + 0x3f)
#define BCLK_SW_SID		(BCLK_SW_OFFSET + 0x40)
#define BCLK_SW_V3D_CPU		(BCLK_SW_OFFSET + 0x41)
#define BCLK_SW_VEC		(BCLK_SW_OFFSET + 0x42)
#define BCLK_SW_XPT_WAKEUP	(BCLK_SW_OFFSET + 0x43)
#define BCLK_SW_TSIO		(BCLK_SW_OFFSET + 0x44)

#define BCLK_SW_AIO_SRAM	(BCLK_SW_OFFSET + 0x60)
#define BCLK_SW_BVN_SRAM	(BCLK_SW_OFFSET + 0x61)
#define BCLK_SW_DVPHR_SRAM	(BCLK_SW_OFFSET + 0x62)
#define BCLK_SW_HVD0_SRAM	(BCLK_SW_OFFSET + 0x63)
#define BCLK_SW_M2MC0_SRAM	(BCLK_SW_OFFSET + 0x64)
#define BCLK_SW_M2MC1_SRAM	(BCLK_SW_OFFSET + 0x65)
#define BCLK_SW_MMM2MC0_SRAM	(BCLK_SW_OFFSET + 0x66)
#define BCLK_SW_RAAGA0_SRAM	(BCLK_SW_OFFSET + 0x67)
#define BCLK_SW_V3D_SRAM	(BCLK_SW_OFFSET + 0x68)
#define BCLK_SW_VEC_SRAM	(BCLK_SW_OFFSET + 0x69)
#define BCLK_SW_VICE0_SRAM	(BCLK_SW_OFFSET + 0x6a)
#define BCLK_SW_VICE1_SRAM	(BCLK_SW_OFFSET + 0x6b)
#define BCLK_SW_XPT_SRAM	(BCLK_SW_OFFSET + 0x6c)

#ifdef CONFIG_BRCMSTB_NEXUS_CLK_API
int brcm_clk_prepare_enable(unsigned int clk_id);
void  brcm_clk_disable_unprepare(unsigned int clk_id);

int brcm_pmap_show(void);
int brcm_pmap_num_pstates(unsigned int core_id, unsigned int *num_pstates);
int brcm_pmap_get_pstate(unsigned int core_id, unsigned int *pstate);
int brcm_pmap_set_pstate(unsigned int core_id, unsigned int pstate);
/* Returns the frequencies in kHz of pstates [0..N-1] */
int brcm_pmap_get_pstate_freqs(unsigned int core_id, unsigned int *freqs);

#else
static inline int brcm_clk_prepare_enable(unsigned int clk_id)
{
	return -ENOTSUPP;
}
static inline void  brcm_clk_disable_unprepare(unsigned int clk_id)
{
}

static inline int brcm_pmap_show(void)
{
	return -ENOTSUPP;
}

static inline int brcm_pmap_num_pstates(unsigned int core_id,
					unsigned int *num_pstates)
{
	return -ENOTSUPP;
}

static inline int brcm_pmap_get_pstate(unsigned int core_id,
				       unsigned int *pstate)
{
	return -ENOTSUPP;
}

static inline int brcm_pmap_set_pstate(unsigned int core_id,
				       unsigned int pstate)
{
	return -ENOTSUPP;
}

static inline int brcm_pmap_get_pstate_freqs(unsigned int core_id,
					     uint32_t *freqs)
{
	return -ENOTSUPP;
}
#endif
#endif
