/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2018 Broadcom */
#ifndef _ASM_BRCMSTB_CLK_API_H
#define _ASM_BRCMSTB_CLK_API_H

#define BCLK_NULL		0xffffffff
#define BCLK_HW_OFFSET		0x0
#define BCLK_SW_OFFSET		0x5000

#define BCLK_SW_BVN		(BCLK_SW_OFFSET + 0x0)
#define BCLK_SW_DVPHT		(BCLK_SW_OFFSET + 0x1)
#define BCLK_SW_GENET0		(BCLK_SW_OFFSET + 0x2)
#define BCLK_SW_GENETWOL0	(BCLK_SW_OFFSET + 0x3)
#define BCLK_SW_HVD0		(BCLK_SW_OFFSET + 0x4)
#define BCLK_SW_PCIE0		(BCLK_SW_OFFSET + 0x5)
#define BCLK_SW_POTP		(BCLK_SW_OFFSET + 0x6)
#define BCLK_SW_SATA3		(BCLK_SW_OFFSET + 0x7)
#define BCLK_SW_SDIO0		(BCLK_SW_OFFSET + 0x8)
#define BCLK_SW_SDIO1		(BCLK_SW_OFFSET + 0x9)
#define BCLK_SW_SID		(BCLK_SW_OFFSET + 0xa)
#define BCLK_SW_V3D		(BCLK_SW_OFFSET + 0xb)
#define BCLK_SW_VEC		(BCLK_SW_OFFSET + 0xc)
#define BCLK_SW_XPT		(BCLK_SW_OFFSET + 0xd)

#define BCLK_SW_AIO		(BCLK_SW_OFFSET + 0xe)
#define BCLK_SW_AIO_SRAM	(BCLK_SW_OFFSET + 0xf)
#define BCLK_SW_BVN_SRAM	(BCLK_SW_OFFSET + 0x10)
#define BCLK_SW_DVPHR		(BCLK_SW_OFFSET + 0x11)
#define BCLK_SW_DVPHR_SRAM	(BCLK_SW_OFFSET + 0x12)
#define BCLK_SW_HVD0_CPU	(BCLK_SW_OFFSET + 0x13)
#define BCLK_SW_HVD0_SRAM	(BCLK_SW_OFFSET + 0x14)
#define BCLK_SW_ITU656		(BCLK_SW_OFFSET + 0x15)
#define BCLK_SW_M2MC0		(BCLK_SW_OFFSET + 0x16)
#define BCLK_SW_M2MC0_SRAM	(BCLK_SW_OFFSET + 0x17)
#define BCLK_SW_M2MC1		(BCLK_SW_OFFSET + 0x18)
#define BCLK_SW_M2MC1_SRAM	(BCLK_SW_OFFSET + 0x19)
#define BCLK_SW_MMM2MC0		(BCLK_SW_OFFSET + 0x1a)
#define BCLK_SW_MMM2MC0_SRAM	(BCLK_SW_OFFSET + 0x1b)
#define BCLK_SW_RAAGA0		(BCLK_SW_OFFSET + 0x1c)
#define BCLK_SW_RAAGA0_CPU	(BCLK_SW_OFFSET + 0x1d)
#define BCLK_SW_RAAGA0_SRAM	(BCLK_SW_OFFSET + 0x1e)
#define BCLK_SW_SMARTCARD0	(BCLK_SW_OFFSET + 0x1f)
#define BCLK_SW_SMARTCARD1	(BCLK_SW_OFFSET + 0x20)
#define BCLK_SW_V3D_CPU		(BCLK_SW_OFFSET + 0x21)
#define BCLK_SW_V3D_SRAM	(BCLK_SW_OFFSET + 0x22)
#define BCLK_SW_VEC_SRAM	(BCLK_SW_OFFSET + 0x23)
#define BCLK_SW_VICE0		(BCLK_SW_OFFSET + 0x24)
#define BCLK_SW_VICE0_SRAM	(BCLK_SW_OFFSET + 0x25)
#define BCLK_SW_VICE1		(BCLK_SW_OFFSET + 0x26)
#define BCLK_SW_VICE1_SRAM	(BCLK_SW_OFFSET + 0x27)
#define BCLK_SW_XPT_SRAM	(BCLK_SW_OFFSET + 0x28)
#define BCLK_SW_XPT_WAKEUP	(BCLK_SW_OFFSET + 0x29)

#ifdef CONFIG_BRCMSTB_NEXUS_CLK_API
int brcm_clk_prepare_enable(unsigned int clk_id);
void  brcm_clk_disable_unprepare(unsigned int clk_id);
#else
static inline int brcm_clk_prepare_enable(unsigned int clk_id)
{
	return -ENOTSUPP;
}
static inline void  brcm_clk_disable_unprepare(unsigned int clk_id)
{}
#endif
#endif
