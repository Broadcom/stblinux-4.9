/*
 * Copyright © 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

#include "pm.h"
#include "pm_data-offsets.h"

	.text
	.align	3

#define AON_CTRL_REG		r8
#define DDR_PHY_STATUS_OFFSET	r9
#define NUM_DDR_MEMC		r10
#define DDR_MEMC_STRUCT		r11

/*
 * r0: AON_CTRL base address
 * r1: DDRY PHY PLL status offset
 * r2: number of DDR controllers
 * r3: first DDR controller structure element
 */
ENTRY(brcmstb_pm_do_s2)
	stmfd	sp!, {r4-r11, lr}
	mov	AON_CTRL_REG, r0
	mov	DDR_PHY_STATUS_OFFSET, r1
	mov	NUM_DDR_MEMC, r2
	mov	DDR_MEMC_STRUCT, r3

	/* Flush memory transactions */
	dsb

	/* Cache DDR_PHY_STATUS_OFFSET translation */
	ldr	r5, [DDR_MEMC_STRUCT, #PM_DATA_DDR_PHY_BASE]
	add	r5, DDR_PHY_STATUS_OFFSET
	ldr	r4, [r5]

	/* power down request */
	ldr	r0, =PM_S2_COMMAND
	ldr	r1, =0
	str	r1, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	r1, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	str	r0, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	r0, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]

	/* Wait for interrupt */
	wfi
	nop

	/* Bring MEMC back up */
1:	ldr	r4, [r5]
	ands	r4, #1
	beq	1b

	/* Power-up handshake */
	ldr	r0, =1
	str	r0, [AON_CTRL_REG, #AON_CTRL_HOST_MISC_CMDS]
	ldr	r0, [AON_CTRL_REG, #AON_CTRL_HOST_MISC_CMDS]

	ldr	r0, =0
	str	r0, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	r0, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]

	/* NUM_DDR_MEMC = 0 is a special case to indicate not to do this */
	ldr	r2, =0
	cmp	NUM_DDR_MEMC, r2
	beq	3f

	/* Pulse DDR SRPD exit (1 -> 0) for each MEMC */
	ldr	r2, =PM_DATA_DDR_CTRL		// offset of ddr_ctrl
2:	ldr	r0, [DDR_MEMC_STRUCT, r2]
	add	r0, #DDR_CTRL_SRPD_CONFIG
	ldr	r1, [r0]
	orr	r1, r1, #FORCE_SRPD_EXIT
	str	r1, [r0]
	dsb
	ldr	r1, [r0]
	bic	r1, r1, #FORCE_SRPD_EXIT
	str	r1, [r0]
	dsb
	add	r2, #PM_DATA_MEMC_SIZE		// sizeof(struct brcmstb_memc)
	subs	NUM_DDR_MEMC, NUM_DDR_MEMC, #1
	bne	2b

	/* Return to caller */
3:	ldr	r0, =0
	ldmfd	sp!, {r4-r11, pc}

	ENDPROC(brcmstb_pm_do_s2)

	/* Place literal pool here */
	.ltorg

ENTRY(brcmstb_pm_do_s2_sz)
	.word   . - brcmstb_pm_do_s2
