/*
 * Copyright © 2016 Broadcom
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
#include <asm/asm-offsets.h>

#include "pm.h"
#include "pm_data-offsets.h"

	.text
	.align	3

#define AON_CTRL_REG		x0
#define DDR_PHY_STATUS_OFFSET	x1
#define NUM_DDR_MEMC		x2
#define DDR_MEMC_STRUCT		x3

/*
 * x0: AON_CTRL base address
 * x1: DDRY PHY PLL status offset
 * x2: number of DDR controllers
 * x3: first DDR controller structure element
 *
 * w4: contains the cached translation of the first memory controller's
 *     DDR PHY PLL status register offset
 */
ENTRY(brcmstb_pm_do_s2)
	/* Flush memory transactions */
	dsb	sy

	/* Cache DDR_PHY_STATUS_OFFSET translation */
	ldr	x5, [DDR_MEMC_STRUCT, #PM_DATA_DDR_PHY_BASE]
	add	x5, x5, DDR_PHY_STATUS_OFFSET
	ldr	w4, [x5]

	/* power down request */
	ldr	w7, =PM_S2_COMMAND

	/* acess registers using 32-bit width but from a 64-bit address */
	str	wzr, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	w6, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	str	w7, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	w7, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]

	/* Wait for interrupt */
	wfi
	nop

	/* Bring MEMC back up */
1:	ldr	w4, [x5]
	ands	w4, w4, #1
	beq	1b

	/* Power-up handshake */
	ldr	w6, =1
	str	w6, [AON_CTRL_REG, #AON_CTRL_HOST_MISC_CMDS]
	ldr	w6, [AON_CTRL_REG, #AON_CTRL_HOST_MISC_CMDS]

	str	wzr, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	w6, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]

	/* NUM_DDR_MEMC = 0 is a special case to indicate not to do this */
	cmp	NUM_DDR_MEMC, xzr
	beq	3f

	/* Pulse DDR SRPD exit for each MEMC */
	ldr	x7, =PM_DATA_DDR_CTRL		// offset of ddr_ctrl
2:	ldr	x5, [DDR_MEMC_STRUCT, x7]
	add	x5, x5, #DDR_CTRL_SRPD_CONFIG
	ldr	w6, [x5]
	orr	w6, w6, #FORCE_SRPD_EXIT
	str	w6, [x5]
	dsb	sy
	ldr	w6, [x5]
	bic	w6, w6, #FORCE_SRPD_EXIT
	str	w6, [x5]
	dsb	sy
	add	x7, x7, #PM_DATA_MEMC_SIZE	// sizeof(struct brcmstb_memc)
	subs	NUM_DDR_MEMC, NUM_DDR_MEMC, #1
	bne	2b

	/* Return to caller */
3:	mov	x0, xzr
	ret

	ENDPROC(brcmstb_pm_do_s2)

	/* Place literal pool here */
	.ltorg

ENTRY(brcmstb_pm_do_s2_sz)
	.word   . - brcmstb_pm_do_s2
