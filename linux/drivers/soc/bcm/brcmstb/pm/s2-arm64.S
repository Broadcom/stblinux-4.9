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

	.text
	.align	3

#define AON_CTRL_REG		x0
#define DDR_PHY_STATUS_REG	x1

/*
 * x0: AON_CTRL base address
 * x1: DDRY PHY PLL status register address
 */
ENTRY(brcmstb_pm_do_s2)
	/* Save registers we are using */
	stp	x2, x3, [sp, #-16]

	/* Flush memory transactions */
	dsb	sy

	/* Cache DDR_PHY_STATUS_REG translation */
	ldr	w2, [DDR_PHY_STATUS_REG]

	/* power down request */
	ldr	w3, =PM_S2_COMMAND

	/* acess registers using 32-bit width but from a 64-bit address */
	str	wzr, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	w2, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	str	w3, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	w3, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]

	/* Wait for interrupt */
	wfi
	nop

	/* Bring MEMC back up */
1:	ldr	w2, [DDR_PHY_STATUS_REG]
	ands	w2, w2, #1
	beq	1b

	/* Power-up handshake */
	ldr	w2, =1
	str	w2, [AON_CTRL_REG, #AON_CTRL_HOST_MISC_CMDS]
	ldr	w2, [AON_CTRL_REG, #AON_CTRL_HOST_MISC_CMDS]

	str	wzr, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]
	ldr	w2, [AON_CTRL_REG, #AON_CTRL_PM_CTRL]

	/* Restore registers we used */
	ldp	x2, x3, [sp, #16]

	/* Return to caller */
	mov	x0, xzr
	ret

	ENDPROC(brcmstb_pm_do_s2)

	/* Place literal pool here */
	.ltorg

ENTRY(brcmstb_pm_do_s2_sz)
	.word   . - brcmstb_pm_do_s2
