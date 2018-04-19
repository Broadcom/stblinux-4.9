/*
 * Copyright (c) 2018 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * A copy of the GPL is available at
 * http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 * Foundation at https://www.gnu.org/licenses/ .
 */

#ifndef _BRCMSTB_AVS_DVFS_H
#define _BRCMSTB_AVS_DVFS_H

#include <linux/types.h>
struct platform_device;

/* AVS Commands */
#define AVS_CMD_AVAILABLE	0x00
#define AVS_CMD_DISABLE		0x10
#define AVS_CMD_ENABLE		0x11
#define AVS_CMD_S2_ENTER	0x12
#define AVS_CMD_S2_EXIT		0x13
#define AVS_CMD_BBM_ENTER	0x14
#define AVS_CMD_BBM_EXIT	0x15
#define AVS_CMD_S3_ENTER	0x16
#define AVS_CMD_S3_EXIT		0x17
#define AVS_CMD_BALANCE		0x18
/* PMAP and P-STATE commands */
#define AVS_CMD_GET_PMAP	0x30
#define AVS_CMD_SET_PMAP	0x31
#define AVS_CMD_GET_PSTATE	0x40
#define AVS_CMD_SET_PSTATE	0x41
/* Read sensor/debug */
#define AVS_CMD_READ_SENSOR	0x50
#define AVS_CMD_READ_DEBUG	0x51

/* AVS function return status definitions */
#define AVS_STATUS_CLEAR	0x00
#define AVS_STATUS_SUCCESS	0xf0
#define AVS_STATUS_FAILURE	0xff
#define AVS_STATUS_INVALID	0xf1
#define AVS_STATUS_NO_SUPP	0xf2
#define AVS_STATUS_NO_MAP	0xf3
#define AVS_STATUS_MAP_SET	0xf4

#define AVS_MAX_PARAMS		0x0c

int brcmstb_issue_avs_command(struct platform_device *pdev, unsigned int cmd,
			      unsigned int num_in, unsigned int num_out,
			      u32 args[]);

#endif /* _BRCMSTB_AVS_DVFS_H */
