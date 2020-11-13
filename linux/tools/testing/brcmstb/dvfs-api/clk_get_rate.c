// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2020 Broadcom */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/brcmstb/brcmstb.h>

/*
 * Stand-alone build example:
 *
 * $ cat Makefile
 * obj-m += xable_clock.o
 * $ make ARCH=arm64 CROSS_COMPILE=aarch64-linux- \
 *	-C $LIBNUX_DIR SUBDIRS=$PWD modules
 */

static int clk_get_rate_init(void)
{
	u64 rate;
	int ret = brcm_clk_get_rate(BCLK_SW_SCB, &rate);

	pr_info("Ret=%d    Freq=%llu\n", ret, rate);

	return 0;
}


static void clk_get_rate_exit(void)
{
}

module_init(clk_get_rate_init);
module_exit(clk_get_rate_exit);
MODULE_LICENSE("GPL");
