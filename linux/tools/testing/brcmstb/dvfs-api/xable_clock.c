#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/brcmstb/brcmstb.h>
#include <linux/delay.h>

/*
 * USAGE:
 *   insmod xable_clock.ko  name=<clock> [enable=<int>]
 *
 * EXAMPLES:
 *
 *   insmod xable_clock.ko name=sw_xpt
 *   insmod xable_clock.ko name=sw_xpt enable=1
 *   insmod xable_clock.ko name=sw_xpt enable=0
 *
 * NOTE:
 *   The insmod will always fail to do the insmod operation but
 *   should execut the clock dis/enable -- this is by design so the
 *   user does not have to keep running an rmmod for every insmod.
 */


/*
 * Stand-alone build example:
 *
 * $ cat Makefile
 * obj-m += xable_clock.o
 * $ make ARCH=arm64 CROSS_COMPILE=aarch64-linux- \
 *	-C /work3/jquinlan/git/49-arm64/linux SUBDIRS=$PWD modules
 */


static int enable = 1;
module_param(enable, int, 0660);

static char *name = "invalid";
module_param(name, charp, 0660);

static int xable_clock_init(void)
{
	struct clk *clk;
	int ret;

	if (!strcmp(name, "invalid")) {
		pr_info("USAGE:\n  insmod xable_clock.ko name=<clk> [enable=<int>]\n");
		pr_info("EXAMPLES:\n");
		pr_info("\tinsmod xable_clock.ko name=sw_xpt /* Enable is implied */\n");
		pr_info("\tinsmod xable_clock.ko name=sw_xpt enable=1\n");
		pr_info("\tinsmod xable_clock.ko name=sw_xpt enable=0\n");
		pr_info("\tinsmod xable_clock.ko name=sw_xpt enable=0\n");
		pr_info("NOTE:\n  By design, the insmod will fail.\n\n");
		return -EIO;
	}


	clk = clk_get(NULL, name);
	if (IS_ERR(clk)) {
		pr_info("Error: could not find clock %s!\n", name);
		return -EIO;
	}
	ret = 0;
	if (!!enable)
		ret = clk_prepare_enable(clk);
	else
		clk_disable_unprepare(clk);

	pr_info("%s to %s clock %s\n\n", ret ? "FAILED" : "Succeeded",
		enable ? "enable" : "disable", name);

	return -EIO;
}


static void xable_clock_exit(void)
{
}

module_init(xable_clock_init);
module_exit(xable_clock_exit);
MODULE_LICENSE("GPL");
