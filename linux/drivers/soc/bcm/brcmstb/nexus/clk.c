// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018 Broadcom */


#include <linux/brcmstb/brcmstb.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

struct brcm_clk_cfg_data {
	const char * const	*hw_names;
	const char * const	*sw_names;
	unsigned int		num_sw_clks;
	unsigned int		num_hw_clks;
};

struct brcm_clk_iface {
	/* clks -- contiguous array of Hw and SW clocks, in that order */
	struct clk			**clks;
	unsigned int			num_clks;
	struct mutex			lock;
	const struct brcm_clk_cfg_data	*cfg;
};

static struct brcm_clk_iface *iface;

static const char * const hw_names_7211[] = {
	/* Hardware Clocks */
	/* [ 0.. 4] */
	"plla", "pllb", "pllc", "plld", "pllh",
	/* [ 5.. 9] */
	"plla_core", "plla_per", "pllb_arm", "pllc_core0", "pllc_core1",
	/* [10..14] */
	"pllc_core2", "pllc_per", "plld_core", "plld_per", "pllh_rcal_prediv",
	/* [15..19] */
	"pllh_aux", "pllh_pix_prediv", "timer", "otp", "uart",
	/* [20..24] */
	"vpu", "v3d", "isp", "h264", "vec",
	/* [25..29] */
	"hsm", "sdram", "tsens", "emmc", "peri_image",
	/* [30..34] */
	"pwm", "pcm", "plla_dsi0", "plla_ccp2", "plld_dsi0",
	/* [35..39] */
	"plld_dsi1", "aveo", "dft", "gp0", "gp1",
	/* [40..44] */
	"gp2", "slim", "smi", "tec", "dpi",
	/* [45..49] */
	"cam0", "cam1", "dsi0e", "dsi1e", "plla_mor_core2",
	/* [50..54] */
	"plla_mor_core3", "pllc_mor_core3", "argon", "emmc2", "gisb",
	/* [55..59] */
	"altscb", "genet_250", "stb27", "stb54", "stb108",
	/* [60..64] */
	"pixel_bvb", "genet_125", "hvd_cpu_alt", "hvd_core_alt", "m2mc",
	/* [65..69] */
	"xpt", "usbxhci",

};

static const char * const sw_names_7211[] = {
	/* Software Clocks */
	/* [00..0f] */
	"sw_bvn", "sw_dvpht", "sw_genet0", "sw_genetwol0",
	"sw_hvd0", "sw_pcie0", "sw_potp", "sw_sata3",
	"sw_sdio0", "sw_sdio1", "sw_sid", "sw_v3d",
	"sw_vec", "sw_xpt", "invalid", 	"invalid",

	/* [10..1f] */
	"invalid", "invalid", "invalid", "invalid",
	"invalid", "invalid", "sw_m2mc0",
};

#define NUM_EXTRA_HW_CLKS_IN_7211B0	5
#define NUM_EXTRA_SW_CLKS_IN_7211B0	10

static const struct brcm_clk_cfg_data brcm7211a0_cfg = {
	.hw_names = hw_names_7211,
	.num_hw_clks = ARRAY_SIZE(hw_names_7211) - NUM_EXTRA_HW_CLKS_IN_7211B0,
	.sw_names = sw_names_7211,
	.num_sw_clks = ARRAY_SIZE(sw_names_7211) - NUM_EXTRA_SW_CLKS_IN_7211B0,
};

static const char * const sw_names_stb[] = {
	/* Software Clocks */
	/* [00..0f] */
	"sw_bvn", "sw_dvpht", "sw_genet0", "sw_genetwol0",
	"sw_hvd0", "sw_pcie0", "sw_potp", "sw_sata3",
	"sw_sdio0", "sw_sdio1", "sw_sid", "sw_v3d",
	"sw_vec", "sw_xpt", "sw_aio", "sw_aio_sram",

	/* [10..1f] */
	"sw_bvn_sram", "sw_dvphr", "sw_dvphr_sram", "sw_hvd0_cpu",
	"sw_hvd0_sram",	"sw_itu656", "sw_m2mc0", "sw_m2mc0_sram",
	"sw_m2mc1", "sw_m2mc1_sram", "sw_mmm2mc0", "sw_mmm2mc0_sram",
	"sw_raaga0", "sw_raaga0_cpu", "sw_raaga0_sram", "sw_smartcard0",

	/* [1f..2f] */
	"sw_smartcard1", "sw_v3d_cpu", "sw_v3d_sram", "sw_vec_sram",
	"sw_vice0", "sw_vice0_sram", "sw_vice1", "sw_vice1_sram",
	"sw_xpt_sram", "sw_xpt_wakeup",
};

static const struct brcm_clk_cfg_data brcm7211_cfg = {
	.hw_names = hw_names_7211,
	.num_hw_clks = ARRAY_SIZE(hw_names_7211),
	.sw_names = sw_names_7211,
	.num_sw_clks = ARRAY_SIZE(sw_names_7211),
};

static const struct brcm_clk_cfg_data brcmstb_cfg = {
	.hw_names = NULL,
	.num_hw_clks = 0,
	.sw_names = sw_names_stb,
	.num_sw_clks = ARRAY_SIZE(sw_names_stb),
};

static inline bool brcm_is_sw_clk(unsigned int clk_id)
{
	return (clk_id >= BCLK_SW_OFFSET) &&
		(clk_id < BCLK_SW_OFFSET + iface->cfg->num_sw_clks);
}

static inline bool brcm_is_hw_clk(unsigned int clk_id)
{
	return (clk_id >= BCLK_HW_OFFSET) &&
		(clk_id < BCLK_HW_OFFSET + iface->cfg->num_hw_clks);
}

static inline int brcm_get_clk_idx(unsigned int clk_id)
{
	int idx = -1;

	if (brcm_is_sw_clk(clk_id))
		idx = (clk_id - BCLK_SW_OFFSET) + iface->cfg->num_hw_clks;
	else if (brcm_is_hw_clk(clk_id))
		idx = clk_id - BCLK_HW_OFFSET;
	else
		pr_debug("brcmstb-clk: bad clk_id: 0x%x\n", clk_id);

	return idx;
}

static struct clk *brcm_find_clk(unsigned int clk_id)
{
	int idx = brcm_get_clk_idx(clk_id);

	return idx < 0 ? NULL : iface->clks[idx];
}

int brcm_clk_prepare_enable(unsigned int clk_id)
{
	int idx, ret = 0;
	struct clk *clk;

	idx = brcm_get_clk_idx(clk_id);
	if (idx < 0)
		return -EINVAL;

	mutex_lock(&iface->lock);
	clk = iface->clks[idx];
	if (!clk) {
		const char *name;

		if (idx < iface->cfg->num_hw_clks)
			name = iface->cfg->hw_names[clk_id - BCLK_HW_OFFSET];
		else
			name = iface->cfg->sw_names[clk_id - BCLK_SW_OFFSET];

		clk = clk_get(NULL, name);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			pr_debug("brcmstb-clk: clk_get fail; clk_id=0x%x(%s)\n",
			       clk_id, name);
		} else {
			iface->clks[idx] = clk;
		}
	}
	mutex_unlock(&iface->lock);

	if (ret)
		return ret;
	else
		return clk_prepare_enable(clk);
}
EXPORT_SYMBOL(brcm_clk_prepare_enable);

void brcm_clk_disable_unprepare(unsigned int clk_id)
{
	struct clk *clk;

	if (!iface)
		return;
	if (clk_id == BCLK_NULL)
		return;
	clk = brcm_find_clk(clk_id);
	if (clk)
		clk_disable_unprepare(clk);
}
EXPORT_SYMBOL(brcm_clk_disable_unprepare);

static int  __init brcm_clk_init(void)
{
	const struct brcm_clk_cfg_data *cfg;

	if (of_find_compatible_node(NULL, NULL, "brcm,bcm7211b0-cprman"))
		cfg = &brcm7211_cfg;
	else if (of_find_compatible_node(NULL, NULL, "brcm,bcm7211a0-cprman"))
		cfg = &brcm7211a0_cfg;
	else
		cfg = &brcmstb_cfg;

	iface = kzalloc(sizeof(struct brcm_clk_iface), GFP_KERNEL);
	if (!iface)
		return -ENOMEM;

	iface->cfg = cfg;
	iface->num_clks = iface->cfg->num_hw_clks + iface->cfg->num_sw_clks;
	iface->clks = kcalloc(iface->num_clks, sizeof(struct clk *),
			      GFP_KERNEL);
	if (!iface->clks) {
		kfree(iface);
		iface = NULL;
		return -ENOMEM;
	}
	mutex_init(&iface->lock);
	pr_info("brcmstb-clk: %u SW, %u HW\n", iface->cfg->num_sw_clks,
		iface->cfg->num_hw_clks);

	return 0;
}

static void __exit brcm_clk_exit(void)
{
	unsigned int i;

	mutex_lock(&iface->lock);
	for (i = 0; i < iface->num_clks; i++)
		if (iface->clks[i])
			clk_put(iface->clks[i]);
	mutex_unlock(&iface->lock);
	kfree(iface->clks);
	kfree(iface);
	iface = NULL;
}

module_init(brcm_clk_init);
module_exit(brcm_clk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom STB Clock Interface Driver");
MODULE_AUTHOR("Broadcom");
