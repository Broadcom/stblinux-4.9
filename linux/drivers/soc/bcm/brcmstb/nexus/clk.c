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

/* These strings' positions must align with the defs in clk_api.h */
static const char * const sw_names_stb[] = {
	/* Software Clocks and/or Cores */
	/* [00..0f] */
	"sw_cpu_core", "sw_v3d", "sw_sysif", "sw_scb",
	"sw_hvd0", "sw_raaga0", "sw_vice0", "sw_vice0_pss",
	"sw_vice1", "sw_vice1_pss", "sw_xpt", "sw_m2mc0",
	"sw_m2mc1", "sw_mipmap0", "sw_tsx0", "sw_smartcard0",

	/* [10..1f] */
	"sw_smartcard1", "reserved", "sw_bne", "sw_asp",
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",

	/* [20..2f] */
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",

	/* Software Clocks ONLY */
	/* [30..3f] */
	"sw_aio", "sw_bvn", "sw_dvphr", "sw_dvpht",
	"sw_genet0", "sw_genetwol0", "sw_hvd0_cpu", "sw_itu656",
	"sw_mmm2mc0", "sw_pcie0", "sw_pcie1", "sw_potp",
	"sw_raaga0_cpu", "sw_sata3", "sw_sdio0", "sw_sdio1",

	/* [40..4f] */
	"sw_sid", "sw_v3d_cpu", "sw_vec", "sw_xpt_wakeup",
	"sw_tsio", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",

	/* [50..5f] */
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",
	"reserved", "reserved", "reserved", "reserved",

	/* [60..6f] */
	"sw_aio_sram", "sw_bvn_sram", "sw_dvphr_sram", "sw_hvd0_sram",
	"sw_m2mc0_sram", "sw_m2mc1_sram", "sw_mmm2mc0_sram", "sw_raaga0_sram",
	"sw_v3d_sram", "sw_vec_sram", "sw_vice0_sram", "sw_vice1_sram",
	"sw_xpt_sram",
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
