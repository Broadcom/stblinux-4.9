/*
 * Copyright (C) 2009-2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#define pr_fmt(fmt) "clk-brcmstb: " fmt

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/syscore_ops.h>

static bool shut_off_unused_clks = true;
static int bcm_full_clk = 2;
static void __iomem *cpu_clk_div_reg;

/* Gate clock structs */
struct bcm_clk_gate {
	struct clk_hw hw;
	void __iomem *reg;
	u8 bit_idx;
	u8 flags;
	u32 delay[2];
	spinlock_t *lock;
	struct clk_ops ops;
};

/* Multiplier clock structs and data */
struct brcmstb_clk_mult_field_desc {
	u8 offset;
	u8 width;
	u8 src_shift;
	u8 dst_shift;
};

struct brcmstb_clk_mult_info {
	unsigned int type;
	unsigned int frac_width;
	unsigned int int_width;
	unsigned int nudge_width;
	const struct brcmstb_clk_mult_field_desc *nudge_fields;
	const struct brcmstb_clk_mult_field_desc *multval_fields;
};

struct bcm_clk_mult {
	struct clk_hw hw;
	void __iomem *reg;
#define MULT_CLK_FLAGS_NUDGE_BIT	BIT(0)
	unsigned long flags;
	const struct brcmstb_clk_mult_info *info;
	spinlock_t *lock;
};

const struct brcmstb_clk_mult_field_desc
brcm_clk_mult_fields_multval_type0[] = {
	{ .offset = 0,
	  .width = 10,
	  .src_shift = 0,
	  .dst_shift = 20, },
	{ .offset = 0,
	  .width = 4,
	  .src_shift = 10,
	  .dst_shift = 0, },
	{ .offset = 4,
	  .width = 16,
	  .src_shift = 0,
	  .dst_shift = 4, },
	{ /* terminating null element */ },
};

const struct brcmstb_clk_mult_field_desc brcm_clk_mult_fields_nudge_type0[] = {
	{ .offset = 8,
	  .width = 12,
	  .src_shift = 0,
	  .dst_shift = 0, },
	{ /* terminating null element */ },
};

static const struct brcmstb_clk_mult_info brcmstb_clk_mult_dbase[] = {
	/* Type 0 brcm mult clock */
	{ .type = 0,
	  .frac_width = 20,
	  .int_width = 8,
	  .nudge_width = 12,
	  .multval_fields = brcm_clk_mult_fields_multval_type0,
	  .nudge_fields = brcm_clk_mult_fields_nudge_type0,
	},
};


/* SW clock structs */
struct bcm_clk_sw {
	struct clk_hw hw;
	struct clk_ops ops;
};

#define to_brcmstb_clk_gate(p) container_of(p, struct bcm_clk_gate, hw)
#define to_brcmstb_clk_mult(p) container_of(p, struct bcm_clk_mult, hw)
#define to_brcmstb_clk_sw(p) container_of(p, struct bcm_clk_sw, hw)
#define to_clk_mux(_hw) container_of(_hw, struct clk_mux, hw)

static DEFINE_SPINLOCK(lock);

static int cpu_clk_div_pos __initdata;
static int cpu_clk_div_width __initdata;

#ifdef CONFIG_PM_SLEEP
static u32 cpu_clk_div_reg_dump;

static int brcmstb_clk_suspend(void)
{
	if (cpu_clk_div_reg)
		cpu_clk_div_reg_dump = __raw_readl(cpu_clk_div_reg);
	return 0;
}

static void brcmstb_clk_resume(void)
{
	if (cpu_clk_div_reg)
		__raw_writel(cpu_clk_div_reg_dump, cpu_clk_div_reg);
}

static struct syscore_ops brcmstb_clk_syscore_ops = {
	.suspend = brcmstb_clk_suspend,
	.resume = brcmstb_clk_resume,
};
#endif /* CONFIG_PM_SLEEP */

static int __init parse_cpu_clk_div_dimensions(struct device_node *np)
{
	struct property *prop;
	const __be32 *p = NULL;
	int len;
	int elem_cnt;
	const char *propname = "div-shift-width";

	prop = of_find_property(np, propname, &len);
	if (!prop) {
		pr_err("%s property undefined\n", propname);
		return -EINVAL;
	}

	elem_cnt = len / sizeof(u32);

	if (elem_cnt != 2) {
		pr_err("%s should have only 2 elements\n", propname);
		return -EINVAL;
	}

	p = of_prop_next_u32(prop, p, &cpu_clk_div_pos);
	of_prop_next_u32(prop, p, &cpu_clk_div_width);

	return 0;
}

static struct clk_div_table *cpu_clk_div_table;

static int __init parse_cpu_clk_div_table(struct device_node *np)
{
	struct property *prop;
	const __be32 *p = NULL;
	struct clk_div_table *cur_tbl_ptr;
	int len;
	int elem_cnt;
	int i;
	const char *propname = "div-table";

	prop = of_find_property(np, propname, &len);
	if (!prop) {
		pr_err("%s property undefined\n", propname);
		return -EINVAL;
	}

	elem_cnt = len / sizeof(u32);

	if (elem_cnt < 2) {
		pr_err("%s should have at least 2 elements\n", propname);
		return -EINVAL;
	}

	if ((elem_cnt % 2) != 0) {
		pr_err("%s should have even number of elements\n", propname);
		return -EINVAL;
	}

	/* need room for last sentinel entry */
	len += 2 * sizeof(u32);

	cpu_clk_div_table = kmalloc(len, GFP_KERNEL);
	if (!cpu_clk_div_table)
		return -ENOMEM;

	cur_tbl_ptr = cpu_clk_div_table;

	for (i = 0; i < elem_cnt; i += 2) {
		p = of_prop_next_u32(prop, p, &cur_tbl_ptr->val);
		p = of_prop_next_u32(prop, p, &cur_tbl_ptr->div);

		cur_tbl_ptr++;
	}

	/* last entry should be zeroed out */
	cur_tbl_ptr->val = 0;
	cur_tbl_ptr->div = 0;

	return 0;
}

static void __init of_brcmstb_cpu_clk_div_setup(struct device_node *np)
{
	struct clk *clk;
	int rc;

	cpu_clk_div_reg = of_iomap(np, 0);
	if (!cpu_clk_div_reg) {
		pr_err("unable to iomap cpu clk divider register!\n");
		return;
	}

	rc = parse_cpu_clk_div_dimensions(np);
	if (rc)
		goto err;

	rc = parse_cpu_clk_div_table(np);
	if (rc)
		goto err;

	clk = clk_register_divider_table(NULL, "cpu-clk-div",
					 of_clk_get_parent_name(np, 0), 0,
					 cpu_clk_div_reg,
					 cpu_clk_div_pos, cpu_clk_div_width,
					 0, cpu_clk_div_table, &lock);
	if (IS_ERR(clk))
		goto err;

	rc = of_clk_add_provider(np, of_clk_src_simple_get, clk);
	if (rc) {
		pr_err("error adding clock provider (%d)\n", rc);
		goto err;
	}

#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&brcmstb_clk_syscore_ops);
#endif
	return;

err:
	kfree(cpu_clk_div_table);
	cpu_clk_div_table = NULL;

	if (cpu_clk_div_reg) {
		iounmap(cpu_clk_div_reg);
		cpu_clk_div_reg = NULL;
	}
}
CLK_OF_DECLARE_DRIVER(brcmstb_cpu_clk_div, "brcm,brcmstb-cpu-clk-div",
		of_brcmstb_cpu_clk_div_setup);

/*
 * It works on following logic:
 *
 * For enabling clock, enable = 1
 *	set2dis = 1	-> clear bit	-> set = 0
 *	set2dis = 0	-> set bit	-> set = 1
 *
 * For disabling clock, enable = 0
 *	set2dis = 1	-> set bit	-> set = 1
 *	set2dis = 0	-> clear bit	-> set = 0
 *
 * So, result is always: enable xor set2dis.
 */
static void brcmstb_clk_gate_endisable(struct clk_hw *hw, int enable)
{
	struct bcm_clk_gate *gate = to_brcmstb_clk_gate(hw);
	int set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	unsigned long flags = 0;
	u32 reg;

	set ^= enable;

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);

	reg = readl(gate->reg);

	if (set)
		reg |= BIT(gate->bit_idx);
	else
		reg &= ~BIT(gate->bit_idx);

	writel(reg, gate->reg);

	if (set == 0 && gate->delay[0])
		udelay(gate->delay[0]);
	else if (set == 1 && gate->delay[1])
		udelay(gate->delay[1]);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
}

static int brcmstb_clk_gate_enable(struct clk_hw *hw)
{
	brcmstb_clk_gate_endisable(hw, 1);
	return 0;
}

static void brcmstb_clk_gate_disable(struct clk_hw *hw)
{
	brcmstb_clk_gate_endisable(hw, 0);
}

static int brcmstb_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct bcm_clk_gate *gate = to_brcmstb_clk_gate(hw);

	reg = readl(gate->reg);

	/* if a set bit disables this clk, flip it before masking */
	if (gate->flags & CLK_GATE_SET_TO_DISABLE)
		reg ^= BIT(gate->bit_idx);

	reg &= BIT(gate->bit_idx);
	return reg ? 1 : 0;
}

static u8 brcmstb_clk_get_parent(struct clk_hw *hw)
{
	return 0;
}

static u64 brcmstb_clk_mult_combine_fields(struct bcm_clk_mult *mult,
			   const struct brcmstb_clk_mult_field_desc *f)
{
	u64 val = 0;

	while (f->width) {
		u64 tmp = readl(mult->reg + f->offset);

		tmp >>= f->src_shift;
		tmp &= (1 << f->width) - 1;
		val |= tmp << f->dst_shift;
		f++;
	}
	return val;
}

static void brcmstb_clk_mult_get_mult_val(struct clk_hw *hw, u64 *int_part,
					  u64 *frac_part)
{
	struct bcm_clk_mult *mult = to_brcmstb_clk_mult(hw);
	const struct brcmstb_clk_mult_info *info = mult->info;
	const struct brcmstb_clk_mult_field_desc *f
		= mult->info->multval_fields;
	const u64 val = brcmstb_clk_mult_combine_fields(mult, f);

	*frac_part = val & GENMASK_ULL(info->frac_width - 1, 0);
	*int_part = val >> info->frac_width;
}

static u64 brcmstb_clk_mult_get_nudge_val(struct bcm_clk_mult *mult)
{
	const struct brcmstb_clk_mult_field_desc *f = mult->info->nudge_fields;

	if (f->width == 0 || !(mult->flags & MULT_CLK_FLAGS_NUDGE_BIT))
		return 0;
	return brcmstb_clk_mult_combine_fields(mult, f);
}

static unsigned long brcmstb_clk_mult_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct bcm_clk_mult *mult = to_brcmstb_clk_mult(hw);
	const struct brcmstb_clk_mult_info *info = mult->info;
	u64 multval, nudge, t0, t1, frac_part, int_part;
	int i, extra;

	if (parent_rate == 0)
		return 0;

	brcmstb_clk_mult_get_mult_val(hw, &int_part, &frac_part);
	nudge = brcmstb_clk_mult_get_nudge_val(mult);
	i = fls64((u64) parent_rate);
	/* Extra bits for precision which may be needed */
	extra = 64 - (info->frac_width + info->int_width + i + 1);

	/* t0 = 2^frac_part_width - nudge */
	t0 = (1 << info->frac_width) - nudge;

	/* frac_part /= t0 */
	t1 = frac_part << (info->frac_width + extra);
	do_div(t1, (u32)t0);

	/* multval = concat(int_part,frac_part) */
	multval = (int_part << (info->frac_width + extra)) | t1;
	return parent_rate * multval >> (info->frac_width + extra);
}

static long brcmstb_clk_mult_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	/*
	 * We currently do not implement setting this clock so the
	 * only rate it can ever have is the one it currently has.
	 */
	return brcmstb_clk_mult_recalc_rate(hw, *parent_rate);
}

static int brcmstb_clk_mult_set_rate(struct clk_hw *hw,
				      unsigned long rate,
				      unsigned long parent_rate)
{
	/*
	 * We currently do not implement setting this clock so the
	 * only rate it can ever have is the one it currently has.
	 */
	return brcmstb_clk_mult_recalc_rate(hw, parent_rate);
}

static const struct clk_ops brcmstb_clk_gate_ops = {
	.enable = brcmstb_clk_gate_enable,
	.disable = brcmstb_clk_gate_disable,
	.is_enabled = brcmstb_clk_gate_is_enabled,
};

static const struct clk_ops brcmstb_clk_mult_ops = {
	.round_rate = brcmstb_clk_mult_round_rate,
	.recalc_rate = brcmstb_clk_mult_recalc_rate,
	.set_rate = brcmstb_clk_mult_set_rate,
};

static const struct clk_ops brcmstb_clk_gate_inhib_dis_ops = {
	.enable = brcmstb_clk_gate_enable,
	.is_enabled = brcmstb_clk_gate_is_enabled,
};

static const struct clk_ops brcmstb_clk_gate_ro_ops = {
	.is_enabled = brcmstb_clk_gate_is_enabled,
};

/**
 * brcm_clk_mult_register - register a bcm mult clock with the clock framework.
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of this clock's parent
 * @flags: framework-specific flags for this clock
 * @reg: register address to control gating of this clock
 * @type: type of bcm mult clock
 * @lock: shared register lock for this clock
 */
static struct clk __init *brcm_clk_mult_register(
	struct device *dev, const char *name, const char *parent_name,
	unsigned long flags, void __iomem *reg, unsigned long clk_mult_flags,
	unsigned int mult_type, spinlock_t *lock)

{
	const struct brcmstb_clk_mult_info *p = brcmstb_clk_mult_dbase;
	struct bcm_clk_mult *mult;
	struct clk *clk;
	struct clk_init_data init;
	int i;

	/* allocate the mult */
	mult = kzalloc(sizeof(struct bcm_clk_mult), GFP_KERNEL);
	if (!mult)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &brcmstb_clk_mult_ops;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	init.flags = flags;
	if (!shut_off_unused_clks)
		init.flags |= CLK_IGNORE_UNUSED; /* FIXME */

	/* struct bcm_mult assignments */
	mult->reg = reg;
	mult->flags = clk_mult_flags;
	for (i = 0; i < ARRAY_SIZE(brcmstb_clk_mult_dbase); i++)
		if (p[i].type == mult_type) {
			mult->info = p;
			break;
		}
	if (!mult->info)
		return ERR_PTR(-EINVAL);

	mult->lock = lock;
	mult->hw.init = &init;

	clk = clk_register(dev, &mult->hw);

	if (IS_ERR(clk))
		kfree(mult);

	return clk;
}

/**
 * brcm_clk_gate_register - register a bcm gate clock with the clock framework.
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of this clock's parent
 * @flags: framework-specific flags for this clock
 * @reg: register address to control gating of this clock
 * @bit_idx: which bit in the register controls gating of this clock
 * @clk_gate_flags: gate-specific flags for this clock
 * @delay: usec delay in turning on, off.
 * @lock: shared register lock for this clock
 */
static struct clk __init *brcm_clk_gate_register(
	struct device *dev, const char *name, const char *parent_name,
	unsigned long flags, void __iomem *reg, u8 bit_idx,
	u8 clk_gate_flags, u32 delay[2], spinlock_t *lock,
	bool read_only, bool inhibit_disable)
{
	struct bcm_clk_gate *gate;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the gate */
	gate = kzalloc(sizeof(struct bcm_clk_gate), GFP_KERNEL);
	if (!gate) {
		pr_err("%s: could not allocate bcm gated clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = read_only ? &brcmstb_clk_gate_ro_ops : inhibit_disable ?
		&brcmstb_clk_gate_inhib_dis_ops : &brcmstb_clk_gate_ops;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	init.flags = flags;
	if (!shut_off_unused_clks)
		init.flags |= CLK_IGNORE_UNUSED; /* FIXME */

	/* struct bcm_gate assignments */
	gate->reg = reg;
	gate->bit_idx = bit_idx;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->delay[0] = delay[0];
	gate->delay[1] = delay[1];
	gate->hw.init = &init;

	clk = clk_register(dev, &gate->hw);

	if (IS_ERR(clk))
		kfree(gate);

	return clk;
}

/*
 * get_parent won't get called for SW Clocks, but add a dummy function
 * to work around a sanity check that wants any clock with multiple
 * parents to have get_parent.
 */
static const struct clk_ops brcmstb_clk_sw_ops = {
	.get_parent = brcmstb_clk_get_parent,
};

/**
 * brcmstb_clk_sw_register - register a bcm gate clock with the clock framework.
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parents: name of this clock's parents; not known by clock framework
 * @num_parents: number of parents
 * @flags: framework-specific flags for this clock
 * @lock: shared register lock for this clock
 */
static struct clk __init *brcmstb_clk_sw_register(
	struct device *dev, const char *name, const char **parent_names,
	int num_parents, unsigned long flags, spinlock_t *lock)
{
	struct bcm_clk_sw *sw_clk;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the gate */
	sw_clk = kzalloc(sizeof(struct bcm_clk_sw), GFP_KERNEL);
	if (!sw_clk) {
		pr_err("%s: could not allocate bcm sw clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &brcmstb_clk_sw_ops;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	init.flags = flags | CLK_IS_BASIC | CLK_IS_SW;
	if (!shut_off_unused_clks)
		init.flags |= CLK_IGNORE_UNUSED; /* FIXME */

	sw_clk->hw.init = &init;
	clk = clk_register(dev, &sw_clk->hw);
	if (IS_ERR(clk))
		kfree(sw_clk);
	return clk;
}

/**
 * of_brcmstb_gate_clk_setup() - Setup function for brcmstb gate clock
 */
static void __init of_brcmstb_clk_gate_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *reg;
	const char *parent_name;
	u8 clk_gate_flags = 0;
	u32 bit_idx = 0;
	u32 delay[2] = {0, 0};
	int ret;
	bool read_only = false;
	bool inhibit_disable = false;
	unsigned long flags = 0;

	of_property_read_string(node, "clock-output-names", &clk_name);
	parent_name = of_clk_get_parent_name(node, 0);
	if (of_property_read_u32(node, "bit-shift", &bit_idx)) {
		pr_err("%s: missing bit-shift property for %s\n",
				__func__, node->name);
		return;
	}
	reg = of_iomap(node, 0);
	if (!reg) {
		pr_err("unable to iomap cpu clk divider register!\n");
		return;
	}

	of_property_read_u32_array(node, "brcm,delay", delay, 2);

	if (of_property_read_bool(node, "set-bit-to-disable"))
		clk_gate_flags |= CLK_GATE_SET_TO_DISABLE;

	if (of_property_read_bool(node, "brcm,read-only"))
		read_only = true;

	if (of_property_read_bool(node, "brcm,inhibit-disable"))
		inhibit_disable = true;

	if (of_property_read_bool(node, "brcm,set-rate-parent"))
		flags |= CLK_SET_RATE_PARENT;

	clk = brcm_clk_gate_register(NULL, clk_name, parent_name, flags, reg,
				     (u8) bit_idx, clk_gate_flags, delay,
				     &lock, read_only, inhibit_disable);
	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		ret = clk_register_clkdev(clk, clk_name, NULL);
		if (ret)
			pr_err("%s: clk device registration failed for '%s'\n",
			       __func__, clk_name);
	}
}
CLK_OF_DECLARE(brcmstb_clk_gate, "brcm,brcmstb-gate-clk",
		of_brcmstb_clk_gate_setup);
CLK_OF_DECLARE(brcmstb_clk_gate_7211, "brcm,brcm7211-gate-clk",
		of_brcmstb_clk_gate_setup);

/**
 * of_brcmstb_mult_clk_setup() - Setup function for brcmstb mult clock
 */
static void __init of_brcmstb_clk_mult_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *reg;
	const char *parent_name;
	u8 clk_mult_flags = 0;
	int ret;
	u32 mult_type = 0;
	unsigned long flags = 0;

	reg = of_iomap(node, 0);
	if (!reg)
		return;

	parent_name = of_clk_get_parent_name(node, 0);
	if (of_property_read_bool(node, "brcm,has-nudge"))
		clk_mult_flags |= MULT_CLK_FLAGS_NUDGE_BIT;
	of_property_read_u32(node, "brcm,mult-type", &mult_type);

	clk = brcm_clk_mult_register(NULL, clk_name, parent_name, flags, reg,
				     clk_mult_flags, mult_type, &lock);

	if (IS_ERR(clk)) {
		iounmap(reg);
	} else {
		ret = of_clk_add_provider(node, of_clk_src_simple_get, clk);
		if (ret == 0)
			ret = clk_register_clkdev(clk, clk_name, NULL);
		if (ret)
			pr_err("%s: clk device registration failed for '%s'\n",
			       __func__, clk_name);
	}
}
CLK_OF_DECLARE(brcmstb_clk_mult, "brcm,brcmstb-mult-clk",
		of_brcmstb_clk_mult_setup);

static void __init of_brcmstb_clk_sw_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	int num_parents;
	const char **parent_names;
	int ret, i;

	of_property_read_string(node, "clock-output-names", &clk_name);
	num_parents = of_property_count_strings(node, "clock-names");
	if (num_parents < 1) {
		pr_err("%s: brcm-sw-clock %s must have parent(s)\n",
				__func__, node->name);
		return;
	}
	parent_names = kzalloc((sizeof(char *) * num_parents),
			GFP_KERNEL);
	if (!parent_names) {
		pr_err("%s: failed to alloc parent_names\n", __func__);
		return;
	}

	for (i = 0; i < num_parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	clk = brcmstb_clk_sw_register(NULL, clk_name, parent_names, num_parents,
				   0, NULL);
	kfree(parent_names);

	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		ret = clk_register_clkdev(clk, clk_name, NULL);
		if (ret)
			pr_err("%s: clk device registration failed for '%s'\n",
			       __func__, clk_name);
	}
}
CLK_OF_DECLARE(brcmstb_clk_sw, "brcm,brcmstb-sw-clk",
		of_brcmstb_clk_sw_setup);

static struct clk_ops clk_mux_ops_brcm = {
	.determine_rate = __clk_mux_determine_rate_closest,
};

static struct clk *clk_register_mux_table_brcm(struct device *dev,
		const char *name, const char **parent_names, u8 num_parents,
		unsigned long flags, void __iomem *reg, u8 shift, u32 mask,
		u8 clk_mux_flags, u32 *table, spinlock_t *lock)
{
	struct clk_mux *mux;
	struct clk *clk;
	struct clk_init_data init;
	u8 width = 0;

	if (clk_mux_ops_brcm.get_parent == NULL) {
		/* we would like to set these at compile time but
		 * that is not possible */
		clk_mux_ops_brcm.get_parent = clk_mux_ops.get_parent;
		clk_mux_ops_brcm.set_parent = clk_mux_ops.set_parent;
	}

	if (clk_mux_flags & CLK_MUX_HIWORD_MASK) {
		width = fls(mask) - ffs(mask) + 1;
		if (width + shift > 16) {
			pr_err("mux value exceeds LOWORD field\n");
			return ERR_PTR(-EINVAL);
		}
	}

	/* allocate the mux */
	mux = kzalloc(sizeof(struct clk_mux), GFP_KERNEL);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	if (clk_mux_flags & CLK_MUX_READ_ONLY)
		init.ops = &clk_mux_ro_ops;
	else
		init.ops = &clk_mux_ops_brcm;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	/* struct clk_mux assignments */
	mux->reg = reg;
	mux->shift = shift;
	mux->mask = mask;
	mux->flags = clk_mux_flags;
	mux->lock = lock;
	mux->table = table;
	mux->hw.init = &init;

	clk = clk_register(dev, &mux->hw);

	if (IS_ERR(clk))
		kfree(mux);

	return clk;
}


/**
 * of_mux_clk_setup_brcm() - Setup function for simple mux rate clock
 */
void of_mux_clk_setup_brcm(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *reg;
	int num_parents;
	const char **parent_names;
	int i;
	u8 clk_mux_flags = 0;
	u32 mask = 0;
	u32 shift = 0;

	of_property_read_string(node, "clock-output-names", &clk_name);

	num_parents = of_clk_get_parent_count(node);
	if (num_parents < 1) {
		pr_err("%s: mux-clock %s must have parent(s)\n",
				__func__, node->name);
		return;
	}

	parent_names = kzalloc((sizeof(char *) * num_parents),
			GFP_KERNEL);

	if (!parent_names) {
		pr_err("%s: could not allocate parent names\n", __func__);
		return;
	}

	for (i = 0; i < num_parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	reg = of_iomap(node, 0);
	if (!reg) {
		pr_err("%s: no memory mapped for property reg\n", __func__);
		goto fail;
	}

	if (of_property_read_u32(node, "bit-mask", &mask)) {
		pr_err("%s: missing bit-mask property for %s\n",
		       __func__, node->name);
		goto fail;
	}

	if (of_property_read_u32(node, "bit-shift", &shift)) {
		shift = __ffs(mask);
		pr_debug("%s: bit-shift property defaults to 0x%x for %s\n",
				__func__, shift, node->name);
	}

	if (of_property_read_bool(node, "index-starts-at-one"))
		clk_mux_flags |= CLK_MUX_INDEX_ONE;

	if (of_property_read_bool(node, "hiword-mask"))
		clk_mux_flags |= CLK_MUX_HIWORD_MASK;

	clk = clk_register_mux_table_brcm(NULL, clk_name, parent_names,
			num_parents, 0, reg, shift,
			mask, clk_mux_flags, NULL, NULL);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return;
fail:
	kfree(parent_names);
}
EXPORT_SYMBOL_GPL(of_mux_clk_setup_brcm);
CLK_OF_DECLARE(mux_clk, "brcm,mux-clock", of_mux_clk_setup_brcm);


static int __init _bcm_full_clk(char *str)
{
	get_option(&str, &bcm_full_clk);
	shut_off_unused_clks = bcm_full_clk > 1;
	return 0;
}

early_param("bcm_full_clk", _bcm_full_clk);
