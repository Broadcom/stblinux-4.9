/*
 * Support for Broadcom STB reg save for power management
 *
 * Copyright (C) 2014 Broadcom Corporation
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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <linux/soc/brcmstb/aon_defs.h>

struct brcmstb_reg_group {
	void __iomem *regs;
	unsigned int count;
};

/*
 * For now, one set of register offsets is sufficient for all SoCs that need
 * this code. In the future, it might become necessary to have different
 * offsets for different SoCs.
 */
unsigned int dtu_cfg_offs[] = {
	0x38,
	0x40,
	0x48,
	0x50,
	0xc0,
	0xc8,
};

static const unsigned int dtu_cfg_count = ARRAY_SIZE(dtu_cfg_offs);

struct regsave_data {
	/* For reg_save */
	struct brcmstb_reg_group *reg_groups;
	unsigned num_reg_groups;
	u32 *reg_mem;

	/* For dtu_save */
	void __iomem **dtu_map;
	void __iomem **dtu_config;
	resource_size_t *map_size;
	unsigned num_dtu_entries;
	u32 **dtu_map_mem;
	u32 **dtu_config_mem;
};

static struct regsave_data priv;

int dtu_save(void)
{
	unsigned i, j;

	for (i = 0; i < priv.num_dtu_entries; i++) {
		for (j = 0; j < priv.map_size[i] / sizeof(u32); j++) {
			priv.dtu_map_mem[i][j] =
				__raw_readl(priv.dtu_map[i] + j * sizeof(u32));
		}
	}

	if (!priv.dtu_config)
		return 0;

	for (i = 0; i < priv.num_dtu_entries; i++) {
		for (j = 0; j < dtu_cfg_count; j++) {
			priv.dtu_config_mem[i][j] =
				__raw_readl(priv.dtu_config[i] +
					    dtu_cfg_offs[j]);
		}
	}

	return 0;
}

int brcmstb_dtusave_init(struct brcmstb_bootloader_dtu_table *tbl)
{
	unsigned int num_maps, num_configs, i;
	struct device_node *dn, *prev;
	struct resource res;
	int ret = 0;

	/* Count dtu-map nodes */
	for (prev = NULL, num_maps = 0; ; prev = dn, num_maps++) {
		dn = of_find_compatible_node(prev, NULL,
					     "brcm,brcmstb-memc-dtu-map");
		if (!dn)
			break;
	}

	/* No dtu-map nodes means we don't need DTU support. */
	if (num_maps == 0)
		return 0;

	/* Count dtu-config nodes */
	for (prev = NULL, num_configs = 0; ; prev = dn, num_configs++) {
		dn = of_find_compatible_node(prev, NULL,
					     "brcm,brcmstb-memc-dtu-config");
		if (!dn)
			break;
	}

	/*
	 * Having no dtu-config nodes is allowed. But if they exist, there must
	 * be the same number of dtu-config nodes as there are dtu-map nodes.
	 */
	if (num_maps != num_configs && num_configs > 0) {
		pr_err("%s: MISMATCH: you have %u DTU map%s and %u DTU "
			"config%s\n", __func__, num_maps,
			(num_maps != 1) ? "s" : "", num_configs,
			(num_configs != 1) ? "s" : "");
		ret = -EINVAL;
		goto out;

	}

	priv.num_dtu_entries = num_maps;

	priv.dtu_map = kmalloc(num_maps * sizeof(*priv.dtu_map), GFP_KERNEL);
	priv.map_size = kmalloc(num_maps * sizeof(*priv.map_size), GFP_KERNEL);
	priv.dtu_map_mem = kmalloc(num_maps * sizeof(*priv.dtu_map_mem),
				   GFP_KERNEL);

	if (!priv.dtu_map || !priv.map_size || !priv.dtu_map_mem) {
		if (priv.dtu_map)
			kfree(priv.dtu_map);
		if (priv.map_size)
			kfree(priv.map_size);
		if (priv.dtu_map_mem)
			kfree(priv.dtu_map_mem);
		ret = -ENOMEM;
		goto out;
	}

	if (num_configs > 0) {
		priv.dtu_config = kmalloc(
					num_configs * sizeof(*priv.dtu_config),
					GFP_KERNEL);
		priv.dtu_config_mem = kmalloc(num_configs *
					sizeof(*priv.dtu_config_mem),
					GFP_KERNEL);
		if (!priv.dtu_config || !priv.dtu_config_mem) {
			if (priv.dtu_config)
				kfree(priv.dtu_config);
			if (priv.dtu_config_mem)
				kfree(priv.dtu_config_mem);
			ret = -ENOMEM;
			goto free_map_mem;
		}
	}

	for (prev = NULL, i = 0; i < num_maps; prev = dn, i++) {
		dn = of_find_compatible_node(prev, NULL,
					     "brcm,brcmstb-memc-dtu-map");
		priv.dtu_map[i] = of_iomap(dn, 0);
		if (!priv.dtu_map[i]) {
			ret = -EIO;
			goto free_config_mem;
		}
		if (of_address_to_resource(dn, 0 , &res)) {
			ret = -EIO;
			goto free_config_mem;
		}

		priv.map_size[i] = resource_size(&res);
		priv.dtu_map_mem[i] = tbl[i].dtu_state_map;
	}
	of_node_put(dn);

	for (prev = NULL, i = 0; i < num_configs; prev = dn, i++) {
		resource_size_t size;
		unsigned int last_offset = dtu_cfg_offs[dtu_cfg_count - 1];
		unsigned int min_size = last_offset + sizeof(u32);

		dn = of_find_compatible_node(prev, NULL,
				     "brcm,brcmstb-memc-dtu-config");
		priv.dtu_config[i] = of_iomap(dn, 0);
		if (!priv.dtu_config[i]) {
			ret = -EIO;
			goto free_config_mem;
		}
		if (of_address_to_resource(dn, 0 , &res)) {
			ret = -EIO;
			goto free_config_mem;
		}
		size = resource_size(&res);
		if (size < min_size) {
			pr_err("%s: dtu-config area must be at least %u bytes "
				"(is only %pa bytes)\n", __func__, min_size,
				&res);
		}
		priv.dtu_config_mem[i] = tbl[i].dtu_config;
	}

	goto out;

free_config_mem:
	kfree(priv.dtu_config);
	kfree(priv.dtu_config_mem);

free_map_mem:
	kfree(priv.dtu_map);
	kfree(priv.map_size);
	kfree(priv.dtu_map_mem);

out:
	if (likely(dn))
		of_node_put(dn);

	return ret;
}

static int reg_save(void)
{
	int i;
	unsigned int j, total;

	if (priv.num_reg_groups == 0)
		return 0;

	for (i = 0, total = 0; i < priv.num_reg_groups; i++) {
		struct brcmstb_reg_group *p = &priv.reg_groups[i];
		for (j = 0; j < p->count; j++)
			priv.reg_mem[total++] = __raw_readl(p->regs + (j * 4));
	}
	return 0;
}

static void reg_restore(void)
{
	int i;
	unsigned int j, total;

	if (priv.num_reg_groups == 0)
		return;

	for (i = 0, total = 0; i < priv.num_reg_groups; i++) {
		struct brcmstb_reg_group *p = &priv.reg_groups[i];
		for (j = 0; j < p->count; j++)
			__raw_writel(priv.reg_mem[total++], p->regs + (j * 4));
	}
}

static struct syscore_ops regsave_pm_ops = {
	.suspend        = reg_save,
	.resume         = reg_restore,
};

int brcmstb_regsave_init(void)
{
	struct resource res;
	struct device_node *dn, *pp;
	int ret = 0, len, num_phandles = 0, i;
	unsigned int total;
	resource_size_t size;
	const char *name;

	if (priv.reg_groups)
		return 0;

	dn = of_find_node_by_name(NULL, "s3");
	if (!dn)
		/* FIXME: return -EINVAL when all bolts have 's3' node */
		goto fail;

	if (of_get_property(dn, "syscon-refs", &len))
		name = "syscon-refs";
	else if (of_get_property(dn, "regsave-refs", &len))
		name = "regsave-refs";
	else
		/* FIXME: return -EINVAL when all bolts have 'syscon-refs' */
		goto fail;

	num_phandles = len / 4;
	priv.reg_groups =
		kzalloc(num_phandles * sizeof(struct brcmstb_reg_group),
			GFP_KERNEL);
	if (priv.reg_groups == NULL) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < num_phandles; i++) {
		void __iomem *regs;

		pp = of_parse_phandle(dn, name, i);
		if (!pp) {
			ret = -EIO;
			goto fail;
		}
		ret = of_address_to_resource(pp, 0, &res);
		if (ret)
			goto fail;
		size = resource_size(&res);
		regs = ioremap(res.start, size);
		if (!regs) {
			ret = -EIO;
			goto fail;
		}

		priv.reg_groups[priv.num_reg_groups].regs = regs;
		priv.reg_groups[priv.num_reg_groups].count =
			(size / sizeof(u32));
		priv.num_reg_groups++;
	};

	for (i = 0, total = 0; i < priv.num_reg_groups; i++)
		total += priv.reg_groups[i].count;
	priv.reg_mem = kmalloc(total * sizeof(u32), GFP_KERNEL);
	if (!priv.reg_mem) {
		ret = -ENOMEM;
		goto fail;
	}
	of_node_put(dn);
	register_syscore_ops(&regsave_pm_ops);
	return 0;
fail:
	if (priv.reg_groups) {
		for (i = 0; i < num_phandles; i++)
			if (priv.reg_groups[i].regs)
				iounmap(priv.reg_groups[i].regs);
			else
				break;
		kfree(priv.reg_groups);
	}
	kfree(priv.reg_mem);
	of_node_put(dn);
	return ret;
}
