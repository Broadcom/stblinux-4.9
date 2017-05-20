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
	struct brcmstb_reg_group *reg_groups;
	u32 *dtu_map_mem;
	u32 *dtu_config_mem;
	u32 *reg_mem;
	void __iomem *dtu_config;
	void __iomem *dtu_map;
	resource_size_t config_size;
	resource_size_t map_size;
	int num_reg_groups;
};

static struct regsave_data priv;

int dtu_save(void)
{
	int i;

	for (i = 0; i < priv.map_size / sizeof(u32); i++)
		priv.dtu_map_mem[i] =
			__raw_readl(priv.dtu_map + i * sizeof(u32));

	if (priv.dtu_config) {
		for (i = 0; i < dtu_cfg_count; i++)
			priv.dtu_config_mem[i] =
				__raw_readl(priv.dtu_config +
					    dtu_cfg_offs[i]);
	}

	return 0;
}

int brcmstb_dtusave_init(u32 *map_buffer, u32 *config_buffer)
{
	struct device_node *dn;
	struct resource res;
	int ret = 0;

	dn = of_find_compatible_node(NULL, NULL, "brcm,brcmstb-memc-dtu-map");
	if (!dn)
		return 0;

	priv.dtu_map = of_iomap(dn, 0);
	if (!priv.dtu_map) {
		ret = -EIO;
		goto out;
	}
	if (of_address_to_resource(dn, 0 , &res)) {
		ret = -EIO;
		goto out;
	}
	of_node_put(dn);

	priv.map_size = resource_size(&res);
	priv.dtu_map_mem = map_buffer;

	dn = of_find_compatible_node(NULL, NULL,
				     "brcm,brcmstb-memc-dtu-config");
	if (dn) {
		priv.dtu_config = of_iomap(dn, 0);
		if (!priv.dtu_config) {
			ret = -EIO;
			goto out;
		}
		if (of_address_to_resource(dn, 0 , &res)) {
			ret = -EIO;
			goto out;
		}
		priv.config_size = resource_size(&res);
		priv.dtu_config_mem = config_buffer;
	}

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
