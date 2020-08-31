/*
 * Broadcom Brahma-B53 CPU read-ahead cache management functions
 *
 * Copyright (C) 2016, Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>

#include <asm/cacheflush.h>

/* RAC register offsets, relative to the HIF_CPU_BIUCTRL register base */
#define A72_RAC_CONFIG0_REG		(0x08)
#define B53_RAC_CONFIG0_REG		(0x78)
#define  RACENPREF_MASK			(0x3)
#define  RACPREFINST_SHIFT		(0)
#define  RACENINST_SHIFT		(2)
#define  RACPREFDATA_SHIFT		(4)
#define  RACENDATA_SHIFT		(6)
#define  RAC_CPU_SHIFT			(8)
#define  RACCFG_MASK			(0xff)
#define A72_RAC_CONFIG1_REG		(0x0c)
#define B53_RAC_CONFIG1_REG		(0x7c)
#define  DPREF_LINE_2_SHIFT		24
#define  DPREF_LINE_2_MASK		0xff
#define A72_RAC_FLUSH_REG		(0x14)
#define B53_RAC_FLUSH_REG		(0x84)
#define  FLUSH_RAC			(1 << 0)

/* Bitmask to enable instruction and data prefetching with a 256-bytes stride,
 * prefetch next 256-byte line after 4 consecutive lines used
 */
#define RAC_DATA_INST_EN_MASK		(1 << RACPREFINST_SHIFT | \
					 RACENPREF_MASK << RACENINST_SHIFT | \
					 1 << RACPREFDATA_SHIFT | \
					 RACENPREF_MASK << RACENDATA_SHIFT)

static void __iomem *b53_rac_base;

enum b53_rac_reg_offset {
	RAC_CONFIG0_REG,
	RAC_CONFIG1_REG,
	RAC_FLUSH_REG
};

static u8 b53_rac_offsets[] = {
	[RAC_CONFIG0_REG] = B53_RAC_CONFIG0_REG,
	[RAC_CONFIG1_REG] = B53_RAC_CONFIG1_REG,
	[RAC_FLUSH_REG]	= B53_RAC_FLUSH_REG,
};

static u8 a72_rac_offsets[] = {
	[RAC_CONFIG0_REG] = A72_RAC_CONFIG0_REG,
	[RAC_CONFIG1_REG] = A72_RAC_CONFIG1_REG,
	[RAC_FLUSH_REG]	= A72_RAC_FLUSH_REG,
};

static u8 *rac_offsets;

/* The read-ahead cache present in the Brahma-B53 CPU is a special piece of
 * hardware after the integrated L2 cache of the B53 CPU complex whose purpose
 * is to prefetch instruction and/or data with a line size of either 64 bytes
 * or 256 bytes. The rationale is that the data-bus of the CPU interface is
 * optimized for 256-byte transactions, and enabling the read-ahead cache
 * provides a significant performance boost (typically twice the performance
 * for a memcpy benchmark application).
 *
 * The read-ahead cache is transparent for Virtual Address cache maintenance
 * operations: IC IVAU, DC IVAC, DC CVAC, DC CVAU and DC CIVAC.  So no special
 * handling is needed for the DMA API above and beyond what is included in the
 * arm64 implementation.
 *
 * In addition, since the Point of Unification is typically between L1 and L2
 * for the Brahma-B53 processor no special read-ahead cache handling is needed
 * for the IC IALLU and IC IALLUIS cache maintenance operations.
 *
 * However, it is not possible to specify the cache level (L3) for the cache
 * maintenance instructions operating by set/way to operate on the read-ahead
 * cache.  The read-ahead cache will maintain coherency when inner cache lines
 * are cleaned by set/way, but if it is necessary to invalidate inner cache
 * lines by set/way to maintain coherency with system masters operating on
 * shared memory that does not have hardware support for coherency, then it
 * will also be necessary to explicitly invalidate the read-ahead cache.
 */
void b53_rac_flush_all(void)
{
	if (b53_rac_base) {
		__raw_writel(FLUSH_RAC, b53_rac_base +
			     rac_offsets[RAC_FLUSH_REG]);
		dsb(osh);
	}
}

static void b53_rac_enable_all(void)
{
	unsigned int cpu;
	u32 enable = 0, pref_dist, shift;

	pref_dist = __raw_readl(b53_rac_base + rac_offsets[RAC_CONFIG1_REG]);
	for_each_possible_cpu(cpu) {
		shift = cpu * RAC_CPU_SHIFT + RACPREFDATA_SHIFT;
		enable |= RAC_DATA_INST_EN_MASK << (cpu * RAC_CPU_SHIFT);
		if (rac_offsets == a72_rac_offsets) {
			enable &= ~(RACENPREF_MASK << shift);
			enable |= 3 << shift;
			pref_dist |= 1 << (cpu + DPREF_LINE_2_SHIFT);
		}
	}
	__raw_writel(enable, b53_rac_base + rac_offsets[RAC_CONFIG0_REG]);
	__raw_writel(pref_dist, b53_rac_base + rac_offsets[RAC_CONFIG1_REG]);
}

static void b53_rac_resume(void)
{
	b53_rac_enable_all();
}

static struct syscore_ops b53_rac_syscore_ops = {
	.resume		= b53_rac_resume,
};

static int __init b53_rac_init(void)
{
	struct device_node *dn, *cpu_dn;
	int ret = 0;

	dn = of_find_compatible_node(NULL, NULL, "brcm,brcmstb-cpu-biu-ctrl");
	if (!dn)
		return -ENODEV;

	if (WARN(num_possible_cpus() > 4, "RAC only supports 4 CPUs\n"))
		goto out;

	b53_rac_base = of_iomap(dn, 0);
	if (!b53_rac_base) {
		pr_err("failed to remap BIU control base\n");
		ret = -ENOMEM;
		goto out;
	}

	cpu_dn = of_get_cpu_node(0, NULL);
	if (!cpu_dn) {
		ret = -ENODEV;
		goto out_unmap;
	}

	if (of_device_is_compatible(cpu_dn, "brcm,brahma-b53"))
		rac_offsets = b53_rac_offsets;
	else if (of_device_is_compatible(cpu_dn, "arm,cortex-a72"))
		rac_offsets = a72_rac_offsets;
	else {
		pr_err("Unsupported CPU\n");
		of_node_put(cpu_dn);
		ret = -EINVAL;
		goto out_unmap;
	}
	of_node_put(cpu_dn);

	if (IS_ENABLED(CONFIG_PM_SLEEP))
		register_syscore_ops(&b53_rac_syscore_ops);

	b53_rac_enable_all();

	pr_info("%s: Broadcom %s read-ahead cache\n",
		dn->full_name, rac_offsets == b53_rac_offsets ?
		"Brahma-B53" : "Cortex-A72");

	goto out;

out_unmap:
	iounmap(b53_rac_base);
	b53_rac_base = NULL;
out:
	of_node_put(dn);
	return ret;
}
arch_initcall(b53_rac_init);
