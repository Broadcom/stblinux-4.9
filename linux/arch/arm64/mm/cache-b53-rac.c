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

#include <asm/cacheflush.h>

/* RAC register offsets, relative to the HIF_CPU_BIUCTRL register base */
#define RAC_CONFIG0_REG			(0x78)
#define  RACENPREF_MASK			(0x3)
#define  RACPREFINST_SHIFT		(0)
#define  RACENINST_SHIFT		(2)
#define  RACPREFDATA_SHIFT		(4)
#define  RACENDATA_SHIFT		(6)
#define  RAC_CPU_SHIFT			(8)
#define  RACCFG_MASK			(0xff)
#define B53_RAC_FLUSH_REG		(0x84)
#define  FLUSH_RAC			(1 << 0)

/* Bitmask to enable instruction and data prefetching with a 256-bytes stride */
#define RAC_DATA_INST_EN_MASK		(1 << RACPREFINST_SHIFT | \
					 RACENPREF_MASK << RACENINST_SHIFT | \
					 1 << RACPREFDATA_SHIFT | \
					 RACENPREF_MASK << RACENDATA_SHIFT)

static void __iomem *b53_rac_base;

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
		__raw_writel(FLUSH_RAC, b53_rac_base + B53_RAC_FLUSH_REG);
		dsb(osh);
	}
}

static void b53_rac_enable_all(void)
{
	unsigned int cpu;
	u32 enable = 0;

	for_each_possible_cpu(cpu) {

		enable |= RAC_DATA_INST_EN_MASK << (cpu * RAC_CPU_SHIFT);
	}
	__raw_writel(enable, b53_rac_base + RAC_CONFIG0_REG);
}

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

	if (!of_device_is_compatible(cpu_dn, "brcm,brahma-b53")) {
		pr_err("Unsupported CPU\n");
		of_node_put(cpu_dn);
		ret = -EINVAL;
		goto out_unmap;
	}
	of_node_put(cpu_dn);

	b53_rac_enable_all();

	pr_info("Broadcom Brahma-B53 read-ahead cache at: 0x%p\n",
		b53_rac_base + RAC_CONFIG0_REG);

	goto out;

out_unmap:
	iounmap(b53_rac_base);
	b53_rac_base = NULL;
out:
	of_node_put(dn);
	return ret;
}
arch_initcall(b53_rac_init);
