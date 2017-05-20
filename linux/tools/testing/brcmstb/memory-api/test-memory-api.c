#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/brcmstb/memory_api.h>
#include <linux/soc/brcmstb/brcmstb.h>

static struct brcmstb_memory bm;

static void test_kva_mem_map(struct brcmstb_range *range)
{
	size_t size = range->size / 2;
	void *addr;
	unsigned long *data, check;

	addr = brcmstb_memory_kva_map_phys(range->addr, size, true);
	if (!addr) {
		pr_err("failed to map %llu MiB at %#016llx\n",
		       size / SZ_1M, range->addr);
		return;
	}

	pr_info("%s: virt: %p, phys: 0x%lx\n",
		__func__, addr, virt_to_phys(addr));

	/* Now try to read from there as well */
	data = addr;
	*data = 0xdeadbeefUL;
	check = *data;
	if (check != 0xdeadbeefUL)
		pr_err("memory mismatch: %llu != %llu\n",
			check, 0xdeadbeefUL);

	brcmstb_memory_kva_unmap(addr);
}

static int __init test_init(void)
{
	struct brcmstb_named_range *nrange;
	struct brcmstb_range *range;
	int ret;
	int i, j;

	ret = brcmstb_memory_get(&bm);
	if (ret) {
		pr_err("could not get memory struct\n");
		return ret;
	}

	/* print ranges */
	pr_info("Range info:\n");
	for (i = 0; i < MAX_BRCMSTB_MEMC; ++i) {
		pr_info(" memc%d\n", i);
		for (j = 0; j < bm.memc[i].count; ++j) {
			if (j >= MAX_BRCMSTB_RANGE) {
				pr_warn("  Need to increase MAX_BRCMSTB_RANGE!\n");
				break;
			}
			range = &bm.memc[i].range[j];
			pr_info("  %llu MiB at %#016llx\n",
					range->size / SZ_1M, range->addr);
		}
	}

	pr_info("lowmem info:\n");
	for (i = 0; i < bm.lowmem.count; ++i) {
		if (i >= MAX_BRCMSTB_RANGE) {
			pr_warn(" Need to increase MAX_BRCMSTB_RANGE!\n");
			break;
		}
		range = &bm.lowmem.range[i];
		pr_info(" %llu MiB at %#016llx\n",
				range->size / SZ_1M, range->addr);
	}

	pr_info("bmem info:\n");
	for (i = 0; i < bm.bmem.count; ++i) {
		if (i >= MAX_BRCMSTB_RANGE) {
			pr_warn(" Need to increase MAX_BRCMSTB_RANGE!\n");
			break;
		}
		range = &bm.bmem.range[i];
		pr_info(" %llu MiB at %#016llx\n",
				range->size / SZ_1M, range->addr);
		test_kva_mem_map(range);
	}

	pr_info("cma info:\n");
	for (i = 0; i < bm.cma.count; ++i) {
		if (i >= MAX_BRCMSTB_RANGE) {
			pr_warn(" Need to increase MAX_BRCMSTB_RANGE!\n");
			break;
		}
		range = &bm.cma.range[i];
		pr_info(" %llu MiB at %#016llx\n",
				range->size / SZ_1M, range->addr);
	}

	pr_info("reserved info:\n");
	for (i = 0; i < bm.reserved.count; ++i) {
		if (i >= MAX_BRCMSTB_RESERVED_RANGE) {
			pr_warn(" Need to increase MAX_BRCMSTB_RESERVED_RANGE!\n");
			break;
		}
		range = &bm.reserved.range[i];
		nrange = &bm.reserved.range_name[i];
		pr_info(" %#016llx-%#016llx (%s)\n",
				range->addr, range->addr + range->size,
				nrange->name);
	}

	/* Test the obtention of the MEMC size */
	for (i = 0; i < MAX_BRCMSTB_MEMC; i++) {
		pr_info("MEMC%d size %llu MiB (%#016llx)\n",
			i, brcmstb_memory_memc_size(i) / SZ_1M,
			brcmstb_memory_memc_size(i));
	}

	return -EINVAL;
}

static void __exit test_exit(void)
{
	pr_info("Goodbye world\n");
}

module_init(test_init);
module_exit(test_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gregory Fong (Broadcom Corporation)");

