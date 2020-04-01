/*
 * Copyright © 2019 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * A copy of the GPL is available at
 * http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 * Foundation at https://www.gnu.org/licenses/ .
 */

#include <linux/bitmap.h>
#include <linux/mm.h>   /* for alloc_contig_range */
#include <linux/brcmstb/bhpa.h>
#include <linux/brcmstb/memory_api.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/page-isolation.h>
#include <linux/seq_file.h> /* seq_print single_open */
#include <linux/slab.h> /* kmalloc */

#define BHPA_ORDER	(21 - PAGE_SHIFT)
/*  size of single HUGE page, in bytes */
#define BHPA_SIZE	(PAGE_SIZE << BHPA_ORDER)

#if pageblock_order > BHPA_ORDER
#define BHPA_ALIGN	(1 << (pageblock_order + PAGE_SHIFT))
#else
#define BHPA_ALIGN	BHPA_SIZE
#endif

/*  size of single HUGE page allocation block, in bytes */
#define BHPA_BLOCK_MAX (1 * (uint64_t)1024 * 1024 * 1024)
/*  number of HUGE pages in single HUGE page allocation block */
#define BHPA_BLOCK_PAGES ((unsigned int)(BHPA_BLOCK_MAX / BHPA_SIZE))

#define MAX_BHPA_REGIONS	8

#define B_LOG_WRN(args...) pr_warn(args)
#define B_LOG_MSG(args...) pr_info(args)
#define B_LOG_DBG(args...) pr_debug(args)
#define B_LOG_TRACE(args...) do { if (0) pr_debug(args); } while (0)

struct bhpa_region {
	phys_addr_t		addr;
	phys_addr_t		size;
	int			memc;
};

/* struct bhpa_block - BHPA block structure
 * @node: list head node
 * @base: start address of this block
 * @count: number of pages in the block
 * @free: number of non-busy pages in the block
 * @stats: statistics structure
 * @busy: bitmap of busy pages - allocate or not available
 * @allocated: bitmap of allocate pages
 */
struct bhpa_block {
	struct list_head node;
	phys_addr_t base;
	unsigned int count;
	unsigned int free;
	struct {
		/* number of busy pages */
		unsigned int busy;
		/* number of allocated pages */
		unsigned int allocated;
		/* largest number of allocated pages */
		unsigned int high_allocated;
		/* largest number of 'busy' pages */
		unsigned int high_busy;
		struct {
			unsigned int fast[4];
			unsigned int slow;
		} allocations;
	} stats;
	DECLARE_BITMAP(busy, BHPA_BLOCK_PAGES);
	DECLARE_BITMAP(allocated, BHPA_BLOCK_PAGES);
};

struct bhpa_memc {
	struct list_head blocks;
};

struct bhpa_allocator {
	struct bhpa_memc memc[MAX_BRCMSTB_MEMC];
	struct dentry *debugfs;
};

static struct bhpa_region bhpa_regions[MAX_BHPA_REGIONS];
static unsigned int n_bhpa_regions;
static DEFINE_MUTEX(bhpa_lock);
struct bhpa_allocator bhpa_allocator;

static int __init __bhpa_setup(phys_addr_t addr, phys_addr_t size)
{
	phys_addr_t end = addr + size;
	unsigned long movablebase;
	int i;

	/* Consolidate overlapping regions */
	for (i = 0; i < n_bhpa_regions; i++) {
		if (addr > bhpa_regions[i].addr + bhpa_regions[i].size)
			continue;
		if (end < bhpa_regions[i].addr)
			continue;
		end = max(end, bhpa_regions[i].addr + bhpa_regions[i].size);
		addr = min(addr, bhpa_regions[i].addr);
		bhpa_regions[i].addr = bhpa_regions[n_bhpa_regions].addr;
		bhpa_regions[i--].size = bhpa_regions[n_bhpa_regions--].size;
	}

	if (n_bhpa_regions == MAX_BHPA_REGIONS) {
		pr_warn_once("too many regions, ignoring extras\n");
		return -E2BIG;
	}

	bhpa_regions[n_bhpa_regions].addr = addr;
	bhpa_regions[n_bhpa_regions].size = end - addr;
	n_bhpa_regions++;

	movablebase = __phys_to_pfn(addr);
	if (movablebase && (!movable_start || movable_start > movablebase))
		movable_start = movablebase;

	return 0;
}

/*
 * Parses command line for bhpa= options
 */
static int __init bhpa_setup(char *str)
{
	phys_addr_t addr = 0, end = 0, size;
	char *orig_str = str;
	int ret;

	addr = memparse(str, &str);
	if (*str == '@') {
		size = addr;
		addr = memparse(str + 1, &str);
		end = addr + size;
	} else if (*str == '-') {
		end = memparse(str + 1, &str);
	}

	addr = ALIGN(addr, BHPA_ALIGN);
	end = ALIGN_DOWN(end, BHPA_ALIGN);
	size = end - addr;

	if (addr > end || size < pageblock_nr_pages << PAGE_SHIFT) {
		pr_warn("ignoring invalid range '%s' (too small)\n",
				orig_str);
		return 0;
	}

	ret = __bhpa_setup(addr, size);
	if (!ret)
		brcmstb_memory_override_defaults = true;
	return ret;
}
early_param("bhpa", bhpa_setup);

static __init void split_bhpa_region(phys_addr_t addr, struct bhpa_region *p)
{
	struct bhpa_region *tmp;
	phys_addr_t end;

	if (p->addr + p->size > addr + BHPA_SIZE) {
		if (n_bhpa_regions < MAX_BHPA_REGIONS) {
			tmp = &bhpa_regions[n_bhpa_regions++];
			while (tmp > p) {
				*tmp = *(tmp - 1);
				tmp--;
			}
			(++tmp)->addr = addr;
			tmp->size -= addr - p->addr;
			end = addr + tmp->size;
			B_LOG_DBG("region split: 0x%pa-0x%pa", &addr, &end);
		} else {
			B_LOG_WRN("bhpa region truncated (MAX_BHPA_REGIONS)");
		}
	}
	p->size = addr - p->addr;
	end = p->addr + p->size;
	B_LOG_DBG("region added: 0x%pa-0x%pa", &p->addr, &end);
}

static __init void intersect_bhpa_ranges(phys_addr_t start, phys_addr_t size,
					 struct bhpa_region **ptr)
{
	struct bhpa_region *tmp, *p = *ptr;
	phys_addr_t end = start + size;

	B_LOG_DBG("range: 0x%pa-0x%pa", &start, &end);
	while (p < &bhpa_regions[n_bhpa_regions] &&
	       p->addr + p->size <= start) {
		tmp = p;
		end = p->addr + p->size;
		B_LOG_WRN("unmapped bhpa region 0x%pa-0x%pa",
			   &p->addr, &end);

		n_bhpa_regions--;
		while (tmp < &bhpa_regions[n_bhpa_regions]) {
			*tmp = *(tmp + 1);
			tmp++;
		}
	}

	end = start + size;
	while (p < &bhpa_regions[n_bhpa_regions] && p->addr < end) {
		phys_addr_t last;

		start = max(start, p->addr);
		start = ALIGN(start, BHPA_ALIGN);
		last = min(end, p->addr + p->size);
		last = ALIGN_DOWN(last, BHPA_ALIGN);

		if (start + BHPA_ALIGN >= last) {
			*ptr = p;
			return;
		}

		B_LOG_DBG("intersection: 0x%pa-0x%pa", &start, &last);
		p->size -= start - p->addr;
		p->addr = start;

		split_bhpa_region(last, p);
		p++;
	}

	*ptr = p;
}

static __init int memc_map(int memc, u64 addr, u64 size, void *context)
{
	struct bhpa_region **ptr = context, *p;
	phys_addr_t start = (phys_addr_t)addr;

	if (start != addr) {
		pr_err("phys_addr_t smaller than provided address 0x%llx!\n",
			addr);
		return -EINVAL;
	}

	if (memc == -1) {
		pr_err("address 0x%llx does not appear to be in any memc\n",
			addr);
		return -EINVAL;
	}

	p = *ptr;
	intersect_bhpa_ranges(start, (phys_addr_t)size, ptr);

	while (p != *ptr) {
		p->memc = memc;
		p++;
	}

	return 0;
}

static void __init bhpa_alloc_ranges(void)
{
	struct bhpa_region *p = bhpa_regions;
	phys_addr_t end;

	while (p < &bhpa_regions[n_bhpa_regions]) {
		end = p->addr + p->size;
		/*
		 * This is based on memblock_alloc_range_nid(), but excludes
		 * the search for efficiency.
		 */
		if (!memblock_reserve(p->addr, p->size)) {
			B_LOG_MSG("Alloc: MEMC%d: 0x%pa-0x%pa", p->memc,
				&p->addr, &end);
			/*
			 * The min_count is set to 0 so that memblock
			 * allocations are never reported as leaks.
			 */
			kmemleak_alloc_phys(p->addr, p->size, 0, 0);
			p++;
		} else {
			B_LOG_WRN("bhpa reservation 0x%pa-0x%pa failed!",
				&p->addr, &end);
			while (++p < &bhpa_regions[n_bhpa_regions])
				*(p - 1) = *p;
			n_bhpa_regions--;
		}
	}
}

void __init brcmstb_bhpa_reserve(void)
{
	phys_addr_t addr, size, start, end;
	struct bhpa_region *p, *tmp;
	u64 loop;
	int i;

	if (!movable_start)
		return;

	if (!n_bhpa_regions) {
		/* Try to grab all available memory above movable_start */
		bhpa_regions[0].addr = __pfn_to_phys(movable_start);
		size = memblock_end_of_DRAM() - bhpa_regions[0].addr;
		bhpa_regions[0].size = size;
		n_bhpa_regions = 1;
	}

	for (i = 0; i < n_bhpa_regions; i++) {
		bhpa_regions[i].memc = -1;
		if (!i)
			continue;

		/* Sort regions */
		p = &bhpa_regions[i];
		addr = p->addr;
		size = p->size;
		while (p != bhpa_regions && p->addr < (p - 1)->addr) {
			p->addr = (p - 1)->addr;
			p->size = (p - 1)->size;
			p--;
		}
		p->addr = addr;
		p->size = size;
	}
	for (i = 0; i < n_bhpa_regions; i++) {
		p = &bhpa_regions[i];
		end = p->addr + p->size;
		B_LOG_DBG("region: 0x%pa-0x%pa", &p->addr, &end);
	}

	p = bhpa_regions;
	early_for_each_memc_range(memc_map, &p);
	while (p < &bhpa_regions[n_bhpa_regions]) {
		tmp = &bhpa_regions[--n_bhpa_regions];
		end = tmp->addr + tmp->size;
		B_LOG_WRN("Drop region: 0x%pa-0x%pa", &tmp->addr, &end);
	}

	if (!n_bhpa_regions)
		return;

	p = bhpa_regions;
	for_each_free_mem_range(loop, NUMA_NO_NODE, MEMBLOCK_NONE, &start,
				&end, NULL) {
		intersect_bhpa_ranges(start, end - start, &p);

		if (p >= &bhpa_regions[n_bhpa_regions])
			break;
	}
	while (p < &bhpa_regions[n_bhpa_regions]) {
		tmp = &bhpa_regions[--n_bhpa_regions];
		end = tmp->addr + tmp->size;
		B_LOG_WRN("Drop region: 0x%pa-%pa", &tmp->addr, &end);
	}

	bhpa_alloc_ranges();
}

void __init brcmstb_bhpa_setup(phys_addr_t addr, phys_addr_t size)
{
	__bhpa_setup(addr, size);
}

/*
 * Returns index if the supplied range falls entirely within a bhpa region
 */
int bhpa_find_region(phys_addr_t addr, phys_addr_t size)
{
	int i;

	for (i = 0; i < n_bhpa_regions; i++) {
		if (addr < bhpa_regions[i].addr)
			return -ENOENT;

		if (addr + size <=
		    bhpa_regions[i].addr + bhpa_regions[i].size)
			return i;
	}
	return -ENOENT;
}

/*
 * Finds the IDX'th bhpa region, and fills in addr/size if it exists.
 * Returns 0 on success, <0 on failure.
 * Can pass in NULL for addr and/or size if you only care about return value.
 */
int bhpa_region_info(int idx, phys_addr_t *addr, phys_addr_t *size)
{
	if (idx >= n_bhpa_regions)
		return -ENOENT;

	if (addr)
		*addr = bhpa_regions[idx].addr;
	if (size)
		*size = bhpa_regions[idx].size;

	return 0;
}

static struct page *bhpa_get_free_range_in_zone(struct zone *zone,
						unsigned long start,
						unsigned long end,
						unsigned int migratetype,
						unsigned int order)
{
	struct page *free_page = NULL;

	if (!populated_zone(zone))
		return NULL;
	B_LOG_TRACE("free_range: zone:%p %s at %lx",
		    (void *)zone, zone->name, zone->zone_start_pfn);
	spin_lock(&zone->lock);
	for (; order < MAX_ORDER; order++) {
		struct free_area *area = &(zone->free_area[order]);
		struct page *page;

		B_LOG_TRACE("free_range: zone:%p area:%p order:%u migratetype:%u",
			    (void *)zone, (void *)area, order, migratetype);
		list_for_each_entry(page, &area->free_list[migratetype], lru) {
			unsigned long pfn;

			B_LOG_TRACE("free_range: zone:%p page:%p",
				    (void *)zone, (void *)page);
			pfn = page_to_pfn(page);
			B_LOG_TRACE("free_range: zone:%p page:%lx..%lx order:%u range:%lx..%lx",
				    (void *)zone, pfn, pfn + (1 << order),
				    order, start, end);
			if (pfn >= start && (pfn + (1 << order)) <= end) {
				free_page = page;
				break;
			}
		}
		if (free_page)
			break;
	}
	spin_unlock(&zone->lock);
	return free_page;
}

static struct page *bhpa_get_free_page_in_range(unsigned long start,
						unsigned long end,
						unsigned int migratetype,
						unsigned int order)
{
	struct page *start_page;
	struct zone *start_zone;

	if (!pfn_valid(start))
		return NULL;

	start_page = pfn_to_page(start);
	start_zone = page_zone(start_page);
	return bhpa_get_free_range_in_zone(start_zone, start, end, migratetype,
					   order);
}

static void bhpa_block_init(struct bhpa_block *block, phys_addr_t base,
			    unsigned int pages)
{
	if (WARN_ON(pages > BHPA_BLOCK_PAGES))
		pages = BHPA_BLOCK_PAGES;
	block->base = base;
	block->count = pages;
	block->free = pages;
	memset(&block->stats, 0, sizeof(block->stats));
	bitmap_zero(block->allocated, BHPA_BLOCK_PAGES);
	bitmap_zero(block->busy, BHPA_BLOCK_PAGES);
	bitmap_set(block->busy, pages, BHPA_BLOCK_PAGES - pages);
}

/* bhpa_block_print - Print a BHPA block details
 *
 * @seq: sequence file when called from debugfs
 * block: BHPA block structure to print
 * @memc: memory controller index
 */
static void bhpa_block_print(struct seq_file *seq,
			     const struct bhpa_block *block,
			     unsigned int memc)
{
	unsigned int i;
	char buf[80];
	int buf_off = 0;

	if (seq)
		seq_printf(seq, "MEMC%u %#llx..%#llx(%p) %u/%u/%u/%u (%u/%u) (%u/%u/%u/%u/%u)\n",
			   memc,
			   (unsigned long long)block->base,
			   (unsigned long long)(block->base + block->count *
						(uint64_t)BHPA_SIZE),
			   block, block->count, block->free,
			   block->stats.allocated, block->stats.busy,
			   block->stats.high_allocated, block->stats.high_busy,
			   block->stats.allocations.fast[0],
			   block->stats.allocations.fast[1],
			   block->stats.allocations.fast[2],
			   block->stats.allocations.fast[3],
			   block->stats.allocations.slow);
	else
		pr_info("MEMC%u %#llx..%#llx(%p) %u/%u/%u/%u (%u/%u) (%u/%u/%u/%u/%u)\n",
			   memc,
			   (unsigned long long)block->base,
			   (unsigned long long)(block->base + block->count *
						(uint64_t)BHPA_SIZE),
			   block, block->count, block->free,
			   block->stats.allocated, block->stats.busy,
			   block->stats.high_allocated, block->stats.high_busy,
			   block->stats.allocations.fast[0],
			   block->stats.allocations.fast[1],
			   block->stats.allocations.fast[2],
			   block->stats.allocations.fast[3],
			   block->stats.allocations.slow);

	for (i = 0; i < block->count; i++) {
		int rc;
		int left;
		int busy;
		int allocated;
		char ch;
		phys_addr_t addr;

		if (buf_off == 0) {
			addr = block->base + i * (phys_addr_t)BHPA_SIZE;
			rc = snprintf(buf, sizeof(buf), " %#llx: ",
				      (unsigned long long)addr);
			if (!WARN_ON(rc < 0 || rc > sizeof(buf)))
				buf_off = rc;
		}
		left = sizeof(buf) - buf_off;
		allocated = test_bit(i, block->allocated);
		busy = test_bit(i, block->busy);
		ch = '.';
		if (allocated)
			if (!busy)
				ch = 'X';
			else
				ch = 'A';
		else if (busy)
			ch = 'B';
		rc = snprintf(buf + buf_off, sizeof(buf) - buf_off, "%c", ch);
		if (rc <= 0 || rc >= left) {
			if (seq)
				seq_printf(seq, "%s\n", buf);
			else
				pr_info("%s\n", buf);
			if (!WARN_ON(i == 0))
				i--;
			buf_off = 0;
			continue;
		}
	buf_off += rc;
	}
	if (buf_off) {
		if (seq)
			seq_printf(seq, "%s\n", buf);
		else
			pr_info("%s\n", buf);
	}
}


/* bhpa_block_update_range - Update a BHPA block range
 *
 * @block: BHPA block to update
 * @range: range to supply to the block
 * @p_first_page: pointer to the first page
 * @p_last_page: pointer to the last page
 */
static void bhpa_block_update_range(const struct bhpa_block *block,
				    const struct brcmstb_range *range,
				    unsigned int *p_first_page,
				    unsigned int *p_last_page)
{
	unsigned int first_page = 0;
	unsigned int last_page = block->count;

	if (block->free == 0) {
		last_page = first_page;
	} else if (range) {
		phys_addr_t start = block->base;
		phys_addr_t end = block->base + block->count *
				  (phys_addr_t)BHPA_SIZE;
		phys_addr_t range_end = range->addr + range->size;

		if (range->addr > start)
			start = ALIGN(range->addr, BHPA_SIZE);
		if (range_end < end)
			end = ALIGN_DOWN(range_end, BHPA_SIZE);
		if (start >= end) {
			/* No overlap */
			last_page = first_page;
		} else {
			first_page = (start - block->base) / BHPA_SIZE;
			last_page = (end - block->base) / BHPA_SIZE;
		}
	}
	*p_first_page = first_page;
	*p_last_page = last_page;
}

/* bhpa_block_alloc_fast - Fast path allocator from BHPA block
 *
 * @block: BHPA block to allocate from
 * @pages: array for allocated pages
 * @count: number of entries in @pages
 * @allocated: number of actually allocated pages
 * @range: optional, restrict allocation to pages within this range
 * @order: allocation order
 */
static int bhpa_block_alloc_fast(struct bhpa_block *block,
				 uint64_t *pages,
				 unsigned int count,
				 unsigned int *allocated,
				 const struct brcmstb_range *range,
				 unsigned int order)
{
	unsigned int first_page = 0;
	unsigned int last_page = block->count;
	int rc = 0;
	unsigned long pfn_start_range;
	unsigned long pfn_end_range;
	phys_addr_t start;
	struct page *free_page = NULL;
	long prev_failed_bit = -1;
	unsigned int tries = 0;

	*allocated = 0;
	bhpa_block_update_range(block, range, &first_page, &last_page);
	B_LOG_DBG("bhpa_block_alloc_fast:%p free:%u/%u count:%u %u..%u",
		  block, block->free, block->count, count, first_page,
		  last_page);
	if (first_page == last_page)
		return rc;

	count = min(count, last_page - first_page);
	start = block->base + first_page * (phys_addr_t)BHPA_SIZE;
	pfn_start_range = (unsigned long)(start >> PAGE_SHIFT);
	pfn_end_range = (((last_page - first_page) * (phys_addr_t)BHPA_SIZE)
			 >> PAGE_SHIFT) + pfn_start_range;
	while (count) {
		phys_addr_t free_start;
		unsigned long free_bit;
		unsigned long pfn_start;
		unsigned long pfn_end;

		free_page = bhpa_get_free_page_in_range(pfn_start_range,
							pfn_end_range,
							MIGRATE_MOVABLE,
							order);
		if (free_page == NULL) {
			B_LOG_TRACE("block_alloc_fast:%p:%u no free pages count:%u",
				    block, order, count);
			break;
		}
		free_start = page_to_phys(free_page);
		if (free_start < block->base)
			break;
		free_bit = (free_start - block->base) / BHPA_SIZE;
		B_LOG_TRACE("block_alloc_fast:%p:%u free:%u:(%lu)",
			    block, order, count, free_bit);
		if (WARN_ON(free_bit >= block->count) ||
		    WARN_ON(free_bit < first_page) ||
		    WARN_ON(free_bit >= last_page) ||
		    WARN_ON(test_bit(free_bit, block->allocated)))
			break;
		free_start = block->base + free_bit * (phys_addr_t)BHPA_SIZE;
		pfn_start = (unsigned long)(free_start >> PAGE_SHIFT);
		pfn_end = pfn_start + (BHPA_SIZE >> PAGE_SHIFT);
		rc = alloc_contig_range(pfn_start, pfn_end, MIGRATE_MOVABLE);
		if (rc == -EINTR)
			break;
		if (rc != 0) {
			B_LOG_DBG("block_alloc_fast:%p: failed : %u:(%lu,prev:%ld) %#llx (%lx..%lx) %#llx",
				  block, count, free_bit, prev_failed_bit,
				  (unsigned long long)free_start, pfn_start,
				  pfn_end,
				  (unsigned long long)((phys_addr_t)
						        pfn_start << PAGE_SHIFT));
			rc = 0;
			if (prev_failed_bit == free_bit)
				/*
				 *  we wouldn't get any different free pages,
				 *  and it can't be allocated, so bail out
				 */
				break;

			prev_failed_bit = free_bit;
			tries++;
			if (tries > 10)
				break;
			continue; /* keep on trying */
		}
		B_LOG_DBG("block_alloc_fast:%p:%u allocated: %u:(%lu) %#llx (%lx..%lx)",
			  block, order, count, free_bit,
			  (unsigned long long)free_start, pfn_start, pfn_end);
		if (!test_and_set_bit(free_bit, block->busy) &&
		    !WARN_ON(!block->free)) {
			unsigned int fast_page_size;

			block->free--;
			block->stats.allocated++;
			block->stats.high_allocated =
				max(block->stats.high_allocated,
				    block->stats.allocated);
			fast_page_size = get_order(BHPA_SIZE) - order;
			WARN_ON(fast_page_size >=
				sizeof(block->stats.allocations.fast) /
				sizeof(block->stats.allocations.fast[0]));
			block->stats.allocations.fast[fast_page_size]++;
		}
		set_bit(free_bit, block->allocated);
		pages[*allocated] = free_start;
		(*allocated)++;
		count--;
	}
	return rc;
}

/* bhpa_block_alloc - Slow path allocator from BHPA block
 *
 * @block: BHPA block to allocate from
 * @pages: array for allocated pages
 * @count: number of entries in @pages
 * @allocated: number of actually allocated pages
 * @range: optional, restrict allocation to pages within this range
 */
static int bhpa_block_alloc(struct bhpa_block *block,
			    uint64_t *pages,
			    unsigned int count,
			    unsigned int *allocated,
			    const struct brcmstb_range *range)
{
	unsigned int first_page = 0;
	unsigned int last_page = block->count;
	int rc = 0;

	*allocated = 0;
	bhpa_block_update_range(block, range, &first_page, &last_page);

	B_LOG_DBG("bhpa_block_alloc:%p free:%u/%u count:%u %u..%u",
		  block, block->free, block->count, count, first_page,
		  last_page);
	if (first_page == last_page)
		return rc;
	count = min(count, last_page - first_page);

	while (count > 0) {
		unsigned long pfn_start;
		unsigned long pfn_end;
		unsigned long free_bit;
		unsigned int max_pages;
		phys_addr_t start;
		unsigned int i;
		unsigned int free_count;

		free_bit = find_next_zero_bit(block->busy, BHPA_BLOCK_PAGES,
					      first_page);
		B_LOG_TRACE("block_alloc:%p count:%u %u..%u -> %lu", block,
			    count, first_page, last_page, free_bit);
		if (free_bit >= last_page)
			break;
		max_pages = min3(8U, count, last_page - (unsigned int)free_bit);
		/* try to extend range of pages */
		free_count = 1;
		while (free_count < max_pages) {
			if (!test_bit(free_bit + free_count, block->busy))
				free_count++;
			else
				break;
		}
		start = block->base + free_bit * (phys_addr_t)BHPA_SIZE;
		pfn_start = (unsigned long)(start >> PAGE_SHIFT);
		pfn_end = pfn_start + free_count * (BHPA_SIZE >> PAGE_SHIFT);
		B_LOG_TRACE("block_alloc:%p: %lu:%u (%lx..%lx) %#llx",
			    block, free_bit, free_count, pfn_start,
			    pfn_end, (unsigned long long)start);
		rc = alloc_contig_range(pfn_start, pfn_end, MIGRATE_MOVABLE);
		if (rc != 0) {
			B_LOG_DBG("block_alloc:%p: %lu:%u (%lx..%lx) %#llx failed:%d",
				  block, free_bit, free_count, pfn_start,
				  pfn_end, (unsigned long long)start, rc);
		}
		if (rc == -EINTR)
			break;
		if (rc != 0 && free_count != 1) {
			/* if multi-page allocation failed, try single page */
			free_count = 1;
			pfn_end = pfn_start + free_count *
				  (BHPA_SIZE >> PAGE_SHIFT);
			B_LOG_TRACE("block_alloc:%p: %lu (%lx..%lx) %#llx",
				    block, free_bit, pfn_start, pfn_end,
				    (unsigned long long)start);
			rc = alloc_contig_range(pfn_start, pfn_end,
						MIGRATE_MOVABLE);
			if (rc != 0) {
				B_LOG_DBG("block_alloc:%p: %lu (%lx..%lx) %#llx failed:%d",
					  block, free_bit, pfn_start, pfn_end,
					  (unsigned long long)start, rc);
			}
			if (rc == -EINTR)
				break;
		}
		for (i = 0; i < free_count; i++)
			set_bit(free_bit + i, block->busy);
		block->free -= free_count;
		first_page = free_bit + free_count;
		if (rc == 0) {
			B_LOG_DBG("block_alloc:%p: allocated: %u:%#llx(%lu) %#llx pages:%u",
				  block, count, (unsigned long long)start,
				  free_bit, (unsigned long long)start,
				  free_count);
			count -= free_count;
			block->stats.allocations.slow += free_count;
			block->stats.allocated += free_count;
			block->stats.high_allocated =
				max(block->stats.high_allocated,
				    block->stats.allocated);
			for (i = 0; i < free_count; i++) {
				set_bit(free_bit + i, block->allocated);
				pages[*allocated] = start + i * BHPA_SIZE;
				(*allocated)++;
			}
		} else {
			B_LOG_DBG("block_alloc:%p: can't be allocated: %u:%#llx(%lu)",
				  block, count, (unsigned long long)start,
				  free_bit);
			block->stats.busy += free_count;
			block->stats.high_busy = max(block->stats.high_busy,
						     block->stats.busy);
			rc = 0;
		}
	}
	return rc;
}

static unsigned int bhpa_block_clear_busy(struct bhpa_block *block)
{
	unsigned int prev_free = block->free;
	unsigned int i;

	B_LOG_TRACE(">block_clear_busy:%p free:%u", block, block->free);
	for (i = 0;;) {
		unsigned long free_bit;

		free_bit = find_next_zero_bit(block->allocated,
					      BHPA_BLOCK_PAGES, i);
		B_LOG_TRACE("block_clear_busy:%p free_bit:%u->%u", block, i,
			    (unsigned int)free_bit);
		if (free_bit >= block->count)
			break;
		if (test_and_clear_bit(free_bit, block->busy) &&
		    !WARN_ON(block->free >= block->count))
			block->free++;
		i = free_bit + 1;
	}
	B_LOG_TRACE("<block_clear_busy:%p free:%u", block, block->free);
	B_LOG_DBG("block_alloc:%p cleared:%u/%u pages free:%u(from %u)",
		  block, block->free - prev_free, block->stats.busy,
		  block->free, block->count);
	WARN_ON(block->stats.busy != block->free - prev_free);
	block->stats.busy = 0;

	return block->free - prev_free;
}

static void bhpa_block_free(struct bhpa_block *block,
			    phys_addr_t page)
{
	unsigned int page_no;
	unsigned long pfn_page;

	if (WARN_ON(page < block->base || page % BHPA_SIZE))
		return;

	page_no = (page - block->base) / BHPA_SIZE;
	B_LOG_DBG("bhpa_block_free:%p free:%u/%u page:%#llx page_no:%u",
		  block, block->free, block->count,
		  (unsigned long long)page, page_no);

	if (WARN_ON(page_no >= block->count ||
		    !test_bit(page_no, block->allocated) ||
		    !test_bit(page_no, block->busy) ||
		    block->free >= block->count))
		return;

	clear_bit(page_no, block->busy);
	clear_bit(page_no, block->allocated);
	block->free++;
	WARN_ON(block->stats.allocated <= 0);
	block->stats.allocated--;
	pfn_page = (unsigned long)(page >> PAGE_SHIFT);
	free_contig_range(pfn_page, BHPA_SIZE / PAGE_SIZE);
}

static __init void bhpa_memc_init(struct bhpa_memc *a)
{
	INIT_LIST_HEAD(&a->blocks);
}

static void bhpa_memc_free(struct bhpa_memc *a, const uint64_t *pages,
			   unsigned int count)
{
	struct bhpa_block *block;

	while (count > 0) {
		phys_addr_t page = *pages;
		bool found = false;

		list_for_each_entry(block, &a->blocks, node) {
			if (page >= block->base &&
			    page < block->base + (block->count * BHPA_SIZE)) {
				bhpa_block_free(block, page);
				found = true;
				break;
			}
		}
		B_LOG_TRACE("bhpa_memc_free:%p pages:%lx", a,
			    (unsigned long)page);
		WARN_ON(!found);
		pages++;
		count--;
	}
}

static void bhpa_memc_print(struct seq_file *seq, const struct bhpa_memc *a,
			    unsigned int memcIndex)
{
	const struct bhpa_block *block;

	list_for_each_entry(block, &a->blocks, node)
		bhpa_block_print(seq, block, memcIndex);
}


static int bhpa_memc_alloc(struct bhpa_memc *a, unsigned memcIndex, uint64_t *pages,
			   unsigned int count, unsigned int *allocated,
			   const struct brcmstb_range *range)
{
	struct bhpa_block *block;
	int rc = 0;
	unsigned int pass;
	unsigned int page_size;

	*allocated = 0;
	for (page_size = BHPA_SIZE; page_size >= BHPA_SIZE / 8;
	     page_size = page_size / 2) {
		unsigned int order = get_order(page_size);

		list_for_each_entry(block, &a->blocks, node) {
			unsigned int block_allocated;

			rc = bhpa_block_alloc_fast(block, pages, count,
						   &block_allocated, range,
						   order);
			*allocated += block_allocated;
			pages += block_allocated;
			count -= block_allocated;
			if (rc != 0)
				goto done;
			if (count == 0)
				goto done;
		}
	}
	for (pass = 0; pass < 2; pass++) {
		list_for_each_entry(block, &a->blocks, node) {
			unsigned int block_allocated;

			rc = bhpa_block_alloc(block, pages, count,
					      &block_allocated, range);
			if (pass == 0 && count == block_allocated) {
				B_LOG_TRACE("bhpa_memc_alloc:%p pages:%u/%u pass:%u",
					    block, block_allocated, count,
					    pass);
			} else {
				B_LOG_DBG("bhpa_memc_alloc:%p pages:%u/%u pass:%u",
					  block, block_allocated, count, pass);
			}
			*allocated += block_allocated;
			pages += block_allocated;
			count -= block_allocated;
			if (rc != 0)
				break;
			if (count == 0)
				goto done;
		}
		if (pass == 0) { /* clear all busy, but not allocated pages */
				 /* and try again */
			unsigned int cleared = 0;

			list_for_each_entry(block, &a->blocks, node) {
				cleared += bhpa_block_clear_busy(block);
			}
			if (cleared == 0)
				break;
		}
	}
done:
	if (rc == 0) {
		if (count != 0 && range == NULL && !list_empty(&a->blocks)) {
			pr_err("BHPA MEMC%u Out of memory\n", memcIndex);
			bhpa_memc_print(NULL, a, memcIndex);
			dump_stack();
		}
	} else {
		/* in case of error free all partially allocated memory */
		bhpa_memc_free(a, pages - *allocated, *allocated);
		*allocated = 0;
	}

	return rc;
}

void brcmstb_hpa_print(struct seq_file *seq)
{
	unsigned int i;

	mutex_lock(&bhpa_lock);
	for (i = 0; i < MAX_BRCMSTB_MEMC; i++)
		bhpa_memc_print(seq, &bhpa_allocator.memc[i], i);

	mutex_unlock(&bhpa_lock);
}

static int bhpa_debugfs_show(struct seq_file *seq, void *p)
{
	brcmstb_hpa_print(seq);
	return 0;
}

static int bhpa_debugfs_open(struct inode *inode,
	struct file *file)
{
	return single_open(file, bhpa_debugfs_show, inode->i_private);
}

static const struct file_operations bhpa_debugfs_fops = {
	.open       = bhpa_debugfs_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static __init void bhpa_debugfs_init(struct bhpa_allocator *a)
{
	a->debugfs = debugfs_create_file("bhpa", 0444, NULL, a,
					 &bhpa_debugfs_fops);
	if (IS_ERR_OR_NULL(a->debugfs))
		a->debugfs = NULL;
}

static __init void bhpa_allocator_init(struct bhpa_allocator *a)
{
	unsigned int i;

	bhpa_debugfs_init(a);
	for (i = 0; i < MAX_BRCMSTB_MEMC; i++)
		bhpa_memc_init(&a->memc[i]);
}

static __init unsigned int bhpa_trim_memory(phys_addr_t *base, phys_addr_t end)
{
	phys_addr_t _base = ALIGN(*base, BHPA_SIZE);

	*base = _base;
	if (_base >= end)
		return 0;

	return (end - _base)/BHPA_SIZE;
}

static __init int bhpa_memc_add_memory(struct bhpa_memc *a, phys_addr_t base,
				       phys_addr_t end)
{
	unsigned int total_pages = bhpa_trim_memory(&base, end);
	struct list_head *head = a->blocks.next;

	while (total_pages) {
		struct bhpa_block *block = kmalloc(sizeof(*block), GFP_KERNEL);
		unsigned int pages;

		if (total_pages > BHPA_BLOCK_PAGES)
			pages = BHPA_BLOCK_PAGES;
		else
			pages = total_pages;

		if (block) {
			bhpa_block_init(block, base, pages);
			B_LOG_DBG("Adding list... %p %p page:%u", &a->blocks,
				  &block->node, pages);
			list_add(&block->node, &a->blocks);
		} else {
			struct bhpa_block *tmp, *p;

			list_for_each_entry_safe(p, tmp, &a->blocks, node) {
				if (&p->node == head)
					break;
				list_del(&p->node);
				kfree(p);
			}
			return -ENOMEM;
		}

		total_pages -= pages;
		base += BHPA_SIZE * BHPA_BLOCK_PAGES;
	}
	return 0;
}

static __init int bhpa_add_memory(struct bhpa_region *p)
{
	unsigned int i;
	unsigned long pfn = __phys_to_pfn(p->addr);
	phys_addr_t end = p->addr + p->size;
	int rc;

	if (WARN_ON(p->memc < 0 || p->memc >= MAX_BRCMSTB_MEMC))
		return -1;

	i = (unsigned int)(p->size >> (pageblock_order + PAGE_SHIFT));

	mutex_lock(&bhpa_lock);
	rc = bhpa_memc_add_memory(&bhpa_allocator.memc[p->memc], p->addr, end);
	mutex_unlock(&bhpa_lock);

	do {
		init_bhpa_reserved_pageblock(pfn_to_page(pfn));
		pfn += pageblock_nr_pages;
	} while (--i);

	return rc;
}

static int __init bhpa_init(void)
{
	int rc, i;

	B_LOG_DBG("Init");
	bhpa_allocator_init(&bhpa_allocator);
	B_LOG_DBG("Adding memory");
	for (i = 0; i < n_bhpa_regions; i++) {
		rc = bhpa_add_memory(&bhpa_regions[i]);
		B_LOG_DBG("Adding memory  -> %d", rc);
	}

	return 0;
}
core_initcall(bhpa_init);

/*
 * brcmstb_hpa_alloc() - Allocate 2MB pages from ZONE_MOVABLE
 *
 * @memcIndex: memory controller
 * @pages: array for allocated pages
 * @count: number of entries in pages array above
 * @alloced: number of actually allocated pages
 * @range: optional, restrict allocation to pages within this range
 *
 * Return: 0 on success, negative on failure.
 */
int brcmstb_hpa_alloc(unsigned int memcIndex, uint64_t *pages,
		      unsigned int count, unsigned int *alloced,
		      const struct brcmstb_range *range)
{
	int rc;

	if (memcIndex >= MAX_BRCMSTB_MEMC || !pages || !count || !alloced)
		return -EINVAL;

	*alloced = 0;

	rc = mutex_lock_interruptible(&bhpa_lock);
	if (rc != 0)
		return rc;

	rc = bhpa_memc_alloc(&bhpa_allocator.memc[memcIndex], memcIndex, pages, count,
			     alloced, range);

	mutex_unlock(&bhpa_lock);
	return rc;
}

/*
 * brcmstb_hpa_free() - Release 2MB pages allocated by brcmstb_hpa_alloc().
 *
 * @memcIndex: memory controller
 * @pages: array for allocated pages
 * @count: number of entries in pages array above
 *
 */
void brcmstb_hpa_free(unsigned int memcIndex, const uint64_t *pages,
		      unsigned int count)
{
	if (WARN_ON(memcIndex >= MAX_BRCMSTB_MEMC || !pages || !count))
		return;

	mutex_lock(&bhpa_lock);
	bhpa_memc_free(&bhpa_allocator.memc[memcIndex], pages, count);
	mutex_unlock(&bhpa_lock);
}
