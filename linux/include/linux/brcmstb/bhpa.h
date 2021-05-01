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

#ifndef _LINUX_BRCMSTB_HPA_H
#define _LINUX_BRCMSTB_HPA_H
#include <linux/brcmstb/memory_api.h>

#ifdef CONFIG_BRCMSTB_HUGEPAGES
extern __init int early_for_each_memc_range(
	int (*fn)(int m, u64 a, u64 s, void *c), void *c);

int bhpa_find_region(phys_addr_t addr, phys_addr_t size);
int bhpa_region_info(int idx, phys_addr_t *addr, phys_addr_t *size);

void __init brcmstb_bhpa_reserve(void);
void __init brcmstb_bhpa_setup(phys_addr_t addr, phys_addr_t size);
void brcmstb_hpa_print(struct seq_file *seq);
int brcmstb_hpa_alloc(unsigned int memcIndex, uint64_t *pages,
		      unsigned int count, unsigned int *alloced,
		      const struct brcmstb_range *range, gfp_t flags);
void brcmstb_hpa_free(unsigned int memcIndex, const uint64_t *pages,
		      unsigned int count);
#else
static inline void brcmstb_bhpa_reserve(void) {}
static inline void brcmstb_hpa_print(struct seq_file *seq) {}
static inline int brcmstb_hpa_alloc(unsigned int memcIndex, uint64_t *pages,
				    unsigned int count, unsigned int *alloced,
				    const struct brcmstb_range *range,
				    gfp_t flags)
{
	return -ENOSYS;
}
static inline void brcmstb_hpa_free(unsigned int memcIndex,
				    const uint64_t *pages,
				    unsigned int count) {}
#endif

#endif /* _LINUX_BRCMSTB_HPA_H */
