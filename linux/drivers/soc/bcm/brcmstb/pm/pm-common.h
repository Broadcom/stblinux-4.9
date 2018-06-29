#ifndef __BRCMSTB_PM_COMMON_H__
#define __BRCMSTB_PM_COMMON_H__

#include <linux/types.h>
#include <linux/brcmstb/memory_api.h>

#include "xpt_dma.h"

#define MAX_EXCLUDE				16
#define MAX_REGION				16
#define MAX_EXTRA				8

extern struct dma_region exclusions[MAX_EXCLUDE];
extern struct dma_region regions[MAX_REGION];

int configure_main_hash(struct dma_region *regions, int max,
			struct dma_region *exclude, int num_exclude);
int __pm_mem_exclude(phys_addr_t addr, size_t len, bool persistent);

extern int num_regions;
extern int num_exclusions;
extern struct brcmstb_memory bm;
extern bool brcmstb_pm_psci_initialized;

#endif /* __BRCMSTB_PM_COMMON_H__ */
