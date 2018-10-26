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

enum brcmstb_pm_method {
	BRCMSTB_PM_BARE_METAL = 0,
	BRCMSTB_PM_PSCI_ASSISTED,
	BRCMSTB_PM_PSCI_FULL,
};

extern enum brcmstb_pm_method brcmstb_pm_method;

#endif /* __BRCMSTB_PM_COMMON_H__ */
