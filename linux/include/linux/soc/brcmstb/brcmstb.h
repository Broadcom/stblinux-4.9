#ifndef __BRCMSTB_SOC_H
#define __BRCMSTB_SOC_H

#include "aon_defs.h"

#define BRCM_ID(reg)	((u32)reg >> 28 ? (u32)reg >> 16 : (u32)reg >> 8)
#define BRCM_REV(reg)	((u32)reg & 0xff)

bool soc_is_brcmstb(void);
int dtu_save(void);
int brcmstb_dtusave_init(struct brcmstb_bootloader_dtu_table *tbl);
int brcmstb_regsave_init(void);

/*
 * Helper functions for getting family or product id from the
 * SoC driver.
 */
u32 brcmstb_get_family_id(void);
u32 brcmstb_get_product_id(void);

#ifdef CONFIG_BRCMSTB_MEMORY_API
int brcmstb_memory_phys_addr_to_memc(phys_addr_t pa);
u64 brcmstb_memory_memc_size(unsigned int memc);
#else
static inline int brcmstb_memory_phys_addr_to_memc(phys_addr_t pa)
{
	return -EINVAL;
}

static inline u64 brcmstb_memory_memc_size(int memc)
{
	return -1;
}
#endif

#endif /* __BRCMSTB_SOC_H */
