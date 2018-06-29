#include <linux/stddef.h>
#include <linux/kbuild.h>
#include "pm.h"

int main(void)
{
	DEFINE(PM_DATA_DDR_PHY_BASE,
	       offsetof(struct brcmstb_memc, ddr_phy_base));
	DEFINE(PM_DATA_DDR_SHIMPHY_BASE,
	       offsetof(struct brcmstb_memc, ddr_shimphy_base));
	DEFINE(PM_DATA_DDR_CTRL, offsetof(struct brcmstb_memc, ddr_ctrl));
	DEFINE(PM_DATA_MEMC_SIZE, sizeof(struct brcmstb_memc));

	return 0;
}
