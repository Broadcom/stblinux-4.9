// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom STB Ethernet PHY library fixups
 */
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/brcmphy.h>
#include <linux/platform_data/mdio-bcm-unimac.h>

#include "../../../net/phy/bcm-phy-lib.h"

static int bcm54810_no_broad_reach(struct phy_device *phydev)
{
	int val;

	/* Make sure we clear the PDOWN bit before attempting any
	 * register access
	 */
	val = genphy_resume(phydev);
	if (val < 0)
		return val;

	/* Disable BroadR-Reach (enabled by default) */
	val = bcm_phy_read_exp(phydev, BCM54810_EXP_BROADREACH_LRE_MISC_CTL);
	val &= ~BCM54810_EXP_BROADREACH_LRE_MISC_CTL_EN;
	return bcm_phy_write_exp(phydev, BCM54810_EXP_BROADREACH_LRE_MISC_CTL,
				 val);
}

static int __init brcmstb_phy_fixups(void)
{
	const char *mdio_bus[] = {
		"9c0c614.mdio--1:07",
		"9c0ce14.mdio--1:07",
		UNIMAC_MDIO_DRV_NAME "-0:07",
	};
	int ret = 0, i;

	if (!of_machine_is_compatible("brcm,bcm72113a0") &&
	    !of_machine_is_compatible("brcm,bcm7211b0") &&
	    !of_machine_is_compatible("brcm,bcm72165a0"))
		return ret;

	/* Register a PHY fixup to disable the auto-power down of CLK125
	 * on BCM97211SV boards connected to the GENET_0 MDIO controller
	 * at address 7 or one of the two ASP UniMAC MDIO controllers
	 * on BCM972165SV boards.
	 */
	for (i = 0; i < ARRAY_SIZE(mdio_bus); i++) {
		ret = phy_register_fixup(mdio_bus[i],
					 PHY_ID_BCM54810, 0xfffffff0,
					 bcm54810_no_broad_reach);
		if (ret)
			return ret;
	}

	return ret;
}
arch_initcall(brcmstb_phy_fixups);
