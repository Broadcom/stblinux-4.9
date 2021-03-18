/*
 * sdhci-brcmstb.c Support for SDHCI on Broadcom BRCMSTB SoC's
 *
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "sdhci-pltfm.h"

#define SDHCI_VENDOR 0x78
#define  SDHCI_VENDOR_ENHANCED_STRB 0x1

#define BRCMSTB_PRIV_FLAGS_NO_64BIT		BIT(0)
#define BRCMSTB_PRIV_FLAGS_BROKEN_TIMEOUT	BIT(1)

#define DRIVER_STRENGTH_A 1	/* 1.5x */
#define DRIVER_STRENGTH_B 0	/* 1.0x */
#define DRIVER_STRENGTH_C 2	/* .75x */
#define DRIVER_STRENGTH_D 3	/* .5x  */

struct sdhci_brcmstb_priv {
	void __iomem *cfg_regs;
	int driver_strength;
	struct regmap *rmap;
};

struct brcmstb_match_priv {
	void (*hs400es)(struct mmc_host *mmc, struct mmc_ios *ios);
	int (*execute_tuning)(struct mmc_host *mmc, u32 opcode);
	const struct sdhci_ops *ops;
	unsigned int flags;
};

static void sdhci_brcmstb_hs400es(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdhci_host *host = mmc_priv(mmc);

	u32 reg;

	dev_dbg(mmc_dev(mmc), "%s(): Setting HS400-Enhanced-Strobe mode\n",
		__func__);
	reg = readl(host->ioaddr + SDHCI_VENDOR);
	if (ios->enhanced_strobe)
		reg |= SDHCI_VENDOR_ENHANCED_STRB;
	else
		reg &= ~SDHCI_VENDOR_ENHANCED_STRB;
	writel(reg, host->ioaddr + SDHCI_VENDOR);
}

static int sdhci_brcmstb_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdhci_host *host = mmc_priv(mmc);
	int stat;

	dev_dbg(mmc_dev(mmc), "%s(): Execute tuning\n", __func__);
	stat = sdhci_execute_tuning(mmc, opcode);
	if (stat)
		return stat;
	if (host->timing == MMC_TIMING_MMC_HS200)
		/* Place holder for CFG register setting needed after tuning */
		dev_dbg(mmc_dev(mmc), "Post tuning hook for HS200\n");
	return stat;
}

static void sdhci_brcmstb_set_clock(struct sdhci_host *host, unsigned int clock)
{
	u16 clk;
	unsigned long timeout;

	host->mmc->actual_clock = 0;

	clk = sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return;

	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			pr_err("%s: Internal clock never stabilised.\n",
			       mmc_hostname(host->mmc));
			return;
		}
		timeout--;
		spin_unlock_irq(&host->lock);
		usleep_range(900, 1100);
		spin_lock_irq(&host->lock);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

void brcmstb_set_uhs_signaling(struct sdhci_host *host, unsigned int timing)
{
	u16 ctrl_2;

	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	if (timing == MMC_TIMING_SD_HS ||
		 timing == MMC_TIMING_MMC_HS ||
		 timing == MMC_TIMING_UHS_SDR25) {
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
		sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);
	} else {
		sdhci_set_uhs_signaling(host, timing);
	}
}

/* definitions for the SYS_CTRL_SUN_TOP_CTRL_GENERAL_CTRL_NO_SCAN_2 register */
#define EMMC_CTL_PAD_SEL_MASK 0x07000000
#define EMMC_CTL_PAD_SEL_SHIFT 24
#define EMMC_DATA_PAD_SEL_MASK 0x00000070
#define EMMC_DATA_PAD_SEL_SHIFT 4
#define EMMC_PAD_SEL_DRIVE_2MA 0
#define EMMC_PAD_SEL_DRIVE_4MA 1
#define EMMC_PAD_SEL_DRIVE_6MA 2
#define EMMC_PAD_SEL_DRIVE_8MA 3
#define EMMC_PAD_SEL_DRIVE_10MA 4
#define EMMC_PAD_SEL_DRIVE_12MA 5
#define EMMC_PAD_SEL_DRIVE_14MA 6
#define EMMC_PAD_SEL_DRIVE_16MA 7

static void set_syscon_strength(struct sdhci_host *host,
				struct sdhci_brcmstb_priv *priv,
				int strength)
{
	int val;
	int err;

	if (!priv->rmap)
		return;
	switch (strength) {
	case DRIVER_STRENGTH_B:
	default:
		val = EMMC_PAD_SEL_DRIVE_8MA;
		break;
	case DRIVER_STRENGTH_A:
		val = EMMC_PAD_SEL_DRIVE_12MA;
		break;
	case DRIVER_STRENGTH_C:
		val = EMMC_PAD_SEL_DRIVE_6MA;
		break;
	case DRIVER_STRENGTH_D:
		val = EMMC_PAD_SEL_DRIVE_4MA;
		break;
	}
	dev_dbg(mmc_dev(host->mmc), "Setting syscon drive strength to 0x%x\n",
		val);

	/* Set the pad drive strength for the eMMC control and data signals */
	err = regmap_update_bits(priv->rmap, 0,
				 EMMC_CTL_PAD_SEL_MASK | EMMC_DATA_PAD_SEL_MASK,
				 (val << EMMC_CTL_PAD_SEL_SHIFT) |
				 (val << EMMC_DATA_PAD_SEL_SHIFT));
	if (err)
		dev_err(mmc_dev(host->mmc),
			"Error setting syscon drive strength\n");
}

static int brcmstb_select_drive_strength(struct sdhci_host *host,
					 struct mmc_card *card,
					 unsigned int max_dtr, int host_drv,
					 int card_drv, int *drv_type)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_brcmstb_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int strength = priv->driver_strength;

	dev_dbg(mmc_dev(host->mmc),
		"Drive strength: speed: %d, host: 0x%x, card: 0x%x, res: %d\n",
		max_dtr, host_drv, card_drv, strength);

	set_syscon_strength(host, priv, strength);
	*drv_type = strength;
	return strength;
}

#ifdef CONFIG_PM_SLEEP
static int sdhci_brcmstb_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int res;

	res = sdhci_suspend_host(host);
	if (res)
		return res;
	clk_disable_unprepare(pltfm_host->clk);
	return res;
}

static int sdhci_brcmstb_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int err;

	err = clk_prepare_enable(pltfm_host->clk);
	if (err)
		return err;
	return sdhci_resume_host(host);
}

#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sdhci_brcmstb_pmops, sdhci_brcmstb_suspend,
			sdhci_brcmstb_resume);

static const struct sdhci_ops sdhci_brcmstb_ops = {
	.set_clock = sdhci_set_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct sdhci_ops sdhci_brcmstb_ops_7216 = {
	.set_clock = sdhci_brcmstb_set_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = brcmstb_set_uhs_signaling,
	.select_drive_strength = brcmstb_select_drive_strength,
};

static const struct brcmstb_match_priv match_priv_7425 = {
	.flags = BRCMSTB_PRIV_FLAGS_NO_64BIT |
	BRCMSTB_PRIV_FLAGS_BROKEN_TIMEOUT,
	.ops = &sdhci_brcmstb_ops,
};

static const struct brcmstb_match_priv match_priv_7445 = {
	.flags = BRCMSTB_PRIV_FLAGS_BROKEN_TIMEOUT,
	.ops = &sdhci_brcmstb_ops,
};

static const struct brcmstb_match_priv match_priv_7216 = {
	.hs400es = sdhci_brcmstb_hs400es,
	.execute_tuning = sdhci_brcmstb_execute_tuning,
	.ops = &sdhci_brcmstb_ops_7216,
};

static const struct of_device_id sdhci_brcm_of_match[] = {
	{ .compatible = "brcm,bcm7425-sdhci", .data = &match_priv_7425 },
	{ .compatible = "brcm,bcm7445-sdhci", .data = &match_priv_7445 },
	{ .compatible = "brcm,bcm7216-sdhci", .data = &match_priv_7216 },
	{},
};

static int sdhci_brcmstb_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_data brcmstb_pdata;
	const struct brcmstb_match_priv *match_priv;
	struct device_node *dn = pdev->dev.of_node;
	struct sdhci_pltfm_host *pltfm_host;
	const struct of_device_id *match;
	struct sdhci_brcmstb_priv *priv;
	struct regmap *rmap = NULL;
	struct sdhci_host *host;
	int driver_strength = 0;
	struct resource *iomem;
	const char *strength;
	struct clk *clk;
	int res;

	match = of_match_node(sdhci_brcm_of_match, pdev->dev.of_node);
	match_priv = match->data;

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_err(&pdev->dev, "Clock not found in Device Tree\n");
		clk = NULL;
	}
	res = clk_prepare_enable(clk);
	if (res)
		return res;

	memset(&brcmstb_pdata, 0, sizeof(brcmstb_pdata));

	res = of_property_read_string(dn, "driver-strength", &strength);
	if (res == 0) {
		switch (strength[0]) {
		case 'A':
			driver_strength = DRIVER_STRENGTH_A;
			break;
		case 'B':
			driver_strength = DRIVER_STRENGTH_B;
			break;
		case 'C':
			driver_strength = DRIVER_STRENGTH_C;
			break;
		case 'D':
			driver_strength = DRIVER_STRENGTH_D;
			break;
		default:
			dev_err(&pdev->dev,
				"Invalid \"driver_strength\" property\n");
		}
	}
	/* Get the optional chip specific drive strength register */
	rmap = syscon_regmap_lookup_by_phandle(dn, "syscon-emmc");
	if (IS_ERR(rmap))
		rmap = NULL;

	brcmstb_pdata.ops = match_priv->ops;
	host = sdhci_pltfm_init(pdev, &brcmstb_pdata,
				sizeof(struct sdhci_brcmstb_priv));
	if (IS_ERR(host)) {
		res = PTR_ERR(host);
		goto err_clk;
	}

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);
	priv->driver_strength = driver_strength;
	priv->rmap = rmap;

	/* Map in the non-standard CFG registers */
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->cfg_regs = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(priv->cfg_regs)) {
		res = PTR_ERR(priv->cfg_regs);
		goto err;
	}

	/* Enable MMC_CAP2_HC_ERASE_SZ for better max discard calculations */
	host->mmc->caps2 |= MMC_CAP2_HC_ERASE_SZ;

	sdhci_get_of_property(pdev);
	res = mmc_of_parse(host->mmc);
	if (res)
		goto err;

	/*
	 * If the chip has enhanced strobe and it's enabled, add
	 * callback
	 */
	if (match_priv->hs400es &&
	    (host->mmc->caps2 & MMC_CAP2_HS400_ES))
		host->mmc_host_ops.hs400_enhanced_strobe = match_priv->hs400es;
	if (match_priv->execute_tuning)
		host->mmc_host_ops.execute_tuning = match_priv->execute_tuning;

	/*
	 * Supply the existing CAPS, but clear the UHS modes. This
	 * will allow these modes to be specified by device tree
	 * properties through mmc_of_parse().
	 */
	host->caps = sdhci_readl(host, SDHCI_CAPABILITIES);
	if (match_priv->flags & BRCMSTB_PRIV_FLAGS_NO_64BIT)
		host->caps &= ~SDHCI_CAN_64BIT;
	host->caps1 = sdhci_readl(host, SDHCI_CAPABILITIES_1);
	host->caps1 &= ~(SDHCI_SUPPORT_SDR50 | SDHCI_SUPPORT_SDR104 |
			 SDHCI_SUPPORT_DDR50);
	host->quirks |= SDHCI_QUIRK_MISSING_CAPS;

	if (match_priv->flags & BRCMSTB_PRIV_FLAGS_BROKEN_TIMEOUT)
		host->quirks |= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;
	res = sdhci_add_host(host);
	if (res)
		goto err;

	pltfm_host->clk = clk;
	return res;

err:
	sdhci_pltfm_free(pdev);
err_clk:
	clk_disable_unprepare(clk);
	return res;
}

static void sdhci_brcmstb_shutdown(struct platform_device *pdev)
{
	sdhci_pltfm_suspend(&pdev->dev);
}

MODULE_DEVICE_TABLE(of, sdhci_brcm_of_match);

static struct platform_driver sdhci_brcmstb_driver = {
	.driver		= {
		.name	= "sdhci-brcmstb",
		.pm	= &sdhci_brcmstb_pmops,
		.of_match_table = of_match_ptr(sdhci_brcm_of_match),
	},
	.probe		= sdhci_brcmstb_probe,
	.remove		= sdhci_pltfm_unregister,
	.shutdown	= sdhci_brcmstb_shutdown,
};

module_platform_driver(sdhci_brcmstb_driver);

MODULE_DESCRIPTION("SDHCI driver for Broadcom BRCMSTB SoCs");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
