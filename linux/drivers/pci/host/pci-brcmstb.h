#ifndef __BRCMSTB_PCI_H
#define __BRCMSTB_PCI_H

/*
 * Copyright (C) 2015 - 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define BRCM_PCIE_NAME_SIZE		8
#define BRCM_INT_PCI_MSI_NR		32
#define BRCM_PCIE_HW_REV_33		0x0303

/* Broadcom PCIE Offsets */
#define PCIE_INTR2_CPU_BASE		0x4300

struct brcm_msi;
struct brcm_info;
struct platform_device;

extern struct of_pci_range *dma_ranges;
extern int num_dma_ranges;

int brcm_register_notifier(void);
int brcm_unregister_notifier(void);

#ifdef CONFIG_PCI_MSI
int brcm_msi_probe(struct platform_device *pdev, struct brcm_info *info);
void brcm_msi_set_regs(struct msi_controller *chip);
void brcm_msi_remove(struct msi_controller *chip);
#else
int brcm_msi_probe(struct platform_device *pdev, struct brcm_info *info)
{
	return -ENODEV;
}

static inline void brcm_msi_set_regs(struct msi_controller *chip) {}
static inline void brcm_msi_remove(struct msi_controller *chip) {}
#endif /* CONFIG_PCI_MSI */

struct brcm_info {
	int rev;
	u64 msi_target_addr;
	void __iomem *base;
	const char *name;
	struct brcm_pcie *pcie;
};

#define BRCMSTB_ERROR_CODE	(~(dma_addr_t)0x0)

#if defined(CONFIG_MIPS)
/* Broadcom MIPs HW implicitly does the swapping if necessary */
#define bcm_readl(a)		__raw_readl(a)
#define bcm_writel(d, a)	__raw_writel(d, a)
#else
#define bcm_readl(a)		readl(a)
#define bcm_writel(d, a)	writel(d, a)
#endif

#endif /* __BRCMSTB_PCI_H */
