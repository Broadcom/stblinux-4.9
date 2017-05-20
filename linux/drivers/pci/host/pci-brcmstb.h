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
struct brcm_pcie;

struct brcm_msi *brcm_pcie_to_msi(struct brcm_pcie *pcie);
dma_addr_t brcm_to_pci(dma_addr_t addr);
dma_addr_t brcm_to_cpu(dma_addr_t addr);

extern struct notifier_block brcmstb_platform_nb;

#ifdef CONFIG_PCI_MSI
struct brcm_msi *brcm_alloc_init_msi(
	struct brcm_pcie *pcie, struct device *dev, void __iomem *base,
	int irq, struct device_node *dn, const char *name);

int brcm_pcie_enable_msi(struct brcm_msi *msi, int nr, bool suspended);

void brcm_set_msi_rev(struct brcm_msi *msi, unsigned rev);

void brcm_set_msi_target_addr(struct brcm_msi *msi, u64 target);

struct msi_controller *brcm_get_msi_chip(struct brcm_msi *msi);

#else

static inline struct brcm_msi *brcm_alloc_init_msi(
	struct brcm_pcie *pcie, struct device *dev, void __iomem *base,
	int irq, struct device_node *dn, const char *name)
{
	return NULL;
}

static inline int brcm_pcie_enable_msi(
	struct brcm_msi *msi, int nr, bool suspended)
{
	return -EOPNOTSUPP;
}

static inline void brcm_set_msi_target_addr(struct brcm_msi *msi, u64 target)
{}

static inline void brcm_set_msi_rev(struct brcm_msi *msi, unsigned rev)
{}

static inline struct msi_controller *brcm_get_msi_chip(struct brcm_msi *msi)
{
	return NULL;
}
#endif /* CONFIG_PCI_MSI */

#endif /* __BRCMSTB_PCI_H */
