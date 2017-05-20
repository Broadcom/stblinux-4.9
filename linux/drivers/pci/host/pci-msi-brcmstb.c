/*
 * Copyright (C) 2015-2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/msi.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/irqdomain.h>

#include "pci-brcmstb.h"

#define PCIE_MISC_MSI_DATA_CONFIG			0x404c
#define PCIE_MSI_INTR2_BASE				0x4500
#define PCIE_MISC_MSI_BAR_CONFIG_LO			0x4044
#define PCIE_MISC_MSI_BAR_CONFIG_HI			0x4048

#define BRCM_MSI_TRAILER	"_msi"
#define BRCM_MSI_NAME_SIZE	(BRCM_PCIE_NAME_SIZE \
				 + sizeof(BRCM_MSI_TRAILER))

/* Offsets from PCIE_INTR2_CPU_BASE and PCIE_MSI_INTR2_BASE */
#define STATUS				0x0
#define SET				0x4
#define CLR				0x8
#define MASK_STATUS			0xc
#define MASK_SET			0x10
#define MASK_CLR			0x14


struct brcm_msi {
	struct irq_domain *domain;
	struct irq_chip irq_chip;
	struct msi_controller chip;
	struct mutex lock;
	u64 target_addr;
	int irq;
	/* intr_base is the base pointer for interrupt status/set/clr regs */
	void __iomem *intr_base;
	/* intr_legacy_mask indicates how many bits are MSI interrupts */
	u32 intr_legacy_mask;
	/* intr_legacy_offset indicates bit position of MSI_01 */
	u32 intr_legacy_offset;
	/* used indicates which MSI interrupts have been alloc'd */
	unsigned long used;
	/* working indicates that on boot we have brought up MSI */
	bool working;
	char name[BRCM_MSI_NAME_SIZE];

	/* copied from the containing pcie structure */
	struct brcm_pcie *pcie;
	void __iomem *pcie_base;
	struct device *pcie_dev;
	struct device_node *pcie_dn;
	unsigned int pcie_rev;
};

struct msi_controller *brcm_get_msi_chip(struct brcm_msi *msi)
{
	return msi == NULL ? NULL : &msi->chip;
}

void brcm_set_msi_rev(struct brcm_msi *msi, unsigned rev)
{
	if (msi)
		msi->pcie_rev = rev;
}

void brcm_set_msi_target_addr(struct brcm_msi *msi, u64 target)
{
	if (msi)
		msi->target_addr = target;
}


struct brcm_msi *brcm_alloc_init_msi(struct brcm_pcie *pcie, struct device *dev,
				     void __iomem *base, int irq,
				     struct device_node *dn, const char *name)
{
	struct brcm_msi *msi = devm_kzalloc(dev, sizeof(struct brcm_msi),
					    GFP_KERNEL);

	if (!msi)
		return msi;
	msi->pcie = pcie;
	msi->pcie_dev = dev;
	msi->pcie_base = base;
	msi->pcie_dn = dn;
	msi->irq = irq;
	strlcpy(msi->name, name, BRCM_PCIE_NAME_SIZE);
	strlcat(msi->name, BRCM_MSI_TRAILER, BRCM_MSI_NAME_SIZE);
	return msi;
}

static struct brcm_msi *to_brcm_msi(struct msi_controller *chip)
{
	return container_of(chip, struct brcm_msi, chip);
}


static int brcm_msi_alloc(struct brcm_msi *chip)
{
	int msi;

	mutex_lock(&chip->lock);
	msi = ~chip->used ? ffz(chip->used) : -1;

	if (msi >= 0 && msi < BRCM_INT_PCI_MSI_NR)
		chip->used |= (1 << msi);
	else
		msi = -ENOSPC;

	mutex_unlock(&chip->lock);
	return msi;
}


static void brcm_msi_free(struct brcm_msi *chip, unsigned long irq)
{
	mutex_lock(&chip->lock);
	chip->used &= ~(1 << irq);
	mutex_unlock(&chip->lock);
}


static irqreturn_t brcm_pcie_msi_irq(int irq, void *data)
{
	struct brcm_pcie *pcie = data;
	struct brcm_msi *msi = brcm_pcie_to_msi(pcie);
	unsigned long status;

	status = __raw_readl(msi->intr_base + STATUS) & msi->intr_legacy_mask;

	if (!status)
		return IRQ_NONE;

	while (status) {
		unsigned int index = ffs(status) - 1;
		unsigned int irq;

		/* clear the interrupt */
		__raw_writel(1 << index, msi->intr_base + CLR);
		status &= ~(1 << index);

		/* Account for legacy interrupt offset */
		index -= msi->intr_legacy_offset;

		irq = irq_find_mapping(msi->domain, index);
		if (irq) {
			if (msi->used & (1 << index))
				generic_handle_irq(irq);
			else
				dev_info(msi->pcie_dev, "unhandled MSI %d\n",
					 index);
		} else {
			/* Unknown MSI, just clear it */
			dev_dbg(msi->pcie_dev, "unexpected MSI\n");
		}
	}
	return IRQ_HANDLED;
}


static int brcm_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			      struct msi_desc *desc)
{
	struct brcm_msi *msi = to_brcm_msi(chip);
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;
	u32 data;

	hwirq = brcm_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq) {
		brcm_msi_free(msi, hwirq);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	msg.address_lo = lower_32_bits(msi->target_addr);
	msg.address_hi = upper_32_bits(msi->target_addr);
	data = __raw_readl(msi->pcie_base + PCIE_MISC_MSI_DATA_CONFIG);
	msg.data = ((data >> 16) & (data & 0xffff)) | hwirq;
	wmb(); /* just being cautious */
	write_msi_msg(irq, &msg);

	return 0;
}


static void brcm_msi_teardown_irq(struct msi_controller *chip, unsigned int irq)
{
	struct brcm_msi *msi = to_brcm_msi(chip);
	struct irq_data *d = irq_get_irq_data(irq);

	brcm_msi_free(msi, d->hwirq);
}


static int brcm_msi_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	struct brcm_pcie *pcie = domain->host_data;
	struct brcm_msi *msi = brcm_pcie_to_msi(pcie);

	irq_set_chip_and_handler(irq, &msi->irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = brcm_msi_map,
};


int brcm_pcie_enable_msi(struct brcm_msi *msi, int nr, bool suspended)
{
	u32 data_val, msi_lo, msi_hi;
	int err;

	if (!suspended) {
		/* We are only here on cold boot */
		mutex_init(&msi->lock);

		msi->chip.dev = msi->pcie_dev;
		msi->chip.setup_irq = brcm_msi_setup_irq;
		msi->chip.teardown_irq = brcm_msi_teardown_irq;

		/* We have multiple RC controllers.  We may have as many
		 * MSI controllers for them.  We want each to have a
		 * unique name, so we go to the trouble of having an
		 * irq_chip per RC (instead of one for all of them). */
		msi->irq_chip.name = msi->name;
		msi->irq_chip.irq_enable = unmask_msi_irq;
		msi->irq_chip.irq_disable = mask_msi_irq;
		msi->irq_chip.irq_mask = mask_msi_irq;
		msi->irq_chip.irq_unmask = unmask_msi_irq;

		msi->domain =
			irq_domain_add_linear(msi->pcie_dn, BRCM_INT_PCI_MSI_NR,
					      &msi_domain_ops,
					      msi->pcie);
		if (!msi->domain) {
			dev_err(msi->pcie_dev,
				"failed to create IRQ domain for MSI\n");
			return -ENOMEM;
		}

		err = devm_request_irq(msi->pcie_dev, msi->irq,
				       brcm_pcie_msi_irq, IRQF_SHARED,
				       msi->irq_chip.name, msi->pcie);

		if (err < 0) {
			dev_err(msi->pcie_dev,
				"failed to request IRQ (%d) for MSI\n",	err);
			goto msi_en_err;
		}

		if (msi->pcie_rev >= BRCM_PCIE_HW_REV_33) {
			msi->intr_base = msi->pcie_base + PCIE_MSI_INTR2_BASE;
			/* This version of PCIe hw has only 32 intr bits
			 * starting at bit position 0. */
			msi->intr_legacy_mask = 0xffffffff;
			msi->intr_legacy_offset = 0x0;
			msi->used = 0x0;

		} else {
			msi->intr_base = msi->pcie_base + PCIE_INTR2_CPU_BASE;
			/* This version of PCIe hw has only 8 intr bits starting
			 * at bit position 24. */
			msi->intr_legacy_mask = 0xff000000;
			msi->intr_legacy_offset = 24;
			msi->used = 0xffffff00;
		}
		msi->working = true;
	}

	/* If we are here, and msi->working is false, it means that we've
	 * already tried and failed to bring up MSI.  Just return 0
	 * since there is nothing to be done. */
	if (!msi->working)
		return 0;

	if (msi->pcie_rev >= BRCM_PCIE_HW_REV_33) {
		/* ffe0 -- least sig 5 bits are 0 indicating 32 msgs
		 * 6540 -- this is our arbitrary unique data value */
		data_val = 0xffe06540;
	} else {
		/* fff8 -- least sig 3 bits are 0 indicating 8 msgs
		 * 6540 -- this is our arbitrary unique data value */
		data_val = 0xfff86540;
	}

	/* Make sure we are not masking MSIs.  Note that MSIs can be masked,
	 * but that occurs on the PCIe EP device */
	__raw_writel(0xffffffff & msi->intr_legacy_mask,
		     msi->intr_base + MASK_CLR);

	msi_lo = lower_32_bits(msi->target_addr);
	msi_hi = upper_32_bits(msi->target_addr);
	/* The 0 bit of PCIE_MISC_MSI_BAR_CONFIG_LO is repurposed to MSI
	 * enable, which we set to 1. */
	__raw_writel(msi_lo | 1, msi->pcie_base + PCIE_MISC_MSI_BAR_CONFIG_LO);
	__raw_writel(msi_hi, msi->pcie_base + PCIE_MISC_MSI_BAR_CONFIG_HI);
	__raw_writel(data_val, msi->pcie_base + PCIE_MISC_MSI_DATA_CONFIG);

	return 0;

msi_en_err:
	irq_domain_remove(msi->domain);
	return err;
}
