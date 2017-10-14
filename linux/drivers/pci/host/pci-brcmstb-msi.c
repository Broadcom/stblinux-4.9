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
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/types.h>

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
	char name[BRCM_MSI_NAME_SIZE];

	void __iomem *base;
	struct device *dev;
	struct device_node *dn;
	unsigned int rev;
};

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
	struct brcm_msi *msi = data;
	unsigned long status;

	status = bcm_readl(msi->intr_base + STATUS) & msi->intr_legacy_mask;

	if (!status)
		return IRQ_NONE;

	while (status) {
		unsigned int index = ffs(status) - 1;
		unsigned int irq;

		/* clear the interrupt */
		bcm_writel(1 << index, msi->intr_base + CLR);
		status &= ~(1 << index);

		/* Account for legacy interrupt offset */
		index -= msi->intr_legacy_offset;

		irq = irq_find_mapping(msi->domain, index);
		if (irq) {
			if (msi->used & (1 << index))
				generic_handle_irq(irq);
			else
				dev_info(msi->dev, "unhandled MSI %d\n",
					 index);
		} else {
			/* Unknown MSI, just clear it */
			dev_dbg(msi->dev, "unexpected MSI\n");
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
	data = bcm_readl(msi->base + PCIE_MISC_MSI_DATA_CONFIG);
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
	irq_dispose_mapping(irq);
}

static int brcm_msi_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	struct brcm_msi *msi = domain->host_data;

	irq_set_chip_and_handler(irq, &msi->irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = brcm_msi_map,
};

void brcm_msi_set_regs(struct msi_controller *chip)
{
	u32 data_val, msi_lo, msi_hi;
	struct brcm_msi *msi = to_brcm_msi(chip);

	if (msi->rev >= BRCM_PCIE_HW_REV_33) {
		/* ffe0 -- least sig 5 bits are 0 indicating 32 msgs
		 * 6540 -- this is our arbitrary unique data value
		 */
		data_val = 0xffe06540;
	} else {
		/* fff8 -- least sig 3 bits are 0 indicating 8 msgs
		 * 6540 -- this is our arbitrary unique data value
		 */
		data_val = 0xfff86540;
	}

	/* Make sure we are not masking MSIs.  Note that MSIs can be masked,
	 * but that occurs on the PCIe EP device
	 */
	bcm_writel(0xffffffff & msi->intr_legacy_mask,
	       msi->intr_base + MASK_CLR);

	msi_lo = lower_32_bits(msi->target_addr);
	msi_hi = upper_32_bits(msi->target_addr);
	/* The 0 bit of PCIE_MISC_MSI_BAR_CONFIG_LO is repurposed to MSI
	 * enable, which we set to 1.
	 */
	bcm_writel(msi_lo | 1, msi->base + PCIE_MISC_MSI_BAR_CONFIG_LO);
	bcm_writel(msi_hi, msi->base + PCIE_MISC_MSI_BAR_CONFIG_HI);
	bcm_writel(data_val, msi->base + PCIE_MISC_MSI_DATA_CONFIG);
}

/* THIS FUNCTION NOT UPSTREAMED */
static int fix_msi_controller_prop(struct device_node *dn)
{
	struct property *new = NULL;
	int err = -ENOMEM;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		goto fail;

	new->name = kstrdup("msi-controller", GFP_KERNEL);
	if (!new->name)
		goto fail;

	new->length = sizeof(__be32);
	new->value = kmalloc(new->length, GFP_KERNEL);
	if (!new->value)
		goto fail;

	*(__be32 *) new->value =  cpu_to_be32(1);
	err = of_update_property(dn, new);
	if (err)
		goto fail;
	return 0;

fail:
	kfree(new->name);
	kfree(new);
	return err;
}

void brcm_msi_remove(struct msi_controller *chip)
{
	struct brcm_msi *msi;

	if (!chip)
		return;
	msi = to_brcm_msi(chip);
	if (msi->domain) {
		irq_domain_remove(msi->domain);
		msi->domain = NULL;
	}
}

int brcm_msi_probe(struct platform_device *pdev, struct brcm_info *info)
{
	struct brcm_msi *msi;
	int irq, err;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 1);
	if (irq <= 0) {
		dev_err(&pdev->dev, "cannot map msi intr\n");
		return -ENODEV;
	}

	msi = devm_kzalloc(&pdev->dev, sizeof(struct brcm_msi), GFP_KERNEL);
	if (!msi)
		return -ENOMEM;

	msi->dev = &pdev->dev;
	msi->base = info->base;
	msi->rev =  info->rev;
	msi->dn = pdev->dev.of_node;
	msi->target_addr = info->msi_target_addr;
	msi->irq = irq;
	strlcpy(msi->name, info->name, BRCM_PCIE_NAME_SIZE);
	strlcat(msi->name, BRCM_MSI_TRAILER, BRCM_MSI_NAME_SIZE);

	/* THIS SNIPPET NOT UPSTREAMED */
	if (!of_property_read_bool(msi->dn, "msi-controller")) {
		err = fix_msi_controller_prop(msi->dn);
		if (err)
			goto msi_fail;
	}

	mutex_init(&msi->lock);
	msi->chip.dev = msi->dev;
	msi->chip.of_node = msi->dn;
	err = of_pci_msi_chip_add(&msi->chip);
	if (err)
		goto msi_fail;

	msi->chip.setup_irq = brcm_msi_setup_irq;
	msi->chip.teardown_irq = brcm_msi_teardown_irq;

	/* We have multiple RC controllers.  We may have as many
	 * MSI controllers for them.  We want each to have a
	 * unique name, so we go to the trouble of having an
	 * irq_chip per RC (instead of one for all of them).
	 */
	msi->irq_chip.name = msi->name;
	msi->irq_chip.irq_enable = unmask_msi_irq;
	msi->irq_chip.irq_disable = mask_msi_irq;
	msi->irq_chip.irq_mask = mask_msi_irq;
	msi->irq_chip.irq_unmask = unmask_msi_irq;

	msi->domain =
		irq_domain_add_linear(msi->dn, BRCM_INT_PCI_MSI_NR,
				      &msi_domain_ops, msi);
	if (!msi->domain) {
		dev_err(msi->dev,
			"failed to create IRQ domain for MSI\n");
		err = -ENOMEM;
		goto msi_fail;
	}

	err = devm_request_irq(msi->dev, msi->irq,
			       brcm_pcie_msi_irq, IRQF_SHARED,
			       msi->irq_chip.name, msi);

	if (err < 0) {
		dev_err(msi->dev,
			"failed to request IRQ (%d) for MSI\n",	err);
		goto msi_fail;
	}

	if (msi->rev >= BRCM_PCIE_HW_REV_33) {
		msi->intr_base = msi->base + PCIE_MSI_INTR2_BASE;
		/* This version of PCIe hw has only 32 intr bits
		 * starting at bit position 0.
		 */
		msi->intr_legacy_mask = 0xffffffff;
		msi->intr_legacy_offset = 0x0;
		msi->used = 0x0;

	} else {
		msi->intr_base = msi->base + PCIE_INTR2_CPU_BASE;
		/* This version of PCIe hw has only 8 intr bits starting
		 * at bit position 24.
		 */
		msi->intr_legacy_mask = 0xff000000;
		msi->intr_legacy_offset = 24;
		msi->used = 0xffffff00;
	}

	brcm_msi_set_regs(&msi->chip);

	return 0;

msi_fail:
	if (msi->domain)
		irq_domain_remove(msi->domain);
	kfree(msi);
	return err;
}
