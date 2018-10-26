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
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/smp.h>

#include "pci-brcmstb.h"

static const struct dma_map_ops *arch_dma_ops;
static const struct dma_map_ops *brcm_dma_ops_ptr;

static dma_addr_t brcm_to_pci(dma_addr_t addr)
{
	struct of_pci_range *p;

	if (!num_dma_ranges)
		return addr;

	for (p = dma_ranges; p < &dma_ranges[num_dma_ranges]; p++)
		if (addr >= p->cpu_addr && addr < (p->cpu_addr + p->size))
			return addr - p->cpu_addr + p->pci_addr;

	return addr;
}

static dma_addr_t brcm_to_cpu(dma_addr_t addr)
{
	struct of_pci_range *p;

	if (!num_dma_ranges)
		return addr;

	for (p = dma_ranges; p < &dma_ranges[num_dma_ranges]; p++)
		if (addr >= p->pci_addr && addr < (p->pci_addr + p->size))
			return addr - p->pci_addr + p->cpu_addr;

	return addr;
}

static void *brcm_alloc(struct device *dev, size_t size, dma_addr_t *handle,
			gfp_t gfp, unsigned long attrs)
{
	void *ret;

	ret = arch_dma_ops->alloc(dev, size, handle, gfp, attrs);
	if (ret)
		*handle = brcm_to_pci(*handle);
	return ret;
}

static void brcm_free(struct device *dev, size_t size, void *cpu_addr,
		      dma_addr_t handle, unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->free(dev, size, cpu_addr, handle, attrs);
}

static int brcm_mmap(struct device *dev, struct vm_area_struct *vma,
		     void *cpu_addr, dma_addr_t dma_addr, size_t size,
		     unsigned long attrs)
{
	dma_addr = brcm_to_cpu(dma_addr);
	return arch_dma_ops->mmap(dev, vma, cpu_addr, dma_addr, size, attrs);
}

static int brcm_get_sgtable(struct device *dev, struct sg_table *sgt,
			    void *cpu_addr, dma_addr_t handle, size_t size,
			    unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	return arch_dma_ops->get_sgtable(dev, sgt, cpu_addr, handle, size,
				       attrs);
}

static dma_addr_t brcm_map_page(struct device *dev, struct page *page,
				unsigned long offset, size_t size,
				enum dma_data_direction dir,
				unsigned long attrs)
{
	return brcm_to_pci(arch_dma_ops->map_page(dev, page, offset, size,
						  dir, attrs));
}

static void brcm_unmap_page(struct device *dev, dma_addr_t handle,
			    size_t size, enum dma_data_direction dir,
			    unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->unmap_page(dev, handle, size, dir, attrs);
}

static int brcm_map_sg(struct device *dev, struct scatterlist *sgl,
		       int nents, enum dma_data_direction dir,
		       unsigned long attrs)
{
	int i, j;
	struct scatterlist *sg;

	for_each_sg(sgl, sg, nents, i) {
#ifdef CONFIG_NEED_SG_DMA_LENGTH
		sg->dma_length = sg->length;
#endif
		sg->dma_address =
			brcm_dma_ops_ptr->map_page(dev, sg_page(sg), sg->offset,
						   sg->length, dir, attrs);
		if (dma_mapping_error(dev, sg->dma_address))
			goto bad_mapping;
	}
	return nents;

bad_mapping:
	for_each_sg(sgl, sg, i, j)
		brcm_dma_ops_ptr->unmap_page(dev, sg_dma_address(sg),
					     sg_dma_len(sg), dir, attrs);
	return 0;
}

static void brcm_unmap_sg(struct device *dev,
			  struct scatterlist *sgl, int nents,
			  enum dma_data_direction dir,
			  unsigned long attrs)
{
	int i;
	struct scatterlist *sg;

	for_each_sg(sgl, sg, nents, i)
		brcm_dma_ops_ptr->unmap_page(dev, sg_dma_address(sg),
					     sg_dma_len(sg), dir, attrs);
}

static void brcm_sync_single_for_cpu(struct device *dev,
				     dma_addr_t handle, size_t size,
				     enum dma_data_direction dir)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->sync_single_for_cpu(dev, handle, size, dir);
}

static void brcm_sync_single_for_device(struct device *dev,
					dma_addr_t handle, size_t size,
					enum dma_data_direction dir)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->sync_single_for_device(dev, handle, size, dir);
}

static dma_addr_t brcm_map_resource(struct device *dev, phys_addr_t phys,
				    size_t size,
				    enum dma_data_direction dir,
				    unsigned long attrs)
{
	if (arch_dma_ops->map_resource)
		return brcm_to_pci(arch_dma_ops->map_resource
				   (dev, phys, size, dir, attrs));
	return brcm_to_pci((dma_addr_t)phys);
}

static void brcm_unmap_resource(struct device *dev, dma_addr_t handle,
				size_t size, enum dma_data_direction dir,
				unsigned long attrs)
{
	if (arch_dma_ops->unmap_resource)
		arch_dma_ops->unmap_resource(dev, brcm_to_cpu(handle), size,
					     dir, attrs);
}

void brcm_sync_sg_for_cpu(struct device *dev, struct scatterlist *sgl,
			  int nents, enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl, sg, nents, i)
		brcm_dma_ops_ptr->sync_single_for_cpu(dev, sg_dma_address(sg),
						      sg->length, dir);
}

void brcm_sync_sg_for_device(struct device *dev, struct scatterlist *sgl,
			     int nents, enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl, sg, nents, i)
		brcm_dma_ops_ptr->sync_single_for_device(dev,
							 sg_dma_address(sg),
							 sg->length, dir);
}

static int brcm_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return arch_dma_ops->mapping_error(dev, dma_addr);
}

static int brcm_dma_supported(struct device *dev, u64 mask)
{
	if (num_dma_ranges) {
		/*
		 * It is our translated addresses that the EP will "see", so
		 * we check all of the ranges for the largest possible value.
		 */
		int i;

		for (i = 0; i < num_dma_ranges; i++)
			if (dma_ranges[i].pci_addr + dma_ranges[i].size - 1
			    > mask)
				return 0;
		return 1;
	}

	return arch_dma_ops->dma_supported(dev, mask);
}

static int brcm_set_dma_mask(struct device *dev, u64 dma_mask)
{
	return arch_dma_ops->set_dma_mask(dev, dma_mask);
}

#ifdef ARCH_HAS_DMA_GET_REQUIRED_MASK
u64 brcm_get_required_mask)(struct device *dev)
{
	return arch_dma_ops->get_required_mask(dev);
}
#endif

static struct dma_map_ops brcm_dma_ops = {
	.alloc			= brcm_alloc,
	.free			= brcm_free,
	.map_page		= brcm_map_page,
	.unmap_page		= brcm_unmap_page,
	.map_sg			= brcm_map_sg,
	.unmap_sg		= brcm_unmap_sg,
	.sync_single_for_cpu	= brcm_sync_single_for_cpu,
	.sync_single_for_device	= brcm_sync_single_for_device,
	.sync_sg_for_cpu	= brcm_sync_sg_for_cpu,
	.sync_sg_for_device	= brcm_sync_sg_for_device,
#ifdef ARCH_HAS_DMA_GET_REQUIRED_MASK
	.get_required_mask	= brcm_get_required_mask,
#endif
};

static void brcm_set_dma_ops(struct device *dev)
{
	if (IS_ENABLED(CONFIG_ARM64)) {
		/*
		 * We are going to invoke get_dma_ops().  That
		 * function, at this point in time, invokes
		 * get_arch_dma_ops(), and for ARM64 that function
		 * returns a pointer to dummy_dma_ops.  So then we'd
		 * like to call arch_setup_dma_ops(), but that isn't
		 * exported.  Instead, we call of_dma_configure(),
		 * which is exported, and this calls
		 * arch_setup_dma_ops().  Once we do this the call to
		 * get_dma_ops() will work properly because
		 * dev->dma_ops will be set.
		 */
		of_dma_configure(dev, dev->of_node);
	}

	arch_dma_ops = get_dma_ops(dev);
	if (!arch_dma_ops) {
		dev_err(dev, "failed to get arch_dma_ops\n");
		return;
	}

	/* Assign these ops only if they exist for the arch */
	if (arch_dma_ops->mmap)
		brcm_dma_ops.mmap = brcm_mmap;
	if (arch_dma_ops->get_sgtable)
		brcm_dma_ops.get_sgtable = brcm_get_sgtable;
	if (arch_dma_ops->set_dma_mask)
		brcm_dma_ops.set_dma_mask = brcm_set_dma_mask;
	if (arch_dma_ops->dma_supported)
		brcm_dma_ops.dma_supported = brcm_dma_supported;
	if (arch_dma_ops->mapping_error)
		brcm_dma_ops.mapping_error = brcm_mapping_error;

	if (arch_dma_ops->map_resource)
		brcm_dma_ops.map_resource = brcm_map_resource;
	if (arch_dma_ops->unmap_resource)
		brcm_dma_ops.unmap_resource = brcm_unmap_resource;

	dev->archdata.dma_ops = &brcm_dma_ops;
}

static int brcmstb_platform_notifier(struct notifier_block *nb,
				     unsigned long event, void *__dev)
{
	struct device *dev = __dev;
	int ret = NOTIFY_OK;

	brcm_dma_ops_ptr = &brcm_dma_ops;

	switch (event) {
	case BUS_NOTIFY_ADD_DEVICE:
		brcm_set_dma_ops(dev);
		break;
	case BUS_NOTIFY_BOUND_DRIVER:
		/*
		 * Set the device as wakeup capable.  We do this so
		 * that userland may 'echo "enabled" > /sys/.../wakeup'
		 * and let us know that we should not turn of the
		 * voltage regulators when entering S2/S3.
		 */
		device_set_wakeup_capable(dev, true);
		break;
	default:
		ret = NOTIFY_DONE;
		break;
	}

	return ret;
}

static struct notifier_block brcmstb_platform_nb = {
	.notifier_call = brcmstb_platform_notifier,
};

int brcm_register_notifier(void)
{
	return bus_register_notifier(&pci_bus_type, &brcmstb_platform_nb);
}

int brcm_unregister_notifier(void)
{
	return bus_unregister_notifier(&pci_bus_type, &brcmstb_platform_nb);
}
