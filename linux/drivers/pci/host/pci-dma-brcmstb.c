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
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include "pci-brcmstb.h"

#if defined(CONFIG_ARM)
#define GEN_DMA_OPS (&arm_dma_ops)
#elif defined(CONFIG_ARM64)
#define GEN_DMA_OPS (&swiotlb_dma_ops)
#elif defined(CONFIG_MIPS)
#define GEN_DMA_OPS mips_dma_map_ops
#endif


static void *brcm_dma_alloc(struct device *dev, size_t size, dma_addr_t *handle,
			    gfp_t gfp, unsigned long attrs)
{
	void *ret;

	ret = GEN_DMA_OPS->alloc(dev, size, handle, gfp, attrs);
	if (ret)
		*handle = brcm_to_pci(*handle);
	return ret;
}

static void brcm_dma_free(struct device *dev, size_t size, void *cpu_addr,
			  dma_addr_t handle, unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	GEN_DMA_OPS->free(dev, size, cpu_addr, handle, attrs);
}

static int brcm_dma_mmap(struct device *dev, struct vm_area_struct *vma,
			 void *cpu_addr, dma_addr_t dma_addr, size_t size,
			 unsigned long attrs)
{
	dma_addr = brcm_to_cpu(dma_addr);
	return GEN_DMA_OPS->mmap(dev, vma, cpu_addr, dma_addr, size, attrs);
}

static int brcm_dma_get_sgtable(struct device *dev, struct sg_table *sgt,
				void *cpu_addr, dma_addr_t handle, size_t size,
				unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	return GEN_DMA_OPS->get_sgtable(dev, sgt, cpu_addr, handle, size,
				       attrs);
}

static dma_addr_t brcm_dma_map_page(struct device *dev, struct page *page,
				    unsigned long offset, size_t size,
				    enum dma_data_direction dir,
				    unsigned long attrs)
{
	return brcm_to_pci(GEN_DMA_OPS->map_page(dev, page, offset, size,
						dir, attrs));
}

static void brcm_dma_unmap_page(struct device *dev, dma_addr_t handle,
				size_t size, enum dma_data_direction dir,
				unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	GEN_DMA_OPS->unmap_page(dev, handle, size, dir, attrs);
}

static void brcm_dma_sync_single_for_cpu(struct device *dev,
					 dma_addr_t handle, size_t size,
					 enum dma_data_direction dir)
{
	handle = brcm_to_cpu(handle);
	GEN_DMA_OPS->sync_single_for_cpu(dev, handle, size, dir);
}

static void brcm_dma_sync_single_for_device(struct device *dev,
					    dma_addr_t handle, size_t size,
					    enum dma_data_direction dir)
{
	handle = brcm_to_cpu(handle);
	GEN_DMA_OPS->sync_single_for_device(dev, handle, size, dir);
}

static int brcm_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	dma_addr = brcm_to_cpu(dma_addr);
	return GEN_DMA_OPS->mapping_error(dev, dma_addr);
}

/* we would like this var to be const but set_dma_ops() disallows it */
static struct dma_map_ops brcm_dma_ops = {
	.alloc = brcm_dma_alloc,
	.free = brcm_dma_free,
	.mmap = brcm_dma_mmap,
	.get_sgtable = brcm_dma_get_sgtable,
	.map_page = brcm_dma_map_page,
	.unmap_page = brcm_dma_unmap_page,
	.sync_single_for_cpu = brcm_dma_sync_single_for_cpu,
	.sync_single_for_device = brcm_dma_sync_single_for_device,

	.mapping_error = brcm_mapping_error,
};

static void brcm_set_dma_ops(struct device *dev)
{
#ifdef ARCH_HAS_DMA_GET_REQUIRED_MASK
	brcm_dma_ops.get_required_mask = GEN_DMA_OPS->get_required_mask;
#endif
	brcm_dma_ops.set_dma_mask = GEN_DMA_OPS->set_dma_mask;
	brcm_dma_ops.dma_supported = GEN_DMA_OPS->dma_supported;
	brcm_dma_ops.sync_sg_for_device = GEN_DMA_OPS->sync_sg_for_device;
	brcm_dma_ops.sync_sg_for_cpu = GEN_DMA_OPS->sync_sg_for_cpu;
	brcm_dma_ops.map_sg = GEN_DMA_OPS->map_sg;
	brcm_dma_ops.unmap_sg = GEN_DMA_OPS->unmap_sg;

#ifdef CONFIG_ARM
	set_dma_ops(dev, &brcm_dma_ops);
#else
	dev->archdata.dma_ops = &brcm_dma_ops;
#endif
}


static int brcmstb_platform_notifier(struct notifier_block *nb,
				     unsigned long event, void *__dev)
{
	struct device *dev = __dev;

	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	brcm_set_dma_ops(dev);
	return NOTIFY_OK;
}

struct notifier_block brcmstb_platform_nb = {
	.notifier_call = brcmstb_platform_notifier,
};

