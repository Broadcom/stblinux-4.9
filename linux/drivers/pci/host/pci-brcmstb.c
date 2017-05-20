/*
 * Copyright (C) 2009 - 2017 Broadcom
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
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/msi.h>
#include <linux/printk.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/module.h>
#include <linux/irqdomain.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/sizes.h>
#include <linux/log2.h>
#include <linux/soc/brcmstb/brcmstb.h>
#include "pci-brcmstb.h"

/* Broadcom PCIE Offsets */
#define PCIE_RC_CFG_PCIE_LINK_CAPABILITY		0x00b8
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL		0x00bc
#define PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL		0x00c8
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2		0x00dc
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1		0x0188
#define PCIE_RC_CFG_PRIV1_ID_VAL3			0x043c
#define PCIE_RC_DL_MDIO_ADDR				0x1100
#define PCIE_RC_DL_MDIO_WR_DATA				0x1104
#define PCIE_RC_DL_MDIO_RD_DATA				0x1108
#define PCIE_MISC_MISC_CTRL				0x4008
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO		0x400c
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI		0x4010
#define PCIE_MISC_RC_BAR1_CONFIG_LO			0x402c
#define PCIE_MISC_RC_BAR1_CONFIG_HI			0x4030
#define PCIE_MISC_RC_BAR2_CONFIG_LO			0x4034
#define PCIE_MISC_RC_BAR2_CONFIG_HI			0x4038
#define PCIE_MISC_RC_BAR3_CONFIG_LO			0x403c
#define PCIE_MISC_RC_BAR3_CONFIG_HI			0x4040
#define PCIE_MISC_MSI_BAR_CONFIG_LO			0x4044
#define PCIE_MISC_MSI_BAR_CONFIG_HI			0x4048
#define PCIE_MISC_MSI_DATA_CONFIG			0x404c
#define PCIE_MISC_PCIE_CTRL				0x4064
#define PCIE_MISC_PCIE_STATUS				0x4068
#define PCIE_MISC_REVISION				0x406c
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT	0x4070
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI		0x4080
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI		0x4084
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG			0x4204
#define PCIE_MSI_INTR2_BASE				0x4500
#define BRCM_NUM_PCI_OUT_WINS		0x4
#define BRCM_MAX_SCB			0x4
#define BRCM_MAX_RANGES			0x6

#define BRCM_MSI_TARGET_ADDR_LT_4GB	0x0fffffffcULL
#define BRCM_MSI_TARGET_ADDR_GT_4GB	0xffffffffcULL

/* Offsets from PCIE_INTR2_CPU_BASE and PCIE_MSI_INTR2_BASE */
#define STATUS				0x0
#define SET				0x4
#define CLR				0x8
#define MASK_STATUS			0xc
#define MASK_SET			0x10
#define MASK_CLR			0x14

#define PCI_BUSNUM_SHIFT		20
#define PCI_SLOT_SHIFT			15
#define PCI_FUNC_SHIFT			12

#define IDX_ADDR(pcie)	\
	((pcie->base) + pcie->reg_offsets[EXT_CFG_INDEX])
#define DATA_ADDR(pcie)	\
	((pcie->base) + pcie->reg_offsets[EXT_CFG_DATA])
#define PCIE_RGR1_SW_INIT_1(pcie) \
	((pcie->base) + pcie->reg_offsets[RGR1_SW_INIT_1])

enum {
	RGR1_SW_INIT_1,
	EXT_CFG_INDEX,
	EXT_CFG_DATA,
};

enum pcie_type {
	BCM7425,
	BCM7435,
	GENERIC,
	BCM7278,
};

struct pcie_cfg_data {
	const int *offsets;
	const enum pcie_type type;
};

static const int pcie_offset_bcm7425[] = {
	[RGR1_SW_INIT_1] = 0x8010,
	[EXT_CFG_INDEX]  = 0x8300,
	[EXT_CFG_DATA]   = 0x8304,
};

static const struct pcie_cfg_data bcm7425_cfg = {
	.offsets	= pcie_offset_bcm7425,
	.type		= BCM7425,
};

static const int pcie_offsets[] = {
	[RGR1_SW_INIT_1] = 0x9210,
	[EXT_CFG_INDEX]  = 0x9000,
	[EXT_CFG_DATA]   = 0x9004,
};

static const struct pcie_cfg_data bcm7435_cfg = {
	.offsets	= pcie_offsets,
	.type		= BCM7435,
};

static const struct pcie_cfg_data generic_cfg = {
	.offsets	= pcie_offsets,
	.type		= GENERIC,
};

static const int pcie_offset_bcm7278[] = {
	[RGR1_SW_INIT_1] = 0xc010,
	[EXT_CFG_INDEX] = 0x9000,
	[EXT_CFG_DATA] = 0x9004,
};

static const struct pcie_cfg_data bcm7278_cfg = {
	.offsets	= pcie_offset_bcm7278,
	.type		= BCM7278,
};

static void __iomem *brcm_pci_map_cfg(struct pci_bus *bus, unsigned int devfn,
				      int where);

static struct pci_ops brcm_pci_ops = {
	.map_bus = brcm_pci_map_cfg,
	.read = pci_generic_config_read32,
	.write = pci_generic_config_write32,
};

/*
 * The roundup_pow_of_two() from log2.h invokes
 * __roundup_pow_of_two(unsigned long), but we really need a
 * such a function to take a native u64 since unsigned long
 * is 32 bits on some configurations.  So we provide this helper
 * function below.
 */
static u64 roundup_pow_of_two_64(u64 n)
{
	return 1ULL << fls64(n - 1);
}


static int brcm_setup_pcie_bridge(int nr, struct pci_sys_data *sys);
static int brcm_map_irq(const struct pci_dev *dev, u8 slot, u8 pin);

struct brcm_window {
	u64 pci_addr;
	u64 size;
	u64 cpu_addr;
	u32 info;
	struct resource pcie_iomem_res;
};

struct brcm_dev_pwr_supply {
	struct list_head node;
	char name[32];
	struct regulator *regulator;
};

static struct of_pci_range *dma_ranges;
static int num_dma_ranges;

/* Internal Bus Controller Information.*/
struct brcm_pcie {
	struct list_head	list;
	void __iomem		*base;
	char			name[BRCM_PCIE_NAME_SIZE];
	bool			suspended;
	struct clk		*clk;
	struct device_node	*dn;
	int			pcie_irq[4];
	int			irq;
	int			num_out_wins;
	bool			ssc;
	int			gen;
	u64			scb_size[BRCM_MAX_SCB];
	struct brcm_window	out_wins[BRCM_NUM_PCI_OUT_WINS];
	struct pci_sys_data	*sys;
	struct device		*dev;
	struct list_head	pwr_supplies;
	bool			broken_pcie_irq_map_dt;
	struct brcm_msi		*msi;
	unsigned		rev;
	unsigned int		num;
	bool			bridge_setup_done;
	const int		*reg_offsets;
	enum pcie_type		type;
};

static int brcm_num_pci_controllers;
static int num_memc;
static void turn_off(struct brcm_pcie *pcie);
static void enter_l23(struct brcm_pcie *pcie);


/***********************************************************************
 * PCIe Bridge setup
 ***********************************************************************/
#if defined(__BIG_ENDIAN)
#define	DATA_ENDIAN		2	/* PCI->DDR inbound accesses */
#define MMIO_ENDIAN		2	/* CPU->PCI outbound accesses */
#else
#define	DATA_ENDIAN		0
#define MMIO_ENDIAN		0
#endif

#ifndef DMA_ERROR_CODE
#define DMA_ERROR_CODE  (~(dma_addr_t)0x0)
#endif

struct brcm_msi *brcm_pcie_to_msi(struct brcm_pcie *pcie)
{
	return pcie->msi;
}

/* This is to convert the size of the inbound bar region to the
 * non-liniear values of PCIE_X_MISC_RC_BAR[123]_CONFIG_LO.SIZE */
int encode_ibar_size(u64 size)
{
	int log2_in = ilog2(size);

	if (log2_in >= 12 && log2_in <= 15)
		/* Covers 4KB to 32KB (inclusive) */
		return (log2_in - 12) + 0x1c;
	else if (log2_in >= 16 && log2_in <= 37)
		/* Covers 64KB to 32GB, (inclusive) */
		return log2_in - 15;
	/* Something is awry so disable */
	return 0;
}


dma_addr_t brcm_to_pci(dma_addr_t addr)
{
	struct of_pci_range *p;

	if (!num_dma_ranges || addr == DMA_ERROR_CODE)
		return addr;

	for (p = dma_ranges; p < &dma_ranges[num_dma_ranges]; p++) {
		if ((u64)addr >= p->cpu_addr
		    && (u64)addr < p->cpu_addr + p->size)
			return (u64)addr - p->cpu_addr + p->pci_addr;
	}
	return addr;
}

dma_addr_t brcm_to_cpu(dma_addr_t addr)
{
	struct of_pci_range *p;

	if (!num_dma_ranges || addr == DMA_ERROR_CODE)
		return addr;
	for (p = dma_ranges; p < &dma_ranges[num_dma_ranges]; p++) {
		if ((u64)addr >= p->pci_addr
		    && (u64)addr < p->pci_addr + p->size)
			return (u64)addr - p->pci_addr + p->cpu_addr;
	}
	return addr;
}

/* negative return value indicates error */
static int mdio_read(void __iomem *base, u8 phyad, u8 regad)
{
	const int TRYS = 10;
	u32 data = ((phyad & 0xf) << 16)
		| (regad & 0x1f)
		| 0x100000;
	int i = 0;

	__raw_writel(data, base + PCIE_RC_DL_MDIO_ADDR);
	__raw_readl(base + PCIE_RC_DL_MDIO_ADDR);

	data = __raw_readl(base + PCIE_RC_DL_MDIO_RD_DATA);
	while (!(data & 0x80000000) && ++i < TRYS) {
		udelay(10);
		data = __raw_readl(base + PCIE_RC_DL_MDIO_RD_DATA);
	}

	return (data & 0x80000000) ? (data & 0xffff) : -EIO;
}


/* negative return value indicates error */
static int mdio_write(void __iomem *base, u8 phyad, u8 regad, u16 wrdata)
{
	u32 data = ((phyad & 0xf) << 16) | (regad & 0x1f);
	const int TRYS = 10;
	int i = 0;

	__raw_writel(data, base + PCIE_RC_DL_MDIO_ADDR);
	__raw_readl(base + PCIE_RC_DL_MDIO_ADDR);
	__raw_writel(0x80000000 | wrdata, base + PCIE_RC_DL_MDIO_WR_DATA);

	data = __raw_readl(base + PCIE_RC_DL_MDIO_WR_DATA);
	while ((data & 0x80000000) && ++i < TRYS) {
		udelay(10);
		data = __raw_readl(base + PCIE_RC_DL_MDIO_WR_DATA);
	}

	return (data & 0x80000000) ? -EIO : 0;
}


static void wr_fld(void __iomem *p, u32 mask, int shift, u32 val)
{
	u32 reg = __raw_readl(p);

	reg = (reg & ~mask) | (val << shift);
	__raw_writel(reg, p);
}


static void wr_fld_rb(void __iomem *p, u32 mask, int shift, u32 val)
{
	wr_fld(p, mask, shift, val);
	(void) __raw_readl(p);
}


/* configures device for ssc mode; negative return value indicates error */
static int set_ssc(void __iomem *base)
{
	int tmp;
	u16 wrdata;

	tmp = mdio_write(base, 0, 0x1f, 0x1100);
	if (tmp < 0)
		return tmp;

	tmp = mdio_read(base, 0, 2);
	if (tmp < 0)
		return tmp;

	wrdata = ((u16)tmp & 0x3fff) | 0xc000;
	tmp = mdio_write(base, 0, 2, wrdata);
	if (tmp < 0)
		return tmp;

	mdelay(1);
	tmp = mdio_read(base, 0, 1);
	if (tmp < 0)
		return tmp;

	return 0;
}


/* returns 0 if in ssc mode, 1 if not, <0 on error */
static int is_ssc(void __iomem *base)
{
	int tmp = mdio_write(base, 0, 0x1f, 0x1100);

	if (tmp < 0)
		return tmp;
	tmp = mdio_read(base, 0, 1);
	if (tmp < 0)
		return tmp;
	return (tmp & 0xc00) == 0xc00 ? 0 : 1;
}


/* limits operation to a specific generation (1, 2, or 3) */
static void set_gen(void __iomem *base, int gen)
{
	wr_fld(base + PCIE_RC_CFG_PCIE_LINK_CAPABILITY, 0xf, 0, gen);
	wr_fld(base + PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2, 0xf, 0, gen);
}


static void set_pcie_outbound_win(struct brcm_pcie *pcie, unsigned win,
				  u64 cpu_addr, u64 pci_addr, u64 size)
{
	void __iomem *base = pcie->base;
	u64 cpu_addr_mb, limit_addr_mb;
	u32 tmp;

	/* Set the base of the pci_addr window */
	__raw_writel(lower_32_bits(pci_addr) + MMIO_ENDIAN,
		     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO+(win*8));
	__raw_writel(upper_32_bits(pci_addr),
		     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI+(win*8));

	cpu_addr_mb = cpu_addr >> 20;
	limit_addr_mb = (cpu_addr + size - 1) >> 20;

	/* Write the cpu addr+limit low register */
	tmp = (u32)((cpu_addr_mb & 0xfff) << 4)
		| (u32)((limit_addr_mb & 0xfff) << 20);
	__raw_writel(tmp,
		     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT+(win*4));

	/* Write the cpu addr high register */
	if (pcie->type != BCM7435 && pcie->type != BCM7425) {
		tmp = (u32)(cpu_addr_mb >> 12);
		__raw_writel(tmp,
			     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI
			     + (win*8));

	/* Write the cpu limit high register */
		tmp = (u32)(limit_addr_mb >> 12);
		__raw_writel(tmp,
			     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI
			     + (win*8));
	}
}


static int is_pcie_link_up(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	u32 val = __raw_readl(base + PCIE_MISC_PCIE_STATUS);

	return  ((val & 0x30) == 0x30) ? 1 : 0;
}

static inline void brcm_pcie_bridge_sw_init_set(struct brcm_pcie *pcie,
						unsigned int val)
{
	unsigned int offset = 0;

	if (pcie->type != BCM7278)
		wr_fld_rb(PCIE_RGR1_SW_INIT_1(pcie), 0x2, 1, val);
	else {
		/* The two PCIE instance on 7278 are not even consistent with
		 * respect to each other for internal offsets, here we offset
		 * by 0x14000 + RGR1_SW_INIT_1's relative offset to account for
		 * that.
		 */
		offset = pcie->num ? 0x14010
		    : pcie->reg_offsets[RGR1_SW_INIT_1];
		wr_fld_rb(pcie->base + offset, 0x1, 0, val);
	}
}

static inline void brcm_pcie_perst_set(struct brcm_pcie *pcie,
				       unsigned int val)
{
	if (pcie->type != BCM7278)
		wr_fld_rb(PCIE_RGR1_SW_INIT_1(pcie), 0x00000001, 0, val);
	else
		/* Assert = 0, de-assert = 1 on 7278 */
		wr_fld_rb(pcie->base + PCIE_MISC_PCIE_CTRL, 0x4, 2, !val);
}


static void brcm_pcie_setup_early(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	unsigned int scb_size_val;
	u64 rc_bar2_size = 0, rc_bar2_offset = 0, total_mem_size = 0;
	u64 msi_target_addr;
	int i;

	/* reset the bridge and the endpoint device */
	/* field: PCIE_BRIDGE_SW_INIT = 1 */
	brcm_pcie_bridge_sw_init_set(pcie, 1);

	/* field: PCIE_SW_PERST = 1, on 7278, we start de-asserted already */
	if (pcie->type != BCM7278)
		brcm_pcie_perst_set(pcie, 1);

	/* delay 100us */
	udelay(100);

	/* take the bridge out of reset */
	/* field: PCIE_BRIDGE_SW_INIT = 0 */
	brcm_pcie_bridge_sw_init_set(pcie, 0);

	/* Grab the PCIe hw revision number */
	pcie->rev = __raw_readl(base + PCIE_MISC_REVISION) & 0xffff;
	brcm_set_msi_rev(pcie->msi, pcie->rev);

	/* enable SCB_MAX_BURST_SIZE | CSR_READ_UR_MODE | SCB_ACCESS_EN */
	if (pcie->type == GENERIC || pcie->type == BCM7278)
		__raw_writel(0x81e03000, base + PCIE_MISC_MISC_CTRL);
	else
		__raw_writel(0x00103000, base + PCIE_MISC_MISC_CTRL);

	for (i = 0; i < pcie->num_out_wins; i++) {
		struct brcm_window *w = &pcie->out_wins[i];

		set_pcie_outbound_win(pcie, i, w->cpu_addr, w->pci_addr,
				      w->size);
	}

	/* Set up inbound memory view for the EP (called RC_BAR2,
	 * not to be confused with the BARs that are advertised by
	 * the EP). */
	for (i = 0; i < num_memc; i++)
		total_mem_size += pcie->scb_size[i];

	/* The PCI host controller by design must set the inbound
	 * viewport to be a contiguous arrangement of all of the
	 * system's memory.  In addition, its size mut be a power of
	 * two.  Further, the MSI target address must NOT be placed
	 * inside this region, as the decoding logic will consider its
	 * address to be inbound memory traffic.  To further
	 * complicate matters, the viewport must start on a
	 * pci-address that is aligned on a multiple of its size.
	 * If a portion of the viewport does not represent system
	 * memory -- e.g. 3GB of memory requires a 4GB viewport --
	 * we can map the outbound memory in or after 3GB and even
	 * though the viewport will overlap the outbound memory
	 * the controller will know to send outbound memory downstream
	 * and everything else upstream. */
	rc_bar2_size = roundup_pow_of_two_64(total_mem_size);

	if (dma_ranges) {
		/* The best-case scenario is to place the inbound
		 * region in the first 4GB of pci-space, as some
		 * legacy devices can only address 32bits.
		 * We would also like to put the MSI under 4GB
		 * as well, since some devices require a 32bit
		 * MSI target address. */
		if (total_mem_size <= 0xc0000000ULL &&
		    rc_bar2_size <= 0x100000000ULL) {

			rc_bar2_offset = 0;
			/* If the viewport is less then 4GB we can fit
			 * the MSI target address under 4GB. Otherwise
			 * put it right below 64GB. */
			msi_target_addr =
				(rc_bar2_size == 0x100000000ULL)
				? BRCM_MSI_TARGET_ADDR_GT_4GB
				: BRCM_MSI_TARGET_ADDR_LT_4GB;

		} else {
			/* The system memory is 4GB or larger so we
			 * cannot start the inbound region at location
			 * 0 (since we have to allow some space for
			 * outbound memory @ 3GB).  So instead we
			 * start it at the 1x multiple of its size */
			rc_bar2_offset = rc_bar2_size;

			/* Since we are starting the viewport at 4GB or
			 * higher, put the MSI target address below 4GB */
			msi_target_addr = BRCM_MSI_TARGET_ADDR_LT_4GB;
		}

	} else {
		/* Set simple configuration based on legacy property
		 * brcm,log2-scb-sizes.  We always start the viewport at
		 * address 0, and set the MSI target address accordingly. */
		rc_bar2_offset = 0;
		msi_target_addr = (rc_bar2_size >= 0x100000000ULL)
			? BRCM_MSI_TARGET_ADDR_GT_4GB
			: BRCM_MSI_TARGET_ADDR_LT_4GB;
	}

	brcm_set_msi_target_addr(pcie->msi, msi_target_addr);

	__raw_writel(lower_32_bits(rc_bar2_offset)
		     | encode_ibar_size(rc_bar2_size),
		     base + PCIE_MISC_RC_BAR2_CONFIG_LO);
	__raw_writel(upper_32_bits(rc_bar2_offset),
		     base + PCIE_MISC_RC_BAR2_CONFIG_HI);

	/* field: SCB0_SIZE, default = 0xf (1 GB) */
	scb_size_val = pcie->scb_size[0]
		? ilog2(pcie->scb_size[0]) - 15 : 0xf;
	wr_fld(base + PCIE_MISC_MISC_CTRL, 0xf8000000, 27, scb_size_val);

	/* field: SCB1_SIZE, default = 0xf (1 GB) */
	if (num_memc > 1) {
		scb_size_val = pcie->scb_size[1]
			? ilog2(pcie->scb_size[1]) - 15 : 0xf;
		wr_fld(base + PCIE_MISC_MISC_CTRL, 0x07c00000,
		       22, scb_size_val);
	}

	/* field: SCB2_SIZE, default = 0xf (1 GB) */
	if (num_memc > 2) {
		scb_size_val = pcie->scb_size[2]
			? ilog2(pcie->scb_size[2]) - 15 : 0xf;
		wr_fld(base + PCIE_MISC_MISC_CTRL, 0x0000001f,
		       0, scb_size_val);
	}

	/* disable the PCIE->GISB memory window (RC_BAR1) */
	__raw_writel(0x00000000, base + PCIE_MISC_RC_BAR1_CONFIG_LO);

	/* disable the PCIE->SCB memory window (RC_BAR3) */
	__raw_writel(0x00000000, base + PCIE_MISC_RC_BAR3_CONFIG_LO);

	if (!pcie->suspended) {
		/* clear any interrupts we find on boot */
		__raw_writel(0xffffffff, base + PCIE_INTR2_CPU_BASE + CLR);
		(void) __raw_readl(base + PCIE_INTR2_CPU_BASE + CLR);
	}

	/* Mask all interrupts since we are not handling any yet */
	__raw_writel(0xffffffff, base + PCIE_INTR2_CPU_BASE + MASK_SET);
	(void) __raw_readl(base + PCIE_INTR2_CPU_BASE + MASK_SET);

	if (pcie->gen)
		set_gen(base, pcie->gen);

	/* take the EP device out of reset */
	/* field: PCIE_SW_PERST = 0 */
	brcm_pcie_perst_set(pcie, 0);
}


static int brcm_setup_pcie_bridge(int nr, struct pci_sys_data *sys)
{
	struct brcm_pcie *pcie = sys->private_data;
	void __iomem *base = pcie->base;
	const int limit = pcie->suspended ? 1000 : 100;
	struct clk *clk;
	unsigned status;
	static const char *link_speed[4] = { "???", "2.5", "5.0", "8.0" };
	int i, j, ret;
	bool ssc_good = false;

	pcie->sys = sys;

	/* Give the RC/EP time to wake up, before trying to configure RC.
	 * Intermittently check status for link-up, up to a total of 100ms
	 * when we don't know if the device is there, and up to 1000ms if
	 * we do know the device is there. */
	for (i = 1, j = 0; j < limit && !is_pcie_link_up(pcie); j += i, i = i*2)
		mdelay(i + j > limit ? limit - j : i);

	if (!is_pcie_link_up(pcie)) {
		dev_info(pcie->dev, "link down\n");
		goto FAIL;
	}

	/* Attempt to enable MSI if we have an interrupt for it. */
	if (pcie->msi) {
		ret = brcm_pcie_enable_msi(pcie->msi, nr, pcie->suspended);
		if (ret < 0) {
			dev_err(pcie->dev, "failed to enable MSI support: %d\n",
				ret);
		}
	}

	/* For config space accesses on the RC, show the right class for
	 * a PCI-PCI bridge */
	wr_fld_rb(base + PCIE_RC_CFG_PRIV1_ID_VAL3, 0x00ffffff, 0, 0x060400);

	if (!pcie->suspended)
		for (i = 0; i < pcie->num_out_wins; i++)
			pci_add_resource_offset(&sys->resources,
				&pcie->out_wins[i].pcie_iomem_res,
				pcie->out_wins[i].pcie_iomem_res.start
					- pcie->out_wins[i].pci_addr);

	status = __raw_readl(base + PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL);

	if (pcie->ssc) {
		if (set_ssc(base))
			dev_err(pcie->dev,
				"mdio rd/wt fail during ssc config\n");

		ret = is_ssc(base);
		if (ret == 0) {
			ssc_good = true;
		} else {
			if (ret < 0)
				dev_err(pcie->dev,
					"mdio rd/wt fail during ssc query\n");
			dev_err(pcie->dev, "failed to enter SSC mode\n");
		}
	}

	dev_info(pcie->dev, "link up, %s Gbps x%u %s\n",
		 link_speed[((status & 0x000f0000) >> 16) & 0x3],
		 (status & 0x03f00000) >> 20, ssc_good ? "(SSC)" : "(!SSC)");

	/* Enable configuration request retry (see pci_scan_device()) */
	/* field RC_CRS_EN = 1 */
	wr_fld(base + PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL, 0x00000010, 4, 1);

	/* PCIE->SCB endian mode for BAR */
	/* field ENDIAN_MODE_BAR2 = DATA_ENDIAN */
	wr_fld_rb(base + PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1, 0x0000000c, 2,
		  DATA_ENDIAN);

	/* Refclk from RC should be gated with CLKREQ# input when ASPM L0s,L1
	 * is enabled =>  setting the CLKREQ_DEBUG_ENABLE field to 1. */
	wr_fld_rb(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG, 0x00000002, 1, 1);

	/* Add bogus IO resource structure so that pcibios_init_resources()
	 * does not allocate the same IO region for different domains */
	sys->io_res.start = (brcm_num_pci_controllers - 1) * SZ_64K ?  :
		PCIBIOS_MIN_IO;
	sys->io_res.end = (brcm_num_pci_controllers * SZ_64K) - 1;
	sys->io_res.flags = IORESOURCE_IO;
	sys->io_res.name = "brcmstb bogus IO";
	pci_add_resource(&sys->resources, &sys->io_res);

	pcie->bridge_setup_done = true;

	return 1;
FAIL:
	if (IS_ENABLED(CONFIG_PM))
		turn_off(pcie);

	clk = pcie->clk;
	if (pcie->suspended)
		clk_disable(clk);
	else {
		clk_disable_unprepare(clk);
		clk_put(clk);
	}

	pcie->bridge_setup_done = false;

	return 0;
}


/*
 * syscore device to handle PCIe bus suspend and resume
 */

static void turn_off(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	/* Reset endpoint device */
	brcm_pcie_perst_set(pcie, 1);
	/* deassert request for L23 in case it was asserted */
	wr_fld_rb(base + PCIE_MISC_PCIE_CTRL, 0x1, 0, 0);
	/* SERDES_IDDQ = 1 */
	wr_fld_rb(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG, 0x08000000,
		  27, 1);
	/* Shutdown PCIe bridge */
	brcm_pcie_bridge_sw_init_set(pcie, 1);
}


static void enter_l23(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	int timeout = 1000;
	int l23;

	/* assert request for L23 */
	wr_fld_rb(base + PCIE_MISC_PCIE_CTRL, 0x1, 0, 1);
	do {
		/* poll L23 status */
		l23 = __raw_readl(base + PCIE_MISC_PCIE_STATUS) & (1 << 6);
	} while (--timeout && !l23);

	if (!timeout)
		dev_err(pcie->dev, "failed to enter L23\n");
}


static int brcm_pcie_suspend(struct device *dev)
{
	struct brcm_pcie *pcie = dev_get_drvdata(dev);
	struct brcm_dev_pwr_supply *supply;
	struct list_head *pos;

	if (!pcie->bridge_setup_done)
		return 0;

	enter_l23(pcie);
	turn_off(pcie);
	clk_disable(pcie->clk);
	list_for_each(pos, &pcie->pwr_supplies) {
		supply = list_entry(pos, struct brcm_dev_pwr_supply,
				    node);
		if (regulator_disable(supply->regulator))
			pr_debug("Unable to turn off %s supply.\n",
				 supply->name);

	}
	pcie->suspended = true;

	return 0;
}

static int brcm_pcie_resume(struct device *dev)
{
	struct brcm_pcie *pcie = dev_get_drvdata(dev);
	struct brcm_dev_pwr_supply *supply;
	struct list_head *pos;
	void __iomem *base;

	if (!pcie->bridge_setup_done)
		return 0;

	base = pcie->base;
	list_for_each(pos, &pcie->pwr_supplies) {
		supply = list_entry(pos, struct brcm_dev_pwr_supply,
				    node);
		if (regulator_enable(supply->regulator))
			pr_debug("Unable to turn on %s supply.\n",
				 supply->name);
	}
	clk_enable(pcie->clk);

	/* Take bridge out of reset so we can access the SERDES reg */
	brcm_pcie_bridge_sw_init_set(pcie, 0);

	/* SERDES_IDDQ = 0 */
	wr_fld_rb(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG, 0x08000000,
		  27, 0);
	/* wait for serdes to be stable */
	udelay(100);

	brcm_pcie_setup_early(pcie);

	brcm_setup_pcie_bridge(pcie->num, pcie->sys);
	pcie->suspended = false;

	return 0;
}

/***********************************************************************
 * Read/write PCI configuration registers
 ***********************************************************************/
static int cfg_index(int busnr, int devfn, int reg)
{
	return ((PCI_SLOT(devfn) & 0x1f) << PCI_SLOT_SHIFT)
		| ((PCI_FUNC(devfn) & 0x07) << PCI_FUNC_SHIFT)
		| (busnr << PCI_BUSNUM_SHIFT)
		| (reg & ~3);
}


static void __iomem *brcm_pci_map_cfg(struct pci_bus *bus, unsigned int devfn,
				      int where)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct brcm_pcie *pcie = sys->private_data;
	void __iomem *base = pcie->base;
	bool rc_access = pci_is_root_bus(bus);
	int idx;

	if (!is_pcie_link_up(pcie))
		return NULL;

	base = pcie->base;
	idx = cfg_index(bus->number, devfn, where);

	__raw_writel(idx, IDX_ADDR(pcie));

	if (rc_access) {
		if (PCI_SLOT(devfn))
			return NULL;
		return base + (where & ~3);
	}

	return DATA_ADDR(pcie);
}


/***********************************************************************
 * PCI slot to IRQ mappings (aka "fixup")
 ***********************************************************************/
static int brcm_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pci_sys_data *sys = dev->bus->sysdata;
	struct brcm_pcie *pcie = sys->private_data;

	if (pcie) {
		if (!pcie->broken_pcie_irq_map_dt)
			return of_irq_parse_and_map_pci(dev, slot, pin);
		if ((pin - 1) > 3)
			return 0;
		return pcie->pcie_irq[pin - 1];
	}
	return 0;
}


/***********************************************************************
 * Per-device initialization
 ***********************************************************************/
static void __attribute__((__section__("pci_fixup_early")))
brcm_pcibios_fixup(struct pci_dev *dev)
{
	struct pci_sys_data *sys = dev->bus->sysdata;
	struct brcm_pcie *pcie = sys->private_data;
	int slot = PCI_SLOT(dev->devfn);

	if (pci_is_root_bus(dev->bus)) {
		/* Set the root pci_dev's device node */
		if (!dev->dev.of_node)
			dev->dev.of_node = pcie->dn;
		/* Set the root bus's msi value */
		if (IS_ENABLED(CONFIG_PCI_MSI))
			dev->bus->msi = brcm_get_msi_chip(pcie->msi);
	}
	dev_info(pcie->dev,
		 "found device %04x:%04x on bus %d (%s), slot %d (irq %d)\n",
		 dev->vendor, dev->device, dev->bus->number, pcie->name,
		 slot, brcm_map_irq(dev, slot, 1));
}
DECLARE_PCI_FIXUP_EARLY(PCI_ANY_ID, PCI_ANY_ID, brcm_pcibios_fixup);

static int brcm_pci_dma_range_parser_init(struct of_pci_range_parser *parser,
					  struct device_node *node)
{
	const int na = 3, ns = 2;
	int rlen;

	parser->node = node;
	parser->pna = of_n_addr_cells(node);
	parser->np = parser->pna + na + ns;

	parser->range = of_get_property(node, "dma-ranges", &rlen);
	if (parser->range == NULL)
		return -ENOENT;

	parser->end = parser->range + rlen / sizeof(__be32);

	return 0;
}

static const struct of_device_id brcm_pci_match[] = {
	{ .compatible = "brcm,bcm7425-pcie", .data = &bcm7425_cfg },
	{ .compatible = "brcm,bcm7435-pcie", .data = &bcm7435_cfg },
	{ .compatible = "brcm,bcm7278-pcie", .data = &bcm7278_cfg },
	{ .compatible = "brcm,pci-plat-dev", .data = &generic_cfg },
	{},
};
MODULE_DEVICE_TABLE(of, brcm_pci_match);

/***********************************************************************
 * PCI Platform Driver
 ***********************************************************************/
static int brcm_pci_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node, *mdn;
	const struct of_device_id *of_id;
	const struct pcie_cfg_data *data;
	const u32 *imap_prop;
	int len, i, irq_offset, rlen, pna, np, ret;
	struct brcm_pcie *pcie;
	struct resource *r;
	const u32 *ranges, *log2_scb_sizes;
	struct of_pci_range_parser parser;
	void __iomem *base;
	struct hw_pci hw;
	u32 tmp;
	int supplies;
	const char *name;
	struct brcm_dev_pwr_supply *supply;

	pcie = devm_kzalloc(&pdev->dev, sizeof(struct brcm_pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	of_id = of_match_node(brcm_pci_match, dn);
	if (!of_id) {
		pr_err("failed to look up compatible string\n");
		return -EINVAL;
	}

	data = of_id->data;
	pcie->reg_offsets = data->offsets;
	pcie->type = data->type;

	platform_set_drvdata(pdev, pcie);

	INIT_LIST_HEAD(&pcie->pwr_supplies);
	supplies = of_property_count_strings(dn, "supply-names");
	if (supplies <= 0)
		supplies = 0;

	for (i = 0; i < supplies; i++) {
		if (of_property_read_string_index(dn, "supply-names", i,
						  &name))
			continue;
		supply = devm_kzalloc(&pdev->dev, sizeof(*supply), GFP_KERNEL);
		if (!supply)
			return -ENOMEM;
		strncpy(supply->name, name, sizeof(supply->name));
		supply->name[sizeof(supply->name) - 1] = '\0';
		supply->regulator = devm_regulator_get(&pdev->dev, name);
		if (IS_ERR(supply->regulator)) {
			dev_err(&pdev->dev, "Unable to get %s supply, err=%d\n",
				name, (int)PTR_ERR(supply->regulator));
			continue;
		}
		if (regulator_enable(supply->regulator))
			dev_err(&pdev->dev, "Unable to enable %s supply.\n",
				name);
		list_add_tail(&supply->node, &pcie->pwr_supplies);
	}

	/* 'num_memc' will be set only by the first controller, and all
	 * other controllers will use the value set by the first. */
	if (num_memc == 0)
		for_each_compatible_node(mdn, NULL, "brcm,brcmstb-memc")
			if (of_device_is_available(mdn))
				num_memc++;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -EINVAL;

	base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(base))
		return PTR_ERR(base);

	imap_prop = of_get_property(dn, "interrupt-map", &len);

	if (imap_prop == NULL) {
		dev_err(&pdev->dev, "missing interrupt-map\n");
		return -EINVAL;
	}

	if (len == sizeof(u32) * 4 * 7 &&
	    (pcie->type == GENERIC || pcie->type == BCM7278)) {
		/* broken method for getting INT{ABCD} */
		dev_info(&pdev->dev, "adjusting to legacy (broken) pcie DT\n");
		pcie->broken_pcie_irq_map_dt = true;
		irq_offset = irq_of_parse_and_map(dn, 0);
		for (i = 0; i < 4 && i*4 < len; i++)
			pcie->pcie_irq[i] = irq_offset
				+ of_read_number(imap_prop + (i * 7 + 5), 1);
	}

	snprintf(pcie->name,
		 sizeof(pcie->name)-1, "PCIe%d", brcm_num_pci_controllers);
	pcie->num = brcm_num_pci_controllers;
	pcie->suspended = false;
	pcie->clk = of_clk_get_by_name(dn, "sw_pcie");
	if (IS_ERR(pcie->clk)) {
		dev_err(&pdev->dev, "could not get clock\n");
		pcie->clk = NULL;
	}
	ret = clk_prepare_enable(pcie->clk);
	if (ret) {
		dev_err(&pdev->dev, "could not enable clock\n");
		return ret;
	}
	pcie->dn = dn;
	pcie->base = base;
	pcie->dev = &pdev->dev;
	pcie->dev->of_node = dn;
	pcie->gen = 0;

	ret = of_property_read_u32(dn, "brcm,gen", &tmp);
	if (ret == 0) {
		if (tmp > 0 && tmp < 3)
			pcie->gen = (int) tmp;
		else
			dev_warn(pcie->dev, "bad DT value for prop 'brcm,gen");
	} else if (ret != -EINVAL) {
		dev_warn(pcie->dev, "error reading DT prop 'brcm,gen");
	}

	pcie->ssc = of_property_read_bool(dn, "brcm,ssc");

	/* Get the value for the log2 of the scb sizes.  Subtract 15 from
	 * each because the target register field has 0==disabled and 1==64KB.
	 */
	log2_scb_sizes = of_get_property(dn, "brcm,log2-scb-sizes", &rlen);
	if (log2_scb_sizes != NULL) {
		for (i = 0; i < rlen/4; i++) {
			pcie->scb_size[i] =
				1ULL << of_read_number(log2_scb_sizes + i, 1);
		}
	}

	/* Parse dma-ranges property if present.  If there are multiple
	 * PCI controllers, we only have to parse from one of them since
	 * the others will have an identical mapping. */
	if (!dma_ranges && !brcm_pci_dma_range_parser_init(&parser, dn)) {
		dma_ranges = devm_kzalloc(&pdev->dev,
					  sizeof(struct of_pci_range) *
					  BRCM_MAX_RANGES, GFP_KERNEL);
		if (!dma_ranges)
			return -ENOMEM;

		i = 0;
		while (of_pci_range_parser_one(&parser, dma_ranges + i)) {
			struct of_pci_range *dma = dma_ranges + i;

			if (i >= BRCM_MAX_RANGES) {
				dev_err(pcie->dev, "too may dma-ranges");
				return -EINVAL;
			}

			dev_dbg(pcie->dev,
				"mem@[%9llx...%9llx] => pci@[%9llx...%9llx]\n",
				dma->cpu_addr,
				dma->cpu_addr + dma->size - 1,
				dma->pci_addr,
				dma->pci_addr + dma->size - 1);

			num_dma_ranges++;
			i++;
		}
	}

	if (dma_ranges || !log2_scb_sizes) {
		for (i = 0; i < num_memc; i++) {
			u64 size = brcmstb_memory_memc_size(i);

			if (size == (u64) -1) {
				dev_err(pcie->dev, "cannot get memc%d size", i);
				return -EINVAL;
			}
			pcie->scb_size[i] = roundup_pow_of_two_64(size);
		}
	}

	if (!pcie->broken_pcie_irq_map_dt) {
		ret = irq_of_parse_and_map(pdev->dev.of_node, 0);
		if (ret == 0) {
			dev_warn(pcie->dev, "cannot get pcie interrupt\n");
			/* keep going, as we don't use this yet */
		} else {
			pcie->irq = ret;
		}
	}

	ranges = of_get_property(dn, "ranges", &rlen);
	if (ranges == NULL) {
		dev_err(pcie->dev, "no ranges property in dev tree.\n");
		return -EINVAL;
	}
	/* set up CPU->PCIE memory windows (max of four) */
	pna = of_n_addr_cells(dn);
	np = pna + 5;

	pcie->num_out_wins = rlen / (np * 4);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = irq_of_parse_and_map(pdev->dev.of_node, 1);
		if (ret) {
			pcie->msi = brcm_alloc_init_msi(pcie, &pdev->dev,
							pcie->base, ret,
							pcie->dn, pcie->name);
			if (!pcie->msi)
				return -ENOMEM;
		} else {
			dev_warn(pcie->dev, "cannot get msi intr; MSI disabled\n");
		}
	}

	for (i = 0; i < pcie->num_out_wins; i++) {
		struct brcm_window *w = &pcie->out_wins[i];
		bool pref;

		w->info = (u32) of_read_ulong(ranges + 0, 1);
		w->pci_addr = of_read_number(ranges + 1, 2);
		w->cpu_addr = of_translate_address(dn, ranges + 3);
		w->size = of_read_number(ranges + pna + 3, 2);
		ranges += np;

		pref = (w->info & 0x40000000) == 0x40000000;


		if (pref) {
			w->pcie_iomem_res.name = "External PCIe MEM (pref)";
			w->pcie_iomem_res.flags	= IORESOURCE_MEM
				| IORESOURCE_PREFETCH;
		} else {
			w->pcie_iomem_res.name = "External PCIe MEM";
			w->pcie_iomem_res.flags	= IORESOURCE_MEM;
		}

		w->pcie_iomem_res.start	= w->cpu_addr;
		w->pcie_iomem_res.end	= w->cpu_addr + w->size - 1;

		/* Request memory region resources. */
		if (request_resource(&iomem_resource, &w->pcie_iomem_res)) {
			dev_err(&pdev->dev,
				"request PCIe memory resource failed\n");
			return -EIO;
		}
	}

	/*
	 * Starts PCIe link negotiation immediately at kernel boot time.  The
	 * RC is supposed to give the endpoint device 100ms to settle down
	 * before attempting configuration accesses.  So we let the link
	 * negotiation happen in the background instead of busy-waiting.
	 */
	brcm_pcie_setup_early(pcie);
	brcm_num_pci_controllers++;

	memset(&hw, 0, sizeof(hw));

	hw.nr_controllers = 1;
	hw.private_data = (void **)&pcie;
	hw.setup = brcm_setup_pcie_bridge;
	/* 28nm platform may not have a correct interrupt-map property yet */
	hw.map_irq = brcm_map_irq;
	hw.ops = &brcm_pci_ops;

	pci_common_init_dev(pcie->dev, &hw);

	return 0;
}

static const struct dev_pm_ops brcm_pcie_pm_ops = {
	.suspend_noirq = brcm_pcie_suspend,
	.resume_noirq = brcm_pcie_resume,
};

static struct platform_driver brcm_pci_driver = {
	.probe = brcm_pci_probe,
	.driver = {
		.name = "brcm-pci",
		.owner = THIS_MODULE,
		.of_match_table = brcm_pci_match,
		.pm = &brcm_pcie_pm_ops,
	},
};

static int __init brcm_pci_init(void)
{
	bus_register_notifier(&pci_bus_type,
			      &brcmstb_platform_nb);
	return platform_driver_register(&brcm_pci_driver);
}
module_init(brcm_pci_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom STB PCIE RC driver");
MODULE_AUTHOR("Broadcom");
