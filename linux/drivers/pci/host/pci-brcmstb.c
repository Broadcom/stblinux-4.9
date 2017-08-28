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
#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/printk.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/brcmstb/brcmstb.h>
#include <linux/string.h>
#include <linux/types.h>
#include "pci-brcmstb.h"


/* Broadcom Settop Box PCIE Register Offsets.  */
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
#define PCIE_MISC_PCIE_CTRL				0x4064
#define PCIE_MISC_PCIE_STATUS				0x4068
#define PCIE_MISC_REVISION				0x406c
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT	0x4070
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI		0x4080
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI		0x4084
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG			0x4204


/* Broadcom Settop Box PCIE Register Field Shift, Mask Info.  */
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_NEG_LINK_WIDTH_MASK  0x3f00000
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_NEG_LINK_WIDTH_SHIFT	0x14
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_NEG_LINK_SPEED_MASK	0xf0000
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_NEG_LINK_SPEED_SHIFT	0x10
#define PCIE_RC_CFG_PCIE_LINK_CAPABILITY_MAX_LINK_SPEED_MASK		0xf
#define PCIE_RC_CFG_PCIE_LINK_CAPABILITY_MAX_LINK_SPEED_SHIFT		0x0
#define PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL_RC_CRS_EN_MASK		0x10
#define PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL_RC_CRS_EN_SHIFT		0x4
#define	PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2_TARGET_LINK_SPEED_MASK	0xf
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2_TARGET_LINK_SPEED_SHIFT	0x0
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR1_MASK	0x3
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR1_SHIFT	0x0
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK	0xc
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_SHIFT	0x2
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR3_MASK	0x30
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR3_SHIFT	0x4
#define PCIE_RC_CFG_PRIV1_ID_VAL3_CLASS_CODE_MASK		0xffffff
#define PCIE_RC_CFG_PRIV1_ID_VAL3_CLASS_CODE_SHIFT		0x0
#define PCIE_MISC_MISC_CTRL_SCB_ACCESS_EN_MASK			0x1000
#define PCIE_MISC_MISC_CTRL_SCB_ACCESS_EN_SHIFT			0xc
#define PCIE_MISC_MISC_CTRL_CFG_READ_UR_MODE_MASK		0x2000
#define PCIE_MISC_MISC_CTRL_CFG_READ_UR_MODE_SHIFT		0xd
#define PCIE_MISC_MISC_CTRL_MAX_BURST_SIZE_MASK			0x300000
#define PCIE_MISC_MISC_CTRL_MAX_BURST_SIZE_SHIFT		0x14
#define PCIE_MISC_MISC_CTRL_SCB0_SIZE_MASK			0xf8000000
#define PCIE_MISC_MISC_CTRL_SCB0_SIZE_SHIFT			0x1b
#define PCIE_MISC_MISC_CTRL_SCB1_SIZE_MASK			0x7c00000
#define PCIE_MISC_MISC_CTRL_SCB1_SIZE_SHIFT			0x16
#define PCIE_MISC_MISC_CTRL_SCB2_SIZE_MASK			0x1f
#define PCIE_MISC_MISC_CTRL_SCB2_SIZE_SHIFT			0x0
#define PCIE_MISC_RC_BAR1_CONFIG_LO_MATCH_ADDRESS_MASK		0xfffff000
#define PCIE_MISC_RC_BAR1_CONFIG_LO_MATCH_ADDRESS_SHIFT		0xc
#define PCIE_MISC_RC_BAR1_CONFIG_LO_SIZE_MASK			0x1f
#define PCIE_MISC_RC_BAR1_CONFIG_LO_SIZE_SHIFT			0x0
#define PCIE_MISC_RC_BAR2_CONFIG_LO_MATCH_ADDRESS_MASK		0xfffff000
#define PCIE_MISC_RC_BAR2_CONFIG_LO_MATCH_ADDRESS_SHIFT		0xc
#define PCIE_MISC_RC_BAR2_CONFIG_LO_SIZE_MASK			0x1f
#define PCIE_MISC_RC_BAR2_CONFIG_LO_SIZE_SHIFT			0x0
#define PCIE_MISC_RC_BAR3_CONFIG_LO_MATCH_ADDRESS_MASK		0xfffff000
#define PCIE_MISC_RC_BAR3_CONFIG_LO_MATCH_ADDRESS_SHIFT		0xc
#define PCIE_MISC_RC_BAR3_CONFIG_LO_SIZE_MASK			0x1f
#define PCIE_MISC_RC_BAR3_CONFIG_LO_SIZE_SHIFT			0x0
#define PCIE_MISC_PCIE_CTRL_PCIE_PERSTB_MASK			0x4
#define PCIE_MISC_PCIE_CTRL_PCIE_PERSTB_SHIFT			0x2
#define PCIE_MISC_PCIE_CTRL_PCIE_L23_REQUEST_MASK		0x1
#define PCIE_MISC_PCIE_CTRL_PCIE_L23_REQUEST_SHIFT		0x0
#define PCIE_MISC_PCIE_STATUS_PCIE_PORT_MASK			0x80
#define PCIE_MISC_PCIE_STATUS_PCIE_PORT_SHIFT			0x7
#define PCIE_MISC_PCIE_STATUS_PCIE_DL_ACTIVE_MASK		0x20
#define PCIE_MISC_PCIE_STATUS_PCIE_DL_ACTIVE_SHIFT		0x5
#define PCIE_MISC_PCIE_STATUS_PCIE_PHYLINKUP_MASK		0x10
#define PCIE_MISC_PCIE_STATUS_PCIE_PHYLINKUP_SHIFT		0x4
#define PCIE_MISC_PCIE_STATUS_PCIE_LINK_IN_L23_MASK		0x40
#define PCIE_MISC_PCIE_STATUS_PCIE_LINK_IN_L23_SHIFT		0x6
#define PCIE_MISC_REVISION_MAJMIN_MASK				0xffff
#define PCIE_MISC_REVISION_MAJMIN_SHIFT				0
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_LIMIT_MASK	0xfff00000
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_LIMIT_SHIFT	0x14
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_BASE_MASK	0xfff0
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_BASE_SHIFT	0x4
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_NUM_MASK_BITS	0xc
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI_BASE_MASK		0xff
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI_BASE_SHIFT	0x0
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI_LIMIT_MASK	0xff
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI_LIMIT_SHIFT	0x0
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_CLKREQ_DEBUG_ENABLE_MASK	0x2
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_CLKREQ_DEBUG_ENABLE_SHIFT 0x1
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_SERDES_IDDQ_MASK		0x08000000
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_SERDES_IDDQ_SHIFT	0x1b
#define PCIE_RGR1_SW_INIT_1_PERST_MASK				0x1
#define PCIE_RGR1_SW_INIT_1_PERST_SHIFT				0x0


#define BRCM_NUM_PCI_OUT_WINS		0x4
#define BRCM_MAX_SCB			0x4

#define BRCM_MSI_TARGET_ADDR_LT_4GB	0x0fffffffcULL
#define BRCM_MSI_TARGET_ADDR_GT_4GB	0xffffffffcULL

#define BURST_SIZE_128			0
#define BURST_SIZE_256			1
#define BURST_SIZE_512			2

/* Offsets from PCIE_INTR2_CPU_BASE */
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

enum {
	RGR1_SW_INIT_1_INIT_MASK,
	RGR1_SW_INIT_1_INIT_SHIFT,
	RGR1_SW_INIT_1_PERST_MASK,
	RGR1_SW_INIT_1_PERST_SHIFT,
};


enum pcie_type {
	BCM7425,
	BCM7435,
	GENERIC,
	BCM7278,
};

#define RD_FLD(base, reg, field) \
	rd_fld(base + reg, reg##_##field##_MASK, reg##_##field##_SHIFT)
#define WR_FLD(base, reg, field, val) \
	wr_fld(base + reg, reg##_##field##_MASK, reg##_##field##_SHIFT, val)
#define WR_FLD_RB(base, reg, field, val) \
	wr_fld_rb(base + reg, reg##_##field##_MASK, reg##_##field##_SHIFT, val)
#define WR_FLD_WITH_OFFSET(base, off, reg, field, val) \
	wr_fld(base + reg + off, reg##_##field##_MASK, \
	       reg##_##field##_SHIFT, val)
#define EXTRACT_FIELD(val, reg, field) \
	((val & reg##_##field##_MASK) >> reg##_##field##_SHIFT)
#define INSERT_FIELD(val, reg, field, field_val) \
	((val & ~reg##_##field##_MASK) | \
	 (reg##_##field##_MASK & (field_val << reg##_##field##_SHIFT)))

struct pcie_cfg_data {
	const int *reg_field_info;
	const int *offsets;
	const enum pcie_type type;
};

static const int pcie_reg_field_info[] = {
	[RGR1_SW_INIT_1_INIT_MASK] = 0x2,
	[RGR1_SW_INIT_1_INIT_SHIFT] = 0x1,
};

static const int pcie_reg_field_info_bcm7278[] = {
	[RGR1_SW_INIT_1_INIT_MASK] = 0x1,
	[RGR1_SW_INIT_1_INIT_SHIFT] = 0x0,
};

static const int pcie_offset_bcm7425[] = {
	[RGR1_SW_INIT_1] = 0x8010,
	[EXT_CFG_INDEX]  = 0x8300,
	[EXT_CFG_DATA]   = 0x8304,
};

static const struct pcie_cfg_data bcm7425_cfg = {
	.reg_field_info	= pcie_reg_field_info,
	.offsets	= pcie_offset_bcm7425,
	.type		= BCM7425,
};

static const int pcie_offsets[] = {
	[RGR1_SW_INIT_1] = 0x9210,
	[EXT_CFG_INDEX]  = 0x9000,
	[EXT_CFG_DATA]   = 0x9004,
};


static const struct pcie_cfg_data bcm7435_cfg = {
	.reg_field_info	= pcie_reg_field_info,
	.offsets	= pcie_offsets,
	.type		= BCM7435,
};

static const struct pcie_cfg_data generic_cfg = {
	.reg_field_info	= pcie_reg_field_info,
	.offsets	= pcie_offsets,
	.type		= GENERIC,
};

static const int pcie_offset_bcm7278[] = {
	[RGR1_SW_INIT_1] = 0xc010,
	[EXT_CFG_INDEX] = 0x9000,
	[EXT_CFG_DATA] = 0x9004,
};

static const struct pcie_cfg_data bcm7278_cfg = {
	.reg_field_info = pcie_reg_field_info_bcm7278,
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

struct brcm_dev_pwr_supply {
	struct list_head node;
	char name[32];
	struct regulator *regulator;
};

static struct of_pci_range *dma_ranges;
static int num_dma_ranges;

/* Size of memory for each memory controller */
static phys_addr_t scb_size[BRCM_MAX_SCB];

struct brcm_window {
	dma_addr_t pci_addr;
	phys_addr_t cpu_addr;
	dma_addr_t size;
};

/* Internal Bus Controller Information.*/
struct brcm_pcie {
	struct list_head	list;
	void __iomem		*base;
	char			name[BRCM_PCIE_NAME_SIZE];
	bool			suspended;
	struct clk		*clk;
	struct device_node	*dn;
	int			irq;
	int			num_out_wins;
	bool			ssc;
	int			gen;
	u64			msi_target_addr;
	struct brcm_window	out_wins[BRCM_NUM_PCI_OUT_WINS];
	struct pci_sys_data	*sys;
	struct device		*dev;
	struct list_head	pwr_supplies;
	struct msi_controller	*msi;
	bool			msi_internal;
	unsigned int		rev;
	unsigned int		num;
	bool			bridge_setup_done;
	const int		*reg_offsets;
	const int		*reg_field_info;
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

/* This is to convert the size of the inbound bar region to the
 * non-liniear values of PCIE_X_MISC_RC_BAR[123]_CONFIG_LO.SIZE
 */
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

	if (!num_dma_ranges)
		return addr;

	for (p = dma_ranges; p < &dma_ranges[num_dma_ranges]; p++) {
		if (addr >= p->cpu_addr
		    && addr < p->cpu_addr + p->size)
			return addr - p->cpu_addr + p->pci_addr;
	}
	return BRCMSTB_ERROR_CODE;
}

dma_addr_t brcm_to_cpu(dma_addr_t addr)
{
	struct of_pci_range *p;

	if (!num_dma_ranges)
		return addr;
	for (p = dma_ranges; p < &dma_ranges[num_dma_ranges]; p++) {
		if (addr >= p->pci_addr
		    && addr < p->pci_addr + p->size)
			return addr - p->pci_addr + p->cpu_addr;
	}
	return addr;
}
#define MDIO_PORT0		0x0
#define MDIO_DATA_MASK		0x7fffffff
#define MDIO_DATA_SHIFT		0x0
#define MDIO_PORT_MASK		0xf0000
#define MDIO_PORT_SHIFT		0x16
#define MDIO_REGAD_MASK		0xffff
#define MDIO_REGAD_SHIFT	0x0
#define MDIO_CMD_MASK		0xfff00000
#define MDIO_CMD_SHIFT		0x14
#define MDIO_CMD_READ		0x1
#define MDIO_CMD_WRITE		0x0
#define MDIO_DATA_DONE_MASK	0x80000000
#define MDIO_RD_DONE(x)		(((x) & MDIO_DATA_DONE_MASK) ? 1 : 0)
#define MDIO_WT_DONE(x)		(((x) & MDIO_DATA_DONE_MASK) ? 0 : 1)
#define SSC_REGS_ADDR		0x1100
#define SET_ADDR_OFFSET		0x1f
#define SSC_CNTL_OFFSET		0x2
#define SSC_CNTL_OVRD_EN_MASK	0x8000
#define SSC_CNTL_OVRD_EN_SHIFT	0xf
#define SSC_CNTL_OVRD_VAL_MASK	0x4000
#define SSC_CNTL_OVRD_VAL_SHIFT	0xe
#define SSC_STATUS_OFFSET	0x1
#define SSC_STATUS_SSC_MASK	0x400
#define SSC_STATUS_SSC_SHIFT	0xa
#define SSC_STATUS_PLL_LOCK_MASK	0x800
#define SSC_STATUS_PLL_LOCK_SHIFT	0xb

static u32 mdio_form_pkt(int port, int regad, int cmd)
{
	u32 pkt = 0;

	pkt |= (port << MDIO_PORT_SHIFT) & MDIO_PORT_MASK;
	pkt |= (regad << MDIO_REGAD_SHIFT) & MDIO_REGAD_MASK;
	pkt |= (cmd << MDIO_CMD_SHIFT) & MDIO_CMD_MASK;

	return pkt;
}

/* negative return value indicates error */
static int mdio_read(void __iomem *base, u8 port, u8 regad)
{
	int tries;
	u32 data;

	bcm_writel(mdio_form_pkt(port, regad, MDIO_CMD_READ),
	       base + PCIE_RC_DL_MDIO_ADDR);
	bcm_readl(base + PCIE_RC_DL_MDIO_ADDR);

	data = bcm_readl(base + PCIE_RC_DL_MDIO_RD_DATA);
	for (tries = 0; !MDIO_RD_DONE(data) && tries < 10; tries++) {
		udelay(10);
		data = bcm_readl(base + PCIE_RC_DL_MDIO_RD_DATA);
	}

	return MDIO_RD_DONE(data)
		? (data & MDIO_DATA_MASK) >> MDIO_DATA_SHIFT
		: -EIO;
}


/* negative return value indicates error */
static int mdio_write(void __iomem *base, u8 port, u8 regad, u16 wrdata)
{
	int tries;
	u32 data;

	bcm_writel(mdio_form_pkt(port, regad, MDIO_CMD_WRITE),
	       base + PCIE_RC_DL_MDIO_ADDR);
	bcm_readl(base + PCIE_RC_DL_MDIO_ADDR);
	bcm_writel(MDIO_DATA_DONE_MASK | wrdata,
	       base + PCIE_RC_DL_MDIO_WR_DATA);

	data = bcm_readl(base + PCIE_RC_DL_MDIO_WR_DATA);
	for (tries = 0; !MDIO_WT_DONE(data) && tries < 10; tries++) {
		udelay(10);
		data = bcm_readl(base + PCIE_RC_DL_MDIO_WR_DATA);
	}

	return MDIO_WT_DONE(data) ? 0 : -EIO;
}

static u32 rd_fld(void __iomem *p, u32 mask, int shift)
{
	return (bcm_readl(p) & mask) >> shift;
}

static void wr_fld(void __iomem *p, u32 mask, int shift, u32 val)
{
	u32 reg = bcm_readl(p);

	reg = (reg & ~mask) | ((val << shift) & mask);
	bcm_writel(reg, p);
}

static void wr_fld_rb(void __iomem *p, u32 mask, int shift, u32 val)
{
	wr_fld(p, mask, shift, val);
	(void) bcm_readl(p);
}

/* configures device for ssc mode; negative return value indicates error */
static int set_ssc(void __iomem *base)
{
	int tmp;
	u16 wrdata;
	int pll, ssc;

	tmp = mdio_write(base, MDIO_PORT0, SET_ADDR_OFFSET, SSC_REGS_ADDR);
	if (tmp < 0)
		return tmp;

	tmp = mdio_read(base, MDIO_PORT0, SSC_CNTL_OFFSET);
	if (tmp < 0)
		return tmp;

	wrdata = INSERT_FIELD(tmp, SSC_CNTL_OVRD, EN, 1);
	wrdata = INSERT_FIELD(wrdata, SSC_CNTL_OVRD, VAL, 1);
	tmp = mdio_write(base, MDIO_PORT0, SSC_CNTL_OFFSET, wrdata);
	if (tmp < 0)
		return tmp;

	mdelay(1);
	tmp = mdio_read(base, MDIO_PORT0, SSC_STATUS_OFFSET);
	if (tmp < 0)
		return tmp;

	ssc = EXTRACT_FIELD(tmp, SSC_STATUS, SSC);
	pll = EXTRACT_FIELD(tmp, SSC_STATUS, PLL_LOCK);

	return (ssc && pll) ? 0 : -EIO;
}


/* limits operation to a specific generation (1, 2, or 3) */
static void set_gen(void __iomem *base, int gen)
{
	WR_FLD(base, PCIE_RC_CFG_PCIE_LINK_CAPABILITY, MAX_LINK_SPEED, gen);
	WR_FLD(base, PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2,
	       TARGET_LINK_SPEED, gen);
}


static void set_pcie_outbound_win(struct brcm_pcie *pcie, unsigned int win,
				  phys_addr_t cpu_addr, dma_addr_t  pci_addr,
				  dma_addr_t size)
{
	void __iomem *base = pcie->base;
	phys_addr_t cpu_addr_mb, limit_addr_mb;
	u32 tmp;

	/* Set the base of the pci_addr window */
	bcm_writel(lower_32_bits(pci_addr) + MMIO_ENDIAN,
	       base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO+(win*8));
	bcm_writel(upper_32_bits(pci_addr),
	       base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI+(win*8));

	cpu_addr_mb = cpu_addr >> 20;
	limit_addr_mb = (cpu_addr + size - 1) >> 20;

	/* Write the addr base low register */
	WR_FLD_WITH_OFFSET(base, (win*4),
			   PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT,
			   BASE, cpu_addr_mb);
	/* Write the addr limit low register */
	WR_FLD_WITH_OFFSET(base, (win*4),
			   PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT,
			   LIMIT, limit_addr_mb);

	if (pcie->type != BCM7435 && pcie->type != BCM7425) {
		/* Write the cpu addr high register */
		tmp = (u32)(cpu_addr_mb >>
			PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_NUM_MASK_BITS);
		WR_FLD_WITH_OFFSET(base, (win*8),
				   PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_HI,
				   BASE, tmp);
		/* Write the cpu limit high register */
		tmp = (u32)(limit_addr_mb >>
			PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT_NUM_MASK_BITS);
		WR_FLD_WITH_OFFSET(base, (win*8),
				   PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LIMIT_HI,
				   LIMIT, tmp);
	}
}


static int is_pcie_link_up(struct brcm_pcie *pcie, bool silent)
{
	void __iomem *base = pcie->base;
	u32 val = bcm_readl(base + PCIE_MISC_PCIE_STATUS);
	u32 dla = EXTRACT_FIELD(val, PCIE_MISC_PCIE_STATUS, PCIE_DL_ACTIVE);
	u32 plu = EXTRACT_FIELD(val, PCIE_MISC_PCIE_STATUS, PCIE_PHYLINKUP);
	u32 rc = EXTRACT_FIELD(val, PCIE_MISC_PCIE_STATUS, PCIE_PORT);

	if (!rc && !silent)
		dev_err(pcie->dev, "controller configured in EP mode\n");

	return  (rc && dla && plu) ? 1 : 0;
}

static inline void brcm_pcie_bridge_sw_init_set(struct brcm_pcie *pcie,
						unsigned int val)
{
	unsigned int offset;
	unsigned int shift = pcie->reg_field_info[RGR1_SW_INIT_1_INIT_SHIFT];
	u32 mask =  pcie->reg_field_info[RGR1_SW_INIT_1_INIT_MASK];

	if (pcie->type != BCM7278) {
		wr_fld_rb(PCIE_RGR1_SW_INIT_1(pcie), mask, shift, val);
	} else if (of_machine_is_compatible("brcm,bcm7278a0")) {
		/* The two PCIE instance on 7278a0 are not even consistent with
		 * respect to each other for internal offsets, here we offset
		 * by 0x14000 + RGR1_SW_INIT_1's relative offset to account for
		 * that.
		 */
		offset = pcie->num ? 0x14010
		    : pcie->reg_offsets[RGR1_SW_INIT_1];
		wr_fld_rb(pcie->base + offset, mask, shift, val);
	} else {
		wr_fld_rb(PCIE_RGR1_SW_INIT_1(pcie), mask, shift, val);
	}
}

static inline void brcm_pcie_perst_set(struct brcm_pcie *pcie,
				       unsigned int val)
{
	if (pcie->type != BCM7278)
		wr_fld_rb(PCIE_RGR1_SW_INIT_1(pcie),
			  PCIE_RGR1_SW_INIT_1_PERST_MASK,
			  PCIE_RGR1_SW_INIT_1_PERST_SHIFT, val);
	else
		/* Assert = 0, de-assert = 1 on 7278 */
		WR_FLD_RB(pcie->base, PCIE_MISC_PCIE_CTRL, PCIE_PERSTB, !val);
}


static int brcm_parse_ranges(struct brcm_pcie *pcie)
{
	struct resource_entry *win;
	int ret;

	ret = of_pci_get_host_bridge_resources(pcie->dn, 0, 0xff,
					       &pcie->sys->resources, NULL);
	if (ret != 0) {
		dev_err(pcie->dev, "failed to get host resources\n");
		return -1;
	}

	resource_list_for_each_entry(win, &pcie->sys->resources) {
		struct resource *parent, *res = win->res;
		dma_addr_t offset = (dma_addr_t) win->offset;

		if (resource_type(res) == IORESOURCE_IO) {
			parent = &ioport_resource;
		} else if (resource_type(res) == IORESOURCE_MEM) {
			if (pcie->num_out_wins >= BRCM_NUM_PCI_OUT_WINS) {
				dev_err(pcie->dev, "too many outbound wins\n");
				goto parse_fail;
			}
			pcie->out_wins[pcie->num_out_wins].cpu_addr
				= (phys_addr_t) res->start;
			pcie->out_wins[pcie->num_out_wins].pci_addr
				= (dma_addr_t) res->start - offset;
			pcie->out_wins[pcie->num_out_wins].size
				= (dma_addr_t) res->end - res->start + 1;
			pcie->num_out_wins++;
			parent = &iomem_resource;
		} else {
			continue;
		}

		ret = devm_request_resource(pcie->dev, parent, res);
		if (ret) {
			dev_err(pcie->dev, "failed to get res %pR\n", res);
			goto parse_fail;
		}
	}

	/* Add bogus IO resource so that pcibios_init_resources() does not
	 * allocate the same IO region for both of our pci controllers
	 */
	pcie->sys->io_res.start = (brcm_num_pci_controllers - 1) * SZ_64K ?  :
		PCIBIOS_MIN_IO;
	pcie->sys->io_res.end = (brcm_num_pci_controllers * SZ_64K) - 1;
	pcie->sys->io_res.flags = IORESOURCE_IO;
	pcie->sys->io_res.name = "brcmstb bogus IO";
	pci_add_resource(&pcie->sys->resources, &pcie->sys->io_res);

	return 0;

parse_fail:
	pci_free_resource_list(&pcie->sys->resources);
	return -1;
}

static void set_regulators(struct brcm_pcie *pcie, bool on)
{
	struct list_head *pos;

	list_for_each(pos, &pcie->pwr_supplies) {
		struct brcm_dev_pwr_supply *supply
			= list_entry(pos, struct brcm_dev_pwr_supply, node);

		if (on && regulator_enable(supply->regulator))
			pr_debug("Unable to turn on %s supply.\n",
				 supply->name);
		else if (!on && regulator_disable(supply->regulator))
			pr_debug("Unable to turn off %s supply.\n",
				 supply->name);
	}
}

static void brcm_pcie_setup_early(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	unsigned int scb_size_val;
	u64 rc_bar2_size = 0, rc_bar2_offset = 0, total_mem_size = 0;
	u64 msi_target_addr;
	u32 tmp, burst;
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
	tmp = bcm_readl(base + PCIE_MISC_REVISION);
	pcie->rev = EXTRACT_FIELD(tmp, PCIE_MISC_REVISION, MAJMIN);

	/* Set SCB_MAX_BURST_SIZE, CFG_READ_UR_MODE, SCB_ACCESS_EN */
	tmp = INSERT_FIELD(0, PCIE_MISC_MISC_CTRL, SCB_ACCESS_EN, 1);
	tmp = INSERT_FIELD(tmp, PCIE_MISC_MISC_CTRL, CFG_READ_UR_MODE, 1);
	burst = (pcie->type == GENERIC || pcie->type == BCM7278)
		? BURST_SIZE_512 : BURST_SIZE_256;
	tmp = INSERT_FIELD(tmp, PCIE_MISC_MISC_CTRL, MAX_BURST_SIZE, burst);
	bcm_writel(tmp, base + PCIE_MISC_MISC_CTRL);

	/* Set up inbound memory view for the EP (called RC_BAR2,
	 * not to be confused with the BARs that are advertised by
	 * the EP).
	 */
	for (i = 0; i < num_memc; i++)
		total_mem_size += scb_size[i];

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
	 * and everything else upstream.
	 */
	rc_bar2_size = roundup_pow_of_two_64(total_mem_size);

	if (dma_ranges) {
		/* The best-case scenario is to place the inbound
		 * region in the first 4GB of pci-space, as some
		 * legacy devices can only address 32bits.
		 * We would also like to put the MSI under 4GB
		 * as well, since some devices require a 32bit
		 * MSI target address.
		 */
		if (total_mem_size <= 0xc0000000ULL &&
		    rc_bar2_size <= 0x100000000ULL) {

			rc_bar2_offset = 0;
			/* If the viewport is less then 4GB we can fit
			 * the MSI target address under 4GB. Otherwise
			 * put it right below 64GB.
			 */
			msi_target_addr =
				(rc_bar2_size == 0x100000000ULL)
				? BRCM_MSI_TARGET_ADDR_GT_4GB
				: BRCM_MSI_TARGET_ADDR_LT_4GB;

		} else {
			/* The system memory is 4GB or larger so we
			 * cannot start the inbound region at location
			 * 0 (since we have to allow some space for
			 * outbound memory @ 3GB).  So instead we
			 * start it at the 1x multiple of its size
			 */
			rc_bar2_offset = rc_bar2_size;

			/* Since we are starting the viewport at 4GB or
			 * higher, put the MSI target address below 4GB
			 */
			msi_target_addr = BRCM_MSI_TARGET_ADDR_LT_4GB;
		}

	} else {
		/* Set simple configuration based on memory sizes
		 * only.  We always start the viewport at address 0,
		 * and set the MSI target address accordingly.
		 */
		rc_bar2_offset = 0;
		msi_target_addr = (rc_bar2_size >= 0x100000000ULL)
			? BRCM_MSI_TARGET_ADDR_GT_4GB
			: BRCM_MSI_TARGET_ADDR_LT_4GB;
	}

	pcie->msi_target_addr = msi_target_addr;

	tmp = lower_32_bits(rc_bar2_offset);
	tmp = INSERT_FIELD(tmp, PCIE_MISC_RC_BAR2_CONFIG_LO, SIZE,
			   encode_ibar_size(rc_bar2_size));
	bcm_writel(tmp, base + PCIE_MISC_RC_BAR2_CONFIG_LO);
	bcm_writel(upper_32_bits(rc_bar2_offset),
	       base + PCIE_MISC_RC_BAR2_CONFIG_HI);

	/* field: SCB0_SIZE, default = 0xf (1 GB) */
	scb_size_val = scb_size[0]
		? ilog2(scb_size[0]) - 15 : 0xf;
	WR_FLD(base, PCIE_MISC_MISC_CTRL, SCB0_SIZE, scb_size_val);

	/* field: SCB1_SIZE, default = 0xf (1 GB) */
	if (num_memc > 1) {
		scb_size_val = scb_size[1]
			? ilog2(scb_size[1]) - 15 : 0xf;
		WR_FLD(base, PCIE_MISC_MISC_CTRL, SCB1_SIZE, scb_size_val);
	}

	/* field: SCB2_SIZE, default = 0xf (1 GB) */
	if (num_memc > 2) {
		scb_size_val = scb_size[2]
			? ilog2(scb_size[2]) - 15 : 0xf;
		WR_FLD(base, PCIE_MISC_MISC_CTRL, SCB2_SIZE, scb_size_val);
	}

	/* disable the PCIE->GISB memory window (RC_BAR1) */
	WR_FLD(base, PCIE_MISC_RC_BAR1_CONFIG_LO, SIZE, 0);

	/* disable the PCIE->SCB memory window (RC_BAR3) */
	WR_FLD(base, PCIE_MISC_RC_BAR3_CONFIG_LO, SIZE, 0);

	if (!pcie->suspended) {
		/* clear any interrupts we find on boot */
		bcm_writel(0xffffffff, base + PCIE_INTR2_CPU_BASE + CLR);
		(void) bcm_readl(base + PCIE_INTR2_CPU_BASE + CLR);
	}

	/* Mask all interrupts since we are not handling any yet */
	bcm_writel(0xffffffff, base + PCIE_INTR2_CPU_BASE + MASK_SET);
	(void) bcm_readl(base + PCIE_INTR2_CPU_BASE + MASK_SET);

	if (pcie->gen)
		set_gen(base, pcie->gen);

	/* take the EP device out of reset */
	/* field: PCIE_SW_PERST = 0 */
	brcm_pcie_perst_set(pcie, 0);
}

static const char *link_speed_to_str(int s)
{
	switch (s) {
	case 1:
		return "2.5";
	case 2:
		return "5.0";
	case 3:
		return "8.0";
	default:
		break;
	}
	return "???";
}

static int brcm_setup_pcie_bridge(int nr, struct pci_sys_data *sys)
{
	struct brcm_pcie *pcie = sys->private_data;
	void __iomem *base = pcie->base;
	const int limit = pcie->suspended ? 1000 : 100;
	struct clk *clk;
	unsigned int status;
	int i, j, ret;
	u32 neg_width, neg_speed;
	bool ssc_good = false;

	pcie->sys = sys;

	/* Give the RC/EP time to wake up, before trying to configure RC.
	 * Intermittently check status for link-up, up to a total of 100ms
	 * when we don't know if the device is there, and up to 1000ms if
	 * we do know the device is there.
	 */
	for (i = 1, j = 0; j < limit && !is_pcie_link_up(pcie, true);
	     j += i, i = i*2)
		mdelay(i + j > limit ? limit - j : i);

	if (!is_pcie_link_up(pcie, false)) {
		dev_info(pcie->dev, "link down\n");
		goto FAIL;
	}

	if (!pcie->suspended && brcm_parse_ranges(pcie))
		goto FAIL;

	for (i = 0; i < pcie->num_out_wins; i++)
		set_pcie_outbound_win(pcie, i, pcie->out_wins[i].cpu_addr,
				      pcie->out_wins[i].pci_addr,
				      pcie->out_wins[i].size);

	/* For config space accesses on the RC, show the right class for
	 * a PCI-PCI bridge
	 */
	WR_FLD_RB(base, PCIE_RC_CFG_PRIV1_ID_VAL3, CLASS_CODE, 0x060400);

	if (pcie->ssc) {
		ret = set_ssc(base);
		if (ret == 0)
			ssc_good = true;
		else
			dev_err(pcie->dev,
				"failed attempt to enter ssc mode\n");
	}

	status = bcm_readl(base + PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL);
	neg_width = EXTRACT_FIELD(status, PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL,
				  NEG_LINK_WIDTH);
	neg_speed = EXTRACT_FIELD(status, PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL,
				  NEG_LINK_SPEED);
	dev_info(pcie->dev, "link up, %s Gbps x%u %s\n",
		 link_speed_to_str(neg_speed), neg_width,
		 ssc_good ? "(SSC)" : "(!SSC)");

	/* Enable configuration request retry (see pci_scan_device()) */
	/* field RC_CRS_EN = 1 */
	WR_FLD(base, PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL, RC_CRS_EN, 1);

	/* PCIE->SCB endian mode for BAR */
	/* field ENDIAN_MODE_BAR2 = DATA_ENDIAN */
	WR_FLD_RB(base, PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1,
		  ENDIAN_MODE_BAR2, DATA_ENDIAN);

	/* Refclk from RC should be gated with CLKREQ# input when ASPM L0s,L1
	 * is enabled =>  setting the CLKREQ_DEBUG_ENABLE field to 1.
	 */
	WR_FLD_RB(base, PCIE_MISC_HARD_PCIE_HARD_DEBUG, CLKREQ_DEBUG_ENABLE, 1);

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

	set_regulators(pcie, false);
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
	WR_FLD_RB(base, PCIE_MISC_PCIE_CTRL, PCIE_L23_REQUEST, 0);
	/* SERDES_IDDQ = 1 */
	WR_FLD_RB(base, PCIE_MISC_HARD_PCIE_HARD_DEBUG, SERDES_IDDQ, 1);
	/* Shutdown PCIe bridge */
	brcm_pcie_bridge_sw_init_set(pcie, 1);
}


static void enter_l23(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	int tries, l23;

	/* assert request for L23 */
	WR_FLD_RB(base, PCIE_MISC_PCIE_CTRL, PCIE_L23_REQUEST, 1);
	/* poll L23 status */
	for (tries = 0, l23 = 0; tries < 1000 && !l23; tries++)
		l23 = RD_FLD(base, PCIE_MISC_PCIE_STATUS, PCIE_LINK_IN_L23);
	if (!l23)
		dev_err(pcie->dev, "failed to enter L23\n");
}

static int brcm_pcie_suspend(struct device *dev)
{
	struct brcm_pcie *pcie = dev_get_drvdata(dev);

	if (!pcie->bridge_setup_done)
		return 0;

	enter_l23(pcie);
	turn_off(pcie);
	clk_disable(pcie->clk);
	set_regulators(pcie, false);
	pcie->suspended = true;

	return 0;
}

static int brcm_pcie_resume(struct device *dev)
{
	struct brcm_pcie *pcie = dev_get_drvdata(dev);
	void __iomem *base;

	if (!pcie->bridge_setup_done)
		return 0;

	base = pcie->base;
	set_regulators(pcie, true);
	clk_enable(pcie->clk);

	/* Take bridge out of reset so we can access the SERDES reg */
	brcm_pcie_bridge_sw_init_set(pcie, 0);

	/* SERDES_IDDQ = 0 */
	WR_FLD_RB(base, PCIE_MISC_HARD_PCIE_HARD_DEBUG, SERDES_IDDQ, 0);
	/* wait for serdes to be stable */
	udelay(100);

	brcm_pcie_setup_early(pcie);

	brcm_setup_pcie_bridge(pcie->num, pcie->sys);
	if (pcie->msi && pcie->msi_internal)
		brcm_msi_set_regs(pcie->msi);

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

	if (!is_pcie_link_up(pcie, true))
		return NULL;

	base = pcie->base;
	idx = cfg_index(bus->number, devfn, where);

	bcm_writel(idx, IDX_ADDR(pcie));

	if (rc_access) {
		if (PCI_SLOT(devfn))
			return NULL;
		return base + (where & ~3);
	}

	return DATA_ADDR(pcie);
}

/* THIS FUNCTION NOT UPSTREAMED */
static int fix_interrupt_map_prop(struct brcm_pcie *pcie)
{
	struct device_node *p, *dn = pcie->dn;
	const char *prop_name = "interrupt-map";
	const __be32 *imap_prop, *tmp;
	struct property *new;
	int len, i, j, cells_per_imap, intr_cells;

	/* Get the #interrupt-cells of the parent */
	p = of_irq_find_parent(dn);
	if (p == NULL)
		return -EINVAL;
	tmp = of_get_property(p, "#interrupt-cells", NULL);
	if (tmp == NULL)
		return -EINVAL;
	intr_cells = be32_to_cpu(*tmp);

	imap_prop = of_get_property(dn, prop_name, &len);
	if (imap_prop == NULL) {
		dev_err(pcie->dev, "missing interrupt-map\n");
		return -EINVAL;
	}

	/* Check for the proper length; there are four interrupts
	 * (A,B,C,D) being mapped, each mapping has 3 address cells, 1
	 * cell for the legacy interrupt specifier, the interrupt
	 * phandle, and then the interrupt specifier.
	 */
	cells_per_imap = 3 + 1 + 1 + intr_cells;
	if (len == sizeof(__be32) * 4 * cells_per_imap)
		return 0;

	/* If we are here, we know that the interrupt-map property is
	 * malformed.  In our case -- an old bootloader, which
	 * generates the incorrect interrupt-map -- each interrupt
	 * mapping is rudely missing the first cell of the interrupt
	 * specifier.  We check if this is the case, and if so,
	 * correct the property by inserting a 0 for this cell.
	 */
	if (len != sizeof(__be32) * 4 * (cells_per_imap - 1))
		return -EINVAL;

	dev_info(pcie->dev, "fixing incorrect interrupt-map in DT node\n");
	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		return -ENOMEM;

	new->name = kstrdup(prop_name, GFP_KERNEL);
	if (!new->name)
		goto cleanup;

	new->length = sizeof(__be32) * 4 * cells_per_imap;
	new->value = kmalloc(new->length, GFP_KERNEL);
	if (!new->value)
		goto cleanup;

	/* For each interrupt map, copy the old to the new making sure to
	 * insert a zero cell in the new map.  Also, ensure that the last
	 * interrupt specifier is set to 4 for level interrupts to the ARM
	 * GIC.
	 */
	for (i = 0; i < 4; i++)
		for (j = 0; j < cells_per_imap; j++) {
			int k  = (cells_per_imap - 1) * i + j;
			int kk = cells_per_imap * i + j;

			if (j < 5)
				((__be32 *)new->value)[kk] = imap_prop[k];
			else if (j == 5)
				((__be32 *)new->value)[kk] = cpu_to_be32(0);
			else if (j == cells_per_imap - 1)
				((__be32 *)new->value)[kk] = cpu_to_be32(4);
			else if (j > 5)
				((__be32 *)new->value)[kk] = imap_prop[k - 1];
		}

	of_update_property(dn, new);
	return 0;

cleanup:
	kfree(new->name);
	kfree(new->value);
	kfree(new);
	return -ENOMEM;
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

	if (pci_is_root_bus(dev->bus) && IS_ENABLED(CONFIG_PCI_MSI))
		dev->bus->msi = pcie->msi;

	dev_info(pcie->dev,
		 "found device %04x:%04x on bus %d (%s), slot %d (irq %d)\n",
		 dev->vendor, dev->device, dev->bus->number, pcie->name,
		 slot, of_irq_parse_and_map_pci(dev, slot, 1));
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
	struct device_node *dn = pdev->dev.of_node, *mdn, *msi_dn;
	const struct of_device_id *of_id;
	const struct pcie_cfg_data *data;
	int i, rlen, ret;
	struct brcm_pcie *pcie;
	struct resource *r;
	const u32 *log2_scb_sizes; /* NOT UPSTREAMED */
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
	pcie->reg_field_info = data->reg_field_info;
	pcie->type = data->type;
	pcie->dn = dn;
	pcie->dev = &pdev->dev;

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
		list_add_tail(&supply->node, &pcie->pwr_supplies);
	}
	set_regulators(pcie, true);

	/* 'num_memc' will be set only by the first controller, and all
	 * other controllers will use the value set by the first.
	 */
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

	/* THIS SNIPPET NOT UPSTREAMED */
	ret = fix_interrupt_map_prop(pcie);
	if (ret < 0)
		return ret;

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
	pcie->base = base;
	pcie->gen = 0;

	if (of_property_read_u32(dn, "brcm,gen", &tmp) == 0)
		pcie->gen = tmp;

	pcie->ssc = of_property_read_bool(dn, "brcm,ssc");

	/* THIS SNIPPET NOT UPSTREAMED */
	/* Get the value for the log2 of the scb sizes.  Subtract 15 from
	 * each because the target register field has 0==disabled and 1==64KB.
	 */
	log2_scb_sizes = of_get_property(dn, "brcm,log2-scb-sizes", &rlen);
	if (log2_scb_sizes != NULL) {
		for (i = 0; i < rlen/4; i++) {
			scb_size[i] =
				1ULL << of_read_number(log2_scb_sizes + i, 1);
		}
	}

	/* Parse dma-ranges property if present.  If there are multiple
	 * PCI controllers, we only have to parse from one of them since
	 * the others will have an identical mapping.
	 */
	if (!dma_ranges && !brcm_pci_dma_range_parser_init(&parser, dn)) {
		unsigned int max_ranges
			= (parser.end - parser.range) / parser.np;

		dma_ranges = devm_kzalloc(&pdev->dev,
					  sizeof(struct of_pci_range) *
					  max_ranges, GFP_KERNEL);
		if (!dma_ranges)
			return -ENOMEM;

		for (i = 0; of_pci_range_parser_one(&parser, dma_ranges + i);
		     i++)
			num_dma_ranges++;
	}

	/* THE CLAUSE "(dma_ranges || !log2_scb_sizes)" NOT UPSTREAMED,
	 * however, the body inside the clause IS UPSTREAMED.
	 */
	if (dma_ranges || !log2_scb_sizes) {
		for (i = 0; i < num_memc; i++) {
			u64 size = brcmstb_memory_memc_size(i);

			if (size == (u64) -1) {
				dev_err(pcie->dev, "cannot get memc%d size", i);
				return -EINVAL;
			}
			scb_size[i] = roundup_pow_of_two_64(size);
		}
	}

	ret = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (ret == 0)
		/* keep going, as we don't use this intr yet */
		dev_warn(pcie->dev, "cannot get pcie interrupt\n");
	else
		pcie->irq = ret;

	/*
	 * Starts PCIe link negotiation immediately at kernel boot time.  The
	 * RC is supposed to give the endpoint device 100ms to settle down
	 * before attempting configuration accesses.  So we let the link
	 * negotiation happen in the background instead of busy-waiting.
	 */
	brcm_pcie_setup_early(pcie);
	brcm_num_pci_controllers++;


	msi_dn = of_parse_phandle(pcie->dn, "msi-parent", 0);
	/* Use the internal MSI if no msi-parent property */
	if (!msi_dn)
		msi_dn = pcie->dn;

	if (IS_ENABLED(CONFIG_PCI_MSI) && pci_msi_enabled()
	    && msi_dn == pcie->dn) {
		struct brcm_info info;

		info.rev = pcie->rev;
		info.msi_target_addr = pcie->msi_target_addr;
		info.base = pcie->base;
		info.name = pcie->name;
		info.pcie = pcie;

		ret = brcm_msi_probe(pdev, &info);
		if (ret)
			dev_err(pcie->dev,
				"probe of internal MSI failed: %d)", ret);
		else
			pcie->msi_internal = true;
	}

	memset(&hw, 0, sizeof(hw));
	hw.nr_controllers = 1;
	hw.private_data = (void **)&pcie;
	hw.setup = brcm_setup_pcie_bridge;
	hw.map_irq = of_irq_parse_and_map_pci;
	hw.ops = &brcm_pci_ops;
	pcie->msi = of_pci_find_msi_chip_by_node(msi_dn);
	hw.msi_ctrl = pcie->msi;
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
