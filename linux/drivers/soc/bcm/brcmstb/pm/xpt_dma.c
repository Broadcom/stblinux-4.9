/*
 * Copyright © 2014 Broadcom Corporation
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

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define BCHP_PHYSICAL_OFFSET	0xf0000000

#include "xpt_dma.h"

struct xpt_dma_priv {
	void __iomem	*memdma_mcpb;
	void __iomem	*bus_if;
	void __iomem	*fe_pid;
	void __iomem	*sec_ns_intrl2;
	void __iomem	*sec_ns_mac0;
	void __iomem	*xpt_mcpb;
	void __iomem	*pmu_fe;
	void __iomem	*sun_top_ctrl;
};

static struct xpt_dma_priv xpt_dma_priv;
static bool has_memdma_mcpb;

#define XPT_DMA_IO_MACRO(name)	\
static inline u32 name##_readl(u32 off)					\
{									\
	return __raw_readl(xpt_dma_priv.name + off);			\
}									\
static inline void name##_writel(u32 val, u32 off)			\
{									\
	__raw_writel(val, xpt_dma_priv.name + off);			\
}									\

XPT_DMA_IO_MACRO(memdma_mcpb);
XPT_DMA_IO_MACRO(bus_if);
XPT_DMA_IO_MACRO(fe_pid);
XPT_DMA_IO_MACRO(sec_ns_intrl2);
XPT_DMA_IO_MACRO(sec_ns_mac0);
XPT_DMA_IO_MACRO(xpt_mcpb);
XPT_DMA_IO_MACRO(pmu_fe);
XPT_DMA_IO_MACRO(sun_top_ctrl);


/* descriptor flags and shifts */
#define MCPB_DW2_LAST_DESC		(1 << 0)

#define MCPB_DW4_PUSH_PARTIAL_PKT	(1 << 28)

#define MCPB_DW5_ENDIAN_STRAP_INV  (1 << 21)
#define MCPB_DW5_PID_CHANNEL_VALID (1 << 19)
#define MCPB_DW5_SCRAM_END         (1 << 18)
#define MCPB_DW5_SCRAM_START       (1 << 17)
#define MCPB_DW5_SCRAM_INIT        (1 << 16)

#define MEMDMA_DRAM_REQ_SIZE	256

#define XPT_MAC_OFFSET		0x14

#define MCPB_CHx_SPACING(channel) \
	((MEMDMA_MCPB_CH1_REG_START - \
	 MEMDMA_MCPB_CH0_REG_START) * (channel))

#define MCPB_CHx_SPACING_NMM(channel) \
	((MCPB_CH1_REG_START - \
	 MCPB_CH0_REG_START) * (channel))

#define XPT_CHANNEL_A	0
#define XPT_CHANNEL_B	1

#define PID_CHANNEL_A	1022
#define PID_CHANNEL_B	1023
#define PID_CHANNEL_A_NMM	510
#define PID_CHANNEL_B_NMM	511

#define XPT_MAC_A	0
#define XPT_MAC_B	1

#define MAX_HASH_WAIT_US	(15 * 1000 * 1000) /* 15 seconds */

/* MEMDMA MCPB registers, relative to XPT_MEMDMA_MCPB_REG_START */
#define MEMDMA_MCPB_RUN_SET_CLEAR		0x00
#define MEMDMA_MCPB_CH0_REG_START		0x400
#define MEMDMA_MCPB_CH1_REG_START		0x600

/* MCPB registers, relative to XPT_MCPB_REG_START */
#define	MCPB_RUN_SET_CLEAR			0x00
#define	MCPB_CH0_REG_START			0x400
#define	MCPB_CH1_REG_START			0x600

/* MCPB_CH0 has same offsets as MEMDMA_MCPB_CH0 */
#define MEMDMA_MCPB_CH0_DMA_DESC_CONTROL	0x400
#define MEMDMA_MCPB_CH0_DMA_DATA_CONTROL	0x404
#define MEMDMA_MCPB_CH0_SP_PKT_LEN		0x474
#define MEMDMA_MCPB_CH0_SP_PARSER_CTRL		0x478
#define MEMDMA_MCPB_CH0_DMA_BBUFF_CTRL		0x4c8

/* BUS IF registers, relative to XPT_BUS_IF_REG_START */
#define BUS_IF_SUB_MODULE_SOFT_INIT_SET		0x34
#define BUS_IF_SUB_MODULE_SOFT_INIT_CLEAR	0x38
#define BUS_IF_SUB_MODULE_SOFT_INIT_STATUS	0x3c
#define  MCPB_SOFT_INIT_STATUS			BIT(0)
#define  MEMDMA_MCPB_SOFT_INIT			BIT(1)
#define BUS_IF_SUB_MODULE_SOFT_INIT_DO_MEM_INIT	0x40

/* XPT FE PID registers, relative to XPT_FE_REG_START */
#define FE_PID_TABLE_i_ARRAY_BASE		0x1000
#define FE_SPID_TABLE_i_ARRAY_BASE		0x3000
#define FE_SPID_EXT_TABLE_i_ARRAY_BASE		0x5000

/* XPT SECURITY NS INTR2_0 */
#define SECURITY_NS_INTR2_0_CPU_STATUS		0x00
#define SECURITY_NS_INTR2_0_CPU_CLEAR		0x08
#define MAC1_READY				BIT(2)
#define MAC0_READY				BIT(0)

/* XPT SECURITY NS MAC0_0 */
#define SECURITY_NS_MAC0_0			0x30

/* PMU FE registers, relative to XPT_PMU_REG_START */
#define PMU_FE_SP_PD_MEM_PWR_DN_CTRL		0x1c
#define PMU_MCPB_SP_PD_MEM_PWR_DN_CTRL		0x24
#define PMU_MEMDMA_SP_PD_MEM_PWR_DN_CTRL	0x2c

#define TOP_CTRL_SW_INIT_0_SET			0x318
#define TOP_CTRL_SW_INIT_0_CLEAR		0x31c
#define TOP_CTRL_SW_INIT_0_XPT_SW_INIT		BIT(18)

static inline void xpt_set_power(int on)
{
	uint32_t v, val = on ? 0 : ~0;

	if (!xpt_dma_priv.pmu_fe)
		return;

	if (on && xpt_dma_priv.sun_top_ctrl) {
		v = sun_top_ctrl_readl(TOP_CTRL_SW_INIT_0_SET);
		v |= TOP_CTRL_SW_INIT_0_XPT_SW_INIT;
		sun_top_ctrl_writel(v, TOP_CTRL_SW_INIT_0_SET);
		wmb(); /* Sequence the reset. */

		v = sun_top_ctrl_readl(TOP_CTRL_SW_INIT_0_CLEAR);
		v |= TOP_CTRL_SW_INIT_0_XPT_SW_INIT;
		sun_top_ctrl_writel(v, TOP_CTRL_SW_INIT_0_CLEAR);
		wmb(); /* Must complete before we power on XPT. */
	}

	/* Power on/off everything */
	pmu_fe_writel(val, PMU_FE_SP_PD_MEM_PWR_DN_CTRL);
	pmu_fe_writel(val, PMU_MCPB_SP_PD_MEM_PWR_DN_CTRL);
	pmu_fe_writel(val, PMU_MEMDMA_SP_PD_MEM_PWR_DN_CTRL);
}

static void mcpb_run(int enable, int channel)
{
	if (has_memdma_mcpb) {
		memdma_mcpb_writel((!!enable << 8) | channel,
				   MEMDMA_MCPB_RUN_SET_CLEAR);
		(void)memdma_mcpb_readl(MEMDMA_MCPB_RUN_SET_CLEAR);
	} else {
		xpt_mcpb_writel((!!enable << 8) | channel,
				   MCPB_RUN_SET_CLEAR);
	}
}

static int mcpb_soft_init(void)
{
	int timeo = 1000 * 1000; /* 1 second timeout */
	u32 reg, mask, value;

	if (has_memdma_mcpb) {
		value = 1 << 1;
		mask = MEMDMA_MCPB_SOFT_INIT;
	} else {
		value = 1;
		mask = MCPB_SOFT_INIT_STATUS;
	}

	bus_if_writel(value, BUS_IF_SUB_MODULE_SOFT_INIT_DO_MEM_INIT);
	bus_if_writel(value, BUS_IF_SUB_MODULE_SOFT_INIT_SET);

	for (;;) {
		reg = bus_if_readl(BUS_IF_SUB_MODULE_SOFT_INIT_STATUS);
		if (!(reg & mask))
			break;

		if (timeo <= 0)
			return -EIO;

		timeo -= 10;
		udelay(10);
	}

	bus_if_writel(value, BUS_IF_SUB_MODULE_SOFT_INIT_CLEAR);

	return 0;
}

static void memdma_init_mcpb_channel(int channel)
{
	unsigned long offs, parser_ctrl, packet_len,
		dma_buf_ctrl, dma_data_ctrl;

	 offs = (has_memdma_mcpb) ?
		MCPB_CHx_SPACING(channel) : MCPB_CHx_SPACING_NMM(channel);

	parser_ctrl = offs + MEMDMA_MCPB_CH0_SP_PARSER_CTRL;
	packet_len = offs + MEMDMA_MCPB_CH0_SP_PKT_LEN;
	dma_buf_ctrl = offs + MEMDMA_MCPB_CH0_DMA_BBUFF_CTRL;
	dma_data_ctrl = offs + MEMDMA_MCPB_CH0_DMA_DATA_CONTROL;

	mcpb_run(0, channel);

	if (has_memdma_mcpb) {
		/* setup for block mode */
		memdma_mcpb_writel(
			(1 << 0)	| /* parser enable */
			(6 << 1)	| /* block mode */
			(channel << 6)	| /* band ID */
			(1 << 14),	/* select playback parser */
			parser_ctrl);

		memdma_mcpb_writel(208, packet_len); /* packet length */
		memdma_mcpb_writel(224, dma_buf_ctrl); /* stream feed size */
		memdma_mcpb_writel(
			(MEMDMA_DRAM_REQ_SIZE << 0) |
			(0 << 11), /* disable run version match */
			dma_data_ctrl);
	} else {
		xpt_mcpb_writel(
			(1 << 0)	| /* parser enable */
			(6 << 1)	| /* block mode */
			(channel << 6)	| /* band ID */
			(1 << 26)	| /* MEM DMA PIPE Enable */
			(1 << 14),	/* select playback parser */
			parser_ctrl);

		xpt_mcpb_writel(208, packet_len); /* packet length */
		xpt_mcpb_writel(224, dma_buf_ctrl); /* stream feed size */
		xpt_mcpb_writel(
			(MEMDMA_DRAM_REQ_SIZE << 0) |
			(1 << 10) |
			(0 << 11), /* disable run version match */
			dma_data_ctrl);
	}
}

static void xpt_init_ctx(unsigned int channel, unsigned int pid_channel)
{
	/* configure PID channel */
	fe_pid_writel((1 << 14) |		/* enable PID channel */
		      (channel << 16) |	/* input parser band */
		      (1 << 23) |		/* playback parser */
		      (1 << 28),		/* direct to XPT security */
		      FE_PID_TABLE_i_ARRAY_BASE + 4 * pid_channel);

	fe_pid_writel(0, FE_SPID_TABLE_i_ARRAY_BASE + 4 * pid_channel);

	/* G pipe */
	fe_pid_writel(1, FE_SPID_EXT_TABLE_i_ARRAY_BASE + 4 * pid_channel);
}

static void memdma_init_hw(int channel, int pid)
{
	memdma_init_mcpb_channel(channel);
	xpt_init_ctx(channel, pid);
}

static int mcpb_init_desc(struct mcpb_dma_desc *desc, dma_addr_t next,
		dma_addr_t buf, size_t len, int first, int last,
		unsigned int pid_channel)
{
	memset(desc, 0, sizeof(*desc));

	/* 5 LSBs must be 0; can only handle 32-bit addresses */
	if (WARN_ON((next & 0x1f) || (upper_32_bits(next) != 0)))
		return -EINVAL;

	desc->buf_hi = upper_32_bits(buf);
	desc->buf_lo = lower_32_bits(buf); /* BUFF_ST_RD_ADDR [31:0] */
	desc->next_offs = lower_32_bits(next); /* NEXT_DESC_ADDR [31:5] */
	desc->size = len; /* BUFF_SIZE [31:0] */
	desc->opts2 = MCPB_DW5_PID_CHANNEL_VALID | MCPB_DW5_ENDIAN_STRAP_INV;
	desc->pid_channel = pid_channel;

	if (first)
		desc->opts2 |= (MCPB_DW5_SCRAM_INIT | MCPB_DW5_SCRAM_START);

	if (last) {
		desc->next_offs = MCPB_DW2_LAST_DESC;
		desc->opts1 |= MCPB_DW4_PUSH_PARTIAL_PKT;
		desc->opts2 |= MCPB_DW5_SCRAM_END;
	}

	return 0;
}

/*
 * memdma_prepare_descs - prepare a MEMDMA descriptor chain
 *
 * @descs: array of descriptors
 * @descs_pa: physical address of @descs
 * @regions: the address ranges to set up for MEMDMA
 * @numregions: number of regions (in @descs and @regions)
 * @channel_A: if true, use the first MAC channel (a.k.a. "channel A"); if
 *     false, use the second MAC channel (a.k.a. "channel B")
 */
int memdma_prepare_descs(struct mcpb_dma_desc *descs, dma_addr_t descs_pa,
		struct dma_region *regions, int numregions, bool channel_A)
{
	int ret, i, pid;
	dma_addr_t next_pa = descs_pa;

	if (has_memdma_mcpb)
		pid = channel_A ? PID_CHANNEL_A : PID_CHANNEL_B;
	else
		pid = channel_A ? PID_CHANNEL_A_NMM : PID_CHANNEL_B_NMM;

	for (i = 0; i < numregions; i++) {
		int first = (i == 0);
		int last = (i == (numregions - 1));

		if (last)
			next_pa = 0;
		else
			next_pa += sizeof(*descs);

		ret = mcpb_init_desc(&descs[i], next_pa, regions[i].addr,
				     regions[i].len, first, last, pid);
		if (ret)
			return ret;
	}

	return 0;
}



static bool hash_is_ready(int mac)
{
	if (mac)
		return !!(sec_ns_intrl2_readl(SECURITY_NS_INTR2_0_CPU_STATUS) &
			  MAC1_READY);
	else
		return !!(sec_ns_intrl2_readl(SECURITY_NS_INTR2_0_CPU_STATUS) &
			  MAC0_READY);
}

static void clear_hash_interrupt(int mac)
{
	if (mac)
		sec_ns_intrl2_writel(MAC1_READY, SECURITY_NS_INTR2_0_CPU_CLEAR);
	else
		sec_ns_intrl2_writel(MAC0_READY, SECURITY_NS_INTR2_0_CPU_CLEAR);
}

static int memdma_wait_for_hash(int mac)
{
	int timeo = MAX_HASH_WAIT_US;
	for (;;) {
		if (hash_is_ready(mac))
			break;
		if (timeo <= 0) {
			pr_err("error: timeout waiting for MAC%d\n", mac);
			return -EIO;
		}

		timeo -= 10;
		udelay(10);
	}

	/* Clear status */
	clear_hash_interrupt(mac);

	return 0;
}

static void memdma_start(dma_addr_t desc, int channel)
{
	unsigned long reg = MEMDMA_MCPB_CH0_DMA_DESC_CONTROL;

	if (has_memdma_mcpb) {
		reg += MCPB_CHx_SPACING(channel);
		memdma_mcpb_writel((uint32_t)desc, reg);
	 } else {
		 reg += MCPB_CHx_SPACING_NMM(channel);
		xpt_mcpb_writel((uint32_t)desc, reg);
	}

	mcpb_run(1, channel);
}

/*
 * memdma_run - Run the MEMDMA MAC on up to 2 DMA descriptor chains
 *
 * @desc1: the physical address of the first descriptor chain
 * @desc2: the physical address of the second descriptor chain (optional)
 * @dual_channel: if true, then use desc2 with a second DMA channel; otherwise,
 *     ignore desc2
 */
int memdma_run(dma_addr_t desc1, dma_addr_t desc2, bool dual_channel)
{
	int ret, ret2 = 0, pid_channel_a, pid_channel_b;

	xpt_set_power(1);

	ret = mcpb_soft_init();
	if (ret)
		goto out;

	if (has_memdma_mcpb) {
		pid_channel_a = PID_CHANNEL_A;
		pid_channel_b = PID_CHANNEL_B;
	} else {
		pid_channel_a = PID_CHANNEL_A_NMM;
		pid_channel_b = PID_CHANNEL_B_NMM;
	}

	memdma_init_hw(0, pid_channel_a);
	mb();

	memdma_start(desc1, XPT_CHANNEL_A);

	if (dual_channel) {
		memdma_init_hw(1, pid_channel_b);
		mb();
		memdma_start(desc2, XPT_CHANNEL_B);
	}

	ret = memdma_wait_for_hash(XPT_MAC_A);
	if (dual_channel)
		ret2 = memdma_wait_for_hash(XPT_MAC_B);

	/* Report the 1st non-zero return code */
	if (!ret)
		ret = ret2;

out:
	xpt_set_power(0);

	return ret;
}

static uint32_t get_hash_idx(int mac_num, int word)
{
	int len = 128 / 32;
	unsigned long reg_base;

	if (word >= len)
		return 0;

	reg_base = SECURITY_NS_MAC0_0 + mac_num * XPT_MAC_OFFSET;
	return sec_ns_mac0_readl(reg_base + word * 4);
}

void get_hash(uint32_t *hash, bool dual_channel)
{
	/* 128-bit AES CMAC hash */
	int i, len = 128 / 8;

	/* MAC0 */
	for (i = 0; i < len / sizeof(*hash); i++)
		hash[i] = get_hash_idx(0, i);

	if (dual_channel)
		/* MAC1 */
		for (i = 0; i < len / sizeof(*hash); i++)
			hash[i] ^= get_hash_idx(1, i);
}

static struct resource xpt_dma_res[] = {
	{
		.name	= "xpt-memdma-mcpb",
		.start	= BCHP_PHYSICAL_OFFSET + 0x00a60800,
		.end	= BCHP_PHYSICAL_OFFSET + 0x00a60f5c,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "xpt-bus-if",
		.start	= BCHP_PHYSICAL_OFFSET + 0x00a00080,
		.end	= BCHP_PHYSICAL_OFFSET + 0x00a000fc,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "xpt-fe-pid",
		.start	= BCHP_PHYSICAL_OFFSET + 0x00a20000,
		.end	= BCHP_PHYSICAL_OFFSET + 0x00a25ffc,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "xpt-security-ns-intr2_0",
		.start	= BCHP_PHYSICAL_OFFSET + 0x00380080,
		.end	= BCHP_PHYSICAL_OFFSET + 0x003800ac,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "xpt-security-ns-mac0",
		.start	= BCHP_PHYSICAL_OFFSET + 0x00380200,
		.end	= BCHP_PHYSICAL_OFFSET + 0x003802c8,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "xpt-mcpb",
		.start	= BCHP_PHYSICAL_OFFSET + 0x00a70800,
		.end	= BCHP_PHYSICAL_OFFSET + 0x00a70b9c,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "xpt-pmu-fe",
		.start	= BCHP_PHYSICAL_OFFSET + 0x00a00200,
		.end	= BCHP_PHYSICAL_OFFSET + 0x00a00270,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device xpt_dma_pdev = {
	.name	= "xpt-dma",
	.resource = xpt_dma_res,
	.num_resources = ARRAY_SIZE(xpt_dma_res),
};

static const struct of_device_id sun_top_ctrl_match[] = {
	{ .compatible = "brcm,brcmstb-sun-top-ctrl", },
	{ }
};

static int xpt_dma_probe(struct platform_device *pdev)
{
	struct xpt_dma_priv *priv = &xpt_dma_priv;
	struct resource *r;
	unsigned int i;
	void __iomem **p;

	p = &priv->memdma_mcpb;

	for (i = 0; i < ARRAY_SIZE(xpt_dma_res); i++) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, i);

		if (!of_machine_is_compatible("brcm,bcm7439b0") &&
		    !strcmp(r->name, "xpt-pmu-fe") && has_memdma_mcpb)
			continue;

		*p = devm_ioremap_resource(&pdev->dev, r);
		if (IS_ERR(*p))
			return PTR_ERR(*p);

		p++;
	}

	if (!has_memdma_mcpb) {
		struct device_node *sun_top_ctrl;

		sun_top_ctrl = of_find_matching_node(NULL, sun_top_ctrl_match);
		if (!sun_top_ctrl)
			return -ENODEV;

		xpt_dma_priv.sun_top_ctrl = of_iomap(sun_top_ctrl, 0);
		if (!xpt_dma_priv.sun_top_ctrl)
			return -ENODEV;
	}

	dev_set_drvdata(&pdev->dev, priv);

	return 0;
}

static struct platform_driver xpt_dma_drv = {
	.driver = {
		.name = "xpt-dma",
	},
	.probe	= xpt_dma_probe,
};

static int __init xpt_dma_init(void)
{
	int ret = 0;

	if (of_machine_is_compatible("brcm,bcm74371a0") ||
	    of_machine_is_compatible("brcm,bcm7278a0"))
		return 0;

	if (of_machine_is_compatible("brcm,bcm7268a0") ||
	    of_machine_is_compatible("brcm,bcm7268b0") ||
	    of_machine_is_compatible("brcm,bcm7260a0") ||
	    of_machine_is_compatible("brcm,bcm7271a0") ||
	    of_machine_is_compatible("brcm,bcm7271b0"))
		has_memdma_mcpb = false;
	else
		has_memdma_mcpb = true;

	ret = platform_device_register(&xpt_dma_pdev);
	if (ret)
		return ret;

	ret = platform_driver_register(&xpt_dma_drv);
	if (ret)
		platform_device_unregister(&xpt_dma_pdev);

	return ret;
}

arch_initcall(xpt_dma_init);
