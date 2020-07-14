// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom STB ASP 2.0 Driver
 *
 * Copyright (c) 2020 Broadcom
 */
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/clk.h>

#include "bcmasp.h"

static const struct of_device_id bcmasp_of_match[] = {
	{ .compatible = "brcm,asp-v2.0", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, bcmasp_of_match);

static const struct of_device_id bcmasp_mdio_of_match[] = {
	{ .compatible = "brcm,asp-v2.0-mdio", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, bcmasp_of_match);

static inline void _intr2_mask_clear(struct bcmasp_priv *priv,
				u32 mask)
{
	priv->irq_mask &= ~mask;
	intr2_core_wl(priv, mask, ASP_INTR2_MASK_CLEAR);
}

static inline void _intr2_mask_set(struct bcmasp_priv *priv,
				u32 mask)
{
	intr2_core_wl(priv, mask, ASP_INTR2_MASK_SET);
	priv->irq_mask |= mask;
}

void bcmasp_enable_tx_irq(struct bcmasp_intf *intf, int en)
{
	struct bcmasp_priv *priv = intf->parent;
	int ch = intf->channel;

	if (en)
		_intr2_mask_clear(priv, ASP_INTR2_TX_DESC(ch));
	else
		_intr2_mask_set(priv, ASP_INTR2_TX_DESC(ch));
}

void bcmasp_enable_rx_irq(struct bcmasp_intf *intf, int en)
{
	struct bcmasp_priv *priv = intf->parent;
	int ch = intf->channel;

	if (en)
		_intr2_mask_clear(priv, ASP_INTR2_RX_ECH(ch));
	else
		_intr2_mask_set(priv, ASP_INTR2_RX_ECH(ch));
}

static irqreturn_t bcmasp_isr(int irq, void *data)
{
	struct bcmasp_priv *priv = data;
	struct bcmasp_intf *intf;
	u32 status, i;


	status = intr2_core_rl(priv, ASP_INTR2_STATUS) &
		~intr2_core_rl(priv, ASP_INTR2_MASK_STATUS);
	intr2_core_wl(priv, status, ASP_INTR2_CLEAR);

	if (unlikely(status == 0)) {
		dev_warn(&priv->pdev->dev, "spurious interrupt\n");
		return IRQ_NONE;
	}

	for (i = 0; i < priv->intf_count; i++)
	{
		intf = priv->intfs[i];

		if (status & ASP_INTR2_RX_ECH(i)) {
			if (likely(napi_schedule_prep(&intf->rx_napi))) {
				bcmasp_enable_rx_irq(intf, 0);
				__napi_schedule_irqoff(&intf->rx_napi);
			}
		}

		if (status & ASP_INTR2_TX_DESC(i)) {
			if (likely(napi_schedule_prep(&intf->tx_napi))) {
				bcmasp_enable_tx_irq(intf, 0);
				__napi_schedule_irqoff(&intf->tx_napi);
			}
		}

	}
	return IRQ_HANDLED;
}

void bcmasp_flush_rx_port(struct bcmasp_intf *intf)
{
	struct bcmasp_priv *priv = intf->parent;
	u32 mask;

	switch (intf->port) {
		case 0:
			mask = ASP_CTRL_UMAC0_FLUSH_MASK;
			break;
		case 1:
			mask = ASP_CTRL_UMAC1_FLUSH_MASK;
			break;
		default:
			/* Not valid port */
			return;
		}

	rx_ctrl_core_wl(priv, mask, ASP_RX_CTRL_FLUSH);
}

static inline void bcmasp_addr_to_uint(unsigned char *addr, u32 *high, u32 *low)
{
	*high = (u32)(addr[0] << 8 | addr[1]);
	*low = (u32)(addr[2] << 24 | addr[3] << 16 | addr[4] << 8 |
		     addr[5]);
}

static void bcmasp_set_mda_filter(struct bcmasp_intf *intf,
				   unsigned char *addr,
				   unsigned char *mask,
				   unsigned int i)
{
	struct bcmasp_priv *priv = intf->parent;
	u32 addr_h, addr_l, mask_h, mask_l;

	/* Set local copy */
	memcpy(priv->mda_filters[i].mask, mask, ETH_ALEN);
	memcpy(priv->mda_filters[i].addr, addr, ETH_ALEN);

	/* Write to HW */
	bcmasp_addr_to_uint(priv->mda_filters[i].mask, &mask_h, &mask_l);
	bcmasp_addr_to_uint(priv->mda_filters[i].addr, &addr_h, &addr_l);
	rx_filter_core_wl(priv, addr_h, ASP_RX_FILTER_MDA_PAT_H(i));
	rx_filter_core_wl(priv, addr_l, ASP_RX_FILTER_MDA_PAT_L(i));
	rx_filter_core_wl(priv, mask_h, ASP_RX_FILTER_MDA_MSK_H(i));
	rx_filter_core_wl(priv, mask_l, ASP_RX_FILTER_MDA_MSK_L(i));
}

static void bcmasp_en_mda_filter(struct bcmasp_intf *intf, bool en,
				 unsigned int i)
{
	struct bcmasp_priv *priv = intf->parent;

	if (priv->mda_filters[i].en == en)
		return;

	priv->mda_filters[i].en = en;
	priv->mda_filters[i].port = intf->port;

	rx_filter_core_wl(priv, ((intf->channel + 8) |
			  (en << ASP_RX_FILTER_MDA_CFG_EN_SHIFT) |
			  ASP_RX_FILTER_MDA_CFG_UMC_SEL(intf->port)),
			  ASP_RX_FILTER_MDA_CFG(i));
}

/*
 * There are 32 MDA filters shared between all ports, we reserve 4 filters per
 * port for the following.
 * - Promisc: Filter to allow all packets when promisc is enabled
 * - All Multicast
 * - Broadcast
 * - Own address
 *
 * The reserved filters are identified as so.
 * - Promisc: (Port * 4) + 0
 * - All Multicast: (Port * 4) + 1
 * - Broadcast: (Port * 4) + 2
 * - Own address: (Port * 4) + 3
 */
enum asp_rx_filter_id {
	ASP_RX_FILTER_PROMISC = 0,
	ASP_RX_FILTER_ALLMULTI,
	ASP_RX_FILTER_BROADCAST,
	ASP_RX_FILTER_OWN_ADDR,
	ASP_RX_FILTER_RES_COUNT,
};
#define ASP_RX_FILT_RES_COUNT(intf)	(intf->parent->intf_count \
					 * ASP_RX_FILTER_RES_COUNT)
#define ASP_RX_FILT(intf, name)		((intf->port * \
					  ASP_RX_FILTER_RES_COUNT) \
					 + ASP_RX_FILTER_##name)

void bcmasp_set_promisc(struct bcmasp_intf *intf, bool en)
{
	unsigned char promisc[ETH_ALEN];
	unsigned int i = ASP_RX_FILT(intf, PROMISC);

	eth_zero_addr(promisc);
	/* Set mask to 00:00:00:00:00:00 to match all packets */
	bcmasp_set_mda_filter(intf, promisc, promisc, i);
	bcmasp_en_mda_filter(intf, en, i);
}

void bcmasp_set_allmulti(struct bcmasp_intf *intf, bool en)
{
	unsigned char allmulti[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned int i = ASP_RX_FILT(intf, ALLMULTI);

	/* Set mask to 01:00:00:00:00:00 to match all multicast */
	bcmasp_set_mda_filter(intf, allmulti, allmulti, i);
	bcmasp_en_mda_filter(intf, en, i);
}

void bcmasp_set_broad(struct bcmasp_intf *intf, bool en)
{
	unsigned char addr[ETH_ALEN];
	unsigned int i = ASP_RX_FILT(intf, BROADCAST);

	eth_broadcast_addr(addr);
	bcmasp_set_mda_filter(intf, addr, addr, i);
	bcmasp_en_mda_filter(intf, en, i);
}

void bcmasp_set_oaddr(struct bcmasp_intf *intf, unsigned char *addr, bool en)
{
	unsigned char mask[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	unsigned int i = ASP_RX_FILT(intf, OWN_ADDR);

	bcmasp_set_mda_filter(intf, addr, mask, i);
	bcmasp_en_mda_filter(intf, en, i);
}

void bcmasp_disable_all_filters(struct bcmasp_intf *intf)
{
	struct bcmasp_priv *priv = intf->parent;
	unsigned int i;

	/* Disable all filters held by this port */
	for (i = ASP_RX_FILT_RES_COUNT(intf); i < ASP_RX_FILTER_MAX; i++) {
		if (priv->mda_filters[i].en &&
		   (priv->mda_filters[i].port == intf->port))
			bcmasp_en_mda_filter(intf, 0, i);
	}
}

static inline void u64_to_mac(unsigned char *addr, u64 val)
{
	addr[0] = (u8)(val >> 40);
	addr[1] = (u8)(val >> 32);
	addr[2] = (u8)(val >> 24);
	addr[3] = (u8)(val >> 16);
	addr[4] = (u8)(val >> 8);
	addr[5] = (u8)val;
}
#define mac_to_u64(a) 		((((u64)a[0]) << 40) | \
				(((u64)a[1]) << 32) | \
				(((u64)a[2]) << 24) | \
				(((u64)a[3]) << 16) | \
				(((u64)a[4]) << 8) | \
				((u64)a[5]))
#define differ_one_bit(x, y)	is_power_of_2((x) ^ (y))

static int bcmasp_combine_set_filter(struct bcmasp_intf *intf,
				     unsigned char *addr, unsigned char *mask,
				     int i)
{
	u64 addr1, addr2, mask1, mask2, mask3;
	unsigned char naddr[ETH_ALEN], nmask[ETH_ALEN];
	struct bcmasp_priv *priv = intf->parent;

	/* Switch to u64 to help with the calculations */
	addr1 = mac_to_u64(priv->mda_filters[i].addr);
	mask1 = mac_to_u64(priv->mda_filters[i].mask);
	addr2 = mac_to_u64(addr);
	mask2 = mac_to_u64(mask);

	/*
	 * We can only combine filters in two cases
	 * 1. They share the same mask and are different by one bit
	 * 2. One filter resides within the other
	 */
	if (mask1 == mask2) {
		if(!differ_one_bit((addr1 & mask1), (addr2 & mask2)))
			return -EINVAL;

		/* Generate new mask */
		mask3 = ((addr1 & mask1) ^ (addr2 & mask1)) ^ mask1;

		/* Set new filter */
		u64_to_mac(naddr, (addr1 & mask3));
		u64_to_mac(nmask, mask3);
		bcmasp_set_mda_filter(intf, naddr, nmask, i);
		return 0;
	}

	/* Check if one filter resides within the other */
	mask3 = mask1 & mask2;
	if ((mask3 == mask1) && ((addr1 & mask1) == (addr2 & mask1))) {
		/* Filter 2 resides within fitler 1, so everthing is good */
		return 0;
	} else if ((mask3 == mask2) && ((addr1 & mask2) == (addr2 & mask2))) {
		/* Filter 1 resides within filter 2, so swap filters */
		bcmasp_set_mda_filter(intf, addr, mask, i);
		return 0;
	}

	/* Unable to combine */
	return -EINVAL;
}

int bcmasp_set_en_mda_filter(struct bcmasp_intf *intf, unsigned char *addr,
			      unsigned char *mask)
{
	struct bcmasp_priv *priv = intf->parent;
	int i, ret;

	for (i = ASP_RX_FILT_RES_COUNT(intf); i < ASP_RX_FILTER_MAX; i++) {
		/* If filter not enabled or belongs to another port skip */
		if (!priv->mda_filters[i].en ||
		    (priv->mda_filters[i].port != intf->port))
			continue;

		/* Attempt to combine filters */
		ret = bcmasp_combine_set_filter(intf, addr, mask, i);
		if (!ret) {
			intf->mib.filters_combine_cnt++;
			return 0;
		}
	}

	/* Create new filter if possible */
	for (i = ASP_RX_FILT_RES_COUNT(intf); i < ASP_RX_FILTER_MAX; i++) {
		if (priv->mda_filters[i].en)
			continue;

		bcmasp_set_mda_filter(intf, addr, mask, i);
		bcmasp_en_mda_filter(intf, 1, i);
		return 0;
	}

	/* No room for new filter */
	return -EINVAL;
}

static inline void bcmasp_core_init_filters(struct bcmasp_priv *priv)
{
	int i;

	/* Disable all filters and reset software view since the HW
	 * can lose context while in deep sleep suspend states
	 */
	for (i = 0; i < NUM_MDA_FILTERS; i++) {
		rx_filter_core_wl(priv, 0x0, ASP_RX_FILTER_MDA_CFG(i));
		priv->mda_filters[i].en = 0;
	}

	for (i = 0; i < NUM_NET_FILTERS; i++)
		rx_filter_core_wl(priv, 0x0, ASP_RX_FILTER_NET_CFG(i));

	/* Top level filter enable bit should be enabled at all times */
	rx_filter_core_wl(priv, (ASP_RX_FILTER_OPUT_EN |
			  ASP_RX_FILTER_MDA_EN |
			  ASP_RX_FILTER_NT_FLT_EN),
			  ASP_RX_FILTER_BLK_CTRL);
}

/* ASP core initalization */
static inline void bcmasp_core_init(struct bcmasp_priv *priv)
{
	tx_analytics_core_wl(priv, 0x0, ASP_TX_ANALYTICS_CTRL);
	rx_analytics_core_wl(priv, 0x4, ASP_RX_ANALYTICS_CTRL);

	rx_ctrl_core_wl(priv, (ASP_RX_CTRL_S2F_DEFAULT_EN |
		       (0xd << ASP_RX_CTRL_S2F_CHID_SHIFT)),
			ASP_RX_CTRL_S2F);

	rx_edpkt_core_wl(priv, (ASP_EDPKT_HDR_SZ_128 << ASP_EDPKT_HDR_SZ_SHIFT),
			 ASP_EDPKT_HDR_CFG);
	rx_edpkt_core_wl(priv,
			(ASP_EDPKT_ENDI_BT_SWP_WD << ASP_EDPKT_ENDI_DESC_SHIFT),
			 ASP_EDPKT_ENDI);

	rx_edpkt_core_wl(priv, 0x1b, ASP_EDPKT_BURST_BUF_PSCAL_TOUT);
	rx_edpkt_core_wl(priv, 0x3e8, ASP_EDPKT_BURST_BUF_WRITE_TOUT);
	rx_edpkt_core_wl(priv, 0x3e8, ASP_EDPKT_BURST_BUF_READ_TOUT);

	rx_edpkt_core_wl(priv, ASP_EDPKT_ENABLE_EN, ASP_EDPKT_ENABLE);
}

static void bcmasp_core_clock_select(struct bcmasp_priv *priv, bool slow)
{
	u32 reg;

	reg = ctrl_core_rl(priv, ASP_CTRL_CORE_CLOCK_SELECT);
	if (slow)
		reg &= ~ASP_CTRL_CORE_CLOCK_SELECT_MAIN;
	else
		reg |= ASP_CTRL_CORE_CLOCK_SELECT_MAIN;
	ctrl_core_wl(priv, reg, ASP_CTRL_CORE_CLOCK_SELECT);
}

static int bcmasp_probe(struct platform_device *pdev)
{
	struct bcmasp_priv *priv;
	struct device_node *ports_node, *intf_node;
	struct device *dev = &pdev->dev;
	struct bcmasp_intf *intf;
	struct resource *r;
	int ret, i, wol_irq;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "failed to alloc priv\n");
		return -ENOMEM;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq <= 0) {
		dev_err(dev, "invalid interrupt\n");
		return -EINVAL;
	}

	priv->clk = devm_clk_get(dev, "sw_asp");
	if (IS_ERR(priv->clk)) {
		if (PTR_ERR(priv->clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "failed to request clock\n");
		priv->clk = NULL;
	}

	/* Base from parent node */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(priv->base)) {
		dev_err(dev, "failed to iomap\n");
		return PTR_ERR(priv->base);
	}

        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
        if (ret)
                ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (ret) {
                dev_err(&pdev->dev, "unable to set DMA mask: %d\n", ret);
                return ret;
        }

	dev_set_drvdata(&pdev->dev, priv);
	priv->pdev = pdev;
	spin_lock_init(&priv->mda_lock);

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	/* Switch to the main clock */
	bcmasp_core_clock_select(priv, false);

	_intr2_mask_set(priv, 0xffffffff);
	intr2_core_wl(priv, 0xffffffff, ASP_INTR2_CLEAR);

	ret = devm_request_irq(&pdev->dev, priv->irq, bcmasp_isr, 0,
			       pdev->name, priv);
	if (ret) {
		dev_err(dev, "failed to request ASP interrupt: %d\n", ret);
		return ret;
	}

	/* Register mdio child nodes */
	of_platform_populate(dev->of_node, bcmasp_mdio_of_match, NULL,
			     dev);

	/*
	 * ASP specific initialization, Needs to be done irregardless of
	 * of how many interfaces come up.
	 */
	bcmasp_core_init(priv);
	bcmasp_core_init_filters(priv);

	ports_node = of_find_node_by_name(dev->of_node, "ethernet-ports");
	if (!ports_node) {
		dev_warn(dev, "No ports found\n");
		return 0;
	}

	priv->intf_count = of_get_available_child_count(ports_node);

	priv->intfs = devm_kcalloc(dev, priv->intf_count,
				   sizeof(struct bcmasp_intf *),
				   GFP_KERNEL);
	if (!priv->intfs)
		return -ENOMEM;
	/*
	 * Probe each interface (Initalization should continue even if
	 * interfaces are unable to come up)
	 */
	i = 0;
	for_each_available_child_of_node(ports_node, intf_node) {
		wol_irq = platform_get_irq(pdev, i + 1);
		priv->intfs[i++] = bcmasp_interface_create(priv, intf_node,
							   wol_irq);
	}

	/* Drop the clock reference clock now and let ndo_open()/ndo_close()
	 * manage it for us now.
	 */
	clk_disable_unprepare(priv->clk);

	/* Now do the registration of the network ports which will take care of
	 * managing the clock properly.
	 */
	for (i = 0; i < priv->intf_count; i++) {
		intf = priv->intfs[i];
		if (!intf)
			continue;

		ret = register_netdev(intf->ndev);
		if (ret) {
			netdev_err(intf->ndev,
				   "failed to register net_device: %d\n", ret);
			bcmasp_interface_destroy(intf, false);
			return ret;
		}
	}

	dev_info(dev, "Initialized %d port(s)\n", priv->intf_count);

	return 0;
}

static int bcmasp_remove(struct platform_device *pdev)
{

	struct bcmasp_priv *priv = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < priv->intf_count; i++)
		bcmasp_interface_destroy(priv->intfs[i], true);

	return 0;
}

static void bcmasp_shutdown(struct platform_device *pdev)
{
	int ret;

	ret = bcmasp_remove(pdev);
	if (ret)
		dev_err(&pdev->dev, "failed to remove: %d\n", ret);
}

static int __maybe_unused bcmasp_suspend(struct device *d)
{
	struct bcmasp_priv *priv = dev_get_drvdata(d);
	unsigned int i;
	int ret = 0;

	for (i = 0; i < priv->intf_count; i++) {
		ret = bcmasp_interface_suspend(priv->intfs[i]);
		if (ret)
			break;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	/* Switch to the slow clock */
	bcmasp_core_clock_select(priv, true);

	clk_disable_unprepare(priv->clk);

	return ret;
}

static int __maybe_unused bcmasp_resume(struct device *d)
{
	struct bcmasp_priv *priv = dev_get_drvdata(d);
	unsigned int i;
	int ret = 0;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	/* Switch to the main clock domain */
	bcmasp_core_clock_select(priv, false);

	bcmasp_core_init(priv);
	bcmasp_core_init_filters(priv);

	clk_disable_unprepare(priv->clk);

	for (i = 0; i < priv->intf_count; i++) {
		ret = bcmasp_interface_resume(priv->intfs[i]);
		if (ret)
			break;
	}

	return ret;
}

static SIMPLE_DEV_PM_OPS(bcmasp_pm_ops,
			 bcmasp_suspend, bcmasp_resume);

static struct platform_driver bcmasp_driver = {
	.probe = bcmasp_probe,
	.remove = bcmasp_remove,
	.shutdown = bcmasp_shutdown,
	.driver = {
		.name = "brcm,asp-v2",
		.of_match_table = bcmasp_of_match,
		.pm = &bcmasp_pm_ops,
	},
};
module_platform_driver(bcmasp_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom ASP 2.0 Ethernet controller driver");
MODULE_ALIAS("platform:brcm,asp-v2");
MODULE_LICENSE("GPL");
