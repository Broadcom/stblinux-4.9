// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom STB ASP 2.0 Driver
 *
 * Copyright (c) 2020 Broadcom
 */
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
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

static inline void _intr2_mask_clear(struct bcmasp_priv *priv, u32 mask)
{
	priv->irq_mask &= ~mask;
	intr2_core_wl(priv, mask, ASP_INTR2_MASK_CLEAR);
}

static inline void _intr2_mask_set(struct bcmasp_priv *priv, u32 mask)
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

	for (i = 0; i < priv->intf_count; i++) {
		intf = priv->intfs[i];
		if (unlikely(!intf))
			continue;

		if (status & ASP_INTR2_RX_ECH(intf->channel)) {
			if (likely(napi_schedule_prep(&intf->rx_napi))) {
				bcmasp_enable_rx_irq(intf, 0);
				__napi_schedule_irqoff(&intf->rx_napi);
			}
		}

		if (status & ASP_INTR2_TX_DESC(intf->channel)) {
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

static void bcmasp_netfilt_hw_en_wake(struct bcmasp_priv *priv,
				      struct bcmasp_net_filter *nfilt)
{
	rx_filter_core_wl(priv, ASP_RX_FILTER_NET_OFFSET_L3_1(64),
			  ASP_RX_FILTER_NET_OFFSET(nfilt->hw_index));

	rx_filter_core_wl(priv, ASP_RX_FILTER_NET_OFFSET_L2(32) |
			  ASP_RX_FILTER_NET_OFFSET_L3_0(32) |
			  ASP_RX_FILTER_NET_OFFSET_L3_1(96) |
			  ASP_RX_FILTER_NET_OFFSET_L4(32),
			  ASP_RX_FILTER_NET_OFFSET(nfilt->hw_index + 1));

	rx_filter_core_wl(priv, ASP_RX_FILTER_NET_CFG_CH(nfilt->port + 8) |
			  ASP_RX_FILTER_NET_CFG_EN |
			  ASP_RX_FILTER_NET_CFG_L2_EN |
			  ASP_RX_FILTER_NET_CFG_L3_EN |
			  ASP_RX_FILTER_NET_CFG_L4_EN |
			  ASP_RX_FILTER_NET_CFG_L3_FRM(2) |
			  ASP_RX_FILTER_NET_CFG_L4_FRM(2) |
			  ASP_RX_FILTER_NET_CFG_UMC(nfilt->port),
			  ASP_RX_FILTER_NET_CFG(nfilt->hw_index));

	rx_filter_core_wl(priv, ASP_RX_FILTER_NET_CFG_CH(nfilt->port + 8) |
			  ASP_RX_FILTER_NET_CFG_EN |
			  ASP_RX_FILTER_NET_CFG_L2_EN |
			  ASP_RX_FILTER_NET_CFG_L3_EN |
			  ASP_RX_FILTER_NET_CFG_L4_EN |
			  ASP_RX_FILTER_NET_CFG_L3_FRM(2) |
			  ASP_RX_FILTER_NET_CFG_L4_FRM(2) |
			  ASP_RX_FILTER_NET_CFG_UMC(nfilt->port),
			  ASP_RX_FILTER_NET_CFG(nfilt->hw_index + 1));
}

#define MAX_WAKE_FILTER_SIZE		256
enum asp_netfilt_reg_type {
	ASP_NETFILT_MATCH = 0,
	ASP_NETFILT_MASK,
	ASP_NETFILT_MAX
};

static int bcmasp_netfilt_get_reg_offset(struct bcmasp_priv *priv,
					 struct bcmasp_net_filter *nfilt,
					 enum asp_netfilt_reg_type reg_type,
					 u32 offset)
{
	u32 block_index, filter_sel;

	if (offset < 32) {
		block_index = ASP_RX_FILTER_L2;
		filter_sel = nfilt->hw_index;
	} else if (offset < 64) {
		block_index = ASP_RX_FILTER_L2;
		filter_sel = nfilt->hw_index + 1;
	} else if (offset < 96) {
		block_index = ASP_RX_FILTER_L3_0;
		filter_sel = nfilt->hw_index;
	} else if (offset < 128) {
		block_index = ASP_RX_FILTER_L3_0;
		filter_sel = nfilt->hw_index + 1;
	} else if (offset < 160) {
		block_index = ASP_RX_FILTER_L3_1;
		filter_sel = nfilt->hw_index;
	} else if (offset < 192) {
		block_index = ASP_RX_FILTER_L3_1;
		filter_sel = nfilt->hw_index + 1;
	} else if (offset < 224) {
		block_index = ASP_RX_FILTER_L4;
		filter_sel = nfilt->hw_index;
	} else if (offset < 256) {
		block_index = ASP_RX_FILTER_L4;
		filter_sel = nfilt->hw_index + 1;
	}

	switch (reg_type) {
	case ASP_NETFILT_MATCH:
		return ASP_RX_FILTER_NET_PAT(filter_sel, block_index,
					     (offset % 32));
	case ASP_NETFILT_MASK:
		return ASP_RX_FILTER_NET_MASK(filter_sel, block_index,
					      (offset % 32));
	default:
		return -EINVAL;
	}
}

static void bcmasp_netfilt_wr(struct bcmasp_priv *priv,
			      struct bcmasp_net_filter *nfilt,
			      enum asp_netfilt_reg_type reg_type,
			      u32 val, u32 offset)
{
	int reg_offset;

	/* HW only accepts 4 byte aligned writes */
	if (!IS_ALIGNED(offset, 4) || (offset > MAX_WAKE_FILTER_SIZE))
		return;

	reg_offset = bcmasp_netfilt_get_reg_offset(priv, nfilt, reg_type,
						   offset);

	rx_filter_core_wl(priv, val, reg_offset);
}

static u32 bcmasp_netfilt_rd(struct bcmasp_priv *priv,
			     struct bcmasp_net_filter *nfilt,
			     enum asp_netfilt_reg_type reg_type,
			     u32 offset)
{
	int reg_offset;

	/* HW only accepts 4 byte aligned writes */
	if (!IS_ALIGNED(offset, 4) || (offset > MAX_WAKE_FILTER_SIZE))
		return 0;

	reg_offset = bcmasp_netfilt_get_reg_offset(priv, nfilt, reg_type,
						   offset);

	return rx_filter_core_rl(priv, reg_offset);
}

static int bcmasp_netfilt_wr_m_wake(struct bcmasp_priv *priv,
				    struct bcmasp_net_filter *nfilt,
				    u32 offset, void *match, void *mask,
				    size_t size)
{
	u32 shift, mask_val = 0, match_val = 0;
	bool first_byte = true;

	if ((offset + size) > MAX_WAKE_FILTER_SIZE)
		return -EINVAL;

	while (size--) {
		/* The HW only accepts 4 byte aligned writes, so if we
		 * begin unaligned or if remaining bytes less than 4,
		 * we need to read then write to avoid losing current
		 * register state
		 */
		if (first_byte && (!IS_ALIGNED(offset, 4) || (size < 3))) {
			match_val = bcmasp_netfilt_rd(priv, nfilt,
						      ASP_NETFILT_MATCH,
						      ALIGN_DOWN(offset, 4));
			mask_val = bcmasp_netfilt_rd(priv, nfilt,
						     ASP_NETFILT_MASK,
						     ALIGN_DOWN(offset, 4));
		}

		shift = (3 - (offset % 4)) * 8;
		match_val &= ~GENMASK(shift + 7, shift);
		mask_val &= ~GENMASK(shift + 7, shift);
		match_val |= (u32)(*((u8 *)match) << shift);
		mask_val |= (u32)(*((u8 *)mask) << shift);


		/* If last byte or last byte of word, write to reg */
		if (!size || ((offset % 4) == 3)) {
			bcmasp_netfilt_wr(priv, nfilt, ASP_NETFILT_MATCH,
					  match_val, ALIGN_DOWN(offset, 4));
			bcmasp_netfilt_wr(priv, nfilt, ASP_NETFILT_MASK,
					  mask_val, ALIGN_DOWN(offset, 4));
			first_byte = true;
		} else {
			first_byte = false;
		}

		offset++;
		match++;
		mask++;
	}

	return 0;
}

static void bcmasp_netfilt_reset_hw(struct bcmasp_priv *priv,
				    struct bcmasp_net_filter *nfilt)
{
	int i;

	for (i = 0; i < MAX_WAKE_FILTER_SIZE; i += 4) {
		bcmasp_netfilt_wr(priv, nfilt, ASP_NETFILT_MATCH, 0, i);
		bcmasp_netfilt_wr(priv, nfilt, ASP_NETFILT_MASK, 0, i);
	}
}

static void bcmasp_netfilt_tcpip4_wr(struct bcmasp_priv *priv,
				     struct bcmasp_net_filter *nfilt,
				     struct ethtool_tcpip4_spec *match,
				     struct ethtool_tcpip4_spec *mask,
				     u32 offset)
{
	__be16 val_16, mask_16;

	val_16 = htons(ETH_P_IP);
	mask_16 = 0xFFFF;
	bcmasp_netfilt_wr_m_wake(priv, nfilt, (ETH_ALEN * 2) + offset,
				 &val_16, &mask_16, sizeof(val_16));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 1,
				 &match->tos, &mask->tos,
				 sizeof(match->tos));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 12,
				 &match->ip4src, &mask->ip4src,
				 sizeof(match->ip4src));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 16,
				 &match->ip4dst, &mask->ip4dst,
				 sizeof(match->ip4dst));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 20,
				 &match->psrc, &mask->psrc,
				 sizeof(match->psrc));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 22,
				 &match->pdst, &mask->pdst,
				 sizeof(match->pdst));
}

static void bcmasp_netfilt_tcpip6_wr(struct bcmasp_priv *priv,
				     struct bcmasp_net_filter *nfilt,
				     struct ethtool_tcpip6_spec *match,
				     struct ethtool_tcpip6_spec *mask,
				     u32 offset)
{
	__be16 val_16, mask_16;

	val_16 = htons(ETH_P_IPV6);
	mask_16 = 0xFFFF;
	bcmasp_netfilt_wr_m_wake(priv, nfilt, (ETH_ALEN * 2) + offset,
				 &val_16, &mask_16, sizeof(val_16));
	val_16 = match->tclass << 4;
	mask_16 = mask->tclass << 4;
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset,
				 &val_16, &mask_16, sizeof(val_16));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 8,
				 &match->ip6src, &mask->ip6src,
				 sizeof(match->ip6src));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 24,
				 &match->ip6dst, &mask->ip6dst,
				 sizeof(match->ip6dst));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 40,
				 &match->psrc, &mask->psrc,
				 sizeof(match->psrc));
	bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 42,
				 &match->pdst, &mask->pdst,
				 sizeof(match->pdst));
}

static int bcmasp_netfilt_wr_to_hw(struct bcmasp_priv *priv,
				   struct bcmasp_net_filter *nfilt)
{
	struct ethtool_rx_flow_spec *fs = &nfilt->fs;
	unsigned int offset;
	__be16 val_16, mask_16;
	u8 val_8, mask_8;

	/* Shouldn't get here, but just a quick sanity test */
	if (nfilt->reserved) {
		dev_warn(&priv->pdev->dev, "Attempted to use reserved filter\n");
		return -EINVAL;
	}

	/* Currently only supports wake filters */
	if (!nfilt->wake_filter)
		return -EINVAL;

	bcmasp_netfilt_reset_hw(priv, nfilt);

	if (fs->flow_type & FLOW_MAC_EXT) {
		bcmasp_netfilt_wr_m_wake(priv, nfilt, 0, &fs->h_ext.h_dest,
					 &fs->m_ext.h_dest,
					 sizeof(fs->h_ext.h_dest));
	}

	if ((fs->flow_type & FLOW_EXT) &&
	    (fs->m_ext.vlan_etype || fs->m_ext.vlan_tci)) {
		bcmasp_netfilt_wr_m_wake(priv, nfilt, (ETH_ALEN * 2),
					 &fs->h_ext.vlan_etype,
					 &fs->m_ext.vlan_etype,
					 sizeof(fs->h_ext.vlan_etype));
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ((ETH_ALEN * 2) + 2),
					 &fs->h_ext.vlan_tci,
					 &fs->m_ext.vlan_tci,
					 sizeof(fs->h_ext.vlan_tci));
		offset += VLAN_HLEN;
	}

	switch (fs->flow_type & ~(FLOW_EXT | FLOW_MAC_EXT)) {
	case ETHER_FLOW:
		bcmasp_netfilt_wr_m_wake(priv, nfilt, 0,
					 &fs->h_u.ether_spec.h_dest,
					 &fs->m_u.ether_spec.h_dest,
					 sizeof(fs->h_u.ether_spec.h_dest));
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_ALEN,
					 &fs->h_u.ether_spec.h_source,
					 &fs->m_u.ether_spec.h_source,
					 sizeof(fs->h_u.ether_spec.h_source));
		bcmasp_netfilt_wr_m_wake(priv, nfilt, (ETH_ALEN * 2) + offset,
					 &fs->h_u.ether_spec.h_proto,
					 &fs->m_u.ether_spec.h_proto,
					 sizeof(fs->h_u.ether_spec.h_proto));

		break;
	case IP_USER_FLOW:
		val_16 = htons(ETH_P_IP);
		mask_16 = 0xFFFF;
		bcmasp_netfilt_wr_m_wake(priv, nfilt, (ETH_ALEN * 2) + offset,
					 &val_16, &mask_16, sizeof(val_16));
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 1,
					 &fs->h_u.usr_ip4_spec.tos,
					 &fs->m_u.usr_ip4_spec.tos,
					 sizeof(fs->h_u.usr_ip4_spec.tos));
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 9,
					 &fs->h_u.usr_ip4_spec.proto,
					 &fs->m_u.usr_ip4_spec.proto,
					 sizeof(fs->h_u.usr_ip4_spec.proto));
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 12,
					 &fs->h_u.usr_ip4_spec.ip4src,
					 &fs->m_u.usr_ip4_spec.ip4src,
					 sizeof(fs->h_u.usr_ip4_spec.ip4src));
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 16,
					 &fs->h_u.usr_ip4_spec.ip4dst,
					 &fs->m_u.usr_ip4_spec.ip4dst,
					 sizeof(fs->h_u.usr_ip4_spec.ip4dst));
		if (!fs->m_u.usr_ip4_spec.l4_4_bytes)
			break;

		/* Only supports 20 byte IPv4 header */
		val_8 = 0x45;
		mask_8 = 0xFF;
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset,
					 &val_8, &mask_8, sizeof(val_8));
		bcmasp_netfilt_wr_m_wake(priv, nfilt,
					 ETH_HLEN + 20 + offset,
					 &fs->h_u.usr_ip4_spec.l4_4_bytes,
					 &fs->m_u.usr_ip4_spec.l4_4_bytes,
					 sizeof(
					 fs->h_u.usr_ip4_spec.l4_4_bytes));
		break;
	case TCP_V4_FLOW:
		val_8 = IPPROTO_TCP;
		mask_8 = 0xFF;
		bcmasp_netfilt_tcpip4_wr(priv, nfilt, &fs->h_u.tcp_ip4_spec,
					 &fs->m_u.tcp_ip4_spec, offset);
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 9,
					 &val_8, &mask_8, sizeof(val_8));
		break;
	case UDP_V4_FLOW:
		val_8 = IPPROTO_UDP;
		mask_8 = 0xFF;
		bcmasp_netfilt_tcpip4_wr(priv, nfilt, &fs->h_u.udp_ip4_spec,
					 &fs->m_u.udp_ip4_spec, offset);

		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 9,
					 &val_8, &mask_8, sizeof(val_8));
		break;
	case TCP_V6_FLOW:
		val_8 = IPPROTO_TCP;
		mask_8 = 0xFF;
		bcmasp_netfilt_tcpip6_wr(priv, nfilt, &fs->h_u.tcp_ip6_spec,
					 &fs->m_u.tcp_ip6_spec, offset);
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 6,
					 &val_8, &mask_8, sizeof(val_8));
		break;
	case UDP_V6_FLOW:
		val_8 = IPPROTO_UDP;
		mask_8 = 0xFF;
		bcmasp_netfilt_tcpip6_wr(priv, nfilt, &fs->h_u.udp_ip6_spec,
					 &fs->m_u.udp_ip6_spec, offset);
		bcmasp_netfilt_wr_m_wake(priv, nfilt, ETH_HLEN + offset + 6,
					 &val_8, &mask_8, sizeof(val_8));
		break;
	}

	bcmasp_netfilt_hw_en_wake(priv, nfilt);

	return 0;
}

void bcmasp_netfilt_suspend(struct bcmasp_intf *intf)
{
	struct bcmasp_priv *priv = intf->parent;
	bool write = false;
	int ret, i;

	/* Write all filters to HW */
	for (i = 0; i < ASP_RX_NET_FILTER_MAX; i++) {
		/* If the filter is reserved by firmware or does not match
		 * the port, skip programming.
		 */
		if (priv->net_filters[i].reserved ||
		    (priv->net_filters[i].port != intf->port))
			continue;

		if (i > 0 && (i % 2) &&
		    priv->net_filters[i].wake_filter &&
		    priv->net_filters[i - 1].wake_filter)
			continue;

		ret = bcmasp_netfilt_wr_to_hw(priv, &priv->net_filters[i]);
		if (!ret)
			write = true;
	}

	/* Successfully programmed at least one wake filter
	 * so enable top level wake config
	 */
	if (write)
		rx_filter_core_wl(priv, (ASP_RX_FILTER_OPUT_EN |
				  ASP_RX_FILTER_MDA_EN |
				  ASP_RX_FILTER_LNR_MD |
				  ASP_RX_FILTER_GEN_WK_EN |
				  ASP_RX_FILTER_NT_FLT_EN),
				  ASP_RX_FILTER_BLK_CTRL);
}

int bcmasp_netfilt_get_max(struct bcmasp_intf *intf)
{
	return intf->parent->net_filters_count_max;
}

void bcmasp_netfilt_get_all_active(struct bcmasp_intf *intf, u32 *rule_locs,
				   u32 *rule_cnt)
{
	struct bcmasp_priv *priv = intf->parent;
	int j = 0, i;

	for (i = 0; i < ASP_RX_NET_FILTER_MAX; i++) {
		if (priv->net_filters[i].reserved ||
		    !priv->net_filters[i].claimed ||
		    (priv->net_filters[i].port != intf->port))
			continue;

		if (i > 0 && (i % 2) &&
		    priv->net_filters[i].wake_filter &&
		    priv->net_filters[i - 1].wake_filter)
			continue;

		rule_locs[j++] = priv->net_filters[i].fs.location;
	}

	*rule_cnt = j;
}

int bcmasp_netfilt_get_active(struct bcmasp_intf *intf)
{
	struct bcmasp_priv *priv = intf->parent;
	int cnt = 0, i;

	for (i = 0; i < ASP_RX_NET_FILTER_MAX; i++) {
		if (priv->net_filters[i].reserved ||
		    !priv->net_filters[i].claimed ||
		    (priv->net_filters[i].port != intf->port))
			continue;

		/* Skip over a wake filter pair */
		if (i > 0 && (i % 2) &&
		    priv->net_filters[i].wake_filter &&
		    priv->net_filters[i - 1].wake_filter)
			continue;

		cnt++;
	}

	return cnt;
}

int bcmasp_netfilt_check_dup(struct bcmasp_intf *intf,
			     struct ethtool_rx_flow_spec *fs)
{
	struct bcmasp_priv *priv = intf->parent;
	struct ethtool_rx_flow_spec *cur;
	size_t fs_size = 0;
	int i;

	for ( i = 0; i < ASP_RX_NET_FILTER_MAX; i++) {

		if (priv->net_filters[i].reserved ||
		    !priv->net_filters[i].claimed ||
		    (priv->net_filters[i].port != intf->port)) {
			continue;
		}

		cur = &priv->net_filters[i].fs;

		if (cur->flow_type != fs->flow_type ||
		    cur->ring_cookie != fs->ring_cookie)
			continue;

		switch (fs->flow_type & ~(FLOW_EXT | FLOW_MAC_EXT)) {
		case ETHER_FLOW:
			fs_size = sizeof(struct ethhdr);
			break;
		case IP_USER_FLOW:
			fs_size = sizeof(struct ethtool_usrip4_spec);
			break;
		case TCP_V6_FLOW:
		case UDP_V6_FLOW:
			fs_size = sizeof(struct ethtool_tcpip6_spec);
			break;
		case TCP_V4_FLOW:
		case UDP_V4_FLOW:
			fs_size = sizeof(struct ethtool_tcpip4_spec);
			break;
		default:
			continue;
		}

		if (memcmp(&cur->h_u, &fs->h_u, fs_size) ||
		    memcmp(&cur->m_u, &fs->m_u, fs_size))
			continue;

		if (cur->flow_type & FLOW_EXT) {
			if ((cur->h_ext.vlan_etype != fs->h_ext.vlan_etype) ||
			    (cur->m_ext.vlan_etype != fs->m_ext.vlan_etype) ||
			    (cur->h_ext.vlan_tci != fs->h_ext.vlan_tci) ||
			    (cur->m_ext.vlan_tci != fs->m_ext.vlan_tci) ||
			    (cur->h_ext.data[0] != fs->h_ext.data[0]))
				continue;
		}
		if (cur->flow_type & FLOW_MAC_EXT) {
			if (memcmp(&cur->h_ext.h_dest,
			    &fs->h_ext.h_dest, ETH_ALEN) ||
			    memcmp(&cur->m_ext.h_dest,
			    &fs->m_ext.h_dest, ETH_ALEN))
				continue;
		}

		return 1;
	}

	return 0;
}

/* If no network filter found, return open filter.
 * If no more open filters return NULL
 */
struct bcmasp_net_filter *bcmasp_netfilt_get_init(struct bcmasp_intf *intf,
						  int loc, bool wake_filter,
						  bool init)
{
	struct bcmasp_priv *priv = intf->parent;
	struct bcmasp_net_filter *nfilter = NULL;
	int i, open_index = -1;

	/* Check whether we exceed the filter table capacity */
	if (loc != RX_CLS_LOC_ANY && loc >= ASP_RX_NET_FILTER_MAX)
		return ERR_PTR(-EINVAL);

	/* If the filter location is busy (already claimed or reserved) and we
	 * are initializing the filter (insertion), return a busy error code.
	 */
	if (loc != RX_CLS_LOC_ANY && init &&
	    (priv->net_filters[loc].claimed || priv->net_filters[loc].reserved))
		return ERR_PTR(-EBUSY);

	/* We need two filters for wake-up, so we cannot use an odd filter */
	if (wake_filter && loc != RX_CLS_LOC_ANY && (loc % 2))
		return ERR_PTR(-EINVAL);

	/* Initialize the loop index based on the desired location or from 0 */
	i = loc == RX_CLS_LOC_ANY ? 0 : loc;

	for ( ; i < ASP_RX_NET_FILTER_MAX; i++) {
		if (priv->net_filters[i].reserved)
			continue;

		/* Found matching network filter */
		if (!init &&
		    priv->net_filters[i].claimed &&
		    (priv->net_filters[i].hw_index == i) &&
		    (priv->net_filters[i].port == intf->port))
			return &priv->net_filters[i];

		/* If we don't need a new filter or new filter already found */
		if (!init || open_index >= 0)
			continue;

		/* Wake filter conslidates two filters to cover more bytes
		 * Wake filter is open if...
		 * 1. It is an even filter
		 * 2. The current and next filter is not claimed
		 */
		if (wake_filter && !(i % 2) && !priv->net_filters[i].claimed &&
		    !priv->net_filters[i + 1].claimed)
			open_index = i;
		else if (!priv->net_filters[i].claimed)
			open_index = i;
	}

	if (open_index >= 0) {
		nfilter = &priv->net_filters[open_index];
		nfilter->claimed = true;
		nfilter->port = intf->port;
		nfilter->hw_index = open_index;
	}

	if (wake_filter && (open_index >= 0)) {
		/* Claim next filter */
		priv->net_filters[open_index + 1].claimed = true;
		priv->net_filters[open_index + 1].wake_filter = true;
		nfilter->wake_filter = true;
	}

	return nfilter ? nfilter : ERR_PTR(-EINVAL);
}

void bcmasp_netfilt_release(struct bcmasp_intf *intf,
			    struct bcmasp_net_filter *nfilt)
{
	struct bcmasp_priv *priv = intf->parent;

	if (nfilt->wake_filter) {
		memset(&priv->net_filters[nfilt->hw_index + 1], 0,
		       sizeof(struct bcmasp_net_filter));
	}

	memset(nfilt, 0, sizeof(struct bcmasp_net_filter));
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

/* There are 32 MDA filters shared between all ports, we reserve 4 filters per
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

#define ASP_RX_FILT_RES_COUNT(intf)	((intf)->parent->intf_count \
					 * ASP_RX_FILTER_RES_COUNT)

#define ASP_RX_FILT(intf, name)		(((intf)->port * \
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

#define mac_to_u64(a)		((((u64)a[0]) << 40) | \
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

	/* We can only combine filters in two cases
	 * 1. They share the same mask and are different by one bit
	 * 2. One filter resides within the other
	 */
	if (mask1 == mask2) {
		if (!differ_one_bit((addr1 & mask1), (addr2 & mask2)))
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

	/* Top level filter enable bit should be enabled at all times, set
	 * GEN_WAKE_CLEAR to clear the network filter wake-up which would
	 * otherwise be sticky
	 */
	rx_filter_core_wl(priv, (ASP_RX_FILTER_OPUT_EN |
			  ASP_RX_FILTER_MDA_EN |
			  ASP_RX_FILTER_GEN_WK_CLR |
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

	/* Disable and clear both UniMAC's wake-up interrupts to avoid
	 * sticky interrupts.
	 */
	_intr2_mask_set(priv, ASP_INTR2_UMC0_WAKE | ASP_INTR2_UMC1_WAKE);
	intr2_core_wl(priv, ASP_INTR2_UMC0_WAKE | ASP_INTR2_UMC1_WAKE,
		      ASP_INTR2_CLEAR);
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

static void bcmasp_core_clock_set_ll(struct bcmasp_priv *priv, u32 clr, u32 set)
{
	u32 reg;

	reg = ctrl_core_rl(priv, ASP_CTRL_CLOCK_CTRL);
	reg &= ~clr;
	reg |= set;
	ctrl_core_wl(priv, reg, ASP_CTRL_CLOCK_CTRL);
}

static void bcmasp_core_clock_set(struct bcmasp_priv *priv, u32 clr, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->clk_lock, flags);
	bcmasp_core_clock_set_ll(priv, clr, set);
	spin_unlock_irqrestore(&priv->clk_lock, flags);
}

void bcmasp_core_clock_set_intf(struct bcmasp_intf *intf, bool en)
{
	u32 intf_mask = ASP_CTRL_CLOCK_CTRL_ASP_RGMII_DIS(intf->channel);
	struct bcmasp_priv *priv = intf->parent;
	unsigned long flags;
	u32 reg;

	/* When enabling an interface, if the RX or TX clocks were not enabled,
	 * enable them. Conversely, while disabling an interface, if this is
	 * the last one enabled, we can turn off the shared RX and TX clocks as
	 * well. We control enable bits which is why we test for equality on
	 * the RGMII clock bit mask.
	 */
	spin_lock_irqsave(&priv->clk_lock, flags);
	if (en) {
		intf_mask |= ASP_CTRL_CLOCK_CTRL_ASP_TX_DISABLE |
			     ASP_CTRL_CLOCK_CTRL_ASP_RX_DISABLE;
		bcmasp_core_clock_set_ll(priv, intf_mask, 0);
	} else {
		reg = ctrl_core_rl(priv, ASP_CTRL_CLOCK_CTRL) | intf_mask;
		if ((reg & ASP_CTRL_CLOCK_CTRL_ASP_RGMII_MASK) ==
		    ASP_CTRL_CLOCK_CTRL_ASP_RGMII_MASK)
			intf_mask |= ASP_CTRL_CLOCK_CTRL_ASP_TX_DISABLE |
				     ASP_CTRL_CLOCK_CTRL_ASP_RX_DISABLE;
		bcmasp_core_clock_set_ll(priv, 0, intf_mask);
	}
	spin_unlock_irqrestore(&priv->clk_lock, flags);
}

static int bcmasp_probe(struct platform_device *pdev)
{
	struct bcmasp_priv *priv;
	struct device_node *ports_node, *intf_node;
	struct device *dev = &pdev->dev;
	int ret, i, wol_irq, count = 0;
	struct bcmasp_intf *intf;
	struct resource *r;
	u32 u32_reserved_filters_bitmask;
	DECLARE_BITMAP(reserved_filters_bitmask, ASP_RX_NET_FILTER_MAX);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

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
	spin_lock_init(&priv->clk_lock);
	mutex_init(&priv->net_lock);

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	/* Enable all clocks to ensure successful probing */
	bcmasp_core_clock_set(priv, ASP_CTRL_CLOCK_CTRL_ASP_ALL_DISABLE, 0);

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

	ret = of_property_read_u32(dev->of_node,
				   "brcm,reserved-net-filters-mask",
				   &u32_reserved_filters_bitmask);
	if (ret)
		u32_reserved_filters_bitmask = 0;

	priv->net_filters_count_max = ASP_RX_NET_FILTER_MAX;
	bitmap_zero(reserved_filters_bitmask, priv->net_filters_count_max);
	bitmap_from_u32array(reserved_filters_bitmask,
			     priv->net_filters_count_max,
			     &u32_reserved_filters_bitmask, 1);

	/* Discover bitmask of reserved filters */
	for_each_set_bit(i, reserved_filters_bitmask, ASP_RX_NET_FILTER_MAX) {
		priv->net_filters[i].reserved = true;
		priv->net_filters_count_max--;
	}

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

	/* Probe each interface (Initalization should continue even if
	 * interfaces are unable to come up)
	 */
	i = 0;
	for_each_available_child_of_node(ports_node, intf_node) {
		wol_irq = platform_get_irq(pdev, i + 1);
		priv->intfs[i++] = bcmasp_interface_create(priv, intf_node,
							   wol_irq);
	}

	/* Drop the clock reference count now and let ndo_open()/ndo_close()
	 * manage it for us from now on.
	 */
	bcmasp_core_clock_set(priv, 0, ASP_CTRL_CLOCK_CTRL_ASP_ALL_DISABLE);

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
			continue;
		}
		count++;
	}

	dev_info(dev, "Initialized %d port(s)\n", count);

	return 0;
}

static int bcmasp_remove(struct platform_device *pdev)
{
	struct bcmasp_priv *priv = dev_get_drvdata(&pdev->dev);
	struct bcmasp_intf *intf;
	int i;

	for (i = 0; i < priv->intf_count; i++) {
		intf = priv->intfs[i];
		if (!intf)
			continue;

		bcmasp_interface_destroy(intf, true);
	}

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
	struct bcmasp_intf *intf;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < priv->intf_count; i++) {
		intf = priv->intfs[i];
		if (!intf)
			continue;

		ret = bcmasp_interface_suspend(intf);
		if (ret)
			break;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	/* Whether Wake-on-LAN is enabled or not, we can always disable
	 * the shared TX clock
	 */
	bcmasp_core_clock_set(priv, 0, ASP_CTRL_CLOCK_CTRL_ASP_TX_DISABLE);

	/* Switch to the slow clock */
	bcmasp_core_clock_select(priv, true);

	clk_disable_unprepare(priv->clk);

	return ret;
}

static int __maybe_unused bcmasp_resume(struct device *d)
{
	struct bcmasp_priv *priv = dev_get_drvdata(d);
	struct bcmasp_intf *intf;
	unsigned int i;
	int ret = 0;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	/* Switch to the main clock domain */
	bcmasp_core_clock_select(priv, false);

	/* Re-enable all clocks for re-initialization */
	bcmasp_core_clock_set(priv, ASP_CTRL_CLOCK_CTRL_ASP_ALL_DISABLE, 0);

	bcmasp_core_init(priv);
	bcmasp_core_init_filters(priv);

	/* And disable them to let the network devices take care of them */
	bcmasp_core_clock_set(priv, 0, ASP_CTRL_CLOCK_CTRL_ASP_ALL_DISABLE);

	clk_disable_unprepare(priv->clk);

	for (i = 0; i < priv->intf_count; i++) {
		intf = priv->intfs[i];
		if (!intf)
			continue;

		ret = bcmasp_interface_resume(intf);
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
