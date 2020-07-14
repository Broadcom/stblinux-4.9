#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/platform_device.h>
#include <net/ip.h>
#include <net/ipv6.h>

#include "bcmasp.h"
#include "bcmasp_intf_defs.h"


static inline int incr_ring(int index, int ring_count)
{
	index++;
	if (index == ring_count)
		return 0;

	return index;
}

/* Points to last byte of descriptor */
static inline dma_addr_t incr_last_byte(dma_addr_t addr, dma_addr_t beg,
				 int ring_count)
{
	dma_addr_t end = beg + (ring_count * DESC_SIZE);

	addr += DESC_SIZE;
	if (addr > end)
		return beg + DESC_SIZE - 1;

	return addr;
}

/* Points to first byte of descriptor */
static inline dma_addr_t incr_first_byte(dma_addr_t addr, dma_addr_t beg,
				  int ring_count)
{
	dma_addr_t end = beg + (ring_count * DESC_SIZE);

	addr += DESC_SIZE;
	if (addr >= end)
		return beg;

	return addr;
}

static inline void bcmasp_enable_tx(struct bcmasp_intf *intf, int en)
{
	if (en) {
		tx_spb_ctrl_wl(intf, TX_SPB_CTRL_ENABLE_EN, TX_SPB_CTRL_ENABLE);
		tx_epkt_core_wl(intf, (TX_EPKT_C_CFG_MISC_EN |
				TX_EPKT_C_CFG_MISC_PT |
			       (intf->port << TX_EPKT_C_CFG_MISC_PS_SHIFT)),
				TX_EPKT_C_CFG_MISC);
	} else {
		tx_spb_ctrl_wl(intf, 0x0, TX_SPB_CTRL_ENABLE);
		tx_epkt_core_wl(intf, 0x0, TX_EPKT_C_CFG_MISC);
	}

}

static inline void bcmasp_enable_rx(struct bcmasp_intf *intf, int en)
{
	if (en)
		rx_edpkt_cfg_wl(intf, RX_EDPKT_CFG_ENABLE_EN, RX_EDPKT_CFG_ENABLE);
	else
		rx_edpkt_cfg_wl(intf, 0x0, RX_EDPKT_CFG_ENABLE);
}

static void bcmasp_set_rx_mode(struct net_device *dev)
{
	struct bcmasp_intf *intf = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	unsigned char mask[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	int ret;

	spin_lock_bh(&intf->parent->mda_lock);

	bcmasp_disable_all_filters(intf);

	if (dev->flags & IFF_PROMISC)
		goto set_promisc;

	bcmasp_set_promisc(intf, 0);

	bcmasp_set_broad(intf, 1);

	bcmasp_set_oaddr(intf, dev->dev_addr, 1);

	if (dev->flags & IFF_ALLMULTI) {
		bcmasp_set_allmulti(intf, 1);
	} else {
		bcmasp_set_allmulti(intf, 0);

		netdev_for_each_mc_addr(ha, dev) {
			ret = bcmasp_set_en_mda_filter(intf, ha->addr, mask);
			if (ret) {
				intf->mib.mc_filters_full_cnt++;
				goto set_promisc;
			}
		}
	}

	netdev_for_each_uc_addr(ha, dev) {
		ret = bcmasp_set_en_mda_filter(intf, ha->addr, mask);
		if (ret) {
			intf->mib.uc_filters_full_cnt++;
			goto set_promisc;
		}
	}

	spin_unlock_bh(&intf->parent->mda_lock);
	return;

set_promisc:
	bcmasp_set_promisc(intf, 1);
	intf->mib.promisc_filters_cnt++;

	/* disable all filters used by this port */
	bcmasp_disable_all_filters(intf);

	spin_unlock_bh(&intf->parent->mda_lock);
}

static int tx_spb_ring_full(struct bcmasp_intf *intf)
{
	int next_index;

	/* Grab next index, if it is the clean index, ring is full */
	next_index = incr_ring(intf->tx_spb_index, DESC_RING_COUNT);
	if (next_index == intf->tx_spb_clean_index)
		return 1;

	return 0;
}

static netdev_tx_t bcmasp_xmit(struct sk_buff *skb,
			struct net_device *dev)
{
	struct bcmasp_intf *intf = netdev_priv(dev);
	struct device *kdev = &intf->parent->pdev->dev;
	struct bcmasp_desc *desc;
	struct bcmasp_tx_cb *txcb;
	dma_addr_t mapping;
	int ret;

	spin_lock(&intf->tx_lock);

	if (tx_spb_ring_full(intf)) {
		netif_stop_queue(dev);
		netdev_err(dev, "Tx Ring Full!\n");
		ret = NETDEV_TX_BUSY;
		goto out;
	}

	mapping = dma_map_single(kdev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(kdev, mapping)) {
		netif_err(intf, tx_err, dev,"DMA map failed at %p (len=%d\n",
			skb->data, skb->len);
		ret = NETDEV_TX_OK;
		intf->mib.tx_dma_failed++;
		goto out;
	}

	txcb = &intf->tx_cbs[intf->tx_spb_index];
	txcb->skb = skb;
	dma_unmap_addr_set(txcb, dma_addr, mapping);
	dma_unmap_len_set(txcb, dma_len, skb->len);

	desc = &intf->tx_spb_cpu[intf->tx_spb_index];
	desc->buf = mapping;
	desc->size = skb->len;
	desc->flags = DESC_INT_EN | DESC_SOF | DESC_EOF;

	wmb();

	netif_dbg(intf, tx_queued, dev,
		  "%s dma_buf=%pad dma_len=0x%x flags=0x%x index=0x%x\n",
		  __func__, &mapping, desc->size, desc->flags,
		  intf->tx_spb_index);

	intf->tx_spb_index = incr_ring(intf->tx_spb_index, DESC_RING_COUNT);
	intf->tx_spb_dma_valid = incr_last_byte(intf->tx_spb_dma_valid,
						intf->tx_spb_dma,
						DESC_RING_COUNT);

	tx_spb_dma_wq(intf, intf->tx_spb_dma_valid, TX_SPB_DMA_VALID);

	if (tx_spb_ring_full(intf))
		netif_stop_queue(dev);

	ret = NETDEV_TX_OK;
out:
	spin_unlock(&intf->tx_lock);
	return ret;
}

static void bcmasp_netif_start(struct net_device *dev)
{
	struct bcmasp_intf *intf = netdev_priv(dev);

	intf->crc_fwd = !!(umac_rl(intf, UMC_CMD) & UMC_CMD_CRC_FWD);

	bcmasp_set_rx_mode(dev);
	napi_enable(&intf->tx_napi);
	napi_enable(&intf->rx_napi);

	bcmasp_enable_rx_irq(intf, 1);
	bcmasp_enable_tx_irq(intf, 1);

	phy_start(dev->phydev);
}

static inline void umac_reset(struct bcmasp_intf *intf)
{
	umac_wl(intf, 0x0, UMC_CMD);
	umac_wl(intf, UMC_CMD_SW_RESET, UMC_CMD);
	udelay(10);
	umac_wl(intf, 0x0, UMC_CMD);
}

static void umac_set_hw_addr(struct bcmasp_intf *intf,
			     unsigned char *addr)
{
	u32 mac0 = (addr[0] << 24) | (addr[1] << 16) | (addr[2] << 8) |
		    addr[3];
	u32 mac1 = (addr[4] << 8) | addr[5];

	umac_wl(intf, mac0, UMC_MAC0);
	umac_wl(intf, mac1, UMC_MAC1);
}

static inline void umac_enable_set(struct bcmasp_intf *intf,
				   u32 mask, unsigned int enable)
{
	u32 reg;

	reg = umac_rl(intf, UMC_CMD);
	if (enable)
		reg |= mask;
	else
		reg &= ~mask;
	umac_wl(intf, reg, UMC_CMD);

	/*
	 * UniMAC stops on a packet boundary, wait for a full-sized packet
	 * to be processed (1 msec).
	 */
	if (enable == 0)
		usleep_range(1000, 2000);
}

static inline void umac_init(struct bcmasp_intf *intf)
{
	umac_wl(intf, 0x800, UMC_FRM_LEN);
	umac_wl(intf, 0xffff, UMC_PAUSE_CNTRL);
	umac_wl(intf, 0x800, UMC_RX_MAX_PKT_SZ);
}

static int bcmasp_tx_poll(struct napi_struct *napi, int budget)
{
	struct bcmasp_intf *intf =
		container_of(napi, struct bcmasp_intf, tx_napi);
	struct device *kdev = &intf->parent->pdev->dev;
	struct bcmasp_tx_cb *txcb;
	struct bcmasp_desc *desc;
	unsigned long read, released = 0;
	dma_addr_t mapping;

	read = tx_spb_dma_rq(intf, TX_SPB_DMA_READ);
	while (intf->tx_spb_dma_read != read) {
		txcb = &intf->tx_cbs[intf->tx_spb_clean_index];
		desc = &intf->tx_spb_cpu[intf->tx_spb_clean_index];
		mapping = dma_unmap_addr(txcb, dma_addr);

		dma_unmap_single(kdev, mapping,
				 dma_unmap_len(txcb, dma_len),
				 DMA_TO_DEVICE);

		intf->ndev->stats.tx_packets++;
		intf->ndev->stats.tx_bytes += txcb->skb->len;

		dev_consume_skb_any(txcb->skb);
		txcb->skb = NULL;
		dma_unmap_addr_set(txcb, dma_addr, 0);
		released++;

		netif_dbg(intf, tx_done, intf->ndev,
			  "%s dma_buf=%pad dma_len=0x%x flags=0x%x c_index=0x%x\n",
			  __func__, &mapping, desc->size, desc->flags,
			  intf->tx_spb_clean_index);

		desc->buf = 0x0;
		desc->size = 0x0;
		desc->flags = 0x0;

		intf->tx_spb_clean_index = incr_ring(intf->tx_spb_clean_index,
						     DESC_RING_COUNT);
		intf->tx_spb_dma_read = incr_first_byte(intf->tx_spb_dma_read,
							intf->tx_spb_dma,
							DESC_RING_COUNT);
	}

	wmb();

	napi_complete(&intf->tx_napi);

	bcmasp_enable_tx_irq(intf, 1);

	if (released)
		netif_wake_queue(intf->ndev);

	return 0;
}

static int bcmasp_rx_poll(struct napi_struct *napi, int budget)
{
	struct bcmasp_intf *intf =
		container_of(napi, struct bcmasp_intf, rx_napi);
	struct device *kdev = &intf->parent->pdev->dev;
	struct bcmasp_desc *desc;
	struct sk_buff *skb;
	unsigned long processed = 0;
	dma_addr_t valid;
	void *data;
	u64 flags;
	u32 len;

	valid = rx_edpkt_dma_rq(intf, RX_EDPKT_DMA_VALID) + 1;
	if (valid == intf->rx_edpkt_dma + DESC_RING_SIZE) {
		valid = intf->rx_edpkt_dma;
	}

	while ((processed < budget) && (valid != intf->rx_edpkt_dma_read))
	{

		desc = &intf->rx_edpkt_cpu[intf->rx_edpkt_index];

		rmb();

		/* Calculate virt addr by offsetting from physical addr */
		data = intf->rx_ring_cpu + (DESC_ADDR(desc->buf) - intf->rx_ring_dma);

		flags = DESC_FLAGS(desc->buf);
		if (unlikely(flags & (DESC_CRC_ERR | DESC_RX_SYM_ERR))) {
			netif_err(intf, rx_status, intf->ndev, "flags=0x%llx\n",
				  flags);

			intf->ndev->stats.rx_errors++;
			intf->ndev->stats.rx_dropped++;
			goto next;
		}

		dma_sync_single_for_cpu(kdev, DESC_ADDR(desc->buf), desc->size,
					DMA_FROM_DEVICE);

		len = desc->size;

		skb = __netdev_alloc_skb(intf->ndev, len, GFP_ATOMIC | __GFP_NOWARN);
		if (!skb) {
			intf->ndev->stats.rx_errors++;
			intf->mib.alloc_rx_skb_failed++;
			netif_warn(intf, rx_err, intf->ndev,
				   "SKB alloc failed\n");
			goto next;
		}

		skb_put(skb, len);
		memcpy(skb->data, data, len);

		skb_pull(skb, 2);
		len -= 2;
		if (likely(intf->crc_fwd)) {
			skb_trim(skb, len - ETH_FCS_LEN);
			len -= ETH_FCS_LEN;
		}

		skb->protocol = eth_type_trans(skb, intf->ndev);

		napi_gro_receive(napi, skb);

		intf->ndev->stats.rx_packets++;
		intf->ndev->stats.rx_bytes += len;

		rx_edpkt_cfg_wq(intf, (DESC_ADDR(desc->buf) + desc->size), RX_EDPKT_RING_BUFFER_READ);
next:
		processed++;
		intf->rx_edpkt_dma_read =
			incr_first_byte(intf->rx_edpkt_dma_read,
				       intf->rx_edpkt_dma, DESC_RING_COUNT);
		intf->rx_edpkt_index = incr_ring(intf->rx_edpkt_index,
						 DESC_RING_COUNT);
	}

	rx_edpkt_dma_wq(intf, intf->rx_edpkt_dma_read, RX_EDPKT_DMA_READ);

	if (processed < budget) {
		napi_complete_done(&intf->rx_napi, processed);
		bcmasp_enable_rx_irq(intf, 1);
	}

	return processed;
}

static void bcmasp_adj_link(struct net_device *dev)
{
	struct bcmasp_intf *intf = netdev_priv(dev);
	struct phy_device *phydev = dev->phydev;
	int changed = 0;
	u32 cmd_bits = 0, reg;

	if (intf->old_link != phydev->link) {
		changed = 1;
		intf->old_link = phydev->link;
	}

	if (intf->old_duplex != phydev->duplex) {
		changed = 1;
		intf->old_duplex = phydev->duplex;
	}

	switch (phydev->speed) {
	case SPEED_2500:
		cmd_bits = UMC_CMD_SPEED_2500;
		break;
	case SPEED_1000:
		cmd_bits = UMC_CMD_SPEED_1000;
		break;
	case SPEED_100:
		cmd_bits = UMC_CMD_SPEED_100;
		break;
	case SPEED_10:
		cmd_bits = UMC_CMD_SPEED_10;
		break;
	default:
		break;
	}
	cmd_bits <<= UMC_CMD_SPEED_SHIFT;

	if (phydev->duplex == DUPLEX_HALF)
		cmd_bits |= UMC_CMD_HD_EN;

	if (intf->old_pause != phydev->pause) {
		changed = 1;
		intf->old_pause = phydev->pause;
	}

	if (!phydev->pause)
		cmd_bits |= UMC_CMD_RX_PAUSE_IGNORE | UMC_CMD_TX_PAUSE_IGNORE;

	if (!changed)
		return;

	if (phydev->link) {
		reg = umac_rl(intf, UMC_CMD);
		reg &= ~((UMC_CMD_SPEED_MASK << UMC_CMD_SPEED_SHIFT) |
			UMC_CMD_HD_EN | UMC_CMD_RX_PAUSE_IGNORE |
			UMC_CMD_TX_PAUSE_IGNORE);
		reg |= cmd_bits;
		umac_wl(intf, reg, UMC_CMD);

		/* Enable RGMII pad */
		reg = rgmii_rl(intf, RGMII_OOB_CNTRL);
		reg |= RGMII_MODE_EN;
		rgmii_wl(intf, reg, RGMII_OOB_CNTRL);
	} else {
		/* Disable RGMII pad */
		reg = rgmii_rl(intf, RGMII_OOB_CNTRL);
		reg &= ~RGMII_MODE_EN;
		rgmii_wl(intf, reg, RGMII_OOB_CNTRL);
	}

	if (changed)
		phy_print_status(phydev);

}

static int bcmasp_init_rx(struct bcmasp_intf *intf)
{
	struct device *kdev = &intf->parent->pdev->dev;
	struct net_device *ndev = intf->ndev;
	void *p;
	dma_addr_t dma;
	struct page *buffer_pg;
	int ret;

	intf->rx_buf_order = get_order(RING_BUFFER_SIZE);
	buffer_pg = alloc_pages(GFP_KERNEL, intf->rx_buf_order);

	dma = dma_map_page(kdev, buffer_pg, 0, RING_BUFFER_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(kdev, dma)) {
		netdev_err(ndev, "Cannot allocate RX buffer\n");
		__free_pages(virt_to_page(intf->rx_ring_cpu), intf->rx_buf_order);
		return -ENOMEM;
	}
	intf->rx_ring_cpu = page_to_virt(buffer_pg);
	intf->rx_ring_dma = dma;
	intf->rx_ring_dma_valid = intf->rx_ring_dma + RING_BUFFER_SIZE - 1;

	p = dma_zalloc_coherent(kdev, DESC_RING_SIZE, &intf->rx_edpkt_dma,
				GFP_KERNEL);
	if (!p) {
		netdev_err(ndev, "Cannot allocate edpkt desc ring\n");
		ret = -ENOMEM;
		goto free_rx_ring;
	}
	intf->rx_edpkt_cpu = p;
	intf->rx_edpkt_dma_read = intf->rx_edpkt_dma;

	intf->rx_edpkt_index = 0;

	netif_napi_add(intf->ndev, &intf->rx_napi, bcmasp_rx_poll,
		       NAPI_POLL_WEIGHT);

	/* Make sure channels are disabled */
	rx_edpkt_cfg_wl(intf, 0x0, RX_EDPKT_CFG_ENABLE);

	/* Rx SPB */
	rx_edpkt_cfg_wq(intf, intf->rx_ring_dma, RX_EDPKT_RING_BUFFER_READ);
	rx_edpkt_cfg_wq(intf, intf->rx_ring_dma, RX_EDPKT_RING_BUFFER_WRITE);
	rx_edpkt_cfg_wq(intf, intf->rx_ring_dma, RX_EDPKT_RING_BUFFER_BASE);
	rx_edpkt_cfg_wq(intf, intf->rx_ring_dma_valid, RX_EDPKT_RING_BUFFER_END);
	rx_edpkt_cfg_wq(intf, intf->rx_ring_dma_valid, RX_EDPKT_RING_BUFFER_VALID);

	/* EDPKT */
	rx_edpkt_cfg_wl(intf, (RX_EDPKT_CFG_CFG0_RBUF_4K <<
			RX_EDPKT_CFG_CFG0_DBUF_SHIFT) |
		       (RX_EDPKT_CFG_CFG0_64_ALN <<
			RX_EDPKT_CFG_CFG0_BALN_SHIFT) |
		       (RX_EDPKT_CFG_CFG0_EFRM_STUF),
			RX_EDPKT_CFG_CFG0);
	rx_edpkt_dma_wq(intf, intf->rx_edpkt_dma, RX_EDPKT_DMA_WRITE);
	rx_edpkt_dma_wq(intf, intf->rx_edpkt_dma, RX_EDPKT_DMA_READ);
	rx_edpkt_dma_wq(intf, intf->rx_edpkt_dma, RX_EDPKT_DMA_BASE);
	rx_edpkt_dma_wq(intf, intf->rx_edpkt_dma + (DESC_RING_SIZE - 1),
			RX_EDPKT_DMA_END);
	rx_edpkt_dma_wq(intf, intf->rx_edpkt_dma + (DESC_RING_SIZE - 1),
			RX_EDPKT_DMA_VALID);

	/* RX Unimac Init */
	umac2fb_wl(intf, UMAC2FB_CFG_DEFAULT_EN |
		  ((intf->channel + 13) << UMAC2FB_CFG_CHID_SHIFT) |
		  (0xc << UMAC2FB_CFG_OK_SEND_SHIFT), UMAC2FB_CFG);

	return 0;

free_rx_ring:
	dma_unmap_page(kdev, intf->rx_ring_dma, RING_BUFFER_SIZE, DMA_FROM_DEVICE);
	__free_pages(virt_to_page(intf->rx_ring_cpu), intf->rx_buf_order);

	return ret;
}

static void bcmasp_reclaim_free_all_rx(struct bcmasp_intf *intf)
{
	struct device *kdev = &intf->parent->pdev->dev;

	dma_free_coherent(kdev, DESC_RING_SIZE, intf->rx_edpkt_cpu,
			  intf->rx_edpkt_dma);
	dma_unmap_page(kdev, intf->rx_ring_dma, RING_BUFFER_SIZE, DMA_FROM_DEVICE);
	__free_pages(virt_to_page(intf->rx_ring_cpu), intf->rx_buf_order);
}

static int bcmasp_init_tx(struct bcmasp_intf *intf)
{
	struct device *kdev = &intf->parent->pdev->dev;
	struct net_device *ndev = intf->ndev;
	void *p;
	int ret;

	p = dma_zalloc_coherent(kdev, DESC_RING_SIZE, &intf->tx_spb_dma,
				GFP_KERNEL);
	if (!p) {
		netdev_err(ndev, "Cannot allocate tx desc ring\n");
		return -ENOMEM;
	}
	intf->tx_spb_cpu = p;
	intf->tx_spb_dma_valid = intf->tx_spb_dma + DESC_RING_SIZE - 1;
	intf->tx_spb_dma_read = intf->tx_spb_dma;

	intf->tx_cbs = kcalloc(DESC_RING_COUNT, sizeof(struct bcmasp_tx_cb),
			       GFP_KERNEL);
	if (!intf->tx_cbs) {
		netdev_err(ndev, "Cannot allocate tx CB\n");
		ret = -ENOMEM;
		goto free_tx_spb;
	}

	spin_lock_init(&intf->tx_lock);
	intf->tx_spb_index = 0;
	intf->tx_spb_clean_index = 0;

	netif_tx_napi_add(intf->ndev, &intf->tx_napi, bcmasp_tx_poll,
			  NAPI_POLL_WEIGHT);

	/* Make sure channels are disabled */
	tx_spb_ctrl_wl(intf, 0x0, TX_SPB_CTRL_ENABLE);
	tx_epkt_core_wl(intf, 0x0, TX_EPKT_C_CFG_MISC);

	idma_trans_wl(intf, 0x100000, ACPUSS_CTRL_TRANS_INFO);

	/* Tx SPB */
	tx_spb_ctrl_wl(intf, ((intf->channel + 8) << TX_SPB_CTRL_XF_BID_SHIFT),
		       TX_SPB_CTRL_XF_CTRL2);
	tx_pause_ctrl_wl(intf, (1 << (intf->channel + 8)), TX_PAUSE_MAP_VECTOR);
	tx_spb_top_wl(intf, 0x1e, TX_SPB_TOP_BLKOUT);
	tx_spb_top_wl(intf, 0x0, TX_SPB_TOP_SPRE_BW_CTRL);

	tx_spb_dma_wq(intf, intf->tx_spb_dma, TX_SPB_DMA_READ);
	tx_spb_dma_wq(intf, intf->tx_spb_dma, TX_SPB_DMA_BASE);
	tx_spb_dma_wq(intf, intf->tx_spb_dma_valid, TX_SPB_DMA_END);
	tx_spb_dma_wq(intf, intf->tx_spb_dma_valid, TX_SPB_DMA_VALID);

	return 0;

free_tx_spb:
	dma_free_coherent(kdev, DESC_RING_SIZE, intf->tx_spb_cpu,
			  intf->tx_spb_dma);

	return ret;
}

static void bcmasp_reclaim_free_all_tx(struct bcmasp_intf *intf)
{
	struct device *kdev = &intf->parent->pdev->dev;

	/* Free descriptors */
	dma_free_coherent(kdev, DESC_RING_SIZE, intf->tx_spb_cpu,
			  intf->tx_spb_dma);

	/* Free cbs */
	kfree(intf->tx_cbs);
}

static int bcmasp_netif_deinit(struct net_device *dev, bool stop_phy)
{
	struct bcmasp_intf *intf = netdev_priv(dev);

	u32 reg, timeout = 1000;
	napi_disable(&intf->tx_napi);

	bcmasp_enable_tx(intf, 0);

	/* Flush any TX packets in the pipe */
	tx_spb_dma_wl(intf, 0x1, TX_SPB_DMA_FIFO_CTRL);
	do {
		reg = tx_spb_dma_rl(intf, TX_SPB_DMA_FIFO_STATUS);
		if (!(reg & 0x1))
			break;
		usleep_range(1000, 2000);
	} while (timeout-- > 0);
	tx_spb_dma_wl(intf, 0x0, TX_SPB_DMA_FIFO_CTRL);

	umac_enable_set(intf, UMC_CMD_TX_EN, 0);

	if (stop_phy)
		phy_stop(dev->phydev);

	/* Shut down RX */
	umac_enable_set(intf, UMC_CMD_RX_EN, 0);

	/* Flush packets in pipe */
	bcmasp_flush_rx_port(intf);
	usleep_range(1000, 2000);
	bcmasp_enable_rx(intf, 0);

	napi_disable(&intf->rx_napi);

	/* Disable interrupts */
	bcmasp_enable_tx_irq(intf, 0);
	bcmasp_enable_rx_irq(intf, 0);

	netif_napi_del(&intf->tx_napi);
	netif_napi_del(&intf->rx_napi);

	bcmasp_reclaim_free_all_rx(intf);
	bcmasp_reclaim_free_all_tx(intf);

	return 0;
}

static int bcmasp_stop(struct net_device *dev)
{
	struct bcmasp_intf *intf = netdev_priv(dev);
	int ret;

	netif_dbg(intf, ifdown, dev, "bcmasp_close\n");

	/* Stop tx from updating HW */
	netif_tx_disable(dev);

	ret = bcmasp_netif_deinit(dev, true);
	if (ret)
		return ret;

	phy_disconnect(dev->phydev);

	clk_disable_unprepare(intf->parent->clk);

	return 0;
}

static void bcmasp_configure_port(struct bcmasp_intf *intf)
{
	u32 reg, id_mode_dis = 0;

	reg = rgmii_rl(intf, RGMII_PORT_CNTRL);
	reg &= ~RGMII_PORT_MODE_MASK;

	switch (intf->phy_interface) {
	case PHY_INTERFACE_MODE_RGMII:
		/* RGMII_NO_ID: TXC transitions at the same time as TXD
		 *		(requires PCB or receiver-side delay)
		 * RGMII:	Add 2ns delay on TXC (90 degree shift)
		 *
		 * ID is implicitly disabled for 100Mbps (RG)MII operation.
		 */
		id_mode_dis = RGMII_ID_MODE_DIS;
		/* fall through */
	case PHY_INTERFACE_MODE_RGMII_TXID:
		reg |= RGMII_PORT_MODE_EXT_GPHY;
		break;
	case PHY_INTERFACE_MODE_MII:
		reg |= RGMII_PORT_MODE_EXT_EPHY;
		break;
	default:
		break;
	}

	rgmii_wl(intf, reg, RGMII_PORT_CNTRL);

	reg = rgmii_rl(intf, RGMII_OOB_CNTRL);
	reg &= ~RGMII_ID_MODE_DIS;
	reg |= id_mode_dis;
	rgmii_wl(intf, reg, RGMII_OOB_CNTRL);
}

static int bcmasp_netif_init(struct net_device *dev, bool phy_connect)
{
	struct bcmasp_intf *intf = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	int ret;

	bcmasp_configure_port(intf);

	umac_reset(intf);

	umac_init(intf);

	/* Disable the UniMAC RX/TX */
	umac_enable_set(intf, (UMC_CMD_RX_EN | UMC_CMD_TX_EN), 0);

	umac_set_hw_addr(intf, dev->dev_addr);

	if (phy_connect) {
		phydev = of_phy_connect(dev, intf->phy_dn, bcmasp_adj_link, 0,
					intf->phy_interface);
		if (!phydev) {
			netdev_err(dev, "could not attach to PHY\n");
			return -ENODEV;
		}
	}
	intf->old_duplex = -1;
	intf->old_link = -1;
	intf->old_pause = -1;

	ret = bcmasp_init_tx(intf);
	if (ret)
		goto err_phy_disconnect;

	bcmasp_init_rx(intf);
	if (ret)
		goto err_reclaim_tx;

	/* Turn on asp */
	bcmasp_enable_tx(intf, 1);
	bcmasp_enable_rx(intf, 1);

	/* Turn on UniMAC TX/RX */
	umac_enable_set(intf, (UMC_CMD_RX_EN | UMC_CMD_TX_EN), 1);

	bcmasp_netif_start(dev);

	netif_start_queue(dev);

	return 0;

err_reclaim_tx:
	bcmasp_reclaim_free_all_tx(intf);
err_phy_disconnect:
	if (phydev)
		phy_disconnect(phydev);
	return ret;
}

static int bcmasp_open(struct net_device *dev)
{
	struct bcmasp_intf *intf = netdev_priv(dev);
	int ret;

	netif_dbg(intf, ifup, dev, "bcmasp_open\n");

	ret = clk_prepare_enable(intf->parent->clk);
	if (ret)
		return ret;

	ret = bcmasp_netif_init(dev, true);
	if (ret)
		clk_disable_unprepare(intf->parent->clk);

	return ret;
}

static void bcmasp_reset_mib(struct bcmasp_intf *intf)
{
	umac_wl(intf, UMC_MIB_CNTRL_RX_CNT_RST |
		UMC_MIB_CNTRL_RUNT_CNT_RST |
		UMC_MIB_CNTRL_TX_CNT_RST, UMC_MIB_CNTRL);
	usleep_range(1000, 2000);
	umac_wl(intf, 0, UMC_MIB_CNTRL);
}

static void bcmasp_tx_timeout(struct net_device *dev)
{
	struct bcmasp_intf *intf = netdev_priv(dev);

	netif_dbg(intf, tx_err, dev, "transmit timeout!\n");

	netif_trans_update(dev);
	dev->stats.tx_errors++;

	netif_wake_queue(dev);
}

static int bcmasp_get_phys_port_name(struct net_device *dev,
				      char *name, size_t len)
{
	struct bcmasp_intf *intf = netdev_priv(dev);

	if (snprintf(name, len, "p%d", intf->port) >= len)
		return -EINVAL;

	return 0;
}

static struct net_device_stats *bcmasp_get_stats(struct net_device *dev)
{
	return &dev->stats;
}

static const struct net_device_ops bcmasp_netdev_ops = {
	.ndo_open		= bcmasp_open,
	.ndo_stop		= bcmasp_stop,
	.ndo_start_xmit		= bcmasp_xmit,
	.ndo_tx_timeout		= bcmasp_tx_timeout,
	.ndo_set_rx_mode	= bcmasp_set_rx_mode,
	.ndo_get_phys_port_name	= bcmasp_get_phys_port_name,
	.ndo_get_stats		= bcmasp_get_stats,
};

static inline void  bcmasp_map_res(struct bcmasp_priv *priv,
				    struct bcmasp_intf *intf)
{
	/* Per port */
	intf->res.umac = priv->base + UMC_OFFSET(intf);
	intf->res.umac2fb = priv->base + UMAC2FB_OFFSET(intf);
	intf->res.rgmii = priv->base + RGMII_OFFSET(intf);
	intf->res.idma_trans = priv->base + ACPUSS_CTRL_TRANS_OFFSET(intf);

	/* Per ch */
	intf->res.tx_spb_dma = priv->base + TX_SPB_DMA_OFFSET(intf);
	intf->res.tx_spb_ctrl = priv->base + TX_SPB_CTRL_OFFSET(intf);
	intf->res.tx_spb_top = priv->base + TX_SPB_TOP_OFFSET(intf);
	intf->res.tx_epkt_core = priv->base + TX_EPKT_C_OFFSET(intf);
	intf->res.tx_pause_ctrl = priv->base + TX_PAUSE_CTRL_OFFSET(intf);

	intf->res.rx_edpkt_dma = priv->base + RX_EDPKT_DMA_OFFSET(intf);
	intf->res.rx_edpkt_cfg = priv->base + RX_EDPKT_CFG_OFFSET(intf);
}

static irqreturn_t bcmasp_wol_isr(int irq, void *dev_id)
{
	struct bcmasp_priv *priv = dev_id;

	pm_wakeup_event(&priv->pdev->dev, 0);

	return IRQ_HANDLED;
}

struct bcmasp_intf *bcmasp_interface_create(struct bcmasp_priv *priv,
					    struct device_node *ndev_dn,
					    int wol_irq)
{
	struct device *dev = &priv->pdev->dev;
	struct bcmasp_intf *intf;
	struct net_device *ndev;
	const void *macaddr;
	int ch, port, ret;

	if (of_property_read_u32(ndev_dn, "reg", &port)) {
		dev_warn(dev, "%s: invalid ch number\n", ndev_dn->name);
		goto err;
	}

	if (of_property_read_u32(ndev_dn, "channel", &ch)) {
		dev_warn(dev, "%s: invalid ch number\n", ndev_dn->name);
		goto err;
	}

	ndev = alloc_etherdev(sizeof(struct bcmasp_intf));
	if (!dev) {
		dev_warn(dev, "%s: unable to alloc ndev\n", ndev_dn->name);
		goto err;
	}
	intf = netdev_priv(ndev);

	/* Interrupt is optional */
	if (wol_irq > 0) {
		intf->wol_irq_disabled = true;
		ret = request_irq(wol_irq, bcmasp_wol_isr, 0, ndev->name, priv);
		if (ret) {
			netdev_err(ndev, "unable to request WoL IRQ (%d)\n",
				   ret);
			goto err_free_netdev;
		}
		device_set_wakeup_capable(dev, 1);
	}

	intf->wol_irq = wol_irq;
	intf->parent = priv;
	intf->ndev = ndev;
	intf->channel = ch;
	intf->port = port;
	intf->ndev_dn = ndev_dn;

	intf->phy_interface = of_get_phy_mode(ndev_dn);
	if (intf->phy_interface < 0)
		intf->phy_interface = PHY_INTERFACE_MODE_GMII;

        intf->phy_dn = of_parse_phandle(ndev_dn, "phy-handle", 0);

	if (!intf->phy_dn && of_phy_is_fixed_link(ndev_dn)) {
		ret = of_phy_register_fixed_link(ndev_dn);
		if (ret) {
			dev_warn(dev, "%s: failed to register fixed PHY\n",
				ndev_dn->name);
			goto err_free_irq;
		}
		intf->phy_dn = ndev_dn;
	}

	/* Map resource */
	bcmasp_map_res(priv, intf);

	if (!phy_interface_mode_is_rgmii(intf->phy_interface) &&
	    intf->phy_interface != PHY_INTERFACE_MODE_MII) {
		netdev_err(intf->ndev, "invalid PHY mode: %s\n",
			   phy_modes(intf->phy_interface));
		ret = -EINVAL;
		goto err_free_irq;
	}

	bcmasp_reset_mib(intf);

	macaddr = of_get_mac_address(ndev_dn);
	if (!macaddr || !is_valid_ether_addr(macaddr)) {
		dev_warn(dev, "%s: using random Ethernet MAC\n",
			 ndev_dn->name);
		eth_hw_addr_random(ndev);
	} else {
		ether_addr_copy(ndev->dev_addr, macaddr);
	}

	SET_NETDEV_DEV(ndev, dev);
	ndev->netdev_ops = &bcmasp_netdev_ops;
	ndev->ethtool_ops = &bcmasp_ethtool_ops;
	intf->msg_enable = netif_msg_init(-1, NETIF_MSG_DRV |
					  NETIF_MSG_PROBE |
					  NETIF_MSG_LINK);

	return intf;

err_free_irq:
	if (wol_irq > 0)
		free_irq(wol_irq, priv);
err_free_netdev:
	free_netdev(ndev);
err:
	return NULL;
}

void bcmasp_interface_destroy(struct bcmasp_intf *intf, bool unregister)
{
	if (intf->wol_irq > 0)
		free_irq(intf->wol_irq, intf->parent);
	if (unregister)
		unregister_netdev(intf->ndev);
	if (of_phy_is_fixed_link(intf->ndev_dn))
		of_phy_deregister_fixed_link(intf->ndev_dn);
	free_netdev(intf->ndev);
}

static int bcmasp_suspend_to_wol(struct bcmasp_intf *intf)
{
	struct net_device *ndev = intf->ndev;
	u32 reg;

	reg = umac_rl(intf, UMC_MPD_CTRL);
	if (intf->wolopts & (WAKE_MAGIC | WAKE_MAGICSECURE))
		reg |= UMC_MPD_CTRL_MPD_EN;
	reg &= ~UMC_MPD_CTRL_PSW_EN;
	if (intf->wolopts & WAKE_MAGICSECURE) {
		/* Program the SecureOn password */
		umac_wl(intf, get_unaligned_be16(&intf->sopass[0]),
			UMC_PSW_MS);
		umac_wl(intf, get_unaligned_be32(&intf->sopass[2]),
			UMC_PSW_LS);
		reg |= UMC_MPD_CTRL_PSW_EN;
	}
	umac_wl(intf, reg, UMC_MPD_CTRL);

	/* UniMAC receive needs to be turned on */
	umac_enable_set(intf, UMC_CMD_RX_EN, 1);

	netif_dbg(intf, wol, ndev, "entered WOL mode\n");

	return 0;
}

int bcmasp_interface_suspend(struct bcmasp_intf *intf)
{
	struct device *kdev = &intf->parent->pdev->dev;
	struct net_device *dev = intf->ndev;
	int ret;

	if (!netif_running(dev))
		return 0;

	netif_device_detach(dev);

	ret = bcmasp_netif_deinit(dev, false);
	if (ret)
		return ret;

	if (!device_may_wakeup(kdev)) {
		ret = phy_suspend(dev->phydev);
		if (ret)
			goto out;
	}

	if (device_may_wakeup(kdev) && intf->wolopts)
		ret = bcmasp_suspend_to_wol(intf);

	clk_disable_unprepare(intf->parent->clk);

	return ret;

out:
	bcmasp_netif_init(dev, false);
	return ret;
}

static void bcmasp_resume_from_wol(struct bcmasp_intf *intf)
{
	u32 reg;

	reg = umac_rl(intf, UMC_MPD_CTRL);
	reg &= ~UMC_MPD_CTRL_MPD_EN;
	umac_wl(intf, reg, UMC_MPD_CTRL);
}

int bcmasp_interface_resume(struct bcmasp_intf *intf)
{
	struct device *kdev = &intf->parent->pdev->dev;
	struct net_device *dev = intf->ndev;
	int ret;

	if (!netif_running(dev))
		return 0;

	ret = clk_prepare_enable(intf->parent->clk);
	if (ret)
		return ret;

	bcmasp_resume_from_wol(intf);

	ret = bcmasp_netif_init(dev, false);
	if (ret)
		goto out;

	if (!device_may_wakeup(kdev)) {
		ret = phy_resume(dev->phydev);
		if (ret)
			goto out_phy_resume;
	}

	netif_device_attach(dev);

	return 0;

out_phy_resume:
	bcmasp_netif_deinit(dev, false);
out:
	clk_disable_unprepare(intf->parent->clk);
	return ret;
}
