// SPDX-License-Identifier: GPL-2.0
/*
 * Lantiq / Intel PMAC driver for XRX200 SoCs
 *
 * Copyright (C) 2010 Lantiq Deutschland
 * Copyright (C) 2012 John Crispin <john@phrozen.org>
 * Copyright (C) 2017 - 2018 Hauke Mehrtens <hauke@hauke-m.de>
 */

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <linux/of_net.h>
#include <linux/of_platform.h>

#include <xway_dma.h>

/* DMA */
#define XRX200_DMA_DATA_LEN	0x600
#define XRX200_DMA_RXn(n)	(2 * (n))
#define XRX200_DMA_RX_MAX_QUEUES	4
#define XRX200_DMA_TXn(n)	(2 * (n) + 1)
#define XRX200_DMA_TX_MAX_QUEUES	4

/* cpu port mac */
#define PMAC_RX_IPG		0x0024
#define PMAC_RX_IPG_MASK	0xf

#define PMAC_HD_CTL		0x0000
/* Add Ethernet header to packets from DMA to PMAC */
#define PMAC_HD_CTL_ADD		BIT(0)
/* Add VLAN tag to Packets from DMA to PMAC */
#define PMAC_HD_CTL_TAG		BIT(1)
/* Add CRC to packets from DMA to PMAC */
#define PMAC_HD_CTL_AC		BIT(2)
/* Add status header to packets from PMAC to DMA */
#define PMAC_HD_CTL_AS		BIT(3)
/* Remove CRC from packets from PMAC to DMA */
#define PMAC_HD_CTL_RC		BIT(4)
/* Remove Layer-2 header from packets from PMAC to DMA */
#define PMAC_HD_CTL_RL2		BIT(5)
/* Status header is present from DMA to PMAC */
#define PMAC_HD_CTL_RXSH	BIT(6)
/* Add special tag from PMAC to switch */
#define PMAC_HD_CTL_AST		BIT(7)
/* Remove specail Tag from PMAC to DMA */
#define PMAC_HD_CTL_RST		BIT(8)
/* Check CRC from DMA to PMAC */
#define PMAC_HD_CTL_CCRC	BIT(9)
/* Enable reaction to Pause frames in the PMAC */
#define PMAC_HD_CTL_FC		BIT(10)

struct xrx200_chan {
	int tx_free;

	struct napi_struct napi;
	struct ltq_dma_channel dma;
	struct sk_buff *skb[LTQ_DESC_NUM];

	struct xrx200_priv *priv;
};

struct xrx200_priv {
	struct clk *clk;

	struct xrx200_chan chan_tx[XRX200_DMA_TX_MAX_QUEUES];
	struct xrx200_chan chan_rx[XRX200_DMA_RX_MAX_QUEUES];

	struct net_device *net_dev;
	struct device *dev;

	__iomem void *pmac_reg;
};

static u32 xrx200_pmac_r32(struct xrx200_priv *priv, u32 offset)
{
	return __raw_readl(priv->pmac_reg + offset);
}

static void xrx200_pmac_w32(struct xrx200_priv *priv, u32 val, u32 offset)
{
	__raw_writel(val, priv->pmac_reg + offset);
}

static void xrx200_pmac_mask(struct xrx200_priv *priv, u32 clear, u32 set,
			     u32 offset)
{
	u32 val = xrx200_pmac_r32(priv, offset);

	val &= ~(clear);
	val |= set;
	xrx200_pmac_w32(priv, val, offset);
}

/* drop all the packets from the DMA ring */
static void xrx200_flush_dma(struct xrx200_chan *ch)
{
	int i;

	for (i = 0; i < LTQ_DESC_NUM; i++) {
		struct ltq_dma_desc *desc = &ch->dma.desc_base[ch->dma.desc];

		if ((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) != LTQ_DMA_C)
			break;

		desc->ctl = LTQ_DMA_OWN | LTQ_DMA_RX_OFFSET(NET_IP_ALIGN) |
			    XRX200_DMA_DATA_LEN;
		ch->dma.desc++;
		ch->dma.desc %= LTQ_DESC_NUM;
	}
}

static void xrx200_open_dma_chan(struct xrx200_chan *chan)
{
	napi_enable(&chan->napi);
	ltq_dma_open(&chan->dma);
}

static int xrx200_open(struct net_device *net_dev)
{
	struct xrx200_priv *priv = netdev_priv(net_dev);
	int i;

	for (i = 0; i < net_dev->real_num_tx_queues; i++) {
		xrx200_open_dma_chan(&priv->chan_tx[i]);
		ltq_dma_enable_irq(&priv->chan_tx[i].dma);
	}

	xrx200_open_dma_chan(&priv->chan_rx[0]);
	/* The boot loader does not always deactivate the receiving of frames
	 * on the ports and then some packets queue up in the PPE buffers.
	 * They already passed the PMAC so they do not have the tags
	 * configured here. Read the these packets here and drop them.
	 * The HW should have written them into memory after 10us
	 */
	usleep_range(20, 40);
	xrx200_flush_dma(&priv->chan_rx[0]);
	ltq_dma_enable_irq(&priv->chan_rx[0].dma);

	netif_tx_start_all_queues(net_dev);

	return 0;
}

static void xrx200_dma_close(struct xrx200_chan *chan)
{
	napi_disable(&chan->napi);
	ltq_dma_close(&chan->dma);
}

static int xrx200_close(struct net_device *net_dev)
{
	struct xrx200_priv *priv = netdev_priv(net_dev);
	int i;

	netif_tx_stop_all_queues(net_dev);

	xrx200_dma_close(&priv->chan_rx[0]);

	for (i = 0; i < net_dev->real_num_tx_queues; i++)
		xrx200_dma_close(&priv->chan_tx[i]);

	return 0;
}

static int xrx200_alloc_skb(struct xrx200_chan *ch)
{
	struct sk_buff *skb = ch->skb[ch->dma.desc];
	dma_addr_t mapping;
	int ret = 0;

	ch->skb[ch->dma.desc] = netdev_alloc_skb_ip_align(ch->priv->net_dev,
							  XRX200_DMA_DATA_LEN);
	if (!ch->skb[ch->dma.desc]) {
		ret = -ENOMEM;
		goto skip;
	}

	mapping = dma_map_single(ch->priv->dev, ch->skb[ch->dma.desc]->data,
				 XRX200_DMA_DATA_LEN, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(ch->priv->dev, mapping))) {
		dev_kfree_skb_any(ch->skb[ch->dma.desc]);
		ch->skb[ch->dma.desc] = skb;
		ret = -ENOMEM;
		goto skip;
	}

	ch->dma.desc_base[ch->dma.desc].addr = mapping;
	/* Make sure the address is written before we give it to HW */
	wmb();
skip:
	ch->dma.desc_base[ch->dma.desc].ctl =
		LTQ_DMA_OWN | LTQ_DMA_RX_OFFSET(NET_IP_ALIGN) |
		XRX200_DMA_DATA_LEN;

	return ret;
}

static int xrx200_hw_receive(struct xrx200_chan *ch)
{
	struct xrx200_priv *priv = ch->priv;
	struct ltq_dma_desc *desc = &ch->dma.desc_base[ch->dma.desc];
	struct sk_buff *skb = ch->skb[ch->dma.desc];
	int len = (desc->ctl & LTQ_DMA_SIZE_MASK);
	struct net_device *net_dev = priv->net_dev;
	int ret;

	ret = xrx200_alloc_skb(ch);

	ch->dma.desc++;
	ch->dma.desc %= LTQ_DESC_NUM;

	if (ret) {
		net_dev->stats.rx_dropped++;
		netdev_err(net_dev, "failed to allocate new rx buffer\n");
		return ret;
	}

	skb_put(skb, len);
	skb->protocol = eth_type_trans(skb, net_dev);
	netif_receive_skb(skb);
	net_dev->stats.rx_packets++;
	net_dev->stats.rx_bytes += len - ETH_FCS_LEN;

	return 0;
}

static int xrx200_poll_rx(struct napi_struct *napi, int budget)
{
	struct xrx200_chan *ch = container_of(napi,
				struct xrx200_chan, napi);
	int rx = 0;
	int ret;

	while (rx < budget) {
		struct ltq_dma_desc *desc = &ch->dma.desc_base[ch->dma.desc];

		if ((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) == LTQ_DMA_C) {
			ret = xrx200_hw_receive(ch);
			if (ret)
				return ret;
			rx++;
		} else {
			break;
		}
	}

	if (rx < budget) {
		if (napi_complete_done(&ch->napi, rx))
			ltq_dma_enable_irq(&ch->dma);
	}

	return rx;
}

static int xrx200_tx_housekeeping(struct napi_struct *napi, int budget)
{
	struct xrx200_chan *ch = container_of(napi,
				struct xrx200_chan, napi);
	struct net_device *net_dev = ch->priv->net_dev;
	struct netdev_queue *txq =
		netdev_get_tx_queue(net_dev, ch->dma.nr >> 1);
	int pkts = 0;
	int bytes = 0;

	__netif_tx_lock(txq, smp_processor_id());
	while (pkts < budget) {
		struct ltq_dma_desc *desc = &ch->dma.desc_base[ch->tx_free];

		if ((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) == LTQ_DMA_C) {
			struct sk_buff *skb = ch->skb[ch->tx_free];

			pkts++;
			bytes += skb->len;
			ch->skb[ch->tx_free] = NULL;
			consume_skb(skb);
			memset(&ch->dma.desc_base[ch->tx_free], 0,
			       sizeof(struct ltq_dma_desc));
			ch->tx_free++;
			ch->tx_free %= LTQ_DESC_NUM;
		} else {
			break;
		}
	}

	net_dev->stats.tx_packets += pkts;
	net_dev->stats.tx_bytes += bytes;
	netdev_tx_completed_queue(txq, pkts, bytes);

	__netif_tx_unlock(txq);
	if (netif_tx_queue_stopped(txq))
		netif_tx_wake_queue(txq);

	if (pkts < budget) {
		if (napi_complete_done(&ch->napi, pkts))
			ltq_dma_enable_irq(&ch->dma);
	}

	return pkts;
}

static netdev_tx_t xrx200_start_xmit(struct sk_buff *skb,
				     struct net_device *net_dev)
{
	struct xrx200_priv *priv = netdev_priv(net_dev);
	u32 queue = skb_get_queue_mapping(skb);
	struct xrx200_chan *ch = &priv->chan_tx[queue];
	struct ltq_dma_desc *desc = &ch->dma.desc_base[ch->dma.desc];
	struct netdev_queue *txq = netdev_get_tx_queue(net_dev, queue);
	u32 byte_offset;
	dma_addr_t mapping;
	int len;

	skb->dev = net_dev;
	if (skb_put_padto(skb, ETH_ZLEN)) {
		net_dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	len = skb->len;

	if ((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) || ch->skb[ch->dma.desc]) {
		netdev_err(net_dev, "tx ring full\n");
		netif_tx_stop_queue(txq);
		return NETDEV_TX_BUSY;
	}

	ch->skb[ch->dma.desc] = skb;

	mapping = dma_map_single(priv->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(priv->dev, mapping)))
		goto err_drop;

	/* dma needs to start on a 16 byte aligned address */
	byte_offset = mapping % 16;

	desc->addr = mapping - byte_offset;
	/* Make sure the address is written before we give it to HW */
	wmb();
	desc->ctl = LTQ_DMA_OWN | LTQ_DMA_SOP | LTQ_DMA_EOP |
		LTQ_DMA_TX_OFFSET(byte_offset) | (len & LTQ_DMA_SIZE_MASK);
	ch->dma.desc++;
	ch->dma.desc %= LTQ_DESC_NUM;
	if (ch->dma.desc == ch->tx_free)
		netif_tx_stop_queue(txq);

	netdev_tx_sent_queue(txq, len);

	return NETDEV_TX_OK;

err_drop:
	dev_kfree_skb(skb);
	net_dev->stats.tx_dropped++;
	net_dev->stats.tx_errors++;
	return NETDEV_TX_OK;
}

static const struct net_device_ops xrx200_netdev_ops = {
	.ndo_open		= xrx200_open,
	.ndo_stop		= xrx200_close,
	.ndo_start_xmit		= xrx200_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_select_queue	= dev_pick_tx_cpu_id,
};

static irqreturn_t xrx200_dma_irq(int irq, void *ptr)
{
	struct xrx200_chan *ch = ptr;

	if (napi_schedule_prep(&ch->napi)) {
		ltq_dma_disable_irq(&ch->dma);
		__napi_schedule(&ch->napi);
	}

	ltq_dma_ack_irq(&ch->dma);

	return IRQ_HANDLED;
}

static int xrx200_dma_rx_chan_init(struct xrx200_priv *priv, unsigned int num)
{
	struct xrx200_chan *ch_rx = priv->chan_rx;
	int ret = 0;
	int i;

	ch_rx[num].dma.nr = XRX200_DMA_RXn(num);
	ch_rx[num].dma.dev = priv->dev;
	ch_rx[num].priv = priv;

	ltq_dma_alloc_rx(&ch_rx[num].dma);
	for (ch_rx[num].dma.desc = 0; ch_rx[num].dma.desc < LTQ_DESC_NUM;
	     ch_rx[num].dma.desc++) {
		ret = xrx200_alloc_skb(&ch_rx[num]);
		if (ret)
			goto rx_free;
	}
	ch_rx[num].dma.desc = 0;
	ret = devm_request_irq(priv->dev, ch_rx[num].dma.irq, xrx200_dma_irq, 0,
			       priv->net_dev->name, &ch_rx[num]);
	if (ret) {
		dev_err(priv->dev, "failed to request RX irq %d\n",
			ch_rx[num].dma.irq);
		goto rx_ring_free;
	}

	return ret;

rx_ring_free:
	/* free the allocated RX ring */
	for (i = 0; i < LTQ_DESC_NUM; i++) {
		if (ch_rx[num].skb[i])
			dev_kfree_skb_any(ch_rx[num].skb[i]);
	}

rx_free:
	ltq_dma_free(&ch_rx[num].dma);

	return ret;
}

static int xrx200_dma_tx_chan_init(struct xrx200_priv *priv, unsigned int num)
{
	struct xrx200_chan *ch_tx = priv->chan_tx;
	int ret = 0;

	ch_tx[num].dma.nr = XRX200_DMA_TXn(num);
	ch_tx[num].dma.dev = priv->dev;
	ch_tx[num].priv = priv;

	ltq_dma_alloc_tx(&ch_tx[num].dma);
	ret = devm_request_irq(priv->dev, ch_tx[num].dma.irq, xrx200_dma_irq, 0,
			       priv->net_dev->name, &ch_tx[num]);
	if (ret) {
		dev_err(priv->dev, "failed to request TX irq %d\n",
			ch_tx[num].dma.irq);
		return ret;
	}

	return ret;
}

static int xrx200_dma_init(struct xrx200_priv *priv)
{
	int ret = 0;

	ltq_dma_init_port(DMA_PORT_ETOP);

	ret = xrx200_dma_rx_chan_init(priv, 0);
	if (ret) {
		return ret;
	}

	ret = xrx200_dma_tx_chan_init(priv, 0);
	if (ret) {
		goto tx_free;
	}

	return ret;

tx_free:
	ltq_dma_free(&priv->chan_tx[0].dma);

	return ret;
}

static void xrx200_hw_cleanup(struct xrx200_priv *priv)
{
	struct net_device *net_dev = priv->net_dev;
	int i;

	for (i = 0; i < net_dev->real_num_tx_queues; i++)
		ltq_dma_free(&priv->chan_tx[i].dma);

	for (i = 0; i < net_dev->real_num_rx_queues; i++)
		ltq_dma_free(&priv->chan_rx[i].dma);

	/* free the allocated RX ring */
	for (i = 0; i < LTQ_DESC_NUM; i++)
		dev_kfree_skb_any(priv->chan_rx[0].skb[i]);
}

static void xrx200_get_channels(struct net_device *net_dev,
				struct ethtool_channels *channel)
{
	channel->max_rx = XRX200_DMA_RX_MAX_QUEUES;
	channel->max_tx = XRX200_DMA_TX_MAX_QUEUES;
	channel->max_other = 0;
	channel->max_combined = 0;
	channel->rx_count = net_dev->real_num_rx_queues;
	channel->tx_count = net_dev->real_num_tx_queues;
	channel->other_count = 0;
	channel->combined_count = 0;
}

static int xrx200_set_channels(struct net_device *net_dev,
			       struct ethtool_channels *channel)
{
	struct xrx200_priv *priv = netdev_priv(net_dev);
	struct netdev_queue *queue;
	int i;

	if (channel->tx_count > net_dev->real_num_tx_queues) {
		printk(KERN_INFO "%s: enable some queue\n", __func__);
		for (i = net_dev->real_num_tx_queues; i < channel->tx_count; i++) {
			printk(KERN_INFO "%s: start queue #%d\n", __func__, i);
			xrx200_dma_tx_chan_init(priv, i);
			printk(KERN_INFO "%s: start3\n", __func__);
			netif_tx_napi_add(net_dev, &priv->chan_tx[i].napi,
					  xrx200_tx_housekeeping, 32);
			printk(KERN_INFO "%s: start4\n", __func__);
			xrx200_open_dma_chan(&priv->chan_tx[i]);
			printk(KERN_INFO "%s: start5\n", __func__);
			queue = netdev_get_tx_queue(net_dev, i);
			printk(KERN_INFO "%s: start6\n", __func__);
			netif_tx_start_queue(queue);
		}
	} else {
		printk(KERN_INFO "%s: disable some queue\n", __func__);
		for (i = channel->tx_count; i < net_dev->real_num_tx_queues; i++) {
			printk(KERN_INFO "%s: stop queue #%d\n", __func__, i);
			queue = netdev_get_tx_queue(net_dev, i);
			netif_tx_stop_queue(queue);
			xrx200_dma_close(&priv->chan_tx[i]);
			netif_napi_del(&priv->chan_tx[i].napi);
			ltq_dma_free(&priv->chan_tx[i].dma);
		}
	}

	netif_set_real_num_tx_queues(net_dev, channel->tx_count);

	if (channel->rx_count > net_dev->real_num_rx_queues) {
		printk(KERN_INFO "%s: rx: enable some queue\n", __func__);
		for (i = net_dev->real_num_rx_queues; i < channel->rx_count; i++) {
			printk(KERN_INFO "%s: rx: start queue #%d\n", __func__, i);
			xrx200_dma_rx_chan_init(priv, i);
			netif_napi_add(net_dev, &priv->chan_rx[i].napi, xrx200_poll_rx, 32);
			xrx200_open_dma_chan(&priv->chan_rx[i]);
			ltq_dma_enable_irq(&priv->chan_rx[i].dma);
		}
	} else {
		printk(KERN_INFO "%s: rx: disable some queue\n", __func__);
		for (i = channel->rx_count; i < net_dev->real_num_rx_queues; i++) {
			printk(KERN_INFO "%s: rx: stop queue #%d\n", __func__, i);
		}
	}

	netif_set_real_num_rx_queues(net_dev, channel->rx_count);

	return 0;
}

static const struct ethtool_ops xrx200_ethtool_ops = {
	.get_channels		= xrx200_get_channels,
	.set_channels		= xrx200_set_channels,
};

static int xrx200_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct xrx200_priv *priv;
	struct net_device *net_dev;
	int err;
	int i;

	/* alloc the network device */
	net_dev = alloc_etherdev_mqs(sizeof(struct xrx200_priv), XRX200_DMA_TX_MAX_QUEUES,
				     XRX200_DMA_RX_MAX_QUEUES);
	if (!net_dev)
		return -ENOMEM;

	priv = netdev_priv(net_dev);
	priv->net_dev = net_dev;
	priv->dev = dev;

	net_dev->ethtool_ops = &xrx200_ethtool_ops;
	net_dev->netdev_ops = &xrx200_netdev_ops;
	SET_NETDEV_DEV(net_dev, dev);
	net_dev->min_mtu = ETH_ZLEN;
	net_dev->max_mtu = XRX200_DMA_DATA_LEN;
	netif_set_real_num_rx_queues(net_dev, 1);
	netif_set_real_num_tx_queues(net_dev, 1);

	/* load the memory ranges */
	priv->pmac_reg = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(priv->pmac_reg))
		return PTR_ERR(priv->pmac_reg);

	for (i = 0; i < XRX200_DMA_RX_MAX_QUEUES; i++) {
		char buffer[4];

		snprintf(buffer, sizeof(buffer), "rx%d", i);
		priv->chan_rx[i].dma.irq = platform_get_irq_byname(pdev, buffer);
		dev_err(priv->dev, "get RX irq %d\n", priv->chan_rx[i].dma.irq);
		if (priv->chan_rx[i].dma.irq < 0)
			return -ENOENT;
	}

	for (i = 0; i < XRX200_DMA_TX_MAX_QUEUES; i++) {
		char buffer[4];

		snprintf(buffer, sizeof(buffer), "tx%d", i);
		priv->chan_tx[i].dma.irq = platform_get_irq_byname(pdev, buffer);
		if (priv->chan_tx[i].dma.irq < 0)
			return -ENOENT;
	}

	/* get the clock */
	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "failed to get clock\n");
		return PTR_ERR(priv->clk);
	}

	err = of_get_mac_address(np, net_dev->dev_addr);
	if (err)
		eth_hw_addr_random(net_dev);

	/* bring up the dma engine and IP core */
	err = xrx200_dma_init(priv);
	if (err)
		return err;

	/* enable clock gate */
	err = clk_prepare_enable(priv->clk);
	if (err)
		goto err_uninit_dma;

	/* set IPG to 12 */
	xrx200_pmac_mask(priv, PMAC_RX_IPG_MASK, 0xb, PMAC_RX_IPG);

	/* enable status header, enable CRC */
	xrx200_pmac_mask(priv, 0,
			 PMAC_HD_CTL_RST | PMAC_HD_CTL_AST | PMAC_HD_CTL_RXSH |
			 PMAC_HD_CTL_AS | PMAC_HD_CTL_AC | PMAC_HD_CTL_RC,
			 PMAC_HD_CTL);

	/* setup NAPI */
	netif_napi_add(net_dev, &priv->chan_rx[0].napi, xrx200_poll_rx, 32);
	netif_tx_napi_add(net_dev, &priv->chan_tx[0].napi, xrx200_tx_housekeeping, 32);

	platform_set_drvdata(pdev, priv);

	err = register_netdev(net_dev);
	if (err)
		goto err_unprepare_clk;

	return 0;

err_unprepare_clk:
	clk_disable_unprepare(priv->clk);

err_uninit_dma:
	xrx200_hw_cleanup(priv);

	return err;
}

static int xrx200_remove(struct platform_device *pdev)
{
	struct xrx200_priv *priv = platform_get_drvdata(pdev);
	struct net_device *net_dev = priv->net_dev;
	int i;

	/* free stack related instances */
	netif_tx_stop_all_queues(net_dev);

	for (i = 0; i < net_dev->real_num_tx_queues; i++)
		netif_napi_del(&priv->chan_tx[i].napi);

	for (i = 0; i < net_dev->real_num_rx_queues; i++)
		netif_napi_del(&priv->chan_rx[i].napi);

	/* remove the actual device */
	unregister_netdev(net_dev);

	/* release the clock */
	clk_disable_unprepare(priv->clk);

	/* shut down hardware */
	xrx200_hw_cleanup(priv);

	return 0;
}

static const struct of_device_id xrx200_match[] = {
	{ .compatible = "lantiq,xrx200-net" },
	{},
};
MODULE_DEVICE_TABLE(of, xrx200_match);

static struct platform_driver xrx200_driver = {
	.probe = xrx200_probe,
	.remove = xrx200_remove,
	.driver = {
		.name = "lantiq,xrx200-net",
		.of_match_table = xrx200_match,
	},
};

module_platform_driver(xrx200_driver);

MODULE_AUTHOR("John Crispin <john@phrozen.org>");
MODULE_DESCRIPTION("Lantiq SoC XRX200 ethernet");
MODULE_LICENSE("GPL");
