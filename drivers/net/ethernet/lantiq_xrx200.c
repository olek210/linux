/*
 *   This program is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License version 2 as published
 *   by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Copyright (C) 2010 Lantiq Deutschland
 *   Copyright (C) 2012 John Crispin <blogic@openwrt.org>
 */

#include <linux/switch.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/if_vlan.h>
#include <asm/delay.h>

#include <linux/refcount.h>

#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include <xway_dma.h>
#include <lantiq_soc.h>

#include "lantiq_pce.h"
#include "lantiq_xrx200_legacy.h"

#define SW_POLLING	//polls phy
#define SW_ROUTING	//adds vlan field

/* set number of TX queues: 1-2 */
#define MAX_TX_RINGS		2

/* set number of RX queues: 1 */
//2, requires irq in devicetree, still not working
#define MAX_RX_RINGS		1

/* set number of TX queues: 1-2 */
#define MAX_TX_QUEUES		2

#define XRX200_MAX_VLAN		64

#define XRX200_PCE_ACTVLAN_IDX	0x01
#define XRX200_PCE_VLANMAP_IDX	0x02

#define XRX200_MAX_PORT		7
#define XRX200_MAX_DMA		8

#define XRX200_HEADROOM		4

//TODO fine tune
#define XRX200_TX_TIMEOUT	(20 * HZ)

/* port type */
#define XRX200_PORT_TYPE_PHY	1
#define XRX200_PORT_TYPE_MAC	2

/* DMA */
#define XRX200_DMA_DATA_LEN	0x600
#define XRX200_DMA_TX_ALIGN	(32 - 1)

//TODO in devicetree?
#define XRX200_DMA_IRQ		INT_NUM_IM2_IRL0
#define XRX200_DMA_RX		0
#define XRX200_DMA_TX		1
#define XRX200_DMA_RX_2		2
#define XRX200_DMA_TX_2		3
// #define XRX200_DMA_TX_3		5

/* fetch / store dma */
#define FDMA_PCTRL0		0x2A00
#define FDMA_PCTRLx(x)		(FDMA_PCTRL0 + (x * 0x18))
#define SDMA_PCTRL0		0x2F00
#define SDMA_PCTRLx(x)		(SDMA_PCTRL0 + (x * 0x18))

/* buffer management */
#define BM_PCFG0		0x200
#define BM_PCFGx(x)		(BM_PCFG0 + (x * 8))

/* MDIO */
#define MDIO_GLOB		0x0000
#define MDIO_CTRL		0x0020
#define MDIO_READ		0x0024
#define MDIO_WRITE		0x0028
#define MDIO_PHY0		0x0054
#define MDIO_PHY(x)		(0x0054 - (x * sizeof(unsigned)))
#define MDIO_CLK_CFG0		0x002C
#define MDIO_CLK_CFG1		0x0030

#define MDIO_GLOB_ENABLE	0x8000
#define MDIO_BUSY		BIT(12)
#define MDIO_RD			BIT(11)
#define MDIO_WR			BIT(10)
#define MDIO_MASK		0x1f
#define MDIO_ADDRSHIFT		5
#define MDIO1_25MHZ		9

#define MDIO_PHY_LINK_DOWN	0x4000
#define MDIO_PHY_LINK_UP	0x2000

#define MDIO_PHY_SPEED_M10	0x0000
#define MDIO_PHY_SPEED_M100	0x0800
#define MDIO_PHY_SPEED_G1	0x1000

#define MDIO_PHY_FDUP_EN	0x0200
#define MDIO_PHY_FDUP_DIS	0x0600

#define MDIO_PHY_LINK_MASK	0x6000
#define MDIO_PHY_SPEED_MASK	0x1800
#define MDIO_PHY_FDUP_MASK	0x0600
#define MDIO_PHY_ADDR_MASK	0x001f
#define MDIO_UPDATE_MASK	MDIO_PHY_ADDR_MASK | MDIO_PHY_LINK_MASK | \
					MDIO_PHY_SPEED_MASK | MDIO_PHY_FDUP_MASK

/* MII */
#define MII_CFG(p)		(p * 8)

#define MII_CFG_EN		BIT(14)

#define MII_CFG_MODE_MIIP	0x0
#define MII_CFG_MODE_MIIM	0x1
#define MII_CFG_MODE_RMIIP	0x2
#define MII_CFG_MODE_RMIIM	0x3
#define MII_CFG_MODE_RGMII	0x4
#define MII_CFG_MODE_MASK	0xf

#define MII_CFG_RATE_M2P5	0x00
#define MII_CFG_RATE_M25	0x10
#define MII_CFG_RATE_M125	0x20
#define MII_CFG_RATE_M50	0x30
#define MII_CFG_RATE_AUTO	0x40
#define MII_CFG_RATE_MASK	0x70

/* cpu port mac */
#define PMAC_HD_CTL		0x0000
#define PMAC_RX_IPG		0x0024
#define PMAC_EWAN		0x002c

#define PMAC_IPG_MASK		0xf
#define PMAC_HD_CTL_AS		0x0008
#define PMAC_HD_CTL_AC		0x0004
#define PMAC_HD_CTL_RC		0x0010
#define PMAC_HD_CTL_RXSH	0x0040
#define PMAC_HD_CTL_AST		0x0080
#define PMAC_HD_CTL_RST		0x0100

/* PCE */
#define PCE_TBL_KEY(x)		(0x1100 + ((7 - x) * 4))
#define PCE_TBL_MASK		0x1120
#define PCE_TBL_VAL(x)		(0x1124 + ((4 - x) * 4))
#define PCE_TBL_ADDR		0x1138
#define PCE_TBL_CTRL		0x113c
#define PCE_PMAP1		0x114c
#define PCE_PMAP2		0x1150
#define PCE_PMAP3		0x1154
#define PCE_GCTRL_REG(x)	(0x1158 + (x * 4))
#define PCE_PCTRL_REG(p, x)	(0x1200 + (((p * 0xa) + x) * 4))

#define PCE_TBL_BUSY		BIT(15)
#define PCE_TBL_CFG_ADDR_MASK	0x1f
#define PCE_TBL_CFG_ADWR	0x20
#define PCE_TBL_CFG_ADWR_MASK	0x60
#define PCE_INGRESS		BIT(11)

/* MAC */
#define MAC_FLEN_REG		(0x2314)
#define MAC_CTRL_REG(p, x)	(0x240c + (((p * 0xc) + x) * 4))

/* buffer management */
#define BM_PCFG(p)		(0x200 + (p * 8))

/* special tag in TX path header */
#define SPID_SHIFT		24
#define DPID_SHIFT		16
#define DPID_ENABLE		1
#define SPID_CPU_PORT		2
#define PORT_MAP_SEL		BIT(15)
#define PORT_MAP_EN		BIT(14)
#define PORT_MAP_SHIFT		1
#define PORT_MAP_MASK		0x3f

#define SPPID_MASK		0x7
#define SPPID_SHIFT		4

/* MII regs not yet in linux */
#define MDIO_DEVAD_NONE		(-1)
#define ADVERTIZE_MPD		(1 << 10)

//dev_set_drvdata
//https://elixir.bootlin.com/linux/v5.1-rc5/source/include/linux/device.h#L1136
//SET_NETDEV_DEV
//https://elixir.bootlin.com/linux/v5.1-rc5/source/include/linux/netdevice.h#L2208


// TODO interesting: save the structure at the begin of the skb - free alloc and free
/* this is used in DMA ring to match skb during cleanup */
struct xrx200_tx_skb {
	/* skb in use reference */
	struct sk_buff *skb;

	/* tx queue reference for TX housekeeping */
	struct xrx200_tx_queue *txq;

	/* saved dma address for unmap */
	dma_addr_t addr;

	/* saved length for unmap, max 4096 */
	unsigned short size;
};


struct xrx200_rx_skb {
	/* skb in use reference */
	struct sk_buff *skb;

	/* saved dma address for unmap */
	dma_addr_t addr;

	/* saved length for unmap, max 4096 */
// 	unsigned short size;
};


//mapped exactly to one ring
struct xrx200_tx_queue {
	//only one writer must be possible!!
	struct xrx200_tx_skb *skb_list;	//up to ring sized number of skbs

	//can overflow?, scheduling
	unsigned int head;	//+1 when enqueued new packet into ring
	unsigned int tail;	//+1 when cleaned

	unsigned int num;	//queue ring size

	struct u64_stats_sync syncp;
	__u64 tx_packets;
	__u64 tx_bytes;
	__u64 tx_errors;
	__u64 tx_dropped;

	//associated netdev queue
	struct netdev_queue *nqueue;

	//for the TX queue list for TX ring
	struct list_head queue;	//TODO rename?

	struct xrx200_tx_ring *ring;	//associated ring
};


struct xrx200_tx_ring {
	struct napi_struct napi;

	/* ring buffer tail pointer */
	unsigned int free;
	// ____cacheline_aligned_in_smp;

	/* user side ring for list pointers to xrx200 skb buffer */
	//NOTICE array could be directly there, it would mean:
	//	ring locking or
	//	(MAX_SKB_FRAGS+1)*sizeof(struct xrx200_skb) bytes in local variables
	//	(MAX_SKB_FRAGS+1)*sizeof(struct xrx200_skb) memcpy'd bytes
	struct xrx200_tx_skb **skb_list_ptr;

	/* settings of this DMA ring */
	struct ltq_dma_channel dma;

	/* number of ring members */
	//TODO this should be adjustable and in ltq_dma_channel
	int num;

	struct xrx200_priv *priv;

	/* sharing the ring between multiple queues */
	spinlock_t lock;

	/* list of assigned TX queues, for IRQ */
	struct list_head queues;

	spinlock_t ref_lock;
	refcount_t refs;
};


//TODO is priv required in both queue and ring? probably just ring?
struct xrx200_rx_queue {
	/* netdev stats */
	struct u64_stats_sync syncp;
	__u64 rx_packets;
	__u64 rx_bytes;
	__u64 rx_dropped;

	/* link back to priv */
	struct xrx200_priv *priv;

	/* associated DMA ring */
	struct xrx200_rx_ring *ring;
};


struct xrx200_rx_ring {
	/* one napi for ring */
	struct napi_struct napi;

	/* associated DMA descriptors */
	struct ltq_dma_channel dma;

	/* link back to priv */
	struct xrx200_priv *priv;

	//TODO these two should be inside ltq_dma_channel
	/* additional data per DMA descriptor */
	struct xrx200_rx_skb *skb_list;

	/* number of descriptors */
	int num;

	spinlock_t ref_lock;
	refcount_t refs;
};


struct xrx200_port {
	u8 num;
	u8 phy_addr;
	u16 flags;
	phy_interface_t phy_if;

	struct list_head port;

	int link;
	int gpio;
	enum of_gpio_flags gpio_flags;

	struct phy_device *phydev;
	struct device_node *phy_node;
};

//MII-0 supports vlan separation
//MII-1 LAN/WAN switch

#define IF_TYPE_SWITCH		0
#define IF_TYPE_WAN		1

//TODO sort by sizes at the end
struct xrx200_iface {
	/* associated netdev */
	struct net_device *net_dev;	//TODO zero alloc, pak se da iterovat?

	/* TX queue, sw frontend for DMA ring */
	struct xrx200_tx_queue *txq;

	/* number of txq members */
	unsigned int num_tx_queues;

	/* RX queue, sw frontend for shared DMA ring */
	/* NOTICE hardcoded 1 queue, because only 1 RX ring */
	struct xrx200_rx_queue *rxq;

	/* interface's MAC address */
	unsigned char mac[6];

	/* link to private */
	//TODO better way?
	struct xrx200_priv *priv;

	/* interface list member, for priv*/
	struct list_head iface;

	/* head for port list */
	//TODO this is logical hierarchy, but everybody uses for each iface/for each port
	struct list_head ports;

	/* the interface is switch/wan type */
	//TODO maybe enum?
	unsigned int iftype;

	/* for interface timeouts */
	struct u64_stats_sync syncp;
	__u64 tx_errors;
};


struct xrx200_priv {
	/* for NAPI polling */
	struct net_device dummy_net;

	/* hardware DMA ring, 2 channels for TX, FIXME if you have TRM */
	struct xrx200_tx_ring tx[MAX_TX_RINGS];

	/* hardware DMA ring, 1 channel for RX, FIXME if you have TRM */
	struct xrx200_rx_ring rx[MAX_RX_RINGS];

	/* head for interfaces list */
 	struct list_head ifaces;

	struct clk *clk;

	struct device *dev;

	/* default portmap */
	//TODO only switch affected? maybe put into iface
	unsigned short d_port_map;

	//TODO seems wan map can be only at MII-1
	/* wan port map, maybe equivalent to d_port_map? */
	unsigned short wan_map;

	u16 vlan_vid[XRX200_MAX_VLAN];
	u16 vlan_port_map[XRX200_MAX_VLAN];

	/* RX: MII port to interface mapping */
	struct xrx200_iface *port_map[XRX200_MAX_PORT];

	struct mii_bus *mii_bus;

	struct switch_dev swdev;
};


static __iomem void *xrx200_switch_membase;
static __iomem void *xrx200_mii_membase;
static __iomem void *xrx200_mdio_membase;
static __iomem void *xrx200_pmac_membase;

#define ltq_switch_r32(x)	ltq_r32(xrx200_switch_membase + (x))
#define ltq_switch_w32(x, y)	ltq_w32(x, xrx200_switch_membase + (y))
#define ltq_switch_w32_mask(x, y, z) \
			ltq_w32_mask(x, y, xrx200_switch_membase + (z))

#define ltq_mdio_r32(x)		ltq_r32(xrx200_mdio_membase + (x))
#define ltq_mdio_w32(x, y)	ltq_w32(x, xrx200_mdio_membase + (y))
#define ltq_mdio_w32_mask(x, y, z) \
			ltq_w32_mask(x, y, xrx200_mdio_membase + (z))

#define ltq_mii_r32(x)		ltq_r32(xrx200_mii_membase + (x))
#define ltq_mii_w32(x, y)	ltq_w32(x, xrx200_mii_membase + (y))
#define ltq_mii_w32_mask(x, y, z) \
			ltq_w32_mask(x, y, xrx200_mii_membase + (z))

#define ltq_pmac_r32(x)		ltq_r32(xrx200_pmac_membase + (x))
#define ltq_pmac_w32(x, y)	ltq_w32(x, xrx200_pmac_membase + (y))
#define ltq_pmac_w32_mask(x, y, z) \
			ltq_w32_mask(x, y, xrx200_pmac_membase + (z))

#define XRX200_GLOBAL_REGATTR(reg) \
	.id = reg, \
	.type = SWITCH_TYPE_INT, \
	.set = xrx200_set_global_attr, \
	.get = xrx200_get_global_attr

#define XRX200_PORT_REGATTR(reg) \
	.id = reg, \
	.type = SWITCH_TYPE_INT, \
	.set = xrx200_set_port_attr, \
	.get = xrx200_get_port_attr

static int xrx200sw_read_x(int reg, int x)
{
	int value, mask, addr;

	addr = xrx200sw_reg[reg].offset + (xrx200sw_reg[reg].mult * x);
	value = ltq_switch_r32(addr);
	mask = (1 << xrx200sw_reg[reg].size) - 1;
	value = (value >> xrx200sw_reg[reg].shift);

	return (value & mask);
}

static int xrx200sw_read(int reg)
{
	return xrx200sw_read_x(reg, 0);
}

static void xrx200sw_write_x(int value, int reg, int x)
{
	int mask, addr;

	addr = xrx200sw_reg[reg].offset + (xrx200sw_reg[reg].mult * x);
	mask = (1 << xrx200sw_reg[reg].size) - 1;
	mask = (mask << xrx200sw_reg[reg].shift);
	value = (value << xrx200sw_reg[reg].shift) & mask;

	ltq_switch_w32_mask(mask, value, addr);
}

static void xrx200sw_write(int value, int reg)
{
	xrx200sw_write_x(value, reg, 0);
}

struct xrx200_pce_table_entry {
	int index;	// PCE_TBL_ADDR.ADDR = pData->table_index
	int table; 	// PCE_TBL_CTRL.ADDR = pData->table
	unsigned short key[8];
	unsigned short val[5];
	unsigned short mask;
	unsigned short type;
	unsigned short valid;
	unsigned short gmap;
};

static int xrx200_pce_table_entry_read(struct xrx200_pce_table_entry *tbl)
{
	// wait until hardware is ready
	while (xrx200sw_read(XRX200_PCE_TBL_CTRL_BAS)) {};

	// prepare the table access:
	// PCE_TBL_ADDR.ADDR = pData->table_index
	xrx200sw_write(tbl->index, XRX200_PCE_TBL_ADDR_ADDR);
	// PCE_TBL_CTRL.ADDR = pData->table
	xrx200sw_write(tbl->table, XRX200_PCE_TBL_CTRL_ADDR);

	//(address-based read)
	xrx200sw_write(0, XRX200_PCE_TBL_CTRL_OPMOD); // OPMOD_ADRD

	xrx200sw_write(1, XRX200_PCE_TBL_CTRL_BAS); // start access

	// wait until hardware is ready
	while (xrx200sw_read(XRX200_PCE_TBL_CTRL_BAS)) {};

	// read the keys
	tbl->key[7] = xrx200sw_read(XRX200_PCE_TBL_KEY_7);
	tbl->key[6] = xrx200sw_read(XRX200_PCE_TBL_KEY_6);
	tbl->key[5] = xrx200sw_read(XRX200_PCE_TBL_KEY_5);
	tbl->key[4] = xrx200sw_read(XRX200_PCE_TBL_KEY_4);
	tbl->key[3] = xrx200sw_read(XRX200_PCE_TBL_KEY_3);
	tbl->key[2] = xrx200sw_read(XRX200_PCE_TBL_KEY_2);
	tbl->key[1] = xrx200sw_read(XRX200_PCE_TBL_KEY_1);
	tbl->key[0] = xrx200sw_read(XRX200_PCE_TBL_KEY_0);

	// read the values
	tbl->val[4] = xrx200sw_read(XRX200_PCE_TBL_VAL_4);
	tbl->val[3] = xrx200sw_read(XRX200_PCE_TBL_VAL_3);
	tbl->val[2] = xrx200sw_read(XRX200_PCE_TBL_VAL_2);
	tbl->val[1] = xrx200sw_read(XRX200_PCE_TBL_VAL_1);
	tbl->val[0] = xrx200sw_read(XRX200_PCE_TBL_VAL_0);

	// read the mask
	tbl->mask = xrx200sw_read(XRX200_PCE_TBL_MASK_0);
	// read the type
	tbl->type = xrx200sw_read(XRX200_PCE_TBL_CTRL_TYPE);
	// read the valid flag
	tbl->valid = xrx200sw_read(XRX200_PCE_TBL_CTRL_VLD);
	// read the group map
	tbl->gmap = xrx200sw_read(XRX200_PCE_TBL_CTRL_GMAP);

	return 0;
}

static int xrx200_pce_table_entry_write(struct xrx200_pce_table_entry *tbl)
{
	// wait until hardware is ready
	while (xrx200sw_read(XRX200_PCE_TBL_CTRL_BAS)) {};

	// prepare the table access:
	// PCE_TBL_ADDR.ADDR = pData->table_index
	xrx200sw_write(tbl->index, XRX200_PCE_TBL_ADDR_ADDR);
	// PCE_TBL_CTRL.ADDR = pData->table
	xrx200sw_write(tbl->table, XRX200_PCE_TBL_CTRL_ADDR);

	//(address-based write)
	xrx200sw_write(1, XRX200_PCE_TBL_CTRL_OPMOD); // OPMOD_ADRD

	// read the keys
	xrx200sw_write(tbl->key[7], XRX200_PCE_TBL_KEY_7);
	xrx200sw_write(tbl->key[6], XRX200_PCE_TBL_KEY_6);
	xrx200sw_write(tbl->key[5], XRX200_PCE_TBL_KEY_5);
	xrx200sw_write(tbl->key[4], XRX200_PCE_TBL_KEY_4);
	xrx200sw_write(tbl->key[3], XRX200_PCE_TBL_KEY_3);
	xrx200sw_write(tbl->key[2], XRX200_PCE_TBL_KEY_2);
	xrx200sw_write(tbl->key[1], XRX200_PCE_TBL_KEY_1);
	xrx200sw_write(tbl->key[0], XRX200_PCE_TBL_KEY_0);

	// read the values
	xrx200sw_write(tbl->val[4], XRX200_PCE_TBL_VAL_4);
	xrx200sw_write(tbl->val[3], XRX200_PCE_TBL_VAL_3);
	xrx200sw_write(tbl->val[2], XRX200_PCE_TBL_VAL_2);
	xrx200sw_write(tbl->val[1], XRX200_PCE_TBL_VAL_1);
	xrx200sw_write(tbl->val[0], XRX200_PCE_TBL_VAL_0);

	// read the mask
	xrx200sw_write(tbl->mask, XRX200_PCE_TBL_MASK_0);
	// read the type
	xrx200sw_write(tbl->type, XRX200_PCE_TBL_CTRL_TYPE);
	// read the valid flag
	xrx200sw_write(tbl->valid, XRX200_PCE_TBL_CTRL_VLD);
	// read the group map
	xrx200sw_write(tbl->gmap, XRX200_PCE_TBL_CTRL_GMAP);

	xrx200sw_write(1, XRX200_PCE_TBL_CTRL_BAS); // start access

	// wait until hardware is ready
	while (xrx200sw_read(XRX200_PCE_TBL_CTRL_BAS)) {};

	return 0;
}

static void xrx200sw_fixup_pvids(void)
{
	int index, p, portmap, untagged;
	struct xrx200_pce_table_entry tem;
	struct xrx200_pce_table_entry tev;

	portmap = 0;
	for (p = 0; p < XRX200_MAX_PORT; p++)
		portmap |= BIT(p);

	tem.table = XRX200_PCE_VLANMAP_IDX;
	tev.table = XRX200_PCE_ACTVLAN_IDX;

	for (index = XRX200_MAX_VLAN; index-- > 0;)
	{
		tev.index = index;
		xrx200_pce_table_entry_read(&tev);

		if (tev.valid == 0)
			continue;

		tem.index = index;
		xrx200_pce_table_entry_read(&tem);

		if (tem.val[0] == 0)
			continue;

		untagged = portmap & (tem.val[1] ^ tem.val[2]);

		for (p = 0; p < XRX200_MAX_PORT; p++)
			if (untagged & BIT(p))
			{
				portmap &= ~BIT(p);
				xrx200sw_write_x(index, XRX200_PCE_DEFPVID_PVID, p);
			}

		for (p = 0; p < XRX200_MAX_PORT; p++)
			if (portmap & BIT(p))
				xrx200sw_write_x(index, XRX200_PCE_DEFPVID_PVID, p);
	}
}

// swconfig interface
static void xrx200_hw_init(struct xrx200_priv *priv);

// global
static int xrx200sw_reset_switch(struct switch_dev *dev)
{
	struct xrx200_priv *priv = container_of(dev, struct xrx200_priv, swdev);

	xrx200_hw_init(priv);

	return 0;
}

static int xrx200_set_vlan_mode_enable(struct switch_dev *dev, const struct switch_attr *attr, struct switch_val *val)
{
	int p;

	if ((attr->max > 0) && (val->value.i > attr->max))
		return -EINVAL;

	for (p = 0; p < XRX200_MAX_PORT; p++) {
		xrx200sw_write_x(val->value.i, XRX200_PCE_VCTRL_VEMR, p);
		xrx200sw_write_x(val->value.i, XRX200_PCE_VCTRL_VIMR, p);
	}

	xrx200sw_write(val->value.i, XRX200_PCE_GCTRL_0_VLAN);
	return 0;
}

static int xrx200_get_vlan_mode_enable(struct switch_dev *dev, const struct switch_attr *attr, struct switch_val *val)
{
	val->value.i = xrx200sw_read(attr->id);
	return 0;
}

static int xrx200_set_global_attr(struct switch_dev *dev, const struct switch_attr *attr, struct switch_val *val)
{
	if ((attr->max > 0) && (val->value.i > attr->max))
		return -EINVAL;

	xrx200sw_write(val->value.i, attr->id);
	return 0;
}

static int xrx200_get_global_attr(struct switch_dev *dev, const struct switch_attr *attr, struct switch_val *val)
{
	val->value.i = xrx200sw_read(attr->id);
	return 0;
}

// vlan
static int xrx200sw_set_vlan_vid(struct switch_dev *dev, const struct switch_attr *attr,
				 struct switch_val *val)
{
	struct xrx200_priv *priv = container_of(dev, struct xrx200_priv, swdev);
	int i;
	struct xrx200_pce_table_entry tev;
	struct xrx200_pce_table_entry tem;

	tev.table = XRX200_PCE_ACTVLAN_IDX;

	for (i = 0; i < XRX200_MAX_VLAN; i++)
	{
		tev.index = i;
		xrx200_pce_table_entry_read(&tev);
		if (tev.key[0] == val->value.i && i != val->port_vlan)
			return -EINVAL;
	}

	priv->vlan_vid[val->port_vlan] = val->value.i;

	tev.index = val->port_vlan;
	xrx200_pce_table_entry_read(&tev);
	tev.key[0] = val->value.i;
	tev.valid = val->value.i > 0;
	xrx200_pce_table_entry_write(&tev);

	tem.table = XRX200_PCE_VLANMAP_IDX;
	tem.index = val->port_vlan;
	xrx200_pce_table_entry_read(&tem);
	tem.val[0] = val->value.i;
	xrx200_pce_table_entry_write(&tem);

	xrx200sw_fixup_pvids();
	return 0;
}

static int xrx200sw_get_vlan_vid(struct switch_dev *dev, const struct switch_attr *attr,
				 struct switch_val *val)
{
	struct xrx200_pce_table_entry te;

	te.table = XRX200_PCE_ACTVLAN_IDX;
	te.index = val->port_vlan;
	xrx200_pce_table_entry_read(&te);
	val->value.i = te.key[0];

	return 0;
}

static int xrx200sw_set_vlan_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct xrx200_priv *priv = container_of(dev, struct xrx200_priv, swdev);
	int i, portmap, tagmap, untagged;
	struct xrx200_pce_table_entry tem;

	portmap = 0;
	tagmap = 0;
	for (i = 0; i < val->len; i++)
	{
		struct switch_port *p = &val->value.ports[i];

		portmap |= (1 << p->id);
		if (p->flags & (1 << SWITCH_PORT_FLAG_TAGGED))
			tagmap |= (1 << p->id);
	}

	tem.table = XRX200_PCE_VLANMAP_IDX;

	untagged = portmap ^ tagmap;
	for (i = 0; i < XRX200_MAX_VLAN; i++)
	{
		tem.index = i;
		xrx200_pce_table_entry_read(&tem);

		if (tem.val[0] == 0)
			continue;

		if ((untagged & (tem.val[1] ^ tem.val[2])) && (val->port_vlan != i))
			return -EINVAL;
	}

	tem.index = val->port_vlan;
	xrx200_pce_table_entry_read(&tem);

	// auto-enable this vlan if not enabled already
	if (tem.val[0] == 0)
	{
		struct switch_val v;
		v.port_vlan = val->port_vlan;
		v.value.i = val->port_vlan;
		if(xrx200sw_set_vlan_vid(dev, NULL, &v))
			return -EINVAL;

		//read updated tem
		tem.index = val->port_vlan;
		xrx200_pce_table_entry_read(&tem);
	}

	tem.val[1] = portmap;
	tem.val[2] = tagmap;
	xrx200_pce_table_entry_write(&tem);

pr_info("set vlan portmap\n");
	ltq_switch_w32_mask(0, portmap, PCE_PMAP2);
	ltq_switch_w32_mask(0, portmap, PCE_PMAP3);
	priv->vlan_port_map[val->port_vlan] = portmap;

	xrx200sw_fixup_pvids();

	return 0;
}

static int xrx200sw_get_vlan_ports(struct switch_dev *dev, struct switch_val *val)
{
	int i;
	unsigned short ports, tags;
	struct xrx200_pce_table_entry tem;

	tem.table = XRX200_PCE_VLANMAP_IDX;
	tem.index = val->port_vlan;
	xrx200_pce_table_entry_read(&tem);

	ports = tem.val[1];
	tags = tem.val[2];

	for (i = 0; i < XRX200_MAX_PORT; i++) {
		struct switch_port *p;

		if (!(ports & (1 << i)))
			continue;

		p = &val->value.ports[val->len++];
		p->id = i;
		if (tags & (1 << i))
			p->flags = (1 << SWITCH_PORT_FLAG_TAGGED);
		else
			p->flags = 0;
	}

	return 0;
}

static int xrx200sw_set_vlan_enable(struct switch_dev *dev, const struct switch_attr *attr,
				 struct switch_val *val)
{
	struct xrx200_pce_table_entry tev;

	tev.table = XRX200_PCE_ACTVLAN_IDX;
	tev.index = val->port_vlan;
	xrx200_pce_table_entry_read(&tev);

	if (tev.key[0] == 0)
		return -EINVAL;

	tev.valid = val->value.i;
	xrx200_pce_table_entry_write(&tev);

	xrx200sw_fixup_pvids();
	return 0;
}

static int xrx200sw_get_vlan_enable(struct switch_dev *dev, const struct switch_attr *attr,
				 struct switch_val *val)
{
	struct xrx200_pce_table_entry tev;

	tev.table = XRX200_PCE_ACTVLAN_IDX;
	tev.index = val->port_vlan;
	xrx200_pce_table_entry_read(&tev);
	val->value.i = tev.valid;

	return 0;
}

// port
static int xrx200sw_get_port_pvid(struct switch_dev *dev, int port, int *val)
{
	struct xrx200_pce_table_entry tev;

	if (port >= XRX200_MAX_PORT)
		return -EINVAL;

	tev.table = XRX200_PCE_ACTVLAN_IDX;
	tev.index = xrx200sw_read_x(XRX200_PCE_DEFPVID_PVID, port);
	xrx200_pce_table_entry_read(&tev);

	*val = tev.key[0];
	return 0;
}

static int xrx200sw_get_port_link(struct switch_dev *dev,
				  int port,
				  struct switch_port_link *link)
{
	if (port >= XRX200_MAX_PORT)
		return -EINVAL;

	link->link = xrx200sw_read_x(XRX200_MAC_PSTAT_LSTAT, port);
	if (!link->link)
		return 0;

	link->duplex = xrx200sw_read_x(XRX200_MAC_PSTAT_FDUP, port);

	link->rx_flow = !!(xrx200sw_read_x(XRX200_MAC_CTRL_0_FCON, port) & 0x0010);
	link->tx_flow = !!(xrx200sw_read_x(XRX200_MAC_CTRL_0_FCON, port) & 0x0020);
	link->aneg = !(xrx200sw_read_x(XRX200_MAC_CTRL_0_FCON, port));

	link->speed = SWITCH_PORT_SPEED_10;
	if (xrx200sw_read_x(XRX200_MAC_PSTAT_MBIT, port))
		link->speed = SWITCH_PORT_SPEED_100;
	if (xrx200sw_read_x(XRX200_MAC_PSTAT_GBIT, port))
		link->speed = SWITCH_PORT_SPEED_1000;

	return 0;
}


static int xrx200sw_set_port_link(struct switch_dev *dev, int port,
				  struct switch_port_link *link)
{
	if (port >= XRX200_MAX_PORT)
		return -EINVAL;

	return switch_generic_set_link(dev, port, link);
}

static int xrx200_mdio_wr(struct mii_bus *bus, int addr, int reg, u16 val);
static int xrx200_mdio_rd(struct mii_bus *bus, int addr, int reg);

static int xrx200sw_phy_read16(struct switch_dev *dev, int addr, u8 reg, u16 *value)
{
	struct xrx200_priv *priv = container_of(dev, struct xrx200_priv, swdev);
	struct list_head *pos;
	u8 phy_addr;

	list_for_each(pos, &priv->ifaces) {
		struct list_head *port_list;
		struct xrx200_iface *iface =
			list_entry(pos, struct xrx200_iface, iface);

		list_for_each(port_list, &iface->ports) {
			struct xrx200_port *port =
				list_entry(port_list, struct xrx200_port,
					   port);
			if (addr == port->num) {
				phy_addr = port->phy_addr;
				goto found;
			}
		}
	}
	phy_addr = 0;	//error?
found:

	//TODO
	//lantiq,xrx200-pdi-port
	//addr = xrx200_port->num
	//value is reg from lantiq,phy11g

	*value =  xrx200_mdio_rd(priv->mii_bus, phy_addr, reg);

	return 0;
}

static int xrx200sw_phy_write16(struct switch_dev *dev, int addr, u8 reg, u16 value)
{
	struct xrx200_priv *priv = container_of(dev, struct xrx200_priv, swdev);
	struct list_head *pos;
	u8 phy_addr;

	list_for_each(pos, &priv->ifaces) {
		struct list_head *port_list;
		struct xrx200_iface *iface =
		list_entry(pos, struct xrx200_iface, iface);

		list_for_each(port_list, &iface->ports) {
			struct xrx200_port *port =
			list_entry(port_list, struct xrx200_port,
				   port);
			if (addr == port->num) {
				phy_addr = port->phy_addr;
				goto found;
			}
		}
	}
	phy_addr = 0;	//error?
found:

	return xrx200_mdio_wr(priv->mii_bus, phy_addr, reg, value);
}

static int xrx200_set_port_attr(struct switch_dev *dev, const struct switch_attr *attr, struct switch_val *val)
{
	if (val->port_vlan >= XRX200_MAX_PORT)
		return -EINVAL;

	if ((attr->max > 0) && (val->value.i > attr->max))
		return -EINVAL;

	xrx200sw_write_x(val->value.i, attr->id, val->port_vlan);
	return 0;
}

static int xrx200_get_port_attr(struct switch_dev *dev, const struct switch_attr *attr, struct switch_val *val)
{
	if (val->port_vlan >= XRX200_MAX_PORT)
		return -EINVAL;

	val->value.i = xrx200sw_read_x(attr->id, val->port_vlan);
	return 0;
}

// attributes
static struct switch_attr xrx200sw_globals[] = {
	{
		.type = SWITCH_TYPE_INT,
		.set = xrx200_set_vlan_mode_enable,
		.get = xrx200_get_vlan_mode_enable,
		.name = "enable_vlan",
		.description = "Enable VLAN mode",
		.max = 1},
};

static struct switch_attr xrx200sw_port[] = {
	{
	XRX200_PORT_REGATTR(XRX200_PCE_VCTRL_UVR),
	.name = "uvr",
	.description = "Unknown VLAN Rule",
	.max = 1,
	},
	{
	XRX200_PORT_REGATTR(XRX200_PCE_VCTRL_VSR),
	.name = "vsr",
	.description = "VLAN Security Rule",
	.max = 1,
	},
	{
	XRX200_PORT_REGATTR(XRX200_PCE_VCTRL_VINR),
	.name = "vinr",
	.description = "VLAN Ingress Tag Rule",
	.max = 2,
	},
	{
	XRX200_PORT_REGATTR(XRX200_PCE_PCTRL_0_TVM),
	.name = "tvm",
	.description = "Transparent VLAN Mode",
	.max = 1,
	},
};

static struct switch_attr xrx200sw_vlan[] = {
	{
		.type = SWITCH_TYPE_INT,
		.name = "vid",
		.description = "VLAN ID (0-4094)",
		.set = xrx200sw_set_vlan_vid,
		.get = xrx200sw_get_vlan_vid,
		.max = 4094,
	},
	{
		.type = SWITCH_TYPE_INT,
		.name = "enable",
		.description = "Enable VLAN",
		.set = xrx200sw_set_vlan_enable,
		.get = xrx200sw_get_vlan_enable,
		.max = 1,
	},
};

static const struct switch_dev_ops xrx200sw_ops = {
	.attr_global = {
		.attr = xrx200sw_globals,
		.n_attr = ARRAY_SIZE(xrx200sw_globals),
	},
	.attr_port = {
		.attr = xrx200sw_port,
		.n_attr = ARRAY_SIZE(xrx200sw_port),
	},
	.attr_vlan = {
		.attr = xrx200sw_vlan,
		.n_attr = ARRAY_SIZE(xrx200sw_vlan),
	},
	.get_vlan_ports = xrx200sw_get_vlan_ports,
	.set_vlan_ports = xrx200sw_set_vlan_ports,
	.get_port_pvid = xrx200sw_get_port_pvid,
	.reset_switch = xrx200sw_reset_switch,
	.get_port_link = xrx200sw_get_port_link,
	.set_port_link = xrx200sw_set_port_link,
//	.get_port_stats = xrx200sw_get_port_stats, //TODO
	.phy_read16 = xrx200sw_phy_read16,
	.phy_write16 = xrx200sw_phy_write16,
};

static int xrx200sw_init(struct xrx200_iface *iface)
{

	struct switch_dev *swdev;

	//TODO is it possible to have working eth without switch? (only wan?)
	//TODO put into interface init under switch block

	swdev = &iface->priv->swdev;

	swdev->name = "Lantiq XRX200 Switch";
	swdev->vlans = XRX200_MAX_VLAN;
	swdev->ports = XRX200_MAX_PORT;
	swdev->cpu_port = 6;
	swdev->ops = &xrx200sw_ops;

	//TODO there should be retval?
	return register_switch(swdev, iface->net_dev);
}

/* drop all the packets from the DMA ring */
static void xrx200_flush_rx(struct xrx200_rx_ring *ring)
{
	int i;

	for (i = 0; i < ring->num; i++) {
		struct ltq_dma_desc *desc = &ring->dma.desc_base[ring->dma.desc];

		if ((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) != LTQ_DMA_C)
			break;

		desc->ctl = LTQ_DMA_OWN | LTQ_DMA_RX_OFFSET(NET_IP_ALIGN) |
				XRX200_DMA_DATA_LEN;

		ring->dma.desc = (ring->dma.desc + 1) % ring->num;
	}
}


static int xrx200_open(struct net_device *dev)
{
	struct xrx200_iface *iface = netdev_priv(dev);
	struct list_head *pos;
	int i;
	unsigned long flags;
	unsigned int refcount;

	pr_info("a1\n");

	for (i = 0; i < iface->num_tx_queues; i++) {
		//TODO must be locked
//TODO refcount!!!!!
		spin_lock_irqsave(&iface->txq[i].ring->ref_lock, flags);
		refcount = refcount_read(&iface->txq[i].ring->refs);
		refcount_inc(&iface->txq[i].ring->refs);
		spin_unlock_irqrestore(&iface->txq[i].ring->ref_lock, flags);

		if (!refcount) {
			pr_info("a2\n");
			napi_enable(&iface->txq[i].ring->napi);
			pr_info("a3\n");
			ltq_dma_open(&iface->txq[i].ring->dma);
			pr_info("a4\n");
			ltq_dma_enable_irq(&iface->txq[i].ring->dma);
			pr_info("a5\n");
		}
	}
	pr_info("b1\n");

	spin_lock_irqsave(&iface->rxq[0].ring->ref_lock, flags);
	refcount = refcount_read(&iface->rxq[0].ring->refs);
	refcount_inc(&iface->rxq[0].ring->refs);
	spin_unlock_irqrestore(&iface->rxq[0].ring->ref_lock, flags);

	if (!refcount) {

//TODO first dma, then napi?
	//TODO must be locked, tx_open?
		napi_enable(&iface->rxq[0].ring->napi);
		pr_info("b2\n");
		ltq_dma_open(&iface->rxq[0].ring->dma);
		pr_info("b2\n");
	}

	/* The boot loader does not always deactivate the receiving of frames
	 * on the ports and then some packets queue up in the PPE buffers.
	 * They already passed the PMAC so they do not have the tags
	 * configured here. Read the these packets here and drop them.
	 * The HW should have written them into memory after 10us
	 */
	usleep_range(20, 40);
	xrx200_flush_rx(iface->rxq[0].ring);

	ltq_dma_enable_irq(&iface->rxq[0].ring->dma);

	list_for_each(pos, &iface->ports) {
		struct xrx200_port *port = list_entry(pos,
						      struct xrx200_port,
						      port);
		if (port->phydev)
			phy_start(port->phydev);
	}

	netif_tx_wake_all_queues(dev);

	return 0;
}

static int xrx200_close(struct net_device *dev)
{
	struct xrx200_iface *iface = netdev_priv(dev);
	struct list_head *pos;
	int i;
	unsigned long flags;
	unsigned int refcount;

	netif_tx_stop_all_queues(dev);

	list_for_each(pos, &iface->ports) {
		struct xrx200_port *port = list_entry(pos,
						      struct xrx200_port,
						      port);
		if (port->phydev)
			phy_stop(port->phydev);
	}

	spin_lock_irqsave(&iface->rxq[0].ring->ref_lock, flags);
	refcount_dec(&iface->rxq[0].ring->refs);
	refcount = refcount_read(&iface->rxq[0].ring->refs);
	spin_unlock_irqrestore(&iface->rxq[0].ring->ref_lock, flags);

	if (!refcount) {
		napi_disable(&iface->rxq[0].ring->napi);
		ltq_dma_close(&iface->rxq[0].ring->dma);
	}

	for (i = 0; i < iface->num_tx_queues; i++) {
		spin_lock_irqsave(&iface->txq[i].ring->ref_lock, flags);
		refcount_dec(&iface->txq[i].ring->refs);
		refcount = refcount_read(&iface->txq[i].ring->refs);
		spin_unlock_irqrestore(&iface->txq[i].ring->ref_lock, flags);

		if (!refcount) {
			napi_disable(&iface->txq[i].ring->napi);
			ltq_dma_close(&iface->txq[i].ring->dma);
		}
	}

	return 0;
}

//TODO maybe change priv to ring, alloc to RX?
static int xrx200_alloc_skb(struct xrx200_rx_ring *ring,
			    struct ltq_dma_channel *dma,
			    struct xrx200_rx_skb *dma_skb)
{
	struct ltq_dma_desc *base = &dma->desc_base[dma->desc];
	struct sk_buff *skb;

//#define DMA_PAD	(NET_IP_ALIGN + NET_SKB_PAD)
//   	skb = netdev_alloc_skb(priv->net_dev, XRX200_DMA_DATA_LEN + DMA_PAD);

	skb = napi_alloc_skb(&ring->napi, XRX200_DMA_DATA_LEN);

	//TODO fix fail path
	if (unlikely(!skb)) {
		/* leave the old skb if not enough memory */
		goto skip;
	}

	if (likely(dma_skb->addr)) {
		dma_unmap_single(ring->priv->dev, dma_skb->addr,
				 XRX200_DMA_DATA_LEN, DMA_FROM_DEVICE);
	}

	// 	skb_reserve(skb, NET_SKB_PAD);
	skb_reserve(skb, -NET_IP_ALIGN);

	base->addr = dma_skb->addr =
		dma_map_single(ring->priv->dev, skb->data,
			       XRX200_DMA_DATA_LEN, DMA_FROM_DEVICE);

//TODO error path
// 		if (dma_mapping_error(&cp->pdev->dev, new_mapping)) {
// 			dev->stats.rx_dropped++;
// 			kfree_skb(new_skb);
// 			goto rx_next;
// 		}


 	skb_reserve(skb, NET_IP_ALIGN);

	dma_skb->skb = skb;

	wmb();

skip:
	base->ctl = LTQ_DMA_OWN | LTQ_DMA_RX_OFFSET(NET_IP_ALIGN) |
		XRX200_DMA_DATA_LEN;

	dma->desc = (dma->desc + 1) % ring->num;

	return 0;
}


static void xrx200_hw_receive(struct xrx200_rx_ring *ring,
			      struct xrx200_iface *iface,
			      struct ltq_dma_channel *dma,
			      struct xrx200_rx_skb *dma_skb
			      )
{
	struct ltq_dma_desc *desc = &dma->desc_base[dma->desc];
	struct net_device *dev;
	struct sk_buff *filled_skb = dma_skb->skb;
	int len = (desc->ctl & LTQ_DMA_SIZE_MASK);
	int ret;
	struct xrx200_rx_queue *rxq;

	if (!iface) {
		dev_warn(ring->priv->dev, "Portmap to iface not defined!\n");
		return;
	}

	dev = iface->net_dev;
	rxq = iface->rxq;

	/* alloc new skb first so DMA ring can work during netif_receive_skb */
	ret = xrx200_alloc_skb(ring, dma, dma_skb);

	if (ret) {
		netdev_err(dev,
			"failed to allocate new rx buffer\n");

		//TODO
		return;
	}

	/* set skb length for netdev */
	skb_put(filled_skb, len);
#ifdef SW_ROUTING
	/* remove special tag */
	skb_pull(filled_skb, 8);
#endif

	filled_skb->protocol = eth_type_trans(filled_skb, dev);

	//TODO redo for netif_receive_skb_list? problem is deciding overhead between netdev
	ret = netif_receive_skb(filled_skb);

	if (likely(ret == NET_RX_SUCCESS)) {
		u64_stats_update_begin(&rxq->syncp);
		rxq->rx_bytes += len;
		rxq->rx_packets++;
		u64_stats_update_end(&rxq->syncp);
	} else {

		u64_stats_update_begin(&rxq->syncp);
		rxq->rx_dropped++;
		u64_stats_update_end(&rxq->syncp);
	}

// info napi_gro_receive(&rxq->napi, filled_skb); too simple soc?

}

static int xrx200_poll_rx(struct napi_struct *napi, int budget)
{
	struct xrx200_rx_ring *ring = container_of(napi,
						  struct xrx200_rx_ring, napi);
	struct ltq_dma_channel *dma = &ring->dma;
	struct xrx200_priv *priv = ring->priv;
	int rx = 0;

	//read the ring
	//put skb to the correct queue
	//(if all queues have more items than ring -> ring will have to wait)
	//if all queues have less items than ring -> ring may get stuck
	//TODO delete ^^

	while (rx < budget) {
		struct ltq_dma_desc *desc = &dma->desc_base[dma->desc];
		if (likely((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) == LTQ_DMA_C)) {
			struct xrx200_rx_skb *dma_skb = &ring->skb_list[dma->desc];

//TODO TODO TODO
#if 1
#if 1
#ifdef SW_ROUTING
			const unsigned char port = (dma_skb->skb->data[7] >> SPPID_SHIFT) & SPPID_MASK;
#else
//TODO not sure if this codepath works, anyone using !SW_ROUTING here?
			const unsigned char port = 0;
#endif
#endif
#else

//TODO forced interface 0, test uninialized ifaces -> NULL pointers
const unsigned char port = 0;
#endif


// TODO it could be even possible to make a single eth interface for every rj45 connector, but things like mac addresses...

#if 0
	{
		int i;

pr_info("%px: %px %px %px %i %i %px, dump:\n",
napi,
dma,
dma_skb,
dma_skb->skb,
dma->desc,
port,
priv->port_map[port]);

		for (i = 0; i < 16;i++) {
			pr_cont("%02x ", dma_skb->skb->data[i]);
			if ((i % 8) == 7)
				pr_cont("\n");
		}
		pr_cont("\n");
	}
#endif

//port (LAN switch/wan) is mapped on xrx200_iface
			xrx200_hw_receive(ring, priv->port_map[port], dma,
					  dma_skb);
			rx++;
		} else {
			break;
		}
	}

	//TODO
	if (rx < budget) {
		if (napi_complete_done(napi, rx)) {
//can an unacked irq event wait here now?
			ltq_dma_enable_irq(dma);
		}
	}

	return rx;
}

//TODO is this macro valid?
#define TX_BUFFS_AVAIL(tail, head, num)		\
	((tail <= head) ?			\
	  tail + (num - 1) - head :	\
	  tail - head - 1)

static int xrx200_tx_housekeeping(struct napi_struct *napi, int budget)
{
	struct xrx200_tx_ring *ring =
		container_of(napi, struct xrx200_tx_ring, napi);
		//	struct net_device *net_dev = txq->priv->net_dev;
	int pkts = 0;	//napi complete has int, hw will fit into usinged short
	unsigned short size = 0;
 	unsigned int free;
	struct xrx200_tx_queue *txq;
	struct ltq_dma_desc *desc;
	struct list_head *pos;

	free = ring->free;	//read once? TODO

//TODO pkts vs frags, frags means fullness of ring, pkts means napi budget!

	while (likely(pkts < budget)) {
		desc = &ring->dma.desc_base[free];

		//TODO into while condition? an additional read
		//speed tests
		if ((desc->ctl & (LTQ_DMA_OWN | LTQ_DMA_C)) == LTQ_DMA_C) {
			struct xrx200_tx_skb *dma_skb = ring->skb_list_ptr[free];

			//all frags after SOP are from the same TXQ
			//NOTICE but frags/pkts
// 			if (desc->ctl & LTQ_DMA_SOP) {
// 				txq = skb_list_ptr[free]->txq;
// 				tail = txq->tail;
// 			}

			//TODO txq operations could be speed up, between SOP and EOP
			// there must be the same txq
			// also we could accumulate stats for one txq run

			txq = dma_skb->txq;

			size = dma_skb->size;

			//TODO device
			dma_unmap_single(ring->priv->dev, dma_skb->addr,
					 (size_t) size, DMA_TO_DEVICE);

			if (desc->ctl & LTQ_DMA_EOP) {
				//TODO there could be a local variable and update it at the end
				//problem: mixed queues
				u64_stats_update_begin(&txq->syncp);
				txq->tx_packets++;
				txq->tx_bytes += size;
				u64_stats_update_end(&txq->syncp);

				if (likely(dma_skb->skb)) {
					//last frag
					dev_consume_skb_irq(dma_skb->skb);
					dma_skb->skb = NULL;	//TODO is it required?
					pkts++;
				} else {
					//debug?
					dev_warn(ring->priv->dev, "TX ring skb pointer is NULL\n");
				}
			} else {
				//TODO there could be a local variable and update it at the end
				//problem: mixed queues
				u64_stats_update_begin(&txq->syncp);
				txq->tx_bytes += size;
				u64_stats_update_end(&txq->syncp);
			}


			txq->tail = (txq->tail + 1) % txq->num;

			/* erase descriptor flags */
			desc->ctl = 0;

			free = (free + 1) % ring->num;
		} else {
			break;
		}
	}

	ring->free = free;

	/* test which queue housekeeping should be scheduled */
	list_for_each(pos, &ring->queues) {
		txq = list_entry(pos, struct xrx200_tx_queue, queue);

		/* wake up queue only if there is enough space */
		if (unlikely(TX_BUFFS_AVAIL(txq->tail, txq->head, txq->num) > (MAX_SKB_FRAGS + 1))) {
			if (netif_tx_queue_stopped(txq->nqueue)) {
				netif_tx_wake_queue(txq->nqueue);
			}
		}
	}

	if (pkts < budget) {
		if (napi_complete_done(napi, pkts)) {
			ltq_dma_enable_irq(&ring->dma);
		}
	}

	return pkts;
}


static void xrx200_tx_timeout(struct net_device *ndev)
{
	struct xrx200_iface *iface = netdev_priv(ndev);

	netdev_err(ndev, "transmit timed out!\n");

	u64_stats_update_begin(&iface->syncp);
	iface->tx_errors++;
	u64_stats_update_end(&iface->syncp);

	netif_tx_wake_all_queues(ndev);
}


#if 0
//testing "packet"
char test_packet[] = {
	0,0,0,0,	//headroom
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc4, 0xe9, 0x84, 0x2a, 0xbd, 0x51, 0x08, 0x06, 0x00, 0x01,
	0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xc4, 0xe9, 0x84, 0x2a, 0xbd, 0x51, 0x0a, 0x00, 0x00, 0x50,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0a, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

long test_counter=1;

#endif

//TODO netdev queues, stats per queue or netdev, is xmit exclusive
//	struct napi_struct	napi ____cacheline_aligned;
//https://elixir.bootlin.com/linux/v5.1-rc5/source/drivers/net/ethernet/broadcom/bcmsysport.h#L740
//	struct ltq_etop_priv *priv = netdev_priv(dev);
//stats collision in housekeeping per queue/netdev/ring ?
//MAX_SKB_FRAGS

static netdev_tx_t xrx200_start_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	struct xrx200_iface *iface = netdev_priv(dev);
	struct xrx200_priv *priv = iface->priv;
	struct xrx200_tx_queue *txq;
	struct xrx200_tx_ring *ring;
	unsigned int ring_head;
	unsigned int head;
	int ret = NETDEV_TX_OK;
	int len;
	int i;
	u16 queue_id;
#ifdef SW_ROUTING
	u32 special_tag = (SPID_CPU_PORT << SPID_SHIFT) | DPID_ENABLE;
#endif
	unsigned long flags;
	unsigned int idx;

	//TODO will always skb queue match nqueue from txq?
	queue_id = skb_get_queue_mapping(skb);
	txq = &iface->txq[queue_id];

	if (unlikely(TX_BUFFS_AVAIL(txq->tail, txq->head, txq->num) <= (MAX_SKB_FRAGS + 1))) {
		netif_tx_stop_queue(txq->nqueue);

		/*
		 * This is usually a bug, the code must foresee this
		 * at the end of this function
		 */
		netdev_err(dev, "not enough space on queue %i\n", queue_id);

		return NETDEV_TX_BUSY;
	}

	if (skb_put_padto(skb, ETH_ZLEN)) {
		/* XXX: is this pr_err or normal/none code path? */
		u64_stats_update_begin(&txq->syncp);
		txq->tx_dropped++;
		u64_stats_update_end(&txq->syncp);

		return NETDEV_TX_OK;
	}

	//TODO is support for more than one queue per cpu per iface overkill?
// 	ring = &priv->tx[queue_id % MAX_TX_RINGS];
	ring = txq->ring;

#ifdef SW_ROUTING
	if (is_multicast_ether_addr(eth_hdr(skb)->h_dest)) {
		u16 port_map = priv->d_port_map;

// 		pr_info("multicast\n");

		if ((iface->iftype & BIT(IF_TYPE_SWITCH)) &&
		    skb->protocol == htons(ETH_P_8021Q)) {
			u16 vid;
			int i;

			port_map = 0;
			if (!__vlan_get_tag(skb, &vid)) {
				for (i = 0; i < XRX200_MAX_VLAN; i++) {
					if (priv->vlan_vid[i] == vid) {
						port_map = priv->vlan_port_map[i];
						break;
					}
				}
			}
		}

		special_tag |= (port_map << PORT_MAP_SHIFT) |
			       PORT_MAP_SEL | PORT_MAP_EN;
	}

	/* Use MII-1 for WAN separation */
	if (iface->iftype & BIT(IF_TYPE_WAN)) {
// 		pr_info("wan\n");
		special_tag |= (1 << DPID_SHIFT);
	}

	if (skb_headroom(skb) < XRX200_HEADROOM) {
		struct sk_buff *tmp = skb_realloc_headroom(skb, XRX200_HEADROOM);
		dev_kfree_skb_any(skb);
		skb = tmp;
	}

	skb_push(skb, XRX200_HEADROOM);
	memcpy(skb->data, &special_tag, sizeof(u32));
#endif

	if (skb_shinfo(skb)->nr_frags == 0) {
		len = skb->len;
	} else {
		len = skb_headlen(skb);
	}

	head = txq->head;

	/* Map to DMA first, we cannot fail after ring allocation */

	/* map basic fragment of packet */
	// TODO,weird : etop uses virt_to_phys, but here it would not work
	txq->skb_list[head].addr = dma_map_single(priv->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(priv->dev, txq->skb_list[head].addr))) {
		netdev_err(dev, "DMA mapping failed for skb\n");

		dev_kfree_skb(skb);

		u64_stats_update_begin(&txq->syncp);
		txq->tx_dropped++;
		txq->tx_errors++;
		u64_stats_update_end(&txq->syncp);

		ret = NETDEV_TX_OK;
		goto out;
	}

	txq->skb_list[head].size = len;

	/* map rest of fragments */
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		idx = (head + i + 1) % txq->num;

		len = skb_frag_size(frag);

		txq->skb_list[idx].addr = dma_map_single(priv->dev,
						  skb_frag_address(frag),
						  len, DMA_TO_DEVICE);

		if (unlikely(dma_mapping_error(priv->dev, txq->skb_list[idx].addr))) {
			netdev_err(dev, "DMA mapping for fragment #%i failed\n", i);

			i--;

			for (; i >= 0; i--) {
				idx = (head + i + 1) % txq->num;
				dma_unmap_single(priv->dev, txq->skb_list[idx].addr,
						 txq->skb_list[idx].size,
						 DMA_TO_DEVICE);
			}

			dma_unmap_single(priv->dev, txq->skb_list[head].addr,
					 txq->skb_list[head].size, DMA_TO_DEVICE);

			dev_kfree_skb(skb);

			u64_stats_update_begin(&txq->syncp);
			txq->tx_dropped++;
			txq->tx_errors++;
			u64_stats_update_end(&txq->syncp);

			ret = NETDEV_TX_OK;
			goto out;
		}

		txq->skb_list[idx].size = len;
	}

	/* we need to lock from other queues using the same ring */
	spin_lock_irqsave(&ring->lock, flags);

	ring_head = ring->dma.desc;

	/* check if there is free space in DMA ring */
	if (unlikely(TX_BUFFS_AVAIL(ring->free, ring_head, ring->num) <= (MAX_SKB_FRAGS + 1))) {
		spin_unlock_irqrestore(&ring->lock, flags);

		netif_tx_stop_queue(txq->nqueue);
		netdev_err(dev, "not enough space on ring %i\n", queue_id);

		for (i = skb_shinfo(skb)->nr_frags; i >= 0; i--) {
			idx = (head + i + 1) % txq->num;

			dma_unmap_single(priv->dev, txq->skb_list[idx].addr,
					 txq->skb_list[idx].size, DMA_TO_DEVICE);
		}

		dma_unmap_single(priv->dev, txq->skb_list[idx].addr,
				 txq->skb_list[idx].size, DMA_TO_DEVICE);

		return NETDEV_TX_BUSY;
	}

	/* Allocate the space in DMA ring, we cannot fail, ring shared */
	ring->dma.desc = (ring_head + skb_shinfo(skb)->nr_frags + 1) % ring->num;

	spin_unlock_irqrestore(&ring->lock, flags);

	/* Allocate the space for queue ring */
	txq->head = (head + skb_shinfo(skb)->nr_frags + 1) % txq->num;

	/* fill all descriptors from queue ring to DMA ring */
	for (i = 0; i < skb_shinfo(skb)->nr_frags + 1; i++) {
		struct ltq_dma_desc *desc;
		struct xrx200_tx_skb *skb_list;
		unsigned int ring_idx = (ring_head + i) % ring->num;

		desc = &ring->dma.desc_base[ring_idx];
		skb_list = &txq->skb_list[(head + i) % txq->num];

		skb_list->skb = skb;
		skb_list->txq = txq;

		ring->skb_list_ptr[ring_idx] = skb_list;

		desc->addr = (skb_list->addr & 0x1fffffe0) | (1<<31);

		if (i == 0) {
			/* first frag of packet needs SOP, cannot have OWN yet */
			desc->ctl = LTQ_DMA_SOP |
				LTQ_DMA_TX_OFFSET(skb_list->addr & XRX200_DMA_TX_ALIGN) |
				(skb_list->size & LTQ_DMA_SIZE_MASK);
		} else {
			/* other fragments of a packet can have OWN */
			desc->ctl = LTQ_DMA_OWN |
				LTQ_DMA_TX_OFFSET(skb_list->addr & XRX200_DMA_TX_ALIGN) |
				(skb_list->size & LTQ_DMA_SIZE_MASK);
		}

		/* the last fragment needs EOP */
		if (i == skb_shinfo(skb)->nr_frags)
			desc->ctl |= LTQ_DMA_EOP;
	}

	/* before changing ownership to HW, everything must be written to RAM */
	wmb();

	/* Start TX DMA, set OWN for the first fragment */
	ring->dma.desc_base[ring_head].ctl |= LTQ_DMA_OWN;

	/* stop the queue until there is enough space in both rings */
	if (unlikely((TX_BUFFS_AVAIL(ring->free, ring->dma.desc, ring->num) <= (MAX_SKB_FRAGS + 1)) ||
		(TX_BUFFS_AVAIL(txq->tail, txq->head, txq->num) <= (MAX_SKB_FRAGS + 1)))) {
		netif_tx_stop_queue(txq->nqueue);
	}

	skb_tx_timestamp(skb);

out:
	return ret;
}

//NOTICE irq events must be as low as possible (big overhead on slow CPU)
static irqreturn_t xrx200_tx_dma_irq(int irq, void *ptr)
{
	struct xrx200_tx_ring *ring = ptr;

	ltq_dma_disable_irq(&ring->dma);
	ltq_dma_ack_irq(&ring->dma);

	napi_schedule_irqoff(&ring->napi);
	return IRQ_HANDLED;
}

//NOTICE it would be nice to have IRQ events in rx as low as possible too, how?
static irqreturn_t xrx200_rx_dma_irq(int irq, void *ptr)
{
	struct xrx200_rx_ring *ring = ptr;

	ltq_dma_disable_irq(&ring->dma);
	ltq_dma_ack_irq(&ring->dma);

	napi_schedule_irqoff(&ring->napi);

	return IRQ_HANDLED;
}


static int xrx200_dma_init(struct xrx200_priv *priv)
{
	int i;
	int ret;
	char *irq_name;

	ltq_dma_init_port(DMA_PORT_ETOP);

	//TODO external definitions?
	priv->rx[0].dma.irq = XRX200_DMA_IRQ + XRX200_DMA_RX;
	priv->rx[0].dma.nr = XRX200_DMA_RX;

	//TODO into ltq_dma_channel?
	priv->rx[0].num = LTQ_DESC_NUM;

#if MAX_RX_RINGS > 1
	//TODO external definitions?
	priv->rx[1].dma.irq = XRX200_DMA_IRQ + XRX200_DMA_RX_2;
	priv->rx[1].dma.nr = XRX200_DMA_RX_2;

	//TODO into ltq_dma_channel?
	priv->rx[1].num = LTQ_DESC_NUM;
#endif

	for (i = 0; i < MAX_RX_RINGS; i++) {
		struct xrx200_rx_ring *ring = &priv->rx[i];
		int idx;

		ring->dma.dev = priv->dev;
		ring->priv = priv;

		spin_lock_init(&ring->ref_lock);
		refcount_set(&ring->refs, 0);

		ltq_dma_alloc_rx(&ring->dma);	//TODO add ring num

		//TODO into ltq_dma_channel?
		ring->skb_list = devm_kzalloc(priv->dev, ring->num *
					sizeof(struct xrx200_rx_skb), GFP_KERNEL);

		//TODO is null

		/* NOTICE this will be incremented in xrx200_alloc_skb */
		ring->dma.desc = 0;

		for (idx = 0; idx < ring->num; idx++) {
			ret = xrx200_alloc_skb(ring, &ring->dma,
					       &ring->skb_list[idx]);
			if (ret)
				#warning "TODO ERROR PATHS"
				goto rx_free;
		}

		/* NOTICE reset "head" after xrx200_alloc_skb */
		ring->dma.desc = 0;

		irq_name = devm_kasprintf(priv->dev, GFP_KERNEL, "xrx200-net rx%d", i);
		//TODO null

		ret = devm_request_irq(priv->dev, ring->dma.irq, xrx200_rx_dma_irq, 0,
				       irq_name, ring);
		if (ret) {
			dev_err(priv->dev, "failed to request RX irq %d\n",
				ring->dma.irq);
			goto rx_ring_free;
		}

		pr_info("RING initilized\n");
	}



	//TODO TX rings vs cpuid nr_cpu_ids

	//TODO this is HACK, devicetree? or at least array
	priv->tx[0].dma.irq = XRX200_DMA_IRQ + XRX200_DMA_TX;
	priv->tx[0].dma.nr = XRX200_DMA_TX;
	priv->tx[0].num = LTQ_DESC_NUM;
	priv->tx[1].dma.irq = XRX200_DMA_IRQ + XRX200_DMA_TX_2;
	priv->tx[1].dma.nr = XRX200_DMA_TX_2;
	priv->tx[1].num = LTQ_DESC_NUM;


	for (i = 0; i < MAX_TX_RINGS; i++) {
		struct xrx200_tx_ring *ring = &priv->tx[i];
		ring->dma.dev = priv->dev;

		ring->priv = priv;

		spin_lock_init(&ring->lock);
		spin_lock_init(&ring->ref_lock);
		refcount_set(&ring->refs, 0);

		ltq_dma_alloc_tx(&ring->dma);

		ring->free = 0;


		INIT_LIST_HEAD(&ring->queues);

		/* array of pointers */
		ring->skb_list_ptr = devm_kzalloc(priv->dev, ring->num *
			sizeof(struct xrx200_tx_skb *), GFP_KERNEL);
		//TODO err path

		irq_name = devm_kasprintf(priv->dev, GFP_KERNEL, "xrx200-net tx%d", i);

		//TODO err path

		ret = devm_request_irq(priv->dev, ring->dma.irq, xrx200_tx_dma_irq, 0,
				       irq_name, ring);

		if (ret) {
			dev_err(priv->dev, "failed to request TX irq %d\n",
				ring->dma.irq);

			for (; i >= 0; i--) {
				ltq_dma_free(&ring->dma);
			}

			goto rx_ring_free;
		}
	}

	return ret;

rx_ring_free:


rx_free:
return ret;
//TODO redo
#if 0
	/* free the allocated RX ring */
	for (i = 0; i < LTQ_DESC_NUM; i++) {
		if (rxq->dma_skb[i].skb)
			dev_kfree_skb_any(rxq->dma_skb[i].skb);
	}

rx_free:
	ltq_dma_free(&rxq->dma);
	return ret;
#endif
}

#ifdef SW_POLLING
static void xrx200_gmac_update(struct xrx200_port *port)
{
	u16 phyaddr = port->phydev->mdio.addr & MDIO_PHY_ADDR_MASK;
	u16 miimode = ltq_mii_r32(MII_CFG(port->num)) & MII_CFG_MODE_MASK;
	u16 miirate = 0;

	switch (port->phydev->speed) {
	case SPEED_1000:
		phyaddr |= MDIO_PHY_SPEED_G1;
		miirate = MII_CFG_RATE_M125;
		break;

	case SPEED_100:
		phyaddr |= MDIO_PHY_SPEED_M100;
		switch (miimode) {
		case MII_CFG_MODE_RMIIM:
		case MII_CFG_MODE_RMIIP:
			miirate = MII_CFG_RATE_M50;
			break;
		default:
			miirate = MII_CFG_RATE_M25;
			break;
		}
		break;

	default:
		phyaddr |= MDIO_PHY_SPEED_M10;
		miirate = MII_CFG_RATE_M2P5;
		break;
	}

	if (port->phydev->link)
		phyaddr |= MDIO_PHY_LINK_UP;
	else
		phyaddr |= MDIO_PHY_LINK_DOWN;

	if (port->phydev->duplex == DUPLEX_FULL)
		phyaddr |= MDIO_PHY_FDUP_EN;
	else
		phyaddr |= MDIO_PHY_FDUP_DIS;

	ltq_mdio_w32_mask(MDIO_UPDATE_MASK, phyaddr, MDIO_PHY(port->num));
	ltq_mii_w32_mask(MII_CFG_RATE_MASK, miirate, MII_CFG(port->num));
	udelay(1);
}
#else
static void xrx200_gmac_update(struct xrx200_port *port)
{

}
#endif

static void xrx200_mdio_link(struct net_device *dev)
{
	struct xrx200_iface *iface = netdev_priv(dev);
	struct list_head *pos;

	list_for_each(pos, &iface->ports) {
		struct xrx200_port *port = list_entry(pos, struct xrx200_port,
						      port);

		if (!port->phydev)
			continue;

		if (port->link != port->phydev->link) {
			xrx200_gmac_update(port);
			port->link = port->phydev->link;
			netdev_info(dev, "port %d %s link\n",
				    port->num,
				    (port->link)?("got"):("lost"));
		}
	}
}

static inline int xrx200_mdio_poll(struct mii_bus *bus)
{
	unsigned cnt = 10000;

	while (likely(cnt--)) {
		unsigned ctrl = ltq_mdio_r32(MDIO_CTRL);
		if ((ctrl & MDIO_BUSY) == 0)
			return 0;
	}

	return 1;
}

static int xrx200_mdio_wr(struct mii_bus *bus, int addr, int reg, u16 val)
{
	if (xrx200_mdio_poll(bus))
		return 1;

	ltq_mdio_w32(val, MDIO_WRITE);
	ltq_mdio_w32(MDIO_BUSY | MDIO_WR |
		((addr & MDIO_MASK) << MDIO_ADDRSHIFT) |
		(reg & MDIO_MASK),
		MDIO_CTRL);

	return 0;
}

static int xrx200_mdio_rd(struct mii_bus *bus, int addr, int reg)
{
	if (xrx200_mdio_poll(bus))
		return -1;

	ltq_mdio_w32(MDIO_BUSY | MDIO_RD |
		((addr & MDIO_MASK) << MDIO_ADDRSHIFT) |
		(reg & MDIO_MASK),
		MDIO_CTRL);

	if (xrx200_mdio_poll(bus))
		return -1;

	return ltq_mdio_r32(MDIO_READ);
}

static int xrx200_phy_has_link(struct net_device *dev)
{
 	struct xrx200_iface *iface = netdev_priv(dev);
	struct list_head *pos;

	list_for_each(pos, &iface->ports) {
		struct xrx200_port *port = list_entry(pos, struct xrx200_port,
						      port);
		if (!port->phydev)
			continue;

		if (port->phydev->link)
			return 1;
	}

	return 0;
}

static void xrx200_phy_link_change(struct phy_device *phydev, bool up, bool do_carrier)
{
	struct net_device *netdev = phydev->attached_dev;

	if (do_carrier) {
		if (up)
			netif_carrier_on(netdev);
		else if (!xrx200_phy_has_link(netdev))
			netif_carrier_off(netdev);
	}

	phydev->adjust_link(netdev);
}

static int xrx200_mdio_probe(struct net_device *dev, struct xrx200_port *port)
{
	struct xrx200_iface *iface = netdev_priv(dev);
	struct xrx200_priv *priv = iface->priv;
	struct phy_device *phydev = NULL;
#ifdef SW_POLLING
	unsigned val;
#endif

	phydev = mdiobus_get_phy(priv->mii_bus, port->phy_addr);

	if (!phydev) {
		netdev_err(dev, "no PHY found\n");
		return -ENODEV;
	}

	phydev = phy_connect(dev, phydev_name(phydev), &xrx200_mdio_link,
				port->phy_if);

	if (IS_ERR(phydev)) {
		netdev_err(dev, "Could not attach to PHY\n");
		return PTR_ERR(phydev);
	}

	linkmode_zero(phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_TP_BIT, phydev->supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_MII_BIT, phydev->supported);
	linkmode_set_bit_array(phy_10_100_features_array,
			       ARRAY_SIZE(phy_10_100_features_array),
			       phydev->supported);
	linkmode_set_bit_array(phy_gbit_features_array,
			       ARRAY_SIZE(phy_gbit_features_array),
			       phydev->supported);
	linkmode_copy(phydev->advertising, phydev->supported);

	port->phydev = phydev;
	phydev->phy_link_change = xrx200_phy_link_change;

	phy_attached_info(phydev);

#ifdef SW_POLLING
	phy_read_status(phydev);

	val = xrx200_mdio_rd(priv->mii_bus, MDIO_DEVAD_NONE, MII_CTRL1000);
	val |= ADVERTIZE_MPD;
	xrx200_mdio_wr(priv->mii_bus, MDIO_DEVAD_NONE, MII_CTRL1000, val);
	xrx200_mdio_wr(priv->mii_bus, 0, 0, 0x1040);

	phy_start_aneg(phydev);
#endif
	return 0;
}

static void xrx200_port_config(struct xrx200_priv *priv,
		const struct xrx200_port *port)
{
	u16 miimode = 0;

	switch (port->num) {
	case 0: /* xMII0 */
	case 1: /* xMII1 */
		switch (port->phy_if) {
		case PHY_INTERFACE_MODE_MII:
			if (port->flags & XRX200_PORT_TYPE_PHY)
				/* MII MAC mode, connected to external PHY */
				miimode = MII_CFG_MODE_MIIM;
			else
				/* MII PHY mode, connected to external MAC */
				miimode = MII_CFG_MODE_MIIP;
			break;
		case PHY_INTERFACE_MODE_RMII:
			if (port->flags & XRX200_PORT_TYPE_PHY)
				/* RMII MAC mode, connected to external PHY */
				miimode = MII_CFG_MODE_RMIIM;
			else
				/* RMII PHY mode, connected to external MAC */
				miimode = MII_CFG_MODE_RMIIP;
			break;
		case PHY_INTERFACE_MODE_RGMII:
			/* RGMII MAC mode, connected to external PHY */
			miimode = MII_CFG_MODE_RGMII;
			break;
		default:
			break;
		}
		break;
	case 2: /* internal GPHY0 */
	case 3: /* internal GPHY0 */
	case 4: /* internal GPHY1 */
		switch (port->phy_if) {
			case PHY_INTERFACE_MODE_MII:
			case PHY_INTERFACE_MODE_GMII:
				/* MII MAC mode, connected to internal GPHY */
				miimode = MII_CFG_MODE_MIIM;
				break;
			default:
				break;
		}
		break;
	case 5: /* internal GPHY1 or xMII2 */
		switch (port->phy_if) {
		case PHY_INTERFACE_MODE_MII:
			/* MII MAC mode, connected to internal GPHY */
			miimode = MII_CFG_MODE_MIIM;
			break;
		case PHY_INTERFACE_MODE_RGMII:
			/* RGMII MAC mode, connected to external PHY */
			miimode = MII_CFG_MODE_RGMII;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	ltq_mii_w32_mask(MII_CFG_MODE_MASK, miimode | MII_CFG_EN,
		MII_CFG(port->num));
}

static int xrx200_init(struct net_device *dev)
{
	struct xrx200_iface *iface = netdev_priv(dev);
	struct xrx200_priv *priv = iface->priv;
	struct xrx200_port *port;
	struct sockaddr mac;
	int err;
	struct list_head *pos;

	/* setup each port */
	list_for_each(pos, &iface->ports) {
		port = list_entry(pos, struct xrx200_port, port);

		xrx200_port_config(priv, port);
	}

	memcpy(&mac.sa_data, iface->mac, ETH_ALEN);
	if (!is_valid_ether_addr(mac.sa_data)) {
		netdev_warn(dev, "net-xrx200: invalid MAC, using random\n");
		eth_random_addr(mac.sa_data);
		dev->addr_assign_type |= NET_ADDR_RANDOM;
	}

	err = eth_mac_addr(dev, &mac);
	if (err)
		goto err_netdev;

	list_for_each(pos, &iface->ports) {
		port = list_entry(pos, struct xrx200_port, port);

		if (xrx200_mdio_probe(dev, port))
			netdev_warn(dev, "xrx200-mdio: probing phy of port %d failed\n",
					 port->num);
			//TODO error path
	}

	return 0;

err_netdev:
	unregister_netdev(dev);
	//free_netdev(dev);	//devm?? TODO
	return err;
}

static void xrx200_pci_microcode(void)
{
	int i;

	ltq_switch_w32_mask(PCE_TBL_CFG_ADDR_MASK | PCE_TBL_CFG_ADWR_MASK,
		PCE_TBL_CFG_ADWR, PCE_TBL_CTRL);
	ltq_switch_w32(0, PCE_TBL_MASK);

	for (i = 0; i < ARRAY_SIZE(pce_microcode); i++) {
		ltq_switch_w32(i, PCE_TBL_ADDR);
		ltq_switch_w32(pce_microcode[i].val[3], PCE_TBL_VAL(0));
		ltq_switch_w32(pce_microcode[i].val[2], PCE_TBL_VAL(1));
		ltq_switch_w32(pce_microcode[i].val[1], PCE_TBL_VAL(2));
		ltq_switch_w32(pce_microcode[i].val[0], PCE_TBL_VAL(3));

		// start the table access:
		ltq_switch_w32_mask(0, PCE_TBL_BUSY, PCE_TBL_CTRL);
		while (ltq_switch_r32(PCE_TBL_CTRL) & PCE_TBL_BUSY);
	}

	/* tell the switch that the microcode is loaded */
	ltq_switch_w32_mask(0, BIT(3), PCE_GCTRL_REG(0));
}

static void xrx200_hw_init(struct xrx200_priv *priv)
{
	int i;

	/* enable clock gate */
	clk_enable(priv->clk);

	ltq_switch_w32(1, 0);
	mdelay(100);
	ltq_switch_w32(0, 0);

	/*
	 * TODO: we should really disable all phys/miis here and explicitly
	 * enable them in the device specific init function
	 */

	/* disable port fetch/store dma */
	for (i = 0; i < 7; i++ ) {
		ltq_switch_w32(0, FDMA_PCTRLx(i));
		ltq_switch_w32(0, SDMA_PCTRLx(i));
	}

	/* enable Switch */
	ltq_mdio_w32_mask(0, MDIO_GLOB_ENABLE, MDIO_GLOB);

	/* load the pce microcode */
	xrx200_pci_microcode();

	/* Default unknown Broadcat/Multicast/Unicast port maps */
// 	ltq_switch_w32(0x40, PCE_PMAP1);
// 	ltq_switch_w32(0x40, PCE_PMAP2);
// 	ltq_switch_w32(0x40, PCE_PMAP3);
	ltq_switch_w32(0x7f, PCE_PMAP1);
	ltq_switch_w32(0x7f, PCE_PMAP2);
	ltq_switch_w32(0x7f, PCE_PMAP3);

//TODO search XRX200_BM_GCTRL_FR_RBC

	/* RMON Counter Enable for all physical ports */
//	for (i = 0; i < 7; i++)
//		ltq_switch_w32(0x1, BM_PCFG(i));

	/* disable auto polling */
	ltq_mdio_w32(0x0, MDIO_CLK_CFG0);

	/* enable port statistic counters */
//	for (i = 0; i < 7; i++)
//		ltq_switch_w32(0x1, BM_PCFGx(i));

	/* set IPG to 12 */
	ltq_pmac_w32_mask(PMAC_IPG_MASK, 0xb, PMAC_RX_IPG);

#ifdef SW_ROUTING
	/* enable status header, enable CRC */
	ltq_pmac_w32_mask(0,
		PMAC_HD_CTL_RST | PMAC_HD_CTL_AST | PMAC_HD_CTL_RXSH | PMAC_HD_CTL_AS | PMAC_HD_CTL_AC | PMAC_HD_CTL_RC,
		PMAC_HD_CTL);
#else
	/* disable status header, enable CRC */
	ltq_pmac_w32_mask(PMAC_HD_CTL_AST | PMAC_HD_CTL_RXSH | PMAC_HD_CTL_AS,
		PMAC_HD_CTL_AC | PMAC_HD_CTL_RC,
		PMAC_HD_CTL);
#endif

	/* enable port fetch/store dma & VLAN Modification */
	for (i = 0; i < 7; i++ ) {
		ltq_switch_w32_mask(0, 0x19, FDMA_PCTRLx(i));
		ltq_switch_w32_mask(0, 0x01, SDMA_PCTRLx(i));
		ltq_switch_w32_mask(0, PCE_INGRESS, PCE_PCTRL_REG(i, 0));
	}

	/* enable special tag insertion on cpu port */
	ltq_switch_w32_mask(0, 0x02, FDMA_PCTRLx(6));
	ltq_switch_w32_mask(0, PCE_INGRESS, PCE_PCTRL_REG(6, 0));
	ltq_switch_w32_mask(0, BIT(3), MAC_CTRL_REG(6, 2));
	ltq_switch_w32(1518 + 8 + 4 * 2, MAC_FLEN_REG);
	xrx200sw_write_x(1, XRX200_BM_QUEUE_GCTRL_GL_MOD, 0);

	for (i = 0; i < XRX200_MAX_VLAN; i++)
		priv->vlan_vid[i] = i;
}

static void xrx200_hw_cleanup(struct xrx200_priv *priv)
{
	int i, idx;

	/* disable the switch */
	ltq_mdio_w32_mask(MDIO_GLOB_ENABLE, 0, MDIO_GLOB);


	for (i = 0; i < MAX_TX_RINGS; i++) {
		struct xrx200_tx_ring *ring = &priv->tx[i];

		ltq_dma_free(&ring->dma);

//TODO cleanup path
//		ring->dma.desc_base = NULL;
	}

	for (i = 0; i < MAX_RX_RINGS; i++) {
		struct xrx200_rx_ring *ring = &priv->rx[i];

		ltq_dma_free(&ring->dma);

		/* free the allocated RX ring */
		for (idx = 0; idx < ring->num; idx++) {
			if (ring->skb_list[idx].skb)
				dev_kfree_skb_any(ring->skb_list[idx].skb);

			ring->skb_list[idx].skb = NULL;
		}
	}

	/* clear the mdio bus */
	mdiobus_unregister(priv->mii_bus);
	mdiobus_free(priv->mii_bus);

	/* release the clock */
	clk_disable(priv->clk);
	clk_put(priv->clk);
}

static int xrx200_of_mdio(struct xrx200_priv *priv, struct device_node *np)
{
	priv->mii_bus = mdiobus_alloc();
	if (!priv->mii_bus)
		return -ENOMEM;

	priv->mii_bus->read = xrx200_mdio_rd;
	priv->mii_bus->write = xrx200_mdio_wr;
	priv->mii_bus->name = "lantiq,xrx200-mdio";
	snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "%x", 0);

	if (of_mdiobus_register(priv->mii_bus, np)) {
		mdiobus_free(priv->mii_bus);
		return -ENXIO;
	}

	return 0;
}

static int xrx200_of_port(struct xrx200_priv *priv, struct xrx200_iface *iface, struct device_node *port)
{
	const __be32 *addr, *id = of_get_property(port, "reg", NULL);
	struct xrx200_port *p;

	if (!id)
		return -EINVAL;

	p = devm_kzalloc(priv->dev, sizeof(struct xrx200_port),
			     GFP_KERNEL);
	if (!p) {
		dev_err(priv->dev, "failed to allocate port structure\n");

		return -ENOMEM;
	}

	p->phy_node = of_parse_phandle(port, "phy-handle", 0);
	addr = of_get_property(p->phy_node, "reg", NULL);

	if (!addr) {
		dev_err(priv->dev, "property 'reg' is missing\n");

		return -EINVAL;
	}

	p->num = *id;
	p->phy_addr = *addr;
	p->phy_if = of_get_phy_mode(port);
	if (p->phy_addr > 0x10)
		p->flags = XRX200_PORT_TYPE_MAC;
	else
		p->flags = XRX200_PORT_TYPE_PHY;

	p->gpio = of_get_gpio_flags(port, 0, &p->gpio_flags);
	if (gpio_is_valid(p->gpio))
		if (!gpio_request(p->gpio, "phy-reset")) {
			gpio_direction_output(p->gpio,
				(p->gpio_flags & OF_GPIO_ACTIVE_LOW) ? (1) : (0));
			udelay(100);
			gpio_set_value(p->gpio, (p->gpio_flags & OF_GPIO_ACTIVE_LOW) ? (0) : (1));
		}

	/* is this port a wan port ? */
	if (iface->iftype & BIT(IF_TYPE_WAN)) {
pr_info("set port %i as WAN\n",p->num);
		priv->wan_map |= BIT(p->num);
	}

	priv->d_port_map |= BIT(p->num);

	/* store the port id in the hw struct so we can map ports -> devices */
	priv->port_map[p->num] = iface;

	list_add(&p->port, &iface->ports);

	return 0;
}

static void xrx200_get_stats64(struct net_device *dev,
			       struct rtnl_link_stats64 *storage)
{
	struct xrx200_iface *iface = netdev_priv(dev);
	unsigned int start;
	int i;

	//TODO are there HW registers?

	do {
		start = u64_stats_fetch_begin_irq(&iface->syncp);
		storage->tx_errors = iface->tx_errors;
	} while (u64_stats_fetch_retry_irq(&iface->syncp, start));

	for (i = 0; i < iface->num_tx_queues; i++) {
		do {
			start = u64_stats_fetch_begin_irq(&iface->txq[i].syncp);
			storage->tx_packets += iface->txq[i].tx_packets;
			storage->tx_bytes += iface->txq[i].tx_bytes;
			storage->tx_errors += iface->txq[i].tx_errors;
			storage->tx_dropped += iface->txq[i].tx_dropped;
		} while (u64_stats_fetch_retry_irq(&iface->txq[i].syncp, start));
	}

	do {
		start = u64_stats_fetch_begin_irq(&iface->rxq[0].syncp);
		storage->rx_packets = iface->rxq[0].rx_packets;
		storage->rx_bytes = iface->rxq[0].rx_bytes;
		storage->rx_dropped = iface->rxq[0].rx_dropped;
	} while (u64_stats_fetch_retry_irq(&iface->rxq[0].syncp, start));

}

//TODO this too?
// * int (*ndo_change_mtu)(struct net_device *dev, int new_mtu);
// *	Called when a user wants to change the Maximum Transfer Unit
// *	of a device.

u16 glqid=0;

static u16 xrx200_select_queue(struct net_device *dev, struct sk_buff *skb,
			    void *accel_priv, select_queue_fallback_t fallback)
{
	u16 qid;

	/*
	 * TODO?
	 * The SoC seems to be slowed down by tx housekeeping
	 * for the highest speed you need to schedule tx housekeeping
	 * interrupt to the other VPE
	 *
	 * The default netdev queue select causes TX speed drops as
	 * userspace is sometimes scheduled to the same VPE which is making
	 * housekeeping.
	 *
	 * The TX DMAs IRQ should be constrained to a single VPE as the
	 * cycling through them will cause 50% of time to have the housekeeping
	 * on the same VPE.
	 */

	//TODO cornercases: single queue, singlecore, constrained affinity


#if 0
	if (skb_rx_queue_recorded(skb))
		qid = skb_get_rx_queue(skb);
	else
		qid = fallback(dev, skb);
//#else
// 	qid = glqid?1:0;

// 	glqid = !glqid;
#endif

	//https://elixir.bootlin.com/linux/v5.1-rc5/source/kernel/irq/cpuhotplug.c#L39

	//HACK only two VPEs max
	if (smp_processor_id()) {
		qid = 0;
	} else {
		qid = 1;
	}

	return qid;
}


static const struct net_device_ops xrx200_netdev_ops = {
	.ndo_init		= xrx200_init,
	.ndo_open		= xrx200_open,
	.ndo_stop		= xrx200_close,
	.ndo_start_xmit		= xrx200_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_tx_timeout		= xrx200_tx_timeout,
	.ndo_get_stats64	= xrx200_get_stats64,
// 	.ndo_select_queue	= xrx200_select_queue,
};

static int xrx200_of_iface(struct xrx200_priv *priv, struct device_node *node, struct device *dev)
{
	struct device_node *port;
// 	const __be32 *wan;
	const u8 *mac;
	struct net_device *net_dev;
	struct xrx200_iface *iface;
	int ret;
	int i;

	/* get interface MII value */
// 	const __be32 *id = of_get_property(port, "reg", NULL);

	//TODO add queues + ring mapping user definition into devicetree

	//TODO hardcoded num queues
	//NOTICE allocated iface struct
	/* alloc the network device */
	net_dev = devm_alloc_etherdev_mqs(dev, sizeof(struct xrx200_iface),
					  MAX_TX_QUEUES, 1);

	if (!net_dev) {
		dev_err(priv->dev, "failed to allocate net device\n");

		return -ENOMEM;
	}

	//NOTICE iface struct allocated in etherdev mqs
	iface = netdev_priv(net_dev);

	//TODO iface is array after netdev, container_of?
	iface->net_dev = net_dev;
	iface->priv = priv;
	iface->num_tx_queues = 2;	//TODO devicetree?

	net_dev->netdev_ops = &xrx200_netdev_ops;	//TODO dvakrat
	SET_NETDEV_DEV(net_dev, priv->dev);
	net_dev->min_mtu = ETH_ZLEN;
	net_dev->max_mtu = XRX200_DMA_DATA_LEN;

	net_dev->features |= NETIF_F_SG ;
	net_dev->hw_features |= NETIF_F_SG;
	net_dev->vlan_features |= NETIF_F_SG;

	/* setup the network device */
	strcpy(net_dev->name, "eth%d");
	net_dev->netdev_ops = &xrx200_netdev_ops;
	net_dev->watchdog_timeo = XRX200_TX_TIMEOUT;
	net_dev->needed_headroom = XRX200_HEADROOM;

	mac = of_get_mac_address(node);
	if (!IS_ERR(mac))
		memcpy(iface->mac, mac, ETH_ALEN);

	/* should the switch be enabled on this interface ? */
	if (of_find_property(node, "lantiq,switch", NULL))
		iface->iftype |= BIT(IF_TYPE_SWITCH);

	/* is this the wan interface ? */
	if (of_find_property(node, "lantiq,wan", NULL)) {
pr_info("interface is WAN\n");
		iface->iftype |= BIT(IF_TYPE_WAN);
	}

	INIT_LIST_HEAD(&iface->ports);

	/* load the ports that are part of the interface */
	for_each_child_of_node(node, port)
		if (of_device_is_compatible(port, "lantiq,xrx200-pdi-port"))
			if (xrx200_of_port(priv, iface, port)) {
				return -EINVAL;
			}

	iface->txq = devm_kzalloc(priv->dev, iface->num_tx_queues *
				  sizeof(struct xrx200_tx_queue),
				  GFP_KERNEL);

	iface->rxq = devm_kzalloc(priv->dev, sizeof(struct xrx200_rx_queue),
				  GFP_KERNEL);

	for (i = 0; i < iface->num_tx_queues; i++) {
		struct xrx200_tx_queue *txq = &iface->txq[i];
		//TODO shorten by txq pointer

		txq->ring = &priv->tx[i % MAX_TX_RINGS];
		txq->nqueue = netdev_get_tx_queue(net_dev, i % MAX_TX_RINGS);

		/* local ring pointers */
		txq->head = 0;
		txq->tail = 0;

		//split from ring length
		txq->num = txq->ring->num / 1;
		//TODO divide by queue count per ring list_for_each txq->ring->queues

		/* array of descriptors */
		txq->skb_list = devm_kzalloc(priv->dev,
					     txq->num * sizeof(struct xrx200_tx_skb),
					     GFP_KERNEL);

		//queue can be more than cpus

		list_add(&txq->queue, &txq->ring->queues);
	}

	iface->rxq[0].priv = priv;

	if (iface->iftype & BIT(IF_TYPE_WAN)) {
#if MAX_RX_RINGS > 1
		iface->rxq[0].ring = &priv->rx[1];
#else
		iface->rxq[0].ring = &priv->rx[0];
#endif

	} else {
		iface->rxq[0].ring = &priv->rx[0];
	}

	ret = register_netdev(net_dev);
	if (ret)
		return ret;

	if (iface->iftype & BIT(IF_TYPE_SWITCH))
		xrx200sw_init(iface);

	list_add(&iface->iface, &priv->ifaces);

	return 0;
}

static int xrx200_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res[4];
	struct device_node *mdio_np, *iface_np, *phy_np;
	struct of_phandle_iterator it;
	int err;
	int i;
	struct xrx200_priv *priv;
#ifndef SW_POLLING
	struct list_head *pos;
	unsigned int reg = 0;
#endif

	priv = devm_kzalloc(dev, sizeof(struct xrx200_priv),
			 GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "failed to allocate priv structure\n");

		return -ENOMEM;
	}

	priv->dev = dev;

//dev_set_drvdata(dev, priv); ???

	/* load the memory ranges */
	for (i = 0; i < 4; i++) {
		res[i] = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res[i]) {
			dev_err(dev, "failed to get resources\n");
			return -ENOENT;
		}
	}

	xrx200_switch_membase = devm_ioremap_resource(dev, res[0]);
	xrx200_mdio_membase = devm_ioremap_resource(dev, res[1]);
	xrx200_mii_membase = devm_ioremap_resource(dev, res[2]);
	xrx200_pmac_membase = devm_ioremap_resource(dev, res[3]);

	if (!xrx200_switch_membase || !xrx200_mdio_membase ||
			!xrx200_mii_membase || !xrx200_pmac_membase) {
		dev_err(dev, "failed to request and remap io ranges \n");
		return -ENOMEM;
	}

	of_for_each_phandle(&it, err, dev->of_node, "lantiq,phys", NULL, 0) {
		phy_np = it.node;
		if (phy_np) {
			struct platform_device *phy = of_find_device_by_node(phy_np);

			of_node_put(phy_np);
			if (!platform_get_drvdata(phy))
				return -EPROBE_DEFER;
		}
	}

//TODO ? devm_clk_get
	/* get the clock */
	priv->clk = clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "failed to get clock\n");
		return PTR_ERR(priv->clk);
	}

	/* bring up the dma engine and IP core */
	err = xrx200_dma_init(priv);
	if (err)
		return err;

	/* enable clock gate */
	err = clk_prepare_enable(priv->clk);
	if (err)
		goto err_uninit_dma;

	xrx200_hw_init(priv);


	/* global dummy netdev for napi, for all DMA rings */
	init_dummy_netdev(&priv->dummy_net);

	/* setup NAPI */
	for (i = 0; i < MAX_RX_RINGS; i++) {
		netif_napi_add(&priv->dummy_net, &priv->rx[i].napi,
			       xrx200_poll_rx, 32);	//32 TODO value by number of queues?
	}

	for (i = 0; i < MAX_TX_RINGS; i++) {
		netif_tx_napi_add(&priv->dummy_net, &priv->tx[i].napi,
				  xrx200_tx_housekeeping, 48);	//TODO number by queues?
	}

	/* bring up the mdio bus */
	mdio_np = of_find_compatible_node(dev->of_node, NULL,
				"lantiq,xrx200-mdio");
	if (mdio_np)
		if (xrx200_of_mdio(priv, mdio_np))
			dev_err(dev, "mdio probe failed\n");	//TODO fail path?

	INIT_LIST_HEAD(&priv->ifaces);

	/* load the interfaces */
	for_each_child_of_node(dev->of_node, iface_np)
			if (of_device_is_compatible(iface_np, "lantiq,xrx200-pdi")) {
				err = xrx200_of_iface(priv, iface_np, dev);

				if (err) {
					//printk?
					goto err_unprepare_clk;
				}
			}

pr_info("WAN map = %04x\n", priv->wan_map);

	/* set wan port mask */
	ltq_pmac_w32(priv->wan_map, PMAC_EWAN);

#ifndef SW_POLLING
//TODO this is sort of ugly, but original way seems to be weird
//double register fills (once for Sw and second for WAN into MDIO_CLK_CFG0)
//RFC if one can RMW MDIO_CLK_CFG0 after setting MDIO_CLK_CFG1, it could be split
//and put back to xrx200_init
//TODO maybe put before xrx200_of_iface
	reg = 0;

	list_for_each(pos, &priv->ifaces) {
		struct list_head *port_list;
		struct xrx200_iface *iface = list_entry(pos,
							struct xrx200_iface,
							iface);

		list_for_each(port_list, &iface->ports) {
			struct xrx200_port *port =
				list_entry(port_list,
					   struct xrx200_port,
					   port);

			reg |= BIT(port->num);
		}
	}

	ltq_mdio_w32(reg, MDIO_CLK_CFG0);
	ltq_mdio_w32(MDIO1_25MHZ, MDIO_CLK_CFG1);
#endif

#if 0
	if (priv->wan_map) {
		//disable VLAN
		//HACK needed for WAN mode
		ltq_switch_w32_mask(BIT(14), 0 , PCE_GCTRL_REG(0));
	}

pr_info("GCTRL = %08x",ltq_switch_r32(PCE_GCTRL_REG(0)));
#endif

	platform_set_drvdata(pdev, priv);

	return 0;

err_unprepare_clk:
//TODO split for fail inside xrx200_of_iface, unregister_netdevs from list
	clk_disable_unprepare(priv->clk);

err_uninit_dma:
//TODO rename to xrx200_dma_cleanup? maybe...
	xrx200_hw_cleanup(priv);

	return err;
}

static int xrx200_remove(struct platform_device *pdev)
{
	int i;
	struct xrx200_priv *priv = platform_get_drvdata(pdev);
	struct xrx200_iface *iface;
	struct list_head *pos;

	/* free stack related instances */
	list_for_each(pos, &priv->ifaces) {
		iface = list_entry(pos, struct xrx200_iface, iface);

		netif_tx_stop_all_queues(iface->net_dev);
	}

	for (i = 0; i < MAX_TX_RINGS; i++) {
		netif_napi_del(&priv->tx[i].napi);
	}

	for (i = 0; i < MAX_RX_RINGS; i++) {
		netif_napi_del(&priv->rx[i].napi);
	}

	/* remove the actual device */
	list_for_each(pos, &priv->ifaces) {
		iface = list_entry(pos, struct xrx200_iface, iface);

		unregister_netdev(iface->net_dev);
	}

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
		.owner = THIS_MODULE,
	},
};

module_platform_driver(xrx200_driver);

MODULE_AUTHOR("John Crispin <blogic@openwrt.org>");
MODULE_DESCRIPTION("Lantiq SoC XRX200 ethernet");
MODULE_LICENSE("GPL");
