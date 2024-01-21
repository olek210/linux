// SPDX-License-Identifier: GPL-2.0
/*
 * TODO: Description
 */

#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <net/dsa.h>

/* Global Control 0 (PHY0) */
#define RTL8306_GC0			16
#define  RTL8306_GCTRL0_PSELL_LO	BIT(15)
#define  RTL8306_GCTRL0_PSEL_HI		BIT(1) /* inverted */

struct rtl8306_hw_info {
	int max_ports;
	int cpu_port;
	const struct dsa_switch_ops *ops;
};

struct rtl8306_priv {
	const struct rtl8306_hw_info *hw_info;
	struct dsa_switch *ds;
	struct device *dev;
};

static void
rtl8306_set_page()
{
	/* TODO: Set page */
}

static enum dsa_tag_protocol rtl8306_get_tag_protocol(struct dsa_switch *ds,
						    int port,
						    enum dsa_tag_protocol mp)
{
	/* TODO: Implement tag protocol */
	return DSA_TAG_PROTO_NONE;
}

static int rtl8306_setup(struct dsa_switch *ds)
{

	/* TODO: Implement me */

	return 0;
}

static const struct dsa_switch_ops rtl8306_switch_ops = {
	.get_tag_protocol	= rtl8306_get_tag_protocol,
	.setup			= rtl8306_setup,
};

static int rtl8306_probe(struct platform_device *pdev)
{

	/* TODO: Implement me */

	return 0;
}

static void rtl8306_remove(struct platform_device *pdev)
{
	struct rtl8306_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return;

	/* disable the switch */
	// TODO: Disable switch

	dsa_unregister_switch(priv->ds);
}

static void rtl8306_shutdown(struct platform_device *pdev)
{
	struct rtl8306_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return;

	dsa_switch_shutdown(priv->ds);

	platform_set_drvdata(pdev, NULL);
}

static const struct rtl8306_hw_info rtl8306 = {
	.max_ports = 6,
	.cpu_port = 5,
	.ops = &rtl8306_switch_ops,
};

static const struct of_device_id rtl8306_of_match[] = {
	{ .compatible = "realtek,rtl8306", .data = &rtl8306 },
	{},
};
MODULE_DEVICE_TABLE(of, rtl8306_of_match);

static struct platform_driver rtl8306_driver = {
	.probe = rtl8306_probe,
	.remove_new = rtl8306_remove,
	.shutdown = rtl8306_shutdown,
	.driver = {
		.name = "rtl8306",
		.of_match_table = rtl8306_of_match,
	},
};

module_platform_driver(rtl8306_driver);

MODULE_AUTHOR("Aleksander Jan Bajkowski <olek2@wp.pl>");
MODULE_DESCRIPTION("Realtek RTL8306 driver");
MODULE_LICENSE("GPL v2");
