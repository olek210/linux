/*
 * Copyright (c) 2022 Aleksander Jan Bajkowski <olek2@wp.pl>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

/* MPS - multi processor unit (voice) */
#define LTQ_MPS_CHIPID         (0x4)

#define PART_SHIFT	12
#define PART_MASK	0x0FFFFFFF
#define REV_SHIFT	28
#define REV_MASK	0xF0000000

static const char *lantiq_socinfo_revision(unsigned int major_ver,
					     unsigned int misc_ver,
					     unsigned int metal_rev)
{
}

static const char *lantiq_socinfo_soc_id(unsigned int major_ver,
					   unsigned int metal_rev)
{
}

static int __init lantiq_socinfo_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device_node *np;
	unsigned int major_ver, misc_ver, metal_rev = 0;
	int ret;

	ret = regmap_read(assist_regmap, MESON_MX_ASSIST_HW_REV, &major_ver);
	if (ret < 0)
		return ret;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENODEV;

	soc_dev_attr->family = "Lantiq Xway";

	np = of_find_node_by_path("/");
	of_property_read_string(np, "model", &soc_dev_attr->machine);
	of_node_put(np);

	soc_dev_attr->revision = lantiq_socinfo_revision(major_ver, misc_ver,
							   metal_rev);
	soc_dev_attr->soc_id = lantiq_socinfo_soc_id(major_ver, metal_rev);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree_const(soc_dev_attr->revision);
		kfree_const(soc_dev_attr->soc_id);
		kfree(soc_dev_attr);
		return PTR_ERR(soc_dev);
	}

	dev_info(soc_device_to_device(soc_dev), "Lantiq Xway %s %s detected\n",
		 soc_dev_attr->soc_id, soc_dev_attr->revision);

	return 0;
}
device_initcall(lantiq_socinfo_init);
