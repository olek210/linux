// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 *  Copyright (C) 2010 John Crispin <john@phrozen.org>
 *  Copyright (C) 2013-2015 Lantiq Beteiligungs-GmbH & Co.KG
 */

#include <linux/export.h>
#include <linux/clk.h>
#include <asm/bootinfo.h>
#include <asm/time.h>

#include <lantiq_soc.h>

#include "../prom.h"

#define SOC_DANUBE	"Danube"
#define SOC_TWINPASS	"Twinpass"
#define SOC_AMAZON_SE	"Amazon_SE"
#define SOC_ARX100	"ARX100"
#define SOC_GRX200	"GRX200"
#define SOC_XRX200	"xRX200"
#define SOC_VRX220	"xRX220"
#define SOC_XRX300	"xRX300"
#define SOC_XRX330	"xRX330"

#define COMP_DANUBE	"lantiq,danube"
#define COMP_TWINPASS	"lantiq,twinpass"
#define COMP_AMAZON_SE	"lantiq,ase"
#define COMP_ARX100	"lantiq,arx100"
#define COMP_GRX200	"lantiq,grx200"
#define COMP_XRX200	"lantiq,xrx200"
#define COMP_XRX300	"lantiq,xrx300"
#define COMP_XRX330	"lantiq,xrx330"

#define PART_SHIFT	12
#define PART_MASK	0x0FFFFFFF
#define REV_SHIFT	28
#define REV_MASK	0xF0000000

void __init ltq_soc_detect(struct ltq_soc_info *i)
{
	i->partnum = (ltq_r32(LTQ_MPS_CHIPID) & PART_MASK) >> PART_SHIFT;
	i->rev = (ltq_r32(LTQ_MPS_CHIPID) & REV_MASK) >> REV_SHIFT;
	sprintf(i->rev_type, "1.%d", i->rev);
	switch (i->partnum) {
	case SOC_ID_DANUBE1:
	case SOC_ID_DANUBE2:
		i->name = SOC_DANUBE;
		i->type = SOC_TYPE_DANUBE;
		i->compatible = COMP_DANUBE;
		break;

	case SOC_ID_TWINPASS:
		i->name = SOC_TWINPASS;
		i->type = SOC_TYPE_DANUBE;
		i->compatible = COMP_TWINPASS;
		break;

	case SOC_ID_ARX188:
	case SOC_ID_ARX168_1:
	case SOC_ID_ARX168_2:
	case SOC_ID_ARX182:
		i->name = SOC_ARX100;
		i->type = SOC_TYPE_ARX100;
		i->compatible = COMP_ARX100;
		break;

	case SOC_ID_GRX188:
	case SOC_ID_GRX168:
		i->name = SOC_GRX200;
		i->type = SOC_TYPE_ARX100;
		i->compatible = COMP_GRX200;
		break;

	case SOC_ID_AMAZON_SE_1:
	case SOC_ID_AMAZON_SE_2:
#ifdef CONFIG_PCI
		panic("ase is only supported for non pci kernels");
#endif
		i->name = SOC_AMAZON_SE;
		i->type = SOC_TYPE_AMAZON_SE;
		i->compatible = COMP_AMAZON_SE;
		break;

	case SOC_ID_VRX282:
	case SOC_ID_VRX268:
	case SOC_ID_VRX288:
		i->name = SOC_XRX200;
		i->type = SOC_TYPE_XRX200;
		i->compatible = COMP_XRX200;
		break;

	case SOC_ID_GRX268:
	case SOC_ID_GRX288:
		i->name = SOC_GRX200;
		i->type = SOC_TYPE_XRX200;
		i->compatible = COMP_GRX200;
		break;

	case SOC_ID_VRX268_2:
	case SOC_ID_VRX288_2:
		i->name = SOC_XRX200;
		i->type = SOC_TYPE_XRX200_2;
		i->compatible = COMP_XRX200;
		break;

	case SOC_ID_VRX220:
		i->name = SOC_VRX220;
		i->type = SOC_TYPE_VRX220;
		i->compatible = COMP_XRX200;
		break;

	case SOC_ID_GRX282_2:
	case SOC_ID_GRX288_2:
		i->name = SOC_GRX200;
		i->type = SOC_TYPE_XRX200_2;
		i->compatible = COMP_GRX200;
		break;

	case SOC_ID_ARX362:
	case SOC_ID_ARX368:
	case SOC_ID_ARX382:
	case SOC_ID_ARX388:
	case SOC_ID_URX388:
		i->name = SOC_XRX300;
		i->type = SOC_TYPE_XRX300;
		i->compatible = COMP_XRX300;
		break;

	case SOC_ID_GRX383:
	case SOC_ID_GRX369:
	case SOC_ID_GRX387:
	case SOC_ID_GRX389:
		i->name = SOC_XRX330;
		i->type = SOC_TYPE_XRX330;
		i->compatible = COMP_XRX330;
		break;

	default:
		unreachable();
		break;
	}
}
