/*
 * Freescale QorIQ Platforms GUTS Driver
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/fsl/guts.h>

struct guts {
	struct ccsr_guts __iomem *regs;
	bool little_endian;
};

static struct guts *guts;
static DEFINE_MUTEX(guts_lock);

u32 fsl_guts_get_svr(void)
{
	u32 svr = 0;

	if (!guts || !guts->regs) {
#ifdef CONFIG_PPC
		svr =  mfspr(SPRN_SVR);
#endif
		return svr;
	}

	if (guts->little_endian)
		svr = ioread32(&guts->regs->svr);
	else
		svr = ioread32be(&guts->regs->svr);

	return svr;
}
EXPORT_SYMBOL(fsl_guts_get_svr);

/*
 * Table for matching compatible strings, for device tree
 * guts node, for Freescale QorIQ SOCs.
 */
static const struct of_device_id guts_of_match[] = {
	{ .compatible = "fsl,qoriq-device-config-1.0", },
	{ .compatible = "fsl,qoriq-device-config-2.0", },
	{ .compatible = "fsl,p1010-guts", },
	{ .compatible = "fsl,p1020-guts", },
	{ .compatible = "fsl,p1021-guts", },
	{ .compatible = "fsl,p1022-guts", },
	{ .compatible = "fsl,p1023-guts", },
	{ .compatible = "fsl,p2020-guts", },
	{ .compatible = "fsl,bsc9131-guts", },
	{ .compatible = "fsl,bsc9132-guts", },
	{ .compatible = "fsl,mpc8536-guts", },
	{ .compatible = "fsl,mpc8544-guts", },
	{ .compatible = "fsl,mpc8548-guts", },
	{ .compatible = "fsl,mpc8568-guts", },
	{ .compatible = "fsl,mpc8569-guts", },
	{ .compatible = "fsl,mpc8572-guts", },
	{ .compatible = "fsl,ls1021a-dcfg", },
	{ .compatible = "fsl,ls1043a-dcfg", },
	{ .compatible = "fsl,ls2080a-dcfg", },
	{}
};

int fsl_guts_init(void)
{
	struct device_node *np;
	int ret;

	mutex_lock(&guts_lock);
	/* Initialize guts only once */
	if (guts) {
		ret = guts->regs ? 0 : -ENOMEM;
		goto out;
	}

	np = of_find_matching_node(NULL, guts_of_match);
	if (!np) {
		ret = -ENODEV;
		goto out;
	}

	guts = kzalloc(sizeof(*guts), GFP_KERNEL);
	if (!guts) {
		ret = -ENOMEM;
		goto out_np;
	}

	guts->little_endian = of_property_read_bool(np, "little-endian");

	guts->regs = of_iomap(np, 0);
	if (!guts->regs) {
		ret = -ENOMEM;
		goto out_np;
	}

	ret = 0;
out_np:
	of_node_put(np);
out:
	mutex_unlock(&guts_lock);
	return ret;
}
EXPORT_SYMBOL(fsl_guts_init);
