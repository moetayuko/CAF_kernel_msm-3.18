/*
 * Versatile board support using the device tree
 *
 *  Copyright (C) 2010 Secret Lab Technologies Ltd.
 *  Copyright (C) 2009 Jeremy Kerr <jeremy.kerr@canonical.com>
 *  Copyright (C) 2004 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "core.h"

#define VERSATILE_SYS_PCICTL_OFFSET           0x44

static void __init versatile_dt_pci_init(void)
{
	u32 val;
	void __iomem *base;
	struct device_node *np;
	struct property *newprop;

	np = of_find_compatible_node(NULL, NULL, "arm,core-module-versatile");
	if (!np)
		return;

	base = of_iomap(np, 0);
	if (!base)
		return;

	/* Check if PCI backplane is detected */
	val = __raw_readl(base + VERSATILE_SYS_PCICTL_OFFSET);
	if (val & 1)
		goto err;

	np = of_find_compatible_node(NULL, NULL, "arm,versatile-pci");
	if (!np)
		goto err;

	newprop = kzalloc(sizeof(*newprop), GFP_KERNEL);
	if (!newprop)
		goto err;

	newprop->name = kstrdup("status", GFP_KERNEL);
	newprop->value = kstrdup("disabled", GFP_KERNEL);
	newprop->length = sizeof("disabled");
	of_update_property(np, newprop);

	pr_info("Not plugged into PCI backplane!\n");
err:
	iounmap(base);
}

static void __init versatile_dt_init(void)
{
	versatile_dt_pci_init();

	platform_device_register(&versatile_flash_device);
	of_platform_populate(NULL, of_default_bus_match_table,
			     versatile_auxdata_lookup, NULL);
}

static const char *versatile_dt_match[] __initconst = {
	"arm,versatile-ab",
	"arm,versatile-pb",
	NULL,
};

DT_MACHINE_START(VERSATILE_PB, "ARM-Versatile (Device Tree Support)")
	.map_io		= versatile_map_io,
	.init_early	= versatile_init_early,
	.init_machine	= versatile_dt_init,
	.dt_compat	= versatile_dt_match,
	.restart	= versatile_restart,
MACHINE_END
