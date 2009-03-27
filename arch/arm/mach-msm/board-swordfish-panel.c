/* linux/arch/arm/mach-msm/board-swordfish-panel.c
 *
 * Copyright (c) 2009 Google Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Dima Zavin <dima@android.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>

#include "devices.h"

#define CLK_NS_TO_RATE(ns)			(1000000000UL / (ns))

int swordfish_panel_blank(void)
{
	/* TODO: Turn backlight off? */
	return 0;
}

int swordfish_panel_unblank(void)
{
	/* TODO: Turn backlight on? */
	return 0;
}

/* TODO: Move me to not be here */
#define MSM_FB_BASE		0x1FE00000
#define MSM_FB_SIZE		0x00200000
static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_lcdc_platform_data swordfish_lcdc_platform_data = {
	.blank		= swordfish_panel_blank,
	.unblank	= swordfish_panel_unblank,
	.panel_info	= {
		.clk_rate		= CLK_NS_TO_RATE(26),
		.hsync_pulse_width	= 60,
		.hsync_back_porch	= 81,
		.hsync_front_porch	= 81,
		.hsync_skew		= 0,
		.vsync_pulse_width	= 2,
		.vsync_back_porch	= 20,
		.vsync_front_porch	= 27,
	},
	.fb_data	= {
		.xres		= 800,
		.yres		= 480,
		.width		= 94,
		.height		= 57,
		.output_format	= 0,
	},
	.fb_resource = &resources_msm_fb[0],
};

static struct msm_mdp_platform_data swordfish_mdp_platform_data = {
	.lcdc_data = &swordfish_lcdc_platform_data,
};

int __init swordfish_init_panel(void)
{
	int rc;
	if (!machine_is_swordfish())
		return 0;

	msm_device_mdp.dev.platform_data = &swordfish_mdp_platform_data;
	if ((rc = platform_device_register(&msm_device_mdp)) != 0)
		return rc;
	return 0;
}

device_initcall(swordfish_init_panel);
