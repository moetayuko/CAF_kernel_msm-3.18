/* linux/arch/arm/mach-msm/board-swordfish.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/irqs.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>

#include "devices.h"

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x70000300,
		.end	= 0x70000400,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(156),
		.end	= MSM_GPIO_TO_INT(156),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#ifdef CONFIG_USB_FUNCTION
static char *swordfish_usb_functions[] = {
#ifdef CONFIG_USB_FUNCTION_ADB
	"adb",
#endif
};

static struct msm_hsusb_product swordfish_usb_products[] = {
	{
		.product_id     = 0x0002,
		.functions      = 0x00000001, /* "adb" only */
	},
};
#endif

static int swordfish_phy_init_seq[] = { 0x1D, 0x0D, 0x1D, 0x10, -1 };

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
//	.phy_reset		= trout_phy_reset,
	.phy_init_seq		= swordfish_phy_init_seq,
#ifdef CONFIG_USB_FUNCTION
	.vendor_id		= 0x18d1,
	.product_id		= 0x0002,
	.version		= 0x0100,
	.product_name		= "Swordfish",
	.serial_number		= "42",
	.manufacturer_name	= "Qualcomm",

	.functions = swordfish_usb_functions,
	.num_functions = ARRAY_SIZE(swordfish_usb_functions),
	.products  = swordfish_usb_products,
	.num_products = ARRAY_SIZE(swordfish_usb_products),
#endif
};

static struct platform_device *devices[] __initdata = {
	&msm_device_uart3,
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_hsusb,
	&smc91x_device,
};

extern struct sys_timer msm_timer;

static struct msm_acpu_clock_platform_data swordfish_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 128000000,
	.wait_for_irq_khz = 128000000,
};

static void __init swordfish_init(void)
{
	msm_acpu_clock_init(&swordfish_clock_data);
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_hsusb_set_vbus_state(1);
}

static void __init swordfish_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = (101*1024*1024);
}

static void __init swordfish_map_io(void)
{
	msm_map_common_io();
	msm_clock_init();
}

MACHINE_START(SWORDFISH, "Swordfish Board (QCT SURF8250)")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x10000100,
	.fixup		= swordfish_fixup,
	.map_io		= swordfish_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= swordfish_init,
	.timer		= &msm_timer,
MACHINE_END
