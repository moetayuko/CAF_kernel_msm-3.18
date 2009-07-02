/* linux/arch/arm/mach-msm/board-mahimahi.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
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

#include <linux/cy8c_tmg_ts.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/usb/mass_storage_function.h>
#include <linux/android_pmem.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_iomap.h>

#include "board-mahimahi.h"
#include "devices.h"
#include "proc_comm.h"

static char *mahimahi_usb_functions[] = {
	"usb_mass_storage",
	"adb",
};

static struct msm_hsusb_product mahimahi_usb_products[] = {
	{
		.product_id     = 0x0d01,
		.functions      = 0x00000001, /* "usb_mass_storage" only */
	},
	{
		.product_id     = 0x0d02,
		.functions      = 0x00000003, /* "usb_mass_storage" and "adb" */
	},
};

static int mahimahi_phy_init_seq[] = { 0x1D, 0x0D, 0x1D, 0x10, -1 };

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= mahimahi_phy_init_seq,
	.vendor_id		= 0x18d1,
	.product_id		= 0x0d02,
	.version		= 0x0100,
	.product_name		= "mahimahi",
	.serial_number		= "42",
	.manufacturer_name	= "Google",

	.functions		= mahimahi_usb_functions,
	.num_functions		= ARRAY_SIZE(mahimahi_usb_functions),
	.products		= mahimahi_usb_products,
	.num_products		= ARRAY_SIZE(mahimahi_usb_products),
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.buf_size	= 16384,
	.vendor		= "Google",
	.product	= "mahimahi",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct platform_device mahimahi_rfkill = {
	.name = "mahimahi_rfkill",
	.id = -1,
};

static struct resource msm_kgsl_resources[] = {
	{
		.name	= "kgsl_reg_memory",
		.start	= MSM_GPU_REG_PHYS,
		.end	= MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "kgsl_phys_memory",
		.start	= MSM_GPU_MEM_BASE,
		.end	= MSM_GPU_MEM_BASE + MSM_GPU_MEM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_kgsl_device = {
	.name		= "kgsl",
	.id		= -1,
	.resource	= msm_kgsl_resources,
	.num_resources	= ARRAY_SIZE(msm_kgsl_resources),
};

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name		= "pmem",
	.start		= MSM_PMEM_MDP_BASE,
	.size		= MSM_PMEM_MDP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_gpu0_pdata = {
	.name		= "pmem_gpu0",
	.start		= MSM_PMEM_GPU0_BASE,
	.size		= MSM_PMEM_GPU0_SIZE,
	.no_allocator	= 0,
	.cached		= 0,
};

static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name		= "pmem_gpu1",
	.start		= MSM_PMEM_GPU1_BASE,
	.size		= MSM_PMEM_GPU1_SIZE,
	.no_allocator	= 0,
	.cached		= 0,
};

static struct platform_device android_pmem_mdp_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &mdp_pmem_pdata
	},
};

static struct platform_device android_pmem_gpu0_device = {
	.name		= "android_pmem",
	.id		= 2,
	.dev		= {
		.platform_data = &android_pmem_gpu0_pdata,
	},
};

static struct platform_device android_pmem_gpu1_device = {
	.name		= "android_pmem",
	.id		= 3,
	.dev		= {
		.platform_data = &android_pmem_gpu1_pdata,
	},
};

static int mahimahi_cy8c_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		/* level shifter should be off */
		gpio_set_value(MAHIMAHI_GPIO_TP_EN, 1);
		msleep(120);
		/* enable touch panel level shift */
		gpio_set_value(MAHIMAHI_GPIO_TP_LS_EN, 1);
		msleep(3);
	} else {
		gpio_set_value(MAHIMAHI_GPIO_TP_LS_EN, 0);
		gpio_set_value(MAHIMAHI_GPIO_TP_EN, 0);
		udelay(50);
	}

	return 0;
}

struct cy8c_i2c_platform_data mahimahi_cy8c_ts_data = {
	.version = 0x0001,
	.abs_x_min = 0,
	.abs_x_max = 479,
	.abs_y_min = 0,
	.abs_y_max = 799,
	.abs_pressure_min = 0,
	.abs_pressure_max = 255,
	.abs_width_min = 0,
	.abs_width_max = 10,
	.power = mahimahi_cy8c_ts_power,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO("cy8c-tmg-ts", 0x34),
		.platform_data = &mahimahi_cy8c_ts_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_TP_INT_N),
	},
};


static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
	&mahimahi_rfkill,
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_hsusb,
	&usb_mass_storage_device,
	&android_pmem_mdp_device,
	&android_pmem_gpu0_device,
	&android_pmem_gpu1_device,
	&msm_kgsl_device,
	&msm_device_i2c,
};


static uint32_t bt_gpio_table[] = {
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(MAHIMAHI_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static struct msm_acpu_clock_platform_data mahimahi_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 128000000,
	.wait_for_irq_khz	= 128000000,
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

static void __init mahimahi_init(void)
{
	printk("mahimahi_init() revision=%d\n", system_rev);

	msm_acpu_clock_init(&mahimahi_clock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      &msm_device_uart1.dev, 1);
#endif

	config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));
	gpio_direction_output(MAHIMAHI_GPIO_TP_LS_EN, 0);
	gpio_direction_output(MAHIMAHI_GPIO_TP_EN, 0);

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	msm_hsusb_set_vbus_state(1);
}

static void __init mahimahi_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = (101*1024*1024);
}

static void __init mahimahi_map_io(void)
{
	msm_map_common_io();
	msm_clock_init();
}

extern struct sys_timer msm_timer;

MACHINE_START(MAHIMAHI, "mahimahi")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x12000100,
	.fixup		= mahimahi_fixup,
	.map_io		= mahimahi_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= mahimahi_init,
	.timer		= &msm_timer,
MACHINE_END
