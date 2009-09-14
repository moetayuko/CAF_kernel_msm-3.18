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
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/usb/mass_storage_function.h>
#include <linux/android_pmem.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/a1026.h>
#include <linux/capella_cm3602.h>
#include <linux/akm8973.h>
#include <../../../drivers/staging/android/timed_gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_debugger.h>
#include <mach/system.h>

#include "board-mahimahi.h"
#include "devices.h"
#include "proc_comm.h"

static uint debug_uart;

module_param_named(debug_uart, debug_uart, uint, 0);

extern void msm_init_pmic_vibrator(void);
extern void __init mahimahi_audio_init(void);

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

static int mahimahi_phy_init_seq[] = {
	0x0C, 0x31,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1 };

static void mahimahi_usb_phy_reset(void)
{
	u32 id;
	int ret;

	id = PCOM_CLKRGM_APPS_RESET_USB_PHY;
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}

	msleep(1);

	id = PCOM_CLKRGM_APPS_RESET_USB_PHY;
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT, &id, NULL);
	if (ret) {
		pr_err("%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
}

static void mahimahi_usb_hw_reset(bool enable)
{
	u32 id;
	int ret;
	u32 func;

	id = PCOM_CLKRGM_APPS_RESET_USBH;
	if (enable)
		func = PCOM_CLK_REGIME_SEC_RESET_ASSERT;
	else
		func = PCOM_CLK_REGIME_SEC_RESET_DEASSERT;
	ret = msm_proc_comm(func, &id, NULL);
	if (ret)
		pr_err("%s: Cannot set reset to %d (%d)\n", __func__, enable,
		       ret);
}


static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= mahimahi_phy_init_seq,
	.phy_reset		= mahimahi_usb_phy_reset,
	.hw_reset		= mahimahi_usb_hw_reset,
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

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.start		= MSM_PMEM_ADSP_BASE,
	.size		= MSM_PMEM_ADSP_SIZE,
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

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 1,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
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

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static int mahimahi_ts_power(int on)
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
	.power = mahimahi_ts_power,
};

static struct synaptics_i2c_rmi_platform_data mahimahi_synaptics_ts_data[] = {
	{
		.power = mahimahi_ts_power,
		.flags = SYNAPTICS_FLIP_Y,
		.inactive_left = -25 * 0x10000 / 480,
		.inactive_right = -20 * 0x10000 / 480,
		.inactive_top = -15 * 0x10000 / 800,
		.inactive_bottom = -40 * 0x10000 / 800,
		.sensitivity_adjust = 12,
	},
};

static struct a1026_platform_data a1026_data = {
	.gpio_a1026_micsel = MAHIMAHI_AUD_MICPATH_SEL,
	.gpio_a1026_wakeup = MAHIMAHI_AUD_A1026_WAKEUP,
	.gpio_a1026_reset = MAHIMAHI_AUD_A1026_RESET,
	/*.gpio_a1026_int = MAHIMAHI_AUD_A1026_INT,*/
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = MAHIMAHI_LAYOUTS,
	.project_name = MAHIMAHI_PROJECT_NAME,
	.reset = MAHIMAHI_GPIO_COMPASS_RST_N,
	.intr = MAHIMAHI_GPIO_COMPASS_INT_N,
};

static struct i2c_board_info base_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ds2482", 0x30 >> 1),
	},
	{
		I2C_BOARD_INFO("cy8c-tmg-ts", 0x34),
		.platform_data = &mahimahi_cy8c_ts_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_TP_INT_N),
	},
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x40),
		.platform_data = mahimahi_synaptics_ts_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_TP_INT_N)
	},
	{
		I2C_BOARD_INFO("mahimahi-microp", 0x66),
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_UP_INT_N)
	},
};

static struct i2c_board_info rev1_i2c_devices[] = {
	{
		I2C_BOARD_INFO("audience_a1026", 0x3E),
		.platform_data = &a1026_data,
		/*.irq = MSM_GPIO_TO_INT(MAHIMAHI_AUD_A1026_INT)*/
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(MAHIMAHI_GPIO_COMPASS_INT_N),
	},
};

static int capella_cm3602_power(int on)
{
	/* TODO eolsen Add Voltage reg control */
	if (on) {
		gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 0);
	} else {
		gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 1);
	}

	return 0;
}


static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en = MAHIMAHI_GPIO_PROXIMITY_EN,
	.p_out = MAHIMAHI_GPIO_PROXIMITY_INT_N
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = MAHIMAHI_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device mahimahi_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
	&ram_console_device,
	&mahimahi_rfkill,
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_hsusb,
	&usb_mass_storage_device,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
	&android_pmem_gpu0_device,
	&android_pmem_gpu1_device,
	&msm_kgsl_device,
	&msm_device_i2c,
	&capella_cm3602
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

static int __init board_serialno_setup(char *serialno)
{
	msm_hsusb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

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

static ssize_t mahimahi_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	/* center: x: home: 55, menu: 185, back: 305, search 425, y: 835 */
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":55:835:70:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":185:835:100:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":305:835:70:55"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":425:835:70:55"
	   "\n");
}

static struct kobj_attribute mahimahi_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &mahimahi_virtual_keys_show,
};

static struct attribute *mahimahi_properties_attrs[] = {
	&mahimahi_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group mahimahi_properties_attr_group = {
	.attrs = mahimahi_properties_attrs,
};

static void mahimahi_reset(void)
{
	gpio_set_value(MAHIMAHI_GPIO_PS_HOLD, 0);
}

int mahimahi_init_mmc(int sysrev, unsigned debug_uart);

static void __init mahimahi_init(void)
{
	int ret;
	struct kobject *properties_kobj;

	printk("mahimahi_init() revision=%d\n", system_rev);

	msm_hw_reset_hook = mahimahi_reset;

	msm_acpu_clock_init(&mahimahi_clock_data);

	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      &msm_device_uart1.dev, 1, MSM_GPIO_TO_INT(139));

	config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));
	gpio_direction_output(MAHIMAHI_GPIO_TP_LS_EN, 0);
	gpio_direction_output(MAHIMAHI_GPIO_TP_EN, 0);
	gpio_direction_output(MAHIMAHI_GPIO_PROXIMITY_EN, 1);
	gpio_direction_output(MAHIMAHI_GPIO_COMPASS_RST_N, 1);
	gpio_direction_input(MAHIMAHI_GPIO_COMPASS_INT_N);

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(0, base_i2c_devices,
		ARRAY_SIZE(base_i2c_devices));

	if (system_rev > 0) {
		/* Only board after XB with Audience A1026 */
		i2c_register_board_info(0, rev1_i2c_devices,
			ARRAY_SIZE(rev1_i2c_devices));
	}

	ret = mahimahi_init_mmc(system_rev, debug_uart);
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC\n", __func__);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
					 &mahimahi_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");

	mahimahi_audio_init();

	if (system_rev > 0)
		platform_device_register(&mahimahi_timed_gpios);
	else
		msm_init_pmic_vibrator();
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
	.boot_params	= 0x20000100,
	.fixup		= mahimahi_fixup,
	.map_io		= mahimahi_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= mahimahi_init,
	.timer		= &msm_timer,
MACHINE_END
