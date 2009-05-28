/* linux/arch/arm/mach-msm/board-halibut-mddi.c
** Author: Brian Swetland <swetland@google.com>
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/bootmem.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>

#include "proc_comm.h"
#include "devices.h"

#define MSM_FB_SIZE 0x00200000

#define MDDI_CLIENT_CORE_BASE  0x108000
#define LCD_CONTROL_BLOCK_BASE 0x110000
#define PWM_BLOCK_BASE         0x140000
#define DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define SYSCLKENA   (MDDI_CLIENT_CORE_BASE|0x2C)
#define START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define PWM0OFF     (PWM_BLOCK_BASE|0x1C)

extern void mddi_remote_write(struct msm_mddi_client_data *cdata,
			uint32_t val,
			uint32_t reg);

static void halibut_mddi_power_client(struct msm_mddi_client_data *mddi,
	int on)
{
#if 0
        if(on) {
                mddi_remote_write(mddi, 0, DPSUS);
                udelay(122);
                mddi_remote_write(mddi, 1, SYSCLKENA);
                mddi_remote_write(mddi, 0x00001387, PWM0OFF);
        }
        else {
                mddi_remote_write(mddi, 0, PWM0OFF);
                udelay(122);
                mddi_remote_write(mddi, 0, SYSCLKENA);
                mddi_remote_write(mddi, 1, DPSUS);
        }
#endif
}

static struct resource resources_msm_fb[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

struct msm_mddi_bridge_platform_data dummy_client_data = {
	.fb_data = {
		.yres = 480,
		.xres = 800,
		.width = 45,
		.height = 67,
		.output_format = 0,
	},
};

static struct msm_mddi_platform_data mddi_pdata = {
	.clk_rate = 153600000,
	.power_client = halibut_mddi_power_client,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0x4474 << 16 | 0xc065),
			.name = "mddi_c_dummy",
			.id = 0,
			.client_data = &dummy_client_data.fb_data,
			.clk_rate = 0,
		},
	},
};

int __init halibut_init_panel(void)
{
	int rc;

	if (!machine_is_halibut())
		return 0;

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	resources_msm_fb[0].end = resources_msm_fb[0].start =
		alloc_bootmem(MSM_FB_SIZE*2);
	resources_msm_fb[0].end += MSM_FB_SIZE*2;
	printk(KERN_INFO "halibut: framebuffer is at %p\n", resources_msm_fb[0].start);
	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	return platform_device_register(&msm_device_mddi0);
}

device_initcall(halibut_init_panel);
