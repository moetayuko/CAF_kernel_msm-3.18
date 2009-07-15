/* linux/arch/arm/mach-msm/board-mahimahi-panel.c
 *
 * Copyright (c) 2009 Google Inc.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>

#include "board-mahimahi.h"
#include "devices.h"


#define SPI_CONFIG              (0x00000000)
#define SPI_IO_CONTROL          (0x00000004)
#define SPI_OPERATIONAL         (0x00000030)
#define SPI_ERROR_FLAGS_EN      (0x00000038)
#define SPI_ERROR_FLAGS         (0x00000038)
#define SPI_OUTPUT_FIFO         (0x00000100)

static void __iomem *spi_base;
static struct clk *spi_clk ;

#define CLK_NS_TO_RATE(ns)			(1000000000UL / (ns))

static int qspi_send(uint32_t id, uint8_t data)
{
	uint32_t err;

	/* bit-5: OUTPUT_FIFO_NOT_EMPTY */
	while (readl(spi_base + SPI_OPERATIONAL) & (1<<5)) {
		if ((err = readl(spi_base + SPI_ERROR_FLAGS))) {
			pr_err("%s: ERROR: SPI_ERROR_FLAGS=0x%08x\n", __func__,
			       err);
			return -EIO;
		}
	}
	writel((0x7000 | (id << 9) | data) << 16, spi_base + SPI_OUTPUT_FIFO);
	udelay(100);

	return 0;
}

static int lcm_writeb(uint8_t reg, uint8_t val)
{
	qspi_send(0x0, reg);
	qspi_send(0x1, val);
	return 0;
}

static int samsung_oled_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	pr_info("%s: +()\n", __func__);
	clk_enable(spi_clk);
	lcm_writeb(0x14, 0x1);
	msleep(200);
	lcm_writeb(0x1d, 0xa1);
	msleep(300);
	clk_disable(spi_clk);
	pr_info("%s: -()\n", __func__);
	return 0;
}

static int samsung_oled_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	pr_info("%s: +()\n", __func__);
	clk_enable(spi_clk);
	lcm_writeb(0x1d, 0xa0);
	msleep(200);
	lcm_writeb(0x14, 0x03);
	msleep(100);
	clk_disable(spi_clk);
	pr_info("%s: -()\n", __func__);
	return 0;
}

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct {
	uint8_t		reg;
	uint8_t		val;
	uint16_t	delay;
} samsung_oled_init_table[] = {
	{0x31, 0x08, 0},
	{0x32, 0x14, 0},
	{0x30, 0x2 , 0},
	{0x27, 0x1 , 0},
	{0x12, 0x8 , 0},
	{0x13, 0x8 , 0},
	{0x15, 0x0 , 0},
	{0x16, 0x02, 0},
	{0x39, 0x44, 0},
	{0x17, 0x22, 0},
	{0x18, 0x33, 0},
	{0x19, 0x3 , 0},
	{0x1A, 0x1 , 0},
	{0x22, 0xA4, 0},
	{0x23, 0x0 , 0},
	{0x26, 0xA0, 0},
	{0x1D, 0xA0, 250},
};

static int samsung_oled_panel_init(struct msm_lcdc_panel_ops *ops)
{
	int i;

	pr_info("%s: +()\n", __func__);
	clk_enable(spi_clk);
	for (i = 0; i< ARRAY_SIZE(samsung_oled_init_table); i++) {
		lcm_writeb(samsung_oled_init_table[i].reg,
			   samsung_oled_init_table[i].val);
		if (samsung_oled_init_table[i].delay)
			mdelay(samsung_oled_init_table[i].delay);
	}

	lcm_writeb(0x14, 0x3);
	clk_disable(spi_clk);
	pr_info("%s: -()\n", __func__);

	return 0;
}

static struct msm_lcdc_panel_ops mahimahi_lcdc_panel_ops = {
	.init		= samsung_oled_panel_init,
	.blank		= samsung_oled_panel_blank,
	.unblank	= samsung_oled_panel_unblank,
};

static struct msm_lcdc_timing mahimahi_lcdc_timing = {
		.clk_rate		= CLK_NS_TO_RATE(26),
		.hsync_pulse_width	= 4,
		.hsync_back_porch	= 8,
		.hsync_front_porch	= 8,
		.hsync_skew		= 0,
		.vsync_pulse_width	= 2,
		.vsync_back_porch	= 8,
		.vsync_front_porch	= 8,
		.vsync_act_low		= 1,
		.hsync_act_low		= 1,
		.den_act_low		= 1,
};

static struct msm_fb_data mahimahi_lcdc_fb_data = {
		.xres		= 480,
		.yres		= 800,
		.width		= 48,
		.height		= 80,
		.output_format	= 0,
};

static struct msm_lcdc_platform_data mahimahi_lcdc_platform_data = {
	.panel_ops	= &mahimahi_lcdc_panel_ops,
	.timing		= &mahimahi_lcdc_timing,
	.fb_id		= 0,
	.fb_data	= &mahimahi_lcdc_fb_data,
	.fb_resource	= &resources_msm_fb[0],
};

static struct platform_device mahimahi_lcdc_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &mahimahi_lcdc_platform_data,
	},
};

static int mahimahi_init_spi_hack(void)
{
	int ret;

	spi_base = ioremap(MSM_SPI_PHYS, MSM_SPI_SIZE);
	if (!spi_base)
		return -1;

	spi_clk = clk_get(&msm_device_spi.dev, "spi_clk");
	if (IS_ERR(spi_clk)) {
		pr_err("%s: unable to get spi_clk\n", __func__);
		ret = PTR_ERR(spi_clk);
		goto err_clk_get;
	}

	clk_enable(spi_clk);

	printk("spi: SPI_CONFIG=%x\n", readl(spi_base + SPI_CONFIG));
	printk("spi: SPI_IO_CONTROL=%x\n", readl(spi_base + SPI_IO_CONTROL));
	printk("spi: SPI_OPERATIONAL=%x\n", readl(spi_base + SPI_OPERATIONAL));
	printk("spi: SPI_ERROR_FLAGS_EN=%x\n",
	       readl(spi_base + SPI_ERROR_FLAGS_EN));
	printk("spi: SPI_ERROR_FLAGS=%x\n", readl(spi_base + SPI_ERROR_FLAGS));
	printk("-%s()\n", __FUNCTION__);
	clk_disable(spi_clk);

	return 0;

err_clk_get:
	iounmap(spi_base);
	return ret;
}

int __init mahimahi_init_panel(void)
{
	int ret;

	if (!machine_is_mahimahi())
		return 0;

	ret = platform_device_register(&msm_device_mdp);
	if (ret != 0)
		return ret;

	ret = mahimahi_init_spi_hack();
	if (ret != 0)
		return ret;

	ret = platform_device_register(&mahimahi_lcdc_device);
	if (ret != 0)
		return ret;

	return 0;
}

late_initcall(mahimahi_init_panel);
