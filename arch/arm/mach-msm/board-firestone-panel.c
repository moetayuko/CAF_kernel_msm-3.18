/* linux/arch/arm/mach-msm/board-firestone-panel.c
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>

#include "board-firestone.h"
#include "devices.h"

#define CLK_NS_TO_RATE(ns)			(1000000000UL / (ns))

#define MICROP_CMD_SPI_INTERFACE		0x21
#define MICROP_CMD_VERSION			0x29
#define MICROP_WRITE_LCM_DATA_1BYTE		0x70

#define MICROP_NUM_I2C_RETRIES			100
#define MICROP_I2C_WRITE_BLOCK_SIZE		21

static struct i2c_client *microp_client;

static int microp_read_buf(uint8_t addr, void *data, int length)

{
	struct i2c_msg msgs[2];
	int cnt = MICROP_NUM_I2C_RETRIES;
	int ret = 0;

	msgs[0].addr = microp_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &addr;
	msgs[1].addr = microp_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = length;
	msgs[1].buf = data;

	while (cnt--) {
		ret = i2c_transfer(microp_client->adapter, msgs, 2);
		if (ret == 2)
			goto done;
		mdelay(10);
	}

	dev_err(&microp_client->dev, "%s: failed to read buf\n", __func__);
	return -EIO;

done:
	return 0;
}

static int microp_write_buf(uint8_t addr, void *data, int length)
{
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	int retry = MICROP_NUM_I2C_RETRIES;
	int ret;
	struct i2c_msg msg;

	if (length >= MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&microp_client->dev, "%s: length (%d) too long\n",
			__func__, length);
		return -EINVAL;
	}

	msg.addr = microp_client->addr,
	msg.flags = 0,
	msg.len = length + 1,
	msg.buf = buf;

	buf[0] = addr;
	memcpy(&buf[1], data, length);

	do {
		ret = i2c_transfer(microp_client->adapter, &msg, 1);
		if (ret == 1)
			goto done;
		mdelay(10);
	} while (retry--);

	dev_err(&microp_client->dev, "%s: Could not write data (%d bytes).\n",
		__func__, length);
	return -EIO;

done:
	return 0;
}

static int lcm_writeb(uint16_t reg, uint8_t val)
{
	uint8_t buf[3];
	int ret;

	buf[0] = (reg >> 8) & 0xff;
	buf[1] = reg & 0xff;
	buf[2] = val;

	ret = microp_write_buf(MICROP_WRITE_LCM_DATA_1BYTE, buf, 3);
	if (ret) {
		pr_err("%s: failed lcm_writeb\n", __func__);
		goto done;
	}
	udelay(100);

done:
	return ret;
}

static int samsung_oled_panel_blank(struct msm_lcdc_panel_ops *ops)
{
	pr_info("%s: +()\n", __func__);
	lcm_writeb(0x14, 0x1);
	msleep(200);
	lcm_writeb(0x1d, 0xa1);
	msleep(300);
	pr_info("%s: -()\n", __func__);
	return 0;
}

static int samsung_oled_panel_unblank(struct msm_lcdc_panel_ops *ops)
{
	pr_info("%s: +()\n", __func__);
	lcm_writeb(0x1d, 0xa0);
	msleep(200);
	lcm_writeb(0x14, 0x03);
	msleep(100);
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
	for (i = 0; i< ARRAY_SIZE(samsung_oled_init_table); i++) {
		lcm_writeb(samsung_oled_init_table[i].reg,
			   samsung_oled_init_table[i].val);
		if (samsung_oled_init_table[i].delay)
			mdelay(samsung_oled_init_table[i].delay);
	}

	lcm_writeb(0x14, 0x3);
	pr_info("%s: -()\n", __func__);

	return 0;
}

static struct msm_lcdc_panel_ops firestone_lcdc_panel_ops = {
	.init			= samsung_oled_panel_init,
	.blank			= samsung_oled_panel_blank,
	.unblank			= samsung_oled_panel_unblank,
};

static struct msm_lcdc_timing firestone_lcdc_timing = {
		.clk_rate		= CLK_NS_TO_RATE(26),
		.hsync_pulse_width	= 4,
		.hsync_back_porch	= 4,
		.hsync_front_porch	= 8,
		.hsync_skew		= 0,
		.vsync_pulse_width	= 2,
		.vsync_back_porch	= 6,
		.vsync_front_porch	= 8,
		.vsync_act_low		= 1,
		.hsync_act_low		= 1,
		.den_act_low		= 1,
};

static struct msm_fb_data firestone_lcdc_fb_data = {
		.xres		= 480,
		.yres		= 800,
		.width		= 48,
		.height		= 80,
		.output_format	= 0,
};

static struct msm_lcdc_platform_data firestone_lcdc_platform_data = {
	.panel_ops	= &firestone_lcdc_panel_ops,
	.timing		= &firestone_lcdc_timing,
	.fb_id		= 0,
	.fb_data	= &firestone_lcdc_fb_data,
	.fb_resource	= &resources_msm_fb[0],
};

static struct platform_device firestone_lcdc_device = {
	.name	= "msm_mdp_lcdc",
	.id	= -1,
	.dev	= {
		.platform_data = &firestone_lcdc_platform_data,
	},
};

static int microp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	uint16_t ver;

	microp_client = client;
	ret = microp_read_buf(MICROP_CMD_VERSION, &ver, 2);
	if (ret || ver == 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get microp version\n");
		goto err;
	}

	ret = platform_device_register(&firestone_lcdc_device);
	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to register lcdc device\n");
		goto err;
	}
	pr_info("%s: microp ver=%0x registered\n", __func__, ver);

	return 0;

err:
	microp_client = NULL;
	return ret;
}

static struct i2c_device_id microp_id[] = {
	{ "microp-i2c", 0},
	{ }
};

static struct i2c_driver microp_driver = {
	.probe		= microp_probe,
	.id_table	= microp_id,
	.driver = {
		.name	= "microp-i2c",
		.owner	= THIS_MODULE,
	},
};

int __init firestone_init_panel(void)
{
	int rc;
	if (!machine_is_firestone())
		return 0;

	if ((rc = platform_device_register(&msm_device_mdp)) != 0)
		return rc;

	rc = i2c_add_driver(&microp_driver);
	if (rc != 0)
		return rc;

	return 0;
}

late_initcall(firestone_init_panel);
