/* drivers/video/msm/mdp_lcdc.c
 *
 * Copyright (c) 2009 Google Inc.
 * Copyright (c) 2009 QUALCOMM Incorporated
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>

#include "mdp_hw.h"

#define panel_to_lcdc(p)	container_of((p), struct mdp_lcdc, panel_data)
#define to_panel_info(p)	(&((p)->pdata->panel_info))

static int lcdc_unblank(struct msm_panel_data *panel)
{
	struct mdp_lcdc *lcdc = panel_to_lcdc(panel);

	lcdc->pdata->unblank();
	pr_info("%s: unblank sesame\n", __func__);
	return 0;
}

static int lcdc_blank(struct msm_panel_data *panel)
{
	struct mdp_lcdc *lcdc = panel_to_lcdc(panel);
	pr_info("%s: well <blank> me\n", __func__);
	lcdc->pdata->blank();

	clk_enable(lcdc->mdp->clk);
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_EN);
	clk_disable(lcdc->mdp->clk);

	clk_disable(lcdc->pclk);
	clk_disable(lcdc->pad_pclk);
	return 0;
}

static int lcdc_suspend(struct msm_panel_data *panel)
{
	pr_info("%s: suspending\n", __func__);
	return 0;
}

static int lcdc_resume(struct msm_panel_data *panel)
{
	struct mdp_lcdc *lcdc = panel_to_lcdc(panel);
	uint32_t dma_cfg;

	pr_info("%s: resuming\n", __func__);

	clk_enable(lcdc->mdp->clk);
	clk_enable(lcdc->pclk);
	clk_enable(lcdc->pad_pclk);

	clk_set_rate(lcdc->pclk, lcdc->pdata->panel_info.clk_rate);
	clk_set_rate(lcdc->pad_pclk, lcdc->pdata->panel_info.clk_rate);

	/* write the lcdc params */
	mdp_writel(lcdc->mdp, lcdc->parms.hsync_ctl, MDP_LCDC_HSYNC_CTL);
	mdp_writel(lcdc->mdp, lcdc->parms.vsync_period, MDP_LCDC_VSYNC_PERIOD);
	mdp_writel(lcdc->mdp, lcdc->parms.vsync_pulse_width,
		   MDP_LCDC_VSYNC_PULSE_WIDTH);
	mdp_writel(lcdc->mdp, lcdc->parms.display_hctl, MDP_LCDC_DISPLAY_HCTL);
	mdp_writel(lcdc->mdp, lcdc->parms.display_vstart,
		   MDP_LCDC_DISPLAY_V_START);
	mdp_writel(lcdc->mdp, lcdc->parms.display_vend, MDP_LCDC_DISPLAY_V_END);
	mdp_writel(lcdc->mdp, lcdc->parms.hsync_skew, MDP_LCDC_HSYNC_SKEW);

	mdp_writel(lcdc->mdp, 0, MDP_LCDC_BORDER_CLR);
	mdp_writel(lcdc->mdp, 0xff, MDP_LCDC_UNDERFLOW_CTL);
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_CTL_POLARITY);
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_ACTIVE_HCTL);
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_ACTIVE_V_START);
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_ACTIVE_V_END);

	/* config the dma_p block that drives the lcdc data */
	mdp_writel(lcdc->mdp, lcdc->fb_start, MDP_DMA_P_IBUF_ADDR);
	mdp_writel(lcdc->mdp, (((panel->fb_data->yres & 0x7ff) << 16) |
			       (panel->fb_data->xres & 0x7ff)),
		   MDP_DMA_P_SIZE);
	/* TODO: pull in the bpp info from somewhere else? */
	mdp_writel(lcdc->mdp, panel->fb_data->xres * 2,
		   MDP_DMA_P_IBUF_Y_STRIDE);
	mdp_writel(lcdc->mdp, 0, MDP_DMA_P_OUT_XY);

	dma_cfg = (DMA_PACK_ALIGN_LSB |
		   DMA_PACK_PATTERN_RGB |
		   DMA_DITHER_EN);
	dma_cfg |= DMA_OUT_SEL_LCDC;
	dma_cfg |= DMA_IBUF_FORMAT_RGB565;
	dma_cfg |= DMA_DSTC0G_8BITS | DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;
	mdp_writel(lcdc->mdp, dma_cfg, MDP_DMA_P_CONFIG);

	/* enable the lcdc timing generation */
	mdp_writel(lcdc->mdp, 1, MDP_LCDC_EN);

	return 0;
}

#if 0
static void lcdc_wait_vsync(struct msm_panel_data *panel)
{
	pr_info("%s: XXXXXXXXXXXXXXX waiting vsync\n", __func__);
}
#endif

static void lcdc_request_vsync(struct msm_panel_data *panel,
			       struct msmfb_callback *cb)
{
	if (cb)
		cb->func(cb);
}

static void lcdc_clear_vsync(struct msm_panel_data *panel)
{
	return;
}

static void compute_timing_parms(struct mdp_lcdc *lcdc)
{
	struct msm_lcdc_panel_info *panel_info = &lcdc->pdata->panel_info;
	struct msm_fb_data *fb_data = &lcdc->pdata->fb_data;
	unsigned int hsync_period;
	unsigned int hsync_start_x;
	unsigned int hsync_end_x;
	unsigned int vsync_period;
	unsigned int display_vstart;
	unsigned int display_vend;

	hsync_period = (panel_info->hsync_pulse_width +
			panel_info->hsync_back_porch +
			fb_data->xres +
			panel_info->hsync_front_porch);
	hsync_start_x = (panel_info->hsync_pulse_width +
			 panel_info->hsync_back_porch);
	hsync_end_x = hsync_period - panel_info->hsync_front_porch - 1;

	vsync_period = (panel_info->vsync_pulse_width +
			panel_info->vsync_back_porch +
			fb_data->yres +
			panel_info->vsync_front_porch) * hsync_period;
	display_vstart = (panel_info->vsync_pulse_width +
			  panel_info->vsync_back_porch) * hsync_period;
	display_vstart += panel_info->hsync_skew;

	display_vend = panel_info->vsync_front_porch * hsync_period;
	display_vend = vsync_period - display_vend + panel_info->hsync_skew - 1;

	/* register values we pre-compute at init time from the timing
	 * information in the panel info */
	lcdc->parms.hsync_ctl = (((hsync_period & 0xfff) << 16) |
				 (panel_info->hsync_pulse_width & 0xfff));
	lcdc->parms.vsync_period = vsync_period & 0xffffff;
	lcdc->parms.vsync_pulse_width = (panel_info->vsync_pulse_width *
					 hsync_period) & 0xffffff;

	lcdc->parms.display_hctl = (((hsync_end_x & 0xfff) << 16) |
				    (hsync_start_x & 0xfff));
	lcdc->parms.display_vstart = display_vstart & 0xffffff;
	lcdc->parms.display_vend = display_vend & 0xffffff;
	lcdc->parms.hsync_skew = panel_info->hsync_skew & 0xfff;
}

int mdp_lcdc_init(struct mdp_info *mdp, struct platform_device *pdev)
{
	struct device *_dev = mdp->mdp_dev.dev.parent;
	struct msm_mdp_platform_data *pdata = _dev->platform_data;
	struct mdp_lcdc *lcdc;
	int ret = 0;

	if (!pdata || !pdata->lcdc_data) {
		pr_err("%s: no LCDC platform data found\n", __func__);
		return -EINVAL;
	}

	lcdc = kzalloc(sizeof(struct mdp_lcdc), GFP_KERNEL);
	if (!lcdc)
		return -ENOMEM;

	lcdc->mdp = mdp;
	mdp->lcdc = lcdc;
	lcdc->pdata = pdata->lcdc_data;

	lcdc->pclk = clk_get(_dev, "lcdc_pclk_clk");
	if (IS_ERR(lcdc->pclk)) {
		pr_err("%s: failed to get lcdc_pclk\n", __func__);
		ret = PTR_ERR(lcdc->pclk);
		goto error_pclk;
	}

	lcdc->pad_pclk = clk_get(_dev, "lcdc_pad_pclk_clk");
	if (IS_ERR(lcdc->pad_pclk)) {
		pr_err("%s: failed to get lcdc_pad_pclk\n", __func__);
		ret = PTR_ERR(lcdc->pad_pclk);
		goto error_pad_pclk;
	}

	compute_timing_parms(lcdc);

	lcdc->fb_start = lcdc->pdata->fb_resource->start;

	lcdc->panel_data.suspend = lcdc_suspend;
	lcdc->panel_data.resume = lcdc_resume;
/*
	lcdc->panel_data.wait_vsync = lcdc_wait_vsync;
*/

	lcdc->panel_data.request_vsync = lcdc_request_vsync;
	lcdc->panel_data.clear_vsync = lcdc_clear_vsync;

	lcdc->panel_data.blank = lcdc_blank;
	lcdc->panel_data.unblank = lcdc_unblank;
	lcdc->panel_data.fb_data =  &lcdc->pdata->fb_data;
	lcdc->panel_data.interface_type = MSM_LCDC_INTERFACE;

	lcdc->pdev.name = "msm_panel";
	lcdc->pdev.id = pdev->id;
	lcdc->pdev.resource = lcdc->pdata->fb_resource;
	lcdc->pdev.num_resources = 1;
	lcdc->pdev.dev.platform_data = &lcdc->panel_data;

	lcdc_resume(&lcdc->panel_data);

	platform_device_register(&lcdc->pdev);

	printk(KERN_INFO "mdp_lcdc initialized\n");

	return 0;

error_pad_pclk:
	clk_put(lcdc->pclk);
error_pclk:
	kfree(lcdc);
	return ret;
}
