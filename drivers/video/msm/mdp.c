/* drivers/video/msm_fb/mdp.c
 *
 * MSM MDP Interface (used by framebuffer core)
 *
 * Copyright (C) 2007 QUALCOMM Incorporated
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/file.h>
#include <linux/list.h>
#include <linux/android_pmem.h>
#include <linux/major.h>
#include <linux/msm_hw3d.h>

#include <mach/msm_iomap.h>
#include <mach/msm_fb.h>
#include <linux/platform_device.h>

#include "mdp_hw.h"

struct class *mdp_class;

#define MDP_CMD_DEBUG_ACCESS_BASE (0x10000)

static uint16_t mdp_default_ccs[] = {
	0x254, 0x000, 0x331, 0x254, 0xF38, 0xE61, 0x254, 0x409, 0x000,
	0x010, 0x080, 0x080
};

static DECLARE_WAIT_QUEUE_HEAD(mdp_dma2_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(mdp_ppp_waitqueue);
static struct msmfb_callback *dma_callback;
static struct clk *clk;
static unsigned int mdp_irq_mask;
static DEFINE_SPINLOCK(mdp_lock);
DEFINE_MUTEX(mdp_mutex);

struct mdp_img_ops {
	int		(*get)(struct mdp_img *img, struct fb_info *fb,
			       struct file **filep, unsigned long *pbase,
			       unsigned long *len);
	void		(*put)(struct file *file);
	bool		(*is_owner)(struct file *);
	char		name[];
};

static bool mdp_is_owner_pmem(struct file *file)
{
	return file && is_pmem_file(file);
}

static int mdp_get_file_pmem(struct mdp_img *img, struct fb_info *fb,
			     struct file **filep, unsigned long *pbase,
			     unsigned long *len)
{
	unsigned long vstart;
	return get_pmem_file(img->memory_id, pbase, &vstart, len, filep);
}

static void mdp_put_file_pmem(struct file *file)
{
	if (file)
		put_pmem_file(file);
}

static bool mdp_is_owner_msm_hw3d(struct file *file)
{
	return file && is_msm_hw3d_file(file);
}

static int mdp_get_file_msm_hw3d(struct mdp_img *img, struct fb_info *fb,
				 struct file **filep, unsigned long *pbase,
				 unsigned long *len)
{
	int ret;

	ret = get_msm_hw3d_file(img->memory_id, HW3D_REGION_ID(img->offset),
				HW3D_OFFSET_IN_REGION(img->offset), pbase, len,
				filep);
	if (ret)
		return -1;
	/* need to chop off the region id from the user offset */
	img->offset = HW3D_OFFSET_IN_REGION(img->offset);
	return 0;
}

static void mdp_put_file_msm_hw3d(struct file *file)
{
	if (file)
		put_msm_hw3d_file(file);
}

static bool mdp_is_owner_msm_fb(struct file *file)
{
	if (file && MAJOR(file->f_dentry->d_inode->i_rdev) == FB_MAJOR)
		return 1;
	return 0;
}
static int mdp_get_file_msm_fb(struct mdp_img *img, struct fb_info *fb,
			       struct file **filep, unsigned long *pbase,
			       unsigned long *len)
{
	*filep = fget(img->memory_id);
	if (*filep == NULL)
		return -1;
	else if (!mdp_is_owner_msm_fb(*filep)) {
		fput(*filep);
		return -1;
	}

	*pbase = fb->fix.smem_start;
	*len = fb->fix.smem_len;
	return 0;
}

static void mdp_put_file_msm_fb(struct file *file)
{
	if (file)
		fput(file);
}

static struct mdp_img_ops mdp_img_types[] = {
	{
		.name		= "pmem",
		.get		= mdp_get_file_pmem,
		.put		= mdp_put_file_pmem,
		.is_owner	= mdp_is_owner_pmem,
	},
	{
		.name		= "msm_hw3d",
		.get		= mdp_get_file_msm_hw3d,
		.put		= mdp_put_file_msm_hw3d,
		.is_owner	= mdp_is_owner_msm_hw3d,
	},
	{
		.name		= "fb",
		.get		= mdp_get_file_msm_fb,
		.put		= mdp_put_file_msm_fb,
		.is_owner	= mdp_is_owner_msm_fb,
	},
};

static int enable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	unsigned long irq_flags;
	int ret = 0;

	BUG_ON(!mask);

	spin_lock_irqsave(&mdp_lock, irq_flags);
	/* if the mask bits are already set return an error, this interrupt
	 * is already enabled */
	if (mdp_irq_mask & mask) {
		printk(KERN_ERR "mdp irq already on already on %x %x\n",
		       mdp_irq_mask, mask);
		ret = -1;
	}
	/* if the mdp irq is not already enabled enable it */
	if (!mdp_irq_mask) {
		if (clk)
			clk_enable(clk);
		enable_irq(mdp->irq);
	}

	/* update the irq mask to reflect the fact that the interrupt is
	 * enabled */
	mdp_irq_mask |= mask;
	spin_unlock_irqrestore(&mdp_lock, irq_flags);
	return ret;
}

static int locked_disable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	/* this interrupt is already disabled! */
	if (!(mdp_irq_mask & mask)) {
		printk(KERN_ERR "mdp irq already off %x %x\n",
		       mdp_irq_mask, mask);
		return -1;
	}
	/* update the irq mask to reflect the fact that the interrupt is
	 * disabled */
	mdp_irq_mask &= ~(mask);
	/* if no one is waiting on the interrupt, disable it */
	if (!mdp_irq_mask) {
		disable_irq(mdp->irq);
		if (clk)
			clk_disable(clk);
	}
	return 0;
}

static int disable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	unsigned long irq_flags;
	int ret;

	spin_lock_irqsave(&mdp_lock, irq_flags);
	ret = locked_disable_mdp_irq(mdp, mask);
	spin_unlock_irqrestore(&mdp_lock, irq_flags);
	return ret;
}

static irqreturn_t mdp_isr(int irq, void *data)
{
	uint32_t status;
	unsigned long irq_flags;
	struct mdp_info *mdp = data;

	spin_lock_irqsave(&mdp_lock, irq_flags);

	status = mdp_readl(mdp, MDP_INTR_STATUS);
	mdp_writel(mdp, status, MDP_INTR_CLEAR);

	status &= mdp_irq_mask;
	if (status & DL0_DMA2_TERM_DONE) {
		if (dma_callback) {
			dma_callback->func(dma_callback);
			dma_callback = NULL;
		}
		wake_up(&mdp_dma2_waitqueue);
	}

	if (status & DL0_ROI_DONE)
		wake_up(&mdp_ppp_waitqueue);

	if (status)
		locked_disable_mdp_irq(mdp, status);

	spin_unlock_irqrestore(&mdp_lock, irq_flags);
	return IRQ_HANDLED;
}

static uint32_t mdp_check_mask(uint32_t mask)
{
	uint32_t ret;
	unsigned long irq_flags;

	spin_lock_irqsave(&mdp_lock, irq_flags);
	ret = mdp_irq_mask & mask;
	spin_unlock_irqrestore(&mdp_lock, irq_flags);
	return ret;
}

static int mdp_wait(struct mdp_info *mdp, uint32_t mask, wait_queue_head_t *wq)
{
	int ret = 0;
	unsigned long irq_flags;

	wait_event_timeout(*wq, !mdp_check_mask(mask), HZ);

	spin_lock_irqsave(&mdp_lock, irq_flags);
	if (mdp_irq_mask & mask) {
		locked_disable_mdp_irq(mdp, mask);
		printk(KERN_WARNING "timeout waiting for mdp to complete %x\n",
		       mask);
		ret = -ETIMEDOUT;
	}
	spin_unlock_irqrestore(&mdp_lock, irq_flags);

	return ret;
}

void mdp_dma_wait(struct mdp_device *mdp_dev)
{
#define MDP_MAX_TIMEOUTS 20
	static int timeout_count;
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);

	if (mdp_wait(mdp, DL0_DMA2_TERM_DONE, &mdp_dma2_waitqueue) == -ETIMEDOUT)
		timeout_count++;
	else
		timeout_count = 0;

	if (timeout_count > MDP_MAX_TIMEOUTS) {
		printk(KERN_ERR "mdp: dma failed %d times, somethings wrong!\n",
		       MDP_MAX_TIMEOUTS);
		BUG();
	}
}

static int mdp_ppp_wait(struct mdp_info *mdp)
{
	return mdp_wait(mdp, DL0_ROI_DONE, &mdp_ppp_waitqueue);
}

void mdp_dma_to_mddi(struct mdp_info *mdp, uint32_t addr, uint32_t stride,
		     uint32_t width, uint32_t height, uint32_t x, uint32_t y,
		     struct msmfb_callback *callback)
{
	uint32_t dma2_cfg;
	uint16_t ld_param = 0; /* 0=PRIM, 1=SECD, 2=EXT */

	if (enable_mdp_irq(mdp, DL0_DMA2_TERM_DONE)) {
		printk(KERN_ERR "mdp_dma_to_mddi: busy\n");
		return;
	}

	dma_callback = callback;

	dma2_cfg = DMA_PACK_TIGHT |
		DMA_PACK_ALIGN_LSB |
		DMA_PACK_PATTERN_RGB |
		DMA_OUT_SEL_AHB |
		DMA_IBUF_NONCONTIGUOUS;

	dma2_cfg |= DMA_IBUF_FORMAT_RGB565;

	dma2_cfg |= DMA_OUT_SEL_MDDI;

	dma2_cfg |= DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY;

	dma2_cfg |= DMA_DITHER_EN;

	/* setup size, address, and stride */
	mdp_writel(mdp, (height << 16) | (width),
		   MDP_CMD_DEBUG_ACCESS_BASE + 0x0184);
	mdp_writel(mdp, addr, MDP_CMD_DEBUG_ACCESS_BASE + 0x0188);
	mdp_writel(mdp, stride, MDP_CMD_DEBUG_ACCESS_BASE + 0x018C);

	/* 666 18BPP */
	dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;

	/* set y & x offset and MDDI transaction parameters */
	mdp_writel(mdp, (y << 16) | (x), MDP_CMD_DEBUG_ACCESS_BASE + 0x0194);
	mdp_writel(mdp, ld_param, MDP_CMD_DEBUG_ACCESS_BASE + 0x01a0);
	mdp_writel(mdp, (MDDI_VDO_PACKET_DESC << 16) | MDDI_VDO_PACKET_PRIM,
		   MDP_CMD_DEBUG_ACCESS_BASE + 0x01a4);

	mdp_writel(mdp, dma2_cfg, MDP_CMD_DEBUG_ACCESS_BASE + 0x0180);

	/* start DMA2 */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0044);
}

void mdp_dma(struct mdp_device *mdp_dev, uint32_t addr, uint32_t stride,
	     uint32_t width, uint32_t height, uint32_t x, uint32_t y,
	     struct msmfb_callback *callback, int interface)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);

	if (interface == MSM_MDDI_PMDH_INTERFACE) {
		mdp_dma_to_mddi(mdp, addr, stride, width, height, x, y,
				callback);
	}
}

static int get_img(struct mdp_img *img, struct fb_info *info,
		   unsigned long *start, unsigned long *len,
		   struct file** filep)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mdp_img_types) && ret != 0; ++i)
		ret = mdp_img_types[i].get(img, info, filep, start, len);
	return ret;
}

static void put_img(struct file *file)
{
	int i;
	int ret = -1;

	for (i = 0; i < ARRAY_SIZE(mdp_img_types) && ret != 0; ++i) {
		if (mdp_img_types[i].is_owner(file)) {
			mdp_img_types[i].put(file);
			return;
		}
	}
	pr_info("%s: unknown file\n", __func__);
}

int mdp_blit(struct mdp_device *mdp_dev, struct fb_info *fb,
	     struct mdp_blit_req *req)
{
	int ret;
	unsigned long src_start = 0, src_len = 0, dst_start = 0, dst_len = 0;
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	struct file *src_file = 0, *dst_file = 0;

	/* WORKAROUND FOR HARDWARE BUG IN BG TILE FETCH */
	if (unlikely(req->src_rect.h == 0 ||
		     req->src_rect.w == 0)) {
		printk(KERN_ERR "mpd_ppp: src img of zero size!\n");
		return -EINVAL;
	}
	if (unlikely(req->dst_rect.h == 0 ||
		     req->dst_rect.w == 0))
		return -EINVAL;

	/* do this first so that if this fails, the caller can always
	 * safely call put_img */
	if (unlikely(get_img(&req->src, fb, &src_start, &src_len, &src_file))) {
		printk(KERN_ERR "mpd_ppp: could not retrieve src image from "
				"memory\n");
		return -EINVAL;
	}

	if (unlikely(get_img(&req->dst, fb, &dst_start, &dst_len, &dst_file))) {
		printk(KERN_ERR "mpd_ppp: could not retrieve dst image from "
				"memory\n");
		put_img(src_file);
		return -EINVAL;
	}
	mutex_lock(&mdp_mutex);

	/* transp_masking unimplemented */
	req->transp_mask = MDP_TRANSP_NOP;
	if (unlikely((req->transp_mask != MDP_TRANSP_NOP ||
		      req->alpha != MDP_ALPHA_NOP ||
		      HAS_ALPHA(req->src.format)) &&
		     (req->flags & MDP_ROT_90 &&
		      req->dst_rect.w <= 16 && req->dst_rect.h >= 16))) {
		int i;
		unsigned int tiles = req->dst_rect.h / 16;
		unsigned int remainder = req->dst_rect.h % 16;
		req->src_rect.w = 16*req->src_rect.w / req->dst_rect.h;
		req->dst_rect.h = 16;
		for (i = 0; i < tiles; i++) {
			enable_mdp_irq(mdp, DL0_ROI_DONE);
			ret = mdp_ppp_blit(mdp, req, src_file, src_start,
					   src_len, dst_file, dst_start,
					   dst_len);
			if (ret)
				goto err_bad_blit;
			ret = mdp_ppp_wait(mdp);
			if (ret)
				goto err_wait_failed;
			req->dst_rect.y += 16;
			req->src_rect.x += req->src_rect.w;
		}
		if (!remainder)
			goto end;
		req->src_rect.w = remainder*req->src_rect.w / req->dst_rect.h;
		req->dst_rect.h = remainder;
	}
	enable_mdp_irq(mdp, DL0_ROI_DONE);
	ret = mdp_ppp_blit(mdp, req, src_file, src_start, src_len, dst_file,
			   dst_start,
			   dst_len);
	if (ret)
		goto err_bad_blit;
	ret = mdp_ppp_wait(mdp);
	if (ret)
		goto err_wait_failed;
end:
	put_img(src_file);
	put_img(dst_file);
	mutex_unlock(&mdp_mutex);
	return 0;
err_bad_blit:
	disable_mdp_irq(mdp, DL0_ROI_DONE);
err_wait_failed:
	put_img(src_file);
	put_img(dst_file);
	mutex_unlock(&mdp_mutex);
	return ret;
}

void mdp_set_grp_disp(struct mdp_device *mdp_dev, unsigned disp_id)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);

	disp_id &= 0xf;
	mdp_writel(mdp, disp_id, MDP_FULL_BYPASS_WORD43);
}

int register_mdp_client(struct class_interface *cint)
{
	if (!mdp_class) {
		pr_err("mdp: no mdp_class when registering mdp client\n");
		return -ENODEV;
	}
	cint->class = mdp_class;
	return class_interface_register(cint);
}

#include "mdp_csc_table.h"
#include "mdp_scale_tables.h"

int mdp_probe(struct platform_device *pdev)
{
	struct resource *resource;
	int ret;
	int n;
	struct mdp_info *mdp;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		pr_err("mdp: can not get mdp mem resource!\n");
		return -ENOMEM;
	}

	mdp = kzalloc(sizeof(struct mdp_info), GFP_KERNEL);
	if (!mdp)
		return -ENOMEM;

	mdp->irq = platform_get_irq(pdev, 0);
	if (mdp->irq < 0) {
		pr_err("mdp: can not get mdp irq\n");
		ret = mdp->irq;
		goto error_get_irq;
	}

	mdp->base = ioremap(resource->start,
			    resource->end - resource->start);
	if (mdp->base == 0) {
		printk(KERN_ERR "msmfb: cannot allocate mdp regs!\n");
		ret = -ENOMEM;
		goto error_ioremap;
	}

	mdp->mdp_dev.dma = mdp_dma;
	mdp->mdp_dev.dma_wait = mdp_dma_wait;
	mdp->mdp_dev.blit = mdp_blit;
	mdp->mdp_dev.set_grp_disp = mdp_set_grp_disp;

	clk = clk_get(&pdev->dev, "mdp_clk");
	if (IS_ERR(clk)) {
		printk(KERN_INFO "mdp: failed to get mdp clk");
		return PTR_ERR(clk);
	}

	ret = request_irq(mdp->irq, mdp_isr, IRQF_DISABLED, "msm_mdp", mdp);
	if (ret)
		goto error_request_irq;
	disable_irq(mdp->irq);
	mdp_irq_mask = 0;

	/* debug interface write access */
	mdp_writel(mdp, 1, 0x60);

	mdp_writel(mdp, MDP_ANY_INTR_MASK, MDP_INTR_ENABLE);
	mdp_writel(mdp, 1, MDP_EBI2_PORTMAP_MODE);

	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01f8);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01fc);

	for (n = 0; n < ARRAY_SIZE(csc_table); n++)
		mdp_writel(mdp, csc_table[n].val, csc_table[n].reg);

	/* clear up unused fg/main registers */
	/* comp.plane 2&3 ystride */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0120);

	/* unpacked pattern */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x012c);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0130);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0134);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0158);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x015c);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0160);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0170);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0174);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x017c);

	/* comp.plane 2 & 3 */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0114);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0118);

	/* clear unused bg registers */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01c8);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01d0);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01dc);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01e0);
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x01e4);

	for (n = 0; n < ARRAY_SIZE(mdp_upscale_table); n++)
		mdp_writel(mdp, mdp_upscale_table[n].val,
		       mdp_upscale_table[n].reg);

	for (n = 0; n < 9; n++)
		mdp_writel(mdp, mdp_default_ccs[n], 0x40440 + 4 * n);
	mdp_writel(mdp, mdp_default_ccs[9], 0x40500 + 4 * 0);
	mdp_writel(mdp, mdp_default_ccs[10], 0x40500 + 4 * 0);
	mdp_writel(mdp, mdp_default_ccs[11], 0x40500 + 4 * 0);

	/* register mdp device */
	mdp->mdp_dev.dev.parent = &pdev->dev;
	mdp->mdp_dev.dev.class = mdp_class;
	snprintf(mdp->mdp_dev.dev.bus_id, BUS_ID_SIZE, "mdp%d", pdev->id);

	/* if you can remove the platform device you'd have to implement
	 * this:
	mdp_dev.release = mdp_class; */

	ret = device_register(&mdp->mdp_dev.dev);
	if (ret)
		goto error_device_register;
	return 0;

error_device_register:
	free_irq(mdp->irq, mdp);
error_request_irq:
	iounmap(mdp->base);
error_get_irq:
error_ioremap:
	kfree(mdp);
	return ret;
}

static struct platform_driver msm_mdp_driver = {
	.probe = mdp_probe,
	.driver = {.name = "msm_mdp"},
};

static int __init mdp_init(void)
{
	mdp_class = class_create(THIS_MODULE, "msm_mdp");
	if (IS_ERR(mdp_class)) {
		printk(KERN_ERR "Error creating mdp class\n");
		return PTR_ERR(mdp_class);
	}
	return platform_driver_register(&msm_mdp_driver);
}

subsys_initcall(mdp_init);
