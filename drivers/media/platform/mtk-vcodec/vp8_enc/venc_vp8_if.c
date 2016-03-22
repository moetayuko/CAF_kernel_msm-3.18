/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Daniel Hsiao <daniel.hsiao@mediatek.com>
 *         PoChun Lin <pochun.lin@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "mtk_vcodec_drv.h"
#include "mtk_vcodec_util.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_enc.h"
#include "mtk_vcodec_enc_pm.h"
#include "mtk_vpu.h"

#include "venc_vp8_if.h"
#include "venc_vp8_vpu.h"

#define VENC_BITSTREAM_FRAME_SIZE 0x0098
#define VENC_BITSTREAM_HEADER_LEN 0x00e8

/* This ac_tag is vp8 frame tag. */
#define MAX_AC_TAG_SIZE 10

static inline void vp8_enc_write_reg(struct venc_vp8_inst *inst, u32 addr,
				     u32 val)
{
	writel(val, inst->hw_base + addr);
}

static inline u32 vp8_enc_read_reg(struct venc_vp8_inst *inst, u32 addr)
{
	return readl(inst->hw_base + addr);
}

static void vp8_enc_free_work_buf(struct venc_vp8_inst *inst)
{
	int i;

	mtk_vcodec_debug_enter(inst);

	/* Buffers need to be freed by AP. */
	for (i = 0; i < VENC_VP8_VPU_WORK_BUF_MAX; i++) {
		if ((inst->work_bufs[i].size == 0))
			continue;
		mtk_vcodec_mem_free(inst->ctx, &inst->work_bufs[i]);
	}

	mtk_vcodec_debug_leave(inst);
}

static int vp8_enc_alloc_work_buf(struct venc_vp8_inst *inst)
{
	int i;
	int ret = 0;
	struct venc_vp8_vpu_buf *wb = inst->vpu_inst.vsi->work_bufs;

	mtk_vcodec_debug_enter(inst);

	for (i = 0; i < VENC_VP8_VPU_WORK_BUF_MAX; i++) {
		if ((wb[i].size == 0))
			continue;
		/*
		 * This 'wb' structure is set by VPU side and shared to AP for
		 * buffer allocation and IO virtual addr mapping. For most of
		 * the buffers, AP will allocate the buffer according to 'size'
		 * field and store the IO virtual addr in 'iova' field. For the
		 * RC_CODEx buffers, they are pre-allocated in the VPU side
		 * because they are inside VPU SRAM, and save the VPU addr in
		 * the 'vpua' field. The AP will translate the VPU addr to the
		 * corresponding IO virtual addr and store in 'iova' field.
		 */
		inst->work_bufs[i].size = wb[i].size;
		ret = mtk_vcodec_mem_alloc(inst->ctx, &inst->work_bufs[i]);
		if (ret) {
			mtk_vcodec_err(inst,
				       "cannot alloc work_bufs[%d]", i);
			goto err_alloc;
		}
		/*
		 * This RC_CODEx is pre-allocated by VPU and saved in VPU addr.
		 * So we need use memcpy to copy RC_CODEx from VPU addr into IO
		 * virtual addr in 'iova' field for reg setting in VPU side.
		 */
		if (i == VENC_VP8_VPU_WORK_BUF_RC_CODE ||
		    i == VENC_VP8_VPU_WORK_BUF_RC_CODE2 ||
		    i == VENC_VP8_VPU_WORK_BUF_RC_CODE3) {
			void *tmp_va;

			tmp_va = vpu_mapping_dm_addr(inst->dev, wb[i].vpua);
			memcpy(inst->work_bufs[i].va, tmp_va, wb[i].size);
		}
		wb[i].iova = inst->work_bufs[i].dma_addr;

		mtk_vcodec_debug(inst,
				 "work_bufs[%d] va=0x%p,iova=0x%p,size=0x%lx",
				 i, inst->work_bufs[i].va,
				 (void *)inst->work_bufs[i].dma_addr,
				 inst->work_bufs[i].size);
	}

	mtk_vcodec_debug_leave(inst);

	return ret;

err_alloc:
	vp8_enc_free_work_buf(inst);

	return ret;
}

static unsigned int vp8_enc_wait_venc_done(struct venc_vp8_inst *inst)
{
	unsigned int irq_status = 0;
	struct mtk_vcodec_ctx *ctx = (struct mtk_vcodec_ctx *)inst->ctx;

	if (!mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					  WAIT_INTR_TIMEOUT_MS)) {
		irq_status = ctx->irq_status;
		mtk_vcodec_debug(inst, "isr return %x", irq_status);
	}
	return irq_status;
}

/*
 * Compose ac_tag, bitstream header and bitstream payload into
 * one bitstream buffer.
 */
static int vp8_enc_compose_one_frame(struct venc_vp8_inst *inst,
				     struct mtk_vcodec_mem *bs_buf,
				     unsigned int *bs_size)
{
	unsigned int not_key;
	u32 bs_frm_size;
	u32 bs_hdr_len;
	unsigned int ac_tag_size;
	u8 ac_tag[MAX_AC_TAG_SIZE];

	bs_frm_size = vp8_enc_read_reg(inst, VENC_BITSTREAM_FRAME_SIZE);
	bs_hdr_len = vp8_enc_read_reg(inst, VENC_BITSTREAM_HEADER_LEN);

	/* if a frame is key frame, not_key is 0 */
	not_key = !inst->is_key_frm;
	*(u32 *)ac_tag = __cpu_to_le32((bs_hdr_len << 5) | 0x10 | not_key);
	/* key frame */
	if (not_key == 0) {
		ac_tag_size = MAX_AC_TAG_SIZE;
		ac_tag[3] = 0x9d;
		ac_tag[4] = 0x01;
		ac_tag[5] = 0x2a;
		ac_tag[6] = inst->vpu_inst.vsi->config.pic_w;
		ac_tag[7] = inst->vpu_inst.vsi->config.pic_w >> 8;
		ac_tag[8] = inst->vpu_inst.vsi->config.pic_h;
		ac_tag[9] = inst->vpu_inst.vsi->config.pic_h >> 8;
	} else {
		ac_tag_size = 3;
	}

	if (bs_buf->size < bs_hdr_len + bs_frm_size + ac_tag_size) {
		mtk_vcodec_err(inst, "bitstream buf size is too small(%ld)",
			       bs_buf->size);
		return -EINVAL;
	}

	/*
	* (1) The vp8 bitstream header and body are generated by the HW vp8
	* encoder separately at the same time. We cannot know the bitstream
	* header length in advance.
	* (2) From the vp8 spec, there is no stuffing byte allowed between the
	* ac tag, bitstream header and bitstream body.
	*/
	memmove(bs_buf->va + bs_hdr_len + ac_tag_size,
		bs_buf->va, bs_frm_size);
	memcpy(bs_buf->va + ac_tag_size,
	       inst->work_bufs[VENC_VP8_VPU_WORK_BUF_BS_HEADER].va,
	       bs_hdr_len);
	memcpy(bs_buf->va, ac_tag, ac_tag_size);
	*bs_size = bs_frm_size + bs_hdr_len + ac_tag_size;

	return 0;
}

static int vp8_enc_encode_frame(struct venc_vp8_inst *inst,
				struct venc_frm_buf *frm_buf,
				struct mtk_vcodec_mem *bs_buf,
				unsigned int *bs_size)
{
	int ret = 0;
	unsigned int irq_status;

	mtk_vcodec_debug(inst, "->frm_cnt=%d", inst->frm_cnt);

	ret = vp8_enc_vpu_encode(inst, frm_buf, bs_buf);
	if (ret)
		return ret;

	irq_status = vp8_enc_wait_venc_done(inst);
	if (irq_status != MTK_VENC_IRQ_STATUS_FRM) {
		mtk_vcodec_err(inst, "irq_status=%d failed", irq_status);
		return -EIO;
	}

	if (vp8_enc_compose_one_frame(inst, bs_buf, bs_size)) {
		mtk_vcodec_err(inst, "vp8_enc_compose_one_frame failed");
		return -EINVAL;
	}

	inst->frm_cnt++;
	mtk_vcodec_debug(inst, "<-size=%d key_frm=%d", *bs_size,
			 inst->is_key_frm);

	return ret;
}

static int vp8_enc_init(struct mtk_vcodec_ctx *ctx, unsigned long *handle)
{
	int ret = 0;
	struct venc_vp8_inst *inst;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;
	inst->dev = mtk_vcodec_get_plat_dev(ctx);
	inst->hw_base = mtk_vcodec_get_reg_addr(inst->ctx, VENC_LT_SYS);

	mtk_vcodec_debug_enter(inst);

	ret = vp8_enc_vpu_init(inst);

	mtk_vcodec_debug_leave(inst);

	if (ret)
		kfree(inst);
	else
		(*handle) = (unsigned long)inst;

	return ret;
}

static int vp8_enc_encode(unsigned long handle,
			  enum venc_start_opt opt,
			  struct venc_frm_buf *frm_buf,
			  struct mtk_vcodec_mem *bs_buf,
			  struct venc_done_result *result)
{
	int ret = 0;
	struct venc_vp8_inst *inst = (struct venc_vp8_inst *)handle;
	struct mtk_vcodec_ctx *ctx = inst->ctx;

	mtk_vcodec_debug_enter(inst);

	enable_irq(ctx->dev->enc_lt_irq);

	switch (opt) {
	case VENC_START_OPT_ENCODE_FRAME:
		ret = vp8_enc_encode_frame(inst, frm_buf, bs_buf,
					   &result->bs_size);
		if (ret)
			goto encode_err;
		result->is_key_frm = inst->is_key_frm;
		break;

	default:
		mtk_vcodec_err(inst, "opt not support:%d", opt);
		ret = -EINVAL;
		break;
	}

encode_err:

	disable_irq(ctx->dev->enc_lt_irq);
	mtk_vcodec_debug_leave(inst);

	return ret;
}

static int vp8_enc_set_param(unsigned long handle,
			     enum venc_set_param_type type,
			     struct venc_enc_prm *enc_prm)
{
	int i;
	int ret = 0;
	struct venc_vp8_inst *inst = (struct venc_vp8_inst *)handle;

	mtk_vcodec_debug(inst, "->type=%d", type);

	switch (type) {
	case VENC_SET_PARAM_ENC:
		ret = vp8_enc_vpu_set_param(inst, type, enc_prm);
		if (ret)
			break;
		if (inst->work_buf_allocated) {
			vp8_enc_free_work_buf(inst);
			inst->work_buf_allocated = false;
		}
		ret = vp8_enc_alloc_work_buf(inst);
		if (ret)
			break;
		inst->work_buf_allocated = true;
		for (i = 0; i < MTK_VCODEC_MAX_PLANES; i++) {
			enc_prm->sizeimage[i] =
				inst->vpu_inst.vsi->sizeimage[i];
			mtk_vcodec_debug(inst, "sizeimage[%d] size=0x%x", i,
					 enc_prm->sizeimage[i]);
		}
		break;

	/*
	 * VENC_SET_PARAM_TS_MODE must be called before VENC_SET_PARAM_ENC
	 */
	case VENC_SET_PARAM_TS_MODE:
		inst->ts_mode = 1;
		mtk_vcodec_debug(inst, "set ts_mode");
		break;

	default:
		ret = vp8_enc_vpu_set_param(inst, type, enc_prm);
		break;
	}

	mtk_vcodec_debug_leave(inst);

	return ret;
}

static int vp8_enc_deinit(unsigned long handle)
{
	int ret = 0;
	struct venc_vp8_inst *inst = (struct venc_vp8_inst *)handle;

	mtk_vcodec_debug_enter(inst);

	ret = vp8_enc_vpu_deinit(inst);

	if (inst->work_buf_allocated)
		vp8_enc_free_work_buf(inst);

	mtk_vcodec_debug_leave(inst);
	kfree(inst);

	return ret;
}

static struct venc_common_if venc_vp8_if = {
	vp8_enc_init,
	vp8_enc_encode,
	vp8_enc_set_param,
	vp8_enc_deinit,
};

struct venc_common_if *get_vp8_enc_comm_if(void)
{
	return &venc_vp8_if;
}
