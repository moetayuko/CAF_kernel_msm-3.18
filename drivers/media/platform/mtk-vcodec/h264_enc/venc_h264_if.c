/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Jungchang Tsao <jungchang.tsao@mediatek.com>
 *         Daniel Hsiao <daniel.hsiao@mediatek.com>
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

#include "venc_h264_if.h"
#include "venc_h264_vpu.h"

static const char h264_filler_marker[] = {0x0, 0x0, 0x0, 0x1, 0xc};

#define H264_FILLER_MARKER_SIZE ARRAY_SIZE(h264_filler_marker)
#define VENC_PIC_BITSTREAM_BYTE_CNT 0x0098

static inline void h264_write_reg(struct venc_h264_inst *inst, u32 addr,
				  u32 val)
{
	writel(val, inst->hw_base + addr);
}

static inline u32 h264_read_reg(struct venc_h264_inst *inst, u32 addr)
{
	return readl(inst->hw_base + addr);
}

static void h264_enc_free_work_buf(struct venc_h264_inst *inst)
{
	int i;

	mtk_vcodec_debug_enter(inst);

	/* Except the SKIP_FRAME buffers,
	 * other buffers need to be freed by AP.
	 */
	for (i = 0; i < VENC_H264_VPU_WORK_BUF_MAX; i++) {
		if (i != VENC_H264_VPU_WORK_BUF_SKIP_FRAME)
			mtk_vcodec_mem_free(inst->ctx, &inst->work_bufs[i]);
	}

	mtk_vcodec_mem_free(inst->ctx, &inst->pps_buf);

	mtk_vcodec_debug_leave(inst);
}

static int h264_enc_alloc_work_buf(struct venc_h264_inst *inst)
{
	int i;
	int ret = 0;
	struct venc_h264_vpu_buf *wb = inst->vpu_inst.vsi->work_bufs;

	mtk_vcodec_debug_enter(inst);

	for (i = 0; i < VENC_H264_VPU_WORK_BUF_MAX; i++) {
		/*
		 * This 'wb' structure is set by VPU side and shared to AP for
		 * buffer allocation and IO virtual addr mapping. For most of
		 * the buffers, AP will allocate the buffer according to 'size'
		 * field and store the IO virtual addr in 'iova' field. There
		 * are two exceptions:
		 * (1) RC_CODE buffer, it's pre-allocated in the VPU side, and
		 * save the VPU addr in the 'vpua' field. The AP will translate
		 * the VPU addr to the corresponding IO virtual addr and store
		 * in 'iova' field for reg setting in VPU side.
		 * (2) SKIP_FRAME buffer, it's pre-allocated in the VPU side,
		 * and save the VPU addr in the 'vpua' field. The AP will
		 * translate the VPU addr to the corresponding AP side virtual
		 * address and do some memcpy access to move to bitstream buffer
		 * assigned by v4l2 layer.
		 */
		inst->work_bufs[i].size = wb[i].size;
		if (i == VENC_H264_VPU_WORK_BUF_SKIP_FRAME) {
			inst->work_bufs[i].va = vpu_mapping_dm_addr(
				inst->dev, wb[i].vpua);
			inst->work_bufs[i].dma_addr = 0;
		} else {
			ret = mtk_vcodec_mem_alloc(inst->ctx,
						   &inst->work_bufs[i]);
			if (ret) {
				mtk_vcodec_err(inst,
					       "cannot allocate buf %d", i);
				goto err_alloc;
			}
			/*
			 * This RC_CODE is pre-allocated by VPU and saved in VPU
			 * addr. So we need use memcpy to copy RC_CODE from VPU
			 * addr into IO virtual addr in 'iova' field for reg
			 * setting in VPU side.
			 */
			if (i == VENC_H264_VPU_WORK_BUF_RC_CODE) {
				void *tmp_va;

				tmp_va = vpu_mapping_dm_addr(inst->dev,
							     wb[i].vpua);
				memcpy(inst->work_bufs[i].va, tmp_va,
				       wb[i].size);
			}
		}
		wb[i].iova = inst->work_bufs[i].dma_addr;

		mtk_vcodec_debug(inst,
				 "work_buf[%d] va=0x%p iova=0x%p size=0x%lx",
				 i, inst->work_bufs[i].va,
				 (void *)inst->work_bufs[i].dma_addr,
				 inst->work_bufs[i].size);
	}

	/* the pps_buf is used by AP side only */
	inst->pps_buf.size = 128;
	ret = mtk_vcodec_mem_alloc(inst->ctx, &inst->pps_buf);
	if (ret) {
		mtk_vcodec_err(inst, "cannot allocate pps_buf");
		goto err_alloc;
	}

	mtk_vcodec_debug_leave(inst);

	return ret;

err_alloc:
	h264_enc_free_work_buf(inst);

	return ret;
}

static unsigned int h264_enc_wait_venc_done(struct venc_h264_inst *inst)
{
	unsigned int irq_status = 0;
	struct mtk_vcodec_ctx *ctx = (struct mtk_vcodec_ctx *)inst->ctx;

	if (!mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					  WAIT_INTR_TIMEOUT_MS)) {
		irq_status = ctx->irq_status;
		mtk_vcodec_debug(inst, "irq_status %x <-", irq_status);
	}
	return irq_status;
}

static int h264_encode_sps(struct venc_h264_inst *inst,
			   struct mtk_vcodec_mem *bs_buf,
			   unsigned int *bs_size)
{
	int ret = 0;
	unsigned int irq_status;

	mtk_vcodec_debug_enter(inst);

	ret = h264_enc_vpu_encode(inst, H264_BS_MODE_SPS, NULL,
				  bs_buf, bs_size);
	if (ret)
		return ret;

	irq_status = h264_enc_wait_venc_done(inst);
	if (irq_status != MTK_VENC_IRQ_STATUS_SPS) {
		mtk_vcodec_err(inst, "expect irq status %d",
			       MTK_VENC_IRQ_STATUS_SPS);
		return -EINVAL;
	}

	*bs_size = h264_read_reg(inst, VENC_PIC_BITSTREAM_BYTE_CNT);
	mtk_vcodec_debug(inst, "bs size %d <-", *bs_size);

	return ret;
}

static int h264_encode_pps(struct venc_h264_inst *inst,
			   struct mtk_vcodec_mem *bs_buf,
			   unsigned int *bs_size)
{
	int ret = 0;
	unsigned int irq_status;

	mtk_vcodec_debug_enter(inst);

	ret = h264_enc_vpu_encode(inst, H264_BS_MODE_PPS, NULL,
				  bs_buf, bs_size);
	if (ret)
		return ret;

	irq_status = h264_enc_wait_venc_done(inst);
	if (irq_status != MTK_VENC_IRQ_STATUS_PPS) {
		mtk_vcodec_err(inst, "expect irq status %d",
			       MTK_VENC_IRQ_STATUS_PPS);
		return -EINVAL;
	}

	*bs_size = h264_read_reg(inst, VENC_PIC_BITSTREAM_BYTE_CNT);
	mtk_vcodec_debug(inst, "bs size %d <-", *bs_size);

	return ret;
}

static int h264_encode_header(struct venc_h264_inst *inst,
			      struct mtk_vcodec_mem *bs_buf,
			      unsigned int *bs_size)
{
	int ret = 0;
	unsigned int bs_size_sps;
	unsigned int bs_size_pps;

	ret = h264_encode_sps(inst, bs_buf, &bs_size_sps);
	if (ret)
		return ret;

	ret = h264_encode_pps(inst, &inst->pps_buf, &bs_size_pps);
	if (ret)
		return ret;

	memcpy(bs_buf->va + bs_size_sps, inst->pps_buf.va, bs_size_pps);
	*bs_size = bs_size_sps + bs_size_pps;

	return ret;
}

static int h264_encode_frame(struct venc_h264_inst *inst,
			     struct venc_frm_buf *frm_buf,
			     struct mtk_vcodec_mem *bs_buf,
			     unsigned int *bs_size)
{
	int ret = 0;
	unsigned int irq_status;

	mtk_vcodec_debug_enter(inst);

	ret = h264_enc_vpu_encode(inst, H264_BS_MODE_FRAME, frm_buf,
				  bs_buf, bs_size);
	if (ret)
		return ret;

	/*
	 * skip frame case: The skip frame buffer is composed by vpu side only,
	 * it does not trigger the hw, so skip the wait interrupt operation.
	 */
	if (!inst->vpu_inst.wait_int) {
		++inst->frm_cnt;
		return ret;
	}

	irq_status = h264_enc_wait_venc_done(inst);
	if (irq_status != MTK_VENC_IRQ_STATUS_FRM) {
		mtk_vcodec_err(inst, "irq_status=%d failed", irq_status);
		return -EIO;
	}

	*bs_size = h264_read_reg(inst, VENC_PIC_BITSTREAM_BYTE_CNT);

	++inst->frm_cnt;
	mtk_vcodec_debug(inst, "frm %d bs size %d key_frm %d <-",
			 inst->frm_cnt,
			 *bs_size, inst->is_key_frm);

	return ret;
}

static void h264_encode_filler(struct venc_h264_inst *inst, void *buf,
			       int size)
{
	unsigned char *p = buf;

	if (size < H264_FILLER_MARKER_SIZE) {
		mtk_vcodec_err(inst, "filler size too small %d", size);
		return;
	}

	memcpy(p, h264_filler_marker, ARRAY_SIZE(h264_filler_marker));
	size -= H264_FILLER_MARKER_SIZE;
	p += H264_FILLER_MARKER_SIZE;
	memset(p, 0xff, size);
}

static int h264_enc_init(struct mtk_vcodec_ctx *ctx, unsigned long *handle)
{
	int ret = 0;
	struct venc_h264_inst *inst;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;
	inst->dev = mtk_vcodec_get_plat_dev(ctx);
	inst->hw_base = mtk_vcodec_get_reg_addr(inst->ctx, VENC_SYS);

	mtk_vcodec_debug_enter(inst);

	ret = h264_enc_vpu_init(inst);

	mtk_vcodec_debug_leave(inst);

	if (ret)
		kfree(inst);
	else
		(*handle) = (unsigned long)inst;

	return ret;
}

static int h264_enc_encode(unsigned long handle,
			   enum venc_start_opt opt,
			   struct venc_frm_buf *frm_buf,
			   struct mtk_vcodec_mem *bs_buf,
			   struct venc_done_result *result)
{
	int ret = 0;
	struct venc_h264_inst *inst = (struct venc_h264_inst *)handle;
	struct mtk_vcodec_ctx *ctx = inst->ctx;

	mtk_vcodec_debug(inst, "opt %d ->", opt);

	enable_irq(ctx->dev->enc_irq);

	switch (opt) {
	case VENC_START_OPT_ENCODE_SEQUENCE_HEADER: {
		unsigned int bs_size_hdr;

		ret = h264_encode_header(inst, bs_buf, &bs_size_hdr);
		if (ret)
			goto encode_err;

		result->bs_size = bs_size_hdr;
		result->is_key_frm = false;
		break;
	}

	case VENC_START_OPT_ENCODE_FRAME: {
		int hdr_sz;
		int hdr_sz_ext;
		int filler_sz = 0;
		const int bs_alignment = 128;
		struct mtk_vcodec_mem tmp_bs_buf;
		unsigned int bs_size_hdr;
		unsigned int bs_size_frm;

		if (!inst->prepend_hdr) {
			ret = h264_encode_frame(inst, frm_buf, bs_buf,
						&result->bs_size);
			if (ret)
				goto encode_err;
			result->is_key_frm = inst->is_key_frm;
			break;
		}

		mtk_vcodec_debug(inst, "h264_encode_frame prepend SPS/PPS");

		ret = h264_encode_header(inst, bs_buf, &bs_size_hdr);
		if (ret)
			goto encode_err;

		hdr_sz = bs_size_hdr;
		hdr_sz_ext = (hdr_sz & (bs_alignment - 1));
		if (hdr_sz_ext) {
			filler_sz = bs_alignment - hdr_sz_ext;
			if (hdr_sz_ext + H264_FILLER_MARKER_SIZE > bs_alignment)
				filler_sz += bs_alignment;
			h264_encode_filler(inst, bs_buf->va + hdr_sz,
					   filler_sz);
		}

		tmp_bs_buf.va = bs_buf->va + hdr_sz + filler_sz;
		tmp_bs_buf.dma_addr = bs_buf->dma_addr + hdr_sz + filler_sz;
		tmp_bs_buf.size = bs_buf->size - (hdr_sz + filler_sz);

		ret = h264_encode_frame(inst, frm_buf, &tmp_bs_buf,
					&bs_size_frm);
		if (ret)
			goto encode_err;

		result->bs_size = hdr_sz + filler_sz + bs_size_frm;

		mtk_vcodec_debug(inst, "hdr %d filler %d frame %d bs %d",
				 hdr_sz, filler_sz, bs_size_frm,
				 result->bs_size);

		inst->prepend_hdr = 0;
		result->is_key_frm = inst->is_key_frm;
		break;
	}

	default:
		mtk_vcodec_err(inst, "venc_start_opt %d not supported", opt);
		ret = -EINVAL;
		break;
	}

encode_err:

	disable_irq(ctx->dev->enc_irq);
	mtk_vcodec_debug(inst, "opt %d <-", opt);

	return ret;
}

static int h264_enc_set_param(unsigned long handle,
			      enum venc_set_param_type type,
			      struct venc_enc_prm *enc_prm)
{
	int i;
	int ret = 0;
	struct venc_h264_inst *inst = (struct venc_h264_inst *)handle;

	mtk_vcodec_debug(inst, "->type=%d", type);

	switch (type) {
	case VENC_SET_PARAM_ENC:
		ret = h264_enc_vpu_set_param(inst, type, enc_prm);
		if (ret)
			break;
		if (inst->work_buf_allocated) {
			h264_enc_free_work_buf(inst);
			inst->work_buf_allocated = false;
		}
		ret = h264_enc_alloc_work_buf(inst);
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

	case VENC_SET_PARAM_PREPEND_HEADER:
		inst->prepend_hdr = 1;
		mtk_vcodec_debug(inst, "set prepend header mode");
		break;

	default:
		ret = h264_enc_vpu_set_param(inst, type, enc_prm);
		break;
	}

	mtk_vcodec_debug_leave(inst);

	return ret;
}

static int h264_enc_deinit(unsigned long handle)
{
	int ret = 0;
	struct venc_h264_inst *inst = (struct venc_h264_inst *)handle;

	mtk_vcodec_debug_enter(inst);

	ret = h264_enc_vpu_deinit(inst);

	if (inst->work_buf_allocated)
		h264_enc_free_work_buf(inst);

	mtk_vcodec_debug_leave(inst);
	kfree(inst);

	return ret;
}

static struct venc_common_if venc_h264_if = {
	h264_enc_init,
	h264_enc_encode,
	h264_enc_set_param,
	h264_enc_deinit,
};

struct venc_common_if *get_h264_enc_comm_if(void)
{
	return &venc_h264_if;
}
