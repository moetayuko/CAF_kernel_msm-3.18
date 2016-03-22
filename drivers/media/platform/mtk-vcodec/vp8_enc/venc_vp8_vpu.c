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

#include "mtk_vpu.h"

#include "venc_vp8_if.h"
#include "venc_vp8_vpu.h"
#include "venc_ipi_msg.h"

static void handle_vp8_enc_init_msg(struct venc_vp8_inst *inst, void *data)
{
	struct venc_vpu_ipi_msg_init *msg = data;

	inst->vpu_inst.id = msg->vpu_inst_addr;
	inst->vpu_inst.vsi = (struct venc_vp8_vsi *)
		vpu_mapping_dm_addr(inst->dev, msg->vpu_inst_addr);
}

static void handle_vp8_enc_encode_msg(struct venc_vp8_inst *inst, void *data)
{
	struct venc_vpu_ipi_msg_enc *msg = data;

	inst->vpu_inst.state = msg->state;
	inst->is_key_frm = msg->is_key_frm;
}

static void vp8_enc_vpu_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct venc_vpu_ipi_msg_common *msg = data;
	struct venc_vp8_inst *inst = (struct venc_vp8_inst *)msg->venc_inst;

	mtk_vcodec_debug(inst, "->msg_id=%x inst=%p status=%d",
			 msg->msg_id, inst, msg->status);

	switch (msg->msg_id) {
	case VPU_IPIMSG_ENC_INIT_DONE:
		handle_vp8_enc_init_msg(inst, data);
		break;
	case VPU_IPIMSG_ENC_SET_PARAM_DONE:
		break;
	case VPU_IPIMSG_ENC_ENCODE_DONE:
		handle_vp8_enc_encode_msg(inst, data);
		break;
	case VPU_IPIMSG_ENC_DEINIT_DONE:
		break;
	default:
		mtk_vcodec_err(inst, "unknown msg id=%x", msg->msg_id);
		break;
	}

	inst->vpu_inst.signaled = 1;
	inst->vpu_inst.failure = (msg->status != VENC_IPI_MSG_STATUS_OK);

	mtk_vcodec_debug_leave(inst);
}

static int vp8_enc_vpu_send_msg(struct venc_vp8_inst *inst, void *msg,
				int len)
{
	int status;

	mtk_vcodec_debug_enter(inst);

	status = vpu_ipi_send(inst->dev, IPI_VENC_VP8, (void *)msg, len);
	if (status) {
		uint32_t msg_id = *(uint32_t *)msg;

		mtk_vcodec_err(inst,
			       "vpu_ipi_send msg_id=%x len=%d failed status=%d",
			       msg_id, len, status);
		return -EINVAL;
	}
	if (inst->vpu_inst.failure)
		return -EINVAL;

	mtk_vcodec_debug_leave(inst);

	return 0;
}

int vp8_enc_vpu_init(struct venc_vp8_inst *inst)
{
	int status;
	struct venc_ap_ipi_msg_init out;

	mtk_vcodec_debug_enter(inst);

	init_waitqueue_head(&inst->vpu_inst.wq_hd);
	inst->vpu_inst.signaled = 0;
	inst->vpu_inst.failure = 0;

	status = vpu_ipi_register(inst->dev, IPI_VENC_VP8,
				  vp8_enc_vpu_ipi_handler,
				  "vp8_enc", NULL);
	if (status) {
		mtk_vcodec_err(inst,
			       "vpu_ipi_register failed status=%d", status);
		return -EINVAL;
	}

	out.msg_id = AP_IPIMSG_ENC_INIT;
	out.venc_inst = (unsigned long)inst;
	if (vp8_enc_vpu_send_msg(inst, &out, sizeof(out))) {
		mtk_vcodec_err(inst, "AP_IPIMSG_ENC_INIT failed");
		return -EINVAL;
	}

	mtk_vcodec_debug_leave(inst);

	return 0;
}

int vp8_enc_vpu_set_param(struct venc_vp8_inst *inst,
			  enum venc_set_param_type id,
			  struct venc_enc_prm *enc_param)
{
	struct venc_ap_ipi_msg_set_param out;

	mtk_vcodec_debug_enter(inst);

	out.msg_id = AP_IPIMSG_ENC_SET_PARAM;
	out.vpu_inst_addr = inst->vpu_inst.id;
	out.param_id = id;
	switch (id) {
	case VENC_SET_PARAM_ENC: {
		inst->vpu_inst.vsi->config.input_fourcc =
			enc_param->input_fourcc;
		inst->vpu_inst.vsi->config.bitrate = enc_param->bitrate;
		inst->vpu_inst.vsi->config.pic_w = enc_param->width;
		inst->vpu_inst.vsi->config.pic_h = enc_param->height;
		inst->vpu_inst.vsi->config.buf_w = enc_param->buf_width;
		inst->vpu_inst.vsi->config.buf_h = enc_param->buf_height;
		inst->vpu_inst.vsi->config.gop_size = enc_param->gop_size;
		inst->vpu_inst.vsi->config.framerate = enc_param->frm_rate;
		inst->vpu_inst.vsi->config.ts_mode = inst->ts_mode;
		out.data_item = 0;
		break;
	}
	case VENC_SET_PARAM_FORCE_INTRA:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_ADJUST_BITRATE:
		out.data_item = 1;
		out.data[0] = enc_param->bitrate;
		break;
	case VENC_SET_PARAM_ADJUST_FRAMERATE:
		out.data_item = 1;
		out.data[0] = enc_param->frm_rate;
		break;
	case VENC_SET_PARAM_GOP_SIZE:
		out.data_item = 1;
		out.data[0] = enc_param->gop_size;
		break;
	default:
		mtk_vcodec_err(inst, "id not support:%d", id);
		return -EINVAL;
	}
	if (vp8_enc_vpu_send_msg(inst, &out, sizeof(out))) {
		mtk_vcodec_err(inst, "AP_IPIMSG_ENC_SET_PARAM failed");
		return -EINVAL;
	}

	mtk_vcodec_debug_leave(inst);

	return 0;
}

int vp8_enc_vpu_encode(struct venc_vp8_inst *inst,
		       struct venc_frm_buf *frm_buf,
		       struct mtk_vcodec_mem *bs_buf)
{
	struct venc_ap_ipi_msg_enc out;

	mtk_vcodec_debug_enter(inst);

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_ENCODE;
	out.vpu_inst_addr = inst->vpu_inst.id;
	if (frm_buf) {
		if ((frm_buf->fb_addr[0].dma_addr % 16 == 0) &&
		    (frm_buf->fb_addr[1].dma_addr % 16 == 0) &&
		    (frm_buf->fb_addr[2].dma_addr % 16 == 0)) {
			out.input_addr[0] = frm_buf->fb_addr[0].dma_addr;
			out.input_addr[1] = frm_buf->fb_addr[1].dma_addr;
			out.input_addr[2] = frm_buf->fb_addr[2].dma_addr;
		} else {
			mtk_vcodec_err(inst, "dma_addr not align to 16");
			return -EINVAL;
		}
	}
	if (bs_buf) {
		out.bs_addr = bs_buf->dma_addr;
		out.bs_size = bs_buf->size;
	}
	if (vp8_enc_vpu_send_msg(inst, &out, sizeof(out))) {
		mtk_vcodec_err(inst, "AP_IPIMSG_ENC_ENCODE failed");
		return -EINVAL;
	}

	mtk_vcodec_debug(inst, "state=%d key_frm=%d",
			 inst->vpu_inst.state, inst->is_key_frm);

	mtk_vcodec_debug_leave(inst);

	return 0;
}

int vp8_enc_vpu_deinit(struct venc_vp8_inst *inst)
{
	struct venc_ap_ipi_msg_deinit out;

	mtk_vcodec_debug_enter(inst);

	out.msg_id = AP_IPIMSG_ENC_DEINIT;
	out.vpu_inst_addr = inst->vpu_inst.id;
	if (vp8_enc_vpu_send_msg(inst, &out, sizeof(out))) {
		mtk_vcodec_err(inst, "AP_IPIMSG_ENC_DEINIT failed");
		return -EINVAL;
	}

	mtk_vcodec_debug_leave(inst);

	return 0;
}
