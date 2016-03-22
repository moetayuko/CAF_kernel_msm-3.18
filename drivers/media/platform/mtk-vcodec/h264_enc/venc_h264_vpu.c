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

#include "mtk_vpu.h"

#include "venc_h264_if.h"
#include "venc_h264_vpu.h"
#include "venc_ipi_msg.h"

static unsigned int h264_get_profile(struct venc_h264_inst *inst,
				     unsigned int profile)
{
	switch (profile) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
		return 66;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		return 77;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
		return 100;
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
		mtk_vcodec_err(inst, "unsupported CONSTRAINED_BASELINE");
		return 0;
	case V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED:
		mtk_vcodec_err(inst, "unsupported EXTENDED");
		return 0;
	default:
		mtk_vcodec_debug(inst, "unsupported profile %d", profile);
		return 100;
	}
}

static unsigned int h264_get_level(struct venc_h264_inst *inst,
				   unsigned int level)
{
	switch (level) {
	case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
		mtk_vcodec_err(inst, "unsupported 1B");
		return 0;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
		return 10;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
		return 11;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
		return 12;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
		return 13;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
		return 20;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
		return 21;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
		return 22;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
		return 30;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
		return 31;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
		return 32;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
		return 40;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
		return 41;
	default:
		mtk_vcodec_debug(inst, "unsupported level %d", level);
		return 31;
	}
}

static void handle_h264_enc_init_msg(struct venc_h264_inst *inst, void *data)
{
	struct venc_vpu_ipi_msg_init *msg = data;

	inst->vpu_inst.id = msg->vpu_inst_addr;
	inst->vpu_inst.vsi = (struct venc_h264_vsi *)vpu_mapping_dm_addr(
		inst->dev, msg->vpu_inst_addr);
}

static void handle_h264_enc_encode_msg(struct venc_h264_inst *inst, void *data)
{
	struct venc_vpu_ipi_msg_enc *msg = data;

	inst->vpu_inst.state = msg->state;
	inst->vpu_inst.bs_size = msg->bs_size;
	inst->is_key_frm = msg->is_key_frm;
}

static void h264_enc_vpu_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct venc_vpu_ipi_msg_common *msg = data;
	struct venc_h264_inst *inst = (struct venc_h264_inst *)msg->venc_inst;

	mtk_vcodec_debug(inst, "msg_id %x inst %p status %d",
			 msg->msg_id, inst, msg->status);

	switch (msg->msg_id) {
	case VPU_IPIMSG_ENC_INIT_DONE:
		handle_h264_enc_init_msg(inst, data);
		break;
	case VPU_IPIMSG_ENC_SET_PARAM_DONE:
		break;
	case VPU_IPIMSG_ENC_ENCODE_DONE:
		handle_h264_enc_encode_msg(inst, data);
		break;
	case VPU_IPIMSG_ENC_DEINIT_DONE:
		break;
	default:
		mtk_vcodec_err(inst, "unknown msg id %x", msg->msg_id);
		break;
	}

	inst->vpu_inst.signaled = 1;
	inst->vpu_inst.failure = (msg->status != VENC_IPI_MSG_STATUS_OK);

	mtk_vcodec_debug_leave(inst);
}

static int h264_enc_vpu_send_msg(struct venc_h264_inst *inst, void *msg,
				 int len)
{
	int status;

	mtk_vcodec_debug_enter(inst);

	status = vpu_ipi_send(inst->dev, IPI_VENC_H264, msg, len);
	if (status) {
		uint32_t msg_id = *(uint32_t *)msg;

		mtk_vcodec_err(inst, "vpu_ipi_send msg_id %x len %d fail %d",
			       msg_id, len, status);
		return -EINVAL;
	}
	if (inst->vpu_inst.failure)
		return -EINVAL;

	mtk_vcodec_debug_leave(inst);

	return 0;
}

int h264_enc_vpu_init(struct venc_h264_inst *inst)
{
	int status;
	struct venc_ap_ipi_msg_init out;

	mtk_vcodec_debug_enter(inst);

	init_waitqueue_head(&inst->vpu_inst.wq_hd);
	inst->vpu_inst.signaled = 0;
	inst->vpu_inst.failure = 0;

	status = vpu_ipi_register(inst->dev, IPI_VENC_H264,
				  h264_enc_vpu_ipi_handler,
				  "h264_enc", NULL);
	if (status) {
		mtk_vcodec_err(inst, "vpu_ipi_register fail %d", status);
		return -EINVAL;
	}

	out.msg_id = AP_IPIMSG_ENC_INIT;
	out.venc_inst = (unsigned long)inst;
	if (h264_enc_vpu_send_msg(inst, &out, sizeof(out))) {
		mtk_vcodec_err(inst, "AP_IPIMSG_ENC_INIT fail");
		return -EINVAL;
	}

	mtk_vcodec_debug_leave(inst);

	return 0;
}

int h264_enc_vpu_set_param(struct venc_h264_inst *inst,
			   enum venc_set_param_type id,
			   struct venc_enc_prm *enc_param)
{
	struct venc_ap_ipi_msg_set_param out;

	mtk_vcodec_debug(inst, "id %d ->", id);

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
		inst->vpu_inst.vsi->config.intra_period =
			enc_param->intra_period;
		inst->vpu_inst.vsi->config.framerate = enc_param->frm_rate;
		inst->vpu_inst.vsi->config.profile =
			h264_get_profile(inst, enc_param->h264_profile);
		inst->vpu_inst.vsi->config.level =
			h264_get_level(inst, enc_param->h264_level);
		inst->vpu_inst.vsi->config.wfd = 0;
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
	case VENC_SET_PARAM_INTRA_PERIOD:
		out.data_item = 1;
		out.data[0] = enc_param->intra_period;
		break;
	case VENC_SET_PARAM_SKIP_FRAME:
		out.data_item = 0;
		break;
	default:
		mtk_vcodec_err(inst, "id %d not supported", id);
		return -EINVAL;
	}
	if (h264_enc_vpu_send_msg(inst, &out, sizeof(out))) {
		mtk_vcodec_err(inst,
			       "AP_IPIMSG_ENC_SET_PARAM %d fail", id);
		return -EINVAL;
	}

	mtk_vcodec_debug(inst, "id %d <-", id);

	return 0;
}

int h264_enc_vpu_encode(struct venc_h264_inst *inst, unsigned int bs_mode,
			struct venc_frm_buf *frm_buf,
			struct mtk_vcodec_mem *bs_buf,
			unsigned int *bs_size)
{
	struct venc_ap_ipi_msg_enc out;

	mtk_vcodec_debug(inst, "bs_mode %d ->", bs_mode);

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_ENCODE;
	out.vpu_inst_addr = inst->vpu_inst.id;
	out.bs_mode = bs_mode;
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
	if (h264_enc_vpu_send_msg(inst, &out, sizeof(out))) {
		mtk_vcodec_err(inst, "AP_IPIMSG_ENC_ENCODE %d fail",
			       bs_mode);
		return -EINVAL;
	}

	mtk_vcodec_debug(inst, "state %d size %d key_frm %d",
			 inst->vpu_inst.state, inst->vpu_inst.bs_size,
			 inst->is_key_frm);

	inst->vpu_inst.wait_int = 1;
	if (inst->vpu_inst.state == VEN_IPI_MSG_ENC_STATE_SKIP) {
		*bs_size = inst->vpu_inst.bs_size;
		memcpy(bs_buf->va,
		       inst->work_bufs[VENC_H264_VPU_WORK_BUF_SKIP_FRAME].va,
		       *bs_size);
		inst->vpu_inst.wait_int = 0;
	}

	mtk_vcodec_debug(inst, "bs_mode %d <-", bs_mode);

	return 0;
}

int h264_enc_vpu_deinit(struct venc_h264_inst *inst)
{
	struct venc_ap_ipi_msg_deinit out;

	mtk_vcodec_debug_enter(inst);

	out.msg_id = AP_IPIMSG_ENC_DEINIT;
	out.vpu_inst_addr = inst->vpu_inst.id;
	if (h264_enc_vpu_send_msg(inst, &out, sizeof(out))) {
		mtk_vcodec_err(inst, "AP_IPIMSG_ENC_DEINIT fail");
		return -EINVAL;
	}

	mtk_vcodec_debug_leave(inst);

	return 0;
}
