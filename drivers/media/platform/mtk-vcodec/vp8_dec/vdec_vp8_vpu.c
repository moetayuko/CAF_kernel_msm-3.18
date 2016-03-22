/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: Jungchang Tsao <jungchang.tsao@mediatek.com>
 *	   PC Chen <pc.chen@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "mtk_vpu.h"
#include "mtk_vcodec_util.h"

#include "vdec_ipi_msg.h"

#include "vdec_vp8_if.h"
#include "vdec_vp8_vpu.h"

#define VDEC_VP8_WAIT_VPU_TIMEOUT_MS		(2000)

/**
 * struct vdec_vp8_ipi_dec_start - for AP_IPIMSG_DEC_START
 * @msg_id        : AP_IPIMSG_DEC_START
 * @vpu_inst_addr : VPU decoder instance addr
 * @data          : Header info
 * @reserved      : Reserved field
 */
struct vdec_vp8_ipi_dec_start {
	uint32_t msg_id;
	uint32_t vpu_inst_addr;
	uint32_t data;
	uint32_t reserved;
};

static void handle_init_ack_msg(struct vdec_vp8_inst *inst, void *data)
{
	struct vdec_vpu_ipi_init_ack *msg = data;

	inst->vpu.inst_addr = msg->vpu_inst_addr;
	inst->vsi = (struct vdec_vp8_vsi *)vpu_mapping_dm_addr(
		    inst->dev, msg->vpu_inst_addr);

	mtk_vcodec_debug(inst, "inst_addr %x map to %p\n",
			 inst->vpu.inst_addr, inst->vsi);
}

/*
 * This function runs in interrupt context and it means there's a IPI MSG
 * from VPU.
 */
static void vp8_dec_vpu_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct vdec_vpu_ipi_ack *msg = data;
	struct vdec_vp8_inst *inst = (struct vdec_vp8_inst *)msg->vdec_inst;

	mtk_vcodec_debug(inst, "+ id=%X", msg->msg_id);
	inst->vpu.failure = msg->status;

	if (msg->status == 0) {
		switch (msg->msg_id) {
		case VPU_IPIMSG_DEC_INIT_ACK:
			handle_init_ack_msg(inst, data);
			break;

		case VPU_IPIMSG_DEC_START_ACK:
		case VPU_IPIMSG_DEC_END_ACK:
		case VPU_IPIMSG_DEC_RESET_ACK:
		case VPU_IPIMSG_DEC_DEINIT_ACK:
			break;

		default:
			mtk_vcodec_err(inst, "unknown msg id %X", msg->msg_id);
			break;
		}
	}

	mtk_vcodec_debug(inst, "- id=%X", msg->msg_id);
	inst->vpu.signaled = 1;
}

static int vp8_dec_vpu_send_msg(struct vdec_vp8_inst *inst, void *msg,
				int len)
{
	int err;

	mtk_vcodec_debug(inst, "id=%X", *(unsigned int *)msg);

	inst->vpu.signaled = 0;
	inst->vpu.failure = 0;

	err = vpu_ipi_send(inst->dev, IPI_VDEC_VP8, msg, len);
	if (err) {
		mtk_vcodec_err(inst, "vpu_ipi_send fail status=%d", err);
		return -EINVAL;
	}

	return err;
}

int vdec_vp8_vpu_init(struct vdec_vp8_inst *inst)
{
	struct vdec_ap_ipi_init msg;
	int err;

	mtk_vcodec_debug_enter(inst);

	init_waitqueue_head(&inst->vpu.wq_hd);
	inst->vpu.signaled = 0;
	inst->vpu.failure = 0;

	err = vpu_ipi_register(inst->dev, IPI_VDEC_VP8,
			       vp8_dec_vpu_ipi_handler, "vdec_vp8",
			       NULL);
	if (err != 0) {
		mtk_vcodec_err(inst, "vpu_ipi_register fail status=%d", err);
		return err;
	}

	memset(&msg, 0, sizeof(msg));
	msg.msg_id	= AP_IPIMSG_DEC_INIT;
	msg.vdec_inst	= (unsigned long)inst;

	mtk_vcodec_debug(inst, "vdec_inst=%p", inst);
	err = vp8_dec_vpu_send_msg(inst, &msg, sizeof(msg));
	if (!err && inst->vpu.failure != 0)
		err = inst->vpu.failure;

	mtk_vcodec_debug(inst, "- ret=%d", err);
	return err;
}

static int vp8_dec_send_ap_ipi(struct vdec_vp8_inst *inst, unsigned int msg_id)
{
	struct vdec_ap_ipi_cmd msg;
	int err = 0;

	mtk_vcodec_debug(inst, "+ id=%X", msg_id);

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = msg_id;
	msg.vpu_inst_addr = inst->vpu.inst_addr;

	err = vp8_dec_vpu_send_msg(inst, &msg, sizeof(msg));
	if (!err && inst->vpu.failure != 0)
		err = inst->vpu.failure;

	mtk_vcodec_debug(inst, "- id=%X ret=%d", msg_id, err);
	return err;
}

int vdec_vp8_vpu_dec_start(struct vdec_vp8_inst *inst, unsigned int data)
{
	struct vdec_vp8_ipi_dec_start msg;
	int err = 0;

	mtk_vcodec_debug_enter(inst);

	memset(&msg, 0, sizeof(msg));
	msg.msg_id		= AP_IPIMSG_DEC_START;
	msg.vpu_inst_addr	= inst->vpu.inst_addr;
	msg.data		= data;

	err = vp8_dec_vpu_send_msg(inst, (void *)&msg, sizeof(msg));
	if (!err && inst->vpu.failure != 0)
		err = inst->vpu.failure;

	mtk_vcodec_debug(inst, "- ret=%d", err);
	return err;
}

int vdec_vp8_vpu_dec_end(struct vdec_vp8_inst *inst)
{
	return vp8_dec_send_ap_ipi(inst, AP_IPIMSG_DEC_END);
}

int vdec_vp8_vpu_reset(struct vdec_vp8_inst *inst)
{
	return vp8_dec_send_ap_ipi(inst, AP_IPIMSG_DEC_RESET);
}

int vdec_vp8_vpu_deinit(struct vdec_vp8_inst *inst)
{
	return vp8_dec_send_ap_ipi(inst, AP_IPIMSG_DEC_DEINIT);
}
