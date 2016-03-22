/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: Jungchang Tsao <jungchang.tsao@mediatek.com>
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

#include "vdec_vp9_core.h"
#include "vdec_vp9_vpu.h"
#include "vdec_vp9_debug.h"
#include "mtk_vpu.h"
#include "vdec_ipi_msg.h"

#define VDEC_VP9_WAIT_VPU_TIMEOUT_MS		(10000)


/**
 * struct vdec_vp9_ipi_dec_start - for AP_IPIMSG_DEC_START
 * @msg_id        : AP_IPIMSG_DEC_START
 * @vpu_inst_addr : VPU decoder instance addr
 * @data          : Header info
 * @reserved      : Reserved field
 */
struct vdec_vp9_ipi_dec_start {
	uint32_t msg_id;
	uint32_t vpu_inst_addr;
	uint32_t data[3];
	uint32_t reserved;
};

enum vdec_vp9_ipi_msg_status {
	VDEC_VP9_IPI_MSG_STATUS_OK, /* message processed successfully */
	VDEC_VP9_IPI_MSG_STATUS_FAIL, /* message processed failed */
};

struct vdec_vp9_ipi_init_ack {
	unsigned int msg_id; /* vp9 decoder init ack */
	unsigned int status; /* vdec_vp9_ipi_msg_status */
	unsigned long long drv_id; /* handle to AP driver */
	unsigned int md32_id; /* handle to MD32 driver, point to share memory */
};

static void handle_init_ack_msg(struct vdec_vp9_inst *inst, void *data)
{
	struct vdec_vpu_ipi_init_ack *msg = data;

	inst->vpu.inst_addr = msg->vpu_inst_addr;
	inst->vpu.vsi = (struct vdec_vp9_vsi *)vpu_mapping_dm_addr(
		inst->dev, msg->vpu_inst_addr);

	mtk_vcodec_debug(inst, "inst_addr %x map to vsi %p",
					inst->vpu.inst_addr,
					inst->vpu.vsi);
}

static void vp9_dec_vpu_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct vdec_vpu_ipi_ack *msg = data;
	struct vdec_vp9_inst *inst = (struct vdec_vp9_inst *)msg->vdec_inst;

	inst->vpu.failure = msg->status;

	mtk_vcodec_debug(inst, "+ id=%X status=%d len=%d", msg->msg_id,
			 msg->status, len);

	switch (msg->msg_id) {
	case VPU_IPIMSG_DEC_INIT_ACK:
		handle_init_ack_msg(inst, data);
		break;
	case VPU_IPIMSG_DEC_START_ACK:
	case VPU_IPIMSG_DEC_END_ACK:
	case VPU_IPIMSG_DEC_RESET_ACK:
	case VPU_IPIMSG_DEC_DEINIT_ACK:
		inst->vpu.failure = msg->status;
		break;
	default:
		mtk_vcodec_err(inst, "unknown msg id %X", msg->msg_id);
		break;
	}
	inst->vpu.signaled = 1;
}

static int vp9_dec_vpu_send_msg(struct vdec_vp9_inst *inst, void *msg,
				int len)
{
	int err;
	struct vdec_vp9_vpu_inst *vpu = &inst->vpu;

	mtk_vcodec_debug(inst, "id=%X", *(unsigned int *)msg);

	vpu->failure = 0;
	vpu->signaled = 0;

	err = vpu_ipi_send(inst->dev, IPI_VDEC_VP9, msg, len);
	if (err) {
		mtk_vcodec_err(inst, "vpu_ipi_send fail status=%d", err);
		return err;
	}

	return err;
}

int vp9_dec_vpu_init(struct vdec_vp9_inst *inst)
{
	int ret;
	struct vdec_ap_ipi_init out;

	mtk_vcodec_debug_enter(inst);

	init_waitqueue_head(&inst->vpu.wq_hd);
	inst->vpu.signaled = 0;
	inst->vpu.failure = 0;

	ret = vpu_ipi_register(inst->dev, IPI_VDEC_VP9,
			       vp9_dec_vpu_ipi_handler, "vp9_dec", NULL);
	if (ret) {
		mtk_vcodec_err(inst, "vpu_ipi_register fail status=%d", ret);
		return ret;
	}
	mtk_vcodec_debug(inst, "vpu_ipi_register success %p", inst);

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_DEC_INIT;
	out.vdec_inst = (unsigned long)inst;

	ret = vp9_dec_vpu_send_msg(inst, &out, sizeof(out));
	if (!ret && inst->vpu.failure != 0)
		ret = inst->vpu.failure;

	mtk_vcodec_debug_leave(inst);

	return ret;
}

static int vp9_dec_send_ap_ipi(void *vdec_inst, unsigned int msg_id)
{
	struct vdec_vp9_inst *inst = vdec_inst;
	struct vdec_ap_ipi_cmd msg;
	int err = 0;

	mtk_vcodec_debug(inst, "+ id=%X", msg_id);

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = msg_id;
	msg.vpu_inst_addr = inst->vpu.inst_addr;

	err = vp9_dec_vpu_send_msg(inst, &msg, sizeof(msg));
	if (!err && inst->vpu.failure != 0)
		err = inst->vpu.failure;

	mtk_vcodec_debug(inst, "- id=%X ret=%d", msg_id, err);
	return err;
}

int vp9_dec_vpu_start(struct vdec_vp9_inst *vdec_inst, unsigned int *data)
{

	struct vdec_vp9_ipi_dec_start msg;
	int ret, i;

	mtk_vcodec_debug_enter(vdec_inst);

	memset(&msg, 0, sizeof(msg));
	msg.msg_id				= AP_IPIMSG_DEC_START;
	msg.vpu_inst_addr		= vdec_inst->vpu.inst_addr;
	for (i = 0; i < 3; i++)
		msg.data[i] = data[i];

	ret = vp9_dec_vpu_send_msg(vdec_inst, (void *)&msg, sizeof(msg));
	if (!ret && vdec_inst->vpu.failure != 0)
		ret = vdec_inst->vpu.failure;

	mtk_vcodec_debug(vdec_inst, "- ret=%d", ret);

	return ret;

}

int vp9_dec_vpu_end(struct vdec_vp9_inst *vdec_inst)
{
	return vp9_dec_send_ap_ipi(vdec_inst, AP_IPIMSG_DEC_END);
}

int vp9_dec_vpu_reset(struct vdec_vp9_inst *vdec_inst)
{
	return vp9_dec_send_ap_ipi(vdec_inst, AP_IPIMSG_DEC_RESET);
}

int vp9_dec_vpu_deinit(struct vdec_vp9_inst *vdec_inst)
{
	return vp9_dec_send_ap_ipi(vdec_inst, AP_IPIMSG_DEC_DEINIT);
}

