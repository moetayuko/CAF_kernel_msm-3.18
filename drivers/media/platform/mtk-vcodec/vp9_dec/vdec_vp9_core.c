/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: Daniel Hsiao <daniel.hsiao@mediatek.com>
 *             Kai-Sean Yang <kai-sean.yang@mediatek.com>
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

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/file.h>

#include "mtk_vcodec_drv.h"
#include "mtk_vpu.h"
#include "vdec_vp9_if.h"
#include "vdec_vp9_core.h"
#include "vdec_vp9_vpu.h"
#include "vdec_vp9_debug.h"


static int vp9_setup_buf(struct vdec_vp9_inst *inst)
{
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;

	vsi->mv_buf.va = (unsigned long)inst->work_buf.mv_buf.va;
	vsi->mv_buf.pa = (unsigned long)inst->work_buf.mv_buf.dma_addr;
	vsi->mv_buf.sz = (unsigned long)inst->work_buf.mv_buf.size;

	if ((vsi->mv_buf.va == 0) || (vsi->mv_buf.pa == 0) ||
		(vsi->mv_buf.sz == 0))
		return -EINVAL;

	mtk_vcodec_debug(inst, "VP9_MV_BUF_Addr: 0x%lX (0x%lX)",
		     vsi->mv_buf.va, vsi->mv_buf.pa);
	return 0;
}

static void vp9_ref_cnt_fb(struct vdec_vp9_inst *inst, int *idx,
			   int new_idx)
{
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;
	int ref_idx = *idx;

	if (ref_idx >= 0 && vsi->frm_bufs[ref_idx].ref_cnt > 0) {
		vsi->frm_bufs[ref_idx].ref_cnt--;

		if (vsi->frm_bufs[ref_idx].ref_cnt == 0) {
			if (!vp9_is_sf_ref_fb(inst,
					      vsi->frm_bufs[ref_idx].buf.fb)) {
				struct vdec_fb *fb;

				fb = vp9_rm_from_fb_use_list(inst,
				     vsi->frm_bufs[ref_idx].buf.fb->base_y.va);
				vp9_add_to_fb_free_list(inst, fb);
			} else
				vp9_free_sf_ref_fb(
					inst, vsi->frm_bufs[ref_idx].buf.fb);
		}
	}

	*idx = new_idx;
	vsi->frm_bufs[new_idx].ref_cnt++;
}

bool vp9_realloc_work_buf(struct vdec_vp9_inst *inst)
{
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;
	int result;
	struct mtk_vcodec_mem *mem;

	inst->frm_hdr.width = inst->vpu.vsi->pic_w;
	inst->frm_hdr.height = inst->vpu.vsi->pic_h;
	inst->work_buf.frmbuf_width = inst->vpu.vsi->buf_w;
	inst->work_buf.frmbuf_height = inst->vpu.vsi->buf_h;

	mtk_vcodec_debug(inst, "BUF CHG(%d): w/h/sb_w/sb_h=%d/%d/%d/%d",
		     inst->frm_hdr.resolution_changed,
		     inst->frm_hdr.width,
		     inst->frm_hdr.height,
		     inst->work_buf.frmbuf_width,
		     inst->work_buf.frmbuf_height);

	if ((inst->frm_hdr.width > 4096) ||
		(inst->frm_hdr.height > 2304)) {
		mtk_vcodec_err(inst, "Invalid w/h %d/%d",
			inst->frm_hdr.width,
			inst->frm_hdr.height);
		return false;
	}

	mem = &inst->work_buf.mv_buf;
	/* Free First */
	if (mem->va)
		mtk_vcodec_mem_free(inst->ctx, mem);
	/* Alloc Later */
	mem->size = ((inst->work_buf.frmbuf_width / 64) *
		    (inst->work_buf.frmbuf_height / 64) + 2) * 36 * 16;

	result = mtk_vcodec_mem_alloc(inst->ctx, mem);
	if (result) {
		mtk_vcodec_err(inst, "Cannot allocate mv_buf");
		return false;
	}
	/* Set the va again */
	vsi->mv_buf.va = (unsigned long)inst->work_buf.mv_buf.va;
	vsi->mv_buf.pa = (unsigned long)inst->work_buf.mv_buf.dma_addr;
	vsi->mv_buf.sz = (unsigned long)inst->work_buf.mv_buf.size;

	vp9_free_all_sf_ref_fb(inst);
	vsi->sf_next_ref_fb_idx = vp9_get_sf_ref_fb(inst);
	result = vp9_setup_buf(inst);
	if (result) {
		mtk_vcodec_err(inst, "Cannot vp9_setup_buf");
		return false;
	}

	inst->frm_hdr.resolution_changed = true;

	return true;
}

static void vp9_swap_frm_bufs(struct vdec_vp9_inst *inst)
{
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;
	struct vp9_fb_info *frm_to_show;
	int ref_index = 0, mask;

	for (mask = inst->vpu.vsi->refresh_frm_flags; mask; mask >>= 1) {
		if (mask & 1)
			vp9_ref_cnt_fb(inst, &vsi->ref_frm_map[ref_index],
				       vsi->new_fb_idx);
		++ref_index;
	}

	frm_to_show = &vsi->frm_bufs[vsi->new_fb_idx].buf;
	--vsi->frm_bufs[vsi->new_fb_idx].ref_cnt;

	if (frm_to_show->fb != inst->cur_fb) {
		if ((frm_to_show->fb != NULL) &&
			(inst->cur_fb->base_y.size >=
				frm_to_show->fb->base_y.size)) {
			memcpy((void *)inst->cur_fb->base_y.va,
				(void *)frm_to_show->fb->base_y.va,
				inst->work_buf.frmbuf_width *
				inst->work_buf.frmbuf_height);
			memcpy((void *)inst->cur_fb->base_c.va,
				(void *)frm_to_show->fb->base_c.va,
				inst->work_buf.frmbuf_width *
				inst->work_buf.frmbuf_height / 2);
		} else {
			mtk_vcodec_debug(inst,
				"inst->cur_fb->base_y.size=%lx, frm_to_show->fb.base_y.size=%lx",
				inst->cur_fb->base_y.size,
				frm_to_show->fb->base_y.size);
		}
		if (!vp9_is_sf_ref_fb(inst, inst->cur_fb)) {
			if (inst->frm_hdr.show_frame)
				vp9_add_to_fb_disp_list(inst, inst->cur_fb);
		}
	} else {
		if (!vp9_is_sf_ref_fb(inst, inst->cur_fb)) {
			if (inst->frm_hdr.show_frame)
				vp9_add_to_fb_disp_list(inst, frm_to_show->fb);
		}
	}

	if (vsi->frm_bufs[vsi->new_fb_idx].ref_cnt == 0) {
		if (!vp9_is_sf_ref_fb(
			inst, vsi->frm_bufs[vsi->new_fb_idx].buf.fb)) {
			struct vdec_fb *fb;

			fb = vp9_rm_from_fb_use_list(inst,
			     vsi->frm_bufs[vsi->new_fb_idx].buf.fb->base_y.va);

			vp9_add_to_fb_free_list(inst, fb);
		} else
			vp9_free_sf_ref_fb(
				inst, vsi->frm_bufs[vsi->new_fb_idx].buf.fb);
	}

	if (vsi->sf_frm_cnt > 0 && vsi->sf_frm_idx != vsi->sf_frm_cnt - 1)
		vsi->sf_next_ref_fb_idx = vp9_get_sf_ref_fb(inst);

}

static bool vp9_wait_dec_end(struct vdec_vp9_inst *inst)
{
	struct mtk_vcodec_ctx *ctx = inst->ctx;
	unsigned int irq_status;

	mtk_vcodec_wait_for_done_ctx(inst->ctx, MTK_INST_IRQ_RECEIVED,
	                             WAIT_INTR_TIMEOUT_MS);

	irq_status = ctx->irq_status;
	mtk_vcodec_debug(inst, "isr return %x", irq_status);

	if (irq_status & 0x10000)
		return true;
	else
		return false;
}

/* End */

bool vp9_get_hw_reg_base(struct vdec_vp9_inst *inst)
{
	mtk_vcodec_debug_enter(inst);

	inst->hw_reg_base.sys = mtk_vcodec_get_reg_addr(inst->ctx,
							  VDEC_SYS);
	inst->hw_reg_base.misc = mtk_vcodec_get_reg_addr(inst->ctx,
							   VDEC_MISC);
	inst->hw_reg_base.ld = mtk_vcodec_get_reg_addr(inst->ctx, VDEC_LD);
	inst->hw_reg_base.top = mtk_vcodec_get_reg_addr(inst->ctx,
							  VDEC_TOP);
	inst->hw_reg_base.cm = mtk_vcodec_get_reg_addr(inst->ctx, VDEC_CM);
	inst->hw_reg_base.av = mtk_vcodec_get_reg_addr(inst->ctx, VDEC_AV);
	inst->hw_reg_base.hwp = mtk_vcodec_get_reg_addr(inst->ctx, VDEC_PP);
	inst->hw_reg_base.hwb = mtk_vcodec_get_reg_addr(inst->ctx,
							  VDEC_HWB);
	inst->hw_reg_base.hwg = mtk_vcodec_get_reg_addr(inst->ctx,
							  VDEC_HWG);

	return true;
}

bool vp9_free_work_buf(struct vdec_vp9_inst *inst)
{
	struct mtk_vcodec_mem *mem;

	mtk_vcodec_debug_enter(inst);

	mem = &inst->work_buf.mv_buf;
	if (mem->va)
		mtk_vcodec_mem_free(inst->ctx, mem);

	vp9_free_all_sf_ref_fb(inst);
	return true;
}

struct vdec_vp9_inst *vp9_alloc_inst(void *ctx)
{
	int result;
	struct mtk_vcodec_mem mem;
	struct vdec_vp9_inst *inst;

	mem.size = sizeof(struct vdec_vp9_inst) * 2;
	result = mtk_vcodec_mem_alloc(ctx, &mem);
	if (result)
		return NULL;

	inst = mem.va;
	inst->mem = mem;

	return inst;
}

void vp9_free_handle(struct vdec_vp9_inst *inst)
{
	struct mtk_vcodec_mem mem;

	mem = inst->mem;
	if (mem.va)
		mtk_vcodec_mem_free(inst->ctx, &mem);
}

bool vp9_init_proc(struct vdec_vp9_inst *inst,
		   struct vdec_pic_info *pic_info)
{
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;

	pic_info->pic_w = vsi->pic_w;
	pic_info->pic_h = vsi->pic_h;
	pic_info->buf_w = vsi->buf_w;
	pic_info->buf_h = vsi->buf_h;

	mtk_vcodec_debug(inst,
			"(PicW,PicH,BufW,BufH) = (%d,%d,%d,%d) profile=%d",
			pic_info->pic_w, pic_info->pic_h,
			pic_info->buf_w, pic_info->buf_h, vsi->profile);

	inst->frm_hdr.width = vsi->pic_w;
	inst->frm_hdr.height = vsi->pic_h;
	inst->work_buf.frmbuf_width = vsi->buf_w;
	inst->work_buf.frmbuf_height = vsi->buf_h;

	/* ----> HW limitation */
	if ((inst->frm_hdr.width > 4096) ||
		(inst->frm_hdr.height > 2304)) {
		mtk_vcodec_err(inst, "Invalid w/h %d/%d",
			inst->frm_hdr.width,
			inst->frm_hdr.height);
		return false;
	}

	/* ----> HW limitation */
	if (vsi->profile > 0) {
		mtk_vcodec_err(inst, "vp9_dec DO NOT support profile(%d) > 0",
			     vsi->profile);
		return false;
	}
	if ((inst->work_buf.frmbuf_width > 4096) ||
	    (inst->work_buf.frmbuf_height > 2304)) {
		mtk_vcodec_err(inst, "vp9_dec DO NOT support (W,H) = (%d,%d)",
			     inst->work_buf.frmbuf_width,
			     inst->work_buf.frmbuf_height);
		return false;
	}
	/* <---- HW limitation */

	return true;
}

bool vp9_dec_proc(struct vdec_vp9_inst *inst, struct mtk_vcodec_mem *bs,
		  struct vdec_fb *fb)
{
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;
	unsigned int i;
	unsigned int data[3];

	mtk_vcodec_debug_enter(inst);

	data[0] = *((unsigned int *)bs->va);
	data[1] = *((unsigned int *)(bs->va + 4));
	data[2] = *((unsigned int *)(bs->va + 8));

	vsi->bs = *bs;

	if (fb)	
		vsi->fb = *fb;

	/* TBD: use barrel shifter if fast enough */
	if (!vsi->sf_init) {
		unsigned int sf_bs_sz;
		unsigned int sf_bs_off;
		unsigned char *sf_bs_src;
		unsigned char *sf_bs_dst;

		sf_bs_sz = bs->size > VP9_SUPER_FRAME_BS_SZ ?
			VP9_SUPER_FRAME_BS_SZ :	bs->size;
		sf_bs_off = VP9_SUPER_FRAME_BS_SZ - sf_bs_sz;
		sf_bs_src = bs->va + bs->size - sf_bs_sz;
		sf_bs_dst = vsi->sf_bs_buf + sf_bs_off;
		memcpy(sf_bs_dst, sf_bs_src, sf_bs_sz);
	} else {
		if ((vsi->sf_frm_cnt > 0) &&
		    (vsi->sf_frm_idx < vsi->sf_frm_cnt)) {
			unsigned int idx = vsi->sf_frm_idx;

			/* TBD: use barrel shifter to reposition bit stream */
			memcpy((void *)vsi->input_ctx.v_frm_sa,
			       (void *)(vsi->input_ctx.v_frm_sa +
			       vsi->sf_frm_offset[idx]),
			       vsi->sf_frm_sz[idx]);
		}
	}

	if (0 != vp9_dec_vpu_start(inst, data)) {
		mtk_vcodec_err(inst, "vp9_dec_vpu_start failed");
		return false;
	}

	if (vsi->resolution_changed) {
		if (!vp9_realloc_work_buf(inst))
			return false;
		return true;
	}

	if (vsi->sf_frm_cnt > 0) {
		if (vsi->sf_frm_idx < vsi->sf_frm_cnt)
			inst->cur_fb =
				&vsi->sf_ref_fb[vsi->sf_next_ref_fb_idx].fb;
		else
			inst->cur_fb = fb;
	} else {
		inst->cur_fb = fb;
	}

	vsi->frm_bufs[vsi->new_fb_idx].buf.fb = inst->cur_fb;
	if (!vp9_is_sf_ref_fb(inst, inst->cur_fb))
		vp9_add_to_fb_use_list(inst, inst->cur_fb);

	mtk_vcodec_debug(inst, "[#pic %d]", vsi->frm_num);

	/* the same as VP9_SKIP_FRAME */
	inst->frm_hdr.show_frame = vsi->show_frm;

	if (vsi->show_exist)
		mtk_vcodec_debug(inst,
			"drv->new_fb_idx=%d, drv->frm_to_show=%d",
			vsi->new_fb_idx, vsi->frm_to_show);

	if (vsi->show_exist && (vsi->frm_to_show < VP9_MAX_FRM_BUFF_NUM)) {
		mtk_vcodec_debug(inst,
			"Skip Decode drv->new_fb_idx=%d, drv->frm_to_show=%d",
			vsi->new_fb_idx, vsi->frm_to_show);
		vp9_ref_cnt_fb(inst, &vsi->new_fb_idx, vsi->frm_to_show);
		return true;
	}

	/* VPU assign the buffer pointer in its address space, reassign here */
	for (i = 0; i < REFS_PER_FRAME; i++) {
		unsigned int idx = vsi->frm_refs[i].idx;

		vsi->frm_refs[i].buf = &vsi->frm_bufs[idx].buf;
	}

	mtk_vcodec_debug_leave(inst);

	return true;
}

bool vp9_check_proc(struct vdec_vp9_inst *inst)
{
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;
	bool ret = false;

	mtk_vcodec_debug_enter(inst);

	if (vsi->show_exist) {
		vp9_swap_frm_bufs(inst);
		mtk_vcodec_debug(inst, "Decode Ok @%d (show_exist)",
				 vsi->frm_num);
		vsi->frm_num++;
		return true;
	}

	ret = vp9_wait_dec_end(inst);
	if (!ret) {
		mtk_vcodec_err(inst, "Decode NG, Decode Timeout @[%d]",
			       vsi->frm_num);
		return false;
	}

	if (0 != vp9_dec_vpu_end(inst)) {
		mtk_vcodec_err(inst, "vp9_dec_vpu_end failed");
		return false;
	}

	vp9_swap_frm_bufs(inst);
	mtk_vcodec_debug(inst, "Decode Ok @%d (%d/%d)", vsi->frm_num,
		     inst->frm_hdr.width, inst->frm_hdr.height);

	vsi->frm_num++;

	mtk_vcodec_debug_leave(inst);

	return true;
}

int vp9_get_sf_ref_fb(struct vdec_vp9_inst *inst)
{
	int i;
	struct mtk_vcodec_mem *mem;
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;

	for (i = 0; i < VP9_MAX_FRM_BUFF_NUM - 1; i++) {
		if (inst->work_buf.sf_ref_buf[i].base_y.va &&
		    vsi->sf_ref_fb[i].used == 0) {
			return i;
		}
	}

	for (i = 0; i < VP9_MAX_FRM_BUFF_NUM - 1; i++) {
		if (inst->work_buf.sf_ref_buf[i].base_y.va == NULL)
			break;
	}

	if (i == VP9_MAX_FRM_BUFF_NUM - 1) {
		mtk_vcodec_err(inst, "List Full");
		return -1;
	}

	mem = &inst->work_buf.sf_ref_buf[i].base_y;
	mem->size = inst->vpu.vsi->buf_sz_y_bs +
		    inst->vpu.vsi->buf_len_sz_y;

	if ((inst->frm_hdr.width > 4096) ||
		(inst->frm_hdr.height > 2304)) {
		mtk_vcodec_err(inst, "Invalid w/h %d/%d",
			inst->frm_hdr.width,
			inst->frm_hdr.height);
		return -1;
	}

	if (mtk_vcodec_mem_alloc(inst->ctx, mem)) {
		mtk_vcodec_err(inst, "Cannot allocate sf_ref_buf y_buf");
		return -1;
	}

	mtk_vcodec_debug(inst, "allocate sf_ref_buf y_buf = 0x%lx, %d",
			mem->size,
			inst->work_buf.frmbuf_width *
			inst->work_buf.frmbuf_height);

	vsi->sf_ref_fb[i].fb.base_y.va =
				inst->work_buf.sf_ref_buf[i].base_y.va;
	vsi->sf_ref_fb[i].fb.base_y.dma_addr =
				inst->work_buf.sf_ref_buf[i].base_y.dma_addr;
	vsi->sf_ref_fb[i].fb.base_y.size =
				inst->work_buf.sf_ref_buf[i].base_y.size;

	mem = &inst->work_buf.sf_ref_buf[i].base_c;
	mem->size = inst->vpu.vsi->buf_sz_c_bs +
		    inst->vpu.vsi->buf_len_sz_c;

	if (mtk_vcodec_mem_alloc(inst->ctx, mem)) {
		mtk_vcodec_err(inst, "Cannot allocate sf_ref_buf c_buf");
		return -1;
	}

	mtk_vcodec_debug(inst, "allocate sf_ref_buf c_buf = 0x%lx, %d",
			mem->size,
			inst->work_buf.frmbuf_width *
			inst->work_buf.frmbuf_height / 2);

	vsi->sf_ref_fb[i].fb.base_c.va =
				inst->work_buf.sf_ref_buf[i].base_c.va;
	vsi->sf_ref_fb[i].fb.base_c.dma_addr =
				inst->work_buf.sf_ref_buf[i].base_c.dma_addr;
	vsi->sf_ref_fb[i].fb.base_c.size =
				inst->work_buf.sf_ref_buf[i].base_c.size;

	vsi->sf_ref_fb[i].used = 0;
	vsi->sf_ref_fb[i].idx = i;

	return i;
}

bool vp9_free_sf_ref_fb(struct vdec_vp9_inst *inst, struct vdec_fb *fb)
{
	struct vp9_sf_ref_fb *sf_ref_fb =
		container_of(fb, struct vp9_sf_ref_fb, fb);

	sf_ref_fb->used = 0;

	return true;
}

void vp9_free_all_sf_ref_fb(struct vdec_vp9_inst *inst)
{
	int i;
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;

	for (i = 0; i < VP9_MAX_FRM_BUFF_NUM - 1; i++) {
		if (inst->work_buf.sf_ref_buf[i].base_y.va) {
			mtk_vcodec_mem_free(inst->ctx,
				    &inst->work_buf.sf_ref_buf[i].base_y);
			mtk_vcodec_mem_free(inst->ctx,
				    &inst->work_buf.sf_ref_buf[i].base_c);
			vsi->sf_ref_fb[i].used = 0;
		}
	}
}

bool vp9_is_sf_ref_fb(struct vdec_vp9_inst *inst, struct vdec_fb *fb)
{
	int i;
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;

	for (i = 0; i < VP9_MAX_FRM_BUFF_NUM - 1; i++) {
		if (fb == &vsi->sf_ref_fb[i].fb)
			break;
	}

	if (i == VP9_MAX_FRM_BUFF_NUM - 1)
		return false;

	return true;
}

bool vp9_is_last_sub_frm(struct vdec_vp9_inst *inst)
{
	struct vdec_vp9_vsi *vsi = inst->vpu.vsi;

	if (vsi->sf_frm_cnt <= 0 || vsi->sf_frm_idx == vsi->sf_frm_cnt)
		return true;

	return false;
}

bool vp9_add_to_fb_disp_list(struct vdec_vp9_inst *inst,
			     struct vdec_fb *fb)
{
	struct vdec_fb_node *node;

	if (!fb)
		return false;

	node = list_first_entry_or_null(&inst->available_fb_node_list,
					struct vdec_fb_node, list);
	if (node) {
		node->fb = fb;
		list_move_tail(&node->list, &inst->fb_disp_list);
	} else {
		mtk_vcodec_debug(inst, "List Full");
		return false;
	}

	mtk_vcodec_debug_leave(inst);

	return true;
}

struct vdec_fb *vp9_rm_from_fb_disp_list(struct vdec_vp9_inst
		*inst)
{
	struct vdec_fb_node *node;
	struct vdec_fb *fb = NULL;

	node = list_first_entry_or_null(&inst->fb_disp_list,
					struct vdec_fb_node, list);
	if (node) {
		fb = (struct vdec_fb *)node->fb;
		fb->status |= FB_ST_DISPLAY;
		list_move_tail(&node->list, &inst->available_fb_node_list);
		mtk_vcodec_debug(inst, "[FB] get disp fb %p st=%d",
				 node->fb, fb->status);
	} else
		mtk_vcodec_debug(inst, "[FB] there is no disp fb");

	return fb;
}

bool vp9_add_to_fb_use_list(struct vdec_vp9_inst *inst,
			    struct vdec_fb *fb)
{
	struct vdec_fb_node *node;

	if (!fb)
		return false;

	node = list_first_entry_or_null(&inst->available_fb_node_list,
					struct vdec_fb_node, list);
	if (node) {
		node->fb = fb;
		list_move_tail(&node->list, &inst->fb_use_list);
	} else {
		mtk_vcodec_debug(inst, "No free fb node");
		return false;
	}

	mtk_vcodec_debug_leave(inst);

	return true;
}

struct vdec_fb *vp9_rm_from_fb_use_list(struct vdec_vp9_inst
					*inst, void *addr)
{
	struct vdec_fb *fb;
	struct vdec_fb_node *node;

	list_for_each_entry(node, &inst->fb_use_list, list) {
		fb = (struct vdec_fb *)node->fb;
		if (fb->base_y.va == addr) {
			list_move_tail(&node->list,
				       &inst->available_fb_node_list);
			break;
		}
	}

	mtk_vcodec_debug_leave(inst);
	return fb;
}


bool vp9_add_to_fb_free_list(struct vdec_vp9_inst *inst,
			     struct vdec_fb *fb)
{
	struct vdec_fb_node *node;

	if (fb) {
		node = list_first_entry_or_null(&inst->available_fb_node_list,
					struct vdec_fb_node, list);
		if (node) {
			node->fb = fb;
			list_move_tail(&node->list, &inst->fb_free_list);
		} else
			mtk_vcodec_debug(inst, "No free fb node");
	}

	mtk_vcodec_debug_leave(inst);

	return true;
}


struct vdec_fb *vp9_rm_from_fb_free_list(struct vdec_vp9_inst
		*inst)
{
	struct vdec_fb_node *node;
	struct vdec_fb *fb = NULL;

	node = list_first_entry_or_null(&inst->fb_free_list,
					struct vdec_fb_node, list);
	if (node) {
		fb = (struct vdec_fb *)node->fb;
		fb->status |= FB_ST_FREE;
		list_move_tail(&node->list, &inst->available_fb_node_list);
		mtk_vcodec_debug(inst, "[FB] get free fb %p st=%d",
				 node->fb, fb->status);
	} else
		mtk_vcodec_debug(inst, "[FB] there is no free fb");

	mtk_vcodec_debug_leave(inst);

	return fb;
}

bool vp9_fb_use_list_to_fb_free_list(struct vdec_vp9_inst *inst)
{

	struct vdec_fb_node *node, *tmp;

	list_for_each_entry_safe(node, tmp, &inst->fb_use_list, list)
		list_move_tail(&node->list, &inst->fb_free_list);

	mtk_vcodec_debug_leave(inst);
	return true;
}

void vp9_reset(struct vdec_vp9_inst *inst)
{
	vp9_fb_use_list_to_fb_free_list(inst);

	vp9_free_all_sf_ref_fb(inst);
	inst->vpu.vsi->sf_next_ref_fb_idx = vp9_get_sf_ref_fb(inst);

	if (0 != vp9_dec_vpu_reset(inst))
		mtk_vcodec_debug(inst, "vp9_dec_vpu_reset failed");

	if (vp9_setup_buf(inst))
		mtk_vcodec_debug(inst, "vp9_setup_buf failed");
}
