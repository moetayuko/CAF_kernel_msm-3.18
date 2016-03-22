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

#ifndef VDEC_VP9_VPU_H_
#define VDEC_VP9_VPU_H_

#include <linux/wait.h>

int vp9_dec_vpu_init(struct vdec_vp9_inst *vdec_inst);
int vp9_dec_vpu_start(struct vdec_vp9_inst *vdec_inst, unsigned int *data);
int vp9_dec_vpu_end(struct vdec_vp9_inst *vdec_inst);
int vp9_dec_vpu_reset(struct vdec_vp9_inst *vdec_inst);
int vp9_dec_vpu_deinit(struct vdec_vp9_inst *vdec_inst);

#endif /* #ifndef VDEC_DRV_VP9_VPU_H_ */

