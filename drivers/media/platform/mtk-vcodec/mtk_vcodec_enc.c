/*
* Copyright (c) 2016 MediaTek Inc.
* Author: PC Chen <pc.chen@mediatek.com>
*         Tiffany Lin <tiffany.lin@mediatek.com>
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

#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "mtk_vcodec_drv.h"
#include "mtk_vcodec_enc.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_util.h"
#include "venc_drv_if.h"

#define MTK_VENC_MIN_W	32U
#define MTK_VENC_MIN_H	32U
#define MTK_VENC_MAX_W	1920U
#define MTK_VENC_MAX_H	1080U
#define DFT_CFG_WIDTH	MTK_VENC_MIN_W
#define DFT_CFG_HEIGHT	MTK_VENC_MIN_H

#define CLAMP(value, low, high) (((value) < (low)) ? (low) : \
					(((value) > (high)) ? (high):(value)))

static void mtk_venc_worker(struct work_struct *work);

static struct mtk_video_fmt mtk_video_formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_YUV420,
		.type		= MTK_FMT_FRAME,
		.num_planes	= 3,
	},
	{
		.fourcc		= V4L2_PIX_FMT_YVU420,
		.type		= MTK_FMT_FRAME,
		.num_planes	= 3,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12,
		.type		= MTK_FMT_FRAME,
		.num_planes	= 2,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV21,
		.type		= MTK_FMT_FRAME,
		.num_planes	= 2,
	},
	{
		.fourcc		= V4L2_PIX_FMT_YUV420M,
		.type		= MTK_FMT_FRAME,
		.num_planes	= 3,
	},
	{
		.fourcc		= V4L2_PIX_FMT_YVU420M,
		.type		= MTK_FMT_FRAME,
		.num_planes	= 3,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.type		= MTK_FMT_FRAME,
		.num_planes	= 2,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV21M,
		.type		= MTK_FMT_FRAME,
		.num_planes	= 2,
	},
	{
		.fourcc		= V4L2_PIX_FMT_H264,
		.type		= MTK_FMT_ENC,
		.num_planes	= 1,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VP8,
		.type		= MTK_FMT_ENC,
		.num_planes	= 1,
	},
};

#define NUM_FORMATS ARRAY_SIZE(mtk_video_formats)

static const struct mtk_codec_framesizes mtk_venc_framesizes[] = {
	{
		.fourcc	= V4L2_PIX_FMT_H264,
		.stepwise = {  160, 1920, 16, 128, 1088, 16 },
	},
	{
		.fourcc = V4L2_PIX_FMT_VP8,
		.stepwise = {  160, 1920, 16, 128, 1088, 16 },
	},
};

#define NUM_SUPPORTED_FRAMESIZE ARRAY_SIZE(mtk_venc_framesizes)

static int vidioc_venc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_vcodec_ctx *ctx = ctrl_to_ctx(ctrl);
	struct mtk_enc_params *p = &ctx->enc_params;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_BITRATE val = %d",
			       ctrl->val);
		p->bitrate = ctrl->val;
		ctx->param_change |= MTK_ENCODE_PARAM_BITRATE;
		break;
	case V4L2_CID_MPEG_VIDEO_B_FRAMES:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_B_FRAMES val = %d",
			       ctrl->val);
		p->num_b_frame = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE val = %d",
			       ctrl->val);
		p->rc_frame = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_MAX_QP val = %d",
			       ctrl->val);
		p->h264_max_qp = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_HEADER_MODE val = %d",
			       ctrl->val);
		p->seq_hdr_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE val = %d",
			       ctrl->val);
		p->rc_mb = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_PROFILE val = %d",
			       ctrl->val);
		p->h264_profile = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_LEVEL val = %d",
			       ctrl->val);
		p->h264_level = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_H264_I_PERIOD val = %d",
			       ctrl->val);
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_GOP_SIZE val = %d",
			       ctrl->val);
		p->gop_size = ctrl->val;
		ctx->param_change |= MTK_ENCODE_PARAM_INTRA_PERIOD;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		mtk_v4l2_debug(2, "V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME");
		p->force_intra = 1;
		ctx->param_change |= MTK_ENCODE_PARAM_FRAME_TYPE;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops mtk_vcodec_enc_ctrl_ops = {
	.s_ctrl = vidioc_venc_s_ctrl,
};

static int vidioc_enum_fmt(struct file *file, struct v4l2_fmtdesc *f,
			   bool out)
{
	struct mtk_video_fmt *fmt;
	int i, j = 0;

	for (i = 0; i < NUM_FORMATS; ++i) {
		if (out && mtk_video_formats[i].type != MTK_FMT_FRAME)
			continue;
		if (!out && mtk_video_formats[i].type != MTK_FMT_ENC)
			continue;

		if (j == f->index) {
			fmt = &mtk_video_formats[i];
			f->pixelformat = fmt->fourcc;
			return 0;
		}
		++j;
	}

	return -EINVAL;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *fsize)
{
	int i = 0;

	for (i = 0; i < NUM_SUPPORTED_FRAMESIZE; ++i) {
		if (fsize->pixel_format != mtk_venc_framesizes[i].fourcc)
			continue;

		if (!fsize->index) {
			fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
			fsize->stepwise = mtk_venc_framesizes[i].stepwise;
			return 0;
		}
	}

	return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap_mplane(struct file *file, void *pirv,
					  struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(file, f, false);
}

static int vidioc_enum_fmt_vid_out_mplane(struct file *file, void *prov,
					  struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(file, f, true);
}

static int vidioc_venc_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	strlcpy(cap->driver, MTK_VCODEC_ENC_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, MTK_PLATFORM_STR, sizeof(cap->bus_info));
	strlcpy(cap->card, MTK_PLATFORM_STR, sizeof(cap->card));

	cap->device_caps  = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vidioc_venc_s_parm(struct file *file, void *priv,
			      struct v4l2_streamparm *a)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	ctx->enc_params.framerate_num =
			a->parm.output.timeperframe.denominator;
	ctx->enc_params.framerate_denom =
			a->parm.output.timeperframe.numerator;
	ctx->param_change |= MTK_ENCODE_PARAM_FRAMERATE;

	return 0;
}

static int vidioc_venc_g_parm(struct file *file, void *priv,
			      struct v4l2_streamparm *a)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	a->parm.output.timeperframe.denominator =
			ctx->enc_params.framerate_num;
	a->parm.output.timeperframe.numerator =
			ctx->enc_params.framerate_denom;

	return 0;
}

static struct mtk_q_data *mtk_venc_get_q_data(struct mtk_vcodec_ctx *ctx,
					      enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->q_data[MTK_Q_DATA_SRC];

	return &ctx->q_data[MTK_Q_DATA_DST];
}

static struct mtk_video_fmt *mtk_venc_find_format(struct v4l2_format *f)
{
	struct mtk_video_fmt *fmt;
	unsigned int k;

	for (k = 0; k < NUM_FORMATS; k++) {
		fmt = &mtk_video_formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			return fmt;
	}

	return NULL;
}

static void mtk_vcodec_enc_calc_src_size(
	unsigned int num_planes, unsigned int pic_width,
	unsigned int pic_height, unsigned int sizeimage[],
	unsigned int bytesperline[])
{
	unsigned int y_pitch_w_div16;
	unsigned int c_pitch_w_div16;

	y_pitch_w_div16 = ALIGN(pic_width, 16) >> 4;
	c_pitch_w_div16 = ALIGN(pic_width, 16) >> 4;

	if (num_planes == 2) {
		sizeimage[0] =
			(y_pitch_w_div16) * (((pic_height + 31) / 32) * 2) *
			256 + ((y_pitch_w_div16 % 8 == 0) ?
			0 : ((ALIGN(pic_width, 16) * 2) * 16));

		sizeimage[1] =
			(c_pitch_w_div16) * (((pic_height + 31) / 32) * 2) *
			128 + ((c_pitch_w_div16 % 8 == 0) ?
			0 : (ALIGN(pic_width, 16) * 16));

		sizeimage[2] = 0;

		bytesperline[0] = ALIGN(pic_width, 16);
		bytesperline[1] = ALIGN(pic_width, 16);
		bytesperline[2] = 0;

	} else {
		sizeimage[0] =
			(y_pitch_w_div16) * (((pic_height + 31) / 32) * 2) *
			256 + ((y_pitch_w_div16 % 8 == 0) ?
			0 : ((ALIGN(pic_width, 16) * 2) * 16));

		sizeimage[1] =
			(c_pitch_w_div16) * (((pic_height + 31) / 32) * 2) *
			64 + ((c_pitch_w_div16 % 8 == 0) ?
			0 : ((ALIGN(pic_width, 16) / 2) * 16));

		sizeimage[2] =
			(c_pitch_w_div16) * (((pic_height + 31) / 32) * 2) *
			64 + ((c_pitch_w_div16 % 8 == 0) ?
			0 : ((ALIGN(pic_width, 16) / 2) * 16));

		bytesperline[0] = ALIGN(pic_width, 16);
		bytesperline[1] = ALIGN(pic_width, 16) / 2;
		bytesperline[2] = ALIGN(pic_width, 16) / 2;
	}
}

static int vidioc_try_fmt(struct v4l2_format *f, struct mtk_video_fmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;

	/* V4L2 specification suggests the driver corrects the format struct
	  * if any of the dimensions is unsupported
	  */
	pix_fmt_mp->height = clamp(pix_fmt_mp->height, MTK_VENC_MIN_H,
							MTK_VENC_MAX_H);
	pix_fmt_mp->width = clamp(pix_fmt_mp->width, MTK_VENC_MIN_W,
							MTK_VENC_MAX_W);

	pix_fmt_mp->field = V4L2_FIELD_NONE;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		int size = pix_fmt_mp->height * pix_fmt_mp->width;

		if (fmt->num_planes != pix_fmt_mp->num_planes)
			pix_fmt_mp->num_planes = fmt->num_planes;

		if (pix_fmt_mp->plane_fmt[0].sizeimage != size)
			pix_fmt_mp->plane_fmt[0].sizeimage = size;
		pix_fmt_mp->plane_fmt[0].bytesperline = 0;
		memset(&(pix_fmt_mp->plane_fmt[0].reserved[0]), 0x0,
		       sizeof(pix_fmt_mp->plane_fmt[0].reserved));
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		int i;
		unsigned int sizeimage[VIDEO_MAX_PLANES];
		unsigned int bytesperline[VIDEO_MAX_PLANES];

		v4l_bound_align_image(&pix_fmt_mp->width, 8, 1920, 1,
				      &pix_fmt_mp->height, 4, 1080, 1, 0);

		if (fmt->num_planes != pix_fmt_mp->num_planes)
			pix_fmt_mp->num_planes = fmt->num_planes;

		mtk_vcodec_enc_calc_src_size(pix_fmt_mp->num_planes,
					     pix_fmt_mp->width,
					     pix_fmt_mp->height,
					     sizeimage, bytesperline);

		for (i = 0; i < pix_fmt_mp->num_planes; i++) {
			pix_fmt_mp->plane_fmt[i].sizeimage = sizeimage[i];
			pix_fmt_mp->plane_fmt[i].bytesperline = bytesperline[i];
			memset(&(pix_fmt_mp->plane_fmt[i].reserved[0]), 0x0,
			       sizeof(pix_fmt_mp->plane_fmt[0].reserved));
		}
	} else {
		return -EINVAL;
	}

	memset(&pix_fmt_mp->flags, 0, sizeof(*pix_fmt_mp) -
			offsetof(struct v4l2_pix_format_mplane, flags));

	memset(&pix_fmt_mp->reserved[0], 0x0, sizeof(pix_fmt_mp->reserved));

	return 0;
}

static void mtk_venc_set_param(struct mtk_vcodec_ctx *ctx, void *param)
{
	struct venc_enc_prm *p = (struct venc_enc_prm *)param;
	struct mtk_q_data *q_data_src = &ctx->q_data[MTK_Q_DATA_SRC];
	struct mtk_enc_params *enc_params = &ctx->enc_params;
	unsigned int frame_rate;

	frame_rate = enc_params->framerate_num / enc_params->framerate_denom;

	switch (q_data_src->fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YUV420M:
		p->input_fourcc = VENC_YUV_FORMAT_420;
		break;
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YVU420M:
		p->input_fourcc = VENC_YUV_FORMAT_YV12;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
		p->input_fourcc = VENC_YUV_FORMAT_NV12;
		break;
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		p->input_fourcc = VENC_YUV_FORMAT_NV21;
		break;
	}
	p->h264_profile = enc_params->h264_profile;
	p->h264_level = enc_params->h264_level;
	p->width = q_data_src->width;
	p->height = q_data_src->height;
	p->buf_width = q_data_src->bytesperline[0];
	p->buf_height = ((q_data_src->height + 0xf) & (~0xf));
	p->frm_rate = frame_rate;
	p->intra_period = enc_params->gop_size;
	p->bitrate = enc_params->bitrate;

	ctx->param_change = MTK_ENCODE_PARAM_NONE;

	mtk_v4l2_debug(1, "fmt 0x%x, P/L %d/%d, w/h %d/%d, buf %d/%d, fps/bps %d/%d, gop %d",
		       p->input_fourcc, p->h264_profile, p->h264_level,
		       p->width, p->height, p->buf_width, p->buf_height,
		       p->frm_rate, p->bitrate, p->intra_period);
}

static int vidioc_venc_s_fmt_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct mtk_q_data *q_data;
	int i, ret;
	struct mtk_video_fmt *fmt;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq) {
		mtk_v4l2_err("fail to get vq\n");
		return -EINVAL;
	}

	if (vb2_is_busy(vq)) {
		mtk_v4l2_err("queue busy\n");
		return -EBUSY;
	}

	q_data = mtk_venc_get_q_data(ctx, f->type);
	if (!q_data) {
		mtk_v4l2_err("fail to get q data\n");
		return -EINVAL;
	}

	fmt = mtk_venc_find_format(f);
	if (!fmt) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			f->fmt.pix.pixelformat = mtk_video_formats[0].fourcc;
			fmt = mtk_venc_find_format(f);
		} else {
			f->fmt.pix.pixelformat = mtk_video_formats[8].fourcc;
			fmt = mtk_venc_find_format(f);
		}
	}

	q_data->fmt = fmt;
	ret = vidioc_try_fmt(f, q_data->fmt);
	if (ret)
		return ret;

	q_data->width		= f->fmt.pix_mp.width;
	q_data->height		= f->fmt.pix_mp.height;
	q_data->field		= f->fmt.pix_mp.field;

	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		struct v4l2_plane_pix_format	*plane_fmt;

		plane_fmt = &f->fmt.pix_mp.plane_fmt[i];
		q_data->bytesperline[i]	= plane_fmt->bytesperline;
		q_data->sizeimage[i]	= plane_fmt->sizeimage;
	}

	if (ctx->state == MTK_STATE_FREE) {
		ret = venc_if_create(ctx, q_data->fmt->fourcc);
		if (ret) {
			mtk_v4l2_err("venc_if_create failed=%d, codec type=%x\n",
						ret, q_data->fmt->fourcc);
			return 0;
		}

		ctx->state = MTK_STATE_INIT;
	}

	return 0;

}

static int vidioc_venc_s_fmt_out(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	struct vb2_queue *vq;
	struct mtk_q_data *q_data;
	struct venc_enc_prm param;
	int i, ret;
	struct mtk_video_fmt *fmt;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq) {
		mtk_v4l2_err("fail to get vq\n");
		return -EINVAL;
	}

	if (vb2_is_busy(vq)) {
		mtk_v4l2_err("queue busy\n");
		return -EBUSY;
	}

	q_data = mtk_venc_get_q_data(ctx, f->type);
	if (!q_data) {
		mtk_v4l2_err("fail to get q data\n");
		return -EINVAL;
	}

	fmt = mtk_venc_find_format(f);
	if (!fmt) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			f->fmt.pix.pixelformat = mtk_video_formats[0].fourcc;
			fmt = mtk_venc_find_format(f);
		} else {
			f->fmt.pix.pixelformat = mtk_video_formats[8].fourcc;
			fmt = mtk_venc_find_format(f);
		}
	}

	q_data->fmt = fmt;
	ret = vidioc_try_fmt(f, q_data->fmt);
	if (ret)
		return ret;

	q_data->width		= f->fmt.pix_mp.width;
	q_data->height		= f->fmt.pix_mp.height;
	q_data->field		= f->fmt.pix_mp.field;

		q_data->colorspace	= f->fmt.pix_mp.colorspace;
		ctx->q_data[MTK_Q_DATA_SRC].bytesperline[0] =
			ALIGN(q_data->width, 16);

		if (q_data->fmt->num_planes == 2) {
			ctx->q_data[MTK_Q_DATA_SRC].bytesperline[1] =
				ALIGN(q_data->width, 16);
			ctx->q_data[MTK_Q_DATA_SRC].bytesperline[2] = 0;
		} else {
			ctx->q_data[MTK_Q_DATA_SRC].bytesperline[1] =
				ALIGN(q_data->width, 16) / 2;
			ctx->q_data[MTK_Q_DATA_SRC].bytesperline[2] =
				ALIGN(q_data->width, 16) / 2;
		}

		memset(&param, 0, sizeof(param));
		mtk_venc_set_param(ctx, &param);
		if (ctx->state == MTK_STATE_INIT) {
			ret = venc_if_set_param(ctx,
						VENC_SET_PARAM_ENC,
						&param);
			if (ret)
				mtk_v4l2_err("venc_if_set_param failed=%d\n",
					     ret);

			/* Get codec driver advice sizeimage from vpu */
			for (i = 0; i < MTK_VCODEC_MAX_PLANES; i++) {
				q_data->sizeimage[i] = param.sizeimage[i];
				pix_fmt_mp->plane_fmt[i].sizeimage =
					param.sizeimage[i];
			}
			q_data->bytesperline[0] =
				pix_fmt_mp->plane_fmt[0].bytesperline;
			q_data->bytesperline[1] =
				pix_fmt_mp->plane_fmt[1].bytesperline;
			q_data->bytesperline[2] =
				pix_fmt_mp->plane_fmt[2].bytesperline;
		} else {
			for (i = 0; i < MTK_VCODEC_MAX_PLANES; i++) {
				q_data->sizeimage[i] =
					pix_fmt_mp->plane_fmt[i].sizeimage;
				q_data->bytesperline[i] =
					pix_fmt_mp->plane_fmt[i].bytesperline;
		}
	}

	return 0;
}

static int vidioc_venc_g_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct mtk_q_data *q_data;
	int i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = mtk_venc_get_q_data(ctx, f->type);

	pix->width = q_data->width;
	pix->height = q_data->height;
	pix->pixelformat = q_data->fmt->fourcc;
	pix->field = q_data->field;
	pix->colorspace = q_data->colorspace;
	pix->num_planes = q_data->fmt->num_planes;
	for (i = 0; i < pix->num_planes; i++) {
		pix->plane_fmt[i].bytesperline = q_data->bytesperline[i];
		pix->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		memset(&(pix->plane_fmt[i].reserved[0]), 0x0,
		       sizeof(pix->plane_fmt[i].reserved));
	}

	memset(&pix->flags, 0, sizeof(pix) -
			offsetof(struct v4l2_pix_format_mplane, flags));
	memset(&pix->reserved[0], 0x0, sizeof(pix->reserved));

	return 0;
}

static int vidioc_try_fmt_vid_cap_mplane(struct file *file, void *priv,
					 struct v4l2_format *f)
{
	struct mtk_video_fmt *fmt;
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);

	fmt = mtk_venc_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = mtk_video_formats[8].fourcc;
		fmt = mtk_venc_find_format(f);
	}
	if (fmt->type != MTK_FMT_ENC) {
		mtk_v4l2_err("Fourcc format (0x%08x) invalid.\n",
			     f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	f->fmt.pix_mp.colorspace = ctx->q_data[MTK_Q_DATA_SRC].colorspace;

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_try_fmt_vid_out_mplane(struct file *file, void *priv,
					 struct v4l2_format *f)
{
	struct mtk_video_fmt *fmt;

	fmt = mtk_venc_find_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = mtk_video_formats[0].fourcc;
		fmt = mtk_venc_find_format(f);
	}
	if (!(fmt->type & MTK_FMT_FRAME)) {
		mtk_v4l2_err("Fourcc format (0x%08x) invalid.\n",
			     f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	if (!f->fmt.pix_mp.colorspace)
		f->fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_venc_g_s_selection(struct file *file, void *priv,
				     struct v4l2_selection *s)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);
	struct mtk_q_data *q_data;

	if (V4L2_TYPE_IS_OUTPUT(s->type)) {
		if (s->target !=  V4L2_SEL_TGT_COMPOSE)
			return -EINVAL;
	} else {
		if (s->target != V4L2_SEL_TGT_CROP)
			return -EINVAL;
	}

	if (s->r.left || s->r.top)
		return -EINVAL;

	q_data = mtk_venc_get_q_data(ctx, s->type);
	if (!q_data)
		return -EINVAL;

	s->r.width = q_data->width;
	s->r.height = q_data->height;

	return 0;
}


static int vidioc_venc_qbuf(struct file *file, void *priv,
			    struct v4l2_buffer *buf)
{

	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);

	if (ctx->state == MTK_STATE_ABORT) {
		mtk_v4l2_err("[%d] Call on QBUF after unrecoverable error\n",
					 ctx->idx);
		return -EIO;
	}

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_venc_dqbuf(struct file *file, void *priv,
			     struct v4l2_buffer *buf)
{
	struct mtk_vcodec_ctx *ctx = fh_to_ctx(priv);

	if (ctx->state == MTK_STATE_ABORT) {
		mtk_v4l2_err("[%d] Call on QBUF after unrecoverable error\n",
					 ctx->idx);
		return -EIO;
	}

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

const struct v4l2_ioctl_ops mtk_venc_ioctl_ops = {
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= vidioc_venc_qbuf,
	.vidioc_dqbuf			= vidioc_venc_dqbuf,

	.vidioc_querycap		= vidioc_venc_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = vidioc_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out_mplane = vidioc_enum_fmt_vid_out_mplane,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes,

	.vidioc_try_fmt_vid_cap_mplane	= vidioc_try_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_out_mplane	= vidioc_try_fmt_vid_out_mplane,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,

	.vidioc_s_parm			= vidioc_venc_s_parm,
	.vidioc_g_parm	= vidioc_venc_g_parm,
	.vidioc_s_fmt_vid_cap_mplane	= vidioc_venc_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane	= vidioc_venc_s_fmt_out,

	.vidioc_g_fmt_vid_cap_mplane	= vidioc_venc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane	= vidioc_venc_g_fmt,

	.vidioc_create_bufs   = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf   = v4l2_m2m_ioctl_prepare_buf,

	.vidioc_g_selection		= vidioc_venc_g_s_selection,
	.vidioc_s_selection		= vidioc_venc_g_s_selection,
};

static int vb2ops_venc_queue_setup(struct vb2_queue *vq,
				   const struct v4l2_format *fmt,
				   unsigned int *nbuffers,
				   unsigned int *nplanes,
				   unsigned int sizes[], void *alloc_ctxs[])
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vq);
	struct mtk_q_data *q_data;
	unsigned int i;

	q_data = mtk_venc_get_q_data(ctx, vq->type);

	if (q_data == NULL)
		return -EINVAL;

	if (vq->num_buffers >= MTK_VIDEO_MAX_FRAME)
		return -EINVAL;

	if (vq->num_buffers + *nbuffers > MTK_VIDEO_MAX_FRAME)
		*nbuffers = MTK_VIDEO_MAX_FRAME - vq->num_buffers;

	if (fmt) {
		/* When called from VIDIOC_CREATE_BUFS, fmt != NULL and it
		  * describes the target frame format (if the format isn't valid
		  * the callback must return -EINVAL). In this case *num_buffers
		  * are being allocated additionally to q->num_buffers
		  */
		*nplanes = fmt->fmt.pix_mp.num_planes;
		for (i = 0; i < *nplanes; i++) {
			sizes[i] = fmt->fmt.pix_mp.plane_fmt[i].sizeimage;
			alloc_ctxs[i] = ctx->dev->alloc_ctx;
		}
	} else {
		/* When called from VIDIOC_REQBUFS, fmt == NULL, the driver
		  * has to use the currently configured format and *num_buffers
		  * is the total number of buffers, that are being allocated
		  */
		*nplanes = q_data->fmt->num_planes;
		for (i = 0; i < *nplanes; i++) {
			sizes[i] = q_data->sizeimage[i];
			alloc_ctxs[i] = ctx->dev->alloc_ctx;
		}
	}
	return 0;
}

static int vb2ops_venc_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_q_data *q_data;
	int i;

	q_data = mtk_venc_get_q_data(ctx, vb->vb2_queue->type);

	for (i = 0; i < q_data->fmt->num_planes; i++) {
		if (vb2_plane_size(vb, i) < q_data->sizeimage[i]) {
			mtk_v4l2_debug(2, "data will not fit into plane %d (%lu < %d)",
				       i, vb2_plane_size(vb, i),
				       q_data->sizeimage[i]);
			return -EINVAL;
		}
	}

	return 0;
}

static void vb2ops_venc_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_video_enc_buf *mtk_buf =
			container_of(vb, struct mtk_video_enc_buf, vb);

	if ((vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) &&
	    (ctx->param_change != MTK_ENCODE_PARAM_NONE)) {
		mtk_v4l2_debug(1, "[%d] Before id=%d encode parameter change %x",
			       ctx->idx, vb->v4l2_buf.index,
			       ctx->param_change);
		mtk_buf->param_change = ctx->param_change;
		if (mtk_buf->param_change & MTK_ENCODE_PARAM_BITRATE) {
			mtk_buf->enc_params.bitrate = ctx->enc_params.bitrate;
			mtk_v4l2_debug(1, "[%d] idx=%d change param br=%d",
				       ctx->idx,
				       vb->v4l2_buf.index,
				       mtk_buf->enc_params.bitrate);
		}
		if (ctx->param_change & MTK_ENCODE_PARAM_FRAMERATE) {
			mtk_buf->enc_params.framerate_num =
				ctx->enc_params.framerate_num;
			mtk_buf->enc_params.framerate_denom =
				ctx->enc_params.framerate_denom;
			mtk_v4l2_debug(1, "[%d] idx=%d, change param fr=%d/%d",
				       ctx->idx,
				       vb->v4l2_buf.index,
				       mtk_buf->enc_params.framerate_num,
				       mtk_buf->enc_params.framerate_denom);
		}
		if (ctx->param_change & MTK_ENCODE_PARAM_INTRA_PERIOD) {
			mtk_buf->enc_params.gop_size = ctx->enc_params.gop_size;
			mtk_v4l2_debug(1, "[%d] idx=%d, change param intra period=%d",
				       ctx->idx,
				       vb->v4l2_buf.index,
				       mtk_buf->enc_params.gop_size);
		}
		if (ctx->param_change & MTK_ENCODE_PARAM_FRAME_TYPE) {
			mtk_buf->enc_params.force_intra =
				ctx->enc_params.force_intra;
			mtk_v4l2_debug(1, "[%d] idx=%d, change param force I=%d",
				       ctx->idx,
				       vb->v4l2_buf.index,
				       mtk_buf->enc_params.force_intra);
		}
		ctx->param_change = MTK_ENCODE_PARAM_NONE;
	}

	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
}

static int vb2ops_venc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(q);
	struct venc_enc_prm param;
	int ret;
	int i;

	/* Once state turn into MTK_STATE_ABORT, we need stop_streaming
	  * to clear it
	  */
	if ((ctx->state == MTK_STATE_ABORT) || (ctx->state == MTK_STATE_FREE))
		goto err_set_param;

	if (!(vb2_start_streaming_called(&ctx->m2m_ctx->out_q_ctx.q) &
	      vb2_start_streaming_called(&ctx->m2m_ctx->cap_q_ctx.q))) {
		mtk_v4l2_debug(1, "[%d]-> out=%d cap=%d",
		       ctx->idx,
		       vb2_start_streaming_called(&ctx->m2m_ctx->out_q_ctx.q),
		       vb2_start_streaming_called(&ctx->m2m_ctx->cap_q_ctx.q));
		return 0;
	}

	mtk_venc_set_param(ctx, &param);
	ret = venc_if_set_param(ctx,
				VENC_SET_PARAM_ENC,
				&param);
	if (ret) {
		mtk_v4l2_err("venc_if_set_param failed=%d\n", ret);
		ctx->state = MTK_STATE_ABORT;
		goto err_set_param;
	}

	if ((ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc == V4L2_PIX_FMT_H264) &&
	    (ctx->enc_params.seq_hdr_mode !=
				V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE)) {
		ret = venc_if_set_param(ctx,
					VENC_SET_PARAM_PREPEND_HEADER,
					0);
		if (ret) {
			mtk_v4l2_err("venc_if_set_param failed=%d\n", ret);
			ctx->state = MTK_STATE_ABORT;
			goto err_set_param;
		}
		ctx->state = MTK_STATE_HEADER;
	}

	return 0;

err_set_param:
	for (i = 0; i < q->num_buffers; ++i) {
		if (q->bufs[i]->state == VB2_BUF_STATE_ACTIVE) {
			mtk_v4l2_debug(0, "[%d] idx=%d, type=%d, %d -> VB2_BUF_STATE_QUEUED",
					ctx->idx, i, q->type,
					(int)q->bufs[i]->state);
			v4l2_m2m_buf_done(q->bufs[i], VB2_BUF_STATE_QUEUED);
		}
	}

	return -EINVAL;
}

static void vb2ops_venc_stop_streaming(struct vb2_queue *q)
{
	struct mtk_vcodec_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_buffer *src_buf, *dst_buf;
	int ret;

	mtk_v4l2_debug(2, "[%d]-> type=%d", ctx->idx, q->type);

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		while ((dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx))) {
			dst_buf->v4l2_planes[0].bytesused = 0;
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		}
	} else {
		while ((src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx)))
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
	}

	if ((q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	     vb2_is_streaming(&ctx->m2m_ctx->out_q_ctx.q)) ||
	    (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	     vb2_is_streaming(&ctx->m2m_ctx->cap_q_ctx.q))) {
		mtk_v4l2_debug(1, "[%d]-> q type %d out=%d cap=%d",
			       ctx->idx, q->type,
			       vb2_is_streaming(&ctx->m2m_ctx->out_q_ctx.q),
			       vb2_is_streaming(&ctx->m2m_ctx->cap_q_ctx.q));
		return;
	}

	ret = venc_if_release(ctx);
	if (ret)
		mtk_v4l2_err("venc_if_release failed=%d\n", ret);

	ctx->state = MTK_STATE_FREE;
}

static struct vb2_ops mtk_venc_vb2_ops = {
	.queue_setup		= vb2ops_venc_queue_setup,
	.buf_prepare		= vb2ops_venc_buf_prepare,
	.buf_queue			= vb2ops_venc_buf_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= vb2ops_venc_start_streaming,
	.stop_streaming		= vb2ops_venc_stop_streaming,
};

static int mtk_venc_encode_header(void *priv)
{
	struct mtk_vcodec_ctx *ctx = priv;
	struct vb2_buffer *dst_buf;
	struct mtk_vcodec_mem bs_buf;
	struct venc_done_result enc_result;
	int ret;

	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	if (!dst_buf) {
		mtk_v4l2_debug(1, "No dst buffer");
		return -EINVAL;
	}

	bs_buf.va = vb2_plane_vaddr(dst_buf, 0);
	bs_buf.dma_addr = vb2_dma_contig_plane_dma_addr(dst_buf, 0);
	bs_buf.size = (unsigned int)dst_buf->v4l2_planes[0].length;

	mtk_v4l2_debug(1, "[%d] buf idx=%d va=0x%p dma_addr=0x%llx size=0x%lx",
		       ctx->idx,
		       dst_buf->v4l2_buf.index,
		       bs_buf.va,
		       (u64)bs_buf.dma_addr,
		       bs_buf.size);

	ret = venc_if_encode(ctx, VENC_START_OPT_ENCODE_SEQUENCE_HEADER,
			     0, &bs_buf, &enc_result);

	if (ret) {
		dst_buf->v4l2_planes[0].bytesused = 0;
		ctx->state = MTK_STATE_ABORT;
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		mtk_v4l2_err("venc_if_encode failed=%d", ret);
		return -EINVAL;
	}

	ctx->state = MTK_STATE_HEADER;
	dst_buf->v4l2_planes[0].bytesused = enc_result.bs_size;

#if defined(DEBUG)
{
	int i;

	mtk_v4l2_debug(1, "[%d] venc_if_encode header len=%d",
		       ctx->idx,
		       enc_result.bs_size);
	for (i = 0; i < enc_result.bs_size; i++) {
		unsigned char *p = (unsigned char *)bs_buf.va;

		mtk_v4l2_debug(1, "[%d] buf[%d]=0x%2x", ctx->idx, i, p[i]);
	}
}
#endif
	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

	return 0;
}

static int mtk_venc_param_change(struct mtk_vcodec_ctx *ctx, void *priv)
{
	struct vb2_buffer *vb = priv;
	struct mtk_video_enc_buf *mtk_buf =
			container_of(vb, struct mtk_video_enc_buf, vb);
	int ret = 0;

	if (mtk_buf->param_change == MTK_ENCODE_PARAM_NONE)
		return 0;

	mtk_v4l2_debug(1, "encode parameters change id=%d", vb->v4l2_buf.index);

	if (mtk_buf->param_change & MTK_ENCODE_PARAM_BITRATE) {
		struct venc_enc_prm enc_prm;

		enc_prm.bitrate = mtk_buf->enc_params.bitrate;
		mtk_v4l2_debug(1, "[%d] idx=%d, change param br=%d",
			       ctx->idx,
			       vb->v4l2_buf.index,
			       enc_prm.bitrate);
		ret |= venc_if_set_param(ctx,
					 VENC_SET_PARAM_ADJUST_BITRATE,
					 &enc_prm);
	}
	if (mtk_buf->param_change & MTK_ENCODE_PARAM_FRAMERATE) {
		struct venc_enc_prm enc_prm;

		enc_prm.frm_rate = mtk_buf->enc_params.framerate_num /
				   mtk_buf->enc_params.framerate_denom;
		mtk_v4l2_debug(1, "[%d] idx=%d, change param fr=%d",
			       ctx->idx,
			       vb->v4l2_buf.index,
			       enc_prm.frm_rate);
		ret |= venc_if_set_param(ctx,
					 VENC_SET_PARAM_ADJUST_FRAMERATE,
					 &enc_prm);
	}
	if (mtk_buf->param_change & MTK_ENCODE_PARAM_INTRA_PERIOD) {
		struct venc_enc_prm enc_prm;

		enc_prm.gop_size = mtk_buf->enc_params.gop_size;
		mtk_v4l2_debug(1, "change param intra period=%d",
			       enc_prm.gop_size);
		ret |= venc_if_set_param(ctx,
					 VENC_SET_PARAM_I_FRAME_INTERVAL,
					 &enc_prm);
	}
	if (mtk_buf->param_change & MTK_ENCODE_PARAM_FRAME_TYPE) {
		mtk_v4l2_debug(1, "[%d] idx=%d, change param force I=%d",
			       ctx->idx,
			       vb->v4l2_buf.index,
			       mtk_buf->enc_params.force_intra);
		if (mtk_buf->enc_params.force_intra)
			ret |= venc_if_set_param(ctx,
						 VENC_SET_PARAM_FORCE_INTRA,
						 0);
	}

	mtk_buf->param_change = MTK_ENCODE_PARAM_NONE;

	if (ret) {
		ctx->state = MTK_STATE_ABORT;
		mtk_v4l2_err("venc_if_set_param %d failed=%d\n",
			     MTK_ENCODE_PARAM_FRAME_TYPE, ret);
		return -1;
	}

	return 0;
}

static void mtk_venc_worker(struct work_struct *work)
{
	struct mtk_vcodec_ctx *ctx = container_of(work, struct mtk_vcodec_ctx,
				    encode_work);
	struct vb2_buffer *src_buf, *dst_buf;
	struct venc_frm_buf frm_buf;
	struct mtk_vcodec_mem bs_buf;
	struct venc_done_result enc_result;
	int ret;

	if ((ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc == V4L2_PIX_FMT_H264) &&
	    (ctx->state != MTK_STATE_HEADER)) {
		/* encode h264 sps/pps header */
		mtk_venc_encode_header(ctx);
		v4l2_m2m_job_finish(ctx->dev->m2m_dev_enc, ctx->m2m_ctx);
		return;
	}

	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	if (!src_buf) {
		v4l2_m2m_job_finish(ctx->dev->m2m_dev_enc, ctx->m2m_ctx);
		return;
	}

	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	if (!dst_buf) {
		v4l2_m2m_job_finish(ctx->dev->m2m_dev_enc, ctx->m2m_ctx);
		return;
	}

	mtk_venc_param_change(ctx, src_buf);

	frm_buf.fb_addr.va = vb2_plane_vaddr(src_buf, 0);
	frm_buf.fb_addr.dma_addr = vb2_dma_contig_plane_dma_addr(src_buf, 0);
	frm_buf.fb_addr.size = (unsigned int)src_buf->v4l2_planes[0].length;
	frm_buf.fb_addr1.va = vb2_plane_vaddr(src_buf, 1);
	frm_buf.fb_addr1.dma_addr = vb2_dma_contig_plane_dma_addr(src_buf, 1);
	frm_buf.fb_addr1.size = (unsigned int)src_buf->v4l2_planes[1].length;
	if (src_buf->num_planes == 3) {
		frm_buf.fb_addr2.va = vb2_plane_vaddr(src_buf, 2);
		frm_buf.fb_addr2.dma_addr =
			vb2_dma_contig_plane_dma_addr(src_buf, 2);
		frm_buf.fb_addr2.size =
			(unsigned int)src_buf->v4l2_planes[2].length;
	} else {
		frm_buf.fb_addr2.va = NULL;
		frm_buf.fb_addr2.dma_addr = 0;
		frm_buf.fb_addr2.size = 0;
	}
	bs_buf.va = vb2_plane_vaddr(dst_buf, 0);
	bs_buf.dma_addr = vb2_dma_contig_plane_dma_addr(dst_buf, 0);
	bs_buf.size = (unsigned int)dst_buf->v4l2_planes[0].length;

	mtk_v4l2_debug(2, "Framebuf VA=%p PA=%llx Size=0x%lx;VA=%p PA=0x%llx Size=0x%lx;VA=%p PA=0x%llx Size=0x%lx",
		       frm_buf.fb_addr.va,
		       (u64)frm_buf.fb_addr.dma_addr,
		       frm_buf.fb_addr.size,
		       frm_buf.fb_addr1.va,
		       (u64)frm_buf.fb_addr1.dma_addr,
		       frm_buf.fb_addr1.size,
		       frm_buf.fb_addr2.va,
		       (u64)frm_buf.fb_addr2.dma_addr,
		       frm_buf.fb_addr2.size);

	ret = venc_if_encode(ctx, VENC_START_OPT_ENCODE_FRAME,
			     &frm_buf, &bs_buf, &enc_result);

	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	if (ret)
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
	else
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);

	if (enc_result.is_key_frm)
		dst_buf->v4l2_buf.flags |= V4L2_BUF_FLAG_KEYFRAME;

	if (ret) {
		dst_buf->v4l2_planes[0].bytesused = 0;
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		mtk_v4l2_err("venc_if_encode failed=%d", ret);
	} else {
		dst_buf->v4l2_planes[0].bytesused = enc_result.bs_size;
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		mtk_v4l2_debug(2, "venc_if_encode bs size=%d",
			       enc_result.bs_size);
	}

	v4l2_m2m_job_finish(ctx->dev->m2m_dev_enc, ctx->m2m_ctx);

	mtk_v4l2_debug(1, "<=== src_buf[%d] dst_buf[%d] venc_if_encode ret=%d Size=%u===>",
		       src_buf->v4l2_buf.index, dst_buf->v4l2_buf.index, ret,
		       enc_result.bs_size);
}

static void m2mops_venc_device_run(void *priv)
{
	struct mtk_vcodec_ctx *ctx = priv;

	queue_work(ctx->dev->encode_workqueue, &ctx->encode_work);
}

static int m2mops_venc_job_ready(void *m2m_priv)
{
	struct mtk_vcodec_ctx *ctx = m2m_priv;

	if (!v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx)) {
		mtk_v4l2_debug(3, "[%d]Not ready: not enough video dst buffers.",
			       ctx->idx);
		return 0;
	}

	if (!v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx)) {
		mtk_v4l2_debug(3, "[%d]Not ready: not enough video src buffers.",
			       ctx->idx);
		return 0;
	}

	if (ctx->state == MTK_STATE_ABORT) {
		mtk_v4l2_debug(3, "[%d]Not ready: state=0x%x.",
			       ctx->idx, ctx->state);
		return 0;
	}

	if (ctx->state == MTK_STATE_FREE) {
		mtk_v4l2_debug(3, "[%d]Not ready: state=0x%x.",
			       ctx->idx, ctx->state);
		return 0;
	}

	return 1;
}

static void m2mops_venc_job_abort(void *priv)
{
	struct mtk_vcodec_ctx *ctx = priv;

	ctx->state = MTK_STATE_ABORT;
}

static void m2mops_venc_lock(void *m2m_priv)
{
	struct mtk_vcodec_ctx *ctx = m2m_priv;

	mutex_lock(&ctx->dev->dev_mutex);
}

static void m2mops_venc_unlock(void *m2m_priv)
{
	struct mtk_vcodec_ctx *ctx = m2m_priv;

	mutex_unlock(&ctx->dev->dev_mutex);
}

const struct v4l2_m2m_ops mtk_venc_m2m_ops = {
	.device_run			= m2mops_venc_device_run,
	.job_ready			= m2mops_venc_job_ready,
	.job_abort			= m2mops_venc_job_abort,
	.lock				= m2mops_venc_lock,
	.unlock				= m2mops_venc_unlock,
};

void mtk_vcodec_enc_ctx_params_setup(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_q_data *q_data;
	struct mtk_video_fmt *fmt;

	ctx->m2m_ctx->q_lock = &ctx->dev->dev_mutex;
	ctx->fh.m2m_ctx = ctx->m2m_ctx;
	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	INIT_WORK(&ctx->encode_work, mtk_venc_worker);

	ctx->q_data[MTK_Q_DATA_SRC].width = DFT_CFG_WIDTH;
	ctx->q_data[MTK_Q_DATA_SRC].height = DFT_CFG_HEIGHT;
	ctx->q_data[MTK_Q_DATA_SRC].fmt = &mtk_video_formats[0];
	ctx->q_data[MTK_Q_DATA_SRC].colorspace = V4L2_COLORSPACE_REC709;
	ctx->q_data[MTK_Q_DATA_SRC].field = V4L2_FIELD_NONE;

	q_data = &ctx->q_data[MTK_Q_DATA_SRC];
	fmt = ctx->q_data[MTK_Q_DATA_SRC].fmt;
	mtk_vcodec_enc_calc_src_size(fmt->num_planes, q_data->width,
			q_data->height,
				     ctx->q_data[MTK_Q_DATA_SRC].sizeimage,
				     ctx->q_data[MTK_Q_DATA_SRC].bytesperline);

	ctx->q_data[MTK_Q_DATA_DST].width = DFT_CFG_WIDTH;
	ctx->q_data[MTK_Q_DATA_DST].height = DFT_CFG_HEIGHT;
	ctx->q_data[MTK_Q_DATA_DST].fmt = &mtk_video_formats[9];
	ctx->q_data[MTK_Q_DATA_DST].colorspace = V4L2_COLORSPACE_REC709;
	ctx->q_data[MTK_Q_DATA_DST].field = V4L2_FIELD_NONE;

	q_data = &ctx->q_data[MTK_Q_DATA_DST];
	fmt = ctx->q_data[MTK_Q_DATA_DST].fmt;
	ctx->q_data[MTK_Q_DATA_DST].sizeimage[0] =
			q_data->width * q_data->height;
	ctx->q_data[MTK_Q_DATA_DST].bytesperline[0] = 0;

}

int mtk_vcodec_enc_ctrls_setup(struct mtk_vcodec_ctx *ctx)
{
	const struct v4l2_ctrl_ops *ops = &mtk_vcodec_enc_ctrl_ops;
	struct v4l2_ctrl_handler *handler = &ctx->ctrl_hdl;

	v4l2_ctrl_handler_init(handler, MTK_MAX_CTRLS);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_BITRATE,
					1, 4000000, 1, 4000000);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_B_FRAMES,
					0, 2, 1, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
					0, 1, 1, 1);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
					0, 51, 1, 51);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
					0, 65535, 1, 30);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
					0, 65535, 1, 30);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE,
					0, 1, 1, 0);
	v4l2_ctrl_new_std_menu(handler, ops,
			V4L2_CID_MPEG_VIDEO_HEADER_MODE,
			V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
			0, V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE);
	v4l2_ctrl_new_std_menu(handler, ops, V4L2_CID_MPEG_VIDEO_H264_PROFILE,
					V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
					0, V4L2_MPEG_VIDEO_H264_PROFILE_MAIN);
	v4l2_ctrl_new_std_menu(handler, ops, V4L2_CID_MPEG_VIDEO_H264_LEVEL,
					V4L2_MPEG_VIDEO_H264_LEVEL_4_2,
					0, V4L2_MPEG_VIDEO_H264_LEVEL_4_0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
					0, 0, 0, 0);

	if (handler->error) {
		mtk_v4l2_err("Init control handler fail %d\n",
				handler->error);
		return handler->error;
	}

	v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);

	return 0;
}

int mtk_vcodec_enc_queue_init(void *priv, struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	struct mtk_vcodec_ctx *ctx = priv;
	int ret;

	src_vq->type		= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes	= VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv	= ctx;
	src_vq->buf_struct_size = sizeof(struct mtk_video_enc_buf);
	src_vq->ops		= &mtk_venc_vb2_ops;
	src_vq->mem_ops		= &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dev->dev_mutex;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes	= VB2_DMABUF | VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv	= ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops		= &mtk_venc_vb2_ops;
	dst_vq->mem_ops		= &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dev->dev_mutex;

	return vb2_queue_init(dst_vq);
}

int mtk_venc_unlock(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_vcodec_dev *dev = ctx->dev;

	dev->curr_ctx = -1;
	mutex_unlock(&dev->enc_mutex);
	return 0;
}

int mtk_venc_lock(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_vcodec_dev *dev = ctx->dev;

	mutex_lock(&dev->enc_mutex);
	dev->curr_ctx = ctx->idx;
	return 0;
}

void mtk_vcodec_enc_release(struct mtk_vcodec_ctx *ctx)
{
	venc_if_release(ctx);
}
