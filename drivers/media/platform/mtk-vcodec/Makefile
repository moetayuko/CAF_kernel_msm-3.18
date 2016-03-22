
obj-$(CONFIG_VIDEO_MEDIATEK_VCODEC) += mtk-vcodec-dec.o \
				       mtk-vcodec-enc.o \
				       mtk-vcodec-common.o

mtk-vcodec-dec-y := vp9_dec/vdec_vp9_core.o \
		vp9_dec/vdec_vp9_if.o \
		vp9_dec/vdec_vp9_debug.o \
		vp9_dec/vdec_vp9_vpu.o \
		vp8_dec/vdec_vp8_if.o \
		vp8_dec/vdec_vp8_vpu.o \
		h264_dec/vdec_h264_if.o \
		h264_dec/vdec_h264_vpu.o \
		mtk_vcodec_dec_drv.o \
		vdec_drv_if.o \
		mtk_vcodec_dec.o \
		mtk_vcodec_dec_pm.o \

mtk-vcodec-enc-y := h264_enc/venc_h264_if.o \
		h264_enc/venc_h264_vpu.o \
		vp8_enc/venc_vp8_if.o \
		vp8_enc/venc_vp8_vpu.o \
		mtk_vcodec_enc.o \
		mtk_vcodec_enc_drv.o \
		mtk_vcodec_enc_pm.o \
		venc_drv_if.o \


mtk-vcodec-common-y := mtk_vcodec_intr.o \
		mtk_vcodec_util.o\


ccflags-y += -I$(srctree)/drivers/media/platform/mtk-vpu
