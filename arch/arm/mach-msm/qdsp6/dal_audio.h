/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define AUDIO_DAL_DEVICE 0x02000028
#define AUDIO_DAL_PORT "DSP_DAL_AQ_AUD"

enum cad_rpc_process_type {
	CAD_RPC_ARM9  = 0,
	CAD_RPC_ARM11 = 1,
	CAD_RPC_PROCESSPR_MAX
};

enum {
	AUDIO_OP_OPEN = DAL_OP_FIRST_DEVICE_API,
	AUDIO_OP_WRITE,
	AUDIO_OP_READ,
	AUDIO_OP_IOCTL,
	AUDIO_OP_INIT,
	AUDIO_OP_CLOSE,
	AUDIO_OP_FLUSH_BUF,
};

#define ADSP_AUDIO_FORMAT_PCM		0x0103d2fd
#define ADSP_AUDIO_FORMAT_DTMF		0x01087725
#define ADSP_AUDIO_FORMAT_ADPCM		0x0103d2ff
#define ADSP_AUDIO_FORMAT_MP3		0x0103d308 /* ISO/IEC 11172 */
#define ADSP_AUDIO_FORMAT_MPEG4_AAC	0x010422f1 /* ISO/IEC 14496 */
#define ADSP_AUDIO_FORMAT_AMRNB_FS	0x0105c16c /* AMR-NB in FS format */
#define ADSP_AUDIO_FORMAT_V13K_FS	0x01080b8a /* QCELP 13k, IS733 */
#define ADSP_AUDIO_FORMAT_EVRC_FS	0x01080b89 /* EVRC   8k, IS127 */
#define ADSP_AUDIO_FORMAT_MIDI		0x0103d300 /* MIDI command stream */

/* For all of the audio formats, unless specified otherwise, */
/* the following apply: */
/* Format block bits are arranged in bytes and words in little-endian */
/* order, i.e., least-significant bit first and least-significant */
/* byte first. */

/* AAC Format Block. */

/* AAC format block consist of a format identifier followed by */
/* AudioSpecificConfig formatted according to ISO/IEC 14496-3 */

/* The following AAC format identifiers are supported */
#define ADSP_AUDIO_AAC_ADTS		0x010619cf
#define ADSP_AUDIO_AAC_MPEG4_ADTS	0x010619d0
#define ADSP_AUDIO_AAC_LOAS		0x010619d1
#define ADSP_AUDIO_AAC_ADIF		0x010619d2
#define ADSP_AUDIO_AAC_RAW		0x010619d3
#define ADSP_AUDIO_AAC_FRAMED_RAW	0x0108c1fb

struct adsp_audio_format_raw_pcm {
	uint16_t channels;
	uint16_t bits_per_sample;
	uint32_t sampling_rate;
	uint8_t is_signed;
	uint8_t is_interleaved;
} __attribute__((packed));


struct adsp_audio_format_adpcm {
	struct adsp_audio_format_raw_pcm base;
	uint32_t block_size;
} __attribute__((packed));


#define ADSP_AUDIO_COMPANDING_ALAW	0x10619cd
#define ADSP_AUDIO_COMPANDING_MLAW	0x10619ce

struct adsp_audio_format_g711 {
	uint32_t companding;
} __attribute__((packed));

/* ---------------------------------------------- */

/* Device direction Rx/Tx flag */
#define ADSP_AUDIO_RX_DEVICE		0x00
#define ADSP_AUDIO_TX_DEVICE		0x01

/* Predefined Audio device ids. */

/* Default RX or TX device */
#define ADSP_AUDIO_DEVICE_ID_DEFAULT		0x1081679

/* Source (TX) devices */
#define ADSP_AUDIO_DEVICE_ID_HANDSET_MIC	0x107ac8d
#define ADSP_AUDIO_DEVICE_ID_HEADSET_MIC	0x1081510
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC	0x1081512
#define ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC		0x1081518
#define ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC	0x108151b
#define ADSP_AUDIO_DEVICE_ID_I2S_MIC		0x1089bf3

/* Special loopback pseudo device to be paired with an RX device */
/* with usage ADSP_AUDIO_DEVICE_USAGE_MIXED_PCM_LOOPBACK */
#define ADSP_AUDIO_DEVICE_ID_MIXED_PCM_LOOPBACK_TX	0x1089bf2

/* Sink (RX) devices */
#define ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR			0x107ac88
#define ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_MONO			0x1081511
#define ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO		0x107ac8a
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO			0x1081513
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_MONO_HEADSET     0x108c508
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_STEREO_HEADSET   0x108c894
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO			0x1081514
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_MONO_HEADSET   0x108c895
#define ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_STEREO_HEADSET	0x108c509
#define ADSP_AUDIO_DEVICE_ID_BT_SCO_SPKR			0x1081519
#define ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_SPKR			0x108151c
#define ADSP_AUDIO_DEVICE_ID_I2S_SPKR				0x1089bf4

/* BT A2DP playback device. */
/* This device must be paired with */
/* ADSP_AUDIO_DEVICE_ID_MIXED_PCM_LOOPBACK_TX using  */
/* ADSP_AUDIO_DEVICE_USAGE_MIXED_PCM_LOOPBACK mode */
#define ADSP_AUDIO_DEVICE_ID_BT_A2DP_SPKR	0x108151a

/*  Audio device usage types. */
/*  This is a bit mask to determine which topology to use in the */
/* device session */
#define ADSP_AUDIO_DEVICE_CONTEXT_VOICE			0x01
#define ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK		0x02
#define ADSP_AUDIO_DEVICE_CONTEXT_RECORD		0x20
#define ADSP_AUDIO_DEVICE_CONTEXT_PCM_LOOPBACK		0x40

/* ---------------------------------------------- */

/* Opcode to open a device stream session to capture audio */
#define ADSP_AUDIO_OPEN_OP_READ		0x01

/* Opcode to open a device stream session to render audio */
#define ADSP_AUDIO_OPEN_OP_WRITE	0x02


/* Opcode to open a control session for device operations. Operations */
/* oncontrol session will affect all stream sessions assoiciated with */
/* the device. */

/* Use this operation to set/update common postproc settings, */
/* set/update calibration params for a device, enable/disable common*/
/* postprocs */

#define ADSP_AUDIO_OPEN_OP_DEVICE_CTRL	0x04

/* Some encoders need configuration information in addition to format */
/* block */

/* AAC Encoder modes */
#define ADSP_AUDIO_ENC_AAC_LC_ONLY_MODE		0
#define ADSP_AUDIO_ENC_AAC_PLUS_MODE		1
#define ADSP_AUDIO_ENC_ENHANCED_AAC_PLUS_MODE	2

/* AAC Encoder configuration */

struct adsp_audio_aac_enc_cfg {
	uint32_t bit_rate;	/* bits per second */
	uint32_t encoder_mode;	/* ADSP_AUDIO_ENC_* */
} __attribute__((packed));


/* AMR NB encoder modes */
#define ADSP_AUDIO_AMR_MR475	0
#define ADSP_AUDIO_AMR_MR515	1
#define ADSP_AUDIO_AMR_MMR59	2
#define ADSP_AUDIO_AMR_MMR67	3
#define ADSP_AUDIO_AMR_MMR74	4
#define ADSP_AUDIO_AMR_MMR795	5
#define ADSP_AUDIO_AMR_MMR102	6
#define ADSP_AUDIO_AMR_MMR122	7


/* The following are valid AMR NB DTX modes */
#define ADSP_AUDIO_AMR_DTX_MODE_OFF		0
#define ADSP_AUDIO_AMR_DTX_MODE_ON_VAD1		1
#define ADSP_AUDIO_AMR_DTX_MODE_ON_VAD2		2
#define ADSP_AUDIO_AMR_DTX_MODE_ON_AUTO		3


struct adsp_audio_amr_enc_cfg {
	uint32_t mode;		/* ADSP_AUDIO_AMR_MR* */
	uint32_t dtx_mode;	/* ADSP_AUDIO_AMR_DTX_MODE* */
	uint32_t enable;		/* 1 = enable, 0 = disable */
} __attribute__((packed));

struct adsp_audio_qcelp13k_enc_cfg {
	uint16_t min_rate;
	uint16_t max_rate;
} __attribute__((packed));

struct adsp_audio_evrc_enc_cfg {
	uint16_t min_rate;
	uint16_t max_rate;
} __attribute__((packed));

struct adsp_audio_codec_config {
	union {
		struct adsp_audio_amr_enc_cfg amr_cfg;
		struct adsp_audio_aac_enc_cfg aac_cfg;
		struct adsp_audio_qcelp13k_enc_cfg qcelp13k_cfg;
		struct adsp_audio_evrc_enc_cfg evrc_cfg;
	};
} __attribute__((packed));


/* Bit masks for adsp_audio_open_stream_device.mode */

/* This is the default value. */
#define  ADSP_AUDIO_OPEN_STREAM_MODE_NONE       0x0000

/* This bit, if set, indicates that the AVSync mode is activated. */
#define  ADSP_AUDIO_OPEN_STREAM_MODE_AVSYNC     0x0001


/* Data for ADSPaudio_DriverOpen operation.  */

/* The client specifies the media format block as defined in  */
/* ADSPaudio_MediaFormat.h for OPEN_OP_READ/WRITE */

#define ADSP_AUDIO_MAX_DEVICES 4

struct adsp_audio_open_stream_device {
	uint32_t num_devices;
	/* List of stream device IDs (ADSP_AUDIO_DEVICE_ID_*,...) */
	uint32_t device[ADSP_AUDIO_MAX_DEVICES];
	uint32_t stream_context;	/* ADSP_AUDIO_DEVICE_CONTEXT_* */
	uint32_t format;		/* ADSP_AUDIO_FORMAT_* */
	void *format_block;		/* phys addr */
	uint32_t format_block_len;	/* bytes */
	uint32_t buf_max_size;		/* bytes */
	uint32_t priority;		/* unused */
	struct adsp_audio_codec_config config; /* for READ ops */
	uint32_t mode;			/* ADSP_AUDIO_OPEN_STREAM_MODE_* */
} __attribute__((packed));


/* adsp_audio_OpenStreamDevice is not specified for OPEN_OP_DEVICE_CTRL */

struct adsp_audio_open_device {
	uint32_t size;		/* sizeof(this) - sizeof(uint32_t) */
	void *context;
	void *data;

	uint32_t op_code;	/* ADSP_AUDIO_OPEN_OP_* */

	/* only for OP_READ / OP_WRITE */
	struct adsp_audio_open_stream_device stream_device;
} __attribute__((packed));


/* Data buffer definition for read/write operations */



/* This flag, if set, indicates that the beginning of the data in the*/
/* buffer is a synchronization point or key frame, meaning no data */
/* before it in the stream is required in order to render the stream */
/* from this point onward. */
#define ADSP_AUDIO_BUFFER_FLAG_SYNC_POINT        0x01

/* This flag, if set, indicates that the buffer object is using valid */
/* physical address used to store the media data */
#define ADSP_AUDIO_BUFFER_FLAG_PHYS_ADDR         0x04

/* This flag, if set, indicates that a media start timestamp has been */
/* set for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_START_SET         0x08

/* This flag, if set, indicates that a media stop timestamp has been set */
/* for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_STOP_SET          0x10

/* This flag, if set, indicates that a preroll timestamp has been set */
/* for a buffer. */
#define ADSP_AUDIO_BUFFER_FLAG_PREROLL_SET       0x20

/* This flag, if set, indicates that the data in the buffer is a fragment of */
/* a larger block of data, and will be continued by the data in the next */
/* buffer to be delivered. */
#define ADSP_AUDIO_BUFFER_FLAG_CONTINUATION      0x40

struct adsp_audio_data_buffer {
	uint32_t buffer_addr;	/* Physical Address of buffer */
	uint32_t max_size;	/* Maximum size of buffer */
	uint32_t actual_size;	/* Actual size of valid data in the buffer */
	uint32_t offset;	/* Offset to the first valid byte */
	uint32_t flags;		/* ADSP_AUDIO_BUFFER_FLAGs that has been set */
	int64_t start;		/* Start timestamp, if any */
	int64_t stop;		/* Stop timestamp, if any */
	int64_t preroll;	/* Preroll timestamp, if any */
} __attribute__((packed));


struct adsp_audio_buffer {
	uint32_t size;		/* sizeof(this) - sizeof(uint32_t) */
	uint32_t context;
	uint32_t data;

	struct adsp_audio_data_buffer	buffer;	/* media data buffer */
} __attribute__((packed));

/* control */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_CONFIG_TABLE	0x0108b6bf

struct adsp_ioctl_device_config_table {
	uint32_t ioctl_id;
	uint32_t size;
	uint32_t context;
	uint32_t data;
	uint32_t device_id;
	uint32_t phys_addr;
	uint32_t phys_size;
	uint32_t phys_used;
} __attribute__((packed));


/* stream control ioctls - no payload */

#define ADSP_AUDIO_IOCTL_CMD_STREAM_START		0x010815c6
#define ADSP_AUDIO_IOCTL_CMD_STREAM_STOP		0x01075c54
#define ADSP_AUDIO_IOCTL_CMD_STREAM_PAUSE		0x01075ee8
#define ADSP_AUDIO_IOCTL_CMD_STREAM_RESUME		0x01075ee9
#define ADSP_AUDIO_IOCTL_CMD_STREAM_FLUSH		0x01075eea
#define ADSP_AUDIO_IOCTL_CMD_STREAM_EOS			0x0108b150

/* defined in msm8k_cad_ioctl.h but apply to dsp? ... */
#define QDSP_IOCTL_CMD_STREAM_DTMF_STOP			0x01087554
#define QDSP_IOCTL_CMD_SET_DEVICE_VOL			0x01087a4a
#define QDSP_IOCTL_CMD_SET_DEVICE_MUTE			0x01087a4b
#define QDSP_IOCTL_CMD_SET_STREAM_MUTE			0x01087b0c
#define QDSP_IOCTL_CMD_SET_STREAM_VOL			0x01087b0b

/* events */


/* This event is generated after a media stream session is opened. */
#define ADSP_AUDIO_EVT_STATUS_OPEN			0x0108c0d6

/* This event is generated after a media stream  session is closed. */
#define ADSP_AUDIO_EVT_STATUS_CLOSE			0x0108c0d7

/* Asyncronous buffer consumption. This event is generated after a */
/* recived  buffer is consumed during rendering or filled during */
/* capture opeartion. */
#define ADSP_AUDIO_EVT_STATUS_BUF_DONE			0x0108c0d8

/* This event is generated when rendering operation is starving for */
/* data. In order to avoid audio loss at the end of a plauback, the */
/* client should wait for this event before issuing the close command. */
#define ADSP_AUDIO_EVT_STATUS_BUF_UNDERRUN		0x0108c0d9

/* This event is generated during capture operation when there are no */
/* buffers available to copy the captured audio data */
#define ADSP_AUDIO_EVT_STATUS_BUF_OVERFLOW		0x0108c0da

struct adsp_audio_event {
	/* DAL common header */
	uint32_t evt_handle;
	uint32_t evt_cookie;
	uint32_t evt_length;

	/* AUDIO common header */
	uint32_t context;
	uint32_t data;
	uint32_t session_id;
	uint32_t event_id;
	int32_t status;
	uint32_t data_len;

	/* AUDIO payload */
	union {
		int32_t ivalue;
		uint32_t uvalue;
		struct adsp_audio_data_buffer buffer;
		int64_t time;
	} u;
} __attribute__((packed));
	

/* control and stream session ioctls */

/* All device switch protocol IOCTLs have payload struct */
/* adsp_audio_device_change A device switch requires three IOCTL */
/* commands in the following sequence: */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY */
/*    ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT */


/* Device Class */
/*   DMA 0 - Int RX */
/*   DMA 1 - Int TX */
/*   DMA 2 - Ext RX */
/*   DMA 3 - Ext TX */

struct adsp_audio_device_change {
	uint32_t ioctl_id;
	uint32_t size; /* sizeof(this) - 2 * sizeof(uint32-t) */
	uint32_t context;
	uint32_t data;

	u32 old_device; /* DeviceID to switch from */
	u32 new_device; /* DeviceID to switch to */
	u8 device_class; /* TBD */
	u8 device_type; /* 0 == Rx, 1 == Tx and 2 == both */
} __attribute__((packed));



/* Device switch protocol step #1. Pause old device and */
/* generate silence for the old device. */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_PREPARE	0x010815c4


/* Device switch protocol step #2. Release old device, */
/* create new device and generate silence for the new device. */

/* When client receives ack for this IOCTL, the client can */
/* start sending IOCTL commands to configure, calibrate and */
/* change filter settings on the new device. */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_STANDBY	0x010815c5


/* Device switch protocol step #3. Start normal operations on new device */

#define ADSP_AUDIO_IOCTL_CMD_DEVICE_SWITCH_COMMIT	0x01075ee7

/* stream ioctls */

#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL		0x0107605c
#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE		0x0107605f
#define ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_EQ_CONFIG	0x0108b10e

#define ADSP_PATH_RX	0
#define ADSP_PATH_TX	1
#define ADSP_PATH_BOTH	2

struct adsp_audio_set_device_volume {
	uint32_t ioctl_id;
	uint32_t size; /* sizeof(this) - 2 *sizeof(uint32_t) */
	uint32_t context;
	uint32_t data;

	uint32_t device_id;
	uint32_t path;
	int32_t volume;
} __attribute__((packed));

struct adsp_audio_set_device_mute {
	uint32_t ioctl_id;
	uint32_t size; /* sizeof(this) - 2 *sizeof(uint32_t) */
	uint32_t context;
	uint32_t data;

	uint32_t device_id;
	uint32_t path;
	uint32_t mute; /* 0 = not muted, 1 = muted */
} __attribute__((packed));

