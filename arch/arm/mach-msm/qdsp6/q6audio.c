/* arch/arm/mach-msm/qdsp6/q6audio.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include "dal.h"
#include "dal_audio.h"
#include "dal_acdb.h"
#include "dal_adie.h"
#include <mach/msm_qdsp6_audio.h>

#include <linux/gpio.h>

static struct clk *icodec_rx_clk;
static struct clk *icodec_tx_clk;
static struct clk *ecodec_clk;
static struct clk *sdac_clk;

static struct q6audio_analog_ops *analog_ops;
static uint32_t tx_clk_freq = 8000;

void q6audio_register_analog_ops(struct q6audio_analog_ops *ops)
{
	analog_ops = ops;
}

uint32_t get_device_group(uint32_t device_id)
{

	switch(device_id) {
	case ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR:
	case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_MONO:
	case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_MONO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_STEREO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_MONO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_STEREO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_SPKR:
		return 0;
	case ADSP_AUDIO_DEVICE_ID_HANDSET_MIC:
	case ADSP_AUDIO_DEVICE_ID_HEADSET_MIC:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC:
	case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC:
		return 1;
	case ADSP_AUDIO_DEVICE_ID_BT_SCO_SPKR:
	case ADSP_AUDIO_DEVICE_ID_BT_A2DP_SPKR:
		return 2;
	case ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC:
		return 3;
	case ADSP_AUDIO_DEVICE_ID_I2S_SPKR:
		return 6;
	case ADSP_AUDIO_DEVICE_ID_I2S_MIC:
		return 7;
	default:
		pr_err("invalid device id %d\n", device_id);
		return -1;
	}
}

uint32_t get_path_id(uint32_t device_id)
{
	switch(device_id) {
	case ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR:
		return ADIE_PATH_HANDSET_RX;
	case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_MONO:
		return ADIE_PATH_HEADSET_MONO_RX;
	case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO:
		return ADIE_PATH_HEADSET_STEREO_RX;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO:
		return ADIE_PATH_SPEAKER_RX;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_MONO_HEADSET:
		return ADIE_PATH_SPKR_MONO_HDPH_MONO_RX;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_STEREO_HEADSET:
		return ADIE_PATH_SPKR_MONO_HDPH_STEREO_RX;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO:
		return ADIE_PATH_SPEAKER_STEREO_RX;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_MONO_HEADSET:
		return ADIE_PATH_SPKR_STEREO_HDPH_MONO_RX;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_STEREO_HEADSET:
		return ADIE_PATH_SPKR_STEREO_HDPH_STEREO_RX;
	case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_SPKR:
		return ADIE_PATH_TTY_HEADSET_RX;
	case ADSP_AUDIO_DEVICE_ID_HANDSET_MIC:
		return ADIE_PATH_HANDSET_TX;
	case ADSP_AUDIO_DEVICE_ID_HEADSET_MIC:
		return ADIE_PATH_HEADSET_MONO_TX;
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC:
		return ADIE_PATH_SPEAKER_TX;
	case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC:
		return ADIE_PATH_TTY_HEADSET_TX;
	case ADSP_AUDIO_DEVICE_ID_BT_SCO_SPKR:
	case ADSP_AUDIO_DEVICE_ID_BT_A2DP_SPKR:
	case ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC:
	default:
		pr_err("ADIE paths not supported for device id %d\n",
			device_id);
		return 0;
	}
}

static inline int adie_open(struct dal_client *client) 
{
	return dal_call_f0(client, DAL_OP_OPEN, 0);
}

static inline int adie_set_path(struct dal_client *client,
				uint32_t id, uint32_t path_type)
{
	return dal_call_f1(client, ADIE_OP_SET_PATH, id, path_type);
}

static inline int adie_set_path_freq_plan(struct dal_client *client,
					  uint32_t path_type, uint32_t plan) 
{
	return dal_call_f1(client, ADIE_OP_SET_PATH_FREQUENCY_PLAN,
			   path_type, plan);
}

static inline int adie_proceed_to_stage(struct dal_client *client,
					uint32_t path_type, uint32_t stage)
{
	return dal_call_f1(client, ADIE_OP_PROCEED_TO_STAGE,
			   path_type, stage);
}

static inline int adie_mute_path(struct dal_client *client,
				 uint32_t path_type, uint32_t mute_state)
{
	return dal_call_f1(client, ADIE_OP_MUTE_PATH, path_type, mute_state);
}


static struct dal_client *adie;
static struct dal_client *audio_evt;
static struct dal_client *audio_ctl;

/* 4k DMA scratch page used for exchanging acdb device config tables
 * and stream format descriptions with the DSP.
 */
static void *audio_data;
static dma_addr_t audio_phys;

struct cad_msg_init {
	uint32_t size;
	void *callback_event;
	void *local_trigger_event;
	uint32_t session_id;
	uint32_t processor_id;
};


/* HACK: The DSP fails to send events and callbacks to the proper
 * target handle (the entity that initiated the event), so we have
 * to do this nonsense with session handles to route things until
 * the DSP code is fixed.
 */
#define SESSION_MIN 2
#define SESSION_MAX 9

static DEFINE_MUTEX(session_lock);
static DEFINE_MUTEX(audio_lock);

static struct audio_client *session[SESSION_MAX];

static int session_alloc(struct audio_client *ac)
{
	int n;

	mutex_lock(&session_lock);
	for (n = SESSION_MIN; n < SESSION_MAX; n++) {
		if (!session[n]) {
			session[n] = ac;
			mutex_unlock(&session_lock);
			return n;
		}
	}
	mutex_unlock(&session_lock);
	return -ENOMEM;
}

static void session_free(int n, struct audio_client *ac)
{
	mutex_lock(&session_lock);
	if (session[n] == ac)
		session[n] = 0;
	mutex_unlock(&session_lock);
}

/* TODO: might as well leave the ACDB session attached always */
static int acdb_get_config_table(uint32_t device_id, uint32_t sample_rate)
{
	struct dal_client *client;
	struct acdb_cmd_device_table rpc;
	struct acdb_result res;
	int r;

	client = dal_attach(ACDB_DAL_DEVICE, ACDB_DAL_PORT, 0, 0);
	if (!client)
		return -EIO;

	memset(audio_data, 0, 4096);

	memset(&rpc, 0, sizeof(rpc));

	rpc.size = sizeof(rpc) - (2 * sizeof(uint32_t));
	rpc.command_id = ACDB_GET_DEVICE_TABLE;
	rpc.device_id = device_id;
	rpc.sample_rate_id = sample_rate;
	rpc.total_bytes = 4096;
	rpc.unmapped_buf = audio_phys;
	rpc.res_size = sizeof(res) - (2 * sizeof(uint32_t));

	r = dal_call(client, ACDB_OP_IOCTL, 8, &rpc, sizeof(rpc),
		     &res, sizeof(res));

	dal_detach(client);

	if ((r == sizeof(res)) && (res.dal_status == 0)) {
		return res.used_bytes;
	}
	return -EIO;
}

static int audio_init(struct dal_client *client, int sessionid)
{
	struct cad_msg_init rpc;
	uint32_t res;
	int r;
	rpc.size = sizeof(rpc) - sizeof(uint32_t);
	rpc.callback_event = client;
	rpc.local_trigger_event = client;
	rpc.session_id = sessionid;
	rpc.processor_id = CAD_RPC_ARM11;
	
	r = dal_call(client, AUDIO_OP_INIT, 5, &rpc, sizeof(rpc),
		     &res, sizeof(res));

	return 0;
}

static int audio_open_control(struct dal_client *client)
{
	struct adsp_audio_open_device rpc;
	uint32_t res;
	int r;

	memset(&rpc, 0, sizeof(rpc));
	rpc.size = sizeof(rpc) - sizeof(uint32_t);
	rpc.context = 0;
	rpc.data = 0;
	rpc.op_code = ADSP_AUDIO_OPEN_OP_DEVICE_CTRL;

	r = dal_call(client, AUDIO_OP_OPEN, 5, &rpc, sizeof(rpc),
		     &res, sizeof(res));
	return 0;
}

static int audio_out_open(struct dal_client *client, uint32_t bufsz,
			  uint32_t rate, uint32_t channels)
{
	struct adsp_audio_format_raw_pcm *fmt = audio_data;
	struct adsp_audio_open_device rpc;
	uint32_t res;
	int r;

	fmt->channels = channels;
	fmt->bits_per_sample = 16;
	fmt->sampling_rate = rate;
	fmt->is_signed = 1;
	fmt->is_interleaved = 1;

	memset(&rpc, 0, sizeof(rpc));
	rpc.size = sizeof(rpc) - sizeof(uint32_t);
	rpc.context = 0;
	rpc.data = 0;
	rpc.op_code = ADSP_AUDIO_OPEN_OP_WRITE;
	rpc.stream_device.num_devices = 1;
	rpc.stream_device.device[0] = ADSP_AUDIO_DEVICE_ID_DEFAULT;
	rpc.stream_device.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_PLAYBACK;
	rpc.stream_device.format = ADSP_AUDIO_FORMAT_PCM;
	rpc.stream_device.format_block = (void*) audio_phys;
	rpc.stream_device.format_block_len = sizeof(*fmt);
	rpc.stream_device.buf_max_size = bufsz; /* XXX ??? */

	r = dal_call(client, AUDIO_OP_OPEN, 5, &rpc, sizeof(rpc),
		     &res, sizeof(res));
	return 0;
}

static int audio_in_open(struct dal_client *client, uint32_t bufsz,
			 uint32_t rate, uint32_t channels)
{
	struct adsp_audio_format_raw_pcm *fmt = audio_data;
	struct adsp_audio_open_device rpc;
	uint32_t res;
	int r;

	fmt->channels = channels;
	fmt->bits_per_sample = 16;
	fmt->sampling_rate = rate;
	fmt->is_signed = 1;
	fmt->is_interleaved = 1;

	memset(&rpc, 0, sizeof(rpc));
	rpc.size = sizeof(rpc) - sizeof(uint32_t);
	rpc.context = 0;
	rpc.data = 0;
	rpc.op_code = ADSP_AUDIO_OPEN_OP_READ;
	rpc.stream_device.num_devices = 1;
	rpc.stream_device.device[0] = ADSP_AUDIO_DEVICE_ID_DEFAULT;
	rpc.stream_device.stream_context = ADSP_AUDIO_DEVICE_CONTEXT_RECORD;
	rpc.stream_device.format = ADSP_AUDIO_FORMAT_PCM;
	rpc.stream_device.format_block = (void*) audio_phys;
	rpc.stream_device.format_block_len = sizeof(*fmt);
	rpc.stream_device.buf_max_size = bufsz; /* XXX ??? */

	r = dal_call(client, AUDIO_OP_OPEN, 5, &rpc, sizeof(rpc),
		     &res, sizeof(res));
	return 0;
}

static int audio_close(struct dal_client *client, int session_id)
{
	dal_call_f0(client, AUDIO_OP_CLOSE, session_id);
	return 0;
}

static int audio_set_table(struct dal_client *client,
			uint32_t device_id, int size)
{
	struct adsp_ioctl_device_config_table rpc;
	uint32_t res;
	int r;

	rpc.ioctl_id = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_CONFIG_TABLE;
	rpc.size = sizeof(rpc) - (2 * sizeof(uint32_t));
	rpc.context = 0;
	rpc.data = 0;
	rpc.device_id = device_id;
	rpc.phys_addr = audio_phys;
	rpc.phys_size = size;
	rpc.phys_used = size;

	r = dal_call(client, AUDIO_OP_IOCTL, 6, &rpc, sizeof(rpc),
		     &res, sizeof(res));
	return 0;
}

int q6audio_read(struct audio_client *ac, struct audio_buffer *ab)
{
	struct adsp_audio_buffer rpc;
	uint32_t res;
	int r;

	memset(&rpc, 0, sizeof(rpc));
	rpc.size = sizeof(rpc) - sizeof(uint32_t);
	rpc.context = 0;
	rpc.data = 0;
	rpc.buffer.buffer_addr = ab->phys;
	rpc.buffer.max_size = ab->size;
	rpc.buffer.actual_size = ab->size;

	r = dal_call(ac->client, AUDIO_OP_READ, 5, &rpc, sizeof(rpc),
		     &res, sizeof(res));
	return 0;
}

int q6audio_write(struct audio_client *ac, struct audio_buffer *ab)
{
	struct adsp_audio_buffer rpc;
	uint32_t res;
	int r;

	memset(&rpc, 0, sizeof(rpc));
	rpc.size = sizeof(rpc) - sizeof(uint32_t);
	rpc.context = 0;
	rpc.data = 0;
	rpc.buffer.buffer_addr = ab->phys;
	rpc.buffer.max_size = ab->size;
	rpc.buffer.actual_size = ab->used;

	r = dal_call(ac->client, AUDIO_OP_WRITE, 5, &rpc, sizeof(rpc),
		     &res, sizeof(res));
	return 0;
}

static int audio_ioctl(struct dal_client *client, void *ptr, uint32_t len)
{
	uint32_t tmp;
	int r;
	r = dal_call(client, AUDIO_OP_IOCTL, 6, ptr, len, &tmp, sizeof(tmp));
	if (r != 4)
		return -EIO;
	return tmp;
}

static int audio_command(struct dal_client *client, uint32_t cmd)
{
	struct {
		uint32_t ioctl;
		uint32_t size;
	} rpc;
	rpc.ioctl = cmd;
	rpc.size = 0;
	return audio_ioctl(client, &rpc, sizeof(rpc));
}

static int audio_rx_volume(struct dal_client *client, uint32_t dev_id, int32_t volume)
{
	struct adsp_audio_set_device_volume rpc;
	rpc.ioctl_id = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL;
	rpc.size = sizeof(rpc) - (2 * sizeof(uint32_t));
	rpc.context = 0;
	rpc.data = 0;
	rpc.device_id = dev_id;
	rpc.path = ADSP_PATH_RX;
	rpc.volume = volume;
	return audio_ioctl(client, &rpc, sizeof(rpc));
}

static int audio_rx_mute(struct dal_client *client, uint32_t dev_id, int mute)
{
	struct adsp_audio_set_device_mute rpc;
	rpc.ioctl_id = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE;
	rpc.size = sizeof(rpc) - (2 * sizeof(uint32_t));
	rpc.context = 0;
	rpc.data = 0;
	rpc.device_id = dev_id;
	rpc.path = ADSP_PATH_RX;
	rpc.mute = !!mute;
	return audio_ioctl(client, &rpc, sizeof(rpc));
}

static int audio_tx_volume(struct dal_client *client, uint32_t dev_id, int32_t volume)
{
	struct adsp_audio_set_device_volume rpc;
	rpc.ioctl_id = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_VOL;
	rpc.size = sizeof(rpc) - (2 * sizeof(uint32_t));
	rpc.context = 0;
	rpc.data = 0;
	rpc.device_id = dev_id;
	rpc.path = ADSP_PATH_TX;
	rpc.volume = volume;
	return audio_ioctl(client, &rpc, sizeof(rpc));
}

static int audio_tx_mute(struct dal_client *client, uint32_t dev_id, int mute)
{
	struct adsp_audio_set_device_mute rpc;
	rpc.ioctl_id = ADSP_AUDIO_IOCTL_CMD_SET_DEVICE_MUTE;
	rpc.size = sizeof(rpc) - (2 * sizeof(uint32_t));
	rpc.context = 0;
	rpc.data = 0;
	rpc.device_id = dev_id;
	rpc.path = ADSP_PATH_TX;
	rpc.mute = !!mute;
	return audio_ioctl(client, &rpc, sizeof(rpc));
}

static void callback(void *data, int len, void *cookie)
{
	struct adsp_audio_event *e = data;
	struct audio_client *ac;

	if (e->session_id >= SESSION_MAX) {
		pr_err("audio callback: bogus session %d\n",
		       e->session_id);
		return;
	}
	ac = session[e->session_id];
	if (!ac) {
		pr_err("audio callback: unknown session %d\n",
		       e->session_id);
		return;
	}

	if (e->event_id == ADSP_AUDIO_EVT_STATUS_BUF_DONE) {
		ac->buf[ac->dsp_buf].used = 0;
		ac->dsp_buf ^= 1;
		wake_up(&ac->wait);
		return;
	}

	pr_info("CB h=%08x c=%08x id=%d e=%08x s=%d\n",
		e->evt_handle, e->evt_cookie, e->session_id,
		e->event_id, e->status);

	if (ac->cb_status == -EBUSY) {
		ac->cb_status = e->status;
		wake_up(&ac->wait);
	}
#if 0
	pr_info("callback %p %d %p\n", data, len, cookie);

	pr_info("  handle %p, cookie %p, length %d\n",
		e->evt_handle, e->evt_cookie, e->evt_length);
	pr_info("  context %p, data %p, session %d\n",
		e->context, e->data, e->session_id);
	pr_info("  event %08x, status %d, len %d, %s\n",
		e->event_id, e->status, e->data_len,
		audio_event(e->event_id));

	if (e->event_id == ADSP_AUDIO_EVT_STATUS_BUF_DONE) {
		pr_info("buffer @ %x max=%d sz=%d off=%d flags=%d\n",
			e->u.buffer.buffer_addr,
			e->u.buffer.max_size,
			e->u.buffer.actual_size,
			e->u.buffer.offset,
			e->u.buffer.flags);
	}
#endif
}

static struct audio_client *ac_control;

static int q6audio_init(void)
{
	struct audio_client *ac = 0;
	int res;

	mutex_lock(&audio_lock);
	if (ac_control) {
		res = 0;
		goto done;
	}

	icodec_rx_clk = clk_get(0, "icodec_rx_clk");
	icodec_tx_clk = clk_get(0, "icodec_tx_clk");
	ecodec_clk = clk_get(0, "ecodec_clk");
	sdac_clk = clk_get(0, "sdac_clk");

	audio_data = dma_alloc_coherent(NULL, 4096, &audio_phys, GFP_KERNEL);
	printk("q6audio_init() dma @ %p (%x)\n", audio_data, audio_phys);

	ac = kzalloc(sizeof(*ac), GFP_KERNEL);
	if (!ac) {
		res = -ENOMEM;
		goto done;
	}
	init_waitqueue_head(&ac->wait);
	session[1] = ac;

	pr_info("audio_init()\n");

	audio_evt = dal_attach(AUDIO_DAL_DEVICE, AUDIO_DAL_PORT,
			       callback, 0);
	if (!audio_evt) {
		pr_err("audio_init: cannot attach to audio event channel\n");
		res = -ENODEV;
		goto done;
	}
	/* event/callback channel is always session 0 */
	audio_init(audio_evt, 0);

	audio_ctl = dal_attach(AUDIO_DAL_DEVICE, AUDIO_DAL_PORT,
			       callback, 0);
	if (!audio_ctl){
		pr_err("audio_init: cannot attach to audio control channel\n");
		res = -ENODEV;
		goto done;
	}
	/* control channel is always session 1 */
	audio_init(audio_ctl, 1);

	ac->cb_status = -EBUSY;
	audio_open_control(audio_ctl);
	wait_event(ac->wait, (ac->cb_status != -EBUSY));

	adie = dal_attach(ADIE_DAL_DEVICE, ADIE_DAL_PORT, 0, 0);
	if (!adie) {
		pr_err("audio_init: cannot attach to adie\n");
		res = -ENODEV;
		goto done;
	}
	res = adie_open(adie);
	if (res) {
		pr_err("audio_init: adie open failed\n");
		res = -ENODEV;
		goto done;
	}

	if (analog_ops->init)
		analog_ops->init();

	pr_info("audio_init: OKAY\n");
	res = 0;
	ac->client = audio_ctl;
	ac_control = ac;

done:
	if ((res < 0) && ac) {
		session[1] = 0;
		kfree(ac);
	}
	mutex_unlock(&audio_lock);

	return res;
}

static void audio_client_free(struct audio_client *ac)
{
	session_free(ac->session, ac);

	if (ac->buf[0].data)
		dma_free_coherent(NULL, ac->buf[0].size,
				  ac->buf[0].data, ac->buf[0].phys);
	if (ac->buf[1].data)
		dma_free_coherent(NULL, ac->buf[1].size,
				  ac->buf[1].data, ac->buf[1].phys);
	kfree(ac);
}

static struct audio_client *audio_client_alloc(unsigned bufsz)
{
	struct audio_client *ac;

	ac = kzalloc(sizeof(*ac), GFP_KERNEL);
	if (!ac)
		return 0;

	ac->buf[0].data = dma_alloc_coherent(NULL, bufsz,
					     &ac->buf[0].phys, GFP_KERNEL);
	if (!ac->buf[0].data)
		goto fail;
	ac->buf[1].data = dma_alloc_coherent(NULL, bufsz,
					     &ac->buf[1].phys, GFP_KERNEL);
	if (!ac->buf[1].data)
		goto fail;

	ac->buf[0].size = bufsz;
	ac->buf[1].size = bufsz;
	init_waitqueue_head(&ac->wait);

	return ac;

fail:
	audio_client_free(ac);
	return 0;
}


static uint32_t audio_rx_path_id = ADIE_PATH_SPEAKER_RX;
static uint32_t audio_rx_device_id = ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO;
static uint32_t audio_rx_device_group = -1;
static uint32_t audio_tx_path_id = ADIE_PATH_SPEAKER_TX;
static uint32_t audio_tx_device_id = ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC;
static uint32_t audio_tx_device_group = -1;

static void _audio_rx_path_enable(void)
{
	uint32_t adev;
	int sz;

	adev = ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR;
	/* always have to load HANDSET_SPKR or things blow up */

	pr_info("audiolib: load HANDSET cfg table\n");
	sz = acdb_get_config_table(adev, 48000);
	ac_control->cb_status = -EBUSY;
	audio_set_table(audio_ctl, adev, sz);
	wait_event(ac_control->wait, (ac_control->cb_status != -EBUSY));
	
	/* load the config for the actual device we're using */
	pr_info("audiolib: load %08x cfg table\n", audio_rx_device_id);
	sz = acdb_get_config_table(audio_rx_device_id, 48000);	
	ac_control->cb_status = -EBUSY;
	audio_set_table(audio_ctl, audio_rx_device_id, sz);
	wait_event(ac_control->wait, (ac_control->cb_status != -EBUSY));
	
	pr_info("audiolib: enable amps\n");
	if (analog_ops->speaker_enable)
		analog_ops->speaker_enable(1);
	if (analog_ops->headset_enable)
		analog_ops->headset_enable(1);

	pr_info("audiolib: set path\n");
	adie_set_path(adie, audio_rx_path_id, ADIE_PATH_RX);
	adie_set_path_freq_plan(adie, ADIE_PATH_RX, 48000);

	pr_info("audiolib: enable adie\n");
	adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_READY);
	adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_ANALOG_READY);

	ac_control->cb_status = -EBUSY;
	audio_rx_mute(audio_ctl, adev, 0);
	wait_event(ac_control->wait, (ac_control->cb_status != -EBUSY));

	ac_control->cb_status = -EBUSY;
	audio_rx_volume(audio_ctl, adev, 496);
	wait_event(ac_control->wait, (ac_control->cb_status != -EBUSY));
}

static void _audio_tx_path_enable(void)
{
	uint32_t adev;
	int sz;

	adev = audio_tx_device_id;

	sz = acdb_get_config_table(adev, 8000);
	ac_control->cb_status = -EBUSY;
	audio_set_table(audio_ctl, adev, sz);
	wait_event(ac_control->wait, (ac_control->cb_status != -EBUSY));

	pr_info("audiolib: enable mic\n");
	if (analog_ops->int_mic_enable)
		analog_ops->int_mic_enable(1);
	if (analog_ops->ext_mic_enable)
		analog_ops->ext_mic_enable(1);

	pr_info("audiolib: set tx path\n");
	adie_set_path(adie, audio_tx_path_id, ADIE_PATH_TX);
	adie_set_path_freq_plan(adie, ADIE_PATH_TX, 8000);

	pr_info("audiolib: enable tx adie\n");
	adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_READY);
	adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_ANALOG_READY);

	ac_control->cb_status = -EBUSY;
	audio_tx_mute(audio_ctl, adev, 0);
	wait_event(ac_control->wait, (ac_control->cb_status != -EBUSY));

	ac_control->cb_status = -EBUSY;
	audio_tx_volume(audio_ctl, adev, 496);
	wait_event(ac_control->wait, (ac_control->cb_status != -EBUSY));
}

static void _audio_rx_path_disable(void)
{
	pr_info("audiolib: disable adie\n");
	adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_ANALOG_OFF);
	adie_proceed_to_stage(adie, ADIE_PATH_RX, ADIE_STAGE_DIGITAL_OFF);

	pr_info("audiolib: disable amps\n");
	if (analog_ops->speaker_enable)
		analog_ops->speaker_enable(0);
	if (analog_ops->headset_enable)
		analog_ops->headset_enable(0);
}

static void _audio_tx_path_disable(void)
{
	pr_info("audiolib: disable adie\n");
	adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_ANALOG_OFF);
	adie_proceed_to_stage(adie, ADIE_PATH_TX, ADIE_STAGE_DIGITAL_OFF);

	pr_info("audiolib: disable mic\n");
	if (analog_ops->int_mic_enable)
		analog_ops->int_mic_enable(0);
	if (analog_ops->ext_mic_enable)
		analog_ops->ext_mic_enable(0);
}

static int icodec_rx_clk_refcount;
static int icodec_tx_clk_refcount;
static int ecodec_clk_refcount;
static int sdac_clk_refcount;

static void _audio_rx_clk_enable(void)
{
	uint32_t device_group;

	device_group = get_device_group(audio_rx_device_id);

	switch(device_group) {
	case 0:
		icodec_rx_clk_refcount++;
		if (icodec_rx_clk_refcount == 1) {
			clk_set_rate(icodec_rx_clk, 12288000);
			clk_enable(icodec_rx_clk);
			audio_rx_device_group = device_group;
		}
		break;
	case 2:
		ecodec_clk_refcount++;
		if (ecodec_clk_refcount == 1) {
			clk_set_rate(ecodec_clk, 2048000);
			clk_enable(ecodec_clk);
			audio_rx_device_group = device_group;
		}
		break;
	case 6:
		sdac_clk_refcount++;
		if (sdac_clk_refcount == 1) {
			clk_set_rate(sdac_clk, 12288000);
			clk_enable(sdac_clk);
			audio_rx_device_group = device_group;
		}
		break;
	default:
		pr_err("audiolib: invalid rx device group %d\n", device_group);
		break;
	}
}

static void _audio_tx_clk_enable(void)
{
	uint32_t device_group;

	device_group = get_device_group(audio_tx_device_id);
	switch (device_group) {
	case 1:
		icodec_tx_clk_refcount++;
		if (icodec_tx_clk_refcount == 1) {
			clk_set_rate(icodec_tx_clk, tx_clk_freq * 256);
			clk_enable(icodec_tx_clk);
			audio_tx_device_group = device_group;
		}
		break;
	case 3:
		ecodec_clk_refcount++;
		if (ecodec_clk_refcount == 1) {
			clk_set_rate(ecodec_clk, 2048000);
			clk_enable(ecodec_clk);
			audio_tx_device_group = device_group;
		}
		break;
	case 7:
		/* TODO: In QCT BSP, clk rate was set to 20480000 */
		sdac_clk_refcount++;
		if (sdac_clk_refcount == 1) {
			clk_set_rate(sdac_clk, 12288000);
			clk_enable(sdac_clk);
			audio_tx_device_group = device_group;
		}
		break;
	default:
		pr_err("audiolib: invalid tx device group %d\n", device_group);
		break;
	}
}

static void _audio_rx_clk_disable(void)
{
	switch (audio_rx_device_group) {
	case 0:
		icodec_rx_clk_refcount--;
		if (icodec_rx_clk_refcount == 0) {
			clk_disable(icodec_rx_clk);
			audio_rx_device_group = -1;
		}
		break;
	case 2:
		ecodec_clk_refcount--;
		if (ecodec_clk_refcount == 0) {
			clk_disable(ecodec_clk);
			audio_rx_device_group = -1;
		}
		break;
	case 6:
		sdac_clk_refcount--;
		if (sdac_clk_refcount == 0) {
			clk_disable(sdac_clk);
			audio_rx_device_group = -1;
		}
		break;
	default:
		pr_err("audiolib: invalid rx device group %d\n", audio_rx_device_group);
		break;
	}
}

static void _audio_tx_clk_disable(void)
{
	switch (audio_tx_device_group) {
	case 1:
		icodec_tx_clk_refcount--;
		if (icodec_tx_clk_refcount == 0) {
			clk_disable(icodec_tx_clk);
			audio_tx_device_group = -1;
		}
		break;
	case 3:
		ecodec_clk_refcount--;
		if (ecodec_clk_refcount == 0) {
			clk_disable(ecodec_clk);
			audio_tx_device_group = -1;
		}
		break;
	case 7:
		sdac_clk_refcount--;
		if (sdac_clk_refcount == 0) {
			clk_disable(sdac_clk);
			audio_tx_device_group = -1;
		}
		break;
	default:
		pr_err("audiolib: invalid tx device group %d\n", audio_tx_device_group);
		break;
	}
}

static void _audio_rx_clk_reinit(uint32_t rx_device)
{
	uint32_t device_group = get_device_group(rx_device);

	if (device_group != audio_rx_device_group)
		_audio_rx_clk_disable();

	audio_rx_device_id = rx_device;
	audio_rx_path_id = get_path_id(rx_device);

	if (device_group != audio_rx_device_group)
		_audio_rx_clk_enable();

}

static void _audio_tx_clk_reinit(uint32_t tx_device)
{
	uint32_t device_group = get_device_group(tx_device);

	if (device_group != audio_tx_device_group)
		_audio_tx_clk_disable();

	audio_tx_device_id = tx_device;
	audio_tx_path_id = get_path_id(tx_device);

	if (device_group != audio_tx_device_group)
		_audio_tx_clk_enable();
}

static DEFINE_MUTEX(audio_path_lock);
static int audio_rx_path_refcount;
static int audio_tx_path_refcount;

static int audio_rx_path_enable(int en)
{
	mutex_lock(&audio_path_lock);
	if (en) {
		audio_rx_path_refcount++;
		if (audio_rx_path_refcount == 1) {
			_audio_rx_clk_enable();
			_audio_rx_path_enable();
		}
	} else {
		audio_rx_path_refcount--;
		if (audio_rx_path_refcount == 0) {
			_audio_rx_path_disable();
			_audio_rx_clk_disable();
		}
	}
	mutex_unlock(&audio_path_lock);
	return 0;
}

static int audio_tx_path_enable(int en)
{
	mutex_lock(&audio_path_lock);
	if (en) {
		audio_tx_path_refcount++;
		if (audio_tx_path_refcount == 1) {
			_audio_tx_clk_enable();
			_audio_tx_path_enable();
		}
	} else {
		audio_tx_path_refcount--;
		if (audio_tx_path_refcount == 0) {
			_audio_tx_path_disable();
			_audio_tx_clk_disable();
		}
	}
	mutex_unlock(&audio_path_lock);
	return 0;
}

int q6audio_update_acdb(uint32_t id)
{
	mutex_lock(&audio_path_lock);
	//TODO: Need to update ACDB here
	mutex_unlock(&audio_path_lock);
	return 0;
}

int q6audio_set_tx_mute(uint32_t mute)
{
	uint32_t adev;
	//TODO: Need to remember current status
	mutex_lock(&audio_path_lock);

	adev = audio_tx_device_id;
	audio_tx_mute(audio_ctl, adev, !!mute);
	mutex_unlock(&audio_path_lock);
	return 0;
}

int q6audio_do_routing(uint32_t device_id)
{
	mutex_lock(&audio_path_lock);

	switch(device_id) {
	case ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR:
	case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_MONO:
	case ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO:
	case ADSP_AUDIO_DEVICE_ID_BT_SCO_SPKR:
	case ADSP_AUDIO_DEVICE_ID_BT_A2DP_SPKR:
	case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_SPKR:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_MONO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO_W_STEREO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_MONO_HEADSET:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_STEREO_W_STEREO_HEADSET:
		if (device_id != audio_rx_device_id) {
			if (audio_rx_path_refcount > 0) {
				_audio_rx_path_disable();
				_audio_rx_clk_reinit(device_id);
				_audio_rx_path_enable();
			} else {
				audio_rx_device_id = device_id;
				audio_rx_path_id = get_path_id(device_id);
			}
		}
		break;
	case ADSP_AUDIO_DEVICE_ID_HANDSET_MIC:
	case ADSP_AUDIO_DEVICE_ID_HEADSET_MIC:
	case ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MIC:
	case ADSP_AUDIO_DEVICE_ID_BT_SCO_MIC:
	case ADSP_AUDIO_DEVICE_ID_TTY_HEADSET_MIC:
		if (device_id != audio_tx_device_id) {
			if (audio_tx_path_refcount > 0) {
				_audio_tx_path_disable();
				_audio_tx_clk_reinit(device_id);
				_audio_tx_path_enable();
			} else {
				audio_tx_device_id = device_id;
				audio_tx_path_id = get_path_id(device_id);
			}
		}
		break;
	default:
		pr_err("%s: unsupported device 0x%08x\n", __func__, device_id);
		break;
	}

	mutex_unlock(&audio_path_lock);
	return 0;
}

int q6audio_set_route(const char *name)
{
	uint32_t route;
	if (!strcmp(name, "speaker")) {
		route = ADIE_PATH_SPEAKER_STEREO_RX;
	} else if (!strcmp(name, "headphones")) {
		route = ADIE_PATH_HEADSET_STEREO_RX;
	} else if (!strcmp(name, "handset")) {
		route = ADIE_PATH_HANDSET_RX;
	} else {
		return -EINVAL;
	}

	mutex_lock(&audio_path_lock);
	if (route == audio_rx_path_id)
		goto done;

	audio_rx_path_id = route;

	if (audio_rx_path_refcount > 0) {
		_audio_rx_path_disable();
		_audio_rx_path_enable();
	}
	if (audio_tx_path_refcount > 0) {
		_audio_tx_path_disable();
		_audio_tx_path_enable();
	}
done:
	mutex_unlock(&audio_path_lock);
	return 0;
}

struct audio_client *q6audio_open_pcm(uint32_t bufsz, uint32_t rate,
				      uint32_t channels, uint32_t flags)
{
	struct audio_client *ac;
	int res;

	printk("q6audio_open()\n");

	if (q6audio_init())
		return 0;

	ac = audio_client_alloc(bufsz);
	if (!ac)
		return 0;

	ac->session = session_alloc(ac);
	if (ac->session < 0) {
		res = -ENOMEM;
		goto fail;
	}

	ac->flags = flags;
	if (ac->flags & AUDIO_FLAG_WRITE)
		audio_rx_path_enable(1);
	else
		audio_tx_path_enable(1);

	pr_info("*** attach audio %p (%d) ***\n", ac, ac->session);
	ac->client = dal_attach(AUDIO_DAL_DEVICE, AUDIO_DAL_PORT, callback, 0);
	if (!ac->client) {
		res = -EIO;
		goto fail;
	}

	pr_info("*** init audio %p ***\n", ac);
	audio_init(ac->client, ac->session);

	pr_info("*** open audio %p ***\n", ac);
	ac->cb_status = -EBUSY;

	if (ac->flags & AUDIO_FLAG_WRITE)
		audio_out_open(ac->client, bufsz, rate, channels);
	else
		audio_in_open(ac->client, bufsz, rate, channels);

	wait_event(ac->wait, (ac->cb_status != -EBUSY));

	pr_info("*** stream start %p (%d) ***\n", ac, ac->session);
	ac->cb_status = -EBUSY;
	audio_command(ac->client, ADSP_AUDIO_IOCTL_CMD_STREAM_START);
	wait_event(ac->wait, (ac->cb_status != -EBUSY));

	if (!(ac->flags & AUDIO_FLAG_WRITE)) {
		ac->buf[0].used = 1;
		ac->buf[1].used = 1;
		q6audio_read(ac, &ac->buf[0]);
		q6audio_read(ac, &ac->buf[1]);
	}
	return ac;

fail:
	if (ac->flags & AUDIO_FLAG_WRITE)
		audio_rx_path_enable(0);
	else
		audio_tx_path_enable(0);
	audio_client_free(ac);
	return 0;
}

int q6audio_close(struct audio_client *ac)
{
	pr_info("*** close audio %p ***\n", ac);
	ac->cb_status = -EBUSY;
	audio_close(ac->client, ac->session);
	wait_event(ac->wait, (ac->cb_status != -EBUSY));

	pr_info("*** detach audio %p ***\n", ac);
	dal_detach(ac->client);
	ac->client = 0;

	if (ac->flags & AUDIO_FLAG_WRITE)
		audio_rx_path_enable(0);
	else
		audio_tx_path_enable(0);

	pr_info("*** done %p ***\n", ac);
	audio_client_free(ac);
	return 0;
}

