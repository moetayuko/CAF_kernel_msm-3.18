/* arch/arm/mach-msm/include/mach/msm_qdsp6_audio.h
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

#ifndef _MACH_MSM_QDSP6_Q6AUDIO_
#define _MACH_MSM_QDSP6_Q6AUDIO_

struct audio_buffer {
        dma_addr_t phys;
        void *data;
        uint32_t size;
        uint32_t used;
};

struct audio_client {
        struct audio_buffer buf[2];
        int write_buf;
        int read_buf;
        int running;
        int session;

        wait_queue_head_t wait;
        struct dal_client *client;

        int cb_status;
};

/* Obtain a 16bit signed, interleaved audio channel of the specified
 * rate (Hz) and channels (1 or 2), with two buffers of bufsz bytes.
 */
struct audio_client *q6audio_open_pcm(uint32_t bufsz,
				      uint32_t rate, uint32_t channels);

int q6audio_close(struct audio_client *ac);

int q6audio_write(struct audio_client *ac, struct audio_buffer *ab);

struct q6audio_analog_ops {
	void (*init)(void);
	void (*speaker_enable)(int en);
	void (*headset_enable)(int en);
	void (*receiver_enable)(int en);
	void (*bt_sco_enable)(int en);
	void (*int_mic_enable)(int en);
	void (*ext_mic_enable)(int en);
};

void q6audio_register_analog_ops(struct q6audio_analog_ops *ops);

#endif
