/* SPDX-License-Identifier: GPL-2.0
* Copyright (c) 2011-2016, The Linux Foundation
* Copyright (c) 2017, Linaro Limited
*/
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/soc/qcom/apr.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include "q6asm.h"
#include "common.h"

#define ASM_CMD_SHARED_MEM_MAP_REGIONS		0x00010D92
#define ASM_CMDRSP_SHARED_MEM_MAP_REGIONS	0x00010D93
#define ASM_CMD_SHARED_MEM_UNMAP_REGIONS	0x00010D94

#define TUN_READ_IO_MODE		0x0004	/* tunnel read write mode */
#define SYNC_IO_MODE			0x0001
#define ASYNC_IO_MODE			0x0002
#define ASM_SHIFT_GAPLESS_MODE_FLAG	31
#define ADSP_MEMORY_MAP_SHMEM8_4K_POOL	3

struct avs_cmd_shared_mem_map_regions {
	struct apr_hdr hdr;
	u16 mem_pool_id;
	u16 num_regions;
	u32 property_flag;
} __packed;

struct avs_shared_map_region_payload {
	u32 shm_addr_lsw;
	u32 shm_addr_msw;
	u32 mem_size_bytes;
} __packed;

struct avs_cmd_shared_mem_unmap_regions {
	struct apr_hdr hdr;
	u32 mem_map_handle;
} __packed;

struct audio_buffer {
	dma_addr_t phys;
	uint32_t used;
	uint32_t size;		/* size of buffer */
};

struct audio_port_data {
	struct audio_buffer *buf;
	uint32_t max_buf_cnt;
	uint32_t dsp_buf;
	uint32_t mem_map_handle;
};

struct audio_client {
	int session;
	app_cb cb;
	int cmd_state;
	void *priv;
	uint32_t io_mode;
	uint64_t time_stamp;
	struct apr_device *adev;
	struct mutex cmd_lock;
	/* idx:1 out port, 0: in port */
	struct audio_port_data port[2];
	wait_queue_head_t cmd_wait;
	int perf_mode;
	int stream_id;
	struct device *dev;
};

struct q6asm {
	struct apr_device *adev;
	int mem_state;
	struct device *dev;
	wait_queue_head_t mem_wait;
	struct mutex	session_lock;
	struct platform_device *pcmdev;
	struct audio_client *session[MAX_SESSIONS + 1];
};

static int q6asm_session_alloc(struct audio_client *ac, struct q6asm *a)
{
	int n = -EINVAL;

	mutex_lock(&a->session_lock);
	for (n = 1; n <= MAX_SESSIONS; n++) {
		if (!a->session[n]) {
			a->session[n] = ac;
			break;
		}
	}
	mutex_unlock(&a->session_lock);

	return n;
}

static bool q6asm_is_valid_audio_client(struct audio_client *ac)
{
	struct q6asm *a = dev_get_drvdata(ac->dev->parent);
	int n;

	for (n = 1; n <= MAX_SESSIONS; n++) {
		if (a->session[n] == ac)
			return 1;
	}

	return 0;
}

static void q6asm_session_free(struct audio_client *ac)
{
	struct q6asm *a = dev_get_drvdata(ac->dev->parent);

	if (!a)
		return;

	mutex_lock(&a->session_lock);
	a->session[ac->session] = 0;
	ac->session = 0;
	ac->perf_mode = LEGACY_PCM_MODE;
	mutex_unlock(&a->session_lock);
}

static inline void q6asm_add_mmaphdr(struct audio_client *ac,
				     struct apr_hdr *hdr, u32 pkt_size,
				     bool cmd_flg, u32 token)
{
	hdr->hdr_field = APR_SEQ_CMD_HDR_FIELD;
	hdr->src_port = 0;
	hdr->dest_port = 0;
	hdr->pkt_size = pkt_size;
	if (cmd_flg)
		hdr->token = token;
}

static inline void q6asm_add_hdr(struct audio_client *ac, struct apr_hdr *hdr,
				 uint32_t pkt_size, bool cmd_flg,
				 uint32_t stream_id)
{
	hdr->hdr_field = APR_SEQ_CMD_HDR_FIELD;
	hdr->src_svc = ac->adev->svc_id;
	hdr->src_domain = APR_DOMAIN_APPS;
	hdr->dest_svc = APR_SVC_ASM;
	hdr->dest_domain = APR_DOMAIN_ADSP;
	hdr->src_port = ((ac->session << 8) & 0xFF00) | (stream_id);
	hdr->dest_port = ((ac->session << 8) & 0xFF00) | (stream_id);
	hdr->pkt_size = pkt_size;
	if (cmd_flg)
		hdr->token = ac->session;
}

static int __q6asm_memory_unmap(struct audio_client *ac,
				phys_addr_t buf_add, int dir)
{
	struct avs_cmd_shared_mem_unmap_regions mem_unmap;
	struct q6asm *a = dev_get_drvdata(ac->dev->parent);
	int rc;

	if (!a)
		return -ENODEV;

	q6asm_add_mmaphdr(ac, &mem_unmap.hdr, sizeof(mem_unmap), true,
			  ((ac->session << 8) | dir));
	a->mem_state = -1;

	mem_unmap.hdr.opcode = ASM_CMD_SHARED_MEM_UNMAP_REGIONS;
	mem_unmap.mem_map_handle = ac->port[dir].mem_map_handle;

	if (mem_unmap.mem_map_handle == 0) {
		dev_err(ac->dev, "invalid mem handle\n");
		return -EINVAL;
	}

	rc = apr_send_pkt(a->adev, (uint32_t *) &mem_unmap);
	if (rc < 0)
		return rc;

	rc = wait_event_timeout(a->mem_wait, (a->mem_state >= 0),
				5 * HZ);
	if (!rc) {
		dev_err(ac->dev, "CMD timeout for memory_unmap 0x%x\n",
			mem_unmap.mem_map_handle);
		return -ETIMEDOUT;
	} else if (a->mem_state > 0) {
		return adsp_err_get_lnx_err_code(a->mem_state);
	}
	ac->port[dir].mem_map_handle = 0;

	return 0;
}

/**
 * q6asm_unmap_memory_regions() - unmap memory regions in the dsp.
 *
 * @dir: direction of audio stream
 * @ac: audio client instanace
 *
 * Return: Will be an negative value on failure or zero on success
 */
int q6asm_unmap_memory_regions(unsigned int dir, struct audio_client *ac)
{
	struct audio_port_data *port;
	int cnt = 0;
	int rc = 0;

	mutex_lock(&ac->cmd_lock);
	port = &ac->port[dir];
	if (!port->buf) {
		mutex_unlock(&ac->cmd_lock);
		return 0;
	}
	cnt = port->max_buf_cnt - 1;
	if (cnt >= 0) {
		rc = __q6asm_memory_unmap(ac, port->buf[dir].phys, dir);
		if (rc < 0) {
			dev_err(ac->dev, "%s: Memory_unmap_regions failed %d\n",
				__func__, rc);
			mutex_unlock(&ac->cmd_lock);
			return rc;
		}
	}

	port->max_buf_cnt = 0;
	kfree(port->buf);
	port->buf = NULL;
	mutex_unlock(&ac->cmd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(q6asm_unmap_memory_regions);

static int __q6asm_memory_map_regions(struct audio_client *ac, int dir,
				      uint32_t period_sz, uint32_t periods,
				      bool is_contiguous)
{
	struct avs_cmd_shared_mem_map_regions *mmap_regions = NULL;
	struct avs_shared_map_region_payload *mregions = NULL;
	struct q6asm *a = dev_get_drvdata(ac->dev->parent);
	struct audio_port_data *port = NULL;
	struct audio_buffer *ab = NULL;
	void *mmap_region_cmd = NULL;
	void *payload = NULL;
	int rc = 0;
	int i = 0;
	int cmd_size = 0;
	uint32_t num_regions;
	uint32_t buf_sz;

	if (!a)
		return -ENODEV;
	num_regions = is_contiguous ? 1 : periods;
	buf_sz = is_contiguous ? (period_sz * periods) : period_sz;
	buf_sz = PAGE_ALIGN(buf_sz);

	cmd_size = sizeof(*mmap_regions) + (sizeof(*mregions) * num_regions);

	mmap_region_cmd = kzalloc(cmd_size, GFP_KERNEL);
	if (!mmap_region_cmd)
		return -ENOMEM;

	mmap_regions = (struct avs_cmd_shared_mem_map_regions *)mmap_region_cmd;
	q6asm_add_mmaphdr(ac, &mmap_regions->hdr, cmd_size, true,
			  ((ac->session << 8) | dir));
	a->mem_state = -1;

	mmap_regions->hdr.opcode = ASM_CMD_SHARED_MEM_MAP_REGIONS;
	mmap_regions->mem_pool_id = ADSP_MEMORY_MAP_SHMEM8_4K_POOL;
	mmap_regions->num_regions = num_regions;
	mmap_regions->property_flag = 0x00;

	payload = ((u8 *) mmap_region_cmd +
		   sizeof(struct avs_cmd_shared_mem_map_regions));

	mregions = (struct avs_shared_map_region_payload *)payload;

	ac->port[dir].mem_map_handle = 0;
	port = &ac->port[dir];

	for (i = 0; i < num_regions; i++) {
		ab = &port->buf[i];
		mregions->shm_addr_lsw = lower_32_bits(ab->phys);
		mregions->shm_addr_msw = upper_32_bits(ab->phys);
		mregions->mem_size_bytes = buf_sz;
		++mregions;
	}

	rc = apr_send_pkt(a->adev, (uint32_t *) mmap_region_cmd);
	if (rc < 0)
		goto fail_cmd;

	rc = wait_event_timeout(a->mem_wait, (a->mem_state >= 0),
				5 * HZ);
	if (!rc) {
		dev_err(ac->dev, "timeout. waited for memory_map\n");
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (a->mem_state > 0) {
		rc = adsp_err_get_lnx_err_code(a->mem_state);
		goto fail_cmd;
	}
	rc = 0;
fail_cmd:
	kfree(mmap_region_cmd);
	return rc;
}

/**
 * q6asm_map_memory_regions() - map memory regions in the dsp.
 *
 * @dir: direction of audio stream
 * @ac: audio client instanace
 * @phys: physcial address that needs mapping.
 * @period_sz: audio period size
 * @periods: number of periods
 *
 * Return: Will be an negative value on failure or zero on success
 */
int q6asm_map_memory_regions(unsigned int dir, struct audio_client *ac,
			     dma_addr_t phys,
			     unsigned int period_sz, unsigned int periods)
{
	struct audio_buffer *buf;
	int cnt;
	int rc;

	if (ac->port[dir].buf) {
		dev_err(ac->dev, "Buffer already allocated\n");
		return 0;
	}

	mutex_lock(&ac->cmd_lock);

	buf = kzalloc(((sizeof(struct audio_buffer)) * periods), GFP_KERNEL);
	if (!buf) {
		mutex_unlock(&ac->cmd_lock);
		return -ENOMEM;
	}


	ac->port[dir].buf = buf;

	buf[0].phys = phys;
	buf[0].used = dir ^ 1;
	buf[0].size = period_sz;
	cnt = 1;
	while (cnt < periods) {
		if (period_sz > 0) {
			buf[cnt].phys = buf[0].phys + (cnt * period_sz);
			buf[cnt].used = dir ^ 1;
			buf[cnt].size = period_sz;
		}
		cnt++;
	}

	ac->port[dir].max_buf_cnt = periods;
	mutex_unlock(&ac->cmd_lock);

	rc = __q6asm_memory_map_regions(ac, dir, period_sz, periods, 1);
	if (rc < 0) {
		dev_err(ac->dev,
			"CMD Memory_map_regions failed %d for size %d\n", rc,
			period_sz);


		ac->port[dir].max_buf_cnt = 0;
		kfree(buf);
		ac->port[dir].buf = NULL;

		return rc;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(q6asm_map_memory_regions);

/**
 * q6asm_audio_client_free() - Freee allocated audio client
 *
 * @ac: audio client to free
 */
void q6asm_audio_client_free(struct audio_client *ac)
{
	q6asm_session_free(ac);
	kfree(ac);
}
EXPORT_SYMBOL_GPL(q6asm_audio_client_free);

static struct audio_client *q6asm_get_audio_client(struct q6asm *a,
						   int session_id)
{
	if ((session_id <= 0) || (session_id > MAX_SESSIONS)) {
		dev_err(a->dev, "invalid session: %d\n", session_id);
		goto err;
	}

	if (!a->session[session_id]) {
		dev_err(a->dev, "session not active: %d\n", session_id);
		goto err;
	}
	return a->session[session_id];
err:
	return NULL;
}

static int q6asm_srvc_callback(struct apr_device *adev, struct apr_client_data *data)
{
	struct q6asm *a, *q6asm = dev_get_drvdata(&adev->dev);
	struct audio_client *ac = NULL;
	struct audio_port_data *port;
	uint32_t dir = 0;
	uint32_t sid = 0;
	uint32_t *payload;

	if (!data) {
		dev_err(&adev->dev, "%s: Invalid CB\n", __func__);
		return 0;
	}

	payload = data->payload;
	sid = (data->token >> 8) & 0x0F;
	ac = q6asm_get_audio_client(q6asm, sid);
	if (!ac) {
		dev_err(&adev->dev, "Audio Client not active\n");
		return 0;
	}

	a = dev_get_drvdata(ac->dev->parent);
	if (data->opcode == APR_BASIC_RSP_RESULT) {
		switch (payload[0]) {
		case ASM_CMD_SHARED_MEM_MAP_REGIONS:
		case ASM_CMD_SHARED_MEM_UNMAP_REGIONS:
			if (payload[1] != 0) {
				dev_err(ac->dev,
					"cmd = 0x%x returned error = 0x%x sid:%d\n",
					payload[0], payload[1], sid);
				a->mem_state = payload[1];
			} else {
				a->mem_state = 0;
			}

			wake_up(&a->mem_wait);
			break;
		default:
			dev_err(&adev->dev, "command[0x%x] not expecting rsp\n",
				 payload[0]);
			break;
		}
		return 0;
	}

	dir = (data->token & 0x0F);
	port = &ac->port[dir];

	switch (data->opcode) {
	case ASM_CMDRSP_SHARED_MEM_MAP_REGIONS:{
			a->mem_state = 0;
			ac->port[dir].mem_map_handle = payload[0];
			wake_up(&a->mem_wait);
			break;
		}
	case ASM_CMD_SHARED_MEM_UNMAP_REGIONS:{
			a->mem_state = 0;
			ac->port[dir].mem_map_handle = 0;
			wake_up(&a->mem_wait);

			break;
		}
	default:
		dev_dbg(&adev->dev, "command[0x%x]success [0x%x]\n",
			payload[0], payload[1]);
		break;
	}
	if (ac->cb)
		ac->cb(data->opcode, data->token, data->payload, ac->priv);
	return 0;
}

/**
 * q6asm_get_session_id() - get session id for audio client
 *
 * @ac: audio client pointer
 *
 * Return: Will be an session id of the audio client.
 */
int q6asm_get_session_id(struct audio_client *c)
{
	return c->session;
}
EXPORT_SYMBOL_GPL(q6asm_get_session_id);

/**
 * q6asm_audio_client_alloc() - Allocate a new audio client
 *
 * @dev: Pointer to asm child device.
 * @cb: event callback.
 * @priv: private data associated with this client.
 *
 * Return: Will be an error pointer on error or a valid audio client
 * on success.
 */
struct audio_client *q6asm_audio_client_alloc(struct device *dev,
					      app_cb cb, void *priv)
{
	struct q6asm *a = dev_get_drvdata(dev->parent);
	struct audio_client *ac;
	int n;

	ac = kzalloc(sizeof(struct audio_client), GFP_KERNEL);
	if (!ac)
		return NULL;

	n = q6asm_session_alloc(ac, a);
	if (n <= 0) {
		dev_err(dev, "ASM Session alloc fail n=%d\n", n);
		kfree(ac);
		return NULL;
	}

	ac->session = n;
	ac->cb = cb;
	ac->dev = dev;
	ac->priv = priv;
	ac->io_mode = SYNC_IO_MODE;
	ac->perf_mode = LEGACY_PCM_MODE;
	/* DSP expects stream id from 1 */
	ac->stream_id = 1;
	ac->adev = a->adev;

	init_waitqueue_head(&ac->cmd_wait);
	mutex_init(&ac->cmd_lock);
	ac->cmd_state = 0;

	return ac;
}
EXPORT_SYMBOL_GPL(q6asm_audio_client_alloc);


static int q6asm_probe(struct apr_device *adev)
{
	struct q6asm *q6asm;

	q6asm = devm_kzalloc(&adev->dev, sizeof(*q6asm), GFP_KERNEL);
	if (!q6asm)
		return -ENOMEM;

	q6asm->dev = &adev->dev;
	q6asm->adev = adev;
	q6asm->mem_state = 0;
	init_waitqueue_head(&q6asm->mem_wait);
	mutex_init(&q6asm->session_lock);
	dev_set_drvdata(&adev->dev, q6asm);

	q6asm->pcmdev = platform_device_register_data(&adev->dev,
						      "q6asm_dai", -1, NULL, 0);

	return 0;
}

static int q6asm_remove(struct apr_device *adev)
{
	struct q6asm *q6asm = dev_get_drvdata(&adev->dev);

	platform_device_unregister(q6asm->pcmdev);

	return 0;
}

static const struct apr_device_id q6asm_id[] = {
	{"Q6ASM", APR_DOMAIN_ADSP, APR_SVC_ASM, APR_CLIENT_AUDIO},
	{}
};

static struct apr_driver qcom_q6asm_driver = {
	.probe = q6asm_probe,
	.remove = q6asm_remove,
	.callback = q6asm_srvc_callback,
	.id_table = q6asm_id,
	.driver = {
		   .name = "qcom-q6asm",
		   },
};

module_apr_driver(qcom_q6asm_driver);
MODULE_DESCRIPTION("Q6 Audio Stream Manager driver");
MODULE_LICENSE("GPL v2");
