/* SPDX-License-Identifier: GPL-2.0
* Copyright (c) 2011-2016, The Linux Foundation
* Copyright (c) 2017, Linaro Limited
*/
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/asound.h>
#include <sound/pcm_params.h>
#include "q6afe.h"
#include "q6asm.h"
#include "q6adm.h"
#include "q6routing.h"

struct session_data {
	int state;
	int port_id;
	int path_type;
	int app_type;
	int acdb_id;
	int sample_rate;
	int bits_per_sample;
	int channels;
	int format;
	int perf_mode;
	int numcopps;
	int fedai_id;
	unsigned long copp_map;
};

struct msm_routing_data {
	struct session_data sessions[MAX_SESSIONS];
	struct device *dev;
	struct mutex lock;
};

static struct msm_routing_data *routing_data;

/**
 * q6routing_reg_phy_stream() - Register a new stream for route setup
 *
 * @fedai_id: Frontend dai id.
 * @perf_mode: Performace mode.
 * @stream_id: ASM stream id to map.
 * @stream_type: Direction of stream
 *
 * Return: Will be an negative on error or a zero on success.
 */
int q6routing_reg_phy_stream(int fedai_id, int perf_mode,
			   int stream_id, int stream_type)
{
	int j, topology, num_copps = 0;
	struct route_payload payload;
	int copp_idx;
	struct session_data *session;

	if (!routing_data) {
		pr_err("Routing driver not yet ready\n");
		return -EINVAL;
	}

	session = &routing_data->sessions[stream_id - 1];
	mutex_lock(&routing_data->lock);
	session->fedai_id = fedai_id;
	payload.num_copps = 0; /* only RX needs to use payload */
	topology = NULL_COPP_TOPOLOGY;
	copp_idx = q6adm_open(routing_data->dev, session->port_id,
			      session->path_type, session->sample_rate,
			      session->channels, topology, perf_mode,
			      session->bits_per_sample, 0, 0);
	if ((copp_idx < 0) || (copp_idx >= MAX_COPPS_PER_PORT)) {
		mutex_unlock(&routing_data->lock);
		return -EINVAL;
	}

	set_bit(copp_idx, &session->copp_map);
	for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
		unsigned long copp = session->copp_map;

		if (test_bit(j, &copp)) {
			payload.port_id[num_copps] = session->port_id;
			payload.copp_idx[num_copps] = j;
			num_copps++;
		}
	}

	if (num_copps) {
		payload.num_copps = num_copps;
		payload.session_id = stream_id;
		q6adm_matrix_map(routing_data->dev, session->path_type,
				 payload, perf_mode);
	}
	mutex_unlock(&routing_data->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(q6routing_reg_phy_stream);

static struct session_data *routing_get_session(struct msm_routing_data *data,
						int port_id, int port_type)
{
	int i;

	for (i = 0; i < MAX_SESSIONS; i++)
		if (port_id == data->sessions[i].port_id)
			return &data->sessions[i];

	return NULL;
}

static struct session_data *get_session_from_id(struct msm_routing_data *data,
						int fedai_id)
{
	int i;

	for (i = 0; i < MAX_SESSIONS; i++) {
		if (fedai_id == data->sessions[i].fedai_id)
			return &data->sessions[i];
	}

	return NULL;
}
/**
 * q6routing_dereg_phy_stream() - Deregister a stream
 *
 * @fedai_id: Frontend dai id.
 * @stream_type: Direction of stream
 *
 * Return: Will be an negative on error or a zero on success.
 */
void q6routing_dereg_phy_stream(int fedai_id, int stream_type)
{
	struct session_data *session;
	int idx;

	session = get_session_from_id(routing_data, fedai_id);
	if (!session)
		return;

	for_each_set_bit(idx, &session->copp_map, MAX_COPPS_PER_PORT)
		q6adm_close(routing_data->dev, session->port_id,
			    session->perf_mode, idx);

	session->fedai_id = -1;
	session->copp_map = 0;
}
EXPORT_SYMBOL_GPL(q6routing_dereg_phy_stream);

static int msm_routing_get_audio_mixer(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
	    snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct soc_mixer_control *mc =
	    (struct soc_mixer_control *)kcontrol->private_value;
	int session_id = mc->shift;
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(dapm);
	struct msm_routing_data *priv = snd_soc_platform_get_drvdata(platform);
	struct session_data *session = &priv->sessions[session_id];

	if (session->port_id != -1)
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	return 0;
}

static int msm_routing_put_audio_mixer(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
				    snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(dapm);
	struct msm_routing_data *data = snd_soc_platform_get_drvdata(platform);
	struct soc_mixer_control *mc =
		    (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;
	int be_id = mc->reg;
	int session_id = mc->shift;
	struct session_data *session = &data->sessions[session_id];

	if (ucontrol->value.integer.value[0]) {
		session->port_id = be_id;
		snd_soc_dapm_mixer_update_power(dapm, kcontrol, 1, update);
	} else {
		session->port_id = -1;
		snd_soc_dapm_mixer_update_power(dapm, kcontrol, 0, update);
	}

	return 1;
}

static const struct snd_kcontrol_new hdmi_mixer_controls[] = {
	SOC_SINGLE_EXT("MultiMedia1", AFE_PORT_HDMI_RX,
		       MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0,
		       msm_routing_get_audio_mixer,
		       msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia2", AFE_PORT_HDMI_RX,
		       MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0,
		       msm_routing_get_audio_mixer,
		       msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia3", AFE_PORT_HDMI_RX,
		       MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0,
		       msm_routing_get_audio_mixer,
		       msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia4", AFE_PORT_HDMI_RX,
		       MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0,
		       msm_routing_get_audio_mixer,
		       msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia5", AFE_PORT_HDMI_RX,
		       MSM_FRONTEND_DAI_MULTIMEDIA5, 1, 0,
		       msm_routing_get_audio_mixer,
		       msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia6", AFE_PORT_HDMI_RX,
		       MSM_FRONTEND_DAI_MULTIMEDIA6, 1, 0,
		       msm_routing_get_audio_mixer,
		       msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia7", AFE_PORT_HDMI_RX,
		       MSM_FRONTEND_DAI_MULTIMEDIA7, 1, 0,
		       msm_routing_get_audio_mixer,
		       msm_routing_put_audio_mixer),
	SOC_SINGLE_EXT("MultiMedia8", AFE_PORT_HDMI_RX,
		       MSM_FRONTEND_DAI_MULTIMEDIA8, 1, 0,
		       msm_routing_get_audio_mixer,
		       msm_routing_put_audio_mixer),
};

static const struct snd_soc_dapm_widget msm_qdsp6_widgets[] = {
	/* Frontend AIF */
	/* Widget name equals to Front-End DAI name<Need confirmation>,
	 * Stream name must contains substring of front-end dai name
	 */
	SND_SOC_DAPM_AIF_IN("MM_DL1", "MultiMedia1 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL2", "MultiMedia2 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL3", "MultiMedia3 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL4", "MultiMedia4 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL5", "MultiMedia5 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL6", "MultiMedia6 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL7", "MultiMedia7 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL8", "MultiMedia8 Playback", 0, 0, 0, 0),

	/* Mixer definitions */
	SND_SOC_DAPM_MIXER("HDMI Mixer", SND_SOC_NOPM, 0, 0,
			   hdmi_mixer_controls,
			   ARRAY_SIZE(hdmi_mixer_controls)),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"HDMI Mixer", "MultiMedia1", "MM_DL1"},
	{"HDMI Mixer", "MultiMedia2", "MM_DL2"},
	{"HDMI Mixer", "MultiMedia3", "MM_DL3"},
	{"HDMI Mixer", "MultiMedia4", "MM_DL4"},
	{"HDMI Mixer", "MultiMedia5", "MM_DL5"},
	{"HDMI Mixer", "MultiMedia6", "MM_DL6"},
	{"HDMI Mixer", "MultiMedia7", "MM_DL7"},
	{"HDMI Mixer", "MultiMedia8", "MM_DL8"},
	{"HDMI", NULL, "HDMI Mixer"},
	{"HDMI-RX", NULL, "HDMI"},
};

static int routing_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int be_id = rtd->cpu_dai->id;
	struct snd_soc_platform *platform = rtd->platform;
	struct msm_routing_data *data = snd_soc_platform_get_drvdata(platform);
	struct session_data *session;
	int port_id, port_type, path_type, bits_per_sample;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		path_type = ADM_PATH_PLAYBACK;
		port_type = MSM_AFE_PORT_TYPE_RX;
	}

	port_id = be_id;

	session = routing_get_session(data, port_id, port_type);

	if (!session) {
		pr_err("No session matrix setup yet..\n");
		return -EINVAL;
	}

	mutex_lock(&data->lock);

	session->path_type = path_type;
	session->sample_rate = params_rate(params);
	session->channels = params_channels(params);
	session->format = params_format(params);

	if (session->format == SNDRV_PCM_FORMAT_S16_LE)
		session->bits_per_sample = 16;
	else if (session->format == SNDRV_PCM_FORMAT_S24_LE)
		bits_per_sample = 24;

	mutex_unlock(&data->lock);
	return 0;
}

static int routing_close(struct snd_pcm_substream *substream)
{
	return 0;
}

static int routing_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_pcm_ops q6pcm_routing_ops = {
	.hw_params = routing_hw_params,
	.close = routing_close,
	.prepare = routing_prepare,
};

/* Not used but frame seems to require it */
static int msm_routing_probe(struct snd_soc_platform *platform)
{
	int i;

	for (i = 0; i < MAX_SESSIONS; i++)
		routing_data->sessions[i].port_id = -1;

	snd_soc_platform_set_drvdata(platform, routing_data);

	return 0;
}

static struct snd_soc_platform_driver msm_soc_routing_platform = {
	.ops = &q6pcm_routing_ops,
	.probe = msm_routing_probe,
	.component_driver = {
			     .dapm_widgets = msm_qdsp6_widgets,
			     .num_dapm_widgets = ARRAY_SIZE(msm_qdsp6_widgets),
			     .dapm_routes = intercon,
			     .num_dapm_routes = ARRAY_SIZE(intercon),
			     },
};

static int q6pcm_routing_probe(struct platform_device *pdev)
{

	routing_data = devm_kzalloc(&pdev->dev,
				    sizeof(*routing_data), GFP_KERNEL);
	if (!routing_data)
		return -ENOMEM;

	routing_data->dev = &pdev->dev;

	mutex_init(&routing_data->lock);
	dev_set_drvdata(&pdev->dev, routing_data);

	return devm_snd_soc_register_platform(&pdev->dev,
					      &msm_soc_routing_platform);
}

static int q6pcm_routing_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver q6pcm_routing_driver = {
	.driver = {
		   .name = "q6routing",
		   .owner = THIS_MODULE,
		   },
	.probe = q6pcm_routing_probe,
	.remove = q6pcm_routing_remove,
};

module_platform_driver(q6pcm_routing_driver);

MODULE_DESCRIPTION("Q6 Routing platform");
MODULE_LICENSE("GPL v2");
