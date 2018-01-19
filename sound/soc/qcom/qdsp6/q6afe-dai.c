/* SPDX-License-Identifier: GPL-2.0
* Copyright (c) 2011-2016, The Linux Foundation
* Copyright (c) 2017, Linaro Limited
*/
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "q6afe.h"

struct q6hdmi_dai_data {
	struct q6afe_port *port;
	struct q6afe_hdmi_cfg port_config;
	bool is_port_started;
};

static int q6hdmi_format_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct q6hdmi_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	dai_data->port_config.datatype = value;

	return 0;
}

static int q6hdmi_format_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{

	struct q6hdmi_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] =
		dai_data->port_config.datatype;

	return 0;
}

static const char * const hdmi_format[] = {
	"LPCM",
	"Compr"
};

static const struct soc_enum hdmi_config_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, hdmi_format),
};

static const struct snd_kcontrol_new hdmi_config_controls[] = {
	SOC_ENUM_EXT("HDMI RX Format", hdmi_config_enum[0],
				 q6hdmi_format_get,
				 q6hdmi_format_put),
};

static int q6hdmi_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct q6hdmi_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int channels = params_channels(params);

	dai_data->port_config.sample_rate = params_rate(params);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dai_data->port_config.bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		dai_data->port_config.bit_width = 24;
		break;
	}

	/*refer to HDMI spec CEA-861-E: Table 28 Audio InfoFrame Data Byte 4*/
	switch (channels) {
	case 2:
		dai_data->port_config.channel_allocation = 0;
		break;
	case 3:
		dai_data->port_config.channel_allocation = 0x02;
		break;
	case 4:
		dai_data->port_config.channel_allocation = 0x06;
		break;
	case 5:
		dai_data->port_config.channel_allocation = 0x0A;
		break;
	case 6:
		dai_data->port_config.channel_allocation = 0x0B;
		break;
	case 7:
		dai_data->port_config.channel_allocation = 0x12;
		break;
	case 8:
		dai_data->port_config.channel_allocation = 0x13;
		break;
	default:
		dev_err(dai->dev, "invalid Channels = %u\n", channels);
		return -EINVAL;
	}

	return 0;
}

static int q6hdmi_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct q6hdmi_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->is_port_started = false;

	return 0;
}

static void q6hdmi_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct q6hdmi_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc;

	rc = q6afe_port_stop(dai_data->port);
	if (rc < 0)
		dev_err(dai->dev, "fail to close AFE port\n");

	dai_data->is_port_started = false;

}

static int q6hdmi_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct q6hdmi_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc;

	if (dai_data->is_port_started) {
		/* stop the port and restart with new port config */
		rc = q6afe_port_stop(dai_data->port);
		if (rc < 0) {
			dev_err(dai->dev, "fail to close AFE port\n");
			return rc;
		}
	}

	q6afe_hdmi_port_prepare(dai_data->port, &dai_data->port_config);
	rc = q6afe_port_start(dai_data->port);
	if (rc < 0) {
		dev_err(dai->dev, "fail to start AFE port %x\n", dai->id);
		return rc;
	}
	dai_data->is_port_started = true;

	return 0;
}

static const struct snd_soc_dapm_route hdmi_dapm_routes[] = {
	{"HDMI Playback", NULL, "HDMI"},
};

static struct snd_soc_dai_ops q6hdmi_ops = {
	.prepare	= q6hdmi_prepare,
	.hw_params	= q6hdmi_hw_params,
	.shutdown	= q6hdmi_shutdown,
	.startup	= q6hdmi_startup,
};

static struct snd_soc_dai_driver q6afe_dai_hdmi_rx = {
	.playback = {
		.stream_name = "HDMI Playback",
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
		 SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
		.channels_min = 2,
		.channels_max = 8,
		.rate_max =     192000,
		.rate_min =	48000,
	},
	.ops = &q6hdmi_ops,
	.id = AFE_PORT_HDMI_RX,
	.name = "HDMI",
};

static const struct snd_soc_dapm_widget hdmi_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("HDMI", "HDMI Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_OUTPUT("HDMI-RX"),
};

static const struct snd_soc_component_driver msm_dai_hdmi_q6_component = {
	.name		= "msm-dai-q6-hdmi",
	.dapm_widgets = hdmi_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(hdmi_dapm_widgets),
	.controls = hdmi_config_controls,
	.num_controls = ARRAY_SIZE(hdmi_config_controls),
	.dapm_routes = hdmi_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(hdmi_dapm_routes),
};

static int q6afe_dai_dev_probe(struct platform_device *pdev)
{
	struct q6hdmi_dai_data *dai_data;
	int rc = 0;
	struct q6afe_port *port;

	dai_data = devm_kzalloc(&pdev->dev, sizeof(*dai_data), GFP_KERNEL);
	if (!dai_data)
		rc = -ENOMEM;

	port = q6afe_port_get_from_id(&pdev->dev, AFE_PORT_HDMI_RX);
	if (IS_ERR(port)) {
		dev_err(&pdev->dev, "Unable to get afe port\n");
		return -EPROBE_DEFER;
	}
	dai_data->port = port;
	dev_set_drvdata(&pdev->dev, dai_data);

	return devm_snd_soc_register_component(&pdev->dev,
					  &msm_dai_hdmi_q6_component,
					  &q6afe_dai_hdmi_rx, 1);
}

static int q6afe_dai_dev_remove(struct platform_device *pdev)
{
	struct q6hdmi_dai_data *dai_data = dev_get_drvdata(&pdev->dev);

	q6afe_port_put(dai_data->port);

	return 0;
}

static struct platform_driver q6afe_dai_driver = {
	.probe  = q6afe_dai_dev_probe,
	.remove = q6afe_dai_dev_remove,
	.driver = {
		.name = "q6afe_dai",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(q6afe_dai_driver);
