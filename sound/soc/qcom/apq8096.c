/*
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/dma-mapping.h>
#include <sound/pcm.h>

#include "qdsp6/q6afe.h"

#define SLIM_MAX_RX_PORTS 16
#define SLIM_MAX_TX_PORTS 16

int msm8996_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	return 0;
}

static struct snd_soc_dapm_route wcd9335_audio_paths[] = {
	{"MIC BIAS1", NULL, "MCLK"},
	{"MIC BIAS2", NULL, "MCLK"},
	{"MIC BIAS3", NULL, "MCLK"},
	{"MIC BIAS4", NULL, "MCLK"},
};

static int apq8096_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	int err;
	void *config_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	void *mbhc_calibration;
	struct snd_card *card;
	struct snd_info_entry *entry;
	struct msm8996_asoc_mach_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);

	/* Codec SLIMBUS configuration
	 * RX1, RX2, RX3, RX4, RX5, RX6, RX7, RX8, RX9, RX10, RX11, RX12, RX13
	 * TX1, TX2, TX3, TX4, TX5, TX6, TX7, TX8, TX9, TX10, TX11, TX12, TX13
	 * TX14, TX15, TX16
	 */
	unsigned int rx_ch[16] = {144, 145, 146, 147, 148, 149, 150,
					    151, 152, 153, 154, 155, 156};
	unsigned int tx_ch[16] = {128, 129, 130, 131, 132, 133,
					    134, 135, 136, 137, 138, 139,
					    140, 141, 142, 143};

	snd_soc_dapm_add_routes(dapm, wcd9335_audio_paths,
				ARRAY_SIZE(wcd9335_audio_paths));
	snd_soc_dapm_sync(dapm);

	snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
				    tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

	return 0;
out:
	return err;
}

static int msm8996_snd_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	//struct snd_soc_dai_link *dai_link = rtd->dai_link;

	int i, ret = 0;
	u32 rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	u32 my_rx_ch[SLIM_MAX_RX_PORTS];

	u32 rx_ch_cnt = 0, tx_ch_cnt = 0;
	//u32 user_set_tx_ch = 0;
	u32 rx_ch_count;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ret = snd_soc_dai_get_channel_map(codec_dai,
					&tx_ch_cnt, tx_ch, &rx_ch_cnt , rx_ch);
		if (ret < 0) {
			pr_err("%s: failed to get codec chan map, err:%d\n",
				__func__, ret);
			goto end;
		}

		my_rx_ch[0] = 0x95;
		my_rx_ch[1] = 0x96;
		rx_ch_count = 2;// msm_slim_0_rx_ch;
		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, 0,
						  rx_ch_count, my_rx_ch);
						// rx_ch_count, rx_ch );
		if (ret < 0) {
			pr_err("%s: failed to set cpu chan map, err:%d\n",
				__func__, ret);
			goto end;
		}
	}
end:
	return ret;
}

static struct snd_soc_ops msm8996_be_ops = {
	.hw_params = msm8996_snd_hw_params,
};

static struct snd_soc_dai_link msm8996_dai_links[] = {
	/* FrontEnd DAI Links */
	{
		.name		= "MultiMedia1 Playback",
		.stream_name	= "MultiMedia1",
		.cpu_dai_name	= "MultiMedia1",
		.platform_name	= "q6asm_dai",
		.dynamic	= 1,
		.dpcm_playback	= 1,
		.codec_dai_name	= "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name		= "MultiMedia2 Playback",
		.stream_name	= "MultiMedia2",
		.cpu_dai_name	= "MultiMedia2",
		.platform_name	= "q6asm_dai",
		.dynamic	= 1,
		.dpcm_playback	= 1,

		.codec_dai_name	= "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	/* Backend DAI Links */
	{
		.name		= "HDMI Playback",
		.stream_name	= "q6afe_dai",
		.cpu_dai_name	= "HDMI",
		.platform_name	= "q6routing",
		.id =  AFE_PORT_HDMI_RX,
		.no_pcm		= 1,
		.dpcm_playback	= 1,
		.be_hw_params_fixup = msm8996_be_hw_params_fixup,
		.codec_dai_name	= "i2s-hifi",
		.codec_name = "hdmi-audio-codec.0.auto",
	},
	{
		.name		= "Slimbus6 Playback",
		.stream_name	= "q6afe_dai",
		.cpu_dai_name	= "SLIMBUS_6_RX",
		.platform_name	= "q6routing",
		.id =  SLIMBUS_6_RX,
		.no_pcm		= 1,
		.dpcm_playback	= 1,
		.init = apq8096_audrx_init,
		.be_hw_params_fixup = msm8996_be_hw_params_fixup,
		.codec_dai_name	= "wcd9335_rx4",
		.codec_name = "91c0000.sc:tas:wcd",
		.ops = &msm8996_be_ops,
	},
};

static int apq8096_sbc_parse_of(struct snd_soc_card *card)
{
	struct device *dev = card->dev;
	int ret;

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret)
		dev_err(dev, "Error parsing card name: %d\n", ret);

	if (of_property_read_bool(dev->of_node, "qcom,audio-routing"))
		ret = snd_soc_of_parse_audio_routing(card,
					"qcom,audio-routing");

	return ret;
}

static int msm_snd_apq8096_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card;

	card = devm_kzalloc(&pdev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->dev = &pdev->dev;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	card->dai_link = msm8996_dai_links;
	card->num_links	= ARRAY_SIZE(msm8996_dai_links);

	ret = apq8096_sbc_parse_of(card);
	if (ret) {
		dev_err(&pdev->dev, "Error parsing OF data\n");
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "sound card register failed (%d)!\n", ret);
	else
		dev_err(&pdev->dev, "sound card register Sucessfull\n");

	return ret;
}

static const struct of_device_id msm_snd_apq8096_dt_match[] = {
	{.compatible = "qcom,apq8096-sndcard"},
	{}
};

static struct platform_driver msm_snd_apq8096_driver = {
	.probe  = msm_snd_apq8096_probe,
	.driver = {
		.name = "msm-snd-apq8096",
		.owner = THIS_MODULE,
		.of_match_table = msm_snd_apq8096_dt_match,
	},
};
module_platform_driver(msm_snd_apq8096_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("APQ8096 ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
