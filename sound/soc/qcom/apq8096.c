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
static struct snd_soc_dai_link msm8996_dai_links[] = {
	/* FrontEnd DAI Links */
	{
		.name		= "MultiMedia1 Playback",
		.stream_name	= "MultiMedia1",
		.cpu_dai_name	= "MM_DL1",
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
		.no_pcm		= 1,
		.dpcm_playback	= 1,
		.be_hw_params_fixup = msm8996_be_hw_params_fixup,
		.codec_dai_name	= "i2s-hifi",
		.codec_name = "hdmi-audio-codec.0.auto",
	},
};

static int apq8096_sbc_parse_of(struct snd_soc_card *card)
{
	struct device *dev = card->dev;
	int ret;

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret)
		dev_err(dev, "Error parsing card name: %d\n", ret);

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
