/*
 * Rockchip machine ASoC driver for boards using a MAX90809 CODEC.
 *
 * Copyright (c) 2014, ROCKCHIP CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/hdmi-codec.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "rockchip_i2s.h"
#include "../codecs/ts3a227e.h"

#define DRV_NAME "rockchip-snd-max98090"

static struct snd_soc_jack headset_jack;

/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin headset_jack_pins[] = {
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},

};

static const struct snd_soc_dapm_widget rk_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_LINE("HDMI", NULL),
};

static const struct snd_soc_dapm_route rk_audio_map[] = {
	{"IN34", NULL, "Headset Mic"},
	{"Headset Mic", NULL, "MICBIAS"},
	{"DMICL", NULL, "Int Mic"},
	{"Headphone", NULL, "HPL"},
	{"Headphone", NULL, "HPR"},
	{"Speaker", NULL, "SPKL"},
	{"Speaker", NULL, "SPKR"},
	{"HDMI", NULL, "TX"},
};

static const struct snd_kcontrol_new rk_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("HDMI"),
};

static int rk_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk,
				     SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(cpu_dai->dev, "Can't set cpu dai clock %d\n", ret);
		return ret;
	}

	/* HDMI codec dai does not need to set sysclk. */
	if (!strcmp(rtd->dai_link->name, "HDMI"))
		return 0;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
				     SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(codec_dai->dev, "Can't set codec dai clock %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct snd_soc_ops rk_aif1_ops = {
	.hw_params = rk_aif1_hw_params,
};

SND_SOC_DAILINK_DEFS(analog,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "HiFi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hdmi,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(HDMI_CODEC_DRV_NAME, "i2s-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

enum {
	DAILINK_MAX98090,
	DAILINK_HDMI,
};

static struct snd_soc_jack rk_hdmi_jack;

static int rk_hdmi_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_component *component = runtime->codec_dai->component;
	int ret;

	/* enable jack detection */
	ret = snd_soc_card_jack_new(card, "HDMI Jack", SND_JACK_LINEOUT,
				    &rk_hdmi_jack, NULL, 0);
	if (ret) {
		dev_err(card->dev, "Can't new HDMI Jack %d\n", ret);
		return ret;
	}

	return hdmi_codec_set_jack_detect(component, &rk_hdmi_jack);
}

static int rk_max98090_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_component *comp = runtime->codec_dai->component;
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(comp);

	/*
	 * Enable max98090 and MICBIAS on it to provide headset jack
	 * detection. Without this, there will be phantom button when
	 * OMTP headset is plugged.
	 */
	snd_soc_dapm_force_enable_pin(dapm, "MICBIAS");
	snd_soc_dapm_force_enable_pin(dapm, "SHDN");

	return snd_soc_dapm_sync(dapm);
}

/* max98090 and HDMI codec dai_link */
static struct snd_soc_dai_link rk_dailinks[] = {
	[DAILINK_MAX98090] = {
		.name = "max98090",
		.stream_name = "Analog",
		.ops = &rk_aif1_ops,
		/* set max98090 as slave */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.init = rk_max98090_init,
		SND_SOC_DAILINK_REG(analog),
	},
	[DAILINK_HDMI] = {
		.name = "HDMI",
		.stream_name = "HDMI",
		.ops = &rk_aif1_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.init = rk_hdmi_init,
		SND_SOC_DAILINK_REG(hdmi),
	}
};

static int rk_98090_headset_init(struct snd_soc_component *component);

static struct snd_soc_aux_dev rk_98090_headset_dev = {
	.dlc = COMP_EMPTY(),
	.init = rk_98090_headset_init,
};

static struct snd_soc_card snd_soc_card_rk = {
	.name = "ROCKCHIP-I2S",
	.owner = THIS_MODULE,
	.dai_link = rk_dailinks,
	.num_links = ARRAY_SIZE(rk_dailinks),
	.aux_dev = &rk_98090_headset_dev,
	.num_aux_devs = 1,
	.dapm_widgets = rk_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rk_dapm_widgets),
	.dapm_routes = rk_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rk_audio_map),
	.controls = rk_mc_controls,
	.num_controls = ARRAY_SIZE(rk_mc_controls),
};

static int rk_98090_headset_init(struct snd_soc_component *component)
{
	int ret;

	/* Enable Headset and 4 Buttons Jack detection */
	ret = snd_soc_card_jack_new(&snd_soc_card_rk, "Headset Jack",
				    SND_JACK_HEADSET |
				    SND_JACK_BTN_0 | SND_JACK_BTN_1 |
				    SND_JACK_BTN_2 | SND_JACK_BTN_3,
				    &headset_jack,
				    headset_jack_pins,
				    ARRAY_SIZE(headset_jack_pins));
	if (ret)
		return ret;

	ret = ts3a227e_enable_jack_detect(component, &headset_jack);

	return ret;
}

static int snd_rk_mc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct snd_soc_card *card = &snd_soc_card_rk;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *np_analog;
	struct device_node *np_cpu;
	struct of_phandle_args args;

	/* register the soc card */
	card->dev = &pdev->dev;

	np_analog = of_parse_phandle(np, "rockchip,audio-codec", 0);
	if (!np_analog) {
		dev_err(&pdev->dev,
			"Property 'rockchip,audio-codec' missing or invalid\n");
		return -EINVAL;
	}
	rk_dailinks[DAILINK_MAX98090].codecs->of_node = np_analog;

	ret = of_parse_phandle_with_fixed_args(np, "rockchip,audio-codec",
					       0, 0, &args);
	if (ret) {
		dev_err(&pdev->dev,
			"Unable to parse property 'rockchip,audio-codec'\n");
		return ret;
	}

	ret = snd_soc_get_dai_name(
			&args, &rk_dailinks[DAILINK_MAX98090].codecs->dai_name);
	if (ret) {
		dev_err(&pdev->dev, "Unable to get codec dai_name\n");
		return ret;
	}

	np_cpu = of_parse_phandle(np, "rockchip,i2s-controller", 0);

	if (!np_cpu) {
		dev_err(&pdev->dev,
			"Property 'rockchip,i2s-controller' missing or invalid\n");
		return -EINVAL;
	}

	rk_dailinks[DAILINK_MAX98090].cpus->of_node = np_cpu;
	rk_dailinks[DAILINK_MAX98090].platforms->of_node = np_cpu;
	rk_dailinks[DAILINK_HDMI].cpus->of_node = np_cpu;
	rk_dailinks[DAILINK_HDMI].platforms->of_node = np_cpu;

	rk_98090_headset_dev.dlc.of_node = of_parse_phandle(np,
			"rockchip,headset-codec", 0);
	if (!rk_98090_headset_dev.dlc.of_node) {
		dev_err(&pdev->dev,
			"Property 'rockchip,headset-codec' missing/invalid\n");
		return -EINVAL;
	}

	ret = snd_soc_of_parse_card_name(card, "rockchip,model");
	if (ret) {
		dev_err(&pdev->dev,
			"Soc parse card name failed %d\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev,
			"Soc register card failed %d\n", ret);
		return ret;
	}

	return ret;
}

static const struct of_device_id rockchip_max98090_of_match[] = {
	{ .compatible = "rockchip,rockchip-audio-max98090", },
	{},
};

MODULE_DEVICE_TABLE(of, rockchip_max98090_of_match);

static struct platform_driver snd_rk_mc_driver = {
	.probe = snd_rk_mc_probe,
	.driver = {
		.name = DRV_NAME,
		.pm = &snd_soc_pm_ops,
		.of_match_table = rockchip_max98090_of_match,
	},
};

module_platform_driver(snd_rk_mc_driver);

MODULE_AUTHOR("jianqun <jay.xu@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip max98090 machine ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
