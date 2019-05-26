// SPDX-License-Identifier: GPL-2.0
// Copyright(c) 2019 Intel Corporation.

/*
 * Intel Cometlake I2S Machine driver for RT1011 + RT5682 codec
 *
 * Modified from:
 *   Intel I2S Machine driver for RT5682 + Maxim98357a
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <asm/cpu_device_id.h>
#include <linux/acpi.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/rt5682.h>
#include <sound/soc-acpi.h>
#include "../skylake/skl.h"
#include "../../codecs/rt1011.h"
#include "../../codecs/rt5682.h"
#include "../../codecs/hdac_hdmi.h"

/* The platform clock outputs 24Mhz clock to codec as I2S MCLK */
#define CML_PLAT_CLK	24000000
#define CML_RT1011_CODEC_DAI "rt1011-aif"
#define CML_RT5682_CODEC_DAI "rt5682-aif1"
#define DUAL_CHANNEL 2
#define QUAD_CHANNEL 4
#define NAME_SIZE 32

static struct snd_soc_jack sof_hdmi[3];

struct cml_hdmi_pcm {
	struct list_head head;
	struct snd_soc_dai *codec_dai;
	int device;
};

struct cml_acpi_card {
	char *codec_id;
	int codec_type;
	struct snd_soc_card *soc_card;
};

struct cml_card_private {
	struct cml_acpi_card *acpi_card;
	char codec_name[SND_ACPI_I2C_ID_LEN];
	struct clk *mclk;
	struct snd_soc_jack cml_headset;
	struct list_head hdmi_pcm_list;
};

static const struct snd_kcontrol_new cml_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("TL Ext Spk"),
	SOC_DAPM_PIN_SWITCH("TR Ext Spk"),
	SOC_DAPM_PIN_SWITCH("WL Ext Spk"),
	SOC_DAPM_PIN_SWITCH("WR Ext Spk"),
};

static const struct snd_soc_dapm_widget cml_rt1011_rt5682_widgets[] = {
	SND_SOC_DAPM_SPK("TL Ext Spk", NULL),
	SND_SOC_DAPM_SPK("TR Ext Spk", NULL),
	SND_SOC_DAPM_SPK("WL Ext Spk", NULL),
	SND_SOC_DAPM_SPK("WR Ext Spk", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static const struct snd_soc_dapm_route cml_rt1011_rt5682_map[] = {
	/*speaker*/
	{"TL Ext Spk", NULL, "TL SPO"},
	{"TR Ext Spk", NULL, "TR SPO"},
	{"WL Ext Spk", NULL, "WL SPO"},
	{"WR Ext Spk", NULL, "WR SPO"},

	/* HP jack connectors - unknown if we have jack detection */
	{ "Headphone Jack", NULL, "HPOL" },
	{ "Headphone Jack", NULL, "HPOR" },

	/* other jacks */
	{ "IN1P", NULL, "Headset Mic" },

};

static int cml_rt5682_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct cml_card_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_component *component = rtd->codec_dai->component;
	struct snd_soc_jack *jack;
	int ret;

	/* need to enable ASRC function for 24MHz mclk rate */
	rt5682_sel_asrc_clk_src(component, RT5682_DA_STEREO1_FILTER,
					RT5682_CLK_SEL_I2S1_ASRC);


	/*
	 * Headset buttons map to the google Reference headset.
	 * These can be configured by userspace.
	 */
	ret = snd_soc_card_jack_new(rtd->card, "Headset Jack",
			SND_JACK_HEADSET | SND_JACK_BTN_0 | SND_JACK_BTN_1 |
			SND_JACK_BTN_2 | SND_JACK_BTN_3 | SND_JACK_LINEOUT,
			&ctx->cml_headset, NULL, 0);
	if (ret) {
		dev_err(rtd->dev, "Headset Jack creation failed: %d\n", ret);
		return ret;
	}

	jack = &ctx->cml_headset;

	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);
	ret = snd_soc_component_set_jack(component, jack, NULL);

	if (ret) {
		dev_err(rtd->dev, "Headset Jack call-back failed: %d\n", ret);
		return ret;
	}

	return ret;
};

static int cml_rt5682_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int clk_id, clk_freq, pll_out, ret;

	clk_id = RT5682_PLL1_S_MCLK;
	clk_freq = 24000000;


	pll_out = params_rate(params) * 512;

	ret = snd_soc_dai_set_pll(codec_dai, 0, clk_id, clk_freq, pll_out);
	if (ret < 0)
		dev_err(rtd->dev, "snd_soc_dai_set_pll err = %d\n", ret);

	/* Configure sysclk for codec */
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5682_SCLK_S_PLL1,
				     pll_out, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(rtd->dev, "snd_soc_dai_set_sysclk err = %d\n", ret);

	/*
	 * slot_width should equal or large than data length, set them
	 * be the same
	 */
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0x0, 0x0, 2,
				       params_width(params));
	if (ret < 0) {
		dev_err(rtd->dev, "set TDM slot err:%d\n", ret);
		return ret;
	}

	return ret;
}

static int cml_rt1011_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	int err, srate, i, ret;

	srate = params_rate(params);

	for (i = 0; i < rtd->num_codecs; i++) {
		codec_dai = rtd->codec_dais[i];

		/* 100 Fs to drive 24 bit data */
		err = snd_soc_dai_set_pll(codec_dai, 0, RT1011_PLL1_S_BCLK,
					100 * srate, 256 * srate);
		if (err < 0) {
			dev_err(card->dev, "codec_dai clock not set\n");
			return err;
		}

		err = snd_soc_dai_set_sysclk(codec_dai, RT1011_FS_SYS_PRE_S_PLL1, 256* srate,
					SND_SOC_CLOCK_IN);
		if (err < 0) {
			dev_err(card->dev, "codec_dai clock not set\n");
			return err;
		}

		/* TDM 4 slots 24 bit, set Rx & Tx bitmask to 4 active slots */
		ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xF, 0xF, 4, 24);
		if (ret < 0) {
			dev_err(rtd->dev, "can't set codec TDM slot %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static struct snd_soc_ops cml_rt5682_ops = {
	.hw_params = cml_rt5682_hw_params,
};

static int sof_card_late_probe(struct snd_soc_card *card)
{
	struct cml_card_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_component *component = NULL;
	char jack_name[NAME_SIZE];
	struct cml_hdmi_pcm *pcm;
	int err = 0;
	int i = 0;

	list_for_each_entry(pcm, &ctx->hdmi_pcm_list, head) {
		component = pcm->codec_dai->component;
		snprintf(jack_name, sizeof(jack_name),
			 "HDMI/DP, pcm=%d Jack", pcm->device);
		err = snd_soc_card_jack_new(card, jack_name,
					    SND_JACK_AVOUT, &sof_hdmi[i],
					    NULL, 0);

		if (err)
			return err;

		err = hdac_hdmi_jack_init(pcm->codec_dai, pcm->device,
					  &sof_hdmi[i]);
		if (err < 0)
			return err;

		i++;
	}
	if (!component)
		return -EINVAL;

	return hdac_hdmi_jack_port_init(component, &card->dapm);
}

static int cml_codec_init(struct snd_soc_pcm_runtime *runtime)
{
	struct cml_card_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	int ret;

	if (!ctx) {
		printk("%s(%d)... ctx null\n", __func__, __LINE__);
		return 0;
	}

	ret = clk_prepare_enable(ctx->mclk);
	if (!ret)
		clk_disable_unprepare(ctx->mclk);

	ret = clk_set_rate(ctx->mclk, CML_PLAT_CLK);

	if (ret)
		dev_err(runtime->dev, "unable to set MCLK rate\n");

	return ret;
}

static const struct snd_soc_ops cml_rt1011_ops = {
	.hw_params = cml_rt1011_hw_params,
};


static int cml_hdmi_init(struct snd_soc_pcm_runtime *rtd)
{
	struct cml_card_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *dai = rtd->codec_dai;
	struct cml_hdmi_pcm *pcm;

	pcm = devm_kzalloc(rtd->card->dev, sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;

	pcm->device = dai->id;
	pcm->codec_dai = dai;

	list_add_tail(&pcm->head, &ctx->hdmi_pcm_list);

	return 0;
}

static int cml_dmic_fixup(struct snd_soc_pcm_runtime *rtd,
		struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
				SNDRV_PCM_HW_PARAM_CHANNELS);

	/*
	 * set BE channel constraint as user FE channels
	 */
	channels->min = channels->max = 4;

	return 0;
}


static struct snd_soc_dai_link_component rt1011_codec_component[] = {
	{
		.name = "i2c-10EC1011:00",
		.dai_name = "rt1011-aif",
	},
	{
		.name = "i2c-10EC1011:01",
		.dai_name = "rt1011-aif",
	},
	{
		.name = "i2c-10EC1011:02",
		.dai_name = "rt1011-aif",
	},
	{
		.name = "i2c-10EC1011:03",
		.dai_name = "rt1011-aif",
	},
};

/* Cometlake digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cml_rt1011_rt5682_dailink[] = {
	/* Back End DAI links */
	{
		/* SSP0 - Codec */
		.name = "SSP0-Codec",
		.id = 0,
		.cpu_dai_name = "SSP0 Pin",
		.platform_name = "0000:00:1f.3",
		.no_pcm = 1,
		.codec_name = "i2c-10EC5682:00",
		.codec_dai_name = CML_RT5682_CODEC_DAI,
		.init = cml_rt5682_codec_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_pmdown_time = 1,
		.ops = &cml_rt5682_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "dmic01",
		.id = 1,
		.cpu_dai_name = "DMIC01 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.be_hw_params_fixup = cml_dmic_fixup,
		.dpcm_capture = 1,
		.no_pcm = 1,
	},
	{
		.name = "iDisp1",
		.id = 2,
		.cpu_dai_name = "iDisp1 Pin",
		.codec_name = "ehdaudio0D2",
		.codec_dai_name = "intel-hdmi-hifi1",
		.platform_name = "0000:00:1f.3",
		.init = cml_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		.name = "iDisp2",
		.id = 3,
		.cpu_dai_name = "iDisp2 Pin",
		.codec_name = "ehdaudio0D2",
		.codec_dai_name = "intel-hdmi-hifi2",
		.platform_name = "0000:00:1f.3",
		.init = cml_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		.name = "iDisp3",
		.id = 4,
		.cpu_dai_name = "iDisp3 Pin",
		.codec_name = "ehdaudio0D2",
		.codec_dai_name = "intel-hdmi-hifi3",
		.platform_name = "0000:00:1f.3",
		.init = cml_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP1 - Codec */
		.name = "SSP1-Codec",
		.id = 5,
		.cpu_dai_name = "SSP1 Pin",
		.platform_name = "0000:00:1f.3",
		.no_pcm = 1,
		.codecs = rt1011_codec_component,
		.num_codecs = ARRAY_SIZE(rt1011_codec_component),
		.dai_fmt = SND_SOC_DAIFMT_DSP_B |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.init = cml_codec_init,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &cml_rt1011_ops,
	},
};

static struct snd_soc_codec_conf cml_rt1011_conf[] = {
	{
		.dev_name = "i2c-10EC1011:00",
		.name_prefix = "WL",
	},
	{
		.dev_name = "i2c-10EC1011:01",
		.name_prefix = "WR",
	},
	{
		.dev_name = "i2c-10EC1011:02",
		.name_prefix = "TL",
	},
	{
		.dev_name = "i2c-10EC1011:03",
		.name_prefix = "TR",
	},
};

/* Cometlake audio machine driver for RT1011 */
static struct snd_soc_card snd_soc_card_cml = {
	.name = "cml_rt1011_rt5682",
	.dai_link = cml_rt1011_rt5682_dailink,
	.num_links = ARRAY_SIZE(cml_rt1011_rt5682_dailink),
	.codec_conf = cml_rt1011_conf,
	.num_configs = ARRAY_SIZE(cml_rt1011_conf),
	.dapm_widgets = cml_rt1011_rt5682_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cml_rt1011_rt5682_widgets),
	.dapm_routes = cml_rt1011_rt5682_map,
	.num_dapm_routes = ARRAY_SIZE(cml_rt1011_rt5682_map),
	.controls = cml_controls,
	.num_controls = ARRAY_SIZE(cml_controls),
	.fully_routed = true,
	.late_probe = sof_card_late_probe,
};

static struct cml_acpi_card snd_soc_cards[] = {
	{"10EC1011", 0, &snd_soc_card_cml},
};

static int snd_cml_rt1011_probe(struct platform_device *pdev)
{
	struct cml_card_private *ctx;
	struct snd_soc_acpi_mach *mach;
	const char *platform_name;
	struct snd_soc_card *card = snd_soc_cards[0].soc_card;
	bool found = false;
	int ret_val = 0;
	int i;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_ATOMIC);
	if (!ctx)
		return -ENOMEM;

	INIT_LIST_HEAD(&ctx->hdmi_pcm_list);
	mach = (&pdev->dev)->platform_data;

	for (i = 0; i < ARRAY_SIZE(snd_soc_cards); i++) {
		if (acpi_dev_found(snd_soc_cards[i].codec_id) &&
			(!strncmp(snd_soc_cards[i].codec_id, mach->id, 8))) {
			dev_dbg(&pdev->dev,
				"found codec %s\n", snd_soc_cards[i].codec_id);
			card = snd_soc_cards[i].soc_card;
			ctx->acpi_card = &snd_soc_cards[i];
			found = true;
			break;
		}
	}

	if (!found) {
		dev_err(&pdev->dev, "No matching HID found in supported list\n");
		return -ENODEV;
	}

	card->dev = &pdev->dev;

	/* override plaform name, if required */
	mach = (&pdev->dev)->platform_data;
	platform_name = mach->mach_params.platform;

	ret_val = snd_soc_fixup_dai_links_platform_name(card,
							platform_name);
	if (ret_val)
		return ret_val;

	snd_soc_card_set_drvdata(card, ctx);

	ret_val = devm_snd_soc_register_card(&pdev->dev, &snd_soc_card_cml);

	if (ret_val) {
		dev_err(&pdev->dev,
			"snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}

	platform_set_drvdata(pdev, card);

	return ret_val;
}

static struct platform_driver snd_cml_rt1011_rt5682_driver = {
	.driver = {
		.name = "cml_rt1011_rt5682",
	},
	.probe = snd_cml_rt1011_probe,
};
module_platform_driver(snd_cml_rt1011_rt5682_driver);

/* Module information */
MODULE_DESCRIPTION("Cometlake Audio Machine driver-RT1011 in I2S mode");
MODULE_AUTHOR("Naveen Manohar <naveen.m@intel.com>");
MODULE_AUTHOR("Sathya Prakash M R <sathya.prakash.m.r@intel.com>");
MODULE_AUTHOR("Shuming Fan <shumingf@realtek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cml_rt1011_rt5682");
