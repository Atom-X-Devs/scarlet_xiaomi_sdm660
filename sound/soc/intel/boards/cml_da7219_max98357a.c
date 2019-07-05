// SPDX-License-Identifier: GPL-2.0
// Copyright(c) 2018 Intel Corporation.

/*
 * Intel Cometlake I2S Machine driver for DA7219 + Maxim98357a codec
 *
 * Modified from:
 *   Intel I2S Machine driver for DA7219
 *   Intel Geminilake I2S Machine driver for DA7219 + Maxim98357a codec
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../../codecs/da7219.h"
#include "../../codecs/da7219-aad.h"
#include <sound/soc-acpi.h>
#include "../skylake/skl.h"
#include "../../codecs/hdac_hdmi.h"
#define CML_DIALOG_CODEC_DAI "da7219-hifi"

/* The platform clock outputs 24Mhz clock to codec as I2S MCLK */
#define CML_MAXIM_CODEC_DAI "HiFi"
#define MAXIM_DEV0_NAME "MX98357A:00"
#define NAME_SIZE 32

static struct snd_soc_jack cml_hdmi[3];

struct cml_hdmi_pcm {
	struct list_head head;
	struct snd_soc_dai *codec_dai;
	int device;
};

struct cml_card_private {
	struct snd_soc_jack cml_headset;
	struct list_head hdmi_pcm_list;
};

static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_dai *codec_dai;
	int ret = 0;

	codec_dai = snd_soc_card_get_codec_dai(card, CML_DIALOG_CODEC_DAI);
	if (!codec_dai) {
		dev_err(card->dev, "Codec dai not found; Unable to set/unset codec pll\n");
		return -EIO;
	}

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		ret = snd_soc_dai_set_pll(codec_dai, 0,
					DA7219_SYSCLK_MCLK, 0, 0);
		if (ret)
			dev_err(card->dev, "failed to stop PLL: %d\n", ret);
	} else if (SND_SOC_DAPM_EVENT_ON(event)) {
		ret = snd_soc_dai_set_pll(codec_dai, 0, DA7219_SYSCLK_PLL_SRM,
					0, DA7219_PLL_FREQ_OUT_98304);
		if (ret)
			dev_err(card->dev, "failed to start PLL: %d\n", ret);
	}

	return ret;
}

static const struct snd_kcontrol_new cml_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Spk"),
};

static const struct snd_soc_dapm_widget cml_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_SPK("Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cml_map[] = {
	/* HP jack connectors */
	{ "Headphone Jack", NULL, "HPL" },
	{ "Headphone Jack", NULL, "HPR" },

	/* speaker */
	{ "Spk", NULL, "Speaker" },

	/* HS Mic jacks */
	{ "MIC", NULL, "Headset Mic" },

	{ "Headphone Jack", NULL, "Platform Clock" },
	{ "Headset Mic", NULL, "Platform Clock" },
};

static int cml_da7219_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct cml_card_private *ctx = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_component *component = rtd->codec_dai->component;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_jack *jack;
	int ret;

	/* Configure sysclk for codec */
	ret = snd_soc_dai_set_sysclk(codec_dai, DA7219_CLKSRC_MCLK, 24576000,
						SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(rtd->dev, "can't set codec sysclk configuration\n");
		return ret;
	}

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
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEDOWN);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOICECOMMAND);
	da7219_aad_jack_det(component, &ctx->cml_headset);

	return ret;
}

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

/* Cometlake digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cml_dais[] = {
	/* Back End DAI links */
		{
		/* SSP0 - Codec */
		.name = "SSP0-Codec",
		.id = 0,
		.cpu_dai_name = "SSP0 Pin",
		.platform_name = "0000:00:1f.3",
		.no_pcm = 1,
		.codec_name = "i2c-DLGS7219:00",
		.codec_dai_name = CML_DIALOG_CODEC_DAI,
		.init = cml_da7219_codec_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
		.ignore_pmdown_time = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		},
	{
		/* SSP1 - Codec */
		.name = "SSP1-Codec",
		.id = 1,
		.cpu_dai_name = "SSP1 Pin",
		.platform_name = "0000:00:1f.3",
		.no_pcm = 1,
		.codec_name = MAXIM_DEV0_NAME,
		.codec_dai_name = CML_MAXIM_CODEC_DAI,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_pmdown_time = 1,
		.dpcm_playback = 1,
	},
	{
		.name = "dmic01",
		.id = 2,
		.cpu_dai_name = "DMIC01 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
	},
	{
		.name = "iDisp1",
		.id = 3,
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
		.id = 4,
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
		.id = 5,
		.cpu_dai_name = "iDisp3 Pin",
		.codec_name = "ehdaudio0D2",
		.codec_dai_name = "intel-hdmi-hifi3",
		.platform_name = "0000:00:1f.3",
		.init = cml_hdmi_init,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
};

static int cml_card_late_probe(struct snd_soc_card *card)
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
					SND_JACK_AVOUT, &cml_hdmi[i],
					NULL, 0);

		if (err)
			return err;

		err = hdac_hdmi_jack_init(pcm->codec_dai, pcm->device,
						&cml_hdmi[i]);
		if (err < 0)
			return err;

		i++;
	}

	if (!component)
		return -EINVAL;

	return hdac_hdmi_jack_port_init(component, &card->dapm);
}

/* Cometlake audio machine driver for DA7219 + MAX98357A */
static struct snd_soc_card cml_audio_card_da7219_m98357a = {
	.name = "cmlda7219max",
	.owner = THIS_MODULE,
	.dai_link = cml_dais,
	.num_links = ARRAY_SIZE(cml_dais),
	.controls = cml_controls,
	.num_controls = ARRAY_SIZE(cml_controls),
	.dapm_widgets = cml_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cml_widgets),
	.dapm_routes = cml_map,
	.num_dapm_routes = ARRAY_SIZE(cml_map),
	.fully_routed = true,
	.late_probe = cml_card_late_probe,
};

static int cml_audio_probe(struct platform_device *pdev)
{
	struct cml_card_private *ctx;
	struct snd_soc_acpi_mach *mach;
	const char *platform_name;
	struct snd_soc_card *card;
	int ret;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_ATOMIC);
	if (!ctx)
		return -ENOMEM;

	INIT_LIST_HEAD(&ctx->hdmi_pcm_list);

	card = &cml_audio_card_da7219_m98357a;
	card->dev = &pdev->dev;

	/* override plaform name, if required */
	mach = (&pdev->dev)->platform_data;
	platform_name = mach->mach_params.platform;

	ret = snd_soc_fixup_dai_links_platform_name(card, platform_name);
	if (ret)
		return ret;

	snd_soc_card_set_drvdata(card, ctx);

	return devm_snd_soc_register_card(&pdev->dev, card);
}

static const struct platform_device_id cml_board_ids[] = {
	{
		.name = "cml_da7219_max98357a",
		.driver_data =
			(kernel_ulong_t)&cml_audio_card_da7219_m98357a,
	},
	{ }
};

static struct platform_driver cml_audio = {
	.probe = cml_audio_probe,
	.driver = {
		.name = "cml_da7219_max98357a",
		.pm = &snd_soc_pm_ops,
	},
	.id_table = cml_board_ids,
};
module_platform_driver(cml_audio)

/* Module information */
MODULE_DESCRIPTION("Cometlake Audio Machine driver-DA7219 & MAX98357A in I2S mode");
MODULE_AUTHOR(" Mac Chiang <mac.chiang@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cml_da7219_max98357a");
