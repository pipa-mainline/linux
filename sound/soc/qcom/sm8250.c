// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <linux/soundwire/sdw.h>
#include <sound/jack.h>
#include <linux/input-event-codes.h>
#include "qdsp6/q6afe.h"
#include "common.h"
#include "sdw.h"

#define DRIVER_NAME "sm8250"
#define MI2S_BCLK_RATE 1536000
#define TDM_BCLK_RATE 12288000

#define PRINTK_LABEL "sm8250_soc: "
static unsigned int tdm_slot_offset[8] = { 0, 4, 8, 12, 16, 20, 24, 28 };

struct sm8250_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct sdw_stream_runtime *sruntime[AFE_PORT_MAX];
	struct snd_soc_jack jack;
	bool jack_setup;
};

static int sm8250_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	printk(PRINTK_LABEL "sm8250_snd_init\n");

	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	printk(KERN_INFO "sm8250_snd_init test1xd\n");

	return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
}

static int sm8250_tdm_snd_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;

	printk(PRINTK_LABEL "sm8250_tdm_snd_hw_params, id=%d\n", cpu_dai->id);
	printk(PRINTK_LABEL "CPU DAI name: %s\n", cpu_dai->name);
	int ret = 0;
	int channels, slots, slot_width;
	bool is_aw88261 = false;
	struct snd_soc_dai *codec_dai;
	int j;

	/* Check if we're dealing with an AW88261 codec */
	for_each_rtd_codec_dais(rtd, j, codec_dai) {
		if (strstr(codec_dai->component->name, "aw88261") != NULL) {
			is_aw88261 = true;
			break;
		}
	}

	channels = params_channels(params);

	/* Configure TDM with more slots for quad-speaker setup */
	if (is_aw88261) {
		slots = 8; /* 8 slots for quad-speaker setup */
		slot_width = 32;
	} else {
		slots = 8;
		slot_width = 32;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Special case for AW88261 (Xiaomi Pad 6) */
		if (is_aw88261 && cpu_dai->id == TERTIARY_TDM_RX_0) {
			dev_info(
				card->dev,
				"Applying AW88261 specific TDM settings (quad-speaker setup)\n");

			/* AW88261 quad-speaker needs all 4 slots active */
			ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0x0F, slots,
						       slot_width);
		} else {
			ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0x03, slots,
						       slot_width);
		}

		if (ret < 0) {
			dev_err(rtd->dev,
				"%s: failed to set tdm slot, err:%d, cpu_dai: %d, slots: %d, slot_width: %d\n",
				__func__, ret, cpu_dai->id, slots, slot_width);
			goto end;
		}

		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL, channels,
						  tdm_slot_offset);
		if (ret < 0) {
			dev_err(rtd->dev,
				"%s: failed to set channel map, err:%d\n",
				__func__, ret);
			goto end;
		}
	} else {
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0xf, 0, slots,
					       slot_width);
		if (ret < 0) {
			dev_err(rtd->dev,
				"%s: failed to set tdm slot2, err:%d, cpu_dai: %d, slots: %d, slot_width: %d\n",
				__func__, ret, cpu_dai->id, slots, slot_width);
			goto end;
		}

		ret = snd_soc_dai_set_channel_map(cpu_dai, channels,
						  tdm_slot_offset, 0, NULL);
		if (ret < 0) {
			dev_err(rtd->dev,
				"%s: failed to set channel map, err:%d\n",
				__func__, ret);
			goto end;
		}
	}

end:
	return ret;
}

static int sm8250_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	printk(PRINTK_LABEL "sm8250_be_hw_params_fixup\n");

	struct snd_interval *rate =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	return 0;
}

static int sm8250_snd_startup(struct snd_pcm_substream *substream)
{
	printk(PRINTK_LABEL "sm8250_snd_startup called\n"); // Dodaj log

	unsigned int fmt = SND_SOC_DAIFMT_BP_FP;
	unsigned int codec_dai_fmt = SND_SOC_DAIFMT_BC_FC;
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	int ret, j;
	bool is_aw88261 = false;

	printk(PRINTK_LABEL "sm8250_snd_startup\n");

	/* Check if we're dealing with an AW88261 codec */
	for_each_rtd_codec_dais(rtd, j, codec_dai) {
		if (strstr(codec_dai->component->name, "aw88261") != NULL) {
			is_aw88261 = true;
			break;
		}
	}

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		ret = snd_soc_dai_set_sysclk(cpu_dai,
					     Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
					     MI2S_BCLK_RATE,
					     SNDRV_PCM_STREAM_PLAYBACK);
		printk(PRINTK_LABEL "snd_soc_dai_set_sysclk returned %d\n",
		       ret);
		if (ret < 0)
			return ret;
		ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
		printk(PRINTK_LABEL "snd_soc_dai_set_fmt returned %d\n", ret);
		if (ret < 0)
			return ret;
		ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		printk(PRINTK_LABEL "snd_soc_dai_set_fmt returned %d\n", ret);
		if (ret < 0)
			return ret;
		break;
	case SECONDARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case TERTIARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		ret = snd_soc_dai_set_sysclk(cpu_dai,
					     Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT,
					     MI2S_BCLK_RATE,
					     SNDRV_PCM_STREAM_PLAYBACK);
		printk(PRINTK_LABEL "snd_soc_dai_set_sysclk returned %d\n",
		       ret);
		if (ret < 0)
			return ret;
		ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
		printk(PRINTK_LABEL "snd_soc_dai_set_fmt returned %d\n", ret);
		if (ret < 0)
			return ret;
		ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		printk(PRINTK_LABEL "snd_soc_dai_set_fmt returned %d\n", ret);
		if (ret < 0)
			return ret;
		break;

	case TERTIARY_TDM_RX_0:
		/* AW88261 specific format and clock setting (Xiaomi Pad 6 fix) */
		if (is_aw88261) {
			/* DSP_A mode needed for AW88261 */
			codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF |
					 SND_SOC_DAIFMT_DSP_A;

			/* AW88261 needs a specific clock rate for stable PLL operation */
			ret = snd_soc_dai_set_sysclk(
				cpu_dai, Q6AFE_LPASS_CLK_ID_TER_TDM_IBIT,
				11289600, SNDRV_PCM_STREAM_PLAYBACK);
			printk(PRINTK_LABEL
			       "snd_soc_dai_set_sysclk returned %d\n",
			       ret);

			if (ret < 0) {
				dev_warn(
					rtd->dev,
					"Setting TDM clock failed: %d, trying standard rate\n",
					ret);
				ret = snd_soc_dai_set_sysclk(
					cpu_dai,
					Q6AFE_LPASS_CLK_ID_TER_TDM_IBIT,
					TDM_BCLK_RATE,
					SNDRV_PCM_STREAM_PLAYBACK);
				printk(PRINTK_LABEL
				       "snd_soc_dai_set_sysclk fallback returned %d\n",
				       ret);
				if (ret < 0) {
					dev_err(rtd->dev,
						"Failed to set any TDM clock rate: %d\n",
						ret);
					return ret;
				}
			} else {
				dev_info(
					rtd->card->dev,
					"Setting AW88261 special clock: 11.2896MHz\n");
			}
		} else {
			codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF |
					 SND_SOC_DAIFMT_DSP_A;
			ret = snd_soc_dai_set_sysclk(
				cpu_dai, Q6AFE_LPASS_CLK_ID_TER_TDM_IBIT,
				TDM_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
			printk(PRINTK_LABEL
			       "snd_soc_dai_set_sysclk returned %d\n",
			       ret);
			if (ret < 0) {
				dev_err(rtd->dev,
					"Failed to set TDM clock rate: %d\n",
					ret);
				return ret;
			}
		}

		/* Set format for all codec DAIs */
		for_each_rtd_codec_dais(rtd, j, codec_dai) {
			ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
			printk(PRINTK_LABEL
			       "snd_soc_dai_set_fmt for codec %s returned %d\n",
			       codec_dai->component->name, ret);
			if (ret < 0) {
				dev_err(rtd->dev,
					"TDM fmt err:%d for codec %s\n", ret,
					codec_dai->component->name);
				continue; /* Continue with other codecs even if one fails */
			}

			if (is_aw88261) {
				/* Try to set the codec clock to match our TDM clock */
				ret = snd_soc_dai_set_sysclk(
					codec_dai, 0, 11289600,
					SNDRV_PCM_STREAM_PLAYBACK);
				printk(PRINTK_LABEL
				       "snd_soc_dai_set_sysclk1 for codec %s returned %d\n",
				       codec_dai->component->name, ret);
				if (ret < 0) {
					dev_dbg(rtd->dev,
						"Setting codec sysclk failed: %d, trying standard rate\n",
						ret);
					snd_soc_dai_set_sysclk(
						codec_dai, 0, TDM_BCLK_RATE,
						SNDRV_PCM_STREAM_PLAYBACK);
					printk(PRINTK_LABEL
					       "snd_soc_dai_set_sysclk fallback for codec %s returned %d\n",
					       codec_dai->component->name, ret);
				}
			} else {
				snd_soc_dai_set_sysclk(
					codec_dai, 0, TDM_BCLK_RATE,
					SNDRV_PCM_STREAM_PLAYBACK);
				printk(PRINTK_LABEL
				       "snd_soc_dai_set_sysclk2 for codec %s returned %d\n",
				       codec_dai->component->name, ret);
			}
		}
		break;
	default:
		break;
	}
	printk(PRINTK_LABEL "starting qcom_snd_sdw_startup...\n");

	ret = qcom_snd_sdw_startup(substream);
	printk(PRINTK_LABEL "qcom_snd_sdw_startup returned %d\n", ret);
	printk(PRINTK_LABEL "sm8250_snd_startup done %d\n", ret);
	return ret;
}

static void sm8250_snd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct sdw_stream_runtime *sruntime = data->sruntime[cpu_dai->id];

	data->sruntime[cpu_dai->id] = NULL;
	sdw_release_stream(sruntime);
}

static int sm8250_snd_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	printk(PRINTK_LABEL "sm8250_snd_hw_params");
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm8250_snd_data *pdata = snd_soc_card_get_drvdata(rtd->card);

	printk(PRINTK_LABEL "sm8250_snd_hw_params cpu_dai->id=%d\n",
	       cpu_dai->id);
	// switch (cpu_dai->id) {
	// case PRIMARY_TDM_RX_0 ... QUINARY_TDM_TX_7:
	return sm8250_tdm_snd_hw_params(substream, params);
	// }

	// return qcom_snd_sdw_hw_params(substream, params,
	//   &pdata->sruntime[cpu_dai->id]);
}

static int sm8250_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct sdw_stream_runtime *sruntime = data->sruntime[cpu_dai->id];

	printk(PRINTK_LABEL "sm8250_snd_prepare\n");

	return qcom_snd_sdw_prepare(substream, sruntime,
				    &data->stream_prepared[cpu_dai->id]);
}

static int sm8250_snd_hw_free(struct snd_pcm_substream *substream)
{
	printk(PRINTK_LABEL "sm8250_snd_hw_free\n");

	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sdw_stream_runtime *sruntime = data->sruntime[cpu_dai->id];

	return qcom_snd_sdw_hw_free(substream, sruntime,
				    &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops sm8250_be_ops = {
	.startup = sm8250_snd_startup,
	.shutdown = sm8250_snd_shutdown,
	.hw_params = sm8250_snd_hw_params,
	.hw_free = sm8250_snd_hw_free,
	.prepare = sm8250_snd_prepare,
};

SND_SOC_DAILINK_DEF(platform_dailink,
		    DAILINK_COMP_ARRAY(COMP_PLATFORM("q6routing")));

SND_SOC_DAILINK_DEF(cpu_dailink, DAILINK_COMP_ARRAY(COMP_CPU("MultiMedia1")));

SND_SOC_DAILINK_DEF(
	codec_link,
	DAILINK_COMP_ARRAY(COMP_CODEC("aw88261.3-0034",
				      "aw88261-aif"), // aw88261_pl @0x34
			   COMP_CODEC("aw88261.3-0035",
				      "aw88261-aif"), // aw88261_sr @0x35
			   COMP_CODEC("aw88261.1-0036",
				      "aw88261-aif"), // aw88261_pr @0x36
			   COMP_CODEC("aw88261.1-0037",
				      "aw88261-aif") // aw88261_sl @0x37
			   ));

static struct snd_soc_dai_link sm8250_dai_links[] = {
	[0] = {
        .name = "Tertiary TDM Playback",
        .cpus = & (struct snd_soc_dai_link_component) {
            .dai_name = "TERTIARY_TDM_RX_0",
        },
        .codecs = & (struct snd_soc_dai_link_component) {
            .dai_name = "aw88261-aif",
            .of_node = NULL, // Dodaj, jeśli potrzebne
        },
        .platforms = & (struct snd_soc_dai_link_component) {
            .dai_name = "q6routing",
        },
        .ignore_pmdown_time = 1,
        .ops = &sm8250_be_ops,
        .be_hw_params_fixup = sm8250_be_hw_params_fixup,
        .init = sm8250_snd_init,
    },
    [1] = {
        .name = "MultiMedia1",
        .cpus = & (struct snd_soc_dai_link_component) {
            .dai_name = "MultiMedia1",
        },
        .codecs = NULL, // Usuń codec
        .platforms = NULL,
        .ignore_pmdown_time = 1,
        .ops = &sm8250_be_ops,
        .be_hw_params_fixup = sm8250_be_hw_params_fixup,
        .init = sm8250_snd_init,
    },
    
};

static void sm8250_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		// if (link->no_pcm == 0) {
		// printk("sm8250_soc: "
		//        "Konfiguruję BE link: %s\n",
		//        link->name);
		// printk("sm8250_soc: "
		//        "CPU DAI: %s\n",
		//        link->cpus->dai_name);
		// printk("sm8250_soc: "
		//        "Codec DAI: %s\n",
		//        link->codecs->dai_name);
		link->be_hw_params_fixup = sm8250_be_hw_params_fixup;
		link->ops = &sm8250_be_ops;
		// }
		link->init = sm8250_snd_init;
		printk(PRINTK_LABEL "sm8250_add_be_ops link %s\n", link->name);
	}
}

static int sm8250_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct sm8250_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	printk(PRINTK_LABEL "snd8250_platform_probe entered\n");

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->owner = THIS_MODULE;
	// no dapm_widgets
	// no controls
	card->driver_name = DRIVER_NAME;
	card->dai_link = sm8250_dai_links; // <--- NASZA TABLICA LINKÓW
	card->num_links = ARRAY_SIZE(sm8250_dai_links);
	dev_info(dev, "Number of DAI links: %d\n",
		 card->num_links); // Dodaj log

	// printk(PRINTK_LABEL "snd8250_platform_probe dai link done\n");

	// card->dapm_widgets = sc7280_snd_widgets;
	// card->num_dapm_widgets = ARRAY_SIZE(sc7280_snd_widgets);
	// card->controls = sc7280_snd_controls;
	// card->num_controls = ARRAY_SIZE(sc7280_snd_controls);

	card->dev = dev;
	dev_set_drvdata(dev, card);
	ret = qcom_snd_parse_of(card);
	printk(PRINTK_LABEL "snd8250_platform_probe parse of result %d\n", ret);
	if (ret)
		return ret;

	snd_soc_card_set_drvdata(card, data);

	sm8250_add_be_ops(card);
	ret = devm_snd_soc_register_card(dev, card);
	printk(PRINTK_LABEL
	       "snd8250_platform_probe card registered with result %d\n",
	       ret);

	// ret = devm_snd_soc_register_component(&pdev->dev, &sm8250_soc_component,
	// 				      NULL, 0);
	// if (ret < 0) {
	// 	dev_err(&pdev->dev, "Failed to register soc component: %d\n",
	// 		ret);
	return ret;
}

static const struct of_device_id snd_sm8250_dt_match[] = {
	{ .compatible = "qcom,sm8250-sndcard" },
	{ .compatible = "qcom,qrb4210-rb2-sndcard" },
	{ .compatible = "qcom,qrb5165-rb5-sndcard" },
	{}
};

MODULE_DEVICE_TABLE(of, snd_sm8250_dt_match);

static struct platform_driver snd_sm8250_driver = {
	.probe  = sm8250_platform_probe,
	.driver = {
		.name = "snd-sm8250",
		.of_match_table = snd_sm8250_dt_match,
	},
};
module_platform_driver(snd_sm8250_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("SM8250 ASoC Machine Driver");
MODULE_LICENSE("GPL");
