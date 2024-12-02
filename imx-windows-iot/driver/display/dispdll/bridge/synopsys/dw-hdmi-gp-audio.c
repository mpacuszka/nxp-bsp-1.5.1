// SPDX-License-Identifier: GPL-2.0-only
// Copyright 2020-2023 NXP
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/platform_device.h>
#include "dw_hdmi.h"
#include <drm/drm_edid.h>
#include <drm/drm_connector.h>

#include <sound/hdmi-codec.h>
#include <sound/asoundef.h>
#include "imx8mpHdmiDevintPar.h"

#define printk(x, ...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

/* #define MP_AUDIO_DEBUG */
#ifdef MP_AUDIO_DEBUG
#define printk_debug printk
#else
#define printk_debug
#endif

struct dw_hdmi;

struct dw_hdmi_channel_conf {
	u8 conf1;
	u8 ca;
};

static struct dw_hdmi_channel_conf default_hdmi_channel_config[7] = {
	{ 0x03, 0x00 },	/* FL,FR */
	{ 0x0b, 0x02 },	/* FL,FR,FC */
	{ 0x33, 0x08 },	/* FL,FR,RL,RR */
	{ 0x37, 0x09 },	/* FL,FR,LFE,RL,RR */
	{ 0x3f, 0x0b },	/* FL,FR,LFE,FC,RL,RR */
	{ 0x7f, 0x0f },	/* FL,FR,LFE,FC,RL,RR,RC */
	{ 0xff, 0x13 },	/* FL,FR,LFE,FC,RL,RR,[FR]RC,[FR]LC */
};

/*pdev is the same as for main dw-hdmi driver*/
int audio_hw_params(struct platform_device *pdev,
			struct imx8mp_hdmi_audio_params *params)
{
	struct dw_hdmi *hdmi = (struct dw_hdmi *)pdev->dev.platform_data;
	int ret = 0;
	u8 ca;

	printk_debug("audio_hw_params: sample_rate=%d sample_width=%d channel=%d.\n", params->sample_rate, params->sample_width, params->channels);
	dw_hdmi_set_sample_rate(hdmi, params->sample_rate);

	ca = default_hdmi_channel_config[params->channels - 2].ca;

	dw_hdmi_set_channel_count(hdmi, params->channels);
	dw_hdmi_set_channel_allocation(hdmi, ca);

	dw_hdmi_set_sample_non_pcm(hdmi,
				   params->iec958_aes0_nonaudio);
	dw_hdmi_set_sample_width(hdmi, params->sample_width);

	return ret;
}

/*pdev is the same as for main dw-hdmi driver*/
int audio_mute_stream(struct platform_device *pdev, bool enable)
{
	struct dw_hdmi *hdmi = (struct dw_hdmi *)pdev->dev.platform_data;
	int ret = 0;

	printk_debug("audio_mute_stream: mute=%s\n", enable?"TRUE": "FALSE");
	if (!enable)
		dw_hdmi_audio_enable(hdmi);
	else
		dw_hdmi_audio_disable(hdmi);

	return ret;
}

/*pdev is the same as for main dw-hdmi driver*/
bool audio_is_supported(struct platform_device *pdev)
{
	struct dw_hdmi *hdmi = (struct dw_hdmi*)pdev->dev.platform_data;

	return dw_hdmi_audio_supported(hdmi);
}
