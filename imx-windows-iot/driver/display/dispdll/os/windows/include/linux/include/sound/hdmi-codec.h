/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * hdmi-codec.h - HDMI Codec driver API
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - https://www.ti.com
 * Copyright 2023 NXP
 *
 * Author: Jyri Sarha <jsarha@ti.com>
 */

#ifndef __HDMI_CODEC_H__
#define __HDMI_CODEC_H__

#include <linux/hdmi.h>
#include <drm/drm_edid.h>
#include <uapi/sound/asound.h>

/*
 * HDMI audio parameters
 */
struct hdmi_codec_params {
	struct hdmi_audio_infoframe cea;
	struct snd_aes_iec958 iec;
	int sample_rate;
	int sample_width;
	int channels;
};

#endif /* __HDMI_CODEC_H__ */
