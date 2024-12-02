// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019, 2022 NXP.
 */

#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_connector.h>

#include <linux/hdmi.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

int dcss_crtc_copy_display_mode(struct drm_display_mode *src_mode,
	struct drm_display_mode *dst_mode)
{

	if (!src_mode || !dst_mode)
		return -1;

	dst_mode->hdisplay = src_mode->hdisplay;
	dst_mode->hsync_start = src_mode->hsync_start;
	dst_mode->hsync_end = src_mode->hsync_end;
	dst_mode->htotal = src_mode->htotal;
	dst_mode->hskew = src_mode->hskew;
	dst_mode->vdisplay = src_mode->vdisplay;
	dst_mode->vsync_start = src_mode->vsync_start;
	dst_mode->vsync_end = src_mode->vsync_end;
	dst_mode->vtotal = src_mode->vtotal;
	dst_mode->vscan = src_mode->vscan;

	dst_mode->clock = src_mode->clock;

	dst_mode->flags = src_mode->flags;

	dst_mode->picture_aspect_ratio = src_mode->picture_aspect_ratio;

	return 0;
}

int dcss_enable_vblank(struct dcss_crtc *crtc)
{
	struct dcss_dev *dcss = crtc->dcss;

	dcss_dtg_vblank_irq_enable(dcss->dtg, true);

	dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, true);

	enable_irq(crtc->irq);

	return 0;
}

void dcss_disable_vblank(struct dcss_crtc *crtc)
{
	struct dcss_dev *dcss = crtc->dcss;

	disable_irq(crtc->irq);

	dcss_dtg_vblank_irq_enable(dcss->dtg, false);

	if (!dcss_dtrc_is_running(dcss->dtrc) &&
		crtc->disable_ctxld_kick_irq)
		dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, false);
}

void dcss_crtc_atomic_flush(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state)
{
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = dcss_crtc->dcss;

	if (dcss_dtg_is_enabled(dcss->dtg))
		dcss_ctxld_enable(dcss->ctxld);
}

void dcss_crtc_atomic_enable(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *old_mode)
{
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = dcss_crtc->dcss;
	struct dcss_crtc_state *dcss_crtc_state =
		to_dcss_crtc_state(dcss_crtc->base.state);
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	dcss_ss_subsam_set(dcss->ss, dcss_crtc_state->output_encoding);
	dcss_dtg_css_set(dcss->dtg, dcss_crtc_state->output_encoding);

	dcss_dtg_sync_set(dcss->dtg, &vm);
	dcss_ss_sync_set(dcss->ss, &vm,
		mode->flags & DRM_MODE_FLAG_PHSYNC,
		mode->flags & DRM_MODE_FLAG_PVSYNC);

	dcss_enable_dtg_and_ss(dcss);

	dcss_ctxld_enable(dcss->ctxld);

	/* Allow CTXLD kick interrupt to be disabled when VBLANK is disabled. */
	dcss_crtc->disable_ctxld_kick_irq = true;
}

static void dcss_disable_planes_on_crtc(struct drm_crtc *crtc)
{
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);

	for (int i = 0; i < 3; i++) {
		if (dcss_crtc->plane[i]->base.state) {
			dcss_plane_atomic_disable(&dcss_crtc->plane[i]->base,
				dcss_crtc->plane[i]->base.state);
		}
	}
}

void dcss_crtc_atomic_disable(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *old_mode)
{
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = dcss_crtc->dcss;

	dcss_disable_planes_on_crtc(crtc);

	dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, true);

	reinit_completion(&dcss->disable_completion);

	dcss_disable_dtg_and_ss(dcss);

	dcss_ctxld_enable(dcss->ctxld);

	if (!wait_for_completion_timeout(&dcss->disable_completion,
		msecs_to_jiffies(100)))
		dev_err(dcss->dev, "Shutting off DTG timed out.\n");

	/*
	 * Do not shut off CTXLD kick interrupt when shutting VBLANK off. It
	 * will be needed to commit the last changes, before going to suspend.
	 */
	dcss_crtc->disable_ctxld_kick_irq = false;
}

enum drm_mode_status dcss_crtc_mode_valid(struct drm_crtc *crtc,
	const struct drm_display_mode *mode)
{
	/*
	 * From DCSS perspective, dissallow any mode higher than
	 * 3840x2160 or 2160x3840.
	 * Current limitation - max supported mode is 1080p.
	 */
	if (mode->hdisplay * mode->vdisplay > 1920 * 1080)
		return MODE_BAD;

	return MODE_OK;
}

static void __dcss_crtc_setup_opipe_gamut(u32 colorspace,
	const struct drm_display_mode *mode,
	enum dcss_hdr10_gamut *g,
	enum dcss_hdr10_nonlinearity *nl)
{
	u8 vic;

	switch (colorspace) {
	case DRM_MODE_COLORIMETRY_BT709_YCC:
	case DRM_MODE_COLORIMETRY_XVYCC_709:
		*g = G_REC709;
		*nl = NL_REC709;
		return;
	case DRM_MODE_COLORIMETRY_SMPTE_170M_YCC:
	case DRM_MODE_COLORIMETRY_XVYCC_601:
	case DRM_MODE_COLORIMETRY_SYCC_601:
	case DRM_MODE_COLORIMETRY_OPYCC_601:
		*g = G_REC601_NTSC;
		*nl = NL_REC709;
		return;
	case DRM_MODE_COLORIMETRY_BT2020_CYCC:
	case DRM_MODE_COLORIMETRY_BT2020_RGB:
	case DRM_MODE_COLORIMETRY_BT2020_YCC:
		*g = G_REC2020;
		*nl = NL_REC2084;
		return;
	case DRM_MODE_COLORIMETRY_OPRGB:
		*g = G_REC709;
		*nl = NL_SRGB;
		return;
	default:
		break;
	}

	/*
	 * If we reached this point, it means the default colorimetry is used.
	 */

	 /* non-CEA mode, sRGB is used */
	vic = drm_match_cea_mode(mode);
	if (vic == 0) {
		*g = G_REC709;
		*nl = NL_SRGB;
		return;
	}

	/* use REC709 otherwise, by default */
	*g = G_REC709;
	*nl = NL_REC709;
}

void __dcss_crtc_setup_opipe(struct dcss_crtc *crtc,
	struct dcss_crtc_state *dcss_crtc_state, struct drm_display_mode *mode,
	struct drm_connector_state *conn_state)
{
	struct dcss_dev *dcss = crtc->dcss;
	enum hdmi_quantization_range qr;

	qr = drm_default_rgb_quant_range(mode);

	__dcss_crtc_setup_opipe_gamut(conn_state->colorspace,
		mode,
		&dcss_crtc_state->opipe_g,
		&dcss_crtc_state->opipe_nl);

	dcss_crtc_state->opipe_pr = qr == HDMI_QUANTIZATION_RANGE_FULL ?
		PR_FULL : PR_LIMITED;

	dcss_crtc_state->output_encoding = DCSS_PIPE_OUTPUT_RGB;

	if (dcss->hdmi_output) {
		struct cdns_mhdp_device *mhdp_dev =
			container_of(conn_state->connector,
				struct cdns_mhdp_device,
				connector.base);

		switch (mhdp_dev->video_info.color_fmt) {
		case YCBCR_4_2_2:
			dcss_crtc_state->output_encoding =
				DCSS_PIPE_OUTPUT_YUV422;
			break;
		case YCBCR_4_2_0:
			dcss_crtc_state->output_encoding =
				DCSS_PIPE_OUTPUT_YUV420;
			break;
		case YCBCR_4_4_4:
			dcss_crtc_state->output_encoding =
				DCSS_PIPE_OUTPUT_YUV444;
			break;
		default:
			break;
		}
	}
}

int dcss_crtc_init(struct dcss_crtc *crtc, struct dcss_dev *dcss)
{
	crtc->dcss = dcss;
	struct platform_device *pdev = to_platform_device(dcss->dev);

	crtc->plane[0] = dcss_plane_init(dcss,
		DRM_PLANE_TYPE_PRIMARY, 2);
	if (IS_ERR(crtc->plane[0]))
		return PTR_ERR(crtc->plane[0]);

	crtc->plane[1] = dcss_plane_init(dcss,
		DRM_PLANE_TYPE_OVERLAY, 1);
	if (IS_ERR(crtc->plane[1]))
		crtc->plane[1] = NULL;

	crtc->plane[2] = dcss_plane_init(dcss,
		DRM_PLANE_TYPE_OVERLAY, 0);
	if (IS_ERR(crtc->plane[2]))
		crtc->plane[2] = NULL;

	crtc->irq = platform_get_irq_byname(pdev, "vblank");
	if (crtc->irq < 0)
		return crtc->irq;

	return 0;
}

bool dcss_crtc_in_use(struct dcss_dev *dcss)
{
	return dcss_dtg_is_enabled(dcss->dtg);
}
