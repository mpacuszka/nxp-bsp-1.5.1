// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019, 2022 NXP.
 */

#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_atomic.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

void dcss_kms_atomic_check(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *old_mode,
	struct drm_connector_state *conn_state)
{
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = dcss_crtc->dcss;
	struct dcss_crtc_state *dcss_crtc_state =
		to_dcss_crtc_state(dcss_crtc->base.state);

	if (!drm_mode_equal(mode, old_mode)) {
		crtc->state->mode_changed = true;
	} else {
		crtc->state->mode_changed = false;
	}

	__dcss_crtc_setup_opipe(dcss_crtc, dcss_crtc_state, mode, conn_state);
}
