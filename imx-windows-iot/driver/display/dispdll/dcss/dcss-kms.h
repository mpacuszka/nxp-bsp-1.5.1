/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019, 2022 NXP.
 */

#ifndef _DCSS_KMS_H_
#define _DCSS_KMS_H_

#include <linux/kernel.h>

#include <drm/drm_atomic.h>
#include <drm/drm_crtc.h>
//#include <drm/drm_device.h>
//#include <drm/drm_encoder.h>

#include "dcss-dev.h"

struct dcss_plane {
	struct drm_plane base;
	struct dcss_dev *dcss;

	uint64_t dtrc_table_ofs_val;
	struct drm_property *dtrc_table_ofs_prop;

	int ch_num;

	enum drm_plane_type type;
	struct drm_plane_state state;
	struct drm_framebuffer fb;
	bool use_dtrc;
};

struct dcss_crtc {
	struct drm_crtc		base;
	struct dcss_plane	*plane[3];
	struct dcss_dev *dcss;
	int			irq;
	bool			disable_ctxld_kick_irq;
};

struct dcss_crtc_state {
	struct drm_crtc_state		base;
	enum dcss_pixel_pipe_output	output_encoding;
	enum dcss_hdr10_nonlinearity	opipe_nl;
	enum dcss_hdr10_gamut		opipe_g;
	enum dcss_hdr10_pixel_range	opipe_pr;
};

struct dcss_kms_dev {
	//struct drm_device base;
	struct dcss_crtc crtc;
	//struct drm_encoder encoder;
	//struct drm_connector *connector;
};

static inline struct dcss_crtc *to_dcss_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct dcss_crtc, base);
}

static inline struct dcss_crtc_state *
to_dcss_crtc_state(struct drm_crtc_state *state)
{
	return container_of(state, struct dcss_crtc_state, base);
}

/* KMS functions */
void dcss_kms_atomic_check(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *old_mode,
	struct drm_connector_state *conn_state);

/* CRTC functions */
int dcss_crtc_copy_display_mode(struct drm_display_mode *src_mode,
	struct drm_display_mode *dst_mode);
int dcss_enable_vblank(struct dcss_crtc *crtc);
void dcss_disable_vblank(struct dcss_crtc *crtc);
void dcss_crtc_atomic_flush(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state);
void dcss_crtc_atomic_enable(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *old_mode);
void dcss_crtc_atomic_disable(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *old_mode);
enum drm_mode_status dcss_crtc_mode_valid(struct drm_crtc *crtc,
	const struct drm_display_mode *mode);
void __dcss_crtc_setup_opipe(struct dcss_crtc *crtc,
	struct dcss_crtc_state *dcss_crtc_state, struct drm_display_mode *mode,
	struct drm_connector_state *conn_state);
int dcss_crtc_init(struct dcss_crtc *crtc, struct dcss_dev *dcss);
bool dcss_crtc_in_use(struct dcss_dev *dcss);

struct dcss_dev *dcss_drv_dev_to_dcss(struct device *dev);

/* Plane functions */
void dcss_plane_copy_state(struct drm_plane_state *state,
	struct drm_plane_state *old_state);
int dcss_plane_atomic_check(struct drm_plane *plane,
	struct drm_plane_state *state);
void dcss_plane_atomic_update(struct drm_plane *plane,
	struct drm_plane_state *old_state);
void dcss_plane_atomic_disable(struct drm_plane *plane,
	struct drm_plane_state *old_state);
struct dcss_plane *dcss_plane_init(struct dcss_dev *dcss,
	enum drm_plane_type type,
	unsigned int zpos);
void dcss_plane_atomic_page_flip(struct drm_plane *plane,
	struct drm_plane_state *old_state);
void dcss_plane_atomic_page_flip_no_ctxld(struct drm_plane *plane,
	unsigned int base_addr);

#endif
