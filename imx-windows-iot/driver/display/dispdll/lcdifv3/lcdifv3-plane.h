/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019,2020,2022 NXP
 */

#ifndef __LCDIFV3_PLANE_H
#define __LCDIFV3_PLANE_H

#include <drm/drm_plane.h>
#include "lcdifv3/imx-lcdifv3.h"

struct lcdifv3_plane {
	enum drm_plane_type type;
	struct lcdifv3_soc *lcdifv3;
};

#define CRTC_PLANE_INDEX_PRIMARY 0
#define CRTC_PLANE_INDEX_OVERLAY 1

struct lcdifv3_plane_state {
	bool mode_change; /* true = all members are valid, false = fb_addr valid only */
	u32 format;       /* drm_fourcc pixel format */
	u32 fb_addr;
	u32 pitch;        /* stride size in bytes */
	/* scr_w, src_h size in pixels */
	u32 src_w;
	u32 src_h;
};

struct lcdifv3_plane *lcdifv3_plane_init(
				     struct lcdifv3_soc *lcdifv3,
				     unsigned int possible_crtcs,
				     enum drm_plane_type type,
				     unsigned int zpos);
void lcdifv3_plane_destroy(struct lcdifv3_plane *lcdifv3_plane);
void lcdifv3_plane_atomic_update(struct platform_device *pdev, u32 plain_index, struct lcdifv3_plane_state *plane_state);
#endif
