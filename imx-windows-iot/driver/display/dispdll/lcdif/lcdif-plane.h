/*
 * Copyright 2018,2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LCDIF_PLANE_H
#define __LCDIF_PLANE_H

#include <drm/drm_plane.h>
#include "imx-lcdif.h"

struct lcdif_plane {
	enum drm_plane_type type;
	struct lcdif_soc *lcdif;
};

struct lcdif_plane_state {
	bool mode_change; /* true = all members are valid, false = fb_addr valid only */
	u32 format;       /* drm_fourcc pixel format */
	u32 fb_addr;
	u32 pitch;        /* stride size in bytes */
	/* scr_w, src_h size in pixels */
	u32 src_w;
	u32 src_h;
};

#define CRTC_PLANE_INDEX_PRIMARY 0
#define CRTC_PLANE_INDEX_OVERLAY 1

struct lcdif_plane *lcdif_plane_init(
				     struct lcdif_soc *lcdif,
				     unsigned int possible_crtcs,
				     enum drm_plane_type type,
				     unsigned int zpos);

void lcdif_plane_deinit(struct lcdif_plane *lcdif_plane);
void lcdif_plane_atomic_update(struct platform_device *pdev,
    u32 plain_index, struct lcdif_plane_state *plane_state);

#endif
