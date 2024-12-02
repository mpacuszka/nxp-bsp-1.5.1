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

#include <linux/platform_device.h>
#include <linux/err.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_atomic.h>
#include <drm/drm_plane.h>
#include "imx-lcdif.h"

#include "lcdif-plane.h"

void lcdif_plane_atomic_update(struct platform_device *pdev,
	u32 plain_index, struct lcdif_plane_state *plane_state)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(pdev);
	struct lcdif_plane *lcdif_plane = lcdif_crtc->plane[plain_index];
	struct lcdif_soc *lcdif = lcdif_plane->lcdif;
	u32 cpp, stride;
	bool crop;

	if (!plane_state || !plane_state->fb_addr)
		return;

	/* TODO: for now we just update the next buf addr
	 * and the fb pixel format, since the mode set will
	 * be done in crtc's ->enable() helper func
	 */
	switch (lcdif_plane->type) {
	case DRM_PLANE_TYPE_PRIMARY:
		break;
	default:
		/* TODO: add overlay later */
		return;
	}

	lcdif_set_fb_addr(lcdif, plain_index, plane_state->fb_addr);

	/* Config pixel format and horizontal cropping
	 * if CRTC needs a full modeset which needs to
	 * enable LCDIF to run at the end.
	 */
	if (unlikely(plane_state->mode_change)) {
		if (lcdif_plane->type == DRM_PLANE_TYPE_PRIMARY)
			lcdif_set_pix_fmt(lcdif, plane_state->format);

		cpp = lcdif_get_bpp_from_fmt(plane_state->format) / 8;
		stride = DIV_ROUND_UP(plane_state->pitch, cpp);

		crop  = plane_state->src_w != stride ? true : false;
		lcdif_set_fb_hcrop(lcdif, plane_state->src_w, stride, crop);

		lcdif_enable_controller(lcdif);
	}
}

static void lcdif_plane_destroy(struct lcdif_plane *lcdif_plane)
{
	if (lcdif_plane) {
		kfree(lcdif_plane);
	}
}

struct lcdif_plane *lcdif_plane_init(
				     struct lcdif_soc *lcdif,
				     unsigned int possible_crtcs,
				     enum drm_plane_type type,
				     unsigned int zpos)
{
	struct lcdif_plane *lcdif_plane;

	/* lcdif doesn't support fb modifiers */
	if (zpos)
		return ERR_PTR(-EINVAL);

	lcdif_plane = kzalloc(sizeof(*lcdif_plane), GFP_KERNEL);
	if (!lcdif_plane)
		return ERR_PTR(-ENOMEM);

	lcdif_plane->lcdif = lcdif;
	lcdif_plane->type = type;

	return lcdif_plane;
}

void lcdif_plane_deinit(
			struct lcdif_plane *lcdif_plane)
{
	lcdif_plane_destroy(lcdif_plane);
}
