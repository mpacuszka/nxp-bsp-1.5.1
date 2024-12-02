// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019,2020,2022 NXP
 */

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>

#include "lcdifv3/lcdifv3-plane.h"

struct lcdifv3_crtc;

void lcdifv3_plane_atomic_update(struct platform_device *pdev,
	u32 plain_index, struct lcdifv3_plane_state *plane_state)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(pdev);
	struct lcdifv3_plane *lcdifv3_plane = lcdifv3_crtc->plane[plain_index];
	struct lcdifv3_soc *lcdifv3 = lcdifv3_plane->lcdifv3;

	u32 cpp, stride;
	bool crop;

	if (!plane_state || !plane_state->fb_addr)
		return;

	/* TODO: for now we just update the next buf addr
	 * and the fb pixel format, since the mode set will
	 * be done in crtc's ->enable() helper func
	 */
	if ((lcdifv3_plane->type == DRM_PLANE_TYPE_PRIMARY) && plane_state->mode_change)
		lcdifv3_set_pix_fmt(lcdifv3, plane_state->format);

		switch (lcdifv3_plane->type) {
	case DRM_PLANE_TYPE_PRIMARY:
		break;
	default:
		/* TODO: add overlay later */
		return;
	}

	lcdifv3_set_fb_addr(lcdifv3, plain_index, plane_state->fb_addr);

	if (unlikely(plane_state->mode_change)) {
		cpp = lcdifv3_get_bpp_from_fmt(plane_state->format) / 8;
		stride = DIV_ROUND_UP(plane_state->pitch, cpp);
		crop = plane_state->src_w != stride ? true : false;
		lcdifv3_set_fb_hcrop(lcdifv3, plane_state->src_w, plane_state->pitch, crop);
	}
}

void lcdifv3_plane_destroy(struct lcdifv3_plane *lcdifv3_plane)
{
	if (lcdifv3_plane) {
		kfree(lcdifv3_plane);
	}
}

struct lcdifv3_plane *lcdifv3_plane_init(
				     struct lcdifv3_soc *lcdifv3,
				     unsigned int possible_crtcs,
				     enum drm_plane_type type,
				     unsigned int zpos)
{
	struct lcdifv3_plane *lcdifv3_plane;

	/* lcdifv3 doesn't support fb modifiers */
	if (zpos )
		return ERR_PTR(-EINVAL);

	lcdifv3_plane = kzalloc(sizeof(*lcdifv3_plane), GFP_KERNEL);
	if (!lcdifv3_plane)
		return ERR_PTR(-ENOMEM);

	lcdifv3_plane->lcdifv3 = lcdifv3;
	lcdifv3_plane->type = type;

	return lcdifv3_plane;
}
