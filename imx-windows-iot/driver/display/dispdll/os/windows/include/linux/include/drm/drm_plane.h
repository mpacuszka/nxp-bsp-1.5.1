/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright 2022 NXP
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#ifndef __DRM_PLANE_H__
#define __DRM_PLANE_H__

#include <drm/drm_color_mgmt.h>
#include <drm/drm_rect.h>

/**
 * struct drm_plane_state - mutable plane state
 *
 * Please not that the destination coordinates @crtc_x, @crtc_y, @crtc_h and
 * @crtc_w and the source coordinates @src_x, @src_y, @src_h and @src_w are the
 * raw coordinates provided by userspace. Drivers should use
 * drm_atomic_helper_check_plane_state() and only use the derived rectangles in
 * @src and @dst to program the hardware.
 */
struct drm_plane_state {
	/** @plane: backpointer to the plane */
	struct drm_plane *plane;

	/**
	 * @crtc:
	 *
	 * Currently bound CRTC, NULL if disabled. Do not this write directly,
	 * use drm_atomic_set_crtc_for_plane()
	 */
	struct drm_crtc *crtc;

	/**
	 * @fb:
	 *
	 * Currently bound framebuffer. Do not write this directly, use
	 * drm_atomic_set_fb_for_plane()
	 */
	struct drm_framebuffer *fb;

	/**
	 * @crtc_x:
	 *
	 * Left position of visible portion of plane on crtc, signed dest
	 * location allows it to be partially off screen.
	 */

	int32_t crtc_x;
	/**
	 * @crtc_y:
	 *
	 * Upper position of visible portion of plane on crtc, signed dest
	 * location allows it to be partially off screen.
	 */
	int32_t crtc_y;

	/** @crtc_w: width of visible portion of plane on crtc */
	/** @crtc_h: height of visible portion of plane on crtc */
	uint32_t crtc_w, crtc_h;

	/**
	 * @src_x: left position of visible portion of plane within plane (in
	 * 16.16 fixed point).
	 */
	uint32_t src_x;
	/**
	 * @src_y: upper position of visible portion of plane within plane (in
	 * 16.16 fixed point).
	 */
	uint32_t src_y;
	/** @src_w: width of visible portion of plane (in 16.16) */
	/** @src_h: height of visible portion of plane (in 16.16) */
	uint32_t src_h, src_w;

	/**
	 * @alpha:
	 * Opacity of the plane with 0 as completely transparent and 0xffff as
	 * completely opaque. See drm_plane_create_alpha_property() for more
	 * details.
	 */
	u16 alpha;

	/**
	 * @pixel_blend_mode:
	 * The alpha blending equation selection, describing how the pixels from
	 * the current plane are composited with the background. Value can be
	 * one of DRM_MODE_BLEND_*
	 */
	uint16_t pixel_blend_mode;

	/**
	 * @rotation:
	 * Rotation of the plane. See drm_plane_create_rotation_property() for
	 * more details.
	 */
	unsigned int rotation;

	/**
	 * @zpos:
	 * Priority of the given plane on crtc (optional).
	 *
	 * User-space may set mutable zpos properties so that multiple active
	 * planes on the same CRTC have identical zpos values. This is a
	 * user-space bug, but drivers can solve the conflict by comparing the
	 * plane object IDs; the plane with a higher ID is stacked on top of a
	 * plane with a lower ID.
	 *
	 * See drm_plane_create_zpos_property() and
	 * drm_plane_create_zpos_immutable_property() for more details.
	 */
	unsigned int zpos;

	/**
	 * @normalized_zpos:
	 * Normalized value of zpos: unique, range from 0 to N-1 where N is the
	 * number of active planes for given crtc. Note that the driver must set
	 * &drm_mode_config.normalize_zpos or call drm_atomic_normalize_zpos() to
	 * update this before it can be trusted.
	 */
	unsigned int normalized_zpos;

	/**
	 * @color_encoding:
	 *
	 * Color encoding for non RGB formats
	 */
	enum drm_color_encoding color_encoding;

	/**
	 * @color_range:
	 *
	 * Color range for non RGB formats
	 */
	enum drm_color_range color_range;

	/**
	 * @src:
	 *
	 * source coordinates of the plane (in 16.16).
	 *
	 * When using drm_atomic_helper_check_plane_state(),
	 * the coordinates are clipped, but the driver may choose
	 * to use unclipped coordinates instead when the hardware
	 * performs the clipping automatically.
	 */
	/**
	 * @dst:
	 *
	 * clipped destination coordinates of the plane.
	 *
	 * When using drm_atomic_helper_check_plane_state(),
	 * the coordinates are clipped, but the driver may choose
	 * to use unclipped coordinates instead when the hardware
	 * performs the clipping automatically.
	 */
	struct drm_rect src, dst;

	/**
	 * @visible:
	 *
	 * Visibility of the plane. This can be false even if fb!=NULL and
	 * crtc!=NULL, due to clipping.
	 */
	bool visible;
};

enum drm_plane_type {
	DRM_PLANE_TYPE_OVERLAY,
	DRM_PLANE_TYPE_PRIMARY,
	DRM_PLANE_TYPE_CURSOR,
};

/**
 * struct drm_plane - central DRM plane control structure
 *
 * Planes represent the scanout hardware of a display block. They receive their
 * input data from a &drm_framebuffer and feed it to a &drm_crtc. Planes control
 * the color conversion, see `Plane Composition Properties`_ for more details,
 * and are also involved in the color conversion of input pixels, see `Color
 * Management Properties`_ for details on that.
 */
struct drm_plane {
	/** @type: Type of plane, see &enum drm_plane_type for details. */
	enum drm_plane_type type;

	/**
	 * @index: Position inside the mode_config.list, can be used as an array
	 * index. It is invariant over the lifetime of the plane.
	 */
	unsigned index;

	/**
	 * @state:
	 *
	 * Current atomic state for this plane.
	 *
	 * This is protected by @mutex. Note that nonblocking atomic commits
	 * access the current plane state without taking locks. Either by going
	 * through the &struct drm_atomic_state pointers, see
	 * for_each_oldnew_plane_in_state(), for_each_old_plane_in_state() and
	 * for_each_new_plane_in_state(). Or through careful ordering of atomic
	 * commit operations as implemented in the atomic helpers, see
	 * &struct drm_crtc_commit.
	 */
	struct drm_plane_state *state;
};

/**
 * drm_plane_index - find the index of a registered plane
 * @plane: plane to find index for
 *
 * Given a registered plane, return the index of that plane within a DRM
 * device's list of planes.
 */
static inline unsigned int drm_plane_index(const struct drm_plane* plane)
{
	return plane->index;
}

/**
 * drm_plane_mask - find the mask of a registered plane
 * @plane: plane to find mask for
 */
static inline u32 drm_plane_mask(const struct drm_plane* plane)
{
	return 1 << drm_plane_index(plane);
}

#endif
