/*
 * Copyright 2017-2019, 2022-2023 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <asm-generic/errno-base.h>
#include <drm/drm_atomic.h>
#include <drm/drm_blend.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_print.h>
#include <linux/err.h>
#include <video/dpu.h>
#include <video/imx8-prefetch.h>
#include "dpu-plane.h"
#include "drm/imx/imx-drm.h"

#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))

static const uint32_t dpu_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565,

	DRM_FORMAT_YUYV,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

static const uint64_t dpu_format_modifiers[] = {
	DRM_FORMAT_MOD_VIVANTE_TILED,
	DRM_FORMAT_MOD_VIVANTE_SUPER_TILED,
	DRM_FORMAT_MOD_AMPHION_TILED,
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static unsigned int dpu_plane_get_default_zpos(enum drm_plane_type type)
{
	if (type == DRM_PLANE_TYPE_PRIMARY)
		return 0;
	else if (type == DRM_PLANE_TYPE_OVERLAY)
		return 1;

	return 0;
}

void dpu_plane_destroy(struct drm_plane *plane)
{
	struct dpu_plane *dpu_plane = to_dpu_plane(plane);

	kfree(dpu_plane);
}

void dpu_plane_reset(struct drm_plane *plane)
{
	struct dpu_plane_state *state;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return;

	plane->state = &state->base;

	plane->state->plane = plane;
	plane->state->rotation = DRM_MODE_ROTATE_0;
	plane->state->alpha = DRM_BLEND_ALPHA_OPAQUE;
	plane->state->pixel_blend_mode = DRM_MODE_BLEND_PREMULTI;
	plane->state->zpos = dpu_plane_get_default_zpos(plane->type);
	plane->state->normalized_zpos = plane->state->zpos;
	plane->state->color_encoding = DRM_COLOR_YCBCR_BT601;
	plane->state->color_range = DRM_COLOR_YCBCR_FULL_RANGE;
}

static bool dpu_drm_plane_format_mod_supported(struct drm_plane *plane,
					       uint32_t format,
					       uint64_t modifier)
{
	if (modifier == DRM_FORMAT_MOD_INVALID) {
		WARN_ON(1);
		return false;
	}

	switch (format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		return modifier == DRM_FORMAT_MOD_LINEAR;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_RGB565:
		return modifier == DRM_FORMAT_MOD_LINEAR ||
		       modifier == DRM_FORMAT_MOD_VIVANTE_TILED ||
		       modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		return modifier == DRM_FORMAT_MOD_LINEAR ||
		       modifier == DRM_FORMAT_MOD_AMPHION_TILED;
	default:
		return false;
	}
}

void dpu_drm_atomic_plane_destroy_state(struct drm_plane *plane,
	struct drm_plane_state *state)
{
	struct dpu_plane_state* dpu_plane_state = to_dpu_plane_state(state);

	if (dpu_plane_state)
		kfree(dpu_plane_state);
}

static inline dma_addr_t
drm_plane_state_to_baseaddr(struct drm_plane_state *state, bool aux_source)
{
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t paddr = fb->paddr[0];
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	unsigned int x = (state->src.x1 >> 16) +
				(aux_source ? dpstate->left_src_w : 0);
	unsigned int y = state->src.y1 >> 16;

	if (fb->modifier)
		return paddr + fb->offsets[0];

	if (fb->flags & DRM_MODE_FB_INTERLACED)
		y /= 2;

	return paddr + fb->offsets[0] + fb->pitches[0] * y +
	       fb->format->cpp[0] * x;
}

static inline dma_addr_t
drm_plane_state_to_uvbaseaddr(struct drm_plane_state *state, bool aux_source)
{
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t paddr = fb->paddr[0];
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	int x = (state->src.x1 >> 16) + (aux_source ? dpstate->left_src_w : 0);
	int y = state->src.y1 >> 16;

	if (fb->modifier)
		return paddr + fb->offsets[1];

	x /= fb->format->hsub;
	y /= fb->format->vsub;

	if (fb->flags & DRM_MODE_FB_INTERLACED)
		y /= 2;

	return paddr + fb->offsets[1] + fb->pitches[1] * y +
	       fb->format->cpp[1] * x;
}

static inline bool dpu_plane_fb_format_is_yuv(u32 fmt)
{
	return fmt == DRM_FORMAT_YUYV || fmt == DRM_FORMAT_UYVY ||
	       fmt == DRM_FORMAT_NV12 || fmt == DRM_FORMAT_NV21;
}

int dpu_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct dpu_plane *dplane = to_dpu_plane(plane);
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb = state->fb;
	struct dpu_fetchunit *fu;
	struct dprc *dprc;
	dma_addr_t baseaddr, uv_baseaddr = 0;
	u32 src_w, src_h, src_x, src_y;
	unsigned int frame_width;
	/* int min_scale, bpp, ret; */
	int bpp;
	bool fb_is_interlaced;
	bool check_aux_source = false;

	/* ok to disable */
	if (!fb) {
		dpstate->stage = LB_PRIM_SEL__DISABLE;
		dpstate->source = LB_SEC_SEL__DISABLE;
		dpstate->blend = ID_NONE;
		dpstate->aux_stage = LB_PRIM_SEL__DISABLE;
		dpstate->aux_source = LB_SEC_SEL__DISABLE;
		dpstate->aux_blend = ID_NONE;
		dpstate->is_top = false;
		dpstate->use_prefetch = false;
		dpstate->use_aux_prefetch = false;
		dpstate->need_aux_source = false;
		dpstate->left_src_w = 0;
		dpstate->left_crtc_w = 0;
		dpstate->left_crtc_x = 0;
		dpstate->right_src_w = 0;
		dpstate->right_crtc_w = 0;
		dpstate->right_crtc_x = 0;
		dpstate->is_left_top = false;
		dpstate->is_right_top = false;
		return 0;
	}

	if (!state->crtc) {
		DRM_DEBUG_KMS("[PLANE:%d] has no CRTC in plane state\n",
				plane->index);
		return -EINVAL;
	}

	src_w = drm_rect_width(&state->src) >> 16;
	src_h = drm_rect_height(&state->src) >> 16;
	src_x = state->src.x1 >> 16;
	src_y = state->src.y1 >> 16;

	fb_is_interlaced = !!(fb->flags & DRM_MODE_FB_INTERLACED);

	if (fb->modifier &&
	    fb->modifier != DRM_FORMAT_MOD_AMPHION_TILED &&
	    fb->modifier != DRM_FORMAT_MOD_VIVANTE_TILED &&
	    fb->modifier != DRM_FORMAT_MOD_VIVANTE_SUPER_TILED) {
		DRM_DEBUG_KMS("[PLANE:%d] unsupported fb modifier\n",
				plane->index);
		return -EINVAL;
	}

	crtc_state = plane->state->crtc->state;

	if (!crtc_state) {
		WARN_ON(1);
		return -EINVAL;
	}

	/* no off screen */
	if (state->dst.x1 < 0 || state->dst.y1 < 0 ||
	    (state->dst.x2 > crtc_state->adjusted_mode.hdisplay) ||
	    (state->dst.y2 > crtc_state->adjusted_mode.vdisplay)) {
		DRM_DEBUG_KMS("[PLANE:%d] no off screen\n", plane->index);
		return -EINVAL;
	}

	/* pixel/line count and position parameters check */
	if (fb->format->hsub == 2) {
		if (dpstate->left_src_w || dpstate->right_src_w) {
			if ((dpstate->left_src_w % 2) ||
			    (dpstate->right_src_w % 2) || (src_x % 2)) {
				DRM_DEBUG_KMS("[PLANE:%d] bad left/right uv width or xoffset\n",
						plane->index);
				return -EINVAL;
			}
		} else {
			if ((src_w % 2) || (src_x % 2)) {
				DRM_DEBUG_KMS("[PLANE:%d] bad uv width or xoffset\n",
						plane->index);
				return -EINVAL;
			}
		}
	}
	if (fb->format->vsub == 2) {
		if (src_h % (fb_is_interlaced ? 4 : 2)) {
			DRM_DEBUG_KMS("[PLANE:%d] bad uv height\n", plane->index);
			return -EINVAL;
		}
		if (src_y % (fb_is_interlaced ? 4 : 2)) {
			DRM_DEBUG_KMS("[PLANE:%d] bad uv yoffset\n", plane->index);
			return -EINVAL;
		}
	}

	/* for tile formats, framebuffer has to be tile aligned */
	switch (fb->modifier) {
	case DRM_FORMAT_MOD_AMPHION_TILED:
		if (fb->width % 8) {
			DRM_DEBUG_KMS("[PLANE:%d] bad fb width for AMPHION tile\n",
					plane->index);
			return -EINVAL;
		}
		if (fb->height % 256) {
			DRM_DEBUG_KMS("[PLANE:%d] bad fb height for AMPHION tile\n",
					plane->index);
			return -EINVAL;
		}
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
		if (fb->width % 4) {
			DRM_DEBUG_KMS("[PLANE:%d] bad fb width for VIVANTE tile\n",
					plane->index);
			return -EINVAL;
		}
		if (fb->height % 4) {
			DRM_DEBUG_KMS("[PLANE:%d] bad fb height for VIVANTE tile\n",
					plane->index);
			return -EINVAL;
		}
		break;
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		if (fb->width % 64) {
			DRM_DEBUG_KMS("[PLANE:%d] bad fb width for VIVANTE super tile\n",
					plane->index);
			return -EINVAL;
		}
		if (fb->height % 64) {
			DRM_DEBUG_KMS("[PLANE:%d] bad fb height for VIVANTE super tile\n",
					plane->index);
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	/* do not support BT709 full range */
	if (dpu_plane_fb_format_is_yuv(fb->format->format) &&
	    state->color_encoding == DRM_COLOR_YCBCR_BT709 &&
	    state->color_range == DRM_COLOR_YCBCR_FULL_RANGE)
		return -EINVAL;

again:
	fu = source_to_fu(res,
		check_aux_source ? dpstate->aux_source : dpstate->source);
	if (!fu) {
		DRM_DEBUG_KMS("[PLANE:%d] cannot get fetch unit\n",
				plane->index);
		return -EINVAL;
	}

	dprc = fu->dprc;

	if (dpstate->need_aux_source)
		frame_width = check_aux_source ?
				dpstate->right_src_w : dpstate->left_src_w;
	else
		frame_width = src_w;

	if (dprc &&
	    dprc_format_supported(dprc, fb->format->format, fb->modifier) &&
	    dprc_stride_supported(dprc, fb->pitches[0], fb->pitches[1],
				  frame_width, fb->format->format)) {
		if (check_aux_source)
			dpstate->use_aux_prefetch = true;
		else
			dpstate->use_prefetch = true;
	} else {
		if (check_aux_source)
			dpstate->use_aux_prefetch = false;
		else
			dpstate->use_prefetch = false;
	}

	if (fb->modifier) {
		if (check_aux_source && !dpstate->use_aux_prefetch) {
			DRM_DEBUG_KMS("[PLANE:%d] cannot do tile resolving wo prefetch\n",
					plane->index);
			return -EINVAL;
		} else if (!check_aux_source && !dpstate->use_prefetch) {
			DRM_DEBUG_KMS("[PLANE:%d] cannot do tile resolving wo prefetch\n",
					plane->index);
			return -EINVAL;
		}
	}

	/* base address alignment check */
	baseaddr = drm_plane_state_to_baseaddr(state, check_aux_source);
	switch (fb->format->format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		bpp = 16;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		bpp = 8;
		break;
	default:
		bpp = fb->format->cpp[0] * 8;
		break;
	}
	switch (bpp) {
	case 32:
		if (baseaddr & 0x3) {
			DRM_DEBUG_KMS("[PLANE:%d] 32bpp fb bad baddr alignment\n",
					plane->index);
			return -EINVAL;
		}
		break;
	case 16:
		if (fb->modifier) {
			if (baseaddr & 0x1) {
				DRM_DEBUG_KMS("[PLANE:%d] 16bpp tile fb bad baddr alignment\n",
						plane->index);
				return -EINVAL;
			}
		} else {
			if (check_aux_source) {
				if (baseaddr &
				    (dpstate->use_aux_prefetch ? 0x7 : 0x1)) {
					DRM_DEBUG_KMS("[PLANE:%d] 16bpp fb bad baddr alignment\n",
							plane->index);
					return -EINVAL;
				}
			} else {
				if (baseaddr &
				    (dpstate->use_prefetch ? 0x7 : 0x1)) {
					DRM_DEBUG_KMS("[PLANE:%d] 16bpp fb bad baddr alignment\n",
							plane->index);
					return -EINVAL;
				}
			}
		}
		break;
	}

	if (fb->pitches[0] > 0x10000) {
		DRM_DEBUG_KMS("[PLANE:%d] fb pitch[0] is too big\n",
				plane->index);
		return -EINVAL;
	}

	/* UV base address alignment check, assuming 16bpp */
	if (fb->format->num_planes > 1) {
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state,
							check_aux_source);
		if (fb->modifier) {
			if (uv_baseaddr & 0x1) {
				DRM_DEBUG_KMS("[PLANE:%d] bad uv baddr alignment for tile fb\n",
						plane->index);
				return -EINVAL;
			}
		} else {
			if (check_aux_source) {
				if (uv_baseaddr &
				    (dpstate->use_aux_prefetch ? 0x7 : 0x1)) {
					DRM_DEBUG_KMS("[PLANE:%d] bad uv baddr alignment\n",
							plane->index);
					return -EINVAL;
				}
			} else {
				if (uv_baseaddr &
				    (dpstate->use_prefetch ? 0x7 : 0x1)) {
					DRM_DEBUG_KMS("[PLANE:%d] bad uv baddr alignment\n",
							plane->index);
					return -EINVAL;
				}
			}
		}

		if (fb->pitches[1] > 0x10000) {
			DRM_DEBUG_KMS("[PLANE:%d] fb pitch[1] is too big\n",
					plane->index);
			return -EINVAL;
		}
	}

	if (!check_aux_source && dpstate->use_prefetch &&
	    !dprc_stride_double_check(dprc, frame_width, src_x,
					fb->format->format,
					fb->modifier,
					baseaddr, uv_baseaddr)) {
		if (fb->modifier) {
			DRM_DEBUG_KMS("[PLANE:%d] bad pitch\n",
					plane->index);
			return -EINVAL;
		}

		if (bpp == 16 && (baseaddr & 0x1)) {
			DRM_DEBUG_KMS("[PLANE:%d] bad baddr alignment\n",
					plane->index);
			return -EINVAL;
		}

		if (uv_baseaddr & 0x1) {
			DRM_DEBUG_KMS("[PLANE:%d] bad uv baddr alignment\n",
					plane->index);
			return -EINVAL;
		}

		dpstate->use_prefetch = false;
	} else if (check_aux_source && dpstate->use_aux_prefetch &&
		   !dprc_stride_double_check(dprc, frame_width, src_x,
					fb->format->format,
					fb->modifier,
					baseaddr, uv_baseaddr)) {
		if (fb->modifier) {
			DRM_DEBUG_KMS("[PLANE:%d] bad pitch\n", plane->index);
			return -EINVAL;
		}

		if (bpp == 16 && (baseaddr & 0x1)) {
			DRM_DEBUG_KMS("[PLANE:%d] bad baddr alignment\n",
					plane->index);
			return -EINVAL;
		}

		if (uv_baseaddr & 0x1) {
			DRM_DEBUG_KMS("[PLANE:%d] bad uv baddr alignment\n",
					plane->index);
			return -EINVAL;
		}

		dpstate->use_aux_prefetch = false;
	}

	if (dpstate->need_aux_source && !check_aux_source) {
		check_aux_source = true;
		goto again;
	}

	return 0;
}

int dpu_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *old_state)
{
	struct dpu_plane *dplane = to_dpu_plane(plane);
	struct drm_plane_state *state = plane->state;
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_fetchunit *fu;
	struct dpu_fetchunit *fe = NULL;
	struct dprc *dprc;
	struct dpu_hscaler *hs = NULL;
	struct dpu_vscaler *vs = NULL;
	struct dpu_layerblend *lb;
	struct dpu_extdst *ed;
	struct dpu_framegen *fg;
	dma_addr_t baseaddr, uv_baseaddr = 0;
	dpu_block_id_t blend, fe_id, vs_id = ID_NONE, hs_id;
	lb_sec_sel_t source;
	lb_prim_sel_t stage;
	unsigned int stream_id;
	unsigned int src_w, src_h, src_x, src_y, dst_w, dst_h;
	unsigned int crtc_x;
	unsigned int mt_w = 0, mt_h = 0;	/* w/h in a micro-tile */
	int bpp, lb_id;
	bool need_fetcheco, need_hscaler = false, need_vscaler = false;
	bool prefetch_start, uv_prefetch_start;
	bool crtc_use_pc = dpstate->left_src_w || dpstate->right_src_w;
	bool update_aux_source = false;
	bool use_prefetch;
	bool need_modeset;
	bool fb_is_interlaced;

	/*
	 * Do nothing since the plane is disabled by
	 * crtc_func->atomic_begin/flush.
	 */
	if (!fb)
		return -EINVAL;

	/* always request full modeset */
	need_modeset = true;
	fb_is_interlaced = !!(fb->flags & DRM_MODE_FB_INTERLACED);

again:
	need_fetcheco = false;
	prefetch_start = false;
	uv_prefetch_start = false;

	source = update_aux_source ? dpstate->aux_source : dpstate->source;
	blend = update_aux_source ? dpstate->aux_blend : dpstate->blend;
	stage = update_aux_source ? dpstate->aux_stage : dpstate->stage;
	use_prefetch = update_aux_source ?
			dpstate->use_aux_prefetch : dpstate->use_prefetch;

	if (crtc_use_pc) {
		if (update_aux_source) {
			stream_id = 1;
			crtc_x = dpstate->right_crtc_x;
		} else {
			stream_id = dpstate->left_src_w ? 0 : 1;
			crtc_x = dpstate->left_src_w ?
				 dpstate->left_crtc_x : dpstate->right_crtc_x;
		}
	} else {
		stream_id = dplane->stream_id;
		crtc_x = state->crtc_x;
	}

	fg = res->fg[stream_id];

	fu = source_to_fu(res, source);
	if (!fu)
		return -EINVAL;

	dprc = fu->dprc;

	lb_id = blend_to_id(blend);
	if (lb_id < 0)
		return -EINVAL;

	lb = res->lb[lb_id];

	if (crtc_use_pc) {
		if (update_aux_source || !dpstate->left_src_w)
			src_w = dpstate->right_src_w;
		else
			src_w = dpstate->left_src_w;
	} else {
		src_w = drm_rect_width(&state->src) >> 16;
	}
	src_h = drm_rect_height(&state->src) >> 16;
	if (crtc_use_pc && update_aux_source) {
		if (fb->modifier)
			src_x = (state->src_x >> 16) + dpstate->left_src_w;
		else
			src_x = 0;
	} else {
		src_x = fb->modifier ? (state->src_x >> 16) : 0;
	}
	src_y = fb->modifier ? (state->src_y >> 16) : 0;
	dst_w = drm_rect_width(&state->dst);
	dst_h = drm_rect_height(&state->dst);

	if (fetchunit_is_fetchdecode(fu)) {
		if (fetchdecode_need_fetcheco(fu, fb->format->format)) {
			need_fetcheco = true;
			fe = fetchdecode_get_fetcheco(fu);
			if (IS_ERR(fe))
				return -EINVAL;
		}

		/* assume pixel combiner is unused */
		if ((src_w != dst_w) && !crtc_use_pc) {
			need_hscaler = true;
			hs = fetchdecode_get_hscaler(fu);
			if (IS_ERR(hs))
				return -EINVAL;
		}

		if ((src_h != dst_h) || fb_is_interlaced) {
			need_vscaler = true;
			vs = fetchdecode_get_vscaler(fu);
			if (IS_ERR(vs))
				return -EINVAL;
		}
	}

	switch (fb->format->format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		bpp = 16;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		bpp = 8;
		break;
	default:
		bpp = fb->format->cpp[0] * 8;
		break;
	}

	switch (fb->modifier) {
	case DRM_FORMAT_MOD_AMPHION_TILED:
		mt_w = 8;
		mt_h = 8;
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		mt_w = (bpp == 16) ? 8 : 4;
		mt_h = 4;
		break;
	default:
		break;
	}

	baseaddr = drm_plane_state_to_baseaddr(state, update_aux_source);
	if (need_fetcheco)
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state,
							update_aux_source);

	if (use_prefetch &&
	    (fu->ops->get_stream_id(fu) == DPU_PLANE_SRC_DISABLED ||
	     need_modeset))
		prefetch_start = true;

	fu->ops->set_burstlength(fu, src_x, mt_w, bpp, baseaddr, use_prefetch);
	fu->ops->set_src_bpp(fu, bpp);
	/* Original linux driver puts src_w into x_offset parameter,
	   which corrupts 1366x768 in tile=2 mode */
	fu->ops->set_src_stride(fu, src_w, src_x, mt_w, bpp, fb->pitches[0],
				baseaddr, use_prefetch);
	fu->ops->set_src_buf_dimensions(fu, src_w, src_h, 0, fb_is_interlaced);
	fu->ops->set_pixel_blend_mode(fu, state->pixel_blend_mode,
					state->alpha, fb->format->format);
	fu->ops->set_fmt(fu, fb->format->format, state->color_encoding,
					state->color_range, fb_is_interlaced);
	fu->ops->enable_src_buf(fu);
	fu->ops->set_framedimensions(fu, src_w, src_h, fb_is_interlaced);
	fu->ops->set_baseaddress(fu, src_w, src_x, src_y, mt_w, mt_h, bpp,
				 baseaddr);
	fu->ops->set_stream_id(fu, stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

	DRM_DEBUG_KMS("[PLANE:%d] %s-0x%02x\n",
		plane->index, fu->name, fu->id);

	if (need_fetcheco) {
		fe_id = fetcheco_get_block_id(fe);
		if (fe_id == ID_NONE)
			return -EINVAL;

		if (use_prefetch &&
		    (fe->ops->get_stream_id(fe) == DPU_PLANE_SRC_DISABLED ||
		     need_modeset))
			uv_prefetch_start = true;

		fetchdecode_pixengcfg_dynamic_src_sel(fu,
						(fd_dynamic_src_sel_t)fe_id);
		fe->ops->set_burstlength(fe, src_w, mt_w, bpp, uv_baseaddr,
					 use_prefetch);
		fe->ops->set_src_bpp(fe, 16);
		fe->ops->set_src_stride(fe, src_w, src_x, mt_w, bpp,
					fb->pitches[1],
					uv_baseaddr, use_prefetch);
		fe->ops->set_fmt(fe, fb->format->format, state->color_encoding,
					state->color_range, fb_is_interlaced);
		fe->ops->set_src_buf_dimensions(fe, src_w, src_h,
						fb->format->format,
						fb_is_interlaced);
		fe->ops->set_framedimensions(fe, src_w, src_h,
						fb_is_interlaced);
		fe->ops->set_baseaddress(fe, src_w, src_x, src_y / 2,
					 mt_w, mt_h, bpp, uv_baseaddr);
		fe->ops->enable_src_buf(fe);
		fe->ops->set_stream_id(fe, stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		DRM_DEBUG_KMS("[PLANE:%d] %s-0x%02x\n",
			plane->index, fe->name, fe_id);
	} else {
		if (fetchunit_is_fetchdecode(fu))
			fetchdecode_pixengcfg_dynamic_src_sel(fu,
								FD_SRC_DISABLE);
	}

	/* vscaler comes first */
	if (need_vscaler) {
		vs_id = vscaler_get_block_id(vs);
		if (vs_id == ID_NONE)
			return -EINVAL;

		vscaler_pixengcfg_dynamic_src_sel(vs, (vs_src_sel_t)source);
		vscaler_pixengcfg_clken(vs, CLKEN__AUTOMATIC);
		vscaler_setup1(vs, src_h, state->crtc_h, fb_is_interlaced);
		vscaler_setup2(vs, fb_is_interlaced);
		vscaler_setup3(vs, fb_is_interlaced);
		vscaler_output_size(vs, dst_h);
		vscaler_field_mode(vs, fb_is_interlaced ?
						SCALER_ALWAYS0 : SCALER_INPUT);
		vscaler_filter_mode(vs, SCALER_LINEAR);
		vscaler_scale_mode(vs, SCALER_UPSCALE);
		vscaler_mode(vs, SCALER_ACTIVE);
		vscaler_set_stream_id(vs, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		source = (lb_sec_sel_t)vs_id;

		DRM_DEBUG_KMS("[PLANE:%d] vscaler-0x%02x\n", plane->index, vs_id);
	}

	/* and then, hscaler */
	if (need_hscaler) {
		hs_id = hscaler_get_block_id(hs);
		if (hs_id == ID_NONE)
			return -EINVAL;

		hscaler_pixengcfg_dynamic_src_sel(hs, need_vscaler ?
							(hs_src_sel_t)vs_id :
							(hs_src_sel_t)source);
		hscaler_pixengcfg_clken(hs, CLKEN__AUTOMATIC);
		hscaler_setup1(hs, src_w, dst_w);
		hscaler_output_size(hs, dst_w);
		hscaler_filter_mode(hs, SCALER_LINEAR);
		hscaler_scale_mode(hs, SCALER_UPSCALE);
		hscaler_mode(hs, SCALER_ACTIVE);
		hscaler_set_stream_id(hs, dplane->stream_id ?
					DPU_PLANE_SRC_TO_DISP_STREAM1 :
					DPU_PLANE_SRC_TO_DISP_STREAM0);

		source = (lb_sec_sel_t)hs_id;

		DRM_DEBUG_KMS("[PLANE:%d] hscaler-0x%02x\n", plane->index, hs_id);
	}

	if (use_prefetch) {
		dprc_configure(dprc, stream_id,
			       src_w, src_h, src_x, src_y,
			       fb->pitches[0], fb->format->format,
			       fb->modifier, baseaddr, uv_baseaddr,
			       prefetch_start, uv_prefetch_start,
			       fb_is_interlaced);

		dprc_enable(dprc);

		dprc_reg_update(dprc);

		if (prefetch_start || uv_prefetch_start) {
			dprc_first_frame_handle(dprc);

			if (!need_modeset && state->normalized_zpos != 0)
				if (framegen_wait_for_frame_counter_moving(fg))
					return -EINVAL;
		}

		if (update_aux_source)
			DRM_DEBUG_KMS("[PLANE:%d] use aux prefetch\n", plane->index);
		else
			DRM_DEBUG_KMS("[PLANE:%d] use prefetch\n", plane->index);
	} else if (dprc) {
		dprc_disable(dprc);

		if (update_aux_source)
			DRM_DEBUG_KMS("[PLANE:%d] bypass aux prefetch\n", plane->index);
		else
			DRM_DEBUG_KMS("[PLANE:%d] bypass prefetch\n", plane->index);
	}

	layerblend_pixengcfg_dynamic_prim_sel(lb, stage);
	layerblend_pixengcfg_dynamic_sec_sel(lb, source);
	layerblend_control(lb, LB_BLEND);
	layerblend_blendcontrol(lb, state->normalized_zpos,
				state->pixel_blend_mode, state->alpha);
	layerblend_pixengcfg_clken(lb, CLKEN__AUTOMATIC);
	layerblend_position(lb, crtc_x, state->crtc_y);

	if (crtc_use_pc) {
		if ((!stream_id && dpstate->is_left_top) ||
		     (stream_id && dpstate->is_right_top)) {
			ed = res->ed[stream_id];
			extdst_pixengcfg_src_sel(ed, (extdst_src_sel_t)blend);
		}
	} else {
		if (dpstate->is_top) {
			ed = res->ed[stream_id];
			extdst_pixengcfg_src_sel(ed, (extdst_src_sel_t)blend);
		}
	}

	if (update_aux_source)
		DRM_DEBUG_KMS("[PLANE:%d] *aux* source-0x%02x stage-0x%02x blend-0x%02x\n",
				plane->index, source, dpstate->stage, dpstate->blend);
	else
		DRM_DEBUG_KMS("[PLANE:%d] source-0x%02x stage-0x%02x blend-0x%02x\n",
				plane->index, source, dpstate->stage, dpstate->blend);

	if (dpstate->need_aux_source && !update_aux_source) {
		update_aux_source = true;
		goto again;
	}

	return 0;
}

void dpu_plane_atomic_page_flip(struct drm_plane *plane,
	struct drm_plane_state *old_state)
{
	struct dpu_plane *dplane = to_dpu_plane(plane);
	struct drm_plane_state *state = plane->state;
	struct dpu_plane_state *dpstate = to_dpu_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_fetchunit *fu;
	struct dprc *dprc;
	dma_addr_t baseaddr, uv_baseaddr = 0;
	lb_sec_sel_t source;
	unsigned int stream_id;
	unsigned int src_w, src_h, src_x, src_y;
	unsigned int mt_w = 0, mt_h = 0;	/* w/h in a micro-tile */
	int bpp;
	bool need_fetcheco = false;
	bool use_prefetch;
	bool fb_is_interlaced;

	if (!fb || !state->crtc || !state->visible)
		return;

	fb_is_interlaced = !!(fb->flags & DRM_MODE_FB_INTERLACED);
	source = dpstate->source;
	use_prefetch = dpstate->use_prefetch;

	stream_id = dplane->stream_id;

	fu = source_to_fu(res, source);
	if (!fu)
		return;

	dprc = fu->dprc;

	src_w = drm_rect_width(&state->src) >> 16;
	src_h = drm_rect_height(&state->src) >> 16;
	src_x = fb->modifier ? (state->src_x >> 16) : 0;
	src_y = fb->modifier ? (state->src_y >> 16) : 0;

	if (fetchunit_is_fetchdecode(fu)) {
		if (fetchdecode_need_fetcheco(fu, fb->format->format)) {
			need_fetcheco = true;
		}
	}

	switch (fb->format->format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		bpp = 16;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		bpp = 8;
		break;
	default:
		bpp = fb->format->cpp[0] * 8;
		break;
	}

	switch (fb->modifier) {
	case DRM_FORMAT_MOD_AMPHION_TILED:
		mt_w = 8;
		mt_h = 8;
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		mt_w = (bpp == 16) ? 8 : 4;
		mt_h = 4;
		break;
	default:
		break;
	}

	baseaddr = drm_plane_state_to_baseaddr(state, false);
	if (need_fetcheco)
		uv_baseaddr = drm_plane_state_to_uvbaseaddr(state,
			false);

	fu->ops->set_burstlength(fu, src_x, mt_w, bpp, baseaddr, use_prefetch);
	/* Original linux driver puts src_w into x_offset parameter,
	   which corrupts 1366x768 in tile=2 mode */
	fu->ops->set_src_stride(fu, src_w, src_x, mt_w, bpp, fb->pitches[0],
		baseaddr, use_prefetch);
	fu->ops->set_pixel_blend_mode(fu, state->pixel_blend_mode,
		state->alpha, fb->format->format);
	fu->ops->set_fmt(fu, fb->format->format, state->color_encoding,
		state->color_range, fb_is_interlaced);
	fu->ops->set_baseaddress(fu, src_w, src_x, src_y, mt_w, mt_h, bpp,
		baseaddr);

	if (need_fetcheco) {
		/* TODO: support color format setting for fetcheco - only needed for two-plane formats */
	}

	if (use_prefetch) {
		dprc_page_flip(dprc, stream_id, fb->pitches[0],
			src_w, src_h, src_x, src_y,
			fb->format->format, fb->modifier,
			baseaddr, uv_baseaddr);

		dprc_enable(dprc);

		dprc_reg_update(dprc);
	}
}

struct dpu_plane *dpu_plane_create(unsigned int stream_id,
				   struct dpu_plane_grp *grp,
				   enum drm_plane_type type, int idx)
{
	struct dpu_plane *dpu_plane;
	struct drm_plane *plane;
	unsigned int zpos = dpu_plane_get_default_zpos(type);

	dpu_plane = kzalloc(sizeof(*dpu_plane), GFP_KERNEL);
	if (!dpu_plane)
		return ERR_PTR(-ENOMEM);

	dpu_plane->stream_id = stream_id;
	dpu_plane->grp = grp;

	plane = &dpu_plane->base;

	plane->index = idx;

	return dpu_plane;
}
