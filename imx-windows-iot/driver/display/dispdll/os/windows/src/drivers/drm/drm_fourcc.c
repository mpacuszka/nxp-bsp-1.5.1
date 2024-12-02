/*
 * Copyright (c) 2016 Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright 2022 NXP
 *
 * DRM core format related functions
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

#include <asm-generic/bug.h>
#include <linux/kernel.h>

#include <drm/drm_fourcc.h>

/*
 * Internal function to query information for a given format. See
 * drm_format_info() for the public API.
 */
const struct drm_format_info *__drm_format_info(u32 format)
{
	static const struct drm_format_info formats[] = {
		{.format = DRM_FORMAT_C8,.depth = 8,.num_planes = 1,.cpp = { 1, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_RGB332,.depth = 8,.num_planes = 1,.cpp = { 1, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_BGR233,.depth = 8,.num_planes = 1,.cpp = { 1, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_XRGB4444,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_XBGR4444,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_RGBX4444,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_BGRX4444,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_ARGB4444,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_ABGR4444,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_RGBA4444,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_BGRA4444,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_XRGB1555,.depth = 15,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_XBGR1555,.depth = 15,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_RGBX5551,.depth = 15,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_BGRX5551,.depth = 15,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_ARGB1555,.depth = 15,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_ABGR1555,.depth = 15,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_RGBA5551,.depth = 15,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_BGRA5551,.depth = 15,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_RGB565,.depth = 16,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_BGR565,.depth = 16,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_RGB888,.depth = 24,.num_planes = 1,.cpp = { 3, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_BGR888,.depth = 24,.num_planes = 1,.cpp = { 3, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_XRGB8888,.depth = 24,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_XBGR8888,.depth = 24,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_RGBX8888,.depth = 24,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_BGRX8888,.depth = 24,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_RGB565_A8,.depth = 24,.num_planes = 2,.cpp = { 2, 1, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_BGR565_A8,.depth = 24,.num_planes = 2,.cpp = { 2, 1, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_XRGB2101010,.depth = 30,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_XBGR2101010,.depth = 30,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_RGBX1010102,.depth = 30,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_BGRX1010102,.depth = 30,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_ARGB2101010,.depth = 30,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_ABGR2101010,.depth = 30,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_RGBA1010102,.depth = 30,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_BGRA1010102,.depth = 30,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_ARGB8888,.depth = 32,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_ABGR8888,.depth = 32,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_RGBA8888,.depth = 32,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_BGRA8888,.depth = 32,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_XRGB16161616F,.depth = 0,.num_planes = 1,.cpp = { 8, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_XBGR16161616F,.depth = 0,.num_planes = 1,.cpp = { 8, 0, 0 },.hsub = 1,.vsub = 1 },
		{.format = DRM_FORMAT_ARGB16161616F,.depth = 0,.num_planes = 1,.cpp = { 8, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_ABGR16161616F,.depth = 0,.num_planes = 1,.cpp = { 8, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_RGB888_A8,.depth = 32,.num_planes = 2,.cpp = { 3, 1, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_BGR888_A8,.depth = 32,.num_planes = 2,.cpp = { 3, 1, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_XRGB8888_A8,.depth = 32,.num_planes = 2,.cpp = { 4, 1, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_XBGR8888_A8,.depth = 32,.num_planes = 2,.cpp = { 4, 1, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_RGBX8888_A8,.depth = 32,.num_planes = 2,.cpp = { 4, 1, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_BGRX8888_A8,.depth = 32,.num_planes = 2,.cpp = { 4, 1, 0 },.hsub = 1,.vsub = 1,.has_alpha = true },
		{.format = DRM_FORMAT_YUV410,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 4,.vsub = 4,.is_yuv = true },
		{.format = DRM_FORMAT_YVU410,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 4,.vsub = 4,.is_yuv = true },
		{.format = DRM_FORMAT_YUV411,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 4,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_YVU411,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 4,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_YUV420,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 2,.vsub = 2,.is_yuv = true },
		{.format = DRM_FORMAT_YVU420,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 2,.vsub = 2,.is_yuv = true },
		{.format = DRM_FORMAT_YUV422,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_YVU422,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_YUV444,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_YVU444,.depth = 0,.num_planes = 3,.cpp = { 1, 1, 1 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_NV12,.depth = 0,.num_planes = 2,.cpp = { 1, 2, 0 },.hsub = 2,.vsub = 2,.is_yuv = true },
		{.format = DRM_FORMAT_NV21,.depth = 0,.num_planes = 2,.cpp = { 1, 2, 0 },.hsub = 2,.vsub = 2,.is_yuv = true },
		{.format = DRM_FORMAT_NV16,.depth = 0,.num_planes = 2,.cpp = { 1, 2, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_NV61,.depth = 0,.num_planes = 2,.cpp = { 1, 2, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_NV24,.depth = 0,.num_planes = 2,.cpp = { 1, 2, 0 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_NV42,.depth = 0,.num_planes = 2,.cpp = { 1, 2, 0 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_YUYV,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_YVYU,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_UYVY,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_VYUY,.depth = 0,.num_planes = 1,.cpp = { 2, 0, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_XYUV8888,.depth = 0,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_VUY888,.depth = 0,.num_planes = 1,.cpp = { 3, 0, 0 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_AYUV,.depth = 0,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true,.is_yuv = true },
		{.format = DRM_FORMAT_Y210,.depth = 0,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_Y212,.depth = 0,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_Y216,.depth = 0,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 2,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_Y410,.depth = 0,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true,.is_yuv = true },
		{.format = DRM_FORMAT_Y412,.depth = 0,.num_planes = 1,.cpp = { 8, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true,.is_yuv = true },
		{.format = DRM_FORMAT_Y416,.depth = 0,.num_planes = 1,.cpp = { 8, 0, 0 },.hsub = 1,.vsub = 1,.has_alpha = true,.is_yuv = true },
		{.format = DRM_FORMAT_XVYU2101010,.depth = 0,.num_planes = 1,.cpp = { 4, 0, 0 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_XVYU12_16161616,.depth = 0,.num_planes = 1,.cpp = { 8, 0, 0 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_XVYU16161616,.depth = 0,.num_planes = 1,.cpp = { 8, 0, 0 },.hsub = 1,.vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_Y0L0,.depth = 0,.num_planes = 1,
		  .char_per_block = { 8, 0, 0 },.block_w = { 2, 0, 0 },.block_h = { 2, 0, 0 },
		  .hsub = 2,.vsub = 2,.has_alpha = true,.is_yuv = true },
		{.format = DRM_FORMAT_X0L0,.depth = 0,.num_planes = 1,
		  .char_per_block = { 8, 0, 0 },.block_w = { 2, 0, 0 },.block_h = { 2, 0, 0 },
		  .hsub = 2,.vsub = 2,.is_yuv = true },
		{.format = DRM_FORMAT_Y0L2,.depth = 0,.num_planes = 1,
		  .char_per_block = { 8, 0, 0 },.block_w = { 2, 0, 0 },.block_h = { 2, 0, 0 },
		  .hsub = 2,.vsub = 2,.has_alpha = true,.is_yuv = true },
		{.format = DRM_FORMAT_X0L2,.depth = 0,.num_planes = 1,
		  .char_per_block = { 8, 0, 0 },.block_w = { 2, 0, 0 },.block_h = { 2, 0, 0 },
		  .hsub = 2,.vsub = 2,.is_yuv = true },
		{.format = DRM_FORMAT_P010,.depth = 0,.num_planes = 2,
		  .char_per_block = { 2, 4, 0 },.block_w = { 1, 1, 0 },.block_h = { 1, 1, 0 },
		  .hsub = 2,.vsub = 2,.is_yuv = true},
		{.format = DRM_FORMAT_P012,.depth = 0,.num_planes = 2,
		  .char_per_block = { 2, 4, 0 },.block_w = { 1, 1, 0 },.block_h = { 1, 1, 0 },
		   .hsub = 2,.vsub = 2,.is_yuv = true},
		{.format = DRM_FORMAT_P016,.depth = 0,.num_planes = 2,
		  .char_per_block = { 2, 4, 0 },.block_w = { 1, 1, 0 },.block_h = { 1, 1, 0 },
		  .hsub = 2,.vsub = 2,.is_yuv = true},
		{.format = DRM_FORMAT_NV12_10LE40,.depth = 0,.num_planes = 2,
		  .char_per_block = { 5, 5, 0 },.block_w = { 4, 2, 0 },.block_h = { 1, 1, 0 },
		  .hsub = 2,.vsub = 2,.is_yuv = true },
		{.format = DRM_FORMAT_P210,.depth = 0,
		  .num_planes = 2,.char_per_block = { 2, 4, 0 },
		  .block_w = { 1, 1, 0 },.block_h = { 1, 1, 0 },.hsub = 2,
		  .vsub = 1,.is_yuv = true },
		{.format = DRM_FORMAT_VUY101010,.depth = 0,
		  .num_planes = 1,.cpp = { 0, 0, 0 },.hsub = 1,.vsub = 1,
		  .is_yuv = true },
		{.format = DRM_FORMAT_YUV420_8BIT,.depth = 0,
		  .num_planes = 1,.cpp = { 0, 0, 0 },.hsub = 2,.vsub = 2,
		  .is_yuv = true },
		{.format = DRM_FORMAT_YUV420_10BIT,.depth = 0,
		  .num_planes = 1,.cpp = { 0, 0, 0 },.hsub = 2,.vsub = 2,
		  .is_yuv = true },
		{.format = DRM_FORMAT_NV15,.depth = 0,
		  .num_planes = 2,.char_per_block = { 5, 5, 0 },
		  .block_w = { 4, 2, 0 },.block_h = { 1, 1, 0 },.hsub = 2,
		  .vsub = 2,.is_yuv = true },
		{.format = DRM_FORMAT_Q410,.depth = 0,
		  .num_planes = 3,.char_per_block = { 2, 2, 2 },
		  .block_w = { 1, 1, 1 },.block_h = { 1, 1, 1 },.hsub = 0,
		  .vsub = 0,.is_yuv = true },
		{.format = DRM_FORMAT_Q401,.depth = 0,
		  .num_planes = 3,.char_per_block = { 2, 2, 2 },
		  .block_w = { 1, 1, 1 },.block_h = { 1, 1, 1 },.hsub = 0,
		  .vsub = 0,.is_yuv = true },
	};

	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].format == format)
			return &formats[i];
	}

	return NULL;
}

/**
 * drm_format_info - query information for a given format
 * @format: pixel format (DRM_FORMAT_*)
 *
 * The caller should only pass a supported pixel format to this function.
 * Unsupported pixel formats will generate a warning in the kernel log.
 *
 * Returns:
 * The instance of struct drm_format_info that describes the pixel format, or
 * NULL if the format is unsupported.
 */
const struct drm_format_info *drm_format_info(u32 format)
{
	const struct drm_format_info *info;

	info = __drm_format_info(format);
	WARN_ON(!info);
	return info;
}

