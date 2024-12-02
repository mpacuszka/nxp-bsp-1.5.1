/*
 * Copyright (C) 2016 Red Hat
 * Copyright 2022,2024 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Rob Clark <robdclark@gmail.com>
 */

#ifndef DRM_PRINT_H_
#define DRM_PRINT_H_

#include <drm/drm.h>

/**
 * enum drm_debug_category - The DRM debug categories
 */
enum drm_debug_category {
	/**
	 * @DRM_UT_CORE: Used in the generic drm code: drm_ioctl.c, drm_mm.c,
	 * drm_memory.c, ...
	 */
	DRM_UT_CORE		= 0x01,
	/**
	 * @DRM_UT_DRIVER: Used in the vendor specific part of the driver: i915,
	 * radeon, ... macro.
	 */
	DRM_UT_DRIVER		= 0x02,
	/**
	 * @DRM_UT_KMS: Used in the modesetting code.
	 */
	DRM_UT_KMS		= 0x04,
	/**
	 * @DRM_UT_PRIME: Used in the prime code.
	 */
	DRM_UT_PRIME		= 0x08,
	/**
	 * @DRM_UT_ATOMIC: Used in the atomic code.
	 */
	DRM_UT_ATOMIC		= 0x10,
	/**
	 * @DRM_UT_VBL: Used for verbose debug message in the vblank code.
	 */
	DRM_UT_VBL		= 0x20,
	/**
	 * @DRM_UT_STATE: Used for verbose atomic state debugging.
	 */
	DRM_UT_STATE		= 0x40,
	/**
	 * @DRM_UT_LEASE: Used in the lease code.
	 */
	DRM_UT_LEASE		= 0x80,
	/**
	 * @DRM_UT_DP: Used in the DP code.
	 */
	DRM_UT_DP		= 0x100,
	/**
	 * @DRM_UT_DRMRES: Used in the drm managed resources code.
	 */
	DRM_UT_DRMRES		= 0x200,
};

void __drm_info(const char *format, ...);
void __drm_warn(const char *format, ...);
void __drm_dbg(enum drm_debug_category category, const char *format, ...);
void __drm_err(const char *format, ...);

#define DRM_DEV_ERROR_RATELIMITED(dev, fmt, ...) \
	DRM_DEV_ERROR(dev, fmt, ##__VA_ARGS__)

#define DRM_DEV_ERROR(dev, fmt, ...)					\
	__drm_err(fmt, ##__VA_ARGS__)

#define DRM_DEV_DEBUG(dev, fmt, ...)					\
	__drm_dbg(DRM_UT_CORE, fmt, ##__VA_ARGS__)

#define DRM_DEV_INFO(dev, fmt, ...)				\
	__drm_info(fmt, ##__VA_ARGS__)

#define DRM_INFO(fmt, ...)						\
	__drm_info(fmt, ##__VA_ARGS__)

#define DRM_WARN(fmt, ...)						\
	__drm_warn(fmt, ##__VA_ARGS__)

#define DRM_ERROR(fmt, ...)						\
	__drm_err(fmt, ##__VA_ARGS__)

#define DRM_ERROR_RATELIMITED(fmt, ...)					\
	__drm_err(fmt, ##__VA_ARGS__)

#define DRM_DEBUG(fmt, ...)						\
	__drm_dbg(DRM_UT_CORE, fmt, ##__VA_ARGS__)

#define DRM_DEBUG_KMS(fmt, ...)						\
	__drm_dbg(DRM_UT_KMS, fmt, ##__VA_ARGS__)

#define DRM_DEV_DEBUG_DRIVER(dev, fmt, ...)				\
	__drm_dbg(DRM_UT_DRIVER, fmt, ##__VA_ARGS__)

#endif /* DRM_PRINT_H_ */
