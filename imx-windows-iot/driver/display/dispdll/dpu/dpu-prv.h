/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2020, 2022-2024 NXP
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
#ifndef __DPU_PRV_H__
#define __DPU_PRV_H__

//#include <linux/firmware/imx/sci.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <drm/drm_fourcc.h>
#include <video/dpu.h>

#include "dpu-crtc.h"

#define STATICCONTROL			0x8
#define SHDLDREQSTICKY(lm)		(((lm) & 0xFF) << 24)
#define SHDLDREQSTICKY_MASK		(0xFF << 24)
#define BASEADDRESSAUTOUPDATE(lm)	(((lm) & 0xFF) << 16)
#define BASEADDRESSAUTOUPDATE_MASK	(0xFF << 16)
#define SHDEN				BIT(0)
#define BURSTBUFFERMANAGEMENT		0xC
#define SETNUMBUFFERS(n)		((n) & 0xFF)
#define SETBURSTLENGTH(n)		(((n) & 0x1F) << 8)
#define SETBURSTLENGTH_MASK		0x1F00
#define LINEMODE_MASK			0x80000000U
#define LINEMODE_SHIFT			31U
enum linemode {
	/*
	 * Mandatory setting for operation in the Display Controller.
	 * Works also for Blit Engine with marginal performance impact.
	 */
	LINEMODE__DISPLAY = 0,
	/* Recommended setting for operation in the Blit Engine. */
	LINEMODE__BLIT = 1 << LINEMODE_SHIFT,
};

#define BITSPERPIXEL(bpp)		(((bpp) & 0x3F) << 16)
#define STRIDE(n)			(((n) - 1) & 0xFFFF)
#define LINEWIDTH(w)			(((w) - 1) & 0x3FFF)
#define LINECOUNT(h)			((((h) - 1) & 0x3FFF) << 16)
#define ITUFORMAT			BIT(31)
#define R_BITS(n)			(((n) & 0xF) << 24)
#define G_BITS(n)			(((n) & 0xF) << 16)
#define B_BITS(n)			(((n) & 0xF) << 8)
#define A_BITS(n)			((n) & 0xF)
#define R_SHIFT(n)			(((n) & 0x1F) << 24)
#define G_SHIFT(n)			(((n) & 0x1F) << 16)
#define B_SHIFT(n)			(((n) & 0x1F) << 8)
#define A_SHIFT(n)			((n) & 0x1F)
#define Y_BITS(n)			R_BITS(n)
#define Y_BITS_MASK			0xF000000
#define U_BITS(n)			G_BITS(n)
#define U_BITS_MASK			0xF0000
#define V_BITS(n)			B_BITS(n)
#define V_BITS_MASK			0xF00
#define Y_SHIFT(n)			R_SHIFT(n)
#define Y_SHIFT_MASK			0x1F000000
#define U_SHIFT(n)			G_SHIFT(n)
#define U_SHIFT_MASK			0x1F0000
#define V_SHIFT(n)			B_SHIFT(n)
#define V_SHIFT_MASK			0x1F00
#define LAYERXOFFSET(x)			((x) & 0x7FFF)
#define LAYERYOFFSET(y)			(((y) & 0x7FFF) << 16)
#define CLIPWINDOWXOFFSET(x)		((x) & 0x7FFF)
#define CLIPWINDOWYOFFSET(y)		(((y) & 0x7FFF) << 16)
#define CLIPWINDOWWIDTH(w)		(((w) - 1) & 0x3FFF)
#define CLIPWINDOWHEIGHT(h)		((((h) - 1) & 0x3FFF) << 16)
#define CONSTANTALPHA_MASK		0xFF
#define CONSTANTALPHA(n)		((n) & CONSTANTALPHA_MASK)
#define	PALETTEENABLE			BIT(0)
typedef enum {
	TILE_FILL_ZERO,
	TILE_FILL_CONSTANT,
	TILE_PAD,
	TILE_PAD_ZERO,
} tilemode_t;
#define ALPHASRCENABLE			BIT(8)
#define ALPHACONSTENABLE		BIT(9)
#define ALPHAMASKENABLE			BIT(10)
#define ALPHATRANSENABLE		BIT(11)
#define ALPHA_ENABLE_MASK		(ALPHASRCENABLE  | ALPHACONSTENABLE | \
					 ALPHAMASKENABLE | ALPHATRANSENABLE)
#define RGBALPHASRCENABLE		BIT(12)
#define RGBALPHACONSTENABLE		BIT(13)
#define RGBALPHAMASKENABLE		BIT(14)
#define RGBALPHATRANSENABLE		BIT(15)
#define RGB_ENABLE_MASK			(RGBALPHASRCENABLE   | \
					 RGBALPHACONSTENABLE | \
					 RGBALPHAMASKENABLE  | \
					 RGBALPHATRANSENABLE)
#define PREMULCONSTRGB			BIT(16)
typedef enum {
	YUVCONVERSIONMODE__OFF,
	YUVCONVERSIONMODE__ITU601,
	YUVCONVERSIONMODE__ITU601_FR,
	YUVCONVERSIONMODE__ITU709,
} yuvconversionmode_t;
#define YUVCONVERSIONMODE_MASK		0x60000
#define YUVCONVERSIONMODE(m)		(((m) & 0x3) << 17)
#define GAMMAREMOVEENABLE		BIT(20)
#define CLIPWINDOWENABLE		BIT(30)
#define SOURCEBUFFERENABLE		BIT(31)
#define EMPTYFRAME			BIT(31)
#define FRAMEWIDTH(w)			(((w) - 1) & 0x3FFF)
#define FRAMEHEIGHT(h)			((((h) - 1) & 0x3FFF) << 16)
#define DELTAX_MASK			0x3F000
#define DELTAY_MASK			0xFC0000
#define DELTAX(x)			(((x) & 0x3F) << 12)
#define DELTAY(y)			(((y) & 0x3F) << 18)
#define YUV422UPSAMPLINGMODE_MASK	BIT(5)
#define YUV422UPSAMPLINGMODE(m)		(((m) & 0x1) << 5)
typedef enum {
	YUV422UPSAMPLINGMODE__REPLICATE,
	YUV422UPSAMPLINGMODE__INTERPOLATE,
} yuv422upsamplingmode_t;
#define INPUTSELECT_MASK		0x18
#define INPUTSELECT(s)			(((s) & 0x3) << 3)
typedef enum {
	INPUTSELECT__INACTIVE,
	INPUTSELECT__COMPPACK,
	INPUTSELECT__ALPHAMASK,
	INPUTSELECT__COORDINATE,
} inputselect_t;
#define RASTERMODE_MASK			0x7
#define RASTERMODE(m)			((m) & 0x7)
typedef enum {
	RASTERMODE__NORMAL,
	RASTERMODE__DECODE,
	RASTERMODE__ARBITRARY,
	RASTERMODE__PERSPECTIVE,
	RASTERMODE__YUV422,
	RASTERMODE__AFFINE,
} rastermode_t;
#define SHDTOKGEN			BIT(0)
#define FETCHTYPE_MASK			0xF

#define DPU_FRAC_PLANE_LAYER_NUM	8

#define DPU_VPROC_CAP_HSCALER4	BIT(0)
#define DPU_VPROC_CAP_VSCALER4	BIT(1)
#define DPU_VPROC_CAP_HSCALER5	BIT(2)
#define DPU_VPROC_CAP_VSCALER5	BIT(3)
#define DPU_VPROC_CAP_FETCHECO0	BIT(4)
#define DPU_VPROC_CAP_FETCHECO1	BIT(5)

#define DPU_VPROC_CAP_HSCALE	(DPU_VPROC_CAP_HSCALER4 | \
				 DPU_VPROC_CAP_HSCALER5)
#define DPU_VPROC_CAP_VSCALE	(DPU_VPROC_CAP_VSCALER4 | \
				 DPU_VPROC_CAP_VSCALER5)
#define DPU_VPROC_CAP_FETCHECO	(DPU_VPROC_CAP_FETCHECO0 | \
				 DPU_VPROC_CAP_FETCHECO1)

struct dpu_unit {
	char *name;
	unsigned int num;
	const unsigned int *ids;
	const unsigned long *pec_ofss;	/* PixEngCFG */
	const unsigned long *ofss;
	const unsigned int *dprc_ids;
};

struct cm_reg_ofs {
	u32 ipidentifier;
	u32 lockunlock;
	u32 lockstatus;
	u32 userinterruptmask;
	u32 interruptenable;
	u32 interruptpreset;
	u32 interruptclear;
	u32 interruptstatus;
	u32 userinterruptenable;
	u32 userinterruptpreset;
	u32 userinterruptclear;
	u32 userinterruptstatus;
	u32 generalpurpose;
};

struct dpu_data {
	unsigned long cm_ofs;			/* common */
	const struct dpu_unit *cfs;
	const struct dpu_unit *decs;
	const struct dpu_unit *eds;
	const struct dpu_unit *fds;
	const struct dpu_unit *fes;
	const struct dpu_unit *fgs;
	const struct dpu_unit *fls;
	const struct dpu_unit *fws;
	const struct dpu_unit *hss;
	const struct dpu_unit *lbs;
	const struct dpu_unit *sigs;
	const struct dpu_unit *sts;
	const struct dpu_unit *tcons;
	const struct dpu_unit *vss;
	const struct cm_reg_ofs *cm_reg_ofs;
	const unsigned long *unused_irq;

	unsigned int syncmode_min_prate;	/* need pixel combiner, KHz */
	unsigned int singlemode_max_width;
	unsigned int master_stream_id;

	u32 plane_src_mask;

	bool has_dual_ldb;
};

struct dpu_soc {
	struct device		*dev;
	const struct dpu_data	*data;
	spinlock_t		lock;

	int		pd_dc_dev;
	int		pd_pll0_dev;
	int		pd_pll1_dev;

	char __iomem		*cm_reg;

	int			id;
	int			usecount;

	int			irq_extdst0_shdload;
	int			irq_extdst4_shdload;
	int			irq_extdst1_shdload;
	int			irq_extdst5_shdload;
	int			irq_disengcfg_shdload0;
	int			irq_disengcfg_framecomplete0;
	int			irq_sig0_shdload;
	int			irq_sig0_valid;
	int			irq_disengcfg_shdload1;
	int			irq_disengcfg_framecomplete1;
	int			irq_sig1_shdload;
	int			irq_sig1_valid;
	int			irq_line_num;

	bool			irq_chip_pm_get_extdst0_shdload;
	bool			irq_chip_pm_get_extdst4_shdload;
	bool			irq_chip_pm_get_extdst1_shdload;
	bool			irq_chip_pm_get_extdst5_shdload;
	bool			irq_chip_pm_get_disengcfg_shdload0;
	bool			irq_chip_pm_get_disengcfg_framecomplete0;
	bool			irq_chip_pm_get_sig0_shdload;
	bool			irq_chip_pm_get_sig0_valid;
	bool			irq_chip_pm_get_disengcfg_shdload1;
	bool			irq_chip_pm_get_disengcfg_framecomplete1;
	bool			irq_chip_pm_get_sig1_shdload;
	bool			irq_chip_pm_get_sig1_valid;

	struct irq_domain	*domain;

	struct _sc_ipc_struct_t *dpu_ipc_handle;

	struct dpu_constframe	*cf_priv[4];
	struct dpu_disengcfg	*dec_priv[2];
	struct dpu_extdst	*ed_priv[4];
	struct dpu_fetchunit	*fd_priv[2];
	struct dpu_fetchunit	*fe_priv[4];
	struct dpu_framegen	*fg_priv[2];
	struct dpu_fetchunit	*fl_priv[1];
	struct dpu_fetchunit	*fw_priv[1];
	struct dpu_hscaler	*hs_priv[3];
	struct dpu_layerblend	*lb_priv[4];
	struct dpu_signature	*sig_priv[2];
	struct dpu_store	*st_priv[1];
	struct dpu_tcon		*tcon_priv[2];
	struct dpu_vscaler	*vs_priv[3];
};

struct Imx8qxpDisplayPDEVs {
    struct platform_device *p_prg_pdev;
    struct platform_device *p_dprc_pdev;
    struct platform_device *p_dpu_pdev;
    struct platform_device *p_client_devices;
    struct platform_device *p_irqsteer_pdev;
    UINT* p_refCount;
};

static const INT PRG_CNT = 9;
static const INT DPRC_CNT = 6;
static const INT CLIENT_DEVICE_CNT = 3;

int dpu_format_horz_chroma_subsampling(u32 format);
int dpu_format_vert_chroma_subsampling(u32 format);
int dpu_format_num_planes(u32 format);
int dpu_format_plane_width(int width, u32 format, int plane);
int dpu_format_plane_height(int height, u32 format, int plane);

#define _DECLARE_DPU_UNIT_INIT_FUNC(block)			\
void _dpu_##block##_init(struct dpu_soc *dpu, unsigned int id)	\

_DECLARE_DPU_UNIT_INIT_FUNC(cf);
_DECLARE_DPU_UNIT_INIT_FUNC(dec);
_DECLARE_DPU_UNIT_INIT_FUNC(ed);
_DECLARE_DPU_UNIT_INIT_FUNC(fd);
_DECLARE_DPU_UNIT_INIT_FUNC(fe);
_DECLARE_DPU_UNIT_INIT_FUNC(fg);
_DECLARE_DPU_UNIT_INIT_FUNC(fl);
_DECLARE_DPU_UNIT_INIT_FUNC(fw);
_DECLARE_DPU_UNIT_INIT_FUNC(hs);
_DECLARE_DPU_UNIT_INIT_FUNC(lb);
_DECLARE_DPU_UNIT_INIT_FUNC(sig);
_DECLARE_DPU_UNIT_INIT_FUNC(st);
_DECLARE_DPU_UNIT_INIT_FUNC(tcon);
_DECLARE_DPU_UNIT_INIT_FUNC(vs);

#define DECLARE_DPU_UNIT_INIT_FUNC(block)			\
int dpu_##block##_init(struct dpu_soc *dpu, unsigned int id,	\
			 unsigned long pec_base, unsigned long base)

DECLARE_DPU_UNIT_INIT_FUNC(cf);
DECLARE_DPU_UNIT_INIT_FUNC(dec);
DECLARE_DPU_UNIT_INIT_FUNC(ed);
DECLARE_DPU_UNIT_INIT_FUNC(fd);
DECLARE_DPU_UNIT_INIT_FUNC(fe);
DECLARE_DPU_UNIT_INIT_FUNC(fg);
DECLARE_DPU_UNIT_INIT_FUNC(fl);
DECLARE_DPU_UNIT_INIT_FUNC(fw);
DECLARE_DPU_UNIT_INIT_FUNC(hs);
DECLARE_DPU_UNIT_INIT_FUNC(lb);
DECLARE_DPU_UNIT_INIT_FUNC(sig);
DECLARE_DPU_UNIT_INIT_FUNC(st);
DECLARE_DPU_UNIT_INIT_FUNC(tcon);
DECLARE_DPU_UNIT_INIT_FUNC(vs);

#define DECLARE_DPU_UNIT_DEINIT_FUNC(block)			\
int dpu_##block##_deinit(struct dpu_soc *dpu, unsigned int id)

DECLARE_DPU_UNIT_DEINIT_FUNC(cf);
DECLARE_DPU_UNIT_DEINIT_FUNC(dec);
DECLARE_DPU_UNIT_DEINIT_FUNC(ed);
DECLARE_DPU_UNIT_DEINIT_FUNC(fd);
DECLARE_DPU_UNIT_DEINIT_FUNC(fe);
DECLARE_DPU_UNIT_DEINIT_FUNC(fg);
DECLARE_DPU_UNIT_DEINIT_FUNC(fl);
DECLARE_DPU_UNIT_DEINIT_FUNC(fw);
DECLARE_DPU_UNIT_DEINIT_FUNC(hs);
DECLARE_DPU_UNIT_DEINIT_FUNC(lb);
DECLARE_DPU_UNIT_DEINIT_FUNC(sig);
DECLARE_DPU_UNIT_DEINIT_FUNC(st);
DECLARE_DPU_UNIT_DEINIT_FUNC(tcon);
DECLARE_DPU_UNIT_DEINIT_FUNC(vs);

static inline u32 dpu_pec_fu_read(struct dpu_fetchunit *fu, unsigned int offset)
{
	return readl(fu->pec_base + offset);
}

static inline void dpu_pec_fu_write(struct dpu_fetchunit *fu,
				    unsigned int offset, u32 value)
{
	writel(value, fu->pec_base + offset);
}

static inline u32 dpu_fu_read(struct dpu_fetchunit *fu, unsigned int offset)
{
	return readl(fu->base + offset);
}

static inline void dpu_fu_write(struct dpu_fetchunit *fu,
				unsigned int offset, u32 value)
{
	writel(value, fu->base + offset);
}

static inline u32 rgb_color(u8 r, u8 g, u8 b, u8 a)
{
	return (r << 24) | (g << 16) | (b << 8) | a;
}

static inline u32 yuv_color(u8 y, u8 u, u8 v)
{
	return (y << 24) | (u << 16) | (v << 8);
}

void tcon_get_pc(struct dpu_tcon *tcon, void *data);

static const unsigned int cf_ids[] = {0, 1, 4, 5};
static const unsigned int dec_ids[] = {0, 1};
static const unsigned int ed_ids[] = {0, 1, 4, 5};
static const unsigned int fd_ids[] = {0, 1};
static const unsigned int fe_ids[] = {0, 1, 2, 9};
static const unsigned int fg_ids[] = {0, 1};
static const unsigned int fl_ids[] = {0};
static const unsigned int fw_ids[] = {2};
static const unsigned int hs_ids[] = {4, 5, 9};
static const unsigned int lb_ids[] = {0, 1, 2, 3};
static const unsigned int sig_ids[] = {0, 1};
static const unsigned int st_ids[] = {9};
static const unsigned int tcon_ids[] = {0, 1};
static const unsigned int vs_ids[] = {4, 5, 9};

static const unsigned int fd_dprc_ids[] = {3, 4};
static const unsigned int fl_dprc_ids[] = {2};
static const unsigned int fw_dprc_ids[] = {5};

struct dpu_pixel_format {
	u32 pixel_format;
	u32 bits;
	u32 shift;
};

static const struct dpu_pixel_format dpu_pixel_format_matrix[] = {
	{
		DRM_FORMAT_ARGB8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(8),
		R_SHIFT(16) | G_SHIFT(8)  | B_SHIFT(0)  | A_SHIFT(24),
	}, {
		DRM_FORMAT_XRGB8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(16) | G_SHIFT(8)  | B_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_ABGR8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(8),
		R_SHIFT(0)  | G_SHIFT(8)  | B_SHIFT(16) | A_SHIFT(24),
	}, {
		DRM_FORMAT_XBGR8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(0)  | G_SHIFT(8)  | B_SHIFT(16) | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGBA8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(8),
		R_SHIFT(24) | G_SHIFT(16) | B_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGBX8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(24) | G_SHIFT(16) | B_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_BGRA8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(8),
		R_SHIFT(8)  | G_SHIFT(16) | B_SHIFT(24) | A_SHIFT(0),
	}, {
		DRM_FORMAT_BGRX8888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(8)  | G_SHIFT(16) | B_SHIFT(24) | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGB888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(16) | G_SHIFT(8)  | B_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_BGR888,
		R_BITS(8)   | G_BITS(8)   | B_BITS(8)   | A_BITS(0),
		R_SHIFT(0)  | G_SHIFT(8)  | B_SHIFT(16) | A_SHIFT(0),
	}, {
		DRM_FORMAT_RGB565,
		R_BITS(5)   | G_BITS(6)   | B_BITS(5)   | A_BITS(0),
		R_SHIFT(11) | G_SHIFT(5)  | B_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_YUYV,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(8)  | V_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_UYVY,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(8)  | U_SHIFT(0)  | V_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_NV12,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(0)  | V_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_NV21,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(8)  | V_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_NV16,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(0)  | V_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_NV61,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(8)  | V_SHIFT(0)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_NV24,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(0)  | V_SHIFT(8)  | A_SHIFT(0),
	}, {
		DRM_FORMAT_NV42,
		Y_BITS(8)   | U_BITS(8)   | V_BITS(8)   | A_BITS(0),
		Y_SHIFT(0)  | U_SHIFT(8)  | V_SHIFT(0)  | A_SHIFT(0),
	},
};

#define DPU_CM_REG_DEFINE1(name1, name2)		\
static inline u32 name1(const struct cm_reg_ofs *ofs)	\
{							\
	return ofs->name2;				\
}

#define DPU_CM_REG_DEFINE2(name1, name2)		\
static inline u32 name1(const struct cm_reg_ofs *ofs,	\
			unsigned int n)			\
{							\
	return ofs->name2 + (4 * n);			\
}

DPU_CM_REG_DEFINE1(LOCKUNLOCK, lockunlock);
DPU_CM_REG_DEFINE1(LOCKSTATUS, lockstatus);
DPU_CM_REG_DEFINE2(USERINTERRUPTMASK, userinterruptmask);
DPU_CM_REG_DEFINE2(INTERRUPTENABLE, interruptenable);
DPU_CM_REG_DEFINE2(INTERRUPTPRESET, interruptpreset);
DPU_CM_REG_DEFINE2(INTERRUPTCLEAR, interruptclear);
DPU_CM_REG_DEFINE2(INTERRUPTSTATUS, interruptstatus);
DPU_CM_REG_DEFINE2(USERINTERRUPTENABLE, userinterruptenable);
DPU_CM_REG_DEFINE2(USERINTERRUPTPRESET, userinterruptpreset);
DPU_CM_REG_DEFINE2(USERINTERRUPTCLEAR, userinterruptclear);
DPU_CM_REG_DEFINE2(USERINTERRUPTSTATUS, userinterruptstatus);
DPU_CM_REG_DEFINE1(GENERALPURPOSE, generalpurpose);

inline u32 dpu_cm_read(struct dpu_soc *dpu, unsigned int offset)
{
	return readl(dpu->cm_reg + offset);
}

inline void dpu_cm_write(struct dpu_soc *dpu,
				unsigned int offset, u32 value)
{
	writel(value, dpu->cm_reg + offset);
}

inline bool is_plane_attached_to_crtc(struct drm_crtc_state *crtc_state,
	struct drm_plane *plane)
{
	return crtc_state->plane_mask & drm_plane_mask(plane);
}

/* Misc functions */
int dpu_sc_misc_get_handle(struct dpu_soc *dpu);
int dpu_pxlink_set_mst_addr(struct dpu_soc *dpu, int disp_id, u32 val);
int dpu_pxlink_set_mst_enable(struct dpu_soc *dpu, int disp_id, bool enable);
int dpu_pxlink_set_mst_valid(struct dpu_soc *dpu, int disp_id, bool enable);
int dpu_pxlink_set_sync_ctrl(struct dpu_soc *dpu, int disp_id, bool enable);
int dpu_pxlink_set_dc_sync_mode(struct dpu_soc *dpu, bool enable);
int dpu_sc_misc_init(struct dpu_soc *dpu);
int dpu_sc_pwr_set_mode(struct dpu_soc *dpu, u16 rsc, u8 mode);

/* Common functions */
int dpu_map_irq(struct dpu_soc *dpu, int irq);
void dpu_irq_ack(struct dpu_soc *dpu, int irq);
void dpu_irq_mask(struct dpu_soc *dpu, int irq);
void dpu_irq_unmask(struct dpu_soc *dpu, int irq);
bool dpu_disp_is_enabled(struct dpu_soc *dpu, int id);

int dpu_probe(struct platform_device *pdev,
	struct platform_device *client_devices, int client_devices_cnt,
	struct platform_device *dprc_pdev, int dprc_cnt);
int dpu_remove(struct platform_device *pdev,
	struct platform_device *client_devices, int client_devices_cnt);

/* CRTC */
int dpu_crtc_copy_display_mode(struct drm_display_mode *src_mode,
	struct drm_display_mode *dst_mode);
void dpu_drm_crtc_reset(struct drm_crtc *crtc);
bool dpu_crtc_is_enabled(struct drm_crtc* crtc);
int dpu_crtc_atomic_enable(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state);
int dpu_crtc_atomic_disable(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state);
int dpu_enable_vblank(struct dpu_crtc *crtc);
void dpu_disable_vblank(struct dpu_crtc *crtc);
int dpu_enable_irq(struct dpu_crtc *crtc, int irq);
void dpu_disable_irq(struct dpu_crtc *crtc, int irq);
int dpu_crtc_atomic_check(struct drm_crtc *crtc,
	struct drm_crtc_state *crtc_state);
void dpu_crtc_atomic_begin(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state);
bool dpu_crtc_get_shdld_status(struct drm_crtc *crtc);
int dpu_crtc_atomic_flush(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state);
void dpu_crtc_mode_set_nofb(struct drm_crtc *crtc,
	struct drm_encoder *encoder);
int dpu_crtc_probe(struct platform_device *pdev, struct dpu_crtc *crtc);
int dpu_crtc_remove(struct platform_device *pdev);

/* Plane */
void dpu_plane_destroy(struct drm_plane *plane);
void dpu_plane_reset(struct drm_plane *plane);
void dpu_drm_atomic_plane_destroy_state(struct drm_plane *plane,
	struct drm_plane_state *state);
int dpu_plane_atomic_check(struct drm_plane *plane,
	struct drm_plane_state *state);
int dpu_plane_atomic_update(struct drm_plane *plane,
	struct drm_plane_state *old_state);
void dpu_plane_atomic_page_flip(struct drm_plane *plane,
	struct drm_plane_state *old_state);

/* KMS */
int dpu_drm_atomic_check(struct drm_device *dev,
	struct drm_crtc **crtc_list, int list_cnt);

#endif				/* __DPU_PRV_H__ */
