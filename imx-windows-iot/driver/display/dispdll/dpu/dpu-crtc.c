/*
 * Copyright 2017-2020, 2022-2023 NXP
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

#include <drm/drm_atomic.h>
#include <drm/drm_encoder.h>
#include <drm/drm_print.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/preempt.h>
#include <video/dpu.h>
#include <video/imx8-pc.h>
#include <video/imx8-prefetch.h>
#include "dpu-crc.h"
#include "dpu-crtc.h"
#include "dpu-plane.h"
#include "dpu-prv.h"
#include "drm/imx/imx-drm.h"

int dpu_crtc_copy_display_mode(struct drm_display_mode* src_mode,
	struct drm_display_mode* dst_mode)
{

	if (!src_mode || !dst_mode)
		return -1;

	dst_mode->hdisplay = src_mode->hdisplay;
	dst_mode->hsync_start = src_mode->hsync_start;
	dst_mode->hsync_end = src_mode->hsync_end;
	dst_mode->htotal = src_mode->htotal;
	dst_mode->hskew = src_mode->hskew;
	dst_mode->vdisplay = src_mode->vdisplay;
	dst_mode->vsync_start = src_mode->vsync_start;
	dst_mode->vsync_end = src_mode->vsync_end;
	dst_mode->vtotal = src_mode->vtotal;
	dst_mode->vscan = src_mode->vscan;

	dst_mode->clock = src_mode->clock;

	dst_mode->flags = src_mode->flags;

	dst_mode->crtc_clock = src_mode->crtc_clock;
	dst_mode->crtc_hdisplay= src_mode->crtc_hdisplay;
	dst_mode->crtc_hblank_start = src_mode->crtc_hblank_start;
	dst_mode->crtc_hblank_end = src_mode->crtc_hblank_end;
	dst_mode->crtc_hsync_start = src_mode->crtc_hsync_start;
	dst_mode->crtc_hsync_end = src_mode->crtc_hsync_end;
	dst_mode->crtc_htotal = src_mode->crtc_htotal;
	dst_mode->crtc_hskew = src_mode->crtc_hskew;
	dst_mode->crtc_vdisplay = src_mode->crtc_vdisplay;
	dst_mode->crtc_vblank_start = src_mode->crtc_vblank_start;
	dst_mode->crtc_vblank_end = src_mode->crtc_vblank_end;
	dst_mode->crtc_vsync_start = src_mode->crtc_vsync_start;
	dst_mode->crtc_vsync_end = src_mode->crtc_vsync_end;
	dst_mode->crtc_vtotal = src_mode->crtc_vtotal;

	dst_mode->picture_aspect_ratio = src_mode->picture_aspect_ratio;

	return 0;
}

static inline struct dpu_plane_state **
alloc_dpu_plane_states(struct dpu_crtc *dpu_crtc)
{
	struct dpu_plane_state **states;

	states = kcalloc(dpu_crtc->hw_plane_num, sizeof(*states), GFP_KERNEL);
	if (!states)
		return ERR_PTR(-ENOMEM);

	return states;
}

bool dpu_crtc_is_enabled(struct drm_crtc* crtc)
{
	struct dpu_crtc* dpu_crtc = to_dpu_crtc(crtc);
	struct dpu_soc* dpu = dev_get_drvdata(dpu_crtc->dev->parent);
	int disp_id = dpu_crtc->stream_id;

	return dpu_disp_is_enabled(dpu, disp_id);
}


struct dpu_crtc *dpu_crtc_get_aux_dpu_crtc(struct dpu_crtc *dpu_crtc)
{

	return dpu_crtc->aux_dpu_crtc;
}

int dpu_crtc_atomic_enable(struct drm_crtc *crtc,
	struct drm_crtc_state *old_crtc_state)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct dpu_soc* dpu = dev_get_drvdata(dpu_crtc->dev->parent);
	struct dpu_crtc *aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct dpu_plane *dplane = to_dpu_plane(crtc->primary);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_extdst *plane_ed = res->ed[dplane->stream_id];
	struct dpu_extdst *aux_plane_ed = dpu_aux_ed_peek(plane_ed);
	struct dpu_extdst *m_plane_ed = NULL, *s_plane_ed;
	struct completion *shdld_done;
	struct completion *m_safety_shdld_done, *s_safety_shdld_done;
	struct completion *m_content_shdld_done, *s_content_shdld_done;
	struct completion *m_dec_shdld_done, *s_dec_shdld_done;
	unsigned long ret;
	unsigned char flags;

	if (dcstate->use_pc) {
		tcon_enable_pc(dpu_crtc->tcon);

		if (extdst_is_master(plane_ed)) {
			m_plane_ed = plane_ed;
			s_plane_ed = aux_plane_ed;
		} else {
			m_plane_ed = aux_plane_ed;
			s_plane_ed = plane_ed;
		}
		extdst_pixengcfg_syncmode_master(m_plane_ed, true);
		extdst_pixengcfg_syncmode_master(s_plane_ed, false);
	} else {
		extdst_pixengcfg_syncmode_master(plane_ed, false);
	}

	dpu_enable_irq(dpu_crtc, dpu_crtc->safety_shdld_irq);
	dpu_enable_irq(dpu_crtc, dpu_crtc->content_shdld_irq);
	dpu_enable_irq(dpu_crtc, dpu_crtc->dec_shdld_irq);
	if (dcstate->use_pc) {
		dpu_enable_irq(aux_dpu_crtc, aux_dpu_crtc->safety_shdld_irq);
		dpu_enable_irq(aux_dpu_crtc, aux_dpu_crtc->content_shdld_irq);
		dpu_enable_irq(aux_dpu_crtc, aux_dpu_crtc->dec_shdld_irq);
	}

	if (dcstate->use_pc) {
		framegen_enable_clock(dpu_crtc->stream_id ?
					dpu_crtc->aux_fg : dpu_crtc->fg);
		extdst_pixengcfg_sync_trigger(m_plane_ed);
		framegen_shdtokgen(dpu_crtc->m_fg);

		/* don't relinquish CPU until TCONs are set to operation mode */
		preempt_disable(&flags);
		/* First turn on the slave stream, second the master stream. */
		framegen_enable(dpu_crtc->s_fg);
		framegen_enable(dpu_crtc->m_fg);
		/*
		 * TKT320590:
		 * Turn TCONs into operation mode as soon as the first dumb
		 * frame is generated by DPU from the master stream(we don't
		 * relinquish CPU to ensure this).  This makes DPRs/PRGs of
		 * the dual stream be able to evade the dumb frames of the
		 * dual stream respectively.
		 */
		if (framegen_wait_for_frame_counter_moving(dpu_crtc->m_fg)) {
			preempt_enable(flags);
			return -EINVAL;
		}

		/* again, slave first, then master */
		tcon_set_operation_mode(dpu_crtc->s_tcon);
		tcon_set_operation_mode(dpu_crtc->m_tcon);
		preempt_enable(flags);

		framegen_enable_pixel_link(dpu_crtc->s_fg);
		framegen_enable_pixel_link(dpu_crtc->m_fg);

		if (dpu_crtc->aux_is_master) {
			m_safety_shdld_done  = &aux_dpu_crtc->safety_shdld_done;
			m_content_shdld_done = &aux_dpu_crtc->content_shdld_done;
			m_dec_shdld_done     = &aux_dpu_crtc->dec_shdld_done;
			s_safety_shdld_done  = &dpu_crtc->safety_shdld_done;
			s_content_shdld_done = &dpu_crtc->content_shdld_done;
			s_dec_shdld_done     = &dpu_crtc->dec_shdld_done;
		} else {
			m_safety_shdld_done  = &dpu_crtc->safety_shdld_done;
			m_content_shdld_done = &dpu_crtc->content_shdld_done;
			m_dec_shdld_done     = &dpu_crtc->dec_shdld_done;
			s_safety_shdld_done  = &aux_dpu_crtc->safety_shdld_done;
			s_content_shdld_done = &aux_dpu_crtc->content_shdld_done;
			s_dec_shdld_done     = &aux_dpu_crtc->dec_shdld_done;
		}

		ret = wait_for_completion_timeout(m_safety_shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC] %s: wait for master safety shdld done timeout\n",
				__func__);
		ret = wait_for_completion_timeout(m_content_shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC] %s: wait for master content shdld done timeout\n",
					__func__);
		ret = wait_for_completion_timeout(m_dec_shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC] %s: wait for master DEC shdld done timeout\n",
					__func__);

		ret = wait_for_completion_timeout(s_safety_shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC] %s: wait for slave safety shdld done timeout\n",
					__func__);
		ret = wait_for_completion_timeout(s_content_shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC] %s: wait for slave content shdld done timeout\n",
					__func__);
		ret = wait_for_completion_timeout(s_dec_shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC] %s: wait for slave DEC shdld done timeout\n",
					__func__);
	} else {
		framegen_enable_clock(dpu_crtc->fg);
		extdst_pixengcfg_sync_trigger(plane_ed);
		extdst_pixengcfg_sync_trigger(dpu_crtc->ed);
		framegen_shdtokgen(dpu_crtc->fg);

		/* don't relinquish CPU until TCON is set to operation mode */
		preempt_disable(&flags);
		framegen_enable(dpu_crtc->fg);
		/*
		 * TKT320590:
		 * Turn TCON into operation mode as soon as the first dumb
		 * frame is generated by DPU(we don't relinquish CPU to ensure
		 * this).  This makes DPR/PRG be able to evade the frame.
		 */
		if (framegen_wait_for_frame_counter_moving(dpu_crtc->fg)) {
			preempt_enable(flags);
			return -EINVAL;
		}

		tcon_set_operation_mode(dpu_crtc->tcon);
		preempt_enable(flags);

		framegen_enable_pixel_link(dpu_crtc->fg);

		shdld_done = &dpu_crtc->safety_shdld_done;
		ret = wait_for_completion_timeout(shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC:] %s: wait for safety shdld done timeout\n",
					__func__);
		shdld_done = &dpu_crtc->content_shdld_done;
		ret = wait_for_completion_timeout(shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC:] %s: wait for content shdld done timeout\n",
					__func__);
		shdld_done = &dpu_crtc->dec_shdld_done;
		ret = wait_for_completion_timeout(shdld_done, HZ);
		if (ret == 0)
			DRM_WARN("[CRTC:] %s: wait for DEC shdld done timeout\n",
					__func__);
	}

	dpu_disable_irq(dpu_crtc, dpu_crtc->safety_shdld_irq);
	dpu_disable_irq(dpu_crtc, dpu_crtc->content_shdld_irq);
	dpu_disable_irq(dpu_crtc, dpu_crtc->dec_shdld_irq);
	if (dcstate->use_pc) {
		dpu_disable_irq(aux_dpu_crtc, aux_dpu_crtc->safety_shdld_irq);
		dpu_disable_irq(aux_dpu_crtc, aux_dpu_crtc->content_shdld_irq);
		dpu_disable_irq(aux_dpu_crtc, aux_dpu_crtc->dec_shdld_irq);
	}

	if (dcstate->use_pc) {
		framegen_wait_for_secondary_syncup(dpu_crtc->m_fg);
		framegen_wait_for_secondary_syncup(dpu_crtc->s_fg);

		if (framegen_secondary_requests_to_read_empty_fifo(dpu_crtc->m_fg)) {
			framegen_secondary_clear_channel_status(dpu_crtc->m_fg);
			DRM_WARN("[CRTC] %s: master FrameGen requests to read empty FIFO\n",
					__func__);
		}
		if (framegen_secondary_requests_to_read_empty_fifo(dpu_crtc->s_fg)) {
			framegen_secondary_clear_channel_status(dpu_crtc->s_fg);
			DRM_WARN("[CRTC] %s: slave FrameGen requests to read empty FIFO\n",
					__func__);
		}
	} else {
		framegen_wait_for_secondary_syncup(dpu_crtc->fg);

		if (framegen_secondary_requests_to_read_empty_fifo(dpu_crtc->fg)) {
			framegen_secondary_clear_channel_status(dpu_crtc->fg);
			DRM_WARN("[CRTC] %s: FrameGen requests to read empty FIFO\n",
					__func__);
		}
	}

	if (dcstate->crc.source != DPU_CRC_SRC_NONE)
		dpu_crtc_enable_crc_source(crtc,
				dcstate->crc.source, &dcstate->crc.roi);

	return 0;
}

int dpu_crtc_atomic_disable(struct drm_crtc *crtc,
	struct drm_crtc_state* old_crtc_state)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct imx_crtc_state *imx_crtc_state =
					to_imx_crtc_state(old_crtc_state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct drm_display_mode *adjusted_mode = &old_crtc_state->adjusted_mode;
	struct dpu_plane *dplane = to_dpu_plane(crtc->primary);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_plane_state *dpstate;
	struct dpu_fetchunit *fu;
	int i;
	unsigned char flags;

	if (dcstate->crc.source != DPU_CRC_SRC_NONE)
		dpu_crtc_disable_crc_source(crtc, dcstate->use_pc);

	if (dcstate->use_pc) {
		tcon_disable_pc(dpu_crtc->tcon);

		framegen_disable_pixel_link(dpu_crtc->m_fg);
		framegen_disable_pixel_link(dpu_crtc->s_fg);

		/* don't relinquish CPU until DPRC repeat_en is disabled */
		preempt_disable(&flags);
		/*
		 * Sync to FrameGen frame counter moving so that
		 * FrameGen can be disabled in the next frame.
		 */
		if (framegen_wait_for_frame_counter_moving(dpu_crtc->m_fg)) {
			preempt_enable(flags);
			return -EINVAL;
		}

		/* First turn off the master stream, second the slave stream. */
		framegen_disable(dpu_crtc->m_fg);
		framegen_disable(dpu_crtc->s_fg);

		/*
		 * There is one frame leftover after FrameGen disablement.
		 * Sync to FrameGen frame counter moving so that
		 * DPRC repeat_en can be disabled in the next frame.
		 */
		if (framegen_wait_for_frame_counter_moving(dpu_crtc->m_fg)) {
			preempt_enable(flags);
			return -EINVAL;
		}

		for (i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
			dpu_block_id_t source;
			bool aux_source_flag;
			bool use_prefetch;

			dpstate = dcstate->dpu_plane_states[i];
			if (!dpstate)
				continue;

			aux_source_flag = false;
again:
			source = aux_source_flag ? dpstate->aux_source :
						   dpstate->source;
			use_prefetch = aux_source_flag ?
						dpstate->use_aux_prefetch :
						dpstate->use_prefetch;
			fu = source_to_fu(res, source);
			if (!fu) {
				preempt_enable(flags);
				return -EINVAL;
			}

			if (fu->dprc && use_prefetch)
				dprc_disable_repeat_en(fu->dprc);

			if (dpstate->need_aux_source && !aux_source_flag) {
				aux_source_flag = true;
				goto again;
			}
		}
		preempt_enable(flags);

		if (framegen_wait_done(dpu_crtc->m_fg, adjusted_mode))
			return -EINVAL;
		if (framegen_wait_done(dpu_crtc->s_fg, adjusted_mode))
			return -EINVAL;

		framegen_disable_clock(dpu_crtc->stream_id ?
					dpu_crtc->aux_fg : dpu_crtc->fg);
	} else {
		framegen_disable_pixel_link(dpu_crtc->fg);

		/* don't relinquish CPU until DPRC repeat_en is disabled */
		preempt_disable(&flags);
		/*
		 * Sync to FrameGen frame counter moving so that
		 * FrameGen can be disabled in the next frame.
		 */
		if (framegen_wait_for_frame_counter_moving(dpu_crtc->fg)) {
			preempt_enable(flags);
			return -EINVAL;
		}

		framegen_disable(dpu_crtc->fg);
		/*
		 * There is one frame leftover after FrameGen disablement.
		 * Sync to FrameGen frame counter moving so that
		 * DPRC repeat_en can be disabled in the next frame.
		 */
		if (framegen_wait_for_frame_counter_moving(dpu_crtc->fg)) {
			preempt_enable(flags);
			return -EINVAL;
		}

		for (i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
			dpstate = dcstate->dpu_plane_states[i];
			if (!dpstate)
				continue;

			fu = source_to_fu(res, dpstate->source);
			if (!fu) {
				preempt_enable(flags);
				return -EINVAL;
			}

			if (fu->dprc && dpstate->use_prefetch)
				dprc_disable_repeat_en(fu->dprc);
		}
		preempt_enable(flags);

		if (framegen_wait_done(dpu_crtc->fg, adjusted_mode))
			return -EINVAL;

		framegen_disable_clock(dpu_crtc->fg);
	}

	return 0;
}

void dpu_drm_crtc_reset(struct drm_crtc *crtc)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct imx_crtc_state *imx_crtc_state;
	struct dpu_crtc_state *state;

	if (!crtc->state)
		return;

	imx_crtc_state = to_imx_crtc_state(crtc->state);
	state = to_dpu_crtc_state(imx_crtc_state);

	state->crc.source = DPU_CRC_SRC_NONE;
	state->crc.roi.x1 = 0;
	state->crc.roi.y1 = 0;
	state->crc.roi.x2 = 0;
	state->crc.roi.y2 = 0;

	state->dpu_plane_states = alloc_dpu_plane_states(dpu_crtc);
	if (IS_ERR(state->dpu_plane_states))
		DRM_DEV_ERROR(dev, "Allocating dou plane states failed.\n");
}

int dpu_enable_irq(struct dpu_crtc* crtc, int irq)
{
	struct dpu_soc* dpu = dev_get_drvdata(crtc->dev->parent);

	dpu_irq_unmask(dpu, irq);
	enable_irq(irq);

	return 0;
}

void dpu_disable_irq(struct dpu_crtc* crtc, int irq)
{
	struct dpu_soc* dpu = dev_get_drvdata(crtc->dev->parent);

	dpu_irq_mask(dpu, irq);
	disable_irq_nosync(irq);
}

int dpu_enable_vblank(struct dpu_crtc* crtc)
{
	struct dpu_soc* dpu = dev_get_drvdata(crtc->dev->parent);

	dpu_irq_unmask(dpu, crtc->vbl_irq);
	enable_irq(crtc->vbl_irq);

	return 0;
}

void dpu_disable_vblank(struct dpu_crtc *crtc)
{
	struct dpu_soc* dpu = dev_get_drvdata(crtc->dev->parent);

	dpu_irq_mask(dpu, crtc->vbl_irq);
	disable_irq_nosync(crtc->vbl_irq);
}

static irqreturn_t dpu_safety_shdld_irq_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;

	complete(&dpu_crtc->safety_shdld_done);

	return IRQ_HANDLED;
}

static irqreturn_t dpu_content_shdld_irq_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;

	dpu_crtc->_content_shdld_done = true;

	complete(&dpu_crtc->content_shdld_done);

	return IRQ_HANDLED;
}

static irqreturn_t dpu_dec_shdld_irq_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;

	complete(&dpu_crtc->dec_shdld_done);

	return IRQ_HANDLED;
}

static irqreturn_t dpu_crc_shdld_irq_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;

	complete(&dpu_crtc->crc_shdld_done);

	return IRQ_HANDLED;
}

int dpu_crtc_atomic_check(struct drm_crtc *crtc,
				 struct drm_crtc_state *crtc_state)
{
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	struct dpu_plane_state *dpstate;
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct imx_crtc_state *old_imx_crtc_state =
					to_imx_crtc_state(crtc->state);
	struct dpu_crtc_state *old_dcstate =
					to_dpu_crtc_state(old_imx_crtc_state);
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	struct videomode vm;
	unsigned long encoder_type = DRM_MODE_ENCODER_NONE;
	int i = 0;

	if (crtc_state->enable && dcstate->use_pc) {
		if (encoder_type != DRM_MODE_ENCODER_TMDS) {
			DRM_DEBUG_KMS("[CRTC:] enc type %lu doesn't support pc\n",
					encoder_type);
			return -EINVAL;
		}

		drm_display_mode_to_videomode(mode, &vm);
		if ((vm.hactive % 2)   || (vm.hfront_porch % 2) ||
		    (vm.hsync_len % 2) || (vm.hback_porch % 2)) {
			DRM_DEBUG_KMS("[CRTC:] video mode is invalid\n");
			return -EINVAL;
		}
	}

	/* disallow to enable CRC when CRTC keeps at inactive status */
	if (!crtc->state->active && !crtc_state->enable &&
	    to_enable_dpu_crc(dcstate, old_dcstate))
		return -EINVAL;

	if (crtc_state->enable && dcstate->crc.source == DPU_CRC_SRC_FRAMEGEN) {
		dcstate->crc.roi.x1 = 0;
		dcstate->crc.roi.y1 = 0;
		dcstate->crc.roi.x2 = mode->hdisplay;
		dcstate->crc.roi.y2 = mode->vdisplay;
	}

	if (crtc_state->enable && dcstate->crc.source != DPU_CRC_SRC_NONE) {
		if (dcstate->crc.roi.x1 < 0 || dcstate->crc.roi.y1 < 0)
			return -EINVAL;

		if (dcstate->crc.roi.x2 > mode->hdisplay ||
		    dcstate->crc.roi.y2 > mode->vdisplay)
			return -EINVAL;

		if (!drm_rect_visible(&dcstate->crc.roi))
			return -EINVAL;
	}

	/*
	 * cache the plane states so that the planes can be disabled in
	 * ->atomic_begin.
	 */
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);

	for (i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
		plane = &dpu_crtc->plane[i]->base;
		plane_state = dpu_crtc->plane[i]->base.state;

		if (!is_plane_attached_to_crtc(dpu_crtc->base.state, plane))
			continue;

		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);

		dpstate = to_dpu_plane_state(plane_state);
		dcstate->dpu_plane_states[i++] = dpstate;
	}

	return 0;
}

void dpu_crtc_atomic_begin(struct drm_crtc *crtc,
				  struct drm_crtc_state *old_crtc_state)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct imx_crtc_state *old_imx_crtc_state =
					to_imx_crtc_state(old_crtc_state);
	struct dpu_crtc_state *old_dcstate =
					to_dpu_crtc_state(old_imx_crtc_state);
	int i;

	/*
	 * Disable all planes' resources in SHADOW only.
	 * Whether any of them would be disabled or kept running depends
	 * on new plane states' commit.
	 */
	for (i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
		struct dpu_plane_state *old_dpstate;
		struct drm_plane_state *plane_state;
		struct dpu_plane *dplane;
		struct dpu_plane_res *res;
		struct dpu_fetchunit *fu;
		struct dpu_fetchunit *fe = NULL;
		struct dpu_hscaler *hs = NULL;
		struct dpu_vscaler *vs = NULL;
		struct dpu_layerblend *lb;
		struct dpu_extdst *ed;
		extdst_src_sel_t ed_src;
		dpu_block_id_t blend, source;
		unsigned int stream_id;
		int lb_id;
		bool release_aux_source;

		old_dpstate = old_dcstate->dpu_plane_states[i];
		if (!old_dpstate)
			continue;

		plane_state = &old_dpstate->base;
		dplane = to_dpu_plane(plane_state->plane);
		res = &dplane->grp->res;

		release_aux_source = false;
again:
		if (old_dcstate->use_pc) {
			if (release_aux_source) {
				source = old_dpstate->aux_source;
				blend = old_dpstate->aux_blend;
				stream_id = 1;
			} else {
				source = old_dpstate->source;
				blend = old_dpstate->blend;
				stream_id = old_dpstate->left_src_w ? 0 : 1;
			}
		} else {
			source = old_dpstate->source;
			blend = old_dpstate->blend;
			stream_id = dplane->stream_id;
		}

		fu = source_to_fu(res, source);
		if (!fu)
			return;

		lb_id = blend_to_id(blend);
		if (lb_id < 0)
			return;

		lb = res->lb[lb_id];

		layerblend_pixengcfg_clken(lb, CLKEN__DISABLE);
		if (fetchunit_is_fetchdecode(fu)) {
			fe = fetchdecode_get_fetcheco(fu);
			hs = fetchdecode_get_hscaler(fu);
			vs = fetchdecode_get_vscaler(fu);
			hscaler_pixengcfg_clken(hs, CLKEN__DISABLE);
			vscaler_pixengcfg_clken(vs, CLKEN__DISABLE);
			hscaler_mode(hs, SCALER_NEUTRAL);
			vscaler_mode(vs, SCALER_NEUTRAL);
		}
		if ((!old_dcstate->use_pc && old_dpstate->is_top) ||
		     (old_dcstate->use_pc &&
		      ((!stream_id && old_dpstate->is_left_top) ||
			(stream_id && old_dpstate->is_right_top)))) {
			ed = res->ed[stream_id];
			ed_src = stream_id ?
				ED_SRC_CONSTFRAME1 : ED_SRC_CONSTFRAME0;
			extdst_pixengcfg_src_sel(ed, ed_src);
		}

		fu->ops->disable_src_buf(fu);
		if (fetchunit_is_fetchdecode(fu)) {
			fetchdecode_pixengcfg_dynamic_src_sel(fu,
							FD_SRC_DISABLE);
			fe->ops->disable_src_buf(fe);
		}

		if (old_dpstate->need_aux_source && !release_aux_source) {
			release_aux_source = true;
			goto again;
		}
	}
}

bool dpu_crtc_get_shdld_status(struct drm_crtc* crtc)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	return dpu_crtc->_content_shdld_done;
}

int dpu_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_crtc_state *old_crtc_state)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc), *aux_dpu_crtc = NULL;
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct imx_crtc_state *old_imx_crtc_state =
					to_imx_crtc_state(old_crtc_state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct dpu_crtc_state *old_dcstate =
					to_dpu_crtc_state(old_imx_crtc_state);
	struct dpu_plane *dplane = to_dpu_plane(crtc->primary);
	struct dpu_plane_state *old_dpstate;
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_extdst *ed = res->ed[dplane->stream_id], *aux_ed;
	struct dpu_fetchunit *fu;
	dpu_block_id_t source;
	struct completion *shdld_done;
	struct completion *m_content_shdld_done = NULL;
	struct completion *s_content_shdld_done = NULL;
	unsigned long ret;
	int i;
	/* do not request full modeset here because
	 we call atomic flush only for page flip */
	bool need_modeset = false;
	bool need_wait4fgfcm = false, need_aux_wait4fgfcm = false;
	bool use_prefetch;

	/*
	 * Scan over old plane fetchunits to determine if we
	 * need to wait for FrameGen frame counter moving in
	 * the next loop prior to DPRC repeat_en disablement
	 * or not.
	 */
	for (i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
		bool aux_source_flag;

		old_dpstate = old_dcstate->dpu_plane_states[i];
		if (!old_dpstate)
			continue;

		aux_source_flag = false;
again1:
		source = aux_source_flag ?
				old_dpstate->aux_source : old_dpstate->source;
		use_prefetch = aux_source_flag ?
					old_dpstate->use_aux_prefetch :
					old_dpstate->use_prefetch;
		fu = source_to_fu(res, source);
		if (!fu)
			return -EINVAL;

		if (!fu->ops->is_enabled(fu) && use_prefetch && !need_modeset) {
			if (aux_source_flag)
				need_aux_wait4fgfcm = true;
			else
				need_wait4fgfcm = true;
		}

		if (old_dpstate->need_aux_source && !aux_source_flag) {
			aux_source_flag = true;
			goto again1;
		}
	}

	/*
	 * Sync with FrameGen frame counter moving so that
	 * we may disable DPRC repeat_en correctly.
	 * FIXME: to disable preemption and irq to make sure
	 *        DPRC repeat_en will be disabled ASAP.
	 */
	if (need_wait4fgfcm || need_aux_wait4fgfcm)
		if (framegen_wait_for_frame_counter_moving(dcstate->use_pc ?
			dpu_crtc->m_fg : dpu_crtc->fg))
				return -EINVAL;

	for (i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
		struct dpu_fetchunit *fe;
		struct dpu_hscaler *hs;
		struct dpu_vscaler *vs;
		bool aux_source_disable;

		old_dpstate = old_dcstate->dpu_plane_states[i];
		if (!old_dpstate)
			continue;

		aux_source_disable = false;
again2:
		source = aux_source_disable ?
				old_dpstate->aux_source : old_dpstate->source;
		use_prefetch = aux_source_disable ?
					old_dpstate->use_aux_prefetch :
					old_dpstate->use_prefetch;
		fu = source_to_fu(res, source);
		if (!fu)
			return -EINVAL;

		if (!fu->ops->is_enabled(fu)) {
			fu->ops->set_stream_id(fu, DPU_PLANE_SRC_DISABLED);
			if (fu->dprc && use_prefetch)
				dprc_disable_repeat_en(fu->dprc);
		}

		if (!fetchunit_is_fetchdecode(fu))
			continue;

		fe = fetchdecode_get_fetcheco(fu);
		if (!fe->ops->is_enabled(fe))
			fe->ops->set_stream_id(fe, DPU_PLANE_SRC_DISABLED);

		hs = fetchdecode_get_hscaler(fu);
		if (!hscaler_is_enabled(hs))
			hscaler_set_stream_id(hs, DPU_PLANE_SRC_DISABLED);

		vs = fetchdecode_get_vscaler(fu);
		if (!vscaler_is_enabled(vs))
			vscaler_set_stream_id(vs, DPU_PLANE_SRC_DISABLED);

		if (old_dpstate->need_aux_source && !aux_source_disable) {
			aux_source_disable = true;
			goto again2;
		}
	}

	if (dcstate->use_pc) {
		aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);

		if (dpu_crtc->aux_is_master) {
			m_content_shdld_done = &aux_dpu_crtc->content_shdld_done;
			s_content_shdld_done = &dpu_crtc->content_shdld_done;
		} else {
			m_content_shdld_done = &dpu_crtc->content_shdld_done;
			s_content_shdld_done = &aux_dpu_crtc->content_shdld_done;
		}
	}

	if (!need_modeset) {
		dpu_enable_irq(dpu_crtc, dpu_crtc->content_shdld_irq);
		if (dcstate->use_pc)
			dpu_enable_irq(aux_dpu_crtc, aux_dpu_crtc->content_shdld_irq);

		if (dcstate->use_pc) {
			if (extdst_is_master(ed)) {
				extdst_pixengcfg_sync_trigger(ed);
			} else {
				aux_ed = dpu_aux_ed_peek(ed);
				extdst_pixengcfg_sync_trigger(aux_ed);
			}
		} else {
			dpu_crtc->_content_shdld_done = false;
			extdst_pixengcfg_sync_trigger(ed);
		}

		if (dcstate->use_pc) {
			shdld_done = m_content_shdld_done;
			ret = wait_for_completion_timeout(shdld_done, HZ);
			if (ret == 0)
				DRM_WARN("[CRTC:] %s: wait for master content shdld done timeout\n",
						__func__);

			shdld_done = s_content_shdld_done;
			ret = wait_for_completion_timeout(shdld_done, HZ);
			if (ret == 0)
				DRM_WARN("[CRTC:] %s: wait for slave content shdld done timeout\n",
						__func__);
		} else {
			/*
			* We can't wait here for shadow loads in Windows,
			* instead we should keep shadow load interrupt always enabled and
			* update shadow_load_status flag in the handler
			*/
			/*
			* shdld_done = &dpu_crtc->content_shdld_done;
			* ret = wait_for_completion_timeout(shdld_done, HZ);
			* if (ret == 0)
			*	DRM_WARN("[CRTC:] %s: wait for content shdld done timeout\n",
			*			__func__);
			*/
		}

		/* keep shadow load irq enabled */
		/*
		* dpu_disable_irq(dpu_crtc, dpu_crtc->content_shdld_irq);
		* if (dcstate->use_pc)
		*	disable_irq(aux_dpu_crtc->content_shdld_irq);
		*/

		if (dcstate->use_pc) {
			if (framegen_secondary_requests_to_read_empty_fifo(dpu_crtc->m_fg)) {
				framegen_secondary_clear_channel_status(dpu_crtc->m_fg);
				DRM_WARN("[CRTC:] %s: master FrameGen requests to read empty FIFO\n",
						__func__);
			}
			if (framegen_secondary_requests_to_read_empty_fifo(dpu_crtc->s_fg)) {
				framegen_secondary_clear_channel_status(dpu_crtc->s_fg);
				DRM_WARN("[CRTC:] %s: slave FrameGen requests to read empty FIFO\n",
						__func__);
			}
		} else {
			if (framegen_secondary_requests_to_read_empty_fifo(dpu_crtc->fg)) {
				framegen_secondary_clear_channel_status(dpu_crtc->fg);
				DRM_WARN("[CRTC:] %s: FrameGen requests to read empty FIFO\n",
						__func__);
			}
		}

	} else if (!crtc->state->active) {
		if (old_dcstate->use_pc) {
			if (extdst_is_master(ed)) {
				extdst_pixengcfg_sync_trigger(ed);
			} else {
				aux_ed = dpu_aux_ed_peek(ed);
				extdst_pixengcfg_sync_trigger(aux_ed);
			}
		} else {
			extdst_pixengcfg_sync_trigger(ed);
		}
	}

	return 0;
}

void dpu_crtc_mode_set_nofb(struct drm_crtc *crtc,
	struct drm_encoder *encoder)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct dpu_crtc *aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct dpu_plane *dplane = to_dpu_plane(crtc->primary);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_constframe *pa_cf, *sa_cf;
	struct dpu_disengcfg *dec;
	struct dpu_extdst *ed, *plane_ed;
	struct dpu_framegen *fg;
	struct dpu_tcon *tcon;
	struct dpu_store *st;
	unsigned long encoder_type = DRM_MODE_ENCODER_NONE;
	unsigned int stream_id;
	int crtc_hdisplay = dcstate->use_pc ?
			(mode->crtc_hdisplay >> 1) : mode->crtc_hdisplay;
	extdst_src_sel_t ed_src;
	bool cfg_aux_pipe = false;

	DRM_DEBUG_KMS("[CRTC:] %s: mode->hdisplay: %d\n",
			__func__, mode->hdisplay);
	DRM_DEBUG_KMS("[CRTC:] %s: mode->vdisplay: %d\n",
			__func__, mode->vdisplay);
	DRM_DEBUG_KMS("[CRTC:] %s: mode->clock: %dKHz\n",
			__func__, mode->clock);
	DRM_DEBUG_KMS("[CRTC:] %s: mode->vrefresh: %dHz\n",
			__func__, drm_mode_vrefresh(mode));
	if (dcstate->use_pc)
		DRM_DEBUG_KMS("[CRTC:] %s: use pixel combiner\n",
				__func__);

	encoder_type = encoder->encoder_type;

again:
	if (cfg_aux_pipe) {
		pa_cf = dpu_crtc->aux_pa_cf;
		sa_cf = dpu_crtc->aux_sa_cf;
		dec = dpu_crtc->aux_dec;
		ed = dpu_crtc->aux_ed;
		fg = dpu_crtc->aux_fg;
		tcon = dpu_crtc->aux_tcon;
		st = aux_dpu_crtc->st;
		st = dpu_crtc->st;
		stream_id = dpu_crtc->stream_id ^ 1;
	} else {
		pa_cf = dpu_crtc->pa_cf;
		sa_cf = dpu_crtc->sa_cf;
		dec = dpu_crtc->dec;
		ed = dpu_crtc->ed;
		fg = dpu_crtc->fg;
		tcon = dpu_crtc->tcon;
		st = dpu_crtc->st;
		stream_id = dpu_crtc->stream_id;
	}

	if (dcstate->use_pc) {
		store_pixengcfg_syncmode_fixup(st, true);
		framegen_syncmode_fixup(fg,
				framegen_is_master(fg) ? false : true);
		framegen_syncmode(fg, framegen_is_master(fg) ?
				FGSYNCMODE__MASTER : FGSYNCMODE__SLAVE_ONCE);
	} else {
		store_pixengcfg_syncmode_fixup(st, false);
		framegen_syncmode_fixup(fg, false);
		framegen_syncmode(fg, FGSYNCMODE__OFF);
	}

	framegen_cfg_videomode(fg, mode, dcstate->use_pc, encoder_type);
	framegen_displaymode(fg, FGDM__SEC_ON_TOP);

	framegen_panic_displaymode(fg, FGDM__TEST);

	tcon_cfg_videomode(tcon, mode, dcstate->use_pc);
	tcon_set_fmt(tcon, imx_crtc_state->bus_format);

	if (dcstate->use_pc)
		tcon_configure_pc(tcon, stream_id, mode->crtc_hdisplay,
				dcstate->use_pc ? PC_COMBINE : PC_BYPASS, 0);

	constframe_framedimensions(pa_cf, crtc_hdisplay, mode->crtc_vdisplay);
	constframe_framedimensions(sa_cf, crtc_hdisplay, mode->crtc_vdisplay);
	constframe_constantcolor(sa_cf, 0x0, 0x0, 0x0, 0);

	ed_src = stream_id ? ED_SRC_CONSTFRAME5 : ED_SRC_CONSTFRAME4;
	extdst_pixengcfg_src_sel(ed, ed_src);

	plane_ed = res->ed[stream_id];
	ed_src = stream_id ? ED_SRC_CONSTFRAME1 : ED_SRC_CONSTFRAME0;
	extdst_pixengcfg_src_sel(plane_ed, ed_src);

	if (dcstate->use_pc && !cfg_aux_pipe) {
		cfg_aux_pipe = true;
		goto again;
	}
}

static void dpu_crtc_put_resources(struct dpu_crtc *dpu_crtc)
{
	if (!IS_ERR_OR_NULL(dpu_crtc->pa_cf))
		dpu_cf_put(dpu_crtc->pa_cf);
	if (!IS_ERR_OR_NULL(dpu_crtc->sa_cf))
		dpu_cf_put(dpu_crtc->sa_cf);
	if (!IS_ERR_OR_NULL(dpu_crtc->dec))
		dpu_dec_put(dpu_crtc->dec);
	if (!IS_ERR_OR_NULL(dpu_crtc->ed))
		dpu_ed_put(dpu_crtc->ed);
	if (!IS_ERR_OR_NULL(dpu_crtc->fg))
		dpu_fg_put(dpu_crtc->fg);
	if (!IS_ERR_OR_NULL(dpu_crtc->sig))
		dpu_sig_put(dpu_crtc->sig);
	if (!IS_ERR_OR_NULL(dpu_crtc->tcon))
		dpu_tcon_put(dpu_crtc->tcon);
}

static int dpu_crtc_get_resources(struct dpu_crtc *dpu_crtc)
{
	struct dpu_soc *dpu = dev_get_drvdata(dpu_crtc->dev->parent);
	unsigned int stream_id = dpu_crtc->stream_id;
	int ret;

	dpu_crtc->pa_cf = dpu_cf_get(dpu, stream_id + 4);
	if (IS_ERR(dpu_crtc->pa_cf)) {
		ret = PTR_ERR(dpu_crtc->pa_cf);
		goto err_out;
	}
	dpu_crtc->aux_pa_cf = dpu_aux_cf_peek(dpu_crtc->pa_cf);

	dpu_crtc->sa_cf = dpu_cf_get(dpu, stream_id);
	if (IS_ERR(dpu_crtc->sa_cf)) {
		ret = PTR_ERR(dpu_crtc->sa_cf);
		goto err_out;
	}
	dpu_crtc->aux_sa_cf = dpu_aux_cf_peek(dpu_crtc->sa_cf);

	dpu_crtc->dec = dpu_dec_get(dpu, stream_id);
	if (IS_ERR(dpu_crtc->dec)) {
		ret = PTR_ERR(dpu_crtc->dec);
		goto err_out;
	}
	dpu_crtc->aux_dec = dpu_aux_dec_peek(dpu_crtc->dec);

	dpu_crtc->ed = dpu_ed_get(dpu, stream_id + 4);
	if (IS_ERR(dpu_crtc->ed)) {
		ret = PTR_ERR(dpu_crtc->ed);
		goto err_out;
	}
	dpu_crtc->aux_ed = dpu_aux_ed_peek(dpu_crtc->ed);

	dpu_crtc->fg = dpu_fg_get(dpu, stream_id);
	if (IS_ERR(dpu_crtc->fg)) {
		ret = PTR_ERR(dpu_crtc->fg);
		goto err_out;
	}
	dpu_crtc->aux_fg = dpu_aux_fg_peek(dpu_crtc->fg);

	dpu_crtc->sig = dpu_sig_get(dpu, stream_id);
	if (IS_ERR(dpu_crtc->sig)) {
		ret = PTR_ERR(dpu_crtc->sig);
		goto err_out;
	}
	dpu_crtc->aux_sig = dpu_aux_sig_peek(dpu_crtc->sig);

	dpu_crtc->tcon = dpu_tcon_get(dpu, stream_id);
	if (IS_ERR(dpu_crtc->tcon)) {
		ret = PTR_ERR(dpu_crtc->tcon);
		goto err_out;
	}
	dpu_crtc->aux_tcon = dpu_aux_tcon_peek(dpu_crtc->tcon);

	if (dpu_crtc->aux_is_master) {
		dpu_crtc->m_pa_cf = dpu_crtc->aux_pa_cf;
		dpu_crtc->m_sa_cf = dpu_crtc->aux_sa_cf;
		dpu_crtc->m_dec  = dpu_crtc->aux_dec;
		dpu_crtc->m_ed   = dpu_crtc->aux_ed;
		dpu_crtc->m_fg   = dpu_crtc->aux_fg;
		dpu_crtc->m_tcon = dpu_crtc->aux_tcon;

		dpu_crtc->s_pa_cf = dpu_crtc->pa_cf;
		dpu_crtc->s_sa_cf = dpu_crtc->sa_cf;
		dpu_crtc->s_dec  = dpu_crtc->dec;
		dpu_crtc->s_ed   = dpu_crtc->ed;
		dpu_crtc->s_fg   = dpu_crtc->fg;
		dpu_crtc->s_tcon = dpu_crtc->tcon;
	} else {
		dpu_crtc->m_pa_cf = dpu_crtc->pa_cf;
		dpu_crtc->m_sa_cf = dpu_crtc->sa_cf;
		dpu_crtc->m_dec  = dpu_crtc->dec;
		dpu_crtc->m_ed   = dpu_crtc->ed;
		dpu_crtc->m_fg   = dpu_crtc->fg;
		dpu_crtc->m_tcon = dpu_crtc->tcon;

		dpu_crtc->s_pa_cf = dpu_crtc->aux_pa_cf;
		dpu_crtc->s_sa_cf = dpu_crtc->aux_sa_cf;
		dpu_crtc->s_dec  = dpu_crtc->aux_dec;
		dpu_crtc->s_ed   = dpu_crtc->aux_ed;
		dpu_crtc->s_fg   = dpu_crtc->aux_fg;
		dpu_crtc->s_tcon = dpu_crtc->aux_tcon;
	}

	return 0;
err_out:
	dpu_crtc_put_resources(dpu_crtc);

	return ret;
}

static int dpu_crtc_init(struct dpu_crtc *dpu_crtc,
	struct dpu_client_platformdata *pdata)
{
	struct dpu_soc *dpu = dev_get_drvdata(dpu_crtc->dev->parent);
	struct device *dev = dpu_crtc->dev;
	struct drm_crtc *crtc = &dpu_crtc->base;
	struct dpu_plane_grp *plane_grp = &pdata->plane_grp;
	unsigned int stream_id = pdata->stream_id;
	int i, ret;

	init_completion(&dpu_crtc->safety_shdld_done);
	init_completion(&dpu_crtc->content_shdld_done);
	init_completion(&dpu_crtc->dec_shdld_done);
	init_completion(&dpu_crtc->crc_shdld_done);
	init_completion(&dpu_crtc->aux_crc_done);

	dpu_crtc->stream_id = stream_id;
	dpu_crtc->crtc_grp_id = pdata->di_grp_id;
	dpu_crtc->hw_plane_num = plane_grp->hw_plane_num;
	dpu_crtc->syncmode_min_prate = dpu_get_syncmode_min_prate(dpu);
	dpu_crtc->singlemode_max_width = dpu_get_singlemode_max_width(dpu);
	dpu_crtc->master_stream_id = dpu_get_master_stream_id(dpu);
	dpu_crtc->aux_is_master = !(dpu_crtc->master_stream_id == stream_id);
	dpu_crtc->st = pdata->st9;

	dpu_crtc->plane = devm_kcalloc(dev, dpu_crtc->hw_plane_num,
					sizeof(*dpu_crtc->plane), GFP_KERNEL);
	if (!dpu_crtc->plane)
		return -ENOMEM;

	ret = dpu_crtc_get_resources(dpu_crtc);
	if (ret) {
		DRM_DEV_ERROR(dev, "getting resources failed with %d.\n", ret);
		return ret;
	}

	plane_grp->res.fg[stream_id] = dpu_crtc->fg;
	dpu_crtc->plane[0] = dpu_plane_create(stream_id, plane_grp,
					DRM_PLANE_TYPE_PRIMARY, 0);
	if (IS_ERR(dpu_crtc->plane[0])) {
		ret = PTR_ERR(dpu_crtc->plane[0]);
		DRM_DEV_ERROR(dev,
				"initializing plane0 failed with %d.\n", ret);
		goto err_put_resources;
	}

	crtc->primary = &dpu_crtc->plane[0]->base;

	for (i = 1; i < (int)dpu_crtc->hw_plane_num; i++) {
		dpu_crtc->plane[i] = dpu_plane_create(stream_id, plane_grp,
					DRM_PLANE_TYPE_OVERLAY, i);
		if (IS_ERR(dpu_crtc->plane[i])) {
			ret = PTR_ERR(dpu_crtc->plane[i]);
			DRM_DEV_ERROR(dev,
				"initializing plane%d failed with %d.\n",
								i, ret);
			goto err_put_resources;
		}
	}

	dpu_crtc->vbl_irq = dpu_map_irq(dpu, stream_id ?
				IRQ_DISENGCFG_FRAMECOMPLETE1 :
				IRQ_DISENGCFG_FRAMECOMPLETE0);
	disable_irq(dpu_crtc->vbl_irq);

	dpu_crtc->safety_shdld_irq = dpu_map_irq(dpu, stream_id ?
			IRQ_EXTDST5_SHDLOAD : IRQ_EXTDST4_SHDLOAD);
	ret = devm_request_irq(dev, dpu_crtc->safety_shdld_irq,
				dpu_safety_shdld_irq_handler, 0, "imx_drm",
				dpu_crtc);
	if (ret < 0) {
		DRM_DEV_ERROR(dev,
			"safety shadow load irq request failed with %d.\n",
									ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->safety_shdld_irq);

	dpu_crtc->content_shdld_irq = dpu_map_irq(dpu, stream_id ?
			IRQ_EXTDST1_SHDLOAD : IRQ_EXTDST0_SHDLOAD);
	ret = devm_request_irq(dev, dpu_crtc->content_shdld_irq,
				dpu_content_shdld_irq_handler, 0, "imx_drm",
				dpu_crtc);
	if (ret < 0) {
		DRM_DEV_ERROR(dev,
			"content shadow load irq request failed with %d.\n",
									ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->content_shdld_irq);

	dpu_crtc->dec_shdld_irq = dpu_map_irq(dpu, stream_id ?
			IRQ_DISENGCFG_SHDLOAD1 : IRQ_DISENGCFG_SHDLOAD0);
	ret = devm_request_irq(dev, dpu_crtc->dec_shdld_irq,
				dpu_dec_shdld_irq_handler, 0, "imx_drm",
				dpu_crtc);
	if (ret < 0) {
		DRM_DEV_ERROR(dev,
			"DEC shadow load irq request failed with %d.\n",
									ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->dec_shdld_irq);

	dpu_crtc->crc_valid_irq = dpu_map_irq(dpu, stream_id ?
					IRQ_SIG1_VALID : IRQ_SIG0_VALID);
	ret = devm_request_threaded_irq(dev, dpu_crtc->crc_valid_irq, NULL,
					dpu_crc_valid_irq_threaded_handler,
					IRQF_ONESHOT, "imx_drm", dpu_crtc);
	if (ret < 0) {
		dev_err(dev,
			"CRC valid irq request failed with %d.\n",
			ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->crc_valid_irq);

	dpu_crtc->crc_shdld_irq = dpu_map_irq(dpu, stream_id ?
					IRQ_SIG1_SHDLOAD : IRQ_SIG0_SHDLOAD);
	ret = devm_request_irq(dev, dpu_crtc->crc_shdld_irq,
				dpu_crc_shdld_irq_handler, 0, "imx_drm",
				dpu_crtc);
	if (ret < 0) {
		dev_err(dev,
			"CRC shadow load irq request failed with %d.\n",
			ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->crc_shdld_irq);

	return 0;

err_put_resources:
	dpu_crtc_put_resources(dpu_crtc);

	return ret;
}

static int dpu_crtc_bind(struct device *dev)
{
	struct dpu_client_platformdata *pdata = dev->platform_data;
	struct dpu_crtc *dpu_crtc = dev_get_drvdata(dev);
	int ret;

	dpu_crtc->dev = dev;

	ret = dpu_crtc_init(dpu_crtc, pdata);
	if (ret)
		return ret;

	return 0;
}

static void dpu_crtc_unbind(struct device *dev)
{
	struct dpu_crtc *dpu_crtc = dev_get_drvdata(dev);
	struct imx_crtc_state* imx_crtc_state;
	struct dpu_crtc_state* state;

	if (!dpu_crtc->base.state)
		return;

	imx_crtc_state = to_imx_crtc_state(dpu_crtc->base.state);
	state = to_dpu_crtc_state(imx_crtc_state);

	if (state->dpu_plane_states)
		kfree(state->dpu_plane_states);

	dpu_crtc_put_resources(dpu_crtc);

	for (int i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
		if (dpu_crtc->plane[i]) {
			kfree(dpu_crtc->plane[i]);
		}
	}

	if (dpu_crtc->plane)
		kfree(dpu_crtc->plane);
}

int dpu_crtc_probe(struct platform_device *pdev, struct dpu_crtc *crtc)
{
	struct device *dev = &pdev->dev;

	if (!dev->platform_data)
		return -EINVAL;

	dev_set_drvdata(dev, crtc);

	dpu_crtc_bind(dev);

	return 0;
}

int dpu_crtc_remove(struct platform_device *pdev)
{
	dpu_crtc_unbind(&pdev->dev);
	return 0;
}
