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

#include <asm-generic/bitops/ffs.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_print.h>
#include <drm/drm_atomic.h>
#include <linux/bitops.h>
#include <video/dpu.h>
#include "dpu-crtc.h"
#include "dpu-plane.h"
#include "dpu-prv.h"
#include "drm/imx/imx-drm.h"

static struct drm_plane_state **
dpu_atomic_alloc_tmp_planes_per_crtc(int num_total_plane)
{
	struct drm_plane_state **states;

	states = kmalloc_array(num_total_plane, sizeof(*states), GFP_KERNEL);
	if (!states)
		return ERR_PTR(-ENOMEM);

	return states;
}

static void
dpu_atomic_compute_plane_lrx_per_crtc(struct drm_crtc_state *crtc_state,
				      struct drm_plane_state **states, int n)
{
	struct dpu_plane_state *dpstate;
	struct drm_plane_state *plane_state;
	int i;
	int half_hdisplay = crtc_state->adjusted_mode.hdisplay >> 1;
	bool lo, ro, bo;

	/* compute left/right_crtc_x if pixel combiner is needed */
	for (i = 0; i < n; i++) {
		plane_state = states[i];
		dpstate = to_dpu_plane_state(plane_state);

		lo =  dpstate->left_src_w && !dpstate->right_src_w;
		ro = !dpstate->left_src_w &&  dpstate->right_src_w;
		bo =  dpstate->left_src_w &&  dpstate->right_src_w;

		if (lo || bo) {
			dpstate->left_crtc_x = plane_state->crtc_x;
			dpstate->right_crtc_x = 0;
		} else if (ro) {
			dpstate->left_crtc_x = 0;
			dpstate->right_crtc_x =
					plane_state->crtc_x - half_hdisplay;
		}
	}
}

static void
dpu_atomic_set_top_plane_per_crtc(struct drm_plane_state **states, int n,
				  bool use_pc)
{
	struct dpu_plane_state *dpstate;
	bool found_l_top = false, found_r_top = false;
	int i;

	for (i = n - 1; i >= 0; i--) {
		dpstate = to_dpu_plane_state(states[i]);
		if (use_pc) {
			if (dpstate->left_src_w && !found_l_top) {
				dpstate->is_left_top = true;
				found_l_top = true;
			} else {
				dpstate->is_left_top = false;
			}

			if (dpstate->right_src_w && !found_r_top) {
				dpstate->is_right_top = true;
				found_r_top = true;
			} else {
				dpstate->is_right_top = false;
			}
		} else {
			dpstate->is_top = (i == (n - 1)) ? true : false;
		}
	}
}

static int
dpu_atomic_assign_plane_source_per_crtc(struct drm_plane_state **states,
					int n, bool use_pc)
{
	struct dpu_plane_state *dpstate;
	struct dpu_plane *dplane;
	struct dpu_plane_grp *grp;
	struct drm_framebuffer *fb;
	struct dpu_fetchunit *fu;
	struct dpu_fetchunit *fe;
	struct dpu_hscaler *hs;
	struct dpu_vscaler *vs;
	lb_prim_sel_t stage;
	dpu_block_id_t blend;
	unsigned int sid, src_sid;
	unsigned int num_planes;
	int bit;
	int i, j, k = 0, m;
	int total_asrc_num;
	int s0_layer_cnt = 0, s1_layer_cnt = 0;
	int s0_n = 0, s1_n = 0;
	u32 src_a_mask, cap_mask, fe_mask, hs_mask, vs_mask;
	bool need_fetcheco, need_hscaler, need_vscaler;
	bool fmt_is_yuv;
	bool alloc_aux_source;

	if (use_pc) {
		for (i = 0; i < n; i++) {
			dpstate = to_dpu_plane_state(states[i]);

			if (dpstate->left_src_w)
				s0_n++;

			if (dpstate->right_src_w)
				s1_n++;
		}
	} else {
		s0_n = n;
		s1_n = n;
	}

	/* for active planes only */
	for (i = 0; i < n; i++) {
		dpstate = to_dpu_plane_state(states[i]);
		dplane = to_dpu_plane(states[i]->plane);
		fb = states[i]->fb;
		num_planes = fb->format->num_planes;
		fmt_is_yuv = drm_format_is_yuv(fb->format->format);
		grp = dplane->grp;
		alloc_aux_source = false;

		if (use_pc)
			sid = dpstate->left_src_w ? 0 : 1;
		else
			sid = dplane->stream_id;

again:
		if (alloc_aux_source)
			sid ^= 1;

		need_fetcheco = (num_planes > 1);
		need_hscaler = (states[i]->src_w >> 16 != states[i]->crtc_w);
		need_vscaler = (states[i]->src_h >> 16 != states[i]->crtc_h);

		total_asrc_num = 0;
		src_a_mask = grp->src_a_mask;
		fe_mask = 0;
		hs_mask = 0;
		vs_mask = 0;

		for_each_set_bit(bit, (unsigned long *)&src_a_mask, 32)
			total_asrc_num++;

		/* assign source */
		mutex_lock(&grp->mutex);
		for (j = 0; j < total_asrc_num; j++) {
			k = ffs(src_a_mask) - 1;
			if (k < 0)
				return -EINVAL;

			/* Changed in comparison with original linux driver:
			   dpu plane is hard assigned according to it's index (override k variable),
			   which is selected in GcKmImx8qxpDisplay object in m_PlaneId and plane_mask
			   (see m_crtc.base.state->plane_mask |= drm_plane_mask(&m_crtc.plane[m_PlaneId]->base))
			   m_PlaneId = 0 => fetchlayer - primary for display stream 0
			   m_PlaneId = 1 => fetchwarp - primary for display stream 1
			   Reason: HwCommitVidPn is not called in the same order for all monitors all the time,
			   depends on the resolution. Sometimes Display stream 1 is called first (assigned fetchlayer)
			   and display stream 0 second (assigns fetchwarp), which doesn't work well.
			   This code need to be reviewed/tested for ovelay support
			*/
			k = dplane->base.index;

			fu = source_to_fu(&grp->res, sources[k]);
			if (!fu)
				return -EINVAL;

			/* avoid on-the-fly/hot migration */
			src_sid = fu->ops->get_stream_id(fu);
			/* Disable the check, because the plane is hard-assigned above */
/*
			if (src_sid && src_sid != BIT(sid))
				goto next;
*/

			if (fetchunit_is_fetchdecode(fu)) {
				cap_mask = fetchdecode_get_vproc_mask(fu);

				if (need_fetcheco) {
					fe = fetchdecode_get_fetcheco(fu);

					/* avoid on-the-fly/hot migration */
					src_sid = fu->ops->get_stream_id(fe);
					if (src_sid && src_sid != BIT(sid))
						goto next;

					/* fetch unit has the fetcheco cap? */
					if (!dpu_vproc_has_fetcheco_cap(cap_mask))
						goto next;

					fe_mask =
					   dpu_vproc_get_fetcheco_cap(cap_mask);

					/* fetcheco available? */
					if (grp->src_use_vproc_mask & fe_mask)
						goto next;
				}

				if (need_hscaler) {
					hs = fetchdecode_get_hscaler(fu);

					/* avoid on-the-fly/hot migration */
					src_sid = hscaler_get_stream_id(hs);
					if (src_sid && src_sid != BIT(sid))
						goto next;

					/* fetch unit has the hscale cap */
					if (!dpu_vproc_has_hscale_cap(cap_mask))
						goto next;

					hs_mask =
					     dpu_vproc_get_hscale_cap(cap_mask);

					/* hscaler available? */
					if (grp->src_use_vproc_mask & hs_mask)
						goto next;
				}

				if (need_vscaler) {
					vs = fetchdecode_get_vscaler(fu);

					/* avoid on-the-fly/hot migration */
					src_sid = vscaler_get_stream_id(vs);
					if (src_sid && src_sid != BIT(sid))
						goto next;

					/* fetch unit has the vscale cap? */
					if (!dpu_vproc_has_vscale_cap(cap_mask))
						goto next;

					vs_mask =
					     dpu_vproc_get_vscale_cap(cap_mask);

					/* vscaler available? */
					if (grp->src_use_vproc_mask & vs_mask)
						goto next;
				}
			} else {
				if (fmt_is_yuv || need_fetcheco ||
				    need_hscaler || need_vscaler)
					goto next;
			}

			grp->src_a_mask &= ~BIT(k);
			grp->src_use_vproc_mask |= fe_mask | hs_mask | vs_mask;
			break;
next:
			src_a_mask &= ~BIT(k);
			fe_mask = 0;
			hs_mask = 0;
			vs_mask = 0;
		}
		mutex_unlock(&grp->mutex);

		if (j == total_asrc_num)
			return -EINVAL;

		if (alloc_aux_source)
			dpstate->aux_source = sources[k];
		else
			dpstate->source = sources[k];

		/* assign stage and blend */
		if (sid) {
			m = grp->hw_plane_num - (s1_n - s1_layer_cnt);
			stage = s1_layer_cnt ? stages[m - 1] : cf_stages[sid];
			blend = blends[m];

			s1_layer_cnt++;
		} else {
			stage = s0_layer_cnt ?
				stages[s0_layer_cnt - 1] : cf_stages[sid];
			blend = blends[s0_layer_cnt];

			s0_layer_cnt++;
		}

		if (alloc_aux_source) {
			dpstate->aux_stage = stage;
			dpstate->aux_blend = blend;
		} else {
			dpstate->stage = stage;
			dpstate->blend = blend;
		}

		if (dpstate->need_aux_source && !alloc_aux_source) {
			alloc_aux_source = true;
			goto again;
		}
	}

	return 0;
}

int dpu_drm_atomic_check(struct drm_device *dev,
	struct drm_crtc **crtc_list, int list_cnt)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct drm_plane *plane;
	struct dpu_plane *dpu_plane;
	struct drm_plane_state *plane_state;
	struct dpu_plane_state *dpstate;
	struct drm_framebuffer *fb;
	struct dpu_plane_grp *grp[MAX_DPU_PLANE_GRP];
	int ret, i, grp_id;
	int active_plane[MAX_DPU_PLANE_GRP];
	int active_plane_fetcheco[MAX_DPU_PLANE_GRP];
	int active_plane_hscale[MAX_DPU_PLANE_GRP];
	int active_plane_vscale[MAX_DPU_PLANE_GRP];
	int half_hdisplay = 0;
	bool pipe_states_prone_to_put[MAX_CRTC];
	bool use_pc[MAX_DPU_PLANE_GRP];
	u32 crtc_mask_in_state = 0;

	ret = 0;

	for (i = 0; i < MAX_CRTC; i++)
		pipe_states_prone_to_put[i] = false;

	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		active_plane[i] = 0;
		active_plane_fetcheco[i] = 0;
		active_plane_hscale[i] = 0;
		active_plane_vscale[i] = 0;
		use_pc[i] = false;
		grp[i] = NULL;
	}

	for (i = 0; i < list_cnt; i++)
		crtc_mask_in_state |= drm_crtc_mask(crtc_list[i]);

	for (i = 0; i < list_cnt; i++) {
		crtc = crtc_list[i];
		struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
		struct imx_crtc_state *imx_crtc_state;
		struct dpu_crtc_state *dcstate;
		bool need_left, need_right, need_aux_source, use_pc_per_crtc;

		use_pc_per_crtc = false;

		crtc_state = crtc->state;
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);

		imx_crtc_state = to_imx_crtc_state(crtc_state);
		dcstate = to_dpu_crtc_state(imx_crtc_state);

		if (crtc_state->enable) {
			if (use_pc[dpu_crtc->crtc_grp_id]) {
				DRM_DEBUG_KMS("other crtc needs pixel combiner\n");
				return -EINVAL;
			}

			if ((unsigned int)crtc_state->adjusted_mode.clock >
					dpu_crtc->syncmode_min_prate ||
			    crtc_state->adjusted_mode.hdisplay >
					dpu_crtc->singlemode_max_width)
				use_pc_per_crtc = true;
		}

		if (use_pc_per_crtc) {
			use_pc[dpu_crtc->crtc_grp_id] = true;
			half_hdisplay = crtc_state->adjusted_mode.hdisplay >> 1;
		}

		dcstate->use_pc = use_pc_per_crtc;

		for (i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
			plane = &dpu_crtc->plane[i]->base;
			plane_state = dpu_crtc->plane[i]->base.state;

			if (!is_plane_attached_to_crtc(dpu_crtc->base.state, plane))
				continue;

			if (IS_ERR(plane_state)) {
				DRM_DEBUG_KMS("failed to get plane state\n");
				return PTR_ERR(plane_state);
			}

			dpstate = to_dpu_plane_state(plane_state);
			fb = plane_state->fb;
			dpu_plane = to_dpu_plane(plane);
			grp_id = dpu_plane->grp->id;
			active_plane[grp_id]++;

			need_left = false;
			need_right = false;
			need_aux_source = false;

			if (use_pc_per_crtc) {
				if (plane_state->crtc_x < half_hdisplay)
					need_left = true;

				if (((int)plane_state->crtc_w +
				     plane_state->crtc_x) > half_hdisplay)
					need_right = true;

				if (need_left && need_right) {
					need_aux_source = true;
					active_plane[grp_id]++;
				}
			}

			if (need_left && need_right) {
				dpstate->left_crtc_w = half_hdisplay;
				dpstate->left_crtc_w -= plane_state->crtc_x;

				dpstate->left_src_w = dpstate->left_crtc_w;
			} else if (need_left) {
				dpstate->left_crtc_w = plane_state->crtc_w;
				dpstate->left_src_w = plane_state->src_w >> 16;
			} else {
				dpstate->left_crtc_w = 0;
				dpstate->left_src_w = 0;
			}

			if (need_right && need_left) {
				dpstate->right_crtc_w = plane_state->crtc_x +
					plane_state->crtc_w;
				dpstate->right_crtc_w -= half_hdisplay;

				dpstate->right_src_w = dpstate->right_crtc_w;
			} else if (need_right) {
				dpstate->right_crtc_w = plane_state->crtc_w;
				dpstate->right_src_w = plane_state->src_w >> 16;
			} else {
				dpstate->right_crtc_w = 0;
				dpstate->right_src_w = 0;
			}

			if (fb->format->num_planes > 1) {
				active_plane_fetcheco[grp_id]++;
				if (need_aux_source)
					active_plane_fetcheco[grp_id]++;
			}

			if (plane_state->src_w >> 16 != plane_state->crtc_w) {
				if (use_pc_per_crtc)
					return -EINVAL;

				active_plane_hscale[grp_id]++;
			}

			if (plane_state->src_h >> 16 != plane_state->crtc_h) {
				if (use_pc_per_crtc)
					return -EINVAL;

				active_plane_vscale[grp_id]++;
			}

			if (grp[grp_id] == NULL)
				grp[grp_id] = dpu_plane->grp;

			dpstate->need_aux_source = need_aux_source;
		}
	}

	/* enough resources? */
	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		if (!grp[i])
			continue;

		if (active_plane[i] > (int)grp[i]->hw_plane_num) {
			DRM_DEBUG_KMS("no enough fetch units\n");
			return -EINVAL;
		}

		if (active_plane_fetcheco[i] > (int)grp[i]->hw_plane_fetcheco_num) {
			DRM_DEBUG_KMS("no enough FetchEcos\n");
			return -EINVAL;
		}

		if (active_plane_hscale[i] > (int)grp[i]->hw_plane_hscaler_num) {
			DRM_DEBUG_KMS("no enough Hscalers\n");
			return -EINVAL;
		}

		if (active_plane_vscale[i] > (int)grp[i]->hw_plane_vscaler_num) {
			DRM_DEBUG_KMS("no enough Vscalers\n");
			return -EINVAL;
		}
	}

	/* initialize resource mask */
	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		if (!grp[i])
			continue;

		mutex_lock(&grp[i]->mutex);
		grp[i]->src_a_mask = grp[i]->src_mask;
		grp[i]->src_use_vproc_mask = 0;
		mutex_unlock(&grp[i]->mutex);
	}

	for (i = 0; i < list_cnt; i++) {
		crtc = crtc_list[i];
		crtc_state = crtc->state;
		struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
		struct drm_plane_state **states;
		int n;

		states = dpu_atomic_alloc_tmp_planes_per_crtc(dpu_crtc->hw_plane_num);
		if (IS_ERR(states)) {
			DRM_DEBUG_KMS(
				"[CRTC] cannot alloc plane state ptrs\n");
			return PTR_ERR(states);
		}

		n = 0;
		for (i = 0; i < (int)dpu_crtc->hw_plane_num; i++) {
			if (!is_plane_attached_to_crtc(dpu_crtc->base.state,
					&dpu_crtc->plane[i]->base))
				continue;
			plane_state = dpu_crtc->plane[i]->base.state;
			states[n++] = plane_state;
		}

		/* no active planes? */
		if (n == 0) {
			kfree(states);
			continue;
		}

		if (use_pc[dpu_crtc->crtc_grp_id])
			dpu_atomic_compute_plane_lrx_per_crtc(crtc_state,
								states, n);

		dpu_atomic_set_top_plane_per_crtc(states, n,
					use_pc[dpu_crtc->crtc_grp_id]);

		ret = dpu_atomic_assign_plane_source_per_crtc(states, n,
						use_pc[dpu_crtc->crtc_grp_id]);
		if (ret) {
			DRM_DEBUG_KMS("[CRTC:] cannot assign plane rscs\n");
			kfree(states);
			return ret;
		}

		kfree(states);
	}

	return ret;
}
