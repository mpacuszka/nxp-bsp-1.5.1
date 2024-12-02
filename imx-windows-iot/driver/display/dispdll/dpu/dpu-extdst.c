/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019, 2022 NXP
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

#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
#include "dpu-prv.h"

#define PIXENGCFG_STATIC		0x8
#define POWERDOWN			BIT(4)
#define SYNC_MODE			BIT(8)
#define SW_RESET			BIT(11)
#define DIV(n)				(((n) & 0xFF) << 16)
#define DIV_RESET			0x80
#define PIXENGCFG_DYNAMIC		0xC
#define PIXENGCFG_REQUEST		0x10
#define SHDLDREQ(n)			BIT(n)
#define SEL_SHDLDREQ			BIT(0)
#define PIXENGCFG_TRIGGER		0x14
#define SYNC_TRIGGER			BIT(0)
#define TRIGGER_SEQUENCE_COMPLETE	BIT(4)
#define PIXENGCFG_STATUS		0x18
#define SYNC_BUSY			BIT(8)
#define KICK_MODE			BIT(8)
#define PERFCOUNTMODE			BIT(12)
#define CONTROL				0xC
#define GAMMAAPPLYENABLE		BIT(0)
#define SOFTWAREKICK			0x10
#define KICK				BIT(0)
#define STATUS				0x14
#define CNT_ERR_STS			BIT(0)
#define CONTROLWORD			0x18
#define CURPIXELCNT			0x1C
static u16 get_xval(u32 pixel_cnt)
{
	return pixel_cnt & 0xFFFF;
}

static u16 get_yval(u32 pixel_cnt)
{
	return pixel_cnt >> 16;
}
#define LASTPIXELCNT			0x20
#define PERFCOUNTER			0x24

struct dpu_extdst {
	char __iomem *pec_base;
	char __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
};

static inline u32 dpu_pec_ed_read(struct dpu_extdst *ed, unsigned int offset)
{
	return readl(ed->pec_base + offset);
}

static inline void dpu_pec_ed_write(struct dpu_extdst *ed,
				unsigned int offset, u32 value)
{
	writel(value, ed->pec_base + offset);
}

static inline u32 dpu_ed_read(struct dpu_extdst *ed, unsigned int offset)
{
	return readl(ed->base + offset);
}

static inline void dpu_ed_write(struct dpu_extdst *ed,
				unsigned int offset, u32 value)
{
	writel(value, ed->base + offset);
}

static inline bool dpu_ed_is_safety_stream(struct dpu_extdst *ed)
{
	if (ed->id == 4 || ed->id == 5)
		return true;

	return false;
}

void extdst_pixengcfg_shden(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_pec_ed_write(ed, PIXENGCFG_STATIC, val);
	mutex_unlock(&ed->mutex);
}

void extdst_pixengcfg_powerdown(struct dpu_extdst *ed, bool powerdown)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (powerdown)
		val |= POWERDOWN;
	else
		val &= ~POWERDOWN;
	dpu_pec_ed_write(ed, PIXENGCFG_STATIC, val);
	mutex_unlock(&ed->mutex);
}

void extdst_pixengcfg_sync_mode(struct dpu_extdst *ed, ed_sync_mode_t mode)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (mode == AUTO)
		val |= SYNC_MODE;
	else
		val &= ~SYNC_MODE;
	dpu_pec_ed_write(ed, PIXENGCFG_STATIC, val);
	mutex_unlock(&ed->mutex);
}

void extdst_pixengcfg_reset(struct dpu_extdst *ed, bool reset)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (reset)
		val |= SW_RESET;
	else
		val &= ~SW_RESET;
	dpu_pec_ed_write(ed, PIXENGCFG_STATIC, val);
	mutex_unlock(&ed->mutex);
}

void extdst_pixengcfg_div(struct dpu_extdst *ed, u16 div)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	val &= ~0xFF0000;
	val |= DIV(div);
	dpu_pec_ed_write(ed, PIXENGCFG_STATIC, val);
	mutex_unlock(&ed->mutex);
}

void extdst_pixengcfg_syncmode_master(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATIC);
	if (enable)
		val |= BIT(16);
	else
		val &= ~BIT(16);
	dpu_pec_ed_write(ed, PIXENGCFG_STATIC, val);
	mutex_unlock(&ed->mutex);
}

int extdst_pixengcfg_src_sel(struct dpu_extdst *ed, extdst_src_sel_t src)
{
	mutex_lock(&ed->mutex);
	dpu_pec_ed_write(ed, PIXENGCFG_DYNAMIC, src);
	mutex_unlock(&ed->mutex);

	return 0;
}

void extdst_pixengcfg_sel_shdldreq(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_REQUEST);
	val |= SEL_SHDLDREQ;
	dpu_pec_ed_write(ed, PIXENGCFG_REQUEST, val);
	mutex_unlock(&ed->mutex);
}

void extdst_pixengcfg_shdldreq(struct dpu_extdst *ed, u32 req_mask)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_REQUEST);
	val |= req_mask;
	dpu_pec_ed_write(ed, PIXENGCFG_REQUEST, val);
	mutex_unlock(&ed->mutex);
}

void extdst_pixengcfg_sync_trigger(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);
	dpu_pec_ed_write(ed, PIXENGCFG_TRIGGER, SYNC_TRIGGER);
	mutex_unlock(&ed->mutex);
}

void extdst_pixengcfg_trigger_sequence_complete(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);
	dpu_pec_ed_write(ed, PIXENGCFG_TRIGGER, TRIGGER_SEQUENCE_COMPLETE);
	mutex_unlock(&ed->mutex);
}

bool extdst_pixengcfg_is_sync_busy(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATUS);
	mutex_unlock(&ed->mutex);

	return val & SYNC_BUSY;
}

ed_pipeline_status_t extdst_pixengcfg_pipeline_status(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_pec_ed_read(ed, PIXENGCFG_STATUS);
	mutex_unlock(&ed->mutex);

	return val & 0x3;
}

void extdst_shden(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, STATICCONTROL);
	if (enable)
		val |= SHDEN;
	else
		val &= ~SHDEN;
	dpu_ed_write(ed, STATICCONTROL, val);
	mutex_unlock(&ed->mutex);
}

void extdst_kick_mode(struct dpu_extdst *ed, ed_kick_mode_t mode)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, STATICCONTROL);
	val &= ~KICK_MODE;
	val |= mode;
	dpu_ed_write(ed, STATICCONTROL, val);
	mutex_unlock(&ed->mutex);
}

void extdst_perfcountmode(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, STATICCONTROL);
	if (enable)
		val |= PERFCOUNTMODE;
	else
		val &= ~PERFCOUNTMODE;
	dpu_ed_write(ed, STATICCONTROL, val);
	mutex_unlock(&ed->mutex);
}

void extdst_gamma_apply_enable(struct dpu_extdst *ed, bool enable)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, CONTROL);
	if (enable)
		val |= GAMMAAPPLYENABLE;
	else
		val &= ~GAMMAAPPLYENABLE;
	dpu_ed_write(ed, CONTROL, val);
	mutex_unlock(&ed->mutex);
}

void extdst_kick(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);
	dpu_ed_write(ed, SOFTWAREKICK, KICK);
	mutex_unlock(&ed->mutex);
}

void extdst_cnt_err_clear(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);
	dpu_ed_write(ed, STATUS, CNT_ERR_STS);
	mutex_unlock(&ed->mutex);
}

bool extdst_cnt_err_status(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, STATUS);
	mutex_unlock(&ed->mutex);

	return val & CNT_ERR_STS;
}

u32 extdst_last_control_word(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, CONTROLWORD);
	mutex_unlock(&ed->mutex);

	return val;
}

void extdst_pixel_cnt(struct dpu_extdst *ed, u16 *x, u16 *y)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, CURPIXELCNT);
	mutex_unlock(&ed->mutex);

	*x = get_xval(val);
	*y = get_yval(val);
}

void extdst_last_pixel_cnt(struct dpu_extdst *ed, u16 *x, u16 *y)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, LASTPIXELCNT);
	mutex_unlock(&ed->mutex);

	*x = get_xval(val);
	*y = get_yval(val);
}

u32 extdst_perfresult(struct dpu_extdst *ed)
{
	u32 val;

	mutex_lock(&ed->mutex);
	val = dpu_ed_read(ed, PERFCOUNTER);
	mutex_unlock(&ed->mutex);

	return val;
}

bool extdst_is_master(struct dpu_extdst *ed)
{
	const struct dpu_data *data = ed->dpu->data;

	return ed->id == (int)data->master_stream_id;
}

struct dpu_extdst *dpu_ed_get(struct dpu_soc *dpu, int id)
{
	struct dpu_extdst *ed;
	int i;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if ((int)ed_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(ed_ids))
		return ERR_PTR(-EINVAL);

	ed = dpu->ed_priv[i];

	mutex_lock(&ed->mutex);

	if (ed->inuse) {
		mutex_unlock(&ed->mutex);
		return ERR_PTR(-EBUSY);
	}

	ed->inuse = true;

	mutex_unlock(&ed->mutex);

	return ed;
}

void dpu_ed_put(struct dpu_extdst *ed)
{
	mutex_lock(&ed->mutex);

	ed->inuse = false;

	mutex_unlock(&ed->mutex);
}

struct dpu_extdst *dpu_aux_ed_peek(struct dpu_extdst *ed)
{
	unsigned int aux_id = ed->id ^ 1;
	int i;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if (ed_ids[i] == aux_id)
			return ed->dpu->ed_priv[i];

	return NULL;
}

void _dpu_ed_init(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_extdst *ed;
	int i;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if (ed_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(ed_ids)) {
		WARN_ON(1);
		return;
	}

	ed = dpu->ed_priv[i];

	/* Modification to the original Linux driver:
	* in WDDM we shouldn't disable src because display pipeline is always running
	* (it has been started by firmware or left running by display driver so that
	* display may continue to work with basic display driver).
	*
	* extdst_pixengcfg_src_sel(ed, ED_SRC_DISABLE);
	*/
	extdst_pixengcfg_shden(ed, true);
	extdst_pixengcfg_powerdown(ed, false);
	extdst_pixengcfg_sync_mode(ed, SINGLE);
	extdst_pixengcfg_reset(ed, false);
	extdst_pixengcfg_div(ed, DIV_RESET);
	extdst_shden(ed, true);
	extdst_perfcountmode(ed, false);
	extdst_kick_mode(ed, EXTERNAL);
}

int dpu_ed_init(struct dpu_soc *dpu, unsigned int id,
		unsigned long pec_base, unsigned long base)
{
	struct dpu_extdst *ed;
	int i;

	ed = devm_kzalloc(dpu->dev, sizeof(*ed), GFP_KERNEL);
	if (!ed)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if (ed_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(ed_ids))
		return -EINVAL;

	dpu->ed_priv[i] = ed;

	ed->pec_base = devm_ioremap(dpu->dev, pec_base, SZ_32);
	if (!ed->pec_base)
		return -ENOMEM;

	ed->base = devm_ioremap(dpu->dev, base, SZ_64);
	if (!ed->base)
		return -ENOMEM;

	ed->dpu = dpu;
	ed->id = id;
	mutex_init(&ed->mutex);

	/* Modification to the original Linux driver:
	* in WDDM we shouldn't disable src because display pipeline is always running
	* (it has been started by firmware or left running by display driver so that
	* display may continue to work with basic display driver).
	*
	* int ret = extdst_pixengcfg_src_sel(ed, ED_SRC_DISABLE);
	* if (ret < 0)
	*	 return ret;
	*/
	_dpu_ed_init(dpu, id);

	return 0;
}

int dpu_ed_deinit(struct dpu_soc *dpu, unsigned int id)
{
	struct dpu_extdst *ed;
	int i;

	for (i = 0; i < ARRAY_SIZE(ed_ids); i++)
		if (ed_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(ed_ids))
		return -EINVAL;

	ed = dpu->ed_priv[i];
	if (!ed)
		return -ENOMEM;

	if (ed->pec_base)
		devm_iounmap(dpu->dev, ed->pec_base, SZ_32);

	if (ed->base)
		devm_iounmap(dpu->dev, ed->base, SZ_64);

	devm_kfree(dpu->dev, ed);

	return 0;
}
