/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/include/linux/clk.h
 *
 *  Copyright (C) 2004 ARM Limited.
 *  Written by Deep Blue Solutions Limited.
 *  Copyright (C) 2011-2012 Linaro Ltd <mturquette@linaro.org>
 *  Copyright 2022-2023 NXP
 */
#ifndef __LINUX_CLK_H
#define __LINUX_CLK_H

#include <linux/err.h>
#include <linux/kernel.h>
#include "clk/clk_imx8m.h"
#include "clk/clk_imx8q.h"

struct device;
struct clk;
struct device_node;
struct of_phandle_args;

/**
 * Clk node parameters for general clock type
 */
/* flags */
#define CLK_SCFW_PRESENT 1
struct clk {
	enum imx_clk_type type;
	const char *name;
	union {
		struct imx_clk_fixed clk_fixed;
		struct imx_clk_pll clk_pll;
		struct imx_clk_gate clk_gate;
		struct imx_clk_slice clk_slice;
		struct imx_clk_lpcg_scu clk_lpcg_scu;
		struct imx_clk_scu clk_scu;
	};
	struct clk *parent;
	int flags;

	int (*clk_enable)(struct clk *clk, bool enable, bool recursive);
	unsigned long (*clk_get_rate)(struct clk *clk);
	unsigned long (*clk_get_reference_rate)(struct clk *clk);
	unsigned long(*clk_set_rate)(struct clk *clk, unsigned long rate);
	int (*clk_set_parent)(struct clk *clk, struct clk * parent);
	struct clk * (*clk_get_parent)(struct clk *clk);
};

/**
 * struct clk_bulk_data - Data used for bulk clk operations.
 *
 * @id: clock consumer ID
 * @clk: struct clk * to store the associated clock
 *
 * The CLK APIs provide a series of clk_bulk_() API calls as
 * a convenience to consumers which require multiple clks.  This
 * structure is used to manage data for these calls.
 */
struct clk_bulk_data {
	const char		*id;
	struct clk		*clk;
};

/**
 * devm_clk_get - lookup and obtain a managed reference to a clock producer.
 * @dev: device for clock "consumer"
 * @id: clock consumer ID
 *
 * Returns a struct clk corresponding to the clock producer, or
 * valid IS_ERR() condition containing errno.  The implementation
 * uses @dev and @id to determine the clock consumer, and thereby
 * the clock producer.  (IOW, @id may be identical strings, but
 * clk_get may return different clock producers depending on @dev.)
 *
 * Drivers must assume that the clock source is not enabled.
 *
 * devm_clk_get should not be called from within interrupt context.
 *
 * The clock will automatically be freed when the device is unbound
 * from the bus.
 */
struct clk *devm_clk_get(struct device *dev, const char *id);

/**
 * devm_clk_put	- "free" a managed clock source
 * @dev: device used to acquire the clock
 * @clk: clock source acquired with devm_clk_get()
 *
 * Note: drivers must ensure that all clk_enable calls made on this
 * clock source are balanced by clk_disable calls prior to calling
 * this function.
 *
 * clk_put should not be called from within interrupt context.
 */
static inline void devm_clk_put(struct device *dev, struct clk *clk) {}

/**
 * devm_clk_bulk_get - managed get multiple clk consumers
 * @dev: device for clock "consumer"
 * @num_clks: the number of clk_bulk_data
 * @clks: the clk_bulk_data table of consumer
 *
 * Return 0 on success, an errno on failure.
 *
 * This helper function allows drivers to get several clk
 * consumers in one operation with management, the clks will
 * automatically be freed when the device is unbound.
 */
int devm_clk_bulk_get(struct device *dev, int num_clks,
				   struct clk_bulk_data *clks);

/* clk_prepare_enable helps cases using clk_enable in non-atomic context. */
static inline int clk_prepare_enable(struct clk *clk)
{
	if (!clk || !clk->clk_enable) {
		return 0;
	}

	return (clk->clk_enable)(clk, true, false);
}

/* clk_disable_unprepare helps cases using clk_disable in non-atomic context. */
static inline void clk_disable_unprepare(struct clk *clk)
{
	if (!clk || !clk->clk_enable) {
		return;
	}
	(void)(clk->clk_enable)(clk, false, false);
}

int clk_bulk_prepare_enable(int num_clks, const struct clk_bulk_data *clks);
void clk_bulk_disable_unprepare(int num_clks, const struct clk_bulk_data *clks);

/**
 * clk_get_rate - obtain the current clock rate (in Hz) for a clock source.
 *		  This is only valid once the clock source has been enabled.
 * @clk: clock source
 */
inline unsigned long clk_get_rate(struct clk *clk)
{
	if (!clk || !clk->clk_get_rate) {
		return 0;
	}
	return (clk->clk_get_rate)(clk);
}

/**
 * clk_get_reference_rate - obtain the current clock reference rate (in Hz) for a clock source.
  * @clk: clock source
 */
inline unsigned long clk_get_reference_rate(struct clk *clk)
{
	if (!clk || !clk->clk_get_reference_rate) {
		return 0;
	}
	return (clk->clk_get_reference_rate)(clk);
}

/**
 * clk_set_rate - set the clock rate for a clock source
 * @clk: clock source
 * @rate: desired clock rate in Hz
 *
 * Updating the rate starts at the top-most affected clock and then
 * walks the tree down to the bottom-most clock that needs updating.
 *
 * Returns success (0) or negative errno.
 */
inline int clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long result_rate;

	if (!clk || !clk->clk_set_rate) {
		return 0;
	}
	result_rate = (clk->clk_set_rate)(clk, rate);
	return(result_rate != 0 ? 0 : -EINVAL);
}

/**
 * clk_set_parent - set the parent clock source for this clock
 * @clk: clock source
 * @parent: parent clock source
 *
 * Returns success (0) or negative errno.
 */
inline int clk_set_parent(struct clk *clk, struct clk *parent)
{
	if (!clk || !clk->clk_set_parent) {
		return 0;
	}
	return (clk->clk_set_parent)(clk, parent);
}

/**
 * clk_get_parent - get the parent clock source for this clock
 * @clk: clock source
 *
 * Returns struct clk corresponding to parent clock source, or
 * valid IS_ERR() condition containing errno.
 */
inline struct clk *clk_get_parent(struct clk *clk)
{
	if (!clk || !clk->clk_get_parent) {
		return NULL;
	}
	return (clk->clk_get_parent)(clk);
}

#endif
