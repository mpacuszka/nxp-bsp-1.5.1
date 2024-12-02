/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (c) 2010-2011 Jeremy Kerr <jeremy.kerr@canonical.com>
 *  Copyright (C) 2011-2012 Linaro Ltd <mturquette@linaro.org>
 *  Copyright 2022 NXP
 */
#ifndef __LINUX_CLK_PROVIDER_H
#define __LINUX_CLK_PROVIDER_H

struct clk_init_data_desc {
	/* Name of the clock, match the device driver in "devm_clk_get" call. */
	const char *name;
	/* Index of the clock in the clock tree. */
	unsigned int id;
	/* Initial value of the parent selector for attached clock slice. Zero if not used. */
	unsigned int parent_sel;
	/* Initial rate of the clock in Hz of attached clock slice. Zero if not used. */
	unsigned int rate;
	/* Non-zero = initially enable attached parents (doesn't apply for the clock itself). Zero if not used. */
	unsigned int parent_enable;
};

#endif /* CLK_PROVIDER_H */
