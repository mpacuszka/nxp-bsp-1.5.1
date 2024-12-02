/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef __KERNEL_PRINTK__
#define __KERNEL_PRINTK__

#define pr_err(fmt, ...) \
	_dev_err(NULL, fmt, ##__VA_ARGS__)

#define pr_warn(fmt, ...) \
	_dev_warn(NULL, fmt, ##__VA_ARGS__)

#define pr_notice(fmt, ...) \
	_dev_notice(NULL, fmt, ##__VA_ARGS__)

#define pr_info(fmt, ...) \
	_dev_info(NULL, fmt, ##__VA_ARGS__)

#endif
