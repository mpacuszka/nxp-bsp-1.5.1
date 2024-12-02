// SPDX-License-Identifier: GPL-2.0
/*
 * dev_printk.h - printk messages helpers for devices
 *
 * Copyright (c) 2001-2003 Patrick Mochel <mochel@osdl.org>
 * Copyright (c) 2004-2009 Greg Kroah-Hartman <gregkh@suse.de>
 * Copyright (c) 2008-2009 Novell Inc.
 * Copyright 2022 NXP
 *
 */

#ifndef _DEVICE_PRINTK_H_
#define _DEVICE_PRINTK_H_

#include <linux/types.h>

#ifndef dev_fmt
#define dev_fmt(fmt) fmt
#endif

struct device;

void _dev_err(const struct device *dev, const char *fmt, ...);
void _dev_warn(const struct device *dev, const char *fmt, ...);
void _dev_notice(const struct device *dev, const char *fmt, ...);
void _dev_info(const struct device *dev, const char *fmt, ...);
void _dev_dbg(const struct device *dev, const char *fmt, ...);

#define dev_err(dev, fmt, ...)						\
	_dev_err(dev, dev_fmt(fmt), ##__VA_ARGS__)
#define dev_warn(dev, fmt, ...)						\
	_dev_warn(dev, dev_fmt(fmt), ##__VA_ARGS__)
#define dev_notice(dev, fmt, ...)						\
	_dev_notice(dev, dev_fmt(fmt), ##__VA_ARGS__)
#define dev_info(dev, fmt, ...)						\
	_dev_info(dev, dev_fmt(fmt), ##__VA_ARGS__)

#if defined(DEBUG)
#define dev_dbg(dev, fmt, ...)						\
	_dev_dbg(dev, dev_fmt(fmt), ##__VA_ARGS__)
#else
#define dev_dbg(dev, fmt, ...)
#endif

#endif /* _DEVICE_PRINTK_H_ */
