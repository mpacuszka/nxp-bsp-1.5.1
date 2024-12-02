// SPDX-License-Identifier: GPL-2.0
/*
 * device.h - generic, centralized driver model
 *
 * Copyright (c) 2001-2003 Patrick Mochel <mochel@osdl.org>
 * Copyright (c) 2004-2009 Greg Kroah-Hartman <gregkh@suse.de>
 * Copyright (c) 2008-2009 Novell Inc.
 * Copyright 2022 NXP
 *
 * See Documentation/driver-api/driver-model/ for more information.
 */

#ifndef _DEVICE_H_
#define _DEVICE_H_

#include <linux/dev_printk.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <ntintsafe.h>

/**
 * struct device - The basic device structure
 * @init_name:	Initial name of the device.
 * @platform_data: Platform data specific to the device.
 * 		Example: For devices on custom boards, as typical of embedded
 * 		and SOC based hardware, Linux often uses platform_data to point
 * 		to board-specific structures describing devices and how they
 * 		are wired.  That can include what ports are available, chip
 * 		variants, which GPIO pins act in what additional roles, and so
 * 		on.  This shrinks the "Board Support Packages" (BSPs) and
 * 		minimizes board-specific #ifdefs in drivers.
 * @driver_data: Private pointer for driver specific info.
 * @of_node:	Associated device tree node.
 */
struct device {
	struct device* parent; /* parent device e.g. crtc has low level device as parent */
	const char		*init_name; /* initial name of the device */
	void		*platform_data;	/* Platform specific data, device
					   core doesn't touch it */
	void		*driver_data;	/* Driver data, set and get with
					   dev_set_drvdata/dev_get_drvdata */

	struct device_node	of_node; /* associated device tree node */
	struct clk_init_data_desc* of_clk; /* data associated with clock initialization of the device */
	struct clk *(*get_clock_item)(int); /* pointer to platform specific function to get clock item for given clock index */
};

static inline void *devm_kzalloc(struct device *dev, size_t size, gfp_t gfp)
{
	return kmalloc(size, gfp | __GFP_ZERO);
}
static inline void *devm_kmalloc_array(struct device *dev,
	size_t n, size_t size, gfp_t flags)
{
	if (n != 0 && size > ULONG_MAX / n)
		return NULL;

	return kmalloc(n * size, flags);
}
static inline void *devm_kcalloc(struct device *dev,
	size_t n, size_t size, gfp_t flags)
{
	return devm_kmalloc_array(dev, n, size, flags | __GFP_ZERO);
}

static inline void devm_kfree(struct device *dev, const void *p)
{
	kfree(p);
}

void *devm_kmemdup(struct device *dev, const void *src, size_t len, gfp_t gfp);

void __iomem *devm_ioremap_resource(struct device *dev,
	const struct resource *res);

static inline void *dev_get_drvdata(const struct device *dev)
{
	return dev->driver_data;
}

static inline void dev_set_drvdata(struct device *dev, void *data)
{
	dev->driver_data = data;
}

#endif /* _DEVICE_H_ */
