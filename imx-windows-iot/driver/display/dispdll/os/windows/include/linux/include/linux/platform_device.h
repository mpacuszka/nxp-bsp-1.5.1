/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * platform_device.h - generic, centralized driver model
 *
 * Copyright (c) 2001-2003 Patrick Mochel <mochel@osdl.org>
 * Copyright 2022 NXP
 *
 * See Documentation/driver-api/driver-model/ for more information.
 */

#ifndef _PLATFORM_DEVICE_H_
#define _PLATFORM_DEVICE_H_

#include <linux/device.h>

struct platform_device {
	const char	*name;
	int		id;
	struct device	dev;
	struct resource	*resource;
	u32		num_resources;

	const char	*plat_name;
	void		*data;
};

extern struct resource *platform_get_resource(struct platform_device *,
					      unsigned int, unsigned int);

extern void __iomem *
devm_platform_ioremap_resource(struct platform_device *pdev,
	unsigned int index);

#define to_platform_device(x) container_of((x), struct platform_device, dev)

extern int platform_irq_count(struct platform_device *);
extern int platform_get_irq(struct platform_device *, unsigned int);
extern int platform_get_irq_byname(struct platform_device *, const char *);

extern int platform_device_add_data(struct platform_device *pdev,
	const void *data, size_t size);
extern void platform_device_remove_data(struct platform_device *pdev);

static inline void *platform_get_drvdata(const struct platform_device *pdev)
{
	return dev_get_drvdata(&pdev->dev);
}

static inline void platform_set_drvdata(struct platform_device *pdev,
	void *data)
{
	dev_set_drvdata(&pdev->dev, data);
}

#endif /* _PLATFORM_DEVICE_H_ */
