// SPDX-License-Identifier: GPL-2.0
/*
 * platform.c - platform 'pseudo' bus for legacy devices
 *
 * Copyright (c) 2002-3 Patrick Mochel
 * Copyright (c) 2002-3 Open Source Development Labs
 *
 * Please see Documentation/driver-api/driver-model/platform.rst for more
 * information.
 */
/*
 * Modifications Copyright 2022 NXP
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <uapi/asm-generic/errno-base.h>

/**
 * platform_device_add_data - add platform-specific data to a platform device
 * @pdev: platform device allocated by platform_device_alloc to add resources to
 * @data: platform specific data for this platform device
 * @size: size of platform specific data
 *
 * Add a copy of platform specific data to the platform device's
 * platform_data pointer.  The memory associated with the platform data
 * will be freed when the platform device is released.
 */
int platform_device_add_data(struct platform_device *pdev, const void *data,
	size_t size)
{
	void* d = NULL;

	if (data) {
		d = kmalloc(size, GFP_KERNEL);
		if (!d)
			return -ENOMEM;
		memcpy(d, data, size);
	}

	pdev->dev.platform_data = d;
	return 0;
}

void platform_device_remove_data(struct platform_device *pdev)
{
	if (!pdev)
		return;

	if (pdev->dev.platform_data)
		kfree(pdev->dev.platform_data);
}
