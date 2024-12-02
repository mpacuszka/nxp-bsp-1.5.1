/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef _LINUX_OF_DEVICE_H
#define _LINUX_OF_DEVICE_H

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>

struct device;

extern const struct of_device_id *of_match_device(
	const struct of_device_id *matches, const struct device *dev);

#endif /* _LINUX_OF_DEVICE_H */