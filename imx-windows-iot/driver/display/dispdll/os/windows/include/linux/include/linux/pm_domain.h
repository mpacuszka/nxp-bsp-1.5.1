/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * pm_domain.h - Definitions and headers related to device power domains.
 *
 * Copyright (C) 2011 Rafael J. Wysocki <rjw@sisk.pl>, Renesas Electronics Corp.
 */
 /* Modifications Copyright 2022 NXP */

#ifndef _LINUX_PM_DOMAIN_H
#define _LINUX_PM_DOMAIN_H

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/spinlock.h>

int dev_pm_domain_attach_by_name(struct device* dev,
	const char* name);

#endif /* _LINUX_PM_DOMAIN_H */
