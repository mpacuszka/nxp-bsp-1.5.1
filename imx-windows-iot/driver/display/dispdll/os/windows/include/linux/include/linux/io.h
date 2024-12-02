/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2006 PathScale, Inc.  All Rights Reserved.
 * Copyright 2022 NXP
 */

#ifndef _LINUX_IO_H
#define _LINUX_IO_H

#include <asm-generic/bug.h>
#include <linux/types.h>
#include <linux/err.h>
#include <asm/io.h>

struct device;
struct resource;

void __iomem *devm_ioremap(struct device *dev, resource_size_t offset,
	resource_size_t size);
void devm_iounmap(struct device *dev, void __iomem *addr,
	resource_size_t size);

#endif /* _LINUX_IO_H */
