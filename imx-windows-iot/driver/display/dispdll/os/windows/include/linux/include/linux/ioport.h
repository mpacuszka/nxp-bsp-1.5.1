/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ioport.h	Definitions of routines for detecting, reserving and
 *		allocating system resources.
 *
 * Authors:	Linus Torvalds
 */
 /*
 * Modifications Copyright 2022 NXP
 */

#ifndef _LINUX_IOPORT_H
#define _LINUX_IOPORT_H

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/bits.h>
/*
 * Resources are tree-like, allowing
 * nesting etc..
 */
struct resource {
	resource_size_t start;
	resource_size_t end;
	const char *name;
	unsigned long flags;
};

/*
 * IO resources have these defined flags.
 */
#define IORESOURCE_MEM		0x00000200

static inline resource_size_t resource_size(const struct resource *res)
{
	return res->end - res->start + 1;
}

#endif	/* _LINUX_IOPORT_H */
