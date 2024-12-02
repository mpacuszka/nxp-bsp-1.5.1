/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Based on arch/arm/include/asm/io.h
 *
 * Copyright (C) 1996-2000 Russell King
 * Copyright (C) 2012 ARM Ltd.
 * Copyright 2022 NXP
 */
#ifndef __ASM_IO_H
#define __ASM_IO_H

#include <linux/types.h>
#include <linux/sizes.h>

/*
 * I/O memory access primitives.
 */
#define readl(c)		(readl_relaxed(c))
#define readb(c)		(readb_relaxed(c))

#define writel(v,c)		(writel_relaxed(v,c))
#define writeb(v,c)		(writeb_relaxed((v),(c)))

#define readl_relaxed(c) (*(volatile u32 __force *)(c))
#define readb_relaxed(c) (*(volatile u8 __force *)(c))

#define writel_relaxed(v,c)	(*(volatile u32 __force *)(c) = (v))
#define writeb_relaxed(v,c)	(*(volatile u8 __force *)(c) = (v))

/*
 * I/O memory mapping functions.
 */
extern void __iomem *__ioremap(phys_addr_t phys_addr, size_t size);
extern void iounmap(void __iomem *addr, size_t size);

#define ioremap(addr, size)		__ioremap((addr), (size))

#endif	/* __ASM_IO_H */
