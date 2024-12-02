/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef __VDSO_BITS_H
#define __VDSO_BITS_H

#include <linux/const.h>
#include <vdso/const.h>
#include <asm/bitsperlong.h>

#define BIT(nr)			((1ULL) << (nr))
#define BIT_MASK(nr)		((1UL) << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)		((nr) / BITS_PER_LONG)
#define BITS_PER_BYTE		8

/*
 * Create a contiguous bitmask starting at bit position @l and ending at
 * position @h. For example
 * GENMASK_ULL(39, 21) gives us the 64bit vector 0x000000ffffe00000.
 */
#define __GENMASK(h, l) \
	(((~(0UL)) - ((1UL) << (l)) + 1) & \
	 (~(0UL) >> (BITS_PER_LONG - 1 - (h))))
#define GENMASK(h, l) \
	(__GENMASK(h, l))

#endif	/* __VDSO_BITS_H */
