/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef _LINUX_BITOPS_H
#define _LINUX_BITOPS_H

#include <linux/bits.h>

#define BITS_PER_TYPE(type)	(sizeof(type) * BITS_PER_BYTE)
#define BITS_TO_LONGS(nr)	(nr / BITS_PER_TYPE(long))
#define BITS_TO_U64(nr)		(nr / BITS_PER_TYPE(u64))
#define BITS_TO_U32(nr)		(nr / BITS_PER_TYPE(u32))
#define BITS_TO_BYTES(nr)	(nr / BITS_PER_TYPE(char))

/*
 * Include this here because some architectures need generic_ffs/fls in
 * scope
 */
#include <asm-generic/bitops.h>

#define for_each_set_bit(bit, addr, size) \
	for ((bit) = find_first_bit((addr), (size));		\
	     (bit) < (size);					\
	     (bit) = find_next_bit((addr), (size), (bit) + 1))

static inline unsigned fls_long(unsigned long l)
{
#if BITS_PER_LONG == 32
	return fls(l);
#elif BITS_PER_LONG == 64
	return fls64(l);
#else
#error BITS_PER_LONG not 32 or 64
#endif
}

#endif
