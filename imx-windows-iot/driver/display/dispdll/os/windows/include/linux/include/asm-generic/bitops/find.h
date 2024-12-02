/* SPDX-License-Identifier: GPL-2.0
 *
 * Modifications Copyright 2022 NXP
 */
#ifndef _ASM_GENERIC_BITOPS_FIND_H_
#define _ASM_GENERIC_BITOPS_FIND_H_

#include <stddef.h>

extern unsigned long _find_next_bit(const unsigned long* addr1,
	const unsigned long* addr2, unsigned long nbits,
	unsigned long start, unsigned long invert, unsigned long le);

/**
 * find_next_bit - find the next set bit in a memory region
 * @addr: The address to base the search on
 * @offset: The bitnumber to start searching at
 * @size: The bitmap size in bits
 *
 * Returns the bit number for the next set bit
 * If no bits are set, returns @size.
 */
static inline
unsigned long find_next_bit(const unsigned long* addr, unsigned long size,
	unsigned long offset)
{
	return _find_next_bit(addr, NULL, size, offset, 0UL, 0);
}

#define find_first_bit(addr, size) find_next_bit((addr), (size), 0)

#endif /*_ASM_GENERIC_BITOPS_FIND_H_ */
