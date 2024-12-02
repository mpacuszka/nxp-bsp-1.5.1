/* SPDX-License-Identifier: GPL-2.0
 *
 * Modifications Copyright 2022 NXP
 */
#ifndef __LINUX_BITMAP_H
#define __LINUX_BITMAP_H

#define BITMAP_FIRST_WORD_MASK(start) (~0UL << ((start) & (BITS_PER_LONG - 1)))
#define BITMAP_LAST_WORD_MASK(nbits) (~0UL >> (-(nbits) & (BITS_PER_LONG - 1)))

#endif /* __LINUX_BITMAP_H */
