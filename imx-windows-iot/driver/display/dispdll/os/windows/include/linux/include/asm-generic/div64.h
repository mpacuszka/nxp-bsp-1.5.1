/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_GENERIC_DIV64_H
#define _ASM_GENERIC_DIV64_H
/*
 * Copyright (C) 2003 Bernardo Innocenti <bernie@develer.com>
 * Based on former asm-ppc/div64.h and asm-m68knommu/div64.h
 *
 * Optimization for constant divisors on 32-bit machines:
 * Copyright (C) 2006-2015 Nicolas Pitre
 *
 * Copyright 2022 NXP
 *
 * The semantics of do_div() are:
 *
 * uint32_t do_div(uint64_t *n, uint32_t base)
 * {
 * 	uint32_t remainder = *n % base;
 * 	*n = *n / base;
 * 	return remainder;
 * }
 *
 * NOTE: macro parameter n is evaluated multiple times,
 *       beware of side effects!
 */

#include <linux/types.h>

 /**
  * do_div - returns 2 values: calculate remainder and update new dividend
  * @n: dividend (will be updated)
  * @base: divisor
 */
inline uint32_t do_div(uint64_t *n, uint32_t base)
{
	uint32_t rem = *n % base;
	*n /= base;

	return rem;
}

#endif /* _ASM_GENERIC_DIV64_H */
