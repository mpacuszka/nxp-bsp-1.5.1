/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef _LINUX_MINMAX_H
#define _LINUX_MINMAX_H

/* WARNING: simple implementation, user is responsible for all side-effects 
   and type conversions and checking. */
#define __cmp(x, y, op)	((x) op (y) ? (x) : (y))

/**
 * min - return minimum of two values of the same types
 * @x: first value
 * @y: second value
 */
#ifndef min
#define min(x, y)	__cmp(x, y, <)
#endif

/**
 * max - return maximum of two values of the same types
 * @x: first value
 * @y: second value
 */
#ifndef max
#define max(x, y)	__cmp(x, y, >)
#endif

/**
 * clamp - return a value clamped to a given range
 * @val: current value
 * @lo: lowest allowable value
 * @hi: highest allowable value
 *
 * This macro does strict typechecking of @lo/@hi to make sure they are of the
 * same type as @val.  See the unnecessary pointer comparisons.
 */
#define clamp(val, lo, hi) min(max(val, lo), hi)

/**
 * min_t - return minimum of two values, using the specified type
 * @type: data type to use
 * @x: first value
 * @y: second value
 */
#define min_t(type, x, y)	min((type)(x), (type)(y))

/**
 * max_t - return maximum of two values, using the specified type
 * @type: data type to use
 * @x: first value
 * @y: second value
 */
#define max_t(type, x, y)	max((type)(x), (type)(y))

#endif	/* _LINUX_MINMAX_H */
