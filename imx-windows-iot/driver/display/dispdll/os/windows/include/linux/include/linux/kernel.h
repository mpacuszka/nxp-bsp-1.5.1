/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022,2024 NXP
 */
#ifndef _LINUX_KERNEL_H
#define _LINUX_KERNEL_H

#include <stdarg.h>
#include <linux/align.h>
#include <stddef.h>
#include <stdlib.h>
#include <linux/limits.h>
#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/bitops.h>
#include <linux/minmax.h>
#include <linux/printk.h>
#include <linux/build_bug.h>
#include <asm-generic/div64.h>

/**
 * ARRAY_SIZE - get the number of elements in array @arr
 * @arr: array to be sized
 */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define DIV_ROUND_UP __KERNEL_DIV_ROUND_UP

/*
 * for u64 dividends. divisor must be a 32-bit
 * number.
 */
#define DIV_ROUND_DOWN_ULL(ll, d) _div_round_down_ull((ll), (d))

static inline uint64_t _div_round_down_ull(uint64_t ll, uint32_t d)
{
	uint64_t _tmp = ll;
	(void)do_div(&_tmp, d);
	return _tmp;
}

#define DIV_ROUND_UP_ULL(ll, d) \
	DIV_ROUND_DOWN_ULL((unsigned long long)(ll) + (d) - 1, (d))

#define DIV_ROUND_CLOSEST(x, divisor) _div_round_closest((x), (divisor))

static inline uint32_t _div_round_closest(uint32_t x, uint32_t divisor)
{
	return (x+(divisor / 2)) / divisor;
}

#define DIV_ROUND_CLOSEST_ULL(x, divisor) _div_round_closest_ull((x), (divisor))

static inline uint64_t _div_round_closest_ull(uint64_t x, uint32_t divisor)
{
	uint64_t _tmp = (x) + (divisor) / 2;
	(void)do_div(&_tmp, divisor);
	return _tmp;
}

#define __round_mask(x, y) (((y)-1))

/**
 * round_up - round up to next specified power of 2
 * @x: the value to round
 * @y: multiple to round up to (must be a power of 2)
 *
 * Rounds @x up to next multiple of @y (which must be a power of 2).
 * To perform arbitrary rounding up, use roundup() below.
 */
#define round_up(x, y) ((((x)-1) | __round_mask(x, y))+1)

 /*
  * This looks more complex than it should be. But we need to
  * get the type for the ~ right in round_down (it needs to be
  * as wide as the result!), and we want to evaluate the macro
  * arguments just once each.
  */
#define __round_down_mask(x, y) ( sizeof(x)==64 ? ((uint64_t)((y)-1)) : (( y )-1))

 /**
  * round_down - round down to next specified power of 2
  * @x: the value to round
  * @y: multiple to round down to (must be a power of 2)
  *
  * Rounds @x down to next multiple of @y (which must be a power of 2).
  * To perform arbitrary rounding down, use rounddown() below.
  */
#define round_down(x, y) ((x) & ~__round_down_mask(x, y))


/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 * This is ISO compliant version of the original Linux kernel macro.
 */
#define container_of(ptr, type, member) \
	( (void)sizeof(0 ? (ptr) : &((type *)0)->member), \
	(type *)((char*)(ptr) - offsetof(type, member)) )

#endif
