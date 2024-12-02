/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef _LINUX_TYPES_H
#define _LINUX_TYPES_H

typedef signed __int64 ssize_t;

typedef signed char s8;
typedef unsigned char u8;
typedef signed short s16;
typedef unsigned short u16;
typedef unsigned int u32;
typedef signed int s32;
typedef unsigned long long u64;
typedef signed __int64 s64;

typedef s8 int8_t;
typedef s16 int16_t;
typedef s32 int32_t;

typedef u8  uint8_t;
typedef u16  uint16_t;
typedef u32 uint32_t;
typedef u64 uint64_t;

typedef long long int64_t;

typedef u8 __u8;
typedef u16 __u16;
typedef u32 __u32;
typedef u64 __u64;

typedef unsigned int gfp_t;

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
typedef u64 dma_addr_t;
#else
typedef u32 dma_addr_t;
#endif

#ifdef CONFIG_PHYS_ADDR_T_64BIT
typedef u64 phys_addr_t;
#else
typedef u32 phys_addr_t;
#endif

typedef phys_addr_t resource_size_t;

typedef long atomic_t;

#define __force
#define __iomem

#define __bool_true_false_are_defined	1

#ifndef __cplusplus

#define bool	_Bool
#define false	0
#define true	1

#endif /* __cplusplus */

typedef int(*cmp_func_t)(const void *a, const void *b);

#endif
