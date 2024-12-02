/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef _LINUX_JIFFIES_H
#define _LINUX_JIFFIES_H

#include <limits.h>
#include <linux/types.h>

unsigned long get_hz();

#define HZ get_hz()

unsigned long long get_jiffies(void);

# define jiffies get_jiffies()

inline bool _time_after(unsigned long long a, unsigned long long b)
{
	return ((long long)(b - a) < 0);
}

#define time_after(a,b)		\
	_time_after(a, b) 
#define time_before(a,b)	time_after(b,a)

/*
 * Change timeval to jiffies, trying to avoid the
 * most obvious overflows..
 *
 * And some not so obvious.
 *
 * Note that we don't want to return LONG_MAX, because
 * for various timeout reasons we often end up having
 * to wait "jiffies+1" in order to guarantee that we wait
 * at _least_ "jiffies" - so "jiffies+1" had better still
 * be positive.
 */
#define MAX_JIFFY_OFFSET ((LONG_MAX >> 1)-1)

unsigned int _jiffies_to_msecs(const unsigned long long j);
unsigned long long _msecs_to_jiffies(const unsigned int m);

static inline unsigned int jiffies_to_msecs(const unsigned long long j)
{
	return _jiffies_to_msecs(j);
}

/**
 * msecs_to_jiffies: - convert milliseconds to jiffies
 * @m:	time in milliseconds
 *
 * conversion is done as follows:
 *
 * - negative values mean 'infinite timeout' (MAX_JIFFY_OFFSET)
 *
 * - 'too large' values [that would result in larger than
 *   MAX_JIFFY_OFFSET values] mean 'infinite timeout' too.
 *
 * - all other values are converted to jiffies
 */
static inline unsigned long long msecs_to_jiffies(const unsigned int m)
{
	if ((int)m < 0)
		return MAX_JIFFY_OFFSET;
	return _msecs_to_jiffies(m);
}

extern unsigned long long nsecs_to_jiffies(u64 n);

#endif
