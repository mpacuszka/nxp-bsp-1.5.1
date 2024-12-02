/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */

#ifndef _LINUX_ATOMIC_H
#define _LINUX_ATOMIC_H

int atomic_read(atomic_t *v);
void atomic_set(atomic_t *v, int i);
int atomic_dec_return(atomic_t *v);
int atomic_inc_return(atomic_t *v);

static inline void atomic_inc(atomic_t *v)
{
    (void)atomic_inc_return(v);
}

static inline bool atomic_dec_and_test(atomic_t *v)
{
	return atomic_dec_return(v) == 0;
}

#endif /* _LINUX_ATOMIC_H */
