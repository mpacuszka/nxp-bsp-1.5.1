/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_COMPLETION_H
#define __LINUX_COMPLETION_H

/*
 * (C) Copyright 2001 Linus Torvalds
 * Copyright 2022 NXP
 *
 * Atomic wait-for-completion handler data structures.
 * See kernel/sched/completion.c for details.
 */

#include <wdm.h>

/*
 * struct completion - structure used to maintain state for a "completion"
  */
struct completion
{
	KEVENT event;
	KDPC dpc;
};

extern void init_completion(struct completion *x);
extern void reinit_completion(struct completion *x);

extern void wait_for_completion(struct completion *x);
extern unsigned long wait_for_completion_timeout(struct completion *x,
	unsigned long long timeout);

extern void complete(struct completion *x);

#endif
