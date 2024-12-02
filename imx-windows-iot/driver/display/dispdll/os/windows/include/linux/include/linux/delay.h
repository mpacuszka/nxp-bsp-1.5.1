/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_DELAY_H
#define _LINUX_DELAY_H

/*
 * Copyright (C) 1993 Linus Torvalds
 * Copyright 2022 NXP
 */

void udelay(unsigned int usecs);
void mdelay(unsigned int msecs);

void msleep(unsigned int msecs);
void usleep_range(unsigned long min, unsigned long max);
#endif /* defined(_LINUX_DELAY_H) */
