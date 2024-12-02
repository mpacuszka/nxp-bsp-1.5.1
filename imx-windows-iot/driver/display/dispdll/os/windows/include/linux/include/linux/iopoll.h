/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012-2014 The Linux Foundation. All rights reserved.
 * Copyright 2022 NXP
 */

#ifndef _LINUX_IOPOLL_H
#define _LINUX_IOPOLL_H

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>

#define readl_poll_timeout(addr, mask, cmp_val, non_equal, delay_us, timeout_us) \
	_readl_poll_timeout(addr, mask, cmp_val, non_equal, delay_us, timeout_us, false)

int _readl_poll_timeout(u32 *addr, u32 mask, u32 cmp_val, bool non_equal, unsigned long delay_us, u64 timeout_us, bool sleep_before_read);

#endif /* _LINUX_IOPOLL_H */
