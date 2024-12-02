/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2024 NXP
 */
#ifndef _LINUX_ALIGN_H
#define _LINUX_ALIGN_H

#include <linux/const.h>

#define ALIGN(x, a)		(((x) + (a) - 1) / (a)) * (a)

#endif	/* _LINUX_ALIGN_H */
