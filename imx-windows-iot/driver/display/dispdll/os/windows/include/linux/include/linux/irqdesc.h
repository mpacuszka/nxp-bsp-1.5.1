/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef _LINUX_IRQDESC_H
#define _LINUX_IRQDESC_H

#include <linux/interrupt.h>
#include <wdm.h>

/**
 * struct irq_desc - interrupt descriptor
 * @name:		interrupt name
 * @irq_data:	per irq and chip data passed down to chip functions
 * @handler:	interrupt handler function
 * @dev_id:	cookie to identify the device
 * @handler:	DPC function called after @handler returns IRQ_WAKE_THREAD
 * @thread_dpc:	DPC object
 */
struct irq_desc {
	const char *name;
	struct irq_data		irq_data;
	irq_handler_t		handler;
	void			*dev_id;
	irq_handler_t		thread_fn;
	KDPC		thread_dpc;
};

#endif
