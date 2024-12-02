/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
#ifndef _LINUX_IRQ_H
#define _LINUX_IRQ_H


#include <linux/irqreturn.h>
#include <linux/io.h>
#include <linux/slab.h>

/*
 * irq_chip type
 *
 * IRQCHIP_TYPE_IRQ_STEER:            IRQ Steer module
 * IRQCHIP_TYPE_GIC:                  GIC
 * IRQCHIP_TYPE_NONE:                 Platform has no dedicated irq chip, used by module drivers
 */
enum irq_chip_type {
	IRQCHIP_TYPE_IRQ_STEER = (1 << 0),
	IRQCHIP_TYPE_GIC = (1 << 1),
	IRQCHIP_TYPE_NONE = (1 << 2),
};

struct irq_data;

/**
 * struct irq_chip - hardware interrupt chip descriptor
 *
 * @name:		name for /proc/interrupts
 * @type:		irq chip type
 * @irq_enable:		enable the interrupt (defaults to chip->unmask if NULL)
 * @irq_disable:	disable the interrupt
 * @irq_mask:		mask an interrupt source
 * @irq_unmask:		unmask an interrupt source
 * @flags:		chip specific flags
 */
struct irq_chip {
	const char	*name;
	enum irq_chip_type type;

	bool(*irq_check)(struct irq_data *data);
	void(*irq_enable)(struct irq_data *data);
	void(*irq_disable)(struct irq_data *data);

	void(*irq_mask)(struct irq_data *data);
	void(*irq_unmask)(struct irq_data *data);

	unsigned long	flags;
};

/**
 * struct irq_data - per irq chip data passed down to chip functions
 * @mask:		precomputed bitmask for accessing the chip registers
 * @irq:		interrupt number
 * @hwirq:		hardware interrupt number, local to the interrupt domain
 * @state_use_accessors: status information for irq chip functions.
 *			Use accessor functions to deal with it
 * @chip:		low level interrupt hardware access
 * @chip_data:		platform-specific per-chip private data for the chip
 *			methods, to allow shared chip implementations
 */
struct irq_data {
	u32			mask;
	unsigned int		irq;
	unsigned long		hwirq;
	unsigned int		state_use_accessors;
	struct irq_chip		chip;
	void			*chip_data;
};

/*
 * Bit masks for irq_data.state_use_accessors
 *
 * IRQD_IRQ_DISABLED		- Disabled state of the interrupt
 * IRQD_IRQ_MASKED		- Masked state of the interrupt
 */
enum {
	IRQD_IRQ_DISABLED = (1 << 16),
	IRQD_IRQ_MASKED = (1 << 17),
};

static inline bool irqd_irq_disabled(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_IRQ_DISABLED;
}

static inline bool irqd_irq_masked(struct irq_data *d)
{
	return d->state_use_accessors & IRQD_IRQ_MASKED;
}

struct irq_desc;

void irq_disable(struct irq_desc *desc);
void irq_enable(struct irq_desc *desc);

#endif /* _LINUX_IRQ_H */
