/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022 NXP
 */
/*
 * IRQ subsystem internal functions and variables:
 *
 * Do not ever include this file from anything else than
 * kernel/irq/. Do not even think about using any information outside
 * of this file for your non core code.
 */
#include <linux/irqdesc.h>

extern void mask_irq(struct irq_desc *desc);
extern void unmask_irq(struct irq_desc *desc);

static inline void irqd_clear(struct irq_data *d, unsigned int mask)
{
	d->state_use_accessors &= ~mask;
}

static inline void irqd_set(struct irq_data *d, unsigned int mask)
{
	d->state_use_accessors |= mask;
}

static inline void irq_state_set_disabled(struct irq_desc *desc)
{
	irqd_set(&desc->irq_data, IRQD_IRQ_DISABLED);
}

static inline void irq_state_set_masked(struct irq_desc *desc)
{
	irqd_set(&desc->irq_data, IRQD_IRQ_MASKED);
}
