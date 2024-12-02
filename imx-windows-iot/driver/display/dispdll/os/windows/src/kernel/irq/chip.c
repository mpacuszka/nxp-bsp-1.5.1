// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 1992, 1998-2006 Linus Torvalds, Ingo Molnar
 * Copyright (C) 2005-2006, Thomas Gleixner, Russell King
 * Copyright 2022 NXP
 *
 * This file contains the core interrupt handling code, for irq-chip based
 * architectures. Detailed information is available in
 * Documentation/core-api/genericirq.rst
 */

#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/interrupt.h>

#include "internals.h"

static void irq_state_clr_disabled(struct irq_desc *desc)
{
	irqd_clear(&desc->irq_data, IRQD_IRQ_DISABLED);
}

static void irq_state_clr_masked(struct irq_desc *desc)
{
	irqd_clear(&desc->irq_data, IRQD_IRQ_MASKED);
}

void irq_enable(struct irq_desc *desc)
{
	if (!irqd_irq_disabled(&desc->irq_data)) {
		unmask_irq(desc);
	} else {
		irq_state_clr_disabled(desc);
		if (desc->irq_data.chip.irq_enable) {
			desc->irq_data.chip.irq_enable(&desc->irq_data);
			irq_state_clr_masked(desc);
		} else {
			unmask_irq(desc);
		}
	}
}

void irq_disable(struct irq_desc *desc)
{
	if (irqd_irq_disabled(&desc->irq_data)) {
		mask_irq(desc);
	} else {
		irq_state_set_disabled(desc);
		if (desc->irq_data.chip.irq_disable) {
			desc->irq_data.chip.irq_disable(&desc->irq_data);
			irq_state_set_masked(desc);
		} else {
			mask_irq(desc);
		}
	}
}

void mask_irq(struct irq_desc *desc)
{
	if (irqd_irq_masked(&desc->irq_data))
		return;

	if (desc->irq_data.chip.irq_mask) {
		desc->irq_data.chip.irq_mask(&desc->irq_data);
		irq_state_set_masked(desc);
	}
}

void unmask_irq(struct irq_desc *desc)
{
	if (!irqd_irq_masked(&desc->irq_data))
		return;

	if (desc->irq_data.chip.irq_unmask) {
		desc->irq_data.chip.irq_unmask(&desc->irq_data);
		irq_state_clr_masked(desc);
	}
}
