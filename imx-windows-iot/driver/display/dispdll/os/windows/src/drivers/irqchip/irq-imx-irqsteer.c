// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017, 2022 NXP
 * Copyright (C) 2018 Pengutronix, Lucas Stach <kernel@pengutronix.de>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/spinlock.h>

#include "boot\dts\freescale\board.h"

#define CTRL_STRIDE_OFF(_t, _r)	(_t * 4 * _r)
#define CHANCTRL		0x0
#define CHANMASK(n, t)		(CTRL_STRIDE_OFF(t, 0) + 0x4 * (n) + 0x4)
#define CHANSET(n, t)		(CTRL_STRIDE_OFF(t, 1) + 0x4 * (n) + 0x4)
#define CHANSTATUS(n, t)	(CTRL_STRIDE_OFF(t, 2) + 0x4 * (n) + 0x4)
#define CHAN_MINTDIS(t)		(CTRL_STRIDE_OFF(t, 3) + 0x4)
#define CHAN_MASTRSTAT(t)	(CTRL_STRIDE_OFF(t, 3) + 0x8)

#define CHAN_MAX_OUTPUT_INT	0x8

struct irqsteer_data {
	struct irq_chip		chip;
	char __iomem		*regs;
	struct clk* ipg_clk;
	int			irq[CHAN_MAX_OUTPUT_INT];
	int			irq_count;
	raw_spinlock_t		lock;
	int			reg_num;
	u32			channel;
	u32			*saved_reg;
	bool			inited;

	struct device		*dev;
};

static int imx_irqsteer_get_reg_index(struct irqsteer_data *data,
				      unsigned long irqnum)
{
	return (data->reg_num - irqnum / 32 - 1);
}

static bool imx_irqsteer_irq_check(struct irq_data *d)
{
	struct irqsteer_data *data = d->chip_data;
	int idx = imx_irqsteer_get_reg_index(data, d->hwirq);
	unsigned long irqmap;

	irqmap = readl_relaxed(data->regs +
		CHANSTATUS(idx, data->reg_num));

	return irqmap & BIT(d->hwirq % 32);
}

static void imx_irqsteer_irq_unmask(struct irq_data *d)
{
	struct irqsteer_data *data = d->chip_data;
	int idx = imx_irqsteer_get_reg_index(data, d->hwirq);
	unsigned char flags;
	u32 val;

	raw_spin_lock_irqsave(&data->lock, &flags);
	val = readl_relaxed(data->regs + CHANMASK(idx, data->reg_num));
	val |= BIT(d->hwirq % 32);
	writel_relaxed(val, data->regs + CHANMASK(idx, data->reg_num));
	raw_spin_unlock_irqrestore(&data->lock, flags);
}

static void imx_irqsteer_irq_mask(struct irq_data *d)
{
	struct irqsteer_data *data = d->chip_data;
	int idx = imx_irqsteer_get_reg_index(data, d->hwirq);
	unsigned char flags;
	u32 val;

	raw_spin_lock_irqsave(&data->lock, &flags);
	val = readl_relaxed(data->regs + CHANMASK(idx, data->reg_num));
	val &= ~BIT(d->hwirq % 32);
	writel_relaxed(val, data->regs + CHANMASK(idx, data->reg_num));
	raw_spin_unlock_irqrestore(&data->lock, flags);
}

static struct irq_chip imx_irqsteer_irq_chip = {
	.name = "irqsteer",
	.type = IRQCHIP_TYPE_IRQ_STEER,
	.irq_check = imx_irqsteer_irq_check,
	.irq_mask = imx_irqsteer_irq_mask,
	.irq_unmask = imx_irqsteer_irq_unmask,
};

static int imx_irqsteer_chans_enable(struct irqsteer_data *data)
{
	int ret;

	ret = clk_prepare_enable(data->ipg_clk);
	if (ret) {
		dev_err(data->dev, "failed to enable ipg clk: %d\n", ret);
		return ret;
	}

	/* steer all IRQs into configured channel */
	writel_relaxed(BIT(data->channel), data->regs + CHANCTRL);

	/* read back CHANCTRL register cannot reflact on HW register
	 * real value due to the HW action, so add one flag here.
	 */
	data->inited = true;
	return 0;
}

static void irq_set_chip_and_data(struct irqsteer_data *data)
{
	for (int i = 0; i < NR_IRQS; i++) {
		if (irq_desc[i].irq_data.chip.type == IRQCHIP_TYPE_IRQ_STEER) {
			irq_desc[i].irq_data.chip_data = data;
			irq_desc[i].irq_data.chip = imx_irqsteer_irq_chip;
		}
	}
}

int imx_irqsteer_probe(struct platform_device *pdev)
{
	struct device_node *np = &pdev->dev.of_node;
	struct irqsteer_data *data;
	u32 irqs_num;
	int ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->chip = imx_irqsteer_irq_chip;
	data->dev = &pdev->dev;
	data->inited = false;
	data->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->regs)) {
		dev_err(&pdev->dev, "failed to initialize reg\n");
		return PTR_ERR(data->regs);
	}

	data->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(data->ipg_clk)) {
		dev_err(&pdev->dev, "failed to get ipg clk\n");
		return PTR_ERR(data->ipg_clk);
	}

	raw_spin_lock_init(&data->lock);

	ret = of_property_read_u32(np, "fsl,num-irqs", &irqs_num);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "fsl,channel", &data->channel);
	if (ret)
		return ret;

	/*
	 * There is one output irq for each group of 64 inputs.
	 * One register bit map can represent 32 input interrupts.
	 */
	data->irq_count = DIV_ROUND_UP(irqs_num, 64);
	data->reg_num = irqs_num / 32;

	ret = imx_irqsteer_chans_enable(data);
	if (ret)
		return ret;

	if (!data->irq_count || data->irq_count > CHAN_MAX_OUTPUT_INT) {
		clk_disable_unprepare(data->ipg_clk);
		return -EINVAL;
	}

	irq_set_chip_and_data(data);

	pdev->dev.driver_data = data;

	return 0;
}

int imx_irqsteer_remove(struct platform_device *pdev)
{
	struct irqsteer_data *data = pdev->dev.driver_data;
	if (!data)
		return -EINVAL;

	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get memory resource\n");
		return -EINVAL;
	}

	if (data->regs)
		iounmap(data->regs, resource_size(res));

	kfree(data);

	irq_set_chip_and_data(NULL);

	return 0;
}
