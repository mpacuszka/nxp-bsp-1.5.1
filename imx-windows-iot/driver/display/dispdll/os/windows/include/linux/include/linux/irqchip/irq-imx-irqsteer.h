// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017, 2022 NXP
 * Copyright (C) 2018 Pengutronix, Lucas Stach <kernel@pengutronix.de>
 */
#ifndef __LINUX_IRQCHIP_IRQ_IMX_IRQSTEER_H
#define __LINUX_IRQCHIP_IRQ_IMX_IRQSTEER_H

int imx_irqsteer_probe(struct platform_device *pdev);
int imx_irqsteer_remove(struct platform_device *pdev);

#endif
