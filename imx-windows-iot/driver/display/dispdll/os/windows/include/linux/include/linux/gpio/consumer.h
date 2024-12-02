/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2023-2024 NXP
 */

#ifndef __LINUX_GPIO_CONSUMER_H
#define __LINUX_GPIO_CONSUMER_H

#include <linux/compiler_types.h>
#include "os/windows/src/comm.h"

struct device;

struct gpio_desc {
	unsigned long		flags;
/* flag symbols are bit numbers */
#define FLAG_ACTIVE_LOW	BIT(6)	/* value has active low */

	unsigned long		device;
	/* Types of GPIO devices */
#define FLAG_DEV_I2C_EXPANDER	BIT(30)	/* GPIO driven by I2C expander */
#define FLAG_DEV_GPIO			BIT(31)	/* "true" GPIO pin */

	unsigned long		expander_data_reg;
	unsigned long		expander_pin_mask;
	struct iotarget_handles io_target;
};

#define devm_gpiod_get_optional devm_gpiod_get
struct gpio_desc* __must_check devm_gpiod_get(struct device* dev,
	const char* con_id,
	unsigned long flags);

void devm_gpiod_put(struct device* dev, struct gpio_desc* desc);

#define gpiod_set_value_cansleep gpiod_set_value
void gpiod_set_value(struct gpio_desc *desc, int value);

#endif
