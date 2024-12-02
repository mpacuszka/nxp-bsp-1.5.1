/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_REGMAP_H
#define __LINUX_REGMAP_H

/*
 * Register map access API
 *
 * Copyright 2011 Wolfson Microelectronics plc
 * Copyright 2022,2024 NXP
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 */

#include <linux/delay.h>
#include <linux/err.h>
#include "os/windows/src/comm.h"

struct i2c_client;

struct regmap_config {
	int reg_bits;
	int reg_stride;
	int val_bits;
	unsigned int max_register;
};

struct regmap {
	struct iotarget_handles io_target;
	NTSTATUS(*regmap_write)(struct iotarget_handles *tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length);
	NTSTATUS(*regmap_read)(struct iotarget_handles *tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length);
};

struct reg_sequence {
    unsigned int reg;
    unsigned int def;
    unsigned int delay_us;
};

#define regmap_read_poll_timeout_ex(map, reg, mask, cmp_val, non_equal, sleep_us, timeout_us) \
	_regmap_read_poll_timeout_ex(map, reg, mask, cmp_val, non_equal, \
			sleep_us, timeout_us, false)
int _regmap_read_poll_timeout_ex(struct regmap* map, u32 reg, u32 mask, u32 cmp_val,
    bool non_equal, unsigned long delay_us, u64 timeout_us, bool sleep_before_read);

struct regmap *devm_regmap_init_i2c(struct i2c_client *i2c);
void regmap_release_i2c(struct regmap *map);

struct regmap* devm_regmap_init_mmio(struct device* dev, struct resource* res, const struct regmap_config* config);
void regmap_release_mmio(struct regmap *map, struct resource *res);

int regmap_write(struct regmap *map, unsigned int reg, unsigned int val);
int regmap_bulk_write(struct regmap *map, unsigned int reg, const void *val, size_t val_count);
int regmap_read(struct regmap *map, unsigned int reg, unsigned int *val);
int regmap_bulk_read(struct regmap *map, unsigned int reg, void *val, size_t val_count);
int regmap_update_bits(struct regmap *map, unsigned int reg, unsigned int mask, unsigned int val);
int regmap_register_patch(struct regmap *map, const struct reg_sequence *regs, int num_regs);

#endif
