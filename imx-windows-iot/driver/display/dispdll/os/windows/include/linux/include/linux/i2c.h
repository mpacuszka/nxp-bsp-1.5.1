/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * i2c.h - definitions for the Linux i2c bus interface
 * Copyright (C) 1995-2000 Simon G. Vogl
 * Copyright (C) 2013-2019 Wolfram Sang <wsa@kernel.org>
 * Copyright 2022 NXP
 *
 * With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and
 * Frodo Looijaard <frodol@dds.nl>
 */
#ifndef _LINUX_I2C_H
#define _LINUX_I2C_H

#include <linux/device.h>	/* for struct device */
#include <uapi/linux/i2c.h>

struct i2c_client {
	unsigned short addr;		/* chip address - NOTE: 7bit	*/
								/* addresses are stored in the	*/
								/* _LOWER_ 7 bits		*/
	struct device dev;		/* the device structure		*/
	int irq;			/* irq issued by device		*/
	LARGE_INTEGER connection_id;		/* I2C connection id from the RESOURCE HUB */
	bool is_initialized;
};

static inline void *i2c_get_clientdata(const struct i2c_client *client)
{
	return dev_get_drvdata(&client->dev);
}

static inline void i2c_set_clientdata(struct i2c_client *client, void *data)
{
	dev_set_drvdata(&client->dev, data);
}

/*
 * i2c_adapter is the structure used to identify a physical i2c bus along
 * with the access algorithms necessary to access it.
 */
struct i2c_adapter {
	const struct i2c_algorithm *algo; /* the algorithm to access the bus */
	void *algo_data;

	/* data fields that are valid for all devices	*/
	int timeout;			/* in jiffies */
	int retries;
	struct device dev;		/* the adapter device */

	char name[48];
};

static inline void *i2c_get_adapdata(const struct i2c_adapter *adap)
{
	return dev_get_drvdata(&adap->dev);
}

static inline void i2c_set_adapdata(struct i2c_adapter *adap, void *data)
{
	dev_set_drvdata(&adap->dev, data);
}

struct i2c_algorithm {
	/*
	 * master_xfer should return the number of messages successfully
	 * processed, or a negative value on error
	 */
	int (*master_xfer)(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num);

	/* To determine what the adapter supports */
	u32 (*functionality)(struct i2c_adapter *adap);

};

int i2c_add_adapter(struct i2c_adapter *adap);
void i2c_del_adapter(struct i2c_adapter *adap);
/* Transfer num messages.
 */
int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);

#endif /* _LINUX_I2C_H */
