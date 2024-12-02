/*
 * Copyright 2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */
#ifndef __IT6263_H
#define __IT6263_H

int it6263_probe(struct i2c_client *client_hdmi, struct i2c_client *client_lvds,
	bool split_mode, u32 ldb_bus_format, bool reset);
int it6263_remove(struct i2c_client *client);
void it6263_bridge_mode_set(struct i2c_client *client_hdmi, const struct videomode *vm);
void it6263_bridge_enable(struct i2c_client *client_hdmi);
void it6263_bridge_disable(struct i2c_client *client_hdmi);

/* #define IT6263_DEBUG */
#ifdef IT6263_DEBUG
void it6263_dumpregs(struct i2c_client *client_hdmi);
#endif

#endif /* __IT6263_H */
