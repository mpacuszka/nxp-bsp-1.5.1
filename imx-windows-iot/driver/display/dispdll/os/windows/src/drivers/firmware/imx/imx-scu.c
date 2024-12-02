// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018, 2022 NXP
 *  Author: Dong Aisheng <aisheng.dong@nxp.com>
 *
 * Implementation of the SCU IPC functions using MUs (client side).
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/firmware/imx/ipc.h>
#include <svc/ipc.h>
#include <svc/scfw.h>

static sc_ipc_struct_t imx_sc_ipc_handle;

/*
 * Get the default handle used by SCU
 */
sc_ipc_struct_t *imx_scu_get_handle()
{
	return &imx_sc_ipc_handle;
}

int imx_scu_probe(sc_ipc_id_struct_t *ipc)
{
	if (!ipc) {
		return -EINVAL;
	}

	return sc_ipc_open(&imx_sc_ipc_handle, ipc);
}

void imx_scu_remove()
{
	sc_ipc_close(&imx_sc_ipc_handle);
}

