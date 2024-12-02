/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2018,2020,2022 NXP
 *
 * Header file for the IPC implementation.
 */

#ifndef _SC_IPC_H
#define _SC_IPC_H

#include <svc/scfw.h>

sc_ipc_struct_t *imx_scu_get_handle();
int imx_scu_probe(sc_ipc_id_struct_t *ipc);
void imx_scu_remove();
#endif /* _SC_IPC_H */
