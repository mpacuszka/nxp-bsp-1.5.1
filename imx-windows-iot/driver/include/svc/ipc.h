/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * Header file for the IPC implementation.
 */

#ifndef SC_IPC_H
#define SC_IPC_H

/* Includes */

#include <svc/types.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* Defines */

/* Types */

/* Functions */

/*!
 * This function opens an IPC channel.
 *
 * @param[out]    ipc         return pointer for ipc handle
 * @param[in]     id          id of channel to open
 *
 * @return Returns an error code (SC_ERR_NONE = success, SC_ERR_IPC
 *         otherwise).
 *
 * The \a id parameter is implementation specific. Could be an MU
 * address, pointer to a driver path, channel index, etc.
 */
NTSTATUS sc_ipc_open(sc_ipc_t ipc, sc_ipc_id_t id);

/*!
 * This function closes an IPC channel.
 *
 * @param[in]     ipc         id of channel to close
 */
void sc_ipc_close(sc_ipc_t ipc);


/*!
 * Converts error code to readable string description .
 *
 * @param[in]     err         id of channel read from
 *
 * @return Returns an error code string description.
 *
 * This function will block if no message is available to be read.
 */
const char *sc_err_to_string(sc_err_t err);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* SC_IPC_H */

