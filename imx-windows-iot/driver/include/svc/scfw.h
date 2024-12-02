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
 * Header file containing includes to system headers and porting types.
 */

#ifndef SC_SCFW_H
#define SC_SCFW_H

/* Includes */

#include <ntddk.h>
#define RESHUB_USE_HELPER_ROUTINES
#include <reshub.h>
#include <initguid.h>
#include <spb.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define SCFW_RPC_MSG_WLEN 8U
#define SCFW_SYNCHRONOUS_TIMEOUT_US 20000

/* Types */

/*!
 * This type is used to declare a handle for an IPC communication
 * channel. Its meaning is specific to the IPC implementation.
 */
typedef struct _sc_ipc_struct_t {
    PDEVICE_OBJECT ipc_device;
    PFILE_OBJECT ipc_file;
    HANDLE ipc_target;
    KEVENT ipc_event;
    ULONG ipc_timeout_us;
    UINT32 rx_buffer[SCFW_RPC_MSG_WLEN];
} sc_ipc_struct_t, *sc_ipc_t;

/*!
 * This type is used to declare an ID for an IPC communication
 * channel. Its meaning is specific to the IPC implementation.
 */
typedef struct _sc_ipc_id_struct_t {
    LARGE_INTEGER ipc_id;
    ULONG ipc_timeout_us;
} sc_ipc_id_struct_t, *sc_ipc_id_t;

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* SC_SCFW_H */

