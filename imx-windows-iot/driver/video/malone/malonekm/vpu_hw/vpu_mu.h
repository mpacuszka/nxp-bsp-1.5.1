/*
 * Copyright 2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _MU_H_
#define _MU_H_

#include <ntddk.h>
#include <wdf.h>
#include "fsl_mu.h"
#include "circ_queue.h"

typedef enum {
    INIT_DONE = 1,
    RPC_BUF_OFFSET,
    BOOT_ADDRESS,
    COMMAND,
    EVENT
} MSG_Type;

typedef enum {
    MSG_TYPE = 0,
    MSG
} MU_index;

/* MU Interrupt Context */
typedef struct {
    mu_interrupt_data_struct_t MUInterruptContext;
    CIRCULAR_BUFFER MsgQueue;
    WDFMEMORY memhandle;
} MALONE_VPU_ISR_CONTEXT;

#define QUEUE_SIZE 256

void mu_send_msg_to_FW(MU_Type *base, UINT32 type, UINT32 value);
BOOLEAN mu_isr(WDFINTERRUPT interrupt, ULONG id, void *isr_container, void *device_container);
VOID mu_dpc(WDFINTERRUPT interrupt, WDFOBJECT AssociatedWdfObject, void *, void *);



#endif