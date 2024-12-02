/*
** ###################################################################
**
**     Copyright (c) 2016 Freescale Semiconductor, Inc.
**     Copyright 2017-2020 NXP
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/*!
 * @file
 *
 * Implementation of the IPC functions using MUs (client side).
 */
/*==========================================================================*/

/* Includes */

#include "iMX8.h"
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Library/IoLib.h>
#include "main/ipc.h"
#include "main/rpc.h"
#include "main/sci.h"
#include "mu/fsl_mu.h"
#ifdef HAS_TEST_PTIM
    #include "main/prof.h"
#endif

/* Local Defines */

/* Local Types */

/* Local Functions */

/* Local Variables */

#ifdef HAS_TEST_PTIM
    SC_PTIM_EXTERN(testProf);
#endif

char *sc_status2str(sc_err_t status)
{
    switch (status) {
        case SC_ERR_NONE:
            return "SC_ERR_NONE";
            break;
        case SC_ERR_VERSION:
            return "SC_ERR_VERSION";
            break;
        case SC_ERR_CONFIG:
            return "SC_ERR_CONFIG";
            break;
        case SC_ERR_PARM:
            return "SC_ERR_PARM";
            break;
        case SC_ERR_NOACCESS:
            return "SC_ERR_NOACCESS";
            break;
        case SC_ERR_LOCKED:
            return "SC_ERR_LOCKED";
            break;
        case SC_ERR_UNAVAILABLE:
            return "SC_ERR_UNAVAILABLE";
            break;
        case SC_ERR_NOTFOUND:
            return "SC_ERR_NOTFOUND";
            break;
        case SC_ERR_NOPOWER:
            return "SC_ERR_NOPOWER";
            break;
        case SC_ERR_IPC:
            return "SC_ERR_IPC";
            break;
        case SC_ERR_BUSY:
            return "SC_ERR_BUSY";
            break;
        case SC_ERR_FAIL:
            return "SC_ERR_FAIL";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

/*----------------------------------------------------------------------*/
/* RPC command/response                                                 */
/*----------------------------------------------------------------------*/
void sc_call_rpc(sc_ipc_t ipc, sc_rpc_msg_t *msg, sc_bool_t no_resp)
{
    sc_err_t err = SC_ERR_NONE;
    /* Check parms */
    if ((ipc == (sc_ipc_t)NULL) || (msg == NULL)) {
        err = SC_ERR_PARM;
        ASSERT(SC_FALSE);
    }
    if (err == SC_ERR_NONE) {
        err = sc_ipc_write(ipc, msg);
    }
    if ((err == SC_ERR_NONE) && (no_resp == SC_FALSE))
    {
        err = sc_ipc_read(ipc, msg);
    }
    if (err != SC_ERR_NONE) {
        RPC_R8(msg) = (uint8_t)err;
    }
}

/*--------------------------------------------------------------------------*/
/* Open an IPC channel                                                      */
/*--------------------------------------------------------------------------*/
sc_err_t sc_ipc_open(sc_ipc_t *ipc, sc_ipc_id_t id)
{
    sc_err_t err = SC_ERR_NONE;
    MU_Type *base = (MU_Type*) id;

    /* Get MU base associated with IPC channel */
    if ((ipc == NULL) || (base == NULL)) {
        err = SC_ERR_PARM;
        ASSERT(SC_FALSE);
    }

    if (err == SC_ERR_NONE)
    {
        /* Init MU */
        MU_DisableInterrupts(base, U32(kMU_Tx0EmptyInterruptEnable)
            | U32(kMU_Tx1EmptyInterruptEnable) | U32(kMU_Tx2EmptyInterruptEnable)
            | U32(kMU_Tx3EmptyInterruptEnable) | U32(kMU_Rx0FullInterruptEnable)
            | U32(kMU_Rx1FullInterruptEnable) | U32(kMU_Rx2FullInterruptEnable)
            | U32(kMU_Rx3FullInterruptEnable) | U32(kMU_GenInt0InterruptEnable)
            | U32(kMU_GenInt1InterruptEnable) | U32(kMU_GenInt2InterruptEnable)
            | U32(kMU_GenInt3InterruptEnable) | U32(kMU_GenInt0InterruptTrigger)
            | U32(kMU_GenInt1InterruptTrigger) | U32(kMU_GenInt2InterruptTrigger)
            | U32(kMU_GenInt3InterruptTrigger));
        MU_ClearStatusFlags(base, U32(kMU_GenInt0Flag) | U32(kMU_GenInt1Flag)
            | U32(kMU_GenInt2Flag) | U32(kMU_GenInt3Flag));

        /* Enable all RX interrupts */
        /* MU_EnableInterrupts(base, U32(kMU_Rx0FullInterruptEnable)
            | U32(kMU_Rx1FullInterruptEnable) | U32(kMU_Rx2FullInterruptEnable)
            | U32(kMU_Rx3FullInterruptEnable)); */

        /* Return MU address as handle */
        *ipc = (sc_ipc_t) id;
    }

    return err;
}

/*--------------------------------------------------------------------------*/
/* Close an IPC channel                                                     */
/*--------------------------------------------------------------------------*/
void sc_ipc_close(sc_ipc_t ipc)
{
    MU_Type *base = (MU_Type*) ipc;

    if (base != NULL)
    {
        MU_DisableInterrupts(base, U32(kMU_Tx0EmptyInterruptEnable)
            | U32(kMU_Tx1EmptyInterruptEnable) | U32(kMU_Tx2EmptyInterruptEnable)
            | U32(kMU_Tx3EmptyInterruptEnable) | U32(kMU_Rx0FullInterruptEnable)
            | U32(kMU_Rx1FullInterruptEnable) | U32(kMU_Rx2FullInterruptEnable)
            | U32(kMU_Rx3FullInterruptEnable) | U32(kMU_GenInt0InterruptEnable)
            | U32(kMU_GenInt1InterruptEnable) | U32(kMU_GenInt2InterruptEnable)
            | U32(kMU_GenInt3InterruptEnable) | U32(kMU_GenInt0InterruptTrigger)
            | U32(kMU_GenInt1InterruptTrigger) | U32(kMU_GenInt2InterruptTrigger)
            | U32(kMU_GenInt3InterruptTrigger));
        MU_ClearStatusFlags(base, U32(kMU_GenInt0Flag) | U32(kMU_GenInt1Flag)
            | U32(kMU_GenInt2Flag) | U32(kMU_GenInt3Flag));
    }
}

/*--------------------------------------------------------------------------*/
/* Reset an IPC channel                                                     */
/*--------------------------------------------------------------------------*/
sc_err_t sc_ipc_reset(sc_ipc_t ipc)
{
    sc_err_t err = SC_ERR_NONE;
    MU_Type *base = (MU_Type*) ipc;

    if (base == NULL) {
        err = SC_ERR_PARM;
        ASSERT(SC_FALSE);
    }
    /* Request IPC reset */
    if (err == SC_ERR_NONE)
    {
        err = SC_ERR_IPC;

        MU_TriggerInterrupts(base, kMU_GenInt3InterruptTrigger);
        for (int cnt = MU_TR_COUNT - 1; cnt >= 0; cnt--) {
            MU_SendMsgNonBlocking(base, cnt, 0U);
            MicroSecondDelay (50);
            if (0U == (MU_GetAcceptance(base) & (uint32_t)kMU_GenInt3InterruptTrigger)) {
                err = SC_ERR_NONE;
                break;
            }
        }
    }
    if (err == SC_ERR_NONE)
    {
        err = sc_ipc_open((sc_ipc_t*)&base, (sc_ipc_id_t)ipc);
    }
    return err;
}

/*--------------------------------------------------------------------------*/
/* Read message from an IPC channel                                         */
/*--------------------------------------------------------------------------*/
sc_err_t sc_ipc_read(sc_ipc_t ipc, void *data)
{
    sc_err_t err = SC_ERR_NONE;
    MU_Type *base = (MU_Type*) ipc;
    sc_rpc_msg_t *msg = (sc_rpc_msg_t*) data;
    uint8_t count = 0U;

    /* Check parms */
    if ((base == NULL) || (msg == NULL)) {
        err = SC_ERR_PARM;
        ASSERT(FALSE);
    }

    if (err == SC_ERR_NONE)
    {

        /* Read first word */
        err = MU_ReceiveMsg(base, 0U, (uint32_t*)msg) ? SC_ERR_IPC : SC_ERR_NONE;
        count++;
    }
    if (err == SC_ERR_NONE)
    {
        /* Check size */
        if (msg->size > SC_RPC_MAX_MSG)
        {
            *((uint32_t*) msg) = 0U;
        }
        else
        {
            /* Read remaining words */
            while (count < msg->size)
            {
                err = MU_ReceiveMsg(base,
                      U32(count) % MU_RR_COUNT, &msg->DATA.u32[count - 1U])
                      ? SC_ERR_IPC : SC_ERR_NONE;

                count++;
            }
        }
    }
    return err;
}

/*--------------------------------------------------------------------------*/
/* Write a message to an IPC channel                                        */
/*--------------------------------------------------------------------------*/
sc_err_t sc_ipc_write(sc_ipc_t ipc, const void *data)
{
    sc_err_t err = SC_ERR_NONE;
    MU_Type *base = (MU_Type*) ipc;
    const sc_rpc_msg_t *msg = (const sc_rpc_msg_t*) data;
    uint8_t size;
    uint8_t count = 0U;
    
    /* Check parms */
    if ((base == NULL) || (msg == NULL)) {
        err = SC_ERR_PARM;
        ASSERT(FALSE);
    }

    /* Capture size from message buffer */
    if (err == SC_ERR_NONE) {
        size = msg->size;
        /* Check size */
        if (size > SC_RPC_MAX_MSG) {
            err = SC_ERR_PARM;
            ASSERT(FALSE);
        }
    }
    /* Write first word */
    if (err == SC_ERR_NONE)
    {
        err = MU_SendMsg(base, 0U, *((const uint32_t*) msg))
              ? SC_ERR_IPC : SC_ERR_NONE;
        count++;
    }

    /* Write remaining words */
    while ((err == SC_ERR_NONE) && (count < size))
    {
        err = MU_SendMsg(base, U32(count) % MU_TR_COUNT,
                msg->DATA.u32[count - 1U])
                ? SC_ERR_IPC : SC_ERR_NONE;

        count++;
    }
    return err;
}

