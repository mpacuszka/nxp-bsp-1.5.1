/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2020 NXP
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
 * File containing client-side RPC functions for the PAD service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup PAD_SVC
 * @{
 */

/* Includes */

#include <svc/types.h>
#include <svc/rm/rm_api.h>
#include <svc/pad/pad_api.h>
#include <svc/rpc.h>
#include "pad_rpc.h"

/* Local Defines */
#ifdef ALLOC_PRAGMA
#pragma alloc_text (PAGE, sc_pad_set_mux)
#pragma alloc_text (PAGE, sc_pad_get_mux)
#pragma alloc_text (PAGE, sc_pad_set_gp)
#pragma alloc_text (PAGE, sc_pad_get_gp)
#pragma alloc_text (PAGE, sc_pad_set_wakeup)
#pragma alloc_text (PAGE, sc_pad_get_wakeup)
#pragma alloc_text (PAGE, sc_pad_set_all)
#pragma alloc_text (PAGE, sc_pad_get_all)
#pragma alloc_text (PAGE, sc_pad_set)
#pragma alloc_text (PAGE, sc_pad_get)
#pragma alloc_text (PAGE, sc_pad_config)
#pragma alloc_text (PAGE, sc_pad_set_gp_28fdsoi)
#pragma alloc_text (PAGE, sc_pad_get_gp_28fdsoi)
#pragma alloc_text (PAGE, sc_pad_set_gp_28fdsoi_hsic)
#pragma alloc_text (PAGE, sc_pad_get_gp_28fdsoi_hsic)
#pragma alloc_text (PAGE, sc_pad_set_gp_28fdsoi_comp)
#pragma alloc_text (PAGE, sc_pad_get_gp_28fdsoi_comp)
#endif

/* Local Types */

/* Local Functions */

sc_err_t sc_pad_set_mux(sc_ipc_t ipc, sc_pad_t pad, UINT8 mux,
    sc_pad_config_t config, sc_pad_iso_t iso)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_SET_MUX);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);
    RPC_U8(&msg, 2U) = U8(mux);
    RPC_U8(&msg, 3U) = U8(config);
    RPC_U8(&msg, 4U) = U8(iso);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_get_mux(sc_ipc_t ipc, sc_pad_t pad, UINT8 *mux,
    sc_pad_config_t *config, sc_pad_iso_t *iso)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_GET_MUX);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (mux != NULL)
    {
        *mux = (UINT8) RPC_U8(&msg, 0U);
    }
    if (config != NULL)
    {
        *config = (sc_pad_config_t) RPC_U8(&msg, 1U);
    }
    if (iso != NULL)
    {
        *iso = (sc_pad_iso_t) RPC_U8(&msg, 2U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_pad_set_gp(sc_ipc_t ipc, sc_pad_t pad, UINT32 ctrl)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_SET_GP);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(ctrl);
    RPC_U16(&msg, 4U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_get_gp(sc_ipc_t ipc, sc_pad_t pad, UINT32 *ctrl)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_GET_GP);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (ctrl != NULL)
    {
        *ctrl = (UINT32) RPC_U32(&msg, 0U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_pad_set_wakeup(sc_ipc_t ipc, sc_pad_t pad, sc_pad_wakeup_t wakeup)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_SET_WAKEUP);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);
    RPC_U8(&msg, 2U) = U8(wakeup);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_get_wakeup(sc_ipc_t ipc, sc_pad_t pad, sc_pad_wakeup_t *wakeup)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_GET_WAKEUP);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (wakeup != NULL)
    {
        *wakeup = (sc_pad_wakeup_t) RPC_U8(&msg, 0U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_pad_set_all(sc_ipc_t ipc, sc_pad_t pad, UINT8 mux,
    sc_pad_config_t config, sc_pad_iso_t iso, UINT32 ctrl,
    sc_pad_wakeup_t wakeup)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 4U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_SET_ALL);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(ctrl);
    RPC_U16(&msg, 4U) = U16(pad);
    RPC_U8(&msg, 6U) = U8(mux);
    RPC_U8(&msg, 7U) = U8(config);
    RPC_U8(&msg, 8U) = U8(iso);
    RPC_U8(&msg, 9U) = U8(wakeup);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_get_all(sc_ipc_t ipc, sc_pad_t pad, UINT8 *mux,
    sc_pad_config_t *config, sc_pad_iso_t *iso, UINT32 *ctrl,
    sc_pad_wakeup_t *wakeup)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_GET_ALL);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (ctrl != NULL)
    {
        *ctrl = (UINT32) RPC_U32(&msg, 0U);
    }
    if (mux != NULL)
    {
        *mux = (UINT8) RPC_U8(&msg, 4U);
    }
    if (config != NULL)
    {
        *config = (sc_pad_config_t) RPC_U8(&msg, 5U);
    }
    if (iso != NULL)
    {
        *iso = (sc_pad_iso_t) RPC_U8(&msg, 6U);
    }
    if (wakeup != NULL)
    {
        *wakeup = (sc_pad_wakeup_t) RPC_U8(&msg, 7U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_pad_set(sc_ipc_t ipc, sc_pad_t pad, UINT32 val)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_SET);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(val);
    RPC_U16(&msg, 4U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_get(sc_ipc_t ipc, sc_pad_t pad, UINT32 *val)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_GET);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (val != NULL)
    {
        *val = (UINT32) RPC_U32(&msg, 0U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_pad_config(sc_ipc_t ipc, sc_pad_t pad, UINT32 val)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_CONFIG);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(val);
    RPC_U16(&msg, 4U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_set_gp_28fdsoi(sc_ipc_t ipc, sc_pad_t pad,
    sc_pad_28fdsoi_dse_t dse, sc_pad_28fdsoi_ps_t ps)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_SET_GP_28FDSOI);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);
    RPC_U8(&msg, 2U) = U8(dse);
    RPC_U8(&msg, 3U) = U8(ps);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_get_gp_28fdsoi(sc_ipc_t ipc, sc_pad_t pad,
    sc_pad_28fdsoi_dse_t *dse, sc_pad_28fdsoi_ps_t *ps)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_GET_GP_28FDSOI);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (dse != NULL)
    {
        *dse = (sc_pad_28fdsoi_dse_t) RPC_U8(&msg, 0U);
    }
    if (ps != NULL)
    {
        *ps = (sc_pad_28fdsoi_ps_t) RPC_U8(&msg, 1U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_pad_set_gp_28fdsoi_hsic(sc_ipc_t ipc, sc_pad_t pad,
    sc_pad_28fdsoi_dse_t dse, sc_bool_t hys, sc_pad_28fdsoi_pus_t pus,
    sc_bool_t pke, sc_bool_t pue)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_SET_GP_28FDSOI_HSIC);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);
    RPC_U8(&msg, 2U) = U8(dse);
    RPC_U8(&msg, 3U) = U8(pus);
    RPC_U8(&msg, 4U) = B2U8(hys);
    RPC_U8(&msg, 5U) = B2U8(pke);
    RPC_U8(&msg, 6U) = B2U8(pue);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_get_gp_28fdsoi_hsic(sc_ipc_t ipc, sc_pad_t pad,
    sc_pad_28fdsoi_dse_t *dse, sc_bool_t *hys, sc_pad_28fdsoi_pus_t *pus,
    sc_bool_t *pke, sc_bool_t *pue)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_GET_GP_28FDSOI_HSIC);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (dse != NULL)
    {
        *dse = (sc_pad_28fdsoi_dse_t) RPC_U8(&msg, 0U);
    }
    if (pus != NULL)
    {
        *pus = (sc_pad_28fdsoi_pus_t) RPC_U8(&msg, 1U);
    }
    if (hys != NULL)
    {
        *hys = (sc_bool_t) U2B(RPC_U8(&msg, 2U));
    }
    if (pke != NULL)
    {
        *pke = (sc_bool_t) U2B(RPC_U8(&msg, 3U));
    }
    if (pue != NULL)
    {
        *pue = (sc_bool_t) U2B(RPC_U8(&msg, 4U));
    }

    /* Return result */
    return err;
}

sc_err_t sc_pad_set_gp_28fdsoi_comp(sc_ipc_t ipc, sc_pad_t pad, UINT8 compen,
    sc_bool_t fastfrz, UINT8 rasrcp, UINT8 rasrcn, sc_bool_t nasrc_sel,
    sc_bool_t psw_ovr)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_SET_GP_28FDSOI_COMP);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);
    RPC_U8(&msg, 2U) = U8(compen);
    RPC_U8(&msg, 3U) = U8(rasrcp);
    RPC_U8(&msg, 4U) = U8(rasrcn);
    RPC_U8(&msg, 5U) = B2U8(fastfrz);
    RPC_U8(&msg, 6U) = B2U8(nasrc_sel);
    RPC_U8(&msg, 7U) = B2U8(psw_ovr);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_pad_get_gp_28fdsoi_comp(sc_ipc_t ipc, sc_pad_t pad, UINT8 *compen,
    sc_bool_t *fastfrz, UINT8 *rasrcp, UINT8 *rasrcn, sc_bool_t *nasrc_sel,
    sc_bool_t *compok, UINT8 *nasrc, sc_bool_t *psw_ovr)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
    RPC_FUNC(&msg) = U8(PAD_FUNC_GET_GP_28FDSOI_COMP);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(pad);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (compen != NULL)
    {
        *compen = (UINT8) RPC_U8(&msg, 0U);
    }
    if (rasrcp != NULL)
    {
        *rasrcp = (UINT8) RPC_U8(&msg, 1U);
    }
    if (rasrcn != NULL)
    {
        *rasrcn = (UINT8) RPC_U8(&msg, 2U);
    }
    if (nasrc != NULL)
    {
        *nasrc = (UINT8) RPC_U8(&msg, 3U);
    }
    if (fastfrz != NULL)
    {
        *fastfrz = (sc_bool_t) U2B(RPC_U8(&msg, 4U));
    }
    if (nasrc_sel != NULL)
    {
        *nasrc_sel = (sc_bool_t) U2B(RPC_U8(&msg, 5U));
    }
    if (compok != NULL)
    {
        *compok = (sc_bool_t) U2B(RPC_U8(&msg, 6U));
    }
    if (psw_ovr != NULL)
    {
        *psw_ovr = (sc_bool_t) U2B(RPC_U8(&msg, 7U));
    }

    /* Return result */
    return err;
}

/** @} */

