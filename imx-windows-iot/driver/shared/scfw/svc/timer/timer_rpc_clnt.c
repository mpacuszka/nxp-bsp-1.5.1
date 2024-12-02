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
 * File containing client-side RPC functions for the TIMER service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup TIMER_SVC
 * @{
 */

/* Includes */

#include <svc/types.h>
#include <svc/rm/rm_api.h>
#include <svc/timer/timer_api.h>
#include <svc/rpc.h>
#include "timer_rpc.h"

/* Local Defines */
#ifdef ALLOC_PRAGMA
#pragma alloc_text(PAGE, sc_timer_set_wdog_timeout)
#pragma alloc_text(PAGE, sc_timer_set_wdog_pre_timeout)
#pragma alloc_text(PAGE, sc_timer_set_wdog_window)
#pragma alloc_text(PAGE, sc_timer_start_wdog)
#pragma alloc_text(PAGE, sc_timer_stop_wdog)
#pragma alloc_text(PAGE, sc_timer_ping_wdog)
#pragma alloc_text(PAGE, sc_timer_get_wdog_status)
#pragma alloc_text(PAGE, sc_timer_pt_get_wdog_status)
#pragma alloc_text(PAGE, sc_timer_set_wdog_action)
#pragma alloc_text(PAGE, sc_timer_set_rtc_time)
#pragma alloc_text(PAGE, sc_timer_get_rtc_time)
#pragma alloc_text(PAGE, sc_timer_get_rtc_sec1970)
#pragma alloc_text(PAGE, sc_timer_set_rtc_alarm)
#pragma alloc_text(PAGE, sc_timer_set_rtc_periodic_alarm)
#pragma alloc_text(PAGE, sc_timer_cancel_rtc_alarm)
#pragma alloc_text(PAGE, sc_timer_set_rtc_calb)
#pragma alloc_text(PAGE, sc_timer_set_sysctr_alarm)
#pragma alloc_text(PAGE, sc_timer_set_sysctr_periodic_alarm)
#pragma alloc_text(PAGE, sc_timer_cancel_sysctr_alarm)
#endif

/* Local Types */

/* Local Functions */

sc_err_t sc_timer_set_wdog_timeout(sc_ipc_t ipc, sc_timer_wdog_time_t timeout)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_WDOG_TIMEOUT);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(timeout);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_wdog_pre_timeout(sc_ipc_t ipc,
    sc_timer_wdog_time_t pre_timeout)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_WDOG_PRE_TIMEOUT);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(pre_timeout);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_wdog_window(sc_ipc_t ipc, sc_timer_wdog_time_t window)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_WDOG_WINDOW);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(window);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_start_wdog(sc_ipc_t ipc, sc_bool_t lock)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_START_WDOG);

    /* Fill in send message */
    RPC_U8(&msg, 0U) = B2U8(lock);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_stop_wdog(sc_ipc_t ipc)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 1U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_STOP_WDOG);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_ping_wdog(sc_ipc_t ipc)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 1U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_PING_WDOG);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_get_wdog_status(sc_ipc_t ipc, sc_timer_wdog_time_t *timeout,
    sc_timer_wdog_time_t *max_timeout, sc_timer_wdog_time_t *remaining_time)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 1U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_GET_WDOG_STATUS);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (timeout != NULL)
    {
        *timeout = (sc_timer_wdog_time_t) RPC_U32(&msg, 0U);
    }
    if (max_timeout != NULL)
    {
        *max_timeout = (sc_timer_wdog_time_t) RPC_U32(&msg, 4U);
    }
    if (remaining_time != NULL)
    {
        *remaining_time = (sc_timer_wdog_time_t) RPC_U32(&msg, 8U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_timer_pt_get_wdog_status(sc_ipc_t ipc, sc_rm_pt_t pt, sc_bool_t *enb,
    sc_timer_wdog_time_t *timeout, sc_timer_wdog_time_t *remaining_time)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_PT_GET_WDOG_STATUS);

    /* Fill in send message */
    RPC_U8(&msg, 0U) = U8(pt);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (timeout != NULL)
    {
        *timeout = (sc_timer_wdog_time_t) RPC_U32(&msg, 0U);
    }
    if (remaining_time != NULL)
    {
        *remaining_time = (sc_timer_wdog_time_t) RPC_U32(&msg, 4U);
    }
    if (enb != NULL)
    {
        *enb = (sc_bool_t) U2B(RPC_U8(&msg, 8U));
    }

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_wdog_action(sc_ipc_t ipc, sc_rm_pt_t pt,
    sc_timer_wdog_action_t action)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_WDOG_ACTION);

    /* Fill in send message */
    RPC_U8(&msg, 0U) = U8(pt);
    RPC_U8(&msg, 1U) = U8(action);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_rtc_time(sc_ipc_t ipc, UINT16 year, UINT8 mon,
    UINT8 day, UINT8 hour, UINT8 min, UINT8 sec)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_RTC_TIME);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(year);
    RPC_U8(&msg, 2U) = U8(mon);
    RPC_U8(&msg, 3U) = U8(day);
    RPC_U8(&msg, 4U) = U8(hour);
    RPC_U8(&msg, 5U) = U8(min);
    RPC_U8(&msg, 6U) = U8(sec);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_get_rtc_time(sc_ipc_t ipc, UINT16 *year, UINT8 *mon,
    UINT8 *day, UINT8 *hour, UINT8 *min, UINT8 *sec)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 1U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_GET_RTC_TIME);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (year != NULL)
    {
        *year = (UINT16) RPC_U16(&msg, 0U);
    }
    if (mon != NULL)
    {
        *mon = (UINT8) RPC_U8(&msg, 2U);
    }
    if (day != NULL)
    {
        *day = (UINT8) RPC_U8(&msg, 3U);
    }
    if (hour != NULL)
    {
        *hour = (UINT8) RPC_U8(&msg, 4U);
    }
    if (min != NULL)
    {
        *min = (UINT8) RPC_U8(&msg, 5U);
    }
    if (sec != NULL)
    {
        *sec = (UINT8) RPC_U8(&msg, 6U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_timer_get_rtc_sec1970(sc_ipc_t ipc, UINT32 *sec)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 1U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_GET_RTC_SEC1970);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Copy out receive message */
    if (sec != NULL)
    {
        *sec = (UINT32) RPC_U32(&msg, 0U);
    }

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_rtc_alarm(sc_ipc_t ipc, UINT16 year, UINT8 mon,
    UINT8 day, UINT8 hour, UINT8 min, UINT8 sec)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_RTC_ALARM);

    /* Fill in send message */
    RPC_U16(&msg, 0U) = U16(year);
    RPC_U8(&msg, 2U) = U8(mon);
    RPC_U8(&msg, 3U) = U8(day);
    RPC_U8(&msg, 4U) = U8(hour);
    RPC_U8(&msg, 5U) = U8(min);
    RPC_U8(&msg, 6U) = U8(sec);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_rtc_periodic_alarm(sc_ipc_t ipc, UINT32 sec)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_RTC_PERIODIC_ALARM);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(sec);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_cancel_rtc_alarm(sc_ipc_t ipc)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 1U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_CANCEL_RTC_ALARM);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_rtc_calb(sc_ipc_t ipc, INT8 count)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 2U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_RTC_CALB);

    /* Fill in send message */
    RPC_I8(&msg, 0U) = I8(count);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_sysctr_alarm(sc_ipc_t ipc, UINT64 ticks)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_SYSCTR_ALARM);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(ticks >> 32ULL);
    RPC_U32(&msg, 4U) = U32(ticks);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_set_sysctr_periodic_alarm(sc_ipc_t ipc, UINT64 ticks)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 3U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_SET_SYSCTR_PERIODIC_ALARM);

    /* Fill in send message */
    RPC_U32(&msg, 0U) = U32(ticks >> 32ULL);
    RPC_U32(&msg, 4U) = U32(ticks);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

sc_err_t sc_timer_cancel_sysctr_alarm(sc_ipc_t ipc)
{
    sc_rpc_msg_t msg;
    sc_err_t err;

    /* Fill in header */
    RPC_VER(&msg) = SC_RPC_VERSION;
    RPC_SIZE(&msg) = 1U;
    RPC_SVC(&msg) = U8(SC_RPC_SVC_TIMER);
    RPC_FUNC(&msg) = U8(TIMER_FUNC_CANCEL_SYSCTR_ALARM);

    /* Call RPC */
    sc_call_rpc(ipc, &msg, SC_FALSE);

    /* Copy out result */
    err = (sc_err_t) RPC_R8(&msg);

    /* Return result */
    return err;
}

/** @} */

