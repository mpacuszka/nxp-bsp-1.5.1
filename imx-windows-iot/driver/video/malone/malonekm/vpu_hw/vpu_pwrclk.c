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

#include "trace.h"
#if (MALONE_TRACE == MALONE_TRACE_WPP)
    #include "vpu_pwrclk.tmh"
#endif

#include <svc/scfw.h>
#include <svc/ipc.h>
#include <svc/pm/pm_api.h>

#include "Device.h"

/**
 * Configure SC to enable VPU peripheral power.
 * @param ipcHndl ipc handler.
 * @param pm      Power mode structure (on/off).
 * @return int EOK if success, otherwise error code.
 */
static int set_vpu_pwr(sc_ipc_t ipcHndl, sc_pm_power_mode_t pm, operation_mode_t mode)
{
    sc_err_t sciErr = { 0 };

    if (!(ipcHndl)) {
        return ERR_IPC_HANDLER;
    }

    /* Power on or off DEC or ENC */
    if (mode == DECODER) {
        sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_MU_0, pm); /* MU of CORE #0 */
        if (sciErr != SC_ERR_NONE) {
            return ERR_VPU_PER_PWR;
        }
        sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_DEC_0, pm);
        if (sciErr != SC_ERR_NONE) {
            return ERR_VPU_PER_PWR;
        }
    } else {
        sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_MU_1, pm); /* MU of CORE #1 */
        if (sciErr != SC_ERR_NONE) {
            return ERR_VPU_PER_PWR;
        }
        sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_ENC_0, pm);
        if (sciErr != SC_ERR_NONE) {
            return ERR_VPU_PER_PWR;
        }
    }

    sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU, pm);
    if (sciErr != SC_ERR_NONE) {
        return sciErr;
    }
    return ERR_OK;
}

/**
 * Enable/disable VPU peripheral power.
 * @param mu_ipcHandle ipc handler.
 * @param enable       Enable / Disable power.
 * @return int EOK if success, otherwise error code.
 */
static int vpu_set_power(sc_ipc_t mu_ipcHandle, BOOL enable, operation_mode_t mode)
{
    int ret = ERR_OK;

    if (!(mu_ipcHandle)) {
        return ERR_IPC_HANDLER;
    }

    if (enable) {
        ret |= set_vpu_pwr(mu_ipcHandle, SC_PM_PW_MODE_ON, mode);
        if (ret) {
            DBG_PRINT_ERROR("failed to power on");
        }


    } else {
        ret = set_vpu_pwr(mu_ipcHandle, SC_PM_PW_MODE_OFF, mode);
        if (ret) {
            DBG_PRINT_ERROR("failed to power off");
        }
    }
    return ret;
}

/**
 * Enable power&clk of VPU peripheral.
 * @return EOK if success, otherwise error.
 */
int init_vpu_pwr_clk(MALONE_VPU_DEVICE_CONTEXT *dev, operation_mode_t mode)
{
    int retval = ERR_OK;
    sc_ipc_struct_t mu_ipcHandle = { 0 };
    NTSTATUS status = STATUS_SUCCESS;

    DBG_DEV_METHOD_BEG();

    status = sc_ipc_open(&mu_ipcHandle, &dev->scfw_ipc_id);
    if (!NT_SUCCESS(status)) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "--- sc_ipc_getMuID() cannot open MU channel to SCU error!");
        sc_ipc_close(&mu_ipcHandle);
        DBG_DEV_METHOD_END();
        return status;
    }

    retval = vpu_set_power(&mu_ipcHandle, TRUE, mode);

    if (retval != ERR_OK) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "FATAL ERROR: Unable to power up VPU devices!");
    }

    sc_ipc_close(&mu_ipcHandle);

    DBG_DEV_METHOD_END();
    return retval;
}

/**
 * Disable power&clk of VPU peripheral.
 * @return EOK if success, otherwise error.
 */
int deinit_vpu_pwr_clk(MALONE_VPU_DEVICE_CONTEXT *dev, operation_mode_t mode)
{
    int retval = ERR_OK;
    sc_ipc_struct_t mu_ipcHandle = { 0 };
    NTSTATUS status = STATUS_SUCCESS;
    DBG_DEV_METHOD_BEG();

    status = sc_ipc_open(&mu_ipcHandle, &dev->scfw_ipc_id);
    if (!NT_SUCCESS(status)) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "--- sc_ipc_getMuID() cannot open MU channel to SCU error!");
        sc_ipc_close(&mu_ipcHandle);
        DBG_DEV_METHOD_END();
        return status;
    }

    retval |= vpu_set_power(&mu_ipcHandle, FALSE, mode);
    if (retval != ERR_OK) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "FATAL ERROR: Unable to power down VPU devices!");
    }

    sc_ipc_close(&mu_ipcHandle);

    DBG_DEV_METHOD_END();
    return retval;
}
