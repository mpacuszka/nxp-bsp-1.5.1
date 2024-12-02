/****************************************************************************
* Copyright (c) Microsoft Corporation.
*
*    Licensed under the MIT License.
*    Licensed under the GPL License.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmd7LAdapter.h"

NTSTATUS
GcKm7LAdapter::SetPowerState(
    IN_ULONG                DeviceUid,
    IN_DEVICE_POWER_STATE   DevicePowerState,
    IN_POWER_ACTION         ActionType)
{
    if (DISPLAY_ADAPTER_HW_ID == DeviceUid)
    {
        gceCHIPPOWERSTATE   ChipPowerState;

        switch (DevicePowerState)
        {
        case PowerDeviceD0:
            ChipPowerState = gcvPOWER_ON;
            break;
        case PowerDeviceD3:
            ChipPowerState = gcvPOWER_OFF;
            break;
        default:
            NT_ASSERT(false);
            return STATUS_NOT_SUPPORTED;
        }

        gceSTATUS   GcStatus;

        GcStatus = gckHARDWARE_SetPowerState(galDevice->kernels[0]->hardware, ChipPowerState);
        NT_ASSERT(gcvSTATUS_OK == GcStatus);

        return STATUS_SUCCESS;
    }

    return m_pDisplay->SetPowerState(DeviceUid, DevicePowerState, ActionType);
}


