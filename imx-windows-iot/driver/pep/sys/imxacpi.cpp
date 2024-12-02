// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxacpi.cpp
//
// Abstract:
//
//   IMX PEP ACPI Notification Routines
//

#include "precomp.h"

#include "trace.h"
#include "imxacpi.tmh"

#include "imxpeputil.h"
#include "imxpepioctl.h"
#include "imxpephw.h"
#include "imxpep.h"
#include "imxpep_dbg.h"

IMX_NONPAGED_SEGMENT_BEGIN; //==============================================

//
// Checks if device needs to be reserved
//
_Use_decl_annotations_
BOOLEAN IMX_PEP::AcpiPrepareDevice (PEP_ACPI_PREPARE_DEVICE* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    ArgsPtr->OutputFlags = PEP_ACPI_PREPARE_DEVICE_OUTPUT_FLAG_NONE;

    _DEVICE_ID pepDeviceId = this->pepDeviceIdFromPnpDeviceId(
                                    ArgsPtr->AcpiDeviceName
                                    );

    DBG_DRV_METHOD_BEG_WITH_PARAMS("AcpiPrepareDevice: DeviceId=0x%02x DeviceName=%s AcpiName=%ls", 
        pepDeviceId, Dbg_GetDeviceName((ULONG)pepDeviceId), ArgsPtr->AcpiDeviceName->Buffer);

    if (pepDeviceId == _DEVICE_ID::_INVALID) {
        ArgsPtr->DeviceAccepted = FALSE;
        return TRUE;
    }

    ArgsPtr->DeviceAccepted = TRUE;

    return TRUE;
}

_Use_decl_annotations_
BOOLEAN IMX_PEP::AcpiAbandonDevice (PEP_ACPI_ABANDON_DEVICE* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    _DEVICE_ID pepDeviceId = this->pepDeviceIdFromPnpDeviceId(
                                    ArgsPtr->AcpiDeviceName
                                    );
    if (pepDeviceId == _DEVICE_ID::_INVALID) {
        ArgsPtr->DeviceAccepted = FALSE;
        return TRUE;
    }

    ArgsPtr->DeviceAccepted = FALSE;

    return TRUE;
}

//
// N.B. No need to keep track of the Kernel handle since all devices
// will implement the same _STA=0. This would need to change if more
// ACPI methods need to be overloaded.
//
_Use_decl_annotations_
BOOLEAN IMX_PEP::AcpiRegisterDevice (PEP_ACPI_REGISTER_DEVICE* ArgsPtr)
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    _DEVICE_ID pepDeviceId = this->pepDeviceIdFromPnpDeviceId(
                                    ArgsPtr->AcpiDeviceName
                                    );
    if (pepDeviceId == _DEVICE_ID::_INVALID) {
        ArgsPtr->DeviceHandle = NULL;
        return TRUE;
    }

    _DEVICE_CONTEXT* contextPtr = this->contextFromDeviceId(pepDeviceId);
    if (contextPtr->isDeviceReserved == FALSE) {
        ArgsPtr->DeviceHandle = NULL;
        return TRUE;
    }

    // Use the device ID as the handle value
    ArgsPtr->DeviceHandle = pepHandleFromDeviceId(pepDeviceId);
    ArgsPtr->OutputFlags = PEP_ACPI_REGISTER_DEVICE_OUTPUT_FLAG_NONE;

    return TRUE;

}

//
// N.B. No need to check handle since all ACPI notify callbacks
// will implement the same _STA=0
//
_Use_decl_annotations_
BOOLEAN IMX_PEP::AcpiEnumerateDeviceNamespace (
    PEP_ACPI_ENUMERATE_DEVICE_NAMESPACE* ArgsPtr
    )
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    ArgsPtr->ObjectCount = 1;

    //
    // Check if output buffer size is sufficient or not
    //
    if (ArgsPtr->ObjectBufferSize < sizeof(PEP_ACPI_OBJECT_NAME_WITH_TYPE)) {
        ArgsPtr->Status = STATUS_BUFFER_TOO_SMALL;
        return TRUE;
    }

    ArgsPtr->Objects[0].Name.NameAsUlong = ACPI_OBJECT_NAME_STA;
    ArgsPtr->Objects[0].Type = PepAcpiObjectTypeMethod;
    ArgsPtr->Status = STATUS_SUCCESS;

    return TRUE;
}

//
// N.B. No need to check handle since all ACPI notify callbacks
// will implement the same _STA=0
//
_Use_decl_annotations_
BOOLEAN IMX_PEP::AcpiQueryObjectInformation (
    PEP_ACPI_QUERY_OBJECT_INFORMATION* ArgsPtr
    )
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    if (ArgsPtr->Name.NameAsUlong == ACPI_OBJECT_NAME_STA) {
        ArgsPtr->MethodObject.InputArgumentCount = 0;
        ArgsPtr->MethodObject.OutputArgumentCount = 1;
    }

    return TRUE;
}

//
// N.B. No need to check handle since all ACPI notify callbacks will
// implement the same _STA=0. Status not implemented is equivalent to _STA=0
//
_Use_decl_annotations_
BOOLEAN IMX_PEP::AcpiEvaluateControlMethod (
    PEP_ACPI_EVALUATE_CONTROL_METHOD* ArgsPtr
    )
{
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    ArgsPtr->MethodStatus = STATUS_NOT_IMPLEMENTED;

    return TRUE;
}

IMX_NONPAGED_SEGMENT_END; //================================================
