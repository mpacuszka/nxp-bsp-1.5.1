/*
 * Copyright 2022-2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
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
 *
 */

#include "precomp.h"
#include "getresrc.h"
#include "linux/slab.h"

NTSTATUS GetReslist(DXGKRNL_INTERFACE* pDxgkInterface, PWCHAR pReslist,
    ULONG ResListLen)
{
    NTSTATUS Status;
    DXGK_DEVICE_INFO DeviceInfo;
    ULONG Len;

    if (!pReslist || !pDxgkInterface)
    {
        return STATUS_INVALID_PARAMETER;
    }

    Status = pDxgkInterface->DxgkCbGetDeviceInformation(
        pDxgkInterface->DeviceHandle, &DeviceInfo);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }

    Status = IoGetDeviceProperty(
        DeviceInfo.PhysicalDeviceObject,
        DevicePropertyBootConfigurationTranslated,
        ResListLen, pReslist, &Len);
    if (!NT_SUCCESS(Status))
    {
        return Status;
    }
    if (Len >= ResListLen)
    {
        // Resource list was probably truncated due to insufficient buffer
        return STATUS_INVALID_PARAMETER;
    }

    return STATUS_SUCCESS;
}

//
// Parse resource list - currently used for I2C (Connection) and Memory resources
//
// Param ResList  resource list to be parsed
// Param ResType  resource type we are looking for. Supported CmResourceTypeConnection for I2C and CmResourceTypeMemory
// Param pConnMem For CmResourceTypeConnection returns the connection ID
//                For CmResourceTypeMemory returns the Address (Memory.Start)
// Param pSize  For CmResourceTypeConnection not used - can be NULL
//             For CmResourceTypeMemory returns the Size (Memory.Length)
// Param Index index (order) of the resource in the ACPI
// Param ResTypeExt Defines further resource type for CmResourceTypeConnection: ResourceType_I2C or ResourceType_GPIO
//                                             for CmResourceTypeMemory: ResourceType_Memory (not required)
NTSTATUS ParseReslist(PCM_RESOURCE_LIST Reslist, UCHAR ResType,
    LARGE_INTEGER *pConnMem, ULONG *pSize, ULONG Index, enum ResType ResTypeExt)
{
    PCM_FULL_RESOURCE_DESCRIPTOR List;
    ULONG Count = 0;
    ULONG Found = 0;

    if (!Reslist || !pConnMem)
    {
        return STATUS_INVALID_PARAMETER;
    }
    List = Reslist->List;
    for (ULONG i = 0; i < Reslist->Count; ++i)
    {
        // Process resources in CM_FULL_RESOURCE_DESCRIPTOR block number i.

        for (ULONG j = 0; j < List->PartialResourceList.Count; ++j)
        {
            PCM_PARTIAL_RESOURCE_DESCRIPTOR Desc;
            Desc = List->PartialResourceList.PartialDescriptors + j;
            switch (Desc->Type)
            {
            case CmResourceTypeConnection:  // Check for I2C resource
                switch (ResTypeExt)
                {
                case ResourceType_I2C:
                    if (Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL
                        && Desc->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)
                    {
                        if (Desc->Type == ResType)
                        {
                            if (Count == Index)
                            {
                                pConnMem->LowPart = Desc->u.Connection.IdLowPart;
                                pConnMem->HighPart = Desc->u.Connection.IdHighPart;
                                Found++;
                            }
                            Count++;
                        }
                    }
                    break;
                case ResourceType_GPIO:
                    if (Desc->u.Connection.Class == CM_RESOURCE_CONNECTION_CLASS_GPIO
                        && Desc->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_GPIO_IO)
                    {
                        if (Desc->Type == ResType)
                        {
                            if (Count == Index)
                            {
                                pConnMem->LowPart = Desc->u.Connection.IdLowPart;
                                pConnMem->HighPart = Desc->u.Connection.IdHighPart;
                                Found++;
                            }
                            Count++;
                        }
                    }
                    break;
                default:
                    break; // We don't care about other types.
                }
                break;
            case CmResourceTypeMemory: // Check for Memory resource
                if (Desc->Type == ResType)
                {
                    if (Count == Index)
                    {
                        pConnMem->QuadPart = Desc->u.Memory.Start.QuadPart;
                        if (pSize)
                        {
                            *pSize = Desc->u.Memory.Length;
                        }
                        Found++;
                    }
                    Count++;
                }
                break;
            default:
                break; // We don't care about other descriptors.
            }
        }
        // Advance to next CM_FULL_RESOURCE_DESCRIPTOR block in memory.
        List = (PCM_FULL_RESOURCE_DESCRIPTOR)(List->PartialResourceList.PartialDescriptors +
            List->PartialResourceList.Count);
    }
    if (!Found)
    {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    return STATUS_SUCCESS;
}

NTSTATUS GetDwordRegistryParam(DXGKRNL_INTERFACE* pDxgkInterface, PWSTR Name, ULONG* Value)
{
    NTSTATUS status;
    DXGK_DEVICE_INFO device_info;
    /* query_table must be number of items + 1 (last item must be zero) */
    RTL_QUERY_REGISTRY_TABLE query_table[2];
    USHORT len;
    PUCHAR reg_path;
    ULONG tmp_param = 0;

    if (!pDxgkInterface || !Value) {
        return STATUS_INVALID_PARAMETER;
    }
    status = pDxgkInterface->DxgkCbGetDeviceInformation(pDxgkInterface->DeviceHandle, &device_info);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    /* Convert registry path from unicode string to wide string and ensure zero termination (__GFP_ZERO in kmalloc) */
    len = device_info.DeviceRegistryPath.Length;
    reg_path = static_cast<PUCHAR>(ExAllocatePool2(POOL_FLAG_NON_PAGED, len + sizeof(WCHAR), '8XMI'));
    if (reg_path == NULL) {
        return STATUS_NO_MEMORY;
    }
    RtlZeroMemory(reg_path, len + sizeof(WCHAR));
    RtlCopyMemory(reg_path, device_info.DeviceRegistryPath.Buffer, len);

    /* Initialize query table - read 1 items from registry, last item in query_table is zeroed */
    RtlZeroMemory(query_table, sizeof(query_table));
    query_table[0].Flags = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK;
    query_table[0].Name = Name;
    query_table[0].EntryContext = &tmp_param;
    query_table[0].DefaultType = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    status = RtlQueryRegistryValues(RTL_REGISTRY_ABSOLUTE, (PCWSTR)reg_path, query_table, NULL, NULL);
    ExFreePool((void*)reg_path);

    *Value = tmp_param;

    return status;
}

PWSTR GetRegParamString(enum RegParamType regParType, UINT regIndex)
{
    static const PWSTR parInterface[] = { L"Display0Interface", L"Display1Interface", L"Display2Interface" };
    static const PWSTR parBusDataWidth[] = { L"Display0BusDataWidth", L"Display1BusDataWidth", L"Display2BusDataWidth" };
    static const PWSTR parBusMapping[] = { L"Display0BusMapping", L"Display1BusMapping", L"Display2BusMapping" };
    static const PWSTR parEdid[] = { L"Display0EDID", L"Display1EDID", L"Display2EDID" };
    static const PWSTR parNumLanes[] = { L"Display0NumLanes", L"Display1NumLanes", L"Display2NumLanes" };
    static const PWSTR parChannelId[] = { L"Display0ChannelId", L"Display1ChannelId", L"Display2ChannelId" };

    /* Assume max 3 displays */
    if (regIndex >= 3) {
        return nullptr;
    }

    switch (regParType) {
    case RegParInterface:
        return parInterface[regIndex];
    case RegParBusDataWidth:
        return parBusDataWidth[regIndex];
    case RegParBusMapping:
        return parBusMapping[regIndex];
    case RegParEdid:
        return parEdid[regIndex];
    case RegParNumLanes:
        return parNumLanes[regIndex];
    case RegParChannelId:
        return parChannelId[regIndex];
    default:
        return nullptr;
    }
}
