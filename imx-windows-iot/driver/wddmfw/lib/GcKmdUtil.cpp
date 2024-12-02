/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdUtil.tmh"

#include "GcKmdUtil.h"

GC_NONPAGED_SEGMENT_BEGIN; //================================================

void* __cdecl
operator new(
    size_t      size,
    POOL_TYPE   poolType,
    UINT        tag
    ) throw ()
{
    if (!size)
    {
        size = 1;
    }

    return ExAllocatePoolWithTag(poolType, size, tag);
}

void __cdecl
operator delete(
    void   *ptr,
    size_t  size
    ) throw ()
{
    if (ptr)
    {
        ExFreePool(ptr);
    }
}

GC_NONPAGED_SEGMENT_END; //================================================

_Use_decl_annotations_
DXGI_FORMAT
TranslateD3dDdiFormat(
    D3DDDIFORMAT    Format)
{
    PAGED_CODE();

    switch (Format)
    {
    case D3DDDIFMT_A8:
        return DXGI_FORMAT_A8_UNORM;
    case D3DDDIFMT_L8:
        return DXGI_FORMAT_R8_UNORM;
    case D3DDDIFMT_L16:
        return DXGI_FORMAT_R16_UNORM;
    case D3DDDIFMT_R5G6B5:
        return DXGI_FORMAT_B5G6R5_UNORM;
    case D3DDDIFMT_A1R5G5B5:
        return DXGI_FORMAT_B5G5R5A1_UNORM;
    case D3DDDIFMT_A8R8G8B8:
        return DXGI_FORMAT_B8G8R8A8_UNORM;
    case D3DDDIFMT_X8R8G8B8:
        GC_LOG_WARNING("Forcing D3DDDIFMT_X8R8G8B8 to DXGI_FORMAT_B8G8R8A8_UNORM");
        return DXGI_FORMAT_B8G8R8A8_UNORM;  //DXGI_FORMAT_R8G8B8A8_UNORM;
    case D3DDDIFMT_A8B8G8R8:
        return DXGI_FORMAT_R8G8B8A8_UNORM;
    case D3DDDIFMT_YUY2:
        return DXGI_FORMAT_YUY2;
    case D3DDDIFMT_DXT1:
        return DXGI_FORMAT_BC1_UNORM;
    case D3DDDIFMT_DXT2:
    case D3DDDIFMT_DXT3:
        return DXGI_FORMAT_BC2_UNORM;
    case D3DDDIFMT_DXT4:
    case D3DDDIFMT_DXT5:
        return DXGI_FORMAT_BC3_UNORM;
    case D3DDDIFMT_V8U8:
        return DXGI_FORMAT_R8G8_SNORM;
    case D3DDDIFMT_D16:
    case D3DDDIFMT_D16_LOCKABLE:
        return DXGI_FORMAT_D16_UNORM;
    case D3DDDIFMT_D32F_LOCKABLE:
        return DXGI_FORMAT_D32_FLOAT;
    case D3DDDIFMT_S8D24:
    case D3DDDIFMT_X8D24:
    case D3DDDIFMT_D24S8:
    case D3DDDIFMT_D24X8:
        return DXGI_FORMAT_D24_UNORM_S8_UINT;
    case D3DDDIFMT_A16B16G16R16:
        return DXGI_FORMAT_R16G16B16A16_UNORM;
    case D3DDDIFMT_R16F:
        return DXGI_FORMAT_R16_FLOAT;
    case D3DDDIFMT_G16R16F:
        return DXGI_FORMAT_R16G16_FLOAT;
    case D3DDDIFMT_A16B16G16R16F:
        return DXGI_FORMAT_R16G16B16A16_FLOAT;
    case D3DDDIFMT_R32F:
        return DXGI_FORMAT_R32_FLOAT;
    case D3DDDIFMT_G32R32F:
        return DXGI_FORMAT_R32G32_FLOAT;
    case D3DDDIFMT_A32B32G32R32F:
        return DXGI_FORMAT_R32G32B32A32_FLOAT;
    case D3DDDIFMT_A2B10G10R10:
        return DXGI_FORMAT_R10G10B10A2_UNORM;
    case D3DDDIFMT_G16R16:
        return DXGI_FORMAT_R16G16_UNORM;
    case D3DDDIFMT_V16U16:
        return DXGI_FORMAT_R16G16_SNORM;
    case D3DDDIFMT_Q8W8V8U8:
        return DXGI_FORMAT_R8G8B8A8_SNORM;
    case D3DDDIFMT_Q16W16V16U16:
        return DXGI_FORMAT_R16G16B16A16_SNORM;
    case D3DDDIFMT_A4R4G4B4:
        return DXGI_FORMAT_B4G4R4A4_UNORM;
    case D3DDDIFMT_P8:
        return DXGI_FORMAT_P8;
    case D3DDDIFMT_A8P8:
        return DXGI_FORMAT_A8P8;
    case D3DDDIFMT_VERTEXDATA:
    case D3DDDIFMT_INDEX16:
    case D3DDDIFMT_INDEX32:
    case D3DDDIFMT_UNKNOWN:
        return DXGI_FORMAT_UNKNOWN;
    default:
        NT_ASSERT(!"Unknown format");
        return DXGI_FORMAT_UNKNOWN;
    }
}

D3DDDIFORMAT
TranslateDxgiFormat(
    DXGI_FORMAT DxgiFormat)
{
    switch (DxgiFormat)
    {
    case DXGI_FORMAT_R8G8B8A8_UNORM:
    case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
        return D3DDDIFMT_A8B8G8R8;
    case DXGI_FORMAT_R10G10B10A2_UNORM:
        return D3DDDIFMT_A2B10G10R10;
    case DXGI_FORMAT_R10G10B10_XR_BIAS_A2_UNORM:
        return D3DDDIFMT_A2B10G10R10_XR_BIAS;
    case DXGI_FORMAT_R16G16B16A16_FLOAT:
        return D3DDDIFMT_A16B16G16R16F;
    case DXGI_FORMAT_B8G8R8A8_UNORM:
    case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
        return D3DDDIFMT_A8R8G8B8;
    case DXGI_FORMAT_B8G8R8X8_UNORM:
    case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB:
        return D3DDDIFMT_X8R8G8B8;
    case DXGI_FORMAT_B5G6R5_UNORM:
        return D3DDDIFMT_R5G6B5;
    case DXGI_FORMAT_B5G5R5A1_UNORM:
        return D3DDDIFMT_A1R5G5B5;
    case DXGI_FORMAT_B4G4R4A4_UNORM:
        return D3DDDIFMT_A4R4G4B4;
    case DXGI_FORMAT_A8_UNORM:
        return D3DDDIFMT_A8;
    case DXGI_FORMAT_R16G16_UNORM:
        return D3DDDIFMT_G16R16;
    case DXGI_FORMAT_R16G16B16A16_UNORM:
        return D3DDDIFMT_A16B16G16R16;
    case DXGI_FORMAT_R16_FLOAT:
        return D3DDDIFMT_R16F;
    case DXGI_FORMAT_R16G16_FLOAT:
        return D3DDDIFMT_G16R16F;
    case DXGI_FORMAT_R32_FLOAT:
        return D3DDDIFMT_R32F;
    case DXGI_FORMAT_R32G32_FLOAT:
        return D3DDDIFMT_G32R32F;
    case DXGI_FORMAT_R32G32B32A32_FLOAT:
        return D3DDDIFMT_A32B32G32R32F;
    case DXGI_FORMAT_YUY2:
        return D3DDDIFMT_YUY2;
    case DXGI_FORMAT_D16_UNORM:
        return D3DDDIFMT_D16_LOCKABLE;
    case DXGI_FORMAT_D32_FLOAT:
        return D3DDDIFMT_D32F_LOCKABLE;
    case DXGI_FORMAT_R8G8_SNORM:
        return D3DDDIFMT_V8U8;
    case DXGI_FORMAT_R8_UNORM:
        return D3DDDIFMT_R8;
    case DXGI_FORMAT_R8G8_UNORM:
        return D3DDDIFMT_G8R8;
    case DXGI_FORMAT_BC1_UNORM:
    case DXGI_FORMAT_BC1_UNORM_SRGB:
        return D3DDDIFMT_DXT1;
    case DXGI_FORMAT_BC2_UNORM:
    case DXGI_FORMAT_BC2_UNORM_SRGB:
        return D3DDDIFMT_DXT2;
    case DXGI_FORMAT_BC3_UNORM:
    case DXGI_FORMAT_BC3_UNORM_SRGB:
        return D3DDDIFMT_DXT3;
    case DXGI_FORMAT_R8G8B8A8_SNORM:
        return D3DDDIFMT_Q8W8V8U8;
    case DXGI_FORMAT_R16G16_SNORM:
        return D3DDDIFMT_V16U16;
    case DXGI_FORMAT_UNKNOWN:
        return D3DDDIFMT_UNKNOWN;
    default:
        NT_ASSERT(FALSE);
        return D3DDDIFMT_UNKNOWN;
    }
}

bool
ReadDwordRegistryValue(
    UNICODE_STRING* pRegistryPath,
    WCHAR* pRegKeyName,
    DWORD* pData)
{
    NTSTATUS Status;
    HANDLE DriverRegKeyPath;

    OBJECT_ATTRIBUTES Attr = { 0 };
    InitializeObjectAttributes(&Attr,
                               pRegistryPath,
                               OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE,
                               NULL,
                               NULL);
    Status = ZwOpenKey(&DriverRegKeyPath, KEY_QUERY_VALUE, &Attr);
    if (!NT_SUCCESS(Status))
    {
        return false;
    }

    DWORD Data = 0;
    RTL_QUERY_REGISTRY_TABLE QueryTable[2];

    RtlZeroMemory(&QueryTable, sizeof(QueryTable));
    QueryTable[0].Name          = pRegKeyName;
    QueryTable[0].Flags         = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK | RTL_QUERY_REGISTRY_REQUIRED;
    QueryTable[0].DefaultType   = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    QueryTable[0].EntryContext  = &Data;
    Status = RtlQueryRegistryValues(RTL_REGISTRY_HANDLE, (PWSTR)DriverRegKeyPath, QueryTable, NULL, NULL);
    if (NT_SUCCESS(Status))
    {
        *pData = Data;
    }

    ZwClose(DriverRegKeyPath);

    return NT_SUCCESS(Status);
}

void
ReadDwordRegistryValue2(
    HANDLE* phRegKey,
    WCHAR*  pRegValueName,
    DWORD*  pData)
{
    NTSTATUS    Status;
    RTL_QUERY_REGISTRY_TABLE    QueryTable[2];
    DWORD       Data;

    RtlZeroMemory(&QueryTable, sizeof(QueryTable));
    QueryTable[0].Name          = pRegValueName;
    QueryTable[0].Flags         = RTL_QUERY_REGISTRY_DIRECT | RTL_QUERY_REGISTRY_TYPECHECK | RTL_QUERY_REGISTRY_REQUIRED;
    QueryTable[0].DefaultType   = (REG_DWORD << RTL_QUERY_REGISTRY_TYPECHECK_SHIFT) | REG_NONE;
    QueryTable[0].EntryContext  = &Data;

    Status = RtlQueryRegistryValues(RTL_REGISTRY_HANDLE, (PCWSTR)*phRegKey, QueryTable, NULL, NULL);
    if (NT_SUCCESS(Status))
    {
        *pData = Data;
    }
    else if (STATUS_KEY_DELETED == Status)
    {
        ZwClose(*phRegKey);
        *phRegKey = 0;
    }
}

