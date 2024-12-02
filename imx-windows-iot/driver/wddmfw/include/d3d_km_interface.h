/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#ifndef __d3d_km_interface_h__
#define __d3d_km_interface_h__

#include "d3dukmdt.h"

typedef struct _D3DKmImageInfoFlags
{
    union
    {
        struct
        {
            UINT    Pitch           : 1;
            UINT    TileMode        : 1;
            UINT    LinearOffset    : 1;
        };

        UINT    Value;
    };
} D3DKmImageInfoFlags;

typedef struct _D3dKmUpdateImageInfo
{
    D3DKMT_HANDLE       hAllocation;
    D3DKmImageInfoFlags UpdateFlags;
    UINT                Pitch;
    UINT                TileMode;
    UINT                LinearOffset;
} D3dKmUpdateImageInfo;

typedef struct _D3dKmGetTileModeCap
{
    UINT                TileModeCap;
} D3dKmGetTileModeCap;

typedef union _D3dKmGetAdapterCaps
{
    struct
    {
        UINT            UseNV12AliasCopy : 1;
        UINT            Unused : 31;
    };
    UINT                AdapterCaps;
} D3dKmGetAdapterCaps, D3dKmAdapterCaps;

typedef struct _D3dKmCheckDirectFlip
{
    D3DKMT_HANDLE       hAllocation;
    BOOL                bSupported;
} D3dKmCheckDirectFlip;

typedef enum _D3dKmEscapeType
{
    D3D_KM_ESCAPE_UPDATE_IMAGE_INFO = 'UImI',
    D3D_KM_ESCAPE_GET_TILE_MODE_CAP = 'QTMO',
    D3D_KM_ESCAPE_GET_ADAPTER_CAPS  = 'GADC',
    D3D_KM_ESCAPE_CHECK_DIRECT_FLIP = 'CDFL'
} D3dKmEscapeType;

typedef struct _D3dKmEscape
{
    UINT            Size;
    D3dKmEscapeType Type;

    union
    {
        D3dKmUpdateImageInfo    UpdateImageInfo;
        D3dKmGetTileModeCap     GetTileModeCap;
        D3dKmGetAdapterCaps     GetAdapterCaps;
        D3dKmCheckDirectFlip    CheckDirectFlip;
    };
} D3dKmEscape;

typedef struct _D3dKmPresentSyncInfo
{
    union 
    {
        HANDLE  hUmdEvent;
        UINT64  Aligner1;
    };
    union
    {
        HANDLE  hKmdEvent;
        UINT64  Aligner2;
    };
} D3dKmPresentSyncInfo;

#define KMD_UM_BLT_PRESENT_ACK  'KBPA'

typedef struct _D3dUmBltPresentInfo
{
    UINT                    KmdAckCode;

    RECT                    DstRect;
    RECT                    SrcRect;
    UINT                    SubRectCnt;
    union 
    {
        RECT*               pDstSubRects;
        UINT64              Aligner;
    };

    D3DGPU_VIRTUAL_ADDRESS  UmBltCmdBufGpuVa;
    UINT                    UmBltCmdBufSize;

    RECT                    RectStash[64];
} D3dUmBltPresentInfo;

#define UMD_UM_BLT_PRESENT_SIGNATURE    'UBPr'

typedef struct _D3dKmPresentPrivateData
{
    UINT                        UmdSignature;

    union
    {
        D3dUmBltPresentInfo*    pUmBltPresentInfo;
        UINT64                  Aligner;
    };
} D3dKmPresentPrivateData;

#endif // __d3d_km_interface_h__
