/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2023 NXP
   Licensed under the MIT License.

Abstract:
    CAdapterCommon class declaration.

*/

#pragma once

#include <reshub.h>

//=============================================================================
// Macros
//=============================================================================

#define UNREFERENCED_VAR(status) \
    status = status 

//-------------------------------------------------------------------------
// Description:
//
// If the condition evaluates to TRUE, jump to the given label.
//
// Parameters:
//
//      condition - [in] Code that fits in if statement
//      label - [in] label to jump if condition is met
//
#define IF_TRUE_JUMP(condition, label)                          \
    if (condition)                                               \
    {                                                           \
        goto label;                                             \
    }

//-------------------------------------------------------------------------
// Description:
//
// If the condition evaluates to TRUE, perform the given statement
// then jump to the given label.
//
// Parameters:
//
//      condition - [in] Code that fits in if statement
//      action - [in] action to perform in body of if statement
//      label - [in] label to jump if condition is met
//
#define IF_TRUE_ACTION_JUMP(condition, action, label)           \
    if (condition)                                               \
    {                                                           \
        action;                                                 \
        goto label;                                             \
    }
    
//-------------------------------------------------------------------------
// Description:
//
// If the ntStatus is not NT_SUCCESS, perform the given statement then jump to
// the given label.
//
// Parameters:
//
//      ntStatus - [in] Value to check
//      action - [in] action to perform in body of if statement
//      label - [in] label to jump if condition is met
//
#define IF_FAILED_ACTION_JUMP(ntStatus, action, label)          \
        if (!NT_SUCCESS(ntStatus))                              \
        {                                                       \
            action;                                             \
            goto label;                                         \
        }
    
//-------------------------------------------------------------------------
// Description:
//
// If the ntStatus passed is not NT_SUCCESS, jump to the given label.
//
// Parameters:
//
//      ntStatus - [in] Value to check
//      label - [in] label to jump if condition is met
//
#define IF_FAILED_JUMP(ntStatus, label)                         \
    if (!NT_SUCCESS(ntStatus))                                   \
    {                                                           \
        goto label;                                             \
    }

#define SAFE_RELEASE(p) {if (p) { (p)->Release(); (p) = nullptr; } }

// JACKDESC_RGB(r, g, b) 
#define JACKDESC_RGB(r, g, b) \
    ((COLORREF)((r << 16) | (g << 8) | (b)))

// Min/Max defines.
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

#define MINWAVERT_POOLTAG   'RWNM'
#define MINTOPORT_POOLTAG   'RTNM'
#define MINADAPTER_POOLTAG  'uAyS'

typedef enum
{
    eSpeakerHpDevice = 0,
    eMicInDevice,
    eHdmiRenderDevice,
    eMaxDeviceType
} eDeviceType;

//
// Signal processing modes and default formats structs.
//
typedef struct _MODE_AND_DEFAULT_FORMAT {
    GUID            Mode;
    KSDATAFORMAT*   DefaultFormat;
} MODE_AND_DEFAULT_FORMAT, *PMODE_AND_DEFAULT_FORMAT;

//
// Enumeration of the various types of pins implemented in this driver.
//
typedef enum
{
    NoPin,
    BridgePin,
    SystemRenderPin,
    SystemCapturePin,
} PINTYPE;

//
// PIN_DEVICE_FORMATS_AND_MODES
//
//  Used to specify a pin's type (e.g. system, offload, etc.), formats, and
//  modes. Conceptually serves similar purpose as the PCPIN_DESCRIPTOR to
//  define a pin, but is more specific to driver implementation.
//
//  Arrays of these structures follow the same order as the filter's
//  pin descriptor array so that KS pin IDs can serve as an index.
//
typedef struct _PIN_DEVICE_FORMATS_AND_MODES
{
    PINTYPE                             PinType;

    KSDATAFORMAT_WAVEFORMATEXTENSIBLE * WaveFormats;
    ULONG                               WaveFormatsCount;
    
    MODE_AND_DEFAULT_FORMAT *           ModeAndDefaultFormat;
    ULONG                               ModeAndDefaultFormatCount;

} PIN_DEVICE_FORMATS_AND_MODES, *PPIN_DEVICE_FORMATS_AND_MODES;

// forward declaration.
typedef struct _ENDPOINT_MINIPAIR *PENDPOINT_MINIPAIR;

// both wave & topology miniport create function prototypes have this form:
typedef HRESULT (*PFNCREATEMINIPORT)(
    _Out_           PUNKNOWN                              * Unknown,
    _In_            REFCLSID                                RefClsId,
    _In_opt_        PUNKNOWN                                UnknownOuter,
    _When_((PoolType & NonPagedPoolMustSucceed) != 0,
       __drv_reportError("Must succeed pool allocations are forbidden. "
			 "Allocation failures cause a system crash"))
    _In_            POOL_TYPE                               PoolType, 
    _In_            PUNKNOWN                                UnknownAdapter,
    _In_opt_        PVOID                                   DeviceContext,
    _In_            PENDPOINT_MINIPAIR                      MiniportPair
);

#define ENDPOINT_NO_FLAGS                   0x00000000

//
// Endpoint miniport pair (wave/topology) descriptor.
//
typedef struct _ENDPOINT_MINIPAIR 
{
    eDeviceType                     DeviceType;

    // Topology miniport.
    PWSTR                           TopoName;               // make sure this name matches with IMXWAV.<TopoName>.szPname in the inf's [Strings] section
    PFNCREATEMINIPORT               TopoCreateCallback;
    PCFILTER_DESCRIPTOR*            TopoDescriptor;

    // Wave RT miniport.
    PWSTR                           WaveName;               // make sure this name matches with IMXWAV.<WaveName>.szPname in the inf's [Strings] section
    PFNCREATEMINIPORT               WaveCreateCallback;
    PCFILTER_DESCRIPTOR*            WaveDescriptor;

    USHORT                          DeviceMaxChannels;
    PIN_DEVICE_FORMATS_AND_MODES*   PinDeviceFormatsAndModes;
    ULONG                           PinDeviceFormatsAndModesCount;
    
    // Miniport physical connections.
    PHYSICALCONNECTIONTABLE*        PhysicalConnections;
    ULONG                           PhysicalConnectionCount;

    // General endpoint flags (one of more ENDPOINT_<flag-type>, see above)
    ULONG                           DeviceFlags;

    
} ENDPOINT_MINIPAIR;

class CMiniportWaveRTStream;

// Event callback definition.
typedef VOID(*PFNEVENTNOTIFICATION)(
    _In_opt_    PVOID   Context
    );
// To support event notification.
struct EventCallback
{
    PFNEVENTNOTIFICATION    Handler;
    PVOID                   Context;
};

//=============================================================================
// Defines
//=============================================================================

DEFINE_GUID(IID_IAdapterCommon,
0x7eda2950, 0xbf9f, 0x11d0, 0x87, 0x1f, 0x0, 0xa0, 0xc9, 0x11, 0xb5, 0x44);

//=============================================================================
// Interfaces
//=============================================================================

///////////////////////////////////////////////////////////////////////////////
// IAdapterCommon
//
DECLARE_INTERFACE_(IAdapterCommon, IUnknown)
{
    STDMETHOD_(NTSTATUS,        Init) 
    ( 
        THIS_
        _In_  PDEVICE_OBJECT      DeviceObject,
        _In_  PRESOURCELIST       ResourceList
    ) PURE;

    STDMETHOD_(PDEVICE_OBJECT,  GetDeviceObject)
    (
        THIS
    ) PURE;
    
    STDMETHOD_(PDEVICE_OBJECT,  GetPhysicalDeviceObject)
    (
        THIS
    ) PURE;

    STDMETHOD_(WDFDEVICE,       GetWdfDevice)
    (
        THIS
    ) PURE;

    STDMETHOD_(VOID,            SetWaveServiceGroup) 
    ( 
        THIS_
        _In_ PSERVICEGROUP        ServiceGroup 
    ) PURE;

    STDMETHOD_(NTSTATUS,        WriteEtwEvent) 
    ( 
        THIS_ 
        _In_ EPcMiniportEngineEvent    MiniportEventType,
        _In_ ULONGLONG  Data1,
        _In_ ULONGLONG  Data2,
        _In_ ULONGLONG  Data3,
        _In_ ULONGLONG  Data4
    ) PURE;

    STDMETHOD_(VOID,            SetEtwHelper) 
    ( 
        THIS_
        PPORTCLSETWHELPER PortClsEtwHelper
    ) PURE;
    
    STDMETHOD_(NTSTATUS,        InstallSubdevice)
    ( 
        _In_opt_        PIRP                                    Irp,
        _In_            PWSTR                                   Name,
        _In_            REFGUID                                 PortClassId,
        _In_            REFGUID                                 MiniportClassId,
        _In_opt_        PFNCREATEMINIPORT                       MiniportCreate,
        _In_opt_        PVOID                                   DeviceContext,
        _In_            PENDPOINT_MINIPAIR                      MiniportPair,
        _In_opt_        PRESOURCELIST                           ResourceList,
        _In_            REFGUID                                 PortInterfaceId,
        _Out_opt_       PUNKNOWN                              * OutPortInterface,
        _Out_opt_       PUNKNOWN                              * OutPortUnknown,
        _Out_opt_       PUNKNOWN                              * OutMiniportUnknown
    );    

    STDMETHOD_(NTSTATUS,        UnregisterSubdevice)
    (
        THIS_
        _In_opt_   PUNKNOWN     UnknownPort
    );

    STDMETHOD_(NTSTATUS,        ConnectTopologies)
    (
        THIS_
        _In_ PUNKNOWN                   UnknownTopology,
        _In_ PUNKNOWN                   UnknownWave,
        _In_ PHYSICALCONNECTIONTABLE*   PhysicalConnections,
        _In_ ULONG                      PhysicalConnectionCount
    );

    STDMETHOD_(NTSTATUS,        DisconnectTopologies)
    (
        THIS_
        _In_ PUNKNOWN                   UnknownTopology,
        _In_ PUNKNOWN                   UnknownWave,
        _In_ PHYSICALCONNECTIONTABLE*   PhysicalConnections,
        _In_ ULONG                      PhysicalConnectionCount
    );

    STDMETHOD_(NTSTATUS,        InstallEndpointFilters)
    (
        THIS_
        _In_opt_    PIRP                Irp, 
        _In_        PENDPOINT_MINIPAIR  MiniportPair,
        _In_opt_    PVOID               DeviceContext,
        _Out_opt_   PUNKNOWN *          UnknownTopology,
        _Out_opt_   PUNKNOWN *          UnknownWave
    );

    STDMETHOD_(NTSTATUS,        RemoveEndpointFilters)
    (
        THIS_
        _In_        PENDPOINT_MINIPAIR  MiniportPair,
        _In_opt_    PUNKNOWN            UnknownTopology,
        _In_opt_    PUNKNOWN            UnknownWave
    );

    STDMETHOD_(NTSTATUS,        GetFilters)
    (
        THIS_
        _In_        PENDPOINT_MINIPAIR  MiniportPair,
        _Out_opt_   PUNKNOWN            *UnknownTopologyPort,
        _Out_opt_   PUNKNOWN            *UnknownTopologyMiniport,
        _Out_opt_   PUNKNOWN            *UnknownWavePort,
        _Out_opt_   PUNKNOWN            *UnknownWaveMiniport
    );

    STDMETHOD_(NTSTATUS,        SetIdlePowerManagement)
    (
        THIS_
        _In_        PENDPOINT_MINIPAIR  MiniportPair,
        _In_        BOOL                Enable
    );

    STDMETHOD_(NTSTATUS,        RegisterStream)
    (
        THIS_
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    );

    STDMETHOD_(NTSTATUS,        UnregisterStream)
    (
        THIS_
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    );

    STDMETHOD_(NTSTATUS, FreeBuffer)
    (
        THIS_
        _In_        CMiniportWaveRTStream *Stream,
                    eDeviceType            DeviceType,
        _In_        PMDL                   Mdl,
        _In_        ULONG                  Size
    );

    STDMETHOD_(NTSTATUS, AllocBuffer)
    (
        THIS_
        _In_        CMiniportWaveRTStream *Stream,
                    eDeviceType            DeviceType,
        _Inout_     PULONG                 Size,
        _Out_       PMDL                  *pMdl,
        _Out_       MEMORY_CACHING_TYPE   *CacheType
    );
    
    STDMETHOD_(NTSTATUS,        StartDma)
    (
        THIS_
        _In_        CMiniportWaveRTStream* Stream
    );

    STDMETHOD_(NTSTATUS,        StopDma)
    (
        THIS_
        _In_        CMiniportWaveRTStream* Stream
    );

    STDMETHOD_(NTSTATUS,        PauseDma)
    (
        THIS_
        _In_        CMiniportWaveRTStream* Stream
    );

    STDMETHOD_(VOID, SetConnectionStatusHandler)
    (
        THIS_
        _In_opt_    PFNEVENTNOTIFICATION    EventHandler,
        _In_opt_    PVOID                   EventHandlerContext
    );

    STDMETHOD_(BOOL, GetConnectionStatus)
        (VOID);

    STDMETHOD_(VOID, GetContainerId)
    (
        THIS_
        _In_        ULONG                   nPinId,
        _Out_       GUID*                   ContainerId
    );

    STDMETHOD_(VOID, UpdateSinkInfo)
    (
        THIS_
        _In_        ULONG                       nPinId,
        _Out_       PKSJACK_SINK_INFORMATION    SinkInfo
    );
    STDMETHOD_(BOOL, isRenderSupported)
        (VOID);

    STDMETHOD_(BOOL, isCaptureSupported)
        (VOID);

};

typedef IAdapterCommon *PADAPTERCOMMON;

//=============================================================================
// Function Prototypes
//=============================================================================
NTSTATUS
NewAdapterCommon
( 
    _Out_       PUNKNOWN *              Unknown,
    _In_        REFCLSID                RefClsId,
    _In_opt_    PUNKNOWN                UnknownOuter,
    _When_((PoolType & NonPagedPoolMustSucceed) != 0,
        __drv_reportError("Must succeed pool allocations are forbidden. "
			    "Allocation failures cause a system crash"))
    _In_        POOL_TYPE               PoolType 
);


//
// Wrap the PNP_FUNCTION_CONFIG_DESCRIPTOR to make table parsing a little easier.
//
typedef struct  {
    PNP_FUNCTION_CONFIG_DESCRIPTOR Config;

    //
    // ...followed by Pin table (USHORT)
    // ...followed RESOURCE_NAME string
    // ...followed by optional vendor data.
    //
    // Note the offsets in the fixed block should be used to access this data.
    //

    union {
        USHORT AudMuxPin[2]; // element [0] is the SsiPort, element [1] is the external port.
    } u;

} AUDIO_PNP_FUNCTION_CONFIG_DESCRIPTOR, *PAUDIO_PNP_FUNCTION_CONFIG_DESCRIPTOR;
