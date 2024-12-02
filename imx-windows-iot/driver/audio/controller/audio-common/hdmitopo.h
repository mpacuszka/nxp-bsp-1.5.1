/* Copyright (c) Microsoft Corporation All Rights Reserved
   Copyright 2023 NXP
   Licensed under the MIT License.

Abstract:
    Declaration of topology miniport for the hdmi endpoint.

--*/

#ifndef _HDMITOPO_H_
#define _HDMITOPO_H_

#include "basetopo.h"

//=============================================================================
// Classes
//=============================================================================

///////////////////////////////////////////////////////////////////////////////
// CHdmiMiniportTopology 
//   

class CHdmiMiniportTopology :
    public CMiniportTopologyIMXWAV,
    public IMiniportTopology,
    public CUnknown
{
  public:
    DECLARE_STD_UNKNOWN();
    CHdmiMiniportTopology
    (
        _In_opt_    PUNKNOWN                UnknownOuter,
        _In_        PCFILTER_DESCRIPTOR    *FilterDesc,
        _In_        USHORT                  DeviceMaxChannels
    ) 
    : CUnknown(UnknownOuter), 
      CMiniportTopologyIMXWAV(FilterDesc, DeviceMaxChannels)
    {}
    
    ~CHdmiMiniportTopology();

    IMP_IMiniportTopology;

    NTSTATUS            PropertyHandlerJackContainerId
    (
        _In_ PPCPROPERTY_REQUEST  PropertyRequest
    );

    NTSTATUS            PropertyHandlerJackSinkInfo
    (
        _In_ PPCPROPERTY_REQUEST  PropertyRequest
    );
    
    NTSTATUS            PropertyHandlerJackDescription
    (
        _In_ PPCPROPERTY_REQUEST  PropertyRequest
    );

    NTSTATUS            PropertyHandlerJackDescription2
    (
        _In_ PPCPROPERTY_REQUEST  PropertyRequest
    );

    static VOID
    EvtConnectionStatusHandler
    (
        _In_opt_    PVOID   Context
    );
};
    
typedef CHdmiMiniportTopology *PCHdmiMiniportTopology;


NTSTATUS
CreateHdmiMiniportTopology
(
    _Out_           PUNKNOWN                              * Unknown,
    _In_            REFCLSID,
    _In_opt_        PUNKNOWN                                UnknownOuter,
    _In_            POOL_TYPE                               PoolType,
    _In_            PUNKNOWN                                UnknownAdapter,
    _In_opt_        PVOID                                   DeviceContext,
    _In_            PENDPOINT_MINIPAIR                      MiniportPair
);

NTSTATUS PropertyHandler_HdmiTopoFilter
(
    _In_ PPCPROPERTY_REQUEST PropertyRequest
);

NTSTATUS EventHandler_HdmiTopoFilter
(
    _In_ PPCEVENT_REQUEST EventRequest
);


#endif // _HDMITOPO_H_

