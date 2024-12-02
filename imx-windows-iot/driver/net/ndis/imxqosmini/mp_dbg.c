/*
* Copyright 2018-2020, 2022 NXP
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* * Neither the name of NXP nor the names of its contributors may be used to
* endorse or promote products derived from this software without specific prior
* written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "precomp.h"

#if DBG

LARGE_INTEGER        DriverStartTime;

/*++
Routine Description:
    .
Argument:
    NdisRequest     OID
    Status          NDIS status
Return Value:
    None
--*/
_Use_decl_annotations_
void Dbg_PrintOidDescription(PMP_ADAPTER pAdapter, PNDIS_OID_REQUEST NdisRequest, NTSTATUS Status) {
    NDIS_OID  Oid = NdisRequest->DATA.QUERY_INFORMATION.Oid;
    PCHAR     oidType = NULL;
    PCHAR     oidName = NULL;
    CHAR      Buffer[256];

    UNREFERENCED_PARAMETER(pAdapter);
    UNREFERENCED_PARAMETER(Status);
    if ((Oid == OID_GEN_STATISTICS) || (Oid == OID_IP6_OFFLOAD_STATS) || (Oid == OID_IP4_OFFLOAD_STATS) || (Oid == OID_GEN_NETWORK_LAYER_ADDRESSES)) {
        return;
    }
    switch (NdisRequest->RequestType) {
        case NdisRequestQueryInformation: oidType = "GET_INFO"; break;
        case NdisRequestSetInformation:   oidType = "SET_INFO"; break;
        case NdisRequestQueryStatistics:  oidType = "GET_STAT"; break;
        case NdisRequestMethod:           oidType = "REQ_MTHD"; break;
        default:                          oidType = "UNKNOWN "; break;
    }
    #undef MAKECASE
    #define MAKECASE(oidName, oidx) case oidx: oidName = #oidx; break;
    switch (Oid) {
        /* Operational OIDs */
        MAKECASE(oidName, OID_GEN_SUPPORTED_LIST)
        MAKECASE(oidName, OID_GEN_HARDWARE_STATUS)
        MAKECASE(oidName, OID_GEN_MEDIA_SUPPORTED)
        MAKECASE(oidName, OID_GEN_MEDIA_IN_USE)
        MAKECASE(oidName, OID_GEN_MAXIMUM_LOOKAHEAD)
        MAKECASE(oidName, OID_GEN_MAXIMUM_FRAME_SIZE)
        MAKECASE(oidName, OID_GEN_LINK_SPEED)
        MAKECASE(oidName, OID_GEN_TRANSMIT_BUFFER_SPACE)
        MAKECASE(oidName, OID_GEN_RECEIVE_BUFFER_SPACE)
        MAKECASE(oidName, OID_GEN_TRANSMIT_BLOCK_SIZE)
        MAKECASE(oidName, OID_GEN_RECEIVE_BLOCK_SIZE)
        MAKECASE(oidName, OID_GEN_VENDOR_ID)
        MAKECASE(oidName, OID_GEN_VENDOR_DESCRIPTION)
        MAKECASE(oidName, OID_GEN_VENDOR_DRIVER_VERSION)
        MAKECASE(oidName, OID_GEN_CURRENT_PACKET_FILTER)
        MAKECASE(oidName, OID_GEN_CURRENT_LOOKAHEAD)
        MAKECASE(oidName, OID_GEN_DRIVER_VERSION)
        MAKECASE(oidName, OID_GEN_MAXIMUM_TOTAL_SIZE)
        MAKECASE(oidName, OID_GEN_PROTOCOL_OPTIONS)
        MAKECASE(oidName, OID_GEN_MAC_OPTIONS)
        MAKECASE(oidName, OID_GEN_MEDIA_CONNECT_STATUS)
        MAKECASE(oidName, OID_GEN_MAXIMUM_SEND_PACKETS)
        MAKECASE(oidName, OID_GEN_SUPPORTED_GUIDS)
        MAKECASE(oidName, OID_GEN_NETWORK_LAYER_ADDRESSES)
        MAKECASE(oidName, OID_GEN_TRANSPORT_HEADER_OFFSET)
        MAKECASE(oidName, OID_GEN_MEDIA_CAPABILITIES)
        MAKECASE(oidName, OID_GEN_PHYSICAL_MEDIUM)
        MAKECASE(oidName, OID_GEN_MACHINE_NAME)
        MAKECASE(oidName, OID_GEN_VLAN_ID)
        MAKECASE(oidName, OID_GEN_RNDIS_CONFIG_PARAMETER)

        /* Operational OIDs for NDIS 6.0 */
        MAKECASE(oidName, OID_GEN_MAX_LINK_SPEED)
        MAKECASE(oidName, OID_GEN_LINK_STATE)
        MAKECASE(oidName, OID_GEN_LINK_PARAMETERS)
        MAKECASE(oidName, OID_GEN_MINIPORT_RESTART_ATTRIBUTES)
        MAKECASE(oidName, OID_GEN_ENUMERATE_PORTS)
        MAKECASE(oidName, OID_GEN_PORT_STATE)
        MAKECASE(oidName, OID_GEN_PORT_AUTHENTICATION_PARAMETERS)
        MAKECASE(oidName, OID_GEN_INTERRUPT_MODERATION)
        MAKECASE(oidName, OID_GEN_PHYSICAL_MEDIUM_EX)

        /* Statistical OIDs */
        MAKECASE(oidName, OID_GEN_XMIT_OK)
        MAKECASE(oidName, OID_GEN_RCV_OK)
        MAKECASE(oidName, OID_GEN_XMIT_ERROR)
        MAKECASE(oidName, OID_GEN_RCV_ERROR)
        MAKECASE(oidName, OID_GEN_RCV_NO_BUFFER)
        MAKECASE(oidName, OID_GEN_DIRECTED_BYTES_XMIT)
        MAKECASE(oidName, OID_GEN_DIRECTED_FRAMES_XMIT)
        MAKECASE(oidName, OID_GEN_MULTICAST_BYTES_XMIT)
        MAKECASE(oidName, OID_GEN_MULTICAST_FRAMES_XMIT)
        MAKECASE(oidName, OID_GEN_BROADCAST_BYTES_XMIT)
        MAKECASE(oidName, OID_GEN_BROADCAST_FRAMES_XMIT)
        MAKECASE(oidName, OID_GEN_DIRECTED_BYTES_RCV)
        MAKECASE(oidName, OID_GEN_DIRECTED_FRAMES_RCV)
        MAKECASE(oidName, OID_GEN_MULTICAST_BYTES_RCV)
        MAKECASE(oidName, OID_GEN_MULTICAST_FRAMES_RCV)
        MAKECASE(oidName, OID_GEN_BROADCAST_BYTES_RCV)
        MAKECASE(oidName, OID_GEN_BROADCAST_FRAMES_RCV)
        MAKECASE(oidName, OID_GEN_RCV_CRC_ERROR)
        MAKECASE(oidName, OID_GEN_TRANSMIT_QUEUE_LENGTH)

        /* Statistical OIDs for NDIS 6.0 */
        MAKECASE(oidName, OID_GEN_STATISTICS)
        MAKECASE(oidName, OID_GEN_BYTES_RCV)
        MAKECASE(oidName, OID_GEN_BYTES_XMIT)
        MAKECASE(oidName, OID_GEN_RCV_DISCARDS)
        MAKECASE(oidName, OID_GEN_XMIT_DISCARDS)

        /* Misc OIDs */
        MAKECASE(oidName, OID_GEN_GET_TIME_CAPS)
        MAKECASE(oidName, OID_GEN_GET_NETCARD_TIME)
        MAKECASE(oidName, OID_GEN_NETCARD_LOAD)
        MAKECASE(oidName, OID_GEN_DEVICE_PROFILE)
        MAKECASE(oidName, OID_GEN_INIT_TIME_MS)
        MAKECASE(oidName, OID_GEN_RESET_COUNTS)
        MAKECASE(oidName, OID_GEN_MEDIA_SENSE_COUNTS)

        /* PnP power management operational OIDs */
        MAKECASE(oidName, OID_PNP_CAPABILITIES)
        MAKECASE(oidName, OID_PNP_SET_POWER)
        MAKECASE(oidName, OID_PNP_QUERY_POWER)
        MAKECASE(oidName, OID_PNP_ADD_WAKE_UP_PATTERN)
        MAKECASE(oidName, OID_PNP_REMOVE_WAKE_UP_PATTERN)
        MAKECASE(oidName, OID_PNP_ENABLE_WAKE_UP)
        MAKECASE(oidName, OID_PNP_WAKE_UP_PATTERN_LIST)

        /* PnP power management statistical OIDs */
        MAKECASE(oidName, OID_PNP_WAKE_UP_ERROR)
        MAKECASE(oidName, OID_PNP_WAKE_UP_OK)

        /* Ethernet operational OIDs */
        MAKECASE(oidName, OID_802_3_PERMANENT_ADDRESS)
        MAKECASE(oidName, OID_802_3_CURRENT_ADDRESS)
        MAKECASE(oidName, OID_802_3_MULTICAST_LIST)
        MAKECASE(oidName, OID_802_3_MAXIMUM_LIST_SIZE)
        MAKECASE(oidName, OID_802_3_MAC_OPTIONS)

        /* Ethernet operational OIDs for NDIS 6.0 */
        MAKECASE(oidName, OID_802_3_ADD_MULTICAST_ADDRESS)
        MAKECASE(oidName, OID_802_3_DELETE_MULTICAST_ADDRESS)

        /* Ethernet statistical OIDs */
        MAKECASE(oidName, OID_802_3_RCV_ERROR_ALIGNMENT)
        MAKECASE(oidName, OID_802_3_XMIT_ONE_COLLISION)
        MAKECASE(oidName, OID_802_3_XMIT_MORE_COLLISIONS)
        MAKECASE(oidName, OID_802_3_XMIT_DEFERRED)
        MAKECASE(oidName, OID_802_3_XMIT_MAX_COLLISIONS)
        MAKECASE(oidName, OID_802_3_RCV_OVERRUN)
        MAKECASE(oidName, OID_802_3_XMIT_UNDERRUN)
        MAKECASE(oidName, OID_802_3_XMIT_HEARTBEAT_FAILURE)
        MAKECASE(oidName, OID_802_3_XMIT_TIMES_CRS_LOST)
        MAKECASE(oidName, OID_802_3_XMIT_LATE_COLLISIONS)

        /*  TCP/IP OIDs */
        MAKECASE(oidName, OID_TCP_TASK_OFFLOAD)
        MAKECASE(oidName, OID_TCP_TASK_IPSEC_ADD_SA)
        MAKECASE(oidName, OID_TCP_TASK_IPSEC_DELETE_SA)
        MAKECASE(oidName, OID_TCP_SAN_SUPPORT)
        MAKECASE(oidName, OID_TCP_TASK_IPSEC_ADD_UDPESP_SA)
        MAKECASE(oidName, OID_TCP_TASK_IPSEC_DELETE_UDPESP_SA)
        MAKECASE(oidName, OID_TCP4_OFFLOAD_STATS)
        MAKECASE(oidName, OID_TCP6_OFFLOAD_STATS)
        MAKECASE(oidName, OID_IP4_OFFLOAD_STATS)
        MAKECASE(oidName, OID_IP6_OFFLOAD_STATS)

        /* TCP offload OIDs for NDIS 6 */
        MAKECASE(oidName, OID_TCP_OFFLOAD_CURRENT_CONFIG)
        MAKECASE(oidName, OID_TCP_OFFLOAD_PARAMETERS)
        MAKECASE(oidName, OID_TCP_OFFLOAD_HARDWARE_CAPABILITIES)
        MAKECASE(oidName, OID_TCP_CONNECTION_OFFLOAD_CURRENT_CONFIG)
        MAKECASE(oidName, OID_TCP_CONNECTION_OFFLOAD_HARDWARE_CAPABILITIES)
        MAKECASE(oidName, OID_OFFLOAD_ENCAPSULATION)

        #if (NDIS_SUPPORT_NDIS620)
        /* VMQ OIDs for NDIS 6.20 */
        MAKECASE(oidName, OID_RECEIVE_FILTER_FREE_QUEUE)
        MAKECASE(oidName, OID_RECEIVE_FILTER_CLEAR_FILTER)
        MAKECASE(oidName, OID_RECEIVE_FILTER_ALLOCATE_QUEUE)
        MAKECASE(oidName, OID_RECEIVE_FILTER_QUEUE_ALLOCATION_COMPLETE)
        MAKECASE(oidName, OID_RECEIVE_FILTER_SET_FILTER)
        #endif

        #if (NDIS_SUPPORT_NDIS630)
        /* NDIS QoS OIDs for NDIS 6.30 */
        MAKECASE(oidName, OID_QOS_PARAMETERS)
        #endif
    }
    if (!oidType){
        RtlStringCbPrintfA(Buffer, sizeof(Buffer), "Unknown OID type: 0x%08X", Oid);
    } else if (!oidName) {
        RtlStringCbPrintfA(Buffer, sizeof(Buffer), "Unknown OID 0x%08X", Oid);
    } else  {
        RtlStringCbPrintfA(Buffer, sizeof(Buffer), "%s, %s", oidType, oidName);
    }
    DBG_ENET_DEV_OIDS_PRINT_INFO("%s, status: 0x%08X", Buffer, Status);
}


#define RETAILMSG(_zone_,...)         DbgPrintEx(DPFLTR_IHVDRIVER_ID,DPFLTR_INFO_LEVEL,__VA_ARGS__)
#define WSTRINGER(_x)   L ## #_x
#define FORMAT_REGNAME_REGVALUE(_x)    "[%04X] %s\t= %#08x\n", (int)offsetof(CSP_ENET_REGS,_x),#_x, (pAdapter->ENETRegBase->_x)

#undef MAKECASE
#undef MAKECASE1
#undef MAKEDEFAULT
#define MAKECASE(Value) case Value:  return #Value;
#define MAKECASE1(Value,Name) case Value:  return #Name;
#define MAKEDEFAULT(Message) default: return"!!! "Message" name unknown !!!";

/*++
Routine Description:
    Get state machine name
Argument:
    MpState     State machine state
Return Value:
    Pointer to state name
--*/
_Use_decl_annotations_
const char* Dbg_GetMpStateMachineStateName(MP_STATE i) {
    switch (i) {
        MAKECASE1(SM_STATE_HALTED,       HALTED)
        MAKECASE1(SM_STATE_SHUTDOWN,     SHUTDOWN)
        MAKECASE1(SM_STATE_INITIALIZING, INITIALIZING)
        MAKECASE1(SM_STATE_PAUSED,       PAUSED)
        MAKECASE1(SM_STATE_PAUSING,      PAUSING)
        MAKECASE1(SM_STATE_RESTARTING,   RESTARTING)
        MAKECASE1(SM_STATE_RUNNING,      RUNNING)
        MAKECASE1(SM_STATE_RESET,        RESET)
        MAKECASE1(SM_STATE_ERROR,        ERROR)
        MAKEDEFAULT("State machine state")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetMpMediaConnectStateName(NDIS_MEDIA_CONNECT_STATE i) {
    switch (i) {
        MAKECASE1(MediaConnectStateUnknown,      UNKNOWN)
        MAKECASE1(MediaConnectStateDisconnected, DISCONNECTED)
        MAKECASE1(MediaConnectStateConnected,    CONNECTED)
        MAKEDEFAULT("Media connect state")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetMpMediaDuplexStateName(NDIS_MEDIA_CONNECT_STATE i) {
    switch (i) {
        MAKECASE1(MediaDuplexStateUnknown, UNKNOWN)
        MAKECASE1(MediaDuplexStateHalf,    HALF DUPLEX)
        MAKECASE1(MediaDuplexStateFull,    FULL DUPLEX)
        MAKEDEFAULT("Media duplex state")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetPhyStateEngineStateName(MP_PHY_DEVICE_STATE i) {
    switch (i) {
        MAKECASE(MP_PHY_STATE_NOT_INITIALIZED)
        MAKECASE(MP_PHY_STATE_INITIALIZED)
        MAKECASE(MP_PHY_STATE_DEFAULT_CFG_RAED)
        MAKECASE(MP_PHY_STATE_LINK_UP)
        MAKECASE(MP_PHY_STATE_LINK_DOWN_TO_UP)
        MAKECASE(MP_PHY_STATE_LINK_DOWN)
        MAKEDEFAULT("Unknown state")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetNdisPowerStateName(NDIS_DEVICE_POWER_STATE i) {
    switch (i) {
        MAKECASE1(NdisDeviceStateUnspecified, NdisDeviceStateUnspecified)
        MAKECASE1(NdisDeviceStateD0,          NdisDeviceStateD0)
        MAKECASE1(NdisDeviceStateD1,          NdisDeviceStateD1)
        MAKECASE1(NdisDeviceStateD2,          NdisDeviceStateD2)
        MAKECASE1(NdisDeviceStateD3,          NdisDeviceStateD3)
        MAKECASE1(NdisDeviceStateMaximum,     NdisDeviceStateMaximum)
        MAKEDEFAULT("NDIS power state")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetNdisStatusName(NDIS_STATUS i) {
    switch (i) {
        MAKECASE(NDIS_STATUS_SUCCESS)
        MAKECASE(NDIS_STATUS_RESOURCES)
        MAKECASE(NDIS_STATUS_MEDIA_DISCONNECTED)
        MAKECASE(NDIS_STATUS_PAUSED)
        MAKECASE(NDIS_STATUS_REQUEST_ABORTED)
        MAKECASE(NDIS_STATUS_LOW_POWER_STATE)
        MAKECASE(NDIS_STATUS_BUFFER_TOO_SHORT)
        MAKECASE(NDIS_STATUS_NOT_SUPPORTED)
        MAKECASE(NDIS_STATUS_INVALID_DATA)
        MAKECASE(NDIS_STATUS_FAILURE)
        MAKECASE(NDIS_STATUS_RESOURCE_CONFLICT)
        MAKECASE(NDIS_ERROR_CODE_RESOURCE_CONFLICT)
        MAKECASE(STATUS_ACPI_INVALID_DATA)
        MAKECASE(STATUS_TIMEOUT)
        MAKECASE(STATUS_OBJECT_NAME_NOT_FOUND)
        MAKEDEFAULT("NDIS status")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetNdisOidRequestTypeName(NDIS_REQUEST_TYPE i) {
    switch (i) {
        MAKECASE(NdisRequestQueryInformation)
        MAKECASE(NdisRequestSetInformation)
        MAKECASE(NdisRequestQueryStatistics)
        MAKEDEFAULT("NDIS OID request type")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetNdisOidName(NDIS_OID i) {

    switch (i) {
        /* Operational OIDs */
        MAKECASE(OID_GEN_SUPPORTED_LIST)
        MAKECASE(OID_GEN_HARDWARE_STATUS)
        MAKECASE(OID_GEN_MEDIA_SUPPORTED)
        MAKECASE(OID_GEN_MEDIA_IN_USE)
        MAKECASE(OID_GEN_MAXIMUM_LOOKAHEAD)
        MAKECASE(OID_GEN_MAXIMUM_FRAME_SIZE)
        MAKECASE(OID_GEN_LINK_SPEED)
        MAKECASE(OID_GEN_TRANSMIT_BUFFER_SPACE)
        MAKECASE(OID_GEN_RECEIVE_BUFFER_SPACE)
        MAKECASE(OID_GEN_TRANSMIT_BLOCK_SIZE)
        MAKECASE(OID_GEN_RECEIVE_BLOCK_SIZE)
        MAKECASE(OID_GEN_VENDOR_ID)
        MAKECASE(OID_GEN_VENDOR_DESCRIPTION)
        MAKECASE(OID_GEN_VENDOR_DRIVER_VERSION)
        MAKECASE(OID_GEN_CURRENT_PACKET_FILTER)
        MAKECASE(OID_GEN_CURRENT_LOOKAHEAD)
        MAKECASE(OID_GEN_DRIVER_VERSION)
        MAKECASE(OID_GEN_MAXIMUM_TOTAL_SIZE)
        MAKECASE(OID_GEN_PROTOCOL_OPTIONS)
        MAKECASE(OID_GEN_MAC_OPTIONS)
        MAKECASE(OID_GEN_MEDIA_CONNECT_STATUS)
        MAKECASE(OID_GEN_MAXIMUM_SEND_PACKETS)
        MAKECASE(OID_GEN_SUPPORTED_GUIDS)
        MAKECASE(OID_GEN_NETWORK_LAYER_ADDRESSES)
        MAKECASE(OID_GEN_TRANSPORT_HEADER_OFFSET)
        MAKECASE(OID_GEN_MEDIA_CAPABILITIES)
        MAKECASE(OID_GEN_PHYSICAL_MEDIUM)
        MAKECASE(OID_GEN_MACHINE_NAME)
        MAKECASE(OID_GEN_VLAN_ID)
        MAKECASE(OID_GEN_RNDIS_CONFIG_PARAMETER)

        /* Operational OIDs for NDIS 6.0 */
        MAKECASE(OID_GEN_MAX_LINK_SPEED)
        MAKECASE(OID_GEN_LINK_STATE)
        MAKECASE(OID_GEN_LINK_PARAMETERS)
        MAKECASE(OID_GEN_MINIPORT_RESTART_ATTRIBUTES)
        MAKECASE(OID_GEN_ENUMERATE_PORTS)
        MAKECASE(OID_GEN_PORT_STATE)
        MAKECASE(OID_GEN_PORT_AUTHENTICATION_PARAMETERS)
        MAKECASE(OID_GEN_INTERRUPT_MODERATION)
        MAKECASE(OID_GEN_PHYSICAL_MEDIUM_EX)

        /* Statistical OIDs */
        MAKECASE(OID_GEN_XMIT_OK)
        MAKECASE(OID_GEN_RCV_OK)
        MAKECASE(OID_GEN_XMIT_ERROR)
        MAKECASE(OID_GEN_RCV_ERROR)
        MAKECASE(OID_GEN_RCV_NO_BUFFER)
        MAKECASE(OID_GEN_DIRECTED_BYTES_XMIT)
        MAKECASE(OID_GEN_DIRECTED_FRAMES_XMIT)
        MAKECASE(OID_GEN_MULTICAST_BYTES_XMIT)
        MAKECASE(OID_GEN_MULTICAST_FRAMES_XMIT)
        MAKECASE(OID_GEN_BROADCAST_BYTES_XMIT)
        MAKECASE(OID_GEN_BROADCAST_FRAMES_XMIT)
        MAKECASE(OID_GEN_DIRECTED_BYTES_RCV)
        MAKECASE(OID_GEN_DIRECTED_FRAMES_RCV)
        MAKECASE(OID_GEN_MULTICAST_BYTES_RCV)
        MAKECASE(OID_GEN_MULTICAST_FRAMES_RCV)
        MAKECASE(OID_GEN_BROADCAST_BYTES_RCV)
        MAKECASE(OID_GEN_BROADCAST_FRAMES_RCV)
        MAKECASE(OID_GEN_RCV_CRC_ERROR)
        MAKECASE(OID_GEN_TRANSMIT_QUEUE_LENGTH)

        /* Statistical OIDs for NDIS 6.0 */
        MAKECASE(OID_GEN_STATISTICS)
        MAKECASE(OID_GEN_BYTES_RCV)
        MAKECASE(OID_GEN_BYTES_XMIT)
        MAKECASE(OID_GEN_RCV_DISCARDS)
        MAKECASE(OID_GEN_XMIT_DISCARDS)

        /* Misc OIDs */
        MAKECASE(OID_GEN_GET_TIME_CAPS)
        MAKECASE(OID_GEN_GET_NETCARD_TIME)
        MAKECASE(OID_GEN_NETCARD_LOAD)
        MAKECASE(OID_GEN_DEVICE_PROFILE)
        MAKECASE(OID_GEN_INIT_TIME_MS)
        MAKECASE(OID_GEN_RESET_COUNTS)
        MAKECASE(OID_GEN_MEDIA_SENSE_COUNTS)

        /* PnP power management operational OIDs */
        MAKECASE(OID_PNP_CAPABILITIES)
        MAKECASE(OID_PNP_SET_POWER)
        MAKECASE(OID_PNP_QUERY_POWER)
        MAKECASE(OID_PNP_ADD_WAKE_UP_PATTERN)
        MAKECASE(OID_PNP_REMOVE_WAKE_UP_PATTERN)
        MAKECASE(OID_PNP_ENABLE_WAKE_UP)
        MAKECASE(OID_PNP_WAKE_UP_PATTERN_LIST)

        /* PnP power management statistical OIDs */
        MAKECASE(OID_PNP_WAKE_UP_ERROR)
        MAKECASE(OID_PNP_WAKE_UP_OK)

        /* Ethernet operational OIDs */
        MAKECASE(OID_802_3_PERMANENT_ADDRESS)
        MAKECASE(OID_802_3_CURRENT_ADDRESS)
        MAKECASE(OID_802_3_MULTICAST_LIST)
        MAKECASE(OID_802_3_MAXIMUM_LIST_SIZE)
        MAKECASE(OID_802_3_MAC_OPTIONS)

        /* Ethernet operational OIDs for NDIS 6.0 */
        MAKECASE(OID_802_3_ADD_MULTICAST_ADDRESS)
        MAKECASE(OID_802_3_DELETE_MULTICAST_ADDRESS)

        /* Ethernet statistical OIDs */
        MAKECASE(OID_802_3_RCV_ERROR_ALIGNMENT)
        MAKECASE(OID_802_3_XMIT_ONE_COLLISION)
        MAKECASE(OID_802_3_XMIT_MORE_COLLISIONS)
        MAKECASE(OID_802_3_XMIT_DEFERRED)
        MAKECASE(OID_802_3_XMIT_MAX_COLLISIONS)
        MAKECASE(OID_802_3_RCV_OVERRUN)
        MAKECASE(OID_802_3_XMIT_UNDERRUN)
        MAKECASE(OID_802_3_XMIT_HEARTBEAT_FAILURE)
        MAKECASE(OID_802_3_XMIT_TIMES_CRS_LOST)
        MAKECASE(OID_802_3_XMIT_LATE_COLLISIONS)

        /*  TCP/IP OIDs */
        MAKECASE(OID_TCP_TASK_OFFLOAD)
        MAKECASE(OID_TCP_TASK_IPSEC_ADD_SA)
        MAKECASE(OID_TCP_TASK_IPSEC_DELETE_SA)
        MAKECASE(OID_TCP_SAN_SUPPORT)
        MAKECASE(OID_TCP_TASK_IPSEC_ADD_UDPESP_SA)
        MAKECASE(OID_TCP_TASK_IPSEC_DELETE_UDPESP_SA)
        MAKECASE(OID_TCP4_OFFLOAD_STATS)
        MAKECASE(OID_TCP6_OFFLOAD_STATS)
        MAKECASE(OID_IP4_OFFLOAD_STATS)
        MAKECASE(OID_IP6_OFFLOAD_STATS)

        /* TCP offload OIDs for NDIS 6 */
        MAKECASE(OID_TCP_OFFLOAD_CURRENT_CONFIG)
        MAKECASE(OID_TCP_OFFLOAD_PARAMETERS)
        MAKECASE(OID_TCP_OFFLOAD_HARDWARE_CAPABILITIES)
        MAKECASE(OID_TCP_CONNECTION_OFFLOAD_CURRENT_CONFIG)
        MAKECASE(OID_TCP_CONNECTION_OFFLOAD_HARDWARE_CAPABILITIES)
        MAKECASE(OID_OFFLOAD_ENCAPSULATION)

        #if (NDIS_SUPPORT_NDIS620)
        /* VMQ OIDs for NDIS 6.20 */
        MAKECASE(OID_RECEIVE_FILTER_FREE_QUEUE)
        MAKECASE(OID_RECEIVE_FILTER_CLEAR_FILTER)
        MAKECASE(OID_RECEIVE_FILTER_ALLOCATE_QUEUE)
        MAKECASE(OID_RECEIVE_FILTER_QUEUE_ALLOCATION_COMPLETE)
        MAKECASE(OID_RECEIVE_FILTER_SET_FILTER)
        #endif

        #if (NDIS_SUPPORT_NDIS630)
        /* NDIS QoS OIDs for NDIS 6.30 */
        MAKECASE(OID_QOS_PARAMETERS)
        #endif

        MAKEDEFAULT("NDIS OID request")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetACPIFunctionName(ULONG i) {
    switch (i) {
        MAKECASE1(IMX_ENET_DSM_FUNCTION_SUPPORTED_FUNCTIONS_INDEX,           SUPPORTED_FUNCTIONS)
        MAKECASE1(IMX_ENET_DSM_FUNCTION_GET_MAC_ADDRESS_INDEX,               MAC_ADDRESS)
        MAKEDEFAULT("ACPI function")
    }
}

/*++
Routine Description:
    Converts input value to a string.
Argument:
    i  input value
Return Value:
    Pointer to the string
--*/
_Use_decl_annotations_
const char* Dbg_GetEnetPhyInterfaceTypeName(_In_ MP_MII_PHY_INTERFACE_TYPE i) {
    switch (i) {
        MAKECASE(RGMII)
        MAKECASE(RMII)
        MAKECASE(MII)
        MAKEDEFAULT("Enet Phy interface type")
    }
}

#endif // DBG