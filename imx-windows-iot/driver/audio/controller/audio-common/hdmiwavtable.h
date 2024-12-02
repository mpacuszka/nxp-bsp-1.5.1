/* Copyright (c) Microsoft Corporation All Rights Reserved
   Licensed under the MIT License.

Module Name:

    hdmiwavtable.h

Abstract:

    Declaration of wave miniport tables for the hdmi endpoint.

    Note: the HDMI in this sample shows how to do h/w loopback 
          for non-offloaded devices.

--*/

#ifndef _HDMIWAVTABLE_H_
#define _HDMIWAVTABLE_H_


//=============================================================================
// Defines
//=============================================================================


#define HDMI_DEVICE_MAX_CHANNELS                 2       // Max Channels.

#define HDMI_HOST_MAX_CHANNELS                   2       // Max Channels.
#define HDMI_HOST_MIN_BITS_PER_SAMPLE            32      // Min Bits Per Sample
#define HDMI_HOST_MAX_BITS_PER_SAMPLE            32      // Max Bits Per Sample
#define HDMI_HOST_MIN_SAMPLE_RATE                48000   // Min Sample Rate
#define HDMI_HOST_MAX_SAMPLE_RATE                48000   // Max Sample Rate

//
// Max # of pin instances.
//
#define HDMI_MAX_INPUT_SYSTEM_STREAMS            1

//=============================================================================
static 
KSDATAFORMAT_WAVEFORMATEXTENSIBLE HdmiHostPinSupportedDeviceFormats[] =
{
    {
        {
            sizeof(KSDATAFORMAT_WAVEFORMATEXTENSIBLE),
            0,
            0,
            0,
            STATICGUIDOF(KSDATAFORMAT_TYPE_AUDIO),
            STATICGUIDOF(KSDATAFORMAT_SUBTYPE_PCM),
            STATICGUIDOF(KSDATAFORMAT_SPECIFIER_WAVEFORMATEX)
        },
        {
            {
                WAVE_FORMAT_EXTENSIBLE,
                2,
                HDMI_HOST_MIN_SAMPLE_RATE,
                8*HDMI_HOST_MAX_SAMPLE_RATE,
                8,
                32,  // specify a 32 bit container for 24 bit audio to make fifo management easier.
                sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX)
            },
            HDMI_HOST_MAX_BITS_PER_SAMPLE,
            KSAUDIO_SPEAKER_STEREO,
            STATICGUIDOF(KSDATAFORMAT_SUBTYPE_PCM)
        }
    },
};

//
// Supported modes (only on streaming pins).
//
static
MODE_AND_DEFAULT_FORMAT HdmiHostPinSupportedDeviceModes[] =
{
    {
        STATIC_AUDIO_SIGNALPROCESSINGMODE_RAW,
        &HdmiHostPinSupportedDeviceFormats[0].DataFormat // 44.1kHz
    }
};

//
// The entries here must follow the same order as the filter's pin
// descriptor array.
//
static 
PIN_DEVICE_FORMATS_AND_MODES HdmiPinDeviceFormatsAndModes[] = 
{
    {
        SystemRenderPin,
        HdmiHostPinSupportedDeviceFormats,
        SIZEOF_ARRAY(HdmiHostPinSupportedDeviceFormats),
        HdmiHostPinSupportedDeviceModes,
        SIZEOF_ARRAY(HdmiHostPinSupportedDeviceModes)
    },
    {
        BridgePin,
        NULL,
        0,
        NULL,
        0
    },
};

//=============================================================================
static
KSDATARANGE_AUDIO HdmiPinDataRangesStream[] =
{
    { // 0 - PCM host
        {
            sizeof(KSDATARANGE_AUDIO),
            KSDATARANGE_ATTRIBUTES,         // An attributes list follows this data range
            0,
            0,
            STATICGUIDOF(KSDATAFORMAT_TYPE_AUDIO),
            STATICGUIDOF(KSDATAFORMAT_SUBTYPE_PCM),
            STATICGUIDOF(KSDATAFORMAT_SPECIFIER_WAVEFORMATEX)
        },
        HDMI_HOST_MAX_CHANNELS,           
        HDMI_HOST_MIN_BITS_PER_SAMPLE,    
        HDMI_HOST_MAX_BITS_PER_SAMPLE,    
        HDMI_HOST_MIN_SAMPLE_RATE,            
        HDMI_HOST_MAX_SAMPLE_RATE             
    }
};

static
PKSDATARANGE HdmiPinDataRangePointersStream[] =
{
    PKSDATARANGE(&HdmiPinDataRangesStream[0]),
    PKSDATARANGE(&PinDataRangeAttributeList)
};

//=============================================================================
static
KSDATARANGE HdmiPinDataRangesBridge[] =
{
    {
        sizeof(KSDATARANGE),
        0,
        0,
        0,
        STATICGUIDOF(KSDATAFORMAT_TYPE_AUDIO),
        STATICGUIDOF(KSDATAFORMAT_SUBTYPE_ANALOG),
        STATICGUIDOF(KSDATAFORMAT_SPECIFIER_NONE)
    }
};

static
PKSDATARANGE HdmiPinDataRangePointersBridge[] =
{
    &HdmiPinDataRangesBridge[0]
};


//=============================================================================
static
PCPIN_DESCRIPTOR HdmiWaveMiniportPins[] =
{
    // Wave Out Streaming Pin (Renderer) KSPIN_WAVE_RENDER_SINK_SYSTEM
    {
        HDMI_MAX_INPUT_SYSTEM_STREAMS,
        HDMI_MAX_INPUT_SYSTEM_STREAMS, 
        0,
        NULL,
        {
            0,
            NULL,
            0,
            NULL,
            SIZEOF_ARRAY(HdmiPinDataRangePointersStream),
            HdmiPinDataRangePointersStream,
            KSPIN_DATAFLOW_IN,
            KSPIN_COMMUNICATION_SINK,
            &KSCATEGORY_AUDIO,
            NULL,
            0
        }
    },
    // Wave Out Bridge Pin (Renderer) KSPIN_WAVE_RENDER_SOURCE
    {
        0,
        0,
        0,
        NULL,
        {
            0,
            NULL,
            0,
            NULL,
            SIZEOF_ARRAY(HdmiPinDataRangePointersBridge),
            HdmiPinDataRangePointersBridge,
            KSPIN_DATAFLOW_OUT,
            KSPIN_COMMUNICATION_NONE,
            &KSCATEGORY_AUDIO,
            NULL,
            0
        }
    },
};

//=============================================================================

PCCONNECTION_DESCRIPTOR HdmiWaveMiniportConnections[] =
{
    { PCFILTER_NODE, KSPIN_WAVE_RENDER_SINK_SYSTEM, PCFILTER_NODE, KSPIN_WAVE_RENDER_SOURCE },
};

//=============================================================================
static
PCPROPERTY_ITEM PropertiesHdmiWaveFilter[] =
{
    {
        &KSPROPSETID_Pin,
        KSPROPERTY_PIN_PROPOSEDATAFORMAT,
        KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_BASICSUPPORT,
        PropertyHandler_WaveFilter
    }
};

DEFINE_PCAUTOMATION_TABLE_PROP(AutomationHdmiWaveFilter, PropertiesHdmiWaveFilter);

//=============================================================================
static
PCFILTER_DESCRIPTOR HdmiWaveMiniportFilterDescriptor =
{
    0,                                              // Version
    &AutomationHdmiWaveFilter,                      // AutomationTable
    sizeof(PCPIN_DESCRIPTOR),                       // PinSize
    SIZEOF_ARRAY(HdmiWaveMiniportPins),             // PinCount
    HdmiWaveMiniportPins,                           // Pins
    sizeof(PCNODE_DESCRIPTOR),                      // NodeSize
    0,                                              // NodeCount
    NULL,                                           // Nodes
    SIZEOF_ARRAY(HdmiWaveMiniportConnections),      // ConnectionCount
    HdmiWaveMiniportConnections,                    // Connections
    0,                                              // CategoryCount
    NULL                                            // Categories  - use defaults (audio, render, capture)
};

#endif // _HDMIWAVTABLE_H_

