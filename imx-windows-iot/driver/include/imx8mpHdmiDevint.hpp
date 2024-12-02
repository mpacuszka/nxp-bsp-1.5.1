/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2023 NXP
   Licensed under the MIT License.

Abstract:
    Setup and miniport installation.

*/

#include <ntddk.h>
#include <dispmprt.h>

#include "imx8mpHdmiDevintPar.h"

//
// IMX8 Plus HDMI Audio interface
// 1116487b-f98e-4e74-9a0e-669c6399866a
//

DEFINE_GUID(GUID_DEVINTERFACE_IMX8_PLUS_HDMI_AUDIO, 0x1116487b, 0xf98e, 0x4e74, 0x9a, 0x0e, 0x66, 0x9c, 0x63, 0x99, 0x86, 0x6a);

#define IMX8_PLUS_HDMI_AUDIO_INTERFACE_VERSION  1

typedef VOID(*PINTERFACE_REFERENCE)(PVOID Context);
typedef VOID(*PINTERFACE_DEREFERENCE)(PVOID Context);
typedef INT(*PAUDIO_SET_HW_PARAMS)(IN PVOID Context, IN struct imx8mp_hdmi_audio_params* AudioParams);
typedef INT(*PAUDIO_MUTE_STREAM)(IN PVOID Context, IN BOOLEAN Enable);
typedef INT(*PAUDIO_GET_CONTAINER_ID)(IN PVOID Context, OUT DXGK_CHILD_CONTAINER_ID* ContainerId);

struct Imx8mpHdmiAudioInterface : public INTERFACE
{
    PAUDIO_SET_HW_PARAMS AudioSetHwParams;
    PAUDIO_MUTE_STREAM AudioMuteStream;
    PAUDIO_GET_CONTAINER_ID AudioGetContainerId;
};

