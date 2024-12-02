/* Copyright (c) Microsoft Corporation.
 * Copyright 2023 NXP
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmdImx8mpDisplayController.h"
#include "GcKmdUtil.h"

#include "getresrc.h"

GcKmImx8mpDisplayController::~GcKmImx8mpDisplayController()
{
    if (p_LvdsDisplay != nullptr) {
        delete p_LvdsDisplay;
    }
    if (p_HdmiDisplay != nullptr) {
        delete p_HdmiDisplay;
    }
    if (p_MipiDisplay != nullptr) {
        delete p_MipiDisplay;
    }
}

GcKmImx8mpDisplayController::GcKmImx8mpDisplayController(
    DXGKRNL_INTERFACE*  pDxgkInterface)
    : GcKmBaseDisplayController(pDxgkInterface)
{
    DisplayInterface di[DISP_INTERFACE_PARALLEL_LCD + 1] = { 0 };
    WCHAR   szDisplayInterface[] = L"Display0Interface";
    NTSTATUS    Status;
    ULONG   Interface;
    BOOL RegisterLVDS = FALSE, SetShared = FALSE;

    m_IsInitialized = TRUE;
    p_LvdsDisplay = nullptr;
    p_HdmiDisplay = nullptr;
    p_MipiDisplay = nullptr;

    /* Iterate over possible maximum multi-displays config on MP, indexes bigger than that will be ingnored */
    for (UINT i = 0; i < MP_MAX_MULTIPLE_DISPLAYS; i++) {
        Status = GetDwordRegistryParam(pDxgkInterface, szDisplayInterface, &Interface);
        if (NT_SUCCESS(Status)) {
            switch (Interface) {
            case DISP_INTERFACE_LVDS0:
            case DISP_INTERFACE_LVDS_DUAL0:
                if (!di[Interface].UseDisplay) {
                    di[Interface].UseDisplay = TRUE;
                    di[Interface].RegistryIndex = i;
                    di[Interface].DisplayInterfaceIndex = 0;
                }
                break;
            case DISP_INTERFACE_LVDS1:
                if (!di[Interface].UseDisplay) {
                    di[Interface].UseDisplay = TRUE;
                    di[Interface].RegistryIndex = i;
                    di[Interface].DisplayInterfaceIndex = 1;
                }
                break;
            case DISP_INTERFACE_HDMI:
                if (!di[Interface].UseDisplay) {
                    di[Interface].UseDisplay = TRUE;
                    di[Interface].RegistryIndex = i;
                    di[Interface].DisplayInterfaceIndex = 0;
                }
                break;
            case DISP_INTERFACE_MIPI_DSI0:
                if (!di[Interface].UseDisplay) {
                    di[Interface].UseDisplay = TRUE;
                    di[Interface].RegistryIndex = i;
                    di[Interface].DisplayInterfaceIndex = 0;
                }
                break;
            default:
                /* disabled or not supported interface - nothing to do */
                break;
            }
        }
        szDisplayInterface[7]++;
    }

    BOOLEAN bMonitorsEnabled = FALSE;

    for (UINT i = 1; i <= DISP_INTERFACE_PARALLEL_LCD; i++)
    {
        if (di[i].UseDisplay)
        {
            bMonitorsEnabled = TRUE;
        }
    }

    if (FALSE == bMonitorsEnabled)
    {
        /* If no supported monitor, force HDMI enabled */
        di[DISP_INTERFACE_HDMI].UseDisplay = TRUE;
        di[DISP_INTERFACE_HDMI].RegistryIndex = 0;
        di[DISP_INTERFACE_HDMI].DisplayInterfaceIndex = 0;
    }

    /* Determine if LVDS and MIPI-DSI used at the same time - they have shared clock source VIDEO_PLL1 */
    if ((di[DISP_INTERFACE_LVDS0].UseDisplay || di[DISP_INTERFACE_LVDS1].UseDisplay || di[DISP_INTERFACE_LVDS_DUAL0].UseDisplay)
        && di[DISP_INTERFACE_MIPI_DSI0].UseDisplay) {
        SetShared = TRUE;
    }

    if (di[DISP_INTERFACE_LVDS0].UseDisplay) {
        if (SetShared) {
            di[DISP_INTERFACE_LVDS0].Shared = TRUE;
        }
        p_LvdsDisplay = new (NonPagedPoolNx, 'PSID') GcKmImx8mpDisplay(&di[DISP_INTERFACE_LVDS0]);
        RegisterLVDS = TRUE;
    }
    if (di[DISP_INTERFACE_LVDS_DUAL0].UseDisplay) {
        if (SetShared) {
            di[DISP_INTERFACE_LVDS_DUAL0].Shared = TRUE;
        }
        p_LvdsDisplay = new (NonPagedPoolNx, 'PSID') GcKmImx8mpDisplay(&di[DISP_INTERFACE_LVDS_DUAL0]);
        RegisterLVDS = TRUE;
    }
    if (di[DISP_INTERFACE_LVDS1].UseDisplay && m_IsInitialized) {
        if (SetShared) {
            di[DISP_INTERFACE_LVDS1].Shared = TRUE;
        }
        p_LvdsDisplay = new (NonPagedPoolNx, 'PSID') GcKmImx8mpDisplay(&di[DISP_INTERFACE_LVDS1]);
        RegisterLVDS = TRUE;
    }
    if (RegisterLVDS && p_LvdsDisplay == nullptr) {
        m_IsInitialized = FALSE;
    }
    else if (RegisterLVDS) {
        RegisterDisplayPipeline(p_LvdsDisplay);
    }

    if (di[DISP_INTERFACE_HDMI].UseDisplay && m_IsInitialized) {
        p_HdmiDisplay = new (NonPagedPoolNx, 'PSID') GcKmImx8mpHdmiDisplay();
        if (p_HdmiDisplay == nullptr) {
            m_IsInitialized = FALSE;
        }
        else {
            RegisterDisplayPipeline(p_HdmiDisplay, true);
        }
    }

    if (di[DISP_INTERFACE_MIPI_DSI0].UseDisplay && m_IsInitialized) {
        if (SetShared) {
            di[DISP_INTERFACE_MIPI_DSI0].Shared = TRUE;
        }
        p_MipiDisplay = new (NonPagedPoolNx, 'PSID') GcKmImx8mpMipiDsiDisplay(&di[DISP_INTERFACE_MIPI_DSI0]);
        if (p_MipiDisplay == nullptr) {
            m_IsInitialized = FALSE;
        }
        else {
            RegisterDisplayPipeline(p_MipiDisplay);
        }
    }

}

