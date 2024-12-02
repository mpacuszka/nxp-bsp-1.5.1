/* Copyright (c) Microsoft Corporation.
 * Copyright 2023-2024 NXP
   Licensed under the MIT License. */

#include "precomp.h"

#include "GcKmdImx8qxpDisplayController.h"
#include "GcKmdUtil.h"

#include "getresrc.h"

GcKmImx8qxpDisplayController::~GcKmImx8qxpDisplayController()
{
    for (UINT i = 0; i < QXP_MAX_LVDS_DISPLAYS; i++) {
        if (m_p_LvdsDisplay[i] != nullptr) {
            delete m_p_LvdsDisplay[i];
        }
    }
    for (UINT i = 0; i < QXP_MAX_MIPI_DSI_DISPLAYS; i++) {
        if (m_p_DsiDisplay[i] != nullptr) {
            delete m_p_DsiDisplay[i];
        }
    }

}

GcKmImx8qxpDisplayController::GcKmImx8qxpDisplayController(
    DXGKRNL_INTERFACE* pDxgkInterface)
    : GcKmBaseDisplayController(pDxgkInterface)
{
    DisplayInterface di[DISP_INTERFACE_PARALLEL_LCD + 1] = { 0 };
    WCHAR   szDisplayInterface[] = L"Display0Interface";
    NTSTATUS    Status;
    ULONG   Interface;

    m_IsInitialized = TRUE;
    for (UINT i = 0; i < QXP_MAX_LVDS_DISPLAYS; i++) {
        m_p_LvdsDisplay[i] = nullptr;
    }
    for (UINT i = 0; i < QXP_MAX_MIPI_DSI_DISPLAYS; i++) {
        m_p_DsiDisplay[i] = nullptr;
    }

    /* Iterate over possible maximum multi-displays config on QXP, indexes bigger than that will be ingnored */
    for (UINT i = 0; i <= QXP_MAX_MULTIPLE_DISPLAYS; i++) {
        Status = GetDwordRegistryParam(pDxgkInterface, szDisplayInterface, &Interface);
        if (NT_SUCCESS(Status)) {
            switch (Interface) {
            case DISP_INTERFACE_LVDS0:
            case DISP_INTERFACE_LVDS_DUAL0:
            case DISP_INTERFACE_MIPI_DSI0:
                if (!di[Interface].UseDisplay) {
                    di[Interface].UseDisplay = TRUE;
                    di[Interface].RegistryIndex = i;
                    di[Interface].DisplayInterfaceIndex = 0;
                }
                break;
            case DISP_INTERFACE_LVDS1:
            case DISP_INTERFACE_MIPI_DSI1:
                if (!di[Interface].UseDisplay) {
                    di[Interface].UseDisplay = TRUE;
                    di[Interface].RegistryIndex = i;
                    di[Interface].DisplayInterfaceIndex = 1;
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

    if (!bMonitorsEnabled) {
        /* If no supported monitor, force LVDS0 enabled */
        di[DISP_INTERFACE_LVDS0].UseDisplay = TRUE;
        di[DISP_INTERFACE_LVDS0].RegistryIndex = 0;
        di[DISP_INTERFACE_LVDS0].DisplayInterfaceIndex = 0;
    }

    //shared objects of DPU0 between display stream 0 and 1
    RtlZeroMemory(m_dpu0_prg_pdev, sizeof(m_dpu0_prg_pdev));
    RtlZeroMemory(m_dpu0_dprc_pdev, sizeof(m_dpu0_dprc_pdev));
    RtlZeroMemory(m_dpu0_client_devices, sizeof(m_dpu0_client_devices));
    m_dpu0_dpu_pdev = {};
    m_dpu0_irqsteer_pdev = {};
    m_dpu0_refCount = 0;

    m_dpu0_pdevs.p_prg_pdev = m_dpu0_prg_pdev;
    m_dpu0_pdevs.p_dprc_pdev = m_dpu0_dprc_pdev;
    m_dpu0_pdevs.p_dpu_pdev = &m_dpu0_dpu_pdev;
    m_dpu0_pdevs.p_client_devices = m_dpu0_client_devices;
    m_dpu0_pdevs.p_irqsteer_pdev = &m_dpu0_irqsteer_pdev;
    m_dpu0_pdevs.p_refCount = &m_dpu0_refCount;

    if (di[DISP_INTERFACE_LVDS0].UseDisplay || di[DISP_INTERFACE_LVDS_DUAL0].UseDisplay) {
        m_p_LvdsDisplay[0] = new (NonPagedPoolNx, 'PSID') GcKmImx8qxpDisplay(&di[DISP_INTERFACE_LVDS0], &m_dpu0_pdevs);
        if (m_p_LvdsDisplay[0] == nullptr) {
            m_IsInitialized = FALSE;
        }
        else {
            RegisterDisplayPipeline(m_p_LvdsDisplay[0]);
        }

    }
    if (di[DISP_INTERFACE_MIPI_DSI0].UseDisplay && m_IsInitialized) {
        m_p_DsiDisplay[0] = new (NonPagedPoolNx, 'PSID') GcKmImx8qxpMipiDsiDisplay(&di[DISP_INTERFACE_MIPI_DSI0], &m_dpu0_pdevs);
        if (m_p_DsiDisplay[0] == nullptr) {
            m_IsInitialized = FALSE;
        }
        else {
            RegisterDisplayPipeline(m_p_DsiDisplay[0]);
        }

    }
    if (di[DISP_INTERFACE_LVDS1].UseDisplay && m_IsInitialized) {
        m_p_LvdsDisplay[1] = new (NonPagedPoolNx, 'PSID') GcKmImx8qxpDisplay(&di[DISP_INTERFACE_LVDS1], &m_dpu0_pdevs);
        if (m_p_LvdsDisplay[1] == nullptr) {
            m_IsInitialized = FALSE;
        }
        else {
            RegisterDisplayPipeline(m_p_LvdsDisplay[1]);
        }
    }
    if (di[DISP_INTERFACE_MIPI_DSI1].UseDisplay && m_IsInitialized) {
        m_p_DsiDisplay[1] = new (NonPagedPoolNx, 'PSID') GcKmImx8qxpMipiDsiDisplay(&di[DISP_INTERFACE_MIPI_DSI1], &m_dpu0_pdevs);
        if (m_p_DsiDisplay[1] == nullptr) {
            m_IsInitialized = FALSE;
        }
        else {
            RegisterDisplayPipeline(m_p_DsiDisplay[1]);
        }
    }
}

