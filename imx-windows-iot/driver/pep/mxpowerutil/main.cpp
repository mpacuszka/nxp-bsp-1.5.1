// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
// imxpeputil
//
//   Utility to dump clock and power information from the PEP.
//

#include <windows.h>
#include <winioctl.h>
#include <strsafe.h>
#include <cfgmgr32.h>

#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <memory>

#include <wrl.h>

#include <initguid.h>
#include <imxpephw.h>
#include <imxpepioctl.h>
#include "util.h"

using namespace Microsoft::WRL;
using namespace Microsoft::WRL::Wrappers;

std::wstring GetInterfacePath (const GUID& InterfaceGuid)
{
    ULONG length;
    CONFIGRET cr = CM_Get_Device_Interface_List_SizeW(
            &length,
            const_cast<GUID*>(&InterfaceGuid),
            nullptr,        // pDeviceID
            CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

    if (cr != CR_SUCCESS) {
        throw wexception::make(
            HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
            L"Failed to get size of device interface list. (cr = 0x%x)",
            cr);
    }

    if (length < 2) {
        throw wexception::make(
            HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
            L"The IMXPEP device was not found on this system. (cr = 0x%x)",
            cr);
    }

    std::unique_ptr<WCHAR[]> buf(new WCHAR[length]);
    cr = CM_Get_Device_Interface_ListW(
            const_cast<GUID*>(&InterfaceGuid),
            nullptr,        // pDeviceID
            buf.get(),
            length,
            CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

    if (cr != CR_SUCCESS) {
        throw wexception::make(
            HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
            L"Failed to get device interface list. (cr = 0x%x)",
            cr);
    }

    // Return the first string in the multistring
    return std::wstring(buf.get());
}

FileHandle OpenImxPepHandle ()
{
    auto interfacePath = GetInterfacePath(GUID_DEVINTERFACE_IMXPEP);

    FileHandle fileHandle(CreateFile(
            interfacePath.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,          // dwShareMode
            nullptr,    // lpSecurityAttributes
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr));  // hTemplateFile


    if (!fileHandle.IsValid()) {
        if (GetLastError() == ERROR_ACCESS_DENIED) {
            // Try opening read-only
            fileHandle.Attach(CreateFile(
                interfacePath.c_str(),
                GENERIC_READ,
                0,          // dwShareMode
                nullptr,    // lpSecurityAttributes
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                nullptr));  // hTemplateFile

            if (fileHandle.IsValid()) {
                return fileHandle;
            }
        }

        throw wexception::make(
            HRESULT_FROM_WIN32(GetLastError()),
            L"Failed to open a handle to the imxpep device. "
            L"(GetLastError() = 0x%x, interfacePath = %s)",
            GetLastError(),
            interfacePath.c_str());
    }

    return fileHandle;
}

int HandleDumpCommand (int argc, _In_reads_(argc) wchar_t * /*argv*/ [])
{
    // imxpeputil dump
    if (argc > 2) {
        fwprintf(stderr, L"Too many arguments to 'dump' command\n");
        return 1;
    }

    auto pepHandle = OpenImxPepHandle();

    IMXPEP_DUMP_REGISTERS_OUTPUT output;
    DWORD information;
    if (!DeviceIoControl(
            pepHandle.Get(),
            IOCTL_IMXPEP_DUMP_REGISTERS,
            nullptr,
            0,
            &output,
            sizeof(output),
            &information,
            nullptr) || (information != sizeof(output))) {

        throw wexception::make(
            HRESULT_FROM_WIN32(GetLastError()),
            L"IOCTL_IMXPEP_DUMP_REGISTERS failed. "
            L"(GetLastError() = 0x%x, information = %d)",
            GetLastError(),
            information);
    }

    struct REG_DESCRIPTOR {
        PCWSTR Name;
        ULONG Value;
    };

    const REG_DESCRIPTOR ccmRegisters[] = {
        { L"CCR",       output.Ccm.CCR },
        { L"CCDR",      output.Ccm.CCDR },
        { L"CSR",       output.Ccm.CSR },
        { L"CCSR",      output.Ccm.CCSR },
        { L"CACRR",     output.Ccm.CACRR },
        { L"CBCDR",     output.Ccm.CBCDR },
        { L"CBCMR",     output.Ccm.CBCMR },
        { L"CSCMR1",    output.Ccm.CSCMR1 },
        { L"CSCMR2",    output.Ccm.CSCMR2 },
        { L"CSCDR1",    output.Ccm.CSCDR1 },
        { L"CS1CDR",    output.Ccm.CS1CDR },
        { L"CS2CDR",    output.Ccm.CS2CDR },
        { L"CDCDR",     output.Ccm.CDCDR },
        { L"CHSCCDR",   output.Ccm.CHSCCDR },
        { L"CSCDR2",    output.Ccm.CSCDR2 },
        { L"CSCDR3",    output.Ccm.CSCDR3 },
        { L"CDHIPR",    output.Ccm.CDHIPR },
        { L"CLPCR",     output.Ccm.CLPCR },
        { L"CISR",      output.Ccm.CISR },
        { L"CIMR",      output.Ccm.CIMR },
        { L"CCOSR",     output.Ccm.CCOSR },
        { L"CGPR",      output.Ccm.CGPR },
        { L"CCGR[0]",   output.Ccm.CCGR[0] },
        { L"CCGR[1]",   output.Ccm.CCGR[1] },
        { L"CCGR[2]",   output.Ccm.CCGR[2] },
        { L"CCGR[3]",   output.Ccm.CCGR[3] },
        { L"CCGR[4]",   output.Ccm.CCGR[4] },
        { L"CCGR[5]",   output.Ccm.CCGR[5] },
        { L"CCGR[6]",   output.Ccm.CCGR[6] },
        { L"CMEOR",     output.Ccm.CMEOR }
    };
    
    const REG_DESCRIPTOR analogRegisters[] = {
        { L"PLL_ARM", output.Analog.PLL_ARM },
        { L"PLL_USB1", output.Analog.PLL_USB1 },
        { L"PLL_USB2", output.Analog.PLL_USB2 },
        { L"PLL_SYS", output.Analog.PLL_SYS },
        { L"PLL_SYS_SS", output.Analog.PLL_SYS_SS },
        { L"PLL_SYS_NUM", output.Analog.PLL_SYS_NUM },
        { L"PLL_SYS_DENOM", output.Analog.PLL_SYS_DENOM },
        { L"PLL_AUDIO", output.Analog.PLL_AUDIO },
        { L"PLL_AUDIO_NUM", output.Analog.PLL_AUDIO_NUM },
        { L"PLL_AUDIO_DENOM", output.Analog.PLL_AUDIO_DENOM },
        { L"PLL_VIDEO", output.Analog.PLL_VIDEO },
        { L"PLL_VIDEO_NUM", output.Analog.PLL_VIDEO_NUM },
        { L"PLL_VIDEO_DENOM", output.Analog.PLL_VIDEO_DENOM },
        { L"PLL_MLB", output.Analog.PLL_MLB },
        { L"PLL_ENET", output.Analog.PLL_ENET },
        { L"PFD_480", output.Analog.PFD_480 },
        { L"PFD_528", output.Analog.PFD_528 },
        { L"PMU_REG_1P1", output.Analog.PMU_REG_1P1 },
        { L"PMU_REG_3P0", output.Analog.PMU_REG_3P0 },
        { L"PMU_REG_2P5", output.Analog.PMU_REG_2P5 },
        { L"PMU_REG_CORE", output.Analog.PMU_REG_CORE },
        { L"MISC0", output.Analog.MISC0 },
        { L"MISC1", output.Analog.MISC1 },
        { L"MISC2", output.Analog.MISC2 },
    };
    
    const REG_DESCRIPTOR gpcRegisters[] = {
        { L"CNTR", output.Gpc.CNTR },
        { L"PGR", output.Gpc.PGR },
        { L"IMR1", output.Gpc.IMR[0] },
        { L"IMR2", output.Gpc.IMR[1] },
        { L"IMR3", output.Gpc.IMR[2] },
        { L"IMR4", output.Gpc.IMR[3] },
        { L"ISR1", output.Gpc.ISR[0] },
        { L"ISR2", output.Gpc.ISR[1] },
        { L"ISR3", output.Gpc.ISR[2] },
        { L"ISR4", output.Gpc.ISR[3] },
        { L"PGC_GPU.CTRL", output.Gpc.PGC_GPU.CTRL },
        { L"PGC_GPU.PUPSCR", output.Gpc.PGC_GPU.PUPSCR },
        { L"PGC_GPU.PDNSCR", output.Gpc.PGC_GPU.PDNSCR },
        { L"PGC_GPU.SR", output.Gpc.PGC_GPU.SR },
        { L"PGC_CPU.CTRL", output.Gpc.PGC_CPU.CTRL },
        { L"PGC_CPU.PUPSCR", output.Gpc.PGC_CPU.PUPSCR },
        { L"PGC_CPU.PDNSCR", output.Gpc.PGC_CPU.PDNSCR },
        { L"PGC_CPU.SR", output.Gpc.PGC_CPU.SR },
    };

    wprintf(L"CCM Register Dump\n");
    for (const auto& entry : ccmRegisters) {
        wprintf(L"    %20s = 0x%08x\n", entry.Name, entry.Value);
    }

    wprintf(L"\nANALOG Register Dump\n");
    for (const auto& entry : analogRegisters) {
        wprintf(L"    %20s = 0x%08x\n", entry.Name, entry.Value);
    }

    wprintf(L"\nGPC Register Dump\n");
    for (const auto& entry : gpcRegisters) {
        wprintf(L"    %20s = 0x%08x\n", entry.Name, entry.Value);
    }

/*    const IMX_CCM_CLPCR_REG clpcr = {output.Ccm.CLPCR};

    wprintf(
        L"\nCLPCR = 0x%x\n"
        L"    CLPCR[LPM] = %d\n"
        L"    CLPCR[ARM_clk_dis_on_lpm] = %d\n"
        L"    CLPCR[SBYOS] = %d\n"
        L"    CLPCR[dis_ref_osc] = %d\n"
        L"    CLPCR[VSTBY] = %d\n"
        L"    CLPCR[stby_count] = %d\n"
        L"    CLPCR[cosc_pwrdown] = %d\n"
        L"    CLPCR[wb_per_at_lpm] = %d\n"
        L"    CLPCR[bypass_mmdc_ch0_lpm_hs] = %d\n"
        L"    CLPCR[bypass_mmdc_ch1_lpm_hs] = %d\n"
        L"    CLPCR[mask_core0_wfi] = %d\n"
        L"    CLPCR[mask_core1_wfi] = %d\n"
        L"    CLPCR[mask_core2_wfi] = %d\n"
        L"    CLPCR[mask_core3_wfi] = %d\n"
        L"    CLPCR[mask_scu_idle] = %d\n"
        L"    CLPCR[mask_l2cc_idle] = %d\n",
        clpcr.AsUlong,
        clpcr.LPM,
        clpcr.ARM_clk_dis_on_lpm,
        clpcr.SBYOS,
        clpcr.dis_ref_osc,
        clpcr.VSTBY,
        clpcr.stby_count,
        clpcr.cosc_pwrdown,
        clpcr.wb_per_at_lpm,
        clpcr.bypass_mmdc_ch0_lpm_hs,
        clpcr.bypass_mmdc_ch1_lpm_hs,
        clpcr.mask_core0_wfi,
        clpcr.mask_core1_wfi,
        clpcr.mask_core2_wfi,
        clpcr.mask_core3_wfi,
        clpcr.mask_scu_idle,
        clpcr.mask_l2cc_idle);
*/
    return 0;
}

static const PCWSTR platformStateNames[_PEP_STATE_CNT] = {
    L"PLATFORM_IDLE_STATE_WAIT",
    L"PLATFORM_IDLE_STATE_STOP_LIGHT",
    L"PLATFORM_IDLE_STATE_ARM_OFF",
    L"PEP_PLATFORM_IDLE_STATE_NONE",
    L"PLATFORM_IDLE_STATE_UNKNOWN"
};


int HandleStatCommand(int argc, _In_reads_(argc) wchar_t* /*argv*/[])
{
    // imxpeputil stat
    if (argc > 2) {
        fwprintf(stderr, L"Too many arguments to 'stat' command\n");
        return 1;
    }

    auto pepHandle = OpenImxPepHandle();

    IMXPEP_STAT_OUTPUT output;
    DWORD information;
    if (!DeviceIoControl(
        pepHandle.Get(),
        IOCTL_IMXPEP_STAT,
        nullptr,
        0,
        &output,
        sizeof(output),
        &information,
        nullptr) || (information != sizeof(output))) {

        throw wexception::make(
            HRESULT_FROM_WIN32(GetLastError()),
            L"IOCTL_IMXPEP_STAT failed. "
            L"(GetLastError() = 0x%x, information = %d)",
            GetLastError(),
            information);
    }

    wprintf(L"IMX PEP Statistical Counters:\n");
    for (ULONG platformStateIdx = 0; platformStateIdx < _PEP_STATE_CNT; platformStateIdx++) {
        wprintf(L"\nPlatform State %s (%u)\n", platformStateNames[platformStateIdx], platformStateIdx);
        for (ULONG cpuIdx = 0; cpuIdx < _PEP_CPU_CNT; cpuIdx++) {
            auto& cnt = output.PepCounters[platformStateIdx][cpuIdx];
            if (cnt.WfiCount + cnt.Wfi2Count + cnt.GatedCount + cnt.UndefinedCpuStateCount == 0) {
                continue;
            }

            wprintf(L"\n  CPU %u\n", cpuIdx);
            if (cnt.WfiCount) {
                wprintf(L"    %20s = %12u\n", L"WfiCount", cnt.WfiCount);
                wprintf(L"    %20s = %12u\n", L"WfiTime [ms]", cnt.WfiTime);
            }
            if (cnt.Wfi2Count) {
                wprintf(L"    %20s = %12u\n", L"Wfi2Count", cnt.Wfi2Count);
                wprintf(L"    %20s = %12u\n", L"Wfi2Time [ms]", cnt.Wfi2Time);
            }
            if (cnt.GatedCount) {
                wprintf(L"    %20s = %12u\n", L"GatedCount", cnt.GatedCount);
                wprintf(L"    %20s = %12u\n", L"GatedTime [ms]", cnt.GatedTime);
            }
            if (cnt.UndefinedCpuStateCount) {
                wprintf(L"    %20s = %12u\n", L"UndefCount", cnt.UndefinedCpuStateCount);
                wprintf(L"    %20s = %12u\n", L"UndefTime [ms]", cnt.UndefinedCpuStateTime);
            }
        }
    }

    return 0;
}


void PrintUsage ()
{
    PCWSTR Usage =
L"imxpeputil: IMX Clock and Power Utility\n"
L"Usage: imxpeputil [dump|stat]\n"
L"\n"
L" dump                        Dump miscellaneous information about the CCM/GPC\n"
L" stat                        Statistical counters from IMX PEP\n"
L"\n"
L"Examples:\n"
L"  Dump register values:\n"
L"    imxpeputil dump\n"
L"  Display statistical counters:\n"
L"    imxpeputil stat\n"
L"\n";

    wprintf(Usage);
}

int mainexcpt (_In_ int argc, _In_reads_(argc) wchar_t* argv[])
{
    if (argc < 2) {
        fwprintf(
            stderr,
            L"Missing required parameter 'command'. Run 'imxpeputil /?' for usage.\n");

        return 1;
    }

    PCWSTR command = argv[1];
    if (!_wcsicmp(command, L"-h") || !_wcsicmp(command, L"/h") ||
        !_wcsicmp(command, L"-?") || !_wcsicmp(command, L"/?")) {

        PrintUsage();
        return 0;
    }

    if (!_wcsicmp(command, L"dump")) {
        return HandleDumpCommand(argc, argv);
    } 
    else if (!_wcsicmp(command, L"stat")) {
        return HandleStatCommand(argc, argv);
    } else {
        fwprintf(
            stderr,
            L"Unrecognized command: %s. Type '%s /?' for usage.\n",
            command,
            argv[0]);

        return 1;
    }
}

int __cdecl wmain (_In_ int argc, _In_reads_(argc) wchar_t* argv[])
{
    try {
        return mainexcpt(argc, argv);
    } catch (const wexception& ex) {
        fwprintf(stderr, L"Error: %s\n", ex.wwhat());
        return 1;
    }
}
