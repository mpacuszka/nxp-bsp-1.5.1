/*
 * Copyright 2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <ntddk.h> /* for KeGetCurrentProcessorNumber in a Trace.h */
#include <wdm.h>
#include "trace.h"
#if (MALONE_TRACE == MALONE_TRACE_WPP)
    #include "vpu_core.tmh"
#endif

#include <vpu_hw/vpu_hw_defs.h>
#include <vpu_rpc/mediasys_types.h>

/**
 * init_vpu_core()
 * @brief Initializes VPU core.
 * @param FWBasePtr Pointer to virtual memory where FW is stored
 * @param FWSize Static size of FW.
 * @param FWBasePhy Physical Address of FW
 * @param csr_cpuwait Pointer to csr_cpuwait register
 * @param csr_offset Pointer to csr_offset register (boot address of M0)
 * @param reset - BOOL dereference if reset of VPU Core is necessary.
 */
NTSTATUS init_vpu_core(uintptr_t FWBasePtr, ULONG FWSize, UINT32 FWBasePhy, uintptr_t csr_cpuwait, uintptr_t csr_offset,
                       int *reset)
{
    UNICODE_STRING uniName = { 0 };
    OBJECT_ATTRIBUTES objAttr = { 0 };
    HANDLE handle = { 0 };
    NTSTATUS ntstatus = STATUS_SUCCESS;
    IO_STATUS_BLOCK ioStatusBlock = { 0 };
    LARGE_INTEGER byteOffset = { 0 };
    UINT32 cpu_waiting = readl(csr_cpuwait);

    DBG_DEV_METHOD_BEG();

    RtlInitUnicodeString(&uniName, L"\\SystemRoot\\System32\\vpu_fw_imx8_dec.bin");  /* or L"\\SystemRoot\\example.txt" */
    InitializeObjectAttributes(&objAttr, &uniName,
                               OBJ_CASE_INSENSITIVE | OBJ_KERNEL_HANDLE,
                               NULL, NULL);

    ntstatus = ZwCreateFile(&handle,
                            GENERIC_READ,
                            &objAttr, &ioStatusBlock,
                            NULL,
                            FILE_ATTRIBUTE_NORMAL,
                            0,
                            FILE_OPEN,
                            FILE_SYNCHRONOUS_IO_NONALERT,
                            NULL, 0);

    if (!NT_SUCCESS(ntstatus)) {
        DBG_PRINT_ERROR_WITH_STATUS(ntstatus,
                                    "M0 core firmware file was not found in C:\\WINDOWS\\System32\\vpu_fw_imx8_dec.bin");
        return ntstatus;
    }

    if (!cpu_waiting) {
        DBG_DEV_PRINT_INFO("M0 core firmware is already running ...Firmware is NOT downloaded.");
        ZwClose(handle);
        DBG_DEV_METHOD_END();
        *reset = TRUE;
        return ntstatus;
    }

    /* Stop M4 core 0 */
    writel(0x1, csr_cpuwait);


    if (NT_SUCCESS(ntstatus)) {
        byteOffset.LowPart = byteOffset.HighPart = 0;

        ntstatus = ZwReadFile(handle,
                              NULL,
                              NULL,
                              NULL,
                              &ioStatusBlock,
                              (void *)FWBasePtr,
                              FWSize,
                              &byteOffset,
                              NULL);

        ZwClose(handle);
    }

    char *ptr = (char *)FWBasePtr;
    /* Zero for QXP */
#if 0
    if (platform_id == IMX8QM) {
        ptr[16] = 1; /* PLATFORM */
    }
#endif
    ptr[16] = 0;

    /* Zero for decoder */
#if 0
    if (mode == ENCODER) {
        ptr[17] = 1; /* CORE ID */
    }
#endif
    ptr[17] = 0; /* CORE ID */

    _DataSynchronizationBarrier();

    /* Set Base address of M4 core */
    writel(FWBasePhy, csr_offset);

    /* Start M4 core 0 */
    writel(0x0, csr_cpuwait);

    DBG_DEV_METHOD_END();
    return ntstatus;
}


/**
 * init_vpu_reg_memory()
 * @brief Init VPU register area.
 * @param regs_base PHY address of VPU Registers
 * @param mode (DECODER/ENCODER)
 * @return NTSTATUS
 */
NTSTATUS init_vpu_reg_memory(uintptr_t regs_base, operation_mode_t mode)
{
    DBG_DEV_METHOD_BEG();
    UINT32 read_data = 0;

    writel(0x1, regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
    writel(0xFFFFFFFF, regs_base + 0x70190);
    writel(0xFFFFFFFF, regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_XMEM_RESET_SET);

    writel(0x0E, regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
    writel(0x07, regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_SET);


    if (mode == DECODER) {    /* This part is for decoder only */
        writel(0x1F, regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_SET);
        writel(0xFFFFFFFF, regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_SET);
    }
    writel(0x0102, regs_base + XMEM_CONTROL);

    read_data = readl(regs_base + 0x70108);
    DBG_DEV_PRINT_INFO("read_data=%x", read_data);
    DBG_DEV_METHOD_END();
    return STATUS_SUCCESS;
}

/**
 * reset_vpu_reg_memory()
 * @brief Resets VPU peripheral registers.
 * @param regs_base Pointer to VPU register base memory.
 * @param mode (DECODER/ENCODER)
 * @return NTSTATUS
 */
NTSTATUS reset_vpu_reg_memory(uintptr_t regs_base, operation_mode_t mode)
{
    DBG_DEV_METHOD_BEG();
    writel(0x7, regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_CLR);

    if (mode == DECODER) {
        writel(0xFFFFFFFF, regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_CLR);
    }

    DBG_DEV_METHOD_END();
    return STATUS_SUCCESS;
}

