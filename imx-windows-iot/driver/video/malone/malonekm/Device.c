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

#include "Trace.h"
#if (MALONE_TRACE == MALONE_TRACE_WPP)
    #include "device.tmh"
#endif

#include "imx8q_driver.h"
#include "Public.h"
#include "Device.h"
#include "imxblit_public.h"



#ifdef ALLOC_PRAGMA
    #pragma alloc_text (PAGE, OnPrepareHardware)
#endif

/* Configuration */
typedef struct _VPU_REG_VALUE_DESC {
    PCWSTR ValueName;
    ULONG *DestinationPtr;
    ULONG DefaultValue;
} VPU_REG_VALUE_DESC, *PVPU_REG_VALUE_DESC;

/* Function prototypes from other files (vpu_memory.c) */
NTSTATUS init_vpu_sys_memory(MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr, WDFCMRESLIST ResourcesRaw,
                             WDFCMRESLIST ResourcesTranslated);
void deinit_vpu_sys_memory(MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr);
int init_vpu_pwr_clk(MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr, operation_mode_t mode);
int deinit_vpu_pwr_clk(MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr, operation_mode_t mode);
NTSTATUS vpu_alloc_ctx_memory(MALONE_VPU_DEVICE_CONTEXT *dev);
void vpu_free_ctx_memory(MALONE_VPU_DEVICE_CONTEXT *dev);


void vpu_sw_reset(MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr)
{
    WdfSpinLockAcquire(deviceContextPtr->rpc_mutex);
    rpc_send_cmd_buf(&deviceContextPtr->shared_mem, 0, VID_API_CMD_FIRM_RESET, 0, NULL);
    _DataSynchronizationBarrier();
    WdfSpinLockRelease(deviceContextPtr->rpc_mutex);
    while (STATUS_DEVICE_BUSY == MU_SendMsg((MU_Type *)deviceContextPtr->RegistersMUPtr, MSG_TYPE, COMMAND));
}

/* Functions */
/*++
   Routine Description:
    Read the following from the registry
        1. All the parameters
        2. NetworkAddres
   Arguments:
    Adapter     Pointer to our adapter
   Return Value:
    STATUS_SUCCESS
    STATUS_FAILURE
   --*/
_Use_decl_annotations_
NTSTATUS vpu_read_reg_parameters(MALONE_VPU_DEVICE_CONTEXT *dev)
{
    NTSTATUS status = STATUS_SUCCESS;
    WDFKEY hKey = NULL;

    status = WdfDeviceOpenRegistryKey(dev->WdfDevice, PLUGPLAY_REGKEY_DEVICE, STANDARD_RIGHTS_ALL, WDF_NO_OBJECT_ATTRIBUTES,
                                      &hKey);
    if (!NT_SUCCESS(status)) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "WdfDeviceOpenRegistryKey(...) failed.");
        return status;
    }

    struct _VPU_REG_VALUE_DESC regTable[] = {
        {
            L"MaxContexts",
            &dev->config.MaxContexts,
            (ULONG)2,
        },
        {
            L"FrameBuffers",
            &dev->config.FrameBuffers,
            22,
        },
        {
            L"OutFrameBuffers",
            &dev->config.OutFrameBuffers,
            3,
        },
        {
            L"MaxWidth",
            &dev->config.MaxWidth,
            1920,
        },
        {
            L"MaxHeigth",
            &dev->config.MaxHeigth,
            1080,
        },
        {
            L"StreamBuffSize",
            &dev->config.StreamBuffSize,
            2097152, /*10485760, */
        },
        {
            L"EnableHEVC",
            &dev->config.EnableHEVC,
            0,
        },
        {
            L"Allow10BitFormat",
            &dev->config.Allow10BitFormat,
            0,
        },
    };

    for (ULONG i = 0; i < ARRAYSIZE(regTable); ++i) {
        PVPU_REG_VALUE_DESC descriptorPtr = &regTable[i];

        UNICODE_STRING valueName;
        status = RtlUnicodeStringInit(
                     &valueName,
                     descriptorPtr->ValueName);

        NT_ASSERT(NT_SUCCESS(status));

        status = WdfRegistryQueryULong(hKey, &valueName, descriptorPtr->DestinationPtr);

        if (!NT_SUCCESS(status)) {
            DBG_PRINT_ERROR_WITH_STATUS(status,
                                        "Failed to query registry value, using default value, descriptorPtr->ValueName = %S, descriptorPtr->DefaultValue = %lu)",
                                        descriptorPtr->ValueName,
                                        descriptorPtr->DefaultValue);

            *descriptorPtr->DestinationPtr = descriptorPtr->DefaultValue;
        }
    }
    /* Close registry handle */
    DBG_DRV_METHOD_END_WITH_STATUS(status);
    return STATUS_SUCCESS;
}


_Use_decl_annotations_
NTSTATUS
OnPrepareHardware(
    WDFDEVICE WdfDevice,
    WDFCMRESLIST ResourcesRaw,
    WDFCMRESLIST ResourcesTranslated
)
{
    PAGED_CODE();
    IMXVPU_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    UNREFERENCED_PARAMETER(ResourcesRaw);

    NTSTATUS ntstatus = STATUS_SUCCESS;
    LARGE_INTEGER Interval = { 0 };
    DBG_DRV_METHOD_BEG();
    BOOL encoder_running = 0;
    //
    // ReleaseHardware is ALWAYS called, even if PrepareHardware fails, so
    // the cleanup of registersPtr is handled there
    //
    MALONE_VPU_DEVICE_CONTEXT *dev = DeviceGetContext(WdfDevice);

    Interval.QuadPart = -1 * (10000 * /*msec*/ 1000);
    dev->ctx = NULL;

    /* Init system memory */
    ntstatus = init_vpu_sys_memory(dev, ResourcesRaw, ResourcesTranslated);
    if (!NT_SUCCESS(ntstatus)) {
        DBG_DRV_METHOD_END_WITH_STATUS(ntstatus);
        return ntstatus;
    }

    /* Read the registry parameters */
    ntstatus = vpu_read_reg_parameters(dev);
    if (!NT_SUCCESS(ntstatus)) {
        DBG_DRV_METHOD_END_WITH_STATUS(ntstatus);
        return ntstatus;
    }

    /* Reserve memory for VPU HW */
    ntstatus = vpu_alloc_ctx_memory(dev);
    if (!NT_SUCCESS(ntstatus)) {
        DBG_DRV_METHOD_END_WITH_STATUS(ntstatus);
        return ntstatus;
    }

    // TODO: Consider moving power up code into D0 Entry. We are in D3 state here.

    if (encoder_running) {
        init_vpu_pwr_clk(dev, DECODER);
    } else {
        deinit_vpu_pwr_clk(dev, DECODER);
        init_vpu_pwr_clk(dev, DECODER);
    }

    /* Init VPU register memory */
    ntstatus = init_vpu_reg_memory((uintptr_t)dev->RegistersPtr, DECODER);
    if (!NT_SUCCESS(ntstatus)) {
        DBG_DRV_METHOD_END_WITH_STATUS(ntstatus);
        return ntstatus;
    }

    /* Init RPC memory */
    rpc_init_shared_memory(&dev->shared_mem,
                           dev->RPCBasePhy.QuadPart - dev->FWBasePhy.QuadPart,
                           (void *)dev->RPCBasePtr,
                           VPU_RPC_SIZE);

    rpc_set_system_cfg_value(dev->shared_mem.pSharedInterface, VPU_REG_BASE_MCORE);

    /* Init MU */
    MU_FromDevicePrepareHardware(dev->WdfDevice, (MU_Type *)(dev->RegistersMUPtr));


    /* Init VPU core (burn firmware and run CPU) */
    int reset = FALSE;
    ntstatus = init_vpu_core((intptr_t)dev->FWBasePtr, (ULONG)sizeof(MALONE_VPU_FW), (UINT32)dev->FWBasePhy.QuadPart,
                             dev->csr_cpuwait, dev->csr_offset, &reset);   /*BurnFirmware(deviceContextPtr); */
    if (!NT_SUCCESS(ntstatus)) {
        DBG_DRV_METHOD_END_WITH_STATUS(ntstatus);
        return ntstatus;
    }


    /* SW reset of MCU */
    if (reset) {
        vpu_sw_reset(dev);
    }


    /* FW configuration */
    UINT32 msg = 0;
    int attempts = 10; /*10s */
    MU_Type *base = dev->MUDeviceContext.base;
    while (!dev->vpu_fw_started) {
        while (STATUS_DEVICE_BUSY == MU_ReceiveMsg(base, 0, &msg)) {
            attempts--;
            if (attempts <= 0) {
                return STATUS_ABANDONED;
            }
            KeDelayExecutionThread(KernelMode, TRUE, &Interval);
        };

        DBG_DRV_PRINT_INFO("\t\t\tRECEIVED MU message during init: 0x%x", msg);

        if (msg == 0xAA) {
            dev->running = 1;
            /* Set shared memory */
            mu_send_msg_to_FW(base, RPC_BUF_OFFSET,
                              (UINT32)(dev->RPCBasePhy.QuadPart - dev->FWBasePhy.QuadPart)); /* CM0 use relative address */
            mu_send_msg_to_FW(base, BOOT_ADDRESS, (UINT32)dev->FWBasePhy.QuadPart);
            /* Send init_done flag */
            mu_send_msg_to_FW(base, INIT_DONE, 2);
            DBG_DRV_PRINT_INFO("\t\t\tSent INIT_DONE!");
        } else if (msg == 0x55) {
            /* Report fimware is started */
            dev->vpu_fw_started = TRUE;
            UINT32 fw_version = dev->shared_mem.pSharedInterface->FWVersion;
            UNREFERENCED_PARAMETER(fw_version);
            DBG_DRV_PRINT_INFO("\t\t\tFirmware version is %d.%d.%d", (fw_version & 0x00ff0000) >> 16,
                               (fw_version & 0x0000ff00) >> 8,
                               fw_version & 0x000000ff);
            DBG_DRV_PRINT_INFO("\t\t\tFIRMWARE STARTED!");
        } else {
            DBG_DRV_PRINT_INFO("\t\t\tUnexpected MU message during init: 0x%x", msg);
        }
    }


    /* Enable MU interrupts */
    MU_EnableReceiveInterrupts((MU_Type *)(dev->RegistersMUPtr),
                               0); // FIXME One doesn't enable interrupts while OS expects them to be disabled.

    BlitPrepareDpuHw(&dev->BlitterCtx);

    DBG_DRV_METHOD_END();
    return ntstatus;
}

NTSTATUS
OnReleaseHardware(
    WDFDEVICE WdfDevice,
    WDFCMRESLIST ResourcesTranslated
)
{
    PAGED_CODE();
    IMXVPU_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    DBG_DRV_METHOD_BEG();
    UNREFERENCED_PARAMETER(ResourcesTranslated);

    MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr =
        DeviceGetContext(WdfDevice);


    deinit_vpu_sys_memory(deviceContextPtr);

    vpu_free_ctx_memory(deviceContextPtr);

    DBG_DRV_METHOD_END();
    return STATUS_SUCCESS;
}
