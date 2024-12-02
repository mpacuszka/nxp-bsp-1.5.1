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
    #include "driver.tmh"
#endif
#include "imx8q_driver.h"
#include "device.h"
#include "public.h"
#include "vpu_dec\vdec.h"
#include "imxblit_public.h"

#ifdef ALLOC_PRAGMA
    #pragma alloc_text (INIT, DriverEntry)
    #pragma alloc_text (PAGE, OnDeviceAdd)
    #pragma alloc_text (PAGE, OnDriverContextCleanup)
#endif

NTSTATUS
DriverEntry(
    _In_ PDRIVER_OBJECT DriverObject,
    _In_ PUNICODE_STRING RegistryPath
)
/*++

   Routine Description:
    DriverEntry initializes the driver and is the first routine called by the
    system after the driver is loaded. DriverEntry specifies the other entry
    points in the function driver, such as EvtDevice and DriverUnload.

   Parameters Description:

    DriverObject - represents the instance of the function driver that is loaded
    into memory. DriverEntry must initialize members of DriverObject before it
    returns to the caller. DriverObject is allocated by the system before the
    driver is loaded, and it is released by the system after the system unloads
    the function driver from memory.

    RegistryPath - represents the driver specific path in the Registry.
    The function driver can use the path to store driver related data between
    reboots. The path does not store hardware instance specific data.

   Return Value:

    STATUS_SUCCESS if successful,
    STATUS_UNSUCCESSFUL otherwise.

   --*/
{
    WDF_DRIVER_CONFIG config = { 0 };
    NTSTATUS status = STATUS_SUCCESS;
    WDF_OBJECT_ATTRIBUTES attributes;

    /* Initialize WPP Tracing */
    WPP_INIT_TRACING(DriverObject, RegistryPath);

    /*TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DRIVER, "%!FUNC! iMX VPU Malone KM Entry"); */
    DBG_DRV_METHOD_BEG_WITH_PARAMS("Driver: 0x%016p, '%S'", DriverObject, ((PUNICODE_STRING)RegistryPath)->Buffer);
    DBG_DRV_PRINT_VERBOSE("***********************************************************************************");
#ifdef __DATE__
    DBG_DRV_PRINT_VERBOSE("*** NXP iMX VPU Malone Kernel Driver, date: %s %s                ***", __DATE__, __TIME__);
#else
    DBG_DRV_PRINT_VERBOSE("*** NXP iMX VPU Malone Kernel Driver                             ***");
#endif
    DBG_DRV_PRINT_VERBOSE("***********************************************************************************");



    /* Register a cleanup callback so that we can call WPP_CLEANUP when */
    /* the framework driver object is deleted during driver unload. */
    WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
    attributes.EvtCleanupCallback = OnDriverContextCleanup;

    WDF_DRIVER_CONFIG_INIT(&config,
                           OnDeviceAdd
                          );


    /* Specify a pool tag for allocations WDF makes on our behalf */
    /*config.DriverPoolTag = (ULONG)VPU_KM_ALLOC_TAG_WDF; */
    status = WdfDriverCreate(DriverObject,
                             RegistryPath,
                             &attributes,
                             &config,
                             WDF_NO_HANDLE
                            );

    if (!NT_SUCCESS(status)) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "WdfDriverCreate() failed.");
        WPP_CLEANUP(DriverObject);
        return status;
    }

    DBG_DRV_METHOD_END_WITH_STATUS(status);

    return status;
}



//
// Routine Description:
//
//  Called when any I/O request is sent to the driver.  Called in the process
//  context of the caller that initiated the I/O request.
//  Everything is passed on to the default queue currently.
//
// Arguments:
//
//     FxDevice - a handle to the framework device object
//     FxRequest - a handle to the framework request object
//
//  Return Value:
//
//     None.
//
_Use_decl_annotations_
VOID OnAnyIoInCallerContext(WDFDEVICE FxDevice, WDFREQUEST FxRequest)
{
    NTSTATUS status = STATUS_SUCCESS;
    WDF_REQUEST_PARAMETERS params = { 0 };
    DBG_DEV_METHOD_BEG();
    PAGED_CODE();


    /* This routine only handles Device I/O Control requests. */
    WDF_REQUEST_PARAMETERS_INIT(&params);
    WdfRequestGetParameters(FxRequest, &params);
    if (params.Type == WdfRequestTypeDeviceControl) {

        /* Determine what type of Device I/O Control request this is. */
        switch (params.Parameters.DeviceIoControl.IoControlCode) {
            default:
                break;
        }
    }

    status = WdfDeviceEnqueueRequest(FxDevice, FxRequest);
    if (!NT_SUCCESS(status)) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "WdfDeviceEnqueueRequest(...) failed. (FxDevice=%p, FxRequest=%p)", FxDevice,
                                    FxRequest);
        WdfRequestComplete(FxRequest, STATUS_INTERNAL_ERROR);
        DBG_DEV_METHOD_END();
        return;
    }
    DBG_DEV_METHOD_END();
    return;
}

_Use_decl_annotations_
VOID OnFileCreate(WDFDEVICE FxDevice, WDFREQUEST FxRequest, WDFFILEOBJECT FxFile)
{
    UNREFERENCED_PARAMETER(FxDevice);
    UNREFERENCED_PARAMETER(FxFile);
    DBG_DEV_METHOD_BEG();
    WdfRequestComplete(FxRequest, STATUS_SUCCESS);
    DBG_DEV_METHOD_END();
}


_Use_decl_annotations_
VOID OnFileClose(WDFFILEOBJECT FxFile)
{
    WDFDEVICE device = { 0 };
    MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr = NULL;
    int stream_id = -1;
    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    IMXVPU_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    device = WdfFileObjectGetDevice(FxFile);
    deviceContextPtr = DeviceGetContext(device);
    stream_id = GetStreamIDFromFile(deviceContextPtr, FxFile);
    /* at this moment, it just release what vdec_open created. */
    /* in the future, don´t forget to inform VPU core about instence end! */
    if (stream_id != -1) {
        vdec_stop(deviceContextPtr, stream_id, VPU_BLOCK);
        vdec_close(deviceContextPtr, stream_id); /*<< it makes devmutex locks, don´t worry */
        deviceContextPtr->ctx[stream_id].file = NULL;
    }
    /* Release any VPU locks and release any memory */
    /*WdfSpinLockAcquire(deviceContextPtr->mutex); */
    /*WdfSpinLockRelease(deviceContextPtr->mutex); */
    /* RELEASE ALL MEMORY ALLOCATED in DRIVER here */
    DBG_DEV_METHOD_END();
    return;


}

static EVT_WDF_INTERRUPT_ISR InterruptHandler; // Declaration for compile time type check.

static BOOLEAN InterruptHandler(_In_ WDFINTERRUPT WdfInterrupt, _In_  ULONG MessageID)
/*++
   Routine Description:
    Default interrupt handler.
   Arguments:
    WdfInterrupt - handle to a WDF Interrupt object.
    MessageID - default routine is not using MSI, zero.
   Return Value:
    TRUE if interrupt has been serviced.
--*/
{
    BOOLEAN handled = 0;

    handled = MU_InterruptHandler(WdfInterrupt, MessageID);
    handled |= BlitEvtInterruptIsr(WdfInterrupt, MessageID);

    return handled;
}

NTSTATUS RegisterMuInterruptHandler(MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr,
                                    PCM_PARTIAL_RESOURCE_DESCRIPTOR InterruptRaw, PCM_PARTIAL_RESOURCE_DESCRIPTOR InterruptTranslated)
/*++
   Routine Description:
    Create an interrupt object and initialize MU objects.
   Arguments:
    deviceContextPtr - pointer at device context structure.
    InterruptRaw - raw interrupt resource. Should be NULL when called from EvtonDeviceAdd to register a default handler.
    InterruptTranslated - translated interrupt resource. Should be NULL when called from EvtonDeviceAdd to register a default handler.
   Return Value:
    NTSTATUS
--*/
{
    DBG_DEV_PRINT_INFO("Initializing an interrupt object with an associated context.");

    NTSTATUS status;
    WDFINTERRUPT wdfInterrupt;
    WDF_INTERRUPT_CONFIG interruptConfig;
    WDF_OBJECT_ATTRIBUTES interruptAttributes;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&interruptAttributes, MALONE_VPU_ISR_CONTEXT);

    WDF_INTERRUPT_CONFIG_INIT(&interruptConfig, InterruptHandler, MU_DeferredProcedureCall);

    /* If the driver calls WdfInterruptCreate from EvtDriverDeviceAdd, the InterruptRaw and InterruptTranslated members
     * of the WDF_INTERRUPT_CONFIG structure must be NULL. If the driver calls WdfInterruptCreate from EvtDevicePrepareHardware,
     * these members must both be valid.
    */
    interruptConfig.InterruptRaw = InterruptRaw;
    interruptConfig.InterruptTranslated = InterruptTranslated;
    status = WdfInterruptCreate(deviceContextPtr->WdfDevice, &interruptConfig, &interruptAttributes, &wdfInterrupt);
    if (!NT_SUCCESS(status)) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "Failed to create interrupt object!");
        return status;
    } else {
        deviceContextPtr->MUInterrupt = wdfInterrupt;
    }
    {
        WDF_DPC_CONFIG dpcConfig = { 0 };
        WDF_OBJECT_ATTRIBUTES dpcAttributes;

        WDF_DPC_CONFIG_INIT(&dpcConfig, BlitEvtDpc);
        WDF_OBJECT_ATTRIBUTES_INIT(&dpcAttributes);
        dpcAttributes.ParentObject = deviceContextPtr->WdfDevice;

        status = WdfDpcCreate(&dpcConfig, &dpcAttributes, &deviceContextPtr->BlitterCtx.Dpc);
        if (!NT_SUCCESS(status)) {
            DBG_PRINT_ERROR_WITH_STATUS(status, "Failed to create dpc object!");
            return status;
        }
    }

    /* Configure MU */

    DBG_DEV_PRINT_INFO("Initializing a Messaging Unit");
    {
        mu_isr_dpc_calls_struct calls = { 0 };

        calls.rx_full_isr[0] = mu_isr;
        calls.rx_full_dpc[0] = mu_dpc;

        MU_FromAddDevice(deviceContextPtr->WdfDevice, deviceContextPtr->MUInterrupt, &calls,
                         NULL, NULL);
    }

    /* Initialize mu msg circular buffer */

    DBG_DEV_PRINT_INFO("Initializing a MU msg circular buffer");
    {
        MALONE_VPU_ISR_CONTEXT *interruptContextPtr = InterruptGetContext(deviceContextPtr->MUInterrupt);
        WDFMEMORY memhandle;

        size_t buffersize, capacity, sz;
        WDF_OBJECT_ATTRIBUTES attributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = deviceContextPtr->MUInterrupt;
        capacity = QUEUE_SIZE;
        sz = sizeof(QUEUE_ITEM);
        buffersize = capacity * sz;

        status = WdfMemoryCreate(
                     &attributes,
                     NonPagedPoolNx,
                     MALONE_VPU_POOL_TAG,
                     buffersize,
                     &memhandle,
                     &interruptContextPtr->MsgQueue.buffer);          /* buffer pointer */
        if (!NT_SUCCESS(status)) {
            DBG_PRINT_ERROR_WITH_STATUS(status, "WdfMemoryCreate failed");
            DBG_DEV_METHOD_END();
            return status;
        }
        RtlZeroMemory(interruptContextPtr->MsgQueue.buffer, buffersize);
        interruptContextPtr->memhandle = memhandle;
        interruptContextPtr->MsgQueue.buffer_end = (char *)interruptContextPtr->MsgQueue.buffer + buffersize;
        interruptContextPtr->MsgQueue.capacity = capacity;
        interruptContextPtr->MsgQueue.count = 0;
        interruptContextPtr->MsgQueue.sz = sz;
        interruptContextPtr->MsgQueue.overflow_counter = 0;
        interruptContextPtr->MsgQueue.head = interruptContextPtr->MsgQueue.buffer;
        interruptContextPtr->MsgQueue.tail = interruptContextPtr->MsgQueue.buffer;
    }

    return status;
}

NTSTATUS
OnDeviceAdd(
    _In_ WDFDRIVER Driver,
    _Inout_ PWDFDEVICE_INIT DeviceInit
)
/*++
   Routine Description:
    EvtDeviceAdd is called by the framework in response to AddDevice
    call from the PnP manager. We create and initialize a device object to
    represent a new instance of the device.
   Arguments:
    Driver - Handle to a framework driver object created in DriverEntry
    DeviceInit - Pointer to a framework-allocated WDFDEVICE_INIT structure.
   Return Value:
    NTSTATUS
--*/
{
    NTSTATUS status = STATUS_SUCCESS;
    UNREFERENCED_PARAMETER(Driver);
    PAGED_CODE();
    DBG_DEV_METHOD_BEG();


    /* Setup PNP/Power callbacks. */
    DBG_DEV_PRINT_INFO("Initializing Setup PNP/Power callbacks.");
    {
        WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);


        pnpCallbacks.EvtDevicePrepareHardware =
            OnPrepareHardware;
        pnpCallbacks.EvtDeviceReleaseHardware =
            OnReleaseHardware;

        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
    } /* Setup PNP/Power callbacks */
    /* Register the callback that gets all I/O requests (in caller context) */
    /* before they are queued. */
    WdfDeviceInitSetIoInCallerContextCallback(DeviceInit, OnAnyIoInCallerContext);


    /* Configure file create/close callbacks */
    DBG_DEV_PRINT_INFO("Initializing file create/close callbacks.");
    {
        WDF_OBJECT_ATTRIBUTES fileAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT(&fileAttributes);
        /*  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&fileAttributes, FILE_CONTEXT); */
        fileAttributes.ExecutionLevel = WdfExecutionLevelPassive;

        WDF_FILEOBJECT_CONFIG fileConfig;
        WDF_FILEOBJECT_CONFIG_INIT(
            &fileConfig,
            OnFileCreate,
            OnFileClose,
            WDF_NO_EVENT_CALLBACK);     /* OnFileCleanup */

        WdfDeviceInitSetFileObjectConfig(DeviceInit, &fileConfig, &fileAttributes);
    }


    /* Create and initialize the WDF device */
    DBG_DEV_PRINT_INFO("Initializing the WDF device");
    WDFDEVICE device;
    MALONE_VPU_DEVICE_CONTEXT *deviceContext;
    {
        WdfDeviceInitSetCharacteristics(DeviceInit, FILE_AUTOGENERATED_DEVICE_NAME,
                                        TRUE);                              /* OR with existing values */

        WDF_OBJECT_ATTRIBUTES deviceAttributes;
        WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&deviceAttributes, MALONE_VPU_DEVICE_CONTEXT);

        /* Grant world+dog access  Security Descriptor Definition Language */
        status = WdfDeviceInitAssignSDDLString(DeviceInit,
                                               &SDDL_DEVOBJ_SYS_ALL_ADM_RWX_WORLD_RW_RES_R);
        /*&SDDL_DEVOBJ_SYS_ALL); */
        if (!NT_SUCCESS(status)) {
            DBG_PRINT_ERROR_WITH_STATUS(status, "WdfDeviceInitAssignSDDLString failed. (DeviceInit=%p)", DeviceInit);
            goto end;
        }

        status = WdfDeviceCreate(&DeviceInit, &deviceAttributes, &device);

        if (!NT_SUCCESS(status)) {
            DBG_PRINT_ERROR_WITH_STATUS(status, "WdfDeviceCreate failed. (DeviceInit=%p)", DeviceInit);
            goto end;
        }

        //
        // Get a pointer to the device context structure that we just associated
        // with the device object. We define this structure in the device.h
        // header file. DeviceGetContext is an inline function generated by
        // using the WDF_DECLARE_CONTEXT_TYPE_WITH_NAME macro in device.h.
        // This function will do the type checking and return the device context.
        // If you pass a wrong object handle it will return NULL and assert if
        // run under framework verifier mode.
        //
        deviceContext = DeviceGetContext(device);


        /* Initialize the context. */

        DBG_DEV_PRINT_INFO("Initializing a device context.");
        memset(deviceContext, 0, sizeof(MALONE_VPU_DEVICE_CONTEXT));
        deviceContext->WdfDevice = device;

        WDF_OBJECT_ATTRIBUTES spinlockAttributesDev;
        WDF_OBJECT_ATTRIBUTES spinlockAttributesRpc;
        spinlockAttributesDev.ParentObject = device;
        spinlockAttributesRpc.ParentObject = device;
        WDF_OBJECT_ATTRIBUTES_INIT(&spinlockAttributesDev);
        WdfSpinLockCreate(&spinlockAttributesDev, &deviceContext->mutex);
        WDF_OBJECT_ATTRIBUTES_INIT(&spinlockAttributesRpc);
        WdfSpinLockCreate(&spinlockAttributesRpc, &deviceContext->rpc_mutex);

        deviceContext->BlitterCtx.DeviceContextPtr = deviceContext;

        /* Create an interrupt object with an associated context. */
        RegisterMuInterruptHandler(deviceContext, NULL, NULL);
    }

    /* Initialize IOCTL queue */
    {
        DBG_DEV_PRINT_INFO("Initializing a queue");


        /* Create a device interface so that applications can find and talk */
        /* to us. */

        {
            status = WdfDeviceCreateDeviceInterface(
                         device,
                         &GUID_DEVINTERFACE_malonekm,
                         NULL          /* ReferenceString */
                     );

            if (NT_SUCCESS(status)) {

                /* Initialize the I/O Package and any Queues */

                status = IoInit(device);
            }
        }
    }
end:

    DBG_DEV_METHOD_END_WITH_STATUS(status);

    return status;
}

VOID
OnDriverContextCleanup(
    _In_ WDFOBJECT DriverObject
)
/*++
   Routine Description:
    Free all the resources allocated in DriverEntry.
   Arguments:
    DriverObject - handle to a WDF Driver object.
   Return Value:
    VOID.
--*/
{
    UNREFERENCED_PARAMETER(DriverObject);

    PAGED_CODE();
    DBG_DEV_METHOD_BEG();

    /* Stop WPP Tracing */
    WPP_CLEANUP(WdfDriverWdmGetDriverObject((WDFDRIVER)DriverObject));
    DBG_DEV_METHOD_END();

}
