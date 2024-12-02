/****************************************************************************
* Copyright (c) Microsoft Corporation.
*
*    Licensed under the MIT License.
*    Licensed under the GPL License.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

//
// Include files.
//

#include <ntddk.h>          // various NT definitions
#include <string.h>

#include "galcore.h"
#include "gc_hal_kernel_driver.h"

gcsPLATFORM * gcPlatform;

DRIVER_INITIALIZE DriverEntry;

#ifdef ALLOC_PRAGMA
#pragma alloc_text(INIT, DriverEntry )
#endif // ALLOC_PRAGMA

static BOOLEAN doNotLoad = FALSE;

/* Device attached to current process */
NTSTATUS
static DeviceAttach(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    )
{
    PAGED_CODE();

    // TODO: handle failure
    gckDRIVER_DeviceAttach(DeviceObject);

    Irp->IoStatus.Status = STATUS_SUCCESS;
    Irp->IoStatus.Information = 0;

    IoCompleteRequest(Irp, IO_NO_INCREMENT );

    return STATUS_SUCCESS;
}

/* Device detached from current process */
NTSTATUS
DeviceDetach(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    )
{
    PAGED_CODE();

#ifdef TODO
    DeviceExtension * deviceExtension = (DeviceExtension *) DeviceObject->DeviceExtension;

    if (deviceExtension->m_magic != kMagic)
        __debugbreak();
#endif

    gckDRIVER_DeviceDetach(DeviceObject);

    Irp->IoStatus.Status = STATUS_SUCCESS;
    Irp->IoStatus.Information = 0;

    IoCompleteRequest(Irp, IO_NO_INCREMENT );

    return STATUS_SUCCESS;
}

NTSTATUS
DeviceIoctl(PDEVICE_OBJECT DeviceObject, PIRP Irp)
{
    PIO_STACK_LOCATION  irpSp;// Pointer to current stack location
    NTSTATUS            ntStatus = STATUS_SUCCESS;// Assume success
    ULONG               inBufLength; // Input buffer length
    ULONG               outBufLength; // Output buffer length

    PAGED_CODE();

#ifdef TODO
    DeviceExtension * deviceExtension = (DeviceExtension *) DeviceObject->DeviceExtension;

    if (deviceExtension->m_magic != kMagic)
        __debugbreak();
#endif

    irpSp = IoGetCurrentIrpStackLocation(Irp );
    inBufLength = irpSp->Parameters.DeviceIoControl.InputBufferLength;
    outBufLength = irpSp->Parameters.DeviceIoControl.OutputBufferLength;

    if (!inBufLength || !outBufLength)
    {
        ntStatus = STATUS_INVALID_PARAMETER;
        goto End;
    }

    //
    // Determine which I/O control code was specified.
    //

    switch (irpSp->Parameters.DeviceIoControl.IoControlCode )
    {
#if 0
    case IOCTL_SIOCTL_METHOD_BUFFERED:

        //
        // In this method the I/O manager allocates a buffer large enough to
        // to accommodate larger of the user input buffer and output buffer,
        // assigns the address to Irp->AssociatedIrp.SystemBuffer, and
        // copies the content of the user input buffer into this SystemBuffer
        //

        printk("Called IOCTL_SIOCTL_METHOD_BUFFERED\n");
        PrintIrpInfo(Irp);

        //
        // Input buffer and output buffer is same in this case, read the
        // content of the buffer before writing to it
        //

        inBuf = Irp->AssociatedIrp.SystemBuffer;
        outBuf = Irp->AssociatedIrp.SystemBuffer;

        //
        // Read the data from the buffer
        //

        printk("\tData from User :");
        //
        // We are using the following function to print characters instead
        // DebugPrint with %s format because we string we get may or
        // may not be null terminated.
        //
        PrintChars(inBuf, inBufLength);

        //
        // Write to the buffer over-writes the input buffer content
        //

        RtlCopyBytes(outBuf, data, outBufLength);

        printk("\tData to User : ");
        PrintChars(outBuf, datalen  );

        //
        // Assign the length of the data copied to IoStatus.Information
        // of the Irp and complete the Irp.
        //

        Irp->IoStatus.Information = (outBufLength<datalen?outBufLength:datalen);

        //
        // When the Irp is completed the content of the SystemBuffer
        // is copied to the User output buffer and the SystemBuffer is
        // is freed.
        //

       break;

    case IOCTL_SIOCTL_METHOD_NEITHER:

        //
        // In this type of transfer the I/O manager assigns the user input
        // to Type3InputBuffer and the output buffer to UserBuffer of the Irp.
        // The I/O manager doesn't copy or map the buffers to the kernel
        // buffers. Nor does it perform any validation of user buffer's address
        // range.
        //


        printk("Called IOCTL_SIOCTL_METHOD_NEITHER\n");

        PrintIrpInfo(Irp);

        //
        // A driver may access these buffers directly if it is a highest level
        // driver whose Dispatch routine runs in the context
        // of the thread that made this request. The driver should always
        // check the validity of the user buffer's address range and check whether
        // the appropriate read or write access is permitted on the buffer.
        // It must also wrap its accesses to the buffer's address range within
        // an exception handler in case another user thread deallocates the buffer
        // or attempts to change the access rights for the buffer while the driver
        // is accessing memory.
        //

        inBuf = irpSp->Parameters.DeviceIoControl.Type3InputBuffer;
        outBuf =  Irp->UserBuffer;

        //
        // Access the buffers directly if only if you are running in the
        // context of the calling process. Only top level drivers are
        // guaranteed to have the context of process that made the request.
        //

        try {
            //
            // Before accessing user buffer, you must probe for read/write
            // to make sure the buffer is indeed an userbuffer with proper access
            // rights and length. ProbeForRead/Write will raise an exception if it's otherwise.
            //
            ProbeForRead(inBuf, inBufLength, sizeof(UCHAR ) );

            //
            // Since the buffer access rights can be changed or buffer can be freed
            // anytime by another thread of the same process, you must always access
            // it within an exception handler.
            //

            printk("\tData from User :");
            PrintChars(inBuf, inBufLength);

        }
        except(EXCEPTION_EXECUTE_HANDLER)
        {

            ntStatus = GetExceptionCode();
            printk(
                "Exception while accessing inBuf 0X%08X in METHOD_NEITHER\n",
                            ntStatus);
            break;
        }


        //
        // If you are accessing these buffers in an arbitrary thread context,
        // say in your DPC or ISR, if you are using it for DMA, or passing these buffers to the
        // next level driver, you should map them in the system process address space.
        // First allocate an MDL large enough to describe the buffer
        // and initilize it. Please note that on a x86 system, the maximum size of a buffer
        // that an MDL can describe is 65508 KB.
        //

        mdl = IoAllocateMdl(inBuf, inBufLength, FALSE, TRUE, NULL);
        if (!mdl)
        {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }

        try
        {

            //
            // Probe and lock the pages of this buffer in physical memory.
            // You can specify IoReadAccess, IoWriteAccess or IoModifyAccess
            // Always perform this operation in a try except block.
            //  MmProbeAndLockPages will raise an exception if it fails.
            //
            MmProbeAndLockPages(mdl, UserMode, IoReadAccess);
        }
        except(EXCEPTION_EXECUTE_HANDLER)
        {

            ntStatus = GetExceptionCode();
            printk(
                "Exception while locking inBuf 0X%08X in METHOD_NEITHER\n",
                    ntStatus);
            IoFreeMdl(mdl);
            break;
        }

        //
        // Map the physical pages described by the MDL into system space.
        // Note: double mapping the buffer this way causes lot of
        // system overhead for large size buffers.
        //

        buffer = MmGetSystemAddressForMdlSafe(mdl, NormalPagePriority | MdlMappingNoExecute );

        if (!buffer) {
                ntStatus = STATUS_INSUFFICIENT_RESOURCES;
                MmUnlockPages(mdl);
                IoFreeMdl(mdl);
                break;
        }

        //
        // Now you can safely read the data from the buffer.
        //
        printk("\tData from User (SystemAddress) : ");
        PrintChars(buffer, inBufLength);

        //
        // Once the read is over unmap and unlock the pages.
        //

        MmUnlockPages(mdl);
        IoFreeMdl(mdl);

        //
        // The same steps can be followed to access the output buffer.
        //

        mdl = IoAllocateMdl(outBuf, outBufLength, FALSE, TRUE, NULL);
        if (!mdl)
        {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }


        try {
            //
            // Probe and lock the pages of this buffer in physical memory.
            // You can specify IoReadAccess, IoWriteAccess or IoModifyAccess.
            //

            MmProbeAndLockPages(mdl, UserMode, IoWriteAccess);
        }
        except(EXCEPTION_EXECUTE_HANDLER)
        {

            ntStatus = GetExceptionCode();
            printk(
                "Exception while locking outBuf 0X%08X in METHOD_NEITHER\n",
                    ntStatus);
            IoFreeMdl(mdl);
            break;
        }


        buffer = MmGetSystemAddressForMdlSafe(mdl, NormalPagePriority | MdlMappingNoExecute );

        if (!buffer) {
            MmUnlockPages(mdl);
            IoFreeMdl(mdl);
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }
        //
        // Write to the buffer
        //

        RtlCopyBytes(buffer, data, outBufLength);

        printk("\tData to User : %s\n", buffer);
        PrintChars(buffer, datalen);

        MmUnlockPages(mdl);

        //
        // Free the allocated MDL
        //

        IoFreeMdl(mdl);

        //
        // Assign the length of the data copied to IoStatus.Information
        // of the Irp and complete the Irp.
        //

        Irp->IoStatus.Information = (outBufLength<datalen?outBufLength:datalen);

        break;

    case IOCTL_SIOCTL_METHOD_IN_DIRECT:

        //
        // In this type of transfer, the I/O manager allocates a system buffer
        // large enough to accommodatethe User input buffer, sets the buffer address
        // in Irp->AssociatedIrp.SystemBuffer and copies the content of user input buffer
        // into the SystemBuffer. For the user output buffer, the  I/O manager
        // probes to see whether the virtual address is readable in the callers
        // access mode, locks the pages in memory and passes the pointer to
        // MDL describing the buffer in Irp->MdlAddress.
        //

        printk("Called IOCTL_SIOCTL_METHOD_IN_DIRECT\n");

        PrintIrpInfo(Irp);

        inBuf = Irp->AssociatedIrp.SystemBuffer;

        printk("\tData from User in InputBuffer: ");
        PrintChars(inBuf, inBufLength);

        //
        // To access the output buffer, just get the system address
        // for the buffer. For this method, this buffer is intended for transfering data
        // from the application to the driver.
        //

        buffer = MmGetSystemAddressForMdlSafe(Irp->MdlAddress, NormalPagePriority | MdlMappingNoExecute);

        if (!buffer) {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }

        printk("\tData from User in OutputBuffer: ");
        PrintChars(buffer, outBufLength);

        //
        // Return total bytes read from the output buffer.
        // Note OutBufLength = MmGetMdlByteCount(Irp->MdlAddress)
        //

        Irp->IoStatus.Information = MmGetMdlByteCount(Irp->MdlAddress);

        //
        // NOTE: Changes made to the  SystemBuffer are not copied
        // to the user input buffer by the I/O manager
        //

      break;

    case IOCTL_SIOCTL_METHOD_OUT_DIRECT:

        //
        // In this type of transfer, the I/O manager allocates a system buffer
        // large enough to accommodate the User input buffer, sets the buffer address
        // in Irp->AssociatedIrp.SystemBuffer and copies the content of user input buffer
        // into the SystemBuffer. For the output buffer, the I/O manager
        // probes to see whether the virtual address is writable in the callers
        // access mode, locks the pages in memory and passes the pointer to MDL
        // describing the buffer in Irp->MdlAddress.
        //


        printk("Called IOCTL_SIOCTL_METHOD_OUT_DIRECT\n");

        PrintIrpInfo(Irp);


        inBuf = Irp->AssociatedIrp.SystemBuffer;

        printk("\tData from User : ");
        PrintChars(inBuf, inBufLength);

        //
        // To access the output buffer, just get the system address
        // for the buffer. For this method, this buffer is intended for transfering data
        // from the driver to the application.
        //

        buffer = MmGetSystemAddressForMdlSafe(Irp->MdlAddress, NormalPagePriority | MdlMappingNoExecute);

        if (!buffer) {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }

        //
        // Write data to be sent to the user in this buffer
        //

        RtlCopyBytes(buffer, data, outBufLength);

        printk("\tData to User : ");
        PrintChars(buffer, datalen);

        Irp->IoStatus.Information = (outBufLength<datalen?outBufLength:datalen);

        //
        // NOTE: Changes made to the  SystemBuffer are not copied
        // to the user input buffer by the I/O manager
        //

        break;
#endif

    case IOCTL_GCHAL_INTERFACE_CODE:

        if ((inBufLength != sizeof(gcsHAL_INTERFACE)) ||
            (outBufLength != sizeof(gcsHAL_INTERFACE))) {
            ntStatus = STATUS_INVALID_PARAMETER;
            break;
        }

        //
        // In this method the I/O manager allocates a buffer large enough to
        // to accommodate larger of the user input buffer and output buffer,
        // assigns the address to Irp->AssociatedIrp.SystemBuffer, and
        // copies the content of the user input buffer into this SystemBuffer
        //

        gcsHAL_INTERFACE * ip = Irp->AssociatedIrp.SystemBuffer;
        gckDRIVER_DeviceIoctl(DeviceObject, ip);

        //
        // Assign the length of the data copied to IoStatus.Information
        // of the Irp and complete the Irp.
        //

        Irp->IoStatus.Information = outBufLength;

        //
        // When the Irp is completed the content of the SystemBuffer
        // is copied to the User output buffer and the SystemBuffer is
        // is freed.
        //

        break;

    default:

        //
        // The specified I/O control code is unrecognized by this driver.
        //

        ntStatus = STATUS_INVALID_DEVICE_REQUEST;
        printk("ERROR: unrecognized IOCTL %x\n",
            irpSp->Parameters.DeviceIoControl.IoControlCode);
        break;
    }

End:
    //
    // Finish the I/O operation by simply completing the packet and returning
    // the same status as in the packet itself.
    //

    Irp->IoStatus.Status = ntStatus;

    IoCompleteRequest(Irp, IO_NO_INCREMENT );

    return ntStatus;
}

void
ProcessResources(PCM_RESOURCE_LIST resources)
{
    ULONG fullCount = resources->Count;
    ULONG memCount = 0;
    CM_FULL_RESOURCE_DESCRIPTOR * full = resources->List;
    for(ULONG i = 0; i < fullCount ; i++)
    {
        ULONG partCount = full->PartialResourceList.Count;
        CM_PARTIAL_RESOURCE_DESCRIPTOR * part = full->PartialResourceList.PartialDescriptors;
        for(ULONG j = 0; j < partCount; j++) {
            switch (part->Type) {
            case CmResourceTypeInterrupt:
                if (gcPlatform->interrupt.m_set != TRUE) {
                    if (part->Flags & CM_RESOURCE_INTERRUPT_MESSAGE) {
                        __debugbreak();
                    }
                    else {
#if 0
                        gcPlatform->interrupt.m_affinity = part->u.Interrupt.Affinity;
#else
                        gcPlatform->interrupt.m_affinity = 0x1;
#endif
                        gcPlatform->interrupt.m_level = (KIRQL)part->u.Interrupt.Level;
                        gcPlatform->interrupt.m_vector = part->u.Interrupt.Vector;
                        gcPlatform->interrupt.m_set = TRUE;
                    }
                }
                break;
            case CmResourceTypeMemory:
                switch (memCount) {
                case 0: // First memory block must be GPU registers area
                    //     MEMORY32FIXED(ReadWrite, 0x38000000, 0x10000, )
                    gcPlatform->registerMemBase.QuadPart = part->u.Memory.Start.QuadPart;
                    gcPlatform->registerMemSize = part->u.Memory.Length;
                    /* The information about platform directly mapped memory range can be moved to ACPI, but will require a different
                     * type of resource than MEMORY32FIXED(). Use this as a workaround valid for MQ, MP, MN and QXP.
                     */
                    if (gcPlatform->registerMemBase.QuadPart > 0x40000000)
                    {
                        gcPlatform->baseAddress.QuadPart = 0x80000000;
                        gcPlatform->physSize = 0x80000000;
                    }
                    else
                    {
                        gcPlatform->baseAddress.QuadPart = 0x40000000;
                        gcPlatform->physSize = 0xC0000000;
                    }
                    break;
                case 1: // Second memory block must be Framebuffer allocated.
                    // Intentionally ignored here
                    break;
                case 2: // Third is area of directly mapped memory
                    if ((gcPlatform->registerMemBase.QuadPart > 0x40000000 && part->u.Memory.Start.QuadPart >= 0x80000000) ||
                        part->u.Memory.Start.QuadPart >= 0x40000000) {
                        // Try to filter out possible wrong values in case the ACPI is not updated and there are register addresses on this index.
                        gcPlatform->videoMemoryBase.QuadPart = part->u.Memory.Start.QuadPart;
                        gcPlatform->videoMemorySize = part->u.Memory.Length;
                    }
                    break;
                default: // Not interrested in other areas
                    break;
                }
                memCount++;
                break;
            }
            part++;
        }

        full = (CM_FULL_RESOURCE_DESCRIPTOR *) part;
    }
}

NTSTATUS
DeviceStart(PDEVICE_OBJECT DeviceObject, PIRP Irp)
{
    NTSTATUS status = STATUS_UNSUCCESSFUL;
    if (gcPlatform->interrupt.m_set)
        if(gckDRIVER_DeviceStart() == gcvSTATUS_OK)
            status = STATUS_SUCCESS;

    return status;
}

NTSTATUS
DeviceStop(PDEVICE_OBJECT DeviceObject, PIRP Irp)
{
    gckDRIVER_DeviceStop();
    return STATUS_SUCCESS;
}

const GUID interfaceGuid = { 0x3CE72FD6, 0x7677, 0x4125, 0xB8, 0x32, 0x35, 0x71, 0x87, 0x12, 0x1E, 0xD5 };

// https://docs.microsoft.com/en-us/windows-hardware/drivers/kernel/adddevice-routines-in-function-or-filter-drivers
NTSTATUS
DeviceAdd(
    _In_ struct _DRIVER_OBJECT *DriverObject,
    _In_ struct _DEVICE_OBJECT *PhysicalDeviceObject
)
{
    NTSTATUS status;

    gcPlatform->pDeviceObject = NULL;
    gcPlatform->PhysicalDeviceObject = PhysicalDeviceObject;

    ULONG propertySize;
    status = IoGetDeviceProperty(PhysicalDeviceObject,
        DevicePropertyBootConfigurationTranslated,
        0,
        NULL,
        &propertySize);

    if (status != STATUS_BUFFER_TOO_SMALL)
        goto cleanup;

    CM_RESOURCE_LIST * resources = (CM_RESOURCE_LIST *) ExAllocatePool(PagedPool, propertySize);

    if (resources == NULL)
        return gcvSTATUS_OUT_OF_MEMORY;

    status = IoGetDeviceProperty(PhysicalDeviceObject,
        DevicePropertyBootConfigurationTranslated,
        propertySize,
        resources,
        &propertySize);

    if (status == STATUS_SUCCESS)
        ProcessResources(resources);

    ExFreePool(resources);

    if (status != STATUS_SUCCESS)
        goto cleanup;

    PDEVICE_OBJECT  DeviceObject = NULL;    // ptr to device object

    status = IoCreateDevice(
        DriverObject, // Our Driver Object
        sizeof(DeviceExtension),
        NULL,
        FILE_DEVICE_UNKNOWN, // Device type
        FILE_DEVICE_SECURE_OPEN, // Device characteristics
        FALSE, // Not an exclusive device
        &DeviceObject );

    if (status != STATUS_SUCCESS)
        goto cleanup;

    gcPlatform->pDeviceObject = DeviceObject;

    PDEVICE_OBJECT parentDeviceObject;
    parentDeviceObject = IoAttachDeviceToDeviceStack(DeviceObject, PhysicalDeviceObject);

    if (parentDeviceObject == NULL) {
        printk("Unexpected NULL for parent device object\n");
        status = STATUS_UNEXPECTED_IO_ERROR;
        goto cleanup;
    }

    DeviceObject->Flags &= DO_DEVICE_INITIALIZING;
    DeviceObject->Flags |= DO_POWER_PAGABLE;
    DeviceObject->Flags |= DO_BUFFERED_IO;

    UNICODE_STRING symbolicName;

    status = IoRegisterDeviceInterface(PhysicalDeviceObject, &interfaceGuid, NULL, &symbolicName);
    if (status != STATUS_SUCCESS && status != STATUS_OBJECT_NAME_EXISTS) {
        printk("Device interface registration failed\n");
        goto cleanup;
    }

    status = IoSetDeviceInterfaceState(&symbolicName, TRUE);
    if (status != STATUS_SUCCESS) {
        RtlFreeUnicodeString(&symbolicName);
        printk("Device interface enablement failed\n");
        goto cleanup;
    }

cleanup:

    if (status != STATUS_SUCCESS) {
        if(gcPlatform->pDeviceObject != NULL) {
            IoDeleteDevice(gcPlatform->pDeviceObject );
            gcPlatform->pDeviceObject = NULL;
        }
        gcPlatform->PhysicalDeviceObject = NULL;
    }

    return status;
}

NTSTATUS
DeviceRemove(PDEVICE_OBJECT DeviceObject, PIRP Irp)
{
    gcPlatform->PhysicalDeviceObject = NULL;

    //
    // Create counted string version of our Win32 device name.
    //

    UNICODE_STRING  userPath;
    RtlInitUnicodeString(&userPath, USER_VISIBLE_DEVICE_PATH );

    //
    // Delete the link from our device name to a name in the Win32 namespace.
    //

    IoDeleteSymbolicLink(&userPath );

    if (gcPlatform->pDeviceObject != NULL )
    {
        IoDeleteDevice(gcPlatform->pDeviceObject );
        gcPlatform->pDeviceObject = NULL;
    }

    return STATUS_SUCCESS;
}


NTSTATUS
DevicePnP(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    )
{
    PAGED_CODE();

#ifdef TODO
    DeviceExtension * deviceExtension = (DeviceExtension *) DeviceObject->DeviceExtension;

    if (deviceExtension->m_magic != kMagic)
        __debugbreak();
#endif

    PIO_STACK_LOCATION irpStack = IoGetCurrentIrpStackLocation(Irp);

    NTSTATUS status = STATUS_SUCCESS;
    switch(irpStack->MinorFunction) {
    case IRP_MN_START_DEVICE:
        status = DeviceStart(DeviceObject, Irp);
        break;
    case IRP_MN_REMOVE_DEVICE:
        status = DeviceRemove(DeviceObject, Irp);
        break;
    case IRP_MN_STOP_DEVICE:
        status = DeviceStop(DeviceObject, Irp);
        break;
    case IRP_MN_CANCEL_REMOVE_DEVICE:
    case IRP_MN_QUERY_REMOVE_DEVICE:
    case IRP_MN_QUERY_STOP_DEVICE:
    case IRP_MN_CANCEL_STOP_DEVICE:
    case IRP_MN_QUERY_DEVICE_RELATIONS:
    case IRP_MN_QUERY_INTERFACE:
    case IRP_MN_QUERY_CAPABILITIES:
    case IRP_MN_QUERY_RESOURCES:
    case IRP_MN_QUERY_RESOURCE_REQUIREMENTS:
    case IRP_MN_QUERY_DEVICE_TEXT:
    case IRP_MN_FILTER_RESOURCE_REQUIREMENTS:
    case IRP_MN_READ_CONFIG:
    case IRP_MN_WRITE_CONFIG:
    case IRP_MN_EJECT:
    case IRP_MN_SET_LOCK:
    case IRP_MN_QUERY_ID:
    case IRP_MN_QUERY_PNP_DEVICE_STATE:
    case IRP_MN_QUERY_BUS_INFORMATION:
    case IRP_MN_DEVICE_USAGE_NOTIFICATION:
    case IRP_MN_SURPRISE_REMOVAL:
    case IRP_MN_DEVICE_ENUMERATED:
    default:
        // do nothing
        break;
    }

    Irp->IoStatus.Status = status;
    Irp->IoStatus.Information = 0;

    IoCompleteRequest(Irp, IO_NO_INCREMENT );

    return STATUS_SUCCESS;
}

NTSTATUS
DevicePower(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    )
{
    PAGED_CODE();

#ifdef TODO
    DeviceExtension * deviceExtension = (DeviceExtension *) DeviceObject->DeviceExtension;

    if (deviceExtension->m_magic != kMagic)
        __debugbreak();
#endif

    PIO_STACK_LOCATION irpStack = IoGetCurrentIrpStackLocation(Irp);

    switch(irpStack->MinorFunction) {
    case IRP_MN_WAIT_WAKE:
        __debugbreak();
        break;
    case IRP_MN_POWER_SEQUENCE:
        __debugbreak();
        break;
    case IRP_MN_SET_POWER:
        break;
    case IRP_MN_QUERY_POWER:
        __debugbreak();
        break;
    default:
        // do nothing
        __debugbreak();
    }

    Irp->IoStatus.Status = STATUS_SUCCESS;
    Irp->IoStatus.Information = 0;

    IoCompleteRequest(Irp, IO_NO_INCREMENT );

    return STATUS_SUCCESS;
}

NTSTATUS
DriverLoad(PDRIVER_OBJECT DriverObject)
{
    if (doNotLoad) return STATUS_INTERNAL_ERROR;

    if(gckPLATFORM_Init(DriverObject, &gcPlatform) != gcvSTATUS_OK)
        return STATUS_INTERNAL_ERROR;

    gcPlatform->pDriverObject = DriverObject;


    gckDRIVER_SetParams();

    DriverObject->DriverExtension->AddDevice = DeviceAdd;

    DriverObject->MajorFunction[IRP_MJ_CREATE] = DeviceAttach;
    DriverObject->MajorFunction[IRP_MJ_CLOSE] = DeviceDetach;
    DriverObject->MajorFunction[IRP_MJ_DEVICE_CONTROL] = DeviceIoctl;
    DriverObject->MajorFunction[IRP_MJ_PNP] = DevicePnP;
    DriverObject->MajorFunction[IRP_MJ_POWER] = DevicePower;
    DriverObject->DriverUnload = DriverUnload;

    return STATUS_SUCCESS;
}

VOID
DriverUnload(
    _In_ PDRIVER_OBJECT DriverObject
    )
{
    PAGED_CODE();

    gckPLATFORM_Terminate(gcPlatform);
    gcPlatform = gcvNULL;

}

gctBOOL  g_bWddmMode = TRUE;
gctBOOL  g_bLimitAllocBelow4gbPa = FALSE;
NTSTATUS GcWddmCheckDriverMode();
VOID     GcWddmUnloadDispDll();
NTSTATUS GcWddmDriverEntry(__in IN DRIVER_OBJECT* pDriverObject, __in IN UNICODE_STRING* pRegistryPath);

NTSTATUS
DriverEntry(
    _In_ PDRIVER_OBJECT   DriverObject,
    _In_ PUNICODE_STRING  RegistryPath
    )
{
    UNREFERENCED_PARAMETER(RegistryPath);

#ifdef ARM64

    NTSTATUS Status;

    Status = DriverLoad(DriverObject);
    if (NT_SUCCESS(Status))
    {
        if (g_bWddmMode)
        {
            Status = GcWddmCheckDriverMode();
            if (NT_SUCCESS(Status))
            {
                Status = GcWddmDriverEntry(DriverObject, RegistryPath);
                if (!NT_SUCCESS(Status))
                {
                    GcWddmUnloadDispDll();
                }
            }
        }
    }

    return Status;

#else

    return GcWddmDriverEntry(DriverObject, RegistryPath);

#endif
}


