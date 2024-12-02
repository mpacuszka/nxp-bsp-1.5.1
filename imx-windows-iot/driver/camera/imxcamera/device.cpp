/**************************************************************************

    i.MX AVStream driver

    Copyright (c) 2001, Microsoft Corporation.
    Copyright 2022 NXP

    File:

        device.cpp

    Abstract:

        This file contains the device level implementation of the AVStream
        hardware sample.  Note that this is not the "fake" hardware.  The
        "fake" hardware is in hwsim.cpp.

    History:

        created 3/9/2001

**************************************************************************/

#include "imxcamera.h"
#include "imxcsi\Public.h"
#include "imxmipi\Public.h"
#include "imxsns_ov5640\Public.h"


/**************************************************************************

    PAGEABLE CODE

**************************************************************************/

#ifdef ALLOC_PRAGMA
#pragma code_seg("PAGE")
#endif // ALLOC_PRAGMA


NTSTATUS  CCaptureDevice::DispatchCreate (IN PKSDEVICE Device)
/*++ Routine Description:
   Create the capture device.  This is the creation dispatch for the capture device.
Arguments:
    Device -  The AVStream device being created. */
{
    PAGED_CODE();
    NTSTATUS Status;
    CCaptureDevice *CapDevice = new (NonPagedPoolNx, 'veDC') CCaptureDevice (Device);

    if (!CapDevice) {
        Status = STATUS_INSUFFICIENT_RESOURCES; // Return failure if we couldn't create the pin.
    }
    else {
        //
        // Add the item to the object bag if we were successful.
        // Whenever the device goes away, the bag is cleaned up and
        // we will be freed.
        //
        // For backwards compatibility with DirectX 8.0, we must grab
        // the device mutex before doing this.  For Windows XP, this is
        // not required, but it is still safe.
        //
        KsAcquireDevice (Device);
        Status = KsAddItemToObjectBag (
            Device -> Bag,
            reinterpret_cast <PVOID> (CapDevice),
            reinterpret_cast <PFNKSFREE> (CCaptureDevice::Cleanup)
            );
        KsReleaseDevice (Device);

        if (!NT_SUCCESS (Status)) {
            delete CapDevice;
        } else {
            Device->Context = reinterpret_cast <PVOID> (CapDevice);
        }
    }
    return Status;
}

NTSTATUS CCaptureDevice::Get_DsdAcpiResources()
/*!
 * Load device resources and device specific data.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
    NTSTATUS status;
    UINT32 length = 0;

    status = m_DsdRes.LoadDsd();
    if (NT_SUCCESS(status)) {
        const AcpiDsdRes_t::_DSDVAL_GET_DESCRIPTOR ParamTable[] = {
        {
            "MipiEndpoint0",
            (PUINT32) & (m_MipiEndpoint0[0]),
            sizeof(m_MipiEndpoint0),
            ACPI_METHOD_ARGUMENT_STRING,
        },
        {
            "CsiEndpoint0",
            (PUINT32) & (m_CsiEndpoint0[0]),
            sizeof(m_CsiEndpoint0),
            ACPI_METHOD_ARGUMENT_STRING,
        },
        {
            "SnsEndpoint0",
            (PUINT32) & (m_SnsEndpoint0[0]),
            sizeof(m_SnsEndpoint0),
            ACPI_METHOD_ARGUMENT_STRING,
        },
        {
            "CpuId",
            &m_CpuId,
            (unsigned)ACPI_METHOD_ARGUMENT_INTEGER,
        },
        };
        status = m_DsdRes.GetDsdResources(ParamTable, sizeof(ParamTable) / sizeof(ParamTable[0]));
    }
    return status;
}

NTSTATUS CCaptureDevice::InitResources(IN PCM_RESOURCE_LIST TranslatedResourceList)
{
    NTSTATUS Status = STATUS_SUCCESS;

    Status = Get_DsdAcpiResources();
    if (NT_SUCCESS(Status)) {
        if (NT_SUCCESS(Status)) {
            Status = STATUS_DEVICE_INSUFFICIENT_RESOURCES;
            Status = m_Sensor.SetFile(m_SnsEndpoint0);
        }
    }
    if (NT_SUCCESS(Status)) {
        if (NT_SUCCESS(Status)) {
            Status = STATUS_DEVICE_INSUFFICIENT_RESOURCES;
            Status = m_Csi.SetFile(m_CsiEndpoint0);
        }
    }
    if (NT_SUCCESS(Status)) {
        if (NT_SUCCESS(Status)) {
            Status = STATUS_DEVICE_INSUFFICIENT_RESOURCES;
            Status = m_Mipi.SetFile(m_MipiEndpoint0);
        }
    }
#if _DBG
    if (!NT_SUCCESS(Status)) {
        KdPrint(("\timxcamera: Failed (0x%x)\r\n", Status));
    }
#endif
    return Status;
}
/*************************************************/


NTSTATUS
CCaptureDevice::
PnpStart (
    // IN PIRP Irp,
    IN PCM_RESOURCE_LIST TranslatedResourceList,
    IN PCM_RESOURCE_LIST UntranslatedResourceList
    )

/*++

Routine Description:

    Called at Pnp start.  We start up our virtual hardware simulation.

Arguments:

    TranslatedResourceList -
        The translated resource list from Pnp

    UntranslatedResourceList -
        The untranslated resource list from Pnp

Return Value:

    Success / Failure

--*/

{

    PAGED_CODE();
    
    NTSTATUS Status = STATUS_SUCCESS;
    
    Status = InitResources(TranslatedResourceList);

    if (NT_SUCCESS(Status)) {
        
        //
        // Normally, we'd do things here like parsing the resource lists and
        // connecting our interrupt.  Since this is a simulation, there isn't
        // much to parse.  The parsing and connection should be the same as
        // any WDM driver.  The sections that will differ are illustrated below
        // in setting up a simulated DMA.
        //

        if (!m_Device -> Started) {
            // Create the Filter for the device
            KsAcquireDevice(m_Device);

            switch (m_CpuId) {
                case IMX_CPU_MX8MP:
                case IMX_CPU_MX8MN:
                    CaptureFilterPinDescriptors[0].PinDescriptor.DataRangesCount = (ULONG) GetIsiCapturePinRangesCount();
                    CaptureFilterPinDescriptors[0].PinDescriptor.DataRanges = CapturePinIsiDataRanges;
                    break;
                default:
                case IMX_CPU_MX8MQ:
                case IMX_CPU_MX8MM:
                    CaptureFilterPinDescriptors[0].PinDescriptor.DataRangesCount = (ULONG) GetCsiCapturePinRangesCount();
                    CaptureFilterPinDescriptors[0].PinDescriptor.DataRanges = CapturePinCsiDataRanges;
                    break;
            }

            Status = KsCreateFilterFactory( m_Device->FunctionalDeviceObject,
                                            &CaptureFilterDescriptor,
                                            L"GLOBAL",
                                            NULL,
                                            KSCREATE_ITEM_FREEONSTOP,
                                            NULL,
                                            NULL,
                                            NULL );
            KsReleaseDevice(m_Device);
        }
    }
    
    return Status;

}

/*************************************************/

void
CCaptureDevice::
PnpStop (
    )

/*++

Routine Description:

    This is the pnp stop dispatch for the capture device.  It releases any
    adapter object previously allocated by IoGetDmaAdapter during Pnp Start.

Arguments:

    None

Return Value:

    None

--*/

{

    PAGED_CODE();

    m_Sensor.PnpStop();
    m_Csi.PnpStop();
    m_Mipi.PnpStop();
    if (m_DmaAdapterObject) {
        //
        // Return the DMA adapter back to the system.
        //
        m_DmaAdapterObject -> DmaOperations -> 
            PutDmaAdapter (m_DmaAdapterObject);

        m_DmaAdapterObject = NULL;
    }
    m_DsdRes.Cleanup();
}

/*************************************************/


NTSTATUS
CCaptureDevice::
AcquireHardwareResources (
    IN PKS_VIDEOINFOHEADER VideoInfoHeader,
    IN video_pixel_format_t PixelFmt
    )

/*++

Routine Description:

    Acquire hardware resources for the capture hardware.  If the 
    resources are already acquired, this will return an error.
    The hardware configuration must be passed as a VideoInfoHeader.

Arguments:

    CaptureSink -
        The capture sink attempting to acquire resources.  When scatter /
        gather mappings are completed, the capture sink specified here is
        what is notified of the completions.

    VideoInfoHeader -
        Information about the capture stream.  This **MUST** remain
        stable until the caller releases hardware resources.  Note
        that this could also be guaranteed by bagging it in the device
        object bag as well.

Return Value:

    Success / Failure

--*/

{

    PAGED_CODE();
    NTSTATUS Status = STATUS_SUCCESS;

    //
    // If we're the first pin to go into acquire (remember we can have
    // a filter in another graph going simultaneously), grab the resources.
    //
    if (InterlockedCompareExchange (
        &m_PinsWithResources,
        1,
        0) == 0) {
        
        m_VideoInfoHeader = VideoInfoHeader;
        m_PixelFmt = PixelFmt;

        if (NT_SUCCESS(Status)) {
            Status = m_Sensor.Open();
            if (!NT_SUCCESS(Status)) {
                _DbgPrint(("Failed to open sensor device. (0x%x)\r\n", Status));
            }
        }
        if (NT_SUCCESS(Status)) {
            Status = m_Mipi.Open();
            if (!NT_SUCCESS(Status)) {
                _DbgPrint(("Failed to open MIPI device. (0x%x)\r\n", Status));
            }
        }
        if (NT_SUCCESS(Status)) {
            Status = m_Csi.Open();
            if (!NT_SUCCESS(Status)) {
                _DbgPrint(("Failed to open CSI device. (0x%x)\r\n", Status));
            }
        }
        if (!NT_SUCCESS(Status)) {
            ReleaseHardwareResources();
        }
    
    }
    else {

        //
        // TODO: Better Status code?
        //
        Status = STATUS_SHARING_VIOLATION;

    }

    return Status;

}

/*************************************************/

void
CCaptureDevice::
ReleaseHardwareResources (
    )

/*++

Routine Description:

    Release hardware resources.  This should only be called by
    an object which has acquired them.

Arguments:

    None

Return Value:

    None

--*/

{

    PAGED_CODE();

    m_Sensor.Close();
    m_Mipi.Close();
    m_Csi.Close();

    //
    // Release our "lock" on hardware resources.  This will allow another
    // pin (perhaps in another graph) to acquire them.
    //
    InterlockedExchange (
        &m_PinsWithResources,
        0
        );

}

/*************************************************/

NTSTATUS
CCaptureDevice::
Start (
    )

/*++

Routine Description:

    Start the capture device based on the video info header we were told
    about when resources were acquired.

Arguments:

    None

Return Value:

    Success / Failure

--*/

{

    PAGED_CODE();
    NTSTATUS Status = STATUS_SUCCESS;

    m_LastMappingsCompleted = 0;
    m_InterruptTime = 0;

    if (NT_SUCCESS(Status)) {

        UINT8 csiLanes = 2;
        camera_config_t cfg{ kVIDEO_Resolution720P, m_PixelFmt, m_PixelFmt, 25, 0, csiLanes};

        auto* mipi = m_Mipi.getTarget();
        auto* csi = m_Csi.getTarget();
        auto* sensor = m_Sensor.getTarget();

        if (mipi == NULL || csi == NULL || sensor == NULL) {
            Status = STATUS_INVALID_DEVICE_STATE;
        }

        if (NT_SUCCESS(Status)) {
            NTSTATUS TmpStatus = STATUS_SUCCESS;
            video_pixel_format_t requiredCameraFmt = kVIDEO_PixelFormatNone;

            PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_CSI_REQUIRED_FMT, csi->m_TargetDevicePtr, (PUCHAR)&m_PixelFmt, sizeof(m_PixelFmt), &requiredCameraFmt, sizeof(requiredCameraFmt), FALSE, &csi->m_Event, &csi->m_IoStatus);

            if (LowIrp) {
                TmpStatus = csi->SendIrp(LowIrp, &csi->m_Event);
                if (!NT_SUCCESS(Status)) {
                    _DbgPrint(("\timxcamera: SendIrp failed 0x%x\r\n", TmpStatus));
                    IoFreeIrp(LowIrp);
                }
            } else {
                TmpStatus = STATUS_MEMORY_NOT_ALLOCATED;
            }
            if (NT_SUCCESS(TmpStatus)) {
                TmpStatus = csi->m_IoStatus.Status;
            }
            if (NT_SUCCESS(TmpStatus)) {
                cfg.cameraPixelFormat = requiredCameraFmt;
            }

            LowIrp = IoBuildDeviceIoControlRequest(IOCTL_MIPI_REQUIRED_FMT, mipi->m_TargetDevicePtr, (PUCHAR)&cfg.cameraPixelFormat, sizeof(cfg.cameraPixelFormat), &requiredCameraFmt, sizeof(requiredCameraFmt), FALSE, &mipi->m_Event, &mipi->m_IoStatus);
            if (LowIrp) {
                TmpStatus = mipi->SendIrp(LowIrp, &mipi->m_Event);
                if (!NT_SUCCESS(TmpStatus)) {
                    _DbgPrint(("\timxcamera: SendIrp failed 0x%x\r\n", TmpStatus));
                    IoFreeIrp(LowIrp);
                }
            } else {
                TmpStatus = STATUS_MEMORY_NOT_ALLOCATED;
            }
            if (NT_SUCCESS(TmpStatus)) {
                TmpStatus = mipi->m_IoStatus.Status;
            }
            if (NT_SUCCESS(TmpStatus)) {
                cfg.cameraPixelFormat = requiredCameraFmt;
            }
        }

        /* Configure camera */
        if (NT_SUCCESS(Status)) {
            UINT32 mipiLaneClk = 0;
            PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_SNS_CONFIGURE, sensor->m_TargetDevicePtr, (PUCHAR)&cfg, sizeof(cfg), &mipiLaneClk, sizeof(mipiLaneClk), FALSE, &sensor->m_Event, &sensor->m_IoStatus);

            if (LowIrp) {
                Status = sensor->SendIrp(LowIrp, &sensor->m_Event);
                if (!NT_SUCCESS(Status)) {
                    _DbgPrint(("\timxcamera: SendIrp failed 0x%x\r\n", Status));
                    IoFreeIrp(LowIrp);
                }
            } else {
                Status = STATUS_MEMORY_NOT_ALLOCATED;
            }
            if (NT_SUCCESS(Status)) {
                Status = sensor->m_IoStatus.Status;
            }
            if (NT_SUCCESS(Status)) {
                cfg.mipiLaneClk = mipiLaneClk;
            }
        }

        /* Configure MIPI CSI 2 */
        if (NT_SUCCESS(Status)) {
            PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_MIPI_CONFIGURE, mipi->m_TargetDevicePtr, (PUCHAR)&cfg, sizeof(cfg), NULL, 0, FALSE, &mipi->m_Event, &mipi->m_IoStatus);
            if (LowIrp) {
                Status = mipi->SendIrp(LowIrp, &mipi->m_Event);
                if (!NT_SUCCESS(Status)) {
                    _DbgPrint(("\timxcamera: SendIrp failed 0x%x\r\n", Status));
                    IoFreeIrp(LowIrp);
                }
            } else {
                Status = STATUS_MEMORY_NOT_ALLOCATED;
            }
            if (NT_SUCCESS(Status)) {
                Status = mipi->m_IoStatus.Status;
            }
        }

        /* Configure ISI/CSI */
        if (NT_SUCCESS(Status)) {
            PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_CSI_DRIVER_INIT, csi->m_TargetDevicePtr, (PUCHAR)&cfg, sizeof(cfg), NULL, 0, FALSE, &csi->m_Event, &csi->m_IoStatus);
            if (LowIrp) {
                Status = csi->SendIrp(LowIrp, &csi->m_Event);
                if (!NT_SUCCESS(Status)) {
                    _DbgPrint(("\timxcamera: SendIrp failed 0x%x\r\n", Status));
                    IoFreeIrp(LowIrp);
                }
            } else {
                Status = STATUS_MEMORY_NOT_ALLOCATED;
            }
            if (NT_SUCCESS(Status)) {
                Status = csi->m_IoStatus.Status;
            }
        }

        /* Enable MIPI CSI 2 output from camera */
        if (NT_SUCCESS(Status)) {
            PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_SNS_START, sensor->m_TargetDevicePtr, NULL, 0, NULL, 0, FALSE, &sensor->m_Event, &sensor->m_IoStatus);

            if (LowIrp) {
                Status = sensor->SendIrp(LowIrp, &sensor->m_Event);
                if (!NT_SUCCESS(Status)) {
                    _DbgPrint(("\timxcamera: SendIrp IOCTL_SNS_STARTfailed 0x%x\r\n", Status));
                    IoFreeIrp(LowIrp);
                }
            } else {
                Status = STATUS_MEMORY_NOT_ALLOCATED;
            }
            if (NT_SUCCESS(Status)) {
                Status = sensor->m_IoStatus.Status;
            }
        }
    }
    return Status;
}

/*************************************************/

NTSTATUS
CCaptureDevice::
Pause (
    IN BOOLEAN Pausing
    )

/*++

Routine Description:

    Pause or unpause the hardware simulation.  This is an effective start
    or stop without resetting counters and formats.  Note that this can
    only be called to transition from started -> paused -> started.  Calling
    this without starting the hardware with Start() does nothing.

Arguments:

    Pausing -
        An indicatation of whether we are pausing or unpausing

        TRUE -
            Pause the hardware simulation

        FALSE -
            Unpause the hardware simulation

Return Value:

    Success / Failure

--*/

{
    NTSTATUS Status;
    PAGED_CODE();

    m_Csi.Cancel();

    auto *csi = m_Csi.getTarget();

    if (csi == NULL) {
        Status = STATUS_INVALID_DEVICE_STATE;
    } else {

        PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_CSI_DRIVER_STOP, csi->m_TargetDevicePtr, NULL, 0, NULL, 0, FALSE, &csi->m_Event, &csi->m_IoStatus);

        if (LowIrp) {
            Status = csi->SendIrp(LowIrp, &csi->m_Event);
            if (!NT_SUCCESS(Status)) {
                _DbgPrint(("\timxcamera: SendIrp failed 0x%x\r\n", Status));
                IoFreeIrp(LowIrp);
            }
        } else {
            Status = STATUS_MEMORY_NOT_ALLOCATED;
        }
        if (NT_SUCCESS(Status)) {
            Status = csi->m_IoStatus.Status;
        }
    }
    return Status;
}

/*************************************************/

NTSTATUS
CCaptureDevice::
Stop (
    )

/*++

Routine Description:

    Stop the capture device.

Arguments:

    None

Return Value:

    Success / Failure

--*/

{
    NTSTATUS Status = STATUS_SUCCESS;

    PAGED_CODE();

    if (NT_SUCCESS(Status)) {

        auto* sensor = m_Sensor.getTarget();

        if (sensor == NULL) {
            Status = STATUS_INVALID_DEVICE_STATE;
        } else {

            PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_SNS_STOP, sensor->m_TargetDevicePtr, NULL, 0, NULL, 0, FALSE, &sensor->m_Event, &sensor->m_IoStatus);

            if (LowIrp) {
                Status = sensor->SendIrp(LowIrp, &sensor->m_Event);
                if (!NT_SUCCESS(Status)) {
                    _DbgPrint(("\timxcamera: IOCTL_MIPI_STOP SendIrp failed 0x%x\r\n", Status));
                    IoFreeIrp(LowIrp);
                }
            } else {
                Status = STATUS_MEMORY_NOT_ALLOCATED;
            }
            if (NT_SUCCESS(Status)) {
                Status = sensor->m_IoStatus.Status;
            }
        }
    }

    if (NT_SUCCESS(Status)) {

        auto *mipi = m_Mipi.getTarget();

        if (mipi == NULL) {
            Status = STATUS_INVALID_DEVICE_STATE;
        } else {

            PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_MIPI_STOP, mipi->m_TargetDevicePtr, NULL, 0, NULL, 0, FALSE, &mipi->m_Event, &mipi->m_IoStatus);

            if (LowIrp) {
                Status = mipi->SendIrp(LowIrp, &mipi->m_Event);
                if (!NT_SUCCESS(Status)) {
                    _DbgPrint(("\timxcamera: SendIrp failed 0x%x\r\n", Status));
                    IoFreeIrp(LowIrp);
                }
            } else {
                Status = STATUS_MEMORY_NOT_ALLOCATED;
            }
            if (NT_SUCCESS(Status)) {
                Status = mipi->m_IoStatus.Status;
            }
        }
    }

    auto *csi = m_Csi.getTarget();

    if (csi == NULL) {
        Status = STATUS_INVALID_DEVICE_STATE;
    } else {

        PIRP LowIrp = IoBuildDeviceIoControlRequest(IOCTL_CSI_DRIVER_STOP, csi->m_TargetDevicePtr, NULL, 0, NULL, 0, FALSE, &csi->m_Event, &csi->m_IoStatus);

        if (LowIrp) {
            Status = csi->SendIrp(LowIrp, &csi->m_Event);
            if (!NT_SUCCESS(Status)) {
                _DbgPrint(("\timxcamera: SendIrp failed 0x%x\r\n", Status));
                IoFreeIrp(LowIrp);
            }
        } else {
            Status = STATUS_MEMORY_NOT_ALLOCATED;
        }
        if (NT_SUCCESS(Status)) {
            Status = csi->m_IoStatus.Status;
        }
    }
    return Status;
}

/*************************************************/

/*************************************************************************

    LOCKED CODE

**************************************************************************/

#ifdef ALLOC_PRAGMA
#pragma code_seg()
#endif // ALLOC_PRAGMA

ULONG
CCaptureDevice::
QueryInterruptTime (
    )

/*++

Routine Description:

    Return the number of frame intervals that have elapsed since the
    start of the device.  This will be the frame number.

Arguments:

    None

Return Value:

    The interrupt time of the device (the number of frame intervals that
    have elapsed since the start of the device).

--*/

{
    m_InterruptTime++;

    return m_InterruptTime;

}

/*************************************************/

/**************************************************************************

    DESCRIPTOR AND DISPATCH LAYOUT

**************************************************************************/

//
// CaptureDeviceDispatch:
//
// This is the dispatch table for the capture device.  Plug and play
// notifications as well as power management notifications are dispatched
// through this table.
//
const
KSDEVICE_DISPATCH
CaptureDeviceDispatch = {
    CCaptureDevice::DispatchCreate,         // Pnp Add Device
    CCaptureDevice::DispatchPnpStart,       // Pnp Start
    NULL,                                   // Post-Start
    NULL,                                   // Pnp Query Stop
    NULL,                                   // Pnp Cancel Stop
    CCaptureDevice::DispatchPnpStop,        // Pnp Stop
    NULL,                                   // Pnp Query Remove
    NULL,                                   // Pnp Cancel Remove
    NULL,                                   // Pnp Remove
    NULL,                                   // Pnp Query Capabilities
    NULL,                                   // Pnp Surprise Removal
    NULL,                                   // Power Query Power
    NULL,                                   // Power Set Power
    NULL                                    // Pnp Query Interface
};

//
// CaptureDeviceDescriptor:
//
// This is the device descriptor for the capture device.  It points to the
// dispatch table and contains a list of filter descriptors that describe
// filter-types that this device supports.  Note that the filter-descriptors
// can be created dynamically and the factories created via 
// KsCreateFilterFactory as well.  
//
const
KSDEVICE_DESCRIPTOR
CaptureDeviceDescriptor = {
    &CaptureDeviceDispatch,
    0,
    NULL
};

/**************************************************************************

    INITIALIZATION CODE

**************************************************************************/

extern "C" DRIVER_INITIALIZE DriverEntry;

extern "C"
NTSTATUS
DriverEntry (
    IN PDRIVER_OBJECT DriverObject,
    IN PUNICODE_STRING RegistryPath
    )

/*++

Routine Description:

    Driver entry point.  Pass off control to the AVStream initialization
    function (KsInitializeDriver) and return the Status code from it.

Arguments:

    DriverObject -
        The WDM driver object for our driver

    RegistryPath -
        The registry path for our registry info

Return Value:

    As from KsInitializeDriver

--*/

{
    //
    // Simply pass the device descriptor and parameters off to AVStream
    // to initialize us.  This will cause filter factories to be set up
    // at add & start.  Everything is done based on the descriptors passed
    // here.
    //
    return KsInitializeDriver (
            DriverObject,
            RegistryPath,
            &CaptureDeviceDescriptor
            );

}

