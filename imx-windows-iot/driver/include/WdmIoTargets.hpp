/*
 * Copyright 2022 NXP
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

struct iotarget_t
{
    HANDLE m_TargetHandle;
    OBJECT_ATTRIBUTES m_FileAttributes;
    IO_STATUS_BLOCK ioStatus;
    PFILE_OBJECT m_TargetFileObjectPtr;
    PDEVICE_OBJECT  m_TargetDevicePtr;
    ACCESS_MASK DesiredAccess = GENERIC_WRITE;
    // Have a default event and stuff
    KEVENT m_Event;
    IO_STATUS_BLOCK m_IoStatus;

    PIRP m_OngiongIrpPtr;

    CHAR m_DeviceEndpoint[DEVICE_ENDPOINT_NAME_MAX_LEN];
    WCHAR m_DeviceEndpointUnicodeNameBuff[DEVICE_ENDPOINT_NAME_MAX_LEN];
    UNICODE_STRING m_DeviceEndpointUnicodeName;

    iotarget_t() : m_OngiongIrpPtr(NULL), m_DeviceEndpointUnicodeName{0}
    {
        KeInitializeEvent(&m_Event, NotificationEvent, FALSE);
        RtlInitEmptyUnicodeString(&m_DeviceEndpointUnicodeName, m_DeviceEndpointUnicodeNameBuff, sizeof(m_DeviceEndpointUnicodeNameBuff));
    }

    ~iotarget_t()
    {
        Close();
    }

    iotarget_t *getTarget()
    {
        if (m_TargetHandle != NULL) {
            return this;
        }
        else {
            return NULL;
        }
    }

    NTSTATUS SetFile(char * DeviceEndpointName)
    {
        NTSTATUS Status = STATUS_SUCCESS;
        ANSI_STRING deviceEndpointAnsiName;

        _Analysis_assume_nullterminated_(DeviceEndpointName);
        Status = RtlInitAnsiStringEx(&deviceEndpointAnsiName, (PCSZ)(DeviceEndpointName));
        if (NT_SUCCESS(Status)) {
            Status = RtlAnsiStringToUnicodeString(&m_DeviceEndpointUnicodeName, &deviceEndpointAnsiName, FALSE);
        }
        return Status;
    }

    NTSTATUS Open()
    {
        NTSTATUS Status = STATUS_SUCCESS;

        if (NT_SUCCESS(Status)) {
            InitializeObjectAttributes(&m_FileAttributes, &m_DeviceEndpointUnicodeName, OBJ_KERNEL_HANDLE, NULL, NULL);
            Status = ZwOpenFile(&m_TargetHandle, DesiredAccess, &m_FileAttributes, &ioStatus, 0, 0);
            if (NT_SUCCESS(Status)) {
                Status = ObReferenceObjectByHandle(m_TargetHandle, DesiredAccess, *IoFileObjectType, KernelMode, (PVOID*)&m_TargetFileObjectPtr, NULL);
                if (NT_SUCCESS(Status)) {
                    m_TargetDevicePtr = IoGetRelatedDeviceObject(m_TargetFileObjectPtr);
                    if (m_TargetDevicePtr == NULL) {
                        Status = STATUS_NO_SUCH_DEVICE;
                    }
                }
            }
            if (!NT_SUCCESS(Status)) {
                Close();
            }
        }
        return Status;
    }

    void Close()
    {
        if (m_TargetFileObjectPtr != NULL) {
            ObDereferenceObject(m_TargetFileObjectPtr);
            m_TargetFileObjectPtr = NULL;
        }
        if (m_TargetHandle != NULL) {
            ZwClose(m_TargetHandle);
            m_TargetHandle = NULL;
        }
    }

    void PnpStop()
    {
        Close();
    }

    NTSTATUS SendIrp(PIRP irpPtr, KEVENT *keEventPtr)
    {
        NTSTATUS Status = STATUS_SUCCESS;
        ASSERT(irpPtr != NULL);
        PIO_STACK_LOCATION irpSp = IoGetNextIrpStackLocation(irpPtr);

        if (irpSp == NULL) {
            Status = STATUS_INVALID_DEVICE_REQUEST;
        }
        irpSp->FileObject = m_TargetFileObjectPtr;
        if (keEventPtr != NULL) {
            KeClearEvent(keEventPtr);
        }
        // lock
        ASSERT(m_OngiongIrpPtr == NULL);
        m_OngiongIrpPtr = irpPtr;
        // unlock
        Status = IoCallDriver(m_TargetDevicePtr, irpPtr);
        if (Status == STATUS_PENDING){
            if (keEventPtr != NULL) {
                LARGE_INTEGER timeout;

                timeout.QuadPart = (LONGLONG)-30000000;
                Status = KeWaitForSingleObject(keEventPtr, Executive, KernelMode, FALSE, &timeout);
                // lock
                ASSERT(m_OngiongIrpPtr == irpPtr);
                if (Status == STATUS_TIMEOUT) {
                    BOOLEAN notPending = false;

                    notPending = IoCancelIrp(m_OngiongIrpPtr);
                    if (!notPending) {
                        KeWaitForSingleObject(keEventPtr, Executive, KernelMode, FALSE, NULL);
                    }
                }
                m_OngiongIrpPtr = NULL;
                // unlock
            }
        }
        return Status;
    }

    BOOLEAN Cancel()
    {
        BOOLEAN notPending = true;

        if (m_OngiongIrpPtr != NULL) {
            notPending = IoCancelIrp(m_OngiongIrpPtr);
        }
        return notPending;
    }
};