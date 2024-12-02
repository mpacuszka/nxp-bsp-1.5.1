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

#include "precomp.h"
#pragma hdrstop

#include "imxcanhw.h"
#include "imxcan.h"

#pragma warning(push)

_Use_decl_annotations_
BOOLEAN ImxCanEvtInterruptIsr(WDFINTERRUPT WdfInterrupt, ULONG MessageID)
{
    UNREFERENCED_PARAMETER(MessageID);
    PDEV_CONTEXT deviceContextPtr = ImxCanGetDeviceContext(WdfInterruptGetDevice(WdfInterrupt));
    volatile IMXCAN_REGISTERS *registersPtr = deviceContextPtr->RegistersPtr;
    PINT_CONTEXT interruptContextPtr = ImxCanGetInterruptContext(WdfInterrupt);

    {
        data_callback item;

        status_t status;
        UINT32 mbNum = 0xFFU;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
        UINT64 result = 0U;
#else
        UINT32 result = 0U;
#endif
        do
        {
            /* Get Current FlexCAN Module Error and Status. */
            result = FLEXCAN_GetStatusFlags(registersPtr);

            /* To handle FlexCAN Error and Status Interrupt first. */
            if (0U != (result & FLEXCAN_ERROR_AND_STATUS_INIT_FLAG)) {
                status = kStatus_FLEXCAN_ErrorStatus;
                /* Clear FlexCAN Error and Status Interrupt. */
                FLEXCAN_ClearStatusFlags(registersPtr, FLEXCAN_ERROR_AND_STATUS_INIT_FLAG);
            }
            else if (0U != (result & FLEXCAN_WAKE_UP_FLAG)) {
                status = kStatus_FLEXCAN_WakeUp;
                FLEXCAN_ClearStatusFlags(registersPtr, FLEXCAN_WAKE_UP_FLAG);
            }
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
            else if ((0U != (result & FLEXCAN_MEMORY_ENHANCED_RX_FIFO_INIT_FLAG)) && (0u != (base->ERFIER & FLEXCAN_MEMORY_ENHANCED_RX_FIFO_INIT_MASK))) {
                status = FLEXCAN_SubHandlerForEhancedRxFifo(base, handle, result);
            }
#endif
            else {
                /* To handle Message Buffer or Legacy Rx FIFO transfer. */
                status = FLEXCAN_SubHandlerForDataTransfered(registersPtr, &deviceContextPtr->CANhandle, &mbNum);
                result = mbNum;
            }

            /* Write item to the circular buffer FrameQueue */
            {
                item.status = status;
                item.result = result;
                if (status == kStatus_FLEXCAN_RxIdle) {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
                    if (0U != (registersPtr->MCR & CAN_MCR_FDEN_MASK)) {
                        item.FDframe = *deviceContextPtr->CANhandle.copymbFDFrame;
                    }
                    else
#endif
                    {
                        item.frame = *deviceContextPtr->CANhandle.copymbFrame;
                    }
                }
                if (interruptContextPtr->FrameQueue.count == interruptContextPtr->FrameQueue.capacity) {
                    /* Buffer is full. The oldest item in the buffer is lost! */
                    NT_ASSERTMSG("Circular buffer FrameQueue is full", FALSE);
                    interruptContextPtr->FrameQueue.tail = (char*)interruptContextPtr->FrameQueue.tail + interruptContextPtr->FrameQueue.sz;
                    if (interruptContextPtr->FrameQueue.tail == interruptContextPtr->FrameQueue.buffer_end) {
                        interruptContextPtr->FrameQueue.tail = interruptContextPtr->FrameQueue.buffer;
                    }
                    RtlCopyMemory(interruptContextPtr->FrameQueue.head, &item, interruptContextPtr->FrameQueue.sz);
                    interruptContextPtr->FrameQueue.head = (char*)interruptContextPtr->FrameQueue.head + interruptContextPtr->FrameQueue.sz;
                    if (interruptContextPtr->FrameQueue.head == interruptContextPtr->FrameQueue.buffer_end) {
                        interruptContextPtr->FrameQueue.head = interruptContextPtr->FrameQueue.buffer;
                    }
                    interruptContextPtr->InterruptStatus++  /* Increment counter of lost events */;
                }
                else {
                    /* Writes to the buffer in the standard way. */
                    RtlCopyMemory(interruptContextPtr->FrameQueue.head, &item, interruptContextPtr->FrameQueue.sz);
                    interruptContextPtr->FrameQueue.head = (char*)interruptContextPtr->FrameQueue.head + interruptContextPtr->FrameQueue.sz;
                    if (interruptContextPtr->FrameQueue.head == interruptContextPtr->FrameQueue.buffer_end) {
                        interruptContextPtr->FrameQueue.head = interruptContextPtr->FrameQueue.buffer;
                    }
                    interruptContextPtr->FrameQueue.count++;
                }
            }
        } while (FLEXCAN_CheckUnhandleInterruptEvents(registersPtr));
    }
 /* Continue in DPC, interrupts remain enabled!!! */
    WdfInterruptQueueDpcForIsr(WdfInterrupt);
    return TRUE; 
}

_Use_decl_annotations_
VOID ImxCanEvtInterruptDpc(WDFINTERRUPT WdfInterrupt, WDFOBJECT AssociatedWdfObject)
{
    UNREFERENCED_PARAMETER(AssociatedWdfObject);
    PDEV_CONTEXT deviceContextPtr = ImxCanGetDeviceContext(WdfInterruptGetDevice(WdfInterrupt));
    PINT_CONTEXT interruptContextPtr = ImxCanGetInterruptContext(WdfInterrupt);
    WDFREQUEST currentRequest = deviceContextPtr->CurrentRequest;
    WDFREQUEST notifyRequest;
    //volatile IMXCAN_REGISTERS *registersPtr = deviceContextPtr->RegistersPtr;
    data_callback *outputBufferPtr;
    LONG valueToReturn;
    data_callback item;

    if (currentRequest != NULL) {
        /* CurrentRequest is currently used only by SendNonBlocking... */
        KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "SendNonBlocking() is currently active\n"));
    }
    do {
        NTSTATUS status = WdfIoQueueRetrieveNextRequest(deviceContextPtr->NotificationQueue, &notifyRequest);
        if (NT_SUCCESS(status)) {
            status = WdfRequestRetrieveOutputBuffer(notifyRequest, sizeof(status), (PVOID*)&outputBufferPtr, NULL);
            if (!NT_SUCCESS(status)) {
                KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfRequestRetrieveOutputBuffer(..) failed. (status = %!STATUS!)\n", status));
                WdfRequestComplete(notifyRequest, status);
                return;
            }
            /* Read item from circular buffer FrameQueue */
            {
                if (interruptContextPtr->FrameQueue.count == 0) {
                    NT_ASSERTMSG("cb handle error", FALSE);
                }
                WdfInterruptAcquireLock(deviceContextPtr->WdfInterrupt);
                RtlCopyMemory(&item, interruptContextPtr->FrameQueue.tail, interruptContextPtr->FrameQueue.sz);
                interruptContextPtr->FrameQueue.tail = (char*)interruptContextPtr->FrameQueue.tail + interruptContextPtr->FrameQueue.sz;
                if (interruptContextPtr->FrameQueue.tail == interruptContextPtr->FrameQueue.buffer_end) {
                    interruptContextPtr->FrameQueue.tail = interruptContextPtr->FrameQueue.buffer;
                }
                interruptContextPtr->FrameQueue.count--;
                WdfInterruptReleaseLock(deviceContextPtr->WdfInterrupt);
            }
            valueToReturn = InterlockedAdd(&deviceContextPtr->ControllerInfo.Sequence, 1);
            RtlCopyMemory(outputBufferPtr, &item, sizeof(*outputBufferPtr));
            WdfRequestCompleteWithInformation(notifyRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
        }
        else {
            /* This is a case when the notification queue is empty (because the user application has failed to send
             * a sufficient number of IOCTL_CAN_INVERT_NOTIFICATION requests). Driver will either need to store the event
             * in some driver-specific way, or the user application will simply not be notified of this event occurrence.
             */
            /* NT_ASSERTMSG("the notification queue is empty", FALSE); */
            /* KdPrintEx((DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, "WdfIoQueueRetrieveNextRequest(..) failed. (status = %!STATUS!)\n", status)); */
            break;
        }
    } while (interruptContextPtr->FrameQueue.count != 0);
    deviceContextPtr->CurrentRequest = NULL;
}

#pragma warning(pop)