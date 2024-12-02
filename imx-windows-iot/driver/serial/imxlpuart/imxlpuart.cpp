// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
// Module Name:
//
//    imxlpuart.cpp
//
// Abstract:
//
//    This module contains the iMX LPUART driver implementation which uses
//    the SerCx2 framework.
//

#include <wdm.h>

#include "precomp.h"
#include "imxlpuarthw.h"
#include "imxlpuart.h"
#include "HalExtiMXDmaCfg.h"

#include "trace.h"
#include "imxlpuart.tmh"

#define SECURITY
#define REGISTRY
#define INTERRUPT
#define ISR
#define POWER
#define DPC

IMX_LPUART_NONPAGED_SEGMENT_BEGIN; //==============================================

#if DBG
/// The structure for register description.
///
typedef struct Register_s {
    char *name;
    struct {
        unsigned position, width;
        char *name, *description;
    } flag[32];
} Register_t;

/**
 * Control Register Description
 */
static Register_t CTRL = {
    "Control Register",
    0, 1, "PT", "Parity Type",
    1, 1, "PE", "Parity Enable",
    2, 1, "ILT", "Idle Line Type Select",
    3, 1, "WAKE", "Receiver Wakeup Method Select",
    4, 1, "M", "9-Bit or 8-Bit Mode Select",
    5, 1, "RSRC", "Receiver Source Select",
    6, 1, "DOZEEN", "Doze Enable",
    7, 1, "LOOPS", "Loop Mode Select",
    8, 3, "IDLECFG", "Idle Configuration",
    11, 1, "M7", "7-Bit Mode Select",
    12, 2, "", "Reserved",
    14, 1, "MA2IE", "Match 2 Interrupt Enable",
    15, 1, "MA1IE", "Match 1 Interrupt Enable",
    16, 1, "SBK", "Send Break",
    17, 1, "RWU", "Receiver Wakeup Control",
    18, 1, "RE", "Receiver Enable",
    19, 1, "TE", "Transmitter Enable",
    20, 1, "ILIE", "Idle Line Interrupt Enable",
    21, 1, "RIE", "Receiver Interrupt Enable",
    22, 1, "TCIE", "Transmission Complete Interrupt Enable",
    23, 1, "TIE", "Transmit Interrupt Enable",
    24, 1, "PEIE", "Parity Error Interrupt Enable",
    25, 1, "FEIE", "Framing Error Interrupt Enable",
    26, 1, "NEIE", "Noise Error Interrupt Enable",
    27, 1, "ORIE", "Overrun Interrupt Enable",
    28, 1, "TXINV", "Transmit Data Inversion",
    29, 1, "TXDIR", "TX Pin Direction in Single-Wire Mode",
    30, 1, "R9T8", "Receive Bit 9 / Transmit Bit 8",
    31, 1, "R8T9", "Receive Bit 8 / Transmit Bit 9"
};

/**
 * Helper function for printout of the register flags
 */
static void dump(int line, Register_t reg, UINT32 value)
{
    IMX_LPUART_LOG_TRACE("#%d %s", line, reg.name);
    for (unsigned i = 0; i < 32; i++) {
        if (reg.flag[i].width == 1) {
            if (value & (1 << reg.flag[i].position)) {
                IMX_LPUART_LOG_TRACE(" %s", reg.flag[i].name);
            }
        }
    }
}

#define Dump(r, v) dump(__LINE__, r, v)
#else
#define Dump(r, v)
#endif // DBG

static void dmp_regs(IMX_LPUART_REGISTERS *RegsPtr)
{
    IMX_LPUART_LOG_TRACE("\t VERID;  : 0x%x", RegsPtr->VERID);
    IMX_LPUART_LOG_TRACE("\t PARAM;  : 0x%x", RegsPtr->PARAM);
    IMX_LPUART_LOG_TRACE("\t GLOBAL; : 0x%x", RegsPtr->GLOBAL);
    IMX_LPUART_LOG_TRACE("\t PINCNF; : 0x%x", RegsPtr->PINCNF);
    IMX_LPUART_LOG_TRACE("\t BAUD;   : 0x%x", RegsPtr->BAUD);
    IMX_LPUART_LOG_TRACE("\t STAT;   : 0x%x", RegsPtr->STAT);
    IMX_LPUART_LOG_TRACE("\t CTRL;   : 0x%x", RegsPtr->CTRL);
    // IMX_LPUART_LOG_TRACE("\t DATA;   : 0x%x", RegsPtr->DATA); Do not read out the Rx Data
    IMX_LPUART_LOG_TRACE("\t MATCH;  : 0x%x", RegsPtr->MATCH);
    IMX_LPUART_LOG_TRACE("\t MODIR;  : 0x%x", RegsPtr->MODIR);
    IMX_LPUART_LOG_TRACE("\t FIFO;   : 0x%x", RegsPtr->FIFO);
    IMX_LPUART_LOG_TRACE("\t WATER;  : 0x%x", RegsPtr->WATER);
}

/**
 * Placement new and delete operators
 */

_Use_decl_annotations_
void *operator new (size_t, void *Ptr) throw ()
{
    return Ptr;
}

void operator delete (void*, void*) throw ()
{}

_Use_decl_annotations_
void *operator new[](size_t, void *Ptr) throw ()
{
    return Ptr;
}

void operator delete[](void*, void*) throw ()
{}

_Use_decl_annotations_
ULONG
IMX_LPUART_RING_BUFFER::EnqueueBytes(
    const UCHAR *InputBufferPtr,
    ULONG InputBufferSize
)
{
    ULONG bytesCopied = 0;

    if (InputBufferSize > 0) {
        const ULONG tail = ReadULongAcquire(&this->TailIndex);
        const ULONG head = this->HeadIndex;
        const ULONG size = this->Size;

        if (((head + 1) % size) == tail) {
            return 0;
        }
        else if (head >= tail) {
            // copy from head to the end
            ULONG maxCount = (tail == 0) ? (size - 1) : size;
            ULONG count = min(maxCount - head, InputBufferSize);

            memcpy(&this->BufferPtr[head], &InputBufferPtr[0], count);
            bytesCopied = count;

            if (tail != 0) {
                // copy from beginning to tail
                count = min(tail - 1, InputBufferSize - bytesCopied);

                memcpy(&this->BufferPtr[0], &InputBufferPtr[bytesCopied], count);
                bytesCopied += count;
            }
        }
        else {
            // copy from head to tail
            ULONG count = min(tail - head - 1, InputBufferSize);

            memcpy(&this->BufferPtr[head], &InputBufferPtr[0], count);
            bytesCopied = count;
        }

        { // Update head index
            const ULONG newHeadIndex = (head + bytesCopied) % size;

            WriteULongRelease(&this->HeadIndex, newHeadIndex);
        }
    }
    return bytesCopied;
}

_Use_decl_annotations_
ULONG
IMX_LPUART_RING_BUFFER::DequeueBytes(
    UCHAR *OutputBufferPtr,
    ULONG OutputBufferSize
)
{
    const ULONG head = ReadULongAcquire(&this->HeadIndex);
    const ULONG tail = this->TailIndex;
    const ULONG size = this->Size;
    ULONG bytesCopied = 0;

    if (head != tail) {
        if (head > tail) {
            // copy from tail to head
            ULONG count = min(head - tail, OutputBufferSize);

            memcpy(&OutputBufferPtr[0], &this->BufferPtr[tail], count);

            bytesCopied = count;
        }
        else {
            // copy from tail to end
            ULONG count = min(size - tail, OutputBufferSize);

            memcpy(&OutputBufferPtr[0], &this->BufferPtr[tail], count);

            bytesCopied = count;

            // copy from beginning to head
            count = min(head, OutputBufferSize - bytesCopied);

            memcpy(&OutputBufferPtr[bytesCopied], &this->BufferPtr[0], count);
            bytesCopied += count;
        }

        // update tail pointer
        const ULONG newTailIndex = (tail + bytesCopied) % size;
        WriteULongRelease(&this->TailIndex, newTailIndex);
    }
    return bytesCopied;
}

/**
 * @brief The LPUART Interrupt Service Routine
 */
_Use_decl_annotations_
BOOLEAN
iMXLPUARTEvtInterruptISR(
    WDFINTERRUPT WdfInterrupt,
    ULONG /*MessageID*/
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    bool queueDpc = false;

    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = iMXLPUARTGetInterruptContext(WdfInterrupt);

    NT_ASSERT(interruptContextPtr != NULL);
    IMX_LPUART_REGISTERS *registersPtr = interruptContextPtr->RegistersPtr;
    const ULONG waitMask = interruptContextPtr->WaitMask;
    ULONG waitEvents = interruptContextPtr->WaitEvents;

    const ULONG stat = READ_REGISTER_NOFENCE_ULONG(&registersPtr->STAT);
    const ULONG masked = stat & interruptContextPtr->EnabledInterruptsMask;

    ULONG water = READ_REGISTER_NOFENCE_ULONG(&registersPtr->WATER);
    ULONG txwater = (water >> IMX_LPUART_WATER_TXWATER_OFF) & IMX_LPUART_WATER_WATER_MASK;
    ULONG txcount = (water >> IMX_LPUART_WATER_TXCNT_OFF) & IMX_LPUART_WATER_COUNT_MASK;
    ULONG rxcount = water >> IMX_LPUART_WATER_RXCNT_OFF;

    interruptContextPtr->Counter++;
    IMX_LPUART_LOG_TRACE("Interrupt #%u stat = %08X water = %08X txcount = %u rxcount = %u",
        interruptContextPtr->Counter, stat, water, txcount, rxcount);

        // "TxState = %!IMX_LPUART_STATE!, TxDrainState = %!IMX_LPUART_STATE!, TxPurgeState = %!IMX_LPUART_STATE!, "
    IMX_LPUART_LOG_TRACE(
        "(RxState = %!IMX_LPUART_STATE!, RxDmaState = %!IMX_LPUART_STATE!, "
        "TxState = %!IMX_LPUART_STATE!, TxDrainState = %!IMX_LPUART_STATE!, "
        "TxDmaState = %!IMX_LPUART_STATE!, TxDmaDrainState = %!IMX_LPUART_STATE!, WaitEvents = 0x%lx)",
        int(interruptContextPtr->RxState),
        int(interruptContextPtr->RxDmaState),
        int(interruptContextPtr->TxState),
        int(interruptContextPtr->TxDrainState),
        int(interruptContextPtr->TxDmaState),
        int(interruptContextPtr->TxDmaDrainState),
        interruptContextPtr->WaitEvents);

    if (masked == 0) {
        IMX_LPUART_LOG_WARNING("Not claiming interrupt.");

        return FALSE;
    }
    else {
        { // Pass errors to upper driver.
            if (stat & IMX_LPUART_STAT_LBKDIF) {
                IMX_LPUART_LOG_INFORMATION("SERIAL_ERROR_BREAK");
                interruptContextPtr->CommStatusErrors |= SERIAL_ERROR_BREAK;
                waitEvents |= (waitMask & SERIAL_EV_BREAK);
                interruptContextPtr->CTRLCopy &= ~IMX_LPUART_BAUD_LBKDIE;
            }
            if ((stat & IMX_LPUART_STAT_FE) != 0) {
                IMX_LPUART_LOG_INFORMATION("SERIAL_ERROR_FRAMING");
                interruptContextPtr->CommStatusErrors |= SERIAL_ERROR_FRAMING;
            }

            if ((stat & IMX_LPUART_STAT_PE) != 0) {
                IMX_LPUART_LOG_INFORMATION("SERIAL_ERROR_PARITY");
                interruptContextPtr->CommStatusErrors |= SERIAL_ERROR_PARITY;
            }
        }

        //
        // If DMA RX transfer is not active, service the RX buffer
        //
        if ((stat & IMX_LPUART_STAT_RDRF) || (rxcount > 0)) {
            IMX_LPUART_LOG_TRACE("RX count = %lu", rxcount);

            IMX_LPUART_RING_BUFFER *rxBufferPtr = &interruptContextPtr->RxBuffer;
            const ULONG tail = ReadULongAcquire(&rxBufferPtr->TailIndex);
            const ULONG oldHead = rxBufferPtr->HeadIndex;
            const ULONG size = rxBufferPtr->Size;

            ULONG head = oldHead;
            ULONG nextHead = (head + 1) % size;
            ULONG bytesRead = 0;

            if ((stat & IMX_LPUART_STAT_OR) != 0) {
                IMX_LPUART_LOG_INFORMATION("SERIAL_ERROR_OVERRUN");
                interruptContextPtr->CommStatusErrors |= SERIAL_ERROR_OVERRUN;
            }

            while (nextHead != tail) {
                const ULONG rxd = READ_REGISTER_NOFENCE_ULONG(&registersPtr->DATA);

                if ((rxd & IMX_LPUART_DATA_RXEMPT) > 0) {
                    IMX_LPUART_LOG_TRACE("Receive Buffer Empty");
                    break;
                }
                else {
                    rxBufferPtr->BufferPtr[head] = static_cast<UCHAR>(rxd);
                    ++bytesRead;
                    head = nextHead;
                    nextHead = (nextHead + 1) % size;
                }
            }

            if (bytesRead) {
                waitEvents |= (waitMask & SERIAL_EV_RXCHAR);
            }

            WriteULongRelease(&rxBufferPtr->HeadIndex, head);

            IMX_LPUART_LOG_INFORMATION("Read %lu bytes from FIFO.", bytesRead);

            //
            // If the intermediate buffer is full, disable the RRDY and AGTIM
            // interrupts so they do not continue asserting
            //
            if (nextHead == tail) {
                IMX_LPUART_LOG_INFORMATION("Intermediate receive buffer overflowed, disabling RRDY and AGTIM.");

                interruptContextPtr->CommStatusErrors |= SERIAL_ERROR_QUEUEOVERRUN;
                waitEvents |= (waitMask & SERIAL_EV_ERR);

                interruptContextPtr->CTRLCopy &= ~IMX_LPUART_CTRL_RIE;
                WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);
                interruptContextPtr->EnabledInterruptsMask &= ~(UINT32)IMX_LPUART_STAT_RDRF;

                Dump(CTRL, interruptContextPtr->CTRLCopy);
            }

            //
            // If there are bytes available and RX notifications are enabled,
            // queue the receive ready notification
            //
            if ((head != tail) && (interruptContextPtr->RxState == IMX_LPUART_STATE::WAITING_FOR_INTERRUPT)) {
                IMX_LPUART_LOG_INFORMATION("Bytes available in RX buffer, going to WAITING_FOR_DPC state.");

                interruptContextPtr->RxState = IMX_LPUART_STATE::WAITING_FOR_DPC;
                queueDpc = true;
            }
        }

        if ((masked & IMX_LPUART_STAT_TDRE) | (masked & IMX_LPUART_STAT_TC)) {
            IMX_LPUART_RING_BUFFER *txBufferPtr = &interruptContextPtr->TxBuffer;
            const ULONG head = ReadULongAcquire(&txBufferPtr->HeadIndex);
            ULONG tail = txBufferPtr->TailIndex;
            const ULONG size = txBufferPtr->Size;

            //
            // Take bytes from the intermediate buffer and put them in the TX FIFO.
            // Sending XOFF means stop transmission.
            //
            if (tail != head) {
                while ((tail != head) && (txcount < (interruptContextPtr->TX.FIFO.Size - 2))) {
                    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->DATA, txBufferPtr->BufferPtr[tail]);
                    ++txcount;
                    tail = (tail + 1) % size;
                }
                WriteULongRelease(&txBufferPtr->TailIndex, tail);
                IMX_LPUART_LOG_TRACE("txwater(%u), txcount(%u) ", txwater, txcount);
            }

            //
            // Handle watermark, mask ISR and queue DPC
            //
            {
                const ULONG count = IMX_LPUART_RING_BUFFER::Count(head, tail, size);

                if ((interruptContextPtr->TxState == IMX_LPUART_STATE::WAITING_FOR_INTERRUPT) // { // Possibly popped data from SW fifo, check it's threshold.
                    && (count <= interruptContextPtr->TxDpcThreshold)) {
                    IMX_LPUART_LOG_INFORMATION("TX FIFO is below threshold and notifications are enabled, firing ready notification. (count = %lu, interruptContextPtr->TxDpcThreshold = %lu)", count, interruptContextPtr->TxDpcThreshold);
                    Dump(CTRL, interruptContextPtr->CTRLCopy);
                    //
                    // If the TX ready notification is enabled and the intermediate buffer contains fewer bytes than the threshold, queue a DPC to request more bytes.
                    //
                    if (interruptContextPtr->TxDrainState == IMX_LPUART_STATE::WAITING_FOR_INTERRUPT) {
                        interruptContextPtr->TxDrainState = IMX_LPUART_STATE::WAITING_FOR_DPC;
                        queueDpc = true;
                    }
                }
                if (tail == head) {
                    IMX_LPUART_LOG_INFORMATION("TX intermediate buffer was emptied, lets trigger DPC if needed.");

                    interruptContextPtr->CTRLCopy &= ~(UINT32)IMX_LPUART_CTRL_TIE;
                    interruptContextPtr->EnabledInterruptsMask &= ~(UINT32)IMX_LPUART_STAT_TDRE;

                    if (masked & IMX_LPUART_STAT_TC) {
                        interruptContextPtr->EnabledInterruptsMask &= ~(UINT32)IMX_LPUART_STAT_TC;
                        interruptContextPtr->CTRLCopy &= ~(UINT32)IMX_LPUART_CTRL_TCIE;

                        NT_ASSERT(txcount == 0); // HW queue is empty - Improper TE=1 timming.
                        if (interruptContextPtr->TxState == IMX_LPUART_STATE::WAITING_FOR_INTERRUPT) {
                            interruptContextPtr->TxState = IMX_LPUART_STATE::WAITING_FOR_DPC;
                            queueDpc = true;
                        }
                        if (interruptContextPtr->TxDrainState == IMX_LPUART_STATE::WAITING_FOR_INTERRUPT) {
                            interruptContextPtr->TxDrainState = IMX_LPUART_STATE::WAITING_FOR_DPC;
                            queueDpc = true;
                        }
                        waitEvents |= (waitMask & SERIAL_EV_TXEMPTY);
                    }
                    else {
                        if (((interruptContextPtr->TxDrainState == IMX_LPUART_STATE::WAITING_FOR_INTERRUPT)
                                || (waitMask & SERIAL_EV_TXEMPTY)
                            )) {
                            interruptContextPtr->EnabledInterruptsMask |= IMX_LPUART_STAT_TC;
                            interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_TCIE;
                        }
                    }
                }
            }
            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);
        }

        interruptContextPtr->STATCopy = stat;
        WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->STAT, interruptContextPtr->STATCopy);

        interruptContextPtr->WaitEvents = waitEvents;
        if (waitEvents != 0) {
            queueDpc = true;
        }

        if (queueDpc) {
            WdfInterruptQueueDpcForIsr(WdfInterrupt);
        }
        _DataSynchronizationBarrier();
        return TRUE;
    }
}

/**
 * The LPUART Interrupt Deferred Procedure Call
 */
_Use_decl_annotations_
VOID
iMXLPUARTEvtInterruptDpc(
    WDFINTERRUPT WdfInterrupt,
    WDFOBJECT /*AssociatedObject*/
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = iMXLPUARTGetInterruptContext(WdfInterrupt);

    IMX_LPUART_LOG_TRACE(
        "DPC was fired. (RxState = %!IMX_LPUART_STATE!, RxDmaState = %!IMX_LPUART_STATE!, "
        "TxState = %!IMX_LPUART_STATE!, TxDrainState = %!IMX_LPUART_STATE!, TxPurgeState = %!IMX_LPUART_STATE!, "
        "TxDmaState = %!IMX_LPUART_STATE!, TxDmaDrainState = %!IMX_LPUART_STATE!, WaitEvents = 0x%lx)",
        int(interruptContextPtr->RxState),
        int(interruptContextPtr->RxDmaState),
        int(interruptContextPtr->TxState),
        int(interruptContextPtr->TxDrainState),
        int(interruptContextPtr->TxPurgeState),
        int(interruptContextPtr->TxDmaState),
        int(interruptContextPtr->TxDmaDrainState),
        interruptContextPtr->WaitEvents);

    if (interruptContextPtr->RxState == IMX_LPUART_STATE::WAITING_FOR_DPC) {
        interruptContextPtr->RxState = IMX_LPUART_STATE::COMPLETE;
        IMX_LPUART_LOG_INFORMATION("Calling SerCx2PioReceiveReady()");
        SerCx2PioReceiveReady(interruptContextPtr->SerCx2PioReceive);
    }

    //
    // If the TX buffer is below the threshold and TX notifications are
    // enabled, call SerCx2PioTransmitReady() to request more bytes
    //
    if (interruptContextPtr->TxState == IMX_LPUART_STATE::WAITING_FOR_DPC) {
        interruptContextPtr->TxState = IMX_LPUART_STATE::COMPLETE;
        IMX_LPUART_LOG_INFORMATION("Calling SerCx2PioTransmitReady()");
        SerCx2PioTransmitReady(interruptContextPtr->SerCx2PioTransmit);
    }

    if (interruptContextPtr->TxDrainState == IMX_LPUART_STATE::WAITING_FOR_DPC) {
        interruptContextPtr->TxDrainState = IMX_LPUART_STATE::COMPLETE;
        IMX_LPUART_LOG_INFORMATION("Notifying TX FIFO drain complete");
        SerCx2PioTransmitDrainFifoComplete(interruptContextPtr->SerCx2PioTransmit);
    }

    if (interruptContextPtr->TxDmaDrainState == IMX_LPUART_STATE::WAITING_FOR_DPC) {
        interruptContextPtr->TxDmaDrainState = IMX_LPUART_STATE::COMPLETE;
        IMX_LPUART_LOG_INFORMATION("TX DMA: TX FIFO drain complete");
        iMXLPUARTCompleteCustomTxTransactionRequest(interruptContextPtr->TxDmaTransactionContextPtr, STATUS_SUCCESS);
    }

    if (interruptContextPtr->WaitEvents != 0) {
        //
        // Interlocked exchange the wait events bitmask with 0
        //
        WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
        const ULONG waitEvents = interruptContextPtr->WaitEvents;

        interruptContextPtr->WaitEvents = 0;
        NT_ASSERT((waitEvents & interruptContextPtr->WaitMask) == waitEvents);
        WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

        IMX_LPUART_LOG_INFORMATION("Completing wait. (waitEvents = 0x%lx)", waitEvents);
        SerCx2CompleteWait(WdfInterruptGetDevice(WdfInterrupt), waitEvents);
    }
}

/**
 * This function is called from the SerCx2 framework on opening device
 */
_Use_decl_annotations_
NTSTATUS iMXLPUARTEvtSerCx2FileOpen(WDFDEVICE WdfDevice)
{
    NTSTATUS status;
    IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = iMXLPUARTGetDeviceContext(WdfDevice);
    NT_ASSERT(deviceContextPtr != nullptr);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = deviceContextPtr->InterruptContextPtr;

    IMX_LPUART_LOG_TRACE("%!FUNC!");
    NT_ASSERT(interruptContextPtr != nullptr);

    NT_ASSERT(interruptContextPtr->RxState == IMX_LPUART_STATE::STOPPED);
    NT_ASSERT(interruptContextPtr->RxDmaState == IMX_LPUART_STATE::STOPPED);
    NT_ASSERT(interruptContextPtr->TxState == IMX_LPUART_STATE::STOPPED);
    NT_ASSERT(interruptContextPtr->TxDrainState == IMX_LPUART_STATE::STOPPED);
    NT_ASSERT(interruptContextPtr->TxDmaState == IMX_LPUART_STATE::STOPPED);
    NT_ASSERT(interruptContextPtr->TxDmaDrainState == IMX_LPUART_STATE::STOPPED);
    NT_ASSERT(interruptContextPtr->CommStatusErrors == 0);
    NT_ASSERT(interruptContextPtr->WaitMask == 0);
    NT_ASSERT(interruptContextPtr->WaitEvents == 0);

    interruptContextPtr->CTRLCopy &= (~(UINT32)(IMX_LPUART_CTRL_TE | IMX_LPUART_CTRL_RE));

    if (deviceContextPtr->Initialized) {
        IMX_LPUART_LOG_TRACE("Restoring UART registers from saved state.");
        WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
        iMXLPUARTRestoreHardwareFromShadowedRegisters(interruptContextPtr);
        WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
    }
    else {
        SERIAL_LINE_CONTROL lineControl = { 0 };
        SERIAL_HANDFLOW handflow = { 0 };

        IMX_LPUART_LOG_INFORMATION("Configuring UART for the first time with reasonable defaults.");
        lineControl.StopBits = STOP_BIT_1;
        lineControl.Parity = NO_PARITY;
        lineControl.WordLength = 8;

        status = iMXLPUARTConfigureUart(WdfDevice, 115200, &lineControl, &handflow);

        if (!NT_SUCCESS(status)) {
            IMX_LPUART_LOG_ERROR("Failed to do initial configuration of UART. (status = %!STATUS!)", status);
            return status;
        }
    }
    return STATUS_SUCCESS;
}

/**
 * This function is called from the SerCx2 framework to close device
 */
_Use_decl_annotations_
VOID iMXLPUARTEvtSerCx2FileClose(WDFDEVICE WdfDevice)
{
    IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = iMXLPUARTGetDeviceContext(WdfDevice);
    NT_ASSERT(deviceContextPtr != NULL);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = deviceContextPtr->InterruptContextPtr;
    NT_ASSERT(interruptContextPtr != nullptr);

    IMX_LPUART_LOG_TRACE("%!FUNC!");

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
    iMXLPUARTResetHardwareAndPreserveShadowedRegisters(interruptContextPtr);
    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

    interruptContextPtr->EnabledInterruptsMask = 0;
    interruptContextPtr->CTRLCopy &= ~(UINT32)(IMX_LPUART_CTRL_ORIE | IMX_LPUART_CTRL_NEIE | IMX_LPUART_CTRL_FEIE | IMX_LPUART_CTRL_PEIE | IMX_LPUART_CTRL_TIE | IMX_LPUART_CTRL_TCIE | IMX_LPUART_CTRL_RIE | IMX_LPUART_CTRL_ILIE);

    IMX_LPUART_LOG_TRACE("%u interrupts", interruptContextPtr->Counter);
    interruptContextPtr->Counter = 0;

    interruptContextPtr->RxBuffer.Reset();
    interruptContextPtr->TxBuffer.Reset();

    interruptContextPtr->RxState = IMX_LPUART_STATE::STOPPED;
    interruptContextPtr->RxDmaState = IMX_LPUART_STATE::STOPPED;
    interruptContextPtr->TxState = IMX_LPUART_STATE::STOPPED;
    interruptContextPtr->TxDrainState = IMX_LPUART_STATE::STOPPED;
    interruptContextPtr->TxDmaState = IMX_LPUART_STATE::STOPPED;
    interruptContextPtr->TxDmaDrainState = IMX_LPUART_STATE::STOPPED;
    interruptContextPtr->CommStatusErrors = 0;
    interruptContextPtr->WaitMask = 0;
    interruptContextPtr->WaitEvents = 0;
}

_Use_decl_annotations_
VOID
iMXLPUARTEvtSerCx2SetWaitMask(
    WDFDEVICE WdfDevice,
    WDFREQUEST WdfRequest,
    ULONG WaitMask
)
{
    IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = iMXLPUARTGetDeviceContext(WdfDevice);
    const ULONG unsupportedWaitFlags =
        SERIAL_EV_RXFLAG |
        SERIAL_EV_RING |
        SERIAL_EV_DSR |
        SERIAL_EV_RLSD |
        SERIAL_EV_PERR |
        SERIAL_EV_RX80FULL |
        SERIAL_EV_EVENT1 |
        SERIAL_EV_EVENT2;

    IMX_LPUART_LOG_TRACE("%!FUNC! (, , WaitMask %08X)", WaitMask);

    if ((WaitMask & unsupportedWaitFlags) != 0) {
        IMX_LPUART_LOG_ERROR("Wait mask contains unsupported flags. (WaitMask = 0x%lx, unsupportedWaitFlags = 0x%lx)", WaitMask, unsupportedWaitFlags);

        //
        // Spec explicitly calls for STATUS_INVALID_PARAMETER
        //
        WdfRequestComplete(WdfRequest, STATUS_INVALID_PARAMETER);
        return;
    }

    iMXLPUARTSetWaitMask(deviceContextPtr, WaitMask);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

/**
 * This function is handling I/O control calls
 */
_Use_decl_annotations_
NTSTATUS
iMXLPUARTEvtSerCx2Control(
    WDFDEVICE WdfDevice,
    WDFREQUEST WdfRequest,
    size_t /*OutputBufferLength*/,
    size_t /*InputBufferLength*/,
    ULONG IoControlCode
)
{
    NTSTATUS status = STATUS_SUCCESS;
    IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = iMXLPUARTGetDeviceContext(WdfDevice);

    IMX_LPUART_LOG_TRACE("%!FUNC! (, IoControlCode %08X)", IoControlCode);

    switch (IoControlCode) {
    case IOCTL_SERIAL_SET_BAUD_RATE:
        iMXLPUARTIoctlSetBaudRate(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_GET_BAUD_RATE:
        iMXLPUARTIoctlGetBaudRate(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_SET_HANDFLOW:
        iMXLPUARTIoctlSetHandflow(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_GET_HANDFLOW:
        iMXLPUARTIoctlGetHandflow(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_SET_LINE_CONTROL:
        iMXLPUARTIoctlSetLineControl(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_GET_LINE_CONTROL:
        iMXLPUARTIoctlGetLineControl(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_GET_PROPERTIES:
        iMXLPUARTIoctlGetProperties(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_GET_COMMSTATUS:
        iMXLPUARTIoctlGetCommStatus(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_SET_BREAK_ON: __fallthrough;
    case IOCTL_SERIAL_SET_BREAK_OFF:
        iMXLPUARTIoctlSetBreakOnOff(deviceContextPtr, WdfRequest, IoControlCode == IOCTL_SERIAL_SET_BREAK_ON);
        break;
    case IOCTL_SERIAL_GET_MODEM_CONTROL:
        iMXLPUARTIoctlGetModemControl(deviceContextPtr, WdfRequest);
        break;
    case IOCTL_SERIAL_GET_MODEMSTATUS: __fallthrough;
    case IOCTL_SERIAL_SET_RTS: __fallthrough;
    case IOCTL_SERIAL_CLR_RTS: __fallthrough;
    case IOCTL_SERIAL_GET_DTRRTS: __fallthrough;
    case IOCTL_SERIAL_SET_DTR: __fallthrough;
    case IOCTL_SERIAL_CLR_DTR:
        status = STATUS_NOT_SUPPORTED;
        IMX_LPUART_LOG_ERROR("%!FUNC! (, IoControlCode %08X) NOT_SUPPORTED return 0x%x", IoControlCode, status);
        WdfRequestComplete(WdfRequest, status);
        break;
    case IOCTL_SERIAL_RESET_DEVICE: __fallthrough;
    case IOCTL_SERIAL_SET_QUEUE_SIZE: __fallthrough;
    case IOCTL_SERIAL_SET_XOFF: __fallthrough;
    case IOCTL_SERIAL_SET_XON: __fallthrough;
    case IOCTL_SERIAL_GET_WAIT_MASK: __fallthrough;
    case IOCTL_SERIAL_SET_WAIT_MASK: __fallthrough;
    case IOCTL_SERIAL_WAIT_ON_MASK: __fallthrough;
    case IOCTL_SERIAL_PURGE: __fallthrough;
    case IOCTL_SERIAL_GET_CHARS: __fallthrough;
    case IOCTL_SERIAL_SET_CHARS: __fallthrough;
    case IOCTL_SERIAL_XOFF_COUNTER: __fallthrough;
    case IOCTL_SERIAL_CONFIG_SIZE: __fallthrough;
    case IOCTL_SERIAL_GET_STATS: __fallthrough;
    case IOCTL_SERIAL_CLEAR_STATS: __fallthrough;
    case IOCTL_SERIAL_IMMEDIATE_CHAR: __fallthrough;
    case IOCTL_SERIAL_SET_TIMEOUTS: __fallthrough;
    case IOCTL_SERIAL_GET_TIMEOUTS: __fallthrough;
    case IOCTL_SERIAL_APPLY_DEFAULT_CONFIGURATION:
        NT_ASSERT(!"Should have been handled by SerCx2!");
        __fallthrough;

    case IOCTL_SERIAL_GET_COMMCONFIG: __fallthrough;
    case IOCTL_SERIAL_SET_COMMCONFIG: __fallthrough;
    case IOCTL_SERIAL_SET_MODEM_CONTROL: __fallthrough;
    case IOCTL_SERIAL_SET_FIFO_CONTROL: __fallthrough;
    default:
        IMX_LPUART_LOG_ERROR("%!FUNC! (, IoControlCode %08X) NOT_SUPPORTED return 0x%x", IoControlCode, status);
        WdfRequestComplete(WdfRequest, status);
    }
    return status;
}

/**
 * This function is enabling the receive interrupts from device
 */
static VOID
iMXLPUARTEnableRxInterrupt(
    _Inout_ IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC!");

    InterruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_RIE | IMX_LPUART_CTRL_ORIE | IMX_LPUART_CTRL_RE;
    InterruptContextPtr->EnabledInterruptsMask |= IMX_LPUART_STAT_RDRF | IMX_LPUART_STAT_OR;
    WRITE_REGISTER_NOFENCE_ULONG(&InterruptContextPtr->RegistersPtr->CTRL, InterruptContextPtr->CTRLCopy);

    Dump(CTRL, InterruptContextPtr->CTRLCopy);
}

/**
 * This function is reading characters from the input buffer
 */
_Use_decl_annotations_
ULONG
iMXLPUARTEvtSerCx2PioReceiveReadBuffer(
    SERCX2PIORECEIVE PioReceive,
    UCHAR * BufferPtr,
    ULONG Length
)
{
    ULONG bytesRead;
    IMX_LPUART_RX_CONTEXT *rxContextPtr = iMXLPUARTGetRxContext(PioReceive);
    NT_ASSERT(rxContextPtr != NULL);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = rxContextPtr->InterruptContextPtr;
    NT_ASSERT(interruptContextPtr != NULL);

    IMX_LPUART_LOG_TRACE("%!FUNC! (, BufferPtr %p, Length %u)", BufferPtr, Length);
    interruptContextPtr->RxState = IMX_LPUART_STATE::IDLE;

    if (!(interruptContextPtr->CTRLCopy & IMX_LPUART_CTRL_TE)) { // Guard this for XOFF.
        interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_TE;
        WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, interruptContextPtr->CTRLCopy);
    }
    iMXLPUARTEnableRxInterrupt(interruptContextPtr);

    //
    // Read bytes from intermediate buffer into SerCx2's buffer
    //
    bytesRead = interruptContextPtr->RxBuffer.DequeueBytes(BufferPtr, Length);
    IMX_LPUART_LOG_TRACE("Read bytes from intermediate buffer into receive buffer. (Length = %lu, bytesRead = %lu)", Length, bytesRead);

    return bytesRead;
}

/**
 * This function is enabling notification on character reception
 */
_Use_decl_annotations_
VOID
iMXLPUARTEvtSerCx2PioReceiveEnableReadyNotification(
    SERCX2PIORECEIVE PioReceive
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC!");

    IMX_LPUART_RX_CONTEXT *rxContextPtr = iMXLPUARTGetRxContext(PioReceive);
    NT_ASSERT(rxContextPtr != NULL);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = rxContextPtr->InterruptContextPtr;

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
    if (interruptContextPtr->RxState == IMX_LPUART_STATE::CANCELLED) {
        IMX_LPUART_LOG_TRACE("RX notification was already cancelled.");
        interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_ORIE;
        interruptContextPtr->EnabledInterruptsMask |= IMX_LPUART_STAT_OR;
        WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, interruptContextPtr->CTRLCopy);
        WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
    }
    else {
        NT_ASSERT(interruptContextPtr->RxState == IMX_LPUART_STATE::IDLE);
        {
            IMX_LPUART_RING_BUFFER* rxBufferPtr = &interruptContextPtr->RxBuffer;
            const ULONG tail = ReadULongAcquire(&rxBufferPtr->TailIndex);
            const ULONG head = rxBufferPtr->HeadIndex;

            //
            // See if data can be returned so we do not hang if no ISR comes. Not enabling ISR could also solve he condition.
            //
            if (tail != head) {
                WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
                SerCx2PioReceiveReady(interruptContextPtr->SerCx2PioReceive);
            }
            else {
                interruptContextPtr->RxState = IMX_LPUART_STATE::WAITING_FOR_INTERRUPT;
                WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
                IMX_LPUART_LOG_TRACE("Enabled RRDY and AGTIM, going to WAITING_FOR_INTERRUPT");
            }
        }
    }
}

/**
 * This function is canceling notification on character reception
 */
_Use_decl_annotations_
BOOLEAN
iMXLPUARTEvtSerCx2PioReceiveCancelReadyNotification(
    SERCX2PIORECEIVE PioReceive
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC!");

    BOOLEAN cancelled;
    IMX_LPUART_RX_CONTEXT *rxContextPtr = iMXLPUARTGetRxContext(PioReceive);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = rxContextPtr->InterruptContextPtr;

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);

    const IMX_LPUART_STATE rxState = interruptContextPtr->RxState;

    switch (rxState) {
    case IMX_LPUART_STATE::STOPPED:
    case IMX_LPUART_STATE::CANCELLED:
        NT_ASSERT(!"Unexpected state");
        __fallthrough;
    case IMX_LPUART_STATE::IDLE:
    case IMX_LPUART_STATE::WAITING_FOR_INTERRUPT:
        interruptContextPtr->RxState = IMX_LPUART_STATE::CANCELLED;
        cancelled = TRUE;
        break;
    case IMX_LPUART_STATE::WAITING_FOR_DPC:
    case IMX_LPUART_STATE::COMPLETE:
    default:
        cancelled = FALSE;
        break;
    }

    // Disable ORIE
    interruptContextPtr->CTRLCopy &= ~(UINT32)IMX_LPUART_CTRL_ORIE;
    interruptContextPtr->EnabledInterruptsMask &= ~(UINT32)IMX_LPUART_STAT_OR;
    WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, interruptContextPtr->CTRLCopy);
    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

    IMX_LPUART_LOG_TRACE("Processed receive cancel request. (rxState = %d, cancelled = %d)", int(rxState), cancelled);

    return cancelled;
}

/**
 * This function is writing the characters to output buffer for transmit
 */
_Use_decl_annotations_
ULONG
iMXLPUARTEvtSerCx2PioTransmitWriteBuffer(
    SERCX2PIOTRANSMIT PioTransmit,
    PUCHAR Buffer,
    ULONG Length
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! (, Buffer %p, Length %u)", Buffer, Length);

    IMX_LPUART_TX_CONTEXT *txContextPtr = iMXLPUARTGetTxContext(PioTransmit);
    NT_ASSERT(txContextPtr != NULL);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = txContextPtr->InterruptContextPtr;
    IMX_LPUART_REGISTERS *registersPtr = interruptContextPtr->RegistersPtr;
    dmp_regs(registersPtr);

    interruptContextPtr->TxState = IMX_LPUART_STATE::IDLE;
    interruptContextPtr->TxDrainState = IMX_LPUART_STATE::IDLE;

    ULONG fifoBytesWritten = 0;

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);

    if (interruptContextPtr->TxBuffer.IsEmpty()) {
        //
        // Write directly to TX FIFO
        //
        const UINT32 stat = READ_REGISTER_NOFENCE_ULONG(&registersPtr->STAT);
        ULONG water = READ_REGISTER_NOFENCE_ULONG(&registersPtr->WATER);
        ULONG txcount = (water >> IMX_LPUART_WATER_TXCNT_OFF) & IMX_LPUART_WATER_COUNT_MASK;

        if ((stat & IMX_LPUART_STAT_TDRE) != 0) {
            IMX_LPUART_LOG_TRACE("Transmit data buffer empty");
        }

        while ((fifoBytesWritten < Length) && (txcount < interruptContextPtr->TX.FIFO.Size)) {
            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->DATA, Buffer[fifoBytesWritten]);
            ++txcount;
            ++fifoBytesWritten;
        }
        water = READ_REGISTER_NOFENCE_ULONG(&registersPtr->WATER);
        txcount = (water >> IMX_LPUART_WATER_TXCNT_OFF) & IMX_LPUART_WATER_COUNT_MASK;
    }

    //
    // Queue remaining bytes to the intermediate buffer
    //
    ULONG bytesEnqueued = interruptContextPtr->TxBuffer.EnqueueBytes(Buffer + fifoBytesWritten, Length - fifoBytesWritten);

    //
    // If we put bytes in the intermediate TX buffer, enable TX interrupts
    // so the ISR will drain the intermediate buffer
    //
    if (bytesEnqueued != 0) {
        interruptContextPtr->EnabledInterruptsMask |= IMX_LPUART_STAT_TDRE;
        interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_TIE;
        WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->WATER, interruptContextPtr->WATERCopy);
        WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);

        Dump(CTRL, interruptContextPtr->CTRLCopy);
    }
    if (!(interruptContextPtr->CTRLCopy & IMX_LPUART_CTRL_TE)) {
        interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_TE;
        WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);
    }
    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

    IMX_LPUART_LOG_TRACE("Serviced write buffer request. (Length = %lu, fifoBytesWritten = %lu, bytesEnqueued = %lu)", Length, fifoBytesWritten, bytesEnqueued);
    dmp_regs(registersPtr);

    return fifoBytesWritten + bytesEnqueued;
}

_Use_decl_annotations_
VOID
iMXLPUARTEvtSerCx2PioTransmitEnableReadyNotification(
    SERCX2PIOTRANSMIT PioTransmit
)
{
    IMX_LPUART_TX_CONTEXT *txContextPtr = iMXLPUARTGetTxContext(PioTransmit);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = txContextPtr->InterruptContextPtr;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
    if (interruptContextPtr->TxState == IMX_LPUART_STATE::CANCELLED) {
        IMX_LPUART_LOG_TRACE("Transmit ready notification was already cancelled");
    }
    else {
        interruptContextPtr->TxState = IMX_LPUART_STATE::WAITING_FOR_INTERRUPT;
        IMX_LPUART_LOG_TRACE("Enabled TRDY interrupt - going to WAITING_FOR_INTERRUPT");
    }
    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
}

_Use_decl_annotations_
BOOLEAN
iMXLPUARTEvtSerCx2PioTransmitCancelReadyNotification(
    SERCX2PIOTRANSMIT PioTransmit
)
{
    BOOLEAN cancelled;
    IMX_LPUART_TX_CONTEXT *txContextPtr = iMXLPUARTGetTxContext(PioTransmit);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = txContextPtr->InterruptContextPtr;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
    {
        const IMX_LPUART_STATE txState = interruptContextPtr->TxState;

        switch (txState) {
        case IMX_LPUART_STATE::STOPPED:
        case IMX_LPUART_STATE::CANCELLED:
            NT_ASSERT(!"Unexpected state");
            __fallthrough;

        case IMX_LPUART_STATE::IDLE:
        case IMX_LPUART_STATE::WAITING_FOR_INTERRUPT:
            interruptContextPtr->TxState = IMX_LPUART_STATE::CANCELLED;
            cancelled = TRUE;
            break;

        case IMX_LPUART_STATE::WAITING_FOR_DPC:
        case IMX_LPUART_STATE::COMPLETE:
        default:
            cancelled = FALSE;
            break;
        }
        IMX_LPUART_LOG_TRACE("Processed transmit cancel request. (txState = %d, cancelled = %d)", int(txState), cancelled);
    }
    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

    return cancelled;
}

_Use_decl_annotations_
VOID
iMXLPUARTEvtSerCx2PioTransmitDrainFifo(
    SERCX2PIOTRANSMIT PioTransmit
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    IMX_LPUART_TX_CONTEXT *txContextPtr = iMXLPUARTGetTxContext(PioTransmit);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = txContextPtr->InterruptContextPtr;

    IMX_LPUART_REGISTERS *registersPtr = interruptContextPtr->RegistersPtr;
    bool locked = true;

    //
    // Clear drain complete flag and enable TX FIFO empty interrupt
    //
    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
    if (interruptContextPtr->TxDrainState == IMX_LPUART_STATE::CANCELLED) {
        IMX_LPUART_LOG_TRACE("Drain FIFO request was already cancelled");
        WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
        locked = false;
    }
    else {
        //
        // Can we complete the drain FIFO request synchronously?
        //
        IMX_LPUART_RING_BUFFER *txBufferPtr = &interruptContextPtr->TxBuffer;
        const ULONG head = ReadULongAcquire(&txBufferPtr->HeadIndex);
        ULONG tail = txBufferPtr->TailIndex;

        interruptContextPtr->TxDrainState = IMX_LPUART_STATE::WAITING_FOR_INTERRUPT; // Create else case for the following
#if DBG
        ULONG ctrl = READ_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL);
        ASSERT(0 < (ctrl & IMX_LPUART_CTRL_TE));
#endif // DBG
        if (head == tail) { // SW Fifo is empty

            //
            // If the TX hardware FIFO is already empty, complete the request inline.
            //
            ULONG stat = READ_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->STAT);
            if ((stat & IMX_LPUART_STAT_TC) > 0) {
                IMX_LPUART_LOG_TRACE("TX FIFO is empty, completing DrainFifo request inline.");

                interruptContextPtr->TxDrainState = IMX_LPUART_STATE::COMPLETE;
                WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
                locked = false;
                SerCx2PioTransmitDrainFifoComplete(PioTransmit);
            }
            else { // Interrupt on empty fifo, watermark = 0
                interruptContextPtr->EnabledInterruptsMask = (interruptContextPtr->EnabledInterruptsMask & ~(UINT32)IMX_LPUART_STAT_TDRE) | IMX_LPUART_STAT_TC;
                interruptContextPtr->CTRLCopy = (interruptContextPtr->CTRLCopy & ~(IMX_LPUART_CTRL_TIE)) | IMX_LPUART_CTRL_TCIE;
                WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);
            }
        }
        else {
            //
            // Enable the TX FIFO Empty Interrupt
            //
            interruptContextPtr->EnabledInterruptsMask |= IMX_LPUART_STAT_TDRE;
            interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_TIE;
            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);

            Dump(CTRL, interruptContextPtr->CTRLCopy);

            IMX_LPUART_LOG_TRACE("Enabled TX FIFO empty interrupt to notify of drain completion");
        }
    }
    if (locked) {
        WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
    }
}

_Use_decl_annotations_
BOOLEAN
iMXLPUARTEvtSerCx2PioTransmitCancelDrainFifo(
    SERCX2PIOTRANSMIT PioTransmit
)
{
    BOOLEAN cancelled;
    IMX_LPUART_TX_CONTEXT *txContextPtr = iMXLPUARTGetTxContext(PioTransmit);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = txContextPtr->InterruptContextPtr;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
    {
        const IMX_LPUART_STATE drainState = interruptContextPtr->TxDrainState;

        switch (drainState) {
        case IMX_LPUART_STATE::STOPPED:
        case IMX_LPUART_STATE::CANCELLED:
            NT_ASSERT(!"Unexpected state");
            __fallthrough;
        case IMX_LPUART_STATE::IDLE:
        case IMX_LPUART_STATE::WAITING_FOR_INTERRUPT:

            //
            // Let the ISR disable the TXFE interrupt, since we don't know who
            // else might be relying on the TXFE interrupt.
            //
            interruptContextPtr->TxDrainState = IMX_LPUART_STATE::CANCELLED;
            cancelled = TRUE;
            break;

        case IMX_LPUART_STATE::WAITING_FOR_DPC:
        case IMX_LPUART_STATE::COMPLETE:
        default:
            cancelled = FALSE;
            break;
        }
        IMX_LPUART_LOG_TRACE("Processed drain cancellation request. (drainState = %d, cancelled = %d)", int(drainState), cancelled);
    }
    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

    return cancelled;
}

_Use_decl_annotations_
VOID
iMXLPUARTEvtSerCx2PioTransmitPurgeFifo(
    SERCX2PIOTRANSMIT PioTransmit,
    ULONG BytesAlreadyTransmittedToHardware
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    IMX_LPUART_TX_CONTEXT *txContextPtr = iMXLPUARTGetTxContext(PioTransmit);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = txContextPtr->InterruptContextPtr;
    IMX_LPUART_REGISTERS *registersPtr = interruptContextPtr->RegistersPtr;

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
    {
        //
        // Empty the TX intermediate buffer
        //
        ULONG discardedByteCount = interruptContextPtr->TxBuffer.Reset();
        NT_ASSERT(discardedByteCount <= BytesAlreadyTransmittedToHardware);
        UNREFERENCED_PARAMETER(BytesAlreadyTransmittedToHardware);

        //
        // Disable the TX interrupt
        //
        interruptContextPtr->EnabledInterruptsMask = ~(UINT32)IMX_LPUART_STAT_TDRE;
        interruptContextPtr->CTRLCopy &= ~(IMX_LPUART_CTRL_TIE | IMX_LPUART_CTRL_TCIE);
        WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);
        // Flush FIFO
        WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->FIFO, interruptContextPtr->FIFOCopy | IMX_LPUART_FIFO_TXFLUSH); // TBD test flush!

        Dump(CTRL, interruptContextPtr->CTRLCopy);

        //
        // If the TX hardware FIFO is already empty, complete the request inline.
        //
#if DBG
        ULONG stat = READ_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->STAT);

        IMX_LPUART_LOG_TRACE("TX FIFO flushed, completing PurgeFifo request inline. (BytesAlreadyTransmittedToHardware = %lu, discardedByteCount = %lu, stat = 0x%x)", BytesAlreadyTransmittedToHardware, discardedByteCount, stat);
#endif
        WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
        SerCx2PioTransmitPurgeFifoComplete(PioTransmit, discardedByteCount);
    }
}

_Use_decl_annotations_
VOID
iMXLPUARTCompleteCustomTxTransactionRequest(
    IMX_LPUART_TX_DMA_TRANSACTION_CONTEXT * TxDmaTransactionContextPtr,
    NTSTATUS RequestStatus
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    WdfSpinLockAcquire(TxDmaTransactionContextPtr->Lock);

    WDFREQUEST wdfRequest = TxDmaTransactionContextPtr->WdfRequest;
    if (wdfRequest == WDF_NO_HANDLE) {
        WdfSpinLockRelease(TxDmaTransactionContextPtr->Lock);
    }
    else {
        BOOLEAN isDone;
        NTSTATUS dmaStatus;
        WDFDMATRANSACTION wdfDmaTransaction = TxDmaTransactionContextPtr->WdfDmaTransaction;

        if (!NT_SUCCESS(RequestStatus) || TxDmaTransactionContextPtr->IsDeferredCancellation) {
            isDone = WdfDmaTransactionDmaCompletedFinal(wdfDmaTransaction, TxDmaTransactionContextPtr->BytesTransferred, &dmaStatus);
            NT_ASSERT(isDone);
        }
        else {
            isDone = WdfDmaTransactionDmaCompleted(wdfDmaTransaction, &dmaStatus);
            TxDmaTransactionContextPtr->BytesTransferred = WdfDmaTransactionGetBytesTransferred(wdfDmaTransaction);
        }

        if (!isDone) {
            TxDmaTransactionContextPtr->WdfRequest = WDF_NO_HANDLE;
            WdfDmaTransactionRelease(wdfDmaTransaction);

            //
            // If request was canceled and few bytes were sent,
            // fix the status so caller gets number of bytes sent.
            //
            if ((RequestStatus == STATUS_REQUEST_CANCELED) && (TxDmaTransactionContextPtr->BytesTransferred != 0)) {
                RequestStatus = STATUS_SUCCESS;
            }

            {
                ULONG_PTR requestInfo = TxDmaTransactionContextPtr->BytesTransferred;

                if (TxDmaTransactionContextPtr->IsDeferredCancellation) {
                    WdfSpinLockRelease(TxDmaTransactionContextPtr->Lock);
                    WdfRequestCompleteWithInformation(wdfRequest, RequestStatus, requestInfo);
                }
                else {
                    NTSTATUS cancelStatus = WdfRequestUnmarkCancelable(wdfRequest);

                    if (NT_SUCCESS(cancelStatus) || (cancelStatus == STATUS_CANCELLED)) {
                        WdfSpinLockRelease(TxDmaTransactionContextPtr->Lock);
                        WdfRequestCompleteWithInformation(wdfRequest, RequestStatus, requestInfo);
                    }
                }
            }
        }
    }
    WdfSpinLockRelease(TxDmaTransactionContextPtr->Lock);
}

_Use_decl_annotations_
VOID
iMXLPUARTEvtSerCx2PurgeFifos(
    WDFDEVICE WdfDevice,
    BOOLEAN PurgeRxFifo,
    BOOLEAN PurgeTxFifo
)
{
    IMX_LPUART_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = iMXLPUARTGetDeviceContext(WdfDevice);
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = deviceContextPtr->InterruptContextPtr;
    NT_ASSERT(interruptContextPtr != nullptr);

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
    {
        if (PurgeRxFifo != FALSE) {
            interruptContextPtr->RxBuffer.Reset();
        }

        if (PurgeTxFifo != FALSE) {
            iMXLPUARTResetHardwareAndPreserveShadowedRegisters(interruptContextPtr);

            iMXLPUARTRestoreHardwareFromShadowedRegisters(interruptContextPtr);

            interruptContextPtr->TxBuffer.Reset();
        }
    }
    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

    IMX_LPUART_LOG_TRACE("Purged FIFOs. (PurgeRxFifo = %d, PurgeTxFifo = %d)", PurgeRxFifo, PurgeTxFifo);
}

_Use_decl_annotations_
NTSTATUS iMXLPUARTReadDeviceProperties(
    WDFDEVICE WdfDevice,
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr
)
{
    ACPI_EVAL_OUTPUT_BUFFER UNALIGNED *dsdBufferPtr = nullptr;
    UINT32 rtsEnabled = 0;
    NTSTATUS status = STATUS_SUCCESS;

    IMX_LPUART_ASSERT_MAX_IRQL(APC_LEVEL);
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    status = AcpiQueryDsd(WdfDeviceWdmGetPhysicalDevice(WdfDevice), &dsdBufferPtr);
    if (NT_SUCCESS(status)) {
        const ACPI_METHOD_ARGUMENT UNALIGNED *devicePropertiesPkgPtr;

        status = AcpiParseDsdAsDeviceProperties(dsdBufferPtr, &devicePropertiesPkgPtr);
        if (!NT_SUCCESS(status)) {
            if (dsdBufferPtr != nullptr) {
                ExFreePoolWithTag(dsdBufferPtr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
            }
        }
        else {
            NTSTATUS tmpStatus = AcpiDevicePropertiesQueryIntegerValue(devicePropertiesPkgPtr, "rts-cts-enabled", &rtsEnabled);
            if (!NT_SUCCESS(tmpStatus)) {
                IMX_LPUART_LOG_WARNING("rts-cts-enabled is not present in DSDT. (status = %!STATUS!)", status);
            }
            else {
                DeviceContextPtr->RtsCtsLinesEnabled = (rtsEnabled != 0);
            }

            if (dsdBufferPtr != nullptr) {
                ExFreePoolWithTag(dsdBufferPtr, ACPI_TAG_EVAL_OUTPUT_BUFFER);
            }
        }
    }
    return status;
}

_Use_decl_annotations_
NTSTATUS
iMXLPUARTEvtSerCx2ApplyConfig(
    WDFDEVICE WdfDevice,
    PVOID ConnectionParametersPtr
)
{
    IMX_LPUART_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    ULONG baudRate;
    SERIAL_LINE_CONTROL lineControl;
    SERIAL_HANDFLOW handflow;
    bool rtsCtsEnabled;
    NTSTATUS status;

    status = iMXLPUARTParseConnectionDescriptor(static_cast<const RH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER*>(ConnectionParametersPtr),
                                                &baudRate, &lineControl, &handflow, &rtsCtsEnabled);

    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to parse connection descriptor. (status = %!STATUS!)", status);
    }
    else {
        status = iMXLPUARTConfigureUart(WdfDevice, baudRate, &lineControl, &handflow); // , rtsCtsEnabled);
    }
    return status;
}

_Use_decl_annotations_
NTSTATUS
iMXLPUARTConfigureUart(
    WDFDEVICE WdfDevice,
    ULONG BaudRate,
    const SERIAL_LINE_CONTROL *LineControl,
    const SERIAL_HANDFLOW *Handflow
)
{
    NTSTATUS status = STATUS_SUCCESS;
    IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = iMXLPUARTGetDeviceContext(WdfDevice);

    IMX_LPUART_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    IMX_LPUART_LOG_TRACE("%!FUNC! (, BaudRate %lu, RtsCtsEnabled %s)", BaudRate, deviceContextPtr->RtsCtsLinesEnabled ? "true" : "false");

    status = iMXLPUARTCheckValidStateForConfigIoctl(deviceContextPtr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Configuration cannot be applied while IO is active. (status = %!STATUS!)", status);
    }
    else {
        IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = deviceContextPtr->InterruptContextPtr;

        if (deviceContextPtr->Initialized) {
            WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);
            status = iMXLPUARTResetHardwareAndClearShadowedRegisters(interruptContextPtr);
            WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

            if (!NT_SUCCESS(status)) {
                IMX_LPUART_LOG_ERROR("Failed to reset hardware. (status = %!STATUS!)", status);
            }
        }
        if (NT_SUCCESS(status)) {
            status = iMXLPUARTSetBaudRate(deviceContextPtr, BaudRate);
        }
        if (NT_SUCCESS(status)) {
            status = iMXLPUARTSetLineControl(deviceContextPtr, LineControl);
        }
        if (NT_SUCCESS(status)) {
            status = iMXLPUARTSetHandflow(deviceContextPtr, Handflow);
        }

        //
        // Enable the UART
        //
        {
            WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);

            interruptContextPtr->EnabledInterruptsMask = 0;
            iMXLPUARTEnableRxInterrupt(interruptContextPtr);
            interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_TE | IMX_LPUART_CTRL_RE;
            WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, interruptContextPtr->CTRLCopy);

            Dump(CTRL, interruptContextPtr->CTRLCopy);

            WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
        }

        deviceContextPtr->Loop = false;
        deviceContextPtr->Initialized = true;
    }

    return status;
}

//
// UART configuration changes (Baud, Handflow, etc.) are only allowed before
// the first read or write IRP is sent. Changing line configuration while
// data transmission is in progress could have unpredictable effects.
//
NTSTATUS
iMXLPUARTCheckValidStateForConfigIoctl(
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr
)
{
    NTSTATUS status = STATUS_SUCCESS;
    const IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    switch (interruptContextPtr->RxState) {
    case IMX_LPUART_STATE::WAITING_FOR_INTERRUPT:
    case IMX_LPUART_STATE::WAITING_FOR_DPC:
        status = STATUS_INVALID_DEVICE_STATE;
        IMX_LPUART_LOG_INFORMATION("Invalid RxState %!IMX_LPUART_STATE!", interruptContextPtr->RxState);
    }

    switch (interruptContextPtr->RxDmaState) {
    case IMX_LPUART_STATE::WAITING_FOR_DPC:
        status = STATUS_INVALID_DEVICE_STATE;
        IMX_LPUART_LOG_INFORMATION("Invalid RxDmaState %!IMX_LPUART_STATE!", interruptContextPtr->RxDmaState);
    }

    switch (interruptContextPtr->TxState) {
    case IMX_LPUART_STATE::WAITING_FOR_INTERRUPT:
    case IMX_LPUART_STATE::WAITING_FOR_DPC:
        status = STATUS_INVALID_DEVICE_STATE;
        IMX_LPUART_LOG_INFORMATION("Invalid TxState %!IMX_LPUART_STATE!", interruptContextPtr->TxState);
    }

    switch (interruptContextPtr->TxDrainState) {
    case IMX_LPUART_STATE::WAITING_FOR_INTERRUPT:
    case IMX_LPUART_STATE::WAITING_FOR_DPC:
        status = STATUS_INVALID_DEVICE_STATE;
        IMX_LPUART_LOG_INFORMATION("Invalid TxDrainState %!IMX_LPUART_STATE!", interruptContextPtr->TxDrainState);
    }

    switch (interruptContextPtr->TxDmaState) {
    case IMX_LPUART_STATE::WAITING_FOR_DPC:
        status = STATUS_INVALID_DEVICE_STATE;
        IMX_LPUART_LOG_INFORMATION("Invalid TxDmaState %!IMX_LPUART_STATE!", interruptContextPtr->TxDmaState);
    }

    switch (interruptContextPtr->TxDmaDrainState) {
    case IMX_LPUART_STATE::WAITING_FOR_INTERRUPT:
    case IMX_LPUART_STATE::WAITING_FOR_DPC:
        status = STATUS_INVALID_DEVICE_STATE;
        IMX_LPUART_LOG_INFORMATION("Invalid TxDmaDrainState %!IMX_LPUART_STATE!", interruptContextPtr->TxDmaDrainState);
    }
    return status;
}

NTSTATUS
iMXLPUARTResetHardwareAndPreserveShadowedRegisters(
    const IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
)
{
    IMX_LPUART_REGISTERS *registersPtr = InterruptContextPtr->RegistersPtr;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    //
    // Disable UART and reset control registers
    //
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, 0);
    KeStallExecutionProcessor(30);
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->GLOBAL, IMX_LPUART_GLOBAL_RST);
    KeStallExecutionProcessor(30);
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->GLOBAL, 0);
    KeStallExecutionProcessor(30);

    return STATUS_SUCCESS;
}

NTSTATUS
iMXLPUARTResetHardwareAndClearShadowedRegisters(
    IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
)
{
    NTSTATUS status = iMXLPUARTResetHardwareAndPreserveShadowedRegisters(InterruptContextPtr);

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    InterruptContextPtr->BAUDCopy = 0;
    InterruptContextPtr->STATCopy = 0;
    InterruptContextPtr->CTRLCopy = 0;
    InterruptContextPtr->DATACopy = 0;
    InterruptContextPtr->MATCHCopy = 0;
    InterruptContextPtr->MODIRCopy = 0;
    InterruptContextPtr->FIFOCopy = 0;
    InterruptContextPtr->WATERCopy = 0;

    return status;
}

void
iMXLPUARTRestoreHardwareFromShadowedRegisters(
    const IMX_LPUART_INTERRUPT_CONTEXT *InterruptContextPtr
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    IMX_LPUART_REGISTERS *registersPtr = InterruptContextPtr->RegistersPtr;

    //
    // Precondition: UART should be disabled
    //
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->BAUD, InterruptContextPtr->BAUDCopy);
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->STAT, InterruptContextPtr->STATCopy);
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, InterruptContextPtr->CTRLCopy);
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->MATCH, InterruptContextPtr->MATCHCopy);
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->MODIR, InterruptContextPtr->MODIRCopy);
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->FIFO, InterruptContextPtr->FIFOCopy);
    WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->WATER, InterruptContextPtr->WATERCopy);
}

_Use_decl_annotations_
void
iMXLPUARTComputeFifoThresholds(
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    ULONG BaudRate,
    ULONG BitsPerFrame,
    ULONG *TxFifoThresholdPtr,
    ULONG *RxFifoThresholdPtr
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    //
    // Compute the minimum number of characters that take just longer to
    // transmit than the time threshold.
    //
    ULONG txThreshold = iMXLPUARTComputeCharactersPerDuration(DeviceContextPtr->Parameters.TxFifoThresholdUs, BaudRate, BitsPerFrame);

    //
    // Clamp in range [2, IMX_LPUART_FIFO_COUNT]
    //
    if (txThreshold < 2) {
        txThreshold = 2;
    }
    else if (txThreshold > IMX_LPUART_FIFO_COUNT) {
        txThreshold = IMX_LPUART_FIFO_COUNT;
    }

    ULONG rxThreshold = iMXLPUARTComputeCharactersPerDuration(DeviceContextPtr->Parameters.RxFifoThresholdUs, BaudRate, BitsPerFrame);

    //
    // Clamp in range [1, IMX_LPUART_FIFO_COUNT - 1], which will cause
    // IMX_LPUART_FIFO_COUNT - rxThreshold to be in the same range.
    //
    if (rxThreshold < 1) {
        rxThreshold = 1;
    }
    else if (rxThreshold > (IMX_LPUART_FIFO_COUNT - 1)) {
        rxThreshold = IMX_LPUART_FIFO_COUNT - 1;
    }

    *TxFifoThresholdPtr = txThreshold;
    *RxFifoThresholdPtr = IMX_LPUART_FIFO_COUNT - rxThreshold;
}

ULONG
iMXLPUARTComputeCharactersPerDuration(
    ULONG DurationUs,
    ULONG BaudRate,
    ULONG BitsPerFrame
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    //
    // Compute the FIFO threshold so there's at least this much time between
    // when the interrupt is asserted and when the FIFO goes empty (or full).
    //
    const ULONG minimumTime = ULONG(DurationUs * 10000000ULL / 1000000ULL);
    const ULONG timePerFrame = BitsPerFrame * 10000000UL / BaudRate;

    return (minimumTime + (timePerFrame - 1)) / timePerFrame;
}

NTSTATUS
iMXLPUARTComputeDividers(
    ULONG LPUARTModuleClock,
    ULONG DesiredBaudRate,
    _Out_ ULONG *OSRPtr,
    _Out_ ULONG *SBRPtr
)
{
    NTSTATUS status;
    ULONG sbr, osr, baudDifference;
    ULONG temporaryOsr, temporarySbr, temporaryDifference, temporary;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");
    // The idea is to use the best OSR (oversampling rate) possible.
    // Note, OSR is typically hard-set to 16 in other UART instantiations.
    // Loop to find the best OSR value possible, one that generates minimum
    // Baud difference.
    // iterate through the rest of the supported values of OSR.

    // Calculation Formula:
    // Baud rate = Baud clock / ((OSR + 1) * SBR)
    baudDifference = DesiredBaudRate;
    osr = 0;
    sbr = 0;

    for (temporaryOsr = 4; temporaryOsr <= 32; temporaryOsr++) {
        // calculate the temporary SBR value
        temporarySbr = (LPUARTModuleClock / (DesiredBaudRate * temporaryOsr));
        if (temporarySbr == 0) {
            temporarySbr = 1;
        }

        // calculate the baud rate difference based on the temporary
        // OSR and SBR values
        temporaryDifference = LPUARTModuleClock / (temporaryOsr * temporarySbr) - DesiredBaudRate;

        // select best values between SBR and SBR + 1
        temporary = LPUARTModuleClock / (temporaryOsr * (temporarySbr + 1));
        if (temporaryDifference > (DesiredBaudRate - temporary)) {
            temporaryDifference = DesiredBaudRate - temporary;
            temporarySbr++;
        }

        if (temporarySbr > IMX_LPUART_BAUD_SBR_MASK) {
            continue;
        }
        if (temporaryDifference <= baudDifference) {
            baudDifference = temporaryDifference;
            osr = temporaryOsr;
            sbr = temporarySbr;

            if (baudDifference == 0) {
                break;
            }
        }
    }

    // Handle Baud rate outside acceptable rate
    if (baudDifference > ((DesiredBaudRate / 100) * 3)) {
        IMX_LPUART_LOG_ERROR("Unacceptable baud rate difference of more than 3%%");

        status = STATUS_NOT_SUPPORTED;
    }
    else {
        *OSRPtr = osr;
        *SBRPtr = sbr;
        status = STATUS_SUCCESS;
    }

    return status;
}

_Use_decl_annotations_
NTSTATUS
iMXLPUARTSetBaudRate(
    IMX_LPUART_DEVICE_CONTEXT * DeviceContextPtr,
    ULONG BaudRate
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    IMX_LPUART_REGISTERS *registersPtr = DeviceContextPtr->RegistersPtr;
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;

    ULONG osr; // Oversampling Ratio
    ULONG sbr; // Baud Rate Modulo Divisor
    NTSTATUS status = iMXLPUARTComputeDividers(DeviceContextPtr->Parameters.ModuleClockFrequency, BaudRate, &osr, &sbr);

    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("iMXLPUARTComputeDividers(...) failed. (status = %!STATUS!)", status);
    }
    else {
        //
        // Baud rate could get set before line control settings, so just assume
        // the typical case of 10 bits per frame.
        //

        const ULONG bitsPerFrame8N1 = 10;
        ULONG txThreshold;
        ULONG rxThreshold;
        ULONG txDpcThreshold = iMXLPUARTComputeCharactersPerDuration(DeviceContextPtr->Parameters.TxDpcThresholdUs, BaudRate, bitsPerFrame8N1);
        ULONG FIFO = READ_REGISTER_NOFENCE_ULONG(&registersPtr->FIFO);
        ULONG baud = READ_REGISTER_NOFENCE_ULONG(&registersPtr->BAUD);

        IMX_LPUART_LOG_TRACE("OSR = %u SBR = %u", osr, sbr);
        iMXLPUARTComputeFifoThresholds(DeviceContextPtr, BaudRate, bitsPerFrame8N1, &txThreshold, &rxThreshold);
        NT_ASSERT((txThreshold >= 2) && (txThreshold <= IMX_LPUART_FIFO_COUNT));
        NT_ASSERT((rxThreshold >= 1) && (rxThreshold <= (IMX_LPUART_FIFO_COUNT - 1)));

        txDpcThreshold = min(DeviceContextPtr->InterruptContextPtr->TxBuffer.Size - 1, txDpcThreshold + 1);

        interruptContextPtr->RX.FIFO.Size = IMX_LPUART_FIFO_DEPTH((FIFO >> IMX_LPUART_FIFO_RXSIZE_OFF) & IMX_LPUART_FIFO_SIZE_MASK);
        interruptContextPtr->RX.FIFO.Enable = (FIFO >> 3) & 0b1;
        interruptContextPtr->TX.FIFO.Size = IMX_LPUART_FIFO_DEPTH((FIFO >> IMX_LPUART_FIFO_TXSIZE_OFF) & IMX_LPUART_FIFO_SIZE_MASK);
        interruptContextPtr->TX.FIFO.Enable = (FIFO >> 7) & 0b1;

        IMX_LPUART_LOG_TRACE("FIFO %08X TX %d RX %d", FIFO, interruptContextPtr->TX.FIFO.Size, interruptContextPtr->RX.FIFO.Size);

        // Set BAUDCopy
        if ((osr > 3) && (osr < 8)) {
            baud |= IMX_LPUART_BAUD_BOTHEDGE;
        }
        baud &= ~(IMX_LPUART_BAUD_OSR_MASK << IMX_LPUART_BAUD_OSR_SHIFT);
        baud |= ((osr - 1) & IMX_LPUART_BAUD_OSR_MASK) << IMX_LPUART_BAUD_OSR_SHIFT;
        baud &= ~IMX_LPUART_BAUD_SBR_MASK;
        baud |= sbr & IMX_LPUART_BAUD_SBR_MASK;
        // if (!use_rx_dma)
        baud &= ~IMX_LPUART_BAUD_RDMAE;
        // if (!use_tx_dma)
        baud &= ~IMX_LPUART_BAUD_TDMAE;
        IMX_LPUART_LOG_TRACE("baud = %08X", baud);

        //
        // Update registers under the interrupt spinlock
        //
        WdfInterruptAcquireLock(DeviceContextPtr->WdfInterrupt);
        {
            interruptContextPtr->BAUDCopy = baud;
            interruptContextPtr->TxDpcThreshold = txDpcThreshold;
            interruptContextPtr->FIFOCopy = IMX_LPUART_FIFO_TXFE | IMX_LPUART_FIFO_RXFE | IMX_LPUART_FIFO_RXIDEN_32CHARS;
            interruptContextPtr->WATERCopy = (40 << IMX_LPUART_WATER_RXWATER_OFF) | (7 << IMX_LPUART_WATER_TXWATER_OFF);

            // Disable the transmitter and receiver
            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, 0);
            // Set the Baud rate
            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->BAUD, interruptContextPtr->BAUDCopy);
            // Enable FIFO
            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->FIFO, interruptContextPtr->FIFOCopy | IMX_LPUART_FIFO_TXFLUSH | IMX_LPUART_FIFO_RXFLUSH);
            // Set the watermarks
            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->WATER, interruptContextPtr->WATERCopy);
            // Enable the transmitter and receiver
            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);

            DeviceContextPtr->CurrentBaudRate = BaudRate;
        }
        WdfInterruptReleaseLock(DeviceContextPtr->WdfInterrupt);

        IMX_LPUART_LOG_TRACE("Successfully set baud rate. (BaudRate = %lu, SBR = %lu,  rxThreshold = %lu, txThreshold = %lu, txDpcThreshold = %lu)", BaudRate, sbr, rxThreshold, txThreshold, txDpcThreshold);
    }
    return status;
}

_Use_decl_annotations_
NTSTATUS
iMXLPUARTSetLineControl(
    IMX_LPUART_DEVICE_CONTEXT * DeviceContextPtr,
    const SERIAL_LINE_CONTROL * LineControlPtr
)
{
    NTSTATUS status = STATUS_SUCCESS;
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;
    ULONG baudSetMask = 0;
    ULONG baudClearMask = 0;
    ULONG ctrlSetMask = 0;
    ULONG ctrlClearMask = 0;
    ULONG wordLen = 0;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");
    NT_ASSERT(interruptContextPtr != NULL);
    switch (LineControlPtr->StopBits) {
    case STOP_BIT_1:
        baudClearMask |= IMX_LPUART_BAUD_SBNS;
        break;
    case STOP_BITS_2:
        baudSetMask |= IMX_LPUART_BAUD_SBNS;
        break;
    case STOP_BITS_1_5:
        KdPrint(("STOP_BITS_1_5 is not supported."));
        status = STATUS_INVALID_PARAMETER;
        break;
    default:
        status = STATUS_INVALID_PARAMETER;
    }

    if (NT_SUCCESS(status)) {
        switch (LineControlPtr->Parity) {
        case NO_PARITY:
            ctrlClearMask |= IMX_LPUART_CTRL_PE;
            break;
        case ODD_PARITY:
            ctrlSetMask |= IMX_LPUART_CTRL_PE; // Enable
            ctrlSetMask |= IMX_LPUART_CTRL_PT; // Odd
            wordLen += 1;
            break;
        case EVEN_PARITY:
            ctrlSetMask |= IMX_LPUART_CTRL_PE; // Enable
            ctrlClearMask |= IMX_LPUART_CTRL_PT; // Even
            wordLen += 1;
            break;
        case MARK_PARITY: __fallthrough;
        case SPACE_PARITY:
            IMX_LPUART_LOG_WARNING("MARK_PARITY and SPACE_PARITY are not supported. (LineControlPtr->Parity = %d)", LineControlPtr->Parity);
            status = STATUS_INVALID_PARAMETER;
            break;
        default:
            status = STATUS_INVALID_PARAMETER;
        }
    }
    if (NT_SUCCESS(status)) {
        switch (LineControlPtr->WordLength) {
        case 7:
            wordLen += LineControlPtr->WordLength;
            break;
        case 8:
            wordLen += LineControlPtr->WordLength;
            break;
        default:
            IMX_LPUART_LOG_WARNING("Unsupported word length: %d", LineControlPtr->WordLength);
            status = STATUS_INVALID_PARAMETER;
        }
    }
    if (NT_SUCCESS(status)) {
        switch (wordLen) {
        case 7:
            // Clear
            ctrlClearMask |= IMX_LPUART_CTRL_M;
            baudClearMask |= IMX_LPUART_BAUD_M10;
            // Set
            ctrlSetMask |= IMX_LPUART_CTRL_M7;
            break;
        case 8:
            // Clear
            ctrlClearMask |= IMX_LPUART_CTRL_M7 | IMX_LPUART_CTRL_M ;
            baudClearMask |= IMX_LPUART_BAUD_M10;
            break;
        case 9:
            // Clear
            ctrlClearMask |= IMX_LPUART_CTRL_M7;
            baudClearMask |= IMX_LPUART_BAUD_M10;
            // Set
            ctrlSetMask |= IMX_LPUART_CTRL_M;
            break;
        default:
            IMX_LPUART_LOG_WARNING("Word length + parity: %d is not supported.", wordLen);
            status = STATUS_INVALID_PARAMETER;
        }
    }

    //
    // Set mask and clear mask should not overlap
    //
    NT_ASSERT((!NT_SUCCESS(status)) || (((ctrlClearMask & ctrlSetMask) == 0) && ((baudClearMask & baudSetMask) == 0)));

    if (NT_SUCCESS(status)) {
        IMX_LPUART_LOG_INFORMATION("Updating line control. (ctrlSetMask = 0x%lx, ctrlClearMask = 0x%lx, baudSetMask = 0x%lx, baudClearMask = 0x%lx, StopBits = %d, Parity = %d, WordLength = %d)", ctrlSetMask, ctrlClearMask, baudClearMask, baudSetMask, LineControlPtr->StopBits, LineControlPtr->Parity, LineControlPtr->WordLength);
        //
        // Update the control register under the interrupt spinlock
        //
        {
            WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);

            // Disable the transmitter and receiver
            WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, 0);

            interruptContextPtr->BAUDCopy &= ~baudClearMask;
            interruptContextPtr->BAUDCopy |= baudSetMask;
            WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->BAUD, interruptContextPtr->BAUDCopy);

            interruptContextPtr->CTRLCopy &= ~ctrlClearMask;
            interruptContextPtr->CTRLCopy |= ctrlSetMask;
            WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, interruptContextPtr->CTRLCopy);
            // Enable the transmitter and receiver
            WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, interruptContextPtr->CTRLCopy);

            Dump(CTRL, interruptContextPtr->CTRLCopy);

            WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
        }
    }

    return status;
}

_Use_decl_annotations_
NTSTATUS
iMXLPUARTSetHandflow(
    IMX_LPUART_DEVICE_CONTEXT * DeviceContextPtr,
    const SERIAL_HANDFLOW * LineControlPtr
)
{
    NTSTATUS status = STATUS_SUCCESS;
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;
    UNREFERENCED_PARAMETER(interruptContextPtr);
    ULONG modirSetMask = 0;
    ULONG modirClearMask = 0;

    NT_ASSERT(interruptContextPtr != NULL);
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    if (NT_SUCCESS(status)) {
        switch (LineControlPtr->ControlHandShake & SERIAL_OUT_HANDSHAKEMASK) { // OUT as output data control
        case 0:
            // Ignore CTS (an input pin).
            modirClearMask |= IMX_LPUART_MODIR_TXCTSE;
            break;

        case SERIAL_CTS_HANDSHAKE:
            // Pay attention to CTS.
            if (!DeviceContextPtr->RtsCtsLinesEnabled) {
                IMX_LPUART_LOG_ERROR("RTS/CTS flow control is not allowed on this UART instance since RTS/CTS lines were not specified in the SerialLinesEnabled mask in the UartSerialBus() connection descriptor.");
                status = STATUS_INVALID_DEVICE_STATE;
            }
            else {
                modirSetMask |= IMX_LPUART_MODIR_TXCTSE;
            }
            break;

        case SERIAL_DSR_HANDSHAKE: __fallthrough;
        case SERIAL_DCD_HANDSHAKE:
        default:
            IMX_LPUART_LOG_ERROR("Unsupported flow control setting. (LineControlPtr->ControlHandShake = 0x%lx)", LineControlPtr->ControlHandShake);
            status = STATUS_NOT_SUPPORTED;
        }
    }

    if (NT_SUCCESS(status)) {
        //
        // Handle RTS flow control. The receiver on this UART can optionally
        // set the RTS pin - an output on this UART - depending on FIFO level.
        // Or the user can control it manually with IOCTLs.
        //
        switch (LineControlPtr->FlowReplace & SERIAL_RTS_MASK) {
        case 0:
            modirClearMask |= IMX_LPUART_MODIR_RXRTSE;
            break;

        case SERIAL_RTS_HANDSHAKE:
            if (!DeviceContextPtr->RtsCtsLinesEnabled) {
                IMX_LPUART_LOG_WARNING("SERIAL_RTS_HANDSHAKE was specified, but RTS/CTS were not specified in the SerialLinesEnabled mask in the UartSerialBus() connection descriptor, so RTS/CTS may not be pinned out on this UART.");
                status = STATUS_NOT_SUPPORTED;
            }
            else {
                // RTS signal generation and RTS ocupancy in handshake mode according to doc
                //
                // Line buff ocupancy
                //      _________________________________
                //      |       |       |       |       |
                //      0       1/4     2/4     3/4     full
                //
                //  RTS /----->--------->-------\____>____ When filling RX buffer
                //                      |<----->|          Hysteresis.
                //  RTS /----<-----<----\_____<______<____ When emptyiing RX buffer

                // LPUAR has no hysteresis configuration. Using 2/3 limit.
                modirSetMask |= IMX_LPUART_MODIR_RTSWATER((interruptContextPtr->RX.FIFO.Size / 3) * 2);
                modirSetMask |= IMX_LPUART_MODIR_RXRTSE;
            }
            break;

        case SERIAL_TRANSMIT_TOGGLE:
            modirClearMask |= IMX_LPUART_MODIR_RTSWATER_MASK;
            if (!DeviceContextPtr->RtsCtsLinesEnabled) {
                IMX_LPUART_LOG_WARNING("SERIAL_TRANSMIT_TOGGLE was specified, but RTS/CTS were not specified in the SerialLinesEnabled mask in the UartSerialBus() connection descriptor, so RTS/CTS may not be pinned out on this UART.");
                status = STATUS_NOT_SUPPORTED;
            }
            else {
                modirSetMask |= IMX_LPUART_MODIR_RXRTSE;
            }
            break;

        case SERIAL_RTS_CONTROL: __fallthrough; // This would require routing RTS CTS as GPIO.
        default:
            IMX_LPUART_LOG_ERROR("Unsupported RTS flow control value. (LineControlPtr->FlowReplace = 0x%lx)", LineControlPtr->FlowReplace);
            modirClearMask |= IMX_LPUART_MODIR_RXRTSE;
            status = STATUS_NOT_SUPPORTED;
        }
    }

    if ((LineControlPtr->FlowReplace & ~(SERIAL_RTS_MASK)) != 0) {
        IMX_LPUART_LOG_ERROR("Unsupported FlowReplace flag. (LineControlPtr->FlowReplace = 0x%lx)", LineControlPtr->FlowReplace);
        status = STATUS_NOT_SUPPORTED;
    }

    if (NT_SUCCESS(status)) {
        //
        // Update the control register under the interrupt spinlock
        //
        WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);

        // Disable the transmitter and receiver
        WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, 0);

        interruptContextPtr->MODIRCopy &= ~modirClearMask;
        interruptContextPtr->MODIRCopy |= modirSetMask;
        WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->MODIR, interruptContextPtr->MODIRCopy);

        DeviceContextPtr->CurrentFlowReplace = LineControlPtr->FlowReplace;
        // Enable the transmitter and receiver
        WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, interruptContextPtr->CTRLCopy);
        WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
    }
    return status;
}


_Use_decl_annotations_
void
iMXLPUARTSetBreakOnOff(
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    bool BreakOn
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! (BreakOn = %d)", BreakOn);

    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);

    if (BreakOn) {
        interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_SBK;
    }
    else {
        interruptContextPtr->CTRLCopy &= ~IMX_LPUART_CTRL_SBK;
    }

    WRITE_REGISTER_NOFENCE_ULONG(&interruptContextPtr->RegistersPtr->CTRL, interruptContextPtr->CTRLCopy);

    Dump(CTRL, interruptContextPtr->CTRLCopy);

    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);
}


_Use_decl_annotations_
void
iMXLPUARTSetWaitMask(
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    ULONG WaitMask
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;
    IMX_LPUART_REGISTERS *registersPtr = interruptContextPtr->RegistersPtr;
    //
    // Mask of bits that changed from the old wait mask to the new wait mask
    //
    const ULONG waitMaskDifference = interruptContextPtr->WaitMask ^ WaitMask;

    WdfInterruptAcquireLock(interruptContextPtr->WdfInterrupt);

    //
    // Nothing to do here for SERIAL_EV_ERR - the ISR handles everything
    //
    if ((waitMaskDifference & SERIAL_EV_RXCHAR) != 0) {
        if ((WaitMask & SERIAL_EV_RXCHAR) != 0) {
            iMXLPUARTEnableRxInterrupt(interruptContextPtr);
        }
    }

    if ((waitMaskDifference & SERIAL_EV_BREAK) != 0) {
        if ((WaitMask & SERIAL_EV_BREAK) != 0) {
            interruptContextPtr->CTRLCopy |= IMX_LPUART_BAUD_LBKDIE;
            interruptContextPtr->EnabledInterruptsMask |= IMX_LPUART_STAT_LBKDIF;
        }
        WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);

        Dump(CTRL, interruptContextPtr->CTRLCopy);
    }

    if ((waitMaskDifference & SERIAL_EV_TXEMPTY) != 0) {
        if ((WaitMask & SERIAL_EV_TXEMPTY) != 0) {
            if (0 == (interruptContextPtr->CTRLCopy & IMX_LPUART_CTRL_TE)) {
                interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_TE;
            }

            interruptContextPtr->EnabledInterruptsMask |= IMX_LPUART_STAT_TC;
            interruptContextPtr->CTRLCopy |= IMX_LPUART_CTRL_TCIE;

            WRITE_REGISTER_NOFENCE_ULONG(&registersPtr->CTRL, interruptContextPtr->CTRLCopy);
            Dump(CTRL, interruptContextPtr->CTRLCopy);
        }
    }

    interruptContextPtr->WaitMask = WaitMask;
    interruptContextPtr->WaitEvents = 0;
    WdfInterruptReleaseLock(interruptContextPtr->WdfInterrupt);

    IMX_LPUART_LOG_TRACE("Successfully set wait mask. (WaitMask = 0x%lx)", WaitMask);
}

_Use_decl_annotations_
VOID
iMXLPUARTIoctlSetBaudRate(
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    NTSTATUS status = iMXLPUARTCheckValidStateForConfigIoctl(DeviceContextPtr);
    SERIAL_BAUD_RATE *inputBufferPtr;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_WARNING("Behavior is undefined if baud is set while IO is active. (status = %!STATUS!)", status);
    }
    status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), reinterpret_cast<PVOID*>(&inputBufferPtr), nullptr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retreive input buffer for IOCTL_SERIAL_SET_BAUD_RATE request. (status = %!STATUS!)", status);
    }
    if (NT_SUCCESS(status)) {
        status = iMXLPUARTSetBaudRate(DeviceContextPtr, inputBufferPtr->BaudRate);
    }
    WdfRequestComplete(WdfRequest, status);
}

_Use_decl_annotations_
VOID
iMXLPUARTIoctlGetBaudRate(
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    SERIAL_BAUD_RATE *outputBufferPtr;
    NTSTATUS status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), reinterpret_cast<PVOID*>(&outputBufferPtr), nullptr);

    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retrieve output buffer for IOCTL_SERIAL_GET_BAUD_RATE request. (status = %!STATUS!)", status);
    }
    else {
        outputBufferPtr->BaudRate = DeviceContextPtr->CurrentBaudRate;
    }
    WdfRequestCompleteWithInformation(WdfRequest, status, sizeof(*outputBufferPtr));
}

_Use_decl_annotations_
void
iMXLPUARTIoctlSetLineControl(
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    NTSTATUS status = iMXLPUARTCheckValidStateForConfigIoctl(DeviceContextPtr);
    SERIAL_LINE_CONTROL *inputBufferPtr;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_WARNING("Behavior is undefined if line control is modified while IO is active. (status = %!STATUS!)", status);
    }
    status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), reinterpret_cast<PVOID*>(&inputBufferPtr), nullptr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retrieve input buffer for IOCTL_SERIAL_SET_LINE_CONTROL request. (status = %!STATUS!)", status);
    }

    if (NT_SUCCESS(status)) {
        status = iMXLPUARTSetLineControl(DeviceContextPtr, inputBufferPtr);
    }
    WdfRequestComplete(WdfRequest, status);
}

_Use_decl_annotations_
void
iMXLPUARTIoctlGetLineControl(
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    UNREFERENCED_PARAMETER(DeviceContextPtr);

    SERIAL_LINE_CONTROL *outputBufferPtr;
    NTSTATUS status;

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");
    status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), reinterpret_cast<PVOID*>(&outputBufferPtr), nullptr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retrieve output buffer for IOCTL_SERIAL_GET_LINE_CONTROL request. (status = %!STATUS!)", status);
    }

    if (NT_SUCCESS(status)) {
        const ULONG ctrlCopy = DeviceContextPtr->InterruptContextPtr->CTRLCopy;
        //const ULONG statCopy = DeviceContextPtr->InterruptContextPtr->STATCopy;
        const ULONG baudCopy = DeviceContextPtr->InterruptContextPtr->BAUDCopy;
        UCHAR parityBits = 0;

        //
        // Stop bits
        //
        if (0 < (baudCopy & IMX_LPUART_BAUD_SBNS)) {
            outputBufferPtr->StopBits = STOP_BITS_2;
        }
        else {
            outputBufferPtr->StopBits = STOP_BIT_1;
        }

        //
        // Parity
        //
        if (0 == (ctrlCopy & IMX_LPUART_CTRL_PE)) {
            outputBufferPtr->Parity = NO_PARITY;
            parityBits = 0;
        }
        else {
            if (0 < (ctrlCopy & IMX_LPUART_CTRL_PT)) {
                outputBufferPtr->Parity = ODD_PARITY;
                parityBits = 1;
            }
            else {
                outputBufferPtr->Parity = EVEN_PARITY;
                parityBits = 1;
            }
        }

        //
        // Word len
        //
        {
            const bool len7 = 0 < (ctrlCopy & IMX_LPUART_CTRL_M7);
            const bool len9 = 0 < (ctrlCopy & IMX_LPUART_CTRL_M);
            UCHAR wordLen = 0;

            if (len7) {
                wordLen = 9;
            }
            else {
                if (len9) {
                    wordLen = 7;
                }
                else {
                    wordLen = 8;
                }
            }
            outputBufferPtr->WordLength = wordLen - parityBits;
        }
        WdfRequestSetInformation(WdfRequest, sizeof(*outputBufferPtr));
    }

    WdfRequestComplete(WdfRequest, status);
}

_Use_decl_annotations_
void
iMXLPUARTIoctlSetHandflow(
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    NTSTATUS status;
    SERIAL_HANDFLOW *inputBufferPtr;

    status = iMXLPUARTCheckValidStateForConfigIoctl(DeviceContextPtr);
    if (!NT_SUCCESS(status)) {
        //
        // This is a warning because the MITT tests set handflow after
        // performing IO.
        //
        IMX_LPUART_LOG_WARNING("Behavior is undefined if line control is set after IO has been started. (status = %!STATUS!)", status);
    }

    status = WdfRequestRetrieveInputBuffer(WdfRequest, sizeof(*inputBufferPtr), reinterpret_cast<PVOID*>(&inputBufferPtr), nullptr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retrieve input buffer for IOCTL_SERIAL_SET_HANDFLOW request. (status = %!STATUS!)", status);
    }
    if (NT_SUCCESS(status)) {
        status = iMXLPUARTSetHandflow(DeviceContextPtr, inputBufferPtr);
    }
    WdfRequestComplete(WdfRequest, status);
}

_Use_decl_annotations_
void
iMXLPUARTIoctlGetHandflow(
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    SERIAL_HANDFLOW *outputBufferPtr;
    NTSTATUS status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), reinterpret_cast<PVOID*>(&outputBufferPtr), nullptr);

    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retrieve output buffer for IOCTL_SERIAL_GET_HANDFLOW request. (status = %!STATUS!)", status);
    }
    else {
        IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;
        UINT32 modirCopy = interruptContextPtr->MODIRCopy;

        *outputBufferPtr = SERIAL_HANDFLOW();

        if (modirCopy & IMX_LPUART_MODIR_RXRTSE) {
            if (7 < ((modirCopy & IMX_LPUART_MODIR_RTSWATER_MASK) >> 8u)) {
                outputBufferPtr->FlowReplace |= SERIAL_RTS_HANDSHAKE;
            }
            else {
                outputBufferPtr->FlowReplace |= SERIAL_TRANSMIT_TOGGLE;
            }
        }
        if (modirCopy & IMX_LPUART_MODIR_TXCTSE) {
            outputBufferPtr->ControlHandShake |= SERIAL_CTS_HANDSHAKE;
        }
    }
    WdfRequestCompleteWithInformation(WdfRequest, STATUS_SUCCESS, sizeof(*outputBufferPtr));
}

_Use_decl_annotations_
void
iMXLPUARTIoctlGetProperties(
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    const IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;
    SERIAL_COMMPROP *outputBufferPtr;
    NTSTATUS status;

    status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), reinterpret_cast<PVOID*>(&outputBufferPtr), nullptr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retrieve output buffer for IOCTL_SERIAL_GET_PROPERTIES request. (status = %!STATUS!)", status);
    }
    else {

        *outputBufferPtr = SERIAL_COMMPROP();

        outputBufferPtr->PacketLength = sizeof(*outputBufferPtr);
        outputBufferPtr->PacketVersion = 2;
        outputBufferPtr->ServiceMask = SERIAL_SP_SERIALCOMM;
        outputBufferPtr->MaxTxQueue = interruptContextPtr->TxBuffer.Capacity();
        outputBufferPtr->MaxRxQueue = interruptContextPtr->RxBuffer.Capacity();
        outputBufferPtr->MaxBaud = DeviceContextPtr->Parameters.ModuleClockFrequency / 16;

        outputBufferPtr->ProvSubType = SERIAL_SP_RS232;
        outputBufferPtr->ProvCapabilities = 0;
        if (DeviceContextPtr->RtsCtsLinesEnabled) {
            outputBufferPtr->ProvCapabilities |= SERIAL_PCF_RTSCTS;
        }
        outputBufferPtr->ProvCapabilities |= SERIAL_PCF_PARITY_CHECK;
            // FIXME - Timeouts are possibly supported by SerCx2. Check if this settings matters.
            // SERIAL_PCF_TOTALTIMEOUTS |
            // SERIAL_PCF_INTTIMEOUTS |

        outputBufferPtr->SettableParams =
            SERIAL_SP_PARITY |
            SERIAL_SP_BAUD |
            SERIAL_SP_DATABITS |
            SERIAL_SP_STOPBITS |
            SERIAL_SP_PARITY_CHECK;

        if (DeviceContextPtr->RtsCtsLinesEnabled) {
            outputBufferPtr->SettableParams |= SERIAL_SP_HANDSHAKING;
        }
        outputBufferPtr->SettableBaud = SERIAL_BAUD_USER;
        outputBufferPtr->SettableData = SERIAL_DATABITS_8;
        outputBufferPtr->SettableData |= SERIAL_DATABITS_7 | SERIAL_DATABITS_8;
        outputBufferPtr->SettableStopParity =
            SERIAL_STOPBITS_10 |
            SERIAL_STOPBITS_20 |
            SERIAL_PARITY_ODD |
            SERIAL_PARITY_EVEN |
            SERIAL_PARITY_NONE
            ;

        outputBufferPtr->CurrentTxQueue = 0;
        outputBufferPtr->CurrentRxQueue = 0;
        WdfRequestSetInformation(WdfRequest, sizeof(*outputBufferPtr));
    }
    WdfRequestComplete(WdfRequest, status);
}

_Use_decl_annotations_
void
iMXLPUARTIoctlGetCommStatus(
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    NTSTATUS status;
    SERIAL_STATUS *outputBufferPtr;
    IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;
    NT_ASSERT(interruptContextPtr != NULL);
    IMX_LPUART_REGISTERS *registersPtr = interruptContextPtr->RegistersPtr;

    status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), reinterpret_cast<PVOID*>(&outputBufferPtr), nullptr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retrieve output buffer for IOCTL_SERIAL_GET_COMMSTATUS request. (status = %!STATUS!)", status);
    }

    if (NT_SUCCESS(status)) {
        *outputBufferPtr = SERIAL_STATUS();

        //
        // Read and reset error mask
        //
        outputBufferPtr->Errors =  InterlockedExchange(reinterpret_cast<long*>(&interruptContextPtr->CommStatusErrors), 0);

        //
        // If output CTS flow control is on, check the status of the
        //
        if (0 < (interruptContextPtr->CTRLCopy & IMX_LPUART_MODIR_TXCTSE)) {
            const ULONG stat = READ_REGISTER_NOFENCE_ULONG(&registersPtr->STAT);

            if ((stat & IMX_LPUART_STAT_TC) == 0) {
                outputBufferPtr->HoldReasons |= SERIAL_TX_WAITING_FOR_CTS;
            }
        }

        //
        // Check if break is currently in progress
        //
        if ((interruptContextPtr->CTRLCopy & IMX_LPUART_CTRL_SBK) != 0) {
            outputBufferPtr->HoldReasons |= SERIAL_TX_WAITING_ON_BREAK;
        }

        outputBufferPtr->AmountInInQueue = interruptContextPtr->RxBuffer.Count();
        outputBufferPtr->AmountInOutQueue = interruptContextPtr->TxBuffer.Count();
        outputBufferPtr->EofReceived = FALSE;
        outputBufferPtr->WaitForImmediate = FALSE;

        WdfRequestSetInformation(WdfRequest, sizeof(*outputBufferPtr));
    }
    WdfRequestComplete(WdfRequest, status);
}

_Use_decl_annotations_
void
iMXLPUARTIoctlSetBreakOnOff(
    IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest,
    bool BreakOn
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    iMXLPUARTSetBreakOnOff(DeviceContextPtr, BreakOn);
    WdfRequestComplete(WdfRequest, STATUS_SUCCESS);
}

_Use_decl_annotations_
void
iMXLPUARTIoctlGetModemControl(
    const IMX_LPUART_DEVICE_CONTEXT *DeviceContextPtr,
    WDFREQUEST WdfRequest
)
{
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    //
    // These masks define access the modem control register.
    // Define locally to avoid conflict with potential future public definition.
    //
    enum SERIAL_MCR : ULONG {

        //
        // This bit controls the data terminal ready (DTR) line.  When
        // this bit is set the line goes to logic 0 (which is then inverted
        // by normal hardware).  This is normally used to indicate that
        // the device is available to be used.  Some odd hardware
        // protocols (like the kernel debugger) use this for handshaking
        // purposes.
        //
        SERIAL_MCR_DTR = 0x01,

        //
        // This bit controls the ready to send (RTS) line.  When this bit
        // is set the line goes to logic 0 (which is then inverted by the normal
        // hardware).  This is used for hardware handshaking.  It indicates that
        // the hardware is ready to send data and it is waiting for the
        // receiving end to set clear to send (CTS).
        //
        SERIAL_MCR_RTS = 0x02,

        //
        // This bit is used for general purpose output.
        //
        SERIAL_MCR_OUT1 = 0x04,

        //
        // This bit is used for general purpose output.
        //
        SERIAL_MCR_OUT2 = 0x08,

        //
        // This bit controls the loopback testing mode of the device.  Basically
        // the outputs are connected to the inputs (and vice versa).
        //
        SERIAL_MCR_LOOP = 0x10,
    };

    NTSTATUS status;
    const IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = DeviceContextPtr->InterruptContextPtr;

    ULONG *outputBufferPtr;

    status = WdfRequestRetrieveOutputBuffer(WdfRequest, sizeof(*outputBufferPtr), reinterpret_cast<PVOID*>(&outputBufferPtr), nullptr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("Failed to retrieve output buffer for IOCTL_SERIAL_GET_MODEM_CONTROL request. (status = %!STATUS!)", status);
    }

    if (!NT_SUCCESS(status)) {
        ULONG mcr = 0;
        if ((interruptContextPtr->CTRLCopy & IMX_LPUART_CTRL_LOOPS) != 0) {
            mcr |= SERIAL_MCR_LOOP;
        }
        *outputBufferPtr = mcr;
        WdfRequestSetInformation(WdfRequest, sizeof(*outputBufferPtr));
    }
    WdfRequestComplete(WdfRequest, status);
}

IMX_LPUART_NONPAGED_SEGMENT_END; //================================================
IMX_LPUART_PAGED_SEGMENT_BEGIN; //=================================================

_Use_decl_annotations_
NTSTATUS
iMXLPUARTEvtDevicePrepareHardware(
    WDFDEVICE WdfDevice,
    WDFCMRESLIST /*ResourcesRaw*/,
    WDFCMRESLIST ResourcesTranslated
)
{
    PAGED_CODE();
    IMX_LPUART_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    NTSTATUS status = STATUS_SUCCESS;
    const ULONG resourceCount = WdfCmResourceListGetCount(ResourcesTranslated);
    const CM_PARTIAL_RESOURCE_DESCRIPTOR *memResourcePtr = nullptr;
    const CM_PARTIAL_RESOURCE_DESCRIPTOR *rxDmaResourcePtr = nullptr;
    const CM_PARTIAL_RESOURCE_DESCRIPTOR *txDmaResourcePtr = nullptr;
    ULONG interruptResourceCount = 0;
    ULONG dmaResourceCount = 0;
    LARGE_INTEGER selfConnectionId = {0};

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");
    //
    // Look for single memory resource, single interrupt resource, and
    // optional UARTSerialBus connection resource
    //
    for (ULONG i = 0; i < resourceCount; ++i) {
        const CM_PARTIAL_RESOURCE_DESCRIPTOR *resourcePtr = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);

        switch (resourcePtr->Type) {
        case CmResourceTypeMemory:
            if (memResourcePtr != nullptr) {
                IMX_LPUART_LOG_WARNING("Received unexpected memory resource. (resourcePtr = 0x%p)", resourcePtr);
            }
            else {
                memResourcePtr = resourcePtr;
            }
                break;
        case CmResourceTypeInterrupt:
            switch (interruptResourceCount) {
            case 0:
                break;
            default:
                IMX_LPUART_LOG_WARNING("Received unexpected interrupt resource. (interruptResourceCount = %lu, resourcePtr = 0x%p)", interruptResourceCount, resourcePtr);
            }
            ++interruptResourceCount;
            break;
        case CmResourceTypeDma:
            switch (dmaResourceCount) {
            case 0:
                rxDmaResourcePtr = resourcePtr;
                break;
            case 1:
                txDmaResourcePtr = resourcePtr;
                break;
            default:
                IMX_LPUART_LOG_WARNING("Received unexpected DMA resource. (dmaResourceCount = %lu, resourcePtr = 0x%p)", dmaResourceCount, resourcePtr);
            }
            ++dmaResourceCount;
            break;
        case CmResourceTypeConnection:
            switch (resourcePtr->u.Connection.Class) {
            case CM_RESOURCE_CONNECTION_CLASS_SERIAL:
                switch (resourcePtr->u.Connection.Type) {
                case CM_RESOURCE_CONNECTION_TYPE_SERIAL_UART:
                    if (selfConnectionId.QuadPart != 0) {
                        IMX_LPUART_LOG_WARNING("Received unexpected UART connection resource. (selfConnectionId = 0x%llx)", selfConnectionId.QuadPart);
                    }
                    else {
                        selfConnectionId.LowPart = resourcePtr->u.Connection.IdLowPart;
                        selfConnectionId.HighPart = resourcePtr->u.Connection.IdHighPart;
                    }
                    NT_ASSERT(selfConnectionId.QuadPart != 0);
                    break; // CM_RESOURCE_CONNECTION_TYPE_SERIAL_UART
                } // switch (resourcePtr->u.Connection.Type)
                break; // CM_RESOURCE_CONNECTION_CLASS_SERIAL
            } // switch (resourcePtr->u.Connection.Class)
            break; // CmResourceTypeConnection
            } // switch (resourcePtr->Type)
    } // for (...)

    if ((memResourcePtr == nullptr) || (interruptResourceCount < 1)) {
        IMX_LPUART_LOG_ERROR("Did not receive required memory resource and interrupt resource. (memResourcePtr = 0x%p, interruptResourceCount = %lu)", memResourcePtr, interruptResourceCount);
        status = STATUS_DEVICE_CONFIGURATION_ERROR;
    }
    if (NT_SUCCESS(status)) {
        if (memResourcePtr->u.Memory.Length < sizeof(IMX_LPUART_REGISTERS)) {
            IMX_LPUART_LOG_ERROR("Memory resource is too small. (memResourcePtr->u.Memory.Length = %lu, sizeof(IMX_LPUART_REGISTERS) = %lu)", memResourcePtr->u.Memory.Length, sizeof(IMX_LPUART_REGISTERS));
            status = STATUS_DEVICE_CONFIGURATION_ERROR;
        }
    }

    if (NT_SUCCESS(status)) {
        IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = iMXLPUARTGetDeviceContext(WdfDevice);
        IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr = deviceContextPtr->InterruptContextPtr;

        if (interruptContextPtr == nullptr) {
            status = STATUS_DEVICE_CONFIGURATION_ERROR;
        }

        if (NT_SUCCESS(status)) {
            NT_ASSERT(memResourcePtr->Type == CmResourceTypeMemory);

            deviceContextPtr->RegistersPtr = static_cast<IMX_LPUART_REGISTERS*>(MmMapIoSpaceEx(memResourcePtr->u.Memory.Start, memResourcePtr->u.Memory.Length, PAGE_READWRITE | PAGE_NOCACHE));

            if (deviceContextPtr->RegistersPtr == nullptr) {
                IMX_LPUART_LOG_INFORMATION("MmMapIoSpaceEx(...) failed. \n (memResourcePtr->u.Memory.Start = 0x%llx, \n (memResourcePtr->u.Memory.Length = %lu)", memResourcePtr->u.Memory.Start.QuadPart, memResourcePtr->u.Memory.Length);
                status = STATUS_INSUFFICIENT_RESOURCES;
            }

            interruptContextPtr->RegistersPtr = deviceContextPtr->RegistersPtr;
            //
            // ReleaseHardware is ALWAYS called, even if PrepareHardware fails, so
            // the cleanup of registersPtr is handled there.
            //
        }
        if (NT_SUCCESS(status)) {
            status = iMXLPUARTReadDeviceProperties(WdfDevice, deviceContextPtr);
        }
        //
        // Allocate intermediate buffers
        //
        if (NT_SUCCESS(status)) {
            NT_ASSERT(deviceContextPtr->TxBufferStorage == nullptr);
            deviceContextPtr->TxBufferStorage = static_cast<BYTE*>(ExAllocatePoolWithTag(NonPagedPoolNx, deviceContextPtr->Parameters.TxIntermediateBufferSize, IMX_LPUART_POOL_TAG));

            if (deviceContextPtr->TxBufferStorage == nullptr) {
                IMX_LPUART_LOG_LOW_MEMORY("Failed to allocate TxBufferStorage. (TxIntermediateBufferSize = %lu)", deviceContextPtr->Parameters.TxIntermediateBufferSize);

                status = STATUS_NO_MEMORY;
            }
            else {
                interruptContextPtr->TxBuffer.SetBuffer(deviceContextPtr->TxBufferStorage, deviceContextPtr->Parameters.TxIntermediateBufferSize);
            }
            NT_ASSERT(deviceContextPtr->RxBufferStorage == nullptr);
            deviceContextPtr->RxBufferStorage = static_cast<BYTE*>(ExAllocatePoolWithTag(NonPagedPoolNx, deviceContextPtr->Parameters.RxIntermediateBufferSize, IMX_LPUART_POOL_TAG));

            if (deviceContextPtr->RxBufferStorage == nullptr) {
                IMX_LPUART_LOG_LOW_MEMORY("Failed to allocate RxBufferStorage. (RxIntermediateBufferSize = %lu)", deviceContextPtr->Parameters.RxIntermediateBufferSize);

                status = STATUS_NO_MEMORY;
            }
            else {
                interruptContextPtr->RxBuffer.SetBuffer(deviceContextPtr->RxBufferStorage, deviceContextPtr->Parameters.RxIntermediateBufferSize);
            }
        }
        if (NT_SUCCESS(status)) {
            status = iMXLPUARTResetHardwareAndClearShadowedRegisters(interruptContextPtr);
            if (!NT_SUCCESS(status)) {
                return status;
            }
        }
    }
    return status;
}

_Use_decl_annotations_
NTSTATUS
iMXLPUARTEvtDeviceReleaseHardware(
    WDFDEVICE WdfDevice,
    WDFCMRESLIST /*ResourcesTranslated*/
)
{
    PAGED_CODE();
    IMX_LPUART_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = iMXLPUARTGetDeviceContext(WdfDevice);
    deviceContextPtr->InterruptContextPtr->RxBuffer.SetBuffer(nullptr, 0);
    if (deviceContextPtr->RxBufferStorage != nullptr) {
        ExFreePoolWithTag(deviceContextPtr->RxBufferStorage, IMX_LPUART_POOL_TAG);
        deviceContextPtr->RxBufferStorage = nullptr;
    }
    deviceContextPtr->InterruptContextPtr->TxBuffer.SetBuffer(nullptr, 0);
    if (deviceContextPtr->TxBufferStorage != nullptr) {
        ExFreePoolWithTag(deviceContextPtr->TxBufferStorage, IMX_LPUART_POOL_TAG);
        deviceContextPtr->TxBufferStorage = nullptr;
    }
    if (deviceContextPtr->RegistersPtr != nullptr) {
        IMX_LPUART_LOG_INFORMATION("MmUnmapIoSpace (%p, %u)", deviceContextPtr->RegistersPtr, sizeof(IMX_LPUART_REGISTERS));

        MmUnmapIoSpace(deviceContextPtr->RegistersPtr, sizeof(IMX_LPUART_REGISTERS));
        deviceContextPtr->RegistersPtr = nullptr;
    }
    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS
iMXLPUARTEvtDeviceAdd(
    WDFDRIVER /*WdfDriver*/,
    WDFDEVICE_INIT *DeviceInitPtr
)
{
    PAGED_CODE();
    IMX_LPUART_ASSERT_MAX_IRQL(PASSIVE_LEVEL);

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

#ifdef POWER
    //
    // Set PNP and Power callbacks
    //
    {
        WDF_PNPPOWER_EVENT_CALLBACKS wdfPnppowerEventCallbacks;

        WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&wdfPnppowerEventCallbacks);
        wdfPnppowerEventCallbacks.EvtDevicePrepareHardware = iMXLPUARTEvtDevicePrepareHardware;
        wdfPnppowerEventCallbacks.EvtDeviceReleaseHardware = iMXLPUARTEvtDeviceReleaseHardware;

        WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInitPtr, &wdfPnppowerEventCallbacks);

    } // PNP and Power
#endif // POWER
    //
    // Call SerCx2InitializeDeviceInit() to attach the SerCX2 to the
    // WDF pipeline.
    //
    NTSTATUS status = SerCx2InitializeDeviceInit(DeviceInitPtr);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("SerCx2InitializeDeviceInit(...) failed, (status = %!STATUS!)", status);
    }

#ifdef SECURITY
    if (NT_SUCCESS(status)) {
        //
        // Override the default security descriptor with one that allows
        // usermode read/write access.
        //
        DECLARE_CONST_UNICODE_STRING(SDDL_DEVOBJ_SERCX_SYS_ALL_ADM_ALL_UMDF_ALL_USERS_RDWR, L"D:P(A;;GA;;;SY)(A;;GA;;;BA)(A;;GA;;;UD)(A;;GRGW;;;BU)");

        status = WdfDeviceInitAssignSDDLString(DeviceInitPtr, &SDDL_DEVOBJ_SERCX_SYS_ALL_ADM_ALL_UMDF_ALL_USERS_RDWR);

        if (!NT_SUCCESS(status)) {
            IMX_LPUART_LOG_ERROR("WdfDeviceInitAssignSDDLString(...) failed. (status = %!STATUS!, SDDL_DEVOBJ_SERCX_SYS_ALL_ADM_ALL_UMDF_ALL_USERS_RDWR = %wZ)", status, &SDDL_DEVOBJ_SERCX_SYS_ALL_ADM_ALL_UMDF_ALL_USERS_RDWR);
        }
    }
#endif // SECURITY
    //
    // Create and initialize the WDF device.
    //
    if (NT_SUCCESS(status)) {
        WDFDEVICE wdfDevice;
        {
            WDF_OBJECT_ATTRIBUTES attributes;
            WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, IMX_LPUART_DEVICE_CONTEXT);

            status = WdfDeviceCreate(&DeviceInitPtr, &attributes, &wdfDevice);
            if (!NT_SUCCESS(status)) {
                IMX_LPUART_LOG_ERROR("WdfDeviceCreate(...) failed. (status = %!STATUS!)", status);
            }
            else {
                if (iMXLPUARTGetDeviceContext(wdfDevice) == NULL) {
                    status = STATUS_NO_MEMORY;
                }
            }
        }

        if (NT_SUCCESS(status)) {
            IMX_LPUART_DEVICE_CONTEXT *deviceContextPtr = new (iMXLPUARTGetDeviceContext(wdfDevice)) IMX_LPUART_DEVICE_CONTEXT();

            deviceContextPtr->WdfDevice = wdfDevice;

            //
            // Set power capabilities
            //
            if (NT_SUCCESS(status)) {
                WDF_DEVICE_POWER_CAPABILITIES powerCaps;
                WDF_DEVICE_POWER_CAPABILITIES_INIT(&powerCaps);
                powerCaps.DeviceD1 = WdfFalse;
                powerCaps.DeviceD2 = WdfFalse;

                WdfDeviceSetPowerCapabilities(wdfDevice, &powerCaps);
            }

            //
            // Set PNP capabilities
            //
            if (NT_SUCCESS(status)) {
                WDF_DEVICE_PNP_CAPABILITIES pnpCaps;

                WDF_DEVICE_PNP_CAPABILITIES_INIT(&pnpCaps);
                pnpCaps.Removable = WdfFalse;
                pnpCaps.SurpriseRemovalOK = WdfFalse;
                pnpCaps.EjectSupported = WdfFalse;

                WdfDeviceSetPnpCapabilities(wdfDevice, &pnpCaps);
            }

            //
            // Assign S0 idle policy
            //
            if (NT_SUCCESS(status)) {
                WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS idleSettings;

                WDF_DEVICE_POWER_POLICY_IDLE_SETTINGS_INIT(&idleSettings, IdleCannotWakeFromS0);
                idleSettings.IdleTimeout = IdleTimeoutDefaultValue;
                idleSettings.UserControlOfIdleSettings = IdleAllowUserControl;
                idleSettings.Enabled = WdfTrue;
                idleSettings.PowerUpIdleDeviceOnSystemWake = WdfFalse;
                idleSettings.IdleTimeoutType = SystemManagedIdleTimeout;
                idleSettings.ExcludeD3Cold = WdfFalse;

                status = WdfDeviceAssignS0IdleSettings(wdfDevice, &idleSettings);
                if (!NT_SUCCESS(status)) {
                    IMX_LPUART_LOG_ERROR("WdfDeviceAssignS0IdleSettings(...) failed. (status = %!STATUS!)", status);
                }
            }

            //
            // Create an interrupt object
            //
            if (NT_SUCCESS(status)) {
                IMX_LPUART_INTERRUPT_CONTEXT *interruptContextPtr;
                {
                    WDFINTERRUPT wdfInterrupt;
                    WDF_OBJECT_ATTRIBUTES attributes;
                    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, IMX_LPUART_INTERRUPT_CONTEXT);

                    WDF_INTERRUPT_CONFIG interruptConfig;
                    WDF_INTERRUPT_CONFIG_INIT(&interruptConfig, iMXLPUARTEvtInterruptISR, iMXLPUARTEvtInterruptDpc);

                    status = WdfInterruptCreate(wdfDevice, &interruptConfig, &attributes, &wdfInterrupt);

                    if (!NT_SUCCESS(status)) {
                        IMX_LPUART_LOG_ERROR("Failed to create interrupt object. (status = %!STATUS!)", status);
                    }

                    interruptContextPtr = new (iMXLPUARTGetInterruptContext(wdfInterrupt)) IMX_LPUART_INTERRUPT_CONTEXT();
                    if (interruptContextPtr == NULL) {
                        status = STATUS_NO_MEMORY;
                    }
                    interruptContextPtr->WdfInterrupt = wdfInterrupt;
                    deviceContextPtr->WdfInterrupt = wdfInterrupt;
                    deviceContextPtr->InterruptContextPtr = interruptContextPtr;
                }

                    //
                    // Initialize SerCx2 class extension.
                    //
                if (NT_SUCCESS(status)) {
                    SERCX2_CONFIG serCx2Config;
                    SERCX2_CONFIG_INIT(&serCx2Config, iMXLPUARTEvtSerCx2ApplyConfig, iMXLPUARTEvtSerCx2Control, iMXLPUARTEvtSerCx2PurgeFifos);

                    serCx2Config.EvtSerCx2SetWaitMask = iMXLPUARTEvtSerCx2SetWaitMask;
                    serCx2Config.EvtSerCx2FileOpen = iMXLPUARTEvtSerCx2FileOpen;
                    serCx2Config.EvtSerCx2FileClose = iMXLPUARTEvtSerCx2FileClose;

                    status = SerCx2InitializeDevice(wdfDevice, &serCx2Config);
                    if (!NT_SUCCESS(status)) {
                        IMX_LPUART_LOG_ERROR("SerCx2InitializeDevice(...) failed. (status = %!STATUS!)", status);
                    }
                }

                //
                // Configure receive PIO contexts and callbacks
                //
                if (NT_SUCCESS(status)) {
                    SERCX2_PIO_RECEIVE_CONFIG serCx2PioReceiveConfig;
                    SERCX2_PIO_RECEIVE_CONFIG_INIT(&serCx2PioReceiveConfig, iMXLPUARTEvtSerCx2PioReceiveReadBuffer, iMXLPUARTEvtSerCx2PioReceiveEnableReadyNotification, iMXLPUARTEvtSerCx2PioReceiveCancelReadyNotification);

                    WDF_OBJECT_ATTRIBUTES attributes;
                    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, IMX_LPUART_RX_CONTEXT);

                    //
                    // Create the SerCx2 RX object
                    //
                    SERCX2PIORECEIVE pioReceive;
                    status = SerCx2PioReceiveCreate(wdfDevice, &serCx2PioReceiveConfig, &attributes, &pioReceive);

                    if (!NT_SUCCESS(status)) {
                        IMX_LPUART_LOG_ERROR("SerCx2PioReceiveCreate(...) failed. (status = %!STATUS!)", status);
                    }

                    auto rxContextPtr = new (iMXLPUARTGetRxContext(pioReceive)) IMX_LPUART_RX_CONTEXT();

                    rxContextPtr->InterruptContextPtr = interruptContextPtr;
                    interruptContextPtr->SerCx2PioReceive = pioReceive;
                } // Configure receive PIO

                //
                // Configure transmit PIO contexts and callbacks
                //
                {
                    SERCX2_PIO_TRANSMIT_CONFIG serCx2PioTransmitConfig;
                    SERCX2_PIO_TRANSMIT_CONFIG_INIT(
                        &serCx2PioTransmitConfig,
                        iMXLPUARTEvtSerCx2PioTransmitWriteBuffer,
                        iMXLPUARTEvtSerCx2PioTransmitEnableReadyNotification,
                        iMXLPUARTEvtSerCx2PioTransmitCancelReadyNotification);

                    serCx2PioTransmitConfig.EvtSerCx2PioTransmitDrainFifo = iMXLPUARTEvtSerCx2PioTransmitDrainFifo;

                    serCx2PioTransmitConfig.EvtSerCx2PioTransmitCancelDrainFifo = iMXLPUARTEvtSerCx2PioTransmitCancelDrainFifo;

                    serCx2PioTransmitConfig.EvtSerCx2PioTransmitPurgeFifo = iMXLPUARTEvtSerCx2PioTransmitPurgeFifo;

                    WDF_OBJECT_ATTRIBUTES attributes;
                    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, IMX_LPUART_TX_CONTEXT);

                    //
                    // Create the SerCx2 TX object
                    //
                    SERCX2PIOTRANSMIT pioTransmit;
                    status = SerCx2PioTransmitCreate(wdfDevice, &serCx2PioTransmitConfig, &attributes, &pioTransmit);

                    if (!NT_SUCCESS(status)) {
                        IMX_LPUART_LOG_ERROR("SerCx2PioTransmitCreate(...) failed, (status = %!STATUS!)", status);
                    }

                    if (NT_SUCCESS(status)) {
                        auto txContextPtr = new (iMXLPUARTGetTxContext(pioTransmit)) IMX_LPUART_TX_CONTEXT();

                        if (txContextPtr != NULL) {
                            txContextPtr->InterruptContextPtr = interruptContextPtr;
                            interruptContextPtr->SerCx2PioTransmit = pioTransmit;
                        }
                        else {
                            status = STATUS_NO_MEMORY;
                        }
                    }
                }
            }

#ifdef REGISTRY
            //
            // Query parameters from registry
            //
            if (NT_SUCCESS(status)) {
                IMX_LPUART_WDFKEY wdfKey;

                status = WdfDeviceOpenRegistryKey(wdfDevice, PLUGPLAY_REGKEY_DEVICE, KEY_READ, WDF_NO_OBJECT_ATTRIBUTES, &wdfKey.Handle);
                if (NT_SUCCESS(status)) {
                    struct _REGVAL_DESCRIPTOR {
                        PCWSTR ValueName;
                        ULONG *DestinationPtr;
                        ULONG DefaultValue;
                    };
                    const _REGVAL_DESCRIPTOR regTable[] = {
                            {
                                L"RxIntermediateBufferSize",
                                &deviceContextPtr->Parameters.RxIntermediateBufferSize,
                                1024,
                            },
                            {
                                L"RxDmaIntermediateBufferSize",
                                &deviceContextPtr->Parameters.RxDmaIntermediateBufferSize,
                                1024 * 8,
                            },
                            {
                                L"TxIntermediateBufferSize",
                                &deviceContextPtr->Parameters.TxIntermediateBufferSize,
                                1024,
                            },
                            {
                                L"TxFifoThresholdUs",
                                &deviceContextPtr->Parameters.TxFifoThresholdUs,
                                10,
                            },
                            {
                                L"RxFifoThresholdUs",
                                &deviceContextPtr->Parameters.RxFifoThresholdUs,
                                50,
                            },
                            {
                                L"TxDpcThresholdUs",
                                &deviceContextPtr->Parameters.TxDpcThresholdUs,
                                50,
                            },
                            {
                                L"RtsTriggerLevel",
                                &deviceContextPtr->Parameters.RtsTriggerLevel,
                                30,
                            },
                            {
                                L"ModuleClockFrequency",
                                &deviceContextPtr->Parameters.ModuleClockFrequency,
                                80000000, // 80 Mhz (UART_CLK_ROOT)
                            },
                            {
                                L"RxDmaMinTransactionLength",
                                &deviceContextPtr->Parameters.RxDmaMinTransactionLength,
                                4 * IMX_LPUART_FIFO_COUNT,
                            },
                            {
                                L"TxDmaMinTransactionLength",
                                &deviceContextPtr->Parameters.TxDmaMinTransactionLength,
                                4 * IMX_LPUART_FIFO_COUNT,
                            }
                        };

                    for (ULONG i = 0; i < ARRAYSIZE(regTable); ++i) {
                        const _REGVAL_DESCRIPTOR *descriptorPtr = &regTable[i];
                        UNICODE_STRING valueName;

                        status = RtlUnicodeStringInit(&valueName, descriptorPtr->ValueName);

                        NT_ASSERT(NT_SUCCESS(status));
                        if (NT_SUCCESS(status)) {
                            status = WdfRegistryQueryULong(wdfKey.Handle, &valueName, descriptorPtr->DestinationPtr);
                        }

                        if (!NT_SUCCESS(status)) {
                            IMX_LPUART_LOG_WARNING("Failed to query registry value, using default value. (status = %!STATUS!, descriptorPtr->ValueName = %S, descriptorPtr->DefaultValue = %lu)", status, descriptorPtr->ValueName, descriptorPtr->DefaultValue);

                            *descriptorPtr->DestinationPtr = descriptorPtr->DefaultValue;
                        }
                    }
                    // destructor does 'WdfRegistryClose(wdfKey.Handle);' this.
                }
            }
#endif // REGISTRY
        }
    }
    return status;
}

_Use_decl_annotations_
VOID iMXLPUARTEvtDriverUnload(WDFDRIVER WdfDriver)
{
    PAGED_CODE();

    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    DRIVER_OBJECT *driverObjectPtr = WdfDriverWdmGetDriverObject(WdfDriver);
    WPP_CLEANUP(driverObjectPtr);
}

//
// Routine Description:
//
//  Creates a device interface, using the supplied resource hub connection ID
//  as a reference string. When SerCx2 receives a FileCreate request, it
//  looks for the connection ID in the filename, and queries the resource
//  hub to determine initial connection settings.
//
//  The _DDN ACPI method is queried to determine the device friendly name
//  as seen by WinRT. If the _DDN method is not present, no friendly name
//  will be assigned.
//
//  In 19H1, SerCx2 added support to publish a device interface. When this
//  driver no longer needs to support RS5 (17763), this function can be deleted.
//
// Arguments:
//
//  WdfDevice - The WdfDevice object
//
//  ConnectionId - A resource hub Connection Id to a UARTSerialBus connection.
//
// Return Value:
//
//  STATUS_SUCCESS - On success
//
_Use_decl_annotations_
NTSTATUS
iMXLPUARTCreateDeviceInterface(
    WDFDEVICE WdfDevice,
    LARGE_INTEGER ConnectionId
)
{
    struct _SYMLINK_NAME {
        WDFSTRING WdfString = WDF_NO_HANDLE;

        ~_SYMLINK_NAME()
        {
            PAGED_CODE();
            if (this->WdfString != WDF_NO_HANDLE)
            {
                WdfObjectDelete(this->WdfString);
            }
        }
    };

    NTSTATUS status;

    DECLARE_UNICODE_STRING_SIZE(referenceString, RESOURCE_HUB_CONNECTION_FILE_SIZE);

    PAGED_CODE();
    NT_ASSERT(ConnectionId.QuadPart != 0LL);

    status = RESOURCE_HUB_ID_TO_FILE_NAME(ConnectionId.LowPart, ConnectionId.HighPart, referenceString.Buffer);
    if (!NT_SUCCESS(status)) {
        IMX_LPUART_LOG_ERROR("RESOURCE_HUB_ID_TO_FILE_NAME() failed. (status = %!STATUS!)", status);
    }

    if (NT_SUCCESS(status)) {
        referenceString.Length = RESOURCE_HUB_CONNECTION_FILE_SIZE - sizeof(WCHAR);
        status = WdfDeviceCreateDeviceInterface(WdfDevice, &GUID_DEVINTERFACE_COMPORT, &referenceString);
        if (!NT_SUCCESS(status)) {
            IMX_LPUART_LOG_ERROR("WdfDeviceCreateDeviceInterface(...) failed. (status = %!STATUS!, GUID_DEVINTERFACE_COMPORT = %!GUID!, referenceString = %wZ)", status, &GUID_DEVINTERFACE_COMPORT, &referenceString);
        }
    }

    if (NT_SUCCESS(status)) {
        _SYMLINK_NAME symlinkName;
        UNICODE_STRING symlinkNameWsz;

        status = WdfStringCreate(nullptr, WDF_NO_OBJECT_ATTRIBUTES, &symlinkName.WdfString);
        if (!NT_SUCCESS(status)) {
            IMX_LPUART_LOG_ERROR("WdfStringCreate(...) failed. (status = %!STATUS!)", status);
        }

        if (NT_SUCCESS(status)) {
            status = WdfDeviceRetrieveDeviceInterfaceString(WdfDevice, &GUID_DEVINTERFACE_COMPORT, &referenceString, symlinkName.WdfString);
            if (!NT_SUCCESS(status)) {
                IMX_LPUART_LOG_ERROR("WdfDeviceRetrieveDeviceInterfaceString() failed. (status = %!STATUS!, GUID_DEVINTERFACE_COMPORT = %!GUID!)", status, &GUID_DEVINTERFACE_COMPORT);
            }
        }

        if (NT_SUCCESS(status)) {
            DEVPROP_BOOLEAN isRestricted = DEVPROP_FALSE;
            //
            // IsRestricted interface property works in conjunction with the
            // RestrictedOverrideForSystemContainerAllowed flag set on the interface
            // class to allow access to internal devices (belonging to system
            // container) through the device broker. Setting it to FALSE is required
            // to explicitly grant access.
            //

            WdfStringGetUnicodeString(symlinkName.WdfString, &symlinkNameWsz);
            status = IoSetDeviceInterfacePropertyData(&symlinkNameWsz, &DEVPKEY_DeviceInterface_Restricted, 0, 0,
                /* Flags */ DEVPROP_TYPE_BOOLEAN, sizeof(isRestricted), &isRestricted);
            if (!NT_SUCCESS(status)) {
                IMX_LPUART_LOG_ERROR("IoSetDeviceInterfacePropertyData(...DEVPKEY_DeviceInterface_Restricted...) failed. (status = %!STATUS!, symlinkNameWsz = %wZ)", status, &symlinkNameWsz);
            }
        }

        //
        // Get DosDeviceName from registry
        //
        if (NT_SUCCESS(status)) {
            IMX_LPUART_WDFKEY parametersKey;

            status = WdfDeviceOpenRegistryKey(WdfDevice, PLUGPLAY_REGKEY_DEVICE, KEY_QUERY_VALUE, WDF_NO_OBJECT_ATTRIBUTES, &parametersKey.Handle);
            if (!NT_SUCCESS(status)) {
                IMX_LPUART_LOG_ERROR("Failed to open device parameters registry key. (status = %!STATUS!)", status);
            }

            if (NT_SUCCESS(status)) {
                DECLARE_CONST_UNICODE_STRING(dosDeviceNameRegvalStr, L"DosDeviceName");
                DECLARE_UNICODE_STRING_SIZE(portName, 64);
                status = WdfRegistryQueryUnicodeString(parametersKey.Handle, &dosDeviceNameRegvalStr, nullptr /* ValueByteLength */, &portName);

                if (NT_SUCCESS(status) &&
                    ((portName.Length + sizeof(WCHAR)) < portName.MaximumLength)) {

                    // Null-terminate PortName
                    portName.Buffer[portName.Length / sizeof(WCHAR)] = UNICODE_NULL;

                    // Set the port friendly name
                    status = IoSetDeviceInterfacePropertyData(&symlinkNameWsz, &DEVPKEY_DeviceInterface_Serial_PortName, LOCALE_NEUTRAL, 0,
                        /* Flags */ DEVPROP_TYPE_STRING, portName.Length + sizeof(WCHAR), portName.Buffer);

                    if (!NT_SUCCESS(status)) {
                        IMX_LPUART_LOG_ERROR("IoSetDeviceInterfacePropertyData(...DEVPKEY_DeviceInterface_Serial_PortName...) failed. (status = %!STATUS!, symlinkNameWsz = %wZ, portName = %wZ)", status, &symlinkNameWsz, &portName);
                    }
                    IMX_LPUART_LOG_INFORMATION("Successfully assigned PortName to device interface. (symlinkNameWsz = %wZ, portName = %wZ)", &symlinkNameWsz, &portName);
                }
                else {
                    IMX_LPUART_LOG_WARNING("Failed to query DosDeviceName from registry. Skipping assignment of PortName. (status = %!STATUS!, dosDeviceNameRegvalStr = %wZ, portName.Length = %d, portName.MaximumLength = %d)",
                        status, &dosDeviceNameRegvalStr, portName.Length, portName.MaximumLength);
                }
                IMX_LPUART_LOG_INFORMATION("Successfully created device interface. (symlinkNameWsz = %wZ)", &symlinkNameWsz);
            }
        }
    }
    return status;
}

_Use_decl_annotations_
NTSTATUS
iMXLPUARTParseConnectionDescriptor(
    const RH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER *RhBufferPtr,
    ULONG *BaudRatePtr,
    SERIAL_LINE_CONTROL *LineControlPtr,
    SERIAL_HANDFLOW *HandflowPtr,
    bool *RtsCtsEnabledPtr
)
{
    //
    // See the ACPI 5.0 specification
    //
    enum PNP_SERIAL_BUS_TYPE {
        PNP_SERIAL_BUS_TYPE_I2C = 0x1,
        PNP_SERIAL_BUS_TYPE_SPI = 0x2,
        PNP_SERIAL_BUS_TYPE_UART = 0x3,
    };

    //
    // ACPI-defined values for the TypeSpecificFlags field of
    // PNP_LPUART_SERIAL_BUS_DESCRIPTOR
    //
    enum UART_SERIAL_FLAG : ULONG {
        UART_SERIAL_FLAG_FLOW_CTL_NONE = (0x0000 << 0),
        UART_SERIAL_FLAG_FLOW_CTL_HW = (0x0001 << 0),
        UART_SERIAL_FLAG_FLOW_CTL_XONXOFF = (0x0002 << 0),
        UART_SERIAL_FLAG_FLOW_CTL_MASK = (0x0003 << 0),
        UART_SERIAL_FLAG_STOP_BITS_0 = (0x0000 << 2),
        UART_SERIAL_FLAG_STOP_BITS_1 = (0x0001 << 2),
        UART_SERIAL_FLAG_STOP_BITS_1_5 = (0x0002 << 2),
        UART_SERIAL_FLAG_STOP_BITS_2 = (0x0003 << 2),
        UART_SERIAL_FLAG_STOP_BITS_MASK = (0x0003 << 2),
        UART_SERIAL_FLAG_DATA_BITS_5 = (0x0000 << 4),
        UART_SERIAL_FLAG_DATA_BITS_6 = (0x0001 << 4),
        UART_SERIAL_FLAG_DATA_BITS_7 = (0x0002 << 4),
        UART_SERIAL_FLAG_DATA_BITS_8 = (0x0003 << 4),
        UART_SERIAL_FLAG_DATA_BITS_9 = (0x0004 << 4),
        UART_SERIAL_FLAG_DATA_BITS_MASK = (0x0007 << 4),
        UART_SERIAL_FLAG_BIG_ENDIAN = (0x0001 << 7),
    };

    //
    // ACPI-defined values for the Parity field of
    // PNP_LPUART_SERIAL_BUS_DESCRIPTOR
    //
    enum UART_SERIAL_PARITY : ULONG {
        UART_SERIAL_PARITY_NONE = 0x00,
        UART_SERIAL_PARITY_EVEN = 0x01,
        UART_SERIAL_PARITY_ODD = 0x02,
        UART_SERIAL_PARITY_MARK = 0x03,
        UART_SERIAL_PARITY_SPACE = 0x04,
    };

    //
    // ACPI-defined values for the SerialLinesEnabled field of
    // PNP_LPUART_SERIAL_BUS_DESCRIPTOR
    //
    enum UART_SERIAL_LINES : ULONG {
        UART_SERIAL_LINES_DCD = (0x0001 << 2),
        UART_SERIAL_LINES_RI = (0x0001 << 3),
        UART_SERIAL_LINES_DSR = (0x0001 << 4),
        UART_SERIAL_LINES_DTR = (0x0001 << 5),
        UART_SERIAL_LINES_CTS = (0x0001 << 6),
        UART_SERIAL_LINES_RTS = (0x0001 << 7),
    };

#include <pshpack1.h>
    struct PNP_LPUART_SERIAL_BUS_DESCRIPTOR : PNP_SERIAL_BUS_DESCRIPTOR {
        ULONG BaudRate;
        USHORT RxBufferSize;
        USHORT TxBufferSize;
        UCHAR Parity;
        UCHAR SerialLinesEnabled;

        //
        // followed by optional Vendor Data and
        // PNP_IO_DESCRIPTOR_RESOURCE_NAME
        //
    };
#include <poppack.h>

    NTSTATUS status = STATUS_SUCCESS;
    const PNP_LPUART_SERIAL_BUS_DESCRIPTOR *uartDescriptorPtr = reinterpret_cast<const PNP_LPUART_SERIAL_BUS_DESCRIPTOR*>(&RhBufferPtr->ConnectionProperties);

    PAGED_CODE();
    IMX_LPUART_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    //
    // Get ACPI descriptor
    //
    if (RhBufferPtr->PropertiesLength < sizeof(*uartDescriptorPtr)) {
        IMX_LPUART_LOG_ERROR("Connection properties is too small. (RhBufferPtr->PropertiesLength = %lu, sizeof(*uartDescriptorPtr) = %lu)", RhBufferPtr->PropertiesLength, sizeof(*uartDescriptorPtr));

        status = STATUS_INVALID_PARAMETER;
    }

    if (NT_SUCCESS(status)) {
        if (uartDescriptorPtr->SerialBusType != PNP_SERIAL_BUS_TYPE_UART) {
            IMX_LPUART_LOG_ERROR("ACPI Connnection descriptor is not a UART connection descriptor. (uartDescriptorPtr->SerialBusType = 0x%lx, PNP_SERIAL_BUS_TYPE_UART = 0x%lx)", uartDescriptorPtr->SerialBusType, PNP_SERIAL_BUS_TYPE_UART);

            status = STATUS_INVALID_PARAMETER;
        }
    }

    if (NT_SUCCESS(status)) {
        const ULONG typeSpecificFlags = uartDescriptorPtr->TypeSpecificFlags;

        RtlZeroMemory(LineControlPtr, sizeof(*LineControlPtr));
        RtlZeroMemory(HandflowPtr, sizeof(*HandflowPtr));

        //
        // Validate flow control
        //
        switch (typeSpecificFlags & UART_SERIAL_FLAG_FLOW_CTL_MASK) {
        case UART_SERIAL_FLAG_FLOW_CTL_NONE:
            HandflowPtr->ControlHandShake &= ~SERIAL_OUT_HANDSHAKEMASK;
            HandflowPtr->FlowReplace &= ~(SERIAL_RTS_MASK | SERIAL_AUTO_TRANSMIT | SERIAL_AUTO_RECEIVE);
            HandflowPtr->FlowReplace |= SERIAL_RTS_CONTROL;
            break;

        case UART_SERIAL_FLAG_FLOW_CTL_HW:
            HandflowPtr->ControlHandShake &= ~SERIAL_OUT_HANDSHAKEMASK;
            HandflowPtr->ControlHandShake |= SERIAL_CTS_HANDSHAKE;
            HandflowPtr->FlowReplace &= ~(SERIAL_RTS_MASK | SERIAL_AUTO_TRANSMIT | SERIAL_AUTO_RECEIVE);
            HandflowPtr->FlowReplace |= SERIAL_RTS_HANDSHAKE;
            break;

        default:
            IMX_LPUART_LOG_ERROR("Unsupported flow control value. (typeSpecificFlags = 0x%lx)", typeSpecificFlags);
            status = STATUS_NOT_SUPPORTED;
        }

        //
        // Validate stop bits
        //
        switch (typeSpecificFlags & UART_SERIAL_FLAG_STOP_BITS_MASK) {
        case UART_SERIAL_FLAG_STOP_BITS_1:
            LineControlPtr->StopBits = STOP_BIT_1;
            break;
        case UART_SERIAL_FLAG_STOP_BITS_1_5:
            LineControlPtr->StopBits = STOP_BITS_1_5;
            break;
        case UART_SERIAL_FLAG_STOP_BITS_2:
            LineControlPtr->StopBits = STOP_BITS_2;
            break;

        case UART_SERIAL_FLAG_STOP_BITS_0:  __fallthrough;
        default:
            IMX_LPUART_LOG_ERROR("Unsupported serial stop bits. (typeSpecificFlags = 0x%lx)", typeSpecificFlags);
            status = STATUS_NOT_SUPPORTED;
        }

        //
        // Validate data bits
        //
        switch (typeSpecificFlags & UART_SERIAL_FLAG_DATA_BITS_MASK) {
        case UART_SERIAL_FLAG_DATA_BITS_5:
            LineControlPtr->WordLength = 5;
            break;

        case UART_SERIAL_FLAG_DATA_BITS_6:
            LineControlPtr->WordLength = 6;
            break;

        case UART_SERIAL_FLAG_DATA_BITS_7:
            LineControlPtr->WordLength = 7;
            break;

        case UART_SERIAL_FLAG_DATA_BITS_8:
            LineControlPtr->WordLength = 8;
            break;

        case UART_SERIAL_FLAG_DATA_BITS_9:
            LineControlPtr->WordLength = 9;
            break;

        default:
            status = STATUS_INVALID_PARAMETER;
            IMX_LPUART_LOG_ERROR("Invalid data bits flag. (typeSpecificFlags = 0x%lx)", typeSpecificFlags);
        }

        //
        // Check big endian bit
        //
        if ((typeSpecificFlags & UART_SERIAL_FLAG_BIG_ENDIAN) != 0) {
            status = STATUS_NOT_SUPPORTED;
            IMX_LPUART_LOG_ERROR("Big endian mode is not supported. (typeSpecificFlags = 0x%lx)", typeSpecificFlags);
        }

        //
        // Validate parity
        //
        switch (uartDescriptorPtr->Parity) {
        case UART_SERIAL_PARITY_NONE:
            LineControlPtr->Parity = NO_PARITY;
            break;

        case UART_SERIAL_PARITY_EVEN:
            LineControlPtr->Parity = EVEN_PARITY;
            break;

        case UART_SERIAL_PARITY_ODD:
            LineControlPtr->Parity = ODD_PARITY;
            break;

        case UART_SERIAL_PARITY_MARK:
            LineControlPtr->Parity = MARK_PARITY;
            break;

        case UART_SERIAL_PARITY_SPACE:
            LineControlPtr->Parity = SPACE_PARITY;
            break;

        default:
            status = STATUS_INVALID_PARAMETER;
            IMX_LPUART_LOG_ERROR("Invalid parity. (Parity = %d)", uartDescriptorPtr->Parity);
        }
    }
    if (NT_SUCCESS(status)) {
        //
        // We support RTS/CTS only
        //
        switch (uartDescriptorPtr->SerialLinesEnabled) {
        case 0:
            *RtsCtsEnabledPtr = false;
            break;

        case UART_SERIAL_LINES_CTS | UART_SERIAL_LINES_RTS:
            //
            // The presence of these flags is how we know whether or not to allow
            // RTS/CTS handshaking to be enabled in SetHandflow.
            //
            *RtsCtsEnabledPtr = true;
            break;

        default:
            status = STATUS_NOT_SUPPORTED;
            IMX_LPUART_LOG_ERROR("SerialLinesEnabled not supported. (SerialLinesEnabled = 0x%lx)", uartDescriptorPtr->SerialLinesEnabled);
        }
    }
    if (NT_SUCCESS(status)) {
        *BaudRatePtr = uartDescriptorPtr->BaudRate;
    }
    return status;
}

IMX_LPUART_PAGED_SEGMENT_END; //===================================================
IMX_LPUART_INIT_SEGMENT_BEGIN; //==================================================

_Use_decl_annotations_
NTSTATUS
DriverEntry(
    DRIVER_OBJECT *DriverObjectPtr,
    UNICODE_STRING *RegistryPathPtr
)
{
    NTSTATUS status = STATUS_SUCCESS;

    PAGED_CODE();
    IMX_LPUART_LOG_TRACE("%!FUNC! ()");

    //
    // Initialize logging
    //
    {
        WPP_INIT_TRACING(DriverObjectPtr, RegistryPathPtr);
        RECORDER_CONFIGURE_PARAMS recorderConfigureParams;
        RECORDER_CONFIGURE_PARAMS_INIT(&recorderConfigureParams);
        WppRecorderConfigure(&recorderConfigureParams);
#if DBG
        WPP_RECORDER_LEVEL_FILTER(IMX_LPUART_TRACING_DEFAULT) = TRUE;
#endif // DBG
    }
    {

        WDFDRIVER wdfDriver;
        WDF_DRIVER_CONFIG wdfDriverConfig;

        WDF_DRIVER_CONFIG_INIT(&wdfDriverConfig, iMXLPUARTEvtDeviceAdd);
        wdfDriverConfig.DriverPoolTag = IMX_LPUART_POOL_TAG;
        wdfDriverConfig.EvtDriverUnload = iMXLPUARTEvtDriverUnload;

        status = WdfDriverCreate(DriverObjectPtr, RegistryPathPtr, WDF_NO_OBJECT_ATTRIBUTES, &wdfDriverConfig, &wdfDriver);

        if (!NT_SUCCESS(status)) {
            IMX_LPUART_LOG_ERROR("Failed to create WDF driver object. (status = %!STATUS!, DriverObjectPtr = %p, RegistryPathPtr = %p)", status, DriverObjectPtr, RegistryPathPtr);
        }

    }
    return status;
}

IMX_LPUART_INIT_SEGMENT_END; //====================================================
