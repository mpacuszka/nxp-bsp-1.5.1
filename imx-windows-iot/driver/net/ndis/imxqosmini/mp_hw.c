/*
* Copyright 2018-2020,2022 NXP
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* * Neither the name of NXP nor the names of its contributors may be used to
* endorse or promote products derived from this software without specific prior
* written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "precomp.h"

/*++
Routine Description:
    This function adds the Hash value to the Hash table (Descriptor Individual Address Register).
Arguments:
    pAdapter     Pointer to adapter data
Return Value:
    None
--*/
_Use_decl_annotations_
void SetUnicast(PMP_ADAPTER pAdapter)
{
    DBG_ENET_DEV_OIDS_PRINT_TRACE("UCast        = %02x-%02x-%02x-%02x-%02x-%02x", pAdapter->FecMacAddress[0], pAdapter->FecMacAddress[1], pAdapter->FecMacAddress[2], pAdapter->FecMacAddress[3], pAdapter->FecMacAddress[4], pAdapter->FecMacAddress[5]);
    pAdapter->ENETRegBase->MAC_ADDRESS[0].HIGH.R = *(UINT16*)&pAdapter->FecMacAddress[4];
    pAdapter->ENETRegBase->MAC_ADDRESS[0].LOW.R = *(UINT32*)&pAdapter->FecMacAddress[0];
}

/*++
Routine Description:
    This function adds the Hash value to the Hash table (Descriptor Group Address Register).
Arguments:
    pAdapter    Pointer to adapter data
    pAddr       Pointer to multicast list
Return Value:
    None
--*/
_Use_decl_annotations_
void AddMultiCast(PMP_ADAPTER pAdapter, UCHAR *pAddr)
{
    UINT32 Idx, LastMacIdx;

    // Get index of the last possible MAC address
    LastMacIdx = pAdapter->ENETRegBase->MAC_HW_FEAT[0].B.ADDMACADRSEL;

    // Go through all MAC addresses except the individual one (index 0) and disable them
    for (Idx = 1U; Idx <= LastMacIdx; Idx++) {
        // Search for unused index
        if (pAdapter->ENETRegBase->MAC_ADDRESS[Idx].HIGH.B.AE == 0U) {
            DBG_ENET_DEV_OIDS_PRINT_TRACE("Writing new MAC addr %02x-%02x-%02x-%02x-%02x-%02x at index: %d", pAddr[0], pAddr[1], pAddr[2], pAddr[3], pAddr[4], pAddr[5], Idx);
            pAdapter->ENETRegBase->MAC_ADDRESS[Idx].HIGH.R = *(UINT16*)&pAddr[4];
            pAdapter->ENETRegBase->MAC_ADDRESS[Idx].LOW.R = *(UINT32*)&pAddr[0];
            pAdapter->ENETRegBase->MAC_ADDRESS[Idx].HIGH.B.AE = 1U;
            break;
        }
    }

    if (Idx > LastMacIdx) {
        DBG_ENET_DEV_OIDS_PRINT_TRACE("Unable to add new MAC address, not enaugh empty slots", Idx);
        ASSERT(0);
    }
}

/*++
Routine Description:
    This function clears the Hash table (Descriptor Group Address Register).
Arguments:
    pAdapter    Pointer to adapter data
Return Value:
    None
--*/
_Use_decl_annotations_
void ClearAllMultiCast(PMP_ADAPTER pAdapter)
{
    UINT32 Idx, LastMacIdx;
    DBG_ENET_DEV_OIDS_PRINT_TRACE("Clearing all multicasts addresses");

    // Get index of the last possible MAC address
    LastMacIdx = pAdapter->ENETRegBase->MAC_HW_FEAT[0].B.ADDMACADRSEL;

    // Go through all MAC addresses except the individual one (index 0) and disable them
    for (Idx = 1U; Idx <= LastMacIdx; Idx++) {
        pAdapter->ENETRegBase->MAC_ADDRESS[Idx].HIGH.R = 0U;
    }
}

/*++
Routine Description:
    MiniportHandleInterrupt handler
Arguments:
    MiniportInterruptContext
        Pointer to the interrupt context. In this is a pointer to the adapter structure.
    MiniportDpcContext
        Not used
    ReceiveThrottleParameters
        A pointer to an NDIS_RECEIVE_THROTTLE_PARAMETERS structure. This structure specifies the maximum number of
        NET_BUFFER_LIST structures that a miniport driver should indicate in a DPC.
    NdisReserved2
        Not used
Return Value:
    None
--*/
_Use_decl_annotations_
VOID EnetDpc(NDIS_HANDLE  MiniportInterruptContext, PVOID MiniportDpcContext, PVOID ReceiveThrottleParameters, PVOID NdisReserved2)
{
    PMP_ADAPTER                        pAdapter = (PMP_ADAPTER)MiniportInterruptContext;
    UINT32                             InterruptEvent, InterruptFlags;
    PNDIS_RECEIVE_THROTTLE_PARAMETERS  pRecvThrottleParameters = (PNDIS_RECEIVE_THROTTLE_PARAMETERS)ReceiveThrottleParameters;
    ULONG                              MaxNBLsToIndicate;

    UNREFERENCED_PARAMETER(MiniportDpcContext);
    UNREFERENCED_PARAMETER(NdisReserved2);

    DBG_ENET_DEV_DPC_METHOD_BEG();
    MaxNBLsToIndicate = pRecvThrottleParameters->MaxNblsToIndicate;
    if (MaxNBLsToIndicate == NDIS_INDICATE_ALL_NBLS) {
        MaxNBLsToIndicate = MAXULONG;
    }

    pRecvThrottleParameters->MoreNblsPending = FALSE;
    do {
        NdisDprAcquireSpinLock(&pAdapter->Dev_SpinLock);
        pAdapter->DpcQueued  = FALSE;
        pAdapter->DpcRunning = TRUE;
        InterruptFlags = pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_STAT.R;  // Get current interrupt flags from HW register
        pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_STAT.R = InterruptFlags;  // Clear current interrupt flags in HW register
        InterruptFlags |= pAdapter->InterruptFlags;                        // Add saved interrupt flags
        pAdapter->InterruptFlags = 0;                                      // Clear saved interrupt flags
        NdisDprReleaseSpinLock(&pAdapter->Dev_SpinLock);
        InterruptEvent = InterruptFlags;                                   // Compute new interrupt flags

        if (!InterruptEvent)                                               // An interrupt ready to be served?
            break;                                                         // No, finish DPC

        if (InterruptEvent & IMX_ENET_QOS_DMA_CHX_STAT_TI_MASK) {                        // Handle frame(s) sent or sent error interrupt
            MpHandleTxInterrupt(pAdapter, InterruptEvent);
        }
        if (InterruptEvent & IMX_ENET_QOS_DMA_CHX_STAT_RI_MASK) {                        // Handle frame(s) received or receive error interrupt
            MpHandleRecvInterrupt(pAdapter, &MaxNBLsToIndicate, pRecvThrottleParameters);
            if (pRecvThrottleParameters->MoreNblsPending) {
                NdisDprAcquireSpinLock(&pAdapter->Dev_SpinLock);
                pAdapter->InterruptFlags |= (InterruptEvent & IMX_ENET_QOS_DMA_CHX_STAT_RI_MASK); // We have to prepare interrupt flags for NDIS called EnetDPC
                NdisDprReleaseSpinLock(&pAdapter->Dev_SpinLock);
                break;
            }
        }

    } while (0);

    NdisDprAcquireSpinLock(&pAdapter->Dev_SpinLock);
    pAdapter->DpcRunning = FALSE;
    NdisDprReleaseSpinLock(&pAdapter->Dev_SpinLock);

    if (!pRecvThrottleParameters->MoreNblsPending) {
      NdisMSynchronizeWithInterruptEx(pAdapter->NdisInterruptHandle, 0, EnetEnableRxAndTxInterrupts, pAdapter);
    }
    DBG_ENET_DEV_DPC_METHOD_END();
}

/*++
Routine Description:
    Miniport ISR callback function.
    Disables all ENET peripheral interrupts and queues DPC.
Arguments:
    MiniportInterruptContext  Pointer to the miniport adapter data structure.
    QueueDefaultInterruptDpc  Set to TRUE value to queue DPC on default(this) CPU.
    TargetProcessors          Pointer to a bitmap specifying Target processors which should run the DPC if QueueDefaultInterruptDpc = FALSE.
Return Value:
    TRUE   The miniport recognizes the interrupt as ENET peripheral interrupt.
    FALSE  Not ENET peripheral interrupt, other ISR will be called by OS if registered.
--*/
_Use_decl_annotations_
BOOLEAN EnetIsr(NDIS_HANDLE MiniportInterruptContext, PBOOLEAN QueueDefaultInterruptDpc, PULONG TargetProcessors)
{
    PMP_ADAPTER pAdapter = (PMP_ADAPTER)MiniportInterruptContext;
    UNREFERENCED_PARAMETER(TargetProcessors);
    DBG_ENET_DEV_ISR_METHOD_BEG();
    if (pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_INT_EN.R != 0U) {
        pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_INT_EN.R = 0x00;     // Disable all ENET interrupts. (EnetIsr will not be called again until interrupts are enabled in EnetDpc)
        pAdapter->DpcQueued = TRUE;                                   // Remember that EnetDpc is queued
        *QueueDefaultInterruptDpc = TRUE;                             // Schedule EnetDpc on the current CPU to complete the operation
        __analysis_assume(*TargetProcessors = 0);                     // If QueueDefaultInterruptDpc value is set to TRUE, NDIS ignores the value of the TargetProcessors parameter. Suppress analyser warning "Returning uninitialized memory". 
    } else {
        *QueueDefaultInterruptDpc = FALSE;                            // Do not schedule Dpc
        *TargetProcessors = 0;
        DBG_ENET_DEV_ISR_PRINT_WARNING("Spurious Interrupt.");
    }
    DBG_ENET_DEV_ISR_METHOD_END();
    return *QueueDefaultInterruptDpc;
}

/*++
Routine Description:
    Enables TxF, RxF and MII interrupts
Arguments:
    SynchronizeContext  The handle to the driver allocated context area.
    Return Value:
        Always returns TRUE
--*/
_Use_decl_annotations_
BOOLEAN EnetEnableRxAndTxInterrupts(NDIS_HANDLE SynchronizeContext) {
    PMP_ADAPTER  pAdapter = (PMP_ADAPTER)SynchronizeContext;
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_INT_EN.R |= IMX_ENET_QOS_DMA_CHX_INT_EN_NIE_MASK | IMX_ENET_QOS_DMA_CHX_INT_EN_TIE_MASK | IMX_ENET_QOS_DMA_CHX_INT_EN_RIE_MASK; // Enable Rx and Tx interrupts
    return TRUE;
}

/*++
Routine Description:
    Disables all ENET peripheral interrupt and returns TRUE if Dpc is queued or running.
    It is supposed that this function is called synchronised with EnetIsr by NdisMSynchronizeWithInterruptEx()
Arguments:
    SynchronizeContext  The handle to the driver allocated context area.
Return Value:
    TRUE   Dpc is queued or running.
    FALSE  Dpc is neither queued nor running.
--*/
_Use_decl_annotations_
BOOLEAN EnetDisableRxAndTxInterrupts(NDIS_HANDLE SynchronizeContext) {
    PMP_ADAPTER  pAdapter = (PMP_ADAPTER)SynchronizeContext;
    volatile CSP_ENET_REGS  *ENETRegBase = pAdapter->ENETRegBase;

    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_INT_EN.R &= ~(IMX_ENET_QOS_DMA_CHX_INT_EN_NIE_MASK | IMX_ENET_QOS_DMA_CHX_INT_EN_TIE_MASK | IMX_ENET_QOS_DMA_CHX_INT_EN_RIE_MASK); // Disable Rx and Tx interrupts
    ENETRegBase->DMA_CH[0].DMA_CHX_STAT.R = 0xFFFF;                                                                                      // Clear Rx and Tx interrupts flags
    return pAdapter->DpcQueued;
}

/*++
Routine Description:
    This function stops the ENET hardware.
    Called at DISPATCH_LEVEL with SM_SpinLock held from state machine handler(s).
Arguments:
    pAdapter    Pointer to adapter data
    NdisStatus  Actual NDIS status
Return Value:
    None
--*/
_Use_decl_annotations_
void EnetQos_Stop(PMP_ADAPTER pAdapter, NDIS_STATUS NdisStatus)
{
    volatile CSP_ENET_REGS  *ENETRegBase = pAdapter->ENETRegBase;

    NdisAcquireSpinLock(&pAdapter->Dev_SpinLock);
    NdisAcquireSpinLock(&pAdapter->Rx_SpinLock);
    NdisAcquireSpinLock(&pAdapter->Tx_SpinLock);
    DBG_SM_PRINT_TRACE("Stopping ENET, all spinlocks acquired");
    NdisMSynchronizeWithInterruptEx(pAdapter->NdisInterruptHandle, 0, EnetDisableRxAndTxInterrupts, pAdapter);
    _DataSynchronizationBarrier();                                             // Wait until mem-io accesses are finished
    // Transmitter disable, receiver disable
    ENETRegBase->MAC_CONFIGURATION.R &= ~(IMX_ENET_QOS_MAC_CONFIGURATION_TE_MASK | IMX_ENET_QOS_MAC_CONFIGURATION_RE_MASK);
    pAdapter->NdisStatus = NdisStatus;                    // Remember new NDIS status
    pAdapter->EnetStarted = FALSE;                        // Remember new Enet state
    DBG_SM_PRINT_TRACE("ENET stopped, status: %s, releasing all spinlocks", Dbg_GetNdisStatusName(NdisStatus));
    NdisReleaseSpinLock(&pAdapter->Tx_SpinLock);
    NdisReleaseSpinLock(&pAdapter->Rx_SpinLock);
    NdisReleaseSpinLock(&pAdapter->Dev_SpinLock);
}

/*++
Routine Description:
    This function starts the frame reception.
Arguments:
    pAdapter    Pointer to adapter data
    NdisStatus  Actual NDIS status
Return Value:
    None
--*/
_Use_decl_annotations_
void EnetQos_Start(PMP_ADAPTER pAdapter)
{
    volatile CSP_ENET_REGS  *ENETRegBase = pAdapter->ENETRegBase;

    NdisAcquireSpinLock(&pAdapter->Dev_SpinLock);
    NdisAcquireSpinLock(&pAdapter->Rx_SpinLock);
    NdisAcquireSpinLock(&pAdapter->Tx_SpinLock);
    DBG_SM_PRINT_TRACE("Starting ENET, all spinlocks acquired");
    pAdapter->EnetStarted = TRUE;                                              // Remember new Enet state
    pAdapter->NdisStatus = NDIS_STATUS_SUCCESS;                                // Remember new NDIS status
    NdisMSynchronizeWithInterruptEx(pAdapter->NdisInterruptHandle, 0, EnetEnableRxAndTxInterrupts, pAdapter);
    _DataSynchronizationBarrier();                                             // Wait until mem-io accesses are finished
    ENETRegBase->MAC_CONFIGURATION.R |= (IMX_ENET_QOS_MAC_CONFIGURATION_TE_MASK | IMX_ENET_QOS_MAC_CONFIGURATION_RE_MASK);
    DBG_SM_PRINT_TRACE("ENET started, releasing all spinlocks");
    NdisReleaseSpinLock(&pAdapter->Tx_SpinLock);
    NdisReleaseSpinLock(&pAdapter->Rx_SpinLock);
    NdisReleaseSpinLock(&pAdapter->Dev_SpinLock);
}

/*++
Routine Description:
    This function disables the ENET hardware.
Arguments:
    pAdapter    Pointer to adapter data
Return Value:
    None
--*/
_Use_decl_annotations_
void EnetQos_Deinit(PMP_ADAPTER pAdapter)
{
    DBG_SM_PRINT_TRACE("Disabling ENET");
    NdisMSynchronizeWithInterruptEx(pAdapter->NdisInterruptHandle, 0, EnetDisableRxAndTxInterrupts, pAdapter);
    pAdapter->ENETRegBase->MAC_CONFIGURATION.R &= ~(IMX_ENET_QOS_MAC_CONFIGURATION_TE_MASK | IMX_ENET_QOS_MAC_CONFIGURATION_RE_MASK);
}

/*++
Routine Description:
    Software reset of the MAC, MTL and DMA controller.
Arguments:
    pAdapter    Pointer to adapter data
Return Value:
    None
--*/
_Use_decl_annotations_
NTSTATUS EnetQos_Reset(PMP_ADAPTER pAdapter)
{
    NTSTATUS Status = STATUS_SUCCESS;
    volatile CSP_ENET_REGS* ENETRegBase = pAdapter->ENETRegBase;
    int Timeout;

    // SW reset
    ENETRegBase->DMA_MODE.R = IMX_ENET_QOS_DMA_MODE_SWR_MASK;

    Timeout = 100;
    while (--Timeout && (ENETRegBase->DMA_MODE.R & IMX_ENET_QOS_DMA_MODE_SWR_MASK));

    if (Timeout <= 0) {
        DBG_SM_PRINT_WARNING("Unable to reset ENET_QOS");
        Status = STATUS_TIMEOUT;
    }
    return Status;
}

/*++
Routine Description:
    Initialize the adapter and set up everything
Arguments:
    pAdapter    Pointer to adapter data
Return Value:
    None
--*/
_Use_decl_annotations_
NTSTATUS EnetQos_Init(PMP_ADAPTER pAdapter)
{
    NTSTATUS Status = STATUS_SUCCESS;
    UINT32 HwFeature1;
    UINT32 TxFifoSize, RxFifoSize;
    UINT32 Tqs, Rqs;
    UINT32 Val, Bw;

    // Reset peripheral
    Status = EnetQos_Reset(pAdapter);

    pAdapter->CurrentPowerState = NdisDeviceStateD0;
    pAdapter->NewPowerState = NdisDeviceStateD0;

    // Configure MAC address
    SetUnicast(pAdapter);

    // Enable CRC stripping for ingress packets
    pAdapter->ENETRegBase->MAC_CONFIGURATION.R = IMX_ENET_QOS_MAC_CONFIGURATION_ACS_MASK | IMX_ENET_QOS_MAC_CONFIGURATION_CST_MASK;

    // Disable all interrupts
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_INT_EN.R = 0U;
    pAdapter->ENETRegBase->MAC_INTERRUPT_ENABLE.R = 0U;

    // Read Tx/Rx fifo sizes in
    HwFeature1 = pAdapter->ENETRegBase->MAC_HW_FEAT[1].R;
    TxFifoSize = (HwFeature1 & IMX_ENET_QOS_MAC_HW_FEAT_TXFIFOSIZE_MASK) >> IMX_ENET_QOS_MAC_HW_FEAT_TXFIFOSIZE_SHIFT;
    RxFifoSize = (HwFeature1 & IMX_ENET_QOS_MAC_HW_FEAT_RXFIFOSIZE_MASK) >> IMX_ENET_QOS_MAC_HW_FEAT_RXFIFOSIZE_SHIFT;
    TxFifoSize = (128 << TxFifoSize);
    RxFifoSize = (128 << RxFifoSize);
    Tqs = (TxFifoSize / 256U) - 1U;
    Rqs = (RxFifoSize / 256U) - 1U;

    // Queue enable for DCB/Generic
    pAdapter->ENETRegBase->MAC_RXQ_CTRL[0].B.RXQ0EN = 2U;

    // Set Tx/Rx Queue size in number of 256B blocks and enable store and forward mode
    pAdapter->ENETRegBase->MTL_QUEUE[0].MTL_TXQX_OP_MODE.B.TQS = Tqs;
    pAdapter->ENETRegBase->MTL_QUEUE[0].MTL_TXQX_OP_MODE.B.TXQEN = 2U;
    pAdapter->ENETRegBase->MTL_QUEUE[0].MTL_TXQX_OP_MODE.B.TSF = 1U;

    pAdapter->ENETRegBase->MTL_QUEUE[0].MTL_RXQX_OP_MODE.B.RQS = Rqs;
    pAdapter->ENETRegBase->MTL_QUEUE[0].MTL_RXQX_OP_MODE.B.RSF = 1U;

    // Enable selected interrupts
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_INT_EN.R = IMX_ENET_QOS_DMA_CHX_INT_EN_NIE_MASK |  // Normal summary Interrupt
                                                        IMX_ENET_QOS_DMA_CHX_INT_EN_RIE_MASK |  // Receive Interrupt
                                                        IMX_ENET_QOS_DMA_CHX_INT_EN_TIE_MASK;   // Transmit Interrupt

    pAdapter->ENETRegBase->MAC_MMC_TX_INTERRUPT_MASK.R = 0x0FFFFFFF; // Mask all interrupts from Tx counters
    pAdapter->ENETRegBase->MAC_MMC_RX_INTERRUPT_MASK.R = 0x0FFFFFFF; // Mask all interrupts from Rx counters
    pAdapter->ENETRegBase->MAC_MMC_IPC_RX_INTERRUPT_MASK.R = 0x3FFF3FFF; // Mask all interrupts from MMC IP checksum ofload counters
    pAdapter->ENETRegBase->MAC_MMC_FPE_TX_INTERRUPT_MASK.R = 0x03; // Mask all interrupts from MMC Tx Frame preemption counters
    pAdapter->ENETRegBase->MAC_MMC_FPE_RX_INTERRUPT_MASK.R = 0x0F; // Mask all interrupts from MMC Rx Frame preemption counters

    // Init DMA
    MpTxInit(pAdapter);                                                        // Initialize Tx data structures
    MpRxInit(pAdapter);                                                        //  Initialize Rx data structures
    NdisZeroMemory(&pAdapter->StatisticsAcc, sizeof(pAdapter->StatisticsAcc)); //  Reset accumulated values of HW statistic counters
    pAdapter->InterruptFlags = 0;                                              //  No interrupt flags pending from previous call of DPC
    NT_ASSERT(pAdapter->Rx_DmaBDT_Pa.HighPart == 0);
    NT_ASSERT(pAdapter->Tx_DmaBDT_Pa.HighPart == 0);

    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_TXDESC_TAIL_PTR.R = 0xF;
    Val = pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_TXDESC_TAIL_PTR.R;
    Bw = (Val ^ 0xF) + 1;

    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_TXDESC_RING_LENGTH.R = pAdapter->Tx_DmaBDT_ItemCount - 1U;
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_RXDESC_RING_LENGTH.R = pAdapter->Rx_DmaBDT_ItemCount - 1U;

    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_TXDESC_LIST_ADDR.R = pAdapter->Tx_DmaBDT_Pa.LowPart;
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_RXDESC_LIST_ADDR.R = pAdapter->Rx_DmaBDT_Pa.LowPart;

    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_TXDESC_TAIL_PTR.R = pAdapter->Tx_DmaBDT_Pa.LowPart + (pAdapter->Tx_DmaBDT_ItemCount * (UINT32)sizeof(ENET_BD));
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_RXDESC_TAIL_PTR.R = pAdapter->Rx_DmaBDT_Pa.LowPart + (pAdapter->Rx_DmaBDT_ItemCount * (UINT32)sizeof(ENET_BD));

    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_CTRL.B.PBLX8 = 1U;
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_RX_CTRL.R = ((32U << IMX_ENET_QOS_DMA_CHX_RX_CTRL_RXPBL_SHIFT) | (2048U << IMX_ENET_QOS_DMA_CHX_RX_CTRL_RBSZ_X_0_SHIFT));
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_TX_CTRL.R = (32U << IMX_ENET_QOS_DMA_CHX_TX_CTRL_TXPBL_SHIFT);

    pAdapter->ENETRegBase->MAC_CSR_SW_CTRL.R = IMX_ENET_QOS_MAC_CSR_SW_CTRL_RCWE_MASK;

    pAdapter->ENETRegBase->DMA_SYSBUS_MODE.R = IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN16_MASK
        | IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN8_MASK
        | IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN4_MASK;

    Val = pAdapter->ENETRegBase->DMA_SYSBUS_MODE.R;

    // Enable RX DMA
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_RX_CTRL.B.SR = 1U;
    // Enable TX DMA
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_TX_CTRL.B.ST = 1U;
    // Clear DMA intr status
    pAdapter->ENETRegBase->DMA_CH[0].DMA_CHX_STAT.R = 0xFFFF;

    return Status;
}

/*++
Routine Description:
    Configure input clock rate fot ENET_QOS. Clock rate differs for 1000/100/10 Mbps.
    NOTE: The method relies on UEFI to configure 125MHz as default clock mux !!!
Arguments:
    pAdapter    Pointer to adapter data.
    ClockRate   Desired clock rate in Hz.
Return Value:
    None
--*/
_Use_decl_annotations_
void EnetQos_SetInputClock(PMP_ADAPTER pAdapter, UINT32 ClockRate)
{
    UINT32 Div, Val;
    volatile PUCHAR CcmRegs;
    PHYSICAL_ADDRESS  CCMRegBase;
    CCMRegBase.QuadPart = pAdapter->CcmBase;

    // Map CCM registers
    CcmRegs = (PUCHAR)MmMapIoSpaceEx(CCMRegBase, pAdapter->CcmSize, PAGE_READWRITE | PAGE_NOCACHE);

    // Calculate divider
    Div = 125000000 / ClockRate;

    // Disable ENET_QOS clock and clear PRE_PODF divider bitfield
    *(volatile UINT32*)(CcmRegs + ENET_QOS_CLK_ROOT) &= ~(CCM_TARGET_ROOT_ENABLE_MASK | CCM_TARGET_ROOT_POST_PODF_MASK);

    // Modify PRE_PODF divider
    *(volatile UINT32*)(CcmRegs + ENET_QOS_CLK_ROOT) |= CCM_TARGET_ROOT_POST_PODF(Div - 1U);

    // Disable ENET_QOS clock
    *(volatile UINT32*)(CcmRegs + ENET_QOS_CLK_ROOT) |= CCM_TARGET_ROOT_ENABLE_MASK;

    Val = *(volatile UINT32*)(CcmRegs + ENET_QOS_CLK_ROOT);

    // Unmap CCM registers
    if (CcmRegs) {
        MmUnmapIoSpace(CcmRegs, pAdapter->CcmSize);
    }
}

/*++
Routine Description:
    Called when link state change is detected
Arguments:
    pPHYDev - Pointer to Phy device structure
Return Value:
    None
--*/
_Use_decl_annotations_
void EnetQos_OnLinkStateChanged(PMP_ADAPTER pAdapter) {
    IMX_ENET_QOS_MAC_CONFIGURATION_t MacCfg;
    IMX_MII_LINK_STATE_t             LinkState = pAdapter->ENETDev_PHYDevice.PHYDev_LinkState;
    UINT32                           ClockRate = 125000000U;

    DBG_PHY_DEV_METHOD_BEG();

    // Read current MAC configuration register and clear duplex and speed mode cfg
    MacCfg.R = pAdapter->ENETRegBase->MAC_CONFIGURATION.R;
    MacCfg.R &= ~(IMX_ENET_QOS_MAC_CONFIGURATION_DM_MASK | IMX_ENET_QOS_MAC_CONFIGURATION_FES_MASK | IMX_ENET_QOS_MAC_CONFIGURATION_PS_MASK);

    // Set duplex mode
    if (IMX_MII_LINK_STATE_DUPLEX_MODE_GET(LinkState.R) == MediaDuplexStateFull) {
        MacCfg.B.DM = 1U;
    }

    // Set speed mode
    if (IMX_MII_LINK_STATE_SPEED_MHZ_GET(LinkState.R) == 1000) {
        MacCfg.B.PS  = 0U;
        MacCfg.B.FES = 0U;
        ClockRate = 125000000;
    } else if (IMX_MII_LINK_STATE_SPEED_MHZ_GET(LinkState.R) == 100) {
        MacCfg.B.PS  = 1U;
        MacCfg.B.FES = 1U;
        ClockRate = 25000000;
    } else if (IMX_MII_LINK_STATE_SPEED_MHZ_GET(LinkState.R) == 10) {
        MacCfg.B.PS  = 1U;
        MacCfg.B.FES = 0U;
        ClockRate = 2500000;
    }

    // Write bach MAC configuration register with new speed & duplex mode
    pAdapter->ENETRegBase->MAC_CONFIGURATION.R = MacCfg.R;

    // Reconfigure ENET clock root in CCM module
    EnetQos_SetInputClock(pAdapter, ClockRate);

    DBG_PHY_DEV_METHOD_END();
}
