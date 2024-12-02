/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2022-2023 NXP
   Licensed under the MIT License.

Abstract:
    CSoc class declaration.

*/
#pragma once

#include "imx_audio.h"
#include "common.h"
#include "imx_sairegs.h"
#include "minwavertstream.h"

#define SOC_MAX_BUFFER_NUMBER 2
class CSoc;

class CDmaBuffer
{
public:
    CDmaBuffer() { }
    ~CDmaBuffer() { }

    VOID Init(_In_ PSAI_REGISTERS SaiRegisters)
    {
        m_pSaiRegisters = SaiRegisters;
    }

    BOOLEAN IsMyStream(CMiniportWaveRTStream* stream)
    {
        if (stream == m_pRtStream)
        {
            return TRUE;
        }
        else
        {
            return FALSE;
        }
    }

    VOID ResetTxFifo();

    VOID ResetRxFifo();

    VOID FillFifos();

    VOID DrainFifos();

    VOID RegisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    );

    VOID UnregisterStream
    (
        _In_        CMiniportWaveRTStream* Stream
    );

    ULONG GetSampleIndex()
    {
        ULONG byteOffset;

        byteOffset = m_ulSamplesTransferred * m_pWfExt->Format.nBlockAlign;
        if (byteOffset >= m_ulDmaBufferSize)
        {
            byteOffset = byteOffset % m_ulDmaBufferSize;
        }

        return (byteOffset >> 2); // byteOffset / sizeof(ULONG)
    }

private:

    ULONG                  m_ulSamplesTransferred;
    ULONG                  m_ulChannel;
    CMiniportWaveRTStream* m_pRtStream;
    ULONG*                 m_DataBuffer;
    PWAVEFORMATEXTENSIBLE  m_pWfExt;
    ULONG                  m_ulDmaBufferSize;
    eDeviceType            m_DeviceType;

    volatile PSAI_REGISTERS          m_pSaiRegisters;
};

class CSoc
{
public:
    CSoc();
    ~CSoc();

public:
    NTSTATUS InitSsiBlock
    (
        _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR registersDescriptor,
        _In_ PCM_PARTIAL_RESOURCE_DESCRIPTOR interruptDescriptor,
        _In_ PDEVICE_OBJECT PDO
    )
    {
		IO_CONNECT_INTERRUPT_PARAMETERS intParameters;
		NTSTATUS ntStatus = STATUS_SUCCESS;
		
		ASSERT(m_pSaiRegisters == NULL);
		if (registersDescriptor->u.Memory.Length >= sizeof(SaiRegisters))
		{
			m_pSaiRegisters = (PSAI_REGISTERS)MmMapIoSpaceEx(registersDescriptor->u.Memory.Start,
				sizeof(SaiRegisters),
				PAGE_READWRITE | PAGE_NOCACHE);
			ASSERT(m_pSaiRegisters);
			if (m_pSaiRegisters == NULL)
			{
				ntStatus = STATUS_INSUFFICIENT_RESOURCES;
				goto Done;
			}
		}
		else {
			ntStatus = STATUS_INSUFFICIENT_RESOURCES;
			goto Done;
		}

		m_pPDO = PDO;

		ASSERT(m_pInterruptObject == NULL);

		intParameters.Version = CONNECT_FULLY_SPECIFIED;
		intParameters.FullySpecified.PhysicalDeviceObject = m_pPDO;
		intParameters.FullySpecified.InterruptObject = &m_pInterruptObject;
		intParameters.FullySpecified.ServiceRoutine = &CSoc::ISR;
		intParameters.FullySpecified.ServiceContext = this;
		intParameters.FullySpecified.SpinLock = NULL;
		intParameters.FullySpecified.SynchronizeIrql = (KIRQL)interruptDescriptor->u.Interrupt.Level;
		intParameters.FullySpecified.FloatingSave = FALSE;
		intParameters.FullySpecified.ShareVector = FALSE;
		intParameters.FullySpecified.Vector = interruptDescriptor->u.Interrupt.Vector;
		intParameters.FullySpecified.Irql = (KIRQL)interruptDescriptor->u.Interrupt.Level;
		intParameters.FullySpecified.InterruptMode = LevelSensitive;
		intParameters.FullySpecified.ProcessorEnableMask = interruptDescriptor->u.Interrupt.Affinity;
		intParameters.FullySpecified.Group = 0;

		ntStatus = IoConnectInterruptEx(&intParameters);
		if (ntStatus != STATUS_SUCCESS)
		{
			goto Done;
		}

        m_Buffer[0].Init(m_pSaiRegisters);
        m_Buffer[1].Init(m_pSaiRegisters);

		ntStatus = SetupClocks();

	Done:
		// Cleanup
		if (ntStatus != STATUS_SUCCESS)
		{
			CleanUp();
		}
		return ntStatus;
	}

    NTSTATUS RegisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    );

    NTSTATUS UnregisterStream
    (
        _In_        CMiniportWaveRTStream* Stream,
                    eDeviceType DeviceType
    );

    NTSTATUS StartDma
    (
        _In_        CMiniportWaveRTStream* Stream
    );

    NTSTATUS PauseDma
    (
        _In_        CMiniportWaveRTStream* Stream
    );

    NTSTATUS StopDma
    (
        _In_        CMiniportWaveRTStream* Stream
    );

private:

    CDmaBuffer m_Buffer[SOC_MAX_BUFFER_NUMBER];

    BOOLEAN m_bIsRenderActive;
    BOOLEAN m_bIsCaptureActive;

    volatile PSAI_REGISTERS          m_pSaiRegisters;
    PDEVICE_OBJECT                   m_pPDO;

    PKINTERRUPT                         m_pInterruptObject;
    static KSERVICE_ROUTINE             ISR;

	VOID CleanUp();
	NTSTATUS SetupClocks();

    VOID DisableInterrupts();
    VOID EnableInterrupts();

    VOID DisableInterruptsNoLock();
    VOID EnableInterruptsNoLock();

    VOID TxSoftwareReset();
    VOID RxSoftwareReset();

    BOOLEAN DoIsrWork();

    KIRQL AcquireIsrSpinLock(void)
    {
        return KeAcquireInterruptSpinLock(m_pInterruptObject);
    }

    VOID ReleaseIsrSpinLock(KIRQL oldIrql)
    {
        KeReleaseInterruptSpinLock(m_pInterruptObject, oldIrql);
    }

};

