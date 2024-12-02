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
 
//
// Module Name:
//
//    LPSPIhw.h
//
// Abstract:
//
//    This module contains common enum, types, and methods definitions
//    for accessing the IMX LPSPI controller hardware.
//    This controller driver uses the SPB WDF class extension (SpbCx).
//
// Environment:
//
//    kernel-mode only
//

#ifndef _LPSPI_HW_H_
#define _LPSPI_HW_H_

WDF_EXTERN_C_START


//
// LPSPI controller registers layout.
//
#include <pshpack1.h>

//
// Control Register (LPSPIx_VERIDG)
//
enum LPSPI_VERID_FEATURE : ULONG {
    LPSPI_FEATURE_32_BIT_SHIFT_REG = 0x0004,
};
union LPSPI_VERID {
    ULONG AsUlong;
    struct {
        ULONG FEATURE : 16; // Bit# 0-15
        ULONG MINOR : 8; // Bit# 16-23
        ULONG MAJOR : 8; // Bit# 24-31
    };
};

//
// Control Register (LPSPIx_PARAM)
//
union LPSPI_PARAM {
    ULONG AsUlong;
    struct {
        ULONG TXFIFO : 8; // Bit# 0-7
        ULONG RXFIFO : 8; // Bit# 8-15
        ULONG PCSNUM : 8; // Bit# 16-23
        ULONG Reserverd1 : 8; // Bit# 24-31
    };
};


//
// Control Register (LPSPIx_CR)
//
union LPSPI_CR {
    ULONG AsUlong;
    struct {
        ULONG MEN : 1; // Bit# 0
        ULONG RST : 1; // Bit# 1
        ULONG DOZEN : 1; // Bit# 2
        ULONG DBGEN : 1; // Bit# 3
        ULONG Reserverd1 : 4; // Bit# 6
        ULONG RTF : 1; // Bit# 8
        ULONG RRF : 1; // Bit# 9
        ULONG Reserverd2 : 22; // Bit# 10-31
    };
};

//
// Control Register (LPSPIx_SR)
//
enum LPSPI_INTERRUPT_GROUP : ULONG {
    LPSPI_TX_INTERRUPTS = 0x00000001,
    LPSPI_RX_INTERRUPTS = 0x00000002,
    LPSPI_TC_INTERRUPT = 0x00000400,
    LPSPI_ACKABLE_INTERRUPTS = 0x00003F00,
    LPSPI_ALL_INTERRUPTS = 0x01003F03
};
union LPSPI_SR {
    ULONG AsUlong;
    struct {
        ULONG TDF : 1; // Bit# 0
        ULONG RDF : 1; // Bit# 1
        ULONG Reserverd1 : 6; // Bit# 2-7
        ULONG WCF : 1; // Bit# 8
        ULONG FCF : 1; // Bit# 9
        ULONG TCF : 1; // Bit# 10
        ULONG TEF : 1; // Bit# 11
        ULONG REF : 1; // Bit# 12
        ULONG DMF : 1; // Bit# 13
        ULONG Reserverd2 : 10; // Bit# 14-23
        ULONG MBF : 1; // Bit# 24
        ULONG Reserverd3 : 7; // Bit# 25-31
    };
};

//
// Control Register (LPSPIx_IER)
//
enum LPSPI_INTERRUPT_TYPE : ULONG {
    TRANSMIT_DATA = 0x01,
    RECEIVE_DATA = 0x02,
    WORD_COMPLETE = 0x0100,
    FRAME_COMPLETE = 0x0200,
    TRANSFER_COMPLETE = 0x0400,
    TRANSMIT_ERROR = 0x0800,
    RECEIVE_ERROR = 0x1000,
    DATA_MATCH = 0x2000,

    ALL = 0x3F03
};
union LPSPI_IER {
    ULONG AsUlong;
    struct {
        ULONG TDIE : 1; // Bit# 0
        ULONG RDIE : 1; // Bit# 1
        ULONG Reserverd1 : 6; // Bit# 2-7
        ULONG WCIE : 1; // Bit# 8
        ULONG FCIE : 1; // Bit# 9
        ULONG TCIE : 1; // Bit# 10
        ULONG TEIE : 1; // Bit# 11
        ULONG REIE : 1; // Bit# 12
        ULONG DMIE : 1; // Bit# 13
        ULONG Reserverd2 : 18; // Bit# 14-31
    };
};

//
// Control Register (LPSPIx_DER)
//
union LPSPI_DER {
    ULONG AsUlong;
    struct {
        ULONG TDDE : 1; // Bit# 0
        ULONG RDDE : 1; // Bit# 1
        ULONG Reserverd1 : 7; // Bit# 2-8
        ULONG FCDE : 1; // Bit# 9
        ULONG Reserverd2 : 22; // Bit# 10-31
    };
};

//
// Control Register (LPSPIx_CFGR0)
//
union LPSPI_CFGR0 {
    ULONG AsUlong;
    struct {
        ULONG Reserverd1 : 8; // Bit# 0-7
        ULONG CIRFIFO : 1; // Bit# 8
        ULONG RDMO : 1; // Bit# 9
        ULONG Reserverd2 : 22; // Bit# 10-31
    };
};

//
// Control Register (LPSPIx_CFGR1)
//
enum LPSPI_MASTER : ULONG {
    SLAVE = 0x0,
    MASTER = 0x1
};
enum LPSPI_DATA_OUTPUT_CTL : ULONG {
    DATA_LAST_VALUE = 0x0,
    DATA_TRISTATED = 0x1
};
enum LPSPI_PIN_CTL : ULONG {
    SDI_AS_INPUT_SDO_AS_OUTPUT = 0x0,
    ONLY_SDI_IN_HALF_DUPLEX = 0x1,
    ONLY_SDO_IN_HALF_DUPLEX = 0x2,
    SDO_AS_INPUT_SDI_AS_OUTPUT = 0x3
};
enum LPSPI_MATCH_CTL : ULONG {
    MATCH_DISABLED = 0x0
};
enum LPSPI_PCS_POL : ULONG {
    PCS_ACTIVE_LOW = 0x0,
    PCS_ACTIVE_HIGH = 0x1
};
enum LPSPI_STALL_CTL : ULONG {
    STALL_YES = 0x0,
    STALL_NO = 0x1
};
enum LPSPI_AUTO_CS_CTL : ULONG {
    NO_AUTO_CS = 0x0,
    AUTO_CS = 0x1
};
enum LPSPI_INPUT_DATA_SAMPLE_POINT_CTL : ULONG {
    ON_SCK_EDGE = 0x0,
    ON_DELAYED_SCK_EDGE = 0x1
};
union LPSPI_CFGR1 {
    ULONG AsUlong;
    struct {
        ULONG MASTER : 1; // Bit# 0 (MASTER_MODE)
        ULONG SAMPLE : 1; // Bit# 1
        ULONG AUTOCS : 1; // Bit# 2
        ULONG NOSTALL : 1; // Bit# 3
        ULONG Reserverd1 : 4; // Bit# 4-7
        ULONG CSPOL : 2; // Bit# 8-9
        ULONG Reserverd2 : 6; // Bit# 10-15
        ULONG MATCFG : 3; // Bit# 16-18
        ULONG Reserverd3 : 5; // Bit# 19-23
        ULONG PINCFG : 2; // Bit# 24-25
        ULONG OUTCFG : 1; // Bit# 26
        ULONG Reserverd4 : 5; // Bit# 27-31
    };
};

//
// Control Register (LPSPIx_DMR0)
//
union LPSPI_DMR0 {
    ULONG AsUlong;
    struct {
        ULONG MATCH0 : 32; // Bit# 0-31
    };
};

//
// Control Register (LPSPIx_DMR1)
//
union LPSPI_DMR1 {
    ULONG AsUlong;
    struct {
        ULONG MATCH1 : 32; // Bit# 0-31
    };
};

//
// Clock Configuration Register (LPSPIx_CCR)
//
union LPSPI_CCR {
    ULONG AsUlong;
    struct {
        ULONG SCKDIV : 8; // Bit# 0-7
        ULONG DBT : 8; // Bit# 8-15
        ULONG CSSCK : 8; // Bit# 16-23
        ULONG SCKCS : 8; // Bit# 24-31
    };
};

//
// Clock Configuration 1 Register (LPSPIx_CCR1) - optional
//
union LPSPI_CCR1 {
    ULONG AsUlong;
    struct {
        ULONG SCKSET : 8; // Bit# 0-7
        ULONG SCKHLD : 8; // Bit# 8-15
        ULONG PCSPCS : 8; // Bit# 16-23
        ULONG SCKSCK : 8; // Bit# 24-31
    };
};

//
// Control Register (LPSPIx_FCR)
//
union LPSPI_FCR {
    ULONG AsUlong;
    struct {
        ULONG TXWATER : 6; // Bit# 0-5
        ULONG Reserverd1 : 10; // Bit# 6-15
        ULONG RXWATER : 6; // Bit# 16-21
        ULONG Reserverd2 : 10; // Bit# 22-31
    };
};

//
// Control Register (LPSPIx_FSR)
//
union LPSPI_FSR {
    ULONG AsUlong;
    struct {
        ULONG TXCOUNT : 7; // Bit# 0-6
        ULONG Reserverd1 : 9; // Bit# 7-15
        ULONG RXCOUNT : 7; // Bit# 16-22
        ULONG Reserverd2 : 9; // Bit# 23-31
    };
};

//
// Control Register (LPSPIx_TCR)
//
enum LPSPI_SCLK_POL : ULONG {
    SCLK_POL0 = 0x0, // Active high
    SCLK_POL1 = 0x1 // Active low
};
enum LPSPI_SCLK_PHA : ULONG {
    SCLK_PHASE0 = 0x0,
    SCLK_PHASE1 = 0x1
};
enum LPSPI_CHANNEL : ULONG {
    CS0 = 0x0,
    CS1 = 0x1,

    COUNT = 2
};
enum LPSPI_LSB_MSB_FIRST : ULONG {
    MSB_FIRST = 0x0,
    LSB_FIRST = 0x1
};
enum LPSPI_BYTE_SWAP : ULONG {
    BYTE_SWAP_NO = 0x0,
    BYTE_SWAP_YES = 0x1
};

union LPSPI_TCR {
    ULONG AsUlong;
    struct {
        ULONG FRAMESZ : 12; // Bit# 0-11
        ULONG Reserverd1 : 6; // Bit# 12-17
        ULONG TXMSK : 1; // Bit# 18
        ULONG RXMSK : 1; // Bit# 19
        ULONG CONTC : 1; // Bit# 20
        ULONG CONT : 1; // Bit# 21
        ULONG BYSW : 1; // Bit# 22
        ULONG LSBF : 1; // Bit# 23
        ULONG CS : 2; // Bit# 24
        ULONG Reserverd2 : 1; // Bit# 25-26
        ULONG PRESCALE : 3; // Bit# 27-29
        ULONG CPHA : 1; // Bit# 30
        ULONG CPOL : 1; // Bit# 31
    };
};

//
// Control Register (LPSPIx_TDR)
//
union LPSPI_TDR {
    ULONG AsUlong;
    struct {
        ULONG DATA : 32; // Bit# 0-31
    };
};

//
// Control Register (LPSPIx_RSR)
//
union LPSPI_RSR {
    ULONG AsUlong;
    struct {
        ULONG SOF : 1; // Bit# 0
        ULONG RXEMPTY : 1; // Bit# 1
        ULONG Reserverd1 : 30; // Bit# 2-31
    };
};

//
// Control Register (LPSPIx_RDR)
//
union LPSPI_RDR {
    ULONG AsUlong;
    struct {
        ULONG DATA : 32; // Bit# 0-31
    };
};

//
// The LPSPI register file 
//
struct LPSPI_REGISTERS {
    ULONG   VERID;       // 0x0000
    ULONG   PARAM;       // 0x0004
    ULONG   Reserved1[2];// 0x0008-0x000C
    ULONG   CR;          // 0x0010
    ULONG   SR;          // 0x0014
    ULONG   IER;         // 0x0018
    ULONG   DER;         // 0x001C
    ULONG   CFGR0;       // 0x0020
    ULONG   CFGR1;       // 0x0024
    ULONG   Reserved2[2];// 0x0028-0x002C
    ULONG   DMR0;        // 0x0030
    ULONG   DMR1;        // 0x0034
    ULONG   Reserved3[2];// 0x0038-0x003C
    ULONG   CCR;         // 0x0040
    ULONG   CCR1;        // 0x0044  -  optional
    ULONG   Reserved4[4];// 0x0048-0x0054
    ULONG   FCR;         // 0x0058
    ULONG   FSR;         // 0x005C
    ULONG   TCR;         // 0x0060
    ULONG   TDR;         // 0x0064
    ULONG   Reserved5[2];// 0x0068-0x006C
    ULONG   RSR;         // 0x0070
    ULONG   RDR;         // 0x0074
    ULONG   RDROR;       // 0x0078  -  optional
};

#include <poppack.h> // pshpack1

//
// The LPSPI configuration parameters
//
enum : ULONG {
    LPSPI_REGISTERS_ADDRESS_SPACE_SIZE = 0x10000,
};

//
// LPSPI properties 
//
enum : ULONG {
    LPSPI_PRESCALER_MAX = (1 << 7),
    LPSPI_SCALER_MAX = (0xFF + 2),
    LPSPI_MAX_DATA_BIT_LENGTH = 32, // Due to API assumption
    LPSPI_MAX_BURST_LENGTH_BYTES = ((1 << 12) / 8)
};

enum : ULONG {
    MAX_SPEED_ERROR_PERCENT = 10
};

#if (defined(DBG) || defined(DEBUG))
VOID
LPSPIHwDumpRegisters(_In_ LPSPI_DEVICE_EXTENSION* DevExtPtr);

#endif

//
// Routine Description:
//
//  LPSPI_CH_ATTR set an attribute for a given channel
//  is full.
//
// Arguments:
//
//  ChannelId - Channel Id
//
//  Attribute - The Attribute (binary value) to set to channel.
//
// Return Value:
// 
//  The HW expected formatted value for the given channel with the
//  given attribute.
//
__forceinline
ULONG
LPSPI_CH_ATTR (
    _In_ LPSPI_CHANNEL ChannelId,
    _In_range_(0,1) ULONG Attribute
    )
{
    NT_ASSERT(ULONG(ChannelId) <= LPSPI_CHANNEL::COUNT);
    NT_ASSERT(Attribute <= 1);

    return Attribute << ULONG(ChannelId);
}

//
// Routine Description:
//
//  LPSPIHwIsTxFifoEmpty returns TRUE if TX FIFO is empty.
//
// Arguments:
//
//  LPSPIRegsPtr - LPSPI registers base address
//
// Return Value:
//
//  TRUE if TX FIFO is empty, otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPIHwIsTxFifoEmpty (
    _In_ volatile LPSPI_REGISTERS* LPSPIRegsPtr
    )
{
    LPSPI_FSR fifoStatReg = {
        READ_REGISTER_NOFENCE_ULONG(&LPSPIRegsPtr->FSR)
    };
    return fifoStatReg.TXCOUNT == 0;
}

//
// Routine Description:
//
//  LPSPIHwIsRxFifoEmpty returns TRUE if RX FIFO is empty.
//
// Arguments:
//
//  LPSPIRegsPtr - LPSPI registers base address
//
// Return Value:
//
//  TRUE if RX FIFO is empty, otherwise FALSE.
//
__forceinline
BOOLEAN
LPSPIHwIsRxFifoEmpty (
    _In_ volatile LPSPI_REGISTERS* LPSPIRegsPtr
    )
{
    LPSPI_RSR receiveStatReg = {
        READ_REGISTER_NOFENCE_ULONG(&LPSPIRegsPtr->RSR)
        };
    return receiveStatReg.RXEMPTY == 1;
}

//
// Routine Description:
//
//  LPSPIHwQueryRxFifoCount returns the current number of words in RX FIFO.
//
// Arguments:
//
//  LPSPIRegsPtr - LPSPI registers base address
//
// Return Value:
//
//  Current RX FIFO count.
//
__forceinline
ULONG
LPSPIHwQueryRxFifoCount (
    _In_ volatile LPSPI_REGISTERS* LPSPIRegsPtr
    )
{
    LPSPI_FSR fifoStatReg = {
        READ_REGISTER_NOFENCE_ULONG(&LPSPIRegsPtr->FSR)
    };
    return fifoStatReg.RXCOUNT;
}

//
// Routine Description:
//
//  LPSPIHwQueryTransferComplete returns the status register transfer complete flag
//
// Arguments:
//
//  LPSPIRegsPtr - LPSPI registers base address
//
// Return Value:
//
//  Current TC value.
//
__forceinline
ULONG
LPSPIHwQueryTransferComplete (
    _In_ volatile LPSPI_REGISTERS* LPSPIRegsPtr
    )
{
    LPSPI_SR statReg = {
        READ_REGISTER_NOFENCE_ULONG(&LPSPIRegsPtr->SR)
    };
    return statReg.TCF;
}

NTSTATUS
LPSPIHwInitController (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
    );

NTSTATUS
LPSPIHwSetTargetConfiguration (
    _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr
    );

BOOLEAN
LPSPIHwIsSupportedDataBitLength (
    _In_ ULONG DataBitLength
    );

VOID
LPSPIHwSelectTarget (
    _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr
    );

VOID
LPSPIHwUnselectTarget (
    _In_ LPSPI_TARGET_CONTEXT* TrgCtxPtr
    );

VOID
LPSPIHwGetClockRange (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _Out_ ULONG* MinClockHzPtr,
    _Out_ ULONG* MaxClockHzPtr
    );

ULONG
LPSPIHwInterruptControl (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _In_ ULONG InterruptDisableMask,
    _In_ ULONG InterruptEnableMask
);

VOID
LPSPIHwUpdateTransferConfiguration (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _In_ LPSPI_SPB_TRANSFER* TransferPtr
    );

VOID
LPSPIHwConfigureTransfer (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _In_ LPSPI_SPB_TRANSFER* TransferPtr,
    _In_ BOOLEAN IsInitialSetup,
    _Inout_ PULONG InterruptRegPtr
    );

BOOLEAN
LPSPIHwWriteTxFIFO (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _In_ LPSPI_SPB_TRANSFER* TransferPtr,
    _In_ BOOLEAN IsInitialSetup
);

BOOLEAN
LPSPIHwReadRxFIFO (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _In_ LPSPI_SPB_TRANSFER* TransferPtr
    );

BOOLEAN
LPSPIHwWriteZerosTxFIFO (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _In_ LPSPI_SPB_TRANSFER* TransferPtr
    );

VOID
LPSPIHwClearFIFOs (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
    );

VOID
LPSPIHwDisableTransferInterrupts (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _In_ const LPSPI_SPB_TRANSFER* TransferPtr
    );

BOOLEAN
LPSPIpHwStartBurstIf (
    _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
    _In_ LPSPI_SPB_TRANSFER* TransferPtr
    );

//
// LPSPIhw private methods
//
#ifdef _LPSPI_HW_CPP_

    static NTSTATUS
    LPSPIpHwCalcFreqDivider (
        _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr,
        _In_ ULONG ConnectionSpeedHz,
        _Inout_ ULONG* PrescalerPtr,
        _Inout_ ULONG* ScalerPtr
    );

    static ULONG
    LPSPIpHwReadWordFromMdl (
        _In_ LPSPI_SPB_TRANSFER* TransferPtr,
        _Out_ PULONG DataPtr
        );

    static ULONG
    LPSPIpHwWriteWordToMdl (
        _In_ LPSPI_SPB_TRANSFER* TransferPtr,
        _In_ ULONG Data
        );

    //
    // Routine Description:
    //
    //  LPSPIpHwByteSwap swaps the given number of bytes within a
    //  32 bit variable.
    //
    // Arguments:
    //
    //  Data - The original base value
    //
    //  BytesToSwap - The number of bytes to swap.
    //
    // Return Value:
    //
    //  A ULONG value with the bytes swapped.
    //
    __forceinline
    ULONG
    LPSPIpHwByteSwap (
        _In_ ULONG Data,
        _In_ ULONG BytesToSwap
        ) 
    {
        switch (BytesToSwap) {
        case 1:
            return Data;

        case 2:
            return ULONG(RtlUshortByteSwap(Data));

        case 3:
            return RtlUlongByteSwap(Data) >> 8;

        case 4:
            return RtlUlongByteSwap(Data);

        default:
            NT_ASSERT(FALSE);
            return ULONG(-1);
        }
    }

    //
    // Routine Description:
    //
    //  LPSPIpHwDataSwap swaps the given number based on
    //  the data bit length.
    //  Since LPSPI always shifts MSB first. When we are doing 
    //  8 bit transfers, we need to make sure data is sent 
    //  in the expected order.
    //
    // Arguments:
    //
    //  Data - The original base value
    //
    //  DataBitLengthBytes - The data bit length in bytes
    //
    // Return Value:
    //
    //  A ULONG value with the bytes swapped.
    //
    __forceinline
    ULONG
    LPSPIpHwDataSwap (
        _In_ ULONG Data,
        _In_ ULONG BytesToSwap,
        _In_ ULONG DataBitLengthBytes
        )
    {
        switch (DataBitLengthBytes) {
        case 1: // 8 bit transfers
            return LPSPIpHwByteSwap(Data, BytesToSwap);

        case 2: // 16 bit transfers
            if (BytesToSwap == 4) {

                return UlongWordSwap(Data);
            }
            NT_ASSERT(BytesToSwap == 2);
            __fallthrough;
        case 4: // 32 bit transfers
            return Data;

        default:
            NT_ASSERT(FALSE);
            return ULONG(-1);
        }
    }

    static VOID
    LPSPIpHwUpdateTransfer (
        _In_ LPSPI_SPB_TRANSFER* TransferPtr,
        _In_ ULONG BytesTransferred
        );

    static VOID
    LPSPIpHwEnableLoopbackIf (
        _In_ LPSPI_DEVICE_EXTENSION* DevExtPtr
        );

#endif // _LPSPI_HW_CPP_

WDF_EXTERN_C_END

#endif // !_LPSPI_HW_H_
