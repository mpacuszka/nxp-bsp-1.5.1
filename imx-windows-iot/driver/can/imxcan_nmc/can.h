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

/* IOCTL interface for use by client applications */

/**
 * @file       driver/can/imxcan/can.h
 * @addtogroup imxcan
 * @{
 */

#pragma once


/** CAN Device Interface GUID */
DEFINE_GUID(GUID_DEVINTERFACE_CAN_CONTROLLER, 0x9d3484af, 0xd47f, 0x4e60, 0x8d, 0xce, 0x3a, 0x4e, 0xbf, 0xb8, 0xf6, 0x70);
#define GUID_DEVINTERFACE_CAN_CONTROLLER_WSZ L"{9d3484af-d47f-4e60-8dce-3a4ebfb8f670}"

#define CAN_WORD1_COUNT                          (64U)
/* Has CAN with Flexible Data rate (CAN FD) protocol. */
#define FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE (1)
/* Has memory error control (register MECR). */
#define FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL (0)
/* Has extended interrupt mask and flag register (register IMASK2, IFLAG2). */
#define FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER (1)




/** Type used for all status and error return values. */
typedef INT32 status_t;


/** IOCTL codes enumeration */
enum {
    /* Controller IOCTLs */
    CAN_IOCTL_ID_CONTROLLER_GET_INFO = 2048,
    CAN_IOCTL_ID_CONTROLLER_GET_CONFIG,
    CAN_IOCTL_ID_CONTROLLER_SET_CLKSRC,
    CAN_IOCTL_ID_CONTROLLER_SET_BITRATE,
    CAN_IOCTL_ID_CONTROLLER_SET_LOOPBACK,
    CAN_IOCTL_ID_CONTROLLER_SET_SELFWAKEUP,
    CAN_IOCTL_ID_CONTROLLER_SET_LISTEN_ONLY_MODE,
    CAN_IOCTL_ID_CONTROLLER_SET_DOZE,
    CAN_IOCTL_ID_CONTROLLER_IMPROVED_TIMING,
    CAN_IOCTL_ID_CONTROLLER_FD_IMPROVED_TIMING,
    CAN_IOCTL_ID_CONTROLLER_INIT,
    CAN_IOCTL_ID_CONTROLLER_FD_INIT,
    CAN_IOCTL_ID_CONTROLLER_DEINIT,
    CAN_IOCTL_ID_CONTROLLER_SET_RXMB_CONFIG,
    CAN_IOCTL_ID_CONTROLLER_SET_FD_RXMB_CONFIG,
    CAN_IOCTL_ID_CONTROLLER_SET_TXMB_CONFIG,
    CAN_IOCTL_ID_CONTROLLER_SET_FD_TXMB_CONFIG,
    CAN_IOCTL_ID_CONTROLLER_CREATE_HANDLE,
    CAN_IOCTL_ID_CONTROLLER_RECEIVE_NON_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_FD_RECEIVE_NON_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_SEND_NON_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_FD_SEND_NON_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_RECEIVE_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_SEND_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_FD_SEND_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_FD_RECEIVE_BLOCKING,
    CAN_IOCTL_ID_INVERT_NOTIFICATION,
    CAN_IOCTL_ID_CONTROLLER_ENTER_FREEZEMODE,
    CAN_IOCTL_ID_CONTROLLER_EXIT_FREEZEMODE,
    CAN_IOCTL_ID_CONTROLLER_SET_TIMING_CONFIG,
    CAN_IOCTL_ID_CONTROLLER_SET_FDTIMING_CONFIG,
    CAN_IOCTL_ID_CONTROLLER_SET_RXMB_GLOBAL_MASK,
    CAN_IOCTL_ID_CONTROLLER_SET_RXFIFO_GLOBAL_MASK,
    CAN_IOCTL_ID_CONTROLLER_SET_RX_INDIVIDUAL_MASK,
    CAN_IOCTL_ID_CONTROLLER_SET_RXFIFO_CONFIG,
    CAN_IOCTL_ID_CONTROLLER_GET_STATUS_FLAGS,
    CAN_IOCTL_ID_CONTROLLER_CLEAR_STATUS_FLAGS,
    CAN_IOCTL_ID_CONTROLLER_GET_BUSERR_COUNT,
    CAN_IOCTL_ID_CONTROLLER_GET_MBSTATUS_FLAGS,
    CAN_IOCTL_ID_CONTROLLER_CLEAR_MBSTATUS_FLAGS,
    CAN_IOCTL_ID_CONTROLLER_GET_MEMORY_ERROR_STATUS,
    CAN_IOCTL_ID_CONTROLLER_ENABLE_INTERRUPTS,
    CAN_IOCTL_ID_CONTROLLER_DISABLE_INTERRUPTS,
    CAN_IOCTL_ID_CONTROLLER_ENABLE_MB_INTERRUPTS,
    CAN_IOCTL_ID_CONTROLLER_DISABLE_MB_INTERRUPTS,
    CAN_IOCTL_ID_CONTROLLER_ENABLE_RXFIFO_DMA,
    CAN_IOCTL_ID_CONTROLLER_GET_RXFIFO_ADDR,
    CAN_IOCTL_ID_CONTROLLER_ENABLE,
    CAN_IOCTL_ID_CONTROLLER_WRITE_TXMB,
    CAN_IOCTL_ID_CONTROLLER_READ_RXMB,
    CAN_IOCTL_ID_CONTROLLER_WRITE_FD_TXMB,
    CAN_IOCTL_ID_CONTROLLER_READ_FD_RXMB,
    CAN_IOCTL_ID_CONTROLLER_READ_RXFIFO,
    CAN_IOCTL_ID_CONTROLLER_FD_ABORT_SEND,
    CAN_IOCTL_ID_CONTROLLER_FD_ABORT_RECEIVE,
    CAN_IOCTL_ID_CONTROLLER_RECEIVE_FIFO_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_RECEIVE_FIFO_NON_BLOCKING,
    CAN_IOCTL_ID_CONTROLLER_GET_TIMESTAMP,
    CAN_IOCTL_ID_CONTROLLER_ABORT_SEND,
    CAN_IOCTL_ID_CONTROLLER_ABORT_RECEIVE,
    CAN_IOCTL_ID_CONTROLLER_ABORT_RECEIVE_FIFO,
    /* Helper IOCTLs */
    CAN_IOCTL_ID_HELPER_FLEXCAN_ID_STD,
    CAN_IOCTL_ID_HELPER_FLEXCAN_RX_MB_STD_MASK,
};


/** CAN_CONTROLLER_INFO struct is versioned by its byte size including the Size field. */
typedef struct _CAN_CONTROLLER_INFO {
    /* Info size - must be set to sizeof(CAN_CONTROLLER_INFO) */
    _Field_range_(== , sizeof(CAN_CONTROLLER_INFO))
    SIZE_T Size;

    ULONG tmp;
    ULONG PinCount;
    BOOLEAN             IsInitialized;
    LONG                Sequence;
    LONG                RequestCnt;
} CAN_CONTROLLER_INFO;

/**
 * IOCTL_CAN_CONTROLLER_GET_INFO
 *
 * This function returns CAN_CONTROLLER_INFO structure.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_GET_INFO,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_GET_INFO_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_GET_INFO_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_GET_INFO_OUTPUT A pointer to output parameters structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_GET_INFO \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_GET_INFO, \
                METHOD_BUFFERED, \
                FILE_ANY_ACCESS)

typedef CAN_CONTROLLER_INFO CAN_CONTROLLER_GET_INFO_OUTPUT;

/** FlexCAN protocol timing characteristic configuration structure. */
typedef struct _flexcan_timing_config
{
    UINT16 preDivider;  /* Classic CAN or CAN FD nominal phase bit rate prescaler. */
    UINT8 rJumpwidth;   /* Classic CAN or CAN FD nominal phase Re-sync Jump Width. */
    UINT8 phaseSeg1;    /* Classic CAN or CAN FD nominal phase Segment 1. */
    UINT8 phaseSeg2;    /* Classic CAN or CAN FD nominal phase Segment 2. */
    UINT8 propSeg;      /* Classic CAN or CAN FD nominal phase Propagation Segment. */
    UINT16 fpreDivider; /* CAN FD data phase bit rate prescaler. */
    UINT8 frJumpwidth;  /* CAN FD data phase Re-sync Jump Width. */
    UINT8 fphaseSeg1;   /* CAN FD data phase Phase Segment 1. */
    UINT8 fphaseSeg2;   /* CAN FD data phase Phase Segment 2. */
    UINT8 fpropSeg;     /* CAN FD data phase Propagation Segment. */
} flexcan_timing_config_t;

/** FlexCAN clock source.
 * Do not use the kFLEXCAN_ClkSrcOs.It has been superceded kFLEXCAN_ClkSrc0
 * Do not use the kFLEXCAN_ClkSrcPeri.It has been superceded kFLEXCAN_ClkSrc1
 */
typedef enum _flexcan_clock_source
{
    kFLEXCAN_ClkSrcOsc = 0x0U,  /*!< FlexCAN Protocol Engine clock from Oscillator. */
    kFLEXCAN_ClkSrcPeri = 0x1U, /*!< FlexCAN Protocol Engine clock from Peripheral Clock. */
    kFLEXCAN_ClkSrc0 = 0x0U,    /*!< FlexCAN Protocol Engine clock selected by user as SRC == 0. */
    kFLEXCAN_ClkSrc1 = 0x1U,    /*!< FlexCAN Protocol Engine clock selected by user as SRC == 1. */
} flexcan_clock_source_t;

/** FlexCAN wake up source. */
typedef enum _flexcan_wake_up_source
{
    kFLEXCAN_WakeupSrcUnfiltered = 0x0U,    /*!< FlexCAN uses unfiltered Rx input to detect edge. */
    kFLEXCAN_WakeupSrcFiltered = 0x1U,      /*!< FlexCAN uses filtered Rx input to detect edge. */
} flexcan_wake_up_source_t;

/** FlexCAN module configuration structure. */
typedef struct _flexcan_config
{
    UINT32 bitRate;                     /* FlexCAN bit rate in bps, for classical CAN or CANFD nominal phase. */
    UINT32 bitRateFD;                   /* FlexCAN FD bit rate in bps, for CANFD data phase. */
    flexcan_clock_source_t clkSrc;      /* Clock source for FlexCAN Protocol Engine. */
    flexcan_wake_up_source_t wakeupSrc; /* Wake up source selection. */
    UINT8 maxMbNum;                     /* The maximum number of Message Buffers used by user. */
    BOOLEAN enableLoopBack;             /* Enable or Disable Loop Back Self Test Mode. */
    BOOLEAN enableTimerSync;            /* Enable or Disable Timer Synchronization. */
    BOOLEAN enableSelfWakeup;           /* Enable or Disable Self Wakeup Mode. */
    BOOLEAN enableIndividMask;          /* Enable or Disable Rx Individual Mask and Queue feature. */
    BOOLEAN disableSelfReception;       /* Enable or Disable Self Reflection. */
    BOOLEAN enableListenOnlyMode;       /* Enable or Disable Listen Only Mode. */
    BOOLEAN enableDoze;                 /* Enable or Disable Doze Mode. */
    BOOLEAN enablePretendedeNetworking; /* Enable or Disable the Pretended Networking mode. */
    BOOLEAN enableMemoryErrorControl;   /* Enable or Disable the memory errors detection and correction mechanism. */
    BOOLEAN enableNonCorrectableErrorEnterFreeze; /* Enable or Disable Non-Correctable Errors In FlexCAN Access Put Device In Freeze Mode. */
    BOOLEAN enableTransceiverDelayMeasure;        /* Enable or Disable the transceiver delay measurement, when it is enabled, then the secondary */
                                                  /* sample point position is determined by the sum of the transceiver delay measurement plus the enhanced TDC offset. */
    flexcan_timing_config_t timingConfig;         /* Protocol timing */
} flexcan_config_t;

/**
 * IOCTL_CAN_CONTROLLER_GET_CONFIG
 *
 * This function initializes the FlexCAN configuration structure to default values and the copy of this structure
 * returns. The default values are as follows.
 * flexcanConfig->clkSrc  = kFLEXCAN_ClkSrc0;
 * flexcanConfig->bitRate  = 1000000U;
 * flexcanConfig->bitRateFD  = 2000000U;
 * flexcanConfig->maxMbNum  = 16;
 * flexcanConfig->enableLoopBack  = false;
 * flexcanConfig->enableSelfWakeup  = false;
 * flexcanConfig->enableIndividMask  = false;
 * flexcanConfig->disableSelfReception = false;
 * flexcanConfig->enableListenOnlyMode = false;
 * flexcanConfig->enableDoze = false;
 * flexcanConfig->enableMemoryErrorControl = true;
 * flexcanConfig->enableNonCorrectableErrorEnterFreeze = true;
 * flexcanConfig.timingConfig = timingConfig;
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_GET_CONFIG,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_GET_CONFIG_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_GET_CONFIG_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_GET_CONFIG_OUTPUT Pointer to the returned FlexCAN configuration structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_GET_CONFIG \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_GET_CONFIG, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_config_t CAN_CONTROLLER_GET_CONFIG_OUTPUT;

/**
 * IOCTL_CAN_CONTROLLER_SET_CLKSRC
 *
 * This function sets the ClkSrc in the FlexCAN configuration structure.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_CLKSRC,
 *                  &CAN_CONTROLLER_SET_CLKSRC_INTPUT,
 *                  sizeof(CAN_CONTROLLER_SET_CLKSRC_INTPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SET_CLKSRC_INTPUT Pointer to the clock source.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_CLKSRC \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_CLKSRC, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef flexcan_clock_source_t CAN_CONTROLLER_SET_CLKSRC_INTPUT;

/**
 * IOCTL_CAN_CONTROLLER_SET_BITRATE
 *
 * This function sets the bitRate in the FlexCAN configuration structure.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_BITRATE,
 *                  &CAN_CONTROLLER_SET_BITRATE_INTPUT,
 *                  sizeof(CAN_CONTROLLER_SET_BITRATE_INTPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SET_BITRATE_INTPUT Pointer to the bit rate in Hz.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_BITRATE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_BITRATE, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef UINT32 CAN_CONTROLLER_SET_BITRATE_INTPUT;

/**
 * IOCTL_CAN_CONTROLLER_SET_LOOPBACK
 *
 * This function sets the enableLoopback in the FlexCAN configuration structure.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_LOOPBACK,
 *                  &CAN_CONTROLLER_BOOLEAN_INPUT,
 *                  sizeof(CAN_CONTROLLER_BOOLEAN_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_BOOLEAN_INPUT Pointer to the enable loopback.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_LOOPBACK \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_LOOPBACK, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_SET_SELFWAKEUP
 *
 * This function sets the enableSelfWakeup in the FlexCAN configuration structure.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_SELFWAKEUP,
 *                  &CAN_CONTROLLER_BOOLEAN_INPUT,
 *                  sizeof(CAN_CONTROLLER_BOOLEAN_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_BOOLEAN_INPUT Pointer to the enable Self Wakeup.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_SELFWAKEUP \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_SELFWAKEUP, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_SET_LISTEN_ONLY_MODE
 *
 * This function sets the enableListenOnlyMode in the FlexCAN configuration structure.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_LISTEN_ONLY_MODE,
 *                  &CAN_CONTROLLER_BOOLEAN_INPUT,
 *                  sizeof(CAN_CONTROLLER_BOOLEAN_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_BOOLEAN_INPUT Pointer to the enable Listen Only Mode.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_LISTEN_ONLY_MODE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_LISTEN_ONLY_MODE, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_SET_DOZE
 *
 * This function sets the enableDoze in the FlexCAN configuration structure.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_DOZE,
 *                  &CAN_CONTROLLER_BOOLEAN_INPUT,
 *                  sizeof(CAN_CONTROLLER_BOOLEAN_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_BOOLEAN_INPUT Pointer to the enable doze mode.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_DOZE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_DOZE, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

 /** FlexCAN CalculateImprovedTimingValues output parameters structure. */
typedef struct _flexcan_improved_timing_return
{
    flexcan_timing_config_t     tcfg;   /* FlexCAN protocol timing characteristic configuration structure. */
    status_t                    status; /* Returned status of FlexCAN_CalculateImprovedTimingValues() */
} flexcan_improved_timing_return_t;

/**
 * IOCTL_CAN_CONTROLLER_IMPROVED_TIMING
 *
 * This function use to calculates the Classical CAN timing values according to the given bit rate flexcanConfig.bitRate.
 * The Calculated timing values will be set in CTRL1/CBT/ENCBT register. The calculation is based on the recommendation
 * of the CiA 301 v4.2.0 and previous version document. Timing values are stored internaly in timingConfig and the copy of
 * this structure is returned in CAN_CONTROLLER_IMPROVED_TIMING_OUTPUT.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_IMPROVED_TIMING,
 *                  &CAN_CONTROLLER_SOURCE_CLOCK_INPUT,
 *                  sizeof(CAN_CONTROLLER_SOURCE_CLOCK_INPUT),
 *                  &CAN_CONTROLLER_IMPROVED_TIMING_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_IMPROVED_TIMING_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SOURCE_CLOCK_INPUT Pointer to the source clock frequency in Hz.
 * @param CAN_CONTROLLER_IMPROVED_TIMING_OUTPUT Pointer to the FlexCAN timing configuration structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_IMPROVED_TIMING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_IMPROVED_TIMING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef UINT32 CAN_CONTROLLER_SOURCE_CLOCK_INPUT;
typedef flexcan_improved_timing_return_t CAN_CONTROLLER_IMPROVED_TIMING_OUTPUT;

/** FlexCAN FDCalculateImprovedTimingValues output parameters structure. */
typedef struct _flexcan_fd_improved_timing_return
{
    flexcan_timing_config_t     tcfg;   /* FlexCAN protocol timing characteristic configuration structure. */
    status_t                    status; /* Returned status of FlexCAN_FDCalculateImprovedTimingValues() */
} flexcan_fd_improved_timing_return_t;

/**
 * IOCTL_CAN_CONTROLLER_FD_IMPROVED_TIMING
 *
 * This function use to calculates the CAN FD timing values according to the given nominal phase bit rate
 * (flexcanConfig.bitRate) and data phase bit rate (flexcanConfig.bitRateFD). The Calculated timing values will be set
 * in CBT/ENCBT and FDCBT/EDCBT registers. The calculation is based on the recommendation of the CiA 1301 v1.0.0
 * document. Timing values are stored internaly in timingConfig and the copy of this structure is returned in
 * CAN_CONTROLLER_FD_IMPROVED_TIMING_OUTPUT.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_FD_IMPROVED_TIMING,
 *                  &CAN_CONTROLLER_SOURCE_CLOCK_INPUT,
 *                  sizeof(CAN_CONTROLLER_SOURCE_CLOCK_INPUT),
 *                  &CAN_CONTROLLER_FD_IMPROVED_TIMING_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_FD_IMPROVED_TIMING_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SOURCE_CLOCK_INPUT Pointer to the source clock frequency in Hz.
 * @param CAN_CONTROLLER_FD_IMPROVED_TIMING_OUTPUT Pointer to the FlexCAN timing configuration structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_FD_IMPROVED_TIMING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_FD_IMPROVED_TIMING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_fd_improved_timing_return_t CAN_CONTROLLER_FD_IMPROVED_TIMING_OUTPUT;

/**
 * IOCTL_CAN_CONTROLLER_INIT
 *
 * This function initializes the FlexCAN module with user-defined settings.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_INIT,
 *                  &CAN_CONTROLLER_SOURCE_CLOCK_INPUT,
 *                  sizeof(CAN_CONTROLLER_SOURCE_CLOCK_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SOURCE_CLOCK_INPUT Pointer to the FlexCAN Protocol Engine clock source frequency in Hz.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_INIT \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_INIT, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)


/** FlexCAN Message Buffer Payload size. */
typedef enum _flexcan_mb_size
{
    kFLEXCAN_8BperMB = 0x0U,        /*!< Selects 8 bytes per Message Buffer. */
    kFLEXCAN_16BperMB = 0x1U,       /*!< Selects 16 bytes per Message Buffer. */
    kFLEXCAN_32BperMB = 0x2U,       /*!< Selects 32 bytes per Message Buffer. */
    kFLEXCAN_64BperMB = 0x3U,       /*!< Selects 64 bytes per Message Buffer. */
} flexcan_mb_size_t;

/** FlexCAN Init parameters structure. */
typedef struct _flexcan_fdinit
{
    UINT32              frequency;  /* FlexCAN Protocol Engine clock source frequency in Hz. */
    flexcan_mb_size_t   mbsize;     /* FlexCAN Message Buffer payload size */
    BOOLEAN             brs;        /* TRUE if bit rate switch is enabled in FD mode */
} flexcan_fdinit_t;

/**
 * IOCTL_CAN_CONTROLLER_FD_INIT
 *
 * This function initializes the FlexCAN module with user-defined settings.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_FD_INIT,
 *                  &CAN_CONTROLLER_SET_FDINIT_INTPUT,
 *                  sizeof(IOCTL_CAN_CONTROLLER_FD_INIT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SET_FDINIT_INTPUT Pointer to the FD init structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_FD_INIT \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_FD_INIT, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef flexcan_fdinit_t CAN_CONTROLLER_SET_FDINIT_INTPUT;

/**
 * IOCTL_CAN_CONTROLLER_DEINIT
 *
 * This function disables the FlexCAN module clock and sets all register values to the reset value.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_DEINIT,
 *                  NULL,
 *                  0,
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_DEINIT \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_DEINIT, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/** FlexCAN frame format. */
typedef enum _flexcan_frame_format
{
    kFLEXCAN_FrameFormatStandard = 0x0U, /*!< Standard frame format attribute. */
    kFLEXCAN_FrameFormatExtend = 0x1U,   /*!< Extend frame format attribute. */
} flexcan_frame_format_t;

/** FlexCAN frame type. */
typedef enum _flexcan_frame_type
{
    kFLEXCAN_FrameTypeData = 0x0U,      /*!< Data frame type attribute. */
    kFLEXCAN_FrameTypeRemote = 0x1U,    /*!< Remote frame type attribute. */
} flexcan_frame_type_t;

/**
 * FlexCAN Receive Message Buffer configuration structure
 *
 * This structure is used as the parameter of FLEXCAN_SetRxMbConfig() function. The FLEXCAN_SetRxMbConfig() function is used to configure FlexCAN Receive
 * Message Buffer. The function abort previous receiving process, clean the Message Buffer and activate the Rx Message Buffer using given Message Buffer setting.
 */
typedef struct _flexcan_rx_mb_config
{
    UINT32                  id;         /* CAN Message Buffer Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
    flexcan_frame_format_t  format;     /* CAN Frame Identifier format(Standard of Extend). */
    flexcan_frame_type_t    type;       /* CAN Frame Type(Data or Remote). */
} flexcan_rx_mb_config_t;

/** FlexCAN SetRxMBConfig parameters structure. */
typedef struct _flexcan_set_rxmb_config
{
    UINT8                   mbidx;      /* The Message Buffer index */
    flexcan_rx_mb_config_t  mbConfig;   /* FlexCAN Message Buffer configuration structure */
    BOOLEAN                 enable;     /* Enable/disable Rx Message Buffer */
} flexcan_set_rxmb_config_t;

/**
 * IOCTL_CAN_CONTROLLER_SET_RXMB_CONFIG
 *
 * This function cleans a FlexCAN build-in Message Buffer and configures it as a Receive Message Buffer.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_RXMB_CONFIG,
 *                  &CAN_CONTROLLER_SET_RXMB_CONFIG,
 *                  sizeof(CAN_CONTROLLER_SET_RXMB_CONFIG),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SET_RXMB_CONFIG A pointer to parameters structure
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_RXMB_CONFIG \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_RXMB_CONFIG, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef flexcan_set_rxmb_config_t CAN_CONTROLLER_SET_RXMB_CONFIG;

/**
 * IOCTL_CAN_CONTROLLER_SET_FD_RXMB_CONFIG
 *
 * This function cleans a FlexCAN build-in Message Buffer and configures it as a Receive Message Buffer.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_FD_RXMB_CONFIG,
 *                  &CAN_CONTROLLER_SET_RXMB_CONFIG,
 *                  sizeof(CAN_CONTROLLER_SET_RXMB_CONFIG),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SET_RXMB_CONFIG A pointer to parameters structure
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_FD_RXMB_CONFIG \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_FD_RXMB_CONFIG, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/** FlexCAN SetTxMBConfig parameters structure. */
typedef struct _flexcan_set_txmb_config
{
    UINT8                   mbidx;      /* The Message Buffer index */
    BOOLEAN                 enable;     /* Enable/disable Tx Message Buffer */
} flexcan_set_txmb_config_t;

/**
 * IOCTL_CAN_CONTROLLER_SET_TXMB_CONFIG
 *
 * This function aborts the previous transmission, cleans the Message Buffer, and configures it as a Transmit Message Buffer.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_TXMB_CONFIG,
 *                  &CAN_CONTROLLER_SET_TXMB_CONFIG,
 *                  sizeof(CAN_CONTROLLER_SET_TXMB_CONFIG),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SET_TXMB_CONFIG A pointer to parameters structure
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_TXMB_CONFIG \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_TXMB_CONFIG, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef flexcan_set_txmb_config_t CAN_CONTROLLER_SET_TXMB_CONFIG;

/**
 * IOCTL_CAN_CONTROLLER_SET_FD_TXMB_CONFIG
 *
 * This function aborts the previous transmission, cleans the Message Buffer, and configures it as a Transmit Message Buffer.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_FD_TXMB_CONFIG,
 *                  &CAN_CONTROLLER_SET_TXMB_CONFIG,
 *                  sizeof(CAN_CONTROLLER_SET_TXMB_CONFIG),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SET_TXMB_CONFIG A pointer to parameters structure
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_FD_TXMB_CONFIG \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_FD_TXMB_CONFIG, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/** FlexCAN message frame structure. */
typedef struct _flexcan_frame
{
    struct
    {
        UINT32 timestamp : 16; /* FlexCAN internal Free-Running Counter Time Stamp. */
        UINT32 length : 4;     /* CAN frame data length in bytes (Range: 0~8). */
        UINT32 type : 1;       /* CAN Frame Type(DATA or REMOTE). */
        UINT32 format : 1;     /* CAN Frame Identifier(STD or EXT format). */
        UINT32 : 1;            /* Reserved. */
        UINT32 idhit : 9;      /* CAN Rx FIFO filter hit id(This value is only used in Rx FIFO receive mode). */
    } mfs_0;
    struct
    {
        UINT32 id : 29;        /* CAN Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
        UINT32 : 3;            /* Reserved. */
    } mfs_1;
    union
    {
        struct
        {
            UINT32 dataWord0;  /* CAN Frame payload word0. */
            UINT32 dataWord1;  /* CAN Frame payload word1. */
        } mfp_w;
        struct
        {
            UINT8 dataByte3;   /* CAN Frame payload byte3. */
            UINT8 dataByte2;   /* CAN Frame payload byte2. */
            UINT8 dataByte1;   /* CAN Frame payload byte1. */
            UINT8 dataByte0;   /* CAN Frame payload byte0. */
            UINT8 dataByte7;   /* CAN Frame payload byte7. */
            UINT8 dataByte6;   /* CAN Frame payload byte6. */
            UINT8 dataByte5;   /* CAN Frame payload byte5. */
            UINT8 dataByte4;   /* CAN Frame payload byte4. */
        } mfp_b;
    } mfp;
} flexcan_frame_t;

/**
 * CAN FD message frame structure.
 *
 * The CAN FD message supporting up to sixty four bytes can be used for a data frame, depending on the length selected for the message buffers.
 * The length should be a enumeration member, see @ref _flexcan_fd_frame_length.
 */
typedef struct _flexcan_fd_frame
{
    struct
    {
        UINT32 timestamp : 16;  /* FlexCAN internal Free-Running Counter Time Stamp. */
        UINT32 length : 4;      /* CAN FD frame data length code (DLC), range see @ref _flexcan_fd_frame_length, When the length <= 8, it equal to the data length, otherwise the number */
                                /* of valid frame data is not equal to the length value.  user can use DLC_LENGTH_DECODE(length) macro to get the number of valid data bytes. */
        UINT32 type : 1;        /* CAN Frame Type(DATA or REMOTE). */
        UINT32 format : 1;      /* CAN Frame Identifier(STD or EXT format). */
        UINT32 srr : 1;         /* Substitute Remote request. */
        UINT32 : 6;
        UINT32 esi : 1;         /* Error State Indicator. */
        UINT32 brs : 1;         /* Bit Rate Switch. */
        UINT32 edl : 1;         /* Extended Data Length. */
    } fdmfs_0;
    struct
    {
        UINT32 id : 29;         /* CAN Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
        UINT32 : 3;             /* Reserved. */
    } fdmfs_1;
    union
    {
        struct
        {
            UINT32 dataWord[16]; /* CAN FD Frame payload, 16 double word maximum. */
        } fdmfp_w;
        /* Note: the maximum databyte* below is actually 64, user can add them if needed, or just use dataWord[*] instead. */
        struct
        {
            UINT8 dataByte3;    /* CAN Frame payload byte3. */
            UINT8 dataByte2;    /* CAN Frame payload byte2. */
            UINT8 dataByte1;    /* CAN Frame payload byte1. */
            UINT8 dataByte0;    /* CAN Frame payload byte0. */
            UINT8 dataByte7;    /* CAN Frame payload byte7. */
            UINT8 dataByte6;    /* CAN Frame payload byte6. */
            UINT8 dataByte5;    /* CAN Frame payload byte5. */
            UINT8 dataByte4;    /* CAN Frame payload byte4. */
        } fdmfp_b;
    } fdmfp;
    /* Note: ID HIT offset is changed dynamically according to data length code (DLC), when DLC is 15, they will be located below. */
    /* Using FLEXCAN_FixEnhancedRxFifoFrameIdHit API is recommended to ensure this idhit value is correct. */
    UINT32 idhit;               /* CAN Enhanced Rx FIFO filter hit id (This value is only used in Enhanced Rx FIFO receive mode). */
} flexcan_fd_frame_t;

/**
 * IOCTL_CAN_CONTROLLER_CREATE_HANDLE
 *
 * This function initializes the FlexCAN handle, which can be used for other FlexCAN transactional APIs.
 * Usually, for a specified FlexCAN instance, call this API once to get the initialized handle.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_CREATE_HANDLE,
 *                  NULL,
 *                  0,
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_CREATE_HANDLE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_CREATE_HANDLE, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

/** FlexCAN TransferReceive input parameter structure */
typedef struct _flexcan_mb_ioctl
{
    flexcan_fd_frame_t  framefd;
    flexcan_frame_t     frame;      /* The buffer of CAN Message to be transfer. */
    UINT8               mbIdx;      /* The index of Message buffer used to transfer Message. */
} flexcan_mb_ioctl_t;

/**
 * IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns right away. When the
 * message has been received, the receive callback function is called.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING,
 *                  &CAN_CONTROLLER_NB_RECEIVE_INPUT,
 *                  sizeof(CAN_CONTROLLER_NB_RECEIVE_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_NB_RECEIVE_INPUT A pointer to FlexCAN TransferReceive input parameter structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT  kStatus_Success  - Start Rx Message Buffer receiving process successfully.
 *                                      kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_RECEIVE_NON_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_mb_ioctl_t CAN_CONTROLLER_NB_RECEIVE_INPUT;
typedef flexcan_frame_t CAN_CONTROLLER_FRAME;
typedef flexcan_fd_frame_t CAN_CONTROLLER_FD_FRAME;

/**
 * IOCTL_CAN_CONTROLLER_FD_RECEIVE_NON_BLOCKING
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns right away. When the
 * message has been received, the receive callback function is called.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_FD_RECEIVE_NON_BLOCKING,
 *                  &CAN_CONTROLLER_NB_RECEIVE_INPUT,
 *                  sizeof(CAN_CONTROLLER_NB_RECEIVE_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_NB_RECEIVE_INPUT A pointer to FlexCAN TransferReceive input parameter structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT  kStatus_Success  - Start Rx Message Buffer receiving process successfully.
 *                                      kStatus_FLEXCAN_RxBusy - Rx Message Buffer is in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_FD_RECEIVE_NON_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_FD_RECEIVE_NON_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns right away. When messages
 * have been sent out, the send callback function is called.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING,
 *                  &CAN_CONTROLLER_NB_SEND_INPUT,
 *                  sizeof(CAN_CONTROLLER_NB_SEND_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_NB_SEND_INPUT Pointer to input parameters structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT  kStatus_Success - Start Tx Message Buffer sending process successfully.
 *                                      kStatus_Fail -  Write Tx Message Buffer failed.
 *                                      kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SEND_NON_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_mb_ioctl_t CAN_CONTROLLER_NB_SEND_INPUT;

/**
 * IOCTL_CAN_CONTROLLER_FD_SEND_NON_BLOCKING
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns right away. When messages
 * have been sent out, the send callback function is called.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_FD_SEND_NON_BLOCKING,
 *                  &CAN_CONTROLLER_NB_SEND_INPUT,
 *                  sizeof(CAN_CONTROLLER_NB_SEND_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_NB_SEND_INPUT Pointer to input parameters structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT  kStatus_Success - Start Tx Message Buffer sending process successfully.
 *                                      kStatus_Fail -  Write Tx Message Buffer failed.
 *                                      kStatus_FLEXCAN_TxBusy Tx Message Buffer is in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_FD_SEND_NON_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_FD_SEND_NON_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

/** FlexCAN TransferReceiveBlocking output parameters structure. */
typedef struct _flexcan_return
{
    flexcan_frame_t         RxFrame;    /* Received CAN message frame */
    status_t                status;     /* Returned status of  FLEXCAN_TransferSendBlocking() */
} flexcan_return_t;

/**
 * IOCTL_CAN_CONTROLLER_RECEIVE_BLOCKING
 *
 * Performs a polling receive transaction on the CAN bus. Note:  A transfer handle does not need to be created before
 * calling this API.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_RECEIVE_BLOCKING,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  &CAN_CONTROLLER_RECEIVE_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_RECEIVE_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT Pointer to the FlexCAN Message Buffer index.
 * @param CAN_CONTROLLER_RECEIVE_OUTPUT Pointer to ReceiveBlocking output parameters structure
 *                                          kStatus_Success  - Rx Message Buffer is full and has been read successfully.
 *                                          kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 *                                          kStatus_Fail  - Rx Message Buffer is empty.
  * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_RECEIVE_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_RECEIVE_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_return_t CAN_CONTROLLER_RECEIVE_OUTPUT;

/** FlexCAN TransferSendBlocking parameters structure. */
typedef struct _flexcan_send
{
    UINT8                   mbidx;      /* The Message Buffer index */
    flexcan_frame_t         TxFrame;    /* CAN message frame to be sent */
} flexcan_send_param_t;

/**
 * IOCTL_CAN_CONTROLLER_SEND_BLOCKING
 *
 * Performs a polling send transaction on the CAN bus. Note:  A transfer handle does not need to be created before
 * calling this API.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SEND_BLOCKING,
 *                  &CAN_CONTROLLER_SEND_INPUT,
 *                  sizeof(CAN_CONTROLLER_SEND_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SEND_INPUT Pointer to SendBlocking parameters structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT 	kStatus_Success - Write Tx Message Buffer Successfully.
 *                                      kStatus_Fail    - Tx Message Buffer is currently in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SEND_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SEND_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_send_param_t CAN_CONTROLLER_SEND_INPUT;

/** FlexCAN TransferFDSendBlocking parameters structure. */
typedef struct _flexcan_fd_send
{
    UINT8                   mbidx;      /* The Message Buffer index */
    flexcan_fd_frame_t      TxFrame;    /* CAN message FD frame to be sent */
} flexcan_fd_send_param_t;

/**
 * IOCTL_CAN_CONTROLLER_FD_SEND_BLOCKING
 *
 * Performs a polling send transaction on the CAN bus. Note:  A transfer handle does not need to be created before
 * calling this API.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_FD_SEND_BLOCKING,
 *                  &CAN_CONTROLLER_FD_SEND_INPUT,
 *                  sizeof(CAN_CONTROLLER_FD_SEND_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_FD_SEND_INPUT Pointer to FDSendBlocking parameters structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT 	kStatus_Success - Write Tx Message Buffer Successfully.
 *                                      kStatus_Fail    - Tx Message Buffer is currently in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_FD_SEND_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_FD_SEND_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_fd_send_param_t CAN_CONTROLLER_FD_SEND_INPUT;

/**
 * IOCTL_CAN_CONTROLLER_FD_RECEIVE_BLOCKING
 *
 * Performs a polling receive transaction on the CAN bus. Note:  A transfer handle does not need to be created before
 * calling this API.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_FD_RECEIVE_BLOCKING,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  &CAN_CONTROLLER_FD_RECEIVE_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_FD_RECEIVE_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT A pointer to the FlexCAN FD Message Buffer index.
 * @param CAN_CONTROLLER_FD_RECEIVE_OUTPUT A pointer to ReadFDRxMb output parameters structure.
 *                                              kStatus_Success - Rx Message Buffer is full and has been read successfully.
 *                                              kStatus_FLEXCAN_RxOverflow - Rx Message Buffer is already overflowed and has been read successfully.
 *                                              kStatus_Fail - Rx Message Buffer is empty.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_FD_RECEIVE_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_FD_RECEIVE_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

#define kStatusGroup_Generic 0
#define kStatusGroup_FLEXCAN 53

/** Construct a status code value from a group and code number. */
#define MAKE_STATUS(group, code) ((((group)*100) + (code)))


/** Generic status return codes. */
enum
{
    kStatus_Success = MAKE_STATUS(kStatusGroup_Generic, 0),                 /*!< Generic status for Success. */
    kStatus_Fail = MAKE_STATUS(kStatusGroup_Generic, 1),                    /*!< Generic status for Fail. */
    kStatus_ReadOnly = MAKE_STATUS(kStatusGroup_Generic, 2),                /*!< Generic status for read only failure. */
    kStatus_OutOfRange = MAKE_STATUS(kStatusGroup_Generic, 3),              /*!< Generic status for out of range access. */
    kStatus_InvalidArgument = MAKE_STATUS(kStatusGroup_Generic, 4),         /*!< Generic status for invalid argument check. */
    kStatus_Timeout = MAKE_STATUS(kStatusGroup_Generic, 5),                 /*!< Generic status for timeout. */
    kStatus_NoTransferInProgress = MAKE_STATUS(kStatusGroup_Generic, 6),    /*!< Generic status for no transfer in progress. */
    kStatus_Busy = MAKE_STATUS(kStatusGroup_Generic, 7),                    /*!< Generic status for module is busy. */
};


/** FlexCAN transfer status. */
enum
{
    kStatus_FLEXCAN_TxBusy = MAKE_STATUS(kStatusGroup_FLEXCAN, 0),          /*!< Tx Message Buffer is Busy. */
    kStatus_FLEXCAN_TxIdle = MAKE_STATUS(kStatusGroup_FLEXCAN, 1),          /*!< Tx Message Buffer is Idle. */
    kStatus_FLEXCAN_TxSwitchToRx = MAKE_STATUS(kStatusGroup_FLEXCAN, 2),    /*!< Remote Message is send out and Message buffer changed to Receive one. */
    kStatus_FLEXCAN_RxBusy = MAKE_STATUS(kStatusGroup_FLEXCAN, 3),          /*!< Rx Message Buffer is Busy. */
    kStatus_FLEXCAN_RxIdle = MAKE_STATUS(kStatusGroup_FLEXCAN, 4),          /*!< Rx Message Buffer is Idle. */
    kStatus_FLEXCAN_RxOverflow = MAKE_STATUS(kStatusGroup_FLEXCAN, 5),      /*!< Rx Message Buffer is Overflowed. */
    kStatus_FLEXCAN_RxFifoBusy = MAKE_STATUS(kStatusGroup_FLEXCAN, 6),      /*!< Rx Message FIFO is Busy. */
    kStatus_FLEXCAN_RxFifoIdle = MAKE_STATUS(kStatusGroup_FLEXCAN, 7),      /*!< Rx Message FIFO is Idle. */
    kStatus_FLEXCAN_RxFifoOverflow = MAKE_STATUS(kStatusGroup_FLEXCAN, 8),  /*!< Rx Message FIFO is overflowed. */
    kStatus_FLEXCAN_RxFifoWarning = MAKE_STATUS(kStatusGroup_FLEXCAN, 9),   /*!< Rx Message FIFO is almost overflowed. */
    kStatus_FLEXCAN_ErrorStatus = MAKE_STATUS(kStatusGroup_FLEXCAN, 10),    /*!< FlexCAN Module Error and Status. */
    kStatus_FLEXCAN_WakeUp = MAKE_STATUS(kStatusGroup_FLEXCAN, 11),         /*!< FlexCAN is waken up from STOP mode. */
    kStatus_FLEXCAN_UnHandled = MAKE_STATUS(kStatusGroup_FLEXCAN, 12),      /*!< UnHadled Interrupt asserted. */
    kStatus_FLEXCAN_RxRemote = MAKE_STATUS(kStatusGroup_FLEXCAN, 13),       /*!< Rx Remote Message Received in Mail box. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO)
    kStatus_FLEXCAN_RxFifoUnderflow =
    MAKE_STATUS(kStatusGroup_FLEXCAN, 14),                                  /*!< Enhanced Rx Message FIFO is underflow. */
#endif
};

/** Notification structure */
typedef struct _data_callback
{
    flexcan_frame_t frame;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    flexcan_fd_frame_t FDframe;
#endif
    status_t status;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    UINT64 result;
#else
    UINT32 result;
#endif
} data_callback;

/**
 * IOCTL_CAN_INVERT_NOTIFICATION
 *
 * This function handles the FlexCAN Error, the Message Buffer, and the Rx FIFO IRQ request.
 *
 * DeviceIoControl(
 *                  hPort,
 *                  IOCTL_CAN_INVERT_NOTIFICATION,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_NOTIFICATION_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_NOTIFICATION_OUTPUT),
 *                  &returned,
 *                  &OVERLAPPED
 *                  );
 *
 * @param hPort Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_NOTIFICATION_OUTPUT A pointer to output parameters structure.
 * @param OVERLAPPED Pointer to Overlapped structure
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_INVERT_NOTIFICATION \
	        CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_INVERT_NOTIFICATION, \
                METHOD_BUFFERED, \
                FILE_ANY_ACCESS)

typedef data_callback CAN_CONTROLLER_NOTIFICATION_OUTPUT;

/**
 * IOCTL_CAN_CONTROLLER_ENTER_FREEZEMODE
 *
 * This function makes the FlexCAN work under Freeze Mode.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_ENTER_FREEZEMODE,
 *                  NULL,
 *                  0,
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_ENTER_FREEZEMODE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_ENTER_FREEZEMODE, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)


/**
 * IOCTL_CAN_CONTROLLER_EXIT_FREEZEMODE
 *
 * This function makes the FlexCAN leave Freeze Mode.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_EXIT_FREEZEMODE,
 *                  NULL,
 *                  0,
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_EXIT_FREEZEMODE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_EXIT_FREEZEMODE, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_SET_TIMING_CONFIG
 *
 * This function gives user settings to classical CAN or CANFD nominal phase timing characteristic. The function is for
 * an experienced user. For less experienced users, call the IOCTL_CAN_CONTROLLER_GET_CONFIG and get the default timing
 * characteristicsthe, then call IOCTL_CAN_CONTROLLER_INIT and fill the bit rate field.
 * Note: Calling IOCTL_CAN_CONTROLLER_SET_TIMING_CONFIG overrides the bit rate set in IOCTL_CAN_CONTROLLER_INIT.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_TIMING_CONFIG,
 *                  &CAN_CONTROLLER_TIMING_INPUT,
 *                  sizeof(CAN_CONTROLLER_TIMING_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_TIMING_INPUT A pointer to the timing configuration structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_TIMING_CONFIG \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_TIMING_CONFIG, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef flexcan_timing_config_t CAN_CONTROLLER_TIMING_INPUT;

/**
 * IOCTL_CAN_CONTROLLER_SET_FDTIMING_CONFIG
 *
 * Sets the FlexCAN CANFD data phase timing characteristic. Note: Calling IOCTL_CAN_CONTROLLER_SET_FDTIMING_CONFIG
 * overrides the bit rate set in IOCTL_CAN_CONTROLLER_FD_INIT.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_FDTIMING_CONFIG,
 *                  &CAN_CONTROLLER_TIMING_INPUT,
 *                  sizeof(CAN_CONTROLLER_TIMING_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_TIMING_INPUT A pointer to the timing configuration structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_FDTIMING_CONFIG \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_FDTIMING_CONFIG, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_SET_RXMB_GLOBAL_MASK
 *
 * This function sets the global mask for the FlexCAN message buffer in a matching process. The configuration is only
 * effective when the Rx individual mask is disabled in the IOCTL_CAN_CONTROLLER_INIT.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_RXMB_GLOBAL_MASK,
 *                  &CAN_CONTROLLER_GLOBAL_MASK_INPUT,
 *                  sizeof(CAN_CONTROLLER_GLOBAL_MASK_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_GLOBAL_MASK_INPUT A pointer to Rx Message Buffer Global Mask value.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_RXMB_GLOBAL_MASK \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_RXMB_GLOBAL_MASK, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef UINT32 CAN_CONTROLLER_GLOBAL_MASK_INPUT;

/**
 * IOCTL_CAN_CONTROLLER_SET_RXFIFO_GLOBAL_MASK
 *
 * This function sets the global mask for FlexCAN FIFO in a matching process.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_RXFIFO_GLOBAL_MASK,
 *                  &CAN_CONTROLLER_GLOBAL_MASK _MASK_INPUT,
 *                  sizeof(CAN_CONTROLLER_GLOBAL_MASK _MASK_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_GLOBAL_MASK_INPUT A pointer to Rx Fifo Global Mask value.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_RXFIFO_GLOBAL_MASK \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_RXFIFO_GLOBAL_MASK, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/* FlexCAN TransferSendBlocking parameters structure. */
typedef struct _flexcan_mask
{
    UINT8       maskIdx;    /* The Index of individual Mask */
    UINT32      mask;       /* Rx Individual Mask value */
} flexcan_mask_param_t;

/**
 * IOCTL_CAN_CONTROLLER_SET_RX_INDIVIDUAL_MASK
 *
 * This function sets the individual mask for the FlexCAN matching process. The configuration is only effective when
 * the Rx individual mask is enabled in the IOCTL_CAN_CONTROLLER_INIT. If the Rx FIFO is disabled, the individual mask
 * is applied to the corresponding Message Buffer. If the Rx FIFO is enabled, the individual mask for Rx FIFO occupied
 * Message Buffer is applied to the Rx Filter with the same index. Note that only the first 32 individual masks can be
 * used as the Rx FIFO filter mask.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_RX_INDIVIDUAL_MASK,
 *                  &CAN_CONTROLLER_ INDIVIDUAL _MASK _MASK_INPUT,
 *                  sizeof(CAN_CONTROLLER_ INDIVIDUAL _MASK _MASK_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_INDIVIDUAL_MASK_INPUT A pointer to individual mask structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_RX_INDIVIDUAL_MASK \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_RX_INDIVIDUAL_MASK, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef flexcan_mask_param_t CAN_CONTROLLER_INDIVIDUAL_MASK_INPUT;

/** FlexCAN Rx Fifo Filter type. */
typedef enum _flexcan_rx_fifo_filter_type
{
    kFLEXCAN_RxFifoFilterTypeA = 0x0U,  /*!< One full ID (standard and extended) per ID Filter element. */
    kFLEXCAN_RxFifoFilterTypeB = 0x1U,  /*!< Two full standard IDs or two partial 14-bit ID slices per ID Filter Table element. */
    kFLEXCAN_RxFifoFilterTypeC = 0x2U,  /*!< Four partial 8-bit Standard or extended ID slices per ID Filter Table element. */
    kFLEXCAN_RxFifoFilterTypeD = 0x3U,  /*!< All frames rejected. */
} flexcan_rx_fifo_filter_type_t;

/** FlexCAN Enhanced/Legacy Rx FIFO priority.
 *
 * The matching process starts from the Rx MB(or Enhanced/Legacy Rx FIFO) with higher priority. If no MB(or Enhanced/Legacy Rx FIFO filter) is satisfied, the matching process goes on with
 * the Enhanced/Legacy Rx FIFO(or Rx MB) with lower priority.
 */
typedef enum _flexcan_rx_fifo_priority
{
    kFLEXCAN_RxFifoPrioLow = 0x0U,      /*!< Matching process start from Rx Message Buffer first. */
    kFLEXCAN_RxFifoPrioHigh = 0x1U,     /*!< Matching process start from Enhanced/Legacy Rx FIFO first. */
} flexcan_rx_fifo_priority_t;

/** FlexCAN Legacy Rx FIFO configuration structure. */
typedef struct _flexcan_rx_fifo_config
{
    UINT32 *idFilterTable;                      /* Pointer to the FlexCAN Legacy Rx FIFO identifier filter table. */
    UINT8 idFilterNum;                          /* The FlexCAN Legacy Rx FIFO Filter elements quantity. */
    flexcan_rx_fifo_filter_type_t idFilterType; /* The FlexCAN Legacy Rx FIFO Filter type. */
    flexcan_rx_fifo_priority_t priority;        /* The FlexCAN Legacy Rx FIFO receive priority. */
} flexcan_rx_fifo_config_t;

/** FlexCAN SetRxFifoConfig parameters structure. */
typedef struct _flexcan_set_rxfifo_config
{
    flexcan_rx_fifo_config_t    RxFifoConfig;   /* FlexCAN Legacy Rx FIFO configuration structure. Can be NULL when enable parameter is false. */
    BOOLEAN                     enable;         /* Enable/disable Legacy Rx FIFO */
} flexcan_set_rxfifo_config_t;

/**
 * IOCTL_CAN_CONTROLLER_SET_RXFIFO_CONFIG
 *
 * This function configures the FlexCAN Rx FIFO with given configuration. Note: Legacy Rx FIFO only can receive classic CAN message.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_SET_RXFIFO_CONFIG,
 *                  &CAN_CONTROLLER_SET_RXFIFO_CONFIG,
 *                  sizeof(CAN_CONTROLLER_SET_RXFIFO_CONFIG),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SET_RXFIFO_CONFIG A pointer to parameters structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_SET_RXFIFO_CONFIG \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_SET_RXFIFO_CONFIG, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef flexcan_set_rxfifo_config_t CAN_CONTROLLER_SET_RXFIFO_CONFIG;

/**
 * IOCTL_CAN_CONTROLLER_GET_STATUS_FLAGS
 *
 * This function gets all FlexCAN status flags. The flags are returned as the logical OR value of the enumerators
 * _flexcan_flags. To check the specific status, compare the return value with enumerators in _flexcan_flags.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_GET_STATUS_FLAGS,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_FLAGS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_FLAGS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_FLAGS_OUTPUT A pointer to FlexCAN status flags which are ORed by the enumerators in the _flexcan_flags.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_GET_STATUS_FLAGS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_GET_STATUS_FLAGS, \
                METHOD_BUFFERED, \
                FILE_READ_DATA)

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
typedef UINT64 CAN_CONTROLLER_FLAGS_OUTPUT;
#else
typedef UINT32 CAN_CONTROLLER_FLAGS_OUTPUT;
#endif

/**
 * IOCTL_CAN_CONTROLLER_CLEAR_STATUS_FLAGS
 *
 * This function clears the FlexCAN status flags with a provided mask. An automatically cleared flag can't be cleared by this function.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_CLEAR_STATUS_FLAGS,
 *                  &CAN_CONTROLLER_FLAGS_INPUT,
 *                  sizeof(CAN_CONTROLLER_FLAGS_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_FLAGS_INPUT A pointer to the status flags to be cleared, it is logical OR value of _flexcan_flags.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_CLEAR_STATUS_FLAGS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_CLEAR_STATUS_FLAGS, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE) ||                   \
    (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_RX_FIFO) || \
    (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
typedef UINT64 CAN_CONTROLLER_FLAGS_INPUT;
#else
typedef UINT32 CAN_CONTROLLER_FLAGS_INPUT;
#endif

/** FLEXCAN_GetBusErrCount parameters structure. */
typedef struct _flexcan_buseer_param_t
{
    UINT8       txErrBuf;   /* Buffer to store Tx Error Counter value */
    UINT8       rxErrBuf;   /* Buffer to store Rx Error Counter value */
} flexcan_buseer_param_t;

/**
 * IOCTL_CAN_CONTROLLER_GET_BUSERR_COUNT
 *
 * This function gets the FlexCAN Bus Error Counter value for both Tx and Rx direction. These values may be needed in the upper layer error handling.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_GET_BUSERR_COUNT,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_BUSERR_OUTPUT;
 *                  sizeof(CAN_CONTROLLER_BUSERR_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_BUSERR_OUTPUT A pointer to parameters structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_GET_BUSERR_COUNT \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_GET_BUSERR_COUNT, \
                METHOD_BUFFERED, \
                FILE_READ_DATA)

typedef flexcan_buseer_param_t CAN_CONTROLLER_BUSERR_OUTPUT;

/**
 * IOCTL_CAN_CONTROLLER_GET_MBSTATUS_FLAGS
 *
 * This function gets the interrupt flags of a given Message Buffers.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_GET_MBSTATUS_FLAGS,
 *                  &CAN_CONTROLLER_MBFLAGS_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBFLAGS_INPUT),
 *                  &CAN_CONTROLLER_MBFLAGS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_MBFLAGS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_FLAGS_INPUT A pointer to mask the ORed FlexCAN Message Buffer mask.
 * @param CAN_CONTROLLER_FLAGS_OUTPUT A pointer to  the status of given Message Buffers.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_GET_MBSTATUS_FLAGS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_GET_MBSTATUS_FLAGS, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
typedef UINT64 CAN_CONTROLLER_MBFLAGS_INPUT;
typedef UINT64 CAN_CONTROLLER_MBFLAGS_OUTPUT;
#else
typedef UINT32 CAN_CONTROLLER_MBFLAGS_INPUT;
typedef UINT32 CAN_CONTROLLER_MBFLAGS_OUTPUT;
#endif

/**
 * IOCTL_CAN_CONTROLLER_CLEAR_MBSTATUS_FLAGS
 *
 * This function clears the interrupt flags of a given Message Buffers.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_CLEAR_MBSTATUS_FLAGS,
 *                  &CAN_CONTROLLER_MBFLAGS_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBFLAGS_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBFLAGS_INPUT A pointer to the ORed FlexCAN Message Buffer mask.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_CLEAR_MBSTATUS_FLAGS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_CLEAR_MBSTATUS_FLAGS, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
/** FlexCAN Memory Error Type. */
typedef enum _flexcan_memory_error_type
{
    kFLEXCAN_CorrectableError = 0U,     /*!< The memory error is correctable which means on bit error. */
    kFLEXCAN_NonCorrectableError        /*!< The memory error is non-correctable which means two bit errors. */
} flexcan_memory_error_type_t;

// FlexCAN Memory Access Type.
typedef enum _flexcan_memory_access_type
{
    kFLEXCAN_MoveOutFlexCanAccess = 0U, /*!< The memory error was detected during move-out FlexCAN access. */
    kFLEXCAN_MoveInAccess,              /*!< The memory error was detected during move-in FlexCAN access. */
    kFLEXCAN_TxArbitrationAccess,       /*!< The memory error was detected during Tx Arbitration FlexCAN access. */
    kFLEXCAN_RxMatchingAccess,          /*!< The memory error was detected during Rx Matching FlexCAN access. */
    kFLEXCAN_MoveOutHostAccess          /*!< The memory error was detected during Rx Matching Host (CPU) access. */
} flexcan_memory_access_type_t;

/** FlexCAN Memory Error Byte Syndrome. */
typedef enum _flexcan_byte_error_syndrome
{
    kFLEXCAN_NoError = 0U,              /*!< No bit error in this byte. */
    kFLEXCAN_ParityBits0Error = 1U,     /*!< Parity bit 0 error in this byte. */
    kFLEXCAN_ParityBits1Error = 2U,     /*!< Parity bit 1 error in this byte. */
    kFLEXCAN_ParityBits2Error = 4U,     /*!< Parity bit 2 error in this byte. */
    kFLEXCAN_ParityBits3Error = 8U,     /*!< Parity bit 3 error in this byte. */
    kFLEXCAN_ParityBits4Error = 16U,    /*!< Parity bit 4 error in this byte. */
    kFLEXCAN_DataBits0Error = 28U,      /*!< Data bit 0 error in this byte. */
    kFLEXCAN_DataBits1Error = 22U,      /*!< Data bit 1 error in this byte. */
    kFLEXCAN_DataBits2Error = 19U,      /*!< Data bit 2 error in this byte. */
    kFLEXCAN_DataBits3Error = 25U,      /*!< Data bit 3 error in this byte. */
    kFLEXCAN_DataBits4Error = 26U,      /*!< Data bit 4 error in this byte. */
    kFLEXCAN_DataBits5Error = 7U,       /*!< Data bit 5 error in this byte. */
    kFLEXCAN_DataBits6Error = 21U,      /*!< Data bit 6 error in this byte. */
    kFLEXCAN_DataBits7Error = 14U,      /*!< Data bit 7 error in this byte. */
    kFLEXCAN_AllZeroError = 6U,         /*!< All-zeros non-correctable error in this byte. */
    kFLEXCAN_AllOneError = 31U,         /*!< All-ones non-correctable error in this byte. */
    kFLEXCAN_NonCorrectableErrors       /*!< Non-correctable error in this byte. */
} flexcan_byte_error_syndrome_t;

/** FlexCAN memory error register status structure
 *
 * This structure contains the memory access properties that caused a memory error access. It is used as the parameter of FLEXCAN_GetMemoryErrorReportStatus() function. And user can
 * use FLEXCAN_GetMemoryErrorReportStatus to get the status of the last memory error access.
 */
typedef struct _flexcan_memory_error_report_status
{
    flexcan_memory_error_type_t errorType;      /* The type of memory error that giving rise to the report. */
    flexcan_memory_access_type_t accessType;    /* The type of memory access that giving rise to the memory error. */
    UINT16 accessAddress;                       /* The address where memory error detected. */
    UINT32 errorData;                           /* The raw data word read from memory with error. */
    struct
    {
        BOOLEAN byteIsRead;                     /* The byte n (0~3) was read or not. */
        flexcan_byte_error_syndrome_t bitAffected; /* The type of error and which bit in byte (n) is affected by the error. */
    } byteStatus[4];
} flexcan_memory_error_report_status_t;
#endif

/**
 * IOCTL_CAN_CONTROLLER_GET_MEMORY_ERROR_STATUS
 *
 * This function gets the FlexCAN Memory Error Report registers status.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_GET_MEMORY_ERROR_STATUS,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_MEMORY_ERROR_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_MEMORY_ERROR_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MEMORY_ERROR_OUTPUT A pointer to FlexCAN Memory Error Report registers status structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_GET_MEMORY_ERROR_STATUS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_GET_MEMORY_ERROR_STATUS, \
                METHOD_BUFFERED, \
                FILE_READ_DATA)

#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
typedef flexcan_memory_error_report_status_t CAN_CONTROLLER_MEMORY_ERROR_OUTPUT;
#endif

/**
 * IOCTL_CAN_CONTROLLER_ENABLE_INTERRUPTS
 *
 * This function enables the FlexCAN interrupts according to the provided mask. The mask is a logical OR of enumeration members, see _flexcan_interrupt_enable.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_ENABLE_INTERRUPTS,
 *                  &CAN_CONTROLLER_FLAGS_INPUT,
 *                  sizeof(CAN_CONTROLLER_FLAGS_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_FLAGS_INPUT The interrupts to enable. Logical OR of _flexcan_interrupt_enable.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_ENABLE_INTERRUPTS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_ENABLE_INTERRUPTS, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_DISABLE_INTERRUPTS
 *
 * This function disables the FlexCAN interrupts according to the provided mask. The mask is a logical OR of enumeration members, see _flexcan_interrupt_enable.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_DISABLE_INTERRUPTS,
 *                  &CAN_CONTROLLER_FLAGS_INPUT,
 *                  sizeof(CAN_CONTROLLER_FLAGS_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_FLAGS_INPUT The interrupts to disable. Logical OR of _flexcan_interrupt_enable.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_DISABLE_INTERRUPTS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_DISABLE_INTERRUPTS, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_ENABLE_MB_INTERRUPTS
 *
 * This function disables the interrupts of given Message Buffers.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_ENABLE_MB_INTERRUPTS,
 *                  &CAN_CONTROLLER_MBFLAGS_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBFLAGS_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBFLAGS_INPUT The ORed FlexCAN Message Buffer mask.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_ENABLE_MB_INTERRUPTS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_ENABLE_MB_INTERRUPTS, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_DISABLE_MB_INTERRUPTS
 *
 * This function disables the interrupts of given Message Buffers.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_DISABLE_MB_INTERRUPTS,
 *                  &CAN_CONTROLLER_MBFLAGS_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBFLAGS_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBFLAGS_INPUT The ORed FlexCAN Message Buffer mask.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_DISABLE_MB_INTERRUPTS \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_DISABLE_MB_INTERRUPTS, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_ENABLE_RXFIFO_DMA
 *
 * This function enables or disables the DMA feature of FlexCAN build-in Rx FIFO.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_ENABLE_RXFIFO_DMA,
 *                  &CAN_CONTROLLER_BOOLEAN_INPUT,
 *                  sizeof(CAN_CONTROLLER_BOOLEAN_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_BOOLEAN_INPUT TRUE to enable, FALSE to disable.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_ENABLE_RXFIFO_DMA \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_ENABLE_RXFIFO_DMA, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

typedef BOOLEAN CAN_CONTROLLER_BOOLEAN_INPUT;

/**
 * IOCTL_CAN_CONTROLLER_GET_RXFIFO_ADDR
 *
 * This function returns the FlexCAN Rx FIFO Head address, which is mainly used for the DMA/eDMA use case.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_GET_RXFIFO_ADDR,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_GET_RXFIFO_ADDR_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_GET_RXFIFO_ADDR_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_GET_RXFIFO_ADDR A pointer to FlexCAN Rx FIFO Head address.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_GET_RXFIFO_ADDR \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_GET_RXFIFO_ADDR, \
                METHOD_BUFFERED, \
                FILE_READ_DATA)

typedef UINT32 CAN_CONTROLLER_GET_RXFIFO_ADDR_OUTPUT;

/**
 * IOCTL_CAN_CONTROLLER_ENABLE
 *
 * This function enables or disables the FlexCAN module.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_ENABLE,
 *                  &CAN_CONTROLLER_BOOLEAN_INPUT,
 *                  sizeof(CAN_CONTROLLER_BOOLEAN_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_BOOLEAN_INPUT TRUE to enable, FALSE to disable.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_ENABLE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_ENABLE, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_WRITE_TXMB
 *
 * This function writes a CAN Message to the specified Transmit Message Buffer and changes the Message Buffer state to
 * start CAN Message transmit. After that the function returns immediately.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_WRITE_TXMB,
 *                  &CAN_CONTROLLER_SEND_INPUT,
 *                  sizeof(CAN_CONTROLLER_SEND_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SEND_INPUT A pointer to parameters structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT  kStatus_Success - Write Tx Message Buffer Successfully.
 *                                      kStatus_Fail    - Tx Message Buffer is currently in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_WRITE_TXMB \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_WRITE_TXMB, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef status_t CAN_CONTROLLER_STATUS_OUTPUT;

/**
 * IOCTL_CAN_CONTROLLER_READ_RXMB
 *
 * This function reads a CAN message from a specified Receive Message Buffer. The function fills a receive CAN message
 * frame structure with just received data and activates the Message Buffer again. The function returns immediately.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_READ_RXMB,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  &CAN_CONTROLLER_RECEIVE_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_RECEIVE_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT A pointer to the FlexCAN FD Message Buffer index.
 * @param CAN_CONTROLLER_RECEIVE_OUTPUT A pointer to output parameters structure
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_READ_RXMB \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_READ_RXMB, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef UINT8 CAN_CONTROLLER_MBIDX_INPUT;

/** FlexCAN WriteFDTxMb parameters structure. */
typedef struct _flexcan_send_fd
{
    UINT8                   mbidx;      /* The Message Buffer index */
    flexcan_fd_frame_t      TxFrame;    /* CAN message FD frame to be sent */
} flexcan_send_fd_param_t;

/**
 * IOCTL_CAN_CONTROLLER_WRITE_FD_TXMB
 *
 * This function writes a CAN FD Message to the specified Transmit Message Buffer and changes the Message Buffer state
 * to start CAN FD Message transmit. After that the function returns immediately.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_WRITE_FD_TXMB,
 *                  &CAN_CONTROLLER_SEND_FD_INPUT,
 *                  sizeof(CAN_CONTROLLER_SEND_FD_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_SEND_FD_INPUT Pointer to WriteFDTxMb parameters structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT 	kStatus_Success - Write Tx Message Buffer Successfully.
 *                                      kStatus_Fail    - Tx Message Buffer is currently in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_WRITE_FD_TXMB \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_WRITE_FD_TXMB, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_send_fd_param_t CAN_CONTROLLER_SEND_FD_INPUT;

/** FlexCAN ReadFDRxMb output parameters structure. */
typedef struct _flexcan_fd_return
{
    flexcan_fd_frame_t      RxFrame;    /* Received CAN message FD frame */
    status_t                status;     /* Returned status of  FLEXCAN_TransferFDSendBlocking() */
} flexcan_fd_return_t;

/**
 * IOCTL_CAN_CONTROLLER_READ_FD_RXMB
 *
 * This function reads a CAN FD message from a specified Receive Message Buffer. The function fills a receive CAN FD
 * message frame structure with just received data and activates the Message Buffer again. The function returns immediately.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_READ_FD_RXMB,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  &CAN_CONTROLLER_FD_RECEIVE_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_FD_RECEIVE_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT A pointer to the FlexCAN FD Message Buffer index.
 * @param CAN_CONTROLLER_FD_RECEIVE_OUTPUT A pointer to ReadFDRxMb output parameters structure
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_READ_FD_RXMB \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_READ_FD_RXMB, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_fd_return_t CAN_CONTROLLER_FD_RECEIVE_OUTPUT;

/**
 * IOCTL_CAN_CONTROLLER_READ_RXFIFO
 *
 * This function reads a CAN message from the FlexCAN Legacy Rx FIFO.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_READ_RXFIFO,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_RECEIVE_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_RECEIVE_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_RECEIVE_OUTPUT Pointer to output parameters structure
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_READ_RXFIFO \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_READ_RXFIFO, \
                METHOD_BUFFERED, \
                FILE_READ_DATA)

/**
 * IOCTL_CAN_CONTROLLER_FD_ABORT_SEND
 *
 * This function aborts the interrupt driven message send process.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_FD_ABORT_SEND,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT A pointer to the FlexCAN FD Message Buffer index.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_FD_ABORT_SEND \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_FD_ABORT_SEND, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_FD_ABORT_RECEIVE
 *
 * This function aborts the interrupt driven message receive process.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_FD_ABORT_RECEIVE,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT Pointer to the FlexCAN FD Message Buffer index.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_FD_ABORT_RECEIVE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_FD_ABORT_RECEIVE, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_BLOCKING
 *
 * Performs a polling receive transaction from Legacy Rx FIFO on the CAN bus.
 * Note:  A transfer handle does not need to be created before calling this API.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_BLOCKING,
 *                  NULL,
 *                  0,
 *                  &CAN_CONTROLLER_RECEIVE_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_RECEIVE_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_RECEIVE_OUTPUT Pointer to ReceiveBlocking output parameters structure.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_RECEIVE_FIFO_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

/** FlexCAN Rx FIFO transfer. */
typedef struct _flexcan_fifo_ioctl
{
    flexcan_fd_frame_t  framefd;    /* The buffer of CAN Message to be received from Enhanced Rx FIFO. */
    size_t              frameNum;   /* Number of CAN Message need to be received from Ehanced Rx FIFO. */
    flexcan_frame_t     frame;      /* The buffer of CAN Message to be received from Rx FIFO. */
} flexcan_fifo_ioctl_t;


/**
 * IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_NON_BLOCKING
 *
 * This function receives a message using IRQ. This is a non-blocking function, which returns right away. When all
 * messages have been received, the receive callback function is called.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_NON_BLOCKING,
 *                  &CAN_CONTROLLER_RECEIVE_FIFO_NB_INPUT,
 *                  sizeof(CAN_CONTROLLER_RECEIVE_FIFO_NB_INPUT),
 *                  &CAN_CONTROLLER_STATUS_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_STATUS_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_RECEIVE_FIFO_NB_INPUT A pointer to input parameters structure.
 * @param CAN_CONTROLLER_STATUS_OUTPUT	kStatus_Success - Start Rx FIFO receiving process successfully.
 *                                      kStatus_FLEXCAN_RxFifoBusy - Rx FIFO is currently in use.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_RECEIVE_FIFO_NON_BLOCKING \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_RECEIVE_FIFO_NON_BLOCKING, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_fifo_ioctl_t CAN_CONTROLLER_RECEIVE_FIFO_NB_INPUT;

/**
 * IOCTL_CAN_CONTROLLER_GET_TIMESTAMP
 *
 * Then function can only be used when calling non-blocking Data transfer (TX/RX) API, After TX/RX data transfer done
 * (User can get the status by handler's callback function), we can get the detail index of Mailbox's timestamp by handle.
 * Detail non-blocking data transfer API (TX/RX) contain.
 *  SendNonBlocking
 *  FDSendNonBlocking
 *  ReceiveNonBlocking
 *  FDReceiveNonBlocking
 *  ReceiveFifoNonBlocking
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_GET_TIMESTAMP,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  &CAN_CONTROLLER_TIMESTAMP_OUTPUT,
 *                  sizeof(CAN_CONTROLLER_TIMESTAMP_OUTPUT),
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT A pointer to the FlexCAN Message Buffer index.
 * @param CAN_CONTROLLER_TIMESTAMP_OUTPUT Pointer to mailbox 's timestamp stored in the handle.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_GET_TIMESTAMP \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_GET_TIMESTAMP, \
                METHOD_BUFFERED, \
                FILE_READ_DATA)

typedef UINT32 CAN_CONTROLLER_TIMESTAMP_OUTPUT;

/**
 * IOCTL_CAN_CONTROLLER_ABORT_SEND
 *
 * This function aborts the interrupt driven message send process.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_ABORT_SEND,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT A pointer to the FlexCAN Message Buffer index.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_ABORT_SEND \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_ABORT_SEND, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_ABORT_RECEIVE
 *
 * Aborts the interrupt driven message receive process.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_ABORT_RECEIVE_FIFO,
 *                  &CAN_CONTROLLER_MBIDX_INPUT,
 *                  sizeof(CAN_CONTROLLER_MBIDX_INPUT),
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param CAN_CONTROLLER_MBIDX_INPUT A pointer to the FlexCAN Message Buffer index.
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_ABORT_RECEIVE \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_ABORT_RECEIVE, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)

/**
 * IOCTL_CAN_CONTROLLER_ABORT_RECEIVE_FIFO
 *
 * Aborts the interrupt driven message receive from Legacy Rx FIFO process.
 *
 * DeviceIoControl(
 *                  hDevice,
 *                  IOCTL_CAN_CONTROLLER_ABORT_RECEIVE_FIFO,
 *                  NULL,
 *                  0,
 *                  NULL,
 *                  0,
 *                  &returned,
 *                  NULL
 *                  );
 *
 * @param hDevice Handle of opened CAN device.
 * @param IOCTL The control code for the operation
 * @param returned A pointer to a variable that receives the size of the data stored in the output buffer, in bytes.
 *
 * @return If the operation completes successfully, the return value is nonzero (TRUE).
 */
#define IOCTL_CAN_CONTROLLER_ABORT_RECEIVE_FIFO \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_CONTROLLER_ABORT_RECEIVE_FIFO, \
                METHOD_BUFFERED, \
                FILE_WRITE_DATA)





/**
 *IOCTL_CAN_HELPER_FLEXCAN_ID_STD
 */
#define IOCTL_CAN_HELPER_FLEXCAN_ID_STD \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_HELPER_FLEXCAN_ID_STD, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef UINT32 CAN_HELPER_FLEXCAN_ID_STD;


typedef struct _flexcan_rx_mb_std_ioctl_t
{
    UINT32  id;   // FlexCAN 
    UINT32  rtr;  // FlexCAN 
    UINT32  ide;  // FlexCAN 
} flexcan_rx_mb_std_ioctl_t;

/**
 * IOCTL_CAN_HELPER_FLEXCAN_RX_MB_STD_MASK
 */
#define IOCTL_CAN_HELPER_FLEXCAN_RX_MB_STD_MASK \
            CTL_CODE( \
                FILE_DEVICE_CONTROLLER, \
                CAN_IOCTL_ID_HELPER_FLEXCAN_RX_MB_STD_MASK, \
                METHOD_BUFFERED, \
                FILE_READ_DATA | FILE_WRITE_DATA)

typedef flexcan_rx_mb_std_ioctl_t CAN_HELPER_FLEXCAN_RX_MB_STD_MASK_INPUT;

/** @} */ /* end of imxcan */