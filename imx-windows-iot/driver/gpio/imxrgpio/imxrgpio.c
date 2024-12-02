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

#include "precomp.h"

#include "imxrgpio.h"
#include "imxrgpio_io_map.h"
#include "imx_acpi_utils.h"
#include "trace.h"

#ifdef ALLOC_PRAGMA
#pragma alloc_text (INIT, DriverEntry)
#pragma alloc_text (PAGE, EvtDriverDeviceAdd)
#pragma alloc_text (PAGE, EvtDriverUnload)
#pragma alloc_text (PAGE, ConnectIoPins)
#pragma alloc_text (PAGE, DisconnectIoPins)
#pragma alloc_text (PAGE, QueryControllerBasicInformation)
#pragma alloc_text (PAGE, PrepareController)
#pragma alloc_text (PAGE, ReleaseController)
#pragma alloc_text (PAGE, ConnectFunctionConfigPins)
#pragma alloc_text (PAGE, DisconnectFunctionConfigPins)
#pragma alloc_text (CODE, ReadGpioPinsUsingMask)
#pragma alloc_text (CODE, WriteGpioPinsUsingMask)
#pragma alloc_text (CODE, EnableInterrupt)
#pragma alloc_text (CODE, DisableInterrupt)
#pragma alloc_text (CODE, StartController)
#pragma alloc_text (CODE, StartController)
#pragma alloc_text (CODE, StopController)
#endif

#if DBG
LARGE_INTEGER        DriverStartTime;

#endif

#define CFG_PIN_CMD_CHECK_ONLY             0x01
#define CFG_PIN_CMD_SET                    0x02
#define CFG_PIN_CMD_SET_DEFAULT            0x04

/* ACPI vendor data defines  */
#define ACPI_TAG_EMPTY                     0x00
#define ACPI_TAG_PADCTL                    0x01

/* IOMUXC configuration constants */
#define IOMUXC_PAD_ALT_MAX_VALUE           7
#define IOMUXC_PAD_ALT_GPIO_AUTOSELECT    (IOMUXC_PAD_ALT_MAX_VALUE + 1)

/* Drive strength configuration constant used to instruct drive not to set pad drive strength */
#define GPIO_DRIVE_STRENGTH_DEFAULT       ((UINT32)(-1))

/* GPIO configuration constants */
#define GPIO_MAX_BANKS                     8
#define GPIO_MAX_PINS_PER_BANK             32

/* Memory tags for this driver */
#define GPIO_MEM_TAG_ACPI                  ((ULONG)'AioG')
#define GPIO_MEM_TAG_DRV                   ((ULONG)'DioG')


typedef enum GPIO_PIN_INTERRUPT_CONFIG_e {
    GPIO_PIN_INTERRUPT_CONFIG_LOW_LEVEL    = 0x00080000,
    GPIO_PIN_INTERRUPT_CONFIG_RISING_EDGE  = 0x00090000,
    GPIO_PIN_INTERRUPT_CONFIG_FALLING_EDGE = 0x000A0000,
    GPIO_PIN_INTERRUPT_CONFIG_BOTH_EDGES   = 0x000B0000,
    GPIO_PIN_INTERRUPT_CONFIG_HIGH_LEVEL   = 0x000C0000,
    GPIO_PIN_INTERRUPT_CONFIG_ERROR        = 0xFFFFFFFF
} GPIO_PIN_INTERRUPT_CONFIG_t;

/* ACPI Pad info structure */
#pragma pack(push, 1)

typedef struct ACPI_PAD_CONFIG_s {
    union {
        UINT8 R;
        struct {
            UINT8  PinNumber  : 5;                        /* Zero based index inside GPIO device/bank */
            UINT8  BankNumber : 3;                        /* Zero based index of GPIO device/bank     */
        } B;
    } PadIdx;
    UINT16                      PadCtlRegOffset;          /* Pad mux register offset                  */
    UINT16                      PadMuxRegOffset;          /* Pad control register offset              */
    UINT8                       PadMuxGpioAltValue;       /* Pad mux value for GPIO pin               */
} ACPI_PAD_CONFIG_t;

typedef struct ACPI_PIN_CONFIG_s {
    union {
        UINT8 R;
        struct {
            UINT8  PinNumber  : 5;                        /* Zero based index inside GPIO device/bank */
            UINT8  BankNumber : 3;                        /* Zero based index of GPIO device/bank     */
        } B;
    } PadIdx;
    UINT8                       PadMuxAltValue;           /* ALT mux value for PAD                     */
    UINT16                      InputSelectCtlRegOffset;  /* Input select mux register offset          */
    UINT8                       InputSelectMuxAltValue;   /* Input select mux value                    */
} ACPI_PIN_CONFIG_t;

typedef struct IMX_VENDOR_DATA_ELEMENT_s {
    UINT8      Tag;
    union {
        UINT32 PadCtl;
    } Data;
} IMX_VENDOR_DATA_ELEMENT_t;

#pragma pack (pop)

/*
 * IOMUX_SW_MUX_CTL for PAD x register bits definition *
 */
typedef union IOMUX_SW_MUX_CTL_u {
    UINT32 R;
    struct {
        UINT32 MUX_MODE    :  3;  /* MUX mode select */
        UINT32 Reserved_3  :  1;  /* Reserved */
        UINT32 SION        :  1;  /* Software input on */
        UINT32 Reserved_5  : 27;  /* Reserved */
    } B;
} IOMUX_SW_MUX_CTL_t;

/*
 * GPIO_SW_PAD_CTL for PAD x register bits definition *
 */
typedef union IOMUX_SW_PAD_CTL_u {
    UINT32 R;
    struct {
        UINT32 Reserved_0         :  1;  /* Reserved */
        UINT32 DSE                :  6;  /* Drive strength select */
        UINT32 FSEL               :  2;  /* Slew rate select */
        UINT32 PU_EN              :  1;  /* Pull-up enable */
        UINT32 PD_EN              :  1;  /* Pull-down enable */
        UINT32 OD_EN              :  1;  /* Open drain enable */
        UINT32 HYS_EN             :  1;  /* Hysteresis enable */
        UINT32 Reserved_13        : 11;  /* Reserved */
        UINT32 APC_D0_ACCESS_DIS  :  1;  /* Domain 0 access disable */
        UINT32 APC_D1_ACCESS_DIS  :  1;  /* Domain 1 access disable */
        UINT32 APC_D2_ACCESS_DIS  :  1;  /* Domain 2 access disable */
        UINT32 APC_D3_ACCESS_DIS  :  1;  /* Domain 3 access disable */
        UINT32 APC_D0_LOCK        :  1;  /* Domain access disable APC_D0_ACCESS_DIS bit lock */
        UINT32 APC_D1_LOCK        :  1;  /* Domain access disable APC_D1_ACCESS_DIS bit lock */
        UINT32 APC_D2_LOCK        :  1;  /* Domain access disable APC_D2_ACCESS_DIS bit lock */
        UINT32 APC_D3_LOCK        :  1;  /* Domain access disable APC_D3_ACCESS_DIS bit lock */
    } B;
} IOMUX_SW_PAD_CTL_t;

/*
 * IOMUX_IPP_INPUT_SELECT for PIN X register bits definition *
 */
typedef union IOMUX_IPP_INPUT_SELECT_u {
    UINT32 R;
    struct {
        UINT32 DAISY      : 2;  /* Selects Pads Involved in Daisy Chain. */
        UINT32 Reserved_2 : 6;  /* Reserved */
    } B;
} IOMUX_IPP_INPUT_SELECT_t;

/*
 * GPIO_VIRT_IER register bits definition *
 */
typedef union GPIO_VIRT_IER_union_t {
    UINT32 R;
    struct {
        UINT32 IE0 :  1;  /* Interrupt  0 Enable/Disable */
        UINT32 IE1 :  1;  /* Interrupt  1 Enable/Disable */
        UINT32 IE2 :  1;  /* Interrupt  2 Enable/Disable */
        UINT32 IE3 :  1;  /* Interrupt  3 Enable/Disable */
        UINT32 IE4 :  1;  /* Interrupt  4 Enable/Disable */
        UINT32 IE5 :  1;  /* Interrupt  5 Enable/Disable */
        UINT32 IE6 :  1;  /* Interrupt  6 Enable/Disable */
        UINT32 IE7 :  1;  /* Interrupt  7 Enable/Disable */
        UINT32 IE8 :  1;  /* Interrupt  8 Enable/Disable */
        UINT32 IE9 :  1;  /* Interrupt  9 Enable/Disable */
        UINT32 IE10 : 1;  /* Interrupt 10 Enable/Disable */
        UINT32 IE11 : 1;  /* Interrupt 11 Enable/Disable */
        UINT32 IE12 : 1;  /* Interrupt 12 Enable/Disable */
        UINT32 IE13 : 1;  /* Interrupt 13 Enable/Disable */
        UINT32 IE14 : 1;  /* Interrupt 14 Enable/Disable */
        UINT32 IE15 : 1;  /* Interrupt 15 Enable/Disable */
        UINT32 IE16 : 1;  /* Interrupt 16 Enable/Disable */
        UINT32 IE17 : 1;  /* Interrupt 17 Enable/Disable */
        UINT32 IE18 : 1;  /* Interrupt 18 Enable/Disable */
        UINT32 IE19 : 1;  /* Interrupt 19 Enable/Disable */
        UINT32 IE20 : 1;  /* Interrupt 20 Enable/Disable */
        UINT32 IE21 : 1;  /* Interrupt 21 Enable/Disable */
        UINT32 IE22 : 1;  /* Interrupt 22 Enable/Disable */
        UINT32 IE23 : 1;  /* Interrupt 23 Enable/Disable */
        UINT32 IE24 : 1;  /* Interrupt 24 Enable/Disable */
        UINT32 IE25 : 1;  /* Interrupt 25 Enable/Disable */
        UINT32 IE26 : 1;  /* Interrupt 26 Enable/Disable */
        UINT32 IE27 : 1;  /* Interrupt 27 Enable/Disable */
        UINT32 IE28 : 1;  /* Interrupt 28 Enable/Disable */
        UINT32 IE29 : 1;  /* Interrupt 29 Enable/Disable */
        UINT32 IE30 : 1;  /* Interrupt 30 Enable/Disable */
        UINT32 IE31 : 1;  /* Interrupt 31 Enable/Disable */
    } B;
} GPIO_VIRT_IER_t;

/* GPIO Pin (Device) context structure */
typedef struct GPIO_PAD_CONTEXT_s {
    IOMUX_SW_MUX_CTL_t*         pPadMuxReg;
    IOMUX_SW_PAD_CTL_t*         pPadCtlReg;
    IOMUX_SW_PAD_CTL_t          PadCtlRegDefautlValue;
    UINT8                       PadMuxRegDefaultValue;       /* Default MUX + SION value                                                                     */
    struct {
        UINT8                   InputMuxDefaultValue : 3;
        UINT8                   PadMuxGpioValue      : 3;
        UINT8                   IsConected           : 1;
        UINT8                   IsInterruptEnabled   : 1;
    } State;
    UINT8                       PadInputMuxOffsetIdx[8];     /* One-based index into the array of Input mux (Daisy) configuration info for ALTx of this Pad  */
    UINT8                       PadCurrentInputMuxOffsetIdx;
} GPIO_PAD_CONTEXT_t, *pGPIO_PAD_CONTEXT_t;

/* GPIO Bank context structure */
typedef struct GPIO_BANK_CONTEXT_s {
    GPIO_REGS_t*                pGpioRegs;                               /* Bank registers address                                                   */
    ULONG                       GpioRegsSize;                            /* Bank registers size (required by MmUnmapIoSpace())                       */
    GPIO_PAD_CONTEXT_t*         PadContextArray[GPIO_MAX_PINS_PER_BANK]; /* Array of Pad/pin context pointers                                        */
    GPIO_VIRT_IER_t             VIRT_IER;                                /* Virtual Interrupt enable regsister                                       */
    GPIO_PDDR_t                 PDDR_DefaultValue;                       /* Port Data Direction register value obtained during driver initialization */
    GPIO_PIDR_t                 PIDR_DefaultValue;                       /* Port Input Disable register value obtained during driver initialization  */

} GPIO_BANK_CONTEXT_t, *pGPIO_BANK_CONTEXT_t;

/* GPIO Driver context structure */
typedef struct GPIO_DRV_CONTEXT_s {
    GPIO_BANK_CONTEXT_t*        BankContextArray[GPIO_MAX_BANKS];        /* Array of Bank context pointers                                   */
    ULONG                       GPIO_BankCount;                          /* Real number of Banks (obtaind from ACPI)                         */
    IMX_ACPI_UTILS_DEV_CONTEXT  pACPI_UtilsContext;                      /* Acpi context pointer                                             */
    PVOID                       pIoMuxRegs;                              /* IOMUX registers address                                          */
    ULONG                       IoMuxRegsSize;                           /* IOMUX registers size (required by MmUnmapIoSpace())              */
    ACPI_PIN_CONFIG_t*          AcpiPinConfigArray;                      /* IOMUX input select configuration array address                   */
    PVOID                       pExtDriverContext;                       /* Non-paged memory address alloacted for additional driver data    */
} GPIO_DRV_CONTEXT_t, *pGPIO_DRV_CONTEXT_t;

/* IMX GPIO driver PAD configuration structure */
typedef struct IMX_CONNECT_PIN_s {
    GPIO_DRV_CONTEXT_t*         pDrvContext;
    BANK_ID                     BankId;
    UINT32                      PinCount;
    PIN_NUMBER                  PinNumber;
    UINT8                       Mux;
    GPIO_CONNECT_IO_PINS_MODE   ConnectMode;
    UCHAR                       PullConfiguration;
    UINT32                      DriveStrength;
    PVOID                       VendorData;
    ULONG                       VendorDataLength;
    UINT32                      ConnectedPinsMask;          /* Updated by each call of ImxConnectIoPin() value |= (1 << PinNumber)                           */
    UINT32                      ConnectedOutputPinsMask;    /* Updated by each call of ImxConnectIoPin() value |= (1 << PinNumber) & (ConnectMode == output) */
    UINT32                      ConnectedInputputPinsMask;  /* Updated by each call of ImxConnectIoPin() value |= (1 << PinNumber) & (ConnectMode == input)  */
} IMX_CONNECT_PIN_t;

#define CheckBankIdGetRegsOrBreakIfError(_BankNum)                                                      \
    if (_BankNum >= pDrvContext->GPIO_BankCount) {                                                      \
        DBG_PRINT_ERROR("BankNumber(%d) >= GPIO_BankCount(%d)", _BankNum, pDrvContext->GPIO_BankCount); \
        ntStatus = STATUS_INVALID_PARAMETER;                                                            \
        break;                                                                                          \
    }                                                                                                   \
    pGpioRegs = pDrvContext->BankContextArray[_BankNum]->pGpioRegs;

#define CheckBankIdAndBreakIfError(_BankNum)                                                            \
    if (_BankNum >= pDrvContext->GPIO_BankCount) {                                                      \
        DBG_PRINT_ERROR("BankNumber(%d) >= GPIO_BankCount(%d)", _BankNum, pDrvContext->GPIO_BankCount); \
        ntStatus = STATUS_INVALID_PARAMETER;                                                            \
        break;                                                                                          \
    }                                                                   

#define CheckBankIdAndPinCountAndBreakIfError(_BankNum, _PinNumber)                                         \
    if (_BankNum >= pDrvContext->GPIO_BankCount) {                                                          \
        DBG_PRINT_ERROR("BankNumber(%d) >= GPIO_BankCount(%d)", _BankNum, pDrvContext->GPIO_BankCount);     \
        ntStatus = STATUS_INVALID_PARAMETER;                                                                \
        break;                                                                                              \
    }                                                                                                       \
    if (_PinNumber >= GPIO_MAX_PINS_PER_BANK) {                                                             \
        DBG_PRINT_ERROR("PinNumber(%d) >= GPIO_MAX_PINS_PER_BANK(%d)", _PinNumber, GPIO_MAX_PINS_PER_BANK); \
        ntStatus = STATUS_INVALID_PARAMETER;                                                                \
        break;                                                                                              \
    }

/**
* Releases all resources allocated by this driver.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*
* Return value: none.
*/
void ReleaseResources(GPIO_DRV_CONTEXT_t* pDrvContext) {
    DBG_DEV_METHOD_BEG();
    if (pDrvContext->GPIO_BankCount != 0) {
        for (unsigned int i = 0; i < pDrvContext->GPIO_BankCount; i++) {
            if (pDrvContext->BankContextArray[i] != NULL) {
                if (pDrvContext->BankContextArray[i]->pGpioRegs != NULL) {
                    MmUnmapIoSpace((void*)pDrvContext->BankContextArray[i]->pGpioRegs, pDrvContext->BankContextArray[i]->GpioRegsSize);
                    DBG_DEV_PRINT_INFO("MmUnmapIoSpace: 0x%016p,  GPIODev_GpioRegsSize: 0x%08X", pDrvContext->BankContextArray[i]->pGpioRegs, pDrvContext->BankContextArray[i]->GpioRegsSize);
                }
            }
        }
    }
    if (pDrvContext->pIoMuxRegs != NULL) {
        MmUnmapIoSpace((void*)pDrvContext->pIoMuxRegs, pDrvContext->IoMuxRegsSize);
        DBG_DEV_PRINT_INFO("MmUnmapIoSpace: 0x%016p,  GPIODev_GpioRegsSize: 0x%08X", pDrvContext->pIoMuxRegs, pDrvContext->IoMuxRegsSize);
    }
    if (pDrvContext->pExtDriverContext != NULL) {
        ExFreePoolWithTag(pDrvContext->pExtDriverContext, GPIO_MEM_TAG_DRV);
        DBG_DEV_PRINT_INFO("ExFreePoolWithTag: 0x%016p", pDrvContext->pExtDriverContext);
    }
    Acpi_Deinit(&pDrvContext->pACPI_UtilsContext);
    DBG_DEV_METHOD_END();
}

/**
* Translates OS interrupt settings to the IMX HW interrupt settings.
*
* Params:
*   InterruptMode:   OS interrupt mode.
*   Polarity:        OS interrupt polarity.
*
* Return value: IMX HW interrupt configuration.
*/
GPIO_PIN_INTERRUPT_CONFIG_t ImxGetGpioPinInterruptCfg(KINTERRUPT_MODE InterruptMode, KINTERRUPT_POLARITY Polarity) {
    GPIO_PIN_INTERRUPT_CONFIG_t interruptConfig = GPIO_PIN_INTERRUPT_CONFIG_ERROR;

    switch (InterruptMode) {
    case LevelSensitive:
        switch (Polarity) {
        case InterruptActiveHigh:
            interruptConfig = GPIO_PIN_INTERRUPT_CONFIG_HIGH_LEVEL;
            break;
        case InterruptActiveLow:
            interruptConfig = GPIO_PIN_INTERRUPT_CONFIG_LOW_LEVEL;
            break;
        default:
            interruptConfig = GPIO_PIN_INTERRUPT_CONFIG_ERROR;
        } /* switch (Polarity) */
        break;
    case Latched:
        switch (Polarity) {
        case InterruptRisingEdge:
            interruptConfig = GPIO_PIN_INTERRUPT_CONFIG_RISING_EDGE;
            break;
        case InterruptFallingEdge:
            interruptConfig = GPIO_PIN_INTERRUPT_CONFIG_FALLING_EDGE;
            break;
        case InterruptActiveBoth:
        default:
            break;
        } /* switch (Polarity) */
        break;
    default:
        break;
    } /* switch (InterruptMode) */
    return interruptConfig;
}

/**
* Configures PAD or validates required settings.
*
* Params:
*   pImxConnectPin:  A pointer to the PADs configuration structure.
*   Operation:       IMX_CONNECT_PIN_SET_DEFAULT - sets PAD to the values before the first change of PAD settings.
*                    IMX_CONNECT_PIN_SET_VALUES  - sets PAD acording to the input parameters.
*                    IMX_CONNECT_PIN_CHECK_OLNY  - validates PAD settings, no modification of PAD HW is done
*   PinNumber        Zero-based index of the pin in a pin list
* 
* Return value: STATUS_SUCCESS, STATUS_INVALID_PARAMETER.
*/
NTSTATUS ImxConnectIoPin(IMX_CONNECT_PIN_t* pImxConnectPin, UINT32 Cmd, unsigned int PinNumber) {
    NTSTATUS             ntStatus = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t*  pDrvContext;
    GPIO_BANK_CONTEXT_t* pBankContext;
    GPIO_PAD_CONTEXT_t*  pPadContext;
    IOMUX_SW_MUX_CTL_t   PadMuxRegVal;
    IOMUX_SW_PAD_CTL_t   PadCtlRegVal;
    UINT32               PinMask;

    DBG_DEV_METHOD_BEG_WITH_PARAMS(GPIO_x_IOy_MSG"Cmd: %s, LastPin %s, ALT %d, Mode %s, PullCfg %s, DrvStr %d [mA] ,VndData 0x%016p", GPIO_x_IOy_VAL, ToString_IoConnectPinCmd(Cmd), (PinNumber == (pImxConnectPin->PinCount -1 )) ? "YES":"NO", pImxConnectPin->Mux, ToString_ConnectioMode(pImxConnectPin->ConnectMode), ToString_AcpiPullCfg(pImxConnectPin->PullConfiguration), pImxConnectPin->DriveStrength / 100, pImxConnectPin->VendorData);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    do {
        /* Check if pin number is valid and if pin is available (configured in ACPI) */
        if (pImxConnectPin->PinNumber >= GPIO_MAX_PINS_PER_BANK) {
            ntStatus = STATUS_INVALID_PARAMETER;
            DBG_PRINT_ERROR(GPIO_x_IOy_MSG"Pin number must be < %d", GPIO_x_IOy_VAL, GPIO_MAX_PINS_PER_BANK);
            break;
        }
        pDrvContext  = pImxConnectPin->pDrvContext;
        pBankContext = pDrvContext->BankContextArray[pImxConnectPin->BankId];
        if ((pPadContext = pBankContext->PadContextArray[pImxConnectPin->PinNumber]) == NULL) {
            ntStatus = STATUS_INVALID_PARAMETER;
            DBG_PRINT_ERROR(GPIO_x_IOy_MSG"Pin number is not configured in ACPI Pin_Config[]", GPIO_x_IOy_VAL);
            break;
        }
        if (PinNumber == 0) {  /* Initialze GPIO Bank related internal variables. If more then one pin is configured (see ConnextXXX() and DisconnectXX() methods) */
            pImxConnectPin->ConnectedPinsMask       = 0;                     /* This is the first call of this method, initalize internal variables */
            pImxConnectPin->ConnectedOutputPinsMask = 0;
            pImxConnectPin->ConnectedOutputPinsMask = 0;
        }
        PinMask = (UINT32)(1 << pImxConnectPin->PinNumber);
        pImxConnectPin->ConnectedPinsMask |= PinMask;                        /* Remember that pin X bitfields should be updated */
        if ((Cmd & CFG_PIN_CMD_SET) || (Cmd & CFG_PIN_CMD_CHECK_ONLY)) {
            /* Get current pin configuration from HW registers */
            PadCtlRegVal.R = pPadContext->pPadCtlReg->R;
            PadMuxRegVal.R = pPadContext->pPadMuxReg->R;
            /* Request to reconfigure existing pin setting */
            if (pImxConnectPin->VendorData != NULL) {
                if (pImxConnectPin->VendorDataLength < sizeof(IMX_VENDOR_DATA_ELEMENT_t)) {
                    ntStatus = STATUS_INVALID_PARAMETER;
                    DBG_PRINT_ERROR("Vendor data size is %d. It must be at least %d bytes", pImxConnectPin->VendorDataLength, (UINT32)sizeof(IMX_VENDOR_DATA_ELEMENT_t));
                    break;
                }
                if (((IMX_VENDOR_DATA_ELEMENT_t*)(pImxConnectPin->VendorData))->Tag == ACPI_TAG_PADCTL) {
                    PadCtlRegVal.R = ((IMX_VENDOR_DATA_ELEMENT_t*)(pImxConnectPin->VendorData))->Data.PadCtl;
                    DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"Updating PAD register from vendor defined value: 0x%08X", GPIO_x_IOy_VAL, ((IMX_VENDOR_DATA_ELEMENT_t*)(pImxConnectPin->VendorData))->Data.PadCtl);
                } else {                    
                    DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"Unsupported vendor data tag: %d", GPIO_x_IOy_VAL, ((IMX_VENDOR_DATA_ELEMENT_t*)(pImxConnectPin->VendorData))->Tag);
                }
            }
            /* Update Pin Mux */
            if (pImxConnectPin->Mux == IOMUXC_PAD_ALT_GPIO_AUTOSELECT) {
                PadMuxRegVal.B.MUX_MODE = pPadContext->State.PadMuxGpioValue;
            } else {
                PadMuxRegVal.B.MUX_MODE = pImxConnectPin->Mux;
            }
            /* Update Pull resistors */
            switch (pImxConnectPin->PullConfiguration) {
            case GPIO_PIN_PULL_CONFIGURATION_DEFAULT: /* Do not change current pull value */
                break;
            case GPIO_PIN_PULL_CONFIGURATION_PULLUP:
                PadCtlRegVal.B.PU_EN = 1;
                PadCtlRegVal.B.PD_EN = 0;
                break;
            case GPIO_PIN_PULL_CONFIGURATION_PULLDOWN:
                PadCtlRegVal.B.PU_EN = 0;
                PadCtlRegVal.B.PD_EN = 1;
                break;
            case GPIO_PIN_PULL_CONFIGURATION_NONE:
                PadCtlRegVal.B.PU_EN = 0;
                PadCtlRegVal.B.PD_EN = 0;
                break;
            default:
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR("Unkown Pull mode: %d", pImxConnectPin->PullConfiguration);
                break;
            }
            /* Set Drive strength */
            if (pImxConnectPin->DriveStrength != GPIO_DRIVE_STRENGTH_DEFAULT) {
                /* At the time of writing this driver the drive strength[mA] to PadCtlRegVal.B.DSE fit field is not known so this bitfield is left unchanged */
                /* You can add here yout own mapping */
            }
            if (!NT_SUCCESS(ntStatus)) {
                break;
            }
            /* Remember pin direction */
            switch ((unsigned int)(pImxConnectPin->ConnectMode)) {
            case ConnectModeInput:
                pImxConnectPin->ConnectedInputputPinsMask |= PinMask;                  /* Enable GPIO input direction */
                break;
            case ConnectModeOutput:
                pImxConnectPin->ConnectedOutputPinsMask |= PinMask;                    /* Enable GPIO input output */
                break;
            case (ConnectModeInput | ConnectModeOutput):
                pImxConnectPin->ConnectedInputputPinsMask |= PinMask;                  /* Enable GPIO input direction */
                pImxConnectPin->ConnectedOutputPinsMask |= PinMask;                    /* Enable GPIO input output */
                PadMuxRegVal.B.SION = 1;                                               /* Force inputut path to be ON */
            case ConnectModeInvalid:
                /* Do not modify direction */
                break;
            default:
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR("Unkown Connect mode: %d", pImxConnectPin->ConnectMode);
                break;
            }
            if (!NT_SUCCESS(ntStatus)) {
                break;
            }
        } else {
            /* Request to use default pad setting */
            PadCtlRegVal.R = pPadContext->PadCtlRegDefautlValue.R;  /* Use Pad control register value obtained during driver initialization */
            PadMuxRegVal.R = pPadContext->PadMuxRegDefaultValue;    /* Use Mux control register value obtained during driver initialization */
            DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"Forcing PAD default values", GPIO_x_IOy_VAL);
        }
        if (Cmd & CFG_PIN_CMD_CHECK_ONLY) {
            break;
        }
        if (pPadContext->PadCurrentInputMuxOffsetIdx != 0) {
            ACPI_PIN_CONFIG_t* pAcpiInputSel = &pDrvContext->AcpiPinConfigArray[pPadContext->PadCurrentInputMuxOffsetIdx];
            IOMUX_IPP_INPUT_SELECT_t* pInputSelReg = (IOMUX_IPP_INPUT_SELECT_t*)(PVOID) & (((UINT8*)(pDrvContext->pIoMuxRegs))[pAcpiInputSel->InputSelectCtlRegOffset]);
            DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"Found configured input mux (DaisyRegOffset: 0x%04X), reverting dedault value. CurrentValue: %d, DetaultValue: %d", GPIO_x_IOy_VAL, pAcpiInputSel->InputSelectCtlRegOffset, pInputSelReg->R, pPadContext->State.InputMuxDefaultValue);
            pInputSelReg->R = pPadContext->State.InputMuxDefaultValue;
            pPadContext->PadCurrentInputMuxOffsetIdx = 0;
        }
        /* Update IOMUXC registers for this Pad */        
        if (pImxConnectPin->ConnectMode == ConnectModeInput) {
            unsigned int i = pPadContext->PadInputMuxOffsetIdx[PadMuxRegVal.B.MUX_MODE];
            if (i != 0) {  
                ACPI_PIN_CONFIG_t*        pAcpiInputSel = &pDrvContext->AcpiPinConfigArray[i - 1];
                IOMUX_IPP_INPUT_SELECT_t* pInputSelReg = (IOMUX_IPP_INPUT_SELECT_t*)(PVOID) & (((UINT8*)(pDrvContext->pIoMuxRegs))[pAcpiInputSel->InputSelectCtlRegOffset]);
                DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"ALT_%d Found input mux (DaisyRegOffset: 0x%04X), CurrentValue: %d, NewValue: %d", GPIO_x_IOy_VAL, PadMuxRegVal.B.MUX_MODE, pAcpiInputSel->InputSelectCtlRegOffset, pInputSelReg->R, pAcpiInputSel->InputSelectMuxAltValue);
                pPadContext->PadCurrentInputMuxOffsetIdx = (UINT8)i;
                pPadContext->State.InputMuxDefaultValue = (UINT8)pInputSelReg->R;
                pInputSelReg->R = pAcpiInputSel->InputSelectMuxAltValue;
            }
        }
        /* Update IOMUXC registers for this Pad */
        DBG_DEV_PRINT_INFO_MUX_AND_PAD_CTL_DUMP("(Before)", pPadContext->pPadMuxReg, pPadContext->pPadCtlReg)
        pPadContext->pPadCtlReg->R = PadCtlRegVal.R;
        pPadContext->pPadMuxReg->R = PadMuxRegVal.R;
        DBG_DEV_PRINT_INFO_MUX_AND_PAD_CTL_DUMP("(After )", pPadContext->pPadMuxReg, pPadContext->pPadCtlReg)            
        /* Updated GPIO pin(s) registers as soon as this method is called for the last pin in the pin list */
        if (PinNumber == (pImxConnectPin->PinCount - 1)) {                  /* Check if this pin is the last pin in the pin list */
            GPIO_REGS_t* pGpioRegs = pBankContext->pGpioRegs;                    
            UINT32       PddrRegVal;                                        /* Bank data direction register local copy */
            UINT32       PidrRegVal;                                        /* Bank input disable register local copy */
            if (Cmd & CFG_PIN_CMD_SET_DEFAULT) {                             
                PddrRegVal = pBankContext->PDDR_DefaultValue.R;             /* Use value obtain during driver initialization */
                PidrRegVal = pBankContext->PIDR_DefaultValue.R;             /* Use value obtain during driver initialization */ 
                DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"Forcing GPIO default values", GPIO_x_IOy_VAL);
            } else {
                PddrRegVal = pImxConnectPin->ConnectedOutputPinsMask;       /* Set direction as "output" -> PDDD.PDDn = 1 */
                PidrRegVal = ~(pImxConnectPin->ConnectedInputputPinsMask);  /* Enable input path         -> PIDD.PIDn = 0 */
                DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"Updating GPIO values", GPIO_x_IOy_VAL);
            }
            PddrRegVal &= pImxConnectPin->ConnectedPinsMask;                /* Update only bits for GPIO pins from pin list */
            PidrRegVal &= pImxConnectPin->ConnectedPinsMask;                /* Update only bits for GPIO pins from pin list */
            /* Update GPIO pins direction settings for all gpio pins prom pin list */
            DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"PDDR: 0x%08X, PIDR: 0x%08X, PinBitMask 0x%08X (Before change)", GPIO_x_IOy_VAL, pGpioRegs->PDDR.R, pGpioRegs->PIDR.R, pImxConnectPin->ConnectedPinsMask);
            /* Set Port Data Direction register, PDDn = 1 -> enable output, PDDn = 0 -> disable output */
            pGpioRegs->PDDR.R = (pGpioRegs->PDDR.R & ~pImxConnectPin->ConnectedPinsMask) | PddrRegVal;
            /* Set Port Input Disable register, PIDn = 1 -> disable input, PIDn = 0 -> enable input */
            pGpioRegs->PIDR.R = (pGpioRegs->PIDR.R & ~pImxConnectPin->ConnectedPinsMask) | PidrRegVal;
            DBG_DEV_PRINT_INFO(GPIO_x_IOy_MSG"PDDR: 0x%08X, PIDR: 0x%08X (After change)", GPIO_x_IOy_VAL, pGpioRegs->PDDR.R, pGpioRegs->PIDR.R);
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
* Reads a bank of general-purpose I/O (GPIO) pins.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   ReadParameters -> BankId     The identifier for the bank of GPIO pins.
*                  -> PinValues  A 64-bit variable to which the GPIO controller driver stores the values.
*                  -> Flags      WriteConfiguredPins - Read also from a GPIO pin that is configured for write.
*                  -> Reserved
*
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS ReadGpioPinsUsingMask(PVOID Context, PGPIO_READ_PINS_MASK_PARAMETERS ReadParameters) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t* )Context;
    GPIO_REGS_t*        pGpioRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, ReadOutPin %s", ReadParameters->BankId + 1, (ReadParameters->Flags.WriteConfiguredPins)? "yes":"no");
    do {
        CheckBankIdGetRegsOrBreakIfError(ReadParameters->BankId);
        *ReadParameters->PinValues = pGpioRegs->PDIR.R;           /* Get value of all pins */
        if (ReadParameters->Flags.WriteConfiguredPins == 0) {     /* Request to update also write-configured pins? */
            *ReadParameters->PinValues &= ~pGpioRegs->PDDR.R;     /* No, mask (set to zero) write-configured pins */
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d, PinValues 0x%08X", ReadParameters->BankId + 1, (UINT32)*ReadParameters->PinValues, ntStatus);
    return ntStatus;
}

/**
* Writes to a set of general - purpose I/O (GPIO) pins that are configured as data outputs.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   WriteParameters -> BankId     The identifier for the bank of GPIO pins.
*                   -> SetMask    A bitmask that supplies the pins that need to be set to HIGH.
*                   -> ClearMask  A bitmask that supplies the pins that need to be set to LOW.
*                   -> Flags      Flags controlling the write operation. (not used, pacehoder)
*                   -> Reserved   
*
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS WriteGpioPinsUsingMask(PVOID Context, PGPIO_WRITE_PINS_MASK_PARAMETERS WriteParameters) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    GPIO_REGS_t*        pGpioRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, SetMask 0x%08X, ClrMask 0x%08X", WriteParameters->BankId + 1, (UINT32)WriteParameters->SetMask, (UINT32)WriteParameters->ClearMask);
    do {
        CheckBankIdGetRegsOrBreakIfError(WriteParameters->BankId);
        pGpioRegs->PSOR.R = (UINT32)(WriteParameters->SetMask);
        pGpioRegs->PCOR.R = (UINT32)(WriteParameters->ClearMask);
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d", WriteParameters->BankId + 1, ntStatus);
    return ntStatus;
}

/**
* Opens a logical connection to a set of general-purpose I/O (GPIO) pins and configures these pins for data read or write operations.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   ConnectParameters -> BankId            The identifier for the bank of GPIO pins.
*                     -> PinNumberTable    Pin table address
*                     -> PinCount          Number of pins in PinNumberTable 
*                     -> ConnectMode       ConnectModeInvalid
*                                          ConnectModeInput
*                                          ConnectModeOutput
*                     -> PullConfiguration The pin pull configuration as specified in the ACPI FW descriptor.
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLDEFAULT(0) - use the SOC-defined power-on default pull configuration
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLUP(1)      - enable pull-up resistor
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLDOWN(2)    - enable pull-down resistor
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLNONE(3)    - disable all pull resistors
*                     -> DebounceTimeout   The debounce time in units of 10 microseconds.
*                     -> DriveStrength     The drive strength of the GPIO pin in units of 10 microamperes.
*                     -> VendorData        A pointer to a caller-allocated buffer that contains vendor-specific data obtained from the ACPI firmware for the hardware platform.
*                     -> VendorDataLength  The size, in bytes, of the data buffer that is pointed to by the VendorData member.
*                     -> ConnectFlags      No flags are currently defined for this member.
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS ConnectIoPins(PVOID Context, PGPIO_CONNECT_IO_PINS_PARAMETERS ConnectParameters) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, PinCnt %d, Mode %s, PullCfg %s, DebTimeout %d, DrvStrength %d [mA] ,VendorData 0x%016p", ConnectParameters->BankId + 1, ConnectParameters->PinCount, ToString_ConnectioMode(ConnectParameters->ConnectMode), ToString_AcpiPullCfg(ConnectParameters->PullConfiguration), ConnectParameters->DebounceTimeout, ConnectParameters->DriveStrength/100, ConnectParameters->VendorData);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    do {
        CheckBankIdAndPinCountAndBreakIfError(ConnectParameters->BankId, ConnectParameters->PinCount);
        ImxPinConnectParams.pDrvContext       = pDrvContext;
        ImxPinConnectParams.BankId            = ConnectParameters->BankId;
        ImxPinConnectParams.PinCount          = ConnectParameters->PinCount;
        ImxPinConnectParams.ConnectMode       = ConnectParameters->ConnectMode;
        ImxPinConnectParams.PullConfiguration = ConnectParameters->PullConfiguration;
        ImxPinConnectParams.DriveStrength     = ConnectParameters->DriveStrength;
        ImxPinConnectParams.VendorData        = ConnectParameters->VendorData;
        ImxPinConnectParams.VendorDataLength  = ConnectParameters->VendorDataLength;
        ImxPinConnectParams.Mux               = IOMUXC_PAD_ALT_GPIO_AUTOSELECT;  /* Get GPIO ALT mux value from ACPI */
        for (unsigned int i = 0; i < ConnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber     = ConnectParameters->PinNumberTable[i];
            if (!NT_SUCCESS(ntStatus = ImxConnectIoPin(&ImxPinConnectParams, CFG_PIN_CMD_CHECK_ONLY, i))) {
                DBG_PRINT_ERROR("ImxConnectIoPin() - unsupported pin settings");
                break;
            }
        }
        if (!NT_SUCCESS(ntStatus)) {
            break;
        }
        for (int i = 0; i < ConnectParameters->PinCount; i++) {
            PIN_NUMBER          PinNumber = ConnectParameters->PinNumberTable[i];
            GPIO_PAD_CONTEXT_t* pPadContext = pDrvContext->BankContextArray[ImxPinConnectParams.BankId]->PadContextArray[PinNumber];
            ImxPinConnectParams.PinNumber = PinNumber;
            (void)ImxConnectIoPin(&ImxPinConnectParams, CFG_PIN_CMD_SET, i);
            pPadContext->State.IsConected = 1;
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
* Closes a logical connection to a set of general-purpose I/O (GPIO) pins that are configured for data read or write operations.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   ConnectParameters -> BankId                   The identifier for the bank of GPIO pins.
*                     -> PinNumberTable           Pin table address
*                     -> PinCount                 Number of pins in PinNumberTable
*                     -> DisConnectMode           The mode in which the pins were configured when originally connected. 
*                                                 ConnectModeInvalid    
*                                                 ConnectModeInput
*                                                 ConnectModeOutput
*                     -> DisconnectFlags   
                         - PreserveConfiguration  
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS DisconnectIoPins(PVOID Context, PGPIO_DISCONNECT_IO_PINS_PARAMETERS DisconnectParameters) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, PinCnt %d, Mode %s, PreserveCfg %d", DisconnectParameters->BankId + 1, DisconnectParameters->PinCount, ToString_ConnectioMode(DisconnectParameters->DisconnectMode), DisconnectParameters->DisconnectFlags.PreserveConfiguration);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    do {
        if (DisconnectParameters->DisconnectFlags.PreserveConfiguration == 1) {
            break;
        }
        CheckBankIdAndPinCountAndBreakIfError(DisconnectParameters->BankId, DisconnectParameters->PinCount);
        ImxPinConnectParams.pDrvContext = pDrvContext;
        ImxPinConnectParams.BankId      = DisconnectParameters->BankId;
        ImxPinConnectParams.PinCount    = DisconnectParameters->PinCount;
        for (unsigned int i = 0; i < DisconnectParameters->PinCount; i++) {
            PIN_NUMBER          PinNumber   = DisconnectParameters->PinNumberTable[i];
            GPIO_PAD_CONTEXT_t* pPadContext = pDrvContext->BankContextArray[ImxPinConnectParams.BankId]->PadContextArray[PinNumber];
            ImxPinConnectParams.PinNumber = PinNumber;
            if (!(pPadContext->State.IsInterruptEnabled)){
               (void)ImxConnectIoPin(&ImxPinConnectParams, CFG_PIN_CMD_SET_DEFAULT, i);
            }
            pPadContext->State.IsConected = 0;
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
* Opens a logical connection to a set of general-purpose I/O (GPIO) pins and configures these pins for data read or write operations.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   ConnectParameters -> BankId            The identifier for the bank of GPIO pins.
*                     -> PinNumberTable    Pin table address.
*                     -> PinCount          The number of pins in the pin number table.
*                     -> FunctionNumber    The function number (ALT) in which to configure the pins.
*                     -> PullConfiguration The pin pull configuration as specified in the ACPI FW descriptor.
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLDEFAULT(0) - use the SOC-defined power-on default pull configuration
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLUP(1)      - enable pull-up resistor
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLDOWN(2)    - enable pull-down resistor
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLNONE(3)    - disable all pull resistors
*                     -> VendorData        A pointer to a caller-allocated buffer that contains vendor-specific data obtained from the ACPI firmware for the hardware platform.
*                     -> VendorDataLength  The size, in bytes, of the data buffer that is pointed to by the VendorData member.
*                     -> ConnectFlags      No flags are currently defined for this member.
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS ConnectFunctionConfigPins(PVOID Context, PGPIO_CONNECT_FUNCTION_CONFIG_PINS_PARAMETERS ConnectParameters) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, PinCnt %d, ALT %d, PullCfg %s ,VendorData 0x%016p", ConnectParameters->BankId + 1, ConnectParameters->PinCount, ConnectParameters->FunctionNumber, ToString_AcpiPullCfg(ConnectParameters->PullConfiguration), ConnectParameters->VendorData);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    do {
        CheckBankIdAndPinCountAndBreakIfError(ConnectParameters->BankId, ConnectParameters->PinCount);
        ImxPinConnectParams.pDrvContext       = pDrvContext;
        ImxPinConnectParams.BankId            = ConnectParameters->BankId;
        ImxPinConnectParams.PinCount          = ConnectParameters->PinCount;
        ImxPinConnectParams.ConnectMode       = ConnectModeInvalid;  /* Do not modify mode */
        ImxPinConnectParams.PullConfiguration = ConnectParameters->PullConfiguration;
        ImxPinConnectParams.DriveStrength     = GPIO_DRIVE_STRENGTH_DEFAULT;       /* Do not update drive stregth */
        ImxPinConnectParams.VendorData        = ConnectParameters->VendorData;
        ImxPinConnectParams.VendorDataLength  = ConnectParameters->VendorDataLength;
        ImxPinConnectParams.Mux               = (UINT8)ConnectParameters->FunctionNumber;  /* Caller provided PAD Mux ALT */
        if (ConnectParameters->FunctionNumber > IOMUXC_PAD_ALT_MAX_VALUE) {
            ntStatus = STATUS_INVALID_PARAMETER;
            DBG_PRINT_ERROR("ACPI data error: Pad GPIO_(%d)_IOxx ALT%d. ALT value is out of range(%d)", ConnectParameters->BankId + 1, ConnectParameters->FunctionNumber, IOMUXC_PAD_ALT_MAX_VALUE);
            break;
        }
        for (unsigned int i = 0; i < ConnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber     = ConnectParameters->PinNumberTable[i];
            if (!NT_SUCCESS(ntStatus = ImxConnectIoPin(&ImxPinConnectParams, CFG_PIN_CMD_CHECK_ONLY, i))) {
                DBG_PRINT_ERROR("ImxConnectIoPin() - unsupported pin settings");
                break;
            }
        }
        if (!NT_SUCCESS(ntStatus)) {
            break;
        }
        for ( int i = 0; i < ConnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber = ConnectParameters->PinNumberTable[i];
            (void)ImxConnectIoPin(&ImxPinConnectParams, CFG_PIN_CMD_SET, i);
        }            
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
* Closes a logical connection to a set of general-purpose I/O (GPIO) pins that are configured for data read or write operations.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   DisconnectParameters -> BankId            The identifier for the bank of GPIO pins.
*                        -> PinNumberTable    Pin table address
*                        -> PinCount          Number of pins in PinNumberTable
*                        -> ConnectFlags      No flags are currently defined for this member.
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS DisconnectFunctionConfigPins(PVOID Context, PGPIO_DISCONNECT_FUNCTION_CONFIG_PINS_PARAMETERS DisconnectParameters) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, PinCnt %d", DisconnectParameters->BankId + 1, DisconnectParameters->PinCount);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    do {
        CheckBankIdAndPinCountAndBreakIfError(DisconnectParameters->BankId, DisconnectParameters->PinCount);
        ImxPinConnectParams.pDrvContext = pDrvContext;
        ImxPinConnectParams.BankId      = DisconnectParameters->BankId;
        ImxPinConnectParams.PinCount    = DisconnectParameters->PinCount;
        for (unsigned int i = 0; i < DisconnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber = DisconnectParameters->PinNumberTable[i];
            (void)ImxConnectIoPin(&ImxPinConnectParams, CFG_PIN_CMD_SET_DEFAULT, i);
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
* Enables interrupts on a general-purpose I/O (GPIO) pin that is configured as an interrupt input.
* 
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   EnableParametersPtr -> BankId              The identifier for the bank of GPIO pins.
*                       -> PinNumber           The bank-relative pin number.
*                       -> Flags               No flags are currently defined for this operation.
*                       -> InterruptMode       LevelSensitive(level-triggered) or Latched(edge-triggered)
*                       -> Polarity            LevelSensitive: active-high
*                                                              active-low
*                                              Latched:        on the rising edge
*                                                              on the falling edge
*                                                              or on both edges
*                     -> PullConfiguration     The pin pull configuration as specified in the ACPI FW descriptor.
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLDEFAULT(0) - use the SOC-defined power-on default pull configuration
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLUP(1)      - enable pull-up resistor
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLDOWN(2)    - enable pull-down resistor
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLNONE(3)    - disable all pull resistors
*                       -> DebounceTimeout     The debounce time in units of 10 microseconds.
*                       -> VendorData
*                       -> VendorDataLength
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS EnableInterrupt(PVOID Context, PGPIO_ENABLE_INTERRUPT_PARAMETERS EnableParametersPtr) {
    NTSTATUS                     ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t*          pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    GPIO_REGS_t*                 pGpioRegs;
    GPIO_PIN_INTERRUPT_CONFIG_t  InterruptConfig;
    IMX_CONNECT_PIN_t            ImxPinConnectParams;
    GPIO_PAD_CONTEXT_t*          pPadContext;
    
    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d_IO%02d Mode: %s, Polarity: %s, Pull: %s, Debounce: %d [mA], VndData 0x%016p, size: %d", EnableParametersPtr->BankId + 1, EnableParametersPtr->PinNumber, \
        ToString_kInterruptMode(EnableParametersPtr->InterruptMode), ToString_kInterruptPolarity(EnableParametersPtr->Polarity), ToString_AcpiPullCfg(EnableParametersPtr->PullConfiguration), \
        EnableParametersPtr->DebounceTimeout * 100, EnableParametersPtr->VendorData, EnableParametersPtr->VendorDataLength);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    do {
        CheckBankIdGetRegsOrBreakIfError(EnableParametersPtr->BankId);
        if ((InterruptConfig = ImxGetGpioPinInterruptCfg(EnableParametersPtr->InterruptMode, EnableParametersPtr->Polarity)) == GPIO_PIN_INTERRUPT_CONFIG_ERROR) {
            DBG_PRINT_ERROR("Unsupported interrupt configuration");
            ntStatus = STATUS_INVALID_PARAMETER;
            break;
        }
        ImxPinConnectParams.pDrvContext       = pDrvContext;
        ImxPinConnectParams.BankId            = EnableParametersPtr->BankId;
        ImxPinConnectParams.PinNumber         = EnableParametersPtr->PinNumber;
        ImxPinConnectParams.PinCount          = 1;
        ImxPinConnectParams.ConnectMode       = ConnectModeInput;                             /* Set Pad.Cfg = Normal */
        ImxPinConnectParams.PullConfiguration = EnableParametersPtr->PullConfiguration;
        ImxPinConnectParams.VendorData        = EnableParametersPtr->VendorData;
        ImxPinConnectParams.VendorDataLength  = EnableParametersPtr->VendorDataLength;
        ImxPinConnectParams.Mux               = IOMUXC_PAD_ALT_GPIO_AUTOSELECT;               /* Get GPIO ALT mux value from ACPI */
        ImxPinConnectParams.DriveStrength     = GPIO_DRIVE_STRENGTH_DEFAULT;                  /* Do not update drive stregth      */
        pPadContext = pDrvContext->BankContextArray[ImxPinConnectParams.BankId]->PadContextArray[EnableParametersPtr->PinNumber];
        if (!pPadContext->State.IsConected) {
            DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d Pin is not connected. Calling ImxConnectIoPin() method", EnableParametersPtr->BankId + 1, EnableParametersPtr->PinNumber);
            if (!NT_SUCCESS(ntStatus = ImxConnectIoPin(&ImxPinConnectParams, CFG_PIN_CMD_SET, 0))) {
                DBG_PRINT_ERROR("Unsupported interrupt configuration");
                break;
            }
        } else {
            DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d Pin is used as GPIO, only interrupts will be enabled", EnableParametersPtr->BankId + 1, EnableParametersPtr->PinNumber);
        }
        pPadContext->State.IsInterruptEnabled = 1;
        DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d ICR[%d]: 0x%08X (Before change)", EnableParametersPtr->BankId + 1, EnableParametersPtr->PinNumber, EnableParametersPtr->PinNumber, pGpioRegs->ICRn[EnableParametersPtr->PinNumber].R);
        pDrvContext->BankContextArray[EnableParametersPtr->BankId]->VIRT_IER.R |= 1 << EnableParametersPtr->PinNumber;
        pGpioRegs->ICRn[EnableParametersPtr->PinNumber].R = GPIO_ICRn_ISF_MASK;  /* Disable interrupt and clear interrupt flag */
        pGpioRegs->ICRn[EnableParametersPtr->PinNumber].R = InterruptConfig;     /* Configure and Enable interrupt */
        DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d ICR[%d]: 0x%08X (After  change)", EnableParametersPtr->BankId + 1, EnableParametersPtr->PinNumber, EnableParametersPtr->PinNumber, pGpioRegs->ICRn[EnableParametersPtr->PinNumber].R);
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d_IO%02d", EnableParametersPtr->BankId + 1, EnableParametersPtr->PinNumber, ntStatus);
    return ntStatus;
}

/**
* Disables interrupts on a general-purpose I/O (GPIO) pin that is configured as an interrupt input.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   DisableParametersPtr -> BankId              The identifier for the bank of GPIO pins.
*                        -> PinNumber           The bank-relative pin number.
*                        -> Flags               No flags are currently defined for this operation.
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS DisableInterrupt(PVOID Context, PGPIO_DISABLE_INTERRUPT_PARAMETERS DisableParametersPtr) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;
    GPIO_REGS_t*        pGpioRegs;
    GPIO_PAD_CONTEXT_t* pPadContext;
    PIN_NUMBER          PinNumber = DisableParametersPtr->PinNumber;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d_IO%02d", DisableParametersPtr->BankId + 1, PinNumber);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    do {
        CheckBankIdGetRegsOrBreakIfError(DisableParametersPtr->BankId);
        pPadContext = pDrvContext->BankContextArray[DisableParametersPtr->BankId]->PadContextArray[PinNumber];
        if (!pPadContext->State.IsConected) {
            ImxPinConnectParams.pDrvContext = pDrvContext;
            ImxPinConnectParams.BankId      = DisableParametersPtr->BankId;
            ImxPinConnectParams.PinNumber   = PinNumber;
            DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d Pin is not used as GPIO. Calling ImxConnectIoPin(SET_DEFAULT) method", DisableParametersPtr->BankId + 1, PinNumber);
            if (!NT_SUCCESS(ntStatus = ImxConnectIoPin(&ImxPinConnectParams, CFG_PIN_CMD_SET_DEFAULT, 0))) {
                DBG_PRINT_ERROR("Unsupported interrupt configuration");
                break;
            }
        } else {
            DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d Pin is used as GPIO, only interrupts will be disabled", DisableParametersPtr->BankId + 1, PinNumber);
        }
        DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d ICR[%d]: 0x%08X (Before change)", DisableParametersPtr->BankId + 1, PinNumber, PinNumber, pGpioRegs->ICRn[PinNumber].R);
        pGpioRegs->ICRn[PinNumber].R = GPIO_ICRn_ISF_MASK;  /* Disable interrupt and clear interrupt flag */
        pDrvContext->BankContextArray[DisableParametersPtr->BankId]->VIRT_IER.R &= (UINT32)1 << PinNumber;
        pPadContext->State.IsInterruptEnabled = 0;
        DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d ICR[%d]: 0x%08X (After change)", DisableParametersPtr->BankId + 1, PinNumber, PinNumber, pGpioRegs->ICRn[PinNumber].R);
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d_IO%02d", DisableParametersPtr->BankId + 1, PinNumber, ntStatus);
    return ntStatus;
}

/**
*  Unmasks the interrupt on a general-purpose I/O (GPIO) pin that is configured as an interrupt input.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   UnmaskParametersPtr -> BankId              The identifier for the bank of GPIO pins.
*                       -> PinNumber           The bank-relative pin number.
*                       -> Flags               No flags are currently defined for this operation.
*                       -> InterruptMode       LevelSensitive(level-triggered) or Latched(edge-triggered)
*                       -> Polarity            LevelSensitive: active-high
*                                                              active-low
*                                              Latched:        on the rising edge
*                                                              on the falling edge
*                                                              or on both edges
*                       -> PullConfiguration   The pin pull configuration as specified in the ACPI FW descriptor.
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLDEFAULT(0) - use the SOC-defined power-on default pull configuration
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLUP(1)      - enable pull-up resistor
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLDOWN(2)    - enable pull-down resistor
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLNONE(3)    - disable all pull resistors
*                       -> DebounceTimeout     The debounce time in units of 10 microseconds.
*                       -> VendorData
*                       -> VendorDataLength
*
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS UnmaskInterrupt(PVOID Context, PGPIO_ENABLE_INTERRUPT_PARAMETERS UnmaskParametersPtr) {
    NTSTATUS                     ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t*          pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    GPIO_REGS_t*                 pGpioRegs;
    GPIO_PIN_INTERRUPT_CONFIG_t  InterruptConfig;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d_IO%02d Mode: %s, Polarity: %s, Pull: %s, Debounce: %d [mA], VndData 0x%016p, size: %d", UnmaskParametersPtr->BankId + 1, UnmaskParametersPtr->PinNumber, \
        ToString_kInterruptMode(UnmaskParametersPtr->InterruptMode), ToString_kInterruptPolarity(UnmaskParametersPtr->Polarity), ToString_AcpiPullCfg(UnmaskParametersPtr->PullConfiguration), \
        UnmaskParametersPtr->DebounceTimeout * 100, UnmaskParametersPtr->VendorData, UnmaskParametersPtr->VendorDataLength);
    do {
        CheckBankIdGetRegsOrBreakIfError(UnmaskParametersPtr->BankId);
        if ((InterruptConfig = ImxGetGpioPinInterruptCfg(UnmaskParametersPtr->InterruptMode, UnmaskParametersPtr->Polarity)) == GPIO_PIN_INTERRUPT_CONFIG_ERROR) {
            DBG_PRINT_ERROR("Unsupported interrupt configuration");
            ntStatus = STATUS_INVALID_PARAMETER;
            break;
        }
        DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d ICR[%d]: 0x%08X, ISFR0: 0x%08X (Before change)", UnmaskParametersPtr->BankId + 1, UnmaskParametersPtr->PinNumber, UnmaskParametersPtr->PinNumber, pGpioRegs->ICRn[UnmaskParametersPtr->PinNumber].R, pGpioRegs->ISFR0.R);
        pGpioRegs->ICRn[UnmaskParametersPtr->PinNumber].R = InterruptConfig;     /* Configure and Enable interrupt */
        DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d ICR[%d]: 0x%08X, ISFR0: 0x%08X (After  change)", UnmaskParametersPtr->BankId + 1, UnmaskParametersPtr->PinNumber, UnmaskParametersPtr->PinNumber, pGpioRegs->ICRn[UnmaskParametersPtr->PinNumber].R, pGpioRegs->ISFR0.R);
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d_IO%02d", UnmaskParametersPtr->BankId + 1, UnmaskParametersPtr->PinNumber, ntStatus);
    return ntStatus;
}

/**
* Masks interrupts on a set of general-purpose I/O (GPIO) pins that are configured as interrupt inputs.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   MaskParametersPtr -> BankId              The identifier for the bank of GPIO pins.
*                     -> PinMask             A 64-bit mask to indicate which interrupt pins to mask.
*                     -> FailedMask          A 64-bit mask that identifies the GPIO pins that could not be masked.
*
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS MaskInterrupts(PVOID Context, PGPIO_MASK_INTERRUPT_PARAMETERS MaskParametersPtr) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    GPIO_REGS_t*        pGpioRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, PinMask: 0x%08X", MaskParametersPtr->BankId + 1, (UINT32)MaskParametersPtr->PinMask);
    do {
        CheckBankIdGetRegsOrBreakIfError(MaskParametersPtr->BankId);
        DBG_DEV_PRINT_INFO("GPIO_%d ISFR0: 0x%08X (Before  change)", MaskParametersPtr->BankId + 1, pGpioRegs->ISFR0.R);
        pGpioRegs->GICLR.R = (UINT32)(MaskParametersPtr->PinMask) & 0xFFFF;          /* Disable interrupt 0 - 15 if required */
        pGpioRegs->GICHR.R = (UINT32)(MaskParametersPtr->PinMask >> 16) & 0xFFFF;    /* Disable interrupt 16 - 31 if required */
        DBG_DEV_PRINT_INFO("GPIO_%d ISFR0: 0x%08X (After  change)", MaskParametersPtr->BankId + 1, pGpioRegs->ISFR0.R);
        MaskParametersPtr->FailedMask = 0;                          /* No error */
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d, FailedMask: 0x%08X", MaskParametersPtr->BankId + 1, (UINT32)MaskParametersPtr->FailedMask, ntStatus);
    return ntStatus;
}

/**
* Queries the state of a set of general-purpose I/O (GPIO) pins that are configured as interrupt inputs.
* On return, the bits that are set in the ActiveMask member should correspond to interrupts that are both enabled and active.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   QueryActiveParametersPtr -> BankId        The identifier for the bank of GPIO pins.
*                            -> EnabledMask   A 64-bit mask to indicate which interrupt are enabled.
*                            -> ActiveMask    A 64-bit mask to indicate which interrupt are active.
*
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS QueryActiveInterrupts(PVOID Context, PGPIO_QUERY_ACTIVE_INTERRUPTS_PARAMETERS QueryActiveParametersPtr) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    GPIO_REGS_t*        pGpioRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, EnabledMask: 0x%08X", QueryActiveParametersPtr->BankId + 1, (UINT32)QueryActiveParametersPtr->EnabledMask);
    do {
        CheckBankIdGetRegsOrBreakIfError(QueryActiveParametersPtr->BankId);
        QueryActiveParametersPtr->ActiveMask = QueryActiveParametersPtr->EnabledMask & pGpioRegs->ISFR0.R;
        DBG_DEV_PRINT_INFO("GPIO_%d ISFR0: 0x%08X", QueryActiveParametersPtr->BankId + 1, pGpioRegs->ISFR0.R);
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d, ActiveMask:  0x%08X", QueryActiveParametersPtr->BankId + 1, (UINT32)QueryActiveParametersPtr->ActiveMask, ntStatus);
    return ntStatus;
}

/**
* Clears active interrupts on a set of general-purpose I/O (GPIO) pins that are configured as interrupt inputs.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   ClearParametersPtr -> BankId           The identifier for the bank of GPIO pins.
*                      -> ClearActiveMask  A 64-bit mask to indicate which interrupt to clear.
*                      -> FailedClearMask  A 64-bit mask that identifies the GPIO pins that could not be cleared.
*
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS ClearActiveInterrupts(PVOID Context, PGPIO_CLEAR_ACTIVE_INTERRUPTS_PARAMETERS ClearParametersPtr) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    GPIO_REGS_t*        pGpioRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d, ClearActiveMask: 0x%08X", ClearParametersPtr->BankId + 1, (UINT32)ClearParametersPtr->ClearActiveMask);
    do {
        CheckBankIdGetRegsOrBreakIfError(ClearParametersPtr->BankId);
        DBG_DEV_PRINT_INFO("GPIO_%d: ISFR0: 0x%08X (Before change)", ClearParametersPtr->BankId + 1, pGpioRegs->ISFR0.R);
        pGpioRegs->ISFR0.R = (UINT32)(ClearParametersPtr->ClearActiveMask); /* Clear interrupts flags */
        DBG_DEV_PRINT_INFO("GPIO_%d: ISFR0: 0x%08X (After  change)", ClearParametersPtr->BankId + 1, pGpioRegs->ISFR0.R);
        ClearParametersPtr->FailedClearMask = 0; /* No error */
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d, FailedClearMask: 0x%08X", ClearParametersPtr->BankId + 1, (UINT32)ClearParametersPtr->FailedClearMask, ntStatus);
    return ntStatus;
}

/** 
* Reconfigures a general-purpose I/O (GPIO) pin that is used as an interrupt input.
* 
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   ReconfigureParametersPtr -> BankId            The identifier for the bank of GPIO pins.
*                            -> PinNumber         The bank-relative pin number.
*                            -> InterruptMode     LevelSensitive(level-triggered) or Latched(edge-triggered)
*                            -> Polarity          LevelSensitive: active-high
*                                                                 active-low
*                                                 Latched:        on the rising edge
*                                                                 on the falling edge
*                                                                 or on both edges
*                            -> Flags             No flags are currently defined for this operation.
* 
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS ReconfigureInterrupt(PVOID Context, PGPIO_RECONFIGURE_INTERRUPTS_PARAMETERS ReconfigureParametersPtr) {
    NTSTATUS                     ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t*          pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    GPIO_REGS_t*                 pGpioRegs;
    GPIO_PIN_INTERRUPT_CONFIG_t  InterruptConfig;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d_IO%02d Mode: %s, Polarity: %s", ReconfigureParametersPtr->BankId + 1, ReconfigureParametersPtr->PinNumber, ToString_kInterruptMode(ReconfigureParametersPtr->InterruptMode), ToString_kInterruptPolarity(ReconfigureParametersPtr->Polarity));
    do {
        CheckBankIdGetRegsOrBreakIfError(ReconfigureParametersPtr->BankId);
        if ((InterruptConfig = ImxGetGpioPinInterruptCfg(ReconfigureParametersPtr->InterruptMode, ReconfigureParametersPtr->Polarity)) == GPIO_PIN_INTERRUPT_CONFIG_ERROR) {
            DBG_PRINT_ERROR("Unsupported interrupt configuration");
            ntStatus = STATUS_INVALID_PARAMETER;
            break;
        }
        pGpioRegs->ICRn[ReconfigureParametersPtr->PinNumber].R = GPIO_ICRn_ISF_MASK;  /* Disable interrupt and clear interrupt flag */
        pGpioRegs->ICRn[ReconfigureParametersPtr->PinNumber].R = InterruptConfig;     /* Configure and Enable interrupt */
    }  while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d_IO%02d", ReconfigureParametersPtr->BankId + 1, ReconfigureParametersPtr->PinNumber, ntStatus);
    return ntStatus;
}

/**
* Queries the state of a set of general-purpose I/O (GPIO) pins to determine which pins are both configured as interrupt inputs and enabled for interrupts.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   QueryEnabledParametersPtr -> BankId        The identifier for the bank of GPIO pins.
*                             -> EnabledMask   A 64-bit mask to indicate which interrupt are enabled.
*
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS QueryEnabledInterrupts(PVOID Context, PGPIO_QUERY_ENABLED_INTERRUPTS_PARAMETERS QueryEnabledParametersPtr) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO_%d", QueryEnabledParametersPtr->BankId);
    do {
        CheckBankIdAndBreakIfError(QueryEnabledParametersPtr->BankId);
        QueryEnabledParametersPtr->EnabledMask = pDrvContext->BankContextArray[QueryEnabledParametersPtr->BankId]->VIRT_IER.R;
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO_%d, EnabledMask: 0x%08X", QueryEnabledParametersPtr->BankId + 1, (UINT32)QueryEnabledParametersPtr->EnabledMask, ntStatus);
    return ntStatus;
}

/**
*  Retrieves the hardware attributes of the general-purpose I/O (GPIO) controller.
*
* Params:
*   Context:                   A pointer to the GPIO controller driver's device context.
*   ControllerInformationPtr:  A pointer to a caller-allocated CLIENT_CONTROLLER_BASIC_INFORMATION structure.
* 
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS QueryControllerBasicInformation(PVOID Context, PCLIENT_CONTROLLER_BASIC_INFORMATION ControllerInformationPtr) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;

    DBG_DEV_METHOD_BEG();
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    ControllerInformationPtr->Version                               = GPIO_CONTROLLER_BASIC_INFORMATION_VERSION;
    ControllerInformationPtr->Size                                  = sizeof(*ControllerInformationPtr);
    ControllerInformationPtr->TotalPins                             = (USHORT)(pDrvContext->GPIO_BankCount * GPIO_MAX_PINS_PER_BANK);
    ControllerInformationPtr->NumberOfPinsPerBank                   = GPIO_MAX_PINS_PER_BANK;
    ControllerInformationPtr->Flags.MemoryMappedController          = TRUE;
    ControllerInformationPtr->Flags.ActiveInterruptsAutoClearOnRead = FALSE;
    ControllerInformationPtr->Flags.FormatIoRequestsAsMasks         = TRUE;   /* ReadGpioPinsUsingMask() and WriteGpioPinsUsingMask() are caled instead of ReadGpioPins() WriteGpioPins() */
    ControllerInformationPtr->Flags.DeviceIdlePowerMgmtSupported    = FALSE;
    ControllerInformationPtr->Flags.BankIdlePowerMgmtSupported      = FALSE;
    ControllerInformationPtr->Flags.EmulateDebouncing               = TRUE;
    ControllerInformationPtr->Flags.EmulateActiveBoth               = TRUE;
    ControllerInformationPtr->Flags.IndependentIoHwSupported        = TRUE;
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
*  Retrieves the hardware attributes of the general-purpose I/O (GPIO) controller.
*
* Params:
*   Context:             A pointer to the GPIO controller driver's device context.
*   RestoreContext:      Whether the client driver should restore the GPIO controller to a previously saved hardware context.
*   PreviousPowerState:  The previous device power state.
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS StartController(PVOID Context, BOOLEAN RestoreContext, WDF_POWER_DEVICE_STATE PreviousPowerState) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    GPIO_REGS_t*        pGpioRegs;
 
    DBG_DEV_METHOD_BEG_WITH_PARAMS("RestoreContext: %d, PreviousPowerState: %s", RestoreContext, ToString_WdfPowerDeviceState(PreviousPowerState));
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    UNREFERENCED_PARAMETER(RestoreContext);
    UNREFERENCED_PARAMETER(PreviousPowerState);
    for (ULONG bankId = 0; bankId < pDrvContext->GPIO_BankCount; bankId++) {
        pGpioRegs = pDrvContext->BankContextArray[bankId]->pGpioRegs;
        /* Reset bank interrupts configurations */
        pGpioRegs->PIDR.R = 0xFFFF;                /* Disable input functionality */
        pGpioRegs->GICLR.R = GPIO_ICRn_ISF_MASK | 0xFFFF;
        pGpioRegs->GICHR.R = GPIO_ICRn_ISF_MASK | 0xFFFF;
    } /* for (ULONG bankId = ...) */
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
*  Performs operations that are needed to prepare the general-purpose I/O (GPIO) controller device to exit the D0 power state.
*
* Params:
*   Context:      A pointer to the GPIO controller driver's device context.
*   SaveContext:  Whether the client driver should save the current hardware context of the GPIO controller device.
*   TargetState:  The target device power state.
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS StopController(PVOID Context, BOOLEAN SaveContext, WDF_POWER_DEVICE_STATE TargetState) {
    NTSTATUS  ntStatus = STATUS_SUCCESS;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("SaveContext: %d, TargetState: %s", SaveContext, ToString_WdfPowerDeviceState(TargetState));
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    UNREFERENCED_PARAMETER(Context);
    UNREFERENCED_PARAMETER(SaveContext);
    UNREFERENCED_PARAMETER(TargetState);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
*  Performs any operations that are needed to make the general - purpose I / O(GPIO) controller ready to be accessed by the GPIO controller driver.
*
* Params:
*   Device:       Device handle.
*   Context:      A pointer to the GPIO controller driver's device context.
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS PrepareController(WDFDEVICE Device, PVOID Context, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated) {
    NTSTATUS                         ntStatus               = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t*              pDrvContext            = (GPIO_DRV_CONTEXT_t *)Context;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR  pResDscr               = NULL;
    ULONG                            AcpiDataSize           = 0;
    ULONG                            uInterruptCount        = 0;
    ULONG                            uGpioBankCount         = 0;
    ULONG                            uResourceCount;
    unsigned int                     PadCount;
    unsigned int                     PinCount;
    GPIO_PAD_CONTEXT_t*              pPadContext;
    size_t                           ExtContextSize;
    ACPI_PAD_CONFIG_t*               AcpiPadConfigArray;

    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    UNREFERENCED_PARAMETER(ResourcesRaw);
    pDrvContext->pACPI_UtilsContext.Pdo = WdfDeviceWdmGetPhysicalDevice(Device);     
    pDrvContext->pACPI_UtilsContext.MemPoolTag = GPIO_MEM_TAG_ACPI;
    if (!NT_SUCCESS(ntStatus = Acpi_Init(&pDrvContext->pACPI_UtilsContext))) {
        ntStatus = STATUS_SUCCESS;
        DBG_DEV_PRINT_INFO("Acpi_Init() failed");
    }
    /* Check interrupt and memory resources. There must be at least two memory resourcs. 
       One or more for GPIO device and one for IOMUXC device. Number of memmory resources must be equal to number of interrupts */
    uResourceCount = WdfCmResourceListGetCount(ResourcesTranslated);
    for (unsigned int i = 0; i < uResourceCount; i++) {
        pResDscr = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        switch (pResDscr->Type) {
           case CmResourceTypeInterrupt:
               uInterruptCount++;
               break;
           case CmResourceTypeMemory:
               uGpioBankCount++;
               break;
           default:
               break;
        }
    }
    do {
        if (uGpioBankCount < 2) {
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR("ACPI data error: At least one GPIO device and only one IOMUX device is required.");
            break;
        }
        uGpioBankCount--;  /* Decremnt gpio bank/device counter because of the last memory resource is IOMUXC device (not GPIO) */
        if (uInterruptCount != uGpioBankCount) {
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR("ACPI data error: Number of GPIO device interrupts vectors != number of GPIO devices (%d != %d).", uInterruptCount, uGpioBankCount);
            break;
        }
        if (uGpioBankCount > GPIO_MAX_BANKS) {
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR("ACPI data error: Number of GPIO devices > number of devices supported by this driver (%d > %d).", uGpioBankCount, GPIO_MAX_BANKS);
            break;
        }
        Acpi_GetBufferPropertyAddress(&pDrvContext->pACPI_UtilsContext, "Pad_Config", (UINT8 **)&AcpiPadConfigArray, &AcpiDataSize);
        if (AcpiDataSize % sizeof(ACPI_PAD_CONFIG_t) != 0) {
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR("ACPI data error: SizeOf(Pad_Config) property in ACPI must be multiple of sizeof(ACPI_PAD_CONFIG_t(%d)). Returned size: %d.", (UINT32)sizeof(ACPI_PAD_CONFIG_t), AcpiDataSize);
            break;
        }
        PadCount = AcpiDataSize / sizeof(ACPI_PAD_CONFIG_t);
        if (PadCount > (uGpioBankCount * GPIO_MAX_PINS_PER_BANK)) {
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR("ACPI data error: Number of pads(pins) in Pad_Config[%d] > %d (GPIO count (%d) * Pins per GPIO(%d)).", PadCount, uGpioBankCount * GPIO_MAX_PINS_PER_BANK, uGpioBankCount, GPIO_MAX_PINS_PER_BANK);
            break;
        }
        Acpi_GetBufferPropertyAddress(&pDrvContext->pACPI_UtilsContext, "Pin_Config", (UINT8 **)&pDrvContext->AcpiPinConfigArray, &AcpiDataSize);
        if (AcpiDataSize % sizeof(ACPI_PIN_CONFIG_t) != 0) {
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR("ACPI data error: SizeOf(Pad_Config) property must be multiple of sizeof(ACPI_PAD_CONFIG_t(%d)). Returned size: %d.", (UINT32)sizeof(ACPI_PIN_CONFIG_t), AcpiDataSize);
            break;
        }
        PinCount = AcpiDataSize / sizeof(ACPI_PIN_CONFIG_t);
        DBG_DEV_PRINT_INFO("Found %d GPIO(s), %d pins and %d input selects in ACPI", uGpioBankCount, PadCount, PinCount);
        if (PinCount > 255) { /* UINT8 type is used for index into AcpiPinConfigArray[], Index zero is reserved */
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR("ACPI data error: Pin_Config[%d] - Only 256 item in Pin_Config is supported by the driver", PinCount);
        }
//        ExtContextSize = (uGpioBankCount * sizeof(GPIO_BANK_CONTEXT_t)) + (PadCount * sizeof(GPIO_PAD_CONTEXT_t)) + (PinCount * sizeof(ACPI_PIN_CONFIG_t));
        ExtContextSize = (uGpioBankCount * sizeof(GPIO_BANK_CONTEXT_t)) + (PadCount * sizeof(GPIO_PAD_CONTEXT_t));
        DBG_DEV_PRINT_INFO("Driver context size is %d. Allocating %d bytes for extended driver contex.", (UINT32)sizeof(GPIO_DRV_CONTEXT_t) ,(UINT32)ExtContextSize);
        if ((pDrvContext->pExtDriverContext = ExAllocatePoolWithTag(NonPagedPool, ExtContextSize, GPIO_MEM_TAG_DRV)) == NULL) {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "ExAllocatePoolWithTag(NonPagedPool, %d , GPIO_MEM_TAG_DRV)", (UINT32)ExtContextSize);
            break;
        }
        RtlZeroMemory(pDrvContext->pExtDriverContext, ExtContextSize);
        for (unsigned int i = 0; i < uResourceCount; i++) {
            pResDscr = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
            if (pResDscr->Type == CmResourceTypeMemory) {
                if (pDrvContext->GPIO_BankCount < uGpioBankCount) { /* Map GPIO device/bank registers */
                    pDrvContext->BankContextArray[pDrvContext->GPIO_BankCount] = &((GPIO_BANK_CONTEXT_t *)(pDrvContext->pExtDriverContext))[pDrvContext->GPIO_BankCount];
                    DBG_DEV_PRINT_INFO("GPIO_%d_RegPhyAddr: 0x%016llX, GpioRegsSize: 0x%08X", pDrvContext->GPIO_BankCount + 1, pResDscr->u.Memory.Start.QuadPart, pResDscr->u.Memory.Length);
                    if ((pDrvContext->BankContextArray[pDrvContext->GPIO_BankCount]->pGpioRegs = (GPIO_REGS_t*)MmMapIoSpaceEx(pResDscr->u.Memory.Start, pResDscr->u.Memory.Length, PAGE_READWRITE | PAGE_NOCACHE)) == NULL) {
                        ntStatus = STATUS_INSUFFICIENT_RESOURCES;
                        DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "MmMapIoSpaceEx() failed.");
                        break;
                    }
                    pDrvContext->BankContextArray[pDrvContext->GPIO_BankCount++]->GpioRegsSize = pResDscr->u.Memory.Length;
                } else {
                    DBG_DEV_PRINT_INFO("IOMUXC_RegPhyAddr: 0x%016llX, GpioRegsSize: 0x%08X", pResDscr->u.Memory.Start.QuadPart, pResDscr->u.Memory.Length);
                    if ((pDrvContext->pIoMuxRegs = MmMapIoSpaceEx(pResDscr->u.Memory.Start, pResDscr->u.Memory.Length, PAGE_READWRITE | PAGE_NOCACHE)) == NULL) {
                        ntStatus = STATUS_INSUFFICIENT_RESOURCES;
                        DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "MmMapIoSpaceEx() failed.");
                        break;
                    }
                    pDrvContext->IoMuxRegsSize = pResDscr->u.Memory.Length;
                }
            }
        }
        if (!NT_SUCCESS(ntStatus)) {
            break;
        }
        for (unsigned int bankId = 0; bankId < uGpioBankCount; bankId++) {
            GPIO_REGS_t* pGpioRegs = pDrvContext->BankContextArray[bankId]->pGpioRegs;
            /* Save original value of GPIO bank pins settings (Data direction, Data input disable) */
            pDrvContext->BankContextArray[bankId]->PDDR_DefaultValue.R = pGpioRegs->PDDR.R;
            pDrvContext->BankContextArray[bankId]->PIDR_DefaultValue.R = pGpioRegs->PIDR.R;
            DBG_DEV_PRINT_INFO("*** GPIO_%d Register dump *******************************", bankId + 1);
            DBG_DEV_PRINT_INFO("VERID: 0x%08X", pGpioRegs->VERID.R);
            DBG_DEV_PRINT_INFO("PARAM: 0x%08X", pGpioRegs->PARAM.R);
            DBG_DEV_PRINT_INFO("PCNS:  0x%08X", pGpioRegs->PCNS.R);
            DBG_DEV_PRINT_INFO("ICNS:  0x%08X", pGpioRegs->ICNS.R);
            DBG_DEV_PRINT_INFO("PDDR:  0x%08X (Data direction, 0=Input, 1=Output)", pGpioRegs->PDDR.R);
            DBG_DEV_PRINT_INFO("PDOR:  0x%08X (Data output)", pGpioRegs->PDOR.R);
            DBG_DEV_PRINT_INFO("PIDR:  0x%08X (Input disable, 1=Disabled, 0=Enabled)", pGpioRegs->PIDR.R);
            DBG_DEV_PRINT_INFO("PDIR:  0x%08X (Data input)", pGpioRegs->PDIR.R);
            DBG_DEV_PRINT_INFO("ISFR0: 0x%08X", pGpioRegs->ISFR0.R);
            DBG_DEV_PRINT_INFO("ISFR1: 0x%08X", pGpioRegs->ISFR1.R);
        }
        pPadContext = (GPIO_PAD_CONTEXT_t*)((UINT8*)(pDrvContext->pExtDriverContext) + (uGpioBankCount * sizeof(GPIO_BANK_CONTEXT_t)));
        for (unsigned int i = 0; i < PadCount; i++) {
            ACPI_PAD_CONFIG_t*   pPadInfo   = &AcpiPadConfigArray[i];
            unsigned int         BankNumber = pPadInfo->PadIdx.B.BankNumber;
            unsigned int         PinNumber  = pPadInfo->PadIdx.B.PinNumber;             
            GPIO_BANK_CONTEXT_t* pBankContext;

            if ((BankNumber >= uGpioBankCount) || (PinNumber >= GPIO_MAX_PINS_PER_BANK)) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR("ACPI data error: Pin_Config[%d] - BankNumber(%d) > Bank count(%d) or PinNumber(%d) > Max pin number(%d).", i, BankNumber - 1, uGpioBankCount - 1, PinNumber, GPIO_MAX_PINS_PER_BANK - 1);
                break;
            }
            if (!(pDrvContext->BankContextArray[BankNumber]->pGpioRegs->PCNS.R & (1 << PinNumber))) {
                DBG_PRINT_ERROR("GPIO_%d_IO%02d: Pin is not present or is not enabled for No-Secure access.", BankNumber + 1, PinNumber);
                ntStatus = STATUS_INVALID_PARAMETER;
            }
            pBankContext = pDrvContext->BankContextArray[BankNumber];
            if (pBankContext->PadContextArray[PinNumber] != NULL) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR("ACPI data error: ACPI Pin_Config[%d]: GPIO_%d PinNumber(%d) is already initialized.", i, BankNumber + 1, PinNumber);
                break;
            }
            /* Save original value of PAD settings (Mux & pad) */
            pBankContext->PadContextArray[PinNumber] = pPadContext;
            pPadContext->pPadMuxReg                = (IOMUX_SW_MUX_CTL_t *)(PVOID) & (((UINT8 *)(pDrvContext->pIoMuxRegs))[pPadInfo->PadMuxRegOffset]);
            volatile UINT32 MuxRegVal = pPadContext->pPadMuxReg->R;
            pPadContext->PadMuxRegDefaultValue     = (UINT8)MuxRegVal;
            pPadContext->pPadCtlReg                = (IOMUX_SW_PAD_CTL_t *)(PVOID) & (((UINT8 *)(pDrvContext->pIoMuxRegs))[pPadInfo->PadCtlRegOffset]);
            pPadContext->PadCtlRegDefautlValue.R   = pPadContext->pPadCtlReg->R;
            pPadContext->State.PadMuxGpioValue     = pPadInfo->PadMuxGpioAltValue;
            DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d PadMuxOff: 0x%04X, val: 0x%08X, PadCtlOff: 0x%04X, val: 0x%08X, GPIO_Alt: %d", BankNumber + 1, PinNumber, pPadInfo->PadMuxRegOffset, pPadContext->pPadMuxReg->R, pPadInfo->PadCtlRegOffset, pPadContext->pPadCtlReg->R, pPadContext->State.PadMuxGpioValue);
            pPadContext++;
        }
        if (!NT_SUCCESS(ntStatus)) {
            break;
        }
        for (unsigned int i = 0; i < PinCount; i++) {
            ACPI_PIN_CONFIG_t*    pAcpiInputSel = &pDrvContext->AcpiPinConfigArray[i];
            unsigned int          BankNumber    = pAcpiInputSel->PadIdx.B.BankNumber;
            unsigned int          PinNumber     = pAcpiInputSel->PadIdx.B.PinNumber;
            GPIO_BANK_CONTEXT_t*  pBankContext;

            if ((BankNumber >= uGpioBankCount) || (PinNumber >= GPIO_MAX_PINS_PER_BANK)) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR("ACPI data error: Pin_Config[%d] - BankNumber(%d) > Bank count(%d) or PinNumber(%d) > Max pin number(%d).", i, BankNumber + 1, uGpioBankCount - 1, PinNumber, GPIO_MAX_PINS_PER_BANK - 1);
                break;
            }
            pBankContext = pDrvContext->BankContextArray[BankNumber];
            pPadContext  = pBankContext->PadContextArray[PinNumber];
            if (pPadContext == NULL) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR("ACPI data error: Pin_Config[%d] - Pad GPIO_(%d)_IO%02d is not in Pad_Config[].", i, BankNumber + 1, PinNumber);
                break;
            }
            if (pAcpiInputSel->PadMuxAltValue & ~0x07) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR("ACPI data error: Pin_Config[%d] - Pad GPIO_(%d)_IO%02d ALT value (%d) > 7.", i, BankNumber + 1, PinNumber, pAcpiInputSel->PadMuxAltValue);
                break;
            }
            if (pPadContext->PadInputMuxOffsetIdx[pAcpiInputSel->PadMuxAltValue] != 0) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR("ACPI data error: Pad GPIO_(%d)_IO%02d ALT%d is alreday used by Pin_Config[%d].", BankNumber + 1, PinNumber, pAcpiInputSel->PadMuxAltValue, i);
                break;
            }
            pPadContext->PadInputMuxOffsetIdx[pAcpiInputSel->PadMuxAltValue] = (UINT8)(i + 1); /* Remember one-based index into the AcpiPinConfigArray[], index = 0 -> No input muxing for Pad_x_ALT_y */
        }
        if (!NT_SUCCESS(ntStatus)) {
            break;
        }
#ifdef DBG_DEV
        for (unsigned int BankNumber = 0; BankNumber < uGpioBankCount; BankNumber++) {
            for (unsigned int PinNumber = 0; PinNumber < GPIO_MAX_PINS_PER_BANK; PinNumber++) {
                GPIO_BANK_CONTEXT_t* pBankContext;
                pBankContext = pDrvContext->BankContextArray[BankNumber];
                pPadContext  = pBankContext->PadContextArray[PinNumber];
                if (pPadContext != NULL) {
                    for (unsigned int i = 0; i < 8; i++) {
                        unsigned int AcpiPinConfigArrayIdx = pPadContext->PadInputMuxOffsetIdx[i];
                        if (AcpiPinConfigArrayIdx) {
                            ACPI_PIN_CONFIG_t*         pAcpiInputSel = &pDrvContext->AcpiPinConfigArray[AcpiPinConfigArrayIdx - 1]; /* Index is one-based, Array is zero-based */
                            IOMUX_IPP_INPUT_SELECT_t*  pInputSelReg  = (IOMUX_IPP_INPUT_SELECT_t *)(PVOID) & (((UINT8 *)(pDrvContext->pIoMuxRegs))[pAcpiInputSel->InputSelectCtlRegOffset]);
                            DBG_DEV_PRINT_INFO("GPIO_%d_IO%02d ALT:%d SelIpnutDaisyRegOff: 0x%04X, ALT%d, InpMuxAlt: %d", BankNumber + 1, PinNumber, i, pAcpiInputSel->InputSelectCtlRegOffset, pAcpiInputSel->InputSelectMuxAltValue, pInputSelReg->R);
                        }
                    }
                }
            }
        }
#endif
    } while (0);
    if (!NT_SUCCESS(ntStatus)) {
        ReleaseResources(pDrvContext);
    }
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/**
*  Performs operations that are needed when the general-purpose I/O (GPIO) controller device is no longer accessible.
*
* Params:
*   Device:       Device handle.
*   Context:      A pointer to the GPIO controller driver's device context.
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_ 
NTSTATUS ReleaseController(WDFDEVICE Device, PVOID Context) {
    NTSTATUS            ntStatus = STATUS_SUCCESS;
    GPIO_DRV_CONTEXT_t* pDrvContext; 

    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    UNREFERENCED_PARAMETER(Device);
    pDrvContext = (GPIO_DRV_CONTEXT_t *)Context;
    ReleaseResources(pDrvContext);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/*++
Routine Description:
    EvtDeviceAdd is called by the framework in response to AddDevice call from the PnP manager. We create and initialize a device object
    to represent a new instance of the device.
Arguments:
    Driver     - Handle to a framework driver object created in DriverEntry
    DeviceInit - Pointer to a framework-allocated WDFDEVICE_INIT structure.
Return Value:  ntStatus
--*/
NTSTATUS EvtDriverDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit) {
    NTSTATUS               ntStatus;
    WDF_OBJECT_ATTRIBUTES  wdfDeviceAttributes;
    WDFDEVICE              wdfDevice;

    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    do {
        if (!NT_SUCCESS(ntStatus = GPIO_CLX_ProcessAddDevicePreDeviceCreate(Driver, DeviceInit, &wdfDeviceAttributes))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "GPIO_CLX_ProcessAddDevicePreDeviceCreate() failed");
            break;
        }
        if (!NT_SUCCESS(ntStatus = WdfDeviceCreate(&DeviceInit, &wdfDeviceAttributes, &wdfDevice))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfDeviceCreate() failed");
            break;
        }
        if (!NT_SUCCESS(ntStatus = GPIO_CLX_ProcessAddDevicePostDeviceCreate(Driver, wdfDevice))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "GPIO_CLX_ProcessAddDevicePostDeviceCreate() failed");
            break;
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}

/*++
Routine Description:
    Free all the resources allocated in DriverEntry.
Arguments:
    DriverObject - handle to a WDF Driver object.
Return Value:  VOID.
--*/
VOID EvtDriverUnload(_In_ WDFOBJECT DriverObject) {
    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    GPIO_CLX_UnregisterClient(DriverObject);
    DBG_DEV_METHOD_END();
}

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
_Use_decl_annotations_
NTSTATUS DriverEntry(DRIVER_OBJECT* DriverObjectPtr, UNICODE_STRING* RegistryPathPtr) {
    NTSTATUS              ntStatus  = STATUS_SUCCESS;
    WDFDRIVER             wdfDriver = NULL;
    WDF_OBJECT_ATTRIBUTES wdfObjectAttributes;
    WDF_DRIVER_CONFIG     wdfDriverConfig;

    PAGED_CODE();
    DBG_DEV_METHOD_BEG_WITH_PARAMS("Driver: 0x%016p, '%S')", DriverObjectPtr, ((PUNICODE_STRING)RegistryPathPtr)->Buffer);
    DBG_DEV_PRINT_INFO("***********************************************************************************");
    DBG_DEV_PRINT_INFO("*** NXP GPIO miniport driver, date: %s %s                        ***", __DATE__, __TIME__);
    DBG_DEV_PRINT_INFO("***********************************************************************************");
    do {
        WDF_OBJECT_ATTRIBUTES_INIT(&wdfObjectAttributes);
        WDF_DRIVER_CONFIG_INIT(&wdfDriverConfig, EvtDriverDeviceAdd);
        wdfDriverConfig.DriverPoolTag = GPIO_MEM_TAG_DRV;
        wdfDriverConfig.EvtDriverUnload = EvtDriverUnload;
        if (!NT_SUCCESS(ntStatus = WdfDriverCreate(DriverObjectPtr, RegistryPathPtr, &wdfObjectAttributes, &wdfDriverConfig, &wdfDriver))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "WdfDriverCreate() failed");
            break;
        }
        /* Register with GpioClx */
        GPIO_CLIENT_REGISTRATION_PACKET registrationPacket = {
            GPIO_CLIENT_VERSION,
            sizeof(GPIO_CLIENT_REGISTRATION_PACKET),
            0,          /* Flags */
            sizeof(GPIO_DRV_CONTEXT_t),
            0,          /* Reserved */
            PrepareController,
            ReleaseController,
            StartController,
            StopController,
            QueryControllerBasicInformation,
            NULL,    /* CLIENT_QuerySetControllerInformation */
            EnableInterrupt,
            DisableInterrupt,
            UnmaskInterrupt,
            MaskInterrupts,
            QueryActiveInterrupts,
            ClearActiveInterrupts,
            ConnectIoPins,
            DisconnectIoPins,
            NULL,    /*  CLIENT_ReadGpioPins                   */
            NULL,    /*  CLIENT_WriteGpioPins                  */
            NULL,    /*  CLIENT_SaveBankHardwareContext        */
            NULL,    /*  CLIENT_RestoreBankHardwareContext     */
            NULL,    /*  CLIENT_PreProcessControllerInterrupt  */
            NULL,    /*  CLIENT_ControllerSpecificFunction     */
            ReconfigureInterrupt,
            QueryEnabledInterrupts,
            ConnectFunctionConfigPins,
            DisconnectFunctionConfigPins
        }; /* registrationPacket */
        registrationPacket.CLIENT_ReadGpioPinsUsingMask  = ReadGpioPinsUsingMask;
        registrationPacket.CLIENT_WriteGpioPinsUsingMask = WriteGpioPinsUsingMask;
        if (!NT_SUCCESS(ntStatus = GPIO_CLX_RegisterClient(wdfDriver, &registrationPacket, RegistryPathPtr))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "GPIO_CLX_RegisterClient() failed");
            break;
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_STATUS(ntStatus);
    return ntStatus;
}