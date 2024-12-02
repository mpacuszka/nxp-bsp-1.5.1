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

#include "imxgpio_8x.h"
#include "imxgpio_8x_io_map.h"
#include "imx_acpi_utils.h"
#include <svc/scfw.h>
#include <svc/ipc.h>
#include <svc/pad/pad_api.h>
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

/* IMX8 GPIO configuration constants */
#define GPIO_MAX_BANKS              8
#define GPIO_PIN_PER_BANK           32

/* Memory tags for this driver */
#define GPIO_MEM_TAG_ACPI           ((ULONG)'AioG')
#define GPIO_MEM_TAG_DRV            ((ULONG)'DioG')

#define SC_PAD_TYPE_1V8_3V3         0x00
#define SC_PAD_TYPE_3V3             0x01
#define SC_PAD_TYPE_1V8             0x02
#define SC_PAD_TYPE_HSIC            0x03

#define SC_PAD_SET_MUX_MASK         0x80000000U
#define SC_PAD_SET_GP_MASK          0x40000000U                                    
#define SC_PAD_MUX_MASK             0x38000000U
#define SC_PAD_MUX_SHIFT            27U
#define SC_PAD_CONFIG_MASK          0x06000000U
#define SC_PAD_CONFIG_SHIFT         25U
#define SC_PAD_ISO_MASK             0x01800000U
#define SC_PAD_ISO_SHITF            23U
#define SC_PAD_WAKEUP_MASK          0x00380000U
#define SC_PAD_WAKEUP_SHIFT         19U
#define SC_PAD_PULL_MASK            0x00000060U
#define SC_PAD_PULL_SHIFT           5U
#define SC_PAD_DRIVE_STRENGTH_MASK  0x00000007U
#define SC_PAD_DRIVE_STRENGTH_SHIFT 0U

typedef enum IMX_GPIO_PIN_INTERRUPT_CONFIG_e {
    IMX_GPIO_PIN_INTERRUPT_CONFIG_LOW_LEVEL    = 0x0,
    IMX_GPIO_PIN_INTERRUPT_CONFIG_HIGH_LEVEL   = 0x1,
    IMX_GPIO_PIN_INTERRUPT_CONFIG_RISING_EDGE  = 0x2,
    IMX_GPIO_PIN_INTERRUPT_CONFIG_FALLING_EDGE = 0x3,
    IMX_GPIO_PIN_INTERRUPT_CONFIG_ERROR        = 0x4
} IMX_GPIO_PIN_INTERRUPT_CONFIG;

typedef enum IMX_CONNECT_PIN_OPERATION_s {
    IMX_CONNECT_PIN_SET_VALUES  = 0x00,
    IMX_CONNECT_PIN_CHECK_OLNY  = 0x01,
    IMX_CONNECT_PIN_SET_DEFAULT = 0x02
} IMX_CONNECT_PIN_OPERATION_t;

/* ACPI Pad info structure */
#pragma pack(push, 1)
typedef struct GPIO_AcpiPadInfo_s {
    UINT8                       PadId;                                   /* Pad ID (see e.g. imx8qx_pads.h )                            */
    UINT8                       Mux;                                     /* GPIO ALT mux value or 0xFF if pad has no GPIO functionality */
    UINT8                       Type;                                    /* Pad type see #defines                                       */
} GPIO_AcpiPadInfo_t;
#pragma pop()

/* GPIO Bank (Device) context structure */
typedef struct GPIO_BankContext_s {
    GPIO_t*                     pRegs;                                   /* Bank(GPIO device) registers address */
    ULONG                       RegSize;                                 /* Bank(GPIO device) registers size    */
} GPIO_BankContext_t;

/* GPIO Driver context structure */
typedef struct GPIO_DrvContext_s {
    GPIO_BankContext_t          GPIO_BankContextArray[GPIO_MAX_BANKS];   /* Bank(GPIO device) contect array           */
    ULONG                       GPIO_BankCount;                          /* Real bank(GPIO device) count              */
    IMX_ACPI_UTILS_DEV_CONTEXT  ACPI_UtilsContext;                       /* Acpi context                              */
    GPIO_AcpiPadInfo_t*         PAD_AcpiPadInfoArray;                    /* This Pad info array is obtained from ACPI */
    UINT32                      PAD_DefaultPadValueArray[GPIO_PIN_PER_BANK * GPIO_MAX_BANKS];
    sc_ipc_struct_t             SCFW_Handle;
    sc_ipc_id_struct_t          SCFW_Config;
} GPIO_DrvContext_t, * pGPIO_DrvContext_t;

/* IMX GPIO driver PAD configuration structure */
typedef struct IMX_CONNECT_PIN_s {
    GPIO_DrvContext_t* pDrvContext;
    BANK_ID                    BankId;
    PIN_NUMBER                 PinNumber;
    GPIO_CONNECT_IO_PINS_MODE  ConnectMode;
    UCHAR                      PullConfiguration;
    USHORT                     DriveStrength;
    PVOID                      VendorData;
    ULONG                      VendorDataLength;
    UINT32                     GpioPinDir;
    UINT8                      Mux;
} IMX_CONNECT_PIN_t;

#define CheckBankIdGetRegsOrBreakIfError(_BankId)                                                   \
    if (_BankId >= pDrvContext->GPIO_BankCount) {                                                   \
        DBG_PRINT_ERROR("BankId(%d) > GPIO_BankCount(%d)", _BankId, pDrvContext->GPIO_BankCount);   \
        ntStatus = STATUS_INVALID_PARAMETER;                                                        \
        break;                                                                                      \
    }                                                                                               \
    pDevRegs = pDrvContext->GPIO_BankContextArray[_BankId].pRegs;

#define CheckBankIdAndPinCountAndBreakIfError(_BankId, _PinCount)                                 \
    if (_BankId >= pDrvContext->GPIO_BankCount) {                                                 \
        DBG_PRINT_ERROR("BankId(%d) > GPIO_BankCount(%d)", _BankId, _PinCount);                   \
        ntStatus = STATUS_INVALID_PARAMETER;                                                      \
        break;                                                                                    \
    }                                                                                             \
    if (_PinCount > GPIO_PIN_PER_BANK) {                                                          \
        DBG_PRINT_ERROR("PinCount(%d) > GPIO_PIN_PER_BANK(%d)", _PinCount, GPIO_PIN_PER_BANK);    \
        ntStatus = STATUS_INVALID_PARAMETER;                                                      \
        break;                                                                                    \
    }

/**
* Releases all resources allocated by this driver.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*
* Return value: none.
*/
void ReleaseResources(GPIO_DrvContext_t* pDrvContext) {
    DBG_DEV_METHOD_BEG();
    sc_ipc_close(&pDrvContext->SCFW_Handle);
    if (pDrvContext->GPIO_BankCount != 0) {
        for (unsigned int i = 0; i < pDrvContext->GPIO_BankCount; i++) {
            if (pDrvContext->GPIO_BankContextArray[i].pRegs != NULL) {
                MmUnmapIoSpace((void*)pDrvContext->GPIO_BankContextArray[i].pRegs, pDrvContext->GPIO_BankContextArray[i].RegSize);
                DBG_DEV_PRINT_INFO("MmUnmapIoSpace: 0x%016p,  GPIODev_RegSize: 0x%08X", pDrvContext->GPIO_BankContextArray[i].pRegs, pDrvContext->GPIO_BankContextArray[i].RegSize);
            }
        }
    }
    Acpi_Deinit(&pDrvContext->ACPI_UtilsContext);
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
IMX_GPIO_PIN_INTERRUPT_CONFIG ImxGetGpioPinInterruptCfg(KINTERRUPT_MODE InterruptMode, KINTERRUPT_POLARITY Polarity) {
    IMX_GPIO_PIN_INTERRUPT_CONFIG interruptConfig = IMX_GPIO_PIN_INTERRUPT_CONFIG_ERROR;

    switch (InterruptMode) {
    case LevelSensitive:
        switch (Polarity) {
        case InterruptActiveHigh:
            interruptConfig = IMX_GPIO_PIN_INTERRUPT_CONFIG_HIGH_LEVEL;
            break;
        case InterruptActiveLow:
            interruptConfig = IMX_GPIO_PIN_INTERRUPT_CONFIG_LOW_LEVEL;
            break;
        default:
            interruptConfig = IMX_GPIO_PIN_INTERRUPT_CONFIG_ERROR;
        } /* switch (Polarity) */
        break;
    case Latched:
        switch (Polarity) {
        case InterruptRisingEdge:
            interruptConfig = IMX_GPIO_PIN_INTERRUPT_CONFIG_RISING_EDGE;
            break;
        case InterruptFallingEdge:
            interruptConfig = IMX_GPIO_PIN_INTERRUPT_CONFIG_FALLING_EDGE;
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
* 
* Return value: STATUS_SUCCESS, STATUS_INVALID_PARAMETER.
*/
NTSTATUS ImxConnectIoPin(IMX_CONNECT_PIN_t* pImxConnectPin, IMX_CONNECT_PIN_OPERATION_t Operation) {
    NTSTATUS            ntStatus         = STATUS_SUCCESS;
    UINT32              PadValue         = 0;
    UINT32*             pPadDefaultValue = &pImxConnectPin->pDrvContext->PAD_DefaultPadValueArray[(pImxConnectPin->BankId * 32) + pImxConnectPin->PinNumber];
    GPIO_AcpiPadInfo_t* pAcpiPadInfo     = &pImxConnectPin->pDrvContext->PAD_AcpiPadInfoArray[(pImxConnectPin->BankId * 32) + pImxConnectPin->PinNumber];
    sc_pad_t            PadId            = pAcpiPadInfo->PadId;
    sc_err_t            scfwStatus;
    UINT32              OrgPadValue;
    UINT32              DriveStrength_mA;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("Operation %s, PadId %d, GPIO %d, Pin %d, ALT %d, Mode %s, PullCfg %s, DrvStr %d [mA] ,VndData 0x%016p", ToString_SetPinOpMode(Operation), PadId ,pImxConnectPin->BankId, pImxConnectPin->PinNumber, pImxConnectPin->Mux, ToString_ConnectioMode(pImxConnectPin->ConnectMode), ToString_AcpiPullCfg(pImxConnectPin->PullConfiguration), pImxConnectPin->DriveStrength / 100, pImxConnectPin->VendorData);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    do {
        if (*pPadDefaultValue == 0) {
            if ((scfwStatus = sc_pad_get(&pImxConnectPin->pDrvContext->SCFW_Handle, PadId, pPadDefaultValue)) != SC_ERR_NONE) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "Failed to obtain default pad value, sc_pad_get(Hnd: 0x%016p, Pad: %d, Value: 0x%08X) [Scfw error code: %d]", &pImxConnectPin->pDrvContext->SCFW_Handle, PadId, *pPadDefaultValue , scfwStatus);
                break;
            } else {
                DBG_DEV_PRINT_INFO("Default pad value obtained sc_pad_get(Hnd: 0x%016p, Pad: %d, Value: 0x%08X)", &pImxConnectPin->pDrvContext->SCFW_Handle, PadId, *pPadDefaultValue);
                *pPadDefaultValue |= (SC_PAD_SET_MUX_MASK | SC_PAD_SET_GP_MASK); /* Enable MUX and Pad control */
            }
        }
        if (Operation == IMX_CONNECT_PIN_SET_DEFAULT) {
            PadValue = *pPadDefaultValue;
        } else if (pImxConnectPin->VendorData != NULL) {
            if (pImxConnectPin->VendorDataLength != sizeof(UINT32)) {
                DBG_PRINT_ERROR("Vendor data size is %d. It msut be 4 bytes", pImxConnectPin->VendorDataLength);
                break;
            }
            PadValue = *(UINT32*)(pImxConnectPin->VendorData);
        } else {
            /* Set Pin Mux */
            PadValue |= (UINT32)((pImxConnectPin->Mux << SC_PAD_MUX_SHIFT) & SC_PAD_MUX_MASK);
            /* Set Pad Confid */
            switch ((unsigned int)(pImxConnectPin->ConnectMode)) {
            case ConnectModeInput:
                PadValue |= SC_PAD_CONFIG_NORMAL << SC_PAD_CONFIG_SHIFT;
                pImxConnectPin->GpioPinDir = 0;                                        /* Set GPIO direction to input */
                break;
            case ConnectModeOutput:
                PadValue |= SC_PAD_CONFIG_NORMAL << SC_PAD_CONFIG_SHIFT;
                pImxConnectPin->GpioPinDir = (UINT32)(1 << pImxConnectPin->PinNumber); /* Set GPIO direction to output */
                break;
            case (ConnectModeInput | ConnectModeOutput):
                PadValue |= SC_PAD_CONFIG_OUT_IN << SC_PAD_CONFIG_SHIFT;
                pImxConnectPin->GpioPinDir = (UINT32)(1 << pImxConnectPin->PinNumber); /* Set GPIO direction to output, input path is always enabled in this mode */
            default:
                DBG_PRINT_ERROR("Unkown Connect mode: %d", pImxConnectPin->ConnectMode);
                ntStatus = STATUS_INVALID_PARAMETER;
                break;
            }
            /* Set Pull resistors */
            switch (pImxConnectPin->PullConfiguration) {
            case GPIO_PIN_PULL_CONFIGURATION_DEFAULT: /* Do not change default pull value */
                PadValue |= *pPadDefaultValue & SC_PAD_PULL_MASK;
                break;
            case GPIO_PIN_PULL_CONFIGURATION_PULLUP:
                PadValue |= SC_PAD_28FDSOI_PS_PU << SC_PAD_PULL_SHIFT;
                break;
            case GPIO_PIN_PULL_CONFIGURATION_PULLDOWN:
                PadValue |= SC_PAD_28FDSOI_PS_PD << SC_PAD_PULL_SHIFT;
                break;
            case GPIO_PIN_PULL_CONFIGURATION_NONE:
                PadValue |= SC_PAD_28FDSOI_PS_NONE << SC_PAD_PULL_SHIFT;
                break;
            default:
                DBG_PRINT_ERROR("Unkown Pull mode: %d", pImxConnectPin->PullConfiguration);
                ntStatus = STATUS_INVALID_PARAMETER;
                break;
            }
            /* Set Drive strength */
            if (pImxConnectPin->DriveStrength == 0) {
                /* Do not change default value */
                PadValue |= *pPadDefaultValue & SC_PAD_DRIVE_STRENGTH_MASK;
            } else {
                DriveStrength_mA = (pImxConnectPin->DriveStrength / 100);
                switch (pAcpiPadInfo->Type) {
                case SC_PAD_TYPE_1V8_3V3:
                    if (DriveStrength_mA >= 1) {
                        DriveStrength_mA = 0;
                    } else {
                        DriveStrength_mA = 1;
                    }
                    break;
                case SC_PAD_TYPE_3V3:
                    DriveStrength_mA = ((DriveStrength_mA - 1) >> 2) & 0x03;
                    break;
                case SC_PAD_TYPE_1V8:
                    DriveStrength_mA = ((DriveStrength_mA +1) >> 1) & 0x07;
                    break;
                default:
                    DBG_PRINT_ERROR("Unkown PadType in ACPI pad info: %d , Supported values:  SC_PAD_TYPE_1V8_3V3(0x00), SC_PAD_TYPE_3V3(0x01), SC_PAD_TYPE_1V8(0x02)", pAcpiPadInfo->Type);
                    ntStatus = STATUS_INVALID_PARAMETER;
                }              
                PadValue |= DriveStrength_mA;
            }
        }
        if (!NT_SUCCESS(ntStatus)) {
            break;
        }
        if ((Operation == IMX_CONNECT_PIN_SET_VALUES) || (Operation == IMX_CONNECT_PIN_SET_DEFAULT)) {
            if ((scfwStatus = sc_pad_set(&pImxConnectPin->pDrvContext->SCFW_Handle, PadId, SC_PAD_SET_MUX_MASK | SC_PAD_SET_GP_MASK | PadValue)) != SC_ERR_NONE) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "sc_pad_get(Hnd: 0x%016p, Pad: %d, Value: 0x%08X) [Scfw error code: %d]", &pImxConnectPin->pDrvContext->SCFW_Handle, PadId, PadValue, scfwStatus);
                break;
            }
            if ((scfwStatus = sc_pad_get(&pImxConnectPin->pDrvContext->SCFW_Handle, PadId, &OrgPadValue)) != SC_ERR_NONE) {
                ntStatus = STATUS_INVALID_PARAMETER;
                DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "sc_pad_get(Hnd: 0x%016p, Pad: %d, Value: 0x%08X) [Scfw error code: %d]", &pImxConnectPin->pDrvContext->SCFW_Handle, PadId, OrgPadValue, scfwStatus);
                break;
            } else {
                DBG_DEV_PRINT_INFO("sc_pad_get(Hnd: 0x%016p, Pad: %d, Value: 0x%08X)", &pImxConnectPin->pDrvContext->SCFW_Handle, PadId, OrgPadValue);
            }
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
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, ReadOutPin %s", ReadParameters->BankId, (ReadParameters->Flags.WriteConfiguredPins)? "yes":"no");
    do {
        CheckBankIdGetRegsOrBreakIfError(ReadParameters->BankId);
        *ReadParameters->PinValues = pDevRegs->PSR.R;            /* Get value of all pins */
        if (ReadParameters->Flags.WriteConfiguredPins == 0) {    /* Request to update also write-configured pins? */
            *ReadParameters->PinValues &= ~pDevRegs->GDIR.R;     /* No, mask (set to zero) write-configured pins */
        }
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d, PinValues 0x%08X", ReadParameters->BankId, (UINT32)*ReadParameters->PinValues, ntStatus);
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
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, SetMask 0x%08X, ClrMask 0x%08X", WriteParameters->BankId, (UINT32)WriteParameters->SetMask, (UINT32)WriteParameters->ClearMask);
    do {
        CheckBankIdGetRegsOrBreakIfError(WriteParameters->BankId);
        DBG_DUMP_DATA_REGS(pDevRegs, "Before:");
        pDevRegs->DR.R = (UINT32)(((ULONG64)pDevRegs->DR.R & ~(WriteParameters->ClearMask)) | WriteParameters->SetMask);
        DBG_DUMP_DATA_REGS(pDevRegs, "After:");
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d", WriteParameters->BankId, ntStatus);
    return ntStatus;
}

/**
* Opens a logical connection to a set of general-purpose I/O (GPIO) pins and configures these pins for data read or write operations.
*
* Params:
*   Context:  A pointer to the GPIO controller driver's device context.
*   ConnectParameters -> BankId            The identifier for the bank of GPIO pins.
*                     -> PinNumberTable    Pin table address
*                     -> PinCount          Numbner of pins in PinNumberTable 
*                     -> ConnectMode       ConnectModeInvalid
*                                          ConnectModeInput
*                                          ConnectModeOutput
*                     -> PullConfiguration GPIO_PIN_PULL_CONFIGURATION_PULLDEFAULT
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLUP
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLDOWN
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLNONE
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
    NTSTATUS            ntStatus      = STATUS_SUCCESS;
    GPIO_DrvContext_t*  pDrvContext   = (GPIO_DrvContext_t*)Context;
    UINT32              uPinsBitMask  = 0;
    UINT32              uPinsBitValue = 0;
    GPIO_t*             pDevRegs;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, PinCnt %d, Mode %s, PullCfg %s, DebTimeout %d, DrvStrength %d [mA] ,VendorData 0x%016p", ConnectParameters->BankId, ConnectParameters->PinCount, ToString_ConnectioMode(ConnectParameters->ConnectMode), ToString_AcpiPullCfg(ConnectParameters->PullConfiguration), ConnectParameters->DebounceTimeout, ConnectParameters->DriveStrength/100, ConnectParameters->VendorData);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    do {
        CheckBankIdAndPinCountAndBreakIfError(ConnectParameters->BankId, ConnectParameters->PinCount);
        ImxPinConnectParams.pDrvContext       = pDrvContext;
        ImxPinConnectParams.BankId            = ConnectParameters->BankId;
        ImxPinConnectParams.ConnectMode       = ConnectParameters->ConnectMode;
        ImxPinConnectParams.PullConfiguration = ConnectParameters->PullConfiguration;
        ImxPinConnectParams.DriveStrength     = ConnectParameters->DriveStrength;
        ImxPinConnectParams.VendorData        = ConnectParameters->VendorData;
        ImxPinConnectParams.VendorDataLength  = ConnectParameters->VendorDataLength;
        for (unsigned int i = 0; i < ConnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber     = ConnectParameters->PinNumberTable[i];
            ImxPinConnectParams.Mux           = pDrvContext->PAD_AcpiPadInfoArray[(ConnectParameters->BankId * 32) + ImxPinConnectParams.PinNumber].Mux;
                if (!NT_SUCCESS(ntStatus = ImxConnectIoPin(&ImxPinConnectParams, IMX_CONNECT_PIN_CHECK_OLNY))) {
                DBG_PRINT_ERROR("ImxConnectIoPin() - unsupported pin settings");
                break;
            }
        }
        pDevRegs = pDrvContext->GPIO_BankContextArray[ConnectParameters->BankId].pRegs;
        DBG_DUMP_DATA_REGS(pDevRegs, "Before:");
        for (unsigned int i = 0; i < ConnectParameters->PinCount; i++) {
            uPinsBitMask |= (UINT32)1 << ConnectParameters->PinNumberTable[i];
            ImxPinConnectParams.PinNumber = ConnectParameters->PinNumberTable[i];
            ImxPinConnectParams.Mux       = pDrvContext->PAD_AcpiPadInfoArray[(ConnectParameters->BankId * 32) + ImxPinConnectParams.PinNumber].Mux;
            (void)ImxConnectIoPin(&ImxPinConnectParams, IMX_CONNECT_PIN_SET_VALUES);
            uPinsBitValue |= ImxPinConnectParams.GpioPinDir;
        }   
        /* Update GPIO pins direction */
        GPIO_CLX_AcquireInterruptLock(Context, ConnectParameters->BankId);
        pDevRegs->GDIR.R = (pDevRegs->GDIR.R & ~uPinsBitMask) | uPinsBitValue;
        GPIO_CLX_ReleaseInterruptLock(Context, ConnectParameters->BankId);
        DBG_DUMP_DATA_REGS(pDevRegs, "After:v");
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
*                     -> PinCount                 Numbner of pins in PinNumberTable
*                     -> DisConnectMode           ConnectModeInvalid
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
    GPIO_DrvContext_t*  pDrvContext = (GPIO_DrvContext_t*)Context;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, PinCnt %d, Mode %s, PullCfg %s, PreserveCfg %d", DisconnectParameters->BankId, DisconnectParameters->PinCount, ToString_ConnectioMode(DisconnectParameters->DisconnectMode), DisconnectParameters->DisconnectFlags.PreserveConfiguration);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    do {
        if (DisconnectParameters->DisconnectFlags.PreserveConfiguration == 1) {
            break;
        }
        CheckBankIdAndPinCountAndBreakIfError(DisconnectParameters->BankId, DisconnectParameters->PinCount);
        ImxPinConnectParams.pDrvContext = pDrvContext;
        ImxPinConnectParams.BankId = DisconnectParameters->BankId;
        for (unsigned int i = 0; i < DisconnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber = DisconnectParameters->PinNumberTable[i];
            (void)ImxConnectIoPin(&ImxPinConnectParams, IMX_CONNECT_PIN_SET_DEFAULT);
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
*                     -> PinNumberTable    Pin table address
*                     -> PinCount          Numbner of pins in PinNumberTable
*                     -> FunctionNumber    ALT number
*                     -> PullConfiguration GPIO_PIN_PULL_CONFIGURATION_PULLDEFAULT
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLUP
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLDOWN
*                                          GPIO_PIN_PULL_CONFIGURATION_PULLNONE
*                     -> VendorData        A pointer to a caller-allocated buffer that contains vendor-specific data obtained from the ACPI firmware for the hardware platform.
*                     -> VendorDataLength  The size, in bytes, of the data buffer that is pointed to by the VendorData member.
*                     -> ConnectFlags      No flags are currently defined for this member.
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS ConnectFunctionConfigPins(PVOID Context, PGPIO_CONNECT_FUNCTION_CONFIG_PINS_PARAMETERS ConnectParameters) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t*  pDrvContext = (GPIO_DrvContext_t*)Context;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, PinCnt %d, ALT %d, PullCfg %s ,VendorData 0x%016p", ConnectParameters->BankId, ConnectParameters->PinCount, ConnectParameters->FunctionNumber, ToString_AcpiPullCfg(ConnectParameters->PullConfiguration), ConnectParameters->VendorData);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    do {
        CheckBankIdAndPinCountAndBreakIfError(ConnectParameters->BankId, ConnectParameters->PinCount);
        ImxPinConnectParams.pDrvContext       = pDrvContext;
        ImxPinConnectParams.BankId            = ConnectParameters->BankId;
        ImxPinConnectParams.ConnectMode       = ConnectModeInput;  /* Force Pad.Config to "Normal". GPIO.Direction bit is not be changed in this method. */
        ImxPinConnectParams.PullConfiguration = ConnectParameters->PullConfiguration;
        ImxPinConnectParams.DriveStrength     = 0;
        ImxPinConnectParams.VendorData        = ConnectParameters->VendorData;
        ImxPinConnectParams.VendorDataLength  = ConnectParameters->VendorDataLength;
        ImxPinConnectParams.Mux               = (UINT8)ConnectParameters->FunctionNumber;
        for (unsigned int i = 0; i < ConnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber     = ConnectParameters->PinNumberTable[i];
            ImxPinConnectParams.Mux           = pDrvContext->PAD_AcpiPadInfoArray[(ConnectParameters->BankId * 32) + ImxPinConnectParams.PinNumber].Mux;
                if (!NT_SUCCESS(ntStatus = ImxConnectIoPin(&ImxPinConnectParams, IMX_CONNECT_PIN_CHECK_OLNY))) {
                DBG_PRINT_ERROR("ImxConnectIoPin() - unsupported pin settings");
                break;
            }
        }
        for (unsigned int i = 0; i < ConnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber = ConnectParameters->PinNumberTable[i];
            ImxPinConnectParams.Mux       = pDrvContext->PAD_AcpiPadInfoArray[(ConnectParameters->BankId * 32) + ImxPinConnectParams.PinNumber].Mux;
            (void)ImxConnectIoPin(&ImxPinConnectParams, IMX_CONNECT_PIN_SET_VALUES);
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
*                        -> PinCount          Numbner of pins in PinNumberTable
*                        -> ConnectFlags      No flags are currently defined for this member.
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS DisconnectFunctionConfigPins(PVOID Context, PGPIO_DISCONNECT_FUNCTION_CONFIG_PINS_PARAMETERS DisconnectParameters) {
    NTSTATUS            ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t*  pDrvContext = (GPIO_DrvContext_t*)Context;
    IMX_CONNECT_PIN_t   ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, PinCnt %d", DisconnectParameters->BankId, DisconnectParameters->PinCount);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    do {
        CheckBankIdAndPinCountAndBreakIfError(DisconnectParameters->BankId, DisconnectParameters->PinCount);
        ImxPinConnectParams.pDrvContext   = pDrvContext;
        ImxPinConnectParams.BankId        = DisconnectParameters->BankId;
        for (unsigned int i = 0; i < DisconnectParameters->PinCount; i++) {
            ImxPinConnectParams.PinNumber = DisconnectParameters->PinNumberTable[i];
            (void)ImxConnectIoPin(&ImxPinConnectParams, IMX_CONNECT_PIN_SET_DEFAULT);
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
*                       -> PullConfiguration   GPIO_PIN_PULL_CONFIGURATION_PULLDEFAULT     0
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLUP          1
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLDOWN        2
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLNONE        3
*                                              or vendor-defined constant in the range 128-255
*                       -> DebounceTimeout     The debounce time in units of 10 microseconds.
*                       -> VendorData
*                       -> VendorDataLength
*
* Called at PASSIVE_LEVEL.
*/
_Use_decl_annotations_
NTSTATUS EnableInterrupt(PVOID Context, PGPIO_ENABLE_INTERRUPT_PARAMETERS EnableParametersPtr) {
    NTSTATUS                       ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t*             pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*                        pDevRegs;
    UINT32                         uPinMask, uPinShift;
    IMX_GPIO_PIN_INTERRUPT_CONFIG  InterruptConfig;
    IMX_CONNECT_PIN_t              ImxPinConnectParams;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, Pin %d", EnableParametersPtr->BankId, EnableParametersPtr->PinNumber);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    DBG_DUMP_ENABLE_INTERRUPT_PARAMETERS(EnableParametersPtr);
    do {
        CheckBankIdGetRegsOrBreakIfError(EnableParametersPtr->BankId);
        if ((InterruptConfig = ImxGetGpioPinInterruptCfg(EnableParametersPtr->InterruptMode, EnableParametersPtr->Polarity)) == IMX_GPIO_PIN_INTERRUPT_CONFIG_ERROR) {
            DBG_PRINT_ERROR("Unsupported interrupt configuration");
            ntStatus = STATUS_INVALID_PARAMETER;
            break;
        }
        ImxPinConnectParams.pDrvContext       = pDrvContext;
        ImxPinConnectParams.BankId            = EnableParametersPtr->BankId;
        ImxPinConnectParams.PinNumber         = EnableParametersPtr->PinNumber;
        ImxPinConnectParams.ConnectMode      = ConnectModeInput;  /* Set Pad.Cfg = Normal */
        ImxPinConnectParams.PullConfiguration = EnableParametersPtr->PullConfiguration;
        ImxPinConnectParams.VendorData        = EnableParametersPtr->VendorData;
        ImxPinConnectParams.VendorDataLength  = EnableParametersPtr->VendorDataLength;
        ImxPinConnectParams.Mux               = pDrvContext->PAD_AcpiPadInfoArray[(EnableParametersPtr->BankId * 32) + EnableParametersPtr->PinNumber].Mux;
        ImxPinConnectParams.DriveStrength     = 0;
        if (!NT_SUCCESS(ntStatus = ImxConnectIoPin(&ImxPinConnectParams, FALSE))) {
            DBG_PRINT_ERROR("Unsupported interrupt configuration");
            break;
        }
        uPinShift = (EnableParametersPtr->PinNumber % 16) << 1;
        uPinMask  = ~(GPIO_ICR1_ICR0_MASK << uPinShift);
        InterruptConfig = InterruptConfig << uPinShift;
        GPIO_CLX_AcquireInterruptLock(Context, EnableParametersPtr->BankId);
        DBG_DUMP_ALL_REGS(pDevRegs, "Before:");
        if (EnableParametersPtr->PinNumber < 16) {
            pDevRegs->ICR1.R = (pDevRegs->ICR1.R & uPinMask) | InterruptConfig;
        } else {
            pDevRegs->ICR2.R = (pDevRegs->ICR2.R & uPinMask) | InterruptConfig;
        }
        uPinMask = (UINT32)1 << EnableParametersPtr->PinNumber;
        if (ImxPinConnectParams.GpioPinDir) {
            pDevRegs->GDIR.R |= uPinMask;    /* Set Output direction */
        } else {
            pDevRegs->GDIR.R &= ~uPinMask;   /* Set Input direction */
        }
        pDevRegs->ISR.R   =  uPinMask;       /* Clear interrupt flag */
        pDevRegs->IMR.R  |=  uPinMask;       /* Unmask interrupt */
        DBG_DUMP_ALL_REGS(pDevRegs, "After:");
        GPIO_CLX_ReleaseInterruptLock(Context, EnableParametersPtr->BankId);
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d, Pin %d", EnableParametersPtr->BankId, EnableParametersPtr->PinNumber, ntStatus);
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
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;
    UINT32             uPinMask;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, Pin %d", DisableParametersPtr->BankId, DisableParametersPtr->PinNumber);
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    DBG_DUMP_DISABLE_INTERRUPT_PARAMETERS(DisableParametersPtr);
    do {
        CheckBankIdGetRegsOrBreakIfError(DisableParametersPtr->BankId);
        uPinMask = (UINT32)1 << DisableParametersPtr->PinNumber;
        GPIO_CLX_AcquireInterruptLock(Context, DisableParametersPtr->BankId);
        DBG_DUMP_INT_REGS(pDevRegs, "Before:");
        pDevRegs->IMR.R  &= ~uPinMask;   /* Mask interrupt */
        DBG_DUMP_INT_REGS(pDevRegs, "After:");
        GPIO_CLX_ReleaseInterruptLock(Context, DisableParametersPtr->BankId);
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d, Pin %d", DisableParametersPtr->BankId, DisableParametersPtr->PinNumber, ntStatus);
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
*                       -> PullConfiguration   GPIO_PIN_PULL_CONFIGURATION_PULLDEFAULT     0
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLUP          1
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLDOWN        2
*                                              GPIO_PIN_PULL_CONFIGURATION_PULLNONE        3
*                                              or vendor-defined constant in the range 128-255
*                       -> DebounceTimeout     The debounce time in units of 10 microseconds.
*                       -> VendorData
*                       -> VendorDataLength
*
* Called either at PASSIVE_LEVEL or DIRQL.
*/
_Use_decl_annotations_
NTSTATUS UnmaskInterrupt(PVOID Context, PGPIO_ENABLE_INTERRUPT_PARAMETERS UnmaskParametersPtr) {
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, Pin %d", UnmaskParametersPtr->BankId, UnmaskParametersPtr->PinNumber);
    DBG_DUMP_ENABLE_INTERRUPT_PARAMETERS(UnmaskParametersPtr);
    do {
        CheckBankIdGetRegsOrBreakIfError(UnmaskParametersPtr->BankId);
        DBG_DUMP_INT_REGS(pDevRegs, "Before:");
        pDevRegs->IMR.R |= (UINT32)1 << UnmaskParametersPtr->PinNumber;   /* Unmask interrupt  */
        DBG_DUMP_INT_REGS(pDevRegs, "After:");
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d, Pin %d", UnmaskParametersPtr->BankId, UnmaskParametersPtr->PinNumber, ntStatus);
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
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, PinMask: 0x%08X", MaskParametersPtr->BankId, (UINT32)MaskParametersPtr->PinMask);
    DBG_DUMP_MASK_INTERRUPT_PARAMETERS(MaskParametersPtr);
    do {
        CheckBankIdGetRegsOrBreakIfError(MaskParametersPtr->BankId);
        DBG_DUMP_INT_REGS(pDevRegs, "Before:");
        pDevRegs->IMR.R &= ~(UINT32)(MaskParametersPtr->PinMask);   /* Mask interrupt */
        DBG_DUMP_INT_REGS(pDevRegs, "After:");
        MaskParametersPtr->FailedMask = 0;                          /* No error */
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d, PinMask: 0x%08X, FailedMask: 0x%08X", MaskParametersPtr->BankId, (UINT32)MaskParametersPtr->PinMask, (UINT32)MaskParametersPtr->FailedMask, ntStatus);
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
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, EnabledMask: 0x%08X", QueryActiveParametersPtr->BankId, (UINT32)QueryActiveParametersPtr->EnabledMask);
    DBG_DUMP_QUERY_ACTIVE_INTERRUPTS_PARAMETERS(QueryActiveParametersPtr);
    do {
        CheckBankIdGetRegsOrBreakIfError(QueryActiveParametersPtr->BankId);
        QueryActiveParametersPtr->ActiveMask = QueryActiveParametersPtr->EnabledMask & pDevRegs->IMR.R;
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d, ActiveMask:  0x%08X", QueryActiveParametersPtr->BankId, (UINT32)QueryActiveParametersPtr->ActiveMask, ntStatus);
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
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, ClearActiveMask: 0x%08X", ClearParametersPtr->BankId, (UINT32)ClearParametersPtr->ClearActiveMask);
    DBG_DUMP_CLEAR_ACTIVE_INTERRUPTS_PARAMETERS(ClearParametersPtr);
    do {
        CheckBankIdGetRegsOrBreakIfError(ClearParametersPtr->BankId);
        pDevRegs = pDrvContext->GPIO_BankContextArray[ClearParametersPtr->BankId].pRegs;
        DBG_DUMP_INT_REGS(pDevRegs, "Before:");
        pDevRegs->ISR.R = (UINT32)(ClearParametersPtr->ClearActiveMask);
        DBG_DUMP_INT_REGS(pDevRegs, "After:");
        ClearParametersPtr->FailedClearMask = 0; /* No error */
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d, FailedClearMask: 0x%08X", ClearParametersPtr->BankId, (UINT32)ClearParametersPtr->FailedClearMask, ntStatus);
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
    NTSTATUS                  ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t*        pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*                   pDevRegs;
    UINT32                    uPinMask, uPinShift;
    IMX_GPIO_PIN_INTERRUPT_CONFIG InterruptConfig;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d, Pin %d", ReconfigureParametersPtr->BankId, ReconfigureParametersPtr->PinNumber);
    DBG_DUMP_RECONFIGURE_INTERRUPTS_PARAMETERS(ReconfigureParametersPtr);
    do {
        CheckBankIdGetRegsOrBreakIfError(ReconfigureParametersPtr->BankId);
        if ((InterruptConfig = ImxGetGpioPinInterruptCfg(ReconfigureParametersPtr->InterruptMode, ReconfigureParametersPtr->Polarity)) == IMX_GPIO_PIN_INTERRUPT_CONFIG_ERROR) {
            DBG_PRINT_ERROR("Unsupported interrupt configuration");
            ntStatus = STATUS_INVALID_PARAMETER;
            break;
        }
        uPinShift = (ReconfigureParametersPtr->PinNumber % 16) << 1;
        uPinMask = ~(GPIO_ICR1_ICR0_MASK << uPinShift);
        InterruptConfig = InterruptConfig << uPinShift;
        DBG_DUMP_INT_REGS(pDevRegs, "Before:");
        if (ReconfigureParametersPtr->PinNumber < 16) {
            pDevRegs->ICR1.R = (pDevRegs->ICR1.R & uPinMask) | InterruptConfig;
        } else {
            pDevRegs->ICR2.R = (pDevRegs->ICR2.R & uPinMask) | InterruptConfig;
        }
        DBG_DUMP_INT_REGS(pDevRegs, "After:");
    }  while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO %d, Pin %d", ReconfigureParametersPtr->BankId, ReconfigureParametersPtr->PinNumber, ntStatus);
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
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("GPIO %d", QueryEnabledParametersPtr->BankId);
    DBG_DUMP_QUERY_ENABLED_INTERRUPTS_PARAMETERS(QueryEnabledParametersPtr);
    do {
        CheckBankIdGetRegsOrBreakIfError(QueryEnabledParametersPtr->BankId);
        QueryEnabledParametersPtr->EnabledMask = pDevRegs->IMR.R;
    } while (0);
    DBG_DEV_METHOD_END_WITH_PARAMS_AND_STATUS("GPIO % d, FailedClearMask: 0x % 08X", QueryEnabledParametersPtr->BankId, (UINT32)QueryEnabledParametersPtr->EnabledMask, ntStatus);
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
    GPIO_DrvContext_t*  pDrvContext = (GPIO_DrvContext_t*)Context;

    DBG_DEV_METHOD_BEG();
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    PAGED_CODE();
    ControllerInformationPtr->Version                               = GPIO_CONTROLLER_BASIC_INFORMATION_VERSION;
    ControllerInformationPtr->Size                                  = sizeof(*ControllerInformationPtr);
    ControllerInformationPtr->TotalPins                             = (USHORT)(pDrvContext->GPIO_BankCount * GPIO_PIN_PER_BANK);
    ControllerInformationPtr->NumberOfPinsPerBank                   = GPIO_PIN_PER_BANK;
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
    NTSTATUS           ntStatus    = STATUS_SUCCESS;
    GPIO_DrvContext_t* pDrvContext = (GPIO_DrvContext_t*)Context;
    GPIO_t*            pDevRegs;

    DBG_DEV_METHOD_BEG_WITH_PARAMS("RestoreContext: %d, PreviousPowerState: %s", RestoreContext, ToString_WdfPowerDeviceState(PreviousPowerState));
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    UNREFERENCED_PARAMETER(RestoreContext);
    UNREFERENCED_PARAMETER(PreviousPowerState);
    for (ULONG bankId = 0; bankId < pDrvContext->GPIO_BankCount; ++bankId) {
        pDevRegs = pDrvContext->GPIO_BankContextArray[bankId].pRegs;
        /* Reset bank interrupts configurations */
        pDevRegs->IMR.R = 0;
        pDevRegs->ICR1.R = 0;
        pDevRegs->ICR2.R = 0;
        pDevRegs->ISR.R = 0xFFFFFFFF;
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
    NTSTATUS                        ntStatus               = STATUS_SUCCESS;
    GPIO_DrvContext_t*              pDrvContext            = (GPIO_DrvContext_t*)Context;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR pResDscr               = NULL;
    ULONG                           uAcpiPinConfigDataSize = 0;
    ULONG                           uInterruptCount        = 0;
    ULONG                           uScfwDevice            = 0;
    ULONG                           uResourceCount;

    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    UNREFERENCED_PARAMETER(ResourcesRaw);
    pDrvContext->ACPI_UtilsContext.Pdo = WdfDeviceWdmGetPhysicalDevice(Device);     
    pDrvContext->ACPI_UtilsContext.MemPoolTag = GPIO_MEM_TAG_ACPI;
    if (!NT_SUCCESS(ntStatus = Acpi_Init(&pDrvContext->ACPI_UtilsContext))) {
        ntStatus = STATUS_SUCCESS;
        DBG_DEV_PRINT_INFO("Acpi_Init() failed");
    }
    uResourceCount = WdfCmResourceListGetCount(ResourcesTranslated);
    for (unsigned int i = 0; i < uResourceCount; i++) {
        pResDscr = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        switch (pResDscr->Type) {
           case CmResourceTypeInterrupt:
               uInterruptCount++;
               DBG_DEV_PRINT_INFO("Interrupt resource found at index: %lu", uInterruptCount);
               break;
           case CmResourceTypeMemory:
               DBG_DEV_PRINT_INFO("GPIODev_RegPhyAddr: 0x%016llX,  GPIODev_RegSize: 0x%08X", pResDscr->u.Memory.Start.QuadPart, pResDscr->u.Memory.Length);
               if ((pDrvContext->GPIO_BankContextArray[pDrvContext->GPIO_BankCount].pRegs = (GPIO_t*)MmMapIoSpaceEx(pResDscr->u.Memory.Start, pResDscr->u.Memory.Length, PAGE_READWRITE | PAGE_NOCACHE)) == NULL) {
                   ntStatus = STATUS_INSUFFICIENT_RESOURCES;
                   DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "MmMapIoSpaceEx() failed.");
                   break;
               }
               pDrvContext->GPIO_BankContextArray[pDrvContext->GPIO_BankCount++].RegSize = pResDscr->u.Memory.Length; /* Remember address and increment bank counter */
               break;
           case CmResourceTypeConnection:
               if ((pResDscr->u.Connection.Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)) {
                   uScfwDevice++;
                   pDrvContext->SCFW_Config.ipc_id.HighPart = pResDscr->u.Connection.IdHighPart;
                   pDrvContext->SCFW_Config.ipc_id.LowPart  = pResDscr->u.Connection.IdLowPart;
                   DBG_DEV_PRINT_INFO("SCFW connection found, IdHighPart: 0x%08X, IdLowPart: 0x%08X", pResDscr->u.Connection.IdHighPart, pResDscr->u.Connection.IdLowPart);
               }
           default:
               break;
        }
        if (!NT_SUCCESS(ntStatus)) {
            break;
        }
    }
    do {
        if (pDrvContext->GPIO_BankCount != uInterruptCount) {
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "Interrupt resource count != Memory resource count (%d != %d). Check ACPI table.", uInterruptCount, pDrvContext->GPIO_BankCount);
            break;
        }
        Acpi_GetBufferPropertyAddress(&pDrvContext->ACPI_UtilsContext, "Pin_Config", (UINT8**)&pDrvContext->PAD_AcpiPadInfoArray, &uAcpiPinConfigDataSize);
        if (uAcpiPinConfigDataSize != (pDrvContext->GPIO_BankCount * GPIO_PIN_PER_BANK * (UINT32)sizeof(GPIO_AcpiPadInfo_t))) {
            ntStatus = STATUS_ACPI_INVALID_DATA;
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "SizeOf(Pin_Config) property in ACPI must be %d = BankCount(%d) * %d sizeof(GPIO_AcpiPadInfo_t(%d)). Rerutn size: %d.",
                pDrvContext->GPIO_BankCount * GPIO_PIN_PER_BANK * (UINT32)sizeof(GPIO_AcpiPadInfo_t),
                pDrvContext->GPIO_BankCount,  GPIO_PIN_PER_BANK,  (UINT32)sizeof(GPIO_AcpiPadInfo_t), uAcpiPinConfigDataSize
            );
            break;
        }
        if (uScfwDevice != 1) {
            ntStatus = STATUS_INSUFFICIENT_RESOURCES;
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "No or more then one (%d) Scfw resorce found in ACPI()", uScfwDevice);
            break;
        }
        if (!NT_SUCCESS(ntStatus = sc_ipc_open(&pDrvContext->SCFW_Handle, &pDrvContext->SCFW_Config))) {
            DBG_PRINT_ERROR_WITH_STATUS(ntStatus, "ERROR sc_ipc_open()");
            break;
        }
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
    GPIO_DrvContext_t*  pDrvContext; 

    DBG_DEV_METHOD_BEG();
    PAGED_CODE();
    IMX_ASSERT_MAX_IRQL(PASSIVE_LEVEL);
    UNREFERENCED_PARAMETER(Device);
    pDrvContext = (GPIO_DrvContext_t*)Context;
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
    NTSTATUS              ntStatus;
    WDF_OBJECT_ATTRIBUTES wdfDeviceAttributes;
    WDFDEVICE             wdfDevice;

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
    NTSTATUS              ntStatus = STATUS_SUCCESS;
    WDF_OBJECT_ATTRIBUTES wdfObjectAttributes;
    WDF_DRIVER_CONFIG     wdfDriverConfig;
    WDFDRIVER             wdfDriver = NULL;

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
            sizeof(GPIO_DrvContext_t),
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