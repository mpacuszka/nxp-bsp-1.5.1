/*
* This file is derived from Microsoft example available on https://github.com/microsoft/Windows-driver-samples/blob/master/usb/UcmTcpciCxClientSample
* Original file licensing:
* Copyright (c) 2015 Microsoft
* SPDX-License-Identifier: MS-PL
* NXP modifications are licensed under the same license
* Copyright 2020 NXP
*
*/
#include "UcmTcpciConstatnts.h"

#define USBC_CAP_ID_SUPPORTED_OP_MODES                1 /* Supported Operating Modes */
#define USBC_CAP_ID_SUPPORTED_TYPEC_SRC_CAP           2 /* Supported Type-C Sourcing Capabilities */
#define USBC_CAP_ID_IS_AA_CAPABLE                     3 /* Is Audio Accessory Capable */
#define USBC_CAP_ID_IS_PD_SUPPORTED                   4 /* Is PD supported */
#define USBC_CAP_ID_SUPPORTED_PD_ROLES                5 /* Supported PD Power Roles  */
#define USBC_CAP_ID_PD_SOURCE_PDOS                    6 /* PD Source PDOs */
#define USBC_CAP_ID_PD_SINK_PDOS                      7 /* PD Sink PDOs */
#define USBC_CAP_ID_SUPPORTED_ALT_MODES               8 /* Supported Alternate Modes */

/* 1. Supported Operating Modes */
#define USBC_CAP_OP_MODE_UNDEF           0
#define USBC_CAP_OP_MODE_DFP_ONLY        1
#define USBC_CAP_OP_MODE_UFP_ONLY        2
#define USBC_CAP_OP_MODE_DRP             4

/* 2. Supported Type-C Sourcing Capabilities */
#define USBC_CAP_TYPEC_SRC_UNDEF         0
#define USBC_CAP_TYPEC_SRC_DEFAULT_USB   1
#define USBC_CAP_TYPEC_SRC_1500_MA       2  /* Type C 1.5 A */
#define USBC_CAP_TYPEC_SRC_3000_MA       2  /* Type C 3.0 A */

/* 3. Is Audio Accessory Capable */
#define USBC_CAP_AA_NOT_SUPPORTED        0
#define USBC_CAP_AA_SUPPORTED            1
#define USBC_CAP_AA_DEFAULT              2  /* Depends on what the client driver reports */

/* 4. Is PD supported */
#define USBC_CAP_PD_NOT_SUPPORTED        0
#define USBC_CAP_PD_SUPPORTED            1
#define USBC_CAP_PD_DEFAULT              2  /* Depends on what the client driver reports */

/* 5. Supported Power Roles */
#define USBC_CAP_PD_ROLE_UNDEF           0
#define USBC_CAP_PD_ROLE_SINK            1
#define USBC_CAP_PD_ROLE_SRC             2

/* USB_PD_R2_0 V1.3
Sink Fixed Supply Power Data Object
*/
#define PD_SNK_FX_DRP_MASK             0x20000000
#define PD_SNK_FX_DRP_SHIFT            29
#define PD_SNK_FX_HC_MASK              0x10000000
#define PD_SNK_FX_HC_SHIFT             28
#define PD_SNK_FX_UC_PWR_MASK          0x08000000
#define PD_SNK_FX_UC_PWR_SHIFT         27
#define PD_SNK_FX_USB_CC_MASK          0x04000000
#define PD_SNK_FX_USB_CC_SHIFT         26
#define PD_SNK_FX_DRD_MASK             0x02000000
#define PD_SNK_FX_DRD_SHIFT            25
#define PD_SNK_FX_U_MASK               0x000FFC00
#define PD_SNK_FX_U_SHIFT              10
#define PD_SNK_FX_I_MASK               0x000003FF
#define PD_SNK_FX_I_SHIFT              0

#define PD_SNK_FX(DRP, HC, UC_PWR, USB_CC, DRD, U_mv, I_ma) (((DRP        << PD_SNK_FX_DRP_SHIFT)    & PD_SNK_FX_DRP_MASK)    | \
                                                             ((HC         << PD_SNK_FX_HC_SHIFT)     & PD_SNK_FX_HC_MASK)     | \
                                                             ((UC_PWR     << PD_SNK_FX_UC_PWR_SHIFT) & PD_SNK_FX_UC_PWR_MASK) | \
                                                             ((USB_CC     << PD_SNK_FX_USB_CC_SHIFT) & PD_SNK_FX_USB_CC_MASK) | \
                                                             ((DRD        << PD_SNK_FX_DRD_SHIFT)    & PD_SNK_FX_DRD_MASK)    | \
                                                             (((U_mv/50)  << PD_SNK_FX_U_SHIFT)      & PD_SNK_FX_U_MASK)      | \
                                                             (((I_ma/10)  << PD_SNK_FX_I_SHIFT)      & PD_SNK_FX_I_MASK))

/* USB_PD_R2_0 V1.3
6.4.1.2.3 Source Fixed Supply Power Data Object
*/

#define PD_SRC_FX_DRP_MASK            0x20000000
#define PD_SRC_FX_DRP_SHIFT           29
#define PD_SRC_FX_USB_SUSP_MASK       0x10000000
#define PD_SRC_FX_USB_SUSP_SHIFT      28
#define PD_SRC_FX_UC_PWR_MASK         0x08000000
#define PD_SRC_FX_UC_PWR_SHIFT        27
#define PD_SRC_FX_USB_CC_MASK         0x04000000
#define PD_SRC_FX_USB_CC_SHIFT        26
#define PD_SRC_FX_DRD_MASK            0x02000000
#define PD_SRC_FX_DRD_SHIFT           25
#define PD_SRC_FX_UEM_MASK            0x01000000
#define PD_SRC_FX_UEM_SHIFT           24
#define PD_SRC_FX_PK_MASK             0x01300000
#define PD_SRC_FX_PK_SHIFT            20
#define PD_SRC_FX_U_MASK              0x000FFC00
#define PD_SRC_FX_U_SHIFT             10
#define PD_SRC_FX_I_MASK              0x000003FF
#define PD_SRC_FX_I_SHIFT             0

#define PD_SRC_FX(DRP, USB_SUSP, UC_PWR, USB_CC, DRD, UEM, PK, U_mv, I_ma) (((DRP        << PD_SRC_FX_DRP_SHIFT)      & PD_SRC_FX_DRP_MASK)      | \
                                                                            ((USB_SUSP   << PD_SRC_FX_USB_SUSP_SHIFT) & PD_SRC_FX_USB_SUSP_MASK) | \
                                                                            ((UC_PWR     << PD_SRC_FX_UC_PWR_SHIFT)   & PD_SRC_FX_UC_PWR_MASK)   | \
                                                                            ((USB_CC     << PD_SRC_FX_USB_CC_SHIFT)   & PD_SRC_FX_USB_CC_MASK)   | \
                                                                            ((DRD        << PD_SRC_FX_DRD_SHIFT)      & PD_SRC_FX_DRD_MASK)      | \
                                                                            ((UEM        << PD_SRC_FX_UEM_SHIFT)      & PD_SRC_FX_UEM_MASK)      | \
                                                                            ((PK         << PD_SRC_FX_PK_SHIFT)       & PD_SRC_FX_PK_MASK)       | \
                                                                            (((U_mv/50)  << PD_SRC_FX_U_SHIFT)        & PD_SRC_FX_U_MASK)        | \
                                                                            (((I_ma/10)  << PD_SRC_FX_I_SHIFT)        & PD_SRC_FX_I_MASK))



// UCM-TCPCI device. Can be named anything.
Device(USBC)
{
    // This device needs to be enumerated by ACPI, so it needs a HWID.
    // Your INF should match on it.
    Name(_HID, "USBC0001")
    Method(_CRS, 0x0, NotSerialized)
    {
        Name (RBUF, ResourceTemplate ()
        {
            //
            // Sample I2C and GPIO resources.
            // platform's underlying controllers and connections.
            // \_SB.I2C and \_SB.GPIO are paths to predefined I2C and GPIO controller instances.
            I2CSerialBus(0x50, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C1")
            // Interrupt pin: Pad: SPI2_SCK
            GpioInt(Level, ActiveLow, Exclusive, PullNone, 0, "\\_SB.GPIO",) { IMX_PIN_SPI2_SCK }
            // CBTU02043 switch SEL: Pad: ENET0_REFCLK_125M_25M
            GpioIO(Exclusive, PullNone, 0, 1, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PIN_ENET0_REFCLK_125M_25M }
        })
        Return(RBUF)
    }
    Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () { // CBTU02043 Differential Channel Crossbar Switch settings
            Package (2) {"SwitchPin_Index", 0},
            Package (2) {"SwitchPin_Polarity", 1},
        }
    })

    // Inside the scope of the UCM-TCPCI device, you need to define one "connector" device.
    // It can be named anything.
    Device(CON0)
    {
        // This device is not meant to be enumerated by ACPI, hence you should not assign a
        // HWID to it. Instead, use _ADR to assign address 0 to it.
        Name(_ADR, 0x00000000)
        // _PLD as defined in the ACPI spec. The GroupToken and GroupPosition are used to
        // derive a unique "Connector ID". This PLD should correlate with the PLD associated
        // with the XHCI device for the same port.
        Name(_PLD, Package()
        {
            Buffer()
            {
                0x82,                   // Revision 2, ignore color.
                0x00,0x00,0x00,         // Color (ignored).
                0x00,0x00,0x00,0x00,    // Width and height.
                0x69,                   // User visible; Back panel; VerticalPos:Center.
                0x0c,                   // HorizontalPos:0; Shape:Vertical Rectangle; GroupOrientation:0.
                0x80,0x00,              // Group Token:0; Group Position:1; So Connector ID is 1.
                0x00,0x00,0x00,0x00,    // Not ejectable.
                0xFF,0xFF,0xFF,0xFF     // Vert. and horiz. offsets not supplied.
            }
        })

        // _UPC as defined in the ACPI spec.
        Name(_UPC, Package()
        {
            0x01,                       // Port is connectable.
            0x09,                       // Connector type: Type C connector - Type C connector - USB 2.0 and USB 3.0 with Switch.
            0x00000000,                 // Reserved0 must be zero.
            0x00000000                  // Reserved1 must be zero.
        })

        Name(_DSD, Package()
        {
            // The UUID for Type-C connector capabilities.
            ToUUID("6b856e62-40f4-4688-bd46-5e888a2260de"),

            // The data structure which contains the connector capabilities. Each package
            // element contains two elements: the capability type ID, and the capability data
            // (which depends on the capability type). Note that any information defined here
            // will override similar information described by the driver itself. For example, if
            // the driver claims the port controller is DRP-capable, but ACPI says it is UFP-only
            // ACPI will take precedence.

#ifdef POWER_DELIVERY
            Package()
            {
                Package() {USBC_CAP_ID_SUPPORTED_OP_MODES,             USBC_CAP_OP_MODE_DRP},                          // Supported operating modes
                Package() {USBC_CAP_ID_SUPPORTED_TYPEC_SRC_CAP,        USBC_CAP_TYPEC_SRC_1500_MA},                    // Supported Type-C sourcing capabilities
                Package() {USBC_CAP_ID_IS_AA_CAPABLE,                  USBC_CAP_AA_NOT_SUPPORTED},                     // Is Audio Accessory Capable
                Package() {USBC_CAP_ID_IS_PD_SUPPORTED,                USBC_CAP_PD_SUPPORTED},                         // Is PD supported
                Package() {USBC_CAP_ID_SUPPORTED_PD_ROLES,             USBC_CAP_PD_ROLE_SINK | USBC_CAP_PD_ROLE_SRC},  // Supported power roles (Sink).
                Package() {USBC_CAP_ID_PD_SOURCE_PDOS, Package() {   // Capability type ID of PD Source Capabilities.
                                  PD_SRC_FX(1,0,0,1,0,0,0,5000,1000) // Source PDO #0: Fixed: DRP=1, USB_SUSP=0, UC=0, USB_CC=1, DRD=0, UEM=0, PK=0, U=5V, I=1A.
                              }},
                Package() {USBC_CAP_ID_PD_SINK_PDOS, Package () {    // Capability type ID of PD Sink Capabilities.
                                  PD_SNK_FX(0,1,0,0,0, 5000,1000),   // Sink PDO #0: Fixed: DRP=0, HC=1, UC_PWR=0, USB_CC=0, DRD=0, U= 5V,  I=1A.
                                  PD_SNK_FX(0,1,0,0,0,15000,2500),   // Sink PDO #1: Fixed: DRP=0, HC=1, UC_PWR=0, USB_CC=0, DRD=0, U=15V,  I=2.5A.
                              }},
            }
#else
            Package()
            {
                Package() {USBC_CAP_ID_SUPPORTED_OP_MODES,             USBC_CAP_OP_MODE_DFP_ONLY},                     // Supported operating modes
                Package() {USBC_CAP_ID_SUPPORTED_TYPEC_SRC_CAP,        USBC_CAP_TYPEC_SRC_1500_MA},                    // Supported Type-C sourcing capabilities
                Package() {USBC_CAP_ID_IS_AA_CAPABLE,                  USBC_CAP_AA_NOT_SUPPORTED},                     // Is Audio Accessory Capable
                Package() {USBC_CAP_ID_IS_PD_SUPPORTED,                USBC_CAP_PD_NOT_SUPPORTED},                     // Is PD supported
            }
#endif
        })
    } // Device(CON1)} // Device(USBC)
}