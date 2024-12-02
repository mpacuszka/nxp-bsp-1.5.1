/*
* This file is derived from Microsoft example available on https://github.com/microsoft/Windows-driver-samples/blob/master/usb/UcmTcpciCxClientSample
* Original file licensing:
* Copyright (c) 2015 Microsoft
* SPDX-License-Identifier: MS-PL
* NXP modifications are licensed under the same license
* Copyright 2022 NXP
*
*/

#include "UcmTcpciConstatnts.h"

// UCM-TCPCI device.
Device(USC0) // USB3.0 Type-C (NOT the power supply!)
{
    // This device needs to be enumerated by ACPI, so it needs a HWID.
    // Your INF should match on it.
    Name(_HID, "USBC0001")
    Method(_CRS, 0x0, NotSerialized)
    {
        Name (RBUF, ResourceTemplate ()
        {
            // I2C device: I2C2.
            I2CSerialBus(0x50, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C2")
            // PTN5110 interrupt: Pad: SAI1_TXD7, GPIO: GPIO4_IO19
            GpioIO(Shared, PullNone, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 115 } // (GPIO4_IO19 = 3 * 32 + 19 = 115)
            GpioInt(Level, ActiveLow, Exclusive, PullUp, 0, "\\_SB.GPIO",) { 115 }
            // CBTU02043 switch SEL: Pad: SAI1_MCLK, GPIO: GPIO4_IO20
            GpioIO(Shared, PullNone, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 116 } // (GPIO4_IO20 = 3 * 32 + 20 = 116)
        })
        Return(RBUF)
    }
    Name (_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () { // CBTU02043 Differential Channel Crossbar Switch settings
            Package (2) {"SwitchPin_Index", 1},
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

            Package()
            {
                Package() {1, 4},                                    // Supported operating modes (DRP).
                Package() {2, USB_TYPE_C_SOURCE_DEFAULT},            // Supported Type-C sourcing capabilities. (USB 2.0 - 500 mA, USB 3.1 - 900 mA)
                Package() {4, 0},                                    // Is PD supported (False).
            }
        })
    } // Device(CON0)
} // Device(USBC)


// Uncoment next line for Power delivery setting example
//#define PD_TEST
#ifdef PD_TEST
// UCM-TCPCI device.
Device(USC1) // USB Type-C Power Supply
{
    // This device needs to be enumerated by ACPI, so it needs a HWID.
    // Your INF should match on it.
    Name(_HID, "USBC0001")
    Method(_CRS, 0x0, NotSerialized)
    {
        Name (RBUF, ResourceTemplate ()
        {
            // I2C device: I2C3.
            I2CSerialBus(0x50, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C3")
            // PTN5110 interrupt: Pad: SAI1_TXD6, GPIO: GPIO4_IO18
            GpioIO(Shared, PullNone, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { 114 } // (GPIO4_IO18 = 3 * 32 + 18 = 115)
            GpioInt(Level, ActiveLow, Exclusive, PullUp, 0, "\\_SB.GPIO",) { 114 }
        })
        Return(RBUF)
    }
    // Inside the scope of the UCM-TCPCI device, you need to define one "connector" device.
    // It can be named anything.
    Device(CON1)
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

            Package()
            {
                Package() {1, USB_TYPE_C_ROLE_SNK_ONLY},             // Supported operating modes.
                Package() {2, USB_TYPE_C_SOURCE_DEFAULT},            // Supported Type-C sourcing capabilities. (USB 2.0 - 500 mA, USB 3.1 - 900 mA)
                Package() {3, 0},                                    // Audio accessory capable (False).
                Package() {4, 0},                                    // Is PD supported (False).
                Package() {4, 1},                                    // Is PD supported (True)
                Package() {5, USB_TYPE_C_POWER_ROLE_SNK},            // Supported power roles (Sink).
                Package() {7, Package () {                           // Capability type ID of PD Sink Capabilities.
                                  PD_SNK_FX(1,1,0,1,0, 5000,1000),   // Sink PDO #0: Fixed: DRP=1, HC=0, UC=0, USB_CC=1, DRD=0, U= 5V,  I=1A.
                                  PD_SNK_FX(1,1,0,1,0,15000,2500),   // Sink PDO #0: Fixed: DRP=1, HC=0, UC=0, USB_CC=1, DRD=0, U=15V,  I=2.5A.
                              }},
            }
        })
    } // Device(CON1)
} // Device(USBC)
#endif