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

#include "UcmTcpciConstatnts.h"

// UCM-TCPCI device.
Device(USC0)  // USB2.0 Type C, J401 on Base board, I2C3, Address 50
{
    // This device needs to be enumerated by ACPI, so it needs a HWID.
    // Your INF should match on it.
    Name(_HID, "USBC0001")
    Name (_UID, 0x0)
    Method(_CRS, 0x0, NotSerialized)
    {
        Name (RBUF, ResourceTemplate ()
        {
            //
            // Sample I2C and GPIO resources.
            // platform's underlying controllers and connections.
            // \_SB.I2C and \_SB.GPIO are paths to predefined I2C and GPIO controller instances.
            // \_SB.I2C and \_SB.GPIO are paths to predefined I2C and GPIO controller instances.
            //  GPIO2_IO011 SD1_STROBE
            I2CSerialBus(0x50, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C3")
            GpioIO(Shared, PullNone, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_CCM_CLKO2 }
            GpioInt(Level, ActiveLow, Shared, PullUp, 0, "\\_SB.GPIO",) { IMX_PAD_CCM_CLKO2 }
        })
        Return(RBUF)
    }

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
            0x08,                       // Type C connector - USB2-only.
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
                Package() {2, USB_TYPE_C_SOURCE_DEFAULT} // Supported Type-C sourcing capabilities
            }
        })
    } // Device(CON0)}
} // USC0

// UCM-TCPCI device.
Device(USC1)  // USB2.0 Type C, J302 on Base board, I2C3, Address 51
{
    // This device needs to be enumerated by ACPI, so it needs a HWID.
    // Your INF should match on it.
    Name(_HID, "USBC0001")
    Name (_UID, 0x1)
    Method(_CRS, 0x0, NotSerialized)
    {
        Name (RBUF, ResourceTemplate ()
        {
            //
            // Sample I2C and GPIO resources.
            // platform's underlying controllers and connections.
            // \_SB.I2C and \_SB.GPIO are paths to predefined I2C and GPIO controller instances.
            // \_SB.I2C and \_SB.GPIO are paths to predefined I2C and GPIO controller instances.
            //  GPIO2_IO011 SD1_STROBE
            I2CSerialBus(0x51, ControllerInitiated, 400000, AddressingMode7Bit, "\\_SB.I2C3")
            GpioIO(Shared, PullNone, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PAD_CCM_CLKO2 }
            GpioInt(Level, ActiveLow, Shared, PullUp, 0, "\\_SB.GPIO",) { IMX_PAD_CCM_CLKO2 }
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
            0x08,                       // Type C connector - USB2-only.
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
                Package() {2, USB_TYPE_C_SOURCE_DEFAULT} // Supported Type-C sourcing capabilities
            }
        })
    } // Device(CON1)
} // USC1