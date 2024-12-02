/*
* Description: Processor Devices
*
*  Copyright (c) Microsoft Corporation. All rights reserved.
*  Copyright 2023 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
*/


OperationRegion(FUSE,SystemMemory,0x30350400,0x900)      // 0x3035_0D00
Field(FUSE, AnyAcc, Nolock, Preserve)
{
  Offset(0x240),
  MC15, 8,          // 0x640 NET1 MAC address bytes 5
  MC14, 8,          // 0x641 NET1 MAC address bytes 4
  MC13, 8,          // 0x642 NET1 MAC address bytes 3
  MC12, 8,          // 0x643 NET1 MAC address bytes 2
  Offset(0x250),
  MC11, 8,          // 0x650 NET1 MAC address bytes 1
  MC10, 8,          // 0x651 NET1 MAC address bytes 0
  MC25, 8,          // 0x652 NET2 MAC address bytes 5
  MC24, 8,          // 0x653 NET2 MAC address bytes 4
  Offset(0x260),
  MC23, 8,          // 0x660 NET2 MAC address bytes 3
  MC22, 8,          // 0x661 NET2 MAC address bytes 2
  MC21, 8,          // 0x662 NET2 MAC address bytes 1
  MC20, 8,          // 0x663 NET2 MAC address bytes 0
}

//
// Description: This is a Processor #0 Device
//
Device (CPU0)
{
    Name (_HID, "ACPI0007")
    Name (_UID, 0x0)
    Method (_STA)
    {
        Return(0xf)
    }
}

//
// Description: This is a Processor #1 Device
//
#if FixedPcdGet32(PcdCoreCount) > 1
Device (CPU1)
{
    Name (_HID, "ACPI0007")
    Name (_UID, 0x1)
    Method (_STA)
    {
        Return(0xf)
    }
}
#endif

//
// Description: This is a Processor #2 Device
//
#if FixedPcdGet32(PcdCoreCount) > 2
Device (CPU2)
{
    Name (_HID, "ACPI0007")
    Name (_UID, 0x2)
    Method (_STA)
    {
        Return(0xf)
    }
}
#endif

//
// Description: This is a Processor #3 Device
//
#if FixedPcdGet32(PcdCoreCount) > 3
Device (CPU3)
{
    Name (_HID, "ACPI0007")
    Name (_UID, 0x3)
    Method (_STA)
    {
        Return(0xf)
    }
}
#endif

// Power Engine Plugin
Device (PEP0)
{
  Name (_HID, "NXP0122")
  Name (_UID, 0x0)

  Method (_STA) {
    Return (0xf)
  }
  
  // Enable this section when RUNTIME_DEBUG is TRUE
  // Allow usage of UART2 for UEFI Runtime Debug
  /*Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      MEMORY32FIXED(ReadWrite, 0x30890000, 0x4000, )
    })
    Return(RBUF)
  }*/
}

#if FixedPcdGet32(PcdPowerButtonEnabled)

// Power button device description, see PWRB in ACPI 6.x Manual
Device(PWRB) {
    Name(_HID, EISAID("PNP0C0C"))
    Name(_UID, 0)
    Method (_STA, 0x0, NotSerialized) {
        Return (0xF)
    }
    Name(_PRW, Package(){0, 0x4})
    
    OperationRegion (SNVS, SystemMemory, 0x30370000, 0x10000)
    Field (SNVS, DWordAcc, NoLock, Preserve)
    {
        Offset(0x004C),
        LPCR, 32, // iMX_SNVS_LPSR SNVS Low Power Status Register
    }
}

// Generic Event Device - for power button interrupt 4 (=36)
// See iMX Reference Manual: The interrupt 4 (GSIV = 36 = 32 + 4) comes from 
// SNVS_LP/HP_WRAPPER OR ON-OFF button press shorter than 5 secs (pulse event)
Device (GED1)
{
    Name(_HID,"ACPI0013")
    Name(_CRS, ResourceTemplate ()
    {
        Interrupt(ResourceConsumer, Level, ActiveHigh, ExclusiveAndWake) {36}
    })

    Method (_EVT,1) { // Handle all ACPI Events signaled by the Generic Event Device(GED1)
        Switch (Arg0) // Arg0 = GSIV of the interrupt
        {
            Case (36) { // interrupt 36
                If (\_SB.PWRB.LPCR & 0x00040000) // Bit 18 ... Set Power Off = power button was pressed
                {
                    Local0 = \_SB.PWRB.LPCR
                    \_SB.PWRB.LPCR = Local0  // Writing to the SPO bit will clear the set_pwr_off_irq interrupt 
                                             // (also in GIC GICD_ICPENDR1 offset 0x284)
                    Local1 = \_SB.PWRB.LPCR  // Read back to verify that the bit was cleared
                    Notify(\_SB.PWRB, 0x80)  // Notify OS of event
                }
            }
        }
    }
} // End of Scope

#endif // PcdPowerButtonEnabled
