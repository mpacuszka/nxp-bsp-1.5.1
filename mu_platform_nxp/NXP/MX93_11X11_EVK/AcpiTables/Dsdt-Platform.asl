/*
*  iMX9 Processor Devices
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright 2022-2023 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

OperationRegion(FUSE,SystemMemory,0x47518000,0x1000)
Field(FUSE, AnyAcc, Nolock, Preserve)
{
  Offset(0x4EC),
  MC15, 8,          // NET1 MAC address bytes
  MC14, 8,          // NET1 MAC address bytes
  MC13, 8,          // NET1 MAC address bytes
  MC12, 8,          // NET1 MAC address bytes

  Offset(0x4F0),
  MC11, 8,          // NET1 MAC address bytes
  MC10, 8,          // NET1 MAC address bytes
  MC21, 8,          // NET2 MAC address bytes
  MC20, 8,          // NET2 MAC address bytes

  Offset(0x4F4),
  MC25, 8,          // NET2 MAC address bytes,
  MC24, 8,          // NET2 MAC address bytes,
  MC23, 8,          // NET2 MAC address bytes,
  MC22, 8,          // NET2 MAC address bytes,
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


// Power Engine Plugin
Device (PEP0)
{
  Name (_HID, "NXP0122")
  Name (_UID, 0x0)

  Method (_STA) {
    Return (0xf)
  }
}

// Power button device description, see PWRB in ACPI 6.x Manual
Device(PWRB) {
    Name(_HID, EISAID("PNP0C0C"))
    Name(_UID, 0)
    Method (_STA, 0x0, NotSerialized) {
        Return (0xF)
    }
    Name(_PRW, Package(){0, 0x4})
    
    // BBNSM = 93's Battery Back Non-Secure Module (replacement for SNVS module in M-Scales)
    OperationRegion (BBNS, SystemMemory, 0x44440000, 0x10000)
    Field (BBNS, DWordAcc, NoLock, Preserve)
    {
        Offset(0x0014),
        BBEV, 32, // BBNSM Events Register 
    }
}

// Generic Event Device - for power button interrupt 73 (73+32=105)
// See iMX Reference Manual: The interrupt 73 (GSIV = 105 = 32 + 73) comes from 
// BBNSM ON-OFF button press shorter than 5 secs (pulse event)
Device (GED1)
{
    Name(_HID,"ACPI0013")
    Name(_CRS, ResourceTemplate ()
    {
        Interrupt(ResourceConsumer, Level, ActiveHigh, ExclusiveAndWake) {105}
    })

    Method (_EVT,1) { // Handle all ACPI Events signaled by the Generic Event Device(GED1)
        Switch (Arg0) // Arg0 = GSIV of the interrupt
        {
            Case (105) {
                If (\_SB.PWRB.BBEV & 0x00000020) // Bit 5  ... Set Power Off Event = power button was pressed
                {
                    Local0 = \_SB.PWRB.BBEV
                    \_SB.PWRB.BBEV = Local0  // Writing to the SPO bit will clear the set_pwr_off_irq interrupt 
                                             // (also in GIC GICD_ICPENDR1 offset 0x284)
                    Local1 = \_SB.PWRB.BBEV  // Read back to verify that the bit was cleared
                    Notify(\_SB.PWRB, 0x80)  // Notify OS of event
                }
            }
        }
    }
} // End of Scope

