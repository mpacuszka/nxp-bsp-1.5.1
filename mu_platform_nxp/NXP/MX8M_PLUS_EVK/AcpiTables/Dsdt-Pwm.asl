/** @file
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright 2019, 2023 NXP
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

Device (PWM1)
{
  Name (_HID, "NXP010E")
  Name (_UID, 0x1)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x30660000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 113 }
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 25000000}, 
      Package (2) {"PinCount",          1},
      Package (2) {"Pwm-SchematicName", "PWM_1"},
      // To Enable PWM after boot, uncomment these lines and specify required parameters
      // If you need to enable different PWM instance, then move these lines to appropriate 
      // device block.

      // Specify if the PWM will be enabled after boot (1 enabled, 0 disabled) 
      // Useful for PWM handled LCD panels for exemple
      // Package (2) {"BootOn", 0},                    
      // Desired PWM period used after boot (if BootOn is 1). 
      // Period is 64 bit value -> then must be splitted to two 32 bit integers at ACPI table
      // Package (2) {"BootDesiredPeriodLowPart", 2000000000},    
      // Package (2) {"BootDesiredPeriodHighPart", 0},
      // PWM Duty cycle used after boot (if BootOn is 1). 
      // Duty cycle is 64 bit value -> then must be splitted to two 32 bit integers at ACPI table
      // (these values represent 50%)
      // Package (2) {"BootActiveDutyCycleLowPart", 4294967295},
      // Package (2) {"BootActiveDutyCycleHighPart", 1073741823},
      // PWM polarity used after boot (if BootOn is 1). 
      // 1 -> PWM_ACTIVE_HIGH
      // 0 -> PWM_ACTIVE_LOW
      // Package (2) {"BootPolarity", 1},
    }
  })
}

Device (PWM2)
{
  Name (_HID, "NXP010E")
  Name (_UID, 0x2)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x30670000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 114 }
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 25000000}, 
      Package (2) {"PinCount",          1},
      Package (2) {"Pwm-SchematicName", "PWM_2"},
    }
  })
}

Device (PWM3)
{
  Name (_HID, "NXP010E")
  Name (_UID, 0x3)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x30680000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 115 }
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 25000000}, 
      Package (2) {"PinCount",          1},
      Package (2) {"Pwm-SchematicName", "PWM_3"},
    }
  })
}

Device (PWM4)
{
  Name (_HID, "NXP010E")
  Name (_UID, 0x4)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x30690000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 116 }
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 25000000}, 
      Package (2) {"PinCount",          1},
      Package (2) {"Pwm-SchematicName", "PWM_4"},
    }
  })
}
