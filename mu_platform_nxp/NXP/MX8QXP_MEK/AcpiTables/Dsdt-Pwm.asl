/*
 * Copyright 2022-2023 NXP
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

Device (PWM1)
{
  Name (_HID, "NXP010E")
  Name (_UID, 0x1)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x5D000000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 126 }
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 66000000}, 
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
    MEMORY32FIXED (ReadWrite, 0x5D010000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 127 }
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 66000000}, 
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
    MEMORY32FIXED (ReadWrite, 0x5D020000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 128 }
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 66000000}, 
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
    MEMORY32FIXED (ReadWrite, 0x5D030000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 129 }
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 66000000}, 
      Package (2) {"PinCount",          1},
      Package (2) {"Pwm-SchematicName", "PWM_4"},
    }
  })
}
