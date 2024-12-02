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

Device (TPM3)  // Domain: WAKEUPMIX, InputClock: bus_wakeup_clk_root (Max 133 MHz), LPCG: 46
// CH0: GPIO_IO04 ALT1  Green LED
// CH1: GPIO_IO20 ALT6
// CH2: GPIO_IO12 ALT1  Blue LED
// CH3: GPIO_IO24 ALT4
{
  Name (_HID, "NXP0123")
  Name (_UID, 0x3)
  Method (_STA) {
    Return (0xf)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x424E0000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 107 }  // 75 + 32
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 133000000},
      Package (2) {"PinCount",          4},
      Package (2) {"Pwm-SchematicName", "PWM_3"},
    }
  })
}

Device (TPM4)  // Domain: WAKEUPMIX, InputClock: tpm4_clk_root (Max 83 MHz), LPCG: 47
// CH0: GPIO_IO05 ALT1
// CH1: GPIO_IO21 ALT6
// CH2: GPIO_IO13 ALT1 Red LED
// CH3: GPIO_IO25 ALT4
{
  Name (_HID, "NXP0123")
  Name (_UID, 0x4)
  Method (_STA) {
    Return (0xf)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x424F0000, 0x10000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 108 }  // 76 + 32
  })
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"ClockFrequency_Hz", 8000000},  // See PwmInit() method in iMXBoardInit.c
      Package (2) {"PinCount",          4},
      Package (2) {"Pwm-SchematicName", "PWM_4"},
    }
  })
}
