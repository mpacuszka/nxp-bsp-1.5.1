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

//
// uSDHC1 - eMMC
//
Device (SDH1)
{
   Name (_HID, "NXP0108")
   Name (_UID, 0x1)
   Name (_CCA, 0x0)    // SDHC is not coherent

   Method (_STA) // State
   {
       Return(0xf) // Enabled
   }

   Method (_CRS, 0x0, NotSerialized) {
       Name (RBUF, ResourceTemplate () {
           MEMORY32FIXED(ReadWrite, 0x5B010000, 0x4000, )
           Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 264 }
       })
       Return(RBUF)
   }

   //
   // Child node to represent the only SD/MMC slot on this SD/MMC bus
   // In theory an SDHC can be connected to multiple SD/MMC slots at
   // the same time, but only 1 device will be selected and active at
   // a time
   //
   Device (SD0)
   {
       Method (_ADR) // Address
       {
         Return (0) // SD/MMC Slot 0
       }

       Method (_RMV) // Remove
       {
         Return (0) // Fixed
       }
   }

   Name (_DSD, Package () {
     ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
       Package () {
         Package (2) {"RegisterBasePA", 0x5B010000},
         Package (2) {"BaseClockFrequencyHz", 400000000},
         Package (2) {"Regulator1V8Exist", 1},
         Package (2) {"TuningStartTap", 20},
         Package (2) {"TuningStep", 2}
       }
   })
}

//
// uSDHC2 - uSD
//
Device (SDH2)
{
   Name (_HID, "NXP0108")
   Name (_UID, 0x2)
   Name (_CCA, 0x0)    // SDHC is not coherent

   Method (_STA)
   {
       Return(0xf)
   }

   Method (_CRS, 0x0, NotSerialized) {
       Name (RBUF, ResourceTemplate () {
           MEMORY32FIXED(ReadWrite, 0x5B020000, 0x4000, )
           Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 265 }
           // Interrupt pin: Pad: USDHC1_CD_B  (GPIO4_IO22)
           GpioIO(Shared, PullNone, 0, 0, IoRestrictionNone, "\\_SB.GPIO", 0, ResourceConsumer, , ) { IMX_PIN_USDHC1_CD_B }
           GpioInt(Edge, ActiveBoth, Shared, PullNone, 0,    "\\_SB.GPIO", 0,                 , , ) { IMX_PIN_USDHC1_CD_B }

       })
       Return(RBUF)
   }

   //
   // Child node to represent the only SD/MMC slot on this SD/MMC bus
   // In theory an SDHC can be connected to multiple SD/MMC slots at
   // the same time, but only 1 device will be selected and active at
   // a time
   //
   Device (SD0)
   {
       Method (_ADR) // Address
       {
         Return (0) // SD/MMC Slot 0
       }

       Method (_RMV) // Remove
       {
         Return (1) // Not fixed
       }
   }

   Name (_DSD, Package () {
     ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
       Package () {
         Package (2) {"RegisterBasePA", 0x5B020000},
         Package (2) {"BaseClockFrequencyHz", 200000000},
         Package (2) {"Regulator1V8Exist", 0}
       }
   })
}

