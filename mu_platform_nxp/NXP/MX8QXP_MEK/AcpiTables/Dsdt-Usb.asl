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

Device (USB1)
{
  Name (_HID, "NXP010C")
  Name (_CID, "PNP0D20")
  Name (_UID, 0x1)
  // USB DMA is not coherent
  Name (_CCA, 0x0)
  // Hardware revision 1 supports integrated Transaction Translator (TT) in PortSC register
  Name (_HRV, 0x1)
  // D0 is the lowest supported state to wake itself up
  Name (_S0W, 0x0)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x5B0D0100, 0x100, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, SharedAndWake) { 299 }
  })

  OperationRegion (OTGM, SystemMemory, 0x5B0D0100, 0x100)
  Field (OTGM, WordAcc, NoLock, Preserve) {
    Offset (0x84),  // skip to register 84h
    PTSC, 32,       // port status control
    Offset (0xA8),  // skip to register A8h
    DSBM, 32,       // UOG_USBMOD
  }

  Name (REG, 0x0)
  Method (_UBF, 0x0, NotSerialized) {
    // Reset handled by driver so no reset required here
    Store (0x03, DSBM)          // set host mode & little endian
    Store (PTSC, REG)           // read PORTSC status
    Store (OR (REG, 0x2), PTSC) // clear current PORTSC status
  }
}

Device (USB2)
{
  Name (_HID, "NXPI010C") // NXPI: Windows requires 4 character vendor IDs
  Name (_CID, "PNP0D10")
  Name (_UID, 0x2)
  Name (_CCA, 0x0)    // XHCI is not coherent

  Method(_CRS, 0x0, Serialized) {
    Name(RBUF, ResourceTemplate() {
      Memory32Fixed(ReadWrite, 0x5B130000, 0x40000)
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) {303}
      })
    Return(RBUF)
  }
  Method(_PS0, 0x0, Serialized) {
  }
  Method(_PS3, 0x0, Serialized) {
  }
}