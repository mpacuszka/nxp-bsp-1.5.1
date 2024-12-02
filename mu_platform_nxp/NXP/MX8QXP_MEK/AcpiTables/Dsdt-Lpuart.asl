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
 
/*
 * Description: iMX8X LPUART Controllers
 *
 * iMX 8QXP CPU Board LPUART pinout:
 * LPUART 0 - Debug, FTDI port 0 UART
 * LPUART 1 - M.2 MINI CARD - WiFi UART
 * LPUART 2 - BB RS232 J37
 * LPUART 3 - BB Audio UART
 */



Device (LPU0)
{
    Name (_HID, "NXP0116")
    Name (_UID, 0x0)
    Name (_DDN, "LPUART0")
    Method (_STA)
    {
        Return(0xF) // Enable
    }
    Method (_CRS, 0x0, NotSerialized) {
        Name (RBUF, ResourceTemplate () {
            Memory32Fixed (ReadWrite, 0x5A060000, 0x30, )
            Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 257 }

            // DMA channel 8, SDMA_REQ_UART1_RX for LPUART0 RX DMA
            FixedDMA (SDMA_REQ_UART1_RX, 8, Width8Bit, )
            // DMA channel 9, SDMA_REQ_UART1_TX for LPUART0 TX DMA
            FixedDMA (SDMA_REQ_UART1_TX, 9, Width8Bit, )
        })
        Return(RBUF)
    }

    Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
            Package (2) {"SerCx-FriendlyName", "LPUART0"}
        }
    })
}

Device (LPU1)
{
    Name (_HID, "NXP0116")
    Name (_UID, 0x1)
    Name (_DDN, "LPUART1")
    Method (_STA)
    {
#if FixedPcdGetBool(PcdLpuart1Enable)
        Return(0xF) // Enable
#else
        Return(0x0) // Disable
#endif
    }
    Method (_CRS, 0x0, NotSerialized) {
        Name (RBUF, ResourceTemplate () {
            Memory32Fixed (ReadWrite, 0x5A070000, 0x30, )
            Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 258 }

            // DMA channel 10, SDMA_REQ_UART2_RX for LPUART1 RX DMA
            FixedDMA (SDMA_REQ_UART2_RX, 10, Width8Bit, )
            // DMA channel 11, SDMA_REQ_UART2_TX for LPUART1 TX DMA
            FixedDMA (SDMA_REQ_UART2_TX, 11, Width8Bit, )
        })
        Return(RBUF)
    }

    Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
            Package (2) {"SerCx-FriendlyName", "LPUART1"},
            Package (2) {"rts-cts-enabled", 1}
        }
    })
}

Device (LPU2)
{
    Name (_HID, "NXP0116")
    Name (_UID, 0x2)
    Name (_DDN, "LPUART2")
    Method (_STA)
    {
        Return(0xF) // Enable
    }
    Method (_CRS, 0x0, NotSerialized) {
        Name (RBUF, ResourceTemplate () {
            Memory32Fixed (ReadWrite, 0x5A080000, 0x30, )
            Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 259 }

            // DMA channel 12, SDMA_REQ_UART3_RX for LPUART2 RX DMA
            FixedDMA (SDMA_REQ_UART3_RX, 12, Width8Bit, )
            // DMA channel 13, SDMA_REQ_UART3_TX for LPUART2 TX DMA
            FixedDMA (SDMA_REQ_UART3_TX, 13, Width8Bit, )
        })
        Return(RBUF)
    }

    Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
            Package (2) {"SerCx-FriendlyName", "LPUART2"}
        }
    })
}