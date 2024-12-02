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
 
/*
 * Description: iMX93 LPUART Controllers
 *
 * pinout:
 * LPUART 1 - FTC_TXD, CORTEX-A CONSOLE
 * LPUART 2 - n/a (FTD_TXD, CORTEX-M CONSOLE)
 * LPUART 3 - RPi Expander UART (J1001[8] Tx, J1001[10] Rx)

 */

/* The MIMX9352_ca55.h defines the following clock target and lpcg ids: */
#define CCM_TARGET_LPUART1 25U
#define CCM_TARGET_LPUART2 26U
#define CCM_TARGET_LPUART3 27U
#define CCM_TARGET_LPUART4 28U
#define CCM_TARGET_LPUART5 29U
#define CCM_TARGET_LPUART6 30U
#define CCM_TARGET_LPUART7 31U
#define CCM_TARGET_LPUART8 32U

#define CCM_LPCG_LPUART1   52U
#define CCM_LPCG_LPUART2   53U
#define CCM_LPCG_LPUART3   54U
#define CCM_LPCG_LPUART4   55U
#define CCM_LPCG_LPUART5   56U
#define CCM_LPCG_LPUART6   57U
#define CCM_LPCG_LPUART7   58U
#define CCM_LPCG_LPUART8   59U

Device (LPU1)
{
  Name (_HID, "NXP0116")
  Name (_UID, 0x1)
  Name (_DDN, "LPUART1")

  Name (CCGR, CCM_LPCG_LPUART1)
  Name (CCTR, CCM_TARGET_LPUART1)

  Method (_STA)
  {
    Return(0xF) // Enable
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      Memory32Fixed (ReadWrite, 0x44380000, 0x30, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 51 }
    })
    Return(RBUF)
  }

  Method (_PSC, 0, NotSerialized)
  /* _PSC: Power State Current */
  {
    Local0 = CGST (CCGR)

    If ((Local0 == 0x00)) { // LPCG is off
      Return (3)
    }
    Return (0)
  }

  Method (_PS0, 0, NotSerialized)
  /* _PS0: Power State 0 */
  {
    CRSE (CCGR, CCTR, 2U, 5U)
  }

  Method (_PS3, 0, NotSerialized)
  /* _PS3: Power State 3 */
  {
    CGOF(CCGR)
  }

  Name (_DSD, Package () {
  ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"SerCx-FriendlyName", "LPUART1"},
    }
  })
}

Device (LPU3)
{
  Name (_HID, "NXP0116")
  Name (_UID, 0x3)
  Name (_DDN, "LPUART3")
  
  Name (CCGR, CCM_LPCG_LPUART3)
  Name (CCTR, CCM_TARGET_LPUART3)
  
  Method (_STA)
  {
    Return(0xF) // Enable
  }

  Method (_CRS, 0x0, NotSerialized) {
    Name (RBUF, ResourceTemplate () {
      Memory32Fixed (ReadWrite, 0x42570000, 0x30, )
      Interrupt(ResourceConsumer, Level, ActiveHigh, Exclusive) { 100 }
    })
    Return(RBUF)
  }
  
  Method (_PSC, 0, NotSerialized)
  /* _PSC: Power State Current */
  {
    Local0 = CGST (CCGR)

    If ((Local0 == 0x00)) { // LPCG is off
      Return (3)
    }
    Return (0)
  }

  Method (_PS0, 0, NotSerialized)
  /* _PS0: Power State 0 */
  {
    CRSE (CCGR, CCTR, 2U, 5U)  // Sert clock root to 80 MHz, SYS_PLL_PFD1_DIV2 400 MHz divided by 5, and turn on clock
  }

  Method (_PS3, 0, NotSerialized)
  /* _PS3: Power State 3 */
  {
    CGOF(CCGR)
  }

  Name (_DSD, Package () {
  ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () {
      Package (2) {"SerCx-FriendlyName", "LPUART3"}
    }
  })
}
