/*
*
* Description: 10/100/1000-Mbps Ethernet MAC (ENET)
*
*  Copyright (c) Microsoft Corporation. All rights reserved.
*  Copyright 2019, 2022 NXP
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

#include "ENETPhyRegs.h"

Device (NET1) {
  Name (_HID, "NXP010D")
  Name (_UID, 0x1)
  Method (_STA) {
    Return (0xF)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x5B040000, 0x628, )                      // ENET1 MAC Core Base address
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 290 }  // ENET1 MAC Core IRQ
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 291 }  // ENET1 1588 Timer interrupt
  })
  Name (MAC, Buffer (6) {})
  CreateDWordField (MAC, 0x00, MACL)
  CreateWordField (MAC, 0x04, MACH)
  // Device Specific Method takes 4 args:
  //  Arg0 : Buffer containing a UUID [16 bytes]
  //  Arg1 : Integer containing the Revision ID
  //  Arg2 : Integer containing the Function Index
  //  Arg3 : Package that contains function-specific arguments
  Method (_DSM, 0x4, Serialized) {
    // iMX Enet interface identifier
    if (LEqual (Arg0, ToUUID ("4EBBCB9D-4087-4791-80BD-8F8C08024206"))) {
      // Function selector
      switch (ToInteger (Arg2)) {
        // Function 0: Query support
        //   Bit  Description
        //   ---  -------------------------------
        //     0  Get property (Function 0)
        //     1  Get Mac Address (Function 1)
        case (0) {
          switch (ToInteger (Arg1)) {
            // Revision 0:
            case (0) {
              // Functions {0,1} supported
              Return (Buffer () { 0x03 })
            }
            default {
              Return (Buffer () { 0x00 })
            }
          }
        }
        // Function 1: Return Mac Address
        case (1) {
          if (LEqual (MC0V, 1)) {
            Store (MC0L, MACL);
            Store (MC0H, MACH);
            Return (MAC)
          }
        }
        default {
          Return (Buffer () { 0x00 })
        }
      }
    }
    Return (Buffer () { 0x00 })
  }
  Name (_DSD, Package () {
    ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
    Package () { // ATHEROS AR8031
      Package (2) {"MDIOBusController_InputClk_kHz", 250000},
      Package (2) {"PhyAddress",                     0x00},
      Package (2) {"PhyInterafceType",               0x00},  // RGMII, default value
      Package (2) {"PhyMaxMDIOBusClock_kHz",         15000},
      Package (2) {"PhyMinSTAHoldTime_ns",           10},
      Package (2) {"PhyDisablePreamble",             0},
      Package (2) {"ConfigCmds", Package () {
                                  //  Enable GTX_CLK delay
                                  MII_WRITE_COMMAND(MII_REG_AR8031_DP_ADDR, 0x0005),// Choose SerDes Test and System Mode Control
                                  MII_WRITE_COMMAND(MII_REG_AR8031_DP_RW,   0x0100),// Select 1 - RGMII Tx Clock Delay Enable
                                  ///  Specific
                                  MII_WRITE_COMMAND(MII_REG_AR8031_SS,      0x000C),// Smart speed off
                                  ENET_MII_END}}
    }
  })
}
