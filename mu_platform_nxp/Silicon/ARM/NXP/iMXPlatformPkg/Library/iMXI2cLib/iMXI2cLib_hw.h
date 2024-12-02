/** @file
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright 2020, 2022 NXP
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

#ifndef _IMX_I2C_HW_H_
#define _IMX_I2C_HW_H_

#include <stdint.h>

#define IMX_I2C_I2SR_RXAK        0x0001
#define IMX_I2C_I2SR_IIF         0x0002
#define IMX_I2C_I2SR_SRW         0x0004
#define IMX_I2C_I2SR_IAL         0x0010
#define IMX_I2C_I2SR_IBB         0x0020
#define IMX_I2C_I2SR_IAAS        0x0040
#define IMX_I2C_I2SR_ICF         0x0080

#define IMX_I2C_I2SR_IIF_NO_INTERRUPT_PENDING   0x0
#define IMX_I2C_I2SR_IIF_INTERRUPT_PENDING      0x1
#define IMX_I2C_I2SR_IAL_NO_ARBITRATION_LOST    0x0
#define IMX_I2C_I2SR_IAL_ARBITRATION_LOST       0x1

#define IMX_I2C_I2CR_RSTA_REPEAT_START_DISABLE  0x0
#define IMX_I2C_I2CR_RSTA_REPEAT_START_ENABLE   0x1
#define IMX_I2C_I2CR_TXAK_SEND_TRANSMIT_ACK     0x0
#define IMX_I2C_I2CR_TXAK_NO_TRANSMIT_ACK       0x1
#define IMX_I2C_I2CR_MTX_RECEIVE_MODE           0x0
#define IMX_I2C_I2CR_MTX_TRANSMIT_MODE          0x1
#define IMX_I2C_I2CR_MSTA_SLAVE_MODE            0x0
#define IMX_I2C_I2CR_MSTA_MASTER_MODE           0x1
#define IMX_I2C_I2CR_IEN_INTERRUPT_DISABLED     0x0
#define IMX_I2C_I2CR_IEN_INTERRUPT_ENABLED      0x1

#define IMX_I2C_TX 0
#define IMX_I2C_RX 1

typedef union {
  UINT16 Raw;
  struct {
    UINT16 Reserved0 : 1;
    UINT16 ADR : 7;
    UINT16 Reserved1 : 8;
  };
} IMX_I2C_IADR_REGISTER;

typedef union {
  UINT16 Raw;
  struct {
    UINT16 IC : 6;
    UINT16 Reserved0 : 10;
  };
} IMX_I2C_IFDR_REGISTER;

typedef union {
  UINT16 Raw;
  struct {
    UINT16 Reserved0 : 2;
    UINT16 RSTA : 1;
    UINT16 TXAK : 1;
    UINT16 MTX : 1;
    UINT16 MSTA : 1;
    UINT16 IIEN : 1;
    UINT16 IEN : 1;
    UINT16 Reserved1 : 8;
  };
} IMX_I2C_I2CR_REGISTER;

typedef union {
  UINT8 Raw;
  struct {
    UINT8 Direction : 1;
    UINT8 DeviceAddress : 7;
  };
} IMX_I2C_DEVICE_ADDRESS_PACKET;

typedef union {
  UINT16 Raw;
  struct {
    UINT16 RXAK : 1;
    UINT16 IIF : 1;
    UINT16 SRW : 1;
    UINT16 Reserved0 : 1;
    UINT16 IAL : 1;
    UINT16 IBB : 1;
    UINT16 IAAS : 1;
    UINT16 ICF : 1;
    UINT16 Reserved1 : 8;
  };
} IMX_I2C_I2SR_REGISTER;

typedef union {
  UINT16 Raw;
  struct {
    UINT16 DATA : 8;
    UINT16 Reserved0 : 8;
  };
} IMX_I2C_I2DR_REGISTER;

typedef struct {
  IMX_I2C_IADR_REGISTER IADR;
  UINT16 Reserved0;
  IMX_I2C_IFDR_REGISTER IFDR;
  UINT16 Reserved1;
  IMX_I2C_I2CR_REGISTER I2CR;
  UINT16 Reserved2;
  IMX_I2C_I2DR_REGISTER I2SR;
  UINT16 Reserved3;
  IMX_I2C_I2DR_REGISTER I2DR;
  UINT16 Reserved4;
} IMX_I2C_REGISTERS;

typedef struct {
  UINT32 Divider;
  UINT32 I2cClockRate;
} IMX_I2C_DIVIDER;

#endif
