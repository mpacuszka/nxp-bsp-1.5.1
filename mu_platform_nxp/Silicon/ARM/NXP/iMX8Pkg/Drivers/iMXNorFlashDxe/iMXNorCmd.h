/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef IMX_NOR_CMD_H_
#define IMX_NOR_CMD_H_

#include <stdio.h>

/* The Command structure sent to NorFspi, equivalent to uboot's spi_mem_op */
typedef struct {
  struct {
    UINT8 byte_cnt;
    UINT8 busw;
    UINT16 code;
  } Command;

  struct {
    UINT8 byte_cnt;
    UINT8 busw;
    UINT64 val;
  } Address;

  struct {
    UINT8 byte_cnt;
    UINT8 busw;
  } DummyBytes;

  struct {
    UINT8 busw;
    UINT8 direction;
    unsigned int byte_cnt;

    union {
      VOID *in;
      const VOID *out;
    } buf;
  } Data;
} FspiCommand;

// Direction of the command
#define CMD_DIRECTION_NONE 0
#define CMD_DIRECTION_IN   1
#define CMD_DIRECTION_OUT  2

// Usefull macros to initialize the FspiCommand on one or few lines
#define FSPI_CMD_CMD(__code, __busw)            \
    {                           \
        .busw = __busw,             \
        .code = __code,             \
        .byte_cnt = 1,                    \
    }

#define FSPI_CMD_ADDR(__byte_cnt, __val, __busw)        \
    {                           \
        .byte_cnt = __byte_cnt,             \
        .val = __val,                   \
        .busw = __busw,             \
    }

#define FSPI_CMD_NO_ADDR  { }

#define FSPI_CMD_DUMMY(__byte_cnt, __busw)          \
    {                           \
        .byte_cnt = __byte_cnt,             \
        .busw = __busw,             \
    }

#define FSPI_CMD_NO_DUMMY { }

#define FSPI_CMD_DATA_IN(__byte_cnt, __buf, __busw)     \
    {                           \
        .direction = CMD_DIRECTION_IN,             \
        .byte_cnt = __byte_cnt,             \
        .buf.in = __buf,                \
        .busw = __busw,             \
    }

#define FSPI_CMD_DATA_OUT(__byte_cnt, __buf, __busw)    \
    {                           \
        .direction = CMD_DIRECTION_OUT,            \
        .byte_cnt = __byte_cnt,             \
        .buf.out = __buf,               \
        .busw = __busw,             \
    }

#define FSPI_CMD_NO_DATA  { }

#define FSPI_CMD(__Command, __Address, __DummyBytes, __Data)      \
    {                           \
        .Command = __Command,                   \
        .Address = __Address,                 \
        .DummyBytes = __DummyBytes,               \
        .Data = __Data,                 \
    }

// SPI Communication protocols
#define BITS_PER_LONG 64

#define GENMASK(h, l) \
  (((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))

#define IMX_SPI_PROTOCOL_INST_MASK    GENMASK(23, 16)
#define IMX_SPI_PROTOCOL_INST_SHIFT   16
#define IMX_SPI_PROTOCOL_INST(_nbits) \
    ((((unsigned long)(_nbits)) << IMX_SPI_PROTOCOL_INST_SHIFT) & \
     IMX_SPI_PROTOCOL_INST_MASK)

#define IMX_SPI_PROTOCOL_ADDR_MASK    GENMASK(15, 8)
#define IMX_SPI_PROTOCOL_ADDR_SHIFT   8
#define IMX_SPI_PROTOCOL_ADDR(_nbits) \
    ((((unsigned long)(_nbits)) << IMX_SPI_PROTOCOL_ADDR_SHIFT) & \
     IMX_SPI_PROTOCOL_ADDR_MASK)

#define IMX_SPI_PROTOCOL_DATA_MASK    GENMASK(7, 0)
#define IMX_SPI_PROTOCOL_DATA_SHIFT   0
#define IMX_SPI_PROTOCOL_DATA(_nbits) \
    ((((unsigned long)(_nbits)) << IMX_SPI_PROTOCOL_DATA_SHIFT) & \
     IMX_SPI_PROTOCOL_DATA_MASK)

#define IMX_SPI_PROTOCOL_STR(_inst_nbits, _addr_nbits, _data_nbits)   \
    (IMX_SPI_PROTOCOL_INST(_inst_nbits) |             \
     IMX_SPI_PROTOCOL_ADDR(_addr_nbits) |             \
     IMX_SPI_PROTOCOL_DATA(_data_nbits))

enum iMXNorProtocol {
  IMX_SPI_PROTOCOL_1_1_1 = IMX_SPI_PROTOCOL_STR(1, 1, 1),
  IMX_SPI_PROTOCOL_1_1_2 = IMX_SPI_PROTOCOL_STR(1, 1, 2),
  IMX_SPI_PROTOCOL_1_1_4 = IMX_SPI_PROTOCOL_STR(1, 1, 4),
  IMX_SPI_PROTOCOL_1_1_8 = IMX_SPI_PROTOCOL_STR(1, 1, 8),
  IMX_SPI_PROTOCOL_1_2_2 = IMX_SPI_PROTOCOL_STR(1, 2, 2),
  IMX_SPI_PROTOCOL_1_4_4 = IMX_SPI_PROTOCOL_STR(1, 4, 4),
  IMX_SPI_PROTOCOL_1_8_8 = IMX_SPI_PROTOCOL_STR(1, 8, 8),
  IMX_SPI_PROTOCOL_2_2_2 = IMX_SPI_PROTOCOL_STR(2, 2, 2),
  IMX_SPI_PROTOCOL_4_4_4 = IMX_SPI_PROTOCOL_STR(4, 4, 4),
  IMX_SPI_PROTOCOL_8_8_8 = IMX_SPI_PROTOCOL_STR(8, 8, 8),
};

static inline UINT8 spi_nor_get_protocol_inst_nbits(enum iMXNorProtocol proto) {
  return ((unsigned long) (proto & IMX_SPI_PROTOCOL_INST_MASK)) >>
  IMX_SPI_PROTOCOL_INST_SHIFT;
}

static inline UINT8 spi_nor_get_protocol_addr_nbits(enum iMXNorProtocol proto) {
  return ((unsigned long) (proto & IMX_SPI_PROTOCOL_ADDR_MASK)) >>
  IMX_SPI_PROTOCOL_ADDR_SHIFT;
}

static inline UINT8 spi_nor_get_protocol_data_nbits(enum iMXNorProtocol proto) {
  return ((unsigned long) (proto & IMX_SPI_PROTOCOL_DATA_MASK)) >>
  IMX_SPI_PROTOCOL_DATA_SHIFT;
}

static inline UINT8 spi_nor_get_protocol_width(enum iMXNorProtocol proto) {
  return spi_nor_get_protocol_data_nbits(proto);
}

#define SPINOR_OP_WRDI      0x04    /* Write disable */
#define SPINOR_OP_RDSR      0x05    /* Read status register */
#define SPINOR_OP_WREN      0x06    /* Write enable */
#define SPINOR_OP_BE_4K     0x20    /* Erase 4KiB block */
#define SPINOR_OP_CLSR      0x30    /* Clear status register 1 */
#define SPINOR_OP_PP_1_1_4  0x32    /* Quad page program */
#define SPINOR_OP_CLFSR     0x50    /* Clear flag status register */
#define SPINOR_OP_READ_1_1_4    0x6b    /* Read data bytes (Quad Output SPI) */
#define SPINOR_OP_RDFSR     0x70    /* Read flag status register */
#define SPINOR_OP_RDID      0x9f    /* Read JEDEC ID */

#define BIT(nr) (1UL << (nr))

enum spi_nor_option_flags {
  SNOR_F_USE_FSR = BIT(0),
  SNOR_F_HAS_SR_TB = BIT(1),
  SNOR_F_NO_OP_CHIP_ERASE = BIT(2),
  SNOR_F_S3AN_ADDR_DEFAULT = BIT(3),
  SNOR_F_READY_XSR_RDY = BIT(4),
  SNOR_F_USE_CLSR = BIT(5),
  SNOR_F_BROKEN_RESET = BIT(6),
  SNOR_F_SOFT_RESET = BIT(7),
};

/* Status Register bits. */
#define SR_WIP          BIT(0)  /* Write in progress */
#define SR_WEL          BIT(1)  /* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define SR_BP0          BIT(2)  /* Block protect 0 */
#define SR_BP1          BIT(3)  /* Block protect 1 */
#define SR_BP2          BIT(4)  /* Block protect 2 */
#define SR_TB           BIT(5)  /* Top/Bottom protect */
#define SR_SRWD         BIT(7)  /* SR write protect */
/* Spansion/Cypress specific status bits */
#define SR_E_ERR        BIT(5)
#define SR_P_ERR        BIT(6)

/* Flag Status Register bits */
#define FSR_READY       BIT(7)  /* Device status, 0 = Busy, 1 = Ready */
#define FSR_E_ERR       BIT(5)  /* Erase operation status */
#define FSR_P_ERR       BIT(4)  /* Program operation status */
#define FSR_PT_ERR      BIT(1)  /* Protection error bit */

#ifndef UINT_MAX
#define UINT_MAX  (~0U)
#endif

#define SZ_128      0x00000080
#define SZ_512      0x00000200
#define SZ_1K       0x00000400
#define SZ_2K       0x00000800

#endif //IMX_NOR_SPI_CMD_H_

