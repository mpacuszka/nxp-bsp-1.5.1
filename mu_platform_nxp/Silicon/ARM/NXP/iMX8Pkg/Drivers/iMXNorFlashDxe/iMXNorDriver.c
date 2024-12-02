/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiRuntimeLib.h>

#include "iMXNorDriver.h"

#include "iMX8FlexSpiLib.h"

#define SUPPORTED_SECTOR_4K            BIT(0)
#define SUPPORTED_DUAL_READ            BIT(5)
#define SUPPORTED_QUAD_READ            BIT(6)
#define SUPPORTED_FLAG_STATUS_REGISTER BIT(7)

#define FLASH_DATA(xName, xJedecId, xExtId, xSectorSize, xNumberOfSectors, xFlags) \
    .Name = xName,      \
    .Id = {             \
      ((xJedecId) >> 16) & 0xff,     \
      ((xJedecId) >> 8) & 0xff,      \
      (xJedecId) & 0xff,       \
      ((xExtId) >> 8) & 0xff,      \
      (xExtId) & 0xff,       \
      },            \
    .IdLen = (!(xJedecId) ? 0 : (3 + ((xExtId) ? 2 : 0))), \
    .SectorSize = (xSectorSize),          \
    .NumberOfSectors = (xNumberOfSectors), \
    .PageSize = 256,         \
    .Flags = (xFlags),

#define FLASH_ID_MAXLEN 6

struct FlashID {
  char *Name;

  // The first three bytes are the JEDIC ID
  UINT8 Id[FLASH_ID_MAXLEN];
  UINT8 IdLen;

  UINT32 SectorSize;
  UINT16 NumberOfSectors;

  UINT16 PageSize;
  UINT16 AddrWidth;

  UINT32 Flags;
};

struct iMXNorSetup {
  UINT8 ReadOpcode;
  UINT8 EraseOpcode;
  UINT8 ProgramOpcode;
  UINT8 ReadDummy;
  UINT16 AddrWidth;
  UINT16 EraseSize;
  UINT16 PageSize;

  struct FlashID FlashId;
};

struct iMXNorSetup NorSetup;

const struct FlashID FlashIDs[] =
    {
        /* Micron */
        {
            FLASH_DATA("n25q256a",   0x20ba19, 0, 65536, 512, SUPPORTED_SECTOR_4K | SUPPORTED_DUAL_READ | SUPPORTED_QUAD_READ | SUPPORTED_FLAG_STATUS_REGISTER)
        },
        {
            FLASH_DATA("n25q256ax1", 0x20bb19, 0, 65536, 512, SUPPORTED_SECTOR_4K | SUPPORTED_QUAD_READ | SUPPORTED_FLAG_STATUS_REGISTER)
        },
        /* Winbond */
        {
            FLASH_DATA("w25q64dw",   0xef6017, 0, 64 * 1024, 128, SUPPORTED_SECTOR_4K)
        },
        { }, };

static const struct FlashID* iMXNorReadID();

BOOLEAN iMXFindDevice() {

  const struct FlashID *id = iMXNorReadID();

  if (id != NULL) {
    NorSetup.FlashId = *id;
    NorSetup.ReadOpcode = SPINOR_OP_READ_1_1_4;
    NorSetup.EraseOpcode = SPINOR_OP_BE_4K;
    NorSetup.ProgramOpcode = SPINOR_OP_PP_1_1_4;
    NorSetup.AddrWidth = 3;
    NorSetup.ReadDummy = 8;
    NorSetup.EraseSize = 4096;
    NorSetup.PageSize = 256;
  }

  return (id != NULL);
}

static void iMXNorSetupOp(FspiCommand *cmd, const enum iMXNorProtocol proto) {

  cmd->Command.busw = spi_nor_get_protocol_inst_nbits(proto);

  if (cmd->Address.byte_cnt)
    cmd->Address.busw = spi_nor_get_protocol_addr_nbits(proto);

  if (cmd->DummyBytes.byte_cnt)
    cmd->DummyBytes.busw = spi_nor_get_protocol_addr_nbits(proto);

  if (cmd->Data.byte_cnt)
    cmd->Data.busw = spi_nor_get_protocol_data_nbits(proto);
}

static int iMXNorReadWriteRegister(FspiCommand *cmd, VOID *buffer) {
  if (cmd->Data.direction == CMD_DIRECTION_IN)
    cmd->Data.buf.in = buffer;
  else
    cmd->Data.buf.out = buffer;

  return FspiRunCommand(cmd);
}

static int iMXNorReadRegister(UINT8 code, UINT8 *val, int len) {
  FspiCommand cmd = FSPI_CMD(FSPI_CMD_CMD(code, 0),
      FSPI_CMD_NO_ADDR,
      FSPI_CMD_NO_DUMMY,
      FSPI_CMD_DATA_IN(len, NULL, 0));
  int ret;

  iMXNorSetupOp(&cmd, IMX_SPI_PROTOCOL_1_1_1);

  ret = iMXNorReadWriteRegister(&cmd, val);

  return ret;
}

static int iMXNorWriteRegister(UINT8 opcode, UINT8 *buf, int len) {
  FspiCommand cmd = FSPI_CMD(FSPI_CMD_CMD(opcode, 0),
      FSPI_CMD_NO_ADDR,
      FSPI_CMD_NO_DUMMY,
      FSPI_CMD_DATA_OUT(len, NULL, 0));

  iMXNorSetupOp(&cmd, IMX_SPI_PROTOCOL_1_1_1);

  if (len == 0)
    cmd.Data.direction = CMD_DIRECTION_NONE;

  return iMXNorReadWriteRegister(&cmd, buf);
}

static const struct FlashID* iMXNorReadID() {
  int tmp;
  UINT8 id[FLASH_ID_MAXLEN * 2];
  const struct FlashID *flashId;

  tmp = iMXNorReadRegister(SPINOR_OP_RDID, id, FLASH_ID_MAXLEN);
  if (tmp < 0) {
    DEBUG((DEBUG_ERROR, "Failed to read flash DeviceID, ret=%d\n", tmp));
    return NULL;
  }

  flashId = FlashIDs;
  for (; flashId->Name; flashId++) {
    if (flashId->IdLen) {
      if (!CompareMem(flashId->Id, id, flashId->IdLen)) {
        DEBUG((DEBUG_INFO, "iMX FLASH ID: %s\n", flashId->Name));
        return flashId;
      }
    }
  }

  DEBUG(
      (DEBUG_ERROR, "Unknown NorFlash Device, ID= 0x%02x 0x%02x 0x%02x\n", id[0], id[1], id[2]));

  return NULL;
}

static UINT32 iMXNorReadData(INT64 from, UINT32 len, UINT8 *buf) {
  FspiCommand cmd =
  FSPI_CMD(FSPI_CMD_CMD(NorSetup.ReadOpcode, 0),
      FSPI_CMD_ADDR(NorSetup.AddrWidth, from, 0),
      FSPI_CMD_DUMMY(NorSetup.ReadDummy, 0),
      FSPI_CMD_DATA_IN(len, buf, 0));
  UINT32 remaining = len;
  int ret;

  iMXNorSetupOp(&cmd, IMX_SPI_PROTOCOL_1_1_4);

  /* convert the dummy cycles to the number of bytes */
  cmd.DummyBytes.byte_cnt = (NorSetup.ReadDummy * cmd.DummyBytes.busw) / 8;

  while (remaining) {
    cmd.Data.byte_cnt = remaining < UINT_MAX ? remaining : UINT_MAX;
    ret = FspiAdjustCommandSize(&cmd);
    if (ret)
      return ret;

    ret = FspiRunCommand(&cmd);
    if (ret)
      return ret;

    cmd.Address.val += cmd.Data.byte_cnt;
    remaining -= cmd.Data.byte_cnt;
    cmd.Data.buf.in += cmd.Data.byte_cnt;
  }

  return len;
}

int iMXNorRead(UINT64 from, UINT32 len, UINT32 *retlen, UINT8 *buf) {
  int ret;

  while (len) {
    INT64 addr = from;
    UINT32 read_len = len;

    ret = iMXNorReadData(addr, read_len, buf);
    if (ret == 0) {
      ret = -1;
      goto read_err;
    }
    if (ret < 0)
      goto read_err;

    *retlen += ret;
    buf += ret;
    from += ret;
    len -= ret;
  }
  ret = 0;

  read_err: return ret;
}

static int iMXNorReadSr() {
  FspiCommand cmd;
  int ret;
  UINT8 val[2];
  UINT8 addr_byte_cnt, dummy;

  addr_byte_cnt = 0;
  dummy = 0;

  cmd = (FspiCommand )FSPI_CMD(FSPI_CMD_CMD(SPINOR_OP_RDSR, 0),
          FSPI_CMD_ADDR(addr_byte_cnt, 0, 0),
          FSPI_CMD_DUMMY(dummy, 0),
          FSPI_CMD_DATA_IN(1, NULL, 0));

  iMXNorSetupOp(&cmd, IMX_SPI_PROTOCOL_1_1_1);

  ret = iMXNorReadWriteRegister(&cmd, val);
  if (ret < 0) {
    DEBUG((DEBUG_ERROR, "Failed to read StatusRegister, res = %d\n", (int)ret));
    return ret;
  }

  return *val;
}

static int iMXNorReadFsr() {
  FspiCommand cmd;
  int ret;
  UINT8 val[2];
  UINT8 addr_byte_cnt, dummy;

  addr_byte_cnt = 0;
  dummy = 0;

  cmd = (FspiCommand )FSPI_CMD(FSPI_CMD_CMD(SPINOR_OP_RDFSR, 0),
          FSPI_CMD_ADDR(addr_byte_cnt, 0, 0),
          FSPI_CMD_DUMMY(dummy, 0),
          FSPI_CMD_DATA_IN(1, NULL, 0));

  iMXNorSetupOp(&cmd, IMX_SPI_PROTOCOL_1_1_1);

  ret = iMXNorReadWriteRegister(&cmd, val);
  if (ret < 0) {
    DEBUG((DEBUG_ERROR, "error %d reading FSR\n", ret));
    return ret;
  }

  return *val;
}

static int iMXNorSrReady() {
  int sr = iMXNorReadSr();

  if (sr < 0)
    return sr;

  if ((NorSetup.FlashId.Flags & SNOR_F_USE_CLSR)
      && (sr & (SR_E_ERR | SR_P_ERR))) {
    if (sr & SR_E_ERR)
      DEBUG((DEBUG_ERROR, "Erase Error occurred\n"));
    else
      DEBUG((DEBUG_ERROR, "Programming Error occurred\n"));

    iMXNorWriteRegister(SPINOR_OP_CLSR, NULL, 0);
    return -1;
  }

  return !(sr & SR_WIP);
}

static int iMXNorFsrReady() {
  int fsr = iMXNorReadFsr();

  if (fsr < 0)
    return fsr;

  if (fsr & (FSR_E_ERR | FSR_P_ERR)) {
    if (fsr & FSR_E_ERR)
      DEBUG((DEBUG_ERROR,"Erase operation failed.\n"));
    else
      DEBUG((DEBUG_ERROR,"Program operation failed.\n"));

    if (fsr & FSR_PT_ERR)
      DEBUG((DEBUG_ERROR, "Attempted to modify a protected sector.\n"));

    iMXNorWriteRegister(SPINOR_OP_CLFSR, NULL, 0);
    return -1;
  }

  return fsr & FSR_READY;
}

static int iMXNorReady() {
  int sr, fsr;

  sr = iMXNorSrReady();
  if (sr < 0)
    return sr;
  fsr =
      (NorSetup.FlashId.Flags & SUPPORTED_FLAG_STATUS_REGISTER) ?
          iMXNorFsrReady() : 1;
  if (fsr < 0)
    return fsr;
  return sr && fsr;
}

// Max wait 40sec (e.g. some erase operations can take long
#define DEFAULT_WAIT 40000

static UINT32 iMXNorGetTimer(UINT32 base) {
  UINT64 msec = GetTimeInNanoSecond(GetPerformanceCounter()) / 1000000;

  return (UINT32) msec - base;
}

static int iMXNorWaitForReady() {
  UINT32 timebase;
  int ret;

  UINT32 timeout = DEFAULT_WAIT;

  timebase = iMXNorGetTimer(0);

  while (iMXNorGetTimer(timebase) < timeout) {
    ret = iMXNorReady();
    if (ret < 0)
      return ret;
    if (ret)
      return 0;
  }

  DEBUG((DEBUG_ERROR,"flash operation timed out\n"));
  return -1;

}

static int iMXNorEnableWrite() {
  return iMXNorWriteRegister(SPINOR_OP_WREN, NULL, 0);
}

static int iMXNorDisableWrite() {
  return iMXNorWriteRegister(SPINOR_OP_WRDI, NULL, 0);
}

static int iMXNorEraseSector(UINT32 addr) {
  FspiCommand cmd =
  FSPI_CMD(FSPI_CMD_CMD(NorSetup.EraseOpcode, 0),
      FSPI_CMD_ADDR(NorSetup.AddrWidth, addr, 0),
      FSPI_CMD_NO_DUMMY,
      FSPI_CMD_NO_DATA);
  int ret;

  iMXNorSetupOp(&cmd, IMX_SPI_PROTOCOL_1_1_4);

  ret = FspiRunCommand(&cmd);
  if (ret)
    return ret;

  return NorSetup.EraseSize;
}

int iMXNorErase(UINT32 addr, UINT32 len) {
  int ret, err;

  while (len) {
    ret = iMXNorEnableWrite();
    if (ret < 0)
      goto erase_err;

    ret = iMXNorEraseSector(addr);
    if (ret < 0)
      goto erase_err;

    addr += ret;
    len -= ret;

    ret = iMXNorWaitForReady();
    if (ret)
      goto erase_err;
  }

  erase_err: err = iMXNorDisableWrite();
  if (!ret)
    ret = err;

  return ret;
}

//=== WRITE ===========================================
static UINT32 iMXNorWriteData(INT64 to, UINT32 len, const UINT8 *buf) {
  FspiCommand cmd =
  FSPI_CMD(FSPI_CMD_CMD(NorSetup.ProgramOpcode, 0),
      FSPI_CMD_ADDR(NorSetup.AddrWidth, to, 0),
      FSPI_CMD_NO_DUMMY,
      FSPI_CMD_DATA_OUT(len, buf, 0));
  int ret;

  iMXNorSetupOp(&cmd, IMX_SPI_PROTOCOL_1_1_4);

  ret = FspiAdjustCommandSize(&cmd);
  if (ret)
    return ret;
  cmd.Data.byte_cnt = len < cmd.Data.byte_cnt ? len : cmd.Data.byte_cnt;

  ret = FspiRunCommand(&cmd);
  if (ret)
    return ret;

  return cmd.Data.byte_cnt;
}

int iMXNorWrite(UINT64 to, UINT32 len, UINT32 *retlen, const UINT8 *buf) {
  UINT32 page_offset, page_remain, i;
  UINT32 ret;

  for (i = 0; i < len;) {
    UINT32 written;
    INT64 addr = to + i;

    page_offset = addr & (NorSetup.PageSize - 1);

    /* the size of data remaining on the first page */
    page_remain =
        NorSetup.PageSize - page_offset < len - i ?
            NorSetup.PageSize - page_offset : len - i;

    iMXNorEnableWrite();
    ret = iMXNorWriteData(addr, page_remain, buf + i);
    if (ret < 0)
      goto write_err;
    written = ret;

    ret = iMXNorWaitForReady();
    if (ret)
      goto write_err;
    *retlen += written;
    i += written;
  }

  write_err: return ret;
}
