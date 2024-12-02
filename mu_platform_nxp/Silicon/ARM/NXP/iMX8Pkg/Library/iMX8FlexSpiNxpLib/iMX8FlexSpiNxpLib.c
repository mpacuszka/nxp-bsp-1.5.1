/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Library/BaseLib.h>
#include <Library/TimerLib.h>
#include "iMXNorCmd.h"
#include "iMX8FlexSpiLib.h"


#define FSPI_SEQID_LUT              15
#define FSPI_SEQID_AHB_LUT          14
//
#define IMX_FSPI_MCR0               0x00
#define IMX_FSPI_MCR0_SWRST         BIT(0)

#define IMX_FSPI_INTR               0x14
#define IMX_FSPI_INTR_IPTXWE        BIT(6)
#define IMX_FSPI_INTR_IPRXWA        BIT(5)
//
#define IMX_FSPI_LUTKEY             0x18
#define IMX_FSPI_LUTKEY_VALUE       0x5AF05AF0
//
#define IMX_FSPI_LCKCR              0x1C
//
#define IMX_FSPI_LCKER_LOCK         0x1
#define IMX_FSPI_LCKER_UNLOCK       0x2

#define IMX_FSPI_IPCR0              0xA0
#define IMX_FSPI_IPCR1              0xA4
#define IMX_FSPI_IPCR1_SEQNUM_SHIFT 24
#define IMX_FSPI_IPCR1_SEQID_SHIFT  16

#define IMX_FSPI_IPCMD              0xB0
#define IMX_FSPI_IPCMD_TRG          BIT(0)

#define IMX_FSPI_IPRXFCR            0xB8
#define IMX_FSPI_IPRXFCR_CLR        BIT(0)
#define IMX_FSPI_IPRXFCR_DMA_EN     BIT(1)
//
#define IMX_FSPI_IPTXFCR            0xBC
#define IMX_FSPI_IPTXFCR_CLR        BIT(0)

#define IMX_FSPI_STS0               0xE0
#define IMX_FSPI_STS0_ARB_IDLE      BIT(1)
#define IMX_FSPI_RFDR               0x100
#define IMX_FSPI_TFDR               0x180
//
#define IMX_FSPI_LOOKUPTABLE_BASE           0x200
#define IMX_FSPI_LOOKUPTABLE_OFFSET         (FSPI_SEQID_LUT * 4 * 4)
#define IMX_FSPI_LOOKUPTABLE_REG(idx) \
    (IMX_FSPI_LOOKUPTABLE_BASE + IMX_FSPI_LOOKUPTABLE_OFFSET + (idx) * 4)
//
#define IMX_FSPI_AHB_LOOKUPTABLE_OFFSET            (FSPI_SEQID_AHB_LUT * 4 * 4)
#define IMX_FSPI_AHB_LOOKUPTABLE_REG(idx) \
    (IMX_FSPI_LOOKUPTABLE_BASE + IMX_FSPI_AHB_LOOKUPTABLE_OFFSET + (idx) * 4)
//
//
#define LOOKUPTABLE_STOP            0x00
#define LOOKUPTABLE_CMD             0x01
#define LOOKUPTABLE_ADDR            0x02
#define LOOKUPTABLE_NXP_WRITE            0x08
#define LOOKUPTABLE_NXP_READ            0x09
#define LOOKUPTABLE_DUMMY            0x0C

#define LOOKUPTABLE_PAD(x) (FindMostSignBit(x) - 1)

#define PAD_SHIFT           8
#define INSTR_SHIFT        10
#define OPRND_SHIFT        16

#define LOOKUPTABLE_DEF(idx, ins, pad, opr)              \
    ((((ins) << INSTR_SHIFT) | ((pad) << PAD_SHIFT) | \
    (opr)) << (((idx) % 2) * OPRND_SHIFT))

#define POLL_TOUT        5000
#define NXP_IMX_FSPI_MAX_CHIPSELECT        4

struct nxp_fspi_devtype_data imx8mm_data = { .rxfifo = SZ_512, /* (64  * 64 bits)  */
.txfifo = SZ_1K, /* (128 * 64 bits)  */
.ahb_buf_size = SZ_2K, /* (256 * 64 bits)  */
};

static struct FspiData g_nxp_fspi;

#define out(val, addr) out_le32((volatile unsigned*)(addr), (int)(val))
#define in(addr)        in_le32((volatile unsigned*)(addr))

struct FspiData* GetFspiData() {
  g_nxp_fspi.devtype_data = &imx8mm_data;
  g_nxp_fspi.memmap_phy_size = 0x10000;
  return &g_nxp_fspi;
}

static inline int FindMostSignBit(int n) {
  int mask = 0x80000000;
  int r = 32;

  do {
    if (n & mask)
      return r;
    r--;
    mask >>= 1;
  } while (r > 0);

  return r;
}

static int FspiReadlPollTout(struct FspiData *f, VOID *base, UINT32 mask,
    UINT32 delay_us, UINT32 timeout_us, BOOLEAN c) {
  UINT32 reg;

  if (c)
    return ReadlPollSleepTimeout(base, reg, (reg & mask), delay_us, timeout_us);
  else
    return ReadlPollSleepTimeout(base, reg, !(reg & mask), delay_us, timeout_us);
}

static inline VOID FspiInvalid(struct FspiData *f) {
  UINT32 reg;
  int ret;

  reg = in(f->iobase + IMX_FSPI_MCR0);
  out(reg | IMX_FSPI_MCR0_SWRST, f->iobase + IMX_FSPI_MCR0);

  /* w1c register, wait unit clear */
  ret = FspiReadlPollTout(f, f->iobase + IMX_FSPI_MCR0,
  IMX_FSPI_MCR0_SWRST, 0, POLL_TOUT, FALSE);
  WARN_ON(ret);
}

static VOID FspiPrepareLut(struct FspiData *f, const FspiCommand *cmd) {
  VOID *base = f->iobase;
  UINT32 lutval[4] = { };
  int lutidx = 1, i;

  lutval[0] |= LOOKUPTABLE_DEF(0, LOOKUPTABLE_CMD,
      LOOKUPTABLE_PAD(cmd->Command.busw), cmd->Command.code);

  if (cmd->Address.byte_cnt) {
    lutval[lutidx / 2] |= LOOKUPTABLE_DEF(lutidx, LOOKUPTABLE_ADDR,
        LOOKUPTABLE_PAD(cmd->Address.busw), cmd->Address.byte_cnt * 8);
    lutidx++;
  }

  if (cmd->DummyBytes.byte_cnt) {
    lutval[lutidx / 2] |= LOOKUPTABLE_DEF(lutidx, LOOKUPTABLE_DUMMY,
        LOOKUPTABLE_PAD(cmd->Data.busw),
        cmd->DummyBytes.byte_cnt * 8 / cmd->DummyBytes.busw);
    lutidx++;
  }

  if (cmd->Data.byte_cnt) {
    lutval[lutidx / 2] |=
        LOOKUPTABLE_DEF(lutidx,
            cmd->Data.direction == CMD_DIRECTION_IN ? (LOOKUPTABLE_NXP_READ) : (LOOKUPTABLE_NXP_WRITE),
            LOOKUPTABLE_PAD(cmd->Data.busw), 0);
    lutidx++;
  }

  lutval[lutidx / 2] |= LOOKUPTABLE_DEF(lutidx, LOOKUPTABLE_STOP, 0, 0);

  out(IMX_FSPI_LUTKEY_VALUE, f->iobase + IMX_FSPI_LUTKEY);
  out(IMX_FSPI_LCKER_UNLOCK, f->iobase + IMX_FSPI_LCKCR);

  for (i = 0; i < ARRAY_SIZE(lutval); i++)
    out(lutval[i], base + IMX_FSPI_LOOKUPTABLE_REG(i));

  if (cmd->Data.byte_cnt && cmd->Data.direction == CMD_DIRECTION_IN
      && cmd->Address.byte_cnt) {
    for (i = 0; i < ARRAY_SIZE(lutval); i++)
      out(lutval[i], base + IMX_FSPI_AHB_LOOKUPTABLE_REG(i));
  }

//    DEBUG((DEBUG_ERROR, "CMD[%x] lutval[0:%x \t 1:%x \t 2:%x \t 3:%x], size: 0x%08x\n",
//        cmd->cmd.opcode, lutval[0], lutval[1], lutval[2], lutval[3], cmd->data.byte_cnt));

  /* lock LUT */
  out(IMX_FSPI_LUTKEY_VALUE, f->iobase + IMX_FSPI_LUTKEY);
  out(IMX_FSPI_LCKER_LOCK, f->iobase + IMX_FSPI_LCKCR);
}

static VOID FspiReadAhb(struct FspiData *f, const FspiCommand *cmd) {
  UINT32 len = cmd->Data.byte_cnt;

  /* Read out the data directly from the AHB buffer. */
  CopyMem(cmd->Data.buf.in, (f->ahb_addr + cmd->Address.val), len);
}

static VOID FspiFillTxfifo(struct FspiData *f, const FspiCommand *cmd) {
  VOID *base = f->iobase;
  int i, ret;
  UINT8 *buf = (UINT8*) cmd->Data.buf.out;

  out(IMX_FSPI_IPTXFCR_CLR, base + IMX_FSPI_IPTXFCR);

  for (i = 0; i < ALIGN_DOWN(cmd->Data.byte_cnt, 8); i += 8) {
    ret = FspiReadlPollTout(f, f->iobase + IMX_FSPI_INTR,
    IMX_FSPI_INTR_IPTXWE, 0,
    POLL_TOUT, TRUE);
    WARN_ON(ret);

    out(*(UINT32* ) (buf + i), base + IMX_FSPI_TFDR);
    out(*(UINT32* ) (buf + i + 4), base + IMX_FSPI_TFDR + 4);
    out(IMX_FSPI_INTR_IPTXWE, base + IMX_FSPI_INTR);
  }

  if (i < cmd->Data.byte_cnt) {
    UINT32 data = 0;
    int j;
    ret = FspiReadlPollTout(f, f->iobase + IMX_FSPI_INTR,
    IMX_FSPI_INTR_IPTXWE, 0,
    POLL_TOUT, TRUE);
    WARN_ON(ret);

    for (j = 0; j < ALIGN(cmd->Data.byte_cnt - i, 4); j += 4) {
      CopyMem(&data, buf + i + j, 4);
      out(data, base + IMX_FSPI_TFDR + j);
    }
    out(IMX_FSPI_INTR_IPTXWE, base + IMX_FSPI_INTR);
  }
}

static VOID FspiReadRxfifo(struct FspiData *f, const FspiCommand *cmd) {
  VOID *base = f->iobase;
  int i, ret;
  int len, cnt;
  UINT8 *buf = (UINT8*) cmd->Data.buf.in;

  len =
      (f->flags & IMX_FSPI_DTR_ODD_ADDR) ?
          cmd->Data.byte_cnt + 1 : cmd->Data.byte_cnt;

  if (f->flags & IMX_FSPI_DTR_ODD_ADDR) {
    UINT8 tmp[8];
    ret = FspiReadlPollTout(f, f->iobase + IMX_FSPI_INTR,
    IMX_FSPI_INTR_IPRXWA, 0,
    POLL_TOUT, TRUE);
    WARN_ON(ret);

    *(UINT32*) tmp = in(base + IMX_FSPI_RFDR);
    *(UINT32*) (tmp + 4) = in(base + IMX_FSPI_RFDR + 4);
    cnt = len < 8 ? len : 8;
    CopyMem(buf, tmp + 1, cnt - 1);
    len -= cnt;
    buf = cmd->Data.buf.in + cnt - 1;
    f->flags &= ~IMX_FSPI_DTR_ODD_ADDR;

    out(IMX_FSPI_INTR_IPRXWA, base + IMX_FSPI_INTR);
  }

  cnt = ALIGN_DOWN(len, 8);

  for (i = 0; i < cnt;) {
    ret = FspiReadlPollTout(f, f->iobase + IMX_FSPI_INTR,
    IMX_FSPI_INTR_IPRXWA, 0,
    POLL_TOUT, TRUE);
    WARN_ON(ret);

    *(UINT32*) (buf + i) = in(base + IMX_FSPI_RFDR);
    *(UINT32*) (buf + i + 4) = in(base + IMX_FSPI_RFDR + 4);
    i += 8;
    /* move the FIFO pointer */
    out(IMX_FSPI_INTR_IPRXWA, base + IMX_FSPI_INTR);
  }

  if (i < len) {
    UINT32 tmp;
    int size, j;

    buf += i;
    len -= i;
    ret = FspiReadlPollTout(f, f->iobase + IMX_FSPI_INTR,
    IMX_FSPI_INTR_IPRXWA, 0,
    POLL_TOUT, TRUE);
    WARN_ON(ret);

    for (j = 0; j < cmd->Data.byte_cnt - i; j += 4) {
      tmp = in(base + IMX_FSPI_RFDR + j);
      size = len < 4 ? len : 4;
      CopyMem(buf + j, &tmp, size);
      len -= size;
    }
  }

  out(IMX_FSPI_IPRXFCR_CLR, base + IMX_FSPI_IPRXFCR);
  out(IMX_FSPI_INTR_IPRXWA, base + IMX_FSPI_INTR);
}

static int FspiDoCommand(struct FspiData *f, const FspiCommand *cmd) {
  VOID *base = f->iobase;
  int seqnum = 0;
  int err = 0;
  UINT32 reg;

  reg = in(base + IMX_FSPI_IPRXFCR);
  reg &= ~IMX_FSPI_IPRXFCR_DMA_EN;
  reg = reg | IMX_FSPI_IPRXFCR_CLR;
  out(reg, base + IMX_FSPI_IPRXFCR);

  out(cmd->Address.val, base + IMX_FSPI_IPCR0);

  if (f->flags & IMX_FSPI_DTR_ODD_ADDR)
    out(
        (cmd->Data.byte_cnt + 1) | (FSPI_SEQID_LUT << IMX_FSPI_IPCR1_SEQID_SHIFT) | (seqnum << IMX_FSPI_IPCR1_SEQNUM_SHIFT),
        base + IMX_FSPI_IPCR1);
  else
    out(
        cmd->Data.byte_cnt | (FSPI_SEQID_LUT << IMX_FSPI_IPCR1_SEQID_SHIFT) | (seqnum << IMX_FSPI_IPCR1_SEQNUM_SHIFT),
        base + IMX_FSPI_IPCR1);

  out(IMX_FSPI_IPCMD_TRG, base + IMX_FSPI_IPCMD);

  err = FspiReadlPollTout(f, f->iobase + IMX_FSPI_STS0,
  IMX_FSPI_STS0_ARB_IDLE, 1, 1000 * 1000, TRUE);

  if (!err && cmd->Data.byte_cnt && cmd->Data.direction == CMD_DIRECTION_IN)
    FspiReadRxfifo(f, cmd);

  return err;
}

int FspiRunCommand(const FspiCommand *cmd) {
  struct FspiData *f;
  int err = 0;

  f = GetFspiData();

  DebugPrint(0xffffffff, "BEFORE POLL, ptr=0x%08x\n", f->iobase);
  DebugPrint(0xffffffff, "BEFORE POLL, ptr=0x%08x VAL=0x%08X\n", f->iobase, *(UINT32*)(f->iobase));
  DebugPrint(0xffffffff, "BEFORE POLL, ptr=0x%08x VAL2=0x%08X\n", f->iobase, *(UINT32*)(f->iobase +  IMX_FSPI_STS0));
  err = FspiReadlPollTout(f, f->iobase + IMX_FSPI_STS0,
  IMX_FSPI_STS0_ARB_IDLE, 1, POLL_TOUT, TRUE);
  DebugPrint(0xffffffff, "AFTER POLL, ptr=0x%08x\n", f->iobase);
  WARN_ON(err);

  FspiPrepareLut(f, cmd);

  if (cmd->Data.byte_cnt
      > (f->devtype_data->rxfifo - 4)&& cmd->Data.direction == CMD_DIRECTION_IN) {
    FspiReadAhb(f, cmd);
  } else {
    if (cmd->Data.byte_cnt && cmd->Data.direction == CMD_DIRECTION_OUT)
      FspiFillTxfifo(f, cmd);

    err = FspiDoCommand(f, cmd);
  }

  FspiInvalid(f);

  return err;
}

int FspiAdjustCommandSize(FspiCommand *cmd) {
  struct FspiData *f;
  f = GetFspiData();

  if (cmd->Data.direction == CMD_DIRECTION_OUT) {
    if (cmd->Data.byte_cnt > f->devtype_data->txfifo)
      cmd->Data.byte_cnt = f->devtype_data->txfifo;
  } else {

    if (cmd->Data.byte_cnt > f->devtype_data->ahb_buf_size)
      cmd->Data.byte_cnt = f->devtype_data->ahb_buf_size;
    else if (cmd->Data.byte_cnt > (f->devtype_data->rxfifo - 4))
      cmd->Data.byte_cnt = ALIGN_DOWN(cmd->Data.byte_cnt, 8);
  }

  return 0;
}

UINT64 FspiTimerGetUsec() {
  UINT64 ticks = GetPerformanceCounter();
  return GetTimeInNanoSecond(ticks) / 1000;
}
