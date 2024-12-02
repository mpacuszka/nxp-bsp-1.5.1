/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef IMX_FLEX_SPI_LIB_H_
#define IMX_FLEX_SPI_LIB_H_

#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/TimerLib.h>

#include "iMXNorCmd.h"

#define IS_ALIGNED(x, a)    (((x) & ((typeof(x))(a) - 1)) == 0)

static inline unsigned in_le32(volatile UINT32 *addr) {
  return (*addr);
}

static inline void out_le32(volatile UINT32 *addr, int val) {
  *addr = val;
}

#define WARN_ON(condition) ({           \
  int __ret_warn_on = !!(condition);        \
  if (__ret_warn_on) {         \
      DebugPrint(0xffffffff, "WARNING at %s:%d/%s()!\n", __FILE__, __LINE__, __func__); \
  }          \
})

UINT64 FspiTimerGetUsec();

#define time_after(a,b) (((b) - (a)) < 0)

#define ReadPollTimeout(op, addr, val, cond, sleep_us, timeout_us)  \
({ \
  unsigned long timeout = FspiTimerGetUsec() + timeout_us; \
  for (;;) { \
    (val) = op((volatile UINT32 *)addr); \
    if (cond) \
      break; \
    if (timeout_us && time_after(FspiTimerGetUsec(), timeout)) { \
      (val) = op((volatile UINT32 *)addr); \
      break; \
    } \
    if (sleep_us) \
        MicroSecondDelay(sleep_us); \
  } \
  (cond) ? 0 : -1; \
})

#define ReadxPollSleepTimeout(op, addr, val, cond, sleep_us, timeout_us) \
    ReadPollTimeout(op, addr, val, cond, sleep_us, timeout_us)

#define ReadlPollSleepTimeout(addr, val, cond, sleep_us, timeout_us) \
  ReadxPollSleepTimeout(in_le32, addr, val, cond, sleep_us, timeout_us)

#define ALIGN(x,a)    __ALIGN_MASK((x),(typeof(x))(a)-1)
#define ALIGN_DOWN(x, a)  ALIGN((x) - ((a) - 1), (a))
#define __ALIGN_MASK(x,mask)  (((x)+(mask))&~(mask))
#define PTR_ALIGN(p, a)   ((typeof(p))ALIGN((unsigned long)(p), (a)))
#define IS_ALIGNED(x, a)    (((x) & ((typeof(x))(a) - 1)) == 0)

struct nxp_fspi_devtype_data {
  UINT32 rxfifo;
  UINT32 txfifo;
  UINT32 ahb_buf_size;
  UINT32 quirks;
};

struct FspiData {
  VOID *iobase;
  VOID *ahb_addr;
  UINT32 memmap_phy;
  UINT32 memmap_phy_size;
  UINT32 dll_slvdly;
  struct nxp_fspi_devtype_data *devtype_data;
#define IMX_FSPI_DTR_ODD_ADDR       (1 << 0)
  int flags;
  int selected;
};

struct FspiData* GetFspiData();
int FspiRunCommand(const FspiCommand *cmd);
int FspiAdjustCommandSize(FspiCommand *cmd);

#define IMX_FLEXSPI_BASE_ADDR           0x30bb0000

#endif //IMX_FLEX_SPI_LIB_H_

