/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Library/BaseLib.h>
#include <Library/TimerLib.h>
#include "iMXNorCmd.h"
#include "iMX8FlexSpiLib.h"

#define	FSPI_SEQID_LOOKUPTABLE			15
#define	FSPI_SEQID_LOOKUPTABLE_AHB		14

#define IMX_FSPI_MCR			0x00
#define IMX_FSPI_MCR_RESERVED_MASK	GENMASK(19, 16)
#define IMX_FSPI_MCR_MDIS_MASK		BIT(14)
#define IMX_FSPI_MCR_CLR_TXF_MASK	BIT(11)
#define IMX_FSPI_MCR_CLR_RXF_MASK	BIT(10)
#define IMX_FSPI_MCR_DDR_EN_MASK		BIT(7)
#define IMX_FSPI_MCR_END_CFG_MASK	GENMASK(3, 2)
#define IMX_FSPI_MCR_SWRSTHD_MASK	BIT(1)
#define IMX_FSPI_MCR_SWRSTSD_MASK	BIT(0)

#define IMX_FSPI_IPCR			0x08
#define IMX_FSPI_IPCR_SEQID(x)		((x) << 24)
#define IMX_FSPI_FLSHCR			0x0c
#define IMX_FSPI_FLSHCR_TCSS_MASK	GENMASK(3, 0)
#define IMX_FSPI_FLSHCR_TCSH_MASK	GENMASK(11, 8)
#define IMX_FSPI_FLSHCR_TDH_MASK		GENMASK(17, 16)

#define IMX_FSPI_BUF3CR			0x1c
#define IMX_FSPI_BUF3CR_ALLMST_MASK	BIT(31)
#define IMX_FSPI_BUF3CR_ADATSZ(x)	((x) << 8)
#define IMX_FSPI_BUF3CR_ADATSZ_MASK	GENMASK(15, 8)

#define IMX_FSPI_BFGENCR			0x20
#define IMX_FSPI_BFGENCR_SEQID(x)	((x) << 12)

#define IMX_FSPI_BUF0IND			0x30
#define IMX_FSPI_BUF1IND			0x34
#define IMX_FSPI_BUF2IND			0x38
#define IMX_FSPI_SFAR			0x100

#define IMX_FSPI_SMPR			0x108
#define IMX_FSPI_SMPR_DDRSMP_MASK	GENMASK(18, 16)
#define IMX_FSPI_SMPR_FSDLY_MASK		BIT(6)
#define IMX_FSPI_SMPR_FSPHS_MASK		BIT(5)
#define IMX_FSPI_SMPR_HSENA_MASK		BIT(0)

#define IMX_FSPI_RBCT			0x110
#define IMX_FSPI_RBCT_WMRK_MASK		GENMASK(4, 0)
#define IMX_FSPI_RBCT_RXBRD_USEIPS	BIT(8)

#define IMX_FSPI_TBDR			0x154

#define IMX_FSPI_SR			0x15c
#define IMX_FSPI_SR_IP_ACC_MASK		BIT(1)
#define IMX_FSPI_SR_AHB_ACC_MASK		BIT(2)

#define IMX_FSPI_FR			0x160
#define IMX_FSPI_FR_TFF_MASK		BIT(0)

#define IMX_FSPI_RSER			0x164
#define IMX_FSPI_RSER_TFIE		BIT(0)

#define IMX_FSPI_SPTRCLR			0x16c
#define IMX_FSPI_SPTRCLR_IPPTRC		BIT(8)
#define IMX_FSPI_SPTRCLR_BFPTRC		BIT(0)

#define IMX_FSPI_SFA1AD			0x180
#define IMX_FSPI_SFA2AD			0x184
#define IMX_FSPI_SFB1AD			0x188
#define IMX_FSPI_SFB2AD			0x18c
#define IMX_FSPI_RBDR(x)			(0x200 + ((x) * 4))

#define IMX_FSPI_LUTKEY			0x300
#define IMX_FSPI_LUTKEY_VALUE		0x5AF05AF0

#define IMX_FSPI_LCKCR			0x304
#define IMX_FSPI_LCKER_LOCK		BIT(0)
#define IMX_FSPI_LCKER_UNLOCK		BIT(1)

#define IMX_FSPI_LOOKUPTABLE_BASE		0x310
#define IMX_FSPI_LOOKUPTABLE_OFFSET		(FSPI_SEQID_LOOKUPTABLE * 4 * 4)
#define IMX_FSPI_LOOKUPTABLE_REG(idx) \
	(IMX_FSPI_LOOKUPTABLE_BASE + IMX_FSPI_LOOKUPTABLE_OFFSET + (idx) * 4)

#define IMX_FSPI_AHB_LOOKUPTABLE_OFFSET		(FSPI_SEQID_LOOKUPTABLE_AHB * 4 * 4)
#define IMX_FSPI_AHB_LOOKUPTABLE_REG(idx) \
	(IMX_FSPI_LOOKUPTABLE_BASE + IMX_FSPI_AHB_LOOKUPTABLE_OFFSET + (idx) * 4)

#define LOOKUPTABLE_STOP		0
#define LOOKUPTABLE_CMD			1
#define LOOKUPTABLE_ADDR		2
#define LOOKUPTABLE_DUMMY		3
#define LOOKUPTABLE_MODE		4
#define LOOKUPTABLE_MODE2		5
#define LOOKUPTABLE_MODE4		6
#define LOOKUPTABLE_FSL_READ		7
#define LOOKUPTABLE_FSL_WRITE		8
#define LOOKUPTABLE_JMP_ON_CS		9
#define LOOKUPTABLE_ADDR_DDR		10
#define LOOKUPTABLE_MODE_DDR		11
#define LOOKUPTABLE_MODE2_DDR		12
#define LOOKUPTABLE_MODE4_DDR		13
#define LOOKUPTABLE_FSL_READ_DDR	14
#define LOOKUPTABLE_FSL_WRITE_DDR	15
#define LOOKUPTABLE_DATA_LEARN		16

#define LOOKUPTABLE_PAD(x) (FindMostSignBit(x) - 1)

/*
 * Macro for the LOOKUPTABLE entries
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */

#define PAD_SHIFT           8
#define INSTR_SHIFT        10
#define OPRND_SHIFT        16

#define LOOKUPTABLE_DEF(idx, ins, pad, opr)					\
    ((((ins) << INSTR_SHIFT) | ((pad) << PAD_SHIFT) | \
    (opr)) << (((idx) % 2) * OPRND_SHIFT))

#define IMX_FSPI_QUIRK_SWAP_ENDIAN	BIT(0)
#define IMX_FSPI_QUIRK_4X_INT_CLK	BIT(1)
#define IMX_FSPI_QUIRK_TKT253890		BIT(2)
#define IMX_FSPI_QUIRK_TKT245618		BIT(3)
#define IMX_FSPI_QUIRK_BASE_INTERNAL	BIT(4)
#define IMX_FSPI_QUIRK_USE_TDH_SETTING	BIT(5)

#define IMX_FSPI_QUIRK_SINGLE_BUS		BIT(6)

static struct nxp_fspi_devtype_data imx7d_data = {
	.rxfifo = SZ_128,
	.txfifo = SZ_512,
	.ahb_buf_size = SZ_1K,
	.quirks = IMX_FSPI_QUIRK_TKT253890 | IMX_FSPI_QUIRK_4X_INT_CLK |
		  IMX_FSPI_QUIRK_USE_TDH_SETTING,
};

static struct FspiData g_fsl_fspi;

#define out(val, addr) out_le32((volatile unsigned*)(addr), (int)(val))
#define in(addr)        in_le32((volatile unsigned*)(addr))

struct FspiData* GetFspiData() {
    g_fsl_fspi.devtype_data = &imx7d_data;
    g_fsl_fspi.memmap_phy = 0x8000000;
    g_fsl_fspi.memmap_phy_size = 0x10000;
    return &g_fsl_fspi;
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


static inline int NeedsFillTxfifo(struct FspiData *q)
{
	return q->devtype_data->quirks & IMX_FSPI_QUIRK_TKT253890;
}

static inline int NeedSambaBaseOffset(struct FspiData *q)
{
	return !(q->devtype_data->quirks & IMX_FSPI_QUIRK_BASE_INTERNAL);
}

static inline int NeedsTdhSetting(struct FspiData *q)
{
	return q->devtype_data->quirks & IMX_FSPI_QUIRK_USE_TDH_SETTING;
}

static inline int NeedsSingleBus(struct FspiData *q)
{
	return q->devtype_data->quirks & IMX_FSPI_QUIRK_SINGLE_BUS;
}

static int FslQspiCheckBusw(struct FspiData *q, UINT8 width)
{
	switch (width) {
	case 1:
	case 2:
	case 4:
		return 0;
	}

	return -1;
}

BOOLEAN FslQspiSupportsOp(const FspiCommand *cmd)
{
	struct FspiData *q = GetFspiData();
	int ret;

	ret = FslQspiCheckBusw(q, cmd->Command.busw);

	if (cmd->Address.byte_cnt)
		ret |= FslQspiCheckBusw(q, cmd->Address.busw);

	if (cmd->DummyBytes.byte_cnt)
		ret |= FslQspiCheckBusw(q, cmd->DummyBytes.busw);

	if (cmd->Data.byte_cnt)
		ret |= FslQspiCheckBusw(q, cmd->Data.busw);

	if (ret)
		return FALSE;

	if (cmd->Address.byte_cnt +
	   (cmd->DummyBytes.byte_cnt ? 1 : 0) +
	   (cmd->Data.byte_cnt ? 1 : 0) > 6)
		return FALSE;

	if (cmd->DummyBytes.byte_cnt &&
	    (cmd->DummyBytes.byte_cnt * 8 / cmd->DummyBytes.busw > 64))
		return FALSE;

	if (cmd->Data.direction == CMD_DIRECTION_IN &&
	    (cmd->Data.byte_cnt > q->devtype_data->ahb_buf_size ||
	     (cmd->Data.byte_cnt > q->devtype_data->rxfifo - 4 &&
	      !IS_ALIGNED(cmd->Data.byte_cnt, 8))))
		return FALSE;

	if (cmd->Data.direction == CMD_DIRECTION_OUT &&
	    cmd->Data.byte_cnt > q->devtype_data->txfifo)
		return FALSE;

	return TRUE;
}

static void FslQspiPrepareLut(struct FspiData *q,
				 const FspiCommand *cmd)
{
	void *base = q->iobase;
	UINT32 lutval[4] = {};
	int lutidx = 1, i;

	lutval[0] |= LOOKUPTABLE_DEF(0, LOOKUPTABLE_CMD, LOOKUPTABLE_PAD(cmd->Command.busw),
			     cmd->Command.code);

	if (cmd->Address.byte_cnt) {
		lutval[lutidx / 2] |= LOOKUPTABLE_DEF(lutidx, LOOKUPTABLE_ADDR,
					      LOOKUPTABLE_PAD(cmd->Address.busw),
					      (cmd->Address.byte_cnt == 4) ? 0x20 : 0x18);
		lutidx++;
    }

	if (cmd->DummyBytes.byte_cnt) {
		lutval[lutidx / 2] |= LOOKUPTABLE_DEF(lutidx, LOOKUPTABLE_DUMMY,
					      LOOKUPTABLE_PAD(cmd->DummyBytes.busw),
					      cmd->DummyBytes.byte_cnt * 8 /
					      cmd->DummyBytes.busw);
		lutidx++;
	}

	if (cmd->Data.byte_cnt) {
		lutval[lutidx / 2] |= LOOKUPTABLE_DEF(lutidx,
					      cmd->Data.direction == CMD_DIRECTION_IN ?
					      LOOKUPTABLE_FSL_READ : LOOKUPTABLE_FSL_WRITE,
					      LOOKUPTABLE_PAD(cmd->Data.busw),
					      0);
		lutidx++;
	}

	lutval[lutidx / 2] |= LOOKUPTABLE_DEF(lutidx, LOOKUPTABLE_STOP, 0, 0);

	out(IMX_FSPI_LUTKEY_VALUE, q->iobase + IMX_FSPI_LUTKEY);
	out(IMX_FSPI_LCKER_UNLOCK, q->iobase + IMX_FSPI_LCKCR);

	//DEBUG((DEBUG_ERROR, "CMD[%x] lutval[0:%x \t 1:%x \t 2:%x \t 3:%x]\n",
	//	cmd->Command.code, lutval[0], lutval[1], lutval[2], lutval[3]));

	/* fill LUT */
	for (i = 0; i < ARRAY_SIZE(lutval); i++)
		out(lutval[i], base + IMX_FSPI_LOOKUPTABLE_REG(i));

	if (TRUE /*IS_ENABLED(CONFIG_FSL_QSPI_AHB_FULL_MAP)*/) {
		if (cmd->Data.byte_cnt && cmd->Data.direction == CMD_DIRECTION_IN &&
		    cmd->Address.byte_cnt) {
			for (i = 0; i < ARRAY_SIZE(lutval); i++)
				out(lutval[i], base + IMX_FSPI_AHB_LOOKUPTABLE_REG(i));
		}
	}

	/* lock LUT */
	out(IMX_FSPI_LUTKEY_VALUE, q->iobase + IMX_FSPI_LUTKEY);
	out(IMX_FSPI_LCKER_LOCK, q->iobase + IMX_FSPI_LCKCR);
}

static void FslQspiInvalidate(struct FspiData *q)
{
	UINT32 reg;

	reg = in(q->iobase + IMX_FSPI_MCR);
	reg |= IMX_FSPI_MCR_SWRSTHD_MASK | IMX_FSPI_MCR_SWRSTSD_MASK;
	out(reg, q->iobase + IMX_FSPI_MCR);

	MicroSecondDelay(1);

	reg &= ~(IMX_FSPI_MCR_SWRSTHD_MASK | IMX_FSPI_MCR_SWRSTSD_MASK);
	out(reg, q->iobase + IMX_FSPI_MCR);
}

int platform_chip_selected = 0;

static void FslQspiSelectMem(struct FspiData *q)
{
	if (q->selected == platform_chip_selected)
		return;

	q->selected = platform_chip_selected;
	FslQspiInvalidate(q);
}

static UINT32 FslQspimemsize_per_cs(struct FspiData *q)
{
	if (NeedsSingleBus(q))
		return q->memmap_phy_size / 2;
	else
		return q->memmap_phy_size / 4;
}

static void FslQspiread_ahb(struct FspiData *q, const FspiCommand *cmd)
{
	void *ahb_read_addr = q->ahb_addr;

	if (cmd->Address.byte_cnt)
		ahb_read_addr += cmd->Address.val;

	CopyMem(cmd->Data.buf.in,
		      ahb_read_addr + q->selected * FslQspimemsize_per_cs(q),
		      cmd->Data.byte_cnt);
}

static void FslQspifill_txfifo(struct FspiData *q,
				 const FspiCommand *cmd)
{
	void *base = q->iobase;
	int i;
	UINT32 val;

	for (i = 0; i < ALIGN_DOWN(cmd->Data.byte_cnt, 4); i += 4) {
		CopyMem(&val, cmd->Data.buf.out + i, 4);
		out(val, base + IMX_FSPI_TBDR);
	}

	if (i < cmd->Data.byte_cnt) {
		CopyMem(&val, cmd->Data.buf.out + i, cmd->Data.byte_cnt - i);
		out(val, base + IMX_FSPI_TBDR);
	}

	if (NeedsFillTxfifo(q)) {
		for (i = cmd->Data.byte_cnt; i < 16; i += 4)
			out(0, base + IMX_FSPI_TBDR);
	}
}

static void FslQspiread_rxfifo(struct FspiData *q,
				 const FspiCommand *cmd)
{
	void *base = q->iobase;
	int i;
	UINT8 *buf = cmd->Data.buf.in;
	UINT32 val;

	for (i = 0; i < ALIGN_DOWN(cmd->Data.byte_cnt, 4); i += 4) {
		val = in(base + IMX_FSPI_RBDR(i / 4));
		CopyMem(buf + i, &val, 4);
	}

	if (i < cmd->Data.byte_cnt) {
		val = in(base + IMX_FSPI_RBDR(i / 4));
		CopyMem(buf + i, &val, cmd->Data.byte_cnt - i);
	}
}

static int FslQspireadl_poll_tout(struct FspiData *q, void *base,
				    UINT32 mask, UINT32 delay_us, UINT32 timeout_us)
{
	UINT32 reg;

    return ReadlPollSleepTimeout(base, reg, !(reg & mask), delay_us, timeout_us);

	//return readl_poll_timeout(base, reg, !(reg & mask), timeout_us);
}

static int FslQspiDoOp(struct FspiData *q, const FspiCommand *cmd)
{
	void *base = q->iobase;
	int err = 0;

	out(cmd->Data.byte_cnt | IMX_FSPI_IPCR_SEQID(FSPI_SEQID_LOOKUPTABLE),
		    base + IMX_FSPI_IPCR);

	/* wait for the controller being ready */
	err = FslQspireadl_poll_tout(q, base + IMX_FSPI_SR,
				       (IMX_FSPI_SR_IP_ACC_MASK |
					IMX_FSPI_SR_AHB_ACC_MASK),
					10, 1000);

	if (!err && cmd->Data.byte_cnt && cmd->Data.direction == CMD_DIRECTION_IN)
		FslQspiread_rxfifo(q, cmd);

	return err;
}

int FspiRunCommand(const FspiCommand *cmd)
{
	struct FspiData *q = GetFspiData();
	void *base = q->iobase;
	UINT32 addr_offset = 0;
	int err = 0;

	/* wait for the controller being ready */
	FslQspireadl_poll_tout(q, base + IMX_FSPI_SR, (IMX_FSPI_SR_IP_ACC_MASK |
				 IMX_FSPI_SR_AHB_ACC_MASK), 10, 1000);

	FslQspiSelectMem(q);

	if (NeedSambaBaseOffset(q))
		addr_offset = q->memmap_phy;

	if (cmd->Address.byte_cnt)
		addr_offset += cmd->Address.val;

	out(q->selected * FslQspimemsize_per_cs(q) + addr_offset,
		base + IMX_FSPI_SFAR);

	out(in(base + IMX_FSPI_MCR) |
		    IMX_FSPI_MCR_CLR_RXF_MASK | IMX_FSPI_MCR_CLR_TXF_MASK,
		    base + IMX_FSPI_MCR);

	out(IMX_FSPI_SPTRCLR_BFPTRC | IMX_FSPI_SPTRCLR_IPPTRC,
		    base + IMX_FSPI_SPTRCLR);

	FslQspiPrepareLut(q, cmd);

	/*
	 * If we have large chunks of data, we read them through the AHB bus
	 * by accessing the mapped memory. In all other cases we use
	 * IP commands to access the flash.
	 */
	if (cmd->Data.byte_cnt > (q->devtype_data->rxfifo - 4) &&
	    cmd->Data.direction == CMD_DIRECTION_IN) {
		FslQspiread_ahb(q, cmd);
	} else {
		out(IMX_FSPI_RBCT_WMRK_MASK |
			    IMX_FSPI_RBCT_RXBRD_USEIPS, base + IMX_FSPI_RBCT);

		if (cmd->Data.byte_cnt && cmd->Data.direction == CMD_DIRECTION_OUT)
			FslQspifill_txfifo(q, cmd);

		err = FslQspiDoOp(q, cmd);
	}

	FslQspiInvalidate(q);

	return err;
}

int FspiAdjustCommandSize(FspiCommand *cmd)
{
	struct FspiData *q = GetFspiData();

	if (cmd->Data.direction == CMD_DIRECTION_OUT) {
		if (cmd->Data.byte_cnt > q->devtype_data->txfifo)
			cmd->Data.byte_cnt = q->devtype_data->txfifo;
	} else {
		if (cmd->Data.byte_cnt > q->devtype_data->ahb_buf_size)
			cmd->Data.byte_cnt = q->devtype_data->ahb_buf_size;
		else if (cmd->Data.byte_cnt > (q->devtype_data->rxfifo - 4))
			cmd->Data.byte_cnt = ALIGN_DOWN(cmd->Data.byte_cnt, 8);
	}

	return 0;
}

int FslQspiDefaultSetup(struct FspiData *q)
{
	void *base = q->iobase;
	UINT32 reg, addr_offset = 0, memsize_cs;

	/* Reset the module */
	out(IMX_FSPI_MCR_SWRSTSD_MASK | IMX_FSPI_MCR_SWRSTHD_MASK,
		    base + IMX_FSPI_MCR);
	//udelay(1);
	MicroSecondDelay(1);

	/* Disable the module */
	out(IMX_FSPI_MCR_MDIS_MASK | IMX_FSPI_MCR_RESERVED_MASK,
		    base + IMX_FSPI_MCR);

	if (NeedsTdhSetting(q))
		out(in(base + IMX_FSPI_FLSHCR) &
			    ~IMX_FSPI_FLSHCR_TDH_MASK,
			    base + IMX_FSPI_FLSHCR);

	reg = in(base + IMX_FSPI_SMPR);
	out(reg & ~(IMX_FSPI_SMPR_FSDLY_MASK
			| IMX_FSPI_SMPR_FSPHS_MASK
			| IMX_FSPI_SMPR_HSENA_MASK
			| IMX_FSPI_SMPR_DDRSMP_MASK), base + IMX_FSPI_SMPR);

	/* We only use the buffer3 for AHB read */
	out(0, base + IMX_FSPI_BUF0IND);
	out(0, base + IMX_FSPI_BUF1IND);
	out(0, base + IMX_FSPI_BUF2IND);

	out(IMX_FSPI_BFGENCR_SEQID(FSPI_SEQID_LOOKUPTABLE_AHB),
			q->iobase + IMX_FSPI_BFGENCR);

	out(IMX_FSPI_RBCT_WMRK_MASK, base + IMX_FSPI_RBCT);
	out(IMX_FSPI_BUF3CR_ALLMST_MASK |
		    IMX_FSPI_BUF3CR_ADATSZ(q->devtype_data->ahb_buf_size / 8),
		    base + IMX_FSPI_BUF3CR);

	if (NeedSambaBaseOffset(q))
		addr_offset = q->memmap_phy;

	memsize_cs = FslQspimemsize_per_cs(q);
	out(memsize_cs + addr_offset,
		    base + IMX_FSPI_SFA1AD);
	out(memsize_cs * 2 + addr_offset,
		    base + IMX_FSPI_SFA2AD);
	if (!NeedsSingleBus(q)) {
		out(memsize_cs * 3 + addr_offset,
			    base + IMX_FSPI_SFB1AD);
		out(memsize_cs * 4 + addr_offset,
			    base + IMX_FSPI_SFB2AD);
	}

	q->selected = -1;

	/* Enable the module */
	out(IMX_FSPI_MCR_RESERVED_MASK | IMX_FSPI_MCR_END_CFG_MASK,
		    base + IMX_FSPI_MCR);
	return 0;
}

UINT64 FspiTimerGetUsec() {
  UINT64 ticks = GetPerformanceCounter();
  return GetTimeInNanoSecond(ticks) / 1000;
}
