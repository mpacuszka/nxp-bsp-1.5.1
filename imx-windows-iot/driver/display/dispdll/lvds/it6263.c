/*
 * Copyright 2017-2019,2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/regmap.h>
#include <linux/jiffies.h>
#include <video/videomode.h>
#include <linux/media-bus-format.h>
#include "it6263.h"

#define REG_VENDOR_ID(n)	(0x00 + (n))	/* n: 0/1 */
#define REG_DEVICE_ID(n)	(0x02 + (n))	/* n: 0/1 */
#define LVDS_VENDER_ID_LOW	0x15
#define LVDS_VENDER_ID_HIGH	0xCA
#define LVDS_DEVICE_ID_LOW	0x61
#define LVDS_DEVICE_ID_HIGH	0x62
#define HDMI_VENDER_ID_LOW	0x01
#define HDMI_VENDER_ID_HIGH	0xCA
#define HDMI_DEVICE_ID_LOW	0x13
#define HDMI_DEVICE_ID_HIGH	0x76

/* LVDS registers */
#define LVDS_REG_SW_RST		0x05
#define SOFT_REFCLK_DM_RST	BIT(0)
#define SOFT_PCLK_DM_RST	BIT(1)

#define LVDS_REG_MODE		0x2C
#define LVDS_COLOR_DEPTH	0x3
enum {
	LVDS_COLOR_DEPTH_18 = 0,
	LVDS_COLOR_DEPTH_24,
	LVDS_COLOR_DEPTH_30,
	LVDS_COLOR_DEPTH_36,
};
#define LVDS_OUT_MAP		BIT(4)
#define VESA			BIT(4)
#define JEIDA			0
#define DMODE			BIT(7)
#define SPLIT_MODE		BIT(7)
#define SINGLE_MODE		0

#define LVDS_REG_STABLE		0x30
#define VIDEO_STABLE		BIT(0)
#define PCLK_LOCK		BIT(1)

#define LVDS_REG_39		0x39

#define LVDS_REG_PLL		0x3C
#define LVDS_REG_AFE_3E		0x3E
#define LVDS_REG_AFE_3F		0x3F
#define LVDS_REG_AFE_47		0x47
#define LVDS_REG_AFE_48		0x48
#define LVDS_REG_AFE_4F		0x4F
#define LVDS_REG_52		0x52
#define LVDS_REG_PCLK_CNT_HIGH	0x57
#define LVDS_REG_PCLK_CNT_LOW	0x58

/*
 * HDMI registers
 *
 * Registers are separated into three banks:
 * 1) common bank: 0x00 ~ 0x2F
 * 2) bank0: 0x30  ~ 0xFF
 * 3) bank1: 0x130 ~ 0x1FF	(HDMI packet registers)
 *
 * Use register HDMI_REG_BANK_CTRL @ 0x0F[1:0] to select bank0/1:
 * 2b'00 - bank0
 * 2b'01 - bank1
 */

/******************************/
/* HDMI register common bank  */
/******************************/

/* HDMI genernal registers */
#define HDMI_REG_SW_RST		0x04
#define SOFTREF_RST		BIT(5)
#define SOFTA_RST		BIT(4)
#define SOFTV_RST		BIT(3)
#define AUD_RST			BIT(2)
#define HDCP_RST		BIT(0)
#define HDMI_RST_ALL		(SOFTREF_RST | SOFTA_RST | SOFTV_RST | \
				 AUD_RST | HDCP_RST)

#define HDMI_REG_INT_CTRL	0x05
#define INTPOL_ACTH		BIT(7)
#define INTPOL_ACTL		0
#define INTIOMODE_OPENDRAIN	BIT(6)
#define INTIOMODE_PUSHPULL	0
#define SELXTAL			BIT(5)	/* REFCLK <= XTALCLK */
#define SELXTAL_QUARTER		0	/* REFCLK <= OSCCLK/4 */
#define PDREFCNT(n)		(((n) >> 2) << 2)	/* REFCLK Div(n) */
#define PDREFCLK		BIT(1)
#define PDTXCLK_GATED		BIT(0)
#define PDTXCLK_ACTIVE		0

#define HDMI_REG_INT_STAT(n)	(0x05 + (n))	/* n: 1/2/3 */
#define HDMI_REG_INT_MASK(n)	(0x08 + (n))	/* n: 1/2/3 */

/* INT1 */
#define INT_AUD_OVERFLOW	BIT(7)
#define INT_RDDC_NOACK		BIT(5)
#define INT_DDCFIFO_ERR		BIT(4)
#define INT_DDC_BUS_HANG	BIT(2)
#define INT_RX_SENSE		BIT(1)
#define INT_HPD			BIT(0)

/* INT2 */
#define INT_VID_UNSTABLE	BIT(6)
#define INT_PKTACP		BIT(5)
#define INT_PKTNULL		BIT(4)
#define INT_PKTGEN		BIT(3)
#define INT_KSVLIST_CHK		BIT(2)
#define INT_AUTH_DONE		BIT(1)
#define INT_AUTH_FAIL		BIT(0)

/* INT3 */
#define INT_AUD_CTS		BIT(6)
#define INT_VSYNC		BIT(5)
#define INT_VIDSTABLE		BIT(4)
#define INT_PKTMPG		BIT(3)
#define INT_PKTGBD		BIT(2)
#define INT_PKTAUD		BIT(1)
#define INT_PKTAVI		BIT(0)

#define INT_MASK_AUD_CTS	BIT(5)
#define INT_MASK_VSYNC		BIT(4)
#define INT_MASK_VIDSTABLE	BIT(3)
#define INT_MASK_PKTMPG		BIT(2)
#define INT_MASK_PKTGBD		BIT(1)
#define INT_MASK_PKTAUD		BIT(0)

#define HDMI_REG_INT_CLR(n)	(0x0C + (n))	/* n: 0/1 */

/* CLR0 */
#define INT_CLR_PKTACP		BIT(7)
#define INT_CLR_PKTNULL		BIT(6)
#define INT_CLR_PKTGEN		BIT(5)
#define INT_CLR_KSVLIST_CHK	BIT(4)
#define INT_CLR_AUTH_DONE	BIT(3)
#define INT_CLR_AUTH_FAIL	BIT(2)
#define INT_CLR_RXSENSE		BIT(1)
#define INT_CLR_HPD		BIT(0)

/* CLR1 */
#define INT_CLR_VSYNC		BIT(7)
#define INT_CLR_VIDSTABLE	BIT(6)
#define INT_CLR_PKTMPG		BIT(5)
#define INT_CLR_PKTGBD		BIT(4)
#define INT_CLR_PKTAUD		BIT(3)
#define INT_CLR_PKTAVI		BIT(2)
#define INT_CLR_VID_UNSTABLE	BIT(0)

#define HDMI_REG_SYS_STATUS	0x0E
#define INT_ACTIVE		BIT(7)
#define HPDETECT		BIT(6)
#define RXSENDETECT		BIT(5)
#define TXVIDSTABLE		BIT(4)
#define CTSINTSTEP		0xC
#define CLR_AUD_CTS		BIT(1)
#define INTACTDONE		BIT(0)

#define HDMI_REG_BANK_CTRL	0x0F
#define BANK_SEL(n)		((n) ? 1 : 0)

/* HDMI System DDC control registers */
#define HDMI_REG_DDC_MASTER_CTRL	0x10
#define MASTER_SEL_HOST			BIT(0)
#define MASTER_SEL_HDCP			0

#define HDMI_REG_DDC_HEADER		0x11
#define DDC_HDCP_ADDRESS		0x74

#define HDMI_REG_DDC_REQOFF		0x12
#define HDMI_REG_DDC_REQCOUNT		0x13
#define HDMI_REG_DDC_EDIDSEG		0x14

#define HDMI_REG_DDC_CMD		0x15
#define DDC_CMD_SEQ_BURSTREAD		0x0
#define DDC_CMD_LINK_CHKREAD		0x2
#define DDC_CMD_EDID_READ		0x3
#define DDC_CMD_FIFO_CLR		0x9
#define DDC_CMD_GEN_SCLCLK		0xA
#define DDC_CMD_ABORT			0xF

#define HDMI_REG_DDC_STATUS		0x16
#define DDC_DONE			BIT(7)
#define DDC_ACT				BIT(6)
#define DDC_NOACK			BIT(5)
#define DDC_WAITBUS			BIT(4)
#define DDC_ARBILOSE			BIT(3)
#define DDC_ERROR			(DDC_NOACK | DDC_WAITBUS | DDC_ARBILOSE)
#define DDC_FIFOFULL			BIT(2)
#define DDC_FIFOEMPTY			BIT(1)

#define HDMI_DDC_FIFO_SIZE		32	/* bytes */
#define HDMI_REG_DDC_READFIFO		0x17
#define HDMI_REG_ROM_STAT		0x1C
#define HDMI_REG_LVDS_PORT		0x1D	/* LVDS input ctrl i2c addr */
#define HDMI_REG_LVDS_PORT_EN		0x1E	/* and to enable */
#define LVDS_INPUT_CTRL_I2C_ADDR	0x33

/***********************/
/* HDMI register bank0 */
/***********************/

/* HDMI clock control registers */
#define HDMI_REG_CLK_CTRL1		0x59
#define EN_TXCLK_COUNT			BIT(5)
#define VDO_LATCH_EDGE			BIT(3)

/* HDMI AFE registers */
#define HDMI_REG_AFE_DRV_CTRL		0x61
#define AFE_DRV_PWD			BIT(5)
#define AFE_DRV_RST			BIT(4)
#define AFE_DRV_PDRXDET			BIT(2)
#define AFE_DRV_TERMON			BIT(1)
#define AFE_DRV_ENCAL			BIT(0)

#define HDMI_REG_AFE_XP_CTRL		0x62
#define AFE_XP_GAINBIT			BIT(7)
#define AFE_XP_PWDPLL			BIT(6)
#define AFE_XP_ENI			BIT(5)
#define AFE_XP_ER0			BIT(4)
#define AFE_XP_RESETB			BIT(3)
#define AFE_XP_PWDI			BIT(2)
#define AFE_XP_DEI			BIT(1)
#define AFE_XP_DER			BIT(0)

#define HDMI_REG_AFE_ISW_CTRL		0x63
#define AFE_RTERM_SEL			BIT(7)
#define AFE_IP_BYPASS			BIT(6)
#define AFE_DRV_ISW			0x38
#define AFE_DRV_ISWK			7

#define HDMI_REG_AFE_IP_CTRL		0x64
#define AFE_IP_GAINBIT			BIT(7)
#define AFE_IP_PWDPLL			BIT(6)
#define AFE_IP_CKSEL			0x30
#define AFE_IP_ER0			BIT(3)
#define AFE_IP_RESETB			BIT(2)
#define AFE_IP_ENC			BIT(1)
#define AFE_IP_EC1			BIT(0)

/* HDMI input data format registers */
#define HDMI_REG_INPUT_MODE		0x70
#define IN_RGB				0x00
#define IN_YUV422			0x40
#define IN_YUV444			0x80

#define HDMI_REG_TXFIFO_RST		0x71
#define ENAVMUTERST			BIT(0)
#define TXFFRST				BIT(1)

/* HDMI pattern generation SYNC/DE registers */
#define HDMI_REG_9X(n)			(0x90 + (n))	/* n: 0x0 ~ 0xF */
#define HDMI_REG_AX(n)			(0xA0 + (n))	/* n: 0x0 ~ 0xF */
#define HDMI_REG_B0			0xB0

/* HDMI general control registers */
#define HDMI_REG_HDMI_MODE		0xC0
#define TX_HDMI_MODE			1
#define TX_DVI_MODE			0

#define HDMI_REG_GCP			0xC1
#define AVMUTE				BIT(0)
#define BLUE_SCR_MUTE			BIT(1)
#define NODEF_PHASE			BIT(2)
#define PHASE_RESYNC			BIT(3)
#define HDMI_COLOR_DEPTH		0x70
enum {
	HDMI_COLOR_DEPTH_DEF = 0x0,	/* default as 24bit */
	HDMI_COLOR_DEPTH_24  = 0x40,
	HDMI_COLOR_DEPTH_30  = 0x50,
	HDMI_COLOR_DEPTH_36  = 0x60,
	HDMI_COLOR_DEPTH_48  = 0x70,
};

#define HDMI_REG_OESS_CYCLE		0xC3
#define HDMI_REG_ENCRYPTION		0xC4	/* HDCP */

#define HDMI_REG_PKT_SINGLE_CTRL	0xC5
#define SINGLE_PKT			BIT(0)
#define BURST_PKT			0

#define HDMI_REG_PKT_GENERAL_CTRL	0xC6
#define HDMI_REG_NULL_CTRL		0xC9
#define HDMI_REG_ACP_CTRL		0xCA
#define HDMI_REG_ISRC1_CTRL		0xCB
#define HDMI_REG_ISRC2_CTRL		0xCC
#define HDMI_REG_AVI_INFOFRM_CTRL	0xCD
#define HDMI_REG_AUD_INFOFRM_CTRL	0xCE
#define HDMI_REG_SPD_INFOFRM_CTRL	0xCF
#define HDMI_REG_MPG_INFOFRM_CTRL	0xD0
#define ENABLE_PKT			BIT(0)
#define REPEAT_PKT			BIT(1)

/***********************/
/* HDMI register bank1 */
/***********************/

/* AVI packet registers */
#define HDMI_REG_AVI_DB1		0x58
#define AVI_DB1_COLOR_SPACE		0x60
enum {
	AVI_COLOR_SPACE_RGB    = 0x00,
	AVI_COLOR_SPACE_YUV422 = 0x20,
	AVI_COLOR_SPACE_YUV444 = 0x40,
};

struct it6263 {
	struct i2c_client *hdmi_i2c;
	struct i2c_client *lvds_i2c;
	struct regmap *hdmi_regmap;
	struct regmap *lvds_regmap;
	bool is_hdmi;
	bool split_mode;
	u32 ldb_bus_format;
};

struct it6263_minimode {
	int hdisplay;
	int vdisplay;
	int vrefresh;
};

static const struct it6263_minimode it6263_bad_mode_db[] = {
	{1600, 900,  60},
	{1280, 1024, 60},
	{1280, 720,  30},
	{1280, 720,  25},
	{1280, 720,  24},
	{1152, 864,  75},
};

static inline void lvds_update_bits(struct it6263 *it6263, unsigned int reg,
				    unsigned int mask, unsigned int val)
{
	regmap_update_bits(it6263->lvds_regmap, reg, mask, val);
}

static inline void hdmi_update_bits(struct it6263 *it6263, unsigned int reg,
				    unsigned int mask, unsigned int val)
{
	regmap_update_bits(it6263->hdmi_regmap, reg, mask, val);
}

static void it6263_lvds_reset(struct it6263 *it6263)
{
	/* AFE PLL reset */
	lvds_update_bits(it6263, LVDS_REG_PLL, 0x1, 0x0);
	usleep_range(1000, 2000);
	lvds_update_bits(it6263, LVDS_REG_PLL, 0x1, 0x1);

	/* pclk reset */
	lvds_update_bits(it6263, LVDS_REG_SW_RST,
				SOFT_PCLK_DM_RST, SOFT_PCLK_DM_RST);
	usleep_range(1000, 2000);
	lvds_update_bits(it6263, LVDS_REG_SW_RST, SOFT_PCLK_DM_RST, 0x0);

	usleep_range(1000, 2000);
}

static void it6263_lvds_set_interface(struct it6263 *it6263)
{
	/* color depth */
	lvds_update_bits(it6263, LVDS_REG_MODE, LVDS_COLOR_DEPTH,
						LVDS_COLOR_DEPTH_24);

	switch (it6263->ldb_bus_format) {
		case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
			lvds_update_bits(it6263, LVDS_REG_MODE, LVDS_OUT_MAP, VESA);
			break;
		case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		default:
			lvds_update_bits(it6263, LVDS_REG_MODE, LVDS_OUT_MAP, JEIDA);
			break;
	}

	if (it6263->split_mode) {
		lvds_update_bits(it6263, LVDS_REG_MODE, DMODE, SPLIT_MODE);
		lvds_update_bits(it6263, LVDS_REG_52, BIT(1), BIT(1));
	} else {
		lvds_update_bits(it6263, LVDS_REG_MODE, DMODE, SINGLE_MODE);
		lvds_update_bits(it6263, LVDS_REG_52, BIT(1), 0);
	}
}

static void it6263_lvds_set_afe(struct it6263 *it6263)
{
	struct regmap *regmap = it6263->lvds_regmap;

	regmap_write(regmap, LVDS_REG_AFE_3E, 0xaa);
	regmap_write(regmap, LVDS_REG_AFE_3F, 0x02);
	regmap_write(regmap, LVDS_REG_AFE_47, 0xaa);
	regmap_write(regmap, LVDS_REG_AFE_48, 0x02);
	regmap_write(regmap, LVDS_REG_AFE_4F, 0x11);

	lvds_update_bits(it6263, LVDS_REG_PLL, 0x07, 0);
}

static void it6263_lvds_config(struct it6263 *it6263, bool reset)
{
	if (reset)
		it6263_lvds_reset(it6263);
	it6263_lvds_set_interface(it6263);
	it6263_lvds_set_afe(it6263);
}

static void it6263_hdmi_config(struct it6263 *it6263)
{
	regmap_write(it6263->hdmi_regmap, HDMI_REG_INPUT_MODE, IN_RGB);

	hdmi_update_bits(it6263, HDMI_REG_GCP, HDMI_COLOR_DEPTH,
						HDMI_COLOR_DEPTH_24);
}

void it6263_bridge_disable(struct i2c_client *client_hdmi)
{
	struct it6263 *it6263 = i2c_get_clientdata(client_hdmi);
	struct regmap *regmap = it6263->hdmi_regmap;

	/* AV mute */
	hdmi_update_bits(it6263, HDMI_REG_GCP, AVMUTE, AVMUTE);

	if (it6263->is_hdmi)
		regmap_write(regmap, HDMI_REG_PKT_GENERAL_CTRL, 0);

	hdmi_update_bits(it6263, HDMI_REG_SW_RST, SOFTV_RST, SOFTV_RST);
	regmap_write(regmap, HDMI_REG_AFE_DRV_CTRL, AFE_DRV_RST | AFE_DRV_PWD);
}

void it6263_bridge_enable(struct i2c_client *client_hdmi)
{
	struct it6263 *it6263 = i2c_get_clientdata(client_hdmi);
	struct regmap *regmap = it6263->hdmi_regmap;
	unsigned long long timeout;
	unsigned int status;
	bool is_stable = false;
	int i;

	regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, BANK_SEL(1));
	/* set the color space to RGB in the AVI packet */
	hdmi_update_bits(it6263, HDMI_REG_AVI_DB1, AVI_DB1_COLOR_SPACE,
							AVI_COLOR_SPACE_RGB);
	regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, BANK_SEL(0));

	/* software video reset */
	hdmi_update_bits(it6263, HDMI_REG_SW_RST, SOFTV_RST, SOFTV_RST);
	usleep_range(1000, 2000);
	hdmi_update_bits(it6263, HDMI_REG_SW_RST, SOFTV_RST, 0);

	/* reconfigure LVDS and retry several times in case video is instable */
	for (i = 0; i < 3; i++) {
		timeout = jiffies + msecs_to_jiffies(500);
		do {
			regmap_read(regmap, HDMI_REG_SYS_STATUS, &status);
		} while (!(status & TXVIDSTABLE) &&
					time_before(jiffies, timeout));

		if (status & TXVIDSTABLE) {
			is_stable = true;
			break;
		}

		it6263_lvds_config(it6263, true);

		dev_dbg(&it6263->hdmi_i2c->dev,
					"retry to lock input video %d\n", i);
	}

	if (!is_stable)
		dev_warn(&it6263->hdmi_i2c->dev,
				"failed to wait for video stable\n");

	regmap_write(regmap, HDMI_REG_AFE_DRV_CTRL, 0);

	/* AV unmute */
	hdmi_update_bits(it6263, HDMI_REG_GCP, AVMUTE, 0);

	if (it6263->is_hdmi)
		regmap_write(regmap, HDMI_REG_PKT_GENERAL_CTRL,
						ENABLE_PKT | REPEAT_PKT);
}

void it6263_bridge_mode_set(struct i2c_client *client_hdmi, const struct videomode *vm)
{
	struct it6263 *it6263 = i2c_get_clientdata(client_hdmi);
	struct regmap *regmap = it6263->hdmi_regmap;
	bool pclk_high = vm->pixelclock > 80000 ? true : false;

	regmap_write(regmap, HDMI_REG_HDMI_MODE,
				it6263->is_hdmi ? TX_HDMI_MODE : TX_DVI_MODE);

	dev_dbg(&it6263->hdmi_i2c->dev, "%s mode\n",
				it6263->is_hdmi ? "HDMI" : "DVI");

	/* setup AFE */
	regmap_write(regmap, HDMI_REG_AFE_DRV_CTRL, AFE_DRV_RST);
	if (pclk_high)
		regmap_write(regmap, HDMI_REG_AFE_XP_CTRL,
						AFE_XP_GAINBIT | AFE_XP_RESETB);
	else
		regmap_write(regmap, HDMI_REG_AFE_XP_CTRL,
						AFE_XP_ER0 | AFE_XP_RESETB);
	regmap_write(regmap, HDMI_REG_AFE_ISW_CTRL, 0x10);
	if (pclk_high)
		regmap_write(regmap, HDMI_REG_AFE_IP_CTRL,
						AFE_IP_GAINBIT | AFE_IP_RESETB);
	else
		regmap_write(regmap, HDMI_REG_AFE_IP_CTRL,
						AFE_IP_ER0 | AFE_IP_RESETB);
}

static int it6263_check_chipid(struct it6263 *it6263)
{
	struct device *dev = &it6263->hdmi_i2c->dev;
	u8 vendor_id[2], device_id[2];
	int ret;

	ret = regmap_bulk_read(it6263->hdmi_regmap, REG_VENDOR_ID(0),
				&vendor_id, 2);
	if (ret) {
		dev_err(dev, "regmap_bulk_read failed %d\n", ret);
		return ret;
	}

	if (vendor_id[0] != HDMI_VENDER_ID_LOW ||
	    vendor_id[1] != HDMI_VENDER_ID_HIGH) {
		dev_err(dev,
			"Invalid hdmi vendor id %02x %02x(expect 0x01 0xca)\n",
			vendor_id[0], vendor_id[1]);
		return -EINVAL;
	}

	ret = regmap_bulk_read(it6263->hdmi_regmap, REG_DEVICE_ID(0),
				&device_id, 2);
	if (ret) {
		dev_err(dev, "regmap_bulk_read failed %d\n", ret);
		return ret;
	}

	if (device_id[0] != HDMI_DEVICE_ID_LOW ||
	    device_id[1] != HDMI_DEVICE_ID_HIGH) {
		dev_err(dev,
			"Invalid hdmi device id %02x %02x(expect 0x13 0x76)\n",
			device_id[0], device_id[1]);
		return -EINVAL;
	}

	ret = regmap_bulk_read(it6263->lvds_regmap, REG_VENDOR_ID(0),
				&vendor_id, 2);
	if (ret) {
		dev_err(dev, "regmap_bulk_read failed %d\n", ret);
		return ret;
	}

	if (vendor_id[0] != LVDS_VENDER_ID_LOW ||
	    vendor_id[1] != LVDS_VENDER_ID_HIGH) {
		dev_err(dev,
			"Invalid lvds vendor id %02x %02x(expect 0x15 0xca)\n",
			vendor_id[0], vendor_id[1]);
		return -EINVAL;
	}

	ret = regmap_bulk_read(it6263->lvds_regmap, REG_DEVICE_ID(0),
				&device_id, 2);
	if (ret) {
		dev_err(dev, "regmap_bulk_read failed %d\n", ret);
		return ret;
	}

	if (device_id[0] != LVDS_DEVICE_ID_LOW ||
	    device_id[1] != LVDS_DEVICE_ID_HIGH) {
		dev_err(dev,
			"Invalid lvds device id %02x %02x(expect 0x61 0x62)\n",
			device_id[0], device_id[1]);
		return -EINVAL;
	}

	return ret;
}

int it6263_probe(struct i2c_client *client_hdmi,
	struct i2c_client *client_lvds, bool split_mode,
	u32 ldb_bus_format, bool reset)
{
	struct device *dev = &client_hdmi->dev;
	struct device_node *np = &dev->of_node;
	struct it6263 *it6263;
	int ret;

	if (!client_hdmi || !client_lvds) {
		return -ENODEV;
	}
	it6263 = devm_kzalloc(dev, sizeof(*it6263), GFP_KERNEL);
	if (!it6263)
		return -ENOMEM;

	it6263->split_mode = split_mode;
	/* Todo: parameter is taken from CEA extensions of read edid data.
	   Set to false to comply both DVI and HDMI monitors */
	it6263->is_hdmi = false;
	it6263->ldb_bus_format = ldb_bus_format;

	it6263->hdmi_i2c = client_hdmi;
	it6263->lvds_i2c = client_lvds;

	i2c_set_clientdata(client_hdmi, it6263);

	it6263->hdmi_regmap = devm_regmap_init_i2c(it6263->hdmi_i2c);
	if (IS_ERR(it6263->hdmi_regmap)) {
		ret = PTR_ERR(it6263->hdmi_regmap);
		it6263->hdmi_regmap = NULL;
		return ret;
	}

	it6263->lvds_regmap = devm_regmap_init_i2c(it6263->lvds_i2c);
	if (IS_ERR(it6263->lvds_regmap)) {
		ret = PTR_ERR(it6263->lvds_regmap);
		goto unregister_hdmi_i2c;
	}

	if (reset) {
		ret = regmap_write(it6263->hdmi_regmap, HDMI_REG_SW_RST, HDMI_RST_ALL);
		if (ret)
			goto unregister_lvds_i2c;

		usleep_range(1000, 2000);
	}

	ret = regmap_write(it6263->hdmi_regmap, HDMI_REG_LVDS_PORT,
				LVDS_INPUT_CTRL_I2C_ADDR << 1);
	if (ret)
		goto unregister_lvds_i2c;

	ret = regmap_write(it6263->hdmi_regmap, HDMI_REG_LVDS_PORT_EN, 0x01);
	if (ret)
		goto unregister_lvds_i2c;

	/* select HDMI bank0 */
	ret = regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL,
				BANK_SEL(0));
	if (ret)
		goto unregister_lvds_i2c;

	ret = it6263_check_chipid(it6263);
	if (ret)
		goto unregister_lvds_i2c;

	it6263_lvds_config(it6263, reset);
	it6263_hdmi_config(it6263);

	return ret;

unregister_lvds_i2c:
	regmap_release_i2c(it6263->lvds_regmap);
	it6263->lvds_regmap = NULL;
unregister_hdmi_i2c:
	regmap_release_i2c(it6263->hdmi_regmap);
	it6263->hdmi_regmap = NULL;
	return ret;
}

int it6263_remove(struct i2c_client *client_hdmi)
{
	struct it6263 *it6263 = i2c_get_clientdata(client_hdmi);

	if (it6263) {
		if (it6263->lvds_regmap) {
			regmap_release_i2c(it6263->lvds_regmap);
		}
		if (it6263->hdmi_regmap) {
			regmap_release_i2c(it6263->hdmi_regmap);
		}
		devm_kfree(&client_hdmi->dev, it6263);
	}
	return 0;
}

#ifdef IT6263_DEBUG
/**
  Dump IT6263 registers.
**/
#include "wdm.h"

void it6263_dumpregs(struct i2c_client *client_hdmi)
{
    struct it6263 *it6263 = i2c_get_clientdata(client_hdmi);
    unsigned int status = 0;

    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "------------------------IT6263-------------------------------------\n");
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "----------------HDMI register common bank--------------------------\n");
    regmap_read(it6263->hdmi_regmap, HDMI_REG_SW_RST, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_SW_RST              = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_INT_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_INT_CTRL            = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_INT_STAT(1), &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_INT_STAT(1)         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_INT_STAT(2), &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_INT_STAT(2)         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_INT_STAT(3), &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_INT_STAT(3)         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_INT_MASK(1), &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_INT_MASK(1)         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_INT_MASK(2), &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_INT_MASK(2)         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_INT_MASK(3), &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_INT_MASK(3)         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_SYS_STATUS, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_SYS_STATUS          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_DDC_MASTER_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_DDC_MASTER_CTRL     = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_DDC_HEADER, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_DDC_HEADER          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_DDC_REQOFF, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_DDC_REQOFF          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_DDC_REQCOUNT, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_DDC_REQCOUNT        = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_DDC_EDIDSEG, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_DDC_EDIDSEG         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_DDC_CMD, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_DDC_CMD             = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_DDC_STATUS, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_DDC_STATUS          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_ROM_STAT, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_ROM_STAT            = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_LVDS_PORT, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_LVDS_PORT           = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_LVDS_PORT_EN, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_LVDS_PORT_EN        = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_BANK_CTRL           = 0x%08X\n", status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "----------------HDMI register bank 0 ------------------------------\n");
    regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, BANK_SEL(0));
    regmap_read(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_BANK_CTRL           = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_CLK_CTRL1, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_CLK_CTRL1           = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_AFE_DRV_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_AFE_DRV_CTRL        = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_AFE_XP_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_AFE_XP_CTRL         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_AFE_ISW_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_AFE_ISW_CTRL        = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_AFE_IP_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_AFE_IP_CTRL         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_INPUT_MODE, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_INPUT_MODE          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_TXFIFO_RST, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_TXFIFO_RST          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_B0, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_B0                  = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_HDMI_MODE, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_HDMI_MODE           = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_GCP, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_GCP                 = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_OESS_CYCLE, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_OESS_CYCLE          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_ENCRYPTION, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_ENCRYPTION          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_PKT_SINGLE_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_PKT_SINGLE_CTRL     = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_PKT_GENERAL_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_PKT_GENERAL_CTRL    = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_NULL_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_NULL_CTRL           = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_ACP_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_ACP_CTRL            = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_ISRC1_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_ISRC1_CTRL          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_ISRC2_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_ISRC2_CTRL          = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_AVI_INFOFRM_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_AVI_INFOFRM_CTRL    = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_AUD_INFOFRM_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_AUD_INFOFRM_CTRL    = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_SPD_INFOFRM_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_SPD_INFOFRM_CTRL    = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_MPG_INFOFRM_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_MPG_INFOFRM_CTRL    = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, 0x65, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x65                         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, 0x66, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x66                         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, 0x67, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x67                         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, 0xE1, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xE1                         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, 0xF3, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0xF3                         = 0x%08X\n", status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "----------------HDMI register bank 1-------------------------------\n");
    regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, BANK_SEL(1));
    regmap_read(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_BANK_CTRL           = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, HDMI_REG_AVI_DB1, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_AVI_DB1             = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, 0x33, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x33                         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, 0x34, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x34                         = 0x%08X\n", status);
    regmap_read(it6263->hdmi_regmap, 0x35, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x35                         = 0x%08X\n", status);
    regmap_write(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, BANK_SEL(0));
    regmap_read(it6263->hdmi_regmap, HDMI_REG_BANK_CTRL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "HDMI_REG_BANK_CTRL           = 0x%08X\n", status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "----------------LVDS   register -----------------------------------\n");
    regmap_read(it6263->lvds_regmap, LVDS_REG_SW_RST, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_SW_RST              = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_MODE, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_MODE                = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_STABLE, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_STABLE              = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_39, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_39                  = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_PLL, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_PLL                 = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_AFE_3E, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_AFE_3E              = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_AFE_3F, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_AFE_3F              = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_AFE_47, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_AFE_47              = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_AFE_48, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_AFE_48              = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_AFE_4F, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_AFE_4F              = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_52, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_52                  = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_PCLK_CNT_HIGH, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_PCLK_CNT_HIGH       = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, LVDS_REG_PCLK_CNT_LOW, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "LVDS_REG_PCLK_CNT_LOW        = 0x%08X\n", status);
    regmap_read(it6263->lvds_regmap, 0x0B, &status);
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "0x0B                         = 0x%08X\n", status);
}
#endif
