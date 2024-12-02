/*
 * Copyright 2016-2019, 2023-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <Base.h>
#include <stddef.h>
#include <Uefi/UefiBaseType.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <iMXDisplay.h>
#include "iMX8.h"
#include "svc/types.h"
#include "svc/pm/pm_api.h"
#include "svc/misc/misc_api.h"

#include "mipi_dsi_northwest_regs.h"
#include "mipi_dsi_northwest.h"
#include "mipi_dsi_imx8x.h"
#include "MipiDsi_packet.h"

#define DSIIMX8X_DEBUG_LEVEL DEBUG_INFO
/* #define TX_DEBUG */
#define NWL_CLK_DEBUG

#define MIPI_TX_ESCAPE_CLOCK_FREQ    18000000U
#define MIPI_RX_ESCAPE_CLOCK_FREQ    72000000U
#define MIPI_DSI_MAX_HS_BIT_CLOCK    1500000000U
#define MIPI_DSI_MIN_HS_BIT_CLOCK    80000000U

#define MIPI_LCD_SLEEP_MODE_DELAY	(120)
#define MIPI_FIFO_TIMEOUT		500000 /* 500ms */
#define	PS2KHZ(ps)	(1000000000UL / (ps))

#define DIV_ROUND_CLOSEST_ULL(x, divisor)(		\
{							\
	typeof(divisor) __d = divisor;			\
	unsigned long long _tmp = (x) + (__d) / 2;	\
	_tmp = _tmp / __d;				\
	_tmp;						\
}							\
)

#define DIV_ROUND_UP_ULL(x, divisor) (		\
{							\
	typeof(divisor) __d = divisor;			\
	unsigned long long _tmp = (unsigned long long)(x) + (__d) - 1;	\
	_tmp = _tmp / __d;				\
	_tmp;						\
}							\
)

enum mipi_dsi_mode {
	DSI_COMMAND_MODE,
	DSI_VIDEO_MODE
};

#define DSI_LP_MODE	0
#define DSI_HS_MODE	1

enum mipi_dsi_payload {
	DSI_PAYLOAD_CMD,
	DSI_PAYLOAD_VIDEO,
};

struct mipi_dsi_northwest_info {
	uint32_t mmio_base;
	sc_rsrc_t resource;
	sc_rsrc_t lvds_resource;
	uint32_t mmio_csr_base;
	IMX_DISPLAY_TIMING timings;
	struct nwl_dsi_platform_ops *plat_ops;
	enum dsi_pixel_format format;
	uint32_t max_data_lanes;
	uint32_t max_data_rate;
	uint32_t pll_ref;
};

static struct mipi_dsi_northwest_info mipi_dsi_dev = {0};

struct nwl_dsi_platform_ops {
	EFI_STATUS (*pclk_reset)(struct mipi_dsi_northwest_info *mipi, bool reset);
	EFI_STATUS (*mipi_reset)(struct mipi_dsi_northwest_info *mipi, bool reset);
	EFI_STATUS (*dpi_reset)(struct mipi_dsi_northwest_info *mipi, bool reset);
	EFI_STATUS (*config)(struct mipi_dsi_northwest_info *mipi);
	EFI_STATUS (*clock_enable)(struct mipi_dsi_northwest_info *mipi);
	EFI_STATUS (*clock_disable)(struct mipi_dsi_northwest_info *mipi);
};

struct pll_divider {
	unsigned int cm;  /* mult */
	unsigned int cn;  /* pre-div */
	unsigned int co;  /* output div */
};

/**
 * multiplier value to 'CM' register value
 * 'CM' = [16, 255];
 */
static unsigned int cm_map_table[240] = {
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,	/* 16 ~ 23 */
	0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,	/* 24 ~ 31 */

	0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,	/* 32 ~ 39 */
	0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, /* 40 ~ 47 */

	0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, /* 48 ~ 55 */
	0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, /* 56 ~ 63 */

	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, /* 64 ~ 71 */
	0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, /* 72 ~ 79 */

	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, /* 80 ~ 87 */
	0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, /* 88 ~ 95 */

	0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, /* 96  ~ 103 */
	0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, /* 104 ~ 111 */

	0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, /* 112 ~ 119 */
	0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, /* 120 ~ 127 */

	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, /* 128 ~ 135 */
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, /* 136 ~ 143 */

	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, /* 144 ~ 151 */
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, /* 152 ~ 159 */

	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, /* 160 ~ 167 */
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, /* 168 ~ 175 */

	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 176 ~ 183 */
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, /* 184 ~ 191 */

	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, /* 192 ~ 199 */
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, /* 200 ~ 207 */

	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, /* 208 ~ 215 */
	0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, /* 216 ~ 223 */

	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, /* 224 ~ 231 */
	0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, /* 232 ~ 239 */

	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, /* 240 ~ 247 */
	0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f	/* 248 ~ 255 */
};

/**
 * pre-divider value to 'CN' reigister value
 * 'CN' = [1, 32];
 */
static unsigned int cn_map_table[32] = {
	0x1f, 0x00, 0x10, 0x18, 0x1c, 0x0e, 0x07, 0x13,	/* 1  ~ 8  */
	0x09, 0x04, 0x02, 0x11, 0x08, 0x14, 0x0a, 0x15,	/* 9  ~ 16 */
	0x1a, 0x1d, 0x1e, 0x0f, 0x17, 0x1b, 0x0d, 0x16,	/* 17 ~ 24 */
	0x0b, 0x05, 0x12, 0x19, 0x0c, 0x06, 0x03, 0x01	/* 25 ~ 32 */
};

/**
 * output divider value to 'CO' reigister value
 * 'CO' = { 1, 2, 4, 8 };
 */
static unsigned int co_map_table[4] = {
	0x0, 0x1, 0x2, 0x3
};

unsigned long gcd(unsigned long a, unsigned long b)
{
	unsigned long r = a | b;
	unsigned long tmp;

	if (!a || !b) {
		return r;
	}

	/* Isolate lsbit of r */
	r &= -r;

	while (!(b & r)) {
		b >>= 1;
	}
	if (b == r) {
		return r;
	}

	for (;;) {
		while (!(a & r)) {
			a >>= 1;
		}
		if (a == r) {
			return r;
		}
		if (a == b) {
			return a;
		}

		if (a < b) {
			tmp = a;
			a = b;
			b = tmp;
		}
		a -= b;
		a >>= 1;
		if (a & r)
			a += b;
		a >>= 1;
	}
}

static int mipi_dsi_pixel_format_to_bpp(enum dsi_pixel_format fmt)
{
	switch (fmt) {
	case DSI_FMT_RGB888:
	case DSI_FMT_RGB666:
		return 24;

	case DSI_FMT_RGB666_PACKED:
		return 18;

	case DSI_FMT_RGB565:
		return 16;
	}

	return -1;
}

static void mipi_dsi_set_mode(struct mipi_dsi_northwest_info *mipi_dsi,
			      uint8_t mode)
{
	switch (mode) {
	case DSI_LP_MODE:
		MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK, 0x1);
		MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_AUTOINSERT_EOTP, 0x1);
		break;
	case DSI_HS_MODE:
		MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK, 0x0);
		MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_AUTOINSERT_EOTP, 0x0);
		break;
	default:
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: invalid dsi mode %d\n", mode));
		return;
	}

	MicroSecondDelay(1000);
}

static EFI_STATUS mipi_dsi_dphy_init(struct mipi_dsi_northwest_info *mipi_dsi)
{
	uint32_t time_out = 100;
	uint32_t lock;
	uint32_t req_bit_clk;
	uint32_t bpp;

	int i, best_div = -1;
	int64_t delta;
	uint64_t least_delta = ~0U;
	uint64_t limit, div_result;
	uint64_t denominator, numerator, divisor, tmp;
	uint32_t tmp1, tmp2;
	uint64_t norm_denom, norm_num, split_denom;
	struct pll_divider div = { 0 };

	if (mipi_dsi->timings.PixelClock == 0) {
		return EFI_INVALID_PARAMETER;
	}

	bpp = mipi_dsi_pixel_format_to_bpp(mipi_dsi->format);

	/* req_bit_clk is PLL out, clk_byte is 1/8th of the req_bit_clk
	*  We need meet clk_byte_freq >= dpi_pclk_freq * DPI_pixel_size / ( 8 * (cfg_num_lanes + 1))
	*/

	req_bit_clk = mipi_dsi->timings.PixelClock;
	req_bit_clk = req_bit_clk * bpp;

	switch (mipi_dsi->max_data_lanes) {
	case 1:
		break;
	case 2:
		req_bit_clk = req_bit_clk >> 1;
		break;
	case 4:
		req_bit_clk = req_bit_clk >> 2;
		break;
	default:
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: requested data lane num is invalid: %d\n", mipi_dsi->max_data_lanes));
		return EFI_INVALID_PARAMETER;
	}

	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: req_bit_clk %u\n", req_bit_clk));

	/* The max rate for PLL out is 800Mhz */
	if (req_bit_clk > mipi_dsi->max_data_rate)
		return EFI_INVALID_PARAMETER;

	/* calc CM, CN and CO according to PHY PLL formula:
	 *
	 * 'PLL out bitclk = refclk * CM / (CN * CO);'
	 *
	 * Let:
	 * 'numerator   = bitclk / divisor';
	 * 'denominator = refclk / divisor';
	 * Then:
	 * 'numerator / denominator = CM / (CN * CO)';
	 *
	 * CM is in [16, 255]
	 * CN is in [1, 32]
	 * CO is in { 1, 2, 4, 8 };
	 */
	divisor = gcd(mipi_dsi->pll_ref, req_bit_clk);

	div_result = req_bit_clk;
	div_result = div_result / divisor;
	numerator = div_result;

	div_result = mipi_dsi->pll_ref;
	div_result = div_result / divisor;
	denominator = div_result;

	/* denominator & numerator out of range check */
	if (DIV_ROUND_CLOSEST_ULL(numerator, denominator) > 255 ||
	    DIV_ROUND_CLOSEST_ULL(denominator, numerator) > 32 * 8)
		return EFI_INVALID_PARAMETER;

	/* Normalization: reduce or increase
	 * numerator	to [16, 255]
	 * denominator	to [1, 32 * 8]
	 * Reduce normalization result is 'approximiate'
	 * Increase nomralization result is 'precise'
	 */
	if (numerator > 255 || denominator > 32 * 8) {
		/* approximate */
		if (numerator > denominator) {
			/* 'numerator > 255';
			 * 'limit' should meet below conditions:
			 *  a. '(numerator   / limit) >= 16'
			 *  b. '(denominator / limit) >= 1'
			 */
			tmp = DIV_ROUND_CLOSEST_ULL(numerator, 16);
			limit = denominator < tmp ? denominator : tmp;

			/* Let:
			 * norm_num   = numerator   / i;
			 * norm_denom = denominator / i;
			 *
			 * So:
			 * delta = numerator * norm_denom -
			 * 	   denominator * norm_num
			 */
			for (i = 2; i <= limit; i++) {
				norm_num = DIV_ROUND_CLOSEST_ULL(numerator, i);
				if (norm_num > 255)
					continue;

				norm_denom = DIV_ROUND_CLOSEST_ULL(denominator, i);

				/* 'norm_num <= 255' && 'norm_num > norm_denom'
				 * so, 'norm_denom < 256'
				 */
				delta = numerator * norm_denom -
					denominator * norm_num;
				delta = ABS(delta);
				if (delta < least_delta) {
					least_delta = delta;
					best_div = i;
				} else if (delta == least_delta) {
					/* choose better one IF:
					 * 'norm_denom' derived from last 'best_div'
					 * needs later split, i.e, 'norm_denom > 32'.
					 */
					if (DIV_ROUND_CLOSEST_ULL(denominator, best_div) > 32) {
						least_delta = delta;
						best_div = i;
					}
				}
			}
		} else {
			/* 'denominator > 32 * 8';
			 * 'limit' should meet below conditions:
			 *  a. '(numerator   / limit >= 16'
			 *  b. '(denominator / limit >= 1': obviously.
			 */
			limit = DIV_ROUND_CLOSEST_ULL(numerator, 16);
			if (!limit ||
			    DIV_ROUND_CLOSEST_ULL(denominator, limit) > 32 * 8)
				return EFI_INVALID_PARAMETER;

			for (i = 2; i <= limit; i++) {
				norm_denom = DIV_ROUND_CLOSEST_ULL(denominator, i);
				if (norm_denom > 32 * 8)
					continue;

				norm_num = DIV_ROUND_CLOSEST_ULL(numerator, i);

				/* 'norm_denom <= 256' && 'norm_num < norm_denom'
				 * so, 'norm_num <= 255'
				 */
				delta = numerator * norm_denom -
					denominator * norm_num;
				delta = ABS(delta);
				if (delta < least_delta) {
					least_delta = delta;
					best_div = i;
				} else if (delta == least_delta) {
					if (DIV_ROUND_CLOSEST_ULL(denominator, best_div) > 32) {
						least_delta = delta;
						best_div = i;
					}
				}
			}
		}

		numerator   = DIV_ROUND_CLOSEST_ULL(numerator, best_div);
		denominator = DIV_ROUND_CLOSEST_ULL(denominator, best_div);
	} else if (numerator < 16) {
		/* precise */

		/* 'limit' should meet below conditions:
		 *  a. 'denominator * limit <= 32 * 8'
		 *  b. '16 <= numerator * limit <= 255'
		 *  Choose 'limit' to be the least value
		 *  which makes 'numerator * limit' to be
		 *  in [16, 255].
		 */
		tmp1 = 256 / (uint32_t)denominator;
		tmp2 = 255 / (uint32_t)numerator;
		limit = tmp1 < tmp2 ? tmp1 : tmp2;

		if (limit == 1 || limit < DIV_ROUND_UP_ULL(16, numerator))
			return EFI_INVALID_PARAMETER;

		/* choose the least available value for 'limit' */
		limit = DIV_ROUND_UP_ULL(16, numerator);
		numerator   = numerator * limit;
		denominator = denominator * limit;

		if(numerator < 16 || denominator > 32 * 8) {
			DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: WARNING (numerator < 16 || denominator > 32 * 8) is TRUE: num=%d denom=%d\n",
				 numerator, denominator));
		}
	}

	div.cm = cm_map_table[numerator - 16];

	/* split 'denominator' to 'CN' and 'CO' */
	if (denominator > 32) {
		/* traverse four possible values of 'CO'
		 * there must be some value of 'CO' can be used
		 */
		least_delta = ~0U;
		for (i = 0; i < 4; i++) {
			split_denom = DIV_ROUND_CLOSEST_ULL(denominator, 1 << i);
			if (split_denom > 32)
				continue;

			/* calc deviation to choose the best one */
			delta = denominator - split_denom * (1 << i);
			delta = ABS(delta);
			if (delta < least_delta) {
				least_delta = delta;
				div.co = co_map_table[i];
				div.cn = cn_map_table[split_denom - 1];
			}
		}
	} else {
		div.co = co_map_table[1 >> 1];
		div.cn = cn_map_table[denominator - 1];
	}

	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: DPHY config numerator=%d, denom=%d, cn 0x%x, cm 0x%x, co 0x%x\n",
		numerator, denominator, div.cn, div.cm, div.co));

	MmioWrite32(mipi_dsi->mmio_base + DPHY_PD_PLL, 0x1);
	MmioWrite32(mipi_dsi->mmio_base + DPHY_PD_DPHY, 0x1);

	if (req_bit_clk <= 250000000u) {
		MmioWrite32(mipi_dsi->mmio_base + DPHY_M_PRG_HS_PREPARE, 0x1);
	} else {
		MmioWrite32(mipi_dsi->mmio_base + DPHY_M_PRG_HS_PREPARE, 0x0);
	}
	MmioWrite32(mipi_dsi->mmio_base + DPHY_MC_PRG_HS_PREPARE, 0x0);
	{
		uint32_t n;
		/* hs_zero: formula from NXP BSP */
		n = (144 * (req_bit_clk / 1000000) - 47500) / 10000;
		MmioWrite32(mipi_dsi->mmio_base + DPHY_M_PRG_HS_ZERO, n < 1 ? 1 : (uint8_t)n);
		/* clk_zero: formula from NXP BSP */
		n = (34 * (req_bit_clk / 1000000) - 2500) / 1000;
		MmioWrite32(mipi_dsi->mmio_base + DPHY_MC_PRG_HS_ZERO, n < 1 ? 1 : (uint8_t)n);
		/* clk_trail, hs_trail: formula from NXP BSP */
		n = (103 * (req_bit_clk / 1000000) + 10000) / 10000;
		if (n > 15) {
			n = 15;
		}
		if (n < 1) {
			n = 1;
		}
		MmioWrite32(mipi_dsi->mmio_base + DPHY_M_PRG_HS_TRAIL, (uint8_t)n);
		MmioWrite32(mipi_dsi->mmio_base + DPHY_MC_PRG_HS_TRAIL, (uint8_t)n);
	}

	MmioWrite32(mipi_dsi->mmio_base + DPHY_LOCK_BYP, 0x0);

	MmioWrite32(mipi_dsi->mmio_base + DPHY_AUTO_PD_EN, 0x0);
	MmioWrite32(mipi_dsi->mmio_base + DPHY_RXLPRP, 0x2);
	MmioWrite32(mipi_dsi->mmio_base + DPHY_RXCDRP, 0x2);
	MmioWrite32(mipi_dsi->mmio_base + DPHY_TST, 0x25);
	
	MmioWrite32(mipi_dsi->mmio_base + DPHY_CN, div.cn);
	MmioWrite32(mipi_dsi->mmio_base + DPHY_CM, div.cm);
	MmioWrite32(mipi_dsi->mmio_base + DPHY_CO, div.co);

	MmioWrite32(mipi_dsi->mmio_base + DPHY_PD_PLL, 0x0);

	while (!(lock = MmioRead32(mipi_dsi->mmio_base + DPHY_LOCK))) {
		MicroSecondDelay(10);
		time_out--;
		if (time_out == 0) {
			DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: cannot get the dphy lock = 0x%x\n", lock));
			return EFI_TIMEOUT;
		}
	}
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: dphy lock = 0x%x timeout = %d us\n", lock, time_out*10));

	MmioWrite32(mipi_dsi->mmio_base + DPHY_PD_DPHY, 0x0);

	return EFI_SUCCESS;
}

static EFI_STATUS mipi_dsi_host_init(struct mipi_dsi_northwest_info *mipi_dsi)
{ 
	uint32_t lane_num;
	uint64_t n;
	IMX_DISPLAY_TIMING *timings = &(mipi_dsi->timings);
	int bpp = mipi_dsi_pixel_format_to_bpp(mipi_dsi->format);
	if (bpp < 0) {
		return EFI_INVALID_PARAMETER;
	}

	switch (mipi_dsi->max_data_lanes) {
	case 1:
		lane_num = 0x0;
		break;
	case 2:
		lane_num = 0x1;
		break;
	case 4:
		lane_num = 0x3;
		break;
	default:
		/* Invalid lane num */
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: Invalid lane num = %d\n", mipi_dsi->max_data_lanes));
		return EFI_INVALID_PARAMETER;
	}

	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: HOST init: lanes=%d, reg=%d, pclk=%d\n",
		mipi_dsi->max_data_lanes, lane_num, timings->PixelClock));

	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_NUM_LANES, lane_num);
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK, 0x0);
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_T_PRE, 0x1);
	n = (360000ULL * (uint64_t)bpp * timings->PixelClock) / (mipi_dsi->max_data_lanes * 8ULL * 1000000000000ULL);
	n += 2;
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: HOST_CFG_T_POST = %d (0x%x)\n", n, n));
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_T_POST, n);
	n = (100000ULL * (uint64_t)bpp * timings->PixelClock) / (mipi_dsi->max_data_lanes * 8ULL * 1000000000000ULL);
	n += 1;
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: HOST_CFG_TX_GAP = %d (0x%x)\n", n, n));
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_TX_GAP, n);
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_AUTOINSERT_EOTP, 0x0);
	/*lx=1, other=0*/
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_EXTRA_CMDS_AFTER_EOTP, 0x1);
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_HTX_TO_COUNT, 0);
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_LRX_H_TO_COUNT, 0);
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_BTA_H_TO_COUNT, 0);
	/* Previously 0x3A98 */
	MmioWrite32(mipi_dsi->mmio_base + HOST_CFG_TWAKEUP, 0x4650);

	return EFI_SUCCESS;
}

static EFI_STATUS mipi_dsi_dpi_init(struct mipi_dsi_northwest_info *mipi_dsi)
{
	uint32_t pixel_fmt, hbp, vbp;
	enum mipi_dsi_dpi_fmt color_coding;
	int bpp;
	IMX_DISPLAY_TIMING *timings = &(mipi_dsi->timings);

	if (timings->PixelClock == 0) {
		return EFI_INVALID_PARAMETER;
	}

	bpp = mipi_dsi_pixel_format_to_bpp(mipi_dsi->format);
	if (bpp < 0) {
		return EFI_INVALID_PARAMETER;
	}

	MmioWrite32(mipi_dsi->mmio_base + DPI_PIXEL_PAYLOAD_SIZE, timings->HActive);
	MmioWrite32(mipi_dsi->mmio_base + DPI_PIXEL_FIFO_SEND_LEVEL, timings->HActive);

	switch (bpp) {
	case 24:
		color_coding = MIPI_RGB888;
		pixel_fmt = (uint32_t)mipi_dsi->format;
		break;
	case 16:
	case 18:
	default:
		/* Not supported */
		return EFI_INVALID_PARAMETER;
	}
	
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: DPI init - color_coding = %d, pixel_fmt = %d\n", color_coding, pixel_fmt));

	MmioWrite32(mipi_dsi->mmio_base + DPI_INTERFACE_COLOR_CODING, (uint32_t)color_coding);
	MmioWrite32(mipi_dsi->mmio_base + DPI_PIXEL_FORMAT, pixel_fmt);
	MmioWrite32(mipi_dsi->mmio_base + DPI_VSYNC_POLARITY, 0x0);
	MmioWrite32(mipi_dsi->mmio_base + DPI_HSYNC_POLARITY, 0x0);
	MmioWrite32(mipi_dsi->mmio_base + DPI_VIDEO_MODE, DPI_VIDEO_MODE_NONBURST_SYNC_PULSES);

	MmioWrite32(mipi_dsi->mmio_base + DPI_HFP, timings->HSyncOffset);
	hbp = timings->HBlank - timings->HSyncOffset - timings->HSync;
	MmioWrite32(mipi_dsi->mmio_base + DPI_HBP, hbp);
	MmioWrite32(mipi_dsi->mmio_base + DPI_HSA, timings->HSync);
	MmioWrite32(mipi_dsi->mmio_base + DPI_ENABLE_MULT_PKTS, 0x0);

	vbp = timings->VBlank - timings->VSyncOffset - timings->VSync;
	MmioWrite32(mipi_dsi->mmio_base + DPI_VBP, vbp);
	MmioWrite32(mipi_dsi->mmio_base + DPI_VFP, timings->VSyncOffset);
	MmioWrite32(mipi_dsi->mmio_base + DPI_BLLP_MODE, 0x1);
	MmioWrite32(mipi_dsi->mmio_base + DPI_USE_NULL_PKT_BLLP, 0x0);

	MmioWrite32(mipi_dsi->mmio_base + DPI_VACTIVE, timings->VActive - 1);

	MmioWrite32(mipi_dsi->mmio_base + DPI_VC, 0x0);

	return EFI_SUCCESS;
}

static void mipi_dsi_init_interrupt(struct mipi_dsi_northwest_info *mipi_dsi)
{
	/* disable all the irqs */
	MmioWrite32(mipi_dsi->mmio_base + HOST_IRQ_MASK, 0xffffffff);
	MmioWrite32(mipi_dsi->mmio_base + HOST_IRQ_MASK2, 0x7);
}

static void mipi_dsi_wr_tx_header(struct mipi_dsi_northwest_info *mipi_dsi,
				  uint8_t di, uint8_t data0, uint8_t data1, uint8_t mode, uint8_t need_bta)
{
	uint32_t pkt_control = 0;
	uint16_t word_count = 0;

	word_count = data0 | (data1 << 8);
	pkt_control = HOST_PKT_CONTROL_WC(word_count) |
		      HOST_PKT_CONTROL_VC(0)	      |
		      HOST_PKT_CONTROL_DT(di)	      |
		      HOST_PKT_CONTROL_HS_SEL(mode)   |
		      HOST_PKT_CONTROL_BTA_TX(need_bta);
#ifdef TX_DEBUG
	DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: pkt_control = 0x%x adr=0x%x\n",
		pkt_control, mipi_dsi->mmio_base + HOST_PKT_CONTROL));
#endif
	MmioWrite32(mipi_dsi->mmio_base + HOST_PKT_CONTROL, pkt_control);
}

static void mipi_dsi_wr_tx_data(struct mipi_dsi_northwest_info *mipi_dsi,
				uint32_t tx_data)
{
	MmioWrite32(mipi_dsi->mmio_base + HOST_TX_PAYLOAD, tx_data);
}

static void mipi_dsi_long_data_wr(struct mipi_dsi_northwest_info *mipi_dsi,
			const uint8_t *data0, uint32_t data_size)
{
	uint32_t data_cnt = 0, payload = 0;

	for (data_cnt = 0; data_cnt < data_size; data_cnt += 4) {
		if ((data_size - data_cnt) < 4) {
			if ((data_size - data_cnt) == 3) {
				payload = data0[data_cnt] |
					  (data0[data_cnt + 1] << 8) |
					  (data0[data_cnt + 2] << 16);
				DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: count = 3 payload = %x, %x %x %x\n",
					payload, data0[data_cnt], data0[data_cnt + 1], data0[data_cnt + 2]));
			} else if ((data_size - data_cnt) == 2) {
				payload = data0[data_cnt] |
					  (data0[data_cnt + 1] << 8);
				DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: count = 2 payload = %x, %x %x\n",
					payload, data0[data_cnt], data0[data_cnt + 1]));
			} else if ((data_size - data_cnt) == 1) {
				payload = data0[data_cnt];
				DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: count = 1 payload = %x, %x\n",
					payload, data0[data_cnt]));
			}

			mipi_dsi_wr_tx_data(mipi_dsi, payload);
		} else {
			payload = data0[data_cnt] |
				  (data0[data_cnt + 1] << 8) |
				  (data0[data_cnt + 2] << 16) |
				  (data0[data_cnt + 3] << 24);

			DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: count = 4 payload = %x, %x %x %x %x\n",
				payload, *(uint8_t *)(data0 + data_cnt),
				data0[data_cnt + 1], data0[data_cnt + 2],
				data0[data_cnt + 3]));

			mipi_dsi_wr_tx_data(mipi_dsi, payload);
		}
	}
}

static EFI_STATUS wait_for_pkt_done(struct mipi_dsi_northwest_info *mipi_dsi, unsigned long timeout)
{
	uint32_t irq_status, pkt_status;

	do {
		pkt_status = MmioRead32(mipi_dsi->mmio_base + HOST_PKT_STATUS);
		if (!(pkt_status & HOST_IRQ_STATUS_SM_NOT_IDLE)) {
			break;
		}

		MicroSecondDelay(1);
	} while (--timeout);

	while(timeout--) {
		irq_status = MmioRead32(mipi_dsi->mmio_base + HOST_IRQ_STATUS);
		if (irq_status & HOST_IRQ_STATUS_TX_PKT_DONE) {
			return EFI_SUCCESS;
		}

		MicroSecondDelay(1);
	}

	return EFI_TIMEOUT;
}

static EFI_STATUS mipi_dsi_pkt_write(struct mipi_dsi_northwest_info *mipi_dsi,
			uint8_t data_type, const uint8_t *buf, int len)
{
	EFI_STATUS ret = EFI_SUCCESS;
	const uint8_t *data = (const uint8_t *)buf;

	if (len == 0) {
		/* handle short write command */
		mipi_dsi_wr_tx_header(mipi_dsi, data_type, data[0], data[1], DSI_LP_MODE, 0);
	} else {
		/* handle generic long write command */
		mipi_dsi_long_data_wr(mipi_dsi, data, len);
		mipi_dsi_wr_tx_header(mipi_dsi, data_type, len & 0xff,
				      (len & 0xff00) >> 8, DSI_LP_MODE, 0);
	}

	/* send packet */
	MmioWrite32(mipi_dsi->mmio_base + HOST_SEND_PACKET, 0x1);
	ret = wait_for_pkt_done(mipi_dsi, MIPI_FIFO_TIMEOUT);

	if (ret == EFI_TIMEOUT) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: wait tx done timeout!\n"));
	}

	return ret;
}

static EFI_STATUS reset_dpi_domain(struct mipi_dsi_northwest_info *mipi_dsi, bool reset)
{
	EFI_STATUS ret = EFI_SUCCESS;

	if (mipi_dsi->plat_ops->dpi_reset) {
		ret = mipi_dsi->plat_ops->dpi_reset(mipi_dsi, reset);
	}

	return ret;
}

static EFI_STATUS reset_mipi_domain(struct mipi_dsi_northwest_info *mipi_dsi, bool reset)
{
	EFI_STATUS ret = EFI_SUCCESS;

	if (mipi_dsi->plat_ops->mipi_reset) {
		ret = mipi_dsi->plat_ops->mipi_reset(mipi_dsi, reset);
	}

	return ret;
}

static EFI_STATUS reset_pclk_domain(struct mipi_dsi_northwest_info *mipi_dsi, bool reset)
{
	EFI_STATUS ret = EFI_SUCCESS;

	if (mipi_dsi->plat_ops->pclk_reset) {
		ret = mipi_dsi->plat_ops->pclk_reset(mipi_dsi, reset);
	}

	return ret;
}

static void mipi_dsi_shutdown(struct mipi_dsi_northwest_info *mipi_dsi)
{

	MmioWrite32(mipi_dsi->mmio_base + DPHY_PD_PLL, 0x1);
	MmioWrite32(mipi_dsi->mmio_base + DPHY_PD_DPHY, 0x1);

	if (mipi_dsi->plat_ops->clock_disable) {
		(void)mipi_dsi->plat_ops->clock_disable(mipi_dsi);
	}

	reset_mipi_domain(mipi_dsi, true);
	reset_dpi_domain(mipi_dsi, true);
	reset_pclk_domain(mipi_dsi, true);
}

EFI_STATUS mipi_dsi_northwest_host_attach(VOID)
{
	struct mipi_dsi_northwest_info *mipi_dsi = &mipi_dsi_dev;
	EFI_STATUS ret;

	/* Assert resets */
	ret = reset_pclk_domain(mipi_dsi, true);
	if (ret != EFI_SUCCESS) {
		return ret;
	}
	ret = reset_mipi_domain(mipi_dsi, true);
	if (ret != EFI_SUCCESS) {
		return ret;
	}
	ret = reset_dpi_domain(mipi_dsi, true);
	if (ret != EFI_SUCCESS) {
		return ret;
	}

	/* Enable mipi relevant clocks */
	if (mipi_dsi->plat_ops->clock_enable) {
		ret = mipi_dsi->plat_ops->clock_enable(mipi_dsi);
		if (ret != EFI_SUCCESS) {
			return ret;
		}
	}

	/* Deasser pclk reset */
	ret = reset_pclk_domain(mipi_dsi, false);
	if (ret != EFI_SUCCESS) {
		return ret;
	}
	/* Disable all interrupts, since we use polling */
	mipi_dsi_init_interrupt(mipi_dsi);

	/* Platform specific config */
	if (mipi_dsi->plat_ops->config) {
		ret = mipi_dsi->plat_ops->config(mipi_dsi);
		if (ret != EFI_SUCCESS) {
			return ret;
		}
	}

	ret = mipi_dsi_host_init(mipi_dsi);
	if (ret != EFI_SUCCESS) {
		return ret;
	}

	ret = mipi_dsi_dpi_init(mipi_dsi);
	if (ret != EFI_SUCCESS) {
		return ret;
	}

	ret = mipi_dsi_dphy_init(mipi_dsi);
	if (ret != EFI_SUCCESS) {
		return ret;
	}

	/* Deassert resets */
	ret = reset_mipi_domain(mipi_dsi, false);
	if (ret != EFI_SUCCESS) {
		return ret;
	}

	return EFI_SUCCESS;
}

EFI_STATUS mipi_dsi_northwest_host_dpi_rst_deassert(VOID)
{
	struct mipi_dsi_northwest_info *mipi_dsi = &mipi_dsi_dev;
	EFI_STATUS ret;

	/* Deassert resets */
	ret = reset_dpi_domain(mipi_dsi, false);
	if (ret != EFI_SUCCESS) {
		return ret;
	}

	return EFI_SUCCESS;
}

EFI_STATUS mipi_dsi_northwest_host_transfer(uint8_t Type, uint8_t Chan, uint32_t Flg, const void *Data, uint16_t Size)
{
	struct mipi_dsi_northwest_info *dsi = &mipi_dsi_dev;

#ifdef TX_DEBUG
	uint16_t i = 0;
	uint8_t *p = (uint8_t *)Data;

	DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: sec_mipi_dsi_host_transfer: "));
	for (i = 0; i < Size; i++) {
		DEBUG((DEBUG_ERROR, "0x%.2x ", *(uint8_t *)p));
		p++;
	}
	DEBUG((DEBUG_ERROR, "\n"));
#endif

	if (MipiDsi_IsLong(Type)) {
		return mipi_dsi_pkt_write(dsi, Type, (const uint8_t *)Data, (int)Size);
	} else {
		return mipi_dsi_pkt_write(dsi, Type, (const uint8_t *)Data, 0);
	}
}

static EFI_STATUS imx8x_dsi_mipi_reset(struct mipi_dsi_northwest_info *mipi, bool reset)
{
	EFI_STATUS err;
	uint32_t reset_value = reset ? 0 : 1;

	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: imx8x_dsi_mipi_reset = %d reset_value = %d\n", reset, reset_value));

	err = sc_misc_set_control(SC_IPC_HDL, mipi->resource, SC_C_PHY_RESET, reset_value);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: sc_misc_set_control SC_C_PHY_RESET failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}
	err = sc_misc_set_control(SC_IPC_HDL, mipi->resource, SC_C_MIPI_RESET, reset_value);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: sc_misc_set_control SC_C_MIPI_RESET failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFI_STATUS imx8x_dsi_dpi_reset(struct mipi_dsi_northwest_info *mipi, bool reset)
{
	EFI_STATUS err;
	uint32_t reset_value = reset ? 0 : 1;

	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: imx8x_dsi_dpi_reset = %d reset_value = %d\n", reset, reset_value));

	err = sc_misc_set_control(SC_IPC_HDL, mipi->resource, SC_C_DPI_RESET, reset_value);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: sc_misc_set_control SC_C_MIPI_RESET failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFI_STATUS imx8x_dsi_pclk_reset(struct mipi_dsi_northwest_info *mipi, bool reset)
{
	EFI_STATUS err;
	sc_rsrc_t DcResource = SC_R_DC_0;
	uint32_t dsi_reset_value = reset ? 1 : 0;
	uint32_t pxl_reset_value = reset ? 0 : 1;
	sc_ctrl_t LinkValid = (mipi->resource == SC_R_MIPI_1) ? SC_C_PXL_LINK_MST2_VLD : SC_C_PXL_LINK_MST1_VLD;
	sc_ctrl_t Sync = (mipi->resource == SC_R_MIPI_1) ? SC_C_SYNC_CTRL1 : SC_C_SYNC_CTRL0;

	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: imx8x_dsi_pclk_reset rsrc=%d reset=%d\n", mipi->resource, reset));

	err = sc_misc_set_control(SC_IPC_HDL, mipi->resource, SC_C_MODE, dsi_reset_value);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: sc_misc_set_control SC_C_MODE failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_misc_set_control(SC_IPC_HDL, mipi->resource, SC_C_DUAL_MODE, dsi_reset_value);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: sc_misc_set_control SC_C_DUAL_MODE failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_misc_set_control(SC_IPC_HDL, mipi->resource, SC_C_PXL_LINK_SEL, dsi_reset_value);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: sc_misc_set_control SC_C_PXL_LINK_SEL failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_misc_set_control(SC_IPC_HDL, DcResource, LinkValid, pxl_reset_value);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: MSTx_VLD failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_misc_set_control(SC_IPC_HDL, DcResource, Sync, pxl_reset_value);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: SYNC_CTRLx failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFI_STATUS imx8x_dsi_clock_enable(struct mipi_dsi_northwest_info *mipi)
{
	EFI_STATUS err;
	uint32_t PixelClock = mipi->timings.PixelClock;
	uint32_t RefClock = mipi->pll_ref;
	uint32_t Rate;

	if ((PixelClock == 0) || (RefClock == 0)) {
		return EFI_INVALID_PARAMETER;
	}

	err = sc_pm_set_clock_parent(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PHY, 2);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: set parent SC_PM_CLK_PHY failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}
	
	err = sc_pm_set_clock_parent(SC_IPC_HDL, mipi->resource, SC_PM_CLK_MST_BUS, 2);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: set parent SC_PM_CLK_MST_BUS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}
	err = sc_pm_set_clock_parent(SC_IPC_HDL, mipi->resource, SC_PM_CLK_SLV_BUS, 2);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: set parent SC_PM_CLK_SLV_BUS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_set_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PHY, &RefClock);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW:set rate SC_PM_CLK_PHY failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	Rate = MIPI_TX_ESCAPE_CLOCK_FREQ;
	err = sc_pm_set_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_MST_BUS, &Rate);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW:set rate SC_PM_CLK_MST_BUS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	Rate = MIPI_RX_ESCAPE_CLOCK_FREQ;
	err = sc_pm_set_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_SLV_BUS, &Rate);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW:set rate SC_PM_CLK_SLV_BUS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_set_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_BYPASS, &PixelClock);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW:set rate SC_PM_CLK_BYPASS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_set_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PER, &PixelClock);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW:set rate SC_PM_CLK_PER failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_BYPASS, true, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: enable clock SC_PM_CLK_BYPASS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PER, true, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: enable clock SC_PM_CLK_PER failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_MST_BUS, true, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: enable clock SC_PM_CLK_MST_BUS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_SLV_BUS, true, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: enable clock SC_PM_CLK_SLV_BUS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PHY, true, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: enable clock SC_PM_CLK_PHY failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

#ifdef NWL_CLK_DEBUG
	err = sc_pm_get_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PER, &Rate);
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: SC_PM_CLK_PER rate = %d err=%d\n", Rate, err));
	err = sc_pm_get_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_BYPASS, &Rate);
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: SC_PM_CLK_BYPASS rate = %d err=%d\n", Rate, err));
	err = sc_pm_get_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_MST_BUS, &Rate);
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: SC_PM_CLK_MST_BUS rate = %d err=%d\n", Rate, err));
	err = sc_pm_get_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_SLV_BUS, &Rate);
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: SC_PM_CLK_SLV_BUS rate = %d err=%d\n", Rate, err));
	err = sc_pm_get_clock_rate(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PHY, &Rate);
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: SC_PM_CLK_PHY rate = %d err=%d\n", Rate, err));
#endif

	return EFI_SUCCESS;
}

static EFI_STATUS imx8x_dsi_clock_disable(struct mipi_dsi_northwest_info *mipi)
{
	EFI_STATUS err;


	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PHY, false, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: disable clock SC_PM_CLK_PHY failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}
	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_BYPASS, false, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: disable clock SC_PM_CLK_BYPASS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}
	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_PER, false, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: disable clock SC_PM_CLK_PER failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_MST_BUS, false, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: disable clock SC_PM_CLK_MST_BUS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}
	err = sc_pm_clock_enable(SC_IPC_HDL, mipi->resource, SC_PM_CLK_SLV_BUS, false, false);
	if (err) {
		DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: disable clock SC_PM_CLK_SLV_BUS failed! (error = %d)\n", err));
		return EFI_DEVICE_ERROR;
	}

	return EFI_SUCCESS;
}

static EFI_STATUS imx8x_config(struct mipi_dsi_northwest_info *mipi)
{
	enum mipi_dsi_dpi_fmt mode;

	switch (mipi->format) {
		case DSI_FMT_RGB888:
			mode = MIPI_RGB888;
			break;
		case DSI_FMT_RGB666:
			mode = MIPI_RGB666_LOOSELY;
			break;
		case DSI_FMT_RGB666_PACKED:
			mode = MIPI_RGB666_PACKED;
			break;
		case DSI_FMT_RGB565:
			mode = MIPI_RGB565_PACKED;
			break;
		default:
			DEBUG((DEBUG_ERROR, "MIPI_DSI_NW: unsupported format! (%d)\n", mipi->format));
			return EFI_INVALID_PARAMETER;
	}
	MmioWrite32(mipi->mmio_csr_base + MIPIv2_CSR_TX_ULPS, 0);
	MmioWrite32(mipi->mmio_csr_base + MIPIv2_CSR_PXL2DPI, (uint32_t)mode);
	
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: imx8x_config finished mode=%d\n", mode));
	return EFI_SUCCESS;
}

static struct nwl_dsi_platform_ops imx8x_ops = {
	.pclk_reset = &imx8x_dsi_pclk_reset,
	.mipi_reset = &imx8x_dsi_mipi_reset,
	.dpi_reset = &imx8x_dsi_dpi_reset,
	.clock_enable = &imx8x_dsi_clock_enable,
	.clock_disable = &imx8x_dsi_clock_disable,
	.config = &imx8x_config,
};

EFI_STATUS mipi_dsi_northwest_init(
			    IMX_DISPLAY_TIMING *timings,
			    unsigned int max_data_lanes,
			    enum dsi_pixel_format fmt,
			    imxDisplayInterfaceType displayInterface)
{
	struct mipi_dsi_northwest_info *dsi = &mipi_dsi_dev;

	if ((displayInterface != imxMipiDsi1) && (displayInterface != imxMipiDsi)) {
		return EFI_INVALID_PARAMETER;
	}
	dsi->max_data_lanes = max_data_lanes;
	dsi->format = fmt;

	if (timings != NULL) {
		dsi->timings.PixelClock = timings->PixelClock;
		dsi->timings.HActive = timings->HActive;
		dsi->timings.HBlank = timings->HBlank;
		dsi->timings.HSyncOffset = timings->HSyncOffset;
		dsi->timings.VActive = timings->VActive;
		dsi->timings.VBlank = timings->VBlank;
		dsi->timings.VSyncOffset = timings->VSyncOffset;
		dsi->timings.HSync = timings->HSync;
		dsi->timings.VSync = timings->VSync;
	}

#if defined(CPU_IMX8QXP)
	dsi->mmio_base = (displayInterface == imxMipiDsi1) ? DISP_INTERFACE1_BASE : DISP_INTERFACE0_BASE;
	dsi->mmio_csr_base = dsi->mmio_base;
	dsi->mmio_base += MIPI_DSI_OFFSET;
	dsi->mmio_csr_base += MIPI_CSR_OFFSET;
	dsi->resource = (displayInterface == imxMipiDsi1) ? SC_R_MIPI_1 : SC_R_MIPI_0;
	dsi->lvds_resource = (displayInterface == imxMipiDsi1) ? SC_R_LVDS_1 : SC_R_LVDS_0;
	dsi->plat_ops = &imx8x_ops;
#else
	#error Unsupported derivative!
#endif
	dsi->max_data_rate = 1500000000;
//TODO: more pll ref rates?
	dsi->pll_ref = 27000000;
	MipiDsiPktRegisterCallback(&mipi_dsi_northwest_host_transfer);

	DEBUG((DSIIMX8X_DEBUG_LEVEL, "MIPI_DSI_NW: mipi_dsi_northwest_init lanes=%d, fmt=%d, base=0x%x\n",
		max_data_lanes, fmt, dsi->mmio_base));

	return EFI_SUCCESS;
}

EFI_STATUS mipi_dsi_northwest_enable(VOID)
{
	struct mipi_dsi_northwest_info *mipi_dsi = &mipi_dsi_dev;

	if (mipi_dsi->mmio_base == 0) {
		return EFI_INVALID_PARAMETER;
	}
	/* Enter the HS mode for video stream */
	mipi_dsi_set_mode(mipi_dsi, DSI_HS_MODE);

	return EFI_SUCCESS;
}

EFI_STATUS mipi_dsi_northwest_disable(VOID)
{
	struct mipi_dsi_northwest_info *mipi_dsi = &mipi_dsi_dev;

	if (mipi_dsi->mmio_base == 0) {
		return EFI_INVALID_PARAMETER;
	}
	mipi_dsi_shutdown(mipi_dsi);
	return EFI_SUCCESS;
}

static VOID mipi_dsi_northwest_dump_reg
(
	uint32_t base, uint32_t reg, char* reg_name
)
{
	uint32_t status;

	status = MmioRead32(base + reg);
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "%a(0x%x) = 0x%02X\n", reg_name, (base+reg), status));
}


EFI_STATUS mipi_dsi_northwest_dump(VOID)
{
	struct mipi_dsi_northwest_info *mipi_dsi = &mipi_dsi_dev;

	if (mipi_dsi->mmio_base == 0 || mipi_dsi->mmio_csr_base == 0) {
		return EFI_INVALID_PARAMETER;
	}

	/* CSR initialization */
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "*************************** CSR init ****************************************\n"));
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_csr_base, MIPIv2_CSR_TX_ULPS, "MIPIv2_CSR_TX_ULPS");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_csr_base, MIPIv2_CSR_PXL2DPI, "MIPIv2_CSR_PXL2DPI");

	/* IRQ initialization */
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "*************************** IRQ init ****************************************\n"));
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_IRQ_MASK, "HOST_IRQ_MASK");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_IRQ_MASK2, "HOST_IRQ_MASK2");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_IRQ_STATUS, "HOST_IRQ_STATUS");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_IRQ_STATUS2, "HOST_IRQ_STATUS2");

	/* HOST initialization */
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "*************************** HOST init ****************************************\n"));
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_NUM_LANES, "HOST_CFG_NUM_LANES");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_NONCONTINUOUS_CLK, "HOST_CFG_NONCONTINUOUS_CLK");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_T_PRE, "HOST_CFG_T_PRE");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_T_POST, "HOST_CFG_T_POST");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_TX_GAP, "HOST_CFG_TX_GAP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_AUTOINSERT_EOTP, "HOST_CFG_AUTOINSERT_EOTP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_EXTRA_CMDS_AFTER_EOTP, "HOST_CFG_EXTRA_CMDS_AFTER_EOTP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_HTX_TO_COUNT, "HOST_CFG_HTX_TO_COUNT");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_LRX_H_TO_COUNT, "HOST_CFG_LRX_H_TO_COUNT");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_BTA_H_TO_COUNT, "HOST_CFG_BTA_H_TO_COUNT");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_TWAKEUP, "HOST_CFG_TWAKEUP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_CFG_STATUS_OUT, "HOST_CFG_STATUS_OUT");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, HOST_RX_ERROR_STATUS, "HOST_RX_ERROR_STATUS");

	/* DPI initialization */
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "*************************** DPI init ****************************************\n"));
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_PIXEL_PAYLOAD_SIZE, "DPI_PIXEL_PAYLOAD_SIZE");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_PIXEL_FIFO_SEND_LEVEL, "DPI_PIXEL_FIFO_SEND_LEVEL");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_INTERFACE_COLOR_CODING, "DPI_INTERFACE_COLOR_CODING");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_PIXEL_FORMAT, "DPI_PIXEL_FORMAT");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_VSYNC_POLARITY, "DPI_VSYNC_POLARITY");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_HSYNC_POLARITY, "DPI_HSYNC_POLARITY");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_VIDEO_MODE, "DPI_VIDEO_MODE");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_HFP, "DPI_HFP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_HBP, "DPI_HBP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_HSA, "DPI_HSA");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_ENABLE_MULT_PKTS, "DPI_ENABLE_MULT_PKTS");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_VBP, "DPI_VBP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_VFP, "DPI_VFP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_BLLP_MODE, "DPI_BLLP_MODE");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_USE_NULL_PKT_BLLP, "DPI_USE_NULL_PKT_BLLP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_VACTIVE, "DPI_VACTIVE");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPI_VC, "DPI_VC");
	/* PHY initialization */
	DEBUG((DSIIMX8X_DEBUG_LEVEL, "*************************** PHY init ****************************************\n"));
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_PD_DPHY, "DPHY_PD_DPHY");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_M_PRG_HS_PREPARE, "DPHY_M_PRG_HS_PREPARE");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_MC_PRG_HS_PREPARE, "DPHY_MC_PRG_HS_PREPARE");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_M_PRG_HS_ZERO, "DPHY_M_PRG_HS_ZERO");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_MC_PRG_HS_ZERO, "DPHY_MC_PRG_HS_ZERO");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_M_PRG_HS_TRAIL, "DPHY_M_PRG_HS_TRAIL");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_MC_PRG_HS_TRAIL, "DPHY_MC_PRG_HS_TRAIL");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_PD_PLL, "DPHY_PD_PLL");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_TST, "DPHY_TST");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_CN, "DPHY_CN");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_CM, "DPHY_CM");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_CO, "DPHY_CO");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_LOCK, "DPHY_LOCK");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_LOCK_BYP, "DPHY_LOCK_BYP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_AUTO_PD_EN, "DPHY_AUTO_PD_EN");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_RXLPRP, "DPHY_RXLPRP");
	mipi_dsi_northwest_dump_reg(mipi_dsi->mmio_base, DPHY_RXCDRP, "DPHY_RXCDRP");

	return EFI_SUCCESS;
}

