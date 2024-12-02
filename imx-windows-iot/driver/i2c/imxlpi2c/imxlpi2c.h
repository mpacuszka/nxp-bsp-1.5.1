/*
 * Copyright 2022 NXP
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

Module Name:

    imxlpi2c.h

Abstract:

    This module contains the controller-specific type
    definitions for the SPB controller driver hardware.

Environment:

    kernel-mode only

Revision History:

*/

//
// Includes for hardware register definitions.
//

#ifndef _IMXI2C_H_
#define _IMXI2C_H_

#include "imxlpi2chw.h"

//
// iMX I2C controller registers.
// MITT tests require 16 kB maximum transfer size.
// for short term set to 64 kB, and consider removing the limit eventually
#define IMXI2C_MAX_TRANSFER_LENGTH 0x0000FFFF

// register structure must match the register mapping of iMX I2C controller hardware

typedef struct _IMXI2C_REGISTERS_ {
    HWREG<ULONG> VERID;                                  /**< Version ID Register, offset: 0x0 */
    HWREG<ULONG> PARAM;                                  /**< Parameter Register, offset: 0x4 */
    ULONG RESERVED_0[2];
    HWREG<ULONG> MCR;                                    /**< Master Control Register, offset: 0x10 */
    HWREG<ULONG> MSR;                                    /**< Master Status Register, offset: 0x14 */
    HWREG<ULONG> MIER;                                   /**< Master Interrupt Enable Register, offset: 0x18 */
    HWREG<ULONG> MDER;                                   /**< Master DMA Enable Register, offset: 0x1C */
    HWREG<ULONG> MCFGR0;                                 /**< Master Configuration Register 0, offset: 0x20 */
    HWREG<ULONG> MCFGR1;                                 /**< Master Configuration Register 1, offset: 0x24 */
    HWREG<ULONG> MCFGR2;                                 /**< Master Configuration Register 2, offset: 0x28 */
    HWREG<ULONG> MCFGR3;                                 /**< Master Configuration Register 3, offset: 0x2C */
    ULONG RESERVED_1[4];
    HWREG<ULONG> MDMR;                                   /**< Master Data Match Register, offset: 0x40 */
    ULONG RESERVED_2[1];
    HWREG<ULONG> MCCR0;                                  /**< Master Clock Configuration Register 0, offset: 0x48 */
    ULONG RESERVED_3[1];
    HWREG<ULONG> MCCR1;                                  /**< Master Clock Configuration Register 1, offset: 0x50 */
    ULONG RESERVED_4[1];
    HWREG<ULONG> MFCR;                                   /**< Master FIFO Control Register, offset: 0x58 */
    HWREG<ULONG> MFSR;                                   /**< Master FIFO Status Register, offset: 0x5C */
    HWREG<ULONG> MTDR;                                   /**< Master Transmit Data Register, offset: 0x60 */
    ULONG RESERVED_5[3];
    HWREG<ULONG> MRDR;                                   /**< Master Receive Data Register, offset: 0x70 */
    ULONG RESERVED_6[39];
    HWREG<ULONG> SCR;                                    /**< Slave Control Register, offset: 0x110 */
    HWREG<ULONG> SSR;                                    /**< Slave Status Register, offset: 0x114 */
    HWREG<ULONG> SIER;                                   /**< Slave Interrupt Enable Register, offset: 0x118 */
    HWREG<ULONG> SDER;                                   /**< Slave DMA Enable Register, offset: 0x11C */
    ULONG RESERVED_7[1];
    HWREG<ULONG> SCFGR1;                                 /**< Slave Configuration Register 1, offset: 0x124 */
    HWREG<ULONG> SCFGR2;                                 /**< Slave Configuration Register 2, offset: 0x128 */
    ULONG RESERVED_8[5];
    HWREG<ULONG> SAMR;                                   /**< Slave Address Match Register, offset: 0x140 */
    ULONG RESERVED_9[3];
    HWREG<ULONG> SASR;                                   /**< Slave Address Status Register, offset: 0x150 */
    HWREG<ULONG> STAR;                                   /**< Slave Transmit ACK Register, offset: 0x154 */
    ULONG RESERVED_10[2];
    HWREG<ULONG> STDR;                                   /**< Slave Transmit Data Register, offset: 0x160 */
    ULONG RESERVED_11[3];
    HWREG<ULONG> SRDR;                                   /**< Slave Receive Data Register, offset: 0x170 */
}
IMXI2C_REGISTERS, *PIMXI2C_REGISTERS;

// i.MX LPI2C register bit fields and macros

// LPI2C Version ID Register (VERID)

#define IMX_LPI2C_VERID_FEATURE_MASK                 ((ULONG)0xFFFFU)
#define IMX_LPI2C_VERID_FEATURE_SHIFT                ((ULONG)0U)
#define IMX_LPI2C_VERID_FEATURE(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_VERID_FEATURE_SHIFT)) & IMX_LPI2C_VERID_FEATURE_MASK)

#define IMX_LPI2C_VERID_MINOR_MASK                   ((ULONG)0xFF0000U)
#define IMX_LPI2C_VERID_MINOR_SHIFT                  ((ULONG)16U)
#define IMX_LPI2C_VERID_MINOR(x)                     (((ULONG)(((ULONG)(x)) << IMX_LPI2C_VERID_MINOR_SHIFT)) & IMX_LPI2C_VERID_MINOR_MASK)

#define IMX_LPI2C_VERID_MAJOR_MASK                   ((ULONG)0xFF000000U)
#define IMX_LPI2C_VERID_MAJOR_SHIFT                  ((ULONG)24U)
#define IMX_LPI2C_VERID_MAJOR(x)                     (((ULONG)(((ULONG)(x)) << IMX_LPI2C_VERID_MAJOR_SHIFT)) & IMX_LPI2C_VERID_MAJOR_MASK)

// LPI2C Parameter Register (PARAM)

#define IMX_LPI2C_PARAM_MTXFIFO_MASK                 ((ULONG)0xFU)
#define IMX_LPI2C_PARAM_MTXFIFO_SHIFT                ((ULONG)0U)
#define IMX_LPI2C_PARAM_MTXFIFO(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_PARAM_MTXFIFO_SHIFT)) & IMX_LPI2C_PARAM_MTXFIFO_MASK)

#define IMX_LPI2C_PARAM_MRXFIFO_MASK                 ((ULONG)0xF00U)
#define IMX_LPI2C_PARAM_MRXFIFO_SHIFT                ((ULONG)8U)
#define IMX_LPI2C_PARAM_MRXFIFO(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_PARAM_MRXFIFO_SHIFT)) & IMX_LPI2C_PARAM_MRXFIFO_MASK)

// LPI2C Master Control Register (MCR)

#define IMX_LPI2C_MCR_MEN_MASK                       ((ULONG)0x1U)
#define IMX_LPI2C_MCR_MEN_SHIFT                      ((ULONG)0U)

#define IMX_LPI2C_MCR_RST_MASK                       ((ULONG)0x2U)
#define IMX_LPI2C_MCR_RST_SHIFT                      ((ULONG)1U)

#define IMX_LPI2C_MCR_DOZEN_MASK                     ((ULONG)0x4U)
#define IMX_LPI2C_MCR_DOZEN_SHIFT                    ((ULONG)2U)

#define IMX_LPI2C_MCR_DBGEN_MASK                     ((ULONG)0x8U)
#define IMX_LPI2C_MCR_DBGEN_SHIFT                    ((ULONG)3U)

#define IMX_LPI2C_MCR_RTF_MASK                       ((ULONG)0x100U)
#define IMX_LPI2C_MCR_RTF_SHIFT                      ((ULONG)8U)

#define IMX_LPI2C_MCR_RRF_MASK                       ((ULONG)0x200U)
#define IMX_LPI2C_MCR_RRF_SHIFT                      ((ULONG)9U)

// LPI2C Master Status Register (MSR)

#define IMX_LPI2C_MSR_TDF_MASK                       ((ULONG)0x1U)
#define IMX_LPI2C_MSR_TDF_SHIFT                      ((ULONG)0U)

#define IMX_LPI2C_MSR_RDF_MASK                       ((ULONG)0x2U)
#define IMX_LPI2C_MSR_RDF_SHIFT                      ((ULONG)1U)

#define IMX_LPI2C_MSR_EPF_MASK                       ((ULONG)0x100U)
#define IMX_LPI2C_MSR_EPF_SHIFT                      ((ULONG)8U)

#define IMX_LPI2C_MSR_SDF_MASK                       ((ULONG)0x200U)
#define IMX_LPI2C_MSR_SDF_SHIFT                      ((ULONG)9U)

#define IMX_LPI2C_MSR_NDF_MASK                       ((ULONG)0x400U)
#define IMX_LPI2C_MSR_NDF_SHIFT                      ((ULONG)10U)

#define IMX_LPI2C_MSR_ALF_MASK                       ((ULONG)0x800U)
#define IMX_LPI2C_MSR_ALF_SHIFT                      ((ULONG)11U)

#define IMX_LPI2C_MSR_FEF_MASK                       ((ULONG)0x1000U)
#define IMX_LPI2C_MSR_FEF_SHIFT                      ((ULONG)12U)

#define IMX_LPI2C_MSR_PLTF_MASK                      ((ULONG)0x2000U)
#define IMX_LPI2C_MSR_PLTF_SHIFT                     ((ULONG)13U)

#define IMX_LPI2C_MSR_DMF_MASK                       ((ULONG)0x4000U)
#define IMX_LPI2C_MSR_DMF_SHIFT                      ((ULONG)14U)

#define IMX_LPI2C_MSR_MBF_MASK                       ((ULONG)0x1000000U)
#define IMX_LPI2C_MSR_MBF_SHIFT                      ((ULONG)24U)

#define IMX_LPI2C_MSR_BBF_MASK                       ((ULONG)0x2000000U)
#define IMX_LPI2C_MSR_BBF_SHIFT                      ((ULONG)25U)

// LPI2C Master Interrupt Enable Register (MIER)

#define IMX_LPI2C_MIER_TDIE_MASK                     ((ULONG)0x1U)
#define IMX_LPI2C_MIER_TDIE_SHIFT                    ((ULONG)0U)

#define IMX_LPI2C_MIER_RDIE_MASK                     ((ULONG)0x2U)
#define IMX_LPI2C_MIER_RDIE_SHIFT                    ((ULONG)1U)

#define IMX_LPI2C_MIER_EPIE_MASK                     ((ULONG)0x100U)
#define IMX_LPI2C_MIER_EPIE_SHIFT                    ((ULONG)8U)

#define IMX_LPI2C_MIER_SDIE_MASK                     ((ULONG)0x200U)
#define IMX_LPI2C_MIER_SDIE_SHIFT                    ((ULONG)9U)

#define IMX_LPI2C_MIER_NDIE_MASK                     ((ULONG)0x400U)
#define IMX_LPI2C_MIER_NDIE_SHIFT                    ((ULONG)10U)

#define IMX_LPI2C_MIER_ALIE_MASK                     ((ULONG)0x800U)
#define IMX_LPI2C_MIER_ALIE_SHIFT                    ((ULONG)11U)

#define IMX_LPI2C_MIER_FEIE_MASK                     ((ULONG)0x1000U)
#define IMX_LPI2C_MIER_FEIE_SHIFT                    ((ULONG)12U)

#define IMX_LPI2C_MIER_PLTIE_MASK                    ((ULONG)0x2000U)
#define IMX_LPI2C_MIER_PLTIE_SHIFT                   ((ULONG)13U)

#define IMX_LPI2C_MIER_DMIE_MASK                     ((ULONG)0x4000U)
#define IMX_LPI2C_MIER_DMIE_SHIFT                    ((ULONG)14U)

// LPI2C Master DMA Enable Register (MDER)

#define IMX_LPI2C_MDER_TDDE_MASK                     ((ULONG)0x1U)
#define IMX_LPI2C_MDER_TDDE_SHIFT                    ((ULONG)0U)

#define IMX_LPI2C_MDER_RDDE_MASK                     ((ULONG)0x2U)
#define IMX_LPI2C_MDER_RDDE_SHIFT                    ((ULONG)1U)

// LPI2C Master Configuration Register 0 (MCFGR0)

#define IMX_LPI2C_MCFGR0_HREN_MASK                   ((ULONG)0x1U)
#define IMX_LPI2C_MCFGR0_HREN_SHIFT                  ((ULONG)0U)

#define IMX_LPI2C_MCFGR0_HRPOL_MASK                  ((ULONG)0x2U)
#define IMX_LPI2C_MCFGR0_HRPOL_SHIFT                 ((ULONG)1U)

#define IMX_LPI2C_MCFGR0_HRSEL_MASK                  ((ULONG)0x4U)
#define IMX_LPI2C_MCFGR0_HRSEL_SHIFT                 ((ULONG)2U)

#define IMX_LPI2C_MCFGR0_CIRFIFO_MASK                ((ULONG)0x100U)
#define IMX_LPI2C_MCFGR0_CIRFIFO_SHIFT               ((ULONG)8U)

#define IMX_LPI2C_MCFGR0_RDMO_MASK                   ((ULONG)0x200U)
#define IMX_LPI2C_MCFGR0_RDMO_SHIFT                  ((ULONG)9U)

// LPI2C Master Configuration Register 1 (MCFGR1)

#define IMX_LPI2C_MCFGR1_PRESCALE_MASK               ((ULONG)0x7U)
#define IMX_LPI2C_MCFGR1_PRESCALE_SHIFT              ((ULONG)0U)
#define IMX_LPI2C_MCFGR1_PRESCALE(x)                 (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCFGR1_PRESCALE_SHIFT)) & IMX_LPI2C_MCFGR1_PRESCALE_MASK)

#define IMX_LPI2C_MCFGR1_AUTOSTOP_MASK               ((ULONG)0x100U)
#define IMX_LPI2C_MCFGR1_AUTOSTOP_SHIFT              ((ULONG)8U)

#define IMX_LPI2C_MCFGR1_IGNACK_MASK                 ((ULONG)0x200U)
#define IMX_LPI2C_MCFGR1_IGNACK_SHIFT                ((ULONG)9U)

#define IMX_LPI2C_MCFGR1_TIMECFG_MASK                ((ULONG)0x400U)
#define IMX_LPI2C_MCFGR1_TIMECFG_SHIFT               ((ULONG)10U)

#define IMX_LPI2C_MCFGR1_MATCFG_MASK                 ((ULONG)0x70000U)
#define IMX_LPI2C_MCFGR1_MATCFG_SHIFT                ((ULONG)16U)
#define IMX_LPI2C_MCFGR1_MATCFG(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCFGR1_MATCFG_SHIFT)) & IMX_LPI2C_MCFGR1_MATCFG_MASK)

#define IMX_LPI2C_MCFGR1_PINCFG_MASK                 ((ULONG)0x7000000U)
#define IMX_LPI2C_MCFGR1_PINCFG_SHIFT                ((ULONG)24U)
#define IMX_LPI2C_MCFGR1_PINCFG(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCFGR1_PINCFG_SHIFT)) & IMX_LPI2C_MCFGR1_PINCFG_MASK)

// LPI2C Master Configuration Register 2 (MCFGR2)

#define IMX_LPI2C_MCFGR2_BUSIDLE_MASK                ((ULONG)0xFFFU)
#define IMX_LPI2C_MCFGR2_BUSIDLE_SHIFT               ((ULONG)0U)
#define IMX_LPI2C_MCFGR2_BUSIDLE_SIZE                ((ULONG)12U)
#define IMX_LPI2C_MCFGR2_BUSIDLE(x)                  (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCFGR2_BUSIDLE_SHIFT)) & IMX_LPI2C_MCFGR2_BUSIDLE_MASK)

#define IMX_LPI2C_MCFGR2_FILTSCL_MASK                ((ULONG)0xF0000U)
#define IMX_LPI2C_MCFGR2_FILTSCL_SHIFT               ((ULONG)16U)
#define IMX_LPI2C_MCFGR2_FILTSCL_SIZE                ((ULONG)4U)
#define IMX_LPI2C_MCFGR2_FILTSCL(x)                  (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCFGR2_FILTSCL_SHIFT)) & IMX_LPI2C_MCFGR2_FILTSCL_MASK)

#define IMX_LPI2C_MCFGR2_FILTSDA_MASK                ((ULONG)0xF000000U)
#define IMX_LPI2C_MCFGR2_FILTSDA_SHIFT               ((ULONG)24U)
#define IMX_LPI2C_MCFGR2_FILTSDA_SIZE                ((ULONG)4U)
#define IMX_LPI2C_MCFGR2_FILTSDA(x)                  (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCFGR2_FILTSDA_SHIFT)) & IMX_LPI2C_MCFGR2_FILTSDA_MASK)

// LPI2C Master Configuration Register 3 (MCFGR3)

#define IMX_LPI2C_MCFGR3_PINLOW_MASK                 ((ULONG)0xFFF00U)
#define IMX_LPI2C_MCFGR3_PINLOW_SHIFT                ((ULONG)8U)
#define IMX_LPI2C_MCFGR3_PINLOW_SIZE                 ((ULONG)12U)
#define IMX_LPI2C_MCFGR3_PINLOW(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCFGR3_PINLOW_SHIFT)) & IMX_LPI2C_MCFGR3_PINLOW_MASK)

// LPI2C Master Data Match Register (MDMR)

#define IMX_LPI2C_MDMR_MATCH0_MASK                   ((ULONG)0xFFU)
#define IMX_LPI2C_MDMR_MATCH0_SHIFT                  ((ULONG)0U)
#define IMX_LPI2C_MDMR_MATCH0(x)                     (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MDMR_MATCH0_SHIFT)) & IMX_LPI2C_MDMR_MATCH0_MASK)

#define IMX_LPI2C_MDMR_MATCH1_MASK                   ((ULONG)0xFF0000U)
#define IMX_LPI2C_MDMR_MATCH1_SHIFT                  ((ULONG)16U)
#define IMX_LPI2C_MDMR_MATCH1(x)                     (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MDMR_MATCH1_SHIFT)) & IMX_LPI2C_MDMR_MATCH1_MASK)

// LPI2C Master Clock Configuration Register 0 (MCCR0)

#define IMX_LPI2C_MCCR0_CLKLO_MASK                   ((ULONG)0x3FU)
#define IMX_LPI2C_MCCR0_CLKLO_SHIFT                  ((ULONG)0U)
#define IMX_LPI2C_MCCR0_CLKLO_SIZE                   ((ULONG)6U)
#define IMX_LPI2C_MCCR0_CLKLO(x)                     (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCCR0_CLKLO_SHIFT)) & IMX_LPI2C_MCCR0_CLKLO_MASK)

#define IMX_LPI2C_MCCR0_CLKHI_MASK                   ((ULONG)0x3F00U)
#define IMX_LPI2C_MCCR0_CLKHI_SHIFT                  ((ULONG)8U)
#define IMX_LPI2C_MCCR0_CLKHI_SIZE                   ((ULONG)6U)
#define IMX_LPI2C_MCCR0_CLKHI(x)                     (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCCR0_CLKHI_SHIFT)) & IMX_LPI2C_MCCR0_CLKHI_MASK)

#define IMX_LPI2C_MCCR0_SETHOLD_MASK                 ((ULONG)0x3F0000U)
#define IMX_LPI2C_MCCR0_SETHOLD_SHIFT                ((ULONG)16U)
#define IMX_LPI2C_MCCR0_SETHOLD_SIZE                 ((ULONG)6U)
#define IMX_LPI2C_MCCR0_SETHOLD(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCCR0_SETHOLD_SHIFT)) & IMX_LPI2C_MCCR0_SETHOLD_MASK)

#define IMX_LPI2C_MCCR0_DATAVD_MASK                  ((ULONG)0x3F000000U)
#define IMX_LPI2C_MCCR0_DATAVD_SHIFT                 ((ULONG)24U)
#define IMX_LPI2C_MCCR0_DATAVD_SIZE                  ((ULONG)6U)
#define IMX_LPI2C_MCCR0_DATAVD(x)                    (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCCR0_DATAVD_SHIFT)) & IMX_LPI2C_MCCR0_DATAVD_MASK)

// LPI2C Master Clock Configuration Register 1 (MCCR1)

#define IMX_LPI2C_MCCR1_CLKLO_MASK                   ((ULONG)0x3FU)
#define IMX_LPI2C_MCCR1_CLKLO_SHIFT                  ((ULONG)0U)
#define IMX_LPI2C_MCCR1_CLKLO(x)                     (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCCR1_CLKLO_SHIFT)) & IMX_LPI2C_MCCR1_CLKLO_MASK)

#define IMX_LPI2C_MCCR1_CLKHI_MASK                   ((ULONG)0x3F00U)
#define IMX_LPI2C_MCCR1_CLKHI_SHIFT                  ((ULONG)8U)
#define IMX_LPI2C_MCCR1_CLKHI(x)                     (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCCR1_CLKHI_SHIFT)) & IMX_LPI2C_MCCR1_CLKHI_MASK)

#define IMX_LPI2C_MCCR1_SETHOLD_MASK                 ((ULONG)0x3F0000U)
#define IMX_LPI2C_MCCR1_SETHOLD_SHIFT                ((ULONG)16U)
#define IMX_LPI2C_MCCR1_SETHOLD(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCCR1_SETHOLD_SHIFT)) & IMX_LPI2C_MCCR1_SETHOLD_MASK)

#define IMX_LPI2C_MCCR1_DATAVD_MASK                  ((ULONG)0x3F000000U)
#define IMX_LPI2C_MCCR1_DATAVD_SHIFT                 ((ULONG)24U)
#define IMX_LPI2C_MCCR1_DATAVD(x)                    (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MCCR1_DATAVD_SHIFT)) & IMX_LPI2C_MCCR1_DATAVD_MASK)

// LPI2C Master FIFO Control Register (MFCR)

#define IMX_LPI2C_MFCR_TXWATER_MASK                  ((ULONG)0xFU)  /* Merged from fields with different position or width, of widths (2, 4), largest definition used */
#define IMX_LPI2C_MFCR_TXWATER_SHIFT                 ((ULONG)0U)
#define IMX_LPI2C_MFCR_TXWATER(x)                    (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MFCR_TXWATER_SHIFT)) & IMX_LPI2C_MFCR_TXWATER_MASK)  /* Merged from fields with different position or width, of widths (2, 4), largest definition used */

#define IMX_LPI2C_MFCR_RXWATER_MASK                  ((ULONG)0xF0000U)  /* Merged from fields with different position or width, of widths (2, 4), largest definition used */
#define IMX_LPI2C_MFCR_RXWATER_SHIFT                 ((ULONG)16U)
#define IMX_LPI2C_MFCR_RXWATER(x)                    (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MFCR_RXWATER_SHIFT)) & IMX_LPI2C_MFCR_RXWATER_MASK)  /* Merged from fields with different position or width, of widths (2, 4), largest definition used */

// LPI2C Master FIFO Status Register (MFSR)

#define IMX_LPI2C_MFSR_TXCOUNT_MASK                  ((ULONG)0x1FU)  /* Merged from fields with different position or width, of widths (3, 5), largest definition used */
#define IMX_LPI2C_MFSR_TXCOUNT_SHIFT                 ((ULONG)0U)
#define IMX_LPI2C_MFSR_TXCOUNT(x)                    (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MFSR_TXCOUNT_SHIFT)) & IMX_LPI2C_MFSR_TXCOUNT_MASK)  /* Merged from fields with different position or width, of widths (3, 5), largest definition used */

#define IMX_LPI2C_MFSR_RXCOUNT_MASK                  ((ULONG)0x1F0000U)  /* Merged from fields with different position or width, of widths (3, 5), largest definition used */
#define IMX_LPI2C_MFSR_RXCOUNT_SHIFT                 ((ULONG)16U)
#define IMX_LPI2C_MFSR_RXCOUNT(x)                    (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MFSR_RXCOUNT_SHIFT)) & IMX_LPI2C_MFSR_RXCOUNT_MASK)  /* Merged from fields with different position or width, of widths (3, 5), largest definition used */

// LPI2C Master Transmit Data Register (MTDR)

#define IMX_LPI2C_MTDR_DATA_MASK                     ((ULONG)0xFFU)
#define IMX_LPI2C_MTDR_DATA_SHIFT                    ((ULONG)0U)
#define IMX_LPI2C_MTDR_DATA(x)                       (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MTDR_DATA_SHIFT)) & IMX_LPI2C_MTDR_DATA_MASK)

#define IMX_LPI2C_MTDR_CMD_MASK                      ((ULONG)0x700U)
#define IMX_LPI2C_MTDR_CMD_SHIFT                     ((ULONG)8U)
#define IMX_LPI2C_MTDR_CMD(x)                        (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MTDR_CMD_SHIFT)) & IMX_LPI2C_MTDR_CMD_MASK)

// LPI2C Master Receive Data Register (MRDR)

#define IMX_LPI2C_MRDR_DATA_MASK                     ((ULONG)0xFFU)
#define IMX_LPI2C_MRDR_DATA_SHIFT                    ((ULONG)0U)
#define IMX_LPI2C_MRDR_DATA(x)                       (((ULONG)(((ULONG)(x)) << IMX_LPI2C_MRDR_DATA_SHIFT)) & IMX_LPI2C_MRDR_DATA_MASK)

#define IMX_LPI2C_MRDR_RXEMPTY_MASK                  ((ULONG)0x4000U)
#define IMX_LPI2C_MRDR_RXEMPTY_SHIFT                 ((ULONG)14U)

// LPI2C Slave Control Register (SCR)

#define IMX_LPI2C_SCR_SEN_MASK                       ((ULONG)0x1U)
#define IMX_LPI2C_SCR_SEN_SHIFT                      ((ULONG)0U)

#define IMX_LPI2C_SCR_RST_MASK                       ((ULONG)0x2U)
#define IMX_LPI2C_SCR_RST_SHIFT                      ((ULONG)1U)

#define IMX_LPI2C_SCR_FILTEN_MASK                    ((ULONG)0x10U)
#define IMX_LPI2C_SCR_FILTEN_SHIFT                   ((ULONG)4U)

#define IMX_LPI2C_SCR_FILTDZ_MASK                    ((ULONG)0x20U)
#define IMX_LPI2C_SCR_FILTDZ_SHIFT                   ((ULONG)5U)

#define IMX_LPI2C_SCR_RTF_MASK                       ((ULONG)0x100U)
#define IMX_LPI2C_SCR_RTF_SHIFT                      ((ULONG)8U)

#define IMX_LPI2C_SCR_RRF_MASK                       ((ULONG)0x200U)
#define IMX_LPI2C_SCR_RRF_SHIFT                      ((ULONG)9U)

// LPI2C Slave Status Register (SSR)

#define IMX_LPI2C_SSR_TDF_MASK                       ((ULONG)0x1U)
#define IMX_LPI2C_SSR_TDF_SHIFT                      ((ULONG)0U)

#define IMX_LPI2C_SSR_RDF_MASK                       ((ULONG)0x2U)
#define IMX_LPI2C_SSR_RDF_SHIFT                      ((ULONG)1U)

#define IMX_LPI2C_SSR_AVF_MASK                       ((ULONG)0x4U)
#define IMX_LPI2C_SSR_AVF_SHIFT                      ((ULONG)2U)

#define IMX_LPI2C_SSR_TAF_MASK                       ((ULONG)0x8U)
#define IMX_LPI2C_SSR_TAF_SHIFT                      ((ULONG)3U)

#define IMX_LPI2C_SSR_RSF_MASK                       ((ULONG)0x100U)
#define IMX_LPI2C_SSR_RSF_SHIFT                      ((ULONG)8U)

#define IMX_LPI2C_SSR_SDF_MASK                       ((ULONG)0x200U)
#define IMX_LPI2C_SSR_SDF_SHIFT                      ((ULONG)9U)

#define IMX_LPI2C_SSR_BEF_MASK                       ((ULONG)0x400U)
#define IMX_LPI2C_SSR_BEF_SHIFT                      ((ULONG)10U)

#define IMX_LPI2C_SSR_FEF_MASK                       ((ULONG)0x800U)
#define IMX_LPI2C_SSR_FEF_SHIFT                      ((ULONG)11U)

#define IMX_LPI2C_SSR_AM0F_MASK                      ((ULONG)0x1000U)
#define IMX_LPI2C_SSR_AM0F_SHIFT                     ((ULONG)12U)

#define IMX_LPI2C_SSR_AM1F_MASK                      ((ULONG)0x2000U)
#define IMX_LPI2C_SSR_AM1F_SHIFT                     ((ULONG)13U)

#define IMX_LPI2C_SSR_GCF_MASK                       ((ULONG)0x4000U)
#define IMX_LPI2C_SSR_GCF_SHIFT                      ((ULONG)14U)

#define IMX_LPI2C_SSR_SARF_MASK                      ((ULONG)0x8000U)
#define IMX_LPI2C_SSR_SARF_SHIFT                     ((ULONG)15U)

#define IMX_LPI2C_SSR_SBF_MASK                       ((ULONG)0x1000000U)
#define IMX_LPI2C_SSR_SBF_SHIFT                      ((ULONG)24U)

#define IMX_LPI2C_SSR_BBF_MASK                       ((ULONG)0x2000000U)
#define IMX_LPI2C_SSR_BBF_SHIFT                      ((ULONG)25U)

// LPI2C Slave Interrupt Enable Register (SIER)

#define IMX_LPI2C_SIER_TDIE_MASK                     ((ULONG)0x1U)
#define IMX_LPI2C_SIER_TDIE_SHIFT                    ((ULONG)0U)

#define IMX_LPI2C_SIER_RDIE_MASK                     ((ULONG)0x2U)
#define IMX_LPI2C_SIER_RDIE_SHIFT                    ((ULONG)1U)

#define IMX_LPI2C_SIER_AVIE_MASK                     ((ULONG)0x4U)
#define IMX_LPI2C_SIER_AVIE_SHIFT                    ((ULONG)2U)

#define IMX_LPI2C_SIER_TAIE_MASK                     ((ULONG)0x8U)
#define IMX_LPI2C_SIER_TAIE_SHIFT                    ((ULONG)3U)

#define IMX_LPI2C_SIER_RSIE_MASK                     ((ULONG)0x100U)
#define IMX_LPI2C_SIER_RSIE_SHIFT                    ((ULONG)8U)

#define IMX_LPI2C_SIER_SDIE_MASK                     ((ULONG)0x200U)
#define IMX_LPI2C_SIER_SDIE_SHIFT                    ((ULONG)9U)

#define IMX_LPI2C_SIER_BEIE_MASK                     ((ULONG)0x400U)
#define IMX_LPI2C_SIER_BEIE_SHIFT                    ((ULONG)10U)

#define IMX_LPI2C_SIER_FEIE_MASK                     ((ULONG)0x800U)
#define IMX_LPI2C_SIER_FEIE_SHIFT                    ((ULONG)11U)

#define IMX_LPI2C_SIER_AM0IE_MASK                    ((ULONG)0x1000U)
#define IMX_LPI2C_SIER_AM0IE_SHIFT                   ((ULONG)12U)

#define IMX_LPI2C_SIER_AM1F_MASK                     ((ULONG)0x2000U)
#define IMX_LPI2C_SIER_AM1F_SHIFT                    ((ULONG)13U)

#define IMX_LPI2C_SIER_GCIE_MASK                     ((ULONG)0x4000U)
#define IMX_LPI2C_SIER_GCIE_SHIFT                    ((ULONG)14U)

#define IMX_LPI2C_SIER_SARIE_MASK                    ((ULONG)0x8000U)
#define IMX_LPI2C_SIER_SARIE_SHIFT                   ((ULONG)15U)

// LPI2C Slave DMA Enable Register (SDER)

#define IMX_LPI2C_SDER_TDDE_MASK                     ((ULONG)0x1U)
#define IMX_LPI2C_SDER_TDDE_SHIFT                    ((ULONG)0U)

#define IMX_LPI2C_SDER_RDDE_MASK                     ((ULONG)0x2U)
#define IMX_LPI2C_SDER_RDDE_SHIFT                    ((ULONG)1U)

#define IMX_LPI2C_SDER_AVDE_MASK                     ((ULONG)0x4U)
#define IMX_LPI2C_SDER_AVDE_SHIFT                    ((ULONG)2U)

#define IMX_LPI2C_SDER_RSDE_MASK                     ((ULONG)0x100U)
#define IMX_LPI2C_SDER_RSDE_SHIFT                    ((ULONG)8U)

#define IMX_LPI2C_SDER_SDDE_MASK                     ((ULONG)0x200U)
#define IMX_LPI2C_SDER_SDDE_SHIFT                    ((ULONG)9U)

// LPI2C Slave Configuration Register 1 (SCFGR1)

#define IMX_LPI2C_SCFGR1_ADRSTALL_MASK               ((ULONG)0x1U)
#define IMX_LPI2C_SCFGR1_ADRSTALL_SHIFT              ((ULONG)0U)

#define IMX_LPI2C_SCFGR1_RXSTALL_MASK                ((ULONG)0x2U)
#define IMX_LPI2C_SCFGR1_RXSTALL_SHIFT               ((ULONG)1U)

#define IMX_LPI2C_SCFGR1_TXDSTALL_MASK               ((ULONG)0x4U)
#define IMX_LPI2C_SCFGR1_TXDSTALL_SHIFT              ((ULONG)2U)

#define IMX_LPI2C_SCFGR1_ACKSTALL_MASK               ((ULONG)0x8U)
#define IMX_LPI2C_SCFGR1_ACKSTALL_SHIFT              ((ULONG)3U)

#define IMX_LPI2C_SCFGR1_GCEN_MASK                   ((ULONG)0x100U)
#define IMX_LPI2C_SCFGR1_GCEN_SHIFT                  ((ULONG)8U)

#define IMX_LPI2C_SCFGR1_SAEN_MASK                   ((ULONG)0x200U)
#define IMX_LPI2C_SCFGR1_SAEN_SHIFT                  ((ULONG)9U)

#define IMX_LPI2C_SCFGR1_TXCFG_MASK                  ((ULONG)0x400U)
#define IMX_LPI2C_SCFGR1_TXCFG_SHIFT                 ((ULONG)10U)

#define IMX_LPI2C_SCFGR1_RXCFG_MASK                  ((ULONG)0x800U)
#define IMX_LPI2C_SCFGR1_RXCFG_SHIFT                 ((ULONG)11U)

#define IMX_LPI2C_SCFGR1_IGNACK_MASK                 ((ULONG)0x1000U)
#define IMX_LPI2C_SCFGR1_IGNACK_SHIFT                ((ULONG)12U)

#define IMX_LPI2C_SCFGR1_HSMEN_MASK                  ((ULONG)0x2000U)
#define IMX_LPI2C_SCFGR1_HSMEN_SHIFT                 ((ULONG)13U)

#define IMX_LPI2C_SCFGR1_ADDRCFG_MASK                ((ULONG)0x70000U)
#define IMX_LPI2C_SCFGR1_ADDRCFG_SHIFT               ((ULONG)16U)
#define IMX_LPI2C_SCFGR1_ADDRCFG(x)                  (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SCFGR1_ADDRCFG_SHIFT)) & IMX_LPI2C_SCFGR1_ADDRCFG_MASK)

// LPI2C Slave Configuration Register 2 (SCFGR2)

#define IMX_LPI2C_SCFGR2_CLKHOLD_MASK                ((ULONG)0xFU)
#define IMX_LPI2C_SCFGR2_CLKHOLD_SHIFT               ((ULONG)0U)
#define IMX_LPI2C_SCFGR2_CLKHOLD(x)                  (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SCFGR2_CLKHOLD_SHIFT)) & IMX_LPI2C_SCFGR2_CLKHOLD_MASK)

#define IMX_LPI2C_SCFGR2_DATAVD_MASK                 ((ULONG)0x3F00U)
#define IMX_LPI2C_SCFGR2_DATAVD_SHIFT                ((ULONG)8U)
#define IMX_LPI2C_SCFGR2_DATAVD(x)                   (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SCFGR2_DATAVD_SHIFT)) & IMX_LPI2C_SCFGR2_DATAVD_MASK)

#define IMX_LPI2C_SCFGR2_FILTSCL_MASK                ((ULONG)0xF0000U)
#define IMX_LPI2C_SCFGR2_FILTSCL_SHIFT               ((ULONG)16U)
#define IMX_LPI2C_SCFGR2_FILTSCL(x)                  (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SCFGR2_FILTSCL_SHIFT)) & IMX_LPI2C_SCFGR2_FILTSCL_MASK)

#define IMX_LPI2C_SCFGR2_FILTSDA_MASK                ((ULONG)0xF000000U)
#define IMX_LPI2C_SCFGR2_FILTSDA_SHIFT               ((ULONG)24U)
#define IMX_LPI2C_SCFGR2_FILTSDA(x)                  (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SCFGR2_FILTSDA_SHIFT)) & IMX_LPI2C_SCFGR2_FILTSDA_MASK)

// LPI2C Slave Address Match Register (SAMR)

#define IMX_LPI2C_SAMR_ADDR0_MASK                    ((ULONG)0x7FEU)
#define IMX_LPI2C_SAMR_ADDR0_SHIFT                   ((ULONG)1U)
#define IMX_LPI2C_SAMR_ADDR0(x)                      (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SAMR_ADDR0_SHIFT)) & IMX_LPI2C_SAMR_ADDR0_MASK)

#define IMX_LPI2C_SAMR_ADDR1_MASK                    ((ULONG)0x7FE0000U)
#define IMX_LPI2C_SAMR_ADDR1_SHIFT                   ((ULONG)17U)
#define IMX_LPI2C_SAMR_ADDR1(x)                      (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SAMR_ADDR1_SHIFT)) & IMX_LPI2C_SAMR_ADDR1_MASK)

// LPI2C Slave Address Status Register (SASR)

#define IMX_LPI2C_SASR_RADDR_MASK                    ((ULONG)0x7FFU)
#define IMX_LPI2C_SASR_RADDR_SHIFT                   ((ULONG)0U)
#define IMX_LPI2C_SASR_RADDR(x)                      (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SASR_RADDR_SHIFT)) & IMX_LPI2C_SASR_RADDR_MASK)

#define IMX_LPI2C_SASR_ANV_MASK                      ((ULONG)0x4000U)
#define IMX_LPI2C_SASR_ANV_SHIFT                     ((ULONG)14U)

// LPI2C Slave Transmit ACK Register (STAR)

#define IMX_LPI2C_STAR_TXNACK_MASK                   ((ULONG)0x1U)
#define IMX_LPI2C_STAR_TXNACK_SHIFT                  ((ULONG)0U)

// LPI2C Slave Transmit Data Register (STDR)

#define IMX_LPI2C_STDR_DATA_MASK                     ((ULONG)0xFFU)
#define IMX_LPI2C_STDR_DATA_SHIFT                    ((ULONG)0U)
#define IMX_LPI2C_STDR_DATA(x)                       (((ULONG)(((ULONG)(x)) << IMX_LPI2C_STDR_DATA_SHIFT)) & IMX_LPI2C_STDR_DATA_MASK)

// LPI2C Slave Receive Data Register (SRDR)

#define IMX_LPI2C_SRDR_DATA_MASK                     ((ULONG)0xFFU)
#define IMX_LPI2C_SRDR_DATA_SHIFT                    ((ULONG)0U)
#define IMX_LPI2C_SRDR_DATA(x)                       (((ULONG)(((ULONG)(x)) << IMX_LPI2C_SRDR_DATA_SHIFT)) & IMX_LPI2C_SRDR_DATA_MASK)

#define IMX_LPI2C_SRDR_RXEMPTY_MASK                  ((ULONG)0x4000U)
#define IMX_LPI2C_SRDR_RXEMPTY_SHIFT                 ((ULONG)14U)

#define IMX_LPI2C_SRDR_SOF_MASK                      ((ULONG)0x8000U)
#define IMX_LPI2C_SRDR_SOF_SHIFT                     ((ULONG)15U)


// LPI2C commands
#define IMX_I2C_TX_CMD_TX_DATA			    (((ULONG)0U) << IMX_LPI2C_MTDR_CMD_SHIFT)
#define IMX_I2C_TX_CMD_RX_DATA			    (((ULONG)1U) << IMX_LPI2C_MTDR_CMD_SHIFT)
#define IMX_I2C_TX_CMD_STOP			        (((ULONG)2U) << IMX_LPI2C_MTDR_CMD_SHIFT)
#define IMX_I2C_TX_CMD_RX_DISCARD		    (((ULONG)3U) << IMX_LPI2C_MTDR_CMD_SHIFT)
#define IMX_I2C_TX_CMD_START			    (((ULONG)4U) << IMX_LPI2C_MTDR_CMD_SHIFT)
#define IMX_I2C_TX_CMD_START_NACK		    (((ULONG)5U) << IMX_LPI2C_MTDR_CMD_SHIFT)
#define IMX_I2C_TX_CMD_START_HISPEED		(((ULONG)6U) << IMX_LPI2C_MTDR_CMD_SHIFT)
#define IMX_I2C_TX_CMD_START_NACK_HISPEED	(((ULONG)7U) << IMX_LPI2C_MTDR_CMD_SHIFT)

#endif // of _IMXI2C_H_