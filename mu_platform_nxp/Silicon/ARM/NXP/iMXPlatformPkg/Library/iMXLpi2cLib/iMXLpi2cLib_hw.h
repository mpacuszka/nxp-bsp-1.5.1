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

#ifndef _IMX_LPI2C_HW_H_
#define _IMX_LPI2C_HW_H_

#include <stdint.h>

typedef __SIZE_TYPE__ size_t;

/*! @brief Retry times for waiting flag. */
#ifndef I2C_RETRY_TIMES
#define I2C_RETRY_TIMES 0U /* Define to zero means keep waiting until the flag is assert/deassert. */
#endif

typedef struct _IMX_LPI2C_REGISTERS {
    UINT32 VERID;                                  /**< Version ID Register, offset: 0x0 */
    UINT32 PARAM;                                  /**< Parameter Register, offset: 0x4 */
    UINT32 RESERVED_0[2];
    UINT32 MCR;                                    /**< Master Control Register, offset: 0x10 */
    UINT32 MSR;                                    /**< Master Status Register, offset: 0x14 */
    UINT32 MIER;                                   /**< Master Interrupt Enable Register, offset: 0x18 */
    UINT32 MDER;                                   /**< Master DMA Enable Register, offset: 0x1C */
    UINT32 MCFGR0;                                 /**< Master Configuration Register 0, offset: 0x20 */
    UINT32 MCFGR1;                                 /**< Master Configuration Register 1, offset: 0x24 */
    UINT32 MCFGR2;                                 /**< Master Configuration Register 2, offset: 0x28 */
    UINT32 MCFGR3;                                 /**< Master Configuration Register 3, offset: 0x2C */
    UINT32 RESERVED_1[4];
    UINT32 MDMR;                                   /**< Master Data Match Register, offset: 0x40 */
    UINT32 RESERVED_2[1];
    UINT32 MCCR0;                                  /**< Master Clock Configuration Register 0, offset: 0x48 */
    UINT32 RESERVED_3[1];
    UINT32 MCCR1;                                  /**< Master Clock Configuration Register 1, offset: 0x50 */
    UINT32 RESERVED_4[1];
    UINT32 MFCR;                                   /**< Master FIFO Control Register, offset: 0x58 */
    UINT32 MFSR;                                   /**< Master FIFO Status Register, offset: 0x5C */
    UINT32 MTDR;                                   /**< Master Transmit Data Register, offset: 0x60 */
    UINT32 RESERVED_5[3];
    UINT32 MRDR;                                   /**< Master Receive Data Register, offset: 0x70 */
    UINT32 RESERVED_6[39];
    UINT32 SCR;                                    /**< Slave Control Register, offset: 0x110 */
    UINT32 SSR;                                    /**< Slave Status Register, offset: 0x114 */
    UINT32 SIER;                                   /**< Slave Interrupt Enable Register, offset: 0x118 */
    UINT32 SDER;                                   /**< Slave DMA Enable Register, offset: 0x11C */
    UINT32 RESERVED_7[1];
    UINT32 SCFGR1;                                 /**< Slave Configuration Register 1, offset: 0x124 */
    UINT32 SCFGR2;                                 /**< Slave Configuration Register 2, offset: 0x128 */
    UINT32 RESERVED_8[5];
    UINT32 SAMR;                                   /**< Slave Address Match Register, offset: 0x140 */
    UINT32 RESERVED_9[3];
    UINT32 SASR;                                   /**< Slave Address Status Register, offset: 0x150 */
    UINT32 STAR;                                   /**< Slave Transmit ACK Register, offset: 0x154 */
    UINT32 RESERVED_10[2];
    UINT32 STDR;                                   /**< Slave Transmit Data Register, offset: 0x160 */
    UINT32 RESERVED_11[3];
    UINT32 SRDR;                                   /**< Slave Receive Data Register, offset: 0x170 */
} IMX_LPI2C_REGISTERS;

// i.MX LPI2C register bit fields and macros

// LPI2C Version ID Register (VERID)

#define IMX_LPI2C_VERID_FEATURE_MASK                 ((UINT32)0xFFFFU)
#define IMX_LPI2C_VERID_FEATURE_SHIFT                ((UINT32)0U)
#define IMX_LPI2C_VERID_FEATURE(x)                   (((UINT32)(((UINT32)(x)) << IMX_LPI2C_VERID_FEATURE_SHIFT)) & IMX_LPI2C_VERID_FEATURE_MASK)

#define IMX_LPI2C_VERID_MINOR_MASK                   ((UINT32)0xFF0000U)
#define IMX_LPI2C_VERID_MINOR_SHIFT                  ((UINT32)16U)
#define IMX_LPI2C_VERID_MINOR(x)                     (((UINT32)(((UINT32)(x)) << IMX_LPI2C_VERID_MINOR_SHIFT)) & IMX_LPI2C_VERID_MINOR_MASK)

#define IMX_LPI2C_VERID_MAJOR_MASK                   ((UINT32)0xFF000000U)
#define IMX_LPI2C_VERID_MAJOR_SHIFT                  ((UINT32)24U)
#define IMX_LPI2C_VERID_MAJOR(x)                     (((UINT32)(((UINT32)(x)) << IMX_LPI2C_VERID_MAJOR_SHIFT)) & IMX_LPI2C_VERID_MAJOR_MASK)

// LPI2C Parameter Register (PARAM)

#define IMX_LPI2C_PARAM_MTXFIFO_MASK                 ((UINT32)0xFU)
#define IMX_LPI2C_PARAM_MTXFIFO_SHIFT                ((UINT32)0U)
#define IMX_LPI2C_PARAM_MTXFIFO(x)                   (((UINT32)(((UINT32)(x)) << IMX_LPI2C_PARAM_MTXFIFO_SHIFT)) & IMX_LPI2C_PARAM_MTXFIFO_MASK)

#define IMX_LPI2C_PARAM_MRXFIFO_MASK                 ((UINT32)0xF00U)
#define IMX_LPI2C_PARAM_MRXFIFO_SHIFT                ((UINT32)8U)
#define IMX_LPI2C_PARAM_MRXFIFO(x)                   (((UINT32)(((UINT32)(x)) << IMX_LPI2C_PARAM_MRXFIFO_SHIFT)) & IMX_LPI2C_PARAM_MRXFIFO_MASK)

// LPI2C Master Control Register (MCR)

#define IMX_LPI2C_MCR_MEN_MASK                       ((UINT32)0x1U)
#define IMX_LPI2C_MCR_MEN_SHIFT                      ((UINT32)0U)
#define IMX_LPI2C_MCR_MEN(x)                         (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCR_MEN_SHIFT)) & IMX_LPI2C_MCR_MEN_MASK)

#define IMX_LPI2C_MCR_RST_MASK                       ((UINT32)0x2U)
#define IMX_LPI2C_MCR_RST_SHIFT                      ((UINT32)1U)

#define IMX_LPI2C_MCR_DOZEN_MASK                     ((UINT32)0x4U)
#define IMX_LPI2C_MCR_DOZEN_SHIFT                    ((UINT32)2U)

#define IMX_LPI2C_MCR_DBGEN_MASK                     ((UINT32)0x8U)
#define IMX_LPI2C_MCR_DBGEN_SHIFT                    ((UINT32)3U)

#define IMX_LPI2C_MCR_RTF_MASK                       ((UINT32)0x100U)
#define IMX_LPI2C_MCR_RTF_SHIFT                      ((UINT32)8U)

#define IMX_LPI2C_MCR_RRF_MASK                       ((UINT32)0x200U)
#define IMX_LPI2C_MCR_RRF_SHIFT                      ((UINT32)9U)

// LPI2C Master Status Register (MSR)

#define IMX_LPI2C_MSR_TDF_MASK                       ((UINT32)0x1U)
#define IMX_LPI2C_MSR_TDF_SHIFT                      ((UINT32)0U)

#define IMX_LPI2C_MSR_RDF_MASK                       ((UINT32)0x2U)
#define IMX_LPI2C_MSR_RDF_SHIFT                      ((UINT32)1U)

#define IMX_LPI2C_MSR_EPF_MASK                       ((UINT32)0x100U)
#define IMX_LPI2C_MSR_EPF_SHIFT                      ((UINT32)8U)

#define IMX_LPI2C_MSR_SDF_MASK                       ((UINT32)0x200U)
#define IMX_LPI2C_MSR_SDF_SHIFT                      ((UINT32)9U)

#define IMX_LPI2C_MSR_NDF_MASK                       ((UINT32)0x400U)
#define IMX_LPI2C_MSR_NDF_SHIFT                      ((UINT32)10U)

#define IMX_LPI2C_MSR_ALF_MASK                       ((UINT32)0x800U)
#define IMX_LPI2C_MSR_ALF_SHIFT                      ((UINT32)11U)

#define IMX_LPI2C_MSR_FEF_MASK                       ((UINT32)0x1000U)
#define IMX_LPI2C_MSR_FEF_SHIFT                      ((UINT32)12U)

#define IMX_LPI2C_MSR_PLTF_MASK                      ((UINT32)0x2000U)
#define IMX_LPI2C_MSR_PLTF_SHIFT                     ((UINT32)13U)

#define IMX_LPI2C_MSR_DMF_MASK                       ((UINT32)0x4000U)
#define IMX_LPI2C_MSR_DMF_SHIFT                      ((UINT32)14U)

#define IMX_LPI2C_MSR_MBF_MASK                       ((UINT32)0x1000000U)
#define IMX_LPI2C_MSR_MBF_SHIFT                      ((UINT32)24U)

#define IMX_LPI2C_MSR_BBF_MASK                       ((UINT32)0x2000000U)
#define IMX_LPI2C_MSR_BBF_SHIFT                      ((UINT32)25U)

// LPI2C Master Interrupt Enable Register (MIER)

#define IMX_LPI2C_MIER_TDIE_MASK                     ((UINT32)0x1U)
#define IMX_LPI2C_MIER_TDIE_SHIFT                    ((UINT32)0U)

#define IMX_LPI2C_MIER_RDIE_MASK                     ((UINT32)0x2U)
#define IMX_LPI2C_MIER_RDIE_SHIFT                    ((UINT32)1U)

#define IMX_LPI2C_MIER_EPIE_MASK                     ((UINT32)0x100U)
#define IMX_LPI2C_MIER_EPIE_SHIFT                    ((UINT32)8U)

#define IMX_LPI2C_MIER_SDIE_MASK                     ((UINT32)0x200U)
#define IMX_LPI2C_MIER_SDIE_SHIFT                    ((UINT32)9U)

#define IMX_LPI2C_MIER_NDIE_MASK                     ((UINT32)0x400U)
#define IMX_LPI2C_MIER_NDIE_SHIFT                    ((UINT32)10U)

#define IMX_LPI2C_MIER_ALIE_MASK                     ((UINT32)0x800U)
#define IMX_LPI2C_MIER_ALIE_SHIFT                    ((UINT32)11U)

#define IMX_LPI2C_MIER_FEIE_MASK                     ((UINT32)0x1000U)
#define IMX_LPI2C_MIER_FEIE_SHIFT                    ((UINT32)12U)

#define IMX_LPI2C_MIER_PLTIE_MASK                    ((UINT32)0x2000U)
#define IMX_LPI2C_MIER_PLTIE_SHIFT                   ((UINT32)13U)

#define IMX_LPI2C_MIER_DMIE_MASK                     ((UINT32)0x4000U)
#define IMX_LPI2C_MIER_DMIE_SHIFT                    ((UINT32)14U)

// LPI2C Master DMA Enable Register (MDER)

#define IMX_LPI2C_MDER_TDDE_MASK                     ((UINT32)0x1U)
#define IMX_LPI2C_MDER_TDDE_SHIFT                    ((UINT32)0U)

#define IMX_LPI2C_MDER_RDDE_MASK                     ((UINT32)0x2U)
#define IMX_LPI2C_MDER_RDDE_SHIFT                    ((UINT32)1U)

// LPI2C Master Configuration Register 0 (MCFGR0)

#define IMX_LPI2C_MCFGR0_HREN_MASK                   ((UINT32)0x1U)
#define IMX_LPI2C_MCFGR0_HREN_SHIFT                  ((UINT32)0U)

#define IMX_LPI2C_MCFGR0_HRPOL_MASK                  ((UINT32)0x2U)
#define IMX_LPI2C_MCFGR0_HRPOL_SHIFT                 ((UINT32)1U)

#define IMX_LPI2C_MCFGR0_HRSEL_MASK                  ((UINT32)0x4U)
#define IMX_LPI2C_MCFGR0_HRSEL_SHIFT                 ((UINT32)2U)

#define IMX_LPI2C_MCFGR0_CIRFIFO_MASK                ((UINT32)0x100U)
#define IMX_LPI2C_MCFGR0_CIRFIFO_SHIFT               ((UINT32)8U)

#define IMX_LPI2C_MCFGR0_RDMO_MASK                   ((UINT32)0x200U)
#define IMX_LPI2C_MCFGR0_RDMO_SHIFT                  ((UINT32)9U)

// LPI2C Master Configuration Register 1 (MCFGR1)

#define IMX_LPI2C_MCFGR1_PRESCALE_MASK               ((UINT32)0x7U)
#define IMX_LPI2C_MCFGR1_PRESCALE_SHIFT              ((UINT32)0U)
#define IMX_LPI2C_MCFGR1_PRESCALE(x)                 (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCFGR1_PRESCALE_SHIFT)) & IMX_LPI2C_MCFGR1_PRESCALE_MASK)

#define IMX_LPI2C_MCFGR1_AUTOSTOP_MASK               ((UINT32)0x100U)
#define IMX_LPI2C_MCFGR1_AUTOSTOP_SHIFT              ((UINT32)8U)

#define IMX_LPI2C_MCFGR1_IGNACK_MASK                 ((UINT32)0x200U)
#define IMX_LPI2C_MCFGR1_IGNACK_SHIFT                ((UINT32)9U)

#define IMX_LPI2C_MCFGR1_TIMECFG_MASK                ((UINT32)0x400U)
#define IMX_LPI2C_MCFGR1_TIMECFG_SHIFT               ((UINT32)10U)

#define IMX_LPI2C_MCFGR1_MATCFG_MASK                 ((UINT32)0x70000U)
#define IMX_LPI2C_MCFGR1_MATCFG_SHIFT                ((UINT32)16U)
#define IMX_LPI2C_MCFGR1_MATCFG(x)                   (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCFGR1_MATCFG_SHIFT)) & IMX_LPI2C_MCFGR1_MATCFG_MASK)

#define IMX_LPI2C_MCFGR1_PINCFG_MASK                 ((UINT32)0x7000000U)
#define IMX_LPI2C_MCFGR1_PINCFG_SHIFT                ((UINT32)24U)
#define IMX_LPI2C_MCFGR1_PINCFG(x)                   (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCFGR1_PINCFG_SHIFT)) & IMX_LPI2C_MCFGR1_PINCFG_MASK)

// LPI2C Master Configuration Register 2 (MCFGR2)

#define IMX_LPI2C_MCFGR2_BUSIDLE_MASK                ((UINT32)0xFFFU)
#define IMX_LPI2C_MCFGR2_BUSIDLE_SHIFT               ((UINT32)0U)
#define IMX_LPI2C_MCFGR2_BUSIDLE_SIZE                ((UINT32)12U)
#define IMX_LPI2C_MCFGR2_BUSIDLE(x)                  (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCFGR2_BUSIDLE_SHIFT)) & IMX_LPI2C_MCFGR2_BUSIDLE_MASK)

#define IMX_LPI2C_MCFGR2_FILTSCL_MASK                ((UINT32)0xF0000U)
#define IMX_LPI2C_MCFGR2_FILTSCL_SHIFT               ((UINT32)16U)
#define IMX_LPI2C_MCFGR2_FILTSCL_SIZE                ((UINT32)4U)
#define IMX_LPI2C_MCFGR2_FILTSCL(x)                  (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCFGR2_FILTSCL_SHIFT)) & IMX_LPI2C_MCFGR2_FILTSCL_MASK)

#define IMX_LPI2C_MCFGR2_FILTSDA_MASK                ((UINT32)0xF000000U)
#define IMX_LPI2C_MCFGR2_FILTSDA_SHIFT               ((UINT32)24U)
#define IMX_LPI2C_MCFGR2_FILTSDA_SIZE                ((UINT32)4U)
#define IMX_LPI2C_MCFGR2_FILTSDA(x)                  (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCFGR2_FILTSDA_SHIFT)) & IMX_LPI2C_MCFGR2_FILTSDA_MASK)

// LPI2C Master Configuration Register 3 (MCFGR3)

#define IMX_LPI2C_MCFGR3_PINLOW_MASK                 ((UINT32)0xFFF00U)
#define IMX_LPI2C_MCFGR3_PINLOW_SHIFT                ((UINT32)8U)
#define IMX_LPI2C_MCFGR3_PINLOW_SIZE                 ((UINT32)12U)
#define IMX_LPI2C_MCFGR3_PINLOW(x)                   (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCFGR3_PINLOW_SHIFT)) & IMX_LPI2C_MCFGR3_PINLOW_MASK)

// LPI2C Master Data Match Register (MDMR)

#define IMX_LPI2C_MDMR_MATCH0_MASK                   ((UINT32)0xFFU)
#define IMX_LPI2C_MDMR_MATCH0_SHIFT                  ((UINT32)0U)
#define IMX_LPI2C_MDMR_MATCH0(x)                     (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MDMR_MATCH0_SHIFT)) & IMX_LPI2C_MDMR_MATCH0_MASK)

#define IMX_LPI2C_MDMR_MATCH1_MASK                   ((UINT32)0xFF0000U)
#define IMX_LPI2C_MDMR_MATCH1_SHIFT                  ((UINT32)16U)
#define IMX_LPI2C_MDMR_MATCH1(x)                     (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MDMR_MATCH1_SHIFT)) & IMX_LPI2C_MDMR_MATCH1_MASK)

// LPI2C Master Clock Configuration Register 0 (MCCR0)

#define IMX_LPI2C_MCCR0_CLKLO_MASK                   ((UINT32)0x3FU)
#define IMX_LPI2C_MCCR0_CLKLO_SHIFT                  ((UINT32)0U)
#define IMX_LPI2C_MCCR0_CLKLO_SIZE                   ((UINT32)6U)
#define IMX_LPI2C_MCCR0_CLKLO(x)                     (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCCR0_CLKLO_SHIFT)) & IMX_LPI2C_MCCR0_CLKLO_MASK)

#define IMX_LPI2C_MCCR0_CLKHI_MASK                   ((UINT32)0x3F00U)
#define IMX_LPI2C_MCCR0_CLKHI_SHIFT                  ((UINT32)8U)
#define IMX_LPI2C_MCCR0_CLKHI_SIZE                   ((UINT32)6U)
#define IMX_LPI2C_MCCR0_CLKHI(x)                     (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCCR0_CLKHI_SHIFT)) & IMX_LPI2C_MCCR0_CLKHI_MASK)

#define IMX_LPI2C_MCCR0_SETHOLD_MASK                 ((UINT32)0x3F0000U)
#define IMX_LPI2C_MCCR0_SETHOLD_SHIFT                ((UINT32)16U)
#define IMX_LPI2C_MCCR0_SETHOLD_SIZE                 ((UINT32)6U)
#define IMX_LPI2C_MCCR0_SETHOLD(x)                   (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCCR0_SETHOLD_SHIFT)) & IMX_LPI2C_MCCR0_SETHOLD_MASK)

#define IMX_LPI2C_MCCR0_DATAVD_MASK                  ((UINT32)0x3F000000U)
#define IMX_LPI2C_MCCR0_DATAVD_SHIFT                 ((UINT32)24U)
#define IMX_LPI2C_MCCR0_DATAVD_SIZE                  ((UINT32)6U)
#define IMX_LPI2C_MCCR0_DATAVD(x)                    (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCCR0_DATAVD_SHIFT)) & IMX_LPI2C_MCCR0_DATAVD_MASK)

// LPI2C Master Clock Configuration Register 1 (MCCR1)

#define IMX_LPI2C_MCCR1_CLKLO_MASK                   ((UINT32)0x3FU)
#define IMX_LPI2C_MCCR1_CLKLO_SHIFT                  ((UINT32)0U)
#define IMX_LPI2C_MCCR1_CLKLO(x)                     (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCCR1_CLKLO_SHIFT)) & IMX_LPI2C_MCCR1_CLKLO_MASK)

#define IMX_LPI2C_MCCR1_CLKHI_MASK                   ((UINT32)0x3F00U)
#define IMX_LPI2C_MCCR1_CLKHI_SHIFT                  ((UINT32)8U)
#define IMX_LPI2C_MCCR1_CLKHI(x)                     (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCCR1_CLKHI_SHIFT)) & IMX_LPI2C_MCCR1_CLKHI_MASK)

#define IMX_LPI2C_MCCR1_SETHOLD_MASK                 ((UINT32)0x3F0000U)
#define IMX_LPI2C_MCCR1_SETHOLD_SHIFT                ((UINT32)16U)
#define IMX_LPI2C_MCCR1_SETHOLD(x)                   (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCCR1_SETHOLD_SHIFT)) & IMX_LPI2C_MCCR1_SETHOLD_MASK)

#define IMX_LPI2C_MCCR1_DATAVD_MASK                  ((UINT32)0x3F000000U)
#define IMX_LPI2C_MCCR1_DATAVD_SHIFT                 ((UINT32)24U)
#define IMX_LPI2C_MCCR1_DATAVD(x)                    (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MCCR1_DATAVD_SHIFT)) & IMX_LPI2C_MCCR1_DATAVD_MASK)

// LPI2C Master FIFO Control Register (MFCR)

#define IMX_LPI2C_MFCR_TXWATER_MASK                  ((UINT32)0xFU)  /* Merged from fields with different position or width, of widths (2, 4), largest definition used */
#define IMX_LPI2C_MFCR_TXWATER_SHIFT                 ((UINT32)0U)
#define IMX_LPI2C_MFCR_TXWATER(x)                    (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MFCR_TXWATER_SHIFT)) & IMX_LPI2C_MFCR_TXWATER_MASK)  /* Merged from fields with different position or width, of widths (2, 4), largest definition used */

#define IMX_LPI2C_MFCR_RXWATER_MASK                  ((UINT32)0xF0000U)  /* Merged from fields with different position or width, of widths (2, 4), largest definition used */
#define IMX_LPI2C_MFCR_RXWATER_SHIFT                 ((UINT32)16U)
#define IMX_LPI2C_MFCR_RXWATER(x)                    (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MFCR_RXWATER_SHIFT)) & IMX_LPI2C_MFCR_RXWATER_MASK)  /* Merged from fields with different position or width, of widths (2, 4), largest definition used */

// LPI2C Master FIFO Status Register (MFSR)

#define IMX_LPI2C_MFSR_TXCOUNT_MASK                  ((UINT32)0x1FU)  /* Merged from fields with different position or width, of widths (3, 5), largest definition used */
#define IMX_LPI2C_MFSR_TXCOUNT_SHIFT                 ((UINT32)0U)
#define IMX_LPI2C_MFSR_TXCOUNT(x)                    (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MFSR_TXCOUNT_SHIFT)) & IMX_LPI2C_MFSR_TXCOUNT_MASK)  /* Merged from fields with different position or width, of widths (3, 5), largest definition used */

#define IMX_LPI2C_MFSR_RXCOUNT_MASK                  ((UINT32)0x1F0000U)  /* Merged from fields with different position or width, of widths (3, 5), largest definition used */
#define IMX_LPI2C_MFSR_RXCOUNT_SHIFT                 ((UINT32)16U)
#define IMX_LPI2C_MFSR_RXCOUNT(x)                    (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MFSR_RXCOUNT_SHIFT)) & IMX_LPI2C_MFSR_RXCOUNT_MASK)  /* Merged from fields with different position or width, of widths (3, 5), largest definition used */

// LPI2C Master Transmit Data Register (MTDR)

#define IMX_LPI2C_MTDR_DATA_MASK                     ((UINT32)0xFFU)
#define IMX_LPI2C_MTDR_DATA_SHIFT                    ((UINT32)0U)
#define IMX_LPI2C_MTDR_DATA(x)                       (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MTDR_DATA_SHIFT)) & IMX_LPI2C_MTDR_DATA_MASK)

#define IMX_LPI2C_MTDR_CMD_MASK                      ((UINT32)0x700U)
#define IMX_LPI2C_MTDR_CMD_SHIFT                     ((UINT32)8U)
#define IMX_LPI2C_MTDR_CMD(x)                        (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MTDR_CMD_SHIFT)) & IMX_LPI2C_MTDR_CMD_MASK)

// LPI2C Master Receive Data Register (MRDR)

#define IMX_LPI2C_MRDR_DATA_MASK                     ((UINT32)0xFFU)
#define IMX_LPI2C_MRDR_DATA_SHIFT                    ((UINT32)0U)
#define IMX_LPI2C_MRDR_DATA(x)                       (((UINT32)(((UINT32)(x)) << IMX_LPI2C_MRDR_DATA_SHIFT)) & IMX_LPI2C_MRDR_DATA_MASK)

#define IMX_LPI2C_MRDR_RXEMPTY_MASK                  ((UINT32)0x4000U)
#define IMX_LPI2C_MRDR_RXEMPTY_SHIFT                 ((UINT32)14U)

/*! @brief LPI2C status return codes. */
typedef enum _i2c_status
{
    kStatus_Success = 0,
    kStatus_InvalidArgument = 1,
    kStatus_LPI2C_Busy = 16, /*!< The master is already performing a transfer. */
    kStatus_LPI2C_Idle = 17, /*!< The slave driver is idle. */
    kStatus_LPI2C_Nak = 18, /*!< The slave device sent a NAK in response to a byte. */
    kStatus_LPI2C_FifoError = 19, /*!< FIFO under run or overrun. */
    kStatus_LPI2C_BitError = 20, /*!< Transferred bit was not seen on the bus. */
    kStatus_LPI2C_ArbitrationLost = 21, /*!< Arbitration lost error. */
    kStatus_LPI2C_PinLowTimeout = 22, /*!< SCL or SDA were held low longer than the timeout. */
    kStatus_LPI2C_NoTransferInProgress = 23, /*!< Attempt to abort a transfer when one is not in progress. */
    kStatus_LPI2C_DmaRequestFail = 24, /*!< DMA request failed. */
    kStatus_LPI2C_Timeout = 25, /*!< Timeout polling status flags. */
} i2c_status;

/*! @} */

/*!
 * @addtogroup lpi2c_master_driver
 * @{
 */

 /*!
  * @brief LPI2C master peripheral flags.
  *
  * The following status register flags can be cleared:
  * - #kLPI2C_MasterEndOfPacketFlag
  * - #kLPI2C_MasterStopDetectFlag
  * - #kLPI2C_MasterNackDetectFlag
  * - #kLPI2C_MasterArbitrationLostFlag
  * - #kLPI2C_MasterFifoErrFlag
  * - #kLPI2C_MasterPinLowTimeoutFlag
  * - #kLPI2C_MasterDataMatchFlag
  *
  * All flags except #kLPI2C_MasterBusyFlag and #kLPI2C_MasterBusBusyFlag can be enabled as
  * interrupts.
  *
  * @note These enums are meant to be OR'd together to form a bit mask.
  */
enum _lpi2c_master_flags
{
    kLPI2C_MasterTxReadyFlag = IMX_LPI2C_MSR_TDF_MASK,  /*!< Transmit data flag */
    kLPI2C_MasterRxReadyFlag = IMX_LPI2C_MSR_RDF_MASK,  /*!< Receive data flag */
    kLPI2C_MasterEndOfPacketFlag = IMX_LPI2C_MSR_EPF_MASK,  /*!< End Packet flag */
    kLPI2C_MasterStopDetectFlag = IMX_LPI2C_MSR_SDF_MASK,  /*!< Stop detect flag */
    kLPI2C_MasterNackDetectFlag = IMX_LPI2C_MSR_NDF_MASK,  /*!< NACK detect flag */
    kLPI2C_MasterArbitrationLostFlag = IMX_LPI2C_MSR_ALF_MASK,  /*!< Arbitration lost flag */
    kLPI2C_MasterFifoErrFlag = IMX_LPI2C_MSR_FEF_MASK,  /*!< FIFO error flag */
    kLPI2C_MasterPinLowTimeoutFlag = IMX_LPI2C_MSR_PLTF_MASK, /*!< Pin low timeout flag */
    kLPI2C_MasterDataMatchFlag = IMX_LPI2C_MSR_DMF_MASK,  /*!< Data match flag */
    kLPI2C_MasterBusyFlag = IMX_LPI2C_MSR_MBF_MASK,  /*!< Master busy flag */
    kLPI2C_MasterBusBusyFlag = IMX_LPI2C_MSR_BBF_MASK,  /*!< Bus busy flag */

    /*! All flags which are cleared by the driver upon starting a transfer. */
    kLPI2C_MasterClearFlags = kLPI2C_MasterEndOfPacketFlag | kLPI2C_MasterStopDetectFlag | kLPI2C_MasterNackDetectFlag |
    kLPI2C_MasterArbitrationLostFlag | kLPI2C_MasterFifoErrFlag |
    kLPI2C_MasterPinLowTimeoutFlag | kLPI2C_MasterDataMatchFlag,
    /*! IRQ sources enabled by the non-blocking transactional API. */
    kLPI2C_MasterIrqFlags = kLPI2C_MasterArbitrationLostFlag | kLPI2C_MasterTxReadyFlag | kLPI2C_MasterRxReadyFlag |
    kLPI2C_MasterStopDetectFlag | kLPI2C_MasterNackDetectFlag | kLPI2C_MasterPinLowTimeoutFlag |
    kLPI2C_MasterFifoErrFlag,
    /*! Errors to check for. */
    kLPI2C_MasterErrorFlags = kLPI2C_MasterNackDetectFlag | kLPI2C_MasterArbitrationLostFlag |
    kLPI2C_MasterFifoErrFlag | kLPI2C_MasterPinLowTimeoutFlag
};

/*! @brief Direction of master and slave transfers. */
typedef enum _lpi2c_direction
{
    kLPI2C_Write = 0U, /*!< Master transmit. */
    kLPI2C_Read = 1U  /*!< Master receive. */
} lpi2c_direction_t;

/*!
 * @brief Transfer option flags.
 *
 * @note These enumerations are intended to be OR'd together to form a bit mask of options for
 * the #_lpi2c_master_transfer::flags field.
 */
enum _lpi2c_master_transfer_flags
{
    kLPI2C_TransferDefaultFlag = 0x00U, /*!< Transfer starts with a start signal, stops with a stop signal. */
    kLPI2C_TransferNoStartFlag = 0x01U, /*!< Don't send a start condition, address, and sub address */
    kLPI2C_TransferRepeatedStartFlag = 0x02U, /*!< Send a repeated start condition */
    kLPI2C_TransferNoStopFlag = 0x04U, /*!< Don't send a stop condition. */
};

/*!
 * @brief Structure with settings to initialize the LPI2C master module.
 *
 * This structure holds configuration settings for the LPI2C peripheral.
 *
 * The configuration structure can be made constant so it resides in flash.
 */
typedef struct _lpi2c_master_config
{
    UINT32 baudRate_Hz;                /*!< Desired baud rate in Hertz. */
    UINT32 busIdleTimeout_ns;          /*!< Bus idle timeout in nanoseconds. Set to 0 to disable. */
    UINT32 pinLowTimeout_ns;           /*!< Pin low timeout in nanoseconds. Set to 0 to disable. */
    UINT8 sdaGlitchFilterWidth_ns;     /*!< Width in nanoseconds of glitch filter on SDA pin. Set to 0 to disable. */
    UINT8 sclGlitchFilterWidth_ns;     /*!< Width in nanoseconds of glitch filter on SCL pin. Set to 0 to disable. */
} lpi2c_master_config_t;


/* Forward declaration of the transfer descriptor and handle typedefs. */
typedef struct _lpi2c_master_transfer lpi2c_master_transfer_t;

/*!
 * @brief Non-blocking transfer descriptor structure.
 *
 * This structure is used to pass transaction parameters to the LPI2C_MasterTransferNonBlocking() API.
 */
struct _lpi2c_master_transfer
{
    UINT32 flags;        /*!< Bit mask of options for the transfer. See enumeration #_lpi2c_master_transfer_flags for
                              available options. Set to 0 or #kLPI2C_TransferDefaultFlag for normal transfers. */
    UINT16 slaveAddress; /*!< The 7-bit slave address. */
    lpi2c_direction_t direction; /*!< Either #kLPI2C_Read or #kLPI2C_Write. */
    UINT32 subaddress;         /*!< Sub address. Transferred MSB first. */
    size_t subaddressSize;       /*!< Length of sub address to send in bytes. Maximum size is 4 bytes. */
    void* data;                  /*!< Pointer to data to transfer. */
    size_t dataSize;             /*!< Number of bytes to transfer. */
};

/*!
 * @brief Performs a software reset.
 *
 * Restores the LPI2C master peripheral to reset conditions.
 *
 * @param base The LPI2C peripheral base address.
 */
static inline void LPI2C_MasterReset(IMX_LPI2C_REGISTERS* base)
{
    MmioWrite32((UINTN)&base->MCR, IMX_LPI2C_MCR_RST_MASK);
    MmioWrite32((UINTN)&base->MCR, 0);
}

#endif
