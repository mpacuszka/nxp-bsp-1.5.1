// Copyright (c) Microsoft Corporation. All rights reserved.
// Copyright 2022 NXP
// Licensed under the MIT License.
//
//
// Module Name:
//
//   imxpepioctl.h
//
// Abstract:
//
//   IOCTL interface for IMX Pep diagnostic facilities
//
// Environment:
//
//   User and kernel mode
//

#ifndef _IMXPEPIOCTL_H_
#define _IMXPEPIOCTL_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

enum { IMXPEP_FILE_DEVICE = FILE_DEVICE_UNKNOWN };

enum {
    IMXPEP_IOCTL_ID_DUMP_REGISTERS = 100,
    IMXPEP_IOCTL_ID_STAT,
};

enum IMX_CLK {
    IMX_CLK_NONE,

    IMX_OSC_CLK,
    IMX_PLL1_MAIN_CLK,
    IMX_PLL2_MAIN_CLK,
    IMX_PLL2_PFD0,
    IMX_PLL2_PFD1,
    IMX_PLL2_PFD2,
    IMX_PLL3_MAIN_CLK,
    IMX_PLL3_PFD0,
    IMX_PLL3_PFD1,
    IMX_PLL3_PFD2,
    IMX_PLL3_PFD3,
    IMX_PLL4_MAIN_CLK,
    IMX_PLL5_MAIN_CLK,
    IMX_CLK1,
    IMX_CLK2,

    IMX_PLL1_SW_CLK,
    IMX_STEP_CLK,
    IMX_PLL3_SW_CLK,
    IMX_PLL3_80M,
    IMX_AXI_ALT,
    IMX_AXI_CLK_ROOT,
    IMX_PERIPH_CLK2,
    IMX_PERIPH_CLK,
    IMX_PRE_PERIPH_CLK,
    IMX_PRE_PERIPH2_CLK,
    IMX_PERIPH2_CLK,

    IMX_ARM_CLK_ROOT,
    IMX_MMDC_CH0_CLK_ROOT,
    IMX_MMDC_CH1_CLK_ROOT,
    IMX_AHB_CLK_ROOT,
    IMX_IPG_CLK_ROOT,
    IMX_PERCLK_CLK_ROOT,
    IMX_USDHC1_CLK_ROOT,
    IMX_USDHC2_CLK_ROOT,
    IMX_USDHC3_CLK_ROOT,
    IMX_USDHC4_CLK_ROOT,
    IMX_SSI1_CLK_ROOT,
    IMX_SSI2_CLK_ROOT,
    IMX_SSI3_CLK_ROOT,
    IMX_GPU2D_AXI_CLK_ROOT,
    IMX_GPU3D_AXI_CLK_ROOT,
    IMX_PCIE_AXI_CLK_ROOT,
    IMX_VDO_AXI_CLK_ROOT,
    IMX_IPU1_HSP_CLK_ROOT,
    IMX_IPU2_HSP_CLK_ROOT,
    IMX_GPU2D_CORE_CLK_ROOT,
    IMX_ACLK_EIM_SLOW_CLK_ROOT,
    IMX_ACLK_CLK_ROOT,
    IMX_ENFC_CLK_ROOT,
    IMX_GPU3D_CORE_CLK_ROOT,
    IMX_GPU3D_SHADER_CLK_ROOT,
    IMX_VPU_AXI_CLK_ROOT,
    IMX_IPU1_DI0_CLK_ROOT,
    IMX_IPU1_DI1_CLK_ROOT,
    IMX_IPU2_DI0_CLK_ROOT,
    IMX_IPU2_DI1_CLK_ROOT,
    IMX_LDB_DI0_SERIAL_CLK_ROOT,
    IMX_LDB_DI0_IPU,
    IMX_LDB_DI1_SERIAL_CLK_ROOT,
    IMX_LDB_DI1_IPU,
    IMX_SPDIF0_CLK_ROOT,
    IMX_SPDIF1_CLK_ROOT,
    IMX_ESAI_CLK_ROOT,
    IMX_HSI_TX_CLK_ROOT,
    IMX_CAN_CLK_ROOT,
    IMX_ECSPI_CLK_ROOT,
    IMX_UART_CLK_ROOT,
    IMX_VIDEO_27M_CLK_ROOT,
    IMX_IPG_CLK_MAC0,

    IMX_CLK_MAX,
};

//
// Clock gating definitions
//

enum IMX_CLK_GATE {
    IMX_AIPS_TZ1_CLK_ENABLE,
    IMX_AIPS_TZ2_CLK_ENABLE,
    IMX_APBHDMA_HCLK_ENABLE,
    IMX_ASRC_CLK_ENABLE,
    IMX_CAAM_SECURE_MEM_CLK_ENABLE,
    IMX_CAAM_WRAPPER_ACLK_ENABLE,
    IMX_CAAM_WRAPPER_IPG_ENABLE,
    IMX_CAN1_CLK_ENABLE,
    IMX_CAN1_SERIAL_CLK_ENABLE,
    IMX_CAN2_CLK_ENABLE,
    IMX_CAN2_SERIAL_CLK_ENABLE,
    IMX_ARM_DBG_CLK_ENABLE,
    IMX_DCIC1_CLK_ENABLE,
    IMX_DCIC2_CLK_ENABLE,
    IMX_DTCP_CLK_ENABLE,
    IMX_ECSPI1_CLK_ENABLE,
    IMX_ECSPI2_CLK_ENABLE,
    IMX_ECSPI3_CLK_ENABLE,
    IMX_ECSPI4_CLK_ENABLE,
    IMX_ECSPI5_CLK_ENABLE,
    IMX_ENET_CLK_ENABLE,
    IMX_EPIT1_CLK_ENABLE,
    IMX_EPIT2_CLK_ENABLE,
    IMX_ESAI_CLK_ENABLE,
    IMX_GPT_CLK_ENABLE,
    IMX_GPT_SERIAL_CLK_ENABLE,
    IMX_GPU2D_CLK_ENABLE,
    IMX_GPU3D_CLK_ENABLE,
    IMX_HDMI_TX_ENABLE,
    IMX_HDMI_TX_ISFRCLK_ENABLE,
    IMX_I2C1_SERIAL_CLK_ENABLE,
    IMX_I2C2_SERIAL_CLK_ENABLE,
    IMX_I2C3_SERIAL_CLK_ENABLE,
    IMX_IIM_CLK_ENABLE,
    IMX_IOMUX_IPT_CLK_IO_ENABLE,
    IMX_IPMUX1_CLK_ENABLE,
    IMX_IPMUX2_CLK_ENABLE,
    IMX_IPMUX3_CLK_ENABLE,
    IMX_IPSYNC_IP2APB_TZASC1_IPG_MASTER_CLK_ENABLE,
    IMX_IPSYNC_IP2APB_TZASC2_IPG_MASTER_CLK_ENABLE,
    IMX_IPSYNC_VDOA_IPG_MASTER_CLK_ENABLE,
    IMX_IPU1_IPU_CLK_ENABLE,
    IMX_IPU1_IPU_DI0_CLK_ENABLE,
    IMX_IPU1_IPU_DI1_CLK_ENABLE,
    IMX_IPU2_IPU_CLK_ENABLE,
    IMX_IPU2_IPU_DI0_CLK_ENABLE,
    IMX_IPU2_IPU_DI1_CLK_ENABLE,
    IMX_LDB_DI0_CLK_ENABLE,
    IMX_LDB_DI1_CLK_ENABLE,
    IMX_MIPI_CORE_CFG_CLK_ENABLE,
    IMX_MLB_CLK_ENABLE,
    IMX_MMDC_CORE_ACLK_FAST_CORE_P0_ENABLE,
    IMX_MMDC_CORE_IPG_CLK_P0_ENABLE,
    IMX_OCRAM_CLK_ENABLE,
    IMX_OPENVGAXICLK_CLK_ROOT_ENABLE,
    IMX_PCIE_ROOT_ENABLE,
    IMX_PL301_IMXQFAST1_S133CLK_ENABLE,
    IMX_PL301_IMXQPER1_BCHCLK_ENABLE,
    IMX_PL301_IMXQPER2_MAINCLK_ENABLE,
    IMX_PWM1_CLK_ENABLE,
    IMX_PWM2_CLK_ENABLE,
    IMX_PWM3_CLK_ENABLE,
    IMX_PWM4_CLK_ENABLE,
    IMX_RAWNAND_U_BCH_INPUT_APB_CLK_ENABLE,
    IMX_RAWNAND_U_GPMI_BCH_INPUT_BCH_CLK_ENABLE,
    IMX_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_CLK_ENABLE,
    IMX_RAWNAND_U_GPMI_INPUT_APB_CLK_ENABLE,
    IMX_ROM_CLK_ENABLE,
    IMX_SATA_CLK_ENABLE,
    IMX_SDMA_CLK_ENABLE,
    IMX_SPBA_CLK_ENABLE,
    IMX_SPDIF_CLK_ENABLE,
    IMX_SSI1_CLK_ENABLE,
    IMX_SSI2_CLK_ENABLE,
    IMX_SSI3_CLK_ENABLE,
    IMX_UART_CLK_ENABLE,
    IMX_UART_SERIAL_CLK_ENABLE,
    IMX_USBOH3_CLK_ENABLE,
    IMX_USDHC1_CLK_ENABLE,
    IMX_USDHC2_CLK_ENABLE,
    IMX_USDHC3_CLK_ENABLE,
    IMX_USDHC4_CLK_ENABLE,
    IMX_EIM_SLOW_CLK_ENABLE,
    IMX_VDOAXICLK_CLK_ENABLE,
    IMX_VPU_CLK_ENABLE,

    IMX_CLK_GATE_MAX,
};

enum IMX_CCM_CCGR {
    IMX_CCM_CCGR_OFF = 0x0,                 // Clock is off during all modes. Stop enter hardware handshake is disabled.
    IMX_CCM_CCGR_ON_RUN = 0x1,              // Clock is on in run mode, but off in WAIT and STOP modes
    IMX_CCM_CCGR_ON = 0x3,                  // Clock is on during all modes, except STOP mode.
};

//
// IOCTL_IMXPEP_DUMP_REGISTERS
//
// Retreive values of CCM and CCM Analog registers
//
// Input: none
// Output: IMXPEP_DUMP_REGISTERS_OUTPUT
//
enum {
    IOCTL_IMXPEP_DUMP_REGISTERS = ULONG(
        CTL_CODE(
            IMXPEP_FILE_DEVICE,
            IMXPEP_IOCTL_ID_DUMP_REGISTERS,
            METHOD_BUFFERED,
            FILE_READ_DATA))
};


typedef struct _IMXPEP_DUMP_REGISTERS_OUTPUT {
    struct {
        ULONG CCR;                             // 0x00 CCM Control Register (CCM_CCR)
        ULONG CCDR;                            // 0x04 CCM Control Divider Register (CCM_CCDR)
        ULONG CSR;                             // 0x08 CCM Status Register (CCM_CSR)
        ULONG CCSR;                            // 0x0C CCM Clock Switcher Register (CCM_CCSR)
        ULONG CACRR;                           // 0x10 CCM Arm Clock Root Register (CCM_CACRR)
        ULONG CBCDR;                           // 0x14 CCM Bus Clock Divider Register (CCM_CBCDR)
        ULONG CBCMR;                           // 0x18 CCM Bus Clock Multiplexer Register (CCM_CBCMR)
        ULONG CSCMR1;                          // 0x1C CCM Serial Clock Multiplexer Register 1 (CCM_CSCMR1)
        ULONG CSCMR2;                          // 0x20 CCM Serial Clock Multiplexer Register 2 (CCM_CSCMR2)
        ULONG CSCDR1;                          // 0x24 CCM Serial Clock Divider Register 1 (CCM_CSCDR1)
        ULONG CS1CDR;                          // 0x28 CCM SSI1 Clock Divider Register (CCM_CS1CDR)
        ULONG CS2CDR;                          // 0x2C CCM SSI2 Clock Divider Register (CCM_CS2CDR)
        ULONG CDCDR;                           // 0x30 CCM D1 Clock Divider Register (CCM_CDCDR)
        ULONG CHSCCDR;                         // 0x34 CCM HSC Clock Divider Register (CCM_CHSCCDR)
        ULONG CSCDR2;                          // 0x38 CCM Serial Clock Divider Register 2 (CCM_CSCDR2)
        ULONG CSCDR3;                          // 0x3C CCM Serial Clock Divider Register 3 (CCM_CSCDR3)
        ULONG CDHIPR;                          // 0x48 CCM Divider Handshake In-Process Register (CCM_CDHIPR)
        ULONG CLPCR;                           // 0x54 CCM Low Power Control Register (CCM_CLPCR)
        ULONG CISR;                            // 0x58 CCM Interrupt Status Register (CCM_CISR)
        ULONG CIMR;                            // 0x5C CCM Interrupt Mask Register (CCM_CIMR)
        ULONG CCOSR;                           // 0x60 CCM Clock Output Source Register (CCM_CCOSR)
        ULONG CGPR;                            // 0x64 CCM General Purpose Register (CCM_CGPR)
        ULONG CCGR[7];                         // 0x68-7C CCM Clock Gating Register 0-6 (CCM_CCGR0-CCM_CCGR6)
        ULONG CMEOR;                           // 0x88 CCM Module Enable Override Register (CCM_CMEOR)
    } Ccm;

    struct {
        ULONG PLL_ARM;                         // 0x000 Analog ARM PLL control Register (CCM_ANALOG_PLL_ARM)
        ULONG PLL_USB1;                        // 0x010 Analog USB1 480MHz PLL Control Register (CCM_ANALOG_PLL_USB1)
        ULONG PLL_USB2;                        // 0x020 Analog USB2 480MHz PLL Control Register (CCM_ANALOG_PLL_USB2)
        ULONG PLL_SYS;                         // 0x030 Analog System PLL Control Register (CCM_ANALOG_PLL_SYS)
        ULONG PLL_SYS_SS;                      // 0x040 528MHz System PLL Spread Spectrum Register (CCM_ANALOG_PLL_SYS_SS)
        ULONG PLL_SYS_NUM;                     // 0x050 Numerator of 528MHz System PLL Fractional Loop Divider Register (CCM_ANALOG_PLL_SYS_NUM)
        ULONG PLL_SYS_DENOM;                   // 0x060 Denominator of 528MHz System PLL Fractional Loop Divider Register (CCM_ANALOG_PLL_SYS_DENOM)
        ULONG PLL_AUDIO;                       // 0x070 Analog Audio PLL control Register (CCM_ANALOG_PLL_AUDIO)
        ULONG PLL_AUDIO_NUM;                   // 0x080 Numerator of Audio PLL Fractional Loop Divider Register (CCM_ANALOG_PLL_AUDIO_NUM)
        ULONG PLL_AUDIO_DENOM;                 // 0x090 Denominator of Audio PLL Fractional Loop Divider Register (CCM_ANALOG_PLL_AUDIO_DENOM)
        ULONG PLL_VIDEO;                       // 0x0A0 Analog Video PLL control Register (CCM_ANALOG_PLL_VIDEO)
        ULONG PLL_VIDEO_NUM;                   // 0x0B0 Numerator of Video PLL Fractional Loop Divider Register (CCM_ANALOG_PLL_VIDEO_NUM)
        ULONG PLL_VIDEO_DENOM;                 // 0x0C0 Denominator of Video PLL Fractional Loop Divider Register (CCM_ANALOG_PLL_VIDEO_DENOM)
        ULONG PLL_MLB;                         // 0x0D0 MLB PLL Control Register (CCM_ANALOG_PLL_MLB)
        ULONG PLL_ENET;                        // 0x0E0 Analog ENET PLL Control Register (CCM_ANALOG_PLL_ENET)
        ULONG PFD_480;                         // 0x0F0 480MHz Clock (PLL3) Phase Fractional Divider Control Register (CCM_ANALOG_PFD_480)
        ULONG PFD_528;                         // 0x100 528MHz Clock (PLL2) Phase Fractional Divider Control Register (CCM_ANALOG_PFD_528)
        ULONG PMU_REG_1P1;                     // 0x110 Regulator 1P1 Register (PMU_REG_1P1)
        ULONG PMU_REG_3P0;                     // 0X120 Regulator 3P0 Register (PMU_REG_3P0)
        ULONG PMU_REG_2P5;                     // 0x130 Regulator 2P5 Register (PMU_REG_2P5)
        ULONG PMU_REG_CORE;                    // 0x140 Digital Regulator Core Register (PMU_REG_CORE)
        ULONG MISC0;                           // 0x150 Miscellaneous Register 0 (CCM_ANALOG_MISC0)
        ULONG MISC1;                           // 0x160 Miscellaneous Register 1 (CCM_ANALOG_MISC1)
        ULONG MISC2;                           // 0x170 Miscellaneous Register 2 (CCM_ANALOG_MISC2)
    } Analog;

    struct {
        ULONG CNTR;                            // 0x000 GPC Interface control register (GPC_CNTR)
        ULONG PGR;                             // 0x004 GPC Power Gating Register (GPC_PGR)
        ULONG IMR[4];                          // 0x008 - 0x018 Interrupt mask registers
        ULONG ISR[4];                          // 0x018 - 0x028 IRQ status resisters

        struct {                               // 0x260-0x26C GPU PGC Control
            ULONG CTRL;
            ULONG PUPSCR;
            ULONG PDNSCR;
            ULONG SR;
        } PGC_GPU;

        struct {                               // 0x2A0-0x2AC CPU PGC Control
            ULONG CTRL;
            ULONG PUPSCR;
            ULONG PDNSCR;
            ULONG SR;
        } PGC_CPU;
    } Gpc;

} IMXPEP_DUMP_REGISTERS_OUTPUT, *PIMXPEP_DUMP_REGISTERS_OUTPUT;

//
// IOCTL_IMXPEP_STATE
//
// Retreive statistical counters from IMX PEP
//
// Input: none
// Output: IMXPEP_STAT_OUTPUT
//
enum {
    IOCTL_IMXPEP_STAT = ULONG(
        CTL_CODE(
            IMXPEP_FILE_DEVICE,
            IMXPEP_IOCTL_ID_STAT,
            METHOD_BUFFERED,
            FILE_READ_DATA))
};

#define _PEP_CPU_CNT 5
#define _PEP_STATE_CNT 5

typedef struct _IMXPEP_STAT_OUTPUT {
    struct {
        ULONG WfiCount;
        ULONG WfiTime;
        ULONG Wfi2Count;
        ULONG Wfi2Time;
        ULONG GatedCount;
        ULONG GatedTime;
        ULONG UndefinedCpuStateCount;
        ULONG UndefinedCpuStateTime;
    } PepCounters[_PEP_STATE_CNT][_PEP_CPU_CNT];

} IMXPEP_STAT_OUTPUT, * PIMXPEP_STAT_OUTPUT;


#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // _IMXPEPIOCTL_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// {828C6655-A8E1-4409-B505-87DAC2E24DDE}
DEFINE_GUID(
    GUID_DEVINTERFACE_IMXPEP,
    0x828c6655, 0xa8e1, 0x4409, 0xb5, 0x5, 0x87, 0xda, 0xc2, 0xe2, 0x4d, 0xde);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
