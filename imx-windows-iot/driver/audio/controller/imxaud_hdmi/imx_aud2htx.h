/* Copyright (c) Microsoft Corporation. All rights reserved.
   Copyright 2023 NXP
   Licensed under the MIT License.

Abstract:
    Typedefs for the IMX SAI register blocks.

*/

#pragma once

#define AUD2HTX_DATA_REGISTER_OFFSET 0x8
#define AUD2HTX_FIFO_SIZE            0x20
#define AUD2HTX_DMA_WATER_LOW        0x10
#define AUD2HTX_DMA_BURST_SIZE       (AUD2HTX_FIFO_SIZE - AUD2HTX_DMA_WATER_LOW)
#define AUD2HTX_DMA_WIDTH            Width32Bits
#define AUD2HTX_SAMPLE_WIDTH         0x4

typedef union {
    ULONG AsUlong;
    struct {
        ULONG Enable : 1;         // bit 0
        ULONG Reserved0 : 31;     // bit 1:31
    };
} AUD2HTX_CTRL, *PAUD2HTX_CTRL;

typedef union {
    ULONG AsUlong;
    struct {
        ULONG DmaEnable : 1;         // bit 0
        ULONG DmaType : 2;           // bit 1:2
        ULONG Reserved0 : 13;        // bit 3:15
        ULONG WaterLow : 5;         // bit 16:20
        ULONG Reserved1 : 3;         // bit 21:23
        ULONG WaterHigh : 5;          // bit 24:28
        ULONG Reserved2 : 3;         // bit 29:31
    };
} AUD2HTX_CTRL_EXT, *PAUD2HTX_CTRL_EXT;

typedef struct {
    ULONG AsUlong;
} AUD2HTX_WR, * PAUD2HTX_WR;

typedef union {
    ULONG AsUlong;
    struct {
        ULONG WaterHighFlag : 1;         // bit 0
        ULONG WaterLowFlag : 1;          // bit 1
        ULONG Reserved0 : 30;            // bit 2:31
    };
} AUD2HTX_STATUS, *PAUD2HTX_STATUS;

typedef union {
    ULONG AsUlong;
    struct {
        ULONG Overflow : 1;         // bit 0
        ULONG WaterLow : 1;         // bit 1
        ULONG WaterHigh : 1;        // bit 2
        ULONG Reserved0 : 29;       // bit 3:31
    };
} AUD2HTX_IRQ_NOMASK, *PAUD2HTX_IRQ_NOMASK;

typedef union {
    ULONG AsUlong;
    struct {
        ULONG Overflow : 1;         // bit 0
        ULONG WaterLow : 1;         // bit 1
        ULONG WaterHigh : 1;        // bit 2
        ULONG Reserved0 : 29;       // bit 3:31
    };
} AUD2HTX_IRQ_MASKED, *PAUD2HTX_IRQ_MASKED;

typedef union {
    ULONG AsUlong;
    struct {
        ULONG Overflow : 1;         // bit 0
        ULONG WaterLow : 1;         // bit 1
        ULONG WaterHigh : 1;        // bit 2
        ULONG Reserved0 : 29;       // bit 3:31
    };
} AUD2HTX_IRQ_MASK, *PAUD2HTX_IRQ_MASK;

// AUD2HTX register block
typedef struct Aud2htx_registers {
    AUD2HTX_CTRL Control;
    AUD2HTX_CTRL_EXT ControlExt;
    AUD2HTX_WR Write;
    AUD2HTX_STATUS Status;
    AUD2HTX_IRQ_NOMASK NonMaskedIntFlags;
    AUD2HTX_IRQ_MASKED MaskedIntFlags;
    AUD2HTX_IRQ_MASK IrqMasks;
} AUD2HTX_REGISTERS, *PAUD2HTX_REGISTERS;

