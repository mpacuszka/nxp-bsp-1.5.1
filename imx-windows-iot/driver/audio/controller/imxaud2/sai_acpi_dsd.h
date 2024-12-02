/*
 * Copyright 2023 NXP
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
 *
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

#pragma once

#define SAI_DSD_PROPERTY_MAX    24
static const char* sai_dsd_property[SAI_DSD_PROPERTY_MAX] = {
    "VERSION",       // 1 or 2 relates to SAI_DSD_VER enum
    "MODE",          // master, slave
    "TX_MSEL",       // 0, 1, 2, 3
    "RX_MSEL",       // 0, 1, 2, 3
    "TX_MCLK",       // Hz eg. 22579200 Hz
    "RX_MCLK",       // Hz eg. 22579200 Hz
    "TX_WORD_MASK",  // Tx audio word (sample) mask. Bit 0 means word unmasked, Bit 1 means masked. Default value is 0xFFFFFFFC (two first words enabled)
    "RX_WORD_MASK",  // Rx audio word (sample) mask. Bit 0 means word unmasked, Bit 1 means masked. Default value is 0xFFFFFFFC (two first words enabled)
    "TX_FRAME_SIZE",    // Frame size in bits. Default is 32. It is size of frame/slot used for audio sample (for word).
    "RX_FRAME_SIZE",    // Frame size in bits. Default is 32. It is size of frame/slot used for audio sample (for word).
    "SYNCH",         // 0 - Async mode or both, 1 - Rx synchronous with Tx, 2 - Tx synchronous with Rx
    "PROTOCOL",      // Protocol, I2S default.
    "OPT_TXBCLK_POL",  // Optional. Is set by PROTOCOL. Tx Bit clock polarity.
    "OPT_RXBCLK_POL",  // Optional. Is set by PROTOCOL. Rx Bit clock polarity.
    "OPT_TXSYNC_WIDTH",// Optional. Is set by PROTOCOL. Configures the length of the frame sync in number of bit clocks. 
    "OPT_RXSYNC_WIDTH",// Optional. Is set by PROTOCOL. Configures the length of the frame sync in number of bit clocks. 
    "OPT_TXMSB_FIRST", // Optional. Is set by PROTOCOL. Configures whether the LSB or the MSB is transmitted first. 
    "OPT_RXMSB_FIRST", // Optional. Is set by PROTOCOL. Configures whether the LSB or the MSB is transmitted first. 
    "OPT_TXEARLY_FSYNC",// Optional. Is set by PROTOCOL. Frame sync asserts one bit before the first bit of the frame.
    "OPT_RXEARLY_FSYNC",// Optional. Is set by PROTOCOL. Frame sync asserts one bit before the first bit of the frame.
    "OPT_TXFSYNC_POL",  // Optional. Is set by PROTOCOL. Frame sync polarity.
    "OPT_RXFSYNC_POL",  // Optional. Is set by PROTOCOL. Frame sync polarity.
    "OPT_TXSAMPLE_CNT", // Optional. Default is 2 samples (words) per Frame. This option defines physical interface setup - number of words (samples) per frame.
    "OPT_RXSAMPLE_CNT", // Optional. Default is 2 samples (words) per Frame. This option defines physical interface setup - number of words (samples) per frame.
};

// Example of I2S protocol :
//
// TX_WORD_MASK = 0xFFFFFFFC
// RX_WORD_MASK = 0xFFFFFFFC
// OPT_TXSAMPLE_CNT = 2
// OPT_RXSAMPLE_CNT = 2
// PROTOCOL = 0
// FRAME_SIZE = 32
// 31 ...............0      31................0
// ------------------- FRAME -------------------//
// First unmasked word  // Second unmasked word //
//---------------------////---------------------//
//  Left word (sample) ////  Rigth word (sample)//
//---------------------////---------------------//

typedef enum {
    master = 0,
    slave = 1
} SAI_DSD_MODE;

typedef enum {
    mx7 = 1,
    mx8m = 2,
} SAI_DSD_VER;

typedef enum {
    async = 0,
    syncRx2Tx = 1,
    syncTx2Rx = 2
} SAI_SYNCH_MODE;

typedef enum {
    I2S = 0,
    TDM = 1,
} SAI_PROTOCOL;

typedef enum {
    bclkHigh = 0,   // Bit clock is active high with drive outputs on rising edge and sample inputs on falling edge
    bclkLow = 1,    // Bit clock is active low with drive outputs on falling edge and sample inputs on rising edge.
    bclkNotSet = 0xf
} SAI_BCLK_POL;

typedef enum {
    fsyncHigh = 0,  // Frame sync is active high
    fsyncLow = 1,  // Frame sync is active low
    fsyncNotSet = 0xf
} SAI_FSYNC_POL;

// This uion maps to sai_dsd_property 
typedef union {
    UINT32  dsd[SAI_DSD_PROPERTY_MAX];
    struct {
        SAI_DSD_VER  Version;         // Se description above.
        SAI_DSD_MODE Mode;            // Se description above.
        UINT32       TxMsel;          // Se description above.
        UINT32       RxMsel;          // Se description above.
        UINT32       TxMclk;          // Master clock in Hz.
        UINT32       RxMclk;          // Master clock in Hz.
        UINT32       TxWordMask;      // Se description above.
        UINT32       RxWordMask;      // Se description above.
        UINT32       TxFrameSize;     // Frame size in bits
        UINT32       RxFrameSize;     // Frame size in bits
        SAI_SYNCH_MODE Synch;         // TX RX synchronization
        SAI_PROTOCOL Protocol;        // Protocol. Configures bus settings such as clock polarity etc.
        // Optional params. Overrides protocol settings.
        SAI_BCLK_POL OptTxBcklPol;        // Bit clock polarity.
        SAI_BCLK_POL OptRxBcklPol;        // Bit clock polarity.
        INT32        OptTxSyncWidth;      // Configures the length of the frame sync in number of bit clocks. 
        INT32        OptRxSyncWidth;      // Configures the length of the frame sync in number of bit clocks. 
        INT32        OptTxMsbFirst;       // Configures whether the LSB or the MSB is transmitted first. 
        INT32        OptRxMsbFirst;       // Configures whether the LSB or the MSB is transmitted first. 
        INT32        OptTxEarlyFrameSync; // Frame sync asserts one bit before the first bit of the frame.
        INT32        OptRxEarlyFrameSync; // Frame sync asserts one bit before the first bit of the frame.
        SAI_FSYNC_POL OptTxFrameSyncPol;  // Frame sync polarity.
        SAI_FSYNC_POL OptRxFrameSyncPol;  // Frame sync polarity.
        UINT32        OptTxSampleCount;   // Se description above.
        UINT32        OptRxSampleCount;   // Se description above.
    };
} SAI_DSD_CONFIG;