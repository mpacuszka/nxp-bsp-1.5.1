/**
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

// #include "iMX8.h"

/* Shared general purpose registers. */

#define MEDIA_BLK_CTRL_BASE  0x4AC10000
#define MEDIA_BLK_CTRL_SIZE  0x8000
OperationRegion(GPR, SystemMemory, MEDIA_BLK_CTRL_BASE, MEDIA_BLK_CTRL_SIZE)
Field(GPR, DWordAcc, Nolock, Preserve)
{
  RSTN, 32,  // Reset control register, 0x00
  CLKR, 32,  // Clock control register, 0x04
  Offset(0x0C),
  QOSL, 32, // QOS and CHACHE of LCDIF, 0x0C
  QOSP, 32, // QOS and CHACHE of PXP,   0x10
  CACI, 32, // CACHE os ISI0, 0x14
  Offset(0x1C),
  QOSI, 32, // QOS of ISI1, 0x1C
  LDBC, 32, // LDB control register, 0x20
  LVDC, 32, // LVDS control register, 0x24
  LIM0, 32, // Limiter enable control register, 0x28
  LIM1, 32, // Limiter threshold control register, 0x2C
  CAMM, 32, // Camera mux control register, 0x30
  Offset(0x3C),
  PIXC, 32, // Read pixel control register, 0x3C
  RPCR, 32, // Read pixel count register, 0x40
  RLCR, 32, // Read line count register, 0x44
  CSIR, 32, // CSI register, 0x48
  DSIR, 32, // DSI register, 0x4C
  DSW0, 32, // DSI write register 0 DSI_W0, 0x50
  DSW1, 32, // DSI write register 1 DSI_W1, 0x54
  DSR0, 32, // DSI read register 0 DSI_R0, 0x58
  DSR1, 32, // DSI read register 1 DSI_R1, 0x5C
  DSPM, 32, // Dislay mux control register, 0x60
  Offset(0x70),
  PCIR, 32, // Parallel camera interface register, 0x70
  INSR, 32, // Parallel camera interface status register, 0x74
  ICR0, 32, // Parallel camera interface control register, 0x78
  ICR1, 32, // Parallel camera interface control register 1, 0x7C
}

