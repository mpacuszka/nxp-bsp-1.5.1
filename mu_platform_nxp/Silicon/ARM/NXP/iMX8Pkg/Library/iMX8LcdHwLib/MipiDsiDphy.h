/*
* Copyright 2020, 2022 NXP
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

#ifndef _MIPI_DSI_DPHY_H_
#define _MIPI_DSI_DPHY_H_

#include <stdint.h>

/* DPHY timing parameters structure */
struct DSI_DPHY_TIMING {
    uint32_t tmgBitClkMHZ;
    uint32_t tmgClkPrepare;
    uint32_t tmgClkZero;
    uint32_t tmgClkPost;
    uint32_t tmgClkTrail;
    uint32_t tmgHsPrepare;
    uint32_t tmgHsZero;
    uint32_t tmgHsTrail;
    uint32_t tmgLpx;
    uint32_t tmgHsExit;
};

/* Initialization macro for DPHY timing structure */
#define DSI_DPHY_TIMING(bitclk, clkprepare, clkzero, clkpost, clktrail, \
                       hsprepare, hszero, hstrail, lpx, hsexit) \
    .tmgBitClkMHZ = bitclk, \
    .tmgClkPrepare = clkprepare, \
    .tmgClkZero = clkzero, \
    .tmgClkPost = clkpost, \
    .tmgClkTrail = clktrail, \
    .tmgHsPrepare = hsprepare, \
    .tmgHsZero = hszero, \
    .tmgHsTrail = hstrail, \
    .tmgLpx = lpx, \
    .tmgHsExit = hsexit

#endif  /* _MIPI_DSI_DPHY_H_ */
