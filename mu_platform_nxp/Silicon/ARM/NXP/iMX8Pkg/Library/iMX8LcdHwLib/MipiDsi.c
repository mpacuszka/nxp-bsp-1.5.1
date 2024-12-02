/*
* Copyright 2020, 2022-2023 NXP
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

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include "MipiDsi.h"
#include "MipiDsiDphy.h"
#include "clock.h"
#include "MipiDsi_packet.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)             (sizeof(x) / sizeof(x[0]))
#endif
#ifndef DIV_RD_UP
#define DIV_RD_UP(a,div)          (((a)+(div) - 1U)/(div))
#endif
#ifndef DIV_RD_CLOSEST
#define DIV_RD_CLOSEST(a, div)    (((a) + ((div) >> 1U)) / div)
#endif

/* Macro for absolute value evaluation */
#define ABS_VAL(x) (((int)(x) < 0) ? ( -(x)) : (x))

/* Adjusted HBLANK parameters for ADV7535 converter - 4 lanes */
static struct dsiHblankPar hblank_4l[] = {
  /* {  88, 148, 44 } */
  { DSI_HBLANK_PARAM(1920, 1080, 60, 60, 105, 27, 4), },
  /* { 528, 148, 44 } */
  { DSI_HBLANK_PARAM(1920, 1080, 50, 390, 105, 27, 4), },
  /* {  88, 148, 44 } */
  { DSI_HBLANK_PARAM(1920, 1080, 30, 60, 105, 27, 4), },
  /* { 110, 220, 40 } */
  { DSI_HBLANK_PARAM(1280, 720, 60, 76, 159, 24, 4), },
  /* { 440, 220, 40 } */
  { DSI_HBLANK_PARAM(1280, 720, 50, 324, 159, 24, 4), },
  /* {  16,  60, 62 } */
  { DSI_HBLANK_PARAM(720, 480, 60, 6, 39, 40, 4), },
  /* {  12,  68, 64 } */
  { DSI_HBLANK_PARAM(720, 576, 50, 3, 45, 42, 4), },
  /* {  18,  54, 108 } */
  { DSI_HBLANK_PARAM(720, 400, 70, 8, 35, 74, 4), },
  /* {  16,  48, 96 } */
  { DSI_HBLANK_PARAM(640, 480, 60, 6, 30, 66, 4), },
};

/* Adjusted HBLANK parameters for ADV7535 converter - 2 lanes */
static struct dsiHblankPar hblank_2l[] = {
  /* {  88, 148, 44 } */
  { DSI_HBLANK_PARAM(1920, 1080, 30, 114, 210, 60, 2), },
  /* { 110, 220, 40 } */
  { DSI_HBLANK_PARAM(1280, 720, 60, 159, 320, 40, 2), },
  /* { 440, 220, 40 } */
  { DSI_HBLANK_PARAM(1280, 720, 50, 654, 320, 40, 2), },
  /* {  16,  60, 62 } */
  { DSI_HBLANK_PARAM(720, 480, 60, 16, 66, 88, 2), },
  /* {  12,  68, 64 } */
  { DSI_HBLANK_PARAM(720, 576, 50, 12, 96, 72, 2), },
  /* {  16,  48, 96 } */
  { DSI_HBLANK_PARAM(640, 480, 60, 18, 66, 138, 2), },
};

/* DPU timing params in descending order based on bit clock value */
static struct DSI_DPHY_TIMING dphyTiming[] = {
  { DSI_DPHY_TIMING(2100, 19, 91, 22, 19, 20, 35, 22, 15, 26), },
  { DSI_DPHY_TIMING(2090, 19, 91, 22, 19, 19, 35, 22, 15, 26), },
  { DSI_DPHY_TIMING(2080, 19, 91, 21, 18, 19, 35, 22, 15, 26), },
  { DSI_DPHY_TIMING(2070, 18, 90, 21, 18, 19, 35, 22, 15, 25), },
  { DSI_DPHY_TIMING(2060, 18, 90, 21, 18, 19, 34, 22, 15, 25), },
  { DSI_DPHY_TIMING(2050, 18, 89, 21, 18, 19, 34, 22, 15, 25), },
  { DSI_DPHY_TIMING(2040, 18, 89, 21, 18, 19, 34, 21, 15, 25), },
  { DSI_DPHY_TIMING(2030, 18, 88, 21, 18, 19, 34, 21, 15, 25), },
  { DSI_DPHY_TIMING(2020, 18, 88, 21, 18, 19, 34, 21, 15, 25), },
  { DSI_DPHY_TIMING(2010, 18, 87, 21, 18, 19, 34, 21, 15, 25), },
  { DSI_DPHY_TIMING(2000, 18, 87, 21, 18, 19, 33, 21, 15, 25), },
  { DSI_DPHY_TIMING(1990, 18, 87, 21, 18, 18, 33, 21, 14, 24), },
  { DSI_DPHY_TIMING(1980, 18, 86, 21, 18, 18, 33, 21, 14, 24), },
  { DSI_DPHY_TIMING(1970, 17, 86, 21, 17, 18, 33, 21, 14, 24), },
  { DSI_DPHY_TIMING(1960, 17, 85, 21, 17, 18, 33, 21, 14, 24), },
  { DSI_DPHY_TIMING(1950, 17, 85, 21, 17, 18, 32, 21, 14, 24), },
  { DSI_DPHY_TIMING(1940, 17, 84, 20, 17, 18, 32, 21, 14, 24), },
  { DSI_DPHY_TIMING(1930, 17, 84, 20, 17, 18, 32, 20, 14, 24), },
  { DSI_DPHY_TIMING(1920, 17, 84, 20, 17, 18, 32, 20, 14, 24), },
  { DSI_DPHY_TIMING(1910, 17, 83, 20, 17, 18, 32, 20, 14, 23), },
  { DSI_DPHY_TIMING(1900, 17, 83, 20, 17, 18, 32, 20, 14, 23), },
  { DSI_DPHY_TIMING(1890, 17, 82, 20, 17, 18, 31, 20, 14, 23), },
  { DSI_DPHY_TIMING(1880, 17, 82, 20, 17, 17, 31, 20, 14, 23), },
  { DSI_DPHY_TIMING(1870, 17, 81, 20, 17, 17, 31, 20, 14, 23), },
  { DSI_DPHY_TIMING(1860, 16, 81, 20, 17, 17, 31, 20, 13, 23), },
  { DSI_DPHY_TIMING(1850, 16, 80, 20, 16, 17, 31, 20, 13, 23), },
  { DSI_DPHY_TIMING(1840, 16, 80, 20, 16, 17, 30, 20, 13, 23), },
  { DSI_DPHY_TIMING(1830, 16, 80, 20, 16, 17, 30, 20, 13, 22), },
  { DSI_DPHY_TIMING(1820, 16, 79, 20, 16, 17, 30, 19, 13, 22), },
  { DSI_DPHY_TIMING(1810, 16, 79, 19, 16, 17, 30, 19, 13, 22), },
  { DSI_DPHY_TIMING(1800, 16, 78, 19, 16, 17, 30, 19, 13, 22), },
  { DSI_DPHY_TIMING(1790, 16, 78, 19, 16, 17, 30, 19, 13, 22), },
  { DSI_DPHY_TIMING(1780, 16, 77, 19, 16, 16, 29, 19, 13, 22), },
  { DSI_DPHY_TIMING(1770, 16, 77, 19, 16, 16, 29, 19, 13, 22), },
  { DSI_DPHY_TIMING(1760, 16, 77, 19, 16, 16, 29, 19, 13, 22), },
  { DSI_DPHY_TIMING(1750, 15, 76, 19, 16, 16, 29, 19, 13, 21), },
  { DSI_DPHY_TIMING(1740, 15, 76, 19, 15, 16, 29, 19, 13, 21), },
  { DSI_DPHY_TIMING(1730, 15, 75, 19, 15, 16, 28, 19, 12, 21), },
  { DSI_DPHY_TIMING(1720, 15, 75, 19, 15, 16, 28, 19, 12, 21), },
  { DSI_DPHY_TIMING(1710, 15, 74, 19, 15, 16, 28, 18, 12, 21), },
  { DSI_DPHY_TIMING(1700, 15, 74, 19, 15, 16, 28, 18, 12, 21), },
  { DSI_DPHY_TIMING(1690, 15, 73, 19, 15, 16, 28, 18, 12, 21), },
  { DSI_DPHY_TIMING(1680, 15, 73, 18, 15, 16, 28, 18, 12, 21), },
  { DSI_DPHY_TIMING(1670, 15, 73, 18, 15, 15, 27, 18, 12, 20), },
  { DSI_DPHY_TIMING(1660, 15, 72, 18, 15, 15, 27, 18, 12, 20), },
  { DSI_DPHY_TIMING(1650, 14, 72, 18, 15, 15, 27, 18, 12, 20), },
  { DSI_DPHY_TIMING(1640, 14, 71, 18, 15, 15, 27, 18, 12, 20), },
  { DSI_DPHY_TIMING(1630, 14, 71, 18, 15, 15, 27, 18, 12, 20), },
  { DSI_DPHY_TIMING(1620, 14, 70, 18, 14, 15, 26, 18, 12, 20), },
  { DSI_DPHY_TIMING(1610, 14, 70, 18, 14, 15, 26, 17, 12, 20), },
  { DSI_DPHY_TIMING(1600, 14, 70, 18, 14, 15, 26, 17, 12, 20), },
  { DSI_DPHY_TIMING(1590, 14, 69, 18, 14, 15, 26, 17, 11, 19), },
  { DSI_DPHY_TIMING(1580, 14, 69, 18, 14, 15, 26, 17, 11, 19), },
  { DSI_DPHY_TIMING(1570, 14, 68, 18, 14, 15, 26, 17, 11, 19), },
  { DSI_DPHY_TIMING(1560, 14, 68, 18, 14, 14, 25, 17, 11, 19), },
  { DSI_DPHY_TIMING(1550, 14, 67, 18, 14, 14, 25, 17, 11, 19), },
  { DSI_DPHY_TIMING(1540, 13, 67, 17, 14, 14, 25, 17, 11, 19), },
  { DSI_DPHY_TIMING(1530, 13, 66, 17, 14, 14, 25, 17, 11, 19), },
  { DSI_DPHY_TIMING(1520, 13, 66, 17, 14, 14, 25, 17, 11, 19), },
  { DSI_DPHY_TIMING(1510, 13, 66, 17, 13, 14, 24, 17, 11, 18), },
  { DSI_DPHY_TIMING(1500, 13, 65, 17, 13, 14, 24, 16, 11, 18), },
  { DSI_DPHY_TIMING(1490, 13, 65, 17, 13, 14, 24, 16, 11, 18), },
  { DSI_DPHY_TIMING(1480, 13, 64, 17, 13, 14, 24, 16, 11, 18), },
  { DSI_DPHY_TIMING(1470, 13, 64, 17, 13, 14, 24, 16, 11, 18), },
  { DSI_DPHY_TIMING(1460, 13, 63, 17, 13, 13, 24, 16, 10, 18), },
  { DSI_DPHY_TIMING(1450, 13, 63, 17, 13, 13, 23, 16, 10, 18), },
  { DSI_DPHY_TIMING(1440, 13, 63, 17, 13, 13, 23, 16, 10, 18), },
  { DSI_DPHY_TIMING(1430, 12, 62, 17, 13, 13, 23, 16, 10, 17), },
  { DSI_DPHY_TIMING(1420, 12, 62, 17, 13, 13, 23, 16, 10, 17), },
  { DSI_DPHY_TIMING(1410, 12, 61, 16, 13, 13, 23, 16, 10, 17), },
  { DSI_DPHY_TIMING(1400, 12, 61, 16, 13, 13, 23, 16, 10, 17), },
  { DSI_DPHY_TIMING(1390, 12, 60, 16, 12, 13, 22, 15, 10, 17), },
  { DSI_DPHY_TIMING(1380, 12, 60, 16, 12, 13, 22, 15, 10, 17), },
  { DSI_DPHY_TIMING(1370, 12, 59, 16, 12, 13, 22, 15, 10, 17), },
  { DSI_DPHY_TIMING(1360, 12, 59, 16, 12, 13, 22, 15, 10, 17), },
  { DSI_DPHY_TIMING(1350, 12, 59, 16, 12, 12, 22, 15, 10, 16), },
  { DSI_DPHY_TIMING(1340, 12, 58, 16, 12, 12, 21, 15, 10, 16), },
  { DSI_DPHY_TIMING(1330, 11, 58, 16, 12, 12, 21, 15,  9, 16), },
  { DSI_DPHY_TIMING(1320, 11, 57, 16, 12, 12, 21, 15,  9, 16), },
  { DSI_DPHY_TIMING(1310, 11, 57, 16, 12, 12, 21, 15,  9, 16), },
  { DSI_DPHY_TIMING(1300, 11, 56, 16, 12, 12, 21, 15,  9, 16), },
  { DSI_DPHY_TIMING(1290, 11, 56, 16, 12, 12, 21, 15,  9, 16), },
  { DSI_DPHY_TIMING(1280, 11, 56, 15, 11, 12, 20, 14,  9, 16), },
  { DSI_DPHY_TIMING(1270, 11, 55, 15, 11, 12, 20, 14,  9, 15), },
  { DSI_DPHY_TIMING(1260, 11, 55, 15, 11, 12, 20, 14,  9, 15), },
  { DSI_DPHY_TIMING(1250, 11, 54, 15, 11, 11, 20, 14,  9, 15), },
  { DSI_DPHY_TIMING(1240, 11, 54, 15, 11, 11, 20, 14,  9, 15), },
  { DSI_DPHY_TIMING(1230, 11, 53, 15, 11, 11, 19, 14,  9, 15), },
  { DSI_DPHY_TIMING(1220, 10, 53, 15, 11, 11, 19, 14,  9, 15), },
  { DSI_DPHY_TIMING(1210, 10, 52, 15, 11, 11, 19, 14,  9, 15), },
  { DSI_DPHY_TIMING(1200, 10, 52, 15, 11, 11, 19, 14,  9, 15), },
  { DSI_DPHY_TIMING(1190, 10, 52, 15, 11, 11, 19, 14,  8, 14), },
  { DSI_DPHY_TIMING(1180, 10, 51, 15, 11, 11, 19, 13,  8, 14), },
  { DSI_DPHY_TIMING(1170, 10, 51, 15, 10, 11, 18, 13,  8, 14), },
  { DSI_DPHY_TIMING(1160, 10, 50, 15, 10, 11, 18, 13,  8, 14), },
  { DSI_DPHY_TIMING(1150, 10, 50, 15, 10, 11, 18, 13,  8, 14), },
  { DSI_DPHY_TIMING(1140, 10, 49, 14, 10, 10, 18, 13,  8, 14), },
  { DSI_DPHY_TIMING(1130, 10, 49, 14, 10, 10, 18, 13,  8, 14), },
  { DSI_DPHY_TIMING(1120, 10, 49, 14, 10, 10, 17, 13,  8, 14), },
  { DSI_DPHY_TIMING(1110,  9, 48, 14, 10, 10, 17, 13,  8, 13), },
  { DSI_DPHY_TIMING(1100,  9, 48, 14, 10, 10, 17, 13,  8, 13), },
  { DSI_DPHY_TIMING(1090,  9, 47, 14, 10, 10, 17, 13,  8, 13), },
  { DSI_DPHY_TIMING(1080,  9, 47, 14, 10, 10, 17, 13,  8, 13), },
  { DSI_DPHY_TIMING(1070,  9, 46, 14, 10, 10, 17, 12,  8, 13), },
  { DSI_DPHY_TIMING(1060,  9, 46, 14, 10, 10, 16, 12,  7, 13), },
  { DSI_DPHY_TIMING(1050,  9, 45, 14,  9, 10, 16, 12,  7, 13), },
  { DSI_DPHY_TIMING(1040,  9, 45, 14,  9, 10, 16, 12,  7, 13), },
  { DSI_DPHY_TIMING(1030,  9, 45, 14,  9,  9, 16, 12,  7, 12), },
  { DSI_DPHY_TIMING(1020,  9, 44, 14,  9,  9, 16, 12,  7, 12), },
  { DSI_DPHY_TIMING(1010,  8, 44, 13,  9,  9, 15, 12,  7, 12), },
  { DSI_DPHY_TIMING(1000,  8, 43, 13,  9,  9, 15, 12,  7, 12), },
  { DSI_DPHY_TIMING( 990,  8, 43, 13,  9,  9, 15, 12,  7, 12), },
  { DSI_DPHY_TIMING( 980,  8, 42, 13,  9,  9, 15, 12,  7, 12), },
  { DSI_DPHY_TIMING( 970,  8, 42, 13,  9,  9, 15, 12,  7, 12), },
  { DSI_DPHY_TIMING( 960,  8, 42, 13,  9,  9, 15, 11,  7, 12), },
  { DSI_DPHY_TIMING( 950,  8, 41, 13,  9,  9, 14, 11,  7, 11), },
  { DSI_DPHY_TIMING( 940,  8, 41, 13,  8,  9, 14, 11,  7, 11), },
  { DSI_DPHY_TIMING( 930,  8, 40, 13,  8,  8, 14, 11,  6, 11), },
  { DSI_DPHY_TIMING( 920,  8, 40, 13,  8,  8, 14, 11,  6, 11), },
  { DSI_DPHY_TIMING( 910,  8, 39, 13,  8,  8, 14, 11,  6, 11), },
  { DSI_DPHY_TIMING( 900,  7, 39, 13,  8,  8, 13, 11,  6, 11), },
  { DSI_DPHY_TIMING( 890,  7, 38, 13,  8,  8, 13, 11,  6, 11), },
  { DSI_DPHY_TIMING( 880,  7, 38, 12,  8,  8, 13, 11,  6, 11), },
  { DSI_DPHY_TIMING( 870,  7, 38, 12,  8,  8, 13, 11,  6, 10), },
  { DSI_DPHY_TIMING( 860,  7, 37, 12,  8,  8, 13, 11,  6, 10), },
  { DSI_DPHY_TIMING( 850,  7, 37, 12,  8,  8, 13, 10,  6, 10), },
  { DSI_DPHY_TIMING( 840,  7, 36, 12,  8,  8, 12, 10,  6, 10), },
  { DSI_DPHY_TIMING( 830,  7, 36, 12,  8,  8, 12, 10,  6, 10), },
  { DSI_DPHY_TIMING( 820,  7, 35, 12,  7,  7, 12, 10,  6, 10), },
  { DSI_DPHY_TIMING( 810,  7, 35, 12,  7,  7, 12, 10,  6, 10), },
  { DSI_DPHY_TIMING( 800,  7, 35, 12,  7,  7, 12, 10,  6, 10), },
  { DSI_DPHY_TIMING( 790,  6, 34, 12,  7,  7, 11, 10,  5,  9), },
  { DSI_DPHY_TIMING( 780,  6, 34, 12,  7,  7, 11, 10,  5,  9), },
  { DSI_DPHY_TIMING( 770,  6, 33, 12,  7,  7, 11, 10,  5,  9), },
  { DSI_DPHY_TIMING( 760,  6, 33, 12,  7,  7, 11, 10,  5,  9), },
  { DSI_DPHY_TIMING( 750,  6, 32, 12,  7,  7, 11,  9,  5,  9), },
  { DSI_DPHY_TIMING( 740,  6, 32, 11,  7,  7, 11,  9,  5,  9), },
  { DSI_DPHY_TIMING( 730,  6, 31, 11,  7,  7, 10,  9,  5,  9), },
  { DSI_DPHY_TIMING( 720,  6, 31, 11,  7,  6, 10,  9,  5,  9), },
  { DSI_DPHY_TIMING( 710,  6, 31, 11,  6,  6, 10,  9,  5,  8), },
  { DSI_DPHY_TIMING( 700,  6, 30, 11,  6,  6, 10,  9,  5,  8), },
  { DSI_DPHY_TIMING( 690,  5, 30, 11,  6,  6, 10,  9,  5,  8), },
  { DSI_DPHY_TIMING( 680,  5, 29, 11,  6,  6,  9,  9,  5,  8), },
  { DSI_DPHY_TIMING( 670,  5, 29, 11,  6,  6,  9,  9,  5,  8), },
  { DSI_DPHY_TIMING( 660,  5, 28, 11,  6,  6,  9,  9,  4,  8), },
  { DSI_DPHY_TIMING( 650,  5, 28, 11,  6,  6,  9,  9,  4,  8), },
  { DSI_DPHY_TIMING( 640,  5, 28, 11,  6,  6,  9,  8,  4,  8), },
  { DSI_DPHY_TIMING( 630,  5, 27, 11,  6,  6,  9,  8,  4,  7), },
  { DSI_DPHY_TIMING( 620,  5, 27, 11,  6,  6,  8,  8,  4,  7), },
  { DSI_DPHY_TIMING( 610,  5, 26, 10,  6,  5,  8,  8,  4,  7), },
  { DSI_DPHY_TIMING( 600,  5, 26, 10,  6,  5,  8,  8,  4,  7), },
  { DSI_DPHY_TIMING( 590,  5, 25, 10,  5,  5,  8,  8,  4,  7), },
  { DSI_DPHY_TIMING( 580,  4, 25, 10,  5,  5,  8,  8,  4,  7), },
  { DSI_DPHY_TIMING( 570,  4, 24, 10,  5,  5,  7,  8,  4,  7), },
  { DSI_DPHY_TIMING( 560,  4, 24, 10,  5,  5,  7,  8,  4,  7), },
  { DSI_DPHY_TIMING( 550,  4, 24, 10,  5,  5,  7,  8,  4,  6), },
  { DSI_DPHY_TIMING( 540,  4, 23, 10,  5,  5,  7,  8,  4,  6), },
  { DSI_DPHY_TIMING( 530,  4, 23, 10,  5,  5,  7,  7,  3,  6), },
  { DSI_DPHY_TIMING( 520,  4, 22, 10,  5,  5,  7,  7,  3,  6), },
  { DSI_DPHY_TIMING( 510,  4, 22, 10,  5,  5,  6,  7,  3,  6), },
  { DSI_DPHY_TIMING( 500,  4, 21, 10,  5,  4,  6,  7,  3,  6), },
  { DSI_DPHY_TIMING( 490,  4, 21, 10,  5,  4,  6,  7,  3,  6), },
  { DSI_DPHY_TIMING( 480,  4, 21,  9,  4,  4,  6,  7,  3,  6), },
  { DSI_DPHY_TIMING( 470,  3, 20,  9,  4,  4,  6,  7,  3,  5), },
  { DSI_DPHY_TIMING( 460,  3, 20,  9,  4,  4,  5,  7,  3,  5), },
  { DSI_DPHY_TIMING( 450,  3, 19,  9,  4,  4,  5,  7,  3,  5), },
  { DSI_DPHY_TIMING( 440,  3, 19,  9,  4,  4,  5,  7,  3,  5), },
  { DSI_DPHY_TIMING( 430,  3, 18,  9,  4,  4,  5,  7,  3,  5), },
  { DSI_DPHY_TIMING( 420,  3, 18,  9,  4,  4,  5,  6,  3,  5), },
  { DSI_DPHY_TIMING( 410,  3, 17,  9,  4,  4,  5,  6,  3,  5), },
  { DSI_DPHY_TIMING( 400,  3, 17,  9,  4,  3,  4,  6,  3,  5), },
  { DSI_DPHY_TIMING( 390,  3, 17,  9,  4,  3,  4,  6,  2,  4), },
  { DSI_DPHY_TIMING( 380,  3, 16,  9,  4,  3,  4,  6,  2,  4), },
  { DSI_DPHY_TIMING( 370,  2, 16,  9,  3,  3,  4,  6,  2,  4), },
  { DSI_DPHY_TIMING( 360,  2, 15,  9,  3,  3,  4,  6,  2,  4), },
  { DSI_DPHY_TIMING( 350,  2, 15,  9,  3,  3,  3,  6,  2,  4), },
  { DSI_DPHY_TIMING( 340,  2, 14,  8,  3,  3,  3,  6,  2,  4), },
  { DSI_DPHY_TIMING( 330,  2, 14,  8,  3,  3,  3,  6,  2,  4), },
  { DSI_DPHY_TIMING( 320,  2, 14,  8,  3,  3,  3,  5,  2,  4), },
  { DSI_DPHY_TIMING( 310,  2, 13,  8,  3,  3,  3,  5,  2,  3), },
  { DSI_DPHY_TIMING( 300,  2, 13,  8,  3,  3,  3,  5,  2,  3), },
  { DSI_DPHY_TIMING( 290,  2, 12,  8,  3,  2,  2,  5,  2,  3), },
  { DSI_DPHY_TIMING( 280,  2, 12,  8,  3,  2,  2,  5,  2,  3), },
  { DSI_DPHY_TIMING( 270,  2, 11,  8,  3,  2,  2,  5,  2,  3), },
  { DSI_DPHY_TIMING( 260,  1, 11,  8,  3,  2,  2,  5,  1,  3), },
  { DSI_DPHY_TIMING( 250,  1, 10,  8,  2,  2,  2,  5,  1,  3), },
  { DSI_DPHY_TIMING( 240,  1,  9,  8,  2,  2,  1,  5,  1,  3), },
  { DSI_DPHY_TIMING( 230,  1,  8,  8,  2,  2,  1,  5,  1,  2), },
  { DSI_DPHY_TIMING( 220,  1,  8,  8,  2,  2,  1,  5,  1,  2), },
  { DSI_DPHY_TIMING( 210,  1,  7,  7,  2,  2,  1,  4,  1,  2), },
  { DSI_DPHY_TIMING( 200,  1,  7,  7,  2,  2,  1,  4,  1,  2), },
  { DSI_DPHY_TIMING( 190,  1,  7,  7,  2,  1,  1,  4,  1,  2), },
  { DSI_DPHY_TIMING( 180,  1,  6,  7,  2,  1,  0,  4,  1,  2), },
  { DSI_DPHY_TIMING( 170,  1,  6,  7,  2,  1,  0,  4,  1,  2), },
  { DSI_DPHY_TIMING( 160,  1,  6,  7,  2,  1,  0,  4,  1,  2), },
  { DSI_DPHY_TIMING( 150,  0,  5,  7,  2,  1,  0,  4,  1,  1), },
  { DSI_DPHY_TIMING( 140,  0,  5,  7,  1,  1,  0,  4,  1,  1), },
  { DSI_DPHY_TIMING( 130,  0,  4,  7,  1,  1,  0,  4,  0,  1), },
  { DSI_DPHY_TIMING( 120,  0,  4,  7,  1,  1,  0,  4,  0,  1), },
  { DSI_DPHY_TIMING( 110,  0,  3,  7,  1,  0,  0,  4,  0,  1), },
  { DSI_DPHY_TIMING( 100,  0,  3,  7,  1,  0,  0,  3,  0,  1), },
  { DSI_DPHY_TIMING(  90,  0,  2,  7,  1,  0,  0,  3,  0,  1), },
  { DSI_DPHY_TIMING(  80,  0,  2,  6,  1,  0,  0,  3,  0,  1), },
};



static struct dsiHblankPar *MipiDsiGetHBlankParams(IMX_DISPLAY_TIMING *Timing, int lanes);
static EFI_STATUS MipiDsiBitClockCalc(uint32_t* bit_clock, uint32_t pixelClock, uint32_t bpp, uint32_t num_lanes, imxConverter MipiDsiConverter);
static EFI_STATUS MipiDsiPllParamCalc(uint32_t bitClockKhz, uint32_t phyRefClockHz, uint32_t *pmsk, uint32_t *bestDelta, uint32_t *bitClockRealKhz);
EFI_STATUS MipiDsiPktSend(uint8_t Type, uint8_t Chan, uint32_t Flg, const void *Data, uint16_t Size);

/**
  Reset MIPI DSI block.
**/
VOID
MipiDsiReset (
  VOID   
  )
{
#if defined(CPU_IMX8MP)
    uint32_t tmp;

    /* De-assert LCDIF and MIPI_DSI resets within MEDIA_MIX block.*/
    tmp = MEDIA_BLK_CTRL_SFT_RSTN;
    tmp |= (MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_BUS_BLK_CLK_RESETN_MASK |
            MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_MIPI_DSI_PCLK_RESETN_MASK |
            MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_MIPI_DSI_CLKREF_RESETN_MASK |
            MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_LCDIF_PIXEL_CLK_RESETN_MASK |
            MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_LCDIF_APB_CLK_RESETN_MASK |
            MEDIA_BLK_CTRL_SFT_RSTN_SFT_EN_LCDIF_AXI_CLK_RESETN_MASK);
    MEDIA_BLK_CTRL_SFT_RSTN = tmp;
    /* Enable LCDIF and MIPI_DSI clocks within MEDIA_MIX block. */
    tmp = MEDIA_BLK_CTRL_CLK_EN;
    tmp |= (MEDIA_BLK_CTRL_CLK_EN_SFT_EN_BUS_BLK_CLK_MASK |
            MEDIA_BLK_CTRL_CLK_EN_SFT_EN_MIPI_DSI_PCLK_MASK |
            MEDIA_BLK_CTRL_CLK_EN_SFT_EN_MIPI_DSI_CLKREF_MASK |
            MEDIA_BLK_CTRL_CLK_EN_SFT_EN_LCDIF_PIXEL_CLK_MASK |
            MEDIA_BLK_CTRL_CLK_EN_SFT_EN_LCDIF_APB_CLK_MASK |
            MEDIA_BLK_CTRL_CLK_EN_SFT_EN_LCDIF_AXI_CLK_MASK);
    MEDIA_BLK_CTRL_CLK_EN = tmp;

    tmp = MEDIA_BLK_CTRL_MIPI_RESET_DIV;
    tmp |= MEDIA_BLK_CTRL_MIPI_RESET_DIV_GPR_MIPI_S_RESETN_MASK | MEDIA_BLK_CTRL_MIPI_RESET_DIV_GPR_MIPI_M_RESETN_MASK;
    MEDIA_BLK_CTRL_MIPI_RESET_DIV = tmp;
#endif

  /* Disable data transfer */
  MIPI_DSI_DSI_MDRESOL &= ~MIPI_DSI_DSI_MDRESOL_MainStandby_MASK;

  /* Disable escape clock, byte clock */
  MIPI_DSI_DSI_CLKCTRL &= ~(MIPI_DSI_DSI_CLKCTRL_TxRequestHsClk_MASK
              | MIPI_DSI_DSI_CLKCTRL_EscClkEn_MASK
              | MIPI_DSI_DSI_CLKCTRL_ByteClkEn_MASK);

  /* Disable PLL */
  MIPI_DSI_DSI_PLLCTRL &= ~MIPI_DSI_DSI_PLLCTRL_PllEn_MASK;

  /* Reset */
  MIPI_DSI_DSI_SWRST = MIPI_DSI_DSI_SWRST_SwRst_MASK | MIPI_DSI_DSI_SWRST_FuncRst_MASK;
}

/**
 * Function searches the lookup tables (hblank_4l, hblank_2l) and returns hblank params
 * if hactive, vactive, refresh matches.
 * @param Timing Display timing read from EDID.
 * @param lanes DSI lanes.
 * @return Hblank params.
 */
static struct dsiHblankPar *MipiDsiGetHBlankParams (
  IMX_DISPLAY_TIMING *Timing, 
  int lanes
  )
{
  int i, size;
  struct dsiHblankPar *hpar, *hblank;
  uint32_t hact, vact, vrefresh, htotal, vtotal;

  hact = Timing->HActive;
  vact = Timing->VActive;
  /* calculate refresh rate */
  htotal = Timing->HActive + Timing->HBlank;
  vtotal = Timing->VActive + Timing->VBlank;
  vrefresh = Timing->PixelClock / htotal / vtotal;
  /* Round vrefresh to 10 */
  if ((vrefresh % 10) >= 5) {
    vrefresh /= 10;
    vrefresh++;
    vrefresh *= 10;
  } else {
    vrefresh /= 10;
    vrefresh *= 10;
  }
  switch (lanes) {
    case 2U:
      hblank = hblank_2l;
      size   = ARRAY_SIZE(hblank_2l);
      break;
    case 4U:
      hblank = hblank_4l;
      size   = ARRAY_SIZE(hblank_4l);
      break;
    default:
      return NULL;
   }
  for (i = 0; i < size; i++) {
    hpar = &hblank[i];
    if ((hact == hpar->hactive) && (vact == hpar->vactive) && (vrefresh == hpar->refresh)) {
      return hpar;
    }
  }
  return NULL;
}

/**
 * Function calculates mipi-dsi bit clock.
 * @param port Pointer to a port structure associated.
 * @param bit_clock Calculated bit clock in kHz (output).
 * @param bpp Bits per pixel of output color mode
 * @param num_lanes Number of DSI lanes
 * @return Execution status.
 * @retval -1 Fail.
 * @retval  0 Success.
 */
static EFI_STATUS MipiDsiBitClockCalc (
  uint32_t* bit_clock, 
  uint32_t pixelClock,
  uint32_t bpp, 
  uint32_t num_lanes,
  imxConverter MipiDsiConverter
  )
{
  uint32_t pixelClockKhz, bit_clock_exact, bit_clock_kHz;

  if ((num_lanes > 4U) || (num_lanes < 1U)) {
    return EFI_DEVICE_ERROR;
  }

  /* Get pixel clock in kHz  */
  pixelClockKhz = pixelClock / 1000U;
  /* Calculate DSI bit clock (HS clock): bit_clock = pixel_clock * bpp / number_lanes. */
  bit_clock_exact = (pixelClockKhz * bpp) / num_lanes;
  DEBUG ((DEBUG_INFO, "pixelClock      = %d \n",pixelClock));
  DEBUG ((DEBUG_INFO, "bit_clock_exact = %d \n",bit_clock_exact));
  DEBUG ((DEBUG_INFO, "pixelClockKhz   = %d \n",pixelClockKhz));

  if (MipiDsiConverter == ADV7535) {
    /* Compute the bit clock exactly proportional to pixel clock. It is needed for ADV7535 converter.*/
    bit_clock_kHz = bit_clock_exact;
    if (bit_clock_kHz > DSI_MAX_HS_BIT_CLOCK_KHZ || bit_clock_kHz < DSI_MIN_HS_BIT_CLOCK_KHZ) {
        DEBUG ((DEBUG_ERROR, "bit clock out of allowed range %d - %d kHz (%d kHz).\n",
          DSI_MIN_HS_BIT_CLOCK_KHZ, DSI_MAX_HS_BIT_CLOCK_KHZ, bit_clock_kHz));
        return EFI_DEVICE_ERROR;
    }
   DEBUG ((DEBUG_INFO, "MIPI-DSI clock [kHz]: pixel clock=%d, ideal bit clock=%d (for ADV7535)\n",
              pixelClockKhz, bit_clock_kHz));
  } else {
    /* Compute the bit clock higher than calculated from pixel clock, needed for DSI panel */
    /* Try to add 12% reserve */
    bit_clock_kHz = (bit_clock_exact >> 3U) + bit_clock_exact;
    if (bit_clock_kHz > DSI_MAX_HS_BIT_CLOCK_KHZ) {
      /* Try to add 4% reserve */
      bit_clock_kHz = (bit_clock_exact >> 5U) + bit_clock_exact;
      if (bit_clock_kHz > DSI_MAX_HS_BIT_CLOCK_KHZ) {
        /* Try without reserve */
        bit_clock_kHz = bit_clock_exact;
        if (bit_clock_kHz > DSI_MAX_HS_BIT_CLOCK_KHZ) {
          DEBUG ((DEBUG_ERROR, "bit clock exceeds maximum value %d kHz (%d).\n",
                DSI_MAX_HS_BIT_CLOCK_KHZ, bit_clock_kHz));
          return EFI_DEVICE_ERROR;
        }
      }
    }
    if (bit_clock_kHz < DSI_MIN_HS_BIT_CLOCK_KHZ) {
      bit_clock_kHz = DSI_MIN_HS_BIT_CLOCK_KHZ;
    }
    DEBUG ((DEBUG_INFO, "MIPI-DSI clock [kHz]: pixel clock=%d, ideal bit clock=%d (BITCLK_INCREASED)\n",
             pixelClockKhz, bit_clock_kHz));

  }
  *bit_clock = bit_clock_kHz;

  return EFI_SUCCESS;
}


/**
 * Function calculates parameters of DSI PHY PLL (p, m, s). If successfly computed, 
 * real bit clock and best delta is updated in corresponding output prameters.
 * @param bit_clock_khz requested bit clock frequency in kHz.
 * @param refClockHz PLL reference clock frequency in Hz.
 * @param pmsk output Value of computed PLL parameters. p, m, s, k combined into single number pmsk
 * @param bestDelta Difference between requested and computed bit clock. Output parameter - can be NULL.
 * @param bitClockRealKhz Computed bit clock (achievable by PLL params). Output parameter - can be NULL.
 * @return Execution status.
 * @retval -1 Fail.
 * @retval  0 Success.
 */
/* p {1, 63}, m {64, 1023}, s {0, 5}, k = {0, 32768} (ignore 'k' for PMS calculation)
 * output value pmsk: p = P[5:0], m = M[9:0], s = S[2:0], k = K[15:0];
 *
 * Fout(bit clock) = ((m + k / 65536) * Fin) / (p * 2^s)
 *
 * Fpref = Fin / p
 * Fin: [6MHz ~ 300MHz], Fpref: [2MHz ~ 30MHz]
 *
 * Fvco = ((m + k / 65536) * Fin) / p
 * Fvco: [1050MHz - 2100MHz]
 */
static EFI_STATUS MipiDsiPllParamCalc (
  uint32_t bitClockKhz,
  uint32_t phyRefClockHz,
  uint32_t *pmsk,
  uint32_t *bestDelta,
  uint32_t *bitClockRealKhz
  )
{
  uint32_t p, s;
  uint32_t p_best, m_best, s_best, bit_clock_real_tmp, bit_clock_request_hz;
  uint64_t fvco, m, bit_clock_best_delta, bit_clock_tmp_delta;

  bit_clock_request_hz = bitClockKhz * 1000U;
  bit_clock_best_delta = bit_clock_request_hz;
  bit_clock_tmp_delta = 0U;
  p_best = m_best = s_best = 0U;
  /* Fpref is limited to 30 MHz, so in order to keep Fin between 6-30MHz, p is limited to 1-4 (P_MIN - P_MAX) */
  for (p = DSI_MIN_PLL_P; p <= DSI_MAX_PLL_P; p++) {
    for (s = DSI_MIN_PLL_S; s <= DSI_MAX_PLL_S; s++) {
      /* Fvco = bit_clock * 2^s */
      fvco = (uint64_t)bit_clock_request_hz << s;
      if ((fvco < DSI_MIN_PLL_FVCO_HZ) || (fvco >DSI_MAX_PLL_FVCO_HZ)) {
        continue;
      }
      m = (fvco * p);
      /* If reminder after division, increase the multiplier + 1, so the bit clock is always round up */
      if ((m % phyRefClockHz) > 0U) {
        m = (m / phyRefClockHz) + 1U;
      } else {
        m = m / phyRefClockHz;
      }
      bit_clock_tmp_delta = ((phyRefClockHz * m) / p) >> s;
      bit_clock_tmp_delta = bit_clock_tmp_delta - bit_clock_request_hz;

      /* check m is in valid range */
      if ((m < DSI_MIN_PLL_M) || (m > DSI_MAX_PLL_M)) {
        continue;
      }
      /* Store if closer to requested bit_clock */
      if (bit_clock_tmp_delta < bit_clock_best_delta) {
        p_best = p;
        m_best = (uint32_t)m;
        s_best = s;
        bit_clock_best_delta = bit_clock_tmp_delta;
        if (bit_clock_best_delta == 0U) {
          /* Exact match found, leave the loop */
          break;
        }
      }
    }
    if (bit_clock_best_delta == 0U) {
      /* Exact match found, leave the loop */
      break;
    }
  }
  if (m_best == 0) {
    DEBUG ((DEBUG_ERROR, "Failed to compute DPHY PLL for requested bit clock %d kHz.\n", bitClockKhz));
    return EFI_DEVICE_ERROR;
  }
  bit_clock_real_tmp = (((phyRefClockHz * m_best) / p_best) >> s_best) / 1000U;
  if (bitClockRealKhz != NULL) {
    *bitClockRealKhz = bit_clock_real_tmp;
  }
  if (bestDelta != NULL) {
    *bestDelta = (uint32_t)bit_clock_best_delta;
  }
  *pmsk = DSI_PLLCTRL_P(p_best) | DSI_PLLCTRL_M(m_best) | DSI_PLLCTRL_S(s_best);
  DEBUG ((DEBUG_INFO, "MIPI-DSI pll params: ref_clock=%d p=%d m=%d s=%d\n",
        phyRefClockHz, p_best, m_best, (1U << s_best)));
  DEBUG ((DEBUG_INFO, "MIPI-DSI PHY bit clock (phy pll) = %d kHz (requested bit clock = %d kHz, diff = %lu)\n",
        bit_clock_real_tmp, bitClockKhz, bit_clock_best_delta));
  return EFI_SUCCESS;
}

/**
  Find nearest MIPI DSI DPHY timing parameters for required bit clock.

  @param key Required bit clock.
  @param match Pointer to item of the array containing DPHY PLL params.

  @return EFI_SUCCESS if DPHY timing is found, EFI_DEVICE_ERROR if not.
**/
static EFI_STATUS MipiDsiGetPhyTiming(uint32_t key, struct DSI_DPHY_TIMING **match)
{
  EFI_STATUS status = EFI_DEVICE_ERROR;
  int size;
  size = sizeof(dphyTiming) / sizeof(dphyTiming[0]);
  for(int idx = 0; idx < size; idx++) {
    if (ABS_VAL(dphyTiming[idx].tmgBitClkMHZ - key) <= 5) {
      *match = &dphyTiming[idx];
      status = EFI_SUCCESS;
      break;
    }
  }
  return status;
}

/**
  Reset MIPI DSI block.
**/
EFI_STATUS
MipiDsiConfig (
  IMX_DISPLAY_TIMING* Timing,
  imxConverter MipiDsiConverter
  )
{
  uint8_t lanes;
  uint32_t bp,fp;
  uint32_t bitClockKhz = 0;
  uint32_t bitClockMhz = 0;
  uint32_t phyRefClockHz = 0;
  uint32_t hsaWC, hbpWC, hfpWC;
  uint32_t pmskVal = 0;
  uint32_t regVal, regMask, loop;
  struct dsiHblankPar *hpar = NULL;
  EFI_STATUS status = EFI_SUCCESS;

  MipiDsiPktRegisterCallback(&MipiDsiPktSend);
  /* Dafault 4 MIPI DSI lanes */
  lanes = 4U;

  MipiDsiDisplayClockConfig(Timing);

  /* Calculate bitclock  */
  if ((MipiDsiBitClockCalc(&bitClockKhz, Timing->PixelClock, Timing->Bpp, lanes, MipiDsiConverter) !=  EFI_SUCCESS) || (bitClockKhz == 0U)) {
    DEBUG ((DEBUG_ERROR, "Error calculating bit clock.\n"));
    return EFI_DEVICE_ERROR;
  }

  /* Get phy ref clock for given pixelclock */
  if ((CcmGetDsiPhyRefClk(Timing, &phyRefClockHz) != EFI_SUCCESS) || (phyRefClockHz == 0)) {
    DEBUG ((DEBUG_ERROR, "MIPI DSI PHY REF CLOCK not found.\n"));
    return EFI_DEVICE_ERROR;
  }

  /* Calculate pll params here */
  if (MipiDsiPllParamCalc(bitClockKhz, phyRefClockHz, &pmskVal, NULL, &bitClockKhz) != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "Error calculating dsi phy pll parameters for bit clock %d kHz and dsi phy ref clock %d Hz.",
          bitClockKhz, phyRefClockHz));
    return EFI_DEVICE_ERROR;
  }

  /* Mask all MIPI DSI interrupts */
  MIPI_DSI_DSI_INTMSK = 0xFB37FFFFU; 

  /* Main display image resolution */ 
  MIPI_DSI_DSI_MDRESOL = MIPI_DSI_DSI_MDRESOL_MainVResol(Timing->VActive) | MIPI_DSI_DSI_MDRESOL_MainHResol(Timing->HActive);

  /* Vertical front & back porches calculation */
  fp = Timing->VSyncOffset;
  bp = Timing->VBlank - Timing->VSyncOffset - Timing->VSync;
  MIPI_DSI_DSI_MVPORCH = MIPI_DSI_DSI_MVPORCH_CmdAllow(0U) | MIPI_DSI_DSI_MVPORCH_StableVfp(fp) | MIPI_DSI_DSI_MVPORCH_MainVbp(bp);

  /* Horizontal front & back porches calculation */
  fp = Timing->HSyncOffset;
  bp = Timing->HBlank - Timing->HSyncOffset - Timing->HSync;
  if (MipiDsiConverter == ADV7535) {
    /* For ADV7535 converter hblank parameters needs to be adjusted */
    hpar = MipiDsiGetHBlankParams(Timing, lanes);
  }

  if (hpar == NULL) {
    DEBUG ((DEBUG_INFO, "Horizontal porches manual calculation\n"));
    regVal = DIV_RD_UP(Timing->HSync * (Timing->Bpp >> 3U), lanes);
    hsaWC = (regVal > DSI_HSA_PKT_OVERHEAD) ? (regVal - DSI_HSA_PKT_OVERHEAD) : Timing->HSync;
    regVal = DIV_RD_UP(fp * (Timing->Bpp >> 3U), lanes);
    hfpWC = (regVal > DSI_HFP_PKT_OVERHEAD) ? (regVal - DSI_HFP_PKT_OVERHEAD) : fp;
    regVal = DIV_RD_UP(bp * (Timing->Bpp >> 3U), lanes);
    hbpWC = (regVal > DSI_HBP_PKT_OVERHEAD) ? (regVal - DSI_HBP_PKT_OVERHEAD) : bp;
  } else {
    /* Otherwise take adjusted hblank params */
    hsaWC = hpar->hsaWC;
    hfpWC = hpar->hfpWC;
    hbpWC = hpar->hbpWC;
    DEBUG ((DEBUG_INFO, "ADV7535 exceptions used\n"));
  }

  MIPI_DSI_DSI_MHPORCH = MIPI_DSI_DSI_MHPORCH_MainHbp(hbpWC) | MIPI_DSI_DSI_MHPORCH_MainHfp(hfpWC);
  MIPI_DSI_DSI_MSYNC = MIPI_DSI_DSI_MSYNC_MainVsa(Timing->VSync) | MIPI_DSI_DSI_MSYNC_MainHsa(hsaWC);

  /* DPI config */
  MIPI_DSI_DSI_RGB_STATUS = 0U;
  regVal = (MIPI_DSI_DSI_CONFIG_EoT_r03_MASK | 
        MIPI_DSI_DSI_CONFIG_VideoMode_MASK |
        MIPI_DSI_DSI_CONFIG_SyncInform_MASK | 
        MIPI_DSI_DSI_CONFIG_HseDisableMode_MASK |
        MIPI_DSI_DSI_CONFIG_MainVc(0U));

  regVal |= MIPI_DSI_DSI_CONFIG_MainPixFormat(7U);

  regVal |= MIPI_DSI_DSI_CONFIG_NumOfDatLane(lanes - 1U);
  regVal |= (((1U << lanes) - 1U) << 1U) | 1U;
  MIPI_DSI_DSI_CONFIG = regVal;

  /* DSIM PLL */
  MIPI_DSI_DSI_PLLTMR = 0x8000U;

  MIPI_DSI_DSI_PLLCTRL = regVal = 0U;
  regVal = MIPI_DSI_DSI_PLLCTRL_PMS(pmskVal);
  regVal |= MIPI_DSI_DSI_PLLCTRL_PllEn_MASK;
  MIPI_DSI_DSI_PLLCTRL = regVal;

  /* Wait until PLL stable, timeout 1000us*/
  loop = 0;
  while ((MIPI_DSI_DSI_STATUS & MIPI_DSI_DSI_STATUS_PllStable_MASK) == 0) {
    MicroSecondDelay(10U);
    loop++;
    if (loop > 100U) {
      DEBUG ((DEBUG_ERROR, "Wait on MIPI_DSI_DSI_STATUS_PllStable failed.\n"));
      return EFI_DEVICE_ERROR;
    }
  }

  /* Wait until clk and data lane go to stop state, timeout 10000us */
  loop = 0;
  while ((MIPI_DSI_DSI_STATUS & MIPI_DSI_DSI_STATUS_StopstateDat((1U << lanes) - 1U)) == 0) {
    MicroSecondDelay(10U);
    loop++;
    if (loop > 1000U) {
      DEBUG ((DEBUG_ERROR, "Wait on MIPI_DSI_DSI_STATUS_StopstateDat failed.\n"));
      return EFI_DEVICE_ERROR;
    }
  }

  /* DPHY timing */
  struct DSI_DPHY_TIMING *match = NULL;
  bitClockMhz = DIV_RD_CLOSEST(bitClockKhz, 1000);
  status = MipiDsiGetPhyTiming(bitClockMhz, &match);
  if(status != EFI_SUCCESS || match == NULL) {
    DEBUG ((DEBUG_ERROR, "DPHY timing not found\n"));
    return status;
  }

  DEBUG ((DEBUG_INFO, "DPHY timing found\n"));
  DEBUG ((DEBUG_INFO, "match->tmgBitClkMHZ  = %d\n", match->tmgBitClkMHZ));
  DEBUG ((DEBUG_INFO, "match->tmgClkPrepare = %d\n", match->tmgClkPrepare));
  DEBUG ((DEBUG_INFO, "match->tmgClkZero =  %d\n", match->tmgClkZero));
  DEBUG ((DEBUG_INFO, "match->tmgClkPost =  %d\n", match->tmgClkPost));
  DEBUG ((DEBUG_INFO, "match->tmgClkTrail =   %d\n", match->tmgClkTrail));
  DEBUG ((DEBUG_INFO, "match->tmgHsPrepare =  %d\n", match->tmgHsPrepare));
  DEBUG ((DEBUG_INFO, "match->tmgHsZero =   %d\n", match->tmgHsZero));
  DEBUG ((DEBUG_INFO, "match->tmgHsTrail =  %d\n", match->tmgHsTrail));
  DEBUG ((DEBUG_INFO, "match->tmgLpx =    %d\n", match->tmgLpx));
  DEBUG ((DEBUG_INFO, "match->tmgHsExit =   %d\n", match->tmgHsExit));

  MIPI_DSI_DSI_PHYTIMING = MIPI_DSI_DSI_PHYTIMING_M_TLPXCTL(match->tmgLpx) |
               MIPI_DSI_DSI_PHYTIMING_M_THSEXITCTL(match->tmgHsExit);
  MIPI_DSI_DSI_PHYTIMING1 = MIPI_DSI_DSI_PHYTIMING1_M_TCLKTRAILCTL(match->tmgClkTrail) |
                MIPI_DSI_DSI_PHYTIMING1_M_TCLKPOSTCTL(match->tmgClkPost) |
                MIPI_DSI_DSI_PHYTIMING1_M_TCLKZEROCTL(match->tmgClkZero) |
                MIPI_DSI_DSI_PHYTIMING1_M_TCLKPRPRCTL(match->tmgClkPrepare);
  MIPI_DSI_DSI_PHYTIMING2 = MIPI_DSI_DSI_PHYTIMING2_M_THSTRAILCTL(match->tmgHsTrail) |
                MIPI_DSI_DSI_PHYTIMING2_M_THSZEROCTL(match->tmgHsZero) |
                MIPI_DSI_DSI_PHYTIMING2_M_THSPRPRCTL(match->tmgHsPrepare);

  MIPI_DSI_DSI_TIMEOUT = MIPI_DSI_DSI_TIMEOUT_BtaTout(0xFFU) |
               MIPI_DSI_DSI_TIMEOUT_LpdrTout(0xFFU);

  /* Initialize FIFO pointers */
  regMask = (MIPI_DSI_DSI_FIFOCTRL_nInitRx_MASK |
         MIPI_DSI_DSI_FIFOCTRL_nInitSfr_MASK |
         MIPI_DSI_DSI_FIFOCTRL_nInitI80_MASK |
         MIPI_DSI_DSI_FIFOCTRL_nInitSub_MASK |
         MIPI_DSI_DSI_FIFOCTRL_nInitMain_MASK);
  MIPI_DSI_DSI_FIFOCTRL &= ~regMask;
  MicroSecondDelay(500U);
  MIPI_DSI_DSI_FIFOCTRL |= regMask;
  MicroSecondDelay(500U);


  /* Configure escape clock, byte clock */
  regVal = (MIPI_DSI_DSI_CLKCTRL_TxRequestHsClk_MASK |
       MIPI_DSI_DSI_CLKCTRL_EscClkEn_MASK |
       MIPI_DSI_DSI_CLKCTRL_ByteClkEn_MASK);

  regVal |= MIPI_DSI_DSI_CLKCTRL_LaneEscClkEn((((1U << lanes) - 1U) << 1U) | 1U);
  regMask = DIV_RD_UP((bitClockKhz >> 3U), DSI_MAX_ESC_CLOCK_KHZ);
  regVal |= MIPI_DSI_DSI_CLKCTRL_EscPrescaler(regMask);
  MIPI_DSI_DSI_CLKCTRL = regVal;

  /* Enable data transfer */
  MIPI_DSI_DSI_MDRESOL |= MIPI_DSI_DSI_MDRESOL_MainStandby_MASK;

  return EFI_SUCCESS;
}

EFI_STATUS
MipiDsiPktSend(uint8_t Type, uint8_t Chan, uint32_t Flg, const void *Data, uint16_t Size)
{
  uint32_t val, loop;
  uint32_t Hdr = 0;
  const uint8_t *CharData = Data;
  uint16_t PldSize;

  /* packet hdr */
  Hdr = ((Chan & 0x3) << 6) | (Type & 0x3F);
  if (MipiDsi_IsLong(Type)) {
    Hdr |= (Size & 0xFF) << 8;
    Hdr |= ((Size >> 8) & 0xFF) << 16;
    PldSize = Size;
  } else {
    if (Size > 0) {
      Hdr |= (CharData[0] << 8);
    }
    if (Size > 1) {
      Hdr |= (CharData[1] << 16);
    }
    PldSize = 0;
  }

  /* Low power TX */
  val = MIPI_DSI_DSI_ESCMODE;
  if (Flg & DSI_MODE_LP) {
    val |= MIPI_DSI_DSI_ESCMODE_CmdLpdt_MASK;
  } else {
    val &= ~MIPI_DSI_DSI_ESCMODE_CmdLpdt_MASK;
  }
  MIPI_DSI_DSI_ESCMODE = val;

  val = MIPI_DSI_DSI_INTSRC_SFRPLFifoEmpty_MASK | MIPI_DSI_DSI_INTSRC_SFRPHFifoEmpty_MASK;
  MIPI_DSI_DSI_INTSRC = val;

  if (PldSize > 0) {
    while (PldSize >= 4) {
      val = (uint32_t) CharData[0];
      val |= (CharData[1] << 8);
      val |= (CharData[2] << 16);
      val |= (CharData[3] << 24);
      MIPI_DSI_DSI_PAYLOAD = val;
      PldSize -=4;
      CharData += 4;
    }
    val = 0;
    switch (PldSize) {
      case 3:
        val |= CharData[2] << 16;
        /* Fall through */
      case 2:
        val |= CharData[1] << 8;
        /* Fall through */
      case 1:
        val |= CharData[0];
        MIPI_DSI_DSI_PAYLOAD = val;
        break;
      default:
        break;
    }
  }

  /* Write header */
  MIPI_DSI_DSI_PKTHDR = Hdr;

  /* Wait packet done */
  loop = 0;
  while ((MIPI_DSI_DSI_INTSRC & MIPI_DSI_DSI_INTSRC_SFRPHFifoEmpty_MASK) == 0) {
    MicroSecondDelay(10U);
    loop++;
    if (loop > 10000U) {
      DEBUG ((DEBUG_ERROR, "MipiDsiPktSend: Wait for MIPI_DSI_DSI_INTSRC_SFRPHFifoEmpty failed.\n"));
      return EFI_DEVICE_ERROR;
    }
  }

  return EFI_SUCCESS;
}

/**
  Dump MIPI DSI registers.
**/
VOID
MipiDsiDump (
  VOID
  )
{
  DEBUG ((DEBUG_INFO, "------------------------MIPI DSI------------------------\n"));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_VERSION   = 0x%08X\n", MIPI_DSI_DSI_VERSION));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_STATUS    = 0x%08X\n", MIPI_DSI_DSI_STATUS));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_RGB_STATUS  = 0x%08X\n", MIPI_DSI_DSI_RGB_STATUS));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_SWRST     = 0x%08X\n", MIPI_DSI_DSI_SWRST));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_CLKCTRL   = 0x%08X\n", MIPI_DSI_DSI_CLKCTRL));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_TIMEOUT   = 0x%08X\n", MIPI_DSI_DSI_TIMEOUT));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_CONFIG    = 0x%08X\n", MIPI_DSI_DSI_CONFIG));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_ESCMODE   = 0x%08X\n", MIPI_DSI_DSI_ESCMODE));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_MDRESOL   = 0x%08X\n", MIPI_DSI_DSI_MDRESOL));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_MVPORCH   = 0x%08X\n", MIPI_DSI_DSI_MVPORCH));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_MHPORCH   = 0x%08X\n", MIPI_DSI_DSI_MHPORCH));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_MSYNC     = 0x%08X\n", MIPI_DSI_DSI_MSYNC));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_SDRESOL   = 0x%08X\n", MIPI_DSI_DSI_SDRESOL));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_INTSRC    = 0x%08X\n", MIPI_DSI_DSI_INTSRC));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_INTMSK    = 0x%08X\n", MIPI_DSI_DSI_INTMSK));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PKTHDR    = 0x%08X\n", MIPI_DSI_DSI_PKTHDR));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PAYLOAD   = 0x%08X\n", MIPI_DSI_DSI_PAYLOAD));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_RXFIFO    = 0x%08X\n", MIPI_DSI_DSI_RXFIFO));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_FIFOTHLD  = 0x%08X\n", MIPI_DSI_DSI_FIFOTHLD));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_FIFOCTRL  = 0x%08X\n", MIPI_DSI_DSI_FIFOCTRL));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_MEMACCHR  = 0x%08X\n", MIPI_DSI_DSI_MEMACCHR));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_MULTI_PKT   = 0x%08X\n", MIPI_DSI_DSI_MULTI_PKT));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PLLCTRL_1G  = 0x%08X\n", MIPI_DSI_DSI_PLLCTRL_1G));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PLLCTRL   = 0x%08X\n", MIPI_DSI_DSI_PLLCTRL));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PLLCTRL1  = 0x%08X\n", MIPI_DSI_DSI_PLLCTRL1));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PLLCTRL2  = 0x%08X\n", MIPI_DSI_DSI_PLLCTRL2));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PLLTMR    = 0x%08X\n", MIPI_DSI_DSI_PLLTMR));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PHYCTRL_B1  = 0x%08X\n", MIPI_DSI_DSI_PHYCTRL_B1));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PHYCTRL_B2  = 0x%08X\n", MIPI_DSI_DSI_PHYCTRL_B2));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PHYCTRL_M1  = 0x%08X\n", MIPI_DSI_DSI_PHYCTRL_M1));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PHYCTRL_M2  = 0x%08X\n", MIPI_DSI_DSI_PHYCTRL_M2));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PHYTIMING   = 0x%08X\n", MIPI_DSI_DSI_PHYTIMING));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PHYTIMING1  = 0x%08X\n", MIPI_DSI_DSI_PHYTIMING1));
  DEBUG ((DEBUG_INFO, "MIPI_DSI_DSI_PHYTIMING2  = 0x%08X\n", MIPI_DSI_DSI_PHYTIMING2));
  DEBUG ((DEBUG_INFO, "-----------------------------------------------------\n"));
}
