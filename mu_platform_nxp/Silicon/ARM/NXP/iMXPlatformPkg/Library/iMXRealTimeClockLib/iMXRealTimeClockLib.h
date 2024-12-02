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

/**
    This file contains bit masks for working with the Secure Non-Volatile Storage (SNVS)
    in the i.MX SoC with relation to the real time clock.
*/

// SNVS Registers
#define iMX_SNVS_Base_Address                           0x30370000 //physical address
#define iMX_SNVS_HP_Length                              52         //Hi-Power only SNVS length in bytes
#define iMX_SNVS_LP_Length                              3068       //hi + Low-Power secure SNVS length in bytes
#define iMX_SNVS_HPCOMR                                 0x0004     //HP Command
#define iMX_SNVS_HPCR                                   0x0008     //HP Control
#define iMX_SNVS_HPSR                                   0x0014     //HP Status
#define iMX_SNVS_HPRTCMR                                0x0024     //RTC MSB 0-14
#define iMX_SNVS_HPRTCLR                                0x0028     //RTC LSB 0-31
#define iMX_SNVS_LPCR                                   0x0038     //LP Control
#define iMX_SNVS_LPSRTCMR                               0x0050     //SRTC MSB 0-14
#define iMX_SNVS_LPSRTCLR                               0x0054     //SRTC LSB 0-31

//Oscillator settings registers
#define iMX_CCM_ANALOG_OSC_MISC_CFG                     0x30360070

// Mask values
#define iMX_RTC_ENABLED                                 0x00000001  //also applies for SRTC as SRTC_ENV is in the same position
#define iMX_RESET_RTC_MSB                               0xFFFF8000
#define iMX_SNVS_TS_ENABLE                              0x00010000
#define iMX_SNVS_SW_VIO_RST                             0x00000010  //to reset SW violation bit

typedef enum {RTC, SRTC} TimerType;

