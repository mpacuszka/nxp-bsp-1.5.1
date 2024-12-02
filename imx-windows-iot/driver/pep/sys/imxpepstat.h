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
*
* * Neither the name of the copyright holder nor the
*   names of its contributors may be used to endorse or promote products
*   derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/* Statistical counters related to Power Engine Plugin 
   
*/

#ifndef _IMXPEPSTAT_H
#define _IMXPEPSTAT_H

// Includes counters for undefined CPU/State
#define PEP_CPU_CNT 5
#define PEP_STATE_CNT 5

struct IMX_PEP_STATE_COUNTERS {
    ULONG WfiCount;
    ULONG WfiTime;
    ULONG Wfi2Count;
    ULONG Wfi2Time;
    ULONG GatedCount;
    ULONG GatedTime;
    ULONG UndefinedCpuStateCount;
    ULONG UndefinedCpuStateTime;
};

struct _IMXPEP_STAT_OUTPUT;

struct IMX_PEP_STAT {
    IMX_PEP_STATE_COUNTERS Cnt[PEP_STATE_CNT][PEP_CPU_CNT];

    void Reset();
    void Count(ULONG platformStateId, ULONG cpuId, ULONG processorState, LONGLONG stateEntryTime);
    void FillIoctlBuffer(_IMXPEP_STAT_OUTPUT*);
};

#endif
