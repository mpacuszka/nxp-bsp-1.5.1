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

#include "precomp.h"
#include "imxpepioctl.h"
#include "imxpephw.h"
#include "imxpep.h"
#include "imxpepstat.h"
#include "imxpep_dbg.h"

void IMX_PEP_STAT::Reset()
{
    RtlZeroMemory(this, sizeof(IMX_PEP_STAT));
}

void IMX_PEP_STAT::Count(ULONG platformStateId, ULONG cpuId, ULONG processorState, LONGLONG stateEntryTime)
{
    if (platformStateId == PEP_PLATFORM_IDLE_STATE_NONE)
    {
        platformStateId = IMX_PEP::PLATFORM_IDLE_STATE_COUNT;
    }
    else if (platformStateId > 2)
    {
        platformStateId = IMX_PEP::PLATFORM_IDLE_STATE_COUNT + 1;
    }

    if (cpuId > 3)
    {
        cpuId = 3;
    }

    IMX_PEP_STATE_COUNTERS* cnt = &Cnt[platformStateId][cpuId];

    LARGE_INTEGER  stateExitTime;
    KeQuerySystemTimePrecise(&stateExitTime);
    LONG stateDuration = (LONG)((stateExitTime.QuadPart - stateEntryTime) / 10000);

    switch (processorState)
    {
    case IMX_PEP::CPU_IDLE_STATE_WFI:
        cnt->WfiCount++;
        cnt->WfiTime += stateDuration;
        break;
    case IMX_PEP::CPU_IDLE_STATE_WFI2:
        cnt->Wfi2Count++;
        cnt->Wfi2Time += stateDuration;
        break;
    case IMX_PEP::CPU_IDLE_STATE_POWER_GATED:
        cnt->GatedCount++;
        cnt->GatedTime += stateDuration;
        break;
    default:
        cnt->UndefinedCpuStateCount++;
        cnt->UndefinedCpuStateTime++;
        break;
    }
}

/*typedef struct _IMXPEP_STAT_OUTPUT {
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

} IMXPEP_STAT_OUTPUT, * IMXPEP_STAT_OUTPUT;*/

void IMX_PEP_STAT::FillIoctlBuffer(IMXPEP_STAT_OUTPUT* buf)
{
    for (ULONG platformStateIdx = 0; platformStateIdx < _PEP_STATE_CNT; platformStateIdx++) {
        for (ULONG cpuIdx = 0; cpuIdx < _PEP_CPU_CNT; cpuIdx++) {
            auto& cnt = buf->PepCounters[platformStateIdx][cpuIdx];
            auto& src = this->Cnt[platformStateIdx][cpuIdx];

            cnt.WfiCount   = src.WfiCount;
            cnt.WfiTime    = src.WfiTime;
            cnt.Wfi2Count  = src.Wfi2Count;
            cnt.Wfi2Time   = src.Wfi2Time;
            cnt.GatedCount = src.GatedCount;
            cnt.GatedTime  = src.GatedTime;
            cnt.UndefinedCpuStateCount = src.UndefinedCpuStateCount;
            cnt.UndefinedCpuStateTime  = src.UndefinedCpuStateTime;
        }
    }
}
