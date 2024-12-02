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
#pragma once

#ifndef _VPU_HW_DEFS_H_
#define _VPU_HW_DEFS_H_

/** Data synchronization macro */
#define DSB               _DataSynchronizationBarrier();
#define writel(v, r) (*((volatile UINT32 *)((r))) = (v))
#define readl(r)     (*((volatile UINT32 *)(r)))

typedef enum {
    DECODER = 0, /* DECODER ON CORE #0 */
    ENCODER = 1, /* ENCODER ON CORE #1 */
} operation_mode_t;

NTSTATUS init_vpu_core(uintptr_t FWBasePtr, ULONG FWSize, UINT32 FWBasePhy, uintptr_t csr_cpuwait, uintptr_t csr_offset,
                       int *reset);
NTSTATUS init_vpu_reg_memory(uintptr_t regs_base, operation_mode_t mode);
NTSTATUS reset_vpu_reg_memory(uintptr_t regs_base, operation_mode_t mode);

#endif