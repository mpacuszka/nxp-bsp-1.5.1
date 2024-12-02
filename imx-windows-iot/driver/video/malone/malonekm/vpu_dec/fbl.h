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
#ifndef __FBL__
#define __FBL__

#pragma once

#include "vpu_hw/vpu_memory.h"


/* Function prototypes */
fbl_t *fbl_create();
void fbl_destroy(fbl_t *list);
NTSTATUS fbl_vfb_update(fbl_t *list, size_t capacity);
NTSTATUS fbl_ofb_update(fbl_t *list, size_t capacity);
vfb_t *fbl_find_by_addr(fbl_t *list, uintptr_t luma_phy_addr);
vfb_t *fbl_find_by_index(fbl_t *list, int index);
void fbl_vfb_set_status(fbl_t *list, vfb_t *ele, int status);
void fbl_vfb_clear(fbl_t *list);
void fbl_vfb_release(fbl_t *list, vfb_t *ele);
void fbl_ofb_release(fbl_t *list, ofb_t *ele);
ofb_t *fbl_ofb_acquire(fbl_t *list);
vfb_t *fbl_vfb_acquire(fbl_t *list);
void fbl_log(fbl_t *list);
void fbl_start(fbl_t *list);
void fbl_stop(fbl_t *list);
BOOL fbl_vfb_waiting(fbl_t *list);
size_t fbl_vfb_size(fbl_t *list);
BOOL fbl_ofb_waiting(fbl_t *list);
size_t fbl_ofb_size(fbl_t *list);


#endif /* __FBL__*/
