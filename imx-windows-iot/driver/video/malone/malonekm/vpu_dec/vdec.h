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

#ifndef _VDEC_H_
#define _VDEC_H_

#include "vdec_io.h"

#define FRAME_DEPTH             5


/* Function prototypes */
int GetStreamIDFromFile(MALONE_VPU_DEVICE_CONTEXT *dev, WDFFILEOBJECT FxFile);
void vpu_api_event_handler(MALONE_VPU_DEVICE_CONTEXT *dev, UINT32 uStrIdx, UINT32 uEvent, UINT32 *event_data);
NTSTATUS vdec_open(MALONE_VPU_DEVICE_CONTEXT *dev, WDFFILEOBJECT file, vdec_init_t *dec, vdec_mem_desc_t *mem);
NTSTATUS vdec_close(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id);
NTSTATUS vdec_stop(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, int block);
NTSTATUS vdec_status(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, vdec_status_t *status);
NTSTATUS vdec_decode(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, vdec_decode_t *dec);
NTSTATUS vdec_get_output(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, fbo_t *fbi);
NTSTATUS vdec_flush(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, vdec_flush_t *flush);
NTSTATUS vdec_clear_output(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, int clr_idx);
#endif