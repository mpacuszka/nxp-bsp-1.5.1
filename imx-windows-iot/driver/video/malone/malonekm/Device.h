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
#ifndef _DEVICEXX_H_
#define _DEVICEXX_H_


#include <parent_device.h>
#include "vpu_hw/vpu_hw_defs.h"
#include "vpu_hw/vpu_memory.h"
#include <svc/scfw.h>
#include "vpu_hw/vpu_mu.h"
#include "vpu_rpc/vpu_rpc.h"


EXTERN_C_START

enum {
    ERR_OK = 0,
    ERR_VPU_CORE_TCTL_IO,
    ERR_VPU_CORE_SCI_FD,
    ERR_VPU_CORE_SC_OPEN,
    ERR_VPU_CORE_POWER,
    ERR_VPU_CORE_FIRMWARE_FILE,
    ERR_VPU_CORE_CPU_START,
    ERR_VPU_CORE_MMAP,
    ERR_VPU_PER_MMAP,
    ERR_VPU_MU_POWER,
    ERR_VPU_MU_CLOCK,
    ERR_VPU_PER_PWR,
    ERR_IPC_HANDLER,
};

enum e_blitter_status {
    BLIT_IDLE,
    BLIT_BUSY
};

/* Registry configuration */
typedef struct s_VpuConfig {
    ULONG MaxContexts;
    ULONG FrameBuffers;
    ULONG OutFrameBuffers;
    ULONG MaxWidth;
    ULONG MaxHeigth;
    ULONG StreamBuffSize;
    ULONG EnableHEVC;
    ULONG Allow10BitFormat;
} VpuConfig;

typedef struct s_MaloneVpuDeviceContext MALONE_VPU_DEVICE_CONTEXT;
typedef struct s_vdec_ctx vdec_ctx_t;

typedef struct {
    /* Current Blitter state and currently processed frame information. */
    enum e_blitter_status blitter_status;
    ofb_t *ofb;
    vfb_t *vfb;
    vdec_ctx_t *vdec_stream_ctx;

    /* Dpu Blitter registers */
    PHYSICAL_ADDRESS DcRegPhy;
    UINT32 *DcRegPtr;
    PHYSICAL_ADDRESS Prg0RegPhy;
    UINT32 *Prg0RegPtr;
    PHYSICAL_ADDRESS Prg1RegPhy;
    UINT32 *Prg1RegPtr;
    PHYSICAL_ADDRESS DprCh0RegPhy;
    UINT32 *DprCh0RegPtr;
    PHYSICAL_ADDRESS DprCh1RegPhy;
    UINT32 *DprCh1RegPtr;
    PHYSICAL_ADDRESS LtsRegPhy;
    UINT32 *LtsRegPtr;
    PHYSICAL_ADDRESS IrqSteerRegPhy;
    UINT32 *IrqSteerRegPtr;

    /* Device context and related WDF objects */
    MALONE_VPU_DEVICE_CONTEXT *DeviceContextPtr;
    WDFDPC Dpc;

    /* Measurement */
    LARGE_INTEGER  m_BlitStartTime;
    LARGE_INTEGER  m_BlitDoneIsrTime;
    LARGE_INTEGER  m_BlitDoneDpcTime;
    LONGLONG m_BlitDurationToIsrMs;
    LONGLONG m_BlitDurationToDpcMs;



} BLITTER_DEVICE_CONTEXT;


/* The device context performs the same job as */
/* a WDM device extension in the driver frameworks */
struct s_MaloneVpuDeviceContext {
    WDFDEVICE                   WdfDevice;
    WDFINTERRUPT                MUInterrupt;
    MALONE_VPU_REGISTERS        *RegistersPtr;
    PHYSICAL_ADDRESS            RegistersPhy;
    MALONE_MU_REGISTERS         *RegistersMUPtr;
    PHYSICAL_ADDRESS            RegistersMUPhy;
    MALONE_VPU_FW               *FWBasePtr;
    PHYSICAL_ADDRESS            FWBasePhy;
    MALONE_VPU_RPC              *RPCBasePtr;
    PHYSICAL_ADDRESS            RPCBasePhy;
    struct shared_addr          shared_mem;
    intptr_t                    csr_offset;
    intptr_t                    csr_cpuwait;
    sc_ipc_id_struct_t          scfw_ipc_id;
    mu_device_data_struct_t     MUDeviceContext;

    VpuConfig                   config;

    struct  s_vdec_ctx *ctx;

    /* HW State */
    WDFSPINLOCK                 mutex;     /* mutex to guard device access */
    WDFSPINLOCK                 rpc_mutex; /* mutex to guard RPC memory */

    unsigned char               dev_opened;
    UINT32                      vpu_fw_started;
    UINT32                      running;

    /* Blitter */
    BLITTER_DEVICE_CONTEXT      BlitterCtx;
};

struct s_vdec_ctx {
    MALONE_VPU_DEVICE_CONTEXT   *dev;
    CtxMemory                   CtxMem;

    WDFFILEOBJECT               file;
    INT                         stream_id;

    WDFSPINLOCK                 mutex;
    KEVENT                      cond;      /* condition event for signaling */

    MEDIA_IP_FORMAT             video_format;

    int                         max_mbi_num;
    BOOL                        b_firstseq;
    UINT32                      parsed_frames;
    UINT32                      mbi_count; /* number of mbi buffers granted to VPU */
    UINT32                      dcp_count; /* number of dcp buffers granted to VPU */

    UINT32                      dis_reorder;
    int                         frame_mode;
    UINT32                      force_fbufs_num;
    int                         pic_hdr_cnt;

    int                         eos;
    UINT32                      frame_counter;
    UINT32                      frame_dec_counter;

    MediaIPFW_Video_SeqInfo     pSeqinfo[1];
    u_int32                     stride;
    UINT32                      width;
    UINT32                      height;
    UINT32                      real_width;
    UINT32                      real_height;

    enum vdec_error             error;
    enum abort_status           aborts;
    enum estatus                status;

    UINT32                      space;
    UINT32                      remains;
    UINT32                      started;
    UINT32                      oframe_with_client;
    UINT32                      oframe_with_driver;
    int                         no_out_cnt;

    /* S - Buffers size, but only for current context. Not valid of other stream */
    UINT32                      luma_size;
    UINT32                      chroma_size;
    UINT32                      chroma_height;
    UINT32                      req_mbi_size;
    UINT32                      pending_vfb_req;
    UINT32                      pending_ofb_req;
    ofb_frame_q_t               ofbq;
    vfb_frame_q_t               vfbq;
    /* E - FBL */

#if 0
    FILE *dbg_file;
#endif
};

WDF_DECLARE_CONTEXT_TYPE(vdec_ctx_t);

/* This macro will generate an inline function called DeviceGetContext */
/* which will be used to get a pointer to the device context memory */
/* in a type safe manner. */
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(MALONE_VPU_DEVICE_CONTEXT, DeviceGetContext)

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(MALONE_VPU_ISR_CONTEXT, InterruptGetContext)

EXTERN_C_END

#endif
