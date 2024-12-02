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
#ifndef _VPU_MEMORY_H_
#define _VPU_MEMORY_H_

#include <vpu_dec/vdec_cmn.h> /* for fbi_t structure */
#define MEM_ALIGN                   0x00010000
#define VPU_REG_BASE_MCORE          0x40000000
/* #define M0_C0_CSR_QXP               0x2D040000 // set from ACPI */
/* #define M0_C0_CSR_QM                0x2D080000 // set from ACPI */
/* #define M0_C1_CSR_QXP               0x2D050000 // set from ACPI */
/* #define M0_C1_CSR_QM                0x2D090000 // set from ACPI */
/* #define M0_C2_CSR_QM                0x2D0A0000 // set from ACPI */

#define VPU_REG_SIZE                0x01000000
#define MU_REG_SIZE                 0x00020000
#define VPU_FW_SIZE                 0x02000000
#define VPU_RPC_SIZE                0x00200000

/*#define M0_C0_BOOT_ADDR             0x80C00000 // set from ACPI */
/*#define M0_C0_BOOT_ADDR             0xF0100000 // set from ACPI */

#define MAX_FRAMES              40
#define UDATA_BUFFER_SIZE       0x1000

#define DCP_SIZE                0x3000000  /* 48 MB */

#define MAX_MBI_NUM             18
#define MAX_DCP_NUM             2

#define SB_CACHE_TYPE           MmWriteCombined

enum acpi_memory_resources {
    VPU_REGS_BASE_res_idx,
    VPU_MU0_BASE_res_idx,
    VPU_FW_BASE_res_idx,
    VPU_RPC_BASE_res_idx,
    VPU_BLIT_DC_res_idx,
    VPU_BLIT_PRG0_res_idx,
    VPU_BLIT_PRG1_res_idx,
    VPU_BLIT_DPR_CH0_res_idx,
    VPU_BLIT_DPR_CH1_res_idx,
    VPU_BLIT_LTS_res_idx,
    VPU_BLIT_STEER_res_idx
};

/* Memory descrioption structures */
typedef struct {
    UCHAR regs[VPU_REG_SIZE];
} MALONE_VPU_REGISTERS;

typedef struct {
    UCHAR mu_regs[MU_REG_SIZE];
} MALONE_MU_REGISTERS;

typedef struct {
    UCHAR vpu_fw[VPU_FW_SIZE];
} MALONE_VPU_FW;

typedef struct {
    UCHAR vpu_rpc[VPU_RPC_SIZE];
} MALONE_VPU_RPC;

/* Memory allocation list */
typedef struct _VpuMemory {
    struct _VpuMemory  *next;
    BYTE               *virtAddr;
    BYTE               *virtAddrAligned;
    uintptr_t physAddr;
    uintptr_t physAddrAligned;
    uintptr_t alignOffset;
    PMDL mdl;
    WDFFILEOBJECT file;
} VpuMemory;

/* S __FBL__ */
enum mpool_flags {
    MPOOL_NONE = 0,
    MPOOL_FREED = 1, /* Pool is marked for destruction. */
    MPOOL_HAVE_BUFFERS = 2, /*  this pool has registered frame ( screen)  buffers */
    MPOOL_STOPPING = 4, /*  stopping.  Any acquire should fail; */
};

/* Output Frame buffer */
typedef struct ofb {
    VpuMemory mem;     /* linear RBGA buffer */
    int index;         /* index of buffer */
    int stride;        /* output stride*/
    int width;         /* output width */
    int height;        /* output height */
    BOOLEAN acquired;  /* in display queue or held by client */
    BOOLEAN rdy_for_display;
    BOOLEAN held_by_client;
} ofb_t;

/* VPU Frame buffer */
typedef struct vfb {
    VpuMemory   luma;      /* tiled luma buffer */
    VpuMemory   chroma;    /* tiled chroma buffer*/
    int         index;           /* index of buffer */
    int         status;          /* vpu frame buffer status*/
    BOOLEAN     acquired;    /* set when buffer is taken from pool*/
    BOOLEAN     rdy_for_blit;
    BOOLEAN     held_by_g2d;
} vfb_t;

/* Frame buffer list */
typedef struct fbl {
    /* ctx info */
    INT             *stream_id;
    WDFFILEOBJECT   *file;
    BOOLEAN         *external;
    UINT32          *luma_size;
    UINT32          *chroma_size;
    UINT32          *fb_luma_size;
    UINT32          *fb_chroma_size;
    KEVENT          *cond;
    UINT32          *aborts;
    UINT32          *pending_vfb_req;
    UINT32          *pending_ofb_req;

    enum mpool_flags flags;     /* Pool flags */

    /* fbl info */
    BOOLEAN         vfb_waiting;    /* Waiting for VPU frame buffer */
    BOOLEAN         ofb_waiting;    /* Waiting for VPU frame buffer */

    int             vfb_index;      /* Search VPU buffer from this index in pool */
    int             ofb_index;      /* Search VPU buffer from this index in pool */

    size_t          vfb_capacity;   /* VPU frame buffer max capacity */
    size_t          ofb_capacity;   /* Output frame buffer max capacity */

    vfb_t           *vfb;           /* Vpu frame buffer elements in the pool */
    ofb_t           *ofb;           /* Ouptut frame buffer elements in the pool */

} fbl_t;

typedef struct {
    UINT32 head_ind;
    UINT32 tail_ind;
    UINT32 counter;
    ofb_t *storage[MAX_FRAMES];
} ofb_frame_q_t;

typedef struct {
    UINT32 head_ind;
    UINT32 tail_ind;
    UINT32 counter;
    vfb_t *storage[MAX_FRAMES];
} vfb_frame_q_t;

/* Memory structure for context */
typedef struct _CtxMemory {
    fbl_t       *fbl;
    UINT32 fb_width;
    UINT32 fb_height;
    UINT32 fb_luma_size;
    UINT32 fb_chroma_size;

    VpuMemory sb_mem;

    VpuMemory ud_mem;

    VpuMemory mbi_mem[MAX_MBI_NUM];
    UINT32 mbi_size;

    VpuMemory dcp_mem[MAX_DCP_NUM];
    UINT32 dcp_size;
} CtxMemory;

/* Function prototypes */
NTSTATUS alloc_vpu_buffer(VpuMemory *alloc, ULONG size, ULONG cacheType);
NTSTATUS map_vpu_buffer(VpuMemory *alloc, ULONG cacheType, WDFFILEOBJECT file);
void unmap_vpu_buffer(VpuMemory *buffer);
void vpu_unmap_ctx_memory(CtxMemory *CtxMem);

#endif
