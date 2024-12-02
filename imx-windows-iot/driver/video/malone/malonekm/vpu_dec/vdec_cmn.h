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
#ifndef __VDEC_CMN__
#define __VDEC_CMN__

#ifndef _STDINT
    typedef __int8 int8_t;
    typedef __int16 int16_t;
    typedef __int32 int32_t;
    typedef __int64 int64_t;
    typedef unsigned __int8 uint8_t;
    typedef unsigned __int16 uint16_t;
    typedef unsigned __int32 uint32_t;
    typedef unsigned __int64 uint64_t;
#endif
#include <stdlib.h>


EXTERN_C_START

//#define VPU_COPY_FRAMEBUFFER

#define AOIFOURCC4( C0, C1, C2, C3 ) ( ((uint32_t)C0<<24) | ((uint8_t)C1<<16) | ((uint8_t)C2<<8) | ((uint8_t)C3) )
#define AOIFOURCC( FCC ) AOIFOURCC4( FCC[0], FCC[1], FCC[2], FCC[3] )

#define VPU_BLOCK              1
#define VPU_NOBLOCK            0
#define VPU_SB_MEMOBJ_PATH_LEN 256

#define ENDIAN_RET32(x)           (uint32_t)_byteswap_ulong(x)
#define ENDIAN_BE32(x)            ((uint32_t)(ENDIAN_RET32(x)))

/* eos markers */
#define EOS_MARKER_AVC              ENDIAN_BE32(0x0000010B)
#define EOS_MARKER_VC1              ENDIAN_BE32(0x0000010A)
#define EOS_MARKER_MPEG2            ENDIAN_BE32(0x000001CC)
#define EOS_MARKER_MPEG1            EOS_MARKER_MPEG2
#define EOS_MARKER_MPEG4            ENDIAN_BE32(0x000001B1)
#define EOS_MARKER_RV               ENDIAN_BE32(0x00000134)
#define EOS_MARKER_VP6              EOS_MARKER_RV
#define EOS_MARKER_VP8              EOS_MARKER_RV
#define EOS_MARKER_SPARK            EOS_MARKER_RV
#define EOS_MARKER_MJPEG            ENDIAN_BE32(0x0000FFEF)
#define EOS_MARKER1_HEVC            ENDIAN_BE32(0x0000014A)
#define EOS_MARKER2_HEVC            ENDIAN_BE32(0x20000000)

/*  abort markers */
#define ABORT_MARKER_AVC            ENDIAN_BE32(0x0000010B)
#define ABORT_MARKER_VC1            ENDIAN_BE32(0x0000010A)
#define ABORT_MARKER_MPEG2          ENDIAN_BE32(0x000001B7)
#define ABORT_MARKER_MPEG1          ABORT_MARKER_MPEG2
#define ABORT_MARKER_MPEG4          ENDIAN_BE32(0x000001B1)
#define ABORT_MARKER_RV             ENDIAN_BE32(0x00000134)
#define ABORT_MARKER_VP6            ABORT_MARKER_RV
#define ABORT_MARKER_VP8            ABORT_MARKER_RV
#define ABORT_MARKER_SPARK          ABORT_MARKER_RV
#define ABORT_MARKER_HEVC           ENDIAN_BE32(0x0000014A)

/*!
 * @brief  vpu memory description structure
 */
typedef struct {
    void *virtAddress;
    long long physAddress;
    ULONG size;
    ULONG flags;
} vdec_mem_desc_t;

/* Used when resmgr is providing ouput buffer info to client */
typedef struct fbo {
    int index;          /* Param in:  index of output frame buffer */
    int stride;         /* Param out: output stride */
    int width;          /* Param out: output width */
    int height;         /* Param out: output height */
    uintptr_t planes[1];  /* Param out: 0-luma Virt addr, 1-chroma Virt addr */
} fbo_t;

typedef  struct vdec_init {
    uint32_t fourcc;       /* Param in: Stream fourcc */
    uint32_t dis_reorder; /* Param in: Disable reordering */
    uint32_t frame_mode;  /* Param in: Enable frame mode */
    uint32_t force_fbufs_num; /* Param in: Optimize memory requirements */
} vdec_init_t;

enum vdec_error {
    VDEC_EOK,
    VDEC_LOW_INPUT_FRAMES,
    VDEC_LOW_OUTPUT_BUFFERS,
    VDEC_FATAL_ERROR,/* define any fatal error below and warning before */
    VDEC_FATAL_ERROR_UNSUPPORTED_STREAM,
    VDEC_FATAL_ERROR_NOMEM,
    VDEC_FATAL_ERROR_NOT_ENOUGH_REGISTERED_OBUFFERS,
    VDEC_FATAL_ERROR_FIRMWARE_EXCEPTION,
    VDEC_FATAL_ERROR_DCP_DISABLED_INF
};

static const char *vdec_error2str[] = {
    "VDEC_EOK",
    "VDEC_LOW_INPUT_FRAMES",
    "VDEC_LOW_OUTPUT_BUFFERS",
    "VDEC_FATAL_ERROR",
    "VDEC_FATAL_ERROR_UNSUPPORTED_STREAM",
    "VDEC_FATAL_ERROR_NOMEM",
    "VDEC_FATAL_ERROR_NOT_ENOUGH_REGISTERED_OBUFFERS",
    "VDEC_FATAL_ERROR_FIRMWARE_EXCEPTION",
    "VDEC_FATAL_ERROR_DCP_DISABLED_INF"
};


typedef  struct vdec_decode {
    uint32_t wptr;
    int eos;                /* Param in:  set when input stream eos is detected */
    int oindex;             /* Param out: decoded frame buffer index */
    int timeout;            /* Param in:  timeout to wait for decoded output */
    int width;              /* Param out: stream width */
    int height;             /* Param out: stream height */
    int frame_in_cnt;       /* Param In:  Frame counter must be updated in frame mode */
    enum vdec_error error;  /* Param out: detail error reporting */
    BOOLEAN trick_mode;        /* Param In: trick play mode */
} vdec_decode_t;

typedef  struct vdec_status {
    uint32_t stopped;       /* Param out: set when VPU detected stopped state */
    uint32_t eos;           /* Param out: set when VPU detects eos */
    uint32_t rptr;          /* Param out: bs buffer rptr */
    uint32_t wptr;          /* Param out: bs buffer wptr */
    uint32_t start;         /* Param out: bs bufffer start */
    uint32_t end;           /* Param out: bs buffer end */
    uint32_t parsed_frames; /* Param out: Frames consumed by VPU in frame mode */
} vdec_status_t;

typedef  struct vdec_flush {
    uint32_t wptr;             /* bs buffer wptr before adding abort padding */
    int32_t padding_size;      /* abort padding size , */
} vdec_flush_t;


/* IOCTL METHODS */
#define VPU_IOC_INIT                CTL_CODE(FILE_DEVICE_DEVAPI, 0x00, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define VPU_IOC_GETOUTPUT           CTL_CODE(FILE_DEVICE_DEVAPI, 0x01, METHOD_BUFFERED, FILE_ANY_ACCESS)
/* 2 RSVD */
#define VPU_IOC_DECODE              CTL_CODE(FILE_DEVICE_DEVAPI, 0x03, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define VPU_IOC_STATUS              CTL_CODE(FILE_DEVICE_DEVAPI, 0x04, METHOD_BUFFERED, FILE_ANY_ACCESS)
/* 5 FREE */
/* 6 FREE */
#define VPU_IOC_CLEAR               CTL_CODE(FILE_DEVICE_DEVAPI, 0x05, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define VPU_IOC_DEINIT              CTL_CODE(FILE_DEVICE_DEVAPI, 0x06, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define VPU_IOC_FLUSH               CTL_CODE(FILE_DEVICE_DEVAPI, 0x07, METHOD_BUFFERED, FILE_ANY_ACCESS)
EXTERN_C_END
#endif