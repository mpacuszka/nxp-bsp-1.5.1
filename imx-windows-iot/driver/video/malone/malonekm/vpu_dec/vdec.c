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
#include "Trace.h"
#if (MALONE_TRACE == MALONE_TRACE_WPP)
    #include "vdec.tmh"
#endif
#include "imx8q_driver.h"
#include "Device.h"
#include "vpu_dec/fbl.h"
#include "vdec.h"
#include "imxblit_public.h"
#include "imxdpuv1_registers.h"
#include "dpr_prg_registers.h"
// #include <windowsx.h> WINAPI inside driver?

#define REL_ON_REQ              0

#define DBG_UNIT_SIZE          7
#define VPU_FRMDBG_RAW_EN      1
#define VPU_FRMDBG_RAW_DIS     0

/* Error enum */
enum {
    ERR_EOK,
    ERR_ENOMEM,
    ERR_EINVAL,
};

/* Status to track abort */
enum abort_status {
    ABORT_INIT = 0,
    ABORT_PENDING = 1,  /* Abort cmd is sent to VPU */
    ABORT_DONE = 2,     /* Abort process is completed */
};

/* Status enum */
enum estatus {
    MEDIA_PLAYER_STARTING,
    MEDIA_PLAYER_STARTED,
    MEDIA_PLAYER_STOPPING,
    MEDIA_PLAYER_STOPPED
};

/* Array of strings of status */
static char *sts2str[] = {
    "MEDIA_PLAYER_STARTING",
    "MEDIA_PLAYER_STARTED",
    "MEDIA_PLAYER_STOPPING",
    "MEDIA_PLAYER_STOPPED"
};

/* Array of strings of TB_API_DEC_FMT formats. */
static char *fmt2str[] = {
    "NULL",
    "AVC",
    "MP2",
    "VC1",
    "AVS",
    "ASP",
    "JPG",
    "RV",
    "VP6",
    "SPK",
    "VP8",
    "HEVC"
};

/* Array of strings of RPC commands. */
static char *cmd2str[] = {
    "VID_API_CMD_NULL",   /*0x0*/
    "VID_API_CMD_PARSE_NEXT_SEQ", /*0x1*/
    "VID_API_CMD_PARSE_NEXT_I",
    "VID_API_CMD_PARSE_NEXT_IP",
    "VID_API_CMD_PARSE_NEXT_ANY",
    "VID_API_CMD_DEC_PIC",
    "VID_API_CMD_UPDATE_ES_WR_PTR",
    "VID_API_CMD_UPDATE_ES_RD_PTR",
    "VID_API_CMD_UPDATE_UDATA",
    "VID_API_CMD_GET_FSINFO",
    "VID_API_CMD_SKIP_PIC",
    "VID_API_CMD_DEC_CHUNK",  /*0x0b*/
    "VID_API_CMD_UNDEFINED",
    "VID_API_CMD_UNDEFINED",
    "VID_API_CMD_UNDEFINED",
    "VID_API_CMD_UNDEFINED",
    "VID_API_CMD_START",         /*0x10*/
    "VID_API_CMD_STOP",
    "VID_API_CMD_ABORT",
    "VID_API_CMD_RST_BUF",
    "VID_API_CMD_UNDEFINED",
    "VID_API_CMD_FS_RELEASE",
    "VID_API_CMD_MEM_REGION_ATTACH",
    "VID_API_CMD_MEM_REGION_DETACH",
    "VID_API_CMD_MVC_VIEW_SELECT",
    "VID_API_CMD_FS_ALLOC",   /*0x19*/
    "VID_API_CMD_UNDEFINED",
    "VID_API_CMD_UNDEFINED",
    "VID_API_CMD_DBG_GET_STATUS", /*0x1C*/
    "VID_API_CMD_DBG_START_LOG",
    "VID_API_CMD_DBG_STOP_LOG",
    "VID_API_CMD_DBG_DUMP_LOG",
    "VID_API_CMD_YUV_READY",   /*0x20*/
};

/* Array of strings of RPC events. */
static char *event2str[] = {
    "VID_API_EVENT_NULL",  /*0x0*/
    "VID_API_EVENT_RESET_DONE",  /*0x1*/
    "VID_API_EVENT_SEQ_HDR_FOUND",
    "VID_API_EVENT_PIC_HDR_FOUND",
    "VID_API_EVENT_PIC_DECODED",
    "VID_API_EVENT_FIFO_LOW",
    "VID_API_EVENT_FIFO_HIGH",
    "VID_API_EVENT_FIFO_EMPTY",
    "VID_API_EVENT_FIFO_FULL",
    "VID_API_EVENT_BS_ERROR",
    "VID_API_EVENT_UDATA_FIFO_UPTD",
    "VID_API_EVENT_RES_CHANGE",
    "VID_API_EVENT_FIFO_OVF",
    "VID_API_EVENT_CHUNK_DECODED",  /*0x0D*/
    "VID_API_EVENT_UNDEFINED",
    "VID_API_EVENT_UNDEFINED",
    "VID_API_EVENT_REQ_FRAME_BUFF",  /*0x10*/
    "VID_API_EVENT_FRAME_BUFF_RDY",
    "VID_API_EVENT_REL_FRAME_BUFF",
    "VID_API_EVENT_STR_BUF_RST",
    "VID_API_EVENT_RET_PING",
    "VID_API_EVENT_QMETER",
    "VID_API_EVENT_STR_FMT_CHANGED",
    "VID_API_EVENT_FIRMWARE_XCPT",
    "VID_API_EVENT_START_DONE",
    "VID_API_EVENT_STOPPED",
    "VID_API_EVENT_ABORT_DONE",
    "VID_API_EVENT_FINISHED",
    "VID_API_EVENT_DBG_STAT_UPDATE",
    "VID_API_EVENT_DBG_LOG_STARTED",
    "VID_API_EVENT_DBG_LOG_STOPPED",
    "VID_API_EVENT_DBG_LOG_UPFATED",
    "VID_API_EVENT_DBG_MSG_DEC",  /*0x20*/
    "VID_API_EVENT_DEC_SC_ERR",
    "VID_API_EVENT_CQ_FIFO_DUMP",
    "VID_API_EVENT_DBG_FIFO_DUMP",
    "VID_API_EVENT_DEC_CHECK_RES",
    "VID_API_EVENT_DEC_CFG_INFO",  /*0x25*/
    "VID_API_EVENT_UNSUPPORTED_STREAM", /*0x26*/
    "VID_API_EVENT_UNKNOWN",  /*0x27*/
    "VID_API_EVENT_UNKNOWN",  /*0x28*/
    "VID_API_EVENT_UNKNOWN",  /*0x29*/
    "VID_API_EVENT_STR_SUSPENDED",  /*0x30*/
    "VID_API_EVENT_SNAPSHOT_DONE",  /*0x31*/
    "VID_API_EVENT_INVALID",  /*0x32*/
};

static void vdec_ofb_update(vdec_ctx_t *ctx, ofb_t *ofb);
static void vdec_serve_pending_vfb(vdec_ctx_t *ctx);

volatile int spinlock_counter = 0;
#if 0

static inline BOOLEAN ADAMSpinLockIsAcquire(WDFSPINLOCK spinlock)
{
    UNREFERENCED_PARAMETER(spinlock);
    if (spinlock_counter) {
        return TRUE;
    }
    return FALSE;
}

static inline void WdfSpinLockAcquire(WDFSPINLOCK spinlock)
{
    WdfSpinLockAcquire(spinlock);
    spinlock_counter++;
    if (spinlock_counter > 1) {
        DBG_PRINT_ERROR("!!!!!spinlock WARNING! = %d", spinlock_counter);

    }
    DBG_PRINT_ERROR("+++++spinlock acquired counter = %d", spinlock_counter);
    return;
}

static inline void WdfSpinLockRelease(WDFSPINLOCK spinlock)
{
    spinlock_counter--;
    DBG_PRINT_ERROR("-----spinlock released counter = %d", spinlock_counter);
    WdfSpinLockRelease(spinlock);

    return;
}
#endif

/*****************************************************************************/
/**
 * frame_status_to_str()
 * @brief Converts Frame buffer status to string
 * @param status Frame buffer status
 * @return  char* string String with status
 */
static const char *frame_status_to_str(int status)
{
    switch (status) {
        case FRAME_ALLOC:
            return "FRAME_ALLOC";
            break;
        case FRAME_FREE:
            return "FRAME_FREE";
            break;
        case FRAME_DECODED:
            return "FRAME_DECODED";
            break;
        case FRAME_READY:
            return "FRAME_READY";
            break;
        case FRAME_RELEASE:
            return "FRAME_RELEASE";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

/* Status enum */
enum etrap {
    TRAP_OK = 0,
    TRAP_NO_FREE_VFB,
    TRAP_NO_FREE_OFB,
    TRAP_NO_FREE_G2D
};

/* Status enum */
static const char *trap2str[] = {
    "TRAP_OK",
    "TRAP_NO_FREE_VFB",
    "TRAP_NO_FREE_OFB",
    "TRAP_NO_FREE_G2D"

};
/**
 * it_is_trap()
 * @brief
 * @param reason
 */
void its_a_trap(int reason)
{
    int keep_cycling = 1;
    if (reason == TRAP_OK) {
        return;
    } else {
        DBG_PRINT_ERROR("CODE TRAP: reason code: %d, reason string: %s", reason, trap2str[reason]);
        while (keep_cycling) {
            DbgBreakPointWithStatus(reason);
            keep_cycling = 0;
        };
    }
    DBG_VDEC_METHOD_END();

}

/**
 * dbglog_show()
 * @brief Prints dbg log from FW
 * @param ctx Decoder context
 * @param vpu_frmdbg_raw
 * @return
 */
static int dbglog_show(vdec_ctx_t *ctx, int vpu_frmdbg_raw)
{
    UNREFERENCED_PARAMETER(ctx);
    UNREFERENCED_PARAMETER(vpu_frmdbg_raw);
    return 0;
#if 0
    vpu_dev_t *dev = ctx->dev;
    u_int32 *pbuf;
    u_int32 line;
    int length;

    pbuf = (u_int32 *)dev->shared_mem.dbglog_mem_vir;
    line = (DBGLOG_SIZE) / (DBG_UNIT_SIZE * sizeof(u_int32));
    if (!line) {
        return 0;
    }

    if (!vpu_frmdbg_raw) {
        u_int32 i;

        /* length = 9 * DBG_UNIT_SIZE * line + 1; */
        for (i = 0; i < line; i++) {
            fprintf(ctx->dbg_file, "%08x %08x %08x %08x %08x %08x %08x\n",
                    pbuf[0], pbuf[1], pbuf[2], pbuf[3],
                    pbuf[4], pbuf[5], pbuf[6]);
            pbuf += DBG_UNIT_SIZE;
        }

        return 0;
    }

    length = DBG_UNIT_SIZE * sizeof(u_int32) * line;

    return fwrite((void *)pbuf, sizeof(u_int32), length, ctx->dbg_file);
#endif
}

/**
 * pop_vfb_entry()
 * @brief Pop frame entry from queue
 * @param theq : pointer to frame queue
 * @param fe : pointer to frame entry
 * @return Local ERR code;
 */
static int32_t pop_vfb_entry(vfb_frame_q_t *theq, vfb_t **fe)
{
    DBG_VDEC_METHOD_BEG();
    if (theq->tail_ind == theq->head_ind) {
        /*  No entries. */
        /*DBG_PRINT_ERROR("NO ENTRIES IN VFB QUEUE"); */
        return ERR_EINVAL;
    }
    *fe = theq->storage[theq->tail_ind];
    theq->tail_ind++;
    theq->counter--;
    if (theq->tail_ind == MAX_FRAMES) {
        theq->tail_ind = 0;
    }
    DBG_VDEC_METHOD_END();
    return ERR_EOK;
}

/**
 * push_vfb_entry()
 * @brief Push frame entry to queue
 * @param theq : pointer to frame queue
 * @param fe : pointer to frame entry
 * @return Local ERR code;
 */
static int32_t push_vfb_entry(vfb_frame_q_t *theq, vfb_t *fe)
{
    DBG_VDEC_METHOD_BEG();
    /* */
    /*  Make sure we have space. */
    if ((theq->tail_ind == (theq->head_ind + 1)) ||
        ((theq->tail_ind == 0) && (theq->head_ind == (MAX_FRAMES - 1)))) {
        DBG_PRINT_ERROR("NO SPACE IN VFB QUEUE");
        return ERR_ENOMEM;
    }
    theq->storage[theq->head_ind] = fe;
    fe->rdy_for_blit = TRUE;
    theq->head_ind++;
    theq->counter++;
    if ((theq->head_ind) == MAX_FRAMES) {
        theq->head_ind = 0;
    }
    DBG_VDEC_METHOD_END();
    return ERR_EOK;
}

/**
 * pop_ofb_entry()
 * @brief Pop frame entry from queue
 * @param theq : pointer to frame queue
 * @param fe : pointer to frame entry
 * @return Local ERR code;
 */
static int32_t pop_ofb_entry(ofb_frame_q_t *theq, ofb_t **fe)
{
    DBG_VDEC_METHOD_BEG();
    /* */
    if (theq->tail_ind == theq->head_ind) {
        /*  No entries. */
        /*DBG_PRINT_ERROR("NO ENTRIES IN OFB QUEUE"); */
        return ERR_EINVAL;
    }
    *fe = theq->storage[theq->tail_ind];
    theq->tail_ind++;
    theq->counter--;
    if (theq->tail_ind == MAX_FRAMES) {
        theq->tail_ind = 0;
    }
    DBG_VDEC_METHOD_END();
    return ERR_EOK;
}

/**
 * push_ofb_entry()
 * @brief Push frame entry to queue
 * @param theq : pointer to frame queue
 * @param fe : pointer to frame entry
 * @return Local ERR code;
 */
static int32_t push_ofb_entry(ofb_frame_q_t *theq, ofb_t *fe)
{
    DBG_VDEC_METHOD_BEG();
    /* */
    /*  Make sure we have space. */
    if ((theq->tail_ind == (theq->head_ind + 1)) ||
        ((theq->tail_ind == 0) && (theq->head_ind == (MAX_FRAMES - 1)))) {
        DBG_PRINT_ERROR("NO SPACE IN OFB QUEUE");
        return ERR_ENOMEM;
    }
    theq->storage[theq->head_ind] = fe;
    fe->rdy_for_display = TRUE;
    theq->head_ind++;
    theq->counter++;
    if ((theq->head_ind) == MAX_FRAMES) {
        theq->head_ind = 0;
    }
    DBG_VDEC_METHOD_END();
    return ERR_EOK;
}

/**
 * vpu_log_cmd()
 * @brief Logs vpu RPC commands
 * @param stream_id
 * @param cmdid
 */
static void vpu_log_cmd(int stream_id, u_int32 cmdid)
{
    UNREFERENCED_PARAMETER(stream_id);
    if (cmdid >= (sizeof(cmd2str) / sizeof(cmd2str[0]))) {
        DBG_EVENTS_PRINT_INFO("KM->VPU ctx[%d] send cmd: 0x%X ", stream_id, cmdid);
    } else {
        DBG_EVENTS_PRINT_INFO("KM->VPU ctx[%d] send cmd: 0x%X - %s ", stream_id, cmdid, cmd2str[cmdid]);
    }
}

/**
 * vpu_send_cmd()
 * @brief Unmap allocated buffers in a context
 * @param deviceContextPtr : Pointer to MALONE_VPU_DEVICE_CONTEXT
 * @param idx : Stream index
 * @param cmdid : Command id
 * @param cmdnum : Command number
 * @param local_cmddata* : Data pointer
 */
void vpu_send_cmd(MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr, UINT32 idx, UINT32 cmdid, UINT32 cmdnum,
                  UINT32 *local_cmddata)
{
    WdfSpinLockAcquire(deviceContextPtr->mutex);
    rpc_send_cmd_buf(&deviceContextPtr->shared_mem, idx, cmdid, cmdnum, local_cmddata);
    vpu_log_cmd(idx, cmdid);
    while (STATUS_DEVICE_BUSY == MU_SendMsg((MU_Type *)deviceContextPtr->RegistersMUPtr, MSG_TYPE, COMMAND));
    WdfSpinLockRelease(deviceContextPtr->mutex);
}

/**
 * vpu_log_event()
 * @brief Logs vpu RPC events
 * @param stream_id : Stream index
 * @param uEvent : Event index
 */
static void vpu_log_event(int stream_id, u_int32 uEvent)
{
    UNREFERENCED_PARAMETER(stream_id);
    if (uEvent >= (sizeof(event2str) / sizeof(event2str[0]))) {
        DBG_EVENTS_PRINT_INFO("VPU->KM ctx[%d] receive event: 0x%X ", stream_id, uEvent);
    } else {
        DBG_EVENTS_PRINT_INFO("VPU->KM ctx[%d] receive event: 0x%X - %s ", stream_id, uEvent, event2str[uEvent]);
    }
}

/**
 * is_10bit_format()
 * @brief Check 10bit format
 * @param ctx* : Pointer to vdec_ctx_t struct
 * @return TRUE if 10 bit format, otherwise FALSE
 */
static BOOL is_10bit_format(vdec_ctx_t *ctx)
{
    if (ctx->pSeqinfo->uBitDepthLuma > 8) {
        return TRUE;
    }
    if (ctx->pSeqinfo->uBitDepthChroma > 8) {
        return TRUE;
    }
    return FALSE;
}

VOID WriteDCReg32(BLITTER_DEVICE_CONTEXT *blitterContextPtr, const UINT32 RegAddr, const UINT32 Val)
{
    volatile UINT32 *regPtr = blitterContextPtr->DcRegPtr;

    regPtr += RegAddr / 4;
    *regPtr = Val;
}

VOID ReadDCReg32(BLITTER_DEVICE_CONTEXT *blitterContextPtr, const UINT32 RegAddr, UINT32 *Val)
{
    volatile UINT32 *regPtr = blitterContextPtr->DcRegPtr;

    regPtr += RegAddr / 4;
    *Val = *regPtr;
}

/**
 * G2D DPU t2l function.
 * @param vdec_ctx_t* Pointer to vdec_ctx_t struct
 * @param vfb_t* Pointer to vfb_t struct
 * @retval 1 if error occures, otherwies EOK.
 */
int t2l_frame(vdec_ctx_t *ctx)
{
    DBG_PERF_METHOD_BEG();
#ifdef PROFILE_T2L
    struct timespec to;
    uint64_t start, end;
    uint64_t detile_time = 0;
#endif
    ofb_t *ofb = NULL;
    UNREFERENCED_PARAMETER(ofb);
    vfb_t *vfb = NULL;


    if (ctx->dev->BlitterCtx.blitter_status == BLIT_BUSY) {
        /* Check status of blitter */
        UINT32 Value = 0;
        /* for interrupt, use IMXDPUV1_COMCTRL_USERINTERRUPTCLEAR0, IMXDPUV1_COMCTRL_USERINTERRUPTSTATUS0 */
        ReadDCReg32(&ctx->dev->BlitterCtx, IMXDPUV1_COMCTRL_INTERRUPTSTATUS0, &Value);
        if (Value & (0x1U << 2U)) { // FIXME -> workaround for non working interrupt
            DBG_PERF_PRINT_ERROR("TRAP_MISSED BLITTER INTERRUPT!");
            WriteDCReg32(&ctx->dev->BlitterCtx, IMXDPUV1_COMCTRL_INTERRUPTCLEAR0, (0x1U << 2U));
            t2l_frame_done(ctx, ctx->dev->BlitterCtx.vfb, ctx->dev->BlitterCtx.ofb);
            DBG_EVENTS_PRINT_INFO("MISSED G2D ISR!!! Blit done, state: BUSY -> IDLE, ofb index=%d, vfb index=%d",
                                  ctx->dev->BlitterCtx.ofb->index, ctx->dev->BlitterCtx.vfb->index);
            ctx->dev->BlitterCtx.blitter_status = BLIT_IDLE;
        }
    }

    WdfSpinLockAcquire(ctx->mutex);
    if (ctx->dev->BlitterCtx.blitter_status == BLIT_IDLE) {
        /* Acquire OFB */
        ofb = fbl_ofb_acquire(ctx->CtxMem.fbl);
        /* Check if Output Frame buffer is available*/
        if (ofb != NULL) {
            /* Load resolution from CTX to OFB*/
            vdec_ofb_update(ctx, ofb);
            /* Pop VFB from QUEUE*/
            if (pop_vfb_entry(&ctx->vfbq, &vfb) == ERR_OK) {
                /* Check if VPU Frame buffer is available */
                if (vfb != NULL) {
                    /*
                     * TODO PERFORM CONVERSION T2L !
                     */
                    ASSERT(vfb->acquired);
                    vfb->rdy_for_blit = FALSE;
                    vfb->held_by_g2d = TRUE;
                    BlitStartBlit(ctx, ofb, vfb);
                } else {
                    /*its_a_trap(TRAP_NO_FREE_VFB); */
                    DBG_PERF_PRINT_ERROR("TRAP_NO_VFB_FOR_T2L!");
                }
            } else {
                /*fbl_log(ctx->CtxMem.fbl); */
                /* OFB acquired, but VFB is not available*/
                fbl_ofb_release(ctx->CtxMem.fbl, ofb);
            }
        } else {
            /*its_a_trap(TRAP_NO_FREE_OFB); */
            DBG_PERF_PRINT_ERROR("TRAP_NO_FREE_OFB!");
        }
    } else {
        /*its_a_trap(TRAP_NO_FREE_G2D); */
        DBG_PERF_PRINT_ERROR("G2D_BLITTER is BUSY!");
    }
    WdfSpinLockRelease(ctx->mutex);
    DBG_PERF_METHOD_END();
    return 0;
}

void t2l_frame_done(vdec_ctx_t *ctx, vfb_t *vfb, ofb_t *ofb)
{
    DBG_VDEC_METHOD_BEG();
    WdfSpinLockAcquire(ctx->mutex);
    if (ctx->CtxMem.fbl) {
        /*ctx->oframe_with_driver--; */
        fbl_vfb_release(ctx->CtxMem.fbl, vfb);
        ofb->rdy_for_display = TRUE;
        push_ofb_entry(&ctx->ofbq, ofb);
        vdec_serve_pending_vfb(ctx);
    }
    WdfSpinLockRelease(ctx->mutex);
    DBG_VDEC_METHOD_END();
}

/**
 * caculate_frame_size()
 * @brief Calculate frame size based on values given from decoder
 * @param ctx* : Pointer to vdec_ctx_t struct
 */
static void caculate_frame_size(vdec_ctx_t *ctx)
{
    u_int32 width = ctx->pSeqinfo[0].uHorDecodeRes;
    u_int32 height = ctx->pSeqinfo[0].uVerDecodeRes;
    u_int32 luma_size = 0;
    u_int32 chroma_size = 0;
    u_int32 chroma_height = 0;
    u_int32 uVertAlign = 512 - 1;
    u_int32 uAlign = 0x800 - 1;

    ctx->real_width = width;
    ctx->real_height = height;

    width = is_10bit_format(ctx) ? (width + ((width + 3) >> 2)) : width;
    width = ((width + uVertAlign) & ~uVertAlign);
    ctx->stride = width;

    height = ((height + uVertAlign) & ~uVertAlign);

    chroma_height = height >> 1;

    luma_size = width * height;
    chroma_size = width * chroma_height;
    ctx->height = height;
    ctx->chroma_height = chroma_height;
    ctx->width = width;

    DBG_VDEC_PRINT_INFO("ORG (%dx%d) Aligned (%dx%d) luma size %d chroma size %d", ctx->real_width, ctx->real_height,
                        width, height,
                        luma_size, chroma_size);

    ctx->luma_size = luma_size;
    ctx->chroma_size = chroma_size;

    ctx->req_mbi_size = (ctx->luma_size + ctx->chroma_size) / 4;
    ctx->req_mbi_size = ((ctx->req_mbi_size + uAlign) & ~uAlign);
    return;
}

/**
 * GetStreamIDFromFile()
 * @brief Return Stream ID from a WDF FILE
 * @param dev : pointer to device
 * @param FxFile : WDFFILEOBJECT
 * @return Stream ID
 */
int GetStreamIDFromFile(MALONE_VPU_DEVICE_CONTEXT *dev, WDFFILEOBJECT FxFile)
{
    int stream_id = -1;
    int found = 0;
    /* Find our  stream context*/
    for (int i = 0; i < (int)dev->config.MaxContexts; i++) {
        if (dev->ctx[i].file == FxFile) {
            stream_id = dev->ctx[i].stream_id;
            if (stream_id != i) {
                DBG_PRINT_ERROR("Stream_id = %d doesn´t correspond to ctx id = %d!", stream_id, i);
            } else {
                return i;
            }
        }
    }

    if (found != 1) {
        DBG_PRINT_ERROR("Unable to find a valid Stream_id!");
        return -1;
    }

    return -1;
}

/**
 * block()
 * @brief Blocking function and waiting for ctx->cond condition
 * @param ctx* : Pointer to vdec_ctx_t struct
 * @param ms : timeout in ms. Block without timeout when ms = 0;
 * @return NTSTATUS
 */
static NTSTATUS block(vdec_ctx_t *ctx, int ms)
{
    DBG_VDEC_METHOD_BEG();
    NTSTATUS Status = STATUS_SUCCESS;
    WdfSpinLockRelease(ctx->mutex);
    if (ms) {
        LARGE_INTEGER timeout = { 0 };
        timeout.QuadPart = (LONGLONG)(-10 * 1000 * ms);
        Status = KeWaitForSingleObject(&ctx->cond, Executive, KernelMode, FALSE, &timeout);
    } else {
        Status = KeWaitForSingleObject(&ctx->cond, Executive, KernelMode, FALSE, NULL);
    }
    if (Status != STATUS_SUCCESS) {
        DBG_PRINT_ERROR("ctx[%d] Block: KeWaitForSingleObject (0x%x)", ctx->stream_id, Status);
    }
    WdfSpinLockAcquire(ctx->mutex);

    DBG_VDEC_METHOD_END();
    return Status;
}

/**
 * vdec_ofb_update()
 * @brief Updates ouptut framebuffer values from ctx
 * @param ctx* : Pointer to vdec_ctx_t struct
 * @param fb : frame buffer
 */
static void vdec_ofb_update(vdec_ctx_t *ctx, ofb_t *ofb)
{
#if 0 /* Not used */
    ofb->fbo.top = 0;
    ofb->fbo.left = 0;
    ofb->fbo.right = ctx->pSeqinfo->uHorRes;
    ofb->fbo.bottom = ctx->pSeqinfo->uVerRes;
#endif
    ofb->stride = ctx->real_width;
    ofb->width = ctx->real_width;
    ofb->height = ctx->real_height;
    /* Workaround for interlaced video */
#ifdef VPU_COPY_FRAMEBUFFER
    if (!ctx->pSeqinfo->uProgressive) {
        ofb->height = ctx->real_height / 2;
    }
#endif
}

/**
 * vdec_buffer_status()
 * @brief Update Stream Buffer Status
 * @param ctx : Pointer to vdec_ctx_t struct
 * @param log : 1 log; 0 don´t log
 */
static void vdec_buffer_status(vdec_ctx_t *ctx, BOOL log)
{
    DBG_VDEC_METHOD_BEG();
    MALONE_VPU_DEVICE_CONTEXT *dev = ctx->dev;
    u_int32 uStrIdx = 0; /* Set to be default 0, FIX_ME later */
    pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc = (pSTREAM_BUFFER_DESCRIPTOR_TYPE)((uintptr_t)dev->RegistersPtr +
                                                                                  DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF *
                                                                                  (uintptr_t)ctx->stream_id);
    DSB;
    u_int32 wptr = pStrBufDesc->wptr;
    u_int32 rptr = pStrBufDesc->rptr;
    u_int32 start = pStrBufDesc->start;
    u_int32 end = pStrBufDesc->end;
    dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->stream_id][uStrIdx] = (VPU_REG_BASE_MCORE + DEC_MFD_XREG_SLV_BASE
                                                                                  + MFD_MCX + MFD_MCX_OFF * ctx->stream_id);

    if (wptr < rptr) {
        ctx->space = (rptr - wptr);
    } else {
        ctx->space = (end - wptr) + (rptr - start);
    }
    ctx->remains = dev->config.StreamBuffSize - ctx->space;
    if (log) {

        pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[ctx->stream_id];
        UNREFERENCED_PARAMETER(buffer_info);
        DBG_VDEC_PRINT_INFO("ctx[%d] (rptr 0x%8x wptr 0x%8x start 0x%8x end 0x%8x space %d remains %d) ( in %d rdy %d dec %d phc %d sppc %d)",
                            ctx->stream_id, rptr, wptr, start, end,
                            ctx->space, ctx->remains, buffer_info->stream_pic_input_count, ctx->frame_counter, ctx->frame_dec_counter,
                            ctx->pic_hdr_cnt, ctx->parsed_frames);
    }
    DBG_VDEC_METHOD_END();
}

/**
 * update_frame_level()
 * @brief Update frame level
 * @param ctx* : Pointer to vdec_ctx_t struct
 */
static void update_frame_level(vdec_ctx_t *ctx)
{
    DBG_VDEC_METHOD_BEG();
    MALONE_VPU_DEVICE_CONTEXT *dev = ctx->dev;
    if (ctx->frame_mode) {
        pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[ctx->stream_id];
        WdfSpinLockAcquire(ctx->mutex);
        ctx->parsed_frames = buffer_info->stream_pic_parsed_count;
        ctx->pic_hdr_cnt++;

        if (ctx->vfbq.counter == 0) {
            if ((buffer_info->stream_pic_input_count - ctx->parsed_frames) < FRAME_DEPTH) {
                KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
            }
        }
        vdec_buffer_status(ctx, 0);
        WdfSpinLockRelease(ctx->mutex);
    }
    DBG_VDEC_METHOD_END();
}

/**
 * vdec_clear()
 * @brief Clear decoder context
 * @param ctx* : Pointer to vdec_ctx_t struct
 */
static void vdec_clear(vdec_ctx_t *ctx)
{
    DBG_VDEC_METHOD_BEG();
    MALONE_VPU_DEVICE_CONTEXT *dev = ctx->dev;
    pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[ctx->stream_id];
    ctx->error = VDEC_EOK;
    ctx->eos = 0;
    ctx->pic_hdr_cnt = 0;
    buffer_info->stream_pic_end_flag = 0;
    buffer_info->stream_pic_input_count = 0;
    buffer_info->stream_pic_parsed_count = 0;
    ctx->frame_counter = 0;
    ctx->frame_dec_counter = 0;
    DBG_VDEC_METHOD_END();
}

/**
 * vdec_serve_pending_fb()
 * @brief Serve pending frame buffer requests, if any
 * @param ctx* : Pointer to vdec_ctx_t struct
 */
static void vdec_serve_pending_vfb(vdec_ctx_t *ctx)
{
    DBG_VDEC_METHOD_BEG();
    /* Serve all the pending buffer requests */
    while (ctx->pending_vfb_req) {
        u_int32 local_cmddata[10] = { 0 };
        vfb_t *ele = fbl_vfb_acquire(ctx->CtxMem.fbl);
        if (ele) {
            local_cmddata[0] = ele->index;
            local_cmddata[1] = (UINT32)ele->luma.physAddrAligned;
            local_cmddata[2] = (ctx->pSeqinfo->uProgressive) ? (local_cmddata[1] + ctx->luma_size) :
                               (local_cmddata[1] + ctx->luma_size / 2);
            local_cmddata[3] = (UINT32)ele->chroma.physAddrAligned;
            local_cmddata[4] = (ctx->pSeqinfo->uProgressive) ? (local_cmddata[3] + ctx->chroma_size) :
                               (local_cmddata[3] + ctx->chroma_size / 2);
            local_cmddata[5] = ctx->stride;
            local_cmddata[6] = MEDIAIP_FRAME_REQ;
            if ((ele->status == FRAME_ALLOC) || (ele->status == FRAME_RELEASE)) {
                DBG_VDEC_PRINT_INFO("ctx[%d] pending VID_API_CMD_FS_ALLOC, eType=%d, uFSIdx=%d", ctx->stream_id, MEDIAIP_FRAME_REQ,
                                    ele->index);
                vpu_send_cmd(ctx->dev, ctx->stream_id, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
                fbl_vfb_set_status(ctx->CtxMem.fbl, ele, FRAME_FREE);
            } else {
                DBG_PRINT_ERROR("ctx[%d] VID_API_EVENT_REQ_FRAME_BUFF, No Frame Buffer Available, Something wrong with Frame Buffer management",
                                ctx->stream_id);
            }
            ctx->pending_vfb_req--;
        } else {
            DBG_PRINT_ERROR("ctx[%d] VID_API_EVENT_REQ_FRAME_BUFF, No Frame Buffer Available, Something wrong with Frame Buffer management",
                            ctx->stream_id);
            break;
        }
    }
    DBG_VDEC_METHOD_END();
}

/**
 * vdec_get_output()
 * @brief Provides an frame buffer for delivering to application
 * @param dev* : MALONE_VPU_DEVICE_CONTEXT
 * @param stream_id : Stream ID
 * @param fbi : Frame buffer to be delivered to application
 * @return NTSTATUS
 */
NTSTATUS vdec_get_output(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, fbo_t *fbo)
{
    NTSTATUS ret = STATUS_UNSUCCESSFUL;
    DBG_VDEC_METHOD_BEG();
    vdec_ctx_t *ctx = &dev->ctx[stream_id];
    static int cntr = 0;
    /* */
    DBG_EVENTS_PRINT_INFO("MFT->KM decoder_get_output() ctx[%d], ofb index=%d", stream_id, fbo->index);      /* ofb index */
    if (ctx) {

        WdfSpinLockAcquire(ctx->mutex);
        ofb_t *ofb = NULL;
        if (ctx->CtxMem.fbl && fbo->index < ctx->CtxMem.fbl->ofb_capacity) {
            ofb = &ctx->CtxMem.fbl->ofb[fbo->index];
            fbo->width = ofb->width;
            fbo->height = ofb->height;
            fbo->stride = ofb->stride;
            fbo->planes[0] = (uintptr_t)ofb->mem.virtAddrAligned;
            ofb->held_by_client = TRUE;
            KeFlushIoBuffers(ofb->mem.mdl, TRUE /*READ-OP-INVALIDATE*/, TRUE);
            ret = STATUS_SUCCESS;
        }
        WdfSpinLockRelease(ctx->mutex);
    }
    return ret;
}

/**
 * vdec_flush()
 * @brief Perform flush operation.
 * @param dev* : MALONE_VPU_DEVICE_CONTEXT
 * @param stream_id : Stream ID
 * @param flush : Pointer to flush structure
 * @return NTSTATUS
 */
NTSTATUS vdec_flush(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, vdec_flush_t *flush)
{
    NTSTATUS ret = STATUS_SUCCESS;
    DBG_VDEC_METHOD_BEG();
    vdec_ctx_t *ctx = &dev->ctx[stream_id];
    DBG_EVENTS_PRINT_INFO("MFT->KM decoder_flush() ctx[%d], wptr=0x%08d, padding_size=%d",
                          stream_id,
                          flush->wptr,              /* bs buffer wptr before adding abort padding */
                          flush->padding_size);      /* abort padding size , */

    if (ctx != NULL) {
        WdfSpinLockAcquire(ctx->mutex);
        if ((ctx->status == MEDIA_PLAYER_STOPPED) || (ctx->status == MEDIA_PLAYER_STOPPING)) {
            DBG_VDEC_PRINT_INFO("ctx[%d] Already stopping or stopped", ctx->stream_id);
            WdfSpinLockRelease(ctx->mutex);
            DBG_VDEC_METHOD_END();
            return STATUS_SUCCESS;
        }
        if (ctx->aborts == ABORT_DONE) {
            DBG_VDEC_PRINT_INFO("ctx[%d] Already Aborted", ctx->stream_id);
            WdfSpinLockRelease(ctx->mutex);
            DBG_VDEC_METHOD_END();
            return STATUS_SUCCESS;
        }
        if (ctx->status == MEDIA_PLAYER_STARTING) {
            DBG_EVENTS_PRINT_INFO("ctx[%d] Waiting for VID_API_EVENT_START_DONE event from VPU", ctx->stream_id);
            do {
                WdfSpinLockRelease(ctx->mutex);
                ret = KeWaitForSingleObject(&ctx->cond, Executive, KernelMode, FALSE, NULL);
                if (ret != STATUS_SUCCESS) {
                    DBG_PRINT_ERROR("ctx[%d] pthread_cond_wait failed (%d)", ctx->stream_id, ret);
                    DBG_VDEC_METHOD_END();
                    return ret;
                }
                WdfSpinLockAcquire(ctx->mutex);
            } while (ctx->status == MEDIA_PLAYER_STARTING);
            DBG_VDEC_PRINT_INFO("ctx[%d] Received VID_API_EVENT_START_DONE event from VPU", ctx->stream_id);
        }
        vfb_t *vfb = { 0 };
        ofb_t *ofb = { 0 };
        /* Client will not consume decoded frame buffers anymore
         * vpu will not push frame buffers to queue anymore
         * release all buffers from frame buffer queue */
        while (pop_vfb_entry(&ctx->vfbq, &vfb) == ERR_EOK) {
            fbl_vfb_release(ctx->CtxMem.fbl, vfb);
        }
        while (pop_ofb_entry(&ctx->ofbq, &ofb) == ERR_EOK) {
            fbl_ofb_release(ctx->CtxMem.fbl, ofb);
        }

        if (ctx->aborts != ABORT_PENDING) {
            pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[ctx->stream_id];
            uint32_t padsz = flush->padding_size;
            ctx->aborts = ABORT_PENDING;
            buffer_info->stream_pic_end_flag = 1;
            KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
            DBG_VDEC_PRINT_INFO("ctx[%d] sending abort cmd.....wptr 0x%p padding size %d", ctx->stream_id, (void *)flush->wptr,
                                flush->padding_size);
            vpu_send_cmd(dev, ctx->stream_id, VID_API_CMD_ABORT, 1, &padsz);
        }
        /* Wait until VPU responds */
        ret = STATUS_SUCCESS;
        while (ctx->aborts == ABORT_PENDING) {
            ret = block(ctx, 0);
        }
        WdfSpinLockRelease(ctx->mutex);
        DBG_VDEC_PRINT_INFO("ctx[%d] exit", ctx->stream_id);
    }
    DBG_VDEC_METHOD_END();
    return ret;
}

/*****************************************************************************************************************************
* Release FrameBuffer for reuse
*****************************************************************************************************************************/
/**
 * vdec_clear_output()
 * @brief Release FrameBuffer from application for reuse
 * @param dev : MALONE_VPU_DEVICE_CONTEXT
 * @param stream_id : Stream ID
 * @param index : Indes of frame buffer
 * @return NTSTATUS
 */
NTSTATUS vdec_clear_output(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, int index)
{
    NTSTATUS ret = STATUS_SUCCESS;
    DBG_VDEC_METHOD_BEG();
    vdec_ctx_t *ctx = &dev->ctx[stream_id];

    if (ctx != NULL) {
        WdfSpinLockAcquire(ctx->mutex);
        DBG_EVENTS_PRINT_INFO("MFT->KM decoder_clear_output() ctx[%d], ofb index=%d", stream_id, index);
        do {
            if ((index < 0) || (index >= ctx->CtxMem.fbl->ofb_capacity)) {
                DBG_PRINT_ERROR("OFB index is out of range for releasing!");
                ret = STATUS_RANGE_NOT_FOUND;
                break;
            }

            if (ctx->CtxMem.fbl) {
                fbl_t *list = ctx->CtxMem.fbl;
                ofb_t *ele = &list->ofb[index];
                ctx->oframe_with_client--;
                if (ele->held_by_client) {
                    fbl_ofb_release(ctx->CtxMem.fbl, ele);
                    ele->held_by_client = FALSE;
                } else {
                    DBG_VDEC_PRINT_WARNING("ctx[%d] client clears alredy cleared output ( %d)", stream_id, index);
                }
                ret = STATUS_SUCCESS;
            }
        } while (0);
        WdfSpinLockRelease(ctx->mutex);
    }
    t2l_frame(ctx); /* Optimize, not call upon failure. */
    DBG_VDEC_METHOD_END();
    return ret;
}

/**
 * vpu_api_event_handler()
 * @brief Handle events from VPU FW
 * @param dev* : MALONE_VPU_DEVICE_CONTEXT
 * @param uStrIdx : Stream ID
 * @param uEvent : Event
 * @param event_data : Event data
 */
void vpu_api_event_handler(MALONE_VPU_DEVICE_CONTEXT *dev, u_int32 uStrIdx, u_int32 uEvent, u_int32 *event_data)
{
    UNREFERENCED_PARAMETER(event_data);
    vdec_ctx_t *ctx = &dev->ctx[uStrIdx];
    DBG_VDEC_METHOD_BEG();
    if (ctx == NULL) {
        DBG_PRINT_ERROR("ctx[%d] Ctx is not initialized!", uStrIdx);
        DBG_VDEC_METHOD_END();
        return;
    }

    size_t fbs_cnt = 0;
    size_t bytes = 0;
#if 0
    fb_t *fb = NULL;
    fb_t *ele = NULL;
#endif
    uint32_t parsed = 0, in = 0;

    pDEC_RPC_HOST_IFACE pSharedInterface = (pDEC_RPC_HOST_IFACE)dev->shared_mem.shared_mem_vir;

    vpu_log_event(ctx->stream_id, uEvent);

    switch (uEvent) {
        case VID_API_EVENT_START_DONE:
            WdfSpinLockAcquire(ctx->mutex);
            ctx->status = MEDIA_PLAYER_STARTED;
            DBG_VDEC_PRINT_INFO("ctx[%d] MEDIA_PLAYER_STARTED", uStrIdx);
            KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
            WdfSpinLockRelease(ctx->mutex);
            break;
        case VID_API_EVENT_STOPPED:
            DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_STOPPED", uStrIdx);
            WdfSpinLockAcquire(ctx->mutex);
            if (ctx->status != MEDIA_PLAYER_STOPPED) {
                ctx->status = MEDIA_PLAYER_STOPPED;
                KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
            }
            vdec_clear(ctx);
            /* VPU could release all the buffers when stop cmd is received, but it does not. */
            /* resmgr clear all the buffers back to pool. ( suggested by nxp) */
            fbl_vfb_clear(ctx->CtxMem.fbl);
            WdfSpinLockRelease(ctx->mutex);
            break;
        case VID_API_EVENT_RESET_DONE:
            break;
        case VID_API_EVENT_PIC_DECODED: {
            vfb_t *vfb = NULL;
            MediaIPFW_Video_PicInfo *pPicInfo = (MediaIPFW_Video_PicInfo *)dev->shared_mem.pic_mem_vir;
            MediaIPFW_Video_PicPerfInfo *pPerfInfo = &pPicInfo[uStrIdx].PerfInfo;
            UNREFERENCED_PARAMETER(pPerfInfo);
            uint32_t buffer_id;
            vfb = fbl_find_by_addr(ctx->CtxMem.fbl, event_data[0]);
            ctx->frame_dec_counter++;
            DBG_VDEC_PRINT_INFO("ctx[%d] PICINFO GET: uPicType:%d uPicStruct:%d uPicStAddr:0x%x uFrameStoreID:%d uPercentInErr:%d, uRbspBytesCount=%d, ulLumBaseAddr[0]=%x",
                                uStrIdx,
                                pPicInfo[uStrIdx].uPicType, pPicInfo[uStrIdx].uPicStruct,
                                pPicInfo[uStrIdx].uPicStAddr, pPicInfo[uStrIdx].uFrameStoreID,
                                pPicInfo[uStrIdx].uPercentInErr, pPerfInfo->uRbspBytesCount, event_data[0]);
            if (vfb != NULL) {
                buffer_id = vfb->index;
            } else {
                DBG_PRINT_ERROR("ctx[%d] VID_API_EVENT_PIC_DECODED ele is NULL!!!", ctx->stream_id);
                break;
            }

            if (buffer_id != event_data[7]) {
                DBG_PRINT_ERROR("ctx[%d] error:VID_API_EVENT_PIC_DECODED address and id doesn't match", uStrIdx);
            }
            if (vfb->status != FRAME_FREE) {
                DBG_PRINT_ERROR("ctx[%d] error: buffer(%d) need to set FRAME_DECODED, but previous state %s is not FRAME_FREE",
                                uStrIdx,
                                vfb->index, frame_status_to_str(vfb->status));
            }
            WdfSpinLockAcquire(ctx->mutex);
            fbl_vfb_set_status(ctx->CtxMem.fbl, vfb, FRAME_DECODED);
            WdfSpinLockRelease(ctx->mutex);
            break;
        }
        case VID_API_EVENT_SEQ_HDR_FOUND: {

            MediaIPFW_Video_SeqInfo UNALIGNED *pSeqInfo = (MediaIPFW_Video_SeqInfo *)dev->shared_mem.seq_mem_vir;
            MediaIPFW_Video_PitchInfo *pStreamPitchInfo = &pSharedInterface->StreamPitchInfo[uStrIdx];
            WdfSpinLockAcquire(ctx->mutex);
            if ((!ctx->b_firstseq) && (memcmp(&ctx->pSeqinfo[0], &pSeqInfo[uStrIdx], sizeof(MediaIPFW_Video_SeqInfo)))) {
                DBG_VDEC_PRINT_INFO("ctx[%d] SEQINFO GET: uHorRes:%d uVerRes:%d uHorDecodeRes:%d uVerDecodeRes:%d uNumDPBFrms:%d uNumRefFrms=%d",
                                    uStrIdx,
                                    pSeqInfo[0].uHorRes, pSeqInfo[0].uVerRes,
                                    pSeqInfo[0].uHorDecodeRes, pSeqInfo[0].uVerDecodeRes,
                                    pSeqInfo[0].uNumDPBFrms, pSeqInfo[0].uNumRefFrms);
                DBG_VDEC_PRINT_INFO("ctx[%d] App should take care decoder config changes with resolution or nbuffers", uStrIdx);
                WdfSpinLockRelease(ctx->mutex);
                break;
            }
            if (ctx->b_firstseq) {
                bytes = sizeof(MediaIPFW_Video_SeqInfo);
#if 0           /* Issue with ldp instruction, unaligned memory cause crash
             * WINBSP-1320
             */
                RtlCopyMemory(&ctx->pSeqinfo[0], &pSeqInfo[uStrIdx], bytes);
#else
                BYTE *pByteSrc = (BYTE *)&pSeqInfo[uStrIdx];
                BYTE *pByteDst = (BYTE *)&ctx->pSeqinfo[0];
                for (int i = 0; i < bytes; i++) {
                    *pByteDst = *(pByteSrc + i);
                    pByteDst += 1;
                }
#endif
                DBG_VDEC_PRINT_INFO("ctx[%d] SEQINFO GET: uHorRes:%d uVerRes:%d uHorDecodeRes:%d uVerDecodeRes:%d uNumDPBFrms:%d uNumRefFrms=%d",
                                    uStrIdx,
                                    ctx->pSeqinfo[0].uHorRes, ctx->pSeqinfo[0].uVerRes,
                                    ctx->pSeqinfo[0].uHorDecodeRes, ctx->pSeqinfo[0].uVerDecodeRes,
                                    ctx->pSeqinfo[0].uNumDPBFrms, ctx->pSeqinfo[0].uNumRefFrms);
                fbs_cnt = (size_t)(ctx->pSeqinfo[0].uNumDPBFrms + ctx->pSeqinfo[0].uNumRefFrms + 3);

                if (ctx->pSeqinfo[0].uNumRefFrms == 0) {
                    fbs_cnt++; /* For JPEG, RefFrms == 0, but it is required + 1 */
                }
                if (fbs_cnt > 32) {
                    DBG_PRINT_ERROR("ctx[%d] ERROR: VPU exceeded number of allowed FrameBuffers (32 < %d) for this stream!", uStrIdx,
                                    (int)fbs_cnt);
                    fbs_cnt = 32;
                }
                DBG_PRINT_ERROR("ctx[%d]  VPU needs %u FrameBuffers for this stream", uStrIdx, (int)fbs_cnt);

                if (ctx->force_fbufs_num) {
                    fbs_cnt = ctx->force_fbufs_num;
                    DBG_PRINT_ERROR("ctx[%d] User forced %d FrameBuffers. Playback may be unstable!", uStrIdx, ctx->force_fbufs_num);
                }

                if ((ctx->pSeqinfo[0].uHorDecodeRes == 0) || (ctx->pSeqinfo[0].uVerDecodeRes == 0)) {
                    DBG_VDEC_PRINT_INFO("ctx[%d]  Bad Resolution wxh = %dx%d", uStrIdx, ctx->pSeqinfo[0].uHorDecodeRes,
                                        ctx->pSeqinfo[0].uVerDecodeRes);
                    ctx->error = VDEC_FATAL_ERROR;
                    KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
                    WdfSpinLockRelease(ctx->mutex);
                    break;
                }

                caculate_frame_size(ctx);
                if (ctx->CtxMem.fbl == NULL) {
                    WdfSpinLockRelease(ctx->mutex);
                    break;
                } else {
                    fbl_start(ctx->CtxMem.fbl);
                }
#if 0 /*moved to decoder decode */
                if (alloc_mbi_buffers(ctx) != STATUS_SUCCESS) {
                    DBG_PRINT_ERROR("ctx[%d]  Failed to Allocate mbi buffers", uStrIdx);
                    ctx->error = VDEC_FATAL_ERROR_NOMEM;
                    KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
                    WdfSpinLockRelease(ctx->mutex);
                    break;
                }
#endif
                pStreamPitchInfo->uFramePitch = 0x4000;
                KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
                ctx->b_firstseq = FALSE;
            }
            WdfSpinLockRelease(ctx->mutex);
            break;
        }
        case VID_API_EVENT_PIC_HDR_FOUND:
            if (ctx->frame_mode) {
                update_frame_level(ctx);
            }
            break;
        case VID_API_EVENT_REQ_FRAME_BUFF: {
            MEDIA_PLAYER_FSREQ *pFSREQ = (MEDIA_PLAYER_FSREQ *)event_data;
            u_int32 local_cmddata[10] = { 0 };
            DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_REQ_FRAME_BUFF, type=%d, size=%d", uStrIdx, pFSREQ->eType,
                                sizeof(MEDIA_PLAYER_FSREQ));
            WdfSpinLockAcquire(ctx->mutex);
            do {
                if (ctx->status != MEDIA_PLAYER_STARTED) {
                    break;
                }
                if (pFSREQ->eType == MEDIAIP_DCP_REQ) {
                    VpuMemory *dcp = &(ctx->CtxMem.dcp_mem[ctx->dcp_count]);
                    if (dcp->virtAddr == NULL) {
                        DBG_PRINT_ERROR("ctx[%d] MBI buffer is not mappend. Cann´t serve VPU", uStrIdx);
                        ctx->error = VDEC_FATAL_ERROR_DCP_DISABLED_INF;
                        KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
                        break;
                    }
                    local_cmddata[0] = ctx->dcp_count;
                    local_cmddata[1] = (u_int32)dcp->physAddrAligned;
                    local_cmddata[2] = DCP_SIZE;
                    local_cmddata[3] = 0;
                    local_cmddata[4] = 0;
                    local_cmddata[5] = 0;
                    local_cmddata[6] = pFSREQ->eType;
                    vpu_send_cmd(dev, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
                    DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_CMD_FS_ALLOC, eType=%d, uFSIdx=%d", uStrIdx, pFSREQ->eType, ctx->dcp_count);
                    ctx->dcp_count++;
                } else if (pFSREQ->eType == MEDIAIP_MBI_REQ) {
                    if (ctx->mbi_count >= MAX_MBI_NUM) {
                        ctx->error = VDEC_FATAL_ERROR;
                        DBG_VDEC_PRINT_ERROR("ctx[%d] no mbi buffer in the pool. all of them are served", uStrIdx);
                        KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
                        break;
                    }
                    local_cmddata[0] = ctx->mbi_count;
                    local_cmddata[1] = (u_int32)ctx->CtxMem.mbi_mem[ctx->mbi_count].physAddrAligned;
                    local_cmddata[2] = ctx->req_mbi_size;
                    local_cmddata[3] = 0;
                    local_cmddata[4] = 0;
                    local_cmddata[5] = 0;
                    local_cmddata[6] = pFSREQ->eType;
                    vpu_send_cmd(dev, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
                    DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_CMD_FS_ALLOC, eType=%d, uFSIdx=%d", uStrIdx, pFSREQ->eType, ctx->mbi_count);
                    ctx->mbi_count++;
                } else if (pFSREQ->eType == MEDIAIP_FRAME_REQ) {
                    vfb_t *ele = NULL;
                    ele = fbl_vfb_acquire(ctx->CtxMem.fbl);
                    if (ele) {
                        DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_REQ_FRAME_BUFF request: %d, selected buffer index: %d", uStrIdx,
                                            pFSREQ->eType,
                                            ele->index);
                        local_cmddata[0] = ele->index;
                        local_cmddata[1] = (u_int32)ele->luma.physAddrAligned;
                        local_cmddata[2] = (ctx->pSeqinfo->uProgressive) ? (local_cmddata[1] + ctx->luma_size) :
                                           (local_cmddata[1] + ctx->luma_size / 2);
                        local_cmddata[3] = (u_int32)ele->chroma.physAddrAligned;
                        local_cmddata[4] = (ctx->pSeqinfo->uProgressive) ? (local_cmddata[3] + ctx->chroma_size) :
                                           (local_cmddata[3] + ctx->chroma_size / 2);
                        local_cmddata[5] = ctx->stride;
                        local_cmddata[6] = pFSREQ->eType;
#if REL_ON_REQ
                        if (ele->status == FRAME_RELEASE) {
                            vpu_send_cmd(dev, uStrIdx, VID_API_CMD_FS_RELEASE, 1, (uint32_t *)(&ele->fbi.index));
                        }
#endif
                        if ((ele->status == FRAME_ALLOC) || (ele->status == FRAME_RELEASE)) {
                            vpu_send_cmd(dev, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
                            DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_CMD_FS_ALLOC, eType=%d, uFSIdx=%d", uStrIdx, pFSREQ->eType, ele->index);
                            fbl_vfb_set_status(ctx->CtxMem.fbl, ele, FRAME_FREE);
                        } else {
                            DBG_PRINT_ERROR("ctx[%d] VID_API_EVENT_REQ_FRAME_BUFF, No Frame Buffer Available, Something wrong with Frame Buffer management",
                                            uStrIdx);
                        }
                    } else if (ctx->status == MEDIA_PLAYER_STARTED) {
                        DBG_VDEC_PRINT_INFO("ctx[%d] Frame Buffer Request can not be served at this time", uStrIdx);
                        fbl_log(ctx->CtxMem.fbl);
                    }
                }
            } while (0);
            WdfSpinLockRelease(ctx->mutex);
            break;
        }
        case VID_API_EVENT_REL_FRAME_BUFF: {
            MEDIA_PLAYER_FSREL *fsrel = (MEDIA_PLAYER_FSREL *)event_data;
            uint32_t frame_buffer_id = fsrel->uFSIdx;
            vfb_t *fb;
            DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_REL_FRAME_BUFF  uFSIdx=%d, eType=%d", uStrIdx, fsrel->uFSIdx,
                                fsrel->eType);
            WdfSpinLockAcquire(ctx->mutex);
            fb = fbl_find_by_index(ctx->CtxMem.fbl, frame_buffer_id);
            if (fsrel->eType == MEDIAIP_FRAME_REQ) {
                if (fb != NULL) {
                    int idx = fb->index;
                    if (fb->status != FRAME_READY) {
                        DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_REL_FRAME_BUFF Expected FRAME_READY status, but status is %s,  uFSIdx=%d, eType=%d",
                                            uStrIdx, frame_status_to_str(fb->status), fsrel->uFSIdx, fsrel->eType);
                    }
#if !REL_ON_REQ
                    DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_CMD_FS_RELEASE uFSIdx=%d", ctx->stream_id, idx);
                    vpu_send_cmd(ctx->dev, ctx->stream_id, VID_API_CMD_FS_RELEASE, 1, (uint32_t *)(&idx));
                    vdec_serve_pending_vfb(ctx);
#endif
                    fbl_vfb_set_status(ctx->CtxMem.fbl, fb, FRAME_RELEASE);
                } else {
                    DBG_PRINT_ERROR("ctx[%d] VID_API_EVENT_REL_FRAME_BUFF ele is NULL", ctx->stream_id);
                }
            } else {
                DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_REL_FRAME_BUFF uFSIdx=%d, eType=%d, size=%d", uStrIdx, fsrel->uFSIdx,
                                    fsrel->eType,
                                    sizeof(MEDIA_PLAYER_FSREL));
            }
            WdfSpinLockRelease(ctx->mutex);
            break;
        }
        case VID_API_EVENT_FRAME_BUFF_RDY: {
            u_int32     *FrameInfo = (u_int32 *)event_data;
            vfb_t       *vfb = fbl_find_by_addr(ctx->CtxMem.fbl, FrameInfo[1]);
            uint32_t fs_id = FrameInfo[0];

            /* VFB checks */
            if (vfb == NULL) {
                DBG_PRINT_ERROR("ctx[%d] VID_API_EVENT_FRAME_BUFF_RDY ele is NULL!!!", uStrIdx);
                break;
            }
            if ((fs_id > fbl_vfb_size(ctx->CtxMem.fbl))) {
                DBG_PRINT_ERROR("ctx[%d] error: wrong buffer_id(%d). Breaking.", uStrIdx, fs_id);
                break;
            }

            if (vfb->index != (int)fs_id) {
                DBG_PRINT_ERROR("ctx[%d] error: find buffer_id(%d) and firmware return id(%d) doesn't match", uStrIdx, vfb->index,
                                fs_id);
            }
            WdfSpinLockAcquire(ctx->mutex);
            if (vfb) {
                vfb->rdy_for_blit = TRUE;
                if (ctx->status == MEDIA_PLAYER_STARTED) {
                    if (push_vfb_entry(&ctx->vfbq, vfb) != ERR_EOK) {
                        DBG_PRINT_ERROR("ctx[%d] Decode Picture: no space in frame queue. Increase queue size", uStrIdx);
                        DBG_PRINT_ERROR("ctx[%d] Decode Picture: Frame will not be displayed", uStrIdx);
                    } else {
                        parsed = 0;
                        in = 0;
                        DBG_VDEC_PRINT_INFO("ctx[%d] Decoded Picture at FrameBuffer[%d]", uStrIdx, vfb->index);
                        if (ctx->frame_mode) {
                            pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[uStrIdx];
                            parsed = buffer_info->stream_pic_parsed_count;
                            in = buffer_info->stream_pic_input_count;
                        }
                        vdec_buffer_status(ctx, FALSE);
                        DBG_VDEC_PRINT_INFO("=>ctx[%d]%s frame[%d] dpq %d bsb(bytes %d in %d parsed %d phc %d) co %d",
                                            uStrIdx, ctx->dis_reorder ? "DR" : "R", ctx->frame_counter, ctx->vfbq.counter, ctx->remains, in, parsed,
                                            ctx->pic_hdr_cnt, ctx->oframe_with_driver);
                        KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
                    }
                }
            }
            if (ctx->status != MEDIA_PLAYER_STARTED) {
                DBG_VDEC_PRINT_INFO("ctx[%d] Decode Picture: player is in stopping or stopped state", uStrIdx);
                DBG_VDEC_PRINT_INFO("ctx[%d] Decode Picture: Frame will not be displayed", uStrIdx);
            } else if (vfb == NULL) {
                DBG_PRINT_ERROR("ctx[%d] Why I am here....", uStrIdx);
            }
            if (vfb->status != FRAME_DECODED) {
                DBG_PRINT_ERROR("ctx[%d] ERROR: buffer(%d) need to set FRAME_READY, but previous state %d is not FRAME_DECODED",
                                uStrIdx,
                                vfb->index,
                                vfb->status);
            }
            fbl_vfb_set_status(ctx->CtxMem.fbl, vfb, FRAME_READY);
            DBG_VDEC_PRINT_INFO("ctx[%d] INFO: Frame counter: %u", uStrIdx, ctx->frame_counter);
            ctx->frame_counter++;
            WdfSpinLockRelease(ctx->mutex);

            /* TODO:FRANTA Tady se vola t2l konverze*/
            t2l_frame(ctx);

            break;
        }
        case VID_API_EVENT_CHUNK_DECODED:
            break;
        case VID_API_EVENT_FIFO_LOW:
            break;
        case VID_API_EVENT_FIFO_HIGH:
            break;
        case  VID_API_EVENT_FIFO_EMPTY:
            break;
        case  VID_API_EVENT_FIFO_FULL:
            break;
        case  VID_API_EVENT_FIFO_OVF:
            break;
        case  VID_API_EVENT_UNSUPPORTED_STREAM:
            WdfSpinLockAcquire(ctx->mutex);
            DBG_PRINT_ERROR("ctx[%d] unsupported stream", uStrIdx);
            /*ctx->error = VDEC_FATAL_ERROR_UNSUPPORTED_STREAM; */
            KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
            WdfSpinLockRelease(ctx->mutex);
            break;
        case  VID_API_EVENT_BS_ERROR:
            /* NXP says it could be warning. */
            /* NXP is tracking ( NXP-382)  this issue with vendor. */
            DBG_PRINT_ERROR("ctx[%d] bitstream error", uStrIdx);
            break;
        case  VID_API_EVENT_UDATA_FIFO_UPTD:
            break;
        case VID_API_EVENT_DBG_STAT_UPDATE:
            break;
        case VID_API_EVENT_DBG_LOG_STARTED:
            break;
        case VID_API_EVENT_DBG_LOG_STOPPED:
            break;
        case VID_API_EVENT_ABORT_DONE: {
            pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc = (pSTREAM_BUFFER_DESCRIPTOR_TYPE)((uintptr_t)dev->RegistersPtr +
                                                                                          DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF *
                                                                                          (uintptr_t)ctx->stream_id);
            DSB;
            DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_ABORT_DONE", uStrIdx);
            pStrBufDesc->wptr = pStrBufDesc->rptr;
            dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->stream_id][uStrIdx] = (VPU_REG_BASE_MCORE + DEC_MFD_XREG_SLV_BASE
                                                                                          + MFD_MCX + MFD_MCX_OFF * ctx->stream_id);
            vdec_buffer_status(ctx, TRUE);
            vpu_send_cmd(dev, uStrIdx, VID_API_CMD_RST_BUF, 0, NULL);
            break;
        }
        case VID_API_EVENT_RES_CHANGE:
            break;
        case VID_API_EVENT_STR_BUF_RST: {
            pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[ctx->stream_id];
            vfb_t *vfb = { 0 };
            ofb_t *ofb = { 0 };
            WdfSpinLockAcquire(ctx->mutex);
            /* Client will not consume decoded frame buffers anymore
             * vpu will not push frame buffers to queue anymore
             * release all buffers from frame buffer queue */
            while (pop_vfb_entry(&ctx->vfbq, &vfb) == ERR_EOK) {
                fbl_vfb_release(ctx->CtxMem.fbl, vfb);
            }
            while (pop_ofb_entry(&ctx->ofbq, &ofb) == ERR_EOK) {
                fbl_ofb_release(ctx->CtxMem.fbl, ofb);
            }

            vdec_clear(ctx);
            ctx->aborts = ABORT_DONE;
            ctx->parsed_frames = 0;
            buffer_info->stream_pic_end_flag = 0;
            ctx->eos = 0;
            DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_STR_BUF_RST", ctx->stream_id);
            vdec_buffer_status(ctx, TRUE);
            KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
            vdec_serve_pending_vfb(ctx);
            fbl_log(ctx->CtxMem.fbl);
            WdfSpinLockRelease(ctx->mutex);
            break;
        }
        case VID_API_EVENT_RET_PING:
            break;
        case VID_API_EVENT_STR_FMT_CHANGE:
            break;
        case VID_API_EVENT_FINISHED: {
            pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[ctx->stream_id];
            WdfSpinLockAcquire(ctx->mutex);
            buffer_info->stream_pic_end_flag = 0;
            ctx->eos = 1;
            DBG_VDEC_PRINT_INFO("ctx[%d] VID_API_EVENT_FINISHED", ctx->stream_id);
            vdec_buffer_status(ctx, TRUE);
            KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
            WdfSpinLockRelease(ctx->mutex);
            break;
        }
        case VID_API_EVENT_FIRMWARE_XCPT: {
            WdfSpinLockAcquire(ctx->mutex);
            DBG_PRINT_ERROR("ctx[%d] FW EXCEPTION: %s", ctx->stream_id, (char *)event_data);
            ctx->error = VDEC_FATAL_ERROR_FIRMWARE_EXCEPTION;
            KeSetEvent(&ctx->cond, IO_VIDEO_INCREMENT, 0);
            vdec_buffer_status(ctx, TRUE);
            fbl_log(ctx->CtxMem.fbl);
            dbglog_show(ctx, VPU_FRMDBG_RAW_DIS);
            WdfSpinLockRelease(ctx->mutex);
            break;
        }
        default:
            break;
    }
    DBG_DEV_METHOD_END_WITH_PARAMS("ctx[%d] leave", uStrIdx);
}

/**
 * vdec_close()
 * @brief Close decoder context
 * @param dev* : MALONE_VPU_DEVICE_CONTEXT
 * @param stream_id : Stream ID
 * @return NTSTATUS
 */
NTSTATUS vdec_close(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id)
{
    int ret = STATUS_SUCCESS;
    DBG_VDEC_METHOD_BEG();

    vdec_ctx_t *ctx = NULL;
    if (stream_id != -1) {
        ctx = &dev->ctx[stream_id];
    }
    if (ctx && (stream_id != -1)) {

        vpu_unmap_ctx_memory(&ctx->CtxMem);

        WdfSpinLockAcquire(dev->mutex);
        if (dev->dev_opened) {
            dev->dev_opened--;
        }
        WdfSpinLockRelease(dev->mutex);
        ctx->file = NULL;
        ctx->stream_id = -1;
        /*TODO */
        ret = STATUS_SUCCESS;
    }
    DBG_VDEC_METHOD_END();
    return STATUS_SUCCESS;
}

/**
 * vdec_open()
 * @brief Open Decoding instance
 * @param dev* : MALONE_VPU_DEVICE_CONTEXT
 * @param file : WDFFILEOBJECT
 * @param dec* : Pointer to vdec_init_t object
 * @param out_sb_mem* : Pointer to stream buffer memory descriptor
 * @return
 */
NTSTATUS vdec_open(MALONE_VPU_DEVICE_CONTEXT *dev, WDFFILEOBJECT file, vdec_init_t *dec, vdec_mem_desc_t *out_sb_mem)
{
    NTSTATUS status = STATUS_SUCCESS;
    vdec_ctx_t *ctx = NULL;
    int i = 0;
    u_int32 uStrIdx = 0; /* Set to be default 0, FIX_ME later */
    int stream_id = -1;
    pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc = NULL;
    MediaIPFW_Video_CodecParams *pCodecPara = NULL;

    MEDIA_IP_FORMAT video_format = MEDIA_IP_FMT_NULL;
    TB_API_DEC_FMT malone_format = VSys_FrmtNull;

    DBG_VDEC_METHOD_BEG();

    if (dec->fourcc == (uint32_t)AOIFOURCC("HEVC") ||
        dec->fourcc == (uint32_t)AOIFOURCC("H265")) {
        video_format = MEDIA_IP_FMT_HEVC;
        malone_format = VSys_HevcFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("AVC1")) {
        video_format = MEDIA_IP_FMT_AVC;
        malone_format = VSys_AvcFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("WMV3") ||
               dec->fourcc == (uint32_t)AOIFOURCC("WVC1")) {
        video_format = MEDIA_IP_FMT_VC1;
        malone_format = VSys_Vc1Frmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("H264")) {
        video_format = MEDIA_IP_FMT_AVC;
        malone_format = VSys_AvcFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("S263") ||
               dec->fourcc == (uint32_t)AOIFOURCC("H263")) {
        video_format = MEDIA_IP_FMT_ASP;
        malone_format = VSys_AspFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("MP4V") ||
               dec->fourcc == (uint32_t)AOIFOURCC("MPG4")) {
        video_format = MEDIA_IP_FMT_ASP;
        malone_format = VSys_AspFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("DIV3") ||
               dec->fourcc == (uint32_t)AOIFOURCC("DIV4") ||
               dec->fourcc == (uint32_t)AOIFOURCC("DIV5") ||
               dec->fourcc == (uint32_t)AOIFOURCC("XVID") ||
               dec->fourcc == (uint32_t)AOIFOURCC("DIVX") ||
               dec->fourcc == (uint32_t)AOIFOURCC("DX50")) {
        video_format = MEDIA_IP_FMT_ASP;
        malone_format = VSys_AspFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("MP1V") ||
               dec->fourcc == (uint32_t)AOIFOURCC("MP2V")) {
        video_format = MEDIA_IP_FMT_MP2;
        malone_format = VSys_Mp2Frmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("RV30") ||
               dec->fourcc == (uint32_t)AOIFOURCC("RV40")) {
        video_format = MEDIA_IP_FMT_AVC;
        malone_format = VSys_AvcFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("FLV1")) {
        video_format = MEDIA_IP_FMT_SPK;
        malone_format = VSys_SpkFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("MJPG")) {
        video_format = MEDIA_IP_FMT_JPG;
        malone_format = VSys_JpgFrmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("VP60")) {
        video_format = MEDIA_IP_FMT_VP6;
        malone_format = VSys_Vp6Frmt;
    } else if (dec->fourcc == (uint32_t)AOIFOURCC("VP80")) {
        video_format = MEDIA_IP_FMT_VP8;
        malone_format = VSys_Vp8Frmt;
    } else {
        return STATUS_UNSUCCESSFUL;
    }

    /*assign context - Find free context * / */
    for (i = 0; i < (int)dev->config.MaxContexts; i++) {
        if (dev->ctx[i].file == NULL) {
            stream_id = i;
            break;
        }
    }

    DBG_EVENTS_PRINT_INFO("MFT->KM decoder_init() ctx[%d]", stream_id); /* TODO: report codec, other settings from MFT */

    if (stream_id >= 0) {
        ctx = &dev->ctx[stream_id];
        ctx->stream_id = stream_id;
        DBG_VDEC_PRINT_INFO("ctx[%d] vdec_open", stream_id);
    } else {
        DBG_PRINT_ERROR("Max decoding instances are already active");
        DBG_VDEC_METHOD_END();
        return STATUS_UNSUCCESSFUL;
    }

    ctx->dev = dev;
    ctx->file = file;
    ctx->frame_dec_counter = 0;
    ctx->frame_counter = 0;
    ctx->eos = 0;
    ctx->vfbq.counter = 0;
    ctx->vfbq.head_ind = 0;
    ctx->vfbq.tail_ind = 0;
    ctx->ofbq.counter = 0;
    ctx->ofbq.head_ind = 0;
    ctx->ofbq.tail_ind = 0;
    ctx->oframe_with_client = 0;
    ctx->oframe_with_driver = 0;

    /* Init conditional event */
    KeInitializeEvent(&ctx->cond, SynchronizationEvent, FALSE);

    /* Init ctx mutex */
    WDF_OBJECT_ATTRIBUTES spinlockAttributes = { 0 };
    spinlockAttributes.ParentObject = dev->WdfDevice;
    WDF_OBJECT_ATTRIBUTES_INIT(&spinlockAttributes);
    WdfSpinLockCreate(&spinlockAttributes, &ctx->mutex);

    if (ctx->mutex == NULL) {
        ctx->file = NULL;
        DBG_PRINT_ERROR("ctx[%d] ExInitializeFastMutex failed ", stream_id);
        DBG_VDEC_METHOD_END();
        return STATUS_UNSUCCESSFUL;
    }

    dev->dev_opened++;
    ctx->video_format = video_format;
    rpc_set_stream_cfg_value(dev->shared_mem.pSharedInterface, ctx->stream_id, 1);

    /* Alloc user data buffer */
    status = map_vpu_buffer(&ctx->CtxMem.ud_mem, MmWriteCombined, file);
    if (status != STATUS_SUCCESS) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "ctx[%d] ExInitializeFastMutex failed ", stream_id);
        vdec_close(dev, stream_id);
        DBG_VDEC_METHOD_END();
        return STATUS_UNSUCCESSFUL;
    }

    DBG_VDEC_PRINT_INFO("ctx[%d] udata_buffer_size(%d) udata_buffer_virt(%p) udata_buffer_phy(%p)", stream_id,
                        UDATA_BUFFER_SIZE, ctx->CtxMem.ud_mem.virtAddrAligned,
                        (void *)ctx->CtxMem.ud_mem.physAddrAligned);

    /* Alloc stream buffer */
    status = map_vpu_buffer(&ctx->CtxMem.sb_mem, SB_CACHE_TYPE, file);
    if (status != STATUS_SUCCESS) {
        DBG_PRINT_ERROR_WITH_STATUS(status, "ctx[%d] ExInitializeFastMutex failed ", stream_id);
        vdec_close(dev, stream_id);
        DBG_VDEC_METHOD_END();
        return STATUS_UNSUCCESSFUL;
    }

    /*DANGEROUS! This will overwrite the init structure! */
    out_sb_mem->physAddress = ctx->CtxMem.sb_mem.physAddrAligned;
    out_sb_mem->virtAddress = ctx->CtxMem.sb_mem.virtAddrAligned;
    out_sb_mem->size = dev->config.StreamBuffSize;

    DBG_VDEC_PRINT_INFO("ctx[%d] stream_buffer_size(%d) stream_buffer_virt(%p) stream_buffer_phy(%p)", stream_id,
                        dev->config.StreamBuffSize, out_sb_mem->virtAddress,
                        (void *)out_sb_mem->physAddress);

    ctx->dcp_count = 0;
    ctx->mbi_count = 0;
    ctx->b_firstseq = TRUE;
    ctx->parsed_frames = 0;
    ctx->pending_vfb_req = 0;
    ctx->pending_ofb_req = 0;

    pStrBufDesc = (pSTREAM_BUFFER_DESCRIPTOR_TYPE)((uintptr_t)dev->RegistersPtr + DEC_MFD_XREG_SLV_BASE + MFD_MCX +
                                                   MFD_MCX_OFF * (uintptr_t)ctx->stream_id);
    /*  CAUTION: wptr must not be end */
    pStrBufDesc->wptr = (u_int32)ctx->CtxMem.sb_mem.physAddrAligned;
    pStrBufDesc->rptr = (u_int32)ctx->CtxMem.sb_mem.physAddrAligned;
    pStrBufDesc->start = (u_int32)ctx->CtxMem.sb_mem.physAddrAligned;
    pStrBufDesc->end = (u_int32)ctx->CtxMem.sb_mem.physAddrAligned + dev->config.StreamBuffSize;
    pStrBufDesc->LWM = 0x01;
    DBG_VDEC_PRINT_INFO("ctx[%d] base %p !!!!!stream_id (%d) (rptr 0x%8x wptr 0x%8x start 0x%8x end 0x%8x LWM 0x%8x)",
                        stream_id, dev->RegistersPtr,
                        ctx->stream_id,
                        pStrBufDesc->rptr, pStrBufDesc->wptr, pStrBufDesc->start, pStrBufDesc->end, pStrBufDesc->LWM);
    dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->stream_id][uStrIdx] = (VPU_REG_BASE_MCORE + DEC_MFD_XREG_SLV_BASE
                                                                                  + MFD_MCX + MFD_MCX_OFF * ctx->stream_id);


    MediaIPFW_Video_UData *pUdataBuf = &dev->shared_mem.pSharedInterface->UDataBuffer[ctx->stream_id];
    unsigned int *CurrStrfg = &dev->shared_mem.pSharedInterface->StreamConfig[ctx->stream_id];

    pUdataBuf->uUDataBase = (u_int32)ctx->CtxMem.ud_mem.physAddrAligned;
    pUdataBuf->uUDataSlotSize = UDATA_BUFFER_SIZE;

    /* S - Map MBI buffers */
    for (i = 0; i < MAX_MBI_NUM; i++) {
        VpuMemory *mbi = &ctx->CtxMem.mbi_mem[i];
        if (mbi->virtAddr) {
            DBG_PRINT_ERROR("ctx[%d] Already mapped MBI Buffer!!!!!", uStrIdx);
            unmap_vpu_buffer(mbi);
        }
        map_vpu_buffer(mbi, MmCached, ctx->file);
        DBG_VDEC_PRINT_INFO("index %d MBISize(0x%x) mbi_dma_virt(0x%p) mbi_dma_phy(0x%x)", i, (int)ctx->CtxMem.mbi_size,
                            mbi->virtAddrAligned,
                            (int)mbi->physAddrAligned);
        if (mbi->virtAddr == NULL) {
            mbi->virtAddr = 0;
            DBG_PRINT_ERROR("Unable to alloc buffer Phy Addr(%x)", (int)mbi->physAddrAligned);
            return STATUS_INSUFFICIENT_RESOURCES;
        }
    }
    /* E - Map MBI buffers */

    /* S - Map DCP buffers */
    if (video_format == MEDIA_IP_FMT_HEVC) {
        for (i = 0; i < MAX_DCP_NUM; i++) {
            VpuMemory *dcp = &ctx->CtxMem.dcp_mem[i];
            if (dcp->virtAddr) {
                DBG_PRINT_ERROR("ctx[%d] Already mapped MBI Buffer!!!!!", uStrIdx);
                unmap_vpu_buffer(dcp);
            }
            map_vpu_buffer(dcp, MmCached, ctx->file);
            DBG_VDEC_PRINT_INFO("index %d MBISize(0x%x) mbi_dma_virt(0x%p) mbi_dma_phy(0x%x)", i, ctx->CtxMem.dcp_size,
                                dcp->virtAddrAligned,
                                (int)dcp->physAddrAligned);
            if (dcp->virtAddr == NULL) {
                dcp->virtAddr = 0;
                DBG_PRINT_ERROR("Unable to alloc buffer Phy Addr(%x)", (int)dcp->physAddrAligned);
                return STATUS_INSUFFICIENT_RESOURCES;
            }
        }
    }
    /* E - Map DCP buffers */

    /* Map frame buffers */
    if (fbl_vfb_update(ctx->CtxMem.fbl, dev->config.FrameBuffers) != STATUS_SUCCESS) {
        DBG_PRINT_ERROR("ctx[%d]  Failed to map luma/chroma buffers", uStrIdx);
        ctx->error = VDEC_FATAL_ERROR_NOMEM;
        return STATUS_INSUFFICIENT_RESOURCES;;
    }
    /* Map frame buffers */
    if (fbl_ofb_update(ctx->CtxMem.fbl, dev->config.OutFrameBuffers) != STATUS_SUCCESS) {
        DBG_PRINT_ERROR("ctx[%d]  Failed to map RGBA buffers", uStrIdx);
        ctx->error = VDEC_FATAL_ERROR_NOMEM;
        return STATUS_INSUFFICIENT_RESOURCES;;
    }
    fbl_start(ctx->CtxMem.fbl);

    VID_STREAM_CONFIG_FORMAT_SET(malone_format, CurrStrfg);

    if (video_format == MEDIA_IP_FMT_JPG) {
        MediaIPFW_Video_JpegParams *pJpgPara;

        pJpgPara = (MediaIPFW_Video_JpegParams *)ctx->dev->shared_mem.jpeg_mem_vir;
        pJpgPara[ctx->stream_id].uJpgMjpegMode = 1; /*1:JPGD_MJPEG_MODE_A; 2:JPGD_MJPEG_MODE_B */
        pJpgPara[ctx->stream_id].uJpgMjpegInterlaced = 0; /*0: JPGD_MJPEG_PROGRESSIVE */
    }
    /* Configure stream */
    pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[ctx->stream_id];

    if (dec->frame_mode) {
        buffer_info->stream_input_mode = FRAME_LVL;
        buffer_info->stream_pic_parsed_count = 0;
        buffer_info->stream_pic_input_count = 0;
        buffer_info->stream_buffer_threshold = 0;
        buffer_info->stream_pic_end_flag = 0;

        ctx->frame_mode = 1;
    } else {
        buffer_info->stream_input_mode = NON_FRAME_LVL;
        buffer_info->stream_pic_parsed_count = 0;
        buffer_info->stream_pic_input_count = 0;
        buffer_info->stream_buffer_threshold = 0;
        buffer_info->stream_pic_end_flag = 0;

        ctx->frame_mode = 0;
    }

    /* Disable reorder to support low latency */
    pCodecPara = (MediaIPFW_Video_CodecParams *)ctx->dev->shared_mem.codec_mem_vir;

    if (dec->dis_reorder == 1) {
        ctx->dis_reorder = 1;
        pCodecPara[ctx->stream_id].uDispImm = 1;
    } else {
        ctx->dis_reorder = 0;
        pCodecPara[ctx->stream_id].uDispImm = 0;
    }

    if (dec->force_fbufs_num > 0) {
        ctx->force_fbufs_num = dec->force_fbufs_num;
    } else {
        ctx->force_fbufs_num = 0;
    }

    pCodecPara[ctx->stream_id].uEnableDbgLog = 1;                        /* vpu_frmdbg_ena */
    dev->shared_mem.pSharedInterface->DbgLogDesc.uDecStatusLogLevel = 2;  /* firmware debug level (0-2) */
#if 0
    /* Open the file for writing*/
    char dbg_file_name[32];
    sprintf(dbg_file_name, "/tmp/DbgLog_ctx%d.txt", ctx->stream_id);
    ctx->dbg_file = fopen(dbg_file_name, "w");
#endif
    ctx->status = MEDIA_PLAYER_STOPPED;

    DBG_VDEC_PRINT_INFO("ctx[%d] vdec_opened!", stream_id);
    DBG_VDEC_PRINT_INFO("ctx[%d] Decoder init done!", stream_id);
    DBG_VDEC_PRINT_INFO("ctx[%d] Malone format = %s. !", stream_id, fmt2str[malone_format]);
    DBG_VDEC_PRINT_INFO("ctx[%d] Disable reorder = %d. !", stream_id, ctx->dis_reorder);
    DBG_VDEC_PRINT_INFO("ctx[%d] %s", stream_id, ctx->frame_mode ? "FRAME_MODE" : "NON FRAME MODE");

    if (ctx->status == MEDIA_PLAYER_STARTED) {
        DBG_VDEC_PRINT_INFO("ctx[%d] started!", stream_id);
    }

    DBG_VDEC_METHOD_END();
    return STATUS_SUCCESS;
}

/**
 * vdec_stop()
 * @brief Stop decoder instance
 * @param dev
 * @param stream_id
 * @param block
 * @return NTSTATUS
 */
NTSTATUS vdec_stop(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, int block)
{
    UNREFERENCED_PARAMETER(block);

    NTSTATUS Status = STATUS_SUCCESS;
    vdec_ctx_t *ctx = NULL;

    DBG_VDEC_METHOD_BEG();

    if (stream_id != -1) {
        ctx = &dev->ctx[stream_id];
    } else {
        return STATUS_UNSUCCESSFUL;
    }

    WdfSpinLockAcquire(ctx->mutex);
    DBG_VDEC_PRINT_INFO("ctx[%d] vdec_stop enter; blocking: %d; ctx status: %s;", ctx->stream_id, block,
                        sts2str[ctx->status]);
    if (ctx->status == MEDIA_PLAYER_STOPPED) {
        DBG_VDEC_PRINT_INFO("ctx[%d] Already stopped", ctx->stream_id);
        WdfSpinLockRelease(ctx->mutex);
        DBG_VDEC_METHOD_END();
        return Status;
    } else if (ctx->status == MEDIA_PLAYER_STARTED || ctx->status == MEDIA_PLAYER_STARTING) {
        vfb_t *vfb = { 0 };
        ofb_t *ofb = { 0 };
        /* User stop sending STOP cmd to VPU before VPU responding with STARTED event cause no STOPPED event from VPU
         * blocking call until VPU responds with STARTED event
         */
        if (ctx->status == MEDIA_PLAYER_STARTING) {
            DBG_EVENTS_PRINT_INFO("ctx[%d] Waiting for VID_API_EVENT_START_DONE event from VPU", ctx->stream_id);
            do {
                WdfSpinLockRelease(ctx->mutex);
                Status = KeWaitForSingleObject(&ctx->cond, Executive, KernelMode, FALSE, NULL);
                if (Status != STATUS_SUCCESS) {
                    DBG_PRINT_ERROR("ctx[%d] pthread_cond_wait failed (%d)", ctx->stream_id, Status);
                    DBG_VDEC_METHOD_END();
                    return Status;
                }
                WdfSpinLockAcquire(ctx->mutex);
            } while (ctx->status == MEDIA_PLAYER_STARTING);
            DBG_VDEC_PRINT_INFO("ctx[%d] Received VID_API_EVENT_START_DONE event from VPU", ctx->stream_id);
        }
        ctx->status = MEDIA_PLAYER_STOPPING;
        /* Client will not consume decoded frame buffers anymore
         * vpu will not push frame buffers to queue anymore
         * release all buffers from frame buffer queue */

        while (pop_vfb_entry(&ctx->vfbq, &vfb) == ERR_EOK) {
            fbl_vfb_release(ctx->CtxMem.fbl, vfb);
        }
        while (pop_ofb_entry(&ctx->ofbq, &ofb) == ERR_EOK) {
            fbl_ofb_release(ctx->CtxMem.fbl, ofb);
        }

        fbl_stop(ctx->CtxMem.fbl);
        DBG_VDEC_PRINT_INFO("ctx[%d] vdec_stop .....!", ctx->stream_id);
        vpu_send_cmd(dev, ctx->stream_id, VID_API_CMD_STOP, 0, NULL);
        ctx->dcp_count = 0;
        ctx->mbi_count = 0;
        WdfSpinLockRelease(ctx->mutex);
        Status = STATUS_SUCCESS;
    } else {
        WdfSpinLockRelease(ctx->mutex);
        Status = STATUS_SUCCESS;
    }
    /* Wait for stopped state */
    if (block == VPU_BLOCK) {
        while (ctx->status == MEDIA_PLAYER_STOPPING) {
            DBG_EVENTS_PRINT_INFO("ctx[%d] Waiting for VID_API_EVENT_STOPPED event from VPU", ctx->stream_id);
            WdfSpinLockRelease(ctx->mutex);
            if ((KeWaitForSingleObject(&ctx->cond, Executive, KernelMode, FALSE, NULL)) != STATUS_SUCCESS) {
                DBG_PRINT_ERROR("ctx[%d] KeWaitForSingleObject failed (%d)", ctx->stream_id, Status);
                DBG_PRINT_ERROR("ctx[%d] ERROR: EVENT STOPPED TIMEOUT! (%d)", ctx->stream_id, Status);
                DBG_VDEC_METHOD_END();
                return Status;
            }
        }
    }

    DBG_VDEC_PRINT_INFO("ctx[%d] vdec_stop exit", ctx->stream_id);
    DBG_VDEC_METHOD_END();
    return Status;
}

/**
 * vdec_status()
 * @brief Get bitstream buffer info, player status, eos
 * @param dev* : MALONE_VPU_DEVICE_CONTEXT
 * @param stream_id : Stream ID
 * @param status : vdec_status_t
 * @return NTSTATUS
 */
NTSTATUS vdec_status(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, vdec_status_t *status)
{
    NTSTATUS ret = STATUS_UNSUCCESSFUL;
    vdec_ctx_t *ctx = &dev->ctx[stream_id];
    DBG_VDEC_METHOD_BEG();
    DBG_EVENTS_PRINT_INFO("MFT->KM decoder_status() ctx[%d]", stream_id);
    if (ctx) {

        u_int32 uStrIdx = 0; /* Set to be default 0, FIX_ME later */
        WdfSpinLockAcquire(ctx->mutex);
        pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc = (pSTREAM_BUFFER_DESCRIPTOR_TYPE)((uintptr_t)dev->RegistersPtr +
                                                                                      DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF *
                                                                                      (uintptr_t)ctx->stream_id);
        DSB;
        volatile u_int32 temp = pStrBufDesc->wptr;
        status->wptr = temp;
        status->rptr = pStrBufDesc->rptr;
        status->start = pStrBufDesc->start;
        status->end = pStrBufDesc->end;
        dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->stream_id][uStrIdx] = (VPU_REG_BASE_MCORE + DEC_MFD_XREG_SLV_BASE
                                                                                      +
                                                                                      MFD_MCX + MFD_MCX_OFF *
                                                                                      (uintptr_t)ctx->stream_id);
        if (ctx->frame_mode) {
            status->parsed_frames = ctx->pic_hdr_cnt;
        }

        status->stopped = ctx->status == MEDIA_PLAYER_STOPPED ? 1 : 0;
        status->eos = ctx->eos;

        WdfSpinLockRelease(ctx->mutex);
        ret = STATUS_SUCCESS;
    }
    DBG_VDEC_METHOD_END();
    return ret;
}

/**
 * vdec_decode()
 * @brief Updates wptr, returns output buffer index if decoded output is available
 * @param dev* : MALONE_VPU_DEVICE_CONTEXT
 * @param stream_id : Stream ID
 * @param dec : Pointer to decoder structure vdec_decode_t
 * @return
 */
NTSTATUS vdec_decode(MALONE_VPU_DEVICE_CONTEXT *dev, int stream_id, vdec_decode_t *dec)
{
    NTSTATUS ret = STATUS_UNSUCCESSFUL;
    int32_t depth = 0;
    DBG_VDEC_METHOD_BEG();

    DBG_EVENTS_PRINT_INFO("MFT->KM decoder_decode() ctx[%d], wptr=0x%08d, eos=%d, timeout=%d, frame_in_cnt=%d, trick_mode=%d",
                          stream_id,
                          dec->wptr,
                          dec->eos,                /* Param in:  set when input stream eos is detected */
                          dec->timeout,            /* Param in:  timeout to wait for decoded output */
                          dec->frame_in_cnt,       /* Param In:  Frame counter must be updated in frame mode */
                          dec->trick_mode);        /* Param In: trick play mode */

    vdec_ctx_t *ctx = &dev->ctx[stream_id];

    if (ctx) {
        pBUFFER_INFO_TYPE buffer_info = &dev->shared_mem.pSharedInterface->StreamBuffInfo[ctx->stream_id];
        ofb_t *ofb = { 0 };
        pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc = NULL;
        u_int32 uStrIdx = 0; /* Set to be default 0, FIX_ME later */
        ret = STATUS_SUCCESS;
        dec->oindex = -1;
        dec->width = ctx->real_width;
        dec->height = ctx->real_height;
        dec->error = VDEC_EOK;
        WdfSpinLockAcquire(ctx->mutex);

        if (ctx->aborts == ABORT_PENDING) {
            WdfSpinLockRelease(ctx->mutex);
            DBG_VDEC_METHOD_END();
            return STATUS_SUCCESS;
        } else if (ctx->aborts == ABORT_DONE) {
            ctx->aborts = ABORT_INIT;/*reset */
        }

        if (ctx->frame_mode) {
            if (dec->frame_in_cnt != (int)buffer_info->stream_pic_input_count) {
                DBG_VDEC_PRINT_INFO("=>ctx[%d]%s frame %d ", ctx->stream_id, ctx->dis_reorder ? "DR" : "R", dec->frame_in_cnt);
            }
            buffer_info->stream_pic_input_count = dec->frame_in_cnt;
            if (dec->eos) {
                buffer_info->stream_pic_end_flag = 1;
                if (dec->frame_in_cnt == 0 && dec->eos) {
                    DBG_PRINT_ERROR("ctx[%d] EOS added without frames in BS buffer, Why I am here ????", ctx->stream_id);
                    WdfSpinLockRelease(ctx->mutex);
                    DBG_VDEC_METHOD_END();
                    return STATUS_SUCCESS;
                }
            }
        }

        pStrBufDesc = (pSTREAM_BUFFER_DESCRIPTOR_TYPE)((uintptr_t)dev->RegistersPtr + DEC_MFD_XREG_SLV_BASE + MFD_MCX +
                                                       MFD_MCX_OFF * (uintptr_t)ctx->stream_id);

        if (pStrBufDesc->wptr != dec->wptr) {
            KeFlushIoBuffers(ctx->CtxMem.sb_mem.mdl, FALSE /*WRITE-OP-FLUSH*/, TRUE);
            pStrBufDesc->wptr = dec->wptr;
        }
        dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->stream_id][uStrIdx] = (VPU_REG_BASE_MCORE + DEC_MFD_XREG_SLV_BASE
                                                                                      + MFD_MCX + MFD_MCX_OFF * ctx->stream_id);
        for (;;) {
            vdec_buffer_status(ctx, 0);

            if ((ctx->status == MEDIA_PLAYER_STOPPED) && (ctx->remains > 0)) {
                ctx->status = MEDIA_PLAYER_STARTING;
                vpu_send_cmd(dev, ctx->stream_id, VID_API_CMD_START, 0, NULL);
                DBG_VDEC_PRINT_INFO("ctx[%d] remains %d frame %d", ctx->stream_id, ctx->remains, buffer_info->stream_pic_input_count);
            }
            depth = dec->frame_in_cnt - ctx->pic_hdr_cnt;

            if (ctx->error >= VDEC_FATAL_ERROR) {
                dec->error = ctx->error;
                ret = STATUS_SUCCESS;
                DBG_PRINT_ERROR("ctx[%d] fatal error %s!!!", ctx->stream_id, vdec_error2str[ctx->error]);
                break;
            } else if (ctx->status == MEDIA_PLAYER_STOPPED || ctx->status == MEDIA_PLAYER_STOPPING
                       || ctx->aborts == ABORT_PENDING) {
                /* Decoding is not started yet or stopping or stopped or pending abort */
                break;
            } else if (ctx->status == MEDIA_PLAYER_STARTING) {
                if ((ret = block(ctx, 100)) != STATUS_SUCCESS) {
                    break;
                }
            } else if ((ctx->b_firstseq && ctx->frame_mode && !ctx->eos)) {
                dec->error = VDEC_LOW_INPUT_FRAMES;
                break;
            } else if (dec->trick_mode && dec->eos && !ctx->eos) {
                /* Blocking call until vpu reports finished event */
                if ((ret = block(ctx, 0)) != STATUS_SUCCESS) {
                    break;
                }
            } else if (pop_ofb_entry(&ctx->ofbq, &ofb) == STATUS_SUCCESS) {
                /* Have decoded output */
                /* */
                ret = STATUS_SUCCESS;
                dec->oindex = ofb->index;
                ctx->oframe_with_client++;
                /*ctx->oframe_with_driver++; */
                ctx->no_out_cnt = 0;
                ofb->held_by_client = 1;
                if (dec->trick_mode && dec->eos) {
                    ctx->eos = 0;
                }
                break;
            } else if (!ctx->dis_reorder && ctx->frame_mode && !dec->eos && (depth < FRAME_DEPTH)) {
                ret = STATUS_SUCCESS;
                dec->error = VDEC_LOW_INPUT_FRAMES;
                break;

            } else if (fbl_ofb_waiting(ctx->CtxMem.fbl) && (ctx->oframe_with_client > 0)) {
                dec->error = VDEC_LOW_OUTPUT_BUFFERS;
                ret = STATUS_SUCCESS;
                break;
            } else if (ctx->eos) {
                /* Vpu triggered finsihed event */
                DBG_VDEC_PRINT_INFO("ctx[%d] vdec_decode All frames have been processed. VPU set EOS. Reseting...", ctx->stream_id);
                ctx->eos = 0; /* Reset EOS flag after flush */
                ret = STATUS_SUCCESS;
                break;
            } else if (dec->eos || dec->timeout) {
                int timeout = dec->eos ? 0 : dec->timeout;
                DBG_EVENTS_PRINT_INFO("ctx[%d] vdec_decode wait for output( eos %d,remains %d status %d eos=%d)", ctx->stream_id,
                                      dec->eos,
                                      ctx->remains, ctx->status,
                                      ctx->eos);
                if ((ret = block(ctx, timeout)) != STATUS_SUCCESS) {
                    DBG_PRINT_ERROR("ctx[%d] wait for output timedout (err %d)", ctx->stream_id, ret);
                    break;
                }
            } else {
                break;
            }
        }
        /* Debugging purpose */
        if ((ctx->status == MEDIA_PLAYER_STARTED) && (dec->oindex == -1)) {
            ctx->no_out_cnt++;
            /*fbl_log(ctx->CtxMem.fbl); */
        }

        DBG_EVENTS_PRINT_INFO("KM->MFT ctx[%d] returning: oindex=%d, width=%d, heigth=%d, depth=%d, error=%s)", ctx->stream_id,
                              dec->oindex,
                              dec->width,
                              dec->height,
                              depth,
                              vdec_error2str[dec->error]
                             );
        WdfSpinLockRelease(ctx->mutex);
    }


    if (dec->oindex == -1 && dec->error == VDEC_EOK) {
        fbl_log(ctx->CtxMem.fbl);
        DBG_VDEC_PRINT_INFO("JUST TO BE SURE : ctx->ofb_pending=%d, ctx->vfb_pending=%d", ctx->pending_ofb_req,
                            ctx->pending_vfb_req);
        if (depth >= FRAME_DEPTH) {
            DBG_PRINT_ERROR("!!! Something wrong is happening with releasing buffers!");
            t2l_frame(ctx);
        }
    }

    DBG_VDEC_METHOD_END();

    return ret;
}
