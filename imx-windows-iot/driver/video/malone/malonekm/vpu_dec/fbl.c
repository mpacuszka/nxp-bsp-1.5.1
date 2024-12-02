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
    #include "fbl.tmh"
#endif
#include "imx8q_driver.h"
#include "vpu_rpc/mediasys_types.h"
#include "vpu_dec/fbl.h"

/**
 * frame_status_to_str()
 * @brief Converts Frame buffer status to string
 * @params status
 * @return char* string
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

/**
 * fbl_create()
 * @brief Allocates list of frame buffers
 * @return fbl_t*  pointer to Frame Buffer list handle
 */
fbl_t *fbl_create()
{
    fbl_t *list = (fbl_t *)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(fbl_t), MALONE_VPU_POOL_TAG);

    if (list == NULL) {
        return NULL;
    }
    return list;
}

/**
 * fbl_ofb_update()
 * @brief Update pool with ouput buffer info.
 * @params list : pointer to Frame Buffer list handle
 * @params capacity : number of frame buffers
 * @return NTSTATUS
 */
NTSTATUS fbl_ofb_update(fbl_t *list, size_t capacity)
{
    uint32_t i = 0;
    if (list) {
        if (list->ofb == NULL) {
            DBG_PRINT_ERROR("ctx[%d] list->fb is NULL and should be allocated in OnPrepareHardware", *list->stream_id);
            return STATUS_INSUFFICIENT_RESOURCES;
        }
        list->ofb_capacity = capacity;
        for (i = 0; i < capacity; i++) {
            ofb_t *ele = &list->ofb[i];
            VpuMemory *mem = &ele->mem;
            if (map_vpu_buffer(mem, MmCached, *list->file) != STATUS_SUCCESS) {
                mem->virtAddr = 0;
                return STATUS_INSUFFICIENT_RESOURCES;
            }
            ele->index = i;
            ele->acquired = FALSE;
            ele->held_by_client = FALSE;
            ele->rdy_for_display = FALSE;
            DBG_FBL_PRINT_VERBOSE("ctx[%d] OFB[%d] rgba.virt(phy) = %p(%p)",
                                  *list->stream_id,
                                  i,
                                  (void *)mem->virtAddrAligned,
                                  (void *)mem->physAddrAligned
                                 );

        }
        list->flags = MPOOL_HAVE_BUFFERS;
    }
    return STATUS_SUCCESS;
}

/**
 * fbl_vfb_update()
 * @brief Update pool with ouput buffer info.
 * @params list : pointer to Frame Buffer list handle
 * @params capacity : number of frame buffers
 * @return NTSTATUS
 */
NTSTATUS fbl_vfb_update(fbl_t *list, size_t capacity)
{
    uint32_t i = 0;
    if (list) {
        if (list->vfb == NULL) {
            DBG_PRINT_ERROR("ctx[%d] list->fb is NULL and should be allocated in OnPrepareHardware", *list->stream_id);
            return STATUS_INSUFFICIENT_RESOURCES;
        }
        list->vfb_capacity = capacity;
        for (i = 0; i < capacity; i++) {
            vfb_t *ele = &list->vfb[i];
            VpuMemory *luma = &ele->luma;
            VpuMemory *chroma = &ele->chroma;
            if (map_vpu_buffer(luma, MmCached, *list->file) != STATUS_SUCCESS) {
                luma->virtAddr = 0;
                return STATUS_INSUFFICIENT_RESOURCES;
            }
            if (map_vpu_buffer(chroma, MmCached, *list->file) != STATUS_SUCCESS) {
                luma->virtAddr = 0;
                return STATUS_INSUFFICIENT_RESOURCES;
            }
            DBG_FBL_PRINT_VERBOSE("ctx[%d] FB[%d] luma.virt(phy) %p(%p) chroma.virt(phy) %p(%p)",
                                  *list->stream_id,
                                  i,
                                  (void *)luma->virtAddrAligned,
                                  (void *)luma->physAddrAligned,
                                  (void *)chroma->virtAddrAligned,
                                  (void *)chroma->physAddrAligned
                                 );

            ele->status = FRAME_ALLOC;
            ele->index = i;
            ele->acquired = FALSE;
            ele->held_by_g2d = FALSE;
            ele->rdy_for_blit = FALSE;
        }
    }
    return STATUS_SUCCESS;
}

/**
 * fbl_find_by_addr()
 * @brief Find Frame Buffer by luma phys address
 * @params list : pointer to Frame Buffer list handle
 * @params luma_phy_addr : luma buffer physical address
 * @return fb_t* pointer to Frame Buffer Element
 */
vfb_t *fbl_find_by_addr(fbl_t *list, uintptr_t luma_phy_addr)
{
    int i = 0;
    if (list) {
        for (i = 0; i < list->vfb_capacity; i++) {
            vfb_t *ele = &list->vfb[i];
            VpuMemory *luma = &ele->luma;
            if (luma->physAddrAligned == luma_phy_addr) {
                return ele;
            }
        }
    } else {
        DBG_FBL_PRINT_VERBOSE("No FrameBuffers are not Registered by client");
    }
    return NULL;
}

/**
 * fbl_find_by_index()
 * @brief Find Frame Buffer by index
 * @param list* : pointer to Frame Buffer list handle
 * @param index : luma buffer physical address
 * @return fb_t* : Pointer rto Frame Buffer Element
 */
vfb_t *fbl_find_by_index(fbl_t *list, int index)
{
    if (list && index < list->vfb_capacity) {
        return &list->vfb[index];
    }
    return NULL;
}

/**
 * fbl_vfb_set_status()
 * @brief Set VPU Frame Buffer Status
 * @param list* : pointer to Frame Buffer list handle
 * @param ele* : ele, pointer to Frame Buffer
 * @param status
 */
void fbl_vfb_set_status(fbl_t *list, vfb_t *ele, int status)
{
    UNREFERENCED_PARAMETER(list);
    if (ele) {
        DBG_FBL_PRINT_VERBOSE("ctx[%d] FRAME_STATUS [%d]: %s->%s", *list->stream_id, ele->index,
                              frame_status_to_str(ele->status),
                              frame_status_to_str(status));
        ele->status = status;
        if ((status == FRAME_RELEASE) && (ele->rdy_for_blit == FALSE) && (ele->held_by_g2d == FALSE)) {
            /* VPU might release the buffer without using it for decoding
             * could happen during stop or abort transition. */
            ele->acquired = FALSE;
        }
    } else {
        DBG_FBL_PRINT_VERBOSE("ctx[%d] element is NULL", *list->stream_id);
    }
}

/**
 * fbl_clear()
 * @brief Release all the VPU Frame Buffers
 * @param list* : pointer to Frame Buffer list handle
 */
void fbl_vfb_clear(fbl_t *list)
{
    int i = 0;
    vfb_t *ele = NULL;
    if (list) {
        for (i = 0; i < list->vfb_capacity; i++) {
            ele = &list->vfb[i];
            ele->status = FRAME_RELEASE;
            /* Acquired by vpu, but not in display queue or held by client */
            ele->acquired = FALSE;
            ele->held_by_g2d = FALSE;
            ele->rdy_for_blit = FALSE;
        }
    }
}

/**
 * fbl_vfb_release()
 * @brief Release Frame Buffer
 * @param list* : pointer to Frame Buffer list handle
 * @param index : output Frame Buffer index
 */
void fbl_vfb_release(fbl_t *list, vfb_t *ele)
{
    if (list) {
        if (ele->acquired) {
            ele->acquired = FALSE;
            ele->rdy_for_blit = FALSE;
            ele->held_by_g2d = FALSE;
            DBG_FBL_PRINT_INFO("ctx[%d] VFB Frame Buffer[%d] RELEASED", *list->stream_id, ele->index);
            if (ele->status == FRAME_RELEASE) {
                /* VPU already released the buffer back to pool so buffer is available immediately to reuse. */
                if (list->vfb_waiting) {
                    list->vfb_waiting = FALSE;
                    /*Wake up vpu event handler thread if it is waiting for vfb buffer */
                    KeSetEvent(list->cond, IO_VIDEO_INCREMENT, 0);
                }
            }
        }
    }
}

/**
 * fbl_ofb_release()
 * @brief Release Output Frame Buffer
 * @param list* : pointer to Frame Buffer list handle
 * @param index : output Frame Buffer index
 */
void fbl_ofb_release(fbl_t *list, ofb_t *ele)
{
    if (ele->acquired) {
        ele->acquired = FALSE;
        ele->rdy_for_display = FALSE;
        DBG_FBL_PRINT_INFO("ctx[%d] OFB Frame Buffer[%d] RELEASED", *list->stream_id, ele->index);
        /*DBG_FBL_PRINT_VERBOSE("ctx[%d] Frame Buffer[%d] phys_addr = %p virt_addr %p", *list->stream_id, ele->index,
           ele->mem.physAddrAligned, ele->mem.virtAddrAligned);
         */
        /*vfb and ofb buffers are decoupled. buffer is available immediately to reuse. */
        if (list->ofb_waiting) {
            list->ofb_waiting = FALSE;
            /*Wake up vpu event handler thread if waiting for ofb buffer */
            KeSetEvent(list->cond, IO_VIDEO_INCREMENT, 0); /* TODO T2L do I need to trigger? */
        }
    }
}

/**
 * fbl_vfb_acquire()
 * @brief Acquire VPU Frame Buffer if available, else block
 * @param list* : pointer to Frame Buffer list handle
 * @return fb_t* : Pointer to Frame Buffer element
 */
vfb_t *fbl_vfb_acquire(fbl_t *list)
{

    int i = 0;
    vfb_t *ele = NULL;
    if (list) {
        while (!(list->flags & MPOOL_STOPPING) && (*list->aborts != 1 /*ABORT_PENDING*/)) {
            if (!list->vfb_waiting) {
                int index = list->vfb_index;
                int held_by_vpu = 0;
                int held_for_t2l = 0;
                for (i = 0; i < list->vfb_capacity; i++) {
                    index = (index == list->vfb_capacity) ? 0 : index;
                    ele = &list->vfb[index];
                    if ((!ele->acquired) && (ele->status == FRAME_ALLOC || ele->status == FRAME_RELEASE)) {
                        ele->acquired = TRUE;
                        ele->held_by_g2d = FALSE;
                        ele->rdy_for_blit = FALSE;
                        list->vfb_index = index + 1;
                        DBG_FBL_PRINT_VERBOSE("ctx[%d] Frame Buffer[%d] luma 0x%8x chroma 0x%8x", *list->stream_id, ele->index,
                                              (unsigned int)ele->luma.physAddrAligned,
                                              (unsigned int)ele->chroma.physAddrAligned);
                        return ele;
                    }
                    if ((ele->status == FRAME_FREE) || (ele->status == FRAME_DECODED) || (ele->status == FRAME_READY)) {
                        held_by_vpu++;
                    }
                    if (ele->acquired && ((ele->status == FRAME_READY) || (ele->status == FRAME_RELEASE))) {
                        held_for_t2l++;
                    }
                    index++;
                }
                list->vfb_waiting = TRUE;
                /* To wake up other thread waiting on same cond. and other thread should not goback to waiting again if acquire is blocked. */
                KeSetEvent(list->cond, IO_VIDEO_INCREMENT, 0);
                ele = NULL;
                if (held_by_vpu == list->vfb_capacity) {
                    /* This should never happen if rsmgr allocates enough frame buffers
                     * but we have seen vpu requesting for frame buffer after fisnished event while vpu is holding all the frame buffers
                     * working with nxp to root cause this problem.
                     * temp solution is avoid stall and serve the buffer when vpu release frame buffer
                     * NXP-383 is created to track this issue
                     */
                    list->vfb_waiting = FALSE;
                    break;
                }
            }
            fbl_log(list);
#if 0   /* This can cause to wait in DPC too long - imposible in windows solution*/
            if ((KeWaitForSingleObject(list->cond, Executive, KernelMode, FALSE, NULL)) != STATUS_SUCCESS) {
                DBG_PRINT_ERROR("player %d pthread_cond_wait failed.", *list->stream_id);
                break;
            }
            list->waiting = FALSE;
#else
            /* Frame buffer request will be served when buffer is released by client */
            (*list->pending_vfb_req)++;
            return ele = NULL;
#endif
        }
    }
    return ele;
}

/**
 * fbl_ofb_acquire()
 * @brief Acquire Output Frame Buffer if available, else block
 * @param list* : pointer to Frame Buffer list handle
 * @return fb_t* : Pointer to Frame Buffer element
 */
ofb_t *fbl_ofb_acquire(fbl_t *list)
{
    int i = 0;
    ofb_t *ele = NULL;
    int ret = 0;
    UNREFERENCED_PARAMETER(ret);
    while (!(list->flags & MPOOL_STOPPING) && (*list->aborts != 1 /*ABORT_PENDING*/)) {
        if (!list->ofb_waiting) {
            int index = list->ofb_index;
            for (i = 0; i < list->ofb_capacity; i++) {
                index = (index == list->ofb_capacity) ? 0 : index;
                ele = &list->ofb[index];
                list->ofb_index = index + 1;
                if (ele->acquired == FALSE) {
                    DBG_FBL_PRINT_VERBOSE("ctx[%d] Frame Buffer[%d] stride %d sw %d h %d\n", *list->stream_id, index, ele->stride,
                                          ele->width, ele->height);
                    ele->acquired = TRUE;
                    return ele;
                }
                index++;
            }
        }
        list->ofb_waiting = TRUE;
        /* To wake up other thread waiting on same cond. and other thread should not goback to waiting again if acquire is blocked. */
        KeSetEvent(list->cond, IO_VIDEO_INCREMENT, 0);
        ele = NULL;
#if 0 /* WINDOWS DPC cannot wait! */
        if ((ret = pthread_cond_wait(&ctx->cond, &ctx->mutex)) != EOK) {
            LOGE("player %d pthread_cond_wait failed (%d)", ctx->stream_id, ret);
            break;
        }
#else
        /* Frame buffer request will be served when buffer is released by client */
        (*list->pending_ofb_req)++;
        return ele = NULL;
#endif
    }
    return ele;
}

/**
 * fbl_log()
 * @brief Log all the buffers info ( debug purpose)
 * @param list* : pointer to Frame Buffer list handle
 */
void fbl_log(fbl_t *list)
{
    int i = 0;
    if (list) {
        int held_by_vpu = 0;
        int held_by_g2d = 0;
        int available = 0;
        /* VFB */
        for (i = 0; i < list->vfb_capacity; i++) {
            vfb_t *ele = &list->vfb[i];
            DBG_EVENTS_PRINT_INFO("ctx[%d] Frame Buffer[%d] luma = 0x%8x  chroma = 0x%8x acquired %d status %s", *list->stream_id,
                                  i,
                                  (unsigned int)ele->luma.physAddrAligned, (unsigned int)ele->chroma.physAddrAligned, ele->acquired,
                                  frame_status_to_str(ele->status));
            if (!ele->acquired && (ele->status == FRAME_ALLOC || ele->status == FRAME_RELEASE)) {
                available++;
            }
            if ((ele->status == FRAME_FREE || ele->status == FRAME_DECODED || ele->status == FRAME_READY)) {
                held_by_vpu++;
            }
            if (ele->acquired && (ele->status == FRAME_RELEASE)) {
                held_by_g2d++;
            }
        }
        DBG_EVENTS_PRINT_INFO("ctx[%d] stats held_by_vpu %d, held_by_g2d %d,available %d", *list->stream_id, held_by_vpu,
                              held_by_g2d, available);
        /* OFB */
        int ready_for_display = 0;
        int held_by_client = 0;
        available = 0;
        held_by_g2d = 0;

        for (i = 0; i < list->ofb_capacity; i++) {
            ofb_t *ele = &list->ofb[i];
            DBG_EVENTS_PRINT_INFO("ctx[%d] Out Frame Buffer[%d] rgba.phys = 0x%8x acquired %d", *list->stream_id, i,
                                  (unsigned int)ele->mem.physAddrAligned, ele->acquired);
            if (!ele->acquired) {
                available++;
            }
            if (ele->held_by_client) {
                held_by_client++;
            }
            if (ele->rdy_for_display) {
                ready_for_display++;
            }
            if (ele->acquired && !ele->rdy_for_display) {
                held_by_g2d++;
            }
        }
        DBG_EVENTS_PRINT_INFO("ctx[%d] stats held_by_client %d, ready_for_display %d, held_by_g2d %d,available %d",
                              *list->stream_id, held_by_client,
                              ready_for_display, held_by_g2d, available);
    }

    DBG_EVENTS_PRINT_INFO("ctx[%d] stats ofb_pending=%d, vfb_pending=%d", *list->stream_id,
                          *list->pending_ofb_req,
                          *list->pending_vfb_req);
    DBG_EVENTS_PRINT_INFO("ctx[%d] stats ofb_waiting=%d, vfb_waiting=%d", *list->stream_id,
                          (UINT32)(list->ofb_waiting),
                          (UINT32)(list->vfb_waiting));
}

/**
 * fbl_start()
 * @brief Sets flag, that fbl is active
 * @param list* : pointer to Frame Buffer list handle
 */
void fbl_start(fbl_t *list)
{
    if (list) {
        list->flags &= ~MPOOL_STOPPING;
    }
}

/**
 * fbl_stop()
 * @brief Unblock if there is acquire block
 * @param list* : pointer to Frame Buffer list handle
 */
void fbl_stop(fbl_t *list)
{
    if (list) {
        list->flags |= MPOOL_STOPPING;
        list->vfb_waiting = FALSE;
        list->vfb_index = 0;
        list->ofb_waiting = FALSE;
        list->ofb_index = 0;
        KeSetEvent(list->cond, IO_VIDEO_INCREMENT, 0);
    }
}

/**
 * fbl_vfb_waiting()
 * @brief Test if fbl is waiting
 * @param list* : pointer to Frame Buffer list handle
 * @return TRUE or FALSE
 */
BOOL fbl_vfb_waiting(fbl_t *list)
{
    if (list) {
        return list->vfb_waiting;
    } else {
        return FALSE;
    }
}

/**
 * fbl_vfb_size()
 * @brief Query VPU frame buffer capacity
 * @param list* : pointer to Frame Buffer list handle
 * @return Number of Frame Buffers
 */
size_t fbl_vfb_size(fbl_t *list)
{
    if (list) {
        return list->vfb_capacity;
    } else {
        return 0;
    }
}

/**
 * fbl_ofb_waiting()
 * @brief Test if fbl is waiting
 * @param list* : pointer to Frame Buffer list handle
 * @return TRUE or FALSE
 */
BOOL fbl_ofb_waiting(fbl_t *list)
{
    if (list) {
        return list->ofb_waiting;
    } else {
        return FALSE;
    }
}

/**
 * fbl_ofb_size()
 * @brief Query VPU frame buffer capacity
 * @param list* : pointer to Frame Buffer list handle
 * @return Number of Frame Buffers
 */
size_t fbl_ofb_size(fbl_t *list)
{
    if (list) {
        return list->ofb_capacity;
    } else {
        return 0;
    }
}
