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
#include "circ_queue.h"
#include <wdm.h>

/*++
   Routine Description:
    It is called to add a new entry to the beginning of the queue.
   Arguments:
    pQueue      The target queue address.
    pListEntry  The address of the new item to be added.
   Return Value:
    None
   --*/
_Use_decl_annotations_
void CircQueuePush(PCIRCULAR_BUFFER pQueue, PQUEUE_ITEM item)
{
    if (pQueue->count == pQueue->capacity) {
        /* Buffer is full. The oldest item in the buffer is lost! */
        NT_ASSERTMSG("Circular buffer FrameQueue is full", FALSE);
        pQueue->tail = (char *)pQueue->tail + pQueue->sz;

        /* wrap circiut */
        if (pQueue->tail == pQueue->buffer_end) {
            pQueue->tail = pQueue->buffer;
        }
        RtlCopyMemory(pQueue->head, item, pQueue->sz);
        pQueue->head = (char *)pQueue->head + pQueue->sz;
        if (pQueue->head == pQueue->buffer_end) {
            pQueue->head = pQueue->buffer;
        }
        pQueue->overflow_counter++ /* Increment counter of lost events */;
    } else {
        /* Writes to the buffer in the standard way. */
        RtlCopyMemory(pQueue->head, item, pQueue->sz);
        pQueue->head = (char *)pQueue->head + pQueue->sz;
        if (pQueue->head == pQueue->buffer_end) {
            pQueue->head = pQueue->buffer;
        }
        pQueue->count++;
    }
}

/*++
   Routine Description:
    It is called to POP a new entry to the beginning of the queue.
   Arguments:
    pQueue      The target queue address.
    pListEntry  The address of the new item to be added.
   Return Value:
    None
   --*/
_Use_decl_annotations_
int CircQueuePop(PCIRCULAR_BUFFER pQueue, PQUEUE_ITEM item)
{
    if (pQueue->count == 0) {
        NT_ASSERTMSG("cb handle error", FALSE);
        return -1;
    }

    RtlCopyMemory(item, pQueue->tail, pQueue->sz);
    pQueue->tail = (char *)pQueue->tail + pQueue->sz;
    if (pQueue->tail == pQueue->buffer_end) {
        /* wrap circuilar buffer */
        pQueue->tail = pQueue->buffer;
    }
    pQueue->count--;
    return 0;
}