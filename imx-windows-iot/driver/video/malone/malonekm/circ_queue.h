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
#include <ntdef.h>

/** Notification structure */
typedef struct _QUEUE_ITEM {
    UINT32 msg;
} QUEUE_ITEM, *PQUEUE_ITEM;

typedef struct _CIRCULAR_BUFFER {
    void *buffer;     /* data buffer */
    void *buffer_end; /* end of data buffer */
    size_t capacity;  /* maximum number of items in the buffer */
    size_t count;     /* number of items in the buffer */
    size_t sz;        /* size of each item in the buffer */
    size_t overflow_counter; /* over flow counter */
    void *head;       /* pointer to head */
    void *tail;       /* pointer to tail */
} CIRCULAR_BUFFER, *PCIRCULAR_BUFFER;

void CircQueuePush(_In_ PCIRCULAR_BUFFER QueuePtr, PQUEUE_ITEM item);
int CircQueuePop(_In_ PCIRCULAR_BUFFER QueuePtr, PQUEUE_ITEM item);
