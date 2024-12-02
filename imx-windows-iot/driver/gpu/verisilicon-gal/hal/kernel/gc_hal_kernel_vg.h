/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2022 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2022 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

/*
 * $RCSfile: gc_hal_kernel_vg.h,v $
 *
 * Internal objects known only at the inside of the kernel HAL layer.
 *
 * Written by Frido Garritsen
 */

#ifndef __gc_hal_kernel_vg_h_
#define __gc_hal_kernel_vg_h_

#include "gc_hal.h"
#include "gc_hal_driver.h"
#include "gc_hal_kernel_hardware.h"

/******************************************************************************\
********************************** Structures **********************************
\******************************************************************************/


/* gckKERNEL object. */
struct _gckVGKERNEL
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Pointer to gckHARDWARE object. */
    gckVGHARDWARE                   hardware;

    /* Pointer to gckINTERRUPT object. */
    gckVGINTERRUPT              interrupt;

    /* Pointer to gckCOMMAND object. */
    gckVGCOMMAND                    command;

    /* Pointer to context. */
    gctPOINTER                  context;

    /* Pointer to gckMMU object. */
    gckVGMMU                        mmu;

    gckKERNEL                   kernel;
};

#if 0
/* gcuVIDMEM_NODE structure. */
typedef union _gcuVIDMEM_NODE
{
    /* Allocated from gckVIDMEM. */
    struct _gcsVIDMEM_NODE_VIDMEM
    {
        /* Owner of this node. */
        gckVIDMEM               memory;

        /* Dual-linked list of nodes. */
        gcuVIDMEM_NODE_PTR      next;
        gcuVIDMEM_NODE_PTR      prev;

        /* Dual linked list of free nodes. */
        gcuVIDMEM_NODE_PTR      nextFree;
        gcuVIDMEM_NODE_PTR      prevFree;

        /* Information for this node. */
        gctUINT32               address;
        gctSIZE_T               bytes;
        gctUINT32               alignment;

        /* Locked counter. */
        gctINT32                locked;
    }
    VidMem;

    /* Allocated from gckOS. */
    struct _AQVIDMEM_NODE_VIRTUAL
    {
        /* Pointer to gckKERNEL object. */
        gckKERNEL               kernel;

        /* Information for this node. */
        gctPHYS_ADDR            physical;
        gctSIZE_T               bytes;
        gctPOINTER              logical;

        /* Page table information. */
        gctSIZE_T               pageCount;
        gctPOINTER              pageTable;
        gctUINT32               address;

        /* Mutex. */
        gctPOINTER              mutex;

        /* Locked counter. */
        gctINT32                locked;
    }
    Virtual;
}
gcuVIDMEM_NODE;

/* gckVIDMEM object. */
struct _gckVIDMEM
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Information for this video memory heap. */
    gctSIZE_T                   baseAddress;
    gctSIZE_T                   bytes;
    gctSIZE_T                   freeBytes;

    /* Mapping for each type of surface. */
    gctINT                      mapping[gcvVIDMEM_TYPE_COUNT];

    /* Sentinel nodes for up to 8 banks. */
    gcuVIDMEM_NODE              sentinel[gcvVIDMEM_TYPE_COUNT];

    /* Allocation threshold. */
    gctSIZE_T                   threshold;

    /* The heap mutex. */
    gctPOINTER                  mutex;
};
#endif
/* gckMMU object. */
struct _gckVGMMU
{
    /* The object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Pointer to gckHARDWARE object. */
    gckVGHARDWARE                   hardware;

    /* The page table mutex. */
    gctPOINTER                  mutex;

    /* Page table information. */
    gctSIZE_T                   pageTableSize;
    gctPHYS_ADDR                pageTablePhysical;
    gctPOINTER                  pageTableLogical;

    /* Allocation index. */
    gctUINT32                   entryCount;
    gctUINT32                   entry;
};

#endif /* __gc_hal_kernel_h_ */
