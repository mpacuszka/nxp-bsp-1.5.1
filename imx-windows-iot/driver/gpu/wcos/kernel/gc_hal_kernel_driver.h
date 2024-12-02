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

#pragma once

#include "gc_hal_kernel_wcos.h"
#include "gc_hal_types.h"

extern gcsPLATFORM * gcPlatform;

void module_exit(void);

gceSTATUS device_add(PDRIVER_OBJECT pDriverObject);

void gckDRIVER_SetParams(void);

gceSTATUS gckDRIVER_DeviceAttach(PDEVICE_OBJECT DeviceObject);
gceSTATUS gckDRIVER_DeviceDetach(PDEVICE_OBJECT DeviceObject);
gceSTATUS gckDRIVER_DeviceIoctl(PDEVICE_OBJECT DeviceObject,gcsHAL_INTERFACE * iface);
gceSTATUS gckDRIVER_DeviceStart(void);
void gckDRIVER_DeviceStop(void);

NTSTATUS
DevicePnP(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    );

NTSTATUS
DeviceOpen(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    );

NTSTATUS
DeviceIoctl(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    );

VOID
DriverUnload(
    _In_ PDRIVER_OBJECT DriverObject
    );

NTSTATUS
DeviceClose(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    );

typedef struct DeviceExtension
{
    unsigned long m_magic;
//    gcsHAL_PRIVATE_DATA_PTR m_data;
} DeviceExtension;

static const unsigned long kMagic = 'magc';

#define DEVICE_PATH      L"\\Device\\GALCORE"
#define USER_VISIBLE_DEVICE_PATH     L"\\DosDevices\\GALCOREIO"

VOID
PrintIrpInfo(
    PIRP Irp
    );
VOID
PrintChars(
    _In_reads_(CountChars) PCHAR BufferAddress,
    _In_ size_t CountChars
    );

#if 0
//
// Device type           -- in the "User Defined" range."
//
#define SIOCTL_TYPE 40000
//
// The IOCTL function codes from 0x800 to 0xFFF are for customer use.
//
#define IOCTL_SIOCTL_METHOD_IN_DIRECT \
    CTL_CODE(SIOCTL_TYPE, 0x900, METHOD_IN_DIRECT, FILE_ANY_ACCESS  )

#define IOCTL_SIOCTL_METHOD_OUT_DIRECT \
    CTL_CODE(SIOCTL_TYPE, 0x901, METHOD_OUT_DIRECT, FILE_ANY_ACCESS  )

#define IOCTL_SIOCTL_METHOD_BUFFERED \
    CTL_CODE(SIOCTL_TYPE, 0x902, METHOD_BUFFERED, FILE_ANY_ACCESS  )

#define IOCTL_SIOCTL_METHOD_NEITHER \
    CTL_CODE(SIOCTL_TYPE, 0x903, METHOD_NEITHER, FILE_ANY_ACCESS  )


#define DRIVER_FUNC_INSTALL     0x01
#define DRIVER_FUNC_REMOVE      0x02

#define DRIVER_NAME       "GALCORE"
#define USER_VISIBLE_DEVICE_NAME "GALCOREIO"
#endif

