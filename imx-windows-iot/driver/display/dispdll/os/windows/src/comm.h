/*
 * Copyright 2022-2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __COMM_H__
#define __COMM_H__

#if defined(__cplusplus)
extern "C"
{
#endif

struct iotarget_handles {
    HANDLE m_iotarget_handle;
    PFILE_OBJECT m_iotarget_file_object_ptr;
    PDEVICE_OBJECT m_iotarget_device_object_ptr;
    KEVENT m_event;
    UCHAR* base;
};

typedef struct iotarget_handles iotarget_handles;

NTSTATUS comm_initialize(LARGE_INTEGER connection_id, iotarget_handles *tgt, ACCESS_MASK desired_access);
NTSTATUS i2c_write(iotarget_handles *i2c_tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length);
NTSTATUS i2c_read(iotarget_handles *i2c_tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length);
NTSTATUS gpio_write(iotarget_handles *gpio_tgt, ULONG value);
void comm_close(iotarget_handles *tgt);
void comm_clear_handle(iotarget_handles *tgt);

#if defined(__cplusplus)
} // extern "C"
#endif

#endif /* __COMM_H__ */
