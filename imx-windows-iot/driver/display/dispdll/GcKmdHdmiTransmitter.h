/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. THE SOFTWARE IS PROVIDED "AS IS",
 * WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 */

#pragma once

#include <ntddk.h>
#include "GcKmdBaseTransmitter.h"
#include "GcKmd7LIO.h"

extern "C" {
#include "drm/bridge/cdns-mhdp.h"
}

class HdmiTransmitter : public BaseTransmitter
{
public:

    NTSTATUS Start()
    {
        PHYSICAL_ADDRESS physical;
        physical.QuadPart = 0x32c00000;

        m_HdmiBase = reinterpret_cast<BYTE*>(
            MmMapIoSpaceEx(physical, 0x40000, PAGE_READWRITE | PAGE_NOCACHE)
        );
        if (m_HdmiBase == NULL)
        {
            return STATUS_INTERNAL_ERROR;
        }

        if (!CheckAlive())
        {
            return STATUS_INTERNAL_ERROR;
        }

        if (!TestEcho())
        {
            return STATUS_INTERNAL_ERROR;
        }

        return STATUS_SUCCESS;
    }

    BOOLEAN CheckAlive(void);

    BOOLEAN TestEcho(void)
    {
        UINT8 echo_msg[] = "hello world";
        UINT8 const num_bytes = sizeof(echo_msg);
        UINT8 echo_resp[num_bytes];

        if (EchoMessage(echo_msg, echo_resp, num_bytes))
        {
            return FALSE;
        }

        return memcmp(echo_msg, echo_msg, num_bytes) == 0;
    }

    NTSTATUS EchoMessage(UINT8 const* Data, UINT8* Response, int NumBytes);

    int GetCachedEdid(PVOID *pEdid)
    {
        *pEdid = m_CachedEdid;

        return sizeof(m_CachedEdid);
    }

    virtual NTSTATUS GetHotPlugDetectStatus(UINT8* status) override
    {
        /* Hot-plug detect is not supported, commented code below is needed to read status */
        /*
        auto ret = MakeMessage(MB_MODULE_ID_GENERAL, GENERAL_GET_HPD_STATE);
        if (ret != STATUS_SUCCESS)
        {
            return ret;
        }

        WriteMsgToMbox();
        ReadMsgFromMbox();

        ret = ParseMessage(status);
        */
        /* Status is always connected */
        *status = 1;
        return STATUS_SUCCESS;
    }

    virtual NTSTATUS GetEdid(PVOID Data, ULONG Length, UINT8 Block, UINT8 Segment) override;

    virtual void GetChildDescriptor(DXGK_CHILD_DESCRIPTOR* pDescriptor) override;

private:

    enum GENERAL_SIZES
    {
        GENERAL_TEST_ECHO_MIN_PAYLOAD = 1,
        CMD_HEAD_SIZE = 4,
        GENERAL_TEST_ECHO_MAX_PAYLOAD = 100
    };

#if 0
    enum GENERAL_COMMANDS
    {
        GENERAL_TEST_ECHO = 0x02,
        GENERAL_GET_HPD_STATE = 0x11
    };

    enum HDMI_TX_OPCODE
    {
        HDMI_TX_READ,
        HDMI_TX_WRITE,
        HDMI_TX_UPDATE_READ,
        HDMI_TX_EDID,
        HDMI_TX_EVENTS,
        HDMI_TX_HPD_STATUS,
        HDMI_TX_DEBUG_ECHO = 0xAA,
        HDMI_TX_TEST = 0xBB,
        HDMI_TX_EDID_INTERNAL = 0xF0,
    };
#endif
    struct MailboxState
    {
        static int const BUF_SIZE = 1024;

        MailboxState()
            : TxBuffer{}
            , RxBuffer{}
        {
            ExInitializeFastMutex(&Mutex);
        }

        UINT8 TxBuffer[BUF_SIZE];
        UINT8 RxBuffer[BUF_SIZE];

        alignas(8) FAST_MUTEX Mutex;
    };

    template <typename... Args>
    NTSTATUS MakeMessage(UINT8 Module, UINT8 Opcode, Args... MessageFields)
    {
        UINT8* end_of_msg = nullptr;
        auto tx_buffer = m_MailboxState.TxBuffer;

        auto status = RecursivelyMakeMessage(&end_of_msg, tx_buffer + CMD_HEAD_SIZE, MessageFields...);
        if (status != STATUS_SUCCESS)
        {
            return status;
        }

        auto lenght = static_cast<UINT16>(end_of_msg - CMD_HEAD_SIZE - tx_buffer);

        tx_buffer[0] = Opcode;
        tx_buffer[1] = Module;
        tx_buffer[2] = static_cast<UINT8>(lenght >> 8);
        tx_buffer[3] = static_cast<UINT8>(lenght);

        return STATUS_SUCCESS;
    }

    template <typename... Args>
    NTSTATUS ParseMessage(Args... MessageFields)
    {
        return RecursivelyParseMessage(m_MailboxState.RxBuffer + CMD_HEAD_SIZE, MessageFields...);
    }

    BOOLEAN SameTxRxHeader()
    {
        return m_MailboxState.TxBuffer[0] == m_MailboxState.RxBuffer[0]
            && m_MailboxState.TxBuffer[1] == m_MailboxState.RxBuffer[1];
    }

    BOOLEAN ValidMsgHeader(const UINT8* Message, UINT8 Module, UINT8 Opcode)
    {
        return Message[0] == Opcode && Message[1] == Module;
    }

    NTSTATUS RecursivelyMakeMessage(UINT8** EndOfMessage, UINT8* Message)
    {
        // Base case for variadic template.
        // No more fields to add to message.

        // Return pointer as end of message marker.
        *EndOfMessage = Message;
        return STATUS_SUCCESS;
    }

    // Add an integer type to Message.
    template <typename IntType, typename... Args>
    NTSTATUS RecursivelyMakeMessage(UINT8** EndOfMessage, UINT8* Message, IntType Field, Args... OtherFields)
    {
        if (!InBounds(m_MailboxState.TxBuffer, m_MailboxState.BUF_SIZE, Message, sizeof(Field)))
        {
            return STATUS_INTERNAL_ERROR;
        }

        *reinterpret_cast<IntType*>(Message) = SwapEndianness(Field);
        return RecursivelyMakeMessage(EndOfMessage, Message + sizeof(Field), OtherFields...);
    }

    // Add a buffer to Message.
    template <typename SizeType, typename... Args>
    NTSTATUS RecursivelyMakeMessage(UINT8** EndOfMessage, UINT8* Message, UINT8* BufferField, SizeType BufferSize, Args... OtherFields)
    {
        // Forward to const specialization.
        return RecursivelyMakeMessage(EndOfMessage, Message, const_cast<UINT8 const*>(BufferField), BufferSize, OtherFields...);
    }

    // Add a buffer to Message.
    template <typename SizeType, typename... Args>
    NTSTATUS RecursivelyMakeMessage(UINT8** EndOfMessage, UINT8* Message, UINT8 const* BufferField, SizeType BufferSize, Args... OtherFields)
    {
        if (!InBounds(m_MailboxState.TxBuffer, m_MailboxState.BUF_SIZE, Message, BufferSize))
        {
            return STATUS_INTERNAL_ERROR;
        }

        for (int i = 0; i < BufferSize; i++)
        {
            Message[i] = BufferField[i];
        }
        return RecursivelyMakeMessage(EndOfMessage, Message + BufferSize, OtherFields...);
    }

    NTSTATUS RecursivelyParseMessage(UINT8* /* Message */)
    {
        // Base case for variadic template.
        // No more fields to read from message.

        return STATUS_SUCCESS;
    }

    // Read an integer type from Message.
    template <typename IntType, typename... Args>
    NTSTATUS RecursivelyParseMessage(UINT8* Message, IntType* Field, Args*... OtherFields)
    {
        if (!InBounds(m_MailboxState.RxBuffer, m_MailboxState.BUF_SIZE, Message, sizeof(*Field)))
        {
            return STATUS_INTERNAL_ERROR;
        }

        *Field = SwapEndianness(*reinterpret_cast<IntType*>(Message));
        return RecursivelyParseMessage(Message + sizeof(*Field), OtherFields...);
    }

    // Read a buffer of specified length from Message.
    template <typename... Args>
    NTSTATUS RecursivelyParseMessage(UINT8* Message, int BufferSize, UINT8* BufferField, Args*... OtherFields)
    {
        for (int i = 0; i < BufferSize; i++)
        {
            BufferField[i] = Message[i];
        }
        return RecursivelyParseMessage(Message + BufferSize, OtherFields...);
    }

    // Get a pointer to a buffer of unspecified length from Message.
    template <typename... Args>
    NTSTATUS RecursivelyParseMessage(UINT8* Message, UINT8** BufferField, Args... /* OtherFields */)
    {
        static_assert(sizeof...(Args) == 0, "An unbounded buffer should be the last field in message.");

        if (!InBounds(m_MailboxState.RxBuffer, m_MailboxState.BUF_SIZE, Message, 1))
        {
            return STATUS_INTERNAL_ERROR;
        }

        *BufferField = Message;
        return STATUS_SUCCESS;
    }

    BOOLEAN InBounds(UINT8* MemoryAreaStart, int MemoryAreaSize, UINT8* RangeStart, int RangeSize)
    {
        return MemoryAreaStart <= RangeStart && RangeStart + RangeSize <= MemoryAreaStart + MemoryAreaSize;
    }

    template <typename IntType>
    IntType SwapEndianness(IntType value)
    {
        union
        {
            IntType value;
            UINT8 u8[sizeof(IntType)];
        } source, dest;

        source.value = value;

        for (int i = 0, size = sizeof(IntType); i < size; i++)
        {
            dest.u8[i] = source.u8[size - 1 - i];
        }

        return dest.value;
    }

    /////// Mailbox operations  ////////

    void WriteMsgToMbox();

    void ReadMsgFromMbox();

    BOOLEAN MailboxRead(volatile UINT8* value);

    BOOLEAN MailboxWrite(UINT8 value);

    /////// Advanced Peripheral Bus API  ////////
#if 0
    enum APBRegisterOffset : unsigned int
    {
        MAILBOX_FULL_ADDR = 0x8,
        MAILBOX_EMPTY_ADDR = 0xC,
        MAILBOX0_WR_DATA = 0x10,
        MAILBOX0_RD_DATA = 0x14,
        KEEP_ALIVE = 0x18
    };
#endif
    void AdvancedPeripheralBusRead(UINT32 offset, UINT32* value)
    {
        *value = ReadDoubleWord(m_HdmiBase + offset);
    }


    void AdvancedPeripheralBusWrite(unsigned int offset, UINT32 value)
    {
        WriteDoubleWord(m_HdmiBase + offset, value);
    }

    MailboxState m_MailboxState;
    BYTE* m_HdmiBase{0};

    BYTE m_CachedEdid[128] = {0};
};
