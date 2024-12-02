/*
 * Copyright 2022 NXP
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

#include <ntddk.h>
#include <wdf.h>
#define RESHUB_USE_HELPER_ROUTINES
#include <reshub.h>
#include <acpiioct.h>
#include "acpiutil.hpp"
#include "OperatorNew.hpp"

#include "Trace.h"
#include "WdfIoTargets.tmh"

#define I2C_SYNCHRONOUS_TIMEOUT 300

template <typename T>
class list_iter_t {
	PLIST_ENTRY m_EntryPtr;
    
public:

	list_iter_t& operator++() // prefix ++
	{
		ASSERT(m_EntryPtr->Blink != NULL);
		if(m_EntryPtr->Blink != NULL) {
            m_EntryPtr = m_EntryPtr->Blink;
		}
		return *this;
	};
    
	list_iter_t operator++(int) // postfix ++
	{
		list_iter_t retval = *this;
		++(*this);
		return retval;
	}
	T* operator*() const { return CONTAINING_RECORD(m_EntryPtr, T, ListEntry); }
	bool operator!=(list_iter_t &other) const { return !(m_EntryPtr == other.m_EntryPtr); }
	explicit list_iter_t(PLIST_ENTRY EntryPtr) : m_EntryPtr(EntryPtr) {}
	
};

template <typename T>
class list_t {
	LIST_ENTRY ListHead;
	ULONG len;

public:
	ULONG DriverPoolTag;
	list_t(ULONG driverPoolTag = 'ILXM') : DriverPoolTag(driverPoolTag), len(0) { InitializeListHead(&ListHead); };

	void Append(T *Entry)
	{
		InsertHeadList(&ListHead, &(Entry->ListEntry));
		++len;
	}

	void Prepend(T *Entry)
	{
		InsertTailList(&ListHead, &(Entry->ListEntry));
		++len;
	}

	template <typename... Ts>
	NTSTATUS Emplace(Ts&&... args)
	{
		NTSTATUS status;
		T *a = static_cast<T*>(ExAllocatePoolWithTag(PagedPool, sizeof(T), DriverPoolTag));
		if (a == NULL) {
			status = STATUS_INSUFFICIENT_RESOURCES;
		}
		else {
			new(a) T(args...);
			Append(a);
			status = STATUS_SUCCESS;
		}
		return status;
	}

	T *Pop(ULONG id = 0)
	{
		T *retval = NULL;

		if (id < len) {
			auto item = at(id);
			retval = Pop(item);
		}
		return retval;
	}

	T *Pop(T *Item)
	{
		T *retval = NULL;
		if (Item != NULL) {
			PLIST_ENTRY pEntry = &Item->ListEntry;
			if (pEntry != NULL) {
				bool isOwned = false;
				for (auto a : *this) {
					if (&a->ListEntry == pEntry) {
						isOwned = true;
					}
				}
				if (isOwned) {
					if (RemoveEntryList(pEntry) > 0 && len > 1) {
						KdPrint(("Warning: list_t::pop when empty - but nonzero len.\r\n"));
					}
					--len;
					retval = CONTAINING_RECORD(pEntry, T, ListEntry);
				}
				else {
					KdPrint(("Warning: list_t::Pop called on foreign entry.\r\n"));
				}
			}
		}
		return retval;
	}
	
	T *at(ULONG id)
	{
		if (id < len) {
			auto itr = begin();
			auto eItr = end();
			for (ULONG i = 0; i < id; ++i) {
				++itr;
				ASSERT(itr != eItr);
			}
			return *itr;
		}
		else {
			return NULL;
		}
	}

	SIZE_T size()
	{
		return len;
	}

	list_iter_t<T> begin() {
		return list_iter_t<T>(ListHead.Blink);
	}

	list_iter_t<T> end() {
		return list_iter_t<T>(&ListHead);
	}
};


struct list_item_t {
	LIST_ENTRY ListEntry;

	list_item_t() { InitializeListHead(&ListEntry); }
};

struct mem_res: list_item_t {
	PHYSICAL_ADDRESS m_PhysicalAddress;
	SIZE_T m_Length;
	USHORT m_Flags;

	mem_res(): m_Length(0), m_Flags(0) {};

	mem_res(PHYSICAL_ADDRESS PhysicalAddress, SIZE_T Length, USHORT Flags = 0) : m_PhysicalAddress(PhysicalAddress), m_Length(Length), m_Flags(Flags) {};
	void set(PHYSICAL_ADDRESS PhysicalAddress, SIZE_T Length, USHORT Flags = 0) {
		NT_ASSERT(m_Length != 0 && Length != 0);

		m_PhysicalAddress = PhysicalAddress;
		m_Length = Length;
		m_Flags = Flags;
	}

};

struct int_res : list_item_t {
	ULONG m_Vector;
	USHORT m_Flags;

	int_res() : m_Vector(0) {};
	int_res(ULONG Vector, USHORT Flags = 0) : m_Vector(Vector), m_Flags(Flags) {};

	void set(ULONG Vector, USHORT Flags = 0) {
		NT_ASSERT(Vector != 0);

		m_Vector = Vector;
		m_Flags = Flags;
	}
};

struct connection_res : list_item_t {
	ULONG m_IdLowPart;
	LONG m_IdHighPart;
	USHORT m_Flags;

	connection_res(ULONG IdLowPart, LONG IdHighPart, USHORT Flags = 0) : m_IdLowPart(IdLowPart), m_IdHighPart(IdHighPart), m_Flags(Flags) {};

	void set(ULONG IdLowPart, LONG IdHighPart, USHORT Flags = 0) {
		NT_ASSERT((m_IdLowPart == 0 && m_IdHighPart == 0)
			&& ((IdLowPart != 0) || (IdHighPart != 0)));

		m_IdHighPart = IdHighPart;
		m_IdLowPart = IdLowPart;
		m_Flags = Flags;
	}
};


class io {
protected:
	const WDFDEVICE m_ParentWdfDevice;
	DECLARE_UNICODE_STRING_SIZE(m_usDevicePath, RESOURCE_HUB_PATH_SIZE);      /* Create the device path using the connection ID. */

	io(const WDFDEVICE &ParentWdfDevice) : m_ParentWdfDevice(ParentWdfDevice), m_WdfIoTarget(NULL) {};
public:
	WDFIOTARGET   m_WdfIoTarget;

	struct ctx_acpi_csr_stub
	{
		/* PNP ACPI utils */
		list_t<mem_res> m_MeResList;
		list_t<connection_res> m_GpioResList;
		list_t<connection_res> m_I2cResList;
		list_t<int_res> m_IntResList;
		SIZE_T m_MemResCnt;
		SIZE_T m_GpioResCnt;
		SIZE_T m_I2cResCnt;
		SIZE_T m_IrqResCnt;

		/* ACPI processing */
		NTSTATUS Get_CrsAcpiResources(WDFCMRESLIST ResourcesTranslated);

		/* Utils */
		ctx_acpi_csr_stub(): m_MemResCnt(0), m_GpioResCnt(0), m_I2cResCnt(0), m_IrqResCnt(0) {};
	};

	NTSTATUS IoTargetInit(const connection_res *aConnectionRes);
	void IoTargetClose()
	{
		if (m_WdfIoTarget != NULL) {
			WdfIoTargetClose(m_WdfIoTarget);
			// WdfObjectDelete(m_WdfIoTarget);
			m_WdfIoTarget = NULL;
		}
	}
	~io() { IoTargetClose(); }
};

class regBase
{
protected:
	mem_res m_MemRes;
	const WDFDEVICE m_ParentWdfDevice;
	UINT32 m_Attributes;
	PVOID m_MemoryAddr;
	SIZE_T m_NumBytes;

	regBase(const WDFDEVICE &ParentWdfDevice, UINT32 Attributes = (PAGE_READWRITE | PAGE_NOCACHE)) : m_ParentWdfDevice(ParentWdfDevice), m_MemoryAddr(NULL), m_NumBytes(0), m_Attributes(Attributes) {};

public:

	virtual void IoSpaceUnmap()
	{
		if (m_MemoryAddr != NULL) {
			MmUnmapIoSpace(m_MemoryAddr, m_MemRes.m_Length);
		}
	}

	virtual NTSTATUS IoSpaceMap()
	{
		NTSTATUS status = STATUS_SUCCESS;

		m_MemoryAddr = MmMapIoSpaceEx(m_MemRes.m_PhysicalAddress, m_MemRes.m_Length, (m_Attributes));
		if (m_MemoryAddr == NULL) {
			status = STATUS_INSUFFICIENT_RESOURCES;
		}
		return status;
	}

	virtual NTSTATUS IoSpaceMap(const mem_res &aMemRes)
	{
		NTSTATUS status = STATUS_SUCCESS;
        
		m_MemRes = aMemRes;
		status = IoSpaceMap();
		return status;
	}

	virtual void Destroy() { IoSpaceUnmap(); }
};

template <typename T>
class reg: public regBase
{
public:
	NTSTATUS IoSpaceMap()
	{
		if (m_MemRes.m_Length < sizeof(T)) {
			return STATUS_INVALID_BLOCK_LENGTH;
		}
		else {
			return regBase::IoSpaceMap();
		}
	}

	using regBase::IoSpaceMap;

	T *Reg()
	{
		return static_cast<T*>(m_MemoryAddr);
	}
	
	reg(const WDFDEVICE &ParentWdfDevice, UINT32 Attributes = (PAGE_READWRITE | PAGE_NOCACHE)) : regBase(ParentWdfDevice, Attributes) {};
};
#if 0
class AcpiDsdRes_t
{
	const WDFDEVICE m_WdfDevice;

public:
	struct _DSDVAL_GET_DESCRIPTOR {
		PCSTR ValueName;
		void *const DestinationPtr;
		const SIZE_T DestinationSize;
		const USHORT Type;

		_DSDVAL_GET_DESCRIPTOR(const char *const ValueName, UINT32 *(&&DestinationPtr), USHORT &&Type)
			:ValueName(ValueName), DestinationPtr((void*)DestinationPtr), Type(Type), DestinationSize(sizeof(UINT32)) {};

		_DSDVAL_GET_DESCRIPTOR(const char* const ValueName, UINT8* (&& DestinationPtr), USHORT&& Type)
			:ValueName(ValueName), DestinationPtr((void*)DestinationPtr), Type(Type), DestinationSize(sizeof(UINT8)) {};

		_DSDVAL_GET_DESCRIPTOR(const char *const ValueName, UINT32 *(&&DestinationPtr), SIZE_T &&DestinationSize, USHORT &&Type)
			:ValueName(ValueName), DestinationPtr((void*)DestinationPtr), DestinationSize(DestinationSize), Type(Type) {};
		
	};
	const ACPI_METHOD_ARGUMENT UNALIGNED* m_DevicePropertiesPkgPtr;

	// Init
	PACPI_EVAL_OUTPUT_BUFFER m_DsdBufferPtr;
	AcpiDsdRes_t(const WDFDEVICE &WdfDevice) : m_WdfDevice(WdfDevice) {};
	NTSTATUS LoadDsd();
	void Cleanup();
	// Load values from ACPI DSD
	NTSTATUS GetString(const CHAR* ValueName, SIZE_T DestinationSize, UINT32* Length, PCHAR DestinationPtr);
	NTSTATUS GetInteger(const CHAR* ValueName, UINT32* DestinationPtr);
	NTSTATUS GetInteger(const CHAR* ValueName, UINT8* DestinationPtr);
	NTSTATUS GetDsdResources(const _DSDVAL_GET_DESCRIPTOR ValDescriptor[], UINT16 DescriptorLen);

	NTSTATUS EvalMethodSync(const WDFDEVICE &WdfDevice, ACPI_EVAL_INPUT_BUFFER* InputBufferPtr, SIZE_T InputBufferSize, PACPI_EVAL_OUTPUT_BUFFER *OutBufferPtr);
	NTSTATUS EvalMethodSync(const WDFDEVICE &WdfDevice, ULONG MethodNameUlong, PACPI_EVAL_OUTPUT_BUFFER *OutBufferPtr)
	{
		ACPI_EVAL_INPUT_BUFFER inputBuffer;

		RtlZeroMemory(&inputBuffer, sizeof(inputBuffer));
		inputBuffer.Signature = ACPI_EVAL_INPUT_BUFFER_SIGNATURE;
		inputBuffer.MethodNameAsUlong = MethodNameUlong; //'RPGR'; // Has to be spelled backwards because of endianity
		return EvalMethodSync(WdfDevice, &inputBuffer, sizeof(inputBuffer), OutBufferPtr);
	}
};
#endif 
class gpio_t : public io {
	NTSTATUS Read(bool &) {};
public:
	NTSTATUS gpio_t::Set(_In_ UCHAR GpioValue);
	gpio_t(const WDFDEVICE &ParentWdfDevice) : io(ParentWdfDevice) {};
};

#include "spb.h"
class i2c_bus : public io {
public:
	struct CODEC_COMMAND {
		LARGE_INTEGER m_MsSleepTime;
		UCHAR m_Bytes[4];

		CODEC_COMMAND(LARGE_INTEGER MsSleepTime, UCHAR addr, UCHAR data) : m_MsSleepTime(MsSleepTime), m_Bytes{ addr, data } {};
	};
    
public:
	NTSTATUS Write(CODEC_COMMAND *CodecCommand);
	NTSTATUS WriteBytes(_In_ PVOID Buffer, _In_ ULONG Number);

	NTSTATUS ReadAddr8(_In_ UINT8 Address, PVOID pBuff, _In_ ULONG Length);
	NTSTATUS ReadAddr16(_In_ UINT16 Address, PVOID pBuff, _In_ ULONG Length);

	NTSTATUS WriteArray(CODEC_COMMAND CodecCommand[], USHORT NumCommands);
	NTSTATUS ReadArray(_In_ USHORT Address, PVOID pBuff, size_t BuffLength);

	i2c_bus(const WDFDEVICE &ParentWdfDevice) : io(ParentWdfDevice) {};
};
