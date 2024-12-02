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

#ifndef _NEW_DELETE_OPERATORS_
#define _NEW_DELETE_OPERATORS_

/*************************************************

	Add definitions that are missing for C++14.

*************************************************/

PVOID operator new
(
	size_t          iSize,
	_When_((poolType & NonPagedPoolMustSucceed) != 0,
		__drv_reportError("Must succeed pool allocations are forbidden. "
			"Allocation failures cause a system crash"))
	POOL_TYPE       poolType
	);

PVOID operator new
(
	size_t          iSize,
	_When_((poolType & NonPagedPoolMustSucceed) != 0,
		__drv_reportError("Must succeed pool allocations are forbidden. "
			"Allocation failures cause a system crash"))
	POOL_TYPE       poolType,
	ULONG           tag
	);

/*++

Routine Description:

	Array new() operator for creating objects with a specified allocation tag.

Arguments:

	iSize -
		The size of the entire allocation.

	poolType -
		The type of allocation.  Ex: PagedPool or NonPagedPoolNx

	tag -
		A 4-byte allocation identifier.

Return Value:

	None

--*/
PVOID
operator new[](
	size_t          iSize,
	_When_((poolType & NonPagedPoolMustSucceed) != 0,
		__drv_reportError("Must succeed pool allocations are forbidden. "
			"Allocation failures cause a system crash"))
	POOL_TYPE       poolType,
	ULONG           tag
	);

/*++

Routine Description:

	Array delete() operator.

Arguments:

	pVoid -
		The memory to free.

Return Value:

	None

--*/
void
__cdecl
operator delete[](
	PVOID pVoid
	);

/*++

Routine Description:

	Sized delete() operator.

Arguments:

	pVoid -
		The memory to free.

	size -
		The size of the memory to free.

Return Value:

	None

--*/
void __cdecl operator delete
(
	void *pVoid,
	size_t /*size*/
	);

/*++

Routine Description:

	Sized delete[]() operator.

Arguments:

	pVoid -
		The memory to free.

	size -
		The size of the memory to free.

Return Value:

	None

--*/
void __cdecl operator delete[]
(
	void *pVoid,
	size_t /*size*/
	);


//
// Placement new and delete operators
//

_Use_decl_annotations_
void* operator new (size_t, void* Ptr);

void operator delete (void*, void*);

_Use_decl_annotations_
void* operator new[](size_t, void* Ptr);

void operator delete[](void*, void*);

#endif // _NEW_DELETE_OPERATORS_
