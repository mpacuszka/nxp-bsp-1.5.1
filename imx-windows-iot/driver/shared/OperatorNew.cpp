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

extern "C" {
#include <wdm.h>
}

#include "OperatorNew.hpp"


PVOID operator new
(
    size_t          iSize,
    _When_((poolType & NonPagedPoolMustSucceed) != 0,
       __drv_reportError("Must succeed pool allocations are forbidden. "
             "Allocation failures cause a system crash"))
    POOL_TYPE       poolType
)
/*!
 * Operator new implemented as ExAllocatePoolWithTag, tagged as 'wNCK'. 
 *
 * @param iSize required memory size to allocate.
 * @param poolType from which to allocate memory eg. NonPagedPool.
 *
 * @returns pointer to allocated memory or NULL.
 */
{
    PVOID result = ExAllocatePoolWithTag(poolType, iSize, 'wNCK');

    if (result) {
        RtlZeroMemory(result, iSize);
    }

    return result;
}

PVOID operator new
(
    size_t          iSize,
    _When_((poolType & NonPagedPoolMustSucceed) != 0,
       __drv_reportError("Must succeed pool allocations are forbidden. "
             "Allocation failures cause a system crash"))
    POOL_TYPE       poolType,
    ULONG           tag
)
/*!
 * Operator new implemented as ExAllocatePoolWithTag.
 *
 * @param iSize required memory size to allocate.
 * @param poolType from which to allocate memory eg. NonPagedPool.
 * @param tag user specified four character (32b) tag for debugging purposes. Usually spelled backwards.
 *
 * @returns pointer to allocated memory or NULL.
 */
{
    PVOID result = ExAllocatePoolWithTag(poolType, iSize, tag);

    if (result) {
        RtlZeroMemory(result,iSize);
    }

    return result;
}

PVOID 
operator new[](
    size_t          iSize,
    _When_((poolType & NonPagedPoolMustSucceed) != 0,
        __drv_reportError("Must succeed pool allocations are forbidden. "
            "Allocation failures cause a system crash"))
    POOL_TYPE       poolType,
    ULONG           tag
)
/*!
 * Array new operator implemented as ExFreePool.
 *
 * @param iSize required memory size to allocate.
 * @param poolType from which to allocate memory eg. NonPagedPool.
 * @param tag user specified four character (32b) tag for debugging purposes. Usually spelled backwards.
 *
 * @returns pointer to allocated memory or NULL.
 */
{
    PVOID result = ExAllocatePoolWithTag(poolType, iSize, tag);

    if (result)
    {
        RtlZeroMemory(result, iSize);
    }

    return result;
}

void 
__cdecl 
operator delete[](
    PVOID pVoid
)
/*!
 * Array delete operator implemented as ExFreePool.
 *
 * @param pVoid the memory to free.
 */
{
    if (pVoid)
    {
        ExFreePool(pVoid);
    }
}

void __cdecl operator delete
(
    void *pVoid,
    size_t /*size*/
)
/*!
 * Sized delete() operator implemented as ExFreePool.
 *
 * @param pVoid the memory to free.
 * @param size the size of the memory to free.
 */
{
    if (pVoid)
    {
        ExFreePool(pVoid);
    }
}

void __cdecl operator delete[]
(
    void *pVoid,
    size_t /*size*/
)
/*!
 * Sized array delete operator implemented as ExFreePool.
 *
 * @param pVoid the memory to free.
 * @param size the size of the memory to free.
 */
{
    if (pVoid)
    {
        ExFreePool(pVoid);
    }
}

//
// Placement new and delete operators
//

_Use_decl_annotations_
void* operator new (size_t, void* Ptr) throw ()
/*!
 * Placement new operator.
 *
 * @param Size required memory size to allocate.
 * @param Ptr pointer to allocated memory.
 *
 * @returns pointer to allocated memory or NULL.
 */
{
    return Ptr;
}

void operator delete (void*, void*) throw ()
/*!
 * Placement delete operator.
 */
{}

_Use_decl_annotations_
void* operator new[](size_t, void* Ptr) throw ()
/*!
 * Placement array new operator.
 *
 * @param Size required memory size to allocate.
 * @param Ptr pointer to allocated memory.
 *
 * @returns pointer to allocated memory or NULL.
 */
{
    return Ptr;
}

void operator delete[](void*, void*) throw ()
/*!
 * Palcement array delete operator.
 */
{}