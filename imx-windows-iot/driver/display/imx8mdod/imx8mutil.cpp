/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#include "precomp.h"

UINT
AlignAllocationSize(
    UINT64 *pAllocationSize)
{
    *pAllocationSize = ((*pAllocationSize) + PAGE_SIZE - 1) & (~(PAGE_SIZE - 1));

    return PAGE_SIZE;
}

