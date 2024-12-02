/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#pragma warning(disable : 4201)

#include <wingdi.h>
#include <d3d10umddi.h>

typedef struct _CommandBufferChunk
{
    D3DGPU_VIRTUAL_ADDRESS  m_commandBufferChunkGpuVa;
    UINT                    m_commandBufferChunkSize;
} CommandBufferChunk;

//
// There is at least 1 command buffer chunk
//

typedef struct _SubmissionExchangeData
{
    UINT                m_numCommandBufferChunks;
    CommandBufferChunk  m_commandBufferChunks[1];
} SubmissionExchangeData;

