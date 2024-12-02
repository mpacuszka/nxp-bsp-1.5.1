/****************************************************************************
* Copyright (c) Microsoft Corporation.
*
*    Licensed under the MIT License.
*    Licensed under the GPL License.
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

#include "GcKmd7L.h"
#include "GcKmd7LMmu.h"
#include "GcKmdProcess.h"

class GcKm7LProcess : public GcKmProcess
{
public:

    friend class GcKm7LMmu;

    GcKm7LProcess(
        GcKm7LAdapter          *pGcKm7LAdapter,
        DXGKARG_CREATEPROCESS  *pCreateProcess) :
        GcKmProcess((GcKmAdapter *)pGcKm7LAdapter, pCreateProcess)
    {
        m_pPageTable  = NULL;
        m_GlobalGpuVa = 0;
        m_pPrivateSTlb = NULL;
        m_pPrivateSTlbMdl = NULL;
        m_pPrivateSTlbPfn = NULL;

        memset(m_CachedMTlb, 0, sizeof(m_CachedMTlb));
        m_b1MBRangeUpdated = false;
    }

    virtual ~GcKm7LProcess();

    NTSTATUS
    Initialize(
        UINT    GlobalGpuVaAlignment,
        UINT    GlobalGpuVaBaseAddress,
        UINT    GlobalGpuVaSizeInBytes);

    VOID
    UpdateSTlbPageSize(
        UINT                MTlbEntryIndex,
        Gc7LSTlbPageSize    STlbPageSize,
        BOOLEAN             bReadOnly = false)
    {
        if (STlbPageSize1M == m_CachedMTlb[MTlbEntryIndex].STlbPageSize)
        {
            Gc7LMTlbEntry1M* pMTlbEntry1M = (Gc7LMTlbEntry1M*)(m_CachedMTlb + MTlbEntryIndex);

            pMTlbEntry1M->Zeroed = true;    // Use Zeroed field as dirty flag

            m_b1MBRangeUpdated = true;
        }

        m_CachedMTlb[MTlbEntryIndex].STlbPageSize = STlbPageSize;

        if (STlbPageSize16M == STlbPageSize)
        {
            Gc7LMTlbEntry1M    *pMTlbEntry1M = (Gc7LMTlbEntry1M *)(m_CachedMTlb + MTlbEntryIndex);

            pMTlbEntry1M->Zeroed = bReadOnly;
        }

        if (STlbPageSize1M == STlbPageSize)
        {
            Gc7LMTlbEntry1M* pMTlbEntry1M = (Gc7LMTlbEntry1M*)(m_CachedMTlb + MTlbEntryIndex);

            pMTlbEntry1M->Zeroed = true;    // Use Zeroed field as dirty flag

            m_b1MBRangeUpdated = true;
        }
    }

    VOID
    UpdateMTlbEntries(
        Gc7LMTlbEntry4K    *pMTlbEntry,
        UINT                StartIndex,
        UINT                NumEntries,
        UINT                DummyMTlbEntry)
    {
        UINT    i;

        NT_ASSERT((StartIndex + NumEntries) <= (1 << GC_7L_MTLB_INDEX_BITS));

        for (i = StartIndex; i < (StartIndex + NumEntries); i++)
        {
            if (pMTlbEntry[i].Present && (pMTlbEntry[i].Value != DummyMTlbEntry))
            {
                pMTlbEntry[i].STlbPageSize = m_CachedMTlb[i].STlbPageSize;
            }
            else
            {
                m_CachedMTlb[i].STlbPageSize = STlbPageSize4K;
            }
        }

        for (i = StartIndex; i < (StartIndex + NumEntries); i++)
        {
            if (m_CachedMTlb[i].STlbPageSize > STlbPageSize64K)
            {
                Gc7LMTlbEntry1M    *pMTlbEntry1M = (Gc7LMTlbEntry1M *)&m_CachedMTlb[i];
                UINT                Zeroed = pMTlbEntry1M->Zeroed;

                m_CachedMTlb[i].Value = pMTlbEntry[i].Value;

                pMTlbEntry1M->Zeroed = Zeroed;
            }
            else
            {
                m_CachedMTlb[i].Value = pMTlbEntry[i].Value;
            }
        }
    }

    Gc7LSTlbEntry1M*
    GetPrivateSTlb(
        UINT    MTlbIndex)
    {
        return m_pPrivateSTlb + (MTlbIndex * 16);
    }

    BOOLEAN
    Get1MBRangeUpdated(
        Gc7LMTlbEntry4K   **ppCachedMTlb,
        Gc7LSTlbEntry1M   **ppPrivateSTlb)
    {
        *ppCachedMTlb   = m_CachedMTlb;
        *ppPrivateSTlb = m_pPrivateSTlb;

        return m_b1MBRangeUpdated;
    }

    ULONG64
    GetPrivateSTlbPhysAddr(Gc7LSTlbEntry1M* pVirtualAddress)
    {
        ULONG64 OffsetFromStartInBytes = (pVirtualAddress - m_pPrivateSTlb) * sizeof(Gc7LSTlbEntry1M);
        ULONG64 ArrayIndex = OffsetFromStartInBytes / PAGE_SIZE;
        ULONG64 OffsetInArray = OffsetFromStartInBytes % PAGE_SIZE;

        NT_ASSERT(ArrayIndex < (m_PrivateSTlbSize / PAGE_SIZE));

        return (m_pPrivateSTlbPfn[ArrayIndex] << 12) + OffsetInArray;
    }

    void
    Clear1MBRangeUpdated()
    {
        m_b1MBRangeUpdated = false;
    }

protected:

    void   *m_pPageTable;
    UINT32  m_GlobalGpuVa;

    Gc7LMTlbEntry4K m_CachedMTlb[1 << GC_7L_MTLB_INDEX_BITS];
    BOOLEAN m_b1MBRangeUpdated;

    const UINT m_PrivateSTlbSize = 4 * PAGE_SIZE;
    MDL* m_pPrivateSTlbMdl;
    PPFN_NUMBER m_pPrivateSTlbPfn;
    Gc7LSTlbEntry1M* m_pPrivateSTlb;

private:

    NTSTATUS InitGlobalGpuVaShadow();
    NTSTATUS AllocatePrivateSTlb();
};

CONST UINT  GC_7L_GPU_VA_SIZE_IN_4K_PAGES = 8 << GC_7L_STLB_INDEX_BITS; // By default 8*4096 4K pages or 128 MB


