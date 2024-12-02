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

#ifndef __gc_hal_kernel_wcos_h_
#define __gc_hal_kernel_wcos_h_

#include <ntddk.h>
#include <ntstrsafe.h>

typedef UINT32  gctPID;
typedef UINT32  gctTID;

typedef PVOID   gctMDL;

// PHYSICAL_ADDRESS nonsense

inline PHYSICAL_ADDRESS toPHYSICAL_ADDRESS(unsigned long long address)
{
    return *((PHYSICAL_ADDRESS *)&address);
}

// system info

inline unsigned long long totalram_pages()
{
    // TODO: getting total physical memory size is not easy on windows
    //       this call really shouldn't be needed on windows
    return (0x100000000 / PAGE_SIZE);
}

// debug support

inline void dump_stack(void) {
    __debugbreak();
}

// io register support
inline unsigned long readl(void * addr)
{
    return *((unsigned long *) addr);
//    return READ_PORT_ULONG((unsigned long *)addr);
}

inline void writel(unsigned long data, void * addr) {
    *((unsigned long *) addr) = data;
}

//  page support https://linux-kernel-labs.github.io/master/labs/memory_mapping.html
#define GFP_KERNEL 0x01
#define __GFP_NOWARN 0x10
#define __GFP_DMA32 0x20
#define __GFP_HIGHMEM  0x40
#define __GFP_NORETRY 0x80

typedef int gfp_t;

typedef struct {
    void * m_system;
} page_t;

inline page_t * alloc_page(gfp_t flags) {
    flags;
    page_t * page = (page_t *)ExAllocatePool(NonPagedPool, sizeof(page_t));
    page->m_system = ExAllocatePool(NonPagedPool, 4096);
    return page;
}

inline void SetPageReserved(page_t * page) {
    page;
    // do nothing - we allocate pages non paged
}

inline void ClearPageReserved(page_t * page) {
    page;
    // do nothing - we allocate pages non paged
}

inline void __free_page(page_t * page) {
    ExFreePool(page->m_system);
    ExFreePool(page);
}

typedef void * wait_queue_head_t;
typedef void * delayed_work_t;
typedef KSEMAPHORE semaphore_t;

// mutex support
typedef struct {
    int m_magic;
    KMUTEX m_mutex;
} mutex_t;

inline void mutex_init(mutex_t * m) {
    m->m_magic = 'lock';
    KeInitializeMutex(&m->m_mutex, 0);
}

inline void mutex_lock(mutex_t * m) {
    if(m->m_magic != 'lock') mutex_init(m);
    NTSTATUS status = KeWaitForSingleObject(&m->m_mutex, Executive, KernelMode, FALSE, NULL);
    if (status != STATUS_SUCCESS)
        __debugbreak();
}

inline int mutex_trylock(mutex_t * m) {
    if(m->m_magic != 'lock') mutex_init(m);
    LARGE_INTEGER timeout = {0};
    NTSTATUS status = KeWaitForSingleObject(&m->m_mutex, Executive, KernelMode, FALSE, &timeout);
    return (status == STATUS_SUCCESS);
}

inline void mutex_unlock(mutex_t * m) {
    KeReleaseMutex(&m->m_mutex, FALSE);
}

inline void mutex_destroy(mutex_t * m) {
    m->m_magic = 0;
}

// irq support

inline int in_irq(void) {
    return (KeGetCurrentIrql() >= DISPATCH_LEVEL);
}

// spinlock support
typedef struct {
    unsigned long m_magic;
    KSPIN_LOCK m_lock;
    KIRQL m_irql;
} spinlock_t;

inline void spin_lock_init(spinlock_t * s) {
    s->m_magic = 'SPIN';
    KeInitializeSpinLock(&s->m_lock);
}

inline void spin_lock(spinlock_t * s) {
    if (s->m_magic != 'SPIN') __debugbreak();
    KeAcquireSpinLock(&s->m_lock, &s->m_irql);
}

inline void spin_unlock(spinlock_t * s) {
    KeReleaseSpinLock(&s->m_lock, s->m_irql);
}

inline void spin_lock_at_dpc(spinlock_t * s) {
    if (s->m_magic != 'SPIN') __debugbreak();
    KeAcquireSpinLockAtDpcLevel(&s->m_lock);
}

inline void spin_unlock_from_dpc(spinlock_t * s) {
    KeReleaseSpinLockFromDpcLevel(&s->m_lock);
}

// work queue support
#define kMaxNameSize 32
typedef struct {
    char m_name[kMaxNameSize];
} workqueue_t;

inline workqueue_t * create_singlethread_workqueue(const char * name) {
    // TODO: Can we use paged memory?
    workqueue_t * wq = (workqueue_t *) ExAllocatePool(NonPagedPool, sizeof(workqueue_t));
    RtlStringCbCopyA(wq->m_name, sizeof(wq->m_name), name);
    return wq;
}

inline void destroy_workqueue(workqueue_t * wq)
{
    ExFreePool(wq);
}

inline void flush_workqueue(workqueue_t * wq)
{
    wq;
    // do nothing for now
}


struct delayed_work {
    int dummy;
};

// Semaphore support
// See: https://docs.huihoo.com/doxygen/linux/kernel/3.7/semaphore_8h.html
typedef KSEMAPHORE semaphore_t;

inline static void up(semaphore_t * s) {
    KeReleaseSemaphore(s, 0, 1, FALSE);
}

inline static void down_interruptible(semaphore_t *s) {
    KeWaitForSingleObject(s, Executive, KernelMode, FALSE, NULL);
}

inline static void down(semaphore_t *s) {
    KeWaitForSingleObject(s, Executive, KernelMode, FALSE, NULL);
}

inline static int down_trylock(semaphore_t *s) {
    LARGE_INTEGER timeout = { 0 };
    return (KeWaitForSingleObject(s, Executive, KernelMode, TRUE, &timeout) != STATUS_SUCCESS);
}

/*
 * TODO: Linux sem_init uses SEM_VALUE_MAX, revisit if 1 is enough
 */

inline static void sema_init(semaphore_t * s, int count) {
    KeInitializeSemaphore(s, count, 1);
}

inline static void sema_init_limit(semaphore_t* s, int count, int limit) {
    KeInitializeSemaphore(s, count, limit);
}

// Linux macros

#define likely(x) (x)
#define unlikely(x) (x)

// Kernel print

#define KERN_ERR "ERROR: "
#define KERN_INFO "INFO: "

#define printk(x, ...) DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, x, __VA_ARGS__)

// Interrupt support

typedef BOOLEAN irqreturn_t;
#define IRQ_HANDLED 1
#define IRQ_NONE 0

typedef KSERVICE_ROUTINE * irq_handler_t;

// List support

typedef struct {
    int m_magic;
    LIST_ENTRY m_entry;
} list_head_t;

typedef LIST_ENTRY list_entry_t;

#if 0
#define member_offset(c, m) (((gctUINT8 *) &(c->m) - (gctUINT8 *) c))
#define container_pointer(c, p, m) (((gctUINT8 *) p) - member_offset(c,m))

#define list_for_each_entry(elem, head, entry) \
    for (elem = NULL, \
         *((gctUINT8 **)&elem) = container_pointer(elem, &((head)->Flink), entry); \
         elem != NULL; \
        *((gctUINT8 **)&elem) = container_pointer(elem, &(elem->entry.Flink), entry))

#define list_for_each_entry_safe(elem, tmp, head, entry) \
    for (elem = NULL, \
         *((gctUINT8 **)&elem) = container_pointer(elem, &((head)->Flink), entry), \
         *((gctUINT8 **)&tmp) = (elem != NULL ? container_pointer(elem, elem->entry.Flink, entry) : NULL); \
         elem != NULL; \
         elem = tmp, \
         *((gctUINT8 **)&tmp) = (elem != NULL ? container_pointer(elem, elem->entry.Flink, entry) : NULL))

#define INIT_LIST_HEAD(x) InitializeListHead(x)

#define list_add_tail(entry, head) InsertTailList(head, entry)
#define list_add(entry, head) InsertHeadList(head, entry)
#define list_del(entry) RemoveEntryList(entry)
#else
inline void list_initialize(list_head_t * head) {
    head->m_magic = 'head';
    InitializeListHead(&head->m_entry);
}

inline void list_add_tail(list_entry_t * entry, list_head_t * head) {
    if (head->m_magic != 'head') __debugbreak();
    InsertTailList(&head->m_entry, entry);
}

inline void list_add(list_entry_t * entry, list_head_t * head) {
    if (head->m_magic != 'head') __debugbreak();
    InsertHeadList(&head->m_entry, entry);
}

inline void list_del(list_entry_t * entry) {
    RemoveEntryList(entry);
}

inline int list_empty(list_head_t * head) {
    return IsListEmpty(&head->m_entry);
}

inline list_entry_t * list_first(list_head_t * head) {
    if (head->m_magic != 'head') __debugbreak();
    return head->m_entry.Flink;
}

inline list_entry_t * list_last(list_head_t * head) {
    if (head->m_magic != 'head') __debugbreak();
    return head->m_entry.Flink;
}

inline list_entry_t * list_sentry(list_head_t * head) {
    return &head->m_entry;
}

inline list_entry_t * list_next(list_entry_t * entry) {
    return entry->Flink;
}

#endif

/* atomic support -- see https://www.kernel.org/doc/html/v4.12/core-api/atomic_ops.html */
typedef struct { __int64 counter; } atomic_t;
inline __int64 atomic_read(atomic_t * a) { return a->counter; }
inline void atomic_add(__int64 value, atomic_t * a) { a->counter += value; }
inline void atomic_sub(__int64 value, atomic_t * a) { a->counter -= value; }
inline void atomic_set(atomic_t * a, __int64 value) { a->counter = value; }
inline int atomic_dec_and_test(atomic_t * a) { return (InterlockedDecrement64(&a->counter) == 0); }
inline void atomic_inc(atomic_t * a) { a->counter++; }
inline void atomic_dec(atomic_t * a) { a->counter--; }
inline __int64 atomic_inc_return(atomic_t * a) { return InterlockedIncrement64(&a->counter); }
inline __int64 atomic_dec_return(atomic_t * a) { return InterlockedDecrement64(&a->counter); }

/* debugfs support */
#if 0 // from linux
struct seq_file {
	char *buf;
	size_t size;
	size_t from;
	size_t count;
	size_t pad_until;
	loff_t index;
	loff_t read_pos;
	u64 version;
	struct mutex lock;
	const struct seq_operations *op;
	int poll_event;
	const struct file *file;
	void *private;
};
#endif

typedef struct seq_file {
    void *ptr;
} seq_file_t;

void seq_printf(seq_file_t *, const char * format, ...);

/* kmalloc support */

void * kmalloc(size_t size, int flags);
void * kzalloc(size_t size, int flags);
void kfree(void *ptr);

typedef unsigned __int64 dma_addr_t;
typedef unsigned __int64 phys_addr_t;

struct sg_table {
	struct scatterlist *sgl;	/* the list */
	unsigned int nents;		/* number of mapped entries */
	unsigned int orig_nents;	/* original size of list */
};

struct page {
    int dummy;
};

void * dma_alloc_coherent(struct device * device, size_t size, dma_addr_t * address, gfp_t flags);
void dma_free_coherent(struct device * device, size_t size, void * cpu_address, dma_addr_t dma_adress);
phys_addr_t dma_to_phys(struct device * device, dma_addr_t dma_addr);
int dma_mmap_writecombie(struct device * device, struct vm_area_struct * vma, void * cpu_address, dma_addr_t dma_address, size_t size);

int sg_alloc_table_from_pages(struct sg_table *sgt, struct page **pages,
			      unsigned int n_pages, size_t offset,
			      size_t size, gfp_t gfp_mask);

#define PAGE_MASK  ((1 << PAGE_SHIFT) - 1)
#define PFN_DOWN(x) ((x) >> PAGE_SHIFT)
#define pfn_to_page(x) ((struct page *) x)
#define page_to_pfn(x) ((phys_addr_t) x)
#define phys_to_pfn(x) (PFN_DOWN(x))
#define phys_to_page(x) (pfn_to_page(phys_to_pfn(x)))
#define nth_page(page,n) pfn_to_page(page_to_pfn((page)) + (n))

#define KERNEL_VERSION(a,b,c) ((a << 16) | (b << 8) | c)
#define LINUX_VERSION_CODE KERNEL_VERSION(4,3,3)

struct device {
    int dummy;
};

struct platform_device {
    struct device dev;
};


#if 0
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>

#include <linux/idr.h>
#endif

#ifdef MODVERSIONS
#  include <linux/modversions.h>
#endif

#if 0
#include <asm/io.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,7,0)
    #include <linux/uaccess.h>
#else
    #include <asm/uaccess.h>
#endif
#endif

#if 0
#if ENABLE_GPU_CLOCK_BY_DRIVER && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
#include <linux/clk.h>
#endif
#endif

#define NTSTRSAFE_NO_CCH_FUNCTIONS
#include "gc_hal.h"
#include "gc_hal_driver.h"
#include "gc_hal_kernel.h"
#include "gc_hal_kernel_platform.h"
#include "gc_hal_kernel_device.h"
#include "gc_hal_kernel_os.h"
#include "gc_hal_kernel_debugfs.h"
#include "gc_hal_ta.h"

extern gctBOOL  g_bWddmMode;
extern gctBOOL  g_bLimitAllocBelow4gbPa;

#define FIND_TASK_BY_PID(x) find_task_by_pid(x)

#ifndef DEVICE_NAME
#   define DEVICE_NAME              "galcore"
#endif

#ifndef CLASS_NAME
#   define CLASS_NAME               "graphics_class"
#endif

#define GetPageCount(size, offset)     ((((size) + ((offset) & ~PAGE_MASK)) + PAGE_SIZE - 1) >> PAGE_SHIFT)

#define gcdVM_FLAGS (VM_IO | VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP)

/* Protection bit when mapping memroy to user sapce */
#define gcmkPAGED_MEMROY_PROT(x)    pgprot_writecombine(x)

#define gcdSUPPRESS_OOM_MESSAGE 1

#if gcdSUPPRESS_OOM_MESSAGE
#define gcdNOWARN __GFP_NOWARN
#else
#define gcdNOWARN 0
#endif

#   ifdef gcdIRQ_SHARED
#       define gcdIRQF_FLAG   (IRQF_SHARED | IRQF_TRIGGER_HIGH)
#   else
#       define gcdIRQF_FLAG   (IRQF_TRIGGER_HIGH)
#   endif

extern struct device *galcore_device;

/******************************************************************************\
********************************** Structures **********************************
\******************************************************************************/
typedef struct _gcsIOMMU * gckIOMMU;

#define kDBSize 1024
#define kDBOffset 964321

typedef struct _gcsINTEGER_DB * gcsINTEGER_DB_PTR;
typedef struct _gcsINTEGER_DB
{
    void *                      table[kDBSize];
    spinlock_t                  lock;
    gctINT                      curr;
}
gcsINTEGER_DB;

struct _gckOS
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to device */
    gckGALDEVICE                device;

    /* Memory management */
    mutex_t                     mdlMutex;
    list_head_t                 mdlHead;

    /* Kernel process ID. */
    gctPID                      kernelProcessID;

    /* Signal management. */

    /* Lock. */
    spinlock_t                  signalLock;

    /* signal id database. */
    gcsINTEGER_DB               signalDB;

    /* workqueue for os timer. */
    workqueue_t *               workqueue;

    /* Allocate extra page to avoid cache overflow */
    page_t * paddingPage;

    /* Detect unfreed allocation. */
    atomic_t                    allocateCount;

    list_head_t                 allocatorList;

    gcsDEBUGFS_DIR              allocatorDebugfsDir;

    /* Lock for register access check. */
    spinlock_t                  registerAccessLock;

    /* External power states. */
    gctBOOL                     powerStates[gcdMAX_GPU_COUNT];

    /* External clock states. */
    gctBOOL                     clockStates[gcdMAX_GPU_COUNT];

    /* IOMMU. */
    gckIOMMU                    iommu;

    /* Dump in kernel. */
    HANDLE                      dumpFile;
    mutex_t                     dumpFilpMutex;

    int                         dumpTarget;
    char                        dumpFileName[256];
    gcsDEBUGFS_DIR              dumpDebugfsDir;

    /* Flip */
    gctBOOL                     flipActive;
};

typedef struct _gcsSIGNAL * gcsSIGNAL_PTR;
typedef struct _gcsSIGNAL
{
    /* Kernel sync primitive. */
    volatile unsigned int done;
    spinlock_t lock;

    KEVENT event;

    /* Manual reset flag. */
    gctBOOL manualReset;

    /* The reference counter. */
    atomic_t ref;

    /* The owner of the signal. */
    gctPID processID;

    /* ID. */
    gctUINT32 id;

#if gcdLINUX_SYNC_FILE
#  ifndef CONFIG_SYNC_FILE
    /* Parent timeline. */
    struct sync_timeline * timeline;
#  else
    struct dma_fence *fence;
#  endif
#endif
}
gcsSIGNAL;

typedef struct _gcsOSTIMER * gcsOSTIMER_PTR;
typedef struct _gcsOSTIMER
{
    KDPC m_dpc;
    KTIMER m_timer;
    gctTIMERFUNCTION m_function;
    gctPOINTER m_data;
    gctBOOL m_running;
    gctBOOL m_fired;
} gcsOSTIMER;

gceSTATUS
gckOS_ImportAllocators(
    gckOS Os
    );

gceSTATUS
gckOS_FreeAllocators(
    gckOS Os
    );

gceSTATUS
_ConvertLogical2Physical(
    IN gckOS Os,
    IN gctPOINTER Logical,
    IN gctPID ProcessID,
    IN PLINUX_MDL Mdl,
    OUT gctPHYS_ADDR_T * Physical
    );

gctBOOL
_QuerySignal(
    IN gckOS Os,
    IN gctSIGNAL Signal
    );

static inline gctPID
_GetProcessID(
    void
    )
{
    return gcmALL_TO_UINT32(PsGetCurrentProcessId());
}

static inline void
_MemoryBarrier(
    void
    )
{
    KeMemoryBarrier();
}

static inline void
_Barrier(
    void
    )
{
    KeMemoryBarrierWithoutFence();
}

void
gckIOMMU_Destory(
    IN gckOS Os,
    IN gckIOMMU Iommu
    );

gceSTATUS
gckIOMMU_Construct(
    IN gckOS Os,
    OUT gckIOMMU * Iommu
    );

gceSTATUS
gckIOMMU_Map(
    IN gckIOMMU Iommu,
    IN gctUINT32 DomainAddress,
    IN gctUINT32 Physical,
    IN gctUINT32 Bytes
    );

gceSTATUS
gckIOMMU_Unmap(
    IN gckIOMMU Iommu,
    IN gctUINT32 DomainAddress,
    IN gctUINT32 Bytes
    );


#endif /* __gc_hal_kernel_wcos_h_ */

