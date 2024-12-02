/*
 * Copyright 2022-2024 NXP
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

#include "precomp.h"
#include "os/windows/src/comm.h"

extern "C"
{

#include "boot\dts\freescale\board.h"
#include "linux\completion.h"
#include "linux\delay.h"
#include "linux\dev_printk.h"
#include "linux\dma-mapping.h"
#include "linux\err.h"
#include "linux\interrupt.h"
#include "linux\gfp.h"
#include "linux\jiffies.h"
#include "linux\of.h"
#include "linux\platform_device.h"
#include "linux\slab.h"
#include "linux\spinlock.h"
#include "linux\types.h"
#include "linux\of_device.h"
#include "linux\preempt.h"
#include "linux\atomic.h"
#include "linux\ktime.h"
#include "asm-generic\bug.h"
#include "linux\delay.h"
#include "drm\drm_fourcc.h"
#include "drm\drm_print.h"
#include "linux\iopoll.h"
#include "linux\clk-provider.h"
#include "linux\clk.h"
#include "linux\i2c.h"
#include "linux\regmap.h"
#include "drm\drm_connector.h"
#include "linux\gpio\consumer.h"
#include "asm-generic\bitops\non-atomic.h"

//
// memory allocation
//

void kfree(const void *ptr)
{
    ExFreePool((void*)ptr);
}

void *kmalloc(size_t size, gfp_t flags)
{
    void *ptr;

    ptr = ExAllocatePool2(POOL_FLAG_NON_PAGED, size, '8XMI');
    if (ptr && (flags & __GFP_ZERO))
    {
        memset(ptr, 0, size);
    }

    return ptr;
}

/* aproximate linux krealloc - additionaly original size of the block is needed
   and can only grow */
void* modified_krealloc(void* orig_ptr, size_t orig_size, size_t new_size, gfp_t flags)
{
    void* new_ptr;

    if (new_size == 0) {
        kfree(orig_ptr);
        return NULL;
    }
    else if (!orig_ptr) {
        return kmalloc(new_size, flags);
    }
    else if (new_size <= orig_size) {
        return orig_ptr;
    }
    else {
        new_ptr = kmalloc(new_size, flags);
        if (new_ptr) {
            memcpy(new_ptr, orig_ptr, orig_size);
            kfree(orig_ptr);
        }
    }
    return new_ptr;
}

void* kmemdup(const void* src, size_t len, gfp_t gfp)
{
    void* p;

    p = kmalloc(len, gfp);
    if (p)
        memcpy(p, src, len);

    return p;
}

void *devm_kmemdup(struct device *dev, const void *src, size_t len, gfp_t gfp)
{
    return kmemdup(src, len, gfp);
}


//
// platform helpers
//

struct resource* platform_get_resource(struct platform_device* dev,
    unsigned int type, unsigned int num)
{
    if (type != IORESOURCE_MEM)
    {
        return nullptr;
    }

    if (num >= dev->num_resources)
    {
        return nullptr;
    }

    return &dev->resource[num];
}

void __iomem *devm_ioremap_resource(struct device *dev,
    const struct resource *res)
{
    if (!res)
    {
        return ERR_PTR(-EINVAL);
    }

    void* ptr = __ioremap(res->start, resource_size(res));
    if (!ptr)
    {
        return ERR_PTR(-ENOMEM);
    }

    return ptr;
}

void __iomem* devm_platform_ioremap_resource(struct platform_device *pdev,
    unsigned int index)
{
    resource* res = platform_get_resource(pdev, IORESOURCE_MEM, index);
    return devm_ioremap_resource(&pdev->dev, res);
}

void __iomem *devm_ioremap(struct device *dev, resource_size_t offset,
    resource_size_t size)
{
    void* ptr = __ioremap(offset, size);
    if (!ptr)
    {
        return ERR_PTR(-ENOMEM);
    }

    return ptr;
}

void devm_iounmap(struct device *dev, void __iomem *addr,
    resource_size_t size)
{
    iounmap(addr, size);
}

int platform_irq_count(struct platform_device *pdev)
{
    return _platform_irq_count(pdev);
}

int platform_get_irq(struct platform_device *pdev, unsigned int num)
{
    return num;
}

int platform_get_irq_byname(struct platform_device* pdev, const char* name)
{
    for (int i = 0; i < NR_IRQS; i++)
    {
        /* irq_desc is global for all displays, so depending on enabled displays
           some array items can be null. strcmp doesn't accept nullptr,skip it */
        if (irq_desc[i].name == nullptr) {
            continue;
        }
        if (!strcmp(irq_desc[i].name, name))
        {
            return i;
        }
    }

    return -1;
}

struct property* find_property_by_name(const struct device_node* np, const char *name)
{
    if (!np)
    {
        return nullptr;
    }

    property* p = np->properties;

    while (p && strlen(p->name))
    {
        if (!strcmp(p->name, name))
        {
            return p;
        }
        p++;
    }

    return nullptr;
}

const void *of_get_property(const struct device_node *np, const char *name,
    int *lenp)
{
    struct property *pp = find_property_by_name(np, name);

    if (pp) {
        if (lenp) {
            *lenp = pp->length;
        }
        return pp->value;
    }
    return nullptr;
}

bool of_property_read_bool(const struct device_node* np,
    const char* propname)
{
    struct property *p = find_property_by_name(np, propname);
    return p ? true : false;
}

int of_property_read_u32_array(const struct device_node *np,
    const char *propname,
    u32 *out_values, size_t sz)
{
    struct property* p = find_property_by_name(np, propname);
    if (!p)
    {
        return -EINVAL;
    }

    for (int i = 0; i < sz; i++)
    {
        out_values[i] = ((int*)p->value)[i];
    }

    return 0;
}

int of_property_read_long_array(const struct device_node* np,
    const char* propname,
    LONG* out_values, size_t sz)
{
    struct property* p = find_property_by_name(np, propname);
    if (!p)
    {
        return -EINVAL;
    }

    for (int i = 0; i < sz; i++)
    {
        out_values[i] = ((LONG*)p->value)[i];
    }

    return 0;
}

const struct of_device_id *of_match_device(const struct of_device_id *matches,
    const struct device *dev)
{
    const struct of_device_id *best_match = NULL;
    if ((!matches) || (!dev)) {
        return nullptr;
    }
    struct	property *properties = dev->of_node.properties;
    if (!properties) {
        return nullptr;
    }

    for (; matches->compatible[0]; matches++) {
        for (; properties->name[0]; properties++) {
            if (_stricmp(properties->name, "compatible") == 0) {
                if (_stricmp((char*)properties->value, matches->compatible) == 0) {
                    best_match = matches;
                    break;
                }
            }
        }
        properties = dev->of_node.properties;
    }

    return best_match;
}

int of_property_read_string(const struct device_node* np,
    const char* propname, const char** out_string)
{
    const struct property *p = find_property_by_name(np, propname);
    if (!p)
    {
        return -EINVAL;
    }
    if (!p->value)
    {
        return -ENODATA;
    }

    *out_string = *(const char**)p->value;
    return 0;
}

int of_property_match_string(const struct device_node* np,
    const char* propname, const char* string)
{
    struct property* p = find_property_by_name(np, propname);
    if (!p)
    {
        return -EINVAL;
    }

    for (int i = 0; i < p->length; i++)
    {
        if (!strcmp(((const char**)p->value)[i], string))
        {
            return i;
        }
    }

    return -EINVAL;
}

int of_device_is_compatible(const struct device_node *device,
    const char *compat)
{
    if ((!device) || (!compat)) {
        return FALSE;
    }
    struct	property *properties = find_property_by_name(device, "compatible");
    if (properties == nullptr) {
        return FALSE;
    }
    if (_stricmp((char*)properties->value, compat) != 0) {
        return FALSE;
    }
    return TRUE;
}

struct clk *devm_clk_get(struct device *dev, const char *id)
{
    struct clk_init_data_desc *clk_desc;
    struct clk *clk = nullptr;
    struct clk *clk_parent = nullptr;
    // nullptr is valid ret value here because we need to handle case when
    // the clock is not defined in the DT and can be safely ignored by the driver
    if (!dev)
    {
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_WARNING_LEVEL,
            "Undefined clock\n");
        return nullptr;
    }

    clk_desc = dev->of_clk;

    while (clk_desc && strlen(clk_desc->name)) {
        if (!strcmp(clk_desc->name, id)) {
            if (dev->get_clock_item) {
                clk = (dev->get_clock_item)(clk_desc->id);
            }
            if (!clk) {
                break;
            }
            if (clk_desc->parent_sel != 0) {
                clk_parent = (dev->get_clock_item)(clk_desc->parent_sel);
                if (clk_parent && clk->clk_set_parent) {
                    if (clk->flags & CLK_SCFW_PRESENT) {
                        (void)(clk->clk_enable)(clk, false, false);
                    }
                    (void)(clk->clk_set_parent)(clk, clk_parent);
                }
            }
            if ((clk_desc->rate != 0) && clk->clk_set_rate) {
                (void)(clk->clk_set_rate)(clk, clk_desc->rate);
            }
            if ((clk_desc->parent_enable != 0) && clk->clk_enable) {
                /* Enable all parents recursively */
                (void)(clk->clk_enable)(clk->parent, true, true);
            }
            break;
        }
        clk_desc++;
    }

    if (!clk)
    {
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_WARNING_LEVEL,
            "Undefined clock\n");
    }

    return clk;
}

int devm_clk_bulk_get(struct device *dev, int num_clks, struct clk_bulk_data *clks)
{
    int ret;
    int i;

    for (i = 0; i < num_clks; i++) {
        clks[i].clk = NULL;
    }
    for (i = 0; i < num_clks; i++) {
        clks[i].clk = devm_clk_get(dev, clks[i].id);
        if (IS_ERR(clks[i].clk)) {
            ret = PTR_ERR(clks[i].clk);
            clks[i].clk = NULL;
            dev_err(dev, "Failed to get clk '%s': %d\n", clks[i].id, ret);
            return ret;
        }
    }

    return 0;
}

void clk_bulk_disable_unprepare(int num_clks, const struct clk_bulk_data *clks)
{

    while (--num_clks >= 0)
        clk_disable_unprepare(clks[num_clks].clk);
}

int clk_bulk_prepare_enable(int num_clks, const struct clk_bulk_data *clks)
{
    int ret;
    int i;

    for (i = 0; i < num_clks; i++) {
        ret = clk_prepare_enable(clks[i].clk);
        if (ret) {
            pr_err("Failed to enable clk '%s': %d\n", clks[i].id, ret);
            clk_bulk_disable_unprepare(i, clks);
            return  ret;
        }
    }

    return 0;
}

int dev_pm_domain_attach_by_name(struct device* dev, const char* name)
{
    int index = of_property_match_string(&dev->of_node, "power-domain-names",
        name);
    if (index < 0)
        return -EINVAL;

    struct property* p = find_property_by_name(&dev->of_node, "power-domains");
    if (!p || index > p->length)
    {
        return -EINVAL;
    }

    return ((int*)p->value)[index];
}

//
// interrupt handling
//

static void IrqDpcHandle(KDPC* pDPC, PVOID DeferredContext,
    PVOID SystemArgument1, PVOID SystemArgument2)
{
    struct irq_desc *p_irq_desc = (struct irq_desc *)DeferredContext;

    UNREFERENCED_PARAMETER(pDPC);
    UNREFERENCED_PARAMETER(SystemArgument1);
    UNREFERENCED_PARAMETER(SystemArgument2);

    if (p_irq_desc->thread_fn) {
        p_irq_desc->thread_fn(p_irq_desc->irq_data.irq, p_irq_desc->dev_id);
    }
}

bool irq_handle(int irq)
{
    irqreturn_t ret;

    if (irq < 0 || irq > NR_IRQS)
    {
        DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL,
            "Invalid irq (%d)\n", irq);
        return false;
    }

    irq_data* data = &irq_desc[irq].irq_data;
    NT_ASSERT(data);
    if (data->chip.irq_check && data->chip.irq_check(data))
    {
        if (irq_desc[irq].handler)
        {
            ret = irq_desc[irq].handler(data->irq, irq_desc[irq].dev_id);
            if ((ret == IRQ_WAKE_THREAD) && irq_desc[irq].thread_fn) {
                KeInsertQueueDpc(&irq_desc[irq].thread_dpc, nullptr, nullptr);
            }
            return true;
        }
    }

    return false;
}

/**
 *	request_threaded_irq - Add a handler for an interrupt line
 *	@irq:	The interrupt line to allocate
 *	@handler:	Function to be called when the IRQ occurs.
*	@thread_fn:	Function to be called in DPC context (DISPATCH_LEVEL), when @handler returns IRQ_WAKE_THREAD.
 *	@irqflags:	Handling flags
 *	@devname:	Name of the device generating this interrupt
 *	@dev_id:	A cookie passed to the handler function
 *
 *	This call allocates interrupt resources and enables the interrupt line and IRQ handling.
 */
int request_threaded_irq(unsigned int irq, irq_handler_t handler,
    irq_handler_t thread_fn, unsigned long irqflags,
    const char* devname, void* dev_id)
{
    if (irq >= NR_IRQS) {
        return -1;
    }

    if (thread_fn) {
        KeInitializeDpc(&irq_desc[irq].thread_dpc, IrqDpcHandle, &irq_desc[irq]);
        irq_desc[irq].thread_fn = thread_fn;
    } else {
        irq_desc[irq].thread_fn = NULL;
    }
    irq_desc[irq].handler = handler;
    irq_desc[irq].dev_id = dev_id;

    irq_enable(&irq_desc[irq]);

    return 0;
}

//
// I/O memory mapping functions.
//

void __iomem *__ioremap(phys_addr_t phys_addr, size_t size)
{
    PHYSICAL_ADDRESS PhysAddr;
    PhysAddr.QuadPart = phys_addr;

    return MmMapIoSpaceEx(
        PhysAddr,
        size,
        PAGE_READWRITE | PAGE_NOCACHE);
}

void iounmap(void __iomem *addr, size_t size)
{
    MmUnmapIoSpace(addr, size);
}

//
// debug functions
//

void _dev_err(const struct device *dev, const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vDbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, fmt, argp);
    va_end(argp);
}

void _dev_warn(const struct device *dev, const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vDbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_WARNING_LEVEL, fmt, argp);
    va_end(argp);
}

void _dev_notice(const struct device *dev, const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vDbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_INFO_LEVEL, fmt, argp);
    va_end(argp);
}

void _dev_info(const struct device *dev, const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vDbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_INFO_LEVEL, fmt, argp);
    va_end(argp);
}

void _dev_dbg(const struct device *dev, const char *fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);
    vDbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_TRACE_LEVEL, fmt, argp);
    va_end(argp);
}

void __warn(const char* file, const char* func, int line)
{
    DbgPrintEx(DPFLTR_IHVVIDEO_ID, DPFLTR_ERROR_LEVEL, "ERROR assert condition not true. file: %s function: %s line: %d", file, func, line);
}

void __warn_printfmt(const char* file, const char* func, int line, const char* fmt, ...)
{
    static CHAR Buffer[256];

    sprintf(Buffer,
        "ERROR assert condition not true. file: %s function: %s line: %d:",
        file, func, line);
    Buffer[255] = 0;

    va_list argp;
    va_start(argp, fmt);

    vDbgPrintExWithPrefix(Buffer, DPFLTR_IHVVIDEO_ID,
        DPFLTR_ERROR_LEVEL, fmt, argp);

    va_end(argp);
}

//
// spinlock
//

void wspin_lock_init(spinlock_t* lock)
{
    KeInitializeSpinLock(lock);
}

void wspin_lock_irqsave(spinlock_t* lock, PKIRQL flags)
{
    KIRQL fl;

    KeAcquireSpinLock(lock, &fl);
    if (flags) {
        *flags = fl;
    }
}

void wspin_unlock_irqrestore(spinlock_t* lock, PKIRQL flags)
{
    KIRQL fl = 0;

    if (flags) {
        fl = *flags;
    }
    KeReleaseSpinLock(lock, fl);
}

//
// delay
//

void udelay(unsigned int usecs)
{
    KeStallExecutionProcessor(usecs);
}

void mdelay(unsigned int msecs)
{
    KeStallExecutionProcessor(msecs * 1000);
}

void msleep(unsigned int msecs)
{
    LARGE_INTEGER Interval;
    Interval.QuadPart = -10 * 1000 * (LONGLONG)msecs;  /* number of msecs in 100 ns units */
    KeDelayExecutionThread(KernelMode, FALSE, &Interval);
}

void usleep_range(unsigned long min, unsigned long max)
{
    LARGE_INTEGER native_delay;
    native_delay.QuadPart = -10;  /* 1 us relative time */
    native_delay.QuadPart *= min;
    KeDelayExecutionThread(KernelMode, FALSE, &native_delay);
}

//
// dma mapping
//

void *dma_alloc_coherent(struct device *dev, size_t size,
    dma_addr_t *dma_handle, gfp_t gfp)
{
    PHYSICAL_ADDRESS LowestAcceptableAddress, BoundaryAddressMultiple,
        HighestAcceptableAddress;
    LowestAcceptableAddress.QuadPart = 0;
    BoundaryAddressMultiple.QuadPart = 0;
    HighestAcceptableAddress.QuadPart = 3 * (1ULL << 30) - 1ULL;

    PVOID Ptr = MmAllocateContiguousMemorySpecifyCache(
        size,
        LowestAcceptableAddress,
        HighestAcceptableAddress,
        BoundaryAddressMultiple,
        MmWriteCombined);

    PHYSICAL_ADDRESS PhysAddr = MmGetPhysicalAddress(Ptr);

    *dma_handle = static_cast<UINT32>(PhysAddr.QuadPart);

    return Ptr;
}

void dma_free_coherent(struct device *dev, size_t size,
    void *cpu_addr, dma_addr_t dma_handle)
{
    MmFreeContiguousMemory(cpu_addr);
}

//
// time, jiffies
//

unsigned long get_hz(void)
{
    return (1UL * 1000 * 1000 * 10) / KeQueryTimeIncrement();
}

unsigned long long get_jiffies(void)
{
    LARGE_INTEGER Ticks;
    KeQueryTickCount(&Ticks);

    return Ticks.QuadPart;
}

unsigned int _jiffies_to_msecs(const unsigned long long j)
{
    ULONG Increment = KeQueryTimeIncrement();
    return (unsigned int)(j * Increment) / 10000L;
}

unsigned long long _msecs_to_jiffies(const unsigned int m)
{
    ULONG Increment = KeQueryTimeIncrement();
    return DIV_ROUND_UP(m * 10000L, Increment);
}

unsigned long long nsecs_to_jiffies(u64 n)
{
    ULONG Increment = KeQueryTimeIncrement();
    return DIV_ROUND_UP(n, (100L * Increment));
}

ktime_t ktime_get(void)
{
    LARGE_INTEGER Time;
    KeQuerySystemTimePrecise(&Time);

    return Time.QuadPart / 100;
}

//
// DRM
//

static char printable_char(int c)
{
    return isprint(c) ? (char)c : '?';
}

/**
 * drm_get_format_name - fill a string with a drm fourcc format's name
 * @format: format to compute name of
 * @buf: caller-supplied buffer
 */
const char *drm_get_format_name(uint32_t format, struct drm_format_name_buf *buf)
{
    RtlStringCbPrintfA(buf->str, sizeof(buf->str),
        "%c%c%c%c %s-endian (0x%08x)",
        printable_char(format & 0xff),
        printable_char((format >> 8) & 0xff),
        printable_char((format >> 16) & 0xff),
        printable_char((format >> 24) & 0x7f),
        format & DRM_FORMAT_BIG_ENDIAN ? "big" : "little",
        format);

    return buf->str;
}

void __drm_info(const char *format, ...)
{
    va_list argp;
    va_start(argp, format);

    vDbgPrintExWithPrefix("[drm] ", DPFLTR_IHVVIDEO_ID,
        DPFLTR_INFO_LEVEL, format, argp);

    va_end(argp);
}

void __drm_dbg(enum drm_debug_category category, const char *format, ...)
{
    va_list argp;
    va_start(argp, format);

    vDbgPrintExWithPrefix("[drm] ", DPFLTR_IHVVIDEO_ID,
        DPFLTR_TRACE_LEVEL, format, argp);

    va_end(argp);
}

void __drm_err(const char *format, ...)
{
    va_list argp;
    va_start(argp, format);

    vDbgPrintExWithPrefix("[drm] ", DPFLTR_IHVVIDEO_ID,
        DPFLTR_ERROR_LEVEL, format, argp);

    va_end(argp);
}

void __drm_warn(const char *format, ...)
{
    va_list argp;
    va_start(argp, format);

    vDbgPrintExWithPrefix("[drm] ", DPFLTR_IHVVIDEO_ID,
        DPFLTR_WARNING_LEVEL, format, argp);

    va_end(argp);
}

//
// atomic functions
//
int atomic_read(atomic_t *v)
{
    return InterlockedOr(v, 0);
}

void atomic_set(atomic_t *v, int i)
{
    (void)InterlockedExchange(v, i);
}

int atomic_dec_return(atomic_t *v)
{
    return (int)InterlockedDecrement(v);
}

int atomic_inc_return(atomic_t *v)
{
    return (int)InterlockedIncrement(v);
}

//
// completion
//

static void CompletionDpcRoutine(KDPC *pDPC, PVOID DeferredContext,
    PVOID SystemArgument1, PVOID SystemArgument2)
{
    struct completion *pDisableCompletion = (completion*)DeferredContext;

    UNREFERENCED_PARAMETER(pDPC);
    UNREFERENCED_PARAMETER(SystemArgument1);
    UNREFERENCED_PARAMETER(SystemArgument2);

    KeSetEvent(&pDisableCompletion->event, 0, FALSE);
}

void init_completion(struct completion *x)
{
    KeInitializeEvent(&x->event, SynchronizationEvent, FALSE);
    KeInitializeDpc(&x->dpc, CompletionDpcRoutine, x);
}

void reinit_completion(struct completion *x)
{
    KeResetEvent(&x->event);
}

void wait_for_completion(struct completion *x)
{
    KeWaitForSingleObject(&x->event, Executive, KernelMode, FALSE, NULL);
}

unsigned long wait_for_completion_timeout(struct completion *x,
    unsigned long long timeout)
{
    LARGE_INTEGER   Timeout;
    LONG Inc = KeQueryTimeIncrement();

    // convert timeout in jiffies to 100 ns units
    Timeout.QuadPart = -1 * timeout * Inc;

    NTSTATUS Status = KeWaitForSingleObject(&x->event, Executive,
        KernelMode, TRUE, &Timeout);

    return (Status == STATUS_SUCCESS) ? 1 : 0;
}

void complete(struct completion *x)
{
    KeInsertQueueDpc(&x->dpc, nullptr, nullptr);
}

//
// REGMAP
//
static NTSTATUS mmio_write32(iotarget_handles *tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length)
{
    UINT i;
    PUINT uintbuff = (PUINT)buffer;

    for (i = 0; i < buffer_length; i++) {
        writel(uintbuff[i], (tgt->base + reg_addr + i));
    }
    return STATUS_SUCCESS;
}

static NTSTATUS mmio_read32(iotarget_handles *tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length)
{
    UINT i;
    PUINT uintbuff = (PUINT)buffer;

    for (i = 0; i < buffer_length; i++) {
        uintbuff[i] = readl(tgt->base + reg_addr + i);
    }
    return STATUS_SUCCESS;
}

static NTSTATUS mmio_write8(iotarget_handles* tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length)
{
    UINT i;
    PUCHAR uintbuff = (PUCHAR)buffer;

    for (i = 0; i < buffer_length; i++) {
        writeb(uintbuff[i], (tgt->base + reg_addr + i));
    }
    return STATUS_SUCCESS;
}

static NTSTATUS mmio_read8(iotarget_handles* tgt, ULONG reg_addr, PVOID buffer, ULONG buffer_length)
{
    UINT i;
    PUCHAR uintbuff = (PUCHAR)buffer;

    for (i = 0; i < buffer_length; i++) {
        uintbuff[i] = readb(tgt->base + reg_addr + i);
    }
    return STATUS_SUCCESS;
}

struct regmap *devm_regmap_init_mmio(struct device *dev, struct resource *res, const struct regmap_config* config)
{
    struct regmap *map;
    map = (struct regmap *)kmalloc(sizeof(*map), GFP_KERNEL | __GFP_ZERO);
    if (map == NULL) {
        return (struct regmap *)ERR_PTR(-ENOMEM);
    }
    if (config->val_bits == 32) {
        map->regmap_read = &mmio_read32;
        map->regmap_write = &mmio_write32;
    } else if (config->val_bits == 8) {
        map->regmap_read = &mmio_read8;
        map->regmap_write = &mmio_write8;
    } else {
        kfree(map);
        return (struct regmap*)ERR_PTR(-EINVAL);
    }
    map->io_target.base = (uint8_t *)devm_ioremap_resource(dev, res);
    if (IS_ERR(map->io_target.base)) {
        kfree(map);
        return (struct regmap *)PTR_ERR(map->io_target.base);
    }
    return map;
}

void regmap_release_mmio(struct regmap *map, struct resource *res)
{
    if (map) {
        if (res && map->io_target.base) {
            iounmap(map->io_target.base, resource_size(res));
        }
        kfree(map);
    }
}

struct regmap *devm_regmap_init_i2c(struct i2c_client *i2c)
{
    struct regmap *map = (struct regmap *)kmalloc(sizeof(*map),
        GFP_KERNEL | __GFP_ZERO);
    if (map == nullptr)
    {
        return (struct regmap *)ERR_PTR(-ENOMEM);
    }

    map->regmap_read = &i2c_read;
    map->regmap_write = &i2c_write;
    comm_clear_handle(&map->io_target);

    NTSTATUS Status = comm_initialize(i2c->connection_id, &map->io_target, (GENERIC_READ | GENERIC_WRITE));
    if (Status != STATUS_SUCCESS)
    {
        _dev_err(nullptr, "comm_initialize (i2c) failed, status-0x%lx\n", Status);
        kfree(map);
        return (struct regmap*)ERR_PTR(-ENOMEM);
    }

    return map;
}

void regmap_release_i2c(struct regmap *map)
{
    if (map) {
        comm_close(&map->io_target);
        kfree(map);
    }
}

int regmap_write(struct regmap *map, unsigned int reg, unsigned int val)
{
    NTSTATUS status;

    if (!map || !map->regmap_write) {
        return -EINVAL;
    }
    status = (map->regmap_write)(&map->io_target, reg, &val, 1);
    return (NT_SUCCESS(status) ? 0 : -EIO);
}

int regmap_bulk_write(struct regmap *map, unsigned int reg, const void *val, size_t val_count)
{
    NTSTATUS status;

    if (!map || !map->regmap_write) {
        return -EINVAL;
    }
    status = (map->regmap_write)(&map->io_target, reg, (PVOID)val, (ULONG)val_count);
    return (NT_SUCCESS(status) ? 0 : -EIO);
}

int regmap_read(struct regmap *map, unsigned int reg, unsigned int *val)
{
    NTSTATUS status;

    if (!map || !map->regmap_read) {
        return -EINVAL;
    }

    status = (map->regmap_read)(&map->io_target, reg, (PVOID)val, 1);
    return (NT_SUCCESS(status) ? 0 : -EIO);
}

int regmap_bulk_read(struct regmap *map, unsigned int reg, void *val, size_t val_count)
{
    NTSTATUS status;

    if (!map || !map->regmap_read) {
        return -EINVAL;
    }

    status = (map->regmap_read)(&map->io_target, reg, (PVOID)val, (ULONG)val_count);
    return (NT_SUCCESS(status) ? 0 : -EIO);
}

int regmap_update_bits(struct regmap *map, unsigned int reg, unsigned int mask, unsigned int val)
{
    NTSTATUS status;
    unsigned int tmp, orig = 0;

    if (!map || !map->regmap_read || !map->regmap_write) {
        return -EINVAL;
    }
    status = (map->regmap_read)(&map->io_target, reg, &orig, 1);
    if (!NT_SUCCESS(status)) {
        return -EIO;
    }
    tmp = orig & ~mask;
    tmp |= val & mask;
    status = (map->regmap_write)(&map->io_target, reg, &tmp, 1);

    return (NT_SUCCESS(status) ? 0 : -EIO);
}

int regmap_register_patch(struct regmap *map, const struct reg_sequence *regs, int num_regs)
{
    NTSTATUS status;
    int i;
    int ret = 0;

    if (!map || !map->regmap_write) {
        return -EINVAL;
    }
    for (i = 0; i < num_regs; i++) {
        status = (map->regmap_write)(&map->io_target, regs[i].reg, (PVOID)&regs[i].def, 1);
        ret |= (NT_SUCCESS(status) ? 0 : -EIO);
        if (regs[i].delay_us) {
            udelay(regs[i].delay_us);
        }
    }
    return ret;
}

/*
* Polls specific address until condition is met or timeout occurs
* param addr - address to poll (32bit value will be read into _read_val)
* param mask - mask to apply on the value read from address
* param cmp_val - value compared against masked read from address: (_read_val & mask) == cmp_val
* param non_equal - compare condition changed to (_read_val & mask) != cmp_val
* param delay_us - delay in us to wait after each poll
* param timeout_us - timeout in us after which the polling is terminated
* param sleep_before_read - delay (delay_us) applied also before first poll
*/
int _readl_poll_timeout(u32 *addr, u32 mask, u32 cmp_val,
    bool non_equal, unsigned long delay_us, u64 timeout_us, bool sleep_before_read)
{
    u32 _read_val;
    ktime_t __timeout = ktime_add_us(ktime_get(), timeout_us);
    if (sleep_before_read && delay_us) {
        usleep_range(delay_us, delay_us);
    }
    for (;;) {
        _read_val = readl(addr);
        if (non_equal) {
            if ((_read_val & mask) != cmp_val) {
                break;
            }
        }
        else
        {
            if ((_read_val & mask) == cmp_val) {
                break;
            }
        }
        if (timeout_us && ktime_compare(ktime_get(), __timeout) > 0) {
            _read_val = readl(addr);
            break;
        }
        if (delay_us) {
            usleep_range(delay_us, delay_us);
        }
    }
    if (non_equal) {
        if ((_read_val & mask) != cmp_val) {
            return 0;
        }
    }
    else
    {
        if ((_read_val & mask) == cmp_val) {
            return 0;
        }
    }
    return -ETIMEDOUT;
}

/*
* Polls specific regmap register until condition is met or timeout occurs
* param map - regmap
* param reg - register to poll
* param mask - mask to apply on the value read from address
* param cmp_val - value compared against masked read from address: (_read_val & mask) == cmp_val
* param non_equal - TRUE = compare condition changed to (_read_val & mask) != cmp_val
* param delay_us - delay in us to wait after each poll
* param timeout_us - timeout in us after which the polling is terminated
* param sleep_before_read - delay (delay_us) applied also before first poll
*/
int _regmap_read_poll_timeout_ex(struct regmap* map, u32 reg, u32 mask, u32 cmp_val,
    bool non_equal, unsigned long delay_us, u64 timeout_us, bool sleep_before_read)
{
    u32 _read_val;
    int ret;
    ktime_t __timeout = ktime_add_us(ktime_get(), timeout_us);
    if (sleep_before_read && delay_us) {
        usleep_range(delay_us, delay_us);
    }
    for (;;) {
        ret = regmap_read(map, reg, &_read_val);
        if (ret) {
            return ret;
        }
        if (non_equal) {
            if ((_read_val & mask) != cmp_val) {
                break;
            }
        }
        else
        {
            if ((_read_val & mask) == cmp_val) {
                break;
            }
        }
        if (timeout_us && ktime_compare(ktime_get(), __timeout) > 0) {
            ret = regmap_read(map, reg, &_read_val);
            if (ret) {
                return ret;
            }
            break;
        }
        if (delay_us) {
            usleep_range(delay_us, delay_us);
        }
    }
    if (non_equal) {
        if ((_read_val & mask) != cmp_val) {
            return 0;
        }
    }
    else
    {
        if ((_read_val & mask) == cmp_val) {
            return 0;
        }
    }
    return -ETIMEDOUT;
}

//
// Preemption
//
void preempt_disable(KIRQL *OldIrql)
{
    KeRaiseIrql(HIGH_LEVEL, OldIrql);
}

void preempt_enable(KIRQL OldIrql)
{
    KeLowerIrql(OldIrql);
}

//Hot plug detect
void drm_bridge_hpd_notify(struct platform_device* pdev,
    enum drm_connector_status status, unsigned int uid) {

    DXGKRNL_INTERFACE* pDxgkInterface = (DXGKRNL_INTERFACE *)pdev->data;
    DXGK_CHILD_STATUS ChildStatus;

    ChildStatus.Type = StatusConnection;
    ChildStatus.ChildUid = uid;
    ChildStatus.HotPlug.Connected = (status == connector_status_connected) ? TRUE : FALSE;
    pDxgkInterface->DxgkCbIndicateChildStatus(pDxgkInterface->DeviceHandle, &ChildStatus);
}

//
//GPIO
//

struct gpio_desc* devm_gpiod_get(struct device* dev,
    const char* con_id, unsigned long flags)
{
    LARGE_INTEGER gpio_connection_id;
    LONG cid[5] = {0};
    NTSTATUS status;
    struct gpio_desc* gpio;
    ACCESS_MASK acc_msk = GENERIC_WRITE;

    if (!dev || !con_id) {
        return nullptr;
    }
    if (of_property_read_long_array(&dev->of_node, con_id, (LONG*)&cid, 3) != 0) {
        return nullptr;
    }
    gpio_connection_id.LowPart = cid[0];
    gpio_connection_id.HighPart = cid[1];

    if (cid[2] == FLAG_DEV_I2C_EXPANDER) {
        /* For i2c expander, there are two more data bytes to be read, so repeat the reading with lenght = 5 */
        if (of_property_read_long_array(&dev->of_node, con_id, (LONG*)&cid, 5) != 0) {
            return nullptr;
        }
    }

    gpio = (struct gpio_desc*)kmalloc(sizeof(*gpio), GFP_KERNEL | __GFP_ZERO);
    if (gpio == NULL) {
        return (struct gpio_desc*)ERR_PTR(-ENOMEM);
    }
    gpio->flags = flags;
    gpio->device = cid[2];
    if (cid[2] == FLAG_DEV_I2C_EXPANDER) {
        gpio->expander_data_reg = cid[3];
        gpio->expander_pin_mask = cid[4];
        acc_msk = GENERIC_READ | GENERIC_WRITE;
    }

    comm_clear_handle(&gpio->io_target);
    status = comm_initialize(gpio_connection_id, &gpio->io_target, acc_msk);
    if (!NT_SUCCESS(status))
    {
        _dev_err(nullptr, "comm_initialize (gpio) failed, status-0x%lx\n", status);
        comm_close(&gpio->io_target);
        kfree(gpio);
        return (struct gpio_desc*)ERR_PTR(-ENODEV);
    }

    return gpio;
}

void devm_gpiod_put(struct device* dev, struct gpio_desc* desc)
{
    if (desc) {
        comm_close(&desc->io_target);
        kfree(desc);
    }
}

void gpiod_set_value(struct gpio_desc* desc, int value)
{
    if (!desc) {
        return;
    }
    if (desc->flags & FLAG_ACTIVE_LOW) {
        value = !value;
    }
    if (desc->device == FLAG_DEV_GPIO) {
        (void)gpio_write(&desc->io_target, value);
    }
    else if (desc->device == FLAG_DEV_I2C_EXPANDER) {
        unsigned char port_state;
        NTSTATUS err;

        /* NOTE: simply write data register to the expander. It is assumed the pin is pre-configured to output elsewhere (uefi firmware) */
        err = i2c_read(&desc->io_target, desc->expander_data_reg, &port_state, 1);
        if (!NT_SUCCESS(err)) {
            return;
        }
        if (value) {
            port_state |= desc->expander_pin_mask;
        }
        else {
            port_state &= ~(desc->expander_pin_mask);
        }
        (void)i2c_write(&desc->io_target, desc->expander_data_reg, &port_state, 1);
    }
}

unsigned long __bf_shf(unsigned long mask)
{
    unsigned long index;
    unsigned char ret;

    ret = _BitScanForward(&index, mask);
    return (ret ? index : 0);
}

} // extern "C"
