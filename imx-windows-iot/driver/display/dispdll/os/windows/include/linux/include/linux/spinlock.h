/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Modifications Copyright 2022-2023 NXP
 */
#ifndef __LINUX_SPINLOCK_H
#define __LINUX_SPINLOCK_H

/*
 * include/linux/spinlock.h - generic spinlock/rwlock declarations
 *
 *  linux/spinlock_types.h:
 *                        defines the generic type and initializers
 *
 */

#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/lockdep.h>

/*
 * Pull the arch_spinlock_t and arch_rwlock_t definitions:
 */
#include <linux/spinlock_types.h>

/*
* We implement spinlock API as NOP because there is no straightforward
* Windows spinlock API available. Windows uses different mechanism to synchronize
* driver thread with the interrupt thread.
* On Windows side this should be safe, because the next page flipping/mode setting
* request will not come in until the current one is completed.
* This is not meant to be a generic emulation, but rather for our limited scenario.
*/

# define raw_spin_lock_init(lock)

inline void raw_spin_lock_irqsave(raw_spinlock_t *lock,
	unsigned char *flags)
{
}

inline void raw_spin_unlock_irqrestore(raw_spinlock_t *lock,
	unsigned char flags)
{
}

# define spin_lock_init(lock)

static __always_inline void spin_lock(spinlock_t *lock)
{

}

inline void spin_lock_irq(spinlock_t *lock)
{
}

#define spin_lock_irqsave(lock, flags)	\
	flags = 0

static __always_inline void spin_unlock(spinlock_t *lock)
{
}

inline void spin_unlock_irq(spinlock_t *lock)
{
}

inline void spin_unlock_irqrestore(spinlock_t *lock,
	unsigned long flags)
{
}

/* Following API is implemented using
 * KeInitializeSpinLock, KeAcquireSpinLock, KeReleaseSpinLock.
 * API differs from that above on purpose, because spinlocks from linux code
 * may be called in different context (irq), so their usage in display driver
 * must be thoroughly reviewed on a case by case basis:
 * https://learn.microsoft.com/en-us/windows-hardware/drivers/kernel/introduction-to-spin-locks */
void wspin_lock_init(spinlock_t* lock);
void wspin_lock_irqsave(spinlock_t* lock, PKIRQL flags);
void wspin_unlock_irqrestore(spinlock_t* lock, PKIRQL flags);

#endif /* __LINUX_SPINLOCK_H */
