/*
 * Queued read/write locks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * (C) Copyright 2013-2014 Hewlett-Packard Development Company, L.P.
 *
 * Authors: Waiman Long <waiman.long@hp.com>
 */
#include <linux/smp.h>
#include <linux/bug.h>
#include <linux/cpumask.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <linux/spinlock.h>
#include <asm/qrwlock.h>

#ifndef arch_qrwlock_relax
# define arch_qrwlock_relax(lock)	cpu_relax_lowlatency()
#endif

/*
 * This internal data structure is used for optimizing access to some of
 * the subfields within the atomic_t cnts.
 */
struct __qrwlock {
	union {
		atomic_t cnts;
		struct {
#ifdef __LITTLE_ENDIAN
			u8 wmode;	/* Writer mode   */
			u8 rcnts[3];	/* Reader counts */
#else
			u8 rcnts[3];	/* Reader counts */
			u8 wmode;	/* Writer mode   */
#endif
		};
	};
	arch_spinlock_t	lock;
};

/**
 * rspin_until_writer_unlock - spin until writer is gone
 * @lock  : Pointer to queue rwlock structure
 *
 * In interrupt context or at the head of the queue, the reader will just
 * wait until the writer releases the lock.
 */
static __always_inline void
rspin_until_writer_unlock(struct qrwlock *lock)
{
	u32 cnts = smp_load_acquire((u32 *)&lock->cnts);

	while ((cnts & _QW_WMASK) == _QW_LOCKED) {
		arch_qrwlock_relax(lock);
		cnts = smp_load_acquire((u32 *)&lock->cnts);
	}
}

/**
 * queued_read_lock_slowpath - acquire read lock of a queue rwlock
 * @lock: Pointer to queue rwlock structure
 * @cnts: Current qrwlock lock value
 */
void queued_read_lock_slowpath(struct qrwlock *lock, u32 cnts)
{
	/*
	 * Readers come here when they cannot get the lock without waiting
	 */
	if (unlikely(in_interrupt())) {
		/*
		 * Readers in interrupt context will get the lock immediately
		 * if the writer is just waiting (not holding the lock yet).
		 * Otherwise, they will spin until the lock is available
		 * without waiting in the queue.
		 */
		if ((cnts & _QW_WMASK) == _QW_LOCKED)
			rspin_until_writer_unlock(lock);
		return;
	}
	atomic_sub(_QR_BIAS, &lock->cnts);

	/*
	 * Put the reader into the wait queue
	 */
	arch_spin_lock(&lock->lock);

	/*
	 * At the head of the wait queue now, wait until the writer state
	 * goes to 0 and then try to increment the reader count and get
	 * the lock. It is possible that an incoming writer may steal the
	 * lock in the interim, so it is necessary to check the writer byte
	 * to make sure that the write lock isn't taken.
	 */
	while (atomic_read(&lock->cnts) & _QW_WMASK)
		arch_qrwlock_relax(lock);

	atomic_add(_QR_BIAS, &lock->cnts);
	rspin_until_writer_unlock(lock);

	/*
	 * Signal the next one in queue to become queue head
	 */
	arch_spin_unlock(&lock->lock);
}
EXPORT_SYMBOL(queued_read_lock_slowpath);

/**
 * queued_write_lock_slowpath - acquire write lock of a queue rwlock
 * @lock : Pointer to queue rwlock structure
 */
#ifndef cmpxchg_relaxed
# define cmpxchg_relaxed	cmpxchg
#endif

void queued_write_lock_slowpath(struct qrwlock *lock)
{
	u32 cnts;

	/* Put the writer into the wait queue */
	arch_spin_lock(&lock->lock);

	/* Try to acquire the lock directly if no reader is present */
	if (!atomic_read(&lock->cnts) &&
	    (atomic_cmpxchg(&lock->cnts, 0, _QW_LOCKED) == 0))
		goto unlock;

	/*
	 * Set the waiting flag to notify readers that a writer is pending,
	 * or wait for a previous writer to go away.
	 */
	for (;;) {
		struct __qrwlock *l = (struct __qrwlock *)lock;

		if (!READ_ONCE(l->wmode) &&
		   (cmpxchg_relaxed(&l->wmode, 0, _QW_WAITING) == 0))
			break;

		arch_qrwlock_relax(lock);
	}

	/* When no more readers, set the locked flag */
	for (;;) {
		cnts = atomic_read(&lock->cnts);
		if ((cnts == _QW_WAITING) &&
		    (atomic_cmpxchg(&lock->cnts, _QW_WAITING,
				    _QW_LOCKED) == _QW_WAITING))
			break;

		arch_qrwlock_relax(lock);
	}
unlock:
	arch_spin_unlock(&lock->lock);
}
EXPORT_SYMBOL(queued_write_lock_slowpath);
