/*
 * Copyright (C) 2015 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_QRWLOCK_H
#define __ASM_QRWLOCK_H

#define arch_read_lock_flags(lock, flags)	arch_read_lock(lock)
#define arch_write_lock_flags(lock, flags)	arch_write_lock(lock)

/* FIXME: This is racy. Probably need to rethink the arch interface */
#define arch_qrwlock_relax(lock)					\
do {									\
	asm volatile("ldxr	wzr, %0" :: "Q" (*(u32 *)&lock->cnts));	\
	wfe();								\
} while (0)

#include <asm-generic/qrwlock.h>

#endif /* __ASM_QRWLOCK_H */
