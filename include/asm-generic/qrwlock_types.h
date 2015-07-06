#ifndef __ASM_GENERIC_QRWLOCK_TYPES_H
#define __ASM_GENERIC_QRWLOCK_TYPES_H

#include <linux/types.h>
#include <asm/byteorder.h>
#include <asm/spinlock_types.h>

/*
 * The queue read/write lock data structure
 *
 * The 32-bit count is divided into an 8-bit writer mode byte (least
 * significant) and a 24-bit reader count.
 */

typedef struct qrwlock {
	union {
		atomic_t	cnts;
		struct {
#ifdef __LITTLE_ENDIAN
			u8	wmode;		/* Writer mode   */
			u8	rcnts[3];	/* Reader counts */
#else
			u8	rcnts[3];	/* Reader counts */
			u8	wmode;		/* Writer mode   */
#endif
		};
	};
	arch_spinlock_t		lock;
} arch_rwlock_t;

#define	__ARCH_RW_LOCK_UNLOCKED {		\
	.cnts = ATOMIC_INIT(0),			\
	.lock = __ARCH_SPIN_LOCK_UNLOCKED,	\
}

#endif /* __ASM_GENERIC_QRWLOCK_TYPES_H */
