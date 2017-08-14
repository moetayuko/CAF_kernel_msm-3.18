/*
 *	All files except if stated otherwise in the beginning of the file
 *	are under the ISC license:
 *	----------------------------------------------------------------------
 *	Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
 *	Copyright (c) 2010-2012 Design Art Networks Ltd.
 *
 *	Permission to use, copy, modify, and/or distribute this software for any
 *	purpose with or without fee is hereby granted, provided that the above
 *	copyright notice and this permission notice appear in all copies.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#ifndef __DANIPC_LOWLEVEL_H__
#define __DANIPC_LOWLEVEL_H__

#include <linux/irqflags.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <asm/cacheflush.h>

#include <ipc_api.h>

#define PLATFORM_MAX_NUM_OF_NODES 16

#define DEFAULT_HI_PRIO_M_FIFO	0
#define DEFAULT_HI_PRIO_B_FIFO	1
#define DEFAULT_LO_PRIO_M_FIFO	2
#define DEFAULT_LO_PRIO_B_FIFO	3

#define default_m_fifo(pri)	(((pri) == ipc_prio_hi) ? \
	DEFAULT_HI_PRIO_M_FIFO : DEFAULT_LO_PRIO_M_FIFO)

#define default_b_fifo(pri)	(((pri) == ipc_prio_hi) ? \
	DEFAULT_HI_PRIO_B_FIFO : DEFAULT_LO_PRIO_B_FIFO)

#define MAX_IPC_FIFO_PER_CDU	4
#define CLK_DOMAIN_PER_CDU	2

#define FIFO_RD_ACCESS_0_OFFSET	0x8
#define FIFO_RD_OFFSET(n)	(FIFO_RD_ACCESS_0_OFFSET + (n) * 8)

#define FIFO_WR_ACCESS_0_OFFSET	0x4
#define FIFO_WR_OFFSET(n)	(FIFO_WR_ACCESS_0_OFFSET + (n) * 8)

/* Status register offset */
#define FIFO_0_STATUS		0x24
#define FIFO_STATUS(n)		(FIFO_0_STATUS + (n) * 4)
#define IPC_FIFO_EMPTY	1
#define IPC_FIFO_FULL	0x10

/* Almost Full(AF) interrupt threshold */
#define AF_THRESHOLD		0x7D

/* IPC FIFO interrupt register offsets */
#define FIFO_0_COUNTER		0x6c
#define FIFO_COUNTER(n)		(FIFO_0_COUNTER + (n) * 4)

#define FIFO_THR_AF_CFG		0x34
#define FIFO_THR_AE_CFG		0x38

#define CDU_INT0_MASK		0x44
#define CDU_INT1_MASK		0x48
#define CDU_INT0_ENABLE		0x4c
#define CDU_INT1_ENABLE		0x50
#define CDU_INT0_STATUS		0x54
#define CDU_INT1_STATUS		0x58
#define CDU_INT0_RAW_STATUS	0x5C
#define CDU_INT1_RAW_STATUS	0x60
#define CDU_INT0_CLEAR		0x64
#define CDU_INT1_CLEAR		0x68

#define FIFO_0_POP_COUNTER	0x6C
#define FIFO_POP_COUNTER_ENABLE 0x80

/* Pop counter set FIFO'd*/
#define FIFO_POP_SET_ALL_FIFOS 0x15

#define FIFO_SHIFT(n)		((n) * 8)

/* Almost Full(AF) interrupt indication bitmask */
#define IPC_INT_FIFO_AF_0	0x20
#define IPC_INT_FIFO_AF(n)	(IPC_INT_FIFO_AF_0 << FIFO_SHIFT(n))
#define IPC_INT_FIFO_MASK	0xff

/* AF threshold bitmap for a FIFO */
#define IPC_AF_THR_FIFO_MASK	0x7f
#define IPC_AF_THR_FIFO(v, n)	(((v) & IPC_AF_THR_FIFO_MASK) << FIFO_SHIFT(n))

/* AF threshold bitmap for a FIFO */
#define IPC_AE_THR_FIFO_MASK	0x7f
#define IPC_AE_THR_FIFO(v, n)	(((v) & IPC_AE_THR_FIFO_MASK) << FIFO_SHIFT(n))

struct danipc_fifo;
struct danipc_if_fifo;

#define __IPC_AGENT_ID(cpuid, lid)			\
	(((cpuid&(PLATFORM_MAX_NUM_OF_NODES-1)) << 4) +	\
				(0x0f & (lid)))

void *ipc_to_virt(uint32_t cpuid, phys_addr_t addr);

static inline bool valid_cpu_id(int cpuid)
{
	if (cpuid < 0 || cpuid >= PLATFORM_MAX_NUM_OF_NODES)
		return false;
	return true;
}

static inline bool valid_fifo_unit(uint32_t unit)
{
	return (unit < MAX_IPC_FIFO_PER_CDU) ? true : false;
}

/* inline API to access the HW */
static inline uint32_t danipc_hw_reg_read(void __iomem *base, uint32_t offset)
{
	return __raw_readl_no_log((char *)base + offset);
}

static inline void danipc_hw_reg_write(void __iomem *base,
				       uint32_t offset,
				       uint32_t val)
{
	__raw_writel_no_log(val, (char *)base + offset);
}

static inline void danipc_hw_reg_update(void __iomem *base,
					uint32_t offset,
					uint32_t val,
					uint32_t mask)
{
	uint32_t v = danipc_hw_reg_read(base, offset);

	v &= ~mask;
	v |= (val & mask);
	danipc_hw_reg_write(base, offset, v);
}

void danipc_fifo_init_irq(struct danipc_fifo *fifo);
void danipc_fifo_disable_irq(struct danipc_fifo *fifo);
void danipc_fifo_clear_interrupt(struct danipc_fifo *fifo);
void danipc_fifo_mask_interrupt(struct danipc_fifo *fifo);
void danipc_fifo_unmask_interrupt(struct danipc_fifo *fifo);

void danipc_if_fifo_init_irq(struct danipc_if_fifo *if_fifo);
void danipc_if_fifo_disable_irq(struct danipc_if_fifo *if_fifo);
void danipc_if_fifo_clear_interrupt(struct danipc_if_fifo *if_fifo);
void danipc_if_fifo_mask_interrupt(struct danipc_if_fifo *if_fifo);
void danipc_if_fifo_unmask_interrupt(struct danipc_if_fifo *if_fifo);

static inline void ipc_msg_payload_cache_invalid(struct ipc_msg_hdr *hdr)
{
	dmac_inv_range((char *)(hdr + 1),
		       (char *)(hdr + 1) + hdr->msg_len);
}

static inline void ipc_msg_hdr_cache_invalid(struct ipc_msg_hdr *hdr)
{
	dmac_inv_range(hdr, hdr + 1);
}

static inline void ipc_msg_cache_flush(struct ipc_msg_hdr *hdr)
{
	dmac_flush_range(hdr, (char *)(hdr + 1) + hdr->msg_len);
}

#endif /* __DANIPC_LOWLEVEL_H__ */
