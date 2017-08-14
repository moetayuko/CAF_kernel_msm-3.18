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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/irq.h>

#include "danipc_k.h"

#include "ipc_api.h"

#include "danipc_lowlevel.h"


/* IPC and Linux coexistence.
 * IPC uses/needs physical addresses with bit 31 set while Linux obviously
 * uses virtual addresses. So when writing an address to IPC / reading from IPC
 * make sure it is converted from virtual to IPC address
 * (physical address with bit 31 set) and vice versa.
 * For every cpuid (except my own) FIFO buffers of both priorities remapped.
 * It is guaranteed (?) that FIFO buffers are in contiguous memory of 16kB long.
 * So remapping of 2*16kB is a safe way to access all possible FIFO buffers.
 * For my own CPU just take physical address.
 * Vladik, 21.08.2011
 */

#define FIFO_MAP_SIZE		SZ_256K
#define FIFO_MAP_MASK		(FIFO_MAP_SIZE - 1)

static void danipc_resource_remap(struct danipc_resource *r)
{
	r->base = ioremap_nocache(r->start, r->size);
	BUG_ON(r->base == NULL);
}

static void danipc_resource_remap_cached(struct danipc_resource *r)
{
	r->base = ioremap_cache(r->start, r->size);
	BUG_ON(r->base == NULL);
}

static void danipc_resource_unmap(struct danipc_resource *r)
{
	if (r->base) {
		iounmap(r->base);
		r->base = NULL;
	}
}

static const struct ipc_buf_desc *danipc_find_buf_desc(phys_addr_t addr)
{
	const struct ipc_buf_desc *desc = danipc_driver.region_desc;
	int n;

	for (n = 0; n < danipc_driver.num_region_desc; n++, desc++) {
		if ((addr >= desc->phy_addr) &&
		    (addr < (desc->phy_addr + desc->sz)))
			return desc;
	}
	return NULL;
}

static void ipc_to_virt_remap(const int cpuid, const uint32_t paddr)
{
	struct danipc_resource *res = &danipc_driver.shm_res[cpuid];
	const struct ipc_buf_desc *desc;
	uint32_t mask;

	/* If mem_map was not defined in DT, this will return NULL */
	desc = danipc_find_buf_desc(paddr);

	if (desc) {
		res->start = desc->phy_addr;
		res->size = desc->sz;
	} else if (res->size) {
		mask = res->size - 1;
		res->start = ((paddr + mask) & ~mask) - res->size;
	} else {
		res->size = FIFO_MAP_SIZE;
		res->start = ((paddr + FIFO_MAP_MASK) & ~FIFO_MAP_MASK) -
			2 * FIFO_MAP_SIZE;
	}

	danipc_resource_remap_cached(res);
}

void *ipc_to_virt(uint32_t cpuid, phys_addr_t addr)
{
	char *vaddr = NULL;

	if (likely(cpuid < PLATFORM_MAX_NUM_OF_NODES)) {
		struct danipc_resource *res = &danipc_driver.shm_res[cpuid];
		unsigned offset;

		if (unlikely(!res->base))
			ipc_to_virt_remap(cpuid, addr);
		offset = addr - res->start;
		if (likely(offset < res->size))
			vaddr = (char *)res->base + offset;
	}

	BUG_ON(!vaddr);

	return vaddr;
}

void danipc_if_fifo_clear_interrupt(struct danipc_if_fifo *if_fifo)
{
	struct danipc_fifo *fifo = if_fifo->fifo;

	danipc_hw_reg_update(fifo->io_base,
			     (fifo->probe_info->irq_clk_domain) ?
			     CDU_INT1_CLEAR : CDU_INT0_CLEAR,
			     if_fifo->irq_mask,
			     (IPC_INT_FIFO_MASK <<
			      FIFO_SHIFT(if_fifo->m_fifo_idx)));
}

void danipc_if_fifo_mask_interrupt(struct danipc_if_fifo *if_fifo)
{
	struct danipc_fifo *fifo = if_fifo->fifo;

	danipc_hw_reg_update(fifo->io_base,
			     (fifo->probe_info->irq_clk_domain) ?
			     CDU_INT1_MASK : CDU_INT0_MASK,
			     if_fifo->irq_mask,
			     if_fifo->irq_mask);
}

void danipc_if_fifo_unmask_interrupt(struct danipc_if_fifo *if_fifo)
{
	struct danipc_fifo *fifo = if_fifo->fifo;

	danipc_hw_reg_update(fifo->io_base,
			     (fifo->probe_info->irq_clk_domain) ?
			     CDU_INT1_MASK : CDU_INT0_MASK,
			     ~if_fifo->irq_mask,
			     (IPC_INT_FIFO_MASK <<
			      FIFO_SHIFT(if_fifo->m_fifo_idx)));
}

void danipc_if_fifo_init_irq(struct danipc_if_fifo *if_fifo)
{
	struct danipc_fifo *fifo = if_fifo->fifo;

	danipc_hw_reg_update(fifo->io_base,
			     FIFO_THR_AF_CFG,
			     IPC_AF_THR_FIFO(AF_THRESHOLD, if_fifo->m_fifo_idx),
			     IPC_AF_THR_FIFO(IPC_AF_THR_FIFO_MASK,
					     if_fifo->m_fifo_idx));

	danipc_hw_reg_write(fifo->io_base,
			    FIFO_POP_COUNTER_ENABLE,
			    FIFO_POP_SET_ALL_FIFOS);

	danipc_if_fifo_clear_interrupt(if_fifo);

	danipc_hw_reg_update(fifo->io_base,
			     (fifo->probe_info->irq_clk_domain) ?
			     CDU_INT1_ENABLE : CDU_INT0_ENABLE,
			     if_fifo->irq_mask,
			     (IPC_INT_FIFO_MASK <<
			      FIFO_SHIFT(if_fifo->m_fifo_idx)));

	danipc_if_fifo_unmask_interrupt(if_fifo);

	/* Route interrupts from TCSR to APPS (relevant to APPS-FIFO) */
	/* TBD: makesure apps_ipc_mux is incremented by 4 bytes */
	danipc_hw_reg_write(danipc_res_base(KRAIT_IPC_MUX_RES),
			    fifo->idx * sizeof(uint32_t),
			    fifo->probe_info->mux_enable);
}

void danipc_if_fifo_disable_irq(struct danipc_if_fifo *if_fifo)
{
	struct danipc_fifo *fifo = if_fifo->fifo;

	/* Clear, disable and mask all interrupts from this CDU */
	danipc_if_fifo_clear_interrupt(if_fifo);

	danipc_hw_reg_update(fifo->io_base,
			     (fifo->probe_info->irq_clk_domain) ?
			     CDU_INT1_ENABLE : CDU_INT0_ENABLE,
			     0,
			     (IPC_INT_FIFO_MASK <<
			      FIFO_SHIFT(if_fifo->m_fifo_idx)));

	danipc_if_fifo_mask_interrupt(if_fifo);

	/* Route interrupts from TCSR to APPS (relevant to APPS-FIFO) */
	/* TBD: makesure apps_ipc_mux is incremented by 4 bytes */
	danipc_hw_reg_write(danipc_res_base(KRAIT_IPC_MUX_RES),
			    fifo->idx * sizeof(uint32_t),
			    0);
}

void danipc_fifo_init_irq(struct danipc_fifo *fifo)
{
	int pri;

	for (pri = 0; pri < max_ipc_prio; pri++) {
		struct danipc_if_fifo *if_fifo = &fifo->if_fifo[pri];

		if (if_fifo->probed)
			danipc_if_fifo_init_irq(if_fifo);
	}
}

void danipc_fifo_disable_irq(struct danipc_fifo *fifo)
{
	int pri;

	for (pri = 0; pri < max_ipc_prio; pri++) {
		struct danipc_if_fifo *if_fifo = &fifo->if_fifo[pri];

		if (if_fifo->probed)
			danipc_if_fifo_disable_irq(if_fifo);
	}
}

void danipc_fifo_clear_interrupt(struct danipc_fifo *fifo)
{
	int pri;

	for (pri = 0; pri < max_ipc_prio; pri++) {
		struct danipc_if_fifo *if_fifo = &fifo->if_fifo[pri];

		if (if_fifo->probed)
			danipc_if_fifo_clear_interrupt(if_fifo);
	}
}

void danipc_fifo_mask_interrupt(struct danipc_fifo *fifo)
{
	int pri;

	for (pri = 0; pri < max_ipc_prio; pri++) {
		struct danipc_if_fifo *if_fifo = &fifo->if_fifo[pri];

		if (if_fifo->probed)
			danipc_if_fifo_mask_interrupt(if_fifo);
	}
}

void danipc_fifo_unmask_interrupt(struct danipc_fifo *fifo)
{
	int pri;

	for (pri = 0; pri < max_ipc_prio; pri++) {
		struct danipc_if_fifo *if_fifo = &fifo->if_fifo[pri];

		if (if_fifo->probed)
			danipc_if_fifo_unmask_interrupt(if_fifo);
	}
}

void danipc_ll_init(struct danipc_drvr *drv)
{
	int n;

	danipc_resource_remap(&drv->res[AGENT_TABLE_RES]);
	danipc_resource_remap(&drv->res[KRAIT_IPC_MUX_RES]);
	danipc_resource_remap_cached(&drv->res[IPC_BUFS_RES]);

	for (n = 0; n < PLATFORM_MAX_NUM_OF_NODES; n++)
		if (drv->io_res[n].size)
			danipc_resource_remap(&drv->io_res[n]);

	memset(drv->res[AGENT_TABLE_RES].base,
	       0,
	       drv->res[AGENT_TABLE_RES].size);
}

void danipc_ll_cleanup(struct danipc_drvr *drv)
{
	int n;

	for (n = 0; n < PLATFORM_MAX_NUM_OF_NODES; n++) {
		danipc_resource_unmap(&drv->io_res[n]);
		danipc_resource_unmap(&drv->shm_res[n]);
	}

	danipc_resource_unmap(&drv->res[KRAIT_IPC_MUX_RES]);
	danipc_resource_unmap(&drv->res[AGENT_TABLE_RES]);
	danipc_resource_unmap(&drv->res[IPC_BUFS_RES]);
}
