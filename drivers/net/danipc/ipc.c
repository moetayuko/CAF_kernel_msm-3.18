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

#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/cache.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <asm/cacheflush.h>

#include "ipc_reg.h"
#include "ipc_api.h"

#include "danipc_k.h"
#include "danipc_lowlevel.h"

uint8_t	ipc_req_sn;	/* Maintain node related sequence number */

/* ===========================================================================
 * ipc_buf_free
 * ===========================================================================
 * Description:	Free the buffer, could be called on IPC message receiving node
 *		or on sending node when need to free previously allocated
 *		buffers
 *
 * Parameters:		hdr	- Pointer to IPC message header
 *			b_fifo	- B-FIFO HW FIFO Index
 *
 */
void ipc_msg_free(struct ipc_msg_hdr *hdr, uint32_t b_fifo)
{
	if (hdr)
		danipc_hw_fifo_push(ipc_get_node(hdr->dest_aid), b_fifo, hdr);
}

/* ===========================================================================
 * ipc_msg_alloc
 * ===========================================================================
 * Description:  Allocate message buffer[s] and set the type and length.
 *		Copy message data into allocated buffers.
 *
 *
 * Parameters:		src_aid		- Message source AgentId
 *			dest_aid	- Message destination AgentId
 *			msg_len		- Message length
 *			msg_type	- Message type
 *			b_fifo		- B_FIFO HW FIFO index
 *
 * Returns: Pointer to the message first buffer
 *
 */
struct ipc_msg_hdr *ipc_msg_alloc(
	uint8_t		src_aid,
	uint8_t		dest_aid,
	size_t		msg_len,
	uint8_t		msg_type,
	uint32_t	b_fifo)
{
	struct ipc_msg_hdr *hdr;

	if ((msg_len > IPC_MSG_MAX_LEN) || (msg_len == 0))
		return NULL;

	hdr = danipc_hw_fifo_pop(ipc_get_node(dest_aid), b_fifo);
	if (hdr) {
		hdr->msg_type = msg_type;
		hdr->msg_len = msg_len;
		hdr->reply = NULL;
		hdr->dest_aid = dest_aid;
		hdr->src_aid = src_aid;
		hdr->request_num = ipc_req_sn++;
	}

	return hdr;
}

/* ===========================================================================
 * ipc_msg_send
 * ===========================================================================
 * Description:  Message send, first buffer of the message should be provided,
 *
 * Parameters:		hdr	- Pointer to IPC message header
 *			m_fifo	- M-FIFO HW FIFO index
 *
 * Returns: Result code
 *
 */
int ipc_msg_send(struct ipc_msg_hdr *hdr, uint32_t m_fifo)
{
	if (unlikely(hdr == NULL))
		return -EINVAL;

	/* Flush to DDR entire message */
	/* No data offset in IPC message */
	/* Flush the cache with the length indicated in IPC header */
	dmac_flush_range(hdr, (char *)(hdr + 1) + hdr->msg_len);
	danipc_hw_fifo_push(ipc_get_node(hdr->dest_aid), m_fifo, hdr);

	return 0;
}

/* ===========================================================================
 * ipc_msg_valid
 * ===========================================================================
 * Description:  Validate the IPC message
 *
 * Parameters:	hdr	- Pointer to IPC message header
 *		node	- Receiving Node ID
 *		stats	- Status Pointer
 *
 * Returns: True - The message is valid, otherwise False
 *
 */
bool ipc_msg_valid(const struct ipc_msg_hdr *hdr,
		   uint8_t node,
		   struct ipc_msg_err_stats *stats)
{
	if (hdr->msg_len > IPC_MSG_MAX_LEN) {
		pr_debug("%s: receive the message with bad message len(%u)\n",
			 __func__, hdr->msg_len);
		stats->oversize_msg++;
		return false;
	}
	if (!hdr->msg_len) {
		stats->zlen_msg++;
		return false;
	}
	if (ipc_get_node(hdr->dest_aid) != node) {
		pr_debug("%s: receive the message with bad dest_aid(%u)\n",
			 __func__, hdr->dest_aid);
		stats->inval_msg++;
		return false;
	}
	/* no buffer chain */
	if (hdr->next != NULL) {
		pr_debug("%s: receive the message with next point(%p)\n",
			 __func__, hdr->next);
		stats->chained_msg++;
		return false;
	}
	return true;
}

void ipc_agent_table_clean(uint8_t cpuid)
{
	unsigned aid = __IPC_AGENT_ID(cpuid, 0);
	struct agent_entry *entry = (struct agent_entry *)
		danipc_driver.res[AGENT_TABLE_RES].base;

	memset(entry+aid, 0, (sizeof(struct agent_entry) * MAX_LOCAL_AGENT));
}

#define MAX_SHM_REGION_NUM	4

#define CACHELINE_ALIGNED(a)	(!((a) & (cache_line_size() - 1)))
#define ALIGN_BUF(a, s)		((a) - (a)%(s))

struct shm_region_tbl {
	struct shm_region	*region[MAX_SHM_REGION_NUM];
	uint32_t		num_region;
};

static struct shm_region_tbl region_tbl;

static inline struct shm_buf *buf_in_region(struct shm_region *region)
{
	return (struct shm_buf *)(region+1);
}

static inline void __shm_bufpool_del_buf(struct shm_bufpool *pool,
					 struct shm_buf *buf)
{
	list_del_init(&buf->list);
	buf->head = NULL;
	pool->count--;
}

static inline void __shm_bufpool_add_buf(struct shm_bufpool *pool,
					 struct shm_buf *buf)
{
	list_add_tail(&buf->list, &pool->head);
	buf->head = &pool->head;
	pool->count++;
}

static inline struct shm_buf *find_dir_map_buf_by_offset(
	struct shm_region *region,
	uint32_t offset)
{
	uint32_t index = offset/region->real_buf_sz;

	if (index < region->buf_num)
		return buf_in_region(region)+index;
	return NULL;
}

struct shm_buf *shm_region_find_buf_by_pa(
	struct shm_region *region,
	phys_addr_t phy_addr)
{
	if (!region->dir_buf_map)
		return NULL;

	return find_dir_map_buf_by_offset(region, phy_addr - region->start);
}

struct shm_buf *shm_find_buf_by_pa(phys_addr_t phy_addr)
{
	struct shm_buf *buf = NULL;
	int i;
	int n = 0;

	for (i = 0; i < MAX_SHM_REGION_NUM && n < region_tbl.num_region; i++) {
		if (!region_tbl.region[i])
			continue;
		if (address_in_range(phy_addr,
				     region_tbl.region[i]->start,
				     region_tbl.region[i]->end)) {
			buf = shm_region_find_buf_by_pa(region_tbl.region[i],
							phy_addr);
			break;
		}
		n++;
	}
	return buf;
}

struct shm_region *shm_region_create(
	phys_addr_t	start,
	void		*vaddr,
	resource_size_t	size,
	uint32_t	buf_sz,
	uint32_t	buf_headroom,
	uint32_t	buf_num)
{
	struct shm_region_tbl *tbl = &region_tbl;
	struct shm_region *region;
	struct shm_buf *buf;
	phys_addr_t end = start+size;
	uint32_t real_buf_sz = buf_sz + buf_headroom;
	uint32_t n;
	int i, idx;
	bool dir_map = true;

	if (unlikely(!size || !buf_num || !buf_sz))
		return NULL;

	if (!CACHELINE_ALIGNED(start) || !CACHELINE_ALIGNED(real_buf_sz)) {
		pr_err("%s: %x/%u not cacheline aligned\n",
		       __func__, start, real_buf_sz);
		return NULL;
	}

	if (tbl->num_region >= MAX_SHM_REGION_NUM) {
		pr_err("%s: number of shm_region exceeds the limit\n",
		       __func__);
		return NULL;
	}

	for (i = 0, idx = -1; i < MAX_SHM_REGION_NUM; i++) {
		region = tbl->region[i];
		if (region == NULL) {
			if (idx < 0)
				idx = i;
			continue;
		}
		if (address_in_range(start, region->start, region->end) ||
		    address_in_range(end, region->start, region->end)) {
			pr_err("%s: region(%x/%x) overlay with %x/%x",
			       __func__, start, end,
			       region->start, region->end);
			return NULL;
		}
	}

	if (idx < 0)
		return NULL;

	n = size/real_buf_sz;
	if (buf_num < n) {
		n = buf_num;
		dir_map = false;
	}

	region = kzalloc((sizeof(struct shm_region) +
			  sizeof(struct shm_buf) * n),
			 GFP_KERNEL);
	if (IS_ERR(region)) {
		pr_err("%s: failed to alloc the region data\n", __func__);
		return NULL;
	}

	region->start = start;
	region->vaddr = vaddr;
	region->end = end;
	region->buf_sz = buf_sz;
	region->buf_num = n;
	region->buf_headroom_sz = buf_headroom;
	region->real_buf_sz = real_buf_sz;
	region->dir_buf_map = dir_map;

	buf = (struct shm_buf *)(region + 1);
	for (i = 0; i < region->buf_num; i++, buf++) {
		buf->region = region;
		buf->head = NULL;
		INIT_LIST_HEAD(&buf->list);

		if (dir_map)
			buf->offset = i * real_buf_sz + buf_headroom;
	}

	tbl->region[idx] = region;
	tbl->num_region++;
	return region;
}

void shm_region_release(struct shm_region *region)
{
	struct shm_region_tbl *tbl = &region_tbl;
	int i;

	if (unlikely(!region))
		return;

	for (i = 0; i < MAX_SHM_REGION_NUM; i++) {
		if (tbl->region[i] == region) {
			kfree(region);
			tbl->region[i] = NULL;
			tbl->num_region--;
			break;
		}
	}
}

void shm_region_release_all(void)
{
	struct shm_region_tbl *tbl = &region_tbl;
	int i;

	for (i = 0; i < MAX_SHM_REGION_NUM; i++) {
		kfree(tbl->region[i]);
		tbl->region[i] = NULL;
	}
	tbl->num_region = 0;
}

int shm_bufpool_acquire_region(
	struct shm_bufpool	*pool,
	struct shm_region	*region,
	uint32_t		offset,
	uint32_t		size)
{
	struct shm_buf *buf;
	phys_addr_t start, end;
	uint32_t nbuf;
	int i;

	if (unlikely(!pool || !region || !region->buf_num))
		return -EINVAL;

	if (!region->dir_buf_map)
		return -EPERM;

	start = region->start + ALIGN_BUF((offset + region->real_buf_sz - 1),
					  region->real_buf_sz);

	if (!address_in_range(start, region->start, region->end)) {
		pr_err("%s: invalid offset(0x%x)\n", __func__, offset);
		return -EINVAL;
	}

	end = region->start + ALIGN_BUF((offset + size), region->real_buf_sz);
	if (end > region->end) {
		pr_err("%s: size(%u) is too big\n", __func__, size);
		return -EINVAL;
	}

	nbuf = (end - start)/region->real_buf_sz;
	if (!nbuf) {
		pr_err("%s: size(%u) is too small\n", __func__, size);
		return -EINVAL;
	}

	buf = shm_region_find_buf_by_pa(region, start);
	BUG_ON(buf == NULL);

	for (i = 0; i < nbuf; i++) {
		if (buf[i].head != NULL) {
			pr_err("%s: can't claim the buffer at 0x%x\n",
			       __func__, buf_paddr(buf+i));
			return -EINVAL;
		}
	}

	for (i = 0; i < nbuf; i++, buf++)
		__shm_bufpool_add_buf(pool, buf);

	return 0;
}

int shm_bufpool_acquire_whole_region(
	struct shm_bufpool *pool,
	struct shm_region *region)
{
	struct shm_buf *buf;
	int i;

	if (unlikely(!pool || !region))
		return -EINVAL;

	buf = buf_in_region(region);
	for (i = 0; i < region->buf_num; i++) {
		if (buf[i].head != NULL) {
			pr_err("%s: can't claim the buffer, buffer_idx=%d\n",
			       __func__, i);
			return -EINVAL;
		}
	}

	for (i = 0; i < region->buf_num; i++, buf++)
		__shm_bufpool_add_buf(pool, buf);

	return 0;
}

void shm_bufpool_release(struct shm_bufpool *pool)
{
	struct shm_buf *buf, *p;

	if (unlikely(pool == NULL))
		return;

	list_for_each_entry_safe(buf, p, &pool->head, list)
		__shm_bufpool_del_buf(pool, buf);
}

struct shm_buf *shm_bufpool_get_buf(struct shm_bufpool *pool)
{
	struct shm_buf *buf;

	if (unlikely(!pool))
		return NULL;

	buf = list_first_entry_or_null(&pool->head, struct shm_buf, list);
	if (buf) {
		__shm_bufpool_del_buf(pool, buf);
		pr_debug("%s: get buf %p from pool %p, pool_count=%u\n",
			 __func__, buf, pool, pool->count);
	}
	return buf;
}

void shm_bufpool_put_buf(struct shm_bufpool *pool, struct shm_buf *buf)
{
	if (unlikely(!pool || !buf))
		return;

	__shm_bufpool_add_buf(pool, buf);
	pr_debug("%s: put buf %p into pool %p, pool_count=%u\n",
		 __func__, buf, pool, pool->count);
}

int shm_bufpool_del_buf(struct shm_bufpool *pool, struct shm_buf *buf)
{
	if (unlikely(!pool || !buf))
		return -EINVAL;

	if (unlikely(buf->head != &pool->head)) {
		pr_debug("%s: unable to del buf, pool(%p) head %p\n",
			 __func__, pool, buf->head);
		return -EINVAL;
	}
	__shm_bufpool_del_buf(pool, buf);
	return 0;
}

struct shm_buf *shm_bufpool_find_buf_in_region(struct shm_bufpool *pool,
					       struct shm_region *region,
					       phys_addr_t phy_addr)
{
	struct shm_buf *buf;
	uint32_t offset;

	if (unlikely(!pool || !region))
		return NULL;

	offset = phy_addr - region->start;
	if (region->dir_buf_map) {
		buf = find_dir_map_buf_by_offset(region, offset);
		if (buf && buf->head != &pool->head)
			buf = NULL;
		return buf;
	}

	list_for_each_entry(buf, &pool->head, list) {
		if (buf->region != region)
			continue;
		if (address_in_range(offset, buf->offset,
				     buf->offset+buf->region->real_buf_sz))
			return buf;
	}
	return NULL;
}

struct shm_buf *shm_bufpool_find_buf_overlap(struct shm_bufpool *pool,
					     struct shm_region *region,
					     phys_addr_t phy_addr)
{
	struct shm_buf *buf;
	uint32_t offset;

	if (unlikely(!pool || !region))
		return NULL;

	offset = phy_addr - region->start;
	if (region->dir_buf_map) {
		buf = find_dir_map_buf_by_offset(region, offset);
		if (buf && buf->head != &pool->head)
			buf = NULL;
		return buf;
	}

	list_for_each_entry(buf, &pool->head, list) {
		if (buf->region != region)
			continue;
		if (address_space_overlap(offset,
					  region->real_buf_sz,
					  buf->offset,
					  region->real_buf_sz))
			return buf;
	}
	return NULL;
}
