/*
 *	All files except if stated otherwise in the beginning of the file
 *	are under the ISC license:
 *	----------------------------------------------------------------------
 *	Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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



#ifndef _IPC_API_H_
#define _IPC_API_H_

#include <linux/list.h>

/*****************************************************************************
 *                      MACROS
 *****************************************************************************
 */

/* Build AgentId from given CpuId and LocalId */
#define IPC_AGENT_ID(cpuid, lid)	\
		(((cpuid&(PLATFORM_MAX_NUM_OF_NODES-1)) << 4) + (0x0f & (lid)))

#define ipc_get_node(id)	((id) >> 4)
#define ipc_lid(id)		((id) & 0x0f)

#define IPC_BUF_SIZE_MAX	1600		/* in bytes */
#define IPC_BUF_COUNT_MAX	128

#define IPC_BUFS_SZ_PER_IF_FIFO	(IPC_BUF_SIZE_MAX * IPC_BUF_COUNT_MAX)

#define IPC_MSG_MAX_LEN	(IPC_BUF_SIZE_MAX - sizeof(struct ipc_msg_hdr))

#define MAX_AGENT_NAME_LEN	32

#ifndef MAX_AGENTS
#define MAX_AGENTS		256
#endif

#define MAX_LOCAL_AGENT		16

/*********************************
 *  Message header flag field
 *********************************
*/
typedef struct ipc_msg_flags __bitwise {
	uint8_t	reserved:6;
	uint8_t	reply:1;		/* Message is replay */
	uint8_t	request:1;		/* Message is a request */
} ipc_msg_flags_t;

/********************************************
 *  Message header
 *
 *  This is the header of the first buffer
 *
 ********************************************
*/
struct ipc_msg_hdr {
	struct ipc_buf_hdr	*next;		/* Points to the next buffer */
						/* if exists. 2 LSB are used */
						/* to identify buffer type */
						/* (full, start, end, mid) */
	uint8_t			request_num;	/* Request sequence number */
	uint8_t			reply_num;	/* Reply sequence number */
	uint16_t		msg_len;	/* Total message length */
						/* (Data part) */
	uint8_t			dest_aid;	/* Message destination */
	uint8_t			src_aid;	/* Message source */
	uint8_t			msg_type;	/* Application message type */
	ipc_msg_flags_t		flags;		/* Message flag (req/rep) */
	char			*reply;		/* Optional replay message */
						/* pointer */
} __packed;

/********************************************
 *  Priority (for transport)
 *
 ********************************************
*/
enum ipc_prio {
	ipc_prio_lo,	/* Low */
	ipc_prio_hi,	/* High */
	max_ipc_prio,
};

static inline bool valid_ipc_prio(uint32_t prio)
{
	return ((prio < max_ipc_prio) ? true : false);
}

struct agent_entry {
	char	name[MAX_AGENT_NAME_LEN];
};

struct ipc_msg_err_stats {
	uint32_t	zlen_msg;
	uint32_t	oversize_msg;
	uint32_t	inval_msg;
	uint32_t	chained_msg;
};

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
 *
 * Returns: Result code
 *
 */
void ipc_msg_free(struct ipc_msg_hdr *hdr, uint32_t b_fifo);

/* ===========================================================================
 * ipc_msg_alloc
 * ===========================================================================
 * Description:	Allocate message buffer[s] and set the type and length.
 *
 *
 * Parameters:		src_aid		- Message source AgentId
 *			dest_aid	- Message destination AgentId
 *			msg_len		- Message length
 *			msg_type	- Message type
 *			b_fifo		- B-FIFO HW FIFO index
 *
 * Returns: Pointer to the message header
 *
 */
struct ipc_msg_hdr *ipc_msg_alloc
(
	uint8_t		src_aid,
	uint8_t		dest_aid,
	size_t		msg_len,
	uint8_t		msg_type,
	uint32_t	b_fifo
);

/* ===========================================================================
 * ipc_msg_send
 * ===========================================================================
 * Description:	Message send, first buffer of the message should be provided,
 *
 * Parameters:		hdr	- Pointer to IPC message header
 *			m_fifo	- M-FIFO HW FIFO index
 *
 * Returns: Result code
 *
 */
int ipc_msg_send(struct ipc_msg_hdr *hdr, uint32_t m_fifo);

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
		   struct ipc_msg_err_stats *stats);

static inline int ipc_copy_from(
	char *dst,
	const char *src,
	size_t size,
	bool user_space)
{
	if (!user_space) {
		memcpy(dst, src, size);
		return 0;
	}
	if (copy_from_user(dst, src, size))
		return -EFAULT;
	return 0;
}

void ipc_agent_table_clean(uint8_t local_cpuid);

extern uint8_t ipc_req_sn;

/* Buffer Management */
struct shm_region {
	phys_addr_t	start;
	phys_addr_t	end;
	void		*vaddr;
	uint32_t	buf_headroom_sz;
	uint32_t	buf_sz;
	uint32_t	buf_num;
	uint32_t	real_buf_sz;
	bool		dir_buf_map;
};

struct shm_bufpool {
	struct list_head	head;
	uint32_t		count;
};

struct shm_buf {
	struct list_head	list;
	struct shm_region	*region;
	struct list_head	*head;
	uint32_t		offset;
};

struct shm_buf *shm_region_find_buf_by_pa(
	struct shm_region *region,
	phys_addr_t phy_addr);

struct shm_buf *shm_find_buf_by_pa(phys_addr_t phy_addr);

struct shm_region *shm_region_create(
	phys_addr_t	start,
	void		*vaddr,
	resource_size_t	size,
	uint32_t	buf_sz,
	uint32_t	buf_headroom,
	uint32_t	buf_num);

void shm_region_release(struct shm_region *region);

void shm_region_release_all(void);

static inline void shm_bufpool_init(struct shm_bufpool *pool)
{
	if (pool) {
		INIT_LIST_HEAD(&pool->head);
		pool->count = 0;
	}
}

void shm_bufpool_release(struct shm_bufpool *pool);

int shm_bufpool_acquire_region(
	struct shm_bufpool *pool,
	struct shm_region *region,
	uint32_t offset,
	uint32_t size);

int shm_bufpool_acquire_whole_region(
	struct shm_bufpool *pool,
	struct shm_region *region);

struct shm_buf *shm_bufpool_get_buf(struct shm_bufpool *pool);

int shm_bufpool_del_buf(struct shm_bufpool *pool, struct shm_buf *buf);

void shm_bufpool_put_buf(struct shm_bufpool *pool, struct shm_buf *buf);

struct shm_buf *shm_bufpool_find_buf_in_region(struct shm_bufpool *pool,
					       struct shm_region *region,
					       phys_addr_t phy_addr);

struct shm_buf *shm_bufpool_find_buf_overlap(struct shm_bufpool *pool,
					     struct shm_region *region,
					     phys_addr_t phy_addr);

static inline phys_addr_t buf_paddr(struct shm_buf *buf)
{
	return (buf->region->start + buf->offset);
}

static inline void *buf_vaddr(struct shm_buf *buf)
{
	return (void *)(buf->region->vaddr + buf->offset);
}

static inline bool address_in_range(
	phys_addr_t addr,
	phys_addr_t start,
	phys_addr_t end)
{
	return (((addr >= start) && (addr < end)) ? true : false);
}

static inline bool address_space_overlap(
	phys_addr_t start_l,
	resource_size_t size_l,
	phys_addr_t start_r,
	resource_size_t size_r)
{
	phys_addr_t end_l = start_l + size_l;
	phys_addr_t end_r = start_r + size_r;

	return ((end_r <= start_l || start_r >= end_l) ? false : true);
}

#endif /*_IPC_API_H_*/
