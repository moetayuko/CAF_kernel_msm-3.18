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

#ifndef __DANIPC_H__
#define __DANIPC_H__

#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/bitmap.h>

#include <linux/danipc_ioctl.h>
#include <ipc_api.h>
#include <danipc_lowlevel.h>

enum {
	IPC_BUFS_RES = 0,
	AGENT_TABLE_RES,
	KRAIT_IPC_MUX_RES,
	RESOURCE_NUM
};

#define DANIPC_PROTOCOL_MATCH(p) (((p) & htons(0xf000)) == htons(COOKIE_BASE))
#define DANIPC_AGENT_DISCOVERED(aid, list) \
	(((uint8_t *)(list))[(aid) >> 3] |= 1 << ((aid)&(BITS_PER_BYTE - 1)))
#define DANIPC_IS_AGENT_DISCOVERED(aid, list) \
	(((uint8_t *)(list))[(aid) >> 3] & (1 << ((aid)&(BITS_PER_BYTE - 1))))

/* debugfs handler info */
struct dbgfs_hdlr {
	void (*display)(struct seq_file *);
	ssize_t (*write)(struct file *, const char __user*, size_t, loff_t *);
	void *data;
};

/* debugfs info */
struct danipc_dbgfs {
	const char *fname;
	umode_t mode;
	struct dbgfs_hdlr dbghdlr;
};

struct danipc_dbgfs_dent {
	struct danipc_dbgfs	*dbgfs;
	struct dentry		*dent;
};

#define DBGFS_NODE(n, m, read_op, write_op) {	\
	.fname = n,				\
	.mode = m,				\
	{					\
		.display = read_op,		\
		.write = write_op,		\
	},					\
}

#define DBGFS_NODE_LAST	{	\
	.fname = NULL,		\
}

struct danipc_probe_info {
	const char	*res_name;
	const char	*ifname;
	const int	hi_prio_m_fifo;
	const int	hi_prio_b_fifo;
	const int	lo_prio_m_fifo;
	const int	lo_prio_b_fifo;
	const int	irq_clk_domain;
	const uint32_t	mux_enable;
};

struct danipc_resource {
	resource_size_t	start;
	resource_size_t	size;
	void __iomem	*base;
};

#define DANIPC_MAX_LFIFO	4
#define DANIPC_FIFO_F_INUSE	1

/* IPC RX-Bound FIFO structure */
struct danipc_if_fifo {
	struct danipc_fifo	*fifo;

	uint32_t	b_fifo_idx;	/* M-FIFO HW FIFO index*/
	uint32_t	m_fifo_idx;	/* B-FIFO HW FIFO index */
	uint32_t	irq_mask;
	uint32_t	unit_num;
	bool		probed;
};

/* IPC FIFO software structure */
struct danipc_fifo {
	struct mutex			lock;		/* lock for fifo */
	struct danipc_probe_info	*probe_info;
	void __iomem			*io_base;	/* IPC register base */

	void				*owner;
	uint32_t			irq;
	uint8_t				node_id;
	uint8_t				idx;
	uint32_t			flag;

	struct danipc_if_fifo		if_fifo[max_ipc_prio];

	struct danipc_dbgfs_dent	dent;
};

struct danipc_poll_ctl {
	struct tasklet_struct	task;
	struct hrtimer		timer;
	ktime_t			timer_intval;
	bool			sched;

	uint64_t		poll_cnt;
};

struct danipc_netif_fifo_status {
	uint32_t	rx;
	uint32_t	rx_bytes;
	uint32_t	rx_drop;
	uint32_t	rx_error;
	uint32_t	rx_no_skb;
	uint32_t	rx_ptr;
	uint32_t	rx_ptr_inv_len;
	uint32_t	rx_ptr_inv_addr;

	struct ipc_msg_err_stats	rx_err_msg;


	uint32_t	tx;
	uint32_t	tx_bytes;
	uint32_t	tx_drop;
	uint32_t	tx_unknown_agent;
	uint32_t	tx_bad_proto;
	uint32_t	tx_no_buf;

};

/* Danipc net-device interface FIFO */
struct danipc_netif_fifo {
	struct danipc_if		*intf;
	struct danipc_if_fifo		*if_fifo;

	enum ipc_prio			prio;

	/* Debug fs node */
	struct danipc_dbgfs_dent	dent;

	struct danipc_netif_fifo_status	status;
};

/* Danipc interface specific info */
struct danipc_if {
	struct danipc_drvr	*drvr;
	struct net_device	*dev;
	struct danipc_fifo	*fifo;
	struct mutex		lock;	/* lock for net device interface */

	struct danipc_poll_ctl		poll_ctl;
	struct danipc_netif_fifo	intf_fifo[max_ipc_prio];

	uint8_t			ifidx;

	struct timer_list	tx_timer;
	struct sk_buff		*tx_skb;
	uint32_t		tx_queue_stop;
	uint32_t		tx_queue_restart;

	/* Debug fs node */
	struct danipc_dbgfs_dent	dent;
};

#define DANIPC_MAX_IF		DANIPC_MAX_LFIFO

/* Character device interface */
#define DANIPC_MAJOR		100
#define DANIPC_CDEV_NAME	"danipc"
#define DANIPC_MAX_CDEV		1

#define DEFAULT_POLL_INTERVAL_IN_US	500

struct rx_queue_status {
	uint32_t	recvq_hi;
	uint32_t	freeq_lo;
	uint32_t	bq_lo;
};

struct rx_queue {
	struct shm_bufpool	recvq;
	struct shm_bufpool	freeq;
	struct shm_bufpool	bq;
	struct shm_bufpool	mmapq;
	struct rx_queue_status	status;
};

struct tx_queue {
	struct shm_bufpool	mmapq;
	struct shm_bufpool	mmap_bufcacheq;
};

struct danipc_cdev_if_fifo {
	struct danipc_cdev		*cdev;
	struct danipc_if_fifo		*if_fifo;
	struct rx_queue			rx_queue;
	enum ipc_prio			prio;

	/* Debug fs node */
	struct danipc_dbgfs_dent	dent;
};

struct danipc_cdev_status {
	uint32_t	rx;
	uint32_t	rx_bytes;
	uint32_t	rx_drop;
	uint32_t	rx_no_buf;
	uint32_t	rx_error;

	struct ipc_msg_err_stats	rx_err_stats;

	uint32_t	mmap_rx;
	uint32_t	mmap_rx_done;
	uint32_t	mmap_rx_error;

	uint32_t	tx;
	uint32_t	tx_bytes;
	uint32_t	tx_drop;
	uint32_t	tx_error;
	uint32_t	tx_no_buf;

	uint32_t	mmap_tx;
	uint32_t	mmap_tx_reqbuf;
	uint32_t	mmap_tx_reqbuf_error;
	uint32_t	mmap_tx_nobuf;
	uint32_t	mmap_tx_error;
	uint32_t	mmap_tx_bad_buf;
};

struct danipc_cdev {
	struct danipc_drvr	*drvr;
	struct device		*dev;
	struct danipc_fifo	*fifo;

	struct shm_region	*rx_region;
	struct vm_area_struct	*rx_vma;
	atomic_t		rx_vma_ref;

	struct danipc_cdev_if_fifo	rx_fifo[max_ipc_prio];

	spinlock_t		rx_lock;	/* sync access to HW FIFO */
	wait_queue_head_t	rx_wq;
	struct danipc_poll_ctl	poll_ctl;

	struct shm_region	*tx_region;
	struct vm_area_struct	*tx_vma;
	atomic_t		tx_vma_ref;
	struct tx_queue		tx_queue;

	int			minor;

	/* Debug fs node */
	struct danipc_dbgfs_dent	dent;

	struct danipc_cdev_status	status;
};

/* Network device private data */
struct danipc_drvr {
	struct danipc_resource	res[RESOURCE_NUM];

	struct danipc_resource io_res[PLATFORM_MAX_NUM_OF_NODES];
	struct danipc_resource shm_res[PLATFORM_MAX_NUM_OF_NODES];

	const struct ipc_buf_desc *region_desc;
	uint32_t num_region_desc;

	/* Interface list */
	struct danipc_if		*if_list[DANIPC_MAX_IF];

	/* Valid Dest-aid map */
	uint8_t			dst_aid[MAX_AGENTS/BITS_PER_BYTE];

	/* Number of devices found during probe */
	uint8_t			ndev;

	/* Number of devices active */
	uint8_t			ndev_active;

	/* driver debug fs information */
	struct dentry		*dirent;

	/* memory map to access other processor's private address space */
	struct danipc_resource	*proc_map;
	uint32_t		proc_map_entry;

	/* char device driver interface */
	struct danipc_cdev	cdev[DANIPC_MAX_CDEV];

	/* local FIFO */
	struct danipc_fifo	lfifo[DANIPC_MAX_LFIFO];
	uint8_t			num_lfifo;
	uint8_t			num_l_if_fifo;
};

/* Connection information. */
struct danipc_pair {
	unsigned		prio;
	danipc_addr_t		dst;
	danipc_addr_t		src;
};

#define COOKIE_BASE		0xE000		/* EtherType */

#define PRIO_SHIFT			4
#define PRIO_MASK			(((1 <<  PRIO_SHIFT)) - 1)

#define COOKIE_TO_AGENTID(cookie)	((cookie - COOKIE_BASE) >> PRIO_SHIFT)
#define COOKIE_TO_PRIO(cookie)		((cookie - COOKIE_BASE) & PRIO_MASK)
#define AGENTID_TO_COOKIE(agentid, pri)	(COOKIE_BASE +			\
					  ((agentid) << PRIO_SHIFT) +	\
					  (pri))

/* Describes an IPC buffer region, either ours or an extern one */
struct ipc_buf_desc {
	uint32_t phy_addr;
	uint32_t sz;
};

void danipc_ll_init(struct danipc_drvr *drv);
void danipc_ll_cleanup(struct danipc_drvr *drv);

int acquire_local_fifo(struct danipc_fifo *fifo, void *owner);
int release_local_fifo(struct danipc_fifo *fifo, void *owner);

int danipc_poll_ctl_init(struct danipc_poll_ctl *ctl,
			 void (*func)(unsigned long),
			 unsigned long data,
			 unsigned poll_intval);

void danipc_poll_ctl_sched(struct danipc_poll_ctl *ctl);
void danipc_poll_ctl_stop(struct danipc_poll_ctl *ctl);

int danipc_netdev_init(struct platform_device *pdev);
int danipc_netdev_cleanup(struct platform_device *pdev);

int danipc_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);

extern struct danipc_drvr	danipc_driver;

#define danipc_io_base(n)	(danipc_driver.io_res[n].base)
#define danipc_res_base(n)	(danipc_driver.res[n].base)
#define danipc_shm_base(n)	(danipc_driver.shm_res[n].base)

long danipc_cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
int danipc_cdev_tx(struct danipc_cdev *cdev,
		   struct danipc_cdev_msghdr *hdr,
		   const char __user *buf,
		   size_t count);

int danipc_cdev_mmsg_rx(struct danipc_cdev *cdev,
			struct danipc_cdev_mmsg *mmsg);

void danipc_cdev_if_fifo_refill_b_fifo(struct danipc_cdev_if_fifo *rx_fifo);

int danipc_cdev_mapped_recv(struct danipc_cdev *cdev,
			    struct vm_area_struct *vma,
			    struct danipc_bufs *bufs);

int danipc_cdev_mapped_recv_done(struct danipc_cdev *cdev,
				 struct vm_area_struct *vma,
				 struct danipc_bufs *bufs);

int danipc_cdev_mapped_tx(struct danipc_cdev *cdev,
			  struct danipc_bufs *bufs);

int danipc_cdev_mapped_tx_get_buf(struct danipc_cdev *cdev,
				  struct danipc_bufs *bufs);

int danipc_cdev_mapped_tx_put_buf(struct danipc_cdev *cdev,
				  struct danipc_bufs *bufs);

static inline bool local_fifo_owner(struct danipc_fifo *fifo, void *owner)
{
	return ((fifo->flag & DANIPC_FIFO_F_INUSE) && (fifo->owner == owner));
}

/* DebugFS */
struct dentry *danipc_dbgfs_create_dir(struct dentry *parent_dent,
				       const char *dir_name,
				       struct danipc_dbgfs *nodes,
				       void *private_data);

int danipc_dbgfs_create_dent(struct danipc_dbgfs_dent *dbgfs_dent,
			     struct dentry *parent_dent,
			     const char *dir_name,
			     struct danipc_dbgfs *nodes,
			     void *private_data);

void danipc_dbgfs_remove_dent(struct danipc_dbgfs_dent *dbgfs_dent);

int danipc_dbgfs_netdev_init(void);
void danipc_dbgfs_netdev_remove(void);
int danipc_dbgfs_cdev_init(void);
void danipc_dbgfs_cdev_remove(void);

void danipc_dbgfs_dump_fifo(struct seq_file *s, struct danipc_fifo *fifo);
void danipc_dbgfs_dump_if_fifo(struct seq_file *s,
			       struct danipc_if_fifo *if_fifo);

/* inlines API to access the HW and convert the address */
static inline phys_addr_t virt_to_ipc(uint32_t cpuid, void *vaddr)
{
	if (likely(cpuid < PLATFORM_MAX_NUM_OF_NODES)) {
		struct danipc_resource *res = &danipc_driver.shm_res[cpuid];

		if (likely(res->base)) {
			unsigned offset;

			offset = (unsigned)vaddr - (unsigned)res->base;
			if (likely(offset < res->size))
				return res->start + offset;
		}
	}
	return 0;
}

static inline phys_addr_t danipc_hw_fifo_pop_raw(int node, uint8_t hw_unit)
{
	return danipc_hw_reg_read(danipc_io_base(node),
				  FIFO_RD_OFFSET(hw_unit));
}

static inline void danipc_hw_fifo_push_raw(int node,
					   uint8_t hw_unit,
					   phys_addr_t addr)
{
	danipc_hw_reg_write(danipc_io_base(node),
			    FIFO_WR_OFFSET(hw_unit),
			    addr);
}

static inline void *danipc_hw_fifo_pop(int node, uint8_t hw_unit)
{
	phys_addr_t addr = danipc_hw_fifo_pop_raw(node, hw_unit);

	return ((addr) ? ipc_to_virt(node, addr) : NULL);
}

static inline void danipc_hw_fifo_push(int node, uint8_t hw_unit, void *ptr)
{
	phys_addr_t addr = virt_to_ipc(node, ptr);

	if (addr)
		danipc_hw_fifo_push_raw(node, hw_unit, addr);
}

static inline void danipc_hw_fifo_drain(int node, uint8_t hw_unit)
{
	phys_addr_t addr;

	do {
		addr = danipc_hw_fifo_pop_raw(node, hw_unit);
	} while (addr);
}

static inline bool danipc_hw_fifo_is_empty(int node, uint8_t hw_unit)
{
	uint32_t status = danipc_hw_reg_read(danipc_io_base(node),
					     FIFO_STATUS(hw_unit));

	return (status & IPC_FIFO_EMPTY) ? true : false;
}

static inline bool danipc_hw_fifo_is_full(int node, uint8_t hw_unit)
{
	uint32_t status = danipc_hw_reg_read(danipc_io_base(node),
					     FIFO_STATUS(hw_unit));

	return (status & IPC_FIFO_FULL) ? true : false;
}

static inline void danipc_if_fifo_free_msg_raw(struct danipc_if_fifo *if_fifo,
					       phys_addr_t addr)
{
	if (likely(addr))
		danipc_hw_reg_write(if_fifo->fifo->io_base,
				    FIFO_WR_OFFSET(if_fifo->b_fifo_idx),
				    addr);
}

static inline void danipc_if_fifo_free_msg(struct danipc_if_fifo *if_fifo,
					   struct ipc_msg_hdr *hdr)
{
	phys_addr_t addr = virt_to_ipc(if_fifo->fifo->node_id, hdr);

	danipc_if_fifo_free_msg_raw(if_fifo, addr);
}

static inline phys_addr_t danipc_if_fifo_rx_msg_raw(
	struct danipc_if_fifo *if_fifo)
{
	return danipc_hw_reg_read(if_fifo->fifo->io_base,
				  FIFO_RD_OFFSET(if_fifo->m_fifo_idx));
}

static inline struct ipc_msg_hdr *danipc_if_fifo_rx_msg(
	struct danipc_if_fifo *if_fifo)
{
	phys_addr_t addr = danipc_if_fifo_rx_msg_raw(if_fifo);
	void *vaddr = NULL;

	vaddr = (addr) ? ipc_to_virt(if_fifo->fifo->node_id, addr) : NULL;

	return (struct ipc_msg_hdr *)vaddr;
}

#endif /* __DANIPC_H__ */
