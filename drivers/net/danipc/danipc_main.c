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

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/cpumask.h>
#include <linux/poll.h>

#include "danipc_k.h"
#include "ipc_api.h"
#include "danipc_lowlevel.h"

#define TX_MMAP_REGION_BUF_NUM	512

struct shm_msg {
	struct ipc_msg_hdr	*hdr;
	struct shm_buf		*shmbuf;
	uint8_t			prio;
};

struct danipc_drvr danipc_driver;

static void init_if_fifo_buf(struct danipc_if_fifo *if_fifo)
{
	struct danipc_fifo *fifo = if_fifo->fifo;
	char *base = danipc_res_base(IPC_BUFS_RES);
	int n;

	danipc_hw_fifo_drain(fifo->node_id, if_fifo->m_fifo_idx);
	danipc_hw_fifo_drain(fifo->node_id, if_fifo->b_fifo_idx);

	base += if_fifo->unit_num * IPC_BUFS_SZ_PER_IF_FIFO;
	for (n = 0; n < IPC_BUF_COUNT_MAX; n++, base += IPC_BUF_SIZE_MAX)
		danipc_hw_fifo_push(fifo->node_id, if_fifo->b_fifo_idx, base);
}

static int init_if_fifo(struct danipc_if_fifo *if_fifo,
			struct danipc_fifo *fifo,
			int m_fifo_idx,
			int b_fifo_idx)
{
	memset(if_fifo, 0, sizeof(*if_fifo));

	if_fifo->fifo = fifo;
	if_fifo->unit_num = danipc_driver.num_l_if_fifo;

	if (m_fifo_idx < 0 || b_fifo_idx < 0)
		return 0;
	if (!valid_fifo_unit(m_fifo_idx) || !valid_fifo_unit(b_fifo_idx))
		return -EINVAL;

	if_fifo->m_fifo_idx = m_fifo_idx;
	if_fifo->b_fifo_idx = b_fifo_idx;
	if_fifo->irq_mask = IPC_INT_FIFO_AF(m_fifo_idx);
	if_fifo->probed = true;

	init_if_fifo_buf(if_fifo);

	danipc_driver.num_l_if_fifo++;

	return 0;
}

static int init_local_fifo(struct platform_device *pdev,
			   uint8_t nodeid,
			   struct danipc_probe_info *info)
{
	struct device_node *node = pdev->dev.of_node;
	struct danipc_drvr *pdrv = &danipc_driver;
	struct danipc_fifo *fifo = &pdrv->lfifo[pdrv->num_lfifo];
	int ret = 0;

	if (pdrv->num_lfifo >= DANIPC_MAX_LFIFO)
		return -EINVAL;

	fifo->irq = irq_of_parse_and_map(node, pdrv->num_lfifo);
	if (!(fifo->irq) || (fifo->irq == NO_IRQ)) {
		pr_err("cannot get IRQ from DT\n");
		return -EINVAL;
	}

	mutex_init(&fifo->lock);

	fifo->probe_info = info;
	fifo->node_id = nodeid;
	fifo->io_base = pdrv->io_res[nodeid].base;
	fifo->owner = NULL;
	fifo->flag = 0;
	fifo->idx = pdrv->num_lfifo;

	memcpy(&danipc_driver.shm_res[fifo->node_id],
	       &danipc_driver.res[IPC_BUFS_RES],
	       sizeof(struct danipc_resource));

	ret = init_if_fifo(&fifo->if_fifo[ipc_prio_hi],
			   fifo,
			   info->hi_prio_m_fifo,
			   info->hi_prio_b_fifo);
	if (ret) {
		pr_err("failed to initialize hi-prio FIFO\n");
		return ret;
	}

	ret = init_if_fifo(&fifo->if_fifo[ipc_prio_lo],
			   fifo,
			   info->lo_prio_m_fifo,
			   info->lo_prio_b_fifo);
	if (ret) {
		pr_err("failed to initialize lo-prio FIFO\n");
		return ret;
	}

	pdrv->num_lfifo++;

	return 0;
}

int acquire_local_fifo(struct danipc_fifo *fifo, void *owner)
{
	int ret = 0;
	int pri;
	struct danipc_if_fifo *if_fifo = fifo->if_fifo;

	mutex_lock(&fifo->lock);
	if (fifo->flag & DANIPC_FIFO_F_INUSE) {
		ret = -EBUSY;
		goto out;
	}

	for (pri = 0; pri < max_ipc_prio; pri++, if_fifo++) {
		uint32_t val;

		if (!if_fifo->probed)
			continue;

		while ((val = danipc_hw_fifo_pop_raw(fifo->node_id,
						     if_fifo->m_fifo_idx)))
			danipc_hw_fifo_push_raw(fifo->node_id,
						if_fifo->b_fifo_idx,
						val);
	}

	ipc_agent_table_clean(fifo->node_id);

	fifo->owner = owner;
	fifo->flag |= DANIPC_FIFO_F_INUSE;

out:
	mutex_unlock(&fifo->lock);
	return ret;
}

int release_local_fifo(struct danipc_fifo *fifo, void *owner)
{
	int ret = 0;

	mutex_lock(&fifo->lock);
	if (!local_fifo_owner(fifo, owner)) {
		ret = -EPERM;
		goto out;
	}
	ipc_agent_table_clean(fifo->node_id);
	fifo->owner = NULL;
	fifo->flag &= ~DANIPC_FIFO_F_INUSE;
out:
	mutex_unlock(&fifo->lock);
	return ret;
}

static enum hrtimer_restart danipc_poll_ctl_timeout(struct hrtimer *timer)
{
	struct danipc_poll_ctl *ctl =
		container_of(timer, struct danipc_poll_ctl, timer);

	ctl->poll_cnt++;
	hrtimer_forward_now(timer, ctl->timer_intval);
	tasklet_schedule(&ctl->task);

	return HRTIMER_RESTART;
}

int danipc_poll_ctl_init(struct danipc_poll_ctl *ctl,
			 void (*func)(unsigned long),
			 unsigned long data,
			 unsigned poll_intval)
{
	tasklet_init(&ctl->task, func, data);
	hrtimer_init(&ctl->timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL_PINNED);
	ctl->timer.function = danipc_poll_ctl_timeout;
	ctl->timer_intval = ns_to_ktime(poll_intval * 1000);
	ctl->sched = false;
	ctl->poll_cnt = 0;
	return 0;
}

void danipc_poll_ctl_sched(struct danipc_poll_ctl *ctl)
{
	if (ctl->sched)
		return;
	hrtimer_start(&ctl->timer, ctl->timer_intval, HRTIMER_MODE_REL_PINNED);

	ctl->sched = true;
}

void danipc_poll_ctl_stop(struct danipc_poll_ctl *ctl)
{
	hrtimer_cancel(&ctl->timer);
	tasklet_kill(&ctl->task);
	ctl->sched = false;
}

static int parse_resources(struct platform_device *pdev, const char *regs[],
			   const char *resource[], const char *shm_names[])
{
	struct device_node *node = pdev->dev.of_node;
	struct danipc_drvr *pdrv = &danipc_driver;
	struct resource *res;
	int r;
	const struct ipc_buf_desc *desc;
	uint32_t n, len = 0;

	if (unlikely(node == NULL))
		return -ENODEV;

	for (r = 0; r < RESOURCE_NUM; r++) {
		res = platform_get_resource_byname(pdev,
						   IORESOURCE_MEM,
						   resource[r]);
		if (res == NULL) {
			if (r == IPC_BUFS_RES)
				continue;
			pr_err("cannot get resource %s\n", resource[r]);
			return -EINVAL;
		}
		pdrv->res[r].start = res->start;
		pdrv->res[r].size = resource_size(res);
	}

	if (!pdrv->res[IPC_BUFS_RES].size) {
		desc = of_get_property(node, "ul-bufs", NULL);

		if (desc == NULL) {
			pr_err("could not find ul-bufs property\n");
			return -EINVAL;
		}
		pdrv->res[IPC_BUFS_RES].start = desc->phy_addr;
		pdrv->res[IPC_BUFS_RES].size = desc->sz;
	}

	pdrv->region_desc = of_get_property(node, "dl-bufs", &n);
	pdrv->num_region_desc = (pdrv->region_desc) ?
		(n/sizeof(struct ipc_buf_desc)) : 0;

	desc = of_get_property(node, "memory-region", &len);
	if (desc) {
		struct danipc_resource *p;
		int i;

		n = len/sizeof(struct ipc_buf_desc);
		p = kzalloc((n * sizeof(*p)), GFP_KERNEL);
		if (p == NULL)
			return -ENOMEM;

		for (i = 0; i < n; i++) {
			p[i].start = desc[i].phy_addr;
			p[i].size = desc[i].sz;
			p[i].base = ioremap_cache(
				p[i].start, p[i].size);
			if (p[i].base == NULL) {
				pr_err("failed to map the region\n");
				while (--i >= 0)
					iounmap(p[i].base);
				kfree(p);
				return -ENOMEM;
			}
		}
		danipc_driver.proc_map = p;
		danipc_driver.proc_map_entry = n;
	}

	for (r = 0; r < PLATFORM_MAX_NUM_OF_NODES; r++) {
		if (!regs[r])
			continue;
		res = platform_get_resource_byname(pdev,
						   IORESOURCE_MEM,
						   regs[r]);
		if (res == NULL) {
			pr_debug("reg resource %s not provided\n", regs[r]);
			continue;
		}
		pdrv->io_res[r].start = res->start;
		pdrv->io_res[r].size = resource_size(res);

		/* Don't look at shared memory regions if we support
		 * flexible memory map
		 */
		if (pdrv->num_region_desc)
			continue;
		if (shm_names[r] &&
		    !of_property_read_u32(node, shm_names[r], &n))
			pdrv->shm_res[r].size = n;
	}

	return 0;
}

static int probe_local_fifo(struct platform_device *pdev,
			    struct danipc_probe_info *probe_list,
			    const char *regs[])
{
	struct danipc_drvr *pdrv = &danipc_driver;
	uint8_t nodeid;
	int rc = ENODEV;

	for (nodeid = 0; nodeid < PLATFORM_MAX_NUM_OF_NODES; nodeid++) {
		if (pdrv->io_res[nodeid].base &&
		    (!strcmp(probe_list->res_name, regs[nodeid]))) {
			rc = init_local_fifo(pdev, nodeid, probe_list);
			break;
		}
	}

	pr_info("FIFO %s %s!\n", probe_list->res_name,
		(rc == ENODEV) ? "NOT FOUND" : "FOUND");
	return 0;
}

static struct danipc_probe_info danipc_probe_list[DANIPC_MAX_LFIFO] = {
	{ "apps_ipc_data", "danipc", 0, 1, 2, 3, 0, 0x00000001 },
	{ "apps_ipc_pcap", "danipc-pcap", 0, 1, 2, 3, 0, 0x00004000 },
	{ "apps_hex_ipc_log", "danipc-hex2log", 0, 1, -1, -1, 0, 0x00010000 },
	{ "apps_hex_ipc_log", "danipc-hex3log", -1, -1, 2, 3, 1, 0x00020000 },
};

static int danipc_probe_lfifo(struct platform_device *pdev, const char *regs[])
{
	uint8_t idx;
	int rc = 0;

	/* Probe for local fifos */
	for (idx = 0; idx < DANIPC_MAX_LFIFO && !rc; idx++)
		rc = probe_local_fifo(pdev, &danipc_probe_list[idx], regs);

	return rc;
}

/* Character device interface */
static inline void __shm_msg_free(struct danipc_cdev_if_fifo *rx_fifo,
				  struct shm_msg *msg)
{
	shm_bufpool_put_buf(&rx_fifo->rx_queue.freeq, msg->shmbuf);
}

static void shm_msg_free(struct danipc_cdev *cdev, struct shm_msg *msg)
{
	struct danipc_cdev_if_fifo *rx_fifo = &cdev->rx_fifo[msg->prio];
	unsigned long flags;

	spin_lock_irqsave(&cdev->rx_lock, flags);
	__shm_msg_free(rx_fifo, msg);
	danipc_cdev_if_fifo_refill_b_fifo(rx_fifo);

	spin_unlock_irqrestore(&cdev->rx_lock, flags);
}

static int __shm_msg_get(struct danipc_cdev *cdev, struct shm_msg *msg)
{
	struct danipc_fifo *fifo = cdev->fifo;
	struct shm_buf *buf;
	int prio;
	int ret = -EAGAIN;

	for (prio = ipc_prio_hi; prio >= 0; prio--) {
		struct danipc_cdev_if_fifo *rx_fifo = &cdev->rx_fifo[prio];

		buf = shm_bufpool_get_buf(&rx_fifo->rx_queue.recvq);
		if (buf) {
			msg->prio = prio;
			msg->shmbuf = buf;
			msg->hdr = ipc_to_virt(fifo->node_id, buf_paddr(buf));

			dev_dbg(cdev->dev, "get message at %p\n", msg->hdr);
			ret = 0;
			break;
		}
	}
	return ret;
}

static ssize_t ipc_msg_copy_to_user(void *msg, enum ipc_prio prio,
				    char __user *buf, size_t count)
{
	struct ipc_msg_hdr *hdr = msg;
	struct danipc_cdev_msghdr cdev_hdr;
	ssize_t size = count - sizeof(cdev_hdr);
	ssize_t n = 0;

	if (size <= 0)
		return -EINVAL;

	if (size > hdr->msg_len)
		size = hdr->msg_len;

	cdev_hdr.dst = hdr->dest_aid;
	cdev_hdr.src = hdr->src_aid;
	cdev_hdr.prio = prio;

	if (copy_to_user(buf, &cdev_hdr, sizeof(cdev_hdr)))
		return -EFAULT;
	n += sizeof(cdev_hdr);
	if (copy_to_user(buf+n, hdr+1, size))
		return -EFAULT;
	n += size;
	return n;
}

static void reset_rx_queue_status(struct rx_queue *rxque)
{
	rxque->status.bq_lo = rxque->bq.count;
	rxque->status.freeq_lo = rxque->freeq.count;
	rxque->status.recvq_hi = rxque->recvq.count;
}

static int danipc_cdev_rx_buf_init(struct danipc_cdev *cdev)
{
	struct danipc_fifo *fifo = cdev->fifo;
	int prio;

	for (prio = 0; prio < max_ipc_prio; prio++) {
		struct danipc_cdev_if_fifo *rx_fifo = &cdev->rx_fifo[prio];
		struct danipc_if_fifo *if_fifo = rx_fifo->if_fifo;

		if (if_fifo->probed) {
			danipc_hw_fifo_drain(fifo->node_id,
					     if_fifo->m_fifo_idx);
			danipc_hw_fifo_drain(fifo->node_id,
					     if_fifo->b_fifo_idx);
			danipc_cdev_if_fifo_refill_b_fifo(rx_fifo);
		}
		reset_rx_queue_status(&rx_fifo->rx_queue);
	}

	return 0;
}

static int danipc_cdev_rx_buf_release(struct danipc_cdev *cdev)
{
	int pri;

	for (pri = ipc_prio_lo; pri < max_ipc_prio; pri++) {
		struct danipc_if_fifo *if_fifo = cdev->rx_fifo[pri].if_fifo;

		if (if_fifo->probed)
			init_if_fifo_buf(if_fifo);
	}

	return 0;
}

void danipc_cdev_if_fifo_refill_b_fifo(struct danipc_cdev_if_fifo *rx_fifo)
{
	struct danipc_if_fifo *if_fifo = rx_fifo->if_fifo;
	struct danipc_fifo *fifo = if_fifo->fifo;
	struct rx_queue *rxq = &rx_fifo->rx_queue;
	struct shm_bufpool *bq = &rxq->bq;
	struct shm_bufpool *freeq = &rxq->freeq;
	uint32_t n = 0;

	while (bq->count < IPC_BUF_COUNT_MAX) {
		struct shm_buf *buf;

		buf = shm_bufpool_get_buf(freeq);

		if (buf == NULL)
			break;

		danipc_if_fifo_free_msg_raw(if_fifo, buf_paddr(buf));
		shm_bufpool_put_buf(bq, buf);
		n++;
	}
	if (freeq->count < rxq->status.freeq_lo)
		rxq->status.freeq_lo = freeq->count;
	if (bq->count < rxq->status.bq_lo)
		rxq->status.bq_lo = bq->count;
	if (n)
		pr_debug("%s: fill fifo(%u/%u) with %d buffers\n",
			__func__, fifo->idx, if_fifo->b_fifo_idx, n);
}

static int danipc_cdev_init_tx_region(struct danipc_cdev *cdev,
				      uint8_t cpuid,
				      enum ipc_prio pri)
{
	uint32_t size = SZ_256K;
	phys_addr_t addr;
	phys_addr_t start;

	if (!valid_cpu_id(cpuid) || pri != ipc_prio_hi)
		return -EINVAL;

	addr = danipc_hw_fifo_pop_raw(cpuid, DEFAULT_HI_PRIO_B_FIFO);
	if (!addr)
		return -EAGAIN;

	danipc_hw_fifo_push_raw(cpuid, DEFAULT_HI_PRIO_B_FIFO, addr);

	if (ipc_to_virt(cpuid, addr) == NULL)
		return -EAGAIN;

	start = danipc_driver.shm_res[cpuid].start;
	size = danipc_driver.shm_res[cpuid].size;

	dev_dbg(cdev->dev,
		"%s: b_fifo(%u) start: 0x%x size: 0x%x\n",
		__func__, cpuid, start, size);

	cdev->tx_region = shm_region_create(start,
					    danipc_shm_base(cpuid),
					    size,
					    IPC_BUF_SIZE_MAX,
					    DANIPC_MMAP_TX_BUF_HEADROOM,
					    TX_MMAP_REGION_BUF_NUM);
	if (cdev->tx_region == NULL)
		return -EPERM;

	shm_bufpool_acquire_whole_region(&cdev->tx_queue.mmap_bufcacheq,
					 cdev->tx_region);
	return 0;
}

static ssize_t danipc_cdev_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct danipc_cdev *cdev = file->private_data;
	unsigned int minor = iminor(file_inode(file));
	struct shm_msg msg;
	unsigned long flags;
	ssize_t ret = 0;

	dev_dbg(cdev->dev, "%s: minor %u\n", __func__, minor);

	if (count <= sizeof(struct danipc_cdev_msghdr)) {
		cdev->status.rx_error++;
		return -EINVAL;
	}

	add_wait_queue(&cdev->rx_wq, &wait);
	while (1) {
		spin_lock_irqsave(&cdev->rx_lock, flags);
		ret = __shm_msg_get(cdev, &msg);
		spin_unlock_irqrestore(&cdev->rx_lock, flags);

		if (!ret)
			break;

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto out;
		}

		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		if (signal_pending(current)) {
			cdev->status.rx_error++;
			ret = -ERESTARTSYS;
			goto out;
		}
	}

	ret = ipc_msg_copy_to_user(msg.hdr, msg.prio, buf, count);
	if (ret < 0) {
		cdev->status.rx_error++;
	} else {
		cdev->status.rx++;
		cdev->status.rx_bytes += ret;
	}

	shm_msg_free(cdev, &msg);

out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&cdev->rx_wq, &wait);
	return ret;
}

int danipc_cdev_mmsg_rx(struct danipc_cdev *cdev,
			struct danipc_cdev_mmsg *mmsg)
{
	struct danipc_cdev_if_fifo *rx_fifo;
	struct rx_queue *rx_queue;
	struct shm_bufpool msgs;
	struct shm_buf *buf, *p;
	unsigned long flags;
	int n = 0;

	if (unlikely(!cdev || !mmsg))
		return -EINVAL;
	if (mmsg->msgs.num_entry > DANIPC_BUFS_MAX_NUM_BUF ||
	    !mmsg->msgs.num_entry)
		return -EINVAL;
	if (!valid_ipc_prio(mmsg->hdr.prio))
		return -EINVAL;

	rx_fifo = &cdev->rx_fifo[mmsg->hdr.prio];
	rx_queue = &rx_fifo->rx_queue;
	shm_bufpool_init(&msgs);

	spin_lock_irqsave(&cdev->rx_lock, flags);
	list_for_each_entry_safe(buf, p, &rx_queue->recvq.head, list) {
		struct ipc_msg_hdr *hdr = ipc_to_virt(cdev->fifo->node_id,
						      buf_paddr(buf));
		if (hdr->dest_aid == mmsg->hdr.dst &&
		    hdr->src_aid == mmsg->hdr.src &&
		    hdr->msg_len <= mmsg->msgs.entry[msgs.count].data_len) {
			shm_bufpool_del_buf(&rx_queue->recvq, buf);
			shm_bufpool_put_buf(&msgs, buf);
			if (msgs.count == mmsg->msgs.num_entry)
				break;
		}
	}
	spin_unlock_irqrestore(&cdev->rx_lock, flags);

	if (!msgs.count)
		return 0;

	list_for_each_entry(buf, &msgs.head, list) {
		struct ipc_msg_hdr *hdr = ipc_to_virt(cdev->fifo->node_id,
						      buf_paddr(buf));
		if (copy_to_user(mmsg->msgs.entry[n].data,
				 hdr + 1,
				 hdr->msg_len)) {
			cdev->status.rx_error++;
			n = -EFAULT;
			break;
		}
		mmsg->msgs.entry[n++].data_len = hdr->msg_len;
		cdev->status.rx++;
		cdev->status.rx_bytes += hdr->msg_len;
	}

	spin_lock_irqsave(&cdev->rx_lock, flags);
	while ((buf = shm_bufpool_get_buf(&msgs)))
		shm_bufpool_put_buf(&rx_queue->freeq, buf);
	danipc_cdev_if_fifo_refill_b_fifo(rx_fifo);
	spin_unlock_irqrestore(&cdev->rx_lock, flags);

	return n;
}

int danipc_cdev_tx(struct danipc_cdev *cdev,
		   struct danipc_cdev_msghdr *hdr,
		   const char __user *buf,
		   size_t count)
{
	struct danipc_drvr *pdrv;
	struct ipc_msg_hdr *msg;

	if (unlikely(cdev == NULL || hdr == NULL))
		return -EINVAL;

	if (count < 0)
		return -EINVAL;

	pdrv = cdev->drvr;

	dev_dbg(cdev->dev, "%s: request to send %d bytes to %u:%u\n",
		__func__, count, hdr->dst, hdr->src);

	if (!valid_ipc_prio(hdr->prio)) {
		cdev->status.tx_drop++;
		dev_dbg(cdev->dev, "%s: invalid priorioty %u\n",
			__func__, hdr->prio);
		return -EINVAL;
	}

	if (!DANIPC_IS_AGENT_DISCOVERED(hdr->dst, pdrv->dst_aid)) {
		cdev->status.tx_drop++;
		return -EAGAIN;
	}

	msg = ipc_msg_alloc(hdr->src, hdr->dst, count, 0x12,
			    default_b_fifo(hdr->prio));
	if (msg == NULL) {
		cdev->status.tx_no_buf++;
		dev_dbg(cdev->dev, "%s: failed to alloc %d bytes from fifo\n",
			__func__, count);
		return -EBUSY;
	}

	if (ipc_copy_from((char *)(msg+1), buf, count, true)) {
		cdev->status.tx_error++;
		ipc_msg_free(msg, default_b_fifo(hdr->prio));
		dev_warn(cdev->dev, "%s: failed to copy from user space\n",
			 __func__);
	}

	if (ipc_msg_send(msg, default_m_fifo(hdr->prio))) {
		cdev->status.tx_error++;
		dev_warn(cdev->dev, "%s: failed to send %d bytes\n",
			 __func__, count);
		return -ENOMEM;
	}

	cdev->status.tx++;
	cdev->status.tx_bytes += count;
	return 0;
}

static ssize_t danipc_cdev_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct danipc_cdev *cdev = file->private_data;
	unsigned int minor = iminor(file_inode(file));
	struct danipc_cdev_msghdr hdr;
	int ret;

	dev_dbg(cdev->dev, "%s: minor %u, byte %d\n", __func__, minor, count);

	if (copy_from_user(&hdr, buf, sizeof(hdr)))
		return -EFAULT;

	ret = danipc_cdev_tx(cdev, &hdr, buf + sizeof(hdr),
			     count - sizeof(hdr));
	if (ret)
		return ret;

	return count;
}

static unsigned int danipc_cdev_poll(struct file *file,
				     struct poll_table_struct *wait)
{
	struct danipc_cdev *cdev = file->private_data;
	unsigned long flags;
	unsigned int ret = 0;

	dev_dbg(cdev->dev, "%s\n", __func__);

	poll_wait(file, &cdev->rx_wq, wait);
	spin_lock_irqsave(&cdev->rx_lock, flags);
	if (!list_empty(&cdev->rx_fifo[ipc_prio_hi].rx_queue.recvq.head) ||
	    !list_empty(&cdev->rx_fifo[ipc_prio_lo].rx_queue.recvq.head))
		ret = POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&cdev->rx_lock, flags);

	return ret;
}

static int danipc_cdev_if_fifo_recv(struct danipc_cdev_if_fifo *rx_fifo)
{
	struct danipc_cdev *cdev = rx_fifo->cdev;
	struct danipc_if_fifo *if_fifo = rx_fifo->if_fifo;
	struct danipc_fifo *fifo = if_fifo->fifo;
	struct rx_queue *pq = &rx_fifo->rx_queue;
	uint32_t msg_paddr;
	struct shm_buf *buf;
	struct ipc_msg_hdr *hdr;
	int n = 0;

	while (1) {
		msg_paddr = danipc_if_fifo_rx_msg_raw(if_fifo);
		if (!msg_paddr)
			break;
		buf = shm_region_find_buf_by_pa(cdev->rx_region, msg_paddr);
		if (buf == NULL) {
			cdev->status.rx_error++;
			dev_warn(cdev->dev,
				 "%s: can't find buffer, paddr=0x%x\n",
				 __func__, msg_paddr);
			continue;
		}

		if (shm_bufpool_del_buf(&pq->bq, buf)) {
			cdev->status.rx_error++;
			dev_err(cdev->dev,
				"%s: buffer %p not in the buffer queue\n",
				__func__, buf);
			continue;
		}
		hdr = buf_vaddr(buf);
		ipc_msg_hdr_cache_invalid(hdr);

		if (!ipc_msg_valid(hdr, fifo->node_id,
				   &cdev->status.rx_err_stats)) {
			cdev->status.rx_drop++;
			dev_dbg(cdev->dev,
				"%s: drop message paddr=0x%x, vaddr=%p\n",
				__func__, msg_paddr, hdr);
			shm_bufpool_put_buf(&pq->freeq, buf);
		} else {
			cdev->status.rx++;
			cdev->status.rx_bytes += hdr->msg_len;
			shm_bufpool_put_buf(&pq->recvq, buf);
			ipc_msg_payload_cache_invalid(hdr);
			if (pq->recvq.count > pq->status.recvq_hi)
				pq->status.recvq_hi = pq->recvq.count;
			n++;
		}
	}

	danipc_cdev_if_fifo_refill_b_fifo(rx_fifo);

	return n;
}

static void danipc_cdev_rx_poll(unsigned long data)
{
	struct danipc_cdev *cdev = (struct danipc_cdev *)data;
	unsigned long flags;
	int n, prio;

	spin_lock_irqsave(&cdev->rx_lock, flags);
	for (n = 0, prio = ipc_prio_hi; prio >= 0; prio--) {
		struct danipc_cdev_if_fifo *rx_fifo = &cdev->rx_fifo[prio];

		if (rx_fifo->if_fifo->probed)
			n += danipc_cdev_if_fifo_recv(rx_fifo);
	}
	spin_unlock_irqrestore(&cdev->rx_lock, flags);

	if (n)
		wake_up(&cdev->rx_wq);
}

static irqreturn_t danipc_cdev_interrupt(int irq, void *data)
{
	struct danipc_cdev *cdev = (struct danipc_cdev *)data;

	danipc_fifo_mask_interrupt(cdev->fifo);
	danipc_poll_ctl_sched(&cdev->poll_ctl);

	return IRQ_HANDLED;
}

static inline void *buf_vma_vaddr(struct shm_buf *buf,
				  struct vm_area_struct *vma)
{
	phys_addr_t vm_size = vma->vm_end - vma->vm_start;
	phys_addr_t offset_s = buf->offset + (buf->region->start & ~PAGE_MASK);
	phys_addr_t offset_e = offset_s + buf->region->buf_sz;

	if (!address_in_range(offset_e, 0, vm_size))
		return NULL;
	return (void *)(vma->vm_start + offset_s);
}

static inline phys_addr_t vma_paddr(struct vm_area_struct *vma,
				    struct shm_region *region,
				    void *addr)
{
	unsigned long offset;

	if (unlikely(vma == NULL || region == NULL))
		return 0;

	if (!address_in_range((phys_addr_t)addr, vma->vm_start, vma->vm_end))
		return 0;

	offset = (unsigned long)(addr) - vma->vm_start;

	return ((region->start & PAGE_MASK)+offset);
}

static inline phys_addr_t vma_rx_paddr(struct danipc_cdev *cdev, void *addr)
{
	return vma_paddr(cdev->rx_vma, cdev->rx_region, addr);
}

static inline void *to_ipc_data(void *msg)
{
	struct ipc_msg_hdr *hdr = (struct ipc_msg_hdr *)msg;

	return (void *)(hdr+1);
}

static inline uint8_t vma_get_aid(struct vm_area_struct *vma)
{
	unsigned offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned aid = (offset >> DANIPC_MMAP_AID_SHIFT) & 0xFF;

	return (uint8_t)aid;
}

static inline uint8_t vma_get_lid(struct vm_area_struct *vma)
{
	unsigned offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned lid = (offset >> DANIPC_MMAP_LID_SHIFT) & 0xFF;

	return (uint8_t)lid;
}

static inline int vma_get_node(struct vm_area_struct *vma)
{
	int node_id;

	node_id = ipc_get_node(vma_get_aid(vma)) & 0xFF;
	if (node_id >= PLATFORM_MAX_NUM_OF_NODES)
		return -EINVAL;

	return node_id;
}

static bool vma_in_region(struct vm_area_struct *vma, struct shm_region *region)
{
	unsigned long vma_size = vma->vm_end - vma->vm_start;
	unsigned long region_size = region->end - region->start;
	unsigned long size;

	size = region_size + (region->start & ~PAGE_MASK);

	return (size > vma_size) ? true : false;
}

int danipc_cdev_mapped_recv(struct danipc_cdev *cdev,
			    struct vm_area_struct *vma,
			    struct danipc_bufs *bufs)
{
	struct shm_buf *buf, *p;
	int n = 0, num;
	int i;
	unsigned long flags;

	if (unlikely(!cdev || !vma || !bufs))
		return -EINVAL;

	if (!cdev->rx_vma) {
		dev_err(cdev->dev, "%s: the region is not mmaped\n",
			__func__);
		return -EIO;
	}

	num = bufs->num_entry;
	if (!num || num > DANIPC_BUFS_MAX_NUM_BUF)
		return -EINVAL;

	spin_lock_irqsave(&cdev->rx_lock, flags);

	for (i = ipc_prio_hi; (i >= ipc_prio_lo) && (n < num); i--) {
		struct shm_bufpool *pq = &cdev->rx_fifo[i].rx_queue.recvq;
		struct shm_bufpool *mmapq = &cdev->rx_fifo[i].rx_queue.mmapq;

		list_for_each_entry_safe(buf, p, &pq->head, list) {
			void *vma_addr = buf_vma_vaddr(buf, cdev->rx_vma);

			if (vma_addr) {
				struct ipc_msg_hdr *ipchdr =
					(struct ipc_msg_hdr *)vma_addr;

				dev_dbg(cdev->dev,
					"%s: put %p(%p) in mmapq, prio=%d\n",
					__func__, buf, vma_addr, i);

				shm_bufpool_del_buf(pq, buf);
				shm_bufpool_put_buf(mmapq, buf);
				bufs->entry[n].data = ipchdr+1;
				bufs->entry[n].data_len = ipchdr->msg_len;
				cdev->status.mmap_rx++;
				n++;
				if (n >= num)
					break;
			}
		}
	}

	spin_unlock_irqrestore(&cdev->rx_lock, flags);
	bufs->num_entry = n;
	return n;
}

int danipc_cdev_mapped_recv_done(struct danipc_cdev *cdev,
				 struct vm_area_struct *vma,
				 struct danipc_bufs *bufs)
{
	struct danipc_cdev_if_fifo *rx_fifo;
	unsigned long flags;
	int i;

	if (unlikely(!bufs || !vma || !cdev))
		return -EINVAL;

	if (bufs->num_entry > DANIPC_BUFS_MAX_NUM_BUF || !bufs->num_entry)
		return -EINVAL;

	spin_lock_irqsave(&cdev->rx_lock, flags);

	for (i = 0, rx_fifo = NULL; i < bufs->num_entry; i++) {
		phys_addr_t paddr = vma_rx_paddr(cdev, bufs->entry[i].data);
		struct shm_buf *buf;
		int pri;

		if (!paddr) {
			cdev->status.mmap_rx_error++;
			dev_warn(cdev->dev,
				 "%s: address %x not in mmap area\n",
				 __func__, paddr);
			continue;
		}

		buf = shm_region_find_buf_by_pa(cdev->rx_region, paddr);
		if (!buf) {
			cdev->status.mmap_rx_error++;
			dev_warn(cdev->dev,
				 "%s: no buffer at phy_addr %x\n",
				 __func__, paddr);
			continue;
		}

		for (pri = ipc_prio_hi; pri >= 0; pri--) {
			struct rx_queue *pq = &cdev->rx_fifo[pri].rx_queue;

			if (buf->head == &pq->mmapq.head) {
				dev_dbg(cdev->dev,
					"%s: put mapped buf %p to freeq\n",
					__func__, buf);
				shm_bufpool_del_buf(&pq->mmapq, buf);
				shm_bufpool_put_buf(&pq->freeq, buf);
				cdev->status.mmap_rx_done++;
				rx_fifo = &cdev->rx_fifo[pri];
				break;
			}
		}
		if (!rx_fifo) {
			cdev->status.mmap_rx_error++;
			dev_warn(cdev->dev,
				 "%s: vaddr %p not in mmapq\n",
				 __func__, bufs->entry[i].data);
		}
	}

	if (rx_fifo)
		danipc_cdev_if_fifo_refill_b_fifo(rx_fifo);

	spin_unlock_irqrestore(&cdev->rx_lock, flags);

	return 0;
}

static void danipc_cdev_rx_vma_open(struct vm_area_struct *vma)
{
	struct danipc_cdev *cdev = vma->vm_private_data;

	dev_dbg(cdev->dev, "%s: vma %p\n", __func__, vma);

	if (atomic_add_return(1, &cdev->rx_vma_ref) == 1)
		cdev->rx_vma = vma;
}

static void danipc_cdev_rx_vma_close(struct vm_area_struct *vma)
{
	struct danipc_cdev *cdev = vma->vm_private_data;
	unsigned long flags;
	int i;

	dev_dbg(cdev->dev, "%s: vma %p\n", __func__, vma);

	if (!atomic_dec_and_test(&cdev->rx_vma_ref))
		return;

	spin_lock_irqsave(&cdev->rx_lock, flags);
	for (i = 0; i < max_ipc_prio; i++) {
		struct danipc_cdev_if_fifo *rx_fifo = &cdev->rx_fifo[i];
		struct rx_queue *pq = &rx_fifo->rx_queue;
		struct shm_buf *buf;

		while ((buf = shm_bufpool_get_buf(&pq->mmapq)))
			shm_bufpool_put_buf(&pq->freeq, buf);
		if (rx_fifo->if_fifo->probed)
			danipc_cdev_if_fifo_refill_b_fifo(rx_fifo);
	}

	spin_unlock_irqrestore(&cdev->rx_lock, flags);

	cdev->rx_vma = NULL;
}

static struct vm_operations_struct danipc_cdev_rx_vma_ops = {
	.open	= danipc_cdev_rx_vma_open,
	.close	= danipc_cdev_rx_vma_close,
};

static int danipc_rx_mmap(struct danipc_cdev *cdev, struct vm_area_struct *vma)
{
	struct shm_region *region = cdev->rx_region;
	unsigned long size = vma->vm_end - vma->vm_start;

	if (cdev->rx_vma) {
		dev_warn(cdev->dev, "%s: rx is already mmaped\n",
			 __func__);
		return -EBUSY;
	}

	if (vma_in_region(vma, region)) {
		dev_dbg(cdev->dev,
			"%s: vma area is too small, start=%lx end=%lx\n",
			__func__, vma->vm_start, vma->vm_end);
		return -EINVAL;
	}

	if (remap_pfn_range(vma,
			    vma->vm_start,
			    (region->start) >> PAGE_SHIFT,
			    size,
			    vma->vm_page_prot)) {
		dev_warn(cdev->dev,
			 "%s: remap_pfn_range(%lx/%lx/%lx) failed\n",
			 __func__, vma->vm_start, size, vma->vm_pgoff);
		return -EAGAIN;
	}

	vma->vm_private_data = cdev;
	vma->vm_ops = &danipc_cdev_rx_vma_ops;
	danipc_cdev_rx_vma_open(vma);
	return 0;
}

static struct shm_buf *tx_mmap_bufcacheq_get_buf(struct danipc_cdev *cdev,
						 phys_addr_t phy_addr)
{
	struct shm_bufpool *pq = &cdev->tx_queue.mmap_bufcacheq;
	struct shm_buf *buf = NULL;

	if (cdev->tx_region->dir_buf_map) {
		buf = shm_bufpool_find_buf_in_region(pq,
						     cdev->tx_region,
						     phy_addr);
	} else {
		buf = shm_bufpool_get_buf(pq);
		if (buf)
			buf->offset = phy_addr - buf->region->start;
	}
	return buf;
}

static inline void tx_mmap_bufcacheq_put_buf(struct danipc_cdev *cdev,
					     struct shm_buf *buf)
{
	struct shm_bufpool *pq = &cdev->tx_queue.mmap_bufcacheq;

	shm_bufpool_put_buf(pq, buf);
}

int danipc_cdev_mapped_tx(struct danipc_cdev *cdev, struct danipc_bufs *bufs)
{
	struct shm_region *region;
	struct shm_buf *buf;
	struct ipc_msg_hdr *ipchdr;
	phys_addr_t paddr, paddr_buf, offset;
	int i, num_tx;
	uint8_t aid, lid, node;

	if (unlikely(!bufs || !cdev))
		return -EINVAL;

	region = cdev->tx_region;
	if (!region || !cdev->tx_vma) {
		dev_warn(cdev->dev, "%s: tx is no mmaped\n", __func__);
		return -EPERM;
	}

	if (bufs->num_entry > DANIPC_BUFS_MAX_NUM_BUF || !bufs->num_entry)
		return -EINVAL;

	aid = vma_get_aid(cdev->tx_vma);
	lid = vma_get_lid(cdev->tx_vma);
	node = ipc_get_node(aid);

	for (i = 0, num_tx = 0; i < bufs->num_entry; i++) {
		paddr = vma_paddr(cdev->tx_vma, region, bufs->entry[i].data);

		buf = shm_bufpool_find_buf_in_region(&cdev->tx_queue.mmapq,
						     region,
						     paddr);
		if (buf == NULL) {
			dev_warn(cdev->dev,
				 "%s: buffer at phy address %x not in mmapq\n",
				 __func__, paddr);
			cdev->status.tx_error++;
			cdev->status.mmap_tx_error++;
			continue;
		}

		if (shm_bufpool_del_buf(&cdev->tx_queue.mmapq, buf)) {
			dev_warn(cdev->dev,
				 "%s: vaddr %p not in mmapq\n",
				 __func__, bufs->entry[i].data);
			cdev->status.tx_error++;
			cdev->status.mmap_tx_error++;
			continue;
		}

		paddr_buf = buf_paddr(buf);
		offset = paddr - paddr_buf;

		dev_dbg(cdev->dev, "%s: buf(%p) is returned, paddr=%x\n",
			__func__, buf, paddr_buf);

		/* For now, the user must pass the same address it got
		 * from ioctl
		 */
		if (offset != sizeof(*ipchdr)) {
			dev_warn(cdev->dev,
				 "%s: message offset(%p/%x) not expected.\n",
				 __func__, bufs->entry[i].data, offset);
			goto err;
		}
		if ((offset + bufs->entry[i].data_len > region->buf_sz) ||
		    (offset < sizeof(*ipchdr))) {
			dev_warn(cdev->dev,
				 "%s: message(%p) cross the buffer boundary\n",
				 __func__, bufs->entry[i].data);
			goto err;
		}

		offset -= sizeof(*ipchdr);
		ipchdr = (struct ipc_msg_hdr *)bufs->entry[i].data - 1;
		ipchdr->msg_type = 0x12;
		ipchdr->msg_len = bufs->entry[i].data_len;
		ipchdr->reply = NULL;
		ipchdr->dest_aid = aid;
		ipchdr->src_aid = lid;
		ipchdr->request_num = ipc_req_sn++;
		ipchdr->next = NULL;

		dev_dbg(cdev->dev,
			"%s: send %u bytes to %u, lid=%u paddr=%08x\n",
			__func__, ipchdr->msg_len, ipchdr->dest_aid,
			ipchdr->src_aid, paddr_buf + offset);

		danipc_hw_fifo_push_raw(node,
					DEFAULT_HI_PRIO_M_FIFO,
					paddr_buf + offset);

		cdev->status.mmap_tx++;
		cdev->status.tx++;
		cdev->status.tx_bytes += bufs->entry[i].data_len;
		num_tx++;

		tx_mmap_bufcacheq_put_buf(cdev, buf);
		continue;
err:
		tx_mmap_bufcacheq_put_buf(cdev, buf);
		cdev->status.tx_drop++;
		cdev->status.mmap_tx_error++;
		danipc_hw_fifo_push_raw(node,
					DEFAULT_HI_PRIO_B_FIFO,
					paddr_buf);
	}

	return num_tx;
}

int danipc_cdev_mapped_tx_get_buf(struct danipc_cdev *cdev,
				  struct danipc_bufs *bufs)
{
	struct shm_buf *buf;
	phys_addr_t paddr;
	void *vaddr;
	int ret = 0, node, i = 0;

	if (unlikely(!bufs || !cdev))
		return -EINVAL;

	if (bufs->num_entry > DANIPC_BUFS_MAX_NUM_BUF || !bufs->num_entry)
		return -EINVAL;

	if (!cdev->tx_vma) {
		dev_warn(cdev->dev, "%s: tx is not mmaped.\n", __func__);
		return -EPERM;
	}

	node = vma_get_node(cdev->tx_vma);
	if (node < 0) {
		dev_warn(cdev->dev, "%s: invalid node from tx_vma\n", __func__);
		return -EINVAL;
	}

	while (i < bufs->num_entry) {
		paddr = danipc_hw_fifo_pop_raw(node, DEFAULT_HI_PRIO_B_FIFO);
		if (!paddr)
			break;

		if (!address_in_range(paddr,
				      cdev->tx_region->start,
				      cdev->tx_region->end)) {
			cdev->status.mmap_tx_bad_buf++;
			dev_warn(cdev->dev,
				 "%s: phy_addr %x is out of region range\n",
				 __func__, paddr);
			continue;
		}

		buf = shm_bufpool_find_buf_overlap(&cdev->tx_queue.mmapq,
						   cdev->tx_region,
						   paddr);
		if (buf) {
			cdev->status.mmap_tx_bad_buf++;
			dev_warn(cdev->dev,
				 "%s: addr %x is used by mmap buf(%p/%x)\n",
				 __func__, paddr, buf, buf->offset);
			continue;
		}

		buf = tx_mmap_bufcacheq_get_buf(cdev, paddr);
		if (!buf) {
			cdev->status.mmap_tx_reqbuf_error++;
			dev_warn(cdev->dev,
				 "%s: can't get buf from cache, phy_addr %x\n",
				 __func__, paddr);
			goto err;
		}
		shm_bufpool_put_buf(&cdev->tx_queue.mmapq, buf);
		vaddr = buf_vma_vaddr(buf, cdev->tx_vma);
		BUG_ON(!vaddr);

		dev_dbg(cdev->dev,
			"%s: give buf to user space, phys=%x vaddr=%p\n",
			__func__, paddr, vaddr);

		bufs->entry[i].data = (struct ipc_msg_hdr *)(vaddr) + 1;
		bufs->entry[i].data_len = buf->region->buf_sz -
			sizeof(struct ipc_msg_hdr);
		cdev->status.mmap_tx_reqbuf++;
		i++;
	}

	if (i == 0) {
		cdev->status.mmap_tx_nobuf++;
		ret = -ENOBUFS;
	} else {
		bufs->num_entry = i;
	}

	return ret;
err:
	danipc_hw_fifo_push_raw(node, DEFAULT_HI_PRIO_B_FIFO, paddr);
	return ret;
}

int danipc_cdev_mapped_tx_put_buf(struct danipc_cdev *cdev,
				  struct danipc_bufs *bufs)
{
	struct shm_region *region;
	struct shm_buf *buf;
	phys_addr_t paddr, paddr_buf;
	int i, node, ret = 0;

	if (unlikely(!bufs || !cdev))
		return -EINVAL;

	if (bufs->num_entry > DANIPC_BUFS_MAX_NUM_BUF || !bufs->num_entry)
		return -EINVAL;

	if (!cdev->tx_vma) {
		dev_warn(cdev->dev, "%s: tx is not mmaped.\n", __func__);
		return -EPERM;
	}

	node = vma_get_node(cdev->tx_vma);
	if (node < 0) {
		dev_warn(cdev->dev, "%s: invalid node from tx_vma\n", __func__);
		return -EINVAL;
	}

	region = cdev->tx_region;
	for (i = 0; i < bufs->num_entry; i++) {
		paddr = vma_paddr(cdev->tx_vma, region, bufs->entry[i].data);

		buf = shm_bufpool_find_buf_in_region(&cdev->tx_queue.mmapq,
						     region,
						     paddr);
		if (buf == NULL) {
			dev_warn(cdev->dev,
				 "%s: buffer at phy address %x not in mmapq\n",
				 __func__, paddr);
			ret = -EINVAL;
			continue;
		}

		if (shm_bufpool_del_buf(&cdev->tx_queue.mmapq, buf)) {
			dev_warn(cdev->dev,
				 "%s: vaddr %p not in mmapq\n",
				 __func__, bufs->entry[i].data);
			ret = -EINVAL;
			continue;
		}

		paddr_buf = buf_paddr(buf);

		dev_dbg(cdev->dev, "%s: buf(%p) is returned, paddr=%x\n",
			__func__, buf, paddr_buf);

		danipc_hw_fifo_push_raw(node,
					DEFAULT_HI_PRIO_B_FIFO,
					paddr_buf);

		tx_mmap_bufcacheq_put_buf(cdev, buf);
	}

	return ret;
}

static void danipc_cdev_tx_vma_open(struct vm_area_struct *vma)
{
	struct danipc_cdev *cdev = vma->vm_private_data;

	dev_dbg(cdev->dev, "%s: vma %p\n", __func__, vma);

	if (atomic_add_return(1, &cdev->tx_vma_ref) == 1)
		cdev->tx_vma = vma;
}

static void danipc_cdev_tx_vma_close(struct vm_area_struct *vma)
{
	struct danipc_cdev *cdev = vma->vm_private_data;
	struct tx_queue *pq = &cdev->tx_queue;
	struct shm_buf *buf;
	int node;

	dev_dbg(cdev->dev, "%s: vma %p\n", __func__, vma);

	if (!atomic_dec_and_test(&cdev->tx_vma_ref))
		return;

	node = vma_get_node(cdev->tx_vma);
	while ((buf = shm_bufpool_get_buf(&pq->mmapq))) {
		danipc_hw_fifo_push_raw(node,
					DEFAULT_HI_PRIO_B_FIFO,
					buf_paddr(buf));
		tx_mmap_bufcacheq_put_buf(cdev, buf);
	}

	cdev->tx_vma = NULL;
}

static struct vm_operations_struct danipc_cdev_tx_vma_ops = {
	.open	= danipc_cdev_tx_vma_open,
	.close	= danipc_cdev_tx_vma_close,
};

static int danipc_tx_mmap(struct danipc_cdev *cdev, struct vm_area_struct *vma)
{
	struct shm_region *region;
	unsigned long size = vma->vm_end - vma->vm_start;

	if (cdev->tx_vma) {
		dev_warn(cdev->dev, "%s: tx is already mmaped\n",
			 __func__);
		return -EBUSY;
	}

	if (cdev->tx_region == NULL)
		danipc_cdev_init_tx_region(cdev,
					   vma_get_node(vma),
					   ipc_prio_hi);

	region = cdev->tx_region;
	if (region == NULL)
		return -EPERM;

	if (vma_in_region(vma, region)) {
		dev_dbg(cdev->dev,
			"%s: vma area is too small, start=%lx end=%lx\n",
			__func__, vma->vm_start, vma->vm_end);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_writethroughcache(vma->vm_page_prot);

	if (remap_pfn_range(vma,
			    vma->vm_start,
			    region->start >> PAGE_SHIFT,
			    size,
			    vma->vm_page_prot)) {
		dev_warn(cdev->dev,
			 "%s: remap_pfn_range(%lx/%lx/%lx) failed\n",
			 __func__, vma->vm_start, size, vma->vm_pgoff);
		return -EAGAIN;
	}

	vma->vm_private_data = cdev;
	vma->vm_ops = &danipc_cdev_tx_vma_ops;
	danipc_cdev_tx_vma_open(vma);
	return 0;
}

static int danipc_cdev_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned int minor = iminor(file_inode(file));
	struct danipc_cdev *cdev;
	int node_id;
	int ret;

	if (minor >= DANIPC_MAX_CDEV)
		return -ENODEV;

	cdev = &danipc_driver.cdev[minor];

	dev_dbg(cdev->dev, "%s\n", __func__);

	if (!vma->vm_pgoff)
		node_id = cdev->fifo->node_id;
	else
		node_id = vma_get_node(vma);

	if (node_id < 0) {
		dev_dbg(cdev->dev, "%s: invalid offset 0x%lx\n",
			__func__, vma->vm_pgoff);
		return -EINVAL;
	}

	if (node_id == cdev->fifo->node_id)
		ret = danipc_rx_mmap(cdev, vma);
	else
		ret = danipc_tx_mmap(cdev, vma);

	return ret;
}

static int danipc_cdev_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(file_inode(file));
	struct danipc_cdev *cdev;
	struct danipc_fifo *fifo;
	struct device *dev;
	struct danipc_resource *resource;
	struct danipc_cdev_if_fifo *rx_fifo;
	int i;
	int ret = 0;
	uint32_t off_cdev, sz_cdev;

	if (minor >= DANIPC_MAX_CDEV)
		return -ENODEV;

	cdev = &danipc_driver.cdev[minor];
	fifo = cdev->fifo;
	dev = cdev->dev;
	resource = &danipc_driver.shm_res[fifo->node_id];
	rx_fifo = cdev->rx_fifo;

	dev_dbg(cdev->dev, "%s\n", __func__);

	ret = acquire_local_fifo(fifo, cdev);
	if (ret) {
		dev_warn(cdev->dev, "%s: fifo(%s) is busy\n",
			 __func__, fifo->probe_info->res_name);
		goto out;
	}

	cdev->rx_region = shm_region_create(resource->start,
					    resource->base,
					    resource->size,
					    IPC_BUF_SIZE_MAX,
					    0,
					    resource->size / IPC_BUF_SIZE_MAX);
	if (cdev->rx_region == NULL) {
		ret = -ENOBUFS;
		goto err;
	}

	for (i = 0; i < max_ipc_prio; i++, rx_fifo++) {
		shm_bufpool_init(&rx_fifo->rx_queue.freeq);
		shm_bufpool_init(&rx_fifo->rx_queue.recvq);
		shm_bufpool_init(&rx_fifo->rx_queue.bq);
		shm_bufpool_init(&rx_fifo->rx_queue.mmapq);
	}

	/* The netdev interface use 128 buffer for each priority fifo.
	 * The cdev interface will use these buffers. The remaining
	 * share memory space will be equally divided among the
	 * cdev interface for high priority fifo
	 */
	off_cdev = IPC_BUFS_SZ_PER_IF_FIFO * danipc_driver.num_l_if_fifo;
	sz_cdev = (resource->size - off_cdev) / DANIPC_MAX_CDEV;
	off_cdev += sz_cdev * minor;

	for (i = 0, rx_fifo = cdev->rx_fifo; i < max_ipc_prio; i++, rx_fifo++) {
		struct rx_queue *rx_queue = &rx_fifo->rx_queue;
		struct danipc_if_fifo *if_fifo = rx_fifo->if_fifo;

		if (!if_fifo->probed)
			continue;

		ret = shm_bufpool_acquire_region(&rx_queue->freeq,
						 cdev->rx_region,
						 (if_fifo->unit_num *
						  IPC_BUFS_SZ_PER_IF_FIFO),
						 IPC_BUFS_SZ_PER_IF_FIFO);
		if (ret)
			goto err_release_region;

		if (i != ipc_prio_hi)
			continue;

		ret = shm_bufpool_acquire_region(&rx_queue->freeq,
						 cdev->rx_region,
						 off_cdev,
						 sz_cdev);
		if (ret)
			goto err_release_region;
		off_cdev += sz_cdev;
	}

	danipc_cdev_rx_buf_init(cdev);

	init_waitqueue_head(&cdev->rx_wq);

	shm_bufpool_init(&cdev->tx_queue.mmapq);
	shm_bufpool_init(&cdev->tx_queue.mmap_bufcacheq);

	ret = request_irq(fifo->irq, danipc_cdev_interrupt, 0,
			  dev->kobj.name, cdev);
	if (ret) {
		dev_err(cdev->dev, "%s: request irq(%d) failed, fifo(%s)\n",
			__func__, fifo->irq, fifo->probe_info->res_name);
		goto err_release_ul_buf;
	}

	danipc_fifo_init_irq(fifo);

	file->private_data = cdev;

	atomic_set(&cdev->rx_vma_ref, 0);
	atomic_set(&cdev->tx_vma_ref, 0);
out:
	return ret;

err_release_ul_buf:
	danipc_cdev_rx_buf_release(cdev);
	for (i = 0, rx_fifo = cdev->rx_fifo; i < max_ipc_prio; i++, rx_fifo++)
		shm_bufpool_release(&rx_fifo->rx_queue.freeq);
err_release_region:
	shm_region_release(cdev->rx_region);
err:
	release_local_fifo(fifo, cdev);
	return ret;

}

static int danipc_cdev_release(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(file_inode(file));
	struct danipc_cdev *cdev;
	struct danipc_fifo *fifo;
	int ret = 0;
	int i;

	if (minor >= DANIPC_MAX_CDEV)
		return -ENODEV;

	cdev = &danipc_driver.cdev[minor];
	fifo = cdev->fifo;

	dev_dbg(cdev->dev, "%s\n", __func__);

	danipc_fifo_disable_irq(fifo);
	free_irq(fifo->irq, cdev);
	danipc_poll_ctl_stop(&cdev->poll_ctl);

	for (i = 0; i < max_ipc_prio; i++) {
		shm_bufpool_release(&cdev->rx_fifo[i].rx_queue.freeq);
		shm_bufpool_release(&cdev->rx_fifo[i].rx_queue.recvq);
	}
	shm_region_release(cdev->rx_region);
	shm_region_release(cdev->tx_region);
	cdev->rx_region = NULL;
	cdev->tx_region = NULL;

	release_local_fifo(cdev->fifo, cdev);
	file->private_data = NULL;
	return ret;
}

static const struct file_operations danipc_cdevs_fops = {
	.read		= danipc_cdev_read,
	.write		= danipc_cdev_write,
	.poll		= danipc_cdev_poll,
	.unlocked_ioctl = danipc_cdev_ioctl,
	.mmap		= danipc_cdev_mmap,
	.open		= danipc_cdev_open,
	.release	= danipc_cdev_release,
};

static struct class *danipc_class;

static int cdev_create(struct danipc_cdev *cdev,
		       struct danipc_fifo *fifo,
		       int minor)
{
	int prio;

	cdev->dev = device_create(danipc_class, NULL,
				  MKDEV(DANIPC_MAJOR, minor),
				  NULL, DANIPC_CDEV_NAME "%d", minor);
	if (cdev->dev == NULL) {
		pr_err("%s: failed to create the device %s%d\n",
		       __func__, DANIPC_CDEV_NAME, minor);
		return -ENOMEM;
	}

	cdev->drvr = &danipc_driver;
	cdev->fifo = fifo;
	cdev->minor = minor;
	spin_lock_init(&cdev->rx_lock);

	for (prio = 0; prio < max_ipc_prio; prio++) {
		struct danipc_cdev_if_fifo *rx_fifo = &cdev->rx_fifo[prio];

		rx_fifo->cdev = cdev;
		rx_fifo->if_fifo = &fifo->if_fifo[prio];
		rx_fifo->prio = prio;
	}

	return danipc_poll_ctl_init(&cdev->poll_ctl,
				    danipc_cdev_rx_poll,
				    (unsigned long)cdev,
				    DEFAULT_POLL_INTERVAL_IN_US);
}

static void cdev_remove(struct danipc_cdev *cdev)
{
	struct device *dev = (cdev) ? cdev->dev : NULL;

	if (dev == NULL)
		return;

	device_destroy(danipc_class, MKDEV(DANIPC_MAJOR, cdev->minor));
	cdev->dev = NULL;
}

static int danipc_cdev_init(void)
{
	struct danipc_drvr *pdrv = &danipc_driver;
	struct danipc_cdev *cdev = pdrv->cdev;
	int minor = 0;
	int ret = 0;

	if (register_chrdev(DANIPC_MAJOR, DANIPC_CDEV_NAME,
			    &danipc_cdevs_fops)) {
		pr_err("%s: register_chrdev failed\n", __func__);
		return -ENODEV;
	}

	danipc_class = class_create(THIS_MODULE, DANIPC_CDEV_NAME);
	if (IS_ERR(danipc_class)) {
		pr_err("%s: failed to create the class\n", __func__);
		unregister_chrdev(DANIPC_MAJOR, DANIPC_CDEV_NAME);
		return PTR_ERR(danipc_class);
	}

	for (minor = 0; minor < DANIPC_MAX_CDEV && !ret; minor++, cdev++)
		ret = cdev_create(cdev, &pdrv->lfifo[minor], minor);

	return ret;
}

static void danipc_cdev_cleanup(void)
{
	int minor = 0;

	for (minor = 0; minor < DANIPC_MAX_CDEV; minor++)
		cdev_remove(&danipc_driver.cdev[minor]);
	class_destroy(danipc_class);
	unregister_chrdev(DANIPC_MAJOR, DANIPC_CDEV_NAME);
}

/* DANIPC DEBUGFS for character device interface */
static void danipc_cdev_if_fifo_show_queue_info(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_cdev_if_fifo *rx_fifo =
		(struct danipc_cdev_if_fifo *)hdlr->data;
	struct rx_queue *rx_queue = &rx_fifo->rx_queue;

	seq_puts(s, "\nReceiving:\n");
	seq_printf(s, "%-25s: %u\n", "recv_queue", rx_queue->recvq.count);
	seq_printf(s, "%-25s: %u\n",
		   "recv_queue_hi", rx_queue->status.recvq_hi);
	seq_printf(s, "%-25s: %u\n", "fifo_b_queue", rx_queue->bq.count);
	seq_printf(s, "%-25s: %u\n",
		   "fifo_b_queue_lo", rx_queue->status.bq_lo);
	seq_printf(s, "%-25s: %u\n", "free_queue", rx_queue->freeq.count);
	seq_printf(s, "%-25s: %u\n", "free_queue_lo",
		   rx_queue->status.freeq_lo);
	seq_printf(s, "%-25s: %u\n", "mmap_queue", rx_queue->mmapq.count);

	if (rx_fifo->prio == ipc_prio_hi) {
		struct tx_queue *tx_queue = &rx_fifo->cdev->tx_queue;

		seq_puts(s, "\nSending:\n");
		seq_printf(s, "%-25s: %u\n", "mmap_queue",
			   tx_queue->mmapq.count);
		seq_printf(s, "%-25s: %u\n", "mmap_bufcache_queue",
			   tx_queue->mmap_bufcacheq.count);
	}
}

static ssize_t danipc_cdev_if_fifo_reset_queue_info(struct file *filep,
						    const char __user *ubuf,
						    size_t cnt,
						    loff_t *ppos)
{
	struct seq_file *m = filep->private_data;
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)m->private;
	struct danipc_cdev_if_fifo *rx_fifo =
		(struct danipc_cdev_if_fifo *)hdlr->data;
	struct danipc_cdev *cdev = rx_fifo->cdev;
	unsigned long flags;

	spin_lock_irqsave(&cdev->rx_lock, flags);
	reset_rx_queue_status(&rx_fifo->rx_queue);
	spin_unlock_irqrestore(&cdev->rx_lock, flags);

	return cnt;
}

static void danipc_cdev_if_fifo_show(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_cdev_if_fifo *rx_fifo =
		(struct danipc_cdev_if_fifo *)hdlr->data;

	danipc_dbgfs_dump_if_fifo(s, rx_fifo->if_fifo);
}

static struct danipc_dbgfs cdev_if_fifo_dbgfs[] = {
	DBGFS_NODE("queue", 0666, danipc_cdev_if_fifo_show_queue_info,
		   danipc_cdev_if_fifo_reset_queue_info),
	DBGFS_NODE("interface", 0644, danipc_cdev_if_fifo_show, NULL),
	DBGFS_NODE_LAST
};

static void danipc_cdev_show_status(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_cdev *cdev = (struct danipc_cdev *)hdlr->data;
	struct danipc_cdev_status *stats = &cdev->status;
	struct ipc_msg_err_stats *err_stats = &cdev->status.rx_err_stats;

	seq_printf(s, "%-25s: %u\n", "rx", stats->rx);
	seq_printf(s, "%-25s: %u\n", "rx_bytes", stats->rx_bytes);
	seq_printf(s, "%-25s: %u\n", "rx_drop", stats->rx_drop);
	seq_printf(s, "%-25s: %u\n", "rx_no_buf", stats->rx_no_buf);
	seq_printf(s, "%-25s: %u\n", "rx_error", stats->rx_error);
	seq_printf(s, "%-25s: %u\n", "rx_zero_len_msg", err_stats->zlen_msg);
	seq_printf(s, "%-25s: %u\n", "rx_oversize_msg",
		   err_stats->oversize_msg);
	seq_printf(s, "%-25s: %u\n", "rx_invalid_aid_msg",
		   err_stats->inval_msg);
	seq_printf(s, "%-25s: %u\n", "rx_chained_msg", err_stats->chained_msg);

	seq_printf(s, "%-25s: %u\n", "mmap_rx", stats->mmap_rx);
	seq_printf(s, "%-25s: %u\n", "mmap_rx_done", stats->mmap_rx_done);
	seq_printf(s, "%-25s: %u\n", "mmap_rx_error", stats->mmap_rx_error);

	seq_printf(s, "%-25s: %u\n", "tx", stats->tx);
	seq_printf(s, "%-25s: %u\n", "tx_bytes", stats->tx_bytes);
	seq_printf(s, "%-25s: %u\n", "tx_drop", stats->tx_drop);
	seq_printf(s, "%-25s: %u\n", "tx_error", stats->tx_error);
	seq_printf(s, "%-25s: %u\n", "tx_no_buf", stats->tx_no_buf);

	seq_printf(s, "%-25s: %u\n", "mmap_tx", stats->mmap_tx);
	seq_printf(s, "%-25s: %u\n", "mmap_tx_reqbuf", stats->mmap_tx_reqbuf);
	seq_printf(s, "%-25s: %u\n", "mmap_tx_reqbuf_error",
		   stats->mmap_tx_reqbuf_error);
	seq_printf(s, "%-25s: %u\n", "mmap_tx_nobuf", stats->mmap_tx_nobuf);
	seq_printf(s, "%-25s: %u\n", "mmap_tx_error", stats->mmap_tx_error);

	seq_printf(s, "%-25s: %u\n", "mmap_tx_bad_buf", stats->mmap_tx_bad_buf);
}

static void danipc_cdev_show_vma(struct seq_file *s,
				 struct vm_area_struct *vma)
{
	seq_printf(s, "%-25s: %lx\n", "vma_start", vma->vm_start);
	seq_printf(s, "%-25s: %lx\n", "vma_end", vma->vm_end);
}

static void danipc_cdev_show_region(struct seq_file *s,
				    struct shm_region *region)
{
	seq_printf(s, "%-25s: %x\n", "phy_start", region->start);
	seq_printf(s, "%-25s: %x\n", "phy_end", region->end);
	seq_printf(s, "%-25s: %u\n", "num_buf", region->buf_num);
	seq_printf(s, "%-25s: %u\n", "buf_sz", region->buf_sz);
	seq_printf(s, "%-25s: %u\n", "buf_headroom", region->buf_headroom_sz);
	seq_printf(s, "%-25s: %u\n", "real_buf_sz", region->real_buf_sz);
	seq_printf(s, "%-25s: %s\n", "direct_buf_mapping",
		   (region->dir_buf_map) ? "true" : "false");
}

static void danipc_cdev_show_mmap_mapping(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_cdev *cdev = (struct danipc_cdev *)hdlr->data;

	if (cdev->rx_vma) {
		seq_puts(s, "\nRX_MMAP:\n");
		danipc_cdev_show_vma(s, cdev->rx_vma);
		danipc_cdev_show_region(s, cdev->rx_region);
	}
	if (cdev->tx_vma) {
		seq_puts(s, "\nTX_MMAP:\n");
		danipc_cdev_show_vma(s, cdev->tx_vma);
		danipc_cdev_show_region(s, cdev->tx_region);
	}
}

static void danipc_cdev_show_poll_interval(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_cdev *cdev = (struct danipc_cdev *)hdlr->data;

	seq_printf(s, "%lldus\n", ktime_to_us(cdev->poll_ctl.timer_intval));
}

static ssize_t danipc_cdev_set_poll_interval(struct file *filep,
					     const char __user *ubuf,
					     size_t cnt,
					     loff_t *ppos)
{
	struct seq_file *m = filep->private_data;
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)m->private;
	struct danipc_cdev *cdev = (struct danipc_cdev *)hdlr->data;
	uint32_t val;

	if (kstrtouint_from_user(ubuf, cnt, 0, &val))
		return -EINVAL;
	if (!val)
		return -EINVAL;

	cdev->poll_ctl.timer_intval = ns_to_ktime(val * 1000);
	return cnt;
}

static void danipc_cdev_show_poll_count(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_cdev *cdev = (struct danipc_cdev *)hdlr->data;

	seq_printf(s, "%llu\n", cdev->poll_ctl.poll_cnt);
}

static void danipc_cdev_show_fifo(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_cdev *cdev = (struct danipc_cdev *)hdlr->data;

	danipc_dbgfs_dump_fifo(s, cdev->fifo);
}

static struct danipc_dbgfs cdev_dbgfs[] = {
	DBGFS_NODE("fifo", 0444, danipc_cdev_show_fifo, NULL),
	DBGFS_NODE("status", 0444, danipc_cdev_show_status, NULL),
	DBGFS_NODE("mmap-mapping", 0444, danipc_cdev_show_mmap_mapping, NULL),
	DBGFS_NODE("poll-intval", 0644, danipc_cdev_show_poll_interval,
		   danipc_cdev_set_poll_interval),
	DBGFS_NODE("poll", 0644, danipc_cdev_show_poll_count, NULL),
	DBGFS_NODE_LAST
};

static void danipc_dbgfs_cdev_remove_dent(struct danipc_cdev *cdev)
{
	int prio;

	for (prio = 0; prio < max_ipc_prio; prio++)
		danipc_dbgfs_remove_dent(&cdev->rx_fifo[prio].dent);

	danipc_dbgfs_remove_dent(&cdev->dent);
}

int danipc_dbgfs_cdev_init(void)
{
	struct danipc_drvr *drvr = &danipc_driver;
	struct dentry *dent;
	int ret = 0;
	int i;

	dent = debugfs_create_dir("intf_cdev", drvr->dirent);
	if (IS_ERR(dent)) {
		pr_err("%s: failed to create cdev directory\n", __func__);
		return PTR_ERR(dent);
	}

	for (i = 0; i < DANIPC_MAX_CDEV; i++) {
		struct danipc_cdev *cdev = &drvr->cdev[i];
		struct danipc_cdev_if_fifo *rx_fifo = cdev->rx_fifo;
		int prio;

		ret = danipc_dbgfs_create_dent(&cdev->dent,
					       dent,
					       cdev->dev->kobj.name,
					       cdev_dbgfs,
					       cdev);
		if (ret) {
			pr_err("%s: failed to create dbgfs\n", __func__);
			break;
		}

		for (prio = 0; prio < max_ipc_prio && !ret; prio++, rx_fifo++) {
			if (!rx_fifo->if_fifo->probed)
				continue;
			ret = danipc_dbgfs_create_dent(
				&rx_fifo->dent,
				cdev->dent.dent,
				(rx_fifo->prio == ipc_prio_hi) ?
				"fifo(hi_prio)" : "fifo(lo_prio)",
				cdev_if_fifo_dbgfs,
				rx_fifo);
		}
		if (ret) {
			danipc_dbgfs_cdev_remove_dent(cdev);
			break;
		}
	}

	return ret;
}

void danipc_dbgfs_cdev_remove(void)
{
	struct danipc_drvr *drvr = &danipc_driver;
	int i;

	for (i = 0; i < DANIPC_MAX_CDEV; i++) {
		struct danipc_cdev *cdev = &drvr->cdev[i];

		danipc_dbgfs_cdev_remove_dent(cdev);
	}
}

/* DANIPC DEBUGFS FIFO */
static void danipc_dump_fifo_info(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_fifo *fifo = (struct danipc_fifo *)hdlr->data;
	static const char *format = "%-20s: %-d\n";

	seq_printf(s, "FIFO %s:\n", fifo->probe_info->ifname);
	seq_printf(s, format, "Irq", fifo->irq);
	seq_printf(s, format, "If Index", fifo->idx);
	seq_printf(s, format, "HW fifo Index", fifo->node_id);
}

static void danipc_dump_fifo_register(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_fifo *fifo = (struct danipc_fifo *)hdlr->data;
	void __iomem *base = danipc_io_base(fifo->node_id);
	int i, clk_domain;
	uint32_t val;

	/* FIFO Status*/
	for (i = 0; i < MAX_IPC_FIFO_PER_CDU; i++) {
		uint32_t c;

		val = danipc_hw_reg_read(base, FIFO_STATUS(i));
		c = danipc_hw_reg_read(base, FIFO_COUNTER(i));
		seq_printf(s, "FIFO %d: STATUS=%08x Count=%u", i, val, c);
	}

	for (clk_domain = 0; clk_domain < CLK_DOMAIN_PER_CDU; clk_domain++) {
		val = danipc_hw_reg_read(base,
					 (clk_domain) ? CDU_INT1_MASK :
					 CDU_INT0_MASK);
		seq_printf(s, "IRQ_MASK(clk_domain/%d): %08x\n",
			   clk_domain, val);

		val = danipc_hw_reg_read(base,
					 (clk_domain) ? CDU_INT1_ENABLE :
					 CDU_INT0_ENABLE);
		seq_printf(s, "IRQ_ENABLE(clk_domain/%d): %08x\n",
			   clk_domain, val);

		val = danipc_hw_reg_read(base,
					 (clk_domain) ? CDU_INT1_STATUS :
					 CDU_INT0_STATUS);
		seq_printf(s, "IRQ_STATUS(clk_domain/%d): %08x\n",
			   clk_domain, val);

		val = danipc_hw_reg_read(base,
					 (clk_domain) ? CDU_INT1_RAW_STATUS :
					 CDU_INT0_RAW_STATUS);
		seq_printf(s, "IRQ_RAW_STATUS(clk_domain/%d): %08x\n",
			   clk_domain, val);
	}

	val = danipc_hw_reg_read(base, FIFO_THR_AF_CFG);
	seq_printf(s, "THR_AF_CFG: %08x\n", val);

	val = danipc_hw_reg_read(base, FIFO_THR_AE_CFG);
	seq_printf(s, "THR_AE_CFG: %08x\n", val);
}

static struct danipc_dbgfs fifo_dbgfs[] = {
	DBGFS_NODE("fifo_info", 0444, danipc_dump_fifo_info, NULL),
	DBGFS_NODE("fifo_register", 0444, danipc_dump_fifo_register, NULL),
	DBGFS_NODE_LAST
};

static int danipc_dbgfs_fifo_init(void)
{
	struct danipc_drvr *drvr = &danipc_driver;
	struct dentry *dent;
	int ret = 0;
	int i;

	dent = debugfs_create_dir("fifo", drvr->dirent);
	if (IS_ERR(dent)) {
		pr_err("%s: failed to create cdev directory\n", __func__);
		return PTR_ERR(dent);
	}

	for (i = 0; i < drvr->num_lfifo; i++) {
		struct danipc_fifo *fifo = &drvr->lfifo[i];

		ret = danipc_dbgfs_create_dent(&fifo->dent,
					       dent,
					       fifo->probe_info->ifname,
					       fifo_dbgfs,
					       fifo);
		if (ret) {
			ret = -ENOMEM;
			pr_err("%s: failed to allocate dbgfs\n", __func__);
			break;
		}
	}

	return ret;
}

void danipc_dbgfs_fifo_remove(void)
{
	struct danipc_drvr *drvr = &danipc_driver;
	int i;

	for (i = 0; i < drvr->num_lfifo; i++)
		danipc_dbgfs_remove_dent(&drvr->lfifo[i].dent);
}

void danipc_dbgfs_dump_fifo(struct seq_file *s, struct danipc_fifo *fifo)
{
	struct danipc_probe_info *probe_info = fifo->probe_info;

	seq_printf(s, "index:           %u\n", fifo->idx);
	seq_printf(s, "node_id:         %u\n", fifo->node_id);
	seq_printf(s, "register base:   %08x\n",
		   danipc_driver.io_res[fifo->node_id].start);
	seq_printf(s, "irq_clk_domain:  %u\n", probe_info->irq_clk_domain);
	seq_printf(s, "tcsr_mux:        %u\n", probe_info->mux_enable);
}

void danipc_dbgfs_dump_if_fifo(struct seq_file *s,
			       struct danipc_if_fifo *if_fifo)
{
	if (!if_fifo->probed)
		return;

	seq_printf(s, "%-25s: %u\n", "unit", if_fifo->unit_num);
	seq_printf(s, "%-25s: %u\n", "m-fifo", if_fifo->m_fifo_idx);
	seq_printf(s, "%-25s: %u\n", "b-fifo", if_fifo->b_fifo_idx);
	seq_printf(s, "%-25s: 0x%08x\n", "irq_mask", if_fifo->irq_mask);
}

/* DANIPC DEBUGFS ROOT interface */
static void danipc_dump_fifo_mmap(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_drvr *drvr = (struct danipc_drvr *)hdlr->data;
	int i;

	if (drvr->ndev_active == 0) {
		seq_puts(s, "\n\nNo active device!\n\n");
		return;
	}

	seq_puts(s, "\n\nCPU Fifo memory map:\n\n");
	seq_puts(s, " -------------------------------------------------\n");
	seq_puts(s, "| CPU ID | HW Address | Virtual Address | Size    |\n");
	seq_puts(s, " -------------------------------------------------\n");
	for (i = 0; i < PLATFORM_MAX_NUM_OF_NODES; i++) {
		struct danipc_resource *res = &drvr->io_res[i];

		if (res->size)
			seq_printf(s, "| %-6d | 0x%-8p | 0x%-12p | %-8d |\n", i,
				   (void *)res->start, res->base, res->size);
	}
}

static void danipc_dump_resource_map(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_drvr *drvr = (struct danipc_drvr *)hdlr->data;
	struct danipc_resource *res;
	static const char *format = "| %-11s | 0x%-11p | 0x%-14p | %-6d |\n";

	if (drvr->ndev_active == 0) {
		seq_puts(s, "\n\nNo active device!\n\n");
		return;
	}

	seq_puts(s, "\n\nResource map:\n\n");
	seq_puts(s, " --------------------------------------------------------\n");
	seq_puts(s, "| Name        | Phys Address  | Virtual Address | Size    |\n");
	seq_puts(s, " --------------------------------------------------------\n");

	res = &drvr->res[IPC_BUFS_RES];
	seq_printf(s, format, "q6ul_ipcbuf", res->start, res->base, res->size);

	res = &drvr->res[AGENT_TABLE_RES];
	seq_printf(s, format, "Agnt_tbl", res->start, res->base, res->size);

	res = &drvr->res[KRAIT_IPC_MUX_RES];
	seq_printf(s, format, "Intren_map", res->start, res->base, res->size);
}

static void danipc_dump_local_agents(struct seq_file *s, uint8_t cpuid)
{
	int i;
	struct agent_entry *entry = (struct agent_entry *)
		(danipc_res_base(AGENT_TABLE_RES));

	entry += cpuid * MAX_LOCAL_AGENT;

	for (i = 0; i < MAX_LOCAL_AGENT; i++) {
		if (entry->name[0] != '\0')
			seq_printf(s, "| %-2d | %-35.32s |\n",
				   (cpuid * MAX_LOCAL_AGENT) + i,
				   entry[i].name);
	}
}

static void danipc_dump_drvr_info(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_drvr *drvr = (struct danipc_drvr *)hdlr->data;
	struct agent_entry *entry = (struct agent_entry *)
		(danipc_res_base(AGENT_TABLE_RES));
	int i;

	seq_puts(s, "\n\nDanipc driver status:\n\n");
	seq_printf(s, "%-20s: %-2d\n", "Devices found", drvr->ndev);
	seq_printf(s, "%-20s: %-2d\n", "Devices active", drvr->ndev_active);

	if (drvr->ndev_active == 0)
		return;

	seq_puts(s, "\n\nActive remote agents:\n");
	seq_puts(s, " -------------------------------------------\n");
	seq_puts(s, "| AID | Agent name                          |\n");
	seq_puts(s, " -------------------------------------------\n");

	for (i = 0; i < MAX_AGENTS; i++) {
		if (DANIPC_IS_AGENT_DISCOVERED(i, drvr->dst_aid))
			seq_printf(s, "| %-2d | %-35.32s |\n", i,
				   entry[i].name);
	}

	for (i = 0; i < drvr->ndev; i++) {
		struct danipc_if *intf = drvr->if_list[i];

		if (!netif_running(intf->dev))
			continue;

		seq_printf(s,
			   "\n\nActive %s interface local agents cpuid(%d):\n",
			   intf->dev->name, intf->fifo->node_id);
		seq_puts(s, " -------------------------------------------\n");
		seq_puts(s, "| AID | Agent name                          |\n");
		seq_puts(s, " -------------------------------------------\n");
		danipc_dump_local_agents(s, intf->fifo->node_id);
	}
}

static struct danipc_dbgfs drvr_dbgfs[] = {
	DBGFS_NODE("fifo_mem_map", 0444, danipc_dump_fifo_mmap, NULL),
	DBGFS_NODE("resource_map", 0444, danipc_dump_resource_map, NULL),
	DBGFS_NODE("driver_info", 0444, danipc_dump_drvr_info, NULL),
	DBGFS_NODE_LAST
};

static int danipc_dbgfs_root_init(void)
{
	struct danipc_drvr *drvr = &danipc_driver;

	drvr->dirent = danipc_dbgfs_create_dir(NULL,
					       "danipc",
					       drvr_dbgfs,
					       drvr);
	if (IS_ERR(drvr->dirent)) {
		pr_err("%s: Failed to create danipc debugfs\n", __func__);
		return PTR_ERR(drvr->dirent);
	}
	return 0;
}

static void danipc_dbgfs_root_remove(void)
{
	struct danipc_drvr *drvr = &danipc_driver;

	debugfs_remove_recursive(drvr->dirent);
	drvr->dirent = NULL;
}

/* DANIPC DEBUGFS */
static int danipc_dbgfs_show(struct seq_file *s, void *data)
{
	struct dbgfs_hdlr *dbgfshdlr = (struct dbgfs_hdlr *)s->private;

	dbgfshdlr->display(s);

	return 0;
}

static ssize_t danipc_dbgfs_write(struct file *filp, const char __user *ubuf,
				  size_t cnt, loff_t *ppos)
{
	struct seq_file *m = filp->private_data;
	struct dbgfs_hdlr *dbgfshdlr = (struct dbgfs_hdlr *)m->private;

	if (dbgfshdlr->write)
		return dbgfshdlr->write(filp, ubuf, cnt, ppos);

	return -EINVAL;
}

static int danipc_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, danipc_dbgfs_show, inode->i_private);
}

static const struct file_operations danipc_dbgfs_ops = {
	.open = danipc_dbgfs_open,
	.release = single_release,
	.read = seq_read,
	.write = danipc_dbgfs_write,
	.llseek = seq_lseek,
};

struct dentry *danipc_dbgfs_create_dir(struct dentry *parent_dent,
				       const char *dir_name,
				       struct danipc_dbgfs *nodes,
				       void *private_data)
{
	struct dentry *dent, *ent;

	if (!dir_name || !nodes) {
		pr_err("%s: invalid argument\n", __func__);
		return NULL;
	}

	dent = debugfs_create_dir(dir_name, parent_dent);
	if (IS_ERR(dent)) {
		pr_err("%s: failed to create directory node(%s)\n",
		       __func__, dir_name);
		return NULL;
	}

	while (nodes->fname) {
		ent = debugfs_create_file(nodes->fname,
					  nodes->mode,
					  dent,
					  &nodes->dbghdlr,
					  &danipc_dbgfs_ops);
		if (ent == NULL) {
			pr_err("%s: failed to create node(%s)\n",
			       __func__, nodes->fname);
			return NULL;
		}
		nodes->dbghdlr.data = private_data;
		nodes++;
	}

	return dent;
}

int danipc_dbgfs_create_dent(struct danipc_dbgfs_dent *dbgfs_dent,
			     struct dentry *parent_dent,
			     const char *dir_name,
			     struct danipc_dbgfs *nodes,
			     void *private_data)
{
	struct danipc_dbgfs *dbgfs;
	struct dentry *dent;
	int dbgfs_size = 0, i = 0;

	if (!dbgfs_dent || !dir_name || !nodes)
		return -EINVAL;

	while (nodes[i++].fname)
		dbgfs_size += sizeof(struct danipc_dbgfs);

	dbgfs = kzalloc((dbgfs_size + sizeof(struct danipc_dbgfs)), GFP_KERNEL);
	if (dbgfs == NULL)
		return -ENOMEM;

	memcpy(dbgfs, nodes, dbgfs_size);

	dent = danipc_dbgfs_create_dir(parent_dent, dir_name,
				       dbgfs, private_data);
	if (dent == NULL) {
		kfree(dbgfs);
		return -ENOMEM;
	}

	dbgfs_dent->dbgfs = dbgfs;
	dbgfs_dent->dent = dent;
	return 0;
}

void danipc_dbgfs_remove_dent(struct danipc_dbgfs_dent *dbgfs_dent)
{
	if (!dbgfs_dent)
		return;

	debugfs_remove_recursive(dbgfs_dent->dent);
	kfree(dbgfs_dent->dbgfs);
	dbgfs_dent->dent = NULL;
	dbgfs_dent->dbgfs = NULL;
}

static void __init danipc_dbgfs_init(void)
{
	if (danipc_dbgfs_root_init()) {
		pr_err("%s: Failed to create root debugfs node\n", __func__);
		goto done;
	}
	if (danipc_dbgfs_fifo_init()) {
		pr_err("%s: Failed to create fifo debugfs node\n", __func__);
		goto done;
	}
	if (danipc_dbgfs_netdev_init()) {
		pr_err("%s: Failed to create netdev debugfs\n", __func__);
		goto done;
	}
	if (danipc_dbgfs_cdev_init()) {
		pr_err("%s: Failed to create netdev debugfs\n", __func__);
		goto done;
	}

done:
	return;
}

static void danipc_dbgfs_remove(void)
{
	danipc_dbgfs_netdev_remove();
	danipc_dbgfs_cdev_remove();
	danipc_dbgfs_fifo_remove();
	danipc_dbgfs_root_remove();
}

static int danipc_probe(struct platform_device *pdev)
{
	struct danipc_drvr	*pdrv = &danipc_driver;
	int			rc = -ENOMEM;
	static const char	*regs[PLATFORM_MAX_NUM_OF_NODES] = {
		"phycpu0_ipc", "phycpu1_ipc", "phycpu2_ipc", "phycpu3_ipc",
		"phydsp0_ipc", "phydsp1_ipc", "phydsp2_ipc", NULL,
		"apps_ipc_data", "qdsp6_0_ipc", "qdsp6_1_ipc",
		"qdsp6_2_ipc", "qdsp6_3_ipc", "apps_ipc_pcap",
		"apps_hex_ipc_log", NULL,
	};
	static const char	*resource[RESOURCE_NUM] = {
		"ipc_bufs", "agent_table", "apps_ipc_intr_en"
	};
	static const char	*shm_names[PLATFORM_MAX_NUM_OF_NODES] = {
		"qcom,phycpu0-shm-size", "qcom,phycpu1-shm-size",
		"qcom,phycpu2-shm-size", "qcom,phycpu3-shm-size",
		"qcom,phydsp0-shm-size", "qcom,phydsp1-shm-size",
		"qcom,phydsp2-shm-size", NULL, "qcom,apps-shm-size",
		"qcom,qdsp6-0-shm-size", "qcom,qdsp6-1-shm-size",
		"qcom,qdsp6-2-shm-size", "qcom,qdsp6-3-shm-size",
		NULL, NULL, NULL
	};

	rc = parse_resources(pdev, regs, resource, shm_names);
	if (rc)
		goto err_probe;

	danipc_ll_init(pdrv);

	rc = danipc_probe_lfifo(pdev, regs);
	if (rc)
		goto err_ll_cleanup;

	rc = danipc_netdev_init(pdev);
	if (rc)
		goto err_netdev_cleanup;

	rc = danipc_cdev_init();
	if (rc)
		goto err_cdev_cleanup;

	danipc_dbgfs_init();

	return 0;
err_cdev_cleanup:
	danipc_cdev_cleanup();
err_netdev_cleanup:
	danipc_netdev_cleanup(pdev);
err_ll_cleanup:
	danipc_ll_cleanup(&danipc_driver);
err_probe:
	return rc;
}

int danipc_remove(struct platform_device *pdev)
{
	danipc_dbgfs_remove();
	danipc_netdev_cleanup(pdev);
	danipc_cdev_cleanup();
	danipc_ll_cleanup(&danipc_driver);
	if (danipc_driver.proc_map && danipc_driver.proc_map_entry) {
		int i;

		for (i = 0; i < danipc_driver.proc_map_entry; i++)
			iounmap(danipc_driver.proc_map[i].base);
		kfree(danipc_driver.proc_map);
		danipc_driver.proc_map = NULL;
		danipc_driver.proc_map_entry = 0;
	}
	return 0;
}

static struct of_device_id danipc_ids[] = {
	{
		.compatible = "qcom,danipc",
	},
	{}
};

static struct platform_driver danipc_platform_driver = {
	.probe   = danipc_probe,
	.remove  = danipc_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name	= "danipc",
		.of_match_table = danipc_ids,
	},
};

static int __init danipc_init_module(void)
{
	return platform_driver_register(&danipc_platform_driver);
}

static void __exit danipc_exit_module(void)
{
	platform_driver_unregister(&danipc_platform_driver);
}

module_init(danipc_init_module);
module_exit(danipc_exit_module);
