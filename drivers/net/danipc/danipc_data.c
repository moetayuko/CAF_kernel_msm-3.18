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
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/debugfs.h>
#include <asm/cacheflush.h>
#include <net/arp.h>

#include "ipc_api.h"

#include "danipc_k.h"
#include "danipc_lowlevel.h"

#define HADDR_CB_OFFSET		40
#define TX_TIMER_INTERVAL	(HZ/10)

#define IPC_MSG_TYPE_PTR 0x10

struct fapi_msg_hdr {
	uint16_t eth_type;
	uint16_t type;
	uint8_t sec_id;
	uint8_t pad1;
	uint32_t seq;
	uint16_t size;
	uint8_t frag;
	uint8_t platform;
	uint64_t tmstmp;
} __packed;

struct ipc_fapi_msg_hdr {
	struct fapi_msg_hdr hdr;
	void *msg_ptr;
} __packed;

static inline struct danipc_pair *danipc_skb_cb(const struct sk_buff *skb)
{
	return ((struct danipc_pair *)(&((skb)->cb[HADDR_CB_OFFSET])));
}

static inline void *get_virt_addr(phys_addr_t phy_addr)
{
	struct danipc_resource *map = danipc_driver.proc_map;
	int i;

	for (i = 0; i < danipc_driver.proc_map_entry; i++) {
		if (phy_addr >= map[i].start) {
			phys_addr_t offset = phy_addr - map[i].start;

			if (offset < map[i].size)
				return (char *)map[i].base + offset;
		}
	}
	return NULL;
}

static inline unsigned ipc_msg_len(struct ipc_msg_hdr *h)
{
	unsigned len = h->msg_len;

	if (h->msg_type == IPC_MSG_TYPE_PTR) {
		struct ipc_fapi_msg_hdr *fapi_h;

		fapi_h = (struct ipc_fapi_msg_hdr *)(h + 1);

		len += fapi_h->hdr.size;
		len -= (sizeof(struct ipc_fapi_msg_hdr) -
			sizeof(struct fapi_msg_hdr));
	}

	return len;
}

static void danipc_if_tx_timeout(unsigned long data)
{
	struct danipc_if *intf = (struct danipc_if *)data;
	struct net_device *dev = intf->dev;
	struct sk_buff *skb = intf->tx_skb;
	struct danipc_pair *pair;
	phys_addr_t addr;

	BUG_ON(skb == NULL);

	pair = danipc_skb_cb(skb);

	netdev_dbg(dev, "%s: intf/%p poll tx(dst=%u prio=%u)\n",
		   __func__, intf, pair->dst, pair->prio);

	addr = danipc_hw_fifo_pop_raw(ipc_get_node(pair->dst),
				      default_b_fifo(pair->prio));
	if (addr) {
		danipc_hw_fifo_push_raw(ipc_get_node(pair->dst),
					default_b_fifo(pair->prio),
					addr);
		dev_kfree_skb_any(intf->tx_skb);
		intf->tx_skb = NULL;
		netif_wake_queue(dev);
		intf->tx_queue_restart++;
	} else {
		mod_timer(&intf->tx_timer, jiffies + TX_TIMER_INTERVAL);
	}
}

static int danipc_if_xmit(struct danipc_if *intf, struct sk_buff *skb)
{
	struct danipc_pair	*pair = danipc_skb_cb(skb);
	struct ipc_msg_hdr	*hdr;
	struct net_device	*dev = intf->dev;
	struct danipc_netif_fifo *intf_fifo;
	int			rc;

	netdev_dbg(dev, "%s: pair={dst=0x%x src=0x%x}, len=%u\n",
		   __func__, pair->dst, pair->src, skb->len);

	if (unlikely(!valid_ipc_prio(pair->prio))) {
		netdev_dbg(dev, "%s: invalid priority(%u)\n",
			   __func__, pair->prio);
		goto out;
	}

	intf_fifo = &intf->intf_fifo[pair->prio];

	/* DANIPC is a network device, however it does not support regular IP
	 * packets. All packets not identified by DANIPC protocol (marked with
	 * COOKIE_BASE bits) are discarded.
	 */
	if (unlikely(!DANIPC_PROTOCOL_MATCH(skb->protocol))) {
		intf_fifo->status.tx_bad_proto++;
		intf_fifo->status.tx_drop++;
		dev->stats.tx_dropped++;
		netdev_dbg(dev, "%s() discard packet with protocol=0x%x\n",
			   __func__, ntohs(skb->protocol));
		goto out;
	}
	if (unlikely(!DANIPC_IS_AGENT_DISCOVERED(pair->dst,
						 intf->drvr->dst_aid))) {
		netdev_dbg(dev, "%s: Packet for un-identified agent", __func__);
		intf_fifo->status.tx_unknown_agent++;
		intf_fifo->status.tx_drop++;
		dev->stats.tx_dropped++;
		goto out;
	}

	hdr = ipc_msg_alloc(pair->src,
			    pair->dst,
			    skb->len,
			    0x12,
			    default_b_fifo(pair->prio));
	if (unlikely(hdr == NULL)) {
		netdev_dbg(dev, "%s: ipc_msg_alloc failed!", __func__);
		intf_fifo->status.tx_no_buf++;
		BUG_ON(intf->tx_skb);
		skb_get(skb);
		intf->tx_skb = skb;
		netif_stop_queue(dev);
		mod_timer(&intf->tx_timer, jiffies + TX_TIMER_INTERVAL);
		intf->tx_queue_stop++;
		return NETDEV_TX_BUSY;
	}

	ipc_copy_from((char *)(hdr+1), skb->data, skb->len, false);

	rc = ipc_msg_send(hdr, default_m_fifo(pair->prio));
	if (!rc) {
		intf_fifo->status.tx++;
		intf_fifo->status.tx_bytes += skb->len;

		dev->stats.tx_packets++;
		dev->stats.tx_bytes += skb->len;
	}

out:
	/* This is only called if the device is NOT busy. */
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

static int danipc_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct danipc_if *intf = (struct danipc_if *)netdev_priv(skb->dev);
	int rc;

	rc = danipc_if_xmit(intf, skb);

	return rc;
}

static void danipc_netif_fifo_rx(struct danipc_netif_fifo *rx_fifo,
				 struct ipc_msg_hdr *msg_hdr)
{
	struct danipc_if	*intf = rx_fifo->intf;
	struct net_device	*dev = intf->dev;
	unsigned		msg_len	= ipc_msg_len(msg_hdr);
	struct sk_buff		*skb = netdev_alloc_skb(dev, msg_len);
	struct danipc_pair	*pair;
	struct ipc_fapi_msg_hdr *fapi_hdr;
	uint8_t *fapi_msg_ptr;

	if (unlikely(skb == NULL)) {
		netdev_warn(dev, "%s: skb alloc failed dropping message\n",
			    __func__);
		danipc_if_fifo_free_msg(rx_fifo->if_fifo, msg_hdr);
		rx_fifo->status.rx_no_skb++;
		rx_fifo->status.rx_drop++;
		dev->stats.rx_dropped++;
		return;
	}

	pair = danipc_skb_cb(skb);
	pair->dst = msg_hdr->dest_aid;
	pair->src = msg_hdr->src_aid;
	pair->prio = rx_fifo->prio;

	switch (msg_hdr->msg_type) {
	case IPC_MSG_TYPE_PTR:
		if (msg_hdr->msg_len != sizeof(struct ipc_fapi_msg_hdr)) {
			netdev_dbg(dev, "%s: unexpected msglen(%d)\n",
				   __func__, msg_hdr->msg_len);
			rx_fifo->status.rx_ptr_inv_len++;
			rx_fifo->status.rx_drop++;
			goto err;
		}

		fapi_hdr = (struct ipc_fapi_msg_hdr *)(msg_hdr+1);
		fapi_msg_ptr = get_virt_addr((phys_addr_t)fapi_hdr->msg_ptr);
		if (fapi_msg_ptr == NULL) {
			netdev_dbg(dev,
				   "%s: unexpected fapi message pointer(%p)\n",
				   __func__, fapi_hdr->msg_ptr);
			rx_fifo->status.rx_ptr_inv_addr++;
			rx_fifo->status.rx_drop++;
			goto err;
		}

		dmac_inv_range(fapi_msg_ptr, fapi_msg_ptr + fapi_hdr->hdr.size);
		memcpy(skb->data, &fapi_hdr->hdr, sizeof(fapi_hdr->hdr));
		memcpy(skb->data + sizeof(struct fapi_msg_hdr),
		       fapi_msg_ptr,
		       fapi_hdr->hdr.size);
		rx_fifo->status.rx_ptr++;
		break;
	default:
		memcpy(skb->data, (msg_hdr + 1), msg_hdr->msg_len);
		break;
	}

	danipc_if_fifo_free_msg(rx_fifo->if_fifo, msg_hdr);

	netdev_dbg(dev, "%s() pair={dst=0x%x src=0x%x prio=%d}\n",
		   __func__, pair->dst, pair->src, pair->prio);

	skb_put(skb, msg_len);
	skb_reset_mac_header(skb);

	skb->protocol = cpu_to_be16(AGENTID_TO_COOKIE(pair->dst, pair->prio));

	if (NET_RX_SUCCESS != netif_rx(skb)) {
		netdev_dbg(dev, "%s: netif_rx failed\n", __func__);
		goto err_netif_rx;
	}

	rx_fifo->status.rx++;
	rx_fifo->status.rx_bytes += skb->len;
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += skb->len;

	return;
err:
	dev_kfree_skb_any(skb);
err_netif_rx:
	danipc_if_fifo_free_msg(rx_fifo->if_fifo, msg_hdr);

}

static int danipc_change_mtu(struct net_device *dev, int new_mtu)
{
	if ((new_mtu < 68) || (new_mtu > IPC_BUF_SIZE_MAX))
		return -EINVAL;
	dev->mtu = new_mtu;
	return 0;
}

static void danipc_if_poll(unsigned long cookie)
{
	struct danipc_if *intf = (struct danipc_if *)cookie;
	int prio;

	for (prio = ipc_prio_hi; prio >= 0; prio--) {
		struct danipc_netif_fifo *rx_fifo = &intf->intf_fifo[prio];
		struct danipc_if_fifo *if_fifo = rx_fifo->if_fifo;
		struct ipc_msg_hdr *h;

		if (!if_fifo->probed)
			continue;

		while ((h = danipc_if_fifo_rx_msg(if_fifo))) {
			ipc_msg_hdr_cache_invalid(h);
			if (!ipc_msg_valid(h,
					   intf->fifo->node_id,
					   &rx_fifo->status.rx_err_msg)) {
				rx_fifo->status.rx_drop++;
				danipc_if_fifo_free_msg(if_fifo, h);
				continue;
			}
			ipc_msg_payload_cache_invalid(h);
			danipc_netif_fifo_rx(rx_fifo, h);
		}
	}
}

static irqreturn_t danipc_interrupt(int irq, void *data)
{
	struct danipc_if *intf = (struct danipc_if *)data;

	danipc_fifo_mask_interrupt(intf->fifo);
	danipc_poll_ctl_sched(&intf->poll_ctl);

	return IRQ_HANDLED;
}

static int danipc_open(struct net_device *dev)
{
	struct danipc_if *intf = netdev_priv(dev);
	struct danipc_drvr *drv = intf->drvr;
	struct danipc_fifo *fifo = intf->fifo;
	int rc;

	rc = acquire_local_fifo(fifo, intf);
	if (rc) {
		netdev_err(dev, "local fifo(%s) is in used\n",
			   intf->fifo->probe_info->res_name);
		return rc;
	}

	/* Polling is triggered by IRQ */
	rc = request_irq(dev->irq, danipc_interrupt, 0, dev->name, intf);
	if (rc) {
		netdev_err(dev, "%s request_irq failed\n", dev->name);
		return rc;
	}

	danipc_fifo_init_irq(fifo);

	netif_start_queue(dev);
	drv->ndev_active++;

	return 0;
}

static int danipc_close(struct net_device *dev)
{
	struct danipc_if *intf = netdev_priv(dev);
	struct danipc_drvr *drv = intf->drvr;
	struct danipc_poll_ctl *ctl = &intf->poll_ctl;

	netif_stop_queue(dev);
	del_timer_sync(&intf->tx_timer);
	if (intf->tx_skb) {
		dev_kfree_skb_any(intf->tx_skb);
		intf->tx_skb = NULL;
	}
	danipc_fifo_disable_irq(intf->fifo);
	danipc_poll_ctl_stop(ctl);
	free_irq(dev->irq, intf);
	release_local_fifo(intf->fifo, intf);
	mutex_destroy(&intf->lock);

	drv->ndev_active--;

	return 0;
}

static int danipc_set_mac_addr(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;

	if (!(dev->priv_flags & IFF_LIVE_ADDR_CHANGE) && netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	return 0;
}

static const struct net_device_ops danipc_netdev_ops = {
	.ndo_open		= danipc_open,
	.ndo_stop		= danipc_close,
	.ndo_start_xmit		= danipc_hard_start_xmit,
	.ndo_do_ioctl		= danipc_ioctl,
	.ndo_change_mtu		= danipc_change_mtu,
	.ndo_set_mac_address	= danipc_set_mac_addr,
};

static void danipc_setup(struct net_device *dev)
{
	dev->netdev_ops		= &danipc_netdev_ops;

	dev->type		= ARPHRD_VOID;
	dev->hard_header_len	= sizeof(struct ipc_msg_hdr);
	dev->addr_len		= sizeof(danipc_addr_t);
	dev->tx_queue_len	= 1000;

	/* New-style flags. */
	dev->flags		= IFF_NOARP;
}

/* Our vision of L2 header: it is of type struct danipc_pair
 * it is stored at address skb->cb[HADDR_CB_OFFSET].
 */

static int danipc_header_parse(const struct sk_buff *skb, unsigned char *haddr)
{
	struct danipc_pair *pair = danipc_skb_cb(skb);

	memcpy(haddr, &pair->src, sizeof(danipc_addr_t));
	return sizeof(danipc_addr_t);
}

static int danipc_header(struct sk_buff *skb, struct net_device *dev,
			 unsigned short type, const void *daddr,
			 const void *saddr, unsigned len)
{
	struct danipc_pair *pair = danipc_skb_cb(skb);
	const uint8_t *addr = daddr;

	pair->src = COOKIE_TO_AGENTID(type);
	pair->prio = COOKIE_TO_PRIO(type);
	if (addr)
		pair->dst = *addr;
	return 0;
}

static const struct header_ops danipc_header_ops ____cacheline_aligned = {
	.create	= danipc_header,
	.parse	= danipc_header_parse,
};

static void danipc_netif_fifo_init(struct danipc_if *intf, enum ipc_prio prio)
{
	struct danipc_fifo *fifo = intf->fifo;
	struct danipc_netif_fifo *rx_fifo = &intf->intf_fifo[prio];

	rx_fifo->intf = intf;
	rx_fifo->if_fifo = &fifo->if_fifo[prio];
	rx_fifo->prio = prio;
}

static int danipc_if_init(struct platform_device *pdev,
			  struct danipc_fifo *fifo,
			  uint8_t ifidx)
{
	struct danipc_probe_info *probe_list = fifo->probe_info;
	struct net_device *dev;
	struct danipc_if *intf;
	int prio, rc = 0;

	dev = alloc_netdev(sizeof(struct danipc_if),
			   probe_list->ifname, danipc_setup);
	if (unlikely(dev == NULL))
		return -ENOMEM;

	intf = netdev_priv(dev);

	intf->drvr = &danipc_driver;
	intf->dev = dev;
	intf->ifidx = ifidx;
	intf->fifo = fifo;

	for (prio = 0; prio < max_ipc_prio; prio++)
		danipc_netif_fifo_init(intf, prio);

	mutex_init(&intf->lock);
	danipc_poll_ctl_init(&intf->poll_ctl,
			     danipc_if_poll,
			     (unsigned long)intf,
			     DEFAULT_POLL_INTERVAL_IN_US);

	setup_timer(&intf->tx_timer,
		    danipc_if_tx_timeout,
		    (unsigned long)intf);

	strlcpy(dev->name, probe_list->ifname, sizeof(dev->name));
	dev->header_ops = &danipc_header_ops;
	dev->irq = fifo->irq;
	dev->dev_addr[0] = fifo->node_id;
	dev->mtu = IPC_BUF_SIZE_MAX;

	rc = register_netdev(dev);
	if (rc) {
		netdev_err(dev, "%s: register_netdev failed\n",
			   __func__);
		mutex_destroy(&intf->lock);
		goto danipc_iferr;
	}

	danipc_driver.if_list[ifidx] = intf;
	danipc_driver.ndev++;
danipc_iferr:
	if (rc)
		free_netdev(dev);
	return rc;
}

static void danipc_if_remove(struct danipc_drvr *pdrv, uint8_t ifidx)
{
	struct danipc_if *intf = pdrv->if_list[ifidx];
	struct net_device *netdev = (intf) ? intf->dev : NULL;

	if (netdev == NULL)
		return;

	if (netdev->reg_state == NETREG_REGISTERED)
		unregister_netdev(netdev);

	free_netdev(netdev);

	netdev_info(netdev,
		    "Unregister DANIPC Network Interface(%s).\n",
		    netdev->name);

	pdrv->if_list[ifidx] = NULL;
	pdrv->ndev--;
}

int danipc_netdev_init(struct platform_device *pdev)
{
	struct danipc_drvr *pdrv = &danipc_driver;
	uint8_t i;
	int ret = 0;

	for (i = 0; i < pdrv->num_lfifo && !ret; i++)
		ret = danipc_if_init(pdev, &pdrv->lfifo[i], i);

	return ret;
}

int danipc_netdev_cleanup(struct platform_device *pdev)
{
	struct danipc_drvr *pdrv = &danipc_driver;
	uint8_t i = 0;

	for (i = 0; i < DANIPC_MAX_IF; i++)
		danipc_if_remove(pdrv, i);

	pr_info("DANIPC Network driver unregistered.\n");
	return 0;
}

/* DANIPC netdev debugfs interface */
static void danipc_netif_dump_fifo_status(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_netif_fifo *intf_fifo =
		(struct danipc_netif_fifo *)hdlr->data;
	struct danipc_netif_fifo_status *stats = &intf_fifo->status;
	struct ipc_msg_err_stats *msg_stats = &stats->rx_err_msg;

	seq_printf(s, "%-25s: %u\n", "rx", stats->rx);
	seq_printf(s, "%-25s: %u\n", "rx_bytes", stats->rx_bytes);
	seq_printf(s, "%-25s: %u\n", "rx_ptr", stats->rx_ptr);
	seq_printf(s, "%-25s: %u\n", "rx_drop", stats->rx_drop);
	seq_printf(s, "%-25s: %u\n", "rx_error", stats->rx_error);
	seq_printf(s, "%-25s: %u\n", "rx_no_skb", stats->rx_no_skb);
	seq_printf(s, "%-25s: %u\n", "rx_ptr_inv_len", stats->rx_ptr_inv_len);
	seq_printf(s, "%-25s: %u\n", "rx_ptr_inv_addr", stats->rx_ptr_inv_addr);

	seq_printf(s, "%-25s: %u\n", "rx_zero_len_msg", msg_stats->zlen_msg);
	seq_printf(s, "%-25s: %u\n", "rx_oversize_msg",
		   msg_stats->oversize_msg);
	seq_printf(s, "%-25s: %u\n", "rx_invalid_aid_msg",
		   msg_stats->inval_msg);
	seq_printf(s, "%-25s: %u\n", "rx_chained_msg", msg_stats->chained_msg);

	seq_printf(s, "%-25s: %u\n", "tx", stats->tx);
	seq_printf(s, "%-25s: %u\n", "tx_bytes", stats->tx_bytes);
	seq_printf(s, "%-25s: %u\n", "tx_drop", stats->tx_drop);
	seq_printf(s, "%-25s: %u\n", "tx_unknown_agent",
		   stats->tx_unknown_agent);
	seq_printf(s, "%-25s: %u\n", "tx_bad_proto", stats->tx_bad_proto);
	seq_printf(s, "%-25s: %u\n", "tx_no_buf", stats->tx_no_buf);
}

static void danipc_netif_dump_if_fifo(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_netif_fifo *intf_fifo =
		(struct danipc_netif_fifo *)hdlr->data;

	danipc_dbgfs_dump_if_fifo(s, intf_fifo->if_fifo);
}

static struct danipc_dbgfs netif_fifo_dbgfs[] = {
	DBGFS_NODE("stats", 0444, danipc_netif_dump_fifo_status, NULL),
	DBGFS_NODE("interface", 0444, danipc_netif_dump_if_fifo, NULL),
	DBGFS_NODE_LAST
};

static void danipc_if_dump_poll_count(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_if *intf = (struct danipc_if *)hdlr->data;
	struct danipc_poll_ctl *ctl = &intf->poll_ctl;

	seq_printf(s, "%llu\n", ctl->poll_cnt);
}

static void danipc_if_dump_poll_intval(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_if *intf = (struct danipc_if *)hdlr->data;
	struct danipc_poll_ctl *ctl = &intf->poll_ctl;

	seq_printf(s, "%lldus\n", ktime_to_us(ctl->timer_intval));
}

static ssize_t danipc_if_set_poll_intval(struct file *filp,
					 const char __user *ubuf,
					 size_t cnt,
					 loff_t *ppos)
{
	struct seq_file *m = filp->private_data;
	struct dbgfs_hdlr *dbgfshdlr = (struct dbgfs_hdlr *)m->private;
	struct danipc_if *intf = (struct danipc_if *)dbgfshdlr->data;
	struct danipc_poll_ctl *ctl = &intf->poll_ctl;
	uint32_t intval;

	if (kstrtouint_from_user(ubuf, cnt, 0, &intval))
		return -EINVAL;

	if (!intval)
		return -EINVAL;

	ctl->timer_intval = ns_to_ktime(intval * 1000);
	return cnt;
}

static void danipc_if_dump_fifo(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_if *intf = (struct danipc_if *)hdlr->data;

	danipc_dbgfs_dump_fifo(s, intf->fifo);
}

static void danipc_if_dump_tx_queue(struct seq_file *s)
{
	struct dbgfs_hdlr *hdlr = (struct dbgfs_hdlr *)s->private;
	struct danipc_if *intf = (struct danipc_if *)hdlr->data;

	seq_printf(s, "tx_queue_stop:       %u\n", intf->tx_queue_stop);
	seq_printf(s, "tx_queue_restart:    %u\n", intf->tx_queue_restart);
	seq_printf(s, "tx_queue:            %s\n",
		   (netif_queue_stopped(intf->dev)) ? "stopped" : "running");
}

static struct danipc_dbgfs netdev_dbgfs[] = {
	DBGFS_NODE("timer_intval", 0644, danipc_if_dump_poll_intval,
		   danipc_if_set_poll_intval),
	DBGFS_NODE("poll_cnt", 0444, danipc_if_dump_poll_count, NULL),
	DBGFS_NODE("fifo", 0444, danipc_if_dump_fifo, NULL),
	DBGFS_NODE("tx_queue", 0444, danipc_if_dump_tx_queue, NULL),
	DBGFS_NODE_LAST
};

static void danipc_dbgfs_netdev_remove_dent(struct danipc_if *intf)
{
	int prio;

	for (prio = 0; prio < max_ipc_prio; prio++)
		danipc_dbgfs_remove_dent(&intf->intf_fifo[prio].dent);

	danipc_dbgfs_remove_dent(&intf->dent);
}

int danipc_dbgfs_netdev_init(void)
{
	struct danipc_drvr *drvr = &danipc_driver;
	struct dentry *dent;
	int ret = 0;
	int i;

	dent = debugfs_create_dir("intf_netdev", drvr->dirent);
	if (IS_ERR(dent)) {
		pr_err("%s: failed to create intf directory\n", __func__);
		return PTR_ERR(dent);
	}

	for (i = 0; i < drvr->ndev && !ret; i++) {
		struct danipc_if *intf = drvr->if_list[i];
		struct danipc_netif_fifo *rx_fifo = intf->intf_fifo;
		int prio;

		ret = danipc_dbgfs_create_dent(&intf->dent,
					       dent,
					       intf->dev->name,
					       netdev_dbgfs,
					       intf);
		if (ret) {
			pr_err("%s: failed to allocate dbgfs\n", __func__);
			break;
		}

		for (prio = 0; prio < max_ipc_prio && !ret; prio++, rx_fifo++) {
			if (!rx_fifo->if_fifo->probed)
				continue;
			ret = danipc_dbgfs_create_dent(
				&rx_fifo->dent,
				intf->dent.dent,
				(rx_fifo->prio == ipc_prio_hi) ?
				"fifo(hi_prio)" : "fifo(lo_prio)",
				netif_fifo_dbgfs,
				rx_fifo);
		}
		if (ret)
			danipc_dbgfs_netdev_remove_dent(intf);
	}

	return ret;
}

void danipc_dbgfs_netdev_remove(void)
{
	struct danipc_drvr *drvr = &danipc_driver;
	int i;

	for (i = 0; i < drvr->ndev; i++) {
		struct danipc_if *intf = drvr->if_list[i];

		danipc_dbgfs_netdev_remove_dent(intf);
	}
}
