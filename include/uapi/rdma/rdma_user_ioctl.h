/*
 * Copyright (c) 2016 Mellanox Technologies, LTD. All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef RDMA_USER_IOCTL_H
#define RDMA_USER_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <rdma/ib_user_mad.h>
#include <rdma/hfi/hfi1_ioctl.h>

/* Documentation/ioctl/ioctl-number.txt */
#define RDMA_IOCTL_MAGIC	0x1b
/* Legacy name, for user space application which already use it */
#define IB_IOCTL_MAGIC		RDMA_IOCTL_MAGIC

/* General blocks assignments */
#define MAD_CMD_BASE		0x00
#define HFI1_CMD_BASE		0xE0

/* MAD specific section */
#define MAD_CMD_REG_AGENT	(MAD_CMD_BASE + 0x01)
#define MAD_CMD_UNREG_AGENT	(MAD_CMD_BASE + 0x02)
#define MAD_CMD_ENABLE_PKEY	(MAD_CMD_BASE + 0x03)
#define MAD_CMD_REG_AGENT2	(MAD_CMD_BASE + 0x04)

#define IB_USER_MAD_REGISTER_AGENT \
	_IOWR(RDMA_IOCTL_MAGIC, MAD_CMD_REG_AGENT, struct ib_user_mad_reg_req)
#define IB_USER_MAD_UNREGISTER_AGENT \
	_IOW(RDMA_IOCTL_MAGIC, MAD_CMD_UNREG_AGENT, __u32)
#define IB_USER_MAD_ENABLE_PKEY \
	_IO(RDMA_IOCTL_MAGIC, MAD_CMD_ENABLE_PKEY)
#define IB_USER_MAD_REGISTER_AGENT2 \
	_IOWR(RDMA_IOCTL_MAGIC, MAD_CMD_REG_AGENT2, struct ib_user_mad_reg_req2)

/* HFI specific section */
/* User commands. */
/* allocate HFI and context */
#define HFI1_CMD_ASSIGN_CTXT     (HFI1_CMD_BASE + 0x01)
/* find out what resources we got */
#define HFI1_CMD_CTXT_INFO       (HFI1_CMD_BASE + 0x02)
/* set up userspace */
#define HFI1_CMD_USER_INFO       (HFI1_CMD_BASE + 0x03)
/* update expected TID entries */
#define HFI1_CMD_TID_UPDATE      (HFI1_CMD_BASE + 0x04)
/* free expected TID entries */
#define HFI1_CMD_TID_FREE        (HFI1_CMD_BASE + 0x05)
/* force an update of PIO credit */
#define HFI1_CMD_CREDIT_UPD      (HFI1_CMD_BASE + 0x06)

/* control receipt of packets */
#define HFI1_CMD_RECV_CTRL       (HFI1_CMD_BASE + 0x08)
/* set the kind of polling we want */
#define HFI1_CMD_POLL_TYPE       (HFI1_CMD_BASE + 0x09)
/* ack & clear user status bits */
#define HFI1_CMD_ACK_EVENT       (HFI1_CMD_BASE + 0x0A)
/* set context's pkey */
#define HFI1_CMD_SET_PKEY        (HFI1_CMD_BASE + 0x0B)
/* reset context's HW send context */
#define HFI1_CMD_CTXT_RESET      (HFI1_CMD_BASE + 0x0C)
/* read TID cache invalidations */
#define HFI1_CMD_TID_INVAL_READ  (HFI1_CMD_BASE + 0x0D)
/* get the version of the user cdev */
#define HFI1_CMD_GET_VERS	 (HFI1_CMD_BASE + 0x0E)

/*
 * User IOCTLs can not go above 128 if they do then see common.h and change the
 * base for the snoop ioctl
 */

#define HFI1_IOCTL_ASSIGN_CTXT \
	_IOWR(RDMA_IOCTL_MAGIC, HFI1_CMD_ASSIGN_CTXT, struct hfi1_user_info)
#define HFI1_IOCTL_CTXT_INFO \
	_IOW(RDMA_IOCTL_MAGIC, HFI1_CMD_CTXT_INFO, struct hfi1_ctxt_info)
#define HFI1_IOCTL_USER_INFO \
	_IOW(RDMA_IOCTL_MAGIC, HFI1_CMD_USER_INFO, struct hfi1_base_info)
#define HFI1_IOCTL_TID_UPDATE \
	_IOWR(RDMA_IOCTL_MAGIC, HFI1_CMD_TID_UPDATE, struct hfi1_tid_info)
#define HFI1_IOCTL_TID_FREE \
	_IOWR(RDMA_IOCTL_MAGIC, HFI1_CMD_TID_FREE, struct hfi1_tid_info)
#define HFI1_IOCTL_CREDIT_UPD \
	_IO(RDMA_IOCTL_MAGIC, HFI1_CMD_CREDIT_UPD)
#define HFI1_IOCTL_RECV_CTRL \
	_IOW(RDMA_IOCTL_MAGIC, HFI1_CMD_RECV_CTRL, int)
#define HFI1_IOCTL_POLL_TYPE \
	_IOW(RDMA_IOCTL_MAGIC, HFI1_CMD_POLL_TYPE, int)
#define HFI1_IOCTL_ACK_EVENT \
	_IOW(RDMA_IOCTL_MAGIC, HFI1_CMD_ACK_EVENT, unsigned long)
#define HFI1_IOCTL_SET_PKEY \
	_IOW(RDMA_IOCTL_MAGIC, HFI1_CMD_SET_PKEY, __u16)
#define HFI1_IOCTL_CTXT_RESET \
	_IO(RDMA_IOCTL_MAGIC, HFI1_CMD_CTXT_RESET)
#define HFI1_IOCTL_TID_INVAL_READ \
	_IOWR(RDMA_IOCTL_MAGIC, HFI1_CMD_TID_INVAL_READ, struct hfi1_tid_info)
#define HFI1_IOCTL_GET_VERS \
	_IOR(RDMA_IOCTL_MAGIC, HFI1_CMD_GET_VERS, int)

#endif /* RDMA_USER_IOCTL_H */
