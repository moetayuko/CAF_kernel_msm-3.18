/* SPDX-License-Identifier: GPL-2.0
* Copyright (c) 2011-2016, The Linux Foundation
* Copyright (c) 2017, Linaro Limited
*/

#ifndef __APR_H_
#define __APR_H_

#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>
/* APR Client IDs */
#define APR_CLIENT_AUDIO	0x0
#define APR_CLIENT_VOICE	0x1
#define APR_CLIENT_MAX		0x2

#define APR_DL_SMD    0
#define APR_DL_MAX    1

#define APR_DEST_MODEM 0
#define APR_DEST_QDSP6 1
#define APR_DEST_MAX   2
#define APR_MAX_BUF   8192

#define APR_HDR_LEN(hdr_len) ((hdr_len)/4)
#define APR_PKT_SIZE(hdr_len, payload_len) ((hdr_len) + (payload_len))
#define APR_HDR_FIELD(msg_type, hdr_len, ver)\
	(((msg_type & 0x3) << 8) | ((hdr_len & 0xF) << 4) | (ver & 0xF))

#define APR_HDR_SIZE sizeof(struct apr_hdr)
#define APR_SEQ_CMD_HDR_FIELD APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, \
					    APR_HDR_LEN(APR_HDR_SIZE), \
					    APR_PKT_VER)

/* Version */
#define APR_PKT_VER		0x0

/* Command and Response Types */
#define APR_MSG_TYPE_EVENT	0x0
#define APR_MSG_TYPE_CMD_RSP	0x1
#define APR_MSG_TYPE_SEQ_CMD	0x2
#define APR_MSG_TYPE_NSEQ_CMD	0x3
#define APR_MSG_TYPE_MAX	0x04

/* APR Basic Response Message */
#define APR_BASIC_RSP_RESULT 0x000110E8
#define APR_RSP_ACCEPTED     0x000100BE

/* Domain IDs */
#define APR_DOMAIN_SIM	0x1
#define APR_DOMAIN_PC		0x2
#define APR_DOMAIN_MODEM	0x3
#define APR_DOMAIN_ADSP	0x4
#define APR_DOMAIN_APPS	0x5
#define APR_DOMAIN_MAX	0x6

/* ADSP service IDs */
#define APR_SVC_TEST_CLIENT     0x2
#define APR_SVC_ADSP_CORE	0x3
#define APR_SVC_AFE		0x4
#define APR_SVC_VSM		0x5
#define APR_SVC_VPM		0x6
#define APR_SVC_ASM		0x7
#define APR_SVC_ADM		0x8
#define APR_SVC_ADSP_MVM	0x09
#define APR_SVC_ADSP_CVS	0x0A
#define APR_SVC_ADSP_CVP	0x0B
#define APR_SVC_USM		0x0C
#define APR_SVC_LSM		0x0D
#define APR_SVC_VIDC		0x16
#define APR_SVC_MAX		0x17

/* Modem Service IDs */
#define APR_SVC_MVS		0x3
#define APR_SVC_MVM		0x4
#define APR_SVC_CVS		0x5
#define APR_SVC_CVP		0x6
#define APR_SVC_SRD		0x7

/* APR Port IDs */
#define APR_MAX_PORTS		0x80
#define APR_NAME_MAX		0x40
#define RESET_EVENTS		0x000130D7

/* hdr field Ver [0:3], Size [4:7], Message type [8:10] */
#define APR_HDR_FIELD_VER(h)		(h & 0x000F)
#define APR_HDR_FIELD_SIZE(h)		((h & 0x00F0) >> 4)
#define APR_HDR_FIELD_SIZE_BYTES(h)	(((h & 0x00F0) >> 4) * 4)
#define APR_HDR_FIELD_MT(h)		((h & 0x0300) >> 8)

struct apr_hdr {
	uint16_t hdr_field;
	uint16_t pkt_size;
	uint8_t src_svc;
	uint8_t src_domain;
	uint16_t src_port;
	uint8_t dest_svc;
	uint8_t dest_domain;
	uint16_t dest_port;
	uint32_t token;
	uint32_t opcode;
};

struct apr_client_data {
	uint16_t payload_size;
	uint16_t hdr_len;
	uint16_t msg_type;
	uint16_t src;
	uint16_t dest_svc;
	uint16_t src_port;
	uint16_t dest_port;
	uint32_t token;
	uint32_t opcode;
	void *payload;
};

#define ADSP_AUDIO_APR_DEV(name, id) \
	{name, APR_DOMAIN_ADSP, id, APR_CLIENT_AUDIO}

/* Bits 0 to 15 -- Minor version,  Bits 16 to 31 -- Major version */

#define APR_SVC_MAJOR_VERSION(v)	((v >> 16) & 0xFF)
#define APR_SVC_MINOR_VERSION(v)	(v & 0xFF)

struct apr_device {
	struct device	dev;
	uint16_t	svc_id;
	uint16_t	dest_id;
	uint16_t	client_id;
	uint16_t	domain_id;
	uint16_t	version;

	spinlock_t	lock;
	struct list_head node;
};


#define to_apr_device(d) container_of(d, struct apr_device, dev)

struct apr_driver {
	int	(*probe)(struct apr_device *sl);
	int	(*remove)(struct apr_device *sl);
	int	(*callback)(struct apr_device *a, struct apr_client_data *d);
	struct device_driver		driver;
	const struct apr_device_id	*id_table;
};

#define to_apr_driver(d) container_of(d, struct apr_driver, driver)

/*
 * use a macro to avoid include chaining to get THIS_MODULE
 */
#define apr_driver_register(drv) __apr_driver_register(drv, THIS_MODULE)

int __apr_driver_register(struct apr_driver *drv, struct module *owner);
void apr_driver_unregister(struct apr_driver *drv);
int apr_add_device(struct device *dev, const struct apr_device_id *id);

/**
 * module_apr_driver() - Helper macro for registering a aprbus driver
 * @__aprbus_driver: aprbus_driver struct
 *
 * Helper macro for aprbus drivers which do not do anything special in
 * module init/exit. This eliminates a lot of boilerplate. Each module
 * may only use this macro once, and calling it replaces module_init()
 * and module_exit()
 */
#define module_apr_driver(__apr_driver) \
	module_driver(__apr_driver, apr_driver_register, \
			apr_driver_unregister)

int apr_send_pkt(struct apr_device *adev, uint32_t *buf);

#endif /* __APR_H_ */
