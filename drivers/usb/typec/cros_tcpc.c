/*
 * Copyright 2015-2016 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * USB Type-C Port Controller Interface.
 */

#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/mfd/cros_ec_pd_update.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/usb/pd.h>
#include <linux/usb/pd_vdo.h>
#include <linux/usb/typec.h>

#include "tcpci.h"
#include "tcpm.h"

#define PD_RETRY_COUNT 3

/* hard-coded for our TCPC port 0 */
#define TCPC_ADDR 0x4e

#define EC_BUFSIZ	128
static uint8_t ec_buffer[EC_BUFSIZ];

struct cros_tcpc {
	struct device *dev;
	struct cros_ec_device *ec_device;

	struct tcpm_port *port;

	bool controls_vbus;

	struct tcpc_dev tcpc;
	struct tcpc_mux_dev mux;

	struct mutex lock;

	struct notifier_block notifier;
};

static inline struct cros_tcpc *tcpc_to_cros(struct tcpc_dev *tcpc)
{
	return container_of(tcpc, struct cros_tcpc, tcpc);
}

static inline struct cros_tcpc *mux_to_cros(struct tcpc_mux_dev *mux)
{
	return container_of(mux, struct cros_tcpc, mux);
}

static int _cros_tcpc_read(struct cros_tcpc *cros_tcpc, unsigned int reg,
			   void *val, unsigned int len)
{
	struct cros_ec_command *msg = (struct cros_ec_command *)ec_buffer;
	struct ec_params_i2c_passthru *p =
		(struct ec_params_i2c_passthru *)msg->data;
	struct ec_response_i2c_passthru *r =
		(struct ec_response_i2c_passthru *)msg->data;
	struct ec_params_i2c_passthru_msg *i2c_msg = p->msg;
	uint8_t *pdata;
	int size, err;

	memset(ec_buffer, 0, sizeof(ec_buffer));

	p->num_msgs = 2;
	p->port = 2; /* i2c port */

	size = sizeof(*p) + p->num_msgs * sizeof(*i2c_msg);
	pdata = (uint8_t *)p + size;

	/* Write register to read */
	i2c_msg->addr_flags = TCPC_ADDR;
	i2c_msg->len = 1;
	pdata[0] = reg;

	/* Read data */
	i2c_msg++;
	i2c_msg->addr_flags = TCPC_ADDR | EC_I2C_FLAG_READ;
	i2c_msg->len = len;

	msg->command = EC_CMD_I2C_PASSTHRU;
	msg->outsize = size + 1;
	msg->insize = sizeof(*r) + len;

	err = cros_ec_cmd_xfer_status(cros_tcpc->ec_device, msg);
	if (err < 0)
		return -EPROTO;

	if (r->i2c_status & EC_I2C_STATUS_NAK)
		return -ENXIO;

	if (r->i2c_status & EC_I2C_STATUS_TIMEOUT)
		return -ETIMEDOUT;

	memcpy(val, &(r->data[0]), len);

	return 0;
}

static int cros_tcpc_read(struct cros_tcpc *cros_tcpc, unsigned int reg,
			  void *val, unsigned int len)
{
	int ret;

	mutex_lock(&cros_tcpc->lock);
	ret = _cros_tcpc_read(cros_tcpc, reg, val, len);
	mutex_unlock(&cros_tcpc->lock);

	return ret;
}

static int _cros_tcpc_write_raw(struct cros_tcpc *cros_tcpc, unsigned int reg,
				const void *val, unsigned int len)
{
	struct cros_ec_command *msg = (struct cros_ec_command *)ec_buffer;
	struct ec_params_i2c_passthru *p =
		(struct ec_params_i2c_passthru *)msg->data;
	struct ec_response_i2c_passthru *r =
		(struct ec_response_i2c_passthru *)msg->data;
	struct ec_params_i2c_passthru_msg *i2c_msg = p->msg;
	uint8_t *pdata;
	int size, err;

	memset(ec_buffer, 0, sizeof(ec_buffer));

	p->num_msgs = 1;
	p->port = 2; /* i2c port */

	size = sizeof(*p) + p->num_msgs * sizeof(*i2c_msg);
	pdata = (uint8_t *)p + size;

	/* Write register to read */
	i2c_msg->addr_flags = TCPC_ADDR;
	i2c_msg->len = len + 1;
	pdata[0] = reg;

	/* Write data */
	memcpy(&pdata[1], val, len);

	msg->command = EC_CMD_I2C_PASSTHRU;
	msg->outsize = size + 1 + len;
	msg->insize = sizeof(*r);

	err = cros_ec_cmd_xfer_status(cros_tcpc->ec_device, msg);
	if (err < 0)
		return -EPROTO;

	if (r->i2c_status & EC_I2C_STATUS_NAK)
		return -ENXIO;

	if (r->i2c_status & EC_I2C_STATUS_TIMEOUT)
		return -ETIMEDOUT;

	return 0;
}

static int _cros_tcpc_write(struct cros_tcpc *cros_tcpc, unsigned int reg,
			   unsigned int val, unsigned int len)
{
	return _cros_tcpc_write_raw(cros_tcpc, reg, &val, len);
}

static int cros_tcpc_write(struct cros_tcpc *cros_tcpc, unsigned int reg,
			   unsigned int val, unsigned int len)
{
	int ret;

	mutex_lock(&cros_tcpc->lock);
	ret = _cros_tcpc_write(cros_tcpc, reg, val, len);
	mutex_unlock(&cros_tcpc->lock);

	return ret;
}

static int cros_tcpc_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	unsigned int reg;

	switch (cc) {
	case TYPEC_CC_RA:
		reg = (TCPC_ROLE_CTRL_CC_RA << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RA << TCPC_ROLE_CTRL_CC2_SHIFT);
		break;
	case TYPEC_CC_RD:
		reg = (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
		break;
	case TYPEC_CC_RP_DEF:
		reg = (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT) |
			(TCPC_ROLE_CTRL_RP_VAL_DEF <<
			 TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_1_5:
		reg = (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT) |
			(TCPC_ROLE_CTRL_RP_VAL_1_5 <<
			 TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_3_0:
		reg = (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT) |
			(TCPC_ROLE_CTRL_RP_VAL_3_0 <<
			 TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_OPEN:
	default:
		reg = (TCPC_ROLE_CTRL_CC_OPEN << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_OPEN << TCPC_ROLE_CTRL_CC2_SHIFT);
		break;
	}

	return cros_tcpc_write(cros_tcpc, TCPC_ROLE_CTRL, reg, 1);
}

static int cros_tcpc_set_polarity(struct tcpc_dev *tcpc,
			      enum typec_cc_polarity polarity)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);

	return cros_tcpc_write(cros_tcpc, TCPC_TCPC_CTRL,
			       (polarity == TYPEC_POLARITY_CC2) ?
			       TCPC_TCPC_CTRL_ORIENTATION : 0, 1);
}

static int cros_tcpc_set_vconn(struct tcpc_dev *tcpc, bool enable)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);

	return cros_tcpc_write(cros_tcpc, TCPC_POWER_CTRL,
			       enable ? TCPC_POWER_CTRL_VCONN_ENABLE : 0, 1);
}

static int cros_tcpc_set_vbus(struct tcpc_dev *tcpc, bool enable, bool charge)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	struct cros_ec_command *msg = (struct cros_ec_command *)ec_buffer;
	struct ec_params_usb_pd_vbus_control *p =
		(struct ec_params_usb_pd_vbus_control *)msg->data;
	int err;

	mutex_lock(&cros_tcpc->lock);

	memset(ec_buffer, 0, sizeof(ec_buffer));

	p->port = 0;
	p->control = enable;
	p->charge = charge;

	msg->command = EC_CMD_USB_PD_VBUS;
	msg->outsize = sizeof(*p);

	err = cros_ec_cmd_xfer_status(cros_tcpc->ec_device, msg);
	if (err < 0) {
		err = -EPROTO;
		goto out;
	}
	err = 0;
out:
	mutex_unlock(&cros_tcpc->lock);
	return err;
}

static int cros_tcpc_set_current_limit(struct tcpc_dev *tcpc,
				       u32 max_ma, u32 mv)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	struct cros_ec_command *msg = (struct cros_ec_command *)ec_buffer;
	struct ec_params_usb_pd_current_limit *p =
		(struct ec_params_usb_pd_current_limit *)msg->data;
	int err;

	mutex_lock(&cros_tcpc->lock);

	memset(ec_buffer, 0, sizeof(ec_buffer));

	p->port = 0;
	p->current_limit = max_ma;
	p->supply_voltage = mv;

	msg->command = EC_CMD_USB_PD_CURRENT_LIMIT;
	msg->outsize = sizeof(*p);

	err = cros_ec_cmd_xfer_status(cros_tcpc->ec_device, msg);
	if (err < 0) {
		err = -EPROTO;
		goto out;
	}
	err = 0;
out:
	mutex_unlock(&cros_tcpc->lock);
	return err;
}

static int cros_tcpc_set_roles(struct tcpc_dev *tcpc,
			       bool attached,
			       enum typec_role role,
			       enum typec_data_role data)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	unsigned int reg;

	reg = PD_REV20 << TCPC_MSG_HDR_INFO_REV_SHIFT;
	if (role == TYPEC_SOURCE)
		reg |= TCPC_MSG_HDR_INFO_PWR_ROLE;
	if (data == TYPEC_HOST)
		reg |= TCPC_MSG_HDR_INFO_DATA_ROLE;

	return cros_tcpc_write(cros_tcpc, TCPC_MSG_HDR_INFO, reg, 1);
}

static int cros_tcpc_set_mux(struct tcpc_mux_dev *mux,
			     enum tcpc_mux_mode mux_mode,
			     enum tcpc_usb_switch usb_config,
			     enum typec_cc_polarity polarity)
{
	struct cros_tcpc *cros_tcpc = mux_to_cros(mux);
	struct cros_ec_command *msg = (struct cros_ec_command *)ec_buffer;
	struct ec_params_usb_pd_mux_control *p =
		(struct ec_params_usb_pd_mux_control *)msg->data;
	int err;

	mutex_lock(&cros_tcpc->lock);

	memset(ec_buffer, 0, sizeof(ec_buffer));

	p->port = 0;
	p->mux = (mux_mode == TYPEC_MUX_USB) ? USB_PD_CTRL_MUX_USB
					     : USB_PD_CTRL_MUX_NONE;
	p->polarity = polarity;

	msg->command = EC_CMD_USB_PD_MUX;
	msg->outsize = sizeof(*p);

	err = cros_ec_cmd_xfer_status(cros_tcpc->ec_device, msg);
	if (err < 0) {
		err = -EPROTO;
		goto out;
	}
	err = 0;
out:
	mutex_unlock(&cros_tcpc->lock);
	return err;
}

static int cros_tcpc_set_pd_rx(struct tcpc_dev *tcpc, bool enable)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	unsigned int reg = 0;
	int ret;

	if (enable)
		reg = TCPC_RX_DETECT_SOP | TCPC_RX_DETECT_HARD_RESET;
	ret = cros_tcpc_write(cros_tcpc, TCPC_RX_DETECT, reg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int cros_tcpc_get_vbus(struct tcpc_dev *tcpc)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	int ret;
	u8 reg;

	ret = cros_tcpc_read(cros_tcpc, TCPC_POWER_STATUS, &reg, 1);
	if (ret < 0)
		return ret;

	return !!(reg & TCPC_POWER_STATUS_VBUS_PRES);
}

static enum typec_cc_status tcpci_to_typec_cc(unsigned int cc, bool sink)
{
	switch (cc) {
	case 0x1:
		return sink ? TYPEC_CC_RP_DEF : TYPEC_CC_RA;
	case 0x2:
		return sink ? TYPEC_CC_RP_1_5 : TYPEC_CC_RD;
	case 0x3:
		if (sink)
			return TYPEC_CC_RP_3_0;
	case 0x0:
	default:
		return TYPEC_CC_OPEN;
	}
}

static int cros_tcpc_get_cc(struct tcpc_dev *tcpc, enum typec_cc_status *cc1,
			    enum typec_cc_status *cc2)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	int ret;
	u8 reg;

	ret = cros_tcpc_read(cros_tcpc, TCPC_CC_STATUS, &reg, 1);
	if (ret < 0)
		return ret;

	*cc1 = tcpci_to_typec_cc((reg >> TCPC_CC_STATUS_CC1_SHIFT) &
				 TCPC_CC_STATUS_CC1_MASK,
				 reg & TCPC_CC_STATUS_TERM);
	*cc2 = tcpci_to_typec_cc((reg >> TCPC_CC_STATUS_CC2_SHIFT) &
				 TCPC_CC_STATUS_CC2_MASK,
				 reg & TCPC_CC_STATUS_TERM);
	return 0;
}

static int cros_tcpc_pd_transmit(struct tcpc_dev *tcpc,
			     enum tcpm_transmit_type type,
			     const struct pd_message *msg)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	u8 payload[PD_MAX_PAYLOAD * sizeof(u32) + 4];
	unsigned int reg, cnt;
	int ret;

	mutex_lock(&cros_tcpc->lock);

	cnt = msg ? pd_header_cnt(msg->header) * 4 : 0;

	payload[0] = cnt + 2;
	if (msg) {
		payload[1] = msg->header & 0xff;
		payload[2] = (msg->header >> 8) & 0xff;
		memcpy(&payload[3], msg->payload, cnt);
	} else {
		payload[1] = 0;
		payload[2] = 0;
	}

	ret = _cros_tcpc_write_raw(cros_tcpc, TCPC_TX_BYTE_CNT, payload,
				   cnt + 3);
	if (ret < 0)
		goto out;

	reg = (PD_RETRY_COUNT << TCPC_TRANSMIT_RETRY_SHIFT) |
		(type << TCPC_TRANSMIT_TYPE_SHIFT);
	ret = _cros_tcpc_write(cros_tcpc, TCPC_TRANSMIT, reg, 1);
	if (ret < 0)
		goto out;

	ret = 0;
out:
	mutex_unlock(&cros_tcpc->lock);
	return ret;
}

static int cros_tcpc_init(struct tcpc_dev *tcpc)
{
	struct cros_tcpc *cros_tcpc = tcpc_to_cros(tcpc);
	unsigned long timeout = jiffies + msecs_to_jiffies(2000); /* XXX */
	unsigned int reg;
	int ret;

	mutex_lock(&cros_tcpc->lock);

	while (time_before_eq(jiffies, timeout)) {
		ret = _cros_tcpc_read(cros_tcpc, TCPC_POWER_STATUS, &reg, 1);
		if (ret < 0)
			goto out;
		if (!(reg & TCPC_POWER_STATUS_UNINIT))
			break;
		usleep_range(10000, 20000);
	}
	if (time_after(jiffies, timeout)) {
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Clear all events */
	ret = _cros_tcpc_write(cros_tcpc, TCPC_ALERT, 0xffff, 2);
	if (ret < 0)
		goto out;

	if (cros_tcpc->controls_vbus)
		reg = TCPC_POWER_STATUS_VBUS_PRES;
	else
		reg = 0;
	ret = _cros_tcpc_write(cros_tcpc, TCPC_POWER_STATUS_MASK, reg, 1);
	if (ret < 0)
		goto out;

	reg = TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_FAILED |
		TCPC_ALERT_TX_DISCARDED | TCPC_ALERT_RX_STATUS |
		TCPC_ALERT_RX_HARD_RST | TCPC_ALERT_CC_STATUS;
	if (cros_tcpc->controls_vbus)
		reg |= TCPC_ALERT_POWER_STATUS;
	ret = _cros_tcpc_write(cros_tcpc, TCPC_ALERT_MASK, reg, 2);
	if (ret < 0)
		goto out;
	ret = 0;
out:
	mutex_unlock(&cros_tcpc->lock);
	return ret;
}

static int cros_ec_tcpc_notify(struct notifier_block *nb, unsigned long event,
				void *data)
{
	struct cros_tcpc *cros_tcpc;
	u16 status;

	cros_tcpc = container_of(nb, struct cros_tcpc, notifier);
	if (cros_tcpc_read(cros_tcpc, TCPC_ALERT, &status, 2) < 0) {
		dev_err(cros_tcpc->dev, "Failed to read status register");
		return NOTIFY_DONE;
	}

	/*
	 * Clear alert status for everything except RX_STATUS, which shouldn't
	 * be cleared until we have successfully retrieved message.
	 */
	if (status & ~TCPC_ALERT_RX_STATUS)
		cros_tcpc_write(cros_tcpc, TCPC_ALERT,
				status & ~TCPC_ALERT_RX_STATUS, 2);

	if (status & TCPC_ALERT_CC_STATUS)
		tcpm_cc_change(cros_tcpc->port);

	if (status & TCPC_ALERT_POWER_STATUS)
		tcpm_vbus_change(cros_tcpc->port);

	if (status & TCPC_ALERT_RX_STATUS) {
		struct pd_message msg;
		u8 rx_buffer[32];
		int cnt;

		memset(&msg, 0, sizeof(msg));

		cros_tcpc_read(cros_tcpc, TCPC_RX_BYTE_CNT, rx_buffer,
			       sizeof(rx_buffer));

		cnt = rx_buffer[0];
		if (cnt < 2) {
			dev_err(cros_tcpc->dev, "Rx data too small (%d)",
				rx_buffer[0]);
			goto skip;
		}

		msg.header = rx_buffer[1] | (rx_buffer[2] << 8);

		cnt -= 2;
		if (cnt > sizeof(msg.payload)) {
			dev_err(cros_tcpc->dev,
				 "Payload register reports %d bytes in receive queue, maximum %ld",
				 cnt, sizeof(msg.payload));
			cnt = sizeof(msg.payload);
		}

		if (cnt > 0)
			memcpy(msg.payload, &rx_buffer[3], cnt);

skip:
		/* Read complete, clear RX status alert bit */
		cros_tcpc_write(cros_tcpc, TCPC_ALERT, TCPC_ALERT_RX_STATUS, 2);

		tcpm_pd_receive(cros_tcpc->port, &msg);
	}

	if (status & TCPC_ALERT_RX_HARD_RST)
		tcpm_pd_hard_reset(cros_tcpc->port);

	if (status & TCPC_ALERT_TX_SUCCESS)
		tcpm_pd_transmit_complete(cros_tcpc->port, TCPC_TX_SUCCESS);
	else if (status & TCPC_ALERT_TX_DISCARDED)
		tcpm_pd_transmit_complete(cros_tcpc->port, TCPC_TX_DISCARDED);
	else if (status & TCPC_ALERT_TX_FAILED)
		tcpm_pd_transmit_complete(cros_tcpc->port, TCPC_TX_FAILED);

	return NOTIFY_OK;
}

#define PDO_FIXED_FLAGS (PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP |\
			 PDO_FIXED_USB_COMM)

static const u32 chell_board_src_pdo[] = {
		PDO_FIXED(5000, 1500, PDO_FIXED_FLAGS),
};

static const u32 chell_board_snk_pdo[] = {
		PDO_FIXED(5000, 500, PDO_FIXED_FLAGS),
		PDO_BATT(4750, 21000, 15000),
		PDO_VAR(4750, 21000, 3000),
};

#if 0
const struct svdm_mode chell_supported_modes[] = {
	{
		.svid = USB_SID_DISPLAYPORT,
		.enter = chell_enter_dp_mode,
		.status = chell_dp_status,
		.config = chell_dp_config,
		.post_config = chell_dp_post_config,
		.attention = chell_dp_attention,
		.exit = chell_dp_exit,
	},
};
#endif

static struct typec_altmode_desc chell_alt_modes[] = {
	{
	.svid = USB_SID_DISPLAYPORT,
	.n_modes = 1,
	.modes[0] = {
		.vdo = VDO_MODE_DP(0,	/* UFP pin cfg supported : none */
	                   MODE_DP_PIN_C | MODE_DP_PIN_D, /* DFP pin cfg supported */
			   0,		/* usb2.0 signalling even in AMode */
			   CABLE_PLUG,	/* its a plug */
			   MODE_DP_V13,	/* DPv1.3 Support, no Gen2 */
			   MODE_DP_SRC),/* source only */
		.desc = "DisplayPort",
		},
	},
	{
	.svid = USB_VID_GOOGLE,
	.n_modes = 1,
	.modes[0] = {
		.vdo = VDO_MODE_GOOGLE(MODE_GOOGLE_FU),
		.desc = "Firmware Update",
		},
	},
	{ },
};

#ifdef __NOTYET
static struct tcpm_alt_callbacks chell_alt_callbacks[] =
{
	{
	},
	{
	},
};
#endif

static struct tcpc_config chell_board_config = {
	.src_pdo = chell_board_src_pdo,
	.nr_src_pdo = ARRAY_SIZE(chell_board_src_pdo),
	.snk_pdo = chell_board_snk_pdo,
	.nr_snk_pdo = ARRAY_SIZE(chell_board_snk_pdo),

#if 0
	.modes = chell_supported_modes,
	.num_modes = ARRAY_SIZE(chell_supported_modes);
#endif

	.max_snk_mv = 20000,
	.max_snk_ma = 3000,
	.max_snk_mw = 45000,
	.operating_snk_mw = 15000,

	.type = TYPEC_PORT_DRP,
	.default_role = TYPEC_SINK,

	.alt_modes = chell_alt_modes,
};

static const struct dmi_system_id cros_tcpc_dmi_table[] = {
	{
		.ident = "Chell",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Hewlett-Packard"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Chell"),
		},
		.driver_data = &chell_board_config,
	},
	{ }
};

static int cros_tcpc_parse_config(struct cros_tcpc *cros_tcpc)
{
	const struct dmi_system_id *id;

	/*
	 * TODO:
	 * Populate struct tcpc_config from ACPI and/or devicetree if available
	 */

	id = dmi_first_match(cros_tcpc_dmi_table);
	if (!id || !id->driver_data)
		return -ENODEV;

	cros_tcpc->controls_vbus = true; /* XXX */

	cros_tcpc->tcpc.config = id->driver_data;

	return 0;
}

static int cros_tcpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cros_ec_dev *ec_dev = dev_get_drvdata(pdev->dev.parent);
	struct cros_tcpc *cros_tcpc;
	int err;
	u32 host_events;

	cros_tcpc = devm_kzalloc(dev, sizeof(*cros_tcpc), GFP_KERNEL);
	if (!cros_tcpc)
		return -ENOMEM;

	mutex_init(&cros_tcpc->lock);

	platform_set_drvdata(pdev, cros_tcpc);
	cros_tcpc->ec_device = ec_dev->ec_dev;
	cros_tcpc->dev = dev;

	cros_tcpc->tcpc.init = cros_tcpc_init;
	cros_tcpc->tcpc.get_vbus = cros_tcpc_get_vbus;
	cros_tcpc->tcpc.get_cc = cros_tcpc_get_cc;
	cros_tcpc->tcpc.set_cc = cros_tcpc_set_cc;
	cros_tcpc->tcpc.set_polarity = cros_tcpc_set_polarity;
	cros_tcpc->tcpc.set_vconn = cros_tcpc_set_vconn;
	cros_tcpc->tcpc.set_vbus = cros_tcpc_set_vbus;
	cros_tcpc->tcpc.set_current_limit = cros_tcpc_set_current_limit;
	cros_tcpc->tcpc.set_pd_rx = cros_tcpc_set_pd_rx;
	cros_tcpc->tcpc.set_roles = cros_tcpc_set_roles;
	cros_tcpc->tcpc.pd_transmit = cros_tcpc_pd_transmit;

	cros_tcpc->mux.set = cros_tcpc_set_mux;
	cros_tcpc->tcpc.mux = &cros_tcpc->mux;

	err = cros_tcpc_parse_config(cros_tcpc);
	if (err < 0)
		return err;

	cros_tcpc->port = tcpm_register_port(cros_tcpc->dev, &cros_tcpc->tcpc);
	if (IS_ERR(cros_tcpc->port))
		return PTR_ERR(cros_tcpc->port);

	/* XXX This notifier only works with ACPI  */
	cros_tcpc->notifier.notifier_call = cros_ec_tcpc_notify;
	err = blocking_notifier_chain_register(&cros_ec_pd_notifier,
					       &cros_tcpc->notifier);

	/* XXX What is this for ? */
	cros_tcpc->ec_device->cmd_readmem(cros_tcpc->ec_device, 0x34, 4,
					  &host_events);
	return 0;
}

static int cros_tcpc_remove(struct platform_device *pd)
{
	struct cros_tcpc *cros_tcpc = platform_get_drvdata(pd);

	blocking_notifier_chain_unregister(&cros_ec_pd_notifier,
					   &cros_tcpc->notifier);
	tcpm_unregister_port(cros_tcpc->port);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id cros_tcpc_of_match[] = {
	{ .compatible = "google,cros-ec-tcpc", },
	{},
};
MODULE_DEVICE_TABLE(of, cros_tcpc_of_match);
#endif

static struct platform_driver cros_tcpc_driver = {
	.driver = {
		.name = "cros-ec-tcpc",
		.of_match_table = of_match_ptr(cros_tcpc_of_match),
	},
	.probe = cros_tcpc_probe,
	.remove = cros_tcpc_remove,
};
module_platform_driver(cros_tcpc_driver);

MODULE_DESCRIPTION("Chrome EC USB Type-C Port Controller driver");
MODULE_LICENSE("GPL");
