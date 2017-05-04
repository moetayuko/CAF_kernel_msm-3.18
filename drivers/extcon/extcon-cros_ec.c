/**
 * drivers/extcon/extcon-cros_ec - ChromeOS Embedded Controller extcon
 *
 * Copyright (C) 2015 Google, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/extcon.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/usb/typec.h>

#define CROS_EC_USB_POLLING_DELAY msecs_to_jiffies(1000)

/*
 * Timeout for a USB PD power swap execution
 * 1000 ms for tSwapRecovery : maximum time after Hard Reset to settle
 *  275 ms for tSrcTurnOn (VBUS going from 0V to 5V)
 *  650 ms for tSafe0V (VBUS going to 0V)
 *  500 ms of extra margin
 */
#define POWER_SWAP_TIMEOUT msecs_to_jiffies(2425)
/*
 * Timeout for USB PD data swap execution
 *   30 ms for tSenderResponse
 * 2x 1 ms for tReceive
 *   some margin for events and AP/EC communication
 */
#define DATA_SWAP_TIMEOUT msecs_to_jiffies(150)

struct cros_ec_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	int port_id;

	struct cros_ec_device *ec;

	struct notifier_block notifier;

	bool connected;
	enum typec_data_role dr;	/* data role */
	enum typec_role vr;		/* vconn role */
	enum typec_role pr;		/* power role */
	bool dp;			/* DisplayPort enabled */
	bool mux;			/* SuperSpeed (usb3) enabled */
	wait_queue_head_t role_wait;

	struct typec_capability typec_caps;
	struct typec_port *typec_port;

	struct typec_partner_desc partner_desc;
	struct typec_partner *partner;
};

static const unsigned int usb_type_c_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_DISP_DP,
	EXTCON_NONE,
};

/**
 * cros_ec_pd_command() - Send a command to the EC.
 * @info: pointer to struct cros_ec_extcon_info
 * @command: EC command
 * @version: EC command version
 * @outdata: EC command output data
 * @outsize: Size of outdata
 * @indata: EC command input data
 * @insize: Size of indata
 *
 * Return: 0 on success, <0 on failure.
 */
static int cros_ec_pd_command(struct cros_ec_extcon_info *info,
			      unsigned int command,
			      unsigned int version,
			      void *outdata,
			      unsigned int outsize,
			      void *indata,
			      unsigned int insize)
{
	struct cros_ec_command *msg;
	int ret;

	msg = kzalloc(sizeof(*msg) + max(outsize, insize), GFP_KERNEL);

	msg->version = version;
	msg->command = command;
	msg->outsize = outsize;
	msg->insize = insize;

	if (outsize)
		memcpy(msg->data, outdata, outsize);

	ret = cros_ec_cmd_xfer_status(info->ec, msg);
	if (ret >= 0 && insize)
		memcpy(indata, msg->data, insize);

	kfree(msg);
	return ret;
}

/**
 * cros_ec_usb_get_pd_mux_state() - Get PD mux state for given port.
 * @info: pointer to struct cros_ec_extcon_info
 *
 * Return: PD mux state on success, <0 on failure.
 */
static int cros_ec_usb_get_pd_mux_state(struct cros_ec_extcon_info *info)
{
	struct ec_params_usb_pd_mux_info req;
	struct ec_response_usb_pd_mux_info resp;
	int ret;

	req.port = info->port_id;
	ret = cros_ec_pd_command(info, EC_CMD_USB_PD_MUX_INFO, 0,
				 &req, sizeof(req),
				 &resp, sizeof(resp));
	if (ret < 0)
		return ret;

	return resp.flags;
}

/**
 * cros_ec_usb_get_role() - Get role info about possible PD device attached to a
 * given port.
 * @info: pointer to struct cros_ec_extcon_info
 * @polarity: pointer to cable polarity (return value)
 *
 * Return: role info on success, -ENOTCONN if no cable is connected, <0 on failure.
 */
static int cros_ec_usb_get_role(struct cros_ec_extcon_info *info,
				bool *polarity,
				bool *pd_capable)
{
	struct ec_params_usb_pd_control pd_control;
	struct ec_response_usb_pd_control_v1 resp;
	int ret;

	pd_control.port = info->port_id;
	pd_control.role = USB_PD_CTRL_ROLE_NO_CHANGE;
	pd_control.mux = USB_PD_CTRL_MUX_NO_CHANGE;
	pd_control.swap = USB_PD_CTRL_SWAP_NONE;
	ret = cros_ec_pd_command(info, EC_CMD_USB_PD_CONTROL, 1,
				 &pd_control, sizeof(pd_control),
				 &resp, sizeof(resp));
	if (ret < 0)
		return ret;

	if (!(resp.enabled & PD_CTRL_RESP_ENABLED_CONNECTED))
		return -ENOTCONN;

	*polarity = resp.polarity;
	*pd_capable = resp.enabled & PD_CTRL_RESP_ENABLED_PD_CAPABLE;

	return resp.role;
}

/**
 * cros_ec_pd_get_num_ports() - Get number of EC charge ports.
 * @info: pointer to struct cros_ec_extcon_info
 *
 * Return: number of ports on success, <0 on failure.
 */
static int cros_ec_pd_get_num_ports(struct cros_ec_extcon_info *info)
{
	struct ec_response_usb_pd_ports resp;
	int ret;

	ret = cros_ec_pd_command(info, EC_CMD_USB_PD_PORTS,
				 0, NULL, 0, &resp, sizeof(resp));
	if (ret < 0)
		return ret;

	return resp.num_ports;
}

static const char *cros_ec_usb_role_string(bool connected,
					   enum typec_data_role dr)
{
	return connected ? (dr == TYPEC_HOST ? "DFP" : "UFP") : "DISCONNECTED";
}

static int extcon_cros_ec_detect_cable(struct cros_ec_extcon_info *info,
				       bool force)
{
	struct device *dev = info->dev;
	int role;
	enum typec_data_role dr;
	enum typec_role pr, vr;
	bool polarity, dp, mux, hpd;
	bool pd_capable;
	bool connected;

	role = cros_ec_usb_get_role(info, &polarity, &pd_capable);
	if (role < 0) {
		if (role != -ENOTCONN) {
			dev_err(dev, "failed getting role err = %d\n", role);
			return role;
		}
		connected = false;
		pd_capable = false;
		dr = -1;
		pr = -1;
		vr = -1;
		polarity = false;
		dp = false;
		mux = false;
		hpd = false;
		dev_dbg(dev, "disconnected\n");
	} else {
		int pd_mux_state;

		connected = true;
		dr = (role & PD_CTRL_RESP_ROLE_DATA) ?
			TYPEC_HOST : TYPEC_DEVICE;
		pr = (role & PD_CTRL_RESP_ROLE_POWER) ?
			TYPEC_SOURCE : TYPEC_SINK;
		vr = (role & PD_CTRL_RESP_ROLE_VCONN) ?
		  	TYPEC_SOURCE : TYPEC_SINK;
		pd_mux_state = cros_ec_usb_get_pd_mux_state(info);
		if (pd_mux_state < 0)
			pd_mux_state = USB_PD_MUX_USB_ENABLED;
		dp = pd_mux_state & USB_PD_MUX_DP_ENABLED;
		mux = pd_mux_state & USB_PD_MUX_USB_ENABLED;
		hpd = pd_mux_state & USB_PD_MUX_HPD_IRQ;

		dev_dbg(dev,
			"connected role 0x%x dr %d pr %d pol %d mux %d dp %d hpd %d\n",
			role, dr, pr, polarity, mux, dp, hpd);
	}

	if (force || info->connected != connected || info->dr != dr ||
	    info->pr != pr || info->dp != dp || info->mux != mux ||
	    info->vr != vr) {
		bool host_connected = false, device_connected = false;

		dev_dbg(dev, "Role switch! role = %s\n",
			cros_ec_usb_role_string(connected, dr));
		info->connected = connected;
		info->dr = dr;
		info->pr = pr;
		info->dp = dp;
		info->mux = mux;
		info->vr = vr;

		if (connected) {
			if (dr == TYPEC_DEVICE)
				device_connected = true;
			else
				host_connected = true;
		}

		extcon_set_state(info->edev, EXTCON_USB, device_connected);
		extcon_set_state(info->edev, EXTCON_USB_HOST, host_connected);
		extcon_set_state(info->edev, EXTCON_DISP_DP, dp);
		extcon_set_property(info->edev, EXTCON_USB,
				    EXTCON_PROP_USB_VBUS,
				    (union extcon_property_value)(int)pr);
		extcon_set_property(info->edev, EXTCON_USB_HOST,
				    EXTCON_PROP_USB_VBUS,
				    (union extcon_property_value)(int)pr);
		extcon_set_property(info->edev, EXTCON_USB,
				    EXTCON_PROP_USB_TYPEC_POLARITY,
				    (union extcon_property_value)(int)polarity);
		extcon_set_property(info->edev, EXTCON_USB_HOST,
				    EXTCON_PROP_USB_TYPEC_POLARITY,
				    (union extcon_property_value)(int)polarity);
		extcon_set_property(info->edev, EXTCON_DISP_DP,
				    EXTCON_PROP_USB_TYPEC_POLARITY,
				    (union extcon_property_value)(int)polarity);
		extcon_set_property(info->edev, EXTCON_USB,
				    EXTCON_PROP_USB_SS,
				    (union extcon_property_value)(int)mux);
		extcon_set_property(info->edev, EXTCON_USB_HOST,
				    EXTCON_PROP_USB_SS,
				    (union extcon_property_value)(int)mux);
		extcon_set_property(info->edev, EXTCON_DISP_DP,
				    EXTCON_PROP_USB_SS,
				    (union extcon_property_value)(int)mux);

		extcon_set_property(info->edev, EXTCON_DISP_DP,
				    EXTCON_PROP_DISP_HPD,
				    (union extcon_property_value)(int)hpd);

		extcon_sync(info->edev, EXTCON_USB);
		extcon_sync(info->edev, EXTCON_USB_HOST);
		extcon_sync(info->edev, EXTCON_DISP_DP);

		wake_up_all(&info->role_wait);

		if (connected) {
			if (!info->partner) {
				info->partner_desc.usb_pd = pd_capable;
				info->partner_desc.accessory = 0;
				// TODO: Set partner identity if available
				info->partner =
				  typec_register_partner(info->typec_port,
							 &info->partner_desc);
			}
			typec_set_data_role(info->typec_port, dr);
			typec_set_pwr_role(info->typec_port, pr);
			typec_set_vconn_role(info->typec_port, vr);
			// FIXME
			typec_set_pwr_opmode(info->typec_port,
					     pr == TYPEC_SOURCE ?
					     	TYPEC_PWR_MODE_PD :
						TYPEC_PWR_MODE_USB);
		} else if (info->partner) {
			typec_unregister_partner(info->partner);
			info->partner = NULL;
		}
	} else if (hpd) {
		extcon_set_property(info->edev, EXTCON_DISP_DP,
				    EXTCON_PROP_DISP_HPD,
				    (union extcon_property_value)(int)hpd);
		extcon_sync(info->edev, EXTCON_DISP_DP);
	}
	return 0;
}

static int extcon_cros_ec_event(struct notifier_block *nb,
	unsigned long queued_during_suspend, void *_notify)
{
	struct cros_ec_extcon_info *info;
	struct cros_ec_device *ec;
	u32 host_event;

	info = container_of(nb, struct cros_ec_extcon_info, notifier);
	ec = info->ec;

	host_event = cros_ec_get_host_event(ec);
	if (host_event & (EC_HOST_EVENT_MASK(EC_HOST_EVENT_PD_MCU) |
			  EC_HOST_EVENT_MASK(EC_HOST_EVENT_USB_MUX))) {
		extcon_cros_ec_detect_cable(info, false);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static inline struct cros_ec_extcon_info *
typec_cap_to_info(const struct typec_capability *cap)
{
        return container_of(cap, struct cros_ec_extcon_info, typec_caps);
}

static int extcon_cros_ec_dr_set(const struct typec_capability *cap,
				 enum typec_data_role new_dr)
{
	struct cros_ec_extcon_info *info = typec_cap_to_info(cap);
	struct device *dev = info->dev;
	struct ec_params_usb_pd_control pd_control;
	struct ec_response_usb_pd_control_v1 resp;
	int ret;

	dev_info(dev, "Force Data Role to %d (from %d)\n", new_dr, info->dr);

	if (new_dr == info->dr)
		return 0;

	pd_control.port = info->port_id;
	pd_control.role = USB_PD_CTRL_ROLE_NO_CHANGE;
	pd_control.mux = USB_PD_CTRL_MUX_NO_CHANGE;
	pd_control.swap = USB_PD_CTRL_SWAP_DATA;
	ret = cros_ec_pd_command(info, EC_CMD_USB_PD_CONTROL, 1,
				 &pd_control, sizeof(pd_control),
				 &resp, sizeof(resp));
	dev_dbg(dev, "EC data swap to %s = %d\n",
		new_dr == TYPEC_HOST ? "dfp" : "ufp", ret);
	if (ret < 0)
		return ret;

	/* wait for the swap to happen or timeout */
	ret = wait_event_timeout(info->role_wait, new_dr == info->dr,
				 DATA_SWAP_TIMEOUT);
	dev_dbg(dev, "data swap %s role %s\n",
		ret == 0 ? "timed out" : "succeeded", info->dr ? "UFP" : "DFP");

	return ret == 0 ? -ETIMEDOUT : ret;
}

static int extcon_cros_ec_pr_set(const struct typec_capability *cap,
				 enum typec_role new_pr)
{
	struct cros_ec_extcon_info *info = typec_cap_to_info(cap);
	struct device *dev = info->dev;
	struct ec_params_charge_port_override p;
	int ret;

	dev_info(dev, "Force Power Role to %d (from %d)\n", new_pr, info->pr);

	if (new_pr == info->pr)
		return 0;

	switch (new_pr) {
	case TYPEC_SOURCE:
		p.override_port = OVERRIDE_DONT_CHARGE;
		break;
	case TYPEC_SINK:
		p.override_port = 0;
		break;
	default:
		return -EINVAL;
	}

	ret = cros_ec_pd_command(info, EC_CMD_PD_CHARGE_PORT_OVERRIDE, 0,
				 &p, sizeof(p), NULL, 0);
	dev_dbg(dev, "EC charge port override to %d = %d\n",
		p.override_port, ret);
	if (ret < 0)
		return ret;

	/* wait for the swap to happen or timeout */
	ret = wait_event_timeout(info->role_wait, new_pr == info->pr,
				 POWER_SWAP_TIMEOUT);
	dev_dbg(dev, "power swap %s role %s\n",
		ret == 0 ? "timed out" : "succeed", info->pr ? "SNK" : "SRC");

	return ret == 0 ? -ETIMEDOUT : ret;
}

static int extcon_cros_ec_probe(struct platform_device *pdev)
{
	struct cros_ec_extcon_info *info;
	struct cros_ec_device *ec = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int numports, ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->ec = ec;

	if (np) {
		u32 port;

		ret = of_property_read_u32(np, "google,usb-port-id", &port);
		if (ret < 0) {
			dev_err(dev, "Missing google,usb-port-id property\n");
			return ret;
		}
		info->port_id = port;
	} else {
		info->port_id = pdev->id;
	}

	numports = cros_ec_pd_get_num_ports(info);
	if (numports < 0) {
		dev_err(dev, "failed getting number of ports! ret = %d\n",
			numports);
		return numports;
	}

	if (info->port_id >= numports) {
		dev_err(dev, "This system only supports %d ports\n", numports);
		return -ENODEV;
	}

	info->edev = devm_extcon_dev_allocate(dev, usb_type_c_cable);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}

	extcon_set_property_capability(info->edev, EXTCON_USB,
				       EXTCON_PROP_USB_VBUS);
	extcon_set_property_capability(info->edev, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_VBUS);
	extcon_set_property_capability(info->edev, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(info->edev, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(info->edev, EXTCON_DISP_DP,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(info->edev, EXTCON_USB,
				       EXTCON_PROP_USB_SS);
	extcon_set_property_capability(info->edev, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_SS);
	extcon_set_property_capability(info->edev, EXTCON_DISP_DP,
				       EXTCON_PROP_USB_SS);
	extcon_set_property_capability(info->edev, EXTCON_DISP_DP,
				       EXTCON_PROP_DISP_HPD);

	info->dr = -1;
	info->pr = -1;
	info->connected = false;
	init_waitqueue_head(&info->role_wait);

	platform_set_drvdata(pdev, info);

	info->typec_caps.prefer_role = TYPEC_SINK;
	info->typec_caps.type = TYPEC_PORT_DRP;
	info->typec_caps.revision = 0x0120;
	info->typec_caps.pd_revision = 0x0200;
	info->typec_caps.dr_set = extcon_cros_ec_dr_set;
	info->typec_caps.pr_set = extcon_cros_ec_pr_set;

	info->typec_port = typec_register_port(dev, &info->typec_caps);
	if (!info->typec_port)
		return -ENOMEM;

	/* Get PD events from the EC */
	info->notifier.notifier_call = extcon_cros_ec_event;
	ret = blocking_notifier_chain_register(&info->ec->event_notifier,
					       &info->notifier);
	if (ret < 0) {
		dev_err(dev, "failed to register notifier\n");
		goto unregister_port;
	}

	/* Perform initial detection */
	ret = extcon_cros_ec_detect_cable(info, true);
	if (ret < 0) {
		dev_err(dev, "failed to detect initial cable state\n");
		goto unregister_notifier;
	}

	return 0;

unregister_notifier:
	blocking_notifier_chain_unregister(&info->ec->event_notifier,
					   &info->notifier);
unregister_port:
	typec_unregister_port(info->typec_port);
	return ret;
}

static int extcon_cros_ec_remove(struct platform_device *pdev)
{
	struct cros_ec_extcon_info *info = platform_get_drvdata(pdev);

	blocking_notifier_chain_unregister(&info->ec->event_notifier,
					   &info->notifier);

	typec_unregister_port(info->typec_port);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int extcon_cros_ec_suspend(struct device *dev)
{
	return 0;
}

static int extcon_cros_ec_resume(struct device *dev)
{
	int ret;
	struct cros_ec_extcon_info *info = dev_get_drvdata(dev);

	ret = extcon_cros_ec_detect_cable(info, true);
	if (ret < 0)
		dev_err(dev, "failed to detect cable state on resume\n");

	return 0;
}

static const struct dev_pm_ops extcon_cros_ec_dev_pm_ops = {

	SET_SYSTEM_SLEEP_PM_OPS(extcon_cros_ec_suspend, extcon_cros_ec_resume)
};

#define DEV_PM_OPS	(&extcon_cros_ec_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id extcon_cros_ec_of_match[] = {
	{ .compatible = "google,extcon-cros-ec" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, extcon_cros_ec_of_match);
#endif /* CONFIG_OF */

static struct platform_driver extcon_cros_ec_driver = {
	.driver = {
		.name  = "extcon-cros-ec",
		.of_match_table = of_match_ptr(extcon_cros_ec_of_match),
		.pm = DEV_PM_OPS,
	},
	.remove  = extcon_cros_ec_remove,
	.probe   = extcon_cros_ec_probe,
};

module_platform_driver(extcon_cros_ec_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS Embedded Controller extcon driver");
