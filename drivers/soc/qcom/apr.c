/* SPDX-License-Identifier: GPL-2.0
* Copyright (c) 2011-2016, The Linux Foundation
* Copyright (c) 2017, Linaro Limited
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/soc/qcom/apr.h>
#include <linux/rpmsg.h>
#include <linux/of.h>

struct apr_data {
	int (*get_data_src)(struct apr_hdr *hdr);
	int dest_id;
};

struct apr {
	struct rpmsg_endpoint *ch;
	struct device *dev;
	spinlock_t svcs_lock;
	struct list_head svcs;
	int dest_id;
	const struct apr_data *data;
};

/* Static CORE service on the ADSP */
static const struct apr_device_id core_svc_device_id =
		ADSP_AUDIO_APR_DEV("CORE", APR_SVC_ADSP_CORE);

/**
 * apr_send_pkt() - Send a apr message from apr device
 *
 * @adev: Pointer to previously registered apr device.
 * @buf: Pointer to buffer to send
 *
 * Return: Will be an negative on packet size on success.
 */
int apr_send_pkt(struct apr_device *adev, uint32_t *buf)
{
	struct apr *apr = dev_get_drvdata(adev->dev.parent);
	struct apr_hdr *hdr;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&adev->lock, flags);

	hdr = (struct apr_hdr *)buf;
	hdr->src_domain = APR_DOMAIN_APPS;
	hdr->src_svc = adev->svc_id;
	hdr->dest_domain = adev->domain_id;
	hdr->dest_svc = adev->svc_id;

	ret = rpmsg_send(apr->ch, buf, hdr->pkt_size);
	if (ret) {
		dev_err(&adev->dev, "Unable to send APR pkt %d\n",
			hdr->pkt_size);
	} else {
		ret = hdr->pkt_size;
	}

	spin_unlock_irqrestore(&adev->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(apr_send_pkt);

static void apr_dev_release(struct device *dev)
{
	struct apr_device *adev = to_apr_device(dev);

	kfree(adev);
}

static int qcom_rpmsg_q6_callback(struct rpmsg_device *rpdev, void *buf,
				  int len, void *priv, u32 addr)
{
	struct apr *apr = dev_get_drvdata(&rpdev->dev);
	struct apr_client_data data;
	struct apr_device *p, *c_svc = NULL;
	struct apr_driver *adrv = NULL;
	struct apr_hdr *hdr;
	uint16_t hdr_size;
	uint16_t msg_type;
	uint16_t ver;
	uint16_t src;
	uint16_t svc;

	if (!buf || len <= APR_HDR_SIZE) {
		dev_err(apr->dev, "APR: Improper apr pkt received:%p %d\n",
			buf, len);
		return -EINVAL;
	}

	hdr = buf;
	ver = APR_HDR_FIELD_VER(hdr->hdr_field);
	if (ver > APR_PKT_VER + 1)
		return -EINVAL;

	hdr_size = APR_HDR_FIELD_SIZE_BYTES(hdr->hdr_field);
	if (hdr_size < APR_HDR_SIZE) {
		dev_err(apr->dev, "APR: Wrong hdr size:%d\n", hdr_size);
		return -EINVAL;
	}

	if (hdr->pkt_size < APR_HDR_SIZE) {
		dev_err(apr->dev, "APR: Wrong paket size\n");
		return -EINVAL;
	}

	msg_type = APR_HDR_FIELD_MT(hdr->hdr_field);
	if (msg_type >= APR_MSG_TYPE_MAX && msg_type != APR_BASIC_RSP_RESULT) {
		dev_err(apr->dev, "APR: Wrong message type: %d\n", msg_type);
		return -EINVAL;
	}

	if (hdr->src_domain >= APR_DOMAIN_MAX ||
			hdr->dest_domain >= APR_DOMAIN_MAX ||
			hdr->src_svc >= APR_SVC_MAX ||
			hdr->dest_svc >= APR_SVC_MAX) {
		dev_err(apr->dev, "APR: Wrong APR header\n");
		return -EINVAL;
	}

	svc = hdr->dest_svc;
	src = apr->data->get_data_src(hdr);
	if (src == APR_DEST_MAX)
		return -EINVAL;

	spin_lock(&apr->svcs_lock);
	list_for_each_entry(p, &apr->svcs, node) {
		if (svc == p->svc_id) {
			c_svc = p;
			if (c_svc->dev.driver)
				adrv = to_apr_driver(c_svc->dev.driver);
			break;
		}
	}
	spin_unlock(&apr->svcs_lock);

	if (!adrv) {
		dev_err(apr->dev, "APR: service is not registered\n");
		return -EINVAL;
	}

	data.payload_size = hdr->pkt_size - hdr_size;
	data.opcode = hdr->opcode;
	data.src = src;
	data.src_port = hdr->src_port;
	data.dest_port = hdr->dest_port;
	data.token = hdr->token;
	data.msg_type = msg_type;

	if (data.payload_size > 0)
		data.payload = (char *)hdr + hdr_size;

	adrv->callback(c_svc, &data);

	return 0;
}

static const struct apr_device_id *apr_match(const struct apr_device_id *id,
					       const struct apr_device *adev)
{
	while (id->domain_id != 0 || id->svc_id != 0) {
		if (id->domain_id == adev->domain_id &&
		    id->svc_id == adev->svc_id &&
		    id->client_id == adev->client_id)
			return id;
		id++;
	}
	return NULL;
}

static int apr_device_match(struct device *dev, struct device_driver *drv)
{
	struct apr_device *adev = to_apr_device(dev);
	struct apr_driver *adrv = to_apr_driver(drv);

	return !!apr_match(adrv->id_table, adev);
}

static int apr_device_probe(struct device *dev)
{
	struct apr_device	*adev;
	struct apr_driver	*adrv;
	int ret = 0;

	adev = to_apr_device(dev);
	adrv = to_apr_driver(dev->driver);

	ret = adrv->probe(adev);

	return ret;
}

static int apr_device_remove(struct device *dev)
{
	struct apr_device *adev = to_apr_device(dev);
	struct apr_driver *adrv;
	struct apr *apr = dev_get_drvdata(adev->dev.parent);

	if (dev->driver) {
		adrv = to_apr_driver(dev->driver);
		if (adrv->remove)
			adrv->remove(adev);
		spin_lock(&apr->svcs_lock);
		list_del(&adev->node);
		spin_unlock(&apr->svcs_lock);
	}

	return 0;
}

struct bus_type aprbus_type = {
	.name		= "aprbus",
	.match		= apr_device_match,
	.probe		= apr_device_probe,
	.remove		= apr_device_remove,
};
EXPORT_SYMBOL_GPL(aprbus_type);

/**
 * apr_add_device() - Add a new apr device
 *
 * @dev: Pointer to apr device.
 * @id: Pointer to apr device id to add.
 *
 * Return: Will be an negative on error or a zero on success.
 */
int apr_add_device(struct device *dev, const struct apr_device_id *id)
{
	struct apr *apr = dev_get_drvdata(dev);
	struct apr_device *adev = NULL;

	if (!apr)
		return -EINVAL;

	adev = kzalloc(sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	spin_lock_init(&adev->lock);

	adev->svc_id = id->svc_id;
	adev->dest_id = apr->dest_id;
	adev->client_id = id->client_id;
	adev->domain_id = id->domain_id;
	adev->version = id->svc_version;

	adev->dev.bus = &aprbus_type;
	adev->dev.parent = dev;
	adev->dev.release = apr_dev_release;
	adev->dev.driver = NULL;

	dev_set_name(&adev->dev, "apr:%s:%x:%x:%x", id->name, id->domain_id,
				 id->svc_id, id->client_id);

	spin_lock(&apr->svcs_lock);
	list_add_tail(&adev->node, &apr->svcs);
	spin_unlock(&apr->svcs_lock);

	return device_register(&adev->dev);
}
EXPORT_SYMBOL_GPL(apr_add_device);

static int qcom_rpmsg_q6_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct apr *apr;

	apr = devm_kzalloc(dev, sizeof(*apr), GFP_KERNEL);
	if (!apr)
		return -ENOMEM;

	apr->data = of_device_get_match_data(dev);
	if (!apr->data)
		return -ENODEV;

	apr->dest_id = apr->data->dest_id;
	dev_set_drvdata(dev, apr);
	apr->ch = rpdev->ept;
	apr->dev = dev;
	INIT_LIST_HEAD(&apr->svcs);

	/* register core service */
	apr_add_device(dev, &core_svc_device_id);

	return 0;
}

static int apr_remove_device(struct device *dev, void *null)
{
	struct apr_device *adev = to_apr_device(dev);

	device_unregister(&adev->dev);

	return 0;
}

static void qcom_rpmsg_q6_remove(struct rpmsg_device *rpdev)
{
	device_for_each_child(&rpdev->dev, NULL, apr_remove_device);
}

static int apr_v2_get_data_src(struct apr_hdr *hdr)
{
	if (hdr->src_domain == APR_DOMAIN_MODEM)
		return APR_DEST_MODEM;
	else if (hdr->src_domain == APR_DOMAIN_ADSP)
		return APR_DEST_QDSP6;

	return APR_DEST_MAX;
}

/*
 * __apr_driver_register() - Client driver registration with aprbus
 *
 * @drv:Client driver to be associated with client-device.
 * @owner: owning module/driver
 *
 * This API will register the client driver with the aprbus
 * It is called from the driver's module-init function.
 */
int __apr_driver_register(struct apr_driver *drv, struct module *owner)
{
	/* ID table is mandatory to match the devices to probe */
	if (!drv->id_table)
		return -EINVAL;

	drv->driver.bus = &aprbus_type;
	drv->driver.owner = owner;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(__apr_driver_register);

/*
 * apr_driver_unregister() - Undo effect of apr_driver_register
 *
 * @drv: Client driver to be unregistered
 */
void apr_driver_unregister(struct apr_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(apr_driver_unregister);

static const struct apr_data apr_v2_data = {
	.get_data_src = apr_v2_get_data_src,
	.dest_id = APR_DEST_QDSP6,
};

static const struct of_device_id qcom_rpmsg_q6_of_match[] = {
	{ .compatible = "qcom,apr-msm8996", .data = &apr_v2_data},
	{}
};

static struct rpmsg_driver qcom_rpmsg_q6_driver = {
	.probe = qcom_rpmsg_q6_probe,
	.remove = qcom_rpmsg_q6_remove,
	.callback = qcom_rpmsg_q6_callback,
	.drv = {
		.name = "qcom_rpmsg_q6",
		.owner = THIS_MODULE,
		.of_match_table = qcom_rpmsg_q6_of_match,
		},
};

static int __init apr_init(void)
{
	int ret;

	ret = register_rpmsg_driver(&qcom_rpmsg_q6_driver);
	if (!ret)
		return bus_register(&aprbus_type);

	return ret;
}

static void __exit apr_exit(void)
{
	bus_unregister(&aprbus_type);
	unregister_rpmsg_driver(&qcom_rpmsg_q6_driver);
}

subsys_initcall(apr_init);
module_exit(apr_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm APR Bus");
