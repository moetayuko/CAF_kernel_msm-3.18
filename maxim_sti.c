/* drivers/input/touchscreen/maxim_sti.c
 *
 * Maxim SmartTouch Imager Touchscreen Driver
 *
 * Copyright (c)2013 Maxim Integrated Products, Inc.
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

#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/kthread.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/crc16.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/maxim_sti.h>
#include <asm/byteorder.h>  /* MUST include this header to get byte order */

/****************************************************************************\
* Custom features                                                            *
\****************************************************************************/

#define INPUT_ENABLE_DISABLE  0
#define CPU_BOOST             0

#if CPU_BOOST
#include <linux/pm_qos.h>
#endif

/****************************************************************************\
* Device context structure, globals, and macros                              *
\****************************************************************************/

struct dev_data {
	spinlock_t                   coherency_lock;
	struct mutex                 coherency_mutex;
	struct completion            *coherency_completion;
	u32                          coherency_count;
	u8                           *tx_buf;
	u8                           *rx_buf;
	u32                          nl_seq;
	u8                           nl_mc_group_count;
	bool                         nl_enabled;
	bool                         start_fusion;
	bool                         suspend_in_progress;
	bool                         resume_in_progress;
	bool                         eraser_active;
	bool                         syscall_rx1;
	bool                         syscall_rx2;
	bool                         irq_registered;
	bool                         irq_enabled;
	u16                          irq_param[MAX_IRQ_PARAMS];
	char                         syscall_tx_buf[NL_BUF_SIZE];
	char                         syscall_rx_buf1[NL_BUF_SIZE];
	char                         syscall_rx_buf2[NL_BUF_SIZE];
	char                         input_phys[128];
	struct input_dev             *input_dev;
#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend         early_suspend;
#endif
	struct completion            suspend_resume;
	struct spi_device            *spi;
	struct genl_family           nl_family;
	struct genl_ops              *nl_ops;
	struct genl_multicast_group  *nl_mc_groups;
	struct sk_buff               *outgoing_skb;
	struct sk_buff_head          incoming_skb_queue;
	struct task_struct           *thread;
	struct task_struct           *syscall_reader;
	struct sched_param           thread_sched;
	struct list_head             dev_list;
#if CPU_BOOST
	struct pm_qos_request        cpus_req;
	struct pm_qos_request        freq_req;
	unsigned long                boost_freq;
#endif
};

static struct list_head  dev_list;
static spinlock_t        dev_lock;
static bool async = false;
static u8 xfr_state = 3;
static u16 clr_status, b_address;

static irqreturn_t irq_handler(int irq, void *context);

#define ERROR(a, b...) printk(KERN_ERR "%s driver(ERROR:%s:%d): " a "\n", \
			      dd->nl_family.name, __func__, __LINE__, ##b)
#define INFO(a, b...) printk(KERN_INFO "%s driver: " a "\n", \
			     dd->nl_family.name, ##b)

/****************************************************************************\
* Chip access methods                                                        *
\****************************************************************************/

static	struct spi_message   message;
static	struct spi_transfer  transfer;
static void xfr_complete(void *arg);
extern int msm_spi_transfer(struct spi_device *spi, struct spi_message *msg);
extern int (*maxim_fusion)(int type, char __user *buf, int len);

/* ======================================================================== */

static int chip_read(struct dev_data *dd, u16 address, u8 *buf, u16 len)
{
	u16                  *tx_buf = (u16 *)(((size_t)dd->tx_buf + 64) & ~63);
	u16                  *rx_buf = (u16 *)(((size_t)dd->rx_buf + 64) & ~63);
//	u16                  *tx_buf = (u16 *)dd->tx_buf;
//	u16                  *rx_buf = (u16 *)dd->rx_buf;
	u16                  words = len / sizeof(u16);
	u16                  *ptr2 = rx_buf + 2;
#ifdef __LITTLE_ENDIAN
	u16                  *ptr1 = (u16 *)buf, i;
#endif
	int                  ret;

	if (tx_buf == NULL || rx_buf == NULL)
		return -ENOMEM;

	tx_buf[0] = (address << 1) | 0x0001;
#ifdef __LITTLE_ENDIAN
	tx_buf[0] = (tx_buf[0] << 8) | (tx_buf[0] >> 8);
	tx_buf[1] = (words << 8) | (words >> 8);
#else
	tx_buf[1] = words;
#endif

	spi_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));

	transfer.len = len + 2 * sizeof(u16);
	transfer.tx_buf = tx_buf;
	transfer.rx_buf = rx_buf;
	spi_message_add_tail(&transfer, &message);
if (async) {
	message.complete = xfr_complete;
	message.context = dd;
	message.spi = dd->spi;
	message.status = -EINPROGRESS;
	return msm_spi_transfer(dd->spi, &message);
}

	do {
		ret = spi_sync(dd->spi, &message);
	} while (ret == -EAGAIN);

#ifdef __LITTLE_ENDIAN
	for (i = 0; i < words; i++)
		ptr1[i] = (ptr2[i] << 8) | (ptr2[i] >> 8);
#else
	memcpy(buf, ptr2, len);
#endif
	return ret;
}

static int chip_write(struct dev_data *dd, u16 address, u8 *buf, u16 len)
{
	u16  *tx_buf = (u16 *)dd->tx_buf;
//	u16  *tx_buf = (u16 *)(((size_t)dd->tx_buf + 64) & ~63);
	u16  words = len / sizeof(u16);
#ifdef __LITTLE_ENDIAN
	u16  i;
#endif
	int  ret;

	if (tx_buf == NULL)
		return -ENOMEM;

	tx_buf[0] = address << 1;
	tx_buf[1] = words;
	memcpy(tx_buf + 2, buf, len);
#ifdef __LITTLE_ENDIAN
	for (i = 0; i < (words + 2); i++)
		tx_buf[i] = (tx_buf[i] << 8) | (tx_buf[i] >> 8);
#endif

	spi_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	transfer.len = len + 2 * sizeof(u16);
	transfer.tx_buf = tx_buf;
	spi_message_add_tail(&transfer, &message);
if (async) {
	message.complete = xfr_complete;
	message.context = dd;
	message.spi = dd->spi;
	message.status = -EINPROGRESS;
	ret = msm_spi_transfer(dd->spi, &message);
} else {

	do {
		ret = spi_sync(dd->spi, &message);
	} while (ret == -EAGAIN);
}

	memset(dd->tx_buf, 0xFF, sizeof(dd->tx_buf));
	return ret;
}

static inline void coherency_lock(struct dev_data *dd)
{
	DECLARE_COMPLETION_ONSTACK(wait);
	unsigned long  flags;
	u32            count;

	spin_lock_irqsave(&dd->coherency_lock, flags);
	if ((count = ++dd->coherency_count) == 1)
		mutex_lock(&dd->coherency_mutex);
	spin_unlock_irqrestore(&dd->coherency_lock, flags);

	if (count > 1) {
		mutex_lock(&dd->coherency_mutex);
		return;
	}

	if (!dd->irq_registered) {
		if (xfr_state != 3)
			ERROR("coherency violation %d", xfr_state);
		return;
	}

	disable_irq(dd->spi->irq);
	dd->coherency_completion = &wait;
	if (xfr_state != 3)
		wait_for_completion(&wait);
	if (xfr_state != 3)
		ERROR("coherency violation %d", xfr_state);
	dd->coherency_completion = NULL;
}

static inline void coherency_unlock(struct dev_data *dd)
{
	unsigned long  flags;

	if (xfr_state != 3)
		ERROR("coherency violation %d", xfr_state);

	spin_lock_irqsave(&dd->coherency_lock, flags);
	mutex_unlock(&dd->coherency_mutex);
	if (dd->coherency_count > 0 && (--dd->coherency_count) == 0 &&
	    dd->irq_registered)
		enable_irq(dd->spi->irq);
	spin_unlock_irqrestore(&dd->coherency_lock, flags);
}

/* ======================================================================== */

static void stop_scan_canned(struct dev_data *dd)
{
	u16  value;

	value = dd->irq_param[9];
	(void)chip_write(dd, dd->irq_param[8], (u8 *)&value, sizeof(value));
	value = dd->irq_param[7];
	(void)chip_write(dd, dd->irq_param[0], (u8 *)&value, sizeof(value));
	usleep_range(dd->irq_param[11], dd->irq_param[11] + 1000);
	(void)chip_write(dd, dd->irq_param[0], (u8 *)&value, sizeof(value));
}

static void start_scan_canned(struct dev_data *dd)
{
	u16  value;

	value = dd->irq_param[10];
	(void)chip_write(dd, dd->irq_param[8], (u8 *)&value, sizeof(value));
}

/****************************************************************************\
* Suspend/resume processing                                                  *
\****************************************************************************/

#ifdef CONFIG_PM_SLEEP
static int suspend(struct device *dev)
{
	struct dev_data  *dd = spi_get_drvdata(to_spi_device(dev));

	if (dd->suspend_in_progress)
		return 0;

	init_completion(&dd->suspend_resume);
	dd->suspend_in_progress = true;
	wake_up_process(dd->thread);
	wait_for_completion(&dd->suspend_resume);
	return 0;
}

static int resume(struct device *dev)
{
	struct dev_data  *dd = spi_get_drvdata(to_spi_device(dev));

	if (!dd->suspend_in_progress)
		return 0;

	init_completion(&dd->suspend_resume);
	dd->resume_in_progress = true;
	wake_up_process(dd->thread);
	wait_for_completion(&dd->suspend_resume);
	return 0;
}

static const struct dev_pm_ops pm_ops = {
	.suspend = suspend,
	.resume = resume,
};

#if INPUT_ENABLE_DISABLE
static int input_disable(struct input_dev *dev)
{
	struct dev_data *dd = input_get_drvdata(dev);

	return suspend(&dd->spi->dev);
}

static int input_enable(struct input_dev *dev)
{
	struct dev_data *dd = input_get_drvdata(dev);

	return resume(&dd->spi->dev);
}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void early_suspend(struct early_suspend *h)
{
	struct dev_data *dd = container_of(h, struct dev_data, early_suspend);

	(void)suspend(&dd->spi->dev);
}

static void late_resume(struct early_suspend *h)
{
	struct dev_data *dd = container_of(h, struct dev_data, early_suspend);

	(void)resume(&dd->spi->dev);
}
#endif
#endif

/****************************************************************************\
* Netlink processing                                                         *
\****************************************************************************/

static inline int
nl_msg_new(struct dev_data *dd, u8 dst)
{
	dd->outgoing_skb = alloc_skb(NL_BUF_SIZE, GFP_ATOMIC);
	if (dd->outgoing_skb == NULL)
		return -ENOMEM;
	nl_msg_init(dd->outgoing_skb->data, dd->nl_family.id, dd->nl_seq++,
		    dst);
	if (dd->nl_seq == 0)
		dd->nl_seq++;
	return 0;
}

static int
nl_callback_noop(struct sk_buff *skb, struct genl_info *info)
{
	return 0;
}

static inline bool
nl_process_driver_msg(struct dev_data *dd, u16 msg_id, void *msg)
{
	struct maxim_sti_pdata        *pdata = dd->spi->dev.platform_data;
//	struct dr_add_mc_group        *add_mc_group_msg;
	struct dr_echo_request        *echo_msg;
	struct fu_echo_response       *echo_response;
	struct dr_chip_read           *read_msg;
	struct fu_chip_read_result    *read_result;
	struct dr_chip_write          *write_msg;
	struct dr_delay               *delay_msg;
	struct fu_irqline_status      *irqline_status;
	struct dr_config_irq          *config_irq_msg;
	struct dr_config_input        *config_input_msg;
	struct dr_input               *input_msg;
	u8                            i;
	int                           ret;

	switch (msg_id) {
	case DR_ADD_MC_GROUP:
#if 0
		add_mc_group_msg = msg;
		if (add_mc_group_msg->number >= pdata->nl_mc_groups) {
			ERROR("invalid multicast group number %d (%d)",
			      add_mc_group_msg->number, pdata->nl_mc_groups);
			return false;
		}
		if (dd->nl_mc_groups[add_mc_group_msg->number].id != 0)
			return false;
		dd->nl_ops[add_mc_group_msg->number].cmd =
						add_mc_group_msg->number;
		dd->nl_ops[add_mc_group_msg->number].doit = nl_callback_noop;
		ret = genl_register_ops(&dd->nl_family,
				&dd->nl_ops[add_mc_group_msg->number]);
		if (ret < 0)
			ERROR("failed to add multicast group op (%d)", ret);
		GENL_COPY(dd->nl_mc_groups[add_mc_group_msg->number].name,
			  add_mc_group_msg->name);
		ret = genl_register_mc_group(&dd->nl_family,
				&dd->nl_mc_groups[add_mc_group_msg->number]);
		if (ret < 0)
			ERROR("failed to add multicast group (%d)", ret);
#endif
		return false;
	case DR_ECHO_REQUEST:
		echo_msg = msg;
		echo_response = nl_alloc_attr(/*dd->outgoing_skb->data*/dd->syscall_rx_buf1,
					      FU_ECHO_RESPONSE,
					      sizeof(*echo_response));
		if (echo_response == NULL)
			goto alloc_attr_failure;
		echo_response->cookie = echo_msg->cookie;
		return true;
	case DR_CHIP_READ:
		read_msg = msg;
		read_result = nl_alloc_attr(/*dd->outgoing_skb->data*/dd->syscall_rx_buf1,
				FU_CHIP_READ_RESULT,
				sizeof(*read_result) + read_msg->length);
		if (read_result == NULL)
			goto alloc_attr_failure;
		coherency_lock(dd);
		read_result->address = read_msg->address;
		read_result->length = read_msg->length;
		ret = chip_read(dd, read_msg->address, read_result->data,
				read_msg->length);
		if (ret < 0)
			ERROR("failed to read from chip (%d)", ret);
		coherency_unlock(dd);
		return true;
	case DR_CHIP_WRITE:
		coherency_lock(dd);
		write_msg = msg;
		ret = chip_write(dd, write_msg->address, write_msg->data,
				 write_msg->length);
		if (ret < 0)
			ERROR("failed to write chip (%d)", ret);
		coherency_unlock(dd);
		return false;
	case DR_CHIP_RESET:
		pdata->reset(pdata, ((struct dr_chip_reset *)msg)->state);
		return false;
	case DR_GET_IRQLINE:
		irqline_status = nl_alloc_attr(/*dd->outgoing_skb->data*/dd->syscall_rx_buf1,
					       FU_IRQLINE_STATUS,
					       sizeof(*irqline_status));
		if (irqline_status == NULL)
			goto alloc_attr_failure;
		irqline_status->status = pdata->irq(pdata);
		return true;
	case DR_DELAY:
		delay_msg = msg;
		if (delay_msg->period > 1000)
			msleep(delay_msg->period / 1000);
		usleep_range(delay_msg->period % 1000,
			    (delay_msg->period % 1000) + 10);
		return false;
	case DR_CHIP_ACCESS_METHOD:
		return false;
	case DR_CONFIG_IRQ:
		config_irq_msg = msg;
		if (config_irq_msg->irq_params > MAX_IRQ_PARAMS) {
			ERROR("too many IRQ parameters");
			return false;
		}
		memcpy(dd->irq_param, config_irq_msg->irq_param,
		       config_irq_msg->irq_params * sizeof(dd->irq_param[0]));
		ret = request_irq(dd->spi->irq, irq_handler,
			(config_irq_msg->irq_edge == DR_IRQ_RISING_EDGE) ?
				IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING,
						pdata->nl_family, dd);
		if (ret < 0) {
			ERROR("failed to request IRQ (%d)", ret);
		} else {
			dd->irq_registered = true;
			wake_up_process(dd->thread);
		}
		return false;
	case DR_CONFIG_INPUT:
		config_input_msg = msg;
		dd->input_dev = input_allocate_device();
		if (dd->input_dev == NULL) {
			ERROR("failed to allocate input device");
		} else {
			snprintf(dd->input_phys, sizeof(dd->input_phys),
				 "%s/input0", dev_name(&dd->spi->dev));
			dd->input_dev->name = pdata->nl_family;
			dd->input_dev->phys = dd->input_phys;
			dd->input_dev->id.bustype = BUS_SPI;
#if defined(CONFIG_PM_SLEEP) && INPUT_ENABLE_DISABLE
			dd->input_dev->enable = input_enable;
			dd->input_dev->disable = input_disable;
			dd->input_dev->enabled = true;
			input_set_drvdata(dd->input_dev, dd);
#endif
			__set_bit(EV_SYN, dd->input_dev->evbit);
			__set_bit(EV_ABS, dd->input_dev->evbit);
			__set_bit(EV_KEY, dd->input_dev->evbit);
			__set_bit(BTN_TOOL_RUBBER, dd->input_dev->keybit);
			input_set_abs_params(dd->input_dev, ABS_MT_POSITION_X,
					     0, config_input_msg->x_range, 0,
					     0);
			input_set_abs_params(dd->input_dev, ABS_MT_POSITION_Y,
					     0, config_input_msg->y_range, 0,
					     0);
			input_set_abs_params(dd->input_dev, ABS_MT_PRESSURE,
					     0, 0xFF, 0, 0);
			input_set_abs_params(dd->input_dev,
					     ABS_MT_TRACKING_ID, 0,
					     MAX_INPUT_EVENTS, 0, 0);
			input_set_abs_params(dd->input_dev, ABS_MT_TOOL_TYPE,
					     0, MT_TOOL_MAX, 0, 0);
			ret = input_register_device(dd->input_dev);
			if (ret < 0) {
				input_free_device(dd->input_dev);
				dd->input_dev = NULL;
				ERROR("failed to register input device");
			}
		}
		return false;
	case DR_DECONFIG:
		if (dd->input_dev != NULL) {
			input_unregister_device(dd->input_dev);
			dd->input_dev = NULL;
		}
		if (dd->irq_registered) {
			free_irq(dd->spi->irq, dd);
			dd->irq_registered = false;
		}
		stop_scan_canned(dd);
		return false;
	case DR_INPUT:
		input_msg = msg;
		if (input_msg->events == 0) {
			if (dd->eraser_active) {
				input_report_key(dd->input_dev,
						 BTN_TOOL_RUBBER, 0);
				dd->eraser_active = false;
			}
			input_mt_sync(dd->input_dev);
			input_sync(dd->input_dev);
		} else {
			for (i = 0; i < input_msg->events; i++) {
				switch (input_msg->event[i].tool_type) {
				case DR_INPUT_FINGER:
					input_report_abs(dd->input_dev,
							 ABS_MT_TOOL_TYPE,
							 MT_TOOL_FINGER);
					break;
				case DR_INPUT_STYLUS:
					input_report_abs(dd->input_dev,
							 ABS_MT_TOOL_TYPE,
							 MT_TOOL_PEN);
					break;
				case DR_INPUT_ERASER:
					input_report_key(dd->input_dev,
							 BTN_TOOL_RUBBER, 1);
					dd->eraser_active = true;
					break;
				default:
					ERROR("invalid input tool type (%d)",
					      input_msg->event[i].tool_type);
					break;
				}
				input_report_abs(dd->input_dev,
						 ABS_MT_TRACKING_ID,
						 input_msg->event[i].id);
				input_report_abs(dd->input_dev,
						 ABS_MT_POSITION_X,
						 input_msg->event[i].x);
				input_report_abs(dd->input_dev,
						 ABS_MT_POSITION_Y,
						 input_msg->event[i].y);
				input_report_abs(dd->input_dev,
						 ABS_MT_PRESSURE,
						 input_msg->event[i].z);
				input_mt_sync(dd->input_dev);
			}
			input_sync(dd->input_dev);
		}
		return false;
	case DR_LEGACY_FWDL:
		return false;
	default:
		ERROR("unexpected message %d", msg_id);
		return false;
	}

alloc_attr_failure:
	ERROR("failed to allocate response for msg_id %d", msg_id);
	return false;
}

static int nl_process_msg(struct dev_data *dd, struct sk_buff *skb)
{
	struct nlattr  *attr;
	bool           send_reply = false;
	int            ret = 0, ret2;

	/* process incoming message */
	attr = NL_ATTR_FIRST(skb->data);
	for (; attr < NL_ATTR_LAST(skb->data); attr = NL_ATTR_NEXT(attr)) {
		if (nl_process_driver_msg(dd, attr->nla_type,
					  NL_ATTR_VAL(attr, void)))
			send_reply = true;
	}

	/* send back reply if requested */
	if (send_reply) {
		(void)skb_put(dd->outgoing_skb,
			      NL_SIZE(dd->outgoing_skb->data));
		if (NL_SEQ(skb->data) == 0)
			ret = genlmsg_unicast(sock_net(skb->sk),
					      dd->outgoing_skb,
					      NETLINK_CB(skb).pid);
		else
			ret = genlmsg_multicast(dd->outgoing_skb, 0,
					dd->nl_mc_groups[MC_FUSION].id,
					GFP_ATOMIC);
		if (ret < 0)
			ERROR("could not reply to fusion (%d)", ret);

		/* allocate new outgoing skb */
		ret2 = nl_msg_new(dd, MC_FUSION);
		if (ret2 < 0)
			ERROR("could not allocate outgoing skb (%d)", ret2);
	}

	/* free incoming message */
//	kfree_skb(skb);
	return ret;
}

static int
nl_callback_driver(struct sk_buff *skb, struct genl_info *info)
{
	struct dev_data  *dd;
//	struct sk_buff   *skb2;
	unsigned long    flags;

	/* locate device structure */
	spin_lock_irqsave(&dev_lock, flags);
	list_for_each_entry(dd, &dev_list, dev_list)
		if (dd->nl_family.id == NL_TYPE(skb->data))
			break;
	spin_unlock_irqrestore(&dev_lock, flags);
	if (&dd->dev_list == &dev_list)
		return -ENODEV;
	if (!dd->nl_enabled)
		return -EAGAIN;

#if 0
	/* queue incoming skb and wake up processing thread */
	skb2 = skb_clone(skb, GFP_ATOMIC);
	if (skb2 == NULL) {
		ERROR("failed to clone incoming skb");
		return -ENOMEM;
	} else {
		skb_queue_tail(&dd->incoming_skb_queue, skb2);
		wake_up_process(dd->thread);
		return 0;
	}
#else
	(void)nl_process_msg(dd, skb);
	return 0;
#endif
}

static int
nl_callback_fusion(struct sk_buff *skb, struct genl_info *info)
{
	struct dev_data  *dd;
	unsigned long    flags;

	/* locate device structure */
	spin_lock_irqsave(&dev_lock, flags);
	list_for_each_entry(dd, &dev_list, dev_list)
		if (dd->nl_family.id == NL_TYPE(skb->data))
			break;
	spin_unlock_irqrestore(&dev_lock, flags);
	if (&dd->dev_list == &dev_list)
		return -ENODEV;
	if (!dd->nl_enabled)
		return -EAGAIN;

	(void)genlmsg_multicast(skb_clone(skb, GFP_ATOMIC), 0,
				dd->nl_mc_groups[MC_FUSION].id, GFP_ATOMIC);
	return 0;
}

static struct dev_data *syscall_dd;
static int maxim_fusion_callback(int type, char __user *buf, int len)
{
	struct dev_data  *dd = syscall_dd;
	struct nlattr    *attr;
	bool             send_reply = false;
	int              ret;//, ret2;

	if (type == 20) {	/* send */
		ret = copy_from_user(dd->syscall_tx_buf, buf, len);
		if (ret < 0)
			return -EFAULT;

		nl_msg_init(/*dd->outgoing_skb->data*/dd->syscall_rx_buf1, dd->nl_family.id, dd->nl_seq++,
			    MC_FUSION);
		/* process incoming message */
		for (attr = NL_ATTR_FIRST(dd->syscall_tx_buf);
		     attr < NL_ATTR_LAST(dd->syscall_tx_buf);
		     attr = NL_ATTR_NEXT(attr)) {
			if (nl_process_driver_msg(dd, attr->nla_type,
						  NL_ATTR_VAL(attr, void)))
				send_reply = true;
		}

		/* send back reply if requested */
		if (send_reply) {
#if 0
			(void)skb_put(dd->outgoing_skb,
				      NL_SIZE(dd->outgoing_skb->data));
			ret = genlmsg_multicast(dd->outgoing_skb, 0,
					dd->nl_mc_groups[MC_FUSION].id,
					GFP_KERNEL);
			if (ret < 0)
				ERROR("could not reply to fusion (%d)", ret);

			/* allocate new outgoing skb */
			ret2 = nl_msg_new(dd, MC_FUSION);
			if (ret2 < 0)
				ERROR("could not allocate outgoing skb (%d)", ret2);
#else
			dd->syscall_rx1 = true;
			if (dd->syscall_reader)
				wake_up_process(dd->syscall_reader);
#endif
		}

		return ret;
	}
	if (type == 21) {	/* receive */
		while (1) {
			dd->syscall_reader = current;
			set_current_state(TASK_INTERRUPTIBLE);
			if (dd->syscall_rx1) {
				ret = copy_to_user(buf, dd->syscall_rx_buf1,
						NL_SIZE(dd->syscall_rx_buf1));
				dd->syscall_reader = NULL;
				dd->syscall_rx1 = false;
				dd->syscall_rx2 = false;
				set_current_state(TASK_RUNNING);
				return ret;
			}
			if (dd->syscall_rx2) {
				ret = copy_to_user(buf, dd->syscall_rx_buf2,
						NL_SIZE(dd->syscall_rx_buf2));
				dd->syscall_reader = NULL;
				dd->syscall_rx2 = false;
				set_current_state(TASK_RUNNING);
				return ret;
			}
			schedule();
			if (maxim_fusion == NULL)
				return -1;
		}
	}
	return 0;
}

/****************************************************************************\
* Interrupt processing                                                       *
\****************************************************************************/

#if 0
static irqreturn_t irq_handler(int irq, void *context)
{
	struct dev_data  *dd = context;

	wake_up_process(dd->thread);
	return IRQ_HANDLED;
}
#else
static irqreturn_t irq_handler(int irq, void *context)
{
	struct dev_data  *dd = context;
	u16              status;
	int              ret;

	if (xfr_state != 3 && xfr_state != 4)
		return IRQ_HANDLED;

	async = true;
	xfr_state = 0;
	ret = chip_read(dd, dd->irq_param[0], (u8 *)&status, sizeof(status));
	if (ret < 0) {
		ERROR("can't read IRQ status (%d)", ret);
	}
	async = false;

	return IRQ_HANDLED;
}
#endif

static void xfr_complete(void *arg)
{
	struct dev_data         *dd = arg;
	struct maxim_sti_pdata  *pdata = dd->spi->dev.platform_data;
	struct fu_async_data    *async_data;
	//struct sk_buff          *outgoing_skb;
	u16                     status, test, xbuf, *ptr2;
#ifdef __LITTLE_ENDIAN
	u16                     *ptr1, i;
#endif
	int                     ret;

	switch (xfr_state) {
	case 0:
		status = *((u16 *)transfer.rx_buf + 2);
#ifdef __LITTLE_ENDIAN
		status = (status << 8) | (status >> 8);
#endif
		test = status & (dd->irq_param[5] | dd->irq_param[6]);
		if (test == 0) {
			xfr_state = 2;
			xfr_complete(dd);
			return;
		} else if (test == (dd->irq_param[5] | dd->irq_param[6]))
			xbuf = ((status & dd->irq_param[4]) == 0) ? 0 : 1;
		else if (test == dd->irq_param[5])
			xbuf = 0;
		else if (test == dd->irq_param[6])
			xbuf = 1;
		else {
			ERROR("unexpected IRQ handler case");
			xfr_state = 2;
			xfr_complete(dd);
			return;
		}
		b_address = xbuf ? dd->irq_param[2] : dd->irq_param[1];
		clr_status = 0xFFFF;//(xbuf ? dd->irq_param[6] : dd->irq_param[5]) | 0x40;
		async = true;
		ret = chip_read(dd, b_address, 0, dd->irq_param[3]);
		if (ret < 0) {
			ERROR("can't read IRQ buffer (%d)", ret);
			xfr_state = 2;
			xfr_complete(dd);
			return;
		}
		async = false;
		xfr_state++;
		return;
	case 1:
#if 0
		outgoing_skb = alloc_skb(NL_BUF_SIZE, GFP_ATOMIC);
		if (outgoing_skb == NULL) {
			ERROR("can't allocate skb");
			xfr_state = 2;
			xfr_complete(dd);
			return;
		}
#else
	if (!dd->syscall_rx2) {
#endif
		nl_msg_init(/*outgoing_skb->data*/dd->syscall_rx_buf2, dd->nl_family.id, dd->nl_seq++,
			    MC_FUSION);

		if (dd->nl_seq == 0)
			dd->nl_seq++;
		async_data = nl_alloc_attr(/*outgoing_skb->data*/dd->syscall_rx_buf2, FU_ASYNC_DATA,
					   sizeof(*async_data) + dd->irq_param[3]);
		if (async_data == NULL) {
			ERROR("can't add data to async IRQ buffer");
			xfr_state++;
			xfr_complete(dd);
			return;
		}
		async_data->address = b_address;
		async_data->length = dd->irq_param[3];
		ptr2 = (u16 *)transfer.rx_buf + 2;
#ifdef __LITTLE_ENDIAN
		ptr1 = (u16 *)async_data->data;
		for (i = 0; i < (dd->irq_param[3] / sizeof(u16)); i++)
			ptr1[i] = (ptr2[i] << 8) | (ptr2[i] >> 8);
#else
		memcpy(async_data->data, ptr2, dd->irq_param[3]);
#endif
		ptr2 = (u16 *)async_data->data;
#if 0
		(void)skb_put(outgoing_skb,
			      NL_SIZE(outgoing_skb->data));
#else
		dd->syscall_rx2 = true;
		if (dd->syscall_reader)
			wake_up_process(dd->syscall_reader);
	}
#endif
#if 0
#if 1
		ret = genlmsg_multicast(outgoing_skb, 0,
					dd->nl_mc_groups[MC_FUSION].id,
					GFP_ATOMIC);
#else
		skb_queue_tail(&dd->incoming_skb_queue, outgoing_skb);
		wake_up_process(dd->thread);
#endif
#endif
////		if (ret < 0) {
////			ERROR("can't send IRQ buffer %d", ret);
////		}
//		ret = nl_msg_new(dd, MC_FUSION);
//		if (ret < 0)
//			ERROR("could not allocate outgoing skb (%d)", ret);
		async = true;
		ret = chip_write(dd, dd->irq_param[0], (u8 *)&clr_status,
				 sizeof(clr_status));
//		if (ret < 0)
//			ERROR("can't clear IRQ status (%d)", ret);
		async = false;
		xfr_state++;
		return;
	case 2:
		if (dd->coherency_completion) {
			xfr_state = 3;
			complete(dd->coherency_completion);
		}
		else if (pdata->irq(pdata) == 0) {
			xfr_state = 4;
			(void)irq_handler(0, dd);
		} else {
			xfr_state = 3;
		}
		return;
	}
}

#if 0
static void service_irq(struct dev_data *dd)
{
	struct fu_async_data  *async_data;
	u16                   status, test, address, xbuf;
	int                   ret, ret2;

#if CPU_BOOST
	pm_qos_update_request_timeout(&dd->cpus_req, 1, 10000);
	pm_qos_update_request_timeout(&dd->freq_req, dd->boost_freq, 10000);
#endif

	ret = dd->chip.read(dd, dd->irq_param[0], (u8 *)&status,
			    sizeof(status));
	if (ret < 0) {
		ERROR("can't read IRQ status (%d)", ret);
		return;
	}

	test = status & (dd->irq_param[5] | dd->irq_param[6]);
	if (test == 0)
		return;
	else if (test == (dd->irq_param[5] | dd->irq_param[6]))
		xbuf = ((status & dd->irq_param[4]) == 0) ? 0 : 1;
	else if (test == dd->irq_param[5])
		xbuf = 0;
	else if (test == dd->irq_param[6])
		xbuf = 1;
	else {
		ERROR("unexpected IRQ handler case");
		return;
	}
	address = xbuf ? dd->irq_param[2] : dd->irq_param[1];
	status = xbuf ? dd->irq_param[6] : dd->irq_param[5];

	async_data = nl_alloc_attr(dd->outgoing_skb->data, FU_ASYNC_DATA,
				   sizeof(*async_data) + dd->irq_param[3]);
	if (async_data == NULL) {
		ERROR("can't add data to async IRQ buffer");
		return;
	}
	async_data->address = address;
	async_data->length = dd->irq_param[3];
	ret = dd->chip.read(dd, address, async_data->data, dd->irq_param[3]);

	ret2 = dd->chip.write(dd, dd->irq_param[0], (u8 *)&status,
			     sizeof(status));
	if (ret2 < 0)
		ERROR("can't clear IRQ status (%d)", ret2);

	if (ret < 0) {
		ERROR("can't read IRQ buffer (%d)", ret);
	} else {
		(void)skb_put(dd->outgoing_skb,
			      NL_SIZE(dd->outgoing_skb->data));
		ret = genlmsg_multicast(dd->outgoing_skb, 0,
					dd->nl_mc_groups[MC_FUSION].id,
					GFP_KERNEL);
		if (ret < 0) {
			ERROR("can't send IRQ buffer %d", ret);
			msleep(300);
		}
		ret = nl_msg_new(dd, MC_FUSION);
		if (ret < 0)
			ERROR("could not allocate outgoing skb (%d)", ret);
	}
}
#endif

/****************************************************************************\
* Processing thread                                                          *
\****************************************************************************/

static int processing_thread(void *arg)
{
	struct dev_data         *dd = arg;
	struct maxim_sti_pdata  *pdata = dd->spi->dev.platform_data;
//	struct sk_buff          *skb;
	char                    *argv[] = { pdata->touch_fusion, "daemon",
					    pdata->nl_family,
					    pdata->config_file, NULL };
	int                     ret;

	sched_setscheduler(current, SCHED_FIFO, &dd->thread_sched);

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		/* ensure that we have outgoing skb */
		if (dd->outgoing_skb == NULL) {
			if (nl_msg_new(dd, MC_FUSION) < 0) {
				msleep(100);
				continue;
			}
}

		/* priority 1: start up fusion process */
		if (dd->start_fusion) {
			do {
				ret = call_usermodehelper(argv[0], argv, NULL,
							  UMH_WAIT_EXEC);
				if (ret != 0)
					msleep(100);
			} while (ret != 0 && !kthread_should_stop());
			dd->start_fusion = false;
		}
		if (kthread_should_stop())
			break;

#if 0
		/* priority 2: process pending Netlink messages */
		while ((skb = skb_dequeue(&dd->incoming_skb_queue)) != NULL) {
			if (kthread_should_stop())
				break;
			if (nl_process_msg(dd, skb) < 0)
				skb_queue_purge(&dd->incoming_skb_queue);
		}
		if (kthread_should_stop())
			break;
#endif
#if 0
		while ((skb = skb_dequeue(&dd->incoming_skb_queue)) != NULL) {
			if (kthread_should_stop())
				break;
		(void)genlmsg_multicast(skb, 0,
					dd->nl_mc_groups[MC_FUSION].id,
					GFP_KERNEL);
		}
		if (kthread_should_stop())
			break;
#endif

		/* priority 3: suspend/resume */
		if (dd->suspend_in_progress) {
			coherency_lock(dd);
			dd->syscall_rx2 = false;
			stop_scan_canned(dd);
			complete(&dd->suspend_resume);
			while (!dd->resume_in_progress) {
				if (kthread_should_stop())
					break;
				/* the line below is a MUST */
				set_current_state(TASK_INTERRUPTIBLE);
				schedule();
			}
			if (kthread_should_stop())
				break;
			start_scan_canned(dd);
			dd->resume_in_progress = false;
			dd->suspend_in_progress = false;
			complete(&dd->suspend_resume);

			nl_msg_init(dd->syscall_rx_buf1, dd->nl_family.id, dd->nl_seq++,
				    MC_FUSION);
			ret = nl_add_attr(dd->syscall_rx_buf1, FU_RESUME,
					  NULL, 0);
			if (ret < 0)
				ERROR("can't add data to resume buffer");
#if 0
			(void)skb_put(dd->outgoing_skb,
				      NL_SIZE(dd->outgoing_skb->data));
			ret = genlmsg_multicast(dd->outgoing_skb, 0,
					dd->nl_mc_groups[MC_FUSION].id,
					GFP_KERNEL);
			if (ret < 0)
				ERROR("can't send resume message %d", ret);
			ret = nl_msg_new(dd, MC_FUSION);
			if (ret < 0)
				ERROR("could not allocate outgoing skb (%d)",
				      ret);
#else
			dd->syscall_rx1 = true;
			if (dd->syscall_reader)
				wake_up_process(dd->syscall_reader);
#endif
			coherency_unlock(dd);
		}
		if (kthread_should_stop())
			break;

#if 0
		/* priority 4: service interrupt */
		if (dd->irq_registered && pdata->irq(pdata) == 0)
			service_irq(dd);
		if (kthread_should_stop())
			break;
		if (dd->irq_registered && pdata->irq(pdata) == 0)
			continue;
#endif

		/* nothing more to do; sleep */
		schedule();
	}

	return 0;
}

/****************************************************************************\
* Driver initialization                                                      *
\****************************************************************************/

static int probe(struct spi_device *spi)
{
	struct maxim_sti_pdata  *pdata = spi->dev.platform_data;
	struct dev_data         *dd;
	unsigned long           flags;
	int                     ret, i;
	void                    *ptr;

	/* validate platform data */
	if (pdata == NULL || pdata->init == NULL || pdata->reset == NULL ||
		pdata->irq == NULL || pdata->touch_fusion == NULL ||
		pdata->config_file == NULL || pdata->nl_family == NULL ||
		GENL_CHK(pdata->nl_family) ||
		pdata->nl_mc_groups < MC_REQUIRED_GROUPS ||
		pdata->default_reset_state > 1)
			return -EINVAL;

	/* device context: allocate structure */
	dd = kzalloc(sizeof(*dd) + pdata->tx_buf_size + pdata->rx_buf_size +
		     sizeof(*dd->nl_ops) * pdata->nl_mc_groups +
		     sizeof(*dd->nl_mc_groups) * pdata->nl_mc_groups,
		     GFP_KERNEL);
	if (dd == NULL)
		return -ENOMEM;

	/* device context: set up dynamic allocation pointers */
	ptr = (void *)dd + sizeof(*dd);
	if (pdata->tx_buf_size > 0) {
		dd->tx_buf = ptr;
		ptr += pdata->tx_buf_size;
	}
	if (pdata->rx_buf_size > 0) {
		dd->rx_buf = ptr;
		ptr += pdata->rx_buf_size;
	}
	dd->nl_ops = ptr;
	ptr += sizeof(*dd->nl_ops) * pdata->nl_mc_groups;
	dd->nl_mc_groups = ptr;

	/* device context: initialize structure members */
	spi_set_drvdata(spi, dd);
	dd->spi = spi;
	dd->nl_seq = 1;
	init_completion(&dd->suspend_resume);
	memset(dd->tx_buf, 0xFF, sizeof(dd->tx_buf));
	spin_lock_init(&dd->coherency_lock);
	mutex_init(&dd->coherency_mutex);

	/* initialize platform */
	ret = pdata->init(pdata, true);
	if (ret < 0)
		goto platform_failure;
	(void)pdata->power(1);

	/* start processing thread */
	dd->thread_sched.sched_priority = MAX_USER_RT_PRIO / 2;
	dd->thread = kthread_run(processing_thread, dd, pdata->nl_family);
	if (IS_ERR(dd->thread)) {
		ret = PTR_ERR(dd->thread);
		goto platform_failure;
	}

	/* Netlink: register GENL family */
	dd->nl_family.id      = GENL_ID_GENERATE;
	dd->nl_family.version = NL_FAMILY_VERSION;
	GENL_COPY(dd->nl_family.name, pdata->nl_family);
	ret = genl_register_family(&dd->nl_family);
	if (ret < 0)
		goto nl_family_failure;

	/* Netlink: register family ops */
	for (i = 0; i < MC_REQUIRED_GROUPS; i++) {
		dd->nl_ops[i].cmd = i;
		dd->nl_ops[i].doit = nl_callback_noop;
	}
	dd->nl_ops[MC_DRIVER].doit = nl_callback_driver;
	dd->nl_ops[MC_FUSION].doit = nl_callback_fusion;
	for (i = 0; i < MC_REQUIRED_GROUPS; i++) {
		ret = genl_register_ops(&dd->nl_family, &dd->nl_ops[i]);
		if (ret < 0)
			goto nl_failure;
	}

	/* Netlink: register family multicast groups */
	GENL_COPY(dd->nl_mc_groups[MC_DRIVER].name, MC_DRIVER_NAME);
	GENL_COPY(dd->nl_mc_groups[MC_FUSION].name, MC_FUSION_NAME);
	for (i = 0; i < MC_REQUIRED_GROUPS; i++) {
		ret = genl_register_mc_group(&dd->nl_family,
					     &dd->nl_mc_groups[i]);
		if (ret < 0)
			goto nl_failure;
	}
	dd->nl_mc_group_count = MC_REQUIRED_GROUPS;

	/* Netlink: pre-allocate outgoing skb */
	ret = nl_msg_new(dd, MC_FUSION);
	if (ret < 0)
		goto nl_failure;

#if CPU_BOOST
	/* initialize PM QOS */
	dd->boost_freq = pm_qos_request(PM_QOS_CPU_FREQ_MAX);
	pm_qos_add_request(&dd->cpus_req, PM_QOS_MIN_ONLINE_CPUS,
			   PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&dd->freq_req, PM_QOS_CPU_FREQ_MIN,
			   PM_QOS_DEFAULT_VALUE);
#endif

#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_HAS_EARLYSUSPEND)
	/* register early suspend */
	dd->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	dd->early_suspend.suspend = early_suspend;
	dd->early_suspend.resume = late_resume;
	register_early_suspend(&dd->early_suspend);
#endif

	/* Netlink: initialize incoming skb queue */
	skb_queue_head_init(&dd->incoming_skb_queue);

	/* Netlink: ready to start processing incoming messages */
	dd->nl_enabled = true;
	syscall_dd = dd;
	maxim_fusion = maxim_fusion_callback;

	/* add us to the devices list */
	spin_lock_irqsave(&dev_lock, flags);
	list_add_tail(&dd->dev_list, &dev_list);
	spin_unlock_irqrestore(&dev_lock, flags);

	/* start up Touch Fusion */
	dd->start_fusion = true;
	wake_up_process(dd->thread);
	INFO("driver loaded; version %s; release date %s", DRIVER_VERSION,
	     DRIVER_RELEASE);

	return 0;

nl_failure:
	genl_unregister_family(&dd->nl_family);
nl_family_failure:
	(void)kthread_stop(dd->thread);
platform_failure:
	pdata->init(pdata, false);
	kfree(dd);
	return ret;
}

static int remove(struct spi_device *spi)
{
	struct maxim_sti_pdata  *pdata = spi->dev.platform_data;
	struct dev_data         *dd = spi_get_drvdata(spi);
	unsigned long           flags;

	coherency_lock(dd);

#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_HAS_EARLYSUSPEND)
	/* unregister early suspend before processing thread is terminated */
	unregister_early_suspend(&dd->early_suspend);
#endif

	/* BEWARE: tear-down sequence below is carefully staged:            */
	/* 1) first the feeder of Netlink messages to the processing thread */
	/*    is turned off                                                 */
	/* 2) then the thread itself is shut down                           */
	/* 3) then Netlink family is torn down since no one would be using  */
	/*    it at this point                                              */
	/* 4) above step (3) insures that all Netlink senders are           */
	/*    definitely gone and it is safe to free up outgoing skb buffer */
	/*    and incoming skb queue                                        */
	dd->nl_enabled = false;
	maxim_fusion = NULL;
	if (dd->syscall_reader)
		wake_up_process(dd->syscall_reader);
	msleep(20);
	(void)kthread_stop(dd->thread);
	genl_unregister_family(&dd->nl_family);
	kfree_skb(dd->outgoing_skb);
	skb_queue_purge(&dd->incoming_skb_queue);

	if (dd->input_dev) {
		input_unregister_device(dd->input_dev);
	}

	if (dd->irq_registered)
		free_irq(dd->spi->irq, dd);

#if CPU_BOOST
	if (dd->boost_freq != 0) {
		pm_qos_remove_request(&dd->freq_req);
		pm_qos_remove_request(&dd->cpus_req);
	}
#endif

	stop_scan_canned(dd);

	spin_lock_irqsave(&dev_lock, flags);
	list_del(&dd->dev_list);
	spin_unlock_irqrestore(&dev_lock, flags);

	kfree(dd);

	pdata->init(pdata, false);
	(void)pdata->power(0);
	INFO("driver unloaded");
	return 0;
}

/****************************************************************************\
* Module initialization                                                      *
\****************************************************************************/

static const struct spi_device_id id[] = {
	{ MAXIM_STI_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(spi, id);

static struct spi_driver driver = {
	.probe          = probe,
	.remove         = remove,
	.id_table       = id,
	.driver = {
		.name   = MAXIM_STI_NAME,
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm     = &pm_ops,
#endif
	},
};

static int __devinit maxim_sti_init(void)
{
	INIT_LIST_HEAD(&dev_list);
	spin_lock_init(&dev_lock);
	return spi_register_driver(&driver);
}

static void __exit maxim_sti_exit(void)
{
	spi_unregister_driver(&driver);
}

module_init(maxim_sti_init);
module_exit(maxim_sti_exit);

MODULE_AUTHOR("Maxim Integrated Products, Inc.");
MODULE_DESCRIPTION("Maxim SmartTouch Imager Touchscreen Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

