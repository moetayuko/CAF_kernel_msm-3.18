/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
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

/* Control bluetooth power for firestone platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include "board-firestone.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		gpio_configure(FIRESTONE_GPIO_BT_RESET_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
		gpio_configure(FIRESTONE_GPIO_BT_SHUTDOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		gpio_configure(FIRESTONE_GPIO_BT_SHUTDOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
		gpio_configure(FIRESTONE_GPIO_BT_RESET_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
		break;
	default:
		printk(KERN_ERR "bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int firestone_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	enum rfkill_state default_state = RFKILL_STATE_SOFT_BLOCKED;  /* off */

	rfkill_set_default(RFKILL_TYPE_BLUETOOTH, default_state);
	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt_rfk)
		return -ENOMEM;

	bt_rfk->name = bt_name;
	bt_rfk->state = default_state;
	/* userspace cannot take exclusive control */
	bt_rfk->user_claim_unsupported = 1;
	bt_rfk->user_claim = 0;
	bt_rfk->data = NULL;  // user data
	bt_rfk->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_free(bt_rfk);
	return rc;
}

static int firestone_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_free(bt_rfk);

	return 0;
}

static struct platform_driver firestone_rfkill_driver = {
	.probe = firestone_rfkill_probe,
	.remove = firestone_rfkill_remove,
	.driver = {
		.name = "firestone_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init firestone_rfkill_init(void)
{
	if (!machine_is_firestone())
		return 0;
	return platform_driver_register(&firestone_rfkill_driver);
}

static void __exit firestone_rfkill_exit(void)
{
	platform_driver_unregister(&firestone_rfkill_driver);
}

module_init(firestone_rfkill_init);
module_exit(firestone_rfkill_exit);
MODULE_DESCRIPTION("firestone rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
