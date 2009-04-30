/* arch/arm/mach-msm/board-firestone-keypad.c
 *
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Haley Teng <haley_teng@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio_event.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>

#include "board-firestone.h"

static unsigned int firestone_col_gpios[] = { 33, 32, 31 };
static unsigned int firestone_row_gpios[] = { 42, 41, 40 };

#define KEYMAP_INDEX(col, row)	((col)*ARRAY_SIZE(firestone_row_gpios) + (row))
#define KEYMAP_SIZE		(ARRAY_SIZE(firestone_col_gpios) * \
				 ARRAY_SIZE(firestone_row_gpios))

static const unsigned short firestone_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 1)] = KEY_HOME,

	[KEYMAP_INDEX(2, 0)] = KEY_BACK,
};

static struct gpio_event_matrix_info firestone_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = firestone_keymap,
	.output_gpios = firestone_col_gpios,
	.input_gpios = firestone_row_gpios,
	.noutputs = ARRAY_SIZE(firestone_col_gpios),
	.ninputs = ARRAY_SIZE(firestone_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		  GPIOKPF_REMOVE_PHANTOM_KEYS |
		  GPIOKPF_PRINT_UNMAPPED_KEYS),
};

static struct gpio_event_direct_entry firestone_keypad_nav_map[] = {
	{
		.gpio	= FIRESTONE_GPIO_POWER_KEY,
		.code	= KEY_END,
	},
};

static struct gpio_event_input_info firestone_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = firestone_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(firestone_keypad_nav_map)
};

static struct gpio_event_info *firestone_keypad_info[] = {
	&firestone_keypad_matrix_info.info,
	&firestone_keypad_nav_info.info,
};

static struct gpio_event_platform_data firestone_keypad_data = {
	.name = "firestone-keypad",
	.info = firestone_keypad_info,
	.info_count = ARRAY_SIZE(firestone_keypad_info)
};

static struct platform_device firestone_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &firestone_keypad_data,
	},
};

static int __init firestone_init_keypad(void)
{
	if (!machine_is_firestone())
		return 0;
	return platform_device_register(&firestone_keypad_device);
}

device_initcall(firestone_init_keypad);
