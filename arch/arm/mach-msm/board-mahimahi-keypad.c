/* arch/arm/mach-msm/board-mahimahi-keypad.c
 *
 * Copyright (C) 2009 Google, Inc
 * Copyright (C) 2009 HTC Corporation.
 *
 * Author: Dima Zavin <dima@android.com>
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
#include <mach/vreg.h>

#include <asm/mach-types.h>

#include "board-mahimahi.h"

struct jog_axis_info {
	struct gpio_event_axis_info	info;
	uint16_t			in_state;
	uint16_t			out_state;
};

static struct vreg *jog_vreg;
static bool jog_just_on;
static unsigned long jog_on_jiffies;

static unsigned int mahimahi_col_gpios[] = { 33, 32, 31 };
static unsigned int mahimahi_row_gpios[] = { 42, 41, 40 };

#define KEYMAP_INDEX(col, row)	((col)*ARRAY_SIZE(mahimahi_row_gpios) + (row))
#define KEYMAP_SIZE		(ARRAY_SIZE(mahimahi_col_gpios) * \
				 ARRAY_SIZE(mahimahi_row_gpios))

/* keypad */
static const unsigned short mahimahi_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 1)] = BTN_MOUSE,
};

static struct gpio_event_matrix_info mahimahi_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = mahimahi_keymap,
	.output_gpios = mahimahi_col_gpios,
	.input_gpios = mahimahi_row_gpios,
	.noutputs = ARRAY_SIZE(mahimahi_col_gpios),
	.ninputs = ARRAY_SIZE(mahimahi_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		  GPIOKPF_REMOVE_PHANTOM_KEYS |
		  GPIOKPF_PRINT_UNMAPPED_KEYS),
};

static struct gpio_event_direct_entry mahimahi_keypad_key_map[] = {
	{
		.gpio	= MAHIMAHI_GPIO_POWER_KEY,
		.code	= KEY_END,
	},
};

static struct gpio_event_input_info mahimahi_keypad_key_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = mahimahi_keypad_key_map,
	.keymap_size = ARRAY_SIZE(mahimahi_keypad_key_map)
};

static struct gpio_event_info *mahimahi_keypad_info[] = {
	&mahimahi_keypad_matrix_info.info,
	&mahimahi_keypad_key_info.info,
};

static struct gpio_event_platform_data mahimahi_keypad_data = {
	.name = "mahimahi-keypad",
	.info = mahimahi_keypad_info,
	.info_count = ARRAY_SIZE(mahimahi_keypad_info)
};

static struct platform_device mahimahi_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &mahimahi_keypad_data,
	},
};

/* jogball */
static uint16_t jogball_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct jog_axis_info *ai =
		container_of(info, struct jog_axis_info, info);
	uint16_t out = ai->out_state;

	if (jog_just_on) {
		if (jiffies == jog_on_jiffies || jiffies == jog_on_jiffies + 1)
			goto ignore;
		jog_just_on = 0;
	}
	if((ai->in_state ^ in) & 1)
		out--;
	if((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	return out;
}

static int fake_event_func(struct input_dev *input_dev,
			   struct gpio_event_info *info, void **data, int func)
{
	struct gpio_event_input_info *di;
	int i;

	di = container_of(info, struct gpio_event_input_info, info);

	if (func != GPIO_EVENT_FUNC_INIT)
		return 0;

	for (i = 0; i < di->keymap_size; ++i)
		input_set_capability(input_dev, di->type, di->keymap[i].code);
	return 0;
}

static int jogball_power(const struct gpio_event_platform_data *pdata, bool on)
{
	if (on) {
		vreg_enable(jog_vreg);
		jog_just_on = 1;
		jog_on_jiffies = jiffies;
	} else {
		vreg_disable(jog_vreg);
	}

	return 0;
}

static uint32_t jogball_x_gpios[] = {
	MAHIMAHI_GPIO_BALL_LEFT, MAHIMAHI_GPIO_BALL_RIGHT,
};
static uint32_t jogball_y_gpios[] = {
	MAHIMAHI_GPIO_BALL_UP, MAHIMAHI_GPIO_BALL_DOWN,
};

static struct jog_axis_info jogball_x_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(jogball_x_gpios),
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(jogball_x_gpios),
		.map = jogball_axis_map,
		.gpio = jogball_x_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION,
	}
};

static struct jog_axis_info jogball_y_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(jogball_y_gpios),
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(jogball_y_gpios),
		.map = jogball_axis_map,
		.gpio = jogball_y_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION,
	}
};

static struct gpio_event_direct_entry fake_buttons[] = {
	{ 0, BTN_MOUSE },
};

static struct gpio_event_input_info jogball_button_info = {
	.info.func = fake_event_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = fake_buttons,
	.keymap_size = ARRAY_SIZE(fake_buttons)
};

static struct gpio_event_info *mahimahi_jogball_info[] = {
	&jogball_x_axis.info.info,
	&jogball_y_axis.info.info,
	&jogball_button_info.info,
};

static struct gpio_event_platform_data mahimahi_jogball_data = {
	.name = "mahimahi-nav",
	.info = mahimahi_jogball_info,
	.info_count = ARRAY_SIZE(mahimahi_jogball_info),
	.power = jogball_power,
};

static struct platform_device mahimahi_jogball_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 1,
	.dev = {
		.platform_data = &mahimahi_jogball_data,
	},
};

static int __init mahimahi_init_keypad_jogball(void)
{
	int ret;

	if (!machine_is_mahimahi())
		return 0;

	ret = platform_device_register(&mahimahi_keypad_device);
	if (ret != 0)
		return ret;

	jog_vreg = vreg_get(&mahimahi_jogball_device.dev, "gp2");
	if (jog_vreg == NULL)
		return -ENOENT;

	ret = platform_device_register(&mahimahi_jogball_device);
	if (ret != 0)
		return ret;

	return 0;
}

device_initcall(mahimahi_init_keypad_jogball);
