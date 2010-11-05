/* arch/arm/mach-msm/clock.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2010, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_H
#define __ARCH_ARM_MACH_MSM_CLOCK_H

#include <linux/list.h>
#include <mach/clk.h>

#define CLKFLAG_INVERT			0x00000001
#define CLKFLAG_NOINVERT		0x00000002
#define CLKFLAG_NONEST			0x00000004
#define CLKFLAG_NORESET			0x00000008
#define CLKFLAG_HANDLE			0x00000010

#define CLK_FIRST_AVAILABLE_FLAG	0x00000100
#define CLKFLAG_AUTO_OFF		0x00000200
#define CLKFLAG_MIN			0x00000400
#define CLKFLAG_MAX			0x00000800
#define CLKFLAG_SHARED			0x00001000

struct clk_ops {
	int (*enable)(unsigned id);
	void (*disable)(unsigned id);
	void (*auto_off)(unsigned id);
	int (*reset)(unsigned id, enum clk_reset_action action);
	int (*set_rate)(unsigned id, unsigned rate);
	int (*set_min_rate)(unsigned id, unsigned rate);
	int (*set_max_rate)(unsigned id, unsigned rate);
	int (*set_flags)(unsigned id, unsigned flags);
	unsigned (*get_rate)(unsigned id);
	unsigned (*is_enabled)(unsigned id);
	long (*round_rate)(unsigned id, unsigned rate);
	int (*set_parent)(unsigned id, struct clk *parent);
};

struct clk {
	uint32_t id;
	uint32_t remote_id;
	uint32_t flags;
	const char *name;
	struct clk_ops *ops;
	const char *dbg_name;
	struct hlist_node list;
	struct device *dev;
	struct hlist_head handles;
};

struct clk_handle {
	struct clk clk;
	struct clk *source;
	unsigned long rate;
};

#define OFF CLKFLAG_AUTO_OFF
#define CLK_MIN CLKFLAG_MIN
#define CLK_MAX CLKFLAG_MAX
#define CLK_MINMAX (CLK_MIN | CLK_MAX)

void clk_enter_sleep(int from_idle);
void clk_exit_sleep(void);

#endif
