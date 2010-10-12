/* arch/arm/mach-msm/clock.c
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <linux/pm_qos_params.h>
#include <linux/device.h>
#include <linux/seq_file.h>
#include <mach/clk.h>

#include <asm/clkdev.h>

#include "clock.h"
#include "proc_comm.h"
#include "clock-7x30.h"
#include "clock-pcom.h"

static DEFINE_MUTEX(clocks_mutex);
static HLIST_HEAD(clocks);

int clk_enable(struct clk *clk)
{
	return clk->ops->enable(clk->id);
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	clk->ops->disable(clk->id);
}
EXPORT_SYMBOL(clk_disable);

int clk_reset(struct clk *clk, enum clk_reset_action action)
{
	if (!clk->ops->reset)
		clk->ops->reset = &pc_clk_reset;
	return clk->ops->reset(clk->remote_id, action);
}
EXPORT_SYMBOL(clk_reset);

unsigned long clk_get_rate(struct clk *clk)
{
	return clk->ops->get_rate(clk->id);
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret;

	if (clk->flags & CLKFLAG_MAX) {
		ret = clk->ops->set_max_rate(clk->id, rate);
		if (ret)
			return ret;
	}
	if (clk->flags & CLKFLAG_MIN) {
		ret = clk->ops->set_min_rate(clk->id, rate);
		if (ret)
			return ret;
	}

	if (!(clk->flags & (CLKFLAG_MAX | CLKFLAG_MIN)))
		ret = clk->ops->set_rate(clk->id, rate);
	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->round_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_min_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->set_min_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_min_rate);

int clk_set_max_rate(struct clk *clk, unsigned long rate)
{
	return clk->ops->set_max_rate(clk->id, rate);
}
EXPORT_SYMBOL(clk_set_max_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	if (clk->ops->set_parent)
		return clk->ops->set_parent(clk->id, parent);
	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;
	return clk->ops->set_flags(clk->id, flags);
}
EXPORT_SYMBOL(clk_set_flags);

void clk_enter_sleep(int from_idle)
{
}

void clk_exit_sleep(void)
{
}

static void __init set_clock_ops(struct clk *clk)
{
	if (!clk->ops) {
		clk->ops = &clk_ops_pcom;
		clk->id = clk->remote_id;
	}
}

void __init msm_clock_init(struct clk_lookup *clock_tbl, unsigned num_clocks)
{
	unsigned n;
	struct clk *clk;

	mutex_lock(&clocks_mutex);

	for (n = 0; n < num_clocks; n++) {
		set_clock_ops(clock_tbl[n].clk);
		clkdev_add(&clock_tbl[n]);
		hlist_add_head(&clock_tbl[n].clk->list, &clocks);
	}
	mutex_unlock(&clocks_mutex);

	for (n = 0; n < num_clocks; n++) {
		clk = clock_tbl[n].clk;
		if (clk->flags & CLKFLAG_VOTER) {
			struct clk *agg_clk = clk_get(NULL, clk->aggregator);
			BUG_ON(IS_ERR(agg_clk));

			clk_set_parent(clk, agg_clk);
		}
	}
}

#if defined(CONFIG_DEBUG_FS)
static int clock_debug_rate_set(void *data, u64 val)
{
	struct clk *clock = data;
	int ret;

	/* Only increases to max rate will succeed, but that's actually good
	 * for debugging purposes. So we don't check for error. */
	if (clock->flags & CLK_MAX)
		clk_set_max_rate(clock, val);
	if (clock->flags & CLK_MIN)
		ret = clk_set_min_rate(clock, val);
	else
		ret = clk_set_rate(clock, val);
	if (ret != 0)
		printk(KERN_ERR "clk_set%s_rate failed (%d)\n",
			(clock->flags & CLK_MIN) ? "_min" : "", ret);
	return ret;
}

static int clock_debug_rate_get(void *data, u64 *val)
{
	struct clk *clock = data;
	*val = clk_get_rate(clock);
	return 0;
}

static int clock_debug_enable_set(void *data, u64 val)
{
	struct clk *clock = data;
	int rc = 0;

	if (val)
		rc = clock->ops->enable(clock->id);
	else
		clock->ops->disable(clock->id);

	return rc;
}

static int clock_debug_enable_get(void *data, u64 *val)
{
	struct clk *clock = data;

	*val = clock->ops->is_enabled(clock->id);

	return 0;
}

static int clock_debug_local_get(void *data, u64 *val)
{
	struct clk *clock = data;

	*val = clock->ops != &clk_ops_pcom;

	return 0;
}

static void *clk_info_seq_start(struct seq_file *seq, loff_t *ppos)
{
	struct hlist_node *pos;
	int i = *ppos;
	mutex_lock(&clocks_mutex);
	hlist_for_each(pos, &clocks)
		if (i-- == 0)
			return hlist_entry(pos, struct clk, list);
	return NULL;
}

static void *clk_info_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct clk *clk = v;
	++*pos;
	return hlist_entry(clk->list.next, struct clk, list);
}

static void clk_info_seq_stop(struct seq_file *seq, void *v)
{
	mutex_unlock(&clocks_mutex);
}

static int clk_info_seq_show(struct seq_file *seq, void *v)
{
	struct clk *clk = v;

	seq_printf(seq, "Clock %s\n", clk->dbg_name);
	seq_printf(seq, "  Id          %d\n", clk->id);
	seq_printf(seq, "  Flags       %x\n", clk->flags);
	seq_printf(seq, "  Enabled     %d\n", clk->ops->is_enabled(clk->id));
	seq_printf(seq, "  Rate        %ld\n", clk_get_rate(clk));

	seq_printf(seq, "\n");
	return 0;
}

static struct seq_operations clk_info_seqops = {
	.start = clk_info_seq_start,
	.next = clk_info_seq_next,
	.stop = clk_info_seq_stop,
	.show = clk_info_seq_show,
};

static int clk_info_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &clk_info_seqops);
}

static const struct file_operations clk_info_fops = {
	.owner = THIS_MODULE,
	.open = clk_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

DEFINE_SIMPLE_ATTRIBUTE(clock_rate_fops, clock_debug_rate_get,
			clock_debug_rate_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(clock_enable_fops, clock_debug_enable_get,
			clock_debug_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(clock_local_fops, clock_debug_local_get,
			NULL, "%llu\n");

static int __init clock_debug_init(void)
{
	struct dentry *dent_rate, *dent_enable, *dent_local;
	struct clk *clock;
	struct hlist_node *pos;
	char temp[50], *ptr;

	dent_rate = debugfs_create_dir("clk_rate", 0);
	if (IS_ERR(dent_rate))
		return PTR_ERR(dent_rate);

	dent_enable = debugfs_create_dir("clk_enable", 0);
	if (IS_ERR(dent_enable))
		return PTR_ERR(dent_enable);

	dent_local = debugfs_create_dir("clk_local", NULL);
	if (IS_ERR(dent_local))
		return PTR_ERR(dent_local);

	debugfs_create_file("clk_info", 0x444, 0, NULL, &clk_info_fops);

	mutex_lock(&clocks_mutex);
	hlist_for_each_entry(clock, pos, &clocks, list) {
		strncpy(temp, clock->dbg_name, ARRAY_SIZE(temp)-1);
		for (ptr = temp; *ptr; ptr++)
			*ptr = tolower(*ptr);
		debugfs_create_file(temp, 0644, dent_rate,
				    clock, &clock_rate_fops);
		debugfs_create_file(temp, 0644, dent_enable,
				    clock, &clock_enable_fops);
		debugfs_create_file(temp, S_IRUGO, dent_local,
				    clock, &clock_local_fops);
	}
	mutex_unlock(&clocks_mutex);
	return 0;
}

late_initcall(clock_debug_init);
#endif

/* The bootloader and/or AMSS may have left various clocks enabled.
 * Disable any clocks that belong to us (CLKFLAG_AUTO_OFF) but have
 * not been explicitly enabled by a clk_enable() call.
 */
static int __init clock_late_init(void)
{
	struct clk *clk;
	struct hlist_node *pos;

	mutex_lock(&clocks_mutex);
	hlist_for_each_entry(clk, pos, &clocks, list)
		if (clk->flags & CLKFLAG_AUTO_OFF)
			clk->ops->auto_off(clk->id);
	mutex_unlock(&clocks_mutex);
	return 0;
}
late_initcall(clock_late_init);
