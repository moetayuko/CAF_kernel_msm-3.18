/* drivers/w1/slaves/w1_ds2784.c
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Justin Lin <Justin_Lin@htc.com>
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
 * based on w1_ds2760.c which is:
 * Copyright (C) 2004-2005, Szabolcs Gyurko <szabolcs.gyurko@tlt.hu>
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_family.h"
#include "w1_ds2784.h"

static atomic_t owner = ATOMIC_INIT(0);

static int w1_ds2784_io(void *handle, char *buf, int addr, size_t count,
			int io)
{
	struct w1_slave *sl = (struct w1_slave *)handle;

	if (!sl)
		return 0;

	mutex_lock(&sl->master->mutex);

	if (addr > DS2784_DATA_SIZE || addr < 0) {
		count = 0;
		goto out;
	}
	if (addr + count > DS2784_DATA_SIZE)
		count = DS2784_DATA_SIZE - addr;

	if (!w1_reset_select_slave(sl)) {
		if (!io) {
			w1_write_8(sl->master, W1_DS2784_READ_DATA);
			w1_write_8(sl->master, addr);
			count = w1_read_block(sl->master, buf, count);
		} else {
			w1_write_8(sl->master, W1_DS2784_WRITE_DATA);
			w1_write_8(sl->master, addr);
			w1_write_block(sl->master, buf, count);
			/* XXX w1_write_block returns void, not n_written */
		}
	}

out:
	mutex_unlock(&sl->master->mutex);

	return count;
}

int w1_ds2784_read(void *handle, char *buf, int addr, size_t count)
{
	return w1_ds2784_io(handle, buf, addr, count, 0);
}
EXPORT_SYMBOL(w1_ds2784_read);

int w1_ds2784_write(void *handle, char *buf, int addr, size_t count)
{
	return w1_ds2784_io(handle, buf, addr, count, 1);
}
EXPORT_SYMBOL(w1_ds2784_write);

static ssize_t w1_ds2784_read_bin(struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	return w1_ds2784_read(dev, buf, off, count);
}

static struct bin_attribute w1_ds2784_bin_attr = {
	.attr = {
		.name = "w1_slave",
		.mode = S_IRUGO,
	},
	.size = DS2784_DATA_SIZE,
	.read = w1_ds2784_read_bin,
};

static int w1_ds2784_add_slave(struct w1_slave *sl)
{
	void (*cb)(void *);
	int ret = sysfs_create_bin_file(&sl->dev.kobj, &w1_ds2784_bin_attr);
	if (ret)
		return ret;

	cb = atomic_read(&owner);
	if (cb)
		cb(sl);
	return ret;
}

static void w1_ds2784_remove_slave(struct w1_slave *sl)
{
	sysfs_remove_bin_file(&sl->dev.kobj, &w1_ds2784_bin_attr);
}

static struct w1_family_ops w1_ds2784_fops = {
	.add_slave    = w1_ds2784_add_slave,
	.remove_slave = w1_ds2784_remove_slave,
};

static struct w1_family w1_ds2784_family = {
	.fid = W1_FAMILY_DS2784,
	.fops = &w1_ds2784_fops,
};

int w1_ds2784_init(void (*cb)(void *))
{
	if (atomic_cmpxchg(&owner, 0, cb)) {
		pr_err("%s: busy\n", __func__);
		return -EBUSY;
	}
	return w1_register_family(&w1_ds2784_family);
}
EXPORT_SYMBOL(w1_ds2784_init);

void w1_ds2784_exit(void)
{
	w1_unregister_family(&w1_ds2784_family);
	atomic_set(&owner, 0);
}
EXPORT_SYMBOL(w1_ds2784_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Justin Lin <Justin_Lin@htc.com>");
MODULE_DESCRIPTION("1-wire Driver Dallas 2784 battery monitor chip");
