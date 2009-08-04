/* arch/arm/mach-msm/qdsp6/audio_ctrl.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation
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

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/msm_audio.h>

#include <mach/msm_qdsp6_audio.h>

static int q6_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int q6_ioctl(struct inode *inode, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	int rc;
	uint32_t dev, mute, acdb_id;

	switch (cmd) {
	case AUDIO_SWITCH_DEVICE:
		rc = copy_from_user(&dev, (void *)arg, sizeof(dev));
		if (!rc)
			rc = q6audio_do_routing(dev);
		break;
	case AUDIO_SET_MUTE:
		rc = copy_from_user(&mute, (void *)arg, sizeof(mute));
		if (!rc)
			rc = q6audio_set_tx_mute(mute);
		break;
	case AUDIO_UPDATE_ACDB:
		rc = copy_from_user(&acdb_id, (void *)arg, sizeof(acdb_id));
		if (!rc)
			rc = q6audio_update_acdb(acdb_id);
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}


static int q6_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations q6_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= q6_open,
	.ioctl		= q6_ioctl,
	.release	= q6_release,
};

static struct miscdevice q6_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_audio_ctl",
	.fops	= &q6_dev_fops,
};


static int __init q6_audio_ctl_init(void) {
	return misc_register(&q6_misc);
}

device_initcall(q6_audio_ctl_init);
