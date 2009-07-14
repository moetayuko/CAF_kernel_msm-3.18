/* arch/arm/mach-msm/qdsp6/pcm_out.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include <linux/msm_audio.h>

#include <mach/msm_qdsp6_audio.h>

//#define BUFSZ (4096)
#define BUFSZ (8192)
#define DMASZ (BUFSZ * 2)

static long q6_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct audio_client *ac = file->private_data;
	int rc = 0;

	if (cmd == AUDIO_GET_STATS) {
		struct msm_audio_stats stats;
		memset(&stats, 0, sizeof(stats));
		if (copy_to_user((void*) arg, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;
	}
	if (cmd == AUDIO_SET_VOLUME) {
		return 0;
	}

	switch (cmd) {
	case AUDIO_START:
		printk("AUDIO_START\n");
		break;
	case AUDIO_STOP:
		printk("AUDIO_STOP\n");
		break;
	case AUDIO_FLUSH:
		printk("AUDIO_FLUSH\n");
		break;
	case AUDIO_SET_CONFIG: {
		struct msm_audio_config config;
		if (copy_from_user(&config, (void*) arg, sizeof(config))) {
			rc = -EFAULT;
			break;
		}
		printk("AUDIO_SET_CONFIG sr=%d ch=%d\n",
		       config.sample_rate, config.channel_count);
		break;
	}
	case AUDIO_GET_CONFIG: {
		struct msm_audio_config config;
		config.buffer_size = BUFSZ;
		config.buffer_count = 2;
		config.sample_rate = 44100;
		config.channel_count = 2;
		config.unused[0] = 0;
		config.unused[1] = 0;
		config.unused[2] = 0;
		printk("AUDIO_GET_CONFIG\n");
		if (copy_to_user((void*) arg, &config, sizeof(config))) {
			rc = -EFAULT;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	return rc;
}

static int q6_open(struct inode *inode, struct file *file)
{
	file->private_data = q6audio_open_pcm(BUFSZ, 44100, 2);
	if (!file->private_data)
		return -ENOMEM;

	return 0;
}

static ssize_t q6_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	struct audio_client *ac = file->private_data;
	struct audio_buffer *ab;
	const char __user *start = buf;
	int xfer;

	while (count > 0) {
		ab = ac->buf + ac->write_buf;

		if (ab->used)
			wait_event(ac->wait, (ab->used == 0));

		xfer = count;
		if (xfer > ab->size)
			xfer = ab->size;

		if (copy_from_user(ab->data, buf, xfer)) 
			return -EFAULT;

		buf += xfer;
		count -= xfer;

		ab->used = xfer;
		q6audio_write(ac, ab);
		ac->write_buf ^= 1;
	}

	return buf - start;
}

static int q6_release(struct inode *inode, struct file *file)
{
	return q6audio_close(file->private_data);
}

static struct file_operations q6_fops = {
	.owner		= THIS_MODULE,
	.open		= q6_open,
	.write		= q6_write,
	.release	= q6_release,
	.unlocked_ioctl	= q6_ioctl,
};

struct miscdevice q6_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_pcm_out",
	.fops	= &q6_fops,
};

/* legacy audpp interface -- just until userspace no longer needs it */
static long audpp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	printk("AUDPP_IOCTL %x\n", cmd);
	return 0;
}

static int audpp_open(struct inode *inode, struct file *file)
{
	printk("AUDPP_OPEN\n");
	return 0;
}

static struct file_operations audpp_fops = {
	.owner		= THIS_MODULE,
	.open		= audpp_open,
	.unlocked_ioctl	= audpp_ioctl,
};

struct miscdevice audpp_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_pcm_ctl",
	.fops	= &audpp_fops,
};


static int __init q6_init(void) {
	misc_register(&audpp_misc);
	return misc_register(&q6_misc);
}

device_initcall(q6_init);
