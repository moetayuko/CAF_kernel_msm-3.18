/*
 *  assd.c - Advanced Security SD Extension
 *
 *  Copyright 2011 Giesecke & Devrient GmbH.  All rights reserved.
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 *  Detailed information about the Advanced Security SD Extension (ASSD) can
 *  be found in
 *  [A1] "SD Specifications Part A1  Advanced Security SD Extension Simplified
 *       Specification"
 *  provided by the SD Card Association.
 *  Detailed information about the SD can be found in
 *  [1] "SD Specifications Part 1  Physical Layer Simplified Specification"
 *
 */

#define VERSION	"110502f"
//#define DEBUG 1

#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/scatterlist.h>
extern int mmc_sd_switch(struct mmc_card *card, int mode, int group, u8 value,
			 u8 * resp);

#define CHECK_FUNCTION	0
#define SET_FUNCTION	1

// Currently we support ASSD 1.1 only. See [A1] chapter "Switching to the
// ASSD Mode" for more details.
#define ASSD_1_1	0x1
//#define ASSD_2_0	0x4

static u8 *assd_block = NULL;

#define ASSD_SEND_STOP	(1 << 0)

static unsigned long assd_status = 0;
static struct semaphore assd_lock;

/*
 * A Secure Token is composed of the Command or Response APDU prefixed
 * with a Secure Token Length field (2 bytes).
 * Currently we do not support Secure Tokens with a length exceeding the
 * defined block length (512 bytes). See [A1] chapter "Secure Tokens"
 * for more information.
 */
#define MAX_LENGTH	510

// Additional commands as defined in [A1] chapter "Commands".
#define ASSD_WRITE_SEC_CMD	35
#define ASSD_READ_SEC_CMD	34
#define ASSD_SEND_PSI		36

static int assd_timeout;

// todo: hopefully the host never is removed!
static struct mmc_host *assd_host = NULL;

#if !defined(DEBUG)
#define pr_debug_hexstr(buf, len)
#else

static void pr_debug_hexstr(char *buf, int len)
{
	int i;
	char s[2048];

	snprintf(s, sizeof(s), "assd: ");
	for (i = 0; i < len; i++)
		snprintf(s + strlen(s), sizeof(s) - strlen(s), "%.2x",
			 (unsigned char)buf[i]);
	snprintf(s + strlen(s), sizeof(s) - strlen(s), "\n");

	// Since printk() is limited to 1023 characters per call...
	printk(KERN_DEBUG "%s", s);
	if (strlen(s) >= 1023)
		printk(KERN_DEBUG "%s", s + 1023);
}

#endif // DEBUG

// All information needed to implement this function can be found in [1].
static int check_assd(struct mmc_host *host, int mode, int func)
{
	int ret = 0;
	struct mmc_card *card;

	BUG_ON(!host);

	pr_debug("assd: check_assd()\n");

	mmc_claim_host(host);
	card = host->card;
	if (card == NULL) {

		ret = -ENODEV;
		goto out;
	}
	// SD memory cards compatible with versions earlier than 1.10 do not
	// support the Switch function command.
	if (!card->scr.sda_vsn) {

		ret = -ENODEV;
		goto out;
	}

	// See [1] chapter "Switch Function Command" for information
	// about modes, groups, and functions.
	mmc_sd_switch(card, mode, /* (group 2) */ 1, func, assd_block);

	// We verify the response only if we switched to a non-default
	// function.
	if (func) {

		// See [1] chapter "Switch Function Status" for information
		// about the status bits. The function was successfully
		// switched if bits 383:380 hold the function number,
		// and in bits 431:416 the function group information show
		// that our function is supported.
		if (!(assd_block[11] & (1 << func))) {

			ret = -ENODEV;
			goto out;
		}

		if ((assd_block[16] & 0xf0) != (func << 4)) {

			ret = -ENODEV;
			if (assd_block[27] & (1 << func))
				ret = -EBUSY;
		}
	}
out:
	mmc_release_host(host);
	return ret;
}

// All information needed to implement this function can be found in [1].
static int enable_assd(struct mmc_host *host)
{
	for (;;) {

		int ret;

		// Check if the card was removed.
		if (assd_host == NULL) {

			printk("assd: failed to enable assd\n");
			return -ENODEV;
		}

		ret = check_assd(host, SET_FUNCTION, ASSD_1_1);
		if (!ret) {
			printk
			    ("assd: the assd capable card is ready for use\n");
			return 0;
		}

		if (ret != -EBUSY) {

			printk("assd: failed to enable assd\n");
			return -ENODEV;
		}

		set_task_state(current, TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 100);
	}
}

// All information needed to implement this function can be found in [1].
static int assd_set_blksize(struct mmc_host *host, int blksize)
{
	struct mmc_command cmd;
	int ret;

	BUG_ON(!host);

	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = blksize;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;

	mmc_claim_host(host);
	if (host->card == NULL) {

		mmc_release_host(host);
		return -ENODEV;
	}

	ret = mmc_wait_for_cmd(host, &cmd, 5);
	mmc_release_host(host);

	return ret;
}

// All information needed to implement this function can be found in [A1]
// chapters "Registers" and "Send PSI Command".
static int assd_send_psi(struct mmc_host *host)
{
	struct mmc_request req;
	struct mmc_command cmd;
	struct mmc_data dat;
	struct scatterlist sg;

	BUG_ON(!host);

	memset(&req, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&dat, 0, sizeof(struct mmc_data));

	req.cmd = &cmd;
	req.data = &dat;

	cmd.opcode = ASSD_SEND_PSI;
	cmd.arg = /* ASSD-SR */ 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	dat.blksz = 32;
	dat.blocks = 1;
	dat.flags = MMC_DATA_READ;
	dat.sg = &sg;
	dat.sg_len = 1;

	sg_init_one(&sg, assd_block, 32);

	mmc_claim_host(host);
	if (host->card == NULL) {

		mmc_release_host(host);
		return -ENODEV;
	}

	mmc_set_data_timeout(&dat, host->card);
	mmc_wait_for_req(host, &req);
	mmc_release_host(host);

	if (cmd.error)
		return cmd.error;

	if (dat.error)
		return dat.error;

	return 0;
}

// All information needed to implement this function can be found in [1], [A1].
// The C-APDU must be ready in assd_block.
static int assd_write_sec_cmd(struct mmc_host *host)
{
	struct mmc_request req;
	struct mmc_command cmd;
	struct mmc_command stp;
	struct mmc_data dat;
	struct scatterlist sg;

	BUG_ON(!host);

	memset(&req, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&stp, 0, sizeof(struct mmc_command));
	memset(&dat, 0, sizeof(struct mmc_data));

	req.cmd = &cmd;
	req.data = &dat;

	if (test_bit(ASSD_SEND_STOP, &assd_status))
		req.stop = &stp;

	cmd.opcode = ASSD_WRITE_SEC_CMD;
	cmd.arg = 1;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	dat.blksz = 512;
	dat.blocks = 1;
	dat.flags = MMC_DATA_WRITE;
	dat.sg = &sg;
	dat.sg_len = 1;

	sg_init_one(&sg, assd_block, 512);

	stp.opcode = MMC_STOP_TRANSMISSION;
	stp.arg = 0;
	stp.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

	mmc_claim_host(host);
	if (host->card == NULL) {

		mmc_release_host(host);
		return -ENODEV;
	}

	mmc_set_data_timeout(&dat, host->card);
	mmc_wait_for_req(host, &req);
	mmc_release_host(host);

	if (cmd.error)
		return cmd.error;

	if (dat.error)
		return dat.error;

	// Do not send any STOP_TRANSMISSION command from now on,
	// if this card does not require a STOP_TRANSMISSION command.
	if (stp.error == -ETIMEDOUT)
		clear_bit(ASSD_SEND_STOP, &assd_status);

	return 0;
}

// All information needed to implement this function can be found in [1], [A1].
static int assd_read_sec_cmd(struct mmc_host *host)
{
	struct mmc_request req;
	struct mmc_command cmd;
	struct mmc_command stp;
	struct mmc_data dat;
	struct scatterlist sg;

	BUG_ON(!host);

	memset(&req, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&stp, 0, sizeof(struct mmc_command));
	memset(&dat, 0, sizeof(struct mmc_data));

	req.cmd = &cmd;
	req.data = &dat;

	if (test_bit(ASSD_SEND_STOP, &assd_status))
		req.stop = &stp;

	cmd.opcode = ASSD_READ_SEC_CMD;
	cmd.arg = 1;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	dat.blksz = 512;
	dat.blocks = 1;
	dat.flags = MMC_DATA_READ;
	dat.sg = &sg;
	dat.sg_len = 1;

	sg_init_one(&sg, assd_block, 512);

	stp.opcode = MMC_STOP_TRANSMISSION;
	stp.arg = 0;
	stp.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

	mmc_claim_host(host);
	if (host->card == NULL) {

		mmc_release_host(host);
		return -ENODEV;
	}

	mmc_set_data_timeout(&dat, host->card);
	mmc_wait_for_req(host, &req);
	mmc_release_host(host);

	if (cmd.error)
		return cmd.error;

	if (dat.error)
		return dat.error;

	// The R-APDU is ready in assd_block now.
	return 0;
}

// All information needed to implement this function can be found in [A1].
static int assd_process(struct mmc_host *host, unsigned char *buf)
{
	unsigned int count;

	BUG_ON(assd_timeout <= 0);

	// Copy secure token (STL + C-APDU). See [A1] chapter "Secure Tokens".
        if (copy_from_user(assd_block, buf, 512))
                return -EFAULT;

        count = (buf[0] << 8) + buf[1] - 2;
        if (count > MAX_LENGTH)
                return -EINVAL;

	if (count < 4) {

		printk("assd: command apdu too short\n");
		return -EINVAL;
	}
	// todo: this output may be security ciritcal!
	pr_debug_hexstr(&assd_block[2], count);

	// todo: why do we need several attempts when sending the first time?!
	for (count = 20;; count--) {

		if (!count || (assd_host == NULL)) {

			printk("assd: send command failed\n");
			return -EIO;
		}

		if (!assd_write_sec_cmd(host))
			break;

		set_task_state(current, TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 100);
	}

	if (assd_set_blksize(host, 32)) {

		printk("assd: set blocklength (32) failed\n");
		// Some cards to not support SET_BLOCKLENGTH, but
		// accept SEND_PSI with a block length of 512.
		//return -EIO;
	}

	for (count = assd_timeout;; count--) {

		if (!count || (assd_host == NULL)) {

			printk("assd: get status failed\n");
			return -ETIMEDOUT;
		}

		if (!assd_send_psi(host)) {

			// Check if R-APDU is ready. See [A1] chapter
			// "ASSD Status Register (ASSD-SR)".
			// todo: handle aborts by legacy command issue.
			if (assd_block[0] != 1)
				break;
		}

		set_task_state(current, TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 100);
	}

	if (assd_set_blksize(host, 512)) {

		printk("assd: set blocklength (512) failed\n");
		// Some cards to not support SET_BLOCKLENGTH, but
                // accept SEND_PSI with a block length of 512.
		//return -EIO;
	}

	if (assd_read_sec_cmd(host)) {

		printk("assd: get response failed\n");
		return -EIO;
	}

	// Get secure token length (STL). See [A1] chapter "Secure Tokens".
	count = assd_block[1] + (assd_block[0] << 8) - 2;

	if (count > MAX_LENGTH)
		count = MAX_LENGTH;

	// todo: this output may be security ciritcal!
	pr_debug_hexstr(&assd_block[2], count);

	// Copy secure token (STL + R-APDU). See [A1] chapter "Secure Tokens".
        if (copy_to_user(buf, assd_block, count + 2))
                return -EFAULT;

        return 0;
}

// EINVAL: invalid ioctl number (0), the C-APDU is too short (1),
//         invalid length for security token (2) or
//         invalid value (3)
// ENODEV: no assd capable card is ready for use
// EIO: internal error while processing the C-APDU
// EFAULT: invalid address in attempting to use an argument
// EBUSY: the device is in use
// ETIMEDOUT: wait for device timed out
//
// -4: set timeout for C-APDU processing. returns 0 on success,
//     EINVAL (3) else. default timeout is 800ms.
// -3: returns the version number of this module. returns 0 on success,
//     EFAULT else.
// -2: test if assd capable card is inserted. arg is timeout.
//     returns 0 if assd capable card is inserted, ETIMEDOUT else.
//     default timeout is 800ms.
// -1: test if assd capable card is inserted.
//     returns 0 if assd capable card is inserted, ENODEV else.
//  0: enable assd capable card.
//     returns 0 if assd capable card is ready for use, ENODEV else.
//  1: process the C-APDU given by argument. returns 0 on success.
//     returns ENODEV, EINVAL (1, 2), ETIMEDOUT or EIO else.
static int
assd_ioctl(struct inode *inode, struct file *file, unsigned int nr,
	   unsigned long arg)
{
	struct mmc_host *host = assd_host;
	int ret = 0;

	pr_debug("assd: assd_ioctl(%d)\n", nr);

	if (nr == -3) {

		if (copy_to_user((char *)arg, VERSION, 8))
			return -EFAULT;

		return 0;
	}

	if (nr == -4) {

		if ((int)arg <= 0)
			return EINVAL;

		assd_timeout = arg;
		return 0;
	}
	// Check if a assd capable card is ready.
	if (host == NULL) {

		if (nr == -2) {

			int timeout = 100;
			if (arg)
				timeout = arg;

			for (; timeout > 0; timeout--) {

				if (assd_host != NULL)
					return 0;

				set_task_state(current, TASK_INTERRUPTIBLE);
				schedule_timeout(HZ / 100);
			}

			return -ETIMEDOUT;
		}

		printk("assd: no assd capable card was found\n");
		return -ENODEV;
	}

	if ((nr == -1) || (nr == -2))
		return 0;

	if (down_trylock(&assd_lock))
		return -EBUSY;

	switch (nr) {

	case 0:
		ret = enable_assd(host);
		break;

	case 1:
		ret = assd_process(host, (unsigned char *)arg);
		break;

	default:
		ret = -EINVAL;
	}

	up(&assd_lock);
	return ret;
}

static struct file_operations fops = {

	.owner = THIS_MODULE,
	.ioctl = assd_ioctl,
};

static struct miscdevice dev = {

	.name = "assd",
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &fops,
};

static struct miscdevice dev_inserted = {

	.name = "assdinserted",
	.minor = MISC_DYNAMIC_MINOR,
	.fops = NULL,
};

// All information needed to implement this function can be found in [1], [A1].
static int assd_probe(struct mmc_card *card)
{
	pr_debug("assd: assd_probe()\n");

	// Currently we support one assd capable card only.
	if (assd_host != NULL)
		return -ENODEV;

	// Check if the inserted card supports assd.
	if (check_assd(card->host, CHECK_FUNCTION, ASSD_1_1)) {

		printk("assd: card does not support assd\n");
		return -ENODEV;
	}

	misc_register(&dev_inserted);
	printk("assd: a assd capable card was found\n");

	// Some cards require a STOP_TRANSMISSION command, some not.
	// Sending a STOP_TRANSMISSION command slows down the processing
	// of a C-APDU if the actual card does not need the
	// STOP_TRANSMISSION command. Unfortunately if the actual card
	// needs a STOP_TRANSMISSION command and we do not send the
	// STOP_TRANSMISSION command, the processing of the C-APDU will
	// fail. So better start with sending a STOP_TRANSMISSION command
	// and disable that lateron as soon as we know what kind of card
	// we deal with.
	set_bit(ASSD_SEND_STOP, &assd_status);

        sema_init(&assd_lock, 1);

	assd_timeout = 100;
	assd_host = card->host;
	return 0;
}

static void assd_remove(struct mmc_card *card)
{
	pr_debug("assd: assd_remove()\n");

	if (card->host == assd_host) {

		misc_deregister(&dev_inserted);
		printk("assd: the assd capable card has been removed\n");

		assd_host = NULL;
	}
}

static int assd_suspend(struct mmc_card *card, pm_message_t state)
{
	pr_debug("assd: assd_suspend()\n");
	return 0;
}

static int assd_resume(struct mmc_card *card)
{
	pr_debug("assd: assd_resume()\n");
	return 0;
}

static struct mmc_driver mmc_drv = {

	.drv = {

		.name = "sd_assd",
		},
	.probe = assd_probe,
	.remove = assd_remove,
	.suspend = assd_suspend,
	.resume = assd_resume,
};

static int
__init assd_init(void)
{
	pr_debug("assd: assd_init()\n");

	assd_block = kzalloc(512, GFP_KERNEL);
	if (assd_block == NULL)
		return -ENOMEM;

	if (misc_register(&dev)) {

		kfree(assd_block);
		return -EIO;
	}

	if (mmc_register_driver(&mmc_drv)) {

		misc_deregister(&dev);

		kfree(assd_block);
		return -EIO;
	}

	return 0;
};

static void
__exit assd_exit(void)
{
	pr_debug("assd: assd_exit()\n");

	mmc_unregister_driver(&mmc_drv);
	misc_deregister(&dev);

	if (assd_host != NULL)
		misc_deregister(&dev_inserted);

	kfree(assd_block);
};

module_init(assd_init);
module_exit(assd_exit);

MODULE_AUTHOR("Giesecke & Devrient GmbH");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Advanced Security SD Extension");
MODULE_VERSION(VERSION);
