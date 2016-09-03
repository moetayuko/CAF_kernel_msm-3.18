/*
 * Copyright (C) 2016 IBM Corporation
 *
 * Authors:
 * Thiago Jung Bauermann <bauerman@linux.vnet.ibm.com>
 * Mimi Zohar <zohar@linux.vnet.ibm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/rculist.h>
#include <linux/rcupdate.h>
#include <linux/parser.h>
#include <linux/vmalloc.h>
#include <linux/kexec.h>
#include <linux/reboot.h>

#include "ima.h"

#ifdef CONFIG_IMA_KEXEC
/* Physical address of the measurement buffer in the next kernel. */
static unsigned long kexec_buffer_load_addr;
static size_t kexec_segment_size;

static int ima_dump_measurement_list(unsigned long *buffer_size, void **buffer,
				     unsigned long segment_size)
{
	struct ima_queue_entry *qe;
	struct seq_file file;
	struct ima_kexec_hdr khdr = {
		.version = 1, .buffer_size = 0, .count = 0};
	int ret = 0;

	/* segment size can't change between kexec load and execute */
	file.buf = vmalloc(segment_size);
	if (!file.buf) {
		ret = -ENOMEM;
		goto out;
	}

	file.size = segment_size;
	file.read_pos = 0;
	file.count = sizeof(khdr);	/* reserved space */

	list_for_each_entry_rcu(qe, &ima_measurements, later) {
		if (file.count < file.size) {
			khdr.count++;
			ima_measurements_show(&file, qe);
		} else {
			ret = -EINVAL;
			break;
		}
	}

	if (ret < 0)
		goto out;

	/*
	 * fill in reserved space with some buffer details
	 * (eg. version, buffer size, number of measurements)
	 */
	khdr.buffer_size = file.count;
	memcpy(file.buf, &khdr, sizeof(khdr));
	print_hex_dump(KERN_DEBUG, "ima dump: ", DUMP_PREFIX_NONE,
			16, 1, file.buf,
			file.count < 100 ? file.count : 100, true);

	*buffer_size = file.count;
	*buffer = file.buf;
out:
	if (ret == -EINVAL)
		vfree(file.buf);
	return ret;
}

/*
 * Called during kexec execute so that IMA can save the measurement list.
 */
static int ima_update_kexec_buffer(struct notifier_block *self,
				   unsigned long action, void *data)
{
	void *kexec_buffer = NULL;
	size_t kexec_buffer_size;
	int ret;

	if (!kexec_in_progress)
		return NOTIFY_OK;

	kexec_buffer_size = ima_get_binary_runtime_size();
	if (kexec_buffer_size >
	    (kexec_segment_size - sizeof(struct ima_kexec_hdr))) {
		pr_err("Binary measurement list grew too large.\n");
		goto out;
	}

	ima_dump_measurement_list(&kexec_buffer_size, &kexec_buffer,
				  kexec_segment_size);
	if (!kexec_buffer) {
		pr_err("Not enough memory for the kexec measurement buffer.\n");
		goto out;
	}
	ret = kexec_update_segment(kexec_buffer, kexec_buffer_size,
				   kexec_buffer_load_addr, kexec_segment_size);
	if (ret)
		pr_err("Error updating kexec buffer: %d\n", ret);
out:
	return NOTIFY_OK;
}

struct notifier_block update_buffer_nb = {
	.notifier_call = ima_update_kexec_buffer,
};

/*
 * Called during kexec_file_load so that IMA can add a segment to the kexec
 * image for the measurement list for the next kernel.
 */
void ima_add_kexec_buffer(struct kimage *image)
{
	static int registered = 0;
	struct kexec_buf kbuf = { .image = image, .buf_align = PAGE_SIZE,
				  .buf_min = 0, .buf_max = ULONG_MAX,
				  .top_down = true, .skip_checksum = true };
	int ret;

	if (!kexec_can_hand_over_buffer())
		return;

	kexec_segment_size = ALIGN(ima_get_binary_runtime_size() + PAGE_SIZE,
				   PAGE_SIZE);

	if (kexec_segment_size >= (ULONG_MAX - sizeof(long))) {
		pr_err("Binary measurement list too large.\n");
		return;
	}

	/* Ask not to checksum the segment, we will update it later. */
	kbuf.buffer = NULL;
	kbuf.bufsz = 0;
	kbuf.memsz = kexec_segment_size;
	ret = kexec_add_handover_buffer(&kbuf);
	if (ret) {
		pr_err("Error passing over kexec measurement buffer.\n");
		return;
	}
	kexec_buffer_load_addr = kbuf.mem;

	pr_debug("kexec measurement buffer for the loaded kernel at 0x%lx.\n",
		 kexec_buffer_load_addr);

	if (registered)
		return;

	register_reboot_notifier(&update_buffer_nb);
	registered = 1;
}
#endif /* IMA_KEXEC */

/*
 * Restore the measurement list from the previous kernel.
 */
void ima_load_kexec_buffer(void)
{
	void *kexec_buffer = NULL;
	size_t kexec_buffer_size = 0;
	int rc;

	rc = kexec_get_handover_buffer(&kexec_buffer, &kexec_buffer_size);
	switch (rc) {
	case 0:
		rc = ima_restore_measurement_list(kexec_buffer_size,
						  kexec_buffer);
		if (rc != 0)
			pr_err("Failed to restore the measurement list: %d\n",
				rc);

		kexec_free_handover_buffer();
		break;
	case -ENOTSUPP:
		pr_debug("Restoring the measurement list not supported\n");
		break;
	case -ENOENT:
		pr_debug("No measurement list to restore\n");
		break;
	default:
		pr_debug("Error restoring the measurement list: %d\n", rc);
	}
}
