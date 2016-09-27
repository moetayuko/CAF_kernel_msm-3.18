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
