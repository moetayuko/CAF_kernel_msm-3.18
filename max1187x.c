/* drivers/input/touchscreen/max1187x.c
 *
 * Copyright (c)2013 Maxim Integrated Products, Inc.
 *
 * Driver Version: 3.0.7
 * Release Date: Feb 22, 2013
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/crc16.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/jiffies.h>
#include <asm/byteorder.h>
#include <linux/input/max1187x.h>
#include <linux/input/max1187x_config.h>

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(fmt) MAX1187X_NAME "(%s:%d): " fmt, __func__, __LINE__
#endif

#define pr_info_if(a, b, ...) do { if (debug_mask & a) \
			pr_info(b, ##__VA_ARGS__);	\
			} while (0)
#define debugmask_if(a) (debug_mask & a)

#define ENABLE_IRQ()                            \
do {                                            \
	mutex_lock(&ts->irq_mutex);             \
	if (ts->irq_disabled) {                 \
		enable_irq(ts->client->irq);    \
		ts->irq_disabled = 0;           \
	}                                       \
	mutex_unlock(&ts->irq_mutex);           \
} while (0)

#define DISABLE_IRQ()                           \
do {                                            \
	mutex_lock(&ts->irq_mutex);             \
	if (ts->irq_disabled == 0) {            \
		disable_irq(ts->client->irq);   \
		ts->irq_disabled = 1;           \
	}                                       \
	mutex_unlock(&ts->irq_mutex);           \
} while (0)

#define NWORDS(a)    (sizeof(a) / sizeof(u16))
#define BYTE_SIZE(a) ((a) * sizeof(u16))
#define BYTEH(a)     ((a) >> 8)
#define BYTEL(a)     ((a) & 0xFF)

#define PDATA(a)      (ts->pdata->a)

static u16 debug_mask;

#ifdef MAX1187X_LOCAL_PDATA
struct max1187x_pdata local_pdata = { };
#endif

struct report_reader {
	u16 report_id;
	u16 reports_passed;
	struct semaphore sem;
	int status;
};

struct data {
	struct max1187x_pdata *pdata;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct early_suspend early_suspend;
	u8 early_suspend_registered;
	struct workqueue_struct *wq;
	struct work_struct work_irq;
	atomic_t scheduled_work_irq;
	u32 irq_receive_time;
	struct mutex irq_mutex;
	struct mutex i2c_mutex;
	struct mutex report_mutex;
	struct semaphore report_sem;
	struct report_reader report_readers[MAX_REPORT_READERS];
	u8 irq_disabled;
	u8 report_readers_outstanding;
	u16 rx_report[1000]; /* with header */
	u16 rx_report_len;
	u16 rx_packet[MAX_WORDS_REPORT + 1]; /* with header */
	u32 irq_count;
	u16 framecounter;
	u16 list_finger_ids;
	u8 got_report;
	int fw_index;
	u16 fw_crc16;
	u16 fw_version[MAX_WORDS_REPORT];
	u16 touch_config[MAX_WORDS_COMMAND_ALL];
	char phys[32];
	u8 fw_responsive;
	u8 have_fw;
	u8 have_touchcfg;
	u8 sysfs_created;
	u8 is_raw_mode;
	char debug_string[DEBUG_STRING_LEN_MAX];
};

static void early_suspend(struct early_suspend *h);
static void late_resume(struct early_suspend *h);

static int device_init(struct i2c_client *client);
static int device_deinit(struct i2c_client *client);

static int bootloader_enter(struct data *ts);
static int bootloader_exit(struct data *ts);
static int bootloader_get_crc(struct data *ts, u16 *crc16,
		u16 addr, u16 len, u16 delay);
static int bootloader_set_byte_mode(struct data *ts);
static int bootloader_erase_flash(struct data *ts);
static int bootloader_write_flash(struct data *ts, const u8 *image, u16 length);

static int change_touch_rpt(struct i2c_client *client, u16 to);
static int sreset(struct i2c_client *client);
static int get_touch_config(struct i2c_client *client);
static int get_fw_version(struct i2c_client *client);
static void propagate_report(struct data *ts, int status, u16 *report);
static int get_report(struct data *ts, u16 report_id, ulong timeout);
static void release_report(struct data *ts);

#if MAXIM_TOUCH_REPORT_MODE == 2
static u16 binary_search(const u16 *array, u16 len, u16 val);
static s16 max1187x_orientation(s16 x, s16 y);
static u16 max1187x_sqrt(u32 num);
#endif

static u8 bootloader;
static u8 init_state;

/* I2C communication */
/* debug_mask |= 0x1 for I2C RX communication */
static int i2c_rx_bytes(struct data *ts, u8 *buf, u16 len)
{
	int i, ret, written;

	do {
		ret = i2c_master_recv(ts->client, (char *) buf, (int) len);
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C RX fail (%d)", ret);
		return ret;
	}

	len = ret;

	if (debugmask_if(1)) {
		pr_info("I2C RX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(ts->debug_string + written, 6,
					"0x%02X,", buf[i]);
			if (written + 6 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", ts->debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", ts->debug_string);
	}

	return len;
}

static int i2c_rx_words(struct data *ts, u16 *buf, u16 len)
{
	int i, ret, written;

	do {
		ret = i2c_master_recv(ts->client,
			(char *) buf, (int) (len * 2));
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C RX fail (%d)", ret);
		return ret;
	}

	if ((ret % 2) != 0) {
		pr_err("I2C words RX fail: odd number of bytes (%d)", ret);
		return -EIO;
	}

	len = ret/2;

#ifdef __BIG_ENDIAN
	for (i = 0; i < len; i++)
		buf[i] = (buf[i] << 8) | (buf[i] >> 8);
#endif
	if (debugmask_if(1)) {
		pr_info("I2C RX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(ts->debug_string + written,
					8, "0x%04X,", buf[i]);
			if (written + 8 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", ts->debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", ts->debug_string);
	}

	return len;
}

/* debug_mask |= 0x2 for I2C TX communication */
static int i2c_tx_bytes(struct data *ts, u8 *buf, u16 len)
{
	int i, ret, written;

	do {
		ret = i2c_master_send(ts->client, (char *) buf, (int) len);
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C TX fail (%d)", ret);
		return ret;
	}

	len = ret;

	if (debugmask_if(2)) {
		pr_info("I2C TX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(ts->debug_string + written, 6,
					"0x%02X,", buf[i]);
			if (written + 6 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", ts->debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", ts->debug_string);
	}

	return len;
}

static int i2c_tx_words(struct data *ts, u16 *buf, u16 len)
{
	int i, ret, written;

#ifdef __BIG_ENDIAN
	for (i = 0; i < len; i++)
		buf[i] = (buf[i] << 8) | (buf[i] >> 8);
#endif
	do {
		ret = i2c_master_send(ts->client,
			(char *) buf, (int) (len * 2));
	} while (ret == -EAGAIN);
	if (ret < 0) {
		pr_err("I2C TX fail (%d)", ret);
		return ret;
	}
	if ((ret % 2) != 0) {
		pr_err("I2C words TX fail: odd number of bytes (%d)", ret);
		return -EIO;
	}

	len = ret/2;

	if (debugmask_if(2)) {
		pr_info("I2C TX (%d):", len);
		written = 0;
		for (i = 0; i < len; i++) {
			written += snprintf(ts->debug_string + written, 8,
					"0x%04X,", buf[i]);
			if (written + 8 >= DEBUG_STRING_LEN_MAX) {
				pr_info("%s", ts->debug_string);
				written = 0;
			}
		}
		if (written > 0)
			pr_info("%s", ts->debug_string);
	}

	return len;
}

/* Read report */
static int read_mtp_report(struct data *ts, u16 *buf)
{
	int words = 1, words_tx, words_rx;
	int ret = 0, remainder = 0, offset = 0;
	u16 address = 0x000A;

	mutex_lock(&ts->i2c_mutex);
	/* read header, get size, read entire report */
	{
		words_tx = i2c_tx_words(ts, &address, 1);
		if (words_tx != 1) {
			mutex_unlock(&ts->i2c_mutex);
			pr_err("Report RX fail: failed to set address");
			return -EIO;
		}

		if (ts->is_raw_mode == 0) {
			words_rx = i2c_rx_words(ts, buf, 2);
			if (words_rx != 2 || BYTEL(buf[0]) > MAX_WORDS_REPORT) {
				ret = -EIO;
				pr_err("Report RX fail: received (%d) " \
						"expected (%d) words, " \
						"header (%04X)",
						words_rx, words, buf[0]);
				mutex_unlock(&ts->i2c_mutex);
				return ret;
			}

			if ((buf[0] == 0x31F4
				|| buf[0] == 0x41F4)
				&& buf[1] == 0x0800)
				ts->is_raw_mode = 1;

			words = BYTEL(buf[0]) + 1;

			words_tx = i2c_tx_words(ts, &address, 1);
			if (words_tx != 1) {
				mutex_unlock(&ts->i2c_mutex);
				pr_err("Report RX fail:" \
					"failed to set address");
				return -EIO;
			}

			words_rx = i2c_rx_words(ts, &buf[offset], words);
			if (words_rx != words) {
				mutex_unlock(&ts->i2c_mutex);
				pr_err("Report RX fail 0x%X: received (%d) " \
					"expected (%d) words",
					address, words_rx, remainder);
				return -EIO;

			}

		} else {

			words_rx = i2c_rx_words(ts, buf,
					(u16) PDATA(i2c_words));
			if (words_rx != (u16) PDATA(i2c_words) || BYTEL(buf[0])
					> MAX_WORDS_REPORT) {
				ret = -EIO;
				pr_err("Report RX fail: received (%d) " \
					"expected (%d) words, header (%04X)",
					words_rx, words, buf[0]);
				mutex_unlock(&ts->i2c_mutex);
				return ret;
			}

			if (BYTEH(buf[0]) == 0x11)
				ts->is_raw_mode = 0;

			words = BYTEL(buf[0]) + 1;
			remainder = words;

			if (remainder - (u16) PDATA(i2c_words) > 0) {
				remainder -= (u16) PDATA(i2c_words);
				offset += (u16) PDATA(i2c_words);
				address += (u16) PDATA(i2c_words);
			}

			words_tx = i2c_tx_words(ts, &address, 1);
			if (words_tx != 1) {
				mutex_unlock(&ts->i2c_mutex);
				pr_err("Report RX fail: failed to set " \
					"address 0x%X", address);
				return -EIO;
			}

			words_rx = i2c_rx_words(ts, &buf[offset], remainder);
			if (words_rx != remainder) {
				mutex_unlock(&ts->i2c_mutex);
				pr_err("Report RX fail 0x%X: received (%d) " \
						"expected (%d) words",
						address, words_rx, remainder);
				return -EIO;
			}
		}
	}
	mutex_unlock(&ts->i2c_mutex);
	return ret;
}

/* Send command */
static int send_mtp_command(struct data *ts, u16 *buf, u16 len)
{
	u16 tx_buf[MAX_WORDS_COMMAND + 2]; /* with address and header */
	u16 packets, words, words_tx;
	int i, ret = 0;

	/* check basics */
	if (len < 2) {
		pr_err("Command too short (%d); 2 words minimum", len);
		return -EINVAL;
	}
	if ((buf[1] + 2) != len) {
		pr_err("Inconsistent command length: " \
				"expected (%d) given (%d)", (buf[1] + 2), len);
		return -EINVAL;
	}

	if (len > MAX_WORDS_COMMAND_ALL) {
		pr_err("Command too long (%d); maximum (%d) words",
				len, MAX_WORDS_COMMAND_ALL);
		return -EINVAL;
	}

	/* packetize and send */
	packets = len / MAX_WORDS_COMMAND;
	if (len % MAX_WORDS_COMMAND)
		packets++;
	tx_buf[0] = 0x0000;

	mutex_lock(&ts->i2c_mutex);
	for (i = 0; i < packets; i++) {
		words = (i == (packets - 1)) ? len : MAX_WORDS_COMMAND;
		tx_buf[1] = (packets << 12) | ((i + 1) << 8) | words;
		memcpy(&tx_buf[2], &buf[i * MAX_WORDS_COMMAND],
			BYTE_SIZE(words));
		words_tx = i2c_tx_words(ts, tx_buf, words + 2);
		if (words_tx != (words + 2)) {
			ret = -1;
			pr_err("Command TX fail: transmitted (%d) " \
				"expected (%d) words, packet (%d)",
				words_tx, words + 2, i);
		}
		len -= MAX_WORDS_COMMAND;
	}
	ts->got_report = 0;
	mutex_unlock(&ts->i2c_mutex);

	return ret;
}

/* Integer math operations */
#if MAXIM_TOUCH_REPORT_MODE == 2
/* Returns index of element in array closest to val */
static u16 binary_search(const u16 *array, u16 len, u16 val)
{
	s16 lt, rt, mid;
	if (len < 2)
		return 0;

	lt = 0;
	rt = len - 1;

	while (lt <= rt) {
		mid = (lt + rt)/2;
		if (val == array[mid])
			return mid;
		if (val < array[mid])
			rt = mid - 1;
		else
			lt = mid + 1;
	}

	if (lt >= len)
		return len - 1;
	if (rt < 0)
		return 0;
	if (array[lt] - val > val - array[lt-1])
		return lt-1;
	else
		return lt;
}

/* Given values of x and y, it calculates the orientation
 * with respect to y axis by calculating atan(x/y)
 */
static s16 max1187x_orientation(s16 x, s16 y)
{
	u16 sign = 0;
	u16 len = sizeof(tanlist)/sizeof(tanlist[0]);
	u32 quotient;
	s16 angle;

	if (x == y) {
		angle = 45;
		return angle;
	}
	if (x == 0) {
		angle = 0;
		return angle;
	}
	if (y == 0) {
		if (x > 0)
			angle = 90;
		else
			angle = -90;
		return angle;
	}

	if (x < 0) {
		sign = ~sign;
		x = -x;
	}
	if (y < 0) {
		sign = ~sign;
		y = -y;
	}

	if (x == y)
		angle = 45;
	else if (x < y) {
		quotient = ((u32)x << 16) - (u32)x;
		quotient = quotient / y;
		angle = binary_search(tanlist, len, quotient);
	} else {
		quotient = ((u32)y << 16) - (u32)y;
		quotient = quotient / x;
		angle = binary_search(tanlist, len, quotient);
		angle = 90 - angle;
	}
	if (sign == 0)
		return angle;
	else
		return -angle;
}

u16 max1187x_sqrt(u32 num)
{
	u16 mask = 0x8000;
	u16 guess = 0;
	u32 prod = 0;

	if (num < 2)
		return num;

	while (mask) {
		guess = guess ^ mask;
		prod = guess*guess;
		if (num < prod)
			guess = guess ^ mask;
		mask = mask>>1;
	}
	if (guess != 0xFFFF) {
		prod = guess*guess;
		if ((num - prod) > (prod + 2*guess + 1 - num))
			guess++;
	}

	return guess;
}
#endif
/* debug_mask |= 0x4 for touch reports */
static void process_touch_report(struct data *ts, u16 *buf)
{
	u32 i, j;
	u16 x, y, swap_u16, curr_finger_ids;
	u32 area;
	u32 major_axis, minor_axis;
	s16 xsize, ysize, orientation, swap_s16;

	struct max1187x_touch_report_header *header;
	struct max1187x_touch_report_basic *reportb;
	struct max1187x_touch_report_extended *reporte;

	header = (struct max1187x_touch_report_header *) buf;

	if (!ts->input_dev)
		goto err_process_touch_report_inputdev;
	if (BYTEH(header->header) != 0x11)
		goto err_process_touch_report_header;

	if (header->report_id != MAX1187X_TOUCH_REPORT_BASIC &&
			header->report_id != MAX1187X_TOUCH_REPORT_EXTENDED)
		goto err_process_touch_report_reportid;

	if (ts->framecounter == header->framecounter) {
		pr_err("Same framecounter (%u) encountered at irq (%u)!\n",
				ts->framecounter, ts->irq_count);
		goto err_process_touch_report_framecounter;
	}
	ts->framecounter = header->framecounter;

	if (header->touch_count > 10) {
		pr_err("Touch count (%u) out of bounds [0,10]!",
				header->touch_count);
		goto err_process_touch_report_touchcount;
	}

	if (header->touch_count == 0) {
		pr_info_if(4, "(TOUCH): Fingers up\n");
#ifdef MAX1187X_PROTOCOL_A
		input_mt_sync(ts->input_dev);
#else
		for (i = 0; i < MAX1187X_TOUCH_COUNT_MAX; i++) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev,
						MT_TOOL_FINGER, 0);
		}
#endif
		input_sync(ts->input_dev);
		ts->list_finger_ids = 0;
	} else {
		curr_finger_ids = 0;
		reportb = (struct max1187x_touch_report_basic *)
				((u8 *)buf + sizeof(*header));
		reporte = (struct max1187x_touch_report_extended *)
				((u8 *)buf + sizeof(*header));
		for (i = 0; i < header->touch_count; i++) {
			x = reportb->x;
			y = reportb->y;
			if (PDATA(coordinate_settings) & MAX1187X_REVERSE_X) {
				x = PDATA(panel_margin_xl) + PDATA(lcd_x)
					+ PDATA(panel_margin_xh) - 1 - x;
			}
			if (PDATA(coordinate_settings) & MAX1187X_REVERSE_Y) {
				y = PDATA(panel_margin_yl) + PDATA(lcd_y)
					+ PDATA(panel_margin_yh) - 1 - y;
			}
			if (PDATA(coordinate_settings) & MAX1187X_SWAP_XY) {
				swap_u16 = x;
				x = y;
				y = swap_u16;
			}
			if (reportb->z == 0)
				reportb->z++;
			pr_info_if(8, "(TOUCH): (%u) Finger %u: "\
				"X(%d) Y(%d) Z(%d)",
				header->framecounter, reportb->finger_id,
				x, y, reportb->z);
			curr_finger_ids |= (1<<reportb->finger_id);
#ifdef MAX1187X_PROTOCOL_A
			input_report_abs(ts->input_dev,
				ABS_MT_TRACKING_ID,	reportb->finger_id);
#else
			input_mt_slot(ts->input_dev, reportb->finger_id);
			input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, 1);
#endif
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev,	ABS_MT_POSITION_Y, y);
			input_report_abs(ts->input_dev,
					ABS_MT_PRESSURE, reportb->z);
			if (header->report_id
				== MAX1187X_TOUCH_REPORT_EXTENDED) {
#if MAXIM_TOUCH_REPORT_MODE == 2
				xsize = (reporte->xpixel - 1)
					* (s16)(PDATA(lcd_x)/PDATA(num_rows));
				ysize = (reporte->ypixel - 1)
					* (s16)(PDATA(lcd_y)/PDATA(num_cols));
				if (PDATA(coordinate_settings)
						& MAX1187X_REVERSE_X)
					xsize = -xsize;
				if (PDATA(coordinate_settings)
						& MAX1187X_REVERSE_Y)
					ysize = -ysize;
				if (PDATA(coordinate_settings)
						& MAX1187X_SWAP_XY) {
					swap_s16 = xsize;
					xsize = ysize;
					ysize = swap_s16;
				}
				/* Calculate orientation as
				 * arctan of xsize/ysize) */
				orientation =
					max1187x_orientation(xsize, ysize);
				area = reporte->area
					* (PDATA(lcd_x)/PDATA(num_rows))
					* (PDATA(lcd_y)/PDATA(num_cols));
				/* Major axis of ellipse if hypotenuse
				 * formed by xsize and ysize */
				major_axis = xsize*xsize + ysize*ysize;
				major_axis = max1187x_sqrt(major_axis);
				/* Minor axis can be reverse calculated
				 * using the area of ellipse:
				 * Area of ellipse =
				 *		pi / 4 * Major axis * Minor axis
				 * Minor axis =
				 *		4 * Area / (pi * Major axis)
				 */
				minor_axis = (2 * area) / major_axis;
				minor_axis = (minor_axis<<17) / MAX1187X_PI;
				pr_info_if(8, "(TOUCH): Finger %u: " \
					"Orientation(%d) Area(%u) Major_axis(%u) Minor_axis(%u)",
					reportb->finger_id,	orientation,
					area, major_axis, minor_axis);
				input_report_abs(ts->input_dev,
					ABS_MT_ORIENTATION, orientation);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MAJOR, major_axis);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MINOR, minor_axis);
#endif
				reporte++;
				reportb = (struct max1187x_touch_report_basic *)
						((u8 *) reporte);
			} else {
				reportb++;
			}
#ifdef MAX1187X_PROTOCOL_A
			input_mt_sync(ts->input_dev);
#endif
		}
#ifndef MAX1187X_PROTOCOL_A
		i = 0;
		j = 1;
		while (ts->list_finger_ids) {
			if ((ts->list_finger_ids & j) != 0 &&
					(curr_finger_ids & j) == 0) {
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev,
						MT_TOOL_FINGER, 0);
			}
			i++;
			j <<= 1;
			ts->list_finger_ids >>= 1;
		}
#endif
		input_sync(ts->input_dev);
		ts->list_finger_ids = curr_finger_ids;
	}
err_process_touch_report_touchcount:
err_process_touch_report_inputdev:
err_process_touch_report_header:
err_process_touch_report_reportid:
err_process_touch_report_framecounter:
	return;
}


static void max1187x_wfxn_irq(struct work_struct *work)
{
	struct data *ts = container_of(work, struct data, work_irq);

	int read_retval = read_mtp_report(ts, ts->rx_packet);

	u64	time_elapsed = jiffies;
	if (time_elapsed >= ts->irq_receive_time)
		time_elapsed = time_elapsed - ts->irq_receive_time;
	else
		time_elapsed = time_elapsed +
					0x100000000 - ts->irq_receive_time;

	if (read_retval == 0 || time_elapsed > 2 * HZ) {
		process_touch_report(ts, ts->rx_packet);
		propagate_report(ts, 0, ts->rx_packet);
	}
	atomic_dec(&ts->scheduled_work_irq);
	/* enable_irq(ts->client->irq); */
}

static irqreturn_t irq_handler(int irq, void *context)
{
	struct data *ts = (struct data *) context;

	if (atomic_read(&ts->scheduled_work_irq) != 0)
		return IRQ_HANDLED;

	if (gpio_get_value(ts->pdata->gpio_tirq) != 0)
		return IRQ_HANDLED;

	/* disable_irq_nosync(ts->client->irq); */
	atomic_inc(&ts->scheduled_work_irq);
	ts->irq_receive_time = jiffies;
	ts->irq_count++;

	queue_work(ts->wq, &ts->work_irq);
	return IRQ_HANDLED;
}

static ssize_t init_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", init_state);
}

static ssize_t init_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int value, ret;

	if (sscanf(buf, "%d", &value) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}
	switch (value) {
	case 0:
		if (init_state == 0)
			break;
		ret = device_deinit(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("deinit error (%d)", ret);
			return ret;
		}
		break;
	case 1:
		if (init_state == 1)
			break;
		ret = device_init(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("init error (%d)", ret);
			return ret;
		}
		break;
	case 2:
		if (init_state == 1) {
			ret = device_deinit(to_i2c_client(dev));
			if (ret != 0) {
				pr_err("deinit error (%d)", ret);
				return ret;
			}
		}
		ret = device_init(to_i2c_client(dev));
		if (ret != 0) {
			pr_err("init error (%d)", ret);
			return ret;
		}
		break;
	default:
		pr_err("bad value");
		return -EINVAL;
	}

	return count;
}

static ssize_t sreset_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	DISABLE_IRQ();
	if (sreset(client) != 0) {
		pr_err("Failed to do soft reset.");
		return count;
	}
	if (get_report(ts, 0x01A0, 3000) != 0) {
		pr_err("Failed to receive system status report");
		return count;
	}

	release_report(ts);
	return count;
}

static ssize_t irq_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u\n", ts->irq_count);
}

static ssize_t irq_count_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	ts->irq_count = 0;
	return count;
}

static ssize_t dflt_cfg_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u 0x%x 0x%x\n", PDATA(defaults_allow),
			PDATA(default_chip_config), PDATA(default_chip_id));
}

static ssize_t dflt_cfg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	(void) sscanf(buf, "%u 0x%x 0x%x", &PDATA(defaults_allow),
			&PDATA(default_chip_config), &PDATA(default_chip_id));
	return count;
}

static ssize_t panel_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%u %u %u %u %u %u\n",
			PDATA(panel_margin_xl), PDATA(panel_margin_xh),
			PDATA(panel_margin_yl), PDATA(panel_margin_yh),
			PDATA(lcd_x), PDATA(lcd_y));
}

static ssize_t panel_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);

	(void) sscanf(buf, "%u %u %u %u %u %u", &PDATA(panel_margin_xl),
			&PDATA(panel_margin_xh), &PDATA(panel_margin_yl),
			&PDATA(panel_margin_yh), &PDATA(lcd_x),
			&PDATA(lcd_y));
	return count;
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	u16 build_number = 0;
	u8 branch = BYTEL(ts->fw_version[3]) >> 6;

	if (ts->fw_version[1] >= 3)
		build_number = ts->fw_version[4];
	return snprintf(
			buf,
			PAGE_SIZE,
			"%u.%u.%u p%u%c "
				"(CRC16 0x%04X=>0x%04X) Chip ID 0x%02X\n",
			BYTEH(ts->fw_version[2]),
			BYTEL(ts->fw_version[2]),
			build_number,
			BYTEL(ts->fw_version[3]) & 0x3F,
			(branch == 0) ? ' ' : (branch - 1 + 'a'),
			(ts->fw_index != -1) ? \
			PDATA(fw_mapping[ts->fw_index]).file_codesize \
			: 0, ts->fw_crc16, BYTEH(ts->fw_version[3]));
}

static ssize_t driver_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "3.0.7: Feb 22, 2013\n");
}

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%04X\n", debug_mask);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	if (sscanf(buf, "%hx", &debug_mask) != 1) {
		pr_err("bad parameter");
		return -EINVAL;
	}

	return count;
}

static ssize_t command_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct data *ts = i2c_get_clientdata(client);
	u16 buffer[MAX_WORDS_COMMAND_ALL];
	char scan_buf[5];
	int i;

	count--; /* ignore carriage return */
	if ((count % 4) != 0) {
		pr_err("words not properly defined");
		return -EINVAL;
	}
	scan_buf[4] = '\0';
	for (i = 0; i < count; i += 4) {
		memcpy(scan_buf, &buf[i], 4);
		if (sscanf(scan_buf, "%hx", &buffer[i / 4]) != 1) {
			pr_err("bad word (%s)", scan_buf);
			return -EINVAL;
		}

	}
	if (send_mtp_command(ts, buffer, count / 4))
		pr_err("MTP command failed");
	return ++count;
}

static ssize_t report_read(struct file *file, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	struct data *ts = i2c_get_clientdata(client);
	int printed, i, offset = 0, payload;
	int full_packet;
	int num_term_char;

	if (get_report(ts, 0xFFFF, 0xFFFFFFFF))
		return 0;

	payload = ts->rx_report_len;
	full_packet = payload;
	num_term_char = 2; /* number of term char */
	if (count < (4 * full_packet + num_term_char))
		return -EIO;
	if (count > (4 * full_packet + num_term_char))
		count = 4 * full_packet + num_term_char;

	for (i = 1; i <= payload; i++) {
		printed = snprintf(&buf[offset], PAGE_SIZE, "%04X\n",
			ts->rx_report[i]);
		if (printed <= 0)
			return -EIO;
		offset += printed - 1;
	}
	snprintf(&buf[offset], PAGE_SIZE, ",\n");
	release_report(ts);

	return count;
}

static DEVICE_ATTR(init, 0666, init_show, init_store);
static DEVICE_ATTR(sreset, 0222, NULL, sreset_store);
static DEVICE_ATTR(irq_count, 0666, irq_count_show, irq_count_store);
static DEVICE_ATTR(dflt_cfg, 0666, dflt_cfg_show, dflt_cfg_store);
static DEVICE_ATTR(panel, 0666, panel_show, panel_store);
static DEVICE_ATTR(fw_ver, 0444, fw_ver_show, NULL);
static DEVICE_ATTR(driver_ver, 0444, driver_ver_show, NULL);
static DEVICE_ATTR(debug, 0666, debug_show, debug_store);
static DEVICE_ATTR(command, 0222, NULL, command_store);
static struct bin_attribute dev_attr_report = {
		.attr = {.name = "report", .mode = 0444}, .read = report_read };

static struct device_attribute *dev_attrs[] = {
		&dev_attr_sreset,
		&dev_attr_irq_count,
		&dev_attr_dflt_cfg,
		&dev_attr_panel,
		&dev_attr_fw_ver,
		&dev_attr_driver_ver,
		&dev_attr_debug,
		&dev_attr_command,
		NULL };

/* debug_mask |= 0x8 for all driver INIT */
static void collect_chip_data(struct data *ts)
{
	int ret;

	ret = get_report(ts, 0x01A0, 3000);
	if (ret != 0) {
		pr_err("Failed to receive system status report");
		if (PDATA(defaults_allow) == 0)
			msleep(5000);
	} else {
		release_report(ts);
		ts->fw_responsive = 1;
	}
	DISABLE_IRQ();
	ret = get_fw_version(ts->client);
	if (ret < 0)
		pr_err("Failed to retrieve firmware version");
	if (ret == 0) {
		ret = get_report(ts, 0x0140, 100);
		if (ret != 0)
			pr_err("Failed to receive firmware version report");
		if (ret == 0) {
			memcpy(ts->fw_version, &ts->rx_report[1],
					BYTE_SIZE(ts->rx_report[2] + 2));
			release_report(ts);
			ts->have_fw = 1;
		}
	}
	DISABLE_IRQ();
	ret = get_touch_config(ts->client);
	if (ret < 0)
		pr_err("Failed to retrieve touch config");
	if (ret == 0) {
		ret = get_report(ts, 0x0102, 100);
		if (ret != 0)
			pr_err("Failed to receive touch config report");
		if (ret == 0) {
			memcpy(ts->touch_config, &ts->rx_report[1],
					BYTE_SIZE(ts->rx_report[2] + 2));
			release_report(ts);
			ts->have_touchcfg = 1;
		}
	}
	ENABLE_IRQ();
	pr_info_if(8, "(INIT): firmware responsive: (%u)", ts->fw_responsive);
	if (ts->fw_responsive) {
		if (ts->have_fw)
			pr_info_if(8, "(INIT): firmware version: %u.%u " \
					"Chip ID: 0x%02X",
					BYTEH(ts->fw_version[2]),
					BYTEL(ts->fw_version[2]),
					BYTEH(ts->fw_version[3]));
		if (ts->have_touchcfg)
			pr_info_if(8, "(INIT): configuration ID: 0x%04X",
					ts->touch_config[2]);
	}
}

static int device_fw_load(struct data *ts, const struct firmware *fw,
	u16 fw_index)
{
	u16 filesize, file_codesize, loopcounter;
	u16 file_crc16_1, file_crc16_2, local_crc16;
	int chip_crc16_1 = -1, chip_crc16_2 = -1, ret;

	filesize = PDATA(fw_mapping[fw_index]).filesize;
	file_codesize = PDATA(fw_mapping[fw_index]).file_codesize;

	if (fw->size != filesize) {
		pr_err("filesize (%d) is not equal to expected size (%d)",
				fw->size, filesize);
		return -EIO;
	}

	file_crc16_1 = crc16(0, fw->data, file_codesize);

	loopcounter = 0;
	do {
		ret = bootloader_enter(ts);
		if (ret == 0)
			ret = bootloader_get_crc(ts, &local_crc16,
				0, file_codesize, 200);
		if (ret == 0)
			chip_crc16_1 = local_crc16;
		ret = bootloader_exit(ts);
		loopcounter++;
	} while (loopcounter < MAX_FW_RETRIES && chip_crc16_1 == -1);

	pr_info_if(8, "(INIT): file_crc16_1 = 0x%04x, chip_crc16_1 = 0x%04x\n",
			file_crc16_1, chip_crc16_1);

	ts->fw_index = fw_index;
	ts->fw_crc16 = chip_crc16_1;

	if (file_crc16_1 != chip_crc16_1) {
		loopcounter = 0;
		file_crc16_2 = crc16(0, fw->data, filesize);

		while (loopcounter < MAX_FW_RETRIES && file_crc16_2
				!= chip_crc16_2) {
			pr_info_if(8, "(INIT): Reprogramming chip. Attempt %d",
					loopcounter+1);
			ret = bootloader_enter(ts);
			if (ret == 0)
				ret = bootloader_erase_flash(ts);
			if (ret == 0)
				ret = bootloader_set_byte_mode(ts);
			if (ret == 0)
				ret = bootloader_write_flash(ts, fw->data,
					filesize);
			if (ret == 0)
				ret = bootloader_get_crc(ts, &local_crc16,
					0, filesize, 200);
			if (ret == 0)
				chip_crc16_2 = local_crc16;
			pr_info_if(8, "(INIT): file_crc16_2 = 0x%04x, "\
					"chip_crc16_2 = 0x%04x\n",
					file_crc16_2, chip_crc16_2);
			ret = bootloader_exit(ts);
			loopcounter++;
		}

		if (file_crc16_2 != chip_crc16_2)
			return -EAGAIN;
	}

	loopcounter = 0;
	do {
		ret = bootloader_exit(ts);
		loopcounter++;
	} while (loopcounter < MAX_FW_RETRIES && ret != 0);

	if (ret != 0)
		return -EIO;

	ts->fw_crc16 = file_crc16_1;

	collect_chip_data(ts);
	if (ts->have_fw == 0 || ts->have_touchcfg == 0) {
		pr_err("firmware is unresponsive or inconsistent and "\
				"no valid configuration is present");
		return -ENXIO;
	}

	return 0;
}

static int is_booting(void)
{
	unsigned long long t;
	unsigned long nanosec_rem;

	t = cpu_clock(smp_processor_id());
	nanosec_rem = do_div(t, 1000000000);
	return (t < 30) ? 1 : 0;
}

#ifdef CONFIG_DOWNLOAD_FEATURE
static int compare_u16_arrays(u16 *buf1, u16 *buf2, u16 n)
{
	int i;
	for (i = 0; i < n; i++) {
		if (buf1[i] != buf2[i])
			return 1;
	}
	return 0;
}
#endif

u16 calculate_checksum(u16 *buf, u16 n)
{
	u16 i, cs = 0;
	for (i = 0; i < n; i++)
		cs += buf[i];
	return cs;
}

static void check_fw_and_config(struct data *ts)
{
	const struct firmware *fw;
	u16 config_id, chip_id;
	int i, ret;

	collect_chip_data(ts);
	if ((ts->have_fw == 0 || ts->have_touchcfg == 0) &&
			PDATA(defaults_allow) == 0) {
		pr_err("firmware is unresponsive or inconsistent "\
				"and default selections are disabled");
		return;
	}
	config_id = ts->have_touchcfg ? ts->touch_config[2]
			: PDATA(default_chip_config);
	chip_id = ts->have_fw ? BYTEH(ts->fw_version[3]) : \
			PDATA(default_chip_id);

#ifdef FW_DOWNLOAD_FEATURE

	for (i = 0; i < PDATA(num_fw_mappings); i++) {
		if (PDATA(fw_mapping[i]).config_id == config_id &&
			PDATA(fw_mapping[i]).chip_id == chip_id)
			break;
	}

	if (i == PDATA(num_fw_mappings)) {
		pr_err("FW not found for configID(0x%04X) and chipID(0x%04X)",
			config_id, chip_id);
		return;
	}

	pr_info_if(8, "(INIT): Firmware file (%s)",
		PDATA(fw_mapping[i]).filename);

	ret = request_firmware(&fw, PDATA(fw_mapping[i]).filename,
					&ts->client->dev);

	if (ret || fw == NULL) {
		pr_err("firmware request failed (ret = %d, fwptr = %p)",
			ret, fw);
		return;
	}

	if (device_fw_load(ts, fw, i)) {
		release_firmware(fw);
		pr_err("firmware download failed");
		return;
	}

	release_firmware(fw);
	pr_info_if(8, "(INIT): firmware download OK");

#endif

	/* configure the chip */
#ifdef CONFIG_DOWNLOAD_FEATURE
	u16 reload_touch_config = 0, reload_calib_table = 0,
		reload_private_config = 0, reload_lookup_x = 0,
		reload_lookup_y = 0, reload_imagefactor_table = 0;
	DISABLE_IRQ();
	ret = get_touch_config(ts->client);
	if (ret < 0)
		pr_err("Failed to retrieve touch config");
	if (ret == 0) {
		ret = get_report(ts, 0x0102, 100);
		if (ret != 0)
			pr_err("Failed to receive touch config report");
		if (ret == 0) {
			if (compare_u16_arrays(&ts->rx_report[2],
				&max1187x_Touch_Configuration_Data[1], 43)
				!= 0) {
				pr_info("max1187x_Touch_Configuration_Data "\
					 "mismatch");
				reload_touch_config = 1;
			} else {
				pr_info("max1187x_Touch_Configuration_Data " \
					"okay");
			}
			release_report(ts);
		}
	}
	u16 mtpdata[] = {0x0000, 0x0000, 0x0000};
	DISABLE_IRQ();
	/*Get calibration table*/
	mtpdata[0] = 0x0011;
	ret = send_mtp_command(ts, mtpdata, 2);
	if (ret < 0)
		pr_err("Failed to retrieve calibration table");
	if (ret == 0) {
		ret = get_report(ts, 0x0111, 100);
		if (ret != 0)
			pr_err("Failed to receive calibration table report");
		if (ret == 0) {
			if (compare_u16_arrays(&ts->rx_report[2],
				&max1187x_Calibration_Table_Data[1], 51) != 0) {
				pr_info("max1187x_Calibration_Table_Data "\
				"mismatch");
				reload_calib_table = 1;
			} else {
				pr_info("max1187x_Calibration_Table_Data "\
				"okay");
			}
			release_report(ts);
		}
	}

	DISABLE_IRQ();
	/*Get private configuration*/
	mtpdata[0] = 0x0004;
	ret = send_mtp_command(ts, mtpdata, 2);
	if (ret < 0)
		pr_err("Failed to retrieve private config");
	if (ret == 0) {
		ret = get_report(ts, 0x0104, 100);
		if (ret != 0)
			pr_err("Failed to receive private config report");
		if (ret == 0) {
			if (compare_u16_arrays(&ts->rx_report[2],
				&max1187x_Private_Configuration_Data[1], 24)
				!= 0) {
				pr_info("max1187x_Private_Configuration_Data"\
					" mismatch");
				reload_private_config = 1;
			} else {
				pr_info("max1187x_Private_Configuration_Data"\
						" okay");
			}
			release_report(ts);
		}
	}

	DISABLE_IRQ();
	/*Get Lookup table X*/
	mtpdata[0] = 0x0031;
	mtpdata[1] = 0x0001;
	mtpdata[2] = 0x0000;
	ret = send_mtp_command(ts, mtpdata, 3);
	if (ret < 0)
		pr_err("Failed to retrieve Lookup table X");
	if (ret == 0) {
		ret = get_report(ts, 0x0131, 100);
		if (ret != 0)
			pr_err("Failed to receive Lookup table X report");
		if (ret == 0) {
			if (compare_u16_arrays(&ts->rx_report[3],
				&max1187x_Lookup_Table_X_Data[3], 8) != 0) {
				pr_info("max1187x_Lookup_Table_X_Data "\
				"mismatch");
				reload_lookup_x = 1;
			} else {
				pr_info("max1187x_Lookup_Table_X_Data okay");
			}
			release_report(ts);
		}
	}

	DISABLE_IRQ();
	/*Get Lookup table Y*/
	mtpdata[2] = 0x0001;
	ret = send_mtp_command(ts, mtpdata, 3);
	if (ret < 0)
		pr_err("Failed to retrieve Lookup table Y");
	if (ret == 0) {
		ret = get_report(ts, 0x0131, 100);
		if (ret != 0)
			pr_err("Failed to receive Lookup table Y report");
		if (ret == 0) {
			if (compare_u16_arrays(&ts->rx_report[3],
				&max1187x_Lookup_Table_Y_Data[3], 8) != 0) {
				pr_info("max1187x_Lookup_Table_Y_Data "\
				"mismatch");
				reload_lookup_y = 1;
			} else {
				pr_info("max1187x_Lookup_Table_Y_Data okay");
			}
			release_report(ts);
		}
	}

#ifdef MAX11871
	DISABLE_IRQ();
	/*Get Image Factor Table*/
	mtpdata[0] = 0x0047;
	mtpdata[1] = 0x0000;
	ret = send_mtp_command(ts, mtpdata, 2);
	if (ret < 0)
		pr_err("Failed to retrieve Image Factor Table");
	if (ret == 0) {
		ret = get_report(ts, 0x0147, 100);
		if (ret != 0)
			pr_err("Failed to receive Image Factor Table report");
		if (ret == 0) {
			if (ts->rx_report[3] !=
				calculate_checksum(max1187x_Image_Factor_Table,
				460)) {
				pr_info("max1187x_Image_Factor_Table "\
				"mismatch");
				reload_imagefactor_table = 1;
			} else {
				pr_info("max1187x_Image_Factor_Table okay");
			}
			release_report(ts);
		}
	}
#endif

	/*Configuration check has been done
	 /Now download correct configurations if required*/

	if (reload_touch_config) {
		DISABLE_IRQ();
		ret = send_mtp_command(ts, max1187x_Touch_Configuration_Data,
					44);
		if (ret < 0)
			pr_err("Failed to send Touch Config");
		msleep(100);
		ENABLE_IRQ();
	}
	if (reload_calib_table) {
		DISABLE_IRQ();
		ret = send_mtp_command(ts, max1187x_Calibration_Table_Data, 52);
		if (ret < 0)
			pr_err("Failed to send Calib Table");
		msleep(100);
		ENABLE_IRQ();
	}
	if (reload_private_config) {
		DISABLE_IRQ();
		ret = send_mtp_command(ts, max1187x_Private_Configuration_Data,
			25);
		if (ret < 0)
			pr_err("Failed to send Private Config");
		msleep(100);
		ENABLE_IRQ();
	}
	if (reload_lookup_x) {
		DISABLE_IRQ();
		ret = send_mtp_command(ts, max1187x_Lookup_Table_X_Data, 11);
		if (ret < 0)
			pr_err("Failed to send Lookup Table X");
		msleep(100);
		ENABLE_IRQ();
	}
	if (reload_lookup_y) {
		DISABLE_IRQ();
		ret = send_mtp_command(ts, max1187x_Lookup_Table_Y_Data, 11);
		if (ret < 0)
			pr_err("Failed to send Lookup Table Y");
		msleep(100);
		ENABLE_IRQ();
	}
#ifdef MAX11871
	if (reload_imagefactor_table) {
		DISABLE_IRQ();
		u16 imagefactor_data[104];
		/*0-59 words*/
		imagefactor_data[0] = 0x0046;
		imagefactor_data[1] = 0x003E;
		imagefactor_data[2] = 0x0000;
		memcpy(imagefactor_data+3, max1187x_Image_Factor_Table, 60<<1);
		imagefactor_data[63] = calculate_checksum(imagefactor_data+2,
				61);
		send_mtp_command(ts, imagefactor_data, 64);
		msleep(100);
		/*60-159 words*/
		imagefactor_data[0] = 0x0046;
		imagefactor_data[1] = 0x0066;
		imagefactor_data[2] = 0x003C;
		memcpy(imagefactor_data+3, max1187x_Image_Factor_Table+60,
					100<<1);
		imagefactor_data[103] = calculate_checksum(imagefactor_data+2,
					101);
		send_mtp_command(ts, imagefactor_data, 104);
		msleep(100);
		/*160-259 words*/
		imagefactor_data[0] = 0x0046;
		imagefactor_data[1] = 0x0066;
		imagefactor_data[2] = 0x00A0;
		memcpy(imagefactor_data+3, max1187x_Image_Factor_Table+160,
					100<<1);
		imagefactor_data[103] = calculate_checksum(imagefactor_data+2,
					101);
		send_mtp_command(ts, imagefactor_data, 104);
		msleep(100);
		/*260-359 words*/
		imagefactor_data[0] = 0x0046;
		imagefactor_data[1] = 0x0066;
		imagefactor_data[2] = 0x0104;
		memcpy(imagefactor_data+3, max1187x_Image_Factor_Table+260,
					100<<1);
		imagefactor_data[103] = calculate_checksum(imagefactor_data+2,
					101);
		send_mtp_command(ts, imagefactor_data, 104);
		msleep(100);
		/*360-459 words*/
		imagefactor_data[0] = 0x0046;
		imagefactor_data[1] = 0x0066;
		imagefactor_data[2] = 0x8168;
		memcpy(imagefactor_data+3, max1187x_Image_Factor_Table+360,
					100<<1);
		imagefactor_data[103] = calculate_checksum(imagefactor_data+2,
					101);
		send_mtp_command(ts, imagefactor_data, 104);
		msleep(100);
		ENABLE_IRQ();
	}
#endif
	if (reload_touch_config || reload_calib_table ||
			reload_private_config || reload_lookup_x ||
			reload_lookup_y || reload_imagefactor_table) {
		DISABLE_IRQ();
		sreset(ts->client);
	}
#endif
	ENABLE_IRQ();

	if (change_touch_rpt(ts->client, MAXIM_TOUCH_REPORT_MODE) < 0) {
		pr_err("Failed to set up touch report mode");
		return;
	}
}

/* #ifdef CONFIG_OF */
static struct max1187x_pdata *max1187x_get_platdata_dt(struct device *dev)
{
	struct max1187x_pdata *pdata = NULL;
	struct device_node *devnode = dev->of_node;
	u32 i;
	u32 datalist[MAX1187X_NUM_FW_MAPPINGS_MAX];

	if (!devnode)
		return NULL;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Failed to allocate memory for pdata\n");
		return NULL;
	}

	/* Parse gpio_tirq */
	if (of_property_read_u32(devnode, "gpio_tirq", &pdata->gpio_tirq)) {
		pr_err("Failed to get property: gpio_tirq\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse num_fw_mappings */
	if (of_property_read_u32(devnode, "num_fw_mappings",
		&pdata->num_fw_mappings)) {
		pr_err("Failed to get property: num_fw_mappings\n");
		goto err_max1187x_get_platdata_dt;
	}

	if (pdata->num_fw_mappings > MAX1187X_NUM_FW_MAPPINGS_MAX)
		pdata->num_fw_mappings = MAX1187X_NUM_FW_MAPPINGS_MAX;

	/* Parse config_id */
	if (of_property_read_u32_array(devnode, "config_id", datalist,
			pdata->num_fw_mappings)) {
		pr_err("Failed to get property: config_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].config_id = datalist[i];

	/* Parse chip_id */
	if (of_property_read_u32_array(devnode, "chip_id", datalist,
			pdata->num_fw_mappings)) {
		pr_err("Failed to get property: chip_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].chip_id = datalist[i];

	/* Parse filename */
	for (i = 0; i < pdata->num_fw_mappings; i++) {
		if (of_property_read_string_index(devnode, "filename", i,
			(const char **) &pdata->fw_mapping[i].filename)) {
				pr_err("Failed to get property: "\
					"filename[%d]\n", i);
				goto err_max1187x_get_platdata_dt;
			}
	}

	/* Parse filesize */
	if (of_property_read_u32_array(devnode, "filesize", datalist,
		pdata->num_fw_mappings)) {
		pr_err("Failed to get property: filesize\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].filesize = datalist[i];

	/* Parse file_codesize */
	if (of_property_read_u32_array(devnode, "file_codesize", datalist,
		pdata->num_fw_mappings)) {
		pr_err("Failed to get property: file_codesize\n");
		goto err_max1187x_get_platdata_dt;
	}

	for (i = 0; i < pdata->num_fw_mappings; i++)
		pdata->fw_mapping[i].file_codesize = datalist[i];

	/* Parse defaults_allow */
	if (of_property_read_u32(devnode, "defaults_allow",
		&pdata->defaults_allow)) {
		pr_err("Failed to get property: defaults_allow\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse default_chip_config */
	if (of_property_read_u32(devnode, "default_chip_config",
		&pdata->default_chip_config)) {
		pr_err("Failed to get property: default_chip_config\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse default_chip_id */
	if (of_property_read_u32(devnode, "default_chip_id",
		&pdata->default_chip_id)) {
		pr_err("Failed to get property: default_chip_id\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse i2c_words */
	if (of_property_read_u32(devnode, "i2c_words", &pdata->i2c_words)) {
		pr_err("Failed to get property: i2c_words\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse coordinate_settings */
	if (of_property_read_u32(devnode, "coordinate_settings",
		&pdata->coordinate_settings)) {
		pr_err("Failed to get property: coordinate_settings\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_margin_xl */
	if (of_property_read_u32(devnode, "panel_margin_xl",
		&pdata->panel_margin_xl)) {
		pr_err("Failed to get property: panel_margin_xl\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse lcd_x */
	if (of_property_read_u32(devnode, "lcd_x", &pdata->lcd_x)) {
		pr_err("Failed to get property: lcd_x\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_margin_xh */
	if (of_property_read_u32(devnode, "panel_margin_xh",
		&pdata->panel_margin_xh)) {
		pr_err("Failed to get property: panel_margin_xh\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_margin_yl */
	if (of_property_read_u32(devnode, "panel_margin_yl",
		&pdata->panel_margin_yl)) {
		pr_err("Failed to get property: panel_margin_yl\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse lcd_y */
	if (of_property_read_u32(devnode, "lcd_y", &pdata->lcd_y)) {
		pr_err("Failed to get property: lcd_y\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse panel_margin_yh */
	if (of_property_read_u32(devnode, "panel_margin_yh",
		&pdata->panel_margin_yh)) {
		pr_err("Failed to get property: panel_margin_yh\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse row_count */
	if (of_property_read_u32(devnode, "num_rows",
		&pdata->num_rows)) {
		pr_err("Failed to get property: num_rows\n");
		goto err_max1187x_get_platdata_dt;
	}

	/* Parse num_cols */
	if (of_property_read_u32(devnode, "num_cols",
		&pdata->num_cols)) {
		pr_err("Failed to get property: num_cols\n");
		goto err_max1187x_get_platdata_dt;
	}

	return pdata;

err_max1187x_get_platdata_dt:
	devm_kfree(dev, pdata);
	return NULL;
}
/*
#else
static inline struct max1187x_pdata *
	max1187x_get_platdata_dt(struct device *dev)
{
	return NULL;
}
#endif
*/

static int validate_pdata(struct max1187x_pdata *pdata)
{
	if (pdata == NULL) {
		pr_err("Platform data not found!\n");
		goto err_validate_pdata;
	}

	if (pdata->gpio_tirq == 0) {
		pr_err("gpio_tirq (%u) not defined!\n", pdata->gpio_tirq);
		goto err_validate_pdata;
	}

	if (pdata->lcd_x < 480 || pdata->lcd_x > 0x7FFF) {
		pr_err("lcd_x (%u) out of range!\n", pdata->lcd_x);
		goto err_validate_pdata;
	}

	if (pdata->lcd_y < 240 || pdata->lcd_y > 0x7FFF) {
		pr_err("lcd_y (%u) out of range!\n", pdata->lcd_y);
		goto err_validate_pdata;
	}

	if (pdata->num_rows == 0 || pdata->num_rows > 40) {
		pr_err("num_rows (%u) out of range!\n", pdata->num_rows);
		goto err_validate_pdata;
	}

	if (pdata->num_cols == 0 || pdata->num_cols > 40) {
		pr_err("num_cols (%u) out of range!\n", pdata->num_cols);
		goto err_validate_pdata;
	}

	return 0;

err_validate_pdata:
	return -ENXIO;
}

static int max1187x_chip_init(struct max1187x_pdata *pdata, int value)
{
	int  ret;

	if (value) {
		ret = gpio_request(pdata->gpio_tirq, "max1187x_tirq");
		if (ret) {
			pr_err("GPIO request failed for max1187x_tirq (%d)\n",
				pdata->gpio_tirq);
			return -EIO;
		}
		ret = gpio_direction_input(pdata->gpio_tirq);
		if (ret) {
			pr_err("GPIO set input direction failed for "\
				"max1187x_tirq (%d)\n", pdata->gpio_tirq);
			gpio_free(pdata->gpio_tirq);
			return -EIO;
		}
	} else {
		gpio_free(pdata->gpio_tirq);
	}

	return 0;
}

static int device_init_thread(void *arg)
{
	return device_init((struct i2c_client *) arg);
}

static int device_init(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct data *ts = NULL;
	struct max1187x_pdata *pdata = NULL;
	struct device_attribute **dev_attr = dev_attrs;
	int ret = 0;

	init_state = 1;
	dev_info(dev, "(INIT): Start");

	/* if I2C functionality is not present we are done */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("I2C core driver does not support I2C functionality");
		ret = -ENXIO;
		goto err_device_init;
	}
	pr_info_if(8, "(INIT): I2C functionality OK");

	/* allocate control block; nothing more to do if we can't */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		pr_err("Failed to allocate control block memory");
		ret = -ENOMEM;
		goto err_device_init;
	}

	/* Get platform data */
#ifdef MAX1187X_LOCAL_PDATA
	pdata = &local_pdata;
	if (!pdata) {
		pr_err("Platform data is missing");
		ret = -ENXIO;
		goto err_device_init_pdata;
	}
#else
	pdata = dev_get_platdata(dev);
	/* If pdata is missing, try to get pdata from device tree (dts) */
	if (!pdata)
		pdata = max1187x_get_platdata_dt(dev);

	/* Validate if pdata values are okay */
	ret = validate_pdata(pdata);
	if (ret < 0)
		goto err_device_init_pdata;
	pr_info_if(8, "(INIT): Platform data OK");
#endif

	ts->pdata = pdata;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	mutex_init(&ts->irq_mutex);
	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->report_mutex);
	sema_init(&ts->report_sem, 1);
	ts->fw_index = -1;

	/* Create singlethread workqueue */
	ts->wq = create_singlethread_workqueue("max1187x_wq");
	if (ts->wq == NULL) {
		pr_err("Not able to create workqueue\n");
		ret = -ENOMEM;
		goto err_device_init_memalloc;
	}
	INIT_WORK(&ts->work_irq, max1187x_wfxn_irq);
	atomic_set(&ts->scheduled_work_irq, 0);

	pr_info_if(8, "(INIT): Memory allocation OK");

	/* Initialize GPIO pins */
	if (max1187x_chip_init(ts->pdata, 1) < 0) {
		ret = -EIO;
		goto err_device_init_gpio;
	}
	pr_info_if(8, "(INIT): chip init OK");

	/* Setup IRQ and handler */
	if (request_irq(client->irq, irq_handler,
				IRQF_TRIGGER_FALLING, client->name, ts) != 0) {
			pr_err("Failed to setup IRQ handler");
			ret = -EIO;
			goto err_device_init_gpio;
	}
	pr_info_if(8, "(INIT): IRQ handler OK");

	/* collect controller ID and configuration ID data from firmware   */
	/* and perform firmware comparison/download if we have valid image */
	check_fw_and_config(ts);

	/* allocate and register touch device */
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		pr_err("Failed to allocate touch input device");
		ret = -ENOMEM;
		goto err_device_init_alloc_inputdev;
	}
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0",
			dev_name(dev));
	ts->input_dev->name = MAX1187X_TOUCH;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);
#ifdef MAX1187X_PROTOCOL_A
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
#else
	input_mt_init_slots(ts->input_dev, MAX1187X_TOUCH_COUNT_MAX);
#endif
	ts->list_finger_ids = 0;
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			PDATA(panel_margin_xl),
			PDATA(panel_margin_xl) + PDATA(lcd_x), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			PDATA(panel_margin_yl),
			PDATA(panel_margin_yl) + PDATA(lcd_y), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#if MAXIM_TOUCH_REPORT_MODE == 2
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			0, max(PDATA(lcd_x), PDATA(lcd_x)), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR,
			0, min(PDATA(lcd_x), PDATA(lcd_x)), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_ORIENTATION, -90, 90, 0, 0);
#endif
	if (input_register_device(ts->input_dev)) {
		pr_err("Failed to register touch input device");
		ret = -EPERM;
		goto err_device_init_register_inputdev;
	}
	pr_info_if(8, "(INIT): Input touch device OK");

	/* configure suspend/resume */
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = early_suspend;
	ts->early_suspend.resume = late_resume;
	register_early_suspend(&ts->early_suspend);
	ts->early_suspend_registered = 1;
	pr_info_if(8, "(INIT): suspend/resume registration OK");

	/* set up debug interface */
	while (*dev_attr) {
		if (device_create_file(&client->dev, *dev_attr) < 0) {
			pr_err("failed to create sysfs file");
			return 0;
		}
		ts->sysfs_created++;
		dev_attr++;
	}

	if (device_create_bin_file(&client->dev, &dev_attr_report) < 0) {
		pr_err("failed to create sysfs file [report]");
		return 0;
	}
	ts->sysfs_created++;

	pr_info("(INIT): Done\n");
	return 0;

err_device_init_register_inputdev:
	input_free_device(ts->input_dev);
	ts->input_dev = NULL;
err_device_init_alloc_inputdev:
err_device_init_gpio:
err_device_init_memalloc:
err_device_init_pdata:
	kfree(ts);
err_device_init:
	return ret;
}

static int device_deinit(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	struct max1187x_pdata *pdata = ts->pdata;
	struct device_attribute **dev_attr = dev_attrs;

	if (ts == NULL)
		return 0;

	propagate_report(ts, -1, NULL);

	init_state = 0;
	while (*dev_attr) {
		if (ts->sysfs_created && ts->sysfs_created--)
			device_remove_file(&client->dev, *dev_attr);
		dev_attr++;
	}
	if (ts->sysfs_created && ts->sysfs_created--)
		device_remove_bin_file(&client->dev, &dev_attr_report);

	if (ts->early_suspend_registered)
		unregister_early_suspend(&ts->early_suspend);
	if (ts->input_dev)
		input_unregister_device(ts->input_dev);

	if (client->irq)
		free_irq(client->irq, ts);
	(void) max1187x_chip_init(pdata, 0);
	kfree(ts);

	pr_info("(INIT): Deinitialized\n");
	return 0;
}

static int probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (device_create_file(&client->dev, &dev_attr_init) < 0) {
		pr_err("failed to create sysfs file [init]");
		return 0;
	}

	if (!is_booting())
		return device_init(client);
	if (IS_ERR(kthread_run(device_init_thread, (void *) client,
			MAX1187X_NAME))) {
		pr_err("failed to start kernel thread");
		return -EAGAIN;
	}
	return 0;
}

static int remove(struct i2c_client *client)
{
	int ret = device_deinit(client);

	device_remove_file(&client->dev, &dev_attr_init);
	return ret;
}

/*
 COMMANDS
 */
static int sreset(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = { 0x00E9, 0x0000 };
	return send_mtp_command(ts, data, NWORDS(data));
}

static int get_touch_config(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = { 0x0002, 0x0000 };
	return send_mtp_command(ts, data, NWORDS(data));
}

static int get_fw_version(struct i2c_client *client)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = { 0x0040, 0x0000 };
	return send_mtp_command(ts, data, NWORDS(data));
}

static int change_touch_rpt(struct i2c_client *client, u16 to)
{
	struct data *ts = i2c_get_clientdata(client);
	u16 data[] = { 0x0018, 0x0001, to & 0x0003 };
	return send_mtp_command(ts, data, NWORDS(data));
}

static int combine_multipacketreport(struct data *ts, u16 *report)
{
	u16 packet_header = report[0];
	u8 packet_seq_num = BYTEH(packet_header);
	u8 packet_size = BYTEL(packet_header);
	u16 total_packets, this_packet_num, offset;
	static u16 packet_seq_combined;

	if (packet_seq_num == 0x11) {
		memcpy(ts->rx_report, report, (packet_size + 1) << 1);
		ts->rx_report_len = packet_size;
		packet_seq_combined = 1;
		return 0;
	}

	total_packets = (packet_seq_num & 0xF0) >> 4;
	this_packet_num = packet_seq_num & 0x0F;

	if (this_packet_num == 1) {
		if (report[1] == 0x0800) {
			ts->rx_report_len = report[2] + 2;
			packet_seq_combined = 1;
			memcpy(ts->rx_report, report, (packet_size + 1) << 1);
			return -EAGAIN;
		} else {
			return -EIO;
		}
	} else if (this_packet_num == packet_seq_combined + 1) {
		packet_seq_combined++;
		offset = (this_packet_num - 1) * 0xF4 + 1;
		memcpy(ts->rx_report + offset, report + 1, packet_size << 1);
		if (total_packets == this_packet_num)
			return 0;
		else
			return -EIO;
	}
	return -EIO;
}

static void propagate_report(struct data *ts, int status, u16 *report)
{
	int i, ret;

	down(&ts->report_sem);
	mutex_lock(&ts->report_mutex);

	if (report) {
		ret = combine_multipacketreport(ts, report);
		if (ret) {
			up(&ts->report_sem);
			mutex_unlock(&ts->report_mutex);
			return;
		}
	}

	for (i = 0; i < MAX_REPORT_READERS; i++) {
		if (status == 0) {
			if (ts->report_readers[i].report_id == 0xFFFF
				|| (ts->rx_report[1] != 0
				&& ts->report_readers[i].report_id
				== ts->rx_report[1])) {
				up(&ts->report_readers[i].sem);
				ts->report_readers[i].reports_passed++;
				ts->report_readers_outstanding++;
			}
		} else {
			if (ts->report_readers[i].report_id != 0) {
				ts->report_readers[i].status = status;
				up(&ts->report_readers[i].sem);
			}
		}
	}
	if (ts->report_readers_outstanding == 0)
		up(&ts->report_sem);
	mutex_unlock(&ts->report_mutex);
}

static int get_report(struct data *ts, u16 report_id, ulong timeout)
{
	int i, ret, status;

	mutex_lock(&ts->report_mutex);
	for (i = 0; i < MAX_REPORT_READERS; i++)
		if (ts->report_readers[i].report_id == 0)
			break;
	if (i == MAX_REPORT_READERS) {
		mutex_unlock(&ts->report_mutex);
		ENABLE_IRQ();
		pr_err("maximum readers reached");
		return -EBUSY;
	}
	ts->report_readers[i].report_id = report_id;
	sema_init(&ts->report_readers[i].sem, 1);
	down(&ts->report_readers[i].sem);
	ts->report_readers[i].status = 0;
	ts->report_readers[i].reports_passed = 0;
	mutex_unlock(&ts->report_mutex);
	ENABLE_IRQ();

	if (timeout == 0xFFFFFFFF)
		ret = down_interruptible(&ts->report_readers[i].sem);
	else
		ret = down_timeout(&ts->report_readers[i].sem,
			(timeout * HZ) / 1000);

	mutex_lock(&ts->report_mutex);
	if (ret && ts->report_readers[i].reports_passed > 0)
		if (--ts->report_readers_outstanding == 0)
			up(&ts->report_sem);
	status = ts->report_readers[i].status;
	ts->report_readers[i].report_id = 0;
	mutex_unlock(&ts->report_mutex);

	return (status == 0) ? ret : status;
}

static void release_report(struct data *ts)
{
	mutex_lock(&ts->report_mutex);
	if (--ts->report_readers_outstanding == 0)
		up(&ts->report_sem);
	mutex_unlock(&ts->report_mutex);
}

static void early_suspend(struct early_suspend *h)
{
	u16 data[] = {0x0020, 0x0001, 0x0000};
	struct data *ts;
	ts = container_of(h, struct data, early_suspend);

	DISABLE_IRQ();
	(void)send_mtp_command(ts, data, NWORDS(data));
	ENABLE_IRQ();
}

static void late_resume(struct early_suspend *h)
{
	u16 data[] = {0x0020, 0x0001, 0x0002};
	struct data *ts;
	ts = container_of(h, struct data, early_suspend);

	/* previous_fingers = current_fingers = 0; */
	(void)send_mtp_command(ts, data, NWORDS(data));

	(void)change_touch_rpt(ts->client, MAXIM_TOUCH_REPORT_MODE);
}

#define STATUS_ADDR_H 0x00
#define STATUS_ADDR_L 0xFF
#define DATA_ADDR_H   0x00
#define DATA_ADDR_L   0xFE
#define STATUS_READY_H 0xAB
#define STATUS_READY_L 0xCC
#define RXTX_COMPLETE_H 0x54
#define RXTX_COMPLETE_L 0x32
static int bootloader_read_status_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[] = { STATUS_ADDR_L, STATUS_ADDR_H }, i;

	for (i = 0; i < 3; i++) {
		if (i2c_tx_bytes(ts, buffer, 2) != 2) {
			pr_err("TX fail");
			return -EIO;
		}
		if (i2c_rx_bytes(ts, buffer, 2) != 2) {
			pr_err("RX fail");
			return -EIO;
		}
		if (buffer[0] == byteL && buffer[1] == byteH)
			break;
	}
	if (i == 3) {
		pr_err("Unexpected status => %02X%02X vs %02X%02X",
				buffer[0], buffer[1], byteL, byteH);
		return -EIO;
	}

	return 0;
}

static int bootloader_write_status_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[] = { STATUS_ADDR_L, STATUS_ADDR_H, byteL, byteH };

	if (i2c_tx_bytes(ts, buffer, 4) != 4) {
		pr_err("TX fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_rxtx_complete(struct data *ts)
{
	return bootloader_write_status_reg(ts, RXTX_COMPLETE_L,
				RXTX_COMPLETE_H);
}

static int bootloader_read_data_reg(struct data *ts, u8 *byteL, u8 *byteH)
{
	u8 buffer[] = { DATA_ADDR_L, DATA_ADDR_H, 0x00, 0x00 };

	if (i2c_tx_bytes(ts, buffer, 2) != 2) {
		pr_err("TX fail");
		return -EIO;
	}
	if (i2c_rx_bytes(ts, buffer, 4) != 4) {
		pr_err("RX fail");
		return -EIO;
	}
	if (buffer[2] != 0xCC && buffer[3] != 0xAB) {
		pr_err("Status is not ready");
		return -EIO;
	}

	*byteL = buffer[0];
	*byteH = buffer[1];
	return bootloader_rxtx_complete(ts);
}

static int bootloader_write_data_reg(struct data *ts, const u8 byteL,
	const u8 byteH)
{
	u8 buffer[6] = { DATA_ADDR_L, DATA_ADDR_H, byteL, byteH,
			RXTX_COMPLETE_L, RXTX_COMPLETE_H };

	if (bootloader_read_status_reg(ts, STATUS_READY_L,
		STATUS_READY_H) < 0) {
		pr_err("read status register fail");
		return -EIO;
	}
	if (i2c_tx_bytes(ts, buffer, 6) != 6) {
		pr_err("TX fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_rxtx(struct data *ts, u8 *byteL, u8 *byteH,
	const int tx)
{
	if (tx > 0) {
		if (bootloader_write_data_reg(ts, *byteL, *byteH) < 0) {
			pr_err("write data register fail");
			return -EIO;
		}
		return 0;
	}

	if (bootloader_read_data_reg(ts, byteL, byteH) < 0) {
		pr_err("read data register fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_get_cmd_conf(struct data *ts, int retries)
{
	u8 byteL, byteH;

	do {
		if (bootloader_read_data_reg(ts, &byteL, &byteH) >= 0) {
			if (byteH == 0x00 && byteL == 0x3E)
				return 0;
		}
		retries--;
	} while (retries > 0);

	return -EIO;
}

static int bootloader_write_buffer(struct data *ts, u8 *buffer, int size)
{
	u8 byteH = 0x00;
	int k;

	for (k = 0; k < size; k++) {
		if (bootloader_rxtx(ts, &buffer[k], &byteH, 1) < 0) {
			pr_err("bootloader RX-TX fail");
			return -EIO;
		}
	}
	return 0;
}

static int bootloader_enter(struct data *ts)
{
	int i;
	u16 enter[3][2] = { { 0x7F00, 0x0047 }, { 0x7F00, 0x00C7 }, { 0x7F00,
			0x0007 } };

	DISABLE_IRQ();
	for (i = 0; i < 3; i++) {
		if (i2c_tx_words(ts, enter[i], 2) != 2) {
			ENABLE_IRQ();
			pr_err("Failed to enter bootloader");
			return -EIO;
		}
	}

	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		ENABLE_IRQ();
		pr_err("Failed to enter bootloader mode");
		return -EIO;
	}
	bootloader = 1;
	return 0;
}

static int bootloader_exit(struct data *ts)
{
	u16 exit[] = { 0x00FE, 0x0001, 0x5432 };

	bootloader = 0;
	ts->got_report = 0;
	if (i2c_tx_words(ts, exit, NWORDS(exit)) != NWORDS(exit)) {
		pr_err("Failed to exit bootloader");
		return -EIO;
	}
	return 0;
}

static int bootloader_get_crc(struct data *ts, u16 *crc16,
		u16 addr, u16 len, u16 delay)
{
	u8 crc_command[] = {0x30, 0x02, BYTEL(addr),
			BYTEH(addr), BYTEL(len), BYTEH(len)};
	u8 byteL = 0, byteH = 0;
	u16 rx_crc16 = 0;

	if (bootloader_write_buffer(ts, crc_command, 6) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}
	msleep(delay);

	/* reads low 8bits (crcL) */
	if (bootloader_rxtx(ts, &byteL, &byteH, 0) < 0) {
		pr_err("Failed to read low byte of crc response!");
		return -EIO;
	}
	rx_crc16 = (u16) byteL;

	/* reads high 8bits (crcH) */
	if (bootloader_rxtx(ts, &byteL, &byteH, 0) < 0) {
		pr_err("Failed to read high byte of crc response!");
		return -EIO;
	}
	rx_crc16 = (u16)(byteL << 8) | rx_crc16;

	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		pr_err("CRC get failed!");
		return -EIO;
	}
	*crc16 = rx_crc16;

	return 0;
}

static int bootloader_set_byte_mode(struct data *ts)
{
	u8 buffer[2] = { 0x0A, 0x00 };

	if (bootloader_write_buffer(ts, buffer, 2) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}
	if (bootloader_get_cmd_conf(ts, 10) < 0) {
		pr_err("command confirm fail");
		return -EIO;
	}
	return 0;
}

static int bootloader_erase_flash(struct data *ts)
{
	u8 byteL = 0x02, byteH = 0x00;
	int i, verify = 0;

	if (bootloader_rxtx(ts, &byteL, &byteH, 1) < 0) {
		pr_err("bootloader RX-TX fail");
		return -EIO;
	}

	for (i = 0; i < 10; i++) {
		msleep(60); /* wait 60ms */

		if (bootloader_get_cmd_conf(ts, 0) < 0)
			continue;

		verify = 1;
		break;
	}

	if (verify != 1) {
		pr_err("Flash Erase failed");
		return -EIO;
	}

	return 0;
}

static int bootloader_write_flash(struct data *ts, const u8 *image, u16 length)
{
	u8 buffer[130];
	u8 length_L = length & 0xFF;
	u8 length_H = (length >> 8) & 0xFF;
	u8 command[] = { 0xF0, 0x00, length_H, length_L, 0x00 };
	u16 blocks_of_128bytes;
	int i, j;

	if (bootloader_write_buffer(ts, command, 5) < 0) {
		pr_err("write buffer fail");
		return -EIO;
	}

	blocks_of_128bytes = length >> 7;

	for (i = 0; i < blocks_of_128bytes; i++) {
		for (j = 0; j < 100; j++) {
			usleep_range(1500, 2000);
			if (bootloader_read_status_reg(ts, STATUS_READY_L,
			STATUS_READY_H)	== 0)
				break;
		}
		if (j == 100) {
			pr_err("Failed to read Status register!");
			return -EIO;
		}

		buffer[0] = ((i % 2) == 0) ? 0x00 : 0x40;
		buffer[1] = 0x00;
		memcpy(buffer + 2, image + i * 128, 128);

		if (i2c_tx_bytes(ts, buffer, 130) != 130) {
			pr_err("Failed to write data (%d)", i);
			return -EIO;
		}
		if (bootloader_rxtx_complete(ts) < 0) {
			pr_err("Transfer failure (%d)", i);
			return -EIO;
		}
	}

	usleep_range(10000, 11000);
	if (bootloader_get_cmd_conf(ts, 5) < 0) {
		pr_err("Flash programming failed");
		return -EIO;
	}
	return 0;
}

/****************************************
 *
 * Standard Driver Structures/Functions
 *
 ****************************************/
static const struct i2c_device_id id[] = { { MAX1187X_NAME, 0 }, { } };

MODULE_DEVICE_TABLE(i2c, id);

static struct of_device_id max1187x_dt_match[] = {
	{ .compatible = "maxim,max1187x_tsc" },	{ } };

static struct i2c_driver driver = {
		.probe = probe,
		.remove = remove,
		.id_table = id,
		.driver = {
			.name = MAX1187X_NAME,
			.owner	= THIS_MODULE,
			.of_match_table = max1187x_dt_match,
		},
};

static int __devinit max1187x_init(void)
{
	return i2c_add_driver(&driver);
}

static void __exit max1187x_exit(void)
{
	i2c_del_driver(&driver);
}

module_init(max1187x_init);
module_exit(max1187x_exit);

MODULE_AUTHOR("Maxim Integrated Products, Inc.");
MODULE_DESCRIPTION("MAX1187X Touchscreen Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("3.0.7");
