/* include/linux/ds2784_battery.h
 *
 * Copyright (C) 2009 HTC Corporation
 * Copyright (C) 2009 Google, Inc.
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

#ifndef __LINUX_DS2784_BATTERY_H
#define __LINUX_DS2784_BATTERY_H

#ifdef __KERNEL__

#define CHARGE_OFF	0
#define CHARGE_SLOW	1
#define CHARGE_FAST	2

struct ds2784_platform_data {
	int (*charge)(int how);
};

#endif /* __KERNEL__ */

#endif /* __LINUX_DS2784_BATTERY_H */
