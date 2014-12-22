/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ANX_SLIMPORT_H
#define _ANX_SLIMPORT_H

#include <linux/types.h>
#include <linux/notifier.h>

void sp_set_link_bw(unchar link_bw);

bool slimport_is_connected(void);

int slimport_read_edid_block(int block, uint8_t *edid_buf);

bool is_slimport_dp(void);

bool is_slimport_vga(void);

unchar sp_get_link_bw(void);


/*for quick charge*/

/*Used for get slimport charger status change.
If you want to get charger status change for example:
You want to know the event charger plugged and unplugged.
You need call these 2 functions.
How to use:
Firstly call register_slimport_charge_status(struct notifier_block *nb) in your
initialization function.

When you don't need this function, you can call
unregister_slimport_charge_status(struct notifier_block *nb)
to unregister.

An example code may be helped.

static notifier_block slimport_notifier_block;

static int slimport_charger_status_event(struct notifier_block *nb,
		unsigned long event, void *data)
{
	switch (event) {
	case 0:
		//charger is unplugged
		break;
	case 1:
		//charger is plugged.
		break;
	}

	return 0;
}

In an init function call:
slimport_notifier_block.notifier_call = slimport_charger_status_event;
register_slimport_charge_status(&slimport_notifier_block);

When no need this, call
unregister_slimport_charge_status(&slimport_notifier_block); to release.
*/
int register_slimport_charge_status(struct notifier_block *nb);
int unregister_slimport_charge_status(struct notifier_block *nb);


/*Get slimport maximum voltage
return: 5/9/12/20 v*/
int get_slimport_max_charge_voltage(void);

/*
 return value:
 1:sdp
 2:cdp
 3:dcp
 4:ACA dock
 5:ACA_A
 6:ACA_B
 7:ACA_C
 8:other device
 9:HVDCP
*/
int get_slimport_charger_type(void);


/*request a charge voltage from slimport
  param: 5/9/12/20
  return: 1: fail. 2: success
*/
int request_slimport_charge_voltage(int voltage);

/*
return: 2, yes, ack back. others, no ack.
This function should be called after calling
int request_slimport_charge_voltage(int voltage);
for checking slimport charge request success or fail.
*/
int is_request_ack_back(void);

#endif
