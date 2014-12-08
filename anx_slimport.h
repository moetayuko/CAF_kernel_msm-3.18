
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
#ifndef __ANX_SLIMPORT_H__
#define __ANX_SLIMPORT_H__
#include <linux/notifier.h>


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
int request_charge_voltage(int voltage);

/*return: 5/9/12/20 v*/
int get_current_slimport_charge_voltage(void);

/*Set PMIC real charge voltage to slimport module
parameter: 5/9/12/20 v*/
void set_pmic_charge_voltage(int voltage);

/*Get EDID block*/
int slimport_read_edid_block(int block, uint8_t *edid_buf);

/*Get link bw*/
unsigned char sp_get_link_bw(void);


bool slimport_is_connected(void);


bool is_slimport_vga(void);

bool is_slimport_dp(void);



#endif
