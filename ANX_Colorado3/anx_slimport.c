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
#include <video/anx_slimport.h>
#include "quick_charge.h"
#include "slimport.h"

static struct blocking_notifier_head charger_notifier_head;

struct blocking_notifier_head *get_notifier_list_head(void)
{
	return &charger_notifier_head;
}

EXPORT_SYMBOL(get_notifier_list_head);


int register_slimport_charge_status(struct notifier_block *nb)
{

	return blocking_notifier_chain_register(&charger_notifier_head, nb);

}
int unregister_slimport_charge_status(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&charger_notifier_head, nb);
}

int get_slimport_max_charge_voltage(void)
{
	return get_dongle_capability();
}

int get_slimport_charger_type(void)
{
	return get_input_charger_type();

}

int request_charge_voltage(int voltage)
{
	return set_request_voltage(voltage);
}

int get_current_slimport_charge_voltage(void)
{
	return get_current_voltage();
}

void set_pmic_charge_voltage(int voltage)
{
	set_pmic_voltage(voltage);
}

/*Get EDID block*/
int slimport_read_edid_block(int block, uint8_t *edid_buf)
{

	return slimport_get_edid_block(block, edid_buf);

}

/*Get link bw*/
unsigned char sp_get_link_bw(void)
{
	return sp_get_slimport_link_bw();

}


bool slimport_is_connected(void)
{
	return slimport_dongle_is_connected();

}


bool is_slimport_vga(void)
{
	return is_anx_slimport_vga();

}

bool is_slimport_dp(void)
{
	return is_anx_slimport_dp();

}




