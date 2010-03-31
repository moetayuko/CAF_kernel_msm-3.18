/* Header file for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * include/linux/cyttsp-i2c.h
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */


#ifndef __CYTTSP_I2C_H__
#define __CYTTSP_I2C_H__

#define CYTTSP_USE_I2C

#include <linux/i2c.h>
#include <linux/cyttsp.h>

#define CYTTSP_I2C_NAME		"cyttsp-i2c"

/* CY TTSP SPI Driver private data */
struct cyttsp {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct timer_list timer;
	struct mutex mutex;
	char phys[32];
	struct cyttsp_platform_data *platform_data;
	u8 num_prev_st_touch;
	u16 active_track[CYTTSP_NUM_TRACK_ID];
	u16 prev_mt_touch[CYTTSP_NUM_MT_TOUCH_ID];
	u16 prev_st_touch[CYTTSP_NUM_ST_TOUCH_ID];
};

#endif /* __CYTTSP_I2C_H__ */
