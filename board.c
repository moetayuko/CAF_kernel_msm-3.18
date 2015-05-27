/* board.c
 *
 * Copyright (c)2013 Maxim Integrated Products, Inc.
 *
 * Driver Version: 3.0.7
 * Release Date: Jan 22, 2013
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

#if defined(CONFIG_TOUCHSCREEN_MAX1187X) || defined(CONFIG_TOUCHSCREEN_MAX1187X_MODULE)

#include <linux/input/max1187x.h>

#define BOARD_TP_TIRQ 61

struct max1187x_pdata max1187x_platdata = {
		.gpio_tirq  = BOARD_TP_TIRQ,
		.num_fw_mappings = 2,
		.fw_mapping[0] = {.config_id = 0x0CFD, .chip_id = 0x72, .filename = "max11876.bin", .filesize = 0xC000, .file_codesize = 0xC000},
		.fw_mapping[1] = {.config_id = 0x0CFD, .chip_id = 0x74, .filename = "max11876.bin", .filesize = 0xC000, .file_codesize = 0xC000},
		.defaults_allow = 1,
		.default_chip_config = 0x0CFD,
		.default_chip_id = 0x74,
		//.i2c_words = MAX_WORDS_REPORT,
		.i2c_words = 128,
		//.coordinate_settings = MAX1187X_REVERSE_Y | MAX1187X_SWAP_XY,
		.coordinate_settings = 0,
		.panel_margin_xl = 0,
		.lcd_x = 1300,
		.panel_margin_xh = 0,
		.panel_margin_yl = 0,
		.lcd_y = 700,
		.panel_margin_yh = 0,
		.num_rows = 32,
		.num_cols = 18
};

static struct i2c_board_info __initdata panda_i2c4_boardinfo[] = {
	{
		I2C_BOARD_INFO(MAX1187X_NAME, 0x48),
		.platform_data = &max1187x_platdata,
		.irq = OMAP_GPIO_IRQ(BOARD_TP_TIRQ),
	},
};

#endif
