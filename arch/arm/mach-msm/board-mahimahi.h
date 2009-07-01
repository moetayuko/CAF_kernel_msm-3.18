/* arch/arm/mach-msm/board-mahimahi.h
 *
 * Copyright (C) 2009 HTC Corporation.
 * Author: Haley Teng <Haley_Teng@htc.com>
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_MAHIMAHI_H
#define __ARCH_ARM_MACH_MSM_BOARD_MAHIMAHI_H

#include <mach/board.h>

#define MSM_SMI_BASE		0x02B00000
#define MSM_SMI_SIZE		0x01500000

#define MSM_PMEM_MDP_BASE	0x03000000
#define MSM_PMEM_MDP_SIZE	0x01000000

#define MSM_EBI1_BASE		0x12000000
#define MSM_EBI1_SIZE		0x0E000000

#define MSM_PMEM_GPU0_BASE	0x1E400000
#define MSM_PMEM_GPU0_SIZE	0x01000000

#define MSM_PMEM_GPU1_BASE	0x1F400000
#define MSM_PMEM_GPU1_SIZE	0x00800000

#define MSM_GPU_MEM_BASE	0x1FC00000
#define MSM_GPU_MEM_SIZE	0x00200000

#define MSM_FB_BASE		0x1FE00000
#define MSM_FB_SIZE		0x00200000

#define MAHIMAHI_GPIO_UP_INT_N		35

#define MAHIMAHI_GPIO_TP_INT_N		92
#define MAHIMAHI_GPIO_TP_LS_EN		93
#define MAHIMAHI_GPIO_TP_EN		160

#define MAHIMAHI_GPIO_POWER_KEY		94
#define MAHIMAHI_GPIO_SDMC_CD_N		153

#define MAHIMAHI_GPIO_BT_UART1_RTS	43
#define MAHIMAHI_GPIO_BT_UART1_CTS	44
#define MAHIMAHI_GPIO_BT_UART1_RX	45
#define MAHIMAHI_GPIO_BT_UART1_TX	46
#define MAHIMAHI_GPIO_BT_RESET_N	146
#define MAHIMAHI_GPIO_BT_SHUTDOWN_N	128

#endif /* __ARCH_ARM_MACH_MSM_BOARD_MAHIMAHI_H */
