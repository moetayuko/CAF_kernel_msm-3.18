
/* arch/arm/mach-msm/include/mach/msm_rpc_versions.h
 *
 * Copyright (C) 2008 Google, Inc.
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

#ifndef __ARCH_ARM_MACH_MSM_RPC_VERSIONS_H__
#define __ARCH_ARM_MACH_MSM_RPC_VERSIONS_H__

#define RPC_SND_PROG		0x30000002
#define RPC_SND_CB_PROG		0x31000002
#define PM_LIBPROG		0x30000061
#define AUDMGR_PROG		0x30000013
#define AUDMGR_CB_PROG		0x31000013
#define DOG_KEEPALIVE_PROG	0x30000015
#define TIME_REMOTE_MTOA_PROG	0x3000005d
#define HSUSB_API_PROG		0x30000064
#define RPC_ADSP_RTOS_ATOM_PROG	0x3000000a
#define RPC_ADSP_RTOS_MTOA_PROG	0x3000000b
#define APP_BATT_PROG		0x30100001
#define BATT_MTOA_PROG		0x30100000

/* ----------------------------------------------------------------------------
 * AMSS 6225
 * --------------------------------------------------------------------------*/

#if CONFIG_MSM_AMSS_VERSION == 6225

#define PM_LIBVERS			0xfb837d0b
#define RPC_ADSP_RTOS_ATOM_VERS		0x71d1094b
#define RPC_ADSP_RTOS_MTOA_VERS		0xee3a9966
#define AUDMGR_VERS			0xe94e8f0c
#define AUDMGR_CB_VERS			0x21570ba7
#define RPC_SND_VERS			0xaa2b1a44
#define DOG_KEEPALIVE_VERS		0x731fa727
#define TIME_REMOTE_MTOA_VERS		0x9202a8e4
#define HSUSB_API_VERS			0x00010001
#define APP_BATT_VER                    0
#define BATT_MTOA_VERS                  0

#define MSM_ADSP_DRIVER_NAME		"rs3000000a:71d1094b"
#define APP_TIMEREMOTE_PDEV_NAME	"rs30000048:0da5b528"
#define APP_BATT_PDEV_NAME		"rs30100001:00000000"

/* ----------------------------------------------------------------------------
 * AMSS 6350
 * --------------------------------------------------------------------------*/

#elif CONFIG_MSM_AMSS_VERSION == 6350

#define PM_LIBVERS			0x00010001
#define RPC_ADSP_RTOS_ATOM_VERS		0x00010000
#define RPC_ADSP_RTOS_MTOA_VERS		0x00020001 /* must be actual vers */
#define AUDMGR_VERS			0x00010000
#define AUDMGR_CB_VERS			0xf8e3e2d9
#define RPC_SND_VERS			0x00010000
#define DOG_KEEPALIVE_VERS		0x00010000
#define TIME_REMOTE_MTOA_VERS		0x00010000
#define HSUSB_API_VERS			0x00010001
#define APP_BATT_VER                    0
#define BATT_MTOA_VERS                  0

#define MSM_ADSP_DRIVER_NAME		"rs3000000a:00010000"
#define APP_TIMEREMOTE_PDEV_NAME	"rs30000048:00010000"
#define APP_BATT_PDEV_NAME		"rs30100001:00000000"

#endif

#endif/* __ARCH_ARM_MACH_MSM_RPC_VERSIONS_H__ */
