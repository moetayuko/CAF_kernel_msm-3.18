/* linux/arch/arm/mach-msm/board-mahimahi-mmc.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>

#include <mach/vreg.h>

#include "board-mahimahi.h"
#include "devices.h"
#include "proc_comm.h"

static bool opt_disable_sdcard;
static int __init mahimahi_disablesdcard_setup(char *str)
{
	opt_disable_sdcard = (bool)simple_strtol(str, NULL, 0);
	return 1;
}

__setup("board_mahimahi.disable_sdcard=", mahimahi_disablesdcard_setup);

static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* CLK */
	PCOM_GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* CMD */
	PCOM_GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(64, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static struct vreg	*sdslot_vreg;
static uint32_t		sdslot_vdd = 0xffffffff;
static uint32_t		sdslot_vreg_enabled;

static struct {
	int mask;
	int level;
} mmc_vdd_table[] = {
	{ MMC_VDD_165_195,	1800 },
	{ MMC_VDD_20_21,	2050 },
	{ MMC_VDD_21_22,	2150 },
	{ MMC_VDD_22_23,	2250 },
	{ MMC_VDD_23_24,	2350 },
	{ MMC_VDD_24_25,	2450 },
	{ MMC_VDD_25_26,	2550 },
	{ MMC_VDD_26_27,	2650 },
	{ MMC_VDD_27_28,	2750 },
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2950 },
};

static uint32_t mahimahi_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;
	int ret;

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(sdslot_vreg);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		ret = vreg_enable(sdslot_vreg);
		if (ret)
			pr_err("%s: Error enabling vreg (%d)\n", __func__, ret);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask != (1 << vdd))
			continue;
		ret = vreg_set_level(sdslot_vreg, mmc_vdd_table[i].level);
		if (ret)
			pr_err("%s: Error setting level (%d)\n", __func__, ret);
		return 0;
	}

	pr_err("%s: Invalid VDD (%d) specified\n", __func__, vdd);
	return 0;
}

static unsigned int mahimahi_sdslot_status(struct device *dev)
{
	return !gpio_get_value(MAHIMAHI_GPIO_SDMC_CD_N);
}

#define MAHIMAHI_MMC_VDD	(MMC_VDD_165_195 | MMC_VDD_20_21 | \
				 MMC_VDD_21_22  | MMC_VDD_22_23 | \
				 MMC_VDD_23_24 | MMC_VDD_24_25 | \
				 MMC_VDD_25_26 | MMC_VDD_26_27 | \
				 MMC_VDD_27_28 | MMC_VDD_28_29 | \
				 MMC_VDD_29_30)

static struct mmc_platform_data mahimahi_sdslot_data = {
	.ocr_mask	= MAHIMAHI_MMC_VDD,
	.status		= mahimahi_sdslot_status,
	.translate_vdd	= mahimahi_sdslot_switchvdd,
};

int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
		 unsigned int stat_irq, unsigned long stat_irq_flags);

int __init mahimahi_init_mmc(unsigned int sys_rev)
{
	printk("%s()+\n", __func__);


	if (opt_disable_sdcard) {
		pr_info("%s: sdcard disabled on cmdline\n", __func__);
		goto done;
	}

	sdslot_vreg_enabled = 0;

	sdslot_vreg = vreg_get(0, "gp6");
	if (IS_ERR(sdslot_vreg))
		return PTR_ERR(sdslot_vreg);

	set_irq_wake(MSM_GPIO_TO_INT(MAHIMAHI_GPIO_SDMC_CD_N), 1);

	msm_add_sdcc(2, &mahimahi_sdslot_data,
		     MSM_GPIO_TO_INT(MAHIMAHI_GPIO_SDMC_CD_N),
		     IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE);

done:
	printk("%s()-\n", __func__);
	return 0;
}
