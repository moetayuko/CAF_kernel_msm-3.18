/* linux/arch/arm/mach-msm/board-firestone-mmc.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC
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
#include <linux/debugfs.h>
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
#include <mach/htc_pwrsink.h>

#include "proc_comm.h"
#include "devices.h"
#include "board-firestone.h"

#undef FIRESTONE_DEBUG_MMC

#undef FIRESTONE_DO_WIFI

static struct vreg *vreg_wifi_batpa;	/* WIFI main power */
static int firestone_wifi_cd = 0;	/* WIFI virtual 'card detect' status */

extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);

/* ---- COMMON ---- */
static void config_gpio_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

/* ---- SDCARD ---- */
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

static uint opt_disable_sdcard;

static int __init firestone_disablesdcard_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_sdcard = cal;
	return 1;
}

__setup("board_firestone.disable_sdcard=", firestone_disablesdcard_setup);

static struct vreg *vreg_sdslot;	/* SD slot power */

struct mmc_vdd_xlat {
	int mask;
	int level;
};

static struct mmc_vdd_xlat mmc_vdd_table[] = {
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

static unsigned int sdslot_vdd = 0xffffffff;
static unsigned int sdslot_vreg_enabled;

static uint32_t firestone_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;

	BUG_ON(!vreg_sdslot);

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
		pr_info("%s: Disabling SD slot power\n", __func__);
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		vreg_enable(vreg_sdslot);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		vreg_enable(vreg_wifi_batpa);
		vreg_set_level(vreg_wifi_batpa, 2850);
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			pr_info("%s: Setting level to %u\n", __func__,
				mmc_vdd_table[i].level);
			vreg_set_level(vreg_sdslot, mmc_vdd_table[i].level);
			return 0;
		}
	}

	pr_err("%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int firestone_sdslot_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) gpio_get_value(FIRESTONE_GPIO_SDMC_CD_N);
	return (!status);
}

#define FIRESTONE_MMC_VDD MMC_VDD_165_195 | MMC_VDD_20_21 | MMC_VDD_21_22 \
			  | MMC_VDD_22_23 | MMC_VDD_23_24 | MMC_VDD_24_25 \
			  | MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 \
			  | MMC_VDD_28_29 | MMC_VDD_29_30

static struct mmc_platform_data firestone_sdslot_data = {
	.ocr_mask	= FIRESTONE_MMC_VDD,
	.status		= firestone_sdslot_status,
	.translate_vdd	= firestone_sdslot_switchvdd,
};

/* ---- WIFI ---- */
static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(152, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(51, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(56, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(152, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int firestone_wifi_status_register(
			void (*callback)(int card_present, void *dev_id),
			void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int firestone_wifi_status(struct device *dev)
{
	return firestone_wifi_cd;
}

static struct mmc_platform_data firestone_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= firestone_wifi_status,
	.register_status_notify	= firestone_wifi_status_register,
	.embedded_sdio		= NULL,
};

int firestone_wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	firestone_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(firestone_wifi_set_carddetect);

static int firestone_wifi_power_state;

int firestone_wifi_power(int on)
{
	int rc = 0;

	printk("%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));
		mdelay(50);
		htc_pwrsink_set(PWRSINK_WIFI, 70);
		if (rc)
			return rc;
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
		htc_pwrsink_set(PWRSINK_WIFI, 0);
	}

	mdelay(100);
	gpio_set_value(129, on); /* WIFI_SHUTDOWN */
	mdelay(100);

	firestone_wifi_power_state = on;
	return 0;
}
EXPORT_SYMBOL(firestone_wifi_power);

static int firestone_wifi_reset_state;
int firestone_wifi_reset(int on)
{
	firestone_wifi_reset_state = on;
#if 1
	printk("%s: do nothing\n", __func__);
#else
	printk("%s: %d\n", __func__, on);
	gpio_set_value( TROUT_GPIO_WIFI_PA_RESETX, !on );
	mdelay(50);
#endif
	return 0;
}

int __init firestone_init_mmc(unsigned int sys_rev)
{
	uint32_t id;

	wifi_status_cb = NULL;

	/* initial WIFI_SHUTDOWN# */
	id = PCOM_GPIO_CFG(129, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);

	sdslot_vreg_enabled = 0;

	vreg_sdslot = vreg_get(0, "gp6");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	printk("%s\n", __func__);

	/* vreg_wifi_batpa is always on in Q8K */
	vreg_wifi_batpa = vreg_get(0, "wlan");
	if (IS_ERR(vreg_wifi_batpa))
		return PTR_ERR(vreg_wifi_batpa);

#ifdef FIRESTONE_DO_WIFI
	msm_add_sdcc(1, &firestone_wifi_data, 0, 0);
#endif

	set_irq_wake(MSM_GPIO_TO_INT(FIRESTONE_GPIO_SDMC_CD_N), 1);

	if (!opt_disable_sdcard)
		msm_add_sdcc(2, &firestone_sdslot_data,
			     MSM_GPIO_TO_INT(FIRESTONE_GPIO_SDMC_CD_N),
			     IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE);
	else
		pr_info("firestone: SD-Card interface disabled\n");
	return 0;
}


#if defined(FIRESTONE_DEBUG_MMC) && defined(CONFIG_DEBUG_FS)

static int firestonemmc_dbg_wifi_reset_set(void *data, u64 val)
{
	firestone_wifi_reset((int) val);
	return 0;
}

static int firestonemmc_dbg_wifi_reset_get(void *data, u64 *val)
{
	*val = firestone_wifi_reset_state;
	return 0;
}

static int firestonemmc_dbg_wifi_cd_set(void *data, u64 val)
{
	firestone_wifi_set_carddetect((int) val);
	return 0;
}

static int firestonemmc_dbg_wifi_cd_get(void *data, u64 *val)
{
	*val = firestone_wifi_cd;
	return 0;
}

static int firestonemmc_dbg_wifi_pwr_set(void *data, u64 val)
{
	firestone_wifi_power((int) val);
	return 0;
}

static int firestonemmc_dbg_wifi_pwr_get(void *data, u64 *val)
{
	*val = firestone_wifi_power_state;
	return 0;
}

static int firestonemmc_dbg_sd_pwr_set(void *data, u64 val)
{
	firestone_sdslot_switchvdd(NULL, (unsigned int) val);
	return 0;
}

static int firestonemmc_dbg_sd_pwr_get(void *data, u64 *val)
{
	*val = sdslot_vdd;
	return 0;
}

static int firestonemmc_dbg_sd_cd_set(void *data, u64 val)
{
	return -ENOSYS;
}

static int firestonemmc_dbg_sd_cd_get(void *data, u64 *val)
{
	*val = firestone_sdslot_status(NULL);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(firestonemmc_dbg_wifi_reset_fops,
			firestonemmc_dbg_wifi_reset_get,
			firestonemmc_dbg_wifi_reset_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(firestonemmc_dbg_wifi_cd_fops,
			firestonemmc_dbg_wifi_cd_get,
			firestonemmc_dbg_wifi_cd_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(firestonemmc_dbg_wifi_pwr_fops,
			firestonemmc_dbg_wifi_pwr_get,
			firestonemmc_dbg_wifi_pwr_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(firestonemmc_dbg_sd_pwr_fops,
			firestonemmc_dbg_sd_pwr_get,
			firestonemmc_dbg_sd_pwr_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(firestonemmc_dbg_sd_cd_fops,
			firestonemmc_dbg_sd_cd_get,
			firestonemmc_dbg_sd_cd_set, "%llu\n");

static int __init firestonemmc_dbg_init(void)
{
	struct dentry *dent;

	if (!machine_is_firestone())
		return 0;

	dent = debugfs_create_dir("firestone_mmc_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("wifi_reset", 0644, dent, NULL,
			    &firestonemmc_dbg_wifi_reset_fops);
	debugfs_create_file("wifi_cd", 0644, dent, NULL,
			    &firestonemmc_dbg_wifi_cd_fops);
	debugfs_create_file("wifi_pwr", 0644, dent, NULL,
			    &firestonemmc_dbg_wifi_pwr_fops);
	debugfs_create_file("sd_pwr", 0644, dent, NULL,
			    &firestonemmc_dbg_sd_pwr_fops);
	debugfs_create_file("sd_cd", 0644, dent, NULL,
			    &firestonemmc_dbg_sd_cd_fops);
	return 0;
}

device_initcall(firestonemmc_dbg_init);
#endif
