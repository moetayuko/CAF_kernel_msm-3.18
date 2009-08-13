/* drivers/power/ds2784_battery.c
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
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>

#include "../w1/w1.h"
#include "../w1/slaves/w1_ds2784.h"

struct ds2784_device_info {
	struct device *dev;

	/* DS2784 data, valid after calling ds2784_battery_read_status() */
	unsigned long update_time;	/* jiffies when data read */
	char raw[DS2784_DATA_SIZE];	/* raw DS2784 data */
	int voltage_uV;			/* units of uV */
	int current_uA;			/* units of uA */
	int current_avg_uA;
	int temp_raw;			/* units of 0.125 C */
	int temp_C;			/* units of 0.1 C */
	int charge_status;		/* POWER_SUPPLY_STATUS_* */
	int percentage;			/* battery percentage */
	int guage_status_reg;		/* battery status register offset=01h*/

	int charging_source;		/* 0: no cable, 1:usb, 2:AC */

	struct power_supply bat;
	struct device *w1_dev;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

#define psy_to_dev_info(x) container_of((x), struct ds2784_device_info, bat)

static struct wake_lock vbus_wake_lock;
static unsigned int cache_time = 1000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time, "cache time in milliseconds");

/* Battery ID
PS. 0 or other battery ID use the same parameters*/
#define BATT_NO_SOURCE        (0)  /* 0: No source battery */
#define BATT_FIRST_SOURCE     (1)  /* 1: Main source battery */
#define BATT_SECOND_SOURCE    (2)  /* 2: Second source battery */
#define BATT_THIRD_SOURCE     (3)  /* 3: Third source battery */
#define BATT_FOURTH_SOURCE    (4)  /* 4: Fourth source battery */
#define BATT_FIFTH_SOURCE     (5)  /* 5: Fifth source battery */
#define BATT_UNKNOWN        (255)  /* Other: Unknown battery */

#define BATT_RSNSP			(67)	/*Passion battery source 1*/

#define GPIO_BATTERY_DETECTION		39
#define GPIO_BATTERY_CHARGER_EN		22
#define GPIO_BATTERY_CHARGER_CURRENT	16

static enum power_supply_property battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
};

static int battery_initial;

typedef enum {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG
} batt_ctl_t;

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

/* HTC dedicated attributes */
static ssize_t battery_show_property(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static int battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static void battery_ext_power_changed(struct power_supply *psy);

static int g_usb_online;

#define to_ds2784_device_info(x) container_of((x), struct ds2784_device_info, \
					      bat);

static int ds2784_battery_read_status(struct ds2784_device_info *di)
{
	int ret, start, count;
	short n;

	/* The first time we read the entire contents of SRAM/EEPROM,
	 * but after that we just read the interesting bits that change. */
	if (di->raw[DS2784_REG_RSNSP] == 0x00) {
		start = 0;
		count = DS2784_DATA_SIZE;
	} else {
		start = DS2784_REG_PORT;
		count = DS2784_REG_CURR_LSB - start + 1;
	}

	ret = w1_ds2784_read(di->w1_dev, di->raw + start, start, count);
	if (ret != count) {
		dev_warn(di->dev, "call to w1_ds2784_read failed (0x%p)\n",
			 di->w1_dev);
		return 1;
	}
	di->update_time = jiffies;

	/*
	 * Check if dummy battery in.
	 * Workaround for dummy battery
	 * Write ACR MSB to 0x05, ensure there must be 500mAH .
	 * ONLY check when battery driver init.
	 */
	if (battery_initial == 0) {
		if (di->raw[DS2784_REG_USER_EEPROM_20] == 0x01) {
			unsigned char acr[2];
			acr[0] = 0x05;
			acr[1] = 0x06;
			w1_ds2784_write(di->w1_dev, acr,DS2784_REG_ACCUMULATE_CURR_MSB, 2);
		}
		dev_warn(di->dev, "battery dummy battery = %d\n", di->raw[DS2784_REG_USER_EEPROM_20]);
		battery_initial = 1;
	}

	pr_info("batt: %02x %02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x\n",
		di->raw[0x00], di->raw[0x01], di->raw[0x02], di->raw[0x03], 
		di->raw[0x04], di->raw[0x05], di->raw[0x06], di->raw[0x07], 
		di->raw[0x08], di->raw[0x09], di->raw[0x0a], di->raw[0x0b], 
		di->raw[0x0c], di->raw[0x0d], di->raw[0x0e], di->raw[0x0f]
		);

#if 0
	/*
	 * Get Rsns, get from offset 69H . Rsnsp=1/Rsns
	 * Judge if this is supported battery
	 */
	if (di->raw[DS2784_REG_RSNSP] != BATT_RSNSP)
		htc_batt_info.rep.batt_id = BATT_UNKNOWN;
	else
		htc_batt_info.rep.batt_id = BATT_FIRST_SOURCE;
#endif

	/* Get status reg */
	di->guage_status_reg = di->raw[DS2784_REG_STS];

	/* Get Level */
	di->percentage = di->raw[DS2784_REG_RARC];

	/* Get Voltage: Unit=4.886mV, range is 0V to 4.99V */
	n = (((di->raw[DS2784_REG_VOLT_MSB] << 8) |
	      (di->raw[DS2784_REG_VOLT_LSB])) >> 5);

	di->voltage_uV = n * 4886;

	/* Get Current: Unit= 1.5625uV x Rsnsp(67)=104.68 */
	n = ((di->raw[DS2784_REG_CURR_MSB]) << 8) |
		di->raw[DS2784_REG_CURR_LSB];
	di->current_uA = ((n * 15625) / 10000) * 67;

	n = ((di->raw[DS2784_REG_AVG_CURR_MSB]) << 8) |
		di->raw[DS2784_REG_AVG_CURR_LSB];
	di->current_avg_uA = ((n * 15625) / 10000) * 67;

	count = (di->raw[DS2784_REG_RAAC_MSB] << 8) |
		di->raw[DS2784_REG_RAAC_LSB];

	/* Get Temperature:
	 * Unit=0.125 degree C,therefore, give up LSB ,
	 * just caculate MSB for temperature only.
	 */
	di->temp_raw = (((signed char)di->raw[DS2784_REG_TEMP_MSB]) << 3) |
				     (di->raw[DS2784_REG_TEMP_LSB] >> 5);
	di->temp_C = di->temp_raw + (di->temp_raw / 4);

	pr_info("batt: rsnsp=%d, rarc=%d, %d mV, %d mA, %d C %d mAh\n",
		di->raw[DS2784_REG_RSNSP], di->raw[DS2784_REG_RARC],
		di->voltage_uV / 1000, di->current_uA / 1000,
		di->temp_C, count);
	
	return 0;
}

static int battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct ds2784_device_info *di = psy_to_dev_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (di->charging_source) {
		case CHARGER_BATTERY:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case CHARGER_USB:
		case CHARGER_AC:
			if (di->percentage == 100) 
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* XXX todo */
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->percentage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->current_avg_uA;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*[TO DO] need to port Battery Algorithm to meet power spec here.
Only update uevent to upper if percentage changed currently
*/
static void ds2784_battery_update_status(struct ds2784_device_info *di)
{
	u8 last_level;
	last_level = di->percentage;

	ds2784_battery_read_status(di);

#if 0
	if (htc_batt_info.rep.batt_id == BATT_UNKNOWN) {
		htc_batt_info.rep.level = 0;
		printk("Not support battery %d, power down system\n",htc_batt_info.rep.batt_id);
		power_supply_changed(&di->bat);
	}
#endif

	if (last_level != di->percentage)
		power_supply_changed(&di->bat);
}

static void ds2784_battery_work(struct work_struct *work)
{
	struct ds2784_device_info *di =
		container_of(work, struct ds2784_device_info, monitor_work.work);
	const int interval = HZ * 60;

	ds2784_battery_update_status(di);

	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static int battery_charging_ctrl(batt_ctl_t ctl)
{
	int result = 0;

	switch (ctl) {
	case DISABLE:
		pr_info(" charger OFF\n");
		/* 0 for enable; 1 disable */
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 1);
		break;
	case ENABLE_SLOW_CHG:
		pr_info(" charger ON (SLOW)\n");
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_CURRENT, 0);
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 0);
		break;
	case ENABLE_FAST_CHG:
		pr_info(" charger ON (FAST)\n");
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_CURRENT, 1);
		result = gpio_direction_output(GPIO_BATTERY_CHARGER_EN, 0);
		break;
	default:
		printk(KERN_ERR " Not supported battery ctr called.!\n");
		result = -EINVAL;
		break;
	}
	return result;
}

#if 0
static  int battery_set_charging(batt_ctl_t ctl)
{
	int rc;
	rc = battery_charging_ctrl(ctl);
	if (rc < 0)
		goto result;
	if (!battery_initial)
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
	else
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
result:
	return rc;
}

int htc_cable_status_update(int status)
{
	int rc = 0;
	unsigned last_source;

	if (!battery_initial)
		return 0;

	if (status < CHARGER_BATTERY || status > CHARGER_AC) {
		BATT("%s: Not supported cable status received!", __func__);
		return -EINVAL;
	}

	/* A9 reports USB charging when helf AC cable in and China AC charger. */
	/* Work arround: notify userspace AC charging first,
	   and notify USB charging again when receiving usb connected notificaiton
	   from usb driver. */
	last_source = htc_batt_info.rep.charging_source;
	if (status == CHARGER_USB && g_usb_online == 0)
		htc_batt_info.rep.charging_source = CHARGER_AC;
	else {
		htc_batt_info.rep.charging_source  = status;
		/* usb driver will not notify usb offline. */
		if (status == CHARGER_BATTERY && g_usb_online == 1)
			g_usb_online = 0;
	}
	battery_set_charging(htc_batt_info.rep.charging_source);
	msm_hsusb_set_vbus_state(status == CHARGER_USB);

	if (htc_batt_info.rep.charging_source != last_source) {
		if (htc_batt_info.rep.charging_source == CHARGER_USB ||
		    htc_batt_info.rep.charging_source == CHARGER_AC) {
			wake_lock(&vbus_wake_lock);
		} else {
			/* give userspace some time to see the uevent and update
			 * LED state or whatnot...
			 */
			wake_lock_timeout(&vbus_wake_lock, HZ / 2);
		}
		if (htc_batt_info.rep.charging_source == CHARGER_BATTERY
		    || last_source == CHARGER_BATTERY)
			power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
		if (htc_batt_info.rep.charging_source == CHARGER_USB
		    || last_source == CHARGER_USB)
			power_supply_changed(&htc_power_supplies[CHARGER_USB]);
		if (htc_batt_info.rep.charging_source == CHARGER_AC
		    || last_source == CHARGER_AC)
			power_supply_changed(&htc_power_supplies[CHARGER_AC]);
	}
	return rc;
}

void notify_usb_connected(int online)
{
	if (g_usb_online != online) {
		g_usb_online = online;
		if (online && htc_batt_info.rep.charging_source == CHARGER_AC)
			htc_cable_status_update(CHARGER_USB);
		else if (online) {
			BATT("warning: usb connected but charging source=%d",
			     htc_batt_info.rep.charging_source);
		}
	}
}
#endif

static void battery_ext_power_changed(struct power_supply *psy)
{
	struct ds2784_device_info *di;
	int got_power;

	di = psy_to_dev_info(psy);
	got_power = power_supply_am_i_supplied(psy);

	pr_info("*** batt ext power changed (%d) ***\n", got_power);

	if (got_power) {
		di->charging_source = CHARGER_USB;
		battery_charging_ctrl(ENABLE_SLOW_CHG);
		wake_lock(&vbus_wake_lock);
	} else {
		di->charging_source = CHARGER_BATTERY;
		battery_charging_ctrl(DISABLE);
		/* give userspace some time to see the uevent and update
		 * LED state or whatnot...
		 */
		wake_lock_timeout(&vbus_wake_lock, HZ / 2);
	}
	power_supply_changed(psy);
}

void notify_usb_connected(int online) 
{
}


static int ds2784_battery_probe(struct platform_device *pdev)
{
	int rc;
	struct ds2784_device_info *di;
	struct ds2784_platform_data *pdata;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->update_time = jiffies;
	platform_set_drvdata(pdev, di);

	pdata = pdev->dev.platform_data;
	di->dev = &pdev->dev;
	di->w1_dev = pdev->dev.parent;

	di->bat.name = "battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = battery_properties;
	di->bat.num_properties = ARRAY_SIZE(battery_properties);
	di->bat.external_power_changed = battery_ext_power_changed;
	di->bat.get_property = battery_get_property;

	rc = power_supply_register(&pdev->dev, &di->bat);
	if (rc)
		goto fail_register;

	INIT_DELAYED_WORK(&di->monitor_work, ds2784_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		rc = -ESRCH;
		goto fail_workqueue;
	}

	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, 0);
	return 0;

fail_workqueue:
	power_supply_unregister(&di->bat);
fail_register:
	kfree(di);
	return rc;
}

static struct platform_driver ds2784_battery_driver = {
	.driver = {
		.name = "ds2784-battery",
	},
	.probe	  = ds2784_battery_probe,
};

static int __init ds2784_battery_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	return platform_driver_register(&ds2784_battery_driver);
}

module_init(ds2784_battery_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Justin Lin <Justin_lin@htc.com>");
MODULE_DESCRIPTION("ds2784 battery driver");
