/* board-mahimahi-microp.c
 * Copyright (C) 2009 Google.
 * Copyright (C) 2009 HTC Corporation.
 *
 * The Microp on mahimahi is an i2c device that supports
 * the following functions
 *   - LEDs (Green, Amber, Jogball backlight)
 *   - Lightsensor
 *   - Headset remotekeys
 *   - G-sensor
 *   - Interrupts
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <mach/htc_pwrsink.h>
#include <linux/earlysuspend.h>
#include "board-mahimahi.h"

#define MICROP_I2C_NAME "mahimahi-microp"

#define MICROP_LSENSOR_ADC_CHAN		6
#define MICROP_REMOTE_KEY_ADC_CHAN	7

#define MICROP_I2C_WCMD_MISC			0x20
#define MICROP_I2C_WCMD_SPI_EN			0x21
#define MICROP_I2C_WCMD_AUTO_BL_CTL		0x23
#define MICROP_I2C_RCMD_SPI_BL_STATUS		0x24
#define MICROP_I2C_RCMD_VERSION			0x30
#define MICROP_I2C_WCMD_ADC_TABLE		0x42
#define MICROP_I2C_WCMD_LED_MODE		0x53
#define MICROP_I2C_RCMD_GREEN_LED_REMAIN_TIME	0x54
#define MICROP_I2C_RCMD_AMBER_LED_REMAIN_TIME	0x55
#define MICROP_I2C_WCMD_JOGBALL_LED_MODE	0x5A
#define MICROP_I2C_RCMD_JOGBALL_LED_REMAIN_TIME	0x5B
#define MICROP_I2C_WCMD_READ_ADC_VALUE_REQ	0x60
#define MICROP_I2C_RCMD_ADC_VALUE		0x62
#define MICROP_I2C_WCMD_REMOTEKEY_TABLE		0x63
#define MICROP_I2C_WCMD_LCM_REGISTER		0x70
#define MICROP_I2C_WCMD_GSENSOR_REG		0x73
#define MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ	0x74
#define MICROP_I2C_RCMD_GSENSOR_REG_DATA	0x75
#define MICROP_I2C_WCMD_GSENSOR_DATA_REQ	0x76
#define MICROP_I2C_RCMD_GSENSOR_X_DATA		0x77
#define MICROP_I2C_RCMD_GSENSOR_Y_DATA		0x78
#define MICROP_I2C_RCMD_GSENSOR_Z_DATA		0x79
#define MICROP_I2C_RCMD_GSENSOR_DATA		0x7A
#define MICROP_I2C_WCMD_OJ_REG			0x7B
#define MICROP_I2C_WCMD_OJ_REG_DATA_REQ		0x7C
#define MICROP_I2C_RCMD_OJ_REG_DATA		0x7D
#define MICROP_I2C_WCMD_OJ_POS_DATA_REQ		0x7E
#define MICROP_I2C_RCMD_OJ_POS_DATA		0x7F
#define MICROP_I2C_WCMD_GPI_INT_CTL_EN		0x80
#define MICROP_I2C_WCMD_GPI_INT_CTL_DIS		0x81
#define MICROP_I2C_RCMD_GPI_INT_STATUS		0x82
#define MICROP_I2C_RCMD_GPIO_STATUS		0x83
#define MICROP_I2C_WCMD_GPI_INT_STATUS_CLR	0x84
#define MICROP_I2C_RCMD_GPI_INT_SETTING		0x85
#define MICROP_I2C_RCMD_REMOTE_KEYCODE		0x87
#define MICROP_I2C_WCMD_REMOTE_KEY_DEBN_TIME	0x88
#define MICROP_I2C_WCMD_REMOTE_PLUG_DEBN_TIME	0x89
#define MICROP_I2C_WCMD_SIMCARD_DEBN_TIME	0x8A
#define MICROP_I2C_WCMD_GPO_LED_STATUS_EN	0x90
#define MICROP_I2C_WCMD_GPO_LED_STATUS_DIS	0x91

#define IRQ_GSENSOR	(1<<10)
#define IRQ_LSENSOR  	(1<<9)
#define IRQ_REMOTEKEY	(1<<7)
#define IRQ_HEADSETIN	(1<<2)
#define IRQ_SDCARD	(1<<0)

enum led_type {
	GREEN_LED, AMBER_LED, JOGBALL_LED
};

static uint16_t lsensor_adc_table[10] = {
	0x005, 0x00A, 0x00F, 0x01E, 0x03C, 0x121, 0x205, 0x2BA, 0x26E, 0x3FF
};

static uint16_t remote_key_adc_table[6] = {
	0, 19, 51, 98, 141, 196
};

static struct wake_lock microp_i2c_wakelock;

static struct i2c_client *private_microp_client;

struct microp_int_pin {
	uint16_t int_gsensor;
	uint16_t int_lsensor;
	uint16_t int_reset;
	uint16_t int_simcard;
	uint16_t int_hpin;
	uint16_t int_remotekey;
};

struct microp_led_data {
	int type;
	struct led_classdev ldev;
	struct mutex led_data_mutex;
	uint8_t mode;
	uint8_t blink;
};

struct microp_i2c_work {
	struct work_struct work;
	struct i2c_client *client;
	int (*intr_debounce)(uint8_t *pin_status);
	void (*intr_function)(uint8_t *pin_status);
};

struct microp_i2c_client_data {
	struct microp_led_data green;
	struct microp_led_data amber;
	struct microp_led_data jogball;
	uint16_t version;
	struct microp_i2c_work work;
	struct delayed_work ls_on_work;
	struct delayed_work hpin_work;
	struct early_suspend early_suspend;
	uint8_t enable_early_suspend;
	uint8_t enable_reset_button;
	int microp_is_suspend;
	int auto_backlight_enabled;
	uint8_t light_sensor_enabled;
	int headset_is_in;
};

static struct led_trigger light_sensor_trigger = {
	.name     = "light-sensor-trigger",
};

static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

#define I2C_READ_RETRY_TIMES  10
#define I2C_WRITE_RETRY_TIMES 10

static int i2c_read_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	int ret;
	struct i2c_msg msgs[] = {
	{
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &addr,
	},
	{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
	}
	};

	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2) {
			dev_dbg(&client->dev, "R [%02X] = %s\n", addr,
					hex2string(data, length));
			return 0;
		}
		msleep(10);
	}

	dev_err(&client->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
	return -EIO;
}

#define MICROP_I2C_WRITE_BLOCK_SIZE 21
static int i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n", addr,
			hex2string(data, length));

	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	memcpy((void *)&buf[1], (void *)data, length);

	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret == 1)
			return 0;
		msleep(10);
	}
	dev_err(&client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
	return -EIO;
}

static int microp_read_adc(uint8_t channel, uint16_t *value)
{
	struct i2c_client *client;
	int ret;
	uint8_t cmd[2], data[2];

	client = private_microp_client;
	cmd[0] = 0;
	cmd[1] = channel;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_READ_ADC_VALUE_REQ,
			      cmd, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: request adc fail\n", __func__);
		return -EIO;
	}
	msleep(1);
	ret = i2c_read_block(client, MICROP_I2C_RCMD_ADC_VALUE, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read adc fail\n", __func__);
		return -EIO;
	}
	*value = data[0] << 8 | data[1];
	return 0;
}

/*
 *Headset Support
*/
static int get_remote_keycode(uint8_t *keycode)
{
	struct i2c_client *client = private_microp_client;
	int ret;
	uint8_t data[2];

	ret = i2c_read_block(client, MICROP_I2C_RCMD_REMOTE_KEYCODE, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read remote keycode fail\n",
			 __func__);
		return -EIO;
	}
	if (!data[1]) {
		*keycode = 0;
		return 1;		/* no keycode */
	} else if (data[1] & 0x80) {
		*keycode = 0x0;  /* release keycode */
	} else {
		*keycode = data[1];
	}
	return 0;
}

static ssize_t microp_i2c_remotekey_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	uint16_t value;
	int i, button;
	int ret;

	client = to_i2c_client(dev);

	microp_read_adc(MICROP_REMOTE_KEY_ADC_CHAN, &value);

	for (i = 0; i < 3; i += 2) {
		if ((value > remote_key_adc_table[i]) &&
		    (value < remote_key_adc_table[i])) {
			button = i + 1;
		}

	}

	ret = sprintf(buf, "Remote Key[0x%03X] => button %d\n",
		      value, button);

	return ret;
}

static DEVICE_ATTR(key_adc, 0644, microp_i2c_remotekey_adc_show, NULL);

/*
 * LED support
*/
static int microp_i2c_write_led_mode(struct i2c_client *client,
				struct led_classdev *led_cdev,
				uint8_t mode, uint16_t off_timer)
{
	struct microp_i2c_client_data *cdata;
	struct microp_led_data *ldata;
	uint8_t data[7];
	int ret;

	cdata = i2c_get_clientdata(client);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);


	if (ldata->type == GREEN_LED) {
		data[0] = 0x01;
		data[1] = mode;
		data[2] = off_timer >> 8;
		data[3] = off_timer & 0xFF;
		data[4] = 0x00;
		data[5] = 0x00;
		data[6] = 0x00;
	} else if (ldata->type == AMBER_LED) {
		data[0] = 0x02;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x00;
		data[4] = mode;
		data[5] = off_timer >> 8;
		data[6] = off_timer & 0xFF;
	}

	ret = i2c_write_block(client, MICROP_I2C_WCMD_LED_MODE, data, 7);
	if (ret == 0) {
		mutex_lock(&ldata->led_data_mutex);
		if (mode > 1)
			ldata->blink = mode;
		else
			ldata->mode = mode;
		mutex_unlock(&ldata->led_data_mutex);
	}
	return ret;
}

static ssize_t microp_i2c_led_blink_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	int ret;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);

	mutex_lock(&ldata->led_data_mutex);
	ret = sprintf(buf, "%d\n", ldata->blink ? ldata->blink - 1 : 0);
	mutex_unlock(&ldata->led_data_mutex);

	return ret;
}

static ssize_t microp_i2c_led_blink_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	int val, ret;
	uint8_t mode;

	val = -1;
	sscanf(buf, "%u", &val);

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);

	mutex_lock(&ldata->led_data_mutex);
	switch (val) {
	case 0: /* stop flashing */
		mode = ldata->mode;
		ldata->blink = 0;
		break;
	case 1:
	case 2:
	case 3:
		mode = val + 1;
		break;

	default:
		mutex_unlock(&ldata->led_data_mutex);
		return -EINVAL;
	}
	mutex_unlock(&ldata->led_data_mutex);

	ret = microp_i2c_write_led_mode(client, led_cdev, mode, 0xffff);
	if (ret)
		dev_err(&client->dev, "%s set blink failed\n", led_cdev->name);

	return count;
}

static DEVICE_ATTR(blink, 0644, microp_i2c_led_blink_show,
				microp_i2c_led_blink_store);

static ssize_t microp_i2c_led_off_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_i2c_client_data *cdata;
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	uint8_t data[2];
	int ret, offtime;


	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);
	cdata = i2c_get_clientdata(client);

	dev_info(&client->dev, "Getting %s remaining time\n", led_cdev->name);

	if (ldata->type == GREEN_LED) {
		ret = i2c_read_block(client,
				MICROP_I2C_RCMD_GREEN_LED_REMAIN_TIME, data, 2);
	} else if (ldata->type == AMBER_LED) {
		ret = i2c_read_block(client,
				MICROP_I2C_RCMD_AMBER_LED_REMAIN_TIME, data, 2);
	} else {
		dev_err(&client->dev, "Unknown led %s\n", ldata->ldev.name);
		return -EINVAL;
	}

	if (ret) {
		dev_err(&client->dev,
			"%s get off_timer failed\n", led_cdev->name);
	}
	offtime = (int)((data[1] | data[0] << 8) * 2);

	ret = sprintf(buf, "Time remains %d:%d\n", offtime / 60, offtime % 60);
	return ret;
}

static ssize_t microp_i2c_led_off_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	int min, sec, ret;
	uint16_t off_timer;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);

	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);

	dev_info(&client->dev, "Setting %s off_timer to %d min %d sec\n",
			led_cdev->name, min, sec);

	if (!min && !sec)
		off_timer = 0xFFFF;
	else
		off_timer = (min * 60 + sec) / 2;

	ret = microp_i2c_write_led_mode(client, led_cdev,
					ldata->mode, off_timer);
	if (ret) {
		dev_err(&client->dev,
			"%s set off_timer %d min %d sec failed\n",
			led_cdev->name, min, sec);
	}
	return count;
}

static DEVICE_ATTR(off_timer, 0644, microp_i2c_led_off_timer_show,
			microp_i2c_led_off_timer_store);

static void microp_led_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct i2c_client *client;
	int ret;
	uint8_t mode;

	client = to_i2c_client(led_cdev->dev->parent);

	dev_info(&client->dev, "Setting %s brightness current %d new %d\n",
			led_cdev->name, led_cdev->brightness, brightness);

	if (brightness > 255)
		brightness = 255;
	led_cdev->brightness = brightness;

	if (brightness)
		mode = 1;
	else
		mode = 0;

	ret = microp_i2c_write_led_mode(client, led_cdev, mode, 0xffff);
	if (ret) {
		dev_err(&client->dev,
			 "led_brightness_set failed to set mode\n");
	}
}

static void microp_led_jogball_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct i2c_client *client;
	uint8_t data[3] = {0, 0, 0};
	int ret = 0;

	client = to_i2c_client(led_cdev->dev->parent);

	dev_info(&client->dev, "Setting Jog brightness current %d new %d\n",
			led_cdev->brightness, brightness);

	switch (brightness) {
	case 0:
		data[0] = 0;
		break;
	case 3:
		data[0] = 1;
		data[1] = data[2] = 0xFF;
		break;
	case 7:
		data[0] = 2;
		data[1] = 0;
		data[2] = 60;
		break;
	default:
		dev_warn(&client->dev, "%s: unknown value: %d\n",
			__func__, brightness);
		break;
	}
	ret = i2c_write_block(client, MICROP_I2C_WCMD_JOGBALL_LED_MODE,
			      data, 3);
	if (ret < 0) {
		dev_err(&client->dev, "%s failed on set jogball mode:0x%2.2X\n",
				__func__, data[0]);
	} else {
		led_cdev->brightness = brightness;
	}
}

/*
 * Light Sensor Support
 */
static int microp_i2c_auto_backlight_mode(struct i2c_client *client,
					    uint8_t enabled)
{
	uint8_t data[2];
	int ret = 0;

	data[0] = 0;
	if (enabled)
		data[1] = 1;
	else
		data[1] = 0;

	ret = i2c_write_block(client, MICROP_I2C_WCMD_AUTO_BL_CTL, data, 2);
	if (ret != 0)
		printk(KERN_ERR "%s: set auto light sensor fail\n", __func__);

	return ret;
}

static void light_sensor_activate(struct led_classdev *led_cdev)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	if (cdata->microp_is_suspend) {
		printk(KERN_ERR "%s: abort, uP is going to suspend after #\n",
		       __func__);
		return;
	}

	ret = microp_i2c_auto_backlight_mode(client, 1);
	if (ret < 0)
		printk(KERN_ERR "%s: set auto light sensor fail\n", __func__);
	else
		cdata->auto_backlight_enabled = 1;
}

static void light_sensor_deactivate(struct led_classdev *led_cdev)
{
	/* update trigger data when done */
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	if (cdata->microp_is_suspend) {
		printk(KERN_ERR "%s: abort, uP is going to suspend after #\n",
		       __func__);
		return;
	}

	ret = microp_i2c_auto_backlight_mode(client, 0);
	if (ret < 0)
		printk(KERN_ERR "%s: disable auto light sensor fail\n",
		       __func__);
	else
		cdata->auto_backlight_enabled = 0;
}

static int microp_auto_backlight_function(uint16_t *adc_value,
					  uint8_t *adc_level)
{
	struct i2c_client *client;
	uint8_t i, level = 0;
	int ret;

	client = private_microp_client;

	ret = microp_read_adc(MICROP_LSENSOR_ADC_CHAN, adc_value);
	if (ret != 0)
		return -1;

	if (*adc_value > 0x3FF) {
		printk(KERN_WARNING "%s: get wrong value: 0x%X\n",
			__func__, *adc_value);
		return -1;
	} else {
		for (i = 0; i < 10; i++) {
			if (*adc_value <= lsensor_adc_table[i]) {
				level = i;
				break;
			}
		}
		*adc_level = level;
		printk(KERN_DEBUG "%s: ADC value: 0x%X, level: %d #\n",
				__func__, *adc_value, *adc_level);
	}

	return 0;
}

static ssize_t microp_i2c_lightsensor_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint8_t adc_level = 0;
	uint16_t adc_value = 0;
	int ret;

	ret = microp_auto_backlight_function(&adc_value, &adc_level);

	ret = sprintf(buf, "ADC[0x%03X] => level %d\n", adc_value, adc_level);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0644, microp_i2c_lightsensor_adc_show, NULL);

static ssize_t microp_i2c_ls_auto_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	uint8_t data[2] = {0, 0};
	int ret;

	client = to_i2c_client(dev);

	i2c_read_block(client, MICROP_I2C_RCMD_SPI_BL_STATUS, data, 2);
	ret = sprintf(buf, "Light sensor Auto = %d, SPI enable = %d\n",
			data[0], data[1]);

	return ret;
}

static ssize_t microp_i2c_ls_auto_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t enable = 0;
	int ls_auto;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto < 0 || ls_auto > 1)
		return -EINVAL;

	client = to_i2c_client(dev);
	cdata = i2c_get_clientdata(client);

	if (ls_auto) {
		enable = 1;
		cdata->auto_backlight_enabled = 1;
	} else {
		enable = 0;
		cdata->auto_backlight_enabled = 0;
	}

	microp_i2c_auto_backlight_mode(client, enable);

	return count;
}

static DEVICE_ATTR(ls_auto, 0644,  microp_i2c_ls_auto_show,
			microp_i2c_ls_auto_store);

/*
 * Interrupt
 */
static irqreturn_t microp_i2c_intr_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	client = to_i2c_client(dev_id);
	cdata = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "intr_irq_handler\n");

	disable_irq_nosync(client->irq);
	schedule_work(&cdata->work.work);
	return IRQ_HANDLED;
}

static void microp_i2c_intr_work_func(struct work_struct *work)
{
	struct microp_i2c_work *up_work;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[3], node = 0, adc_level;
	uint16_t intr_status = 0, adc_value;
	int insert = 0, keycode = 0, ret = 0;

	up_work = container_of(work, struct microp_i2c_work, work);
	client = up_work->client;
	cdata = i2c_get_clientdata(client);

	ret = i2c_read_block(client, MICROP_I2C_RCMD_GPI_INT_STATUS, data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read interrupt status fail\n",
			 __func__);
	}
	msleep(1);
	intr_status = data[0]<<8 | data[1];
	ret = i2c_write_block(client, MICROP_I2C_WCMD_GPI_INT_STATUS_CLR,
			      data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: clear interrupt status fail\n",
			 __func__);
	}
	printk(KERN_DEBUG "intr_status=0x%02x\n", intr_status);
	msleep(1);
	if (intr_status & IRQ_LSENSOR) {
		ret = microp_auto_backlight_function(&adc_value, &adc_level);
		if (!ret)
			led_trigger_event(&light_sensor_trigger, adc_level);
	}
#if 0
	if (intr_status & IRQ_REMOTEKEY) {
		node = pdata->function_node[MICROP_FUNCTION_REMOTEKEY];
		if (get_remote_keycode(data) == 0) {
			keycode = (int)datla[1];
			cnf_driver_event("H2W_3button", &keycode);
		}
	}
	if (intr_status & IRQ_SDCARD) {

	}
	if (intr_status & cdata->int_pin.int_simcard) {

	}
	if (intr_status & IRQ_HEADSETIN) {
		msleep(300);
		microp_read_gpio_status(data);
		insert = (((data[0] << 8 | data[1]) &
				cdata->int_pin.int_hpin) == 0) ? 1 : 0;
		if (insert != cdata->headset_is_in) {
			cdata->headset_is_in = insert;
			cnf_driver_event("insert_35mm", &cdata->headset_is_in);
		}
	}
#endif
	enable_irq(client->irq);
}

static void microp_i2c_ls_on_work_func(struct work_struct *work)
{
	struct microp_i2c_client_data *cdata = container_of(work,
				struct microp_i2c_client_data, ls_on_work);
	struct i2c_client *client = private_microp_client;

	cdata->light_sensor_enabled = 1;
	if (cdata->auto_backlight_enabled)
		microp_i2c_auto_backlight_mode(client, 1);
}


static int microp_function_initialize(struct i2c_client *client)
{
	struct microp_i2c_client_data *cdata;
	uint8_t data[20];
	int i;
	int ret;

	cdata = i2c_get_clientdata(client);

	/* Light Sensor */
	for (i = 0; i < 10; i++) {
		data[i] = (uint8_t)(lsensor_adc_table[i] >> 8);
		data[i + 10] = (uint8_t)(lsensor_adc_table[i]);
	}
	ret = i2c_write_block(client, MICROP_I2C_WCMD_ADC_TABLE, data, 20);
	if (ret)
		goto exit;

	INIT_DELAYED_WORK(&cdata->ls_on_work, microp_i2c_ls_on_work_func);

	ret = gpio_request(MAHIMAHI_GPIO_LS_EN_N, "microp_i2c");
	if (ret < 0) {
		dev_err(&client->dev, "failed on request gpio ls_on\n");
		goto exit;
	}
	ret = gpio_direction_output(MAHIMAHI_GPIO_LS_EN_N, 0);
	if (ret < 0) {
		dev_err(&client->dev, "failed on gpio_direction_output"
				"ls_on\n");
		goto err_gpio_ls;
	}
	cdata->light_sensor_enabled = 1;

	/* Headset Keys */
	for (i = 0; i < 6; i++) {
		data[i] = (uint8_t)(remote_key_adc_table[i] >> 8);
		data[i + 6] = (uint8_t)(remote_key_adc_table[i]);
	}
	return 0;
err_gpio_ls:
	gpio_free(MAHIMAHI_GPIO_LS_EN_N);
exit:
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void microp_early_suspend(struct early_suspend *h)
{
	struct microp_i2c_client_data *cdata;
	struct i2c_client *client = private_microp_client;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);

	cdata->microp_is_suspend = 1;
	if (cdata->auto_backlight_enabled)
		microp_i2c_auto_backlight_mode(client, 0);
	if (cdata->light_sensor_enabled == 1) {
		gpio_set_value(MAHIMAHI_GPIO_LS_EN_N, 1);
		cdata->light_sensor_enabled = 0;
	}
	cancel_delayed_work_sync(&cdata->ls_on_work);
}

void microp_early_resume(struct early_suspend *h)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);

	gpio_set_value(MAHIMAHI_GPIO_LS_EN_N, 0);
	schedule_delayed_work(&cdata->ls_on_work, msecs_to_jiffies(800));

	if (cdata->auto_backlight_enabled)
		microp_i2c_auto_backlight_mode(client, 1);

	cdata->microp_is_suspend = 0;
}
#endif

static int __devexit microp_i2c_remove(struct i2c_client *client)
{
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (cdata->enable_early_suspend) {
		unregister_early_suspend(&cdata->early_suspend);
	}
#endif
	led_classdev_unregister(&cdata->jogball.ldev);

	led_classdev_unregister(&cdata->amber.ldev);
	device_remove_file(cdata->amber.ldev.dev, &dev_attr_blink);
	device_remove_file(cdata->amber.ldev.dev, &dev_attr_off_timer);

	led_classdev_unregister(&cdata->green.ldev);
	device_remove_file(cdata->green.ldev.dev, &dev_attr_blink);
	device_remove_file(cdata->green.ldev.dev, &dev_attr_off_timer);

	free_irq(client->irq, &client->dev);

	gpio_free(MAHIMAHI_GPIO_UP_RESET_N);

	device_remove_file(&client->dev, &dev_attr_ls_adc);
	device_remove_file(&client->dev, &dev_attr_key_adc);
	device_remove_file(&client->dev, &dev_attr_ls_auto);

	kfree(cdata);

	return 0;
}

static int microp_i2c_suspend(struct i2c_client *client,
	pm_message_t mesg)
{
	return 0;
}

static int microp_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static int microp_i2c_probe(struct i2c_client *client
	, const struct i2c_device_id *id)
{
	struct microp_i2c_client_data *cdata;
	struct microp_led_data *ldata;
	uint8_t data[6];
	int ret;
	int lcd = -1;

	private_microp_client = client;
	ret = i2c_read_block(client, MICROP_I2C_RCMD_VERSION, data, 2);
	if (ret || !(data[0] && data[1])) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed on get microp version\n");
		goto err_exit;
	}
	dev_info(&client->dev, "microp version [%02X][%02X]\n",
		  data[0], data[1]);

	ret = gpio_request(MAHIMAHI_GPIO_UP_RESET_N, "microp_i2c_wm");
	if (ret < 0) {
		dev_err(&client->dev, "failed on request gpio reset\n");
		goto err_exit;
	}
	ret = gpio_direction_output(MAHIMAHI_GPIO_UP_RESET_N, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			 "failed on gpio_direction_output reset\n");
		goto err_gpio_reset;
	}

	cdata = kzalloc(sizeof(struct microp_i2c_client_data), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed on allocat cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);
	cdata->version = data[0] << 8 | data[1];
	cdata->microp_is_suspend = 0;
	cdata->auto_backlight_enabled = 0;
	cdata->light_sensor_enabled = 0;

	wake_lock_init(&microp_i2c_wakelock, WAKE_LOCK_SUSPEND,
			 "microp_i2c_present");

	/* Light Sensor */
	led_trigger_register(&light_sensor_trigger);
	light_sensor_trigger.activate = light_sensor_activate;
	light_sensor_trigger.deactivate = light_sensor_deactivate;

	ret = device_create_file(&client->dev, &dev_attr_ls_adc);
	ret = device_create_file(&client->dev, &dev_attr_ls_auto);

	/* LEDs */
	ldata = &cdata->green;
	ldata->type = GREEN_LED;
	ldata->ldev.name = "green";
	ldata->ldev.brightness_set = microp_led_brightness_set;
	ldata->ldev.brightness = 0;
	mutex_init(&ldata->led_data_mutex);
	ret = led_classdev_register(&client->dev, &ldata->ldev);
	if (ret < 0) {
		dev_err(&client->dev, "failed on led_classdev_register "
					"for green led\n");
		goto err_register_green_leddev;
	}
	ret = device_create_file(ldata->ldev.dev, &dev_attr_blink);
	ret = device_create_file(ldata->ldev.dev, &dev_attr_off_timer);

	ldata = &cdata->amber;
	ldata->type = AMBER_LED;
	ldata->ldev.name = "amber";
	ldata->ldev.brightness_set = microp_led_brightness_set;
	ldata->ldev.brightness = 0;
	mutex_init(&ldata->led_data_mutex);
	ret = led_classdev_register(&client->dev, &ldata->ldev);
	if (ret < 0) {
		dev_err(&client->dev, "failed on led_classdev_register "
					"for amber led\n");
		goto err_register_amber_leddev;
	}
	ret = device_create_file(ldata->ldev.dev, &dev_attr_blink);
	ret = device_create_file(ldata->ldev.dev, &dev_attr_off_timer);

	ldata = &cdata->jogball;
	ldata->type = JOGBALL_LED;
	ldata->ldev.name = "jogball" ;
	ldata->ldev.brightness_set = microp_led_jogball_brightness_set;
	ldata->ldev.brightness = 0;
	mutex_init(&ldata->led_data_mutex);
	ret = led_classdev_register(&client->dev, &ldata->ldev);
	if (ret < 0) {
		dev_err(&client->dev, "failed on led_classdev_register "
					"for jogball led\n");
		goto err_register_logball_leddev;
	}

	/* Headset */
	ret = device_create_file(&client->dev, &dev_attr_key_adc);

	/* Setup IRQ handler */
	INIT_WORK(&cdata->work.work, microp_i2c_intr_work_func);
	cdata->work.client = client;

	ret = request_irq(client->irq,
			microp_i2c_intr_irq_handler,
			IRQF_TRIGGER_LOW,
			"microp_interrupt",
			&client->dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_intr;
	}
	ret = set_irq_wake(client->irq, 1);
	if (ret) {
		dev_err(&client->dev, "set_irq_wake failed\n");
		goto err_intr;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (cdata->enable_early_suspend) {
		cdata->early_suspend.level =
				EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		cdata->early_suspend.suspend = microp_early_suspend;
		cdata->early_suspend.resume = microp_early_resume;
		register_early_suspend(&cdata->early_suspend);
	}
#endif

	ret = microp_function_initialize(client);
	if (ret) {
		dev_err(&client->dev, "failed on microp function initialize\n");
		goto err_fun_init;
	}

	return 0;

err_fun_init:
err_intr:
	led_classdev_unregister(&cdata->jogball.ldev);

err_register_logball_leddev:
	led_classdev_unregister(&cdata->amber.ldev);
	device_remove_file(cdata->amber.ldev.dev, &dev_attr_blink);
	device_remove_file(cdata->amber.ldev.dev, &dev_attr_off_timer);

err_register_amber_leddev:
	led_classdev_unregister(&cdata->green.ldev);
	device_remove_file(cdata->green.ldev.dev, &dev_attr_blink);
	device_remove_file(cdata->green.ldev.dev, &dev_attr_off_timer);

err_register_green_leddev:
	wake_lock_destroy(&microp_i2c_wakelock);
	device_remove_file(&client->dev, &dev_attr_ls_adc);
	device_remove_file(&client->dev, &dev_attr_ls_auto);
	kfree(cdata);
	i2c_set_clientdata(client, NULL);
err_cdata:
err_gpio_reset:
	gpio_free(MAHIMAHI_GPIO_UP_RESET_N);
err_exit:
	return ret;
}

static const struct i2c_device_id microp_i2c_id[] = {
	{ MICROP_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver microp_i2c_driver = {
	.driver = {
		   .name = MICROP_I2C_NAME,
		   },
	.id_table = microp_i2c_id,
	.probe = microp_i2c_probe,
	.suspend = microp_i2c_suspend,
	.resume = microp_i2c_resume,
	.remove = __devexit_p(microp_i2c_remove),
};


static int __init microp_i2c_init(void)
{
	return i2c_add_driver(&microp_i2c_driver);
}

static void __exit microp_i2c_exit(void)
{
	i2c_del_driver(&microp_i2c_driver);
}

module_init(microp_i2c_init);
module_exit(microp_i2c_exit);

MODULE_AUTHOR("Eric Olsen <eolsen@android.com>");
MODULE_DESCRIPTION("MicroP I2C driver");
MODULE_LICENSE("GPL");
