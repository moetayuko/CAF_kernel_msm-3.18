/*
 * drivers/input/misc/cm36283.c
 *
 * CM36283 ALS sensor driver
 *
 * Copyright (C) 2008-2012 Code Aurora Forum.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

/*
 * 2012-01-13 Update copyright infomation
 * 2011-11-26 First version
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#define VENDOR_NAME     "Capella"
#define SENSOR_NAME 	"cm36283"
#define DRIVER_VERSION  "1.0"
#define CM36283_MAX_DELAY 200

/* register definitation */
#define ALS_CONF		0x00
#define ALS_THDL		0x01
#define ALS_THDH		0x02
#define PS_CONF1_CONF2		0x03
#define PS_CONF3_MS		0x04
#define PS_CANC			0x05
#define PS_THD			0x06
#define PS_DATA			0x08
#define ALS_DATA		0x09
#define INT_FLAG		0x0B
#define DEV_ID			0x0C

#define PS_MAX			255
#define ALS_MAX			3277

#define ALS_CONF_VAL		0x0040
#define ALS_DISABLE		0x0001

#define PS_CONF12_VAL		0x80B2		/*Correct the value of PS_CONF12_VAL
						according to the PS_MAX is 255.*/
#define PS_DISABLE		0x0001

#define PS_CONF3_VAL		0x0000

struct cm36283_data {
	struct i2c_client *cm36283_client;
	atomic_t delay;
	int als_enable;
	int ps_enable;
	unsigned char als_threshold;
	unsigned char ps_threshold;
	struct input_dev *input;
	struct mutex value_mutex;
	struct delayed_work work;
	struct work_struct irq_work;
};

static int cm36283_smbus_read_word(struct i2c_client *client,
		unsigned char reg_addr)
{
	s32 dummy;
	dummy = i2c_smbus_read_word_data(client, reg_addr);
	if (dummy < 0)
	{
		pr_err("%s read addr=%x read data=%x\n",__func__, reg_addr, dummy);
		return -1;
	}
	return dummy & 0x0000ffff;
}

static int cm36283_smbus_write_word(struct i2c_client *client,
		unsigned char reg_addr, unsigned short data)
{
	s32 dummy;
	dummy = i2c_smbus_write_word_data(client,reg_addr,data);
	if (dummy < 0)
	{
		pr_err("%s write addr=%x write data=%x\n",__func__, reg_addr, data);
		return -1;
	}
	return 0;
}

static int cm36283_read_ps(struct i2c_client *client)
{
	int ret;
	ret = cm36283_smbus_read_word(client, PS_DATA);
	return ret;
}

static int cm36283_read_als(struct i2c_client *client)
{
	int val;
	val = cm36283_smbus_read_word(client, ALS_DATA);
	return val;
}

#define lux_calc(step) (step/20)	/*The coefficient 20 is
					determined by Integration time*/
static int cm36283_get_als(struct i2c_client *client)
{
	int ret;
	static int als_buf[3];
	static int idx = 0;
	ret = cm36283_read_als(client);
	if(ret > -1)
		ret = lux_calc(ret);
	if(ret > ALS_MAX)
		ret = ALS_MAX;
	if(ret > -1)
	{
		als_buf[idx] = ret;
		idx++;
	}
	if(idx == 3)
	{
		ret = (als_buf[0] + als_buf[1] + als_buf[2])/3;
		idx = 0;
	}
	else
		ret = -5;
	return ret;
}

static int cm36283_get_ps(struct i2c_client *client)
{
	int ret;
	ret = cm36283_read_ps(client);
	if(ret > -1)
		ret &= 0x00ff;
	return ret;
}

static void cm36283_work_func(struct work_struct *work)
{
	int ps;
	int als;
	struct cm36283_data *cm36283 = container_of((struct delayed_work *)work,
			struct cm36283_data, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&cm36283->delay));

	ps  = cm36283_get_ps(cm36283->cm36283_client);
	if(cm36283->ps_enable)
	{
		if (ps > -1) {
			input_report_abs(cm36283->input, ABS_DISTANCE, ps);
			input_sync(cm36283->input);
		}
	}
	als = cm36283_get_als(cm36283->cm36283_client);
	if(cm36283->als_enable)
	{
		if (als > -1) {
			input_report_abs(cm36283->input, ABS_MISC, als);
			input_sync(cm36283->input);
		}
	}
	if(cm36283->ps_enable || cm36283->als_enable)
		schedule_delayed_work(&cm36283->work, delay);
}

static int cm36283_als_state(struct cm36283_data *cm36283, int als_on)
{
	int rc;
	u16 reg;

	if(als_on)
		reg = ALS_CONF_VAL;
	else
		reg = ALS_DISABLE;

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,ALS_THDL, 0x00);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,ALS_THDH, 0x00);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,ALS_CONF, reg);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	return rc;
}

static int cm36283_ps_state(struct cm36283_data *cm36283, int ps_on)
{
	int rc;
	u16 reg;

	if(ps_on)
		reg = PS_CONF12_VAL;
	else
		reg = PS_DISABLE;

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,PS_CONF1_CONF2, reg);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,PS_THD, 0x00);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	return rc;
}

static int cm36283_state(struct cm36283_data *cm36283,int als_on, int ps_on)
{
	int rc;
	rc = cm36283_als_state(cm36283, als_on);
	if(rc < 0)
		pr_err("%s cm36283 change state error=%d\n", __func__, rc);
	rc = cm36283_ps_state(cm36283, ps_on);
	if(rc < 0)
		pr_err("%s cm36283 change state error=%d\n", __func__, rc);
	return rc;
}

static ssize_t cm36283_als_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	mutex_lock(&cm36283->value_mutex);
	data = cm36283->als_enable;
	mutex_unlock(&cm36283->value_mutex);
	return sprintf(buf, "%d\n", data);
}

static ssize_t cm36283_als_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	unsigned long delay = msecs_to_jiffies(atomic_read(&cm36283->delay));
	error = strict_strtoul(buf, 10, &data);
	if(error)
		return error;
	pr_debug("%s set als = %ld\n", __func__, data);
	if(data)
	{
		if(!cm36283->als_enable)
		{
			mutex_lock(&cm36283->value_mutex);
			cm36283->als_enable = 1;
			mutex_unlock(&cm36283->value_mutex);
			error = cm36283_als_state(cm36283, cm36283->als_enable);
			if(error < 0)
			{
				pr_err("%s cm36283 change state error = %d\n",__func__, error);
			}else
			{
				schedule_delayed_work(&cm36283->work, delay);
			}
		}
	}
	else
	{
		if(cm36283->als_enable)
			cm36283->als_enable = 0;
		error = cm36283_als_state(cm36283, cm36283->als_enable);
		if(error < 0)
		{
			pr_err("%s cm36283 change state error = %d\n", __func__,error);
		}
	}
	return count;
}

static ssize_t cm36283_als_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	data = cm36283_read_als(cm36283->cm36283_client);
	return sprintf(buf, "%d\n", data);
}

static ssize_t cm36283_ps_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	mutex_lock(&cm36283->value_mutex);
	data = cm36283->ps_enable;
	mutex_unlock(&cm36283->value_mutex);
	return sprintf(buf, "%d\n", data);
}

static ssize_t cm36283_ps_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	unsigned long delay = msecs_to_jiffies(atomic_read(&cm36283->delay));
	error = strict_strtoul(buf, 10, &data);
	if(error)
		return error;
	pr_debug("%s set ps = %ld\n", __func__, data);
	if(data)
	{
		if(!cm36283->ps_enable)
		{
			mutex_lock(&cm36283->value_mutex);
			cm36283->ps_enable = 1;
			mutex_unlock(&cm36283->value_mutex);
			error = cm36283_ps_state(cm36283, cm36283->ps_enable);
			if(error < 0)
			{
				pr_err("%s cm36283 change state error = %d\n",__func__, error);
			}else
			{
				schedule_delayed_work(&cm36283->work, delay);
			}
		}
	}
	else
	{
		mutex_lock(&cm36283->value_mutex);
		if(cm36283->ps_enable)
			cm36283->ps_enable = 0;
		mutex_unlock(&cm36283->value_mutex);
		error = cm36283_ps_state(cm36283, cm36283->ps_enable);
		if(error < 0)
		{
			pr_err("%s cm36283 change state error = %d\n",__func__, error);
		}
	}
	return count;
}

static ssize_t cm36283_ps_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	data = cm36283_read_ps(cm36283->cm36283_client);
	return sprintf(buf, "%d\n", data);
}

static ssize_t cm36283_als_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", cm36283->als_threshold);
}

static ssize_t cm36283_als_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	error = strict_strtoul(buf, 10, &data);
	if(error)
			return error;
	mutex_lock(&cm36283->value_mutex);
	cm36283->als_threshold = data;
	mutex_unlock(&cm36283->value_mutex);

	return count;
}

static ssize_t cm36283_ps_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", cm36283->ps_threshold);
}

static ssize_t cm36283_ps_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	error = strict_strtoul(buf, 10, &data);
	if(error)
			return error;
	mutex_lock(&cm36283->value_mutex);
	cm36283->ps_threshold = data;
	mutex_unlock(&cm36283->value_mutex);

	return count;
}

static ssize_t cm36283_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Chip: %s %s\nVersion: %s\n",
				   VENDOR_NAME, SENSOR_NAME, DRIVER_VERSION);
}

static ssize_t cm36283_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&cm36283->delay));

}

static ssize_t cm36283_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	atomic_set(&cm36283->delay, (unsigned int) data);

	return count;
}

static int cm36283_reg = 0x0c;
static ssize_t cm36283_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int data;
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	data = cm36283_smbus_read_word(cm36283->cm36283_client, cm36283_reg);
	return sprintf(buf, "reg = %d = %x\n", data, data);
}

static ssize_t cm36283_reg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	data &= 0x0f;
	cm36283_reg = data;
	return count;
}

static DEVICE_ATTR(enable_als, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_als_enable_show, cm36283_als_enable_store);
static DEVICE_ATTR(als_data, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_als_data_show, NULL);
static DEVICE_ATTR(enable_ps, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_ps_enable_show, cm36283_ps_enable_store);
static DEVICE_ATTR(ps_data, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_ps_data_show, NULL);
static DEVICE_ATTR(raw_adc, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_ps_data_show, NULL);
static DEVICE_ATTR(als_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_als_threshold_show, cm36283_als_threshold_store);
static DEVICE_ATTR(ps_threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_ps_threshold_show, cm36283_ps_threshold_store);
static DEVICE_ATTR(info, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_info_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_delay_show, cm36283_delay_store);
static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		cm36283_reg_show, cm36283_reg_store);

static struct attribute *cm36283_attributes[] = {
	&dev_attr_enable_als.attr,
	&dev_attr_als_data.attr,
	&dev_attr_enable_ps.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_raw_adc.attr,
	&dev_attr_als_threshold.attr,
	&dev_attr_ps_threshold.attr,
	&dev_attr_info.attr,
	&dev_attr_delay.attr,
	&dev_attr_reg.attr,
	NULL
};

static struct attribute_group cm36283_attribute_group = {
	.attrs = cm36283_attributes
};

static int cm36283_input_init(struct cm36283_data *cm36283)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
	{
		pr_err("%s error input_allocate_device\n", __func__);
		return -ENOMEM;
	}
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_DISTANCE);
	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_drvdata(dev, cm36283);
	input_set_abs_params(dev, ABS_DISTANCE, 0, PS_MAX, 0, 0);
	input_set_abs_params(dev, ABS_MISC, 0, ALS_MAX, 0, 0);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	cm36283->input = dev;
	return 0;
}

static void cm36283_input_delete(struct cm36283_data *cm36283)
{
	struct input_dev *dev = cm36283->input;
	input_unregister_device(dev);
	input_free_device(dev);
}

static int cm36283_config(struct cm36283_data *cm36283)
{
	int rc;

	rc = cm36283_smbus_read_word(cm36283->cm36283_client,DEV_ID);
	if(rc < 0)
		pr_err("%s cm36283_smbus_read_word rc=%d\n", __func__, rc);
	else
		pr_info("%s DEV_ID=0x%x\n",__func__, rc);

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,ALS_CONF, ALS_DISABLE);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,ALS_THDL, 0x00);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,ALS_THDH, 0x00);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,PS_CONF1_CONF2, PS_DISABLE);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	rc = cm36283_smbus_write_word(cm36283->cm36283_client,PS_THD, 0x00);
	if(rc < 0)
		pr_err("%s cm36283_smbus_write_word rc=%d\n", __func__, rc);

	return rc;
}

static int cm36283_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct cm36283_data *data;

	pr_info("%s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct cm36283_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	err = cm36283_input_init(data);
	if (err < 0)
		goto kfree_exit;
	i2c_set_clientdata(client, data);
	data->cm36283_client = client;
	mutex_init(&data->value_mutex);
	INIT_DELAYED_WORK(&data->work, cm36283_work_func);
	atomic_set(&data->delay, CM36283_MAX_DELAY);
	err = sysfs_create_group(&data->input->dev.kobj,
						 &cm36283_attribute_group);
	if (err < 0)
		goto error_sysfs;
	data->ps_enable = 0;
	data->als_enable = 0;
	data->ps_threshold = 0;
	data->als_threshold = 0;
	err = cm36283_config(data);
	if(err < 0)
	{
		pr_err("cm36283_config error err=%d\n", err);
		goto error_sysfs;
	}
	return 0;
error_sysfs:
	cm36283_input_delete(data);
kfree_exit:
	kfree(data);
exit:
	return err;
}

static int cm36283_remove(struct i2c_client *client)
{
	struct cm36283_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&data->input->dev.kobj, &cm36283_attribute_group);
	cm36283_input_delete(data);
	kfree(data);
	return 0;
}

static const struct i2c_device_id cm36283_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cm36283_id);

#ifdef CONFIG_PM
static int cm36283_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	int err = 0;
	pr_debug("%s\n", __func__);
	err = cm36283_state(cm36283, 0, 0);
	if(err < 0)
			return err;
	if(cm36283->als_enable || cm36283->ps_enable)
	{
		cancel_delayed_work_sync(&cm36283->work);
	}
	return 0;
}

static int cm36283_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm36283_data *cm36283 = i2c_get_clientdata(client);
	unsigned long delay = msecs_to_jiffies(atomic_read(&cm36283->delay));
	int err = 0;
	pr_debug("%s\n", __func__);
	err = cm36283_state(cm36283, cm36283->als_enable, cm36283->ps_enable);
	if(err < 0)
			return err;
	if(cm36283->ps_enable || cm36283->als_enable)
		schedule_delayed_work(&cm36283->work, delay);
	return 0;
}

static const struct dev_pm_ops cm36283_pm_ops = {
		.suspend = cm36283_suspend,
		.resume = cm36283_resume,
};
#endif

static struct i2c_driver cm36283_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
#ifdef CONFIG_PM
		.pm = &cm36283_pm_ops,
#endif
	},
	.id_table	= cm36283_id,
	.probe		= cm36283_probe,
	.remove		= cm36283_remove,
};

static int __init cm36283_init(void)
{
	return i2c_add_driver(&cm36283_driver);
}

static void __exit cm36283_exit(void)
{
	i2c_del_driver(&cm36283_driver);
}

MODULE_DESCRIPTION("cm36283 driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

module_init(cm36283_init);
module_exit(cm36283_exit);

