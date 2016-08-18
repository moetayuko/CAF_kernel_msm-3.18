/* Himax Android Driver Sample Code for HMX852xES chipset
*
* Copyright (C) 2014-2015 Himax Corporation.
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

#include <linux/himax_852xES.h>
#include <linux/debugfs.h>

#define HIMAX_I2C_RETRY_TIMES 10
#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(2 * HZ)

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
static u8 proximity_flag;
static u8 g_proximity_en;
#endif

#if defined(HX_AUTO_UPDATE_FW) || defined(HX_AUTO_UPDATE_CONFIG)
	static unsigned char i_CTPM_FW[] = {
		#include "D816_2014_11_24.i"
	};
#endif

/*static int tpd_keys_local[HX_KEY_MAX_COUNT] = HX_KEY_ARRAY;*/
static unsigned char	IC_CHECKSUM;
static unsigned char	IC_TYPE;

static int		HX_TOUCH_INFO_POINT_CNT;

static int		HX_RX_NUM = 13;
static int		HX_TX_NUM = 23;
static int		HX_BT_NUM;
static int		HX_X_RES;
static int		HX_Y_RES;
static int		HX_MAX_PT = 5;
static bool		HX_XY_REVERSE;
static bool		HX_INT_IS_EDGE;

static unsigned int		FW_VER_MAJ_FLASH_ADDR;
static unsigned int FW_VER_MAJ_FLASH_LENG;
static unsigned int FW_VER_MIN_FLASH_ADDR;
static unsigned int FW_VER_MIN_FLASH_LENG;

static unsigned int FW_CFG_VER_FLASH_ADDR;

static unsigned int CFG_VER_MAJ_FLASH_ADDR;
static unsigned int CFG_VER_MAJ_FLASH_LENG;
static unsigned int CFG_VER_MIN_FLASH_ADDR;
static unsigned int CFG_VER_MIN_FLASH_LENG;

static uint8_t vk_press = 0x00;
static uint8_t AA_press = 0x00;
/*static uint8_t	IC_STATUS_CHECK	= 0xAA;*/
static uint8_t EN_NoiseFilter = 0x00;
static uint8_t Last_EN_NoiseFilter = 0x00;
static int	hx_point_num;/* for himax_ts_work_func use*/
static int	p_point_num	= 0xFFFF;
static int	tpd_key	= 0x00;
static int	tpd_key_old	= 0x00;

static bool config_load;
static struct himax_config *config_selected;

static int iref_number = 11;
static bool iref_found;

#define HIMAX_INFO_MAX_LEN	512
#define DEBUG_DIR_NAME		"ts_debug"

#define STORE_TS_INFO(buf, name, max_touches, fw_vkey_support) \
	snprintf(buf, HIMAX_INFO_MAX_LEN, \
			"controller     = himax\n" \
			"chip name      = %s\n" \
			"max_touches    = %d\n" \
			"driver_ver     = N/A\n" \
			"fw_ver         = V03\n" \
			"fw_vkey_support= %s\n", \
			name, max_touches, fw_vkey_support)

static int himax852xes_resume(struct device *dev);
static int himax852xes_suspend(struct device *dev);


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_OF
#if defined(HX_LOADIN_CONFIG) || defined(HX_AUTO_UPDATE_CONFIG)
static int himax_parse_config(struct himax_ts_data *ts,
				struct himax_config *pdata);
#endif
#endif

/**************************sys file****************************/
/********ts_info********/
static ssize_t hx8526_ts_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *data = dev_get_drvdata(dev);

	return snprintf(buf, HIMAX_INFO_MAX_LEN, "%s\n", data->ts_info);
}

static DEVICE_ATTR(ts_info, 0664, hx8526_ts_info_show, NULL);

/****************mt_protocol_type*************/
static ssize_t hx8526_mt_protocol_type_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, 16, "%s\n", "MT Protocal B");
}

static DEVICE_ATTR(mt_protocol_type, 0664,
	hx8526_mt_protocol_type_show,
			NULL);

/*****************enable*************/
static ssize_t hx8526_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct himax_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (val) {
		data->enable = true;
		/*himax_ts_resume(data->client);*/
		himax852xes_resume(dev);
	} else {
		data->enable = false;
		/*himax_ts_suspend(data->client, PMSG_SUSPEND);*/
		himax852xes_suspend(dev);
	}

	return size;
}

static ssize_t hx8526_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *data = dev_get_drvdata(dev);

	if (data->suspended) {
		dev_info(&data->client->dev, "Already in suspend state\n");
		return snprintf(buf, 4, "%s\n", "0");
	}

	return snprintf(buf, 4, "%s\n", data->enable ? "1" : "0");
}

static DEVICE_ATTR(enable, 0664, hx8526_enable_show, hx8526_enable_store);

/***********************************************/
static ssize_t hx8526_update_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	return snprintf(buf, 8, "%s\n", "N/A\n");

	return 0;
}

static ssize_t hx8526_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(update_fw, 0664, hx8526_update_fw_show,
		hx8526_update_fw_store);

static ssize_t hx8526_force_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, hx8526_update_fw_show,
		hx8526_force_update_fw_store);

static ssize_t hx8526_fw_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 8, "%s\n", "N/A\n");
}

static ssize_t hx8526_fw_name_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(fw_name, 0664, hx8526_fw_name_show,
			hx8526_fw_name_store);


/**********************debug file***************/
static int debug_dump_info(struct seq_file *m, void *v)
{
	struct himax_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner = THIS_MODULE,
	.open = debugfs_dump_info_open,
	.read = seq_read,
	.release = single_release,
};

static int debug_suspend_set(void *_data, u64 val)
{
	struct himax_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		himax852xes_suspend(&data->client->dev);
	else
		himax852xes_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int debug_suspend_get(void *_data, u64 *val)
{
	struct himax_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, debug_suspend_get,
		debug_suspend_set, "%lld\n");

static bool debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("hx8526 reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int debug_data_set(void *_data, u64 val)
{
	struct himax_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int debug_data_get(void *_data, u64 *val)
{
	struct himax_ts_data *data = _data;
	u8 reg = 0;

	mutex_lock(&data->input_dev->mutex);

	if (debug_addr_is_valid(data->addr)) {
		i2c_himax_read(data->client, data->addr, &reg, 1, 1);
		*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, debug_data_get,
		debug_data_set, "0x%02llX\n");

static int debug_addr_set(void *_data, u64 val)
{
	struct himax_ts_data *data = _data;

	if (debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int debug_addr_get(void *_data, u64 *val)
{
	struct himax_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, debug_addr_get,
		debug_addr_set, "0x%02llX\n");

/********************************************************/


static int himax_ManualMode(int enter)
{
	uint8_t cmd[2];

	cmd[0] = enter;
	if (i2c_himax_write(private_ts->client,
			0x42, &cmd[0], 1, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_FlashMode(int enter)
{
	uint8_t cmd[2];

	cmd[0] = enter;
	if (i2c_himax_write(private_ts->client,
			0x43, &cmd[0], 1, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_lock_flash(int enable)
{
	uint8_t cmd[5];

	if (i2c_himax_write(private_ts->client,
			0xAA, &cmd[0], 0, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}

	/* lock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write(private_ts->client,
		0x43, &cmd[0], 3, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client,
			0x44, &cmd[0], 3, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}

	if (enable != 0) {
		cmd[0] = 0x63;
		cmd[1] = 0x02;
		cmd[2] = 0x70;
		cmd[3] = 0x03;
	} else {
		cmd[0] = 0x63;
		cmd[1] = 0x02;
		cmd[2] = 0x30;
		cmd[3] = 0x00;
	}

	if (i2c_himax_write(private_ts->client,
			0x45, &cmd[0], 4, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(private_ts->client, 0x4A, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}
	msleep(50);

	if (i2c_himax_write(private_ts->client, 0xA9, &cmd[0], 0, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}

	return 0;
	/* lock sequence stop */
}


static void himax_changeIref(int selected_iref)
{

	unsigned char temp_iref[16][2] = {
			{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00},
			{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00},
			{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00},
			{0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}, {0x00, 0x00}
			};
	uint8_t cmd[10];
	int i = 0;
	int j = 0;

	if (i2c_himax_write(private_ts->client, 0xAA, &cmd[0], 0, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

	for (i = 0; i < 16; i++) {
		for (j = 0; j < 2; j++) {
			if (selected_iref == 1)
				temp_iref[i][j] = E_IrefTable_1[i][j];
			else if (selected_iref == 2)
				temp_iref[i][j] = E_IrefTable_2[i][j];
			else if (selected_iref == 3)
				temp_iref[i][j] = E_IrefTable_3[i][j];
			else if (selected_iref == 4)
				temp_iref[i][j] = E_IrefTable_4[i][j];
			else if (selected_iref == 5)
				temp_iref[i][j] = E_IrefTable_5[i][j];
			else if (selected_iref == 6)
				temp_iref[i][j] = E_IrefTable_6[i][j];
			else if (selected_iref == 7)
				temp_iref[i][j] = E_IrefTable_7[i][j];
		}
	}

	if (!iref_found) {
		/*Read Iref
		Register 0x43*/
		cmd[0] = 0x01;
		cmd[1] = 0x00;
		cmd[2] = 0x0A;
		if (i2c_himax_write(private_ts->client,
				0x43, &cmd[0], 3, 3) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return;
		}

		/*Register 0x44*/
		cmd[0] = 0x00;
		cmd[1] = 0x00;
		cmd[2] = 0x00;
		if (i2c_himax_write(private_ts->client,
				0x44, &cmd[0], 3, 3) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return;
		}

		/*Register 0x46*/
		if (i2c_himax_write(private_ts->client,
				0x46, &cmd[0], 0, 3) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return;
		}

		/*Register 0x59*/
		if (i2c_himax_read(private_ts->client,
				0x59, cmd, 4, 3) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return;
		}

		/*find iref group , default is iref 3*/
		for (i = 0; i < 16; i++) {
			if ((cmd[0] == temp_iref[i][0]) &&
				(cmd[1] == temp_iref[i][1])) {
				iref_number = i;
				iref_found = true;
				break;
			}
		}

		if (!iref_found) {
			pr_err("%s: Can't find iref number!\n", __func__);
			return;
		}
	}

	usleep_range(1000, 5000);

	/*iref write
	//Register 0x43*/
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 3, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

	/*Register 0x44*/
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

	/*Register 0x45*/
	cmd[0] = temp_iref[iref_number][0];
	cmd[1] = temp_iref[iref_number][1];
	cmd[2] = 0x17;
	cmd[3] = 0x28;

	if (i2c_himax_write(private_ts->client, 0x45, &cmd[0], 4, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

	/*Register 0x4A*/
	if (i2c_himax_write(private_ts->client, 0x4A, &cmd[0], 0, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

	/*Read SFR to check the result
	//Register 0x43*/
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x0A;
	if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 3, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

	/*Register 0x44*/
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

	/*Register 0x46*/
	if (i2c_himax_write(private_ts->client, 0x46, &cmd[0], 0, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

	/*Register 0x59*/
	if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}


	if (cmd[0] != temp_iref[iref_number][0] ||
			cmd[1] != temp_iref[iref_number][1]) {
		pr_err("%s: IREF Read Back is not match.\n", __func__);
		pr_err("%s: Iref [0]=%d,[1]=%d\n",
					__func__, cmd[0], cmd[1]);
	}

	if (i2c_himax_write(private_ts->client, 0xA9, &cmd[0], 0, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}

}

static uint8_t himax_calculateChecksum(bool change_iref)
{
	int iref_flag = 0;
	uint8_t cmd[10];

	memset(cmd, 0x00, sizeof(cmd));
/*Sleep outi*/
	if (i2c_himax_write(private_ts->client, 0x81,
		&cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}
msleep(120);

while (true) {
	if (change_iref) {
		if (iref_flag == 0)
			himax_changeIref(2); /*iref 2*/
		else if (iref_flag == 1)
			himax_changeIref(5); /*iref 5*/
		else if (iref_flag == 2)
			himax_changeIref(1); /*iref 1*/
		else
			goto CHECK_FAIL;

		iref_flag++;
	}

	cmd[0] = 0x00;
	cmd[1] = 0x04;
	cmd[2] = 0x0A;
	cmd[3] = 0x02;

	if (i2c_himax_write(private_ts->client, 0xED,
			&cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
			return 0;
	}

	/*Enable Flash*/
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x02;

	if (i2c_himax_write(private_ts->client, 0x43, &cmd[0],
					3, DEFAULT_RETRY_CNT) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}
	cmd[0] = 0x05;
	if (i2c_himax_write(private_ts->client, 0xD2,
		&cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x01;
	if (i2c_himax_write(private_ts->client, 0x53,
			&cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return 0;
	}

	msleep(200);

	if (i2c_himax_read(private_ts->client, 0xAD,
			cmd, 4, DEFAULT_RETRY_CNT) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return -ENODATA;
	}


	if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0) {
		himax_FlashMode(0);
		goto CHECK_PASS;
	} else {
		himax_FlashMode(0);
		goto CHECK_FAIL;
	}

CHECK_PASS:
	if (change_iref) {
		if (iref_flag < 3)
			continue;
		else
			return 1;
	} else
		return 1;
CHECK_FAIL:
	return 0;
}
	return 0;
}

int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw,
				int len, bool change_iref)
{
	unsigned char *ImageBuffer = fw;
	int fullFileLength = len;
	int i, j;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;

	/*Try 3 Times*/
	for (j = 0; j < 3; j++) {
		FileLength = fullFileLength;

		#ifdef HX_RST_PIN_FUNC
		himax_HW_reset(false, false);
		#endif

		if (i2c_himax_write(private_ts->client, 0x81,
			&cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return 0;
		}

		msleep(120);

		himax_lock_flash(0);

		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		if (i2c_himax_write(private_ts->client, 0x43,
			&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if (i2c_himax_write(private_ts->client, 0x4F,
			&cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return 0;
		}
		msleep(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++) {
			last_byte = 0;
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
				last_byte = 1;

			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write(private_ts->client, 0x44,
				&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
				pr_err("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (prePage != cmd[1] || i == 0) {
				prePage = cmd[1];
				cmd[0] = 0x01;
				cmd[1] = 0x09;/*cmd[2] = 0x02;*/
				if (i2c_himax_write(private_ts->client, 0x43,
					&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
					pr_err("%s: i2c access fail!\n",
								__func__);
					return 0;
				}

			cmd[0] = 0x01;
			cmd[1] = 0x0D;/*cmd[2] = 0x02;*/
			if (i2c_himax_write(private_ts->client, 0x43,
				&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_err("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x09;/*cmd[2] = 0x02;*/
			if (i2c_himax_write(private_ts->client, 0x43,
				&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_err("%s: i2c access fail!\n", __func__);
				return 0;
			}
		}

		memcpy(&cmd[0], &ImageBuffer[4*i], 4);
		if (i2c_himax_write(private_ts->client, 0x45,
				&cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;
		cmd[1] = 0x0D;/*cmd[2] = 0x02;*/
		if (i2c_himax_write(private_ts->client, 0x43,
				&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;
		cmd[1] = 0x09;/*cmd[2] = 0x02;*/
		if (i2c_himax_write(private_ts->client, 0x43,
			&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_err("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if (last_byte == 1) {
			cmd[0] = 0x01;
			cmd[1] = 0x01;/*cmd[2] = 0x02;*/
			if (i2c_himax_write(private_ts->client, 0x43,
				&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_err("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x05;/*cmd[2] = 0x02;*/
			if (i2c_himax_write(private_ts->client, 0x43,
				&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_err("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x01;/*cmd[2] = 0x02;*/
			if (i2c_himax_write(private_ts->client, 0x43,
				&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_err("%s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01;
			cmd[1] = 0x00;/*cmd[2] = 0x02;*/
			if (i2c_himax_write(private_ts->client, 0x43,
				&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
				pr_err("%s: i2c access fail!\n", __func__);
				return 0;
			}

			usleep_range(5000, 10000);
		if (i == (FileLength - 1)) {
			himax_FlashMode(0);
			himax_ManualMode(0);
			checksumResult = himax_calculateChecksum(change_iref);
			/*himax_ManualMode(0);*/
			himax_lock_flash(1);

			if (checksumResult) /*Success*/
				return 1;

			pr_err("%s: checksumResult fail!\n", __func__);
					/*return 0;*/

		}
		}
		}
	}
	return 0;
}

static int himax_input_register(struct himax_ts_data *ts)
{
	int ret;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		pr_err("%s: Failed to allocate input device\n", __func__);
		return ret;
	}
	ts->input_dev->name = "himax-touchscreen";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#if defined(HX_SMART_WAKEUP) || defined(HX_PALM_REPORT)
	set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
		0, 3, 0, 0);
	} else {/* PROTOCOL_TYPE_B */
		set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
		input_mt_init_slots(ts->input_dev, ts->nFinger_support, 0);
	}


	input_set_abs_params(ts->input_dev,
		ABS_MT_POSITION_X, ts->pdata->abs_x_min,
		ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_POSITION_Y, ts->pdata->abs_y_min,
		ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_TOUCH_MAJOR, ts->pdata->abs_pressure_min,
		ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_PRESSURE, ts->pdata->abs_pressure_min,
		ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev,
		ABS_MT_WIDTH_MAJOR, ts->pdata->abs_width_min,
		ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);


	return input_register_device(ts->input_dev);
}

static void calcDataSize(uint8_t finger_num)
{
	struct himax_ts_data *ts_data = private_ts;

	ts_data->coord_data_size = 4 * finger_num;
	ts_data->area_data_size = ((finger_num / 4) +
			(finger_num % 4 ? 1 : 0)) * 4;
	ts_data->raw_data_frame_size =
		128 - ts_data->coord_data_size -
		ts_data->area_data_size - 4 - 4 - 1;
	ts_data->raw_data_nframes =
		((uint32_t)ts_data->x_channel *
			ts_data->y_channel + ts_data->x_channel +
			ts_data->y_channel) / ts_data->raw_data_frame_size +
			(((uint32_t)ts_data->x_channel * ts_data->y_channel +
			ts_data->x_channel + ts_data->y_channel) %
			ts_data->raw_data_frame_size) ? 1 : 0;
}

static void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = HX_MAX_PT * 4;

	if ((HX_MAX_PT % 4) == 0)
		HX_TOUCH_INFO_POINT_CNT += (HX_MAX_PT / 4) * 4;
	else
		HX_TOUCH_INFO_POINT_CNT += ((HX_MAX_PT / 4) + 1) * 4;
}
#ifdef HX_LOADIN_CONFIG
static int himax_config_reg_write(struct i2c_client *client,
		uint8_t StartAddr, uint8_t *data,
		uint8_t length, uint8_t toRetry)
{
	char cmd[12] = {0};

	cmd[0] = 0x8C;
	cmd[1] = 0x14;
	if ((i2c_himax_master_write(client, &cmd[0], 2, toRetry)) < 0)
		return -ENODATA;

	cmd[0] = 0x8B;
	cmd[1] = 0x00;
	cmd[2] = StartAddr;
	if ((i2c_himax_master_write(client, &cmd[0], 3, toRetry)) < 0)
		return -ENODATA;

	if ((i2c_himax_master_write(client, data, length, toRetry)) < 0)
		return -ENODATA;

	cmd[0] = 0x8C;
	cmd[1] = 0x00;
	if ((i2c_himax_master_write(client, &cmd[0], 2, toRetry)) < 0)
		return -ENODATA;

	return 0;
}
#endif
void himax_touch_information(void)
{
	char data[12] = {0};

	if (IC_TYPE == HX_85XX_ES_SERIES_PWON) {
		data[0] = 0x8C;
		data[1] = 0x14;
		i2c_himax_master_write(private_ts->client,
				&data[0], 2, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
		data[0] = 0x8B;
		data[1] = 0x00;
		data[2] = 0x70;
		i2c_himax_master_write(private_ts->client,
				&data[0], 3, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
		i2c_himax_read(private_ts->client,
			0x5A, data, 12, DEFAULT_RETRY_CNT);
		HX_RX_NUM = data[0];
		HX_TX_NUM = data[1];
		HX_MAX_PT = (data[2] & 0xF0) >> 4;
#ifdef HX_EN_SEL_BUTTON
		HX_BT_NUM = (data[2] & 0x0F);
#endif
		if ((data[4] & 0x04) == 0x04) {
			HX_XY_REVERSE = true;
			HX_Y_RES = data[6]*256 + data[7];
			HX_X_RES = data[8]*256 + data[9];
		} else {
			HX_XY_REVERSE = false;
			HX_X_RES = data[6]*256 + data[7];
			HX_Y_RES = data[8]*256 + data[9];
		}
		data[0] = 0x8C;
		data[1] = 0x00;
		i2c_himax_master_write(private_ts->client,
			&data[0], 2, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
#ifdef HX_EN_MUT_BUTTON
		data[0] = 0x8C;
		data[1] = 0x14;
		i2c_himax_master_write(private_ts->client,
			&data[0], 2, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
		data[0] = 0x8B;
		data[1] = 0x00;
		data[2] = 0x64;
		i2c_himax_master_write(private_ts->client,
			&data[0], 3, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
		i2c_himax_read(private_ts->client, 0x5A,
				data, 4, DEFAULT_RETRY_CNT);
		HX_BT_NUM = (data[0] & 0x03);
		data[0] = 0x8C;
		data[1] = 0x00;
		i2c_himax_master_write(private_ts->client,
			&data[0], 2, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
#endif
#ifdef HX_TP_PROC_2T2R
		data[0] = 0x8C;
		data[1] = 0x14;
		i2c_himax_master_write(private_ts->client,
			&data[0], 2, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);

		data[0] = 0x8B;
		data[1] = 0x00;
		data[2] = HX_2T2R_Addr;
		i2c_himax_master_write(private_ts->client,
				&data[0], 3, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);

		i2c_himax_read(private_ts->client, 0x5A,
				data, 10, DEFAULT_RETRY_CNT);

		HX_RX_NUM_2 = data[0];
		HX_TX_NUM_2 = data[1];

		if (data[2] == HX_2T2R_en_setting)
			Is_2T2R = true;
		else
			Is_2T2R = false;

		data[0] = 0x8C;
		data[1] = 0x00;
		i2c_himax_master_write(private_ts->client,
				&data[0], 2, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
#endif
		data[0] = 0x8C;
		data[1] = 0x14;
		i2c_himax_master_write(private_ts->client,
				&data[0], 2, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
		data[0] = 0x8B;
		data[1] = 0x00;
		data[2] = 0x02;
		i2c_himax_master_write(private_ts->client,
				&data[0], 3, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);
		i2c_himax_read(private_ts->client, 0x5A,
			data, 10, DEFAULT_RETRY_CNT);
		if ((data[1] & 0x01) == 1)
			HX_INT_IS_EDGE = true;
		 else
			HX_INT_IS_EDGE = false;

		data[0] = 0x8C;
		data[1] = 0x00;
		i2c_himax_master_write(private_ts->client,
				&data[0], 2, DEFAULT_RETRY_CNT);
		usleep_range(5000, 10000);

		if (i2c_himax_read(private_ts->client,
			HX_VER_FW_CFG, data, 1, 3) < 0)
			pr_err("%s: i2c access fail!\n", __func__);

			private_ts->vendor_config_ver = data[0];
		} else {
			HX_RX_NUM = 0;
			HX_TX_NUM = 0;
			HX_BT_NUM = 0;
			HX_X_RES = 0;
			HX_Y_RES = 0;
			HX_MAX_PT = 0;
			HX_XY_REVERSE = false;
			HX_INT_IS_EDGE = false;
		}
}
static uint8_t himax_read_Sensor_ID(struct i2c_client *client)
{
	uint8_t val_high[1], val_low[1], ID0 = 0, ID1 = 0;
	uint8_t sensor_id;
	char data[3];
	const int normalRetry = 10;

	data[0] = 0x56;
	data[1] = 0x02;
	data[2] = 0x02;/*ID pin PULL High*/
	i2c_himax_master_write(client, &data[0], 3, normalRetry);
	usleep_range(500, 1000);

	/*read id pin high*/
	i2c_himax_read(client, 0x57, val_high, 1, normalRetry);

	data[0] = 0x56;
	data[1] = 0x01;
	data[2] = 0x01;/*ID pin PULL Low*/
	i2c_himax_master_write(client, &data[0], 3, normalRetry);
	usleep_range(500, 1000);

	/*read id pin low*/
	i2c_himax_read(client, 0x57, val_low, 1, normalRetry);

	if ((val_high[0] & 0x01) == 0)
		ID0 = 0x02;/*GND*/
	else if ((val_low[0] & 0x01) == 0)
		ID0 = 0x01;/*Floating*/
	else
		ID0 = 0x04;/*VCC*/

	if ((val_high[0] & 0x02) == 0)
		ID1 = 0x02;/*GND*/
	else if ((val_low[0] & 0x02) == 0)
		ID1 = 0x01;/*Floating*/
	else
		ID1 = 0x04;/*VCC*/
	if ((ID0 == 0x04) && (ID1 != 0x04)) {
		data[0] = 0x56;
		data[1] = 0x02;
		data[2] = 0x01;/*ID pin PULL High,Low*/
		i2c_himax_master_write(client, &data[0], 3, normalRetry);
		usleep_range(500, 1000);

	} else if ((ID0 != 0x04) && (ID1 == 0x04)) {
		data[0] = 0x56;
		data[1] = 0x01;
		data[2] = 0x02;/*ID pin PULL Low,High*/
		i2c_himax_master_write(client, &data[0], 3, normalRetry);
		usleep_range(500, 1000);

	} else if ((ID0 == 0x04) && (ID1 == 0x04)) {
		data[0] = 0x56;
		data[1] = 0x02;
		data[2] = 0x02;/*ID pin PULL High,High*/
		i2c_himax_master_write(client, &data[0], 3, normalRetry);
		usleep_range(500, 1000);

	}
	sensor_id = (ID1<<4)|ID0;

	data[0] = 0xF3;
	data[1] = sensor_id;
	i2c_himax_master_write(client, &data[0],
				2, normalRetry);/*Write to MCU*/
	usleep_range(500, 1000);

	return sensor_id;

}
static void himax_power_on_initCMD(struct i2c_client *client)
{
	const int normalRetry = 10;

	dev_info(&client->dev, "%s:\n", __func__);
	/*Sense on to update the information*/
	i2c_himax_write_command(client, 0x83, normalRetry);
	msleep(30);

	i2c_himax_write_command(client, 0x81, normalRetry);
	msleep(50);

	i2c_himax_write_command(client, 0x82, normalRetry);
	msleep(50);

	i2c_himax_write_command(client, 0x80, normalRetry);
	msleep(50);

	himax_touch_information();

	i2c_himax_write_command(client, 0x83, normalRetry);
	msleep(30);

	i2c_himax_write_command(client, 0x81, normalRetry);
	msleep(50);
}

#ifdef HX_AUTO_UPDATE_FW
static int i_update_FW(void)
{
	unsigned char *ImageBuffer = i_CTPM_FW;
	int fullFileLength = sizeof(i_CTPM_FW);

	if ((private_ts->vendor_fw_ver_H < i_CTPM_FW[FW_VER_MAJ_FLASH_ADDR])
		|| (private_ts->vendor_fw_ver_L <
			i_CTPM_FW[FW_VER_MIN_FLASH_ADDR])
		|| (private_ts->vendor_config_ver <
			i_CTPM_FW[FW_CFG_VER_FLASH_ADDR])) {
		if (fts_ctpm_fw_upgrade_with_sys_fs(ImageBuffer,
					fullFileLength, true) == 0)
			pr_err("%s: TP upgrade error\n", __func__);
		else {
			private_ts->vendor_fw_ver_H =
				i_CTPM_FW[FW_VER_MAJ_FLASH_ADDR];
			private_ts->vendor_fw_ver_L =
				i_CTPM_FW[FW_VER_MIN_FLASH_ADDR];
			private_ts->vendor_config_ver =
				i_CTPM_FW[FW_CFG_VER_FLASH_ADDR];
		}
#ifdef HX_RST_PIN_FUNC
			himax_HW_reset(false, false);
#endif
			return 1;
		}
	else
		return 0;
}
#endif

#ifdef HX_AUTO_UPDATE_CONFIG
int fts_ctpm_fw_upgrade_with_i_file_flash_cfg(struct himax_config *cfg)
{
	unsigned char *ImageBuffer = i_CTPM_FW;
	int fullFileLength = FLASH_SIZE;
	int i, j, k;
	uint8_t cmd[5], last_byte, prePage;
	uint8_t tmp[5];
	int FileLength;
	int Polynomial = 0x82F63B78;
	int CRC32 = 0xFFFFFFFF;
	int BinDataWord = 0;
	int current_index = 0;
	uint8_t checksumResult = 0;

	current_index = CFB_START_ADDR + CFB_INFO_LENGTH;

	/*Update the Config Part from config array*/
	for (i = 1 ; i < sizeof(cfg->c1)/sizeof(cfg->c1[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c1[i];
	for (i = 1 ; i < sizeof(cfg->c2)/sizeof(cfg->c2[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c2[i];
	for (i = 1 ; i < sizeof(cfg->c3)/sizeof(cfg->c3[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c3[i];
	for (i = 1 ; i < sizeof(cfg->c4)/sizeof(cfg->c4[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c4[i];
	for (i = 1 ; i < sizeof(cfg->c5)/sizeof(cfg->c5[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c5[i];
	for (i = 1 ; i < sizeof(cfg->c6)/sizeof(cfg->c6[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c6[i];
	for (i = 1 ; i < sizeof(cfg->c7)/sizeof(cfg->c7[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c7[i];
	for (i = 1 ; i < sizeof(cfg->c8)/sizeof(cfg->c8[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c8[i];
	for (i = 1 ; i < sizeof(cfg->c9)/sizeof(cfg->c9[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c9[i];
	for (i = 1 ; i < sizeof(cfg->c10)/sizeof(cfg->c10[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c10[i];
	for (i = 1 ; i < sizeof(cfg->c11)/sizeof(cfg->c11[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c11[i];
	for (i = 1 ; i < sizeof(cfg->c12)/sizeof(cfg->c12[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c12[i];
	for (i = 1 ; i < sizeof(cfg->c13)/sizeof(cfg->c13[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c13[i];
	for (i = 1 ; i < sizeof(cfg->c14)/sizeof(cfg->c14[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c14[i];
	for (i = 1 ; i < sizeof(cfg->c15)/sizeof(cfg->c15[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c15[i];
	for (i = 1 ; i < sizeof(cfg->c16)/sizeof(cfg->c16[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c16[i];
	for (i = 1 ; i < sizeof(cfg->c17)/sizeof(cfg->c17[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c17[i];
	for (i = 1 ; i < sizeof(cfg->c18)/sizeof(cfg->c18[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c18[i];
	for (i = 1 ; i < sizeof(cfg->c19)/sizeof(cfg->c19[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c19[i];
	for (i = 1 ; i < sizeof(cfg->c20)/sizeof(cfg->c20[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c20[i];
	for (i = 1 ; i < sizeof(cfg->c21)/sizeof(cfg->c21[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c21[i];
	for (i = 1 ; i < sizeof(cfg->c22)/sizeof(cfg->c22[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c22[i];
	for (i = 1 ; i < sizeof(cfg->c23)/sizeof(cfg->c23[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c23[i];
	for (i = 1 ; i < sizeof(cfg->c24)/sizeof(cfg->c24[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c24[i];
	for (i = 1 ; i < sizeof(cfg->c25)/sizeof(cfg->c25[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c25[i];
	for (i = 1 ; i < sizeof(cfg->c26)/sizeof(cfg->c26[0]) ; i++)
		ImageBuffer[current_index++] = cfg->c26[i];
	for (i = 1; i < sizeof(cfg->c27)/sizeof(cfg->c27[0]); i++)
		ImageBuffer[current_index++] = cfg->c27[i];
	for (i = 1; i < sizeof(cfg->c28)/sizeof(cfg->c28[0]); i++)
		ImageBuffer[current_index++] = cfg->c28[i];
	for (i = 1; i < sizeof(cfg->c29)/sizeof(cfg->c29[0]); i++)
		ImageBuffer[current_index++] = cfg->c29[i];
	for (i = 1; i < sizeof(cfg->c30)/sizeof(cfg->c30[0]); i++)
		ImageBuffer[current_index++] = cfg->c30[i];
	for (i = 1; i < sizeof(cfg->c31)/sizeof(cfg->c31[0]); i++)
		ImageBuffer[current_index++] = cfg->c31[i];
	for (i = 1; i < sizeof(cfg->c32)/sizeof(cfg->c32[0]); i++)
		ImageBuffer[current_index++] = cfg->c32[i];
	for (i = 1; i < sizeof(cfg->c33)/sizeof(cfg->c33[0]); i++)
		ImageBuffer[current_index++] = cfg->c33[i];
	for (i = 1; i < sizeof(cfg->c34)/sizeof(cfg->c34[0]); i++)
		ImageBuffer[current_index++] = cfg->c34[i];
	for (i = 1; i < sizeof(cfg->c35)/sizeof(cfg->c35[0]); i++)
		ImageBuffer[current_index++] = cfg->c35[i];
	for (i = 1; i < sizeof(cfg->c36)/sizeof(cfg->c36[0]); i++)
		ImageBuffer[current_index++] = cfg->c36[i];
	for (i = 1; i < sizeof(cfg->c37)/sizeof(cfg->c37[0]); i++)
		ImageBuffer[current_index++] = cfg->c37[i];
	for (i = 1; i < sizeof(cfg->c38)/sizeof(cfg->c38[0]); i++)
		ImageBuffer[current_index++] = cfg->c38[i];
	for (i = 1; i < sizeof(cfg->c39)/sizeof(cfg->c39[0]); i++)
		ImageBuffer[current_index++] = cfg->c39[i];
		current_index = current_index+50;
	for (i = 1; i < sizeof(cfg->c40)/sizeof(cfg->c40[0]); i++)
		ImageBuffer[current_index++] = cfg->c40[i];
	for (i = 1; i < sizeof(cfg->c41)/sizeof(cfg->c41[0]); i++)
		ImageBuffer[current_index++] = cfg->c41[i];

	/*cal_checksum start*/
	FileLength = fullFileLength;
	FileLength = (FileLength + 3) / 4;
	for (i = 0; i < FileLength; i++) {
		memcpy(&cmd[0], &ImageBuffer[4*i], 4);
	if (i < (FileLength - 1)) {/*cal_checksum*/
		for (k = 0; k < 4; k++)
			BinDataWord |= cmd[k] << (k * 8);

		CRC32 = BinDataWord ^ CRC32;
		for (k = 0; k < 32; k++) {
			if ((CRC32 % 2) != 0)
				CRC32 = ((CRC32 >> 1) &
					0x7FFFFFFF) ^ Polynomial;
			else
				CRC32 = ((CRC32 >> 1) & 0x7FFFFFFF);
		}
		BinDataWord = 0;
	}
	}
	/*cal_checksum end*/

	/*Try 3 Times*/
	for (j = 0; j < 3; j++) {
		FileLength = fullFileLength;

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset(false, false);
#endif

	if (i2c_himax_write(private_ts->client, 0x81,
		&cmd[0], 0, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}

	msleep(120);

	himax_lock_flash(0);

	himax_ManualMode(1);
	himax_FlashMode(1);

	FileLength = (FileLength + 3) / 4;
	for (i = 0, prePage = 0; i < FileLength; i++) {
		last_byte = 0;

		if (i < 0x20)
			continue;
		else if ((i > 0xBF) && (i < (FileLength - 0x20)))
			continue;

	cmd[0] = i & 0x1F;
	if (cmd[0] == 0x1F || i == FileLength - 1)
		last_byte = 1;

	cmd[1] = (i >> 5) & 0x1F;
	cmd[2] = (i >> 10) & 0x1F;

	if (i2c_himax_write(private_ts->client, 0x44,
			&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (prePage != cmd[1] || i == 0x20) {
		prePage = cmd[1];
		tmp[0] = 0x01;
		tmp[1] = 0x09;
	if (i2c_himax_write(private_ts->client, 0x43,
			&tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_err(" %s: i2c access fail!\n", __func__);
			return 0;
	}

	tmp[0] = 0x05;
	tmp[1] = 0x2D;
	if (i2c_himax_write(private_ts->client, 0x43,
		&tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}
		msleep(30);
		tmp[0] = 0x01;
		tmp[1] = 0x09;
	if (i2c_himax_write(private_ts->client, 0x43,
			&tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write(private_ts->client, 0x44,
		&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}
	tmp[0] = 0x01;
	tmp[1] = 0x09;
	if (i2c_himax_write(private_ts->client, 0x43,
		&tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}

	tmp[0] = 0x01;
	tmp[1] = 0x0D;
	if (i2c_himax_write(private_ts->client, 0x43,
		&tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}
	tmp[0] = 0x01;
	tmp[1] = 0x09;
	if (i2c_himax_write(private_ts->client, 0x43,
		&tmp[0], 2, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write(private_ts->client, 0x44,
		&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}
}

	memcpy(&cmd[0], &ImageBuffer[4*i], 4);
	if (i == (FileLength - 1)) {
		tmp[0] = (CRC32 & 0xFF);
		tmp[1] = ((CRC32 >> 8) & 0xFF);
		tmp[2] = ((CRC32 >> 16) & 0xFF);
		tmp[3] = ((CRC32 >> 24) & 0xFF);

		memcpy(&cmd[0], &tmp[0], 4);
	}

	if (i2c_himax_write(private_ts->client, 0x45,
		&cmd[0], 4, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}
	cmd[0] = 0x01;
	cmd[1] = 0x0D;
	/*cmd[2] = 0x02;*/
	if (i2c_himax_write(private_ts->client, 0x43,
		&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x01;
	cmd[1] = 0x09;/*cmd[2] = 0x02;*/
	if (i2c_himax_write(private_ts->client, 0x43,
		&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
		pr_err(" %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (last_byte == 1) {
		cmd[0] = 0x01;
		cmd[1] = 0x01;/*cmd[2] = 0x02;*/
		if (i2c_himax_write(private_ts->client, 0x43,
			&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_err(" %s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;
		cmd[1] = 0x05;/*cmd[2] = 0x02;*/
		if (i2c_himax_write(private_ts->client, 0x43,
			&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_err(" %s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;
		cmd[1] = 0x01;/*cmd[2] = 0x02;*/
		if (i2c_himax_write(private_ts->client, 0x43,
			&cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_err(" %s: i2c access fail!\n", __func__);
			return 0;
		}

		cmd[0] = 0x01;
		cmd[1] = 0x00;/*cmd[2] = 0x02;*/
		if (i2c_himax_write(private_ts->client,
			0x43 , &cmd[0], 2, DEFAULT_RETRY_CNT) < 0) {
			pr_err(" %s: i2c access fail!\n", __func__);
			return 0;
		}

		usleep_range(5000, 10000);
		if (i == (FileLength - 1)) {
			himax_FlashMode(0);
			himax_ManualMode(0);
			checksumResult = himax_calculateChecksum(true);
			/*himax_ManualMode(0);*/
			himax_lock_flash(1);

			if (checksumResult)
				return 1;
			else
				return 0;

		}
	}
	}
	}
	return 0;
}

static int i_update_FWCFG(struct himax_config *cfg)
{

	if (private_ts->vendor_config_ver != cfg->c40[1]) {
		if (fts_ctpm_fw_upgrade_with_i_file_flash_cfg(cfg) == 0)
			pr_err("%s: TP upgrade error\n", __func__);
#ifdef HX_RST_PIN_FUNC
		himax_HW_reset(false, false);
#endif
		return 1;
	} else
		return 0;
}

#endif

static int himax_loadSensorConfig(struct i2c_client *client,
			struct himax_i2c_platform_data *pdata)
{
#if defined(HX_LOADIN_CONFIG) || defined(HX_AUTO_UPDATE_CONFIG)
	int rc = 0;
#endif
#ifdef HX_LOADIN_CONFIG
	const int normalRetry = 10;
#endif
#ifndef CONFIG_OF
#if defined(HX_LOADIN_CONFIG) || defined(HX_AUTO_UPDATE_CONFIG)
	int i = 0;
#endif
#endif
#ifdef HX_ESD_WORKAROUND
	char data[12] = {0};
#endif

	if (!client) {
		pr_err("%s: Necessary parameters client are null!\n", __func__);
		return -ENOMEM;
	}

	if (config_load == false) {
			config_selected = kzalloc(
				sizeof(*config_selected), GFP_KERNEL);
			if (config_selected == NULL)
				return -ENOMEM;

		}
#ifndef CONFIG_OF
	pdata = client->dev.platform_data;
		if (!pdata) {
			pr_err("%s: Necessary parameters pdata are null!\n",
								__func__);
			return -EINVAL;
		}
#endif

#if defined(HX_LOADIN_CONFIG) || defined(HX_AUTO_UPDATE_CONFIG)
#ifdef CONFIG_OF
	if (config_load == false) {
		rc = himax_parse_config(private_ts, config_selected);
		if (rc < 0) {
			goto HimaxErr;
		} else if (rc == 0) {
			if ((private_ts->tw_x_max) && (private_ts->tw_y_max)) {
				pdata->abs_x_min = private_ts->tw_x_min;
				pdata->abs_x_max = private_ts->tw_x_max;
				pdata->abs_y_min = private_ts->tw_y_min;
				pdata->abs_y_max = private_ts->tw_y_max;
			}
			if ((private_ts->pl_x_max) && (private_ts->pl_y_max)) {
				pdata->screenWidth = private_ts->pl_x_max;
				pdata->screenHeight = private_ts->pl_y_max;
			}
				config_load = true;
			}
		}
#else

	if (pdata->hx_config) {
		for (i = 0; i < pdata->hx_config_size/
				sizeof(struct himax_config); ++i) {
			if ((private_ts->vendor_fw_ver_H << 8
				| private_ts->vendor_fw_ver_L) <
				((pdata->hx_config)[i].fw_ver_main
				<< 8 | (pdata->hx_config)[i].fw_ver_minor)) {
					continue;
			} else {
			if ((private_ts->vendor_sensor_id
				== (pdata->hx_config)[i].sensor_id)) {
				config_selected = &((pdata->hx_config)[i]);
				config_load = true;
					break;
			} else if ((pdata->hx_config)[i].default_cfg) {
				config_selected = &((pdata->hx_config)[i]);
				config_load = true;
				break;
			}
			}
		}
	} else
		goto HimaxErr;

#endif
#endif
#ifdef HX_LOADIN_CONFIG
	if (config_selected) {
		private_ts->vendor_config_ver = config_selected->c40[1];

		i2c_himax_master_write(client, config_selected->c1,
				sizeof(config_selected->c1), normalRetry);
		i2c_himax_master_write(client, config_selected->c2,
				sizeof(config_selected->c2), normalRetry);
		i2c_himax_master_write(client, config_selected->c3,
				sizeof(config_selected->c3), normalRetry);
		i2c_himax_master_write(client, config_selected->c4,
				sizeof(config_selected->c4), normalRetry);
		i2c_himax_master_write(client, config_selected->c5,
				sizeof(config_selected->c5), normalRetry);
		i2c_himax_master_write(client, config_selected->c6,
				sizeof(config_selected->c6), normalRetry);
		i2c_himax_master_write(client, config_selected->c7,
				sizeof(config_selected->c7), normalRetry);
		i2c_himax_master_write(client, config_selected->c8,
				sizeof(config_selected->c8), normalRetry);
		i2c_himax_master_write(client, config_selected->c9,
				sizeof(config_selected->c9), normalRetry);
		i2c_himax_master_write(client, config_selected->c10,
				sizeof(config_selected->c10), normalRetry);
		i2c_himax_master_write(client, config_selected->c11,
				sizeof(config_selected->c11), normalRetry);
		i2c_himax_master_write(client, config_selected->c12,
				sizeof(config_selected->c12), normalRetry);
		i2c_himax_master_write(client, config_selected->c13,
				sizeof(config_selected->c13), normalRetry);
		i2c_himax_master_write(client, config_selected->c14,
				sizeof(config_selected->c14), normalRetry);
		i2c_himax_master_write(client, config_selected->c15,
				sizeof(config_selected->c15), normalRetry);
		i2c_himax_master_write(client, config_selected->c16,
				sizeof(config_selected->c16), normalRetry);
		i2c_himax_master_write(client, config_selected->c17,
				sizeof(config_selected->c17), normalRetry);
		i2c_himax_master_write(client, config_selected->c18,
				sizeof(config_selected->c18), normalRetry);
		i2c_himax_master_write(client, config_selected->c19,
				sizeof(config_selected->c19), normalRetry);
		i2c_himax_master_write(client, config_selected->c20,
				sizeof(config_selected->c20), normalRetry);
		i2c_himax_master_write(client, config_selected->c21,
				sizeof(config_selected->c21), normalRetry);
		i2c_himax_master_write(client, config_selected->c22,
				sizeof(config_selected->c22), normalRetry);
		i2c_himax_master_write(client, config_selected->c23,
				sizeof(config_selected->c23), normalRetry);
		i2c_himax_master_write(client, config_selected->c24,
				sizeof(config_selected->c24), normalRetry);
		i2c_himax_master_write(client, config_selected->c25,
				sizeof(config_selected->c25), normalRetry);
		i2c_himax_master_write(client, config_selected->c26,
				sizeof(config_selected->c26), normalRetry);
		i2c_himax_master_write(client, config_selected->c27,
				sizeof(config_selected->c27), normalRetry);
		i2c_himax_master_write(client, config_selected->c28,
				sizeof(config_selected->c28), normalRetry);
		i2c_himax_master_write(client, config_selected->c29,
				sizeof(config_selected->c29), normalRetry);
		i2c_himax_master_write(client, config_selected->c30,
				sizeof(config_selected->c30), normalRetry);
		i2c_himax_master_write(client, config_selected->c31,
				sizeof(config_selected->c31), normalRetry);
		i2c_himax_master_write(client, config_selected->c32,
				sizeof(config_selected->c32), normalRetry);
		i2c_himax_master_write(client, config_selected->c33,
				sizeof(config_selected->c33), normalRetry);
		i2c_himax_master_write(client, config_selected->c34,
				sizeof(config_selected->c34), normalRetry);
		i2c_himax_master_write(client, config_selected->c35,
				sizeof(config_selected->c35), normalRetry);
		i2c_himax_master_write(client, config_selected->c36,
				sizeof(config_selected->c36), normalRetry);
		i2c_himax_master_write(client, config_selected->c37,
				sizeof(config_selected->c37), normalRetry);
		i2c_himax_master_write(client, config_selected->c38,
				sizeof(config_selected->c38), normalRetry);
		i2c_himax_master_write(client, config_selected->c39,
				sizeof(config_selected->c39), normalRetry);
			/*Config Bank register*/
			himax_config_reg_write(client, 0x00,
				config_selected->c40,
				sizeof(config_selected->c40), normalRetry);
			himax_config_reg_write(client, 0x9E,
				config_selected->c41,
				sizeof(config_selected->c41), normalRetry);

			usleep_range(500, 1000);
		} else
			goto HimaxErr;

#endif
#ifdef HX_ESD_WORKAROUND
	i2c_himax_read(client, 0x36, data, 2, 10);
	if (data[0] != 0x0F || data[1] != 0x53)
		return -ENODATA;

#endif

	himax_power_on_initCMD(client);
	return 1;
#if defined(HX_LOADIN_CONFIG) || defined(HX_AUTO_UPDATE_CONFIG)
HimaxErr:
	return -EINVAL;
#endif
}

#ifdef HX_RST_PIN_FUNC
void himax_HW_reset(uint8_t loadconfig, uint8_t int_off)
{
	struct himax_ts_data *ts = private_ts;
	int ret = 0;

	HW_RESET_ACTIVATE = 1;

	if (ts->rst_gpio) {
		if (!int_off) {
			if (ts->use_irq)
				himax_int_enable(private_ts->client->irq, 0);
			else {
				hrtimer_cancel(&ts->timer);
				ret = cancel_work_sync(&ts->work);
			}
		}


		himax_rst_gpio_set(ts->rst_gpio, 0);
		msleep(20);
		himax_rst_gpio_set(ts->rst_gpio, 1);
		msleep(20);

		if (loadconfig)
			himax_loadSensorConfig(private_ts->client,
						private_ts->pdata);

		if (!int_off) {
			if (ts->use_irq)
				himax_int_enable(private_ts->client->irq, 1);
			else
				hrtimer_start(&ts->timer,
					ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	}
}
#endif

/*static u8 himax_read_FW_ver(bool hw_reset)
{
	uint8_t cmd[3];

	himax_int_enable(private_ts->client->irq,0);

#ifdef HX_RST_PIN_FUNC
	if (hw_reset) {
		himax_HW_reset(false,false);
	}
#endif

	msleep(120);
	if (i2c_himax_read(private_ts->client,
			HX_VER_FW_MAJ, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_fw_ver_H = cmd[0];
	if (i2c_himax_read(private_ts->client,
			HX_VER_FW_MIN, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_fw_ver_L = cmd[0];

	if (i2c_himax_read(private_ts->client,
			HX_VER_FW_CFG, cmd, 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	private_ts->vendor_config_ver = cmd[0];

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset(true,false);
#endif

	himax_int_enable(private_ts->client->irq, 1);

	return 0;
}
*/
static bool himax_ic_package_check(struct himax_ts_data *ts)
{
	uint8_t cmd[3];

	memset(cmd, 0x00, sizeof(cmd));

	if (i2c_himax_read(ts->client, 0xD1, cmd, 3, DEFAULT_RETRY_CNT) < 0)
		return false;

	if (cmd[0] == 0x05 && cmd[1] == 0x85 &&
		(cmd[2] == 0x25 || cmd[2] == 0x26 ||
			cmd[2] == 0x27 || cmd[2] == 0x28)) {

		IC_TYPE  = HX_85XX_ES_SERIES_PWON;
		IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
		/*Himax: Set FW and CFG Flash Address*/
		FW_VER_MAJ_FLASH_ADDR = 133;
		FW_VER_MAJ_FLASH_LENG = 1;
		FW_VER_MIN_FLASH_ADDR = 134;  /*0x0086*/
		FW_VER_MIN_FLASH_LENG = 1;
		CFG_VER_MAJ_FLASH_ADDR = 160;
		CFG_VER_MAJ_FLASH_LENG = 12;
		CFG_VER_MIN_FLASH_ADDR = 172;
		CFG_VER_MIN_FLASH_LENG = 12;
		FW_CFG_VER_FLASH_ADDR = 132;  /*0x0084*/
#ifdef HX_AUTO_UPDATE_CONFIG
			CFB_START_ADDR = 0x80;
			CFB_LENGTH = 638;
			CFB_INFO_LENGTH = 68;
#endif
		} else
			return false;

	return true;
}

static void himax_read_TP_info(struct i2c_client *client)
{
	char data[12] = {0};

	if (i2c_himax_read(client, HX_VER_FW_MAJ, data, 1, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}
	private_ts->vendor_fw_ver_H = data[0];

	if (i2c_himax_read(client, HX_VER_FW_MIN, data, 1, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}
	private_ts->vendor_fw_ver_L = data[0];
	if (i2c_himax_read(client, HX_VER_FW_CFG, data, 1, 3) < 0) {
		pr_err("%s: i2c access fail!\n", __func__);
		return;
	}
	private_ts->vendor_config_ver = data[0];
	private_ts->vendor_sensor_id = himax_read_Sensor_ID(client);

	dev_info(&client->dev,
		"sensor_id=%x.\n", private_ts->vendor_sensor_id);
	dev_info(&client->dev,
		"fw_ver=%x,%x.\n",
		private_ts->vendor_fw_ver_H, private_ts->vendor_fw_ver_L);
	dev_info(&client->dev,
		"config_ver=%x.\n",
		private_ts->vendor_config_ver);
}

#ifdef HX_ESD_WORKAROUND
	void ESD_HW_REST(void)
	{
		ESD_RESET_ACTIVATE = 1;
		ESD_COUNTER = 0;
		ESD_R36_FAIL = 0;
#ifdef HX_CHIP_STATUS_MONITOR
		HX_CHIP_POLLING_COUNT = 0;
#endif
		dev_info(&client->dev, "START_Himax TP: ESD - Reset\n");

		while (ESD_R36_FAIL <= 3) {

			himax_rst_gpio_set(private_ts->rst_gpio, 0);
			msleep(20);
			himax_rst_gpio_set(private_ts->rst_gpio, 1);
			msleep(20);

		if (himax_loadSensorConfig(private_ts->client,
					private_ts->pdata) < 0)
			ESD_R36_FAIL++;
		else
			break;
		}
		dev_info(&client->dev, "END_Himax TP: ESD - Reset\n");
	}
#endif

#ifdef HX_CHIP_STATUS_MONITOR
static void himax_chip_monitor_function(struct work_struct *work)
{
	int ret = 0;

	if (HX_CHIP_POLLING_COUNT >= (HX_POLLING_TIMES-1)) {/*POLLING TIME*/
		HX_ON_HAND_SHAKING = 1;
		ret = himax_hand_shaking(); /*0:Running, 1:Stop, 2:I2C Fail*/
		HX_ON_HAND_SHAKING = 0;
		if (ret == 2)
			ESD_HW_REST();
		else if (ret == 1)
			ESD_HW_REST();

		HX_CHIP_POLLING_COUNT = 0;/*clear polling counter*/
	} else
		HX_CHIP_POLLING_COUNT++;

	queue_delayed_work(private_ts->himax_chip_monitor_wq,
		&private_ts->himax_chip_monitor, HX_POLLING_TIMER*HZ);

/*	return;*/
}
#endif

#ifdef HX_DOT_VIEW
static void himax_set_cover_func(unsigned int enable)
{
	uint8_t cmd[4];

	if (enable)
		cmd[0] = 0x40;
	else
		cmd[0] = 0x00;
	if (i2c_himax_write(private_ts->client, 0x8F, &cmd[0],
					1, DEFAULT_RETRY_CNT) < 0)
		pr_err("%s i2c write fail.\n", __func__);

/*	return;*/
}

static int hallsensor_hover_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;
	struct himax_ts_data *ts = private_ts;

	pole_value = 0x1 & status;
	pole = (0x2 & status) >> 1;

	if (pole == 1) {
		if (pole_value == 0)
			ts->cover_enable = 0;
		else
			ts->cover_enable = 1;

		himax_set_cover_func(ts->cover_enable);
	}

	return NOTIFY_OK;
}

static struct notifier_block hallsensor_status_handler = {
	.notifier_call = hallsensor_hover_status_handler_func,
};
#endif

#ifdef HX_SMART_WAKEUP
static int himax_parse_wake_event(struct himax_ts_data *ts)
{
	uint8_t buf[5];

	if (i2c_himax_read(ts->client, 0x86, buf, 4, HIMAX_I2C_RETRY_TIMES))
		pr_err("%s: can't read data from chip!\n", __func__);

	if ((buf[0] == 0x57) && (buf[1] == 0x61) &&
			(buf[2] == 0x6B) && (buf[3] == 0x65)) {
			dev_info(&ts->client->dev,
				"%s: WAKE UP system!\n", __func__);
			return 1;
		}

	dev_info(&ts->client->dev,
		"%s: NOT WKAE packet, SKIP!\n", __func__);
	dev_info(&ts->client->dev,
		"buf[0]=%x, buf[1]=%x, buf[2]=%x, buf[3]=%x\n",
				buf[0], buf[1], buf[2], buf[3]);
	return 0;

}
#endif

static void himax_ts_button_func(int tp_key_index,
				struct himax_ts_data *ts)
{
	uint16_t x_position = 0, y_position = 0;

if (tp_key_index != 0x00) {
	dev_info(&ts->client->dev, "virtual key index =%x\n", tp_key_index);
	if (tp_key_index == 0x01) {
		vk_press = 1;
		dev_info(&ts->client->dev, "back key pressed\n");
		if (ts->pdata->virtual_key) {
			if (ts->button[0].index) {
				x_position = (ts->button[0].x_range_min +
						ts->button[0].x_range_max) / 2;
				y_position = (ts->button[0].y_range_min +
						ts->button[0].y_range_max) / 2;
			}
			if (ts->protocol_type == PROTOCOL_TYPE_A) {
				input_report_abs(ts->input_dev,
					ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev,
					ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev,
					ABS_MT_WIDTH_MAJOR, 100);
				input_report_abs(ts->input_dev,
					ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_Y, y_position);
				input_mt_sync(ts->input_dev);
			} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev,
					ABS_MT_TOUCH_MAJOR, 100);
				input_report_abs(ts->input_dev,
					ABS_MT_WIDTH_MAJOR, 100);
				input_report_abs(ts->input_dev,
					ABS_MT_PRESSURE, 100);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_X, x_position);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_Y, y_position);
			}
		} else
			input_report_key(ts->input_dev, KEY_BACK, 1);
	} else if (tp_key_index == 0x02) {
		vk_press = 1;
		dev_info(&ts->client->dev, "home key pressed\n");
		if (ts->pdata->virtual_key) {
			if (ts->button[1].index) {
				x_position = (ts->button[1].x_range_min +
						ts->button[1].x_range_max) / 2;
				y_position = (ts->button[1].y_range_min +
						ts->button[1].y_range_max) / 2;
			}
	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				100);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				100);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
				100);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
				x_position);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
				y_position);
		input_mt_sync(ts->input_dev);
	} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
		input_mt_slot(ts->input_dev, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
			1);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				100);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				100);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
				100);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
				x_position);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
				y_position);
	}
			} else
				input_report_key(ts->input_dev, KEY_HOME, 1);
		} else if (tp_key_index == 0x04) {
			vk_press = 1;
			dev_info(&ts->client->dev, "APP_switch key pressed\n");
		if (ts->pdata->virtual_key) {
			if (ts->button[2].index) {
				x_position = (ts->button[2].x_range_min +
					ts->button[2].x_range_max) / 2;
				y_position = (ts->button[2].y_range_min +
					ts->button[2].y_range_max) / 2;
				}
	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				100);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				100);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
				100);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
				x_position);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
				y_position);
		input_mt_sync(ts->input_dev);
	} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
		input_mt_slot(ts->input_dev, 0);
		input_mt_report_slot_state(ts->input_dev,
			MT_TOOL_FINGER,	1);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				100);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				100);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
				100);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
				x_position);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
				y_position);
	}
}
		}
		input_sync(ts->input_dev);
	} else {
		dev_info(&ts->client->dev, "virtual key released\n");
		vk_press = 0;
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			input_mt_sync(ts->input_dev);
		} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev,
						MT_TOOL_FINGER, 0);
		}
		input_report_key(ts->input_dev, KEY_BACK, 0);
		input_report_key(ts->input_dev, KEY_HOME, 0);
		input_sync(ts->input_dev);
	}
}

inline void himax_ts_work(struct himax_ts_data *ts)
{
	uint8_t buf[128], finger_num, hw_reset_check[2];
	uint16_t finger_pressed;
	uint16_t old_finger = 0;
	uint8_t finger_on = 0;
	int32_t loop_i;
	uint8_t coordInfoSize = ts->coord_data_size +
					ts->area_data_size + 4;
	unsigned char check_sum_cal = 0;
	int RawDataLen = 0;
	int raw_cnt_max;
	int raw_cnt_rmd;
	int hx_touch_info_size;
	int base, x, y, w;

#ifdef HX_TP_PROC_DIAG
	uint8_t *mutual_data;
	uint8_t *self_data;
	uint8_t diag_cmd;
	int i;
	int mul_num;
	int self_num;
	int index = 0;
	int temp1, temp2;
	char coordinate_char[15+(HX_MAX_PT+5)*2*5+2];
	struct timeval t;
	struct tm broken;
#endif
#ifdef HX_CHIP_STATUS_MONITOR
		int j = 0;
#endif

	memset(buf, 0x00, sizeof(buf));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	if (HX_ON_HAND_SHAKING) {
		for (j = 0; j < 100; j++) {
			if (HX_ON_HAND_SHAKING == 0) {
				dev_info(&ts->client->dev,
				"%s:HX_ON_HAND_SHAKING OK check %d times\n",
				__func__, j);
				break;
			}
			usleep_range(500, 1000);
		}
		if (j == 100)
			return;

	}
#endif
	raw_cnt_max = HX_MAX_PT/4;
	raw_cnt_rmd = HX_MAX_PT%4;

	if (raw_cnt_rmd != 0x00) {
		RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+3)*4) - 1;
		hx_touch_info_size = (HX_MAX_PT+raw_cnt_max+2)*4;
	} else {
		RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+2)*4) - 1;
		hx_touch_info_size = (HX_MAX_PT+raw_cnt_max+1)*4;
	}

#ifdef HX_TP_PROC_DIAG
	diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
	if ((diag_cmd) || (ESD_RESET_ACTIVATE) || (HW_RESET_ACTIVATE))
#else
	if ((diag_cmd) || (HW_RESET_ACTIVATE))
#endif
		{
			ret = i2c_himax_read(ts->client, 0x86,
				buf, 128, HIMAX_I2C_RETRY_TIMES);
		} else {
		if (touch_monitor_stop_flag != 0) {
			ret = i2c_himax_read(ts->client, 0x86,
				buf, 128, HIMAX_I2C_RETRY_TIMES);
			touch_monitor_stop_flag--;
		} else
			ret = i2c_himax_read(ts->client, 0x86,
				buf, hx_touch_info_size,
					HIMAX_I2C_RETRY_TIMES);

	}
	if (ret)
#else
	if (i2c_himax_read(ts->client, 0x86, buf,
			hx_touch_info_size, HIMAX_I2C_RETRY_TIMES))
#endif
		{
			pr_err("%s: can't read data\n", __func__);
			goto err_workqueue_out;
		} else {
#ifdef HX_ESD_WORKAROUND
		for (i = 0; i < hx_touch_info_size; i++)	{
			if (buf[i] == 0x00)
				check_sum_cal = 1;
			else if (buf[i] == 0xED) {
				check_sum_cal = 2;
			} else {
				check_sum_cal = 0;
				i = hx_touch_info_size;
				break;
			}
		}

#ifdef HX_TP_PROC_DIAG
	diag_cmd = getDiagCommand();
#ifdef HX_ESD_WORKAROUND
	if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 &&
			HW_RESET_ACTIVATE == 0 && diag_cmd == 0)
#else
	if (check_sum_cal != 0 && diag_cmd == 0)
#endif
#else
#ifdef HX_ESD_WORKAROUND
	if (check_sum_cal != 0 && ESD_RESET_ACTIVATE ==
				0 && HW_RESET_ACTIVATE == 0)
#else
	if (check_sum_cal != 0)
#endif
#endif
		{
		ret = himax_hand_shaking(); /*0:Running, 1:Stop, 2:I2C Fail*/
		if (ret == 2)
			goto err_workqueue_out;

		if ((ret == 1) && (check_sum_cal == 1))	{
			dev_info(&ts->client->dev,
				"[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
			ESD_HW_REST();
		} else if (check_sum_cal == 2) {
			dev_info(&ts->client->dev,
				"[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
			ESD_HW_REST();
		}

		/*himax_int_enable(ts->client->irq,1);*/
		return;
	} else if (ESD_RESET_ACTIVATE) {
		ESD_RESET_ACTIVATE = 0;
		dev_info(&ts->client->dev,
			"[HIMAX TP MSG]:%s: Back from reset\n",
				__func__);
		return;
	} else if (HW_RESET_ACTIVATE)
#else
		if (HW_RESET_ACTIVATE)
#endif
			{
			HW_RESET_ACTIVATE = 0;
			dev_info(&ts->client->dev,
				"[HIMAX TP MSG]:%s HW_RST Back from reset\n",
					__func__);
			return;
			}

		for (loop_i = 0, check_sum_cal = 0;
			loop_i < hx_touch_info_size; loop_i++)
			check_sum_cal += buf[loop_i];

		if ((check_sum_cal != 0x00)) {
			dev_info(&ts->client->dev,
				"[HIMAX TP MSG] checksum fail 0x%02X\n",
					check_sum_cal);
			return;
		}
	}

	if (ts->debug_log_level & BIT(0)) {
		dev_info(&ts->client->dev, "%s: raw data:\n", __func__);
		for (loop_i = 0; loop_i < hx_touch_info_size; loop_i++) {
			dev_info(&ts->client->dev, "0x%2.2X ", buf[loop_i]);
			if (loop_i % 8 == 7)
				dev_info(&ts->client->dev, "\n");
		}
	}

#ifdef HX_TP_PROC_DIAG
	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6) {
		for (i = hx_touch_info_size,
			check_sum_cal = 0; i < 128; i++) {
			check_sum_cal += buf[i];
		}
		if (check_sum_cal % 0x100 != 0)
			goto bypass_checksum_failed_packet;

#ifdef HX_TP_PROC_2T2R
		if (Is_2T2R && diag_cmd == 4) {
			mutual_data = getMutualBuffer_2();
			self_data = getSelfBuffer();
			mul_num = getXChannel_2() * getYChannel_2();

#ifdef HX_EN_SEL_BUTTON
			self_num = getXChannel_2() +
					getYChannel_2() + HX_BT_NUM;
#else
			self_num = getXChannel_2() + getYChannel_2();
#endif
		} else
#endif
		{
			mutual_data = getMutualBuffer();
			self_data = getSelfBuffer();
			mul_num = getXChannel() * getYChannel();

#ifdef HX_EN_SEL_BUTTON
			self_num = getXChannel() + getYChannel() + HX_BT_NUM;
#else
			self_num = getXChannel() + getYChannel();
#endif
		}

	if (buf[hx_touch_info_size] ==
		buf[hx_touch_info_size+1] &&
		buf[hx_touch_info_size+1] ==
		buf[hx_touch_info_size+2]
		&& buf[hx_touch_info_size+2] ==
		buf[hx_touch_info_size+3] &&
		buf[hx_touch_info_size] > 0) {
		index = (buf[hx_touch_info_size] - 1) * RawDataLen;
		for (i = 0; i < RawDataLen; i++) {
			temp1 = index + i;
			if (temp1 < mul_num) {
				mutual_data[index + i] =
					buf[i + hx_touch_info_size+4];
			} else {
				temp1 = i + index;
				temp2 = self_num + mul_num;

				if (temp1 >= temp2)
					break;

				self_data[i+index-mul_num] =
					buf[i + hx_touch_info_size+4];
			}
		}
	} else {
		dev_info(&ts->client->dev,
		"[HIMAX TP MSG]%s: header format is wrong!\n", __func__);
	}
	} else if (diag_cmd == 7) {
		memcpy(&(diag_coor[0]), &buf[0], 128);
	}
	if (coordinate_dump_enable == 1) {
		for (i = 0; i < (15 + (HX_MAX_PT+5)*2*5); i++)
			coordinate_char[i] = 0x20;

		coordinate_char[15 + (HX_MAX_PT+5)*2*5] = 0xD;
		coordinate_char[15 + (HX_MAX_PT+5)*2*5 + 1] = 0xA;
	}
bypass_checksum_failed_packet:
#endif
	EN_NoiseFilter = (buf[HX_TOUCH_INFO_POINT_CNT+2]>>3);
	EN_NoiseFilter = EN_NoiseFilter & 0x01;
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if (ts->pdata->proximity_bytp_enable) {
		if ((buf[HX_TOUCH_INFO_POINT_CNT] & 0x0F)  ==
			0x00 && (buf[HX_TOUCH_INFO_POINT_CNT+2] >> 2 & 0x01)) {

			if (proximity_flag == 0) {
				dev_info(&ts->client->dev,
					" %s near event trigger\n", __func__);
				touch_report_psensor_input_event(0);
				proximity_flag = 1;
			}
			wake_lock_timeout(&ts->ts_wake_lock,
					TS_WAKE_LOCK_TIMEOUT);
			return;
		}
	}
#endif
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
	tpd_key = (buf[HX_TOUCH_INFO_POINT_CNT+2]>>4);
	if (tpd_key == 0x0F) {/*All (VK+AA)leave*/

		tpd_key = 0x00;
	}
#else
		tpd_key = 0x00;
#endif

p_point_num = hx_point_num;

if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
	hx_point_num = 0;
else
	hx_point_num = buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
	if (hx_point_num != 0) {
		if (vk_press == 0x00) {
			old_finger = ts->pre_finger_mask;
			finger_num = buf[coordInfoSize - 4] & 0x0F;
			finger_pressed = buf[coordInfoSize - 2] << 8
					| buf[coordInfoSize - 3];
			finger_on = 1;
			AA_press = 1;
		for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {

			if (((finger_pressed >> loop_i) & 1) == 1) {
				base = loop_i * 4;
				x = buf[base] << 8 | buf[base + 1];
				y = (buf[base + 2] << 8 | buf[base + 3]);
				w = buf[(ts->nFinger_support * 4) + loop_i];
				finger_num--;
				x = ts->pdata->abs_x_max - x;
				y = ts->pdata->abs_y_max - y;
				if (ts->protocol_type == PROTOCOL_TYPE_B) {
					input_mt_slot(ts->input_dev, loop_i);
					input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MAJOR, w);
					input_report_abs(ts->input_dev,
						ABS_MT_WIDTH_MAJOR, w);
					input_report_abs(ts->input_dev,
						ABS_MT_PRESSURE, w);
					input_report_abs(ts->input_dev,
						ABS_MT_POSITION_X, x);
					input_report_abs(ts->input_dev,
						ABS_MT_POSITION_Y, y);
				}
				if (ts->protocol_type == PROTOCOL_TYPE_A) {
					input_report_abs(ts->input_dev,
						ABS_MT_TRACKING_ID, loop_i);
					input_mt_sync(ts->input_dev);
				} else {
					ts->last_slot = loop_i;
					input_mt_report_slot_state(
					ts->input_dev, MT_TOOL_FINGER, 1);
				}
				if (!ts->first_pressed)
					ts->first_pressed = 1;


				ts->pre_finger_data[loop_i][0] = x;
				ts->pre_finger_data[loop_i][1] = y;
				} else {
				if (ts->protocol_type == PROTOCOL_TYPE_B) {
					input_mt_slot(ts->input_dev, loop_i);
					input_mt_report_slot_state(
					ts->input_dev, MT_TOOL_FINGER, 0);
				}

				if (loop_i == 0 && ts->first_pressed == 1)
					ts->first_pressed = 2;

				}
			}
				ts->pre_finger_mask = finger_pressed;
				} else if ((tpd_key_old != 0x00) &&
						(tpd_key == 0x00)) {
					/*temp_x[0] = 0xFFFF;
					temp_y[0] = 0xFFFF;
					temp_x[1] = 0xFFFF;
					temp_y[1] = 0xFFFF;*/
					himax_ts_button_func(tpd_key, ts);
					finger_on = 0;
				}
#ifdef HX_ESD_WORKAROUND
			ESD_COUNTER = 0;
#endif
			input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
			input_sync(ts->input_dev);
		} else if (hx_point_num == 0) {
#if defined(HX_PALM_REPORT)
			loop_i = 0;
			base = loop_i * 4;
			x = buf[base] << 8 | buf[base + 1];
			y = (buf[base + 2] << 8 | buf[base + 3]);
			w = buf[(ts->nFinger_support * 4) + loop_i];

			if ((!atomic_read(&ts->suspend_mode))
				&& (x == 0xFA5A) &&
				(y == 0xFA5A) && (w == 0x00)) {
				dev_info(&ts->client->dev,
					" %s HX_PALM_REPORT KEY power event press\n",
						__func__);
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				msleep(100);
				dev_info(&ts->client->dev,
					" %s HX_PALM_REPORT KEY power event release\n",
					__func__);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				return;
			}
#endif
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
			if ((ts->pdata->proximity_bytp_enable) &&
						(proximity_flag)) {
				dev_info(&ts->client->dev,
					" %s far event trigger\n", __func__);
				touch_report_psensor_input_event(1);
				proximity_flag = 0;
				wake_lock_timeout(&ts->ts_wake_lock,
					TS_WAKE_LOCK_TIMEOUT);
			} else if (AA_press)
#else
	if (AA_press)
#endif
		{
			finger_on = 0;
			AA_press = 0;
			if (ts->protocol_type == PROTOCOL_TYPE_A)
				input_mt_sync(ts->input_dev);

			for (loop_i = 0; loop_i <
					ts->nFinger_support; loop_i++) {
				if (((ts->pre_finger_mask >>
						loop_i) & 1) == 1) {
					if (ts->protocol_type ==
							PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev,
								loop_i);
						input_mt_report_slot_state(
						ts->input_dev,
						MT_TOOL_FINGER, 0);
					}
				}
			}
		if (ts->pre_finger_mask > 0)
			ts->pre_finger_mask = 0;


		if (ts->first_pressed == 1) {
			ts->first_pressed = 2;
			dev_info(&ts->client->dev,
				"E1@%d, %d\n", ts->pre_finger_data[0][0],
				ts->pre_finger_data[0][1]);
		}


#ifdef HX_TP_PROC_DIAG
			if (coordinate_dump_enable == 1) {
				do_gettimeofday(&t);
				time_to_tm(t.tv_sec, 0, &broken);
			}
#endif
			} else if (tpd_key != 0x00) {
				/*report key
				temp_x[0] = 0xFFFF;
				temp_y[0] = 0xFFFF;
				temp_x[1] = 0xFFFF;
				temp_y[1] = 0xFFFF;*/
				himax_ts_button_func(tpd_key, ts);
				finger_on = 1;
			} else if ((tpd_key_old != 0x00) && (tpd_key == 0x00)) {
				/*temp_x[0] = 0xFFFF;
				temp_y[0] = 0xFFFF;
				temp_x[1] = 0xFFFF;
				temp_y[1] = 0xFFFF;*/
				himax_ts_button_func(tpd_key, ts);
				finger_on = 0;
			}
#ifdef HX_ESD_WORKAROUND
				ESD_COUNTER = 0;
#endif
			input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
			input_sync(ts->input_dev);
		}
		tpd_key_old = tpd_key;
		Last_EN_NoiseFilter = EN_NoiseFilter;

workqueue_out:
	return;

err_workqueue_out:
	dev_info(&ts->client->dev,
		"%s: Now reset the Touch chip.\n", __func__);

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset(true, false);
#endif

	goto workqueue_out;
}

#ifdef QCT
static irqreturn_t himax_ts_thread(int irq, void *ptr)
{
	struct himax_ts_data *ts = ptr;
	struct timespec timeStart, timeEnd, timeDelta;

	if (ts->debug_log_level & BIT(2))
		getnstimeofday(&timeStart);

#ifdef HX_SMART_WAKEUP
	if (atomic_read(&ts->suspend_mode) &&
		(!FAKE_POWER_KEY_SEND) && (ts->SMWP_enable)) {
		wake_lock_timeout(&ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
		if (himax_parse_wake_event((struct himax_ts_data *)ptr)) {
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);
			msleep(100);
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
			FAKE_POWER_KEY_SEND = true;
			return IRQ_HANDLED;
		}
	}
#endif
	himax_ts_work((struct himax_ts_data *)ptr);
	if (ts->debug_log_level & BIT(2)) {
		getnstimeofday(&timeEnd);
		timeDelta.tv_nsec =
			(timeEnd.tv_sec*1000000000+timeEnd.tv_nsec)
			-(timeStart.tv_sec*1000000000+timeStart.tv_nsec);
	}
	return IRQ_HANDLED;
}

static void himax_ts_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work,
				struct himax_ts_data, work);
	himax_ts_work(ts);
}

static enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer,
		ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

int himax_ts_register_interrupt(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;

	ts->irq_enabled = 0;

	if (client->irq) {/*INT mode*/
		ts->use_irq = 1;
		if (HX_INT_IS_EDGE) {
			dev_info(&client->dev,
				"%s edge triiger falling\n ", __func__);
			ret = request_threaded_irq(client->irq,
				NULL, himax_ts_thread,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						client->name, ts);
			} else {
				dev_info(&client->dev,
					"%s level trigger low\n ", __func__);
				ret = request_threaded_irq(client->irq,
						 NULL, himax_ts_thread,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						client->name, ts);
			}
		if (ret == 0) {
			ts->irq_enabled = 1;
			irq_enable_count = 1;
			dev_info(&client->dev,
				"%s: irq enabled at qpio: %d\n",
					__func__, client->irq);
#ifdef HX_SMART_WAKEUP
			irq_set_irq_wake(client->irq, 1);
#endif
		} else {
			ts->use_irq = 0;
			pr_err("%s: request_irq failed\n", __func__);
		}
	} else
		dev_info(&client->dev,
			"%s: client->irq is empty, use polling mode.\n",
						__func__);


	if (!ts->use_irq) {
		ts->himax_wq = create_singlethread_workqueue("himax_touch");

		INIT_WORK(&ts->work, himax_ts_work_func);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		dev_info(&client->dev,
			"%s: polling mode enabled\n", __func__);
	}
	return ret;
}
#endif

#if defined(HX_USB_DETECT)
static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;

	ts = private_ts;
	if (ts->cable_config) {
		if (!atomic_read(&ts->suspend_mode)) {
			if ((!!connect_status) != ts->usb_connected) {
				if (!!connect_status) {
					ts->cable_config[1] = 0x01;
					ts->usb_connected = 0x01;
				} else {
					ts->cable_config[1] = 0x00;
					ts->usb_connected = 0x00;
				}

			i2c_himax_master_write(ts->client, ts->cable_config,
			sizeof(ts->cable_config), HIMAX_I2C_RETRY_TIMES);

			}
		} else {
			if (connect_status)
				ts->usb_connected = 0x01;
			else
				ts->usb_connected = 0x00;
		}
	}
}

static struct t_cable_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};

#endif

#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;

	struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
							work_att.work);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		pr_err(" Unable to register fb_notifier: %d\n", ret);
}
#endif

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
int proximity_enable_from_ps(int on)
{
	char buf_tmp[5];

	if (on)	{
		touch_report_psensor_input_event(1);
		buf_tmp[0] = 0x92;
		buf_tmp[1] = 0x01;
		g_proximity_en = 1;
		enable_irq_wake(private_ts->client->irq);
	} else {
		buf_tmp[0] = 0x92;
		buf_tmp[1] = 0x00;
		g_proximity_en = 0;
		disable_irq_wake(private_ts->client->irq);
	}

	i2c_himax_master_write(private_ts->client,
			buf_tmp, 2, HIMAX_I2C_RETRY_TIMES);

	return 0;
}
EXPORT_SYMBOL_GPL(proximity_enable_from_ps);
#endif

/*=========================================
*
*	Segment : Himax SYS Debug Function
*
*==========================================*/

#ifdef CONFIG_OF
#if defined(HX_LOADIN_CONFIG) || defined(HX_AUTO_UPDATE_CONFIG)
static int himax_parse_config(struct himax_ts_data *ts,
			struct himax_config *pdata)
{
	struct himax_config *cfg_table;
	struct device_node *node, *pp = NULL;
	struct property *prop;
	uint8_t cnt = 0, i = 0;
	u32 data = 0;
	uint32_t coords[4] = {0};
	int len = 0;
	char str[6] = {0};

	node = ts->client->dev.of_node;
	if (node == NULL) {
		pr_err(" %s, can't find device_node", __func__);
		return -ENODEV;
	}

	while ((pp = of_get_next_child(node, pp)))
		cnt++;

	if (!cnt)
		return -ENODEV;

	cfg_table = kzalloc(cnt * (sizeof(*cfg_table)), GFP_KERNEL);
	if (!cfg_table)
		return -ENOMEM;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (of_property_read_u32(pp, "default_cfg", &data) == 0)
			cfg_table[i].default_cfg = data;

		if (of_property_read_u32(pp, "sensor_id", &data) == 0)
			cfg_table[i].sensor_id = (data);

		if (of_property_read_u32(pp, "fw_ver_main", &data) == 0)
			cfg_table[i].fw_ver_main = data;

		if (of_property_read_u32(pp, "fw_ver_minor", &data) == 0)
			cfg_table[i].fw_ver_minor = data;

		if (of_property_read_u32_array(pp,
			"himax,tw-coords", coords, 4) == 0) {
			cfg_table[i].tw_x_min =
				coords[0], cfg_table[i].tw_x_max = coords[1];
			cfg_table[i].tw_y_min =
				coords[2], cfg_table[i].tw_y_max = coords[3];
		}

		if (of_property_read_u32_array(pp,
			"himax,pl-coords", coords, 4) == 0) {
			cfg_table[i].pl_x_min =
				coords[0], cfg_table[i].pl_x_max = coords[1];
			cfg_table[i].pl_y_min =
				coords[2], cfg_table[i].pl_y_max = coords[3];
		}

		prop = of_find_property(pp, "c1", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c1");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c1, prop->value, len);
		prop = of_find_property(pp, "c2", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c2");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c2, prop->value, len);
		prop = of_find_property(pp, "c3", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c3");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c3, prop->value, len);
		prop = of_find_property(pp, "c4", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c4");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c4, prop->value, len);
		prop = of_find_property(pp, "c5", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c5");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c5, prop->value, len);
		prop = of_find_property(pp, "c6", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c6");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c6, prop->value, len);
		prop = of_find_property(pp, "c7", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c7");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c7, prop->value, len);
		prop = of_find_property(pp, "c8", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c8");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c8, prop->value, len);
		prop = of_find_property(pp, "c9", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c9");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c9, prop->value, len);
		prop = of_find_property(pp, "c10", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c10");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c10, prop->value, len);
		prop = of_find_property(pp, "c11", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c11");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c11, prop->value, len);
		prop = of_find_property(pp, "c12", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c12");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c12, prop->value, len);
		prop = of_find_property(pp, "c13", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c13");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c13, prop->value, len);
		prop = of_find_property(pp, "c14", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c14");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c14, prop->value, len);
		prop = of_find_property(pp, "c15", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c15");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c15, prop->value, len);
		prop = of_find_property(pp, "c16", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c16");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c16, prop->value, len);
		prop = of_find_property(pp, "c17", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c17");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c17, prop->value, len);
		prop = of_find_property(pp, "c18", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c18");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c18, prop->value, len);
		prop = of_find_property(pp, "c19", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c19");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c19, prop->value, len);
		prop = of_find_property(pp, "c20", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c20");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c20, prop->value, len);
		prop = of_find_property(pp, "c21", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c21");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c21, prop->value, len);
		prop = of_find_property(pp, "c22", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c22");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c22, prop->value, len);
		prop = of_find_property(pp, "c23", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c23");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c23, prop->value, len);
		prop = of_find_property(pp, "c24", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c24");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c24, prop->value, len);
		prop = of_find_property(pp, "c25", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c25");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c25, prop->value, len);
		prop = of_find_property(pp, "c26", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c26");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c26, prop->value, len);
		prop = of_find_property(pp, "c27", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c27");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c27, prop->value, len);
		prop = of_find_property(pp, "c28", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c28");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c28, prop->value, len);
		prop = of_find_property(pp, "c29", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c29");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c29, prop->value, len);
		prop = of_find_property(pp, "c30", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c30");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c30, prop->value, len);
		prop = of_find_property(pp, "c31", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c31");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c31, prop->value, len);
		prop = of_find_property(pp, "c32", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c32");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c32, prop->value, len);
		prop = of_find_property(pp, "c33", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c33");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c33, prop->value, len);
		prop = of_find_property(pp, "c34", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c34");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c34, prop->value, len);
		prop = of_find_property(pp, "c35", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c35");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c35, prop->value, len);
		prop = of_find_property(pp, "c36", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c36");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c36, prop->value, len);
		prop = of_find_property(pp, "c37", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c37");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c37, prop->value, len);
		prop = of_find_property(pp, "c38", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c38");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c38, prop->value, len);
		prop = of_find_property(pp, "c39", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c39");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c39, prop->value, len);
		prop = of_find_property(pp, "c40", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c40");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c40, prop->value, len);
		prop = of_find_property(pp, "c41", &len);
		if ((!prop) || (!len)) {
			strlcpy(str, "c41");
			goto of_find_property_error;
		}
		memcpy(cfg_table[i].c41, prop->value, len);
		i++;

of_find_property_error:
	if (!prop) {
		pr_err(" %s:Looking up %s property in node %s failed",
			__func__, str, pp->full_name);
		return -ENODEV;
	} else if (!len) {
		pr_err(" %s:Invalid length of configuration data in %s\n",
			__func__, str);
		return -EINVAL;
		}
	}

	i = 0;	/*confirm which config we should load*/
	while ((ts->vendor_fw_ver_H << 8 | ts->vendor_fw_ver_L) <
		(cfg_table[i].fw_ver_main << 8 | cfg_table[i].fw_ver_minor)) {
		i++;
	}
	if (cfg_table[i].default_cfg != 0)
		goto startloadconf;
	while (cfg_table[i].sensor_id > 0 &&
		(cfg_table[i].sensor_id !=  ts->vendor_sensor_id)) {
		i++;
	}

startloadconf:
	if (i <= cnt) {
		pdata->fw_ver_main  = cfg_table[i].fw_ver_main;
		pdata->fw_ver_minor = cfg_table[i].fw_ver_minor;
		pdata->sensor_id = cfg_table[i].sensor_id;

		memcpy(pdata->c1, cfg_table[i].c1, sizeof(pdata->c1));
		memcpy(pdata->c2, cfg_table[i].c2, sizeof(pdata->c2));
		memcpy(pdata->c3, cfg_table[i].c3, sizeof(pdata->c3));
		memcpy(pdata->c4, cfg_table[i].c4, sizeof(pdata->c4));
		memcpy(pdata->c5, cfg_table[i].c5, sizeof(pdata->c5));
		memcpy(pdata->c6, cfg_table[i].c6, sizeof(pdata->c6));
		memcpy(pdata->c7, cfg_table[i].c7, sizeof(pdata->c7));
		memcpy(pdata->c8, cfg_table[i].c8, sizeof(pdata->c8));
		memcpy(pdata->c9, cfg_table[i].c9, sizeof(pdata->c9));
		memcpy(pdata->c10, cfg_table[i].c10, sizeof(pdata->c10));
		memcpy(pdata->c11, cfg_table[i].c11, sizeof(pdata->c11));
		memcpy(pdata->c12, cfg_table[i].c12, sizeof(pdata->c12));
		memcpy(pdata->c13, cfg_table[i].c13, sizeof(pdata->c13));
		memcpy(pdata->c14, cfg_table[i].c14, sizeof(pdata->c14));
		memcpy(pdata->c15, cfg_table[i].c15, sizeof(pdata->c15));
		memcpy(pdata->c16, cfg_table[i].c16, sizeof(pdata->c16));
		memcpy(pdata->c17, cfg_table[i].c17, sizeof(pdata->c17));
		memcpy(pdata->c18, cfg_table[i].c18, sizeof(pdata->c18));
		memcpy(pdata->c19, cfg_table[i].c19, sizeof(pdata->c19));
		memcpy(pdata->c20, cfg_table[i].c20, sizeof(pdata->c20));
		memcpy(pdata->c21, cfg_table[i].c21, sizeof(pdata->c21));
		memcpy(pdata->c22, cfg_table[i].c22, sizeof(pdata->c22));
		memcpy(pdata->c23, cfg_table[i].c23, sizeof(pdata->c23));
		memcpy(pdata->c24, cfg_table[i].c24, sizeof(pdata->c24));
		memcpy(pdata->c25, cfg_table[i].c25, sizeof(pdata->c25));
		memcpy(pdata->c26, cfg_table[i].c26, sizeof(pdata->c26));
		memcpy(pdata->c27, cfg_table[i].c27, sizeof(pdata->c27));
		memcpy(pdata->c28, cfg_table[i].c28, sizeof(pdata->c28));
		memcpy(pdata->c29, cfg_table[i].c29, sizeof(pdata->c29));
		memcpy(pdata->c30, cfg_table[i].c30, sizeof(pdata->c30));
		memcpy(pdata->c31, cfg_table[i].c31, sizeof(pdata->c31));
		memcpy(pdata->c32, cfg_table[i].c32, sizeof(pdata->c32));
		memcpy(pdata->c33, cfg_table[i].c33, sizeof(pdata->c33));
		memcpy(pdata->c34, cfg_table[i].c34, sizeof(pdata->c34));
		memcpy(pdata->c35, cfg_table[i].c35, sizeof(pdata->c35));
		memcpy(pdata->c36, cfg_table[i].c36, sizeof(pdata->c36));
		memcpy(pdata->c37, cfg_table[i].c37, sizeof(pdata->c37));
		memcpy(pdata->c38, cfg_table[i].c38, sizeof(pdata->c38));
		memcpy(pdata->c39, cfg_table[i].c39, sizeof(pdata->c39));
		memcpy(pdata->c40, cfg_table[i].c40, sizeof(pdata->c40));
		memcpy(pdata->c41, cfg_table[i].c41, sizeof(pdata->c41));

		ts->tw_x_min = cfg_table[i].tw_x_min,
			ts->tw_x_max = cfg_table[i].tw_x_max;
		ts->tw_y_min = cfg_table[i].tw_y_min,
			ts->tw_y_max = cfg_table[i].tw_y_max;

		ts->pl_x_min = cfg_table[i].pl_x_min,
			ts->pl_x_max = cfg_table[i].pl_x_max;
		ts->pl_y_min = cfg_table[i].pl_y_min
			ts->pl_y_max = cfg_table[i].pl_y_max;

	} else {
		pr_err(" DT-%s cfg idx(%d) > cnt(%d)", __func__, i, cnt);
		return -EINVAL;
	}
	return 0;
}
#endif
/**********************power*******************************/
static int himax_power_init(struct i2c_client *client,
			struct himax_i2c_platform_data *pdata)
{
	int rc;

	pdata->vcc_ana = regulator_get(&client->dev, "avdd");
	if (IS_ERR(pdata->vcc_ana)) {
		rc = PTR_ERR(pdata->vcc_ana);
		pr_err("Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->vcc_ana) > 0) {
		rc = regulator_set_voltage(pdata->vcc_ana, HX_VTG_MIN_UV,
							HX_VTG_MAX_UV);
		if (rc) {
			pr_err("regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}


		pdata->vcc_i2c = regulator_get(&client->dev, "vdd");
		if (IS_ERR(pdata->vcc_i2c)) {
			rc = PTR_ERR(pdata->vcc_i2c);
			pr_err("Regulator get failed rc=%d\n", rc);
			goto  error_set_vtg_i2c;
		}
		if (regulator_count_voltages(pdata->vcc_i2c) > 0) {
			rc = regulator_set_voltage(pdata->vcc_i2c,
				HX_I2C_VTG_MIN_UV, HX_I2C_VTG_MAX_UV);
			if (rc) {
				pr_err("regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_i2c;
			}
		}


	return 0;

error_set_vtg_i2c:
	regulator_put(pdata->vcc_i2c);

error_set_vtg_vcc_ana:
	regulator_put(pdata->vcc_ana);
	return rc;

}

static int himax_power_on(struct himax_i2c_platform_data *pdata, bool on)
{
	int rc;

	if (on == false)
		goto power_off;

	/*
	rc = reg_set_optimum_mode_check(pdata->vcc_ana, HX_ACTIVE_LOAD_UA);
	if (rc < 0)
		return rc;
	*/

	rc = regulator_enable(pdata->vcc_ana);
	if (rc)
		goto error_reg_en_vcc_ana;


	/*if (pdata->i2c_pull_up) {
		rc = reg_set_optimum_mode_check(pdata->vcc_i2c, HX_I2C_LOAD_UA);
		if (rc < 0) {
			pr_err("Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto error_reg_opt_i2c;
		}*/

		rc = regulator_enable(pdata->vcc_i2c);
		if (rc) {
			pr_err("Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}

	msleep(130);

	return 0;

error_reg_en_vcc_i2c:
	regulator_disable(pdata->vcc_i2c);
error_reg_en_vcc_ana:
	regulator_disable(pdata->vcc_ana);
	return rc;

power_off:
	regulator_set_voltage(pdata->vcc_ana, 0, HX_VTG_MAX_UV);
	regulator_disable(pdata->vcc_ana);
	regulator_set_voltage(pdata->vcc_i2c, 0, HX_I2C_VTG_MAX_UV);
	regulator_disable(pdata->vcc_i2c);
	msleep(50);
	return 0;
}

static void himax_vk_parser(struct device_node *dt,
				struct himax_i2c_platform_data *pdata)
{
	u32 data = 0;
	uint8_t cnt = 0, i = 0;
	uint32_t coords[4] = {0};
	struct device_node *node, *pp = NULL;
	struct himax_virtual_key *vk;

	node = of_parse_phandle(dt, "virtualkey", 0);
	if (node == NULL)
		return;

	while ((pp = of_get_next_child(node, pp)))
		cnt++;
	if (!cnt)
		return;

	vk = kzalloc(cnt * (sizeof(*vk)), GFP_KERNEL);
	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (of_property_read_u32(pp, "idx", &data) == 0)
			vk[i].index = data;
		if (of_property_read_u32_array(pp,
			"range", coords, 4) == 0) {
			vk[i].x_range_min = coords[0];
			vk[i].x_range_max = coords[1];
			vk[i].y_range_min = coords[2],
			vk[i].y_range_max = coords[3];
		}
		i++;
	}
	pdata->virtual_key = vk;

}

static int himax_parse_dt(struct himax_ts_data *ts,
				struct himax_i2c_platform_data *pdata)
{
	int rc, coords_size = 0;
	u32 temp_val;
	uint32_t coords[4] = {0};
	struct property *prop;
	struct device_node *dt = ts->client->dev.of_node;
	u32 data = 0;

	ts->name = "himax";
	rc = of_property_read_string(dt, "himax,name", &ts->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(&ts->client->dev, "Unable to read name\n");
		return rc;
	}

	prop = of_find_property(dt, "himax,panel-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			pr_err(" %s:Invalid panel coords size %d",
						__func__, coords_size);
	}

	if (of_property_read_u32_array(dt, "himax,panel-coords",
					coords, coords_size) == 0) {
		pdata->abs_x_min = coords[0], pdata->abs_x_max = coords[1];
		pdata->abs_y_min = coords[2], pdata->abs_y_max = coords[3];

	}

	prop = of_find_property(dt, "himax,display-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			pr_err(" %s:Invalid display coords size %d",
						__func__, coords_size);
	}
	rc = of_property_read_u32_array(dt,
		"himax,display-coords", coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		pr_err(" %s:Fail to read display-coords %d\n", __func__, rc);
		return rc;
	}
	pdata->screenWidth  = coords[1];
	pdata->screenHeight = coords[3];

	pdata->gpio_irq = of_get_named_gpio(dt, "himax,irq-gpio", 0);
	if (pdata->gpio_irq < 0)
		return pdata->gpio_irq;

	pdata->gpio_reset = of_get_named_gpio(dt, "himax,reset-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_reset))
		dev_info(&ts->client->dev,
			" DT:gpio_rst value is not valid\n");


	rc = of_property_read_u32(dt, "himax,num-max-touches", &temp_val);
	if (!rc)
		ts->num_max_touches = temp_val;
	else
		return rc;

	if (of_property_read_u32(dt, "report_type", &data) == 0) {
		pdata->protocol_type = data;
		dev_info(&ts->client->dev,
			" DT:protocol_type=%d", pdata->protocol_type);
	}

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if (of_property_read_u32(dt, "proximity_bytp_enable", &data) == 0) {
		pdata->proximity_bytp_enable = data;
		dev_info(&ts->client->dev,
		" DT:proximity_bytp_enable=%d", pdata->proximity_bytp_enable);
	}
#endif
	himax_vk_parser(dt, pdata);

	return 0;
}
#endif



static int himax852xes_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0, err = 0;
	struct dentry *temp;
	struct himax_ts_data *ts;
	struct himax_i2c_platform_data *pdata;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}
	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->dev = &client->dev;


#ifdef CONFIG_OF
	if (client->dev.of_node) { /*DeviceTree Init Platform_data*/
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			err = -ENOMEM;
			goto err_dt_platform_data_fail;
		}
		ret = himax_parse_dt(ts, pdata);
		if (ret < 0)
			goto err_alloc_dt_pdata_failed;
	} else
#endif
	{
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			pr_err(" pdata is NULL(dev.platform_data)\n");
			goto err_get_platform_data_fail;
		}
	}
#ifdef HX_RST_PIN_FUNC
	ts->rst_gpio = pdata->gpio_reset;
#endif

/*himax_gpio_power_config(ts->client, pdata);*/
	ret = himax_power_init(client, pdata);
#ifndef CONFIG_OF
	if (pdata->power) {
		ret = pdata->power(1);
		if (ret < 0) {
			pr_err("%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}
#endif
	private_ts = ts;
	ret = himax_power_on(pdata, true);
	/*Get Himax IC Type / FW information / Calculate the point number*/
	if (himax_ic_package_check(ts) == false) {
		pr_err("Himax chip doesn NOT EXIST");
		goto err_ic_package_failed;
	}

	if (pdata->virtual_key)
		ts->button = pdata->virtual_key;
#ifdef HX_TP_PROC_FLASH_DUMP
		ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
		if (!ts->flash_wq) {
			pr_err("%s: create flash workqueue failed\n", __func__);
			err = -ENOMEM;
			goto err_create_wq_failed;
		}
		INIT_WORK(&ts->flash_work, himax_ts_flash_work_func);
		setSysOperation(0);
		setFlashBuffer();
#endif
	himax_read_TP_info(client);
#ifdef HX_AUTO_UPDATE_FW
	if (i_update_FW() == false)
		dev_info(&client->dev, "NOT Have new FW=NOT UPDATE=\n");
	else
		dev_info(&client->dev, "Have new FW=UPDATE=\n");
#endif

	/*Himax Power On and Load Config*/
	if (himax_loadSensorConfig(client, pdata) < 0)
		goto err_detect_failed;


	calculate_point_number();
#ifdef HX_TP_PROC_DIAG
	setXChannel(HX_RX_NUM); /* X channel*/
	setYChannel(HX_TX_NUM); /* Y channel*/

	setMutualBuffer();
	if (getMutualBuffer() == NULL) {
		pr_err("%s: mutual buffer allocate fail failed\n", __func__);
		return -ENOMEM;
	}
#ifdef HX_TP_PROC_2T2R
	if (Is_2T2R) {
		setXChannel_2(HX_RX_NUM_2); /* X channel*/
		setYChannel_2(HX_TX_NUM_2); /* Y channel*/

		setMutualBuffer_2();

		if (getMutualBuffer_2() == NULL) {
			pr_err("%s: buffer 2 allocate failed\n", __func__);
			return -ENOMEM;

		}
	}
#endif
#endif
#ifdef CONFIG_OF
	ts->power = pdata->power;
#endif
	ts->pdata = pdata;

	ts->x_channel = HX_RX_NUM;
	ts->y_channel = HX_TX_NUM;
	ts->nFinger_support = HX_MAX_PT;
	/*calculate the i2c data size*/
	calcDataSize(ts->nFinger_support);
#ifdef CONFIG_OF
	ts->pdata->abs_pressure_min        = 0;
	ts->pdata->abs_pressure_max        = 200;
	ts->pdata->abs_width_min           = 0;
	ts->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0x90;
	pdata->cable_config[1]             = 0x00;
#endif
	ts->suspended                      = false;
#if defined(HX_USB_DETECT)
	ts->usb_connected = 0x00;
	ts->cable_config = pdata->cable_config;
#endif
	ts->protocol_type = pdata->protocol_type;
	dev_info(&client->dev, "%s: himax Use Protocol Type %c\n", __func__,
	ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

	ret = himax_input_register(ts);
	if (ret) {
		pr_err("%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	/*himax_read_FW_ver(true);*/
#ifdef CONFIG_FB
	ts->himax_att_wq =
		create_singlethread_workqueue("HMX_ATT_request");
	if (!ts->himax_att_wq) {
		pr_err(" allocate syn_att_wq failed\n");
		err = -ENOMEM;
		goto err_get_intr_bit_failed;
	}
	INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
	queue_delayed_work(ts->himax_att_wq,
		&ts->work_att, msecs_to_jiffies(15000));

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ts->early_suspend.suspend = himax_ts_early_suspend;
	ts->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef HX_CHIP_STATUS_MONITOR
	ts->himax_chip_monitor_wq =
		create_singlethread_workqueue("himax_chip_monitor_wq");
	if (!ts->himax_chip_monitor_wq)	{
		pr_err(" %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_DELAYED_WORK(&ts->himax_chip_monitor,
			himax_chip_monitor_function);
	queue_delayed_work(ts->himax_chip_monitor_wq,
		&ts->himax_chip_monitor, HX_POLLING_TIMER*HZ);
#endif

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if (pdata->proximity_bytp_enable)
		wake_lock_init(&ts->ts_wake_lock,
			 WAKE_LOCK_SUSPEND, HIMAX852xes_NAME);
#endif
#ifdef HX_SMART_WAKEUP
	ts->SMWP_enable = 0;
	wake_lock_init(&ts->ts_SMWP_wake_lock,
		 WAKE_LOCK_SUSPEND, HIMAX852xes_NAME);
#endif


/***********************sysfile************************************/

err = device_create_file(&client->dev, &dev_attr_ts_info);
if (err) {
	dev_err(&client->dev, "sys file creation failed\n");
	goto err_input_register_device_failed;
}

err = device_create_file(&client->dev, &dev_attr_mt_protocol_type);
if (err) {
	dev_err(&client->dev, "sys file creation failed\n");
	goto free_ts_info;
}

err = device_create_file(&client->dev, &dev_attr_enable);
if (err) {
	dev_err(&client->dev, "sys file creation failed\n");
	goto free_type;
}

err = device_create_file(&client->dev, &dev_attr_fw_name);
if (err) {
	dev_err(&client->dev, "sys file creation failed\n");
	goto free_enable;
}

err = device_create_file(&client->dev, &dev_attr_update_fw);
if (err) {
	dev_err(&client->dev, "sys file creation failed\n");
	goto free_fw_name_sys;
}

err = device_create_file(&client->dev, &dev_attr_force_update_fw);
if (err) {
	dev_err(&client->dev, "sys file creation failed\n");
	goto free_update_fw_sys;
}

ts->ts_info =
	devm_kzalloc(&client->dev, HIMAX_INFO_MAX_LEN, GFP_KERNEL);
if (!ts->ts_info) {
	dev_err(&client->dev, "Not enough memory\n");
	goto free_force_update_fw_sys;
}
ts->enable = true;

STORE_TS_INFO(ts->ts_info, ts->name,
		ts->num_max_touches,
		ts->virtual_key ? "yes" : "no");

ts->dir = debugfs_create_dir(DEBUG_DIR_NAME, NULL);
	if (ts->dir == NULL || IS_ERR(ts->dir)) {
		err = PTR_ERR(ts->dir);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, ts->dir,
				ts, &debug_addr_fops);

	if (temp == NULL || IS_ERR(temp)) {
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp =
		debugfs_create_file("data", S_IRUSR | S_IWUSR, ts->dir,
				ts, &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, ts->dir,
			ts, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp =
		debugfs_create_file("dump_info", S_IRUSR | S_IWUSR,
				ts->dir,	ts, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}


#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 0;
#endif
HW_RESET_ACTIVATE = 0;

#if defined(HX_USB_DETECT)
	if (ts->cable_config)
		cable_detect_register_notifier(&himax_cable_status_handler);
#endif
#ifdef HX_DOT_VIEW
	register_notifier_by_hallsensor(&hallsensor_status_handler);
#endif

	err = himax_ts_register_interrupt(ts->client);
	if (err)
		goto err_register_interrupt_failed;

return 0;

free_debug_dir:
	debugfs_remove_recursive(ts->dir);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
free_enable:
	device_remove_file(&client->dev, &dev_attr_enable);
free_type:
	device_remove_file(&client->dev, &dev_attr_mt_protocol_type);
free_ts_info:
	device_remove_file(&client->dev, &dev_attr_ts_info);


err_register_interrupt_failed:
err_get_intr_bit_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if (pdata->proximity_bytp_enable)
		wake_lock_destroy(&ts->ts_wake_lock);
#endif
#ifdef HX_SMART_WAKEUP
	wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
err_detect_failed:
#ifdef HX_TP_PROC_FLASH_DUMP
err_create_wq_failed:
#endif
err_ic_package_failed:
#ifndef CONFIG_OF
err_power_failed:
#else
err_dt_platform_data_fail:
#endif
	kfree(pdata);

err_get_platform_data_fail:
#ifdef CONFIG_OF
err_alloc_dt_pdata_failed:
#endif
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	return err;

}

static int himax852xes_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);


#ifdef CONFIG_FB
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

	if (!ts->use_irq)
		hrtimer_cancel(&ts->timer);

	destroy_workqueue(ts->himax_wq);
#ifdef HX_CHIP_STATUS_MONITOR
	destroy_workqueue(ts->himax_chip_monitor_wq);
#endif
	if (ts->protocol_type == PROTOCOL_TYPE_B)
		input_mt_destroy_slots(ts->input_dev);

	input_unregister_device(ts->input_dev);
#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if (ts->pdata->proximity_bytp_enable)
		wake_lock_destroy(&ts->ts_wake_lock);
#endif
#ifdef HX_SMART_WAKEUP
		wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
	kfree(ts);

	return 0;

}

static int himax852xes_suspend(struct device *dev)
{
	int ret;
	uint8_t buf[2] = {0};
#ifdef HX_CHIP_STATUS_MONITOR
	int t = 0;
#endif
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	if (ts->suspended) {
		dev_info(dev, "%s: Already suspended. Skipped.\n", __func__);
		return 0;
	}
	ts->suspended = true;
	pr_err("%s: enter\n", __func__);

#if defined(CONFIG_TOUCHSCREEN_PROXIMITY)
	if (ts->pdata->proximity_bytp_enable) {
			dev_info(dev,
				"[Proximity],Proximity en=%d\r\n",
						g_proximity_en);
			if (g_proximity_en) {
				dev_info(dev,
				"[Proximity],Proximity %s\r\n",
					 proximity_flag ? "NEAR":"FAR");
			}
			if ((g_proximity_en) && (proximity_flag)) {
				dev_info(dev,
				"Proximity on,and Near won't enter deep sleep now.\n");
				atomic_set(&ts->suspend_mode, 1);
				ts->first_pressed = 0;
				ts->pre_finger_mask = 0;
				return 0;
			}
		}
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
	if (getFlashDumpGoing()) {
		dev_info(dev,
		"[himax] %s: Flash dump is going, reject suspend\n",
				__func__);
		return 0;
	}
#endif
#ifdef HX_TP_PROC_HITOUCH
	if (hitouch_is_connect) {
		dev_info(dev,
			"[himax] %s: connect, reject suspend\n",
				__func__);
	return 0;
	}
#endif

#ifdef HX_CHIP_STATUS_MONITOR
if (HX_ON_HAND_SHAKING) {
	for (t = 0; t < 100; t++) {
			if (HX_ON_HAND_SHAKING == 0) {
				dev_info(dev,
					"%s:HX_ON_HAND_SHAKING OK check %d times\n",
						 __func__, t);
			break;
			} else
				usleep_range(500, 1000);
		}
	if (t == 100)
		return 0;

}
#endif

#ifdef HX_SMART_WAKEUP
	if (ts->SMWP_enable) {
		atomic_set(&ts->suspend_mode, 1);
		ts->pre_finger_mask = 0;
		FAKE_POWER_KEY_SEND = false;
		buf[0] = 0x8F;
		buf[1] = 0x20;
		ret = i2c_himax_master_write(ts->client,
				buf, 2, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0)
			pr_err("[himax] %s: I2C access failed addr = 0x%x\n",
				__func__, ts->client->addr);

		dev_info(dev,
			"[himax] %s: SMART_WAKEUP enable\n", __func__);
		return 0;
	}
#endif

	himax_int_enable(ts->client->irq, 0);

#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
#endif

	buf[0] = HX_CMD_TSSOFF;
	ret = i2c_himax_master_write(ts->client,
			buf, 1, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0)
		pr_err("[himax] %s: I2C access failed addr = 0x%x\n",
					 __func__, ts->client->addr);

	msleep(140);

	buf[0] = HX_CMD_TSSLPIN;
	ret = i2c_himax_master_write(ts->client, buf, 1, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0)
		pr_err("[himax] %s: I2C access failed addr = 0x%x\n",
				 __func__, ts->client->addr);


	if (!ts->use_irq) {
		ret = cancel_work_sync(&ts->work);
		if (ret)
			himax_int_enable(ts->client->irq, 1);
	}

	/*ts->first_pressed = 0;*/
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;

	if (ts->pdata->power)
		ts->pdata->power(0);

	/*himax_power_on(ts->pdata, false);*/
	return 0;
}

static int himax852xes_resume(struct device *dev)
{
	int ret;
#ifdef HX_SMART_WAKEUP
	int ret;
	uint8_t buf[2] = {0};
#endif
#ifdef HX_CHIP_STATUS_MONITOR
	int t = 0;
#endif
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	dev_info(dev, "%s: enter\n", __func__);
	himax_power_on(ts->pdata, true);
#ifdef HX_CHIP_STATUS_MONITOR
	if (HX_ON_HAND_SHAKING) {
		for (t = 0; t < 100; t++) {

			if (HX_ON_HAND_SHAKING == 0) {
				dev_info(dev, "%s:HX_ON_HAND_SHAKING OK check %d times\n",
							__func__, t);
						break;
					}
				else
					usleep_range(500, 1000);
			}
		if (t == 100)
			return 0;

	}
#endif
#ifdef HX_SMART_WAKEUP
	if (ts->SMWP_enable) {

		i2c_himax_write_command(ts->client,
			0x82, HIMAX_I2C_RETRY_TIMES);
		msleep(40);

		i2c_himax_write_command(ts->client,
				0x80, HIMAX_I2C_RETRY_TIMES);
		buf[0] = 0x8F;
		buf[1] = 0x00;
		ret = i2c_himax_master_write(ts->client,
				buf, 2, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			pr_err("[himax] %s: I2C access failed addr = 0x%x\n",
					 __func__, ts->client->addr);
		}
		msleep(50);
	}
#endif

	ret = i2c_himax_write_command(ts->client, 0x83, HIMAX_I2C_RETRY_TIMES);

	msleep(140);
	ret = i2c_himax_write_command(ts->client, 0x81, HIMAX_I2C_RETRY_TIMES);

	atomic_set(&ts->suspend_mode, 0);

	/*himax_power_on();*/

	himax_int_enable(ts->client->irq, 1);

#ifdef HX_CHIP_STATUS_MONITOR
	HX_CHIP_POLLING_COUNT = 0;
	queue_delayed_work(ts->himax_chip_monitor_wq,
		&ts->himax_chip_monitor, HX_POLLING_TIMER*HZ);
#endif

	ts->suspended = false;
	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct himax_ts_data *ts =
		container_of(self, struct himax_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
			ts->client) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			himax852xes_resume(&ts->client->dev);
		break;

		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			himax852xes_suspend(&ts->client->dev);
		break;
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts;

	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax852xes_suspend(ts->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts;

	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax852xes_resume(ts->client);

}
#endif

static const struct dev_pm_ops himax852xes_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = himax852xes_suspend,
	.resume = himax852xes_resume,
#else
	.suspend = himax852xes_suspend,
#endif
};


static const struct i2c_device_id himax852xes_ts_id[] = {
	{ HIMAX852xes_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id himax_match_table[] = {
	{ .compatible = "himax,852xes" },
	{ },
};
#else
#define himax_match_table NULL
#endif

#ifdef QCT
static struct i2c_driver himax852xes_driver = {
	.id_table = himax852xes_ts_id,
	.probe = himax852xes_probe,
	.remove	= himax852xes_remove,
	.driver	= {
		.name = HIMAX852xes_NAME,
		.owner = THIS_MODULE,
		.of_match_table = himax_match_table,
#ifdef CONFIG_PM
		.pm = &himax852xes_pm_ops,
#endif
	},
};
#endif

static void __init himax852xes_init_async(void *unused, async_cookie_t cookie)
{
	i2c_add_driver(&himax852xes_driver);
}

static int __init himax852xes_init(void)
{
	async_schedule(himax852xes_init_async, NULL);
	return 0;
}

static void __exit himax852xes_exit(void)
{
	i2c_del_driver(&himax852xes_driver);
}

module_init(himax852xes_init);
module_exit(himax852xes_exit);

MODULE_DESCRIPTION("Himax852xes driver");
MODULE_LICENSE("GPL");
