/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include "Cm36283.h"
#include <cutils/log.h>

#define SENSOR_NAME     "cm36283"
#define PROX_LOWTHRESHOLD  80	//set the low proximity threshold
#define PROX_HIGHTHRESHOLD  120	//set the high proximity threshold
int pre_value = 0;
/*****************************************************************************/
CmSensor::CmSensor(): SensorBase(NULL, SENSOR_NAME),
	mInputReader(32),
	alsEnabled(0),
	psEnabled(0),
	mHasPendingEvent(false)
{
	memset(&mPendingEvent, 0, sizeof(mPendingEvent));
	if(sensor_get_class_path()<0)
		LOGD("cm36283 sensor get class path error \n");

}

CmSensor::~CmSensor(){
}

int CmSensor::setInitialState() {
	return 0;
}

int CmSensor::setEnable(int32_t handle, int en) {
	char buffer[20];
	int als = 0;
	int ps = 0;
	int count = 0;
	if (handle == ID_L)  {
		if (!en)
		{
			alsEnabled = 0;
		}else
		{
			alsEnabled = 1;
			als = 1;
		}
		LOGD("cm36283 %s Light Sensor En=%d\n", __func__, en);
		count = sprintf(buffer, "%d\n", als);
		set_sysfs_input_attr(class_path,"enable_als",buffer,count);
	}
	if (handle == ID_P)  {
		if (!en)
		{
			psEnabled = 0;
		}else
		{
			psEnabled = 1;
			ps = 1;
		}
		LOGD("cm36283 %s Prox Sensor En=%d\n", __func__, en);
		count = sprintf(buffer, "%d\n", ps);
		set_sysfs_input_attr(class_path,"enable_ps",buffer,count);
	}
	return 0;
}

bool CmSensor::hasPendingEvents() const {
	return mHasPendingEvent;
}

int CmSensor::readEvents(sensors_event_t* data, int count)
{
	if (count < 1)
		return -EINVAL;

	ssize_t n = mInputReader.fill(data_fd);
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const* event;

	while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_ABS) {
			processEvent(event->code, event->value);
		} else if (type == EV_SYN) {
			int64_t time = timevalToNano(event->time);

			mPendingEvent.timestamp = time;
			*data++ = mPendingEvent;
			count--;
			numEventReceived++;

		} else {
			LOGE("Cm36283 Sensor: unknown event (type=%d, code=%d)",
					type, event->code);
		}
		mInputReader.next();
	}
	return numEventReceived;
}

void CmSensor::processEvent(int code, int value)
{
	switch (code) {
		case ABS_DISTANCE:
			mPendingEvent.version = sizeof(sensors_event_t);
			mPendingEvent.sensor = ID_P;
			mPendingEvent.type = SENSOR_TYPE_PROXIMITY;
			memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
			LOGE("cm36283 mPendingEvent.distance = %d\n", value);
			if(value < PROX_LOWTHRESHOLD){
				pre_value = 5;
				mPendingEvent.distance = 5;
			} else if(value > PROX_HIGHTHRESHOLD){
					pre_value = 0;
					mPendingEvent.distance = 0;
				} else{
					mPendingEvent.distance = pre_value;
				}
			break;
		case ABS_MISC:
			mPendingEvent.version = sizeof(sensors_event_t);
			mPendingEvent.sensor = ID_L;
			mPendingEvent.type = SENSOR_TYPE_LIGHT;
			memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
			mPendingEvent.light = value;
			LOGD("cm36283 mPendingEvent.light = %d\n", value);
			break;
		default:
			break;
	}

}
int CmSensor::getFd() const
{
	return data_fd;
}

int CmSensor::getEnable(int32_t handle) {
	int en = 0;
	if (handle == ID_L)
		en = alsEnabled ? 1 : 0;
	if (handle == ID_P)
		en = psEnabled ? 1 : 0;
	return en;
}

int CmSensor::sensor_get_class_path()
{
	char dirname[] = "/sys/class/input";
	char buf[256];
	int res;
	DIR *dir;
	struct dirent *de;
	int fd = -1;
	int found = 0;

	dir = opendir(dirname);
	if (dir == NULL)
		return -1;

	while((de = readdir(dir))) {
		if (strncmp(de->d_name, "input", strlen("input")) != 0) {
			continue;
		}

		sprintf(class_path, "%s/%s", dirname, de->d_name);
		snprintf(buf, sizeof(buf), "%s/name", class_path);

		fd = open(buf, O_RDONLY);
		if (fd < 0) {
			continue;
		}
		if ((res = read(fd, buf, sizeof(buf))) < 0) {
			close(fd);
			continue;
		}
		buf[res - 1] = '\0';
		if (strcmp(buf, SENSOR_NAME) == 0) {
			found = 1;
			close(fd);
			break;
		}

		close(fd);
		fd = -1;
	}
	closedir(dir);

	if (found) {
		return 0;
	}else {
		*class_path = '\0';
		return -1;
	}
}

int CmSensor:: set_sysfs_input_attr(char *class_path,
		const char *attr, char *value, int len)
{
	char path[256];
	int fd;

	if (class_path == NULL || *class_path == '\0'
			|| attr == NULL || value == NULL || len < 1) {
		return -EINVAL;
	}
	snprintf(path, sizeof(path), "%s/%s", class_path, attr);
	path[sizeof(path) - 1] = '\0';
	fd = open(path, O_RDWR);
	if (fd < 0) {
		return -errno;
	}
	if (write(fd, value, len) < 0) {
		close(fd);
		return -errno;
	}
	close(fd);
	return 0;
}
