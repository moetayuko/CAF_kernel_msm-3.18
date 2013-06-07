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

#include <cutils/log.h>
#include "STKProximitySensor.h"

/*****************************************************************************/
STKProximitySensor::STKProximitySensor()
    : SensorBase(NULL, "proximity"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_P;
    mPendingEvent.type = SENSOR_TYPE_PROXIMITY;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

    if (data_fd)
	{
        strcpy(input_sysfs_path, "/sys/class/input/");
        strcat(input_sysfs_path, input_name);
        strcat(input_sysfs_path, "/device/driver/");
        input_sysfs_path_len = strlen(input_sysfs_path);
    }
}

STKProximitySensor::~STKProximitySensor()
{
	if (mEnabled)
		setEnable(ID_P,0);
}


int STKProximitySensor::setInitialState() {
    struct input_absinfo absinfo;
    if (!ioctl(data_fd, EVIOCGABS(ABS_DISTANCE), &absinfo)) {
        mPendingEvent.distance = indexToValue(absinfo.value ? 1:0);
        mHasPendingEvent = true;
    }
	else
		LOGE("%s:ioctl failed!", __func__);
    return 0;
}

int STKProximitySensor::setEnable(int32_t, int en) {
    int flags = en ? 1 : 0;
    if (flags != mEnabled)
	{
        int fd;
        strcpy(&input_sysfs_path[input_sysfs_path_len], "enable");
        fd = open(input_sysfs_path, O_RDWR);

        if (fd >= 0) {
            char buf[2];
            buf[1] = 0;
            if (flags) {
                buf[0] = '1';
            } else {
                buf[0] = '0';
            }
            write(fd, buf, sizeof(buf));
            close(fd);
            mEnabled = flags;
			if(flags)
				setInitialState();
            return 0;
        }
        return -1;
    }
    return 0;
}

int STKProximitySensor:: getEnable(int32_t) {
	return mEnabled ? true : false;
}

bool STKProximitySensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

float STKProximitySensor::indexToValue(size_t index) const
{
    return index * PROXIMITY_THRESHOLD_GP2A;
}

int STKProximitySensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS) {
            if (event->code == ABS_DISTANCE) {
                mPendingEvent.distance = indexToValue(event->value);
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if (mEnabled) {
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
            }
        } else {
            LOGE("STKProximitySensor: unknown event (type=%d, code=%d)",
                    type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}
