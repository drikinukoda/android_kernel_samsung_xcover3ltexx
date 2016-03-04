/*
 * Copyright (C) 2012 Senodia.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/sensor/sensors_core.h>
#include <linux/power_supply.h>
#include "st480.h"

#define VENDOR_NAME			"SENODIA"
#define MODEL_NAME			"ST480"
#define MODULE_NAME			"magnetic_sensor"

struct st480_platform_data {
	u32 chip_pos;
};

struct st480_v {
	union {
		s16 v[3];
		struct {
			s16 t;
			s16 x;
			s16 y;
			s16 z;
		};
	};
};

struct st480_p {
	struct mutex lock;
	struct i2c_client *client;
#if defined(CONFIG_SENSORS_IIO)
	struct iio_dev *indio_dev;
#else
	struct input_dev *input;
#endif
	struct delayed_work work_magn;
	struct st480_platform_data *pdata;
	struct device *factory_device;
	struct st480_v magdata;
	atomic_t enable;
	atomic64_t delay;

	/* Remember state for suspend and resume functions */
	bool suspended;
#if defined(CONFIG_SENSORS_IIO)
	u64 timestamp;
#endif
};

#if defined(CONFIG_SENSORS_IIO)
#define IIO_BUFFER_6_BYTES	14 /* data 6 bytes + timestamp 8 bytes */

static const struct iio_chan_spec st480_channels[] = {
	{
		.type = IIO_MAGN,
		.channel = -1,
		.scan_index = 0,
		.scan_type = IIO_ST('s', IIO_BUFFER_6_BYTES * 8,
			IIO_BUFFER_6_BYTES * 8, 0)
	}
};
#endif

#if defined(CONFIG_MACH_XCOVER3LTE)
int cable_status = CABLE_STATUS_NONE;
#endif

/*
 * i2c transfer
 * read/write
 */
static int st480_i2c_transfer_data(struct i2c_client *client,
	int len, char *buf, int length)
{
	int err = 0;
	struct i2c_msg msgs[] = {
		{
			.addr  =  client->addr,
			.flags  =  0,
			.len  =  len,
			.buf  =  buf,
		},
		{
			.addr  =  client->addr,
			.flags  = I2C_M_RD,
			.len  =  length,
			.buf  =  buf,
		},
	};

	err = i2c_transfer(client->adapter, msgs, 2);
	if (err < 0) {
		pr_err("%s: transfer error ret = %d\n", __func__, err);
		return -EIO;
	} else
		return 0;
}

static int st480_detect(struct i2c_client *client)
{
	int ret = 0;
	unsigned char buf[3] = {0,};

	buf[0] = READ_REGISTER_CMD;
	buf[1] = DEVICE_ID_REG;

	while (st480_i2c_transfer_data(client, 2, buf, 3) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(client, 2, buf, 3) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	if (((buf[1] << 8)|buf[2]) != DEVICE_ID)
		return -ENODEV;

	return 0;
}

static int st480_set_mode(struct st480_p *data)
{
	char buffer[1];
	int ret;

	/*set mode config*/
	buffer[0] = SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 1, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 1, buffer, 1) == 0)
			break;

		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	return 0;
}

static int st480_configure(struct st480_p *data)
{
	int ret;
	unsigned char buf[4] = {0,};

/*init register step 1*/
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = ONE_INIT_DATA_HIGH;
	buf[2] = ONE_INIT_DATA_LOW;
	buf[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 4, buf, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

/*init register step 2*/
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = TWO_INIT_DATA_HIGH;
	buf[2] = TWO_INIT_DATA_LOW;
	buf[3] = TWO_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 4, buf, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

/*set calibration register*/
	buf[0] = WRITE_REGISTER_CMD;
	buf[1] = CALIBRATION_DATA_HIGH;
	buf[2] = CALIBRATION_DATA_LOW;
	buf[3] = CALIBRATION_REG;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 4, buf, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 4, buf, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

/*set mode*/
	ret = st480_set_mode(data);

	return ret;
}


void st480_remap_axis(struct st480_p *data,
	struct st480_v *rawdata, struct st480_v *mag)
{
	if (ST480_SIZE_2X2_BGA) {
		if (data->pdata->chip_pos == 0) {
			mag->x = rawdata->x;
			mag->y = rawdata->y;
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 1) {
			mag->x = rawdata->y;
			mag->y = (-1)*(rawdata->x);
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 2) {
			mag->x = (-1)*(rawdata->x);
			mag->y = (-1)*(rawdata->y);
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 3) {
			mag->x = (-1)*(rawdata->y);
			mag->y = rawdata->x;
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 4) {
			mag->x = (-1)*(rawdata->x);
			mag->y = rawdata->y;
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 5) {
			mag->x = (-1)*(rawdata->y);
			mag->y = (-1)*(rawdata->x);
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 6) {
			mag->x = rawdata->x;
			mag->y = (-1)*(rawdata->y);
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 7) {
			mag->x = rawdata->y;
			mag->y = rawdata->x;
			mag->z = (-1)*(rawdata->z);
		}
	} else if (ST480_SIZE_1_6X1_6_LGA) {
		if (data->pdata->chip_pos == 0) {
			mag->x = rawdata->y;
			mag->y = rawdata->x;
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 1) {
			mag->x = rawdata->x;
			mag->y = (-1)*(rawdata->y);
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 2) {
			mag->x = (-1)*(rawdata->y);
			mag->y = (-1)*(rawdata->x);
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 3) {
			mag->x = (-1)*(rawdata->x);
			mag->y = rawdata->y;
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 4) {
			mag->x = (-1)*(rawdata->y);
			mag->y = rawdata->x;
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 5) {
			mag->x = (-1)*(rawdata->x);
			mag->y = (-1)*(rawdata->y);
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 6) {
			mag->x = rawdata->y;
			mag->y = (-1)*(rawdata->x);
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 7) {
			mag->x = rawdata->x;
			mag->y = rawdata->y;
			mag->z = rawdata->z;
		}
	} else if (ST480_SIZE_1_6X1_6_BGA) {
		if (data->pdata->chip_pos == 0) {
			mag->x = rawdata->y;
			mag->y = (-1)*(rawdata->x);
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 1) {
			mag->x = (-1)*(rawdata->x);
			mag->y = (-1)*(rawdata->y);
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 2) {
			mag->x = (-1)*(rawdata->y);
			mag->y = rawdata->x;
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 3) {
			mag->x = rawdata->x;
			mag->y = rawdata->y;
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 4) {
			mag->x = (-1)*(rawdata->y);
			mag->y = (-1)*(rawdata->x);
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 5) {
			mag->x = rawdata->x;
			mag->y = (-1)*(rawdata->y);
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 6) {
			mag->x = rawdata->y;
			mag->y = rawdata->x;
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 7) {
			mag->x = (-1)*(rawdata->x);
			mag->y = rawdata->y;
			mag->z = (-1)*(rawdata->z);
		}
	} else if (ST480_SIZE_1_2X1_2) {
		if (data->pdata->chip_pos == 0) {
			mag->x = rawdata->y;
			mag->y = rawdata->x;
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 1) {
			mag->x = rawdata->x;
			mag->y = (-1)*(rawdata->y);
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 2) {
			mag->x = (-1)*(rawdata->y);
			mag->y = (-1)*(rawdata->x);
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 3) {
			mag->x = (-1)*(rawdata->x);
			mag->y = rawdata->y;
			mag->z = (-1)*(rawdata->z);
		} else if (data->pdata->chip_pos == 4) {
			mag->x = (-1)*(rawdata->y);
			mag->y = rawdata->x;
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 5) {
			mag->x = (-1)*(rawdata->x);
			mag->y = (-1)*(rawdata->y);
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 6) {
			mag->x = rawdata->y;
			mag->y = (-1)*(rawdata->x);
			mag->z = rawdata->z;
		} else if (data->pdata->chip_pos == 7) {
			mag->x = rawdata->x;
			mag->y = rawdata->y;
			mag->z = rawdata->z;
		}
	}

#if defined(CONFIG_MACH_XCOVER3LTE)
	switch (cable_status) {
	/*Xcover3 TA charging compensation*/
	case CABLE_STATUS_TA:
		mag->x = mag->x + TA_X_CHARGE_COMPE;
		mag->y = mag->y + TA_Y_CHARGE_COMPE;
		mag->z = mag->z + TA_Z_CHARGE_COMPE;
		break;
	/*Xcover3 usb charging compensation*/
	case CABLE_STATUS_USB:
		mag->x = mag->x + USB_X_CHARGE_COMPE;
		mag->y = mag->y + USB_Y_CHARGE_COMPE;
		mag->z = mag->z + USB_Z_CHARGE_COMPE;
		break;
	default:
		break;
	}
#endif
}

static int st480_read_adc(struct st480_p *data, struct st480_v *rawdata)
{
	char buffer[9] = {0,};
	int ret;

	buffer[0] = READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 1, buffer, 9) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 1, buffer, 9) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	if (((buffer[0]>>4) & 0X01))
		return -EAGAIN;

	rawdata->x = (buffer[3]<<8)|buffer[4];
	rawdata->y = (buffer[5]<<8)|buffer[6];
	rawdata->z = (buffer[7]<<8)|buffer[8];

	rawdata->t = (buffer[1]<<8)|buffer[2];

	/*Temperature compensation	*/
	if (((buffer[1]<<8)|(buffer[2])) > 46244) {
		rawdata->x = rawdata->x * (1 + (70/128/4096)
			* (((buffer[1]<<8)|(buffer[2])) - 46244));
		rawdata->y = rawdata->y * (1 + (70/128/4096)
			* (((buffer[1]<<8)|(buffer[2])) - 46244));
		rawdata->z = rawdata->z * (1 + (70/128/4096)
			* (((buffer[1]<<8)|(buffer[2])) - 46244));
	} else if (((buffer[1]<<8)|(buffer[2])) < 46244) {
		rawdata->x = rawdata->x * (1 + (60/128/4096)
			* (((buffer[1]<<8)|(buffer[2])) - 46244));
		rawdata->y = rawdata->y * (1 + (60/128/4096)
			* (((buffer[1]<<8)|(buffer[2])) - 46244));
		rawdata->z = rawdata->z * (1 + (60/128/4096)
			* (((buffer[1]<<8)|(buffer[2])) - 46244));
	}

	return 0;
}

static int st480_read_axis(struct st480_p *data, struct st480_v *mag)
{
	int ret;
	struct st480_v rawdata;

	ret = st480_read_adc(data, &rawdata);
	if (ret)
		pr_err("%s error %d\n", __func__, ret);
	else
		st480_remap_axis(data, &rawdata, mag);

	/*set mode*/
	ret = st480_set_mode(data);

	return ret;
}

static int st480_single_measure_mode_bist(struct st480_p *data)
{
	char buffer[1];
	int ret;

	/*set mode config*/
	buffer[0] = BIST_SINGLE_MEASUREMENT_MODE_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 1, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 1, buffer, 1) == 0)
			break;

		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}
	return 0;
}

static int st480_read_bist(struct st480_p *data, struct st480_v *mag)
{
	char buffer[3] = {0,};
	int ret;

	buffer[0] = BIST_READ_MEASUREMENT_CMD;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 1, buffer, 3) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 1, buffer, 3) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT)
			return -EIO;
	}

	if (((buffer[0]>>4) & 0X01)) {
		pr_err("[SENSOR]: %s error\n", __func__);
		return -EAGAIN;
	}

	mag->z = (buffer[1]<<8)|buffer[2];

	return 0;
}

static void st480_work_func(struct work_struct *work)
{
	struct st480_v mag;
	struct st480_p *data = container_of((struct delayed_work *)work,
			struct st480_p, work_magn);
#if defined(CONFIG_SENSORS_IIO)
	struct iio_dev *indio_dev = iio_priv_to_dev(data);
	struct timespec ts;
	u8 buf[IIO_BUFFER_6_BYTES];
#endif
	unsigned long delay = nsecs_to_jiffies(atomic_read(&data->delay));

	st480_read_axis(data, &mag);

#if defined(CONFIG_SENSORS_IIO)
	ts = ktime_to_timespec(ktime_get_boottime());
	data->timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

	memcpy(buf, &mag.x, sizeof(mag.x));
	memcpy(buf + 2, &mag.y, sizeof(mag.y));
	memcpy(buf + 4, &mag.z, sizeof(mag.z));
	memcpy(buf + 6, &data->timestamp, sizeof(data->timestamp));

	iio_push_to_buffers(indio_dev, buf);
#else

	input_report_rel(data->input, REL_X, mag.x);
	input_report_rel(data->input, REL_Y, mag.y);
	input_report_rel(data->input, REL_Z, mag.z);
	input_sync(data->input);
#endif

	data->magdata = mag;

	schedule_delayed_work(&data->work_magn, delay);
}

static void st480_set_enable(struct st480_p *data, int enable)
{
	int pre_enable = atomic_read(&data->enable);

	if (enable) {
		if (pre_enable == 0) {
			schedule_delayed_work(&data->work_magn,
				nsecs_to_jiffies(atomic_read(&data->delay)));
			atomic_set(&data->enable, 1);
		}
	} else {
		if (pre_enable == 1) {
			cancel_delayed_work_sync(&data->work_magn);
			atomic_set(&data->enable, 0);
		}
	}
}

/*
 *  0 = off, 1 = on.
 */
static ssize_t st480_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_SENSORS_IIO)
	struct st480_p *data = iio_priv(dev_get_drvdata(dev));
#else
	struct st480_p *data = dev_get_drvdata(dev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->enable));
}

static ssize_t st480_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
#if defined(CONFIG_SENSORS_IIO)
	struct st480_p *data = iio_priv(dev_get_drvdata(dev));
#else
	struct st480_p *data = dev_get_drvdata(dev);
#endif
	u8 enable;
	int ret;

	ret = kstrtou8(buf, 2, &enable);
	if (ret) {
		pr_err("%s Invalid Argument\n", __func__);
		return ret;
	}

	pr_info("%s, pre_enable = %d, enable = %d\n", __func__,
		atomic_read(&data->enable), enable);

	if ((enable == 0) || (enable == 1))
		st480_set_enable(data, (int)enable);

	return size;
}

static ssize_t st480_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_SENSORS_IIO)
	struct st480_p *data = iio_priv(dev_get_drvdata(dev));
#else
	struct st480_p *data = dev_get_drvdata(dev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->delay));
}

static ssize_t st480_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
#if defined(CONFIG_SENSORS_IIO)
	struct st480_p *data = iio_priv(dev_get_drvdata(dev));
#else
	struct st480_p *data = dev_get_drvdata(dev);
#endif
	int ret;
	int64_t delay;

	ret = kstrtoll(buf, 10, &delay);
	if (ret) {
		pr_err("%s Invalid Argument\n", __func__);
		return ret;
	}

	atomic_set(&data->delay, delay);
	pr_info("%s poll_delay = %lld\n", __func__, delay);

	return size;
}

#if defined(CONFIG_SENSORS_IIO)
static IIO_DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	st480_delay_show, st480_delay_store, 0);
static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	st480_enable_show, st480_enable_store, 0);

static struct attribute *st480_attributes[] = {
	&iio_dev_attr_poll_delay.dev_attr.attr,
	&iio_dev_attr_enable.dev_attr.attr,
	NULL,
};
#else
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		st480_delay_show, st480_delay_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		st480_enable_show, st480_enable_store);

static struct attribute *st480_attributes[] = {
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	NULL
};
#endif

static struct attribute_group st480_attribute_group = {
	.attrs = st480_attributes
};

static ssize_t st480_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t st480_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", MODEL_NAME);
}

static ssize_t st480_read_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	bool success = false;
	int ret;
	struct st480_p *data = dev_get_drvdata(dev);
	struct st480_v mag = data->magdata;

	if (atomic_read(&data->enable) == 1) {
		success = true;
		msleep(20);
		goto exit;
	}

	ret = st480_read_axis(data, &mag);

	if (ret < 0)
		success = false;
	else
		success = true;

	data->magdata = mag;

exit:
	return snprintf(buf, PAGE_SIZE, "%s,%d,%d,%d\n",
			(success ? "OK" : "NG"), mag.x, mag.y, mag.z);
}

static ssize_t st480_raw_data_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct st480_p *data = dev_get_drvdata(dev);
	struct st480_v mag = data->magdata;

	if (atomic_read(&data->enable) == 1) {
		msleep(20);
		goto exit;
	}

	msleep(20);
	ret = st480_read_axis(data, &mag);

	data->magdata = mag;

exit:
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", mag.x, mag.y, mag.z);
}

static ssize_t st480_get_selftest(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int sf_ret = 0;
	int reg_zero = 0;
	int reg_one = 0;
	struct st480_p *data = dev_get_drvdata(dev);
	struct st480_v rawdata;
	unsigned char buffer[4] = {0,};
	s16 zh[3] = {0};

	pr_info("[SENSOR]: %s start\n", __func__);

	/*Step 1: Read default value:*/
	if (atomic_read(&data->enable) == 1)
		cancel_delayed_work_sync(&data->work_magn);

	usleep_range(30000, 30100);
	ret = 0;
	buffer[0] = READ_REGISTER_CMD;
	buffer[1] = ONE_INIT_REG;

	while (st480_i2c_transfer_data(data->client, 2, buffer, 3) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 2, buffer, 3) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n",
				__func__);
		}
	}

	reg_zero = ((buffer[1] << 8)|buffer[2]);

	if (reg_zero != REGISTER_ZERO_DEFAULT_VALUE) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s check default value error. reg_zero = %d\n",
			__func__, reg_zero);
	}

	ret = 0;
	buffer[0] = READ_REGISTER_CMD;
	buffer[1] = REGISTER_ONE;

	while (st480_i2c_transfer_data(data->client, 2, buffer, 3) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 2, buffer, 3) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n",
				__func__);
		}
	}

	reg_one = ((buffer[1] << 8)|buffer[2]);

	if (reg_one != REGISTER_ONE_DEFAULT_VALUE) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s check default value error. reg_one = %d\n",
			__func__, reg_one);
	}

	/*Step 2, step 3 (temperature output and ADC vaue)*/
	ret = st480_read_adc(data, &rawdata);

	if ((rawdata.t > -14000) || (rawdata.t < -24000)) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s temperature value error\n", __func__);
	}

	if ((rawdata.x > 10672) || (rawdata.x < -10672)
		|| (rawdata.y > 10672) || (rawdata.y < -10672)
		|| (rawdata.z > 6400) || (rawdata.z < -6400)) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s adc value error\n", __func__);
	}

	pr_info("[SENSOR]: %s adc value x=%d, y=%d, z=%d\n",
		__func__, rawdata.x, rawdata.y, rawdata.z);

	zh[0] = rawdata.z;

	/*step 4(self test)*/
	buffer[0] = WRITE_REGISTER_CMD;
	buffer[1] = ONE_INIT_BIST_TEST;
	buffer[2] = ONE_INIT_DATA_LOW;
	buffer[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 4, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 4, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n",
				__func__);
		}
	}

	usleep_range(150000, 150100);
	st480_single_measure_mode_bist(data);

	usleep_range(30000, 30100);
	st480_read_bist(data, &rawdata);

	zh[1] = rawdata.z;
	zh[2] = (abs(zh[1])) - (abs(zh[0]));

	pr_info("[SENSOR]: %s zh[0] = %d, zh[1] = %d, zh[2] = %d\n",
		__func__, zh[0], zh[1], zh[2]);

	rawdata.z = zh[0];

	/*Reduce to the initial setup*/
	buffer[0] = WRITE_REGISTER_CMD;
	buffer[1] = ONE_INIT_DATA_HIGH;
	buffer[2] = ONE_INIT_DATA_LOW;
	buffer[3] = ONE_INIT_REG;
	ret = 0;
	while (st480_i2c_transfer_data(data->client, 4, buffer, 1) != 0) {
		ret++;
		usleep_range(1000, 1100);
		if (st480_i2c_transfer_data(data->client, 4, buffer, 1) == 0)
			break;
		if (ret > MAX_FAILURE_COUNT) {
			sf_ret = -1;
			pr_err("[SENSOR]: %s i2c transfer data error\n",
				__func__);
		}
	}

	/*set mode*/
	st480_set_mode(data);

	if ((abs(zh[2]) <= 90) || (abs(zh[2]) >= 180)) {
		sf_ret = -1;
		pr_err("[SENSOR]: %s BIST test error\n", __func__);
	}

	if (sf_ret == 0)
		pr_info("[SENSOR]: %s success\n", __func__);
	else
		pr_info("[SENSOR]: %s fail\n", __func__);

	if (atomic_read(&data->enable) == 1) {
		schedule_delayed_work(&data->work_magn,
			nsecs_to_jiffies(atomic_read(&data->delay)));
	}

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		sf_ret, reg_zero, reg_one, zh[0], zh[1], zh[2],
		rawdata.x, rawdata.y, rawdata.z);
}

static DEVICE_ATTR(name, S_IRUGO, st480_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, st480_vendor_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, st480_read_raw, NULL);
static DEVICE_ATTR(adc, S_IRUGO, st480_raw_data_read, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, st480_get_selftest, NULL);

/*static DEVICE_ATTR(dac, S_IRUGO, st480_check_cntl, NULL);
static DEVICE_ATTR(chk_registers, S_IRUGO, st480_check_registers, NULL);
static DEVICE_ATTR(asa, S_IRUGO, st480_get_asa, NULL);
static DEVICE_ATTR(status, S_IRUGO, st480_get_status, NULL);*/

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_raw_data,
	&dev_attr_adc,
	&dev_attr_selftest,
	/*&dev_attr_dac,
	&dev_attr_chk_registers,
	&dev_attr_asa,
	&dev_attr_status,*/
	NULL,
};

/* magnetic sensor compensation at TA/USB or no cable status - H/W request */
#if defined(CONFIG_MACH_XCOVER3LTE)
static void st480_select_value(int status)
{
	switch (status) {
	case POWER_SUPPLY_TYPE_MAINS:		/* TA */
		pr_info("%s: charger status TA\n", __func__);
		cable_status = CABLE_STATUS_TA;
		break;
	case POWER_SUPPLY_TYPE_MISC:		/* USB */
	case POWER_SUPPLY_TYPE_USB:		/* USB */
		pr_info("%s: charger status USB\n", __func__);
		cable_status = CABLE_STATUS_USB;
		break;
	case POWER_SUPPLY_TYPE_UNKNOWN:		/* ignore(skip) */
	case POWER_SUPPLY_TYPE_BATTERY:		/* no cable */
		pr_info("%s: charger status no cable\n", __func__);
		cable_status = CABLE_STATUS_NONE;
		break;
	default:
		pr_info("%s: status none\n", __func__);
		cable_status = CABLE_STATUS_NONE;
		break;
	}
}

void magnetic_charger_status_cb(int status)
{
	pr_info("[st480]: charger status %d\n", status);
	st480_select_value(status);
}

EXPORT_SYMBOL(magnetic_charger_status_cb);
#endif


#if defined(CONFIG_SENSORS_IIO)
static int st480_iio_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan,
	int *val, int *val2, long m)
{
	struct st480_p *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (chan->type != IIO_MAGN) {
		pr_err("%s, invalied type\n", __func__);
		return ret;
	}

	switch (chan->channel2) {
	case IIO_MOD_X:
		*val = data->magdata.x;
		break;
	case IIO_MOD_Y:
		*val = data->magdata.y;
		break;
	case IIO_MOD_Z:
		*val = data->magdata.z;
		break;
	default:
		pr_err("%s, invalied channel\n", __func__);
		return ret;
	}

	return IIO_VAL_INT;
}

static int st480_read_event_config(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct st480_p *data = iio_priv(indio_dev);

	return atomic_read(&data->enable);
}

static const struct iio_info st480_info = {
	.attrs = &st480_attribute_group,
	.driver_module = THIS_MODULE,
	.read_raw = st480_iio_read_raw,
	.read_event_config = st480_read_event_config,
};

#else /* !defined(CONFIG_SENSORS_IIO) */

static int st480_input_init(struct st480_p *data)
{
	int ret = 0;
	struct input_dev *dev;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = MODULE_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_X);
	input_set_capability(dev, EV_REL, REL_Y);
	input_set_capability(dev, EV_REL, REL_Z);
	input_set_drvdata(dev, data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	ret = sensors_create_symlink(&dev->dev.kobj, dev->name);
	if (ret < 0) {
		input_unregister_device(dev);
		return ret;
	}

	/* sysfs node creation */
	ret = sysfs_create_group(&dev->dev.kobj, &st480_attribute_group);
	if (ret < 0) {
		sensors_remove_symlink(&data->input->dev.kobj,
			data->input->name);
		input_unregister_device(dev);
		return ret;
	}

	data->input = dev;
	return 0;
}
#endif

static int st480_parse_dt(struct device *dev,
	struct st480_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	if (pdata == NULL)
		return -ENODEV;

	ret = of_property_read_u32(np, "st480,chip_pos",
		&pdata->chip_pos);
	if (ret < 0)
		pr_err("%s: failed to get chip_pos dts\n", __func__);

	pr_info("%s chip_pos: %d\n", __func__, pdata->chip_pos);

	return 0;
}

static int st480_regulator_onoff(struct device *dev, bool onoff)
{
	struct regulator *vdd;
	int ret = 0;

	pr_info("%s %s\n", __func__, (onoff) ? "on" : "off");

	vdd = devm_regulator_get(dev, "st480,vdd");
	if (IS_ERR(vdd)) {
		pr_err("%s: cannot get vdd\n", __func__);
		ret = -ENOMEM;
		goto err_vdd;
	}
	ret = regulator_set_voltage(vdd, 2850000, 2850000);

	if (onoff) {
		ret = regulator_enable(vdd);
		if (ret)
			pr_err("%s: Failed to enable vdd.\n", __func__);
	} else {
		ret = regulator_disable(vdd);
		if (ret)
			pr_err("%s: Failed to enable vdd.\n", __func__);
	}
	msleep(20);

	devm_regulator_put(vdd);
err_vdd:
	return ret;
}


static int st480_probe(struct i2c_client *client,
	const struct i2c_device_id *device_id)
{
#if defined(CONFIG_SENSORS_IIO)
	struct iio_dev *indio_dev;
#endif
	struct st480_p *data;
	struct st480_platform_data *pdata;
	int ret = -ENODEV;

	pr_info("%s - Probe Start!\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s - i2c_check_functionality error\n",
			__func__);
		return -ENODEV;
	}

	st480_regulator_onoff(&client->dev, true);

	ret = st480_detect(client);
	if (ret) {
		pr_err("%s, detect device failed\n", __func__);
		goto exit_detect_device;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct st480_platform_data), GFP_KERNEL);
		ret = st480_parse_dt(&client->dev, pdata);
		if (ret < 0) {
			pr_err("%s, failed parse dt (%d)\n", __func__, ret);
			ret = -ENODEV;
			goto exit_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		pr_err("%s, missing pdata!\n", __func__);
		ret = -ENOMEM;
		goto exit_no_pdata;
	}

#if defined(CONFIG_SENSORS_IIO)
	/* Configure IIO device */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev) {
		pr_err("%s, iio_device_alloc failed\n", __func__);
		ret = -ENOMEM;
		goto exit_mem_alloc;
	}

	data = iio_priv(indio_dev);
	dev_set_name(&indio_dev->dev, "magnetic_sensor");
	i2c_set_clientdata(client, indio_dev);

	/* Init IIO device */
	indio_dev->name = client->name;
	indio_dev->channels = st480_channels;
	indio_dev->num_channels = ARRAY_SIZE(st480_channels);
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &st480_info;

	ret = sensors_iio_configure_buffer(indio_dev);
	if (ret) {
		pr_err("%s, configure ring buffer failed\n", __func__);
		goto exit_iio_configure_buffer_failed;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		pr_err("%s: iio_device_register failed(%d)\n", __func__, ret);
		goto exit_iio_dev_register_failed;
	}

	data->client = client;
	data->pdata = pdata;

#else	/* !defined(CONFIG_SENSORS_IIO)*/

	/* Allocate memory for driver data */
	data = kzalloc(sizeof(struct st480_p), GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s, memory allocation failed.\n", __func__);
		ret = -ENOMEM;
		goto exit_mem_alloc;
	}

	i2c_set_clientdata(client, data);

	data->client = client;
	data->pdata = pdata;

	/* input device init */
	ret = st480_input_init(data);
	if (ret < 0)
		goto exit_input_init;

#endif

	ret = sensors_register(data->factory_device, data, sensor_attrs,
		MODULE_NAME);
	if (ret) {
		pr_err("%s, failed to sensors_register (%d)\n", __func__, ret);
		goto exit_sensor_register_failed;
	}

	/* workqueue init */
	INIT_DELAYED_WORK(&data->work_magn, st480_work_func);
	mutex_init(&data->lock);

	atomic_set(&data->delay, ST480_DEFAULT_DELAY);
	atomic_set(&data->enable, 0);

	ret = st480_configure(data);
	if (ret) {
		pr_err("%s, device configure error %d\n", __func__, ret);
		goto exit_configure_failed;
	}

	pr_info("%s, done!\n", __func__);

	return 0;

exit_configure_failed:
	sensors_unregister(data->factory_device, sensor_attrs);
exit_sensor_register_failed:
#if defined(CONFIG_SENSORS_IIO)
	iio_device_unregister(indio_dev);
exit_iio_dev_register_failed:
	sensors_iio_unconfigure_buffer(indio_dev);
exit_iio_configure_buffer_failed:
#else
	sysfs_remove_group(&data->input->dev.kobj, &st480_attribute_group);
	sensors_remove_symlink(&data->input->dev.kobj, data->input->name);
	input_unregister_device(data->input);
exit_input_init:
	kfree(data);
#endif
exit_mem_alloc:
exit_no_pdata:
exit_parse_dt:
exit_detect_device:
	st480_regulator_onoff(&client->dev, false);
	pr_err("%s Probe fail (ret = %d)\n", __func__, ret);
	return ret;
}

static int st480_remove(struct i2c_client *client)
{
#if defined(CONFIG_SENSORS_IIO)
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct st480_p *data = iio_priv(indio_dev);
#else
	struct st480_p *data = i2c_get_clientdata(client);
#endif

	pr_info("%s\n", __func__);

	mutex_destroy(&data->lock);
	sensors_unregister(data->factory_device, sensor_attrs);
#if defined(CONFIG_SENSORS_IIO)
	iio_device_unregister(indio_dev);
	sensors_iio_unconfigure_buffer(indio_dev);
#else
	sysfs_remove_group(&data->input->dev.kobj, &st480_attribute_group);
	sensors_remove_symlink(&data->input->dev.kobj, data->input->name);
	input_unregister_device(data->input);

	kfree(data);
#endif
	st480_regulator_onoff(&client->dev, false);

	return 0;
}

static int st480_suspend(struct device *dev)
{
#if defined(CONFIG_SENSORS_IIO)
	struct st480_p *data = iio_priv(i2c_get_clientdata(
						to_i2c_client(dev)));
#else
	struct st480_p *data = dev_get_drvdata(dev);
#endif

	pr_info("%s is called\n", __func__);

	if (atomic_read(&data->enable) == 1)
		cancel_delayed_work_sync(&data->work_magn);

	mutex_lock(&data->lock);
	data->suspended = true;
	mutex_unlock(&data->lock);

	return 0;
}

static int st480_resume(struct device *dev)
{
#if defined(CONFIG_SENSORS_IIO)
	struct st480_p *data = iio_priv(i2c_get_clientdata(
						to_i2c_client(dev)));
#else
	struct st480_p *data = dev_get_drvdata(dev);
#endif
	int ret;

	pr_info("%s is called\n", __func__);

	mutex_lock(&data->lock);

	ret = st480_configure(data);
	if (ret) {
		pr_err("%s, configure failed\n", __func__);
		goto out;
	}

	data->suspended = false;

	if (atomic_read(&data->enable))
		schedule_delayed_work(&data->work_magn,
		nsecs_to_jiffies(atomic_read(&data->delay)));
out:
	mutex_unlock(&data->lock);
	return ret;
}

static const struct i2c_device_id st480_id[] = {
	{ "st480", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, st480_id);

#ifdef CONFIG_OF
static struct of_device_id magnetic_match_table[] = {
	{.compatible = "magnetic,st480",},
	{},
};
#else
#define magnetic_match_table NULL
#endif

static const struct dev_pm_ops st480_pm_ops = {
	.suspend = st480_suspend,
	.resume = st480_resume
};
static struct i2c_driver st480_driver = {
	.driver     = {
		.name	= MODEL_NAME,
		.owner	= THIS_MODULE,
		.pm = &st480_pm_ops,
		.of_match_table = magnetic_match_table,
	},
	.probe		= st480_probe,
	.remove     = st480_remove,
	.id_table	= st480_id,
};

static int __init st480_init(void)
{
	return i2c_add_driver(&st480_driver);
}

static void __exit st480_exit(void)
{
	i2c_del_driver(&st480_driver);
}

module_init(st480_init);
module_exit(st480_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("senodia st480 linux driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
