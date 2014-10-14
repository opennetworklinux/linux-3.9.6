/*
 * max6620.c - Simple RPM driver for the max6620.
 *
 * This driver is designed for the following operatinng modes only:
 *   1. All fans in RPM mode (via tach targets).
 *   2. Fan failure detection.
 *
 * TODO: Parameterize values via driver data.
 *
 * (C) Big Switch Networks, Inc. (support@bigswitch.com)
 *
 * based on code written by :
 * (C) 2012 by L. Grunenberg <contact@lgrunenberg.de>
 * 2007 by Hans J. Koch <hjk@hansjkoch.de>
 * John Morris <john.morris@spirentcom.com>
 * Copyright (c) 2003 Spirent Communications
 * and Claus Gindhart <claus.gindhart@kontron.com>
 *
 * This module has only been tested with the MAX6620 chip.
 *
 * The datasheet was last seen at:
 *
 *        http://datasheets.maximintegrated.com/en/ds/MAX6620.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>

/*
 * MAX 6620 registers
 */

#define MAX6620_REG_CONFIG	0x00
#define MAX6620_REG_FAULT	0x01
#define MAX6620_REG_CONF_FAN0	0x02
#define MAX6620_REG_CONF_FAN1	0x03
#define MAX6620_REG_CONF_FAN2	0x04
#define MAX6620_REG_CONF_FAN3	0x05
#define MAX6620_REG_DYN_FAN0	0x06
#define MAX6620_REG_DYN_FAN1	0x07
#define MAX6620_REG_DYN_FAN2	0x08
#define MAX6620_REG_DYN_FAN3	0x09
#define MAX6620_REG_TACH0	0x10
#define MAX6620_REG_TACH1	0x12
#define MAX6620_REG_TACH2	0x14
#define MAX6620_REG_TACH3	0x16
#define MAX6620_REG_VOLT0	0x18
#define MAX6620_REG_VOLT1	0x1A
#define MAX6620_REG_VOLT2	0x1C
#define MAX6620_REG_VOLT3	0x1E
#define MAX6620_REG_TAR0	0x20
#define MAX6620_REG_TAR1	0x22
#define MAX6620_REG_TAR2	0x24
#define MAX6620_REG_TAR3	0x26
#define MAX6620_REG_DAC0	0x28
#define MAX6620_REG_DAC1	0x2A
#define MAX6620_REG_DAC2	0x2C
#define MAX6620_REG_DAC3	0x2E

/*
 * Config register bits
 */

#define MAX6620_CFG_RUN			0x80
#define MAX6620_CFG_POR			0x40
#define MAX6620_CFG_TIMEOUT		0x20
#define MAX6620_CFG_FULLFAN		0x10
#define MAX6620_CFG_OSC			0x08
#define MAX6620_CFG_WD_MASK		0x06
#define MAX6620_CFG_WD_2		0x02
#define MAX6620_CFG_WD_6		0x04
#define MAX6620_CFG_WD10		0x06
#define MAX6620_CFG_WD			0x01


/*
 * Failure status register bits
 */

#define MAX6620_FAIL_TACH0	0x10
#define MAX6620_FAIL_TACH1	0x20
#define MAX6620_FAIL_TACH2	0x40
#define MAX6620_FAIL_TACH3	0x80
#define MAX6620_FAIL_MASK0	0x01
#define MAX6620_FAIL_MASK1	0x02
#define MAX6620_FAIL_MASK2	0x04
#define MAX6620_FAIL_MASK3	0x08


/*
 * Minimum and maximum values of the FAN-RPM.
 * TODO: Make these dynamic via platform driver data.
 */
#define FAN_RPM_MIN 1000
#define FAN_RPM_MAX 18000

static int max6620_probe(struct i2c_client *client,
                         const struct i2c_device_id *id);

static int max6620_remove(struct i2c_client *client);


/** Fan Configuration Registers */
static const u8 config_reg[] = {
    MAX6620_REG_CONF_FAN0,
    MAX6620_REG_CONF_FAN1,
    MAX6620_REG_CONF_FAN2,
    MAX6620_REG_CONF_FAN3,
};

/** Fan Dynamics Registers */
static const u8 dyn_reg[] = {
	MAX6620_REG_DYN_FAN0,
	MAX6620_REG_DYN_FAN1,
	MAX6620_REG_DYN_FAN2,
	MAX6620_REG_DYN_FAN3,
};

/** Measured Tach Registers -- Use to calculate actual speed */
static const u8 tach_reg[] = {
	MAX6620_REG_TACH0,
	MAX6620_REG_TACH1,
	MAX6620_REG_TACH2,
	MAX6620_REG_TACH3,
};

static const u8 volt_reg[] = {
	MAX6620_REG_VOLT0,
	MAX6620_REG_VOLT1,
	MAX6620_REG_VOLT2,
	MAX6620_REG_VOLT3,
};

/** Target Tach Registers -- Use to set desired speed. */
static const u8 target_reg[] = {
	MAX6620_REG_TAR0,
	MAX6620_REG_TAR1,
	MAX6620_REG_TAR2,
	MAX6620_REG_TAR3,
};


static const u8 dac_reg[] = {
	MAX6620_REG_DAC0,
	MAX6620_REG_DAC1,
	MAX6620_REG_DAC2,
	MAX6620_REG_DAC3,
};

static const struct i2c_device_id max6620_id[] = {
    { "max6620", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, max6620_id);


static struct i2c_driver max6620_driver = {
    .class		= I2C_CLASS_HWMON,
    .driver = {
        .name	= "max6620",
    },
    .probe		= max6620_probe,
    .remove		= max6620_remove,
    .id_table	= max6620_id,
};


static int calc_tach_to_rpm(int tach)
{
    /**
     * Calculation:
     *
     * Target Tach = 60 / (NP * RPM) * SR * 8192
     * Where (in this driver):
     *   NP = 2
     *   SR = 4
     */

    int np = 2;
    int sr = 4;

    /**
     * This makes the RPM Calculation:
     *   RPM = (491520 * SR) / (Tach * NP)
     * RPM = 491520 * SR / (TT * NP )
     */
    return (491520 * sr) / (tach * np);
}
static int calc_rpm_to_tach(int rpm)
{
    /** See above */
    int np = 2;
    int sr = 4;

    return (491520*sr) / (rpm * np);
}


static int get_tach_base(struct i2c_client* client, int base)
{
    int tach_high = i2c_smbus_read_byte_data(client, base);
    int tach_low = i2c_smbus_read_byte_data(client, base + 1);
    int tach = (tach_high << 8) | tach_low;
    return tach >> 5;
}

static int set_tach_base(struct i2c_client* client, int base, int tach)
{
    int rv = 0;
    int tach_high, tach_low;

    tach <<= 5;
    tach_high = tach >> 8;
    tach_low = tach & 0xFF;

    rv += i2c_smbus_write_byte_data(client, base, tach_high);
    rv += i2c_smbus_write_byte_data(client, base+1, tach_low);

    return rv;
}

static int get_rpm_base(struct i2c_client* client, int base)
{
    int tach = get_tach_base(client, base);
    return calc_tach_to_rpm(tach);
}

static int set_rpm_base(struct i2c_client* client, int base, int rpm)
{
    int tach = calc_rpm_to_tach(rpm);
    return set_tach_base(client, base, tach);
}

static int get_fan_rpm(struct device *dev,
                       struct device_attribute *devattr)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
    return get_rpm_base(client, tach_reg[attr->index]);
}
static int get_fan_target_rpm(struct device *dev,
                              struct device_attribute *devattr)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
    return get_rpm_base(client, target_reg[attr->index]);
}
static int set_fan_target_rpm(struct device *dev,
                              struct device_attribute *devattr, int rpm)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
    return set_rpm_base(client, target_reg[attr->index], rpm);
}

static ssize_t
get_fan(struct device *dev, struct device_attribute *devattr, char *buf)
{
    return sprintf(buf, "%d\n", get_fan_rpm(dev, devattr));
}
static ssize_t
get_target(struct device *dev, struct device_attribute *devattr, char *buf)
{
    return sprintf(buf, "%d\n", get_fan_target_rpm(dev, devattr));
}

static ssize_t
set_target(struct device *dev, struct device_attribute *devattr,
           const char *buf, size_t count)
{
    int err;
    unsigned long rpm;

    err = kstrtoul(buf, 10, &rpm);
    if(err) {
        return err;
    }

    if(rpm < FAN_RPM_MIN) {
        rpm = FAN_RPM_MIN;
    }
    if(rpm > FAN_RPM_MAX) {
        rpm = FAN_RPM_MAX;
    }
    set_fan_target_rpm(dev, devattr, rpm);
    return count;
}

/*
 * Get alarm stati:
 * Possible values:
 * 0 = no alarm
 * 1 = alarm
 */

static ssize_t
get_alarm(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
    struct i2c_client *client = to_i2c_client(dev);
    int fault = 0;

    fault = i2c_smbus_read_byte_data(client, MAX6620_REG_FAULT);
    /* Skip the masks */
    fault >>= 4;
    return sprintf(buf, "%d\n", (fault & (1 << attr->index)) ? 1 : 0);
}

static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, get_fan, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, get_fan, NULL, 1);
static SENSOR_DEVICE_ATTR(fan3_input, S_IRUGO, get_fan, NULL, 2);
static SENSOR_DEVICE_ATTR(fan4_input, S_IRUGO, get_fan, NULL, 3);
static SENSOR_DEVICE_ATTR(fan1_target, S_IWUSR | S_IRUGO, get_target, set_target, 0);
static SENSOR_DEVICE_ATTR(fan2_target, S_IWUSR | S_IRUGO, get_target, set_target, 1);
static SENSOR_DEVICE_ATTR(fan3_target, S_IWUSR | S_IRUGO, get_target, set_target, 2);
static SENSOR_DEVICE_ATTR(fan4_target, S_IWUSR | S_IRUGO, get_target, set_target, 3);
static SENSOR_DEVICE_ATTR(fan1_alarm, S_IRUGO, get_alarm, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_alarm, S_IRUGO, get_alarm, NULL, 1);
static SENSOR_DEVICE_ATTR(fan3_alarm, S_IRUGO, get_alarm, NULL, 2);
static SENSOR_DEVICE_ATTR(fan4_alarm, S_IRUGO, get_alarm, NULL, 3);

static struct attribute *max6620_attrs[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan1_target.dev_attr.attr,
	&sensor_dev_attr_fan2_target.dev_attr.attr,
	&sensor_dev_attr_fan3_target.dev_attr.attr,
	&sensor_dev_attr_fan4_target.dev_attr.attr,
	&sensor_dev_attr_fan1_alarm.dev_attr.attr,
	&sensor_dev_attr_fan2_alarm.dev_attr.attr,
	&sensor_dev_attr_fan3_alarm.dev_attr.attr,
	&sensor_dev_attr_fan4_alarm.dev_attr.attr,
	NULL
};

static struct attribute_group max6620_attr_grp = {
	.attrs = max6620_attrs,
};

struct max6620_data {
    struct device* device;
    struct mutex lock;
};

static int max6620_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int i;
    struct max6620_data *data;
    int err;

    data = devm_kzalloc(&client->dev, sizeof(struct max6620_data), GFP_KERNEL);
    if (!data) {
        dev_err(&client->dev, "out of memory.\n");
        return -ENOMEM;
    }

    i2c_set_clientdata(client, data);
    mutex_init(&data->lock);

    /*
     * Initialize the max6620 chip.
     *
     * We configure all fans in RPM mode.
     */
    for(i = 0; i < 4; i++) {
        i2c_smbus_write_byte_data(client, config_reg[i], 0xC0);
    }

    err = sysfs_create_group(&client->dev.kobj, &max6620_attr_grp);
    if (err)
        return err;

    data->device = hwmon_device_register(&client->dev);

    if (IS_ERR(data->device)) {
        err = PTR_ERR(data->device);
        dev_err(&client->dev, "error registering hwmon device.\n");
        sysfs_remove_group(&client->dev.kobj, &max6620_attr_grp);
        return err;
    }

    /*
     * Initialize all fans to max.
     * TODO: Make configurable through platform data.
     */
    for(i = 0; i < 4; i++) {
        set_rpm_base(client, target_reg[i], 18000);
    }

    return 0;
}

static int max6620_remove(struct i2c_client *client)
{
    struct max6620_data *data = i2c_get_clientdata(client);
    hwmon_device_unregister(data->device);
    sysfs_remove_group(&client->dev.kobj, &max6620_attr_grp);
    return 0;
}

module_i2c_driver(max6620_driver);

MODULE_AUTHOR("Big Switch Networks");
MODULE_DESCRIPTION("MAX6620 sensor driver");
MODULE_LICENSE("GPL");
