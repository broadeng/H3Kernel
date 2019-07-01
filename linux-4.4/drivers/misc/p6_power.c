/*
 * Power supply driver for P6.
 *
 * Copyright (C) 2018 Peter Wei <peter@microbt.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>

/*
 * P6 Version ID
 */
#define P6_VERSION_ID				0x22

/*
* P6 Operations (Read)
*/
#define P6_CMD_READ_VERSION 		0x00	/* Get Data Byte: 1 */
#define P6_CMD_READ_STATUS 			0x04	/* Get Data Byte: 1 */
#define P6_CMD_READ_ERRORCODE 		0x05	/* Get Data Byte: 1 */
#define P6_CMD_READ_VOUT_12V 		0x06	/* Get Data Byte: 2 */
#define P6_CMD_READ_VOUT 			0x07	/* Get Data Byte: 2 */
#define P6_CMD_READ_IOUT 			0x08	/* Get Data Byte: 2 */
#define P6_CMD_READ_POUT 			0x09	/* Get Data Byte: 2 */
#define P6_CMD_READ_TEMP0 			0x0A	/* Get Data Byte: 2 */
#define P6_CMD_READ_TEMP1 			0x0B	/* Get Data Byte: 2 */
#define P6_CMD_READ_FAN_PWM_DUTY	0x0D	/* Get Data Byte: 2 */

/*
* P6 Operations (Write)
*/
#define P6_CMD_WRITE_ON_OFF 		0x02	/* Power on/off the power supply, Data Byte: 1, Write 1: on, Write 0: off */
#define P6_CMD_CLEAR_FAULT 			0x03	/* Clear fault conditions, Data Byte: 0 */
#define P6_CMD_SET_VOUT 			0x12	/* Set Power VOUT, Data Byte: 2, Unit: 0.01V, Range: 12-17V */

/*
 * P6 Data Structure
 */
struct p6_data {
	struct i2c_client *client;
	struct mutex update_mutex;
};

/*
 * P6 CRC & I2C Operations
 */
static u8 basic_crc(u8 remainder, unsigned char nbyte)
{
    char i = 0;
    remainder ^= nbyte;
    for (i = 8; i > 0; --i) {
        if (remainder & 0x80) {
            remainder = (remainder << 1) ^ 0x7;
        }
        else {
            remainder = (remainder << 1);
        }
    }
    return remainder;
}

static u8 calc_crc(unsigned char i2c_addr, unsigned char i2c_command,
                   unsigned char *data, unsigned int len)
{
    int i;
    unsigned char crc = 0;
    crc = basic_crc(crc, (i2c_addr << 1) + 0);
    crc = basic_crc(crc, i2c_command);
    crc = basic_crc(crc, (i2c_addr << 1) + 1);
    for (i = 0; i < len; i++) {
        crc = basic_crc(crc, *data++);
    }
    return crc;
}

static s32 p6_i2c_smbus_read_i2c_block_data(const struct i2c_client *client, u8 command, u8 length, u8 *values)
{
    int retry_count = 10;
    s32 status = 0;
    u8 crc_calc = 0;

    while (retry_count--) {
        status = i2c_smbus_read_i2c_block_data(client, command, length, values);
        if (status >= 0) {
            crc_calc = calc_crc(client->addr, command, values, length - 1);
            if (crc_calc == *(values + length - 1)) break;
            printk("p6_i2c_smbus_read_i2c_block_data: crc checking error, command = 0x%x, crc_read = 0x%02x, crc_calc = 0x%02x\n",
                   command, *(values + length - 1), crc_calc);
        }
        msleep(1);
    }

    if (status < 0)
		return status;

    return length;
}

static s32 p6_i2c_smbus_read_word_data(const struct i2c_client *client, u8 command)
{
    int retry_count = 10;
    s32 status = 0;
    u8 crc_calc = 0;

    while (retry_count--) {
        status = i2c_smbus_read_word_data(client, command);
        if (status >= 0) {
            crc_calc = calc_crc(client->addr, command, (unsigned char *)&status, 1);
            if (crc_calc == ((status >> 8) & 0xff)) break;
            printk("p6_i2c_smbus_read_word_data: crc checking error, command = 0x%x, crc_read = 0x%02x, crc_calc = 0x%02x\n",
                   command, (status >> 8) & 0xff, crc_calc);
        }
        msleep(1);
    }
    return status;
}

/*
 * Management functions
 */
static ssize_t show_list_all(struct device *dev,
                             struct device_attribute *devattr, char *buf)
{
	struct p6_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
    ssize_t c = PAGE_SIZE;
    u32 reg, d0, d1;

    mutex_lock(&data->update_mutex);

    c -= snprintf(buf + PAGE_SIZE - c, c, "Client Address     = 0x%02x\n", client->addr);

    reg = p6_i2c_smbus_read_word_data(client, P6_CMD_READ_VERSION);
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power Version      = 0x%02x\n", reg & 0xff);

    reg = p6_i2c_smbus_read_word_data(client, P6_CMD_READ_STATUS);
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power Status       = 0x%02x (%s)\n", reg & 0xff, (reg & 0x1) ? "on" : "off");

    reg = p6_i2c_smbus_read_word_data(client, P6_CMD_READ_ERRORCODE);
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power ErrorCode    = 0x%02x\n", reg & 0xff);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_READ_VOUT_12V, 3, (u8 *)&reg);
    d0 = (reg & 0xffff) / 100;
    d1 = (reg & 0xffff) - d0 * 100;
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power VOUT_12V     = 0x%04x (%d.%02d V)\n", reg & 0xffff, d0, d1);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_READ_VOUT, 3, (u8 *)&reg);
    d0 = (reg & 0xffff) / 100;
    d1 = (reg & 0xffff) - d0 * 100;
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power VOUT         = 0x%04x (%d.%02d V)\n", reg & 0xffff, d0, d1);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_SET_VOUT, 3, (u8 *)&reg);
    d0 = (reg & 0xffff) / 100;
    d1 = (reg & 0xffff) - d0 * 100;
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power VOUT_SET     = 0x%04x (%d.%02d V)\n", reg & 0xffff, d0, d1);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_READ_IOUT, 3, (u8 *)&reg);
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power IOUT         = 0x%04x (%d A)\n", reg & 0xffff, reg & 0xffff);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_READ_POUT, 3, (u8 *)&reg);
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power POUT         = 0x%04x (%d W)\n", reg & 0xffff, reg & 0xffff);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_READ_TEMP0, 3, (u8 *)&reg);
    d0 = (reg & 0xffff) / 10;
    d1 = (reg & 0xffff) - d0 * 10;
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power TEMP0        = 0x%04x (%d.%01d ^C)\n", reg & 0xffff, d0, d1);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_READ_TEMP1, 3, (u8 *)&reg);
    d0 = (reg & 0xffff) / 10;
    d1 = (reg & 0xffff) - d0 * 10;
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power TEMP1        = 0x%04x (%d.%01d ^C)\n", reg & 0xffff, d0, d1);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_READ_FAN_PWM_DUTY, 3, (u8 *)&reg);
    c -= snprintf(buf + PAGE_SIZE - c, c, "Power FAN_PWM_DUTY = 0x%04x (%d%%)\n", reg & 0xff, reg & 0xff);

    mutex_unlock(&data->update_mutex);

	return PAGE_SIZE - c;
}

static ssize_t show_version(struct device *dev,
                            struct device_attribute *devattr, char *buf)
{
	struct p6_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
    ssize_t c = PAGE_SIZE;
    u32 reg;

    mutex_lock(&data->update_mutex);

    reg = p6_i2c_smbus_read_word_data(client, P6_CMD_READ_VERSION);
    c -= snprintf(buf + PAGE_SIZE - c, c, "%02x\n", reg & 0xff);

    mutex_unlock(&data->update_mutex);

	return PAGE_SIZE - c;
}

static ssize_t show_on_off(struct device *dev, struct device_attribute *da,
                           char *buf)
{
	struct p6_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
    ssize_t c = PAGE_SIZE;
    u32 reg;

    mutex_lock(&data->update_mutex);

    reg = p6_i2c_smbus_read_word_data(client, P6_CMD_READ_STATUS);
    c -= snprintf(buf + PAGE_SIZE - c, c, "%s\n", (reg & 0x1) ? "on" : "off");

    mutex_unlock(&data->update_mutex);

	return PAGE_SIZE - c;
}

static ssize_t store_on_off(struct device *dev, struct device_attribute *da,
                            const char *buf, size_t count)
{
	struct p6_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
    unsigned long val;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

    mutex_lock(&data->update_mutex);
    i2c_smbus_write_byte_data(client, P6_CMD_WRITE_ON_OFF, val & 0x1);
    mutex_unlock(&data->update_mutex);

	return count;
}

static ssize_t show_vout(struct device *dev, struct device_attribute *da,
                         char *buf)
{
	struct p6_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
    ssize_t c = PAGE_SIZE;
    u32 reg = 0;

    mutex_lock(&data->update_mutex);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_READ_VOUT, 3, (u8 *)&reg);
    c -= snprintf(buf + PAGE_SIZE - c, c, "%d\n", reg & 0xffff);

    mutex_unlock(&data->update_mutex);

	return PAGE_SIZE - c;;
}

static ssize_t store_vout(struct device *dev, struct device_attribute *da,
                        const char *buf, size_t count)
{
	struct p6_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
    unsigned long val;

	if (kstrtol(buf, 10, &val) < 0)
		return -EINVAL;

    mutex_lock(&data->update_mutex);
    i2c_smbus_write_word_data(client, P6_CMD_SET_VOUT, val & 0xffff);
    mutex_unlock(&data->update_mutex);

	return count;
}

static ssize_t show_vout_set(struct device *dev, struct device_attribute *da,
                             char *buf)
{
	struct p6_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
    ssize_t c = PAGE_SIZE;
    u32 reg = 0;

    mutex_lock(&data->update_mutex);

    p6_i2c_smbus_read_i2c_block_data(client, P6_CMD_SET_VOUT, 3, (u8 *)&reg);
    c -= snprintf(buf + PAGE_SIZE - c, c, "%d\n", reg & 0xffff);

    mutex_unlock(&data->update_mutex);

	return PAGE_SIZE - c;;
}

static ssize_t show_clear_fault(struct device *dev, struct device_attribute *da,
                                char *buf)
{
	return 0;
}

static ssize_t store_clear_fault(struct device *dev, struct device_attribute *da,
                                 const char *buf, size_t count)
{
	struct p6_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

    mutex_lock(&data->update_mutex);
    i2c_smbus_write_byte(client, P6_CMD_CLEAR_FAULT);
    mutex_unlock(&data->update_mutex);

	return count;
}

static DEVICE_ATTR(list_all, S_IRUGO, show_list_all, NULL);
static DEVICE_ATTR(version, S_IRUGO, show_version, NULL);
static DEVICE_ATTR(on_off, S_IWUSR | S_IRUGO, show_on_off, store_on_off);
static DEVICE_ATTR(vout, S_IWUSR | S_IRUGO, show_vout, store_vout);
static DEVICE_ATTR(vout_set, S_IRUGO, show_vout_set, NULL);
static DEVICE_ATTR(clear_fault, S_IWUSR | S_IRUGO, show_clear_fault, store_clear_fault);

static struct attribute *p6_attributes[] = {
	&dev_attr_list_all.attr,
	&dev_attr_version.attr,
	&dev_attr_on_off.attr,
	&dev_attr_vout.attr,
	&dev_attr_vout_set.attr,
	&dev_attr_clear_fault.attr,
	NULL
};

static const struct attribute_group p6_attr_group = {
	.attrs = p6_attributes,
};

/*
 * Initialization function
 */

static int p6_init_client(struct i2c_client *client)
{
    s32 status;

    status = p6_i2c_smbus_read_word_data(client, P6_CMD_READ_VERSION);
    if (status < 0) {
        printk("p6_init_client failed status = 0x%x\n", status);
        return -1;
    }

	return 0;
}

/*
 * I2C init/probing/exit functions
 */
static int p6_power_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct p6_data *data;
	int err = 0;

	data = kzalloc(sizeof(struct p6_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}
	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->update_mutex);

	err = p6_init_client(client);
	if (err)
		goto exit_kfree;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &p6_attr_group);
	if (err)
		goto exit_kfree;

    dev_info(&client->dev, "Power supply %s probed.\n", client->name);

	return 0;

exit_kfree:
	kfree(data);
	return err;
}

static int p6_power_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &p6_attr_group);

	kfree(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id p6_power_id[] = {
	{ "p6-power", 0x22 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, p6_power_id);

static struct i2c_driver p6_power_driver = {
	.driver = {
		.name	= "p6-power",
		.owner	= THIS_MODULE,
	},
	.probe	= p6_power_probe,
	.remove	= p6_power_remove,
	.id_table = p6_power_id,
};

module_i2c_driver(p6_power_driver);

MODULE_AUTHOR("Peter Wei <peter@microbt.com>");
MODULE_DESCRIPTION("P6 power supply driver");
MODULE_LICENSE("GPL");
