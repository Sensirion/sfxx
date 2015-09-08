/* Sensirion SFxx flow and differential pressure sensor driver.
 * Currently, the SF04 and SF05 chips are supported. These chips are used
 * in several mass flow meters and differential pressure sensors.
 *
 * Copyright (C) 2015 Sensirion AG, Switzerland
 * Author: David Frey <david.frey@sensirion.com>
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
 */

#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "sfxx.h"

#define SFXX_CMD_LENGTH      2
#define SFXX_CRC8_POLYNOMIAL 0x31
#define SFXX_CRC8_INIT       0x00
#define SFXX_RESPONSE_LENGTH 3
#define SFXX_WORD_LENGTH     2
#define SFXX_ADDRESS_LENGTH  2
#define SFXX_SILICON_ID_MASK 0x1f

DECLARE_CRC8_TABLE(sfxx_crc8_table);

#define SFXX_NAME    "sfxx"
#define SF04_NAME    "sf04"
#define SF05_NAME    "sf05"
#define SDP6XX_NAME  "sdp6xx"
#define SFM3500_NAME "sfm3500"

enum DEVICE_ID {
	SFXX_ID,
	SF04_ID,
	SF05_ID,
	SDP6XX_ID,
	SFM3500_ID,
};

/* commands */
struct sfxx_commands {
	const u8 *measure;
	const u8 *read_eeprom;
	const u8 *read_id_reg;
	u8 length;

	u16 addr_scale_factor;
	u16 addr_offset;
	u16 addr_flow_unit;
	u8 silicon_id;
	unsigned int warmup_time_ms;
};

static const u8 sf04_command_measure[]     = { 0xf1 };
static const u8 sf04_command_read_eeprom[] = { 0xfa };
static const u8 sf04_command_read_id_reg[] = { 0xef };

static const struct sfxx_commands sf04_commands = {
	.measure     = sf04_command_measure,
	.read_eeprom = sf04_command_read_eeprom,
	.read_id_reg = sf04_command_read_id_reg,
	.length      = 1,

	.addr_scale_factor = 0x2b60,
	.addr_offset       = 0x2be0,
	.addr_flow_unit    = 0x2b70,
	.silicon_id  = 3,
	.warmup_time_ms = 0,
};

const u8 sf05_command_measure[]     = { 0x10, 0x00 };
const u8 sf05_command_read_id_reg[] = { 0x77, 0x00 };

static const struct sfxx_commands sf05_commands = {
	.measure     = sf05_command_measure,
	.read_eeprom = NULL,
	.read_id_reg = sf05_command_read_id_reg,
	.length      = 2,

	.addr_scale_factor = 0x30de,
	.addr_offset       = 0x30df,
	.addr_flow_unit    = 0x31e1,
	.silicon_id  = 5,
	.warmup_time_ms = 35,
};

struct sfxx_data {
	struct i2c_client *client;
	struct input_dev *dev;
	struct mutex update_lock;
	bool measurement_started;

	const struct sfxx_commands *commands;
	struct sfxx_platform_data setup;

	s32 measured_value;	/* 16/16 fixed point */
	s32 scale_factor;
	u16 offset;
	u16 unit;
};

static int sfxx_read_word(struct i2c_client *client, u16 *value)
{
	int ret;
	u8 crc;
	u8 buffer[SFXX_RESPONSE_LENGTH];

	ret = i2c_master_recv(client, buffer, SFXX_RESPONSE_LENGTH);
	if (ret != SFXX_RESPONSE_LENGTH) {
		dev_err(&client->dev, "failed to read values: %d", ret);
		return ret < 0 ? ret : -EIO;
	}
	crc = crc8(sfxx_crc8_table, buffer, SFXX_WORD_LENGTH, SFXX_CRC8_INIT);
	if (crc != buffer[2]) {
		dev_err(&client->dev, "crc received: %x, calculated: %x",
			(u32)buffer[2], (u32) crc);
		return -EIO;
	}
	*value = be16_to_cpup((__be16 *) buffer);
	return 0;
}

static int sfxx_read_from_command(struct i2c_client *client,
				  const u8 *command, u16 length,
				  u16 *value)
{
	int ret;

	ret = i2c_master_send(client, command, length);
	if (ret != length) {
		dev_err(&client->dev, "failed to send command: %d", ret);
		return ret < 0 ? ret : -EIO;
	}
	return sfxx_read_word(client, value);
}

static struct sfxx_data *sfxx_update_client(struct device *dev)
{
	struct sfxx_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	s16 raw_value;
	s32 value;
	int ret;

	mutex_lock(&data->update_lock);
	if (!data->measurement_started) {
		/* start the continuous measurement on the sensor */
		ret = i2c_master_send(client, data->commands->measure,
				      data->commands->length);
		if (ret != data->commands->length) {
			dev_err(&client->dev, "failed to start measurement: %d", ret);
			ret = ret < 0 ? ret : -EIO;
			goto out;
		}
		if (data->commands->warmup_time_ms > 0)
			msleep(data->commands->warmup_time_ms);
		data->measurement_started = true;
	}

	ret = sfxx_read_word(client, &raw_value);
	if (ret) {
		data->measurement_started = false;
		goto out;
	}

	/*
	 * From datasheet:
	 * F = (SF - offset) / scale_factor
	 *
	 * Adapted for integer fixed point arithmetic.
	 */
	value = (s32)((raw_value - data->offset) << 16) / data->scale_factor;
	data->measured_value = value;

out:
	mutex_unlock(&data->update_lock);

	return ret == 0 ? data : ERR_PTR(ret);
}

static int sfxx_read_eeprom_word(struct i2c_client *client,
				 const struct sfxx_commands *commands,
				 u16 address, u16 *value)
{
	u8 command[SFXX_CMD_LENGTH + SFXX_ADDRESS_LENGTH];
	size_t position = 0;
	if (commands->read_eeprom != NULL) {
		memcpy(command, commands->read_eeprom, commands->length);
		position += commands->length;
	}
	address = cpu_to_be16(address);
	memcpy(command + position, &address, sizeof(address));
	return sfxx_read_from_command(client, command,
				      position + sizeof(address),
				      value);
}

static ssize_t print_fixed_point(char *buffer, s32 value)
{
	char sign[] = "\0\0";
	if (value < 0) {
		value = -value;
		sign[0] = '-';
	}
	/* We separate the fixed point number into the decimal and the fraction
	 * part for printing.
	 */
	return scnprintf(buffer, PAGE_SIZE, "%s%d.%07u\n", sign,
			 value >> 16,
			 (value & 0xffff) * (10000000 / (1 << 16)));
}

static ssize_t measured_value_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buffer)
{
	struct sfxx_data *data = sfxx_update_client(dev);
	if (IS_ERR(data))
		return PTR_ERR(data);

	return print_fixed_point(buffer, data->measured_value);
}

static ssize_t scale_factor_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buffer)
{
	struct sfxx_data *data = dev_get_drvdata(dev);
	return scnprintf(buffer, PAGE_SIZE, "%d\n", data->scale_factor);
}

static ssize_t offset_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buffer)
{
	struct sfxx_data *data = dev_get_drvdata(dev);
	return scnprintf(buffer, PAGE_SIZE, "%u\n", (unsigned)data->offset);
}

static ssize_t unit_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buffer)
{
	struct sfxx_data *data = dev_get_drvdata(dev);
	u8 p1, p2, p3;
	/* the unit is encoded as three indexes into lists for prefix,
	 * unit and time base.
	 */
	const char *prefix[]= {
		"", "", "", "n", "u", "m", "c", "d", "", "10", "h", "k",
		"M", "G", "", ""
	};
	const char *base_unit[] = {
		"ln", "sl", "sl", "sl", "", "", "", "", "l", "g", "",
		"", "", "", "", "", "Pa", "bar", "mH2O", "inH2O",
		"", "", "", "", "", "", "", "", "", "", "", ""
	};
	const char *time_base[] = {
		"", "/us", "/ms", "/s", "/min", "/h", "/day", "", "", "", "",
		"", "", "", "", ""
	};
	p1 = data->unit & 0x0f;
	p2 = (data->unit >> 8) & 0x1f;
	p3 = (data->unit >> 4) & 0x0f;
	return scnprintf(buffer, PAGE_SIZE, "%s%s%s\n",
			 prefix[p1], base_unit[p2], time_base[p3]);
}

static DEVICE_ATTR_RO(measured_value);
static DEVICE_ATTR_RO(scale_factor);
static DEVICE_ATTR_RO(offset);
static DEVICE_ATTR_RO(unit);

static struct attribute *sfxx_attributes[] = {
	&dev_attr_measured_value.attr,
	&dev_attr_scale_factor.attr,
	&dev_attr_offset.attr,
	&dev_attr_unit.attr,
	NULL
};

static const struct attribute_group sfxx_attr_group = {
	.attrs = sfxx_attributes,
};

static int read_details_from_eeprom(struct i2c_client *client,
				    struct sfxx_data *data)
{
	int ret;
	u16 word;
	ret = sfxx_read_eeprom_word(client, data->commands,
				    data->commands->addr_scale_factor, &word);
	if (ret < 0) {
		dev_err(&client->dev, "could not read scale factor: %d", ret);
		return ret;
	}
	data->scale_factor = word;
	ret = sfxx_read_eeprom_word(client, data->commands,
				    data->commands->addr_offset, &word);
	if (ret < 0) {
		dev_err(&client->dev, "could not read offset: %d", ret);
		return ret;
	}
	data->offset = word;
	ret = sfxx_read_eeprom_word(client, data->commands,
				    data->commands->addr_flow_unit, &word);
	if (ret < 0) {
		dev_err(&client->dev, "could not read scale factor: %d", ret);
		return ret;
	}
	data->unit = word;
	return 0;
}

static int probe_for_sensor(struct i2c_client *client,
			    const struct sfxx_commands *commands)
{
	u16 word;
	int ret;
	ret = sfxx_read_from_command(client, commands->read_id_reg,
				     commands->length, &word);
	if (ret < 0) {
		dev_err(&client->dev, "could not read id register: %d", ret);
		return ret;
	}
	if ((word & SFXX_SILICON_ID_MASK) != commands->silicon_id) {
		dev_err(&client->dev, "connected device is not sfxx");
		return -ENODEV;
	}
	return 0;
}

static int sfxx_probe(struct i2c_client *client,
		      const struct i2c_device_id *id)
{
	int ret;
	struct sfxx_data *data;
	struct i2c_adapter *adap = client->adapter;
	struct device *dev = &client->dev;
	enum DEVICE_ID device_id = (enum DEVICE_ID)id->driver_data;
	const struct sfxx_commands *commands;

	if (!i2c_check_functionality(adap, I2C_FUNC_I2C)) {
		dev_err(dev, "plain i2c transactions not supported");
		return -ENODEV;
	}
	/* has to be done before the first I2C communication */
	crc8_populate_msb(sfxx_crc8_table, SFXX_CRC8_POLYNOMIAL);

	switch (device_id) {
		case SFXX_ID:
			dev_info(dev, "probing multiple chips, ignore error messages on success");
			commands = &sf04_commands;
			ret = probe_for_sensor(client, commands);
			if (ret == 0)
				break;
			commands = &sf05_commands;
			ret = probe_for_sensor(client, commands);
			break;
		case SF04_ID:
		case SDP6XX_ID:
			commands = &sf04_commands;
			ret = probe_for_sensor(client, commands);
			break;
		case SF05_ID:
		case SFM3500_ID:
			commands = &sf05_commands;
			ret = probe_for_sensor(client, commands);
			break;
		default:
			dev_err(dev, "unknown device");
			return -ENODEV;
	}
	if (ret < 0) {
		dev_err(dev, "probing for %s failed", id->name);
		return ret;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->client = client;
	data->commands = commands;

	ret = read_details_from_eeprom(client, data);
	if (ret < 0)
		return ret;

	if (client->dev.platform_data)
		data->setup = *(struct sfxx_platform_data *)dev->platform_data;
	mutex_init(&data->update_lock);

	i2c_set_clientdata(client, data);

	ret = sysfs_create_group(&client->dev.kobj, &sfxx_attr_group);
	if (ret) {
		dev_err(&client->dev, "Could not create sysfs group");
		return ret;
	}

	data->dev = input_allocate_device();
	if (!data->dev || IS_ERR(data->dev)) {
		dev_err(&client->dev, "Unable to allocate device data");
		ret = PTR_ERR(data->dev);
		goto fail_remove_sysfs;
	}
	ret = input_register_device(data->dev);
	if (ret) {
		dev_err(&client->dev, "Unable to register device data");
		goto fail_remove_sysfs;
	}

	return 0;

fail_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &sfxx_attr_group);
	i2c_set_clientdata(client, NULL);
	return ret;
}

static int __exit sfxx_remove(struct i2c_client *client)
{
	struct sfxx_data *data = i2c_get_clientdata(client);
	if (data != NULL) {
		input_unregister_device(data->dev);
		sysfs_remove_group(&client->dev.kobj, &sfxx_attr_group);
	}
	return 0;
}

/* device ID table */
static const struct i2c_device_id sfxx_id[] = {
	{SFXX_NAME, SFXX_ID},
	{SF04_NAME, SF04_ID},
	{SF05_NAME, SF05_ID},
	{SDP6XX_NAME, SDP6XX_ID},
	{SFM3500_NAME, SFM3500_ID},
	{}
};

MODULE_DEVICE_TABLE(i2c, sfxx_id);

static struct i2c_driver sfxx_i2c_driver = {
	.driver.name = "sfxx",
	.probe       = sfxx_probe,
	.remove      = __exit_p(sfxx_remove),
	.id_table    = sfxx_id,
};

module_i2c_driver(sfxx_i2c_driver);

MODULE_AUTHOR("David Frey <david.frey@sensirion.com>");
MODULE_DESCRIPTION("Sensirion SFxx mass flow and differential pressure sensor driver");
MODULE_LICENSE("GPL");
