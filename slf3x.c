// SPDX-License-Identifier: GPL-2.0
/*
 * slf3x.c - Support for Sensirion SLF3x liquid flow sensors
 *
 * Copyright (C) 2021 Jonatan Midtgaard <jonatan.midtgaard@gmail.com>
 *
 * I2C slave address: 0x08
 *
 * Datasheets:
 * https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/4_Liquid_Flow_Meters/Sensirion_Liquid_Flow_Sensors_SLF3S-0600F_Datasheet.pdf
 * https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/4_Liquid_Flow_Meters/Liquid_Flow/Sensirion_Liquid_Flow_Meters_SLF3S-1300F_Datasheet_EN_D1.pdf
 *
 * TODO:
 * - Support 0600F device type
 * - Check Device type at boot
 * - Support IPA measurement
 * - crc8 check
 * - air-in-line, etc. flags
 */

#include <linux/crc8.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/sysfs.h>
#include <linux/delay.h>

#define SLF3X_WORD_LEN 2
#define SLF3X_CRC8_POLYNOMIAL 0x31
#define SLF3X_CRC8_INIT 0xff
#define SLF3X_CRC8_LEN 1

// Divide raw measurements with these to get physical values
#define SLF3S_1300F_SCALE 500 // (ml/min)^-1
#define SLF3S_0600F_SCALE 10 // (ul/min)^-1
#define SLF3X_TEMP_SCALE 200 // (deg C)^-1
#define ML_TO_UL 1000

// Commands
static const char SLF3X_START_H2O[] = { 0x36, 0x08 };
static const char SLF3X_START_IPA[] = { 0x36, 0x15 };
static const char SLF3X_STOP[] = { 0x3F, 0xF9 };

enum slf3x_product_id {
	SLF3S_1300F = 0,
	SLF3S_0600F = 1,
};

enum slf3x_state {
	IDLE = 0,
	BUSY = 1,
	H2O = 2,
	IPA = 3,
};

struct slf3x_crc_word {
	s16 value;
	u8 crc8;
} __attribute__((__packed__));

union slf3x_reading {
	u8 start;
	struct slf3x_crc_word raw_words[3];
};

struct slf3x_data {
	struct i2c_client *client;
	union slf3x_reading buffer;
	enum slf3x_product_id product_id;
	enum slf3x_state state;
};

// Device and channels
struct slf3x_device {
	const struct iio_chan_spec *channels;
	int num_channels;
};

static const struct iio_chan_spec slf3x_channels[] = {
	{
		.type = IIO_MASSFLOW,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 24,
			.shift = 8,
			.endianness = IIO_BE,
		},
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 24,
			.shift = 8,
			.endianness = IIO_BE,
		},
	},
	{
		.type = IIO_MISCFLAGS,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 2,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 24,
			.shift = 8,
			.endianness = IIO_BE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct slf3x_device slf3x_devices[] = {
	[SLF3S_1300F] = {
		.channels = slf3x_channels,
		.num_channels = ARRAY_SIZE(slf3x_channels),
	},
	[SLF3S_0600F] = {
		.channels = slf3x_channels,
		.num_channels = ARRAY_SIZE(slf3x_channels),
	},
};

/**
 * slf3x_verify_buffer() - verify the checksums of the data buffer words
 *
 * @data:       SLF3x data
 * @buf:        Raw data buffer
 * @word_count: Num data words stored in the buffer, excluding CRC bytes
 *
 * Return:      0 on success, negative error otherwise.
 */

DECLARE_CRC8_TABLE(slf3x_crc8_table);
static int slf3x_verify_buffer(const struct slf3x_data *data,
			       union slf3x_reading *buf, size_t word_count)
{
	size_t size = word_count * (SLF3X_WORD_LEN + SLF3X_CRC8_LEN);
	int i;
	u8 crc;
	u8 *data_buf = &buf->start;

	for (i = 0; i < size; i += SLF3X_WORD_LEN + SLF3X_CRC8_LEN) {
		crc = crc8(slf3x_crc8_table, &data_buf[i], SLF3X_WORD_LEN,
			   SLF3X_CRC8_INIT);
		if (crc != data_buf[i + SLF3X_WORD_LEN]) {
			dev_err(&data->client->dev, "CRC error\n");
			return -EIO;
		}
	}

	return 0;
}

static int slf3x_buffer_postenable(struct iio_dev *indio_dev)
{
	struct slf3x_data *data = iio_priv(indio_dev);
	int ret;

	ret = iio_triggered_buffer_postenable(indio_dev);
	return ret;
}

static int slf3x_buffer_predisable(struct iio_dev *indio_dev)
{
	struct slf3x_data *data = iio_priv(indio_dev);
	int ret;

	ret = iio_triggered_buffer_predisable(indio_dev);
	return ret;
}

static const struct iio_buffer_setup_ops slf3x_buffer_setup_ops = {
	.postenable = slf3x_buffer_postenable,
	.predisable = slf3x_buffer_predisable,
};

static int slf3x_do_meas(struct slf3x_data *data) {
	// Get all data from sensor.
	int ret;
	// TODO: Not all is necessary. For most reads, we can end early
	size_t num_words = 3;
	size_t size = num_words * (SLF3X_WORD_LEN + SLF3X_CRC8_LEN);
	u8 *data_buf = &data->buffer.start;
	ret = i2c_master_recv(data->client, data_buf, size);
	// TODO: crc check
	// i2c_master_recv returns negative error codes
	return ret < 0;
}

static irqreturn_t slf3x_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct slf3x_data *data = iio_priv(indio_dev);
	int ret;

	ret = slf3x_do_meas(data);
	if (ret)
		goto err;

	iio_push_to_buffers_with_timestamp(indio_dev, data->buffer.raw_words,
					   iio_get_time_ns(indio_dev));
err:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int slf3x_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan, int *val, int *val2,
			  long mask)
{
	struct slf3x_data *data = iio_priv(indio_dev);
	struct slf3x_crc_word *words;
	int ret;
	// Needed to cast into signed form
	u16 uval;
	s16 *sval;

	// Get all data from sensor.
	slf3x_do_meas(data);
	words = data->buffer.raw_words;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_MASSFLOW:
			uval = be16_to_cpu(words[0].value);
			sval = &uval;
			*val = *sval;
			ret = IIO_VAL_INT;
			break;
		case IIO_TEMP:
			uval = be16_to_cpu(words[1].value);
			sval = &uval;
			*val = *sval;
			ret = IIO_VAL_INT;
			break;
		case IIO_MISCFLAGS:
			*val = be16_to_cpu(words[2].value);
			ret = IIO_VAL_INT;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_MASSFLOW:
			// TODO: Do not hardcode this. Should depend on device.
			*val = ML_TO_UL;
			*val2 = SLF3S_1300F_SCALE;
			ret = IIO_VAL_FRACTIONAL;
			break;
		case IIO_TEMP:
			*val = 1;
			*val2 = SLF3X_TEMP_SCALE;
			ret = IIO_VAL_FRACTIONAL;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int slf3x_stop_meas(struct slf3x_data *data)
{
	int ret = i2c_master_send(data->client, SLF3X_STOP, SLF3X_WORD_LEN);
	if (ret != SLF3X_WORD_LEN)
		return ret < 0 ? ret : -EIO;

	data->state = IDLE;
	return 0;
}

static int slf3x_start_meas(struct slf3x_data *data, enum slf3x_state state)
{
	int ret;
	switch (state) {
	case H2O:
		ret = i2c_master_send(data->client, SLF3X_START_H2O,
				      SLF3X_WORD_LEN);
		break;
	case IPA:
		ret = i2c_master_send(data->client, SLF3X_START_IPA,
				      SLF3X_WORD_LEN);
		break;
	default:
		return 0;
	}
	if (ret != SLF3X_WORD_LEN)
		return ret < 0 ? ret : -EIO;

	data->state = state;
	return 0;
}

static const struct iio_info slf3x_info = {
	.read_raw = slf3x_read_raw,
};

static const struct of_device_id slf3x_dt_ids[] = {
	{ .compatible = "sensirion,slf3s-1300f", .data = (void *)SLF3S_1300F },
	{ .compatible = "sensirion,slf3s-0600f", .data = (void *)SLF3S_0600F },
	{}
};

static int slf3x_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct slf3x_data *data;
	const struct of_device_id *of_id;
	int ret;
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	of_id = of_match_device(slf3x_dt_ids, &client->dev);

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	crc8_populate_msb(slf3x_crc8_table, SLF3X_CRC8_POLYNOMIAL);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &slf3x_info;
	indio_dev->name = id->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	// TODO: Detect product variant as in sgp30. Hardcoded for now.
	data->product_id = SLF3S_1300F;

	indio_dev->channels = slf3x_devices[data->product_id].channels;
	indio_dev->num_channels = slf3x_devices[data->product_id].num_channels;

	// Set up triggered buffer
	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev, NULL,
					      slf3x_trigger_handler,
					      &slf3x_buffer_setup_ops);
	if (ret) {
		dev_err(&client->dev, "cannot setup triggered buffer\n");
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register iio device\n");
		goto unregister_buffer;
	}

	// This is not a proper reset, but we prefer not to use the general call address.
	// It is needed to support a soft reboot
	data->state = IDLE;
	ret = slf3x_stop_meas(data);
	if (ret) {
		dev_err(&client->dev,
			"failed to reset sensor\n");
		goto unregister_buffer;
	}
	// Sleep for at least 0.5 msecs as per sensor specification
	usleep_range(500, 700);

	// TODO: do this later, with an option to choose either calibration
	data->state = BUSY;
	ret = slf3x_start_meas(data, H2O);
	if (ret) {
		dev_err(&client->dev,
			"failed to start continuous measurement\n");
		goto unregister_buffer;
	}

	return 0;

unregister_buffer:
	iio_triggered_buffer_cleanup(indio_dev);
	return ret;
}

static int slf3x_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);

	return 0;
}

static const struct i2c_device_id slf3x_id[] = { { "slf3s-1300f", SLF3S_1300F },
						 { "slf3s-0600f", SLF3S_0600F },
						 {} };
MODULE_DEVICE_TABLE(i2c, slf3x_id);
MODULE_DEVICE_TABLE(of, slf3x_dt_ids);

static struct i2c_driver slf3x_driver = {
	.driver = {
		.name = "slf3x",
		.of_match_table = of_match_ptr(slf3x_dt_ids),
	},
	.probe = slf3x_probe,
	.remove = slf3x_remove,
	.id_table = slf3x_id,
};
module_i2c_driver(slf3x_driver);

MODULE_AUTHOR("Jonatan Midtgaard <jonatan.midtgaard@gmail.com>");
MODULE_DESCRIPTION("Sensirion SLF3x Liquid Flow Sensors");
MODULE_LICENSE("GPL v2");
