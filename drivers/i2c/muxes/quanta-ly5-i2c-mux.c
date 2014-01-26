/*
 * <bsn.cl fy=2013 v=gpl>
 * 
 *        Copyright 2013, 2014 BigSwitch Networks, Inc.        
 * 
 * This program is free software; you can redistribute it
 * and/or modify it under  the terms ofthe GNU General Public License as
 * published by the Free Software Foundation;  either version 2 of the  License,
 * or (at your option) any later version.
 * 
 * 
 * </bsn.cl>
 *
 * An I2C multiplexer driver for the Quanta LY5
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/gpio.h>

#define QUANTA_LY5_I2C_MUX_CHANNEL_FIRST 1

#define QUANTA_LY5_I2C_MUX_NUM_CHANNELS 32
#define QUANTA_LY5_I2C_MUX_NUM_GPIOS 16

#define QUANTA_LY5_I2C_MUX_CMD_SET_CHANNEL 0
#define QUANTA_LY5_I2C_MUX_CMD_SET_GPIO 1

struct quanta_ly5_i2c_mux {
	struct i2c_client *client;
	struct i2c_adapter *chan_adap[QUANTA_LY5_I2C_MUX_NUM_CHANNELS];
	struct gpio_chip gpio_chip;
	u8 last_chan;
	u16 gpio_val;
};

static const struct i2c_device_id quanta_ly5_i2c_mux_id[] = {
	{"quanta_ly5_i2c_mux", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, quanta_ly5_i2c_mux_id);

static int quanta_ly5_i2c_mux_reg_write(struct i2c_adapter *adap,
				struct i2c_client *client,
				u8 command, u8 val)
{
	int ret = -ENODEV;

	if (adap->algo->master_xfer) {
		struct i2c_msg msg;
		char buf[2];

		msg.addr = client->addr;
		msg.flags = 0;
		msg.len = 2;
		buf[0] = command;
		buf[1] = val;
		msg.buf = buf;
		ret = adap->algo->master_xfer(adap, &msg, 1);
	} else {
		union i2c_smbus_data data;

		data.byte = val;
		ret = adap->algo->smbus_xfer(adap, client->addr,
					     client->flags,
					     I2C_SMBUS_WRITE,
					     command,
					     I2C_SMBUS_BYTE_DATA, &data);
	}

	return ret;
}

static void quanta_ly5_i2c_mux_gpio_set(struct gpio_chip *gc, unsigned offset,
				int val)
{
	/* TODO */
}

static int quanta_ly5_i2c_mux_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	/* TODO */
	return 0;
}

static int quanta_ly5_i2c_mux_select_chan(struct i2c_adapter *adap,
					void *client, u32 chan)
{
	struct quanta_ly5_i2c_mux *data = i2c_get_clientdata(client);
	int ret = 0;
	u32 c = QUANTA_LY5_I2C_MUX_CHANNEL_FIRST + chan;

	if (data->last_chan != c) {
		ret = quanta_ly5_i2c_mux_reg_write(
			adap, client,
			QUANTA_LY5_I2C_MUX_CMD_SET_CHANNEL, c);
		data->last_chan = c;
	}

	return ret;
}

static int quanta_ly5_i2c_mux_release_chan(struct i2c_adapter *adap,
					  void *client, u32 chan)
{
	struct quanta_ly5_i2c_mux *data = i2c_get_clientdata(client);
	int ret = 0;

	ret = quanta_ly5_i2c_mux_reg_write(
		adap, client,
		QUANTA_LY5_I2C_MUX_CMD_SET_CHANNEL, 0);
	data->last_chan = 0;

	return ret;
}

static struct gpio_chip quanta_ly5_i2c_mux_gpio_chip = {
	.label = "quanta_ly5_i2c_mux_gpio_chip",
	.owner = THIS_MODULE,
	.ngpio = QUANTA_LY5_I2C_MUX_NUM_GPIOS,
	.base = -1,
	.set = quanta_ly5_i2c_mux_gpio_set,
	.get = quanta_ly5_i2c_mux_gpio_get,
};

static int quanta_ly5_i2c_mux_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = client->adapter;
	int chan;
	struct quanta_ly5_i2c_mux *data;
	int ret = -ENODEV;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE_DATA))
		goto err;

	data = kzalloc(sizeof(struct quanta_ly5_i2c_mux), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err;
	}

	i2c_set_clientdata(client, data);
	data->client = client;

	if (i2c_smbus_write_byte(client, 0) < 0) {
		dev_warn(&client->dev, "probe failed\n");
		goto exit_free;
	}

	i2c_lock_adapter(adap);
	quanta_ly5_i2c_mux_release_chan(adap, client, 0);
	i2c_unlock_adapter(adap);

	for (chan = 0; chan < QUANTA_LY5_I2C_MUX_NUM_CHANNELS; chan++) {
		data->chan_adap[chan] =
			i2c_add_mux_adapter(adap, &client->dev, client,
					0, chan, 0,
					quanta_ly5_i2c_mux_select_chan,
					quanta_ly5_i2c_mux_release_chan);

		if (data->chan_adap[chan] == NULL) {
			ret = -ENODEV;
			dev_err(&client->dev,
				"failed to register multiplexed adapter %d\n",
				chan);
			goto adap_reg_failed;
		}
	}

	dev_info(&client->dev,
		"registered %d multiplexed buses for I2C mux %s\n",
		chan, client->name);

	data->gpio_chip = quanta_ly5_i2c_mux_gpio_chip;
	data->gpio_chip.dev = &client->dev;
	data->gpio_val = 0xffff;

	ret = gpiochip_add(&data->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "failed to register GPIOs\n");
		goto adap_reg_failed;
	}

	dev_info(&client->dev, "registered GPIOs for I2C mux %s\n",
		client->name);

	return 0;

adap_reg_failed:
	for (chan--; chan >= 0; chan--)
		i2c_del_mux_adapter(data->chan_adap[chan]);

exit_free:
	kfree(data);
err:
	return ret;
}

static int quanta_ly5_i2c_mux_remove(struct i2c_client *client)
{
	struct quanta_ly5_i2c_mux *data = i2c_get_clientdata(client);
	int chan, ret;

	for (chan = 0; chan < QUANTA_LY5_I2C_MUX_NUM_CHANNELS; chan++)
		if (data->chan_adap[chan]) {
			ret = i2c_del_mux_adapter(data->chan_adap[chan]);
			if (ret)
				return ret;
			data->chan_adap[chan] = NULL;
		}

	ret = gpiochip_remove(&data->gpio_chip);
	if (ret)
		return ret;

	kfree(data);
	return 0;
}

static struct i2c_driver quanta_ly5_i2c_mux_driver = {
	.driver = {
		   .name = "quanta_ly5_i2c_mux",
		   .owner = THIS_MODULE,
		   },
	.probe = quanta_ly5_i2c_mux_probe,
	.remove = quanta_ly5_i2c_mux_remove,
	.id_table = quanta_ly5_i2c_mux_id,
};

module_i2c_driver(quanta_ly5_i2c_mux_driver);

MODULE_AUTHOR("Big Switch Networks <support@bigswitch.com>");
MODULE_DESCRIPTION("Quanta LY5 I2C multiplexer driver");
MODULE_LICENSE("GPL");
