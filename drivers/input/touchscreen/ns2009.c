/*
 * Nsiway NS2009 touchscreen controller driver
 *
 * Copyright (C) 2017 Icenowy Zheng <icenowy@aosc.xyz>
 *
 * Some codes are from silead.c, which is
 *   Copyright (C) 2014-2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/input/touchscreen.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/delay.h>

/* polling interval in ms */
#define POLL_INTERVAL	30

/* this driver uses 12-bit readout */
#define MAX_12BIT	0xfff

#define NS2009_TS_NAME	"ns2009_ts"

#define NS2009_READ_X_LOW_POWER_12BIT	0xc0
#define NS2009_READ_Y_LOW_POWER_12BIT	0xd0
#define NS2009_READ_Z1_LOW_POWER_12BIT	0xe0
#define NS2009_READ_Z2_LOW_POWER_12BIT	0xf0

#define NS2009_DEF_X_FUZZ	32
#define NS2009_DEF_Y_FUZZ	16

/*
 * The chip have some error in z1 value when pen is up, so the data read out
 * is sometimes not accurately 0.
 * This value is based on experiements.
 */
#define NS2009_PEN_UP_Z1_ERR	40

struct ns2009_data {
	struct i2c_client		*client;
	struct input_dev		*input;

	struct touchscreen_properties	prop;

	bool				pen_down;
};

static int ns2009_ts_read_data(struct ns2009_data *data, u8 cmd, u16 *val)
{
	u8 raw_data[2];
	int error;

	error = i2c_smbus_read_i2c_block_data(data->client, cmd, 2, raw_data);
	if (error < 0)
	{
		printk("error = %d\n", error);
		return error;
	}

	if (unlikely(raw_data[1] & 0xf))
	{
		printk("xxxxxxxxxxx\n");
		return -EINVAL;
	}

	*val = (raw_data[0] << 4) | (raw_data[1] >> 4);
	if(NS2009_READ_Z1_LOW_POWER_12BIT == cmd)
	{
		//printk("Z = %d", *val);
	}
	else if(NS2009_READ_X_LOW_POWER_12BIT == cmd)
	{
		//printk("	X = %d", *val);
	}
	else if(NS2009_READ_Y_LOW_POWER_12BIT == cmd)
	{
		//printk("		Y = %d", *val);
	}

	return 0;
}

void swap1(u16 *xp, u16 *yp)
{
    u16 temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// Two-pass algorithm:
//   http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Two-pass_algorithm
u16 variance(u16 v[], size_t n)
{
    uint32_t mean = 0;
    uint32_t ssd = 0;

	int i;
    for (i = 0; i < n; i++) {
        mean += v[i];
    }
    mean /= n;

    for (i = 0; i < n; i++) {
        ssd += (v[i] - mean) * (v[i] - mean);
    }
    return (u16)(ssd / (n));
}

// A function to implement bubble sort
void bubbleSort(u16 arr[], int n)
{
   int i, j;
   for (i = 0; i < n-1; i++)      

       // Last i elements are already in place   
       for (j = 0; j < n-i-1; j++) 
           if (arr[j] > arr[j+1])
              swap1(&arr[j], &arr[j+1]);
}

static unsigned long last_jiffies = 0;
static u16 last_x, last_y, distance;
#define TOUCH_COUNT_TOTAL    6
#define TOUCH_COUNT_VALID    4
static int ns2009_ts_report(struct ns2009_data *data)
{
	u16 x, y, z1[9], x1[TOUCH_COUNT_TOTAL], y1[TOUCH_COUNT_TOTAL], i, count, count2, x_sum, y_sum;
	int ret;
	u16 var_x=0, var_y =0;
	/*
	 * NS2009 chip supports pressure measurement, but currently it needs
	 * more investigation, so we only use z1 axis to detect pen down
	 * here.
	 */
	for(i=0;i<sizeof(z1)/sizeof(z1[0]);i++)
	{
		ret = ns2009_ts_read_data(data, NS2009_READ_Z1_LOW_POWER_12BIT, &z1[i]);
		if (ret)
			return ret;
	}


	if ((z1[0] >= NS2009_PEN_UP_Z1_ERR) && (z1[1] >= NS2009_PEN_UP_Z1_ERR) && (z1[2] >= NS2009_PEN_UP_Z1_ERR)
		&&(z1[3] >= NS2009_PEN_UP_Z1_ERR) && (z1[4] >= NS2009_PEN_UP_Z1_ERR) && (z1[5] >= NS2009_PEN_UP_Z1_ERR)
		&&(z1[6] >= NS2009_PEN_UP_Z1_ERR) && (z1[7] >= NS2009_PEN_UP_Z1_ERR) && (z1[8] >= NS2009_PEN_UP_Z1_ERR)
	)
	{
		count = 0;
		do
		{
			ns2009_ts_read_data(data, NS2009_READ_X_LOW_POWER_12BIT, &x1[count]);
			ns2009_ts_read_data(data, NS2009_READ_Y_LOW_POWER_12BIT, &y1[count]);

			if ((x1[count] > 3900) || (y1[count] > 3900))
			{
				printk(" > 3900 break");
				//return -3;
				break;
			}

			if((x1[count] < 80) || (y1[count] < 80))
			{
				printk(" < 80 break");
				break;
			}



			distance = int_sqrt((last_x-x1[count])*(last_x-x1[count]) + (last_y-y1[count])*(last_y-y1[count]));
			//printk("count=%d,x=%d,y=%d,last_j=%lu,current_j=%lu,diff=%lu,lastx=%d,lasty=%d, distance=%d", count, x, y, last_jiffies, jiffies, jiffies-last_jiffies, last_x, last_y, distance);
			if((jiffies-last_jiffies <= 30) && (distance > 800))
			{
				printk("feidian 3: distance=%d", distance);
				return -5;
			}



			count++;
		}
		while(count < TOUCH_COUNT_TOTAL);

		if(count < TOUCH_COUNT_VALID)
		{
			printk("count < %d", TOUCH_COUNT_VALID);
			return -1;
		}

		var_x = variance(x1, count);
		var_y = variance(y1, count);

		printk("var_x=%d, var_y=%d", var_x, var_y);

		if((var_x > 800) || (var_y > 800))
		{
			printk("!!!!!!!!!error var report!!!!!!! var_x=%d, var_y=%d", var_x, var_y);
			printk("!!!!!!!!!error var report!!!!!!! var_x=%d, var_y=%d", var_x, var_y);
			printk("!!!!!!!!!error var report!!!!!!! var_x=%d, var_y=%d", var_x, var_y);
			return -1;
		}

		
		bubbleSort(x1, count);
		bubbleSort(y1, count);
		x_sum = 0;
		y_sum = 0;
		for(i=0;i<count;i++)
		{
			printk("\t\t%d: x=%d, y=%d", i, x1[i], y1[i]);
		}

		count2 = 0;
		for(i=1;i<count-1;i++)
		{
			x_sum += x1[i];
			y_sum += y1[i];
			count2++;
			printk("\t\t\t\t%d: x=%d, y=%d", i, x1[i], y1[i]);
		}

		x = x_sum / count2;
		y = y_sum / count2;

		if ((x > 4000) || (y > 4000) || (count <= 0))
			return -4;

		distance = int_sqrt((last_x-x)*(last_x-x) + (last_y-y)*(last_y-y));
		printk("count=%d,x=%d,y=%d,last_j=%lu,current_j=%lu,diff=%lu,lastx=%d,lasty=%d, distance=%d,z1=%d,%d,%d", count, x, y, last_jiffies, jiffies, jiffies-last_jiffies, last_x, last_y, distance, z1[0], z1[1], z1[2]);
#if 0
		if((jiffies-last_jiffies <= 20) && (distance > 700))
		{
			printk("feidian 1");
			return -5;
		}
		else if((jiffies-last_jiffies <= 100) && (distance > 1500))
		{
			printk("feidian 2");
			return -5;
		}
#endif
		if (!data->pen_down) {
			printk("pen down!");
			input_report_key(data->input, BTN_TOUCH, 1);
			data->pen_down = true;
		}

		last_jiffies = jiffies;
		last_x = x;
		last_y = y;
		printk("report end.");
		input_report_abs(data->input, ABS_X, x);
		input_report_abs(data->input, ABS_Y, y);
		input_sync(data->input);
	} else if (data->pen_down) {
		input_report_key(data->input, BTN_TOUCH, 0);
		input_sync(data->input);
		data->pen_down = false;
	}
	return 0;
}

static void ns2009_ts_poll(struct input_polled_dev *dev)
{
	struct ns2009_data *data = dev->private;
	int ret;
	static int error = 0;

	if(error == 1)
	{
		mdelay(10);
		error = 0;
	}

	ret = ns2009_ts_report(data);
	if (ret == -1)
	{
		dev_err(&dev->input->dev, "Poll touch data failed: %d\n", ret);
		mdelay(10);
	}
	else if (ret == -5) 
	{
		printk("ret == -5");
		input_report_key(data->input, BTN_TOUCH, 0);
		input_sync(data->input);
		data->pen_down = false;
		//mdelay(100);
		error = 1;
	}
	else if (ret == -4) 
	{
		printk("ret == -4");
		input_report_key(data->input, BTN_TOUCH, 0);
		input_sync(data->input);
		data->pen_down = false;
		//mdelay(100);
		error = 1;
	}
	else if (ret == -3) 
	{
		printk("ret == -3");
		input_report_key(data->input, BTN_TOUCH, 0);
		input_sync(data->input);
		data->pen_down = false;
		//mdelay(100);
		error = 1;
	}
}

static void ns2009_ts_config_input_dev(struct ns2009_data *data)
{
	struct input_dev *input = data->input;

	input_set_abs_params(input, ABS_X, 0, MAX_12BIT, NS2009_DEF_X_FUZZ, 0);
	input_set_abs_params(input, ABS_Y, 0, MAX_12BIT, NS2009_DEF_Y_FUZZ, 0);
	touchscreen_parse_properties(input, false, &data->prop);

	input->name = NS2009_TS_NAME;
	input->phys = "input/ts";
	input->id.bustype = BUS_I2C;
	input_set_capability(input, EV_KEY, BTN_TOUCH);
}

static int ns2009_ts_request_polled_input_dev(struct ns2009_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_polled_dev *polled_dev;
	int error;

	polled_dev = devm_input_allocate_polled_device(dev);
	if (!polled_dev) {
		dev_err(dev,
			"Failed to allocate polled input device\n");
		return -ENOMEM;
	}
	data->input = polled_dev->input;

	ns2009_ts_config_input_dev(data);
	polled_dev->private = data;
	polled_dev->poll = ns2009_ts_poll;
	polled_dev->poll_interval = POLL_INTERVAL;

	error = input_register_polled_device(polled_dev);
	if (error) {
		dev_err(dev, "Failed to register polled input device: %d\n",
			error);
		return error;
	}

	return 0;
}

static int ns2009_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ns2009_data *data;
	struct device *dev = &client->dev;
	int error;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK |
				     I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		dev_err(dev, "I2C functionality check failed\n");
		return -ENXIO;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;

	error = ns2009_ts_request_polled_input_dev(data);
	if (error)
		return error;

	return 0;
};

static const struct i2c_device_id ns2009_ts_id[] = {
	{ "ns2009", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ns2009_ts_id);

static struct i2c_driver ns2009_ts_driver = {
	.probe = ns2009_ts_probe,
	.id_table = ns2009_ts_id,
	.driver = {
		.name = NS2009_TS_NAME,
	},
};
module_i2c_driver(ns2009_ts_driver);
