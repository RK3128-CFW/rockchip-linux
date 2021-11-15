// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *	Driver for WX8 joysticks
 *
 *	Authors:
 *	Martin Cerveny		  <M.Cerveny@computer.org>
 */
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/input-event-codes.h>

#define DRIVER_DESC "WX8 joysticks"
#define MODULE_DEVICE_ALIAS "wx8-joysticks"

MODULE_AUTHOR("Martin Cerveny <M.Cerveny@computer.org>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/* i2c data offsets */
#define WX8JOY_LV	9
#define WX8JOY_LH	7
#define WX8JOY_RV	1
#define WX8JOY_RH	3

/* i2c joystick ADC data */
#define WX8JOY_MAX_AXIS	225
#define WX8JOY_MIN_AXIS	95
#define WX8JOY_FUZZ	2
#define WX8JOY_FLAT	4

/* joystick keys emulation */
#define WX8JOY_MIDDLE	165
#define WX8JOY_RESET	15	// +- from MIDDLE
#define WX8JOY_SET	30	// +- from MIDDLE

/* timer, 200Hz */
#define WX8JOY_INTERVAL 5

struct wx8joy_device {
	struct input_dev *input_dev;
	struct i2c_client *i2c_client;
	struct workqueue_struct *wq;
	struct delayed_work dw;

	int key_emulation;	// switch between analog joysticks (default) and key emulation
	u32 axes[4];		// joystick axes mapping
	int axes_size;
	u32 keys[8];		// joystick keys emulation mapping
	int keys_size;
};

/* main periodic worker */
static void wx8joy_work(struct work_struct *work)
{
	struct wx8joy_device *wx8joy = container_of(work, struct wx8joy_device, dw.work);
	u8 buf[16];
	int ret;

	// TODO: multiple scan, majority vote
	// TODO: Scale 0-255 ? With truncate ? Calibrate ?

	ret = i2c_master_recv(wx8joy->i2c_client, buf, sizeof(buf));
	if (ret == sizeof(buf)) {
		u8 idx;
		u8 vals[] = {buf[WX8JOY_RH], buf[WX8JOY_RV], buf[WX8JOY_LH], buf[WX8JOY_LV]};

		for(idx=0; idx<wx8joy->axes_size; idx++) {
			if (!wx8joy->key_emulation)
				input_report_abs(wx8joy->input_dev, wx8joy->axes[idx], vals[idx]);
			else {
				if (vals[idx]<WX8JOY_MIDDLE-WX8JOY_SET)
					input_event(wx8joy->input_dev, EV_KEY, wx8joy->keys[idx*2+0], 1);
				else if (vals[idx]>WX8JOY_MIDDLE+WX8JOY_SET)
					input_event(wx8joy->input_dev, EV_KEY, wx8joy->keys[idx*2+1], 1);
				else if (vals[idx]>WX8JOY_MIDDLE-WX8JOY_RESET && vals[idx]<WX8JOY_MIDDLE+WX8JOY_RESET) {
					input_event(wx8joy->input_dev, EV_KEY, wx8joy->keys[idx*2+0], 0);
					input_event(wx8joy->input_dev, EV_KEY, wx8joy->keys[idx*2+1], 0);
				}
			}
		}
		input_sync(wx8joy->input_dev);
	}
	
	WARN_ON(!queue_delayed_work(wx8joy->wq, &wx8joy->dw, msecs_to_jiffies(WX8JOY_INTERVAL)));
}

/* sysfs mgmt */
static ssize_t wx8joy_store_key_type(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct wx8joy_device *wx8joy = platform_get_drvdata(pdev);
	int ret;
	int val;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	wx8joy->key_emulation = !!val;
	return count;
}

static ssize_t wx8joy_show_key_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct wx8joy_device *wx8joy = platform_get_drvdata(pdev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE - 1, "%d", wx8joy->key_emulation);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	return ret;
}

// switch between keys emulation and joysticks
static DEVICE_ATTR(key_emulation, S_IWUSR | S_IRUGO,
		   wx8joy_show_key_type,
		   wx8joy_store_key_type);

static struct attribute *wx8joy_attrs[] = {
	&dev_attr_key_emulation.attr,
	NULL,
};

static struct attribute_group wx8joy_attr_group = {
	.attrs = wx8joy_attrs,
};

/* device startup and shutdown */
static int wx8joy_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct device *dev = &client->dev;
	struct wx8joy_device *wx8joy;
	struct input_dev *input_dev;
	int ret, length, idx;

	wx8joy = kmalloc(sizeof(struct wx8joy_device), GFP_KERNEL);

	if (!of_find_property(np, "axes", &length) || length % sizeof(u32) != 0 || length>sizeof(wx8joy->axes)) {
		dev_err(dev, "bad axes!\n");
		return -EINVAL;
	}
	wx8joy->axes_size = length/sizeof(u32);
	ret = of_property_read_u32_array(np, "axes", wx8joy->axes, wx8joy->axes_size);
	if (ret < 0)
		return ret;

	if (!of_find_property(np, "keys", &length) || length % sizeof(u32) != 0 || length/sizeof(u32) != wx8joy->axes_size*2 || length>sizeof(wx8joy->keys)) {
		dev_err(dev, "bad keys!\n");
		return -EINVAL;
	}
	wx8joy->keys_size = length/sizeof(u32);
	ret = of_property_read_u32_array(np, "keys", wx8joy->keys, wx8joy->keys_size);
	if (ret < 0)
		return ret;

	input_dev = input_allocate_device();
	if (!wx8joy || !input_dev) {
		dev_err(dev, "Can't allocate memory for device structure\n");
		ret = -ENOMEM;
		goto err_free_mem;
	}
	wx8joy->i2c_client = client;
	wx8joy->input_dev = input_dev;
	wx8joy->key_emulation = 0;

	wx8joy->wq = create_singlethread_workqueue("wx8joy");
	if (!wx8joy->wq) {
		dev_err(dev, "Failed to create work queue\n");
		ret = -EFAULT;
		goto err_free_mem;
	}

	ret = sysfs_create_group(&dev->kobj, &wx8joy_attr_group);
	if (ret) {
		dev_err(dev, "Unable to create sysfs\n");
		goto err_wq;
	}

	input_dev->name = DRIVER_DESC;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_REP, input_dev->evbit);

	for(idx=0; idx<wx8joy->axes_size; idx++)
		input_set_abs_params(input_dev, wx8joy->axes[idx],
			WX8JOY_MIN_AXIS, WX8JOY_MAX_AXIS, WX8JOY_FUZZ, WX8JOY_FLAT);

	for(idx=0; idx<wx8joy->keys_size; idx++)
		input_set_capability(input_dev, EV_KEY, wx8joy->keys[idx]);

	ret = input_register_device(wx8joy->input_dev);
	if (ret) {
		dev_err(dev, "Failed to register input device\n");
		goto err_sysfs;
	}

	i2c_set_clientdata(client, wx8joy);

	INIT_DELAYED_WORK(&wx8joy->dw, wx8joy_work);
	WARN_ON(!queue_delayed_work(wx8joy->wq, &wx8joy->dw, msecs_to_jiffies(WX8JOY_INTERVAL)));

	return 0;

err_sysfs:
	sysfs_remove_group(&dev->kobj, &wx8joy_attr_group);
err_wq:
	destroy_workqueue(wx8joy->wq);
err_free_mem:
	input_free_device(input_dev);
	kfree(wx8joy);
	return ret;
}

static int wx8joy_remove(struct i2c_client *client)
{
	struct wx8joy_device *wx8joy = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &wx8joy_attr_group);
	destroy_workqueue(wx8joy->wq);
	input_unregister_device(wx8joy->input_dev);
	kfree(wx8joy);

	return 0;
}

/* device registration */
static const struct i2c_device_id wx8joy_id[] = {
	{ MODULE_DEVICE_ALIAS, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wx8joy_id);

static struct i2c_driver wx8joy_driver = {
	.driver = {
		.name = MODULE_DEVICE_ALIAS,
	},
	.probe		= wx8joy_probe,
	.remove		= wx8joy_remove,
	.id_table	= wx8joy_id,
};
module_i2c_driver(wx8joy_driver);
