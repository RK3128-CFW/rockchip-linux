// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *	WX8 power management and charger driver
 *
 *	- charger is independent Li-Ion linear charger TP4056 with 2 status output
 *	- battery voltage read with ADC
 *
 *	Authors:
 *	Martin Cerveny		  <M.Cerveny@computer.org>
 */
#include <linux/module.h>

#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#define DRIVER_DESC "WX8 battery charger and power management driver"
#define MODULE_DEVICE_ALIAS "platform:wx8-charger"

MODULE_LICENSE("GPL");
MODULE_ALIAS(MODULE_DEVICE_ALIAS);
MODULE_AUTHOR("M.Cerveny@computer.org");

struct wx8_charger {
	// DEVICE REGISTRATION

	struct platform_device		*pdev;
	struct device			*dev;

	struct power_supply		*bat;
	struct power_supply		*usb;

	struct workqueue_struct		*bat_monitor_wq;
	struct delayed_work		bat_delay_work;

	// RUNTIME DATA

	u32 voltage;
	u32 percent;
	u32 status;

	// DT

	struct iio_channel		*iio_chan;

	u32	ref_voltage;
	u32	voltage_divider[2];
	u32	discharge_table[32];
	u32	charge_table[32];
	u32	design_capacity;

	int	chg_ok_pin;
	int	led_power_pin;
	int	dc_det_pin;
	int	usb_boost_pin;
	u8	chg_ok_level;
	u8	led_power_level;
	u8	dc_det_level;
	u8	usb_boost_level;

	u8	charge_table_size;
	u8	discharge_table_size;
};

static enum power_supply_property wx8_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static int wx8_bat_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	struct wx8_charger *chgr = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chgr->voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = !!chgr->voltage;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chgr->status;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chgr->percent;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = chgr->design_capacity * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = max(chgr->charge_table[chgr->charge_table_size-1], chgr->discharge_table[chgr->discharge_table_size-1]) * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = min(chgr->charge_table[0], chgr->discharge_table[0]) * 1000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property wx8_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int wx8_usb_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int ret = 0;
	struct wx8_charger *chgr = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
                val->intval = chgr->status != POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct power_supply_desc wx8_bat_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= wx8_bat_props,
	.num_properties	= ARRAY_SIZE(wx8_bat_props),
	.get_property	= wx8_bat_get_property,
};

static const struct power_supply_desc wx8_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = wx8_usb_props,
	.num_properties = ARRAY_SIZE(wx8_usb_props),
	.get_property = wx8_usb_get_property,
};

static int wx8_init_power_supply(struct wx8_charger *chgr)
{
	struct power_supply_config psy_cfg = { .drv_data = chgr, };

	chgr->bat = devm_power_supply_register(chgr->dev,
					     &wx8_bat_desc, &psy_cfg);
	if (IS_ERR(chgr->bat)) {
		dev_err(chgr->dev, "register bat power supply fail\n");
		return PTR_ERR(chgr->bat);
	}

	chgr->usb = devm_power_supply_register(chgr->dev,
					     &wx8_usb_desc, &psy_cfg);
	if (IS_ERR(chgr->usb)) {
		dev_err(chgr->dev, "register usb power supply fail\n");
		return PTR_ERR(chgr->usb);
	}

	return 0;
}

static void wx8_battery_work(struct work_struct *work)
{
	struct wx8_charger *chgr = container_of(work, struct wx8_charger, bat_delay_work.work);

	int val, ret, i, d;
	u32 *table, size;

	ret = gpio_get_value(chgr->dc_det_pin);
	if (ret == chgr->dc_det_level) {
		// DC ON
		ret = gpio_get_value(chgr->chg_ok_pin);
		if (ret == chgr->chg_ok_level) {
			// CHARGING
			chgr->status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else {
			// NOT CHARGING
			//ret = gpio_get_value(chgr->led_power_pin); ?
			chgr->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		table = chgr->charge_table;
		size = chgr->charge_table_size;
	} else {
		// DC OFF
		chgr->status = POWER_SUPPLY_STATUS_DISCHARGING;
		table = chgr->discharge_table;
		size = chgr->discharge_table_size;
	}

	ret = iio_read_channel_raw(chgr->iio_chan, &val);
	chgr->voltage = (val * chgr->ref_voltage * (chgr->voltage_divider[0]+chgr->voltage_divider[1])) / (1024*chgr->voltage_divider[0]);
	for (i = 0; i < size; i++) {
		if (chgr->voltage < table[i])
			break;
	}
	if ((i > 0) && (i < size)) {
		d = (chgr->voltage - table[i - 1]) * (1000 / (size - 1)) / (table[i] - table[i - 1]) + (i - 1) * (1000 / (size - 1));
	} else {
		if (i == 0) d = 0;
		else d = 1000;
	}
	chgr->percent = d/10;

	queue_delayed_work(chgr->bat_monitor_wq, &chgr->bat_delay_work, msecs_to_jiffies(500));
}

static int wx8_charger_parse_dt(struct wx8_charger *chgr)
{
	int length, ret;
	struct device_node *np = chgr->dev->of_node;
	struct device *dev = chgr->dev;
	enum of_gpio_flags flags;

	ret = of_property_read_u32(np, "ref_voltage", &chgr->ref_voltage);
	if (ret < 0) {
		dev_err(dev, "ref_voltage not found!\n");
		return ret;
	}

	ret = of_property_read_u32(np, "design_capacity", &chgr->design_capacity);
	if (ret < 0) {
		dev_err(dev, "design_capacity not found!\n");
		return ret;
	}

	if (!of_find_property(np, "charge_table", &length) || length % sizeof(u32) != 0 || length / sizeof(u32)<2 || length>sizeof(chgr->charge_table)) {
		dev_err(dev, "bad charge_table!\n");
		return -EINVAL;
	}
	chgr->charge_table_size = length / sizeof(u32);
	ret = of_property_read_u32_array(np, "charge_table", chgr->charge_table, chgr->charge_table_size);
	if (ret < 0)
		return ret;

	if (!of_find_property(np, "discharge_table", &length) || length % sizeof(u32) !=0 || length / sizeof(u32)<2 || length>sizeof(chgr->discharge_table)) {
		dev_err(dev, "bad discharge_table!\n");
		return -EINVAL;
	}
	chgr->discharge_table_size = length / sizeof(u32);
	ret = of_property_read_u32_array(np, "discharge_table", chgr->discharge_table, chgr->discharge_table_size);
	if (ret < 0)
		return ret;

	if (!of_find_property(np, "voltage_divider", &length) || length != sizeof(chgr->voltage_divider)) {
		dev_err(dev, "bad voltage_divider!\n");
		return -EINVAL;
	}
	ret = of_property_read_u32_array(np, "voltage_divider", chgr->voltage_divider, ARRAY_SIZE(chgr->voltage_divider));
	if (ret < 0)
		return ret;
	if (!chgr->voltage_divider[0]) {
		dev_err(dev, "zero in first voltage_divider, use <1 0> to run without divider!\n");
		return -EINVAL;
	}

	chgr->dc_det_pin = of_get_named_gpio_flags(np, "dc_det_gpio", 0, &flags);
	if (!gpio_is_valid(chgr->dc_det_pin)) {
		dev_err(dev, "bad dc_det_gpio!\n");
		return -EINVAL;
	}
	chgr->dc_det_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = devm_gpio_request(dev, chgr->dc_det_pin, NULL);
	if (ret<0) {
		dev_err(dev, "bad request dc_det_gpio!\n");
		return ret;
	}
	ret = gpio_direction_input(chgr->dc_det_pin);
	if (ret<0) {
		dev_err(dev, "bad input dc_det_pin!\n");
		return ret;
	}

	chgr->chg_ok_pin = of_get_named_gpio_flags(np, "chg_ok_gpio", 0, &flags);
	if (!gpio_is_valid(chgr->chg_ok_pin)) {
		dev_err(dev, "bad chg_ok_gpio!\n");
		return -EINVAL;
	}
	chgr->chg_ok_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = devm_gpio_request(dev, chgr->chg_ok_pin, NULL);
	if (ret<0) {
		dev_err(dev, "bad request chg_ok_pin!\n");
		return ret;
	}
	ret = gpio_direction_input(chgr->chg_ok_pin);
	if (ret<0) {
		dev_err(dev, "bad input chg_ok_pin!\n");
		return ret;
	}

	chgr->led_power_pin = of_get_named_gpio_flags(np, "led_power_gpio", 0, &flags);
	if (!gpio_is_valid(chgr->led_power_pin)) {
		dev_err(dev, "bad led_power_gpio!\n");
		return -EINVAL;
	}
	chgr->led_power_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = devm_gpio_request(dev, chgr->led_power_pin, NULL);
	if (ret<0) {
		dev_err(dev, "bad request led_power_pin!\n");
		return ret;
	}
	ret = gpio_direction_input(chgr->led_power_pin);
	if (ret<0) {
		dev_err(dev, "bad input led_power_pin!\n");
		return ret;
	}

	chgr->usb_boost_pin = of_get_named_gpio_flags(np, "usb_boost_gpio", 0, &flags);
	if (!gpio_is_valid(chgr->usb_boost_pin)) {
		dev_err(dev, "bad usb_boost_gpio!\n");
		return -EINVAL;
	}
	chgr->usb_boost_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = devm_gpio_request(dev, chgr->usb_boost_pin, NULL);
	if (ret<0) {
		dev_err(dev, "bad request usb_boost_pin!\n");
		return ret;
	}
	ret = gpio_direction_output(chgr->chg_ok_pin, 0 + chgr->usb_boost_level);
	if (ret<0) {
		dev_err(dev, "bad input chg_ok_pin!\n");
		return ret;
	}
	chgr->iio_chan = devm_iio_channel_get(dev, NULL);
	if (IS_ERR(chgr->iio_chan)) {
		if (PTR_ERR(chgr->iio_chan) != -EPROBE_DEFER) dev_err(dev, "ADC io-channels error!\n");
		return PTR_ERR(chgr->iio_chan);
	}
	return 0;
}

static const struct of_device_id wx8_charger_of_match[] = {
	{.compatible = "wx8-charger",},
	{ },
};

static int wx8_charger_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id = of_match_device(wx8_charger_of_match, &pdev->dev);
	struct wx8_charger *chgr;
	int ret;

	if (!of_id) {
		dev_err(&pdev->dev, "Failed to find matching dt id\n");
		return -ENODEV;
	}

	chgr = devm_kzalloc(&pdev->dev, sizeof(*chgr), GFP_KERNEL);
	if (!chgr)
		return -ENOMEM;

	chgr->pdev = pdev;
	chgr->dev = &pdev->dev;
	platform_set_drvdata(pdev, chgr);

	ret = wx8_charger_parse_dt(chgr);
	if (ret < 0) {
	 	dev_err(&pdev->dev, "WX8 battery parse dt failed!\n");
	 	return ret;
	}

	ret = wx8_init_power_supply(chgr);
	if (ret) {
		dev_err(&pdev->dev, "WX8 power supply register failed!\n");
		return ret;
	}
	chgr->bat_monitor_wq = alloc_ordered_workqueue("%s",
			WQ_MEM_RECLAIM | WQ_FREEZABLE, "wx8-bat-monitor-wq");
	INIT_DELAYED_WORK(&chgr->bat_delay_work, wx8_battery_work);
	queue_delayed_work(chgr->bat_monitor_wq, &chgr->bat_delay_work,
			   msecs_to_jiffies(100 * 5));

	return 0;
}


static void wx8_charger_shutdown(struct platform_device *dev)
{
	struct wx8_charger *chgr = platform_get_drvdata(dev);

	cancel_delayed_work_sync(&chgr->bat_delay_work);
	destroy_workqueue(chgr->bat_monitor_wq);
	// devm for memory, gpio, adc
}

static struct platform_driver wx8_charger_driver = {
	.driver = {
		.name = MODULE_DEVICE_ALIAS,
		.of_match_table = wx8_charger_of_match,
	},
	.probe = wx8_charger_probe,
	.shutdown = wx8_charger_shutdown,
};

static int __init charger_init(void)
{
	return platform_driver_register(&wx8_charger_driver);
}
fs_initcall_sync(charger_init);

static void __exit charger_exit(void)
{
	platform_driver_unregister(&wx8_charger_driver);
}
module_exit(charger_exit);
