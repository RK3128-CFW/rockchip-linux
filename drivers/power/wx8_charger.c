// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *	WX8 power management and charger driver
 *
 *	- charger is independent Li-Ion linear charger TP4056 with 2 status output (only /CHGR is connected in WX8)
 *	- battery voltage read with ADC
 *	- automatic power switching "usb2sys" and "boost2usb"
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

// config constatnts

#define WX8_RECHECK_CHARGER	(60*60*1000)	// [ms] recheck charger, when running from battery and usb device is connected
#define WX8_UPDATE_STATUS	(2000)		// [ms] repeatedly updates battery status

// data structures

struct wx8_charger {
	// device registration

	struct platform_device		*pdev;
	struct device			*dev;

	struct power_supply		*bat;
	struct power_supply		*usb;

	struct workqueue_struct		*bat_monitor_wq;
	struct delayed_work		bat_delay_work;

	struct workqueue_struct		*usb_charger_wq;
	struct delayed_work		usb_delay_work;
	struct notifier_block 		usb_nb;

	// runtime data

	u32 voltage;
	u32 percent;
	u32 status;

	bool ready;

	// device-tree

	struct extcon_dev 		*edev;
	struct iio_channel		*iio_chan;

	u32	ref_voltage;
	u32	voltage_divider[2];
	u32	discharge_table[32];
	u32	charge_table[32];
	u32	design_capacity;

	int	charging_pin;
	int	usb2sys_pin;
	int	dc_det_pin;
	int	boost2usb_pin;
	u8	charging_level;
	u8	usb2sys_level;
	u8	dc_det_level;
	u8	boost2usb_level;

	u8	charge_table_size;
	u8	discharge_table_size;
};

// power supply properties and registration

static enum power_supply_property wx8_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
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
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
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

// workers, irq, notifiers

static void wx8_battery_work(struct work_struct *work)
{
	struct wx8_charger *chgr = container_of(work, struct wx8_charger, bat_delay_work.work);

	int val, ret, i, d;
	u32 *table, size;

	ret = gpio_get_value(chgr->dc_det_pin);
	if (ret == chgr->dc_det_level) {
		// DC ON
		ret = gpio_get_value(chgr->charging_pin);
		if (ret == chgr->charging_level) {
			// CHARGING
			chgr->status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else {
			// NOT CHARGING
			if (chgr->percent > 90) // last value
				// TP4056 stop charging, assume FULL, pin /STBY not connected
				chgr->status = POWER_SUPPLY_STATUS_FULL;
			else
				 // some sort of fail - vin too low; temperature of battery too low or too high; no battery
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

	// read voltage and recompute
	ret = iio_read_channel_raw(chgr->iio_chan, &val);
	chgr->voltage = (val * chgr->ref_voltage * (chgr->voltage_divider[0]+chgr->voltage_divider[1])) / (1024*chgr->voltage_divider[0]);
	for (i = 0; i < size; i++) {
		if (chgr->voltage < table[i])
			break;
	}
	if ((i > 0) && (i < size)) {
		d = ((chgr->voltage - table[i - 1]) * (1000 / (size - 1))) / (table[i] - table[i - 1]) + (i - 1) * (1000 / (size - 1));
	} else {
		if (i == 0) d = 0;
		else d = 1000;
	}
	chgr->percent = d/10;

	queue_delayed_work(chgr->bat_monitor_wq, &chgr->bat_delay_work, msecs_to_jiffies(WX8_UPDATE_STATUS));
}

static void wx8_usb_work(struct work_struct *work)
{
	struct wx8_charger *chgr = container_of(work, struct wx8_charger, usb_delay_work.work);
	struct device *dev = chgr->dev;
	int ret;

	if (extcon_get_state(chgr->edev, EXTCON_USB_VBUS_EN) > 0) { // in host mode
		// need power
		ret = gpio_get_value(chgr->dc_det_pin);
		if (ret != chgr->dc_det_level) {
			if (gpio_get_value(chgr->boost2usb_pin) == chgr->boost2usb_level) {
				// TODO: try other methods to test charger connected, problem: connected microUSB ID between host+otg, EXTCON_* does not work as expected

				// temporary disable boost (if enabled) and check again for dc_det ...
				gpio_set_value(chgr->boost2usb_pin, !chgr->boost2usb_level);
				gpio_set_value(chgr->usb2sys_pin, chgr->usb2sys_level);

				msleep(5);

				ret = gpio_get_value(chgr->dc_det_pin);
				if (ret != chgr->dc_det_level) {
					dev_dbg(dev, "recheck, continue booster to USB\n");

					gpio_set_value(chgr->usb2sys_pin, !chgr->usb2sys_level);
					gpio_set_value(chgr->boost2usb_pin, chgr->boost2usb_level);

					queue_delayed_work(chgr->usb_charger_wq, &chgr->usb_delay_work, msecs_to_jiffies(WX8_RECHECK_CHARGER));
				}
				else dev_dbg(dev, "recheck, now charger powered\n");
			} else {
				dev_dbg(dev, "enable booster to USB\n");

				gpio_set_value(chgr->usb2sys_pin, !chgr->usb2sys_level);
				gpio_set_value(chgr->boost2usb_pin, chgr->boost2usb_level);

				queue_delayed_work(chgr->usb_charger_wq, &chgr->usb_delay_work, msecs_to_jiffies(WX8_RECHECK_CHARGER));
			}
		} else dev_dbg(dev, "charger powered\n");
	} else {
		// does not need power
		dev_dbg(dev, "power not needed\n");
		gpio_set_value(chgr->boost2usb_pin, !chgr->boost2usb_level);
		gpio_set_value(chgr->usb2sys_pin, chgr->usb2sys_level);
	}
}

static int wx8_usb_evt_notifier(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct wx8_charger *chgr = container_of(nb, struct wx8_charger, usb_nb);

	dev_dbg(chgr->dev, "wx8_usb_evt_notifier\n");
	if (chgr->ready) mod_delayed_work(chgr->usb_charger_wq, &chgr->usb_delay_work, 0);

	return NOTIFY_DONE;
}

static irqreturn_t wx8_dc_det_isr(int irq, void *data)
{
	struct wx8_charger *chgr = (struct wx8_charger *)data;

	dev_dbg(chgr->dev, "wx8_dc_det_isr\n");
	irq_set_irq_type(irq, gpio_get_value(chgr->dc_det_pin) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
	if (chgr->ready) mod_delayed_work(chgr->usb_charger_wq, &chgr->usb_delay_work, 0);

	return IRQ_HANDLED;
}

// registration

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

	// works only when usb2sys_gpio !
	chgr->dc_det_pin = of_get_named_gpio_flags(np, "dc_det_gpio", 0, &flags);
	if (!gpio_is_valid(chgr->dc_det_pin)) {
		dev_err(dev, "bad dc_det_gpio!\n");
		return -EINVAL;
	}
	chgr->dc_det_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = devm_gpio_request(dev, chgr->dc_det_pin, "DC detected");
	if (ret<0) {
		dev_err(dev, "bad request dc_det_gpio!\n");
		return ret;
	}
	ret = gpio_direction_input(chgr->dc_det_pin);
	if (ret<0) {
		dev_err(dev, "bad input dc_det_pin!\n");
		return ret;
	}

	// /CHRG output from TP4056
	chgr->charging_pin = of_get_named_gpio_flags(np, "charging_gpio", 0, &flags);
	if (!gpio_is_valid(chgr->charging_pin)) {
		dev_err(dev, "bad charging_gpio!\n");
		return -EINVAL;
	}
	chgr->charging_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = devm_gpio_request(dev, chgr->charging_pin, "charging now");
	if (ret<0) {
		dev_err(dev, "bad request charging_pin!\n");
		return ret;
	}
	ret = gpio_direction_input(chgr->charging_pin);
	if (ret<0) {
		dev_err(dev, "bad input charging_pin!\n");
		return ret;
	}

	// power switch "vcc_booster" -> vcc_usb"
	chgr->boost2usb_pin = of_get_named_gpio_flags(np, "boost2usb_gpio", 0, &flags);
	if (!gpio_is_valid(chgr->boost2usb_pin)) {
		dev_err(dev, "bad boost2usb_gpio!\n");
		return -EINVAL;
	}
	chgr->boost2usb_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = devm_gpio_request(dev, chgr->boost2usb_pin, "booster power to usb");
	if (ret<0) {
		dev_err(dev, "bad request boost2usb_pin!\n");
		return ret;
	}
	// disable
	ret = gpio_direction_output(chgr->boost2usb_pin, !chgr->boost2usb_level);
	if (ret<0) {
		dev_err(dev, "bad input boost2usb_pin!\n");
		return ret;
	}

	// power switch "vcc_usb" -> "vcc_sys" & "vcc_charger"
	chgr->usb2sys_pin = of_get_named_gpio_flags(np, "usb2sys_gpio", 0, &flags);
	if (!gpio_is_valid(chgr->usb2sys_pin)) {
		dev_err(dev, "bad usb2sys_gpio!\n");
		return -EINVAL;
	}
	chgr->usb2sys_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	ret = devm_gpio_request(dev, chgr->usb2sys_pin, "usb power to system");
	if (ret<0) {
		dev_err(dev, "bad request usb2sys_pin!\n");
		return ret;
	}
	// enable
	ret = gpio_direction_output(chgr->usb2sys_pin, chgr->usb2sys_level);
	if (ret<0) {
		dev_err(dev, "bad input usb2sys_pin!\n");
		return ret;
	}

	chgr->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(chgr->edev)) {
		if (PTR_ERR(chgr->edev) != -EPROBE_DEFER) dev_err(dev, "Invalid or missing extcon\n");
		return PTR_ERR(chgr->edev);
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
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id = of_match_device(wx8_charger_of_match, dev);
	struct wx8_charger *chgr;
	int ret, dc_det_irq;

	if (!of_id) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -ENODEV;
	}

	chgr = devm_kzalloc(dev, sizeof(*chgr), GFP_KERNEL);
	if (!chgr)
		return -ENOMEM;

	chgr->pdev = pdev;
	chgr->dev = dev;
	platform_set_drvdata(pdev, chgr);

	ret = wx8_charger_parse_dt(chgr);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER) dev_err(dev, "DT parse failed!\n");
	 	return ret;
	}

	ret = wx8_init_power_supply(chgr);
	if (ret) {
		dev_err(dev, "power supply register failed!\n");
		return ret;
	}

	chgr->bat_monitor_wq = alloc_ordered_workqueue("%s",
			WQ_MEM_RECLAIM | WQ_FREEZABLE, "wx8-bat-monitor-wq");
	INIT_DELAYED_WORK(&chgr->bat_delay_work, wx8_battery_work);
	queue_delayed_work(chgr->bat_monitor_wq, &chgr->bat_delay_work, 0);

	chgr->usb_charger_wq = alloc_ordered_workqueue("%s",
			WQ_MEM_RECLAIM | WQ_FREEZABLE, "wx8-usb-charger-wq");
	INIT_DELAYED_WORK(&chgr->usb_delay_work, wx8_usb_work);
	queue_delayed_work(chgr->usb_charger_wq, &chgr->usb_delay_work, 0);
	chgr->usb_nb.notifier_call = wx8_usb_evt_notifier;
	ret = devm_extcon_register_notifier(dev, chgr->edev, EXTCON_USB_VBUS_EN, &chgr->usb_nb);
	if (ret < 0) {
		dev_err(dev, "failed to register notifier for VBUS_EN\n");
		return ret;
	}

	dc_det_irq = gpio_to_irq(chgr->dc_det_pin);
	ret = devm_request_irq(dev, dc_det_irq, wx8_dc_det_isr,
			       gpio_get_value(chgr->dc_det_pin) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH, "wx8_dc_det", chgr);
	if (ret != 0) {
		dev_err(dev, "wx8_dc_det_irq request failed!\n");
		return ret;
	}
	enable_irq_wake(dc_det_irq);

	chgr->ready = true;
	return 0;
}


static void wx8_charger_shutdown(struct platform_device *dev)
{
	struct wx8_charger *chgr = platform_get_drvdata(dev);

	chgr->ready = false;

	cancel_delayed_work_sync(&chgr->usb_delay_work);
	destroy_workqueue(chgr->usb_charger_wq);

	cancel_delayed_work_sync(&chgr->bat_delay_work);
	destroy_workqueue(chgr->bat_monitor_wq);
	// devm for memory, gpio, adc, extcon, irq
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
