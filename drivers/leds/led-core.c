/*
 * LED Class Core
 *
 * Copyright 2005-2006 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/leds.h>
#include "leds.h"

DECLARE_RWSEM(leds_list_lock);
EXPORT_SYMBOL_GPL(leds_list_lock);

LIST_HEAD(leds_list);
EXPORT_SYMBOL_GPL(leds_list);

/*Magnum 2014-1-14, transfer blink_brt,blink_offms, blink_onms to led driver code. */
extern void  tinno_max77660_get_rg_blink_brightness(unsigned long blink_brt);
extern void  tinno_max77660_get_rg_blink_offms(unsigned long offms);
extern void  tinno_max77660_get_rg_blink_onms(unsigned long onms);


static void led_stop_software_blink(struct led_classdev *led_cdev)
{
	/* deactivate previous settings */
	del_timer_sync(&led_cdev->blink_timer);
	led_cdev->blink_delay_on = 0;
	led_cdev->blink_delay_off = 0;
	/*Magnum 2014-1-15 change global blink_brightness, delay_on && delay_off to 
	* led driver	*/ 
	tinno_max77660_get_rg_blink_brightness(0);
	tinno_max77660_get_rg_blink_onms(0);
	tinno_max77660_get_rg_blink_offms(0);
}

static void led_set_software_blink(struct led_classdev *led_cdev,
				   unsigned long delay_on,
				   unsigned long delay_off)
{
	int current_brightness;

	current_brightness = led_get_brightness(led_cdev);
	if (current_brightness)
		led_cdev->blink_brightness = current_brightness;
	if (!led_cdev->blink_brightness)
		led_cdev->blink_brightness = led_cdev->max_brightness;

	if (led_get_trigger_data(led_cdev) &&
	    delay_on == led_cdev->blink_delay_on &&
	    delay_off == led_cdev->blink_delay_off)
		return;

	led_stop_software_blink(led_cdev);

	led_cdev->blink_delay_on = delay_on;
	led_cdev->blink_delay_off = delay_off;

	/* never on - don't blink */
	if (!delay_on)
		return;

	/* never off - just set to brightness */
	if (!delay_off) {
		led_set_brightness(led_cdev, led_cdev->blink_brightness);
		return;
	}
	/*Magnum 2014-1-15 change global blink_brightness, delay_on && delay_off to 
	* led driver	*/ 
	tinno_max77660_get_rg_blink_brightness(led_cdev->blink_brightness);
	tinno_max77660_get_rg_blink_onms(led_cdev->blink_delay_on);
	tinno_max77660_get_rg_blink_offms(led_cdev->blink_delay_off);
	mod_timer(&led_cdev->blink_timer, jiffies + 1);
}


void led_blink_set(struct led_classdev *led_cdev,
		   unsigned long *delay_on,
		   unsigned long *delay_off)
{
	del_timer_sync(&led_cdev->blink_timer);

	if (led_cdev->blink_set &&
	    !led_cdev->blink_set(led_cdev, delay_on, delay_off))
		return;

	/* blink with 1 Hz as default if nothing specified */
	if (!*delay_on && !*delay_off)
		*delay_on = *delay_off = 500;

	led_set_software_blink(led_cdev, *delay_on, *delay_off);
}
EXPORT_SYMBOL(led_blink_set);

void led_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness brightness)
{
	led_stop_software_blink(led_cdev);
	led_cdev->brightness_set(led_cdev, brightness);
}
EXPORT_SYMBOL(led_brightness_set);
