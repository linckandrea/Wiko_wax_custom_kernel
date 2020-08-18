/*
 * arch/arm/mach-tegra/board-p1852-kbc.c
 * Wake key configuration for Nvidia tegra3 p1852 platform.
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-p1852.h"

#include "gpio-names.h"
#include "devices.h"

#define GPIO_IKEY(_id, _irq, _iswake, _deb)	\
	{					\
		.code = _id,			\
		.gpio = -1,			\
		.irq = _irq,			\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb,	\
	}

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button p1852_wake_key[] = {
	[0] = GPIO_KEY(KEY_POWER, PDD3, 1),
};

static struct gpio_keys_platform_data p1852_wake_key_pdata = {
	.buttons	= p1852_wake_key,
	.nbuttons	= ARRAY_SIZE(p1852_wake_key),
};

static struct platform_device p1852_wake_key_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data = &p1852_wake_key_pdata,
	},
};

int __init p1852_keys_init(void)
{
	pr_info("Registering gpio keys\n");
	platform_device_register(&p1852_wake_key_device);
	return 0;
}
