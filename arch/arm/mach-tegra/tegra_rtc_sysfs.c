/*
 * Copyright (c) 2014 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "timer.h" /* u64 tegra_rtc_read_ms() */

static ssize_t tegra_rtc_sysfs_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%llu\n", tegra_rtc_read_ms());
}

static DEVICE_ATTR(value, S_IRUSR|S_IRGRP|S_IROTH, tegra_rtc_sysfs_show, NULL);

static int tegra_rtc_sysfs_probe(struct platform_device *pdev)
{
	return device_create_file(&pdev->dev, &dev_attr_value);
}

static int __exit tegra_rtc_sysfs_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_value);
	return 0;
}

static struct platform_driver tegra_rtc_sysfs_driver = {
	.driver = {
		.name = "tegra_rtc_sysfs",
		.owner = THIS_MODULE,
	},
	.probe = tegra_rtc_sysfs_probe,
	.remove = tegra_rtc_sysfs_remove,
};

static int __init tegra_rtc_sysfs_init(void)
{
	return platform_driver_register(&tegra_rtc_sysfs_driver);
}

static void __exit tegra_rtc_sysfs_exit(void)
{
	platform_driver_unregister(&tegra_rtc_sysfs_driver);
}

module_init(tegra_rtc_sysfs_init);
module_exit(tegra_rtc_sysfs_exit);

MODULE_AUTHOR("Hervé Fache <hfache@nvidia.com>");
MODULE_DESCRIPTION("Tegra internal RTC sysfs access");
MODULE_LICENSE("GPL");
