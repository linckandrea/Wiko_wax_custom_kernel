/*
 * gpio button support
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t14.h>
#include <linux/gpio.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"

#define INIT_GPIO	TEGRA_GPIO_PJ3

#define DRV_NAME	"gpiobutton"

static DEFINE_MUTEX(gpio_button_lock);

struct gpio_button_info {
	struct device dev;
	struct mutex in_use;
	struct mutex update_lock;
	struct delayed_work	dwork;
	unsigned int irq;
	unsigned int gpio_value;
};

static void gpio_button_work_handler(struct work_struct *work)
{
	struct gpio_button_info *data = container_of(work, struct gpio_button_info, dwork.work);
	data->gpio_value = gpio_get_value_cansleep(data->irq);
}

static irqreturn_t gpio_button_interrupt(int vec, void *info)
{
	unsigned long flags;
	struct gpio_button_info *ginfo = (struct gpio_button_info*)info;
	ginfo->gpio_value = gpio_get_value_cansleep(INIT_GPIO);
	printk("==> gpio_button_interrupt %d\n", gpio_get_value_cansleep(INIT_GPIO));

	return IRQ_HANDLED;
}


static int gpio_button_open(struct inode *inode, struct file *file)
{
	return 0; 
}

static int gpio_button_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t gpio_button_show_gpiodata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gpio_button_info *ginfo = container_of(dev, struct gpio_button_info, dev);
	int gpiodata = gpio_get_value_cansleep(INIT_GPIO);
	return sprintf(buf, "%d\n", gpiodata);
}

static DEVICE_ATTR(gpiodata, S_IRUGO, gpio_button_show_gpiodata, NULL);

static struct attribute *gpio_button_attributes[] = {
	&dev_attr_gpiodata.attr,
	NULL
};

static const struct attribute_group gpio_button_attr_group = {
	.attrs = gpio_button_attributes,
};

static struct file_operations gpio_button_fops = {
	.owner = THIS_MODULE,
	.open = gpio_button_open,
	.release = gpio_button_release,
};

static struct miscdevice gpio_button_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpio_button_dev",
	.fops = &gpio_button_fops,
};

static int gpio_button_probe(struct platform_device *pdev)
{
	struct gpio_button_info *trig_info;
	int irq = INIT_GPIO, ret = 0, err;

	printk("%s enter\n", __func__);

	trig_info = kzalloc(sizeof(*trig_info), GFP_KERNEL);
	if (!trig_info) {
		ret = -ENOMEM;
		goto exit;
	}

	platform_set_drvdata(pdev, trig_info);

	trig_info->irq = irq;
	gpio_set_debounce(trig_info->irq, 10 * 1000);
	ret = request_irq(gpio_to_irq(trig_info->irq), gpio_button_interrupt, IRQ_TYPE_EDGE_RISING|IRQ_TYPE_EDGE_FALLING,
			  DRV_NAME, (void *)&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "request IRQ-%d failed", irq);
		goto exit;
	}
	disable_irq_wake(gpio_to_irq(trig_info->irq));

	INIT_DELAYED_WORK(&trig_info->dwork, gpio_button_work_handler);

	err = sysfs_create_group(&pdev->dev.kobj, &gpio_button_attr_group);
	if (err)
		goto error_release_irq;

	err = misc_register(&gpio_button_device);
	if (err) {
		printk("Unalbe to register ps misc: %d", err);
		goto exit_remove_sysfs_group;
	}

	enable_irq_wake(gpio_to_irq(trig_info->irq));
	printk("%s end\n", __func__);

	return 0;

/* First clean up the partly allocated trigger */
exit_remove_sysfs_group:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_button_attr_group);
error_release_irq:
	free_irq(gpio_to_irq(trig_info->irq), &pdev->dev);
exit:
	return ret;
}

static int gpio_button_remove(struct platform_device *pdev)
{
	struct iio_trigger *trig;
	struct gpio_button_info *trig_info;

	mutex_lock(&gpio_button_lock);
	free_irq(trig_info->irq, trig);
	mutex_unlock(&gpio_button_lock);
	platform_set_drvdata(pdev, NULL);

	return 0;
}


static struct platform_driver gpio_button_driver = {
	.probe = gpio_button_probe,
	.remove = gpio_button_remove,
	.driver = {
		.name = "gpio-button",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(gpio_button_driver);

MODULE_AUTHOR("ljs<ljs@tinno.com>");
MODULE_DESCRIPTION("gpio button for the iio subsystem");
MODULE_LICENSE("GPL v2");
