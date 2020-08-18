/*
 * leds-max77660.c -- MAXIM MAX77660 led driver.
 *
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
//#define DEBUG 1
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/pm.h>
#include <linux/mfd/max77660/max77660-core.h>

//edit by Magnum 
#include <linux/leds.h>
#include <linux/workqueue.h>

//static int gOn_time;
struct max77660_leds {
	struct device		*dev;
	struct device		*parent;
};

struct max77660_ledblnk_map {
	unsigned long bits_val;
	unsigned long time_ms;
};

//edit by Magnum 
/*** As all current sources share the same blink on-time and period settings with a common register.
****  blink led through setting timer to turn on && off when resume,
****  blink led through writing MAX77660_REG_LEDBLNK register when suspend.
****  so a blink need 3 values, enabl + onms+ offms, all of them should be sensitve.
****  
*****   Note:  2014-1-8
******    CONFIG_S8515_PR_VERSION = 1 --- PR1 pcb , led mapping below:
		Red -- 	 MAX77660_REG_LED2BRT
		Green --   MAX77660_REG_LED1BRT
		Button key -- MAX77660_REG_LED0BRT(brighter) + MAX77660_REG_LED3BRT(darker)
******************************************************************************
	 CONFIG_S8515_PR_VERSION = 2 --- PR2 pcb , led mapping below:
		Red -- 	 MAX77660_REG_LED0BRT
		Green --   MAX77660_REG_LED3BRT
		Button key -- MAX77660_REG_LED1BRT(brighter) + MAX77660_REG_LED2BRT(darker)
		
*************/

#if (CONFIG_S8515_PR_VERSION == 2)
/*  led enable bit	 */
#define RED_ENABLE 			1
#define BUTTON0_ENABLE		2
#define BUTTON1_ENABLE 		4
#define GREEN_ENABLE  		8
#else
/*  led enable bit	 */
#define BUTTON0_ENABLE 		1
#define GREEN_ENABLE			2
#define RED_ENABLE 			4
#define BUTTON1_ENABLE   		8
#define RG_ENABLE			GREEN_ENABLE + RED_ENABLE
#endif

/*  led  disable bit	*/
#define RED_DISABLE			0xF - RED_ENABLE
#define GREEN_DISABLE		0xF - GREEN_ENABLE
#define BUTTON0_DISABLE		0xF - BUTTON0_ENABLE
#define BUTTON1_DISABLE		0xF - BUTTON1_ENABLE

#define MAX77660_DOUBLE_BUTTON_LIGHT

//static spinlock_t max77660_leds_lock;
static struct mutex max77660_leds_mutex;
static struct work_struct set_rg_leds_work;
static struct work_struct set_button_leds_work;

struct tinno_max77660_led {
	struct led_classdev	cdev;
	struct work_struct	work;
	unsigned int brightness;
	struct device		*master;
};

enum max77660_LED{
	RED_LED,
	GREEN_LED,
	BLUE_LED,
	BUTTON0_LED,
	BUTTON1_LED,
	RED_GREEN_LED,
};

static unsigned long g_rg_leds_brt= 0;
static unsigned long g_btn_leds_brt= 0;
static unsigned long g_blink_brightness; //= 0;
static unsigned long g_rg_blink_onms ;//= 500;
static unsigned long g_rg_blink_offms ;//= 2000;
static struct device *g_max77660_dev;
static int currentLedPower = 0;

struct max77660_ledblnk_map max77660_ledblnkp[] = {
	{ 0x0, 1000  },
	{ 0x1, 1500  },
	{ 0x2, 2000  },
	{ 0x3, 2500  },
	{ 0x4, 3000  },
	{ 0x5, 3500  },
	{ 0x6, 4000  },
	{ 0x7, 4500  },
	{ 0x8, 5000  },
	{ 0x9, 5500  },
	{ 0xA, 6000  },
	{ 0xB, 7000  },
	{ 0xC, 8000  },
	{ 0xD, 10000 },
	{ 0xE, 12000 },
	{ 0xF, 0     },
};

struct max77660_ledblnk_map max77660_ledblnkd[] = {
	{ 0x0, 0    },
	{ 0x1, 50   },
	{ 0x2, 100  },
	{ 0x3, 150  },
	{ 0x4, 200  },
	{ 0x5, 300  },
	{ 0x6, 350  },
	{ 0x7, 400  },
	{ 0x8, 450  },
	{ 0x9, 500  },
	{ 0xA, 550  },
	{ 0xB, 600  },
	{ 0xC, 700  },
	{ 0xD, 800  },
	{ 0xE, 900  },
	{ 0xF, 1000 },
};

static int max77660_add_attributes(struct device *dev,
				 struct device_attribute *attrs)
{
	int error = 0;
	int i;

	if (attrs) {
		for (i = 0; attr_name(attrs[i]); i++) {
			error = device_create_file(dev, &attrs[i]);
			if (error)
				break;
		}
		if (error)
			while (--i >= 0)
				device_remove_file(dev, &attrs[i]);
	}
	return error;
}

static void max77660_remove_attributes(struct device *dev,
				     struct device_attribute *attrs)
{
	int i;

	if (attrs)
		for (i = 0; attr_name(attrs[i]); i++)
			device_remove_file(dev, &attrs[i]);
}

static int max77660_disable_leds(struct max77660_leds *leds)
{
	int ret;

	/* Disable LED driver by default */
	ret = max77660_reg_write(leds->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LEDEN, 0x80);
	if (ret < 0) {
		dev_err(leds->dev, "LED write failed: %d\n", ret);
		return ret;
	}

	ret = max77660_reg_write(leds->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LEDBLNK, 0x0);
	if (ret < 0) {
		dev_err(leds->dev, "LEDBLNK write failed: %d\n", ret);
		return ret;
	}
	return 0;
}

static ssize_t max77660_leds_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned long leds_en = 0;

	ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LEDEN, &leds_en);
	dev_dbg(dev,"Ivan max77660_leds_enable_show =%x\n",leds_en);
	if (ret < 0) {
		dev_err(dev, "LEDEN read failed: %d\n", ret);
		return ret;
	}
	return sprintf(buf, "%u\n", (leds_en & 0xF));
}

static ssize_t max77660_leds_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	char *after;
	unsigned long leds_en = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;
	dev_dbg(dev,"Ivan max77660_leds_enable_store leds_en = %x\n",leds_en);

	if (count == size) {
	    
		ret = count;
		leds_en &= 0x0F;
		/* switch the led0 control to LED0EN bit */
		leds_en |= 0x80;
		/* enable leds */
		dev_dbg(dev,"Magnum max77660_leds_enable_store leds_en = 0x%x\n",leds_en);
		ret = max77660_reg_write(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LEDEN, leds_en);
		if (ret < 0) {
			dev_err(dev, "LEDEN write failed: %d\n", ret);
			goto out;
		}
	    dev_dbg(dev,"Ivan max77660_leds_enable_store OK!\n");
		
	} else {
		ret = -EINVAL;
		goto out;
	}

out:
	return count;
}

static ssize_t max77660_leds_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned long val = 0;
	unsigned long leds_brt = 0;
	dev_dbg(dev->parent,"Ivan max77660_leds_brightness_show \n");

	ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LED0BRT, &val);
	if (ret < 0) {
		dev_err(dev, "LED0BRT read failed: %d\n", ret);
		return ret;
	}
	leds_brt |= (val & 0xFF) << 1;
	dev_dbg(g_max77660_dev,"Magnum max77660_leds_brightness_show BUTTON0 == %u , val == 0x%x\n",leds_brt,val);
	ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LED1BRT, &val);
	if (ret < 0) {
		dev_err(dev, "LED1BRT read failed: %d\n", ret);
		return ret;
	}
	leds_brt |= (val & 0xFF) << 9;
	dev_dbg(g_max77660_dev,"Magnum max77660_leds_brightness_show GREEN == %u , val == 0x%x\n",leds_brt,val);
	ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LED2BRT, &val);
	if (ret < 0) {
		dev_err(dev, "LED2BRT read failed: %d\n", ret);
		return ret;
	}
	leds_brt |= (val & 0xFF) << 17;
	dev_dbg(g_max77660_dev,"Magnum max77660_leds_brightness_show RED == %u , val == 0x%x\n",leds_brt,val);
	ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LED3BRT, &val);
	if (ret < 0) {
		dev_err(dev, "LED3BRT read failed: %d\n", ret);
		return ret;
	}
	leds_brt |= (val & 0xFF) << 25;
	dev_dbg(g_max77660_dev,"Magnum max77660_leds_brightness_show BUTTON1 == %u , val == 0x%x\n",leds_brt,val);

	return sprintf(buf, "%u\n", leds_brt);
}

static ssize_t max77660_leds_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	char *after;
	unsigned long leds_brt = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	dev_dbg(dev,"Ivan max77660_leds_brightness_store leds_brt = %d \n",leds_brt);

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
//Ivan led0 and led3 connect to button backlight, led1 connect to RED, led2 connect to GREEN

		/* led0 */
		ret = max77660_reg_write(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LED0BRT, (leds_brt & 0xFF) >> 1);
		if (ret < 0) {
			dev_err(dev, "LED0BRT write failed: %d\n", ret);
			goto out;
		}
		/* led1 */
		ret = max77660_reg_write(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LED1BRT, ((leds_brt >> 8) & 0xFF) >> 1);
		if (ret < 0){
			dev_err(dev, "LED1BRT write failed: %d\n", ret);
			goto out;
		}
		/* led2 */
		ret = max77660_reg_write(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LED2BRT, ((leds_brt >> 16) & 0xFF) >> 1);
		if (ret < 0){
			dev_err(dev, "LED2BRT write failed: %d\n", ret);
			goto out;
		}
		/* led3 */
		ret = max77660_reg_write(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LED3BRT, ((leds_brt >> 24) & 0xFF) >> 1);
		if (ret < 0){
			dev_err(dev, "LED3BRT write failed: %d\n", ret);
			goto out;
		}
	    dev_dbg(dev,"Ivan max77660_leds_brightness_store OK! \n");
		
	} else {
		ret = -EINVAL;
		goto out;
	}

out:
	return count;

}

static ssize_t max77660_leds_onms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int i;
	unsigned long val = 0;
	unsigned long leds_blnkd = 0;
	dev_dbg(dev,"Ivan max77660_leds_onms_show \n");

	ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LEDBLNK, &val);
	if (ret < 0) {
		dev_err(dev, "LEDBLNK read failed: %d\n", ret);
		return ret;
	}
	leds_blnkd = (val >> 4) & 0xF;
	for (i = 0; i < ARRAY_SIZE(max77660_ledblnkd); i++) {
		if (max77660_ledblnkd[i].bits_val == leds_blnkd)
			break;
	}
	if (i >= ARRAY_SIZE(max77660_ledblnkd))
		return -EINVAL;
	else {
		return sprintf(buf, "%u\n", max77660_ledblnkd[i].time_ms);
	}
}

static ssize_t max77660_leds_onms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int i;
	char *after;
	unsigned long leds_onms = simple_strtoul(buf, &after, 10);
	unsigned long val = 0;
	size_t count = after - buf;
	dev_dbg(dev,"Ivan max77660_leds_onms_store leds_onms = %d\n",leds_onms);

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

//		gOn_time = leds_onms;

		for (i = 0; i < ARRAY_SIZE(max77660_ledblnkd); i++) {
			if (leds_onms <= max77660_ledblnkd[i].time_ms)
				break;
		}
		if (i >= ARRAY_SIZE(max77660_ledblnkd)) {
			dev_err(dev, "the time of blinking duration is too long, %d ms\n", leds_onms);
			return -EINVAL;
		}

		ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LEDBLNK, &val);
		if (ret < 0) {
			dev_err(dev, "LEDBLNK read failed: %d\n", ret);
			return ret;
		}

		val &= 0xF;
		val |= max77660_ledblnkd[i].bits_val << 4;

		ret = max77660_reg_write(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LEDBLNK, val);
		if (ret < 0) {
			dev_err(dev, "LEDBLNK write failed: %d\n", ret);
			goto out;
		}
	    dev_dbg(dev,"Ivan max77660_leds_onms_store OK!!\n");
		
	} else {
		ret = -EINVAL;
		goto out;
	}

out:
	return count;
}

static ssize_t max77660_leds_offms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int i;
	unsigned long val = 0;
	unsigned long leds_blnk = 0;
	unsigned long offms = 0;
	dev_dbg(dev,"Ivan max77660_leds_offms_show \n");

	ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LEDBLNK, &val);
	if (ret < 0) {
		dev_err(dev, "LEDBLNK read failed: %d\n", ret);
		return ret;
	}

	leds_blnk = val & 0xF;
	for (i = 0; i < ARRAY_SIZE(max77660_ledblnkp); i++) {
		if (max77660_ledblnkp[i].bits_val == leds_blnk)
			break;
	}
	if (i >= ARRAY_SIZE(max77660_ledblnkd))
		return -EINVAL;

	offms = max77660_ledblnkp[i].time_ms;

	leds_blnk = (val >> 4) & 0xF;
	for (i = 0; i < ARRAY_SIZE(max77660_ledblnkd); i++) {
		if (max77660_ledblnkd[i].bits_val == leds_blnk)
			break;
	}
	if (i >= ARRAY_SIZE(max77660_ledblnkd))
		return -EINVAL;

	offms -= max77660_ledblnkd[i].time_ms;

	return sprintf(buf, "%u\n", offms);
}

static ssize_t max77660_leds_offms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int i;
	char *after;
	unsigned long leds_offms = simple_strtoul(buf, &after, 10);
	unsigned long leds_blnkp_ms = 0;
	unsigned long leds_blnkd = 0;
	unsigned long val = 0;
	size_t count = after - buf;
	dev_dbg(dev,"Ivan max77660_leds_offms_store leds_offms = %d\n",leds_offms);

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		/* read max77660 ledblnk register */
		ret = max77660_reg_read(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LEDBLNK, &val);
		if (ret < 0) {
			dev_err(dev, "LEDBLNK read failed: %d\n", ret);
			return ret;
		}
		leds_blnkd = (val >> 4) & 0xF;

		/* look up duration time_ms of max77660 leds */
		for (i = 0; i < ARRAY_SIZE(max77660_ledblnkd); i++) {
			if (max77660_ledblnkd[i].bits_val == leds_blnkd)
				break;
		}
		if (i >= ARRAY_SIZE(max77660_ledblnkd))
			return -EINVAL;

		/* calculate the period time_ms of max77660 leds blinking */
		leds_blnkp_ms = leds_offms + max77660_ledblnkd[i].time_ms;

		/* look up the value need setting,
		 * matching with blinking period time_ms. */
		for (i = 0; i < ARRAY_SIZE(max77660_ledblnkp); i++) {
			if (leds_blnkp_ms <= max77660_ledblnkp[i].time_ms)
				break;
		}
		if (i >= ARRAY_SIZE(max77660_ledblnkp)) {
			dev_err(dev, "the time of blinking period is too long, %d ms\n", leds_blnkp_ms);
			return -EINVAL;
		}


		val &= 0xF0;
		val |= max77660_ledblnkp[i].bits_val;
		/* write max77660 ledblnk register */
		ret = max77660_reg_write(dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LEDBLNK, val);
//Ivan added
/*		
		if (gOn_time == 0 && leds_offms == 0)
		{
		    ret = max77660_reg_write(dev->parent, MAX77660_PWR_SLAVE,
				    MAX77660_REG_LEDBLNK, 0xF0);		//Ivan always on
	    	    dev_dbg(dev,"Ivan max77660_leds_offms_store ALWAYS ON!!\n");
		}
*/
		if (ret < 0) {
			dev_err(dev, "LEDBLNK write failed: %d\n", ret);
			goto out;
		}
	    dev_dbg(dev,"Ivan max77660_leds_offms_store OK!!\n");
		
	} else {
		ret = -EINVAL;
		goto out;
	}

out:
	return count;
}

static struct device_attribute max77660_leds_attrs[] = {
	__ATTR(enable, 0644, max77660_leds_enable_show, max77660_leds_enable_store),
	__ATTR(brightness, 0644, max77660_leds_brightness_show, max77660_leds_brightness_store),
	__ATTR(onms, 0644, max77660_leds_onms_show, max77660_leds_onms_store),
	__ATTR(offms, 0644, max77660_leds_offms_show, max77660_leds_offms_store),
	__ATTR_NULL,
};

//edit by Magnum 2013-12-12
static int set_leds_power(int source_bit, int red_bit, int green_bit, int button_bit){
	//dev_dbg(g_max77660_dev,"Magnum  ..current source_brt== 0x%x,  red_bit== 0x%x  ,green_bit== 0x%x , button_bit== 0x%x\n",
	//		source_bit,red_bit,green_bit,button_bit);
	
	if(red_bit == RED_ENABLE)
		source_bit = source_bit |RED_ENABLE;
	else if(red_bit == RED_DISABLE)
		source_bit = source_bit & RED_DISABLE;
	
	if(green_bit == GREEN_ENABLE)
		source_bit = source_bit |GREEN_ENABLE;
	else if(green_bit == GREEN_DISABLE)
		source_bit = source_bit & GREEN_DISABLE;
	
	if(button_bit == BUTTON1_ENABLE)
		source_bit = source_bit |BUTTON1_ENABLE;
	else if(button_bit == BUTTON1_DISABLE)
		source_bit = source_bit & BUTTON1_DISABLE;
	
	if(source_bit != currentLedPower)
		currentLedPower = source_bit;

//	dev_dbg(g_max77660_dev,"Magnum currentLedPower == 0x%x\n",currentLedPower);
	return currentLedPower;
}


static enum led_brightness max77660_led_get_brightness(enum max77660_LED led_type)
{
	int ret;
	int max77660_led_reg;
	unsigned long max77660_leds_brt = 0;
	dev_dbg(g_max77660_dev,"Magnum max77660_led_get_brightness \n");
	#if (CONFIG_S8515_PR_VERSION == 2)
		switch(led_type){
			case RED_LED : 
				max77660_led_reg = MAX77660_REG_LED0BRT;
				break;
			  
			case GREEN_LED:
			 	  max77660_led_reg = MAX77660_REG_LED3BRT;
				break;
				  
		        case BUTTON0_LED:
			 	 max77660_led_reg = MAX77660_REG_LED1BRT;
				break;
				  
			case BUTTON1_LED:
			 	 max77660_led_reg = MAX77660_REG_LED2BRT;
				break;	
			default:  dev_err(g_max77660_dev, " %s() no match LED \n",__func__); 
					return 0;
		}
	#else
		switch(led_type){
			case RED_LED : 
				max77660_led_reg = MAX77660_REG_LED2BRT;
				break;
			  
			case GREEN_LED:
			 	  max77660_led_reg = MAX77660_REG_LED1BRT;
				break;
				  
		        case BUTTON0_LED:
			 	 max77660_led_reg = MAX77660_REG_LED0BRT;
				break;
				  
			case BUTTON1_LED:
			 	 max77660_led_reg = MAX77660_REG_LED3BRT;
				break;	
			default:  dev_err(g_max77660_dev, " %s() no match LED \n",__func__); 
					return 0;
		}
	#endif
	
	ret = max77660_reg_read(g_max77660_dev->parent, MAX77660_PWR_SLAVE,
			max77660_led_reg, &max77660_leds_brt);
	if (ret < 0) {
		dev_err(g_max77660_dev, "LED0BRT read failed: %d\n", ret);
		return ret;
	}
	//dev_dbg(g_max77660_dev,"Magnum  %s() 0x%02x\n\n",__func__,max77660_leds_brt << 1);
	max77660_leds_brt = (max77660_leds_brt << 1) & 0xFF;
	//dev_dbg(g_max77660_dev,"Magnum %s() 0x%02x\n\n",__func__,max77660_leds_brt);
	return max77660_leds_brt;

}

static int max77660_led_set_brightness(enum max77660_LED led_type ,int leds_brt)
{
	int ret;
	int max77660_led_reg;
	if(leds_brt > 0xff ||leds_brt < 0){
		dev_err(g_max77660_dev, " %s() param brightness error\n",__func__);
		return 1;
	}
	#if (CONFIG_S8515_PR_VERSION == 2)
		switch(led_type){
			case RED_LED : 
				max77660_led_reg = MAX77660_REG_LED0BRT;
				break;
			  
			case GREEN_LED:
			 	  max77660_led_reg = MAX77660_REG_LED3BRT;
				break;
				  
		        case BUTTON0_LED:
			 	 max77660_led_reg = MAX77660_REG_LED1BRT;
				break;
				  
			case BUTTON1_LED:
			 	 max77660_led_reg = MAX77660_REG_LED2BRT;
				break;	
			default:  dev_err(g_max77660_dev, " %s() no match LED \n",__func__); 
					return 0;
		}
	#else
		switch(led_type){
			case RED_LED : 
				max77660_led_reg = MAX77660_REG_LED2BRT;
				break;
			  
			case GREEN_LED:
			 	  max77660_led_reg = MAX77660_REG_LED1BRT;
				break;
				  
		        case BUTTON0_LED:
			 	 max77660_led_reg = MAX77660_REG_LED0BRT;
				break;
				  
			case BUTTON1_LED:
			 	 max77660_led_reg = MAX77660_REG_LED3BRT;
				break;	
			default:  dev_err(g_max77660_dev, " %s() no match LED \n",__func__); 
					return 1;
		}
	#endif
	//dev_dbg(g_max77660_dev,"Magnum  %s()  max77660_led_reg == 0x%02x, leds_brt == 0x%02x\n",
	//	__func__,max77660_led_reg,leds_brt);
	
	ret = max77660_reg_write(g_max77660_dev->parent, MAX77660_PWR_SLAVE,max77660_led_reg, leds_brt >> 1);
	if (ret < 0) {
		dev_err(g_max77660_dev, "LEDEN read failed: %d\n", ret);
	}
	return ret;
}

static int max77660_led_get_enable()
{
	int ret;
	int max77660_leds_en;
	ret = max77660_reg_read(g_max77660_dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LEDEN, &max77660_leds_en);
	if (ret < 0) {
		dev_err(g_max77660_dev,"%s() failed \n", __func__);
		return ret;
	}
	//dev_dbg(g_max77660_dev,"Magnum max77660_led_get_enable =%x\n",max77660_leds_en);
	max77660_leds_en = max77660_leds_en & 0xF;
	return max77660_leds_en;
}
static int max77660_led_set_enable(int led_power)
{
	int ret;
	if(led_power > 0xf ||led_power < 0){
		dev_err(g_max77660_dev, "led_power== %d invalid \n",led_power);
		return 1;
	}
	int max77660_led_enable = led_power;
//	dev_dbg(g_max77660_dev,"Magnum  %s()  max77660_led_enable == %d\n\n",
//		__func__,max77660_led_enable);
	max77660_led_enable |= 0x80;     // notice this 
	ret = max77660_reg_write(g_max77660_dev->parent, MAX77660_PWR_SLAVE,MAX77660_REG_LEDEN, max77660_led_enable);
	if (ret < 0) {
		dev_err(g_max77660_dev, "LEDEN write failed: %d\n", ret);
	}
	return ret;
}

int max77660_led_set_hardware_blink(unsigned long onms, unsigned long offms)
{
	int ret;int i;
	unsigned long leds_onms = onms;
	unsigned long leds_offms = offms; 
	unsigned long leds_blnkp_ms = 0;
	unsigned long leds_blnkp = 0;
	unsigned long leds_blnkd = 0;
	unsigned long val = 0;
	dev_dbg(g_max77660_dev,"Magnum %s(), onms == %d, offms == %d \n",__func__,onms,offms);
	/* look up the value need setting, matching with blinking duration time_ms. */
	for (i = 0; i < ARRAY_SIZE(max77660_ledblnkd); i++) {
		if (leds_onms <= max77660_ledblnkd[i].time_ms)
			break;
	}
	if (i >= ARRAY_SIZE(max77660_ledblnkd)) {
		dev_err(g_max77660_dev, "the time of blinking duration is too long, %d ms\n", leds_onms);
		return -EINVAL;
	}
	
	leds_blnkd |= max77660_ledblnkd[i].bits_val << 4;

	//notice below:
	leds_blnkp_ms = leds_offms + max77660_ledblnkd[i].time_ms;
	//leds_blnkp_ms = leds_offms;
	
	/* look up the value need setting, matching with blinking period time_ms. */
	for (i = 0; i < ARRAY_SIZE(max77660_ledblnkp); i++) {
		if (leds_blnkp_ms <= max77660_ledblnkp[i].time_ms)
			break;
	}
	if (i >= ARRAY_SIZE(max77660_ledblnkp)) {
		dev_err(g_max77660_dev, "the time of blinking period is too long, %d ms\n", leds_blnkp_ms);
		return -EINVAL;
	}
	leds_blnkp = max77660_ledblnkd[i].bits_val;

	val = leds_blnkp |leds_blnkd;

	ret = max77660_reg_write(g_max77660_dev->parent, MAX77660_PWR_SLAVE,
			MAX77660_REG_LEDBLNK, val);
	if (ret < 0) {
		dev_err(g_max77660_dev, "LEDBLNK write failed: %d\n", ret);
		return -1;
	}
	return 0;	
}

//turn led into one-shot mode 
static int max77660_led_one_shot_mode()
{
	int ret;
	unsigned long val = 0xF0;
	ret = max77660_reg_write(g_max77660_dev->parent, MAX77660_PWR_SLAVE,
				MAX77660_REG_LEDBLNK, val);
	if (ret < 0) {
		dev_err(g_max77660_dev, "LEDEN read failed: %d\n", ret);
	}
	return ret;
}

static void set_rg_leds_brt(unsigned long leds_brt ){
	unsigned int leds_en = 0;
	unsigned long tmp_leds_brt= leds_brt;
	int tmp_led_type = RED_GREEN_LED;
	int red_enable_bit = 0;
	int green_enable_bit = 0;
	int button_enable_bit = 0;
	unsigned int red_brt = 0;
	unsigned int green_brt = 0;
	
	// Magnum 2013-12-23  step 1:setting brightness
	if(tmp_led_type == RED_GREEN_LED){
		green_brt = tmp_leds_brt & 0xFF;
		//dev_dbg(g_max77660_dev,"Magnum green_brt = 0x%02x ,led_brt ==0x%04x\n",red_brt,tmp_leds_brt);
		if(green_brt > 0){
			dev_dbg(g_max77660_dev,"Magnum green enable \n");
			tmp_led_type =GREEN_LED;
			green_enable_bit = GREEN_ENABLE;
			max77660_led_set_brightness(tmp_led_type,green_brt);
		}
		else {
			green_brt = 0;
			dev_dbg(g_max77660_dev,"Magnum green disable \n");
			tmp_led_type =GREEN_LED;
			green_enable_bit = GREEN_DISABLE;
			max77660_led_set_brightness(tmp_led_type,green_brt);
		}
		
		red_brt = (tmp_leds_brt >> 8) & 0xFF;
		//dev_dbg(g_max77660_dev,"Magnum red_brt = 0x%02x \n",green_brt);
		if(red_brt> 0){
			dev_dbg(g_max77660_dev,"Magnum red enable \n");
			tmp_led_type =RED_LED;
			red_enable_bit = RED_ENABLE;
			max77660_led_set_brightness(tmp_led_type,red_brt);
		}
		else {
			red_brt = 0;
			dev_dbg(g_max77660_dev,"Magnum red disable \n");
			tmp_led_type =RED_LED;
			red_enable_bit = RED_DISABLE;
			max77660_led_set_brightness(tmp_led_type,red_brt);	
		}
	}
//#endif
	//Magnum 2013-12-13  step 2: get  led enable;
	leds_en = max77660_led_get_enable();
	set_leds_power(leds_en, red_enable_bit, green_enable_bit, button_enable_bit);
	
	//Magnum 2013-12-13  step 3 :turn led into one-shot mode 
	max77660_led_one_shot_mode();

	//Magnum 2013-12-13  step 4 :set  led enable
	if(currentLedPower != leds_en){
		max77660_led_set_enable(currentLedPower);
	     	//dev_dbg(g_max77660_dev,"Magnum max77660_leds_enable_store OK!\n");
	}
	
	//dev_dbg(g_max77660_dev,"Magnum finish !\n");
}

static void set_rg_leds_work_func(struct work_struct *work)
{
	mutex_lock(&max77660_leds_mutex);
	set_rg_leds_brt(g_rg_leds_brt);
	mutex_unlock(&max77660_leds_mutex);	
}

static void set_button_leds_work_func(struct work_struct *work)
{
	mutex_lock(&max77660_leds_mutex);
	unsigned int leds_en = 0;
	unsigned long tmp_leds_brt= g_btn_leds_brt;
	int tmp_led_type = BUTTON1_LED;
	int red_enable_bit = 0;
	int green_enable_bit = 0;
	int button_enable_bit = 0;

	// Magnum 2013-12-23  step 1:setting brightness
	if(tmp_led_type == BUTTON1_LED){
		if( (tmp_leds_brt & 0xFF) > 0){
			dev_dbg(g_max77660_dev,"Magnum button enable \n");
			button_enable_bit = BUTTON1_ENABLE;
		}
		else {
			dev_dbg(g_max77660_dev,"Magnum button disable \n");
			button_enable_bit = BUTTON1_DISABLE;
		}
		tmp_led_type =BUTTON1_LED;
		max77660_led_set_brightness(tmp_led_type,tmp_leds_brt);
	}

	//Magnum 2013-12-13  step 2: get  led enable;
	leds_en = max77660_led_get_enable();
	set_leds_power(leds_en, red_enable_bit, green_enable_bit, button_enable_bit);
	
	//Magnum 2013-12-13  step 3 :turn led into one-shot mode 
	max77660_led_one_shot_mode();

	//Magnum 2013-12-13  step 4 :set  led enable
	if(currentLedPower != leds_en){
		max77660_led_set_enable(currentLedPower);
	     	//dev_dbg(g_max77660_dev,"Magnum max77660_leds_enable_store OK!\n");
	}
	
	//dev_dbg(g_max77660_dev,"Magnum finish !\n");
	mutex_unlock(&max77660_leds_mutex);
	
}

static void max77660_button1_set(struct led_classdev *led_cdev,enum led_brightness value){
	g_btn_leds_brt = value;
	dev_dbg(g_max77660_dev,"Magnum %s() ..value == %d\n",__func__,value);
	schedule_work(&set_button_leds_work); 
 }

static void max77660_red_green_set(struct led_classdev *led_cdev,unsigned int  value){
	g_rg_leds_brt = value;
	dev_dbg(g_max77660_dev,"Magnum %s() ..value == %04X\n",__func__,value);
	schedule_work(&set_rg_leds_work); 
 }

static enum led_brightness max77660_button1_get(struct led_classdev *led_cdev){
	return max77660_led_get_brightness(BUTTON1_LED);
}

static unsigned int  max77660_red_green_get(struct led_classdev *led_cdev){
	unsigned int  rg_brt ;
	rg_brt = max77660_led_get_brightness(RED_LED);
	rg_brt = (rg_brt << 8) | max77660_led_get_brightness(GREEN_LED);
	return rg_brt;
}

static struct led_classdev max77660_button1_led = {
	 .name= "button1",
	 .brightness_set = max77660_button1_set,
	 .brightness_get = max77660_button1_get,
	 .flags = 0,
};

static struct led_classdev max77660_rg_led = {
	 .name= "red-green",
	 .brightness_set = max77660_red_green_set,
	 .brightness_get = max77660_red_green_get,
	 .max_brightness = 65535,
	 .flags = 0,
};

void  tinno_max77660_get_rg_blink_brightness(unsigned long blink_brt){
	if(g_blink_brightness !=blink_brt){
		g_blink_brightness= blink_brt;
		dev_dbg(g_max77660_dev,"Magnum g_blink_brightness== %d\n",g_blink_brightness);
	}
}

void  tinno_max77660_get_rg_blink_offms(unsigned long offms){
	if(g_rg_blink_offms !=offms){
		g_rg_blink_offms= offms;
		dev_dbg(g_max77660_dev,"Magnum g_rg_blink_offms== %d\n",g_rg_blink_offms);
	}
}

void  tinno_max77660_get_rg_blink_onms(unsigned long onms){
	if(g_rg_blink_onms !=onms){
		g_rg_blink_onms= onms;
		dev_dbg(g_max77660_dev,"Magnum g_rg_blink_onms== %d\n",g_rg_blink_onms);
	}
}


#ifdef CONFIG_PM
/*	Magnum 2014-1-13
***   when suspend, ap enter into LP0, the blink timer disabled,so leds don't blink, 
***   so we should blink red or green led through writing register to pmic.
***/
int tinno_max77660_leds_suspend()
{
	//printk("Magnum %s() \n",__func__);
	set_rg_leds_brt(g_blink_brightness);
	max77660_led_set_hardware_blink(g_rg_blink_onms, g_rg_blink_offms);
	return 0;
}

int tinno_max77660_leds_resume()
{
	//printk("Magnum %s() \n",__func__);
	max77660_led_one_shot_mode();
	return 0;
}
static int max77660_leds_suspend(struct platform_device *ndev, pm_message_t state)
{
	//printk("Magnum %s() \n",__func__);
	led_classdev_suspend(&max77660_rg_led);
	max77660_rg_led.brightness = LED_OFF;
	led_classdev_suspend(&max77660_button1_led);
	max77660_button1_led.brightness = LED_OFF;
	
	if(g_blink_brightness > LED_OFF && g_rg_blink_onms > 0 && g_rg_blink_offms > 0)
	{
		set_rg_leds_brt(g_blink_brightness);
		max77660_led_set_hardware_blink(g_rg_blink_onms, g_rg_blink_offms);
	}
	return 0;
}

static int max77660_leds_resume(struct platform_device *ndev)
{
	//printk("Magnum %s() \n",__func__);
	led_classdev_resume(&max77660_rg_led);
	led_classdev_resume(&max77660_button1_led);

	//not needed ,just for ensure
	set_rg_leds_brt(LED_OFF);
	max77660_led_one_shot_mode();
	return 0;
}

#endif /* CONFIG_PM */


static int __devinit max77660_leds_probe(struct platform_device *pdev)
{
	struct max77660_leds *leds;
	struct max77660_platform_data *pdata;
	int ret = 0;
	
	dev_dbg(&pdev->dev,"Magnum max77660_leds_probe \n");
	pdata = dev_get_platdata(pdev->dev.parent);
	if (!pdata) {
		dev_err(&pdev->dev, "No Platform data\n");
		return -EINVAL;
	}

	leds = devm_kzalloc(&pdev->dev, sizeof(*leds), GFP_KERNEL);
	if (!leds) {
		dev_err(&pdev->dev, "Memory allocation failed for leds\n");
		return -ENOMEM;
	}

	leds->dev = &pdev->dev;
	leds->parent = pdev->dev.parent;
	dev_set_drvdata(&pdev->dev, leds);

	if (pdata->led_disable)
		ret = max77660_disable_leds(leds);

	ret = max77660_add_attributes(leds->dev, max77660_leds_attrs);
	if (ret < 0) {
		dev_err(&pdev->dev, "Create max77660 leds attributes failed, %d!\n", ret);
		return ret;
	}

	g_max77660_dev =&pdev->dev;
	

	ret = led_classdev_register(&pdev->dev, &max77660_rg_led);
	if (ret < 0) {
		dev_err(&pdev->dev, "Create max77660_red_led attributes failed, %d!\n", ret);
		return ret;
	}

	ret = led_classdev_register(&pdev->dev, &max77660_button1_led);
	if (ret < 0) {
		dev_err(&pdev->dev, "Create max77660_button_led attributes failed, %d!\n", ret);
		return ret;
	}  

	mutex_init(&max77660_leds_mutex);
	//spin_lock_init(&max77660_leds_lock);

        INIT_WORK(&set_rg_leds_work,set_rg_leds_work_func);
	INIT_WORK(&set_button_leds_work,set_button_leds_work_func);

	return ret;
}

static int __devexit max77660_leds_remove(struct platform_device *pdev)
{
	struct max77660_leds *leds = platform_get_drvdata(pdev);

	if (!leds)
		return 0;
	max77660_remove_attributes(&pdev->dev, max77660_leds_attrs);
	devm_kfree(&pdev->dev, leds);
	return 0;
}

static struct platform_driver max77660_leds_driver = {
	.driver = {
		.name = "max77660-leds",
		.owner = THIS_MODULE,
	},
	.probe = max77660_leds_probe,
	#ifdef CONFIG_PM
	.suspend = max77660_leds_suspend,
	.resume = max77660_leds_resume,
	#endif
	.remove = __devexit_p(max77660_leds_remove),
};

module_platform_driver(max77660_leds_driver);

MODULE_DESCRIPTION("max77660 LEDs driver");
MODULE_AUTHOR("Laxman Dewangan<ldewangan@nvidia.com>");
MODULE_ALIAS("platform:max77660-leds");
MODULE_LICENSE("GPL v2");
