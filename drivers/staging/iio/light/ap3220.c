/*
 * A iio driver for the light sensor AP3220.
 *
 * IIO Light driver for monitoring ambient light intensity in lux and proximity
 * ir.
 *
 * Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/regulator/consumer.h>
#include "../iio.h"
#include "../sysfs.h"

/*ap3220 als/ps sensor register related macro*/
#define AP3220_REG_SYS_CONF		0x00
#define AP3220_REG_SYS_ISTATUS		0x01
#define AP3220_REG_SYS_ICLEAR_M		0x02
#define AP3220_REG_SYS_IR_DATA_LOW	0x0A
#define AP3220_REG_SYS_IR_DATA_HIGH	0x0B
#define AP3220_REG_SYS_ALS_DATA_LOW	0x0C
#define AP3220_REG_SYS_ALS_DATA_HIGH	0x0D
#define AP3220_REG_SYS_PS_DATA_LOW	0x0E
#define AP3220_REG_SYS_PS_DATA_HIGH	0x0F
#define	AP3220_SYSTEM_PS_IR_OVERFLOW	0x40
#define	AP3220_SYSTEM_PS_OBJ_CLOSE	0x80


#define AP3220_REG_ALS_CONF		0x10
#define AP3220_REG_ALS_CAL		0x19
#define AP3220_REG_ALS_THDL_L		0x1A
#define AP3220_REG_ALS_THDL_H		0x1B
#define AP3220_REG_ALS_THDH_L		0x1C
#define AP3220_REG_ALS_THDH_H		0x1D

#define AP3220_REG_PS_CONF		0x20
#define AP3220_REG_PS_LED		0x21
#define AP3220_REG_PS_INT_FORM		0x22
#define AP3220_REG_PS_MEAN_TIME		0x23
#define AP3220_REG_PS_WAIT_TIME		0x24
#define AP3220_REG_PS_CAL_L		0x28
#define AP3220_REG_PS_CAL_H		0x29
#define AP3220_REG_PS_THDL_L		0x2A
#define AP3220_REG_PS_THDL_H		0x2B
#define AP3220_REG_PS_THDH_L		0x2C
#define AP3220_REG_PS_THDH_H		0x2D

//SYSTEM MODE
#define	AP3220_SYSTEM_DEVICE_DOWN	0x00
#define	AP3220_SYSTEM_PS_ENABLE		0x02
#define	AP3220_SYSTEM_ALS_ENABLE	0x01
#define	AP3220_SYSTEM_DEVICE_MASK	0x03

#define	AP3220_SYSTEM_ALS_INT_TRIGGER	0x01
#define	AP3220_SYSTEM_PS_INT_TRIGGER	0x02

#define	AP3220_SYSTEM_PS_OBJ		0x80
#define	AP3220_SYSTEM_PS_IR_OF		0x40
#define	AP3220_SYSTEM_PS_DATA_MASK_L	0x0F
#define	AP3220_SYSTEM_PS_DATA_MASK_H	0x3F

#define	AP3220_ALS_RANGE_MASK		0x30
#define	AP3220_ALS_RANGE_SHIFT		4
#define	AP3220_ALS_PERSIST_MASK		0x0F
#define	AP3220_ALS_PERSIST_SHIFT	0x00
//ALS Setting
#define AP3220_ALS_SETTING_RANGE_65536	0x00
#define AP3220_ALS_SETTING_RANGE_16383	0x01
#define AP3220_ALS_SETTING_RANGE_4095	0x02
#define AP3220_ALS_SETTING_RANGE_1023	0x03
#define AP3220_ALS_SETTING_PERSIST_1	0x00
#define AP3220_ALS_SETTING_PERSIST_4	0x01
#define AP3220_ALS_SETTING_PERSIST_8	0x02
#define AP3220_ALS_SETTING_PERSIST_12	0x03
#define AP3220_ALS_SETTING_PERSIST_16	0x04
#define AP3220_ALS_SETTING_PERSIST_60	0x0F


#define AP3220_PS_INTEGEATED_TIME_MASK	0xF0
#define AP3220_PS_INTEGEATED_TIME_SHIFT	4
#define AP3220_PS_GAIN_MASK		0x0C
#define AP3220_PS_GAIN_SHIFT		2
#define AP3220_PS_PERSIST_MASK		0x03
#define AP3220_PS_PERSIST_SHIFT		0x00
#define AP3220_PS_LED_PULSE_MASK	0x30
#define AP3220_PS_LED_PULSE_SHIFT	4
#define AP3220_PS_LED_RATIO_MASK	0x03
#define AP3220_PS_LED_RATIO_SHIFT	0
//PS Setting
#define AP3220_PS_SETTING_INTG_TIME_1	0x00
#define AP3220_PS_SETTING_INTG_TIME_2	0x01
#define AP3220_PS_SETTING_INTG_TIME_3	0x02
#define AP3220_PS_SETTING_INTG_TIME_4	0x03
#define AP3220_PS_SETTING_INTG_TIME_8	0x07
#define AP3220_PS_SETTING_INTG_TIME_16	0x0F
#define AP3220_PS_SETTING_GAIN_1	0x00
#define AP3220_PS_SETTING_GAIN_2	0x01
#define AP3220_PS_SETTING_GAIN_4	0x02
#define AP3220_PS_SETTING_GAIN_8	0x03
#define AP3220_PS_SETTING_PERSIST_1	0x00
#define AP3220_PS_SETTING_PERSIST_2	0x01
#define AP3220_PS_SETTING_PERSIST_4	0x02
#define AP3220_PS_SETTING_PERSIST_5	0x03
#define AP3220_PS_SETTING_LED_PULSE_0	0x00
#define AP3220_PS_SETTING_LED_PULSE_1	0x01
#define AP3220_PS_SETTING_LED_PULSE_2	0x02
#define AP3220_PS_SETTING_LED_PULSE_3	0x03
#define AP3220_PS_SETTING_LED_RATIO_16	0x00
#define AP3220_PS_SETTING_LED_RATIO_33	0x01
#define AP3220_PS_SETTING_LED_RATIO_66	0x02
#define AP3220_PS_SETTING_LED_RATIO_100	0x03
#define AP3220_PS_SETTING_PS_ALGO_ZONE	0x00
#define AP3220_PS_SETTING_PS_ALGO_HYST	0x01
#define AP3220_PS_SETTING_PS_MEAN_12	0x00
#define AP3220_PS_SETTING_PS_MEAN_25	0x01
#define AP3220_PS_SETTING_PS_MEAN_37	0x02
#define AP3220_PS_SETTING_PS_MEAN_50	0x03

struct alsps_cfg {
  u16	ps_threshold;
  u16	ps_threshold_high;
  u16	ps_threshold_low;
  u16	als_threshold_high;
  u16	als_threshold_low;
};

static struct alsps_cfg default_cfg = {
    .ps_threshold = 2,	//3,
    .ps_threshold_high = 0x1F,
    .ps_threshold_low = 0x10,		//0x18
    .als_threshold_high = 0xFFFF,
    .als_threshold_low = 0,
};


/*AP3220 related driver tag macro*/
#define AP3220_SUCCESS			0
#define AP3220_ERR_I2C			-1

#define POWER_ON_DELAY		20 /* 20ms */

/*
#define AP3220_SYSFS_SHOW(en, reg_addr, nbytes)			 \
	do {								 \
		int ret;						 \
		int value;						 \
		struct iio_dev *indio_dev = dev_get_drvdata(dev);	 \
		struct ap3220_chip *chip = iio_priv(indio_dev);	 \
		if (!en)						 \
			return sprintf(buf, "-1");			 \
		mutex_lock(&chip->lock);				 \
		ret = ap3220_read(chip, &value, reg_addr, nbytes);	 \
		if (ret < 0) {						 \
			mutex_unlock(&chip->lock);			 \
			return sprintf(buf, "-1");			 \
		}							 \
		mutex_unlock(&chip->lock);				 \
		return sprintf(buf, "%d", value);			 \
	} while (0);							 \
*/
#define CLEAR_ENABLED		(chip->using_als)

#define PROXIMITY_ENABLED	(chip->using_proximity)


enum {
	CHIP = 0,
	LED
};

static struct class *sensor_class;

static int g_last_ps_value;

struct ap3220_chip {
	struct i2c_client	*client;
	struct mutex		lock;

	struct regulator	*supply[2];
	bool			power_utilization[2];

	bool			using_als;
	bool			using_proximity;

	bool			is_standby;
	int			shutdown_complete;
};

static int ap3220_read(struct ap3220_chip *chip, u8 reg_addr, u8 *rval)
{
	u8 val[2];
	int ret;

	if (chip->supply[CHIP] && !regulator_is_enabled(chip->supply[CHIP]))
		return -EINVAL;

	if (chip->shutdown_complete)
		return -EINVAL;

	ret = i2c_smbus_read_i2c_block_data(chip->client, reg_addr,
						1, val);

	if (ret != 1) {
		dev_err(&chip->client->dev, "[AP3220] i2c_read_failed" \
			"in func: %s\n", __func__);
		if (ret < 0)
			return ret;
		return -EINVAL;
	}

	*rval = val[0];
	return 0;
}

/*
static int ap3220_read_16(struct ap3220_chip *chip, u8 reg_addr, u16 *rval,
				int nbytes)
{
	u8 val[2];
	int ret;

	if (chip->supply[CHIP] && !regulator_is_enabled(chip->supply[CHIP]))
		return -EINVAL;

	if (chip->shutdown_complete)
		return -EINVAL;

	ret = i2c_smbus_read_i2c_block_data(chip->client, reg_addr,
						nbytes, val);

	if (ret != nbytes) {
		dev_err(&chip->client->dev, "[AP3220] i2c_read_failed" \
			"in func: %s\n", __func__);
		if (ret < 0)
			return ret;
		return -EINVAL;
	}

	*rval = val[0];
	if (nbytes == 2)
		*rval = ((*rval) << 8) | val[1];
	return 0;
}
*/

static int ap3220_write(struct ap3220_chip *chip, u8 reg_addr, u8 val)
{
	int ret;

	if (chip->supply[CHIP] && !regulator_is_enabled(chip->supply[CHIP]))
		return -EINVAL;

	if (chip->shutdown_complete)
		return -EINVAL;

	ret = i2c_smbus_write_byte_data(chip->client, reg_addr, val);
	if (ret < 0) {
		dev_err(&chip->client->dev, "[AP3220] i2c_write_failed" \
			"in func: %s\n", __func__);
	}
	return ret;
}

/*--------------------------------------------------------------------------------*/
static int ap3220_chip_init(struct ap3220_chip *chip)
{
	u8 data, data2, data3;
	int res = 0;
	u16 tmp_data;
//ALS Range, 0 ~ 65536
//ALS PERSIST	
	data = (AP3220_ALS_SETTING_RANGE_16383 << AP3220_ALS_RANGE_SHIFT) & AP3220_ALS_RANGE_MASK;
	data2 = (AP3220_ALS_SETTING_PERSIST_1 << AP3220_ALS_PERSIST_SHIFT) & AP3220_ALS_PERSIST_MASK;
	data |= data2;
	res = ap3220_write(chip, AP3220_REG_ALS_CONF,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//Ivan FIXME tunnable value to be confirmed by HW
//ALS Calibration 
	data = 0x80;				//0x40/64 = 1	
	res = ap3220_write(chip, AP3220_REG_ALS_CAL,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	
//Ivan Set Low threshold to 0; try to disable ALS interrupt	
//ALS Low Threshold L byte
	tmp_data = default_cfg.als_threshold_low;
	data = tmp_data & 0x00FF;		// Read from cust_alsps.c
//	printk("Ivan ALS Low Threshold L = %x\n",data);		
	res = ap3220_write(chip, AP3220_REG_ALS_THDL_L,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//ALS Low Threshold H byte
	data = tmp_data >> 8;		// Read from cust_alsps.c
//	printk("Ivan ALS Low Threshold H = %x\n",data);			
	res = ap3220_write(chip, AP3220_REG_ALS_THDL_H,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//Ivan Set High threshold to 65536; try to disable ALS interrupt
//ALS High Threshold L byte
	tmp_data = default_cfg.als_threshold_high;
	data = tmp_data & 0x00FF;		// Read from cust_alsps.c
//	printk("Ivan ALS High Threshold L = %x\n",data);			
	res = ap3220_write(chip, AP3220_REG_ALS_THDH_L,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//ALS High Threshold H byte
	data = tmp_data >> 8;		// Read from cust_alsps.c
//	printk("Ivan ALS High Threshold H = %x\n",data);				
	res = ap3220_write(chip, AP3220_REG_ALS_THDH_L,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	
//PS Configuration 
	data = (AP3220_PS_SETTING_INTG_TIME_2 << AP3220_PS_INTEGEATED_TIME_SHIFT) & AP3220_PS_INTEGEATED_TIME_MASK;
	data2 = (AP3220_PS_SETTING_GAIN_1 << AP3220_PS_GAIN_SHIFT) & AP3220_PS_GAIN_MASK;
	data3 = (AP3220_PS_SETTING_PERSIST_4 << AP3220_PS_PERSIST_SHIFT) & AP3220_PS_PERSIST_MASK;	
	data |= data2;
	data |= data3;
	res = ap3220_write(chip, AP3220_REG_PS_CONF,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//PS LED Control
//	data = (AP3220_PS_SETTING_LED_PULSE_1 << AP3220_PS_LED_PULSE_SHIFT) & AP3220_PS_LED_PULSE_MASK;
//	data2 = (AP3220_PS_SETTING_LED_RATIO_66 << AP3220_PS_LED_RATIO_SHIFT) & AP3220_PS_LED_RATIO_MASK;
	data = (AP3220_PS_SETTING_LED_PULSE_2 << AP3220_PS_LED_PULSE_SHIFT) & AP3220_PS_LED_PULSE_MASK;
	data2 = (AP3220_PS_SETTING_LED_RATIO_33 << AP3220_PS_LED_RATIO_SHIFT) & AP3220_PS_LED_RATIO_MASK;
	
	data |= data2;
	res = ap3220_write(chip, AP3220_REG_PS_LED,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//PS INT MODE
	data = AP3220_PS_SETTING_PS_ALGO_HYST;
	res = ap3220_write(chip, AP3220_REG_PS_INT_FORM,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//PS MEAN TIME
	data = AP3220_PS_SETTING_PS_MEAN_12;
	res = ap3220_write(chip, AP3220_REG_PS_MEAN_TIME,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS LED WAITING
	data = 0;		// 0 = no waiting; 1 = 1 mean time, etc
	res = ap3220_write(chip, AP3220_REG_PS_WAIT_TIME,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//IVAN FIXME value to be confirmed with HW
//PS Calibration L 
	data = 0;		// 0 = no waiting; 1 = 1 mean time, etc
	res = ap3220_write(chip, AP3220_REG_PS_CAL_L,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
//PS Calibration H	
	data = 0;		// 0 = no waiting; 1 = 1 mean time, etc
	res = ap3220_write(chip, AP3220_REG_PS_CAL_H,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS Low Threshold L 
	tmp_data = default_cfg.ps_threshold_low;
	data = tmp_data & 0x0003;		// Read from cust_alsps.c
//	printk("Ivan PS Low Threshold L = %x\n",data);	
	res = ap3220_write(chip, AP3220_REG_PS_THDL_L,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS Low Threshold H
	data = tmp_data >> 2;		// Read from cust_alsps.c
//	printk("Ivan PS Low Threshold H = %x\n",data);		
	res = ap3220_write(chip, AP3220_REG_PS_THDL_H,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS High Threshold L 
	tmp_data = default_cfg.ps_threshold_high;
	data = tmp_data & 0x0003;	// Read from cust_alsps.c
//	printk("Ivan PS High Threshold L = %x\n",data);	
	res = ap3220_write(chip, AP3220_REG_PS_THDH_L,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//PS High Threshold H
	data = tmp_data >> 2;		// Read from cust_alsps.c
//	printk("Ivan PS High Threshold H = %x\n",data);		
	res = ap3220_write(chip, AP3220_REG_PS_THDH_H,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
//DEVICE POWER DOWN
	data = 0;
	res = ap3220_write(chip, AP3220_REG_SYS_CONF,data);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto EXIT_ERR;
	}	
				
	return AP3220_SUCCESS;
	
EXIT_ERR:
	printk("ap3220_chip_init ERROR: %d\n", res);
	return res;
}

/* assumes chip is power on */
/*
static void ap3220_standby(struct ap3220_chip *chip, bool shutdown)
{
	int ret = 0;

	if (chip->is_standby == shutdown)
		return;

	if (shutdown == chip->power_utilization[CHIP])
		return;

	if (shutdown == false) {
		ret = ap3220_write(chip, MAX_SHDN_DISABLE,
					INT_STATUS_REG_ADDR);
		if (!ret)
			chip->is_standby = false;
	} else {
		ret = ap3220_write(chip, MAX_SHDN_ENABLE,
					INT_STATUS_REG_ADDR);
		if (!ret)
			chip->is_standby = true;
	}
}

static bool set_main_conf(struct ap3220_chip *chip, int mode)
{
	return ap3220_write(chip, mode << MODE_SHIFT,
				MAIN_CONF_REG_ADDR) == 0;
}
*/

/* current is in mA */
static bool set_ps_sensor_enable(struct ap3220_chip *chip, int cur)
{
	int res;
	u8 databuf[1];    
	if (!chip->supply[LED])
		goto finish;

	if (cur)
	{
	    res = ap3220_read(chip, AP3220_REG_SYS_CONF,&databuf[0]);
	    if(res < 0)
	    {
		    printk("set_ps_sensor_enable i2c_master_read function err\n");
		    goto ENABLE_PS_EXIT_ERR;
	    }			
	    databuf[0] |= AP3220_SYSTEM_PS_ENABLE;
	    databuf[0] &= AP3220_SYSTEM_DEVICE_MASK;
	    res = ap3220_write(chip, AP3220_REG_SYS_CONF,databuf[0]);
	    if(res < 0)
	    {
		    printk("set_ps_sensor_enable i2c_master_send function err\n");
		    goto ENABLE_PS_EXIT_ERR;
	    }
	}
	else
	{
	    res = ap3220_read(chip, AP3220_REG_SYS_CONF,&databuf[0]);
	    if(res < 0)
	    {
		    printk("set_ps_sensor_enable i2c_master_read function err\n");
		    goto ENABLE_PS_EXIT_ERR;
	    }			
	    databuf[0] &= ~AP3220_SYSTEM_PS_ENABLE;
	    databuf[0] &= AP3220_SYSTEM_DEVICE_MASK;			
	    res = ap3220_write(chip, AP3220_REG_SYS_CONF,databuf[0]);
	    if(res < 0)
	    {
		    printk("set_ps_sensor_enable i2c_master_send function err\n");
		    goto ENABLE_PS_EXIT_ERR;
	    }	    
	}
finish:
    return 1;
    
ENABLE_PS_EXIT_ERR:
    return 0;
}


/* current is in mA */
static bool set_als_sensor_enable(struct ap3220_chip *chip, int cur)
{
	int res;
	u8 databuf[1];    
	
	printk("set_als_sensor_enable 1\n");
	
	if (!chip->supply[LED])
		goto finish;

	if (cur)
	{
	    printk("set_als_sensor_enable 2\n");
	    res = ap3220_read(chip, AP3220_REG_SYS_CONF,&databuf[0]);
	    if(res < 0)
	    {
		    printk("set_als_sensor_enable i2c_master_read function err\n");
		    goto ENABLE_ALS_EXIT_ERR;
	    }			
	    databuf[0] |= AP3220_SYSTEM_ALS_ENABLE;
	    databuf[0] &= AP3220_SYSTEM_DEVICE_MASK;
	    res = ap3220_write(chip, AP3220_REG_SYS_CONF,databuf[0]);
	    if(res < 0)
	    {
		    printk("set_als_sensor_enable i2c_master_send function err\n");
		    goto ENABLE_ALS_EXIT_ERR;
	    }
	}
	else
	{
	    res = ap3220_read(chip, AP3220_REG_SYS_CONF,&databuf[0]);
	    if(res < 0)
	    {
		    printk("set_als_sensor_enable i2c_master_read function err\n");
		    goto ENABLE_ALS_EXIT_ERR;
	    }			
	    databuf[0] &= ~AP3220_SYSTEM_ALS_ENABLE;
	    databuf[0] &= AP3220_SYSTEM_DEVICE_MASK;			
	    res = ap3220_write(chip, AP3220_REG_SYS_CONF,databuf[0]);
	    printk("set_als_sensor Disable 2\n");
	    
	    if(res < 0)
	    {
		    printk("set_als_sensor_enable i2c_master_send function err\n");
		    goto ENABLE_ALS_EXIT_ERR;
	    }	    
	}
finish:
    return 1;
    
ENABLE_ALS_EXIT_ERR:
    return 0;
}

static bool ap3220_power(struct ap3220_chip *chip, int power_on)
{
	int was_regulator_already_on = false;
	
	printk("Ivan ap3220_power power_on = %d\n",power_on);

	if (power_on && chip->power_utilization[CHIP])
		return true;

	if (power_on) {
		if (chip->supply[CHIP]) {
			was_regulator_already_on =
				regulator_is_enabled(chip->supply[CHIP]);
			if (regulator_enable(chip->supply[CHIP]))
				return false;
			if (regulator_enable(chip->supply[LED]))
				return false;
			if (!was_regulator_already_on)
				msleep(POWER_ON_DELAY);
		}
		chip->power_utilization[CHIP] = 1;

		/* wakeup if still in shutdown state */
		ap3220_chip_init(chip);
//Ivan		ap3220_standby(chip, false);
		
		return true;
	}


	/* power off request */
	/* disable the power source as chip doesnot need it anymore */
	if (chip->supply[CHIP] && chip->power_utilization[CHIP])
	{
	
	    regulator_disable(chip->supply[CHIP]);
	    regulator_disable(chip->supply[LED]);
	}
	chip->power_utilization[CHIP] = 0;
	/* chip doesnt utilize power now, power being
	 * supplied is being wasted, so put the device to standby
	 * to reduce wastage */
//Ivan	ap3220_standby(chip, true);

	return true;
}

/* assumes power is on */
/*
static bool ap3220_restore_state(struct ap3220_chip *chip)
{
	int ret;

	if (PROXIMITY_ENABLED)
		ret = set_led_drive_strength(chip, LED_DRV_STRENGTH);
	else
		ret = set_led_drive_strength(chip, 0);

	if (!ret)
		return false;

	switch ((CLEAR_ENABLED << 1) | PROXIMITY_ENABLED) {
	case 0:
		ret = ap3220_power(chip, false);
		break;
	case 1:
		ret = set_main_conf(chip, MODE_PROX_ONLY);
		break;
	case 2:
		ret = set_main_conf(chip, MODE_CLEAR_ONLY);
		break;
	case 3:
		ret = set_main_conf(chip, MODE_CLEAR_PROX);
		break;
	}

	return false;
}
*/

int ap3220_read_ps(struct ap3220_chip *chip, u16 *data)
{
	int res;
	u8 databuf[2];
	u16 ps_l, ps_h;
	u16 ps_val = 0;
	
//	APS_FUN(f);
	res = ap3220_read(chip, AP3220_REG_SYS_PS_DATA_LOW,&databuf[0]);
	res = ap3220_read(chip, AP3220_REG_SYS_PS_DATA_HIGH,&databuf[1]);
	if(res < 0)
	{
		printk("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}
	
//	APS_LOG("AP3220_REG_PS_DATA value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
//	*data = databuf;
//Ivan check valid data before pass the value to upper layer
	if (!(databuf[0] & AP3220_SYSTEM_PS_IR_OVERFLOW) && !(databuf[1] & AP3220_SYSTEM_PS_IR_OVERFLOW))
	{
	  ps_val = ((databuf[1]<<8)|databuf[0]);
	  
	  ps_l = ps_val & 0x000F;
	  ps_h = ps_val >> 4;
	  ps_h = ps_h & 0x03F0;
	  *data = ps_l | ps_h;	  
	  if (databuf[0] & AP3220_SYSTEM_PS_OBJ_CLOSE)
	    *data |= 0x8000;
	  
	  g_last_ps_value = *data;
	}
	else
	  //*data = g_last_ps_value;
	  *data = 0;  //far
//Ivan
//	printk("Ivan ap3220_read_ps raw = %x, ps_val = %x \n",ps_val, *data);
	
	return 0;
READ_PS_EXIT_ERR:
	return res;
}

/********************************************************************/
long ap3220_read_als(struct ap3220_chip *chip, u16 *data)
{
	long res;
	u8 databuf[2];
//	APS_FUN(f);
	
	res = ap3220_read(chip, AP3220_REG_SYS_ALS_DATA_LOW,&databuf[0]);
	res = ap3220_read(chip, AP3220_REG_SYS_ALS_DATA_HIGH,&databuf[1]);
	if(res < 0)
	{
		printk("ap3220_read_als i2c_master_send function err\n");
		goto READ_ALS_EXIT_ERR;
	}
	
//	APS_LOG("AP3220_REG_ALS_DATA value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	*data = ((databuf[1]<<8)|databuf[0]);
//	printk("Ivan ap3220_read_als value = %d\n",*data);
	
	return 0;
READ_ALS_EXIT_ERR:
	return res;
}

/* sysfs name begin */
static ssize_t show_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ap3220_chip *chip = iio_priv(indio_dev);
	return sprintf(buf, "%s\n", chip->client->name);
}
/* sysfs name end */

/* amb clear begin */
static ssize_t show_amb_clear_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ap3220_chip *chip = iio_priv(indio_dev);
	u16 als_val;
	int res;
	
	res = ap3220_read_als(chip, &als_val);
	if (res < 0)
	    return sprintf(buf, "-1");
	else
	    return sprintf(buf, "%d", als_val);
	
//	AP3220_SYSFS_SHOW(CLEAR_ENABLED, AMB_CLEAR_HIGH_ADDR, 2);
}

static ssize_t amb_clear_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 lval;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ap3220_chip *chip = iio_priv(indio_dev);
	    

	if (kstrtou32(buf, 10, &lval))
		return -EINVAL;

	printk("Ivan amb_clear_enable = %d!\n",lval);

	if (lval && (lval != 1))
		return -EINVAL;

	if (lval == chip->using_als)
		return count;

	mutex_lock(&chip->lock);

	if (lval) {
		if (!ap3220_power(chip, true))
			goto fail;

		if (!set_als_sensor_enable(chip, 1))
			goto fail;
		
//		printk("Ivan amb_clear_enable OK!\n");
		
//		if (!PROXIMITY_ENABLED)
//			goto success;

		/* if clear not enabled and LED enabled
		 * change the mode to CLEAR+LED enabled*/
//		if (PROXIMITY_ENABLED &&
//				set_main_conf(chip, MODE_CLEAR_PROX))
//			goto success;
		/* CLEAR channel remains intact due to lost communication */
//		goto fail;
	} else {
		if (!set_als_sensor_enable(chip, 0))
			goto fail;

		if (!PROXIMITY_ENABLED && ap3220_power(chip, false))
		{
//			printk("Ivan amb_clear Disable OK!\n");
			goto success;
		}
	}

success:
	chip->using_als = lval;
	mutex_unlock(&chip->lock);
	return count;
fail:
	mutex_unlock(&chip->lock);
	return -EBUSY;
}
/* amb clear end */

/* amb LED begin */
static ssize_t show_prox_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ap3220_chip *chip = iio_priv(indio_dev);
	u16 ps_val;
	int res;
	
	res = ap3220_read_ps(chip, &ps_val);
	if (res < 0)
	    return sprintf(buf, "-1");
	else
	    return sprintf(buf, "%d", ps_val);
	    
//	AP3220_SYSFS_SHOW(PROXIMITY_ENABLED, PROX_HIGH_ADDR, 2);
}

static ssize_t prox_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 lval;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ap3220_chip *chip = iio_priv(indio_dev);

	if (kstrtou32(buf, 10, &lval))
		return -EINVAL;

	if (lval && (lval != 1))
		return -EINVAL;

	if (lval == PROXIMITY_ENABLED)
		return count;
	
	printk("Ivan prox_enable = %d!\n",lval);

	mutex_lock(&chip->lock);
	if (lval) {
		if (!ap3220_power(chip, true))
			goto fail;

		if (!set_ps_sensor_enable(chip, 1))
			goto fail;
/*
		if (CLEAR_ENABLED && set_main_conf(chip, MODE_CLEAR_PROX))
			goto success;

		if (!CLEAR_ENABLED && set_main_conf(chip, MODE_PROX_ONLY))
			goto success;
*/
	} else {
		if (!set_ps_sensor_enable(chip, 0))
			goto fail;

		/* power off if no other channel is active */
		if (!CLEAR_ENABLED && ap3220_power(chip, false))
			goto success;
	}

success:
	chip->using_proximity = lval;
	mutex_unlock(&chip->lock);
	return count;
fail:
	mutex_unlock(&chip->lock);
	return -EBUSY;
}
/* amb LED end */

static IIO_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);
static IIO_DEVICE_ATTR(amb_clear, S_IRUGO | S_IWUSR, show_amb_clear_value,
			amb_clear_enable, 0);
static IIO_DEVICE_ATTR(proximity, S_IRUGO | S_IWUSR, show_prox_value,
			prox_enable, 0);

/* sysfs attr */
static struct attribute *ap3220_iio_attr[] = {
	&iio_dev_attr_name.dev_attr.attr,
	&iio_dev_attr_amb_clear.dev_attr.attr,
	&iio_dev_attr_proximity.dev_attr.attr,
	NULL
};

static const struct attribute_group ap3220_iio_attr_group = {
	.attrs = ap3220_iio_attr,
};

static const struct iio_info ap3220_iio_info = {
	.attrs = &ap3220_iio_attr_group,
	.driver_module = THIS_MODULE,
};

static int __devinit ap3220_sysfs_init(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct device *class_device;

	sensor_class = class_create(THIS_MODULE, "sensors");
	if (!sensor_class) {
		dev_err(&client->dev, "create /sys/class/sensors fails\n");
		return -EINVAL;
	}

	class_device = device_create(sensor_class, &indio_dev->dev,
					0, NULL, "%s", "light");
	if (!class_device) {
		dev_err(&client->dev, "create ...sensors/light fails\n");
		class_destroy(sensor_class);
		return -EINVAL;
	}

	return 0;
}





static int __devinit ap3220_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ap3220_chip *chip;
	int err;
	
	printk("Ivan ap3220_probe \n");
	indio_dev = iio_allocate_device(sizeof(struct ap3220_chip));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "iio allocation fails\n");
		return -ENOMEM;
	}

	chip = iio_priv(indio_dev);

	i2c_set_clientdata(client, indio_dev);
	chip->client = client;
	mutex_init(&chip->lock);

	indio_dev->info = &ap3220_iio_info;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	err = iio_device_register(indio_dev);
	if (err | ap3220_sysfs_init(client)) {
		dev_err(&client->dev, "iio registration fails\n");
		device_destroy(sensor_class, 0);
		class_destroy(sensor_class);
		mutex_destroy(&chip->lock);
		iio_free_device(indio_dev);
		return err;
	}

	chip->supply[CHIP] = regulator_get(&client->dev, "vdd_prox");

	if (IS_ERR_OR_NULL(chip->supply[CHIP])) {
		dev_err(&client->dev, "could not get regulator\n"
				"assuming power supply is always on\n");
		kfree(chip->supply);
		chip->supply[CHIP] = NULL;
		chip->supply[LED] = NULL;
	} else {
		chip->supply[LED] = regulator_get(&client->dev, "vio_prox");

		if (IS_ERR_OR_NULL(chip->supply[LED])) {
			dev_err(&client->dev, "als_prox regulator not present\n"
					"proximity sensor may not work fine\n");
			chip->supply[LED] = NULL;
		}
	}

	mutex_lock(&chip->lock);
	ap3220_power(chip, false);
	mutex_unlock(&chip->lock);

	chip->using_als = false;
	chip->using_proximity = false;
	chip->shutdown_complete = 0;
	dev_info(&client->dev, "%s() success\n", __func__);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ap3220_suspend(struct device *dev)
{
	int ret = 0;
/* Keep suspend/resume disabled till LP0 is stable on ceres.
 * iio core adds NULL pm_ops if driver doesnt have them.
 * This causes crash during suspend/resume. Hence, instead
 * of removing them, make them "noop".
 *
 * struct iio_dev *indio_dev = i2c_get_clientdata(client);
 * struct ap3220_chip *chip = iio_priv(indio_dev);
 * mutex_lock(&chip->lock);
 * ap3220_power(chip, false);
 * mutex_unlock(&chip->lock);
 */
	return ret;
}

static int ap3220_resume(struct device *dev)
{
	int ret = 0;
/* struct iio_dev *indio_dev = i2c_get_clientdata(client);
 * struct ap3220_chip *chip = iio_priv(indio_dev);
 * mutex_lock(&chip->lock);
 * ap3220_power(chip, true);
 * ap3220_restore_state(chip);
 * mutex_unlock(&chip->lock);
 */
	return ret;
}

static SIMPLE_DEV_PM_OPS(ap3220_pm_ops, ap3220_suspend, ap3220_resume);
#define AP3220_PM_OPS (&ap3220_pm_ops)
#else
#define AP3220_PM_OPS NULL
#endif

static int __devexit ap3220_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ap3220_chip *chip = iio_priv(indio_dev);
	dev_dbg(&client->dev, "%s()\n", __func__);
	if (chip->supply[CHIP])
		regulator_put(chip->supply[CHIP]);
	mutex_destroy(&chip->lock);
	device_destroy(sensor_class, 0);
	class_destroy(sensor_class);
	iio_device_unregister(indio_dev);
	iio_free_device(indio_dev);
	return 0;
}

static void ap3220_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ap3220_chip *chip = iio_priv(indio_dev);
	mutex_lock(&chip->lock);
	if (chip->supply[CHIP])
		regulator_put(chip->supply[CHIP]);
	chip->shutdown_complete = 1;
	mutex_unlock(&chip->lock);
	mutex_destroy(&chip->lock);
	device_destroy(sensor_class, 0);
	class_destroy(sensor_class);
	iio_device_unregister(indio_dev);
	iio_free_device(indio_dev);
}

static const struct i2c_device_id ap3220_id[] = {
	{"ap3220", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ap3220_id);

static struct i2c_driver ap3220_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver  = {
		.name = "ap3220",
		.owner = THIS_MODULE,
		.pm = AP3220_PM_OPS,
	},
	.probe	 = ap3220_probe,
	.shutdown = ap3220_shutdown,
	.remove  = __devexit_p(ap3220_remove),
	.id_table = ap3220_id,
};

static int __init ap3220_init(void)
{
	return i2c_add_driver(&ap3220_driver);
}

static void __exit ap3220_exit(void)
{
	i2c_del_driver(&ap3220_driver);
}

module_init(ap3220_init);
module_exit(ap3220_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AP3220 Driver");
MODULE_AUTHOR("Sri Krishna chowdary <schowdary@nvidia.com>");
