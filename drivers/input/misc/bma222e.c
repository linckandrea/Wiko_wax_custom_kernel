/*
 * Copyright (c) 2011 Bosch Sensortec GmbH
 * Copyright (c) 2011 Unixphere
 *
 * This driver adds support for Bosch Sensortec's digital acceleration
 * sensors BMA222E and SMB380.
 * The SMB380 is fully compatible with BMA222E and only differs in packaging.
 *
 * The datasheet for the BMA222E chip can be found here:
 * http://www.bosch-sensortec.com/content/language1/downloads/BST-BMA222E-DS000-07.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/bma222e.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_INV_MPU
#include <linux/mpu.h>
#else
#include <linux/akm8963.h>
#endif

#define BMA_NAME			"bma222e"
#define BMA_POLL_DELAY_MS_DFLT		(200)
#define BMA_INPUT_DELAY_MS_MIN		(50)

#define BMA_DEBUG_ON	1

enum inv_accl_fs_e {
	INV_FS_02G = 0,
	INV_FS_04G,
	INV_FS_08G,
	INV_FS_16G,
	NUM_ACCL_FSR
};

#define ABSMAX_ACC_VAL		0x07F
#define ABSMIN_ACC_VAL		-(ABSMAX_ACC_VAL)

/* Each axis is represented by a 2-byte data word */
#define BMA222E_XYZ_DATA_SIZE	6
#define BMA222_AXIS_X          0
#define BMA222_AXIS_Y          1
#define BMA222_AXIS_Z          2
#define BMA222_AXES_NUM        3

/* Input poll interval in milliseconds */
#define BMA222E_POLL_INTERVAL	30
#define BMA222E_POLL_MAX		200
#define BMA222E_POLL_MIN		0

#define BMA222E_BW_MASK				0x1f
#define BMA222E_BW_200HZ				0x0d
#define BMA222E_BW_100HZ				0x0c
#define BMA222E_BW_50HZ				0x0b
#define BMA222E_BW_25HZ				0x0a


#define BMA222E_RANGE_MASK			0x0f
#define BMA222E_RANGE_2G			0x03
#define BMA222E_RANGE_4G			0x05
#define BMA222E_RANGE_8G			0x08

#define BMA222E_MODE_NORMAL	0
#define BMA222E_MODE_SLEEP	1
//#define BMA222E_MODE_WAKE_UP	3


#define BMA222_REG_DEVID	0x00
#if 0
/* Data register addresses */
#define BMA222E_DATA_0_REG	0x00
#define BMA222E_DATA_1_REG	0x01
#define BMA222E_DATA_2_REG	0x02

/* Control register addresses */
#define BMA222E_CTRL_0_REG	0x0A
#define BMA222E_CTRL_1_REG	0x0B
#define BMA222E_CTRL_2_REG	0x14
#define BMA222E_CTRL_3_REG	0x15

/* Configuration/Setting register addresses */
#define BMA222E_CFG_0_REG	0x0C
#define BMA222E_CFG_1_REG	0x0D
#define BMA222E_CFG_2_REG	0x0E
#define BMA222E_CFG_3_REG	0x0F
#define BMA222E_CFG_4_REG	0x10
#define BMA222E_CFG_5_REG	0x11
#endif

#define BMA222E_CHIP_ID		2
#define BMA222E_CHIP_ID_REG	0x00

#define BMA222E_ACC_X_LSB_REG	0x03

#define BMA222E_SLEEP_POS	7
#define BMA222E_SLEEP_MSK	0x80
#define BMA222E_SLEEP_REG	0x11

#define BMA222E_BANDWIDTH_POS	0
#define BMA222E_BANDWIDTH_MSK	0x1F
#define BMA222E_BANDWIDTH_REG	0x10

#define BMA222E_RANGE_POS	0
#define BMA222E_RANGE_MSK	0x0F
#define BMA222E_RANGE_REG	0x0F
/*
#define BMA222E_WAKE_UP_POS	7
#define BMA222E_WAKE_UP_MSK	0x80
#define BMA222E_WAKE_UP_REG	0x11
*/
#define BMA222E_SW_RES_POS	0
#define BMA222E_SW_RES_MSK	0xFF
#define BMA222E_SW_RES_REG	0x14


//Ivan not used at the moment
/* Any-motion interrupt register fields */
#define BMA222E_NOT_USE 		0x26		//Ivan high_th, write to it should have no effect...
#define BMA222E_ANY_MOTION_EN_POS	6
#define BMA222E_ANY_MOTION_EN_MSK	0x40
#define BMA222E_ANY_MOTION_EN_REG	BMA222E_NOT_USE

#define BMA222E_ANY_MOTION_DUR_POS	6
#define BMA222E_ANY_MOTION_DUR_MSK	0xC0
#define BMA222E_ANY_MOTION_DUR_REG	BMA222E_NOT_USE

#define BMA222E_ANY_MOTION_THRES_REG	BMA222E_NOT_USE

/* Advanced interrupt register fields */
#define BMA222E_ADV_INT_EN_POS		6
#define BMA222E_ADV_INT_EN_MSK		0x40
#define BMA222E_ADV_INT_EN_REG		BMA222E_NOT_USE

/* High-G interrupt register fields */
#define BMA222E_HIGH_G_EN_POS		1
#define BMA222E_HIGH_G_EN_MSK		0x02
#define BMA222E_HIGH_G_EN_REG		BMA222E_NOT_USE

#define BMA222E_HIGH_G_HYST_POS		3
#define BMA222E_HIGH_G_HYST_MSK		0x38
#define BMA222E_HIGH_G_HYST_REG		BMA222E_NOT_USE

#define BMA222E_HIGH_G_DUR_REG		BMA222E_NOT_USE
#define BMA222E_HIGH_G_THRES_REG	BMA222E_NOT_USE

/* Low-G interrupt register fields */
#define BMA222E_LOW_G_EN_POS		0
#define BMA222E_LOW_G_EN_MSK		0x01
#define BMA222E_LOW_G_EN_REG		BMA222E_NOT_USE

#define BMA222E_LOW_G_HYST_POS		0
#define BMA222E_LOW_G_HYST_MSK		0x07
#define BMA222E_LOW_G_HYST_REG		BMA222E_NOT_USE

#define BMA222E_LOW_G_DUR_REG		BMA222E_NOT_USE
#define BMA222E_LOW_G_THRES_REG		BMA222E_NOT_USE

#define C_MAX_HWMSEN_EVENT_NUM          4 

/*----------------------------------------------------------------------------*/
#if 0
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
#else
#define GSE_LOG(fmt, arg...) do {} while (0)
#define GSE_ERR(fmt, arg...) do {} while (0)
#define GSE_FUN(fmt, arg...) do {} while (0)
#endif

static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}
/*----------------------------------------------------------------------------*/

struct direction_convert {
    s8 sign[C_MAX_HWMSEN_EVENT_NUM];
    u8 map[C_MAX_HWMSEN_EVENT_NUM];
};

struct bma222e_data {
	struct i2c_client *client;
	struct input_polled_dev *input_polled;
	struct input_dev *input;
	struct regulator *vdd;  	
	struct input_dev *idev;
	struct workqueue_struct *wq;
	struct delayed_work dw;
	struct direction_convert   cvt;

	bool enable;			/* enable status */
	unsigned int poll_delay_us;	/* requested sampling delay (us) */
#ifdef CONFIG_INV_MPU
	struct mpu_platform_data pdata;
#else
	struct akm8963_platform_data pdata;
#endif
	unsigned int range_i;		/* max_range index */
	unsigned int dbg;		/* device id */

	u8 mode;
    /*misc*/
    struct data_resolution *reso;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s16                     cali_sw[BMA222_AXES_NUM+1];

    /*data*/
    s8                      offset[BMA222_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[BMA222_AXES_NUM+1];	
};

/*
 * The settings for the given range, bandwidth and interrupt features
 * are stated and verified by Bosch Sensortec where they are configured
 * to provide a generic sensitivity performance.
 */
static struct bma222e_cfg default_cfg __devinitdata = {
	.any_motion_int = 1,
	.hg_int = 1,
	.lg_int = 1,
	.any_motion_dur = 0,
	.any_motion_thres = 0,
	.hg_hyst = 0,
	.hg_dur = 150,
	.hg_thres = 160,
	.lg_hyst = 0,
	.lg_dur = 150,
	.lg_thres = 20,
	.range = BMA222E_RANGE_2G,
	.bandwidth = BMA222E_BW_25HZ
};

struct direction_convert map[] = {
    { { 1, 1, 1}, {0, 1, 2} },
    { {-1, 1, 1}, {1, 0, 2} },
    { {-1,-1, 1}, {0, 1, 2} },
    { { 1,-1, 1}, {1, 0, 2} },

    { {-1, 1,-1}, {0, 1, 2} },
    { { 1, 1,-1}, {1, 0, 2} },
    { { 1,-1,-1}, {0, 1, 2} },
    { {-1,-1,-1}, {1, 0, 2} },      

};
/*----------------------------------------------------------------------------*/
int direction_get_convert(int direction, struct direction_convert *cvt) 
{
    if (!cvt)
        return -EINVAL;
    else if (direction >= sizeof(map)/sizeof(map[0]))
        return -EINVAL;

    *cvt = map[direction];
    return 0;
}

static int bma222e_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;

	/* As per specification, disable irq in between register writes */
	if (client->irq)
		disable_irq_nosync(client->irq);

	ret = i2c_smbus_write_byte_data(client, reg, val);

	if (client->irq)
		enable_irq(client->irq);

	return ret;
}

static int bma222e_set_reg_bits(struct i2c_client *client,
					int val, int shift, u8 mask, u8 reg)
{
	int data;

	data = i2c_smbus_read_byte_data(client, reg);
	if (data < 0)
		return data;

	data = (data & ~mask) | ((val << shift) & mask);
	return bma222e_write_byte(client, reg, data);
}

/***********************************************************************************
 *   port from MTK function
 * *********************************************************************************/
/*----------------------------------------------------------------------------*/
static int BMA222_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);    
	databuf[0] = BMA222_REG_DEVID;    

	res = i2c_smbus_read_byte_data(client, 0);
	
	printk("Ivan BMA222_CheckDeviceID id = %x \n", res);
	
	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto exit_BMA222_CheckDeviceID;
	}
	
	udelay(200);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_BMA222_CheckDeviceID;
	}
	

	GSE_LOG("BMA222_CheckDeviceID %d done!\n ", databuf[0]);


exit_BMA222_CheckDeviceID:
	if (res <= 0)
	{
		GSE_ERR("BMA222_CheckDeviceID %d failt!\n ", -1);
		return -1;
	}
	mdelay(1);
	return 0;
}

/***************************************************************************************/


static int bma222e_set_mode(struct bma222e_data *bma222e, u8 mode)
{
	int error;
/*
	error = bma222e_set_reg_bits(bma222e->client, mode, BMA222E_WAKE_UP_POS,
				BMA222E_WAKE_UP_MSK, BMA222E_WAKE_UP_REG);
	if (error)
		return error;
*/

	error = bma222e_set_reg_bits(bma222e->client, mode, BMA222E_SLEEP_POS,
				BMA222E_SLEEP_MSK, BMA222E_SLEEP_REG);
	if (error)
	{
	    printk("Ivan bma222e_set_mode error!!! \n");
	    return error;
	}
	if (mode == BMA222E_MODE_NORMAL)
	{
	    msleep(2);
	    printk("Ivan queue_delayed_work!!! \n");	    
	    queue_delayed_work(bma222e->wq, &bma222e->dw,
				usecs_to_jiffies(bma222e->poll_delay_us));		
	}
	printk("Ivan bma222e_set_mode = %d!!! \n",mode);	    
	
	bma222e->mode = mode;
	return 0;
}

static int __devinit bma222e_soft_reset(struct bma222e_data *bma222e)
{
	int error;

	error = bma222e_set_reg_bits(bma222e->client, 0xB6, BMA222E_SW_RES_POS,
				BMA222E_SW_RES_MSK, BMA222E_SW_RES_REG);
	if (error)
		return error;

	msleep(2);
	return 0;
}

static int __devinit bma222e_set_range(struct bma222e_data *bma222e, u8 range)
{
	return bma222e_set_reg_bits(bma222e->client, range, BMA222E_RANGE_POS,
				BMA222E_RANGE_MSK, BMA222E_RANGE_REG);
}

static int __devinit bma222e_set_bandwidth(struct bma222e_data *bma222e, u8 bw)
{
	return bma222e_set_reg_bits(bma222e->client, bw, BMA222E_BANDWIDTH_POS,
				BMA222E_BANDWIDTH_MSK, BMA222E_BANDWIDTH_REG);
}

static int __devinit bma222e_set_low_g_interrupt(struct bma222e_data *bma222e,
					u8 enable, u8 hyst, u8 dur, u8 thres)
{
	int error;

	error = bma222e_set_reg_bits(bma222e->client, hyst,
				BMA222E_LOW_G_HYST_POS, BMA222E_LOW_G_HYST_MSK,
				BMA222E_LOW_G_HYST_REG);
	if (error)
		return error;

	error = bma222e_write_byte(bma222e->client, BMA222E_LOW_G_DUR_REG, dur);
	if (error)
		return error;

	error = bma222e_write_byte(bma222e->client, BMA222E_LOW_G_THRES_REG, thres);
	if (error)
		return error;

	return bma222e_set_reg_bits(bma222e->client, !!enable,
				BMA222E_LOW_G_EN_POS, BMA222E_LOW_G_EN_MSK,
				BMA222E_LOW_G_EN_REG);
}

static int __devinit bma222e_set_high_g_interrupt(struct bma222e_data *bma222e,
					u8 enable, u8 hyst, u8 dur, u8 thres)
{
	int error;

	error = bma222e_set_reg_bits(bma222e->client, hyst,
				BMA222E_HIGH_G_HYST_POS, BMA222E_HIGH_G_HYST_MSK,
				BMA222E_HIGH_G_HYST_REG);
	if (error)
		return error;

	error = bma222e_write_byte(bma222e->client,
				BMA222E_HIGH_G_DUR_REG, dur);
	if (error)
		return error;

	error = bma222e_write_byte(bma222e->client,
				BMA222E_HIGH_G_THRES_REG, thres);
	if (error)
		return error;

	return bma222e_set_reg_bits(bma222e->client, !!enable,
				BMA222E_HIGH_G_EN_POS, BMA222E_HIGH_G_EN_MSK,
				BMA222E_HIGH_G_EN_REG);
}


static int __devinit bma222e_set_any_motion_interrupt(struct bma222e_data *bma222e,
						u8 enable, u8 dur, u8 thres)
{
	int error;

	error = bma222e_set_reg_bits(bma222e->client, dur,
				BMA222E_ANY_MOTION_DUR_POS,
				BMA222E_ANY_MOTION_DUR_MSK,
				BMA222E_ANY_MOTION_DUR_REG);
	if (error)
		return error;

	error = bma222e_write_byte(bma222e->client,
				BMA222E_ANY_MOTION_THRES_REG, thres);
	if (error)
		return error;

	error = bma222e_set_reg_bits(bma222e->client, !!enable,
				BMA222E_ADV_INT_EN_POS, BMA222E_ADV_INT_EN_MSK,
				BMA222E_ADV_INT_EN_REG);
	if (error)
		return error;

	return bma222e_set_reg_bits(bma222e->client, !!enable,
				BMA222E_ANY_MOTION_EN_POS,
				BMA222E_ANY_MOTION_EN_MSK,
				BMA222E_ANY_MOTION_EN_REG);
}

static void bma222e_report_xyz(struct bma222e_data *bma222e)
{
	u8 data[BMA222E_XYZ_DATA_SIZE];
	s16 data2[BMA222E_XYZ_DATA_SIZE];
	s16 final_data[BMA222E_XYZ_DATA_SIZE];
	s32 ret;

	ret = i2c_smbus_read_i2c_block_data(bma222e->client,
			BMA222E_ACC_X_LSB_REG, 5, data);
	if (ret < 5)
		return;

	data2[BMA222_AXIS_X] = (s16)data[BMA222_AXIS_X*2] ;
	data2[BMA222_AXIS_Y] = (s16)data[BMA222_AXIS_Y*2];
	data2[BMA222_AXIS_Z] = (s16)data[BMA222_AXIS_Z*2] ;

	if(data2[BMA222_AXIS_X] & 0x80)
	{
		data2[BMA222_AXIS_X] = ~data2[BMA222_AXIS_X];
		data2[BMA222_AXIS_X] &= 0xff;
		data2[BMA222_AXIS_X]+=1;
		data2[BMA222_AXIS_X] = -data2[BMA222_AXIS_X];
	}
	if(data2[BMA222_AXIS_Y] & 0x80)
	{
		data2[BMA222_AXIS_Y] = ~data2[BMA222_AXIS_Y];
		data2[BMA222_AXIS_Y] &= 0xff;
		data2[BMA222_AXIS_Y]+=1;
		data2[BMA222_AXIS_Y] = -data2[BMA222_AXIS_Y];
	}
	if(data2[BMA222_AXIS_Z] & 0x80)
	{
		data2[BMA222_AXIS_Z] = ~data2[BMA222_AXIS_Z];
		data2[BMA222_AXIS_Z] &= 0xff;
		data2[BMA222_AXIS_Z]+=1;
		data2[BMA222_AXIS_Z] = -data2[BMA222_AXIS_Z];
	}

//	GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] \n", data[BMA222_AXIS_X*2], data[BMA222_AXIS_Y*2], data[BMA222_AXIS_Z*2],
//				x, y, z);
	/*remap coordinate*/
	final_data[bma222e->cvt.map[BMA222_AXIS_X]] = bma222e->cvt.sign[BMA222_AXIS_X]*data2[BMA222_AXIS_X];
	final_data[bma222e->cvt.map[BMA222_AXIS_Y]] = bma222e->cvt.sign[BMA222_AXIS_Y]*data2[BMA222_AXIS_Y];
	final_data[bma222e->cvt.map[BMA222_AXIS_Z]] = bma222e->cvt.sign[BMA222_AXIS_Z]*data2[BMA222_AXIS_Z];
	
	input_report_rel(bma222e->input, REL_X, final_data[BMA222_AXIS_X]);
	input_report_rel(bma222e->input, REL_Y, final_data[BMA222_AXIS_Y]);
	input_report_rel(bma222e->input, REL_Z, final_data[BMA222_AXIS_Z]);
	input_sync(bma222e->input);
}

static irqreturn_t bma222e_irq_thread(int irq, void *dev)
{
	bma222e_report_xyz(dev);

	return IRQ_HANDLED;
}

static void bma222e_read(struct bma222e_data *bma222e)
{
	bma222e_report_xyz(bma222e);
}

static int bma222e_open(struct bma222e_data *bma222e)
{
	int error;
	printk("Ivan bma222e_open \n");

//	error = pm_runtime_get_sync(&bma222e->client->dev);
//	if (error && error != -ENOSYS)
//		return error;

	/*
	 * See if runtime PM woke up the device. If runtime PM
	 * is disabled we need to do it ourselves.
	 */
	if (bma222e->mode != BMA222E_MODE_NORMAL) {
		error = bma222e_set_mode(bma222e, BMA222E_MODE_NORMAL);
		if (error)
			return error;
	}
	queue_delayed_work(bma222e->wq, &bma222e->dw,
			    usecs_to_jiffies(bma222e->poll_delay_us));	
	return 0;
}

static int bma222e_close(struct bma222e_data *bma222e)
{
	int error = 0;    
//	pm_runtime_put_sync(&bma222e->client->dev);
	printk("Ivan bma222e_close \n");
	if (bma222e->mode != BMA222E_MODE_SLEEP)
	{
		error = bma222e_set_mode(bma222e, BMA222E_MODE_SLEEP);
	    if (error)
		    return error;	
	}
	cancel_delayed_work_sync(&bma222e->dw);	
	return 0;	
}

static int bma222e_irq_open(struct input_dev *input)
{
	struct bma222e_data *bma222e = input_get_drvdata(input);

	return bma222e_open(bma222e);
}

static void bma222e_irq_close(struct input_dev *input)
{
	struct bma222e_data *bma222e = input_get_drvdata(input);

	bma222e_close(bma222e);
}


static void bma222e_work(struct work_struct *ws)
{
	struct bma222e_data *bma;

	bma = container_of(ws, struct bma222e_data, dw.work);
//	printk("Ivan bma222e_work mode = %d \n", bma->mode);
	if (bma->mode == BMA222E_MODE_NORMAL)
	    bma222e_read(bma);
	queue_delayed_work(bma->wq, &bma->dw,
			   usecs_to_jiffies(bma->poll_delay_us));
}


static int __devinit bma222e_initialize(struct bma222e_data *bma222e,
				       const struct bma222e_cfg *cfg, int reset_cali)
{
	int error;
	
	error = regulator_enable(bma222e->vdd);
	if (error)
	    dev_err(&bma222e->client->dev, "%s %s ERR!\n",
	    __func__, "vdd regulator");
	msleep_range(5);
	
	error = BMA222_CheckDeviceID(bma222e->client); 
	if(error != 0)
	{
		return error;
	}	
	
/*	
	error = bma222e_soft_reset(bma222e);
	if (error)
		return error;
*/
	error = bma222e_set_bandwidth(bma222e, cfg->bandwidth);
	if (error)
		return error;

	error = bma222e_set_range(bma222e, cfg->range);
	if (error)
		return error;

	if (bma222e->client->irq) {
		error = bma222e_set_any_motion_interrupt(bma222e,
					cfg->any_motion_int,
					cfg->any_motion_dur,
					cfg->any_motion_thres);
		if (error)
			return error;

		error = bma222e_set_high_g_interrupt(bma222e,
					cfg->hg_int, cfg->hg_hyst,
					cfg->hg_dur, cfg->hg_thres);
		if (error)
			return error;

		error = bma222e_set_low_g_interrupt(bma222e,
					cfg->lg_int, cfg->lg_hyst,
					cfg->lg_dur, cfg->lg_thres);
		if (error)
			return error;
	}
	if (reset_cali)
	{
//Ivan TODO calibartion data init
	}
//	regulator_disable(bma222e->vdd);
	printk("bma222e_initialize ok \n");
	return bma222e_set_mode(bma222e, BMA222E_MODE_SLEEP);
}


static ssize_t bma_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct bma222e_data *bma;
	unsigned int enable;
	bool en;
	int err;

	bma = dev_get_drvdata(dev);
	err = kstrtouint(buf, 10, &enable);
	if (err)
		return -EINVAL;

	if (enable)
	{
		en = true;
		err = bma222e_open(bma);
	}
	else
	{
		en = false;
		err = bma222e_close(bma);
	}
	if (bma->dbg)
		dev_info(&bma->client->dev, "%s: %x\n", __func__, en);
	if (err) {
		dev_err(&bma->client->dev, "%s: %x ERR=%d\n", __func__, en, err);
		return err;
	}

	return count;
}

static ssize_t bma_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct bma222e_data *bma;
	unsigned int enable;

	bma = dev_get_drvdata(dev);
	if (bma->mode == BMA222E_MODE_NORMAL)
		enable = 1;
	else
		enable = 0;
	return sprintf(buf, "%u\n", enable);
}

static ssize_t bma_delay_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct bma222e_data *bma;
	unsigned int delay_us;
	int err;

	bma = dev_get_drvdata(dev);
	err = kstrtouint(buf, 10, &delay_us);
//	dev_info(&bma->client->dev, "%s: %u\n",
//		    __func__, delay_us);	
	if (err)
		return -EINVAL;
	
	delay_us = delay_us * 1000;
	
	if (delay_us < (BMA_INPUT_DELAY_MS_MIN * 1000))
		delay_us = (BMA_INPUT_DELAY_MS_MIN * 1000);
	if ((bma->mode == BMA222E_MODE_NORMAL) && (delay_us != bma->poll_delay_us))
		bma->poll_delay_us = delay_us;

//	dev_info(&bma->client->dev, "%s: %u\n",
//		    __func__, delay_us);	
/*	    
		err = bma_delay(bma, delay_us);
	if (!err) {
		if (bma->dbg)
			dev_info(&bma->i2c->dev, "%s: %u\n",
				 __func__, delay_us);
		bma->poll_delay_us = delay_us;
	} else {
		dev_err(&bma->i2c->dev, "%s: %u ERR=%d\n",
			__func__, delay_us, err);
		return err;
	}
*/
	return count;
}

static ssize_t bma_delay_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct bma222e_data *bma;

	bma = dev_get_drvdata(dev);
	if (bma->mode == BMA222E_MODE_NORMAL)
		return sprintf(buf, "%u\n", bma->poll_delay_us);

	return sprintf(buf, "%u\n", (BMA_INPUT_DELAY_MS_MIN * 1000));
}

static ssize_t bma_max_range_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct bma222e_data *bma;
	unsigned int range_i;
	int err;

	bma = dev_get_drvdata(dev);
	err = kstrtouint(buf, 10, &range_i);
	if (err)
		return -EINVAL;

	dev_dbg(&bma->client->dev, "%s %u\n", __func__, range_i);

	if (range_i > 1)
		return -EINVAL;

	if (bma->mode == BMA222E_MODE_NORMAL) {
	    if (range_i == 0)
		err = bma222e_set_range(bma,BMA222E_RANGE_2G);
	    else
		err = bma222e_set_range(bma,BMA222E_RANGE_4G);
		
	    if (err)
		    return err;
	    bma->range_i = range_i;
		
	}

	return count;
}

static ssize_t bma_max_range_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct bma222e_data *bma;
	unsigned int range;

	bma = dev_get_drvdata(dev);
	if (bma->mode == BMA222E_MODE_NORMAL) {
		range = bma->range_i;
	} else {
		range = 0x4000 >> bma->range_i;
	}
	return sprintf(buf, "%u\n", range);
}


static ssize_t bma_mpu_fifo_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct bma222e_data *bma;
	unsigned int fifo_enable;
	int err;

	return count;
}

static ssize_t bma_mpu_fifo_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct bma222e_data *bma = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", 0);
}


static ssize_t bma_orientation_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct bma222e_data *bma;
	signed char *m;

	bma = dev_get_drvdata(dev);
#ifdef CONFIG_INV_MPU
	m = bma->pdata.orientation;
#endif
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		       m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}



static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   bma_enable_show, bma_enable_store);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   bma_delay_show, bma_delay_store);
static DEVICE_ATTR(max_range, S_IRUGO | S_IWUSR | S_IWGRP,
		   bma_max_range_show, bma_max_range_store);
static DEVICE_ATTR(mpu_fifo_en, S_IRUGO | S_IWUSR | S_IWGRP,
		   bma_mpu_fifo_enable_show, bma_mpu_fifo_enable_store);
#ifdef CONFIG_INV_MPU
static DEVICE_ATTR(orientation, S_IRUGO,
		   bma_orientation_show, NULL);
#endif


static struct attribute *bma_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_max_range.attr,
#ifdef CONFIG_INV_MPU
	&dev_attr_orientation.attr,
#endif
	&dev_attr_mpu_fifo_en.attr,
//	&dev_attr_key,	
	NULL
};

static struct attribute_group bma_attr_group = {
	.name = BMA_NAME,
	.attrs = bma_attrs
};

static int bma_sysfs_create(struct bma222e_data *bma)
{
	int err;

	err = sysfs_create_group(&bma->input->dev.kobj, &bma_attr_group);
	return err;
}

static void bma_input_close(struct input_dev *idev)
{
	struct bma222e_data *bma;

	bma = input_get_drvdata(idev);
	if (bma != NULL)
		bma222e_close(bma);
}

static int bma222e_input_create(struct bma222e_data *bma222e)
{
	int err;

	bma222e->input = input_allocate_device();
	if (!bma222e->input) {
		err = -ENOMEM;
		dev_err(&bma222e->client->dev, "%s ERR %d\n", __func__, err);
		return err;
	}

	bma222e->input->name = BMA_NAME;
	bma222e->input->dev.parent = &bma222e->client->dev;
	bma222e->input->close = bma_input_close;
	input_set_drvdata(bma222e->input, bma222e);
	input_set_capability(bma222e->input, EV_REL, REL_X);
	input_set_capability(bma222e->input, EV_REL, REL_Y);
	input_set_capability(bma222e->input, EV_REL, REL_Z);
//	input_set_capability(bma222e->input, EV_REL, REL_MISC);
//	input_set_capability(bma222e->input, EV_REL, REL_WHEEL);
	err = input_register_device(bma222e->input);
	if (err) {
		input_free_device(bma222e->input);
		bma222e->input = NULL;
	}
	return err;
}


static int bma222e_regulator_get(struct i2c_client *client, struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = devm_regulator_get(&client->dev, vreg_name);
	if (unlikely(IS_ERR_OR_NULL(reg))) {
		printk("Ivan %s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&client->dev, "%s: %s\n",
			__func__, vreg_name);

	*vreg = reg;
	return err;
}

static int __devinit bma222e_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
#ifdef CONFIG_INV_MPU
	const struct mpu_platform_data *pdata = client->dev.platform_data;
#else
	const struct akm8963_platform_data *pdata = client->dev.platform_data;
#endif
	const struct bma222e_cfg *cfg;
	struct bma222e_data *bma222e;
	int chip_id;
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

/*
	chip_id = i2c_smbus_read_byte_data(client, BMA222E_CHIP_ID_REG);
	if (chip_id != BMA222E_CHIP_ID) {
		dev_err(&client->dev, "BMA222E chip id error: %d\n", chip_id);
		return -EINVAL;
	}
*/
	bma222e = kzalloc(sizeof(struct bma222e_data), GFP_KERNEL);
	if (!bma222e)
		return -ENOMEM;

	memset(bma222e, 0, sizeof(struct bma222e_data));
	bma222e->pdata = *pdata;
	bma222e->client = client;
	i2c_set_clientdata(client, bma222e);
	bma222e->poll_delay_us = (BMA_POLL_DELAY_MS_DFLT * 1000);
	bma222e->range_i = INV_FS_02G;
	bma222e->dbg = BMA_DEBUG_ON;
/*	
	if (pdata) {
		if (pdata->irq_gpio_cfg) {
			error = pdata->irq_gpio_cfg();
			if (error) {
				dev_err(&client->dev,
					"IRQ GPIO conf. error %d, error %d\n",
					client->irq, error);
				goto err_free_mem;
			}
		}
		cfg = &pdata->cfg;
	} else {
		cfg = &default_cfg;
	}
*/
//Ivan
	direction_get_convert(pdata->layout,&bma222e->cvt);
	cfg = &default_cfg;
	bma222e_input_create(bma222e);
	
	bma222e_regulator_get(client, &bma222e->vdd, "vdd_sensor");		//

	error = bma222e_initialize(bma222e, cfg, 1);
	if (error)
		goto err_free_mem;
	
	bma222e->wq = create_singlethread_workqueue(BMA_NAME);
	if (!bma222e->wq) {
		dev_err(&client->dev, "%s workqueue ERR\n", __func__);
		error = -ENOMEM;
		goto err_free_mem;
	}
	

	INIT_DELAYED_WORK(&bma222e->dw, bma222e_work);
	error = bma_sysfs_create(bma222e);	

//	pm_runtime_enable(&client->dev);

	return 0;

err_free_mem:
	kfree(bma222e);
	return error;
}

static int __devexit bma222e_remove(struct i2c_client *client)
{
	struct bma222e_data *bma222e = i2c_get_clientdata(client);

//	pm_runtime_disable(&client->dev);
/*
	if (client->irq > 0) {
		free_irq(client->irq, bma222e);
		input_unregister_device(bma222e->input);
	} else {
		input_unregister_polled_device(bma222e->input_polled);
		input_free_polled_device(bma222e->input_polled);
	}
*/
	kfree(bma222e);

	return 0;
}

#ifdef CONFIG_PM
static int bma222e_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma222e_data *bma222e = i2c_get_clientdata(client);
	int err = 0;
	
	if(bma222e == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	atomic_set(&bma222e->suspend, 1);
	if(err = bma222e_set_mode(bma222e, BMA222E_MODE_SLEEP))
	{
		GSE_ERR("write power control fail!!\n");
		return -EINVAL;
	}       
	return err;	
//	return bma222e_set_mode(bma222e, BMA222E_MODE_SLEEP);
}

static int bma222e_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma222e_data *bma222e = i2c_get_clientdata(client);
	if(bma222e == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

#if 0 //ALPS00400022 sensor not work issue
	if(err = bma222_init_client(client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
#endif
	bma222e_set_mode(bma222e, BMA222E_MODE_NORMAL);

	return 0;
//	return bma222e_set_mode(bma222e, BMA222E_MODE_NORMAL);
}
#endif

static UNIVERSAL_DEV_PM_OPS(bma222e_pm, bma222e_suspend, bma222e_resume, NULL);

static const struct i2c_device_id bma222e_id[] = {
	{ "bma222e", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma222e_id);

static struct i2c_driver bma222e_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= BMA222E_DRIVER,
		.pm	= &bma222e_pm,
	},
	.class		= I2C_CLASS_HWMON,
	.id_table	= bma222e_id,
	.probe		= bma222e_probe,
	.remove		= __devexit_p(bma222e_remove),
};

module_i2c_driver(bma222e_driver);

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA222E driver");
MODULE_LICENSE("GPL");
