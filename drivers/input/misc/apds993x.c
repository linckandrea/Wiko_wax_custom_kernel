/*
 *  apds993x.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2012 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2012 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t14.h>
#include <linux/gpio.h>

#define APDS993X_DRV_NAME	"apds993x"
#define DRIVER_VERSION		"1.0.0"

#define ALS_POLLING_ENABLED

//#define IOCTOLL
#define APDS993X_HAL_USE_SYS_ENABLE

#define APDS993X_INT		gpio_to_irq(104) //IRQ_EINT20

#define APDS993X_PS_DETECTION_THRESHOLD		600
#define APDS993X_PS_HSYTERESIS_THRESHOLD	500
#define APDS993X_PS_PULSE_NUMBER		8

#define APDS993X_ALS_THRESHOLD_HSYTERESIS	20	/* 20 = 20% */

#define APDS993X_GA	48	/* 0.48 without glass window */
#define APDS993X_COE_B	223	/* 2.23 without glass window */
#define APDS993X_COE_C	70	/* 0.70 without glass window */
#define APDS993X_COE_D	142	/* 1.42 without glass window */
#define APDS993X_DF	52

/* Change History 
 *
 * 1.0.0	Fundamental Functions of APDS-993x
 *
 */

#define APDS993X_IOCTL_PS_ENABLE	1
#define APDS993X_IOCTL_PS_GET_ENABLE	2
#define APDS993X_IOCTL_PS_GET_PDATA	3	// pdata
#define APDS993X_IOCTL_ALS_ENABLE	4
#define APDS993X_IOCTL_ALS_GET_ENABLE	5
#define APDS993X_IOCTL_ALS_GET_CH0DATA	6	// ch0data
#define APDS993X_IOCTL_ALS_GET_CH1DATA	7	// ch1data
#define APDS993X_IOCTL_ALS_DELAY	8

/*
 * Defines
 */

#define APDS993X_ENABLE_REG	0x00
#define APDS993X_ATIME_REG	0x01
#define APDS993X_PTIME_REG	0x02
#define APDS993X_WTIME_REG	0x03
#define APDS993X_AILTL_REG	0x04
#define APDS993X_AILTH_REG	0x05
#define APDS993X_AIHTL_REG	0x06
#define APDS993X_AIHTH_REG	0x07
#define APDS993X_PILTL_REG	0x08
#define APDS993X_PILTH_REG	0x09
#define APDS993X_PIHTL_REG	0x0A
#define APDS993X_PIHTH_REG	0x0B
#define APDS993X_PERS_REG	0x0C
#define APDS993X_CONFIG_REG	0x0D
#define APDS993X_PPCOUNT_REG	0x0E
#define APDS993X_CONTROL_REG	0x0F
#define APDS993X_REV_REG	0x11
#define APDS993X_ID_REG		0x12
#define APDS993X_STATUS_REG	0x13
#define APDS993X_CH0DATAL_REG	0x14
#define APDS993X_CH0DATAH_REG	0x15
#define APDS993X_CH1DATAL_REG	0x16
#define APDS993X_CH1DATAH_REG	0x17
#define APDS993X_PDATAL_REG	0x18
#define APDS993X_PDATAH_REG	0x19

#define CMD_BYTE		0x80
#define CMD_WORD		0xA0
#define CMD_SPECIAL		0xE0

#define CMD_CLR_PS_INT		0xE5
#define CMD_CLR_ALS_INT		0xE6
#define CMD_CLR_PS_ALS_INT	0xE7


/* Register Value define : ATIME */
#define APDS993X_100MS_ADC_TIME	0xDB  /* 100.64ms integration time */
#define APDS993X_50MS_ADC_TIME	0xED  /* 51.68ms integration time */
#define APDS993X_27MS_ADC_TIME	0xF6  /* 27.2ms integration time */

/* Register Value define : PRXCNFG */
#define APDS993X_ALS_REDUCE	0x04  /* ALSREDUCE - ALS Gain reduced by 4x */

/* Register Value define : PERS */
#define APDS993X_PPERS_0	0x00  /* Every proximity ADC cycle */
#define APDS993X_PPERS_1	0x10  /* 1 consecutive proximity value out of range */
#define APDS993X_PPERS_2	0x20  /* 2 consecutive proximity value out of range */
#define APDS993X_PPERS_3	0x30  /* 3 consecutive proximity value out of range */
#define APDS993X_PPERS_4	0x40  /* 4 consecutive proximity value out of range */
#define APDS993X_PPERS_5	0x50  /* 5 consecutive proximity value out of range */
#define APDS993X_PPERS_6	0x60  /* 6 consecutive proximity value out of range */
#define APDS993X_PPERS_7	0x70  /* 7 consecutive proximity value out of range */
#define APDS993X_PPERS_8	0x80  /* 8 consecutive proximity value out of range */
#define APDS993X_PPERS_9	0x90  /* 9 consecutive proximity value out of range */
#define APDS993X_PPERS_10	0xA0  /* 10 consecutive proximity value out of range */
#define APDS993X_PPERS_11	0xB0  /* 11 consecutive proximity value out of range */
#define APDS993X_PPERS_12	0xC0  /* 12 consecutive proximity value out of range */
#define APDS993X_PPERS_13	0xD0  /* 13 consecutive proximity value out of range */
#define APDS993X_PPERS_14	0xE0  /* 14 consecutive proximity value out of range */
#define APDS993X_PPERS_15	0xF0  /* 15 consecutive proximity value out of range */

#define APDS993X_APERS_0	0x00  /* Every ADC cycle */
#define APDS993X_APERS_1	0x01  /* 1 consecutive proximity value out of range */
#define APDS993X_APERS_2	0x02  /* 2 consecutive proximity value out of range */
#define APDS993X_APERS_3	0x03  /* 3 consecutive proximity value out of range */
#define APDS993X_APERS_5	0x04  /* 5 consecutive proximity value out of range */
#define APDS993X_APERS_10	0x05  /* 10 consecutive proximity value out of range */
#define APDS993X_APERS_15	0x06  /* 15 consecutive proximity value out of range */
#define APDS993X_APERS_20	0x07  /* 20 consecutive proximity value out of range */
#define APDS993X_APERS_25	0x08  /* 25 consecutive proximity value out of range */
#define APDS993X_APERS_30	0x09  /* 30 consecutive proximity value out of range */
#define APDS993X_APERS_35	0x0A  /* 35 consecutive proximity value out of range */
#define APDS993X_APERS_40	0x0B  /* 40 consecutive proximity value out of range */
#define APDS993X_APERS_45	0x0C  /* 45 consecutive proximity value out of range */
#define APDS993X_APERS_50	0x0D  /* 50 consecutive proximity value out of range */
#define APDS993X_APERS_55	0x0E  /* 55 consecutive proximity value out of range */
#define APDS993X_APERS_60	0x0F  /* 60 consecutive proximity value out of range */

/* Register Value define : CONTROL */
#define APDS993X_AGAIN_1X	0x00  /* 1X ALS GAIN */
#define APDS993X_AGAIN_8X	0x01  /* 8X ALS GAIN */
#define APDS993X_AGAIN_16X	0x02  /* 16X ALS GAIN */
#define APDS993X_AGAIN_120X	0x03  /* 120X ALS GAIN */

#define APDS993X_PRX_IR_DIOD	0x20  /* Proximity uses CH1 diode */

#define APDS993X_PGAIN_1X	0x00  /* PS GAIN 1X */
#define APDS993X_PGAIN_2X	0x04  /* PS GAIN 2X */
#define APDS993X_PGAIN_4X	0x08  /* PS GAIN 4X */
#define APDS993X_PGAIN_8X	0x0C  /* PS GAIN 8X */

#define APDS993X_PDRVIE_100MA	0x00  /* PS 100mA LED drive */
#define APDS993X_PDRVIE_50MA	0x40  /* PS 50mA LED drive */
#define APDS993X_PDRVIE_25MA	0x80  /* PS 25mA LED drive */
#define APDS993X_PDRVIE_12_5MA	0xC0  /* PS 12.5mA LED drive */


typedef enum 
{
  APDS993X_ALS_RES_10240 = 0,    /* 27.2ms integration time */ 
  APDS993X_ALS_RES_19456 = 1,    /* 51.68ms integration time */
  APDS993X_ALS_RES_37888 = 2     /* 100.64ms integration time */
} apds993x_als_res_e;

typedef enum 
{
  APDS993X_ALS_GAIN_1X    = 0,    /* 1x AGAIN */ 
  APDS993X_ALS_GAIN_8X    = 1,    /* 8x AGAIN */
  APDS993X_ALS_GAIN_16X   = 2,    /* 16x AGAIN */
  APDS993X_ALS_GAIN_120X  = 3     /* 120x AGAIN */
} apds993x_als_gain_e;

/*
 * Structs
 */

struct apds993x_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct delayed_work	dwork;		/* for PS interrupt */
	struct delayed_work	als_dwork;	/* for ALS polling */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;

	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	/* PS parameters */
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold; 	/* always lower than ps_threshold */
	unsigned int ps_detection;		/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_data;			/* to store PS data */

	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	unsigned int als_data;		/* to store ALS data */
	int als_prev_lux;		/* to store previous lux value */

	unsigned int als_gain;		/* needed for Lux calculation */
	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */
	unsigned int als_atime_index;	/* storage for als integratiion time */
	unsigned int als_again_index;	/* storage for als GAIN */
	unsigned int als_reduce;	/* flag indicate ALS 6x reduction */
};

/*
 * Global data
 */
static struct i2c_client *apds993x_i2c_client; /* global i2c_client to support ioctl */

static unsigned char apds993x_als_atime_tb[] = { 0xF6, 0xED, 0xDB };
static unsigned short apds993x_als_integration_tb[] = {2720, 5168, 10064};
static unsigned short apds993x_als_res_tb[] = { 10240, 19456, 37888 };
static unsigned char apds993x_als_again_tb[] = { 1, 8, 16, 120 };
static unsigned char apds993x_als_again_bit_tb[] = { 0x00, 0x01, 0x02, 0x03 };

#ifdef ALS_POLLING_ENABLED
static int apds993x_set_als_poll_delay(struct i2c_client *client, unsigned int val);
#endif

/*
 * Management functions
 */

static int apds993x_set_command(struct i2c_client *client, int command)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;
		
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clearInt);
	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds993x_set_enable(struct i2c_client *client, int enable)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG, enable);
	mutex_unlock(&data->update_lock);

	data->enable = enable;

	return ret;
}

static int apds993x_set_atime(struct i2c_client *client, int atime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}

static int apds993x_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds993x_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds993x_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_AILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->ailt = threshold;

	return ret;
}

static int apds993x_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_AIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->aiht = threshold;

	return ret;
}

static int apds993x_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->pilt = threshold;

	return ret;
}

static int apds993x_set_piht(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->piht = threshold;

	return ret;
}

static int apds993x_set_pers(struct i2c_client *client, int pers)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds993x_set_config(struct i2c_client *client, int config)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds993x_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_PPCOUNT_REG, ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static int apds993x_set_control(struct i2c_client *client, int control)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_CONTROL_REG, control);
	mutex_unlock(&data->update_lock);

	data->control = control;

	return ret;
}

static int LuxCalculation(struct i2c_client *client, int ch0data, int ch1data)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int luxValue=0;

	int IAC1=0;
	int IAC2=0;
	int IAC=0;

	IAC1 = (ch0data - (APDS993X_COE_B*ch1data)/100);			// re-adjust COE_B to avoid 2 decimal point
	IAC2 = ((APDS993X_COE_C*ch0data)/100 - (APDS993X_COE_D*ch1data)/100); 	// re-adjust COE_C and COE_D to void 2 decimal point

	if (IAC1 > IAC2)
		IAC = IAC1;
	else if (IAC1 <= IAC2)
		IAC = IAC2;
	else
		IAC = 0;

	if (IAC1<0 && IAC2<0) {
		IAC = 0;	// cdata and irdata saturated
		return -1; 	// don't report first, change gain may help
	}

	if (data->als_reduce) {
		luxValue = ((IAC*APDS993X_GA*APDS993X_DF)/100)*4/((apds993x_als_integration_tb[data->als_atime_index]/100)*apds993x_als_again_tb[data->als_again_index]);
	}
	else {
		luxValue = ((IAC*APDS993X_GA*APDS993X_DF)/100)/((apds993x_als_integration_tb[data->als_atime_index]/100)*apds993x_als_again_tb[data->als_again_index]);
	}

	return luxValue;
}

static void apds993x_change_ps_threshold(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);

	data->ps_data =	i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PDATAL_REG);

	if ( (data->ps_data > data->pilt) && (data->ps_data >= data->piht) ) {
		/* far-to-near detected */
		data->ps_detection = 1;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);/* FAR-to-NEAR detection */	
		input_sync(data->input_dev_ps);     //\C9ϱ\A8\CA\FD\BE\DD

		i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PILTL_REG, data->ps_hysteresis_threshold);
		i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PIHTL_REG, 1023);

		data->pilt = data->ps_hysteresis_threshold;
		data->piht = 1023;

		printk("far-to-near detected\n");
	}
	else if ( (data->ps_data <= data->pilt) && (data->ps_data < data->piht) ) {
		/* near-to-far detected */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);    //\C9ϱ\A8\CA\FD\BE\DD

		i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PILTL_REG, 0);
		i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PIHTL_REG, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		printk("near-to-far detected\n");
	}
}

static void apds993x_change_als_threshold(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ch0data, ch1data;
	int luxValue=0;
	int err;
	unsigned char change_again=0;
	unsigned char control_data=0;
	unsigned char lux_is_valid=1;

	ch0data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH0DATAL_REG);
	ch1data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH1DATAL_REG);

	luxValue = LuxCalculation(client, ch0data, ch1data);

	if (luxValue >= 0) {
		luxValue = luxValue<30000 ? luxValue : 30000;
		data->als_prev_lux = luxValue;
	}
	else {
		lux_is_valid = 0;	// don't report, the lux is invalid value
		luxValue = data->als_prev_lux;
		if (data->als_reduce) {
			lux_is_valid = 1;
			luxValue = 30000;	// report anyway since this is the lowest gain
		}
	}

//Ivan	printk("lux=%d ch0data=%d ch1data=%d again=%d als_reduce=%d\n", luxValue, ch0data, ch1data, apds993x_als_again_tb[data->als_again_index], data->als_reduce);
	
	// check PS under sunlight
	if ( (data->ps_detection == 1) && (ch0data > (75*(1024*(256-apds993x_als_atime_tb[data->als_atime_index])))/100))	// PS was previously in far-to-near condition
	{
		// need to inform input event as there will be no interrupt from the PS
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);

		i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PILTL_REG, 0);
		i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PIHTL_REG, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		data->ps_detection = 0;	/* near-to-far detected */

		printk("apds_993x_proximity_handler = FAR\n");	
	}

	if (lux_is_valid) {
		input_report_abs(data->input_dev_als, ABS_MISC, luxValue); // report the lux level
		input_sync(data->input_dev_als);
	}

	data->als_data = ch0data;

	data->als_threshold_l = (data->als_data * (100-APDS993X_ALS_THRESHOLD_HSYTERESIS) ) /100;
	data->als_threshold_h = (data->als_data * (100+APDS993X_ALS_THRESHOLD_HSYTERESIS) ) /100;

	if (data->als_threshold_h >= apds993x_als_res_tb[data->als_atime_index]) {
		data->als_threshold_h = apds993x_als_res_tb[data->als_atime_index];
	}

	if (data->als_data >= (apds993x_als_res_tb[data->als_atime_index]*90)/100) {
		// lower AGAIN if possible
		if (data->als_again_index != APDS993X_ALS_GAIN_1X) {
			data->als_again_index--;
			change_again = 1;
		}
		else {
			err = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_CONFIG_REG, APDS993X_ALS_REDUCE);
			if (err >= 0) {			
				data->als_reduce = 1;
			}				
		}	
	}
	else if (data->als_data <= (apds993x_als_res_tb[data->als_atime_index]*10)/100) {
		// increase AGAIN if possible
		if (data->als_reduce) {
			err = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_CONFIG_REG, 0);
			if (err >= 0) {			
				data->als_reduce = 0;
			}				
		}
		else if (data->als_again_index != APDS993X_ALS_GAIN_120X) {
			data->als_again_index++;
			change_again = 1;
		}
	}

	if (change_again) {
		control_data = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_CONTROL_REG);
		control_data = control_data & 0xFC;
		control_data = control_data | apds993x_als_again_bit_tb[data->als_again_index];
		i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_CONTROL_REG, control_data);
	}
	
	i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_AILTL_REG, data->als_threshold_l);
	i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_AIHTL_REG, data->als_threshold_h);

}

static void apds993x_reschedule_work(struct apds993x_data *data,
					  unsigned long delay)
{
	//unsigned long flags;

	//spin_lock_irqsave(&data->update_lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, delay);

	//spin_unlock_irqrestore(&data->update_lock, flags);
}


#ifdef ALS_POLLING_ENABLED

/* ALS polling routine */
static void apds993x_als_polling_work_handler(struct work_struct *work)
{
	struct apds993x_data *data = container_of(work, struct apds993x_data, als_dwork.work);
	struct i2c_client *client=data->client;
	int ch0data, ch1data, pdata;
	int luxValue=0;
	int err;
	unsigned char change_again=0;
	unsigned char control_data=0;
	unsigned char lux_is_valid=1;
	
	ch0data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH0DATAL_REG);
	ch1data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH1DATAL_REG);
	pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PDATAL_REG);
	
	luxValue = LuxCalculation(client, ch0data, ch1data);
	
	if (luxValue >= 0) {
		luxValue = luxValue<30000 ? luxValue : 30000;
		data->als_prev_lux = luxValue;
	}
	else {
		lux_is_valid = 0;	// don't report, this is invalid lux value
		luxValue = data->als_prev_lux;
		if (data->als_reduce) {
			lux_is_valid = 1;
			luxValue = 30000;	// report anyway since this is the lowest gain
		}
	}
	
//Ivan	printk("lux=%d ch0data=%d ch1data=%d pdata=%d delay=%d again=%d als_reduce=%d)\n", luxValue, ch0data, ch1data, pdata, data->als_poll_delay, apds993x_als_again_tb[data->als_again_index], data->als_reduce);

	// check PS under sunlight
	if ( (data->ps_detection == 1) && (ch0data > (75*(1024*(256-data->atime)))/100))	// PS was previously in far-to-near condition
	{
		// need to inform input event as there will be no interrupt from the PS
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);

		i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PILTL_REG, 0);
		i2c_smbus_write_word_data(client, CMD_WORD|APDS993X_PIHTL_REG, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		data->ps_detection = 0;	/* near-to-far detected */

		printk("apds_993x_proximity_handler = FAR\n");	
	}

	if (lux_is_valid) {
		input_report_abs(data->input_dev_als, ABS_MISC, luxValue); // report the lux level
		input_sync(data->input_dev_als);
	}

	data->als_data = ch0data;

	if (data->als_data >= (apds993x_als_res_tb[data->als_atime_index]*90)/100) {
		// lower AGAIN if possible
		if (data->als_again_index != APDS993X_ALS_GAIN_1X) {
			data->als_again_index--;
			change_again = 1;
		}
		else {
			err = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_CONFIG_REG, APDS993X_ALS_REDUCE);
			if (err >= 0) {			
				data->als_reduce = 1;
			}				
		}	
	}
	else if (data->als_data <= (apds993x_als_res_tb[data->als_atime_index]*10)/100) {
		// increase AGAIN if possible
		if (data->als_reduce) {
			err = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_CONFIG_REG, 0);
			if (err >= 0) {			
				data->als_reduce = 0;
			}				
		}
		else if (data->als_again_index != APDS993X_ALS_GAIN_120X) {
			data->als_again_index++;
			change_again = 1;
		}
	}

	if (change_again) {
		control_data = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_CONTROL_REG);
		control_data = control_data & 0xFC;
		control_data = control_data | apds993x_als_again_bit_tb[data->als_again_index];
		i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_CONTROL_REG, control_data);
	}
	
	schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));	// restart timer
}

#endif // ALS_POLLING_ENABLED

/* PS interrupt routine */
static void apds993x_work_handler(struct work_struct *work)
{
	struct apds993x_data *data = container_of(work, struct apds993x_data, dwork.work);
	struct i2c_client *client=data->client;
	int status;
	int ch0data;
	int enable;

	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_STATUS_REG);//\B6\C1ȡAPDS ״̬\BCĴ\E6\C6\F7
	enable = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG);

	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG, 1);	/* disable 993x's ADC first */

	printk("status = %x\n", status);

	if ((status & enable & 0x30) == 0x30) {
		/* both PS and ALS are interrupted */
		apds993x_change_als_threshold(client);
		
		ch0data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH0DATAL_REG);
		if (ch0data < (75*(1024*(256-data->atime)))/100)
			apds993x_change_ps_threshold(client);
		else {
			if (data->ps_detection == 1) {
				apds993x_change_ps_threshold(client);			
			}
			else {
				printk("Triggered by background ambient noise\n");
			}
		}

		apds993x_set_command(client, 2);	/* 2 = CMD_CLR_PS_ALS_INT */
	}
	else if ((status & enable & 0x20) == 0x20) {
		/* only PS is interrupted */
		
		/* check if this is triggered by background ambient noise */
		ch0data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH0DATAL_REG);
		if (ch0data < (75*(apds993x_als_res_tb[data->als_atime_index]))/100)
			apds993x_change_ps_threshold(client);
		else {
			if (data->ps_detection == 1) {
				apds993x_change_ps_threshold(client);			
			}
			else {
				printk("Triggered by background ambient noise\n");
			}
		}

		apds993x_set_command(client, 0);	/* 0 = CMD_CLR_PS_INT */
	}
	else if ((status & enable & 0x10) == 0x10) {
		/* only ALS is interrupted */	
		apds993x_change_als_threshold(client);

		apds993x_set_command(client, 1);	/* 1 = CMD_CLR_ALS_INT */
	}

	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG, data->enable);	
}

/* assume this is ISR */
static irqreturn_t apds993x_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds993x_data *data = i2c_get_clientdata(client);

	printk("==> apds993x_interrupt\n");
	apds993x_reschedule_work(data, 0);	

	return IRQ_HANDLED;
}

/*
 * IOCTL support
 */
static int apds993x_enable_als_sensor(struct i2c_client *client, int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
#ifdef ALS_POLLING_ENABLED
 	unsigned long flags;
#endif
	
	printk("%s: enable als sensor ( %d)\n", __func__, val);
	
	if ((val != 0) && (val != 1)) {
		printk("%s: enable als sensor=%d\n", __func__, val);
		return -1;
	}
	
	if(val == 1) {
		// turn on light  sensor
		if (data->enable_als_sensor==0) {

			data->enable_als_sensor = 1;
		
			apds993x_set_enable(client,0); /* Power Off */
				
#ifdef ALS_POLLING_ENABLED	
			if (data->enable_ps_sensor) {
				apds993x_set_enable(client, 0x27);	 /* Enable PS with interrupt */
			}
			else {
				apds993x_set_enable(client, 0x03);	 /* no interrupt*/
			}
#else
			apds993x_set_ailt( client, 0xFFFF);	// force first ALS interrupt in order to get environment reading
			apds993x_set_aiht( client, 0);
	
			if (data->enable_ps_sensor) {
				apds993x_set_enable(client, 0x37);	 /* Enable both ALS and PS with interrupt */
			}
			else {
				apds993x_set_enable(client, 0x13);	 /* only enable light sensor with interrupt*/
			}
#endif
		
#ifdef ALS_POLLING_ENABLED

			spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
		
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);
			schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
		
			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);

#endif	// ALS_POLLING_ENABLED

		}
	}
	else {
		//turn off light sensor
		// what if the p sensor is active?
		data->enable_als_sensor = 0;

		if (data->enable_ps_sensor) {
			apds993x_set_enable(client,0); /* Power Off */
			
			apds993x_set_piht(client, 0);
			apds993x_set_piht(client, APDS993X_PS_DETECTION_THRESHOLD);

			apds993x_set_enable(client, 0x27);	 /* only enable prox sensor with interrupt */			
		}
		else {
			apds993x_set_enable(client, 0);
		}
				
#ifdef ALS_POLLING_ENABLED

		spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
		
		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		__cancel_delayed_work(&data->als_dwork);
		
		spin_unlock_irqrestore(&data->update_lock.wait_lock, flags); 

#endif	// ALS_POLLING_ENABLED

	}
	
	return 0;
}

#ifdef ALS_POLLING_ENABLED
static int apds993x_set_als_poll_delay(struct i2c_client *client, unsigned int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	int atime_index=0;
 	unsigned long flags;
	
	printk("%s : %d\n", __func__, val);

	if (val<3000)
		val = 3000;	// minimum 5ms
	
	data->als_poll_delay = val/1000;	// convert us => ms

	if (data->als_poll_delay >= 100) {
		atime_index = APDS993X_ALS_RES_37888;
	}
	else if (data->als_poll_delay >= 50) {
		atime_index = APDS993X_ALS_RES_19456;
	}
	else {
		atime_index = APDS993X_ALS_RES_10240;
	}

	ret = apds993x_set_atime(client, apds993x_als_atime_tb[atime_index]);
	if (ret >= 0) {
		data->als_atime_index = atime_index;
		printk("poll delay %d, atime_index %d\n", data->als_poll_delay, data->als_atime_index);
	}
	else
		return -1;	

	/* we need this polling timer routine for sunlight canellation */
	spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->als_dwork);
	schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
			
	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);	
	
	return 0;
}
#endif

static int apds993x_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
#ifdef ALS_POLLING_ENABLED
 	unsigned long flags;
#endif
	
	printk("enable ps senosr ( %d)\n", val);
	
	if ((val != 0) && (val != 1)) {
		printk("%s:store unvalid value=%d\n", __func__, val);
		return -1;
	}
	
	if(val == 1) {
		//turn on p sensor
		if (data->enable_ps_sensor==0) {

			data->enable_ps_sensor= 1;
		
			apds993x_set_enable(client,0); /* Power Off */
		
			apds993x_set_pilt(client, 0);		// init threshold for proximity
			apds993x_set_piht(client, APDS993X_PS_DETECTION_THRESHOLD);
			
			if (data->enable_als_sensor==0) {
				apds993x_set_enable(client, 0x27);	 /* only enable PS interrupt */
			}
			else {
#ifdef ALS_POLLING_ENABLED
				apds993x_set_enable(client, 0x27);	 /* enable PS interrupt */
#else
				apds993x_set_enable(client, 0x37);	 /* enable ALS and PS interrupt */
#endif	// ALS_POLLING_ENABLED
			}
		}
	} 
	else {
		//turn off p sensor - kk 25 Apr 2011 we can't turn off the entire sensor, the light sensor may be needed by HAL
		data->enable_ps_sensor = 0;
		if (data->enable_als_sensor) {
#ifdef ALS_POLLING_ENABLED
			apds993x_set_enable(client, 0x03);	 /* no ALS interrupt */

			spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
			
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);
			schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));	// 100ms
			
			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);

#else
			// reconfigute light sensor setting			
			apds993x_set_enable(client,0); /* Power Off */
			apds993x_set_ailt( client, 0xFFFF);	// Force ALS interrupt
			apds993x_set_aiht( client, 0);
						
			apds993x_set_enable(client, 0x13);	 /* enable ALS interrupt */
			
#endif	// ALS_POLLING_ENABLED
			
		}
		else {
			apds993x_set_enable(client, 0);

#ifdef ALS_POLLING_ENABLED

			spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
			
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);
		
			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags); 

#endif	// ALS_POLLING_ENABLED

		}
	}
	
	return 0;
}

static int apds993x_ps_open(struct inode *inode, struct file *file)
{
//	printk("apds993x_ps_open\n");
	return 0; 
}

static int apds993x_ps_release(struct inode *inode, struct file *file)
{
//	printk("apds993x_ps_release\n");
	return 0;
}

#ifdef IOCTOLL
static int apds993x_ps_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    struct apds993x_data *data;
    struct i2c_client *client;
    int enable;
    int ret = -1;

    if (arg == 0) return -1;

    if(apds993x_i2c_client == NULL) {
		printk("apds993x_ps_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

    client = apds993x_i2c_client;   
    data = i2c_get_clientdata(apds993x_i2c_client);

    switch (cmd) {
	case APDS993X_IOCTL_PS_ENABLE:              

		if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
			printk("apds993x_ps_ioctl: copy_from_user failed\n");
			return -EFAULT;
		}

		ret = apds993x_enable_ps_sensor(client, enable);        
		if(ret < 0) {
			return ret;
		}
		break;

     	case APDS993X_IOCTL_PS_GET_ENABLE:
		if (copy_to_user((void __user *)arg, &data->enable_ps_sensor, sizeof(data->enable_ps_sensor))) {
			printk("apds993x_ps_ioctl: copy_to_user failed\n");
			return -EFAULT;
		}

		break;

        case APDS993X_IOCTL_PS_GET_PDATA:

		data->ps_data =	i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PDATAL_REG);

		if (copy_to_user((void __user *)arg, &data->ps_data, sizeof(data->ps_data))) {
			printk("apds993x_ps_ioctl: copy_to_user failed\n");
			return -EFAULT;
		}
		break;

	default:
		break;
    }

	
    return 0;
}
#endif

static int apds993x_als_open(struct inode *inode, struct file *file)
{
//	printk("apds993x_als_open\n");
	return 0;
}

static int apds993x_als_release(struct inode *inode, struct file *file)
{
//	printk("apds993x_als_release\n");
	return 0;
}

#ifdef IOCTOLL
static int apds993x_als_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct apds993x_data *data;
	struct i2c_client *client;
	int enable;
	int ret = -1;
    
#ifdef ALS_POLLING_ENABLED
	unsigned int delay;
#endif

	if (arg == 0) return -1;

	if(apds993x_i2c_client == NULL){    
		printk("apds993x_als_ioctl error: i2c driver not installed\n");
		return -EFAULT;
	}

	client = apds993x_i2c_client;   
	data = i2c_get_clientdata(apds993x_i2c_client);

	switch (cmd) {

	case APDS993X_IOCTL_ALS_ENABLE:
		
		if (copy_from_user(&enable, (void __user *)arg, sizeof(enable))) {
			printk("apds993x_als_ioctl: copy_from_user failed\n");
			return -EFAULT;
		}

		ret = apds993x_enable_als_sensor(client, enable); 
		if(ret < 0){
			return ret;
		}
		break;

#ifdef ALS_POLLING_ENABLED
        case APDS993X_IOCTL_ALS_DELAY:
	
		if (copy_from_user(&delay, (void __user *)arg, sizeof(delay))) {
			printk("apds993x_als_ioctl: copy_to_user failed\n");
			return -EFAULT;
		}
        
		ret = apds993x_set_als_poll_delay (client, delay); 
		if(ret < 0){
			return ret;
		}
		break;
#endif

        case APDS993X_IOCTL_ALS_GET_ENABLE:
		if (copy_to_user((void __user *)arg, &data->enable_als_sensor, sizeof(data->enable_als_sensor))) {
			printk("apds993x_als_ioctl: copy_to_user failed\n");
			return -EFAULT;
		}
		break;

        case APDS993X_IOCTL_ALS_GET_CH0DATA:

		data->als_data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH0DATAL_REG);

            	if (copy_to_user((void __user *)arg, &data->als_data, sizeof(data->als_data))) {
			printk("apds993x_ps_ioctl: copy_to_user failed\n");
			return -EFAULT;
		}
		break;

        case APDS993X_IOCTL_ALS_GET_CH1DATA:

		data->als_data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH1DATAL_REG);

            	if (copy_to_user((void __user *)arg, &data->als_data, sizeof(data->als_data))) {
			printk("apds993x_ps_ioctl: copy_to_user failed\n");
			return -EFAULT;
		}
		break;

	default:
		break;
	}

	return 0;
}
#endif

/*
 * SysFS support
 */

static ssize_t apds993x_show_ch0data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ch0data;

	mutex_lock(&data->update_lock);
	ch0data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH0DATAL_REG);
	mutex_unlock(&data->update_lock);
	
	return sprintf(buf, "%d\n", ch0data);
}

static DEVICE_ATTR(ch0data, S_IRUGO,
		   apds993x_show_ch0data, NULL);

static ssize_t apds993x_show_ldata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ch1data;
	int ch0data;
	int luxValue=0;

	mutex_lock(&data->update_lock);
	ch0data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH0DATAL_REG);
	mutex_unlock(&data->update_lock);

	mutex_lock(&data->update_lock);
	ch1data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH1DATAL_REG);
	mutex_unlock(&data->update_lock);	

	luxValue = LuxCalculation(client, ch0data, ch1data);

	return sprintf(buf, "%d\n", luxValue);
}

static DEVICE_ATTR(ldata, S_IRUGO,
		   apds993x_show_ldata, NULL);


static ssize_t apds993x_show_ch1data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ch1data;

	mutex_lock(&data->update_lock);
	ch1data = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_CH1DATAL_REG);
	mutex_unlock(&data->update_lock);
	
	return sprintf(buf, "%d\n", ch1data);
}

static DEVICE_ATTR(ch1data, S_IRUGO,
		   apds993x_show_ch1data, NULL);


static ssize_t apds993x_show_pdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	int pdata;

	mutex_lock(&data->update_lock);
	pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PDATAL_REG);
	mutex_unlock(&data->update_lock);
	
	return sprintf(buf, "%d\n", pdata);
}

static DEVICE_ATTR(pdata, S_IRUGO,
		   apds993x_show_pdata, NULL);


#ifdef APDS993X_HAL_USE_SYS_ENABLE
static ssize_t apds993x_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds993x_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct apds993x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: enable ps senosr ( %ld)\n", __func__, val);
	
	if ((val != 0) && (val != 1)) {
		printk("%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}

	apds993x_enable_ps_sensor(client, val);	
	
	return count;
}

static DEVICE_ATTR(enable_ps_sensor, S_IWUSR | S_IRUGO,
		apds993x_show_enable_ps_sensor, apds993x_store_enable_ps_sensor);

static ssize_t apds993x_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds993x_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct apds993x_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	printk("%s: enable als sensor ( %ld)\n", __func__, val);
	
	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}

	apds993x_enable_als_sensor(client, val); 

	
	return count;
}

static DEVICE_ATTR(enable_als_sensor, S_IWUSR | S_IRUGO,
		apds993x_show_enable_als_sensor, apds993x_store_enable_als_sensor);

static ssize_t apds993x_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->als_poll_delay*1000);	// return in micro-second
}

static ssize_t apds993x_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef ALS_POLLING_ENABLED
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	apds993x_set_als_poll_delay(client, val);

#endif

	return count;
}

static DEVICE_ATTR(als_poll_delay, S_IWUSR | S_IRUGO,
				   apds993x_show_als_poll_delay, apds993x_store_als_poll_delay);

#endif

static struct attribute *apds993x_attributes[] = {
	&dev_attr_ch0data.attr,
	&dev_attr_ch1data.attr,
	&dev_attr_pdata.attr,
	&dev_attr_ldata.attr,
#ifdef APDS993X_HAL_USE_SYS_ENABLE
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_poll_delay.attr,
#endif
	NULL
};

static const struct attribute_group apds993x_attr_group = {
	.attrs = apds993x_attributes,
};

static struct file_operations apds993x_ps_fops = {
	.owner = THIS_MODULE,
	.open = apds993x_ps_open,
	.release = apds993x_ps_release,
#ifdef IOCTOLL
	.ioctl = apds993x_ps_ioctl,
#endif
};

static struct miscdevice apds993x_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "apds993x_ps_dev",
	.fops = &apds993x_ps_fops,
};

static struct file_operations apds993x_als_fops = {
	.owner = THIS_MODULE,
	.open = apds993x_als_open,
	.release = apds993x_als_release,
#ifdef IOCTOLL
	.ioctl = apds993x_als_ioctl,
#endif
};

static struct miscdevice apds993x_als_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "apds993x_als_dev",
	.fops = &apds993x_als_fops,
};

/*
 * Initialization function
 */

static int apds993x_init_client(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int err;
	int id;

	err = apds993x_set_enable(client, 0);

	if (err < 0)
		return err;
	
	id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_ID_REG);
	if (id == 0x30) {
		printk("APDS-9931\n");
	}
	else if (id == 0x39) {
		printk("APDS-9930\n");
	}
	else {
		printk("Neither APDS-9931 nor APDS-9930\n");
		return -EIO;
	}

	err = apds993x_set_atime(client, apds993x_als_atime_tb[data->als_atime_index]);	// 100.64ms ALS integration time
	if (err < 0) return err;

	err = apds993x_set_ptime(client, 0xFF);	// 2.72ms Prox integration time
	if (err < 0) return err;

	err = apds993x_set_wtime(client, 0xFF);	// 2.72ms Wait time
	if (err < 0) return err;

	err = apds993x_set_ppcount(client, APDS993X_PS_PULSE_NUMBER);	
	if (err < 0) return err;

	err = apds993x_set_config(client, 0);		// no long wait
	if (err < 0) return err;

	err = apds993x_set_control(client, APDS993X_PDRVIE_100MA|APDS993X_PRX_IR_DIOD|APDS993X_PGAIN_4X|apds993x_als_again_bit_tb[data->als_again_index]);
	if (err < 0) return err;

	err = apds993x_set_pilt(client, 0);		// init threshold for proximity
	if (err < 0) return err;

	err = apds993x_set_piht(client, APDS993X_PS_DETECTION_THRESHOLD);
	if (err < 0) return err;

	err = apds993x_set_ailt(client, 0xFFFF);	// force first ALS interrupt to get the environment reading
	if (err < 0) return err;

	err = apds993x_set_aiht(client, 0);
	if (err < 0) return err;

	err = apds993x_set_pers(client, APDS993X_PPERS_2|APDS993X_APERS_2);	// 2 consecutive Interrupt persistence
	if (err < 0) return err;

	// sensor is in disabled mode but all the configurations are preset

	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver apds993x_driver;
static int __devinit apds993x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds993x_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds993x_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->client = client;
	apds993x_i2c_client = client;

	i2c_set_clientdata(client, data);

	data->enable = 0;	/* default mode is standard */
	data->ps_threshold = APDS993X_PS_DETECTION_THRESHOLD;
	data->ps_hysteresis_threshold = APDS993X_PS_HSYTERESIS_THRESHOLD;
	data->ps_detection = 0;	/* default to no detection */
	data->enable_als_sensor = 0;	// default to 0
	data->enable_ps_sensor = 0;	// default to 0
	data->als_poll_delay = 100;	// default to 100ms
	data->als_atime_index = APDS993X_ALS_RES_37888;	// 100ms ATIME
	data->als_again_index = APDS993X_ALS_GAIN_8X;	// 8x AGAIN
	data->als_reduce = 0;	// no ALS 6x reduction
	data->als_prev_lux = 0;
	
	mutex_init(&data->update_lock);

	if (request_irq(APDS993X_INT, apds993x_interrupt, IRQF_DISABLED|IRQ_TYPE_EDGE_FALLING,
		APDS993X_DRV_NAME, (void *)client)) {
		printk("%s Could not allocate APDS993X_INT !\n", __func__);
	
		goto exit_kfree;
	}

	INIT_DELAYED_WORK(&data->dwork, apds993x_work_handler);

#ifdef ALS_POLLING_ENABLED

	INIT_DELAYED_WORK(&data->als_dwork, apds993x_als_polling_work_handler); 

#endif	// ALS_POLLING_ENABLED

	printk("%s interrupt is hooked\n", __func__);

	/* Initialize the APDS993X chip */
	err = apds993x_init_client(client);
	if (err)
		goto exit_kfree;

	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		printk("Failed to allocate input device als\n");
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		printk("Failed to allocate input device ps\n");
		goto exit_free_dev_als;
	}
	
	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 30000, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_als->name = "Avago light sensor";
	data->input_dev_ps->name = "Avago proximity sensor";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		printk("Unable to register input device als: %s\n",
		       data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("Unable to register input device ps: %s\n",
		       data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds993x_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	/* Register for sensor ioctl */
    err = misc_register(&apds993x_ps_device);
	if (err) {
		printk("Unalbe to register ps ioctl: %d", err);
		goto exit_remove_sysfs_group;
	}

    err = misc_register(&apds993x_als_device);
	if (err) {
		printk("Unalbe to register als ioctl: %d", err);
		goto exit_unregister_ps_ioctl;
	}

	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);

	return 0;

exit_unregister_ps_ioctl:
	misc_deregister(&apds993x_ps_device);
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &apds993x_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);	
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
exit_free_dev_als:
exit_free_irq:
	free_irq(APDS993X_INT, client);	
exit_kfree:
	kfree(data);
exit:
	return err;
}

static int __devexit apds993x_remove(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);

	/* Power down the device */
	apds993x_set_enable(client, 0);

	misc_deregister(&apds993x_als_device);
	misc_deregister(&apds993x_ps_device);	

	sysfs_remove_group(&client->dev.kobj, &apds993x_attr_group);

	input_unregister_device(data->input_dev_ps);
	input_unregister_device(data->input_dev_als);
	
	free_irq(APDS993X_INT, client);

	kfree(data);

	return 0;
}

#ifdef CONFIG_PM

static int apds993x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int err=0;
#ifdef ALS_POLLING_ENABLED
	unsigned long flags;
#endif	// ALS_POLLING_ENABLED

	if (data->enable_als_sensor || data->enable_ps_sensor ) {
		err = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG, 0);
		
		if (err < 0)
			return err;

#ifdef ALS_POLLING_ENABLED
		if (data->enable_als_sensor) {
			spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
			__cancel_delayed_work(&data->als_dwork);		
			spin_unlock_irqrestore(&data->update_lock.wait_lock, flags); 
		}
#endif	// ALS_POLLING_ENABLED		
	}

	return err;
}

static int apds993x_resume(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int err=0;
#ifdef ALS_POLLING_ENABLED
	unsigned long flags;
#endif	// ALS_POLLING_ENABLED

	if (data->enable_als_sensor || data->enable_ps_sensor) {
	
		err = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG, data->enable);	
		if (err < 0)
			return err;

#ifdef ALS_POLLING_ENABLED
		spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
		__cancel_delayed_work(&data->als_dwork);
		schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
			
		spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
#endif	// ALS_POLLING_ENABLED		
				
	}

	return err;
}

#else

#define apds993x_suspend	NULL
#define apds993x_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds993x_id[] = {
	{ "apds993x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds993x_id);

static struct i2c_driver apds993x_driver = {
	.driver = {
		.name	= APDS993X_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = apds993x_suspend,
	.resume	= apds993x_resume,
	.probe	= apds993x_probe,
	.remove	= __devexit_p(apds993x_remove),
	.id_table = apds993x_id,
};

static int __init apds993x_init(void)
{
	return i2c_add_driver(&apds993x_driver);
}

static void __exit apds993x_exit(void)
{
	i2c_del_driver(&apds993x_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS993X ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds993x_init);
module_exit(apds993x_exit);

