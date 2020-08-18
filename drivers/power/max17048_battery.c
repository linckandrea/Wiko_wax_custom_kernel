/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *  Chandler Zhang <chazhang@nvidia.com>
 *  Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/max17048_battery.h>
#include <linux/power/battery-charger-gauge-comm.h>
#include <linux/pm.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>

#define MAX17048_VCELL		0x02
#define MAX17048_SOC		0x04
#define MAX17048_MODE		0x06
#define MAX17048_VER		0x08
#define MAX17048_HIBRT		0x0A
#define MAX17048_CONFIG		0x0C
#define MAX17048_OCV		0x0E
#define MAX17048_VALRT		0x14
#define MAX17048_VRESET		0x18
#define MAX17048_STATUS		0x1A
#define MAX17048_UNLOCK		0x3E
#define MAX17048_TABLE		0x40
#define MAX17048_RCOMPSEG1	0x80
#define MAX17048_RCOMPSEG2	0x90
#define MAX17048_CMD		0xFF
#define MAX17048_UNLOCK_VALUE	0x4a57
#define MAX17048_RESET_VALUE	0x5400
#define MAX17048_DELAY		(20*HZ)
#define MAX17048_BATTERY_FULL	100
#define MAX17048_BATTERY_LOW	15
#define MAX17048_VERSION_NO	0x11
#define TOPOFF_TIME_COUNT 90
#if (CONFIG_MACH_S9321 == 1)
#define BATTERY_MAX_OCV 4350000
#define BATTERY_RECHARGE_OCV 4300000
#define BATTERY_RECHARGE_VCELL 4250
#define BATTERY_SOFTWARE_POWER_OFF_LEVEL 3480
#define BATTERY_HARDWARE_POWER_OFF_LEVEL 3200
#else
#define BATTERY_MAX_OCV 4200000
#define BATTERY_RECHARGE_OCV 4180000
#define BATTERY_RECHARGE_VCELL 4175
#endif
extern void max77660_power_forceoff(void);
//static int max_fg_w[128];
//static uint8_t g_17048_fg_byte[128];

/*Ivan battery special config*/
#define MAX17048_SOC_AVERAGE		/*Default Set: Set if you want to smooth the SOC change*/
#define MAX17048_RECHARGER_HANDLE	/*Default Set: Set if you want charger continuous to turn on for while after 100% and re-charger only when battery voltage below BATTERY_RECHARGE_VCELL*/
#define MAX17048_FAKE_FULL_HANDLE	/*Special handle for S9321, Full will be set if battery voltage >=99% and last for 15 minutes*/



#define TN_BATT_HOT_STOP_TEMPERATURE		62 - 1
#define TN_BATT_HOT_RESTART_TEMPERATURE		57

#define VCELL_LEN	10
#define VSOC_LEN	10
#define TN_FAKE_FULL_COUNT 	45	//15 minutes

static long g_fg_record_time;
static int g_vcell_fifo[VCELL_LEN];
static int g_vcell_fifo_init = 0;
static int g_bat_temperature = 0xffff;

#ifdef MAX17048_SOC_AVERAGE
static int g_soc_fifo[VSOC_LEN];
static int g_soc_fifo_init = 0;
static struct timeval g_charger_discon_time;
static struct timeval g_previous_time;
#endif

static long g_soc_special_counter = 0;

static g_Batt_VL_IRQ_Count = 0;

extern int tegra_get_bootloader_fg_status(void);

//extern int get_tn_bat_test_count(void);

struct max17048_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		battery;
	struct max17048_platform_data *pdata;
	struct battery_gauge_dev	*bg_dev;

	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int raw_soc;
	/* Raw soc value read from register */
	int status;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;

	int lasttime_soc;
	int lasttime_status;
	int shutdown_complete;
	int charge_complete;
	int is_recharged;
	struct mutex mutex;
	int irq;
};
struct max17048_chip *max17048_data;

static int max17048_update_battery_status(struct battery_gauge_dev *bg_dev,
		enum battery_charger_status status);


static int max17048_write_word(struct i2c_client *client, int reg, u16 value)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}


	ret = i2c_smbus_write_word_data(client, reg, swab16(value));

	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x err %d\n", __func__, reg, ret);

	mutex_unlock(&chip->mutex);
	return ret;
}


static int max17048_write_block(const struct i2c_client *client,
		uint8_t command, uint8_t length, const uint8_t *values)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}

	ret = i2c_smbus_write_i2c_block_data(client, command, length, values);
	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing block data to"
				"0x%02x err %d\n", __func__, command, ret);
	mutex_unlock(&chip->mutex);
	return ret;
}


static int max17048_read_word(struct i2c_client *client, int reg)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
					"0x%02x err %d\n", __func__, reg, ret);

		mutex_unlock(&chip->mutex);
		return ret;
	} else {
		ret = (int)swab16((uint16_t)(ret & 0x0000ffff));

		mutex_unlock(&chip->mutex);
		return ret;

	}
}

/* Return value in uV */
static int max17048_get_ocv(struct max17048_chip *chip)
{
	int r;
	int reg;
	int ocv;

	r = max17048_write_word(chip->client, MAX17048_UNLOCK,
			MAX17048_UNLOCK_VALUE);
	if (r)
		return r;

	reg = max17048_read_word(chip->client, MAX17048_OCV);
	ocv = (reg >> 4) * 1250;

	r = max17048_write_word(chip->client, MAX17048_UNLOCK, 0);
	WARN_ON(r);

	return ocv;
}

/* Return value in uV */
static void max17048_debug_RCOMP_Seg(struct max17048_chip *chip)
{
	int r,i;
	int reg[16];
	int status;
	int version;
	int para;

	status = max17048_read_word(chip->client, 0x1A);
	version = max17048_read_word(chip->client, 0x08);
	
	r = max17048_write_word(chip->client, MAX17048_UNLOCK,
			MAX17048_UNLOCK_VALUE);
	if (r)
		return r;
	
	for (i =0; i < 16; i++)
	{
	    reg[i] = max17048_read_word(chip->client, 0x80 +i*2);
	}
	
	para = max17048_read_word(chip->client, 0x40);

	r = max17048_write_word(chip->client, MAX17048_UNLOCK, 0);
	WARN_ON(r);
	printk("\n");
	printk("Max17048_debug: Version[%x], Status[%x], Para[%x]\n",version,status >> 8,para);
	printk("Max17048_debug: RCOMPSeg[%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x]\n",reg[0],reg[1],reg[2],reg[3],reg[4],reg[5],
	       reg[6],reg[7],reg[8],reg[9],reg[10],reg[11],reg[12],reg[13],reg[14],reg[15]);

	
}



static int max17048_rcomp_adjust(struct max17048_chip *chip)
{
	struct max17048_battery_model *mdata = chip->pdata->model_data;
#if CONFIG_MACH_S9321
//Ivan	int rcomp0 = 129;
//Ivan	int tempCoUp = -1.05 * 100;
//Ivan	int tempCoDown = 0 * 100;
	
	int rcomp0 = 161;
	int tempCoUp = -2.05 * 100;
	int tempCoDown = -1.25 * 100;	
#else
	int rcomp0 = 115;
	int tempCoUp = -0.15 * 100;
	int tempCoDown = -4.95 * 100;
#endif
	int rcomp, ret = 0;
	int config_reg;
	int temp;

	ret = battery_gauge_get_battery_temperature(chip->bg_dev, &temp);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Failed to read battery temperature!\n");
		return ret;
	}

	if (g_bat_temperature == temp)
	  return ret;

	g_bat_temperature = temp;
	
	if (temp > 20)
		rcomp = rcomp0 + (int)((temp - 20)*tempCoUp/(100));
	else
		rcomp = rcomp0 + (int)((temp - 20)*tempCoDown/(100));

//Ivan added RCOMP clamped to 120 for temperature above 40 RCOMP(40˚C)
#if CONFIG_MACH_S9321		
	if (temp > 40)
	    rcomp = 120;
#endif

	config_reg = max17048_read_word(chip->client, MAX17048_CONFIG);
	if (config_reg < 0) {
		dev_err(&chip->client->dev, "Error reading config register.\n");
		return config_reg;
	}

	config_reg &= 0x00ff;
	config_reg |= (rcomp << 8);

	ret = max17048_write_word(chip->client, MAX17048_CONFIG, config_reg);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Error writing to config register.\n");
		return ret;
	}

	return ret;
}


static int max17048_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17048_chip *chip = container_of(psy,
				struct max17048_chip, battery);
	int temp;
	int ret;
	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if ( chip->status == POWER_SUPPLY_STATUS_CHARGING)
		{
		    ret = battery_gauge_get_battery_temperature(chip->bg_dev,
								    &temp);
		    if (ret >=0)
		    {
//Ivan test code			
//			if (get_tn_bat_test_count() > 50 && get_tn_bat_test_count() < 150)
//			    temp = 65;
//Ivan
			if (temp >= TN_BATT_HOT_STOP_TEMPERATURE)
			{
			    val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			    printk("Ivan max17048_get_property report disable!!\n");
			}
			else
			    val->intval = chip->status;
		    }
		}
		else
		    val->intval = chip->status;
		    printk("Ivan max17048_get_property status = %d !\n", val->intval);		
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		if (chip->soc == 15)
			dev_warn(&chip->client->dev,
			"/nSystem Running low on battery - 15%\n");
		if (chip->soc == 10)
			dev_warn(&chip->client->dev,
			"/nSystem Running low on battery - 10%\n");
		if (chip->soc == 5)
			dev_warn(&chip->client->dev,
			"/nSystem Running low on battery - 5%\n");
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->capacity_level;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = max17048_get_ocv(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = battery_gauge_get_battery_temperature(chip->bg_dev,
								&temp);
		if (ret < 0)
			return -EINVAL;
		val->intval = temp * 10;
		break;
	default:
	return -EINVAL;
	}

	ret = max17048_rcomp_adjust(chip);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"Temperature adjustment failed.\n");
		return -EINVAL;
	}

	return 0;
}

static void max17048_get_vcell(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int vcell;

	vcell = max17048_read_word(client, MAX17048_VCELL);
	if (vcell < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, vcell);
	else
		chip->vcell = (uint16_t)(((vcell >> 4) * 125) / 100);
}

static void max17048_get_soc(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	struct max17048_platform_data *pdata;
	int soc;
	int ocv;

	static int topoff_count;

	pdata = chip->pdata;
	soc = max17048_read_word(client, MAX17048_SOC);
	if (soc < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, soc);
	else {
		chip->soc = (uint16_t)soc >> 8;
		if (pdata->soc_error_max_value)
			chip->soc =(chip->soc * MAX17048_BATTERY_FULL)
						/ pdata->soc_error_max_value;
	}

	chip->raw_soc = chip->soc;

	if (chip->soc > MAX17048_BATTERY_FULL)
		chip->soc = MAX17048_BATTERY_FULL;

#if defined (MAX17048_FAKE_FULL_HANDLE) && (CONFIG_MACH_S9321 == 1)
	if (chip->status == POWER_SUPPLY_STATUS_CHARGING)
	{
	    if (chip->soc >= 99)
	    {
		g_soc_special_counter ++;
		if (g_soc_special_counter >= TN_FAKE_FULL_COUNT)
		{
			g_soc_special_counter = TN_FAKE_FULL_COUNT;
			chip->soc = 100;
		}
	    }
	}
#endif
	
	
	mutex_lock(&charger_gauge_list_mutex);
	if (chip->soc >= MAX17048_BATTERY_FULL
		&& chip->status == POWER_SUPPLY_STATUS_CHARGING) {
		/*It will take 15 minutes in TOPOFF stage.*/
		if (++topoff_count > TOPOFF_TIME_COUNT) {
			topoff_count = 0;
			max17048_update_battery_status(chip->bg_dev,
					BATTERY_CHARGING_DONE);
			battery_set_charging(chip->bg_dev, false);
			chip->soc = MAX17048_BATTERY_FULL;
		} else {
#ifdef MAX17048_RECHARGER_HANDLE
			max17048_update_battery_status(chip->bg_dev,
					BATTERY_CHARGING_DONE);
			chip->soc = MAX17048_BATTERY_FULL;
#else
			chip->soc = MAX17048_BATTERY_FULL-1;			
#endif
		}
	} else {
	      if (chip->status == POWER_SUPPLY_STATUS_CHARGING)
		topoff_count = 0;
	}
	
		
	if (chip->status == POWER_SUPPLY_STATUS_FULL && chip->charge_complete) {
		ocv = max17048_get_ocv(chip);
		
#ifdef MAX17048_RECHARGER_HANDLE
//Ivan Handling top off here
//		max17048_get_vcell(client);
		if (++topoff_count == TOPOFF_TIME_COUNT) {
			max17048_update_battery_status(chip->bg_dev,
					BATTERY_CHARGING_DONE);
			battery_set_charging(chip->bg_dev, false);
			chip->soc = MAX17048_BATTERY_FULL;
		} else if (topoff_count < TOPOFF_TIME_COUNT){
		    chip->soc = MAX17048_BATTERY_FULL;	//Ivan added
		    printk("Ivan max17048_get_soc TOP_OFF: status = [%d]; topoff_count[%d]; is_recharged[%d]; charge_complete[%d]; g_soc_special_counter[%d]\n ", chip->status,topoff_count,chip->is_recharged,chip->charge_complete,g_soc_special_counter);
		    mutex_unlock(&charger_gauge_list_mutex);
		    return;
		}
		if (chip->vcell < BATTERY_RECHARGE_VCELL && !chip->is_recharged) {	//Ivan recharge depended on vcell voltage
#else
		if (ocv < BATTERY_MAX_OCV && !chip->is_recharged) {
#endif
			battery_set_charging(chip->bg_dev, true);
			chip->is_recharged = 1;
		} else if (ocv >= BATTERY_MAX_OCV && chip->is_recharged && chip->raw_soc >= 100) {	//Ivan Full depended on ocv voltage
			battery_set_charging(chip->bg_dev, false);
			chip->is_recharged = 0;
		}
		chip->soc = MAX17048_BATTERY_FULL;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (chip->soc < MAX17048_BATTERY_LOW) {
		chip->status = chip->lasttime_status;
		chip->health = POWER_SUPPLY_HEALTH_DEAD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		chip->is_recharged = 0;
	} else {
		chip->status = chip->lasttime_status;
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		chip->is_recharged = 0;
	}
	mutex_unlock(&charger_gauge_list_mutex);
	printk("Ivan max17048_get_soc: status = [%d]; topoff_count[%d]; is_recharged[%d]; charge_complete[%d]; vcell[%d]; g_soc_special_counter[%d]\n ", chip->status,topoff_count,chip->is_recharged,chip->charge_complete,chip->vcell,g_soc_special_counter);
}

static int max17048_read_soc_raw_value(struct battery_gauge_dev *bg_dev)
{
	struct max17048_chip *chip = battery_gauge_get_drvdata(bg_dev);
	return chip->raw_soc;
}

static uint16_t max17048_get_version(struct i2c_client *client)
{
	return swab16(max17048_read_word(client, MAX17048_VER));
}

static void max17048_work(struct work_struct *work)
{
	struct max17048_chip *chip;
//Ivan added
	int loop,avg_v;
	int temp;
	int rcomp;
	struct timeval now;
	time_t diff;
	uint16_t status;
	unsigned long local_time;
	struct rtc_time tm;
	
	chip = container_of(work, struct max17048_chip, work.work);
//Ivan added	
//	status = max17048_read_word(chip->client, MAX17048_STATUS);
//	printk("Ivan max17048_work Status[%x]\n",status >> 8 );
//	status = max17048_read_word(chip->client, MAX17048_CONFIG);
//	printk("Ivan max17048_work Config[%x]\n",status & 0xFF );

//Ivan End
	
	max17048_get_vcell(chip->client);
	max17048_get_soc(chip->client);

//Ivan added	
	do_gettimeofday(&now);
	
	local_time = (u32)(now.tv_sec - (sys_tz.tz_minuteswest * 60));
	rtc_time_to_tm(local_time, &tm);

	printk(" @ (%04d-%02d-%02d %02d:%02d:%02d)\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);


	diff = now.tv_sec - g_previous_time.tv_sec;
	
#ifdef MAX17048_SOC_AVERAGE
//Ivan Init the voltage buffer to the initialize voltage value
	avg_v = 0;
	if (g_soc_fifo_init == 0 || diff > 1500 || chip->status == POWER_SUPPLY_STATUS_CHARGING)	//Sleep more than 25 minutes
	{
	    g_soc_fifo_init = 1;
	    for (loop = 0; loop < VSOC_LEN; loop ++)
	      g_soc_fifo[loop] = chip->soc;
	    for (loop = 0; loop < VCELL_LEN; loop ++)
	      g_vcell_fifo[loop] = chip->vcell;	    
	}
//Ivan End
//Ivan add new soc sample
	for (loop = 0; loop < (VSOC_LEN - 1); loop ++)
	    g_soc_fifo[loop] = g_soc_fifo[loop+1];
	
	g_soc_fifo[VSOC_LEN-1] = chip->soc;

//Ivan end
//Ivan average the vcell voltage
	for (loop = 0; loop < VSOC_LEN; loop ++)
	  avg_v += g_soc_fifo[loop];
	
	if (avg_v > VSOC_LEN*80)
	  avg_v += (VSOC_LEN - 1);		//stay at 100% if SOC > 99.2%
	chip->soc = avg_v/VSOC_LEN;
	
#endif
	g_previous_time = now;

	printk("Ivan time pass = %lu \n",diff);
	
	battery_gauge_get_battery_temperature(chip->bg_dev,&temp);
	rcomp = max17048_read_word(chip->client, 0x0c);
	g_fg_record_time+=20;
	printk("\n");
	printk("Ivan max17048_work vcell[%d], soc[%d], raw_soc[%d], temp[%d], rcomp[%d]\n",chip->vcell,chip->soc,chip->raw_soc,temp,rcomp>>8 );
	printk("MAX17048_FG:%6d,%6d,%6d,%6d,%6d\n",g_fg_record_time,chip->vcell,temp,chip->raw_soc,rcomp>>8);
	
//Ivan print debug information for Maxim
	if (g_fg_record_time == 100)
	    max17048_debug_RCOMP_Seg(chip);
	
	schedule_delayed_work(&chip->work, MAX17048_DELAY);

//Init the voltage buffer to the initialize voltage value
	avg_v = 0;
	if (g_vcell_fifo_init == 0)
	{
	    g_vcell_fifo_init = 1;
	    for (loop = 0; loop < VCELL_LEN; loop ++)
	      g_vcell_fifo[loop] = chip->vcell;
	}
//Ivan average the vcell voltage
	for (loop = 0; loop < (VCELL_LEN - 1); loop ++)
	  g_vcell_fifo[loop] = g_vcell_fifo[loop+1];
	
	g_vcell_fifo[VCELL_LEN-1] = chip->vcell;

	for (loop = 0; loop < VCELL_LEN; loop ++)
	  avg_v += g_vcell_fifo[loop];
	
	avg_v = avg_v/VCELL_LEN;
	
	if (chip->status == POWER_SUPPLY_STATUS_DISCHARGING)
	{
#if (CONFIG_MACH_S9321 == 1)    
	  if (avg_v < BATTERY_SOFTWARE_POWER_OFF_LEVEL + 10)
#else	    
	  if (avg_v < 3510)
#endif
	  {
	    printk("Ivan Battery < 3510, Android power off...\n");	   
#if (CONFIG_MACH_S9321 == 1)    	    
	    if (chip->soc > 0)
	    {
		printk("Ivan Battery Reset FG...\n");	    
		max17048_write_word(chip->client, MAX17048_MODE, 0x4000);	//Reset FG
	    }
#endif	    
	    chip->soc = 0;
	  }
	  
#if (CONFIG_MACH_S9321 == 1)    
	  if (avg_v < BATTERY_HARDWARE_POWER_OFF_LEVEL)
#else	    	  
	  if (avg_v < 3500 && chip->soc != 0)
#endif
	  {
	    printk("Ivan Battery too low (3.5V), force power off...\n");	
#if (CONFIG_MACH_S9321 == 1)    	    
	    if (chip->soc > 0)
	    {
		printk("Ivan Battery Reset FG...\n");		
		max17048_write_word(chip->client, MAX17048_MODE, 0x4000);	//Reset FG	    
	    }
#endif	    
	    mdelay(200);	    
	    max77660_power_forceoff();
	  }
	}
	
	if (chip->soc != chip->lasttime_soc ||
		chip->status != chip->lasttime_status) {
		chip->lasttime_soc = chip->soc;
		power_supply_changed(&chip->battery);
	}
}

void max17048_battery_status(int status)
{
	if (!max17048_data)
		return;

	if (status == progress)
		max17048_data->status = POWER_SUPPLY_STATUS_CHARGING;
	else
		max17048_data->status = POWER_SUPPLY_STATUS_DISCHARGING;

	max17048_data->lasttime_status = max17048_data->status;
	power_supply_changed(&max17048_data->battery);
}
EXPORT_SYMBOL_GPL(max17048_battery_status);

static enum power_supply_property max17048_battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TEMP,
};

static int max17048_write_rcomp_seg(struct i2c_client *client,
						uint16_t rcomp_seg)
{
	uint8_t rs1, rs2;
	int ret;
	uint8_t rcomp_seg_table[16];

	rs1 = (rcomp_seg >> 8) & 0xff;
	rs2 = rcomp_seg & 0xff;

	rcomp_seg_table[0] = rcomp_seg_table[2] = rcomp_seg_table[4] =
		rcomp_seg_table[6] = rcomp_seg_table[8] = rcomp_seg_table[10] =
			rcomp_seg_table[12] = rcomp_seg_table[14] = rs1;

	rcomp_seg_table[1] = rcomp_seg_table[3] = rcomp_seg_table[5] =
		rcomp_seg_table[7] = rcomp_seg_table[9] = rcomp_seg_table[11] =
			rcomp_seg_table[13] = rcomp_seg_table[15] = rs2;

	ret = max17048_write_block(client, MAX17048_RCOMPSEG1,
				16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	ret = max17048_write_block(client, MAX17048_RCOMPSEG2,
				16, (uint8_t *)rcomp_seg_table);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int max17048_load_model_data(struct max17048_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct max17048_battery_model *mdata = chip->pdata->model_data;
	uint16_t soc_tst, ocv;
	int i, ret = 0;

	/* read OCV */
	ret = max17048_read_word(client, MAX17048_OCV);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	ocv = (uint16_t)ret;
	printk("@@xu: ocv:%d\n",(ret >> 4) * 1250);	
	if (ocv == 0xffff) {
		dev_err(&client->dev, "%s: Failed in unlocking"
					"max17048 err: %d\n", __func__, ocv);
		return -1;
	}

	/* write custom model data */
	for (i = 0; i < 4; i += 1) {
		if (max17048_write_block(client,
			(MAX17048_TABLE+i*16), 16,
				&mdata->data_tbl[i*0x10]) < 0) {
			dev_err(&client->dev, "%s: error writing model data:\n",
								__func__);
			return -1;
		}
	}

	/* Write OCV Test value */
	ret = max17048_write_word(client, MAX17048_OCV, mdata->ocvtest);
	if (ret < 0)
		return ret;

	ret = max17048_write_rcomp_seg(client, mdata->rcomp_seg);
	if (ret < 0)
		return ret;

	/* Disable hibernate */
	ret = max17048_write_word(client, MAX17048_HIBRT, 0x0000);
	if (ret < 0)
		return ret;

	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;

	/* Delay between 150ms to 600ms */
	mdelay(200);

	/* Read SOC Register and compare to expected result */
	ret = max17048_read_word(client, MAX17048_SOC);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	soc_tst = (uint16_t)ret;
	if (!((soc_tst >> 8) >= mdata->soccheck_A &&
				(soc_tst >> 8) <=  mdata->soccheck_B)) {
		dev_err(&client->dev, "%s: soc comparison failed %d\n",
					__func__, ret);
		return ret;
	} else {
		dev_info(&client->dev, "MAX17048 Custom data"
						" loading successfull\n");
	}

	/* unlock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK,
					MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

	/* Restore OCV */
	ret = max17048_write_word(client, MAX17048_OCV, ocv);
	if (ret < 0)
		return ret;

	return ret;
}

static int max17048_initialize(struct max17048_chip *chip)
{
	uint8_t ret, config = 0;
	struct i2c_client *client = chip->client;
	struct max17048_battery_model *mdata = chip->pdata->model_data;
	uint16_t status;
	uint16_t vcell,ocv,rcomp;
	uint16_t bl_status;

//Ivan
	do_gettimeofday(&g_previous_time);
	bl_status = tegra_get_bootloader_fg_status();
	printk("Ivan max17048_initialize bootloader fg status[%x]\n",bl_status );

	status = max17048_read_word(client, MAX17048_STATUS);
//Ivan 01 or 09
	printk("Ivan max17048_initialize Status[%x]\n",status >> 8 );

	if (((status >> 8) & 0x01) == 0) {
		dev_info(&client->dev, "don't need initialise fuel gauge.\n");
		
	vcell = max17048_read_word(client, MAX17048_VCELL);
	rcomp = max17048_read_word(client, 0x0c);
	rcomp = rcomp >> 8;
	printk("Ivan max17048_initialize vcell[%d],rcomp[%d]\n",vcell,rcomp );

//Ivan only init VRESET value		
	ret = max17048_write_word(client, MAX17048_UNLOCK,
			MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

	/* Voltage Alert configuration */
	ret = max17048_write_word(client, MAX17048_VALRT, mdata->valert);
	if (ret < 0)
		return ret;

//Ivan added battery config
//Ivan	config = mdata->one_percent_alerts | config;
      
	ret = max17048_write_word(client, MAX17048_CONFIG,
			((mdata->rcomp << 8) | config));		
	if (ret < 0)
		return ret;		

//Clear status register	
	status = status & ~0x0100;
	/* set EnVR as 1 */
	ret = max17048_write_word(client, MAX17048_STATUS,
			status);
	if (ret < 0)
		return ret;
	
	status = max17048_read_word(client, MAX17048_VALRT);
	printk("Ivan max17048_initialize valert[%x] reading[%x]\n",mdata->valert, status );
//Ivan end

//Ivan S9321 non swapable battery
#if (CONFIG_MACH_S9321 != 1)
	ocv = max17048_read_word(client, MAX17048_OCV);
	if (ocv < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ocv);
		return ret;
	}
	printk("max17048_initialize: ocv:%d\n",ocv);	
	
	if ((vcell + 650) > ocv  && rcomp == 151)		//around 50mV
	{
	  if (!(bl_status & 0x100)/* && (bl_status & 0x01)*/)	//reset and no charger
	  {
	    printk("max17048_initialize: rewrite OCV!\n");	

	    max17048_write_word(client, MAX17048_OCV, vcell + 650);
	  }
	}
#endif
	
	ret = max17048_write_word(client, MAX17048_VRESET, mdata->vreset);
	if (ret < 0)
		return ret;
	
	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;	
//Ivan END
	if ((vcell + 650) > ocv)
	  mdelay(200);

	return 0;
	}
	dev_info(&client->dev, "initialise fuel gauge.\n");
	/* unlock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK,
			MAX17048_UNLOCK_VALUE);
	if (ret < 0)
		return ret;

	/* load model data */
	ret = max17048_load_model_data(chip);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	status = max17048_read_word(client, MAX17048_STATUS);
	status = status & ~0x0100;
	/* set EnVR as 1 */
//Ivan 	status |= 0x4000;

	ret = max17048_write_word(client, MAX17048_STATUS,
			status);
	if (ret < 0)
		return ret;

	if (mdata->bits == 19)
		config = 32 - (mdata->alert_threshold * 2);
	else if (mdata->bits == 18)
		config = 32 - mdata->alert_threshold;

//Ivan disable one percentage changes alert
//Ivan	config = mdata->one_percent_alerts | config;
      
	ret = max17048_write_word(client, MAX17048_CONFIG,
			((mdata->rcomp << 8) | config));
	if (ret < 0)
		return ret;

	/* Voltage Alert configuration */
	ret = max17048_write_word(client, MAX17048_VALRT, mdata->valert);
	if (ret < 0)
		return ret;

	ret = max17048_write_word(client, MAX17048_VRESET, mdata->vreset);
	if (ret < 0)
		return ret;

	/* Lock model access */
	ret = max17048_write_word(client, MAX17048_UNLOCK, 0x0000);
	if (ret < 0)
		return ret;

	/* Add delay */
	mdelay(200);
	return 0;
}

int max17048_check_battery()
{
	uint16_t version;

	if (!max17048_data)
		return -ENODEV;

	version = max17048_get_version(max17048_data->client);
	if (version != MAX17048_VERSION_NO)
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL_GPL(max17048_check_battery);

#ifdef CONFIG_OF
static struct max17048_platform_data *max17048_parse_dt(struct device *dev)
{
	struct max17048_platform_data *pdata;
	struct max17048_battery_model *model_data;
	struct device_node *np = dev->of_node;
	u32 val, val_array[MAX17048_DATA_SIZE];
	int i, ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	model_data = devm_kzalloc(dev, sizeof(*model_data), GFP_KERNEL);
	if (!model_data)
		return ERR_PTR(-ENOMEM);

	pdata->model_data = model_data;

	ret = of_property_read_u32(np, "bits", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	if ((val == 18) || (val == 19))
		model_data->bits = val;

	ret = of_property_read_u32(np, "alert-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	model_data->alert_threshold = val;
	if (model_data->bits == 19) /* LSB is 0.5%, if 19-bit model. */
		model_data->alert_threshold /= 2;

	ret = of_property_read_u32(np, "one-percent-alerts", &val);
	if (ret < 0)
		return ERR_PTR(ret);

	if (val)
		model_data->one_percent_alerts = 0x40;

	ret = of_property_read_u32(np, "valert-max", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->valert = ((val / 20) & 0xFF) << 8; /* LSB is 20mV. */

	ret = of_property_read_u32(np, "valert-min", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->valert |= (val / 20) & 0xFF; /* LSB is 20mV. */

	ret = of_property_read_u32(np, "vreset-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->vreset = ((val / 40) & 0xFE) << 8; /* LSB is 40mV. */

	ret = of_property_read_u32(np, "vreset-disable", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->vreset |= (val & 0x01) << 8;

	ret = of_property_read_u32(np, "hib-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->hibernate = (val & 0xFF) << 8;

	ret = of_property_read_u32(np, "hib-active-threshold", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->hibernate |= val & 0xFF;

	ret = of_property_read_u32(np, "rcomp", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->rcomp = val;

	ret = of_property_read_u32(np, "rcomp-seg", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->rcomp_seg = val;

	ret = of_property_read_u32(np, "soccheck-a", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->soccheck_A = val;

	ret = of_property_read_u32(np, "soccheck-b", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->soccheck_B = val;

	ret = of_property_read_u32(np, "ocvtest", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	model_data->ocvtest = val;

	ret = of_property_read_u32_array(np, "data-tbl", val_array,
					 MAX17048_DATA_SIZE);
	if (ret < 0)
		return ERR_PTR(ret);

	for (i = 0; i < MAX17048_DATA_SIZE; i++)
		model_data->data_tbl[i] = val_array[i];

	return pdata;
}
#else
static struct max17048_platform_data *max17048_parse_dt(struct device *dev)
{
	return NULL;
}
#endif /* CONFIG_OF */

static int max17048_update_battery_status(struct battery_gauge_dev *bg_dev,
		enum battery_charger_status status)
{
	struct max17048_chip *chip = battery_gauge_get_drvdata(bg_dev);

	if (status == BATTERY_CHARGING) {
		chip->charge_complete = 0;
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
#if defined (MAX17048_FAKE_FULL_HANDLE) && (CONFIG_MACH_S9321 == 1)
		g_soc_special_counter = 0;
#endif
	}
	else if (status == BATTERY_CHARGING_DONE) {
		chip->charge_complete = 1;
		chip->soc = MAX17048_BATTERY_FULL;
		chip->status = POWER_SUPPLY_STATUS_FULL;
		power_supply_changed(&chip->battery);
		return 0;
	} else {
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
		chip->charge_complete = 0;
#if defined (MAX17048_FAKE_FULL_HANDLE) && (CONFIG_MACH_S9321 == 1)
		g_soc_special_counter = 0;
#endif
	}
	chip->lasttime_status = chip->status;
	power_supply_changed(&chip->battery);
	return 0;
}

static ssize_t max17048_reg_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{

	printk("Ivan max17048_reg_show! \n");

}

irqreturn_t max17048_irq_process(int irq, void *dev_id)
{
    struct max17048_chip *chip = (struct inv_mpu_state *)dev_id;
    uint16_t status;
   
    printk("Ivan max17048_irq_process! \n");    
//Ivan added	
    status = max17048_read_word(chip->client, MAX17048_STATUS);
    printk("Ivan max17048_irq_process Status[%x]\n",status >> 8 );
    if (status & 0x0600)	//VL or VH
    {
	if (g_Batt_VL_IRQ_Count > 5)
	{
	    printk("Ivan Battery too low/hight (3.2V), force power off...\n");	
//Ivan End    
	    mdelay(200);
	    max77660_power_forceoff();	    
	}
	g_Batt_VL_IRQ_Count++;
    }
    status = max17048_read_word(chip->client, MAX17048_CONFIG);
    printk("Ivan max17048_irq_process Config[%x]\n",status & 0xFF );
    status &= 0xff00;
    max17048_write_word(chip->client, MAX17048_CONFIG, status);
//Ivan End    
    mdelay(100);

    return IRQ_HANDLED;
}

static irqreturn_t max17048_irq_handler(int irq, void *dev_id)
{
    printk("Ivan max17048_irq_handler! \n");
    return IRQ_WAKE_THREAD;
}

static DEVICE_ATTR(readreg, S_IRUGO | S_IWUSR | S_IWGRP,
		   max17048_reg_show, NULL);

static struct attribute *max17048_attrs[] = {
	&dev_attr_readreg,	
	NULL
};

static struct attribute_group max17048_attr_group = {
	.name = "max17048",
	.attrs = max17048_attrs
};

static int max17048_sysfs_create(struct i2c_client *client)
{
	int err;

	err = sysfs_create_group(&client->dev.kobj, &max17048_attr_group);
	return err;
}

static struct battery_gauge_ops max17048_bg_ops = {
	.update_battery_status = max17048_update_battery_status,
	.get_soc_value = max17048_read_soc_raw_value,
};

static struct battery_gauge_info max17048_bgi = {
	.cell_id = 0,
	.bg_ops = &max17048_bg_ops,
};

static int __devinit max17048_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max17048_chip *chip;
	int ret;
	uint16_t version;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	if (client->dev.of_node) {
		chip->pdata = max17048_parse_dt(&client->dev);
		if (IS_ERR(chip->pdata))
			return PTR_ERR(chip->pdata);
	} else {
		chip->pdata = client->dev.platform_data;
		if (!chip->pdata)
			return -ENODATA;
	}

	max17048_data = chip;
	mutex_init(&chip->mutex);
	chip->shutdown_complete = 0;
	i2c_set_clientdata(client, chip);

	version = max17048_check_battery();
	if (version < 0) {
		ret = -ENODEV;
		goto error;
	}
	dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver 0x%x\n", version);

	ret = max17048_initialize(chip);
	if (ret < 0) {
		dev_err(&client->dev, "Error: Initializing fuel-gauge\n");
		goto error;
	}

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17048_get_property;
	chip->battery.properties	= max17048_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17048_battery_props);
	chip->status			= POWER_SUPPLY_STATUS_DISCHARGING;
	chip->lasttime_status   	= POWER_SUPPLY_STATUS_DISCHARGING;
	chip->charge_complete   	= 0;
	chip->is_recharged		= 0;

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error;
	}
	max17048_bgi.tz_name = chip->pdata->tz_name;
	chip->bg_dev = battery_gauge_register(&client->dev, &max17048_bgi,
				chip);
	if (IS_ERR(chip->bg_dev)) {
		ret = PTR_ERR(chip->bg_dev);
		dev_err(&client->dev, "battery gauge register failed: %d\n",
			ret);
		goto bg_err;
	}
//Ivan added alert handler
	printk("Ivan max17048 alert_irq = %d! \n", chip->pdata->alert_irq);    
	chip->irq = chip->pdata->alert_irq;
	if (chip->pdata->alert_irq != 0)
	{
	    if (request_threaded_irq(chip->pdata->alert_irq, max17048_irq_handler, max17048_irq_process,
			IRQF_TRIGGER_FALLING | IRQF_SHARED, "max17048", chip))
		printk("%s Could not allocate MAX17048 Alert IRQ !\n", __func__);
	    else
		ret = device_init_wakeup(&client->dev, 1);
	    
//		disable_irq_wake(chip->irq);
		disable_irq(chip->irq);
	}
	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17048_work);
	schedule_delayed_work(&chip->work, 0);
	ret = max17048_sysfs_create(client);
	printk("MAX17048_FG:  TIME,  VOLT,  TEMP,   SOC, RCOMP,CONFIG\n");
	g_fg_record_time= 0;
	g_vcell_fifo_init = 0;
	if (ret)
		printk("Ivan max17048_probe Create FS Error!");;
	return 0;
bg_err:
	power_supply_unregister(&chip->battery);
error:
	mutex_destroy(&chip->mutex);

	return ret;
}

static int __devexit max17048_remove(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	battery_gauge_unregister(chip->bg_dev);
	power_supply_unregister(&chip->battery);
	cancel_delayed_work_sync(&chip->work);
	mutex_destroy(&chip->mutex);

	return 0;
}

static void max17048_shutdown(struct i2c_client *client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->work);
	mutex_lock(&chip->mutex);
	chip->shutdown_complete = 1;
	mutex_unlock(&chip->mutex);

}

#ifdef CONFIG_PM_SLEEP
static int max17048_suspend(struct device *dev)
{
	struct max17048_chip *chip = dev_get_drvdata(dev);
	int ret;

	cancel_delayed_work_sync(&chip->work);
//Ivan	
	g_Batt_VL_IRQ_Count = 0;
	
	if (chip->irq != 0)
	{
	    enable_irq(chip->irq);
	    enable_irq_wake(chip->irq);
	}
	printk("Ivan max17048_suspend! \n");
	/*ret = max17048_write_word(chip->client, MAX17048_HIBRT, 0xffff);
	if (ret < 0) {
		dev_err(dev, "failed in entering hibernate mode\n");
		return ret;
	}*/

	return 0;
}

static int max17048_resume(struct device *dev)
{
	struct max17048_chip *chip = dev_get_drvdata(dev);
	int ret;
	struct max17048_battery_model *mdata = chip->pdata->model_data;

	/*ret = max17048_write_word(chip->client, MAX17048_HIBRT, mdata->hibernate);
	if (ret < 0) {
		dev_err(dev, "failed in exiting hibernate mode\n");
		return ret;
	}*/
	if (chip->irq != 0)
	{
	    disable_irq_wake(chip->irq);
	    disable_irq(chip->irq);
	}
//Ivan	schedule_delayed_work(&chip->work, MAX17048_DELAY);
	printk("Ivan max17048_resume! \n");
	schedule_delayed_work(&chip->work, 0);
	return 0;
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(max17048_pm_ops, max17048_suspend, max17048_resume);

#ifdef CONFIG_OF
static const struct of_device_id max17048_dt_match[] = {
	{ .compatible = "maxim,max17048" },
	{ },
};
MODULE_DEVICE_TABLE(of, max17048_dt_match);
#endif

static const struct i2c_device_id max17048_id[] = {
	{ "max17048", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
	.driver	= {
		.name	= "max17048",
		.of_match_table = of_match_ptr(max17048_dt_match),
		.pm = &max17048_pm_ops,
	},
	.probe		= max17048_probe,
	.remove		= __devexit_p(max17048_remove),
	.id_table	= max17048_id,
	.shutdown	= max17048_shutdown,
};

static int __init max17048_init(void)
{
	return i2c_add_driver(&max17048_i2c_driver);
}
subsys_initcall(max17048_init);

static void __exit max17048_exit(void)
{
	i2c_del_driver(&max17048_i2c_driver);
}
module_exit(max17048_exit);

MODULE_AUTHOR("Chandler Zhang <chazhang@nvidia.com>");
MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");
