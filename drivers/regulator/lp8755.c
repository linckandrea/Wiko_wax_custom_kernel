/*
 * LP8755 High Performance Power Management Unit : System Interface Driver
 * (based on rev. 0.26)
 * Copyright 2012 Texas Instruments
 *
 * Author: Daniel(Geon Si) Jeong <daniel.jeong@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/platform_data/lp8755.h>

#define LP8755_REG_BUCK0	0x00
#define LP8755_REG_BUCK1	0x03
#define LP8755_REG_BUCK2	0x04
#define LP8755_REG_BUCK3	0x01
#define LP8755_REG_BUCK4	0x05
#define LP8755_REG_BUCK5	0x02

#define LP8755_REG_B0_CTRL	0x07
#define LP8755_REG_PH_LEV_B0	0x1F
#define LP8755_REG_PH_LEV_B3	0x20
#define LP8755_REG_LOCK		0xDD
#define LP8755_REG_DEBUG	0xFF

#define LP8755_REG_MAX		(0xFF + 1)

#define LP8755_BUCK_EN_M	BIT(7)
#define LP8755_BUCK_LINEAR_OUT_MAX	0x76
#define LP8755_BUCK_VOUT_M	0x7F

enum bucks {
	BUCK0 = 0,
	BUCK1,
	BUCK2,
	BUCK3,
	BUCK4,
	BUCK5,
};

struct lp8755_mphase {
	int nreg;
	int buck_num[LP8755_BUCK_MAX];
};

struct lp8755_chip {
	struct device *dev;
	struct regmap *regmap;
	struct lp8755_platform_data *pdata;

	int irq;
	unsigned int irqmask;

	int mphase;
	struct regulator_dev *rdev[LP8755_BUCK_MAX];
	int test_mode;
};

/**
 *lp8755_read : read a single register value from lp8755.
 *@pchip : device to read from
 *@reg   : register to read from
 *@val   : pointer to store read value
 */
static int lp8755_read(struct lp8755_chip *pchip, unsigned int reg,
		       unsigned int *val)
{
	return regmap_read(pchip->regmap, reg, val);
}

/**
 *lp8755_write : write a single register value to lp8755.
 *@pchip : device to write to
 *@reg   : register to write to
 *@val   : value to be written
 */
static int lp8755_write(struct lp8755_chip *pchip, unsigned int reg,
			unsigned int val)
{
	return regmap_write(pchip->regmap, reg, val);
}

/**
 *lp8755_update_bits : set the values of bit fields in lp8755 register.
 *@pchip : device to read from
 *@reg   : register to update
 *@mask  : bitmask to be changed
 *@val   : value for bitmask
 */
static int lp8755_update_bits(struct lp8755_chip *pchip, unsigned int reg,
			      unsigned int mask, unsigned int val)
{
	return regmap_update_bits(pchip->regmap, reg, mask, val);
}

static int lp8755_buck_enable_time(struct regulator_dev *rdev)
{
	int ret;
	unsigned int regval;
	enum lp8755_bucks id = rdev_get_id(rdev);
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);

	ret = lp8755_read(pchip, 0x12 + id, &regval);
	if (ret < 0) {
		dev_err(pchip->dev, "i2c acceess error %s\n", __func__);
		return ret;
	}
	return (regval & 0xff) * 100;
}

static int lp8755_buck_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	int ret;
	unsigned int regbval = 0x0;
	enum lp8755_bucks id = rdev_get_id(rdev);
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);

	switch (mode) {
	case REGULATOR_MODE_FAST:
		/* forced pwm mode */
		regbval = (0x01 << id);
		break;
	case REGULATOR_MODE_NORMAL:
		/* enable automatic pwm/pfm mode */
		ret = lp8755_update_bits(pchip, 0x08 + id, 0x20, 0x00);
		if (ret < 0)
			goto err_i2c;
		break;
	case REGULATOR_MODE_IDLE:
		/* enable automatic pwm/pfm/lppfm mode */
		ret = lp8755_update_bits(pchip, 0x08 + id, 0x20, 0x20);
		if (ret < 0)
			goto err_i2c;

		ret = lp8755_update_bits(pchip, 0x10, 0x01, 0x01);
		if (ret < 0)
			goto err_i2c;
		break;
	default:
		dev_err(pchip->dev, "Not supported buck mode %s\n", __func__);
		/* forced pwm mode */
		regbval = (0x01 << id);
	}

	ret = lp8755_update_bits(pchip, 0x06, 0x01 << id, regbval);
	if (ret < 0)
		goto err_i2c;
	return ret;
err_i2c:
	dev_err(pchip->dev, "i2c acceess error %s\n", __func__);
	return ret;
}

static unsigned int lp8755_buck_get_mode(struct regulator_dev *rdev)
{
	int ret;
	unsigned int regval;
	enum lp8755_bucks id = rdev_get_id(rdev);
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);

	ret = lp8755_read(pchip, 0x06, &regval);
	if (ret < 0)
		goto err_i2c;

	/* mode fast means forced pwm mode */
	if (regval & (0x01 << id))
		return REGULATOR_MODE_FAST;

	ret = lp8755_read(pchip, 0x08 + id, &regval);
	if (ret < 0)
		goto err_i2c;

	/* mode idle means automatic pwm/pfm/lppfm mode */
	if (regval & 0x20)
		return REGULATOR_MODE_IDLE;

	/* mode normal means automatic pwm/pfm mode */
	return REGULATOR_MODE_NORMAL;

err_i2c:
	dev_err(pchip->dev, "i2c acceess error %s\n", __func__);
	return 0;
}

static int lp8755_buck_set_ramp(struct regulator_dev *rdev, int ramp)
{
	int ret;
	unsigned int regval = 0x00;
	enum lp8755_bucks id = rdev_get_id(rdev);
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);

	/* uV/us */
	switch (ramp) {
	case 0 ... 230:
		regval = 0x07;
		break;
	case 231 ... 470:
		regval = 0x06;
		break;
	case 471 ... 940:
		regval = 0x05;
		break;
	case 941 ... 1900:
		regval = 0x04;
		break;
	case 1901 ... 3800:
		regval = 0x03;
		break;
	case 3801 ... 7500:
		regval = 0x02;
		break;
	case 7501 ... 15000:
		regval = 0x01;
		break;
	case 15001 ... 30000:
		regval = 0x00;
		break;
	default:
		dev_err(pchip->dev,
			"Not supported ramp value %d %s\n", ramp, __func__);
		return -EINVAL;
	}

	ret = lp8755_update_bits(pchip, 0x07 + id, 0x07, regval);
	if (ret < 0)
		goto err_i2c;
	return ret;
err_i2c:
	dev_err(pchip->dev, "i2c acceess error %s\n", __func__);
	return ret;
}

static int lp8755_list_voltage_buck(struct regulator_dev *rdev,
		unsigned int selector)
{
	if (selector >= rdev->desc->n_voltages)
		return -EINVAL;

	return 500000 + (10000 * selector);
}

static int lp8755_set_voltage_buck_sel(struct regulator_dev *rdev, unsigned sel)
{
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);

	sel <<= ffs(rdev->desc->vsel_mask) - 1;

	return lp8755_update_bits(pchip, rdev->desc->vsel_reg,
			rdev->desc->vsel_mask, sel);
}

static int lp8755_get_voltage_buck_sel(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);

	ret = lp8755_read(pchip, rdev->desc->vsel_reg, &val);
	if (ret != 0)
		return ret;

	val &= rdev->desc->vsel_mask;
	val >>= ffs(rdev->desc->vsel_mask) - 1;

	return val;
}

static int lp8755_enable_buck(struct regulator_dev *rdev)
{
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);

	return lp8755_update_bits(pchip, rdev->desc->enable_reg,
			rdev->desc->enable_mask,
			rdev->desc->enable_mask);
}

static int lp8755_disable_buck(struct regulator_dev *rdev)
{
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);

	return lp8755_update_bits(pchip, rdev->desc->enable_reg,
			rdev->desc->enable_mask, 0);
}

static int lp8755_is_enabled_buck(struct regulator_dev *rdev)
{
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);
	unsigned int reg;

	lp8755_read(pchip, rdev->desc->enable_reg, &reg);
	reg &= rdev->desc->enable_mask;

	return !!(reg);
}

static struct regulator_ops lp8755_buck_ops = {
	.list_voltage = lp8755_list_voltage_buck,
	.set_voltage_sel = lp8755_set_voltage_buck_sel,
	.get_voltage_sel = lp8755_get_voltage_buck_sel,
	.enable = lp8755_enable_buck,
	.disable = lp8755_disable_buck,
	.is_enabled = lp8755_is_enabled_buck,
	.enable_time = lp8755_buck_enable_time,
	.set_mode = lp8755_buck_set_mode,
	.get_mode = lp8755_buck_get_mode,
	.set_ramp_delay = lp8755_buck_set_ramp,
};

#define lp8755_rail(_id) "lp8755_buck"#_id
#define lp8755_buck_init(_id)\
{\
	.constraints = {\
		.name = lp8755_rail(_id),\
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,\
		.min_uV = 500000,\
		.max_uV = 1675000,\
	},\
}

static struct regulator_init_data lp8755_reg_default[LP8755_BUCK_MAX] = {
	[BUCK0] = lp8755_buck_init(0),
	[BUCK1] = lp8755_buck_init(1),
	[BUCK2] = lp8755_buck_init(2),
	[BUCK3] = lp8755_buck_init(3),
	[BUCK4] = lp8755_buck_init(4),
	[BUCK5] = lp8755_buck_init(5),
};

static const struct lp8755_mphase mphase_buck[MPHASE_CONF_MAX] = {
	{3, {BUCK0, BUCK3, BUCK5}
	 },
	{6, {BUCK0, BUCK1, BUCK2, BUCK3, BUCK4, BUCK5}
	 },
	{5, {BUCK0, BUCK2, BUCK3, BUCK4, BUCK5}
	 },
	{4, {BUCK0, BUCK3, BUCK4, BUCK5}
	 },
	{3, {BUCK0, BUCK4, BUCK5}
	 },
	{2, {BUCK0, BUCK5}
	 },
	{1, {BUCK0}
	 },
	{2, {BUCK0, BUCK3}
	 },
	{4, {BUCK0, BUCK2, BUCK3, BUCK5}
	 },
};

static int lp8755_init_data(struct lp8755_chip *pchip)
{
	unsigned int regval;
	int ret, icnt, buck_num;
	struct lp8755_platform_data *pdata = pchip->pdata;

	/* read back  muti-phase configuration */
	ret = lp8755_read(pchip, 0x3D, &regval);
	if (ret < 0)
		goto out_i2c_error;
	pchip->mphase = regval & 0x0F;

	/* set default data based on multi-phase config */
	for (icnt = 0; icnt < mphase_buck[pchip->mphase].nreg; icnt++) {
		buck_num = mphase_buck[pchip->mphase].buck_num[icnt];
		pdata->buck_data[buck_num] = &lp8755_reg_default[buck_num];
	}
	return ret;

out_i2c_error:
	dev_err(pchip->dev, "i2c acceess error %s\n", __func__);
	return ret;
}

static int lp8755_test_mode_show(void *data, u64 *val)
{
	struct regulator_dev *rdev = (struct regulator_dev *)data;
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);
	*val = pchip->test_mode;
	return 0;
}

static int lp8755_test_mode_set(void *data, u64 val)
{
	struct regulator_dev *rdev = (struct regulator_dev *)data;
	struct lp8755_chip *pchip = rdev_get_drvdata(rdev);
	int ret;

	dev_info(pchip->dev, "Doing Test mode writes\n");

	pchip->test_mode++;
	ret = lp8755_write(pchip, LP8755_REG_LOCK, 0x00);
	if (ret < 0) {
		dev_err(pchip->dev, "REG_LOCK write failed: %d\n", ret);
		return ret;
	}

	ret = lp8755_write(pchip, LP8755_REG_LOCK, 0x2C);
	if (ret < 0) {
		dev_err(pchip->dev, "REG_LOCK write failed: %d\n", ret);
		return ret;
	}

	ret = lp8755_write(pchip, LP8755_REG_LOCK, 0x58);
	if (ret < 0) {
		dev_err(pchip->dev, "REG_LOCK write failed: %d\n", ret);
		return ret;
	}

	ret = lp8755_write(pchip, LP8755_REG_DEBUG, 0x01);
	if (ret < 0) {
		dev_err(pchip->dev, "REG_DEUG write failed: %d\n", ret);
		return ret;
	}

	ret = lp8755_write(pchip, LP8755_REG_B0_CTRL, 0x83);
	if (ret < 0) {
		dev_err(pchip->dev, "REG_B0_CTRL write failed: %d\n", ret);
		return ret;
	}

	ret = lp8755_write(pchip, LP8755_REG_DEBUG, 0x00);
	if (ret < 0) {
		dev_err(pchip->dev, "REG_DEBUG write failed: %d\n", ret);
		return ret;
	}

	ret = lp8755_write(pchip, LP8755_REG_PH_LEV_B0, 0x00);
	if (ret < 0) {
		dev_err(pchip->dev, "REG_PH_LEV_B0 write failed: %d\n", ret);
		return ret;
	}

	ret = lp8755_write(pchip, LP8755_REG_PH_LEV_B3, 0x00);
	if (ret < 0) {
		dev_err(pchip->dev, "REG_PH_LEV_B3 write failed: %d\n", ret);
		return ret;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(lp8755_test_mode_ops, lp8755_test_mode_show,
		lp8755_test_mode_set, "%llu\n");

static int lp8755_buck_debug_fs_init(struct regulator_dev *rdev)
{
	if (!debugfs_create_file("test_mode", 0644, rdev->debugfs,
			(void *)rdev, &lp8755_test_mode_ops))
		return -ENODEV;
	return 0;
}

#define lp8755_buck_desc(_id)\
{\
	.name = lp8755_rail(_id),\
	.id   = LP8755_BUCK##_id,\
	.ops  = &lp8755_buck_ops,\
	.n_voltages = LP8755_BUCK_LINEAR_OUT_MAX+1,\
	.type = REGULATOR_VOLTAGE,\
	.owner = THIS_MODULE,\
	.enable_reg = LP8755_REG_BUCK##_id,\
	.enable_mask = LP8755_BUCK_EN_M,\
	.vsel_reg = LP8755_REG_BUCK##_id,\
	.vsel_mask = LP8755_BUCK_VOUT_M,\
}

static struct regulator_desc lp8755_regulators[] = {
	lp8755_buck_desc(0),
	lp8755_buck_desc(1),
	lp8755_buck_desc(2),
	lp8755_buck_desc(3),
	lp8755_buck_desc(4),
	lp8755_buck_desc(5),
};

static int lp8755_regulator_init(struct lp8755_chip *pchip)
{
	int ret, icnt, buck_num;
	struct lp8755_platform_data *pdata = pchip->pdata;

	for (icnt = 0; icnt < mphase_buck[pchip->mphase].nreg; icnt++) {
		buck_num = mphase_buck[pchip->mphase].buck_num[icnt];
		if (!pdata->buck_data[buck_num])
			pdata->buck_data[buck_num] =
				&lp8755_reg_default[buck_num];

		pchip->rdev[buck_num] =
		    regulator_register(&lp8755_regulators[buck_num], pchip->dev,
				    pdata->buck_data[buck_num], pchip, NULL);
		if (IS_ERR(pchip->rdev[buck_num])) {
			ret = PTR_ERR(pchip->rdev[buck_num]);
			pchip->rdev[buck_num] = NULL;
			dev_err(pchip->dev, "regulator init failed: buck %d\n",
				buck_num);
			goto err_buck;
		}

		lp8755_buck_debug_fs_init(pchip->rdev[buck_num]);
		lp8755_buck_set_ramp(pchip->rdev[buck_num],
					pdata->ramp_us[buck_num]);
	}

	return 0;

err_buck:
	for (icnt = 0; icnt < LP8755_BUCK_MAX; icnt++)
		regulator_unregister(pchip->rdev[icnt]);
	return ret;
}

static irqreturn_t lp8755_irq_handler(int irq, void *data)
{
	int ret, icnt;
	unsigned int flag0, flag1;
	struct lp8755_chip *pchip = data;

	/* read flag0 register */
	ret = lp8755_read(pchip, 0x0D, &flag0);
	if (ret < 0)
		goto err_i2c;
	/* clear flag register to pull up int. pin */
	ret = lp8755_write(pchip, 0x0D, 0x00);
	if (ret < 0)
		goto err_i2c;

	/* sent power fault detection event to specific regulator */
	for (icnt = 0; icnt < 6; icnt++)
		if ((flag0 & (0x4 << icnt))
		    && (pchip->irqmask & (0x04 << icnt))
		    && (pchip->rdev[icnt] != NULL))
			regulator_notifier_call_chain(pchip->rdev[icnt],
						      LP8755_EVENT_PWR_FAULT,
						      NULL);

	/* read flag1 register */
	ret = lp8755_read(pchip, 0x0E, &flag1);
	if (ret < 0)
		goto err_i2c;
	/* clear flag register to pull up int. pin */
	ret = lp8755_write(pchip, 0x0E, 0x00);
	if (ret < 0)
		goto err_i2c;

	/* send OCP event to all regualtor devices */
	if ((flag1 & 0x01) && (pchip->irqmask & 0x01))
		for (icnt = 0; icnt < LP8755_BUCK_MAX; icnt++)
			if (pchip->rdev[icnt] != NULL)
				regulator_notifier_call_chain(pchip->rdev[icnt],
							      LP8755_EVENT_OCP,
							      NULL);

	/* send OVP event to all regualtor devices */
	if ((flag1 & 0x02) && (pchip->irqmask & 0x02))
		for (icnt = 0; icnt < LP8755_BUCK_MAX; icnt++)
			if (pchip->rdev[icnt] != NULL)
				regulator_notifier_call_chain(pchip->rdev[icnt],
							      LP8755_EVENT_OVP,
							      NULL);
	return IRQ_HANDLED;

err_i2c:
	dev_err(pchip->dev, "i2c acceess error %s\n", __func__);
	return IRQ_NONE;
}

static int lp8755_int_config(struct lp8755_chip *pchip)
{
	int ret;
	unsigned int regval;

	if (pchip->irq == 0) {
		dev_warn(pchip->dev, "not use interrupt : %s\n", __func__);
		return 0;
	}

	ret = lp8755_read(pchip, 0x0F, &regval);
	if (ret < 0)
		goto err_i2c;
	pchip->irqmask = regval;
	ret = request_threaded_irq(pchip->irq, NULL, lp8755_irq_handler,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "lp8755-irq", pchip);
	if (ret)
		return ret;

	return ret;

err_i2c:
	dev_err(pchip->dev, "i2c acceess error %s\n", __func__);
	return ret;
}

static const struct regmap_config lp8755_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LP8755_REG_MAX,
};

static int lp8755_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret, icnt;
	struct lp8755_chip *pchip;
	struct lp8755_platform_data *pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functionality check fail.\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev,
			     sizeof(struct lp8755_chip), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;

	pchip->dev = &client->dev;
	pchip->regmap = devm_regmap_init_i2c(client, &lp8755_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail to allocate regmap %d\n", ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	if (pdata != NULL) {
		pchip->pdata = pdata;
		pchip->mphase = pdata->mphase;
	} else {
		pchip->pdata = devm_kzalloc(pchip->dev,
					    sizeof(struct lp8755_platform_data),
					    GFP_KERNEL);
		if (!pchip->pdata)
			return -ENOMEM;
		ret = lp8755_init_data(pchip);
		if (ret < 0)
			goto err_chip_init;
	}

	ret = lp8755_regulator_init(pchip);
	if (ret < 0)
		goto err_regulator;

	pchip->irq = client->irq;
	ret = lp8755_int_config(pchip);
	if (ret < 0)
		goto err_irq;

	return ret;

err_irq:
	dev_err(&client->dev, "fail to irq config\n");

	for (icnt = 0; icnt < mphase_buck[pchip->mphase].nreg; icnt++)
		regulator_unregister(pchip->rdev[icnt]);

err_regulator:
	dev_err(&client->dev, "fail to initialize regulators\n");
	/* output disable */
	for (icnt = 0; icnt < 0x06; icnt++)
		lp8755_write(pchip, icnt, 0x00);

err_chip_init:
	dev_err(&client->dev, "fail to initialize chip\n");
	return ret;
}

static int lp8755_remove(struct i2c_client *client)
{
	int icnt;
	struct lp8755_chip *pchip = i2c_get_clientdata(client);

	for (icnt = 0; icnt < mphase_buck[pchip->mphase].nreg; icnt++)
		regulator_unregister(pchip->rdev[icnt]);

	for (icnt = 0; icnt < 0x06; icnt++)
		lp8755_write(pchip, icnt, 0x00);

	if (pchip->irq != 0)
		free_irq(pchip->irq, pchip);

	return 0;
}

static const struct i2c_device_id lp8755_id[] = {
	{LP8755_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lp8755_id);

static struct i2c_driver lp8755_i2c_driver = {
	.driver = {
		   .name = LP8755_NAME,
		   },
	.probe = lp8755_probe,
	.remove = lp8755_remove,
	.id_table = lp8755_id,
};

static int __init lp8755_init(void)
{
	return i2c_add_driver(&lp8755_i2c_driver);
}

subsys_initcall(lp8755_init);

static void __exit lp8755_exit(void)
{
	i2c_del_driver(&lp8755_i2c_driver);
}

module_exit(lp8755_exit);

MODULE_DESCRIPTION("Texas Instruments lp8755 driver");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL v2");
