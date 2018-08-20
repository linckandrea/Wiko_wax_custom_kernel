#ifndef _IMX179_OTP_H
#define _IMX179__OTP_H

#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/edp.h>
#include <media/imx179.h>
#include <linux/clk.h>

#define IMX179_ID			0x0179
#define IMX179_ID_ADDRESS		0x0000
#define IMX179_STREAM_CONTROL_REG	0x0100
#define IMX179_STREAM_ENABLE		0x01
#define IMX179_STREAM_DISABLE		0x00
#define IMX179_SENSOR_TYPE		NVC_IMAGER_TYPE_RAW
#define IMX179_STARTUP_DELAY_MS		50
#define IMX179_RES_CHG_WAIT_TIME_MS	100
#define IMX179_SIZEOF_I2C_BUF		16
#define IMX179_TABLE_WAIT_MS		0
#define IMX179_TABLE_END		1
#define IMX179_NUM_MODES		ARRAY_SIZE(imx179_mode_table)
#define IMX179_MODE_UNKNOWN		(IMX179_NUM_MODES + 1)
#define IMX179_LENS_MAX_APERTURE	0 /* / _INT2FLOAT_DIVISOR */
#define IMX179_LENS_FNUMBER		0 /* / _INT2FLOAT_DIVISOR */
#define IMX179_LENS_FOCAL_LENGTH	3700 /* / _INT2FLOAT_DIVISOR */
#define IMX179_LENS_VIEW_ANGLE_H	75600 /* / _INT2FLOAT_DIVISOR */
#define IMX179_LENS_VIEW_ANGLE_V	75600 /* / _INT2FLOAT_DIVISOR */
#define IMX179_WAIT_MS 3
#define IMX179_I2C_TABLE_MAX_ENTRIES	400

struct imx179_info {
	atomic_t in_use;
	struct i2c_client *i2c_client;
	struct imx179_platform_data *pdata;
	struct nvc_imager_cap *cap;
	struct miscdevice miscdev;
	struct list_head list;
	struct nvc_gpio gpio[3];
	struct nvc_regulator vreg[3];
	struct edp_client *edpc;
	unsigned edp_state;
	int pwr_api;
	int pwr_dev;
	u8 s_mode;
	struct imx179_info *s_info;
	u32 mode_index;
	bool mode_valid;
	bool mode_enable;
	bool reset_flag;
	struct clk *mclk;
	unsigned test_pattern;
	struct nvc_imager_static_nvc sdata;
	u8 i2c_buf[IMX179_SIZEOF_I2C_BUF];
	u8 bin_en;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
	u16 i2c_reg;
#endif
	struct nvc_fuseid fuse_id;
};

int IMX179_ReadFuseIDFromOTP(struct imx179_info *info);
int ReadOTPIMX179(struct imx179_info *info);
#endif
