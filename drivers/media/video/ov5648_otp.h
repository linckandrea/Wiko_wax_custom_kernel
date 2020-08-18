#ifndef _OV5648_OTP_H
#define _OV5648__OTP_H

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
#include <media/ov5648.h>
#include <linux/clk.h>

#define SIZEOF_I2C_TRANSBUF 32
#define OV5648_REG_GLOBAL_GAIN          0x350a
#define OV5648_REG_GLOBAL_COARSE_TIME   0x3500
#define OV5648_REG_GLOBAL_FRAME_LENGTH  0x380e

#define OV5648_RoverG_dec_base  0x14b	/* R/G_Typical eg */
#define OV5648_BoverG_dec_base  0x142	/* B/G_Typical eg */

static struct nvc_regulator_init ov5648_vregs[] = {
	{ OV5648_VREG_AVDD, "avdd_cam_ldo2", },
	{ OV5648_VREG_AVDD_MIPI_SWITCH, "avdd_cam1_ldo3", },
	{ OV5648_VREG_IOVDD, "vdd_cam_1v8", },
};

struct ov5648_info {
	struct i2c_client *i2c_client;
	struct ov5648_platform_data *pdata;
	struct miscdevice miscdev_info;
	struct kobject *kobj_ov5648;
	struct nvc_regulator vreg[ARRAY_SIZE(ov5648_vregs)];
	struct clk			*mclk;
	int mode;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	u32 flag;

	struct nvc_fuseid fuse_id;

	struct edp_client *edpc;
	unsigned edp_state;
};

u32 OV5648_ReadFuseIDFromOTP(struct ov5648_info *info);
int update_truly_otp_wb(struct ov5648_info *info, unsigned short golden_rg, unsigned short golden_bg);
#endif
