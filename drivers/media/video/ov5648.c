/*
 * ov5648.c - ov5648 sensor driver
 *
 * Copyright (c) 2011-2013, NVIDIA CORPORATION. All Rights Reserved.
 *
 * Contributors:
 *    Jerry Chang <jerchang@nvidia.com>
 *
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/edp.h>
#include <media/imx091.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/ov5648.h>
#include "ov5648_otp.h"
#include <../../../arch/arm/mach-tegra/board-ceres.h>
#if 0
#define SIZEOF_I2C_TRANSBUF 32
#define OV5648_REG_GLOBAL_GAIN          0x350a
#define OV5648_REG_GLOBAL_COARSE_TIME   0x3500
#define OV5648_REG_GLOBAL_FRAME_LENGTH  0x380e
#endif

struct ov5648_reg {
	u16 addr;
	u16 val;
};

static LIST_HEAD(ov5648_info_list);

#define OV5648_TABLE_WAIT_MS 0
#define OV5648_WAIT_MS 5
#define OV5648_TABLE_END 1
#define OV5648_MAX_RETRIES 3

static struct ov5648_reg mode_2592x1944[] = {
	/*reset*/
	{0x0100, 0x00},
	{0x0103, 0x01},
	{OV5648_TABLE_WAIT_MS, OV5648_WAIT_MS},

	/*system control*/
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3011, 0x02},
	{0x3017, 0x05},
	{0x3018, 0x4c},
	{0x301c, 0xd2},
	{0x3022, 0x00},
	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x69},
	{0x3037, 0x03},
	{0x3038, 0x00},
	{0x3039, 0x00},
	{0x303a, 0x00},
	{0x303b, 0x19},
	{0x303c, 0x11},
	{0x303d, 0x30},
	{0x3105, 0x11},
	{0x3106, 0x05},

	{0x3304, 0x28},
	{0x3305, 0x41},
	{0x3306, 0x30},
	{0x3308, 0x00},
	{0x3309, 0xc8},
	{0x330a, 0x01},
	{0x330b, 0x90},
	{0x330c, 0x02},
	{0x330d, 0x58},
	{0x330e, 0x03},
	{0x330f, 0x20},
	{0x3300, 0x00},

	/*AEC/AGC*/
	{0x3500, 0x00},
	{0x3501, 0x7b},
	{0x3502, 0x00},
	{0x3503, 0x07},
	{0x350a, 0x00},
	{0x350b, 0x40},

	/*(SCCB) message*/
	{0x3601, 0x33},
	{0x3602, 0x00},
	{0x3611, 0x0e},
	{0x3612, 0x2b},
	{0x3614, 0x50},
	{0x3620, 0x33},
	{0x3622, 0x00},
	{0x3630, 0xad},
	{0x3631, 0x00},
	{0x3632, 0x94},
	{0x3633, 0x17},
	{0x3634, 0x14},

	{0x3704, 0xc0},
	{0x3705, 0x2a},
	{0x3708, 0x63},
	{0x3709, 0x12},
	{0x370b, 0x23},
	{0x370c, 0xc0},
	{0x370d, 0x00},
	{0x370e, 0x00},
	{0x371c, 0x07},
	{0x3739, 0xd2},
	{0x373c, 0x00},

	/*timing control*/
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xa3},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x380c, 0x0b},
	{0x380d, 0x00},
	{0x380e, 0x07},
	{0x380f, 0xc0},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3817, 0x00},
	{0x3820, 0x46},
	{0x3821, 0x00},
	{0x3826, 0x03},
	{0x3829, 0x00},
	{0x382b, 0x0b},
	{0x3830, 0x00},

	{0x3836, 0x00},
	{0x3837, 0x00},
	{0x3838, 0x00},
	{0x3839, 0x04},
	{0x383a, 0x00},
	{0x383b, 0x01},

	/*strobe/frame exposure*/
	{0x3b00, 0x00},
	{0x3b02, 0x08},
	{0x3b03, 0x00},
	{0x3b04, 0x04},
	{0x3b05, 0x00},
	{0x3b06, 0x04},
	{0x3b07, 0x08},
	{0x3b08, 0x00},
	{0x3b09, 0x02},
	{0x3b0a, 0x04},
	{0x3b0b, 0x00},
	{0x3b0c, 0x3d},

	{0x3f01, 0x0d},
	{0x3f0f, 0xf5},

	/*BLC control*/
	{0x4000, 0x89},
	{0x4001, 0x02},
	{0x4002, 0x45},
	{0x4004, 0x04},
	{0x4005, 0x18},
	{0x4006, 0x08},
	{0x4007, 0x10},
	{0x4008, 0x00},

	{0x4300, 0xf8},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4307, 0xff},

	{0x4520, 0x00},
	{0x4521, 0x00},
	{0x4511, 0x22},

	/*MIPI control*/
	{0x4801, 0x0f},
	{0x4814, 0x2a},
	{0x481f, 0x3c},
	{0x4823, 0x3c},
	{0x4826, 0x00},
	{0x481b, 0x3c},
	{0x4827, 0x32},
	{0x4837, 0x18},

	{0x4b00, 0x06},
	{0x4b01, 0x0a},
	{0x4b04, 0x10},

	/*ISP control*/
	{0x5000, 0xff},
	{0x5001, 0x00},
	{0x5002, 0x41},
	{0x5003, 0x0a},
	{0x5004, 0x00},
	{0x5043, 0x00},
	{0x5013, 0x00},
	{0x501f, 0x03},
	{0x503d, 0x00},

	{0x5a00, 0x08},
	{0x5b00, 0x01},
	{0x5b01, 0x40},
	{0x5b02, 0x00},
	{0x5b03, 0xf0},

	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x5780, 0xfc},
	{0x5781, 0x1f},
	{0x5782, 0x03},
	{0x5786, 0x20},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x5789, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578d, 0x0c},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},

	{0x0100, 0x01},
	{OV5648_TABLE_END, 0x0000}
};

static struct ov5648_reg mode_1920x1080[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{OV5648_TABLE_WAIT_MS, OV5648_WAIT_MS},

	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3011, 0x02},
	{0x3017, 0x05},
	{0x3018, 0x4c},
	{0x301c, 0xd2},
	{0x3022, 0x00},
	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x69},
	{0x3037, 0x03},
	{0x3038, 0x00},
	{0x3039, 0x00},
	{0x303a, 0x00},
	{0x303b, 0x19},
	{0x303c, 0x11},
	{0x303d, 0x30},

	{0x3105, 0x11},
	{0x3106, 0x05},

	{0x3304, 0x28},
	{0x3305, 0x41},
	{0x3306, 0x30},
	{0x3308, 0x00},
	{0x3309, 0xc8},
	{0x330a, 0x01},
	{0x330b, 0x90},
	{0x330c, 0x02},
	{0x330d, 0x58},
	{0x330e, 0x03},
	{0x330f, 0x20},
	{0x3300, 0x00},

	{0x3500, 0x00},
	{0x3501, 0x45},
	{0x3502, 0x00},
	{0x3503, 0x07},
	{0x350a, 0x00},
	{0x350b, 0x40},

	{0x3601, 0x33},
	{0x3602, 0x00},
	{0x3611, 0x0e},
	{0x3612, 0x2b},
	{0x3614, 0x50},
	{0x3620, 0x33},
	{0x3622, 0x00},
	{0x3630, 0xad},
	{0x3631, 0x00},
	{0x3632, 0x94},
	{0x3633, 0x17},
	{0x3634, 0x14},

	{0x3704, 0xc0},
	{0x3705, 0x2a},
	{0x3708, 0x63},
	{0x3709, 0x12},
	{0x370b, 0x23},
	{0x370c, 0xc0},
	{0x370d, 0x00},
	{0x370e, 0x00},
	{0x371c, 0x07},

	{0x3739, 0xd2},
	{0x373c, 0x00},

	{0x3800, 0x01},
	{0x3801, 0x50},
	{0x3802, 0x01},
	{0x3803, 0xb2},
	{0x3804, 0x08},
	{0x3805, 0xef},
	{0x3806, 0x05},
	{0x3807, 0xf1},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x380c, 0x09},
	{0x380d, 0xc4},
	{0x380e, 0x04},
	{0x380f, 0x60},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3817, 0x00},
	{0x3820, 0x46},
	{0x3821, 0x00},
	{0x3826, 0x03},
	{0x3829, 0x00},
	{0x382b, 0x0b},
	{0x3830, 0x00},
	{0x3836, 0x00},
	{0x3837, 0x00},
	{0x3838, 0x00},
	{0x3839, 0x04},
	{0x383a, 0x00},
	{0x383b, 0x01},

	{0x3b00, 0x00},
	{0x3b02, 0x08},
	{0x3b03, 0x00},
	{0x3b04, 0x04},
	{0x3b05, 0x00},
	{0x3b06, 0x04},
	{0x3b07, 0x08},
	{0x3b08, 0x00},
	{0x3b09, 0x02},
	{0x3b0a, 0x04},
	{0x3b0b, 0x00},
	{0x3b0c, 0x3d},

	{0x3f01, 0x0d},
	{0x3f0f, 0xf5},

	{0x4000, 0x89},
	{0x4001, 0x02},
	{0x4002, 0x45},
	{0x4004, 0x04},
	{0x4005, 0x18},
	{0x4006, 0x08},
	{0x4007, 0x10},
	{0x4008, 0x00},

	{0x4300, 0xf8},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4307, 0xff},

	{0x4520, 0x00},
	{0x4521, 0x00},
	{0x4511, 0x22},

	{0x4801, 0x0f},
	{0x4814, 0x2a},
	{0x481f, 0x3c},

	{0x4823, 0x3c},
	{0x4826, 0x00},
	{0x481b, 0x3c},
	{0x4827, 0x32},
	{0x4837, 0x18},

	{0x4b00, 0x06},
	{0x4b01, 0x0a},
	{0x4b04, 0x10},

	{0x5000, 0xff},
	{0x5001, 0x00},
	{0x5002, 0x41},
	{0x5003, 0x0a},
	{0x5004, 0x00},
	{0x5043, 0x00},

	{0x5013, 0x00},
	{0x501f, 0x03},

	{0x503d, 0x00},
	{0x5a00, 0x08},
	{0x5b00, 0x01},
	{0x5b01, 0x40},
	{0x5b02, 0x00},
	{0x5b03, 0xf0},

	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x5780, 0xfc},
	{0x5781, 0x1f},
	{0x5782, 0x03},
	{0x5786, 0x20},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x5789, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578d, 0x0c},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},

	{0x0100, 0x01},
	{OV5648_TABLE_END, 0x0000}
};
#if 0
static struct ov5648_reg mode_1600x1200[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{OV5648_TABLE_WAIT_MS, OV5648_WAIT_MS},

	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3011, 0x02},
	{0x3017, 0x05},
	{0x3018, 0x4c},
	{0x301c, 0xd2},
	{0x3022, 0x00},

	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x69},
	{0x3037, 0x03},
	{0x3038, 0x00},
	{0x3039, 0x00},
	{0x303a, 0x00},
	{0x303b, 0x19},
	{0x303c, 0x11},
	{0x303d, 0x30},

	{0x3105, 0x11},
	{0x3106, 0x05},

	{0x3304, 0x28},
	{0x3305, 0x41},
	{0x3306, 0x30},
	{0x3308, 0x00},
	{0x3309, 0xc8},
	{0x330a, 0x01},
	{0x330b, 0x90},
	{0x330c, 0x02},
	{0x330d, 0x58},
	{0x330e, 0x03},
	{0x330f, 0x20},
	{0x3300, 0x00},

	{0x3500, 0x00},
	{0x3501, 0x4d},
	{0x3502, 0x00},
	{0x3503, 0x07},
	{0x350a, 0x00},
	{0x350b, 0x40},

	{0x3601, 0x33},
	{0x3602, 0x00},
	{0x3611, 0x0e},
	{0x3612, 0x2b},
	{0x3614, 0x50},
	{0x3620, 0x33},
	{0x3622, 0x00},
	{0x3630, 0xad},
	{0x3631, 0x00},
	{0x3632, 0x94},
	{0x3633, 0x17},
	{0x3634, 0x14},

	{0x3704, 0xc0},
	{0x3705, 0x2a},
	{0x3708, 0x63},
	{0x3709, 0x12},
	{0x370b, 0x23},
	{0x370c, 0xc0},
	{0x370d, 0x00},
	{0x370e, 0x00},
	{0x371c, 0x07},
	{0x3739, 0xd2},
	{0x373c, 0x00},

	{0x3800, 0x01},
	{0x3801, 0xf0},
	{0x3802, 0x01},
	{0x3803, 0x76},
	{0x3804, 0x08},
	{0x3805, 0x4f},
	{0x3806, 0x06},
	{0x3807, 0x2d},
	{0x3808, 0x06},
	{0x3809, 0x40},
	{0x380a, 0x04},
	{0x380b, 0xb0},
	{0x380c, 0x08},
	{0x380d, 0xd2},
	{0x380e, 0x04},
	{0x380f, 0xd8},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3817, 0x00},
	{0x3820, 0x46},
	{0x3821, 0x00},
	{0x3826, 0x03},
	{0x3829, 0x00},
	{0x382b, 0x0b},
	{0x3830, 0x00},
	{0x3836, 0x00},
	{0x3837, 0x00},
	{0x3838, 0x00},
	{0x3839, 0x04},
	{0x383a, 0x00},
	{0x383b, 0x01},

	{0x3b00, 0x00},
	{0x3b02, 0x08},
	{0x3b03, 0x00},
	{0x3b04, 0x04},
	{0x3b05, 0x00},
	{0x3b06, 0x04},
	{0x3b07, 0x08},
	{0x3b08, 0x00},
	{0x3b09, 0x02},
	{0x3b0a, 0x04},
	{0x3b0b, 0x00},
	{0x3b0c, 0x3d},

	{0x3f01, 0x0d},
	{0x3f0f, 0xf5},

	{0x4000, 0x89},
	{0x4001, 0x02},
	{0x4002, 0x45},
	{0x4004, 0x04},
	{0x4005, 0x18},
	{0x4006, 0x08},
	{0x4007, 0x10},
	{0x4008, 0x00},

	{0x4300, 0xf8},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4307, 0xff},

	{0x4520, 0x00},
	{0x4521, 0x00},
	{0x4511, 0x22},

	{0x4801, 0x0f},
	{0x4814, 0x2a},
	{0x481f, 0x3c},
	{0x4823, 0x3c},
	{0x4826, 0x00},
	{0x481b, 0x3c},
	{0x4827, 0x32},
	{0x4837, 0x18},

	{0x4b00, 0x06},
	{0x4b01, 0x0a},
	{0x4b04, 0x10},

	{0x5000, 0xff},
	{0x5001, 0x00},
	{0x5002, 0x41},
	{0x5003, 0x0a},
	{0x5004, 0x00},

	{0x5043, 0x00},
	{0x5013, 0x00},
	{0x501f, 0x03},
	{0x503d, 0x00},
	{0x5a00, 0x08},
	{0x5b00, 0x01},
	{0x5b01, 0x40},
	{0x5b02, 0x00},
	{0x5b03, 0xf0},

	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x5780, 0xfc},
	{0x5781, 0x1f},
	{0x5782, 0x03},
	{0x5786, 0x20},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x5789, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578d, 0x0c},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},

	{0x0100, 0x01},

	{OV5648_TABLE_END, 0x0000}
};
#endif
static struct ov5648_reg mode_1296x972[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{OV5648_TABLE_WAIT_MS, OV5648_WAIT_MS},

	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3011, 0x02},
	{0x3017, 0x05},
	{0x3018, 0x4c},
	{0x301c, 0xd2},
	{0x3022, 0x00},

	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x69},
	{0x3037, 0x03},
	{0x3038, 0x00},
	{0x3039, 0x00},
	{0x303a, 0x00},
	{0x303b, 0x19},
	{0x303c, 0x11},
	{0x303d, 0x30},

	{0x3105, 0x11},
	{0x3106, 0x05},

	{0x3304, 0x28},
	{0x3305, 0x41},
	{0x3306, 0x30},
	{0x3308, 0x00},
	{0x3309, 0xc8},
	{0x330a, 0x01},
	{0x330b, 0x90},
	{0x330c, 0x02},
	{0x330d, 0x58},
	{0x330e, 0x03},
	{0x330f, 0x20},
	{0x3300, 0x00},

	{0x3500, 0x00},
	{0x3501, 0x7b},
	{0x3502, 0x00},
	{0x3503, 0x07},
	{0x350a, 0x00},
	{0x350b, 0x40},

	{0x3601, 0x33},
	{0x3602, 0x00},
	{0x3611, 0x0e},
	{0x3612, 0x2b},
	{0x3614, 0x50},
	{0x3620, 0x33},
	{0x3622, 0x00},
	{0x3630, 0xad},
	{0x3631, 0x00},
	{0x3632, 0x94},
	{0x3633, 0x17},
	{0x3634, 0x14},

	{0x3704, 0xc0},
	{0x3705, 0x2a},
	{0x3708, 0x66},
	{0x3709, 0x52},
	{0x370b, 0x23},
	{0x370c, 0xcf},
	{0x370d, 0x00},
	{0x370e, 0x00},
	{0x371c, 0x07},
	{0x3739, 0xd2},
	{0x373c, 0x00},

	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xa3},
	{0x3808, 0x05},
	{0x3809, 0x10},
	{0x380a, 0x03},
	{0x380b, 0xcc},
	{0x380c, 0x0b},
	{0x380d, 0x00},
	{0x380e, 0x03},
	{0x380f, 0xe0},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3817, 0x00},
	{0x3820, 0x0e},
	{0x3821, 0x01},
	{0x3826, 0x03},
	{0x3829, 0x00},
	{0x382b, 0x0b},
	{0x3830, 0x00},
	{0x3836, 0x00},
	{0x3837, 0x00},
	{0x3838, 0x00},
	{0x3839, 0x04},
	{0x383a, 0x00},
	{0x383b, 0x01},

	{0x3b00, 0x00},
	{0x3b02, 0x08},
	{0x3b03, 0x00},
	{0x3b04, 0x04},
	{0x3b05, 0x00},
	{0x3b06, 0x04},
	{0x3b07, 0x08},
	{0x3b08, 0x00},
	{0x3b09, 0x02},
	{0x3b0a, 0x04},
	{0x3b0b, 0x00},
	{0x3b0c, 0x3d},

	{0x3f01, 0x0d},
	{0x3f0f, 0xf5},

	{0x4000, 0x89},
	{0x4001, 0x02},
	{0x4002, 0x45},
	{0x4004, 0x02},
	{0x4005, 0x18},
	{0x4006, 0x08},
	{0x4007, 0x10},
	{0x4008, 0x00},

	{0x4300, 0xf8},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4307, 0xff},

	{0x4520, 0x00},
	{0x4521, 0x00},
	{0x4511, 0x22},

	{0x4801, 0x0f},
	{0x4814, 0x2a},
	{0x481f, 0x3c},
	{0x4823, 0x3c},
	{0x4826, 0x00},
	{0x481b, 0x3c},
	{0x4827, 0x32},
	{0x4837, 0x18},

	{0x4b00, 0x06},
	{0x4b01, 0x0a},
	{0x4b04, 0x10},

	{0x5000, 0xff},
	{0x5001, 0x00},
	{0x5002, 0x41},
	{0x5003, 0x0a},
	{0x5004, 0x00},

	{0x5043, 0x00},
	{0x5013, 0x00},
	{0x501f, 0x03},
	{0x503d, 0x00},
	{0x5a00, 0x08},
	{0x5b00, 0x01},
	{0x5b01, 0x40},
	{0x5b02, 0x00},
	{0x5b03, 0xf0},

	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x5780, 0xfc},
	{0x5781, 0x1f},
	{0x5782, 0x03},
	{0x5786, 0x20},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x5789, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578d, 0x0c},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},

	{0x0100, 0x01},

	{OV5648_TABLE_END, 0x0000}
};

static struct ov5648_reg mode_1280x720[] = {
	/*reset*/
	{0x0100, 0x00},
	{0x0103, 0x01},
	{OV5648_TABLE_WAIT_MS, OV5648_WAIT_MS},

	/*system control*/
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3011, 0x02},
	{0x3017, 0x05},
	{0x3018, 0x4c},
	{0x301c, 0xd2},
	{0x3022, 0x00},
	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x69},
	{0x3037, 0x03},
	{0x3038, 0x00},
	{0x3039, 0x00},
	{0x303a, 0x00},
	{0x303b, 0x19},
	{0x303c, 0x11},
	{0x303d, 0x30},
	{0x3105, 0x11},
	{0x3106, 0x05},

	{0x3304, 0x28},
	{0x3305, 0x41},
	{0x3306, 0x30},
	{0x3308, 0x00},
	{0x3309, 0xc8},
	{0x330a, 0x01},
	{0x330b, 0x90},
	{0x330c, 0x02},
	{0x330d, 0x58},
	{0x330e, 0x03},
	{0x330f, 0x20},
	{0x3300, 0x00},

	/*AEC/AGC*/
	{0x3500, 0x00},
	{0x3501, 0x2d},
	{0x3502, 0xc0},
	{0x3503, 0x07},
	{0x350a, 0x00},
	{0x350b, 0x40},

	/*(SCCB) message*/
	{0x3601, 0x33},
	{0x3602, 0x00},
	{0x3611, 0x0e},
	{0x3612, 0x2b},
	{0x3614, 0x50},
	{0x3620, 0x33},
	{0x3622, 0x00},
	{0x3630, 0xad},
	{0x3631, 0x00},
	{0x3632, 0x94},
	{0x3633, 0x17},
	{0x3634, 0x14},

	{0x3704, 0xc0},
	{0x3705, 0x2a},
	{0x3708, 0x66},
	{0x3709, 0x52},
	{0x370b, 0x23},
	{0x370c, 0xcf},
	{0x370d, 0x00},
	{0x370e, 0x00},
	{0x371c, 0x07},
	{0x3739, 0xd2},
	{0x373c, 0x00},

	/*timing control*/
	{0x3800, 0x00},
	{0x3801, 0x10},
	{0x3802, 0x00},
	{0x3803, 0xfe},
	{0x3804, 0x0a},
	{0x3805, 0x2f},
	{0x3806, 0x06},
	{0x3807, 0xa5},
	{0x3808, 0x05},
	{0x3809, 0x00},
	{0x380a, 0x02},
	{0x380b, 0xd0},
	{0x380c, 0x0e},
	{0x380d, 0xc4},
	{0x380e, 0x02},
	{0x380f, 0xe6},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3817, 0x00},
	{0x3820, 0x0e},
	{0x3821, 0x01},
	{0x3826, 0x03},
	{0x3829, 0x00},
	{0x382b, 0x0b},
	{0x3830, 0x00},

	{0x3836, 0x00},
	{0x3837, 0x00},
	{0x3838, 0x00},
	{0x3839, 0x04},
	{0x383a, 0x00},
	{0x383b, 0x01},

	/*strobe/frame exposure*/
	{0x3b00, 0x00},
	{0x3b02, 0x08},
	{0x3b03, 0x00},
	{0x3b04, 0x04},
	{0x3b05, 0x00},
	{0x3b06, 0x04},
	{0x3b07, 0x08},
	{0x3b08, 0x00},
	{0x3b09, 0x02},
	{0x3b0a, 0x04},
	{0x3b0b, 0x80},
	{0x3b0c, 0x3d},

	{0x3f01, 0x0d},
	{0x3f0f, 0xf5},

	/*BLC control*/
	{0x4000, 0x89},
	{0x4001, 0x02},
	{0x4002, 0x45},
	{0x4004, 0x02},
	{0x4005, 0x18},
	{0x4006, 0x08},
	{0x4007, 0x10},
	{0x4008, 0x00},

	{0x4300, 0xf8},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4307, 0xff},

	{0x4520, 0x00},
	{0x4521, 0x00},
	{0x4511, 0x22},

	/*MIPI control*/
	{0x4801, 0x0f},
	{0x4814, 0x2a},
	{0x481f, 0x3c},
	{0x4823, 0x3c},
	{0x4826, 0x00},
	{0x481b, 0x3c},
	{0x4827, 0x32},
	{0x4837, 0x17},

	{0x4b00, 0x06},
	{0x4b01, 0x0a},
	{0x4b04, 0x10},

	/*ISP control*/
	{0x5000, 0xff},
	{0x5001, 0x00},
	{0x5002, 0x41},
	{0x5003, 0x0a},
	{0x5004, 0x00},
	{0x5043, 0x00},
	{0x5013, 0x00},
	{0x501f, 0x03},
	{0x503d, 0x00},

	{0x5a00, 0x08},
	{0x5b00, 0x01},
	{0x5b01, 0x40},
	{0x5b02, 0x00},
	{0x5b03, 0xf0},

	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x5780, 0xfc},
	{0x5781, 0x1f},
	{0x5782, 0x03},
	{0x5786, 0x20},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x5789, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578d, 0x0c},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},

	{0x0100, 0x01},
	{OV5648_TABLE_END, 0x0000}
};


enum {
	OV5648_MODE_2592x1944,
	OV5648_MODE_1920x1080,
	//OV5648_MODE_1600x1200,
	OV5648_MODE_1296x972,
	OV5648_MODE_1280x720,
};

static struct ov5648_reg *mode_table[] = {
	[OV5648_MODE_2592x1944] = mode_2592x1944,
	[OV5648_MODE_1920x1080] = mode_1920x1080,
	//[OV5648_MODE_1600x1200] = mode_1600x1200,
	[OV5648_MODE_1296x972] = mode_1296x972,
	[OV5648_MODE_1280x720] = mode_1280x720,
};

static void ov5648_edp_lowest(struct ov5648_info *info)
{
	if (!info->edpc)
		return;

	info->edp_state = info->edpc->num_states - 1;
	dev_dbg(&info->i2c_client->dev, "%s %d\n", __func__, info->edp_state);
	if (edp_update_client_request(info->edpc, info->edp_state, NULL)) {
		dev_err(&info->i2c_client->dev, "THIS IS NOT LIKELY HAPPEN!\n");
		dev_err(&info->i2c_client->dev,
			"UNABLE TO SET LOWEST EDP STATE!\n");
	}
}

static void ov5648_edp_register(struct ov5648_info *info)
{
	struct edp_manager *edp_manager;
	struct edp_client *edpc = &info->pdata->edpc_config;
	int ret;

	info->edpc = NULL;
	if (!edpc->num_states) {
		dev_warn(&info->i2c_client->dev,
			"%s: NO edp states defined.\n", __func__);
		return;
	}

	strncpy(edpc->name, "ov5648", EDP_NAME_LEN - 1);
	edpc->name[EDP_NAME_LEN - 1] = 0;
	edpc->private_data = info;

	dev_dbg(&info->i2c_client->dev, "%s: %s, e0 = %d, p %d\n",
		__func__, edpc->name, edpc->e0_index, edpc->priority);
	for (ret = 0; ret < edpc->num_states; ret++)
		dev_dbg(&info->i2c_client->dev, "e%d = %d mA",
			ret - edpc->e0_index, edpc->states[ret]);

	edp_manager = edp_get_manager("battery");
	if (!edp_manager) {
		dev_err(&info->i2c_client->dev,
			"unable to get edp manager: battery\n");
		return;
	}

	ret = edp_register_client(edp_manager, edpc);
	if (ret) {
		dev_err(&info->i2c_client->dev,
			"unable to register edp client\n");
		return;
	}

	info->edpc = edpc;
	/* set to lowest state at init */
	ov5648_edp_lowest(info);
}

static int ov5648_edp_req(struct ov5648_info *info, unsigned new_state)
{
	unsigned approved;
	int ret = 0;

	if (!info->edpc)
		return 0;

	dev_dbg(&info->i2c_client->dev, "%s %d\n", __func__, new_state);
	ret = edp_update_client_request(info->edpc, new_state, &approved);
	if (ret) {
		dev_err(&info->i2c_client->dev, "E state transition failed\n");
		return ret;
	}

	if (approved > new_state) {
		dev_err(&info->i2c_client->dev, "EDP no enough current\n");
		return -ENODEV;
	}

	info->edp_state = approved;
	return 0;
}

static inline void ov5648_get_frame_length_regs(
					struct ov5648_reg *regs,
					u32 frame_length)
{
	regs->addr = 0x380e;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x380f;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void ov5648_get_coarse_time_regs(
					struct ov5648_reg *regs,
					u32 coarse_time)
{
	regs->addr = 0x3500;
	//regs->val = (coarse_time >> 16) & 0x0f;
	regs->val = (coarse_time >> 12) & 0x0f;
	(regs + 1)->addr = 0x3501;
	//(regs + 1)->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = 0x3502;
	//(regs + 2)->val = coarse_time & 0xf0;
	(regs + 2)->val = (coarse_time <<4)& 0xf0;
}

static inline void ov5648_get_gain_reg(
					struct ov5648_reg *regs,
					u16 gain)
{
	regs->addr = 0x350a;
	regs->val = (gain >> 8) & 0x03;
	(regs + 1)->addr = 0x350b;
	(regs + 1)->val = gain & 0xff;
}

static int ov5648_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2];

	return 0;
}

static int ov5648_read_reg_bulk(struct i2c_client *client,
	u16 addr, u8 *buf, u8 num)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = num;
	msg[1].buf = buf;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	return 0;
}

int OV5648MIPI_read_cmos_sensor(struct ov5648_info *info, u16 reg)
{
	u8 val;
	if(ov5648_read_reg(info->i2c_client, reg, &val))
		return -EIO;
	else
		return val;
}

static int ov5648_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("ov5648: i2c transfer failed, retrying %x %x\n",
			   addr, val);

		msleep(20);
	} while (retry <= OV5648_MAX_RETRIES);

	return err;
}

static int ov5648_write_bulk_reg(
			struct i2c_client *client,
			u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	pr_err("ov5648: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

int OV5648MIPI_write_cmos_sensor(struct ov5648_info *info, u16 reg, u8 val)
{
	return ov5648_write_reg(info->i2c_client, reg, val);
}

static int ov5648_write_table(struct ov5648_info *info,
			const struct ov5648_reg table[],
			const struct ov5648_reg override_list[],
			int num_override_regs)
{
	int err;
	const struct ov5648_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	unsigned int i;
	u16 val;

	for (next = table; next->addr != OV5648_TABLE_END; next++) {
		if (next->addr == OV5648_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;
		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list*/
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		if (!buf_filled) {
			b_ptr = info->i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != OV5648_TABLE_END &&
			n_next->addr != OV5648_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = ov5648_write_bulk_reg(info->i2c_client,
				info->i2c_trans_buf, buf_filled);
		if (err)
			return err;
		buf_filled = 0;
	}
	return 0;
}

static int ov5648_set_mode(
			struct ov5648_info *info,
			struct ov5648_mode *mode)
{
	int sensor_mode;
	int err;
	struct ov5648_reg reg_list[7];

	pr_info("%s: xres: %u yres: %u framelength: %u coarsetime: %u gain: %u\n",
			__func__, mode->xres, mode->yres, mode->frame_length,
			mode->coarse_time, mode->gain);

	if (mode->xres == 2592 && mode->yres == 1944)
		sensor_mode = OV5648_MODE_2592x1944;
	else if (mode->xres == 1920 && mode->yres == 1080)
		sensor_mode = OV5648_MODE_1920x1080;
	//else if (mode->xres == 1600 && mode->yres == 1200)
	//	sensor_mode = OV5648_MODE_1600x1200;
	else if (mode->xres == 1296 && mode->yres == 972)
		sensor_mode = OV5648_MODE_1296x972;
	else if (mode->xres == 1280 && mode->yres == 720)
		sensor_mode = OV5648_MODE_1280x720;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
				__func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.*/
	ov5648_get_frame_length_regs(reg_list, mode->frame_length);
	ov5648_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	ov5648_get_gain_reg(reg_list + 5, mode->gain);

	err = ov5648_write_table(info, mode_table[sensor_mode],	reg_list, 7);

	if (err) {
		pr_err("%d: ov5648_set_mode failed", err);
		return err;
	}
	info->mode = sensor_mode;
	/*
	update_truly_otp_wb(info, OV5648_RoverG_dec_base, OV5648_BoverG_dec_base);
	*/
	return 0;
}
static int ov5648_set_frame_length(
			struct ov5648_info *info,
			u32 frame_length)
{
	int ret;
	struct ov5648_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	ov5648_get_frame_length_regs(reg_list, frame_length);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = ov5648_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return ret;
}

static int ov5648_set_coarse_time(
			struct ov5648_info *info,
			u32 coarse_time)
{
	int ret;
	struct ov5648_reg reg_list[3];
	u8 *b_ptr = info->i2c_trans_buf;

	ov5648_get_coarse_time_regs(reg_list, coarse_time);
	pr_info("%s: coarsetime: %u \n",
			__func__,
			coarse_time);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	*b_ptr++ = reg_list[2].val & 0xff;
	ret = ov5648_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 5);

	return ret;
}

static int ov5648_set_gain(struct ov5648_info *info, u16 gain)
{
	int ret;
	struct ov5648_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	ov5648_get_gain_reg(reg_list, gain);

	*b_ptr++ = (reg_list[0].addr >> 8);
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = ov5648_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return ret;
}

static int ov5648_set_group_hold(
			struct ov5648_info *info,
			struct ov5648_ae *ae)
{
	int ret;
	int count = 0;
	bool groupHoldEnabled = false;
	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		groupHoldEnabled = true;

	if (groupHoldEnabled) {
		ret = ov5648_write_reg(info->i2c_client, 0x3208, 0x01);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		ov5648_set_gain(info, ae->gain);
	if (ae->coarse_time_enable)
		ov5648_set_coarse_time(info, ae->coarse_time);
	if (ae->frame_length_enable)
		ov5648_set_frame_length(info, ae->frame_length);

	if (groupHoldEnabled) {
		ret = ov5648_write_reg(info->i2c_client, 0x3208, 0x11);
		if (ret)
			return ret;

		ret = ov5648_write_reg(info->i2c_client, 0x3208, 0xa1);
		if (ret)
			return ret;
	}

	return 0;
}


static int ov5648_get_status(struct ov5648_info *info, u8 *status)
{
	int err;

	*status = 0;
	err = ov5648_read_reg(info->i2c_client, 0x002, status);
	return err;
}

static u32 ov5648_get_fuse_id(struct ov5648_info *info)
{
	u32 ret;
	if (info->fuse_id.size)
		return 0;

	ret = OV5648_ReadFuseIDFromOTP(info);

	if (ret) {
		info->fuse_id.size = 4;
		info->fuse_id.data[0] = ret >> 24;
		info->fuse_id.data[1] = ret >> 16;
		info->fuse_id.data[2] = ret >> 8;
		info->fuse_id.data[3] = ret;
		ret = 0;
	}
	dev_dbg(&info->i2c_client->dev, "%s mingji test: %d\n", __func__, ret);
	return ret;
}

static long ov5648_ioctl(
			struct file *file,
			unsigned int cmd,
			unsigned long arg)
{
	int err;
	struct ov5648_info *info = file->private_data;

	switch (cmd) {
	case OV5648_IOCTL_GET_FUSEID:
		dev_dbg(&info->i2c_client->dev, "%s mingji test!\n", __func__);
		err = ov5648_get_fuse_id(info);
		if (err) {
			pr_err("%s %d %d\n", __func__, __LINE__, err);
			return err;
		}
		if (copy_to_user((void __user *)arg,
					&info->fuse_id,
					sizeof(struct nvc_fuseid))) {
			pr_err("%s: %d: fail copy fuse id to user space\n",
					__func__, __LINE__);
			return -EFAULT;
		}
		return 0;

	case OV5648_IOCTL_SET_MODE:
	{
		struct ov5648_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov5648_mode))) {
			return -EFAULT;
		}

		return ov5648_set_mode(info, &mode);
	}
	case OV5648_IOCTL_SET_FRAME_LENGTH:
		return ov5648_set_frame_length(info, (u32)arg);
	case OV5648_IOCTL_SET_COARSE_TIME:
		return ov5648_set_coarse_time(info, (u32)arg);
	case OV5648_IOCTL_SET_GAIN:
		return ov5648_set_gain(info, (u16)arg);
	case OV5648_IOCTL_SET_GROUP_HOLD:
	{
		struct ov5648_ae ae;
		if (copy_from_user(&ae,
				(const void __user *)arg,
				sizeof(struct ov5648_ae))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return ov5648_set_group_hold(info, &ae);
	}
	case OV5648_IOCTL_GET_STATUS:
	{
		u8 status;

		err = ov5648_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 2))
			return -EFAULT;
		return 0;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static void ov5648_mclk_disable(struct ov5648_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int ov5648_mclk_enable(struct ov5648_info *info)
{
	int err;

	dev_dbg(&info->i2c_client->dev, "%s: enable MCLK\n", __func__);

	err = clk_set_rate(info->mclk, 24000000);
	if (!err) {
		err = clk_prepare_enable(info->mclk);
	}

	return err;
}

static struct ov5648_info *info;
static struct ov5648_info *k_info; /*Only for sysfs*/

static int ov5648_open(struct inode *inode, struct file *file)
{
	u8 status;
	int err;
	file->private_data = info;
	err = ov5648_mclk_enable(info);
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on(info->vreg);
	ov5648_get_status(info, &status);
	err = ov5648_edp_req(info, 0);
	if (err) {
		printk("%s not enough power not open camera\n", __func__);
		return err;
	}

	return 0;
}

int ov5648_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off(info->vreg);
	file->private_data = NULL;
	ov5648_mclk_disable(info);
	ov5648_edp_lowest(info);
	return 0;
}

static int ov5648_sysfs_init(struct ov5648_info *info);

static const struct file_operations ov5648_fileops = {
	.owner = THIS_MODULE,
	.open = ov5648_open,
	.unlocked_ioctl = ov5648_ioctl,
	.release = ov5648_release,
};

static struct miscdevice ov5648_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov5648",
	.fops = &ov5648_fileops,
};

static struct i2c_driver ov5648_i2c_driver;
static ssize_t ov5648_caminfo_show(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%s","ov5648");
}
static DRIVER_ATTR(subcaminfo, 0644, ov5648_caminfo_show, NULL);

static int ov5648_probe(
			struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	unsigned i, j;

	pr_info("---ov5648: Sensor probing---\n");
	info = kzalloc(sizeof(struct ov5648_info), GFP_KERNEL);
	if (!info) {
		pr_err("ov5648: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&ov5648_device);
	if (err) {
		pr_err("ov5648: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);

	info->mclk = devm_clk_get(&client->dev, "mclk2");

	pr_info(" mclk name: 0x%p", info->mclk);

	/*ov5648_vregs initializations*/
	for (i = 0; i < ARRAY_SIZE(ov5648_vregs); i++) {
		j = ov5648_vregs[i].vreg_num;
		info->vreg[j].vreg_name = ov5648_vregs[i].vreg_name;
		info->vreg[j].vreg_flag = false;
		info->vreg[j].vreg = regulator_get(
						&info->i2c_client->dev,
						info->vreg[j].vreg_name);
		if (IS_ERR(info->vreg[j].vreg)) {
			dev_dbg(&info->i2c_client->dev, "%s %s ERR: %d\n",
					__func__, info->vreg[j].vreg_name,
					(int)info->vreg[j].vreg);
			err |= PTR_ERR(info->vreg[j].vreg);
			info->vreg[j].vreg = NULL;
		} else
			dev_dbg(&info->i2c_client->dev, "%s: %s\n",
			__func__, info->vreg[j].vreg_name);
	}
	ov5648_edp_register(info);
	ov5648_sysfs_init(info);
	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);

	err = driver_create_file(&ov5648_i2c_driver.driver, &driver_attr_subcaminfo);
	if (err) {
                printk("failed to register subcaminfo attributes\n");
                err = 0;
	}

	return 0;
}

static int ov5648_remove(struct i2c_client *client)
{
	info = i2c_get_clientdata(client);
	if (info->edpc)
		edp_unregister_client(info->edpc);
	misc_deregister(&ov5648_device);
	kfree(info);
	return 0;
}

static ssize_t ov5648_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = k_info->i2c_client;
	u16 reg_read[4];
	u16 *p_reg = reg_read;
	ssize_t length;
	int err;

	dev_info(&client->dev, "%s %p\n", __func__, dev);
	if (!dev)
		return 0;

	if (k_info->mode < 0) {
		memset(reg_read, 0, sizeof(reg_read));
		goto ss_dis;
	}

	err = ov5648_read_reg_bulk(client, OV5648_REG_GLOBAL_FRAME_LENGTH,
		(u8 *)p_reg, 2);
	if (err)
		goto fail;
	p_reg++;

	err = ov5648_read_reg_bulk(client, OV5648_REG_GLOBAL_COARSE_TIME,
		(u8 *)p_reg, 2);
	if (err)
		goto fail;
	p_reg++;

	err = ov5648_read_reg(client, OV5648_REG_GLOBAL_COARSE_TIME + 2, (u8 *)p_reg);
	if (err)
		goto fail;
	p_reg++;

	*p_reg = 0;
	err = ov5648_read_reg_bulk(client, OV5648_REG_GLOBAL_GAIN,
		(u8 *)p_reg, 2);
	if (err)
		goto fail;

ss_dis:
	reg_read[1] = (reg_read[1] & 0x0fff);
	reg_read[2] = (reg_read[2] & 0x00f0);
	reg_read[1] = reg_read[1] + reg_read[2];

	length = sprintf(buf, "OV5648 sensor status:\n"
				"    Addr: 0x%02x, bus %d\n\n"
				"    Framelength(0x380e-0x380f) = 0x%04x\n"
				"    CoarseTime (0x3500-0x3502) = 0x%04x\n"
				"    Gain       (0x350a-0x350b) = 0x%04x\n"
				"    Flag                       = 0x%04x\n",
				client->addr, client->adapter->nr,
				be16_to_cpu(reg_read[0]),
				reg_read[1],
				be16_to_cpu(reg_read[3]),
				k_info->flag
				);

	return length;

fail:
	dev_err(&client->dev, "%s: get sensor params error.\n", __func__);

	return 0;
}

static ssize_t ov5648_attr_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = k_info->i2c_client;
	int err = 0;
	u32 val = 0;

	dev_info(&client->dev, "%s\n", __func__);

	if (!buf || count <= 1)
		return count;

	if (sscanf(buf + 1, "0x%x", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "0X%x", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "%xh", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "%xH", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "%d", &val) == 1)
		goto set_attr;

	dev_err(&client->dev, "SYNTAX ERROR: %s\n", buf);
	err = -EFAULT;

set_attr:
	if (!err) {
		switch (buf[0]) {
		case 'g':
			if (k_info->mode < 0) {
				dev_err(&client->dev, "Sensor is not ON!\n");
				break;
			}
			dev_info(&client->dev, "new gain = %x\n", val);
			err = ov5648_set_gain(k_info, (u16)val);
			break;
		case 'c':
			if (k_info->mode < 0) {
				dev_err(&client->dev, "Sensor is not ON!\n");
				break;
			}
			dev_info(&client->dev, "new coarse time = %x\n", val);
			err = ov5648_set_coarse_time(k_info, val);
			break;
		case 'f':
			if (k_info->mode < 0) {
				dev_err(&client->dev, "Sensor is not ON!\n");
				break;
			}
			dev_info(&client->dev, "new frame length = %x\n", val);
			err = ov5648_set_frame_length(k_info, val);
			break;

		}
	}

	return count;
}

static DEVICE_ATTR(d, 0755, ov5648_status_show, ov5648_attr_set);

static int ov5648_sysfs_init(struct ov5648_info *info)
{
	int ret ;

	/*NOTICE: The path of sysfs folder: /sys/cam_ov5648*/
	info->kobj_ov5648 = kobject_create_and_add("cam_ov5648", NULL);

	if (info->kobj_ov5648 == NULL) {
		dev_err(&info->i2c_client->dev,
			"%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret ;
	}

	ret = sysfs_create_file(info->kobj_ov5648, &dev_attr_d.attr);
	if (ret) {
		dev_err(&info->i2c_client->dev,
			"%s: sysfs_create_file failed\n", __func__);
		kobject_del(info->kobj_ov5648);
		info->kobj_ov5648 = NULL;
	}

	k_info = info;
	return ret ;
}

static const struct i2c_device_id ov5648_id[] = {
	{ "ov5648", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov5648_id);

int ov5648_i2c_resume(struct i2c_client *c)
{
	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);
	return 0;
}

static struct i2c_driver ov5648_i2c_driver = {
	.driver = {
		.name = "ov5648",
		.owner = THIS_MODULE,
	},
	.id_table = ov5648_id,
	.probe = ov5648_probe,
	.remove = ov5648_remove,
	.resume = ov5648_i2c_resume,
};

static int __init ov5648_init(void)
{
	pr_info("---ov5648 sensor driver loading---\n");
	return i2c_add_driver(&ov5648_i2c_driver);
}

static void __exit ov5648_exit(void)
{
	i2c_del_driver(&ov5648_i2c_driver);
}

module_init(ov5648_init);
module_exit(ov5648_exit);
MODULE_LICENSE("GPL v2");
