/*
 * 0v16825.c - 0v16825 sensor driver
 *
 *  * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
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
#include <linux/list.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <media/ov16825.h>
#include <linux/regmap.h>
#include <linux/edp.h>

#define OV16825_ID							0x0168
#define OV16825_SENSOR_TYPE					NVC_IMAGER_TYPE_RAW
#define OV16825_STARTUP_DELAY_MS			50
#define OV16825_RES_CHG_WAIT_TIME_MS		100
#define OV16825_SIZEOF_I2C_BUF				16
#define OV16825_TABLE_WAIT_MS				0
#define OV16825_TABLE_END					1
#define OV16825_TABLE_RESET					2
#define OV16825_TABLE_RESET_TIMEOUT			50
#define OV16825_NUM_MODES					ARRAY_SIZE(ov16825_mode_table)
#define OV16825_MODE_UNKNOWN				(OV16825_NUM_MODES + 1)
#define OV16825_LENS_MAX_APERTURE			10000 /* / _INT2FLOAT_DIVISOR */
#define OV16825_LENS_FNUMBER				2200 /* / _INT2FLOAT_DIVISOR */
#define OV16825_LENS_FOCAL_LENGTH			4880 /* / _INT2FLOAT_DIVISOR */
#define OV16825_LENS_VIEW_ANGLE_H			63900 /* / _INT2FLOAT_DIVISOR */
#define OV16825_LENS_VIEW_ANGLE_V			50100 /* / _INT2FLOAT_DIVISOR */
#define OV16825_I2C_TABLE_MAX_ENTRIES		400

static u16 ov16825_ids[] = {
	OV16825_ID,
};

static struct nvc_gpio_init ov18625_gpio[] = {
	{ OV16825_GPIO_TYPE_SHTDN, GPIOF_OUT_INIT_LOW, "shutdn", false, true, },
	{ OV16825_GPIO_TYPE_PWRDN, GPIOF_OUT_INIT_HIGH, "pwrdn", false, true, },
};

#define OV16825_EEPROM_BLOCK1_FUSEID_LEN	9
#define OV16825_EEPROM_BLOCK1_FLAG_INVALID	0
struct ov16825_eeprom_block1
{
	u8 fuseid[OV16825_EEPROM_BLOCK1_FUSEID_LEN];
	u8 flag;
};

#define OV16825_EEPROM_SIZE	256
#define OV16825_EEPROM_COUNT 6
struct ov16825_eeprom_device {
	struct i2c_client *i2c_client;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct regmap *regmap;
	int block_nr;
	u8 buf[OV16825_EEPROM_SIZE];
	int size;
};

static int eeprom_dev_addr[OV16825_EEPROM_COUNT] = {
	0xa0 >> 1,
	0xa4 >> 1,
	0xa6 >> 1,
	0xa8 >> 1,
	0xaa >> 1,
	0xac >> 1,
};

struct ov16825_info {
	atomic_t in_use;
	struct i2c_client *i2c_client;
	struct ov16825_platform_data *pdata;
	struct nvc_imager_cap *cap;
	struct miscdevice miscdev;
	int pwr_api;
	int pwr_dev;
	struct nvc_gpio gpio[ARRAY_SIZE(ov18625_gpio)];
	struct ov16825_power_rail regulators;
	bool power_on;
	u8 s_mode;
	struct ov16825_info *s_info;
	u32 mode_index;
	bool mode_valid;
	bool mode_enable;
	unsigned test_pattern;
	struct nvc_imager_static_nvc sdata;
	u8 i2c_buf[OV16825_SIZEOF_I2C_BUF];
	u8 bin_en;
	struct clk *mclk;
	struct nvc_fuseid fuse_id;
	struct ov16825_eeprom_device
		eeprom[OV16825_EEPROM_COUNT];
	struct edp_client *edpc;
	unsigned edp_state;
};

struct ov16825_reg {
	u16 addr;
	u16 val;
};

struct ov16825_mode_data {
	struct nvc_imager_mode sensor_mode;
	struct nvc_imager_dynamic_nvc sensor_dnvc;
	struct ov16825_reg *p_mode_i2c;
};

static struct nvc_imager_cap ov16825_dflt_cap = {
	.identifier		= "OV16825",
	.sensor_nvc_interface	= NVC_IMAGER_SENSOR_INTERFACE_SERIAL_A,
	.pixel_types[0]		= 0x101,
	.orientation		= 0,
	.direction		= 0,
	.initial_clock_rate_khz	= 6000,
	.clock_profiles[0] = {
		.external_clock_khz	= 24000,
		.clock_multiplier	= 8500000, /* value / 1,000,000 */
	},
	.clock_profiles[1] = {
		.external_clock_khz	= 0,
		.clock_multiplier	= 0,
	},
	.h_sync_edge		= 0,
	.v_sync_edge		= 0,
	.mclk_on_vgp0		= 0,
	.csi_port		= 0,
	.data_lanes		= 4,
	.virtual_channel_id	= 0,
	.discontinuous_clk_mode	= 1,
	.cil_threshold_settle	= 0,
	.min_blank_time_width	= 16,
	.min_blank_time_height	= 16,
	.preferred_mode_index	= 0,
	.focuser_guid		= 0,
	.torch_guid		= 0,
	.cap_version		= NVC_IMAGER_CAPABILITIES_VERSION2,
};

static struct ov16825_platform_data ov16825_dflt_pdata = {
	.cfg			= 0,
	.num			= 0,
	.sync			= 0,
	.dev_name		= "camera",
	.cap			= &ov16825_dflt_cap,
};

static struct nvc_imager_static_nvc ov16825_dflt_sdata = {
	.api_version		= NVC_IMAGER_API_STATIC_VER,
	.sensor_type		= OV16825_SENSOR_TYPE,
	.bits_per_pixel		= 10,
	.sensor_id		= OV16825_ID,
	.sensor_id_minor	= 0,
	.focal_len		= OV16825_LENS_FOCAL_LENGTH,
	.max_aperture		= OV16825_LENS_MAX_APERTURE,
	.fnumber		= OV16825_LENS_FNUMBER,
	.view_angle_h		= OV16825_LENS_VIEW_ANGLE_H,
	.view_angle_v		= OV16825_LENS_VIEW_ANGLE_V,
	.res_chg_wait_time	= OV16825_RES_CHG_WAIT_TIME_MS,
};

static void ov16825_read_fuseid_test(
	struct ov16825_info *info,
	int *len,
	unsigned char *buf);

static struct ov16825_reg tp_none_seq[] = {
	{0x5040, 0x00},
	{OV16825_TABLE_END, 0x0000}
};

static struct ov16825_reg tp_cbars_seq[] = {
	{0x5040, 0x80},
	{OV16825_TABLE_END, 0x0000}
};

static struct ov16825_reg *test_patterns[] = {
	tp_none_seq,
	tp_cbars_seq,
};


static struct ov16825_reg ov16825_1920x1080_i2c[] = {
	//{0x0103, 0x01},
	//{OV16825_TABLE_WAIT_MS, 20},
	{0x0300, 0x02},
	//{0x0302, 0x64},
	{0x0302, 0x40},
	{0x0305, 0x01},//0x04
	{0x0306, 0x00},
	{0x030b, 0x02},
	{0x030c, 0x14},
	{0x030e, 0x00},
	{0x0313, 0x02},
	{0x0314, 0x14},
	{0x031f, 0x00},
	{0x3022, 0x01},
	{0x3032, 0x80},
	{0x3601, 0xf8},
	{0x3602, 0x00},
	{0x3605, 0x50},
	{0x3606, 0x00},
	{0x3607, 0x2b},
	{0x3608, 0x16},
	{0x3609, 0x00},
	{0x360e, 0x99},
	{0x360f, 0x75},
	{0x3610, 0x69},
	{0x3611, 0x59},
	{0x3612, 0x40},
	{0x3613, 0x89},
	{0x3615, 0x44},//0x54
	{0x3617, 0x00},
	{0x3618, 0x20},
	{0x3619, 0x00},
	{0x361a, 0x10},
	{0x361c, 0x10},
	{0x361d, 0x00},
	{0x361e, 0x00},
	{0x3640, 0x15},
	{0x3641, 0x54},
	{0x3642, 0x63},
	{0x3643, 0x32},
	{0x3644, 0x03},
	{0x3645, 0x04},
	{0x3646, 0x85},
	{0x364a, 0x07},
	{0x3707, 0x08},
	{0x3718, 0x75},
	{0x371a, 0x55},
	{0x371c, 0x55},
	{0x3733, 0x80},
	{0x3760, 0x00},
	{0x3761, 0x30},
	{0x3762, 0x00},
	{0x3763, 0xc0},
	{0x3764, 0x03},
	{0x3765, 0x00},
	{0x3823, 0x08},
	{0x3827, 0x02},
	{0x3828, 0x00},
	{0x3832, 0x00},
	{0x3833, 0x00},
	{0x3834, 0x00},
	{0x3d85, 0x17},
	{0x3d8c, 0x70},
	{0x3d8d, 0xa0},
	{0x3f00, 0x02},
	{0x4001, 0x83},
	{0x400e, 0x00},
	{0x4011, 0x00},
	{0x4012, 0x00},
	{0x4200, 0x08},
	{0x4302, 0x7f},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4305, 0x00},
	{0x4501, 0x30},
	{0x4603, 0x20},
	{0x4b00, 0x22},
	{0x4903, 0x00},
	{0x5000, 0x7f},
	{0x5001, 0x01},
	{0x5004, 0x00},
	{0x5013, 0x20},
	{0x5051, 0x00},
	{0x5500, 0x01},
	{0x5501, 0x00},
	{0x5502, 0x07},
	{0x5503, 0xff},
	{0x5505, 0x6c},
	{0x5509, 0x02},
	{0x5780, 0xfc},
	{0x5781, 0xff},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},
	{0x5792, 0x00},
	{0x5980, 0x00},
	{0x5981, 0x21},
	{0x5982, 0x00},
	{0x5983, 0x00},
	{0x5984, 0x00},
	{0x5985, 0x00},
	{0x5986, 0x00},
	{0x5987, 0x00},
	{0x5988, 0x00},
	{0x3201, 0x15},
	{0x3202, 0x2a},
	{0x0305, 0x01},
	{0x030e, 0x01},
	{0x3018, 0x7a},
	{0x3031, 0x0a},
	{0x3603, 0x05},
	{0x3604, 0x02},
	{0x360a, 0x00},
	{0x360b, 0x02},
	{0x360c, 0x12},
	{0x360d, 0x04},
	{0x3614, 0x77},
	{0x3616, 0x30},
	{0x3631, 0x40},
	{0x3700, 0x30},
	{0x3701, 0x08},
	{0x3702, 0x11},
	{0x3703, 0x20},
	{0x3704, 0x08},
	{0x3705, 0x00},
	{0x3706, 0x84},
	{0x3708, 0x20},
	{0x3709, 0x3c},
	{0x370a, 0x01},
	{0x370b, 0x5d},
	{0x370c, 0x03},
	{0x370e, 0x20},
	{0x370f, 0x05},
	{0x3710, 0x20},
	{0x3711, 0x20},
	{0x3714, 0x31},
	{0x3719, 0x13},
	{0x371b, 0x03},
	{0x371d, 0x03},
	{0x371e, 0x09},
	{0x371f, 0x17},
	{0x3720, 0x0b},
	{0x3721, 0x18},
	{0x3722, 0x0b},
	{0x3723, 0x18},
	{0x3724, 0x04},
	{0x3725, 0x04},
	{0x3726, 0x02},
	{0x3727, 0x02},
	{0x3728, 0x02},
	{0x3729, 0x02},
	{0x372a, 0x25},
	{0x372b, 0x65},
	{0x372c, 0x55},
	{0x372d, 0x65},
	{0x372e, 0x53},
	{0x372f, 0x33},
	{0x3730, 0x33},
	{0x3731, 0x33},
	{0x3732, 0x03},
	{0x3734, 0x10},
	{0x3739, 0x03},
	{0x373a, 0x20},
	{0x373b, 0x0c},
	{0x373c, 0x1c},
	{0x373e, 0x0b},
	{0x373f, 0x80},
	{0x3800, 0x01},
	{0x3801, 0x80},
	{0x3802, 0x02},
	{0x3803, 0x94},
	{0x3804, 0x10},
	{0x3805, 0xbf},
	{0x3806, 0x0b},
	{0x3807, 0x0f},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x380c, 0x04},
	{0x380d, 0xb6},
	{0x380e, 0x08},
	{0x380f, 0xa4},
	{0x3811, 0x17},
	{0x3813, 0x02},
	{0x3814, 0x03},
	{0x3815, 0x01},
	{0x3820, 0x01},
	{0x3821, 0x07},
	{0x3829, 0x02},
	{0x382a, 0x03},
	{0x382b, 0x01},
	{0x3830, 0x08},
	{0x3f08, 0x20},
	{0x4002, 0x02},
	{0x4003, 0x04},
	{0x4800, 0x24},
	{0x4837, 0x1f},//0x14
	{0x3501, 0x8a},
	{0x3502, 0x00},
	{0x3508, 0x08},
	{0x3509, 0xff},
	{0x3638, 0x00},
	{0x0100, 0x01},
	{OV16825_TABLE_END, 0x0000},
};

static struct ov16825_reg ov16825_2304x1728_i2c[] = {
	//{0x0103, 0x01},
	//{OV16825_TABLE_WAIT_MS, 20},
	{0x0300, 0x02},
	//{0x0302, 0x50},
	{0x0302, 0x40},//0x32
	{0x0305, 0x01},
	{0x0306, 0x00},
	{0x030b, 0x02},
	{0x030c, 0x14},
	{0x030e, 0x00},
	{0x0313, 0x02},
	{0x0314, 0x14},
	{0x031f, 0x00},
	{0x3022, 0x01},
	{0x3032, 0x80},
	{0x3601, 0xf8},
	{0x3602, 0x00},
	{0x3605, 0x50},
	{0x3606, 0x00},
	{0x3607, 0x2b},
	{0x3608, 0x16},
	{0x3609, 0x00},
	{0x360e, 0x99},
	{0x360f, 0x75},
	{0x3610, 0x69},
	{0x3611, 0x59},
	{0x3612, 0x40},
	{0x3613, 0x89},
	{0x3615, 0x54},
	{0x3617, 0x00},
	{0x3618, 0x20},
	{0x3619, 0x00},
	{0x361a, 0x10},
	{0x361c, 0x10},
	{0x361d, 0x00},
	{0x361e, 0x00},
	{0x3640, 0x15},
	{0x3641, 0x54},
	{0x3642, 0x63},
	{0x3643, 0x32},
	{0x3644, 0x03},
	{0x3645, 0x04},
	{0x3646, 0x85},
	{0x364a, 0x07},
	{0x3707, 0x08},
	{0x3718, 0x75},
	{0x371a, 0x55},
	{0x371c, 0x55},
	{0x3733, 0x80},
	{0x3760, 0x00},
	{0x3761, 0x30},
	{0x3762, 0x00},
	{0x3763, 0xc0},
	{0x3764, 0x03},
	{0x3765, 0x00},
	{0x3823, 0x08},
	{0x3827, 0x02},
	{0x3828, 0x00},
	{0x3832, 0x00},
	{0x3833, 0x00},
	{0x3834, 0x00},
	{0x3d85, 0x17},
	{0x3d8c, 0x70},
	{0x3d8d, 0xa0},
	{0x3f00, 0x02},
	{0x4001, 0x83},
	{0x400e, 0x00},
	{0x4011, 0x00},
	{0x4012, 0x00},
	{0x4200, 0x08},
	{0x4302, 0x7f},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4305, 0x00},
	{0x4501, 0x30},
	{0x4603, 0x20},
	{0x4b00, 0x22},
	{0x4903, 0x00},
	{0x5000, 0x7f},
	{0x5001, 0x01},
	{0x5004, 0x00},
	{0x5013, 0x20},
	{0x5051, 0x00},
	{0x5500, 0x01},
	{0x5501, 0x00},
	{0x5502, 0x07},
	{0x5503, 0xff},
	{0x5505, 0x6c},
	{0x5509, 0x02},
	{0x5780, 0xfc},
	{0x5781, 0xff},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},
	{0x5792, 0x00},
	{0x5980, 0x00},
	{0x5981, 0x21},
	{0x5982, 0x00},
	{0x5983, 0x00},
	{0x5984, 0x00},
	{0x5985, 0x00},
	{0x5986, 0x00},
	{0x5987, 0x00},
	{0x5988, 0x00},
	{0x3201, 0x15},
	{0x3202, 0x2a},
	{0x0305, 0x01},
	{0x030e, 0x01},
	{0x3018, 0x7a},
	{0x3031, 0x0a},
	{0x3603, 0x05},
	{0x3604, 0x02},
	{0x360a, 0x00},
	{0x360b, 0x02},
	{0x360c, 0x12},
	{0x360d, 0x04},
	{0x3614, 0x77},
	{0x3616, 0x30},
	{0x3631, 0x40},
	{0x3700, 0x30},
	{0x3701, 0x08},
	{0x3702, 0x11},
	{0x3703, 0x20},
	{0x3704, 0x08},
	{0x3705, 0x00},
	{0x3706, 0x84},
	{0x3708, 0x20},
	{0x3709, 0x3c},
	{0x370a, 0x01},
	{0x370b, 0x5d},
	{0x370c, 0x03},
	{0x370e, 0x20},
	{0x370f, 0x05},
	{0x3710, 0x20},
	{0x3711, 0x20},
	{0x3714, 0x31},
	{0x3719, 0x13},
	{0x371b, 0x03},
	{0x371d, 0x03},
	{0x371e, 0x09},
	{0x371f, 0x17},
	{0x3720, 0x0b},
	{0x3721, 0x18},
	{0x3722, 0x0b},
	{0x3723, 0x18},
	{0x3724, 0x04},
	{0x3725, 0x04},
	{0x3726, 0x02},
	{0x3727, 0x02},
	{0x3728, 0x02},
	{0x3729, 0x02},
	{0x372a, 0x25},
	{0x372b, 0x65},
	{0x372c, 0x55},
	{0x372d, 0x65},
	{0x372e, 0x53},
	{0x372f, 0x33},
	{0x3730, 0x33},
	{0x3731, 0x33},
	{0x3732, 0x03},
	{0x3734, 0x10},
	{0x3739, 0x03},
	{0x373a, 0x20},
	{0x373b, 0x0c},
	{0x373c, 0x1c},
	{0x373e, 0x0b},
	{0x373f, 0x80},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x0c},
	{0x3804, 0x12},
	{0x3805, 0x3f},
	{0x3806, 0x0d},
	{0x3807, 0x97},
	{0x3808, 0x09},
	{0x3809, 0x00},
	{0x380a, 0x06},
	{0x380b, 0xc0},
	{0x380c, 0x05},
	{0x380d, 0xc9},
	{0x380e, 0x07},
	{0x380f, 0x08},

	{0x3811, 0x17},
	{0x3813, 0x02},
	{0x3814, 0x03},
	{0x3815, 0x01},
	{0x3820, 0x01},
	{0x3821, 0x07},
	{0x3829, 0x02},
	{0x382a, 0x03},
	{0x382b, 0x01},
	{0x3830, 0x08},
	{0x3f08, 0x20},
	{0x4002, 0x02},
	{0x4003, 0x04},
	{0x4800, 0x24},
	//{0x4837, 0x19},
	{0x4837, 0x1f},
	{0x3501, 0x6d},
	{0x3502, 0x60},

	/* test pattern */
	/* {0x5040, 0x80}, */

	{0x3508, 0x08},
	{0x3509, 0xff},
	{0x3638, 0x00},
	{0x0100, 0x01},
	{OV16825_TABLE_WAIT_MS, 30},
	{OV16825_TABLE_END, 0x0000},
};

static struct ov16825_reg ov16825_2176x1632_i2c[] = {
	//{0x0103, 0x01},
	//{OV16825_TABLE_WAIT_MS, 20},
	{0x0300, 0x02},
	{0x0302, 0x64},
	{0x0305, 0x01},
	{0x0306, 0x00},
	{0x030b, 0x02},
	{0x030c, 0x14},
	{0x030e, 0x00},
	{0x0313, 0x02},
	{0x0314, 0x14},
	{0x031f, 0x00},
	{0x3022, 0x01},
	{0x3032, 0x80},
	{0x3601, 0xf8},
	{0x3602, 0x00},
	{0x3605, 0x50},
	{0x3606, 0x00},
	{0x3607, 0x2b},
	{0x3608, 0x16},
	{0x3609, 0x00},
	{0x360e, 0x99},
	{0x360f, 0x75},
	{0x3610, 0x69},
	{0x3611, 0x59},
	{0x3612, 0x40},
	{0x3613, 0x89},
	{0x3615, 0x54},
	{0x3617, 0x00},
	{0x3618, 0x20},
	{0x3619, 0x00},
	{0x361a, 0x10},
	{0x361c, 0x10},
	{0x361d, 0x00},
	{0x361e, 0x00},
	{0x3640, 0x15},
	{0x3641, 0x54},
	{0x3642, 0x63},
	{0x3643, 0x32},
	{0x3644, 0x03},
	{0x3645, 0x04},
	{0x3646, 0x85},
	{0x364a, 0x07},
	{0x3707, 0x08},
	{0x3718, 0x75},
	{0x371a, 0x55},
	{0x371c, 0x55},
	{0x3733, 0x80},
	{0x3760, 0x00},
	{0x3761, 0x30},
	{0x3762, 0x00},
	{0x3763, 0xc0},
	{0x3764, 0x03},
	{0x3765, 0x00},
	{0x3823, 0x08},
	{0x3827, 0x02},
	{0x3828, 0x00},
	{0x3832, 0x00},
	{0x3833, 0x00},
	{0x3834, 0x00},
	{0x3d85, 0x17},
	{0x3d8c, 0x70},
	{0x3d8d, 0xa0},
	{0x3f00, 0x02},
	{0x4001, 0x83},
	{0x400e, 0x00},
	{0x4011, 0x00},
	{0x4012, 0x00},
	{0x4200, 0x08},
	{0x4302, 0x7f},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4305, 0x00},
	{0x4501, 0x30},
	{0x4603, 0x20},
	{0x4b00, 0x22},
	{0x4903, 0x00},
	{0x5000, 0x7f},
	{0x5001, 0x01},
	{0x5004, 0x00},
	{0x5013, 0x20},
	{0x5051, 0x00},
	{0x5500, 0x01},
	{0x5501, 0x00},
	{0x5502, 0x07},
	{0x5503, 0xff},
	{0x5505, 0x6c},
	{0x5509, 0x02},
	{0x5780, 0xfc},
	{0x5781, 0xff},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},
	{0x5792, 0x00},
	{0x5980, 0x00},
	{0x5981, 0x21},
	{0x5982, 0x00},
	{0x5983, 0x00},
	{0x5984, 0x00},
	{0x5985, 0x00},
	{0x5986, 0x00},
	{0x5987, 0x00},
	{0x5988, 0x00},
	{0x3201, 0x15},
	{0x3202, 0x2a},
	{0x0305, 0x01},
	{0x030e, 0x01},
	{0x3018, 0x7a},
	{0x3031, 0x0a},
	{0x3603, 0x05},
	{0x3604, 0x02},
	{0x360a, 0x00},
	{0x360b, 0x02},
	{0x360c, 0x12},
	{0x360d, 0x04},
	{0x3614, 0x77},
	{0x3616, 0x30},
	{0x3631, 0x40},
	{0x3700, 0x30},
	{0x3701, 0x08},
	{0x3702, 0x11},
	{0x3703, 0x20},
	{0x3704, 0x08},
	{0x3705, 0x00},
	{0x3706, 0x84},
	{0x3708, 0x20},
	{0x3709, 0x3c},
	{0x370a, 0x01},
	{0x370b, 0x5d},
	{0x370c, 0x03},
	{0x370e, 0x20},
	{0x370f, 0x05},
	{0x3710, 0x20},
	{0x3711, 0x20},
	{0x3714, 0x31},
	{0x3719, 0x13},
	{0x371b, 0x03},
	{0x371d, 0x03},
	{0x371e, 0x09},
	{0x371f, 0x17},
	{0x3720, 0x0b},
	{0x3721, 0x18},
	{0x3722, 0x0b},
	{0x3723, 0x18},
	{0x3724, 0x04},
	{0x3725, 0x04},
	{0x3726, 0x02},
	{0x3727, 0x02},
	{0x3728, 0x02},
	{0x3729, 0x02},
	{0x372a, 0x25},
	{0x372b, 0x65},
	{0x372c, 0x55},
	{0x372d, 0x65},
	{0x372e, 0x53},
	{0x372f, 0x33},
	{0x3730, 0x33},
	{0x3731, 0x33},
	{0x3732, 0x03},
	{0x3734, 0x10},
	{0x3739, 0x03},
	{0x373a, 0x20},
	{0x373b, 0x0c},
	{0x373c, 0x1c},
	{0x373e, 0x0b},
	{0x373f, 0x80},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x0c},
	{0x3804, 0x12},
	{0x3805, 0x3f},
	{0x3806, 0x0d},
	{0x3807, 0x97},
	{0x3808, 0x08},
	{0x3809, 0x80},
	{0x380a, 0x06},
	{0x380b, 0x60},
	{0x380c, 0x05},
	{0x380d, 0xf0},
	{0x380e, 0x06},
	{0x380f, 0xda},
	{0x3811, 0x17},
	{0x3813, 0x02},
	{0x3814, 0x03},
	{0x3815, 0x01},
	{0x3820, 0x01},
	{0x3821, 0x07},
	{0x3829, 0x02},
	{0x382a, 0x03},
	{0x382b, 0x01},
	{0x3830, 0x08},
	{0x3f08, 0x20},
	{0x4002, 0x02},
	{0x4003, 0x04},
	{0x4800, 0x24},
	{0x4837, 0x14},
	{0x3501, 0x6d},
	{0x3502, 0x60},

	{0x3508, 0x08},
	{0x3509, 0xff},
	{0x3638, 0x00},
	{0x0100, 0x01},
	{OV16825_TABLE_WAIT_MS, 30},
	{OV16825_TABLE_END, 0x0000},
};

static struct ov16825_reg ov16825_3840x2160_i2c[] = {
	//{0x0103, 0x01},
	//{OV16825_TABLE_WAIT_MS, 20},
	{0x0300, 0x02},
	{0x0302, 0x64},
	{0x0305, 0x01},
	{0x0306, 0x00},
	{0x030b, 0x02},
	{0x030c, 0x14},
	{0x030e, 0x00},
	{0x0313, 0x02},
	{0x0314, 0x14},
	{0x031f, 0x00},
	{0x3022, 0x01},
	{0x3032, 0x80},
	{0x3601, 0xf8},
	{0x3602, 0x00},
	{0x3605, 0x50},
	{0x3606, 0x00},
	{0x3607, 0x2b},
	{0x3608, 0x16},
	{0x3609, 0x00},
	{0x360e, 0x99},
	{0x360f, 0x75},
	{0x3610, 0x69},
	{0x3611, 0x59},
	{0x3612, 0x40},
	{0x3613, 0x89},
	{0x3615, 0x54},
	{0x3617, 0x00},
	{0x3618, 0x20},
	{0x3619, 0x00},
	{0x361a, 0x10},
	{0x361c, 0x10},
	{0x361d, 0x00},
	{0x361e, 0x00},
	{0x3640, 0x15},
	{0x3641, 0x54},
	{0x3642, 0x63},
	{0x3643, 0x32},
	{0x3644, 0x03},
	{0x3645, 0x04},
	{0x3646, 0x85},
	{0x364a, 0x07},
	{0x3707, 0x08},
	{0x3718, 0x75},
	{0x371a, 0x55},
	{0x371c, 0x55},
	{0x3733, 0x80},
	{0x3760, 0x00},
	{0x3761, 0x30},
	{0x3762, 0x00},
	{0x3763, 0xc0},
	{0x3764, 0x03},
	{0x3765, 0x00},
	{0x3823, 0x08},
	{0x3827, 0x02},
	{0x3828, 0x00},
	{0x3832, 0x00},
	{0x3833, 0x00},
	{0x3834, 0x00},
	{0x3d85, 0x17},
	{0x3d8c, 0x70},
	{0x3d8d, 0xa0},
	{0x3f00, 0x02},
	{0x4001, 0x83},
	{0x400e, 0x00},
	{0x4011, 0x00},
	{0x4012, 0x00},
	{0x4200, 0x08},
	{0x4302, 0x7f},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4305, 0x00},
	{0x4501, 0x30},
	{0x4603, 0x20},
	{0x4b00, 0x22},
	{0x4903, 0x00},
	{0x5000, 0x7f},
	{0x5001, 0x01},
	{0x5004, 0x00},
	{0x5013, 0x20},
	{0x5051, 0x00},
	{0x5500, 0x01},
	{0x5501, 0x00},
	{0x5502, 0x07},
	{0x5503, 0xff},
	{0x5505, 0x6c},
	{0x5509, 0x02},
	{0x5780, 0xfc},
	{0x5781, 0xff},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},
	{0x5792, 0x00},
	{0x5980, 0x00},
	{0x5981, 0x21},
	{0x5982, 0x00},
	{0x5983, 0x00},
	{0x5984, 0x00},
	{0x5985, 0x00},
	{0x5986, 0x00},
	{0x5987, 0x00},
	{0x5988, 0x00},
	{0x3201, 0x15},
	{0x3202, 0x2a},
	{0x0305, 0x01},
	{0x030e, 0x01},
	{0x3018, 0x7a},
	{0x3031, 0x0a},
	{0x3603, 0x00},
	{0x3604, 0x00},
	{0x360a, 0x00},
	{0x360b, 0x02},
	{0x360c, 0x12},
	{0x360d, 0x00},
	{0x3614, 0x77},
	{0x3616, 0x30},
	{0x3631, 0x60},
	{0x3700, 0x30},
	{0x3701, 0x08},
	{0x3702, 0x11},
	{0x3703, 0x20},
	{0x3704, 0x08},
	{0x3705, 0x00},
	{0x3706, 0x84},
	{0x3708, 0x20},
	{0x3709, 0x3c},
	{0x370a, 0x01},
	{0x370b, 0x5d},
	{0x370c, 0x03},
	{0x370e, 0x20},
	{0x370f, 0x05},
	{0x3710, 0x20},
	{0x3711, 0x20},
	{0x3714, 0x31},
	{0x3719, 0x13},
	{0x371b, 0x03},
	{0x371d, 0x03},
	{0x371e, 0x09},
	{0x371f, 0x17},
	{0x3720, 0x0b},
	{0x3721, 0x18},
	{0x3722, 0x0b},
	{0x3723, 0x18},
	{0x3724, 0x04},
	{0x3725, 0x04},
	{0x3726, 0x02},
	{0x3727, 0x02},
	{0x3728, 0x02},
	{0x3729, 0x02},
	{0x372a, 0x25},
	{0x372b, 0x65},
	{0x372c, 0x55},
	{0x372d, 0x65},
	{0x372e, 0x53},
	{0x372f, 0x33},
	{0x3730, 0x33},
	{0x3731, 0x33},
	{0x3732, 0x03},
	{0x3734, 0x10},
	{0x3739, 0x03},
	{0x373a, 0x20},
	{0x373b, 0x0c},
	{0x373c, 0x1c},
	{0x373e, 0x0b},
	{0x373f, 0x80},
	{0x3800, 0x01},
	{0x3801, 0xa0},
	{0x3802, 0x02},
	{0x3803, 0x96},
	{0x3804, 0x10},
	{0x3805, 0xbf},
	{0x3806, 0x0b},
	{0x3807, 0x0b},
	{0x3808, 0x0f},
	{0x3809, 0x00},
	{0x380a, 0x08},
	{0x380b, 0x70},
	{0x380c, 0x04},
	{0x380d, 0xca},
	{0x380e, 0x08},
	{0x380f, 0x92},
	{0x3811, 0x0f},
	{0x3813, 0x02},
	{0x3814, 0x01},
	{0x3815, 0x01},
	{0x3820, 0x00},
	{0x3821, 0x06},
	{0x3829, 0x00},
	{0x382a, 0x01},
	{0x382b, 0x01},
	{0x3830, 0x08},
	{0x3f08, 0x20},
	{0x4002, 0x04},
	{0x4003, 0x08},
	{0x4800, 0x24},
	{0x4837, 0x14},
	{0x3501, 0x88},
	{0x3502, 0xe0},
	{0x3508, 0x04},
	{0x3509, 0xff},
	{0x3638, 0x00},
	{0x0100, 0x01},
	{OV16825_TABLE_WAIT_MS, 30},
	{OV16825_TABLE_END, 0x0000},
};


static struct ov16825_reg ov16825_4608x3456_i2c[] = {
	//{0x0103, 0x01},
	//{OV16825_TABLE_WAIT_MS, 20},
	{0x0300, 0x02},
	{0x0302, 0x64},
	{0x0305, 0x01},
	{0x0306, 0x00},
	{0x030b, 0x02},
	{0x030c, 0x14},
	{0x030e, 0x00},
	{0x0313, 0x02},
	{0x0314, 0x14},
	{0x031f, 0x00},
	{0x3022, 0x01},
	{0x3032, 0x80},
	{0x3601, 0xf8},
	{0x3602, 0x00},
	{0x3605, 0x50},
	{0x3606, 0x00},
	{0x3607, 0x2b},
	{0x3608, 0x16},
	{0x3609, 0x00},
	{0x360e, 0x99},
	{0x360f, 0x75},
	{0x3610, 0x69},
	{0x3611, 0x59},
	{0x3612, 0x40},
	{0x3613, 0x89},
	{0x3615, 0x44},
	{0x3617, 0x00},
	{0x3618, 0x20},
	{0x3619, 0x00},
	{0x361a, 0x10},
	{0x361c, 0x10},
	{0x361d, 0x00},
	{0x361e, 0x00},
	{0x3640, 0x15},
	{0x3641, 0x54},
	{0x3642, 0x63},
	{0x3643, 0x32},
	{0x3644, 0x03},
	{0x3645, 0x04},
	{0x3646, 0x85},
	{0x364a, 0x07},
	{0x3707, 0x08},
	{0x3718, 0x75},
	{0x371a, 0x55},
	{0x371c, 0x55},
	{0x3733, 0x80},
	{0x3760, 0x00},
	{0x3761, 0x30},
	{0x3762, 0x00},
	{0x3763, 0xc0},
	{0x3764, 0x03},
	{0x3765, 0x00},
	{0x3823, 0x08},
	{0x3827, 0x02},
	{0x3828, 0x00},
	{0x3832, 0x00},
	{0x3833, 0x00},
	{0x3834, 0x00},
	{0x3d85, 0x17},
	{0x3d8c, 0x70},
	{0x3d8d, 0xa0},
	{0x3f00, 0x02},
	{0x4001, 0x83},
	{0x400e, 0x00},
	{0x4011, 0x00},
	{0x4012, 0x00},
	{0x4200, 0x08},
	{0x4302, 0x7f},
	{0x4303, 0xff},
	{0x4304, 0x00},
	{0x4305, 0x00},
	{0x4501, 0x30},
	{0x4603, 0x20},
	{0x4b00, 0x22},
	{0x4903, 0x00},
	{0x5000, 0x7f},
	{0x5001, 0x01},
	{0x5004, 0x00},
	{0x5013, 0x20},
	{0x5051, 0x00},
	{0x5500, 0x01},
	{0x5501, 0x00},
	{0x5502, 0x07},
	{0x5503, 0xff},
	{0x5505, 0x6c},
	{0x5509, 0x02},
	{0x5780, 0xfc},
	{0x5781, 0xff},
	{0x5787, 0x40},
	{0x5788, 0x08},
	{0x578a, 0x02},
	{0x578b, 0x01},
	{0x578c, 0x01},
	{0x578e, 0x02},
	{0x578f, 0x01},
	{0x5790, 0x01},
	{0x5792, 0x00},
	{0x5980, 0x00},
	{0x5981, 0x21},
	{0x5982, 0x00},
	{0x5983, 0x00},
	{0x5984, 0x00},
	{0x5985, 0x00},
	{0x5986, 0x00},
	{0x5987, 0x00},
	{0x5988, 0x00},
	{0x3201, 0x15},
	{0x3202, 0x2a},
	{0x0305, 0x01},
	{0x030e, 0x01},
	{0x3018, 0x7a},
	{0x3031, 0x0a},
	{0x3603, 0x00},
	{0x3604, 0x00},
	{0x360a, 0x00},
	{0x360b, 0x02},
	{0x360c, 0x12},
	{0x360d, 0x00},
	{0x3614, 0x77},
	{0x3616, 0x30},
	{0x3631, 0x60},
	{0x3700, 0x30},
	{0x3701, 0x08},
	{0x3702, 0x11},
	{0x3703, 0x20},
	{0x3704, 0x08},
	{0x3705, 0x00},
	{0x3706, 0x84},
	{0x3708, 0x20},
	{0x3709, 0x3c},
	{0x370a, 0x01},
	{0x370b, 0x5d},
	{0x370c, 0x03},
	{0x370e, 0x20},
	{0x370f, 0x05},
	{0x3710, 0x20},
	{0x3711, 0x20},
	{0x3714, 0x31},
	{0x3719, 0x13},
	{0x371b, 0x03},
	{0x371d, 0x03},
	{0x371e, 0x09},
	{0x371f, 0x17},
	{0x3720, 0x0b},
	{0x3721, 0x18},
	{0x3722, 0x0b},
	{0x3723, 0x18},
	{0x3724, 0x04},
	{0x3725, 0x04},
	{0x3726, 0x02},
	{0x3727, 0x02},
	{0x3728, 0x02},
	{0x3729, 0x02},
	{0x372a, 0x25},
	{0x372b, 0x65},
	{0x372c, 0x55},
	{0x372d, 0x65},
	{0x372e, 0x53},
	{0x372f, 0x33},
	{0x3730, 0x33},
	{0x3731, 0x33},
	{0x3732, 0x03},
	{0x3734, 0x10},
	{0x3739, 0x03},
	{0x373a, 0x20},
	{0x373b, 0x0c},
	{0x373c, 0x1c},
	{0x373e, 0x0b},
	{0x373f, 0x80},
	{0x3800, 0x00},
	{0x3801, 0x20},
	{0x3802, 0x00},
	{0x3803, 0x0e},
	{0x3804, 0x12},
	{0x3805, 0x3f},
	{0x3806, 0x0d},
	{0x3807, 0x93},
	{0x3808, 0x12},
	{0x3809, 0x00},
	{0x380a, 0x0d},
	{0x380b, 0x80},
	{0x380c, 0x05},
	{0x380d, 0xf8},
	{0x380e, 0x0d},
	{0x380f, 0xa2},
	{0x3811, 0x0f},
	{0x3813, 0x02},
	{0x3814, 0x01},
	{0x3815, 0x01},
	{0x3820, 0x00},
	{0x3821, 0x06},
	{0x3829, 0x00},
	{0x382a, 0x01},
	{0x382b, 0x01},
	{0x3830, 0x08},
	{0x3f08, 0x20},
	{0x4002, 0x04},
	{0x4003, 0x08},
	{0x4800, 0x24},
	{0x4837, 0x14},
	{0x3501, 0xd9},
	{0x3502, 0xe0},
	{0x3508, 0x04},
	{0x3509, 0xff},
	{0x3638, 0x00},
	{0x0100, 0x01},
	{OV16825_TABLE_WAIT_MS, 30},
	{OV16825_TABLE_END, 0x0000},
};

static struct ov16825_mode_data ov16825_1920x1080 = {
	.sensor_mode = {
		.res_x			= 1920,
		.res_y			= 1080,
		.active_start_x		= 0,
		.active_stary_y		= 0,
		.peak_frame_rate	= 30000, /* / _INT2FLOAT_DIVISOR */
		.pixel_aspect_ratio	= 1000, /* / _INT2FLOAT_DIVISOR */
		.pll_multiplier		= 8500, /* / _INT2FLOAT_DIVISOR */
		.crop_mode		= NVC_IMAGER_CROPMODE_NONE,
	},
	.sensor_dnvc = {
		.api_version		= NVC_IMAGER_API_DYNAMIC_VER,
		.region_start_x		= 0,
		.region_start_y		= 0,
		.x_scale		= 1,
		.y_scale		= 1,
		.bracket_caps		= 1,
		.flush_count		= 1,
		.init_intra_frame_skip	= 3,
		.ss_intra_frame_skip	= 2,
		.ss_frame_number	= 3,
		.coarse_time		= 0x974, /* reg 0x3500,0x3501,0x3502 */
		.max_coarse_diff	= 4,
		.min_exposure_course	= 2,
		.max_exposure_course	= 0x7ff0,
		.diff_integration_time	= 0, /* / _INT2FLOAT_DIVISOR */
		.line_length		= 0x4b6, /* reg 0x380c,0x380d */
		.frame_length		= 0x8a4, /* reg 0x380e,0x380f */
		.min_frame_length	= 0x8a4,
		.max_frame_length	= 0x7ff8,
		.min_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.max_gain		= 9000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain_bin_en	= 1000, /* / _INT2FLOAT_DIVISOR */
		.support_bin_control	= 0,
		.support_fast_mode	= 0,
		.pll_mult		= 0x32,
		.pll_div		= 0x06,
		.mode_sw_wait_frames	= 1500, /* / _INT2FLOAT_DIVISOR */
	},
	.p_mode_i2c			= ov16825_1920x1080_i2c,
};

static struct ov16825_mode_data ov16825_2176x1632 = {
	.sensor_mode = {
		.res_x			= 2176,
		.res_y			= 1632,
		.active_start_x		= 0,
		.active_stary_y		= 0,
		.peak_frame_rate	= 30000, /* / _INT2FLOAT_DIVISOR */
		.pixel_aspect_ratio	= 1000, /* / _INT2FLOAT_DIVISOR */
		.pll_multiplier		= 8500, /* / _INT2FLOAT_DIVISOR */
		.crop_mode		= NVC_IMAGER_CROPMODE_NONE,
	},
	.sensor_dnvc = {
		.api_version		= NVC_IMAGER_API_DYNAMIC_VER,
		.region_start_x		= 0,
		.region_start_y		= 0,
		.x_scale		= 1,
		.y_scale		= 1,
		.bracket_caps		= 1,
		.flush_count		= 1,
		.init_intra_frame_skip	= 3,
		.ss_intra_frame_skip	= 2,
		.ss_frame_number	= 3,
		.coarse_time		= 0x974, /* reg 0x3500,0x3501,0x3502 */
		.max_coarse_diff	= 4,
		.min_exposure_course	= 2,
		.max_exposure_course	= 0x7ff0,
		.diff_integration_time	= 0, /* / _INT2FLOAT_DIVISOR */
		.line_length		= 0x5f0, /* reg 0x380c,0x380d */
		.frame_length		= 0x6da, /* reg 0x380e,0x380f */
		.min_frame_length	= 0x6da,
		.max_frame_length	= 0x7ff8,
		.min_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.max_gain		= 9000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain_bin_en	= 1000, /* / _INT2FLOAT_DIVISOR */
		.support_bin_control	= 0,
		.support_fast_mode	= 0,
		.pll_mult		= 0x32,
		.pll_div		= 0x06,
		.mode_sw_wait_frames	= 1500, /* / _INT2FLOAT_DIVISOR */
	},
	.p_mode_i2c			= ov16825_2176x1632_i2c,
};

static struct ov16825_mode_data ov16825_2304x1728 = {
	.sensor_mode = {
		.res_x			= 2304,
		.res_y			= 1728,
		.active_start_x		= 0,
		.active_stary_y		= 0,
		.peak_frame_rate	= 30000, /* / _INT2FLOAT_DIVISOR */
		.pixel_aspect_ratio	= 1000, /* / _INT2FLOAT_DIVISOR */
		.pll_multiplier		= 8500, /* / _INT2FLOAT_DIVISOR */
		.crop_mode		= NVC_IMAGER_CROPMODE_NONE,
	},
	.sensor_dnvc = {
		.api_version		= NVC_IMAGER_API_DYNAMIC_VER,
		.region_start_x		= 0,
		.region_start_y		= 0,
		.x_scale		= 1,
		.y_scale		= 1,
		.bracket_caps		= 1,
		.flush_count		= 1,
		.init_intra_frame_skip	= 3,
		.ss_intra_frame_skip	= 2,
		.ss_frame_number	= 3,
		.coarse_time		= 0x974, /* reg 0x3500,0x3501,0x3502 */
		.max_coarse_diff	= 4,
		.min_exposure_course	= 2,
		.max_exposure_course	= 0x7ff0,
		.diff_integration_time	= 0, /* / _INT2FLOAT_DIVISOR */
		.line_length		= 0x5c9, /* reg 0x380c,0x380d */
		.frame_length		= 0x708, /* reg 0x380e,0x380f */
		.min_frame_length	= 0x708,
		.max_frame_length	= 0x7ff8,
		.min_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.max_gain		= 9000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain_bin_en	= 1000, /* / _INT2FLOAT_DIVISOR */
		.support_bin_control	= 0,
		.support_fast_mode	= 0,
		.pll_mult		= 0x32,
		.pll_div		= 0x06,
		.mode_sw_wait_frames	= 1500, /* / _INT2FLOAT_DIVISOR */
	},
	.p_mode_i2c			= ov16825_2304x1728_i2c,
};

static struct ov16825_mode_data ov16825_3840x2160 = {
	.sensor_mode = {
		.res_x			= 3840,
		.res_y			= 2160,
		.active_start_x		= 0,
		.active_stary_y		= 0,
		.peak_frame_rate	= 30000, /* / _INT2FLOAT_DIVISOR */
		.pixel_aspect_ratio	= 1000, /* / _INT2FLOAT_DIVISOR */
		.pll_multiplier		= 13000, /* / _INT2FLOAT_DIVISOR */
		.crop_mode		= NVC_IMAGER_CROPMODE_NONE,
	},
	.sensor_dnvc = {
		.api_version		= NVC_IMAGER_API_DYNAMIC_VER,
		.region_start_x		= 0,
		.region_start_y		= 0,
		.x_scale		= 1,
		.y_scale		= 1,
		.bracket_caps		= 1,
		.flush_count		= 1,
		.init_intra_frame_skip	= 3,
		.ss_intra_frame_skip	= 2,
		.ss_frame_number	= 3,
		.coarse_time		= 0x88e, /* reg 0x3500,0x3501,0x3502 */
		.max_coarse_diff	= 4,
		.min_exposure_course	= 2,
		.max_exposure_course	= 0x7ff0,
		.diff_integration_time	= 0, /* / _INT2FLOAT_DIVISOR */
		.line_length		= 0x04ca, /* reg 0x380c,0x380d */
		.frame_length		= 0x0892, /* reg 0x380e,0x380f */
		.min_frame_length	= 0x0892,
		.max_frame_length	= 0x7ff8,
		.min_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.max_gain		= 9000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain_bin_en	= 1000, /* / _INT2FLOAT_DIVISOR */
		.support_bin_control	= 0,
		.support_fast_mode	= 0,
		.pll_mult		= 0x32,
		.pll_div		= 0x06,
		.mode_sw_wait_frames	= 1500, /* / _INT2FLOAT_DIVISOR */
	},
	.p_mode_i2c			= ov16825_3840x2160_i2c,
};

static struct ov16825_mode_data ov16825_4608x3456 = {
	.sensor_mode = {
		.res_x			= 4608,
		.res_y			= 3456,
		.active_start_x		= 0,
		.active_stary_y		= 0,
		.peak_frame_rate	= 15000, /* / _INT2FLOAT_DIVISOR */
		.pixel_aspect_ratio	= 1000, /* / _INT2FLOAT_DIVISOR */
		.pll_multiplier		= 12333, /* / _INT2FLOAT_DIVISOR */
		.crop_mode		= NVC_IMAGER_CROPMODE_NONE,
	},
	.sensor_dnvc = {
		.api_version		= NVC_IMAGER_API_DYNAMIC_VER,
		.region_start_x		= 0,
		.region_start_y		= 0,
		.x_scale		= 1,
		.y_scale		= 1,
		.bracket_caps		= 1,
		.flush_count		= 1,
		.init_intra_frame_skip	= 3,
		.ss_intra_frame_skip	= 2,
		.ss_frame_number	= 3,
		.coarse_time		= 0xbd4, /* reg 0x3500,0x3501,0x3502 */
		.max_coarse_diff	= 4,
		.min_exposure_course	= 2,
		.max_exposure_course	= 0x7ff0,
		.diff_integration_time	= 0, /* / _INT2FLOAT_DIVISOR */
		.line_length		= 0x05f8, /* reg 0x380c,0x380d */
		.frame_length		= 0xda2, /* reg 0x380e,0x380f */
		.min_frame_length	= 0xda2,
		.max_frame_length	= 0x7ff8,
		.min_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.max_gain		= 9000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain		= 1000, /* / _INT2FLOAT_DIVISOR */
		.inherent_gain_bin_en	= 1000, /* / _INT2FLOAT_DIVISOR */
		.support_bin_control	= 0,
		.support_fast_mode	= 0,
		.pll_mult		= 0x32,
		.pll_div		= 0x06,
		.mode_sw_wait_frames	= 1500, /* / _INT2FLOAT_DIVISOR */
	},
	.p_mode_i2c			= ov16825_4608x3456_i2c,
};


void *unused_p = &ov16825_4608x3456;
void *unused_p_2 = &ov16825_2176x1632;
void *unused_p_1 = &ov16825_1920x1080;
void *unused_p_3 = &ov16825_3840x2160;
void *unused_p_4 = &ov16825_2304x1728;
static struct ov16825_mode_data *ov16825_mode_table[] = {
#if 1
	[0] = &ov16825_4608x3456,
	[1] = &ov16825_3840x2160,
	[2] = &ov16825_2304x1728,
	[3] = &ov16825_1920x1080,
#else
	[0] = &ov16825_3840x2160,
#endif
};

static void ov16825_edp_callback(unsigned int new_state, void *priv_data)
{
}

static void ov16825_edp_lowest(struct ov16825_info *info)
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

static void ov16825_edp_register(struct ov16825_info *info)
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

	strncpy(edpc->name, "ov16825", EDP_NAME_LEN - 1);
	edpc->name[EDP_NAME_LEN - 1] = 0;
	edpc->private_data = info;
	edpc->throttle = ov16825_edp_callback;
	edpc->notify_promotion = ov16825_edp_callback;

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
	ov16825_edp_lowest(info);
}

static int ov16825_edp_req(struct ov16825_info *info, unsigned new_state)
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

static int ov16825_i2c_rd8(struct ov16825_info *info, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[3];
	buf[0] = (reg >> 8);
	buf[1] = (reg & 0x00FF);
	msg[0].addr = info->i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = &buf[0];
	msg[1].addr = info->i2c_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf[2];
	*val = 0;
	if (i2c_transfer(info->i2c_client->adapter, msg, 2) != 2)
		return -EIO;

	*val = buf[2];
	return 0;
}

static int ov16825_i2c_rd16(struct ov16825_info *info, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u8 buf[4];

	buf[0] = (reg >> 8);
	buf[1] = (reg & 0x00FF);
	msg[0].addr = info->i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = &buf[0];
	msg[1].addr = info->i2c_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = &buf[2];
	*val = 0;
	if (i2c_transfer(info->i2c_client->adapter, msg, 2) != 2)
		return -EIO;

	*val = (((u16)buf[2] << 8) | (u16)buf[3]);
	return 0;
}

static int ov16825_i2c_wr8(struct ov16825_info *info, u16 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[3];

	buf[0] = (reg >> 8);
	buf[1] = (reg & 0x00FF);
	buf[2] = val;
	msg.addr = info->i2c_client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = &buf[0];
	if (i2c_transfer(info->i2c_client->adapter, &msg, 1) != 1)
		return -EIO;

	return 0;
}

static int ov16825_i2c_rd_table(struct ov16825_info *info,
				struct ov16825_reg table[])
{
	struct ov16825_reg *p_table = table;
	u8 val;
	int err = 0;

	while (p_table->addr != OV16825_TABLE_END) {
		err = ov16825_i2c_rd8(info, p_table->addr, &val);
		if (err)
			return err;

		p_table->val = (u16)val;
		p_table++;
	}

	return err;
}

static int ov1682530_i2c_wr_blk(struct ov16825_info *info, u8 *buf, int len)
{
	struct i2c_msg msg;

	msg.addr = info->i2c_client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buf;
	if (i2c_transfer(info->i2c_client->adapter, &msg, 1) != 1)
		return -EIO;

	return 0;
}

static int ov16825_i2c_wr_table(struct ov16825_info *info,
				struct ov16825_reg table[])
{
	int err;
	const struct ov16825_reg *next;
	const struct ov16825_reg *n_next;
	u8 *b_ptr = info->i2c_buf;
	u16 buf_count = 0;
	u8 reset_status = 1;
	u8 reset_tries_left = OV16825_TABLE_RESET_TIMEOUT;

	for (next = table; next->addr != OV16825_TABLE_END; next++) {
		if (next->addr == OV16825_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		} else if (next->addr == OV16825_TABLE_RESET) {
			err = ov16825_i2c_wr8(info, 0x0103, 0x01);
			if (err)
				return err;
			while (reset_status) {
				usleep_range(200, 300);
				if (reset_tries_left < 1)
					return -EIO;
				err = ov16825_i2c_rd8(info, 0x0103,
							&reset_status);
				if (err)
					return err;
				reset_status &= 0x01;
				reset_tries_left -= 1;
			}
			continue;
		}

		if (!buf_count) {
			b_ptr = info->i2c_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xFF;
			buf_count = 2;
		}
		*b_ptr++ = next->val;
		buf_count++;
		n_next = next + 1;

		if (n_next->addr == next->addr + 1 &&
				n_next->addr != OV16825_TABLE_WAIT_MS &&
				buf_count < OV16825_SIZEOF_I2C_BUF &&
				n_next->addr != OV16825_TABLE_RESET &&
				n_next->addr != OV16825_TABLE_END)
			continue;

		err = ov1682530_i2c_wr_blk(info, info->i2c_buf, buf_count);
		if (err)
			return err;

		buf_count = 0;
	}

	return 0;
}

static struct ov16825_reg regs_prepare_NVM[] = {
	{0x0300, 0x02},
	{0x0302, 0x64},
	{0x0305, 0x01},
	{0x0306, 0x00},
	{0x030b, 0x02},
	{0x030c, 0x14},
	{0x030e, 0x00},
	{0x0313, 0x02},
	{0x0314, 0x14},
	{0x031f, 0x00},
	{0x0100, 0x01},
	{OV16825_TABLE_WAIT_MS, 20},
	{OV16825_TABLE_END, 0x0000},
};

static int ov16825_NVM_read_byte(struct ov16825_info *info,
		u16 addr,
		u8 *byte)
{
	int err = 0;

	if (addr < 0x7000 || addr > 0x7800)
		return -1;

	err |= ov16825_i2c_wr_table(info, regs_prepare_NVM);
	if (err)
		return err;

	err = ov16825_i2c_wr8(info, 0x3d81, 0x01);
	if (err)
		return err;

	msleep(20);

	err = ov16825_i2c_rd8(info, addr, byte);
	if (err)
		return err;

	return err;
}

static int ov16825_NVM_read_bytes(struct ov16825_info *info,
		u16 addr,
		u8 *byte,
		u16 length)
{
	int err = 0;
	int i = 0;

	if (addr < 0x7000 || addr + length > 0x7800)
		return -1;

	err |= ov16825_i2c_wr_table(info, regs_prepare_NVM);
	if (err)
		return err;

	err = ov16825_i2c_wr8(info, 0x3d81, 0x01);
	if (err)
		return err;

	msleep(20);

	for (i = 0; i < length; i++) {
		err = ov16825_i2c_rd8(info, addr + i, byte + i);
		if (err)
			return err;
	}

	return 0;
}

static inline void ov16825_frame_length_reg(struct ov16825_reg *regs,
					u32 frame_length)
{
	regs->addr = 0x380e;
	regs->val = (frame_length >> 8) & 0x7f;
	(regs + 1)->addr = 0x380f;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void ov16825_coarse_time_reg(struct ov16825_reg *regs,
					u32 coarse_time)
{
	if (coarse_time % 2)
	{
		coarse_time--;
	}
	regs->addr = 0x3500;
	regs->val = (coarse_time >> 12) & 0x07;
	(regs + 1)->addr = 0x3501;
	(regs + 1)->val = (coarse_time >> 4) & 0xFF;
	(regs + 2)->addr = 0x3502;
	(regs + 2)->val = (coarse_time << 4) & 0xFF;
}


static inline void ov16825_gain_reg(struct ov16825_reg *regs, u32 gain)
{
#if 0
	unsigned char bits = 0;
	if (gain < 1000)
		gain = 1000;

	if (gain >= 1000 && gain < 2000) {
		bits = 0x00;
	}

	if (gain >= 2000 && gain < 4000) {
		bits = 1;
		gain >>= 1;
	}

	if (gain >= 4000 && gain < 8000) {
		bits = 2;
		gain >>= 2;
	}

	if (gain >= 8000 && gain < 16000) {
		bits = 3;
		gain >>= 3;
	}

	gain = (gain - 1000) * (0xff - 0x80) / (1992 - 1000) + 0x80;
	if (gain > 0xff)
		gain = 0xff;

	bits <<= 2;

	(regs + 1)->addr = 0x3508;
	(regs + 1)->val = bits;
	(regs + 0)->addr = 0x3509;
	(regs + 0)->val = (unsigned char)gain;
#else
	(regs + 1)->addr = 0x3508;
	(regs + 1)->val = (unsigned char)((gain & 0x0000ff00) >> 8);
	(regs + 0)->addr = 0x3509;
	(regs + 0)->val = (unsigned char)((gain & 0x000000ff) >> 0);
#endif

}

static int ov16825_bin_wr(struct ov16825_info *info, u8 enable)
{
	int err = 0;

	if (enable == info->bin_en)
		return 0;

	if (!info->mode_valid || !ov16825_mode_table[info->mode_index]->
				sensor_dnvc.support_bin_control)
		return -EINVAL;

	if (!err)
		info->bin_en = enable;
	dev_dbg(&info->i2c_client->dev, "%s bin_en=%x err=%d\n",
		__func__, info->bin_en, err);
	return err;
}

static void group_hold_end(struct ov16825_info *info)
{
	int err = 0;
	err |= ov16825_i2c_wr8(info, 0x3208, 0x11);
	err |= ov16825_i2c_wr8(info, 0x320b, 0x00);
	err |= ov16825_i2c_wr8(info, 0x3208, 0xe1);
}

static int ov16825_exposure_wr(struct ov16825_info *info,
				struct nvc_imager_bayer *mode)
{
	struct ov16825_reg reg_list[8];
	int err;
	bool groupHoldEnable = 1;

	if (groupHoldEnable)
		err |= ov16825_i2c_wr8(info, 0x3208, 0x01);

	ov16825_frame_length_reg(reg_list, mode->frame_length);
	ov16825_coarse_time_reg(reg_list + 2, mode->coarse_time);
	ov16825_gain_reg(reg_list + 5, mode->gain);
	reg_list[7].addr = OV16825_TABLE_END;
	err = ov16825_i2c_wr_table(info, reg_list);
	if (!err)
		err = ov16825_bin_wr(info, mode->bin_en);

	if (groupHoldEnable) {
		group_hold_end(info);
	}

	#if 0
	{
		u8 v = 0;
		u16 v16 = 0;

		ov16825_i2c_rd8(info, 0x380c, &v);
		v16 = (v & 0x7f) << 8;
		ov16825_i2c_rd8(info, 0x380d, &v);
		v16 |= v;
		printk("ov %s line length %d\n", __func__, v16);


		v = 0;
		v16 = 0;
		ov16825_i2c_rd8(info, 0x380e, &v);
		v16 = (v & 0x7f) << 8;
		ov16825_i2c_rd8(info, 0x380f, &v);
		v16 |= v;
		printk("ov %s frame length %d\n", __func__, v16);
	}
	#endif

	return err;
}


static int ov16825_gain_wr(struct ov16825_info *info, u32 gain)
{
#if 0
	unsigned char bits = 0;
	int err = 0;
	if (gain < 1000)
		gain = 1000;

	if (gain >= 1000 && gain < 2000) {
		bits = 0x00;
	}

	if (gain >= 2000 && gain < 4000) {
		bits = 1;
		gain >>= 1;
	}

	if (gain >= 4000 && gain < 8000) {
		bits = 2;
		gain >>= 2;
	}

	if (gain >= 8000 && gain < 16000) {
		bits = 3;
		gain >>= 3;
	}

	gain = (gain - 1000) * (0xff - 0x80) / (1992 - 1000) + 0x80;
	if (gain > 0xff)
		gain = 0xff;

	bits <<= 2;

	err = ov16825_i2c_wr8(info, 0x3508, (u8)bits);
	err |= ov16825_i2c_wr8(info, 0x3509, (u8)gain);

	return err;
#else
	struct ov16825_reg reg_list[2] = {{0, 0}, {0, 0}};
	int err = 0;

	ov16825_gain_reg(reg_list, gain);
	err |= ov16825_i2c_wr8(info, (reg_list + 0)->addr, (reg_list + 0)->val);
	err |= ov16825_i2c_wr8(info, (reg_list + 1)->addr, (reg_list + 1)->val);

	return err;
#endif
}


static int ov16825_gain_rd(struct ov16825_info *info, u32 *gain)
{
#if 0
	int err;
	unsigned char bits = 0;
	u32 gain_tmp = 0;

	err = ov16825_i2c_rd8(info, 0x3508, (u8 *)&bits);
	if (err)
		return err;

	err = ov16825_i2c_rd8(info, 0x3509, (u8 *)&gain_tmp);
	if (err)
		return err;

	bits &= 0x0c;
	bits >>= 2;


	gain_tmp = (1992 - 1000) * (gain_tmp - 0x80) / (0xff - 0x80) + 1000;
	if (gain_tmp > 1992)
		gain_tmp = 1992;

	gain_tmp <<= bits;

	*gain = gain_tmp;

	return err;
#else
	int err;
	unsigned char bits = 0;
	u32 gain_tmp = 0;

	err = ov16825_i2c_rd8(info, 0x3508, (u8 *)&bits);
	if (err)
		return err;

	err = ov16825_i2c_rd8(info, 0x3509, (u8 *)&gain_tmp);
	if (err)
		return err;

	gain_tmp |= bits << 8;
	*gain = gain_tmp;
	return err;
#endif
}

static int ov16825_group_hold_wr(struct ov16825_info *info,
				struct nvc_imager_ae *ae)
{
	int err = 0;
	bool groupHoldEnable;
	struct ov16825_reg reg_list[8];
	int count = 0;
	int offset = 0;


	if (ae->gain_enable)
		count += 1;
	if (ae->coarse_time_enable)
		count += 1;
	if (ae->frame_length_enable)
		count += 1;
	groupHoldEnable = (count > 1) ? 1 : 0;

	if (groupHoldEnable)
		err |= ov16825_i2c_wr8(info, 0x3208, 0x01);

	if (ae->gain_enable) {
		ov16825_gain_reg(reg_list + offset, ae->gain);
		offset += 2;
	}
	if (ae->frame_length_enable) {
		/* printk("ov %s frame length %d\n", __func__, ae->frame_length); */
		ov16825_frame_length_reg(reg_list + offset, ae->frame_length);
		offset += 2;
	}

	if (ae->coarse_time_enable) {
		ov16825_coarse_time_reg(reg_list + offset, ae->coarse_time);
		offset += 3;
	}
	reg_list[offset].addr = OV16825_TABLE_END;
	err |= ov16825_i2c_wr_table(info, reg_list);

	if (groupHoldEnable) {
		group_hold_end(info);
	}

	#if 0
	{
		u8 v = 0;
		u16 v16 = 0;

		ov16825_i2c_rd8(info, 0x380c, &v);
		v16 = (v & 0x7f) << 8;
		ov16825_i2c_rd8(info, 0x380d, &v);
		v16 |= v;
		printk("ov %s line length %d\n", __func__, v16);
	}
	#endif

	return err;
}

static int ov16825_test_pattern_wr(struct ov16825_info *info, unsigned pattern)
{
	if (pattern >= ARRAY_SIZE(test_patterns))
		return -EINVAL;

	return ov16825_i2c_wr_table(info, test_patterns[pattern]);
}

static int ov16825_gpio_rd(struct ov16825_info *info,
			enum ov16825_gpio_type type)
{
	int val = -EINVAL;

	if (info->gpio[type].gpio) {
		val = gpio_get_value_cansleep(info->gpio[type].gpio);
		dev_dbg(&info->i2c_client->dev, "%s %u %d\n",
			__func__, info->gpio[type].gpio, val);
		if (!info->gpio[type].active_high)
			val = !val;
		val &= 1;
	}
	return val; /* return read value or error */
}

static int ov16825_gpio_wr(struct ov16825_info *info,
			enum ov16825_gpio_type type,
			int val) /* val: 0=deassert, 1=assert */
{
	int err = -EINVAL;

	if (info->gpio[type].gpio) {
		if (!info->gpio[type].active_high)
			val = !val;
		val &= 1;
		err = val;
		gpio_set_value_cansleep(info->gpio[type].gpio, val);
		dev_dbg(&info->i2c_client->dev, "%s %u %d\n",
			__func__, info->gpio[type].gpio, val);
	}
	return err; /* return value written or error */
}

static void ov16825_gpio_shutdn(struct ov16825_info *info, int val)
{
	ov16825_gpio_wr(info, OV16825_GPIO_TYPE_SHTDN, val);
}

static void ov16825_gpio_pwrdn(struct ov16825_info *info, int val)
{
	int prev_val;

	prev_val = ov16825_gpio_rd(info, OV16825_GPIO_TYPE_PWRDN);
	if ((prev_val < 0) || (val == prev_val))
		return;

	ov16825_gpio_wr(info, OV16825_GPIO_TYPE_PWRDN, val);
	if (!val && prev_val)
		/* if transition from assert to deassert then delay for I2C */
		msleep(OV16825_STARTUP_DELAY_MS);
}

static void ov16825_gpio_exit(struct ov16825_info *info)
{
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(ov18625_gpio); i++) {
		if (info->gpio[i].gpio && info->gpio[i].own)
			gpio_free(info->gpio[i].gpio);
	}
}

static void ov16825_gpio_init(struct ov16825_info *info)
{
	char label[32];
	unsigned long flags;
	unsigned type;
	unsigned i;
	unsigned j;
	int err;

	if (!info->pdata->gpio_count || !info->pdata->gpio)
		return;

	for (i = 0; i < ARRAY_SIZE(ov18625_gpio); i++) {
		type = ov18625_gpio[i].gpio_type;
		for (j = 0; j < info->pdata->gpio_count; j++) {
			if (type == info->pdata->gpio[j].gpio_type)
				break;
		}
		if (j == info->pdata->gpio_count)
			continue;

		info->gpio[type].gpio = info->pdata->gpio[j].gpio;
		if (ov18625_gpio[i].use_flags) {
			flags = ov18625_gpio[i].flags;
			info->gpio[type].active_high =
						ov18625_gpio[i].active_high;
		} else {
			info->gpio[type].active_high =
					info->pdata->gpio[j].active_high;
			if (info->gpio[type].active_high)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		}
		if (!info->pdata->gpio[j].init_en)
			continue;

		snprintf(label, sizeof(label), "ov16825_%u_%s",
			 info->pdata->num, ov18625_gpio[i].label);
		err = gpio_request_one(info->gpio[type].gpio, flags, label);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s ERR %s %u\n",
				__func__, label, info->gpio[type].gpio);
		} else {
			info->gpio[type].own = true;
			dev_dbg(&info->i2c_client->dev, "%s %s %u\n",
				__func__, label, info->gpio[type].gpio);
		}
	}
}

static int ov16825_power_off(struct ov16825_info *info)
{
	struct ov16825_power_rail *pw = &info->regulators;
	int err = 0;

	if (!info->power_on)
		goto ov16825_poweroff_skip;

	if (info->pdata && info->pdata->power_off)
		err = info->pdata->power_off(pw, info->pdata->gpio, info->pdata->gpio_count);
	/* if customized design handles the power off process specifically,
	* return is bigger than 0 (normally 1), otherwise 0 or error num.
	*/
	if (err > 0) {
		info->power_on = false;
		return 0;
	}

	if (!err) {
		ov16825_gpio_pwrdn(info, 1);
		ov16825_gpio_shutdn(info, 1);
		if (pw->avdd)
			WARN_ON(IS_ERR_VALUE(
				err = regulator_disable(pw->avdd)));
		if (pw->dvdd)
			WARN_ON(IS_ERR_VALUE(
				err |= regulator_disable(pw->dvdd)));
		if (pw->dovdd)
			WARN_ON(IS_ERR_VALUE(
				err |= regulator_disable(pw->dovdd)));
	}

	if (!err)
		info->power_on = false;

ov16825_poweroff_skip:
	return err;
}

static int ov16825_power_on(struct ov16825_info *info, bool standby)
{
	struct ov16825_power_rail *pw = &info->regulators;
	int err = 0;

	if (info->power_on)
		goto ov16825_poweron_skip;

	if (info->pdata && info->pdata->power_on)
		err = info->pdata->power_on(pw, info->pdata->gpio, info->pdata->gpio_count);
	/* if customized design handles the power on process specifically,
	* return is bigger than 0 (normally 1), otherwise 0 or error num.
	*/
	if (!err) {
		if (pw->dvdd)
			WARN_ON(IS_ERR_VALUE(
				err = regulator_enable(pw->dvdd)));
		if (pw->dovdd)
			WARN_ON(IS_ERR_VALUE(
				err |= regulator_enable(pw->dovdd)));
		if (pw->avdd)
			WARN_ON(IS_ERR_VALUE(
				err |= regulator_enable(pw->avdd)));
		ov16825_gpio_shutdn(info, 0);
		ov16825_gpio_pwrdn(info, 0); /* PWRDN off to access I2C */
	}
	if (IS_ERR_VALUE(err))
		return err;
	info->power_on = true;
	err = 0;

ov16825_poweron_skip:
	return err;
}

static int ov16825_pm_wr(struct ov16825_info *info, int pwr)
{
	int err = 0;

	if ((info->pdata->cfg & (NVC_CFG_OFF2STDBY | NVC_CFG_BOOT_INIT)) &&
			(pwr == NVC_PWR_OFF ||
			 pwr == NVC_PWR_STDBY_OFF))
		pwr = NVC_PWR_STDBY;
	if (pwr == info->pwr_dev)
		return 0;

	switch (pwr) {
	case NVC_PWR_OFF_FORCE:
	case NVC_PWR_OFF:
	case NVC_PWR_STDBY_OFF:
		err = ov16825_power_off(info);
		info->mode_valid = false;
		info->bin_en = 0;
		break;

	case NVC_PWR_STDBY:
		err = ov16825_power_on(info, true);
		break;

	case NVC_PWR_COMM:
	case NVC_PWR_ON:
		err = ov16825_power_on(info, false);
		ov16825_i2c_wr8(info, 0x0103, 0x01);
		mdelay(20);
		break;

	default:
		err = -EINVAL;
		break;
	}

	if (err < 0) {
		dev_err(&info->i2c_client->dev, "%s err %d\n", __func__, err);
		pwr = NVC_PWR_ERR;
	}
	info->pwr_dev = pwr;
	dev_dbg(&info->i2c_client->dev, "%s pwr_dev=%d\n",
		__func__, info->pwr_dev);
	if (err > 0)
		return 0;

	return err;
}

static int ov16825_pm_wr_s(struct ov16825_info *info, int pwr)
{
	int err1 = 0;
	int err2 = 0;

	if ((info->s_mode == NVC_SYNC_OFF) ||
			(info->s_mode == NVC_SYNC_MASTER) ||
			(info->s_mode == NVC_SYNC_STEREO))
		err1 = ov16825_pm_wr(info, pwr);
	if ((info->s_mode == NVC_SYNC_SLAVE) ||
			(info->s_mode == NVC_SYNC_STEREO))
		err2 = ov16825_pm_wr(info->s_info, pwr);
	return err1 | err2;
}

static int ov16825_pm_api_wr(struct ov16825_info *info, int pwr)
{
	int err = 0;

	if (!pwr || (pwr > NVC_PWR_ON))
		return 0;

	if (pwr > info->pwr_dev)
		err = ov16825_pm_wr_s(info, pwr);
	if (!err)
		info->pwr_api = pwr;
	else
		info->pwr_api = NVC_PWR_ERR;
	if (info->pdata->cfg & NVC_CFG_NOERR)
		return 0;

	return err;
}

static int ov16825_pm_dev_wr(struct ov16825_info *info, int pwr)
{
	if (info->mode_enable)
		pwr = NVC_PWR_ON;
	if (pwr < info->pwr_api)
		pwr = info->pwr_api;
	return ov16825_pm_wr(info, pwr);
}

static void ov16825_pm_exit(struct ov16825_info *info)
{
	struct ov16825_power_rail *pw = &info->regulators;

	ov16825_pm_wr(info, NVC_PWR_OFF_FORCE);

	if (pw->avdd)
		regulator_put(pw->avdd);
	if (pw->dvdd)
		regulator_put(pw->dvdd);
	if (pw->dovdd)
		regulator_put(pw->dovdd);
	pw->avdd = NULL;
	pw->dvdd = NULL;
	pw->dovdd = NULL;

	ov16825_gpio_exit(info);
}

static int ov16825_regulator_get(
	struct ov16825_info *info, struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = regulator_get(&info->i2c_client->dev, vreg_name);
	if (IS_ERR(reg)) {
		dev_err(&info->i2c_client->dev, "%s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&info->i2c_client->dev, "%s: %s\n",
			__func__, vreg_name);

	*vreg = reg;
	return err;
}

static void ov16825_pm_init(struct ov16825_info *info)
{
	struct ov16825_power_rail *pw = &info->regulators;

	ov16825_gpio_init(info);

	ov16825_regulator_get(info, &pw->avdd, "avdd");
	ov16825_regulator_get(info, &pw->dvdd, "dvdd");
	ov16825_regulator_get(info, &pw->dovdd, "dovdd");
	info->power_on = false;
}

static int ov16825_reset(struct ov16825_info *info, int level)
{
	int err;

	if (level == NVC_RESET_SOFT) {
		err = ov16825_pm_wr(info, NVC_PWR_COMM);
		err |= ov16825_i2c_wr8(info, 0x0103, 0x01); /* SW reset */
	} else
		err = ov16825_pm_wr(info, NVC_PWR_OFF_FORCE);
	err |= ov16825_pm_wr(info, info->pwr_api);
	return err;
}

static int ov16825_dev_id(struct ov16825_info *info)
{
	u16 val = 0;
	unsigned i;
	int err;

	if (info->sdata.sensor_id_minor)
		return 0;

	ov16825_pm_dev_wr(info, NVC_PWR_COMM);
	msleep(10);
	err = ov16825_i2c_rd16(info, 0x300a, &val);
	if (!err) {
		dev_info(&info->i2c_client->dev, "%s found devId: %x\n",
			__func__, val);
		info->sdata.sensor_id_minor = 0;
		for (i = 0; i < ARRAY_SIZE(ov16825_ids); i++) {
			if (val == ov16825_ids[i]) {
				info->sdata.sensor_id_minor = val;
				break;
			}
		}
		if (!info->sdata.sensor_id_minor) {
			err = -ENODEV;
			dev_dbg(&info->i2c_client->dev, "%s No devId match\n",
				__func__);
		}
	}
	ov16825_pm_dev_wr(info, NVC_PWR_OFF);
	return err;
}

static int ov16825_mode_able(struct ov16825_info *info, bool mode_enable)
{
	u8 val;
	int err;

	if (mode_enable)
		val = 0x01;
	else
		val = 0x00;
	err = ov16825_i2c_wr8(info, 0x0100, val);
	if (!err) {
		info->mode_enable = mode_enable;
		dev_dbg(&info->i2c_client->dev, "%s streaming=%x\n",
			__func__, info->mode_enable);
		if (!mode_enable)
			ov16825_pm_dev_wr(info, NVC_PWR_STDBY);
	}
	return err;
}

static int ov16825_mode_rd(struct ov16825_info *info,
			s32 res_x,
			s32 res_y,
			u32 *index)
{
	int i;

	if (!res_x && !res_y) {
		*index = info->cap->preferred_mode_index;
		return 0;
	}

	for (i = 0; i < OV16825_NUM_MODES; i++) {
		if ((res_x == ov16825_mode_table[i]->sensor_mode.res_x) &&
		   (res_y == ov16825_mode_table[i]->sensor_mode.res_y)) {
			break;
		}
	}

	if (i == OV16825_NUM_MODES) {
		dev_err(&info->i2c_client->dev,
			"%s invalid resolution: %dx%d\n",
			__func__, res_x, res_y);
		return -EINVAL;
	}

	*index = i;
	return 0;
}

static int ov16825_mode_wr_full(struct ov16825_info *info, u32 mode_index)
{
	int err;

	ov16825_pm_dev_wr(info, NVC_PWR_ON);
	ov16825_bin_wr(info, 0);
	err = ov16825_i2c_wr_table(info,
				ov16825_mode_table[mode_index]->p_mode_i2c);
	if (!err) {
		info->mode_index = mode_index;
		info->mode_valid = true;
	} else {
		info->mode_valid = false;
	}
	return err;
}

static int ov16825_mode_wr(struct ov16825_info *info,
			struct nvc_imager_bayer *mode)
{
	u32 mode_index;
	int err;

#ifdef OV16825_REGISTER_DUMP
	int i;
	__u8 buf;
	__u16 bufarray[2][6];
	int col;
#endif
	err = ov16825_mode_rd(info, mode->res_x, mode->res_y, &mode_index);
	if (err < 0)
		return err;


	printk("ov %s mode->res_x %d mode->res_y %d mode_index %d\n",
					__func__,
					mode->res_x,
					mode->res_y,
					mode_index
					);


	if (!mode->res_x && !mode->res_y) {
		if (mode->frame_length || mode->coarse_time || mode->gain) {
			/* write exposure only */
			err = ov16825_exposure_wr(info, mode);
			return err;
		} else {
			/* turn off streaming */
			err = ov16825_mode_able(info, false);
			return err;
		}
	}

	if (!info->mode_valid || (info->mode_index != mode_index))
		err = ov16825_mode_wr_full(info, mode_index);
	else
		dev_dbg(&info->i2c_client->dev, "%s short mode\n", __func__);


	err |= ov16825_exposure_wr(info, mode);
	if (err < 0) {
		info->mode_valid = false;
		goto ov16825_mode_wr_err;
	}


	err = ov16825_mode_able(info, true);
	if (err < 0)
		goto ov16825_mode_wr_err;
/*
	if((mode->res_x == 2304) && (mode->res_y == 1728))
		mdelay(50);
*/
	return 0;


ov16825_mode_wr_err:
	if (!info->mode_enable)
		ov16825_pm_dev_wr(info, NVC_PWR_OFF);
	return err;
}


static int ov16825_param_rd(struct ov16825_info *info, unsigned long arg)
{
	struct nvc_param params;
	struct ov16825_reg *p_i2c_table;
	const void *data_ptr;
	u32 data_size = 0;
	u32 u32val;
	int err;

	if (copy_from_user(&params,
			(const void __user *)arg,
			sizeof(struct nvc_param))) {
		dev_err(&info->i2c_client->dev,
			"%s copy_from_user err line %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (info->s_mode == NVC_SYNC_SLAVE)
		info = info->s_info;
	switch (params.param) {
	case NVC_PARAM_GAIN:

		ov16825_pm_dev_wr(info, NVC_PWR_COMM);
		err = ov16825_gain_rd(info, &u32val);
		ov16825_pm_dev_wr(info, NVC_PWR_OFF);
		dev_dbg(&info->i2c_client->dev, "%s GAIN: %u err: %d\n",
			__func__, u32val, err);
		if (err)
			return err;

		data_ptr = &u32val;
		data_size = sizeof(u32val);
		break;

	case NVC_PARAM_STEREO_CAP:
		if (info->s_info != NULL)
			err = 0;
		else
			err = -ENODEV;
		dev_dbg(&info->i2c_client->dev, "%s STEREO_CAP: %d\n",
			__func__, err);
		data_ptr = &err;
		data_size = sizeof(err);
		break;

	case NVC_PARAM_STEREO:
		dev_dbg(&info->i2c_client->dev, "%s STEREO: %d\n",
			__func__, info->s_mode);
		data_ptr = &info->s_mode;
		data_size = sizeof(info->s_mode);
		break;

	case NVC_PARAM_STS:
		err = ov16825_dev_id(info);
		dev_dbg(&info->i2c_client->dev, "%s STS: %d\n",
			__func__, err);
		data_ptr = &err;
		data_size = sizeof(err);
		break;

	case NVC_PARAM_DEV_ID:
		if (!info->sdata.sensor_id_minor)
			ov16825_dev_id(info);
		data_ptr = &info->sdata.sensor_id;
		data_size = sizeof(info->sdata.sensor_id) * 2;
		dev_dbg(&info->i2c_client->dev, "%s DEV_ID: %x-%x\n",
			__func__, info->sdata.sensor_id,
			info->sdata.sensor_id_minor);
		break;

	case NVC_PARAM_SENSOR_TYPE:
		data_ptr = &info->sdata.sensor_type;
		data_size = sizeof(info->sdata.sensor_type);
		dev_dbg(&info->i2c_client->dev, "%s SENSOR_TYPE: %d\n",
			__func__, info->sdata.sensor_type);
		break;

	case NVC_PARAM_FOCAL_LEN:
		data_ptr = &info->sdata.focal_len;
		data_size = sizeof(info->sdata.focal_len);
		dev_dbg(&info->i2c_client->dev, "%s FOCAL_LEN: %u\n",
			__func__, info->sdata.focal_len);
		break;

	case NVC_PARAM_MAX_APERTURE:
		data_ptr = &info->sdata.max_aperture;
		data_size = sizeof(info->sdata.max_aperture);
		dev_dbg(&info->i2c_client->dev, "%s MAX_APERTURE: %u\n",
			__func__, info->sdata.max_aperture);
		break;

	case NVC_PARAM_FNUMBER:
		data_ptr = &info->sdata.fnumber;
		data_size = sizeof(info->sdata.fnumber);
		dev_dbg(&info->i2c_client->dev, "%s FNUMBER: %u\n",
			__func__, info->sdata.fnumber);
		break;

	case NVC_PARAM_VIEW_ANGLE_H:
		data_ptr = &info->sdata.view_angle_h;
		data_size = sizeof(info->sdata.view_angle_h);
		dev_dbg(&info->i2c_client->dev, "%s VIEW_ANGLE_H: %u\n",
			__func__, info->sdata.view_angle_h);
		break;

	case NVC_PARAM_VIEW_ANGLE_V:
		data_ptr = &info->sdata.view_angle_v;
		data_size = sizeof(info->sdata.view_angle_v);
		dev_dbg(&info->i2c_client->dev, "%s VIEW_ANGLE_V: %u\n",
			__func__, info->sdata.view_angle_v);
		break;

	case NVC_PARAM_I2C:
		dev_dbg(&info->i2c_client->dev, "%s I2C\n", __func__);
		if (params.sizeofvalue > OV16825_I2C_TABLE_MAX_ENTRIES) {
			pr_err("%s: requested size too large\n", __func__);
			return -EINVAL;
		}
		p_i2c_table = kzalloc(sizeof(params.sizeofvalue), GFP_KERNEL);
		if (p_i2c_table == NULL) {
			pr_err("%s: kzalloc error\n", __func__);
			return -ENOMEM;
		}

		if (copy_from_user(p_i2c_table,
				(const void __user *)params.p_value,
				params.sizeofvalue)) {
			dev_err(&info->i2c_client->dev,
				"%s copy_from_user err line %d\n",
				__func__, __LINE__);
			kfree(p_i2c_table);
			return -EINVAL;
		}

		ov16825_pm_dev_wr(info, NVC_PWR_COMM);
		err = ov16825_i2c_rd_table(info, p_i2c_table);
		ov16825_pm_dev_wr(info, NVC_PWR_OFF);
		if (copy_to_user((void __user *)params.p_value,
				 p_i2c_table,
				 params.sizeofvalue)) {
			dev_err(&info->i2c_client->dev,
				"%s copy_to_user err line %d\n",
				__func__, __LINE__);
			err = -EINVAL;
		}
		kfree(p_i2c_table);
		return err;

	default:
		dev_err(&info->i2c_client->dev,
			"%s unsupported parameter: %d\n",
			__func__, params.param);
		return -EINVAL;
	}

	if (params.sizeofvalue < data_size) {
		dev_err(&info->i2c_client->dev,
			"%s data size mismatch %d != %d Param: %d\n",
			__func__, params.sizeofvalue, data_size, params.param);
		return -EINVAL;
	}

	if (copy_to_user((void __user *)params.p_value,
			 data_ptr,
			 data_size)) {
		dev_err(&info->i2c_client->dev,
			"%s copy_to_user err line %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

static int ov16825_param_wr_s(struct ov16825_info *info,
			struct nvc_param *params,
			u32 u32val)
{
	struct ov16825_reg *p_i2c_table;
	u8 u8val;
	int err;

	u8val = (u8)u32val;
	switch (params->param) {
	case NVC_PARAM_GAIN:
		dev_dbg(&info->i2c_client->dev, "%s GAIN: %u\n",
			__func__, u32val);
		ov16825_pm_dev_wr(info, NVC_PWR_COMM);
		err = ov16825_gain_wr(info, u32val);
		ov16825_pm_dev_wr(info, NVC_PWR_STDBY);
		return err;

	case NVC_PARAM_RESET:
		err = ov16825_reset(info, u32val);
		dev_dbg(&info->i2c_client->dev, "%s RESET=%d err=%d\n",
			__func__, u32val, err);
		return err;

	case NVC_PARAM_TESTMODE:
		dev_dbg(&info->i2c_client->dev, "%s TESTMODE: %u\n",
			__func__, (unsigned)u8val);
		if (u8val)
			u32val = info->test_pattern;
		else
			u32val = 0;
		ov16825_pm_dev_wr(info, NVC_PWR_ON);
		err = ov16825_test_pattern_wr(info, u32val);
		if (!u8val)
			ov16825_pm_dev_wr(info, NVC_PWR_OFF);
		return err;

	case NVC_PARAM_TEST_PATTERN:
		dev_dbg(&info->i2c_client->dev, "%s TEST_PATTERN: %d\n",
			__func__, u32val);
		info->test_pattern = u32val;
		return 0;

	case NVC_PARAM_SELF_TEST:
		err = ov16825_dev_id(info);
		dev_dbg(&info->i2c_client->dev, "%s SELF_TEST: %d\n",
			__func__, err);
		return err;

	case NVC_PARAM_I2C:
		dev_dbg(&info->i2c_client->dev, "%s I2C\n", __func__);
		if (params->sizeofvalue > OV16825_I2C_TABLE_MAX_ENTRIES) {
			pr_err("%s: requested size too large\n", __func__);
			return -EINVAL;
		}
		p_i2c_table = kzalloc(sizeof(params->sizeofvalue), GFP_KERNEL);
		if (p_i2c_table == NULL) {
			dev_err(&info->i2c_client->dev,
				"%s kzalloc err line %d\n",
				__func__, __LINE__);
			return -ENOMEM;
		}

		if (copy_from_user(p_i2c_table,
				(const void __user *)params->p_value,
				params->sizeofvalue)) {
			dev_err(&info->i2c_client->dev,
				"%s copy_from_user err line %d\n",
				__func__, __LINE__);
			kfree(p_i2c_table);
			return -EFAULT;
		}

		ov16825_pm_dev_wr(info, NVC_PWR_ON);
		err = ov16825_i2c_wr_table(info, p_i2c_table);
		kfree(p_i2c_table);
		return err;

	default:
		dev_err(&info->i2c_client->dev,
			"%s unsupported parameter: %d\n",
			__func__, params->param);
		return -EINVAL;
	}
}

static int ov16825_param_wr(struct ov16825_info *info, unsigned long arg)
{
	struct nvc_param params;
	u8 u8val;
	u32 u32val;
	int err = 0;

	if (copy_from_user(&params, (const void __user *)arg,
			sizeof(struct nvc_param))) {
		dev_err(&info->i2c_client->dev,
			"%s copy_from_user err line %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (copy_from_user(&u32val, (const void __user *)params.p_value,
			sizeof(u32val))) {
		dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	u8val = (u8)u32val;
	/* parameters independent of sync mode */
	switch (params.param) {
	case NVC_PARAM_STEREO:
		dev_dbg(&info->i2c_client->dev, "%s STEREO: %d\n",
			__func__, u8val);
		if (u8val == info->s_mode)
			return 0;

		switch (u8val) {
		case NVC_SYNC_OFF:
			info->s_mode = u8val;
			if (info->s_info != NULL) {
				info->s_info->s_mode = u8val;
				ov16825_pm_wr(info->s_info, NVC_PWR_OFF);
			}
			break;

		case NVC_SYNC_MASTER:
			info->s_mode = u8val;
			if (info->s_info != NULL)
				info->s_info->s_mode = u8val;
			break;

		case NVC_SYNC_SLAVE:
			if (info->s_info != NULL) {
				/* sync power */
				info->s_info->pwr_api = info->pwr_api;
				err = ov16825_pm_wr(info->s_info,
						info->pwr_dev);
				if (!err) {
					info->s_mode = u8val;
					info->s_info->s_mode = u8val;
				} else {
					if (info->s_mode != NVC_SYNC_STEREO)
						ov16825_pm_wr(info->s_info,
							NVC_PWR_OFF);
					err = -EIO;
				}
			}
			break;

		case NVC_SYNC_STEREO:
			if (info->s_info != NULL) {
				/* sync power */
				info->s_info->pwr_api = info->pwr_api;
				err = ov16825_pm_wr(info->s_info,
						info->pwr_dev);
				if (!err) {
					info->s_mode = u8val;
					info->s_info->s_mode = u8val;
				} else {
					if (info->s_mode != NVC_SYNC_SLAVE)
						ov16825_pm_wr(info->s_info,
								NVC_PWR_OFF);
					err = -EIO;
				}
			}
			break;

		default:
			err = -EINVAL;
		}
		if (info->pdata->cfg & NVC_CFG_NOERR)
			return 0;

		return err;

	case NVC_PARAM_GROUP_HOLD:
	{
		struct nvc_imager_ae ae;
		dev_dbg(&info->i2c_client->dev, "%s GROUP_HOLD\n",
			__func__);

		if (copy_from_user(&ae, (const void __user *)params.p_value,
				sizeof(struct nvc_imager_ae))) {
			dev_err(&info->i2c_client->dev,
				"%s %d copy_from_user err\n",
				__func__, __LINE__);
			return -EFAULT;
		}
		ov16825_pm_dev_wr(info, NVC_PWR_COMM);
		err = ov16825_group_hold_wr(info, &ae);
		ov16825_pm_dev_wr(info, NVC_PWR_STDBY);
		return err;
	}

	default:
	/* parameters dependent on sync mode */
		switch (info->s_mode) {
		case NVC_SYNC_OFF:
		case NVC_SYNC_MASTER:
			err = ov16825_param_wr_s(info, &params, u32val);
			return err;

		case NVC_SYNC_SLAVE:
			err = ov16825_param_wr_s(info->s_info,
						 &params,
						 u32val);
			return err;

		case NVC_SYNC_STEREO:
			err = ov16825_param_wr_s(info, &params, u32val);
			if (!(info->pdata->cfg & NVC_CFG_SYNC_I2C_MUX))
				err |= ov16825_param_wr_s(info->s_info,
							 &params,
							 u32val);
			return err;

		default:
			dev_err(&info->i2c_client->dev, "%s %d internal err\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	}
}

static int ov16825_eeprom_release(
	struct ov16825_info *info)
{
	int i;

	for (i = 0; i < OV16825_EEPROM_COUNT; i++) {
		if (info->eeprom[i].i2c_client != NULL) {
			i2c_unregister_device(info->eeprom[i].i2c_client);
			info->eeprom[i].i2c_client = NULL;
		}
	}

	return 0;
}

static int ov16825_eeprom_init(
	struct ov16825_info *info)
{
	const char *dev_name = "eeprom_GT24C16";
	static struct regmap_config eeprom_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int i;
	int err;

	for (i = 0; i < OV16825_EEPROM_COUNT; i++) {
		info->eeprom[i].adap = i2c_get_adapter(
				info->i2c_client->adapter->nr);
		memset(&info->eeprom[i].brd, 0, sizeof(info->eeprom[i].brd));
		strncpy(info->eeprom[i].brd.type, dev_name,
				sizeof(info->eeprom[i].brd.type));
		info->eeprom[i].brd.addr = eeprom_dev_addr[i];
		info->eeprom[i].i2c_client = i2c_new_device(
				info->eeprom[i].adap, &info->eeprom[i].brd);

		info->eeprom[i].regmap = devm_regmap_init_i2c(
			info->eeprom[i].i2c_client, &eeprom_regmap_config);
		if (IS_ERR(info->eeprom[i].regmap)) {
			err = PTR_ERR(info->eeprom[i].regmap);
			ov16825_eeprom_release(info);
			return err;
		}

		info->eeprom[i].block_nr = 1 + i;
		memset(info->eeprom[i].buf,
			0,
			OV16825_EEPROM_SIZE);
		info->eeprom[i].size = OV16825_EEPROM_SIZE;
	}

	return 0;
}

static int ov16825_eeprom_rd_block_nr(
	struct ov16825_info *info,
	int block_nr)
{
	struct regmap *regmap = NULL;
	int i = 0;
	int reg = 0;
	u8 *buf = NULL;
	int length = 0;

	for (i = 0; i < OV16825_EEPROM_COUNT; i++) {
		if(info->eeprom[i].block_nr == block_nr) {
			regmap = info->eeprom[i].regmap;
			buf = info->eeprom[i].buf;
			length = info->eeprom[i].size;
		}
	}

	if (regmap == NULL)
		return -EFAULT;

	return regmap_raw_read(regmap,
		reg,
		buf,
		length);
}

static int ov16825_eeprom_get_buf(
	struct ov16825_info *info,
	int block_nr,
	u8 **ppbuf,
	int *psize)
{
	int i = 0;

	*ppbuf = NULL;
	*psize = 0;

	for (i = 0; i < OV16825_EEPROM_COUNT; i++) {
		if(info->eeprom[i].block_nr == block_nr) {
			*ppbuf = info->eeprom[i].buf;
			*psize = info->eeprom[i].size;
			return 0;
		}
	}
	return -EFAULT;
}

static int ov16825_eeprom_rd_cal(
	struct ov16825_info *info,
	void *user_dest)
{
	int err = 0;
	int i = 0;
	u8 *buf = NULL;
	int size = 0;
	int user_pointer = 0;
	const int block_nr_start = 2;
	const int block_nr_end = 5;

	for (i = block_nr_start; i <= block_nr_end; i++) {
		err = ov16825_eeprom_rd_block_nr(
			info,
			i);
		if (err) {
			printk("%s read block err, nr %d err %x\n", __func__,
				i,
				err);
			return err;
		}
	}

	for (i = block_nr_start; i <= block_nr_end; i++) {
		err = ov16825_eeprom_get_buf(
			info,
			i,
			&buf,
			&size);
		if (err) {
			printk("%s read block err, nr %d err %x\n", __func__,
				i,
				err);
			return err;
		}

		if (copy_to_user(
			(void __user *)(user_dest + user_pointer),
			buf,
			size)) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to copy status to user\n",
				__func__);
			return -EFAULT;
		}
		user_pointer += size;
	}

	printk("%s bytes copied to user %d\n", __func__,
		user_pointer);

	return err;
}
#if 0
static int ov16825_eeprom_rd_fuseid(
	struct ov16825_info *info,
	int *plength,
	u8 **ppbuf)
{
	int err = 0;
	int i = 0;
	u8 *buf = NULL;
	int size = 0;
	const int block_nr_fuseid = 1;
	//struct ov16825_eeprom_block1 *eb1 = NULL;

	*plength = 0;
	*ppbuf = NULL;

	err = ov16825_eeprom_rd_block_nr(
		info,
		block_nr_fuseid);
	if (err) {
		printk("%s read block err, nr %d err %x\n", __func__,
			1,
			err);
		return err;
	}

	err = ov16825_eeprom_get_buf(
		info,
		block_nr_fuseid,
		&buf,
		&size);
	if (err) {
		printk("%s read block err, nr %d err %x\n", __func__,
			i,
			err);
		return err;
	}
#if 0
	eb1 = (struct ov16825_eeprom_block1 *)buf;
	if (size < OV16825_EEPROM_BLOCK1_FUSEID_LEN ||
		eb1->flag == OV16825_EEPROM_BLOCK1_FLAG_INVALID)
		return -EFAULT;
#endif

	/* *plength = OV16825_EEPROM_BLOCK1_FUSEID_LEN; */
	*plength = OV16825_EEPROM_SIZE;
	*ppbuf = buf;

	return err;
}

/* use eeprom */
static int ov16825_get_fuse_id(struct ov16825_info *info)
{
	int ret;
	int length = 0;
	u8 *buf = NULL;
	int i = 0;

	if (info->fuse_id.size)
		return 0;

	ret = ov16825_eeprom_rd_fuseid(
		info,
		&length,
		&buf);
	if (ret) {
		printk("%s error read fuseid %d\n", __func__, ret);
		return ret;
	}

	info->fuse_id.size = length;
	for (i = 0; i < length; i++) {
		info->fuse_id.data[i] = buf[i];
		printk("%s fuse id d[%d] = %02x\n", __func__, i, buf[i]);
	}

	return ret;
}
#else
static int ov16825_get_fuse_id(struct ov16825_info *info)
{
	int ret;
	int i = 0;
	#define SIZE_FUSEID		9
	u8 bytes[SIZE_FUSEID] = {0, };

	if (info->fuse_id.size)
		return 0;

	ret = ov16825_NVM_read_bytes(info,
		0x7080,
		bytes,
		SIZE_FUSEID);
	if (ret)
		return ret;

	info->fuse_id.size = SIZE_FUSEID;
	for (i = 0; i < info->fuse_id.size; i++) {
		info->fuse_id.data[i] = bytes[i];
		printk("%s fuse id d[%d] = %02x\n", __func__, i, bytes[i]);
	}

	return ret;
}
#endif

static long ov16825_ioctl(struct file *file,
			 unsigned int cmd,
			 unsigned long arg)
{
	struct ov16825_info *info = file->private_data;
	struct nvc_imager_bayer mode;
	struct nvc_imager_mode_list mode_list;
	struct nvc_imager_mode mode_table[OV16825_NUM_MODES];
	struct nvc_imager_dnvc dnvc;
	const void *data_ptr;
	s32 num_modes;
	u32 i;
	int pwr;
	int err = 0;

	switch (cmd) {
	case NVC_IOCTL_FUSE_ID:
		err = ov16825_get_fuse_id(info);
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
	case NVC_IOCTL_GET_EEPROM_DATA:
		err = ov16825_eeprom_rd_cal(
			info,
			(void *)arg);
		if (err) {
			pr_err("%s read eeprom data err %d\n", __func__, err);
		}
		return err;

	case NVC_IOCTL_PARAM_WR:
		err = ov16825_param_wr(info, arg);
		return err;

	case NVC_IOCTL_PARAM_RD:
		err = ov16825_param_rd(info, arg);
		return err;

	case NVC_IOCTL_DYNAMIC_RD:
		if (copy_from_user(&dnvc, (const void __user *)arg,
				sizeof(struct nvc_imager_dnvc))) {
			dev_err(&info->i2c_client->dev,
				"%s copy_from_user err line %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		dev_dbg(&info->i2c_client->dev, "%s DYNAMIC_RD x=%d y=%d\n",
			__func__, dnvc.res_x, dnvc.res_y);
		err = ov16825_mode_rd(info, dnvc.res_x, dnvc.res_y, &i);
		if (err)
			return -EINVAL;

		if (dnvc.p_mode) {
			if (copy_to_user((void __user *)dnvc.p_mode,
					 &ov16825_mode_table[i]->sensor_mode,
					 sizeof(struct nvc_imager_mode))) {
				dev_err(&info->i2c_client->dev,
					"%s copy_to_user err line %d\n",
					__func__, __LINE__);
				return -EFAULT;
			}
		}

		if (dnvc.p_dnvc) {
			if (copy_to_user((void __user *)dnvc.p_dnvc,
				    &ov16825_mode_table[i]->sensor_dnvc,
				    sizeof(struct nvc_imager_dynamic_nvc))) {
				dev_err(&info->i2c_client->dev,
					"%s copy_to_user err line %d\n",
					__func__, __LINE__);
				return -EFAULT;
			}
		}

		return 0;

	case NVC_IOCTL_MODE_WR:
		if (copy_from_user(&mode, (const void __user *)arg,
				sizeof(struct nvc_imager_bayer))) {
			dev_err(&info->i2c_client->dev,
				"%s copy_from_user err line %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		dev_dbg(&info->i2c_client->dev, "%s MODE_WR x=%d y=%d ",
			__func__, mode.res_x, mode.res_y);
		dev_dbg(&info->i2c_client->dev, "coarse=%u frame=%u gain=%u\n",
			mode.coarse_time, mode.frame_length, mode.gain);
		err = ov16825_mode_wr(info, &mode);
		return err;

	case NVC_IOCTL_MODE_RD:
		/*
		 * Return a list of modes that sensor bayer supports.
		 * If called with a NULL ptr to pModes,
		 * then it just returns the count.
		 */
		dev_dbg(&info->i2c_client->dev, "%s MODE_RD n=%d\n",
			__func__, OV16825_NUM_MODES);
		if (copy_from_user(&mode_list, (const void __user *)arg,
				sizeof(struct nvc_imager_mode_list))) {
			dev_err(&info->i2c_client->dev,
				"%s copy_from_user err line %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		num_modes = OV16825_NUM_MODES;
		if (mode_list.p_num_mode != NULL) {
			if (copy_to_user((void __user *)mode_list.p_num_mode,
					 &num_modes, sizeof(num_modes))) {
				dev_err(&info->i2c_client->dev,
					"%s copy_to_user err line %d\n",
					__func__, __LINE__);
				return -EFAULT;
			}
		}

		if (mode_list.p_modes != NULL) {
			for (i = 0; i < OV16825_NUM_MODES; i++) {
				mode_table[i] =
					ov16825_mode_table[i]->sensor_mode;
			}
			if (copy_to_user((void __user *)mode_list.p_modes,
					 (const void *)&mode_table,
					 sizeof(mode_table))) {
				dev_err(&info->i2c_client->dev,
					"%s copy_to_user err line %d\n",
					__func__, __LINE__);
				return -EFAULT;
			}
		}

		return 0;

	case NVC_IOCTL_PWR_WR:
		/* This is a Guaranteed Level of Service (GLOS) call */
		pwr = (int)arg * 2;
		dev_dbg(&info->i2c_client->dev, "%s PWR_WR: %d\n",
			__func__, pwr);
		err = ov16825_pm_api_wr(info, pwr);
		return err;

	case NVC_IOCTL_PWR_RD:
		if (info->s_mode == NVC_SYNC_SLAVE)
			pwr = info->s_info->pwr_api / 2;
		else
			pwr = info->pwr_api / 2;
		dev_dbg(&info->i2c_client->dev, "%s PWR_RD: %d\n",
			__func__, pwr);
		if (copy_to_user((void __user *)arg, (const void *)&pwr,
				 sizeof(pwr))) {
			dev_err(&info->i2c_client->dev,
					"%s copy_to_user err line %d\n",
					__func__, __LINE__);
			return -EFAULT;
		}

		return 0;

	case NVC_IOCTL_CAPS_RD:
		dev_dbg(&info->i2c_client->dev, "%s CAPS_RD n=%d\n",
			__func__, sizeof(ov16825_dflt_cap));
		data_ptr = info->cap;
		if (copy_to_user((void __user *)arg,
				 data_ptr,
				 sizeof(ov16825_dflt_cap))) {
			dev_err(&info->i2c_client->dev,
				"%s copy_to_user err line %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		return 0;

	case NVC_IOCTL_STATIC_RD:
		dev_dbg(&info->i2c_client->dev, "%s STATIC_RD n=%d\n",
			__func__, sizeof(struct nvc_imager_static_nvc));
		data_ptr = &info->sdata;
		if (copy_to_user((void __user *)arg,
				 data_ptr,
				 sizeof(struct nvc_imager_static_nvc))) {
			dev_err(&info->i2c_client->dev,
				"%s copy_to_user err line %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		return 0;

	default:
		dev_err(&info->i2c_client->dev, "%s unsupported ioctl: %x\n",
			__func__, cmd);
	}
	return -EINVAL;
}

static void ov16825_sdata_init(struct ov16825_info *info)
{
	memcpy(&info->sdata, &ov16825_dflt_sdata, sizeof(info->sdata));
	if (info->pdata->lens_focal_length)
		info->sdata.focal_len = info->pdata->lens_focal_length;
	if (info->pdata->lens_max_aperture)
		info->sdata.max_aperture = info->pdata->lens_max_aperture;
	if (info->pdata->lens_fnumber)
		info->sdata.fnumber = info->pdata->lens_fnumber;
	if (info->pdata->lens_view_angle_h)
		info->sdata.view_angle_h = info->pdata->lens_view_angle_h;
	if (info->pdata->lens_view_angle_v)
		info->sdata.view_angle_v = info->pdata->lens_view_angle_v;
}

static int ov16825_mclk_enable(struct ov16825_info *info)
{
	int err;

	dev_dbg(&info->i2c_client->dev, "%s: enable MCLK\n", __func__);

	err = clk_set_rate(info->mclk, 24000000);
	if (!err)
		err = clk_prepare_enable(info->mclk);

	return err;
}
static void ov16825_mclk_disable(struct ov16825_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int ov16825_open(struct inode *inode, struct file *file)
{
	struct ov16825_info *info = NULL;
	int err;
	struct miscdevice   *miscdev = file->private_data;
	info = container_of(miscdev, struct ov16825_info, miscdev);

	err = ov16825_mclk_enable(info);
	if (err)
		return err;

	if (atomic_xchg(&info->in_use, 1)){
		dev_err(&info->i2c_client->dev, "device is BUSY now %s\n", __func__);
		return -EBUSY;
	}
	file->private_data = info;
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);

	ov16825_edp_req(info, 0);

	return 0;
}

int ov16825_release(struct inode *inode, struct file *file)
{
	struct ov16825_info *info = file->private_data;

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	ov16825_edp_lowest(info);
	ov16825_mclk_disable(info);
	ov16825_pm_wr_s(info, NVC_PWR_OFF);
	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	if (info->s_info != NULL)
		WARN_ON(!atomic_xchg(&info->s_info->in_use, 0));
	return 0;
}

static const struct file_operations ov16825_fileops = {
	.owner = THIS_MODULE,
	.open = ov16825_open,
	.unlocked_ioctl = ov16825_ioctl,
	.release = ov16825_release,
};

static ssize_t ov16825_show_fuseid(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ov16825_info *info = i2c_get_clientdata(client);
	int len = 0;
	unsigned char buf_fuse[16] = {0, };/* max size for nvc_fuseid.data */
	int pos = 0;
	int i = 0;

	if (atomic_read(&info->in_use))
		return sprintf(buf, "device opened, close it first\n");

	ov16825_read_fuseid_test(info,
		&len,
		buf_fuse);

	if (len) {
		for (i = 0; i < len; i++) {
			pos += sprintf(buf + pos, "%02x", buf_fuse[i]);
		}
		pos += sprintf(buf + pos, "\n");
		return pos;
	} else
		return sprintf(buf, "Not implemented yet %p\n", info);
}

static DEVICE_ATTR(fuseid, S_IRUSR | S_IRGRP,
					ov16825_show_fuseid, NULL);

static struct attribute *ov16825_attributes[] = {
	&dev_attr_fuseid.attr,
	NULL
};

static const struct attribute_group ov16825_attr_group = {
	.attrs = ov16825_attributes,
};

static void ov16825_del(struct ov16825_info *info)
{
	ov16825_pm_exit(info);
	if ((info->s_mode == NVC_SYNC_SLAVE) ||
					(info->s_mode == NVC_SYNC_STEREO))
		ov16825_pm_exit(info->s_info);
}

static int ov16825_remove(struct i2c_client *client)
{
	struct ov16825_info *info = i2c_get_clientdata(client);

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	if (info->edpc)
		edp_unregister_client(info->edpc);
	sysfs_remove_group(&info->i2c_client->dev.kobj, &ov16825_attr_group);
	ov16825_eeprom_release(info);
	misc_deregister(&info->miscdev);
	ov16825_del(info);
	return 0;
}

static void ov16825_read_fuseid_test(
	struct ov16825_info *info,
	int *len,
	unsigned char *buf)
{
	int err = 0;
	int i = 0;
	void *p = NULL;
	#define SIZE_FUSEID		9
	u8 bytes[SIZE_FUSEID] = {0, };

	if (len && buf)
		*len = 0;

	p = ov16825_NVM_read_byte;

	err = ov16825_mclk_enable(info);
	if (err == 0) {
		ov16825_pm_dev_wr(info, NVC_PWR_COMM);

		err = ov16825_NVM_read_bytes(info,
			0x7080,
			bytes,
			SIZE_FUSEID);
		if (err == 0) {
			if (len)
				*len = SIZE_FUSEID;
			for (i = 0; i < SIZE_FUSEID; i++)
					if (buf) {
						buf[i] = bytes[i];
					}
		} else {
			printk("nvidia %s cannot read fuseid %x", __func__, err);
		}

		ov16825_pm_dev_wr(info, NVC_PWR_OFF);

		ov16825_mclk_disable(info);
	}
}


static struct i2c_driver ov16825_i2c_driver;
static ssize_t ov16825_caminfo_show(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%s","ov16825");
}
static DRIVER_ATTR(caminfo, 0644, ov16825_caminfo_show, NULL);

static int ov16825_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct ov16825_info *info;
	char dname[16];
	unsigned long clock_probe_rate;
	int err;


	dev_dbg(&client->dev, "%s\n", __func__);
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&client->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}

	info->i2c_client = client;
	if (client->dev.platform_data) {
		info->pdata = client->dev.platform_data;
	} else {
		info->pdata = &ov16825_dflt_pdata;
		dev_dbg(&client->dev,
			"%s No platform data.  Using defaults.\n", __func__);
	}
	if (info->pdata->cap)
		info->cap = info->pdata->cap;
	else
		info->cap = &ov16825_dflt_cap;
	i2c_set_clientdata(client, info);
	ov16825_pm_init(info);
	ov16825_sdata_init(info);
	if (info->pdata->cfg & (NVC_CFG_NODEV | NVC_CFG_BOOT_INIT)) {
		if (info->pdata->probe_clock) {
			if (info->cap->initial_clock_rate_khz)
				clock_probe_rate = info->cap->
							initial_clock_rate_khz;
			else
				clock_probe_rate = ov16825_dflt_cap.
							initial_clock_rate_khz;
			clock_probe_rate *= 1000;
			info->pdata->probe_clock(clock_probe_rate);
		}
		err = ov16825_dev_id(info);
		if (err < 0) {
			if (info->pdata->cfg & NVC_CFG_NODEV) {
				ov16825_del(info);
				if (info->pdata->probe_clock)
					info->pdata->probe_clock(0);
				return -ENODEV;
			} else {
				dev_err(&client->dev, "%s device not found\n",
					__func__);
			}
		} else {
			dev_dbg(&client->dev, "%s device found\n", __func__);
			if (info->pdata->cfg & NVC_CFG_BOOT_INIT)
				ov16825_mode_wr_full(info, info->cap->
						preferred_mode_index);
		}
		ov16825_pm_dev_wr(info, NVC_PWR_OFF);
		if (info->pdata->probe_clock)
			info->pdata->probe_clock(0);
	}
	if (info->pdata->dev_name != 0)
		strcpy(dname, info->pdata->dev_name);
	else
		strcpy(dname, "ov16825");
	if (info->pdata->num)
		snprintf(dname, sizeof(dname), "%s.%u",
			 dname, info->pdata->num);
	info->miscdev.name = dname;
	info->miscdev.fops = &ov16825_fileops;
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&info->miscdev)) {
		dev_err(&client->dev, "%s unable to register misc device %s\n",
			__func__, dname);
		ov16825_del(info);
		return -ENODEV;
	}


	info->mclk = devm_clk_get(&client->dev, info->pdata->clk_name);
	if (info->mclk == NULL) {
		misc_deregister(&info->miscdev);
		devm_kfree(&client->dev, info);
		return -ENODEV;
	}

#if 0
	printk("nvidia %s ---> do init i2c device check \n", __func__);
	err = ov16825_mclk_enable(info);
	if (err == 0) {
		err = ov16825_dev_id(info);

		if (err == 0)
			printk("nvidia %s do init i2c device val 0x%x \n", __func__, info->sdata.sensor_id_minor);
		else
			printk("nvidia %s cannot read device id \n", __func__);
		ov16825_mclk_disable(info);
	}
	printk("nvidia %s <--- do init i2c device check done \n", __func__);
#endif

	err = driver_create_file(&ov16825_i2c_driver.driver, &driver_attr_caminfo);
	if (err) {
		printk("failed to register caminfo attributes\n");
		err = 0;
	}
	ov16825_read_fuseid_test(info, NULL, NULL);

	err = ov16825_eeprom_init(info);
	if (err)
		printk("%s ov16825_eeprom_init failed\n", __func__);
#if 0
	{
		int len = 0;
		u8 *buf = NULL;
		int i = 0;

		ov16825_mclk_enable(info);
		ov16825_pm_api_wr(info, NVC_PWR_ON);
		ov16825_eeprom_rd_fuseid(
			info,
			&len,
			&buf);
		for (i = 0; i < len; i++)
			printk("%s %d %02x\n", __func__, i, buf[i]);
		ov16825_pm_api_wr(info, NVC_PWR_OFF);
		ov16825_mclk_disable(info);
	}
#endif
	err = sysfs_create_group(&client->dev.kobj,
		&ov16825_attr_group);
	if (err)
		printk("%s cannot create sysfs node\n", __func__);

	ov16825_edp_register(info);

	return 0;
}

static const struct i2c_device_id ov16825_id[] = {
	{ "ov16825", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov16825_id);

static struct i2c_driver ov16825_i2c_driver = {
	.driver = {
		.name = "ov16825",
		.owner = THIS_MODULE,
	},
	.id_table = ov16825_id,
	.probe = ov16825_probe,
	.remove = ov16825_remove,
};

module_i2c_driver(ov16825_i2c_driver);
MODULE_LICENSE("GPL v2");
