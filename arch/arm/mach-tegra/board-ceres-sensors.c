/*
 * arch/arm/mach-tegra/board-ceres-sensors.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/mpu.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/nct1008.h>
#include <linux/max17048_battery.h>
#include <linux/pid_thermal_gov.h>
#include <generated/mach-types.h>
#include <mach/edp.h>

#include <media/camera.h>
#include <media/imx091.h>
#include <media/imx132.h>
#include <media/ad5816.h>
#include <media/imx135.h>
#include <media/ov9772.h>
#include <media/as364x.h>
#include <media/ov5693.h>
#include <media/ov5640.h>
#include <media/ad5823.h>
#include <media/ar0833.h>
#include <media/dw9718.h>
#include <media/max77387.h>
#include <media/lm3565.h>

#include "cpu-tegra.h"
#include "devices.h"
#include "board-common.h"
#include "board-ceres.h"
#include "board-atlantis.h"
#include "tegra-board-id.h"
#include "board.h"
#include "battery-ini-model-data.h"

static struct board_info board_info;

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1938000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1912500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1887000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1861500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1836000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1275000, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1249500, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1224000, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1198500, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1173000, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1147500, 672000, NO_CAP, NO_CAP, 768000 } },
	{ { 1122000, 672000, NO_CAP, NO_CAP, 768000 } },
	{ { 1096500, 672000, NO_CAP, NO_CAP, 768000 } },
	{ { 1071000, 672000, NO_CAP, 384000, 768000 } },
	{ { 1045500, 672000, 499200, 384000, 768000 } },
	{ { 1020000, 595200, 499200, 384000, 768000 } },
	{ {  994500, 595200, 499200, 384000, 768000 } },
	{ {  969000, 595200, 499200, 384000, 768000 } },
	{ {  943500, 595200, 499200, 384000, 768000 } },
	{ {  918000, 595200, 499200, 384000, 768000 } },
	{ {  892500, 595200, 460800, 384000, 768000 } },
	{ {  867000, 556800, 460800, 384000, 768000 } },
	{ {  841500, 556800, 460800, 384000, 768000 } },
	{ {  816000, 556800, 460800, 384000, 652800 } },
	{ {  790500, 556800, 460800, 384000, 652800 } },
	{ {  765000, 556800, 460800, 204000, 652800 } },
	{ {  739500, 556800, 403200, 204000, 652800 } },
	{ {  714000, 499200, 403200, 204000, 652800 } },
	{ {  688500, 499200, 403200, 204000, 518400 } },
	{ {  663000, 499200, 403200, 204000, 518400 } },
	{ {  637500, 499200, 403200, 102000, 518400 } },
	{ {  612000, 499200, 345600, 102000, 518400 } },
	{ {  586500, 422400, 345600, 102000, 518400 } },
	{ {  561000, 422400, 345600, 102000, 518400 } },
	{ {  535500, 422400, 345600, 102000, 408000 } },
	{ {  510000, 422400, 345600, 102000, 408000 } },
	{ {  484500, 422400, 288000, 102000, 408000 } },
	{ {  459000, 307200, 288000, 102000, 408000 } },
	{ {  433500, 307200, 288000, 102000, 408000 } },
	{ {  408000, 307200, 288000, 102000, 408000 } },
	{ {  382500, 307200, 288000, 102000, 408000 } },
	{ {  357000, 307200, 288000, 102000, 408000 } },
	{ {  331500, 307200, 288000, 102000, 408000 } },
	{ {  306000, 307200, 288000, 102000, 408000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init ceres_throttle_init(void)
{
	if (of_machine_is_compatible("nvidia,ceres"))
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(ceres_throttle_init);

static struct nct1008_platform_data ceres_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.shutdown_ext_limit = 95, /* C */
	.shutdown_local_limit = 100, /* C */

	.passive_delay = 2000,

	.num_trips = 1,
	.trips = {
		{
			.cdev_type = "suspend_soctherm",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 5000,
		},
	},
};

struct max17048_platform_data max17048_pdata = {
	.model_data = &ceres_yoku_2000_ssv_3_1_max17048_battery,
	.tz_name = "battery-temp",
	.soc_error_max_value = 96,
};

static struct i2c_board_info __initdata max77660_fg_board_info[] = {
	{
		I2C_BOARD_INFO("max17048", 0x36),
		.platform_data  = &max17048_pdata,
	},
};

static struct i2c_board_info ceres_i2c0_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4C),
		.platform_data = &ceres_nct1008_pdata,
		.irq = -1,
	}
};

#define CERES_TEMP_ALERT_GPIO	TEGRA_GPIO_PO1
static int ceres_nct1008_init(void)
{
	int ret = 0;

	tegra_add_cdev_trips(ceres_nct1008_pdata.trips,
			     &ceres_nct1008_pdata.num_trips);

	/* FIXME: enable irq when throttling is supported */
	ceres_i2c0_nct1008_board_info[0].irq =
		gpio_to_irq(CERES_TEMP_ALERT_GPIO);

	ret = gpio_request(CERES_TEMP_ALERT_GPIO, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed\n", __func__);
		return ret;
	}

	ret = gpio_direction_input(CERES_TEMP_ALERT_GPIO);
	if (ret < 0) {
		pr_err("%s: set gpio to input failed\n", __func__);
		gpio_free(CERES_TEMP_ALERT_GPIO);
	}

	return ret;
}
static int ceres_ad5816_power_on(struct ad5816_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	err = regulator_enable(pw->vdd_i2c);
	if (unlikely(err))
		goto ad5816_vdd_i2c_fail;

	err = regulator_enable(pw->vdd);
	if (unlikely(err))
		goto ad5816_vdd_fail;

	return 0;

ad5816_vdd_fail:
	regulator_disable(pw->vdd_i2c);

ad5816_vdd_i2c_fail:
	pr_err("%s FAILED\n", __func__);

	return -ENODEV;
}

static int ceres_ad5816_power_off(struct ad5816_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	regulator_disable(pw->vdd);
	regulator_disable(pw->vdd_i2c);

	return 0;
}

static int ceres_imx091_power_on(struct nvc_regulator *vreg)
{
	int err;

	if (unlikely(WARN_ON(!vreg)))
		return -EFAULT;

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(10, 20);

	err = regulator_enable(vreg[IMX091_VREG_AVDD].vreg);
	if (err)
		goto imx091_avdd_fail;

	err = regulator_enable(vreg[IMX091_VREG_DVDD].vreg);
	if (unlikely(err))
		goto imx091_dvdd_fail;

	err = regulator_enable(vreg[IMX091_VREG_IOVDD].vreg);
	if (err)
		goto imx091_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);

	usleep_range(300, 310);

	return 1;

imx091_iovdd_fail:
	regulator_disable(vreg[IMX091_VREG_DVDD].vreg);

imx091_dvdd_fail:
	regulator_disable(vreg[IMX091_VREG_AVDD].vreg);

imx091_avdd_fail:
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);

	return -ENODEV;
}

static int ceres_imx091_power_off(struct nvc_regulator *vreg)
{
	if (unlikely(WARN_ON(!vreg)))
		return -EFAULT;

	usleep_range(1, 2);

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(1, 2);

	regulator_disable(vreg[IMX091_VREG_IOVDD].vreg);
	regulator_disable(vreg[IMX091_VREG_AVDD].vreg);
	regulator_disable(vreg[IMX091_VREG_DVDD].vreg);
	return 0;
}

static int ceres_imx135_power_on(struct imx135_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->dvdd || !pw->avdd)))
		return -EFAULT;

	err = gpio_request_one(CAM_RSTN,
			GPIOF_DIR_OUT | GPIOF_INIT_LOW,
			"camera_reset");
	if (err < 0)
		pr_notice("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_RSTN");

	err = gpio_request_one(CAM1_POWER_DWN_GPIO,
			GPIOF_DIR_OUT | GPIOF_INIT_LOW,
			"camera1_power_down");
	if (err < 0)
		pr_notice("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM1_POWER_DWN_GPIO");
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto imx135_avdd_fail;

	err = regulator_enable(pw->dvdd);
	if (err)
		goto imx135_dvdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto imx135_iovdd_fail;

	udelay(2);
	gpio_set_value(CAM_RSTN, 1);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);

	usleep_range(300, 310);

	return 1;

imx135_iovdd_fail:
	regulator_disable(pw->dvdd);

imx135_dvdd_fail:
	regulator_disable(pw->avdd);

imx135_avdd_fail:
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);

	return -ENODEV;
}

static int ceres_imx135_power_off(struct imx135_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->dvdd || !pw->avdd)))
		return -EFAULT;

	udelay(2);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	udelay(2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);

	gpio_free(CAM_RSTN);
	gpio_free(CAM1_POWER_DWN_GPIO);
	return 0;
}

static unsigned imx135_estates[] = { 656, 0 };

struct imx135_platform_data ceres_imx135_data = {
	.flash_cap = {
		.enable = 1,
		.edge_trig_en = 1,
		.start_edge = 0,
		.repeat = 1,
		.delay_frm = 0,
	},
	.edpc_config = {
		.states = imx135_estates,
		.num_states = ARRAY_SIZE(imx135_estates),
		.e0_index = ARRAY_SIZE(imx135_estates) - 1,
		.priority = EDP_MAX_PRIO + 1,
	},
	.power_on = ceres_imx135_power_on,
	.power_off = ceres_imx135_power_off,
};


static int ceres_imx132_power_on(struct imx132_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);


	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto imx132_avdd_fail;

	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto imx132_dvdd_fail;

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto imx132_iovdd_fail;

	usleep_range(1, 2);

	gpio_set_value(CAM2_POWER_DWN_GPIO, 1);

	return 0;

imx132_iovdd_fail:
	regulator_disable(pw->dvdd);

imx132_dvdd_fail:
	regulator_disable(pw->avdd);

imx132_avdd_fail:

	return -ENODEV;
}

static int ceres_imx132_power_off(struct imx132_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd || !pw->dvdd)))
		return -EFAULT;

	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);

	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);

	return 0;
}

static struct nvc_gpio_pdata imx091_gpio_pdata[] = {
	{IMX091_GPIO_RESET, CAM_RSTN, true, false},
	{IMX091_GPIO_PWDN, CAM1_POWER_DWN_GPIO, true, false},
};

static struct nvc_imager_cap imx091_cap = {
	.identifier		= "IMX091",
	.sensor_nvc_interface	= 3,
	.pixel_types[0]		= 0x100,
	.orientation		= 0,
	.direction		= 0,
	.initial_clock_rate_khz	= 6000,
	.clock_profiles[0] = {
		.external_clock_khz	= 24000,
		.clock_multiplier	= 850000, /* value / 1,000,000 */
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
	.discontinuous_clk_mode	= 0,
	.cil_threshold_settle	= 0xd,
	.min_blank_time_width	= 16,
	.min_blank_time_height	= 16,
	.preferred_mode_index	= 0,
	.focuser_guid		= NVC_FOCUS_GUID(0),
	.torch_guid		= NVC_TORCH_GUID(0),
	.cap_version		= NVC_IMAGER_CAPABILITIES_VERSION2,
};

static unsigned imx091_estates[] = {600, 0};

static struct imx091_platform_data ceres_imx091_data = {
	.num			= 0,
	.sync			= 0,
	.cfg			= 0,
	.dev_name		= "camera",
	.gpio_count		= ARRAY_SIZE(imx091_gpio_pdata),
	.gpio			= imx091_gpio_pdata,
	.flash_cap		= {
		.sdo_trigger_enabled = 1,
		.adjustable_flash_timing = 1,
	},
	.cap			= &imx091_cap,
	.edpc_config		= {
		.states = imx091_estates,
		.num_states = ARRAY_SIZE(imx091_estates),
		.e0_index = ARRAY_SIZE(imx091_estates) - 1,
		.priority = EDP_MAX_PRIO + 1,
	},
	.power_on		= ceres_imx091_power_on,
	.power_off		= ceres_imx091_power_off,
};

static unsigned imx132_estates[] = {202, 0};

struct imx132_platform_data ceres_imx132_data = {
	.edpc_config	= {
		.states = imx132_estates,
		.num_states = ARRAY_SIZE(imx132_estates),
		.e0_index = ARRAY_SIZE(imx132_estates) - 1,
		.priority = EDP_MAX_PRIO + 1,
		},
	.power_on = ceres_imx132_power_on,
	.power_off = ceres_imx132_power_off,
};

static struct ad5816_platform_data ceres_ad5816_pdata = {
	.cfg		= 0,
	.num		= 0,
	.sync		= 0,
	.dev_name	= "focuser",
	.power_on	= ceres_ad5816_power_on,
	.power_off	= ceres_ad5816_power_off,
};

/* estate values under 1000/200/0/0mA, 3.5V input */
static unsigned max77387_estates[] = {3500, 710, 0};

static struct max77387_platform_data ceres_max77387_pdata = {
	.config		= {
		.led_mask		= 3,
		.flash_trigger_mode	= 1,
		/* use ONE-SHOOT flash mode - flash triggered at the
		 * raising edge of strobe or strobe signal.
		*/
		.flash_mode		= 1,
		.def_ftimer		= 0x24,
		.max_total_current_mA	= 1000,
		.max_peak_current_mA	= 600,
		.led_config[0]	= {
			.flash_torch_ratio	= 18100,
			.granularity		= 1000,
			.flash_levels		= 0,
			.lumi_levels	= NULL,
			},
		.led_config[1]	= {
			.flash_torch_ratio	= 18100,
			.granularity		= 1000,
			.flash_levels		= 0,
			.lumi_levels		= NULL,
			},
		},
	.cfg		= 0,
	.dev_name	= "torch",
	.gpio_strobe	= CAM_FLASH_STROBE,
	.edpc_config	= {
		.states		= max77387_estates,
		.num_states	= ARRAY_SIZE(max77387_estates),
		.e0_index	= ARRAY_SIZE(max77387_estates) - 1,
		.priority	= EDP_MAX_PRIO + 2,
		},
};

/* estate values under 1000/200/0/0mA, 3.5V input */
static unsigned lm3565_estates[] = {3500, 710, 0};

static struct lm3565_platform_data atlantis_lm3565_pdata = {
	.config		= {
		.max_peak_current_mA	= 930,
		.vin_low_v_mV		= 3000,
		.vin_low_c_mA		= 210,
		.strobe_type		= 2, /* Edge signal strobe */
		},
	.enable_gpio	= {
		.gpio_type		= 1,
		.gpio			= TEGRA_GPIO_PS1,
		.init_en		= true,
		.active_high		= true,
		},
	.dev_name	= "torch",
	.edpc_config	= {
		.states		= lm3565_estates,
		.num_states	= ARRAY_SIZE(lm3565_estates),
		.e0_index	= ARRAY_SIZE(lm3565_estates) - 1,
		.priority	= EDP_MAX_PRIO + 2,
		},
};

static int ceres_ar0833_power_on(struct ar0833_power_rail *pw)
{
	int err;
	pr_debug("%s ++\n", __func__);

	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	gpio_set_value(CAM_RSTN, 0);
	usleep_range(1000, 1020);

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto ar0833_iovdd_fail;
	usleep_range(300, 320);

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto ar0833_avdd_fail;

	usleep_range(1000, 1020);
	gpio_set_value(CAM_RSTN, 1);

	usleep_range(200, 220);

	/* return 1 to skip the in-driver power_on sequence */
	pr_debug("%s --\n", __func__);
	return 0;

ar0833_avdd_fail:
	regulator_disable(pw->iovdd);

ar0833_iovdd_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int ceres_ar0833_power_off(struct ar0833_power_rail *pw)
{
	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	usleep_range(100, 120);
	gpio_set_value(CAM_RSTN, 0);
	regulator_disable(pw->avdd);
	usleep_range(100, 120);
	regulator_disable(pw->iovdd);

	return 1;
}

struct ar0833_platform_data ceres_ar0833_pdata = {
	.power_on = ceres_ar0833_power_on,
	.power_off = ceres_ar0833_power_off,
};

static int ceres_dw9718_power_on(struct dw9718_power_rail *pw)
{
	int err;
	pr_info("%s\n", __func__);

	if (unlikely(!pw || !pw->vdd || !pw->vdd_i2c))
		return -EFAULT;

	err = regulator_enable(pw->vdd);
	if (unlikely(err))
		goto dw9718_vdd_fail;

	err = regulator_enable(pw->vdd_i2c);
	if (unlikely(err))
		goto dw9718_i2c_fail;

	usleep_range(1000, 1020);

	/* return 1 to skip the in-driver power_on sequence */
	pr_debug("%s --\n", __func__);
	return 1;

dw9718_i2c_fail:
	regulator_disable(pw->vdd);

dw9718_vdd_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int ceres_dw9718_power_off(struct dw9718_power_rail *pw)
{
	pr_info("%s\n", __func__);

	if (unlikely(!pw || !pw->vdd || !pw->vdd_i2c))
		return -EFAULT;

	regulator_disable(pw->vdd);
	regulator_disable(pw->vdd_i2c);

	return 1;
}

static struct nvc_focus_cap dw9718_cap = {
	.settle_time = 30,
	.slew_rate = 0x3A200C,
	.focus_macro = 450,
	.focus_infinity = 200,
	.focus_hyper = 200,
};

static struct dw9718_platform_data ceres_dw9718_pdata = {
	.cfg = 0,
	.num = 1,
	.sync = 0,
	.dev_name = "focuser",
	.cap = &dw9718_cap,
	.power_on = ceres_dw9718_power_on,
	.power_off = ceres_dw9718_power_off,
};

static int ceres_ov5693_power_on(struct ov5693_power_rail *pw)
{
	int err;

	if (unlikely(!pw || !pw->avdd || !pw->dovdd))
		return -EFAULT;

	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto ov5693_avdd_fail;

	err = regulator_enable(pw->dovdd);
	if (err)
		goto ov5693_iovdd_fail;

	udelay(2);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 1);
	usleep_range(300, 310);

	return 0;

ov5693_iovdd_fail:
	regulator_disable(pw->avdd);

ov5693_avdd_fail:
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);

	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int ceres_ov5693_power_off(struct ov5693_power_rail *pw)
{
	if (unlikely(!pw || !pw->dovdd || !pw->avdd))
		return -EFAULT;

	usleep_range(21, 25);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
	udelay(2);

	regulator_disable(pw->dovdd);
	regulator_disable(pw->avdd);

	return 0;
}

static struct nvc_gpio_pdata ov5693_gpio_pdata[] = {
	{ OV5693_GPIO_TYPE_PWRDN, CAM_RSTN, true, 0, },
};

static struct ov5693_platform_data ceres_ov5693_pdata = {
	.num		= 5693,
	.dev_name	= "camera",
	.gpio_count	= ARRAY_SIZE(ov5693_gpio_pdata),
	.gpio		= ov5693_gpio_pdata,
	.power_on	= ceres_ov5693_power_on,
	.power_off	= ceres_ov5693_power_off,
};

static int ceres_ad5823_power_on(struct ad5823_platform_data *pdata)
{
	int err = 0;

	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 1);

	return err;
}

static int ceres_ad5823_power_off(struct ad5823_platform_data *pdata)
{
	pr_info("%s\n", __func__);
	gpio_set_value_cansleep(pdata->gpio, 0);

	return 0;
}

static struct ad5823_platform_data ceres_ad5823_pdata = {
	.gpio = CAM_AF_PWDN,
	.power_on	= ceres_ad5823_power_on,
	.power_off	= ceres_ad5823_power_off,
};

static struct regulator *dvdd_1v8;
static struct regulator *avdd_cam2;
static int ceres_ov5640_power_on(struct device *dev)
{
	if (avdd_cam2 == NULL) {
		avdd_cam2 = regulator_get(dev, "avdd");
		if (WARN_ON(IS_ERR(avdd_cam2))) {
			pr_err("%s: couldn't get regulator avdd_cam2: %ld\n",
				__func__, PTR_ERR(avdd_cam2));
			avdd_cam2 = NULL;
			goto avdd_cam2_fail;
		}
	}
	if (dvdd_1v8 == NULL) {
		dvdd_1v8 = regulator_get(dev, "dvdd");
		if (WARN_ON(IS_ERR(dvdd_1v8))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam: %ld\n",
				__func__, PTR_ERR(dvdd_1v8));
			dvdd_1v8 = NULL;
			goto dvdd_1v8_fail;
		}
	}

	/* power up sequence */
	gpio_set_value(CAM2_POWER_DWN_GPIO, 1);
	gpio_set_value(CAM_RSTN, 0);
	mdelay(1);

	regulator_enable(dvdd_1v8);
	regulator_enable(avdd_cam2);

	mdelay(5);
	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);
	mdelay(1);
	gpio_set_value(CAM_RSTN, 1);

	mdelay(20);

	return 0;

dvdd_1v8_fail:
	regulator_disable(avdd_cam2);
avdd_cam2_fail:
	return -ENODEV;
}

static int ceres_ov5640_power_off(struct device *dev)
{
	gpio_set_value(CAM_RSTN, 0);

	if (avdd_cam2)
		regulator_disable(avdd_cam2);
	if (dvdd_1v8)
		regulator_disable(dvdd_1v8);

	gpio_set_value(CAM2_POWER_DWN_GPIO, 1);

	return 0;
}

struct ov5640_platform_data ceres_ov5640_data = {
	.power_on = ceres_ov5640_power_on,
	.power_off = ceres_ov5640_power_off,
};

static struct i2c_board_info ceres_i2c_board_info_ov5640 = {
	I2C_BOARD_INFO("ov5640", 0x3c),
	.platform_data = &ceres_ov5640_data,
};

static struct i2c_board_info ceres_i2c_board_info_imx091 = {
	I2C_BOARD_INFO("imx091", 0x10),
	.platform_data = &ceres_imx091_data,
};

static struct i2c_board_info ceres_i2c_board_info_ar0833 = {
	I2C_BOARD_INFO("ar0833", 0x36),
	.platform_data = &ceres_ar0833_pdata,
};

static struct i2c_board_info ceres_i2c_board_info_ov5693 = {
	I2C_BOARD_INFO("ov5693", 0x10),
	.platform_data = &ceres_ov5693_pdata,
};

static struct i2c_board_info ceres_i2c_board_info_imx132 = {
	I2C_BOARD_INFO("imx132", 0x36),
	.platform_data = &ceres_imx132_data,
};

static struct i2c_board_info ceres_i2c_board_info_ad5816 = {
	I2C_BOARD_INFO("ad5816", 0x0E),
	.platform_data = &ceres_ad5816_pdata,
};

static struct i2c_board_info ceres_i2c_board_info_ad5823 = {
		I2C_BOARD_INFO("ad5823", 0x0c),
		.platform_data = &ceres_ad5823_pdata,
};

static struct i2c_board_info ceres_i2c_board_info_dw9718 = {
	I2C_BOARD_INFO("dw9718", 0x0c),
	.platform_data = &ceres_dw9718_pdata,
};

static struct i2c_board_info ceres_i2c_board_info_imx135 = {
	I2C_BOARD_INFO("imx135", 0x10),
	.platform_data = &ceres_imx135_data,
};

static struct i2c_board_info ceres_i2c_board_info_max77387 = {
	I2C_BOARD_INFO("max77387", 0x4A),
	.platform_data = &ceres_max77387_pdata,
};

static struct i2c_board_info atlantis_i2c_board_info_lm3565 = {
	I2C_BOARD_INFO("lm3565", 0x30),
	.platform_data = &atlantis_lm3565_pdata,
};

static struct camera_module ceres_camera_module_info[] = {
	/* E1707 camera board */
	{
		/* rear camera */
		.sensor = &ceres_i2c_board_info_imx091,
		.focuser = &ceres_i2c_board_info_ad5816,
		.flash = &ceres_i2c_board_info_max77387,
	},
	{
		/* rear camera */
		.sensor = &ceres_i2c_board_info_imx135,
		.focuser = &ceres_i2c_board_info_ad5816,
		.flash = &ceres_i2c_board_info_max77387,
	},
	{
		/* rear camera */
		.sensor = &ceres_i2c_board_info_ar0833,
		.focuser = &ceres_i2c_board_info_dw9718,
		.flash = &ceres_i2c_board_info_max77387,
	},
	{
		/* rear camera */
		.sensor = &ceres_i2c_board_info_ov5693,
		.focuser = &ceres_i2c_board_info_ad5823,
	},
	{
		/* rear camera */
		.sensor = &ceres_i2c_board_info_ov5640,
	},
	{
		/* front camera */
		.sensor = &ceres_i2c_board_info_imx132,
	},

	{}
};

static struct camera_platform_data ceres_pcl_pdata = {
	.cfg = 0xAA55AA55,
	.modules = ceres_camera_module_info,
};

static struct platform_device ceres_camera_generic = {
	.name = "pcl-generic",
	.id = -1,
};

static struct i2c_board_info ceres_i2c_board_info_e1740[] = {
	{
		I2C_BOARD_INFO("imx135", 0x10),
		.platform_data = &ceres_imx135_data,
	},
	{
		I2C_BOARD_INFO("imx132", 0x36),
		.platform_data = &ceres_imx132_data,
	},
	{
		I2C_BOARD_INFO("ad5816", 0x0E),
		.platform_data = &ceres_ad5816_pdata,
	},
	{
		I2C_BOARD_INFO("lm3565", 0x30),
		.platform_data = &atlantis_lm3565_pdata,
	},
};

static struct i2c_board_info ceres_i2c_board_info_e1690[] = {
	{
		I2C_BOARD_INFO("imx135", 0x10),
		.platform_data = &ceres_imx135_data,
	},
	{
		I2C_BOARD_INFO("imx132", 0x36),
		.platform_data = &ceres_imx132_data,
	},
	{
		I2C_BOARD_INFO("ad5816", 0x0E),
		.platform_data = &ceres_ad5816_pdata,
	},
	{
		I2C_BOARD_INFO("max77387", 0x4A),
		.platform_data = &ceres_max77387_pdata,
	},
};

static int ceres_camera_init(void)
{
	/* on atlantis FFD, only IMX135/AD5816/LM3565/IMX132 are
	supported, auto-detection is not neccessary. */
	if (board_info.board_id == BOARD_E1740) {
		i2c_register_board_info(2, ceres_i2c_board_info_e1740,
			ARRAY_SIZE(ceres_i2c_board_info_e1740));
		return 0;
	}

	/* on ceres FFD, only IMX135/AD5816/LM3565/IMX132 are
	supported, auto-detection is not neccessary. */
	if (board_info.board_id == BOARD_E1690) {
		i2c_register_board_info(2, ceres_i2c_board_info_e1690,
			ARRAY_SIZE(ceres_i2c_board_info_e1690));
		return 0;
	}

	/* the default camera board on atlantis ERS is E1697, it's almost the
	same as E1707 except the flash device is lm3565 instead of max77387. */
	if (board_info.board_id == BOARD_E1670)
		ceres_camera_module_info[0].flash =
			&atlantis_i2c_board_info_lm3565;

	platform_device_add_data(&ceres_camera_generic,
		&ceres_pcl_pdata, sizeof(ceres_pcl_pdata));
	platform_device_register(&ceres_camera_generic);

	return 0;
}

static struct i2c_board_info __initdata ceres_i2c_board_info_max44005[] = {
	{
		I2C_BOARD_INFO("max44005", 0x44),
	},
};

static struct i2c_board_info __initdata ceres_i2c_board_info_tcs3772[] = {
	{
		I2C_BOARD_INFO("tcs3772", 0x29),
	},
};

/* MPU board file definition	*/
static struct mpu_platform_data mpu9150_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu9150_gyro_data_e1680 = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION_E1680,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu9150_gyro_data_e1670 = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION_E1670,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu_compass_data = {
	.orientation	= MPU_COMPASS_ORIENTATION,
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct mpu_platform_data mpu_compass_data_e1680 = {
	.orientation	= MPU_COMPASS_ORIENTATION_E1680,
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct mpu_platform_data mpu_compass_data_e1670 = {
	.orientation	= MPU_COMPASS_ORIENTATION_E1670,
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct mpu_platform_data bmp180_pdata = {
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct i2c_board_info __initdata inv_mpu9150_i2c1_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu9150_gyro_data,
	},
	{
		/* The actual BMP180 address is 0x77 but because this conflicts
		 * with another device, this address is hacked so Linux will
		 * call the driver.  The conflict is technically okay since the
		 * BMP180 is behind the MPU.  Also, the BMP180 driver uses a
		 * hard-coded address of 0x77 since it can't be changed anyway.
		 */
		I2C_BOARD_INFO("bmp180", 0x78),
		.platform_data = &bmp180_pdata,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = MPU_GYRO_IRQ_GPIO;
	unsigned gyro_bus_num = MPU_GYRO_BUS_NUM;
	char *gyro_name = MPU_GYRO_NAME;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(gyro_irq_gpio, gyro_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	if (board_info.board_id == BOARD_E1680) {
		inv_mpu9150_i2c1_board_info[0].platform_data
					= &mpu9150_gyro_data_e1680;
		inv_mpu9150_i2c1_board_info[2].platform_data
					= &mpu_compass_data_e1680;
	} else if (board_info.board_id == BOARD_E1670) {
		inv_mpu9150_i2c1_board_info[0].platform_data
					= &mpu9150_gyro_data_e1670;
		inv_mpu9150_i2c1_board_info[2].platform_data
					= &mpu_compass_data_e1670;
	}
	inv_mpu9150_i2c1_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu9150_i2c1_board_info,
		ARRAY_SIZE(inv_mpu9150_i2c1_board_info));
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_trip_info skin_trips[] = {
	{
		.cdev_type = "skin-balanced",
		.trip_temp = 43000,
		.trip_type = THERMAL_TRIP_PASSIVE,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
	{
		.cdev_type = "tegra-shutdown",
		.trip_temp = 57000,
		.trip_type = THERMAL_TRIP_CRITICAL,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
};

static struct therm_est_subdevice skin_devs[] = {
	{
		.dev_data = "Tdiode",
		.coeffs = {
			-2, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, -1,
			-1, -1, -1, -3
		},
	},
	{
		.dev_data = "Tboard",
		.coeffs = {
			6, 5, 5, 5,
			5, 5, 5, 5,
			5, 5, 4, 4,
			4, 4, 4, 3,
			3, 2, 2, 1
		},
	},
};

static struct pid_thermal_gov_params skin_pid_params = {
	.max_err_temp = 4000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 15,
	.down_compensation = 15,
};

static struct thermal_zone_params skin_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &skin_pid_params,
};

static struct therm_est_data skin_data = {
	.num_trips = ARRAY_SIZE(skin_trips),
	.trips = skin_trips,
	.toffset = 7450,
	.polling_period = 1100,
	.passive_delay = 15000,
	.tc1 = 10,
	.tc2 = 1,
	.ndevs = ARRAY_SIZE(skin_devs),
	.devs = skin_devs,
	.tzp = &skin_tzp,
};

static struct throttle_table skin_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1938000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1912500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1887000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1861500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1836000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1275000, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1249500, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1224000, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1198500, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1173000, 710400, NO_CAP, NO_CAP, 768000 } },
	{ { 1147500, 672000, NO_CAP, NO_CAP, 768000 } },
	{ { 1122000, 672000, NO_CAP, NO_CAP, 768000 } },
	{ { 1096500, 672000, NO_CAP, NO_CAP, 768000 } },
	{ { 1071000, 672000, NO_CAP, 384000, 768000 } },
	{ { 1045500, 672000, 499200, 384000, 768000 } },
	{ { 1020000, 595200, 499200, 384000, 768000 } },
	{ {  994500, 595200, 499200, 384000, 768000 } },
	{ {  969000, 595200, 499200, 384000, 768000 } },
	{ {  943500, 595200, 499200, 384000, 768000 } },
	{ {  918000, 595200, 499200, 384000, 768000 } },
	{ {  892500, 595200, 460800, 384000, 768000 } },
	{ {  867000, 556800, 460800, 384000, 768000 } },
	{ {  841500, 556800, 460800, 384000, 768000 } },
	{ {  816000, 556800, 460800, 384000, 652800 } },
	{ {  790500, 556800, 460800, 384000, 652800 } },
	{ {  765000, 556800, 460800, 204000, 652800 } },
	{ {  739500, 556800, 403200, 204000, 652800 } },
	{ {  714000, 499200, 403200, 204000, 652800 } },
	{ {  688500, 499200, 403200, 204000, 518400 } },
	{ {  663000, 499200, 403200, 204000, 518400 } },
	{ {  637500, 499200, 403200, 102000, 518400 } },
	{ {  612000, 499200, 345600, 102000, 518400 } },
	{ {  586500, 422400, 345600, 102000, 518400 } },
	{ {  561000, 422400, 345600, 102000, 518400 } },
	{ {  535500, 422400, 345600, 102000, 408000 } },
	{ {  510000, 422400, 345600, 102000, 408000 } },
	{ {  484500, 422400, 288000, 102000, 408000 } },
	{ {  459000, 307200, 288000, 102000, 408000 } },
	{ {  433500, 307200, 288000, 102000, 408000 } },
	{ {  408000, 307200, 288000, 102000, 408000 } },
	{ {  382500, 307200, 288000, 102000, 408000 } },
	{ {  357000, 307200, 288000, 102000, 408000 } },
	{ {  331500, 307200, 288000, 102000, 408000 } },
	{ {  306000, 307200, 288000, 102000, 408000 } },
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};

static int __init ceres_skin_init(void)
{
	int i;

	if (of_machine_is_compatible("nvidia,ceres")) {
		if ((board_info.board_id == BOARD_E1690 &&
			board_info.fab <= BOARD_FAB_B) ||
				is_tskin_shutdown_disabled()) {
			/* we effectively disabled the shutdown trip point */
			for (i = 0; i < skin_data.num_trips; i++) {
				if (strcmp(skin_data.trips[i].cdev_type,
					  "tegra-shutdown") == 0) {
					break;
				}
			}

			if (i != skin_data.num_trips)
				skin_data.trips[i].trip_temp = 200000;
		}

		balanced_throttle_register(&skin_throttle, "skin-balanced");
		tegra_skin_therm_est_device.dev.platform_data = &skin_data;
		platform_device_register(&tegra_skin_therm_est_device);
	}

	return 0;
}
late_initcall(ceres_skin_init);
#endif

int __init ceres_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	ceres_camera_init();

	mpuirq_init();

	err = ceres_nct1008_init();
	if (err)
		pr_err("%s: nct1008 init failed\n", __func__);
	else
		i2c_register_board_info(0, ceres_i2c0_nct1008_board_info,
				ARRAY_SIZE(ceres_i2c0_nct1008_board_info));

	if ((board_info.board_id != BOARD_E1670) &&
		 (board_info.board_id != BOARD_E1740)) {
		i2c_register_board_info(0, ceres_i2c_board_info_max44005,
				ARRAY_SIZE(ceres_i2c_board_info_max44005));
		if (get_power_supply_type() == POWER_SUPPLY_TYPE_BATTERY)
			i2c_register_board_info(0, max77660_fg_board_info, 1);
	} else {
		i2c_register_board_info(0, ceres_i2c_board_info_tcs3772,
				ARRAY_SIZE(ceres_i2c_board_info_tcs3772));
	}

	return 0;
}
