/*
 * camera.c - generic camera device driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * Contributors:
 *	Charlie Huang <chahuang@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if (defined(CONFIG_MACH_S9321) && CONFIG_MACH_S9321) || \
	(defined(CONFIG_MACH_S8515) && CONFIG_MACH_S8515)
	#define POWER_CONTROL_ENABLE	1
#else
	#define POWER_CONTROL_ENABLE	0
#endif

#define CAMERA_DEVICE_INTERNAL
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <media/nvc.h>
#include <media/nvc_torch.h>
#include <linux/power_supply.h>
#if defined(POWER_CONTROL_ENABLE) && POWER_CONTROL_ENABLE
#include <linux/edpdev.h>
#endif

#include <media/camera.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/edp.h>
#include <linux/regmap.h>
#include <media/nvc.h>
#include <media/sgm3780.h>
#include <linux/hrtimer.h>

#define tinno_flash_LED_NUM		1
#define tinno_flash_FLASH_LEVELS		(1 << 3)
#define tinno_flash_MAX_FLASH_LEVEL	(tinno_flash_FLASH_LEVELS + 1)
#define tinno_flash_TORCH_LEVELS		(1 << 3)
#define tinno_flash_MAX_TORCH_LEVEL	(tinno_flash_TORCH_LEVELS + 1)

#define tinno_flash_FLASH_TIMER_NUM	(1 << 3)
#define tinno_flash_TORCH_TIMER_NUM	(1 << 3)
#define tinno_flash_FTIMER_MASK		(tinno_flash_FLASH_TIMER_NUM - 1)
#define tinno_flash_TTIMER_MASK		(tinno_flash_TORCH_TIMER_NUM - 1)

#define MAXFLASH_MODE_NONE		 0
#define MAXFLASH_MODE_TORCH		1
#define MAXFLASH_MODE_FLASH		2

#define TINNO_TORCH_CURRENT		10
#define TINNO_FLASH_CURRENT		27
#define TINNO_BAT_CAP_LIMIT		15

#define tinno_flash_flash_cap_size \
			(sizeof(struct nvc_torch_flash_capabilities_v1) \
			+ sizeof(struct nvc_torch_lumi_level_v1) \
			* tinno_flash_MAX_FLASH_LEVEL)
#define tinno_flash_flash_timeout_size \
			(sizeof(struct nvc_torch_timer_capabilities_v1) \
			+ sizeof(struct nvc_torch_timeout_v1) \
			* tinno_flash_FLASH_TIMER_NUM)
#define tinno_flash_max_flash_cap_size (tinno_flash_flash_cap_size * 2 \
			+ tinno_flash_flash_timeout_size * 2)

#define tinno_flash_torch_cap_size \
			(sizeof(struct nvc_torch_torch_capabilities_v1) \
			+ sizeof(struct nvc_torch_lumi_level_v1) \
			* tinno_flash_MAX_TORCH_LEVEL)
#define tinno_flash_torch_timeout_size \
			(sizeof(struct nvc_torch_timer_capabilities_v1) \
			+ sizeof(struct nvc_torch_timeout_v1) \
			* tinno_flash_TORCH_TIMER_NUM)
#define tinno_flash_max_torch_cap_size (tinno_flash_torch_timeout_size * 2\
			+ tinno_flash_torch_timeout_size * 2)

struct tinno_flash_info {
	struct miscdevice miscdev;
	struct device *dev;
	struct mutex mutex;
	struct tinno_flash_power_rail pwr_rail;
	struct tinno_flash_platform_data *pdata;
	struct nvc_torch_capability_query query;
	struct nvc_torch_flash_capabilities_v1 *flash_cap[2];
	struct nvc_torch_timer_capabilities_v1 *flash_timeouts[2];
	struct nvc_torch_torch_capabilities_v1 *torch_cap[2];
	struct nvc_torch_timer_capabilities_v1 *torch_timeouts[2];
	struct tinno_flash_config config;
	struct edp_client *edpc;
	unsigned edp_state;
	atomic_t in_use;
	int flash_cap_size;
	int torch_cap_size;
	int pwr_state;
	u8 max_flash[2];
	u8 max_torch[2];
	u8 op_mode;
	u8 power_is_on;
	u8 new_timer;
	int gpio_en_torch;
	int gpio_en_flash;
	unsigned char flash_level;
	unsigned char torch_level;
	int edp_state_flash;
	int edp_state_torch;
	struct hrtimer timeout;
	spinlock_t lock;

#if defined(POWER_CONTROL_ENABLE) && POWER_CONTROL_ENABLE
	struct power_supply *psy;
	struct psy_depletion_platform_data *psy_data;

	/* state machine for torch/flash control */
	int torch_flash_low_bat_state;
	int torch_phase;
#endif
};

#if defined(POWER_CONTROL_ENABLE) && POWER_CONTROL_ENABLE
#define TORCH_PHASE_ON                  1
#define TORCH_PHASE_OFF                 0
#define TORCH_FLASH_LOW_BAT_ON          1
#define TORCH_FLASH_LOW_BAT_OFF         0
#endif

struct tinno_flash_info *info;
static struct nvc_torch_lumi_level_v1
		tinno_flash_def_flash_levels[tinno_flash_FLASH_LEVELS];

static struct tinno_flash_platform_data tinno_flash_default_pdata = {
	.config		= {
			.led_config[0] = {
				.flash_torch_ratio = 10000,
				.granularity = 1000,
				.flash_levels = ARRAY_SIZE(
				tinno_flash_def_flash_levels),
				.lumi_levels = tinno_flash_def_flash_levels,
				},
		},
	.dev_name	= "torch",
	.pinstate	= {0x0004, 0x0004},
};

/* flash timer duration settings in uS */
static u32 tinno_flash_flash_timer[tinno_flash_FLASH_TIMER_NUM] = {
	128, 384, 640, 896, 1410, 1920, 2430, 2940,
};

/* torch timer duration settings in uS */
static u32 tinno_flash_torch_timer[tinno_flash_TORCH_TIMER_NUM] = {
	128, 384, 640, 896, 1410, 1920, 2430, 2940,
};

#if defined(POWER_CONTROL_ENABLE) && POWER_CONTROL_ENABLE
static int bat_read_cap(struct tinno_flash_info *info)
{
	union power_supply_propval pv;
	if (info->psy) {
		if (!info->psy->get_property(info->psy,
				POWER_SUPPLY_PROP_CAPACITY, &pv))
			return pv.intval;
		else
			return -1;
	}
	return -1;
}

#if 0
static int bat_read_vol(struct tinno_flash_info *info)
{
	union power_supply_propval pv;
	if (info->psy) {
		/* if (! info->psy->get_property(info->psy,
			POWER_SUPPLY_PROP_VOLTAGE_OCV, &pv)) */
		if (! info->psy->get_property(info->psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pv))
			return pv.intval;
		else
			return -1;
	}
	return -1;
}

static int interpolate(int x, int x1, int y1, int x2, int y2)
{
#if 0
	if (x1 == x2)
		return y1;
	return (y2 * (x - x1) - y1 * (x - x2)) / (x2 - x1);
#else
	return (y1 + y2) >> 1;
#endif
}

static int bat_read_rbat(struct tinno_flash_info *info, unsigned int capacity)
{
	struct psy_depletion_rbat_lut *p;
	struct psy_depletion_rbat_lut *q;
	int rbat = 0;

	if (info->psy_data == NULL)
		return -1;

	/* rbat = info->psy_data->r_const; */
	p = info->psy_data->rbat_lut;
	if (!p)
		return rbat;

	while (p->capacity > capacity)
		p++;

	if (p == info->psy_data->rbat_lut)
		return rbat + p->rbat;

	q = p - 1;

	rbat += interpolate(capacity, p->capacity, p->rbat,
			q->capacity, q->rbat);
	return rbat;
}

static int bat_vol_drop(int ma_100, int r)
{
	return ma_100 * r / 10;
}
#endif
/*
 whether the ma_100 can be satisfied by bat
 return 0 means safe
 return -1 means unsafe and will lead to system shutdown
*/
static int bat_is_safe(struct tinno_flash_info *info, int ma_100)
{
#if 0
	int vol = 0;
	int cap = 0;
	int rbat = 0;
	int v_drop = 0;

	if (info->psy == NULL)
		return 0;

	vol = bat_read_vol(info);
	if (vol < 0)
		return 0;

	cap = bat_read_cap(info);
	if (cap < 0)
		return 0;

	rbat = bat_read_rbat(info, cap);
	if (rbat < 0)
		return 0;

	v_drop = bat_vol_drop(ma_100, rbat);

	if (vol - v_drop > 3000000)
		return 0;
	else
		return -1;
#else
	/* use only bat capacity */
	int cap = 0;

	if (info->psy == NULL)
		return 0;

	cap = bat_read_cap(info);
	if (cap < 0)
		return 0;

	if (cap >= TINNO_BAT_CAP_LIMIT)
		return 0;
	else
		return -1;
#endif
}
#endif

static void tinno_flash_set_flash(struct tinno_flash_info *info, int on);
static void tinno_flash_set_torch(struct tinno_flash_info *info, int on);
static void start_timer(struct tinno_flash_info *info);

static void tinno_flash_cb(
struct tinno_flash_info *info,
union nvc_imager_flash_control *fm)
{
	unsigned long flags;
	if (fm->settings.enable) {
		/* need to protect */
		spin_lock_irqsave(&info->lock, flags);
		tinno_flash_set_flash(info, 1);
		start_timer(info);
		spin_unlock_irqrestore(&info->lock, flags);
	} else {
		spin_lock_irqsave(&info->lock, flags);
		tinno_flash_set_flash(info, 0);
		spin_unlock_irqrestore(&info->lock, flags);
	}
}


static void tinno_flash_dev_cb(
struct device *dev,
union nvc_imager_flash_control *fc)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tinno_flash_info *info = NULL;

	info = platform_get_drvdata(pdev);

	if (info == NULL)
		return ;

	tinno_flash_cb(info, fc);
}

static void tinno_flash_throttle(unsigned int new_state, void *priv_data)
{
#if 0
	struct tinno_flash_info *info = priv_data;

	if (!info)
		return;

	/* if we are throttled, then our state cannot be met */
	if (info->flash_level) {
		tinno_flash_set_flash(info, 0);
		info->flash_level = 0;
	}

	if (info->torch_level) {
		tinno_flash_set_torch(info, 0);
		info->torch_level = 0;
	}
#endif
}


static void tinno_flash_edp_lowest(struct tinno_flash_info *info)
{
	if (!info->edpc)
		return;

	info->edp_state = info->edpc->num_states - 1;
	dev_dbg(info->dev, "%s %d\n", __func__, info->edp_state);
	if (edp_update_client_request(info->edpc, info->edp_state, NULL)) {
		dev_err(info->dev, "THIS IS NOT LIKELY HAPPEN!\n");
		dev_err(info->dev, "UNABLE TO SET LOWEST EDP STATE!\n");
	}
}
static void tinno_flash_edp_register(struct tinno_flash_info *info)
{
	struct edp_manager *edp_manager;
	struct edp_client *edpc = &info->pdata->edpc_config;
	int ret;

	info->edpc = NULL;
	if (!edpc->num_states) {
		dev_notice(info->dev, "%s: NO edp states defined.\n", __func__);
		return;
	}

	strncpy(edpc->name, "flash_torch", EDP_NAME_LEN - 1);
	edpc->name[EDP_NAME_LEN - 1] = 0;

	edpc->throttle = tinno_flash_throttle;
	edpc->private_data = info;

	dev_dbg(info->dev, "%s: %s, e0 = %d, p %d\n",
		__func__, edpc->name, edpc->e0_index, edpc->priority);
	for (ret = 0; ret < edpc->num_states; ret++)
		dev_dbg(info->dev, "e%d = %d mA\n",
			ret - edpc->e0_index, edpc->states[ret]);

	edp_manager = edp_get_manager("battery");
	if (!edp_manager) {
		dev_err(info->dev, "unable to get edp manager: battery\n");
		return;
	}

	ret = edp_register_client(edp_manager, edpc);
	if (ret) {
		dev_err(info->dev, "unable to register edp client\n");
		return;
	}

	info->edpc = edpc;

	info->edp_state_flash = info->edp_state_torch = -1;
	if (info->pdata->edp_state_flash >= 0)
		info->edp_state_flash = info->pdata->edp_state_flash;
	if (info->pdata->edp_state_torch >= 0)
		info->edp_state_torch = info->pdata->edp_state_torch;

	/* set to lowest state at init */
	tinno_flash_edp_lowest(info);
}

static int tinno_flash_remove(struct platform_device *dev)
{
	struct tinno_flash_info *info = NULL;

	info = platform_get_drvdata(dev);

	if (hrtimer_active(&info->timeout))
		hrtimer_cancel(&info->timeout);
	if (info->edpc) {
		edp_unregister_client(info->edpc);
		info->edpc = NULL;
	}

	if (info->gpio_en_torch > 0) {
		gpio_free(info->gpio_en_torch);
		info->gpio_en_torch = -1;
	}

	if (info->gpio_en_flash > 0) {
		gpio_free(info->gpio_en_flash);
		info->gpio_en_flash = -1;
	}

	misc_deregister(&info->miscdev);
	platform_set_drvdata(dev, NULL);

	return 0;
}

static void tinno_flash_caps_layout(struct tinno_flash_info *info)
{
#define tinno_flash_FLASH_CAP_TIMEOUT_SIZE \
	(tinno_flash_flash_cap_size + tinno_flash_flash_timeout_size)
#define tinno_flash_TORCH_CAP_TIMEOUT_SIZE \
	(tinno_flash_torch_cap_size + tinno_flash_torch_timeout_size)
	void *start_ptr = (void *)info + sizeof(*info);

	info->flash_cap[0] = start_ptr;
	info->flash_timeouts[0] = start_ptr + tinno_flash_flash_cap_size;

	start_ptr += tinno_flash_FLASH_CAP_TIMEOUT_SIZE;
	info->flash_cap[1] = start_ptr;
	info->flash_timeouts[1] = start_ptr + tinno_flash_flash_cap_size;

	info->flash_cap_size = tinno_flash_FLASH_CAP_TIMEOUT_SIZE;

	start_ptr += tinno_flash_FLASH_CAP_TIMEOUT_SIZE;
	info->torch_cap[0] = start_ptr;
	info->torch_timeouts[0] = start_ptr + tinno_flash_torch_cap_size;

	start_ptr += tinno_flash_TORCH_CAP_TIMEOUT_SIZE;
	info->torch_cap[1] = start_ptr;
	info->torch_timeouts[1] = start_ptr + tinno_flash_torch_cap_size;

	info->torch_cap_size = tinno_flash_TORCH_CAP_TIMEOUT_SIZE;
	dev_dbg(info->dev, "%s: %d(%d + %d), %d(%d + %d)\n", __func__,
		info->flash_cap_size, tinno_flash_flash_cap_size,
		tinno_flash_flash_timeout_size, info->torch_cap_size,
		tinno_flash_torch_cap_size, tinno_flash_torch_timeout_size);
}

static void tinno_flash_update_config(struct tinno_flash_info *info)
{
	struct tinno_flash_config *pcfg = &info->config;
	int i;
	int delta;

	dev_dbg(info->dev, "%s +++\n", __func__);
	dev_dbg(info->dev, "tinno_flash_def_flash_levels:\n");
	for (i = 0; i < ARRAY_SIZE(tinno_flash_def_flash_levels); i++) {
		tinno_flash_def_flash_levels[i].guidenum = i;
		tinno_flash_def_flash_levels[i].luminance = i;

		dev_dbg(info->dev, "0x%02x - %d\n",
			i, tinno_flash_def_flash_levels[i].luminance);
	}

	dev_dbg(info->dev, "tinno_flash_flash_timer:\n");
	delta = 1024;
	for (i = 8; i < ARRAY_SIZE(tinno_flash_flash_timer); i++) {
		tinno_flash_flash_timer[i] = tinno_flash_flash_timer[i - 1] + delta;

		dev_dbg(info->dev,
			"0x%02x - %06d\n", i, tinno_flash_flash_timer[i]);
	}

	dev_dbg(info->dev, "tinno_flash_torch_timer:\n");
	delta = 1024;
	for (i = 8; i < ARRAY_SIZE(tinno_flash_torch_timer); i++) {
		tinno_flash_torch_timer[i] = tinno_flash_torch_timer[i - 1] + delta;
		dev_dbg(info->dev,
			"0x%02x - %08d\n", i, tinno_flash_torch_timer[i]);
	}

	memcpy(pcfg, &tinno_flash_default_pdata.config, sizeof(*pcfg));
}


static int tinno_flash_configure(struct tinno_flash_info *info, bool update)
{
	struct tinno_flash_config *pcfg = &info->config;
	struct nvc_torch_capability_query *pqry = &info->query;
	struct nvc_torch_flash_capabilities_v1	*pfcap = NULL;
	struct nvc_torch_torch_capabilities_v1	*ptcap = NULL;
	struct nvc_torch_timer_capabilities_v1	*ptmcap = NULL;
	struct nvc_torch_lumi_level_v1		*plvls = NULL;
	int i, j;

	/* number of leds enabled */
	pqry->flash_num = tinno_flash_LED_NUM;
	pqry->torch_num = tinno_flash_LED_NUM;

	pqry->version = NVC_TORCH_CAPABILITY_VER_1;
	pqry->led_attr = 0;
	for (i = 0; i < pqry->flash_num; i++) {
		pfcap = info->flash_cap[i];
		pfcap->version = NVC_TORCH_CAPABILITY_VER_1;
		pfcap->led_idx = i;
		pfcap->attribute = 0;
		pfcap->granularity = pcfg->led_config[i].granularity;
		pfcap->timeout_num = ARRAY_SIZE(tinno_flash_flash_timer);
		ptmcap = info->flash_timeouts[i];
		pfcap->timeout_off = (void *)ptmcap - (void *)pfcap;
		pfcap->flash_torch_ratio =
				pcfg->led_config[i].flash_torch_ratio;

		plvls = pcfg->led_config[i].lumi_levels;

		for (j = 0; j < pcfg->led_config[i].flash_levels; j++) {
			pfcap->levels[j].guidenum = plvls[j].guidenum;
			pfcap->levels[j].luminance = plvls[j].luminance;
			info->max_flash[i] = plvls[j - 1].guidenum;
			dev_dbg(info->dev, "%02d - %d\n",
				pfcap->levels[j].guidenum,
				pfcap->levels[j].luminance);
		}

		pfcap->numberoflevels = j;
		dev_dbg(info->dev,
			"%s flash#%d, attr: %x, levels: %d, g: %d, ratio: %d\n",
			__func__, pfcap->led_idx, pfcap->attribute,
			pfcap->numberoflevels, pfcap->granularity,
			pfcap->flash_torch_ratio);

		ptmcap->timeout_num = pfcap->timeout_num;
		for (j = 0; j < ptmcap->timeout_num; j++) {
			ptmcap->timeouts[j].timeout = tinno_flash_flash_timer[j];
			dev_dbg(info->dev, "t: %02d - %d uS\n", j,
				ptmcap->timeouts[j].timeout);
		}
	}

	for (i = 0; i < pqry->torch_num; i++) {
		ptcap = info->torch_cap[i];
		ptcap->version = NVC_TORCH_CAPABILITY_VER_1;
		ptcap->led_idx = i;
		ptcap->attribute = 0;
		ptcap->granularity = pcfg->led_config[i].granularity;
		ptcap->timeout_num = ARRAY_SIZE(tinno_flash_torch_timer);
		ptmcap = info->torch_timeouts[i];
		ptcap->timeout_off = (void *)ptmcap - (void *)ptcap;

		plvls = pcfg->led_config[i].lumi_levels;

		for (j = 0; j < pcfg->led_config[i].flash_levels; j++) {

			ptcap->levels[j].guidenum = plvls[j].guidenum;
			ptcap->levels[j].luminance = plvls[j].luminance;
			info->max_torch[i] = plvls[j - 1].guidenum;
			dev_dbg(info->dev, "%02d - %d\n",
				ptcap->levels[j].guidenum,
				ptcap->levels[j].luminance);
		}

		ptcap->numberoflevels = j;
		dev_dbg(info->dev, "torch#%d, attr: %x, levels: %d, g: %d\n",
			ptcap->led_idx, ptcap->attribute,
			ptcap->numberoflevels, ptcap->granularity);

		ptmcap->timeout_num = ptcap->timeout_num;
		for (j = 0; j < ptmcap->timeout_num; j++) {
			ptmcap->timeouts[j].timeout = tinno_flash_torch_timer[j];

			dev_dbg(info->dev, "t: %02d - %d uS\n", j,
				ptmcap->timeouts[j].timeout);
		}
	}

	return 0;
}
static int tinno_flash_power_off(struct tinno_flash_info *info)
{
	struct tinno_flash_power_rail *pw = &info->pwr_rail;
	int err = 0;

	if (!info->power_is_on)
		return 0;

	mutex_lock(&info->mutex);

	if (info->pdata && info->pdata->poweroff_callback)
		err = info->pdata->poweroff_callback(pw);

	if (IS_ERR_VALUE(err)) {
		mutex_unlock(&info->mutex);
		return err;
	}
	info->power_is_on = 0;

	mutex_unlock(&info->mutex);
	tinno_flash_edp_lowest(info);
	return err;
}

static int tinno_flash_power_on(struct tinno_flash_info *info)
{
	struct tinno_flash_power_rail *pw = &info->pwr_rail;
	int err = 0;

	if (info->power_is_on)
		return 0;

	mutex_lock(&info->mutex);

	if (info->pdata && info->pdata->poweron_callback)
		err = info->pdata->poweron_callback(pw);

	if (IS_ERR_VALUE(err))
		goto tinno_flash_poweron_callback_fail;

	info->power_is_on = 1;

	mutex_unlock(&info->mutex);

	tinno_flash_edp_lowest(info);

	return 0;

tinno_flash_poweron_callback_fail:
	mutex_unlock(&info->mutex);
	return err;
}

static int tinno_flash_power_set(struct tinno_flash_info *info, int pwr)
{
	int err = 0;

	if (pwr == info->pwr_state)
		return 0;

	switch (pwr) {
	case NVC_PWR_OFF:
		tinno_flash_edp_lowest(info);
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			(info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err = tinno_flash_power_off(info);
		break;
	case NVC_PWR_STDBY_OFF:
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			(info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err = tinno_flash_power_on(info);
		break;
	case NVC_PWR_STDBY:
		err = tinno_flash_power_on(info);
		break;

	case NVC_PWR_COMM:
	case NVC_PWR_ON:
		err = tinno_flash_power_on(info);
		break;

	default:
		err = -EINVAL;
		break;
	}

	if (err < 0) {
		dev_err(info->dev, "%s error\n", __func__);
		pwr = NVC_PWR_ERR;
	}
	info->pwr_state = pwr;
	if (err > 0)
		return 0;

	return err;
}

static inline int tinno_flash_power_user_set(struct tinno_flash_info *info, int pwr)
{
	int err = 0;

	if (!pwr || (pwr > NVC_PWR_ON))
		return 0;

	err = tinno_flash_power_set(info, pwr);
	if (info->pdata->cfg & NVC_CFG_NOERR)
		return 0;

	return err;
}

static int tinno_flash_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*miscdev = file->private_data;
	struct tinno_flash_info *info;

	info = container_of(miscdev, struct tinno_flash_info, miscdev);
	if (!info)
		return -ENODEV;

	if (atomic_xchg(&info->in_use, 1))
		return -EBUSY;

	file->private_data = info;
	dev_dbg(info->dev, "%s\n", __func__);

	return 0;
}

static int tinno_flash_release(struct inode *inode, struct file *file)
{
	struct tinno_flash_info *info = file->private_data;

	dev_dbg(info->dev, "%s\n", __func__);
	tinno_flash_power_set(info, NVC_PWR_OFF);
	tinno_flash_set_torch(info, 0);
	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&info->in_use, 0));

	return 0;
}

static int tinno_flash_get_levels(struct tinno_flash_info *info,
				   struct nvc_param *params,
				   bool flash_mode,
				   struct nvc_torch_set_level_v1 *plevels)
{
	struct nvc_torch_timer_capabilities_v1 *p_tm;
	u8 op_mode;

	if (copy_from_user(plevels, (const void __user *)params->p_value,
			   sizeof(*plevels))) {
		dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	if (flash_mode) {
		dev_dbg(info->dev, "%s FLASH_LEVEL: %d %d %d\n",
			__func__, plevels->ledmask,
			plevels->levels[0], plevels->levels[1]);
		p_tm = info->flash_timeouts[0];
		op_mode = MAXFLASH_MODE_FLASH;
	} else {
		dev_dbg(info->dev, "%s TORCH_LEVEL: %d %d %d\n",
			__func__, plevels->ledmask,
			plevels->levels[0], plevels->levels[1]);
		p_tm = info->torch_timeouts[0];
		op_mode = MAXFLASH_MODE_TORCH;
	}

	info->new_timer = plevels->timeout;
	info->op_mode = op_mode;

	dev_dbg(info->dev, "Return: %d - %d %d %d\n", info->op_mode,
		plevels->ledmask, plevels->levels[0], plevels->levels[1]);
	return 0;
}

static int tinno_flash_set_leds(struct tinno_flash_info *info,
		u8 mask, u8 curr1, u8 curr2)
{
	if (info->op_mode == MAXFLASH_MODE_FLASH) {
		info->flash_level = curr1;

		if (info->flash_level == 0) {
			tinno_flash_set_flash (info, 0);
		}
	} else if (info->op_mode == MAXFLASH_MODE_TORCH) {
		if (curr1 && mask) {
			tinno_flash_set_torch (info, 1);
		} else {
			tinno_flash_set_torch (info, 0);
		}
		info->torch_level = curr1;
	}
	return 0;
}

#if defined(POWER_CONTROL_ENABLE) && POWER_CONTROL_ENABLE
/* new version for low bat control */
static int tinno_flash_edp_req(struct tinno_flash_info *info,
		u8 mask, u8 *curr1, u8 *curr2)
{
	unsigned approved;
	unsigned new_state;
	int ret = 0;

	if (!info->edpc)
		return 0;

	new_state = info->edpc->num_states - 1;

	dev_dbg(info->dev, "%s: %d curr1 = %02x curr2 = %02x\n",
		__func__, mask, *curr1, *curr2);

	if (info->op_mode == MAXFLASH_MODE_FLASH && *curr1 && mask) {
		if (info->edp_state_flash >= 0) {
			new_state = info->edp_state_flash;
		} else
			new_state = 0;
	} else if (info->op_mode == MAXFLASH_MODE_TORCH && *curr1 && mask) {
		if (info->edp_state_torch >= 0) {
			new_state = info->edp_state_torch;
		} else
			new_state = 1;
	}

	BUG_ON(new_state >= info->edpc->num_states);

	dev_dbg(info->dev, "edp req: %d\n", new_state);
	ret = edp_update_client_request(info->edpc, new_state, &approved);
	if (ret || approved > new_state) {
		printk ("%s edp requirement not satisfied\n", __func__);
		info->edp_state = info->edpc->num_states - 1;
		*curr1 = 0;
	}
	else
		info->edp_state = approved;

	return 0;
}

static int
tinno_torch_check_lowbat(struct tinno_flash_info *info,
		u8 mask, u8 *curr1, u8 *curr2)
{
	tinno_flash_edp_req(info, mask, curr1, curr2);

	if (bat_is_safe(info, TINNO_FLASH_CURRENT))
		*curr1 = 0;

	if (*curr1 == 0)
		printk ("%s low bat condition\n", __func__);
	return 0;
}

static int
tinno_flash_edp_set_leds(struct tinno_flash_info *info,
		u8 mask, u8 curr1, u8 curr2)
{
	if (curr1 == 0)
		info->torch_phase = TORCH_PHASE_OFF;
	else {
		if (info->torch_phase == TORCH_PHASE_ON) {
			if (info->torch_flash_low_bat_state ==
				TORCH_FLASH_LOW_BAT_OFF)
					curr1 = 0;
		}
		else {
			tinno_torch_check_lowbat(info,
				mask, &curr1, &curr2);
		}
	}

	tinno_flash_set_leds(info, mask, curr1, curr2);
	if (info->op_mode == MAXFLASH_MODE_NONE)
		tinno_flash_edp_lowest(info);

	return 0;
}

static int
tinno_torch_edp_set_leds(struct tinno_flash_info *info,
		u8 mask, u8 curr1, u8 curr2)
{
	if (curr1) {
		info->torch_flash_low_bat_state =
			TORCH_FLASH_LOW_BAT_ON;
		info->torch_phase = TORCH_PHASE_ON;

		tinno_torch_check_lowbat(info, mask, &curr1, &curr2);

		if (curr1 == 0)
			info->torch_flash_low_bat_state =
				TORCH_FLASH_LOW_BAT_OFF;
	}

	if (curr1 > 0)
		tinno_flash_set_torch(info,  1);
	else
		tinno_flash_set_torch(info,  0);

	info->torch_level = curr1;

	if (info->op_mode == MAXFLASH_MODE_NONE)
		tinno_flash_edp_lowest(info);

	return 0;
}
#else
/* old version */
static int tinno_flash_edp_req(struct tinno_flash_info *info,
		u8 mask, u8 *curr1, u8 *curr2)
{
	unsigned approved;
	unsigned new_state;
	int ret = 0;

	if (!info->edpc)
		return 0;

	new_state = info->edpc->num_states - 1;

	dev_dbg(info->dev, "%s: %d curr1 = %02x curr2 = %02x\n",
		__func__, mask, *curr1, *curr2);

	if (info->op_mode == MAXFLASH_MODE_FLASH && *curr1 && mask) {
		if (info->edp_state_flash >= 0) {
			new_state = info->edp_state_flash;
		} else
			new_state = 0;
	} else if (info->op_mode == MAXFLASH_MODE_TORCH && *curr1 && mask) {
		if (info->edp_state_torch >= 0) {
			new_state = info->edp_state_torch;
		} else
			new_state = 1;
	}

	BUG_ON (new_state >= info->edpc->num_states);


	dev_dbg(info->dev, "edp req: %d\n", new_state);
	ret = edp_update_client_request(info->edpc, new_state, &approved);
	if (ret) {
		dev_err(info->dev, "E state transition failed\n");
		return ret;
	}

	if (approved > new_state) {
		printk ("%s edp requirement not satisfied\n", __func__);
		printk ("%s need furture process\n", __func__);

		*curr1 = 0;
	}

	info->edp_state = approved;

	return 0;
}

static int tinno_flash_edp_set_leds(struct tinno_flash_info *info,
		u8 mask, u8 curr1, u8 curr2)
{
	int err;

	err = tinno_flash_edp_req(info, mask, &curr1, &curr2);
	if (err)
		goto edp_set_leds_end;
	err = tinno_flash_set_leds(info, mask, curr1, curr2);
	if (!err && info->op_mode == MAXFLASH_MODE_NONE)
		tinno_flash_edp_lowest(info);

edp_set_leds_end:
	if (err)
		dev_err(info->dev, "%s ERROR: %d\n", __func__, err);
	return err;
}

static int tinno_torch_edp_set_leds(struct tinno_flash_info *info,
		u8 mask, u8 curr1, u8 curr2)
{
	int err;

	err = tinno_flash_edp_req(info, mask, &curr1, &curr2);
	if (err)
		goto edp_set_leds_end;
	if (curr1 > 0)
		tinno_flash_set_torch(info,  1);
	else
		tinno_flash_set_torch(info,  0);
	if (!err && info->op_mode == MAXFLASH_MODE_NONE)
		tinno_flash_edp_lowest(info);

edp_set_leds_end:
	if (err)
		dev_err(info->dev, "%s ERROR: %d\n", __func__, err);
	return err;
}

#endif
static int tinno_flash_strobe(struct tinno_flash_info *info, int t_on)
{
	return 0;
}
/* wangjian add attr for torch app*/

int torch_status = 0;
static struct platform_driver tinno_flash_driver;
void turnoff_torch(int on);
static ssize_t torch_value_show(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d", torch_status);
}

static ssize_t torch_value_store(struct device_driver *ddri, const char *buf, size_t count)
{
	if (sscanf(buf, "%u", &torch_status) != 1) {
			printk("[sgm3780]: Invalid values\n");
			return -EINVAL;
	}
	turnoff_torch(torch_status);
	return count;
}
static DRIVER_ATTR(value, 0644,torch_value_show, torch_value_store);

/* wangjian add attr for torch app end*/
static void tinno_flash_set_torch(struct tinno_flash_info *info, int on)
{
	if (info->gpio_en_torch > 0) {
		gpio_set_value(info->gpio_en_torch, on ? 1 : 0);
	}
/* wangjian add  for torch app*/
	if((1==torch_status)&&(0==on)){
		torch_status = on;
		printk("[sgm3780] reset torch_status to off\n");
	}
/* wangjian add  for torch app*/
}

void turnoff_torch(int on)
{
	if(info)
		tinno_flash_set_torch(info, on);
}
static void tinno_flash_set_flash(struct tinno_flash_info *info, int on)
{
	if (info->gpio_en_flash > 0) {
	/*gpio_set_value(info->gpio_en_flash, on ? 1 : 0); */
	}
}

static int tinno_flash_set_param(struct tinno_flash_info *info, long arg)
{
	struct nvc_param params;
	struct nvc_torch_set_level_v1 led_levels;
	u8 curr1;
	u8 curr2;
	u8 val;
	int err = 0;

	if (copy_from_user(
		&params, (const void __user *)arg, sizeof(struct nvc_param))) {
		dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	switch (params.param) {
	case NVC_PARAM_FLASH_LEVEL:
		tinno_flash_get_levels(info, &params, true, &led_levels);
		curr1 = led_levels.levels[0];
		curr2 = led_levels.levels[1];
		err = tinno_flash_edp_set_leds(info,
			led_levels.ledmask, curr1, curr2);
		break;
	case NVC_PARAM_TORCH_LEVEL:
		tinno_flash_get_levels(info, &params, false, &led_levels);
		curr1 = led_levels.levels[0];
		curr2 = led_levels.levels[1];
		err = tinno_torch_edp_set_leds(info,
			led_levels.ledmask, curr1, curr2);
		break;
	case NVC_PARAM_FLASH_PIN_STATE:
		if (copy_from_user(&val, (const void __user *)params.p_value,
			   sizeof(val))) {
			dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
			err = -EINVAL;
			break;
		}
		dev_dbg(info->dev, "%s FLASH_PIN_STATE: %d\n",
				__func__, val);
		err = tinno_flash_strobe(info, val);
		break;
	default:
		dev_err(info->dev, "%s unsupported parameter: %d\n",
				__func__, params.param);
		err = -EINVAL;
		break;
	}

	return err;
}

static int tinno_flash_get_param(struct tinno_flash_info *info, long arg)
{
	struct nvc_param params;
	struct nvc_torch_pin_state pinstate;
	const void *data_ptr = NULL;
	u32 data_size = 0;
	int err = 0;
	u8 reg;

	if (copy_from_user(&params, (const void __user *)arg,
			sizeof(struct nvc_param))) {
		dev_err(info->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	switch (params.param) {
	case NVC_PARAM_TORCH_QUERY:
		dev_dbg(info->dev, "%s QUERY\n", __func__);
		data_ptr = &info->query;
		data_size = sizeof(info->query);
		break;
	case NVC_PARAM_FLASH_EXT_CAPS:
		dev_dbg(info->dev, "%s EXT_FLASH_CAPS %d\n",
			__func__, params.variant);
		if (params.variant >= info->query.flash_num) {
			dev_err(info->dev, "%s unsupported flash index.\n",
				__func__);
			err = -EINVAL;
			break;
		}
		data_ptr = info->flash_cap[params.variant];
		data_size = info->flash_cap_size;
		break;
	case NVC_PARAM_TORCH_EXT_CAPS:
		dev_dbg(info->dev, "%s EXT_TORCH_CAPS %d\n",
			__func__, params.variant);
		if (params.variant >= info->query.torch_num) {
			dev_err(info->dev, "%s unsupported torch index.\n",
				__func__);
			err = -EINVAL;
			break;
		}
		data_ptr = info->torch_cap[params.variant];
		data_size = info->torch_cap_size;
		break;
	case NVC_PARAM_FLASH_LEVEL:
		if (params.variant >= info->query.flash_num) {
			dev_err(info->dev,
				"%s unsupported flash index.\n", __func__);
			err = -EINVAL;
			break;
		}
		reg = info->flash_level;
		data_ptr = &reg;
		data_size = sizeof(reg);
		dev_dbg(info->dev, "%s FLASH_LEVEL %d\n", __func__, reg);
		break;
	case NVC_PARAM_TORCH_LEVEL:
	if (params.variant >= info->query.torch_num) {
			dev_err(info->dev, "%s unsupported torch index.\n",
				__func__);
			err = -EINVAL;
			break;
		}
		reg = info->torch_level;
		data_ptr = &reg;
		data_size = sizeof(reg);
		dev_dbg(info->dev, "%s TORCH_LEVEL %d\n", __func__, reg);
		break;
	case NVC_PARAM_FLASH_PIN_STATE:
		pinstate = info->pdata->pinstate;
		if (info->op_mode != MAXFLASH_MODE_FLASH)
			pinstate.values ^= 0xffff;
		pinstate.values &= pinstate.mask;
		dev_dbg(info->dev, "%s FLASH_PIN_STATE: %x & %x\n",
				__func__, pinstate.mask, pinstate.values);

		if (info->flash_level == 0)
			pinstate.values = 0;

		data_ptr = &pinstate;
		data_size = sizeof(pinstate);
		break;
	default:
		dev_err(info->dev, "%s unsupported parameter: %d\n",
				__func__, params.param);
		err = -EINVAL;
	}

	dev_dbg(info->dev, "%s data size user %d vs local %d\n",
			__func__, params.sizeofvalue, data_size);
	if (!err && params.sizeofvalue < data_size) {
		dev_err(info->dev, "%s data size mismatch\n", __func__);
		err = -EINVAL;
	}

	if (!err && copy_to_user((void __user *)params.p_value,
			 data_ptr, data_size)) {
		dev_err(info->dev, "%s copy_to_user err line %d\n",
				__func__, __LINE__);
		err = -EFAULT;
	}

	return err;
}

static long tinno_flash_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tinno_flash_info *info = file->private_data;
	int pwr;
	int err = 0;

	switch (cmd) {
	case NVC_IOCTL_PARAM_WR:
		err = tinno_flash_set_param(info, arg);
		break;
	case NVC_IOCTL_PARAM_RD:
		err = tinno_flash_get_param(info, arg);
		break;
	case NVC_IOCTL_PWR_WR:
		/* This is a Guaranteed Level of Service (GLOS) call */
		pwr = (int)arg * 2;
		dev_dbg(info->dev, "%s PWR_WR: %d\n", __func__, pwr);
		err = tinno_flash_power_user_set(info, pwr);
		break;
	case NVC_IOCTL_PWR_RD:
		pwr = info->pwr_state / 2;
		dev_dbg(info->dev, "%s PWR_RD: %d\n", __func__, pwr);
		if (copy_to_user(
			(void __user *)arg, (const void *)&pwr, sizeof(pwr))) {
			dev_err(info->dev, "%s copy_to_user err line %d\n",
					__func__, __LINE__);
			err = -EFAULT;
		}
		break;
	default:
		dev_err(info->dev, "%s unsupported ioctl: %x\n",
				__func__, cmd);
		err = -EINVAL;
		break;
	}

	return err;
}

static const struct file_operations tinno_flash_fileops = {
	.owner = THIS_MODULE,
	.open = tinno_flash_open,
	.unlocked_ioctl = tinno_flash_ioctl,
	.release = tinno_flash_release,
};

enum hrtimer_restart tinno_flash_timeout_event(struct hrtimer *timer)
{
	struct tinno_flash_info *info;

	info = container_of(timer, struct tinno_flash_info, timeout);

	/* need to protect the timing */
	spin_lock(&info->lock);
	tinno_flash_set_flash(info, 0);
	spin_unlock(&info->lock);
	return HRTIMER_NORESTART;
}

static void start_timer(struct tinno_flash_info *info)
{
	int due_time = 70;

	if (info->pdata->timeout_ms > 0)
		due_time = info->pdata->timeout_ms;

	if (hrtimer_active(&info->timeout)) {
		hrtimer_set_expires(&info->timeout, ktime_set(due_time /
		MSEC_PER_SEC, (due_time % MSEC_PER_SEC) *
					NSEC_PER_MSEC));
		hrtimer_start_expires(&info->timeout, HRTIMER_MODE_REL);
	} else {
		hrtimer_start(&info->timeout, ktime_set(due_time /
		MSEC_PER_SEC, (due_time % MSEC_PER_SEC) *
				NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}
}
static int tinno_flash_probe(struct platform_device *dev)
{
	char dname[16];
	int ret,err;

	dev_info(&dev->dev, "%s\n", __func__);

	info = devm_kzalloc(&dev->dev, sizeof(*info) +
			tinno_flash_max_flash_cap_size +
			tinno_flash_max_torch_cap_size,
			GFP_KERNEL);
	if (info == NULL) {
		dev_err(&dev->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}

	info->dev = &dev->dev;

	if (dev->dev.platform_data) {
		info->pdata = dev->dev.platform_data;
		dev_dbg(&dev->dev, "pdata: %s\n", info->pdata->dev_name);
	} else
		dev_notice(&dev->dev, "%s NO platform data\n", __func__);

	info->pdata->flash_dev_cb = tinno_flash_dev_cb;
	spin_lock_init(&info->lock);
	tinno_flash_caps_layout(info);

	tinno_flash_update_config(info);

	info->op_mode = MAXFLASH_MODE_NONE;

	tinno_flash_configure(info, false);

	platform_set_drvdata(dev, info);

	mutex_init(&info->mutex);

	if (info->pdata->dev_name != NULL)
		strncpy(dname, info->pdata->dev_name, sizeof(dname));
	else
		strncpy(dname, "torch", sizeof(dname));

	if (info->pdata->num)
		snprintf(dname, sizeof(dname), "%s.%u",
				dname, info->pdata->num);

	info->miscdev.name = dname;
	info->miscdev.fops = &tinno_flash_fileops;
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&info->miscdev)) {
		dev_err(&dev->dev, "%s unable to register misc device %s\n",
				__func__, dname);
		goto free_mem;
	}
// wangjian add attr for torch app
	printk("[soc_driver]:driver_create_file_call_state \n");
	err = driver_create_file(&tinno_flash_driver.driver,&driver_attr_value);
	if (err) {
		printk( " failed to register tinno_flash_driver attributes\n");
		err = 0;
	}
// wangjian add attr for torch app end

	info->gpio_en_torch = info->gpio_en_flash = -1;
	if (info->pdata->gpio_en_torch > 0) {
		info->gpio_en_torch = info->pdata->gpio_en_torch;
		ret = gpio_request(info->gpio_en_torch, "EN_TORCH");
		if (ret) {
			goto free_misc_dev;
		}

		gpio_direction_output(info->gpio_en_torch, 0);

		tinno_flash_set_torch(info, 0);
	}
	if (info->pdata->gpio_en_flash > 0) {
		info->gpio_en_flash = info->pdata->gpio_en_flash;
		ret = gpio_request(info->gpio_en_flash, "EN_FLASH");
		if (ret) {
			goto free_misc_dev;
		}

		gpio_direction_output(info->gpio_en_flash, 0);

		tinno_flash_set_flash(info, 0);
	}
	tinno_flash_edp_register(info);

	hrtimer_init(&info->timeout, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	info->timeout.function = tinno_flash_timeout_event;

#if defined(POWER_CONTROL_ENABLE) && POWER_CONTROL_ENABLE
	info->psy = power_supply_get_by_name("battery");
	info->psy_data = psy_get_pdata();
#endif
	return 0;

free_misc_dev:
	misc_deregister(&info->miscdev);

free_mem:
	if (info)
		devm_kfree(&dev->dev, info);

	return -ENODEV;
}

static const struct platform_device_id flash_torch_id[] = {
	{ "tinno_flash", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, flash_torch_id);

static struct platform_driver tinno_flash_driver = {
	.driver = {
		.name = "flash_torch",
		.owner = THIS_MODULE,
	},
	.id_table = flash_torch_id,
	.probe = tinno_flash_probe,
	.remove = tinno_flash_remove,
};

module_platform_driver(tinno_flash_driver);

MODULE_DESCRIPTION("Tinno Flash/Torch driver");
MODULE_AUTHOR("Nvidia Corp");
MODULE_LICENSE("GPL v2");
