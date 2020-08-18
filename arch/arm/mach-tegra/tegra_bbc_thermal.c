/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/printk.h>
#include <linux/thermal.h>
#include <linux/nvshm_stats.h>

#define TZ_NAME_MAXLEN 15

struct bbc_thermal_private {
	u32 disabled_safe;
	const u32 *enabled;
	struct thermal_zone_device **tzds;
	int zones;
	struct notifier_block nb;
};

static struct bbc_thermal_private therm;

static int bbc_get_temp(struct thermal_zone_device *tzd, unsigned long *t)
{
	const u32 *tempCelcius = (const u32 *) tzd->devdata;

	/* Check that the thermal is enabled and temperature has been updated */
	if (!*therm.enabled || (*tempCelcius > 300))
		return -ENODATA;

	/* °C to m°C */
	*t = *tempCelcius * 1000;
	return 0;
}

static const struct thermal_zone_device_ops bbc_thermal_ops = {
	.get_temp = bbc_get_temp,
};

static void bbc_thermal_remove(void)
{
	int i;

	if (!therm.tzds)
		return;

	therm.enabled = &therm.disabled_safe;
	for (i = 0; i < therm.zones; i++)
		thermal_zone_device_unregister(therm.tzds[i]);

	kfree(therm.tzds);
	therm.tzds = NULL;
}

/*
 * This function is inline called from bbc_thermal_install() below, so pointers
 * are expected to be correct on enter.
 */
static inline int get_zone_data(struct nvshm_stats_iter *it, int index,
				char *name, u32 **temp)
{
	/* Empty name so we can check whether we got one at the end */
	name[0] = '\0';
	/* Set temperature pointer to NULL for the same reason */
	*temp = NULL;
	while (nvshm_stats_type(it) != NVSHM_STATS_END) {
		if (!strcmp(nvshm_stats_name(it), "name")) {
			if (nvshm_stats_type(it) != NVSHM_STATS_STRING) {
				pr_err("name has incorrect type: %d\n",
				       nvshm_stats_type(it));
				return -EINVAL;
			}

			strncpy(name, nvshm_stats_valueptr_string(it),
				TZ_NAME_MAXLEN);
			name[TZ_NAME_MAXLEN] = '\0';
		} else if (!strcmp(nvshm_stats_name(it), "tempCelcius")) {
			if (nvshm_stats_type(it) != NVSHM_STATS_UINT32) {
				pr_err("tempCelcius has incorrect type: %d\n",
				       nvshm_stats_type(it));
				return -EINVAL;
			}

			*temp = nvshm_stats_valueptr_uint32(it, 0);
		}

		if (nvshm_stats_next(it)) {
			pr_err("corruption detected in shared memory\n");
			return -EINVAL;
		}
	}

	if (*temp == NULL) {
		pr_err("tempCelcius not found\n");
		return -EINVAL;
	}

	/* Make sure we have a [unique] name */
	if (name[0] == '\0')
		sprintf(name, "BBC-therm%d", index);

	return 0;
}

static int bbc_thermal_install(void)
{
	struct nvshm_stats_iter it;
	unsigned int index;
	const u32 *enabled;
	int zones, ret = 0;

	if (therm.tzds) {
		pr_warn("BBC thermal already registered, unregistering\n");
		bbc_thermal_remove();
	}

	/* Get iterator for top structure */
	enabled = nvshm_stats_top("DrvTemperatureSysStats", &it);
	if (IS_ERR(enabled)) {
		pr_err("BBC thermal zones missing\n");
		return PTR_ERR(enabled);
	}

	/* Look for array of sensor data structures */
	while (nvshm_stats_type(&it) != NVSHM_STATS_END) {
		if (!strcmp(nvshm_stats_name(&it), "sensorStats"))
			break;

		if (nvshm_stats_next(&it)) {
			pr_err("corruption detected in shared memory\n");
			return -EINVAL;
		}
	}

	if (nvshm_stats_type(&it) != NVSHM_STATS_SUB) {
		pr_err("sensorStats not found or incorrect type: %d\n",
		       nvshm_stats_type(&it));
		return -EINVAL;
	}

	/* Parse sensors */
	zones = nvshm_stats_elems(&it);
	if (!zones) {
		pr_info("BBC does not report any temperatures\n");
		return 0;
	}

	pr_info("BBC can report temperatures from %d thermal zones\n", zones);
	therm.tzds = kmalloc(zones * sizeof(*therm.tzds), GFP_KERNEL);
	if (!therm.tzds) {
		pr_err("failed to allocate array of sensors\n");
		return -ENOMEM;
	}

	therm.zones = 0;
	for (index = 0; index < zones; index++) {
		struct nvshm_stats_iter sub_it;
		char name[TZ_NAME_MAXLEN + 1];
		u32 *temp_ptr;

		/* Get iterator to sensor data structure */
		nvshm_stats_sub(&it, index, &sub_it);
		/* Try to add a valid zone */
		ret = get_zone_data(&sub_it, index, name, &temp_ptr);
		if (ret < 0)
			goto failed;

		/* Ok we got it, let's register a new thermal zone */
		therm.tzds[therm.zones] = thermal_zone_device_register(name,
			0, 0, temp_ptr, &bbc_thermal_ops, NULL, 0, 0);
		if (IS_ERR(therm.tzds)) {
			pr_err("failed to register thermal zone #%d, abort\n",
			       index);
			ret = PTR_ERR(therm.tzds);
			goto failed;
		}

		therm.zones++;
	}

	therm.enabled = enabled;
	return 0;

failed:
	bbc_thermal_remove();
	return ret;
}

static int bbc_thermal_notify(struct notifier_block *self, unsigned long action,
			      void *user)
{
	switch (action) {
	case NVSHM_STATS_MODEM_UP:
		bbc_thermal_install();
		break;
	case NVSHM_STATS_MODEM_DOWN:
		bbc_thermal_remove();
		break;
	}
	return NOTIFY_OK;
}

void tegra_bbc_thermal_init(void)
{
	therm.enabled = &therm.disabled_safe;
	therm.nb.notifier_call = bbc_thermal_notify;
	nvshm_stats_register(&therm.nb);
}
