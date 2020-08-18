/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/clk.h>
#include <linux/syscore_ops.h>
#include <linux/platform_device.h>
#include <linux/nvshm_stats.h>
#include <mach/clk.h>

/*
 * This module stores the memory frequency in a BBC-shared area at each update.
 * When the system is going to suspend, the memory frequency is set to 0.
 */

static struct bbc_power_private {
	struct notifier_block memfreq_nb;
	struct notifier_block modem_nb;
	u32 *mem_freq_ptr;
	u32 *emc_floor_set_ptr;
} power;

void bbc_power_set_memfreq(u32 mem_freq)
{
	if (power.mem_freq_ptr)
		*power.mem_freq_ptr = mem_freq;
}

void bbc_power_emc_floor_set(u32 set)
{
	if (power.emc_floor_set_ptr)
		*power.emc_floor_set_ptr = set;
}

static int bbc_power_install(void)
{
	struct nvshm_stats_iter it;
	const u32 *enabled_ptr;

	/* Get iterator for top structure */
	enabled_ptr = nvshm_stats_top("DrvWakeSchStats", &it);
	if (IS_ERR(enabled_ptr)) {
		pr_err("BBC power stats missing\n");
		return PTR_ERR(enabled_ptr);
	}

	/* Look for memory frequency entry */
	while (nvshm_stats_type(&it) != NVSHM_STATS_END) {
		if (!strcmp(nvshm_stats_name(&it), "ap_memory_frequency")) {
			if (nvshm_stats_type(&it) != NVSHM_STATS_UINT32) {
				pr_err("ap_memory_frequency incorrect type: %d\n",
					nvshm_stats_type(&it));
				return -EINVAL;
			}
			power.mem_freq_ptr =
			   nvshm_stats_valueptr_uint32(&it, 0);
		}

		if (!strcmp(nvshm_stats_name(&it), "emc_floor_set")) {
			if (nvshm_stats_type(&it) != NVSHM_STATS_UINT32) {
				pr_err("emc_floor_set incorrect type: %d\n",
					nvshm_stats_type(&it));
				return -EINVAL;
			}
			power.emc_floor_set_ptr =
			   nvshm_stats_valueptr_uint32(&it, 0);
		}

		if (nvshm_stats_next(&it)) {
			pr_err("corruption detected in shared memory\n");
			return -EINVAL;
		}
	}

	if (!power.mem_freq_ptr) {
		pr_err("ap_memory_frequency not found\n");
		return -EINVAL;
	}

	if (!power.emc_floor_set_ptr) {
		pr_err("emc_floor_set not found\n");
		return -EINVAL;
	}

	return 0;
}

static int bbc_power_memfreq_notify(struct notifier_block *nb,
				    unsigned long rate, void *user)
{
	bbc_power_set_memfreq(rate);
	return NOTIFY_OK;
}

static int bbc_power_modem_notify(struct notifier_block *nb,
				  unsigned long action, void *user)
{
	switch (action) {
	case NVSHM_STATS_MODEM_UP:
		bbc_power_install();
		break;
	}
	return NOTIFY_OK;
}

static int tegra_bbc_power_suspend(void)
{
	bbc_power_set_memfreq(0);
	bbc_power_emc_floor_set(1);
	return 0;
}

static struct syscore_ops tegra_bbc_power_syscore_ops = {
	.suspend = tegra_bbc_power_suspend,
};

int tegra_bbc_power_init(struct platform_device *pdev)
{
	struct clk *clk;
	int ret = 0;

	/* Suspend */
	register_syscore_ops(&tegra_bbc_power_syscore_ops);
	/* Frequency changes */
	power.memfreq_nb.notifier_call = bbc_power_memfreq_notify;
	clk = clk_get(&pdev->dev, "emc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		pr_err("failed to get clock 'emc': ret=%d", ret);
		return ret;
	}

	ret = tegra_register_clk_rate_notifier(clk, &power.memfreq_nb);
	if (ret) {
		pr_err("failed to register with rate notifier: ret=%d", ret);
		return ret;
	}

	/* Modem up/down */
	power.modem_nb.notifier_call = bbc_power_modem_notify;
	nvshm_stats_register(&power.modem_nb);
	return 0;
}
