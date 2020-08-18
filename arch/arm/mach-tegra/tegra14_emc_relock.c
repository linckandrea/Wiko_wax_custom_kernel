/*
 * arch/arm/mach-tegra/tegra14_emc_relock.c
 *
 * Copyright (c) 2013, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/platform_data/tegra_emc.h>

#include "tegra14_emc.h"

static struct dentry *relock_root;
static u32 __last_dll_val;

/*
 * Relock the DLL. Makes sure this is a sensible operation before actually
 * trying to do so.
 */
static int relock_dll(void *data, u64 val)
{
	return emc_relock_dll(&__last_dll_val);
}

static int last_dll_val(void *data, u64 *val)
{
	*val = __last_dll_val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(relock_dll_fops, last_dll_val, relock_dll, "0x%llx\n");

static int tegra_emc_relock_init(void)
{
	relock_root = debugfs_create_dir("dll_relock", NULL);
	if (!relock_root)
		return -ENOMEM;

	if (!debugfs_create_file("relock", S_IWUSR|S_IRUGO, relock_root, NULL,
				 &relock_dll_fops))
		return -ENOMEM;
	return 0;
}
late_initcall(tegra_emc_relock_init);
