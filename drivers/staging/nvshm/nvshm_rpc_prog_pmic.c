/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/delay.h>
#include <mach/tegra_bb.h>
#include <mach/tegra_bbc_proxy.h>
#include <nvshm_rpc_utils.h>
#include <nvshm_rpc_dispatcher.h>
#include <nvshm_priv.h>

static struct device *bbc_proxy_dev;
static struct device *bbc_dev;

static enum rpc_accept_stat rpc_bbc_fusing_voltage(
	u32 version,
	struct nvshm_rpc_message *req,
	struct nvshm_rpc_message **resp)
{
	int ret;
	u32 enabled = 0;
	struct nvshm_rpc_datum_out req_data[] = {
		NVSHM_RPC_OUT_UINT(&enabled),
	};

	/* Decode request */
	if (nvshm_rpc_utils_decode_args(req, false, req_data,
					ARRAY_SIZE(req_data)) < 0)
		return RPC_GARBAGE_ARGS;

	/* Call tegra_bbc_proxy... */
	ret = tegra_bbc_proxy_bb_fusing_voltage(bbc_proxy_dev, enabled);

	/* Encode response */
	{
		struct nvshm_rpc_datum_in resp_data[] = {
			NVSHM_RPC_IN_SINT(ret),
		};

		*resp = nvshm_rpc_utils_prepare_response(req, RPC_SUCCESS,
				resp_data, ARRAY_SIZE(resp_data));
	}
	return *resp ? RPC_SUCCESS : RPC_SYSTEM_ERR;
}

static enum rpc_accept_stat rpc_bbc_reset(
	u32 version,
	struct nvshm_rpc_message *req,
	struct nvshm_rpc_message **resp)
{
	u32 crash_ind = 0;
	struct nvshm_handle *handle = nvshm_get_handle();
	struct nvshm_rpc_datum_out req_data[] = {
		NVSHM_RPC_OUT_UINT(&crash_ind),
	};

	/* Decode request */
	if (nvshm_rpc_utils_decode_args(req, false, req_data,
					ARRAY_SIZE(req_data)) < 0)
		return RPC_GARBAGE_ARGS;

	if (!crash_ind) {
		/* Reset must not generate coredump reception */
		pr_warn("Clear IPC mailbox before BBC reset.\n");
		*((int *)handle->mb_base_virt) =
			NVSHM_IPC_MESSAGE(NVSHM_IPC_BOOT_COLD_BOOT_IND);
	}

	/* Call tegra_bb... */
	pr_warn("BBC reset on BBC request.\n");
	tegra_bb_set_reset(bbc_dev, 1);
	udelay(100);
	tegra_bb_set_reset(bbc_dev, 0);

	return RPC_SUCCESS;
}

static nvshm_rpc_function_t procedures[] = {
	rpc_bbc_fusing_voltage,
	rpc_bbc_reset,
};

static struct nvshm_rpc_program program = {
	.version_min = 0,
	.version_max = 0,
	.procedures_size = ARRAY_SIZE(procedures),
	.procedures = procedures,
};

static int __init prog_pmic_init(void)
{
	bbc_proxy_dev = bus_find_device_by_name(&platform_bus_type, NULL,
					"tegra_bbc_proxy");
	if (!bbc_proxy_dev) {
		pr_err("failed to get proxy device pointer\n");
		return -ENXIO;
	}

	bbc_dev = bus_find_device_by_name(&platform_bus_type, NULL,
					"tegra_bb.0");
	if (!bbc_dev) {
		pr_err("failed to get bbc device pointer\n");
		return -ENXIO;
	}

	return nvshm_rpc_program_register(NVSHM_RPC_PROGRAM_PMIC, &program);
}

static void __exit prog_pmic_exit(void)
{
	nvshm_rpc_program_unregister(NVSHM_RPC_PROGRAM_PMIC);
}

module_init(prog_pmic_init);
module_exit(prog_pmic_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NVSHM RPC PMIC program");
