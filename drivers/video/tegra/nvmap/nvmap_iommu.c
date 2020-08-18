/*
 * IOMMU backend support for NVMAP
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/slab.h>
#include <linux/platform_device.h>
#include <mach/iomap.h>

#include "nvmap_priv.h"

struct tegra_iommu_area *tegra_iommu_create_vm(struct device *dev,
			       dma_addr_t req, size_t size, pgprot_t prot)
{
	struct tegra_iommu_area *area;
	dma_addr_t iova;

	area = kzalloc(sizeof(*area), GFP_KERNEL);
	if (!area)
		return NULL;

	if (!req)
		iova = dma_iova_alloc(dev, size);
	else
		iova = dma_iova_alloc_at(dev, &req, size);

	if (iova == DMA_ERROR_CODE)
		goto err_out;
	area->iovm_start = iova;
	area->iovm_length = size;
	area->pgprot = prot;
	area->dev = dev;
	area->flags |= TEGRA_IOMMU_IOVA;
	return area;

err_out:
	kfree(area);
	return NULL;
}

void tegra_iommu_free_vm(struct tegra_iommu_area *area)
{
	DEFINE_DMA_ATTRS(attrs);
	u32 flags = TEGRA_IOMMU_IOVA | TEGRA_IOMMU_MAP;

	if ((area->flags & flags) == flags) {
		dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
		dma_unmap_single_attrs(area->dev, area->iovm_start,
				       area->iovm_length,
				       0, &attrs);
	} else if ((area->flags & flags) == TEGRA_IOMMU_IOVA) {
		pr_err("%s(): area flags: %08x != %08x\n",
		       __func__, area->flags, flags);
		dma_iova_free(area->dev, area->iovm_start, area->iovm_length);
	}

	kfree(area);
}

void tegra_iommu_zap_vm(struct tegra_iommu_area *area)
{
	DEFINE_DMA_ATTRS(attrs);
	u32 flags = TEGRA_IOMMU_IOVA | TEGRA_IOMMU_MAP;

	if (WARN_ON((area->flags & flags) != flags)) {
		pr_err("%s(): area flags: %08x != %08x\n",
		       __func__, area->flags, flags);
		return;
	}

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_SKIP_FREE_IOVA, &attrs);
	dma_unmap_single_attrs(area->dev, area->iovm_start, area->iovm_length,
			       0, &attrs);
	area->flags &= ~TEGRA_IOMMU_MAP;
}

#ifdef CONFIG_PLATFORM_ENABLE_IOMMU

static inline int tegra_iommu_create_map(struct device *dev)
{
	return 0;
}

static inline void tegra_iommu_delete_map(struct device *dev)
{
}

#else

static int tegra_iommu_create_map(struct device *dev)
{
	int err;
	struct dma_iommu_mapping *map;

	map = arm_iommu_create_mapping(&platform_bus_type,
				       TEGRA_IOMMU_BASE, TEGRA_IOMMU_SIZE, 0);
	if (IS_ERR(map))
		return PTR_ERR(map);

	err = arm_iommu_attach_device(dev, map);
	if (err) {
		arm_iommu_release_mapping(map);
		return err;
	}
	return 0;
}

static void tegra_iommu_delete_map(struct device *dev)
{
	arm_iommu_release_mapping(dev->archdata.mapping);
}

#endif

struct tegra_iommu_client *tegra_iommu_alloc_client(struct device *dev)
{
	struct tegra_iommu_client *client;

	if (WARN_ON(!dev))
		return NULL;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return NULL;

	if (tegra_iommu_create_map(dev)) {
		kfree(client);
		return NULL;
	}

	client->dev = dev;

	return client;
}

void tegra_iommu_free_client(struct tegra_iommu_client *client)
{
	if (WARN_ON(!client))
		return;
	tegra_iommu_delete_map(client->dev);
	kfree(client);
}
