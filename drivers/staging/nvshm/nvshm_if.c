/*
 * Copyright (c) 2012-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/debugfs.h>

#include "nvshm_types.h"
#include "nvshm_if.h"
#include "nvshm_priv.h"
#include "nvshm_ipc.h"
#include "nvshm_queue.h"
#include "nvshm_iobuf.h"

struct nvshm_channel *nvshm_open_channel(int chan,
					 struct nvshm_if_operations *ops,
					 void *interface_data)
{
	unsigned long f;
	struct nvshm_handle *handle = nvshm_get_handle();

	pr_debug("%s(%d)\n", __func__, chan);
	spin_lock_irqsave(&handle->lock, f);
	if (handle->chan[chan].ops) {
		spin_unlock_irqrestore(&handle->lock, f);
		pr_err("%s: already registered on chan %d\n", __func__, chan);
		return NULL;
	}

	handle->chan[chan].ops = ops;
	handle->chan[chan].data = interface_data;
	spin_unlock_irqrestore(&handle->lock, f);
	return &handle->chan[chan];
}

void nvshm_close_channel(struct nvshm_channel *handle)
{
	unsigned long f;
	struct nvshm_handle *priv = nvshm_get_handle();

	/* we cannot flush the work queue here as the call to
	   nvshm_close_channel() is made from cleanup_interfaces(),
	   which executes from the context of the work queue

	   additionally, flushing the work queue is unnecessary here
	   as the main work queue handler always checks the state of
	   the IPC */

	spin_lock_irqsave(&priv->lock, f);
	/* Clear ops but not data as it may be used for cleanup */
	priv->chan[handle->index].ops = NULL;
	spin_unlock_irqrestore(&priv->lock, f);
}

int nvshm_write(struct nvshm_channel *handle, struct nvshm_iobuf *iob)
{
	unsigned long f;
	struct nvshm_handle *priv = nvshm_get_handle();
	struct nvshm_iobuf *list, *leaf;
	int count = 0, ret = 0;

	spin_lock_irqsave(&priv->lock, f);
	if (!priv->chan[handle->index].ops) {
		pr_err("%s: channel not mapped\n", __func__);
		spin_unlock_irqrestore(&priv->lock, f);
		return -EINVAL;
	}

	list = iob;
	while (list) {
		count++;
		list->chan = handle->index;
		leaf = list->sg_next;
		while (leaf) {
			count++;
			leaf = NVSHM_B2A(priv, leaf);
			leaf->chan = handle->index;
			leaf = leaf->sg_next;
		}
		list = list->next;
		if (list)
			list = NVSHM_B2A(priv, list);
	}
	priv->chan[handle->index].rate_counter -= count;
	if (priv->chan[handle->index].rate_counter < 0) {
		priv->chan[handle->index].xoff = 1;
		pr_warn("%s: rate limit hit on chan %d\n", __func__,
			handle->index);
		ret = 1;
	}

	iob->qnext = NULL;
	nvshm_queue_put(priv, iob);
	nvshm_generate_ipc(priv);
	spin_unlock_irqrestore(&priv->lock, f);
	return ret;
}

void nvshm_start_tx(struct nvshm_channel *handle)
{
	if (handle->ops)
		handle->ops->start_tx(handle);
}

void nvshm_error_event(struct nvshm_channel *handle,
		       enum nvshm_error_id error)
{
	if (handle->ops)
		handle->ops->error_event(handle, error);
}

