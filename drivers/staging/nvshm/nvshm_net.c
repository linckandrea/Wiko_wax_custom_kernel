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

#include <linux/device.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/semaphore.h>
#include "nvshm_types.h"
#include "nvshm_if.h"
#include "nvshm_priv.h"
#include "nvshm_iobuf.h"

#define MAX_XMIT_SIZE 1500

#define NVSHM_NETIF_PREFIX "wwan"

/* This structure holds the per network port information like
 * nvshm_iobuf queues, and the back reference to nvshm_channel */
struct nvshm_net_line {
	int use;
	int nvshm_chan;
	struct net_device_stats stats;

	/* iobuf queues for nvshm flow control support */
	struct nvshm_iobuf *q_head;
	struct nvshm_iobuf *q_tail;
	struct net_device *net;
	struct nvshm_channel *pchan; /* contains (struct net_device *)data */
	int errno;
	spinlock_t lock;
	int stop_tx; /* stop tx when >= 0 */
	int ipc_bb2ap;
};

/* rx_event() is called when a packet of data is received.
 * The receiver should consume all iobuf in the given list.
 */
void nvshm_netif_rx_event(struct nvshm_channel *chan,
			   struct nvshm_iobuf *iobuf)
{
	struct net_device *dev = (struct net_device *)chan->data;
	struct nvshm_net_line *priv = netdev_priv(dev);
	struct nvshm_iobuf *bb_iob, *bb_next, *ap_iob, *ap_next;
	unsigned char *src;	 /* AP address for BB source buffer */
	unsigned char *dst;	 /* AP address for skb */
	unsigned int datagram_len;	/* datagram total data length */
	struct sk_buff *skb;

	pr_debug("%s()\n", __func__);
	if (!priv) {
		pr_err("%s() no private info on iface!\n", __func__);
		return;
	}

	if (!iobuf) {
		pr_err("%s() null input buffer address\n", __func__);
		return;
	}

	ap_next = iobuf;
	bb_next = NVSHM_A2B(priv, iobuf);
	while (bb_next) {
		datagram_len = 0;
		ap_iob = ap_next;
		bb_iob = bb_next;
		while (bb_iob) {
			datagram_len += ap_iob->length;
			bb_iob = ap_iob->sg_next;
			ap_iob = NVSHM_B2A(priv, bb_iob);
		}
		/* construct the skb */
		skb = (struct sk_buff *) __netdev_alloc_skb(dev,
							    datagram_len,
							    GFP_KERNEL);
		if (!skb) {
			/* Out of memory - nothing to do except */
			/* free current iobufs and return */
			pr_err("%s: skb alloc failed!\n", __func__);
			priv->stats.rx_errors++;
			nvshm_iobuf_free_cluster(ap_next);
			return;
		}
		dst = skb_put(skb, datagram_len);

		ap_iob = ap_next;
		bb_iob = bb_next;
		bb_next = ap_next->next;
		ap_next = NVSHM_B2A(priv, bb_next);
		while (bb_iob) {
			src = NVSHM_B2A(priv, ap_iob->npduData)
			      + ap_iob->dataOffset;
			memcpy(dst, src, ap_iob->length);
			dst += ap_iob->length;
			bb_iob = ap_iob->sg_next;
			nvshm_iobuf_free(ap_iob);
			ap_iob = NVSHM_B2A(priv, bb_iob);
		}
		/* deliver skb to netif */
		skb->dev = dev;
		skb_reset_mac_header(skb);
		skb_reset_transport_header(skb);
		switch (skb->data[0] & 0xf0) {
		case 0x40:
			skb->protocol = htons(ETH_P_IP);
			break;
		case 0x60:
			skb->protocol = htons(ETH_P_IPV6);
			break;
		default:
			pr_err("%s() Non IP packet received!\n", __func__);
			priv->stats.rx_errors++;
			/* Drop packet */
			kfree_skb(skb);
			/* move to next datagram */
			continue;
		}
		skb->pkt_type = PACKET_HOST;
		skb->ip_summed = CHECKSUM_NONE;
		priv->stats.rx_packets++;
		priv->stats.rx_bytes += datagram_len;
		if (netif_rx_ni(skb) == NET_RX_DROP)
			pr_debug("%s() : dropped packet\n", __func__);
	}
}

/* error_event() is called when an error event is received */
void nvshm_netif_error_event(struct nvshm_channel *chan,
	enum nvshm_error_id error)
{
	unsigned long f;
	struct net_device *dev = (struct net_device *)chan->data;
	struct nvshm_net_line *priv = netdev_priv(dev);

	pr_debug("%s()\n", __func__);
	if (chan->data == NULL) {
		pr_err("%s() chan->data is null pointer\n", __func__);
		return;
	}

	priv->errno = error;

	spin_lock_irqsave(&priv->lock, f);
	priv->stats.tx_errors++;
	spin_unlock_irqrestore(&priv->lock, f);
	pr_err("%s() : error on nvshm net interface!\n", __func__);
}

/* start_tx() is called to restart the transmit */
void nvshm_netif_start_tx(struct nvshm_channel *chan)
{
	struct net_device *dev = (struct net_device *)chan->data;
	struct nvshm_net_line *priv = netdev_priv(dev);
	unsigned long f;

	pr_debug("%s()\n", __func__);

	if (!priv)
		return;

	spin_lock_irqsave(&priv->lock, f);
	priv->stop_tx--;
	/* Wake up queue */
	netif_wake_queue(dev);
	spin_unlock_irqrestore(&priv->lock, f);
}

static struct nvshm_if_operations nvshm_netif_ops = {
	.rx_event    = nvshm_netif_rx_event,	/* nvshm_queue.c */
	.error_event = nvshm_netif_error_event,	/* nvshm_iobuf.c */
	.start_tx    = nvshm_netif_start_tx,
};


/* called when ifconfig <if> up */
static int nvshm_netops_open(struct net_device *dev)
{
	unsigned long f;
	struct nvshm_net_line *priv = netdev_priv(dev);
	int ret = 0;

	pr_debug("%s()\n", __func__);
	if (!priv)
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, f);
	if (!priv->use) {
		priv->pchan = nvshm_open_channel(priv->nvshm_chan,
			&nvshm_netif_ops,
			dev);
		if (priv->pchan == NULL)
			ret = -EINVAL;
		priv->stop_tx = 0;
	}
	if (!ret)
		priv->use++;
	spin_unlock_irqrestore(&priv->lock, f);

	/* Start if queue */
	netif_start_queue(dev);
	return ret;
}

/* called when ifconfig <if> down */
static int nvshm_netops_close(struct net_device *dev)
{
	unsigned long f;
	struct nvshm_net_line *priv = netdev_priv(dev);

	pr_debug("%s()\n", __func__);
	if (!priv)
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, f);
	if (priv->use > 0)
		priv->use--;
	spin_unlock_irqrestore(&priv->lock, f);

	if (!priv->use) {
		/* Cleanup if data are still present in io queue */
		if (priv->q_head) {
			pr_debug("%s: still some data in queue!\n", __func__);
			nvshm_iobuf_free_cluster(
				(struct nvshm_iobuf *)priv->q_head);
			priv->q_head = priv->q_tail = NULL;
		}
		nvshm_close_channel(priv->pchan);
	}

	netif_stop_queue(dev);
	return 0;
}

static int nvshm_netops_xmit_frame(struct sk_buff *skb, struct net_device *dev)
{
	struct nvshm_net_line *priv = netdev_priv(dev);
	struct nvshm_iobuf *iob, *leaf = NULL, *list = NULL;
	int to_send = 0, remain;
	int len;
	char *data;
	unsigned long f;

	pr_debug("Transmit frame\n");
	pr_debug("%s()\n", __func__);
	if (!priv)
		return -EINVAL;

	/* Check first if TX is possible */
	spin_lock_irqsave(&priv->lock, f);
	if (priv->stop_tx > 0) {
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&priv->lock, f);
		return NETDEV_TX_BUSY;
	}
	spin_unlock_irqrestore(&priv->lock, f);

	len = skb->len;
	data = skb->data;

	/* write a frame to an nvshm channel */
	pr_debug("len=%d\n", len);

	/* write data from skb (data,len) to net_device dev */
	remain = len;
	while (remain) {
		pr_debug("remain=%d\n", remain);
		to_send = remain < dev->mtu ? remain : dev->mtu;
		iob = nvshm_iobuf_alloc(priv->pchan, to_send);
		if (!iob) {
			pr_warn("%s iobuf alloc failed\n", __func__);
			netif_stop_queue(dev);
			if (list)
				nvshm_iobuf_free_cluster(list);
			return NETDEV_TX_BUSY; /* was return -ENOMEM; */
		}

		iob->length = to_send;
		remain -= to_send;

		memcpy(NVSHM_B2A(priv, iob->npduData + iob->dataOffset),
		       data,
		       to_send);

		data += to_send;

		if (!list)
			leaf = list = iob;
		else {
			leaf->sg_next = NVSHM_A2B(priv, iob);
			leaf = iob;
		}
	}
	if (nvshm_write(priv->pchan, list)) {
		/* no more transmit possible - stop queue on next TX */
		pr_warning("%s rate limit hit on channel %d\n",
			   __func__, priv->nvshm_chan);
		spin_lock_irqsave(&priv->lock, f);
		priv->stop_tx++;
		spin_unlock_irqrestore(&priv->lock, f);
	}

	/* successfully written len data bytes */
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += len;

	pr_debug("packets=%ld, tx_bytes=%ld\n", priv->stats.tx_packets,
			priv->stats.tx_bytes);

	/* free skb now as nvshm is a ref on it now */
	kfree_skb(skb);

	return NETDEV_TX_OK;
}

static int nvshm_netops_change_mtu(struct net_device *dev, int new_mtu)
{
	struct nvshm_net_line *priv = netdev_priv(dev);

	pr_debug("%s()\n", __func__);
	if (!priv)
		return -EINVAL;

	if (new_mtu <= MAX_XMIT_SIZE)
		dev->mtu = new_mtu;
	else
		return -EINVAL;

	return 0;
}

struct net_device_stats *nvshm_netops_get_stats(struct net_device *dev)
{
	struct nvshm_net_line *priv = netdev_priv(dev);

	pr_debug("%s()\n", __func__);
	if (priv)
		return &priv->stats;
	else
		return NULL;
}

static void nvshm_netops_tx_timeout(struct net_device *dev)
{
	unsigned long f;
	struct nvshm_net_line *priv = netdev_priv(dev);

	pr_debug("%s()\n", __func__);
	if (!priv)
		return;

	spin_lock_irqsave(&priv->lock, f);
	priv->stats.tx_errors++;
	spin_unlock_irqrestore(&priv->lock, f);
	netif_wake_queue(dev);
}

static const struct net_device_ops nvshm_netdev_ops = {
	.ndo_open   = nvshm_netops_open,
	.ndo_stop   = nvshm_netops_close,
	.ndo_start_xmit = nvshm_netops_xmit_frame,
	.ndo_get_stats  = nvshm_netops_get_stats,
	.ndo_change_mtu = nvshm_netops_change_mtu,
	.ndo_tx_timeout = nvshm_netops_tx_timeout,
};

static void nvshm_nwif_init_dev(struct net_device *dev)
{
	dev->netdev_ops = &nvshm_netdev_ops;
	dev->mtu = MAX_XMIT_SIZE;
	dev->type = ARPHRD_NONE;
	/* No hardware address */
	dev->hard_header_len = 0;
	/* Should be tuned as soon as framer allow multiple frames */
	dev->tx_queue_len    = 10;
	/* No hardware address */
	dev->addr_len        = 0;
	dev->watchdog_timeo = HZ;
	dev->flags |= IFF_POINTOPOINT | IFF_NOARP;
}

struct net_device *nvshm_net_create(struct nvshm_handle *handle, int chan,
	int index)
{
	struct net_device *dev;
	struct nvshm_net_line *line;
	int sts = 0;

	pr_debug("Register %s%d on [%d]\n",
		NVSHM_NETIF_PREFIX, index, chan);

	dev = alloc_netdev(sizeof(struct nvshm_net_line),
		NVSHM_NETIF_PREFIX"%d", nvshm_nwif_init_dev);
	if (!dev)
		return NULL;

	dev->base_addr = index;
	line = netdev_priv(dev);
	line->net = dev;
	line->ipc_bb2ap = handle->ipc_bb2ap;
	line->nvshm_chan = chan;
	spin_lock_init(&line->lock);

	sts = register_netdev(dev);
	if (sts) {
		pr_err("Error %i registering %s%d device\n", sts,
			NVSHM_NETIF_PREFIX, index);
		free_netdev(dev);
		return NULL;
	}
	return dev;
}

int nvshm_net_remove(struct net_device *dev)
{
	struct nvshm_net_line *line;

	if (!dev)
		return -EINVAL;

	line = netdev_priv(dev);
	if (!line)
		return -EINVAL;

	if (!line->net)
		return -EINVAL;

	pr_debug("%s free %s%d on [%d]\n", __func__, NVSHM_NETIF_PREFIX,
		(int)line->net->base_addr, line->nvshm_chan);

	unregister_netdev(line->net);
	free_netdev(line->net);
	line->net = NULL;
	return 0;
}

int nvshm_net_init(struct nvshm_handle *handle)
{
	int chan;
	int net_count;

	pr_debug("%s()\n", __func__);

	net_count = 0;
	for (chan = 0; chan < handle->chan_count; chan++) {
		if (handle->chan[chan].map.type == NVSHM_CHAN_NET) {
			handle->chan[chan].data =
			nvshm_net_create(handle, chan, net_count);
			net_count++;
		}
	}
	return 0;
}

void nvshm_net_cleanup(struct nvshm_handle *handle)
{
	int chan;

	pr_debug("%s()\n", __func__);

	for (chan = 0; chan < handle->chan_count; chan++) {
		if (handle->chan[chan].map.type == NVSHM_CHAN_NET)
			nvshm_net_remove(handle->chan[chan].data);
	}
}
