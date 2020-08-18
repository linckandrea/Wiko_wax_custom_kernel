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

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <linux/srcu.h>
#include <linux/skbuff.h>

#include "nvshm_types.h"
#include "nvshm_if.h"
#include "nvshm_priv.h"
#include "nvshm_iobuf.h"

/* NVSHM interface */

#define MAX_OUTPUT_SIZE 1500

#define NVSHM_TTY_UP (1)

/*
 * This structure hold per tty line information like
 * nvshm_iobuf queues and back reference to nvshm channel/driver
 */

struct nvshm_tty_line {
	int nvshm_chan; /* nvshm channel */
	struct tty_port port;
	/* iobuf queues for nvshm flow control support */
	struct nvshm_iobuf *io_queue_head;
	struct nvshm_iobuf *io_queue_tail;
	struct nvshm_channel *pchan;
	int errno;
	spinlock_t lock;
	int ipc_bb2ap;
};

struct nvshm_tty_device {
	int up;
	struct tty_driver *tty_driver;
	int nlines;
	struct workqueue_struct *tty_wq;
	struct work_struct tty_worker;
	/* line[] array of up to NVSHM_MAX_CHANNELS */
	struct nvshm_tty_line *line;
};

static struct nvshm_tty_device tty_dev;
static struct srcu_struct tty_dev_lock;
static bool tty_dev_lock_initialized;

static void nvshm_tty_rx_rewrite_line(struct nvshm_tty_line *line)
{
	struct nvshm_iobuf *ioblist;
	struct tty_struct *tty;
	int len;

	if (!line)
		return;
	tty = tty_port_tty_get(&line->port);
	if (!tty)
		return;

	spin_lock(&line->lock);
	while (line->io_queue_head) {
		ioblist = line->io_queue_head;
		spin_unlock(&line->lock);
		len = tty_insert_flip_string(tty,
			NVSHM_B2A(line, ioblist->npduData)
				+ ioblist->dataOffset,
			ioblist->length);
		tty_flip_buffer_push(tty);
		if (len < ioblist->length) {
			ioblist->dataOffset += len;
			ioblist->length -= len;
			tty_kref_put(tty);
			return;
		}
		spin_lock(&line->lock);
		if (ioblist->sg_next) {
			/* Propagate ->next to the sg_next fragment
			   do not forget to move tail also */
			if (line->io_queue_head != line->io_queue_tail) {
				line->io_queue_head =
					NVSHM_B2A(line, ioblist->sg_next);
				line->io_queue_head->next = ioblist->next;
			} else {
				line->io_queue_head =
					NVSHM_B2A(line, ioblist->sg_next);
				line->io_queue_tail = line->io_queue_head;
				BUG_ON(ioblist->next);
			}
		} else {
			if (ioblist->next) {
				if (line->io_queue_head !=
					line->io_queue_tail) {
					line->io_queue_head =
						NVSHM_B2A(line, ioblist->next);
				} else {
					line->io_queue_head =
						NVSHM_B2A(line, ioblist->next);
					line->io_queue_tail =
						line->io_queue_head;
				}
			} else {
				line->io_queue_tail = NULL;
				line->io_queue_head = NULL;
			}
		}
		nvshm_iobuf_free((struct nvshm_iobuf *)ioblist);
	}
	spin_unlock(&line->lock);
	tty_kref_put(tty);
}

/* Called in a workqueue from rx_event and when tty is unthrottled */
static void nvshm_tty_rx_rewrite(struct work_struct *work)
{
	struct nvshm_tty_device *ttydev =
		container_of(work, struct nvshm_tty_device, tty_worker);
	struct nvshm_tty_line *line;
	int idx, i;

	idx = srcu_read_lock(&tty_dev_lock);
	line = srcu_dereference(ttydev->line, &tty_dev_lock);
	if (line) {
		for (i = 0; i < ttydev->nlines; i++)
			nvshm_tty_rx_rewrite_line(&line[i]);
	}

	srcu_read_unlock(&tty_dev_lock, idx);
}

/*
 * nvshm_tty_rx_event()
 * NVSHM has received data insert it in FIFO and wake up
 * tty writer workqueue
 */
void nvshm_tty_rx_event(struct nvshm_channel *chan, struct nvshm_iobuf *iob)
{
	struct nvshm_tty_line *line = chan->data;

	/* line can be null if TTY install failed or not executed yet */
	if (!line) {
		nvshm_iobuf_free_cluster(iob);
		return;
	}

	spin_lock(&line->lock);

	/* Queue into FIFO */
	if (line->io_queue_tail)
		line->io_queue_tail->next = NVSHM_A2B(line, iob);
	else {
		if (line->io_queue_head)
			line->io_queue_head->next = NVSHM_A2B(line, iob);
		else
			line->io_queue_head = iob;
	}
	line->io_queue_tail = iob;
	spin_unlock(&line->lock);
	queue_work(tty_dev.tty_wq, &tty_dev.tty_worker);
}

void nvshm_tty_error_event(struct nvshm_channel *chan,
			   enum nvshm_error_id error)
{
	struct nvshm_tty_line *line = chan->data;
	struct tty_struct *tty;

	tty = tty_port_tty_get(&line->port);
	pr_debug("%s\n", __func__);
	tty_dev.line[tty->index].errno = error;
	tty_hangup(tty);
	tty_kref_put(tty);
}

void nvshm_tty_start_tx(struct nvshm_channel *chan)
{
	struct nvshm_tty_line *line = chan->data;
	struct tty_struct *tty;

	tty = tty_port_tty_get(&line->port);

	pr_debug("%s\n", __func__);
	tty_unthrottle(tty);
	tty_kref_put(tty);
}

static struct nvshm_if_operations nvshm_tty_ops = {
	.rx_event = nvshm_tty_rx_event,
	.error_event = nvshm_tty_error_event,
	.start_tx = nvshm_tty_start_tx
};

/* TTY interface */

static int nvshm_tty_open(struct tty_struct *tty, struct file *f)
{
	struct nvshm_tty_line *line;
	int idx, ret = -EIO;

	idx = srcu_read_lock(&tty_dev_lock);
	line = srcu_dereference(tty->driver_data, &tty_dev_lock);
	if (line && tty_dev.up)
		ret = tty_port_open(&line->port, tty, f);

	srcu_read_unlock(&tty_dev_lock, idx);
	return ret;
}

static void nvshm_tty_close(struct tty_struct *tty, struct file *f)
{
	struct nvshm_tty_line *line;
	int idx;

	idx = srcu_read_lock(&tty_dev_lock);
	line = srcu_dereference(tty->driver_data, &tty_dev_lock);
	if (line && tty_dev.up)
		tty_port_close(&line->port, tty, f);
	srcu_read_unlock(&tty_dev_lock, idx);
}

static void nvshm_tty_hangup(struct tty_struct *tty)
{
	struct nvshm_tty_line *line;
	int idx;

	idx = srcu_read_lock(&tty_dev_lock);
	line = srcu_dereference(tty->driver_data, &tty_dev_lock);
	if (line)
		tty_port_hangup(&line->port);

	srcu_read_unlock(&tty_dev_lock, idx);
}


static int nvshm_tty_write_room(struct tty_struct *tty)
{
	return MAX_OUTPUT_SIZE;
}

static int nvshm_tty_write(struct tty_struct *tty, const unsigned char *buf,
			   int len)
{
	struct nvshm_iobuf *iob, *leaf = NULL, *ioblist = NULL;
	int to_send = 0, remain, ret;
	struct nvshm_tty_line *line;
	int idx, rc = len;

	idx = srcu_read_lock(&tty_dev_lock);
	if (!tty_dev.up) {
		rc = -EIO;
		goto err;
	}

	remain = len;
	line = srcu_dereference(tty->driver_data, &tty_dev_lock);
	if (!line) {
		rc = -EIO;
		goto err;
	}
	while (remain) {
		to_send = remain < MAX_OUTPUT_SIZE ? remain : MAX_OUTPUT_SIZE;
		iob = nvshm_iobuf_alloc(line->pchan, to_send);
		if (!iob) {
			if (line->errno) {
				pr_err("%s iobuf alloc failed\n", __func__);
				/* Stop processing until modem has been reset */
				tty_dev.up = 0;
				if (ioblist)
					nvshm_iobuf_free_cluster(ioblist);
				rc = -ENOMEM;
			} else {
				pr_err("%s: Xoff condition\n", __func__);
				tty_throttle(tty);
				rc = 0;
			}

			goto err;
		}

		iob->length = to_send;
		iob->chan = line->pchan->index;
		remain -= to_send;
		memcpy(NVSHM_B2A(line, iob->npduData + iob->dataOffset),
		       buf,
		       to_send);
		buf += to_send;

		if (!ioblist) {
			leaf = ioblist = iob;
		} else {
			leaf->sg_next = NVSHM_A2B(line, iob);
			leaf = iob;
		}
	}
	ret = nvshm_write(line->pchan, ioblist);

	if (ret == 1)
		tty_throttle(tty);

err:
	srcu_read_unlock(&tty_dev_lock, idx);
	return rc;
}

static void nvshm_tty_unthrottle(struct tty_struct *tty)
{
	pr_debug("%s\n", __func__);

	if (tty_dev.up)
		queue_work(tty_dev.tty_wq, &tty_dev.tty_worker);
}

static void nvshm_tty_dtr_rts(struct tty_port *tport, int onoff)
{
	pr_debug("%s\n", __func__);
}

static int nvshm_tty_carrier_raised(struct tty_port *tport)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int  nvshm_tty_activate(struct tty_port *tport, struct tty_struct *tty)
{
	struct nvshm_tty_line *line;

	/* Set TTY flags */
	set_bit(TTY_IO_ERROR, &tty->flags);
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	tty->low_latency = 1;

	line = &tty_dev.line[tty->index];
	pr_debug("%s line %d\n", __func__, tty->index);
	line->pchan = nvshm_open_channel(line->nvshm_chan,
				   &nvshm_tty_ops,
				   line);
	if (!line->pchan) {
		pr_err("%s fail to open SHM chan\n", __func__);
		return -EIO;
	}
	clear_bit(TTY_IO_ERROR, &tty->flags);
	return 0;
}

static void  nvshm_tty_shutdown(struct tty_port *tport)
{
	struct nvshm_tty_line *line, *tty_line;
	int idx;

	idx = srcu_read_lock(&tty_dev_lock);
	tty_line = container_of(tport, struct nvshm_tty_line, port);
	line = srcu_dereference(tty_line, &tty_dev_lock);
	if (line) {
		pr_debug("%s\n", __func__);
		nvshm_close_channel(line->pchan);
		line->pchan = NULL;
	}

	srcu_read_unlock(&tty_dev_lock, idx);
}

static int nvshm_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct nvshm_tty_line *line = &tty_dev.line[tty->index];
	int ret = tty_standard_install(driver, tty);

	pr_debug("%s\n", __func__);
	if (ret == 0)
		tty->driver_data = line;
	return ret;
}

static const struct tty_operations nvshm_tty_ttyops = {
	.open = nvshm_tty_open,
	.close = nvshm_tty_close,
	.hangup = nvshm_tty_hangup,
	.write = nvshm_tty_write,
	.write_room = nvshm_tty_write_room,
	.unthrottle = nvshm_tty_unthrottle,
	.install = nvshm_tty_install,
};

static const struct tty_port_operations nvshm_tty_port_ops = {
	.dtr_rts = nvshm_tty_dtr_rts,
	.carrier_raised = nvshm_tty_carrier_raised,
	.shutdown = nvshm_tty_shutdown,
	.activate = nvshm_tty_activate,
};

int nvshm_tty_init(struct nvshm_handle *handle)
{
	struct nvshm_tty_line *line;
	int ret, chan, count;

	pr_debug("%s\n", __func__);

	/*
	 * This main lock must always remain valid, else we may crash on a race
	 * between cleanup and open/close.
	 */
	if (!tty_dev_lock_initialized) {
		init_srcu_struct(&tty_dev_lock);
		tty_dev_lock_initialized = true;
	}

	memset(&tty_dev, 0, sizeof(tty_dev));

	/* calculate number of lines for tty */
	count = 0;
	for (chan = 0; chan < handle->chan_count; chan++) {
		if ((handle->chan[chan].map.type == NVSHM_CHAN_TTY)
			|| (handle->chan[chan].map.type == NVSHM_CHAN_LOG)) {
			count++;
		}
	}
	tty_dev.nlines = count;
	tty_dev.tty_wq = create_singlethread_workqueue("NVSHM_tty");
	INIT_WORK(&tty_dev.tty_worker, nvshm_tty_rx_rewrite);

	tty_dev.tty_driver = alloc_tty_driver(count);

	if (tty_dev.tty_driver == NULL)
		return -ENOMEM;

	tty_dev.tty_driver->owner = THIS_MODULE;
	tty_dev.tty_driver->driver_name = "nvshm_tty";
	tty_dev.tty_driver->name = "ttySHM";
	tty_dev.tty_driver->major = 0;
	tty_dev.tty_driver->minor_start = 0;
	tty_dev.tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	tty_dev.tty_driver->subtype = SERIAL_TYPE_NORMAL;
	tty_dev.tty_driver->init_termios = tty_std_termios;
	tty_dev.tty_driver->init_termios.c_iflag = 0;
	tty_dev.tty_driver->init_termios.c_oflag = 0;
	tty_dev.tty_driver->init_termios.c_cflag =
		B115200 | CS8 | CREAD | CLOCAL;
	tty_dev.tty_driver->init_termios.c_lflag = 0;
	tty_dev.tty_driver->flags =
		TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW |
		TTY_DRIVER_DYNAMIC_DEV;

	tty_set_operations(tty_dev.tty_driver, &nvshm_tty_ttyops);

	ret = tty_register_driver(tty_dev.tty_driver);

	/* allocate line[] array */
	line = kzalloc(sizeof(struct nvshm_tty_line) * count, GFP_KERNEL);
	tty_dev.line = line;
	if (!line) {
		tty_unregister_driver(tty_dev.tty_driver);
		put_tty_driver(tty_dev.tty_driver);
		return -ENOMEM;
	}

	/* map nvshm channels to tty lines */
	for (chan = 0; chan < handle->chan_count; chan++) {
		if ((handle->chan[chan].map.type == NVSHM_CHAN_TTY)
			|| (handle->chan[chan].map.type == NVSHM_CHAN_LOG)) {
			tty_dev.line[chan].nvshm_chan = chan;
			tty_dev.line[chan].ipc_bb2ap =
				handle->ipc_bb2ap;
		}
	}

	/* register as tty for each tty line */
	for (chan = 0; chan < count; chan++) {
		pr_debug("%s: register tty#%d cha %d\n",
			 __func__, chan, tty_dev.line[chan].nvshm_chan);
		spin_lock_init(&tty_dev.line[chan].lock);
		tty_port_init(&tty_dev.line[chan].port);
		tty_dev.line[chan].port.ops = &nvshm_tty_port_ops;
		tty_register_device(tty_dev.tty_driver, chan, 0);
	}

	tty_dev.up = NVSHM_TTY_UP;
	return 0;
}

void nvshm_tty_cleanup(struct nvshm_handle *handle)
{
	int chan;

	pr_debug("%s\n", __func__);
	tty_dev.up = 0;
	/* Wait for all current readers to finish; next ones will fail */
	synchronize_srcu(&tty_dev_lock);
	for (chan = 0; chan < tty_dev.nlines; chan++) {
		struct tty_struct *tty;

		tty = tty_port_tty_get(&tty_dev.line[chan].port);
		if (tty) {
			tty_vhangup(tty);
			tty->driver_data = NULL;
			tty_kref_put(tty);
		}
		/* No need to cleanup data as iobufs are invalid now */
		/* Next nvshm_tty_init will do it */
		pr_debug("%s unregister tty device %d\n", __func__, chan);
		tty_unregister_device(tty_dev.tty_driver, chan);
	}
	destroy_workqueue(tty_dev.tty_wq);
	tty_unregister_driver(tty_dev.tty_driver);
	kfree(tty_dev.line);
	put_tty_driver(tty_dev.tty_driver);
}
