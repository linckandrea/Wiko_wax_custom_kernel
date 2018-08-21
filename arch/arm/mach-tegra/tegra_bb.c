/*
 * arch/arm/mach-tegra/tegra_bb.c
 *
 * Copyright (C) 2012-2014 NVIDIA Corporation. All rights reserved.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/types.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/suspend.h>
#include <linux/pm_runtime.h>
#include <linux/kthread.h>
#include <linux/pm_qos.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/tegra_bb.h>
#include <mach/tegra_bbc_proxy.h>
#include <mach/tegra_bbc_power.h>
#include <linux/platform_data/nvshm.h>

#include "clock.h"
#include "sleep.h"
#include "tegra_emc.h"
#include "pm.h"

/* BB mailbox offset */
#define TEGRA_BB_REG_MAILBOX (0x0)
/* BB mailbox return code (MOSD) offset */
#define TEGRA_BB_REG_RETCODE (0x4)
/* BB status mask */
#define TEGRA_BB_STATUS_MASK  (0xffff)
#define TEGRA_BB_IPC_COLDBOOT  (0x0001)
#define TEGRA_BB_IPC_FW_READY (0x0004)
#define TEGRA_BB_IPC_READY  (0x0005)
#define TEGRA_BB_BOOT_RESTART_FW_REQ	(0x0003)

#define BBC_MC_MIN_FREQ		600000000
#define BBC_MC_MAX_FREQ		760000000

#define PMC_EVENT_COUNTER_0		(0x44c)
#define PMC_EVENT_COUNTER_0_EN_MASK	(1<<20)
#define PMC_EVENT_COUNTER_0_LP0BB	(0<<16)
#define PMC_EVENT_COUNTER_0_LP0ACTIVE	(1<<16)

#define PMC_IPC_STS_0			(0x500)
#define AP2BB_RESET_SHIFT		(1)
#define AP2BB_RESET_DEFAULT_MASK	(1)
#define BB2AP_MEM_REQ_SHIFT		(3)
#define BB2AP_MEM_REQ_SOON_SHIFT	(4)

#define PMC_IPC_SET_0			(0x504)
#define AP2BB_MEM_STS_SHIFT		(5)

#define PMC_IPC_CLEAR_0			(0x508)

#define FLOW_IPC_STS_0			(0x500)
#define AP2BB_MSC_STS_SHIFT		(4)
#define BB2AP_INT1_STS_SHIFT		(3)
#define BB2AP_INT0_STS_SHIFT		(2)
#define AP2BB_INT1_STS_SHIFT		(1)
#define AP2BB_INT1_STS_MASK		(0x1)
#define AP2BB_INT0_STS_SHIFT		(0)

#define FLOW_IPC_SET_0			(0x504)
#define FLOW_IPC_CLR_0			(0x508)

#define MC_BBC_MEM_REGIONS_0_OFFSET (0xF0)

#define MC_BBC_MEM_REGIONS_0_PRIV_SIZE_MASK  (0x3)
#define MC_BBC_MEM_REGIONS_0_PRIV_SIZE_SHIFT (0)

#define MC_BBC_MEM_REGIONS_0_PRIV_BASE_MASK  (0x1FF)
#define MC_BBC_MEM_REGIONS_0_PRIV_BASE_SHIFT (3)

#define MC_BBC_MEM_REGIONS_0_IPC_SIZE_MASK   (0x3)
#define MC_BBC_MEM_REGIONS_0_IPC_SIZE_SHIFT  (16)

#define MC_BBC_MEM_REGIONS_0_IPC_BASE_MASK   (0x1FF)
#define MC_BBC_MEM_REGIONS_0_IPC_BASE_SHIFT  (19)

#define BBC_CPU_MIN_FREQ     (310000) /* 310MHz */
#define BBC_CRASHDUMP_CPU_MIN_FREQ	(1400000) /* 1.4GHz */
static struct pm_qos_request bb_cpufreq_min_req;

#define MEM_REQ_DET_HIGH	true
#define MEM_REQ_DET_LOW		false

enum bbc_pm_state {
	BBC_REMOVE_FLOOR = 1,
	BBC_SET_FLOOR,
	BBC_CRASHDUMP_FLOOR,
};

enum bbc_set_floor_type {
	BBC_MEM_REQ_SOON,
	BBC_IPC,
};

struct tegra_bb {
	spinlock_t lock;
	int status;
	int instance;
	char name[16];
	char priv_name[16];
	char ipc_name[16];
	unsigned long priv_phy;
	unsigned long ipc_phy;
	void *ipc_virt;
	void *mb_virt;
	unsigned long priv_size;
	unsigned long ipc_size;
	unsigned long ipc_irq;
	unsigned long emc_min_freq;
	unsigned long cpu_min_freq;
	u32 emc_flags;
	char ipc_serial[NVSHM_SERIAL_BYTE_SIZE];
	unsigned int irq;
	unsigned int mem_req_soon;
	enum bbc_pm_state next_state, current_state;
	struct regulator *vdd_bb_core;
	struct regulator *vdd_bb_pll;
	unsigned int pll_voltage;
	void (*ipc_cb)(void *data);
	void  *ipc_cb_data;
	struct miscdevice dev_priv;
	struct miscdevice dev_ipc;
	struct device *dev;
	struct sysfs_dirent *sd;
	struct nvshm_platform_data nvshm_pdata;
	struct platform_device nvshm_device;
	struct task_struct *bbc_pm_thread;
	atomic_t state_for_thread;
	wait_queue_head_t emc_wait_q;
	struct clk *emc_clk;
	struct device *proxy_dev;
	struct notifier_block pm_notifier;
	bool is_suspending;
	bool send_ul_flag;
	int set_floor_type;
	long t[8]; /* tracing bb emc floor latency */
};

#define SET_FLOOR_GUARD_TIME     100
#define MAX_SMALL_STAT_TIME      300     /* 300ms */
#define SMALL_STAT_STEP 5
unsigned int bb_emc_set_s_stats[MAX_SMALL_STAT_TIME/SMALL_STAT_STEP];
unsigned int bb_emc_set_l_stats[32];
static int max_emc_set_latency;
#ifdef CONFIG_DEBUG_FS
static void dump_emc_set_stats(void);
#endif

static int tegra_bb_open(struct inode *inode, struct file *filp);
static int tegra_bb_map(struct file *filp, struct vm_area_struct *vma);
static int tegra_bb_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

static const struct file_operations tegra_bb_priv_fops = {
	.owner		= THIS_MODULE,
	.open           = tegra_bb_open,
	.mmap		= tegra_bb_map,
};

static const struct file_operations tegra_bb_ipc_fops = {
	.owner		= THIS_MODULE,
	.open           = tegra_bb_open,
	.mmap		= tegra_bb_map,
};

static const struct vm_operations_struct tegra_bb_vm_ops = {
	.fault = tegra_bb_vm_fault,
};

static void tegra_bb_set_ap2bb_int1(void)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* raise AP2BB INT1 */
	writel(1 << AP2BB_INT1_STS_SHIFT, fctrl + FLOW_IPC_SET_0);
}

static void tegra_bb_clear_ap2bb_int1(void)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* clear AP2BB INT1 status */
	writel(1 << AP2BB_INT1_STS_SHIFT, fctrl + FLOW_IPC_CLR_0);
}

static void tegra_bb_set_ap2bb_int0(void)
{
	void __iomem *flow = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* set AP2BB INT0 */
	writel(1 << AP2BB_INT0_STS_SHIFT, flow + FLOW_IPC_SET_0);
}

void tegra_bb_register_ipc(struct platform_device *pdev,
			   void (*cb)(void *data), void *cb_data)
{
	struct tegra_bb *bb = platform_get_drvdata(pdev);

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return;
	}
	bb->ipc_cb = cb;
	bb->ipc_cb_data = cb_data;
}
EXPORT_SYMBOL_GPL(tegra_bb_register_ipc);

void tegra_bb_generate_ipc(struct platform_device *pdev)
{
	struct tegra_bb *bb = platform_get_drvdata(pdev);
	unsigned long irq_flags;

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return;
	}

#ifdef CONFIG_TEGRA_BASEBAND_SIMU
	{
		int sts;
		if (bb->ipc_cb)
			bb->ipc_cb(bb->ipc_cb_data);
		/* Notify sysfs */
		sts = *(unsigned int *)bb->mb_virt & TEGRA_BB_STATUS_MASK;

		if ((bb->status != TEGRA_BB_IPC_READY) ||
		    (bb->status != sts)) {
			sysfs_notify(&bb->dev->kobj, NULL, "status");
		}
		bb->status = sts;
	}
#else
	{
		unsigned int prev_state = 0;

		bb->t[BBC_IPC] = jiffies;
		/* Do we need to restore BB MC frequency floor? */
		spin_lock_irqsave(&bb->lock, irq_flags);
		if ((bb->current_state == BBC_REMOVE_FLOOR) &&
		    !bb->send_ul_flag) {
			/* Yes => add work to kernel thread */
			pr_debug("%s Request BBC floor", __func__);
			bb->next_state = BBC_SET_FLOOR;
			bb->set_floor_type = BBC_IPC;
			/* set flag to let work know that we have a
			   pending request to signal BB on UL IPC */
			bb->send_ul_flag = true;
			prev_state = atomic_xchg(&bb->state_for_thread,
								 BBC_SET_FLOOR);
			wake_up(&(bb->emc_wait_q));
		}
		if (!bb->send_ul_flag)
			tegra_bb_set_ap2bb_int0();
		spin_unlock_irqrestore(&bb->lock, irq_flags);
		if (prev_state)
			pr_debug("previous state[%u] will be skipped in %s\n",
					 prev_state, __func__);
	}
#endif
}
EXPORT_SYMBOL_GPL(tegra_bb_generate_ipc);

void tegra_bb_clear_ipc(struct platform_device *dev)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* clear BB2AP INT status */
	writel(1 << BB2AP_INT0_STS_SHIFT, fctrl + FLOW_IPC_CLR_0);
}
EXPORT_SYMBOL_GPL(tegra_bb_clear_ipc);

void tegra_bb_abort_ipc(struct platform_device *dev)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* clear AP2BB INT status */
	writel(1 << AP2BB_INT0_STS_SHIFT, fctrl + FLOW_IPC_CLR_0);
}
EXPORT_SYMBOL_GPL(tegra_bb_abort_ipc);

int tegra_bb_check_ipc(struct platform_device *dev)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	int sts;

	sts = readl(fctrl + FLOW_IPC_STS_0);
	if (sts & (1 << AP2BB_INT0_STS_SHIFT))
		return 0;
	return 1;
}
EXPORT_SYMBOL_GPL(tegra_bb_check_ipc);

int tegra_bb_check_bb2ap_ipc(void)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	int sts;

	sts = readl(pmc + PMC_IPC_STS_0);
	sts = sts >> AP2BB_RESET_SHIFT;
	sts &= AP2BB_RESET_DEFAULT_MASK;
	if (!sts)
		return 0;

	sts = readl(fctrl + FLOW_IPC_STS_0);
	if (sts & (1 << BB2AP_INT0_STS_SHIFT))
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_bb_check_bb2ap_ipc);

static int tegra_bb_open(struct inode *inode, struct file *filp)
{
	int ret;

	ret = nonseekable_open(inode, filp);
	return ret;
}

static int tegra_bb_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	pr_warn("%s  vma->vm_start=0x%x vma->vm_end=0x%x vma->vm_pgoff=0x%x\n"
		, __func__,
		(unsigned int)vma->vm_start,
		(unsigned int)vma->vm_end,
		(unsigned int)vma->vm_pgoff);
	vmf = vmf;
	return VM_FAULT_NOPAGE;
}

static inline void tegra_bb_disable_mem_req_soon(void)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* AP2BB_MSC_STS[3] is to mask or unmask
	 * mem_req_soon interrupt to interrupt controller */
	writel((0x8 << AP2BB_MSC_STS_SHIFT), fctrl + FLOW_IPC_CLR_0);

	trace_printk("%s: fctrl ipc_sts = %x\n", __func__,
		readl(fctrl + FLOW_IPC_STS_0));
}

static inline void tegra_bb_enable_mem_req_soon(void)
{
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	/* AP2BB_MSC_STS[3] is to mask or unmask
	 * mem_req_soon interrupt to interrupt controller */
	writel((0x8 << AP2BB_MSC_STS_SHIFT), fctrl + FLOW_IPC_SET_0);

	pr_debug("%s: fctrl ipc_sts = %x\n", __func__,
		readl(fctrl + FLOW_IPC_STS_0));
}

static int tegra_bb_map(struct file *filp, struct vm_area_struct *vma)
{
	struct miscdevice *miscdev = filp->private_data;
	struct tegra_bb *bb;
	unsigned long phy;
	unsigned long off;
	unsigned long vsize;
	unsigned long psize;
	int ret;

	off = vma->vm_pgoff << PAGE_SHIFT;
	vsize = vma->vm_end - vma->vm_start;
	bb = dev_get_drvdata(miscdev->parent);
	if (miscdev->fops == &tegra_bb_priv_fops) {
		phy = bb->priv_phy + off;
		pr_debug("%s map priv section @0x%x\n",
			__func__,
			(unsigned int)phy);
		psize = bb->priv_size - off;
	} else {
		phy = bb->ipc_phy + off;
		pr_debug("%s map ipc section @0x%x\n",
			__func__,
			(unsigned int)phy);
		psize =  bb->ipc_size - off;
	}
	if (vsize > psize) {
		pr_err("%s request exceed mapping!\n",
		       __func__);
		return -EINVAL;
	}

	vma->vm_ops = &tegra_bb_vm_ops;
	ret = remap_pfn_range(vma, vma->vm_start, __phys_to_pfn(phy),
			      vsize, pgprot_noncached(vma->vm_page_prot));
	if (ret) {
		pr_err("%s remap_pfn_range ret %d\n", __func__, (int)ret);
		return -EAGAIN;
	}
	return ret;
}

static ssize_t show_tegra_bb_retcode(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct tegra_bb *bb = dev_get_drvdata(dev);
	int retcode;

	if (!bb) {
		dev_err(dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	retcode = *(int *)((unsigned long)bb->mb_virt + TEGRA_BB_REG_RETCODE);
	dev_dbg(dev, "%s retcode=%d\n", __func__, (int)retcode);
	return sprintf(buf, "%d\n", retcode);
}
static DEVICE_ATTR(retcode, S_IRUSR | S_IRGRP, show_tegra_bb_retcode, NULL);

static ssize_t show_tegra_bb_status(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tegra_bb *bb = dev_get_drvdata(dev);
	unsigned int *ptr;
	int status;

	if (!bb) {
		dev_err(dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	ptr = bb->mb_virt + TEGRA_BB_REG_MAILBOX;
	status = *ptr & 0xFFFF;

	dev_dbg(dev, "%s status=%x\n", __func__, status);
	return sprintf(buf, "%d\n", status);
}

static ssize_t store_tegra_bb_status(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int ret, value;
	unsigned int *ptr;
	struct tegra_bb *bb = dev_get_drvdata(dev);
	struct platform_device *pdev = container_of(dev, struct platform_device,
						    dev);
	if (!bb) {
		dev_err(dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	ret = sscanf(buf, "%d", &value);
	if (ret != 1)
		return -1;
	value &= 0xFFFF;
	ptr = bb->mb_virt + TEGRA_BB_REG_MAILBOX;
	*ptr = (value & 0xFFFF) | ((~value << 16) & 0xFFFF0000);
	tegra_bb_generate_ipc(pdev);
	if (value == TEGRA_BB_IPC_FW_READY) {
		/* request enough iso bw for boot */
		tegra_bbc_proxy_bw_request(bb->proxy_dev, 0, BBC_ISO_BOOT_BW,
						1000, BBC_ISO_MARGIN_BW);
	}

	dev_dbg(dev, "%s status=0x%x\n",  __func__, (unsigned int)value);
	return count;
}

static DEVICE_ATTR(status, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
		   show_tegra_bb_status, store_tegra_bb_status);

static ssize_t show_tegra_bb_priv_size(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct tegra_bb *bb = dev_get_drvdata(dev);

	if (!bb) {
		dev_err(dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	dev_dbg(dev, "%s priv_size=%d\n", __func__,
		 (unsigned int)bb->priv_size);
	return sprintf(buf, "%d\n", (int)bb->priv_size);
}
static DEVICE_ATTR(priv_size, S_IRUSR | S_IRGRP, show_tegra_bb_priv_size, NULL);

static ssize_t show_tegra_bb_ipc_size(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct tegra_bb *bb = dev_get_drvdata(dev);

	if (!bb) {
		dev_err(dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	dev_dbg(dev, "%s ipc_size=%d\n", __func__, (unsigned int)bb->ipc_size);
	return sprintf(buf, "%d\n", (int)bb->ipc_size);
}
static DEVICE_ATTR(ipc_size, S_IRUSR | S_IRGRP, show_tegra_bb_ipc_size, NULL);

static ssize_t store_tegra_bb_fault(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) != 1)
		return -1;

	if ((value != 0) && (value != 1))
		return -EINVAL;

	if (value) {
		dev_warn(dev, "%s: generate BB fault\n", __func__);
		tegra_bb_set_ap2bb_int1();
	} else {
		dev_warn(dev, "%s: clear BB fault\n", __func__);
		tegra_bb_clear_ap2bb_int1();
	}

	return 1;
}

static ssize_t show_tegra_bb_fault(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	u32 sts;
	void __iomem *flow = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);

	sts = readl(flow + FLOW_IPC_STS_0);
	sts = (sts >> AP2BB_INT1_STS_SHIFT) & AP2BB_INT1_STS_MASK;

	return sprintf(buf, "%d\n", (int)sts);
}

static DEVICE_ATTR(fault, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
		   show_tegra_bb_fault, store_tegra_bb_fault);

static ssize_t store_tegra_bb_reset(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) != 1)
		return -1;

	return tegra_bb_set_reset(dev, value);
}

static ssize_t show_tegra_bb_reset(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int sts;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	/* reset is active low - sysfs interface assume reset is active high */

	sts = readl(pmc + PMC_IPC_STS_0);
	dev_dbg(dev, "%s IPC_STS=0x%x\n", __func__, (unsigned int)sts);
	sts = ~sts >> AP2BB_RESET_SHIFT;
	sts &= AP2BB_RESET_DEFAULT_MASK;

	dev_dbg(dev, "%s reset=%d\n", __func__, (unsigned int)sts);
	return sprintf(buf, "%d\n", (int)sts);
}

static DEVICE_ATTR(reset, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
		   show_tegra_bb_reset, store_tegra_bb_reset);

int tegra_bb_set_reset(struct device *dev, int value)
{
	static bool regulator_status;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	struct tegra_bb *bb = dev_get_drvdata(dev);

	if (!bb) {
		dev_err(dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	if ((value < 0) || (value > 1))
		return -EINVAL;

	dev_dbg(dev, "%s reset=%d\n",  __func__, (unsigned int)value);

	/* reset is active low - sysfs interface assume reset is active high */
	if (value) {
		unsigned int prev_state;
		writel(1 << AP2BB_RESET_SHIFT | 1 << AP2BB_MEM_STS_SHIFT,
			pmc + PMC_IPC_CLEAR_0);

		if (regulator_status == true) {
			pr_debug("%s: disabling bbc regulators\n", __func__);
			regulator_disable(bb->vdd_bb_core);
			regulator_disable(bb->vdd_bb_pll);

			regulator_status = false;
		}

		bb->next_state = BBC_REMOVE_FLOOR;
		prev_state = atomic_xchg(&bb->state_for_thread,
							 BBC_REMOVE_FLOOR);
		wake_up(&(bb->emc_wait_q));
		if (prev_state)
			pr_debug("previous state[%u] will be skipped in %s\n",
					 prev_state, __func__);
	} else {
		/* power on bbc rails */
		if (bb->vdd_bb_core && bb->vdd_bb_pll &&
				regulator_status == false) {
			pr_debug("%s: enabling bbc regulators\n", __func__);
			regulator_set_voltage(bb->vdd_bb_core, 1100000,
							1100000);
			regulator_enable(bb->vdd_bb_core);

			regulator_set_voltage(bb->vdd_bb_pll, bb->pll_voltage,
							bb->pll_voltage);
			regulator_enable(bb->vdd_bb_pll);
			regulator_status = true;

			tegra_bb_enable_mem_req_soon();
		}

		writel(1 << AP2BB_RESET_SHIFT | 1 << AP2BB_MEM_STS_SHIFT,
			pmc + PMC_IPC_SET_0);
	}
	return 1;
}
EXPORT_SYMBOL(tegra_bb_set_reset);

static ssize_t show_tegra_bb_state(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	unsigned int sts, mem_req, mem_req_soon;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);

	sts = readl(pmc + PMC_IPC_STS_0);
	dev_dbg(dev, "%s IPC_STS=0x%x\n", __func__, (unsigned int)sts);

	mem_req = (sts >> BB2AP_MEM_REQ_SHIFT) & 1;
	mem_req_soon = (sts >> BB2AP_MEM_REQ_SOON_SHIFT) & 1;

	dev_dbg(dev, "%s mem_req=%d mem_req_soon=%d\n", __func__,
					mem_req, mem_req_soon);
	return sprintf(buf, "%d\n", (unsigned int)mem_req);
}

static DEVICE_ATTR(state, S_IRUSR | S_IRGRP, show_tegra_bb_state, NULL);

static ssize_t show_tegra_bb_serial(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct tegra_bb *bb = dev_get_drvdata(dev);
	int idx, ret;

	if (!bb) {
		dev_err(dev, "%s tegra_bb not found!\n", __func__);
		return 0;
	}

	for (idx = 0; NVSHM_SERIAL_BYTE_SIZE > idx; ++idx) {
		ret = sprintf(buf+2*idx, "%02X", bb->ipc_serial[idx]);
		if (ret < 0) {
			dev_err(dev, "%s sprintf shm serial failure!\n",
					__func__);
			return 0;
		}
	}

	buf[2*NVSHM_SERIAL_BYTE_SIZE] = '\n';

	return (2*NVSHM_SERIAL_BYTE_SIZE+1);
}

static DEVICE_ATTR(serial, S_IRUSR | S_IRGRP, show_tegra_bb_serial, NULL);

void tegra_bb_set_ipc_serial(struct platform_device *pdev, char *serial)
{
	struct tegra_bb *bb = platform_get_drvdata(pdev);

	if (!bb) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return;
	}

	if (serial == NULL) {
		/* Remove sysfs entry */
		device_remove_file(&pdev->dev, &dev_attr_serial);
		return;
	}

	/* Create sysfs entry */
	device_create_file(&pdev->dev, &dev_attr_serial);

	/* Locally store serail number for future sysfs access */
	memcpy(bb->ipc_serial, serial, sizeof(bb->ipc_serial));
}
EXPORT_SYMBOL_GPL(tegra_bb_set_ipc_serial);

#ifndef CONFIG_TEGRA_BASEBAND_SIMU
static irqreturn_t tegra_bb_isr_handler(int irq, void *data)
{
	struct tegra_bb *bb = (struct tegra_bb *)data;
	void __iomem *fctrl = IO_ADDRESS(TEGRA_FLOW_CTRL_BASE);
	unsigned long irq_flags;
	int sts;

	disable_irq_nosync(irq);
#ifdef CONFIG_PM
	if (bb->is_suspending)
		pm_wakeup_event(bb->dev, 0);
#endif

	spin_lock_irqsave(&bb->lock, irq_flags);
	/* read/clear INT status */
	sts = readl(fctrl + FLOW_IPC_STS_0);
	if (sts & (1 << BB2AP_INT0_STS_SHIFT))
		writel(1 << BB2AP_INT0_STS_SHIFT, fctrl + FLOW_IPC_CLR_0);

	spin_unlock_irqrestore(&bb->lock, irq_flags);
	/* wake IPC mechanism */
	if (bb->ipc_cb)
		bb->ipc_cb(bb->ipc_cb_data);

	/* Notify sysfs */
	sts = *(unsigned int *)bb->mb_virt & TEGRA_BB_STATUS_MASK;

	if ((bb->status != TEGRA_BB_IPC_READY) ||
	    (bb->status != sts)) {
		pr_debug("%s: notify sysfs status %d\n", __func__, sts);
		sysfs_notify_dirent(bb->sd);
	}

	if (sts == TEGRA_BB_BOOT_RESTART_FW_REQ) {
		unsigned int prev_state;
		pr_debug("%s: boot_restart_fw_req\n", __func__);
		bb->next_state = BBC_CRASHDUMP_FLOOR;
		prev_state = atomic_xchg(&bb->state_for_thread,
							 BBC_CRASHDUMP_FLOOR);
		wake_up(&(bb->emc_wait_q));
		if (prev_state)
			pr_debug("previous state[%u] will be skipped in %s\n",
					 prev_state, __func__);
	}

	bb->status = sts;
	return IRQ_HANDLED;
}

static irqreturn_t tegra_bb_mem_req_soon(int irq, void *data)
{
	struct tegra_bb *bb = (struct tegra_bb *)data;
	unsigned int prev_state;

	bb->t[BBC_MEM_REQ_SOON] = jiffies;
	spin_lock(&bb->lock);

	tegra_bb_disable_mem_req_soon();
	bb->next_state = BBC_SET_FLOOR;
	bb->set_floor_type = BBC_MEM_REQ_SOON;
	prev_state = atomic_xchg(&bb->state_for_thread,
						 BBC_SET_FLOOR);
	wake_up(&(bb->emc_wait_q));
	if (prev_state)
		pr_debug("previous state[%u] will be skipped in %s\n",
				 prev_state, __func__);

	spin_unlock(&bb->lock);

	return IRQ_HANDLED;
}

static inline void pmc_32kwritel(u32 val, unsigned long offs)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	writel(val, pmc + offs);
	udelay(130);
}

static void tegra_bb_disable_pmc_wake(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	void __iomem *tert_ictlr = IO_ADDRESS(TEGRA_TERTIARY_ICTLR_BASE);

	u32 reg = readl(pmc + PMC_CTRL2);
	reg &= ~(PMC_CTRL2_WAKE_DET_EN);
	pmc_32kwritel(reg, PMC_CTRL2);

	writel(TRI_ICTLR_PMC_WAKE_INT, tert_ictlr + TRI_ICTLR_CPU_IER_CLR);
}

static void tegra_bb_enable_pmc_wake(bool active_high)
{
	/* Set PMC wake interrupt, see Bug 1181348 for SW level programming
	 * details
	 */
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	void __iomem *tert_ictlr = IO_ADDRESS(TEGRA_TERTIARY_ICTLR_BASE);

	u32 reg = readl(pmc + PMC_CTRL2);
	reg &= ~(PMC_CTRL2_WAKE_DET_EN);
	pmc_32kwritel(reg, PMC_CTRL2);

	reg = readl(pmc + PMC_WAKE2_LEVEL);
	if (active_high)
		reg |= PMC_WAKE2_BB_MEM_REQ;
	else
		reg &= ~(PMC_WAKE2_BB_MEM_REQ);
	pmc_32kwritel(reg, PMC_WAKE2_LEVEL);

	udelay(100);
	pmc_32kwritel(1, PMC_AUTO_WAKE_LVL);

	udelay(100);
	reg = PMC_WAKE2_BB_MEM_REQ;
	pmc_32kwritel(reg, PMC_WAKE2_MASK);

	reg = readl(pmc + PMC_CTRL2);
	reg |= PMC_CTRL2_WAKE_DET_EN;
	pmc_32kwritel(reg, PMC_CTRL2);

	writel(TRI_ICTLR_PMC_WAKE_INT, tert_ictlr + TRI_ICTLR_CPU_IER_SET);

	pr_debug("%s\n", __func__);
}

static irqreturn_t tegra_pmc_wake_intr(int irq, void *data)
{
	struct tegra_bb *bb = (struct tegra_bb *)data;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	static void __iomem *tert_ictlr = IO_ADDRESS(TEGRA_TERTIARY_ICTLR_BASE);
	u32 lic, sts;
	int mem_req_soon, mem_req;

	spin_lock(&bb->lock);
	sts = readl(pmc + PMC_IPC_STS_0);
	mem_req_soon = (sts >> BB2AP_MEM_REQ_SOON_SHIFT) & 1;
	mem_req = (sts >> BB2AP_MEM_REQ_SHIFT) & 1;

	/* clear interrupt */
	lic = readl(tert_ictlr + TRI_ICTLR_VIRQ_CPU);
	if (lic & TRI_ICTLR_PMC_WAKE_INT)
		writel(TRI_ICTLR_PMC_WAKE_INT,
				tert_ictlr + TRI_ICTLR_CPU_IER_CLR);

	if (mem_req) {
		tegra_bb_enable_pmc_wake(MEM_REQ_DET_LOW);
		spin_unlock(&bb->lock);
		return IRQ_HANDLED;
	}

	if (!mem_req_soon) {
		unsigned int prev_state;
		bb->next_state = BBC_REMOVE_FLOOR;
		prev_state = atomic_xchg(&bb->state_for_thread,
							 BBC_REMOVE_FLOOR);
		wake_up(&(bb->emc_wait_q));
		if (prev_state)
			pr_debug("previous state[%u] will be skipped in %s\n",
					 prev_state, __func__);
	} else
		pr_debug("%s: mem_req 0 but mem_req_soon 1\n", __func__);

	tegra_bb_enable_pmc_wake(MEM_REQ_DET_HIGH);
	spin_unlock(&bb->lock);

	return IRQ_HANDLED;
}

static void tegra_bb_set_emc(struct tegra_bb *bb)
{
	unsigned long flags;
	static long start, diff0, diff, diff1, diff2, diff3, diff4;
	static int cnt;

	if (!bb)
		return;

	if (bb->next_state == BBC_SET_FLOOR) {
		bb->t[2] = jiffies;
		start = max(bb->t[0], bb->t[1]);
		diff0 = (bb->t[2] - start) * 1000 / HZ;
	}

	spin_lock_irqsave(&bb->lock, flags);
	if (bb->current_state == bb->next_state) {
		spin_unlock_irqrestore(&bb->lock, flags);
		return;
	}

	bb->t[4] = jiffies;
	diff1 = (bb->t[4] - start) * 1000 / HZ;

	switch (bb->next_state) {
	case BBC_SET_FLOOR:
		spin_unlock_irqrestore(&bb->lock, flags);
		bb->cpu_min_freq = BBC_CPU_MIN_FREQ;
		bb->t[5] = jiffies;
		diff2 = (bb->t[5] - start) * 1000 / HZ;
		pm_runtime_get_sync(bb->dev);
		bb->t[6] = jiffies;
		diff3 = (bb->t[6] - start) * 1000 / HZ;
		/* going from 0 to high */
		clk_prepare_enable(bb->emc_clk);
		if (bb->emc_flags & EMC_DSR)
			tegra_emc_dsr_override(TEGRA_EMC_DSR_OVERRIDE);
		else
			tegra_emc_dsr_override(TEGRA_EMC_DSR_NORMAL);
		bb->t[7] = jiffies;
		diff4 = (bb->t[7] - start) * 1000 / HZ;
		if ((bb->emc_flags & EMC_LL) &&
		    tegra_emc_request_low_latency_mode(true))
			dev_err(bb->dev, "emc low latency request failed\n");
		bb->t[3] = jiffies;

		diff = (bb->t[3] - start) * 1000 / HZ;
		if (diff >= 0) {
			if (diff < MAX_SMALL_STAT_TIME)
				bb_emc_set_s_stats[diff/SMALL_STAT_STEP]++;
			bb_emc_set_l_stats[fls(diff)]++;
			max_emc_set_latency = (diff > max_emc_set_latency) ? diff : max_emc_set_latency;
		}
		if (diff > SET_FLOOR_GUARD_TIME) {
			pr_info("bbc setting floor latency: %ld ms, irq2entry: %ld ms, diff1:%ld ms,\
				diff2:%ld ms, diff3:%ld ms, diff4:%ld ms, type:%d, cnt:%d\n",
				diff, diff0, diff1, diff2, diff3, diff4, bb->set_floor_type, cnt++);
		}
		pr_debug("bbc setting floor to %luMHz\n",
						bb->emc_min_freq/1000000);

		/* restore iso bw request*/
		tegra_bbc_proxy_restore_iso(bb->proxy_dev);


		spin_lock_irqsave(&bb->lock, flags);
		if (bb->send_ul_flag) {
			tegra_bb_set_ap2bb_int0();
			pr_debug("bbc floor applied\n");
			bb->send_ul_flag = false;
		} else {
			/* reenable pmc_wake_det irq if mem_req_soon is high */
			tegra_bb_enable_pmc_wake(MEM_REQ_DET_LOW);
		}

		/* update current state, now that BB floor
		   is in place */
		bb->current_state = BBC_SET_FLOOR;

		/* let BB know we have set its floor */
		bbc_power_emc_floor_set(1);

		spin_unlock_irqrestore(&bb->lock, flags);
		return;
	case BBC_REMOVE_FLOOR:
		/* discard erroneous request */
		if (bb->current_state != BBC_SET_FLOOR) {
			spin_unlock_irqrestore(&bb->lock, flags);
			return;
		}
		bb->current_state = bb->next_state;
		/* let BB know we are removing its floor */
		bbc_power_emc_floor_set(0);
		spin_unlock_irqrestore(&bb->lock, flags);

		/* remove iso bandwitdh request from bbc */
		tegra_bbc_proxy_clear_iso(bb->proxy_dev);

		if (bb->emc_flags & EMC_LL)
			tegra_emc_request_low_latency_mode(false);
		/* going from high to 0 */
		tegra_emc_dsr_override(TEGRA_EMC_DSR_NORMAL);

		clk_disable_unprepare(bb->emc_clk);
		pr_debug("bbc removing emc floor\n");

		/* reenable mem_req_soon irq */
		tegra_bb_enable_mem_req_soon();
		pm_runtime_put(bb->dev);
		return;

	case BBC_CRASHDUMP_FLOOR:
		/* BBC is crashed and ready to send coredump.
		 * do not store prev_state */
		spin_unlock_irqrestore(&bb->lock, flags);

		pr_info("%s: bbc crash detected, set EMC to max (current=%lu Hz)\n",
			__func__, clk_get_rate(bb->emc_clk));

		if (bb->current_state != BBC_SET_FLOOR) {
			pm_runtime_get_sync(bb->dev);
			clk_prepare_enable(bb->emc_clk);
			pr_info("%s: enable EMC clk because of BBC_CRASHDUMP_FLOOR\n",
				__func__);
		}

		if (bb->emc_flags & EMC_LL) {
			tegra_emc_request_low_latency_mode(false);
			bb->emc_flags &= ~EMC_LL;
		}
		tegra_emc_dsr_override(TEGRA_EMC_DSR_OVERRIDE);
		clk_set_rate(bb->emc_clk, BBC_MC_MAX_FREQ);

		/* Boost CPU freq. */
		bb->cpu_min_freq = BBC_CRASHDUMP_CPU_MIN_FREQ;
		pm_qos_update_request(&bb_cpufreq_min_req,
			bb->cpu_min_freq);


		/* request enough iso bw for crash handling */
		tegra_bbc_proxy_bw_request(bb->proxy_dev, 0, BBC_ISO_CRASH_BW,
						1000, BBC_ISO_MARGIN_BW);

		tegra_bbc_proxy_la_request(bb->proxy_dev, BBCLLR_LA_CRASH_BW,
					BBCR_LA_CRASH_BW, BBCW_LA_CRASH_BW);

		/* request a high enough power budget for crash handling */
		tegra_bbc_proxy_edp_request(bb->proxy_dev, 0, BBC_EDP_E0_INDEX,
						BBC_EDP_E0_THRESHOLD);

#ifdef CONFIG_DEBUG_FS
		dump_emc_set_stats();
#endif
		return;
	default:
		spin_unlock_irqrestore(&bb->lock, flags);
		break;
	}

	return;
}

static int tegra_bb_emc_dvfs_thread(void *arg)
{
	struct tegra_bb *bb = (struct tegra_bb *)arg;

	if (!bb) {
		pr_err("Invalid tegra_bb !!!\n");
		return -1;
	}

	while (!kthread_should_stop()) {
		wait_event_interruptible(bb->emc_wait_q, atomic_read(&bb->state_for_thread));
		if (atomic_xchg(&bb->state_for_thread, 0))
			tegra_bb_set_emc(bb);
	}
	pr_info("tegra_bb_emc_dvfs_thread exit!!\n");
	return 0;
}

void tegra_bb_set_emc_floor(struct device *dev, unsigned long freq, u32 flags)
{
	struct tegra_bb *bb = dev_get_drvdata(dev);

	if (!bb) {
		dev_err(dev, "%s tegra_bb not found!\n", __func__);
		return;
	}

	if (bb->emc_min_freq != freq) {
		bb->emc_min_freq = freq;
		clk_set_rate(bb->emc_clk, bb->emc_min_freq);
	}

	if (((bb->emc_flags & EMC_LL) != (flags & EMC_LL)) &&
	    tegra_emc_request_low_latency_mode(flags & EMC_LL))
		dev_err(bb->dev, "emc low latency request failed\n");

	bb->emc_flags = flags;
	return;
}
EXPORT_SYMBOL(tegra_bb_set_emc_floor);

#endif

static int tegra_bb_pm_notifier_event(struct notifier_block *this,
					unsigned long event, void *ptr)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	struct tegra_bb *bb = container_of(this, struct tegra_bb, pm_notifier);
	int sts, mem_req_soon;

	if (!bb) {
		pr_err("tegra_bb not found!\n");
		return NOTIFY_OK;
	}

	sts = readl(pmc + PMC_IPC_STS_0);
	mem_req_soon = (sts >> BB2AP_MEM_REQ_SOON_SHIFT) & 1;
	sts = sts >> AP2BB_RESET_SHIFT;
	sts &= AP2BB_RESET_DEFAULT_MASK;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		/* make sure IRQ will send a pm wake event */
		bb->is_suspending = true;

		/* inform tegra_common_suspend about EMC requirement */
		tegra_lp1bb_suspend_emc_rate(bb->emc_min_freq, BBC_MC_MAX_FREQ);

		/* prepare for possible LP1BB state */
		if (sts) {
			clk_prepare_enable(bb->emc_clk);
			clk_set_rate(bb->emc_clk, BBC_MC_MAX_FREQ);
		}

		return NOTIFY_OK;

	case PM_POST_SUSPEND:
		/* no need for IRQ to send a pm wake events anymore */
		bb->is_suspending = false;

		if (sts) {
			clk_set_rate(bb->emc_clk, bb->emc_min_freq);
			clk_disable_unprepare(bb->emc_clk);
		}
		/* else, wait for IRQs to do the job */
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static int tegra_bb_probe(struct platform_device *pdev)
{
	struct tegra_bb *bb;
	int ret;
	struct tegra_bb_platform_data *pdata;
	void __iomem *tegra_mc = IO_ADDRESS(TEGRA_MC_BASE);
	unsigned int size, bbc_mem_regions_0;
	struct clk *c;
	unsigned int mb_size = SZ_4K + SZ_128K; /* config + stats */
	int ret_th_run = 0;
	struct sched_param sch_param = {
		.sched_priority = MAX_RT_PRIO - 1
	};

	if (!pdev) {
		pr_err("%s platform device is NULL!\n", __func__);
		return -EINVAL;
	}

	pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s tegra_bb not found!\n", __func__);
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "%s\n", __func__);
	bb =  kzalloc(sizeof(struct tegra_bb), GFP_KERNEL);

	if (bb == NULL) {
		kfree(bb);
		return -ENOMEM;
	}
	bb->cpu_min_freq = BBC_CPU_MIN_FREQ;
	pm_qos_add_request(&bb_cpufreq_min_req, PM_QOS_CPU_FREQ_MIN,
		PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
	spin_lock_init(&bb->lock);
	atomic_set(&bb->state_for_thread, 0);
	init_waitqueue_head(&(bb->emc_wait_q));
	bb->bbc_pm_thread = kthread_run(tegra_bb_emc_dvfs_thread, bb, "bbc-pm");
	if (IS_ERR(bb->bbc_pm_thread)) {
		ret_th_run = PTR_ERR(bb->bbc_pm_thread);
		pr_err("cannot run kthread!\n");
		return ret_th_run;
	}

	sched_setscheduler(bb->bbc_pm_thread,
					   SCHED_FIFO,
					   &sch_param);
	pr_info("run kthread for bb_emc_dvfs_thread\n");

	/* Private region */
	bbc_mem_regions_0 = readl(tegra_mc + MC_BBC_MEM_REGIONS_0_OFFSET);

	pr_info("%s MC_BBC_MEM_REGIONS_0=0x%x\n", __func__, bbc_mem_regions_0);

	size = (bbc_mem_regions_0 >> MC_BBC_MEM_REGIONS_0_PRIV_SIZE_SHIFT) &
		MC_BBC_MEM_REGIONS_0_PRIV_SIZE_MASK;

	/* Private */
	switch (size) {
	case 0:
		bb->priv_size = SZ_8M;
		break;
	case 1:
		bb->priv_size = SZ_16M;
		break;
	case 2:
		bb->priv_size = SZ_32M;
		break;
	case 3:
		bb->priv_size = SZ_64M;
		break;
	case 7:
		pr_err("%s no private memory mapped\n", __func__);
		break;
	default:
		pr_err("%s invalid private memory size 0x%x\n", __func__, size);
		break;
	}

	bb->priv_phy =
		((bbc_mem_regions_0 >> MC_BBC_MEM_REGIONS_0_PRIV_BASE_SHIFT)
		 & MC_BBC_MEM_REGIONS_0_PRIV_BASE_MASK) << 23;

	/* IPC */
	size = (bbc_mem_regions_0 >> MC_BBC_MEM_REGIONS_0_IPC_SIZE_SHIFT) &
		MC_BBC_MEM_REGIONS_0_IPC_SIZE_MASK;

	switch (size) {
	case 0:
		bb->ipc_size = SZ_8M;
		break;
	case 1:
		bb->ipc_size = SZ_16M;
		break;
	case 2:
		bb->ipc_size = SZ_32M;
		break;
	case 3:
		bb->ipc_size = SZ_64M;
		break;
	case 7:
		pr_err("%s no IPC memory mapped\n", __func__);
		break;
	default:
		pr_err("%s invalid IPC  memory size 0x%x\n", __func__, size);
		break;
	}

	bb->ipc_phy =
		((bbc_mem_regions_0 >> MC_BBC_MEM_REGIONS_0_IPC_BASE_SHIFT)
		 & MC_BBC_MEM_REGIONS_0_IPC_BASE_MASK) << 23;

	pr_info("%s  priv@0x%lx/0x%lx\n", __func__,
		 (unsigned long)bb->priv_phy,
		bb->priv_size);

	pr_info("%s  ipc@0x%lx/0x%lx\n", __func__,
		 (unsigned long)bb->ipc_phy,
		bb->ipc_size);

	if (!(bb->ipc_size && bb->priv_size)) {
		pr_err("%s: invalid tegra_bb regions\n", __func__);
		BUG();
	}

	bb->irq = pdata->bb_irq;
	bb->mem_req_soon = pdata->mem_req_soon;

	/* Map mb_virt uncached (first 132K of IPC for config + statistics) */
	bb->mb_virt = ioremap_nocache(bb->ipc_phy, mb_size);
	pr_debug("%s: uncached IPC Virtual=0x%p\n", __func__, bb->mb_virt);

	/* IPC memory is cached */
	bb->ipc_virt =  ioremap_cached(bb->ipc_phy, bb->ipc_size);
	pr_debug("%s: IPC Virtual=0x%p\n", __func__, bb->ipc_virt);

	/* clear the first 4K of IPC memory */
	memset(bb->mb_virt, 0, SZ_1K*4);

	/* init value of cold boot */
	*(unsigned int *)bb->mb_virt = TEGRA_BB_IPC_COLDBOOT |
		((~TEGRA_BB_IPC_COLDBOOT) << 16);

	/* Register devs */
	bb->dev_priv.minor = MISC_DYNAMIC_MINOR;
	snprintf(bb->priv_name, sizeof(bb->priv_name),
		 "tegra_bb_priv%d", pdev->id);

	bb->dev_priv.name = bb->priv_name;
	bb->dev_priv.fops = &tegra_bb_priv_fops;
	bb->dev_priv.parent = &pdev->dev;

	bb->dev_ipc.minor = MISC_DYNAMIC_MINOR;
	snprintf(bb->ipc_name, sizeof(bb->ipc_name),
		 "tegra_bb_ipc%d",
		 pdev->id);

	bb->dev_ipc.name = bb->ipc_name;
	bb->dev_ipc.fops = &tegra_bb_ipc_fops;
	bb->dev_ipc.parent = &pdev->dev;

	ret = misc_register(&bb->dev_priv);
	if (ret) {
		dev_err(&pdev->dev, "unable to register miscdevice %s\n",
			bb->dev_priv.name);
		kfree(bb);
		return -EAGAIN;
	}

	ret = misc_register(&bb->dev_ipc);
	if (ret) {
		dev_err(&pdev->dev, "unable to register miscdevice %s\n",
			bb->dev_ipc.name);
		kfree(bb);
		return -EAGAIN;
	}

	bb->dev = &pdev->dev;
	bb->instance = pdev->id;
	bb->status = 0;

	ret = device_create_file(&pdev->dev, &dev_attr_status);
	ret = device_create_file(&pdev->dev, &dev_attr_retcode);
	ret = device_create_file(&pdev->dev, &dev_attr_priv_size);
	ret = device_create_file(&pdev->dev, &dev_attr_ipc_size);
	ret = device_create_file(&pdev->dev, &dev_attr_reset);
	ret = device_create_file(&pdev->dev, &dev_attr_state);
	ret = device_create_file(&pdev->dev, &dev_attr_fault);

	bb->sd = sysfs_get_dirent(pdev->dev.kobj.sd, NULL, "status");

	bb->vdd_bb_core = regulator_get(NULL, "vdd_bb");
	if (IS_ERR_OR_NULL(bb->vdd_bb_core)) {
		pr_err("vdd_bb regulator get failed\n");
		bb->vdd_bb_core = NULL;
	}

	bb->vdd_bb_pll = regulator_get(NULL, "avdd_bb_pll");
	if (IS_ERR_OR_NULL(bb->vdd_bb_pll)) {
		pr_err("avdd_bb_pll regulator get failed\n");
		bb->vdd_bb_pll = NULL;
	}

	bb->pll_voltage = pdata->pll_voltage;

	/* clk enable for mc_bbc / pll_p_bbc */
	c = tegra_get_clock_by_name("mc_bbc");
	if (IS_ERR_OR_NULL(c))
		pr_err("mc_bbc get failed\n");
	else
		clk_enable(c);
	c = tegra_get_clock_by_name("pll_p_bbc");
	if (IS_ERR_OR_NULL(c))
		pr_err("pll_p_bbc get failed\n");
	else
		clk_enable(c);

	bb->nvshm_pdata.ipc_base_virt = bb->ipc_virt;
	bb->nvshm_pdata.ipc_size = bb->ipc_size;
	bb->nvshm_pdata.mb_base_virt = bb->mb_virt;
	bb->nvshm_pdata.mb_size = mb_size;
	bb->nvshm_pdata.bb_irq = bb->irq;
	bb->nvshm_pdata.tegra_bb = pdev;
	bb->nvshm_device.name = "nvshm";
	bb->nvshm_device.id = bb->instance;
	bb->nvshm_device.dev.platform_data = &bb->nvshm_pdata;
	platform_device_register(&bb->nvshm_device);

	pm_runtime_enable(&pdev->dev);

#ifndef CONFIG_TEGRA_BASEBAND_SIMU
	snprintf(bb->name, sizeof(bb->name), "tegra_bb%d", pdev->id);

	ret = request_irq(bb->irq, tegra_bb_isr_handler, IRQF_TRIGGER_HIGH,
				bb->name, bb);
	if (ret) {
		dev_err(&pdev->dev, "Could not register irq handler\n");
		kfree(bb);
		return -EAGAIN;
	}

	ret = enable_irq_wake(bb->irq);
	if (ret) {
		dev_err(&pdev->dev, "set enable_irq_wake failed\n");
		kfree(bb);
		return -EAGAIN;
	}

	/* setup emc dvfs request framework */
	bb->emc_clk = clk_get(&pdev->dev, "emc_fl");
	if (IS_ERR(bb->emc_clk)) {
		dev_err(&pdev->dev, "can't get emc clock\n");
		kfree(bb);
		return -ENOENT;
	}

	bb->emc_min_freq = BBC_MC_MIN_FREQ;
	bb->emc_flags = EMC_DSR;

	/* get bbc proxy device struct, it should be registered
	 * before this driver.
	 */
	bb->proxy_dev = bus_find_device_by_name(&platform_bus_type,  NULL,
				"tegra_bbc_proxy");
	if (!bb->proxy_dev)
		dev_warn(&pdev->dev, "%s: bbc proxy device not found!\n",
				__func__);

	ret = request_irq(bb->mem_req_soon, tegra_bb_mem_req_soon,
			IRQF_TRIGGER_HIGH, "bb_mem_req_soon", bb);
	if (ret) {
		dev_err(&pdev->dev, "Could not register mem_req_soon irq\n");
		kfree(bb);
		return -EAGAIN;
	}
	tegra_bb_enable_mem_req_soon();

	ret = request_irq(INT_PMC_WAKE_INT, tegra_pmc_wake_intr,
			IRQF_TRIGGER_HIGH, "tegra_pmc_wake_intr", bb);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not register pmc_wake_int irq handler\n");
		kfree(bb);
		return -EAGAIN;
	}

	bb->pm_notifier.notifier_call = tegra_bb_pm_notifier_event;
	register_pm_notifier(&bb->pm_notifier);

	/* initially clear 2nd AP2BB irq */
	tegra_bb_clear_ap2bb_int1();
#endif
	bb->is_suspending = false;
	bb->send_ul_flag = false;

	dev_set_drvdata(&pdev->dev, bb);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_bb_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	tegra_bb_disable_pmc_wake();
	irq_set_irq_type(INT_PMC_WAKE_INT, IRQF_TRIGGER_RISING);

	return 0;
}

static int tegra_bb_resume(struct device *dev)
{
#ifndef CONFIG_TEGRA_BASEBAND_SIMU
	struct tegra_bb *bb = dev_get_drvdata(dev);
#endif
	dev_dbg(dev, "%s\n", __func__);

#ifndef CONFIG_TEGRA_BASEBAND_SIMU

	tegra_bb_enable_mem_req_soon();
	irq_set_irq_type(bb->mem_req_soon, IRQF_TRIGGER_HIGH);

	/* clear the wake mask to avoid non mem_req related interrupts */
	pmc_32kwritel(0, PMC_WAKE_MASK);
	pmc_32kwritel(0, PMC_WAKE2_MASK);
	irq_set_irq_type(INT_PMC_WAKE_INT, IRQF_TRIGGER_HIGH);
	tegra_bb_enable_pmc_wake(MEM_REQ_DET_LOW);
#endif
	return 0;
}

static int tegra_bb_suspend_noirq(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	/* abort suspend if IPC interrupt is pending*/
	if (tegra_bb_check_bb2ap_ipc())
		return -EBUSY;
	return 0;
}

static int tegra_bb_resume_noirq(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops tegra_bb_pm_ops = {
	.suspend_noirq = tegra_bb_suspend_noirq,
	.resume_noirq = tegra_bb_resume_noirq,
	.suspend = tegra_bb_suspend,
	.resume = tegra_bb_resume,
};
#endif

static struct platform_driver tegra_bb_driver = {
	.driver = {
		.name = "tegra_bb",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &tegra_bb_pm_ops,
#endif
	},
	.probe = tegra_bb_probe,
};

static int __init tegra_bb_init(void)
{
	int ret;
	ret = platform_driver_register(&tegra_bb_driver);
	pr_debug("%s ret %d\n", __func__, ret);
	return ret;
}

static void __exit tegra_bb_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&tegra_bb_driver);
}

fs_initcall(tegra_bb_init);
module_exit(tegra_bb_exit);

#ifdef CONFIG_DEBUG_FS
static struct dentry *bb_debugfs_root;
static enum pmc_event_sel {
	PMC_EVENT_NONE,
	PMC_EVENT_LP0BB,
	PMC_EVENT_LP0ACTIVE,
} pmc_event_sel;

static int lp0bb_transitions_set(void *data, u64 val)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 reg;

	if (!val) {
		/* disable event counting and clear counter */
		reg = 0;
		writel(reg, pmc + PMC_EVENT_COUNTER_0);
		pmc_event_sel = PMC_EVENT_NONE;
	} else if (val == 1) {
		reg = PMC_EVENT_COUNTER_0_EN_MASK | PMC_EVENT_COUNTER_0_LP0BB;
		 /* lp0->lp0bb transitions */
		writel(reg, pmc + PMC_EVENT_COUNTER_0);
		pmc_event_sel = PMC_EVENT_LP0BB;
	}
	return 0;
}

static inline unsigned long read_pmc_event_counter(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 reg = readl(pmc + PMC_EVENT_COUNTER_0);
	/* hw event counter is 16 bit */
	reg = reg & 0xffff;
	return reg;
}

static int lp0bb_transitions_get(void *data, u64 *val)
{
	if (pmc_event_sel == PMC_EVENT_LP0BB)
		*val = (u32) read_pmc_event_counter();
	else
		*val = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(lp0bb_fops, lp0bb_transitions_get,
		lp0bb_transitions_set, "%lld\n");

static int lp0active_transitions_set(void *data, u64 val)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 reg;

	if (!val) {
		/* disable event counting and clear counter */
		reg = 0;
		writel(reg, pmc + PMC_EVENT_COUNTER_0);
		pmc_event_sel = PMC_EVENT_NONE;
	} else if (val == 1) {
		reg = PMC_EVENT_COUNTER_0_EN_MASK |
				PMC_EVENT_COUNTER_0_LP0ACTIVE;
		 /* lp0->active transitions */
		writel(reg, pmc + PMC_EVENT_COUNTER_0);
		pmc_event_sel = PMC_EVENT_LP0ACTIVE;
	}
	return 0;
}
static int lp0active_transitions_get(void *data, u64 *val)
{
	if (pmc_event_sel == PMC_EVENT_LP0ACTIVE)
		*val = (u32) read_pmc_event_counter();
	else
		*val = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(lp0active_fops, lp0active_transitions_get,
		lp0active_transitions_set, "%lld\n");

int bb_emc_set_stats_show(struct seq_file *s, void *data)
{
	int bin, total;

	seq_printf(s, "\n");
	seq_printf(s, "BBC_SET_FLOOR delay    count\n");
	seq_printf(s, "-------------------------------\n");
	for (bin = 0; bin < (MAX_SMALL_STAT_TIME/SMALL_STAT_STEP); bin++) {
		if (bb_emc_set_s_stats[bin] == 0)
			continue;
		seq_printf(s, "%6u - %6u ms: %8u\n",
			bin * SMALL_STAT_STEP, (bin+1) * SMALL_STAT_STEP - 1,
			bb_emc_set_s_stats[bin]);
	}
	seq_printf(s, "-------------------------------\n");
	total = 0;
	for (bin = 0; bin < 32; bin++) {
		if (bb_emc_set_l_stats[bin] == 0)
			continue;
		seq_printf(s, "%6u - %6u ms: %8u\n",
			1 << (bin - 1), 1 << bin,
			bb_emc_set_l_stats[bin]);
		total += bb_emc_set_l_stats[bin];
	}
	seq_printf(s, "-------------------------------\n");
	seq_printf(s, "%18s: %8u ms\n", "max latency", max_emc_set_latency);
	seq_printf(s, "-------------------------------\n");
	seq_printf(s, "%18s: %8u\n", "TOTAL", total);
	seq_printf(s, "\n");

    return 0;
}

#ifdef CONFIG_DEBUG_FS
static void dump_emc_set_stats(void)
{
	int bin, total;

	pr_info("\n");
	pr_info("BBC_SET_FLOOR delay    count\n");
	pr_info("-------------------------------\n");
	for (bin = 0; bin < (MAX_SMALL_STAT_TIME/SMALL_STAT_STEP); bin++) {
		if (bb_emc_set_s_stats[bin] == 0)
			continue;
		pr_info("%6u - %6u ms: %8u\n",
			bin * SMALL_STAT_STEP, (bin+1) * SMALL_STAT_STEP - 1,
			bb_emc_set_s_stats[bin]);
	}
	pr_info("-------------------------------\n");
	total = 0;
	for (bin = 0; bin < 32; bin++) {
		if (bb_emc_set_l_stats[bin] == 0)
			continue;
		pr_info("%6u - %6u ms: %8u\n",
			1 << (bin - 1), 1 << bin,
			bb_emc_set_l_stats[bin]);
		total += bb_emc_set_l_stats[bin];
	}
	pr_info("-------------------------------\n");
	pr_info("%18s: %8u ms\n", "max latency", max_emc_set_latency);
	pr_info("-------------------------------\n");
	pr_info("%18s: %8u\n", "TOTAL", total);
	pr_info("\n");
}
#endif
static int bb_emc_set_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, bb_emc_set_stats_show,
				inode->i_private);
}

static const struct file_operations bb_emc_set_stats_fops = {
	.open	= bb_emc_set_stats_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static int __init tegra_bb_debug_init(void)
{
	struct dentry *d;

	bb_debugfs_root = debugfs_create_dir("tegra_bb", NULL);
	if (!bb_debugfs_root)
		return -ENOMEM;

	d = debugfs_create_file("lp0bb_transitions", S_IWUSR | S_IRUGO,
			bb_debugfs_root, NULL, &lp0bb_fops);
	if (!d)
		goto err;

	d = debugfs_create_file("lp0active_transitions", S_IWUSR | S_IRUGO,
			bb_debugfs_root, NULL, &lp0active_fops);
	if (!d)
		goto err;

	d = debugfs_create_file("bb_emc_set_stats", S_IRUGO,
			bb_debugfs_root, NULL, &bb_emc_set_stats_fops);
	if (!d)
		goto err;

	return 0;
err:
	debugfs_remove_recursive(bb_debugfs_root);
	return -ENOMEM;
}

late_initcall(tegra_bb_debug_init);
#endif

MODULE_DESCRIPTION("Tegra T148 BB Module");
MODULE_LICENSE("GPL");
