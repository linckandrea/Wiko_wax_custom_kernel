/*
 * Copyright (c) 2011-2013, NVIDIA CORPORATION. All Rights Reserved.
 *
 */

#ifndef __OV5648_H__
#define __OV5648_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <media/nvc.h>
#include <media/nvc_image.h>
#include <linux/edp.h>

#define OV5648_IOCTL_SET_MODE           _IOW('o', 1, struct ov5648_mode)
#define OV5648_IOCTL_SET_FRAME_LENGTH   _IOW('o', 2, __u32)
#define OV5648_IOCTL_SET_COARSE_TIME    _IOW('o', 3, __u32)
#define OV5648_IOCTL_SET_GAIN           _IOW('o', 4, __u16)
#define OV5648_IOCTL_GET_STATUS         _IOR('o', 5, __u8)
#define OV5648_IOCTL_SET_GROUP_HOLD     _IOW('o', 6, struct ov5648_ae)
#define OV5648_IOCTL_GET_FUSEID         _IOR('o', 7, struct nvc_fuseid)

/* The enumeration must be in the order the regulators are to be enabled */
/* See Power Requirements note in the driver */
enum ov5648_vreg {
	OV5648_VREG_AVDD,
	OV5648_VREG_IOVDD,
	OV5648_VREG_AVDD_MIPI_SWITCH,
};

struct ov5648_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};

struct ov5648_ae {
	__u32 frame_length;
	__u8 frame_length_enable;
	__u32 coarse_time;
	__u8 coarse_time_enable;
	__s32 gain;
	__u8 gain_enable;
};

#ifdef __KERNEL__
struct ov5648_platform_data {
	const char *dev_name;
	int (*power_on)(struct nvc_regulator *);
	int (*power_off)(struct nvc_regulator *);
	struct edp_client edpc_config;
};
#endif /* __KERNEL__ */
#endif /* __OV5648_H__ */
