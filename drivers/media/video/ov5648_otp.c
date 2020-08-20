#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "ov5648_otp.h"

int OV5648MIPI_write_cmos_sensor(struct ov5648_info *info, u16 reg, u8 val);
int OV5648MIPI_read_cmos_sensor(struct ov5648_info *info, u16 reg);

#define OTP_DATA_ADDR         0x3D00
#define OTP_LOAD_ADDR         0x3D81

#define OTP_WB_GROUP_ADDR     0x3D05
#define OTP_WB_GROUP_SIZE     9
#define OTP_BANK_ADDR         0x3D84
#define OTP_BANK              0x3D85
#define OTP_END_ADDR          0x3D86

#define GAIN_RH_ADDR          0x5186
#define GAIN_RL_ADDR          0x5187
#define GAIN_GH_ADDR          0x5188
#define GAIN_GL_ADDR          0x5189
#define GAIN_BH_ADDR          0x518A
#define GAIN_BL_ADDR          0x518B

#define GAIN_DEFAULT_VALUE    0x0400	/* 1x gain */

#define OTP_MID               0x02

/*R/G and B/G of current camera module*/
static unsigned char RG_MSB;
static unsigned char BG_MSB;
static unsigned char AWB_LSB;

/*Enable OTP read function*/
static void otp_read_enable(struct ov5648_info *info)
{
	OV5648MIPI_write_cmos_sensor(info, OTP_LOAD_ADDR, 0x01);
	mdelay(15);		/* sleep > 10ms */
}

/*Disable OTP read function*/
static void otp_read_disable(struct ov5648_info *info)
{
	OV5648MIPI_write_cmos_sensor(info, OTP_LOAD_ADDR, 0x00);
}

static void otp_read(struct ov5648_info *info, unsigned short otp_addr,
		     unsigned char *otp_data)
{
	otp_read_enable(info);
	*otp_data = OV5648MIPI_read_cmos_sensor(info, otp_addr);
	otp_read_disable(info);
}

/*******************************************************************************
* Function    :  otp_clear
* Description :  Clear OTP buffer
* Parameters  :  none
* Return      :  none
*******************************************************************************/
static void otp_clear(struct ov5648_info *info)
{
/*        After read/write operation, the OTP buffer should be cleared to avoid accident write*/
	unsigned char i;
	for (i = 0; i < 16; i++) {
		OV5648MIPI_write_cmos_sensor(info, OTP_DATA_ADDR + i, 0x00);
	}
}

/*******************************************************************************
* Function    :  otp_check_wb_group
* Description :  Check OTP Space Availability
* Parameters  :  [in] index : index of otp group (0, 1, 2)
* Return      :  0, group index is empty
* 1, group index has invalid data
* 2, group index has valid data
* -1, group index error
*******************************************************************************/
static signed char otp_check_wb_group(struct ov5648_info *info,
				      unsigned char index)
{
	unsigned short otp_addr = OTP_WB_GROUP_ADDR + index * OTP_WB_GROUP_SIZE;
	unsigned char flag;

	if (index > 2) {
		printk("[OV5648_OTP] OTP input wb group index %d error\n",
		       index);
		return -1;
	} else if (index < 2) {
/*                select bank 0*/
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK_ADDR, 0xc0);
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK, 0x00);
		OV5648MIPI_write_cmos_sensor(info, OTP_END_ADDR, 0x0f);
	} else {		/*index==2 */

/*                select bank 1*/
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK_ADDR, 0xc0);
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK, 0x10);
		OV5648MIPI_write_cmos_sensor(info, OTP_END_ADDR, 0x1f);
		otp_addr = 0x3D07;
	}

	otp_read(info, otp_addr, &flag);
	otp_clear(info);

/*        Check all bytes of a group. If all bytes are '0', then the group is empty. */
/*         Check from group 1 to group 2, then group 3.*/
	printk("[OV5648_OTP] flag %x\n", flag);

	if (!flag) {
		printk("[OV5648_OTP] wb group %d is empty\n", index);
		return 0;
	}
/*        else if ((!(flag&0x80)) && ((flag&0x7f) == OTP_MID))*/
/*        else if(flag == OTP_MID)*/
	else if (flag == 0x02) {
		printk("[OV5648_OTP] wb group %d has valid data\n", index);
		return 2;
	} else {
		printk("[OV5648_OTP] wb group %d has invalid data\n", index);
		return 1;
	}
}

/*******************************************************************************
* Function    :  otp_read_wb_group
* Description :  Read group value and store it in OTP Struct
* Parameters  :  [in] index : index of otp group (0, 1, 2)
* Return      :  group index (0, 1, 2)
*                -1, error
*******************************************************************************/
static signed char otp_read_wb_group(struct ov5648_info *info,
				     signed char index)
{
	unsigned short awb_addr;
/*        unsigned char  mid;*/

	if (index == -1) {
/*                Check first OTP with valid data*/
		for (index = 0; index < 3; index++) {
			if (otp_check_wb_group(info, index) == 2) {
				printk("[OV5648_OTP] read wb from group %d",
				       index);
				break;
			}
		}

		if (index > 2) {
			printk("[OV5648_OTP] no group has valid data\n");
			return -1;
		}
	} else {
		if (otp_check_wb_group(info, index) != 2) {
			printk("[OV5648_OTP] read wb from group %d failed\n",
			       index);
			return -1;
		}
	}

	if (index == 0) {
/*                select bank 0*/
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK_ADDR, 0xc0);
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK, 0x00);
		OV5648MIPI_write_cmos_sensor(info, OTP_END_ADDR, 0x0f);
		otp_read(info, OTP_WB_GROUP_ADDR + 2, &RG_MSB);
		otp_read(info, OTP_WB_GROUP_ADDR + 3, &BG_MSB);
		otp_read(info, OTP_WB_GROUP_ADDR + 6, &AWB_LSB);
	} else {		/* (index == 1 || index ==2) */

		awb_addr = OTP_DATA_ADDR + (index - 1) * OTP_WB_GROUP_SIZE;
/*                select bank 1*/
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK_ADDR, 0xc0);
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK, 0x10);
		OV5648MIPI_write_cmos_sensor(info, OTP_END_ADDR, 0x1f);
		otp_read(info, awb_addr, &RG_MSB);
		otp_read(info, awb_addr + 1, &BG_MSB);
		otp_read(info, awb_addr + 4, &AWB_LSB);
	}
	printk("[OV5648_OTP] RG_MSB = 0x%x BG_MSB =0x%x AWB_LSB =0x%x\n",
	       RG_MSB, BG_MSB, AWB_LSB);

	otp_clear(info);

	printk("[OV5648_OTP] read wb finished\n");
	return index;
}

#define OTP_MULTIPLE_FAC	10000
static int otp_apply_wb(struct ov5648_info *info, unsigned short golden_rg,
			unsigned short golden_bg)
{
	unsigned long gain_r = GAIN_DEFAULT_VALUE;
	unsigned long gain_g = GAIN_DEFAULT_VALUE;
	unsigned long gain_b = GAIN_DEFAULT_VALUE;
	unsigned long rg_ratio = (RG_MSB << 2) | ((AWB_LSB & 0xC0) >> 6);
	unsigned long bg_ratio = (BG_MSB << 2) | ((AWB_LSB & 0x30) >> 4);
	unsigned long ratio_r, ratio_g, ratio_b;
	unsigned long cmp_rg, cmp_bg;
	printk("[OV5648_OTP] rg_ratio = 0x%x bg_ratio =0x%x \n", rg_ratio,
	       bg_ratio);

	if (!golden_rg || !golden_bg) {
		printk("[OV5648_OTP] golden_rg / golden_bg can not be zero\n");
		return 0;
	}
/*        Calcualte R, G, B gain of current module from R/G, B/G of golden module*/
/*        and R/G, B/G of current module*/

	cmp_rg = OTP_MULTIPLE_FAC * rg_ratio / golden_rg;
	cmp_bg = OTP_MULTIPLE_FAC * bg_ratio / golden_bg;

	if ((cmp_rg < 1 * OTP_MULTIPLE_FAC) && (cmp_bg < 1 * OTP_MULTIPLE_FAC)) {
/*                R/G < R/G golden, B/G < B/G golden*/
		ratio_g = 1 * OTP_MULTIPLE_FAC;
		ratio_r = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_rg;
		ratio_b = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_bg;
	} else if (cmp_rg > cmp_bg) {
/*                R/G >= R/G golden, B/G < B/G golden*/
/*                R/G >= R/G golden, B/G >= B/G golden*/
		ratio_r = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_rg;
		ratio_b = OTP_MULTIPLE_FAC * cmp_rg / cmp_bg;
	} else {
/*                B/G >= B/G golden, R/G < R/G golden*/
/*                B/G >= B/G golden, R/G >= R/G golden*/
		ratio_b = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_bg;
		ratio_r = OTP_MULTIPLE_FAC * cmp_bg / cmp_rg;
	}

/*        write sensor wb gain to registers*/
/*        0x0400 = 1x gain*/
	if (ratio_r != 1 * OTP_MULTIPLE_FAC) {
		gain_r = GAIN_DEFAULT_VALUE * ratio_r / OTP_MULTIPLE_FAC;
		OV5648MIPI_write_cmos_sensor(info, GAIN_RH_ADDR, gain_r >> 8);
		OV5648MIPI_write_cmos_sensor(info, GAIN_RL_ADDR,
					     gain_r & 0x00ff);
	}

	if (ratio_g != 1 * OTP_MULTIPLE_FAC) {
		gain_g = GAIN_DEFAULT_VALUE * ratio_g / OTP_MULTIPLE_FAC;
		OV5648MIPI_write_cmos_sensor(info, GAIN_GH_ADDR, gain_g >> 8);
		OV5648MIPI_write_cmos_sensor(info, GAIN_GL_ADDR,
					     gain_g & 0x00ff);
	}

	if (ratio_b != 1 * OTP_MULTIPLE_FAC) {
		gain_b = GAIN_DEFAULT_VALUE * ratio_b / OTP_MULTIPLE_FAC;
		OV5648MIPI_write_cmos_sensor(info, GAIN_BH_ADDR, gain_b >> 8);
		OV5648MIPI_write_cmos_sensor(info, GAIN_BL_ADDR,
					     gain_b & 0x00ff);
	}

	return 1;
}

/*******************************************************************************
* Function    :  otp_update_wb
* Description :  Update white balance settings from OTP
* Parameters  :  [in] golden_rg : R/G of golden camera module
*                [in] golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/
int update_truly_otp_wb(struct ov5648_info *info, unsigned short golden_rg,
			unsigned short golden_bg)
{
	printk("[OV5648_OTP] start wb update\n");

	if (otp_read_wb_group(info, -1) != -1) {
		if (otp_apply_wb(info, golden_rg, golden_bg) == 1) {
			printk("[OV5648_OTP] wb update finished\n");
			return 1;
		}
	}

	printk("[OV5648_OTP] wb update failed\n");
	return 0;
}

/*******************************************************************************
* Function    :  check_truly_otp_mid
* Description :  Check modules id from OTP.
* Return      :  1, success; 0, fail
*******************************************************************************/
int check_truly_otp_mid(struct ov5648_info *info)
{
	uint32_t address, index, i;
	u8 mid = 0x00;

	OV5648MIPI_write_cmos_sensor(info, 0x0100, 0x01);
	mdelay(50);
	for (index = 0; index < 3; index++) {
		if (index < 2) {
/*                        read otp --Bank 0*/
			OV5648MIPI_write_cmos_sensor(info, 0x3d84, 0xc0);
			OV5648MIPI_write_cmos_sensor(info, 0x3d85, 0x00);
			OV5648MIPI_write_cmos_sensor(info, 0x3d86, 0x0f);
			OV5648MIPI_write_cmos_sensor(info, 0x3d81, 0x01);
			mdelay(15);
			address = 0x3d05 + index * 9;
		} else {
/*                        read otp --Bank 1*/
			OV5648MIPI_write_cmos_sensor(info, 0x3d84, 0xc0);
			OV5648MIPI_write_cmos_sensor(info, 0x3d85, 0x10);
			OV5648MIPI_write_cmos_sensor(info, 0x3d86, 0x1f);
			OV5648MIPI_write_cmos_sensor(info, 0x3d81, 0x01);
			mdelay(15);
			address = 0x3d05 + index * 9 - 16;
		}

		mid = OV5648MIPI_read_cmos_sensor(info, address);
/*                mid &= 0x7f;*/
		printk
		    ("[OV5648_OTP] [check_truly_otp_mid] MID == %d, index == %d\n",
		     mid, index);
		if (0x02 == mid) {
			printk
			    ("[OV5648_OTP] [check_truly_otp_mid] This is truly module.\n");
			break;
		}
	}

	OV5648MIPI_write_cmos_sensor(info, 0x3d81, 0x00);
/*        clear otp buffer*/
	address = 0x3d00;
	for (i = 0; i < 16; i++) {
		OV5648MIPI_write_cmos_sensor(info, address + i, 0x00);
	}

	printk("[OV5648_OTP] [check_truly_otp_mid] index==%d\n", index);
	if (index > 2)
		return 0;
	else
		return 1;
}

u32 OV5648_ReadFuseIDFromOTP(struct ov5648_info *info)
{
	u32 fuse_id = 0;
	int i, index;
	u8 Temp[4] = { 0 };

	OV5648MIPI_write_cmos_sensor(info, 0x0100, 0x01);
	mdelay(15);

/*        Check first OTP with valid data*/
	for (index = 0; index < 3; index++) {
		if (otp_check_wb_group(info, index) == 2) {
			printk
			    ("[OV5648_OTP] [OV5648_ReadFuseIDFromOTP]read wb from group %d",
			     index);
			break;
		}
	}
	if (index > 2) {
		printk
		    ("[OV5648_OTP] [OV5648_ReadFuseIDFromOTP]no group has valid data\n");
		return -1;
	}
	if (index == 0) {
/*                select bank 0*/
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK_ADDR, 0xc0);
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK, 0x00);
		OV5648MIPI_write_cmos_sensor(info, OTP_END_ADDR, 0x0f);
	} else {		/* (index == 1 || index ==2) */

		OV5648MIPI_write_cmos_sensor(info, OTP_BANK_ADDR, 0xc0);
		OV5648MIPI_write_cmos_sensor(info, OTP_BANK, 0x10);
		OV5648MIPI_write_cmos_sensor(info, OTP_END_ADDR, 0x1f);
	}

	OV5648MIPI_write_cmos_sensor(info, 0x3d81, 0x01);
	mdelay(15);
	for (i = 0; i < 4; i++) {
		Temp[i] = OV5648MIPI_read_cmos_sensor(info, OTP_DATA_ADDR + i);
		printk
		    ("[OV5648_OTP] [OV5648_ReadFuseIDFromOTP]mingji test Temp[%d] == 0x%x\n",
		     i, Temp[i]);
	}
	OV5648MIPI_write_cmos_sensor(info, 0x3d81, 0x00);
	otp_clear(info);

	fuse_id =
	    (u32) (Temp[0] << 24) | (u32) (Temp[1] << 16) | (u32) (Temp[2] << 8)
	    | (u32) (Temp[3]);
	printk
	    ("[OV5648_OTP] [OV5648_ReadFuseIDFromOTP]mingji test fuse_id == 0x%x\n",
	     fuse_id);

	return fuse_id;
}

/**********************************OTP END**************************************/
