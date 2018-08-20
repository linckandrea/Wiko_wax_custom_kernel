/*
* imx179_otp.c - imx179 sensor otp driver
*
* Copyright (c) 2012-2013 NVIDIA Corporation. All Rights Reserved.
*
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/
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

#include "imx179_otp.h"
u8 MID_Sunny = 0x01;
u8 LensID_Sunny = 0x03;
u8 VcmID_Sunny = 0;

u16 RoverG_dec_base = 0x249;	/*R/G_Typical eg */
u16 BoverG_dec_base = 0x232;	/*B/G_Typical eg */

int ReadOTPIMX179(struct imx179_info *info);
int IMX179_ReadWbFromOtp(struct imx179_info *info);
int IMX179_ReadOtp(struct imx179_info *info, u16 tempbank, u16 address,
		   u8 * iBuffer, u16 buffersize);
void updateAWBIMX179(struct imx179_info *info, u16 RoverG_dec, u16 BoverG_dec,
		     u16 GboverGr_dec);
int IMX179MIPI_read_cmos_sensor(struct imx179_info *info, u16 reg);
int IMX179MIPI_write_cmos_sensor(struct imx179_info *info, u16 reg, u8 val);

/**************************** otp start ************************/
int ReadOTPIMX179(struct imx179_info *info)
{
	IMX179MIPI_write_cmos_sensor(info, 0x0100, 0x00);/* stream off */
	if (false == IMX179_ReadWbFromOtp(info)) {
		pr_err("[IMX179_OTP] update_otp fail!\n");
		return 0;
	}
	IMX179MIPI_write_cmos_sensor(info, 0x0100, 0x01);/* stream on */

	return 1;
}

int IMX179_ReadWbFromOtp(struct imx179_info *info)
{
	u16 awbGroupbank[] = { 3, 2, 1 };
	u16 address = 0x3404;

	u16 r, g, b;
	int i;
	u8 Temp[42] = { 0 };
	u8 temp3;
	int index = -1;
	int tempbank = 0;

	for (i = 0; i < 3; i++)
	{
		IMX179MIPI_write_cmos_sensor(info, 0x3380, 0x00);	//ECC OFF
		IMX179MIPI_write_cmos_sensor(info, 0x3400, 0x01);	//enable RW
		IMX179MIPI_write_cmos_sensor(info, 0x3402, awbGroupbank[i]);	// pages address
		temp3 = IMX179MIPI_read_cmos_sensor(info, 0x3404);	//0x3404 ~0x3443 64bytes pages map
		if (temp3 == 1)
		{
			index = i;
			break;
		}
	}
	if (index == -1) {
		return false;
	}
	tempbank = awbGroupbank[index];

	IMX179_ReadOtp(info, tempbank, address, Temp, 42);

	if (Temp[6] != MID_Sunny || Temp[7] != LensID_Sunny || Temp[8] != VcmID_Sunny)
	{
		return false;
	}
	r = (Temp[15] << 8) | Temp[16];
	b = (Temp[17] << 8) | Temp[18];
	g = (Temp[19] << 8) | Temp[20];

	updateAWBIMX179(info, r, b, g);

	return true;
}

int IMX179_ReadAfmacroFromOtp(struct imx179_info *info)
{
	u16 awbGroupbank[] = { 6, 5, 4 };
	u16 address = 0x3404;

	int i;
	u8 Temp[7] = { 0 };
	u8 temp3;
	int index = -1;
	int tempbank = 0;
	int af_macro = 0;

	for (i = 0; i < 3; i++)
	{
		IMX179MIPI_write_cmos_sensor(info, 0x3380, 0x00);	//ECC OFF
		IMX179MIPI_write_cmos_sensor(info, 0x3400, 0x01);	//enable RW
		IMX179MIPI_write_cmos_sensor(info, 0x3402, awbGroupbank[i]);	// pages address
		temp3 = IMX179MIPI_read_cmos_sensor(info, 0x3404);	//0x3404 ~0x3443 64bytes pages map
		if (temp3 != 0)
		{
			index = i;
			break;
		}
	}
	if (index == -1) {
		return false;
	}
	tempbank = awbGroupbank[index];

	IMX179_ReadOtp(info, tempbank, address, Temp, 7);

	af_macro = Temp[3] << 8 | Temp[4];

	return af_macro;
}

int IMX179_ReadAfinfFromOtp(struct imx179_info *info)
{
	u16 awbGroupbank[] = { 9, 8, 7 };
	u16 address = 0x3404;

	int i;
	u8 Temp[7] = { 0 };
	u8 temp3;
	int index = -1;
	int tempbank = 0;
	int af_inf = 0;

	for (i = 0; i < 3; i++)
	{
		IMX179MIPI_write_cmos_sensor(info, 0x3380, 0x00);	//ECC OFF
		IMX179MIPI_write_cmos_sensor(info, 0x3400, 0x01);	//enable RW
		IMX179MIPI_write_cmos_sensor(info, 0x3402, awbGroupbank[i]);	// pages address
		temp3 = IMX179MIPI_read_cmos_sensor(info, 0x3404);	//0x3404 ~0x3443 64bytes pages map
		if (temp3 != 0)
		{
			index = i;
			break;
		}
	}
	if (index == -1) {
		return false;
	}
	tempbank = awbGroupbank[index];

	IMX179_ReadOtp(info, tempbank, address, Temp, 7);

	af_inf = Temp[3] << 8 | Temp[4];

	return af_inf;
}

//read otp data by the given page
//tempbank:the first group for read
//address:the first address for read
//iBuffer:the return data buffer
//buffersize: buffer size
int IMX179_ReadOtp(struct imx179_info *info, u16 tempbank, u16 address,
		   		   u8 * iBuffer, u16 buffersize)
{
	int i;
	u8 reVal;
	IMX179MIPI_write_cmos_sensor(info, 0x3380, 0x00);
	IMX179MIPI_write_cmos_sensor(info, 0x3400, 0x01);
	IMX179MIPI_write_cmos_sensor(info, 0x3402, tempbank);

	reVal = IMX179MIPI_read_cmos_sensor(info, 0x3401);
	if (reVal)
		pr_err("[IMX179_OTP] KERN_ERR otp is not ready\n");
	for (i = 0; i < buffersize; i++) {
		reVal = IMX179MIPI_read_cmos_sensor(info, address + i);
		*(iBuffer + i) = reVal;
	}
	return true;
}

void updateAWBIMX179(struct imx179_info *info, u16 RoverG_dec, u16 BoverG_dec,
		     u16 GboverGr_dec)
{

	u16 R_test, B_test, G_test;
	u16 R_test_H3, R_test_L8, B_test_H3, B_test_L8, G_test_H3, G_test_L8;
	u16 G_test_R, G_test_B;
	if (BoverG_dec < BoverG_dec_base) {
		if (RoverG_dec < RoverG_dec_base) {
			G_test = 0x100;
			B_test = 0x100 * BoverG_dec_base / BoverG_dec;
			R_test = 0x100 * RoverG_dec_base / RoverG_dec;
		} else {
			R_test = 0x100;
			G_test = 0x100 * RoverG_dec / RoverG_dec_base;
			B_test = G_test * BoverG_dec_base / BoverG_dec;
		}
	} else {
		if (RoverG_dec < RoverG_dec_base) {
			B_test = 0x100;
			G_test = 0x100 * BoverG_dec / BoverG_dec_base;
			R_test = G_test * RoverG_dec_base / RoverG_dec;
		} else {
			G_test_B = 0x100 * BoverG_dec / BoverG_dec_base;
			G_test_R = 0x100 * RoverG_dec / RoverG_dec_base;
			if (G_test_B > G_test_R) {
				B_test = 0x100;
				G_test = G_test_B;
				R_test = G_test * RoverG_dec_base / RoverG_dec;
			} else {
				R_test = 0x100;
				G_test = G_test_R;
				B_test = G_test * BoverG_dec_base / BoverG_dec;
			}
		}
	}
	if (R_test < 0x100)
		R_test = 0x100;
	if (G_test < 0x100)
		G_test = 0x100;
	if (B_test < 0x100)
		B_test = 0x100;

	R_test_H3 = (R_test >> 8) & 0x0F;
	R_test_L8 = R_test & 0xFF;
	B_test_H3 = (B_test >> 8) & 0x0F;
	B_test_L8 = B_test & 0xFF;
	G_test_H3 = (G_test >> 8) & 0x0F;
	G_test_L8 = G_test & 0xFF;

	/* reset the digital gain */
	IMX179MIPI_write_cmos_sensor(info, 0x020F, G_test_L8);

	IMX179MIPI_write_cmos_sensor(info, 0x0211, R_test_L8);

	IMX179MIPI_write_cmos_sensor(info, 0x0213, B_test_L8);

	IMX179MIPI_write_cmos_sensor(info, 0x0215, G_test_L8);
}

#if defined(CONFIG_MACH_S9321) && CONFIG_MACH_S9321
/* for s9321 */
int IMX179_ReadFuseIDFromOTP(struct imx179_info *info)
{
	IMX179_ReadOtp(info,
		1,
		0x3404,
		info->fuse_id.data,
		info->fuse_id.size);
	return 0;
}
#else
/* for non s9321 */
int IMX179_ReadFuseIDFromOTP(struct imx179_info *info)
{
	u16 awbGroupbank[] = { 3, 2, 1 };
	u16 address = 0x3404;
	int i;
	u8 temp3;
	int index = -1;
	int tempbank = 0;

	for (i = 0; i < 3; i++)
	{
		IMX179MIPI_write_cmos_sensor(info, 0x3380, 0x00);	//ECC OFF
		IMX179MIPI_write_cmos_sensor(info, 0x3400, 0x01);	//enable RW
		IMX179MIPI_write_cmos_sensor(info, 0x3402, awbGroupbank[i]);	// pages address
		temp3 = IMX179MIPI_read_cmos_sensor(info, 0x3404);	//0x3404 ~0x3443 64bytes pages map
		if (temp3 != 0)
		{
			index = i;
			break;
		}
	}
	if (index == -1) {
		return false;
	}
	tempbank = awbGroupbank[index];

	IMX179_ReadOtp(info, tempbank, address, info->fuse_id.data, info->fuse_id.size);

	return 0;
}
#endif
/**************************** otp end ****************************/
