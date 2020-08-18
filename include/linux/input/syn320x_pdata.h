#ifndef _LINUX_SYN320X_PDATA_H
#define _LINUX_SYN320X_PDATA_H

struct syn320x_platform_data
{
	int reset_pin;		/* Reset pin is wired to this GPIO (optional) */
	int irq_pin;		/* IRQ pin is wired to this GPIO */
	int pdown_pin;
	int scl_pin;
	int sda_pin;
	unsigned short addr;
};
#endif
