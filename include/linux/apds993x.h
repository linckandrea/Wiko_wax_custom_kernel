/*
 * Definitions for akm8963 compass chip.
 */
#ifndef APDS993X_H
#define APDS993X_H

struct apds993x_platform_data {
	int ps_pulse_number;
	int ps_detect_threshold;
	int ps_hysteresis_threshold;
	int als_hysteresis_threshold;
	int irq;
};

#endif

