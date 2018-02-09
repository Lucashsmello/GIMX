/*
 * adv-mouse2axis.h
 *
 *  Created on: 9 de fev de 2018
 *      Author: lmello
 */

#ifndef ADV_MOUSE2AXIS_H_
#define ADV_MOUSE2AXIS_H_

#include <adapter.h>

typedef enum {
	false, true
} boolean;

typedef struct {
	boolean preserve_angle;
	boolean motion_residue_extrapolation; // if true, accepts too low or too high motion_residues.
	boolean zero_axis_is_positive;
} mouse2axis_config;

double adv_mouse2axis(s_adapter* controller, int which, double x, double y, s_axis_props* axis_props,
		double multiplier, int dead_zone, const mouse2axis_config* m2a_config);

#endif /* ADV_MOUSE2AXIS_H_ */
